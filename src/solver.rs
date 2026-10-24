//! Position solve from arrival times with a full time covariance.
//!
//! IMPLEMENTATION OWNER: agent "solver". Keep the public signatures.
//!
//! Model: t_i = t0 + |p - r_i| / C_AIR + e_i, with e ~ N(0, cov) (k x k, s^2,
//! NOT diagonal: arrival times share clock-correction errors). Unknowns p
//! (ECEF, m) and t0 (s, common timebase). Optional altitude pseudo-measurement
//! (height above WGS84, m, sigma m) as a soft constraint.
//!
//! Requirements:
//! - Whiten with the Cholesky factor of `cov` (fall back to the diagonal if not
//!   positive definite); Gauss-Newton / Levenberg-Marquardt with an analytic
//!   Jacobian; work in metres (multiply times by C_AIR) for conditioning; solve
//!   for p and t0 jointly. Altitude residual via geodetic height (geo.rs).
//! - Multi-start from every `guesses` entry (and the mirror image across the
//!   receivers' best-fit line/plane when there are <= 4 receivers, the classic
//!   two-lobe ambiguity); keep the lowest chi2. Clamp the initial altitude to
//!   [-500 m, 20 km].
//! - Robustness: after a converged fit with dof >= 1 and a chi2 above the
//!   99% quantile, try leave-one-out (when k >= 5) and keep the best fit whose
//!   chi2 passes; report which measurements were used.
//! - Reject: no convergence in 20 iterations; position outside [-500 m, 25 km]
//!   height; farther than 600 km from the nearest used receiver; t0 later than
//!   the earliest arrival; error ellipse 1-sigma horizontal > 20 km.
//! - Covariance: inverse of J^T W J for p (3x3, m^2); `err_h` = sqrt of the
//!   horizontal trace in the local ENU frame; `gdop` from the unweighted
//!   geometry.
//! - Must be deterministic and panic-free on degenerate input (collinear
//!   receivers, identical positions, NaN). Unit tests: synthetic geometries
//!   (3+alt, 4, 6, 12 receivers; noise from the given cov) recover the truth
//!   within the stated covariance (check the normalised error distribution);
//!   mirror case; outlier case; degenerate cases return None, not garbage.
//! - Performance: < 20 us for k = 8 on one core (no allocation per iteration
//!   beyond what nalgebra needs; DMatrix is fine for k <= 32).
//!
//! Implementation notes (the why behind the choices):
//!
//! - Whitening. With L L^T = C_AIR^2 cov, the whitened residuals L^-1 (rho -
//!   model) are iid N(0, 1), so generalised least squares with correlated
//!   errors becomes ordinary least squares and chi2 = |whitened residual|^2
//!   follows chi2(dof) at the truth. L^-1 is formed once per problem; each
//!   iteration whitens the k x 4 Jacobian/residual block in O(k^2).
//! - Joint t0. Estimating the transmit time as a fourth unknown is exactly
//!   equivalent to the correctly weighted TDOA solution (differences against
//!   any reference receiver with covariance D cov D^T), but the result does not
//!   depend on which receiver is the reference, the Jacobian rows are the
//!   simple [unit line-of-sight, 1], and a common-mode clock error (shared by
//!   every conversion through the same reference clock) is absorbed by t0
//!   instead of biasing p.
//! - Conditioning. Every position inside the solver is relative to the
//!   centroid of the used receivers and every time is relative to the earliest
//!   arrival, both in metres: all four unknowns and all Jacobian entries are
//!   then O(1) per metre and the 4 x 4 normal matrix is well scaled.
//! - Levenberg-Marquardt with Marquardt (diagonal) scaling and Nielsen's
//!   damping update; convergence is declared when the undamped Gauss-Newton
//!   step would lower chi2 by less than 1e-10, i.e. the remaining step is
//!   ~1e-5 sigma: statistically meaningless, independent of units.
//! - Algebraic starts. From a guess 100+ km away (a receiver position, the
//!   network centroid) Gauss-Newton crawls along the poorly determined
//!   range/height valley and often needs more than the 20-step budget, or
//!   falls into the below-ground lobe. So every search also starts from a
//!   closed-form solution: with the height fixed (the measured altitude, or
//!   10 km), the squared range equations are linear in the horizontal
//!   position and w = b^2 - |p|^2 for a given offset b, and the definition
//!   of w leaves a quadratic in b whose two roots are the two lobes (the
//!   Bancroft idea, reduced to the plane). It costs a 3 x 3 solve and lands
//!   within a few km, where Gauss-Newton converges in 3-5 steps.
//! - Mirror starts. With few receivers the residual surface has two lobes
//!   that fit almost equally well. For (near-)collinear receivers the ghost
//!   is the mirror image across the vertical plane through their best-fit
//!   line (same altitude, other side of the line); for coplanar receivers it
//!   is the reflection across their best-fit plane (the above/below
//!   ambiguity, present for any k because receivers on the Earth are always
//!   nearly coplanar). With <= 4 receivers every guess and every converged
//!   solution is reflected and re-solved; with more, only solutions that are
//!   implausible or fail the chi2 test are, keeping the common case cheap.
//!   The lowest chi2 among the valid fits wins; an exact tie (dof 0: both
//!   lobes fit perfectly) goes to the one nearer the first guess, usually the
//!   track's prediction, and when that guess is not at least 3x nearer one
//!   lobe the fix is refused as ambiguous rather than guessed.
//! - Outliers. Leave-one-out runs only after a converged fit fails the 99%
//!   chi2 test (a plausible-looking fit, or the best implausible one: an
//!   outlier can drag the fit underground), and only while the reduced fit
//!   keeps dof >= 1. Subsets are refitted from the full fit, and from scratch
//!   only if none of them then fits consistently. The drop is accepted only
//!   if it lowers chi2 by more than the Bonferroni-corrected 0.1% quantile of
//!   chi2(1) (Baarda's data snooping); otherwise the full fit is returned
//!   with its covariance scaled by chi2/dof.
//! - Plausibility. The t0 check allows 3 sigma of the earliest arrival's own
//!   noise (a taxiing aircraft is tens of metres from a receiver); the 20 km
//!   limit applies to the semi-major axis of the horizontal ellipse. Without
//!   altitude the height of a low aircraft is uncertain by km, and estimates
//!   falling below -500 m are rejected as specified: the engine should pass
//!   the Mode S altitude whenever it has one.

use nalgebra::{DMatrix, Matrix3, Matrix4, SymmetricEigen, Vector4};

use crate::geo::{self, Ecef, Llh, C_AIR};

mod chi2;
pub(crate) use chi2::{chi2_q99, chi2_q999, chi2_quantile};

#[cfg(test)]
pub(crate) mod testutil;
#[cfg(test)]
mod tests;

#[derive(Debug, Clone, Copy)]
pub struct Meas {
    /// Receiver antenna position, ECEF metres.
    pub rx: Ecef,
    /// Arrival time in the group's common timebase, seconds.
    pub t: f64,
}

pub struct SolveInput<'a> {
    pub meas: &'a [Meas],
    /// Covariance of the arrival times, seconds^2, k x k.
    pub cov: &'a DMatrix<f64>,
    /// (height above WGS84 m, 1-sigma m)
    pub alt: Option<(f64, f64)>,
    /// Starting points, ECEF metres; at least one.
    pub guesses: &'a [Ecef],
}

#[derive(Debug, Clone)]
pub struct Fix {
    pub pos: Ecef,
    /// Position covariance, ECEF, m^2.
    pub cov: Matrix3<f64>,
    /// Transmit time, common timebase, seconds.
    pub t0: f64,
    pub chi2: f64,
    /// measurements used + (1 if altitude) - 4
    pub dof: i32,
    /// 1-sigma horizontal error estimate, m.
    pub err_h: f64,
    pub gdop: f64,
    /// Per input measurement: kept in the final fit.
    pub used: Vec<bool>,
}

/// Iteration budget per start (each LM step, accepted or not, counts).
const MAX_ITER: usize = 20;
/// Converged when the full Gauss-Newton step would reduce chi2 by less.
const CONV_DECREMENT: f64 = 1e-10;
/// Starting points are clamped into this height band (m).
const GUESS_ALT: (f64, f64) = (-500.0, 20_000.0);
/// Accepted fixes must lie in this height band (m).
const FIX_ALT: (f64, f64) = (-500.0, 25_000.0);
/// Maximum distance from the nearest used receiver (m).
const MAX_RANGE: f64 = 600e3;
/// Maximum 1-sigma semi-major axis of the horizontal error ellipse (m).
const MAX_ELLIPSE: f64 = 20e3;
/// t0 may exceed the earliest arrival by at most this many sigmas of that
/// arrival's own timing noise: an aircraft parked next to a receiver has a
/// true range of tens of metres, well inside the noise, so a strict t0 <=
/// t_min would reject good ground fixes.
const T0_TOL_SIGMA: f64 = 3.0;
/// Height assumed by the algebraic starts when no altitude is measured: above
/// the receivers' plane (so Gauss-Newton starts on the right side of the
/// vertical ambiguity) at a typical en-route level.
const ALGEBRAIC_ALT: f64 = 10_000.0;
/// Mean Earth radius for the curvature term of the algebraic starts (m).
const EARTH_R: f64 = 6_371_000.0;
/// With two equally exact solutions, the reference point must be this many
/// times nearer the chosen one, or the fix is refused as ambiguous.
const AMBIGUITY_RATIO: f64 = 3.0;
/// Relative pivot below which a normal matrix counts as singular.
const SINGULAR_PIVOT: f64 = 1e-12;

/// Multilaterate one transmission. Returns None when the input is malformed
/// (NaN, wrong sizes, fewer than 4 measurements counting altitude, no finite
/// guess) or no fit passes the plausibility checks listed above.
///
/// When the best fit fails the 99% chi2 test and no leave-one-out subset
/// passes it, that fit is still returned (the caller sees `chi2` and `dof`),
/// but with its covariance inflated by chi2/dof (the Birge ratio) so that
/// `cov` and `err_h` describe the scatter the data actually show.
pub fn solve(inp: &SolveInput) -> Option<Fix> {
    let k = inp.meas.len();
    if k == 0 || inp.cov.nrows() != k || inp.cov.ncols() != k {
        return None;
    }
    if inp.meas.iter().any(|m| !m.t.is_finite() || !finite3(&m.rx)) || inp.cov.iter().any(|v| !v.is_finite()) {
        return None;
    }
    if let Some((h, s)) = inp.alt {
        if !h.is_finite() || !s.is_finite() || s <= 0.0 {
            return None;
        }
    }
    let n_alt = usize::from(inp.alt.is_some());
    if k + n_alt < 4 {
        return None;
    }
    // A guess near the geocentre has no meaningful latitude/longitude to clamp.
    let guesses: Vec<Ecef> = inp.guesses.iter().filter(|g| finite3(g) && norm3(g) > 1e6).copied().collect();
    let reference = *guesses.first()?;

    let all: Vec<usize> = (0..k).collect();
    let dof_full = (k + n_alt) as i32 - 4;
    let fails = |chi2: f64, dof: i32| dof >= 1 && chi2 > chi2_q99(dof);
    let search = Problem::new(inp, &all).map(|mut p| p.multistart(&guesses, &reference, true)).unwrap_or_default();
    let full = search.best;
    // The consistency of the full set is judged on its best converged fit,
    // plausible or not: an outlier can also drag the fit somewhere
    // implausible. A plausible-looking fit with a passing chi2 is final, and
    // so is a rejected one with a passing chi2 (e.g. a low aircraft without
    // altitude whose estimate falls below -500 m): dropping a good receiver
    // would not make that answer any more true.
    let full_chi2 = full.as_ref().map(|c| c.chi2).or(search.min.map(|m| m.0));
    let mut loo: Option<Cand> = None;
    if full_chi2.is_none_or(|x| fails(x, dof_full)) && k >= 5 && dof_full >= 2 {
        // Leave-one-out; only while the reduced fit keeps dof >= 1 (with dof
        // 0 every subset fits exactly and the choice would be arbitrary).
        // First a cheap pass started from the full fit (an outlier distorts
        // it but usually leaves it in the right basin); if that finds
        // nothing, a second pass from scratch (guesses, algebraic starts,
        // mirrors), for outliers big enough to have dragged the fit away.
        let mut passes: Vec<(Vec<Ecef>, bool)> = Vec::with_capacity(2);
        match (&full, search.min) {
            (Some(c), _) => passes.push((vec![c.pos], false)),
            (None, Some((_, p))) => passes.push((vec![p], false)),
            (None, None) => {}
        }
        passes.push((guesses.clone(), true));
        // Data snooping (Baarda's w-test): dropping the culprit lowers chi2
        // by its squared normalised residual, chi2(1) when it is in fact
        // fine. Accept the drop only beyond the Bonferroni-corrected 0.1%
        // level for k candidates (about 14 for k = 6, 17 for k = 30): real
        // outliers (multipath, clock glitches) are tens of sigma, while a 1%
        // chi2 false alarm on clean data would otherwise drop a good receiver
        // and leave a weaker, sometimes degenerate, geometry.
        let snoop = chi2_quantile(1.0 - 1e-3 / k as f64, 1);
        for (starts, scratch) in passes {
            // Did any subset fit consistently? If so but none significantly
            // better, the data simply have no outlier: no second pass.
            let mut consistent = false;
            for drop in 0..k {
                let idx: Vec<usize> = (0..k).filter(|&i| i != drop).collect();
                let Some(c) = Problem::new(inp, &idx).and_then(|mut p| p.multistart(&starts, &reference, scratch).best) else {
                    continue;
                };
                if fails(c.chi2, c.dof) {
                    continue;
                }
                consistent = true;
                let significant = full_chi2.is_none_or(|fc| fc - c.chi2 > snoop);
                if significant && loo.as_ref().is_none_or(|b| c.chi2 < b.chi2) {
                    loo = Some(c);
                }
            }
            if consistent {
                break;
            }
        }
    }
    let (best, inflate) = match (loo, full) {
        (Some(c), _) => (c, false),
        (None, Some(c)) => {
            let inflate = fails(c.chi2, c.dof);
            (c, inflate)
        }
        (None, None) => return None,
    };

    let mut cov = best.cov3;
    let mut err_h = best.err_h;
    if inflate {
        let s = best.chi2 / best.dof as f64;
        if best.major * s.sqrt() > MAX_ELLIPSE {
            return None;
        }
        cov *= s;
        err_h *= s.sqrt();
    }
    let mut used = vec![false; k];
    for &i in &best.idx {
        used[i] = true;
    }
    Some(Fix { pos: best.pos, cov, t0: best.t0, chi2: best.chi2, dof: best.dof, err_h, gdop: best.gdop, used })
}

/// One least-squares problem: a subset of the measurements, whitened.
struct Problem {
    /// Centroid of the used receivers, ECEF; all positions below are relative.
    origin: Ecef,
    /// Input index of each used measurement.
    idx: Vec<usize>,
    /// Receiver positions relative to `origin`.
    rx: Vec<[f64; 3]>,
    /// Pseudoranges C_AIR * (t_i - t_ref), metres; t_ref = earliest arrival.
    rho: Vec<f64>,
    t_ref: f64,
    /// 1-sigma of the earliest arrival, metres (t0 plausibility tolerance).
    sigma_first: f64,
    /// L^-1 as a packed row-major lower triangle (row i holds i + 1 entries),
    /// where L L^T = C_AIR^2 cov restricted to `idx`.
    linv: Vec<f64>,
    /// L^-1 * [1 .. 1]^T: the whitened Jacobian column of the offset unknown,
    /// the same at every iterate.
    u: Vec<f64>,
    alt: Option<(f64, f64)>,
    /// Mirror planes as (point, unit normal), relative coordinates.
    mirrors: Vec<([f64; 3], [f64; 3])>,
    /// Per measurement, per iteration: unwhitened [line-of-sight unit vector
    /// (3), residual]. Allocated once per problem.
    raw: Vec<[f64; 4]>,
}

/// A linearisation at one iterate.
#[derive(Clone, Copy)]
struct Lin {
    chi2: f64,
    /// J^T W J (whitened Jacobian of the model w.r.t. [p, b]).
    n: Matrix4<f64>,
    /// J^T W e (e = measured - model).
    g: Vector4<f64>,
}

/// A converged iterate: x = [p - origin (3), b], b = C_AIR * (t0 - t_ref).
struct Sol {
    x: [f64; 4],
    lin: Lin,
}

/// Outcome of a multi-start search.
#[derive(Default)]
struct Search {
    /// Best fit that passed every plausibility check.
    best: Option<Cand>,
    /// Lowest chi2 among all converged fits, plausible or not, and where.
    min: Option<(f64, Ecef)>,
}

/// A converged fit that passed every plausibility check.
#[derive(Debug, Clone)]
struct Cand {
    pos: Ecef,
    cov3: Matrix3<f64>,
    t0: f64,
    chi2: f64,
    dof: i32,
    err_h: f64,
    /// 1-sigma semi-major axis of the horizontal error ellipse, m.
    major: f64,
    gdop: f64,
    idx: Vec<usize>,
}

impl Problem {
    fn new(inp: &SolveInput, idx: &[usize]) -> Option<Problem> {
        let k = idx.len();
        let mut origin = [0.0; 3];
        for &i in idx {
            for (o, v) in origin.iter_mut().zip(inp.meas[i].rx) {
                *o += v / k as f64;
            }
        }
        let rx: Vec<[f64; 3]> = idx.iter().map(|&i| sub3(&inp.meas[i].rx, &origin)).collect();
        let first = (0..k).min_by(|&a, &b| inp.meas[idx[a]].t.total_cmp(&inp.meas[idx[b]].t))?;
        let t_ref = inp.meas[idx[first]].t;
        let rho = idx.iter().map(|&i| (inp.meas[i].t - t_ref) * C_AIR).collect();

        let c2 = C_AIR * C_AIR;
        let sigma = DMatrix::from_fn(k, k, |a, b| inp.cov[(idx[a], idx[b])] * c2);
        let l = lower_factor(sigma.clone()).or_else(|| {
            // Not positive definite: drop the correlations, keep the variances.
            (0..k).all(|i| sigma[(i, i)] > 0.0).then(|| DMatrix::from_fn(k, k, |a, b| if a == b { sigma[(a, a)].sqrt() } else { 0.0 }))
        })?;

        // Invert the triangular factor column by column (forward substitution).
        let mut linv = vec![0.0; k * (k + 1) / 2];
        let at = |i: usize, j: usize| i * (i + 1) / 2 + j;
        for j in 0..k {
            linv[at(j, j)] = 1.0 / l[(j, j)];
            for i in j + 1..k {
                let mut s = 0.0;
                for m in j..i {
                    s += l[(i, m)] * linv[at(m, j)];
                }
                linv[at(i, j)] = -s / l[(i, i)];
            }
        }
        let u = (0..k).map(|i| linv[at(i, 0)..=at(i, i)].iter().sum()).collect();
        let mirrors = mirror_planes(&rx, &origin);
        Some(Problem {
            origin,
            idx: idx.to_vec(),
            rx,
            rho,
            t_ref,
            sigma_first: sigma[(first, first)].sqrt(),
            linv,
            u,
            alt: inp.alt,
            mirrors,
            raw: vec![[0.0; 4]; k],
        })
    }

    fn dof(&self) -> i32 {
        (self.rx.len() + usize::from(self.alt.is_some())) as i32 - 4
    }

    fn to_rel(&self, p: &Ecef) -> [f64; 3] {
        sub3(p, &self.origin)
    }

    fn to_abs(&self, p: &[f64; 3]) -> Ecef {
        [p[0] + self.origin[0], p[1] + self.origin[1], p[2] + self.origin[2]]
    }

    /// Residual, chi2 and normal equations at x = [p_rel, b].
    fn eval(&mut self, x: &[f64; 4]) -> Option<Lin> {
        let k = self.rx.len();
        for (i, r) in self.rx.iter().enumerate() {
            let d3 = [x[0] - r[0], x[1] - r[1], x[2] - r[2]];
            let d = norm3(&d3);
            // At a receiver the line of sight is undefined; a zero row is a
            // harmless subgradient (the other receivers still constrain p).
            let inv = if d > 1e-6 { 1.0 / d } else { 0.0 };
            self.raw[i] = [d3[0] * inv, d3[1] * inv, d3[2] * inv, self.rho[i] - x[3] - d];
        }
        let mut n = Matrix4::zeros();
        let mut g = Vector4::zeros();
        let mut chi2 = 0.0;
        let mut off = 0;
        for i in 0..k {
            // Row i of L^-1 times the raw block: whitened Jacobian row + residual.
            let mut w = [0.0; 4];
            for (lij, a) in self.linv[off..off + i + 1].iter().zip(&self.raw) {
                w[0] += lij * a[0];
                w[1] += lij * a[1];
                w[2] += lij * a[2];
                w[3] += lij * a[3];
            }
            off += i + 1;
            let j = Vector4::new(w[0], w[1], w[2], self.u[i]);
            n += j * j.transpose();
            g += j * w[3];
            chi2 += w[3] * w[3];
        }
        if let Some((h, s)) = self.alt {
            // d(geodetic height)/d(ECEF) is exactly the ellipsoid normal.
            let llh = geo::ecef_to_llh(self.to_abs(&[x[0], x[1], x[2]]));
            let up = up_vector(&llh);
            let j = Vector4::new(up[0] / s, up[1] / s, up[2] / s, 0.0);
            let e = (h - llh.alt) / s;
            n += j * j.transpose();
            g += j * e;
            chi2 += e * e;
        }
        chi2.is_finite().then_some(Lin { chi2, n, g })
    }

    /// Levenberg-Marquardt from `p0` (relative); None if not converged within
    /// MAX_ITER steps or the problem is singular there.
    fn fit(&mut self, p0: [f64; 3]) -> Option<Sol> {
        // Optimal offset for the starting position: with W = (L L^T)^-1 it is
        // the weighted mean of (rho - d), i.e. g_b / N_bb at b = 0.
        let lin0 = self.eval(&[p0[0], p0[1], p0[2], 0.0])?;
        let b0 = if lin0.n[(3, 3)] > 0.0 { lin0.g[3] / lin0.n[(3, 3)] } else { 0.0 };
        let mut x = [p0[0], p0[1], p0[2], b0];
        let mut lin = self.eval(&x)?;
        let mut lambda = 1e-3;
        let mut nu = 2.0;
        for _ in 0..MAX_ITER {
            if converged(&lin) {
                return Some(Sol { x, lin });
            }
            let dmax = lin.n.diagonal().max();
            let mut a = lin.n;
            let mut dscale = [0.0; 4];
            for (i, d) in dscale.iter_mut().enumerate() {
                *d = lin.n[(i, i)].max(1e-12 * dmax);
                a[(i, i)] += lambda * *d;
            }
            let step = a.cholesky().map(|c| c.solve(&lin.g));
            let trial = step.and_then(|s| {
                let xn = [x[0] + s[0], x[1] + s[1], x[2] + s[2], x[3] + s[3]];
                // 10,000 km from the receivers is not an aircraft; stop the
                // iterate from wandering into float-precision trouble.
                if norm3(&[xn[0], xn[1], xn[2]]) > 1e7 || xn[3].abs() > 1e7 {
                    return None;
                }
                self.eval(&xn).filter(|ln| ln.chi2 <= lin.chi2).map(|ln| (s, xn, ln))
            });
            match trial {
                Some((s, xn, ln)) => {
                    // Gain ratio actual / predicted reduction (Nielsen 1999).
                    let pred = lin.g.dot(&s) + lambda * (0..4).map(|i| dscale[i] * s[i] * s[i]).sum::<f64>();
                    let rho = if pred > 0.0 { (lin.chi2 - ln.chi2) / pred } else { 1.0 };
                    lambda = (lambda * (1.0f64 / 3.0).max(1.0 - (2.0 * rho - 1.0).powi(3))).max(1e-12);
                    nu = 2.0;
                    x = xn;
                    lin = ln;
                }
                None => {
                    lambda *= nu;
                    nu *= 2.0;
                    if lambda > 1e12 {
                        return None;
                    }
                }
            }
        }
        converged(&lin).then_some(Sol { x, lin })
    }

    /// Covariance, plausibility checks and derived quantities of a converged
    /// iterate; None if it is not a believable aircraft position.
    fn validate(&self, sol: &Sol) -> Option<Cand> {
        let chol = sol.lin.n.cholesky()?;
        let dmax = sol.lin.n.diagonal().max();
        let lmat = chol.l_dirty();
        // Written to also catch NaN pivots.
        if (0..4).any(|i| {
            let pivot = lmat[(i, i)] * lmat[(i, i)];
            pivot.is_nan() || pivot <= SINGULAR_PIVOT * dmax
        }) {
            return None;
        }
        let cov4 = chol.inverse();
        let p_rel = [sol.x[0], sol.x[1], sol.x[2]];
        let pos = self.to_abs(&p_rel);
        let llh = geo::ecef_to_llh(pos);
        if !(FIX_ALT.0..=FIX_ALT.1).contains(&llh.alt) {
            return None;
        }
        let nearest = self.rx.iter().map(|r| geo::dist(&p_rel, r)).fold(f64::INFINITY, f64::min);
        if nearest > MAX_RANGE {
            return None;
        }
        if sol.x[3] > T0_TOL_SIGMA * self.sigma_first {
            return None;
        }

        let cov3: Matrix3<f64> = cov4.fixed_view::<3, 3>(0, 0).into_owned();
        let r = geo::Enu::at(pos).r;
        let rot = Matrix3::from_fn(|i, j| r[i][j]);
        let enu = rot * cov3 * rot.transpose();
        let (a, b, c) = (enu[(0, 0)], enu[(0, 1)], enu[(1, 1)]);
        let major = (0.5 * (a + c) + (0.25 * (a - c) * (a - c) + b * b).sqrt()).sqrt();
        let err_h = (a + c).sqrt();
        if major.is_nan() || major > MAX_ELLIPSE || !err_h.is_finite() {
            return None;
        }

        // GDOP: same geometry, unit weights, position + offset unknowns.
        let mut h = Matrix4::zeros();
        for rxi in &self.rx {
            let d3 = sub3(&p_rel, rxi);
            let d = norm3(&d3).max(1e-6);
            let j = Vector4::new(d3[0] / d, d3[1] / d, d3[2] / d, 1.0);
            h += j * j.transpose();
        }
        if self.alt.is_some() {
            let up = up_vector(&llh);
            let j = Vector4::new(up[0], up[1], up[2], 0.0);
            h += j * j.transpose();
        }
        let gdop = h.cholesky().map(|c| c.inverse().trace().sqrt()).unwrap_or(f64::INFINITY);

        Some(Cand {
            pos,
            cov3,
            t0: self.t_ref + sol.x[3] / C_AIR,
            chi2: sol.lin.chi2,
            dof: self.dof(),
            err_h,
            major,
            gdop,
            idx: self.idx.clone(),
        })
    }

    /// Fit from every start (plus mirrors), return the best valid candidate.
    ///
    /// With <= 4 receivers both lobes can be valid, so every start and every
    /// solution is mirrored. With more receivers the ghost lobe still exists
    /// (receivers on the Earth are always nearly coplanar, and a start on
    /// their plane, e.g. a receiver position, can fall either way) but fits
    /// clearly worse, so a solution is mirrored only when it is implausible
    /// or fails the chi2 test; the common case stays a single fit.
    /// `scratch`: a search from scratch, i.e. also from the algebraic starts
    /// and mirroring implausible or chi2-failing solutions. Leave-one-out
    /// subsets that start from the full fit (already in the right basin) skip
    /// both: there a failing chi2 means the outlier is still in, not a ghost.
    fn multistart(&mut self, starts: &[Ecef], reference: &Ecef, scratch: bool) -> Search {
        let always = self.rx.len() <= 4;
        let mut queue: Vec<[f64; 3]> = Vec::with_capacity(starts.len() * 6 + 6);
        let push = |queue: &mut Vec<[f64; 3]>, s: [f64; 3]| {
            queue.push(s);
            if always {
                for m in 0..self.mirrors.len() {
                    queue.push(self.reflect_clamped(&s, m));
                }
            }
        };
        for s in starts {
            push(&mut queue, self.to_rel(&clamp_alt(s)));
        }
        if scratch {
            let h = self.alt.map_or(ALGEBRAIC_ALT, |(h, _)| h.clamp(GUESS_ALT.0, GUESS_ALT.1));
            for s in self.algebraic_starts(h) {
                push(&mut queue, s);
            }
        }
        let direct = queue.len();
        let mut tried: Vec<[f64; 3]> = Vec::with_capacity(queue.len());
        let mut out = Search::default();
        let mut valid: Vec<Cand> = Vec::with_capacity(4);
        let mut qi = 0;
        while qi < queue.len() {
            let s = queue[qi];
            qi += 1;
            if tried.iter().any(|t| geo::dist(t, &s) < 1.0) {
                continue;
            }
            tried.push(s);
            let Some(sol) = self.fit(s) else { continue };
            if out.min.is_none_or(|m| sol.lin.chi2 < m.0) {
                out.min = Some((sol.lin.chi2, self.to_abs(&[sol.x[0], sol.x[1], sol.x[2]])));
            }
            let cand = self.validate(&sol);
            let suspicious = cand.as_ref().is_none_or(|c| c.dof >= 1 && c.chi2 > chi2_q99(c.dof));
            if qi <= direct && (always || (scratch && suspicious)) {
                // Reflect the solution: if this start fell into the ghost
                // lobe, its mirror image starts next to the true one.
                let p = [sol.x[0], sol.x[1], sol.x[2]];
                for m in 0..self.mirrors.len() {
                    queue.push(self.reflect_clamped(&p, m));
                }
            }
            if let Some(c) = cand {
                valid.push(c);
            }
        }
        // Best fit, then the ambiguity check: a distinct solution that fits
        // exactly as well (both exact, dof 0) is the two-lobe ambiguity the
        // data cannot resolve. Only a reference point clearly nearer one
        // lobe (usually the track's prediction) can; failing that the fix is
        // bimodal, no Gaussian summarises it, and it is refused.
        let Some(bi) = (0..valid.len()).reduce(|a, b| if better(&valid[b], &valid[a], reference) { b } else { a }) else {
            return out;
        };
        let best = valid.swap_remove(bi);
        let d_best = geo::dist(&best.pos, reference);
        let ambiguous = valid.iter().any(|c| {
            let distinct = geo::dist(&c.pos, &best.pos) > 5.0 * (c.major + best.major) + 1_000.0;
            distinct && chi2_tie(c.chi2, best.chi2) && geo::dist(&c.pos, reference) < AMBIGUITY_RATIO * d_best
        });
        if !ambiguous {
            out.best = Some(best);
        }
        out
    }

    /// Closed-form starting points, relative coordinates (see the module
    /// notes). In the ENU frame of the centroid, with the aircraft's up
    /// coordinate U fixed from a height `h`, the squared range equations
    /// |p - q_i|^2 = (rho_i - b)^2 are linear in (E, N, w) for a given offset b,
    /// with w = b^2 - |p|^2. Solving them by least squares as (E, N, w) =
    /// alpha + beta * b and imposing the definition of w leaves a quadratic in
    /// b; its (up to two) roots are the two lobes. U depends on (E, N) through
    /// the Earth's curvature, so each root is refined twice.
    fn algebraic_starts(&self, h: f64) -> Vec<[f64; 3]> {
        let mut out = Vec::new();
        if self.rx.len() < 3 {
            return out;
        }
        let frame = geo::Enu::at(self.origin);
        let h_c = geo::ecef_to_llh(self.origin).alt;
        let r = frame.r;
        let rot = |v: &[f64; 3]| {
            [
                r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
                r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
                r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
            ]
        };
        // Scale to O(1): squared ranges of 1e10 m^2 next to unit columns
        // would cost ten digits in the normal equations.
        let q: Vec<[f64; 3]> = self.rx.iter().map(rot).collect();
        let s = q.iter().map(norm3).fold(1e3, f64::max);
        let q: Vec<[f64; 3]> = q.iter().map(|v| [v[0] / s, v[1] / s, v[2] / s]).collect();
        let rho: Vec<f64> = self.rho.iter().map(|v| v / s).collect();
        let up_of = |e: f64, n: f64| (h - h_c) / s - (e * e + n * n) * s / (2.0 * EARTH_R);

        for (e0, n0, _) in algebraic_roots(&q, &rho, up_of(0.0, 0.0)) {
            let (mut e, mut n) = (e0, n0);
            for _ in 0..2 {
                let next = algebraic_roots(&q, &rho, up_of(e, n));
                let Some(&(e1, n1, _)) = next.iter().min_by(|a, b| {
                    let da = (a.0 - e).powi(2) + (a.1 - n).powi(2);
                    let db = (b.0 - e).powi(2) + (b.1 - n).powi(2);
                    da.total_cmp(&db)
                }) else {
                    break;
                };
                (e, n) = (e1, n1);
            }
            if (e * e + n * n).sqrt() * s > MAX_RANGE + 400e3 {
                continue;
            }
            let l = [e * s, n * s, up_of(e, n) * s];
            // ENU -> ECEF offset: the rows of r are orthonormal, so R^T.
            out.push([
                r[0][0] * l[0] + r[1][0] * l[1] + r[2][0] * l[2],
                r[0][1] * l[0] + r[1][1] * l[1] + r[2][1] * l[2],
                r[0][2] * l[0] + r[1][2] * l[1] + r[2][2] * l[2],
            ]);
        }
        out
    }

    fn reflect_clamped(&self, p: &[f64; 3], m: usize) -> [f64; 3] {
        let (q, n) = &self.mirrors[m];
        let s = 2.0 * ((p[0] - q[0]) * n[0] + (p[1] - q[1]) * n[1] + (p[2] - q[2]) * n[2]);
        let r = [p[0] - s * n[0], p[1] - s * n[1], p[2] - s * n[2]];
        self.to_rel(&clamp_alt(&self.to_abs(&r)))
    }
}

/// Cholesky factor of a covariance, None unless clearly positive definite
/// (a pivot ~1e-12 of its diagonal means numerically singular: whitening by
/// it would amplify rounding into fake precision).
fn lower_factor(sigma: DMatrix<f64>) -> Option<DMatrix<f64>> {
    let diag: Vec<f64> = sigma.diagonal().iter().copied().collect();
    let l = sigma.cholesky()?.unpack();
    (0..diag.len()).all(|i| l[(i, i)] * l[(i, i)] > 1e-12 * diag[i]).then_some(l)
}

fn converged(lin: &Lin) -> bool {
    lin.n.cholesky().is_some_and(|c| lin.g.dot(&c.solve(&lin.g)) < CONV_DECREMENT)
}

/// Roots (E, N, b) of the fixed-height algebraic system (scaled units) for
/// receivers `q` and pseudoranges `rho`; see `Problem::algebraic_starts`.
/// Only roots with b <= ~0 are physical (transmission before reception).
fn algebraic_roots(q: &[[f64; 3]], rho: &[f64], up: f64) -> Vec<(f64, f64, f64)> {
    // Normal equations of rows [2e, 2n, 1] against y + z b.
    let mut m = Matrix3::zeros();
    let mut ay = nalgebra::Vector3::zeros();
    let mut az = nalgebra::Vector3::zeros();
    for (qi, &ri) in q.iter().zip(rho) {
        let a = nalgebra::Vector3::new(2.0 * qi[0], 2.0 * qi[1], 1.0);
        let y = qi[0] * qi[0] + qi[1] * qi[1] + qi[2] * qi[2] - ri * ri - 2.0 * up * qi[2];
        m += a * a.transpose();
        ay += a * y;
        az += a * (2.0 * ri);
    }
    let Some(ch) = m.cholesky() else { return Vec::new() };
    let (al, be) = (ch.solve(&ay), ch.solve(&az));
    let a2 = 1.0 - be[0] * be[0] - be[1] * be[1];
    let b2 = -(be[2] + 2.0 * (al[0] * be[0] + al[1] * be[1]));
    let c2 = -(al[2] + al[0] * al[0] + al[1] * al[1] + up * up);
    let mut bs = Vec::with_capacity(2);
    if a2.abs() < 1e-12 {
        if b2.abs() > 1e-12 {
            bs.push(-c2 / b2);
        }
    } else {
        let disc = b2 * b2 - 4.0 * a2 * c2;
        if disc < 0.0 {
            // Noise pushed the lobes apart: take the closest approach.
            bs.push(-b2 / (2.0 * a2));
        } else {
            let qq = -0.5 * (b2 + b2.signum() * disc.sqrt());
            bs.push(qq / a2);
            if qq != 0.0 {
                bs.push(c2 / qq);
            }
        }
    }
    bs.into_iter()
        .filter(|b| b.is_finite() && *b <= 0.01)
        .map(|b| (al[0] + be[0] * b, al[1] + be[1] * b, b))
        .filter(|r| r.0.is_finite() && r.1.is_finite())
        .collect()
}

/// Mirror planes for the two-lobe ambiguity of small receiver sets (see the
/// module notes). Receiver positions are relative to their centroid.
fn mirror_planes(rx: &[[f64; 3]], origin: &Ecef) -> Vec<([f64; 3], [f64; 3])> {
    let k = rx.len() as f64;
    let mut c = [0.0; 3];
    for r in rx {
        for (ci, ri) in c.iter_mut().zip(r) {
            *ci += ri / k;
        }
    }
    let mut s = Matrix3::zeros();
    for r in rx {
        let d = nalgebra::Vector3::new(r[0] - c[0], r[1] - c[1], r[2] - c[2]);
        s += d * d.transpose();
    }
    let eig = SymmetricEigen::new(s);
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| eig.eigenvalues[b].total_cmp(&eig.eigenvalues[a]));
    let (l1, l2) = (eig.eigenvalues[order[0]], eig.eigenvalues[order[1]]);
    let mut out = Vec::with_capacity(2);
    if l1.is_nan() || l1 <= 1.0 {
        // All receivers within a metre of each other: no geometry to mirror.
        return out;
    }
    let axis = eig.eigenvectors.column(order[0]).into_owned();
    let up = up_vector(&geo::ecef_to_llh(*origin));
    let up = nalgebra::Vector3::new(up[0], up[1], up[2]);
    let n = axis.cross(&up);
    if n.norm() > 0.1 {
        let n = n.normalize();
        out.push((c, [n[0], n[1], n[2]]));
    }
    if l2 > 1e-6 * l1 {
        let n = eig.eigenvectors.column(order[2]);
        out.push((c, [n[0], n[1], n[2]]));
    }
    out
}

/// Candidate ordering: lower chi2; on a tie (exactly determined fits) the one
/// nearer the reference point.
fn better(a: &Cand, b: &Cand, reference: &Ecef) -> bool {
    if chi2_tie(a.chi2, b.chi2) {
        geo::dist(&a.pos, reference) < geo::dist(&b.pos, reference)
    } else {
        a.chi2 < b.chi2
    }
}

/// Equal up to convergence noise (exact fits end at chi2 ~1e-12..1e-9).
fn chi2_tie(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-6 * (1.0 + a.min(b))
}

fn clamp_alt(p: &Ecef) -> Ecef {
    let llh = geo::ecef_to_llh(*p);
    if llh.alt < GUESS_ALT.0 || llh.alt > GUESS_ALT.1 {
        geo::llh_to_ecef(Llh { alt: llh.alt.clamp(GUESS_ALT.0, GUESS_ALT.1), ..llh })
    } else {
        *p
    }
}

/// Outward ellipsoid normal (geodetic "up") at a geodetic position, ECEF.
pub(crate) fn up_vector(llh: &Llh) -> [f64; 3] {
    let (slat, clat) = llh.lat.to_radians().sin_cos();
    let (slon, clon) = llh.lon.to_radians().sin_cos();
    [clat * clon, clat * slon, slat]
}

fn sub3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm3(a: &[f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn finite3(a: &[f64; 3]) -> bool {
    a.iter().all(|v| v.is_finite())
}
