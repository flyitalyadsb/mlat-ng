//! Aircraft heard by only two receiver sites: a position from minutes of
//! their TDOA plus the aircraft's own motion and the receivers' coverage.
//!
//! One TDOA from a pair of sites puts the aircraft somewhere on a hyperbola.
//! Many TDOAs of a MOVING aircraft whose velocity we know (Comm-B BDS 5,0
//! ground velocity) do not: going back in time along the known velocity, only
//! one point at the end of the window makes every earlier TDOA fit, because
//! the aircraft crosses the hyperbola family at a rate that depends on where it
//! is. The mirror point across the baseline fits too when the velocity is
//! parallel to the baseline; coverage breaks that tie (where the two receivers
//! hear, and where the receivers nearby that did NOT hear the aircraft would
//! have).
//!
//! Measured offline on the 2026-10-24 capture (scratchpad motion study,
//! aircraft that only two receivers heard, 240 s windows): motion alone p50
//! 2.4 km / p90 82 km; with coverage p50 2.2 km / p90 29 km. The tail is why
//! every solution carries its own checks (residual, uniqueness margin, error
//! estimate) and the engine decides what to do with it.
//!
//! Frame: a local tangent plane at the midpoint of the two sites, with the
//! Earth's curvature as a correction on the up axis (good to a few metres at
//! 350 km, far below the timing noise). The aircraft's displacements are in
//! its OWN local east/north: at a candidate point Δlon away from the origin
//! north is rotated by the meridian convergence (Δlon · sin lat), which at
//! 300 km east of the origin is ~2.7 degrees, ~2.6 km over a 55 km path.

use crate::geo::{dist, ecef_to_llh, Ecef, Enu};

/// One 2-site measurement.
#[derive(Debug, Clone)]
pub struct Rec2 {
    /// Wall time of the transmission, seconds.
    pub t: f64,
    /// Where the aircraft went from this record to the end of the window
    /// (east, north metres, in the aircraft's own local frame): the integral
    /// of its ground velocity, so turns are followed.
    pub off: [f64; 2],
    pub ra: Ecef,
    pub rb: Ecef,
    /// |p - rb| - |p - ra|, metres (arrival time difference x C_AIR).
    pub d: f64,
    /// 1-sigma of `d`, metres.
    pub sd: f64,
    /// Aircraft height above the ellipsoid at `t`, metres.
    pub h: f64,
}

#[derive(Debug, Clone)]
pub struct Solution {
    /// Position at `t_end`, at the last record's height.
    pub pos: Ecef,
    /// 1-sigma horizontal error from the fit, metres.
    pub err_h: f64,
    /// RMS of the normalised residuals.
    pub rms: f64,
    /// Total cost of the second-best basin (>= 20 km away) minus the best's:
    /// how much better this point explains the data than any other.
    pub margin: f64,
    pub n: usize,
    /// Seconds between the first and the last record.
    pub span: f64,
}

/// Half-size and step of the coarse grid, metres.
const GRID_HALF: f64 = 350_000.0;
const GRID_STEP: f64 = 5_000.0;
/// Soft-L1 scale (in sigmas): residuals beyond it count linearly.
const SOFT: f64 = 3.0;
/// Grid points within this timing cost of the best go on to coverage.
const CANDIDATE_COST: f64 = 40.0;
const MAX_CANDIDATES: usize = 600;
const BASINS: usize = 5;
const BASIN_SEP: f64 = 20_000.0;
/// A refinement may move this far from its grid start.
const REFINE_BOUND: f64 = 30_000.0;
const R_EARTH: f64 = 6_371_000.0;
/// Margin reported when there is no second basin at all.
pub const NO_RIVAL: f64 = 1e6;

fn soft(r: f64) -> f64 {
    2.0 * ((1.0 + (r / SOFT) * (r / SOFT)).sqrt() - 1.0)
}

struct Frame {
    lat0: f64,
    o: Ecef,
    e: [f64; 3],
    n: [f64; 3],
    u: [f64; 3],
    enu: Enu,
}

impl Frame {
    fn at(a: &Ecef, b: &Ecef) -> Frame {
        let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0];
        let mut l = ecef_to_llh(mid);
        l.alt = 0.0;
        let o = crate::geo::llh_to_ecef(l);
        let enu = Enu::at(o);
        let (lat, lon) = (l.lat.to_radians(), l.lon.to_radians());
        let (sl, cl) = lat.sin_cos();
        let (so, co) = lon.sin_cos();
        Frame { lat0: lat, o, e: [-so, co, 0.0], n: [-sl * co, -sl * so, cl], u: [cl * co, cl * so, sl], enu }
    }

    /// The point (e, n) of the plane at height h, curvature-corrected.
    fn pos(&self, e: f64, n: f64, h: f64) -> Ecef {
        let drop = (e * e + n * n) / (2.0 * (R_EARTH + h));
        let up = h - drop;
        [
            self.o[0] + e * self.e[0] + n * self.n[0] + up * self.u[0],
            self.o[1] + e * self.e[1] + n * self.n[1] + up * self.u[1],
            self.o[2] + e * self.e[2] + n * self.n[2] + up * self.u[2],
        ]
    }

    /// (cos, sin) of the meridian convergence at plane point (e, n): local
    /// north there, in plane axes, is (-sin, cos).
    fn convergence(&self, e: f64) -> (f64, f64) {
        let dlon = e / (R_EARTH * self.lat0.cos().max(0.05));
        let g = dlon * self.lat0.sin();
        (g.cos(), g.sin())
    }

    fn latlon(&self, e: f64, n: f64) -> (f64, f64) {
        let l = ecef_to_llh(self.enu.to_ecef(&[e, n, 0.0]));
        (l.lat, l.lon)
    }
}

/// Displacements from each time in `ts` to `t_end` (east, north m) along a
/// piecewise-constant velocity history `vel` ((east, north m/s), time),
/// sorted by time; before the first sample its value holds.
pub fn offsets(ts: &[f64], vel: &[([f64; 2], f64)], t_end: f64) -> Vec<[f64; 2]> {
    let v_at = |t: f64| vel.iter().rev().find(|x| x.1 <= t).or(vel.first()).map(|x| x.0).unwrap_or([0.0; 2]);
    ts.iter()
        .map(|&t0| {
            // Integrate from t0 to t_end, cutting at each velocity change.
            let mut cuts: Vec<f64> = vel.iter().map(|x| x.1).filter(|&c| c > t0 && c < t_end).collect();
            cuts.push(t_end);
            let (mut a, mut out) = (t0, [0.0; 2]);
            for b in cuts {
                let v = v_at(a);
                out[0] += v[0] * (b - a);
                out[1] += v[1] * (b - a);
                a = b;
            }
            out
        })
        .collect()
}

/// Normalised residual of record `r` for an end position (e, n), and its
/// gradient in (e, n) (the convergence held fixed: it varies over hundreds
/// of km, the step over metres).
fn residual(f: &Frame, r: &Rec2, e_end: f64, n_end: f64) -> (f64, [f64; 2]) {
    let (c, s) = f.convergence(e_end);
    let (e, n) = (e_end - (r.off[0] * c - r.off[1] * s), n_end - (r.off[0] * s + r.off[1] * c));
    let p = f.pos(e, n, r.h);
    let (da, db) = (dist(&p, &r.ra), dist(&p, &r.rb));
    let res = (db - da - r.d) / r.sd;
    let k = 1.0 / (R_EARTH + r.h);
    let dpe = [f.e[0] - e * k * f.u[0], f.e[1] - e * k * f.u[1], f.e[2] - e * k * f.u[2]];
    let dpn = [f.n[0] - n * k * f.u[0], f.n[1] - n * k * f.u[1], f.n[2] - n * k * f.u[2]];
    let g = |dp: &[f64; 3]| {
        let mut s = 0.0;
        for i in 0..3 {
            s += (p[i] - r.rb[i]) / db * dp[i] - (p[i] - r.ra[i]) / da * dp[i];
        }
        s / r.sd
    };
    (res, [g(&dpe), g(&dpn)])
}

fn timing_cost(f: &Frame, recs: &[Rec2], e: f64, n: f64) -> f64 {
    recs.iter().map(|r| soft(residual(f, r, e, n).0)).sum()
}

/// Levenberg-Marquardt on the timing alone (soft-L1 by reweighting), within
/// REFINE_BOUND of the start. Returns (e, n, normal matrix).
fn refine(f: &Frame, recs: &[Rec2], start: [f64; 2]) -> ([f64; 2], [[f64; 2]; 2]) {
    let mut x = start;
    let mut mu = 1e-3;
    let mut cost = timing_cost(f, recs, x[0], x[1]);
    let mut normal = [[0.0; 2]; 2];
    for _ in 0..40 {
        let (mut a, mut g) = ([[0.0; 2]; 2], [0.0; 2]);
        for r in recs {
            let (res, j) = residual(f, r, x[0], x[1]);
            let w = 1.0 / (1.0 + (res / SOFT) * (res / SOFT)).sqrt();
            for p in 0..2 {
                g[p] += w * j[p] * res;
                for q in 0..2 {
                    a[p][q] += w * j[p] * j[q];
                }
            }
        }
        normal = a;
        let mut improved = false;
        for _ in 0..8 {
            let (a00, a11) = (a[0][0] * (1.0 + mu), a[1][1] * (1.0 + mu));
            let det = a00 * a11 - a[0][1] * a[1][0];
            if det.abs() < 1e-30 {
                mu *= 10.0;
                continue;
            }
            let dx = [-(a11 * g[0] - a[0][1] * g[1]) / det, -(a00 * g[1] - a[1][0] * g[0]) / det];
            let cand = [
                (x[0] + dx[0]).clamp(start[0] - REFINE_BOUND, start[0] + REFINE_BOUND),
                (x[1] + dx[1]).clamp(start[1] - REFINE_BOUND, start[1] + REFINE_BOUND),
            ];
            let c = timing_cost(f, recs, cand[0], cand[1]);
            if c < cost {
                let step = ((cand[0] - x[0]).powi(2) + (cand[1] - x[1]).powi(2)).sqrt();
                x = cand;
                cost = c;
                mu = (mu * 0.3).max(1e-9);
                improved = step > 0.5;
                break;
            }
            mu *= 10.0;
        }
        if !improved {
            break;
        }
    }
    (x, normal)
}

/// Solve for the position at the last record's time, from the records and
/// their displacements to it (`Rec2::off`). `cov_cost(lat, lon)` is the
/// coverage evidence as a cost (minus a log-likelihood); pass `|_, _| 0.0`
/// for none.
pub fn solve(recs: &[Rec2], cov_cost: &dyn Fn(f64, f64) -> f64) -> Option<Solution> {
    if recs.len() < 4 {
        return None;
    }
    let last = recs.last()?;
    let t_end = last.t;
    let f = Frame::at(&last.ra, &last.rb);
    // Coarse grid on the timing.
    let k = (GRID_HALF / GRID_STEP) as i32;
    let mut grid: Vec<(f64, f64, f64)> = Vec::with_capacity(((2 * k + 1) * (2 * k + 1)) as usize);
    for i in -k..=k {
        for j in -k..=k {
            let (e, n) = (i as f64 * GRID_STEP, j as f64 * GRID_STEP);
            grid.push((timing_cost(&f, recs, e, n), e, n));
        }
    }
    let best_t = grid.iter().map(|g| g.0).fold(f64::INFINITY, f64::min);
    let mut cands: Vec<(f64, f64, f64)> = grid.into_iter().filter(|g| g.0 <= best_t + CANDIDATE_COST).collect();
    cands.sort_by(|a, b| a.0.total_cmp(&b.0));
    cands.truncate(MAX_CANDIDATES);
    // Coverage picks among the basins the timing allows.
    let mut scored: Vec<(f64, f64, f64)> = cands
        .iter()
        .map(|&(c, e, n)| {
            let (lat, lon) = f.latlon(e, n);
            (c + cov_cost(lat, lon), e, n)
        })
        .collect();
    scored.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut starts: Vec<[f64; 2]> = Vec::new();
    for &(_, e, n) in &scored {
        if starts.iter().all(|s| ((s[0] - e).powi(2) + (s[1] - n).powi(2)).sqrt() > BASIN_SEP) {
            starts.push([e, n]);
            if starts.len() == BASINS {
                break;
            }
        }
    }
    // Refine each basin on the timing; judge them on timing + coverage.
    let mut basins: Vec<(f64, [f64; 2], [[f64; 2]; 2])> = starts
        .iter()
        .map(|s| {
            let (x, normal) = refine(&f, recs, *s);
            let (lat, lon) = f.latlon(x[0], x[1]);
            (timing_cost(&f, recs, x[0], x[1]) + cov_cost(lat, lon), x, normal)
        })
        .collect();
    basins.sort_by(|a, b| a.0.total_cmp(&b.0));
    let (best_cost, x, a) = basins.first().cloned()?;
    let margin = basins
        .iter()
        .skip(1)
        .find(|b| ((b.1[0] - x[0]).powi(2) + (b.1[1] - x[1]).powi(2)).sqrt() > BASIN_SEP)
        .map(|b| b.0 - best_cost)
        .unwrap_or(NO_RIVAL);
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    let err_h = if det > 0.0 { ((a[0][0] + a[1][1]) / det).sqrt() } else { f64::INFINITY };
    let rms = (recs.iter().map(|r| residual(&f, r, x[0], x[1]).0.powi(2)).sum::<f64>()
        / recs.len() as f64)
        .sqrt();
    Some(Solution {
        pos: f.pos(x[0], x[1], last.h),
        err_h,
        rms,
        margin,
        n: recs.len(),
        span: t_end - recs[0].t,
    })
}

#[cfg(test)]
mod tests;
