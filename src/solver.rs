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

use nalgebra::{DMatrix, Matrix3};

use crate::geo::Ecef;

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

pub fn solve(_inp: &SolveInput) -> Option<Fix> {
    todo!()
}
