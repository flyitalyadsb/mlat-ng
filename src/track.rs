//! Per-aircraft tracking filter, with direct TDOA updates.
//!
//! IMPLEMENTATION OWNER: agent "solver" (same agent as solver.rs).
//!
//! Why direct TDOA updates matter: a stand-alone fix needs >= 4 receivers, or
//! 3 + altitude. Once a track exists, its prediction is a strong prior, so a
//! group seen by only 2-3 receivers (+ Mode S altitude) still carries useful
//! information. Feeding the arrival times straight into the filter (instead of
//! requiring a fix first) is what lets sparse networks keep tracks alive.
//!
//! Requirements:
//! - EKF in a local ENU frame anchored at the first fix (geo::Enu). State
//!   [e, n, u, ve, vn, vu]; constant velocity; white-acceleration process
//!   noise, sigma_a = 3 m/s^2 horizontal, 1.5 m/s^2 vertical.
//! - `new` from a Fix (velocity unknown: sigma 250 m/s horizontal, 30 m/s
//!   vertical).
//! - `update_fix`: position measurement with the fix covariance (rotate ECEF
//!   cov to ENU). Gate on the Mahalanobis distance (chi2, 3 dof, 99.9%);
//!   reject outliers without touching the state.
//! - `update_tdoa`: measurements = arrival-time differences w.r.t. the first
//!   measurement, predicted from the state position; measurement covariance
//!   derived from the k x k time covariance (D cov D^T with the differencing
//!   matrix D), times C_AIR^2 to work in metres. Optional altitude
//!   measurement on u. Gate with chi2 at 99.9% for (k-1 [+1]) dof. Require
//!   k >= 2 (k-1 >= 1 difference) and a track updated within the last 30 s.
//!   Iterate the EKF linearisation up to 3 times (IEKF) for accuracy.
//! - Confirmation: `confirmed()` after 3 accepted updates of which >= 2 are
//!   fixes, within 60 s. Tracks not updated for 120 s are `stale()`.
//! - Derived outputs: ECEF position, ENU velocity, ground speed (kt), track
//!   angle (deg, 0 = north), vertical rate (ft/min), 1-sigma horizontal
//!   position error (m).
//! - Unit tests: straight-line synthetic flight with noisy fixes converges
//!   (position error < fix error, speed within 5 kt after 30 s); TDOA-only
//!   updates keep a track within 1 km over 60 s with 3 receivers + altitude;
//!   a 50 km outlier fix is rejected; a 2 g turn is followed.
//!
//! Implementation notes:
//!
//! - One model cannot meet both test requirements. With the specified
//!   sigma_a = 3 m/s^2 a constant-velocity filter (a) lags a sustained turn by
//!   roughly (turn acceleration / sigma_a) measurement sigmas, ~6 sigma in a
//!   2 g turn, far outside a 99.9% gate, so it rejects every fix of the turn
//!   and loses the aircraft; and (b) its velocity noise in straight flight is
//!   ~8 kt RMS with 150 m fixes at 1 Hz (it keeps re-estimating a velocity it
//!   believes may change by 3 m/s every second). The filter is therefore an
//!   IMM (interacting multiple model, Blom & Bar-Shalom) over three CV EKFs
//!   in the same ENU frame, differing only in process noise: steady cruise
//!   (0.1 m/s^2), the specified 3 / 1.5 m/s^2 model, and a 2 g manoeuvre
//!   model, with continuous-time Markov switching. In straight flight the
//!   steady model carries the weight and the speed settles to ~2 kt RMS with
//!   100 m fixes at 1 Hz (8 kt for the specified model alone); in a turn the
//!   weight moves to the manoeuvre model within a couple of updates. A measurement is accepted when it passes the gate under at
//!   least one model.
//! - Process noise is the discrete white-noise-acceleration model (piecewise
//!   constant acceleration of sigma_a over each interval), per ENU axis.
//! - TDOA updates are an iterated EKF: re-linearising the range differences
//!   at the updated position matters when the prior is wide (a fresh track)
//!   and the receivers are close. The altitude row uses the geodetic height of
//!   the ENU point (exact anywhere), whose gradient is the local vertical: the
//!   frame's u axis near the anchor, tilted by the Earth's curvature farther
//!   out.
//! - Derived velocities are rotated into the ENU frame at the current
//!   position, not the anchor's: 300 km from the anchor a level aircraft has
//!   a 2.7 degree pitch in the anchor frame (a fake 2000+ ft/min).

use std::collections::VecDeque;

use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, Vector3, Vector6};

use crate::geo::{self, Ecef, Enu, C_AIR, MS_TO_KT, M_TO_FT};
use crate::solver::{chi2_q999, up_vector, Fix, Meas};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gate {
    Accepted,
    /// Mahalanobis distance squared of the rejected innovation.
    Rejected(f64),
    /// Not applicable (too few measurements, track too old, singular).
    Skipped,
}

/// Number of models in the IMM bank.
const N: usize = 3;
/// Index of the specified model (sigma_a 3 / 1.5 m/s^2).
const NOMINAL: usize = 1;
/// White-acceleration sigmas (e, n, u), m/s^2: steady cruise (autopilot
/// corrections, turbulence), the specified model (gentle turns, speed and
/// level changes), and manoeuvres (2 g horizontally: steep turns, light
/// aircraft; 0.5 g vertically: level-offs).
const SIGMA_A: [[f64; 3]; N] = [[0.1, 0.1, 0.05], [3.0, 3.0, 1.5], [20.0, 20.0, 5.0]];
/// Switching rates of the Markov chain between models, 1/s: RATES[i][j] is
/// the rate of leaving model i for model j. Cruise legs last ~10 minutes,
/// manoeuvres ~10 s. The low leak out of the steady model is what keeps its
/// velocity clean in straight flight (each step mixes in a little of the
/// noisier models); a turn still moves the weight within 1-2 updates, as
/// the innovations under the steady model grow by many sigma.
const RATES: [[f64; N]; N] = [
    [0.0, 1.0 / 600.0, 1.0 / 3000.0],
    [1.0 / 10.0, 0.0, 1.0 / 60.0],
    [1.0 / 60.0, 1.0 / 10.0, 0.0],
];
/// Initial model probabilities: a new track's regime is unknown.
const MU0: [f64; N] = [0.4, 0.4, 0.2];
/// Initial velocity sigmas (horizontal, vertical), m/s.
const SIGMA_V0: [f64; 2] = [250.0, 30.0];
/// TDOA updates need a track updated at least this recently (s).
const TDOA_MAX_AGE: f64 = 30.0;
/// Up to this many linearisations per TDOA update (IEKF).
const IEKF_ITER: usize = 3;
/// Confirmation window and stale age (s).
const CONFIRM_WINDOW: f64 = 60.0;
const STALE_AGE: f64 = 120.0;

/// One constant-velocity EKF of the bank.
#[derive(Debug, Clone)]
struct Model {
    x: Vector6<f64>,
    p: Matrix6<f64>,
}

#[derive(Debug, Clone)]
pub struct Track {
    frame: Enu,
    /// Rows of `frame.r` as a matrix: ENU = rot * (ECEF - origin).
    rot: Matrix3<f64>,
    /// Time of the state (last accepted update), unix s.
    t: f64,
    models: [Model; N],
    /// Model probabilities.
    mu: [f64; N],
    /// False only in tests: freeze the bank to the specified model, i.e. the
    /// plain single-model EKF.
    imm: bool,
    updates: u32,
    /// Accepted updates within the confirmation window: (time, was a fix).
    recent: VecDeque<(f64, bool)>,
    confirmed: bool,
}

/// What an update measures, linearised on demand (the IEKF re-linearises).
trait Observation {
    fn dim(&self) -> usize;
    /// Measurement covariance, m^2.
    fn r(&self) -> &DMatrix<f64>;
    /// (z - h(x), dh/dx) at state x; None if undefined there.
    fn linearise(&self, x: &Vector6<f64>) -> Option<(DVector<f64>, DMatrix<f64>)>;
    fn iterations(&self) -> usize;
}

/// A position fix in the track frame.
struct PosObs {
    z: Vector3<f64>,
    r: DMatrix<f64>,
}

impl Observation for PosObs {
    fn dim(&self) -> usize {
        3
    }
    fn r(&self) -> &DMatrix<f64> {
        &self.r
    }
    fn linearise(&self, x: &Vector6<f64>) -> Option<(DVector<f64>, DMatrix<f64>)> {
        let resid = DVector::from_fn(3, |i, _| self.z[i] - x[i]);
        let h = DMatrix::from_fn(3, 6, |i, j| if i == j { 1.0 } else { 0.0 });
        Some((resid, h))
    }
    fn iterations(&self) -> usize {
        1
    }
}

/// Range differences to `rx[1..]` against `rx[0]` (+ optional height).
struct TdoaObs {
    frame: Enu,
    rot: Matrix3<f64>,
    rx: Vec<Ecef>,
    /// Measured C_AIR * (t_i - t_0), i = 1..k-1, then the height if any.
    z: DVector<f64>,
    r: DMatrix<f64>,
    alt: bool,
}

impl Observation for TdoaObs {
    fn dim(&self) -> usize {
        self.z.len()
    }
    fn r(&self) -> &DMatrix<f64> {
        &self.r
    }
    fn linearise(&self, x: &Vector6<f64>) -> Option<(DVector<f64>, DMatrix<f64>)> {
        let p = self.frame.to_ecef(&[x[0], x[1], x[2]]);
        let m = self.dim();
        let k = self.rx.len();
        let mut resid = DVector::zeros(m);
        let mut h = DMatrix::zeros(m, 6);
        // Unit line of sight from receiver i, in the track's ENU axes.
        let los = |r: &Ecef| -> Option<(f64, Vector3<f64>)> {
            let d = geo::dist(&p, r);
            (d > 1e-3).then(|| (d, self.rot * Vector3::new((p[0] - r[0]) / d, (p[1] - r[1]) / d, (p[2] - r[2]) / d)))
        };
        let (d0, u0) = los(&self.rx[0])?;
        for i in 1..k {
            let (di, ui) = los(&self.rx[i])?;
            resid[i - 1] = self.z[i - 1] - (di - d0);
            for c in 0..3 {
                h[(i - 1, c)] = ui[c] - u0[c];
            }
        }
        if self.alt {
            let llh = geo::ecef_to_llh(p);
            let up = up_vector(&llh);
            let g = self.rot * Vector3::new(up[0], up[1], up[2]);
            resid[m - 1] = self.z[m - 1] - llh.alt;
            for c in 0..3 {
                h[(m - 1, c)] = g[c];
            }
        }
        Some((resid, h))
    }
    fn iterations(&self) -> usize {
        IEKF_ITER
    }
}

/// Ground velocity east/north at the aircraft (m/s), as Comm-B BDS 5,0
/// reports it (ground speed + true track, from GNSS/IRS: no wind error). A
/// linear observation of the state velocity, projected on the local east and
/// north at the state position (the frame's axes drift from those with
/// distance from its anchor).
struct VelObs {
    frame: Enu,
    rot: Matrix3<f64>,
    z: DVector<f64>,
    r: DMatrix<f64>,
}

impl Observation for VelObs {
    fn dim(&self) -> usize {
        2
    }
    fn r(&self) -> &DMatrix<f64> {
        &self.r
    }
    fn linearise(&self, x: &Vector6<f64>) -> Option<(DVector<f64>, DMatrix<f64>)> {
        let llh = geo::ecef_to_llh(self.frame.to_ecef(&[x[0], x[1], x[2]]));
        let (sla, cla) = llh.lat.to_radians().sin_cos();
        let (slo, clo) = llh.lon.to_radians().sin_cos();
        let east = self.rot * Vector3::new(-slo, clo, 0.0);
        let north = self.rot * Vector3::new(-sla * clo, -sla * slo, cla);
        let mut h = DMatrix::zeros(2, 6);
        for c in 0..3 {
            h[(0, 3 + c)] = east[c];
            h[(1, 3 + c)] = north[c];
        }
        let v = Vector3::new(x[3], x[4], x[5]);
        let resid = DVector::from_vec(vec![self.z[0] - east.dot(&v), self.z[1] - north.dot(&v)]);
        Some((resid, h))
    }
    fn iterations(&self) -> usize {
        1
    }
}

/// Result of updating one model.
struct Upd {
    model: Model,
    /// Squared Mahalanobis distance of the predicted innovation.
    d2: f64,
    /// Log-likelihood of the innovation (for the model probabilities).
    ln_lik: f64,
}

impl Track {
    pub fn new(fix: &Fix, t: f64) -> Track {
        let frame = Enu::at(fix.pos);
        let rot = Matrix3::from_fn(|i, j| frame.r[i][j]);
        let mut pc = rot * fix.cov * rot.transpose();
        pc = 0.5 * (pc + pc.transpose());
        if !pc.iter().all(|v| v.is_finite()) || pc.cholesky().is_none() {
            // A fix without a usable covariance: assume a poor one.
            pc = Matrix3::from_diagonal_element(1e6);
        }
        let mut p = Matrix6::zeros();
        p.fixed_view_mut::<3, 3>(0, 0).copy_from(&pc);
        p[(3, 3)] = SIGMA_V0[0] * SIGMA_V0[0];
        p[(4, 4)] = SIGMA_V0[0] * SIGMA_V0[0];
        p[(5, 5)] = SIGMA_V0[1] * SIGMA_V0[1];
        let model = Model { x: Vector6::zeros(), p };
        let mut recent = VecDeque::with_capacity(8);
        recent.push_back((t, true));
        Track { frame, rot, t, models: std::array::from_fn(|_| model.clone()), mu: MU0, imm: true, updates: 1, recent, confirmed: false }
    }

    /// Time of the last accepted update (unix seconds).
    pub fn last_update(&self) -> f64 {
        self.t
    }

    /// Predicted ECEF position and its 3x3 ECEF covariance at time t.
    pub fn predict(&self, t: f64) -> (Ecef, Matrix3<f64>) {
        let dt = t - self.t;
        let (pred, c) = self.mixed_prediction(if dt.is_finite() { dt } else { 0.0 });
        let (x, p) = combine(&pred, &c);
        self.to_ecef(&x, &p)
    }

    pub fn update_fix(&mut self, fix: &Fix, t: f64) -> Gate {
        if !fix.pos.iter().all(|v| v.is_finite()) || !fix.cov.iter().all(|v| v.is_finite()) {
            return Gate::Skipped;
        }
        let l = self.frame.to_enu(&fix.pos);
        let rc = self.rot * fix.cov * self.rot.transpose();
        let r = DMatrix::from_fn(3, 3, |i, j| 0.5 * (rc[(i, j)] + rc[(j, i)]));
        let obs = PosObs { z: Vector3::new(l[0], l[1], l[2]), r };
        self.update(&obs, t, true)
    }

    /// `meas[i].t` are arrival times (s) in one common timebase; `cov` their
    /// k x k covariance (s^2); `t` the unix time of the transmission.
    pub fn update_tdoa(&mut self, meas: &[Meas], cov: &DMatrix<f64>, alt: Option<(f64, f64)>, t: f64) -> Gate {
        match self.tdoa_obs(meas, cov, alt, t) {
            Some(obs) => self.update(&obs, t, false),
            None => Gate::Skipped,
        }
    }

    /// The TDOA observation, or None if the update does not apply.
    fn tdoa_obs(&self, meas: &[Meas], cov: &DMatrix<f64>, alt: Option<(f64, f64)>, t: f64) -> Option<TdoaObs> {
        let k = meas.len();
        if k < 2 || cov.nrows() != k || cov.ncols() != k || t - self.t > TDOA_MAX_AGE {
            return None;
        }
        if meas.iter().any(|m| !m.t.is_finite() || !m.rx.iter().all(|v| v.is_finite())) || !cov.iter().all(|v| v.is_finite()) {
            return None;
        }
        if alt.is_some_and(|(h, s)| !h.is_finite() || !s.is_finite() || s <= 0.0) {
            return None;
        }
        let m = k - 1 + usize::from(alt.is_some());
        let mut z = DVector::zeros(m);
        let mut r = DMatrix::zeros(m, m);
        let c2 = C_AIR * C_AIR;
        for i in 1..k {
            z[i - 1] = (meas[i].t - meas[0].t) * C_AIR;
            for j in 1..k {
                // (D cov D^T)_ij with D = [-1 | I]: the first arrival's error
                // is in every difference, so they are correlated even when the
                // arrival times are not.
                r[(i - 1, j - 1)] = c2 * (cov[(i, j)] - cov[(i, 0)] - cov[(0, j)] + cov[(0, 0)]);
            }
        }
        if let Some((h, s)) = alt {
            z[m - 1] = h;
            r[(m - 1, m - 1)] = s * s;
        }
        Some(TdoaObs { frame: self.frame, rot: self.rot, rx: meas.iter().map(|m| m.rx).collect(), z, r, alt: alt.is_some() })
    }

    /// Ground velocity (east, north m/s at the aircraft) with a 1-sigma per
    /// component, e.g. from Comm-B BDS 5,0. Gated like any update, so a
    /// mis-inferred register is rejected; it never counts as a fix.
    pub fn update_velocity(&mut self, ve: f64, vn: f64, sigma: f64, t: f64) -> Gate {
        if !(ve.is_finite() && vn.is_finite() && sigma.is_finite() && sigma > 0.0) {
            return Gate::Skipped;
        }
        let obs = VelObs {
            frame: self.frame,
            rot: self.rot,
            z: DVector::from_vec(vec![ve, vn]),
            r: DMatrix::from_diagonal_element(2, 2, sigma * sigma),
        };
        self.update(&obs, t, false)
    }

    pub fn confirmed(&self) -> bool {
        self.confirmed
    }

    pub fn stale(&self, now: f64) -> bool {
        now - self.t > STALE_AGE
    }

    pub fn position(&self) -> Ecef {
        let (x, _) = combine(&self.models, &self.mu);
        self.frame.to_ecef(&[x[0], x[1], x[2]])
    }

    pub fn ground_speed_kt(&self) -> f64 {
        let v = self.local_velocity();
        v[0].hypot(v[1]) * MS_TO_KT
    }

    pub fn track_deg(&self) -> f64 {
        let v = self.local_velocity();
        v[0].atan2(v[1]).to_degrees().rem_euclid(360.0)
    }

    pub fn vrate_fpm(&self) -> f64 {
        self.local_velocity()[2] * M_TO_FT * 60.0
    }

    pub fn err_h(&self) -> f64 {
        let (x, p) = combine(&self.models, &self.mu);
        let (pos, cov) = self.to_ecef(&x, &p);
        let local = Enu::at(pos);
        let rl = Matrix3::from_fn(|i, j| local.r[i][j]);
        let c = rl * cov * rl.transpose();
        (c[(0, 0)] + c[(1, 1)]).max(0.0).sqrt()
    }

    /// Number of accepted updates (fixes + TDOA).
    pub fn updates(&self) -> u32 {
        self.updates
    }

    /// Velocity in the ENU frame at the current position, m/s.
    fn local_velocity(&self) -> Vector3<f64> {
        let (x, _) = combine(&self.models, &self.mu);
        let v_ecef = self.rot.transpose() * Vector3::new(x[3], x[4], x[5]);
        let pos = self.frame.to_ecef(&[x[0], x[1], x[2]]);
        let local = Enu::at(pos);
        Matrix3::from_fn(|i, j| local.r[i][j]) * v_ecef
    }

    fn to_ecef(&self, x: &Vector6<f64>, p: &Matrix6<f64>) -> (Ecef, Matrix3<f64>) {
        let pos = self.frame.to_ecef(&[x[0], x[1], x[2]]);
        let pp = p.fixed_view::<3, 3>(0, 0).into_owned();
        (pos, self.rot.transpose() * pp * self.rot)
    }

    /// Markov transition probabilities over dt: pi[i][j] = P(model j at the
    /// end | model i at the start) = exp(G dt) for the generator G built from
    /// RATES (exact for any dt, which matters: updates arrive 0.1..30 s
    /// apart).
    fn transition(&self, dt: f64) -> [[f64; N]; N] {
        if !self.imm || !dt.is_finite() {
            return std::array::from_fn(|i| std::array::from_fn(|j| f64::from(u8::from(i == j))));
        }
        let g = Matrix3::from_fn(|i, j| if i == j { -RATES[i].iter().sum::<f64>() } else { RATES[i][j] });
        let pi = (g * dt.abs()).exp();
        std::array::from_fn(|i| {
            // Clamp rounding and renormalise each row.
            let row: [f64; N] = std::array::from_fn(|j| pi[(i, j)].max(0.0));
            let s: f64 = row.iter().sum();
            row.map(|v| v / s)
        })
    }

    /// IMM interaction + per-model prediction over dt. Returns the predicted
    /// models and the predicted model probabilities c_j.
    fn mixed_prediction(&self, dt: f64) -> ([Model; N], [f64; N]) {
        let pi = self.transition(dt);
        let c: [f64; N] = std::array::from_fn(|j| (0..N).map(|i| pi[i][j] * self.mu[i]).sum());
        let out = std::array::from_fn(|j| {
            let mut m = self.models[j].clone();
            if c[j] > 1e-300 {
                // Mixed initial condition for model j.
                let w: [f64; N] = std::array::from_fn(|i| pi[i][j] * self.mu[i] / c[j]);
                m = mix(&self.models, &w);
            }
            predict_cv(&m, dt, &SIGMA_A[j])
        });
        (out, c)
    }

    /// The common IMM update: predict, gate, update every model, re-weight.
    fn update(&mut self, obs: &dyn Observation, t: f64, is_fix: bool) -> Gate {
        let dt = t - self.t;
        if !dt.is_finite() || dt < -1e-6 {
            // Out of order: the state has already moved past t.
            return Gate::Skipped;
        }
        let dt = dt.max(0.0);
        let (pred, c) = self.mixed_prediction(dt);
        let ups: [Option<Upd>; N] = std::array::from_fn(|j| if c[j] > 1e-12 { iekf(&pred[j], obs) } else { None });
        let Some(d2) = ups.iter().flatten().map(|u| u.d2).min_by(f64::total_cmp) else {
            return Gate::Skipped;
        };
        if d2 > chi2_q999(obs.dim() as i32) {
            return Gate::Rejected(d2);
        }

        // Model probabilities: mu_j ~ c_j * likelihood_j, in the log domain.
        let ln_w: [f64; N] = std::array::from_fn(|j| ups[j].as_ref().map_or(f64::NEG_INFINITY, |u| c[j].ln() + u.ln_lik));
        let top = ln_w.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let w: [f64; N] = ln_w.map(|l| (l - top).exp());
        let sum: f64 = w.iter().sum();
        // A model whose update was undefined keeps its prediction (and gets
        // no weight).
        for (j, (u, p)) in ups.into_iter().zip(pred).enumerate() {
            self.models[j] = u.map_or(p, |u| u.model);
        }
        if sum > 0.0 && sum.is_finite() {
            self.mu = w.map(|v| v / sum);
        }
        if !self.imm {
            self.mu = std::array::from_fn(|j| f64::from(u8::from(j == NOMINAL)));
        }
        self.t = t;
        self.updates += 1;
        self.recent.push_back((t, is_fix));
        while self.recent.front().is_some_and(|&(ti, _)| ti < t - CONFIRM_WINDOW) {
            self.recent.pop_front();
        }
        if self.recent.len() >= 3 && self.recent.iter().filter(|r| r.1).count() >= 2 {
            self.confirmed = true;
        }
        Gate::Accepted
    }

    /// Per-model gate statistic of a TDOA group and the predicted model
    /// probabilities, without updating (tests only).
    #[cfg(test)]
    fn probe_tdoa(&self, meas: &[Meas], cov: &DMatrix<f64>, alt: Option<(f64, f64)>, t: f64) -> Option<([f64; N], [f64; N])> {
        let obs = self.tdoa_obs(meas, cov, alt, t)?;
        let (pred, c) = self.mixed_prediction(t - self.t);
        let d2 = std::array::from_fn(|j| iekf(&pred[j], &obs).map_or(f64::INFINITY, |u| u.d2));
        Some((d2, c))
    }

    /// The plain single-model EKF with the specified process noise, for
    /// comparison in tests.
    #[cfg(test)]
    fn without_imm(mut self) -> Track {
        self.imm = false;
        self.mu = std::array::from_fn(|j| f64::from(u8::from(j == NOMINAL)));
        self
    }
}

/// Constant-velocity prediction with piecewise-constant white acceleration:
/// per axis Q = sigma_a^2 [dt^4/4, dt^3/2; dt^3/2, dt^2].
fn predict_cv(m: &Model, dt: f64, sigma_a: &[f64; 3]) -> Model {
    let mut f = Matrix6::identity();
    for i in 0..3 {
        f[(i, i + 3)] = dt;
    }
    let mut q = Matrix6::zeros();
    // Q = sigma^2 G G^T with G = [dt^2/2, dt]: positive semi-definite for
    // either sign of dt (predict() may look slightly back in time).
    let (dt2, dt3, dt4) = (dt * dt, dt * dt * dt, dt * dt * dt * dt);
    for (i, s) in sigma_a.iter().enumerate() {
        let s2 = s * s;
        q[(i, i)] = s2 * dt4 / 4.0;
        q[(i, i + 3)] = s2 * dt3 / 2.0;
        q[(i + 3, i)] = s2 * dt3 / 2.0;
        q[(i + 3, i + 3)] = s2 * dt2;
    }
    let p = f * m.p * f.transpose() + q;
    Model { x: f * m.x, p: 0.5 * (p + p.transpose()) }
}

/// Iterated EKF update of one model. The gate statistic and the likelihood
/// come from the innovation at the prediction (the standard choice: it is
/// the quantity whose distribution the prediction actually states); the
/// iterations only refine where the update lands.
fn iekf(pred: &Model, obs: &dyn Observation) -> Option<Upd> {
    let m = obs.dim();
    let p = DMatrix::from_fn(6, 6, |i, j| pred.p[(i, j)]);
    let xp = DVector::from_fn(6, |i, _| pred.x[i]);
    let (r0, h0) = obs.linearise(&pred.x)?;
    let s0 = &h0 * &p * h0.transpose() + obs.r();
    let ch0 = s0.clone().cholesky()?;
    let d2 = r0.dot(&ch0.solve(&r0));
    let ln_det: f64 = 2.0 * (0..m).map(|i| ch0.l_dirty()[(i, i)].ln()).sum::<f64>();
    let ln_lik = -0.5 * (d2 + ln_det + m as f64 * (2.0 * std::f64::consts::PI).ln());

    let (mut resid, mut h, mut ch) = (r0, h0, ch0);
    let mut xi = xp.clone();
    let mut k;
    let mut it = 0;
    loop {
        // K = P H^T S^-1, via the Cholesky factor of S (S symmetric).
        let pht = &p * h.transpose();
        k = ch.solve(&pht.transpose()).transpose();
        // IEKF step: x_{i+1} = x_pred + K (z - h(x_i) - H (x_pred - x_i)).
        let x_next = &xp + &k * (&resid + &h * (&xi - &xp));
        let moved = (&x_next - &xi).rows(0, 3).norm();
        xi = x_next;
        it += 1;
        if it >= obs.iterations() || moved < 0.5 {
            break;
        }
        let xv = Vector6::from_fn(|i, _| xi[i]);
        let Some((r1, h1)) = obs.linearise(&xv) else { break };
        let Some(c1) = (&h1 * &p * h1.transpose() + obs.r()).cholesky() else { break };
        (resid, h, ch) = (r1, h1, c1);
    }
    // Joseph form: stays symmetric positive semi-definite under rounding.
    let ikh = DMatrix::identity(6, 6) - &k * &h;
    let pn = &ikh * &p * ikh.transpose() + &k * obs.r() * k.transpose();
    let x = Vector6::from_fn(|i, _| xi[i]);
    let pn = Matrix6::from_fn(|i, j| 0.5 * (pn[(i, j)] + pn[(j, i)]));
    if !x.iter().chain(pn.iter()).all(|v| v.is_finite()) {
        return None;
    }
    Some(Upd { model: Model { x, p: pn }, d2, ln_lik })
}

/// Moment-matched mixture of models with weights w (summing to 1).
fn mix(models: &[Model; N], w: &[f64; N]) -> Model {
    let x = (0..N).fold(Vector6::zeros(), |a, i| a + models[i].x * w[i]);
    let mut p = Matrix6::zeros();
    for (m, wi) in models.iter().zip(w) {
        let d = m.x - x;
        p += (m.p + d * d.transpose()) * *wi;
    }
    Model { x, p }
}

/// The IMM output estimate: the mixture under the model probabilities.
fn combine(models: &[Model; N], w: &[f64; N]) -> (Vector6<f64>, Matrix6<f64>) {
    let s: f64 = w.iter().sum();
    let w = if s > 0.0 { w.map(|v| v / s) } else { std::array::from_fn(|j| f64::from(u8::from(j == NOMINAL))) };
    let m = mix(models, &w);
    (m.x, m.p)
}

#[cfg(test)]
mod tests;
