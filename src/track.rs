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

use nalgebra::{DMatrix, Matrix3};

use crate::geo::Ecef;
use crate::solver::{Fix, Meas};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gate {
    Accepted,
    /// Mahalanobis distance squared of the rejected innovation.
    Rejected(f64),
    /// Not applicable (too few measurements, track too old, singular).
    Skipped,
}

#[derive(Debug, Clone)]
pub struct Track {
    // implementation-defined
    _private: (),
}

impl Track {
    pub fn new(_fix: &Fix, _t: f64) -> Track {
        todo!()
    }

    /// Time of the last accepted update (unix seconds).
    pub fn last_update(&self) -> f64 {
        todo!()
    }

    /// Predicted ECEF position and its 3x3 ECEF covariance at time t.
    pub fn predict(&self, _t: f64) -> (Ecef, Matrix3<f64>) {
        todo!()
    }

    pub fn update_fix(&mut self, _fix: &Fix, _t: f64) -> Gate {
        todo!()
    }

    /// `meas[i].t` are arrival times (s) in one common timebase; `cov` their
    /// k x k covariance (s^2); `t` the unix time of the transmission.
    pub fn update_tdoa(&mut self, _meas: &[Meas], _cov: &DMatrix<f64>, _alt: Option<(f64, f64)>, _t: f64) -> Gate {
        todo!()
    }

    pub fn confirmed(&self) -> bool {
        todo!()
    }

    pub fn stale(&self, _now: f64) -> bool {
        todo!()
    }

    pub fn position(&self) -> Ecef {
        todo!()
    }

    pub fn ground_speed_kt(&self) -> f64 {
        todo!()
    }

    pub fn track_deg(&self) -> f64 {
        todo!()
    }

    pub fn vrate_fpm(&self) -> f64 {
        todo!()
    }

    pub fn err_h(&self) -> f64 {
        todo!()
    }

    /// Number of accepted updates (fixes + TDOA).
    pub fn updates(&self) -> u32 {
        todo!()
    }
}
