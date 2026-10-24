//! Pairwise receiver clock models.
//!
//! For every pair of receivers (a, b) that hear the same ADS-B transmissions
//! we track theta = s_b - s_a, the difference between their clock readings
//! (seconds) for one and the same instant, as a function of a's time. Each
//! ADS-B position message with a known transmitter position gives one direct
//! observation of theta once the propagation delays are removed.
//!
//! The model is a 2-state Kalman filter (offset, drift) instead of
//! mlat-server's windowed fit plus PI drift loop. It handles irregular
//! sampling natively, its predicted variance grows with time since the last
//! update (so a stale pair automatically weighs less), and that variance is
//! exactly what the group normalisation needs as edge weight.
//!
//! Numerics: clock readings reach ~2e7 s (48-bit counter at 12 MHz), where an
//! f64 still resolves ~4 ns. theta itself can be as large, so the filter state
//! is kept relative to `base` (theta at initialisation) and every time is
//! re-centred on the last update.

use crate::types::ClockType;

/// Random-walk frequency noise, (s/s)^2 per second: lets the drift wander by
/// ~0.5 ppm over an hour (an RTL-SDR crystal after warm-up; sharper
/// temperature steps show up as rejections and, if sustained, a reset).
const Q_FREQ: f64 = 1e-16;
/// Extra white phase noise, s^2 per second, beyond the per-reading jitter.
const Q_PHASE: f64 = 5e-17;
/// Innovation gate (squared, in sigmas): 4 sigma.
const GATE2: f64 = 16.0;
/// Before the pair has converged the gate is wider, or a correct but young
/// model could reject everything it sees.
const GATE2_YOUNG: f64 = 64.0;
/// Consecutive rejections that may mean "this clock jumped" (then checked
/// for coherence before starting over).
const RESET_AFTER: u32 = 6;
/// Largest noise scale a pair may adopt (variance factor; 25 = 5x sigma).
const NIS_MAX: f64 = 25.0;
/// EWMA weight of the noise-level tracker (~50 observations).
const NIS_ALPHA: f64 = 0.02;
/// Updates (observations) before the pair is usable for multilateration.
/// 10, not 6: in production (2026-10-24) young pairs that had just been
/// reset were the ones feeding the worst normalisations.
pub const MIN_UPDATES: u32 = 10;
/// A pair whose predicted offset sigma exceeds this is not used.
pub const MAX_SIGMA: f64 = 1.5e-6;
/// Not updated for this long: the pair is not used (and is expired later).
pub const MAX_AGE: f64 = 45.0;

#[derive(Debug, Clone)]
pub struct PairModel {
    /// theta at initialisation; state x0 is relative to it.
    base: f64,
    /// a's clock time (s) the state refers to.
    s_ref: f64,
    /// [offset relative to base (s), drift (s/s)]
    x: [f64; 2],
    /// covariance of x
    p: [[f64; 2]; 2],
    /// measurement variance per observation (both receivers' jitter)
    r: f64,
    /// limit on |drift|
    max_drift: f64,
    pub updates: u32,
    pub rejects_in_row: u32,
    pub rejects: u32,
    pub resets: u32,
    /// wall-clock time of the last accepted update (unix s)
    pub updated_wall: f64,
    /// exponentially weighted rejection rate, for receiver health
    pub reject_rate: f64,
    /// distance band (0..=3), for the per-receiver pair budget
    pub band: u8,
    /// (drift, its variance) to keep across the next init (offset reset)
    keep_drift: Option<(f64, f64)>,
    /// EWMA of the normalised innovation squared against the NOMINAL noise:
    /// ~1 when the receivers are as good as their clock type says, larger
    /// when they are noisier. The measurement noise used is r * nis (clamped):
    /// production showed many pairs far noisier than the nominal 0.5 us,
    /// which a fixed r turned into rejection runs and resets.
    nis: f64,
    /// Innovations of the current run of rejections, to tell a real clock
    /// jump (they agree with each other) from plain noise (they don't).
    rej_nu: Vec<f64>,
    /// Consecutive updates refused because they implied an impossible drift.
    drift_rejects_in_row: u32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Outcome {
    Accepted,
    Rejected,
    /// The pair restarted after a run of rejections (a clock jump).
    Reset,
    /// The pair restarted because its drift estimate left the physical range.
    ResetDrift,
}

impl PairModel {
    pub fn new(ca: ClockType, cb: ClockType, band: u8) -> PairModel {
        PairModel {
            base: 0.0,
            s_ref: 0.0,
            x: [0.0, 0.0],
            p: [[0.0; 2]; 2],
            r: ca.jitter().powi(2) + cb.jitter().powi(2),
            max_drift: 1.5 * (ca.max_freq_error() + cb.max_freq_error()),
            updates: 0,
            rejects_in_row: 0,
            rejects: 0,
            resets: 0,
            updated_wall: 0.0,
            reject_rate: 0.0,
            band,
            keep_drift: None,
            nis: 1.0,
            rej_nu: Vec::with_capacity(RESET_AFTER as usize),
            drift_rejects_in_row: 0,
        }
    }

    /// Effective measurement variance: nominal, scaled by observed noise.
    fn r_eff(&self) -> f64 {
        self.r * self.nis.clamp(1.0, NIS_MAX)
    }

    /// How much noisier than nominal this pair is (1 = as specified).
    pub fn noise_scale(&self) -> f64 {
        self.nis.clamp(1.0, NIS_MAX).sqrt()
    }

    fn init(&mut self, s_a: f64, theta: f64, wall: f64) {
        self.base = theta;
        self.s_ref = s_a;
        let (drift, pd) = self.keep_drift.take().unwrap_or((0.0, self.max_drift * self.max_drift));
        self.x = [0.0, drift];
        self.p = [[self.r_eff(), 0.0], [0.0, pd]];
        self.updates = 1;
        self.rejects_in_row = 0;
        self.rej_nu.clear();
        self.drift_rejects_in_row = 0;
        self.updated_wall = wall;
    }

    /// Propagate the state to a's time `s_a`; returns (x, P) without storing.
    fn propagate(&self, s_a: f64) -> ([f64; 2], [[f64; 2]; 2]) {
        let dt = s_a - self.s_ref;
        let adt = dt.abs();
        let x = [self.x[0] + self.x[1] * dt, self.x[1]];
        let p = &self.p;
        // F P F^T with F = [[1, dt], [0, 1]]
        let p00 = p[0][0] + 2.0 * dt * p[0][1] + dt * dt * p[1][1];
        let p01 = p[0][1] + dt * p[1][1];
        let p11 = p[1][1];
        // Integrated random-walk frequency noise + white phase noise.
        let q00 = Q_FREQ * adt * adt * adt / 3.0 + Q_PHASE * adt;
        let q01 = Q_FREQ * adt * adt / 2.0 * dt.signum();
        let q11 = Q_FREQ * adt;
        (x, [[p00 + q00, p01 + q01], [p01 + q01, p11 + q11]])
    }

    /// Observe theta (s) at a's clock time s_a (s).
    pub fn observe(&mut self, s_a: f64, theta: f64, wall: f64) -> Outcome {
        if self.updates == 0 {
            self.init(s_a, theta, wall);
            return Outcome::Accepted;
        }
        let (x, p) = self.propagate(s_a);
        let nu = (theta - self.base) - x[0];
        // Noise level tracking against the nominal r, capped so one wild
        // observation cannot blow it up.
        let nis_obs = (nu * nu / (p[0][0] + self.r)).min(4.0 * NIS_MAX);
        let s = p[0][0] + self.r_eff();
        let gate = if self.updates < MIN_UPDATES { GATE2_YOUNG } else { GATE2 };
        if nu * nu > gate * s {
            self.rejects += 1;
            self.rejects_in_row += 1;
            self.reject_rate += 0.05 * (1.0 - self.reject_rate);
            self.nis += NIS_ALPHA * 0.5 * (nis_obs - self.nis);
            self.rej_nu.push(nu);
            if self.rejects_in_row >= RESET_AFTER {
                // A clock jump moves every following observation by the same
                // amount; noise scatters them. Only the first is a reason to
                // start over (mlat-server resets on any 77-point streak).
                let lo = self.rej_nu.iter().cloned().fold(f64::INFINITY, f64::min);
                let hi = self.rej_nu.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                let mean = self.rej_nu.iter().sum::<f64>() / self.rej_nu.len() as f64;
                let coherent = hi - lo <= (6.0 * self.r_eff().sqrt()).max(0.2 * mean.abs());
                self.rej_nu.clear();
                self.rejects_in_row = 0;
                if coherent {
                    self.resets += 1;
                    self.init(s_a, theta, wall);
                    return Outcome::Reset;
                }
            }
            return Outcome::Rejected;
        }
        let k0 = p[0][0] / s;
        let k1 = p[0][1] / s;
        let mut nx = [x[0] + k0 * nu, x[1] + k1 * nu];
        if nx[1].abs() > self.max_drift {
            // An update that would put the drift beyond what the two crystals
            // can do is bad data: refuse it, as mlat-server does. Only a
            // sustained run means the pair itself is wrong.
            self.rejects += 1;
            self.drift_rejects_in_row += 1;
            if self.drift_rejects_in_row >= RESET_AFTER {
                self.resets += 1;
                self.init(s_a, theta, wall);
                return Outcome::ResetDrift;
            }
            return Outcome::Rejected;
        }
        // Joseph-free update is fine for a 2x2 with these magnitudes.
        let np = [
            [(1.0 - k0) * p[0][0], (1.0 - k0) * p[0][1]],
            [p[0][1] - k1 * p[0][0], p[1][1] - k1 * p[0][1]],
        ];
        // Re-centre: keep the offset small relative to base.
        if nx[0].abs() > 1.0 {
            self.base += nx[0];
            nx[0] = 0.0;
        }
        self.x = nx;
        self.p = [[np[0][0], 0.5 * (np[0][1] + np[1][0])], [0.5 * (np[0][1] + np[1][0]), np[1][1]]];
        self.s_ref = s_a;
        self.updates = self.updates.saturating_add(1);
        self.rejects_in_row = 0;
        self.rej_nu.clear();
        self.drift_rejects_in_row = 0;
        self.updated_wall = wall;
        self.reject_rate -= 0.05 * self.reject_rate;
        self.nis += NIS_ALPHA * (nis_obs - self.nis);
        Outcome::Accepted
    }

    /// Predicted theta and its variance at a's clock time s_a.
    pub fn predict(&self, s_a: f64) -> (f64, f64) {
        let (x, p) = self.propagate(s_a);
        (self.base + x[0], p[0][0].max(0.0))
    }

    pub fn drift_ppm(&self) -> f64 {
        self.x[1] * 1e6
    }

    /// Usable for normalising a group at wall time `now`.
    pub fn usable(&self, now: f64) -> bool {
        self.updates >= MIN_UPDATES && now - self.updated_wall <= MAX_AGE && self.rejects_in_row < 3
    }

    /// Forget the offset but keep the drift (the counter of one receiver was
    /// reset or jumped; its crystal did not change).
    pub fn reset_offset(&mut self) {
        if self.updates >= MIN_UPDATES {
            self.keep_drift = Some((self.x[1], self.p[1][1]));
        }
        self.updates = 0;
        self.resets += 1;
    }
}

/// Distance band used by the pair budget: <50, 50-100, 100-150, >=150 km.
pub fn band_of(dist_m: f64) -> u8 {
    ((dist_m / 50e3) as u8).min(3)
}

/// Pairs a receiver may hold per band. Nearby pairs are the most valuable
/// (they share the most aircraft) and the cheapest to keep accurate.
pub const PAIR_BUDGET: [u16; 4] = [28, 24, 16, 12];

#[cfg(test)]
mod tests {
    use super::*;

    fn rng(seed: &mut u64) -> f64 {
        // xorshift, deterministic, uniform in [-0.5, 0.5)
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        (*seed as f64 / u64::MAX as f64) - 0.5
    }

    fn gauss(seed: &mut u64) -> f64 {
        // sum of 12 uniforms: approximately N(0, 1)
        (0..12).map(|_| rng(seed)).sum::<f64>()
    }

    #[test]
    fn converges_on_offset_and_drift() {
        let mut m = PairModel::new(ClockType::Dump1090, ClockType::Dump1090, 0);
        let mut seed = 0x1234_5678_9abc_def0u64;
        let offset = 12_345.678_9;
        let drift = 37e-6;
        let jitter = (2.0f64).sqrt() * 500e-9;
        let mut s = 1_000_000.0;
        for i in 0..400 {
            s += 0.4 + 0.3 * (rng(&mut seed) + 0.5);
            let theta = offset + drift * (s - 1_000_000.0) + jitter * gauss(&mut seed);
            let o = m.observe(s, theta, i as f64);
            assert_ne!(o, Outcome::Reset, "reset at {i}");
        }
        let (pred, var) = m.predict(s + 1.0);
        let truth = offset + drift * (s + 1.0 - 1_000_000.0);
        eprintln!("pred err {:e} sigma {:e} drift {}", pred - truth, var.sqrt(), m.drift_ppm());
        assert!((pred - truth).abs() < 0.5e-6, "pred err {:e}", pred - truth);
        assert!(var.sqrt() < 0.5e-6, "sigma {:e}", var.sqrt());
        assert!((m.drift_ppm() - 37.0).abs() < 0.5, "drift {}", m.drift_ppm());
    }

    #[test]
    fn rejects_outliers_and_resets_on_jump() {
        let mut m = PairModel::new(ClockType::Beast, ClockType::Beast, 0);
        let mut s = 10.0;
        for i in 0..50 {
            s += 0.5;
            m.observe(s, 5.0 + 1e-6 * s, i as f64);
        }
        // a single 30 us outlier is rejected
        s += 0.5;
        assert_eq!(m.observe(s, 5.0 + 1e-6 * s + 30e-6, 51.0), Outcome::Rejected);
        // a persistent jump of 1 ms resets the pair
        // A preceding lone outlier makes the first run of rejections
        // incoherent, so the jump is confirmed on the second run at the latest.
        let mut reset_at = None;
        for i in 0..2 * RESET_AFTER {
            s += 0.5;
            if m.observe(s, 5.001 + 1e-6 * s, 52.0 + i as f64) == Outcome::Reset {
                reset_at = Some(i);
                break;
            }
        }
        assert!(reset_at.is_some(), "no reset");
        assert_eq!(m.updates, 1);
    }

    #[test]
    fn predicted_variance_grows_with_age() {
        let mut m = PairModel::new(ClockType::Dump1090, ClockType::Dump1090, 0);
        let mut s = 0.0;
        for i in 0..100 {
            s += 0.5;
            m.observe(s, 1.0, i as f64);
        }
        let (_, v0) = m.predict(s);
        let (_, v1) = m.predict(s + 30.0);
        assert!(v1 > v0 * 2.0, "{v0:e} {v1:e}");
    }
}
