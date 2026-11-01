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
//!
//! Noise level. Each pair tracks its normalised innovation (NIS) against the
//! nominal noise of its two clock types. It used to be clamped at 1, so a pair
//! could only be found noisier than nominal; now it may also go below, to
//! `NIS_MIN` (a tenth of the nominal variance). The EWMA starts at 1 and moves
//! ~2% per observation, so a pair is credited with less noise only after
//! ~50 of its own observations say so. A receiver's noise scale (the factor
//! on its per-copy jitter in normalisation and the solver) comes from all its
//! converged pairs through `receiver_noise_scale`: worse than nominal as
//! before, better than nominal only as far as the number of its pairs backs
//! it (shrinkage towards nominal, see there). On the 2026-10-25 replays this
//! calibration (with the 250 ns dump1090 jitter, types.rs) took published p50
//! from 116 to 101 m (10-minute capture) and brought the solver's error
//! estimate close to the real error.
//!
//! Re-anchoring (2026-11-01). Most offset resets are not clock jumps: every
//! new mlat-client session announces its receiver input twice ("input
//! connected", then a synthesised "Decoder mode changed" clock_reset;
//! mlat-client coordinator.py server_connected, receiver.py
//! start_connection), whether or not the readsb behind it restarted. So a
//! reset sets a converged model aside instead of throwing it away; the
//! observations after it decide (`reset_offset`, `observe`).

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
/// Smallest: a tenth of the nominal variance (~80 ns per dump1090 receiver,
/// still well above the 24 ns a 12 MHz counter's quantisation allows). In
/// the 2026-10-25 study 0.1 and 0.25 gave the same accuracy.
pub const NIS_MIN: f64 = 0.1;
/// Converged pairs a receiver needs before its noise scale moves at all.
const RX_MIN_PAIRS: usize = 3;
/// Prior weight of the nominal noise, in pairs, when a receiver looks
/// better than nominal (see `receiver_noise_scale`).
const RX_PRIOR_PAIRS: f64 = 3.0;
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
/// Observations after an offset reset that must all pass the gate of the
/// model it set aside before that model comes back: at least two sync
/// points (each gives a pair two observations), against the 10 a fresh
/// offset needs.
pub const REANCHOR_AGREE: u32 = 4;

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
    /// The current state started from a kept drift (so it is worth keeping
    /// again, even before the offset has converged).
    drift_kept: bool,
    /// The converged model the last offset reset set aside, and how many
    /// observations since then passed its gate (`reset_offset`).
    held: Option<Box<(PairModel, u32)>>,
    /// EWMA of the normalised innovation squared against the NOMINAL noise:
    /// ~1 when the receivers are as good as their clock type says, larger
    /// when they are noisier, smaller when better. The measurement noise used
    /// is r * nis (clamped to [NIS_MIN, NIS_MAX]): production showed many
    /// pairs far noisier than nominal, which a fixed r turned into rejection
    /// runs and resets, and most of them quieter than nominal, which a fixed r
    /// turned into weak gates and over-wide covariances.
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
    /// The observations since the last offset reset agreed with the model it
    /// had set aside: that model is back (this observation included).
    Reanchored,
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
            drift_kept: false,
            held: None,
            nis: 1.0,
            rej_nu: Vec::with_capacity(RESET_AFTER as usize),
            drift_rejects_in_row: 0,
        }
    }

    /// Effective measurement variance: nominal, scaled by observed noise.
    fn r_eff(&self) -> f64 {
        self.r * self.nis.clamp(NIS_MIN, NIS_MAX)
    }

    /// Observed noise relative to nominal, as a sigma factor (1 = as
    /// specified, < 1 quieter, > 1 noisier).
    pub fn noise_scale(&self) -> f64 {
        self.nis.clamp(NIS_MIN, NIS_MAX).sqrt()
    }

    fn init(&mut self, s_a: f64, theta: f64, wall: f64) {
        self.base = theta;
        self.s_ref = s_a;
        self.drift_kept = self.keep_drift.is_some();
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
        // A model set aside by an offset reset: this observation passes its
        // (mature) gate and updates it, or shows the counter moved and ends
        // it. One older than MAX_AGE would not be usable anyway: it would
        // come back on a few observations as a fresh one with a long record.
        if let Some(h) = self.held.as_mut() {
            if wall - h.0.updated_wall <= MAX_AGE && h.0.observe(s_a, theta, wall) == Outcome::Accepted {
                h.1 += 1;
                if h.1 >= REANCHOR_AGREE {
                    let resets = self.resets;
                    *self = self.held.take().unwrap().0;
                    self.resets = resets;
                    return Outcome::Reanchored;
                }
            } else {
                self.held = None;
            }
        }
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

    /// a's clock time at the instant b's clock reads `s_b` (the inverse of
    /// `predict`, whose model is a function of a's time). A fixed point:
    /// theta changes by at most |drift| <= ~2e-4 per second of a's time, so
    /// each iteration shrinks the error ~5000-fold; the first guess can be off
    /// by drift x offset (offsets reach 1e6 s), four more reach the ns level.
    pub fn a_time_at(&self, s_b: f64) -> f64 {
        let mut s_a = s_b - self.predict(s_b).0;
        for _ in 0..4 {
            s_a = s_b - self.predict(s_a).0;
        }
        s_a
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
    ///
    /// A converged model is set aside, not lost: while the fresh offset
    /// converges as before, each new observation is also checked against it
    /// (`observe`). If the first `REANCHOR_AGREE` all pass its gate, the
    /// counter did not move (a reconnect, not a restart) and it comes back
    /// whole, usable at once; the first that does not pass discards it. A
    /// reset before that keeps it and starts the count again: a session's two
    /// announcements arrive back to back, sometimes with observations from
    /// the receiver's Beast feed in between. Such a reset keeps the drift too
    /// (it used to be lost once the fresh offset had taken it).
    pub fn reset_offset(&mut self) {
        if self.updates >= MIN_UPDATES || (self.updates > 0 && self.drift_kept) {
            self.keep_drift = Some((self.x[1], self.p[1][1]));
        }
        if self.updates >= MIN_UPDATES {
            let mut old = self.clone();
            old.held = None;
            self.held = Some(Box::new((old, 0)));
        } else if let Some(h) = self.held.as_mut() {
            h.1 = 0;
        }
        self.updates = 0;
        self.resets += 1;
    }
}

/// A receiver's timestamp noise relative to its clock type's nominal jitter,
/// from the noise scales of its converged clock pairs.
///
/// A pair's noise mixes both receivers, but averaged over all of a
/// receiver's pairs it mostly reflects the receiver itself. With fewer than
/// `RX_MIN_PAIRS` pairs nothing is concluded (1). Worse than nominal (mean >=
/// 1): the mean, as before. Better than nominal: a lower noise makes every
/// copy of this receiver weigh more, so the claim must be backed by evidence:
/// the mean is shrunk towards 1 as if `RX_PRIOR_PAIRS` more pairs had said
/// "nominal". A receiver whose 3 pairs say 0.5 gets 0.75; with 30 pairs it
/// gets 0.55. Never below sqrt(NIS_MIN), the floor a single pair can reach.
pub fn receiver_noise_scale(pair_scales: &[f64]) -> f64 {
    let n = pair_scales.len();
    if n < RX_MIN_PAIRS {
        return 1.0;
    }
    let mean = pair_scales.iter().sum::<f64>() / n as f64;
    if mean >= 1.0 {
        return mean;
    }
    let shrunk = (n as f64 * mean + RX_PRIOR_PAIRS) / (n as f64 + RX_PRIOR_PAIRS);
    shrunk.max(NIS_MIN.sqrt())
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

    /// Feed a dump1090 pair with Gaussian noise of `scale` x nominal.
    fn pair_with_noise(scale: f64, n: usize) -> PairModel {
        let mut m = PairModel::new(ClockType::Dump1090, ClockType::Dump1090, 0);
        let mut seed = 0x0bad_cafe_1234_5678u64;
        let sigma = scale * (2.0f64).sqrt() * ClockType::Dump1090.jitter();
        let mut s = 1000.0;
        for i in 0..n {
            s += 0.5;
            m.observe(s, 42.0 + 3e-6 * (s - 1000.0) + sigma * gauss(&mut seed), i as f64 * 0.5);
        }
        m
    }

    #[test]
    fn pair_noise_can_go_below_nominal_with_evidence() {
        // Young: the estimate starts at nominal and has not moved far yet.
        let young = pair_with_noise(0.5, 12);
        assert!(young.noise_scale() > 0.8, "young {}", young.noise_scale());
        // After a few hundred observations it follows the evidence (0.5).
        let old = pair_with_noise(0.5, 600);
        assert!((old.noise_scale() - 0.5).abs() < 0.12, "old {}", old.noise_scale());
        // Never below the floor, however quiet the pair.
        let quiet = pair_with_noise(0.05, 600);
        assert!((quiet.noise_scale() - NIS_MIN.sqrt()).abs() < 1e-9, "quiet {}", quiet.noise_scale());
        // And still found noisier than nominal when it is.
        let noisy = pair_with_noise(2.0, 600);
        assert!(noisy.noise_scale() > 1.6, "noisy {}", noisy.noise_scale());
    }

    #[test]
    fn receiver_floor_follows_its_own_evidence() {
        // Too few pairs: no conclusion either way.
        assert_eq!(receiver_noise_scale(&[0.4, 0.4]), 1.0);
        assert_eq!(receiver_noise_scale(&[3.0, 3.0]), 1.0);
        // Noisier than nominal: the plain mean, as before.
        assert!((receiver_noise_scale(&[2.0, 2.0, 2.0]) - 2.0).abs() < 1e-12);
        // Quieter: 3 pairs move it only half way, 30 pairs nearly all the way.
        let few = receiver_noise_scale(&[0.5; 3]);
        let many = receiver_noise_scale(&[0.5; 30]);
        assert!((few - 0.75).abs() < 1e-12, "few {few}");
        assert!((many - (30.0 * 0.5 + 3.0) / 33.0).abs() < 1e-12, "many {many}");
        assert!(many < few && few < 1.0);
        // Never below what a single pair can reach.
        assert!((receiver_noise_scale(&[NIS_MIN.sqrt(); 1000]) - NIS_MIN.sqrt()).abs() < 5e-3);
        assert!(receiver_noise_scale(&[0.0; 1000]) >= NIS_MIN.sqrt());
    }

    #[test]
    fn a_time_inverts_predict() {
        let mut m = PairModel::new(ClockType::Dump1090, ClockType::Dump1090, 0);
        // b runs 1.2e6 s ahead of a and 80 ppm fast.
        let mut s = 5000.0;
        for i in 0..60 {
            s += 0.5;
            m.observe(s, 1.2e6 + 80e-6 * (s - 5000.0), i as f64);
        }
        let s_a = s + 3.0;
        let s_b = s_a + m.predict(s_a).0;
        assert!((m.a_time_at(s_b) - s_a).abs() < 1e-9, "{}", m.a_time_at(s_b) - s_a);
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

    /// A converged dump1090 pair: theta = 12_345.6789 + 37 ppm x (s - 1e6),
    /// nominal noise, observed every ~0.5 s from wall time 0. Returns the
    /// pair, its last clock time and the noise generator.
    fn converged() -> (PairModel, f64, u64) {
        let mut m = PairModel::new(ClockType::Dump1090, ClockType::Dump1090, 0);
        let mut seed = 0x5eed_cafe_f00d_c10c_u64;
        let mut s = 1_000_000.0;
        for i in 0..120 {
            s += 0.5;
            m.observe(s, theta_at(s) + 0.35e-6 * gauss(&mut seed), i as f64 * 0.5);
        }
        assert!(m.usable(60.0));
        (m, s, seed)
    }

    fn theta_at(s: f64) -> f64 {
        12_345.678_9 + 37e-6 * (s - 1_000_000.0)
    }

    #[test]
    fn a_counter_that_did_not_move_is_reanchored() {
        let (mut m, mut s, mut seed) = converged();
        let before = m.updates;
        // A reconnect 8 s later: the session's two announcements, the
        // receiver's counter running on.
        m.reset_offset();
        m.reset_offset();
        assert!(!m.usable(68.0));
        let mut out = Vec::new();
        for i in 0..REANCHOR_AGREE {
            s += 0.5;
            out.push(m.observe(s + 8.0, theta_at(s + 8.0) + 0.35e-6 * gauss(&mut seed), 68.0 + i as f64 * 0.5));
        }
        assert_eq!(out.last(), Some(&Outcome::Reanchored), "{out:?}");
        assert!(out[..out.len() - 1].iter().all(|o| *o == Outcome::Accepted), "{out:?}");
        // The whole model is back: usable at once, with its record.
        assert!(m.usable(70.0));
        assert_eq!(m.updates, before + REANCHOR_AGREE);
        assert_eq!(m.resets, 2);
        let (pred, var) = m.predict(s + 9.0);
        assert!((pred - theta_at(s + 9.0)).abs() < 0.3e-6 && var.sqrt() < 0.3e-6, "{:e} {:e}", pred - theta_at(s + 9.0), var.sqrt());
    }

    #[test]
    fn a_counter_that_moved_is_never_reanchored() {
        let (mut m, mut s, mut seed) = converged();
        m.reset_offset();
        // readsb restarted with the client: b's counter is 3.7 s behind
        // where it was. Its crystal is the same, so the drift still holds.
        let moved = |s: f64| theta_at(s) - 3.7;
        let mut outcomes = Vec::new();
        for i in 0..40 {
            s += 0.5;
            outcomes.push(m.observe(s, moved(s) + 0.35e-6 * gauss(&mut seed), 70.0 + i as f64 * 0.5));
            if i == 0 {
                assert!(m.held.is_none(), "the first observation shows the jump");
            }
        }
        assert!(!outcomes.contains(&Outcome::Reanchored));
        assert!(outcomes.iter().all(|o| *o == Outcome::Accepted), "{outcomes:?}");
        assert!(m.usable(90.0));
        assert!((m.predict(s + 1.0).0 - moved(s + 1.0)).abs() < 0.5e-6);
        assert!((m.drift_ppm() - 37.0).abs() < 0.5, "drift {}", m.drift_ppm());
    }

    #[test]
    fn a_stale_set_aside_model_does_not_come_back() {
        let (mut m, mut s, mut seed) = converged();
        m.reset_offset();
        // Observations again only after MAX_AGE (from wall 60): the pair
        // starts over as a fresh one, even though the counter did not move.
        let mut outcomes = Vec::new();
        for i in 0..12 {
            s += 0.5;
            outcomes.push(m.observe(s + 50.0, theta_at(s + 50.0) + 0.35e-6 * gauss(&mut seed), 110.0 + i as f64 * 0.5));
        }
        assert!(!outcomes.contains(&Outcome::Reanchored), "{outcomes:?}");
        assert!(m.held.is_none());
        assert_eq!(m.updates, 12);
    }

    #[test]
    fn a_second_reset_keeps_the_drift() {
        let (mut m, mut s, mut seed) = converged();
        // Reset, the counter moved (the set-aside model goes), three
        // observations of the fresh offset, then another reset before it
        // converged: the drift is still the pair's, not a guess of 0.
        m.reset_offset();
        for i in 0..3 {
            s += 0.5;
            m.observe(s, theta_at(s) + 2.0 + 0.35e-6 * gauss(&mut seed), 70.0 + i as f64 * 0.5);
        }
        assert!(m.held.is_none());
        m.reset_offset();
        s += 0.5;
        m.observe(s, theta_at(s) + 5.0, 72.0);
        assert!((m.drift_ppm() - 37.0).abs() < 0.5, "drift {}", m.drift_ppm());
    }
}
