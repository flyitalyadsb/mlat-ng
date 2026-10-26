//! Receiver position calibration ("self-survey") from clock-sync data.
//!
//! Every sync observation is an ADS-B transmission from a known place heard
//! by two receivers. With the DECLARED receiver positions the engine computes
//! each copy's propagation-corrected time `t_d = t / f - |p - x| / c`. If
//! receiver R's antenna really is at x + d, then |p - x_true| ~ |p - x| -
//! u.d, with u the unit vector from the receiver towards the aircraft, so
//!
//!   (t_dK - t_dR) * c = offset_RK(t) + u_R . d_R - u_K . d_K + noise.
//!
//! The pair's clock offset (and its drift) is unknown but smooth: inside a
//! window of `WINDOW` seconds it is a line in time, eliminated in closed form
//! (variable projection, as in the 2026-10-25 prototype, which placed
//! receivers to 17 m p50 from 60 s of data). What remains is linear least
//! squares in d_R. Partners' own errors (d_K) are not modelled: with many
//! partners in different directions they average out as noise, and a window
//! that fits badly is dropped whole.
//!
//! Everything is kept as sums, so the cost is a few dozen additions per
//! observation and a 3x3 solve per question:
//! - per open window (R, K): the moments of t, z and u needed to project out
//!   the offset line;
//! - per receiver: the normal equations of d accumulated over closed
//!   windows, forgotten with time constant `TAU`.
//!
//! d is in the receiver's local east/north/up. Height is barely observable
//! (aircraft are seen near the horizon), so it carries a tight prior; the
//! horizontal prior is weak. The estimate is a diagnostic first ("bench"):
//! nothing here moves a receiver.

use std::collections::HashMap;

use nalgebra::{Matrix3, Vector3};

pub type RxKey = u64;

/// Window length for one pair's offset line, seconds.
pub const WINDOW: f64 = 60.0;
/// Forgetting time constant of a receiver's accumulated windows, seconds.
pub const TAU: f64 = 3600.0;
/// Observations a window needs to count.
const MIN_WIN_OBS: f64 = 6.0;
/// A window whose residual rms at the current estimate exceeds this (m) is
/// dropped: a clock reset, a multi-clock id, a partner far off.
const GATE_M: f64 = 250.0;
/// Prior 1-sigma of the position error (m): weak horizontally, tight in
/// height.
const PRIOR_H_M: f64 = 2000.0;
const PRIOR_V_M: f64 = 30.0;
/// Nominal 1-sigma of one observation (m) for the prior's weight.
const OBS_SIGMA_M: f64 = 60.0;
/// An estimate is "reliable" with this many windows and its weakest
/// horizontal sigma under this (m). On the 2026-10-24 capture, receivers
/// moved 300 m on purpose came back to 23 m p50 / 64 m p90 under this rule
/// (81 of 87 within 100 m), and correcting the 964 reliable ones improved
/// the holdout from 50 / 1132 m to 42 / 862 m (p50 / p90).
pub const RELIABLE_MIN_WINDOWS: u32 = 200;
pub const RELIABLE_MAX_SD_M: f64 = 5.0;

#[derive(Default, Clone)]
struct Win {
    /// Clock time (the receiver's own, seconds) and z of the first
    /// observation: sums are kept relative to them. z carries the pair's raw
    /// clock offset (counters start anywhere: up to 1e15 m), which would
    /// swamp the residuals' precision in f64.
    t0: f64,
    z0: f64,
    /// Wall time the window opened (for `sweep`).
    wall0: f64,
    n: f64,
    st: f64,
    stt: f64,
    sz: f64,
    stz: f64,
    szz: f64,
    sx: [f64; 3],
    stx: [f64; 3],
    sxx: [[f64; 3]; 3],
    sxz: [f64; 3],
}

#[derive(Default, Clone)]
struct Acc {
    n: [[f64; 3]; 3],
    b: [f64; 3],
    zz: f64,
    dof: f64,
    last: f64,
    windows: u32,
    dropped: u32,
}

/// A receiver's position correction: true = declared + d (east, north, up).
#[derive(Debug, Clone, Copy, serde::Serialize)]
pub struct Estimate {
    pub d: [f64; 3],
    /// 1-sigma of each component, metres.
    pub sd: [f64; 3],
    /// RMS of one observation's residual, metres.
    pub sigma: f64,
    /// Smallest horizontal information eigenvalue's sigma (m): how well the
    /// geometry pins the horizontal position in its weakest direction.
    pub sd_weak_h: f64,
    pub windows: u32,
    pub dropped: u32,
}

impl Estimate {
    pub fn reliable(&self) -> bool {
        self.windows >= RELIABLE_MIN_WINDOWS && self.sd_weak_h < RELIABLE_MAX_SD_M
    }

    pub fn horizontal_m(&self) -> f64 {
        self.d[0].hypot(self.d[1])
    }
}

#[derive(Default)]
pub struct Survey {
    open: HashMap<(RxKey, RxKey), Win>,
    acc: HashMap<RxKey, Acc>,
    pub observations: u64,
    pub windows_closed: u64,
    pub windows_dropped: u64,
}

impl Survey {
    /// One observation for receiver `r` against partner `k`: `t` = r's own
    /// clock time of it (seconds; the offset line is fitted against it, not
    /// against arrival, which jitters by seconds), `now` = wall time,
    /// `z` = (t_dK - t_dR) * c in metres, `u` = unit vector from r to the
    /// aircraft in r's east/north/up.
    pub fn observe(&mut self, r: RxKey, k: RxKey, t: f64, now: f64, z: f64, u: [f64; 3]) {
        self.observations += 1;
        let fresh = || Win { t0: t, z0: z, wall0: now, ..Win::default() };
        let w = self.open.entry((r, k)).or_insert_with(fresh);
        // A clock that jumped or went backwards starts a new window too.
        if t - w.t0 > WINDOW || t < w.t0 || now - w.wall0 > 2.0 * WINDOW {
            let done = std::mem::replace(w, fresh());
            self.close(r, done, now);
            let w = self.open.get_mut(&(r, k)).unwrap();
            add(w, t, z, u);
        } else {
            add(w, t, z, u);
        }
    }

    /// Close every window opened more than `WINDOW` ago (call now and then).
    pub fn sweep(&mut self, now: f64) {
        let old: Vec<(RxKey, RxKey)> = self.open.iter().filter(|(_, w)| now - w.wall0 > WINDOW).map(|(k, _)| *k).collect();
        for key in old {
            if let Some(w) = self.open.remove(&key) {
                self.close(key.0, w, now);
            }
        }
    }

    /// Forget a receiver (it moved, or left for good).
    pub fn forget(&mut self, r: RxKey) {
        self.acc.remove(&r);
        self.open.retain(|(a, _), _| *a != r);
    }

    fn close(&mut self, r: RxKey, w: Win, now: f64) {
        if w.n < MIN_WIN_OBS {
            return;
        }
        // Project out offset + drift: W = [1, t].
        let det = w.n * w.stt - w.st * w.st;
        let (g00, g01, g11) = if det > 1e-9 * w.n * w.n {
            (w.stt / det, -w.st / det, w.n / det)
        } else {
            (1.0 / w.n, 0.0, 0.0) // all at one instant: offset only
        };
        let mut nc = [[0.0; 3]; 3];
        let mut bc = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                let wx = g00 * w.sx[i] * w.sx[j] + g01 * (w.sx[i] * w.stx[j] + w.stx[i] * w.sx[j]) + g11 * w.stx[i] * w.stx[j];
                nc[i][j] = w.sxx[i][j] - wx;
            }
            let wz = g00 * w.sx[i] * w.sz + g01 * (w.sx[i] * w.stz + w.stx[i] * w.sz) + g11 * w.stx[i] * w.stz;
            bc[i] = w.sxz[i] - wz;
        }
        let zzc = w.szz - (g00 * w.sz * w.sz + 2.0 * g01 * w.sz * w.stz + g11 * w.stz * w.stz);
        let dofc = w.n - if g11 > 0.0 { 2.0 } else { 1.0 };
        let a = self.acc.entry(r).or_default();
        // Gate the window at the current estimate.
        let d = solve(a).map(|e| e.0).unwrap_or([0.0; 3]);
        let mut r2 = zzc;
        for i in 0..3 {
            r2 -= 2.0 * d[i] * bc[i];
            for j in 0..3 {
                r2 += d[i] * nc[i][j] * d[j];
            }
        }
        self.windows_closed += 1;
        if dofc <= 0.0 || r2 / dofc > GATE_M * GATE_M {
            a.dropped += 1;
            self.windows_dropped += 1;
            return;
        }
        let f = if a.last > 0.0 { (-(now - a.last).max(0.0) / TAU).exp() } else { 1.0 };
        for i in 0..3 {
            for j in 0..3 {
                a.n[i][j] = a.n[i][j] * f + nc[i][j];
            }
            a.b[i] = a.b[i] * f + bc[i];
        }
        a.zz = a.zz * f + zzc;
        a.dof = a.dof * f + dofc;
        a.last = now;
        a.windows += 1;
    }

    pub fn estimate(&self, r: RxKey) -> Option<Estimate> {
        let a = self.acc.get(&r)?;
        let (d, cov, sigma, weak) = solve(a)?;
        Some(Estimate {
            d,
            sd: [cov[(0, 0)].sqrt(), cov[(1, 1)].sqrt(), cov[(2, 2)].sqrt()],
            sigma,
            sd_weak_h: weak,
            windows: a.windows,
            dropped: a.dropped,
        })
    }

    pub fn receivers(&self) -> impl Iterator<Item = RxKey> + '_ {
        self.acc.keys().copied()
    }
}

fn add(w: &mut Win, t: f64, z: f64, u: [f64; 3]) {
    let t = t - w.t0;
    let z = z - w.z0;
    w.n += 1.0;
    w.st += t;
    w.stt += t * t;
    w.sz += z;
    w.stz += t * z;
    w.szz += z * z;
    for i in 0..3 {
        w.sx[i] += u[i];
        w.stx[i] += t * u[i];
        w.sxz[i] += u[i] * z;
        for j in 0..3 {
            w.sxx[i][j] += u[i] * u[j];
        }
    }
}

/// (d, covariance of d, observation sigma, weakest horizontal sigma).
fn solve(a: &Acc) -> Option<([f64; 3], Matrix3<f64>, f64, f64)> {
    if a.windows == 0 {
        return None;
    }
    let lh = (OBS_SIGMA_M / PRIOR_H_M).powi(2);
    let lv = (OBS_SIGMA_M / PRIOR_V_M).powi(2);
    let m = Matrix3::from_fn(|i, j| a.n[i][j]) + Matrix3::from_diagonal(&Vector3::new(lh, lh, lv));
    let inv = m.try_inverse()?;
    let b = Vector3::new(a.b[0], a.b[1], a.b[2]);
    let d = inv * b;
    let dof = (a.dof - 3.0).max(1.0);
    let s2 = ((a.zz - d.dot(&b)).max(0.0) / dof).max(1.0);
    let cov = inv * s2;
    // Horizontal information: the 2x2 block's smaller eigenvalue.
    let (h00, h01, h11) = (a.n[0][0] + lh, a.n[0][1], a.n[1][1] + lh);
    let tr = h00 + h11;
    let disc = ((h00 - h11).powi(2) + 4.0 * h01 * h01).sqrt();
    let lmin = ((tr - disc) / 2.0).max(1e-12);
    Some(([d[0], d[1], d[2]], cov, s2.sqrt(), (s2 / lmin).sqrt()))
}

#[cfg(test)]
mod tests;
