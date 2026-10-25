//! Tracker tests on synthetic flights. Run with `-- --nocapture` to see the
//! statistics they print.

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

use super::*;
use crate::geo::{ecef_to_llh, llh_to_ecef, Llh};
use crate::solver::testutil::{arrivals, llh, po_valley, random_cov, Gauss};
use crate::solver::{solve, SolveInput};

const ALT_SIGMA: f64 = 76.2;
const T_START: f64 = 1.7e9;

/// A synthetic trajectory: horizontal ENU coordinates in a fixed local frame
/// and a geodetic height, so "level" means level over the curved Earth, as
/// a real aircraft flies.
struct Flight {
    frame: Enu,
    h: f64,
}

impl Flight {
    fn at(lat: f64, lon: f64, h: f64) -> Flight {
        Flight { frame: Enu::at(llh_to_ecef(Llh { lat, lon, alt: 0.0 })), h }
    }

    /// ECEF point at horizontal ENU (e, n) and geodetic height h + dh.
    fn point(&self, e: f64, n: f64, dh: f64) -> Ecef {
        let mut u = self.h + dh;
        let mut p = self.frame.to_ecef(&[e, n, u]);
        for _ in 0..3 {
            u -= ecef_to_llh(p).alt - (self.h + dh);
            p = self.frame.to_ecef(&[e, n, u]);
        }
        p
    }
}

/// A fix at `truth` with ENU covariance diag(sh.0^2, sh.1^2, sv^2), its
/// horizontal axes rotated by `rot_deg` (MLAT ellipses are rarely round);
/// the noise is drawn from exactly that covariance.
fn fix_at(g: &mut Gauss, truth: &Ecef, sh: (f64, f64), sv: f64, rot_deg: f64) -> Fix {
    let local = Enu::at(*truth);
    let r = Matrix3::from_fn(|i, j| local.r[i][j]);
    let (s, c) = rot_deg.to_radians().sin_cos();
    let a = Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
    let cov_enu = a * Matrix3::from_diagonal(&Vector3::new(sh.0 * sh.0, sh.1 * sh.1, sv * sv)) * a.transpose();
    let cov = r.transpose() * cov_enu * r;
    let l = cov.cholesky().unwrap().unpack();
    let e = l * Vector3::new(g.normal(), g.normal(), g.normal());
    Fix {
        pos: [truth[0] + e[0], truth[1] + e[1], truth[2] + e[2]],
        cov,
        t0: 0.0,
        chi2: 0.0,
        dof: 1,
        err_h: (cov_enu[(0, 0)] + cov_enu[(1, 1)]).sqrt(),
        gdop: 2.0,
        used: vec![true; 4],
    }
}

fn horiz_dist(a: &Ecef, b: &Ecef) -> f64 {
    let l = Enu::at(*b).to_enu(a);
    l[0].hypot(l[1])
}

fn track_nees(tr: &Track, truth: &Ecef) -> f64 {
    let (p, c) = tr.predict(tr.last_update());
    let e = Vector3::new(p[0] - truth[0], p[1] - truth[1], p[2] - truth[2]);
    (e.transpose() * c.try_inverse().unwrap() * e)[0]
}

fn wrap180(d: f64) -> f64 {
    (d + 540.0).rem_euclid(360.0) - 180.0
}

// ---------------------------------------------------------------- straight

/// One update of a synthetic straight flight.
struct Step {
    t: f64,
    accepted: bool,
    /// Horizontal error of the track / of the fix, m.
    err: f64,
    fix_err: f64,
    speed_err_kt: f64,
    track_err_deg: f64,
    vrate_fpm: f64,
    nees: f64,
}

/// Straight and level at 250 m/s on heading `hdg`, 1 Hz fixes with a
/// (sh, 0.7 sh) horizontal ellipse rotating slowly and 60 m vertical (a fix
/// with Mode S altitude).
fn straight_flight(seed: u64, hdg: f64, secs: usize, sh: f64, single_model: bool) -> Vec<Step> {
    let mut g = Gauss::new(seed);
    let fl = Flight::at(45.0, 9.5, 11_000.0);
    let v = 250.0;
    let (ve, vn) = (v * hdg.to_radians().sin(), v * hdg.to_radians().cos());
    let truth = |t: f64| fl.point(ve * t, vn * t, 0.0);
    let mut tr = Track::new(&fix_at(&mut g, &truth(0.0), (sh, 0.7 * sh), 60.0, 30.0), T_START);
    if single_model {
        tr = tr.without_imm();
    }
    (1..=secs)
        .map(|i| {
            let t = i as f64;
            let p = truth(t);
            let f = fix_at(&mut g, &p, (sh, 0.7 * sh), 60.0, 30.0 + t);
            let accepted = tr.update_fix(&f, T_START + t) == Gate::Accepted;
            Step {
                t,
                accepted,
                err: horiz_dist(&tr.position(), &p),
                fix_err: horiz_dist(&f.pos, &p),
                speed_err_kt: tr.ground_speed_kt() - v * MS_TO_KT,
                track_err_deg: wrap180(tr.track_deg() - hdg),
                vrate_fpm: tr.vrate_fpm(),
                nees: track_nees(&tr, &p),
            }
        })
        .collect()
}

#[derive(Default)]
struct FlightStats {
    n: usize,
    rejected: usize,
    pos2: f64,
    fix2: f64,
    after30: usize,
    speed_bad: usize,
    speed2: f64,
    worst_trk: f64,
    vr2: f64,
    vr_bad: usize,
    nees: f64,
}

impl FlightStats {
    fn collect(seeds: std::ops::Range<u64>, sh: f64, single_model: bool) -> FlightStats {
        let mut s = FlightStats::default();
        for seed in seeds {
            for st in straight_flight(seed, 60.0 + 7.0 * seed as f64, 120, sh, single_model) {
                s.rejected += usize::from(!st.accepted);
                if st.t >= 10.0 {
                    s.pos2 += st.err * st.err;
                    s.fix2 += st.fix_err * st.fix_err;
                    s.n += 1;
                    s.nees += st.nees;
                }
                if st.t >= 30.0 {
                    s.after30 += 1;
                    s.speed_bad += usize::from(st.speed_err_kt.abs() > 5.0);
                    s.speed2 += st.speed_err_kt * st.speed_err_kt;
                    s.worst_trk = s.worst_trk.max(st.track_err_deg.abs());
                    s.vr2 += st.vrate_fpm * st.vrate_fpm;
                    s.vr_bad += usize::from(st.vrate_fpm.abs() > 500.0);
                }
            }
        }
        s
    }

    fn pos_rms(&self) -> f64 {
        (self.pos2 / self.n as f64).sqrt()
    }

    fn fix_rms(&self) -> f64 {
        (self.fix2 / self.n as f64).sqrt()
    }

    fn speed_rms(&self) -> f64 {
        (self.speed2 / self.after30 as f64).sqrt()
    }

    fn vrate_rms(&self) -> f64 {
        (self.vr2 / self.after30 as f64).sqrt()
    }

    fn report(&self, name: &str) {
        eprintln!(
            "[track] {name}: horizontal RMS {:.0} m vs fixes {:.0} m | after 30 s: speed RMS {:.2} kt, >5 kt in {}/{}, \
             worst track error {:.2} deg, vrate RMS {:.0} fpm (>500 in {}) | NEES mean {:.2} | gate rejections {}",
            self.pos_rms(),
            self.fix_rms(),
            self.speed_rms(),
            self.speed_bad,
            self.after30,
            self.worst_trk,
            self.vrate_rms(),
            self.vr_bad,
            self.nees / self.n as f64,
            self.rejected,
        );
    }
}

#[test]
fn straight_flight_converges() {
    // A good network (fixes with altitude, err_h ~ 100 m), 50 flights x 120 s.
    let s = FlightStats::collect(0..50, 80.0, false);
    s.report("straight, 80 x 56 m fixes at 1 Hz, x50");
    assert!(s.pos_rms() < 0.4 * s.fix_rms(), "{} vs {}", s.pos_rms(), s.fix_rms());
    assert!(s.speed_bad as f64 <= 0.03 * s.after30 as f64, "{}/{}", s.speed_bad, s.after30);
    assert!(s.speed_rms() < 2.5, "{}", s.speed_rms());
    assert!(s.worst_trk < 2.0, "{}", s.worst_trk);
    assert!(s.vrate_rms() < 200.0 && s.vr_bad as f64 <= 0.01 * s.after30 as f64, "{} {}", s.vrate_rms(), s.vr_bad);
    // A 99.9% gate under each of three models: false rejections are rare.
    assert!(s.rejected <= 6, "{}", s.rejected);
    // Consistent, if anything conservative: straight flight never uses the
    // acceleration the process noise allows for.
    let nees = s.nees / s.n as f64;
    assert!((1.0..=3.3).contains(&nees), "{nees}");

    // A sparse network (150 x 105 m): the speed is still within 5 kt on
    // average, the position much better than any single fix.
    let s = FlightStats::collect(0..50, 150.0, false);
    s.report("straight, 150 x 105 m fixes at 1 Hz, x50");
    assert!(s.speed_rms() < 4.0, "{}", s.speed_rms());
    assert!(s.pos_rms() < 0.4 * s.fix_rms());
}

#[test]
fn imm_beats_the_single_model_in_straight_flight() {
    // The specified single CV model (sigma_a 3 m/s^2) keeps re-estimating a
    // velocity it believes may change by 3 m/s every second.
    let imm = FlightStats::collect(100..130, 80.0, false);
    let one = FlightStats::collect(100..130, 80.0, true);
    imm.report("straight, IMM");
    one.report("straight, single CV EKF (sigma_a 3)");
    assert!(imm.speed_rms() < 0.5 * one.speed_rms(), "{} vs {}", imm.speed_rms(), one.speed_rms());
    assert!(imm.pos_rms() < one.pos_rms());
}

// ------------------------------------------------------------------- turns

/// A level coordinated turn: straight 40 s east, then through `turn_deg` to
/// the right at `accel` m/s^2 centripetal, then straight for 40 s; 1 Hz
/// fixes of (sh, 2/3 sh). Returns (accepted fixes during the turn, fixes
/// during the turn, worst horizontal error from the turn on, final track
/// error deg).
fn turn(seed: u64, accel: f64, turn_deg: f64, sh: f64, single_model: bool) -> (usize, usize, f64, f64) {
    let mut g = Gauss::new(seed);
    let fl = Flight::at(45.2, 9.0, 9_000.0);
    let v = 230.0;
    let w = accel / v; // rad/s
    let (t1, t2) = (40.0, 40.0 + turn_deg.to_radians() / w);
    let rad = v / w;
    let pos = |t: f64| -> (f64, f64, f64) {
        if t <= t1 {
            (v * t, 0.0, 90.0)
        } else if t <= t2 {
            let th = w * (t - t1);
            (v * t1 + rad * th.sin(), -rad * (1.0 - th.cos()), 90.0 + th.to_degrees())
        } else {
            let th = w * (t2 - t1);
            let hdg = (90.0 + turn_deg).to_radians();
            let (e2, n2) = (v * t1 + rad * th.sin(), -rad * (1.0 - th.cos()));
            (e2 + v * hdg.sin() * (t - t2), n2 + v * hdg.cos() * (t - t2), 90.0 + turn_deg)
        }
    };
    let shv = (sh, 2.0 * sh / 3.0);
    let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), shv, 60.0, 0.0), T_START);
    if single_model {
        tr = tr.without_imm();
    }
    let (mut acc, mut total, mut worst) = (0, 0, 0.0f64);
    let end = t2 + 40.0;
    let mut t = 1.0;
    while t <= end {
        let (e, n, _) = pos(t);
        let p = fl.point(e, n, 0.0);
        let gate = tr.update_fix(&fix_at(&mut g, &p, shv, 60.0, 3.0 * t), T_START + t);
        if t > t1 && t <= t2 {
            total += 1;
            acc += usize::from(gate == Gate::Accepted);
        }
        if t > t1 {
            worst = worst.max(horiz_dist(&tr.predict(T_START + t).0, &p));
        }
        t += 1.0;
    }
    (acc, total, worst, wrap180(tr.track_deg() - pos(end).2))
}

#[test]
fn two_g_turn_is_followed() {
    let g2 = 2.0 * 9.80665;
    let (mut acc, mut tot, mut worst, mut worst_trk) = (0, 0, 0.0f64, 0.0f64);
    for seed in 0..20 {
        let (a, n, w, te) = turn(seed, g2, 180.0, 120.0, false);
        acc += a;
        tot += n;
        worst = worst.max(w);
        worst_trk = worst_trk.max(te.abs());
    }
    let (sa, st, sw, ste) = turn(0, g2, 180.0, 120.0, true);
    eprintln!(
        "[track] 2 g 180 deg turn, 120 x 80 m fixes, x20: IMM accepted {acc}/{tot} fixes in the turn, \
         worst horizontal error {worst:.0} m, worst final track error {worst_trk:.1} deg | \
         single CV EKF: accepted {sa}/{st}, worst error {sw:.0} m, final track error {ste:.0} deg"
    );
    assert!(acc as f64 >= 0.98 * tot as f64, "{acc}/{tot}");
    assert!(worst < 1_000.0, "{worst}");
    assert!(worst_trk < 3.0, "{worst_trk}");
    // The reason for the IMM: the specified single model loses the aircraft.
    assert!((sa as f64) < 0.5 * st as f64 && sw > 2_000.0, "single model unexpectedly fine: {sa}/{st}, {sw}");
}

#[test]
fn gentle_turn_is_followed() {
    // An airliner's 25 degree bank (4.6 m/s^2, ~1.1 deg/s) through 90 deg.
    let a = 9.80665 * 25f64.to_radians().tan();
    let (mut acc, mut tot, mut worst, mut worst_trk) = (0, 0, 0.0f64, 0.0f64);
    for seed in 0..20 {
        let (x, n, w, te) = turn(seed, a, 90.0, 150.0, false);
        acc += x;
        tot += n;
        worst = worst.max(w);
        worst_trk = worst_trk.max(te.abs());
    }
    eprintln!(
        "[track] 25 deg bank 90 deg turn, 150 x 100 m fixes, x20: accepted {acc}/{tot}, worst error {worst:.0} m, \
         worst final track error {worst_trk:.1} deg"
    );
    assert!(acc as f64 >= 0.98 * tot as f64 && worst < 1_000.0 && worst_trk < 3.0);
}

// ---------------------------------------------------------------- outliers

#[test]
fn outlier_fix_is_rejected_without_touching_the_state() {
    let mut g = Gauss::new(9);
    let fl = Flight::at(45.0, 10.0, 10_000.0);
    let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (150.0, 100.0), 60.0, 0.0), T_START);
    for i in 1..=20 {
        let t = i as f64;
        tr.update_fix(&fix_at(&mut g, &fl.point(240.0 * t, 0.0, 0.0), (150.0, 100.0), 60.0, 0.0), T_START + t);
    }
    assert!(tr.confirmed());
    let before = tr.clone();
    let truth = fl.point(240.0 * 21.0, 0.0, 0.0);
    let mut bad = fix_at(&mut g, &truth, (150.0, 100.0), 60.0, 0.0);
    bad.pos = Enu::at(truth).to_ecef(&[0.0, 50_000.0, 0.0]);
    match tr.update_fix(&bad, T_START + 21.0) {
        Gate::Rejected(d2) => assert!(d2 > 1e4, "{d2}"),
        other => panic!("{other:?}"),
    }
    assert_eq!(tr.position(), before.position());
    assert_eq!(tr.last_update(), before.last_update());
    assert_eq!(tr.updates(), before.updates());
    assert_eq!(tr.mu, before.mu);
    assert_eq!(tr.predict(T_START + 22.0).1, before.predict(T_START + 22.0).1);
    // A sane fix afterwards is still accepted.
    let f = fix_at(&mut g, &fl.point(240.0 * 22.0, 0.0, 0.0), (150.0, 100.0), 60.0, 0.0);
    assert_eq!(tr.update_fix(&f, T_START + 22.0), Gate::Accepted);
}

// -------------------------------------------------------------------- TDOA

/// Milan, Turin, Bologna (100-300 km baselines) around a flight at 11 km.
fn tdoa_setup() -> (Vec<Ecef>, Flight) {
    let rx = vec![llh(45.464, 9.190, 150.0), llh(45.070, 7.687, 260.0), llh(44.494, 11.343, 80.0)];
    (rx, Flight::at(45.0, 9.3, 11_000.0))
}

/// A track built from 15 s of fixes, then `secs` of TDOA-only updates from
/// the first `k` receivers (+ Mode S altitude when `alt`) with the engine's
/// correlated time covariance. Returns (worst horizontal error during the
/// TDOA phase, accepted, attempted, final speed error kt).
fn tdoa_only(seed: u64, k: usize, alt: bool, secs: usize) -> (f64, usize, usize, f64) {
    let (rx, fl) = tdoa_setup();
    let rx = &rx[..k];
    let mut g = Gauss::new(seed);
    let (ve, vn) = (180.0, 150.0);
    let truth = |t: f64| fl.point(ve * t, vn * t, 0.0);
    let mut tr = Track::new(&fix_at(&mut g, &truth(0.0), (200.0, 150.0), 60.0, 20.0), T_START);
    for i in 1..=15 {
        let t = i as f64;
        tr.update_fix(&fix_at(&mut g, &truth(t), (200.0, 150.0), 60.0, 20.0), T_START + t);
    }
    let (mut worst, mut acc, mut n) = (0.0f64, 0, 0);
    for i in 1..=secs {
        let t = 15.0 + i as f64;
        let p = truth(t);
        let cov = random_cov(&mut g, k, 0.1e-6, 1.0e-6);
        let meas = arrivals(&p, 123.0 + t, rx, &g.mvn(&cov));
        let a = alt.then(|| (ecef_to_llh(p).alt + ALT_SIGMA * g.normal(), ALT_SIGMA));
        n += 1;
        acc += usize::from(tr.update_tdoa(&meas, &cov, a, T_START + t) == Gate::Accepted);
        worst = worst.max(horiz_dist(&tr.position(), &p));
    }
    (worst, acc, n, tr.ground_speed_kt() - f64::hypot(ve, vn) * MS_TO_KT)
}

#[test]
fn tdoa_only_keeps_the_track_with_three_receivers_and_altitude() {
    let (mut worst, mut acc, mut n, mut worst_sp) = (0.0f64, 0, 0, 0.0f64);
    for seed in 0..20 {
        let (w, a, m, sp) = tdoa_only(seed, 3, true, 60);
        worst = worst.max(w);
        acc += a;
        n += m;
        worst_sp = worst_sp.max(sp.abs());
    }
    eprintln!(
        "[track] TDOA-only 60 s, 3 rx + alt, x20: worst horizontal error {worst:.0} m, accepted {acc}/{n}, \
         worst final speed error {worst_sp:.1} kt"
    );
    assert!(worst < 1_000.0, "{worst}");
    assert!(acc as f64 >= 0.98 * n as f64, "{acc}/{n}");
}

#[test]
fn tdoa_only_with_two_receivers_and_altitude() {
    // A single time difference + altitude: the along-hyperbola direction is
    // held only by the motion model. Require it to stay useful over 30 s.
    let (mut worst, mut acc, mut n) = (0.0f64, 0, 0);
    for seed in 0..20 {
        let (w, a, m, _) = tdoa_only(seed, 2, true, 30);
        worst = worst.max(w);
        acc += a;
        n += m;
    }
    eprintln!("[track] TDOA-only 30 s, 2 rx + alt, x20: worst horizontal error {worst:.0} m, accepted {acc}/{n}");
    assert!(worst < 2_000.0, "{worst}");
    assert!(acc as f64 >= 0.95 * n as f64, "{acc}/{n}");
}

#[test]
fn tdoa_innovations_are_calibrated() {
    // On settled straight tracks the gate statistic of the dominant (steady)
    // model is chi2(m), m = (k - 1) + 1 for the altitude: its mean checks the
    // D cov D^T measurement covariance (drop its cross terms and it moves far
    // from 3) and the altitude row together.
    let (rx, fl) = tdoa_setup();
    let (mut sum, mut n) = (0.0, 0);
    for seed in 0..20 {
        let mut g = Gauss::new(300 + seed);
        let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (100.0, 80.0), 50.0, 0.0), T_START);
        for i in 1..=60 {
            let t = i as f64;
            tr.update_fix(&fix_at(&mut g, &fl.point(200.0 * t, 0.0, 0.0), (100.0, 80.0), 50.0, 0.0), T_START + t);
        }
        for i in 0..100 {
            // Probe many groups at one instant without updating.
            let t = 60.5 + 1e-4 * i as f64;
            let p = fl.point(200.0 * t, 0.0, 0.0);
            let cov = random_cov(&mut g, 3, 0.1e-6, 1.0e-6);
            let meas = arrivals(&p, 7.0 + t, &rx, &g.mvn(&cov));
            let alt = Some((ecef_to_llh(p).alt + ALT_SIGMA * g.normal(), ALT_SIGMA));
            let (d2, c) = tr.probe_tdoa(&meas, &cov, alt, T_START + t).unwrap();
            let j = (0..N).max_by(|&a, &b| c[a].total_cmp(&c[b])).unwrap();
            sum += d2[j];
            n += 1;
        }
    }
    let mean = sum / n as f64;
    eprintln!("[track] TDOA gate statistic (3 rx + alt, m = 3) on settled tracks: mean {mean:.2} (chi2(3): 3)");
    assert!((2.6..=3.4).contains(&mean), "{mean}");
}

#[test]
fn tdoa_gate_rejects_a_bad_group() {
    let (rx, fl) = tdoa_setup();
    let mut g = Gauss::new(4);
    let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (200.0, 150.0), 60.0, 0.0), T_START);
    for i in 1..=10 {
        let t = i as f64;
        tr.update_fix(&fix_at(&mut g, &fl.point(200.0 * t, 0.0, 0.0), (200.0, 150.0), 60.0, 0.0), T_START + t);
    }
    let p = fl.point(2_200.0, 0.0, 0.0);
    let cov = DMatrix::from_diagonal_element(3, 3, (0.3e-6f64).powi(2));
    let mut noise = DVector::zeros(3);
    noise[1] = 40e-6; // a 12 km mismatch: wrong aircraft or a clock jump
    let meas = arrivals(&p, 50.0, &rx, &noise);
    let before = tr.position();
    assert!(matches!(tr.update_tdoa(&meas, &cov, Some((11_000.0, ALT_SIGMA)), T_START + 11.0), Gate::Rejected(_)));
    assert_eq!(tr.position(), before);
    let meas = arrivals(&p, 50.0, &rx, &DVector::zeros(3));
    assert_eq!(tr.update_tdoa(&meas, &cov, Some((11_000.0, ALT_SIGMA)), T_START + 11.0), Gate::Accepted);
}

// ------------------------------------------------------------- bookkeeping

#[test]
fn skipped_cases() {
    let (rx, fl) = tdoa_setup();
    let mut g = Gauss::new(5);
    let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (200.0, 150.0), 60.0, 0.0), T_START);
    let p = fl.point(0.0, 0.0, 0.0);
    let cov3 = DMatrix::from_diagonal_element(3, 3, (0.3e-6f64).powi(2));
    let meas = arrivals(&p, 50.0, &rx, &DVector::zeros(3));
    // One receiver: no difference to form.
    assert_eq!(tr.update_tdoa(&meas[..1], &DMatrix::from_element(1, 1, 1e-13), None, T_START + 1.0), Gate::Skipped);
    // Covariance of the wrong size, NaN time, bad altitude sigma.
    assert_eq!(tr.update_tdoa(&meas, &DMatrix::zeros(2, 2), None, T_START + 1.0), Gate::Skipped);
    let mut m2 = meas.clone();
    m2[1].t = f64::NAN;
    assert_eq!(tr.update_tdoa(&m2, &cov3, None, T_START + 1.0), Gate::Skipped);
    assert_eq!(tr.update_tdoa(&meas, &cov3, Some((11_000.0, 0.0)), T_START + 1.0), Gate::Skipped);
    // Track too old for TDOA (31 s), and updates from the past.
    assert_eq!(tr.update_tdoa(&meas, &cov3, None, T_START + 31.0), Gate::Skipped);
    assert_eq!(tr.update_tdoa(&meas, &cov3, None, T_START - 1.0), Gate::Skipped);
    assert_eq!(tr.update_fix(&fix_at(&mut g, &p, (200.0, 150.0), 60.0, 0.0), T_START - 1.0), Gate::Skipped);
    // A fix with a NaN covariance.
    let mut f = fix_at(&mut g, &p, (200.0, 150.0), 60.0, 0.0);
    f.cov[(0, 0)] = f64::NAN;
    assert_eq!(tr.update_fix(&f, T_START + 1.0), Gate::Skipped);
    assert_eq!(tr.updates(), 1);
    // Two receivers is enough; a 30 s old track still takes TDOA.
    let cov2 = DMatrix::from_diagonal_element(2, 2, (0.3e-6f64).powi(2));
    assert_eq!(tr.update_tdoa(&meas[..2], &cov2, Some((11_000.0, ALT_SIGMA)), T_START + 30.0), Gate::Accepted);
}

#[test]
fn confirmation_and_staleness() {
    let (rx, fl) = tdoa_setup();
    let mut g = Gauss::new(6);
    let p = fl.point(0.0, 0.0, 0.0);
    let fix = |g: &mut Gauss| fix_at(g, &p, (200.0, 150.0), 60.0, 0.0);
    let cov3 = DMatrix::from_diagonal_element(3, 3, (0.3e-6f64).powi(2));
    let meas = arrivals(&p, 50.0, &rx, &DVector::zeros(3));
    let alt = Some((11_000.0, ALT_SIGMA));

    // The initial fix counts: fix (new) + fix + TDOA within 60 s confirms.
    let mut tr = Track::new(&fix(&mut g), T_START);
    assert!(!tr.confirmed());
    assert_eq!(tr.update_fix(&fix(&mut g), T_START + 5.0), Gate::Accepted);
    assert!(!tr.confirmed());
    assert_eq!(tr.update_tdoa(&meas, &cov3, alt, T_START + 10.0), Gate::Accepted);
    assert!(tr.confirmed());
    assert_eq!(tr.updates(), 3);

    // fix + two TDOA: three updates but only one fix.
    let mut tr = Track::new(&fix(&mut g), T_START);
    tr.update_tdoa(&meas, &cov3, alt, T_START + 5.0);
    tr.update_tdoa(&meas, &cov3, alt, T_START + 10.0);
    assert_eq!(tr.updates(), 3);
    assert!(!tr.confirmed());
    assert_eq!(tr.update_fix(&fix(&mut g), T_START + 15.0), Gate::Accepted);
    assert!(tr.confirmed());

    // Three fixes spread over more than 60 s do not confirm.
    let mut tr = Track::new(&fix(&mut g), T_START);
    tr.update_fix(&fix(&mut g), T_START + 35.0);
    tr.update_fix(&fix(&mut g), T_START + 70.0);
    assert!(!tr.confirmed());
    tr.update_fix(&fix(&mut g), T_START + 80.0);
    assert!(tr.confirmed());

    assert!(!tr.stale(T_START + 80.0 + 119.0));
    assert!(tr.stale(T_START + 80.0 + 121.0));
}

// ------------------------------------------------------------------ outputs

#[test]
fn derived_outputs() {
    // Climbing at 10 m/s (1969 ft/min) on heading 225 at 200 m/s.
    let mut g = Gauss::new(8);
    let fl = Flight::at(45.0, 9.0, 3_000.0);
    let hdg: f64 = 225.0;
    let (ve, vn, vu) = (200.0 * hdg.to_radians().sin(), 200.0 * hdg.to_radians().cos(), 10.0);
    let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (80.0, 60.0), 40.0, 0.0), T_START);
    for i in 1..=60 {
        let t = i as f64;
        tr.update_fix(&fix_at(&mut g, &fl.point(ve * t, vn * t, vu * t), (80.0, 60.0), 40.0, 0.0), T_START + t);
    }
    let (gs, trk, vr, eh) = (tr.ground_speed_kt(), tr.track_deg(), tr.vrate_fpm(), tr.err_h());
    eprintln!("[track] climb: gs {gs:.1} kt, trk {trk:.2} deg, vrate {vr:.0} fpm, err_h {eh:.0} m");
    assert!((tr.ground_speed_kt() - 200.0 * MS_TO_KT).abs() < 5.0);
    assert!((tr.track_deg() - 225.0).abs() < 1.0);
    assert!((tr.vrate_fpm() - 10.0 * M_TO_FT * 60.0).abs() < 300.0);
    assert!(tr.err_h() > 5.0 && tr.err_h() < 100.0);
    // err_h agrees with the predicted covariance at the last update.
    let (pos, c) = tr.predict(tr.last_update());
    let l = Enu::at(pos);
    let r = Matrix3::from_fn(|i, j| l.r[i][j]);
    let ce = r * c * r.transpose();
    assert!(((ce[(0, 0)] + ce[(1, 1)]).sqrt() - tr.err_h()).abs() < 1e-6);
    // Track angle convention: 0 = north, clockwise.
    for (h, (e, n)) in [(0.0, (0.0, 1.0)), (90.0, (1.0, 0.0)), (180.0, (0.0, -1.0)), (270.0, (-1.0, 0.0))] {
        let mut g = Gauss::new(1);
        let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (30.0, 30.0), 30.0, 0.0), T_START);
        for i in 1..=20 {
            let t = i as f64;
            tr.update_fix(&fix_at(&mut g, &fl.point(200.0 * e * t, 200.0 * n * t, 0.0), (30.0, 30.0), 30.0, 0.0), T_START + t);
        }
        assert!(wrap180(tr.track_deg() - h).abs() < 1.0, "{h} -> {}", tr.track_deg());
        assert!((0.0..360.0).contains(&tr.track_deg()));
    }
}

#[test]
fn level_flight_far_from_the_anchor_has_no_fake_vertical_rate() {
    // 400 km east of the first fix at constant height: the anchor frame's
    // "up" is tilted by 3.6 degrees there, worth a fake 3000 ft/min if the
    // velocity were read in the anchor frame.
    let mut g = Gauss::new(10);
    let fl = Flight::at(45.0, 6.0, 11_000.0);
    let v = 250.0;
    let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (100.0, 80.0), 50.0, 0.0), T_START);
    let (mut vr_sum, mut n, mut last_e) = (0.0, 0, 0.0);
    for i in 1..=1600 {
        let t = i as f64;
        tr.update_fix(&fix_at(&mut g, &fl.point(v * t, 0.0, 0.0), (100.0, 80.0), 50.0, 0.0), T_START + t);
        if i > 1300 {
            vr_sum += tr.vrate_fpm();
            n += 1;
            last_e = horiz_dist(&tr.position(), &fl.point(v * t, 0.0, 0.0));
        }
    }
    let vr = vr_sum / n as f64;
    let (gs, trk) = (tr.ground_speed_kt(), tr.track_deg());
    eprintln!("[track] 400 km from the anchor: mean vrate {vr:.0} fpm, gs {gs:.1} kt, trk {trk:.2}, error {last_e:.0} m");
    assert!(vr.abs() < 100.0, "{vr}");
    assert!((tr.ground_speed_kt() - v * MS_TO_KT).abs() < 5.0);
    // East along the anchor's tangent plane is not a parallel 400 km later:
    // the local track angle there is ~90 + 3.5 deg (meridian convergence).
    assert!((tr.track_deg() - 90.0).abs() < 5.0, "{}", tr.track_deg());
}

#[test]
fn end_to_end_solver_fixes_then_tdoa() {
    // The engine's intended use: solver fixes while 6 receivers hear the
    // aircraft, then only 3 do and the track lives on TDOA updates.
    let rx = po_valley(6);
    let fl = Flight::at(45.0, 9.0, 10_500.0);
    let mut g = Gauss::new(77);
    let (ve, vn) = (200.0, -120.0);
    let mut tr: Option<Track> = None;
    let (mut worst_fix_phase, mut worst_tdoa_phase, mut tdoa_acc) = (0.0f64, 0.0f64, 0);
    for i in 0..120 {
        let t = i as f64;
        let p = fl.point(ve * t, vn * t, 0.0);
        let alt = (ecef_to_llh(p).alt + ALT_SIGMA * g.normal(), ALT_SIGMA);
        if i < 60 {
            let cov = random_cov(&mut g, 6, 0.1e-6, 1.0e-6);
            let meas = arrivals(&p, 500.0 + t, &rx, &g.mvn(&cov));
            let guess = tr.as_ref().map_or(rx[0], |tr| tr.predict(T_START + t).0);
            let Some(fix) = solve(&SolveInput { meas: &meas, cov: &cov, alt: Some(alt), guesses: &[guess] }) else { continue };
            match &mut tr {
                None => tr = Some(Track::new(&fix, T_START + t)),
                Some(tr) => {
                    tr.update_fix(&fix, T_START + t);
                }
            }
            if i > 20 {
                worst_fix_phase = worst_fix_phase.max(horiz_dist(&tr.as_ref().unwrap().position(), &p));
            }
        } else {
            let sub = [rx[0], rx[2], rx[5]];
            let cov = random_cov(&mut g, 3, 0.1e-6, 1.0e-6);
            let meas = arrivals(&p, 500.0 + t, &sub, &g.mvn(&cov));
            let tr = tr.as_mut().unwrap();
            tdoa_acc += usize::from(tr.update_tdoa(&meas, &cov, Some(alt), T_START + t) == Gate::Accepted);
            worst_tdoa_phase = worst_tdoa_phase.max(horiz_dist(&tr.position(), &p));
        }
    }
    let tr = tr.unwrap();
    eprintln!(
        "[track] end-to-end (solver fixes 60 s, then 3-rx TDOA 60 s): worst error {worst_fix_phase:.0} m / {worst_tdoa_phase:.0} m, \
         TDOA accepted {tdoa_acc}/60, gs {:.0} kt",
        tr.ground_speed_kt()
    );
    assert!(tr.confirmed());
    assert!(tdoa_acc >= 58);
    assert!(worst_fix_phase < 300.0 && worst_tdoa_phase < 600.0);
}

#[test]
fn predict_extrapolates_and_grows_uncertainty() {
    let mut g = Gauss::new(12);
    let fl = Flight::at(45.0, 9.0, 10_000.0);
    let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (100.0, 80.0), 50.0, 0.0), T_START);
    for i in 1..=30 {
        let t = i as f64;
        tr.update_fix(&fix_at(&mut g, &fl.point(0.0, 220.0 * t, 0.0), (100.0, 80.0), 50.0, 0.0), T_START + t);
    }
    let (p10, c10) = tr.predict(T_START + 40.0);
    let (_, c0) = tr.predict(T_START + 30.0);
    assert!(horiz_dist(&p10, &fl.point(0.0, 220.0 * 40.0, 0.0)) < 300.0);
    assert!(c10.trace() > 2.0 * c0.trace());
    // Slightly in the past is allowed and stays finite.
    let (pb, cb) = tr.predict(T_START + 29.5);
    assert!(pb.iter().chain(cb.iter()).all(|v| v.is_finite()));
}

#[test]
#[ignore = "timing; run with --release -- --ignored --nocapture"]
fn bench_track() {
    let (rx, fl) = tdoa_setup();
    let mut g = Gauss::new(13);
    let mut tr = Track::new(&fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (100.0, 80.0), 50.0, 0.0), T_START);
    let fixes: Vec<Fix> = (1..=2000).map(|i| fix_at(&mut g, &fl.point(200.0 * i as f64, 0.0, 0.0), (100.0, 80.0), 50.0, 0.0)).collect();
    let t0 = std::time::Instant::now();
    for (i, f) in fixes.iter().enumerate() {
        std::hint::black_box(tr.update_fix(f, T_START + 1.0 + i as f64));
    }
    let per_fix = t0.elapsed().as_secs_f64() / fixes.len() as f64 * 1e6;
    let cov = DMatrix::from_diagonal_element(3, 3, (0.3e-6f64).powi(2));
    let groups: Vec<Vec<Meas>> =
        (1..=2000).map(|i| arrivals(&fl.point(200.0 * (2000.0 + i as f64), 0.0, 0.0), 0.0, &rx, &DVector::zeros(3))).collect();
    let t0 = std::time::Instant::now();
    for (i, m) in groups.iter().enumerate() {
        std::hint::black_box(tr.update_tdoa(m, &cov, Some((11_000.0, ALT_SIGMA)), T_START + 2001.0 + i as f64));
    }
    let per_tdoa = t0.elapsed().as_secs_f64() / groups.len() as f64 * 1e6;
    eprintln!("[bench] track: {per_fix:.1} us per fix update, {per_tdoa:.1} us per TDOA update (k = 3 + alt), 3-model IMM");
}

#[test]
fn non_finite_times_do_not_panic() {
    let mut g = Gauss::new(14);
    let fl = Flight::at(45.0, 9.0, 10_000.0);
    let f = fix_at(&mut g, &fl.point(0.0, 0.0, 0.0), (100.0, 80.0), 50.0, 0.0);
    let mut tr = Track::new(&f, T_START);
    let (p, c) = tr.predict(f64::NAN);
    assert!(p.iter().chain(c.iter()).all(|v| v.is_finite()));
    assert_eq!(tr.update_fix(&f, f64::NAN), Gate::Skipped);
    assert_eq!(tr.update_fix(&f, f64::INFINITY), Gate::Skipped);
    assert!(!tr.stale(T_START));
}

// --------------------------------------------------------------- velocity

/// tdoa_only with 2 receivers + altitude for `secs`, plus (when `vel`) a
/// Comm-B BDS 5,0 ground velocity every 4 s (1 m/s noise per component, as
/// the register's 2 kt / 0.18 deg resolution gives). Returns (worst
/// horizontal error, velocity updates accepted, offered).
fn two_rx_with_velocity(seed: u64, secs: usize, vel: bool) -> (f64, usize, usize) {
    let (rx, fl) = tdoa_setup();
    let rx = &rx[..2];
    let mut g = Gauss::new(seed);
    let (ve, vn) = (180.0, 150.0);
    let truth = |t: f64| fl.point(ve * t, vn * t, 0.0);
    let mut tr = Track::new(&fix_at(&mut g, &truth(0.0), (200.0, 150.0), 60.0, 20.0), T_START);
    for i in 1..=15 {
        let t = i as f64;
        tr.update_fix(&fix_at(&mut g, &truth(t), (200.0, 150.0), 60.0, 20.0), T_START + t);
    }
    let (mut worst, mut acc, mut n) = (0.0f64, 0, 0);
    for i in 1..=secs {
        let t = 15.0 + i as f64;
        let p = truth(t);
        if vel && i % 4 == 0 {
            n += 1;
            acc += usize::from(tr.update_velocity(ve + g.normal(), vn + g.normal(), 1.0, T_START + t) == Gate::Accepted);
        }
        let cov = random_cov(&mut g, 2, 0.1e-6, 1.0e-6);
        let meas = arrivals(&p, 123.0 + t, rx, &g.mvn(&cov));
        let a = Some((ecef_to_llh(p).alt + ALT_SIGMA * g.normal(), ALT_SIGMA));
        tr.update_tdoa(&meas, &cov, a, T_START + t);
        worst = worst.max(horiz_dist(&tr.position(), &p));
    }
    (worst, acc, n)
}

#[test]
fn comm_b_velocity_keeps_a_two_receiver_track_for_five_minutes() {
    let (mut with_v, mut without, mut acc, mut n) = (0.0f64, 0.0f64, 0, 0);
    for seed in 0..10 {
        let (w, a, m) = two_rx_with_velocity(seed, 300, true);
        with_v = with_v.max(w);
        acc += a;
        n += m;
        without = without.max(two_rx_with_velocity(seed, 300, false).0);
    }
    eprintln!(
        "[track] 2 rx + alt, 300 s, x10: worst horizontal error with BDS 5,0 velocity {with_v:.0} m, \
         without {without:.0} m; velocity accepted {acc}/{n}"
    );
    assert!(with_v < 1_500.0, "{with_v}");
    assert!(with_v < without, "{with_v} vs {without}");
    assert!(acc as f64 >= 0.95 * n as f64, "{acc}/{n}");
}

#[test]
fn a_wrong_velocity_is_rejected() {
    let (rx, fl) = tdoa_setup();
    let _ = rx;
    let mut g = Gauss::new(3);
    let truth = |t: f64| fl.point(180.0 * t, 150.0 * t, 0.0);
    let mut tr = Track::new(&fix_at(&mut g, &truth(0.0), (200.0, 150.0), 60.0, 20.0), T_START);
    for i in 1..=30 {
        tr.update_fix(&fix_at(&mut g, &truth(i as f64), (200.0, 150.0), 60.0, 20.0), T_START + i as f64);
    }
    // A BDS 6,0 read as 5,0, say: 120 m/s off in both components.
    assert!(matches!(tr.update_velocity(60.0, 30.0, 1.0, T_START + 31.0), Gate::Rejected(_)));
    assert_eq!(tr.update_velocity(f64::NAN, 0.0, 1.0, T_START + 31.0), Gate::Skipped);
    assert_eq!(tr.update_velocity(180.0, 150.0, 1.0, T_START + 31.0), Gate::Accepted);
}
