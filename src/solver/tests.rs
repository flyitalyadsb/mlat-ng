//! Solver tests. The Monte-Carlo ones print their statistics; run with
//! `cargo test --release solver -- --nocapture` to see them.

use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

use super::testutil::{arrivals, llh, po_valley, random_cov, Gauss};
use super::*;
use crate::geo::{ecef_to_llh, Enu};

/// Altitude pseudo-measurement sigma used throughout (250 ft, as mlat-server).
const ALT_SIGMA: f64 = 76.2;
const T0: f64 = 1_000.0;

fn solve_one(meas: &[Meas], cov: &DMatrix<f64>, alt: Option<(f64, f64)>, guesses: &[Ecef]) -> Option<Fix> {
    solve(&SolveInput { meas, cov, alt, guesses })
}

/// Squared Mahalanobis distance of the position error under the reported cov.
fn nees(fix: &Fix, truth: &Ecef) -> f64 {
    let e = Vector3::new(fix.pos[0] - truth[0], fix.pos[1] - truth[1], fix.pos[2] - truth[2]);
    let inv = fix.cov.try_inverse().expect("reported covariance must be invertible");
    (e.transpose() * inv * e)[0]
}

fn horiz_err2(fix: &Fix, truth: &Ecef) -> f64 {
    let l = Enu::at(*truth).to_enu(&fix.pos);
    l[0] * l[0] + l[1] * l[1]
}

/// Horizontal (2-D) NEES: the ENU east/north error under the reported
/// horizontal covariance; chi2(2) if calibrated.
fn nees_h(fix: &Fix, truth: &Ecef) -> f64 {
    let f = Enu::at(fix.pos);
    let rot = Matrix3::from_fn(|i, j| f.r[i][j]);
    let c = rot * fix.cov * rot.transpose();
    let l = Enu::at(fix.pos).to_enu(truth);
    let (a, b, d) = (c[(0, 0)], c[(0, 1)], c[(1, 1)]);
    let det = a * d - b * b;
    (d * l[0] * l[0] - 2.0 * b * l[0] * l[1] + a * l[1] * l[1]) / det
}

#[derive(Default)]
struct Mc {
    trials: usize,
    ok: usize,
    nees: Vec<f64>,
    nees_h: Vec<f64>,
    eh_ratio: Vec<f64>,
    chi2: Vec<f64>,
    err_h: Vec<f64>,
    dof: i32,
    dropped: usize,
}

/// chi2(3) quantiles used below.
const CHI2_3_MEDIAN: f64 = 2.365_974;
const CHI2_3_Q95: f64 = 7.814_728;

impl Mc {
    fn mean(v: &[f64]) -> f64 {
        v.iter().sum::<f64>() / v.len().max(1) as f64
    }

    fn median(v: &[f64]) -> f64 {
        let mut s = v.to_vec();
        s.sort_by(f64::total_cmp);
        s.get(s.len() / 2).copied().unwrap_or(f64::NAN)
    }

    fn coverage95(&self) -> f64 {
        self.nees.iter().filter(|&&v| v <= CHI2_3_Q95).count() as f64 / self.nees.len().max(1) as f64
    }

    fn report(&self, name: &str) {
        eprintln!(
            "[mc] {name}: ok {}/{} | NEES mean {:.3} median {:.3} P95 {:.3} (3 / 2.366 / 0.95) | \
             NEES_h mean {:.3} (2) | E[e_h^2/err_h^2] {:.3} (1) | chi2 mean {:.3} (dof {}) | LOO drops {} | median err_h {:.0} m",
            self.ok,
            self.trials,
            Mc::mean(&self.nees),
            Mc::median(&self.nees),
            self.coverage95(),
            Mc::mean(&self.nees_h),
            Mc::mean(&self.eh_ratio),
            Mc::mean(&self.chi2),
            self.dof,
            self.dropped,
            Mc::median(&self.err_h),
        );
    }

    /// Full calibration: the normalised error is chi2(3) and err_h is the RMS
    /// horizontal error. Tolerances are ~4 sigma of the Monte-Carlo sampling
    /// error at N = 2000 (sd of a chi2(3) mean: 0.055; of a 95% coverage:
    /// 0.005) plus a small allowance for linearisation.
    fn check(&self, name: &str, min_ok: f64) {
        self.report(name);
        let ok_rate = self.ok as f64 / self.trials as f64;
        assert!(ok_rate >= min_ok, "{name}: success rate {ok_rate}");
        let m = Mc::mean(&self.nees);
        assert!((2.78..=3.22).contains(&m), "{name}: NEES mean {m}");
        let q95 = self.coverage95();
        assert!((0.93..=0.97).contains(&q95), "{name}: NEES 95% coverage {q95}");
        let mh = Mc::mean(&self.nees_h);
        assert!((1.82..=2.18).contains(&mh), "{name}: horizontal NEES mean {mh}");
        let r = Mc::mean(&self.eh_ratio);
        assert!((0.92..=1.08).contains(&r), "{name}: err_h calibration {r}");
        self.check_chi2(name);
    }

    /// Without altitude the vertical is weakly determined (sigma_u of
    /// 1..10 km) and at realistic noise the range model is visibly nonlinear
    /// over that span: the core of the error distribution is still the
    /// predicted one (median, horizontal calibration) but its tail is heavier.
    /// The full check runs on the same geometry at reduced noise.
    fn check_core(&self, name: &str, min_ok: f64) {
        self.report(name);
        let ok_rate = self.ok as f64 / self.trials as f64;
        assert!(ok_rate >= min_ok, "{name}: success rate {ok_rate}");
        let med = Mc::median(&self.nees);
        assert!((0.88 * CHI2_3_MEDIAN..=1.12 * CHI2_3_MEDIAN).contains(&med), "{name}: NEES median {med}");
        let q95 = self.coverage95();
        assert!(q95 >= 0.9, "{name}: NEES 95% coverage {q95}");
        // err_h may err on the conservative side here (0.89..0.95 for 4
        // receivers across seeds: the draws whose lobes merge are refused,
        // and they are the ones with the largest errors).
        let r = Mc::mean(&self.eh_ratio);
        assert!((0.85..=1.1).contains(&r), "{name}: err_h calibration {r}");
    }

    fn check_chi2(&self, name: &str) {
        if self.dof > 0 {
            let d = self.dof as f64;
            let c = Mc::mean(&self.chi2);
            let tol = 0.05 * d + 4.0 * (2.0 * d / self.chi2.len() as f64).sqrt();
            assert!((c - d).abs() <= tol, "{name}: mean chi2 {c} vs dof {d}");
        }
    }
}

/// Monte-Carlo over random truths and random correlated covariances; `scale`
/// multiplies every noise sigma (timing and altitude).
fn monte_carlo(
    seed: u64,
    trials: usize,
    rx: &[Ecef],
    with_alt: bool,
    scale: f64,
    mut truth: impl FnMut(&mut Gauss) -> Ecef,
    mut guess: impl FnMut(&mut Gauss, &Ecef) -> Ecef,
) -> Mc {
    let mut g = Gauss::new(seed);
    let k = rx.len();
    let mut mc = Mc { trials, dof: (k + usize::from(with_alt)) as i32 - 4, ..Default::default() };
    for _ in 0..trials {
        let p = truth(&mut g);
        let cov = random_cov(&mut g, k, 0.1e-6 * scale, 1.5e-6 * scale);
        let noise = g.mvn(&cov);
        let meas = arrivals(&p, T0, rx, &noise);
        let alt = with_alt.then(|| (ecef_to_llh(p).alt + ALT_SIGMA * scale * g.normal(), ALT_SIGMA * scale));
        let gs = [guess(&mut g, &p)];
        let Some(fix) = solve_one(&meas, &cov, alt, &gs) else { continue };
        mc.ok += 1;
        if fix.used.iter().any(|u| !u) {
            mc.dropped += 1;
        }
        mc.nees.push(nees(&fix, &p));
        mc.nees_h.push(nees_h(&fix, &p));
        mc.eh_ratio.push(horiz_err2(&fix, &p) / (fix.err_h * fix.err_h));
        mc.chi2.push(fix.chi2);
        mc.err_h.push(fix.err_h);
    }
    mc
}

/// An aircraft somewhere over the Po valley network at cruise-ish altitude.
fn po_truth(g: &mut Gauss) -> Ecef {
    llh(g.uniform(44.6, 45.4), g.uniform(8.0, 11.2), g.uniform(3_000.0, 12_000.0))
}

/// A previous-position style guess: tens of km off, wrong altitude.
fn warm_guess(g: &mut Gauss, truth: &Ecef) -> Ecef {
    let l = ecef_to_llh(*truth);
    llh(l.lat + 0.2 * g.normal(), l.lon + 0.25 * g.normal(), g.uniform(0.0, 12_000.0))
}

/// Cold start: the receivers' centroid on the ground (no history at all).
fn centroid_guess(rx: &[Ecef]) -> Ecef {
    let mut c = [0.0; 3];
    for r in rx {
        for i in 0..3 {
            c[i] += r[i] / rx.len() as f64;
        }
    }
    c
}

/// The engine's usual guess: a track prediction a few km off.
fn track_guess(g: &mut Gauss, truth: &Ecef) -> Ecef {
    let l = ecef_to_llh(*truth);
    llh(l.lat + 0.03 * g.normal(), l.lon + 0.04 * g.normal(), l.alt + 300.0 * g.normal())
}

#[test]
fn mc_three_receivers_plus_altitude() {
    // Sparse network: Milan, Turin, Bologna + Mode S altitude; dof 0, so
    // both lobes fit exactly and the guess (a track prediction) picks one.
    let rx = po_valley(3);
    let truth = |g: &mut Gauss| llh(g.uniform(44.7, 45.3), g.uniform(8.3, 10.8), g.uniform(5_000.0, 12_000.0));
    monte_carlo(101, 2000, &rx, true, 1.0, truth, track_guess).check("3 rx + alt (Milan/Turin/Bologna)", 0.99);
}

#[test]
fn three_receivers_plus_altitude_ambiguity_is_refused_not_guessed() {
    // Same geometry with a poor guess (~30 km off). Where both lobes are
    // valid and the guess does not clearly favour one, the solver must
    // return None rather than a confident fix in the wrong lobe.
    let rx = po_valley(3);
    let mut g = Gauss::new(111);
    let (mut ok, mut wrong, mut none) = (0, 0, 0);
    for _ in 0..4000 {
        let p = llh(g.uniform(44.7, 45.3), g.uniform(8.3, 10.8), g.uniform(5_000.0, 12_000.0));
        let cov = random_cov(&mut g, 3, 0.1e-6, 1.5e-6);
        let meas = arrivals(&p, T0, &rx, &g.mvn(&cov));
        let alt = Some((ecef_to_llh(p).alt + ALT_SIGMA * g.normal(), ALT_SIGMA));
        match solve_one(&meas, &cov, alt, &[warm_guess(&mut g, &p)]) {
            None => none += 1,
            Some(f) if nees(&f, &p) > 100.0 => wrong += 1,
            Some(_) => ok += 1,
        }
    }
    eprintln!("[mc] 3 rx + alt, guess ~30 km off: {ok} fixes, {none} refused, {wrong} in the wrong lobe (of 4000)");
    assert!(wrong <= 4, "{wrong}");
    assert!(ok >= 3800, "{ok}");
}

#[test]
fn mc_four_receivers() {
    // Milan, Turin, Bologna, Genoa without altitude: dof 0 and a weak
    // vertical. In ~1/6 of the draws the hyperboloids do not intersect at
    // all (the least-squares point is where the two lobes merge, with a
    // singular normal matrix) and the solver rightly returns None.
    let rx = po_valley(4);
    let truth = |g: &mut Gauss| llh(g.uniform(44.6, 45.3), g.uniform(8.3, 10.8), g.uniform(8_000.0, 12_000.0));
    monte_carlo(102, 2000, &rx, false, 1.0, truth, warm_guess).check_core("4 rx", 0.75);
    // dof 0 and a vertical sigma of km: linear only at GPS-grade timing.
    monte_carlo(202, 2000, &rx, false, 0.01, truth, warm_guess).check("4 rx, noise/100", 0.99);
}

#[test]
fn mc_six_receivers_european() {
    // The realistic case: six receivers around Milan/Turin/Bologna, aircraft
    // at FL360 (11 km), cold start from the receivers' centroid.
    let rx = po_valley(6);
    let c = centroid_guess(&rx);
    let truth = |g: &mut Gauss| llh(g.uniform(44.6, 45.4), g.uniform(8.0, 11.2), 11_000.0);
    monte_carlo(104, 2000, &rx, true, 1.0, truth, |_, _| c).check("6 rx, 11 km, + alt", 0.995);
    // Without altitude a few estimates fall below -500 m (sigma_u ~ 2 km)
    // and are rejected, as specified.
    monte_carlo(103, 2000, &rx, false, 1.0, truth, |_, _| c).check_core("6 rx, 11 km, no alt", 0.98);
    monte_carlo(203, 2000, &rx, false, 0.03, truth, |_, _| c).check("6 rx, 11 km, no alt, noise/30", 0.995);
}

#[test]
fn mc_twelve_receivers() {
    let rx = po_valley(12);
    let c = centroid_guess(&rx);
    monte_carlo(106, 2000, &rx, true, 1.0, po_truth, |_, _| c).check("12 rx + alt", 0.995);
    // 3..12 km without altitude: the low ones have sigma_u ~ 4 km and ~2%
    // of their estimates fall below -500 m.
    monte_carlo(105, 2000, &rx, false, 1.0, po_truth, |_, _| c).check_core("12 rx", 0.97);
    monte_carlo(205, 2000, &rx, false, 0.03, po_truth, |_, _| c).check("12 rx, noise/30", 0.995);
}

#[test]
fn mc_cold_start_from_a_receiver() {
    // mlat-server's cold start: the earliest receiver's position as the only
    // guess. The algebraic starts make this as reliable as a warm guess.
    let rx = po_valley(8);
    let mut g = Gauss::new(107);
    let (mut ok, mut near) = (0, 0);
    for _ in 0..1000 {
        let p = po_truth(&mut g);
        let cov = random_cov(&mut g, 8, 0.1e-6, 1.5e-6);
        let meas = arrivals(&p, T0, &rx, &g.mvn(&cov));
        let first = (0..8).min_by(|&a, &b| meas[a].t.total_cmp(&meas[b].t)).unwrap();
        if let Some(f) = solve_one(&meas, &cov, None, &[rx[first]]) {
            ok += 1;
            near += usize::from(nees(&f, &p) < 50.0);
        }
    }
    eprintln!("[mc] cold start from the earliest receiver: {ok}/1000 solved, {near} within NEES 50");
    // Same acceptance as from a warm guess (a few low aircraft without
    // altitude are rejected below -500 m), and no wrong-basin answers.
    assert!(ok >= 970 && near + 3 >= ok, "{ok} {near}");
}

/// Two clock-sync clusters: the western receivers reach the common timebase
/// through one reference clock, the eastern ones through another, so errors
/// are strongly correlated inside each cluster (0.8) and independent across.
fn clustered_cov(g: &mut Gauss, west: &[bool]) -> DMatrix<f64> {
    let s: Vec<f64> = west.iter().map(|_| g.uniform(0.2e-6, 1.0e-6)).collect();
    DMatrix::from_fn(west.len(), west.len(), |i, j| {
        let rho = if i == j { 1.0 } else if west[i] == west[j] { 0.8 } else { 0.0 };
        rho * s[i] * s[j]
    })
}

#[test]
fn full_covariance_beats_diagonal_weighting() {
    // Same data solved with the true covariance and with its diagonal only.
    // Inside a cluster the shared error cancels in the time differences, so
    // the full-covariance fit leans on the precise intra-cluster differences
    // and is clearly more accurate, while staying calibrated.
    let rx = po_valley(6);
    let west = [true, true, false, true, false, false]; // Milan Turin | Bologna | Genoa | Verona Parma
    let c = centroid_guess(&rx);
    let mut g = Gauss::new(7);
    let (mut e_full, mut e_diag, mut nees_full, mut n) = (0.0, 0.0, 0.0, 0);
    for _ in 0..2000 {
        let p = llh(g.uniform(44.6, 45.4), g.uniform(8.0, 11.2), 11_000.0);
        let cov = clustered_cov(&mut g, &west);
        let diag = DMatrix::from_diagonal(&cov.diagonal());
        let meas = arrivals(&p, T0, &rx, &g.mvn(&cov));
        let alt = Some((11_000.0 + ALT_SIGMA * g.normal(), ALT_SIGMA));
        let (Some(a), Some(b)) = (solve_one(&meas, &cov, alt, &[c]), solve_one(&meas, &diag, alt, &[c])) else { continue };
        e_full += horiz_err2(&a, &p);
        e_diag += horiz_err2(&b, &p);
        nees_full += nees(&a, &p);
        n += 1;
    }
    let n = n as f64;
    let (rf, rd) = ((e_full / n).sqrt(), (e_diag / n).sqrt());
    let nf = nees_full / n;
    eprintln!("[mc] clustered clocks (rho 0.8), full vs diagonal weighting: horizontal RMS {rf:.0} m vs {rd:.0} m; full NEES mean {nf:.2}");
    assert!(n >= 1990.0);
    assert!(rf < 0.85 * rd, "full {rf} diag {rd}");
    assert!((2.78..=3.22).contains(&nf), "{nf}");
}

#[test]
fn exact_data_recovers_truth_and_t0() {
    let rx = po_valley(6);
    let p = llh(45.1, 9.6, 10_500.0);
    let cov = DMatrix::from_diagonal_element(6, 6, (0.3e-6f64).powi(2));
    let meas = arrivals(&p, T0, &rx, &DVector::zeros(6));
    let fix = solve_one(&meas, &cov, None, &[centroid_guess(&rx)]).unwrap();
    assert!(geo::dist(&fix.pos, &p) < 0.01, "{}", geo::dist(&fix.pos, &p));
    assert!((fix.t0 - T0).abs() < 1e-10);
    assert!(fix.chi2 < 1e-9);
    assert_eq!(fix.dof, 2);
    assert!(fix.used.iter().all(|&u| u));
    assert!(fix.gdop.is_finite() && fix.gdop > 1.0);
    // err_h is the root of the horizontal trace, consistent with cov.
    let r = Enu::at(fix.pos).r;
    let rot = Matrix3::from_fn(|i, j| r[i][j]);
    let enu = rot * fix.cov * rot.transpose();
    assert!(((enu[(0, 0)] + enu[(1, 1)]).sqrt() - fix.err_h).abs() < 1e-6 * fix.err_h);
}

#[test]
fn deterministic() {
    let rx = po_valley(8);
    let mut g = Gauss::new(3);
    let p = po_truth(&mut g);
    let cov = random_cov(&mut g, 8, 0.1e-6, 1.5e-6);
    let meas = arrivals(&p, T0, &rx, &g.mvn(&cov));
    let gs = [centroid_guess(&rx), rx[2]];
    let alt = Some((ecef_to_llh(p).alt, ALT_SIGMA));
    let a = solve_one(&meas, &cov, alt, &gs).unwrap();
    let b = solve_one(&meas, &cov, alt, &gs).unwrap();
    assert_eq!(a.pos, b.pos);
    assert_eq!(a.chi2.to_bits(), b.chi2.to_bits());
    assert_eq!(a.cov, b.cov);
}

/// Four receivers strung along a valley (nearly collinear, ~3 km off the
/// line), aircraft 60 km north of it, altitude known (dof 1). The ghost lobe
/// is the mirror image south of the line.
fn valley_case() -> (Vec<Ecef>, Ecef, Ecef) {
    let rx = vec![llh(45.00, 8.00, 200.0), llh(45.03, 8.70, 150.0), llh(44.98, 9.40, 100.0), llh(45.02, 10.10, 80.0)];
    let truth = llh(45.55, 9.05, 10_000.0);
    let ghost_side_guess = llh(44.45, 9.05, 10_000.0);
    (rx, truth, ghost_side_guess)
}

#[test]
fn mirror_start_finds_the_true_lobe() {
    let (rx, truth, guess) = valley_case();
    let cov = DMatrix::from_diagonal_element(4, 4, (0.1e-6f64).powi(2));
    let meas = arrivals(&truth, T0, &rx, &DVector::zeros(4));
    let alt = Some((ecef_to_llh(truth).alt, ALT_SIGMA));
    let inp = SolveInput { meas: &meas, cov: &cov, alt, guesses: &[guess] };

    // Without mirrors, the start south of the line settles in the ghost lobe:
    // proves the case is a real two-lobe ambiguity, not a trivial one.
    let all: Vec<usize> = (0..4).collect();
    let mut plain = Problem::new(&inp, &all).unwrap();
    plain.mirrors.clear();
    let ghost = plain.multistart(&[guess], &guess, false).best.expect("ghost lobe is a valid-looking fit");
    assert!(geo::dist(&ghost.pos, &truth) > 50e3, "ghost {:?}", ecef_to_llh(ghost.pos));
    assert!(ghost.chi2 > 1.0, "ghost chi2 {}", ghost.chi2);

    // Mirrors alone (no algebraic starts) recover the true lobe ...
    let mut mirrored = Problem::new(&inp, &all).unwrap();
    assert!(!mirrored.mirrors.is_empty());
    let c = mirrored.multistart(&[guess], &guess, false).best.unwrap();
    assert!(geo::dist(&c.pos, &truth) < 1.0, "{:?}", ecef_to_llh(c.pos));

    // ... and so does the full solver.
    let fix = solve(&inp).unwrap();
    assert!(geo::dist(&fix.pos, &truth) < 1.0, "{:?}", ecef_to_llh(fix.pos));
    assert!(fix.chi2 < 1e-6);
}

#[test]
fn mirror_with_noise_prefers_lower_chi2() {
    // Same ambiguity with realistic noise: over many draws the solver picks
    // the true lobe from the wrong-side guess nearly always.
    let (rx, truth, guess) = valley_case();
    let mut g = Gauss::new(17);
    let (mut right, mut n) = (0, 0);
    for _ in 0..300 {
        let cov = random_cov(&mut g, 4, 0.1e-6, 0.5e-6);
        let meas = arrivals(&truth, T0, &rx, &g.mvn(&cov));
        let alt = Some((ecef_to_llh(truth).alt + ALT_SIGMA * g.normal(), ALT_SIGMA));
        if let Some(f) = solve_one(&meas, &cov, alt, &[guess]) {
            n += 1;
            right += usize::from(geo::dist(&f.pos, &truth) < 20e3);
        }
    }
    eprintln!("[mc] mirror with noise: true lobe {right}/{n}");
    assert!(n >= 290 && right as f64 >= 0.95 * n as f64, "{right}/{n}");
}

#[test]
fn outlier_is_removed_by_leave_one_out() {
    let rx = po_valley(8);
    let c = centroid_guess(&rx);
    let mut g = Gauss::new(23);
    let (mut found, mut n, mut nees_sum) = (0, 0, 0.0);
    for trial in 0..300 {
        let p = po_truth(&mut g);
        let cov = random_cov(&mut g, 8, 0.1e-6, 1.0e-6);
        let mut noise = g.mvn(&cov);
        let bad = trial % 8;
        // 5..50 us: a multipath echo or a clock glitch (1.5..15 km).
        noise[bad] += g.uniform(5e-6, 50e-6) * if trial % 2 == 0 { 1.0 } else { -1.0 };
        let meas = arrivals(&p, T0, &rx, &noise);
        let Some(fix) = solve_one(&meas, &cov, None, &[c]) else { continue };
        n += 1;
        if !fix.used[bad] && fix.used.iter().filter(|&&u| !u).count() == 1 {
            found += 1;
            nees_sum += nees(&fix, &p);
            assert!(fix.chi2 <= chi2_q99(fix.dof));
            assert_eq!(fix.dof, 3);
        }
    }
    eprintln!("[mc] outlier LOO: identified {found}/{n} (of 300), NEES mean after removal {:.2}", nees_sum / found as f64);
    assert!(n >= 295, "{n}");
    assert!(found as f64 >= 0.97 * n as f64, "{found}/{n}");
    let m = nees_sum / found as f64;
    assert!((2.4..=3.6).contains(&m), "{m}");
}

#[test]
fn inconsistent_data_without_loo_inflates_covariance() {
    // k = 5 without altitude: dof 1, LOO would leave dof 0, so the bad fit is
    // returned with its covariance scaled by chi2/dof.
    let rx = po_valley(5);
    let p = llh(45.0, 9.5, 11_000.0);
    let cov = DMatrix::from_diagonal_element(5, 5, (0.2e-6f64).powi(2));
    let mut noise = DVector::zeros(5);
    noise[4] = 3e-6;
    let meas = arrivals(&p, T0, &rx, &noise);
    let gs = [centroid_guess(&rx)];
    let fix = solve_one(&meas, &cov, None, &gs).unwrap();
    assert!(fix.chi2 > chi2_q99(1));
    assert!(fix.used.iter().all(|&u| u));
    let inp = SolveInput { meas: &meas, cov: &cov, alt: None, guesses: &gs };
    let raw = Problem::new(&inp, &[0, 1, 2, 3, 4]).unwrap().multistart(&gs, &gs[0], true).best.unwrap();
    let s = fix.chi2 / fix.dof as f64;
    assert!(((fix.cov[(0, 0)] / raw.cov3[(0, 0)]) - s).abs() < 1e-9 * s);
    assert!((fix.err_h / raw.err_h - s.sqrt()).abs() < 1e-9);
}

#[test]
fn non_positive_definite_covariance_falls_back_to_diagonal() {
    let rx = po_valley(6);
    let p = llh(45.0, 9.5, 11_000.0);
    // "Correlations" of 1.5: not a covariance. The diagonal still is.
    let s2 = (0.3e-6f64).powi(2);
    let cov = DMatrix::from_fn(6, 6, |i, j| if i == j { s2 } else { 1.5 * s2 });
    let meas = arrivals(&p, T0, &rx, &DVector::zeros(6));
    let fix = solve_one(&meas, &cov, None, &[centroid_guess(&rx)]).unwrap();
    assert!(geo::dist(&fix.pos, &p) < 0.01);
    let diag = DMatrix::from_diagonal_element(6, 6, s2);
    let reference = solve_one(&meas, &diag, None, &[centroid_guess(&rx)]).unwrap();
    assert!((fix.cov - reference.cov).norm() < 1e-6 * reference.cov.norm());
}

#[test]
fn degenerate_inputs_return_none() {
    let rx = po_valley(6);
    let p = llh(45.0, 9.5, 11_000.0);
    let cov = DMatrix::from_diagonal_element(6, 6, (0.3e-6f64).powi(2));
    let good = arrivals(&p, T0, &rx, &DVector::zeros(6));
    let gs = [centroid_guess(&rx)];
    assert!(solve_one(&good, &cov, None, &gs).is_some());

    // No guesses, or only unusable ones.
    assert!(solve_one(&good, &cov, None, &[]).is_none());
    assert!(solve_one(&good, &cov, None, &[[f64::NAN, 0.0, 0.0]]).is_none());
    assert!(solve_one(&good, &cov, None, &[[0.0, 0.0, 0.0]]).is_none());
    // ... but a bad guess next to a good one is just skipped.
    assert!(solve_one(&good, &cov, None, &[[f64::NAN, 0.0, 0.0], gs[0]]).is_some());

    // NaN / infinite anywhere in the data.
    let mut m = good.clone();
    m[2].t = f64::NAN;
    assert!(solve_one(&m, &cov, None, &gs).is_none());
    let mut m = good.clone();
    m[1].rx[0] = f64::INFINITY;
    assert!(solve_one(&m, &cov, None, &gs).is_none());
    let mut c = cov.clone();
    c[(1, 2)] = f64::NAN;
    assert!(solve_one(&good, &c, None, &gs).is_none());
    assert!(solve_one(&good, &cov, Some((f64::NAN, 50.0)), &gs).is_none());
    assert!(solve_one(&good, &cov, Some((10_000.0, 0.0)), &gs).is_none());

    // Shape errors and too few measurements.
    assert!(solve_one(&good, &DMatrix::zeros(5, 5), None, &gs).is_none());
    assert!(solve_one(&good[..3], &cov.view((0, 0), (3, 3)).into_owned(), None, &gs).is_none());
    assert!(solve_one(&[], &DMatrix::zeros(0, 0), Some((1e4, 50.0)), &gs).is_none());
    // Zero variances on the diagonal: neither factor nor fallback exists.
    assert!(solve_one(&good, &DMatrix::zeros(6, 6), None, &gs).is_none());

    // Identical receiver positions.
    let same: Vec<Meas> = (0..5).map(|i| Meas { rx: rx[0], t: T0 + 1e-4 + i as f64 * 1e-7 }).collect();
    let c5 = DMatrix::from_diagonal_element(5, 5, (0.3e-6f64).powi(2));
    assert!(solve_one(&same, &c5, None, &gs).is_none());
    assert!(solve_one(&same, &c5, Some((10_000.0, ALT_SIGMA)), &gs).is_none());
}

#[test]
fn collinear_receivers_return_none() {
    // Five receivers exactly on a straight ECEF line: the solution is only
    // known up to a rotation about that line.
    let a = llh(45.0, 8.0, 100.0);
    let b = llh(45.0, 10.0, 100.0);
    let rx: Vec<Ecef> = (0..5).map(|i| {
        let s = i as f64 / 4.0;
        [a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1]), a[2] + s * (b[2] - a[2])]
    }).collect();
    let p = llh(45.4, 9.1, 11_000.0);
    let cov = DMatrix::from_diagonal_element(5, 5, (0.3e-6f64).powi(2));
    let mut g = Gauss::new(5);
    for _ in 0..20 {
        let meas = arrivals(&p, T0, &rx, &g.mvn(&cov));
        assert!(solve_one(&meas, &cov, None, &[llh(45.2, 9.0, 5_000.0)]).is_none());
    }
}

#[test]
fn implausible_positions_are_rejected() {
    let rx = po_valley(6);
    let cov = DMatrix::from_diagonal_element(6, 6, (0.3e-6f64).powi(2));
    let zero = DVector::zeros(6);
    let gs = [centroid_guess(&rx)];

    // Too high: a 32 km "aircraft" (balloon or a broken group).
    let p = llh(45.0, 9.5, 32_000.0);
    assert!(solve_one(&arrivals(&p, T0, &rx, &zero), &cov, None, &gs).is_none());

    // Too far: 700+ km from every receiver (over Corsica's far side).
    let p = llh(39.0, 8.0, 11_000.0);
    assert!(solve_one(&arrivals(&p, T0, &rx, &zero), &cov, None, &[p]).is_none());

    // Hopeless geometry: a compact 4-receiver cluster (~15 km) and an
    // aircraft 250 km away: the horizontal ellipse is far beyond 20 km.
    let cluster = vec![llh(45.00, 9.00, 100.0), llh(45.10, 9.05, 120.0), llh(45.02, 9.18, 90.0), llh(45.08, 9.12, 300.0)];
    let p = llh(47.2, 9.3, 11_000.0);
    let c4 = DMatrix::from_diagonal_element(4, 4, (0.5e-6f64).powi(2));
    let meas = arrivals(&p, T0, &cluster, &DVector::zeros(4));
    assert!(solve_one(&meas, &c4, Some((11_000.0, ALT_SIGMA)), &[p]).is_none());
}

#[test]
fn t0_after_earliest_arrival_is_rejected() {
    let rx = po_valley(6);
    let p = llh(45.0, 9.5, 11_000.0);
    let cov = DMatrix::from_diagonal_element(6, 6, (0.3e-6f64).powi(2));
    let meas = arrivals(&p, T0, &rx, &DVector::zeros(6));
    let inp = SolveInput { meas: &meas, cov: &cov, alt: None, guesses: &[p] };
    let all: Vec<usize> = (0..6).collect();
    let mut prob = Problem::new(&inp, &all).unwrap();
    let start = prob.to_rel(&p);
    let mut sol = prob.fit(start).unwrap();
    assert!(prob.validate(&sol).is_some());
    // Same geometry, transmit time pushed past the earliest arrival by more
    // than the tolerance: physically impossible, must be refused.
    sol.x[3] = (T0_TOL_SIGMA + 0.5) * prob.sigma_first;
    assert!(prob.validate(&sol).is_none());
    sol.x[3] = 0.5 * prob.sigma_first;
    assert!(prob.validate(&sol).is_some());
}

#[test]
fn ground_target_next_to_a_receiver() {
    // A taxiing aircraft 300 m from the Milan receiver: t0 is within noise of
    // the earliest arrival, which must not trip the t0 check.
    let rx = po_valley(6);
    let p = llh(45.466, 9.193, 120.0);
    let mut g = Gauss::new(31);
    let mut ok = 0;
    for _ in 0..200 {
        let cov = random_cov(&mut g, 6, 0.1e-6, 0.5e-6);
        let meas = arrivals(&p, T0, &rx, &g.mvn(&cov));
        ok += usize::from(solve_one(&meas, &cov, Some((120.0, ALT_SIGMA)), &[rx[0]]).is_some());
    }
    assert!(ok >= 195, "{ok}/200");
}

fn bench(k: usize, n: usize) -> (f64, f64, f64) {
    let rx = po_valley(12);
    let rx: Vec<Ecef> = (0..k)
        .map(|i| {
            // Beyond 12, shift the Po valley set by ~30 km to make new sites.
            let l = ecef_to_llh(rx[i % 12]);
            let j = (i / 12) as f64;
            llh(l.lat + 0.27 * j, l.lon - 0.31 * j, l.alt)
        })
        .collect();
    let mut g = Gauss::new(99);
    // (clean arrivals, arrivals with a 20 us outlier, cov, cold guess, warm guess)
    type Case = (Vec<Meas>, Vec<Meas>, DMatrix<f64>, Ecef, Ecef);
    let cases: Vec<Case> = (0..64)
        .map(|i| {
            let p = po_truth(&mut g);
            let cov = random_cov(&mut g, k, 0.1e-6, 1.5e-6);
            let noise = g.mvn(&cov);
            let meas = arrivals(&p, T0, &rx, &noise);
            let mut bad = meas.clone();
            bad[i % k].t += 20e-6;
            let warm = warm_guess(&mut g, &p);
            (meas, bad, cov, centroid_guess(&rx), warm)
        })
        .collect();
    let time = |mode: u8| {
        let t = std::time::Instant::now();
        let mut ok = 0;
        for i in 0..n {
            let (m, bad, c, cold, warm) = &cases[i % cases.len()];
            let (meas, gs) = match mode {
                0 => (m, [*cold]),
                1 => (m, [*warm]),
                _ => (bad, [*warm]),
            };
            ok += usize::from(std::hint::black_box(solve_one(meas, c, None, &gs)).is_some());
        }
        assert!(ok > n * 9 / 10);
        t.elapsed().as_secs_f64() / n as f64 * 1e6
    };
    (time(0), time(1), time(2))
}

#[test]
#[ignore = "timing; run with --release -- --ignored --nocapture"]
fn bench_solve() {
    for k in [8, 20] {
        let (cold, warm, outlier) = bench(k, 20_000);
        eprintln!(
            "[bench] solve k={k}: {cold:.1} us (cold start at the centroid), {warm:.1} us (warm guess), \
             {outlier:.1} us (one 20 us outlier: leave-one-out path)"
        );
    }
}
