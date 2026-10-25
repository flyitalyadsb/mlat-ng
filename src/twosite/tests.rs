use super::*;
use crate::geo::{llh_to_ecef, Llh};

/// Deterministic N(0, 1)-ish noise (sum of uniforms).
struct Noise(u64);
impl Noise {
    fn next(&mut self) -> f64 {
        let mut s = 0.0;
        for _ in 0..12 {
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            s += (self.0 >> 11) as f64 / (1u64 << 53) as f64;
        }
        s - 6.0
    }
}

fn site(lat: f64, lon: f64) -> Ecef {
    llh_to_ecef(Llh { lat, lon, alt: 200.0 })
}

/// Records every 5 s over `span` seconds of an aircraft that ends at
/// (lat, lon, h) flying at `vel` (east, north) m/s; TDOA noise `sd` metres.
fn flight(ra: Ecef, rb: Ecef, end: Llh, vel: [f64; 2], span: f64, sd: f64, seed: u64) -> Vec<Rec2> {
    let enu = Enu::at(llh_to_ecef(end));
    let mut z = Noise(seed);
    let t_end = 1_000.0;
    let mut out = Vec::new();
    let mut t = t_end - span;
    while t <= t_end + 1e-9 {
        let dt = t_end - t;
        let mut l = ecef_to_llh(enu.to_ecef(&[-vel[0] * dt, -vel[1] * dt, 0.0]));
        l.alt = end.alt;
        let p = llh_to_ecef(l);
        let off = [vel[0] * dt, vel[1] * dt];
        out.push(Rec2 { t, off, ra, rb, d: dist(&p, &rb) - dist(&p, &ra) + sd * z.next(), sd, h: end.alt });
        t += 5.0;
    }
    out
}

fn err_km(s: &Solution, truth: Llh) -> f64 {
    dist(&s.pos, &llh_to_ecef(truth)) / 1000.0
}

#[test]
fn motion_pins_the_point_on_the_hyperbola() {
    let (ra, rb) = (site(45.0, 9.0), site(45.0, 11.0)); // ~157 km apart
    let end = Llh { lat: 45.8, lon: 10.4, alt: 11_000.0 };
    let recs = flight(ra, rb, end, [150.0, 170.0], 240.0, 40.0, 7);
    let s = solve(&recs, &|_, _| 0.0).expect("a solution");
    assert!(err_km(&s, end) < 1.5, "error {:.2} km", err_km(&s, end));
    assert!(s.rms < 2.0, "rms {}", s.rms);
    assert!(s.err_h < 3000.0, "err_h {}", s.err_h);
    assert_eq!(s.n, recs.len());
    assert!((s.span - 240.0).abs() < 1e-6);
}

#[test]
fn a_velocity_along_the_baseline_leaves_a_mirror_that_coverage_resolves() {
    let (ra, rb) = (site(45.0, 9.0), site(45.0, 11.0));
    let end = Llh { lat: 45.9, lon: 10.1, alt: 10_000.0 };
    let vel = [230.0, 0.0];
    let recs = flight(ra, rb, end, vel, 240.0, 40.0, 11);
    // Timing alone: both sides of the baseline fit about as well.
    let s = solve(&recs, &|_, _| 0.0).unwrap();
    assert!(s.margin < 5.0, "margin {}", s.margin);
    // The receivers hear north of the baseline, never south of it.
    let cov = |lat: f64, _lon: f64| if lat > 45.0 { 0.0 } else { 40.0 };
    let s = solve(&recs, &cov).unwrap();
    assert!(err_km(&s, end) < 2.0, "error {:.2} km", err_km(&s, end));
    assert!(s.margin > 20.0, "margin {}", s.margin);
}

#[test]
fn a_wrong_velocity_shows_in_the_residuals() {
    let (ra, rb) = (site(45.0, 9.0), site(45.0, 11.0));
    let end = Llh { lat: 45.8, lon: 10.4, alt: 11_000.0 };
    let recs = flight(ra, rb, end, [150.0, 170.0], 240.0, 40.0, 3);
    let good = solve(&recs, &|_, _| 0.0).unwrap();
    let wrong: Vec<Rec2> = recs.iter().map(|r| Rec2 { off: [-r.off[1], r.off[0]], ..r.clone() }).collect();
    let bad = solve(&wrong, &|_, _| 0.0).unwrap();
    assert!(bad.rms > good.rms * 2.0, "good {} bad {}", good.rms, bad.rms);
}

#[test]
fn too_few_records_give_nothing() {
    let (ra, rb) = (site(45.0, 9.0), site(45.0, 11.0));
    let end = Llh { lat: 45.8, lon: 10.4, alt: 11_000.0 };
    let recs = flight(ra, rb, end, [150.0, 170.0], 10.0, 40.0, 1);
    assert!(recs.len() < 4);
    assert!(solve(&recs, &|_, _| 0.0).is_none());
}

#[test]
fn far_from_the_origin_the_local_north_is_rotated() {
    // Sites ~157 km apart, aircraft ~300 km east of their midpoint, 240 s at
    // 230 m/s: without the meridian convergence (~2.7 deg) the path would
    // be rotated and the end point off by ~2.6 km.
    let (ra, rb) = (site(45.0, 9.0), site(45.0, 11.0));
    let end = Llh { lat: 45.6, lon: 13.9, alt: 11_000.0 };
    let recs = flight(ra, rb, end, [30.0, 228.0], 240.0, 30.0, 5);
    let s = solve(&recs, &|_, _| 0.0).unwrap();
    assert!(err_km(&s, end) < 1.0, "error {:.2} km", err_km(&s, end));
}

#[test]
fn offsets_follow_a_turn() {
    // 100 s east at 200 m/s, then 100 s north at 200 m/s.
    let hist = [([200.0, 0.0], 0.0), ([0.0, 200.0], 100.0)];
    let off = offsets(&[0.0, 50.0, 150.0, 200.0], &hist, 200.0);
    assert_eq!(off[0], [20_000.0, 20_000.0]);
    assert_eq!(off[1], [10_000.0, 20_000.0]);
    assert_eq!(off[2], [0.0, 10_000.0]);
    assert_eq!(off[3], [0.0, 0.0]);
    // Before the first sample its velocity holds.
    assert_eq!(offsets(&[-10.0], &hist, 0.0)[0], [2_000.0, 0.0]);
}
