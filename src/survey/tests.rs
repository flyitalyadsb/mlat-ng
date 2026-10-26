use super::*;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

/// Unit vector towards an aircraft at azimuth `az` (deg), elevation `el`.
fn dir(az: f64, el: f64) -> [f64; 3] {
    let (a, e) = (az.to_radians(), el.to_radians());
    [e.cos() * a.sin(), e.cos() * a.cos(), e.sin()]
}

/// Observations of receiver 1 against `partners` for `secs` seconds, with a
/// true position error `d`, Gaussian noise `noise` m, and per-pair clock
/// offsets drifting along.
fn feed(s: &mut Survey, d: [f64; 3], noise: f64, secs: f64, azimuths: &[f64], rng: &mut StdRng) {
    let partners = [2u64, 3, 4, 5, 6];
    let mut t = 0.0;
    while t < secs {
        for &k in &partners {
            let az = azimuths[rng.gen_range(0..azimuths.len())] + rng.gen_range(-10.0..10.0);
            let u = dir(az, rng.gen_range(1.0..8.0));
            let offset = 3.0e14 * k as f64 + 3.0e4 * t; // m: raw counter offsets, 100 ppm drift
            let z = offset + u[0] * d[0] + u[1] * d[1] + u[2] * d[2] + noise * rng.gen_range(-1.7..1.7);
            s.observe(1, k, t, t, z, u);
        }
        t += 0.5;
    }
    s.sweep(t + WINDOW + 1.0);
}

#[test]
fn recovers_a_horizontal_offset_from_all_round_geometry() {
    let mut rng = StdRng::seed_from_u64(1);
    let mut s = Survey::default();
    let truth = [300.0, -200.0, 0.0];
    feed(&mut s, truth, 50.0, 1800.0, &[0.0, 90.0, 180.0, 270.0], &mut rng);
    let e = s.estimate(1).unwrap();
    assert!((e.d[0] - truth[0]).abs() < 15.0 && (e.d[1] - truth[1]).abs() < 15.0, "{e:?}");
    assert!(e.sd[0] < 10.0 && e.sd_weak_h < 10.0, "{e:?}");
    assert!((e.sigma - 50.0).abs() < 15.0, "{e:?}");
}

#[test]
fn a_correct_position_stays_near_zero() {
    let mut rng = StdRng::seed_from_u64(2);
    let mut s = Survey::default();
    feed(&mut s, [0.0; 3], 50.0, 1800.0, &[0.0, 120.0, 240.0], &mut rng);
    let e = s.estimate(1).unwrap();
    assert!(e.d[0].hypot(e.d[1]) < 15.0, "{e:?}");
}

#[test]
fn one_sided_geometry_is_reported_as_weak() {
    let mut rng = StdRng::seed_from_u64(3);
    let mut s = Survey::default();
    // Every aircraft to the north: east is not observable.
    feed(&mut s, [200.0, 0.0, 0.0], 50.0, 1800.0, &[0.0], &mut rng);
    let e = s.estimate(1).unwrap();
    assert!(e.sd_weak_h > 50.0, "{e:?}");
}

#[test]
fn a_window_that_does_not_fit_is_dropped() {
    let mut rng = StdRng::seed_from_u64(4);
    let mut s = Survey::default();
    feed(&mut s, [0.0; 3], 50.0, 600.0, &[0.0, 120.0, 240.0], &mut rng);
    // A partner whose timestamps jump around by km: every window dropped.
    for i in 0..200 {
        let t = 700.0 + i as f64 * 0.5;
        s.observe(1, 9, t, t, rng.gen_range(-5000.0..5000.0), dir(45.0, 3.0));
    }
    s.sweep(2000.0);
    let e = s.estimate(1).unwrap();
    assert!(e.dropped >= 1, "{e:?}");
    assert!(e.d[0].hypot(e.d[1]) < 20.0, "{e:?}");
}

#[test]
fn nothing_before_a_window_closes() {
    let mut s = Survey::default();
    s.observe(1, 2, 0.0, 0.0, 0.0, dir(0.0, 5.0));
    assert!(s.estimate(1).is_none());
}
