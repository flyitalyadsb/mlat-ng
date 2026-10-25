use super::*;

const NOW: f64 = 1_792_900_000.0;

/// A point `km` from (45, 9) towards `bearing` degrees (flat-earth, fine here).
fn at(km: f64, bearing: f64) -> (f64, f64) {
    let b = bearing.to_radians();
    (45.0 + km * b.cos() / 111.2, 9.0 + km * b.sin() / (111.2 * 45f64.to_radians().cos()))
}

/// Receiver 1 at (45, 9) hears everything within 150 km, nothing beyond;
/// receiver 2, 300 km east, hears only within 100 km of itself.
fn trained() -> Coverage {
    let mut c = Coverage::default();
    c.place(1, 45.0, 9.0, NOW);
    let (la2, lo2) = at(300.0, 90.0);
    c.place(2, la2, lo2, NOW);
    let mut t = NOW;
    let mut icao = 0u32;
    for km in (10..480).step_by(10) {
        for brg in (0..360).step_by(15) {
            let (la, lo) = at(km as f64, brg as f64);
            icao += 1;
            t += 0.01;
            let d2 = haversine_m(la2, lo2, la, lo);
            c.observe(icao, la, lo, 10_000.0, t, |id| (id == 1 && km < 150) || (id == 2 && d2 < 100_000.0));
        }
    }
    c
}

#[test]
fn hearing_probability_follows_the_learned_coverage() {
    let c = trained();
    let (la, lo) = at(80.0, 200.0);
    assert!(c.p_hear(1, la, lo, 10_000.0).unwrap() > 0.8);
    let (la, lo) = at(300.0, 200.0);
    assert!(c.p_hear(1, la, lo, 10_000.0).unwrap() < 0.05, "beyond where it ever heard");
    let (la, lo) = at(600.0, 0.0);
    assert!(c.p_hear(1, la, lo, 10_000.0).unwrap() < 1e-3, "beyond range");
    assert!(c.p_hear(99, la, lo, 10_000.0).is_none(), "no map");
}

#[test]
fn silent_receivers_rule_out_their_coverage() {
    let c = trained();
    // Heard by 1 only: inside 1's coverage, away from 2's.
    let west = at(100.0, 270.0);
    let between = at(140.0, 90.0); // 160 km from 2: outside its 100 km
    let near2 = at(250.0, 90.0); // 50 km from 2: it would have heard
    let ll = |p: (f64, f64)| c.log_lik(&[1], &[2], p.0, p.1, 10_000.0, 3.0);
    assert!(ll(west) > ll(near2) + 5.0);
    assert!(ll(between) > ll(near2) + 5.0);
}

#[test]
fn one_sample_per_aircraft_every_few_seconds() {
    let mut c = Coverage::default();
    c.place(1, 45.0, 9.0, NOW);
    let (la, lo) = at(50.0, 0.0);
    assert!(c.observe(7, la, lo, 3000.0, NOW, |_| true));
    assert!(!c.observe(7, la, lo, 3000.0, NOW + 1.0, |_| true));
    assert!(c.observe(7, la, lo, 3000.0, NOW + SAMPLE_EVERY, |_| true));
    assert!(c.observe(8, la, lo, 3000.0, NOW + 1.0, |_| true), "another aircraft");
    assert_eq!(c.samples, 3);
}

#[test]
fn near_finds_only_placed_receivers_in_range() {
    let mut c = trained();
    let (la, lo) = at(100.0, 90.0);
    let mut n = c.near(la, lo);
    n.sort();
    assert_eq!(n, vec![1, 2]);
    let (la, lo) = at(450.0, 270.0);
    assert_eq!(c.near(la, lo), vec![1], "receiver 2 is 750 km away");
    c.unplace(1);
    assert!(c.near(la, lo).is_empty());
    assert_eq!(c.receivers(), 2, "an unplaced receiver keeps its map");
}

#[test]
fn a_moved_antenna_starts_a_new_map() {
    let mut c = trained();
    let (la, lo) = at(80.0, 200.0);
    assert!(c.p_hear(1, la, lo, 10_000.0).unwrap() > 0.8);
    c.place(1, 46.0, 9.0, NOW + 10.0); // 111 km north
    assert_eq!(c.p_hear(1, la, lo, 10_000.0), Some(0.2), "no evidence yet");
    c.place(2, c.maps[&2].lat, c.maps[&2].lon + 0.001, NOW); // ~80 m: same antenna
    let (la, lo) = at(300.0, 90.0);
    assert!(c.p_hear(2, la, lo, 10_000.0).unwrap() > 0.8);
}

#[test]
fn counts_decay_with_a_half_life_of_a_day() {
    let mut c = trained();
    let before: f32 = c.maps[&1].present.iter().sum();
    let (la, lo) = at(50.0, 0.0);
    c.observe(1_000_000, la, lo, 10_000.0, NOW + HALF_LIFE, |_| true);
    let after: f32 = c.maps[&1].present.iter().sum();
    assert!((after - (before / 2.0 + 1.0)).abs() < 1.0, "{before} -> {after}");
}

#[test]
fn save_and_load_round_trip() {
    let c = trained();
    let dir = std::env::temp_dir().join(format!("cov-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("coverage.json.gz");
    c.save(path.to_str().unwrap()).unwrap();
    let d = Coverage::load(path.to_str().unwrap()).unwrap();
    assert_eq!(d.receivers(), 2);
    let (la, lo) = at(80.0, 200.0);
    assert_eq!(c.p_hear(1, la, lo, 10_000.0), d.p_hear(1, la, lo, 10_000.0));
    assert!(d.near(la, lo).is_empty(), "loaded maps wait for their receivers to be placed");
    std::fs::remove_dir_all(dir).ok();
}
