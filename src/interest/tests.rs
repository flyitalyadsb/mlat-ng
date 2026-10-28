use super::*;

fn ac(icao: u32, tracking: usize, last_adsb: f64) -> AcState {
    AcState { icao, tracking, last_adsb, sync_bad: 0.0 }
}

fn set(v: &[u32]) -> HashSet<u32> {
    v.iter().copied().collect()
}

fn meta_all(_: u32) -> Option<AcMeta> {
    Some(AcMeta { alt_ft: Some(35_000), sync_dont_use: false })
}

#[test]
fn mlat_wanted_needs_two_receivers_and_no_recent_adsb() {
    let mut i = Interest::new(Params { force_mlat_interval: 1e9, ..Params::default() }, 1);
    let now = 10_000.0;
    i.refresh_wanted(now, [ac(1, 2, 0.0), ac(2, 1, 0.0), ac(3, 5, now - 5.0), ac(4, 3, now - 121.0)].into_iter());
    assert_eq!(*i.wanted(), set(&[1, 4]));
    // Rejected sync points make an ADS-B aircraft worth multilaterating.
    let newly = i.refresh_wanted(now + 1.0, [AcState { icao: 3, tracking: 5, last_adsb: now, sync_bad: 0.2 }].into_iter());
    assert_eq!(*i.wanted(), set(&[3]));
    assert_eq!(newly, vec![3]);
    // Still wanted next time: not new any more.
    assert!(i.refresh_wanted(now + 2.0, [AcState { icao: 3, tracking: 5, last_adsb: now, sync_bad: 0.2 }].into_iter()).is_empty());
}

#[test]
fn refresh_is_rate_limited() {
    let mut i = Interest::new(Params::default(), 1);
    i.refresh_wanted(1000.0, [ac(1, 2, 0.0)].into_iter());
    i.refresh_wanted(1000.05, [ac(2, 2, 0.0)].into_iter());
    assert_eq!(*i.wanted(), set(&[1]));
}

#[test]
fn forced_window_comes_round_for_adsb_aircraft() {
    let mut i = Interest::new(Params { force_mlat_interval: 60.0, ..Params::default() }, 7);
    let mut forced = 0;
    let mut t = 0.0;
    while t < 600.0 {
        // ADS-B always fresh: only the forced window can make it wanted.
        i.refresh_wanted(t, [ac(1, 4, t)].into_iter());
        if i.wanted().contains(&1) {
            forced += 1;
        }
        t += 1.0;
    }
    // About 15 s in every ~75 s cycle.
    assert!((80..=200).contains(&forced), "{forced}");
}

#[test]
fn legacy_client_gets_up_to_max_sync_random_aircraft() {
    let mut i = Interest::new(Params::default(), 3);
    let tracking: HashSet<u32> = (1..=40).collect();
    let (sync, mlat) = i.select(false, None, &tracking, |_| vec![], meta_all);
    assert_eq!(sync.len(), 15);
    assert!(sync.is_subset(&tracking));
    assert!(mlat.is_empty());
}

#[test]
fn empty_rate_report_gets_the_floor() {
    let mut i = Interest::new(Params::default(), 3);
    let tracking: HashSet<u32> = (1..=40).collect();
    let empty = HashMap::default();
    let (sync, _) = i.select(false, Some(&empty), &tracking, |_| vec![], meta_all);
    assert_eq!(sync.len(), 3);
}

#[test]
fn sync_prefers_aircraft_shared_with_peers_and_respects_the_rate_cap() {
    let mut i = Interest::new(Params::default(), 5);
    // 30 aircraft at 2 positions/s; only 1..=10 are seen by peers.
    let rate: HashMap<u32, f32> = (1..=30).map(|a| (a, 2.0)).collect();
    let tracking: HashSet<u32> = (1..=30).collect();
    let peers = |a: u32| if a <= 10 { vec![(100 + (a as u64 % 3), 2.0f32)] } else { vec![] };
    let (sync, _) = i.select(false, Some(&rate), &tracking, peers, meta_all);
    assert!(!sync.is_empty());
    assert!(sync.iter().all(|&a| a <= 10), "{sync:?}");
    // 12 msg/s cap: the loop stops once the total passes 12 (one extra add).
    assert!(sync.len() as f64 * 2.0 <= 12.0 + 2.0, "{sync:?}");
}

#[test]
fn dont_use_aircraft_only_in_the_second_round() {
    let mut i = Interest::new(Params::default(), 9);
    let rate: HashMap<u32, f32> = [(1, 1.0), (2, 1.0)].into_iter().collect();
    let tracking = set(&[1, 2]);
    let peers = |_: u32| vec![(100, 1.0f32), (101, 1.0)];
    let meta = |a: u32| Some(AcMeta { alt_ft: Some(30_000), sync_dont_use: a == 1 });
    let (sync, _) = i.select(false, Some(&rate), &tracking, peers, meta);
    assert!(sync.contains(&2));
    // Round 2 (threshold 3.5) still takes it: peers have little shared rate.
    assert!(sync.contains(&1));
}

#[test]
fn mlat_is_what_the_receiver_sees_of_the_wanted_set() {
    let mut i = Interest::new(Params { force_mlat_interval: 1e9, ..Params::default() }, 1);
    i.refresh_wanted(1000.0, [ac(1, 3, 0.0), ac(2, 3, 0.0), ac(3, 3, 999.0)].into_iter());
    let (_, mlat) = i.select(false, Some(&HashMap::default()), &set(&[1, 3, 4]), |_| vec![], meta_all);
    assert_eq!(mlat, set(&[1]));
}

#[test]
fn bad_clock_gets_three_sync_aircraft_and_no_mlat() {
    let mut i = Interest::new(Params { force_mlat_interval: 1e9, ..Params::default() }, 1);
    i.refresh_wanted(1000.0, [ac(1, 3, 0.0)].into_iter());
    let tracking: HashSet<u32> = (1..=40).collect();
    let (sync, mlat) = i.select(true, None, &tracking, |_| vec![], meta_all);
    assert_eq!(sync.len(), 3);
    assert!(mlat.is_empty());
}

#[test]
fn diff_is_start_and_stop() {
    let (start, stop) = diff(&set(&[1, 2, 3]), &set(&[2, 3, 4, 5]));
    assert_eq!(start, vec![4, 5]);
    assert_eq!(stop, vec![1]);
}
