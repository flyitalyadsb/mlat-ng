//! Takeovers and reconnects (M3 in docs/PLAN.md, "Redundancy"): a receiver
//! whose connection ends keeps its clock pairs for the next connection of
//! the same receiver, from whichever input; after the grace it goes as
//! before.

use super::*;
use crate::clock::{MIN_UPDATES, REANCHOR_AGREE};
use crate::hybrid::{Router, RouterStats, Src};
use crate::types::{ClockType, ResultFormat, ServeOpts};

const NOW: f64 = 10_000.0;
/// No ADS-B: mlat-wanted once two receivers see it.
const MODES_ONLY: u32 = 0x111111;
const ADSB: u32 = 0x222222;
const OTHER: u32 = 0x333333;

fn engine() -> Engine {
    let (out, _) = broadcast::channel(16);
    Engine::new(EngineConfig { shadow: true, mlat_delay: 1.0, holdout_every: 0 }, Arc::new(Stats::default()), out)
}

/// Feeder `n`: its own UUID, 0.3 degrees of longitude from the next.
fn info(n: u64, serve: bool) -> ReceiverInfo {
    ReceiverInfo {
        user: format!("rx{n}"),
        uuid: Some(format!("{n:016x}-0000-4000-8000-000000000000")),
        lat: 45.0,
        lon: 9.0 + 0.3 * n as f64,
        alt: 100.0,
        clock: ClockType::Dump1090,
        privacy: false,
        source_ip: "test".into(),
        source_port: String::new(),
        client_version: "test".into(),
        compress: "none",
        connection: String::new(),
        msgs: Default::default(),
        serve: serve.then_some(ServeOpts { results: ResultFormat::Ecef, stats: false }),
    }
}

/// The same feeder's receiver on the Beast feed (relay/adapter.rs).
fn feed_info(n: u64) -> ReceiverInfo {
    ReceiverInfo {
        user: format!("rx{n}~feed"),
        uuid: Some(format!("{n:016x}")),
        source_ip: "feed".into(),
        client_version: "feed".into(),
        ..info(n, false)
    }
}

/// Connect, and the receiving end of the connection.
fn connect(e: &mut Engine, conn: ConnId, info: ReceiverInfo, now: f64) -> mpsc::Receiver<ToClient> {
    let (tx, rx) = mpsc::channel(64);
    e.handle_at(Event::Connect { conn, info, tx }, now);
    rx
}

fn started(rx: &mut mpsc::Receiver<ToClient>) -> Vec<u32> {
    let mut v = Vec::new();
    while let Ok(m) = rx.try_recv() {
        if let ToClient::StartSending(a) = m {
            v.extend(a);
        }
    }
    v.sort_unstable();
    v
}

/// `n` sync points heard by a and b, every 0.5 s from true time `tau` and
/// wall time `now`, through the engine's pair update. Receiver k's clock reads
/// tau + 1000 k (+ `jump_a` for a). Returns (accepted, rejected).
fn sync(e: &mut Engine, a: RxKey, b: RxKey, tau: f64, n: usize, now: f64, jump_a: f64) -> (u32, u32) {
    let p = llh_to_ecef(Llh { lat: 45.2, lon: 9.4, alt: 9000.0 });
    let obs = |e: &Engine, k: RxKey, t: f64| {
        let s = t + 1000.0 * k as f64 + if k == a { jump_a } else { 0.0 };
        SyncObs { rx: k, tda: s, tdb: s + 0.5, sb: s + 0.5, epoch: e.rxs[&k].epoch }
    };
    let mut tot = (0, 0);
    for i in 0..n {
        let t = tau + i as f64 * 0.5;
        let (oa, ob) = (obs(e, a, t), obs(e, b, t));
        let (ok, bad) = e.pair_update(b, &ob, oa, now + i as f64 * 0.5, (p, p));
        tot = (tot.0 + ok, tot.1 + bad);
    }
    tot
}

fn usable(e: &Engine, a: RxKey, b: RxKey, now: f64) -> bool {
    e.pairs.get(&pair_key(a, b)).is_some_and(|m| m.usable(now))
}

/// Feeder 1 (served or not) and two shadow peers, converged with each other
/// 10 s before NOW; feeder 1 and 2 see a Mode-S-only and an ADS-B aircraft,
/// feeder 1 sent a rate report.
fn network(e: &mut Engine, serve: bool) -> (RxKey, RxKey, RxKey) {
    connect(e, 1, info(1, serve), NOW - 30.0);
    connect(e, 2, info(2, false), NOW - 30.0);
    connect(e, 3, info(3, false), NOW - 30.0);
    let (a, b, c) = (e.rx_by_conn[&1], e.rx_by_conn[&2], e.rx_by_conn[&3]);
    for (x, y) in [(a, b), (a, c), (b, c)] {
        sync(e, x, y, 100.0, 15, NOW - 10.0, 0.0);
    }
    for conn in [1, 2] {
        e.handle_at(Event::Seen { conn, icao: vec![MODES_ONLY, ADSB, OTHER] }, NOW - 5.0);
    }
    e.aircraft.get_mut(&ADSB).unwrap().last_adsb = NOW;
    e.handle_at(Event::RateReport { conn: 1, rates: vec![(ADSB, 1.5)] }, NOW - 4.0);
    assert!(usable(e, a, b, NOW) && usable(e, a, c, NOW));
    (a, b, c)
}

#[test]
fn a_handover_between_inputs_keeps_usable_pairs() {
    // The served session gives way to the same feeder's shadow leg and back
    // (a Disconnect and a Connect in one go, as the hybrid router hands a
    // key over): the same receiver throughout, its pairs usable throughout.
    let mut e = engine();
    let (a, b, c) = network(&mut e, true);
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    let mut shadow = connect(&mut e, 11, info(1, false), NOW);
    assert_eq!(e.rx_by_conn[&11], a);
    assert_eq!(e.served_n, 0);
    assert!(usable(&e, a, b, NOW) && usable(&e, a, c, NOW));
    assert!(started(&mut shadow).is_empty(), "a shadow connection is never asked");
    e.handle_at(Event::Disconnect { conn: 11 }, NOW + 1.0);
    let mut served = connect(&mut e, 21, info(1, true), NOW + 1.0);
    assert_eq!(e.rx_by_conn[&21], a);
    assert_eq!(e.served_n, 1);
    assert!(usable(&e, a, b, NOW + 1.0) && usable(&e, a, c, NOW + 1.0));
    // Asked at once, from what the previous connection reported, not at
    // the new session's first report 5-22 s from now.
    let asked = started(&mut served);
    assert!(asked.contains(&MODES_ONLY) && asked.contains(&ADSB), "{asked:x?}");
    assert_eq!(e.stats.rx_resumed.load(Ordering::Relaxed), 2);
    assert_eq!(e.stats.rx_resumed_pairs.load(Ordering::Relaxed), 4);

    // Before: the same handover made a new receiver with no pairs and
    // nothing to ask for.
    let mut e = engine().with_reconnect_grace(0.0);
    let (a, _, _) = network(&mut e, true);
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    let mut served = connect(&mut e, 21, info(1, true), NOW);
    let k = e.rx_by_conn[&21];
    assert_ne!(k, a);
    assert!(e.rxs[&k].peers.is_empty());
    assert!(started(&mut served).is_empty());
}

#[test]
fn a_takeover_through_the_router_keeps_the_pairs() {
    // The standby's view of a takeover, through the hybrid router (M2):
    // feeder 1 is known by its hub copy (a shadow client) and its Beast
    // feed. Its old session ends, its new one arrives served, with a hub copy
    // of its own; later the served leg goes and the hub copy remains.
    let feed = crate::relay::adapter::CONN_BASE + 1;
    let mut e = engine();
    let mut r = Router::new(Arc::new(RouterStats::default()));
    let mut route = |e: &mut Engine, src: Src, ev: Event, now: f64| {
        let mut out = Vec::new();
        r.on_event(src, ev, &mut out);
        for ev in out {
            e.handle_at(ev, now);
        }
    };
    let (tx, _) = mpsc::channel(8);
    let conn = |conn: ConnId, info: ReceiverInfo| Event::Connect { conn, info, tx: tx.clone() };
    route(&mut e, Src::Client, conn(1, info(1, false)), NOW - 30.0);
    route(&mut e, Src::Feed, conn(feed, feed_info(1)), NOW - 30.0);
    route(&mut e, Src::Client, conn(2, info(2, false)), NOW - 30.0);
    let (a, b) = (e.rx_by_conn[&1], e.rx_by_conn[&2]);
    sync(&mut e, a, b, 100.0, 15, NOW - 10.0, 0.0);
    // The old session's hub leg closes: the feed receiver takes the key.
    route(&mut e, Src::Client, Event::Disconnect { conn: 1 }, NOW);
    assert_eq!(e.rx_by_conn[&feed], a);
    assert!(usable(&e, a, b, NOW));
    // The new session: served here, then its hub copy (whose data the
    // router drops while the served leg owns the key).
    route(&mut e, Src::Client, conn(5, info(1, true)), NOW + 3.0);
    route(&mut e, Src::Client, conn(6, info(1, false)), NOW + 3.0);
    assert_eq!(e.rx_by_conn[&5], a);
    assert!(!e.rx_by_conn.contains_key(&6));
    assert_eq!(e.served_n, 1);
    assert!(usable(&e, a, b, NOW + 3.0));
    // Both legs announce the session's input; the served one counts. The
    // pair is set aside until the feed's sync (the same counter, remapped
    // to the owner) confirms it.
    for c in [5, 6] {
        route(&mut e, Src::Client, Event::ClockReset { conn: c, reason: "clientmsg: input connected" }, NOW + 3.1);
    }
    assert!(!usable(&e, a, b, NOW + 3.1));
    sync(&mut e, a, b, 120.0, 2, NOW + 3.5, 0.0);
    assert!(usable(&e, a, b, NOW + 4.5));
    // The served leg goes (the proxy moved the feeder on): the hub copy
    // takes over, the receiver and its pairs stay.
    route(&mut e, Src::Client, Event::Disconnect { conn: 5 }, NOW + 60.0);
    assert_eq!(e.rx_by_conn[&6], a);
    assert_eq!(e.served_n, 0);
    assert!(e.pairs.contains_key(&pair_key(a, b)));
    assert_eq!(e.rxs.len(), 2);
    assert_eq!(e.stats.rx_resumed.load(Ordering::Relaxed), 3);
}

#[test]
fn a_quick_reconnect_keeps_the_pairs_and_reanchors_them() {
    let mut e = engine();
    let (a, b, _) = network(&mut e, true);
    let before = e.pairs[&pair_key(a, b)].updates;
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    for i in 1..=3 {
        e.tick_at(NOW + i as f64, false);
    }
    // Waiting: its pairs kept, in nothing else.
    assert_eq!(e.parked.len(), 1);
    assert!(e.pairs.contains_key(&pair_key(a, b)));
    assert!(!e.aircraft[&MODES_ONLY].seen_by.contains(&a));
    e.publish_at(0, NOW + 3.0);
    let snap = e.stats.snapshot.lock().unwrap().clone();
    assert_eq!((snap.receivers, snap.receivers_parked, snap.serve_receivers), (2, 1, 0));
    let g = Group { first_wall: NOW, copies: vec![(a, 1.0), (b, 2.0)], truth: None };
    assert_eq!(e.build(&g).0, vec![b], "a parked receiver's copies are not used");
    // mlat-client's new session, 3 s later, and its two announcements.
    let _rx = connect(&mut e, 9, info(1, true), NOW + 3.0);
    assert_eq!(e.rx_by_conn[&9], a);
    assert!(e.parked.is_empty());
    e.handle_at(Event::ClockReset { conn: 9, reason: "clientmsg: input connected" }, NOW + 3.1);
    e.handle_at(Event::ClockReset { conn: 9, reason: "clientmsg: clock_reset" }, NOW + 3.2);
    assert!(!usable(&e, a, b, NOW + 3.2));
    // Two sync points with its counter run on: the pair is back whole.
    assert_eq!(sync(&mut e, a, b, 120.0, 2, NOW + 3.5, 0.0), (4, 0));
    assert!(usable(&e, a, b, NOW + 4.5));
    assert_eq!(e.pairs[&pair_key(a, b)].updates, before + REANCHOR_AGREE);
    assert_eq!(e.stats.pair_reanchors.load(Ordering::Relaxed), 1);

    // Its readsb restarted with it: the counter moved, the pair is not
    // re-anchored and needs a fresh offset, MIN_UPDATES observations.
    let mut e = engine();
    let (a, b, _) = network(&mut e, true);
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    let _rx = connect(&mut e, 9, info(1, true), NOW + 3.0);
    e.handle_at(Event::ClockReset { conn: 9, reason: "clientmsg: input connected" }, NOW + 3.1);
    let n = (MIN_UPDATES as usize).div_ceil(2);
    sync(&mut e, a, b, 120.0, n - 1, NOW + 3.5, -3.7);
    assert!(!usable(&e, a, b, NOW + 3.5 + n as f64 * 0.5));
    sync(&mut e, a, b, 130.0, 1, NOW + 3.5 + n as f64 * 0.5, -3.7);
    assert!(usable(&e, a, b, NOW + 4.0 + n as f64 * 0.5));
    assert_eq!(e.stats.pair_reanchors.load(Ordering::Relaxed), 0);
    let m = &e.pairs[&pair_key(a, b)];
    assert!((m.predict(140.0 + 1000.0 * a as f64 - 3.7).0 - (1000.0 * (b as f64 - a as f64) + 3.7)).abs() < 1e-6);
}

#[test]
fn after_the_grace_the_receiver_and_its_pairs_go_as_before() {
    let mut e = engine();
    let (a, b, c) = network(&mut e, false);
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    e.tick_at(NOW + RECONNECT_GRACE_S - 0.5, false);
    assert!(e.rxs.contains_key(&a));
    assert_eq!(e.pairs.len(), 3);
    e.tick_at(NOW + RECONNECT_GRACE_S + 0.5, false);
    assert!(!e.rxs.contains_key(&a));
    assert!(e.parked.is_empty());
    assert_eq!(e.pairs.keys().copied().collect::<Vec<_>>(), vec![pair_key(b, c)]);
    assert!(!e.rxs[&b].peers.contains(&a));
    assert_eq!(e.rxs[&b].band_count.iter().sum::<u16>(), 1);
    assert_eq!(e.stats.rx_park_expired.load(Ordering::Relaxed), 1);
    // Back after that: a new receiver, from zero.
    connect(&mut e, 9, info(1, false), NOW + 25.0);
    let k = e.rx_by_conn[&9];
    assert_ne!(k, a);
    assert!(e.rxs[&k].peers.is_empty() && e.rxs[&k].seen.is_empty());

    // With no grace, at once (the old behaviour).
    let mut e = engine().with_reconnect_grace(0.0);
    let (a, b, c) = network(&mut e, false);
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    assert!(!e.rxs.contains_key(&a) && e.parked.is_empty());
    assert_eq!(e.pairs.keys().copied().collect::<Vec<_>>(), vec![pair_key(b, c)]);
    assert_eq!(e.aircraft[&MODES_ONLY].tracking, 1);
}

#[test]
fn another_place_or_clock_is_another_receiver() {
    let mut e = engine();
    let (a, _, _) = network(&mut e, false);
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    // Its identity ~800 m away, then on another clock type: new receivers.
    connect(&mut e, 7, ReceiverInfo { lon: 9.31, ..info(1, false) }, NOW + 1.0);
    connect(&mut e, 8, ReceiverInfo { clock: ClockType::Beast, ..info(1, false) }, NOW + 1.0);
    assert!(e.rx_by_conn[&7] != a && e.rx_by_conn[&8] != a);
    assert!(e.rxs[&e.rx_by_conn[&7]].peers.is_empty());
    assert_eq!(e.stats.rx_resume_refused.load(Ordering::Relaxed), 2);
    // The parked one waits on for its own.
    connect(&mut e, 9, info(1, false), NOW + 2.0);
    assert_eq!(e.rx_by_conn[&9], a);
}

#[test]
fn what_a_resumed_receiver_brought_is_confirmed_or_dropped() {
    let mut e = engine();
    let (a, _, _) = network(&mut e, true);
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    assert_eq!(e.aircraft[&OTHER].tracking, 1);
    let _rx = connect(&mut e, 9, info(1, true), NOW + 3.0);
    assert_eq!(e.aircraft[&OTHER].tracking, 2);
    // mlat-client's empty first report: the rates it brought stay.
    e.handle_at(Event::RateReport { conn: 9, rates: vec![] }, NOW + 3.1);
    assert_eq!(e.rxs[&a].rate.as_ref().map(|r| r.len()), Some(1));
    // Its first real aircraft report has two of the three.
    e.handle_at(Event::Seen { conn: 9, icao: vec![MODES_ONLY, ADSB] }, NOW + 15.0);
    e.tick_at(NOW + 3.0 + INHERIT_CONFIRM_S - 1.0, false);
    assert!(e.rxs[&a].seen.contains(&OTHER));
    e.tick_at(NOW + 3.0 + INHERIT_CONFIRM_S + 0.5, false);
    let seen: HashSet<u32> = [MODES_ONLY, ADSB].into_iter().collect();
    assert_eq!(e.rxs[&a].seen, seen);
    assert_eq!(e.aircraft[&OTHER].tracking, 1);
    assert!(!e.aircraft[&OTHER].seen_by.contains(&a));
    // No real rate report came: what the session said itself (the marker).
    assert_eq!(e.rxs[&a].rate.as_ref().map(|r| r.len()), Some(0));
    assert!(e.rxs[&a].inherited.is_none());

    // A real rate report replaces the inherited one at once.
    let mut e = engine();
    let (a, _, _) = network(&mut e, true);
    e.handle_at(Event::Disconnect { conn: 1 }, NOW);
    let _rx = connect(&mut e, 9, info(1, true), NOW + 3.0);
    e.handle_at(Event::RateReport { conn: 9, rates: vec![(OTHER, 2.0)] }, NOW + 10.0);
    let rate = e.rxs[&a].rate.clone().unwrap();
    assert_eq!((rate.len(), rate.get(&OTHER).copied()), (1, Some(2.0)));
    e.tick_at(NOW + 3.0 + INHERIT_CONFIRM_S + 0.5, false);
    assert_eq!(e.rxs[&a].rate.as_ref().map(|r| r.len()), Some(1));
}

#[test]
fn an_observation_from_before_a_clock_reset_meets_no_later_one() {
    let em = crate::sim::test_df17_position(0x4ca123, 45.2, 9.4, 30_000, false);
    let om = crate::sim::test_df17_position(0x4ca123, 45.2, 9.4, 30_000, true);
    for reset in [false, true] {
        let mut e = engine();
        connect(&mut e, 1, info(1, false), NOW - 1.0);
        connect(&mut e, 2, info(2, false), NOW - 1.0);
        e.handle_at(Event::Sync { conn: 1, et: 1.2e10, ot: 1.2e10 + 6e6, em: em.clone(), om: om.clone() }, NOW);
        if reset {
            e.handle_at(Event::ClockReset { conn: 1, reason: "clientmsg: clock_reset" }, NOW + 0.1);
        }
        e.handle_at(Event::Sync { conn: 2, et: 3.6e10, ot: 3.6e10 + 6e6, em: em.clone(), om: om.clone() }, NOW + 0.2);
        assert_eq!(e.pairs.len(), usize::from(!reset), "reset {reset}");
    }
}
