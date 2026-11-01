use super::*;
use crate::types::ClockType;

const KEY: &str = "0123456789abcdef";

fn info(uuid: Option<&str>) -> ReceiverInfo {
    ReceiverInfo {
        user: "r".into(),
        uuid: uuid.map(|s| s.to_string()),
        lat: 45.0,
        lon: 9.0,
        alt: 100.0,
        clock: ClockType::Dump1090,
        privacy: false,
        source_ip: "x".into(),
        source_port: String::new(),
        client_version: "x".into(),
        compress: "none",
        connection: String::new(),
        msgs: Default::default(),
        serve: None,
    }
}

fn conn_ev(conn: ConnId, uuid: Option<&str>) -> Event {
    let (tx, _) = mpsc::channel(1);
    Event::Connect { conn, info: info(uuid), tx }
}

fn mlat(conn: ConnId, t: f64, b: u8) -> Event {
    Event::Mlat { conn, t, msg: vec![0x5d, b, 2, 3, 4, 5, 6], rx: 0.0 }
}

fn router() -> Router {
    Router::new(Arc::new(RouterStats::default()))
}

/// (kind, conn) of each output event, for compact assertions.
fn shape(evs: &[Event]) -> Vec<(&'static str, ConnId)> {
    evs.iter()
        .map(|e| match e {
            Event::Connect { conn, .. } => ("connect", *conn),
            Event::Disconnect { conn } => ("disconnect", *conn),
            Event::Mlat { conn, .. } => ("mlat", *conn),
            Event::Sync { conn, .. } => ("sync", *conn),
            Event::Seen { conn, .. } => ("seen", *conn),
            Event::Lost { conn, .. } => ("lost", *conn),
            Event::ClockReset { conn, .. } => ("reset", *conn),
            Event::PositionUpdate { conn, .. } => ("position", *conn),
            Event::RateReport { conn, .. } => ("rates", *conn),
            Event::GeoBaro { .. } => ("geo_baro", 0),
        })
        .collect()
}

#[test]
fn feed_data_goes_to_the_client_owner_once() {
    let mut r = router();
    let mut out = Vec::new();
    // The feed key is the client UUID's first 16 hex digits.
    r.on_event(Src::Client, conn_ev(1, Some("01234567-89ab-cdef-0000-000000000000")), &mut out);
    r.on_event(Src::Feed, conn_ev(100, Some(KEY)), &mut out);
    assert_eq!(shape(&out), vec![("connect", 1)], "only the client's Connect reaches the engine");
    out.clear();
    r.on_event(Src::Client, mlat(1, 1000.0, 7), &mut out);
    r.on_event(Src::Feed, mlat(100, 1000.0, 7), &mut out); // same copy via the feed
    r.on_event(Src::Feed, mlat(100, 2000.0, 8), &mut out); // a copy only the feed has
    assert_eq!(shape(&out), vec![("mlat", 1), ("mlat", 1)]);
    assert_eq!(r.stats.dup_mlat.load(Ordering::Relaxed), 1);
    assert_eq!(r.stats.remapped.load(Ordering::Relaxed), 1);
    assert_eq!(r.stats.merged.load(Ordering::Relaxed), 1);
}

#[test]
fn sync_pairs_are_deduplicated_too() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, conn_ev(1, Some(KEY)), &mut out);
    r.on_event(Src::Feed, conn_ev(100, Some(KEY)), &mut out);
    out.clear();
    let sync = |conn| Event::Sync { conn, et: 10.0, ot: 20.0, em: vec![1; 14], om: vec![2; 14] };
    r.on_event(Src::Feed, sync(100), &mut out);
    r.on_event(Src::Client, sync(1), &mut out);
    assert_eq!(shape(&out), vec![("sync", 1)]);
    assert_eq!(r.stats.dup_sync.load(Ordering::Relaxed), 1);
}

#[test]
fn a_client_takes_over_from_the_feed_and_hands_back() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Feed, conn_ev(100, Some(KEY)), &mut out);
    r.on_event(Src::Feed, mlat(100, 1.0, 1), &mut out);
    r.on_event(Src::Client, conn_ev(1, Some(KEY)), &mut out);
    assert_eq!(shape(&out), vec![("connect", 100), ("mlat", 100), ("disconnect", 100), ("connect", 1)]);
    out.clear();
    r.on_event(Src::Feed, mlat(100, 5.0, 2), &mut out);
    assert_eq!(shape(&out), vec![("mlat", 1)], "feed data now flows under the client's id");
    out.clear();
    r.on_event(Src::Client, Event::Disconnect { conn: 1 }, &mut out);
    assert_eq!(shape(&out), vec![("disconnect", 1), ("connect", 100)], "the feed takes over again");
    out.clear();
    r.on_event(Src::Feed, mlat(100, 6.0, 3), &mut out);
    assert_eq!(shape(&out), vec![("mlat", 100)]);
    assert_eq!(r.stats.handovers.load(Ordering::Relaxed), 2);
}

#[test]
fn the_secondary_input_does_not_report_seen_or_resets() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, conn_ev(1, Some(KEY)), &mut out);
    r.on_event(Src::Feed, conn_ev(100, Some(KEY)), &mut out);
    out.clear();
    r.on_event(Src::Feed, Event::Seen { conn: 100, icao: vec![1] }, &mut out);
    r.on_event(Src::Feed, Event::Lost { conn: 100, icao: vec![1] }, &mut out);
    r.on_event(Src::Feed, Event::ClockReset { conn: 100, reason: "x" }, &mut out);
    r.on_event(Src::Client, Event::Seen { conn: 1, icao: vec![1] }, &mut out);
    assert_eq!(shape(&out), vec![("seen", 1)]);
    assert_eq!(r.stats.secondary_dropped.load(Ordering::Relaxed), 3);
}

#[test]
fn a_reset_forgets_old_copies() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, conn_ev(1, Some(KEY)), &mut out);
    r.on_event(Src::Client, mlat(1, 1000.0, 7), &mut out);
    r.on_event(Src::Client, Event::ClockReset { conn: 1, reason: "x" }, &mut out);
    // After a restart the counter may hand out the same value again.
    r.on_event(Src::Client, mlat(1, 1000.0, 7), &mut out);
    assert_eq!(shape(&out), vec![("connect", 1), ("mlat", 1), ("reset", 1), ("mlat", 1)]);
}

#[test]
fn no_uuid_and_cloned_uuids_pass_through() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, conn_ev(1, None), &mut out);
    r.on_event(Src::Client, conn_ev(2, Some(KEY)), &mut out);
    r.on_event(Src::Client, conn_ev(3, Some(KEY)), &mut out); // same UUID, another client
    r.on_event(Src::Client, mlat(1, 1.0, 1), &mut out);
    r.on_event(Src::Client, mlat(3, 1.0, 1), &mut out);
    r.on_event(Src::Client, Event::Disconnect { conn: 3 }, &mut out);
    r.on_event(Src::Client, Event::Disconnect { conn: 1 }, &mut out);
    assert_eq!(
        shape(&out),
        vec![("connect", 1), ("connect", 2), ("connect", 3), ("mlat", 1), ("mlat", 3), ("disconnect", 3), ("disconnect", 1)]
    );
    assert_eq!(r.stats.owner_client.load(Ordering::Relaxed), 1);
}

#[test]
fn the_secondary_leaving_changes_nothing_for_the_engine() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, conn_ev(1, Some(KEY)), &mut out);
    r.on_event(Src::Feed, conn_ev(100, Some(KEY)), &mut out);
    r.on_event(Src::Feed, Event::Disconnect { conn: 100 }, &mut out);
    assert_eq!(shape(&out), vec![("connect", 1)]);
    assert_eq!(r.stats.merged.load(Ordering::Relaxed), 0);
    out.clear();
    r.on_event(Src::Client, Event::Disconnect { conn: 1 }, &mut out);
    assert_eq!(shape(&out), vec![("disconnect", 1)]);
    assert!(r.slots.is_empty());
}

#[test]
fn dedup_memory_is_bounded() {
    let mut s = Slot::default();
    for i in 0..(DEDUP_MAX as u64 + 500) {
        assert!(s.first(i, 1));
    }
    assert!(s.seen.len() <= DEDUP_MAX);
    // Old entries age out by time as well.
    assert!(s.first(DEDUP_SPAN_TICKS * 3, 1));
    assert!(s.seen.len() < 10);
}

#[test]
fn pairs_sharing_the_even_half_are_different_sync_points() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Feed, conn_ev(100, Some(KEY)), &mut out);
    out.clear();
    r.on_event(Src::Feed, Event::Sync { conn: 100, et: 10.0, ot: 20.0, em: vec![1; 14], om: vec![2; 14] }, &mut out);
    r.on_event(Src::Feed, Event::Sync { conn: 100, et: 10.0, ot: 30.0, em: vec![1; 14], om: vec![3; 14] }, &mut out);
    assert_eq!(shape(&out), vec![("sync", 100), ("sync", 100)]);
    assert_eq!(r.stats.dup_sync.load(Ordering::Relaxed), 0);
}

// ---------------------------------------------------------------- M2 --
// A served feeder and the hub's shadow copy of it (M2).

const UUID: &str = "01234567-89ab-cdef-0000-000000000000";

/// The Connect of a served connection (`--serve-listen`): it reaches the
/// router as a Client, like the shadow copy.
fn served_ev(conn: ConnId, uuid: Option<&str>) -> Event {
    let (tx, _) = mpsc::channel(1);
    let serve = Some(crate::types::ServeOpts { results: crate::types::ResultFormat::Ecef, stats: false });
    Event::Connect { conn, info: ReceiverInfo { serve, ..info(uuid) }, tx }
}

fn gauge(r: &Router, f: impl Fn(&RouterStats) -> &AtomicU64) -> u64 {
    f(&r.stats).load(Ordering::Relaxed)
}

#[test]
fn a_served_feeder_and_its_shadow_copy_are_one_receiver() {
    // Served first (the proxy opens the shadow legs after choosing).
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, served_ev(1, Some(UUID)), &mut out);
    r.on_event(Src::Client, conn_ev(2, Some(UUID)), &mut out);
    assert_eq!(shape(&out), vec![("connect", 1)], "the shadow copy is no second receiver");
    out.clear();
    r.on_event(Src::Client, mlat(2, 1000.0, 7), &mut out); // the copy
    r.on_event(Src::Client, mlat(1, 1000.0, 7), &mut out);
    r.on_event(Src::Client, Event::Seen { conn: 2, icao: vec![1] }, &mut out);
    r.on_event(Src::Client, Event::RateReport { conn: 2, rates: vec![(1, 1.0)] }, &mut out);
    r.on_event(Src::Client, Event::ClockReset { conn: 2, reason: "x" }, &mut out);
    r.on_event(Src::Client, Event::Sync { conn: 2, et: 10.0, ot: 20.0, em: vec![1; 14], om: vec![2; 14] }, &mut out);
    r.on_event(Src::Client, Event::Seen { conn: 1, icao: vec![1] }, &mut out);
    assert_eq!(shape(&out), vec![("mlat", 1), ("seen", 1)], "only the served stream reaches the engine");
    assert_eq!(gauge(&r, |s| &s.shadow_of_served_dropped), 5);
    assert_eq!(gauge(&r, |s| &s.dup_mlat), 0, "dropped whole, never deduplicated");
    assert_eq!(gauge(&r, |s| &s.passthrough_conns), 0);
    assert_eq!((gauge(&r, |s| &s.owner_serve), gauge(&r, |s| &s.owner_client), gauge(&r, |s| &s.served_with_shadow)), (1, 0, 1));

    // Shadow first (a standby taking over): the served connection takes
    // the key from it, as a client takes it from the feed.
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, conn_ev(2, Some(UUID)), &mut out);
    r.on_event(Src::Client, served_ev(1, Some(UUID)), &mut out);
    assert_eq!(shape(&out), vec![("connect", 2), ("disconnect", 2), ("connect", 1)]);
    assert_eq!(gauge(&r, |s| &s.handovers), 1);
    out.clear();
    r.on_event(Src::Client, mlat(2, 5.0, 1), &mut out);
    r.on_event(Src::Client, mlat(1, 6.0, 2), &mut out);
    assert_eq!(shape(&out), vec![("mlat", 1)]);
}

#[test]
fn the_shadow_copy_leaving_changes_nothing_while_served() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, served_ev(1, Some(UUID)), &mut out);
    r.on_event(Src::Client, conn_ev(2, Some(UUID)), &mut out);
    out.clear();
    r.on_event(Src::Client, Event::Disconnect { conn: 2 }, &mut out);
    assert!(out.is_empty(), "{:?}", shape(&out));
    assert_eq!(gauge(&r, |s| &s.served_with_shadow), 0);
    r.on_event(Src::Client, mlat(1, 1.0, 1), &mut out);
    assert_eq!(shape(&out), vec![("mlat", 1)]);
}

#[test]
fn when_the_served_connection_goes_the_shadow_copy_takes_over() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, served_ev(1, Some(UUID)), &mut out);
    r.on_event(Src::Client, conn_ev(2, Some(UUID)), &mut out);
    out.clear();
    // The proxy moved the feeder to the other instance.
    r.on_event(Src::Client, Event::Disconnect { conn: 1 }, &mut out);
    assert_eq!(shape(&out), vec![("disconnect", 1), ("connect", 2)]);
    out.clear();
    r.on_event(Src::Client, mlat(2, 1.0, 1), &mut out);
    r.on_event(Src::Client, Event::Seen { conn: 2, icao: vec![1] }, &mut out);
    assert_eq!(shape(&out), vec![("mlat", 2), ("seen", 2)], "the copy is now the receiver");
    assert_eq!((gauge(&r, |s| &s.owner_serve), gauge(&r, |s| &s.owner_client)), (0, 1));
    // It comes back (the proxy moved it back): served again.
    out.clear();
    r.on_event(Src::Client, served_ev(3, Some(UUID)), &mut out);
    assert_eq!(shape(&out), vec![("disconnect", 2), ("connect", 3)]);
}

#[test]
fn served_shadow_and_feed_rank_serve_then_shadow_then_feed() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Feed, conn_ev(100, Some(KEY)), &mut out);
    r.on_event(Src::Client, conn_ev(2, Some(UUID)), &mut out);
    r.on_event(Src::Client, served_ev(1, Some(UUID)), &mut out);
    assert_eq!(shape(&out), vec![("connect", 100), ("disconnect", 100), ("connect", 2), ("disconnect", 2), ("connect", 1)]);
    out.clear();
    // The feed still merges into the served receiver; the copy does not.
    r.on_event(Src::Feed, mlat(100, 1000.0, 7), &mut out);
    r.on_event(Src::Client, mlat(1, 1000.0, 7), &mut out);
    r.on_event(Src::Client, mlat(2, 2000.0, 8), &mut out);
    assert_eq!(shape(&out), vec![("mlat", 1)]);
    assert_eq!((gauge(&r, |s| &s.dup_mlat), gauge(&r, |s| &s.remapped), gauge(&r, |s| &s.merged)), (1, 1, 1));
    out.clear();
    r.on_event(Src::Client, Event::Disconnect { conn: 1 }, &mut out);
    r.on_event(Src::Client, Event::Disconnect { conn: 2 }, &mut out);
    r.on_event(Src::Feed, Event::Disconnect { conn: 100 }, &mut out);
    assert_eq!(shape(&out), vec![("disconnect", 1), ("connect", 2), ("disconnect", 2), ("connect", 100), ("disconnect", 100)]);
    assert!(r.slots.is_empty() && r.conn_key.is_empty());
}

#[test]
fn a_cloned_uuid_on_serve_passes_through_as_before() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, served_ev(1, Some(UUID)), &mut out);
    r.on_event(Src::Client, served_ev(3, Some(UUID)), &mut out); // another feeder, same UUID
    r.on_event(Src::Client, served_ev(4, None), &mut out); // no UUID
    r.on_event(Src::Client, mlat(3, 1.0, 1), &mut out);
    r.on_event(Src::Client, Event::Disconnect { conn: 3 }, &mut out);
    assert_eq!(shape(&out), vec![("connect", 1), ("connect", 3), ("connect", 4), ("mlat", 3), ("disconnect", 3)]);
    assert_eq!(gauge(&r, |s| &s.passthrough_conns), 1);
    assert_eq!(gauge(&r, |s| &s.shadow_of_served_dropped), 0);
}

#[test]
fn router_metrics_include_the_serve_gauges() {
    let mut r = router();
    let mut out = Vec::new();
    r.on_event(Src::Client, served_ev(1, Some(UUID)), &mut out);
    r.on_event(Src::Client, conn_ev(2, Some(UUID)), &mut out);
    r.on_event(Src::Client, mlat(2, 1.0, 1), &mut out);
    let mut o = String::new();
    render_metrics(&mut o, &r.stats);
    for want in ["mlat_ng_hybrid_owner_serve 1\n", "mlat_ng_hybrid_served_with_shadow 1\n", "mlat_ng_hybrid_shadow_of_served_dropped_total 1\n"] {
        assert!(o.contains(want), "missing {want:?} in\n{o}");
    }
}
