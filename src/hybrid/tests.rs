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
        client_version: "x".into(),
        compress: "none",
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
