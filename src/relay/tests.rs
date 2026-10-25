use std::collections::HashMap;
use std::sync::Arc;

use super::adapter::{Adapter, AdapterConfig, AdapterStats, CONN_BASE};
use super::beast::{encode, Frame, Parser};
use super::registry::{self, key_of_uuid, Receiver};
use crate::types::{ClockType, Event};

fn frame(rid: u64, ts: u64, msg: &[u8]) -> Frame {
    let mut m = [0u8; 14];
    m[..msg.len()].copy_from_slice(msg);
    let kind = match msg.len() {
        2 => b'1',
        7 => b'2',
        _ => b'3',
    };
    Frame { rid, ts, sig: 0x40, kind, len: msg.len() as u8, msg: m }
}

#[test]
fn beast_roundtrip_with_escapes_in_every_field() {
    // 0x1A in the receiver id, the timestamp, the signal and the payload.
    let frames = vec![
        frame(0x1A1A_0000_1A00_0001, 0x1A_0000_1A1A, &[0x8D, 0x1A, 0x1A, 0x1A, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1A]),
        Frame { sig: 0x1A, ..frame(0x1A1A_0000_1A00_0001, 5, &[0x5D, 0x1A, 0x02, 0x03, 0x1A, 0x1A, 0x00]) },
        frame(42, 7, &[0x00, 0x00]),
    ];
    let wire = encode(&frames);
    // all at once
    let mut p = Parser::default();
    let mut out = Vec::new();
    p.push(&wire, &mut out);
    assert_eq!(out, frames);
    assert_eq!(p.desync, 0);
    // one byte at a time
    let mut p = Parser::default();
    let mut out = Vec::new();
    for b in &wire {
        p.push(std::slice::from_ref(b), &mut out);
    }
    assert_eq!(out, frames);
    assert_eq!(p.desync, 0);
}

#[test]
fn beast_skips_garbage_and_cut_messages() {
    let good = frame(9, 100, &[0x5D, 1, 2, 3, 4, 5, 6]);
    let mut wire = vec![0x00, 0x1A, b'X', 0x99];
    // a long frame cut short by the start of the next message
    wire.extend_from_slice(&[0x1A, b'3', 0, 0, 0, 0, 0, 1, 0x40, 0x8D]);
    wire.extend_from_slice(&encode(&[good]));
    let mut p = Parser::default();
    let mut out = Vec::new();
    p.push(&wire, &mut out);
    assert_eq!(out, vec![good]);
    assert_eq!(p.desync, 2);
}

#[test]
fn uuid_key_matches_readsb() {
    assert_eq!(key_of_uuid("945ecb8c-842b-46f9-9891-f96bb312a820"), Some(0x945e_cb8c_842b_46f9));
    assert_eq!(key_of_uuid("945ECB8C842B46F9"), Some(0x945e_cb8c_842b_46f9));
    assert_eq!(key_of_uuid("945ecb8c-842b"), None);
    assert_eq!(key_of_uuid("zz5ecb8c-842b-46f9-9891-f96bb312a820"), None);
}

#[test]
fn registry_parse_skips_bad_entries() {
    let doc: registry::Doc = serde_json::from_str(
        r#"{"receivers":[
            {"key":"945ecb8c842b46f9","user":"a","lat":45.0,"lon":9.0,"alt":100.0,"clock":"dump1090"},
            {"key":"945ecb8c842b","user":"short","lat":45.0,"lon":9.0,"alt":100.0,"clock":"dump1090"},
            {"key":"0000000000000002","user":"nowhere","lat":0.0,"lon":0.0,"alt":0.0,"clock":"dump1090"},
            {"key":"0000000000000003","user":"weird","lat":45.0,"lon":9.0,"alt":0.0,"clock":"martian"}
        ]}"#,
    )
    .unwrap();
    let m = registry::parse(&doc);
    assert_eq!(m.len(), 1);
    assert_eq!(m[&0x945e_cb8c_842b_46f9].user, "a");
}

fn reg(rids: &[(u64, ClockType)]) -> HashMap<u64, Receiver> {
    rids.iter()
        .map(|&(r, clock)| {
            (r, Receiver { user: format!("u{r}"), lat: 45.0 + r as f64 * 0.01, lon: 9.0, alt: 100.0, clock, privacy: false })
        })
        .collect()
}

fn adapter(cfg: AdapterConfig, rids: &[(u64, ClockType)]) -> (Adapter, Arc<AdapterStats>) {
    let st = Arc::new(AdapterStats::default());
    let mut a = Adapter::new(cfg, st.clone());
    let mut evs = Vec::new();
    a.set_registry(reg(rids), &mut evs);
    assert!(evs.is_empty());
    (a, st)
}

/// A clean DF11 acquisition squitter from `icao`.
fn df11(icao: u32) -> Vec<u8> {
    let mut f = vec![0x5D, (icao >> 16) as u8, (icao >> 8) as u8, icao as u8, 0, 0, 0];
    let c = crate::modes::crc_residual(&f);
    f[4] = (c >> 16) as u8;
    f[5] = (c >> 8) as u8;
    f[6] = c as u8;
    f
}

fn kinds(evs: &[Event]) -> Vec<&'static str> {
    evs.iter()
        .map(|e| match e {
            Event::Connect { .. } => "connect",
            Event::Disconnect { .. } => "disconnect",
            Event::Sync { .. } => "sync",
            Event::Mlat { .. } => "mlat",
            Event::Seen { .. } => "seen",
            Event::Lost { .. } => "lost",
            Event::ClockReset { .. } => "reset",
            Event::PositionUpdate { .. } => "moved",
            Event::RateReport { .. } => "rate",
            Event::GeoBaro { .. } => "geo_baro",
        })
        .collect()
}

#[test]
fn receiver_joins_after_probation_and_only_with_a_12mhz_clock() {
    let cfg = AdapterConfig { probation: 20.0, ..Default::default() };
    let (mut a, st) = adapter(cfg, &[(1, ClockType::Dump1090), (2, ClockType::Dump1090), (3, ClockType::RadarcapeGps)]);
    let mut evs = Vec::new();
    let sq = df11(0x3C1234);
    for s in 0..=25 {
        let wall = 1000.0 + s as f64;
        // rx 1: a good 12 MHz counter; rx 2: 1 GHz ticks (Radarcape-like); rx 3: GPS per the registry
        a.on_frame(&frame(1, 5_000_000 + (s as u64) * 12_000_000, &sq), wall, &mut evs);
        a.on_frame(&frame(2, 5_000_000 + (s as u64) * 1_000_000_000, &sq), wall, &mut evs);
        a.on_frame(&frame(3, 5_000_000 + (s as u64) * 12_000_000, &sq), wall, &mut evs);
        a.on_frame(&frame(99, 5_000_000 + (s as u64) * 12_000_000, &sq), wall, &mut evs);
    }
    let connects: Vec<u64> = evs
        .iter()
        .filter_map(|e| match e {
            Event::Connect { conn, info, .. } => {
                assert_eq!(info.user, "u1~feed");
                Some(*conn)
            }
            _ => None,
        })
        .collect();
    assert_eq!(connects, vec![CONN_BASE]);
    // the jumpy 1 GHz clock trips the jump check, never the rate check: it never joins
    a.tick(1026.0, &mut evs);
    assert!(kinds(&evs).contains(&"seen"));
    assert_eq!(st.receivers_live.load(std::sync::atomic::Ordering::Relaxed), 1);
    assert!(st.receivers_bad_clock.load(std::sync::atomic::Ordering::Relaxed) >= 1);
    assert_eq!(st.receivers_unmapped.load(std::sync::atomic::Ordering::Relaxed), 1);
}

fn live(a: &mut Adapter, rids: &[u64], evs: &mut Vec<Event>) {
    // 21 s of squitters so every receiver passes probation
    for s in 0..=21 {
        for &r in rids {
            a.on_frame(&frame(r, 1_000_000 + s * 12_000_000, &df11(0x3C0001)), 1000.0 + s as f64, evs);
        }
    }
    a.tick(1021.5, evs);
}

#[test]
fn sync_pairs_are_the_same_subset_at_every_receiver() {
    let cfg = AdapterConfig { sync_every: 3, bench_every: 0, ..Default::default() };
    let (mut a, st) = adapter(cfg, &[(1, ClockType::Dump1090), (2, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    live(&mut a, &[1, 2], &mut evs);
    evs.clear();
    // 60 alternating even/odd positions of one aircraft, heard by both
    let mut pairs: HashMap<u64, Vec<(Vec<u8>, Vec<u8>)>> = HashMap::new();
    for i in 0..60u64 {
        let f = crate::sim::test_df17_position(0x4CA001, 45.5 + i as f64 * 1e-3, 9.2, 35_000, i % 2 == 1);
        for r in [1u64, 2] {
            a.on_frame(&frame(r, 1_000_000 + (22 * 12_000_000) + i * 6_000_000 + r * 100, &f), 1022.0 + i as f64 * 0.5, &mut evs);
        }
        for e in evs.drain(..) {
            if let Event::Sync { conn, em, om, et, ot } = e {
                assert!(ot != et && (ot - et).abs() <= 5.0 * 12e6);
                pairs.entry(conn).or_default().push((em, om));
            }
        }
    }
    let p1 = &pairs[&CONN_BASE];
    let p2 = &pairs[&(CONN_BASE + 1)];
    assert_eq!(p1, p2, "both receivers must keep the same pairs");
    // 59 pairs formed, about a third kept
    assert!(p1.len() >= 10 && p1.len() <= 35, "kept {}", p1.len());
    assert_eq!(st.ev_sync.load(std::sync::atomic::Ordering::Relaxed) as usize, 2 * p1.len());
}

#[test]
fn replies_route_by_adsb_presence() {
    // bench_every 1 = every ADS-B aircraft is on the bench
    let cfg = AdapterConfig { sync_every: 1, bench_every: 1, modes_only_after: 20.0, ..Default::default() };
    let (mut a, st) = adapter(cfg, &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    live(&mut a, &[1], &mut evs);
    evs.clear();
    let ld = |x: &std::sync::atomic::AtomicU64| x.load(std::sync::atomic::Ordering::Relaxed);
    let (bench0, modes0) = (ld(&st.ev_mlat_bench), ld(&st.ev_mlat_modes_only));
    let t0 = 1_000_000 + 22 * 12_000_000u64;
    // aircraft A has ADS-B positions; aircraft B never does
    let pos = crate::sim::test_df17_position(0x4CA00A, 45.5, 9.2, 30_000, false);
    a.on_frame(&frame(1, t0, &pos), 1022.0, &mut evs);
    let ra = df11(0x4CA00A);
    let rb = df11(0x3C000B);
    a.on_frame(&frame(1, t0 + 12_000_000, &ra), 1023.0, &mut evs);
    a.on_frame(&frame(1, t0 + 12_000_001, &rb), 1023.0, &mut evs);
    assert_eq!(kinds(&evs), vec!["mlat", "mlat"]);
    assert_eq!(ld(&st.ev_mlat_bench) - bench0, 1);
    assert_eq!(ld(&st.ev_mlat_modes_only) - modes0, 1);
    // with the bench off, A's replies are not forwarded at all
    let cfg = AdapterConfig { sync_every: 1, bench_every: 0, ..Default::default() };
    let (mut a, _) = adapter(cfg, &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    live(&mut a, &[1], &mut evs);
    evs.clear();
    a.on_frame(&frame(1, t0, &pos), 1022.0, &mut evs);
    a.on_frame(&frame(1, t0 + 12_000_000, &ra), 1023.0, &mut evs);
    assert!(evs.is_empty());
    // 21 s later A counts as Mode-S-only again
    a.on_frame(&frame(1, t0 + 22 * 12_000_000, &ra), 1044.0, &mut evs);
    assert_eq!(kinds(&evs), vec!["mlat"]);
}

#[test]
fn clock_jump_resets_and_silence_disconnects() {
    let (mut a, st) = adapter(AdapterConfig::default(), &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    live(&mut a, &[1], &mut evs);
    evs.clear();
    // the feeder restarted: its counter is back near zero
    a.on_frame(&frame(1, 1000, &df11(0x3C0001)), 1022.0, &mut evs);
    assert_eq!(kinds(&evs)[0], "reset");
    assert_eq!(st.clock_jumps.load(std::sync::atomic::Ordering::Relaxed), 1);
    evs.clear();
    a.tick(1022.0 + 200.0, &mut evs);
    assert!(kinds(&evs).contains(&"disconnect"));
    assert_eq!(st.receivers_live.load(std::sync::atomic::Ordering::Relaxed), 0);
}

#[test]
fn moved_antenna_is_reported() {
    let (mut a, _) = adapter(AdapterConfig::default(), &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    live(&mut a, &[1], &mut evs);
    evs.clear();
    let mut r = reg(&[(1, ClockType::Dump1090)]);
    r.get_mut(&1).unwrap().lat += 0.01;
    a.set_registry(r, &mut evs);
    assert_eq!(kinds(&evs), vec!["moved"]);
}

#[test]
fn two_clocks_under_one_id_are_left_out() {
    let (mut a, st) = adapter(AdapterConfig::default(), &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    let ld = |x: &std::sync::atomic::AtomicU64| x.load(std::sync::atomic::Ordering::Relaxed);
    // an aggregating feeder: two counters 50 s apart, interleaved
    for s in 0..40u64 {
        for (k, base) in [(0u64, 1_000_000u64), (1, 600_000_000)] {
            a.on_frame(&frame(1, base + s * 12_000_000 + k, &df11(0x3C0001)), 1000.0 + s as f64, &mut evs);
        }
    }
    a.tick(1040.0, &mut evs);
    assert!(!kinds(&evs).contains(&"connect"), "{:?}", kinds(&evs));
    assert_eq!(ld(&st.receivers_multi_clock), 1);
    assert_eq!(ld(&st.clock_resets), 0);
}

#[test]
fn a_live_receiver_that_turns_multi_clock_is_disconnected_then_retried() {
    let (mut a, st) = adapter(AdapterConfig::default(), &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    let ld = |x: &std::sync::atomic::AtomicU64| x.load(std::sync::atomic::Ordering::Relaxed);
    live(&mut a, &[1], &mut evs);
    evs.clear();
    let t0 = 1_000_000 + 22 * 12_000_000u64;
    // one restart (counter back near zero, past the frame-start shift): a reset, still live
    a.on_frame(&frame(1, 5_000, &df11(0x3C0001)), 1022.0, &mut evs);
    assert_eq!(kinds(&evs)[0], "reset");
    assert_eq!(ld(&st.clock_resets), 1);
    evs.clear();
    // then a second clock appears: jumps pile up
    for i in 0..4u64 {
        a.on_frame(&frame(1, t0 + i * 12_000_000, &df11(0x3C0001)), 1023.0 + i as f64, &mut evs);
        a.on_frame(&frame(1, 5_000 + (i + 1) * 12_000_000, &df11(0x3C0001)), 1023.2 + i as f64, &mut evs);
    }
    assert!(kinds(&evs).contains(&"disconnect"), "{:?}", kinds(&evs));
    // the restart and the next jump each reset; the third makes it multi-clock: no reset storm
    assert_eq!(ld(&st.clock_resets), 2);
    a.tick(1030.0, &mut evs);
    assert_eq!(ld(&st.receivers_multi_clock), 1);
    evs.clear();
    // the duplicate goes away; after 10 calm minutes and a probation it is back
    let base = 5_000 + 5 * 12_000_000u64;
    for s in 0..=640u64 {
        a.on_frame(&frame(1, base + s * 12_000_000, &df11(0x3C0001)), 1027.2 + s as f64, &mut evs);
    }
    assert!(kinds(&evs).contains(&"connect"), "{:?}", kinds(&evs));
    a.tick(1027.2 + 641.0, &mut evs);
    assert_eq!(ld(&st.receivers_multi_clock), 0);
    assert_eq!(ld(&st.receivers_live), 1);
}

#[test]
fn late_delivery_is_not_a_clock_jump() {
    let (mut a, st) = adapter(AdapterConfig::default(), &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    let ld = |x: &std::sync::atomic::AtomicU64| x.load(std::sync::atomic::Ordering::Relaxed);
    live(&mut a, &[1], &mut evs);
    evs.clear();
    // three 5 s stalls in a minute: frames come late, then in a burst
    let mut ts = 1_000_000 + 21 * 12_000_000u64;
    let mut wall = 1021.0;
    for _ in 0..3 {
        ts += 12_000_000; // generated 1 s later...
        wall += 6.0; // ...delivered 6 s later
        a.on_frame(&frame(1, ts, &df11(0x3C0001)), wall, &mut evs);
        for _ in 0..5 {
            ts += 12_000_000; // the backlog, then on time again
            a.on_frame(&frame(1, ts, &df11(0x3C0001)), wall, &mut evs);
        }
        wall += 0.01;
    }
    assert!(!kinds(&evs).contains(&"reset") && !kinds(&evs).contains(&"disconnect"), "{:?}", kinds(&evs));
    assert_eq!(ld(&st.clock_jumps), 0);
    assert_eq!(ld(&st.late_deliveries), 3);
    a.tick(wall, &mut evs);
    assert_eq!(ld(&st.receivers_live), 1);
}

#[test]
fn readsb_marker_timestamps_are_not_clock_jumps() {
    let (mut a, st) = adapter(AdapterConfig::default(), &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    let ld = |x: &std::sync::atomic::AtomicU64| x.load(std::sync::atomic::Ordering::Relaxed);
    live(&mut a, &[1], &mut evs);
    evs.clear();
    // A US feeder forwards UAT (and MLAT results) between its own frames,
    // stamped with readsb's markers instead of a receive time.
    for (i, ts) in [0xFF00_4D4C_4155u64, 0xFF00_4D4C_4154, 0xFF00_4D4C_4160, 0xFFFF_FFFF_FFFF].iter().enumerate() {
        a.on_frame(&frame(1, *ts, &df11(0x3C0001)), 1022.0 + i as f64 * 0.1, &mut evs);
        a.on_frame(&frame(1, 1_000_000 + (22 * 12_000_000) + i as u64 * 1_200_000, &df11(0x3C0001)), 1022.05 + i as f64 * 0.1, &mut evs);
    }
    a.tick(1023.0, &mut evs);
    assert!(!kinds(&evs).contains(&"reset"), "{:?}", kinds(&evs));
    assert!(!kinds(&evs).contains(&"disconnect"));
    assert_eq!(ld(&st.clock_jumps), 0);
    assert_eq!(ld(&st.frames_magic_ts), 4);
    assert_eq!(ld(&st.receivers_live), 1);
}

#[test]
fn mode_s_times_are_frame_start_like_mlat_client() {
    let (mut a, _) = adapter(AdapterConfig::default(), &[(1, ClockType::Dump1090)]);
    let mut evs = Vec::new();
    live(&mut a, &[1], &mut evs);
    evs.clear();
    // A Mode-S-only aircraft's DF11 (no ADS-B anywhere): an Mlat event.
    let ts = 1_000_000 + 22 * 12_000_000;
    a.on_frame(&frame(1, ts, &df11(0x3C0002)), 1022.0, &mut evs);
    let t = evs.iter().find_map(|e| match e {
        Event::Mlat { t, .. } => Some(*t),
        _ => None,
    });
    assert_eq!(t, Some((ts - 768) as f64), "{:?}", kinds(&evs));
}
