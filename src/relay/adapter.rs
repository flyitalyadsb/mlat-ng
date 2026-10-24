//! Feed frames -> engine events.
//!
//! The feed has no handshake, no requests and no clock events, so the adapter
//! supplies what mlat-client would have:
//! - Connect, once a mapped receiver's clock has run at 12 MHz for a
//!   probation period (Radarcape GPS and other formats arrive unflagged);
//! - ClockReset, when its timestamps jump against wall time;
//! - Seen/Lost, from the DF11/DF17 addresses it forwards (the engine only
//!   multilaterates aircraft some receiver reports as seen);
//! - Sync, as mlat-client builds it: a DF17 airborne position completes a
//!   pair with the latest opposite-parity one from the same receiver within
//!   5 s. Pairs are kept when hash(pair) % sync_every == 0, so the subset is
//!   the same at every receiver that heard the pair and clock pairs still meet;
//! - Mlat, for aircraft with no ADS-B position from any receiver (Mode-S-only,
//!   DF17 without position included) and, for the bench, for 1 in
//!   `bench_every` ADS-B aircraft from their non-DF17 replies only.

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use tokio::sync::mpsc;

use super::beast::Frame;
use super::registry::Receiver;
use crate::engine::fxhash;
use crate::modes::{self, EsKind};
use crate::types::{ClockType, ConnId, Event, ReceiverInfo, ToClient};

/// Feed receivers get connection ids in their own range, clear of mlat-client's.
pub const CONN_BASE: ConnId = 1 << 62;
const TICK_HZ: f64 = 12e6;
/// Max distance in time between the halves of a sync pair (mlat-client's).
const PAIR_WINDOW: f64 = 5.0;
/// A receiver's timestamps may disagree with wall time by this much (flush
/// and WAN jitter) before it counts as a clock jump.
const JUMP_TOLERANCE: f64 = 2.0;
const SEEN_TTL: f64 = 60.0;

#[derive(Debug, Clone)]
pub struct AdapterConfig {
    pub sync_every: u64,
    pub bench_every: u64,
    pub modes_only_after: f64,
    /// Seconds of frames before a receiver's clock rate is checked.
    pub probation: f64,
    pub idle_timeout: f64,
}

impl Default for AdapterConfig {
    fn default() -> Self {
        AdapterConfig { sync_every: 2, bench_every: 10, modes_only_after: 20.0, probation: 20.0, idle_timeout: 120.0 }
    }
}

/// Counters for /metrics (gauges are rewritten on every tick).
#[derive(Default)]
pub struct AdapterStats {
    pub frames: AtomicU64,
    pub frames_modeac: AtomicU64,
    pub frames_zero_ts: AtomicU64,
    pub frames_unmapped: AtomicU64,
    pub frames_not_live: AtomicU64,
    pub ev_sync: AtomicU64,
    pub ev_mlat_modes_only: AtomicU64,
    pub ev_mlat_bench: AtomicU64,
    pub clock_jumps: AtomicU64,
    pub receivers_live: AtomicU64,
    pub receivers_probation: AtomicU64,
    pub receivers_bad_clock: AtomicU64,
    pub receivers_unmapped: AtomicU64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Probation,
    Live,
    BadClock,
}

struct RxState {
    conn: ConnId,
    phase: Phase,
    /// (ts, wall) where the current rate check started
    first: (u64, f64),
    last: (u64, f64),
    /// latest even [0] and odd [1] airborne position: (ts, frame)
    cpr: HashMap<u32, [Option<(u64, [u8; 14])>; 2]>,
    seen: HashMap<u32, f64>,
    unsent_seen: Vec<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Route {
    ModesOnly,
    Bench,
}

pub struct Adapter {
    cfg: AdapterConfig,
    stats: Arc<AdapterStats>,
    registry: HashMap<u64, Receiver>,
    rxs: HashMap<u64, RxState>,
    unmapped: HashMap<u64, f64>,
    /// last ADS-B airborne position (any receiver), wall time
    adsb_pos: HashMap<u32, f64>,
    next_conn: ConnId,
    tx: mpsc::Sender<ToClient>,
}

impl Adapter {
    pub fn new(cfg: AdapterConfig, stats: Arc<AdapterStats>) -> Adapter {
        // Feed receivers cannot be asked for anything: whatever the engine
        // would send them goes to a closed channel.
        let (tx, _) = mpsc::channel(1);
        Adapter {
            cfg,
            stats,
            registry: HashMap::new(),
            rxs: HashMap::new(),
            unmapped: HashMap::new(),
            adsb_pos: HashMap::new(),
            next_conn: CONN_BASE,
            tx,
        }
    }

    /// Merge a registry snapshot. Entries are never dropped (a feeder's
    /// mlat-client may be reconnecting); a moved antenna is reported.
    pub fn set_registry(&mut self, reg: HashMap<u64, Receiver>, out: &mut Vec<Event>) {
        for (k, r) in reg {
            if let (Some(old), Some(rx)) = (self.registry.get(&k), self.rxs.get(&k)) {
                let moved = crate::geo::haversine_m(old.lat, old.lon, r.lat, r.lon) > 100.0 || (old.alt - r.alt).abs() > 100.0;
                if moved && rx.phase == Phase::Live {
                    out.push(Event::PositionUpdate { conn: rx.conn, lat: r.lat, lon: r.lon, alt: r.alt });
                }
            }
            self.registry.insert(k, r);
        }
    }

    pub fn registry_len(&self) -> usize {
        self.registry.len()
    }

    fn route(&self, addr: u32, wall: f64) -> Option<Route> {
        let age = self.adsb_pos.get(&addr).map(|t| wall - t).unwrap_or(f64::INFINITY);
        if age > self.cfg.modes_only_after {
            Some(Route::ModesOnly)
        } else if self.cfg.bench_every > 0 && fxhash(&addr.to_be_bytes()[1..]) % self.cfg.bench_every == 0 {
            Some(Route::Bench)
        } else {
            None
        }
    }

    pub fn on_frame(&mut self, f: &Frame, wall: f64, out: &mut Vec<Event>) {
        self.stats.frames.fetch_add(1, Ordering::Relaxed);
        if f.kind == b'1' {
            self.stats.frames_modeac.fetch_add(1, Ordering::Relaxed);
            return;
        }
        if f.ts == 0 {
            self.stats.frames_zero_ts.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let twelve_mhz = match self.registry.get(&f.rid) {
            Some(r) => matches!(r.clock, ClockType::Dump1090 | ClockType::Beast),
            None => {
                self.unmapped.insert(f.rid, wall);
                self.stats.frames_unmapped.fetch_add(1, Ordering::Relaxed);
                return;
            }
        };
        if !self.rxs.contains_key(&f.rid) {
            let conn = self.next_conn;
            self.next_conn += 1;
            self.rxs.insert(
                f.rid,
                RxState {
                    conn,
                    phase: Phase::Probation,
                    first: (f.ts, wall),
                    last: (f.ts, wall),
                    cpr: HashMap::new(),
                    seen: HashMap::new(),
                    unsent_seen: Vec::new(),
                },
            );
        }
        enum Next {
            SeenOnly,
            Connect(ConnId),
            Live,
        }
        let next = {
            let rx = self.rxs.get_mut(&f.rid).unwrap();
            // Clock continuity against wall time.
            let dts = (f.ts as f64 - rx.last.0 as f64) / TICK_HZ;
            let dwall = wall - rx.last.1;
            if rx.phase != Phase::BadClock && (dts - dwall).abs() > JUMP_TOLERANCE + 1e-3 * dwall.abs() {
                self.stats.clock_jumps.fetch_add(1, Ordering::Relaxed);
                if rx.phase == Phase::Live {
                    out.push(Event::ClockReset { conn: rx.conn, reason: "feed timestamp jump" });
                }
                rx.cpr.clear();
                rx.first = (f.ts, wall);
            }
            rx.last = (f.ts, wall);
            match rx.phase {
                Phase::BadClock => {
                    self.stats.frames_not_live.fetch_add(1, Ordering::Relaxed);
                    return;
                }
                Phase::Live => Next::Live,
                Phase::Probation => {
                    let span = wall - rx.first.1;
                    if span < self.cfg.probation {
                        Next::SeenOnly
                    } else {
                        let rate = (f.ts as f64 - rx.first.0 as f64) / TICK_HZ / span;
                        if !twelve_mhz || (rate - 1.0).abs() > 0.05 {
                            rx.phase = Phase::BadClock;
                            self.stats.frames_not_live.fetch_add(1, Ordering::Relaxed);
                            return;
                        }
                        rx.phase = Phase::Live;
                        Next::Connect(rx.conn)
                    }
                }
            }
        };
        match next {
            Next::SeenOnly => {
                self.note_seen_only(f, wall);
                self.stats.frames_not_live.fetch_add(1, Ordering::Relaxed);
                return;
            }
            Next::Connect(conn) => {
                let reg = &self.registry[&f.rid];
                out.push(Event::Connect {
                    conn,
                    info: ReceiverInfo {
                        user: format!("{}~feed", reg.user),
                        uuid: None,
                        lat: reg.lat,
                        lon: reg.lon,
                        alt: reg.alt,
                        clock: reg.clock,
                        privacy: reg.privacy,
                        source_ip: "feed".into(),
                        client_version: "feed".into(),
                        compress: "none",
                    },
                    tx: self.tx.clone(),
                });
            }
            Next::Live => {}
        }
        self.live_frame(f, wall, out);
    }

    /// During probation: remember which aircraft the receiver hears, so its
    /// first Seen goes out with the Connect.
    fn note_seen_only(&mut self, f: &Frame, wall: f64) {
        let msg = f.bytes();
        let df = msg[0] >> 3;
        let addr = match df {
            17 if msg.len() == 14 && modes::crc_residual(msg) == 0 => u24(&msg[1..4]),
            11 if msg.len() == 7 && modes::crc_residual(msg) <= 0x7f => u24(&msg[1..4]),
            _ => return,
        };
        let rx = self.rxs.get_mut(&f.rid).unwrap();
        if rx.seen.insert(addr, wall).is_none() {
            rx.unsent_seen.push(addr);
        }
    }

    fn live_frame(&mut self, f: &Frame, wall: f64, out: &mut Vec<Event>) {
        let msg = f.bytes();
        let df = msg[0] >> 3;
        let conn = self.rxs[&f.rid].conn;
        let (addr, seen_ok) = match df {
            17 if msg.len() == 14 => {
                let Some(d) = modes::decode(msg) else { return };
                if d.crc_ok != Some(true) {
                    return;
                }
                let addr = d.address;
                self.mark_seen(f.rid, addr, wall);
                if let Some(es) = d.es.as_ref().filter(|e| e.kind == EsKind::AirbornePosition && (9..=18).contains(&e.tc)) {
                    self.adsb_pos.insert(addr, wall);
                    // NUCp >= 6 is TC 9..12: the engine refuses the rest.
                    if es.tc <= 12 {
                        self.pair(f, addr, es.odd, conn, out);
                    }
                    return;
                }
                // ADS-B without a position: as good as Mode-S-only.
                if self.route(addr, wall) == Some(Route::ModesOnly) {
                    self.stats.ev_mlat_modes_only.fetch_add(1, Ordering::Relaxed);
                    out.push(Event::Mlat { conn, t: f.ts as f64, msg: msg.to_vec(), rx: wall });
                }
                return;
            }
            11 if msg.len() == 7 => {
                if modes::crc_residual(msg) > 0x7f {
                    return;
                }
                (u24(&msg[1..4]), true)
            }
            0 | 4 | 5 | 16 | 20 | 21 => (modes::crc_residual(msg), false),
            _ => return,
        };
        if seen_ok {
            self.mark_seen(f.rid, addr, wall);
        }
        match self.route(addr, wall) {
            Some(Route::ModesOnly) => {
                self.stats.ev_mlat_modes_only.fetch_add(1, Ordering::Relaxed);
                out.push(Event::Mlat { conn, t: f.ts as f64, msg: msg.to_vec(), rx: wall });
            }
            Some(Route::Bench) => {
                self.stats.ev_mlat_bench.fetch_add(1, Ordering::Relaxed);
                out.push(Event::Mlat { conn, t: f.ts as f64, msg: msg.to_vec(), rx: wall });
            }
            None => {}
        }
    }

    fn mark_seen(&mut self, rid: u64, addr: u32, wall: f64) {
        let rx = self.rxs.get_mut(&rid).unwrap();
        if rx.seen.insert(addr, wall).is_none() {
            rx.unsent_seen.push(addr);
        }
    }

    fn pair(&mut self, f: &Frame, addr: u32, odd: bool, conn: ConnId, out: &mut Vec<Event>) {
        let rx = self.rxs.get_mut(&f.rid).unwrap();
        let slot = rx.cpr.entry(addr).or_insert([None, None]);
        let p = odd as usize;
        slot[p] = Some((f.ts, f.msg));
        let Some((ots, om)) = slot[1 - p] else { return };
        if ots >= f.ts || (f.ts - ots) as f64 / TICK_HZ > PAIR_WINDOW {
            return;
        }
        let (et, em, ot, omsg) = if odd { (ots, om, f.ts, f.msg) } else { (f.ts, f.msg, ots, om) };
        let mut key = [0u8; 28];
        key[..14].copy_from_slice(&em);
        key[14..].copy_from_slice(&omsg);
        if self.cfg.sync_every > 1 && fxhash(&key) % self.cfg.sync_every != 0 {
            return;
        }
        self.stats.ev_sync.fetch_add(1, Ordering::Relaxed);
        out.push(Event::Sync { conn, et: et as f64, ot: ot as f64, em: em.to_vec(), om: omsg.to_vec() });
    }

    /// Once a second: Seen/Lost, idle receivers, stale state, gauges.
    pub fn tick(&mut self, wall: f64, out: &mut Vec<Event>) {
        let idle = self.cfg.idle_timeout;
        let mut gone = Vec::new();
        let (mut live, mut prob, mut bad) = (0u64, 0u64, 0u64);
        for (rid, rx) in self.rxs.iter_mut() {
            if wall - rx.last.1 > idle {
                if rx.phase == Phase::Live {
                    out.push(Event::Disconnect { conn: rx.conn });
                }
                gone.push(*rid);
                continue;
            }
            match rx.phase {
                Phase::Live => live += 1,
                Phase::Probation => prob += 1,
                Phase::BadClock => bad += 1,
            }
            let mut lost = Vec::new();
            rx.seen.retain(|a, t| {
                let keep = wall - *t <= SEEN_TTL;
                if !keep {
                    lost.push(*a);
                }
                keep
            });
            rx.cpr.retain(|_, s| s.iter().flatten().any(|(ts, _)| rx.last.0.saturating_sub(*ts) as f64 / TICK_HZ <= 10.0));
            if rx.phase == Phase::Live {
                if !rx.unsent_seen.is_empty() {
                    out.push(Event::Seen { conn: rx.conn, icao: std::mem::take(&mut rx.unsent_seen) });
                }
                if !lost.is_empty() {
                    out.push(Event::Lost { conn: rx.conn, icao: lost });
                }
            } else {
                rx.unsent_seen.retain(|a| rx.seen.contains_key(a));
            }
        }
        for rid in gone {
            self.rxs.remove(&rid);
        }
        self.adsb_pos.retain(|_, t| wall - *t <= 120.0);
        self.unmapped.retain(|_, t| wall - *t <= 300.0);
        self.stats.receivers_live.store(live, Ordering::Relaxed);
        self.stats.receivers_probation.store(prob, Ordering::Relaxed);
        self.stats.receivers_bad_clock.store(bad, Ordering::Relaxed);
        self.stats.receivers_unmapped.store(self.unmapped.len() as u64, Ordering::Relaxed);
    }
}

fn u24(b: &[u8]) -> u32 {
    (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2])
}
