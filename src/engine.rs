//! The engine: one thread that owns all state (receivers, clock pairs, sync
//! points, message groups, aircraft, tracks) and processes events in arrival
//! order. Arrival order matters: a clock pair's observations must be applied in
//! time order, and a single owner makes that free and deterministic.
//!
//! Pipeline per event:
//! - Sync (DF17 even/odd pair): validate once per distinct pair of frames,
//!   then every receiver that heard it updates its clock pair with every
//!   receiver that heard it before (bounded by a per-receiver pair budget).
//!   5% of sync points are held out of clock sync and multilaterated instead,
//!   against their own ADS-B position: an honest, continuous accuracy measure.
//! - Mlat (any Mode S frame): copies are grouped by exact bytes for
//!   `mlat_delay`; then normalised, split into transmissions, solved, and fed
//!   to the aircraft's track (a full fix, or a direct TDOA update when too few
//!   receivers heard it for a stand-alone fix).
//! - Mode A/C (false_ac frames): same timing path; each solution is first
//!   associated with a Mode S track (by squawk or Mode C altitude and
//!   position), where it becomes one more TDOA update for that aircraft; only
//!   unassociated, repeatedly consistent solutions become Mode A/C tracks.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use nalgebra::DMatrix;
use tokio::sync::{broadcast, mpsc};

use crate::clock::{band_of, Outcome, PairModel, PAIR_BUDGET};
use crate::geo::{dist, ecef_to_llh, llh_to_ecef, Ecef, Llh, C_AIR, FT_TO_M};
use crate::modes;
use crate::norm::{self, Copy, Node};
use crate::output::{sbs_line, SbsFix};
use crate::solver::{self, Fix, Meas, SolveInput};
use crate::track::{Gate, Track};
use crate::types::{ConnId, Event, ReceiverInfo, ToClient};

/// Aircraft-to-receiver range beyond which a sync point is not used (radio
/// horizon at FL450 is ~480 km).
const MAX_RANGE: f64 = 500e3;
/// Maximum distance between the even and odd positions of a sync pair.
const MAX_PAIR_SPREAD: f64 = 10e3;
const SYNC_TTL: f64 = 10.0;
/// A converged pair is not updated more often than this: beyond it more
/// observations add CPU, not accuracy.
const PAIR_MIN_INTERVAL: f64 = 0.25;
/// Cap on copies kept per message group.
const MAX_COPIES: usize = 40;
/// Mode A/C payloads that are shared by too many aircraft at once (VFR,
/// conspicuity, defaults) or are line noise, as acbridge.py/mlat-client.
const MODEAC_BLACKLIST: [u16; 20] = [
    0x1200, 0x7000, 0x2000, 0x0000, 0x1000, 0x0002, 0x0004, 0x0006, 0x0040, 0x0060, 0x0020, 0x0030, 0x0010, 0x0410,
    0x0430, 0x0420, 0x0460, 0x0440, 0x0640, 0x0660,
];

pub fn unix_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs_f64()).unwrap_or(0.0)
}

#[derive(Debug, Clone)]
pub struct EngineConfig {
    pub shadow: bool,
    pub mlat_delay: f64,
    /// 1 in N sync points is held out for self-evaluation (0 disables).
    pub holdout_every: u64,
}

type RxKey = u64;

struct Rx {
    key: RxKey,
    conn: ConnId,
    info: ReceiverInfo,
    pos: Ecef,
    freq: f64,
    jitter: f64,
    peers: HashSet<RxKey>,
    band_count: [u16; 4],
    seen: HashSet<u32>,
    tx: mpsc::Sender<ToClient>,
    suspect: bool,
    connected: f64,
    sync_msgs: u64,
    mlat_msgs: u64,
    /// Copies this receiver contributed to accepted multilateration updates.
    used: u64,
    /// Timestamp noise relative to its clock type's nominal jitter, from the
    /// noise its clock pairs observe (1 = as specified).
    noise_scale: f64,
}

#[derive(Clone, Copy)]
struct SyncObs {
    rx: RxKey,
    /// propagation-corrected times of A and B, receiver seconds
    tda: f64,
    tdb: f64,
    /// raw local time of B (for holdout evaluation)
    sb: f64,
}

struct Variant {
    interval: f64,
    holdout: bool,
    obs: Vec<SyncObs>,
}

struct SyncEntry {
    /// (pos A, pos B, icao, altitude ft of B) when the pair is a usable reference
    valid: Option<(Ecef, Ecef, u32, i32)>,
    variants: Vec<Variant>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct MsgKey {
    len: u8,
    b: [u8; 14],
}

impl MsgKey {
    fn new(m: &[u8]) -> Option<MsgKey> {
        if m.len() != 7 && m.len() != 14 {
            return None;
        }
        let mut b = [0u8; 14];
        b[..m.len()].copy_from_slice(m);
        Some(MsgKey { len: m.len() as u8, b })
    }
    fn bytes(&self) -> &[u8] {
        &self.b[..self.len as usize]
    }
}

struct Group {
    first_wall: f64,
    copies: Vec<(RxKey, f64)>,
    /// Ground truth for holdout groups: ADS-B position and altitude (ft).
    truth: Option<(Ecef, i32)>,
}

#[derive(Default)]
struct Aircraft {
    tracking: u32,
    alt_ft: Option<i32>,
    alt_t: f64,
    squawk: Option<u16>,
    callsign: Option<String>,
    last_adsb: f64,
    track: Option<Track>,
    track_rejects: u32,
    last_fix: Option<(Ecef, f64)>,
    last_out: f64,
    /// Time of the last full fix accepted into the track (TDOA updates don't
    /// count): a track is only published while a real fix anchors it.
    last_fix_update: f64,
    /// Last ADS-B position from a validated sync point, and its arrival time,
    /// plus a velocity from the previous one: the in-engine truth used to
    /// diagnose multilateration of aircraft that also broadcast ADS-B.
    adsb: Option<(Ecef, f64)>,
    adsb_vel: Option<[f64; 3]>,
}

struct AcTrack {
    code: u16,
    mode_c: bool,
    track: Track,
    last_out: f64,
}

/// One multilaterated update of an aircraft that also broadcasts ADS-B,
/// compared with its ADS-B position at the same time.
#[derive(Debug, Clone, serde::Serialize)]
pub struct Diag {
    pub t: f64,
    pub icao: String,
    pub df: u8,
    /// receivers in this update
    pub n: usize,
    /// transmissions (clusters) found in the message group, and its copies
    pub clusters: usize,
    pub copies: usize,
    /// clock-pair edges used / dropped by the normalisation
    pub edges: usize,
    pub dropped: usize,
    /// stand-alone fix (None for a TDOA update)
    pub chi2: Option<f64>,
    pub dof: Option<i32>,
    pub err_h: Option<f64>,
    pub gdop: Option<f64>,
    /// horizontal error of the stand-alone fix vs ADS-B (m)
    pub fix_err_m: Option<f64>,
    /// horizontal error of what was published (track position) vs ADS-B (m)
    pub out_err_m: Option<f64>,
    pub accepted: bool,
    pub confirmed: bool,
    pub users: Vec<String>,
}

/// Counters published for /metrics. Monotonic unless named *_now.
#[derive(Default)]
pub struct Stats {
    pub sync_msgs: AtomicU64,
    pub sync_points: AtomicU64,
    pub sync_invalid: AtomicU64,
    pub pair_updates: AtomicU64,
    pub pair_rejects: AtomicU64,
    pub pair_resets: AtomicU64,
    pub pair_resets_drift: AtomicU64,
    pub tracks_born: AtomicU64,
    pub tracks_refused_dof0: AtomicU64,
    pub outputs_held_no_fix: AtomicU64,
    pub pair_budget_refused: AtomicU64,
    pub mlat_msgs: AtomicU64,
    pub groups: AtomicU64,
    pub groups_unknown_ac: AtomicU64,
    pub groups_no_norm: AtomicU64,
    pub clusters: AtomicU64,
    pub solves: AtomicU64,
    pub fixes: AtomicU64,
    pub track_fix_updates: AtomicU64,
    pub track_tdoa_updates: AtomicU64,
    pub track_rejects: AtomicU64,
    pub outputs: AtomicU64,
    pub modeac_groups: AtomicU64,
    pub modeac_blacklisted: AtomicU64,
    pub modeac_fixes: AtomicU64,
    pub modeac_assoc: AtomicU64,
    pub modeac_outputs: AtomicU64,
    pub holdout_solved: AtomicU64,
    pub holdout_failed: AtomicU64,
    pub loop_late_ms_max: AtomicU64,
    /// Snapshot for /state.json and the gauges, rewritten every second.
    pub snapshot: Mutex<Snapshot>,
    /// Worst recent ADS-B-checked updates, for /debug.json.
    pub debug: Mutex<Vec<Diag>>,
}

#[derive(Default, Clone, serde::Serialize)]
pub struct Snapshot {
    pub receivers: usize,
    pub receivers_suspect: usize,
    pub pairs: usize,
    pub pairs_usable: usize,
    pub sync_entries: usize,
    pub pending_groups: usize,
    pub aircraft: usize,
    pub tracks: usize,
    pub tracks_confirmed: usize,
    pub modeac_tracks: usize,
    pub queue_depth: usize,
    /// Holdout horizontal error quantiles (m) over the last 10 minutes.
    pub holdout_n: usize,
    pub holdout_p50: f64,
    pub holdout_p90: f64,
    pub holdout_p99: f64,
    /// MLAT vs ADS-B for aircraft that broadcast both, last 10 minutes, by
    /// receivers per update: (bucket, count, fix p50, fix p90, out p50, out p90).
    pub adsb_check: Vec<(String, usize, f64, f64, f64, f64)>,
    pub started: f64,
}

pub struct Engine {
    cfg: EngineConfig,
    stats: Arc<Stats>,
    out: broadcast::Sender<Arc<str>>,
    rx_by_conn: HashMap<ConnId, RxKey>,
    rxs: HashMap<RxKey, Rx>,
    next_key: RxKey,
    pairs: HashMap<(RxKey, RxKey), PairModel>,
    sync: HashMap<[u8; 28], SyncEntry>,
    sync_expiry: VecDeque<(f64, [u8; 28])>,
    groups: HashMap<MsgKey, Group>,
    group_due: VecDeque<(f64, MsgKey)>,
    holdout_due: VecDeque<(f64, MsgKey)>,
    holdout_groups: HashMap<MsgKey, Group>,
    aircraft: HashMap<u32, Aircraft>,
    actracks: HashMap<u32, AcTrack>,
    next_ac_id: u32,
    holdout_err: VecDeque<(f64, f64)>,
    /// (time, receivers, fix error, published error) of ADS-B-checked updates
    adsb_err: VecDeque<(f64, usize, Option<f64>, Option<f64>)>,
    diag_bad: VecDeque<Diag>,
}

fn pair_key(a: RxKey, b: RxKey) -> (RxKey, RxKey) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

impl Engine {
    pub fn new(cfg: EngineConfig, stats: Arc<Stats>, out: broadcast::Sender<Arc<str>>) -> Engine {
        stats.snapshot.lock().unwrap().started = unix_now();
        Engine {
            cfg,
            stats,
            out,
            rx_by_conn: HashMap::new(),
            rxs: HashMap::new(),
            next_key: 1,
            pairs: HashMap::new(),
            sync: HashMap::new(),
            sync_expiry: VecDeque::new(),
            groups: HashMap::new(),
            group_due: VecDeque::new(),
            holdout_due: VecDeque::new(),
            holdout_groups: HashMap::new(),
            aircraft: HashMap::new(),
            actracks: HashMap::new(),
            next_ac_id: 0,
            holdout_err: VecDeque::new(),
            adsb_err: VecDeque::new(),
            diag_bad: VecDeque::new(),
        }
    }

    /// Run until the event channel closes. Meant for a dedicated thread with
    /// its own current-thread runtime, away from the network workers.
    pub async fn run(mut self, mut rx: mpsc::Receiver<Event>) {
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut last_tick = Instant::now();
        let mut last_second = Instant::now();
        let mut last_ten = Instant::now();
        loop {
            tokio::select! {
                biased;
                _ = tick.tick() => {
                    let late = last_tick.elapsed().as_millis() as u64;
                    last_tick = Instant::now();
                    self.stats.loop_late_ms_max.fetch_max(late.saturating_sub(50), Ordering::Relaxed);
                    let now = unix_now();
                    self.process_due(now);
                    if last_second.elapsed() >= Duration::from_secs(1) {
                        last_second = Instant::now();
                        self.expire(now);
                        self.publish(rx.len());
                    }
                    if last_ten.elapsed() >= Duration::from_secs(10) {
                        last_ten = Instant::now();
                        self.housekeeping(now);
                    }
                }
                ev = rx.recv() => match ev {
                    Some(ev) => self.handle(ev),
                    None => return,
                },
            }
        }
    }

    fn handle(&mut self, ev: Event) {
        self.handle_at(ev, unix_now());
    }

    /// Process one event at wall time `now` (separate so tests can drive a
    /// simulated clock).
    pub(crate) fn handle_at(&mut self, ev: Event, now: f64) {
        match ev {
            Event::Connect { conn, info, tx } => self.connect(conn, info, tx, now),
            Event::Disconnect { conn } => self.disconnect(conn),
            Event::Sync { conn, et, ot, em, om } => self.on_sync(conn, et, ot, &em, &om, now),
            Event::Mlat { conn, t, msg, rx } => self.on_mlat(conn, t, &msg, rx.min(now)),
            Event::Seen { conn, icao } => self.on_seen(conn, &icao),
            Event::Lost { conn, icao } => self.on_lost(conn, &icao),
            Event::RateReport { .. } => {}
            Event::ClockReset { conn, .. } => self.clock_reset(conn),
            Event::PositionUpdate { conn, lat, lon, alt } => {
                if let Some(k) = self.rx_by_conn.get(&conn).copied() {
                    // A moved antenna invalidates every pair it is in.
                    self.drop_pairs(k);
                    if let Some(r) = self.rxs.get_mut(&k) {
                        r.info.lat = lat;
                        r.info.lon = lon;
                        r.info.alt = alt;
                        r.pos = llh_to_ecef(Llh { lat, lon, alt });
                    }
                }
            }
        }
    }

    fn connect(&mut self, conn: ConnId, info: ReceiverInfo, tx: mpsc::Sender<ToClient>, now: f64) {
        let key = self.next_key;
        self.next_key += 1;
        let pos = llh_to_ecef(Llh { lat: info.lat, lon: info.lon, alt: info.alt });
        let r = Rx {
            key,
            conn,
            freq: info.clock.freq(),
            jitter: info.clock.jitter(),
            info,
            pos,
            peers: HashSet::new(),
            band_count: [0; 4],
            seen: HashSet::new(),
            tx,
            suspect: false,
            connected: now,
            sync_msgs: 0,
            mlat_msgs: 0,
            used: 0,
            noise_scale: 1.0,
        };
        self.rx_by_conn.insert(conn, key);
        self.rxs.insert(key, r);
    }

    fn drop_pairs(&mut self, k: RxKey) {
        let peers: Vec<RxKey> = match self.rxs.get_mut(&k) {
            Some(r) => {
                r.band_count = [0; 4];
                r.peers.drain().collect()
            }
            None => return,
        };
        for p in peers {
            if let Some(m) = self.pairs.remove(&pair_key(k, p)) {
                if let Some(pr) = self.rxs.get_mut(&p) {
                    pr.peers.remove(&k);
                    pr.band_count[m.band as usize] = pr.band_count[m.band as usize].saturating_sub(1);
                }
            }
        }
    }

    fn disconnect(&mut self, conn: ConnId) {
        let Some(k) = self.rx_by_conn.remove(&conn) else { return };
        self.drop_pairs(k);
        if let Some(r) = self.rxs.remove(&k) {
            for icao in r.seen {
                if let Some(a) = self.aircraft.get_mut(&icao) {
                    a.tracking = a.tracking.saturating_sub(1);
                }
            }
        }
    }

    fn clock_reset(&mut self, conn: ConnId) {
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        let peers: Vec<RxKey> = self.rxs.get(&k).map(|r| r.peers.iter().copied().collect()).unwrap_or_default();
        for p in peers {
            if let Some(m) = self.pairs.get_mut(&pair_key(k, p)) {
                m.reset_offset();
            }
        }
    }

    fn on_seen(&mut self, conn: ConnId, icao: &[u32]) {
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        let Some(r) = self.rxs.get_mut(&k) else { return };
        let mut start = Vec::new();
        for &a in icao {
            if r.seen.insert(a) {
                self.aircraft.entry(a).or_default().tracking += 1;
                start.push(a);
            }
        }
        if !self.cfg.shadow && !start.is_empty() {
            // Serve mode (experimental): ask for every aircraft the receiver
            // hears. Never used in shadow mode, where the primary decides.
            let _ = r.tx.try_send(ToClient::StartSending(start));
        }
    }

    fn on_lost(&mut self, conn: ConnId, icao: &[u32]) {
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        let Some(r) = self.rxs.get_mut(&k) else { return };
        for &a in icao {
            if r.seen.remove(&a) {
                if let Some(ac) = self.aircraft.get_mut(&a) {
                    ac.tracking = ac.tracking.saturating_sub(1);
                }
            }
        }
    }

    // ---------------------------------------------------------------- sync --

    fn validate_sync(&self, ma: &[u8], mb: &[u8]) -> Option<(Ecef, Ecef, u32, i32)> {
        let a = modes::decode(ma)?;
        let b = modes::decode(mb)?;
        if a.df != 17 || b.df != 17 || a.crc_ok != Some(true) || b.crc_ok != Some(true) || a.address != b.address {
            return None;
        }
        let (ea, eb) = (a.es.as_ref()?, b.es.as_ref()?);
        if ea.kind != modes::EsKind::AirbornePosition || eb.kind != modes::EsKind::AirbornePosition {
            return None;
        }
        if ea.odd == eb.odd || ea.nuc.unwrap_or(0) < 6 || eb.nuc.unwrap_or(0) < 6 {
            return None;
        }
        let (alt_a, alt_b) = (a.altitude_ft?, b.altitude_ft?);
        if (alt_a - alt_b).abs() > 5000 {
            return None;
        }
        let (even, odd, alt_e, alt_o) = if !ea.odd { (ea, eb, alt_a, alt_b) } else { (eb, ea, alt_b, alt_a) };
        let (elat, elon, olat, olon) = modes::cpr_global_airborne(even.lat_cpr, even.lon_cpr, odd.lat_cpr, odd.lon_cpr)?;
        for (lat, lon) in [(elat, elon), (olat, olon)] {
            if !(-85.0..=85.0).contains(&lat) || lat == 0.0 || lon == 0.0 {
                return None;
            }
        }
        let pe = llh_to_ecef(Llh { lat: elat, lon: elon, alt: alt_e as f64 * FT_TO_M });
        let po = llh_to_ecef(Llh { lat: olat, lon: olon, alt: alt_o as f64 * FT_TO_M });
        if dist(&pe, &po) > MAX_PAIR_SPREAD {
            return None;
        }
        let (pa, pb) = if !ea.odd { (pe, po) } else { (po, pe) };
        Some((pa, pb, a.address, alt_b))
    }

    fn on_sync(&mut self, conn: ConnId, et: f64, ot: f64, em: &[u8], om: &[u8], now: f64) {
        self.stats.sync_msgs.fetch_add(1, Ordering::Relaxed);
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        if em.len() != 14 || om.len() != 14 {
            return;
        }
        let (ta, tb, ma, mb) = if et < ot { (et, ot, em, om) } else { (ot, et, om, em) };
        let (freq, rpos) = match self.rxs.get_mut(&k) {
            Some(r) => {
                r.sync_msgs += 1;
                (r.freq, r.pos)
            }
            None => return,
        };
        let interval = (tb - ta) / freq;
        if !(interval > 0.0 && interval <= 5.0) {
            return;
        }
        let mut key = [0u8; 28];
        key[..14].copy_from_slice(ma);
        key[14..].copy_from_slice(mb);
        if !self.sync.contains_key(&key) {
            let valid = self.validate_sync(ma, mb);
            if valid.is_none() {
                self.stats.sync_invalid.fetch_add(1, Ordering::Relaxed);
            } else {
                self.stats.sync_points.fetch_add(1, Ordering::Relaxed);
            }
            self.sync.insert(key, SyncEntry { valid, variants: Vec::new() });
            self.sync_expiry.push_back((now, key));
        }
        let entry = self.sync.get_mut(&key).unwrap();
        let Some((pa, pb, icao, alt_b)) = entry.valid else { return };
        let (da, db) = (dist(&pa, &rpos), dist(&pb, &rpos));
        if da > MAX_RANGE || db > MAX_RANGE {
            return;
        }
        let obs = SyncObs { rx: k, tda: ta / freq - da / C_AIR, tdb: tb / freq - db / C_AIR, sb: tb / freq };
        // Identical bytes can be two transmissions (a stationary aircraft
        // repeats itself): the measured interval tells them apart.
        let vi = match entry.variants.iter().position(|v| (v.interval - interval).abs() < 0.75e-3) {
            Some(i) => i,
            None => {
                let holdout = self.cfg.holdout_every > 0
                    && entry.variants.is_empty()
                    && fxhash(&key) % self.cfg.holdout_every == 0;
                entry.variants.push(Variant { interval, holdout, obs: Vec::new() });
                entry.variants.len() - 1
            }
        };
        if let Some(ac) = self.aircraft.get_mut(&icao) {
            ac.last_adsb = now;
            ac.alt_ft = Some(alt_b);
            ac.alt_t = now;
            if let Some((p0, t0)) = ac.adsb {
                let dt = now - t0;
                if dt > 0.5 && dt < 15.0 {
                    ac.adsb_vel = Some([(pb[0] - p0[0]) / dt, (pb[1] - p0[1]) / dt, (pb[2] - p0[2]) / dt]);
                }
            }
            if ac.adsb.map(|(_, t0)| now - t0 > 0.5).unwrap_or(true) {
                ac.adsb = Some((pb, now));
            }
        }
        let variant = &mut entry.variants[vi];
        if variant.obs.iter().any(|o| o.rx == k) {
            return;
        }
        if variant.holdout {
            if variant.obs.is_empty() {
                if let Some(mk) = MsgKey::new(mb) {
                    self.holdout_due.push_back((now + 1.5, mk));
                    self.holdout_groups.insert(mk, Group { first_wall: now, copies: Vec::new(), truth: Some((pb, alt_b)) });
                }
            }
            variant.obs.push(obs);
            if let Some(mk) = MsgKey::new(mb) {
                if let Some(g) = self.holdout_groups.get_mut(&mk) {
                    if g.copies.len() < MAX_COPIES {
                        g.copies.push((k, obs.sb));
                    }
                }
            }
            return;
        }
        let previous: Vec<SyncObs> = variant.obs.clone();
        variant.obs.push(obs);
        for q in previous {
            self.pair_update(k, &obs, q, now);
        }
    }

    fn pair_update(&mut self, k: RxKey, o: &SyncObs, q: SyncObs, now: f64) {
        let pk = pair_key(k, q.rx);
        if !self.pairs.contains_key(&pk) {
            let (Some(r1), Some(r2)) = (self.rxs.get(&k), self.rxs.get(&q.rx)) else { return };
            let band = band_of(dist(&r1.pos, &r2.pos));
            let b = band as usize;
            let (c1, c2) = (r1.band_count[b], r2.band_count[b]);
            if (c1 >= PAIR_BUDGET[b] && c2 >= PAIR_BUDGET[b]) || c1 >= 2 * PAIR_BUDGET[b] || c2 >= 2 * PAIR_BUDGET[b] {
                self.stats.pair_budget_refused.fetch_add(1, Ordering::Relaxed);
                return;
            }
            let m = PairModel::new(r1.info.clock, r2.info.clock, band);
            self.pairs.insert(pk, m);
            for (x, y) in [(k, q.rx), (q.rx, k)] {
                if let Some(r) = self.rxs.get_mut(&x) {
                    r.peers.insert(y);
                    r.band_count[b] += 1;
                }
            }
        }
        let m = self.pairs.get_mut(&pk).unwrap();
        if m.updates >= 30 && now - m.updated_wall < PAIR_MIN_INTERVAL {
            return;
        }
        // Orientation: the model is theta = s_b - s_a with a = the lower key.
        let (a, b) = if k == pk.0 { (o, &q) } else { (&q, o) };
        for (sa, sb) in [(a.tda, b.tda), (a.tdb, b.tdb)] {
            match m.observe(sa, sb - sa, now) {
                Outcome::Accepted => self.stats.pair_updates.fetch_add(1, Ordering::Relaxed),
                Outcome::Rejected => self.stats.pair_rejects.fetch_add(1, Ordering::Relaxed),
                Outcome::Reset => self.stats.pair_resets.fetch_add(1, Ordering::Relaxed),
                Outcome::ResetDrift => self.stats.pair_resets_drift.fetch_add(1, Ordering::Relaxed),
            };
        }
    }

    // ---------------------------------------------------------------- mlat --

    fn on_mlat(&mut self, conn: ConnId, t: f64, msg: &[u8], wall: f64) {
        self.stats.mlat_msgs.fetch_add(1, Ordering::Relaxed);
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        let Some(mk) = MsgKey::new(msg) else { return };
        let freq = match self.rxs.get_mut(&k) {
            Some(r) => {
                r.mlat_msgs += 1;
                if r.suspect {
                    return;
                }
                r.freq
            }
            None => return,
        };
        let g = self.groups.entry(mk).or_insert_with(|| {
            self.group_due.push_back((wall + self.cfg.mlat_delay, mk));
            Group { first_wall: wall, copies: Vec::new(), truth: None }
        });
        if g.copies.len() < MAX_COPIES {
            g.copies.push((k, t / freq));
        }
    }

    /// Everything the periodic tick does, at wall time `now` (for tests).
    pub(crate) fn tick_at(&mut self, now: f64, housekeeping: bool) {
        self.process_due(now);
        self.expire(now);
        if housekeeping {
            self.housekeeping(now);
        }
    }

    fn process_due(&mut self, now: f64) {
        while let Some(&(due, mk)) = self.group_due.front() {
            if due > now {
                break;
            }
            self.group_due.pop_front();
            if let Some(g) = self.groups.remove(&mk) {
                self.process_group(mk, g, now);
            }
        }
        while let Some(&(due, mk)) = self.holdout_due.front() {
            if due > now {
                break;
            }
            self.holdout_due.pop_front();
            if let Some(g) = self.holdout_groups.remove(&mk) {
                self.process_holdout(g, now);
            }
        }
    }

    /// Nodes and copies for a group, receivers deduplicated.
    fn build(&self, g: &Group) -> (Vec<RxKey>, Vec<Node>, Vec<Copy>) {
        let mut keys: Vec<RxKey> = Vec::new();
        let mut nodes = Vec::new();
        let mut copies = Vec::new();
        for &(k, s) in &g.copies {
            let Some(r) = self.rxs.get(&k) else { continue };
            if r.suspect {
                continue;
            }
            let idx = match keys.iter().position(|&x| x == k) {
                Some(i) => i,
                None => {
                    keys.push(k);
                    nodes.push(Node { pos: r.pos, jitter: r.jitter * r.noise_scale });
                    keys.len() - 1
                }
            };
            copies.push(Copy { node: idx, s });
        }
        (keys, nodes, copies)
    }

    fn normalize(&self, keys: &[RxKey], nodes: &[Node], copies: &[Copy], now: f64) -> Option<norm::Normalized> {
        // Representative local time per node: its first copy.
        let mut s_of = vec![f64::NAN; nodes.len()];
        for c in copies {
            if s_of[c.node].is_nan() {
                s_of[c.node] = c.s;
            }
        }
        norm::normalize(nodes, copies, |i, j| {
            let pk = pair_key(keys[i], keys[j]);
            let m = self.pairs.get(&pk)?;
            if !m.usable(now) {
                return None;
            }
            let (theta, var) = if keys[i] == pk.0 {
                m.predict(s_of[i])
            } else {
                // model is s_i - s_j at j's time; flip it
                let (th, v) = m.predict(s_of[j]);
                (-th, v)
            };
            if var.sqrt() > crate::clock::MAX_SIGMA {
                return None;
            }
            Some((theta, var))
        })
    }

    fn altitude_for(&self, ac: &Aircraft, decoded_alt: Option<i32>, now: f64) -> Option<(f64, f64)> {
        if let Some(a) = decoded_alt {
            return Some((a as f64 * FT_TO_M, 76.0));
        }
        let a = ac.alt_ft?;
        let age = now - ac.alt_t;
        if age > 30.0 {
            return None;
        }
        // 250 ft at report time, degrading at ~70 ft/s (mlat-server's model).
        Some((a as f64 * FT_TO_M, (250.0 + 70.0 * age) * FT_TO_M))
    }

    fn process_group(&mut self, mk: MsgKey, g: Group, now: f64) {
        self.stats.groups.fetch_add(1, Ordering::Relaxed);
        let bytes = mk.bytes().to_vec();
        if let Some(ac) = modes::false_ac(&bytes) {
            self.process_modeac(ac, g, now);
            return;
        }
        let Some(d) = modes::decode(&bytes) else { return };
        // The AP address of a corrupted frame is garbage; requiring that some
        // receiver reported the aircraft as seen filters those out.
        let known = self.aircraft.get(&d.address).map(|a| a.tracking > 0).unwrap_or(false);
        if !known || g.copies.len() < 2 {
            self.stats.groups_unknown_ac.fetch_add(1, Ordering::Relaxed);
            return;
        }
        {
            let ac = self.aircraft.get_mut(&d.address).unwrap();
            if let Some(a) = d.altitude_ft {
                if (-1500..=75_000).contains(&a) {
                    ac.alt_ft = Some(a);
                    ac.alt_t = now;
                }
            }
            if d.squawk.is_some() {
                ac.squawk = d.squawk;
            }
            if d.callsign.is_some() {
                ac.callsign = d.callsign.clone();
            }
        }
        let (keys, nodes, copies) = self.build(&g);
        if keys.len() < 2 {
            return;
        }
        let Some(normd) = self.normalize(&keys, &nodes, &copies, now) else {
            self.stats.groups_no_norm.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let mut cls = norm::clusters(&normd, &copies, &nodes, 2);
        cls.sort_by(|a, b| normd.t[a[0]].partial_cmp(&normd.t[b[0]]).unwrap_or(std::cmp::Ordering::Equal));
        let t_first = cls.first().map(|c| normd.t[c[0]]).unwrap_or(0.0);
        let n_clusters = cls.len();
        for cl in cls {
            self.stats.clusters.fetch_add(1, Ordering::Relaxed);
            let t_wall = g.first_wall + (normd.t[cl[0]] - t_first);
            let (meas, cov) = sub_problem(&normd, &cl, &copies, &nodes);
            let ac = self.aircraft.get(&d.address).unwrap();
            let alt = self.altitude_for(ac, d.altitude_ft, now);
            let used_keys: Vec<RxKey> = cl.iter().map(|&i| keys[copies[normd.copies[i]].node]).collect();
            let sites = distinct_sites(&meas);
            let ctx = GroupCtx { df: d.df, clusters: n_clusters, copies: g.copies.len(), edges: normd.edges, dropped: normd.dropped, sites };
            self.update_aircraft(d.address, &meas, &cov, alt, t_wall, now, &used_keys, &ctx);
        }
    }

    fn guesses(&self, ac: Option<&Aircraft>, meas: &[Meas], alt: Option<(f64, f64)>, t: f64) -> Vec<Ecef> {
        let mut g = Vec::with_capacity(3);
        if let Some(ac) = ac {
            if let Some(tr) = &ac.track {
                if t - tr.last_update() < 60.0 {
                    g.push(tr.predict(t).0);
                }
            }
            if let Some((p, ft)) = ac.last_fix {
                if t - ft < 120.0 {
                    g.push(p);
                }
            }
        }
        let n = meas.len() as f64;
        let c = meas.iter().fold([0.0; 3], |s, m| [s[0] + m.rx[0] / n, s[1] + m.rx[1] / n, s[2] + m.rx[2] / n]);
        let mut l = ecef_to_llh(c);
        l.alt = alt.map(|a| a.0).unwrap_or(9000.0);
        g.push(llh_to_ecef(l));
        g
    }

    #[allow(clippy::too_many_arguments)]
    fn update_aircraft(
        &mut self,
        icao: u32,
        meas: &[Meas],
        cov: &DMatrix<f64>,
        alt: Option<(f64, f64)>,
        t: f64,
        now: f64,
        used: &[RxKey],
        ctx: &GroupCtx,
    ) {
        let n = meas.len();
        // Decisions count SITES, not receivers: two antennas a few metres
        // apart (production: ADSB_PRI/ADSB_SEC, 3 m) are one point of
        // geometry. A stand-alone fix needs one redundant site (dof >= 1):
        // an exactly determined fix fits both mirror lobes perfectly, so a
        // wrong one is undetectable (chi2 = 0). Those groups go to the track
        // as a TDOA update instead, where the track's prior picks the lobe.
        let sites_dof = ctx.sites as i32 + alt.is_some() as i32 - 4;
        let can_fix = sites_dof >= 1;
        let guesses = self.guesses(self.aircraft.get(&icao), meas, alt, t);
        let mut accepted = false;
        let mut fix_used: Option<Fix> = None;
        if can_fix {
            self.stats.solves.fetch_add(1, Ordering::Relaxed);
            if let Some(fix) = solver::solve(&SolveInput { meas, cov, alt, guesses: &guesses }) {
                if fix.err_h < 5000.0 {
                    self.stats.fixes.fetch_add(1, Ordering::Relaxed);
                    let ac = self.aircraft.get_mut(&icao).unwrap();
                    ac.last_fix = Some((fix.pos, t));
                    match ac.track.as_mut() {
                        Some(tr) if t - tr.last_update() < 60.0 => match tr.update_fix(&fix, t) {
                            Gate::Accepted => {
                                accepted = true;
                                ac.track_rejects = 0;
                                ac.last_fix_update = t;
                                self.stats.track_fix_updates.fetch_add(1, Ordering::Relaxed);
                            }
                            Gate::Rejected(_) => {
                                ac.track_rejects += 1;
                                self.stats.track_rejects.fetch_add(1, Ordering::Relaxed);
                                if ac.track_rejects >= 4 && fix.dof >= 1 {
                                    // The track lost the aircraft (or was a
                                    // ghost): restart from an over-determined fix.
                                    ac.track = Some(Track::new(&fix, t));
                                    ac.track_rejects = 0;
                                    ac.last_fix_update = t;
                                    self.stats.tracks_born.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                            Gate::Skipped => {}
                        },
                        // A track is born only from an over-determined fix:
                        // with dof 0 (3 receivers + altitude, or 4 without)
                        // both mirror solutions fit exactly, and a track born
                        // on the wrong one was then kept alive for minutes by
                        // 2-receiver TDOA updates (production, 2026-10-24:
                        // 612 of the 713 errors over 5 km).
                        _ if fix.dof >= 1 => {
                            ac.track = Some(Track::new(&fix, t));
                            ac.track_rejects = 0;
                            ac.last_fix_update = t;
                            accepted = true;
                            self.stats.tracks_born.fetch_add(1, Ordering::Relaxed);
                        }
                        _ => {
                            self.stats.tracks_refused_dof0.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    fix_used = Some(fix);
                }
            }
        } else if n >= 2 && ctx.sites >= 2 {
            let ac = self.aircraft.get_mut(&icao).unwrap();
            if let Some(tr) = ac.track.as_mut() {
                if t - tr.last_update() < 30.0 {
                    match tr.update_tdoa(meas, cov, alt, t) {
                        Gate::Accepted => {
                            // 2-site updates feed the track but never publish
                            // it on their own: they constrain a hyperbola, not
                            // a point (production: 85 of the 113 published
                            // errors over 1 km).
                            accepted = ctx.sites >= 3;
                            self.stats.track_tdoa_updates.fetch_add(1, Ordering::Relaxed);
                        }
                        Gate::Rejected(_) => {
                            self.stats.track_rejects.fetch_add(1, Ordering::Relaxed);
                        }
                        Gate::Skipped => {}
                    }
                }
            }
        }
        let published = if accepted {
            for k in used {
                if let Some(r) = self.rxs.get_mut(k) {
                    r.used += 1;
                }
            }
            self.emit(icao, fix_used.as_ref(), n, t, now)
        } else {
            None
        };
        self.check_adsb(icao, n, t, now, fix_used.as_ref(), published, accepted, used, ctx);
    }

    /// Compare this update with the aircraft's own ADS-B position, when it has
    /// one: the live equivalent of the holdout, but for the real MLAT path
    /// (grouping, clustering, tracks, publication), not just the solver.
    #[allow(clippy::too_many_arguments)]
    fn check_adsb(
        &mut self,
        icao: u32,
        n: usize,
        t: f64,
        now: f64,
        fix: Option<&Fix>,
        published: Option<Ecef>,
        accepted: bool,
        used: &[RxKey],
        ctx: &GroupCtx,
    ) {
        let Some(ac) = self.aircraft.get(&icao) else { return };
        let Some((p0, t0)) = ac.adsb else { return };
        let dt = t - t0;
        let truth = match ac.adsb_vel {
            Some(v) if dt.abs() <= 5.0 => [p0[0] + v[0] * dt, p0[1] + v[1] * dt, p0[2] + v[2] * dt],
            _ if dt.abs() <= 1.0 => p0,
            _ => return,
        };
        let tl = ecef_to_llh(truth);
        let herr = |p: &Ecef| {
            let l = ecef_to_llh(*p);
            crate::geo::haversine_m(l.lat, l.lon, tl.lat, tl.lon)
        };
        let fix_err = fix.map(|f| herr(&f.pos));
        let out_err = published.as_ref().map(herr);
        if fix_err.is_none() && out_err.is_none() {
            return;
        }
        self.adsb_err.push_back((now, n, fix_err, out_err));
        let bad = fix_err.unwrap_or(0.0) > 1000.0 || out_err.unwrap_or(0.0) > 1000.0;
        if bad {
            let users = used.iter().filter_map(|k| self.rxs.get(k).map(|r| r.info.user.clone())).collect();
            self.diag_bad.push_back(Diag {
                t,
                icao: format!("{icao:06X}"),
                df: ctx.df,
                n,
                clusters: ctx.clusters,
                copies: ctx.copies,
                edges: ctx.edges,
                dropped: ctx.dropped,
                chi2: fix.map(|f| f.chi2),
                dof: fix.map(|f| f.dof),
                err_h: fix.map(|f| f.err_h),
                gdop: fix.map(|f| f.gdop),
                fix_err_m: fix_err,
                out_err_m: out_err,
                accepted,
                confirmed: ac.track.as_ref().map(|tr| tr.confirmed()).unwrap_or(false),
                users,
            });
            while self.diag_bad.len() > 400 {
                self.diag_bad.pop_front();
            }
        }
    }

    /// Publish the aircraft's position; returns what was published.
    fn emit(&mut self, icao: u32, fix: Option<&Fix>, n: usize, t: f64, now: f64) -> Option<Ecef> {
        let ac = self.aircraft.get_mut(&icao).unwrap();
        if t - ac.last_out < 0.5 {
            return None;
        }
        let tr = ac.track.as_ref()?;
        // Publish from a confirmed track, or a stand-alone fix good enough on
        // its own (over-determined and tight). Everything else waits: one
        // wrong fix on the public map costs more than a second of latency.
        // A confirmed track is published only while a full fix anchors it
        // (one in the last 10 s): TDOA updates from 2-3 receivers constrain a
        // hyperbola, not a point, and alone they let a track slide.
        let anchored = t - ac.last_fix_update <= 10.0;
        let (pos, err, vel) = if tr.confirmed() && anchored {
            (tr.predict(t).0, tr.err_h(), true)
        } else if let Some(f) = fix.filter(|f| f.dof >= 1 && f.err_h < 1000.0) {
            (f.pos, f.err_h, false)
        } else {
            if tr.confirmed() {
                self.stats.outputs_held_no_fix.fetch_add(1, Ordering::Relaxed);
            }
            return None;
        };
        let l = ecef_to_llh(pos);
        let alt_ft = ac.alt_ft.filter(|_| now - ac.alt_t < 10.0);
        let addr = format!("{icao:06X}");
        let line = sbs_line(&SbsFix {
            addr: &addr,
            t_fix: t,
            t_now: now,
            callsign: ac.callsign.as_deref(),
            alt_ft,
            speed_kt: vel.then(|| tr.ground_speed_kt()),
            track_deg: vel.then(|| tr.track_deg()),
            lat: l.lat,
            lon: l.lon,
            vrate_fpm: vel.then(|| tr.vrate_fpm()),
            squawk: ac.squawk,
            receivers: n,
            err_m: Some(err),
        });
        ac.last_out = t;
        self.stats.outputs.fetch_add(1, Ordering::Relaxed);
        let _ = self.out.send(Arc::from(line));
        Some(pos)
    }

    // ------------------------------------------------------------ mode a/c --

    fn process_modeac(&mut self, mac: modes::ModeAc, g: Group, now: f64) {
        self.stats.modeac_groups.fetch_add(1, Ordering::Relaxed);
        if MODEAC_BLACKLIST.contains(&mac.code) {
            self.stats.modeac_blacklisted.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let (keys, nodes, copies) = self.build(&g);
        if keys.len() < 3 {
            return;
        }
        let Some(normd) = self.normalize(&keys, &nodes, &copies, now) else { return };
        let cls = norm::clusters(&normd, &copies, &nodes, 3);
        let t_first = cls.iter().map(|c| normd.t[c[0]]).fold(f64::INFINITY, f64::min);
        for cl in cls {
            let t = g.first_wall + (normd.t[cl[0]] - t_first);
            let (meas, cov) = sub_problem(&normd, &cl, &copies, &nodes);
            // The same reply answers a Mode A or a Mode C interrogation: try
            // the Mode C reading (altitude-constrained, 3+ receivers) and the
            // Mode A reading (free altitude, 4+ receivers).
            let mut candidates: Vec<(Fix, bool)> = Vec::new();
            let centroid = self.guesses(None, &meas, None, t);
            if let Some(ft) = mac.altitude_ft {
                let alt = Some((ft as f64 * FT_TO_M, 100.0));
                let mut gs = centroid.clone();
                gs[0] = {
                    let mut l = ecef_to_llh(gs[0]);
                    l.alt = ft as f64 * FT_TO_M;
                    llh_to_ecef(l)
                };
                if let Some(f) = solver::solve(&SolveInput { meas: &meas, cov: &cov, alt, guesses: &gs }) {
                    candidates.push((f, true));
                }
            }
            if meas.len() >= 4 {
                if let Some(f) = solver::solve(&SolveInput { meas: &meas, cov: &cov, alt: None, guesses: &centroid }) {
                    candidates.push((f, false));
                }
            }
            candidates.retain(|(f, _)| f.err_h < 3000.0);
            if candidates.is_empty() {
                continue;
            }
            self.stats.modeac_fixes.fetch_add(1, Ordering::Relaxed);
            // 1) Association with a Mode S aircraft: same identity (Mode A) or
            //    same altitude (Mode C), within 15 km of its track.
            let mut assoc = None;
            'outer: for (f, mode_c) in &candidates {
                for (icao, ac) in &self.aircraft {
                    let Some(tr) = &ac.track else { continue };
                    if t - tr.last_update() > 30.0 {
                        continue;
                    }
                    let matches = if *mode_c {
                        match (ac.alt_ft, mac.altitude_ft) {
                            (Some(a), Some(b)) => (a - b).abs() <= 300,
                            _ => false,
                        }
                    } else {
                        ac.squawk == Some(mac.squawk)
                    };
                    if matches && dist(&tr.predict(t).0, &f.pos) < 15e3 {
                        assoc = Some((*icao, *mode_c));
                        break 'outer;
                    }
                }
            }
            if let Some((icao, mode_c)) = assoc {
                self.stats.modeac_assoc.fetch_add(1, Ordering::Relaxed);
                let alt = if mode_c { mac.altitude_ft.map(|ft| (ft as f64 * FT_TO_M, 100.0)) } else { None };
                let ac = self.aircraft.get_mut(&icao).unwrap();
                if let Some(tr) = ac.track.as_mut() {
                    if tr.update_tdoa(&meas, &cov, alt, t) == Gate::Accepted {
                        self.stats.track_tdoa_updates.fetch_add(1, Ordering::Relaxed);
                    }
                }
                continue;
            }
            // 2) Mode A/C-only tracks: same code, consistent position.
            let mut taken = false;
            for (id, at) in self.actracks.iter_mut() {
                if at.code != mac.code || t - at.track.last_update() > 60.0 {
                    continue;
                }
                for (f, mode_c) in &candidates {
                    if *mode_c == at.mode_c && at.track.update_fix(f, t) == Gate::Accepted {
                        taken = true;
                        if at.track.confirmed() && t - at.last_out >= 1.0 {
                            at.last_out = t;
                            let l = ecef_to_llh(at.track.position());
                            let addr = format!("~{:06X}", id);
                            let line = sbs_line(&SbsFix {
                                addr: &addr,
                                t_fix: t,
                                t_now: now,
                                callsign: None,
                                alt_ft: mac.altitude_ft.filter(|_| at.mode_c),
                                speed_kt: Some(at.track.ground_speed_kt()),
                                track_deg: Some(at.track.track_deg()),
                                lat: l.lat,
                                lon: l.lon,
                                vrate_fpm: None,
                                squawk: (!at.mode_c).then_some(mac.squawk),
                                receivers: meas.len(),
                                err_m: Some(at.track.err_h()),
                            });
                            self.stats.modeac_outputs.fetch_add(1, Ordering::Relaxed);
                            let _ = self.out.send(Arc::from(line));
                        }
                        break;
                    }
                }
                if taken {
                    break;
                }
            }
            if !taken {
                let (f, mode_c) = &candidates[0];
                self.next_ac_id = (self.next_ac_id + 1) & 0x0F_FFFF;
                let id = 0xF0_0000 | self.next_ac_id;
                self.actracks.insert(id, AcTrack { code: mac.code, mode_c: *mode_c, track: Track::new(f, t), last_out: 0.0 });
            }
        }
    }

    // ------------------------------------------------------------- holdout --

    fn process_holdout(&mut self, g: Group, now: f64) {
        let Some((truth, alt_ft)) = g.truth else { return };
        let (keys, nodes, copies) = self.build(&g);
        if keys.len() < 3 {
            return;
        }
        let Some(normd) = self.normalize(&keys, &nodes, &copies, now) else {
            self.stats.holdout_failed.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let Some(cl) = norm::clusters(&normd, &copies, &nodes, 3).into_iter().max_by_key(|c| c.len()) else {
            self.stats.holdout_failed.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let (meas, cov) = sub_problem(&normd, &cl, &copies, &nodes);
        let alt = Some((alt_ft as f64 * FT_TO_M, 76.0));
        let guesses = self.guesses(None, &meas, alt, now);
        match solver::solve(&SolveInput { meas: &meas, cov: &cov, alt, guesses: &guesses }) {
            Some(f) => {
                self.stats.holdout_solved.fetch_add(1, Ordering::Relaxed);
                let (a, b) = (ecef_to_llh(f.pos), ecef_to_llh(truth));
                let e = crate::geo::haversine_m(a.lat, a.lon, b.lat, b.lon);
                self.holdout_err.push_back((now, e));
            }
            None => {
                self.stats.holdout_failed.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    // --------------------------------------------------------- maintenance --

    fn expire(&mut self, now: f64) {
        while let Some(&(t, key)) = self.sync_expiry.front() {
            if now - t < SYNC_TTL {
                break;
            }
            self.sync_expiry.pop_front();
            self.sync.remove(&key);
        }
        while let Some(&(t, _)) = self.holdout_err.front() {
            if now - t < 600.0 {
                break;
            }
            self.holdout_err.pop_front();
        }
        while let Some(&(t, ..)) = self.adsb_err.front() {
            if now - t < 600.0 {
                break;
            }
            self.adsb_err.pop_front();
        }
    }

    fn housekeeping(&mut self, now: f64) {
        // Pairs not updated for 3x their usable age are dropped.
        let stale: Vec<(RxKey, RxKey)> = self
            .pairs
            .iter()
            .filter(|(_, m)| now - m.updated_wall > 3.0 * crate::clock::MAX_AGE && m.updated_wall > 0.0)
            .map(|(k, _)| *k)
            .collect();
        for pk in stale {
            if let Some(m) = self.pairs.remove(&pk) {
                for (x, y) in [(pk.0, pk.1), (pk.1, pk.0)] {
                    if let Some(r) = self.rxs.get_mut(&x) {
                        r.peers.remove(&y);
                        r.band_count[m.band as usize] = r.band_count[m.band as usize].saturating_sub(1);
                    }
                }
            }
        }
        // Receiver health: a receiver most of whose converged pairs keep
        // rejecting observations has a wrong position or a broken clock; its
        // copies are kept out of multilateration until that changes.
        let mut rate: HashMap<RxKey, (f64, u32, f64)> = HashMap::new();
        for ((a, b), m) in &self.pairs {
            if m.updates < crate::clock::MIN_UPDATES {
                continue;
            }
            for k in [a, b] {
                let e = rate.entry(*k).or_insert((0.0, 0, 0.0));
                e.0 += m.reject_rate;
                e.1 += 1;
                e.2 += m.noise_scale();
            }
        }
        for (k, r) in self.rxs.iter_mut() {
            let (sum, n, noise) = rate.get(k).copied().unwrap_or((0.0, 0, 0.0));
            r.suspect = n >= 4 && sum / n as f64 > 0.35;
            // A pair's noise mixes both receivers; averaged over all of a
            // receiver's pairs it mostly reflects the receiver itself.
            r.noise_scale = if n >= 3 { (noise / n as f64).max(1.0) } else { 1.0 };
        }
        // Tracks.
        self.aircraft.retain(|_, a| {
            if let Some(tr) = &a.track {
                if tr.stale(now) {
                    a.track = None;
                }
            }
            a.tracking > 0 || a.track.is_some()
        });
        self.actracks.retain(|_, t| !t.track.stale(now));
        // Groups that somehow outlived their deadline (clock jumps).
        if self.groups.len() > 200_000 {
            self.groups.clear();
            self.group_due.clear();
        }
    }

    fn publish(&self, queue_depth: usize) {
        let now = unix_now();
        let mut errs: Vec<f64> = self.holdout_err.iter().map(|x| x.1).collect();
        errs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let q = |p: f64| if errs.is_empty() { 0.0 } else { errs[((errs.len() - 1) as f64 * p) as usize] };
        let mut s = self.stats.snapshot.lock().unwrap();
        s.receivers = self.rxs.len();
        s.receivers_suspect = self.rxs.values().filter(|r| r.suspect).count();
        s.pairs = self.pairs.len();
        s.pairs_usable = self.pairs.values().filter(|m| m.usable(now)).count();
        s.sync_entries = self.sync.len();
        s.pending_groups = self.groups.len();
        s.aircraft = self.aircraft.len();
        s.tracks = self.aircraft.values().filter(|a| a.track.is_some()).count();
        s.tracks_confirmed = self.aircraft.values().filter(|a| a.track.as_ref().map(|t| t.confirmed()).unwrap_or(false)).count();
        s.modeac_tracks = self.actracks.len();
        s.queue_depth = queue_depth;
        s.holdout_n = errs.len();
        s.holdout_p50 = q(0.5);
        s.holdout_p90 = q(0.9);
        s.holdout_p99 = q(0.99);
        let qv = |v: &mut Vec<f64>, p: f64| {
            if v.is_empty() {
                return 0.0;
            }
            v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            v[((v.len() - 1) as f64 * p) as usize]
        };
        let mut buckets = Vec::new();
        for (name, lo, hi) in [("2", 2, 2), ("3", 3, 3), ("4-5", 4, 5), ("6-9", 6, 9), ("10+", 10, usize::MAX)] {
            let sel: Vec<&(f64, usize, Option<f64>, Option<f64>)> =
                self.adsb_err.iter().filter(|x| x.1 >= lo && x.1 <= hi).collect();
            let mut f: Vec<f64> = sel.iter().filter_map(|x| x.2).collect();
            let mut o: Vec<f64> = sel.iter().filter_map(|x| x.3).collect();
            buckets.push((name.to_string(), sel.len(), qv(&mut f, 0.5), qv(&mut f, 0.9), qv(&mut o, 0.5), qv(&mut o, 0.9)));
        }
        s.adsb_check = buckets;
        drop(s);
        *self.stats.debug.lock().unwrap() = self.diag_bad.iter().cloned().collect();
    }
}

/// Per-group facts carried into the per-cluster update, for diagnostics.
struct GroupCtx {
    /// distinct receiver sites in this update (receivers < 1 km apart = one)
    sites: usize,
    df: u8,
    clusters: usize,
    copies: usize,
    edges: usize,
    dropped: usize,
}

/// Number of distinct sites among the measurements: receivers closer than
/// 1 km merge into one (mlat-server's `is_distinct` rule).
fn distinct_sites(meas: &[Meas]) -> usize {
    let mut reps: Vec<Ecef> = Vec::with_capacity(meas.len());
    for m in meas {
        if !reps.iter().any(|r| dist(r, &m.rx) < 1000.0) {
            reps.push(m.rx);
        }
    }
    reps.len()
}

/// Measurements and covariance for one cluster of a normalised group.
fn sub_problem(normd: &norm::Normalized, cl: &[usize], copies: &[Copy], nodes: &[Node]) -> (Vec<Meas>, DMatrix<f64>) {
    let meas: Vec<Meas> =
        cl.iter().map(|&i| Meas { rx: nodes[copies[normd.copies[i]].node].pos, t: normd.t[i] }).collect();
    let mut cov = DMatrix::<f64>::zeros(cl.len(), cl.len());
    for (a, &i) in cl.iter().enumerate() {
        for (b, &j) in cl.iter().enumerate() {
            cov[(a, b)] = normd.cov[(i, j)];
        }
    }
    (meas, cov)
}

/// FNV-1a: a stable hash for the holdout selection (same choice on restart).
fn fxhash(b: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &x in b {
        h ^= x as u64;
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}
