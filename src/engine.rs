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
//!
//! Altitudes are geometric (2026-10-25): every Mode S altitude is barometric,
//! and the ADS-B sync references and the solver's altitude constraint used to
//! take it as height above the ellipsoid, ~290 m off at cruise. Both now add
//! the geometric-minus-baro difference: a sync reference its own (ADS-B TC 19)
//! when recent, else the regional model's (altmodel.rs); a multilaterated
//! aircraft the regional model's from OTHER aircraft only (a Mode-S-only
//! aircraft has no TC 19, so the bench must not use its own), with a wider
//! sigma and no correction where the model has no data.
//!
//! Normalisation may route through up to `MAX_BRIDGES` receivers outside the
//! group when the group's own clock graph is disconnected (norm.rs explains
//! why only then).

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use nalgebra::{DMatrix, Matrix3, Vector3};
use tokio::sync::{broadcast, mpsc, oneshot};

use crate::altmodel::{self, AltModel};
use crate::clock::{band_of, receiver_noise_scale, Outcome, PairModel, PAIR_BUDGET};
use crate::coverage::{CovId, Coverage};
use crate::geo::{dist, ecef_to_llh, llh_to_ecef, Ecef, Enu, Llh, C_AIR, FT_TO_M};
use crate::interest::{self, AcMeta, AcState, Interest};
use crate::modes;
use crate::norm::{self, Copy, Node};
use crate::output::{result_ecef, result_old, sbs_line, ClientResult, SbsFix};
use crate::solver::{self, Fix, Meas, SolveInput};
use crate::survey::{self, Survey};
use crate::track::{Gate, Track};
use crate::twosite::{self, Rec2};
use crate::types::{ConnId, Event, ReceiverInfo, ResultFormat, ToClient};

/// Aircraft-to-receiver range beyond which a sync point is not used (radio
/// horizon at FL450 is ~480 km).
const MAX_RANGE: f64 = 500e3;
/// Coverage maps are saved this often (seconds) when a file is configured.
const COVERAGE_SAVE_EVERY: f64 = 1800.0;
/// Maximum distance between the even and odd positions of a sync pair.
const MAX_PAIR_SPREAD: f64 = 10e3;
const SYNC_TTL: f64 = 10.0;
/// A converged pair is not updated more often than this: beyond it more
/// observations add CPU, not accuracy.
const PAIR_MIN_INTERVAL: f64 = 0.25;
/// Cap on copies kept per message group.
const MAX_COPIES: usize = 40;
/// Receivers outside a disconnected group that its normalisation may route
/// through (norm.rs, "Bridges"). 2 and 4 performed the same in replays.
const MAX_BRIDGES: usize = 2;
/// 1-sigma (m) of a decoded Mode S altitude as a constraint, before the
/// barometric -> geometric correction's own uncertainty.
const ALT_SIGMA_M: f64 = 76.0;
/// An aircraft's own geometric-minus-baro difference is used for its sync
/// references up to this age (s), and enters the model up to `GEO_SAMPLE_AGE`.
const GEO_OWN_AGE: f64 = 60.0;
const GEO_SAMPLE_AGE: f64 = 30.0;
/// Mode A/C payloads that are shared by too many aircraft at once (VFR,
/// conspicuity, defaults) or are line noise, as acbridge.py/mlat-client.
/// Seconds of clock-dated ADS-B positions kept per aircraft (adsb_hist).
const ADSB_HIST_S: f64 = 30.0;
/// WallMap: bucket length (s; the minimum covers one to two buckets) and the
/// deliveries needed before it maps.
const WALLMAP_BUCKET: f64 = 30.0;
const WALLMAP_MIN_OBS: u32 = 20;
/// Serve mode: `{"stats": ...}` to clients that asked, this often (s); the
/// sync-point share of an aircraft is judged over this many observations.
const SERVE_STATS_EVERY: f64 = 15.0;
const SYNC_QUALITY_MIN_OBS: f32 = 20.0;
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
    /// Stable identity for its coverage map (survives reconnects).
    cov: CovId,
    /// Local east/north/up frame at the declared position (survey).
    enu: Enu,
    /// Its clock to wall time (2-site records, ADS-B truth).
    wall: WallMap,
    /// Serve mode: its last rate report (None = it never sent one), what it
    /// is currently asked for, and how that splits into sync and mlat.
    rate: Option<HashMap<u32, f32>>,
    requested: HashSet<u32>,
    want_sync: usize,
    want_mlat: usize,
}

/// A receiver's clock mapped to wall time through its quickest deliveries:
/// arrival minus clock time, as low as seen in the last 30-60 s. Batching
/// (mlat-client flushes its output every second), the shadow hub and the
/// network only ever add delay, so the minimum is the clock's offset plus the
/// fastest path's latency, and `clock + offset` dates a transmission without
/// the seconds of jitter its own arrival carries. In the hybrid one receiver
/// arrives by two paths (mlat-client via the hub, the Beast feed) with
/// different delays; dating its ADS-B positions by arrival mixed them, and
/// the 2-site self-check measured that (1.7 km p50 live while readsb's truth
/// put the same positions at 61 m, 2026-10-26).
#[derive(Clone, Copy)]
struct WallMap {
    start: f64,
    cur: f64,
    prev: f64,
    n: u32,
}

impl Default for WallMap {
    fn default() -> Self {
        WallMap { start: f64::NEG_INFINITY, cur: f64::INFINITY, prev: f64::INFINITY, n: 0 }
    }
}

impl WallMap {
    fn observe(&mut self, now: f64, clock_s: f64) {
        if now - self.start > WALLMAP_BUCKET {
            self.prev = self.cur;
            self.cur = f64::INFINITY;
            self.start = now;
        }
        self.cur = self.cur.min(now - clock_s);
        self.n = self.n.saturating_add(1);
    }

    /// Wall time of clock time `clock_s`, once enough deliveries were seen
    /// and when it is plausible against the arrival `now` (a clock that
    /// jumped is not mapped).
    fn map(&self, clock_s: f64, now: f64) -> Option<f64> {
        let off = self.cur.min(self.prev);
        if self.n < WALLMAP_MIN_OBS || !off.is_finite() {
            return None;
        }
        let t = clock_s + off;
        (t <= now + 0.5 && now - t <= 10.0).then_some(t)
    }
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
    /// Ground truth for holdout groups: ADS-B position, altitude (ft), address.
    truth: Option<(Ecef, i32, u32)>,
}

#[derive(Default)]
struct Aircraft {
    tracking: u32,
    /// The receivers counted in `tracking` (serve mode's rate pairs).
    seen_by: HashSet<RxKey>,
    /// Sync observations of this aircraft accepted / rejected by clock
    /// pairs, decayed: a share of rejects marks bad ADS-B positions.
    sync_ok: f32,
    sync_bad: f32,
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
    /// The last `ADSB_HIST_S` seconds of its ADS-B positions, dated by the
    /// receiving receivers' clocks (WallMap): the 2-site check interpolates
    /// in them. A velocity from consecutive positions is no good for that:
    /// they come from different receivers whose datings differ by up to a
    /// second, and live it put the "truth" ~2.4 km off (2026-10-26).
    adsb_hist: VecDeque<(Ecef, f64)>,
    /// Time of the last Comm-B ground velocity accepted into the track.
    last_vel_update: f64,
    /// Own geometric-minus-baro altitude (ft, from ADS-B TC 19) and its time.
    geo_baro: Option<(f64, f64)>,
    /// Last unambiguous Comm-B BDS 5,0 ground velocity (east, north m/s) and
    /// its time, kept also without a track: the 2-site path needs it.
    commb_vel: Option<([f64; 2], f64)>,
    /// The same over the last TWO_SITE_WINDOW + 60 s, oldest first, to follow
    /// turns across the 2-site window.
    commb_hist: VecDeque<([f64; 2], f64)>,
    /// 2-site TDOAs of the last TWO_SITE_WINDOW seconds, the coverage ids of
    /// the receivers in them, and when a solve was last tried.
    two: VecDeque<Rec2>,
    two_heard: Vec<CovId>,
    two_tried: f64,
    /// Replay bench: the one pair of receivers this aircraft keeps.
    bench_pair: Option<(RxKey, RxKey)>,
}

/// What the engine does with aircraft that only two sites hear.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TwoSite {
    /// TDOA updates of existing tracks only (as before).
    Off,
    /// Solve them from motion + coverage, but only aircraft that also send
    /// ADS-B get the result (to the bench output, where it is judged).
    Bench,
    /// Solve them and publish what passes the checks.
    Publish,
}

impl std::str::FromStr for TwoSite {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        match s {
            "off" => Ok(TwoSite::Off),
            "bench" => Ok(TwoSite::Bench),
            "publish" => Ok(TwoSite::Publish),
            _ => Err(format!("two-site mode must be off|bench|publish, not {s}")),
        }
    }
}

/// 2-site path: window of TDOAs, minimum records and span, solve interval,
/// age of the Comm-B velocity, and the checks a solution must pass.
const TWO_SITE_WINDOW: f64 = 240.0;
const TWO_SITE_MIN_RECS: usize = 8;
const TWO_SITE_MIN_SPAN: f64 = 60.0;
const TWO_SITE_MAX_RECS: usize = 48;
const TWO_SITE_EVERY: f64 = 10.0;
const TWO_SITE_VEL_AGE: f64 = 30.0;
// Chosen on the 2026-10-24 capture with ADS-B aircraft cut to two sites
// (replay --bench-two-sites, 4,277 solutions): these keep 202, p50 340 m,
// p90 0.8 km, none over 5 km; without them p50 1.8 km, p90 70 km, 32% over
// 5 km. A low own error estimate is the strongest single sign of the right
// basin. One 10-minute capture: to be confirmed live in Bench mode.
const TWO_SITE_MAX_RMS: f64 = 1.3;
const TWO_SITE_MAX_ERR: f64 = 150.0;
const TWO_SITE_MIN_MARGIN: f64 = 8.0;
/// Silent receivers count as this many independent misses (offline study).
const TWO_SITE_N_EFF: f64 = 8.0;

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
    /// Comm-B BDS 5,0 ground velocities offered to tracks / accepted.
    pub commb_vel_offered: AtomicU64,
    pub commb_vel_accepted: AtomicU64,
    /// 2-site path: solves tried, solutions found, solutions that passed the
    /// checks, and of those how many fed a track.
    pub two_site_tries: AtomicU64,
    pub two_site_solved: AtomicU64,
    pub two_site_accepted: AtomicU64,
    pub two_site_tracked: AtomicU64,
    /// Engine-thread microseconds spent in 2-site solves.
    pub two_site_us: AtomicU64,
    pub track_rejects: AtomicU64,
    pub outputs: AtomicU64,
    /// Positions of aircraft that also broadcast ADS-B, sent to the bench
    /// output instead (feed mode only).
    pub bench_outputs: AtomicU64,
    /// unix ms of the last published position (liveness).
    pub last_output_ms: AtomicU64,
    pub modeac_groups: AtomicU64,
    pub modeac_blacklisted: AtomicU64,
    pub modeac_fixes: AtomicU64,
    pub modeac_assoc: AtomicU64,
    pub modeac_outputs: AtomicU64,
    pub holdout_solved: AtomicU64,
    pub holdout_failed: AtomicU64,
    /// Altitude constraints corrected to geometric by the regional model, and
    /// those left barometric (no model data nearby; wider sigma).
    pub alt_geo_corrected: AtomicU64,
    pub alt_geo_fallback: AtomicU64,
    /// Sync reference pairs placed at geometric height (own TC 19 or model).
    pub sync_refs_geometric: AtomicU64,
    /// Groups normalised through bridge receivers, bridges used, and copies
    /// those groups kept beyond their largest own component.
    pub groups_bridged: AtomicU64,
    pub bridge_receivers: AtomicU64,
    pub bridged_copies: AtomicU64,
    pub loop_late_ms_max: AtomicU64,
    /// Serve mode: start_sending / stop_sending lines, result and stats lines
    /// sent, and engine lines dropped on a full connection queue.
    pub serve_start_lines: AtomicU64,
    pub serve_stop_lines: AtomicU64,
    pub serve_results: AtomicU64,
    pub serve_stats_lines: AtomicU64,
    pub serve_send_full: AtomicU64,
    /// Aircraft asked for the moment they became wanted (beyond mlat-server).
    pub serve_fast_starts: AtomicU64,
    /// Positions kept off the output because no served receiver contributed
    /// (served-output-only).
    pub outputs_not_served: AtomicU64,
    /// Snapshot for /state.json and the gauges, rewritten every second.
    pub snapshot: Mutex<Snapshot>,
    /// Worst recent ADS-B-checked updates, for /debug.json.
    pub debug: Mutex<Vec<Diag>>,
    /// `/receivers.json`: mlat-client receivers with a UUID and their
    /// positions, for the feed instance (cluster-internal only).
    pub registry_json: Mutex<String>,
    /// `/survey.json`: each receiver's position correction estimate
    /// (cluster-internal: identities, corrections relative to declared
    /// positions).
    pub survey_json: Mutex<String>,
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
    /// 2-site solutions of aircraft with ADS-B in the last 10 minutes:
    /// (count, p50 m, p90 m) for all solutions and for the accepted ones.
    pub two_site_all: (usize, f64, f64),
    pub two_site_ok: (usize, f64, f64),
    pub two_site_pub: (usize, f64, f64),
    /// MLAT vs ADS-B for aircraft that broadcast both, last 10 minutes, by
    /// receivers per update: (bucket, count, fix p50, fix p90, out p50, out p90).
    pub adsb_check: Vec<(String, usize, f64, f64, f64, f64)>,
    pub started: f64,
    /// Receivers with a coverage map, and ADS-B samples counted into them.
    pub coverage_maps: usize,
    pub coverage_samples: u64,
    /// Altitude model: 1-degree cells with data, and aircraft samples held.
    pub alt_model_cells: usize,
    pub alt_model_samples: usize,
    /// Survey: receivers with an estimate, reliable ones, and reliable ones
    /// more than 100 m from their declared position.
    pub survey_receivers: usize,
    pub survey_reliable: usize,
    pub survey_reliable_over_100m: usize,
    /// Serve mode: served receivers, the aircraft they are asked for in
    /// total (sync / mlat), and the mlat-wanted aircraft.
    pub serve_receivers: usize,
    pub serve_requested_sync: usize,
    pub serve_requested_mlat: usize,
    pub serve_mlat_wanted: usize,
}

pub struct Engine {
    cfg: EngineConfig,
    stats: Arc<Stats>,
    out: broadcast::Sender<Arc<str>>,
    /// Feed mode: where positions of aircraft that broadcast ADS-B go (the
    /// bench), so the main output carries only aircraft without ADS-B.
    bench_out: Option<broadcast::Sender<Arc<str>>>,
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
    coverage: Coverage,
    rx_by_cov: HashMap<CovId, RxKey>,
    /// Where coverage maps are saved (and loaded from at start), if anywhere.
    coverage_file: Option<String>,
    coverage_saved: f64,
    /// Coverage questions from the HTTP side, answered on this thread.
    queries: Option<mpsc::Receiver<CovQuery>>,
    two_site: TwoSite,
    /// Replay bench: aircraft with ADS-B keep one pair of receivers only.
    bench_two_sites: bool,
    /// (time, error m vs own ADS-B, accepted) of 2-site solutions.
    two_site_err: VecDeque<(f64, f64, bool)>,
    /// (time, error m vs own ADS-B) of positions published after a 2-site fix.
    two_site_pub: VecDeque<(f64, f64)>,
    /// Replay: every 2-site solution of an aircraft with ADS-B, as JSON lines.
    two_site_dump: Option<std::io::BufWriter<std::fs::File>>,
    /// Regional geometric-minus-baro altitude model (altmodel.rs).
    alt_model: AltModel,
    /// Receiver position calibration from sync residuals (survey.rs), keyed
    /// by coverage identity so it survives reconnects.
    survey: Survey,
    /// Serve mode: traffic selection, and when stats last went out.
    interest: Interest,
    serve_stats_at: f64,
    /// Served receivers connected (no serve work at all while 0).
    served_n: usize,
    /// Publish only positions some served receiver contributed to: the
    /// output then covers what a production shard for the served region
    /// would, while the shadow input still helps every fix.
    served_output_only: bool,
}

/// A receiver's coverage identity: its UUID key (as the Beast feed keys it)
/// when it has one, else a hash of its user name.
fn cov_id(info: &ReceiverInfo) -> CovId {
    cov_id_of(info.uuid.as_deref(), &info.user)
}

pub fn cov_id_of(uuid: Option<&str>, user: &str) -> CovId {
    uuid.and_then(crate::relay::registry::key_of_uuid).unwrap_or_else(|| fxhash(user.as_bytes()) | (1 << 63))
}

/// The same identity as the HTTP side names it: "uuid:<16 hex>" or
/// "user:<name>".
fn who_of(info: &ReceiverInfo) -> String {
    match info.uuid.as_deref().and_then(crate::relay::registry::key_of_uuid) {
        Some(k) => format!("uuid:{k:016x}"),
        None => format!("user:{}", info.user),
    }
}

/// A question about coverage maps from the HTTP side; the answer is the
/// JSON body, or None for "no such map".
pub struct CovQuery {
    pub what: CovWhat,
    pub reply: oneshot::Sender<Option<String>>,
}

pub enum CovWhat {
    /// One receiver's map, by its coverage identity.
    Map(CovId),
    /// Every named map's daily facts.
    Summary,
}

async fn recv_opt<T>(rx: &mut Option<mpsc::Receiver<T>>) -> Option<T> {
    match rx {
        Some(r) => r.recv().await,
        None => std::future::pending().await,
    }
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
            bench_out: None,
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
            coverage: Coverage::default(),
            rx_by_cov: HashMap::new(),
            coverage_file: None,
            coverage_saved: 0.0,
            queries: None,
            two_site: TwoSite::Off,
            bench_two_sites: false,
            two_site_err: VecDeque::new(),
            two_site_pub: VecDeque::new(),
            two_site_dump: None,
            alt_model: AltModel::default(),
            survey: Survey::default(),
            interest: Interest::new(interest::Params::default(), 0x5e7e),
            serve_stats_at: 0.0,
            served_n: 0,
            served_output_only: false,
        }
    }

    /// Start from the coverage maps in `path`, without saving there (replay).
    pub fn load_coverage(mut self, path: &str) -> anyhow::Result<Engine> {
        self.coverage = Coverage::load(path)?;
        Ok(self)
    }

    /// Write every 2-site solution of an aircraft with ADS-B to `path`.
    pub fn with_two_site_dump(mut self, path: &str) -> std::io::Result<Engine> {
        self.two_site_dump = Some(std::io::BufWriter::new(std::fs::File::create(path)?));
        Ok(self)
    }

    pub fn flush_two_site_dump(&mut self) {
        if let Some(w) = self.two_site_dump.as_mut() {
            let _ = std::io::Write::flush(w);
        }
    }

    /// (time, error m vs own ADS-B, accepted) of the 2-site solutions so far.
    pub fn two_site_errors(&self) -> Vec<(f64, f64, bool)> {
        self.two_site_err.iter().copied().collect()
    }

    /// (time, error m vs own ADS-B) of the positions published after them.
    pub fn two_site_published(&self) -> Vec<(f64, f64)> {
        self.two_site_pub.iter().copied().collect()
    }

    /// Save the coverage maps now (replay calls this at the end).
    pub fn save_coverage(&self, path: &str) -> anyhow::Result<()> {
        self.coverage.save(path)
    }

    /// Keep receiver coverage maps in `path` (gzip'd JSON): loaded now if it
    /// exists, saved every `COVERAGE_SAVE_EVERY` seconds.
    pub fn with_coverage_file(mut self, path: &str) -> Engine {
        match Coverage::load(path) {
            Ok(c) => {
                tracing::info!(path, maps = c.receivers(), "coverage maps loaded");
                self.coverage = c;
            }
            Err(e) => tracing::info!(path, error = %e, "no coverage maps loaded: starting empty"),
        }
        self.coverage_file = Some(path.to_string());
        self
    }

    pub fn with_served_output_only(mut self, on: bool) -> Engine {
        self.served_output_only = on;
        self
    }

    /// Serve mode's selection parameters (interest.rs).
    pub fn with_serve_params(mut self, p: interest::Params) -> Engine {
        self.interest = Interest::new(p, 0x5e7e);
        self
    }

    pub fn with_two_site(mut self, mode: TwoSite) -> Engine {
        self.two_site = mode;
        self
    }

    /// Replay bench of the 2-site path: aircraft with ADS-B keep the copies
    /// of one pair of receivers only, so the bench scores 2-site positions.
    pub fn with_bench_two_sites(mut self, on: bool) -> Engine {
        self.bench_two_sites = on;
        self
    }

    /// Answer coverage questions arriving on `rx` (see `CovQuery`).
    pub fn with_queries(mut self, rx: mpsc::Receiver<CovQuery>) -> Engine {
        self.queries = Some(rx);
        self
    }

    fn answer(&self, q: CovQuery) {
        let now = unix_now();
        let body = match q.what {
            CovWhat::Map(id) => self.coverage.map_json(id, now).map(|v| v.to_string()),
            CovWhat::Summary => Some(self.coverage.summary_json(now).to_string()),
        };
        let _ = q.reply.send(body);
    }

    /// Send positions of aircraft that also broadcast ADS-B to `bench`.
    pub fn with_bench_output(mut self, bench: broadcast::Sender<Arc<str>>) -> Engine {
        self.bench_out = Some(bench);
        self
    }

    /// Run until the event channel closes. Meant for a dedicated thread with
    /// its own current-thread runtime, away from the network workers.
    pub async fn run(mut self, mut rx: mpsc::Receiver<Event>) {
        let mut tick = tokio::time::interval(Duration::from_millis(50));
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        let mut last_tick = Instant::now();
        let mut last_second = Instant::now();
        let mut last_ten = Instant::now();
        let mut queries = self.queries.take();
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
                        self.serve_refresh(now);
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
                Some(q) = recv_opt(&mut queries) => self.answer(q),
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
            Event::Seen { conn, icao } => self.on_seen(conn, &icao, now),
            Event::Lost { conn, icao } => self.on_lost(conn, &icao, now),
            Event::RateReport { conn, rates } => self.on_rate_report(conn, rates, now),
            Event::GeoBaro { icao, delta_ft } => {
                self.aircraft.entry(icao).or_default().geo_baro = Some((delta_ft as f64, now));
            }
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
                        r.enu = Enu::at(r.pos);
                        self.coverage.place(r.cov, lat, lon, now);
                        self.survey.forget(r.cov);
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
            cov: 0,
            enu: Enu::at(pos),
            wall: WallMap::default(),
            rate: None,
            requested: HashSet::new(),
            want_sync: 0,
            want_mlat: 0,
        };
        let cov = cov_id(&r.info);
        self.coverage.place(cov, r.info.lat, r.info.lon, now);
        self.coverage.name(cov, &who_of(&r.info));
        self.rx_by_cov.insert(cov, key);
        let mut r = r;
        r.cov = cov;
        self.rx_by_conn.insert(conn, key);
        if r.info.serve.is_some() {
            self.served_n += 1;
        }
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
            if r.info.serve.is_some() {
                self.served_n -= 1;
            }
            if self.rx_by_cov.get(&r.cov) == Some(&k) {
                self.rx_by_cov.remove(&r.cov);
                self.coverage.unplace(r.cov);
            }
            for icao in r.seen {
                if let Some(a) = self.aircraft.get_mut(&icao) {
                    a.tracking = a.tracking.saturating_sub(1);
                    a.seen_by.remove(&k);
                }
            }
        }
    }

    fn clock_reset(&mut self, conn: ConnId) {
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        if let Some(r) = self.rxs.get_mut(&k) {
            r.wall = WallMap::default();
        }
        let peers: Vec<RxKey> = self.rxs.get(&k).map(|r| r.peers.iter().copied().collect()).unwrap_or_default();
        for p in peers {
            if let Some(m) = self.pairs.get_mut(&pair_key(k, p)) {
                m.reset_offset();
            }
        }
    }

    fn on_seen(&mut self, conn: ConnId, icao: &[u32], now: f64) {
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        let Some(r) = self.rxs.get_mut(&k) else { return };
        let mut fresh = Vec::new();
        for &a in icao {
            if r.seen.insert(a) {
                let ac = self.aircraft.entry(a).or_default();
                ac.tracking += 1;
                ac.seen_by.insert(k);
                fresh.push(a);
            }
        }
        let (served, legacy) = (r.info.serve.is_some(), r.rate.is_none());
        // A client without rate reports is re-steered on what it sees
        // (coordinator.py receiver_tracking_add).
        if served && legacy {
            self.update_interest(k, now);
            return;
        }
        if self.served_n == 0 || fresh.is_empty() {
            return;
        }
        // Beyond mlat-server: a second receiver seeing an aircraft can make it
        // wanted, and a receiver newly seeing a wanted aircraft is asked for
        // it now, not at its next rate report (~18 s): aircraft crossing the
        // edge of coverage get multilaterated before they are gone.
        self.serve_refresh(now);
        if served {
            let mine: Vec<u32> = fresh.into_iter().filter(|a| self.interest.wanted().contains(a)).collect();
            self.push_mlat(&mine);
        }
    }

    /// Serve mode: recompute the mlat-wanted set (rate-limited) and ask the
    /// served receivers that see a newly wanted aircraft for it at once.
    fn serve_refresh(&mut self, now: f64) {
        if self.served_n == 0 {
            return;
        }
        let newly = self.interest.refresh_wanted(
            now,
            self.aircraft.iter().filter(|(_, a)| a.tracking > 0).map(|(&icao, a)| AcState {
                icao,
                tracking: a.tracking as usize,
                last_adsb: a.last_adsb,
                sync_bad: sync_bad_share(a),
            }),
        );
        if !newly.is_empty() {
            self.push_mlat(&newly);
        }
    }

    /// Ask every served receiver that sees one of `icaos` (and is not
    /// distrusted, nor already asked) for it.
    fn push_mlat(&mut self, icaos: &[u32]) {
        let mut by_rx: HashMap<RxKey, Vec<u32>> = HashMap::new();
        for &a in icaos {
            let Some(ac) = self.aircraft.get(&a) else { continue };
            for &k in &ac.seen_by {
                if let Some(r) = self.rxs.get(&k) {
                    if r.info.serve.is_some() && !r.suspect && !r.requested.contains(&a) {
                        by_rx.entry(k).or_default().push(a);
                    }
                }
            }
        }
        for (k, mut list) in by_rx {
            let Some(r) = self.rxs.get_mut(&k) else { continue };
            list.sort_unstable();
            if r.tx.try_send(ToClient::StartSending(list.clone())).is_ok() {
                self.stats.serve_start_lines.fetch_add(1, Ordering::Relaxed);
                self.stats.serve_fast_starts.fetch_add(list.len() as u64, Ordering::Relaxed);
                r.want_mlat += list.len();
                r.requested.extend(list);
            } else {
                self.stats.serve_send_full.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    fn on_lost(&mut self, conn: ConnId, icao: &[u32], now: f64) {
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        let Some(r) = self.rxs.get_mut(&k) else { return };
        for &a in icao {
            if r.seen.remove(&a) {
                if let Some(ac) = self.aircraft.get_mut(&a) {
                    ac.tracking = ac.tracking.saturating_sub(1);
                    ac.seen_by.remove(&k);
                }
            }
        }
        if r.info.serve.is_some() && r.rate.is_none() {
            self.update_interest(k, now);
        }
    }

    /// Every receiver's rate report is kept (a served receiver's peers may be
    /// shadow receivers: their rates still score the pairs); a served
    /// receiver is re-steered on its own (coordinator.py
    /// receiver_rate_report).
    fn on_rate_report(&mut self, conn: ConnId, rates: Vec<(u32, f32)>, now: f64) {
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        let Some(r) = self.rxs.get_mut(&k) else { return };
        r.rate = Some(rates.into_iter().collect());
        if r.info.serve.is_some() {
            self.update_interest(k, now);
        }
    }

    /// Serve mode: choose what receiver `k` is asked for and send the change
    /// (interest.rs).
    fn update_interest(&mut self, k: RxKey, now: f64) {
        self.serve_refresh(now);
        let (interest, aircraft, rxs) = (&mut self.interest, &self.aircraft, &self.rxs);
        let Some(r) = rxs.get(&k) else { return };
        let peers = |icao: u32| -> Vec<(interest::RxKey, f32)> {
            let Some(a) = aircraft.get(&icao) else { return Vec::new() };
            a.seen_by
                .iter()
                .filter(|&&p| p != k)
                .filter_map(|p| rxs.get(p))
                .map(|p| (p.key, p.rate.as_ref().map_or(0.8, |m| m.get(&icao).copied().unwrap_or(0.0))))
                .collect()
        };
        let dont_use = interest.p.sync_bad_dont_use;
        let meta = |icao: u32| -> Option<AcMeta> {
            let a = aircraft.get(&icao)?;
            Some(AcMeta { alt_ft: a.alt_ft, sync_dont_use: sync_bad_share(a) > dont_use })
        };
        let (sync, mlat) = interest.select(r.suspect, r.rate.as_ref(), &r.seen, peers, meta);
        let wanted: HashSet<u32> = sync.union(&mlat).copied().collect();
        let (start, stop) = interest::diff(&r.requested, &wanted);
        let tx = r.tx.clone();
        let (n_sync, n_mlat) = (sync.len(), mlat.len());
        let mut requested = r.requested.clone();
        if !start.is_empty() {
            if tx.try_send(ToClient::StartSending(start.clone())).is_ok() {
                self.stats.serve_start_lines.fetch_add(1, Ordering::Relaxed);
                requested.extend(start);
            } else {
                self.stats.serve_send_full.fetch_add(1, Ordering::Relaxed);
            }
        }
        if !stop.is_empty() {
            if tx.try_send(ToClient::StopSending(stop.clone())).is_ok() {
                self.stats.serve_stop_lines.fetch_add(1, Ordering::Relaxed);
                for a in stop {
                    requested.remove(&a);
                }
            } else {
                self.stats.serve_send_full.fetch_add(1, Ordering::Relaxed);
            }
        }
        if let Some(r) = self.rxs.get_mut(&k) {
            r.requested = requested;
            r.want_sync = n_sync;
            r.want_mlat = n_mlat;
        }
    }

    /// Serve mode: `{"stats": ...}` to served clients that asked, like
    /// mlat-server's every_15: sync peers, a bad-sync timeout while the
    /// receiver's clock is distrusted, and its pairs' reject share.
    fn send_serve_stats(&mut self, now: f64) {
        if now - self.serve_stats_at < SERVE_STATS_EVERY {
            return;
        }
        self.serve_stats_at = now;
        let mut reject: HashMap<RxKey, (f64, u32)> = HashMap::new();
        for ((a, b), m) in &self.pairs {
            if m.updates < crate::clock::MIN_UPDATES {
                continue;
            }
            for k in [a, b] {
                let e = reject.entry(*k).or_insert((0.0, 0));
                e.0 += m.reject_rate;
                e.1 += 1;
            }
        }
        for r in self.rxs.values() {
            if !r.info.serve.is_some_and(|o| o.stats) {
                continue;
            }
            let (sum, n) = reject.get(&r.key).copied().unwrap_or((0.0, 0));
            let outlier = if n > 0 { (1000.0 * sum / n as f64).round() / 10.0 } else { 0.0 };
            let line = format!(
                "{{\"stats\":{{\"peer_count\":{},\"bad_sync_timeout\":{},\"outlier_percent\":{}}}}}",
                r.peers.len(),
                if r.suspect { 150 } else { 0 },
                outlier
            );
            if r.tx.try_send(ToClient::StatsLine(line)).is_ok() {
                self.stats.serve_stats_lines.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    // ---------------------------------------------------------------- sync --

    /// Geometric-minus-baro difference (m) for a sync reference: the
    /// aircraft's own when recent, else the regional model's, else 0.
    fn sync_geo_m(&self, icao: u32, lat: f64, lon: f64, baro_ft: i32, now: f64) -> Option<f64> {
        let own = self.aircraft.get(&icao).and_then(|a| a.geo_baro).filter(|&(_, t)| now - t <= GEO_OWN_AGE).map(|(d, _)| d);
        own.or_else(|| self.alt_model.estimate(lat, lon, baro_ft as f64, now, None)).map(|d| d * FT_TO_M)
    }

    /// A barometric altitude as a constraint on height above the ellipsoid:
    /// (height m, 1-sigma m). Corrected to geometric with the regional model
    /// built from aircraft other than `icao`; without model data nearby it
    /// stays barometric with the spread of the difference added to sigma.
    fn geo_constraint(&self, icao: Option<u32>, baro_ft: i32, sigma: f64, at: Llh, now: f64) -> (f64, f64) {
        let h = baro_ft as f64 * FT_TO_M;
        match self.alt_model.estimate(at.lat, at.lon, baro_ft as f64, now, icao) {
            Some(d) => {
                self.stats.alt_geo_corrected.fetch_add(1, Ordering::Relaxed);
                (h + d * FT_TO_M, sigma.hypot(altmodel::MODEL_SIGMA_M))
            }
            None => {
                self.stats.alt_geo_fallback.fetch_add(1, Ordering::Relaxed);
                (h, sigma.hypot(altmodel::fallback_sigma(h)))
            }
        }
    }

    fn validate_sync(&self, ma: &[u8], mb: &[u8], now: f64) -> Option<(Ecef, Ecef, u32, i32)> {
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
        // At geometric height: the clock offsets learnt from these positions
        // are only as good as their geometry.
        let ge = self.sync_geo_m(a.address, elat, elon, alt_e, now);
        let go = self.sync_geo_m(a.address, olat, olon, alt_o, now);
        if ge.is_some() || go.is_some() {
            self.stats.sync_refs_geometric.fetch_add(1, Ordering::Relaxed);
        }
        let pe = llh_to_ecef(Llh { lat: elat, lon: elon, alt: alt_e as f64 * FT_TO_M + ge.unwrap_or(0.0) });
        let po = llh_to_ecef(Llh { lat: olat, lon: olon, alt: alt_o as f64 * FT_TO_M + go.unwrap_or(0.0) });
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
        let (freq, rpos, tb_wall) = match self.rxs.get_mut(&k) {
            Some(r) => {
                r.sync_msgs += 1;
                let tb_s = et.max(ot) / r.freq;
                r.wall.observe(now, tb_s);
                (r.freq, r.pos, r.wall.map(tb_s, now))
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
            let valid = self.validate_sync(ma, mb, now);
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
            // The same dated by the receiver's clock, for the 2-site path
            // (WallMap): positions arriving by paths with different delays
            // keep their order and spacing.
            if let Some(tw) = tb_wall {
                // Replay dump: the clock-dated ADS-B positions of aircraft
                // with 2-site records, for an offline interpolated truth.
                if !ac.two.is_empty() {
                    if let Some(w) = self.two_site_dump.as_mut() {
                        let l = ecef_to_llh(pb);
                        let line = serde_json::json!({"adsb": format!("{icao:06x}"), "t": tw, "lat": l.lat, "lon": l.lon});
                        let _ = std::io::Write::write_all(w, format!("{line}\n").as_bytes());
                    }
                }
                let h = &mut ac.adsb_hist;
                if h.iter().all(|(_, t0)| (tw - t0).abs() > 0.2) {
                    // Datings from different receivers can arrive out of
                    // order: keep the history sorted.
                    let i = h.partition_point(|(_, t0)| *t0 < tw);
                    h.insert(i, (pb, tw));
                }
                while h.back().is_some_and(|(_, tl)| h.front().is_some_and(|(_, t0)| tl - t0 > ADSB_HIST_S)) {
                    h.pop_front();
                }
            }
            // A validated ADS-B position: who hears this aircraft, and who
            // in range does not, is coverage evidence.
            let llh = ecef_to_llh(pb);
            // A validated position with a recent own difference: a sample for
            // the altitude model (at its barometric level).
            if let Some((d, t)) = ac.geo_baro {
                if now - t <= GEO_SAMPLE_AGE {
                    self.alt_model.insert(icao, llh.lat, llh.lon, alt_b as f64, d, now);
                }
            }
            let (rxs, by_cov) = (&self.rxs, &self.rx_by_cov);
            self.coverage.observe(icao, llh.lat, llh.lon, alt_b as f64 * 0.3048, now, |id| {
                by_cov.get(&id).and_then(|k| rxs.get(k)).is_some_and(|r| r.seen.contains(&icao))
            });
        }
        let variant = &mut entry.variants[vi];
        if variant.obs.iter().any(|o| o.rx == k) {
            return;
        }
        if variant.holdout {
            if variant.obs.is_empty() {
                if let Some(mk) = MsgKey::new(mb) {
                    self.holdout_due.push_back((now + 1.5, mk));
                    self.holdout_groups.insert(mk, Group { first_wall: now, copies: Vec::new(), truth: Some((pb, alt_b, icao)) });
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
        let (mut ok, mut bad) = (0, 0);
        for q in previous {
            let (o, b) = self.pair_update(k, &obs, q, now, (pa, pb));
            ok += o;
            bad += b;
        }
        if ok + bad > 0 {
            if let Some(ac) = self.aircraft.get_mut(&icao) {
                ac.sync_ok += ok as f32;
                ac.sync_bad += bad as f32;
            }
        }
    }

    /// Returns the observations the pair (accepted, rejected).
    fn pair_update(&mut self, k: RxKey, o: &SyncObs, q: SyncObs, now: f64, at: (Ecef, Ecef)) -> (u32, u32) {
        let pk = pair_key(k, q.rx);
        if !self.pairs.contains_key(&pk) {
            let (Some(r1), Some(r2)) = (self.rxs.get(&k), self.rxs.get(&q.rx)) else { return (0, 0) };
            let band = band_of(dist(&r1.pos, &r2.pos));
            let b = band as usize;
            let (c1, c2) = (r1.band_count[b], r2.band_count[b]);
            if (c1 >= PAIR_BUDGET[b] && c2 >= PAIR_BUDGET[b]) || c1 >= 2 * PAIR_BUDGET[b] || c2 >= 2 * PAIR_BUDGET[b] {
                self.stats.pair_budget_refused.fetch_add(1, Ordering::Relaxed);
                return (0, 0);
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
        if self.pairs.get(&pk).is_some_and(|m| m.updates >= 30 && now - m.updated_wall < PAIR_MIN_INTERVAL) {
            return (0, 0);
        }
        self.survey_observe(k, o, &q, now, at);
        let m = self.pairs.get_mut(&pk).unwrap();
        // Orientation: the model is theta = s_b - s_a with a = the lower key.
        let (a, b) = if k == pk.0 { (o, &q) } else { (&q, o) };
        let (mut ok, mut bad) = (0, 0);
        for (sa, sb) in [(a.tda, b.tda), (a.tdb, b.tdb)] {
            match m.observe(sa, sb - sa, now) {
                Outcome::Accepted => {
                    ok += 1;
                    self.stats.pair_updates.fetch_add(1, Ordering::Relaxed)
                }
                Outcome::Rejected => {
                    bad += 1;
                    self.stats.pair_rejects.fetch_add(1, Ordering::Relaxed)
                }
                Outcome::Reset => self.stats.pair_resets.fetch_add(1, Ordering::Relaxed),
                Outcome::ResetDrift => self.stats.pair_resets_drift.fetch_add(1, Ordering::Relaxed),
            };
        }
        (ok, bad)
    }

    /// Both receivers' view of one sync observation, for the survey: for
    /// each of the pair's two positions, z = (t_dK - t_dR) * c and the unit
    /// vector from R to the aircraft in R's local frame (survey.rs).
    fn survey_observe(&mut self, k: RxKey, o: &SyncObs, q: &SyncObs, now: f64, at: (Ecef, Ecef)) {
        let (Some(r1), Some(r2)) = (self.rxs.get(&k), self.rxs.get(&q.rx)) else { return };
        if r1.suspect || r2.suspect {
            return;
        }
        let unit = |r: &Rx, p: &Ecef| {
            let d = [p[0] - r.pos[0], p[1] - r.pos[1], p[2] - r.pos[2]];
            let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt().max(1.0);
            let m = &r.enu.r;
            let e = |i: usize| (m[i][0] * d[0] + m[i][1] * d[1] + m[i][2] * d[2]) / n;
            [e(0), e(1), e(2)]
        };
        let (c1, c2) = (r1.cov, r2.cov);
        let obs = [(at.0, o.tda, q.tda), (at.1, o.tdb, q.tdb)];
        let mut out = Vec::with_capacity(4);
        for (p, t1, t2) in obs {
            out.push((c1, c2, t1, (t2 - t1) * C_AIR, unit(r1, &p)));
            out.push((c2, c1, t2, (t1 - t2) * C_AIR, unit(r2, &p)));
        }
        for (r, kk, t, z, u) in out {
            self.survey.observe(r, kk, t, now, z, u);
        }
    }

    /// Position corrections (true = declared + d, metres east/north/up) for
    /// receivers with at least one closed window, by coverage identity.
    pub fn survey_estimates(&self) -> Vec<(CovId, survey::Estimate)> {
        self.survey.receivers().filter_map(|c| self.survey.estimate(c).map(|e| (c, e))).collect()
    }

    /// `/survey.json` and the survey gauges. Corrections only, never an
    /// absolute position; nothing here moves a receiver.
    fn render_survey(&self) {
        let names: HashMap<CovId, String> = self.rxs.values().map(|r| (r.cov, who_of(&r.info))).collect();
        let mut rows = Vec::new();
        let (mut n, mut reliable, mut over100) = (0usize, 0usize, 0usize);
        for (c, e) in self.survey_estimates() {
            n += 1;
            if e.reliable() {
                reliable += 1;
                if e.horizontal_m() > 100.0 {
                    over100 += 1;
                }
            }
            rows.push(serde_json::json!({
                "who": names.get(&c).cloned().unwrap_or_else(|| format!("cov:{c:016x}")),
                "east_m": e.d[0].round(), "north_m": e.d[1].round(), "up_m": e.d[2].round(),
                "sd_m": [e.sd[0].round(), e.sd[1].round(), e.sd[2].round()],
                "sd_weak_h_m": (e.sd_weak_h * 10.0).round() / 10.0, "sigma_m": e.sigma.round(),
                "windows": e.windows, "dropped": e.dropped, "reliable": e.reliable(),
            }));
        }
        {
            let mut snap = self.stats.snapshot.lock().unwrap();
            snap.survey_receivers = n;
            snap.survey_reliable = reliable;
            snap.survey_reliable_over_100m = over100;
        }
        *self.stats.survey_json.lock().unwrap() = serde_json::to_string(&rows).unwrap_or_default();
    }

    pub fn survey_counts(&self) -> (u64, u64, u64) {
        (self.survey.observations, self.survey.windows_closed, self.survey.windows_dropped)
    }

    // ---------------------------------------------------------------- mlat --

    fn on_mlat(&mut self, conn: ConnId, t: f64, msg: &[u8], wall: f64) {
        self.stats.mlat_msgs.fetch_add(1, Ordering::Relaxed);
        let Some(k) = self.rx_by_conn.get(&conn).copied() else { return };
        let Some(mk) = MsgKey::new(msg) else { return };
        let freq = match self.rxs.get_mut(&k) {
            Some(r) => {
                r.mlat_msgs += 1;
                r.wall.observe(wall, t / r.freq);
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
        self.serve_refresh(now);
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

    /// The clock edge between nodes i and j of a group (receiver keys, local
    /// times of this event), as norm::normalize wants it: (theta, variance)
    /// with s_j - s_i ~= theta, or None when that pair is not usable.
    fn edge(&self, keys: &[RxKey], s_of: &[f64], i: usize, j: usize, now: f64) -> Option<(f64, f64)> {
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
    }

    fn normalize(&self, keys: &[RxKey], nodes: &[Node], copies: &[Copy], now: f64) -> Option<norm::Normalized> {
        // Representative local time per node: its first copy.
        let mut s_of = vec![f64::NAN; nodes.len()];
        for c in copies {
            if s_of[c.node].is_nan() {
                s_of[c.node] = c.s;
            }
        }
        // The members' edges, once: they decide on bridges and feed the solve.
        let k = keys.len();
        let mut cache = vec![None; k * k];
        for i in 0..k {
            for j in (i + 1)..k {
                cache[i * k + j] = self.edge(keys, &s_of, i, j, now);
            }
        }
        // Bridges only when the members' own clock graph is disconnected
        // (norm.rs, "Bridges"): connected groups take exactly the old path.
        let comp = norm::member_components(k, |i, j| cache[i * k + j].is_some());
        let mut links = Vec::new();
        if comp.iter().any(|&c| c != comp[0]) {
            for (i, &k) in keys.iter().enumerate() {
                let Some(r) = self.rxs.get(&k) else { continue };
                for &p in &r.peers {
                    if keys.contains(&p) || self.rxs.get(&p).is_none_or(|x| x.suspect) {
                        continue;
                    }
                    if self.pairs.get(&pair_key(k, p)).is_some_and(|m| m.usable(now)) {
                        links.push((p, i));
                    }
                }
            }
        }
        let bridges = norm::choose_bridges(&comp, links, MAX_BRIDGES);
        if bridges.is_empty() {
            return norm::normalize(nodes, copies, |i, j| cache[i * k + j]);
        }
        let mut keys_all = keys.to_vec();
        let mut nodes_all = nodes.to_vec();
        for b in bridges {
            // The bridge's own clock reading at this event, through its pair
            // with the first member linked to it.
            let s_b = keys.iter().enumerate().find_map(|(i, &k)| {
                let pk = pair_key(k, b);
                let m = self.pairs.get(&pk).filter(|m| m.usable(now))?;
                Some(if pk.0 == k { s_of[i] + m.predict(s_of[i]).0 } else { m.a_time_at(s_of[i]) })
            });
            let (Some(s_b), Some(r)) = (s_b.filter(|v| v.is_finite()), self.rxs.get(&b)) else { continue };
            keys_all.push(b);
            // No copies: its jitter is never used.
            nodes_all.push(Node { pos: r.pos, jitter: 0.0 });
            s_of.push(s_b);
        }
        if keys_all.len() == k {
            return norm::normalize(nodes, copies, |i, j| cache[i * k + j]);
        }
        // normalize asks for i < j only: member pairs are cached.
        let normd = norm::normalize(&nodes_all, copies, |i, j| if j < k { cache[i * k + j] } else { self.edge(&keys_all, &s_of, i, j, now) })?;
        let mut per_comp: HashMap<usize, usize> = HashMap::new();
        for c in copies {
            *per_comp.entry(comp[c.node]).or_default() += 1;
        }
        let own = per_comp.values().copied().max().unwrap_or(0);
        self.stats.groups_bridged.fetch_add(1, Ordering::Relaxed);
        self.stats.bridge_receivers.fetch_add((keys_all.len() - k) as u64, Ordering::Relaxed);
        self.stats.bridged_copies.fetch_add(normd.copies.len().saturating_sub(own) as u64, Ordering::Relaxed);
        Some(normd)
    }

    /// Altitude constraint (geometric height m, sigma m) for aircraft `icao`
    /// near `at`, from this frame's altitude or a recent one.
    fn altitude_for(&self, icao: u32, ac: &Aircraft, decoded_alt: Option<i32>, now: f64, at: Llh) -> Option<(f64, f64)> {
        if let Some(a) = decoded_alt {
            return Some(self.geo_constraint(Some(icao), a, ALT_SIGMA_M, at, now));
        }
        let a = ac.alt_ft?;
        let age = now - ac.alt_t;
        if age > 30.0 {
            return None;
        }
        // 250 ft at report time, degrading at ~70 ft/s (mlat-server's model).
        Some(self.geo_constraint(Some(icao), a, (250.0 + 70.0 * age) * FT_TO_M, at, now))
    }

    /// Roughly where an aircraft is, for the altitude model (whose field is
    /// smooth over ~100 km): its track, its last fix, else the centroid of
    /// the receivers that heard it.
    fn rough_position(ac: &Aircraft, meas: &[Meas], t: f64) -> Llh {
        if let Some(tr) = ac.track.as_ref().filter(|tr| t - tr.last_update() < 60.0) {
            return ecef_to_llh(tr.predict(t).0);
        }
        if let Some((p, _)) = ac.last_fix.filter(|&(_, ft)| t - ft < 120.0) {
            return ecef_to_llh(p);
        }
        let n = meas.len().max(1) as f64;
        ecef_to_llh(meas.iter().fold([0.0; 3], |s, m| [s[0] + m.rx[0] / n, s[1] + m.rx[1] / n, s[2] + m.rx[2] / n]))
    }

    fn process_group(&mut self, mk: MsgKey, mut g: Group, now: f64) {
        self.stats.groups.fetch_add(1, Ordering::Relaxed);
        let bytes = mk.bytes().to_vec();
        if let Some(ac) = modes::false_ac(&bytes) {
            self.process_modeac(ac, g, now);
            return;
        }
        let Some(d) = modes::decode(&bytes) else { return };
        // The group's time by its receivers' clocks (WallMap): the earliest
        // mapped copy. None until some receiver's clock is mapped.
        let t_rx = g.copies.iter().filter_map(|(k, cs)| self.rxs.get(k).and_then(|r| r.wall.map(*cs, now))).reduce(f64::min);
        // The AP address of a corrupted frame is garbage; requiring that some
        // receiver reported the aircraft as seen filters those out.
        let known = self.aircraft.get(&d.address).map(|a| a.tracking > 0).unwrap_or(false);
        if known && (d.df == 20 || d.df == 21) && bytes.len() == 14 {
            self.commb_velocity(d.address, &bytes, t_rx.unwrap_or(g.first_wall));
        }
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
        if self.bench_two_sites {
            let ac = self.aircraft.get_mut(&d.address).unwrap();
            if now - ac.last_adsb < 30.0 {
                if ac.bench_pair.is_none() {
                    let mut first: Option<(RxKey, Ecef)> = None;
                    for (k, _) in &g.copies {
                        let Some(r) = self.rxs.get(k) else { continue };
                        match first {
                            None => first = Some((*k, r.pos)),
                            Some((a, pa)) if dist(&pa, &r.pos) >= 1000.0 => {
                                ac.bench_pair = Some((a, *k));
                                break;
                            }
                            _ => {}
                        }
                    }
                }
                match ac.bench_pair {
                    Some((a, b)) => g.copies.retain(|(k, _)| *k == a || *k == b),
                    None => g.copies.clear(),
                }
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
            let at = Self::rough_position(ac, &meas, t_wall);
            let alt = self.altitude_for(d.address, ac, d.altitude_ft, now, at);
            let used_keys: Vec<RxKey> = cl.iter().map(|&i| keys[copies[normd.copies[i]].node]).collect();
            let sites = distinct_sites(&meas);
            let t_clock = t_rx.map(|t0| t0 + (normd.t[cl[0]] - t_first));
            let ctx = GroupCtx { df: d.df, clusters: n_clusters, copies: g.copies.len(), edges: normd.edges, dropped: normd.dropped, sites, t_clock };
            self.update_aircraft(d.address, &meas, &cov, alt, t_wall, now, &used_keys, &ctx);
        }
    }

    /// A DF20/21 reply's MB field as Comm-B BDS 5,0 ground velocity, into the
    /// aircraft's track. Radars ask EHS aircraft for it every sweep; with it a
    /// track that drops to 2 receivers keeps its along-curve position (a
    /// 2-receiver + altitude track stayed within 333 m for 5 min in tests,
    /// 1.7 km without). Ambiguous 5,0/6,0 readings are taken as 5,0 only when
    /// they agree with the track's own speed; the track's gate rejects the rest.
    fn commb_velocity(&mut self, icao: u32, msg: &[u8], t: f64) {
        let keep = self.two_site != TwoSite::Off;
        let Some(ac) = self.aircraft.get_mut(&icao) else { return };
        let tracked = ac.track.as_ref().is_some_and(|tr| t - tr.last_update() <= 60.0);
        if !tracked && !keep {
            return;
        }
        let mut mb = [0u8; 7];
        mb.copy_from_slice(&msg[4..11]);
        let b50 = match modes::commb::infer_kinematics(&mb, ac.alt_ft) {
            Some(modes::commb::Kinematics::Bds50(b)) => b,
            // Ambiguous with BDS 6,0: only a track can tell them apart.
            Some(modes::commb::Kinematics::Ambiguous(b, _)) if tracked => {
                let tr = ac.track.as_ref().unwrap();
                let agrees = b.gs_kt.is_some_and(|gs| (gs - tr.ground_speed_kt()).abs() < 40.0)
                    && b.true_track_deg.is_some_and(|trk| ((trk - tr.track_deg() + 540.0) % 360.0 - 180.0).abs() < 20.0);
                if !agrees {
                    return;
                }
                b
            }
            _ => return,
        };
        let Some((ve, vn)) = b50.ground_velocity_en() else { return };
        if keep {
            ac.commb_vel = Some(([ve, vn], t));
            if ac.commb_hist.back().is_none_or(|x| t >= x.1) {
                ac.commb_hist.push_back(([ve, vn], t));
            }
            while ac.commb_hist.front().is_some_and(|x| t - x.1 > TWO_SITE_WINDOW + 60.0) {
                ac.commb_hist.pop_front();
            }
        }
        let Some(tr) = ac.track.as_mut().filter(|_| tracked) else { return };
        self.stats.commb_vel_offered.fetch_add(1, Ordering::Relaxed);
        // 2 kt and 0.18 deg resolution plus reporting latency: ~1.5 m/s.
        let t = t.max(tr.last_update());
        if tr.update_velocity(ve, vn, 1.5, t) == Gate::Accepted {
            ac.last_vel_update = t;
            self.stats.commb_vel_accepted.fetch_add(1, Ordering::Relaxed);
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
            if self.two_site != TwoSite::Off && ctx.sites == 2 && n == 2 {
                self.two_site_record(icao, meas, cov, alt, ctx.t_clock.unwrap_or(t), used);
            }
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
            let p = self.emit(icao, fix_used.as_ref(), n, t, now, used);
            // Credit the receivers with a position that reached the map
            // (not the bench copies of ADS-B aircraft), for their page.
            let to_map = self.bench_out.is_none() || self.aircraft.get(&icao).is_none_or(|ac| now - ac.last_adsb >= 30.0);
            if p.is_some() && to_map {
                for k in used {
                    if let Some(c) = self.rxs.get(k).map(|r| r.cov) {
                        self.coverage.count_fix(c, now);
                    }
                }
            }
            p
        } else {
            None
        };
        self.check_adsb(icao, n, t, now, fix_used.as_ref(), published, accepted, used, ctx);
        if self.two_site != TwoSite::Off && !can_fix {
            self.two_site_try(icao, ctx.t_clock.unwrap_or(t), t, now);
        }
    }

    /// Keep a 2-site TDOA (and who heard it) for the motion + coverage solve.
    fn two_site_record(&mut self, icao: u32, meas: &[Meas], cov: &DMatrix<f64>, alt: Option<(f64, f64)>, t: f64, used: &[RxKey]) {
        let Some((h, _)) = alt else { return };
        let var = cov[(0, 0)] + cov[(1, 1)] - 2.0 * cov[(0, 1)];
        let sd = (var.max(0.0).sqrt() * C_AIR).max(30.0);
        let rec = Rec2 { t, off: [0.0; 2], ra: meas[0].rx, rb: meas[1].rx, d: (meas[1].t - meas[0].t) * C_AIR, sd, h };
        let covs: Vec<CovId> = used.iter().filter_map(|k| self.rxs.get(k).map(|r| r.cov)).collect();
        let ac = self.aircraft.get_mut(&icao).unwrap();
        ac.two.push_back(rec);
        while ac.two.front().is_some_and(|r| t - r.t > TWO_SITE_WINDOW) {
            ac.two.pop_front();
        }
        for c in covs {
            if !ac.two_heard.contains(&c) {
                ac.two_heard.push(c);
            }
        }
    }

    /// Solve an aircraft that only two sites hear, from its recent 2-site
    /// TDOAs, its Comm-B velocity and the coverage maps (twosite.rs). What
    /// passes the checks becomes a fix for its track, which is published by
    /// the usual rules (confirmed, anchored by a fix in the last 10 s); in
    /// Bench mode only aircraft that also send ADS-B, to the bench output.
    ///
    /// `t` is the update's time by the receivers' clocks (the records, the
    /// Comm-B history and the ADS-B check are dated the same way), `t_arr`
    /// the arrival-based time the track and the output run on: the fix is
    /// moved along the velocity from one to the other.
    fn two_site_try(&mut self, icao: u32, t: f64, t_arr: f64, now: f64) {
        let Some(ac) = self.aircraft.get(&icao) else { return };
        if t - ac.two_tried < TWO_SITE_EVERY || t_arr - ac.last_fix_update < 30.0 || ac.two.len() < TWO_SITE_MIN_RECS {
            return;
        }
        // Study knobs (replay): the velocity source and the window length.
        let oracle = std::env::var("MLAT_NG_TWO_SITE_VEL").is_ok_and(|v| v == "adsb");
        let window = std::env::var("MLAT_NG_TWO_SITE_WINDOW").ok().and_then(|v| v.parse::<f64>().ok()).unwrap_or(TWO_SITE_WINDOW);
        let (vel, vt) = if oracle {
            let (Some(v), Some((p, ta))) = (ac.adsb_vel, ac.adsb) else { return };
            let f = Enu::at(p);
            let l = f.to_enu(&[p[0] + v[0], p[1] + v[1], p[2] + v[2]]);
            ([l[0], l[1]], ta)
        } else {
            let Some(cv) = ac.commb_vel else { return };
            cv
        };
        if t - vt > TWO_SITE_VEL_AGE {
            return;
        }
        let has_adsb = now - ac.last_adsb < 30.0;
        if self.two_site == TwoSite::Bench && !has_adsb {
            return;
        }
        let span = ac.two.back().unwrap().t - ac.two.front().unwrap().t.max(t - window);
        if span < TWO_SITE_MIN_SPAN {
            return;
        }
        let in_window: Vec<&Rec2> = ac.two.iter().filter(|r| t - r.t <= window).collect();
        if in_window.len() < TWO_SITE_MIN_RECS {
            return;
        }
        let step = in_window.len().div_ceil(TWO_SITE_MAX_RECS);
        let mut recs: Vec<Rec2> = in_window.iter().rev().step_by(step).map(|r| (*r).clone()).collect();
        recs.reverse();
        // Where the aircraft went from each record to the last: its velocity
        // history integrated (the oracle study knob: one ADS-B velocity).
        let hist: Vec<([f64; 2], f64)> = if oracle { vec![(vel, vt)] } else { ac.commb_hist.iter().copied().collect() };
        let t_last = recs.last().unwrap().t;
        let ts: Vec<f64> = recs.iter().map(|r| r.t).collect();
        for (r, off) in recs.iter_mut().zip(twosite::offsets(&ts, &hist, t_last)) {
            r.off = off;
        }
        let heard = ac.two_heard.clone();
        // The aircraft's own ADS-B at the solution's time, both dated by the
        // receivers' clocks: interpolated, never extrapolated (adsb_hist).
        let truth = adsb_at(&ac.adsb_hist, t);
        let last = recs.last().unwrap().clone();
        let mid = ecef_to_llh([(last.ra[0] + last.rb[0]) / 2.0, (last.ra[1] + last.rb[1]) / 2.0, (last.ra[2] + last.rb[2]) / 2.0]);
        let silent: Vec<CovId> = self
            .coverage
            .near(mid.lat, mid.lon)
            .into_iter()
            .filter(|id| {
                !heard.contains(id)
                    && !self.rx_by_cov.get(id).and_then(|k| self.rxs.get(k)).is_some_and(|r| r.seen.contains(&icao))
            })
            .collect();
        self.stats.two_site_tries.fetch_add(1, Ordering::Relaxed);
        let coverage = &self.coverage;
        // Study knob (replay): which coverage evidence the solve uses.
        let mode = std::env::var("MLAT_NG_TWO_SITE_COV").unwrap_or_default();
        let none: Vec<CovId> = Vec::new();
        let (h_ids, s_ids) = match mode.as_str() {
            "none" => (&none, &none),
            "heard" => (&heard, &none),
            _ => (&heard, &silent),
        };
        let cov_cost = |lat: f64, lon: f64| -coverage.log_lik(h_ids, s_ids, lat, lon, last.h, TWO_SITE_N_EFF);
        let t0 = Instant::now();
        let sol = twosite::solve(&recs, &cov_cost);
        self.stats.two_site_us.fetch_add(t0.elapsed().as_micros() as u64, Ordering::Relaxed);
        self.aircraft.get_mut(&icao).unwrap().two_tried = t;
        let Some(sol) = sol else { return };
        self.stats.two_site_solved.fetch_add(1, Ordering::Relaxed);
        let ok = sol.rms <= TWO_SITE_MAX_RMS && sol.err_h <= TWO_SITE_MAX_ERR && sol.margin >= TWO_SITE_MIN_MARGIN;
        if let Some(p) = truth {
            self.two_site_err.push_back((now, dist(&p, &sol.pos), ok));
            if let Some(w) = self.two_site_dump.as_mut() {
                let mid = [(last.ra[0] + last.rb[0]) / 2.0, (last.ra[1] + last.rb[1]) / 2.0, (last.ra[2] + last.rb[2]) / 2.0];
                let (sl, tl) = (ecef_to_llh(sol.pos), ecef_to_llh(p));
                let line = serde_json::json!({
                    "sol_lat": sl.lat, "sol_lon": sl.lon, "truth_lat": tl.lat, "truth_lon": tl.lon, "now": now,
                    "t": t, "icao": format!("{icao:06x}"), "err_m": dist(&p, &sol.pos), "ok": ok,
                    "rms": sol.rms, "err_h": sol.err_h, "margin": sol.margin, "n": sol.n, "span": sol.span,
                    "baseline_m": dist(&last.ra, &last.rb), "range_m": dist(&mid, &p), "h": last.h,
                    "speed": (vel[0] * vel[0] + vel[1] * vel[1]).sqrt(),
                    "heard": heard.len(), "silent": silent.len(),
                    "pairs": recs.iter().map(|r| (r.ra[0] as i64, r.rb[0] as i64)).collect::<std::collections::HashSet<_>>().len(),
                });
                let _ = std::io::Write::write_all(w, format!("{line}\n").as_bytes());
            }
        }
        if !ok {
            return;
        }
        self.stats.two_site_accepted.fetch_add(1, Ordering::Relaxed);
        // The fix: the solution with its horizontal error, the altitude as
        // the constraint it came from.
        let frame = Enu::at(sol.pos);
        let rot = Matrix3::from_fn(|i, j| frame.r[i][j]);
        let var_h = sol.err_h * sol.err_h / 2.0;
        let cov = rot.transpose() * Matrix3::from_diagonal(&Vector3::new(var_h, var_h, 100.0 * 100.0)) * rot;
        // Where it is at the track's (arrival-based) time.
        let dt = t_arr - t;
        let shift = rot.transpose() * Vector3::new(vel[0] * dt, vel[1] * dt, 0.0);
        let pos = [sol.pos[0] + shift[0], sol.pos[1] + shift[1], sol.pos[2] + shift[2]];
        let fix = Fix { pos, cov, t0: 0.0, chi2: sol.rms * sol.rms, dof: 0, err_h: sol.err_h, gdop: 0.0, used: Vec::new() };
        let ac = self.aircraft.get_mut(&icao).unwrap();
        let fed = match ac.track.as_mut() {
            Some(tr) if t_arr - tr.last_update() < 60.0 => tr.update_fix(&fix, t_arr) == Gate::Accepted,
            _ => {
                ac.track = Some(Track::new(&fix, t_arr));
                ac.track_rejects = 0;
                true
            }
        };
        if !fed {
            return;
        }
        if let Some(tr) = ac.track.as_mut() {
            tr.update_velocity(vel[0], vel[1], 1.5, t_arr);
        }
        ac.last_fix_update = t_arr;
        ac.last_vel_update = t_arr;
        self.stats.two_site_tracked.fetch_add(1, Ordering::Relaxed);
        // The published position, taken back to the solution's time.
        if let (Some(pos), Some(p)) = (self.emit(icao, Some(&fix), 2, t_arr, now, &[]), truth) {
            let back = [pos[0] - shift[0], pos[1] - shift[1], pos[2] - shift[2]];
            self.two_site_pub.push_back((now, dist(&p, &back)));
        }
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
    /// Publish the aircraft's position; served receivers in `to` (the ones
    /// whose copies made it) also get it back as a result line.
    fn emit(&mut self, icao: u32, fix: Option<&Fix>, n: usize, t: f64, now: f64, to: &[RxKey]) -> Option<Ecef> {
        let publish = !self.served_output_only || to.iter().any(|k| self.rxs.get(k).is_some_and(|r| r.info.serve.is_some()));
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
        // Or coasting: a fix in the last 5 min, a Comm-B ground velocity in
        // the last 15 s holding the along-curve position, and a tight track.
        let anchored = t - ac.last_fix_update <= 10.0
            || (t - ac.last_fix_update <= 300.0 && t - ac.last_vel_update <= 15.0 && tr.err_h() < 1500.0);
        let (pos, err, vel, cov) = if tr.confirmed() && anchored {
            let (p, c) = tr.predict(t);
            (p, tr.err_h(), true, c)
        } else if let Some(f) = fix.filter(|f| f.dof >= 1 && f.err_h < 1000.0) {
            (f.pos, f.err_h, false, f.cov)
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
        let result = (!to.is_empty()).then(|| {
            let (gs, trk) = (tr.ground_speed_kt(), tr.track_deg().to_radians());
            ClientResult {
                addr: icao,
                t,
                ecef: [pos[0], pos[1], pos[2]],
                lat: l.lat,
                lon: l.lon,
                alt_m: l.alt,
                cov: Some([[cov[(0, 0)], cov[(0, 1)], cov[(0, 2)]], [cov[(1, 0)], cov[(1, 1)], cov[(1, 2)]], [cov[(2, 0)], cov[(2, 1)], cov[(2, 2)]]]),
                receivers: n,
                distinct: n,
                vel_kt: vel.then(|| (gs * trk.cos(), gs * trk.sin())),
                vrate_fpm: vel.then(|| tr.vrate_fpm()),
                callsign: None,
                squawk: ac.squawk,
            }
        });
        let callsign = ac.callsign.clone();
        if let Some(mut res) = result {
            res.callsign = callsign.as_deref();
            let (mut old, mut ecef): (Option<String>, Option<String>) = (None, None);
            for k in to {
                let Some(r) = self.rxs.get(k) else { continue };
                let Some(opts) = r.info.serve else { continue };
                let line = match opts.results {
                    ResultFormat::None => continue,
                    ResultFormat::Old => old.get_or_insert_with(|| result_old(&res)).clone(),
                    ResultFormat::Ecef => ecef.get_or_insert_with(|| result_ecef(&res)).clone(),
                };
                if r.tx.try_send(ToClient::ResultLine(line)).is_ok() {
                    self.stats.serve_results.fetch_add(1, Ordering::Relaxed);
                } else {
                    self.stats.serve_send_full.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
        self.stats.last_output_ms.store((now * 1000.0) as u64, Ordering::Relaxed);
        if !publish {
            self.stats.outputs_not_served.fetch_add(1, Ordering::Relaxed);
            return Some(pos);
        }
        match &self.bench_out {
            Some(bench) if now - ac.last_adsb < 30.0 => {
                self.stats.bench_outputs.fetch_add(1, Ordering::Relaxed);
                let _ = bench.send(Arc::from(line));
            }
            _ => {
                self.stats.outputs.fetch_add(1, Ordering::Relaxed);
                let _ = self.out.send(Arc::from(line));
            }
        }
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
            let at = ecef_to_llh(centroid[0]);
            if let Some(ft) = mac.altitude_ft {
                let alt = Some(self.geo_constraint(None, ft, 100.0, at, now));
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
                let alt = if mode_c { mac.altitude_ft.map(|ft| self.geo_constraint(None, ft, 100.0, at, now)) } else { None };
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
        let Some((truth, alt_ft, icao)) = g.truth else { return };
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
        // Like any multilaterated aircraft: corrected by OTHER aircraft only.
        let alt = Some(self.geo_constraint(Some(icao), alt_ft, ALT_SIGMA_M, ecef_to_llh(truth), now));
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
        while self.two_site_err.front().is_some_and(|x| now - x.0 >= 600.0) {
            self.two_site_err.pop_front();
        }
        while self.two_site_pub.front().is_some_and(|x| now - x.0 >= 600.0) {
            self.two_site_pub.pop_front();
        }
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
        self.survey.sweep(now);
        self.render_survey();
        // Sync quality per aircraft over roughly the last minute.
        for a in self.aircraft.values_mut() {
            a.sync_ok *= 0.8;
            a.sync_bad *= 0.8;
        }
        self.send_serve_stats(now);
        if let Some(path) = &self.coverage_file {
            if now - self.coverage_saved >= COVERAGE_SAVE_EVERY {
                self.coverage_saved = now;
                if let Err(e) = self.coverage.save(path) {
                    tracing::warn!(path, error = %e, "coverage maps not saved");
                }
            }
        }
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
        let mut rate: HashMap<RxKey, (f64, u32, Vec<f64>)> = HashMap::new();
        for ((a, b), m) in &self.pairs {
            if m.updates < crate::clock::MIN_UPDATES {
                continue;
            }
            for k in [a, b] {
                let e = rate.entry(*k).or_insert((0.0, 0, Vec::new()));
                e.0 += m.reject_rate;
                e.1 += 1;
                e.2.push(m.noise_scale());
            }
        }
        for (k, r) in self.rxs.iter_mut() {
            let (sum, n, noise) = rate.get(k).map(|e| (e.0, e.1, &e.2[..])).unwrap_or((0.0, 0, &[]));
            r.suspect = n >= 4 && sum / n as f64 > 0.35;
            // Quieter than nominal only as far as its own pairs back it.
            r.noise_scale = receiver_noise_scale(noise);
        }
        self.alt_model.expire(now);
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
        self.render_registry(now);
    }

    /// Receivers with a UUID, keyed as readsb keys the feed (first 64 bits).
    fn render_registry(&self, now: f64) {
        let receivers: Vec<crate::relay::registry::Entry> = self
            .rxs
            .values()
            .filter_map(|r| {
                let k = crate::relay::registry::key_of_uuid(r.info.uuid.as_deref()?)?;
                Some(crate::relay::registry::Entry {
                    key: format!("{k:016x}"),
                    user: r.info.user.clone(),
                    lat: r.info.lat,
                    lon: r.info.lon,
                    alt: r.info.alt,
                    clock: r.info.clock.as_str().to_string(),
                    privacy: r.info.privacy,
                })
            })
            .collect();
        let doc = crate::relay::registry::Doc { generated: now, receivers };
        *self.stats.registry_json.lock().unwrap() = serde_json::to_string(&doc).unwrap_or_default();
    }

    fn publish(&self, queue_depth: usize) {
        self.publish_at(queue_depth, unix_now());
    }

    /// ADS-B-checked updates of the last 10 minutes: (receivers, stand-alone
    /// fix error, published error), metres.
    pub(crate) fn adsb_errors(&self) -> Vec<(usize, Option<f64>, Option<f64>)> {
        self.adsb_err.iter().map(|x| (x.1, x.2, x.3)).collect()
    }

    pub(crate) fn publish_at(&self, queue_depth: usize, now: f64) {
        let mut errs: Vec<f64> = self.holdout_err.iter().map(|x| x.1).collect();
        errs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let q = |p: f64| if errs.is_empty() { 0.0 } else { errs[((errs.len() - 1) as f64 * p) as usize] };
        let mut s = self.stats.snapshot.lock().unwrap();
        s.receivers = self.rxs.len();
        s.coverage_maps = self.coverage.receivers();
        s.coverage_samples = self.coverage.samples;
        s.alt_model_cells = self.alt_model.cells();
        s.alt_model_samples = self.alt_model.samples();
        let served = self.rxs.values().filter(|r| r.info.serve.is_some());
        let (mut n, mut ws, mut wm) = (0, 0, 0);
        for r in served {
            n += 1;
            ws += r.want_sync;
            wm += r.want_mlat;
        }
        s.serve_receivers = n;
        s.serve_requested_sync = ws;
        s.serve_requested_mlat = wm;
        s.serve_mlat_wanted = self.interest.wanted().len();
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
        let pct = |mut v: Vec<f64>| {
            v.sort_by(|a, b| a.total_cmp(b));
            let at = |p: f64| if v.is_empty() { 0.0 } else { v[((v.len() - 1) as f64 * p) as usize] };
            (v.len(), at(0.5), at(0.9))
        };
        s.two_site_all = pct(self.two_site_err.iter().filter(|x| now - x.0 < 600.0).map(|x| x.1).collect());
        s.two_site_ok = pct(self.two_site_err.iter().filter(|x| now - x.0 < 600.0 && x.2).map(|x| x.1).collect());
        s.two_site_pub = pct(self.two_site_pub.iter().filter(|x| now - x.0 < 600.0).map(|x| x.1).collect());
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
    /// This update's time by the receivers' clocks (WallMap), when mapped.
    t_clock: Option<f64>,
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
/// Share of an aircraft's recent sync observations its clock pairs rejected
/// (0 until there are enough of them).
fn sync_bad_share(a: &Aircraft) -> f64 {
    let n = a.sync_ok + a.sync_bad;
    if n < SYNC_QUALITY_MIN_OBS {
        0.0
    } else {
        (a.sync_bad / n) as f64
    }
}

/// An aircraft's ADS-B position at time `t` from its clock-dated history:
/// interpolated between the positions either side (at most 3 s apart), or
/// the nearest one within 1 s; None otherwise.
fn adsb_at(h: &VecDeque<(Ecef, f64)>, t: f64) -> Option<Ecef> {
    let i = h.partition_point(|(_, ti)| *ti < t);
    if i > 0 && i < h.len() {
        let ((p0, t0), (p1, t1)) = (h[i - 1], h[i]);
        if t1 - t0 <= 3.0 {
            let k = if t1 > t0 { (t - t0) / (t1 - t0) } else { 0.0 };
            return Some([p0[0] + k * (p1[0] - p0[0]), p0[1] + k * (p1[1] - p0[1]), p0[2] + k * (p1[2] - p0[2])]);
        }
    }
    [i.checked_sub(1), (i < h.len()).then_some(i)]
        .into_iter()
        .flatten()
        .map(|j| h[j])
        .filter(|(_, tj)| (tj - t).abs() <= 1.0)
        .min_by(|a, b| (a.1 - t).abs().total_cmp(&(b.1 - t).abs()))
        .map(|(p, _)| p)
}

pub(crate) fn fxhash(b: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &x in b {
        h ^= x as u64;
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::ClockType;

    fn engine() -> Engine {
        let (out, _) = broadcast::channel(16);
        Engine::new(EngineConfig { shadow: true, mlat_delay: 1.0, holdout_every: 0 }, Arc::new(Stats::default()), out)
    }

    fn add_rx(e: &mut Engine, conn: ConnId, lon: f64) -> RxKey {
        let (tx, _) = mpsc::channel(1);
        let info = ReceiverInfo {
            user: format!("rx{conn}"),
            uuid: None,
            lat: 45.0,
            lon,
            alt: 100.0,
            clock: ClockType::Dump1090,
            privacy: false,
            source_ip: "test".into(),
            client_version: "test".into(),
            compress: "none",
            serve: None,
        };
        e.connect(conn, info, tx, 0.0);
        e.rx_by_conn[&conn]
    }

    /// A converged pair between a and b for clock offsets (s_x = tau + off_x).
    fn add_pair(e: &mut Engine, a: RxKey, b: RxKey, off: &HashMap<RxKey, f64>, now: f64) {
        let pk = pair_key(a, b);
        let mut m = PairModel::new(ClockType::Dump1090, ClockType::Dump1090, 0);
        for i in 0..20 {
            let tau = 1000.0 + i as f64;
            m.observe(tau + off[&pk.0], off[&pk.1] - off[&pk.0], now - 10.0 + i as f64 * 0.5);
        }
        e.pairs.insert(pk, m);
        e.rxs.get_mut(&a).unwrap().peers.insert(b);
        e.rxs.get_mut(&b).unwrap().peers.insert(a);
    }

    #[test]
    fn own_geo_baro_corrects_sync_references_but_never_its_own_constraint() {
        let mut e = engine();
        let now = 100.0;
        let at = Llh { lat: 45.2, lon: 9.2, alt: 0.0 };
        // Aircraft 1 broadcasts +1000 ft and is the only sample in the region.
        e.aircraft.entry(1).or_default().geo_baro = Some((1000.0, now));
        e.alt_model.insert(1, 45.2, 9.2, 35_000.0, 1000.0, now);
        // Its sync references use its own difference...
        let own = e.sync_geo_m(1, at.lat, at.lon, 35_000, now).unwrap();
        assert!((own - 1000.0 * FT_TO_M).abs() < 1e-9);
        // ...its multilateration constraint does not: no other aircraft, so
        // barometric with the wide fallback sigma.
        let (h, sigma) = e.geo_constraint(Some(1), 35_000, ALT_SIGMA_M, at, now);
        assert!((h - 35_000.0 * FT_TO_M).abs() < 1e-9);
        assert!(sigma > 300.0, "{sigma}");
        // Four other aircraft saying +600 ft: corrected by theirs alone.
        for k in 2..6u32 {
            e.alt_model.insert(k, 45.2 + 0.05 * k as f64, 9.2, 35_000.0, 600.0, now);
        }
        let (h, sigma) = e.geo_constraint(Some(1), 35_000, ALT_SIGMA_M, at, now);
        assert!((h - (35_000.0 + 600.0) * FT_TO_M).abs() < 1.0, "{}", h / FT_TO_M);
        assert!(sigma < 100.0, "{sigma}");
        assert_eq!(e.stats.alt_geo_fallback.load(Ordering::Relaxed), 1);
        assert_eq!(e.stats.alt_geo_corrected.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn bridges_only_when_the_group_is_disconnected() {
        let mut e = engine();
        let now = 50.0;
        let (a, b, c) = (add_rx(&mut e, 1, 9.0), add_rx(&mut e, 2, 9.3), add_rx(&mut e, 3, 9.6));
        let off: HashMap<RxKey, f64> = [(a, 0.0), (b, 1234.5), (c, -77.25)].into_iter().collect();
        // a and b heard it; they share no pair, but both have one with c.
        add_pair(&mut e, a, c, &off, now);
        add_pair(&mut e, b, c, &off, now);
        let tau = 1015.0;
        let g = Group { first_wall: now, copies: vec![(a, tau + off[&a]), (b, tau + off[&b])], truth: None };
        let (keys, nodes, copies) = e.build(&g);
        let n = e.normalize(&keys, &nodes, &copies, now).expect("bridged through c");
        assert_eq!(n.copies.len(), 2);
        assert!((n.t[0] - n.t[1]).abs() < 1e-6, "{:?}", n.t);
        assert_eq!(e.stats.groups_bridged.load(Ordering::Relaxed), 1);
        assert_eq!(e.stats.bridge_receivers.load(Ordering::Relaxed), 1);
        assert_eq!(e.stats.bridged_copies.load(Ordering::Relaxed), 1);
        // Once a and b have their own pair the group is connected: no bridge.
        add_pair(&mut e, a, b, &off, now);
        let n = e.normalize(&keys, &nodes, &copies, now).unwrap();
        assert_eq!(n.copies.len(), 2);
        assert_eq!(n.edges, 1, "only the direct edge");
        assert_eq!(e.stats.groups_bridged.load(Ordering::Relaxed), 1);
    }

    /// A receiver in serve mode, with the receiving end of its connection.
    fn add_served(e: &mut Engine, conn: ConnId, lon: f64, now: f64) -> (RxKey, mpsc::Receiver<ToClient>) {
        let (tx, rx) = mpsc::channel(64);
        let info = ReceiverInfo {
            user: format!("srv{conn}"),
            uuid: None,
            lat: 45.0,
            lon,
            alt: 100.0,
            clock: ClockType::Dump1090,
            privacy: false,
            source_ip: "test".into(),
            client_version: "test".into(),
            compress: "none",
            serve: Some(crate::types::ServeOpts { results: ResultFormat::Ecef, stats: true }),
        };
        e.connect(conn, info, tx, now);
        (e.rx_by_conn[&conn], rx)
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

    #[test]
    fn served_receivers_are_asked_shadow_ones_never() {
        let mut e = engine();
        let now = 10_000.0;
        let (_, mut rx1) = add_served(&mut e, 1, 9.0, now);
        let (_, mut rx2) = add_served(&mut e, 2, 9.5, now);
        let (tx3, mut rx3) = mpsc::channel(64);
        let info3 = ReceiverInfo { serve: None, ..e.rxs[&e.rx_by_conn[&1]].info.clone() };
        e.connect(3, info3, tx3, now);
        // 0x111111 has no ADS-B: once two receivers see it, it is wanted.
        for c in [1, 2, 3] {
            e.handle_at(Event::Seen { conn: c, icao: vec![0x111111, 0x222222] }, now);
        }
        e.aircraft.get_mut(&0x222222).unwrap().last_adsb = now;
        // mlat-client's empty first rate report, then the real ones.
        for c in [1, 2, 3] {
            e.handle_at(Event::RateReport { conn: c, rates: vec![] }, now + 1.0);
        }
        e.handle_at(Event::RateReport { conn: 1, rates: vec![(0x222222, 1.5)] }, now + 2.0);
        e.handle_at(Event::RateReport { conn: 2, rates: vec![(0x222222, 1.5)] }, now + 2.2);
        let (a1, a2) = (started(&mut rx1), started(&mut rx2));
        assert!(a1.contains(&0x111111) && a2.contains(&0x111111), "{a1:x?} {a2:x?}");
        assert!(a2.contains(&0x222222), "sync aircraft shared with a peer: {a2:x?}");
        assert!(rx3.try_recv().is_err(), "a shadow connection is never asked");
        assert!(e.stats.serve_start_lines.load(Ordering::Relaxed) >= 2);
    }

    #[test]
    fn newly_wanted_aircraft_are_asked_at_once() {
        let mut e = engine();
        let now = 10_000.0;
        let (_, mut rx1) = add_served(&mut e, 1, 9.0, now);
        let (_, mut rx2) = add_served(&mut e, 2, 9.5, now);
        for c in [1, 2] {
            e.handle_at(Event::RateReport { conn: c, rates: vec![] }, now);
        }
        let _ = (started(&mut rx1), started(&mut rx2));
        // Seen by one receiver: not wanted yet. The second one makes it
        // wanted, and both are asked without waiting for a rate report.
        e.handle_at(Event::Seen { conn: 1, icao: vec![0x333333] }, now + 1.0);
        assert!(started(&mut rx1).is_empty());
        e.handle_at(Event::Seen { conn: 2, icao: vec![0x333333] }, now + 2.0);
        assert_eq!(started(&mut rx1), vec![0x333333]);
        assert_eq!(started(&mut rx2), vec![0x333333]);
        assert_eq!(e.stats.serve_fast_starts.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn stats_lines_go_to_served_clients_that_asked() {
        let mut e = engine();
        let (_, mut rx1) = add_served(&mut e, 1, 9.0, 0.0);
        e.send_serve_stats(100.0);
        let Ok(ToClient::StatsLine(l)) = rx1.try_recv() else { panic!("no stats line") };
        let v: serde_json::Value = serde_json::from_str(&l).unwrap();
        assert_eq!(v["stats"]["peer_count"], 0);
        assert_eq!(v["stats"]["bad_sync_timeout"], 0);
        assert!(v["stats"]["outlier_percent"].is_number());
        // Not again before 15 s.
        e.send_serve_stats(110.0);
        assert!(rx1.try_recv().is_err());
    }

    #[test]
    fn wall_map_follows_the_quickest_deliveries() {
        let mut m = WallMap::default();
        // A clock 1000 s behind wall time; deliveries 0.2-1.4 s late
        // (batches), the quickest 0.2 s.
        for i in 0..40 {
            let clock = 5000.0 + i as f64 * 0.5;
            let late = 0.2 + (i % 7) as f64 * 0.2;
            assert_eq!(m.map(clock, clock + 1000.0 + late).is_some(), i >= WALLMAP_MIN_OBS as usize);
            m.observe(clock + 1000.0 + late, clock);
        }
        let t = m.map(5100.0, 6101.0).unwrap();
        assert!((t - 6100.2).abs() < 1e-9, "{t}");
        // A clock that jumped is not mapped.
        assert!(m.map(9000.0, 6102.0).is_none());
        // After two buckets the old minimum is gone: a slower path alone
        // maps with its own (larger) delay.
        let mut now = 6200.0;
        while now < 6300.0 {
            m.observe(now, now - 1000.0 - 0.9);
            now += 1.0;
        }
        let t = m.map(5300.0, 6301.0).unwrap();
        assert!((t - 6300.9).abs() < 1e-9, "{t}");
    }

    #[test]
    fn adsb_truth_is_interpolated_never_extrapolated() {
        let h: VecDeque<(Ecef, f64)> = [([0.0, 0.0, 0.0], 10.0), ([200.0, 0.0, 0.0], 11.0), ([1000.0, 0.0, 0.0], 20.0)].into_iter().collect();
        assert_eq!(adsb_at(&h, 10.5).unwrap(), [100.0, 0.0, 0.0]);
        // A 9 s gap: only a position within 1 s counts.
        assert_eq!(adsb_at(&h, 11.8).unwrap(), [200.0, 0.0, 0.0]);
        assert!(adsb_at(&h, 15.0).is_none());
        assert_eq!(adsb_at(&h, 20.6).unwrap(), [1000.0, 0.0, 0.0]);
        assert!(adsb_at(&h, 22.0).is_none());
        assert!(adsb_at(&VecDeque::new(), 1.0).is_none());
    }
}
