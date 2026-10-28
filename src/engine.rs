//! The engine: one thread that owns all state (receivers, clock pairs, sync
//! points, message groups, aircraft, tracks) and processes events in arrival
//! order. Arrival order matters: a clock pair's observations must be applied in
//! time order, and a single owner makes that free and deterministic.
//!
//! Since 2026-10-28 (docs/MULTICORE.md, step 1) it is in two stages. This file
//! is stage 1: everything that must stay in one order, plus freezing each due
//! Mode S group into a job. worker.rs makes the positions from those jobs,
//! one worker per share of the aircraft addresses; the workers still run
//! inline here, and the output does not depend on how many there are.
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

use crate::hashing::{HashMap, HashSet};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use tokio::sync::{broadcast, mpsc, oneshot};

use crate::altmodel::AltModel;
use crate::clock::{band_of, receiver_noise_scale, Outcome, PairModel, PAIR_BUDGET};
use crate::coverage::{self, CovId, Coverage};
use crate::covworker::{CovHandle, CovShared, ToCoverage};
use crate::geo::{dist, ecef_to_llh, llh_to_ecef, Ecef, Enu, Llh, C_AIR, FT_TO_M};
use crate::interest::{self, AcMeta, AcState, Interest};
use crate::modes;
use crate::norm::{self, Copy, Node};
use crate::output::{sbs_line, SbsFix};
use crate::solver::{self, Fix, SolveInput};
use crate::survey::{self, Survey};
use crate::track::{Gate, Track};
use crate::types::{ConnId, Event, ReceiverInfo, ToClient};
use crate::worker::{self, AdsbUpdate, Effects, Frozen, GroupJob, HoldoutJob, ModeAcCand, Pool, RxRef, ToWorker, Worker};

/// Aircraft-to-receiver range beyond which a sync point is not used (radio
/// horizon at FL450 is ~480 km).
const MAX_RANGE: f64 = 500e3;
/// Coverage maps are saved this often (seconds) when a file is configured.
const COVERAGE_SAVE_EVERY: f64 = 1800.0;
/// The survey and its corrections, likewise (corrections change every 10
/// minutes).
const SURVEY_SAVE_EVERY: f64 = 600.0;
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
pub(crate) const ALT_SIGMA_M: f64 = 76.0;
/// An aircraft's own geometric-minus-baro difference is used for its sync
/// references up to this age (s), and enters the model up to `GEO_SAMPLE_AGE`.
const GEO_OWN_AGE: f64 = 60.0;
const GEO_SAMPLE_AGE: f64 = 30.0;
/// Mode A/C payloads that are shared by too many aircraft at once (VFR,
/// conspicuity, defaults) or are line noise, as acbridge.py/mlat-client.
/// Seconds of clock-dated ADS-B positions kept per aircraft (adsb_hist).
pub(crate) const ADSB_HIST_S: f64 = 30.0;
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
    /// `info.user`, shared with jobs (diagnostics).
    user: Arc<str>,
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

/// The sync half of an aircraft: what stage 1 needs (the track half,
/// worker::Aircraft, lives in the worker that owns the address).
#[derive(Default)]
struct AcSync {
    tracking: u32,
    /// The receivers counted in `tracking` (serve mode's rate pairs).
    seen_by: HashSet<RxKey>,
    /// Sync observations of this aircraft accepted / rejected by clock
    /// pairs, decayed: a share of rejects marks bad ADS-B positions.
    sync_ok: f32,
    sync_bad: f32,
    /// Its altitude and last ADS-B time (also kept by the worker).
    alt_ft: Option<i32>,
    alt_t: f64,
    last_adsb: f64,
    /// Own geometric-minus-baro altitude (ft, from ADS-B TC 19) and its time.
    geo_baro: Option<(f64, f64)>,
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
pub(crate) const TWO_SITE_WINDOW: f64 = 240.0;
pub(crate) const TWO_SITE_MIN_RECS: usize = 8;
pub(crate) const TWO_SITE_MIN_SPAN: f64 = 60.0;
pub(crate) const TWO_SITE_MAX_RECS: usize = 48;
pub(crate) const TWO_SITE_EVERY: f64 = 10.0;
pub(crate) const TWO_SITE_VEL_AGE: f64 = 30.0;
// Chosen on the 2026-10-24 capture with ADS-B aircraft cut to two sites
// (replay --bench-two-sites, 4,277 solutions): these keep 202, p50 340 m,
// p90 0.8 km, none over 5 km; without them p50 1.8 km, p90 70 km, 32% over
// 5 km. A low own error estimate is the strongest single sign of the right
// basin. One 10-minute capture: to be confirmed live in Bench mode.
pub(crate) const TWO_SITE_MAX_RMS: f64 = 1.3;
pub(crate) const TWO_SITE_MAX_ERR: f64 = 150.0;
pub(crate) const TWO_SITE_MIN_MARGIN: f64 = 8.0;
/// Silent receivers count as this many independent misses (offline study).
pub(crate) const TWO_SITE_N_EFF: f64 = 8.0;

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
    /// Survey corrections applied, and ones too large to apply.
    pub survey_applies: AtomicU64,
    pub survey_apply_too_big: AtomicU64,
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
    /// Receivers whose position the engine has corrected.
    pub survey_applied: usize,
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
    aircraft: HashMap<u32, AcSync>,
    actracks: HashMap<u32, AcTrack>,
    next_ac_id: u32,
    /// The coverage maps (covworker.rs), inline or on their own thread.
    coverage: CovHandle,
    cov_shared: Arc<CovShared>,
    /// When each aircraft last gave the maps a sample (their
    /// `SAMPLE_EVERY`, applied here so only samples that count are sent).
    cov_last_sample: HashMap<u32, f64>,
    rx_by_cov: HashMap<CovId, RxKey>,
    /// Where coverage maps are saved (and loaded from at start), if anywhere.
    coverage_file: Option<String>,
    coverage_saved: f64,
    /// Where the survey and the applied corrections are kept, if anywhere.
    survey_file: Option<String>,
    survey_saved: f64,
    survey_writer: crate::persist::Writer,
    /// Coverage questions from the HTTP side, answered on this thread.
    queries: Option<mpsc::Receiver<CovQuery>>,
    two_site: TwoSite,
    /// Replay bench: aircraft with ADS-B keep one pair of receivers only.
    bench_two_sites: bool,
    /// Replay: every 2-site solution of an aircraft with ADS-B, as JSON lines.
    two_site_dump: Option<std::io::BufWriter<std::fs::File>>,
    /// Regional geometric-minus-baro altitude model (altmodel.rs).
    alt_model: AltModel,
    /// Receiver position calibration from sync residuals (survey.rs), keyed
    /// by coverage identity so it survives reconnects.
    survey: Survey,
    /// Corrections applied to declared positions (east, north, up metres),
    /// by coverage identity; re-applied when a receiver reconnects, dropped
    /// when it declares a new position. In memory only: after a restart the
    /// survey learns them again in about an hour.
    applied: HashMap<CovId, [f64; 3]>,
    apply: Option<survey::ApplyCfg>,
    apply_at: f64,
    /// Serve mode: traffic selection, and when stats last went out.
    interest: Interest,
    serve_stats_at: f64,
    /// Served receivers connected (no serve work at all while 0).
    served_n: usize,
    /// Publish only positions some served receiver contributed to: the
    /// output then covers what a production shard for the served region
    /// would, while the shadow input still helps every fix.
    served_output_only: bool,
    /// The aircraft workers (worker.rs), by `worker::shard` of the address.
    workers: Pool,
    /// The altitude model changed since the workers' snapshot.
    alt_dirty: bool,
    /// The coverage snapshot the workers' 2-site path reads (its period).
    cov_epoch: Option<i64>,
    threads: bool,
}

/// The 2-site path reads coverage maps as of the last multiple of this
/// (seconds): a snapshot the workers can share (docs/MULTICORE.md, change 1;
/// the maps move with a 24 h half-life).
const COVERAGE_SNAPSHOT_S: f64 = 60.0;

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
        let workers = Pool::new(vec![Worker::new(stats.clone(), out.clone())]);
        let (coverage, cov_shared) = CovHandle::new(Coverage::default());
        Engine {
            cfg,
            stats,
            out,
            bench_out: None,
            rx_by_conn: HashMap::default(),
            rxs: HashMap::default(),
            next_key: 1,
            pairs: HashMap::default(),
            sync: HashMap::default(),
            sync_expiry: VecDeque::new(),
            groups: HashMap::default(),
            group_due: VecDeque::new(),
            holdout_due: VecDeque::new(),
            holdout_groups: HashMap::default(),
            aircraft: HashMap::default(),
            actracks: HashMap::default(),
            next_ac_id: 0,
            coverage,
            cov_shared,
            cov_last_sample: HashMap::default(),
            rx_by_cov: HashMap::default(),
            coverage_file: None,
            coverage_saved: 0.0,
            survey_file: None,
            survey_saved: 0.0,
            survey_writer: crate::persist::Writer::default(),
            queries: None,
            two_site: TwoSite::Off,
            bench_two_sites: false,
            two_site_dump: None,
            alt_model: AltModel::default(),
            survey: Survey::default(),
            applied: HashMap::default(),
            apply: None,
            apply_at: 0.0,
            interest: Interest::new(interest::Params::default(), 0x5e7e),
            serve_stats_at: 0.0,
            served_n: 0,
            served_output_only: false,
            workers,
            alt_dirty: true,
            cov_epoch: None,
            threads: false,
        }
    }

    /// Start from the coverage maps in `path`, without saving there (replay).
    pub fn load_coverage(mut self, path: &str) -> anyhow::Result<Engine> {
        let c = Coverage::load(path)?;
        self.coverage.inline_mut().expect("maps loaded before their thread starts").set(c);
        Ok(self)
    }

    /// Write every 2-site solution of an aircraft with ADS-B to `path`.
    pub fn with_two_site_dump(mut self, path: &str) -> std::io::Result<Engine> {
        self.two_site_dump = Some(std::io::BufWriter::new(std::fs::File::create(path)?));
        self.configure_workers();
        Ok(self)
    }

    /// Run `n` aircraft workers (worker.rs). The output does not depend on
    /// `n`; they run inline on the engine thread.
    pub fn with_workers(mut self, n: usize) -> Engine {
        self.workers = Pool::new((0..n.max(1)).map(|_| Worker::new(self.stats.clone(), self.out.clone())).collect());
        self.configure_workers();
        self
    }

    /// Move the workers and the coverage maps to their own threads (after
    /// the last `with_*`).
    pub fn spawn_workers(&mut self) {
        self.workers.spawn();
        self.coverage.spawn();
    }

    /// `run` moves the workers to their own threads.
    pub fn with_threads(mut self, on: bool) -> Engine {
        self.threads = on;
        self
    }

    fn configure_workers(&mut self) {
        let (bench, two_site, only, dump) = (self.bench_out.clone(), self.two_site, self.served_output_only, self.two_site_dump.is_some());
        for w in self.workers.inline_mut() {
            w.configure(bench.clone(), two_site, only, dump);
        }
    }

    fn shard(&self, icao: u32) -> usize {
        worker::shard(icao, self.workers.len())
    }

    /// Apply what the workers left so far.
    fn drain_effects(&mut self) {
        let fxs: Vec<Effects> = self.workers.effects().collect();
        for fx in fxs {
            self.apply_effects(fx);
        }
    }

    /// Wait for the workers to finish what they were sent, and apply what
    /// they left (replay end, shutdown).
    pub fn flush(&mut self) {
        self.workers.flush();
        self.drain_effects();
        self.coverage.flush();
        self.flush_two_site_dump();
    }

    /// The read-only views the workers' jobs of this tick must see: the
    /// altitude model when it changed, the coverage maps (2-site) at each
    /// `COVERAGE_SNAPSHOT_S` boundary.
    fn send_views(&mut self, now: f64) {
        if self.alt_dirty {
            self.alt_dirty = false;
            let m = Arc::new(self.alt_model.clone());
            self.workers.broadcast(|| ToWorker::AltModel(m.clone()));
        }
        if self.two_site != TwoSite::Off {
            let e = (now / COVERAGE_SNAPSHOT_S).floor() as i64;
            if self.cov_epoch != Some(e) {
                self.cov_epoch = Some(e);
                // Taken where this request sits in the maps' stream; each
                // worker waits for it where this message sits in its own.
                let (txs, rxs): (Vec<_>, Vec<_>) = (0..self.workers.len()).map(|_| std::sync::mpsc::channel()).unzip();
                self.coverage.send(ToCoverage::Snapshot(txs));
                let mut rxs = rxs.into_iter();
                self.workers.broadcast(|| ToWorker::CoverageFrom(rxs.next().unwrap()));
            }
        }
    }

    /// The workers' diagnostics, merged.
    fn diag<T>(&self, f: impl Fn(&worker::WorkerDiag) -> Vec<T>) -> Vec<T> {
        self.workers.shared().iter().flat_map(|s| f(&s.diag.lock().unwrap())).collect()
    }

    /// Apply what a worker left for stage 1.
    fn apply_effects(&mut self, fx: Effects) {
        for k in fx.used {
            if let Some(r) = self.rxs.get_mut(&k) {
                r.used += 1;
            }
        }
        for (id, now) in fx.fixes {
            self.coverage.send(ToCoverage::CountFix { id, now });
        }
        if let Some(w) = self.two_site_dump.as_mut() {
            for line in fx.dump {
                let _ = std::io::Write::write_all(w, line.as_bytes());
            }
        }
    }

    pub fn flush_two_site_dump(&mut self) {
        if let Some(w) = self.two_site_dump.as_mut() {
            let _ = std::io::Write::flush(w);
        }
    }

    /// (time, error m vs own ADS-B, accepted) of the 2-site solutions so far.
    pub fn two_site_errors(&self) -> Vec<(f64, f64, bool)> {
        self.diag(|d| d.two_site_err.iter().copied().collect())
    }

    /// (time, error m vs own ADS-B) of the positions published after them.
    pub fn two_site_published(&self) -> Vec<(f64, f64)> {
        self.diag(|d| d.two_site_pub.iter().copied().collect())
    }

    /// Save the coverage maps now (replay calls this at the end).
    pub fn save_coverage(&mut self, path: &str) -> anyhow::Result<()> {
        self.coverage.save_now(path)
    }

    /// Keep receiver coverage maps in `path` (gzip'd JSON): loaded now if it
    /// exists, saved every `COVERAGE_SAVE_EVERY` seconds.
    pub fn with_coverage_file(mut self, path: &str) -> Engine {
        match Coverage::load(path) {
            Ok(c) => {
                tracing::info!(path, maps = c.receivers(), "coverage maps loaded");
                self.coverage.inline_mut().expect("maps loaded before their thread starts").set(c);
            }
            Err(e) => tracing::info!(path, error = %e, "no coverage maps loaded: starting empty"),
        }
        self.coverage_file = Some(path.to_string());
        self
    }

    /// Keep the survey and the applied corrections in `path` (gzip'd JSON):
    /// loaded now if it exists, saved every `SURVEY_SAVE_EVERY` seconds and
    /// when the engine stops. Without it a restart starts the survey over
    /// (an hour before estimates are reliable again) and forgets every
    /// correction.
    pub fn with_survey_file(mut self, path: &str) -> Engine {
        match load_survey(path) {
            Ok((sv, applied)) => {
                tracing::info!(path, receivers = sv.receivers().count(), applied = applied.len(), "survey loaded");
                self.survey = sv;
                self.applied = applied;
            }
            Err(e) => tracing::info!(path, error = %e, "no survey loaded: starting empty"),
        }
        self.survey_file = Some(path.to_string());
        self
    }

    fn encode_survey(&self) -> anyhow::Result<Vec<u8>> {
        let applied: Vec<(String, [f64; 3])> = self.applied.iter().map(|(k, d)| (format!("{k:016x}"), *d)).collect();
        let doc = serde_json::json!({"version": 1, "survey": self.survey.to_json(), "applied": applied});
        let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast());
        std::io::Write::write_all(&mut gz, serde_json::to_string(&doc)?.as_bytes())?;
        Ok(gz.finish()?)
    }

    /// Save everything that has a file, now, on this thread (shutdown).
    fn save_state_now(&mut self) {
        if let Some(path) = self.coverage_file.clone() {
            match self.coverage.save_now(&path) {
                Ok(()) => tracing::info!(path, "coverage maps saved"),
                Err(e) => tracing::warn!(path, error = %e, "coverage maps not saved"),
            }
        }
        if let Some(path) = &self.survey_file {
            match self.encode_survey().and_then(|b| crate::persist::write_atomic(path, &b)) {
                Ok(()) => tracing::info!(path, "survey saved"),
                Err(e) => tracing::warn!(path, error = %e, "survey not saved"),
            }
        }
    }

    /// Move receivers to their reliable survey estimates (survey::ApplyCfg).
    pub fn with_survey_apply(mut self, cfg: Option<survey::ApplyCfg>) -> Engine {
        self.apply = cfg;
        self
    }

    pub fn with_served_output_only(mut self, on: bool) -> Engine {
        self.served_output_only = on;
        self.configure_workers();
        self
    }

    /// Serve mode's selection parameters (interest.rs).
    pub fn with_serve_params(mut self, p: interest::Params) -> Engine {
        self.interest = Interest::new(p, 0x5e7e);
        self
    }

    pub fn with_two_site(mut self, mode: TwoSite) -> Engine {
        self.two_site = mode;
        self.configure_workers();
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

    /// Lightcone's questions are answered by the maps, on their thread.
    fn answer(&mut self, q: CovQuery) {
        self.coverage.send(ToCoverage::Query(q));
    }

    /// Send positions of aircraft that also broadcast ADS-B to `bench`.
    pub fn with_bench_output(mut self, bench: broadcast::Sender<Arc<str>>) -> Engine {
        self.bench_out = Some(bench);
        self.configure_workers();
        self
    }

    /// Run until the event channel closes. Meant for a dedicated thread with
    /// its own current-thread runtime, away from the network workers.
    pub async fn run(mut self, mut rx: mpsc::Receiver<Event>) {
        if self.threads {
            self.spawn_workers();
        }
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
                    // Every sender gone: the process is stopping.
                    None => {
                        self.flush();
                        self.save_state_now();
                        self.workers.stop();
                        self.coverage.stop();
                        return;
                    }
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
                        self.coverage.send(ToCoverage::Place { id: r.cov, lat, lon, now });
                        self.survey.forget(r.cov);
                        // A new declared position: an old correction no
                        // longer applies.
                        self.applied.remove(&r.cov);
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
            user: Arc::from(info.user.as_str()),
            freq: info.clock.freq(),
            jitter: info.clock.jitter(),
            info,
            pos,
            peers: HashSet::default(),
            band_count: [0; 4],
            seen: HashSet::default(),
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
            requested: HashSet::default(),
            want_sync: 0,
            want_mlat: 0,
        };
        let cov = cov_id(&r.info);
        let mut r = r;
        if let Some(d) = self.applied.get(&cov) {
            r.pos = corrected(&r.enu, d);
        }
        self.coverage.send(ToCoverage::Place { id: cov, lat: r.info.lat, lon: r.info.lon, now });
        self.coverage.send(ToCoverage::Name { id: cov, who: who_of(&r.info) });
        self.rx_by_cov.insert(cov, key);
        r.cov = cov;
        self.rx_by_conn.insert(conn, key);
        if r.info.serve.is_some() {
            self.served_n += 1;
        }
        self.rxs.insert(key, r);
    }

    /// Clock pairs in key order: sums over them must not depend on the
    /// map's order (hashing.rs).
    fn pairs_sorted(&self) -> Vec<(&(RxKey, RxKey), &PairModel)> {
        let mut v: Vec<_> = self.pairs.iter().collect();
        v.sort_unstable_by_key(|(k, _)| **k);
        v
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
                self.coverage.send(ToCoverage::Unplace(r.cov));
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
        // In address order: refresh_wanted draws from its rng per aircraft.
        let mut acs: Vec<AcState> = self
            .aircraft
            .iter()
            .filter(|(_, a)| a.tracking > 0)
            .map(|(&icao, a)| AcState { icao, tracking: a.tracking as usize, last_adsb: a.last_adsb, sync_bad: sync_bad_share(a) })
            .collect();
        acs.sort_unstable_by_key(|a| a.icao);
        let newly = self.interest.refresh_wanted(now, acs.into_iter());
        if !newly.is_empty() {
            self.push_mlat(&newly);
        }
    }

    /// Ask every served receiver that sees one of `icaos` (and is not
    /// distrusted, nor already asked) for it.
    fn push_mlat(&mut self, icaos: &[u32]) {
        let mut by_rx: HashMap<RxKey, Vec<u32>> = HashMap::default();
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
            let mut v: Vec<(interest::RxKey, f32)> = a
                .seen_by
                .iter()
                .filter(|&&p| p != k)
                .filter_map(|p| rxs.get(p))
                .map(|p| (p.key, p.rate.as_ref().map_or(0.8, |m| m.get(&icao).copied().unwrap_or(0.0))))
                .collect();
            // Receiver order: the selection sums over this list.
            v.sort_unstable_by_key(|(p, _)| *p);
            v
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
        let mut reject: HashMap<RxKey, (f64, u32)> = HashMap::default();
        for ((a, b), m) in self.pairs_sorted() {
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
        worker::geo_constraint(&self.alt_model, &self.stats, icao, baro_ft, sigma, at, now)
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
        let geo_baro = self.aircraft.get_mut(&icao).map(|ac| {
            ac.last_adsb = now;
            ac.alt_ft = Some(alt_b);
            ac.alt_t = now;
            ac.geo_baro
        });
        if let Some(geo_baro) = geo_baro {
            // The track half: ADS-B truth for the 2-site path and the check.
            // (Fields, not methods: `entry` still borrows the sync table.)
            let i = worker::shard(icao, self.workers.len());
            self.workers.send(i, ToWorker::Adsb(AdsbUpdate { icao, pb, alt_b, now, tb_wall }));
            // A validated ADS-B position: who hears this aircraft, and who
            // in range does not, is coverage evidence.
            let llh = ecef_to_llh(pb);
            // A validated position with a recent own difference: a sample for
            // the altitude model (at its barometric level).
            if let Some((d, t)) = geo_baro {
                if now - t <= GEO_SAMPLE_AGE {
                    self.alt_model.insert(icao, llh.lat, llh.lon, alt_b as f64, d, now);
                    self.alt_dirty = true;
                }
            }
            if self.cov_last_sample.get(&icao).is_none_or(|t| now - t >= coverage::SAMPLE_EVERY) {
                self.cov_last_sample.insert(icao, now);
                let seeing = seeing_of(&self.aircraft, &self.rxs, &self.rx_by_cov, icao);
                self.coverage.send(ToCoverage::Observe { icao, lat: llh.lat, lon: llh.lon, alt_m: alt_b as f64 * 0.3048, now, seeing });
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

    /// Move the receivers whose reliable estimate is off by more than
    /// `min_m` (and 5 sigma) to it, the largest first. The estimate is
    /// relative to where the receiver is now, so corrections add up; its
    /// survey starts over from the new place, and its clock pairs
    /// re-converge (they were fitted to the old one).
    fn apply_survey(&mut self, cfg: survey::ApplyCfg) {
        let mut cands: Vec<(f64, CovId, [f64; 3])> = Vec::new();
        for (c, e) in self.survey_estimates() {
            let h = e.horizontal_m();
            if !e.reliable() || h <= cfg.min_m || h <= 5.0 * e.sd[0].hypot(e.sd[1]) {
                continue;
            }
            let prev = self.applied.get(&c).copied().unwrap_or([0.0; 3]);
            let total = [prev[0] + e.d[0], prev[1] + e.d[1], prev[2] + e.d[2]];
            if total[0].hypot(total[1]) > cfg.max_m {
                self.stats.survey_apply_too_big.fetch_add(1, Ordering::Relaxed);
                continue;
            }
            cands.push((h, c, total));
        }
        cands.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        for (_, c, total) in cands.into_iter().take(cfg.per_round) {
            self.applied.insert(c, total);
            self.survey.forget(c);
            self.stats.survey_applies.fetch_add(1, Ordering::Relaxed);
            if let Some(k) = self.rx_by_cov.get(&c).copied() {
                self.drop_pairs(k);
                if let Some(r) = self.rxs.get_mut(&k) {
                    r.pos = corrected(&r.enu, &total);
                }
            }
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
                "applied_m": self.applied.get(&c).map(|a| [a[0].round(), a[1].round(), a[2].round()]),
            }));
        }
        {
            let mut snap = self.stats.snapshot.lock().unwrap();
            snap.survey_receivers = n;
            snap.survey_reliable = reliable;
            snap.survey_reliable_over_100m = over100;
            snap.survey_applied = self.applied.len();
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
        self.drain_effects();
        self.send_views(now);
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

    /// What norm::normalize needs for this group, read from the clock pairs
    /// now (worker::Frozen).
    fn freeze(&self, keys: Vec<RxKey>, nodes: Vec<Node>, copies: Vec<Copy>, now: f64) -> Frozen {
        // Representative local time per node: its first copy.
        let mut s_of = vec![f64::NAN; nodes.len()];
        for c in &copies {
            if s_of[c.node].is_nan() {
                s_of[c.node] = c.s;
            }
        }
        // The members' edges, once: they decide on bridges and feed the solve.
        let k = keys.len();
        let mut cache = vec![None; k * k];
        for i in 0..k {
            for j in (i + 1)..k {
                cache[i * k + j] = self.edge(&keys, &s_of, i, j, now);
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
        let unbridged = |keys, nodes, copies, cache, comp| Frozen { keys, nodes, copies, nodes_all: None, edges: cache, comp };
        if bridges.is_empty() {
            return unbridged(keys, nodes, copies, cache, comp);
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
            return unbridged(keys, nodes, copies, cache, comp);
        }
        // normalize asks for every i < j: member pairs from the cache, the
        // rest now.
        let n = keys_all.len();
        let mut edges = vec![None; n * n];
        for i in 0..n {
            for j in (i + 1)..n {
                edges[i * n + j] = if j < k { cache[i * k + j] } else { self.edge(&keys_all, &s_of, i, j, now) };
            }
        }
        Frozen { keys, nodes, copies, nodes_all: Some(nodes_all), edges, comp }
    }

    fn normalize(&self, keys: &[RxKey], nodes: &[Node], copies: &[Copy], now: f64) -> Option<norm::Normalized> {
        self.freeze(keys.to_vec(), nodes.to_vec(), copies.to_vec(), now).normalize(&self.stats)
    }

    /// The receivers of a job, as its worker needs them.
    fn rx_refs(&self, keys: &[RxKey]) -> Vec<RxRef> {
        keys.iter()
            .filter_map(|k| self.rxs.get(k))
            .map(|r| RxRef {
                key: r.key,
                cov: r.cov,
                user: r.user.clone(),
                serve: r.info.serve,
                tx: r.info.serve.map(|_| r.tx.clone()),
            })
            .collect()
    }

    /// Coverage ids of the receivers that report seeing `icao`, as the 2-site
    /// path and the coverage maps ask it of each receiver near the aircraft.
    fn seeing_covs(&self, icao: u32) -> Vec<CovId> {
        seeing_of(&self.aircraft, &self.rxs, &self.rx_by_cov, icao)
    }

    /// A due group: Mode A/C here; Mode S frozen into a job for the worker
    /// that owns the aircraft.
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
        let commb = known && (d.df == 20 || d.df == 21) && bytes.len() == 14;
        let mut job = GroupJob {
            icao: d.address,
            bytes,
            d,
            t_rx,
            first_wall: g.first_wall,
            now,
            commb,
            update: false,
            copies_n: 0,
            frozen: None,
            rx: Vec::new(),
            seeing: Vec::new(),
        };
        if !known || g.copies.len() < 2 {
            self.stats.groups_unknown_ac.fetch_add(1, Ordering::Relaxed);
            if commb {
                self.run_group(job);
            }
            return;
        }
        job.update = true;
        {
            let ac = self.aircraft.get_mut(&job.icao).unwrap();
            if let Some(a) = job.d.altitude_ft {
                if (-1500..=75_000).contains(&a) {
                    ac.alt_ft = Some(a);
                    ac.alt_t = now;
                }
            }
        }
        if self.bench_two_sites {
            let ac = self.aircraft.get_mut(&job.icao).unwrap();
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
        job.copies_n = g.copies.len();
        let (keys, nodes, copies) = self.build(&g);
        if keys.len() >= 2 {
            job.rx = self.rx_refs(&keys);
            if self.two_site != TwoSite::Off {
                job.seeing = self.seeing_covs(job.icao);
            }
            job.frozen = Some(self.freeze(keys, nodes, copies, now));
        }
        self.run_group(job);
    }

    fn run_group(&mut self, job: GroupJob) {
        let i = self.shard(job.icao);
        self.workers.send(i, ToWorker::Group(job));
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
            let (meas, cov) = worker::sub_problem(&normd, &cl, &copies, &nodes);
            // The same reply answers a Mode A or a Mode C interrogation: try
            // the Mode C reading (altitude-constrained, 3+ receivers) and the
            // Mode A reading (free altitude, 4+ receivers).
            let mut candidates: Vec<(Fix, bool)> = Vec::new();
            let centroid = worker::guesses(None, &meas, None, t);
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
            //    same altitude (Mode C), within 15 km of its track. The
            //    nearest one (then the lowest address), not the map's first:
            //    the result must not depend on the map's order (hashing.rs).
            //    The workers own the tracks: each answers with its nearest
            //    match once it has done the jobs sent before this question,
            //    so they see the tracks as they are now.
            let mut assoc = None;
            let cands: Vec<ModeAcCand> = candidates.iter().map(|(f, mode_c)| ModeAcCand { pos: f.pos, mode_c: *mode_c }).collect();
            let (altitude_ft, squawk) = (mac.altitude_ft, mac.squawk);
            let answers = self.workers.ask(|reply| ToWorker::ModeAcQuery { cands: cands.clone(), altitude_ft, squawk, t, reply });
            for (i, (_, mode_c)) in candidates.iter().enumerate() {
                let best = answers.iter().filter_map(|a| a[i]).min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
                if let Some((_, icao)) = best {
                    assoc = Some((icao, *mode_c));
                    break;
                }
            }
            if let Some((icao, mode_c)) = assoc {
                self.stats.modeac_assoc.fetch_add(1, Ordering::Relaxed);
                let alt = if mode_c { mac.altitude_ft.map(|ft| self.geo_constraint(None, ft, 100.0, at, now)) } else { None };
                let i = self.shard(icao);
                self.workers.send(i, ToWorker::ModeAcUpdate { icao, meas, cov, alt, t });
                continue;
            }
            // 2) Mode A/C-only tracks: same code, consistent position.
            let mut taken = false;
            // In id order: the first track that takes the fix keeps it.
            let mut ids: Vec<u32> = self.actracks.keys().copied().collect();
            ids.sort_unstable();
            for id in ids {
                let at = self.actracks.get_mut(&id).unwrap();
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
        // The model's estimate now; the worker turns it into the constraint
        // (and counts it) only if the group normalises.
        let at = ecef_to_llh(truth);
        let alt_est = self.alt_model.estimate(at.lat, at.lon, alt_ft as f64, now, Some(icao));
        let frozen = self.freeze(keys, nodes, copies, now);
        let i = self.shard(icao);
        self.workers.send(i, ToWorker::Holdout(HoldoutJob { frozen, truth, alt_ft, alt_est, now }));
    }

    // --------------------------------------------------------- maintenance --

    fn expire(&mut self, now: f64) {
        self.workers.broadcast(|| ToWorker::Expire(now));
        while let Some(&(t, key)) = self.sync_expiry.front() {
            if now - t < SYNC_TTL {
                break;
            }
            self.sync_expiry.pop_front();
            self.sync.remove(&key);
        }
    }

    fn housekeeping(&mut self, now: f64) {
        self.survey.sweep(now);
        if let Some(cfg) = self.apply {
            if now - self.apply_at >= cfg.every_s {
                self.apply_at = now;
                self.apply_survey(cfg);
            }
        }
        self.render_survey();
        // Sync quality per aircraft over roughly the last minute.
        for a in self.aircraft.values_mut() {
            a.sync_ok *= 0.8;
            a.sync_bad *= 0.8;
        }
        self.send_serve_stats(now);
        // Encoded here, written by another thread: a slow volume must not
        // hold up the engine (persist.rs).
        if let Some(path) = self.coverage_file.clone() {
            if now - self.coverage_saved >= COVERAGE_SAVE_EVERY {
                self.coverage_saved = now;
                self.coverage.send(ToCoverage::Save(path));
            }
        }
        if let Some(path) = &self.survey_file {
            if now - self.survey_saved >= SURVEY_SAVE_EVERY {
                self.survey_saved = now;
                match self.encode_survey() {
                    Ok(b) => self.survey_writer.spawn(path.clone(), b),
                    Err(e) => tracing::warn!(path, error = %e, "survey not encoded"),
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
        let mut rate: HashMap<RxKey, (f64, u32, Vec<f64>)> = HashMap::default();
        for ((a, b), m) in self.pairs_sorted() {
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
        self.alt_dirty = true;
        // Tracks: the workers drop stale ones and say which aircraft still
        // have one (once they have done every job sent so far); an aircraft
        // nobody sees and without a track is forgotten by both halves.
        let tracked: HashSet<u32> = self.workers.ask(|reply| ToWorker::Housekeeping { now, reply }).into_iter().flatten().collect();
        let mut gone = Vec::new();
        self.aircraft.retain(|icao, a| {
            let keep = a.tracking > 0 || tracked.contains(icao);
            if !keep {
                gone.push(*icao);
            }
            keep
        });
        let mut by_worker: Vec<Vec<u32>> = vec![Vec::new(); self.workers.len()];
        for icao in gone {
            by_worker[self.shard(icao)].push(icao);
        }
        for (i, v) in by_worker.into_iter().enumerate() {
            if !v.is_empty() {
                self.workers.send(i, ToWorker::Forget(v));
            }
        }
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
        self.diag(|d| d.adsb_err.iter().map(|x| (x.1, x.2, x.3)).collect())
    }

    pub(crate) fn publish_at(&self, queue_depth: usize, now: f64) {
        let mut errs: Vec<f64> = self.diag(|d| d.holdout_err.iter().map(|x| x.1).collect());
        errs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let q = |p: f64| if errs.is_empty() { 0.0 } else { errs[((errs.len() - 1) as f64 * p) as usize] };
        let mut s = self.stats.snapshot.lock().unwrap();
        s.receivers = self.rxs.len();
        s.coverage_maps = self.cov_shared.maps.load(Ordering::Relaxed);
        s.coverage_samples = self.cov_shared.samples.load(Ordering::Relaxed);
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
        s.tracks = self.workers.shared().iter().map(|w| w.tracks.load(Ordering::Relaxed)).sum();
        s.tracks_confirmed = self.workers.shared().iter().map(|w| w.confirmed.load(Ordering::Relaxed)).sum();
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
        let two_err = self.diag(|d| d.two_site_err.iter().copied().collect());
        s.two_site_all = pct(two_err.iter().filter(|x| now - x.0 < 600.0).map(|x| x.1).collect());
        s.two_site_ok = pct(two_err.iter().filter(|x| now - x.0 < 600.0 && x.2).map(|x| x.1).collect());
        s.two_site_pub = pct(self.diag(|d| d.two_site_pub.iter().filter(|x| now - x.0 < 600.0).map(|x| x.1).collect()));
        let qv = |v: &mut Vec<f64>, p: f64| {
            if v.is_empty() {
                return 0.0;
            }
            v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            v[((v.len() - 1) as f64 * p) as usize]
        };
        let adsb_err = self.diag(|d| d.adsb_err.iter().copied().collect());
        let mut buckets = Vec::new();
        for (name, lo, hi) in [("2", 2, 2), ("3", 3, 3), ("4-5", 4, 5), ("6-9", 6, 9), ("10+", 10, usize::MAX)] {
            let sel: Vec<(f64, usize, Option<f64>, Option<f64>)> = adsb_err.iter().filter(|x| x.1 >= lo && x.1 <= hi).copied().collect();
            let mut f: Vec<f64> = sel.iter().filter_map(|x| x.2).collect();
            let mut o: Vec<f64> = sel.iter().filter_map(|x| x.3).collect();
            buckets.push((name.to_string(), sel.len(), qv(&mut f, 0.5), qv(&mut f, 0.9), qv(&mut o, 0.5), qv(&mut o, 0.9)));
        }
        s.adsb_check = buckets;
        drop(s);
        // The workers' last cases, oldest first.
        let mut diag: Vec<Diag> = self.diag(|d| d.diag_bad.iter().cloned().collect());
        diag.sort_by(|a, b| a.t.total_cmp(&b.t));
        *self.stats.debug.lock().unwrap() = diag.split_off(diag.len().saturating_sub(400));
    }
}

/// Coverage ids (sorted) of the receivers that report seeing `icao`: for each
/// receiver id, `rx_by_cov` gives its receiver and that receiver's `seen`
/// holds the aircraft.
fn seeing_of(aircraft: &HashMap<u32, AcSync>, rxs: &HashMap<RxKey, Rx>, rx_by_cov: &HashMap<CovId, RxKey>, icao: u32) -> Vec<CovId> {
    let Some(a) = aircraft.get(&icao) else { return Vec::new() };
    let mut v: Vec<CovId> = a
        .seen_by
        .iter()
        .filter_map(|k| rxs.get(k))
        .filter(|r| r.seen.contains(&icao) && rx_by_cov.get(&r.cov) == Some(&r.key))
        .map(|r| r.cov)
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Share of an aircraft's recent sync observations its clock pairs rejected
/// (0 until there are enough of them).
fn sync_bad_share(a: &AcSync) -> f64 {
    let n = a.sync_ok + a.sync_bad;
    if n < SYNC_QUALITY_MIN_OBS {
        0.0
    } else {
        (a.sync_bad / n) as f64
    }
}

/// A survey file written by `encode_survey`: (the survey's sums, applied
/// corrections).
fn load_survey(path: &str) -> anyhow::Result<(Survey, HashMap<CovId, [f64; 3]>)> {
    let mut txt = String::new();
    std::io::Read::read_to_string(&mut flate2::read::GzDecoder::new(std::fs::File::open(path)?), &mut txt)?;
    let doc: serde_json::Value = serde_json::from_str(&txt)?;
    let mut sv = Survey::default();
    sv.load_json(&doc["survey"]);
    let applied: Vec<(String, [f64; 3])> = serde_json::from_value(doc["applied"].clone()).unwrap_or_default();
    let applied = applied.into_iter().filter_map(|(k, d)| u64::from_str_radix(&k, 16).ok().map(|k| (k, d))).collect();
    Ok((sv, applied))
}

/// The declared position (`enu`'s origin) moved by `d` (east, north, up m).
fn corrected(enu: &Enu, d: &[f64; 3]) -> Ecef {
    let (o, r) = (enu.origin, &enu.r);
    [
        o[0] + d[0] * r[0][0] + d[1] * r[1][0] + d[2] * r[2][0],
        o[1] + d[0] * r[0][1] + d[1] * r[1][1] + d[2] * r[2][1],
        o[2] + d[0] * r[0][2] + d[1] * r[1][2] + d[2] * r[2][2],
    ]
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
    use crate::types::{ClockType, ResultFormat};
    use crate::worker::adsb_at;

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

    #[test]
    fn corrections_move_the_receiver_and_survive_a_reconnect() {
        let mut e = engine().with_survey_apply(Some(survey::ApplyCfg::default()));
        let a = add_rx(&mut e, 1, 9.0);
        let b = add_rx(&mut e, 2, 9.5);
        let off: HashMap<_, _> = [(a, 0.0), (b, 1e-6)].into_iter().collect();
        add_pair(&mut e, a, b, &off, 100.0);
        let declared = e.rxs[&a].pos;
        let cov = e.rxs[&a].cov;
        e.applied.insert(cov, [300.0, -400.0, 0.0]);
        // Applying moves it and drops its clock pairs.
        let total = e.applied[&cov];
        e.drop_pairs(a);
        let enu = e.rxs[&a].enu;
        e.rxs.get_mut(&a).unwrap().pos = corrected(&enu, &total);
        assert!((dist(&e.rxs[&a].pos, &declared) - 500.0).abs() < 1e-6);
        assert!(e.pairs.is_empty());
        // A reconnect comes back at the corrected place.
        let info = e.rxs[&a].info.clone();
        e.disconnect(1);
        let (tx, _) = mpsc::channel(1);
        e.connect(7, info, tx, 200.0);
        let k = e.rx_by_conn[&7];
        assert!((dist(&e.rxs[&k].pos, &declared) - 500.0).abs() < 1e-6);
        let _ = b;
    }

    #[test]
    fn corrected_is_east_north_up() {
        let pos = llh_to_ecef(Llh { lat: 45.0, lon: 9.0, alt: 100.0 });
        let enu = Enu::at(pos);
        let p = corrected(&enu, &[100.0, 200.0, 0.0]);
        let l = enu.to_enu(&p);
        assert!((l[0] - 100.0).abs() < 1e-6 && (l[1] - 200.0).abs() < 1e-6 && l[2].abs() < 1e-6, "{l:?}");
    }

    #[test]
    fn survey_and_corrections_survive_a_restart() {
        let dir = std::env::temp_dir().join(format!("mlat-ng-survey-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("survey.json.gz").to_string_lossy().to_string();
        let mut e = engine().with_survey_file(&path);
        // A receiver with closed windows, and a correction.
        for w in 0..3 {
            for i in 0..20 {
                let t = w as f64 * 70.0 + i as f64;
                let az = (i * 37 % 360) as f64;
                let u = [az.to_radians().sin() * 0.99, az.to_radians().cos() * 0.99, 0.1];
                e.survey.observe(42, 7, t, t, 5.0e14 + 10.0 * t + u[0] * 150.0, u);
            }
        }
        e.survey.sweep(1000.0);
        e.applied.insert(42, [120.0, -80.0, 0.0]);
        let before = e.survey.estimate(42).unwrap();
        e.save_state_now();
        let e2 = engine().with_survey_file(&path);
        let after = e2.survey.estimate(42).unwrap();
        assert_eq!(e2.applied.get(&42), Some(&[120.0, -80.0, 0.0]));
        assert!((after.d[0] - before.d[0]).abs() < 1e-9 && after.windows == before.windows, "{before:?} {after:?}");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
