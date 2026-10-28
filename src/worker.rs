//! Aircraft workers: the part of the engine that makes positions.
//!
//! Step 1 of docs/MULTICORE.md. The engine (stage 1) keeps everything that
//! must stay in one order: receivers, clock pairs, sync points, grouping,
//! and the sync half of each aircraft (who sees it, its ADS-B timing, the
//! serve mode's view). At the tick it freezes each due group into a job that
//! carries what processing it will read from stage 1: nodes, the clock edges
//! as `Engine::normalize` would read them at that moment, the receivers'
//! identities. A worker owns the track half of its aircraft (sharded by
//! address) and does the rest: normalisation, clustering, solving, tracks,
//! 2-site, the ADS-B check, output.
//!
//! A worker reads nothing of stage 1 but its jobs and messages, plus two
//! read-only views (`Cx`): the altitude model and the coverage maps. Its
//! writes to stage 1 are commutative counters, returned as `Effects`. So
//! the output is the same for any number of workers, and does not depend on
//! when a worker runs a job as long as each worker sees its messages in
//! order.
//!
//! Step 2: the read-only views reach the workers as messages too (`ToWorker`),
//! in stream order: the altitude model as a snapshot at each tick that follows
//! a change, the coverage maps for the 2-site path as a snapshot every
//! `COVERAGE_SNAPSHOT_S`. With that a worker can run on its own thread
//! (`Pool::spawn`), and inline and threaded workers run the same code on the
//! same messages.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{mpsc as std_mpsc, Arc, Mutex};
use std::time::Instant;

use nalgebra::{DMatrix, Matrix3, Vector3};
use tokio::sync::{broadcast, mpsc};

use crate::altmodel::{self, AltModel};
use crate::coverage::{CovId, Coverage};
use crate::engine::{Diag, Stats, TwoSite, ADSB_HIST_S, ALT_SIGMA_M, TWO_SITE_EVERY, TWO_SITE_MAX_ERR, TWO_SITE_MAX_RECS,
    TWO_SITE_MAX_RMS, TWO_SITE_MIN_MARGIN, TWO_SITE_MIN_RECS, TWO_SITE_MIN_SPAN, TWO_SITE_N_EFF, TWO_SITE_VEL_AGE, TWO_SITE_WINDOW};
use crate::geo::{dist, ecef_to_llh, llh_to_ecef, Ecef, Enu, Llh, C_AIR, FT_TO_M};
use crate::hashing::HashMap;
use crate::modes;
use crate::norm::{self, Copy, Node};
use crate::output::{result_ecef, result_old, sbs_line, ClientResult, SbsFix};
use crate::solver::{self, Fix, Meas, SolveInput};
use crate::track::{Gate, Track};
use crate::twosite::{self, Rec2};
use crate::types::{ResultFormat, ServeOpts, ToClient};

pub(crate) type RxKey = u64;

/// The track half of an aircraft (the sync half is `engine::AcSync`).
#[derive(Default)]
pub(crate) struct Aircraft {
    /// Copies of the sync half's fields, updated by the same events.
    pub alt_ft: Option<i32>,
    pub alt_t: f64,
    pub last_adsb: f64,
    pub squawk: Option<u16>,
    pub callsign: Option<String>,
    pub track: Option<Track>,
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
}

/// A receiver of a job, as the worker needs it.
#[derive(Clone)]
pub(crate) struct RxRef {
    pub key: RxKey,
    pub cov: CovId,
    pub user: Arc<str>,
    pub serve: Option<ServeOpts>,
    /// Only for served receivers (result lines).
    pub tx: Option<mpsc::Sender<ToClient>>,
}

/// What `norm::normalize` needs for a group, read from the clock pairs when
/// the group was due: the members' edges and, when their own graph is
/// disconnected, bridge receivers and every edge involving them (norm.rs,
/// "Bridges"). normalize asks for every pair i < j, so the table holds
/// exactly what it would have read.
pub(crate) struct Frozen {
    pub keys: Vec<RxKey>,
    pub nodes: Vec<Node>,
    pub copies: Vec<Copy>,
    /// Members then bridges (only when bridged).
    pub nodes_all: Option<Vec<Node>>,
    /// Edge (i, j), i < j, at `i * n + j`, n = all nodes.
    pub edges: Vec<Option<(f64, f64)>>,
    /// Members' clock-graph components (for the bridging counters).
    pub comp: Vec<usize>,
}

impl Frozen {
    pub fn normalize(&self, stats: &Stats) -> Option<norm::Normalized> {
        let Some(nodes_all) = &self.nodes_all else {
            let n = self.nodes.len();
            return norm::normalize(&self.nodes, &self.copies, |i, j| self.edges[i * n + j]);
        };
        let n = nodes_all.len();
        let normd = norm::normalize(nodes_all, &self.copies, |i, j| self.edges[i * n + j])?;
        let mut per_comp: HashMap<usize, usize> = HashMap::default();
        for c in &self.copies {
            *per_comp.entry(self.comp[c.node]).or_default() += 1;
        }
        let own = per_comp.values().copied().max().unwrap_or(0);
        stats.groups_bridged.fetch_add(1, Ordering::Relaxed);
        stats.bridge_receivers.fetch_add((n - self.nodes.len()) as u64, Ordering::Relaxed);
        stats.bridged_copies.fetch_add(normd.copies.len().saturating_sub(own) as u64, Ordering::Relaxed);
        Some(normd)
    }
}

/// A due Mode S group.
pub(crate) struct GroupJob {
    pub icao: u32,
    pub bytes: Vec<u8>,
    pub d: modes::Decoded,
    /// The group's time by its receivers' clocks (WallMap), when mapped.
    pub t_rx: Option<f64>,
    pub first_wall: f64,
    pub now: f64,
    /// Comm-B velocity to read from the frame.
    pub commb: bool,
    /// Known aircraft with at least two copies: decoded fields go to the
    /// aircraft, then the group is solved if `frozen`.
    pub update: bool,
    /// Copies in the group (diagnostics).
    pub copies_n: usize,
    pub frozen: Option<Frozen>,
    pub rx: Vec<RxRef>,
    /// Coverage ids of the receivers that report seeing the aircraft (2-site).
    pub seeing: Vec<CovId>,
}

/// A due holdout group: solved against its own ADS-B position.
pub(crate) struct HoldoutJob {
    pub frozen: Frozen,
    pub truth: Ecef,
    pub alt_ft: i32,
    /// The altitude model's estimate at the truth, when the group was due.
    pub alt_est: Option<f64>,
    pub now: f64,
}

/// A validated ADS-B position of an aircraft (from a sync point).
pub(crate) struct AdsbUpdate {
    pub icao: u32,
    pub pb: Ecef,
    pub alt_b: i32,
    pub now: f64,
    /// The position's time by the receiving receiver's clock (WallMap).
    pub tb_wall: Option<f64>,
}

/// What a worker leaves for stage 1: counters, and replay dump lines.
#[derive(Default)]
pub(crate) struct Effects {
    pub used: Vec<RxKey>,
    pub fixes: Vec<(CovId, f64)>,
    pub dump: Vec<String>,
}

impl Effects {
    fn is_empty(&self) -> bool {
        self.used.is_empty() && self.fixes.is_empty() && self.dump.is_empty()
    }
}

/// A worker's diagnostics, read by stage 1 for /metrics and the replay
/// summary.
#[derive(Default)]
pub(crate) struct WorkerDiag {
    pub holdout_err: VecDeque<(f64, f64)>,
    /// (time, receivers, fix error, published error) of ADS-B-checked updates
    pub adsb_err: VecDeque<(f64, usize, Option<f64>, Option<f64>)>,
    pub diag_bad: VecDeque<Diag>,
    /// (time, error m vs own ADS-B, accepted) of 2-site solutions.
    pub two_site_err: VecDeque<(f64, f64, bool)>,
    /// (time, error m vs own ADS-B) of positions published after a 2-site fix.
    pub two_site_pub: VecDeque<(f64, f64)>,
}

#[derive(Default)]
pub(crate) struct WorkerShared {
    pub diag: Mutex<WorkerDiag>,
    /// Tracks and confirmed tracks, as of the last `Expire`.
    pub tracks: AtomicUsize,
    pub confirmed: AtomicUsize,
}

/// A Mode A/C solution looking for a Mode S track (engine, process_modeac).
#[derive(Clone, Copy)]
pub(crate) struct ModeAcCand {
    pub pos: Ecef,
    pub mode_c: bool,
}

/// What stage 1 sends a worker, in the order it must be applied.
pub(crate) enum ToWorker {
    Adsb(AdsbUpdate),
    Group(GroupJob),
    Holdout(HoldoutJob),
    /// The altitude model's changes, as stage 1 makes them: each worker keeps
    /// its own copy, the same as stage 1's at every tick.
    AltInsert { icao: u32, lat: f64, lon: f64, baro_ft: f64, delta_ft: f64, t: f64 },
    AltExpire(f64),
    /// Stage 1's model fingerprint: the copy must match it.
    AltCheck(u64),
    /// The coverage maps the 2-site path reads until the next one, as the
    /// maps' thread hands them over (covworker.rs).
    CoverageFrom(std_mpsc::Receiver<Arc<Coverage>>),
    /// For each candidate (in order), this worker's nearest matching track:
    /// (distance, address).
    ModeAcQuery { cands: Vec<ModeAcCand>, altitude_ft: Option<i32>, squawk: u16, t: f64, reply: std_mpsc::Sender<Vec<Option<(f64, u32)>>> },
    /// A Mode A/C solution associated with one of its tracks.
    ModeAcUpdate { icao: u32, meas: Vec<Meas>, cov: DMatrix<f64>, alt: Option<(f64, f64)>, t: f64 },
    /// Drop stale tracks; reply with the aircraft that still have one.
    Housekeeping { now: f64, reply: std_mpsc::Sender<Vec<u32>> },
    Forget(Vec<u32>),
    Expire(f64),
    /// Reply once everything before it is done.
    Flush(std_mpsc::Sender<()>),
}

/// What processing a job may read besides the worker's own state.
pub(crate) struct Cx<'a> {
    pub alt_model: &'a AltModel,
    pub coverage: &'a Coverage,
    pub rx: &'a [RxRef],
    pub seeing: &'a [CovId],
    pub fx: &'a mut Effects,
}

impl Cx<'_> {
    fn rx(&self, k: &RxKey) -> Option<&RxRef> {
        self.rx.iter().find(|r| r.key == *k)
    }
}

pub(crate) struct Worker {
    pub(crate) aircraft: HashMap<u32, Aircraft>,
    stats: Arc<Stats>,
    out: broadcast::Sender<Arc<str>>,
    bench_out: Option<broadcast::Sender<Arc<str>>>,
    two_site: TwoSite,
    served_output_only: bool,
    /// Replay: 2-site dump lines go to `Effects::dump`.
    dump: bool,
    shared: Arc<WorkerShared>,
    /// The read-only views, as of the last message that carried them.
    alt: Arc<AltModel>,
    coverage: Arc<Coverage>,
}

/// The worker that owns aircraft `icao`, of `n`.
pub(crate) fn shard(icao: u32, n: usize) -> usize {
    (crate::engine::fxhash(&icao.to_le_bytes()) % n as u64) as usize
}

impl Worker {
    pub(crate) fn new(stats: Arc<Stats>, out: broadcast::Sender<Arc<str>>) -> Worker {
        Worker {
            aircraft: HashMap::default(),
            stats,
            out,
            bench_out: None,
            two_site: TwoSite::Off,
            served_output_only: false,
            dump: false,
            shared: Arc::new(WorkerShared::default()),
            alt: Arc::new(AltModel::default()),
            coverage: Arc::new(Coverage::default()),
        }
    }

    pub(crate) fn shared(&self) -> Arc<WorkerShared> {
        self.shared.clone()
    }

    /// Apply one message.
    pub(crate) fn handle(&mut self, msg: ToWorker, fx: &mut Effects) {
        match msg {
            ToWorker::Adsb(u) => self.adsb(u, fx),
            ToWorker::Group(job) => self.group(job, fx),
            ToWorker::Holdout(job) => self.holdout(job),
            // Between jobs the worker holds the only reference: no copy.
            ToWorker::AltInsert { icao, lat, lon, baro_ft, delta_ft, t } => Arc::make_mut(&mut self.alt).insert(icao, lat, lon, baro_ft, delta_ft, t),
            ToWorker::AltExpire(now) => Arc::make_mut(&mut self.alt).expire(now),
            ToWorker::AltCheck(fp) => {
                if self.alt.fingerprint() != fp {
                    self.stats.alt_replica_mismatch.fetch_add(1, Ordering::Relaxed);
                    tracing::warn!("altitude model copy differs from stage 1's");
                }
            }
            ToWorker::CoverageFrom(rx) => self.coverage = rx.recv().expect("coverage thread gone"),
            ToWorker::ModeAcQuery { cands, altitude_ft, squawk, t, reply } => {
                let _ = reply.send(cands.iter().map(|c| self.modeac_match(c, altitude_ft, squawk, t)).collect());
            }
            ToWorker::ModeAcUpdate { icao, meas, cov, alt, t } => {
                let ac = self.aircraft.get_mut(&icao).unwrap();
                if let Some(tr) = ac.track.as_mut() {
                    if tr.update_tdoa(&meas, &cov, alt, t) == Gate::Accepted {
                        self.stats.track_tdoa_updates.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
            ToWorker::Housekeeping { now, reply } => {
                let _ = reply.send(self.stale_tracks(now));
            }
            ToWorker::Forget(v) => {
                for icao in v {
                    self.aircraft.remove(&icao);
                }
            }
            ToWorker::Expire(now) => self.expire(now),
            ToWorker::Flush(reply) => {
                let _ = reply.send(());
            }
        }
    }

    /// Mode A/C association (engine, process_modeac): the nearest of its
    /// tracks that matches by identity (Mode A) or altitude (Mode C), within
    /// 15 km, then the lowest address.
    fn modeac_match(&self, c: &ModeAcCand, altitude_ft: Option<i32>, squawk: u16, t: f64) -> Option<(f64, u32)> {
        let mut best: Option<(f64, u32)> = None;
        for (icao, ac) in &self.aircraft {
            let Some(tr) = &ac.track else { continue };
            if t - tr.last_update() > 30.0 {
                continue;
            }
            let matches = if c.mode_c {
                match (ac.alt_ft, altitude_ft) {
                    (Some(a), Some(b)) => (a - b).abs() <= 300,
                    _ => false,
                }
            } else {
                ac.squawk == Some(squawk)
            };
            let d = dist(&tr.predict(t).0, &c.pos);
            if matches && d < 15e3 && best.is_none_or(|(bd, bi)| (d, *icao) < (bd, bi)) {
                best = Some((d, *icao));
            }
        }
        best
    }

    pub(crate) fn configure(&mut self, bench_out: Option<broadcast::Sender<Arc<str>>>, two_site: TwoSite, served_output_only: bool, dump: bool) {
        self.bench_out = bench_out;
        self.two_site = two_site;
        self.served_output_only = served_output_only;
        self.dump = dump;
    }

    /// A validated ADS-B position of one of its aircraft.
    fn adsb(&mut self, u: AdsbUpdate, fx: &mut Effects) {
        let AdsbUpdate { icao, pb, alt_b, now, tb_wall } = u;
        let ac = self.aircraft.entry(icao).or_default();
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
            if !ac.two.is_empty() && self.dump {
                let l = ecef_to_llh(pb);
                let line = serde_json::json!({"adsb": format!("{icao:06x}"), "t": tw, "lat": l.lat, "lon": l.lon});
                fx.dump.push(format!("{line}\n"));
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
    }

    /// A due Mode S group of one of its aircraft.
    fn group(&mut self, job: GroupJob, fx: &mut Effects) {
        let GroupJob { icao, bytes, d, t_rx, first_wall, now, commb, update, copies_n, frozen, rx, seeing } = job;
        let (alt, coverage) = (self.alt.clone(), self.coverage.clone());
        let cx = &mut Cx { alt_model: &alt, coverage: &coverage, rx: &rx, seeing: &seeing, fx };
        if commb {
            self.commb_velocity(icao, &bytes, t_rx.unwrap_or(first_wall));
        }
        if !update {
            return;
        }
        {
            let ac = self.aircraft.entry(icao).or_default();
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
        let Some(fz) = frozen else { return };
        let Some(normd) = fz.normalize(&self.stats) else {
            self.stats.groups_no_norm.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let (keys, nodes, copies) = (&fz.keys, &fz.nodes, &fz.copies);
        let mut cls = norm::clusters(&normd, copies, nodes, 2);
        cls.sort_by(|a, b| normd.t[a[0]].partial_cmp(&normd.t[b[0]]).unwrap_or(std::cmp::Ordering::Equal));
        let t_first = cls.first().map(|c| normd.t[c[0]]).unwrap_or(0.0);
        let n_clusters = cls.len();
        for cl in cls {
            self.stats.clusters.fetch_add(1, Ordering::Relaxed);
            let t_wall = first_wall + (normd.t[cl[0]] - t_first);
            let (meas, cov) = sub_problem(&normd, &cl, copies, nodes);
            let ac = self.aircraft.get(&icao).unwrap();
            let at = rough_position(ac, &meas, t_wall);
            let alt = self.altitude_for(cx.alt_model, icao, ac, d.altitude_ft, now, at);
            let used_keys: Vec<RxKey> = cl.iter().map(|&i| keys[copies[normd.copies[i]].node]).collect();
            let sites = distinct_sites(&meas);
            let t_clock = t_rx.map(|t0| t0 + (normd.t[cl[0]] - t_first));
            let ctx = GroupCtx { df: d.df, clusters: n_clusters, copies: copies_n, edges: normd.edges, dropped: normd.dropped, sites, t_clock };
            self.update_aircraft(icao, &meas, &cov, alt, t_wall, now, &used_keys, &ctx, cx);
        }
    }

    /// A due holdout group: multilaterated, then compared with its ADS-B.
    fn holdout(&mut self, job: HoldoutJob) {
        let HoldoutJob { frozen: fz, truth, alt_ft, alt_est, now } = job;
        let Some(normd) = fz.normalize(&self.stats) else {
            self.stats.holdout_failed.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let Some(cl) = norm::clusters(&normd, &fz.copies, &fz.nodes, 3).into_iter().max_by_key(|c| c.len()) else {
            self.stats.holdout_failed.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let (meas, cov) = sub_problem(&normd, &cl, &fz.copies, &fz.nodes);
        // Like any multilaterated aircraft: corrected by OTHER aircraft only.
        let alt = Some(geo_from(alt_est, alt_ft, ALT_SIGMA_M, &self.stats));
        let guesses = guesses(None, &meas, alt, now);
        match solver::solve(&SolveInput { meas: &meas, cov: &cov, alt, guesses: &guesses }) {
            Some(f) => {
                self.stats.holdout_solved.fetch_add(1, Ordering::Relaxed);
                let (a, b) = (ecef_to_llh(f.pos), ecef_to_llh(truth));
                let e = crate::geo::haversine_m(a.lat, a.lon, b.lat, b.lon);
                self.shared.diag.lock().unwrap().holdout_err.push_back((now, e));
            }
            None => {
                self.stats.holdout_failed.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Drop stale tracks; the aircraft that still have one.
    fn stale_tracks(&mut self, now: f64) -> Vec<u32> {
        let mut tracked = Vec::new();
        for (icao, a) in self.aircraft.iter_mut() {
            if let Some(tr) = &a.track {
                if tr.stale(now) {
                    a.track = None;
                }
            }
            if a.track.is_some() {
                tracked.push(*icao);
            }
        }
        tracked
    }

    fn expire(&mut self, now: f64) {
        let tracks = self.aircraft.values().filter_map(|a| a.track.as_ref());
        let (n, confirmed) = tracks.fold((0, 0), |(n, c), t| (n + 1, c + t.confirmed() as usize));
        self.shared.tracks.store(n, Ordering::Relaxed);
        self.shared.confirmed.store(confirmed, Ordering::Relaxed);
        let mut dg = self.shared.diag.lock().unwrap();
        while dg.two_site_err.front().is_some_and(|x| now - x.0 >= 600.0) {
            dg.two_site_err.pop_front();
        }
        while dg.two_site_pub.front().is_some_and(|x| now - x.0 >= 600.0) {
            dg.two_site_pub.pop_front();
        }
        while let Some(&(t, _)) = dg.holdout_err.front() {
            if now - t < 600.0 {
                break;
            }
            dg.holdout_err.pop_front();
        }
        while let Some(&(t, ..)) = dg.adsb_err.front() {
            if now - t < 600.0 {
                break;
            }
            dg.adsb_err.pop_front();
        }
    }

    /// Altitude constraint (geometric height m, sigma m) for aircraft `icao`
    /// near `at`, from this frame's altitude or a recent one.
    fn altitude_for(&self, alt_model: &AltModel, icao: u32, ac: &Aircraft, decoded_alt: Option<i32>, now: f64, at: Llh) -> Option<(f64, f64)> {
        if let Some(a) = decoded_alt {
            return Some(geo_constraint(alt_model, &self.stats, Some(icao), a, ALT_SIGMA_M, at, now));
        }
        let a = ac.alt_ft?;
        let age = now - ac.alt_t;
        if age > 30.0 {
            return None;
        }
        // 250 ft at report time, degrading at ~70 ft/s (mlat-server's model).
        Some(geo_constraint(alt_model, &self.stats, Some(icao), a, (250.0 + 70.0 * age) * FT_TO_M, at, now))
    }

    fn commb_velocity(&mut self, icao: u32, msg: &[u8], t: f64) {
        let keep = self.two_site != TwoSite::Off;
        let ac = self.aircraft.entry(icao).or_default();
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
        cx: &mut Cx,
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
        let guesses = guesses(self.aircraft.get(&icao), meas, alt, t);
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
                self.two_site_record(icao, meas, cov, alt, ctx.t_clock.unwrap_or(t), used, cx);
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
            cx.fx.used.extend_from_slice(used);
            let p = self.emit(icao, fix_used.as_ref(), n, t, now, used, cx);
            // Credit the receivers with a position that reached the map
            // (not the bench copies of ADS-B aircraft), for their page.
            let to_map = self.bench_out.is_none() || self.aircraft.get(&icao).is_none_or(|ac| now - ac.last_adsb >= 30.0);
            if p.is_some() && to_map {
                for k in used {
                    if let Some(c) = cx.rx(k).map(|r| r.cov) {
                        cx.fx.fixes.push((c, now));
                    }
                }
            }
            p
        } else {
            None
        };
        self.check_adsb(icao, n, t, now, fix_used.as_ref(), published, accepted, used, ctx, cx);
        if self.two_site != TwoSite::Off && !can_fix {
            self.two_site_try(icao, ctx.t_clock.unwrap_or(t), t, now, cx);
        }
    }

    /// Keep a 2-site TDOA (and who heard it) for the motion + coverage solve.
    #[allow(clippy::too_many_arguments)]
    fn two_site_record(&mut self, icao: u32, meas: &[Meas], cov: &DMatrix<f64>, alt: Option<(f64, f64)>, t: f64, used: &[RxKey], cx: &Cx) {
        let Some((h, _)) = alt else { return };
        let var = cov[(0, 0)] + cov[(1, 1)] - 2.0 * cov[(0, 1)];
        let sd = (var.max(0.0).sqrt() * C_AIR).max(30.0);
        let rec = Rec2 { t, off: [0.0; 2], ra: meas[0].rx, rb: meas[1].rx, d: (meas[1].t - meas[0].t) * C_AIR, sd, h };
        let covs: Vec<CovId> = used.iter().filter_map(|k| cx.rx(k).map(|r| r.cov)).collect();
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
    fn two_site_try(&mut self, icao: u32, t: f64, t_arr: f64, now: f64, cx: &mut Cx) {
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
        let silent: Vec<CovId> = cx.coverage.near(mid.lat, mid.lon).into_iter().filter(|id| !heard.contains(id) && !cx.seeing.contains(id)).collect();
        self.stats.two_site_tries.fetch_add(1, Ordering::Relaxed);
        let coverage = cx.coverage;
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
            self.shared.diag.lock().unwrap().two_site_err.push_back((now, dist(&p, &sol.pos), ok));
            if self.dump {
                let mid = [(last.ra[0] + last.rb[0]) / 2.0, (last.ra[1] + last.rb[1]) / 2.0, (last.ra[2] + last.rb[2]) / 2.0];
                let (sl, tl) = (ecef_to_llh(sol.pos), ecef_to_llh(p));
                let line = serde_json::json!({
                    "sol_lat": sl.lat, "sol_lon": sl.lon, "truth_lat": tl.lat, "truth_lon": tl.lon, "now": now,
                    "t": t, "icao": format!("{icao:06x}"), "err_m": dist(&p, &sol.pos), "ok": ok,
                    "rms": sol.rms, "err_h": sol.err_h, "margin": sol.margin, "n": sol.n, "span": sol.span,
                    "baseline_m": dist(&last.ra, &last.rb), "range_m": dist(&mid, &p), "h": last.h,
                    "speed": (vel[0] * vel[0] + vel[1] * vel[1]).sqrt(),
                    "heard": heard.len(), "silent": silent.len(),
                    "pairs": recs.iter().map(|r| (r.ra[0] as i64, r.rb[0] as i64)).collect::<crate::hashing::HashSet<_>>().len(),
                });
                cx.fx.dump.push(format!("{line}\n"));
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
        if let (Some(pos), Some(p)) = (self.emit(icao, Some(&fix), 2, t_arr, now, &[], cx), truth) {
            let back = [pos[0] - shift[0], pos[1] - shift[1], pos[2] - shift[2]];
            self.shared.diag.lock().unwrap().two_site_pub.push_back((now, dist(&p, &back)));
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
        cx: &Cx,
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
        let mut dg = self.shared.diag.lock().unwrap();
        dg.adsb_err.push_back((now, n, fix_err, out_err));
        let bad = fix_err.unwrap_or(0.0) > 1000.0 || out_err.unwrap_or(0.0) > 1000.0;
        if bad {
            let users = used.iter().filter_map(|k| cx.rx(k).map(|r| r.user.to_string())).collect();
            dg.diag_bad.push_back(Diag {
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
            while dg.diag_bad.len() > 400 {
                dg.diag_bad.pop_front();
            }
        }
    }

    /// Publish the aircraft's position; returns what was published.
    /// Publish the aircraft's position; served receivers in `to` (the ones
    /// whose copies made it) also get it back as a result line.
    #[allow(clippy::too_many_arguments)]
    fn emit(&mut self, icao: u32, fix: Option<&Fix>, n: usize, t: f64, now: f64, to: &[RxKey], cx: &Cx) -> Option<Ecef> {
        let publish = !self.served_output_only || to.iter().any(|k| cx.rx(k).is_some_and(|r| r.serve.is_some()));
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
                let Some(r) = cx.rx(k) else { continue };
                let (Some(opts), Some(tx)) = (r.serve, r.tx.as_ref()) else { continue };
                let line = match opts.results {
                    ResultFormat::None => continue,
                    ResultFormat::Old => old.get_or_insert_with(|| result_old(&res)).clone(),
                    ResultFormat::Ecef => ecef.get_or_insert_with(|| result_ecef(&res)).clone(),
                };
                if tx.try_send(ToClient::ResultLine(line)).is_ok() {
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
}

/// A threaded worker gets its messages in batches of up to `BATCH`: one
/// wake-up per batch instead of one per message (waking a sleeping thread
/// is a system call, and per message it was ~25% of stage 1, 2026-10-28).
/// Stage 1 sends what it has at the end of each tick, before every question,
/// and whenever a batch is full; the order is unchanged.
const BATCH: usize = 512;
/// Batches a threaded worker may have queued before stage 1 waits for it.
const QUEUE: usize = 256;

/// The workers, inline or on their own threads; either way each gets its
/// messages in the order they are sent.
pub(crate) struct Pool {
    handles: Vec<Handle>,
    shared: Vec<Arc<WorkerShared>>,
    fx_tx: std_mpsc::Sender<Effects>,
    fx_rx: std_mpsc::Receiver<Effects>,
}

enum Handle {
    Inline(Box<Worker>),
    Thread { tx: Option<std_mpsc::SyncSender<Vec<ToWorker>>>, join: Option<std::thread::JoinHandle<()>>, buf: Vec<ToWorker> },
}

impl Pool {
    pub(crate) fn new(workers: Vec<Worker>) -> Pool {
        let (fx_tx, fx_rx) = std_mpsc::channel();
        let shared = workers.iter().map(|w| w.shared()).collect();
        Pool { handles: workers.into_iter().map(|w| Handle::Inline(Box::new(w))).collect(), shared, fx_tx, fx_rx }
    }

    pub(crate) fn len(&self) -> usize {
        self.handles.len()
    }

    pub(crate) fn shared(&self) -> &[Arc<WorkerShared>] {
        &self.shared
    }

    /// The workers, while still inline (to configure them).
    pub(crate) fn inline_mut(&mut self) -> impl Iterator<Item = &mut Worker> {
        self.handles.iter_mut().filter_map(|h| match h {
            Handle::Inline(w) => Some(w.as_mut()),
            Handle::Thread { .. } => None,
        })
    }

    /// Move every worker to its own thread.
    pub(crate) fn spawn(&mut self) {
        for (i, h) in self.handles.iter_mut().enumerate() {
            let Handle::Inline(_) = h else { continue };
            let Handle::Inline(mut w) = std::mem::replace(h, Handle::Thread { tx: None, join: None, buf: Vec::new() }) else { unreachable!() };
            let (tx, rx) = std_mpsc::sync_channel::<Vec<ToWorker>>(QUEUE);
            let fx_tx = self.fx_tx.clone();
            let join = std::thread::Builder::new()
                .name(format!("worker-{i}"))
                .spawn(move || {
                    while let Ok(batch) = rx.recv() {
                        let mut fx = Effects::default();
                        for m in batch {
                            w.handle(m, &mut fx);
                        }
                        if !fx.is_empty() {
                            let _ = fx_tx.send(fx);
                        }
                    }
                })
                .expect("worker thread");
            *h = Handle::Thread { tx: Some(tx), join: Some(join), buf: Vec::with_capacity(BATCH) };
        }
    }

    pub(crate) fn send(&mut self, i: usize, msg: ToWorker) {
        match &mut self.handles[i] {
            Handle::Inline(w) => {
                let mut fx = Effects::default();
                w.handle(msg, &mut fx);
                if !fx.is_empty() {
                    let _ = self.fx_tx.send(fx);
                }
            }
            Handle::Thread { tx, buf, .. } => {
                buf.push(msg);
                if buf.len() >= BATCH {
                    send_batch(tx, buf);
                }
            }
        }
    }

    /// Send every threaded worker what is waiting for it (end of a tick).
    pub(crate) fn send_batches(&mut self) {
        for h in &mut self.handles {
            if let Handle::Thread { tx, buf, .. } = h {
                if !buf.is_empty() {
                    send_batch(tx, buf);
                }
            }
        }
    }

    /// To every worker, in order.
    pub(crate) fn broadcast(&mut self, mut msg: impl FnMut() -> ToWorker) {
        for i in 0..self.handles.len() {
            self.send(i, msg());
        }
    }

    /// Ask every worker; one reply each, in worker order.
    pub(crate) fn ask<T>(&mut self, mut msg: impl FnMut(std_mpsc::Sender<T>) -> ToWorker) -> Vec<T> {
        let mut rxs = Vec::with_capacity(self.handles.len());
        for i in 0..self.handles.len() {
            let (tx, rx) = std_mpsc::channel();
            self.send(i, msg(tx));
            rxs.push(rx);
        }
        self.send_batches();
        rxs.into_iter().map(|rx| rx.recv().expect("worker gone")).collect()
    }

    /// Wait until every worker has done everything sent so far.
    pub(crate) fn flush(&mut self) {
        self.ask(ToWorker::Flush);
    }

    /// What the workers left for stage 1 so far.
    pub(crate) fn effects(&self) -> std_mpsc::TryIter<'_, Effects> {
        self.fx_rx.try_iter()
    }

    /// Stop the threads once they have done everything sent.
    pub(crate) fn stop(&mut self) {
        self.send_batches();
        for h in &mut self.handles {
            if let Handle::Thread { tx, join, .. } = h {
                drop(tx.take());
                if let Some(j) = join.take() {
                    let _ = j.join();
                }
            }
        }
    }
}

fn send_batch(tx: &Option<std_mpsc::SyncSender<Vec<ToWorker>>>, buf: &mut Vec<ToWorker>) {
    // A worker that panicked takes the engine down with it.
    tx.as_ref().unwrap().send(std::mem::replace(buf, Vec::with_capacity(BATCH))).expect("worker gone");
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

pub(crate) fn guesses(ac: Option<&Aircraft>, meas: &[Meas], alt: Option<(f64, f64)>, t: f64) -> Vec<Ecef> {
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

/// A barometric altitude as a constraint on height above the ellipsoid:
/// (height m, 1-sigma m). Corrected to geometric with the regional model
/// built from aircraft other than `icao`; without model data nearby it
/// stays barometric with the spread of the difference added to sigma.
pub(crate) fn geo_constraint(alt_model: &AltModel, stats: &Stats, icao: Option<u32>, baro_ft: i32, sigma: f64, at: Llh, now: f64) -> (f64, f64) {
    geo_from(alt_model.estimate(at.lat, at.lon, baro_ft as f64, now, icao), baro_ft, sigma, stats)
}

/// `geo_constraint` from the model's estimate (geometric minus baro, ft),
/// read earlier.
pub(crate) fn geo_from(est: Option<f64>, baro_ft: i32, sigma: f64, stats: &Stats) -> (f64, f64) {
    let h = baro_ft as f64 * FT_TO_M;
    match est {
        Some(d) => {
            stats.alt_geo_corrected.fetch_add(1, Ordering::Relaxed);
            (h + d * FT_TO_M, sigma.hypot(altmodel::MODEL_SIGMA_M))
        }
        None => {
            stats.alt_geo_fallback.fetch_add(1, Ordering::Relaxed);
            (h, sigma.hypot(altmodel::fallback_sigma(h)))
        }
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
pub(crate) fn distinct_sites(meas: &[Meas]) -> usize {
    let mut reps: Vec<Ecef> = Vec::with_capacity(meas.len());
    for m in meas {
        if !reps.iter().any(|r| dist(r, &m.rx) < 1000.0) {
            reps.push(m.rx);
        }
    }
    reps.len()
}

/// Measurements and covariance for one cluster of a normalised group.
pub(crate) fn sub_problem(normd: &norm::Normalized, cl: &[usize], copies: &[Copy], nodes: &[Node]) -> (Vec<Meas>, DMatrix<f64>) {
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
/// An aircraft's ADS-B position at time `t` from its clock-dated history:
/// interpolated between the positions either side (at most 3 s apart), or
/// the nearest one within 1 s; None otherwise.
pub(crate) fn adsb_at(h: &VecDeque<(Ecef, f64)>, t: f64) -> Option<Ecef> {
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
