//! mlat-ng: Mode S / Mode A/C multilateration server for flyitalyadsb.
//!
//! Threads: the tokio multi-thread runtime does all network I/O, inflate and
//! JSON parsing (one task per feeder connection); the engine runs alone on its
//! own thread and owns every piece of MLAT state (see engine.rs for why).

// Parts of the API (serve mode, Mode A/C helpers, diagnostics) are not yet
// wired everywhere; keep the build quiet until they are.
#![allow(dead_code)]

mod altmodel;
mod clock;
mod clientsim;
mod coverage;
mod covworker;
mod engine;
mod geo;
mod hashing;
mod hybrid;
mod interest;
mod modes;
mod norm;
mod output;
mod persist;
mod proto;
mod ready;
mod relay;
mod replay;
#[cfg(test)]
mod sim;
mod survey;
mod solver;
mod track;
mod twosite;
mod types;
mod worker;

use std::fmt::Write as _;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use clap::Parser;
use tokio::net::TcpListener;
use tokio::sync::{broadcast, mpsc};

#[derive(Parser, Debug)]
#[command(version, about = "Mode S / Mode A/C multilateration server")]
struct Args {
    /// mlat-client listen address.
    #[arg(long, default_value = "0.0.0.0:30100", env = "MLAT_NG_LISTEN")]
    listen: String,
    /// BaseStation (SBS) results listen address.
    #[arg(long, default_value = "0.0.0.0:31003", env = "MLAT_NG_SBS_LISTEN")]
    sbs_listen: String,
    /// HTTP /metrics, /healthz, /state.json.
    #[arg(long, default_value = "0.0.0.0:9100", env = "MLAT_NG_HTTP_LISTEN")]
    http_listen: String,
    /// Shadow mode: never ask clients for traffic, never send results. The
    /// feeders' streams arrive duplicated by mlat-proxy.
    #[arg(long, default_value_t = false, env = "MLAT_NG_SHADOW")]
    shadow: bool,
    /// A second mlat-client listener in serve mode, whatever --shadow says:
    /// feeders routed here (mlat-proxy's region map) get their traffic chosen
    /// by this engine and its results, while --listen keeps receiving the
    /// shadow copy of everyone else. Empty = off.
    #[arg(long, default_value = "", env = "MLAT_NG_SERVE_LISTEN")]
    serve_listen: String,
    /// --serve-listen opens once /readyz would answer 200, or after this
    /// many seconds anyway, so that two cold instances cannot wait on each
    /// other (src/ready.rs); 0 = open at start, without a gate.
    #[arg(long, default_value_t = 240.0, env = "MLAT_NG_SERVE_GATE_MAX_S")]
    serve_gate_max_s: f64,
    /// On SIGTERM /readyz says draining and --serve-listen closes; the
    /// feeders already served keep being served for up to this many seconds
    /// while mlat-proxy moves them to the other instance, then the process
    /// stops as before.
    #[arg(long, default_value_t = 30.0, env = "MLAT_NG_DRAIN_MAX_S")]
    drain_max_s: f64,
    /// /readyz answers 200 only after this many seconds (the hub's reconnect
    /// backoff, the feed's registry).
    #[arg(long, default_value_t = 120.0, env = "MLAT_NG_READY_MIN_UPTIME_S")]
    ready_min_uptime_s: f64,
    /// /readyz answers 200 only with at least this share of the clock pairs
    /// usable (0 = no check).
    #[arg(long, default_value_t = 0.6, env = "MLAT_NG_READY_MIN_USABLE")]
    ready_min_usable: f64,
    /// Serve mode: seconds without an ADS-B position before an aircraft is
    /// multilaterated (mlat-server's MLAT_SERVER_NO_ADSB_MLAT_SECONDS).
    #[arg(long, default_value_t = 120.0, env = "MLAT_NG_SERVE_NO_ADSB_S")]
    serve_no_adsb_s: f64,
    /// Serve mode: ADS-B positions per second a receiver is asked for sync
    /// (MLAT_SERVER_MAX_SYNC_RATE).
    #[arg(long, default_value_t = 12.0, env = "MLAT_NG_SERVE_MAX_SYNC_RATE")]
    serve_max_sync_rate: f64,
    /// Publish only positions a served receiver contributed to (a region's
    /// rollout: the map gets what that region's shard would have produced).
    #[arg(long, default_value_t = false, env = "MLAT_NG_SERVED_OUTPUT_ONLY")]
    served_output_only: bool,
    /// Move receivers to their reliable survey estimates (src/survey.rs):
    /// every 10 min, up to 25 receivers off by more than this many metres.
    #[arg(long, default_value_t = false, env = "MLAT_NG_SURVEY_APPLY")]
    survey_apply: bool,
    #[arg(long, default_value_t = 100.0, env = "MLAT_NG_SURVEY_APPLY_MIN_M")]
    survey_apply_min_m: f64,
    #[arg(long, default_value = "flyitalyadsb MLAT [mlat-ng]", env = "MLAT_NG_MOTD")]
    motd: String,
    #[arg(long, default_value = "https://github.com/nesso-labs/mlat-ng", env = "MLAT_NG_SOURCE_URL")]
    source_url: String,
    /// Seconds to collect copies of a message before solving.
    #[arg(long, default_value_t = 0.9, env = "MLAT_NG_MLAT_DELAY")]
    mlat_delay: f64,
    /// Hold out 1 in N sync points for self-evaluation (0 = off).
    #[arg(long, default_value_t = 20, env = "MLAT_NG_HOLDOUT_EVERY")]
    holdout_every: u64,
    /// Engine event queue capacity.
    #[arg(long, default_value_t = 400_000, env = "MLAT_NG_QUEUE")]
    queue: usize,
    /// Service name in the liveness metrics (nesso_last_useful_work_*).
    #[arg(long, default_value = "mlat-ng", env = "MLAT_NG_SERVICE")]
    service: String,
    /// Feed input: NAME=HOST:PORT,... of readsb-ingest receiver-tagged Beast
    /// outputs (port 4906), one per cell. Empty = no feed.
    #[arg(long, default_value = "", env = "MLAT_NG_FEED")]
    feed: String,
    /// Where the feed gets receiver positions (the primary's /receivers.json).
    #[arg(long, default_value = "http://mlat-ng:9100/receivers.json", env = "MLAT_NG_FEED_RECEIVERS_URL")]
    feed_receivers_url: String,
    /// Keep 1 in N DF17 sync pairs (the same pairs at every receiver).
    #[arg(long, default_value_t = 2, env = "MLAT_NG_FEED_SYNC_EVERY")]
    feed_sync_every: u64,
    /// Bench: multilaterate 1 in N ADS-B aircraft from their non-ADS-B replies (0 = off).
    #[arg(long, default_value_t = 10, env = "MLAT_NG_FEED_BENCH_EVERY")]
    feed_bench_every: u64,
    /// Seconds without an ADS-B position before an aircraft counts as Mode-S-only.
    #[arg(long, default_value_t = 20.0, env = "MLAT_NG_FEED_MODES_ONLY_AFTER")]
    feed_modes_only_after: f64,
    /// SBS listen address for bench positions (aircraft that also broadcast
    /// ADS-B); empty = they go to the main SBS output like any other.
    #[arg(long, default_value = "", env = "MLAT_NG_BENCH_SBS_LISTEN")]
    bench_sbs_listen: String,
    /// Merge each feeder's mlat-client connection and its Beast feed receiver
    /// into one engine receiver (src/hybrid.rs). Needs --feed; point
    /// --feed-receivers-url at this instance's own /receivers.json.
    #[arg(long, default_value_t = false, env = "MLAT_NG_HYBRID")]
    hybrid: bool,
    /// Keep receiver coverage maps in this file (gzip'd JSON, src/coverage.rs):
    /// loaded at start, saved every 30 min. Empty = in memory only.
    #[arg(long, default_value = "", env = "MLAT_NG_COVERAGE_FILE")]
    coverage_file: String,
    /// Keep the receiver position survey and the corrections applied from
    /// it here (gzip'd JSON): loaded at start, saved every 10 minutes and
    /// at shutdown. Empty = in memory only.
    #[arg(long, default_value = "", env = "MLAT_NG_SURVEY_FILE")]
    survey_file: String,
    /// Aircraft heard by only two sites (twosite.rs): off, bench (solve, but
    /// only aircraft with ADS-B get the result, on the bench output) or
    /// publish.
    #[arg(long, default_value = "off", env = "MLAT_NG_TWO_SITE")]
    two_site: engine::TwoSite,
    /// Seed of every hash map (hashing.rs); unset: a random one. The output
    /// does not depend on it; a seed nobody knows keeps crafted collisions
    /// in feeder-supplied keys out of reach.
    #[arg(long, env = "MLAT_NG_HASH_SEED")]
    hash_seed: Option<u64>,
    /// Aircraft worker threads (worker.rs); 0 = one worker on the engine
    /// thread. The output does not depend on it.
    #[arg(long, default_value_t = 0, env = "MLAT_NG_WORKERS")]
    workers: usize,
}

fn parse_cells(s: &str) -> anyhow::Result<Vec<(String, String)>> {
    s.split(',')
        .filter(|x| !x.trim().is_empty())
        .map(|x| {
            let (n, a) = x.trim().split_once('=').ok_or_else(|| anyhow::anyhow!("feed cell must be NAME=HOST:PORT: {x}"))?;
            Ok((n.to_string(), a.to_string()))
        })
        .collect()
}

/// /readyz's criteria and the serve gate's and drain's limits, checked.
fn redundancy(args: &Args) -> anyhow::Result<(ready::ReadyConfig, std::time::Duration, std::time::Duration)> {
    let secs = |name: &str, v: f64| {
        anyhow::ensure!(v.is_finite() && v >= 0.0, "{name} must be a number of seconds >= 0: {v}");
        Ok(std::time::Duration::from_secs_f64(v))
    };
    anyhow::ensure!((0.0..=1.0).contains(&args.ready_min_usable), "--ready-min-usable must be in 0..1: {}", args.ready_min_usable);
    secs("--ready-min-uptime-s", args.ready_min_uptime_s)?;
    let cfg = ready::ReadyConfig { min_uptime_s: args.ready_min_uptime_s, min_usable: args.ready_min_usable, feed: !args.feed.is_empty() };
    Ok((cfg, secs("--serve-gate-max-s", args.serve_gate_max_s)?, secs("--drain-max-s", args.drain_max_s)?))
}

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| "info".into()))
        .with_target(false)
        // No colour codes in `kubectl logs`: only when stdout is a terminal.
        .with_ansi(std::io::IsTerminal::is_terminal(&std::io::stdout()))
        .init();
    let argv: Vec<String> = std::env::args().collect();
    if argv.get(1).map(|a| a == "replay").unwrap_or(false) {
        return replay::main(argv[1..].to_vec());
    }
    let args = Args::parse();
    let lifecycle = Arc::new(ready::Lifecycle::default());
    hashing::set_seed(args.hash_seed.unwrap_or_else(rand::random));
    let cells = parse_cells(&args.feed)?;
    let (ready_cfg, serve_gate_max, drain_max) = redundancy(&args)?;
    tracing::info!(?args, "mlat-ng {} starting", env!("CARGO_PKG_VERSION"));

    let (ev_tx, ev_rx) = mpsc::channel(args.queue);
    let (sbs_tx, _) = broadcast::channel::<Arc<str>>(16_384);
    let stats = Arc::new(engine::Stats::default());
    let counters = Arc::new(proto::Counters::default());

    // The engine: one OS thread, its own single-threaded runtime.
    let ecfg = engine::EngineConfig { shadow: args.shadow, mlat_delay: args.mlat_delay, holdout_every: args.holdout_every };
    let (bench_tx, _) = broadcast::channel::<Arc<str>>(16_384);
    let mut eng = engine::Engine::new(ecfg, stats.clone(), sbs_tx.clone());
    if !args.bench_sbs_listen.is_empty() {
        eng = eng.with_bench_output(bench_tx.clone());
    }
    if !args.coverage_file.is_empty() {
        eng = eng.with_coverage_file(&args.coverage_file);
    }
    if !args.survey_file.is_empty() {
        eng = eng.with_survey_file(&args.survey_file);
    }
    eng = eng.with_two_site(args.two_site);
    eng = eng.with_served_output_only(args.served_output_only);
    eng = eng.with_survey_apply(args.survey_apply.then(|| survey::ApplyCfg { min_m: args.survey_apply_min_m, ..Default::default() }));
    eng = eng.with_serve_params(interest::Params {
        no_adsb_mlat_s: args.serve_no_adsb_s,
        max_sync_rate: args.serve_max_sync_rate,
        ..interest::Params::default()
    });
    eng = eng.with_workers(args.workers.max(1)).with_threads(args.workers > 0);
    let (q_tx, q_rx) = mpsc::channel(64);
    eng = eng.with_queries(q_rx);
    let feed_cfg = (!cells.is_empty()).then(|| relay::FeedConfig {
        cells: cells.clone(),
        receivers_url: args.feed_receivers_url.clone(),
        adapter: relay::adapter::AdapterConfig {
            sync_every: args.feed_sync_every,
            bench_every: args.feed_bench_every,
            modes_only_after: args.feed_modes_only_after,
            ..Default::default()
        },
    });
    let feed_stats = feed_cfg.as_ref().map(|c| Arc::new(relay::FeedStats::new(c)));
    anyhow::ensure!(!args.hybrid || feed_cfg.is_some(), "--hybrid needs --feed");
    let hybrid_stats = args.hybrid.then(|| Arc::new(hybrid::RouterStats::default()));
    // Is this instance warm (src/ready.rs)? For /readyz and the serve gate.
    let verdict: Arc<dyn Fn() -> ready::Verdict + Send + Sync> = {
        let (st, fs, lc) = (stats.clone(), feed_stats.clone(), lifecycle.clone());
        Arc::new(move || ready::assess(&ready_cfg, lc.is_draining(), &ready::Inputs::read(lc.uptime_s(), &st, fs.as_deref())))
    };
    // The engine returns when every event sender is gone, after saving its
    // state; main waits for that (bounded) before exiting.
    let (engine_done, engine_done_rx) = std::sync::mpsc::channel::<()>();
    let engine_thread = std::thread::Builder::new().name("engine".into()).spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread().enable_time().build().expect("engine runtime");
        rt.block_on(eng.run(ev_rx));
        let _ = engine_done.send(());
    })?;

    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().thread_name("net").build()?;
    let served = rt.block_on(async move {
        let client_l = TcpListener::bind(&args.listen).await?;
        let sbs_l = TcpListener::bind(&args.sbs_listen).await?;
        let http_l = TcpListener::bind(&args.http_listen).await?;
        tracing::info!(clients = %args.listen, sbs = %args.sbs_listen, http = %args.http_listen, shadow = args.shadow, "listening");

        tokio::spawn(output::sbs_server(sbs_l, sbs_tx));
        if !args.bench_sbs_listen.is_empty() {
            let bench_l = TcpListener::bind(&args.bench_sbs_listen).await?;
            tracing::info!(bench_sbs = %args.bench_sbs_listen, "bench output listening");
            tokio::spawn(output::sbs_server(bench_l, bench_tx));
        }
        // In hybrid mode both inputs go through the router; otherwise each
        // writes to the engine directly.
        let (client_tx, feed_tx) = match &hybrid_stats {
            Some(hs) => {
                let (ctx, crx) = mpsc::channel(args.queue);
                let (ftx, frx) = mpsc::channel(args.queue);
                tokio::spawn(hybrid::run(crx, frx, ev_tx.clone(), hs.clone()));
                tracing::info!("hybrid input: mlat-client and feed merged per receiver");
                (ctx, ftx)
            }
            None => (ev_tx.clone(), ev_tx.clone()),
        };
        if let (Some(fc), Some(fs)) = (feed_cfg.clone(), feed_stats.clone()) {
            tracing::info!(cells = ?fc.cells, receivers = %fc.receivers_url, "feed input enabled");
            tokio::spawn(relay::run(fc, fs, feed_tx));
        }
        let render_stats = stats.clone();
        let render_counters = counters.clone();
        let render_feed = feed_stats.clone();
        let render_hybrid = hybrid_stats.clone();
        let service = args.service.clone();
        let render: Arc<dyn Fn(&str) -> Option<(&'static str, String)> + Send + Sync> = Arc::new(move |path: &str| {
            match path {
                "/healthz" => Some(("text/plain", "ok\n".into())),
                "/metrics" => Some((
                    "text/plain; version=0.0.4",
                    render_metrics(&render_stats, &render_counters, &service, render_feed.as_deref(), render_hybrid.as_deref()),
                )),
                "/receivers.json" => Some(("application/json", render_stats.registry_json.lock().unwrap().clone())),
                "/survey.json" => Some(("application/json", render_stats.survey_json.lock().unwrap().clone())),
                "/debug.json" => {
                    let d = render_stats.debug.lock().unwrap().clone();
                    Some(("application/json", serde_json::to_string_pretty(&d).unwrap_or_default()))
                }
                "/state.json" => {
                    let s = render_stats.snapshot.lock().unwrap().clone();
                    Some(("application/json", serde_json::to_string_pretty(&s).unwrap_or_default()))
                }
                _ => None,
            }
        });
        let readyz: output::Readyz = {
            let v = verdict.clone();
            Arc::new(move || v().http())
        };
        tokio::spawn(output::http_server(http_l, render, Some(q_tx), Some(readyz)));

        // The serve listener opens once warm and closes on a drain
        // (src/ready.rs); a bind error stops the process, as it did when it
        // was bound at start.
        let serve_task = (!args.serve_listen.is_empty()).then(|| {
            tracing::info!(serve = %args.serve_listen, gate_max_s = args.serve_gate_max_s, "serve listener opens once warm");
            let scfg = Arc::new(proto::ListenConfig { shadow: false, motd: args.motd.clone(), source_url: args.source_url.clone() });
            let (stx, sc, v) = (client_tx.clone(), counters.clone(), verdict.clone());
            let serve = move |l| proto::serve(l, scfg, stx, sc);
            tokio::spawn(ready::serve_gated(args.serve_listen.clone(), lifecycle.clone(), move || v(), serve_gate_max, ready::POLL, serve))
        });
        let serve_failed = async {
            match serve_task {
                Some(t) => match t.await {
                    Ok(Ok(())) => anyhow::bail!("serve listener stopped"),
                    Ok(Err(e)) => Err(e),
                    Err(e) => Err(e.into()),
                },
                None => std::future::pending().await,
            }
        };
        let cfg = Arc::new(proto::ListenConfig { shadow: args.shadow, motd: args.motd, source_url: args.source_url });
        // Its own task: the shadow listener keeps accepting while draining.
        let mut client_task = tokio::spawn(proto::serve(client_l, cfg, client_tx, counters.clone()));
        tokio::select! {
            r = &mut client_task => return r?,
            r = serve_failed => return r,
            _ = tokio::signal::ctrl_c() => tracing::info!("SIGINT"),
            _ = sigterm() => tracing::info!("SIGTERM"),
        }
        // Drain: /readyz says so and the serve listener closes at once; the
        // served feeders stay until mlat-proxy has moved them (it closes
        // their connections here once the other instance is usable), or
        // --drain-max-s. A second signal stops waiting.
        lifecycle.drain();
        if !args.serve_listen.is_empty() {
            let t0 = std::time::Instant::now();
            tracing::info!(served = counters.serve.active(), max_s = args.drain_max_s, "draining: /readyz answers draining, serve listener closed");
            let left = tokio::select! {
                n = ready::wait_served(|| counters.serve.active(), drain_max, ready::POLL / 10) => n,
                _ = tokio::signal::ctrl_c() => counters.serve.active(),
                _ = sigterm() => counters.serve.active(),
            };
            let waited_s = t0.elapsed().as_secs_f64();
            if left == 0 {
                tracing::info!(waited_s, "drained: no served feeder left");
            } else {
                tracing::warn!(waited_s, served = left, "drain over with served feeders still connected");
            }
        }
        Ok(())
    });
    // Stop the network side: its tasks hold the engine's event senders, so
    // the engine sees its input close, saves and returns.
    rt.shutdown_timeout(std::time::Duration::from_secs(2));
    match engine_done_rx.recv_timeout(std::time::Duration::from_secs(20)) {
        Ok(()) => tracing::info!("engine stopped, state saved"),
        Err(_) => tracing::warn!("engine did not stop within 20 s: exiting without waiting"),
    }
    drop(engine_thread);
    served
}

async fn sigterm() {
    #[cfg(unix)]
    {
        if let Ok(mut s) = tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate()) {
            s.recv().await;
            return;
        }
    }
    std::future::pending::<()>().await
}

/// Process CPU seconds and resident memory, from /proc (Linux only).
fn proc_self() -> (f64, f64) {
    let cpu = std::fs::read_to_string("/proc/self/stat")
        .ok()
        .and_then(|s| {
            let rest = &s[s.rfind(')')? + 2..];
            let f: Vec<&str> = rest.split_whitespace().collect();
            let ticks = f.get(11)?.parse::<f64>().ok()? + f.get(12)?.parse::<f64>().ok()?;
            Some(ticks / 100.0)
        })
        .unwrap_or(0.0);
    let rss = std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| {
            let l = s.lines().find(|l| l.starts_with("VmRSS:"))?;
            Some(l.split_whitespace().nth(1)?.parse::<f64>().ok()? * 1024.0)
        })
        .unwrap_or(0.0);
    (cpu, rss)
}

fn render_metrics(
    s: &engine::Stats,
    c: &proto::Counters,
    service: &str,
    feed: Option<&relay::FeedStats>,
    hybrid: Option<&hybrid::RouterStats>,
) -> String {
    let mut o = String::with_capacity(4096);
    let mut m = |name: &str, kind: &str, v: f64| {
        let _ = writeln!(o, "# TYPE mlat_ng_{name} {kind}\nmlat_ng_{name} {v}");
    };
    let l = |a: &std::sync::atomic::AtomicU64| a.load(Ordering::Relaxed) as f64;
    m("build_info", "gauge", 1.0);
    m("conn_accepted_total", "counter", l(&c.accepted));
    m("conn_handshake_ok_total", "counter", l(&c.handshake_ok));
    m("conn_handshake_failed_total", "counter", l(&c.handshake_failed));
    m("conn_disconnects_total", "counter", l(&c.disconnects));
    m("conn_active", "gauge", l(&c.active));
    m("msgs_sync_total", "counter", l(&c.msgs_sync));
    m("msgs_mlat_total", "counter", l(&c.msgs_mlat));
    m("msgs_other_total", "counter", l(&c.msgs_other));
    m("msgs_bad_total", "counter", l(&c.msgs_bad));
    m("msgs_dropped_queue_full_total", "counter", l(&c.dropped_full));
    m("bytes_in_total", "counter", l(&c.bytes_in));
    m("sync_events_total", "counter", l(&s.sync_msgs));
    m("sync_points_total", "counter", l(&s.sync_points));
    m("sync_invalid_total", "counter", l(&s.sync_invalid));
    m("pair_updates_total", "counter", l(&s.pair_updates));
    m("pair_rejects_total", "counter", l(&s.pair_rejects));
    m("pair_resets_total", "counter", l(&s.pair_resets));
    m("pair_resets_drift_total", "counter", l(&s.pair_resets_drift));
    m("tracks_born_total", "counter", l(&s.tracks_born));
    m("tracks_refused_dof0_total", "counter", l(&s.tracks_refused_dof0));
    m("outputs_held_no_fix_total", "counter", l(&s.outputs_held_no_fix));
    m("pair_budget_refused_total", "counter", l(&s.pair_budget_refused));
    m("mlat_events_total", "counter", l(&s.mlat_msgs));
    m("groups_total", "counter", l(&s.groups));
    m("groups_unknown_aircraft_total", "counter", l(&s.groups_unknown_ac));
    m("groups_unnormalised_total", "counter", l(&s.groups_no_norm));
    m("clusters_total", "counter", l(&s.clusters));
    m("solves_total", "counter", l(&s.solves));
    m("fixes_total", "counter", l(&s.fixes));
    m("track_fix_updates_total", "counter", l(&s.track_fix_updates));
    m("track_tdoa_updates_total", "counter", l(&s.track_tdoa_updates));
    m("commb_vel_offered_total", "counter", l(&s.commb_vel_offered));
    m("commb_vel_accepted_total", "counter", l(&s.commb_vel_accepted));
    m("track_rejects_total", "counter", l(&s.track_rejects));
    m("outputs_total", "counter", l(&s.outputs));
    m("bench_outputs_total", "counter", l(&s.bench_outputs));
    m("modeac_groups_total", "counter", l(&s.modeac_groups));
    m("modeac_blacklisted_total", "counter", l(&s.modeac_blacklisted));
    m("modeac_fixes_total", "counter", l(&s.modeac_fixes));
    m("modeac_associated_total", "counter", l(&s.modeac_assoc));
    m("modeac_outputs_total", "counter", l(&s.modeac_outputs));
    m("holdout_solved_total", "counter", l(&s.holdout_solved));
    m("holdout_failed_total", "counter", l(&s.holdout_failed));
    m("alt_geo_corrected_total", "counter", l(&s.alt_geo_corrected));
    m("alt_geo_fallback_total", "counter", l(&s.alt_geo_fallback));
    m("alt_replica_mismatch_total", "counter", l(&s.alt_replica_mismatch));
    m("sync_refs_geometric_total", "counter", l(&s.sync_refs_geometric));
    m("groups_bridged_total", "counter", l(&s.groups_bridged));
    m("bridge_receivers_total", "counter", l(&s.bridge_receivers));
    m("bridged_copies_total", "counter", l(&s.bridged_copies));
    m("engine_late_ms_max", "gauge", l(&s.loop_late_ms_max));
    let snap = s.snapshot.lock().unwrap().clone();
    m("receivers", "gauge", snap.receivers as f64);
    m("receivers_suspect", "gauge", snap.receivers_suspect as f64);
    m("coverage_maps", "gauge", snap.coverage_maps as f64);
    m("coverage_samples_total", "counter", snap.coverage_samples as f64);
    m("alt_model_cells", "gauge", snap.alt_model_cells as f64);
    m("alt_model_samples", "gauge", snap.alt_model_samples as f64);
    m("pairs", "gauge", snap.pairs as f64);
    m("pairs_usable", "gauge", snap.pairs_usable as f64);
    m("sync_entries", "gauge", snap.sync_entries as f64);
    m("pending_groups", "gauge", snap.pending_groups as f64);
    m("aircraft", "gauge", snap.aircraft as f64);
    m("tracks", "gauge", snap.tracks as f64);
    m("tracks_confirmed", "gauge", snap.tracks_confirmed as f64);
    m("modeac_tracks", "gauge", snap.modeac_tracks as f64);
    m("engine_queue_depth", "gauge", snap.queue_depth as f64);
    m("holdout_samples", "gauge", snap.holdout_n as f64);
    m("holdout_error_p50_m", "gauge", snap.holdout_p50);
    m("holdout_error_p90_m", "gauge", snap.holdout_p90);
    m("holdout_error_p99_m", "gauge", snap.holdout_p99);
    m("serve_receivers", "gauge", snap.serve_receivers as f64);
    m("serve_requested_sync", "gauge", snap.serve_requested_sync as f64);
    m("serve_requested_mlat", "gauge", snap.serve_requested_mlat as f64);
    m("serve_mlat_wanted", "gauge", snap.serve_mlat_wanted as f64);
    m("serve_start_lines_total", "counter", l(&s.serve_start_lines));
    m("serve_stop_lines_total", "counter", l(&s.serve_stop_lines));
    m("serve_results_total", "counter", l(&s.serve_results));
    m("serve_stats_lines_total", "counter", l(&s.serve_stats_lines));
    m("serve_send_full_total", "counter", l(&s.serve_send_full));
    m("serve_fast_starts_total", "counter", l(&s.serve_fast_starts));
    m("outputs_not_served_total", "counter", l(&s.outputs_not_served));
    m("survey_receivers", "gauge", snap.survey_receivers as f64);
    m("survey_reliable", "gauge", snap.survey_reliable as f64);
    m("survey_reliable_over_100m", "gauge", snap.survey_reliable_over_100m as f64);
    m("survey_applied", "gauge", snap.survey_applied as f64);
    m("survey_applies_total", "counter", l(&s.survey_applies));
    m("survey_apply_too_big_total", "counter", l(&s.survey_apply_too_big));
    m("two_site_tries_total", "counter", l(&s.two_site_tries));
    m("two_site_solved_total", "counter", l(&s.two_site_solved));
    m("two_site_accepted_total", "counter", l(&s.two_site_accepted));
    m("two_site_tracked_total", "counter", l(&s.two_site_tracked));
    m("two_site_solve_seconds_total", "counter", l(&s.two_site_us) / 1e6);
    m("two_site_bench_n", "gauge", snap.two_site_all.0 as f64);
    m("two_site_bench_error_p50_m", "gauge", snap.two_site_all.1);
    m("two_site_bench_error_p90_m", "gauge", snap.two_site_all.2);
    m("two_site_bench_ok_n", "gauge", snap.two_site_ok.0 as f64);
    m("two_site_bench_ok_error_p50_m", "gauge", snap.two_site_ok.1);
    m("two_site_bench_ok_error_p90_m", "gauge", snap.two_site_ok.2);
    m("two_site_bench_published_n", "gauge", snap.two_site_pub.0 as f64);
    m("two_site_bench_published_error_p50_m", "gauge", snap.two_site_pub.1);
    m("two_site_bench_published_error_p90_m", "gauge", snap.two_site_pub.2);
    let mut extra = String::new();
    for (b, n, f50, f90, o50, o90) in &snap.adsb_check {
        let _ = writeln!(extra, "mlat_ng_adsb_check_count{{receivers=\"{b}\"}} {n}");
        let _ = writeln!(extra, "mlat_ng_adsb_check_fix_error_m{{receivers=\"{b}\",q=\"0.5\"}} {f50}");
        let _ = writeln!(extra, "mlat_ng_adsb_check_fix_error_m{{receivers=\"{b}\",q=\"0.9\"}} {f90}");
        let _ = writeln!(extra, "mlat_ng_adsb_check_out_error_m{{receivers=\"{b}\",q=\"0.5\"}} {o50}");
        let _ = writeln!(extra, "mlat_ng_adsb_check_out_error_m{{receivers=\"{b}\",q=\"0.9\"}} {o90}");
    }
    let (cpu, rss) = proc_self();
    m("process_cpu_seconds_total", "counter", cpu);
    m("process_resident_memory_bytes", "gauge", rss);
    o.push_str(&extra);
    // The estate's "useful work" liveness series (docs/APP-LIVENESS.md in
    // nesso-infra): published positions; the feed adds its own below.
    let _ = writeln!(
        o,
        "nesso_last_useful_work_timestamp_seconds{{service=\"{service}\",work=\"positions\"}} {}",
        l(&s.last_output_ms) / 1000.0
    );
    let _ = writeln!(
        o,
        "nesso_useful_work_items_total{{service=\"{service}\",work=\"positions\"}} {}",
        l(&s.outputs) + l(&s.bench_outputs)
    );
    let _ = writeln!(o, "nesso_useful_work_max_age_seconds{{service=\"{service}\",work=\"positions\"}} 300");
    if let Some(f) = feed {
        relay::render_metrics(&mut o, f, service);
    }
    if let Some(h) = hybrid {
        hybrid::render_metrics(&mut o, h);
    }
    c.serve.render(&mut o);
    o
}
