//! mlat-ng: Mode S / Mode A/C multilateration server for flyitalyadsb.
//!
//! Threads: the tokio multi-thread runtime does all network I/O, inflate and
//! JSON parsing (one task per feeder connection); the engine runs alone on its
//! own thread and owns every piece of MLAT state (see engine.rs for why).

// Parts of the API (serve mode, Mode A/C helpers, diagnostics) are not yet
// wired everywhere; keep the build quiet until they are.
#![allow(dead_code)]

mod clock;
mod engine;
mod geo;
mod modes;
mod norm;
mod output;
mod proto;
mod relay;
mod replay;
#[cfg(test)]
mod sim;
mod solver;
mod track;
mod types;

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
    let cells = parse_cells(&args.feed)?;
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
    let engine_thread = std::thread::Builder::new().name("engine".into()).spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread().enable_time().build().expect("engine runtime");
        rt.block_on(eng.run(ev_rx));
    })?;

    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().thread_name("net").build()?;
    rt.block_on(async move {
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
        if let (Some(fc), Some(fs)) = (feed_cfg.clone(), feed_stats.clone()) {
            tracing::info!(cells = ?fc.cells, receivers = %fc.receivers_url, "feed input enabled");
            tokio::spawn(relay::run(fc, fs, ev_tx.clone()));
        }
        let render_stats = stats.clone();
        let render_counters = counters.clone();
        let render_feed = feed_stats.clone();
        let service = args.service.clone();
        let render: Arc<dyn Fn(&str) -> Option<(&'static str, String)> + Send + Sync> = Arc::new(move |path: &str| {
            match path {
                "/healthz" => Some(("text/plain", "ok\n".into())),
                "/metrics" => Some((
                    "text/plain; version=0.0.4",
                    render_metrics(&render_stats, &render_counters, &service, render_feed.as_deref()),
                )),
                "/receivers.json" => Some(("application/json", render_stats.registry_json.lock().unwrap().clone())),
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
        tokio::spawn(output::http_server(http_l, render));

        let cfg = Arc::new(proto::ListenConfig { shadow: args.shadow, motd: args.motd, source_url: args.source_url });
        tokio::select! {
            r = proto::serve(client_l, cfg, ev_tx, counters) => r,
            _ = tokio::signal::ctrl_c() => { tracing::info!("SIGINT"); Ok(()) }
            _ = sigterm() => { tracing::info!("SIGTERM"); Ok(()) }
        }
    })?;
    drop(engine_thread);
    Ok(())
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

fn render_metrics(s: &engine::Stats, c: &proto::Counters, service: &str, feed: Option<&relay::FeedStats>) -> String {
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
    m("engine_late_ms_max", "gauge", l(&s.loop_late_ms_max));
    let snap = s.snapshot.lock().unwrap().clone();
    m("receivers", "gauge", snap.receivers as f64);
    m("receivers_suspect", "gauge", snap.receivers_suspect as f64);
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
    o
}
