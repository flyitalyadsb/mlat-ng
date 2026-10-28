//! Feed input: multilateration from the receiver-tagged Beast stream that
//! readsb-ingest serves on port 4906 (one per cell), with no mlat-client
//! requests involved.
//!
//! What makes this work (measured on production captures, 2026-10-24):
//! the feeders' own 12 MHz timestamps survive every hop untouched, the
//! receiver id is the first 64 bits of the feeder's UUID (89% match an
//! mlat-client handshake, which gives the position), and feeders' reduced
//! output still forwards nearly every DF17 position, so DF17 clock sync is
//! dense (pair residual p50 0.15 us). Non-DF17 replies are only the subset
//! each feeder's reduce timers pick, so copies of one transmission are
//! sparser than on the mlat-client path.
//!
//! Operationally (readsb source, same session):
//! - read `readsb-ingest-a:4906` and `-b:4906`, never the shared Service:
//!   feeders sit on one cell and move between cells;
//! - each consumer has a 16 KiB queue at readsb: a slow reader silently
//!   loses chunks, so reading must be cheap and never block on the engine;
//! - a half-open connection must not look alive: every read has a timeout.

pub mod adapter;
pub mod beast;
pub mod registry;

use std::fmt::Write as _;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::sync::mpsc;

use crate::engine::unix_now;
use crate::types::Event;
use adapter::{Adapter, AdapterConfig, AdapterStats};
use beast::{Frame, Parser};

/// No byte for this long on a feed connection = stalled: reconnect.
const READ_TIMEOUT: Duration = Duration::from_secs(10);
const REGISTRY_EVERY: Duration = Duration::from_secs(60);
const REGISTRY_RETRY: Duration = Duration::from_secs(10);

#[derive(Debug, Clone)]
pub struct FeedConfig {
    /// (name, host:port) per cell.
    pub cells: Vec<(String, String)>,
    pub receivers_url: String,
    pub adapter: AdapterConfig,
}

#[derive(Default)]
pub struct CellStats {
    pub bytes: AtomicU64,
    pub frames: AtomicU64,
    pub connects: AtomicU64,
    /// Connections ended by a read timeout (half-open or stalled).
    pub stalls: AtomicU64,
    pub errors: AtomicU64,
    pub desync: AtomicU64,
    /// Read batches the adapter could not take in time.
    pub batches_dropped: AtomicU64,
    /// unix ms of the last frame
    pub last_frame_ms: AtomicU64,
}

#[derive(Default)]
pub struct FeedStats {
    pub cells: Vec<(String, CellStats)>,
    pub adapter: Arc<AdapterStats>,
    pub registry_size: AtomicU64,
    pub registry_ok_ms: AtomicU64,
    pub registry_errors: AtomicU64,
    /// Events dropped because the engine queue was full.
    pub events_dropped: AtomicU64,
}

impl FeedStats {
    pub fn new(cfg: &FeedConfig) -> FeedStats {
        FeedStats { cells: cfg.cells.iter().map(|(n, _)| (n.clone(), CellStats::default())).collect(), ..Default::default() }
    }
}

struct Batch {
    cell: usize,
    wall: f64,
    frames: Vec<Frame>,
}

/// Run the feed until the engine goes away.
pub async fn run(cfg: FeedConfig, stats: Arc<FeedStats>, engine: mpsc::Sender<Event>) {
    let (btx, mut brx) = mpsc::channel::<Batch>(4096);
    for (i, (name, addr)) in cfg.cells.iter().enumerate() {
        tokio::spawn(read_cell(i, name.clone(), addr.clone(), stats.clone(), btx.clone()));
    }
    let (rtx, mut rrx) = mpsc::channel(4);
    tokio::spawn(fetch_registry(cfg.receivers_url.clone(), stats.clone(), rtx));

    let mut ad = Adapter::new(cfg.adapter.clone(), stats.adapter.clone());
    let mut tick = tokio::time::interval(Duration::from_secs(1));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut evs: Vec<Event> = Vec::with_capacity(1024);
    loop {
        tokio::select! {
            b = brx.recv() => {
                let Some(b) = b else { return };
                for f in &b.frames {
                    ad.on_frame(f, b.wall, &mut evs);
                }
            }
            r = rrx.recv() => if let Some(r) = r { ad.set_registry(r, &mut evs) },
            _ = tick.tick() => ad.tick(unix_now(), &mut evs),
        }
        for ev in evs.drain(..) {
            match engine.try_send(ev) {
                Ok(()) => {}
                Err(mpsc::error::TrySendError::Full(_)) => {
                    stats.events_dropped.fetch_add(1, Ordering::Relaxed);
                }
                Err(mpsc::error::TrySendError::Closed(_)) => return,
            }
        }
    }
}

async fn read_cell(i: usize, name: String, addr: String, stats: Arc<FeedStats>, out: mpsc::Sender<Batch>) {
    let cs = &stats.cells[i].1;
    let mut backoff = Duration::from_secs(1);
    let mut buf = vec![0u8; 256 * 1024];
    loop {
        let conn = tokio::time::timeout(READ_TIMEOUT, TcpStream::connect(&addr)).await;
        let mut sock = match conn {
            Ok(Ok(s)) => s,
            _ => {
                cs.errors.fetch_add(1, Ordering::Relaxed);
                tracing::warn!(cell = %name, %addr, "feed connect failed");
                tokio::time::sleep(backoff).await;
                backoff = (backoff * 2).min(Duration::from_secs(30));
                continue;
            }
        };
        let _ = sock.set_nodelay(true);
        cs.connects.fetch_add(1, Ordering::Relaxed);
        tracing::info!(cell = %name, %addr, "feed connected");
        let mut parser = Parser::default();
        loop {
            match tokio::time::timeout(READ_TIMEOUT, sock.read(&mut buf)).await {
                Err(_) => {
                    cs.stalls.fetch_add(1, Ordering::Relaxed);
                    tracing::warn!(cell = %name, "feed stalled: no data for {:?}, reconnecting", READ_TIMEOUT);
                    break;
                }
                Ok(Ok(0)) | Ok(Err(_)) => {
                    cs.errors.fetch_add(1, Ordering::Relaxed);
                    tracing::warn!(cell = %name, "feed closed, reconnecting");
                    break;
                }
                Ok(Ok(n)) => {
                    backoff = Duration::from_secs(1);
                    cs.bytes.fetch_add(n as u64, Ordering::Relaxed);
                    let mut frames = Vec::with_capacity(n / 20);
                    parser.push(&buf[..n], &mut frames);
                    cs.desync.store(parser.desync, Ordering::Relaxed);
                    if frames.is_empty() {
                        continue;
                    }
                    let wall = unix_now();
                    cs.frames.fetch_add(frames.len() as u64, Ordering::Relaxed);
                    cs.last_frame_ms.store((wall * 1000.0) as u64, Ordering::Relaxed);
                    // Never block the socket on the engine: readsb would drop
                    // our chunks silently. Count what we lose instead.
                    if out.try_send(Batch { cell: i, wall, frames }).is_err() {
                        cs.batches_dropped.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        }
        tokio::time::sleep(backoff).await;
    }
}

async fn fetch_registry(
    url: String,
    stats: Arc<FeedStats>,
    out: mpsc::Sender<crate::hashing::HashMap<u64, registry::Receiver>>,
) {
    loop {
        match http_get(&url).await.and_then(|b| Ok(serde_json::from_str::<registry::Doc>(&b)?)) {
            Ok(doc) => {
                let reg = registry::parse(&doc);
                let empty = reg.is_empty();
                stats.registry_size.store(reg.len() as u64, Ordering::Relaxed);
                stats.registry_ok_ms.store((unix_now() * 1000.0) as u64, Ordering::Relaxed);
                if out.send(reg).await.is_err() {
                    return;
                }
                // The primary fills its registry ~10 s after a restart.
                if empty {
                    tokio::time::sleep(REGISTRY_RETRY).await;
                    continue;
                }
            }
            Err(e) => {
                stats.registry_errors.fetch_add(1, Ordering::Relaxed);
                tracing::warn!(%url, error = %e, "receiver registry fetch failed");
                tokio::time::sleep(REGISTRY_RETRY).await;
                continue;
            }
        }
        tokio::time::sleep(REGISTRY_EVERY).await;
    }
}

/// Minimal HTTP/1.0 GET for a cluster-internal `http://host:port/path`.
pub async fn http_get(url: &str) -> anyhow::Result<String> {
    let rest = url.strip_prefix("http://").ok_or_else(|| anyhow::anyhow!("only http:// URLs"))?;
    let (hostport, path) = match rest.find('/') {
        Some(i) => (&rest[..i], &rest[i..]),
        None => (rest, "/"),
    };
    let host = hostport.split(':').next().unwrap_or(hostport);
    let fut = async {
        let mut s = TcpStream::connect(hostport).await?;
        s.write_all(format!("GET {path} HTTP/1.0\r\nHost: {host}\r\nConnection: close\r\n\r\n").as_bytes()).await?;
        let mut body = Vec::new();
        s.read_to_end(&mut body).await?;
        anyhow::Ok(body)
    };
    let raw = tokio::time::timeout(Duration::from_secs(10), fut).await??;
    let text = String::from_utf8_lossy(&raw);
    let (head, body) = text.split_once("\r\n\r\n").ok_or_else(|| anyhow::anyhow!("malformed HTTP response"))?;
    let status = head.lines().next().unwrap_or("");
    if !status.contains(" 200 ") {
        anyhow::bail!("HTTP status: {status}");
    }
    Ok(body.to_string())
}

/// Feed metrics, plus the estate's "useful work" liveness series for it.
pub fn render_metrics(o: &mut String, s: &FeedStats, service: &str) {
    let l = |a: &AtomicU64| a.load(Ordering::Relaxed);
    for (name, c) in &s.cells {
        for (m, v) in [
            ("feed_bytes_total", l(&c.bytes)),
            ("feed_frames_total", l(&c.frames)),
            ("feed_connects_total", l(&c.connects)),
            ("feed_stalls_total", l(&c.stalls)),
            ("feed_errors_total", l(&c.errors)),
            ("feed_desync_total", l(&c.desync)),
            ("feed_batches_dropped_total", l(&c.batches_dropped)),
        ] {
            let _ = writeln!(o, "mlat_ng_{m}{{cell=\"{name}\"}} {v}");
        }
        let last = l(&c.last_frame_ms) as f64 / 1000.0;
        let _ = writeln!(o, "nesso_last_useful_work_timestamp_seconds{{service=\"{service}\",work=\"feed_frames_{name}\"}} {last}");
        let _ = writeln!(o, "nesso_useful_work_items_total{{service=\"{service}\",work=\"feed_frames_{name}\"}} {}", l(&c.frames));
        let _ = writeln!(o, "nesso_useful_work_max_age_seconds{{service=\"{service}\",work=\"feed_frames_{name}\"}} 60");
    }
    let a = &s.adapter;
    for (m, v) in [
        ("feed_adapter_frames_total", l(&a.frames)),
        ("feed_frames_modeac_total", l(&a.frames_modeac)),
        ("feed_frames_zero_ts_total", l(&a.frames_zero_ts)),
        ("feed_frames_magic_ts_total", l(&a.frames_magic_ts)),
        ("feed_frames_unmapped_total", l(&a.frames_unmapped)),
        ("feed_frames_not_live_total", l(&a.frames_not_live)),
        ("feed_sync_events_total", l(&a.ev_sync)),
        ("feed_mlat_modes_only_events_total", l(&a.ev_mlat_modes_only)),
        ("feed_mlat_bench_events_total", l(&a.ev_mlat_bench)),
        ("feed_clock_jumps_total", l(&a.clock_jumps)),
        ("feed_clock_resets_total", l(&a.clock_resets)),
        ("feed_late_deliveries_total", l(&a.late_deliveries)),
        ("feed_receivers_multi_clock", l(&a.receivers_multi_clock)),
        ("feed_receivers_live", l(&a.receivers_live)),
        ("feed_receivers_probation", l(&a.receivers_probation)),
        ("feed_receivers_bad_clock", l(&a.receivers_bad_clock)),
        ("feed_receivers_unmapped", l(&a.receivers_unmapped)),
        ("feed_registry_size", l(&s.registry_size)),
        ("feed_registry_errors_total", l(&s.registry_errors)),
        ("feed_events_dropped_total", l(&s.events_dropped)),
    ] {
        let _ = writeln!(o, "mlat_ng_{m} {v}");
    }
    let _ = writeln!(o, "mlat_ng_feed_registry_age_seconds {}", unix_now() - l(&s.registry_ok_ms) as f64 / 1000.0);
}

#[cfg(test)]
mod tests;
