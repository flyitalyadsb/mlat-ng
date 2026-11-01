//! mlat-client wire protocol: accept, PROXY v1, handshake, compression
//! framing, message parsing. Runs on the tokio worker threads; the engine only
//! ever sees typed `Event`s.
//!
//! IMPLEMENTATION OWNER: agent "proto". Keep the public signatures.
//!
//! Layout: `conn` (one connection: handshake phase, read loop), `writer`
//! (the per-connection writer task), `handshake` (PROXY line, validation,
//! replies), `messages` (line -> message), `framing` (input buffer, zlib
//! frames), `py` (the Python builtins mlat-server applies to client JSON).
//!
//! Compatibility target: what flyitalyadsb/mlat-server (master 33c3361 + PR
//! #14) accepts, and what every mlat-client from 0.2.13 to 0.4.2 sends. The
//! authoritative references are mlat-server `mlat/jsonclient.py` and
//! mlat-client `mlat/client/jsonclient.py`.
//!
//! Hard requirements:
//! - PROXY v1 line before the handshake (`PROXY TCP4|TCP6 src dst sport dport`)
//!   is optional; when present, source_ip comes from it.
//! - Compression is chosen EXACTLY like mlat-server: the first of zlib2, zlib,
//!   none that the client offers. In shadow mode a proxy forwards the client's
//!   bytes to us and discards our replies, so the client is talking the
//!   compression the PRIMARY server chose; choosing differently would make us
//!   read garbage. Inbound framing: "none" = newline JSON lines; "zlib" and
//!   "zlib2" = 2-byte big-endian length + deflate data flushed with
//!   Z_SYNC_FLUSH with the trailing 00 00 ff ff removed (append it back before
//!   inflating), one persistent inflate stream per connection. Outbound: zlib2
//!   compresses the same way, zlib and none write raw lines.
//! - Handshake validation and user-name sanitising as mlat-server does
//!   (version 2 or 3, `compress` required, lat/lon/alt ranges, lon>180 -> the
//!   same -180 quirk so positions agree with the primary, name regex and the
//!   3..40 length rules). Placeholder uuids (empty, all X, all 0 once dashes
//!   and spaces are removed) are treated as no uuid. A uuid that is not a
//!   string is treated as no uuid.
//! - Messages: sync, mlat, seen, lost, rate_report, heartbeat,
//!   input_connected, input_disconnected, clock_reset, clock_jump,
//!   position_update, modeac_messages_count, split_sync (ignore), quine
//!   (ignore). A malformed message is counted and skipped, never fatal.
//!   mlat `t`/sync `et`,`ot` are counter ticks (floats in JSON); `m`,`em`,
//!   `om` are hex strings. Hex must decode to 7 or 14 bytes else skip.
//! - Limits: handshake line <= 16 KiB within 15 s; JSON line <= 64 KiB;
//!   compressed frame <= 65535 (the length field); inflated data per frame <=
//!   1 MiB; an idle connection (nothing for 150 s) is closed.
//! - The engine channel is bounded. Connect/Disconnect use `send().await`.
//!   Sync/Mlat/Seen/... use `try_send`; on Full, drop the event and increment
//!   `Counters::dropped_full`. Never block the socket reader on the engine.
//! - In shadow mode: reply to the handshake exactly like mlat-server would
//!   (compress, reconnect_in, selective_traffic, heartbeat, return_results,
//!   rate_reports, motd) and send heartbeats every 30 s, but never send
//!   start_sending/stop_sending/result. Nobody reads them, but a correct reply
//!   keeps the code path identical to serve mode.
//! - In serve mode: forward `ToClient` messages from the engine to the socket.
//!   A listener is in serve mode when its `ListenConfig::shadow` is false
//!   (main.rs: `--serve-listen`); its connections carry `ReceiverInfo::serve`.
//! - Must never panic on any input; fuzz-style tests over random bytes.
//!
//! Where this module knowingly differs from mlat-server (none of these change
//! how a stream mlat-client actually sends is decoded):
//! - A bad line is skipped and counted. mlat-server drops the whole
//!   connection on any bad line: invalid JSON, a missing field, non-ASCII, a
//!   line over its readline limit, a zlib frame not ending on a newline (here
//!   a line may span frames).
//! - NaN lat/lon/alt (only reachable as the string "nan") is denied; it
//!   slips through mlat-server's range checks.
//! - A PROXY line followed by EOF is a quiet probe, not a denied handshake.
//! - The name-collision rules of PR #14 (identity.py: refuse a live
//!   duplicate, rename another receiver) need the receiver registry and are
//!   the engine's; the random suffix of a name under 3 characters cannot
//!   match the primary's anyway.
//! - `reconnect_in` is a fixed 10 / 900 (mlat-server adds +-10% jitter).
//! - The idle close happens 150 s after the last line or frame; mlat-server
//!   checks on its 30 s heartbeat tick, so somewhere in 150..180 s.
//! - position_update becomes an event (mlat-server ignores it; mlat-client
//!   never sends it over JSON). No UDP transport is offered: the production
//!   shards listen TCP-only (`--client-listen=30100`).
//! - A ResultLine is dropped for a client that did not ask for results, and a
//!   StatsLine for one that did not ask for stats; the result layout it asked
//!   for ("old" / "ecef") is in `ReceiverInfo::serve` for the engine to use.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use tokio::net::TcpListener;
use tokio::sync::mpsc;
use tracing::{debug, warn};

use crate::types::Event;

mod conn;
mod framing;
mod handshake;
mod messages;
mod py;
mod writer;
// Served sessions, timed (M5 in docs/PLAN.md, "Redundancy").
pub mod session;

#[cfg(test)]
mod tests;

#[derive(Debug, Clone)]
pub struct ListenConfig {
    /// Never ask clients for traffic, never send results (see module docs).
    pub shadow: bool,
    pub motd: String,
    /// AGPL notice URL appended to the motd.
    pub source_url: String,
}

/// Lock-free counters exported on /metrics by main.rs.
#[derive(Debug, Default)]
pub struct Counters {
    /// TCP connections accepted.
    pub accepted: AtomicU64,
    pub handshake_ok: AtomicU64,
    /// Handshakes denied, malformed, too long or timed out. A connection that
    /// closes without sending a byte (a TCP probe) counts in neither.
    pub handshake_failed: AtomicU64,
    /// Accepted receivers whose connection has ended.
    pub disconnects: AtomicU64,
    /// Accepted receivers currently connected.
    pub active: AtomicU64,
    pub msgs_sync: AtomicU64,
    pub msgs_mlat: AtomicU64,
    /// Every other well-formed message (seen, lost, rate_report, clock
    /// resets, heartbeat, ...).
    pub msgs_other: AtomicU64,
    /// Lines skipped: not JSON, unknown, failed conversions, bad frame
    /// length, over the line limit.
    pub msgs_bad: AtomicU64,
    /// Events dropped because the engine channel was full.
    pub dropped_full: AtomicU64,
    /// Bytes read from clients, compressed as on the wire.
    pub bytes_in: AtomicU64,
    /// Connections on a serve-mode listener: open, closed, and how long each
    /// waited for its first request and result (session.rs).
    pub serve: session::ServeSessions,
}

impl Counters {
    pub fn inc(c: &AtomicU64) {
        c.fetch_add(1, Ordering::Relaxed);
    }
}

/// Protocol limits and timers. The defaults are the module contract and
/// mlat-server's values; tests shrink the timers.
#[derive(Debug, Clone)]
pub(crate) struct Limits {
    /// Per handshake line (PROXY line and handshake each), like mlat-server's
    /// `wait_for(readline(), 15)`.
    pub handshake_timeout: Duration,
    pub max_handshake_line: usize,
    /// JSON line after the handshake.
    pub max_line: usize,
    /// Inflated bytes out of one zlib frame.
    pub max_frame_inflate: usize,
    /// No complete line / frame for this long closes the connection
    /// (mlat-server's read_heartbeat_interval).
    pub idle_timeout: Duration,
    /// mlat-server's write_heartbeat_interval.
    pub heartbeat_interval: Duration,
    /// zlib2 output is batched for this long (mlat-server's call_later(1.0)).
    pub zlib_flush_delay: Duration,
    /// Capacity of each connection's ToClient channel.
    pub to_client_queue: usize,
    /// Unsent output per connection beyond which new lines are dropped.
    pub max_out_backlog: usize,
    /// Bound on the final writes of a deny or an engine Close.
    pub close_write_timeout: Duration,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            handshake_timeout: Duration::from_secs(15),
            max_handshake_line: 16 * 1024,
            max_line: 64 * 1024,
            max_frame_inflate: 1 << 20,
            idle_timeout: Duration::from_secs(150),
            heartbeat_interval: Duration::from_secs(30),
            zlib_flush_delay: Duration::from_secs(1),
            to_client_queue: 1024,
            max_out_backlog: 256 * 1024,
            close_write_timeout: Duration::from_secs(5),
        }
    }
}

/// Accept loop: one task per connection. Returns only on listener error.
pub async fn serve(
    listener: TcpListener,
    cfg: Arc<ListenConfig>,
    engine: mpsc::Sender<Event>,
    counters: Arc<Counters>,
) -> anyhow::Result<()> {
    serve_with(listener, cfg, engine, counters, Arc::new(Limits::default())).await
}

pub(crate) async fn serve_with(
    listener: TcpListener,
    cfg: Arc<ListenConfig>,
    engine: mpsc::Sender<Event>,
    counters: Arc<Counters>,
    limits: Arc<Limits>,
) -> anyhow::Result<()> {
    // Process-wide, so ids stay unique even across several listeners.
    static NEXT_CONN_ID: AtomicU64 = AtomicU64::new(1);
    loop {
        let (stream, peer) = match listener.accept().await {
            Ok(x) => x,
            Err(e) => match accept_error(&e) {
                AcceptError::Retry => {
                    debug!(error = %e, "accept");
                    continue;
                }
                AcceptError::Backoff => {
                    // Out of fds / buffers: the pending connection stays in
                    // the backlog; give the process a moment to free some.
                    warn!(error = %e, "accept: out of resources, backing off");
                    tokio::time::sleep(Duration::from_millis(100)).await;
                    continue;
                }
                AcceptError::Fatal => return Err(anyhow::Error::new(e).context("accept on the client listener")),
            },
        };
        Counters::inc(&counters.accepted);
        let conn = NEXT_CONN_ID.fetch_add(1, Ordering::Relaxed);
        tokio::spawn(conn::run(stream, peer, conn, cfg.clone(), engine.clone(), counters.clone(), limits.clone()));
    }
}

enum AcceptError {
    Retry,
    Backoff,
    Fatal,
}

/// accept() fails for reasons of the connection being accepted (reset,
/// aborted, and on Linux any pending network error), of resources (EMFILE,
/// ENFILE, ENOBUFS, ENOMEM), or because the listening socket itself is
/// unusable (EBADF, EINVAL, ENOTSOCK). Only the last ends `serve`.
fn accept_error(e: &std::io::Error) -> AcceptError {
    #[cfg(target_os = "linux")]
    const ENOBUFS: i32 = 105;
    #[cfg(not(target_os = "linux"))]
    const ENOBUFS: i32 = 55;
    #[cfg(target_os = "linux")]
    const ENOTSOCK: i32 = 88;
    #[cfg(not(target_os = "linux"))]
    const ENOTSOCK: i32 = 38;
    match e.raw_os_error() {
        Some(9 | 22) => AcceptError::Fatal, // EBADF, EINVAL
        Some(n) if n == ENOTSOCK => AcceptError::Fatal,
        Some(12 | 23 | 24) => AcceptError::Backoff, // ENOMEM, ENFILE, EMFILE
        Some(n) if n == ENOBUFS => AcceptError::Backoff,
        _ => AcceptError::Retry,
    }
}
