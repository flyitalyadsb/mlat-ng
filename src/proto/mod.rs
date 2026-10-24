//! mlat-client wire protocol: accept, PROXY v1, handshake, compression
//! framing, message parsing. Runs on the tokio worker threads; the engine only
//! ever sees typed `Event`s.
//!
//! IMPLEMENTATION OWNER: agent "proto". Keep the public signatures.
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
//! - Must never panic on any input; fuzz-style tests over random bytes.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use tokio::net::TcpListener;
use tokio::sync::mpsc;

use crate::types::Event;

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
    pub accepted: AtomicU64,
    pub handshake_ok: AtomicU64,
    pub handshake_failed: AtomicU64,
    pub disconnects: AtomicU64,
    pub active: AtomicU64,
    pub msgs_sync: AtomicU64,
    pub msgs_mlat: AtomicU64,
    pub msgs_other: AtomicU64,
    pub msgs_bad: AtomicU64,
    pub dropped_full: AtomicU64,
    pub bytes_in: AtomicU64,
}

impl Counters {
    pub fn inc(c: &AtomicU64) {
        c.fetch_add(1, Ordering::Relaxed);
    }
}

/// Accept loop: one task per connection. Returns only on listener error.
pub async fn serve(
    _listener: TcpListener,
    _cfg: Arc<ListenConfig>,
    _engine: mpsc::Sender<Event>,
    _counters: Arc<Counters>,
) -> anyhow::Result<()> {
    todo!()
}
