//! The per-connection writer task: the handshake reply, heartbeats every 30 s,
//! and (serve mode only) what the engine asks for. It owns the socket's write
//! half so a peer that never reads (the shadow proxy may not) can stall it
//! without ever stalling the reader.
//!
//! Output follows mlat-server's `send`: "zlib" and "none" write each line as
//! it comes (`write_raw`), "zlib2" buffers lines and compresses them one
//! second after the first one (`write_zlib` + `_flush_zlib`).

use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use tokio::io::AsyncWriteExt;
use tokio::net::tcp::OwnedWriteHalf;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{interval_at, sleep_until, timeout, Instant, MissedTickBehavior};

use super::framing::Deflater;
use super::Limits;
use crate::types::ToClient;

/// Engine lines longer than this are dropped: keeps every zlib2 frame far
/// below the 65535-byte length field.
const MAX_OUT_LINE: usize = 16 * 1024;

pub(super) struct WriterCfg {
    pub zlib2: bool,
    pub shadow: bool,
    /// The client asked for results; mlat-server discards them otherwise.
    pub return_results: bool,
    /// The client asked for stats (`send_stats`: `if self.return_stats`).
    pub return_stats: bool,
}

/// Bytes waiting for the socket, plus zlib2 lines waiting for their flush.
struct Out {
    buf: Vec<u8>,
    pos: usize,
    deflater: Option<Deflater>,
    zlines: Vec<Vec<u8>>,
    zbytes: usize,
    flush_at: Option<Instant>,
    /// False once a write failed: from then on output is discarded.
    writable: bool,
    cap: usize,
    flush_delay: std::time::Duration,
}

impl Out {
    fn backlog(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// Queue one newline-terminated line. Past the backlog cap a line is
    /// dropped whole (for zlib2 before it reaches the compressor, so the
    /// stream the client inflates never has a hole).
    fn queue(&mut self, line: Vec<u8>) {
        if !self.writable || line.len() > MAX_OUT_LINE {
            return;
        }
        if self.deflater.is_some() {
            if self.backlog() + self.zbytes + line.len() > self.cap {
                return;
            }
            self.zbytes += line.len();
            self.zlines.push(line);
            if self.flush_at.is_none() {
                self.flush_at = Some(Instant::now() + self.flush_delay);
            }
        } else {
            if self.backlog() + line.len() > self.cap {
                return;
            }
            self.compact();
            self.buf.extend_from_slice(&line);
        }
    }

    /// Compress the pending zlib2 lines into frames.
    fn flush_zlib(&mut self) {
        self.flush_at = None;
        if self.writable && !self.zlines.is_empty() {
            self.compact();
            if let Some(d) = self.deflater.as_mut() {
                d.frames(self.zlines.iter().map(|l| l.as_slice()), &mut self.buf);
            }
        }
        self.zlines.clear();
        self.zbytes = 0;
    }

    /// Drop the already-written prefix once it is most of the buffer.
    fn compact(&mut self) {
        if self.pos > 0 && self.pos * 2 >= self.buf.len() {
            self.buf.drain(..self.pos);
            self.pos = 0;
        }
    }

    fn wrote(&mut self, r: std::io::Result<usize>) {
        match r {
            Ok(n) if n > 0 => {
                self.pos += n;
                if self.pos == self.buf.len() {
                    self.buf.clear();
                    self.pos = 0;
                }
            }
            // Peer gone or reset. The reader will see it too, or (shadow
            // proxy that shut its read side) nobody cares: keep reading.
            _ => {
                self.writable = false;
                self.buf = Vec::new();
                self.pos = 0;
                self.zlines.clear();
                self.zbytes = 0;
            }
        }
    }
}

/// `{"heartbeat":{"server_time":round(time.time(), 3)}}`.
fn heartbeat_line() -> Vec<u8> {
    let now = SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64());
    format!("{{\"heartbeat\":{{\"server_time\":{}}}}}\n", (now * 1000.0).round() / 1000.0).into_bytes()
}

/// `{"start_sending":["4ca123",...]}` and friends; None = nothing to send.
fn engine_line(m: ToClient, return_results: bool, return_stats: bool) -> Option<Vec<u8>> {
    let icaos = |key: &str, v: Vec<u32>| {
        if v.is_empty() {
            return None; // mlat-server only sends non-empty sets
        }
        let list: Vec<String> = v.iter().map(|a| format!("\"{a:06x}\"")).collect();
        Some(format!("{{\"{key}\":[{}]}}\n", list.join(",")).into_bytes())
    };
    match m {
        ToClient::StartSending(v) => icaos("start_sending", v),
        ToClient::StopSending(v) => icaos("stop_sending", v),
        ToClient::ResultLine(s) if return_results => {
            let mut b = s.into_bytes();
            b.push(b'\n');
            Some(b)
        }
        ToClient::StatsLine(s) if return_stats => {
            let mut b = s.into_bytes();
            b.push(b'\n');
            Some(b)
        }
        ToClient::ResultLine(_) | ToClient::StatsLine(_) | ToClient::Close => None,
    }
}

/// Run until the engine sends Close (then flush, shut the socket and tell the
/// reader through `close_tx`) or until the reader aborts the task.
pub(super) async fn run(
    mut wr: OwnedWriteHalf,
    mut rx: mpsc::Receiver<ToClient>,
    close_tx: oneshot::Sender<()>,
    reply: Vec<u8>,
    cfg: WriterCfg,
    limits: Arc<Limits>,
) {
    let mut o = Out {
        buf: reply,
        pos: 0,
        deflater: cfg.zlib2.then(Deflater::new),
        zlines: Vec::new(),
        zbytes: 0,
        flush_at: None,
        writable: true,
        cap: limits.max_out_backlog,
        flush_delay: limits.zlib_flush_delay,
    };
    // mlat-server: sleep 30 s, then a heartbeat, forever.
    let mut hb = interval_at(Instant::now() + limits.heartbeat_interval, limits.heartbeat_interval);
    hb.set_missed_tick_behavior(MissedTickBehavior::Delay);
    let mut rx_open = true;
    loop {
        let flush_at = o.flush_at;
        // Futures borrow `o` only until one completes; the handlers run after.
        tokio::select! {
            r = wr.write(&o.buf[o.pos..]), if o.writable && o.pos < o.buf.len() => o.wrote(r),
            m = rx.recv(), if rx_open => match m {
                None => rx_open = false,
                Some(ToClient::Close) => {
                    o.flush_zlib();
                    if o.writable && o.pos < o.buf.len() {
                        let _ = timeout(limits.close_write_timeout, wr.write_all(&o.buf[o.pos..])).await;
                    }
                    let _ = timeout(limits.close_write_timeout, wr.shutdown()).await;
                    let _ = close_tx.send(());
                    return;
                }
                // Shadow mode: the proxy throws our output away and the
                // primary does the steering; never ask for anything.
                Some(m) => {
                    if !cfg.shadow {
                        if let Some(line) = engine_line(m, cfg.return_results, cfg.return_stats) {
                            o.queue(line);
                        }
                    }
                }
            },
            _ = hb.tick() => o.queue(heartbeat_line()),
            () = sleep_until(flush_at.unwrap_or_else(Instant::now)), if flush_at.is_some() => o.flush_zlib(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn engine_lines() {
        assert_eq!(engine_line(ToClient::StartSending(vec![0x4ca123, 0xabc]), false, false).unwrap(), b"{\"start_sending\":[\"4ca123\",\"000abc\"]}\n");
        assert_eq!(engine_line(ToClient::StopSending(vec![1]), false, false).unwrap(), b"{\"stop_sending\":[\"000001\"]}\n");
        assert_eq!(engine_line(ToClient::StartSending(vec![]), true, true), None);
        assert_eq!(engine_line(ToClient::ResultLine("{\"result\":{}}".into()), true, false).unwrap(), b"{\"result\":{}}\n");
        assert_eq!(engine_line(ToClient::ResultLine("{\"result\":{}}".into()), false, true), None);
        assert_eq!(engine_line(ToClient::StatsLine("{\"stats\":{}}".into()), false, true).unwrap(), b"{\"stats\":{}}\n");
        assert_eq!(engine_line(ToClient::StatsLine("{\"stats\":{}}".into()), true, false), None);
    }

    #[test]
    fn heartbeat_shape() {
        let l = heartbeat_line();
        let v: serde_json::Value = serde_json::from_slice(&l).unwrap();
        let t = v["heartbeat"]["server_time"].as_f64().unwrap();
        assert!(t > 1.7e9 && (t * 1000.0 - (t * 1000.0).round()).abs() < 1e-3);
    }
}
