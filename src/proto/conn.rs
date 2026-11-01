//! One client connection: handshake, then the read loop that turns lines or
//! zlib frames into engine events. The writer runs as its own task
//! (writer.rs) so nothing on the write side can hold up reading.

use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use tokio::io::AsyncWriteExt;
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{sleep_until, timeout, timeout_at, Instant};
use tracing::{debug, info};

use super::framing::{FrameError, InBuf, Inflater};
use super::handshake::{self, Accepted, Outcome};
use super::messages::{self, Msg};
use super::py;
use super::session::Session;
use super::writer::{self, WriterCfg};
use super::{Counters, Limits, ListenConfig};
use crate::types::{ConnId, Event, ReceiverInfo, ServeOpts};

/// How the message phase ended (for the disconnect log line).
#[derive(Debug)]
enum End {
    Eof,
    Idle,
    /// The engine sent ToClient::Close.
    Closed,
    Io,
    Frame(FrameError),
}

pub(super) async fn run(
    stream: TcpStream,
    peer: SocketAddr,
    conn: ConnId,
    cfg: Arc<ListenConfig>,
    engine: mpsc::Sender<Event>,
    counters: Arc<Counters>,
    limits: Arc<Limits>,
) {
    let _ = stream.set_nodelay(true);
    let (mut rd, mut wr) = stream.into_split();
    let mut inb = InBuf::new(16 * 1024);

    // Behind the proxy every TCP peer is the proxy: log the PROXY source.
    let mut source_ip = peer.ip().to_string();
    let a = match handshake_phase(&mut rd, &mut inb, &mut source_ip, &limits, &counters.bytes_in).await {
        Hs::Accept(a) => a,
        // PR #14: a TCP probe (connect, send nothing, leave) is not an event.
        Hs::Probe => {
            debug!(conn, source = %source_ip, "closed before a handshake line");
            return;
        }
        Hs::Failed(why) => {
            Counters::inc(&counters.handshake_failed);
            info!(conn, source = %source_ip, why, "handshake failed");
            return;
        }
        Hs::Deny(reason) => {
            Counters::inc(&counters.handshake_failed);
            info!(conn, source = %source_ip, %reason, "handshake denied");
            deny(&mut wr, &reason, &limits).await;
            return;
        }
    };

    let info = ReceiverInfo {
        user: a.user.clone(),
        uuid: a.uuid.clone(),
        lat: a.lat,
        lon: a.lon,
        alt: a.alt,
        clock: a.clock,
        privacy: a.privacy,
        source_ip,
        client_version: a.client_version.clone(),
        compress: a.compress,
        serve: (!cfg.shadow).then(|| ServeOpts { results: a.result_format, stats: py::truthy(Some(&a.return_stats)) }),
    };
    Counters::inc(&counters.handshake_ok);
    info!(conn, user = %info.user, source = %info.source_ip, compress = a.compress, clock = info.clock.as_str(), version = %info.client_version, "handshake accepted");
    let (tx, rx) = mpsc::channel(limits.to_client_queue);
    if engine.send(Event::Connect { conn, info, tx }).await.is_err() {
        return; // the engine is gone: the process is shutting down
    }
    counters.active.fetch_add(1, Ordering::Relaxed);
    // Serve mode: time to the first request and result (session.rs).
    let session = (!cfg.shadow).then(|| Session::open(counters.clone(), conn, &a.user));

    let mut reply = handshake::accept_line(&a, &cfg).into_bytes();
    reply.push(b'\n');
    let (close_tx, close_rx) = oneshot::channel();
    let wcfg = WriterCfg {
        zlib2: a.compress == "zlib2",
        shadow: cfg.shadow,
        return_results: a.return_results,
        return_stats: py::truthy(Some(&a.return_stats)),
        session,
    };
    let writer = tokio::spawn(writer::run(wr, rx, close_tx, reply, wcfg, limits.clone()));

    let mut sink = Sink { conn, engine: &engine, n: Local::default(), rx_time: 0.0 };
    let end = read_messages(&mut rd, &mut inb, &mut sink, &counters, &limits, &a, close_rx).await;
    sink.n.flush(&counters);
    writer.abort();

    counters.active.fetch_sub(1, Ordering::Relaxed);
    Counters::inc(&counters.disconnects);
    info!(conn, user = %a.user, reason = ?end, "disconnected");
    let _ = engine.send(Event::Disconnect { conn }).await;
}

enum Hs {
    Accept(Box<Accepted>),
    Probe,
    Failed(&'static str),
    Deny(String),
}

/// Read and judge the handshake; a PROXY line replaces `source_ip`.
async fn handshake_phase(rd: &mut OwnedReadHalf, inb: &mut InBuf, source_ip: &mut String, limits: &Limits, bytes_in: &AtomicU64) -> Hs {
    let mut line = match read_line(rd, inb, limits, bytes_in).await {
        Ok(Some(l)) => l,
        Ok(None) => return Hs::Probe,
        Err(why) => return Hs::Failed(why),
    };
    if line.starts_with(b"PROXY ") {
        if !line.is_ascii() {
            return Hs::Deny("Badly formatted handshake: 'ascii' codec can't decode the PROXY line".into());
        }
        match handshake::parse_proxy(&line) {
            Some(ip) => *source_ip = ip,
            None => return Hs::Failed("malformed PROXY line"),
        }
        line = match read_line(rd, inb, limits, bytes_in).await {
            Ok(Some(l)) => l,
            // The proxy announced a client that then sent nothing: a probe
            // through the proxy (mlat-server would deny an empty line to a
            // socket that is already gone).
            Ok(None) => return Hs::Probe,
            Err(why) => return Hs::Failed(why),
        };
    }
    match handshake::evaluate(&line) {
        Outcome::Accept(a) => Hs::Accept(a),
        Outcome::Deny(r) => Hs::Deny(r),
        Outcome::Fatal(why) => Hs::Failed(why),
    }
}

/// asyncio's `readline()` under `wait_for(..., 15)`: the line with its
/// newline, or at EOF whatever partial line there is; Ok(None) when the peer
/// closed without sending anything more.
async fn read_line(rd: &mut OwnedReadHalf, inb: &mut InBuf, limits: &Limits, bytes_in: &AtomicU64) -> Result<Option<Vec<u8>>, &'static str> {
    let deadline = Instant::now() + limits.handshake_timeout;
    let mut scanned = 0;
    loop {
        let d = inb.data();
        if let Some(i) = d[scanned..].iter().position(|&b| b == b'\n') {
            let end = scanned + i + 1;
            if end - 1 > limits.max_handshake_line {
                return Err("handshake line too long");
            }
            let line = d[..end].to_vec();
            inb.consume(end);
            return Ok(Some(line));
        }
        scanned = d.len();
        if scanned > limits.max_handshake_line {
            return Err("handshake line too long");
        }
        match timeout_at(deadline, inb.fill(rd)).await {
            Err(_) => return Err("handshake timeout"),
            Ok(Err(_)) => return Err("read error during handshake"),
            Ok(Ok(0)) => {
                if inb.data().is_empty() {
                    return Ok(None);
                }
                let line = inb.data().to_vec();
                inb.consume(line.len());
                return Ok(Some(line));
            }
            Ok(Ok(n)) => {
                bytes_in.fetch_add(n as u64, Ordering::Relaxed);
            }
        }
    }
}

/// Write the refusal (best effort, bounded) and close.
async fn deny(wr: &mut OwnedWriteHalf, reason: &str, limits: &Limits) {
    let mut line = handshake::deny_line(reason).into_bytes();
    line.push(b'\n');
    let _ = timeout(limits.close_write_timeout, async {
        wr.write_all(&line).await?;
        wr.shutdown().await
    })
    .await;
}

/// Per-connection counts, added to the shared atomics once per read instead
/// of once per message.
#[derive(Default)]
struct Local {
    sync: u64,
    mlat: u64,
    other: u64,
    bad: u64,
    dropped: u64,
}

impl Local {
    fn flush(&mut self, c: &Counters) {
        for (n, a) in [
            (&mut self.sync, &c.msgs_sync),
            (&mut self.mlat, &c.msgs_mlat),
            (&mut self.other, &c.msgs_other),
            (&mut self.bad, &c.msgs_bad),
            (&mut self.dropped, &c.dropped_full),
        ] {
            if *n > 0 {
                a.fetch_add(*n, Ordering::Relaxed);
                *n = 0;
            }
        }
    }
}

/// Turns parsed lines into engine events.
struct Sink<'a> {
    conn: ConnId,
    engine: &'a mpsc::Sender<Event>,
    n: Local,
    /// Wall-clock arrival of the current read, unix seconds (mlat-server
    /// takes time.time() per mlat message; one read is well under 1 ms).
    rx_time: f64,
}

impl Sink<'_> {
    fn line(&mut self, line: &[u8]) {
        let conn = self.conn;
        let ev = match messages::parse(line) {
            Msg::Sync { et, ot, em, om } => {
                self.n.sync += 1;
                Event::Sync { conn, et, ot, em, om }
            }
            Msg::Mlat { t, m } => {
                self.n.mlat += 1;
                Event::Mlat { conn, t, msg: m, rx: self.rx_time }
            }
            Msg::Seen(icao) => {
                self.n.other += 1;
                Event::Seen { conn, icao }
            }
            Msg::Lost(icao) => {
                self.n.other += 1;
                Event::Lost { conn, icao }
            }
            Msg::RateReport(rates) => {
                self.n.other += 1;
                Event::RateReport { conn, rates }
            }
            Msg::ClockReset(reason) => {
                self.n.other += 1;
                Event::ClockReset { conn, reason }
            }
            Msg::PositionUpdate { lat, lon, alt } => {
                self.n.other += 1;
                Event::PositionUpdate { conn, lat, lon, alt }
            }
            Msg::Ignored => {
                self.n.other += 1;
                return;
            }
            Msg::Empty => return,
            Msg::Bad => {
                self.n.bad += 1;
                return;
            }
        };
        // Never wait for the engine: a full queue costs this event, not the
        // socket. (A closed one means shutdown; same treatment.)
        if self.engine.try_send(ev).is_err() {
            self.n.dropped += 1;
        }
    }
}

/// Line-splitting state shared by both framings.
#[derive(Default)]
struct Lines {
    /// Bytes of the current partial line already searched for '\n'.
    scan: usize,
    /// Inside a line longer than the limit: drop up to the next '\n'.
    discarding: bool,
}

fn unix_now() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

async fn read_messages(
    rd: &mut OwnedReadHalf,
    inb: &mut InBuf,
    sink: &mut Sink<'_>,
    counters: &Counters,
    limits: &Limits,
    a: &Accepted,
    close_rx: oneshot::Receiver<()>,
) -> End {
    let zlib = a.compress != "none";
    let mut inflater = Inflater::new();
    // zlib: inflated text not yet split into lines.
    let mut text: Vec<u8> = Vec::new();
    let mut lines = Lines::default();
    // mlat-server's _last_message_time: refreshed per complete line or frame.
    let mut last = Instant::now();
    let idle = sleep_until(last + limits.idle_timeout);
    tokio::pin!(idle);
    tokio::pin!(close_rx);
    let mut close_armed = true;
    loop {
        // Everything already buffered first (the handshake read may have
        // pulled in the first messages too).
        sink.rx_time = unix_now();
        let progressed = if zlib {
            match drain_frames(inb, &mut inflater, &mut text, &mut lines, limits, sink) {
                Ok(p) => p,
                Err(e) => return End::Frame(e),
            }
        } else {
            drain_lines(inb, &mut lines, limits.max_line, sink)
        };
        if progressed {
            last = Instant::now();
        }
        sink.n.flush(counters);

        tokio::select! {
            r = inb.fill(rd) => match r {
                Ok(0) => {
                    // readline() hands back a final unterminated line.
                    if !zlib && !lines.discarding && !inb.data().is_empty() {
                        sink.line(inb.data());
                    }
                    return End::Eof;
                }
                Ok(n) => {
                    counters.bytes_in.fetch_add(n as u64, Ordering::Relaxed);
                }
                Err(_) => return End::Io,
            },
            () = &mut idle => {
                if last.elapsed() >= limits.idle_timeout {
                    return End::Idle;
                }
                idle.as_mut().reset(last + limits.idle_timeout);
            }
            r = &mut close_rx, if close_armed => match r {
                Ok(()) => return End::Closed,
                // The writer is gone without a Close: keep reading.
                Err(_) => close_armed = false,
            },
        }
    }
}

/// "none": newline-terminated JSON lines.
fn drain_lines(inb: &mut InBuf, st: &mut Lines, max_line: usize, sink: &mut Sink<'_>) -> bool {
    let mut progressed = false;
    loop {
        let d = inb.data();
        match d[st.scan..].iter().position(|&b| b == b'\n') {
            Some(i) => {
                let end = st.scan + i;
                if st.discarding || end > max_line {
                    st.discarding = false;
                    sink.n.bad += 1;
                } else {
                    sink.line(&d[..end]);
                }
                inb.consume(end + 1);
                st.scan = 0;
                progressed = true;
            }
            None => {
                if st.discarding || d.len() > max_line {
                    st.discarding = true;
                    let n = d.len();
                    inb.consume(n);
                    st.scan = 0;
                } else {
                    st.scan = d.len();
                }
                return progressed;
            }
        }
    }
}

/// "zlib" / "zlib2": length-prefixed frames of one inflate stream.
fn drain_frames(inb: &mut InBuf, z: &mut Inflater, text: &mut Vec<u8>, st: &mut Lines, limits: &Limits, sink: &mut Sink<'_>) -> Result<bool, FrameError> {
    let mut progressed = false;
    loop {
        let d = inb.data();
        if d.len() < 2 {
            return Ok(progressed);
        }
        let n = u16::from_be_bytes([d[0], d[1]]) as usize;
        // A sync flush is never empty; mlat-server fails on it too.
        if n == 0 {
            return Err(FrameError::Corrupt);
        }
        if d.len() < 2 + n {
            return Ok(progressed);
        }
        z.frame(&d[2..2 + n], text, limits.max_frame_inflate)?;
        inb.consume(2 + n);
        progressed = true;
        split_text(text, st, limits.max_line, sink);
    }
}

/// Hand every complete line of `text` to the sink and keep the partial tail.
/// mlat-client ends every frame on a line boundary (mlat-server even insists
/// on it); a line spanning frames is accepted here all the same.
fn split_text(text: &mut Vec<u8>, st: &mut Lines, max_line: usize, sink: &mut Sink<'_>) {
    let mut start = 0;
    while let Some(i) = text[start..].iter().position(|&b| b == b'\n') {
        let end = start + i;
        if st.discarding || end - start > max_line {
            st.discarding = false;
            sink.n.bad += 1;
        } else {
            sink.line(&text[start..end]);
        }
        start = end + 1;
    }
    if st.discarding || text.len() - start > max_line {
        st.discarding = true;
        text.clear();
    } else {
        text.drain(..start);
    }
    // One oversized frame shouldn't pin a megabyte for the connection's life.
    if text.capacity() > 256 * 1024 {
        text.shrink_to(64 * 1024);
    }
}
