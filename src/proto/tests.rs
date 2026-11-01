//! Socket-level tests: a real listener on loopback, clients that encode
//! exactly like mlat-client, and the engine channel observed from outside.

use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use flate2::{Compress, Compression, Decompress, FlushCompress, FlushDecompress};
use serde_json::{json, Value};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;
use tokio::time::{sleep, timeout, Instant};

use super::framing::{Inflater, SYNC_TRAILER};
use super::{serve_with, Counters, Limits, ListenConfig};
use crate::types::{ClockType, ConnId, Event, ReceiverInfo, ToClient};

const WAIT: Duration = Duration::from_secs(5);

// ------------------------------------------------------------ client side

/// mlat-client's fill_zlib, line for line: compress each line, close a frame
/// once 32768 bytes are pending, sync-flush, strip the 4-byte trailer, prefix
/// the 2-byte length of what is left.
pub(crate) fn client_fill_zlib(z: &mut Compress, lines: &[&[u8]], out: &mut Vec<u8>) {
    fn run(z: &mut Compress, mut input: &[u8], data: &mut Vec<u8>, flush: FlushCompress) {
        loop {
            data.reserve(65536);
            let in0 = z.total_in();
            z.compress_vec(input, data, flush).unwrap();
            input = &input[(z.total_in() - in0) as usize..];
            if input.is_empty() && data.len() < data.capacity() {
                return;
            }
        }
    }
    fn close(z: &mut Compress, data: &mut Vec<u8>, out: &mut Vec<u8>) {
        run(z, &[], data, FlushCompress::Sync);
        assert!(data.len() < 65540);
        assert_eq!(data[data.len() - 4..], SYNC_TRAILER);
        data.truncate(data.len() - 4);
        out.extend_from_slice(&(data.len() as u16).to_be_bytes());
        out.append(data);
    }
    let mut data = Vec::new();
    let mut pending = false;
    for l in lines {
        run(z, l, &mut data, FlushCompress::None);
        pending = true;
        if data.len() >= 32768 {
            close(z, &mut data, out);
            pending = false;
        }
    }
    if pending {
        close(z, &mut data, out);
    }
}

/// mlat-client's handshake: json.dumps(sort_keys=True) (serde_json's maps
/// are sorted too), then 128 spaces and a newline.
fn hs_line(compress: &[&str], tweak: impl FnOnce(&mut Value)) -> Vec<u8> {
    let mut v = json!({
        "version": 3, "client_version": "0.3.91", "compress": compress,
        "selective_traffic": true, "heartbeat": true, "return_results": false,
        "udp_transport": false, "return_result_format": "ecef",
        "uuid": "6a9423e0-7ba6-4672-81a4-81b715ac1ad2",
        "lat": 45.4642, "lon": 9.19, "alt": 120.0, "user": "test-feeder",
        "clock_type": "dump1090", "clock_frequency": 12000000, "clock_epoch": null, "privacy": false
    });
    tweak(&mut v);
    let mut s = ascii_json(&v);
    s.push_str(&" ".repeat(128));
    s.push('\n');
    s.into_bytes()
}

/// serde_json with json.dumps' default ensure_ascii=True: non-ASCII as \\uXXXX.
pub(crate) fn ascii_json(v: &Value) -> String {
    let mut out = String::new();
    for c in serde_json::to_string(v).unwrap().chars() {
        if c.is_ascii() {
            out.push(c);
        } else {
            let mut units = [0u16; 2];
            for u in c.encode_utf16(&mut units) {
                out.push_str(&format!("\\u{:04x}", u));
            }
        }
    }
    out
}

fn offer(mode: &str) -> &'static [&'static str] {
    // What makes mlat-server pick `mode`.
    match mode {
        "zlib2" => &["none", "zlib", "zlib2"],
        "zlib" => &["none", "zlib"],
        _ => &["none"],
    }
}

struct Client {
    s: TcpStream,
    rbuf: Vec<u8>,
    mode: &'static str,
    z: Compress,
    dz: Decompress,
    /// zlib2 frames decoded so far, and the largest one.
    frames: usize,
    max_frame: usize,
}

impl Client {
    async fn connect(addr: SocketAddr, first: &[u8]) -> Client {
        let mut s = TcpStream::connect(addr).await.unwrap();
        s.write_all(first).await.unwrap();
        Client { s, rbuf: Vec::new(), mode: "none", z: Compress::new(Compression::new(1), true), dz: Decompress::new(true), frames: 0, max_frame: 0 }
    }

    /// Connect, handshake offering `mode`, check the reply picked it.
    async fn ready(addr: SocketAddr, mode: &'static str) -> Client {
        let mut c = Client::connect(addr, &hs_line(offer(mode), |_| {})).await;
        let r = c.reply().await;
        assert_eq!(r["compress"], mode, "{r}");
        c.mode = mode;
        c
    }

    /// The first (always uncompressed) reply line.
    async fn reply(&mut self) -> Value {
        let deadline = Instant::now() + WAIT;
        loop {
            if let Some(i) = self.rbuf.iter().position(|&b| b == b'\n') {
                let line: Vec<u8> = self.rbuf.drain(..=i).collect();
                return serde_json::from_slice(&line).unwrap();
            }
            let mut b = [0u8; 4096];
            let n = timeout(deadline - Instant::now(), self.s.read(&mut b)).await.expect("reply timeout").unwrap();
            assert!(n > 0, "EOF before the reply");
            self.rbuf.extend_from_slice(&b[..n]);
        }
    }

    /// Send lines in the negotiated framing, as one write.
    async fn send(&mut self, lines: &[&str]) {
        let mut out = Vec::new();
        if self.mode == "none" {
            for l in lines {
                out.extend_from_slice(l.as_bytes());
                out.push(b'\n');
            }
        } else {
            let owned: Vec<Vec<u8>> = lines.iter().map(|l| format!("{l}\n").into_bytes()).collect();
            let refs: Vec<&[u8]> = owned.iter().map(|l| l.as_slice()).collect();
            client_fill_zlib(&mut self.z, &refs, &mut out);
        }
        self.s.write_all(&out).await.unwrap();
    }

    /// Everything the server writes within `d`, or until it closes.
    /// Returns whether it closed.
    async fn read_for(&mut self, d: Duration) -> bool {
        let deadline = Instant::now() + d;
        let mut b = vec![0u8; 65536];
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match timeout(left, self.s.read(&mut b)).await {
                Err(_) => return false,
                Ok(Ok(0)) | Ok(Err(_)) => return true,
                Ok(Ok(n)) => self.rbuf.extend_from_slice(&b[..n]),
            }
        }
    }

    /// Wait for the server to close the connection.
    async fn expect_closed(&mut self) {
        assert!(self.read_for(WAIT).await, "server did not close the connection");
    }

    /// Decode what the server wrote after the reply, as mlat-client does:
    /// raw lines for none/zlib, zlib frames for zlib2.
    fn server_lines(&mut self) -> Vec<Value> {
        let mut text = Vec::new();
        if self.mode == "zlib2" {
            let mut i = 0;
            while i + 2 <= self.rbuf.len() {
                let n = u16::from_be_bytes([self.rbuf[i], self.rbuf[i + 1]]) as usize;
                if i + 2 + n > self.rbuf.len() {
                    break;
                }
                let mut p = self.rbuf[i + 2..i + 2 + n].to_vec();
                p.extend_from_slice(&SYNC_TRAILER);
                let mut o = Vec::with_capacity(1 << 20);
                self.dz.decompress_vec(&p, &mut o, FlushDecompress::Sync).unwrap();
                text.extend_from_slice(&o);
                self.frames += 1;
                self.max_frame = self.max_frame.max(n);
                i += 2 + n;
            }
            self.rbuf.drain(..i);
        } else {
            let end = self.rbuf.iter().rposition(|&b| b == b'\n').map_or(0, |i| i + 1);
            text = self.rbuf.drain(..end).collect();
        }
        text.split(|&b| b == b'\n').filter(|l| !l.is_empty()).map(|l| serde_json::from_slice(l).unwrap()).collect()
    }
}

// ------------------------------------------------------------ server side

struct H {
    addr: SocketAddr,
    ev: mpsc::Receiver<Event>,
    c: Arc<Counters>,
}

fn fast_limits() -> Limits {
    Limits {
        handshake_timeout: Duration::from_millis(500),
        idle_timeout: Duration::from_secs(10),
        heartbeat_interval: Duration::from_secs(3600),
        zlib_flush_delay: Duration::from_millis(20),
        ..Limits::default()
    }
}

async fn start_with(shadow: bool, limits: Limits, cap: usize) -> H {
    let l = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = l.local_addr().unwrap();
    let (tx, ev) = mpsc::channel(cap);
    let c = Arc::new(Counters::default());
    let cfg = Arc::new(ListenConfig { shadow, motd: "test motd".into(), source_url: "https://example.org/mlat-ng".into() });
    tokio::spawn(serve_with(l, cfg, tx, c.clone(), Arc::new(limits)));
    H { addr, ev, c }
}

async fn start() -> H {
    start_with(true, fast_limits(), 4096).await
}

impl H {
    async fn next(&mut self) -> Event {
        timeout(WAIT, self.ev.recv()).await.expect("no event").expect("engine channel closed")
    }

    async fn connected(&mut self) -> (ConnId, ReceiverInfo, mpsc::Sender<ToClient>) {
        match self.next().await {
            Event::Connect { conn, info, tx } => (conn, info, tx),
            e => panic!("expected Connect, got {e:?}"),
        }
    }

    /// Events of `conn` up to its Disconnect (excluded).
    async fn until_disconnect(&mut self, conn: ConnId) -> Vec<Event> {
        let mut v = Vec::new();
        loop {
            match self.next().await {
                Event::Disconnect { conn: c } if c == conn => return v,
                e => v.push(e),
            }
        }
    }

    fn n(&self, f: impl Fn(&Counters) -> &AtomicU64) -> u64 {
        f(&self.c).load(Ordering::Relaxed)
    }

    /// Wait until a counter reaches `want`.
    async fn wait_for(&self, f: impl Fn(&Counters) -> &AtomicU64, want: u64) {
        let deadline = Instant::now() + WAIT;
        while self.n(&f) < want {
            assert!(Instant::now() < deadline, "counter stuck at {} (want {want})", self.n(&f));
            sleep(Duration::from_millis(5)).await;
        }
    }

    fn assert_no_event(&mut self) {
        if let Ok(e) = self.ev.try_recv() {
            panic!("unexpected event {e:?}");
        }
    }
}

const L7: &str = "5d4ca1239bb7c0";
const L14: &str = "8d4ca123589b8665a2c7b1e6ba5f";

fn hx(s: &str) -> Vec<u8> {
    hex::decode(s).unwrap()
}

// ------------------------------------------------------------ tests

async fn roundtrip(mode: &'static str) {
    let mut h = start().await;
    let mut c = Client::ready(h.addr, mode).await;
    let (conn, info, _tx) = h.connected().await;
    assert_eq!(info.compress, mode);
    assert_eq!(info.user, "test-feeder");
    assert_eq!(info.uuid.as_deref(), Some("6a9423e0-7ba6-4672-81a4-81b715ac1ad2"));
    assert_eq!((info.lat, info.lon, info.alt), (45.4642, 9.19, 120.0));
    assert_eq!(info.clock, ClockType::Dump1090);
    assert_eq!(info.source_ip, "127.0.0.1");
    assert_eq!(info.source_port, c.s.local_addr().unwrap().port().to_string());
    assert_eq!(info.client_version, "0.3.91");
    assert_eq!(info.connection, format!("test-feeder v3 dump1090 0.3.91 tcp {mode}"));

    // Two separate writes, so zlib modes span frames of one stream.
    c.send(&["{\"rate_report\":{}}", &format!("{{\"sync\":{{\"et\":1000,\"em\":\"{L14}\",\"ot\":2000,\"om\":\"{L14}\"}}}}")]).await;
    c.send(&[
        &format!("{{\"mlat\":{{\"t\":123456789012,\"m\":\"{L7}\"}}}}"),
        &format!("{{\"mlat\":{{\"t\":123456789013,\"m\":\"{L14}\"}}}}"),
        "{\"seen\":[\"4ca123\",\"3c6444\"]}",
        "{\"lost\":[\"3c6444\"]}",
        "{\"clock_jump\":true}",
        "{\"heartbeat\":{\"client_time\":1729760000.123}}",
    ])
    .await;
    c.s.shutdown().await.unwrap();
    let ev = h.until_disconnect(conn).await;
    assert_eq!(ev.len(), 7, "{ev:?}");
    assert!(matches!(&ev[0], Event::RateReport { rates, .. } if rates.is_empty()));
    assert!(matches!(&ev[1], Event::Sync { et, ot, em, om, .. } if *et == 1000.0 && *ot == 2000.0 && *em == hx(L14) && *om == hx(L14)));
    assert!(matches!(&ev[2], Event::Mlat { t, msg, rx, .. } if *t == 123456789012.0 && *msg == hx(L7) && *rx > 1.7e9));
    assert!(matches!(&ev[3], Event::Mlat { t, msg, .. } if *t == 123456789013.0 && *msg == hx(L14)));
    assert!(matches!(&ev[4], Event::Seen { icao, .. } if *icao == vec![0x4ca123, 0x3c6444]));
    assert!(matches!(&ev[5], Event::Lost { icao, .. } if *icao == vec![0x3c6444]));
    assert!(matches!(&ev[6], Event::ClockReset { reason: "clientmsg: clock_jump", .. }));
    assert_eq!(h.n(|c| &c.msgs_sync), 1);
    assert_eq!(h.n(|c| &c.msgs_mlat), 2);
    assert_eq!(h.n(|c| &c.msgs_other), 5);
    assert_eq!(h.n(|c| &c.msgs_bad), 0);
    assert_eq!(h.n(|c| &c.active), 0);
    assert_eq!(h.n(|c| &c.disconnects), 1);
    // The connection's own count: every line, heartbeat included.
    assert_eq!(info.msgs.load(Ordering::Relaxed), 8);
}

#[tokio::test]
async fn handshake_sync_mlat_none() {
    roundtrip("none").await;
}

#[tokio::test]
async fn handshake_sync_mlat_zlib() {
    roundtrip("zlib").await;
}

#[tokio::test]
async fn handshake_sync_mlat_zlib2() {
    roundtrip("zlib2").await;
}

/// The bulk lines of scripts/proto_golden.py, from the same LCG.
enum Bulk {
    Sync { et: u64, em: Vec<u8>, ot: u64, om: Vec<u8> },
    Mlat { t: u64, m: Vec<u8> },
}

fn bulk() -> Vec<Bulk> {
    let mut x: u64 = 20261024;
    let mut next32 = move || {
        x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        x >> 32
    };
    (0..2000)
        .map(|i| {
            let hi = next32();
            let t = (hi << 16) | (next32() & 0xffff);
            if i % 4 == 0 {
                let em: Vec<u8> = (0..14).map(|_| (next32() & 0xff) as u8).collect();
                let om: Vec<u8> = (0..14).map(|_| (next32() & 0xff) as u8).collect();
                let ot = t + (next32() & 0xffffff);
                Bulk::Sync { et: t, em, ot, om }
            } else {
                let n = if i % 2 == 1 { 7 } else { 14 };
                Bulk::Mlat { t, m: (0..n).map(|_| (next32() & 0xff) as u8).collect() }
            }
        })
        .collect()
}

fn bulk_line(b: &Bulk) -> String {
    match b {
        Bulk::Sync { et, em, ot, om } => format!("{{\"sync\":{{\"et\":{et},\"em\":\"{}\",\"ot\":{ot},\"om\":\"{}\"}}}}", hex::encode(em), hex::encode(om)),
        Bulk::Mlat { t, m } => format!("{{\"mlat\":{{\"t\":{t},\"m\":\"{}\"}}}}", hex::encode(m)),
    }
}

const GOLDEN: &[u8] = include_bytes!("testdata/mlat_client_zlib2.bin");

/// The fixture's frames, decoded by our framing, give back exactly the lines
/// CPython's zlib compressed (mlat-client's real encoder, not ours).
#[test]
fn cpython_golden_stream_decodes() {
    let hs_end = GOLDEN.iter().position(|&b| b == b'\n').unwrap() + 1;
    let mut want: Vec<String> = vec![
        "{\"rate_report\":{}}".into(),
        "{\"seen\":[\"4ca123\",\"3c6444\",\"000001\"]}".into(),
        "{\"input_connected\":\"connected\"}".into(),
    ];
    want.extend(bulk().iter().map(bulk_line));
    want.extend(
        [
            "{\"lost\":[\"3c6444\"]}",
            "{\"clock_reset\":{\"reason\":\"Decoder mode changed to BEAST\",\"frequency\":12000000,\"epoch\":null,\"mode\":\"BEAST\"}}",
            "{\"clock_jump\":true}",
            "{\"heartbeat\":{\"client_time\":1729760000.123}}",
            "{\"modeac_messages_count\":{\"count\":42}}",
            "{\"rate_report\":{\"4CA123\":1.5,\"3C6444\":0.25}}",
            "{\"input_disconnected\":\"disconnected\"}",
        ]
        .map(String::from),
    );
    let mut z = Inflater::new();
    let mut text = Vec::new();
    let mut b = &GOLDEN[hs_end..];
    let mut frames = 0;
    while !b.is_empty() {
        let n = u16::from_be_bytes([b[0], b[1]]) as usize;
        z.frame(&b[2..2 + n], &mut text, 1 << 20).unwrap();
        // Every mlat-client frame ends on a line boundary.
        assert_eq!(text.last(), Some(&b'\n'));
        b = &b[2 + n..];
        frames += 1;
    }
    assert_eq!(frames, 5);
    let got: Vec<&str> = std::str::from_utf8(&text).unwrap().lines().collect();
    assert_eq!(got.len(), want.len());
    for (g, w) in got.iter().zip(&want) {
        assert_eq!(g, w);
    }
}

/// The same CPython bytes over a socket, as a proxy would replay them.
#[tokio::test]
async fn cpython_golden_stream_over_socket() {
    let mut h = start_with(true, fast_limits(), 8192).await;
    let mut c = Client::connect(h.addr, GOLDEN).await;
    let (conn, info, _tx) = h.connected().await;
    assert_eq!(info.user, "golden-feeder");
    assert_eq!(info.compress, "zlib2");
    assert_eq!(info.clock, ClockType::Dump1090);
    assert_eq!((info.lat, info.lon, info.alt), (45.4642, 9.19, 120.0));
    assert_eq!(c.reply().await["compress"], "zlib2");
    c.s.shutdown().await.unwrap();
    let ev = h.until_disconnect(conn).await;
    let b = bulk();
    // rate_report, seen, input_connected, 2000 bulk, lost, clock_reset,
    // clock_jump, rate_report, input_disconnected (heartbeat and modeac
    // count are not engine events).
    assert_eq!(ev.len(), 3 + b.len() + 5);
    assert!(matches!(&ev[2], Event::ClockReset { reason: "clientmsg: input connected", .. }));
    for (e, w) in ev[3..3 + b.len()].iter().zip(&b) {
        match (e, w) {
            (Event::Sync { et, ot, em, om, .. }, Bulk::Sync { et: we, ot: wo, em: wem, om: wom }) => {
                assert_eq!((*et, *ot), (*we as f64, *wo as f64));
                assert_eq!((em, om), (wem, wom));
            }
            (Event::Mlat { t, msg, .. }, Bulk::Mlat { t: wt, m }) => {
                assert_eq!(*t, *wt as f64);
                assert_eq!(msg, m);
            }
            (e, _) => panic!("out of order: {e:?}"),
        }
    }
    assert!(matches!(&ev[3 + b.len() + 1], Event::ClockReset { reason: "clientmsg: clock_reset", .. }));
    // A rate report is a dict: order is not part of it.
    match &ev[3 + b.len() + 3] {
        Event::RateReport { rates, .. } => {
            let mut r = rates.clone();
            r.sort_by_key(|x| x.0);
            assert_eq!(r, vec![(0x3c6444, 0.25), (0x4ca123, 1.5)]);
        }
        e => panic!("{e:?}"),
    }
    assert_eq!(h.n(|c| &c.msgs_bad), 0);
    assert_eq!(h.n(|c| &c.msgs_sync), 500);
    assert_eq!(h.n(|c| &c.msgs_mlat), 1500);
    assert_eq!(h.n(|c| &c.bytes_in), GOLDEN.len() as u64);
}

#[tokio::test]
async fn proxy_v1_line_sets_source_ip() {
    let mut h = start().await;
    let mut first = b"PROXY TCP4 203.0.113.7 10.0.0.1 51234 30005\r\n".to_vec();
    first.extend(hs_line(&["none"], |_| {}));
    let mut c = Client::connect(h.addr, &first).await;
    assert_eq!(c.reply().await["compress"], "none");
    let (_, info, _) = h.connected().await;
    assert_eq!(info.source_ip, "203.0.113.7");
    assert_eq!(info.source_port, "51234");

    let mut first = b"PROXY TCP6 2001:db8::7 2001:db8::1 51234 30005\r\n".to_vec();
    first.extend(hs_line(&["none"], |_| {}));
    let _c = Client::connect(h.addr, &first).await;
    let (_, info, _) = h.connected().await;
    assert_eq!(info.source_ip, "2001:db8::7");
    assert_eq!(info.source_port, "51234");
}

/// A served client behind the proxy, on through the engine: clients.json
/// has the PROXY line's address and port and the client's own line rate.
#[tokio::test]
async fn served_client_in_clients_json_with_its_proxy_source() {
    use crate::engine::{Engine, EngineConfig, Stats};
    let mut h = start_with(false, fast_limits(), 4096).await;
    let mut first = b"PROXY TCP4 203.0.113.7 10.0.0.1 51234 30101\r\n".to_vec();
    first.extend(hs_line(&["none"], |_| {}));
    let mut c = Client::connect(h.addr, &first).await;
    assert_eq!(c.reply().await["compress"], "none");
    let (conn, info, tx) = h.connected().await;
    assert!(info.serve.is_some());
    let stats = Arc::new(Stats::default());
    let (out, _) = tokio::sync::broadcast::channel(16);
    let mut e = Engine::new(EngineConfig { shadow: false, mlat_delay: 1.0, holdout_every: 0 }, stats.clone(), out);
    e.handle_at(Event::Connect { conn, info, tx }, 1000.0);
    c.send(&["{\"heartbeat\":{\"client_time\":1729760000.123}}"; 30]).await;
    h.wait_for(|c| &c.msgs_other, 30).await;
    e.tick_at(1015.0, true);
    let v: Value = serde_json::from_str(&stats.clients_json.lock().unwrap()).unwrap();
    let x = &v["test-feeder"];
    assert_eq!(x["source_ip"], "203.0.113.7");
    assert_eq!(x["source_port"], "51234");
    assert_eq!(x["message_rate"], 2);
    assert_eq!(x["connection"], "test-feeder v3 dump1090 0.3.91 tcp none");
    assert_eq!(x["uuid"], "6a9423e0-7ba6-4672-81a4-81b715ac1ad2");
}

#[tokio::test]
async fn placeholder_uuid_is_none_and_noted_in_the_motd() {
    let mut h = start().await;
    let line = hs_line(&["none"], |v| v["uuid"] = json!("XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX"));
    let mut c = Client::connect(h.addr, &line).await;
    let r = c.reply().await;
    assert!(r["motd"].as_str().unwrap().contains("placeholder uuid"));
    assert!(r["motd"].as_str().unwrap().contains("https://example.org/mlat-ng"));
    let (_, info, _) = h.connected().await;
    assert_eq!(info.uuid, None);
    // A non-string uuid is no uuid either.
    let line = hs_line(&["none"], |v| v["uuid"] = json!(["6a9423e0-7ba6-4672-81a4-81b715ac1ad2"]));
    let _c = Client::connect(h.addr, &line).await;
    let (_, info, _) = h.connected().await;
    assert_eq!(info.uuid, None);
}

#[tokio::test]
async fn bad_handshakes_are_denied() {
    let mut h = start().await;
    let lines: Vec<Vec<u8>> = vec![
        b"hello there\n".to_vec(),
        hs_line(&["none"], |v| v["version"] = json!(1)),
        hs_line(&["none"], |v| {
            v.as_object_mut().unwrap().remove("compress");
        }),
        hs_line(&["zlib2", "lz4"], |_| {}),
        hs_line(&["none"], |v| v["lat"] = json!(91)),
        hs_line(&["none"], |v| v["lon"] = json!(400)),
        hs_line(&["none"], |v| v["alt"] = json!(20000)),
        hs_line(&["none"], |v| {
            v.as_object_mut().unwrap().remove("user");
        }),
        hs_line(&["none"], |v| {
            v["return_results"] = json!(true);
            v["return_result_format"] = json!("xml");
        }),
        "{\"user\":\"caf\u{e9}\"}\n".as_bytes().to_vec(),
    ];
    for (i, l) in lines.iter().enumerate() {
        let mut c = Client::connect(h.addr, l).await;
        let r = c.reply().await;
        assert_eq!(r["reconnect_in"], 900, "{i}: {r}");
        assert!(r["deny"][0].is_string(), "{i}: {r}");
        c.expect_closed().await;
    }
    assert_eq!(h.n(|c| &c.handshake_failed), lines.len() as u64);
    assert_eq!(h.n(|c| &c.handshake_ok), 0);
    h.assert_no_event();
}

#[tokio::test]
async fn fatal_handshakes_close_without_a_reply() {
    let mut h = start().await;
    let mut proxy_unknown = b"PROXY UNKNOWN\r\n".to_vec();
    proxy_unknown.extend(hs_line(&["none"], |_| {}));
    for l in [b"[1,2,3]\n".to_vec(), hs_line(&["none"], |v| v["clock_type"] = json!("rtlsdr")), proxy_unknown] {
        let mut c = Client::connect(h.addr, &l).await;
        c.expect_closed().await;
        assert!(c.rbuf.is_empty(), "replied {:?}", String::from_utf8_lossy(&c.rbuf));
    }
    assert_eq!(h.n(|c| &c.handshake_failed), 3);
    h.assert_no_event();
}

#[tokio::test]
async fn tcp_probe_closes_quietly() {
    let h = start().await;
    for first in [&b""[..], b"PROXY TCP4 1.2.3.4 5.6.7.8 1 2\r\n"] {
        let mut c = Client::connect(h.addr, first).await;
        c.s.shutdown().await.unwrap();
        c.expect_closed().await;
        assert!(c.rbuf.is_empty());
    }
    h.wait_for(|c| &c.accepted, 2).await;
    sleep(Duration::from_millis(50)).await;
    assert_eq!(h.n(|c| &c.handshake_failed), 0);
    assert_eq!(h.n(|c| &c.handshake_ok), 0);
}

#[tokio::test]
async fn handshake_timeout_and_length_limit() {
    let h = start().await; // 500 ms handshake timeout
    let t0 = Instant::now();
    let mut c = Client::connect(h.addr, b"{\"version\":3").await;
    c.expect_closed().await;
    assert!(t0.elapsed() >= Duration::from_millis(400));
    // 16 KiB without a newline: closed at once, long before the timeout.
    let mut c = Client::connect(h.addr, &vec![b' '; 17 * 1024]).await;
    c.expect_closed().await;
    assert_eq!(h.n(|c| &c.handshake_failed), 2);
}

#[tokio::test]
async fn messages_in_the_handshake_segment_and_a_final_unterminated_line() {
    let mut h = start().await;
    let mut first = hs_line(&["none"], |_| {});
    first.extend_from_slice(format!("{{\"mlat\":{{\"t\":1,\"m\":\"{L7}\"}}}}\n{{\"mlat\":{{\"t\":2,\"m\":\"{L7}\"}}}}").as_bytes());
    let mut c = Client::connect(h.addr, &first).await;
    c.s.shutdown().await.unwrap();
    let (conn, _, _) = h.connected().await;
    let ev = h.until_disconnect(conn).await;
    assert!(matches!(ev[..], [Event::Mlat { t: a, .. }, Event::Mlat { t: b, .. }] if a == 1.0 && b == 2.0), "{ev:?}");
}

#[tokio::test]
async fn overlong_lines_are_skipped_not_fatal() {
    let mut h = start().await;
    let mut c = Client::ready(h.addr, "none").await;
    let (conn, _, _) = h.connected().await;
    let long = format!("{{\"seen\":[{}\"4ca123\"]}}", "\"4ca123\",".repeat(8000)); // ~72 KiB
    c.send(&[&long, &format!("{{\"mlat\":{{\"t\":7,\"m\":\"{L7}\"}}}}")]).await;
    c.s.shutdown().await.unwrap();
    let ev = h.until_disconnect(conn).await;
    assert!(matches!(ev[..], [Event::Mlat { t, .. }] if t == 7.0), "{ev:?}");
    assert_eq!(h.n(|c| &c.msgs_bad), 1);
}

#[tokio::test]
async fn zlib_line_spanning_frames_and_corrupt_frames() {
    let mut h = start().await;
    let mut c = Client::ready(h.addr, "zlib2").await;
    let (conn, _, _) = h.connected().await;
    // One line cut in two frames: mlat-server would hang up, we decode it.
    let line = format!("{{\"mlat\":{{\"t\":9,\"m\":\"{L14}\"}}}}\n");
    let (a, b) = line.as_bytes().split_at(10);
    let mut out = Vec::new();
    client_fill_zlib(&mut c.z, &[a], &mut out);
    client_fill_zlib(&mut c.z, &[b], &mut out);
    c.s.write_all(&out).await.unwrap();
    // Then a frame that is not deflate data: the stream is lost, close.
    c.s.write_all(&[0, 4, 0xde, 0xad, 0xbe, 0xef]).await.unwrap();
    c.expect_closed().await;
    let ev = h.until_disconnect(conn).await;
    assert!(matches!(ev[..], [Event::Mlat { t, .. }] if t == 9.0), "{ev:?}");
}

#[tokio::test]
async fn zlib_bomb_frame_closes() {
    let mut h = start().await;
    let mut c = Client::ready(h.addr, "zlib").await;
    let (conn, _, _) = h.connected().await;
    let bomb = vec![b' '; 4 << 20]; // whitespace: would even be valid "lines"
    let mut out = Vec::new();
    client_fill_zlib(&mut c.z, &[&bomb], &mut out);
    assert!(out.len() < 65537, "fits one frame");
    c.s.write_all(&out).await.unwrap();
    c.expect_closed().await;
    h.until_disconnect(conn).await;
}

#[tokio::test]
async fn full_engine_does_not_block_reading() {
    // Room for the Connect and one more event; nobody drains.
    let mut h = start_with(true, fast_limits(), 2).await;
    let hs = hs_line(&["none"], |_| {});
    let mut c = Client::connect(h.addr, &hs).await;
    c.reply().await;
    let n = 20_000;
    let mut payload = Vec::new();
    for i in 0..n {
        payload.extend_from_slice(format!("{{\"mlat\":{{\"t\":{i},\"m\":\"{L14}\"}}}}\n").as_bytes());
    }
    timeout(WAIT, c.s.write_all(&payload)).await.expect("server stopped reading").unwrap();
    h.wait_for(|c| &c.msgs_mlat, n).await;
    assert!(h.n(|c| &c.dropped_full) >= n - 1);
    assert_eq!(h.n(|c| &c.bytes_in), (hs.len() + payload.len()) as u64);
    // Disconnect still arrives once the engine catches up.
    c.s.shutdown().await.unwrap();
    let (conn, _, _) = h.connected().await;
    h.until_disconnect(conn).await;
}

#[tokio::test]
async fn idle_connections_are_closed() {
    let limits = Limits { idle_timeout: Duration::from_millis(300), ..fast_limits() };
    let mut h = start_with(true, limits, 64).await;
    let mut c = Client::ready(h.addr, "none").await;
    let (conn, _, _) = h.connected().await;
    // Talking keeps it open past the idle timeout...
    for _ in 0..6 {
        c.send(&["{\"heartbeat\":{\"client_time\":1.0}}"]).await;
        sleep(Duration::from_millis(100)).await;
    }
    assert!(!c.read_for(Duration::from_millis(1)).await, "closed while active");
    // ...silence closes it.
    let t0 = Instant::now();
    c.expect_closed().await;
    assert!(t0.elapsed() >= Duration::from_millis(150), "{:?}", t0.elapsed());
    h.until_disconnect(conn).await;
}

#[tokio::test]
async fn shadow_mode_heartbeats_but_never_steers() {
    let limits = Limits { heartbeat_interval: Duration::from_millis(100), ..fast_limits() };
    let mut h = start_with(true, limits, 64).await;
    let mut c = Client::ready(h.addr, "zlib2").await;
    let (conn, _, tx) = h.connected().await;
    tx.send(ToClient::StartSending(vec![0x4ca123])).await.unwrap();
    tx.send(ToClient::StopSending(vec![0x4ca123])).await.unwrap();
    tx.send(ToClient::ResultLine("{\"result\":{}}".into())).await.unwrap();
    assert!(!c.read_for(Duration::from_millis(450)).await);
    let got = c.server_lines();
    assert!(got.len() >= 2, "{got:?}");
    assert!(got.iter().all(|v| v["heartbeat"]["server_time"].is_f64()), "{got:?}");
    // Close is honoured in shadow mode too.
    tx.send(ToClient::Close).await.unwrap();
    c.expect_closed().await;
    h.until_disconnect(conn).await;
}

async fn serve_mode(mode: &'static str) {
    let mut h = start_with(false, fast_limits(), 64).await;
    let mut c = Client::connect(h.addr, &hs_line(offer(mode), |v| v["return_results"] = json!(true))).await;
    let r = c.reply().await;
    assert_eq!(r["return_results"], true);
    c.mode = mode;
    let (conn, _, tx) = h.connected().await;
    tx.send(ToClient::StartSending(vec![0x4ca123, 0xabc])).await.unwrap();
    tx.send(ToClient::StopSending(vec![0x3c6444])).await.unwrap();
    tx.send(ToClient::ResultLine("{\"result\":{\"@\":1.5,\"addr\":\"4ca123\"}}".into())).await.unwrap();
    tx.send(ToClient::Close).await.unwrap();
    c.expect_closed().await;
    assert_eq!(
        c.server_lines(),
        vec![
            json!({"start_sending": ["4ca123", "000abc"]}),
            json!({"stop_sending": ["3c6444"]}),
            json!({"result": {"@": 1.5, "addr": "4ca123"}}),
        ]
    );
    h.until_disconnect(conn).await;
}

#[tokio::test]
async fn serve_mode_forwards_engine_messages_none() {
    serve_mode("none").await;
}

#[tokio::test]
async fn serve_mode_forwards_engine_messages_zlib2() {
    serve_mode("zlib2").await;
}

// ------------------------------------------------------------ fuzz

fn lcg(seed: u64) -> impl FnMut() -> u64 {
    let mut x = seed;
    move || {
        x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        x >> 16
    }
}

/// Random bytes after a valid handshake, in every framing: plain noise,
/// noise inside well-formed frames, and corrupted mlat-client lines. Every
/// Connect must be followed by its Disconnect (a panicking task would never
/// send it), and the listener must keep working.
#[tokio::test]
async fn garbage_after_handshake_never_panics() {
    let mut h = start_with(true, fast_limits(), 1 << 16).await;
    let mut r = lcg(7);
    for round in 0..60 {
        let mode = ["none", "zlib", "zlib2"][round % 3];
        let mut c = Client::ready(h.addr, mode).await;
        let (conn, _, _) = h.connected().await;
        let mut junk = Vec::new();
        match round % 4 {
            0 => (0..(r() % 5000)).for_each(|_| junk.push(r() as u8)),
            1 if mode != "none" => {
                const ALPHABET: &[u8] = b"{}[]\":,0123456789abcdef \n\\x";
                let text: Vec<u8> = (0..(r() % 5000)).map(|_| ALPHABET[r() as usize % ALPHABET.len()]).collect();
                client_fill_zlib(&mut c.z, &[&text], &mut junk);
            }
            _ => {
                for _ in 0..200 {
                    let mut l = format!("{{\"mlat\":{{\"t\":{},\"m\":\"{L14}\"}}}}", r()).into_bytes();
                    let p = (r() as usize) % l.len();
                    l[p] = r() as u8;
                    l.push(b'\n');
                    junk.extend_from_slice(&l);
                }
                if mode != "none" {
                    let mut framed = Vec::new();
                    client_fill_zlib(&mut c.z, &[&junk], &mut framed);
                    junk = framed;
                }
            }
        }
        let _ = c.s.write_all(&junk).await;
        let _ = c.s.shutdown().await;
        h.until_disconnect(conn).await;
    }
    assert_eq!(h.n(|c| &c.active), 0);
    // Still serving.
    let _c = Client::ready(h.addr, "zlib2").await;
    h.connected().await;
}

#[tokio::test]
async fn random_handshakes_never_panic() {
    let mut h = start().await;
    let mut r = lcg(99);
    for round in 0..80 {
        let mut first: Vec<u8> = match round % 3 {
            0 => (0..(r() % 400)).map(|_| r() as u8).collect(),
            1 => {
                const ALPHABET: &[u8] = b"{}[]\":,0123456789 versioncompresuzlibnoe\n";
                (0..(r() % 400)).map(|_| ALPHABET[r() as usize % ALPHABET.len()]).collect()
            }
            _ => {
                // A real handshake with one byte changed.
                let mut l = hs_line(&["zlib2", "none"], |_| {});
                let p = (r() as usize) % (l.len() - 129);
                l[p] = r() as u8;
                l
            }
        };
        if round % 7 == 0 {
            let mut p = b"PROXY TCP4 1.2.3.4 5.6.7.8 1 2\r\n".to_vec();
            p.append(&mut first);
            first = p;
        }
        let mut c = Client::connect(h.addr, &first).await;
        let _ = c.s.shutdown().await;
        c.expect_closed().await;
    }
    // A mutated handshake may still be valid (Connect + Disconnect events
    // queued): skip to the Connect of a last, real client.
    let _c = Client::ready(h.addr, "zlib").await;
    loop {
        if let Event::Connect { info, .. } = h.next().await {
            if info.compress == "zlib" {
                break;
            }
        }
    }
}

/// Our zlib2 output decodes with mlat-client's algorithm over many frames:
/// one persistent stream across batches, and a batch big enough to be split
/// at 32 KiB like mlat-server's _flush_zlib_locked does.
#[tokio::test]
async fn zlib2_output_stream_is_continuous() {
    let limits = Limits { zlib_flush_delay: Duration::from_millis(150), max_out_backlog: 1 << 20, ..fast_limits() };
    let mut h = start_with(false, limits, 64).await;
    let mut c = Client::connect(h.addr, &hs_line(offer("zlib2"), |v| v["return_results"] = json!(true))).await;
    c.reply().await;
    c.mode = "zlib2";
    let (_, _, tx) = h.connected().await;
    let mut r = lcg(3);
    let mut want = Vec::new();
    for burst in 0..3u64 {
        // ~2000 poorly compressible lines inside one flush window.
        for i in 0..2000u64 {
            let line = format!("{{\"result\":{{\"@\":{},\"addr\":\"{:06x}\",\"x\":\"{:016x}{:016x}\"}}}}", burst * 10000 + i, r() & 0xffffff, r(), r());
            want.push(serde_json::from_str::<Value>(&line).unwrap());
            tx.send(ToClient::ResultLine(line)).await.unwrap();
        }
        sleep(Duration::from_millis(250)).await;
    }
    c.read_for(Duration::from_millis(300)).await;
    assert_eq!(c.server_lines(), want);
    assert!(c.frames >= 6, "{} frames", c.frames);
    assert!(c.max_frame >= 32768 - 2 && c.max_frame < 65536, "largest frame {}", c.max_frame);
}

/// Served sessions on the wire (session.rs): a served client's first
/// start_sending and first result are timed once, its close is counted; a
/// shadow connection is no session at all.
#[tokio::test]
async fn served_sessions_are_timed_to_their_first_request_and_result() {
    let mut h = start_with(false, fast_limits(), 64).await;
    let mut c = Client::connect(h.addr, &hs_line(offer("none"), |v| v["return_results"] = json!(true))).await;
    c.reply().await;
    let (conn, _, tx) = h.connected().await;
    assert_eq!(h.c.serve.active(), 1);
    tx.send(ToClient::StopSending(vec![1])).await.unwrap();
    tx.send(ToClient::StartSending(vec![0x4ca123])).await.unwrap();
    tx.send(ToClient::StartSending(vec![0x4ca124])).await.unwrap();
    tx.send(ToClient::ResultLine("{\"result\":{\"@\":1.5,\"addr\":\"4ca123\"}}".into())).await.unwrap();
    tx.send(ToClient::ResultLine("{\"result\":{\"@\":2.5,\"addr\":\"4ca123\"}}".into())).await.unwrap();
    c.read_for(Duration::from_millis(100)).await;
    assert_eq!(c.server_lines().len(), 5);
    let mut o = String::new();
    h.c.serve.render(&mut o);
    assert!(o.contains("mlat_ng_serve_first_start_seconds_count 1\n"), "{o}");
    assert!(o.contains("mlat_ng_serve_first_result_seconds_count 1\n"), "{o}");
    c.s.shutdown().await.unwrap();
    h.until_disconnect(conn).await;
    // The writer is aborted after the reader's Disconnect: its session
    // closes a moment later.
    let deadline = Instant::now() + WAIT;
    while h.c.serve.active() > 0 {
        assert!(Instant::now() < deadline, "the session never closed");
        sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(h.n(|c| &c.serve.closed), 1);
    assert_eq!(h.n(|c| &c.serve.closed_no_result), 0);

    // A served client that leaves before any result.
    let mut c = Client::connect(h.addr, &hs_line(offer("none"), |_| {})).await;
    c.reply().await;
    let (conn, _, _tx) = h.connected().await;
    c.s.shutdown().await.unwrap();
    h.until_disconnect(conn).await;
    h.wait_for(|c| &c.serve.closed_no_result, 1).await;
    assert_eq!(h.n(|c| &c.serve.opened), 2);

    // Shadow connections are not served sessions.
    let mut h = start().await;
    let _c = Client::ready(h.addr, "none").await;
    h.connected().await;
    assert_eq!(h.n(|c| &c.serve.opened), 0);
}
