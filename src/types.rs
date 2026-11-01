//! Types shared between the network layer (proto) and the engine.
//!
//! The network layer parses everything off the engine thread and hands the
//! engine only typed, validated events. The engine never touches sockets.

use std::sync::atomic::AtomicU64;
use std::sync::Arc;

use tokio::sync::mpsc;

/// Receiver clock families, with the parameters every clock-sync decision
/// needs. Values follow mlat-server's `make_clock` table (clocktrack.pyx), which
/// is a decade of field tuning; the jitter is the 1-sigma timestamp noise.
///
/// One deliberate change: the dump1090/readsb (RTL-SDR) jitter is 250 ns, not
/// mlat-server's 500 ns. That value is a nominal starting point, and our
/// network measures it: the clock-pair residual p50 is ~0.15 us, i.e. ~0.22 us
/// sigma for a pair or ~160 ns per receiver, and with 500 ns the solver's own
/// error estimate was twice the real error (|error| / err_h p50 0.39 against
/// 0.83 for a Gaussian, 2026-10-25 replays). A too-wide noise model is not
/// "safe": the track over-smooths and the chi2 tests lose their power. The
/// 2026-10-25 study (branch study/algorithm) halved it: published p50 116 ->
/// 101 m on a 10-minute capture. The other types keep mlat-server's values:
/// they are <0.1% of our receivers and we have no measurement of them; their
/// clock pairs still move their noise down (or up) on their own evidence (see
/// clock.rs).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ClockType {
    /// dump1090/readsb on an RTL-SDR, 12 MHz counter (the vast majority).
    Dump1090,
    /// Mode-S Beast / Radarcape in 12 MHz mode.
    Beast,
    /// Radarcape with 1 GHz GPS-disciplined timestamps.
    RadarcapeGps,
    /// Radarcape 1 GHz without GPS lock.
    Radarcape,
    /// Kinetic SBS-3, 20 MHz.
    Sbs,
}

impl ClockType {
    pub fn parse(s: &str) -> Option<ClockType> {
        Some(match s {
            "dump1090" | "unknown" => ClockType::Dump1090,
            "beast" | "radarcape_12mhz" => ClockType::Beast,
            "radarcape_gps" => ClockType::RadarcapeGps,
            "radarcape" => ClockType::Radarcape,
            "sbs" => ClockType::Sbs,
            _ => return None,
        })
    }

    /// Counter frequency, Hz.
    pub fn freq(self) -> f64 {
        match self {
            ClockType::Dump1090 | ClockType::Beast => 12e6,
            ClockType::RadarcapeGps | ClockType::Radarcape => 1e9,
            ClockType::Sbs => 20e6,
        }
    }

    /// Worst-case relative frequency error (1e-6 = 1 ppm).
    pub fn max_freq_error(self) -> f64 {
        match self {
            ClockType::Dump1090 | ClockType::Sbs => 100e-6,
            ClockType::Beast => 5e-6,
            ClockType::RadarcapeGps | ClockType::Radarcape => 1e-6,
        }
    }

    /// 1-sigma timestamp jitter, seconds.
    pub fn jitter(self) -> f64 {
        match self {
            ClockType::Dump1090 => 250e-9,
            ClockType::Sbs => 500e-9,
            ClockType::Beast => 83e-9,
            ClockType::RadarcapeGps | ClockType::Radarcape => 15e-9,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            ClockType::Dump1090 => "dump1090",
            ClockType::Beast => "beast",
            ClockType::RadarcapeGps => "radarcape_gps",
            ClockType::Radarcape => "radarcape",
            ClockType::Sbs => "sbs",
        }
    }
}

/// Everything the handshake tells us about a receiver.
#[derive(Debug, Clone)]
pub struct ReceiverInfo {
    /// Sanitised user name, as mlat-server would register it.
    pub user: String,
    /// The handshake uuid when it is a string and not a known placeholder.
    pub uuid: Option<String>,
    pub lat: f64,
    pub lon: f64,
    /// metres
    pub alt: f64,
    pub clock: ClockType,
    pub privacy: bool,
    /// Address from the PROXY v1 line when present, else the TCP peer.
    pub source_ip: String,
    /// Port from the same place, a string as mlat-server keeps it.
    pub source_port: String,
    pub client_version: String,
    /// Negotiated compression: "zlib2" | "zlib" | "none".
    pub compress: &'static str,
    /// mlat-server's `connection_info` ("<user> v<version> <clock_type>
    /// <client_version> tcp <compress>"), for clients.json.
    pub connection: String,
    /// Lines this connection has read so far (all kinds, as mlat-server's
    /// `message_counter`), counted by the connection task; the engine turns
    /// it into clients.json's `message_rate`. Never moves for inputs that are
    /// not an mlat-client connection.
    pub msgs: Arc<AtomicU64>,
    /// Set when this server is the receiver's real server (serve mode): the
    /// engine then chooses its traffic and answers it. None = shadow, or an
    /// input that is not an mlat-client (the Beast feed).
    pub serve: Option<ServeOpts>,
}

/// What a served client asked for in its handshake.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServeOpts {
    pub results: ResultFormat,
    /// `return_stats`: a `{"stats": ...}` line every 15 s.
    pub stats: bool,
}

/// mlat-server's two result layouts (`report_mlat_position_old` / `_ecef`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResultFormat {
    None,
    Old,
    Ecef,
}

/// What the engine may ask a connection to do. Only used in serve mode: in
/// shadow mode the proxy discards everything this server writes.
#[derive(Debug, Clone)]
pub enum ToClient {
    StartSending(Vec<u32>),
    StopSending(Vec<u32>),
    /// A pre-serialised `{"result": ...}` line (without the newline).
    ResultLine(String),
    /// A pre-serialised `{"stats": ...}` line (without the newline).
    StatsLine(String),
    /// Close the connection after flushing.
    Close,
}

pub type ConnId = u64;

/// Events from connections to the engine, in arrival order per connection.
#[derive(Debug)]
pub enum Event {
    /// Handshake accepted. `tx` reaches the connection's writer.
    Connect { conn: ConnId, info: ReceiverInfo, tx: mpsc::Sender<ToClient> },
    Disconnect { conn: ConnId },
    /// A DF17 even/odd pair. Times are raw counter ticks of the receiver clock.
    Sync { conn: ConnId, et: f64, ot: f64, em: Vec<u8>, om: Vec<u8> },
    /// A Mode S frame (7 or 14 bytes) to multilaterate; `t` in counter ticks,
    /// `rx` the server wall-clock arrival (unix seconds).
    Mlat { conn: ConnId, t: f64, msg: Vec<u8>, rx: f64 },
    Seen { conn: ConnId, icao: Vec<u32> },
    Lost { conn: ConnId, icao: Vec<u32> },
    RateReport { conn: ConnId, rates: Vec<(u32, f32)> },
    /// The client says its clock was reset / jumped / its input reconnected.
    ClockReset { conn: ConnId, reason: &'static str },
    /// Client reported a new antenna position.
    PositionUpdate { conn: ConnId, lat: f64, lon: f64, alt: f64 },
    /// An aircraft's geometric (GNSS, WGS84 height) minus barometric altitude
    /// from its ADS-B airborne velocity message (TC 19), feet. Not tied to a
    /// receiver: a fact about the aircraft, for the altitude model.
    GeoBaro { icao: u32, delta_ft: i32 },
}
