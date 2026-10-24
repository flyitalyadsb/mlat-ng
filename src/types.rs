//! Types shared between the network layer (proto) and the engine.
//!
//! The network layer parses everything off the engine thread and hands the
//! engine only typed, validated events. The engine never touches sockets.

use tokio::sync::mpsc;

/// Receiver clock families, with the parameters every clock-sync decision
/// needs. Values follow mlat-server's `make_clock` table (clocktrack.pyx), which
/// is a decade of field tuning; the jitter is the 1-sigma timestamp noise.
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
            ClockType::Dump1090 | ClockType::Sbs => 500e-9,
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
    pub client_version: String,
    /// Negotiated compression: "zlib2" | "zlib" | "none".
    pub compress: &'static str,
}

/// What the engine may ask a connection to do. Only used in serve mode: in
/// shadow mode the proxy discards everything this server writes.
#[derive(Debug, Clone)]
pub enum ToClient {
    StartSending(Vec<u32>),
    StopSending(Vec<u32>),
    /// A pre-serialised `{"result": ...}` line (without the newline).
    ResultLine(String),
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
}
