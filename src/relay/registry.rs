//! Where the feed's receivers are. The Beast feed carries only a receiver id
//! (the first 64 bits of the feeder's UUID); positions come from the
//! mlat-client handshakes of the same feeders, which the primary mlat-ng
//! publishes as `/receivers.json` (cluster-internal: it holds antenna
//! positions). Replay reads the same format from a file.

use crate::hashing::HashMap;

use serde::{Deserialize, Serialize};

use crate::types::ClockType;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Entry {
    /// 16 lowercase hex digits: the UUID without dashes, first 64 bits.
    pub key: String,
    pub user: String,
    pub lat: f64,
    pub lon: f64,
    /// metres
    pub alt: f64,
    pub clock: String,
    #[serde(default)]
    pub privacy: bool,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Doc {
    #[serde(default)]
    pub generated: f64,
    pub receivers: Vec<Entry>,
}

/// The relay id for a handshake UUID, as readsb derives it: hex digits only,
/// dashes skipped, the first 16 make the id. None when there are fewer.
pub fn key_of_uuid(uuid: &str) -> Option<u64> {
    let hex: String = uuid.chars().filter(|c| *c != '-' && *c != ' ').take(16).collect();
    if hex.len() < 16 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    u64::from_str_radix(&hex, 16).ok()
}

#[derive(Debug, Clone)]
pub struct Receiver {
    pub user: String,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub clock: ClockType,
    pub privacy: bool,
}

/// Parse a registry document into id -> receiver. Entries with a bad key, an
/// impossible position or an unknown clock type are skipped.
pub fn parse(doc: &Doc) -> HashMap<u64, Receiver> {
    let mut m = HashMap::default();
    for e in &doc.receivers {
        let Ok(k) = u64::from_str_radix(&e.key, 16) else { continue };
        if e.key.len() != 16 || !(-90.0..=90.0).contains(&e.lat) || !(-180.0..=180.0).contains(&e.lon) {
            continue;
        }
        if e.lat == 0.0 && e.lon == 0.0 {
            continue;
        }
        let Some(clock) = ClockType::parse(&e.clock) else { continue };
        m.insert(k, Receiver { user: e.user.clone(), lat: e.lat, lon: e.lon, alt: e.alt, clock, privacy: e.privacy });
    }
    m
}
