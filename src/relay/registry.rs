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

/// Several registries in one (`--feed-receivers-url` with more than one URL): on the same id the later
/// source wins. The device gateway's registry (receivers with no mlat-client, positions averaged from
/// their own GNSS) goes after this instance's own.
pub fn merge(parts: &[HashMap<u64, Receiver>]) -> HashMap<u64, Receiver> {
    let mut m = HashMap::default();
    for part in parts {
        for (k, r) in part {
            m.insert(*k, r.clone());
        }
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(key: &str, lat: f64, clock: &str) -> Entry {
        Entry { key: key.into(), user: "u".into(), lat, lon: 10.0, alt: 100.0, clock: clock.into(), privacy: true }
    }

    #[test]
    fn merge_later_source_wins_and_keeps_the_rest() {
        let own = parse(&Doc { generated: 0.0, receivers: vec![entry("00000000000000aa", 45.0, "dump1090"),
                                                               entry("00000000000000bb", 46.0, "beast")] });
        let devices = parse(&Doc { generated: 0.0, receivers: vec![entry("00000000000000bb", 46.5, "beast"),
                                                                   entry("abcdef0102038f1a", 47.0, "beast")] });
        let m = merge(&[own, devices]);
        assert_eq!(m.len(), 3);
        assert_eq!(m[&0xaa].lat, 45.0);
        assert_eq!(m[&0xbb].lat, 46.5);
        assert_eq!(m[&0xabcdef0102038f1a].clock, ClockType::Beast);
        assert!(merge(&[]).is_empty());
    }

    #[test]
    fn parse_skips_bad_entries() {
        let mut at_null_island = entry("00000000000000cc", 0.0, "beast");
        at_null_island.lon = 0.0; // 0,0 = no position
        let m = parse(&Doc { generated: 0.0, receivers: vec![entry("short", 45.0, "beast"),
                                                             at_null_island,
                                                             entry("00000000000000dd", 45.0, "nope")] });
        assert!(m.is_empty());
    }
}
