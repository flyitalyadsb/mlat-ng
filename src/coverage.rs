//! Receiver coverage maps: where each receiver hears aircraft, learned from
//! the aircraft that broadcast their position.
//!
//! For every ADS-B position the engine validates (a sync point), sampled once
//! per aircraft every few seconds, each receiver within `MAX_RANGE_M` gets one
//! "present" count in its polar bin (20 deg sector x altitude band x 25 km
//! range ring), and one "heard" count too if the aircraft is in its seen set.
//! P(hear | bin) = heard / present: it knows where aircraft were and the
//! receiver did NOT hear them, which is what terrain, buildings and antennas
//! do to coverage. Counts decay with a half-life of a day so the maps follow
//! an antenna that changes.
//!
//! Why: an aircraft heard by only 2-3 receivers leaves the position free
//! along a curve. Requiring the receivers that heard it to cover the point,
//! and the receivers that did not to not cover it, rules out most of the
//! curve: on 2026-10-24 (polar maps from 5 minutes), the best point along
//! the curve of 1,047 genuinely 2-receiver aircraft went from 277 km p50
//! (no information) to 27 km.
//!
//! Maps are keyed by a stable receiver identity (UUID key, else the user
//! name), not by connection, so they survive reconnects; they can be saved
//! to and reloaded from a gzip'd JSON file.
//!
//! Each map also keeps a few daily facts for the receiver's own page
//! (Lightcone): the farthest aircraft it heard, the aircraft only it heard,
//! and the MLAT positions it helped publish, for today and yesterday (UTC).
//! The HTTP side reads them, and the maps, through `map_json` and
//! `summary_json`, which run on the engine thread.

use crate::hashing::{HashMap, HashSet};
use std::io::{Read, Write};

use serde::{Deserialize, Serialize};

use crate::geo::haversine_m;

pub type CovId = u64;

pub const SECTOR_DEG: f64 = 20.0;
pub const SECTORS: usize = 18;
/// Altitude band lower edges, metres.
pub const BANDS_M: [f64; 5] = [0.0, 1500.0, 3000.0, 6000.0, 9000.0];
pub const RING_M: f64 = 25_000.0;
pub const RINGS: usize = 20;
pub const MAX_RANGE_M: f64 = RING_M * RINGS as f64;
const CELLS: usize = SECTORS * BANDS_M.len() * RINGS;
/// One sample per aircraft this often.
pub const SAMPLE_EVERY: f64 = 5.0;
const HALF_LIFE: f64 = 24.0 * 3600.0;
/// Decay is applied lazily, at most this often per map.
const DECAY_EVERY: f64 = 600.0;
/// A bin needs this many "present" counts before it speaks for itself.
const MIN_PRESENT: f32 = 4.0;
/// Index cell size for finding receivers near a point, degrees.
const INDEX_DEG: f64 = 2.0;

/// One UTC day of a receiver's facts.
#[derive(Clone, Default, Serialize, Deserialize, PartialEq, Debug)]
pub struct DayStats {
    /// Days since the epoch (UTC).
    pub day: i64,
    /// Farthest aircraft heard: distance, altitude (m), bearing (deg), when.
    pub far_m: f32,
    pub far_alt_m: f32,
    pub far_az: f32,
    pub far_t: f64,
    /// Aircraft that no other placed receiver in range heard.
    pub unique: u32,
    /// MLAT positions published with this receiver's data.
    pub fixes: u32,
    /// Coverage samples counted ("present") and heard.
    pub samples: u32,
    pub heard: u32,
}

impl DayStats {
    fn new(day: i64) -> DayStats {
        DayStats { day, ..Default::default() }
    }
}

fn day_of(t: f64) -> i64 {
    (t / 86_400.0).floor() as i64
}

#[derive(Clone, Serialize, Deserialize)]
struct Map {
    lat: f64,
    lon: f64,
    present: Vec<f32>,
    heard: Vec<f32>,
    decayed: f64,
    /// Who this map belongs to, as the HTTP side asks for it:
    /// "uuid:<16 hex>" or "user:<name>". Empty until the engine names it.
    who: String,
    today: DayStats,
    yesterday: DayStats,
    /// Aircraft already counted in `today.unique`.
    unique_seen: HashSet<u32>,
}

impl Map {
    fn new(lat: f64, lon: f64, now: f64) -> Map {
        Map {
            lat,
            lon,
            present: vec![0.0; CELLS],
            heard: vec![0.0; CELLS],
            decayed: now,
            who: String::new(),
            today: DayStats::new(day_of(now)),
            yesterday: DayStats::default(),
            unique_seen: HashSet::default(),
        }
    }

    /// Start a new day if `now` is past the current one.
    fn roll(&mut self, now: f64) {
        let d = day_of(now);
        if d != self.today.day {
            self.yesterday = if d == self.today.day + 1 { std::mem::take(&mut self.today) } else { DayStats::default() };
            self.today = DayStats::new(d);
            self.unique_seen.clear();
        }
    }

    /// The decay factor from the last applied decay to `now`, without
    /// applying it (for readers).
    fn decay_factor(&self, now: f64) -> f32 {
        let dt = (now - self.decayed).max(0.0);
        (-dt * std::f64::consts::LN_2 / HALF_LIFE).exp() as f32
    }

    fn decay(&mut self, now: f64) {
        let dt = now - self.decayed;
        if dt < DECAY_EVERY {
            return;
        }
        let f = (-dt * std::f64::consts::LN_2 / HALF_LIFE).exp() as f32;
        for v in self.present.iter_mut().chain(self.heard.iter_mut()) {
            *v *= f;
        }
        self.decayed = now;
    }
}

/// Bearing from a receiver to a point, degrees in [0, 360).
fn bearing(rlat: f64, rlon: f64, lat: f64, lon: f64) -> f64 {
    let (p1, p2) = (rlat.to_radians(), lat.to_radians());
    let dl = (lon - rlon).to_radians();
    let az = (dl.sin() * p2.cos()).atan2(p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos()).to_degrees();
    (az + 360.0) % 360.0
}

/// (sector, band, ring) of a point seen from a receiver; None beyond range.
fn bin(rlat: f64, rlon: f64, lat: f64, lon: f64, alt_m: f64) -> Option<(usize, usize, usize, f64)> {
    let d = haversine_m(rlat, rlon, lat, lon);
    if !d.is_finite() || d >= MAX_RANGE_M {
        return None;
    }
    let sector = (bearing(rlat, rlon, lat, lon) / SECTOR_DEG) as usize % SECTORS;
    let band = BANDS_M.iter().rposition(|b| alt_m >= *b).unwrap_or(0);
    Some((sector, band, (d / RING_M) as usize, d))
}

fn idx(sector: usize, band: usize, ring: usize) -> usize {
    (sector * BANDS_M.len() + band) * RINGS + ring
}

#[derive(Default)]
pub struct Coverage {
    maps: HashMap<CovId, Map>,
    /// Receivers placed on the map right now, by index cell.
    index: HashMap<(i32, i32), Vec<CovId>>,
    placed: HashMap<CovId, (i32, i32)>,
    last_sample: HashMap<u32, f64>,
    pub samples: u64,
}

fn cell_of(lat: f64, lon: f64) -> (i32, i32) {
    ((lat / INDEX_DEG).floor() as i32, (lon / INDEX_DEG).floor() as i32)
}

impl Coverage {
    /// A receiver is at (lat, lon) and can hear: index it (again, if moved).
    /// A moved antenna starts a new map (its name and daily facts carry over).
    pub fn place(&mut self, id: CovId, lat: f64, lon: f64, now: f64) {
        self.unplace(id);
        let moved = self.maps.get(&id).is_some_and(|m| haversine_m(m.lat, m.lon, lat, lon) > 500.0);
        if moved || !self.maps.contains_key(&id) {
            let mut m = Map::new(lat, lon, now);
            if let Some(old) = self.maps.remove(&id) {
                m.who = old.who;
                m.today = old.today;
                m.yesterday = old.yesterday;
                m.unique_seen = old.unique_seen;
            }
            self.maps.insert(id, m);
        }
        let c = cell_of(lat, lon);
        self.index.entry(c).or_default().push(id);
        self.placed.insert(id, c);
    }

    /// The receiver went away: its map is kept for when it comes back.
    pub fn unplace(&mut self, id: CovId) {
        if let Some(c) = self.placed.remove(&id) {
            if let Some(v) = self.index.get_mut(&c) {
                v.retain(|x| *x != id);
            }
        }
    }

    /// Record who a map belongs to (see `Map::who`).
    pub fn name(&mut self, id: CovId, who: &str) {
        if let Some(m) = self.maps.get_mut(&id) {
            if m.who != who {
                m.who = who.to_string();
            }
        }
    }

    pub fn receivers(&self) -> usize {
        self.maps.len()
    }

    /// An MLAT position was published with this receiver's data.
    pub fn count_fix(&mut self, id: CovId, now: f64) {
        if let Some(m) = self.maps.get_mut(&id) {
            m.roll(now);
            m.today.fixes += 1;
        }
    }

    /// Placed receivers within `MAX_RANGE_M` of a point.
    pub fn near(&self, lat: f64, lon: f64) -> Vec<CovId> {
        let dlat = (MAX_RANGE_M / 111_000.0 / INDEX_DEG).ceil() as i32 + 1;
        let dlon = (MAX_RANGE_M / (111_000.0 * lat.to_radians().cos().max(0.2)) / INDEX_DEG).ceil() as i32 + 1;
        let (ci, cj) = cell_of(lat, lon);
        let mut out = Vec::new();
        for i in ci - dlat..=ci + dlat {
            for j in cj - dlon..=cj + dlon {
                if let Some(v) = self.index.get(&(i, j)) {
                    for id in v {
                        let m = &self.maps[id];
                        if haversine_m(m.lat, m.lon, lat, lon) < MAX_RANGE_M {
                            out.push(*id);
                        }
                    }
                }
            }
        }
        out
    }

    /// An ADS-B aircraft is at (lat, lon, alt): count it for every placed
    /// receiver in range, as heard where `heard(id)` says so. Sampled once
    /// per aircraft every `SAMPLE_EVERY` seconds; returns whether it counted.
    pub fn observe(&mut self, icao: u32, lat: f64, lon: f64, alt_m: f64, now: f64, heard: impl Fn(CovId) -> bool) -> bool {
        if self.last_sample.get(&icao).is_some_and(|t| now - t < SAMPLE_EVERY) {
            return false;
        }
        self.last_sample.insert(icao, now);
        let mut heard_by: Option<CovId> = None;
        let mut n_heard = 0usize;
        for id in self.near(lat, lon) {
            let m = self.maps.get_mut(&id).unwrap();
            m.decay(now);
            m.roll(now);
            if let Some((s, b, r, d)) = bin(m.lat, m.lon, lat, lon, alt_m) {
                let i = idx(s, b, r);
                m.present[i] += 1.0;
                m.today.samples += 1;
                if heard(id) {
                    m.heard[i] += 1.0;
                    m.today.heard += 1;
                    n_heard += 1;
                    heard_by = Some(id);
                    if d as f32 > m.today.far_m {
                        m.today.far_m = d as f32;
                        m.today.far_alt_m = alt_m as f32;
                        m.today.far_az = bearing(m.lat, m.lon, lat, lon) as f32;
                        m.today.far_t = now;
                    }
                }
            }
        }
        if let (1, Some(id)) = (n_heard, heard_by) {
            let m = self.maps.get_mut(&id).unwrap();
            if m.unique_seen.insert(icao) {
                m.today.unique += 1;
            }
        }
        self.samples += 1;
        true
    }

    /// One map for the HTTP side: decayed bins with `present >= 0.5`
    /// as [sector, band, ring, present, heard], and the daily facts.
    /// Never the position.
    pub fn map_json(&self, id: CovId, now: f64) -> Option<serde_json::Value> {
        let m = self.maps.get(&id)?;
        let f = m.decay_factor(now);
        let r1 = |x: f32| (x as f64 * 10.0).round() / 10.0;
        let mut bins = Vec::new();
        let mut samples = 0.0f32;
        for s in 0..SECTORS {
            for b in 0..BANDS_M.len() {
                for k in 0..RINGS {
                    let i = idx(s, b, k);
                    let (p, h) = (m.present[i] * f, m.heard[i] * f);
                    samples += p;
                    if p >= 0.5 {
                        bins.push(serde_json::json!([s, b, k, r1(p), r1(h)]));
                    }
                }
            }
        }
        let (today, yesterday) = self.days(m, now);
        Some(serde_json::json!({
            "id": format!("{id:016x}"),
            "who": m.who,
            "placed": self.placed.contains_key(&id),
            "updated": now,
            "half_life_h": HALF_LIFE / 3600.0,
            "sector_deg": SECTOR_DEG,
            "ring_km": RING_M / 1000.0,
            "bands_m": BANDS_M.iter().enumerate().map(|(i, lo)| {
                [*lo, BANDS_M.get(i + 1).copied().unwrap_or(13_000.0)]
            }).collect::<Vec<_>>(),
            "samples": (samples as f64).round(),
            "bins": bins,
            "today": today,
            "yesterday": yesterday,
        }))
    }

    /// Every named map's daily facts and whether it is placed now, for the
    /// hourly snapshot that keeps records and uptime.
    pub fn summary_json(&self, now: f64) -> serde_json::Value {
        let rows: Vec<serde_json::Value> = self
            .maps
            .iter()
            .filter(|(_, m)| !m.who.is_empty())
            .map(|(id, m)| {
                let (today, yesterday) = self.days(m, now);
                serde_json::json!({
                    "id": format!("{id:016x}"),
                    "who": m.who,
                    "placed": self.placed.contains_key(id),
                    "today": today,
                    "yesterday": yesterday,
                })
            })
            .collect();
        serde_json::json!({"now": now, "receivers": rows})
    }

    /// Today's and yesterday's facts as of `now`, also for a map that has
    /// not rolled over yet because nothing reached it since midnight.
    fn days(&self, m: &Map, now: f64) -> (DayStats, DayStats) {
        let d = day_of(now);
        if m.today.day == d {
            (m.today.clone(), m.yesterday.clone())
        } else if m.today.day == d - 1 {
            (DayStats::new(d), m.today.clone())
        } else {
            (DayStats::new(d), DayStats::default())
        }
    }

    /// P(the receiver hears an aircraft at this point), or None without a
    /// map. Sparse bins fall back to the receiver's ring average over all
    /// sectors for that band; beyond the farthest ring it has heard in that
    /// sector and band (+50 km) it is small, and beyond range it is ~0.
    pub fn p_hear(&self, id: CovId, lat: f64, lon: f64, alt_m: f64) -> Option<f64> {
        let m = self.maps.get(&id)?;
        let Some((s, b, r, _)) = bin(m.lat, m.lon, lat, lon, alt_m) else { return Some(1e-6) };
        let i = idx(s, b, r);
        let (pr, hd) = (m.present[i], m.heard[i]);
        let p = if pr >= MIN_PRESENT {
            (hd as f64 + 0.5) / (pr as f64 + 1.0)
        } else {
            let (mut sp, mut sh) = (0.0f32, 0.0f32);
            for sec in 0..SECTORS {
                let j = idx(sec, b, r);
                sp += m.present[j];
                sh += m.heard[j];
            }
            if sp >= MIN_PRESENT {
                (sh as f64 + 0.5) / (sp as f64 + 1.0)
            } else {
                0.2
            }
        };
        let farthest = (0..RINGS).rev().find(|&k| m.heard[idx(s, b, k)] > 0.5);
        if farthest.is_some_and(|f| r > f + 2) {
            return Some(1e-4);
        }
        Some(p.clamp(1e-6, 0.995))
    }

    /// Log-likelihood of a candidate position given who heard the aircraft
    /// and which placed receivers in range did not (each silent receiver
    /// counts as `n_eff` independent misses). Receivers without a map add
    /// nothing.
    pub fn log_lik(&self, heard: &[CovId], silent: &[CovId], lat: f64, lon: f64, alt_m: f64, n_eff: f64) -> f64 {
        let mut s = 0.0;
        for id in heard {
            if let Some(p) = self.p_hear(*id, lat, lon, alt_m) {
                s += p.ln();
            }
        }
        for id in silent {
            if let Some(p) = self.p_hear(*id, lat, lon, alt_m) {
                s += n_eff * (1.0 - p).ln();
            }
        }
        s
    }

    /// Save every map (gzip'd JSON). Only non-empty bins are written.
    pub fn save(&self, path: &str) -> anyhow::Result<()> {
        crate::persist::write_atomic(path, &self.encode()?)
    }

    /// The maps as `save` writes them, in memory (the engine encodes on its
    /// thread and leaves the disk to another one).
    pub fn encode(&self) -> anyhow::Result<Vec<u8>> {
        #[derive(Serialize)]
        struct Out<'a> {
            id: String,
            lat: f64,
            lon: f64,
            decayed: f64,
            bins: Vec<(u32, f32, f32)>,
            who: &'a str,
            today: &'a DayStats,
            yesterday: &'a DayStats,
            unique_seen: Vec<u32>,
        }
        let maps: Vec<Out> = self
            .maps
            .iter()
            .map(|(id, m)| Out {
                id: format!("{id:016x}"),
                lat: m.lat,
                lon: m.lon,
                decayed: m.decayed,
                bins: (0..CELLS).filter(|&i| m.present[i] > 0.0).map(|i| (i as u32, m.present[i], m.heard[i])).collect(),
                who: &m.who,
                today: &m.today,
                yesterday: &m.yesterday,
                unique_seen: m.unique_seen.iter().copied().collect(),
            })
            .collect();
        let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::fast());
        gz.write_all(serde_json::to_string(&serde_json::json!({"version": 1, "maps": maps}))?.as_bytes())?;
        Ok(gz.finish()?)
    }

    /// Load maps saved by `save`; receivers are placed again when they connect.
    pub fn load(path: &str) -> anyhow::Result<Coverage> {
        // The daily facts came later (2026-10-26): files from before have
        // none, and load with empty ones.
        #[derive(Deserialize)]
        struct In {
            id: String,
            lat: f64,
            lon: f64,
            decayed: f64,
            bins: Vec<(u32, f32, f32)>,
            #[serde(default)]
            who: String,
            #[serde(default)]
            today: DayStats,
            #[serde(default)]
            yesterday: DayStats,
            #[serde(default)]
            unique_seen: Vec<u32>,
        }
        #[derive(Deserialize)]
        struct Doc {
            maps: Vec<In>,
        }
        let mut s = String::new();
        flate2::read::GzDecoder::new(std::fs::File::open(path)?).read_to_string(&mut s)?;
        let doc: Doc = serde_json::from_str(&s)?;
        let mut c = Coverage::default();
        for m in doc.maps {
            let Ok(id) = u64::from_str_radix(&m.id, 16) else { continue };
            let mut map = Map::new(m.lat, m.lon, m.decayed);
            map.who = m.who;
            if m.today.day != 0 {
                map.today = m.today;
                map.yesterday = m.yesterday;
                map.unique_seen = m.unique_seen.into_iter().collect();
            }
            for (i, p, h) in m.bins {
                if (i as usize) < CELLS {
                    map.present[i as usize] = p;
                    map.heard[i as usize] = h;
                }
            }
            c.maps.insert(id, map);
        }
        Ok(c)
    }
}

#[cfg(test)]
mod tests;
