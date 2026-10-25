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

use std::collections::HashMap;
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

#[derive(Clone, Serialize, Deserialize)]
struct Map {
    lat: f64,
    lon: f64,
    present: Vec<f32>,
    heard: Vec<f32>,
    decayed: f64,
}

impl Map {
    fn new(lat: f64, lon: f64, now: f64) -> Map {
        Map { lat, lon, present: vec![0.0; CELLS], heard: vec![0.0; CELLS], decayed: now }
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

/// (sector, band, ring) of a point seen from a receiver; None beyond range.
fn bin(rlat: f64, rlon: f64, lat: f64, lon: f64, alt_m: f64) -> Option<(usize, usize, usize, f64)> {
    let d = haversine_m(rlat, rlon, lat, lon);
    if !d.is_finite() || d >= MAX_RANGE_M {
        return None;
    }
    let (p1, p2) = (rlat.to_radians(), lat.to_radians());
    let dl = (lon - rlon).to_radians();
    let az = (dl.sin() * p2.cos()).atan2(p1.cos() * p2.sin() - p1.sin() * p2.cos() * dl.cos()).to_degrees();
    let sector = (((az + 360.0) % 360.0) / SECTOR_DEG) as usize % SECTORS;
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
    /// A moved antenna starts a new map.
    pub fn place(&mut self, id: CovId, lat: f64, lon: f64, now: f64) {
        self.unplace(id);
        let moved = self.maps.get(&id).is_some_and(|m| haversine_m(m.lat, m.lon, lat, lon) > 500.0);
        if moved || !self.maps.contains_key(&id) {
            self.maps.insert(id, Map::new(lat, lon, now));
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

    pub fn receivers(&self) -> usize {
        self.maps.len()
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
        for id in self.near(lat, lon) {
            let m = self.maps.get_mut(&id).unwrap();
            m.decay(now);
            if let Some((s, b, r, _)) = bin(m.lat, m.lon, lat, lon, alt_m) {
                let i = idx(s, b, r);
                m.present[i] += 1.0;
                if heard(id) {
                    m.heard[i] += 1.0;
                }
            }
        }
        self.samples += 1;
        true
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
        #[derive(Serialize)]
        struct Out {
            id: String,
            lat: f64,
            lon: f64,
            decayed: f64,
            bins: Vec<(u32, f32, f32)>,
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
            })
            .collect();
        let tmp = format!("{path}.tmp");
        let f = std::fs::File::create(&tmp)?;
        let mut gz = flate2::write::GzEncoder::new(f, flate2::Compression::fast());
        gz.write_all(serde_json::to_string(&serde_json::json!({"version": 1, "maps": maps}))?.as_bytes())?;
        gz.finish()?;
        std::fs::rename(tmp, path)?;
        Ok(())
    }

    /// Load maps saved by `save`; receivers are placed again when they connect.
    pub fn load(path: &str) -> anyhow::Result<Coverage> {
        #[derive(Deserialize)]
        struct In {
            id: String,
            lat: f64,
            lon: f64,
            decayed: f64,
            bins: Vec<(u32, f32, f32)>,
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
