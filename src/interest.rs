//! Serve mode: which aircraft each served receiver is asked to send.
//!
//! mlat-client forwards nothing by default: the server asks for aircraft with
//! `start_sending` / `stop_sending`, and for a requested aircraft the client
//! itself decides what to send (mlat-client coordinator.py): DF17 even/odd
//! pairs for clock sync when it has a recent ADS-B position of it, its other
//! Mode S replies for multilateration when it does not. So one requested set
//! per receiver covers both uses.
//!
//! The rules are mlat-server's (flyitalyadsb/mlat-server master 33c3361,
//! `tracker.py` update_interest, `coordinator.py` update_interest_sets), a
//! decade of field tuning, ported as they are so that serve mode starts from
//! the traffic shape feeders already send to production:
//!
//! - **mlat**: every receiver that sees an aircraft is asked for it when the
//!   aircraft is "mlat wanted": seen by >= 2 receivers and no ADS-B position
//!   for 120 s, or more than 10% of its sync points rejected, or inside its
//!   periodic 15 s forced window (every ~600 s, staggered per aircraft).
//! - **sync**: from the receiver's rate report (ADS-B positions per second
//!   per aircraft), pairs (aircraft, other receiver) are scored by the product
//!   of the two rates, favouring high aircraft; two greedy rounds add aircraft
//!   until each peer has enough shared rate (0.3, then 3.5) or the receiver's
//!   total rate passes 12 msg/s, and a floor keeps at least 3 aircraft.
//!   Aircraft whose sync points are mostly rejected are skipped in round 1.
//! - A client without rate reports (legacy) is asked for up to 15 random
//!   aircraft it sees. mlat-client sends an empty report right after the
//!   handshake, which is not "no report": it gets the 3-aircraft floor.
//! - A receiver with a bad clock (here `suspect`) is asked for at most 3 sync
//!   aircraft and nothing to multilaterate.
//!
//! Known differences: the random draws use a seeded generator, and ties in
//! the sorted rate pairs are broken by (receiver, aircraft) instead of
//! mlat-server's object comparison.

use std::collections::{HashMap, HashSet};

use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use rand::{Rng, SeedableRng};

pub type RxKey = u64;

#[derive(Debug, Clone)]
pub struct Params {
    /// MLAT_SERVER_MAX_SYNC_AC
    pub max_sync_ac: usize,
    /// MLAT_SERVER_MAX_SYNC_RATE, ADS-B positions per second
    pub max_sync_rate: f64,
    /// MLAT_SERVER_NO_ADSB_MLAT_SECONDS
    pub no_adsb_mlat_s: f64,
    /// MLAT_SERVER_FORCE_MLAT_INTERVAL
    pub force_mlat_interval: f64,
    /// Share of rejected sync points above which an aircraft is
    /// multilaterated anyway (mlat-server: sync_bad_percent > 10).
    pub sync_bad_mlat: f64,
    /// ... and above which it is not picked in the first sync round.
    pub sync_bad_dont_use: f64,
}

impl Default for Params {
    fn default() -> Self {
        Params {
            max_sync_ac: 15,
            max_sync_rate: 12.0,
            no_adsb_mlat_s: 120.0,
            force_mlat_interval: 600.0,
            sync_bad_mlat: 0.10,
            sync_bad_dont_use: 0.25,
        }
    }
}

/// What the selection needs to know about one aircraft.
#[derive(Debug, Clone, Copy)]
pub struct AcMeta {
    pub alt_ft: Option<i32>,
    /// Too many of its sync points rejected: not a first-round sync choice.
    pub sync_dont_use: bool,
}

/// One aircraft's input to the mlat-wanted decision.
#[derive(Debug, Clone, Copy)]
pub struct AcState {
    pub icao: u32,
    /// Receivers that report seeing it.
    pub tracking: usize,
    /// Wall time of its last validated ADS-B position (0 = never).
    pub last_adsb: f64,
    /// Share of its sync points rejected recently (0..1).
    pub sync_bad: f64,
}

/// The per-aircraft periodic forced-mlat window (`last_force_mlat`).
struct Force {
    last: f64,
    on: bool,
}

pub struct Interest {
    pub p: Params,
    rng: StdRng,
    wanted: HashSet<u32>,
    wanted_at: f64,
    force: HashMap<u32, Force>,
}

impl Interest {
    pub fn new(p: Params, seed: u64) -> Interest {
        Interest { p, rng: StdRng::seed_from_u64(seed), wanted: HashSet::new(), wanted_at: f64::NEG_INFINITY, force: HashMap::new() }
    }

    pub fn wanted(&self) -> &HashSet<u32> {
        &self.wanted
    }

    /// Recompute the mlat-wanted set, at most every 0.1 s (mlat-server's
    /// `mlat_wanted_ts`). `aircraft` yields every known aircraft. Returns the
    /// aircraft that just became wanted (empty when nothing was recomputed).
    pub fn refresh_wanted(&mut self, now: f64, aircraft: impl Iterator<Item = AcState>) -> Vec<u32> {
        if now - self.wanted_at <= 0.1 {
            return Vec::new();
        }
        self.wanted_at = now;
        let before = std::mem::take(&mut self.wanted);
        let fi = self.p.force_mlat_interval;
        let mut alive = HashSet::new();
        for a in aircraft {
            alive.insert(a.icao);
            let rng = &mut self.rng;
            let f = self.force.entry(a.icao).or_insert_with(|| Force { last: now - fi * rng.gen::<f64>(), on: false });
            let since = now - f.last;
            if !f.on && since > fi - 15.0 {
                f.on = true;
            }
            if since > fi + 15.0 {
                f.last = now + self.rng.gen::<f64>();
                f.on = false;
            }
            let since = now - f.last;
            let forced = since > fi - 15.0 && since < fi;
            if a.tracking >= 2 && (now - a.last_adsb > self.p.no_adsb_mlat_s || a.sync_bad > self.p.sync_bad_mlat || forced) {
                self.wanted.insert(a.icao);
            }
        }
        self.force.retain(|k, _| alive.contains(k));
        let mut newly: Vec<u32> = self.wanted.difference(&before).copied().collect();
        newly.sort_unstable();
        newly
    }

    /// The aircraft to request from receiver `me`: (sync, mlat).
    ///
    /// `peers(icao)` lists the OTHER receivers that see the aircraft with
    /// their own rate for it (0.8 for a receiver without rate reports, as
    /// mlat-server guesses); `meta(icao)` is None for an unknown aircraft.
    pub fn select(
        &mut self,
        me_bad: bool,
        rate: Option<&HashMap<u32, f32>>,
        tracking: &HashSet<u32>,
        peers: impl Fn(u32) -> Vec<(RxKey, f32)>,
        meta: impl Fn(u32) -> Option<AcMeta>,
    ) -> (HashSet<u32>, HashSet<u32>) {
        let mlat: HashSet<u32> = tracking.iter().filter(|a| self.wanted.contains(a)).copied().collect();
        let mut sync = match rate {
            None => {
                // Legacy client: no rate reports, so no basis to choose.
                let mut v: Vec<u32> = tracking.iter().copied().collect();
                v.sort_unstable();
                if v.len() > self.p.max_sync_ac {
                    v = v.choose_multiple(&mut self.rng, self.p.max_sync_ac).copied().collect();
                }
                v.into_iter().collect()
            }
            Some(rate) => self.select_sync(rate, tracking, &peers, &meta),
        };
        if me_bad {
            // update_interest_sets: bad_syncs > 3 -> 3 sync aircraft, and
            // bad_syncs > 0 -> no multilateration.
            if sync.len() > 3 {
                let mut v: Vec<u32> = sync.into_iter().collect();
                v.sort_unstable();
                sync = v.choose_multiple(&mut self.rng, 3).copied().collect();
            }
            return (sync, HashSet::new());
        }
        (sync, mlat)
    }

    fn select_sync(
        &mut self,
        rate: &HashMap<u32, f32>,
        tracking: &HashSet<u32>,
        peers: &impl Fn(u32) -> Vec<(RxKey, f32)>,
        meta: &impl Fn(u32) -> Option<AcMeta>,
    ) -> HashSet<u32> {
        // (rate product, other receiver, aircraft, my rate for it)
        let mut pairs: Vec<(f64, RxKey, u32, f64)> = Vec::new();
        let mut by_ac: HashMap<u32, Vec<(f64, RxKey)>> = HashMap::new();
        let mut dont_use: HashSet<u32> = HashSet::new();
        let mut icaos: Vec<(&u32, &f32)> = rate.iter().collect();
        icaos.sort_unstable_by_key(|(a, _)| **a);
        for (&icao, &r) in icaos {
            let Some(m) = meta(icao) else { continue };
            if m.sync_dont_use {
                dont_use.insert(icao);
            }
            let r = r as f64;
            let alt_factor = m.alt_ft.filter(|&a| a > 0).map(|a| 1.0 + (a as f64 / 20_000.0).powf(1.5));
            let l = by_ac.entry(icao).or_default();
            for (r1, rate1) in peers(icao) {
                let mut rp = r * rate1 as f64 / 2.25;
                if let Some(f) = alt_factor {
                    rp *= f;
                }
                if rp < 0.01 {
                    continue;
                }
                l.push((rp, r1));
                pairs.push((rp, r1, icao, r));
            }
        }
        pairs.sort_by(|a, b| b.0.total_cmp(&a.0).then(b.1.cmp(&a.1)).then(b.2.cmp(&a.2)));
        let mut first: Vec<(f64, RxKey, u32, f64)> = pairs[..pairs.len() / 2].to_vec();
        first.shuffle(&mut self.rng);

        let mut ntotal: HashMap<RxKey, f64> = HashMap::new();
        let mut sync: HashSet<u32> = HashSet::new();
        let mut total_rate = 0.0;
        let take = |sync: &mut HashSet<u32>, ntotal: &mut HashMap<RxKey, f64>, total: &mut f64, icao: u32, r: f64| {
            sync.insert(icao);
            *total += r;
            for &(rp2, r2) in &by_ac[&icao] {
                *ntotal.entry(r2).or_insert(0.0) += rp2;
            }
        };
        for &(_, r1, icao, r) in &first {
            if sync.contains(&icao) || dont_use.contains(&icao) {
                continue;
            }
            if total_rate > self.p.max_sync_rate {
                break;
            }
            if ntotal.get(&r1).copied().unwrap_or(0.0) < 0.3 {
                take(&mut sync, &mut ntotal, &mut total_rate, icao, r);
            }
        }
        for &(_, r1, icao, r) in &pairs {
            if sync.contains(&icao) {
                continue;
            }
            if total_rate > self.p.max_sync_rate {
                break;
            }
            if ntotal.get(&r1).copied().unwrap_or(0.0) < 3.5 {
                take(&mut sync, &mut ntotal, &mut total_rate, icao, r);
            }
        }
        // Floor: at least max_sync_ac / 4 aircraft, from the rate report
        // first, then from anything the receiver sees.
        let floor = self.p.max_sync_ac / 4;
        if sync.len() < floor {
            let mut avail: Vec<u32> = by_ac.keys().filter(|a| !sync.contains(a)).copied().collect();
            avail.sort_unstable();
            let k = (floor - sync.len()).min(avail.len());
            sync.extend(avail.choose_multiple(&mut self.rng, k).copied());
            if sync.len() < floor {
                let mut avail: Vec<u32> = tracking.iter().filter(|a| !sync.contains(a)).copied().collect();
                avail.sort_unstable();
                let k = (floor - sync.len()).min(avail.len());
                sync.extend(avail.choose_multiple(&mut self.rng, k).copied());
            }
        }
        sync
    }
}

/// The start_sending / stop_sending lists that move `requested` to `wanted`
/// (mlat-server's send_traffic_updates), sorted.
pub fn diff(requested: &HashSet<u32>, wanted: &HashSet<u32>) -> (Vec<u32>, Vec<u32>) {
    let mut start: Vec<u32> = wanted.difference(requested).copied().collect();
    let mut stop: Vec<u32> = requested.difference(wanted).copied().collect();
    start.sort_unstable();
    stop.sort_unstable();
    (start, stop)
}

#[cfg(test)]
mod tests;
