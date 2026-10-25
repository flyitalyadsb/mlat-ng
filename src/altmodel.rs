//! Barometric -> geometric altitude: a regional model of the difference,
//! learned online from ADS-B.
//!
//! Every altitude an aircraft reports in Mode S (AC13, AC12, Mode C) is
//! pressure altitude: height in the ICAO standard atmosphere for 1013.25 hPa.
//! Multilateration works in WGS84 geometry, and used to take pressure altitude
//! as height above the ellipsoid, in two places: the positions of the ADS-B
//! sync references (clock.rs learns its offsets from them) and the altitude
//! constraint of the solver and the track. The two differ by the local
//! pressure (~27 ft/hPa), the air temperature against the standard (a few %
//! of the altitude) and the geoid. On a 60 s capture of our network
//! (2026-10-24, ~4,700 aircraft) the geometric-minus-baro difference that
//! ADS-B airborne velocity messages carry (TC 19, GNSS height above the
//! ellipsoid minus baro) was a median 950 ft (~290 m) at FL330 and above,
//! p90 1775 ft; 250 ft p50 below 5,000 ft. A 290 m vertical error biases a
//! sync reference's ranges by 290 m x sin(elevation), different at each
//! receiver: tens to ~100 m of range difference.
//!
//! The difference is smooth in space and in level, so the aircraft that
//! broadcast it describe it for the ones that do not: each estimate is a
//! weighted linear fit delta = a + b * baro over the samples within
//! `RADIUS_KM`, weights 1 / (1 + (d / 50 km)^2) x exp(-age / `TAU_S`), with a
//! ridge prior on the slope (one flight level alone must not extrapolate
//! wildly). Left-one-out on that capture it predicted an aircraft's own
//! difference to 66 ft p50 / 249 ft p90 (575 / 1600 ft uncorrected). With it
//! the sync references and the constraint are at geometric height: holdout
//! p50 66 -> 56 m, published p50 -1.5 m on a 10-minute capture.
//!
//! Storage: 1 x 1 degree cells holding the latest sample of each aircraft
//! (an aircraft that moves on leaves its old cell), at most `MAX_PER_CELL`
//! per cell (the oldest goes) and none older than `MAX_AGE_S`: memory is
//! bounded by the traffic, not by time.
//!
//! An estimate can exclude one aircraft: the multilateration of an ADS-B
//! aircraft (the bench) must not use its own TC 19, which a Mode-S-only
//! aircraft never has. With fewer than `MIN_SAMPLES` neighbours there is no
//! estimate: the caller falls back to a zero correction with `fallback_sigma`.

use std::collections::HashMap;

/// Neighbourhood of an estimate (km).
const RADIUS_KM: f64 = 150.0;
/// Distance at which a sample's weight halves (km).
const HALF_KM: f64 = 50.0;
/// Time constant of the weight decay (s): pressure and temperature fields
/// change over hours; ten minutes keeps the model current.
const TAU_S: f64 = 600.0;
/// Samples older than this are dropped (weight e^-3 by then).
pub const MAX_AGE_S: f64 = 1800.0;
/// Bound on samples per cell (the newest are kept).
const MAX_PER_CELL: usize = 64;
/// Fewer neighbours than this: no estimate.
const MIN_SAMPLES: usize = 4;
/// Network-wide slope (ft per 10,000 ft of baro altitude) from the same
/// capture, and the ridge weight pulling the fitted slope towards it.
const SLOPE_PRIOR: f64 = 196.0;
const SLOPE_RIDGE: f64 = 0.5;
/// 1-sigma of a model estimate (m): its left-one-out error, ~20 m p50.
pub const MODEL_SIGMA_M: f64 = 30.0;

/// One aircraft's geometric-minus-baro difference at a place and level.
#[derive(Debug, Clone, Copy)]
struct Sample {
    icao: u32,
    lat: f64,
    lon: f64,
    baro_ft: f64,
    delta_ft: f64,
    t: f64,
}

#[derive(Default)]
pub struct AltModel {
    cells: HashMap<(i32, i32), Vec<Sample>>,
    cell_of: HashMap<u32, (i32, i32)>,
}

fn cell(lat: f64, lon: f64) -> (i32, i32) {
    (lat.floor() as i32, lon.floor() as i32)
}

impl AltModel {
    /// Record an aircraft's difference (ft) at its position and baro level.
    pub fn insert(&mut self, icao: u32, lat: f64, lon: f64, baro_ft: f64, delta_ft: f64, t: f64) {
        if !(lat.is_finite() && lon.is_finite() && baro_ft.is_finite() && delta_ft.is_finite()) {
            return;
        }
        let c = cell(lat, lon);
        if let Some(old) = self.cell_of.insert(icao, c) {
            if let Some(v) = self.cells.get_mut(&old) {
                v.retain(|s| s.icao != icao);
                if v.is_empty() {
                    self.cells.remove(&old);
                }
            }
        }
        let v = self.cells.entry(c).or_default();
        v.push(Sample { icao, lat, lon, baro_ft, delta_ft, t });
        if v.len() > MAX_PER_CELL {
            // The oldest sample goes (and its aircraft is no longer stored).
            let (i, _) = v.iter().enumerate().min_by(|a, b| a.1.t.total_cmp(&b.1.t)).unwrap();
            let gone = v.swap_remove(i);
            self.cell_of.remove(&gone.icao);
        }
    }

    /// Drop samples older than `MAX_AGE_S`.
    pub fn expire(&mut self, now: f64) {
        let cell_of = &mut self.cell_of;
        self.cells.retain(|_, v| {
            v.retain(|s| {
                let keep = now - s.t <= MAX_AGE_S;
                if !keep {
                    cell_of.remove(&s.icao);
                }
                keep
            });
            !v.is_empty()
        });
    }

    /// Cells holding at least one sample.
    pub fn cells(&self) -> usize {
        self.cells.len()
    }

    /// Samples stored (one per aircraft).
    pub fn samples(&self) -> usize {
        self.cell_of.len()
    }

    /// Geometric-minus-baro difference (ft) expected at a place and baro
    /// level at time `now`, from the neighbours other than `exclude`; None
    /// with fewer than `MIN_SAMPLES` of them.
    pub fn estimate(&self, lat: f64, lon: f64, baro_ft: f64, now: f64, exclude: Option<u32>) -> Option<f64> {
        if !(lat.is_finite() && lon.is_finite() && baro_ft.is_finite()) {
            return None;
        }
        let clat = lat.to_radians().cos().max(0.2);
        let span_lat = (RADIUS_KM / 111.0).ceil() as i32;
        let span_lon = (RADIUS_KM / 111.0 / clat).ceil() as i32;
        let (ci, cj) = cell(lat, lon);
        // Weighted normal equations of delta = a + b x, x = baro / 10,000 ft.
        let (mut s00, mut s01, mut s11, mut r0, mut r1, mut n) = (0.0, 0.0, 0.0, 0.0, 0.0, 0usize);
        for i in ci - span_lat..=ci + span_lat {
            for j in cj - span_lon..=cj + span_lon {
                let Some(v) = self.cells.get(&(i, j)) else { continue };
                for s in v {
                    if Some(s.icao) == exclude {
                        continue;
                    }
                    let dy = (s.lat - lat) * 111.2;
                    let dx = (s.lon - lon) * 111.2 * clat;
                    let d = (dx * dx + dy * dy).sqrt();
                    if d > RADIUS_KM {
                        continue;
                    }
                    let age = (now - s.t).max(0.0);
                    let w = (-age / TAU_S).exp() / (1.0 + (d / HALF_KM).powi(2));
                    let x = s.baro_ft / 1e4;
                    s00 += w;
                    s01 += w * x;
                    s11 += w * x * x;
                    r0 += w * s.delta_ft;
                    r1 += w * x * s.delta_ft;
                    n += 1;
                }
            }
        }
        if n < MIN_SAMPLES {
            return None;
        }
        let a11 = s11 + SLOPE_RIDGE;
        let rr1 = r1 + SLOPE_RIDGE * SLOPE_PRIOR;
        let det = s00 * a11 - s01 * s01;
        if det.is_nan() || det.abs() <= 1e-12 {
            return None;
        }
        let a = (r0 * a11 - s01 * rr1) / det;
        let b = (s00 * rr1 - s01 * r0) / det;
        let est = a + b * baro_ft / 1e4;
        est.is_finite().then(|| est.clamp(-1500.0, 4000.0))
    }
}

/// 1-sigma (m) of the difference itself, for a level with no model estimate
/// (a zero correction is then applied): ~50 m plus 3% of the height, the
/// spread of uncorrected differences measured by level (median 76 m below
/// 5,000 ft, ~290 m at FL330+).
pub fn fallback_sigma(height_m: f64) -> f64 {
    50.0 + 0.03 * height_m.max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A field with a known difference: 200 ft + 300 ft per 10,000 ft.
    fn truth(baro_ft: f64) -> f64 {
        200.0 + 300.0 * baro_ft / 1e4
    }

    fn filled(now: f64) -> AltModel {
        let mut m = AltModel::default();
        for k in 0..40u32 {
            let lat = 45.0 + (k % 8) as f64 * 0.15;
            let lon = 9.0 + (k / 8) as f64 * 0.2;
            let baro = 5_000.0 + 900.0 * k as f64;
            m.insert(k, lat, lon, baro, truth(baro), now);
        }
        m
    }

    #[test]
    fn learns_a_regional_field() {
        let m = filled(100.0);
        for baro in [3_000.0, 20_000.0, 37_000.0] {
            let e = m.estimate(45.5, 9.4, baro, 100.0, None).unwrap();
            assert!((e - truth(baro)).abs() < 15.0, "{baro}: {e} vs {}", truth(baro));
        }
        assert_eq!(m.samples(), 40);
        assert!(m.cells() >= 2);
    }

    #[test]
    fn falls_back_far_away_or_without_enough_neighbours() {
        let m = filled(100.0);
        // 400 km away: nothing within the radius.
        assert!(m.estimate(49.0, 9.4, 35_000.0, 100.0, None).is_none());
        let mut sparse = AltModel::default();
        for k in 0..3u32 {
            sparse.insert(k, 45.0, 9.0 + 0.1 * k as f64, 30_000.0, 900.0, 0.0);
        }
        assert!(sparse.estimate(45.0, 9.1, 30_000.0, 0.0, None).is_none());
        assert!(fallback_sigma(11_000.0) > 300.0 && fallback_sigma(0.0) >= 50.0);
    }

    #[test]
    fn decays_and_expires() {
        let mut m = filled(0.0);
        // Fresh samples of a different field outweigh the old ones.
        for k in 100..140u32 {
            let lat = 45.0 + (k % 8) as f64 * 0.15;
            let lon = 9.0 + ((k - 100) / 8) as f64 * 0.2;
            m.insert(k, lat, lon, 30_000.0, 1_500.0, 1_200.0);
        }
        let e = m.estimate(45.5, 9.4, 30_000.0, 1_200.0, None).unwrap();
        // Old samples weigh e^-2 = 0.14: the estimate sits near the new field.
        assert!((e - 1_500.0).abs() < (truth(30_000.0) - 1_500.0).abs() * 0.25, "{e}");
        // Past MAX_AGE_S they are gone.
        m.expire(MAX_AGE_S + 1.0);
        assert_eq!(m.samples(), 40);
        m.expire(1_200.0 + MAX_AGE_S + 1.0);
        assert_eq!(m.samples(), 0);
        assert_eq!(m.cells(), 0);
        assert!(m.estimate(45.5, 9.4, 30_000.0, 4_000.0, None).is_none());
    }

    #[test]
    fn an_aircraft_never_corrects_itself_when_excluded() {
        // Alone in its region, an aircraft's own sample gives no estimate
        // once excluded, whatever it says.
        let mut m = AltModel::default();
        for k in 0..5u32 {
            m.insert(7, 45.0 + 0.01 * k as f64, 9.0, 35_000.0, 2_000.0, k as f64);
        }
        assert_eq!(m.samples(), 1, "one sample per aircraft");
        let mut lonely = AltModel::default();
        for k in 0..4u32 {
            lonely.insert(k, 45.0, 9.0 + 0.05 * k as f64, 35_000.0, 800.0, 0.0);
        }
        lonely.insert(99, 45.0, 9.1, 35_000.0, 3_000.0, 0.0);
        let with = lonely.estimate(45.0, 9.1, 35_000.0, 0.0, None).unwrap();
        let without = lonely.estimate(45.0, 9.1, 35_000.0, 0.0, Some(99)).unwrap();
        assert!((without - 800.0).abs() < 1.0, "{without}");
        assert!(with > without + 100.0, "its own sample would have pulled it: {with}");
    }

    #[test]
    fn memory_is_bounded_per_cell() {
        let mut m = AltModel::default();
        for k in 0..500u32 {
            m.insert(k, 45.5, 9.5, 30_000.0, 900.0, k as f64);
        }
        assert_eq!(m.samples(), MAX_PER_CELL);
        assert_eq!(m.cells(), 1);
        // An aircraft that moves leaves its old cell.
        m.insert(499, 47.5, 9.5, 30_000.0, 900.0, 600.0);
        assert_eq!(m.cells(), 2);
        assert_eq!(m.samples(), MAX_PER_CELL);
    }
}
