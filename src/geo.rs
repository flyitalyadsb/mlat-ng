//! WGS84 geodesy shared by every module: ECEF <-> geodetic, distances, and a
//! local East-North-Up frame for tracking.

pub const WGS84_A: f64 = 6_378_137.0;
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F);
pub const WGS84_E2: f64 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);
const EP2: f64 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B);

/// Propagation speed used for every time <-> range conversion. Same value as
/// mlat-server (vacuum speed over a 1.00032 refractive index), so the shadow
/// comparison is not skewed by a different constant.
pub const C_AIR: f64 = 299_792_458.0 / 1.00032;

pub const FT_TO_M: f64 = 0.3048;
pub const M_TO_FT: f64 = 1.0 / FT_TO_M;
pub const MS_TO_KT: f64 = 1.943_844;

pub type Ecef = [f64; 3];

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Llh {
    /// degrees
    pub lat: f64,
    /// degrees
    pub lon: f64,
    /// metres above the ellipsoid
    pub alt: f64,
}

pub fn llh_to_ecef(p: Llh) -> Ecef {
    let lat = p.lat.to_radians();
    let lon = p.lon.to_radians();
    let (slat, clat) = lat.sin_cos();
    let (slon, clon) = lon.sin_cos();
    let n = WGS84_A / (1.0 - WGS84_E2 * slat * slat).sqrt();
    [
        (n + p.alt) * clat * clon,
        (n + p.alt) * clat * slon,
        (n * (1.0 - WGS84_E2) + p.alt) * slat,
    ]
}

/// Bowring's method; sub-millimetre at aircraft altitudes.
pub fn ecef_to_llh(e: Ecef) -> Llh {
    let [x, y, z] = e;
    let lon = y.atan2(x);
    let p = (x * x + y * y).sqrt();
    let th = (WGS84_A * z).atan2(WGS84_B * p);
    let (sth, cth) = th.sin_cos();
    let lat = (z + EP2 * WGS84_B * sth * sth * sth).atan2(p - WGS84_E2 * WGS84_A * cth * cth * cth);
    let slat = lat.sin();
    let n = WGS84_A / (1.0 - WGS84_E2 * slat * slat).sqrt();
    let alt = if lat.cos().abs() > 1e-9 { p / lat.cos() - n } else { z.abs() - WGS84_B };
    Llh { lat: lat.to_degrees(), lon: lon.to_degrees(), alt }
}

#[inline]
pub fn dist(a: &Ecef, b: &Ecef) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Great-circle distance in metres on a spherical Earth (good to ~0.5%).
pub fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dp = p2 - p1;
    let dl = (lon2 - lon1).to_radians();
    let h = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    2.0 * 6_371_008.8 * h.sqrt().min(1.0).asin()
}

/// A local tangent plane anchored at `origin`: ENU axes as rows of `r`.
#[derive(Debug, Clone, Copy)]
pub struct Enu {
    pub origin: Ecef,
    pub r: [[f64; 3]; 3],
}

impl Enu {
    pub fn at(origin: Ecef) -> Self {
        let o = ecef_to_llh(origin);
        let (slat, clat) = o.lat.to_radians().sin_cos();
        let (slon, clon) = o.lon.to_radians().sin_cos();
        Enu {
            origin,
            r: [
                [-slon, clon, 0.0],
                [-slat * clon, -slat * slon, clat],
                [clat * clon, clat * slon, slat],
            ],
        }
    }

    pub fn to_enu(&self, e: &Ecef) -> [f64; 3] {
        let d = [e[0] - self.origin[0], e[1] - self.origin[1], e[2] - self.origin[2]];
        let r = &self.r;
        [
            r[0][0] * d[0] + r[0][1] * d[1] + r[0][2] * d[2],
            r[1][0] * d[0] + r[1][1] * d[1] + r[1][2] * d[2],
            r[2][0] * d[0] + r[2][1] * d[1] + r[2][2] * d[2],
        ]
    }

    pub fn to_ecef(&self, l: &[f64; 3]) -> Ecef {
        let r = &self.r;
        [
            self.origin[0] + r[0][0] * l[0] + r[1][0] * l[1] + r[2][0] * l[2],
            self.origin[1] + r[0][1] * l[0] + r[1][1] * l[1] + r[2][1] * l[2],
            self.origin[2] + r[0][2] * l[0] + r[1][2] * l[1] + r[2][2] * l[2],
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_llh_ecef() {
        for &(lat, lon, alt) in &[(45.46, 9.19, 120.0), (-33.9, 151.2, 11000.0), (64.1, -21.9, 0.0), (0.0, 0.0, 0.0)] {
            let p = Llh { lat, lon, alt };
            let q = ecef_to_llh(llh_to_ecef(p));
            assert!((p.lat - q.lat).abs() < 1e-9, "{p:?} {q:?}");
            assert!((p.lon - q.lon).abs() < 1e-9);
            assert!((p.alt - q.alt).abs() < 1e-3);
        }
    }

    #[test]
    fn enu_roundtrip() {
        let o = llh_to_ecef(Llh { lat: 45.0, lon: 9.0, alt: 0.0 });
        let f = Enu::at(o);
        let p = llh_to_ecef(Llh { lat: 45.1, lon: 9.2, alt: 10000.0 });
        let back = f.to_ecef(&f.to_enu(&p));
        assert!(dist(&p, &back) < 1e-6);
        let up = f.to_enu(&llh_to_ecef(Llh { lat: 45.0, lon: 9.0, alt: 1000.0 }));
        assert!((up[2] - 1000.0).abs() < 1e-6 && up[0].abs() < 1e-6 && up[1].abs() < 1e-6);
    }
}
