//! Synthetic-data helpers shared by the solver and track tests: seeded
//! Gaussian noise, realistic correlated time covariances, arrival times.

use nalgebra::{DMatrix, DVector};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::geo::{self, Ecef, Llh, C_AIR};
use crate::solver::Meas;

/// Seeded standard-normal source (Box-Muller; `rand_distr` is not a
/// dependency).
pub(crate) struct Gauss {
    rng: StdRng,
    spare: Option<f64>,
}

impl Gauss {
    pub(crate) fn new(seed: u64) -> Gauss {
        Gauss { rng: StdRng::seed_from_u64(seed), spare: None }
    }

    pub(crate) fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.rng.gen::<f64>()
    }

    pub(crate) fn normal(&mut self) -> f64 {
        if let Some(s) = self.spare.take() {
            return s;
        }
        let u1 = 1.0 - self.rng.gen::<f64>(); // (0, 1]: ln is finite
        let u2 = self.rng.gen::<f64>();
        let r = (-2.0 * u1.ln()).sqrt();
        let (s, c) = (2.0 * std::f64::consts::PI * u2).sin_cos();
        self.spare = Some(r * s);
        r * c
    }

    /// One draw from N(0, cov).
    pub(crate) fn mvn(&mut self, cov: &DMatrix<f64>) -> DVector<f64> {
        let l = cov.clone().cholesky().expect("test covariance must be PD").unpack();
        let z = DVector::from_fn(cov.nrows(), |_, _| self.normal());
        l * z
    }
}

pub(crate) fn llh(lat: f64, lon: f64, alt: f64) -> Ecef {
    geo::llh_to_ecef(Llh { lat, lon, alt })
}

/// A time covariance shaped like the engine's: per-receiver 1-sigma in
/// [smin, smax] seconds and correlations from two shared factors (a common
/// reference clock plus a regional one), giving pairwise correlations of
/// roughly 0..0.8 with a few slightly negative.
pub(crate) fn random_cov(g: &mut Gauss, k: usize, smin: f64, smax: f64) -> DMatrix<f64> {
    let s: Vec<f64> = (0..k).map(|_| g.uniform(smin, smax)).collect();
    let a: Vec<f64> = (0..k).map(|_| g.uniform(0.0, 0.85)).collect();
    let b: Vec<f64> = (0..k)
        .map(|i| {
            let room = (0.9 - a[i] * a[i]).max(0.0).sqrt();
            g.uniform(-0.4f64.min(room), 0.4f64.min(room))
        })
        .collect();
    DMatrix::from_fn(k, k, |i, j| {
        let rho = if i == j { 1.0 } else { a[i] * a[j] + b[i] * b[j] };
        rho * s[i] * s[j]
    })
}

/// Exact arrival times for an emission at `truth`, time `t0`, plus `noise`
/// (seconds per receiver).
pub(crate) fn arrivals(truth: &Ecef, t0: f64, rx: &[Ecef], noise: &DVector<f64>) -> Vec<Meas> {
    rx.iter().enumerate().map(|(i, r)| Meas { rx: *r, t: t0 + geo::dist(truth, r) / C_AIR + noise[i] }).collect()
}

/// Receivers of a realistic northern-Italy network (lat, lon, antenna height).
pub(crate) const PO_VALLEY: [(f64, f64, f64); 12] = [
    (45.464, 9.190, 150.0),  // Milan
    (45.070, 7.687, 260.0),  // Turin
    (44.494, 11.343, 80.0),  // Bologna
    (44.405, 8.946, 40.0),   // Genoa
    (45.438, 10.992, 90.0),  // Verona
    (44.801, 10.328, 75.0),  // Parma
    (45.695, 9.670, 300.0),  // Bergamo
    (45.186, 9.155, 90.0),   // Pavia
    (44.648, 10.925, 60.0),  // Modena
    (45.541, 10.212, 180.0), // Brescia
    (44.912, 8.611, 120.0),  // Alessandria
    (45.406, 11.876, 30.0),  // Padua
];

pub(crate) fn po_valley(k: usize) -> Vec<Ecef> {
    PO_VALLEY[..k].iter().map(|&(la, lo, h)| llh(la, lo, h)).collect()
}
