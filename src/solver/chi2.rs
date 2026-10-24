//! Chi-squared quantiles for the consistency tests (solver) and the gates
//! (track).
//!
//! The quantile is the inverse of the regularised lower incomplete gamma
//! function P(dof/2, x/2), found by bisection. That costs a few microseconds,
//! too much for every solve, so the two levels the server uses (99% and 99.9%)
//! are tabulated once for 1..=TABLE_DOF degrees of freedom; anything larger is
//! computed on demand (more than 64 dof means more than 60 receivers, which no
//! real group reaches).

use std::sync::OnceLock;

const TABLE_DOF: usize = 64;

/// 99% quantile of chi2(dof); 0 for dof <= 0 (an exactly determined fit has
/// no residual to test).
pub(crate) fn chi2_q99(dof: i32) -> f64 {
    static T: OnceLock<[f64; TABLE_DOF]> = OnceLock::new();
    lookup(&T, 0.99, dof)
}

/// 99.9% quantile of chi2(dof); 0 for dof <= 0.
pub(crate) fn chi2_q999(dof: i32) -> f64 {
    static T: OnceLock<[f64; TABLE_DOF]> = OnceLock::new();
    lookup(&T, 0.999, dof)
}

/// Any quantile, computed on demand (a few microseconds: for the rare
/// outlier path only).
pub(crate) fn chi2_quantile(p: f64, dof: i32) -> f64 {
    if dof <= 0 {
        0.0
    } else {
        quantile(p, dof as f64)
    }
}

fn lookup(table: &OnceLock<[f64; TABLE_DOF]>, p: f64, dof: i32) -> f64 {
    if dof <= 0 {
        return 0.0;
    }
    let d = dof as usize;
    if d <= TABLE_DOF {
        let t = table.get_or_init(|| std::array::from_fn(|i| quantile(p, (i + 1) as f64)));
        t[d - 1]
    } else {
        quantile(p, d as f64)
    }
}

/// x such that P(chi2(dof) <= x) = p, by bisection (the CDF is monotone, so
/// this cannot fail to converge; 200 halvings reach machine precision from any
/// bracket we produce).
fn quantile(p: f64, dof: f64) -> f64 {
    let cdf = |x: f64| gamma_p(0.5 * dof, 0.5 * x);
    let mut lo = 0.0;
    let mut hi = dof.max(1.0);
    while cdf(hi) < p {
        lo = hi;
        hi *= 2.0;
    }
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if cdf(mid) < p {
            lo = mid;
        } else {
            hi = mid;
        }
        if hi - lo <= 1e-12 * hi {
            break;
        }
    }
    0.5 * (lo + hi)
}

/// Regularised lower incomplete gamma P(a, x): power series below a + 1,
/// Lentz continued fraction for the complement above (Numerical Recipes 6.2).
fn gamma_p(a: f64, x: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    let ln_pre = -x + a * x.ln() - ln_gamma(a);
    if x < a + 1.0 {
        let mut term = 1.0 / a;
        let mut sum = term;
        let mut ap = a;
        for _ in 0..1000 {
            ap += 1.0;
            term *= x / ap;
            sum += term;
            if term.abs() < sum.abs() * 1e-16 {
                break;
            }
        }
        sum * ln_pre.exp()
    } else {
        const TINY: f64 = 1e-300;
        let mut b = x + 1.0 - a;
        let mut c = 1.0 / TINY;
        let mut d = 1.0 / b;
        let mut h = d;
        for i in 1..1000 {
            let an = -(i as f64) * (i as f64 - a);
            b += 2.0;
            d = an * d + b;
            if d.abs() < TINY {
                d = TINY;
            }
            c = b + an / c;
            if c.abs() < TINY {
                c = TINY;
            }
            d = 1.0 / d;
            let del = d * c;
            h *= del;
            if (del - 1.0).abs() < 1e-16 {
                break;
            }
        }
        1.0 - h * ln_pre.exp()
    }
}

/// ln Gamma(a) for a > 0, Lanczos (g = 7, 9 terms), ~1e-15 relative.
fn ln_gamma(a: f64) -> f64 {
    const G: f64 = 7.0;
    const C: [f64; 9] = [
        0.999_999_999_999_809_9,
        676.520_368_121_885_1,
        -1_259.139_216_722_402_8,
        771.323_428_777_653_1,
        -176.615_029_162_140_6,
        12.507_343_278_686_905,
        -0.138_571_095_265_720_12,
        9.984_369_578_019_572e-6,
        1.505_632_735_149_311_6e-7,
    ];
    if a < 0.5 {
        // Reflection keeps the series in its accurate range.
        let pi = std::f64::consts::PI;
        return (pi / (pi * a).sin()).ln() - ln_gamma(1.0 - a);
    }
    let a = a - 1.0;
    let mut s = C[0];
    for (i, &c) in C.iter().enumerate().skip(1) {
        s += c / (a + i as f64);
    }
    let t = a + G + 0.5;
    0.5 * (2.0 * std::f64::consts::PI).ln() + (a + 0.5) * t.ln() - t + s.ln()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_quantiles() {
        // Reference values from standard chi-square tables (scipy.stats.chi2.ppf).
        let q99 = [(1, 6.634_897), (2, 9.210_340), (3, 11.344_867), (4, 13.276_704), (5, 15.086_272), (10, 23.209_251), (30, 50.892_181)];
        let q999 = [(1, 10.827_566), (2, 13.815_511), (3, 16.266_236), (4, 18.466_827), (6, 22.457_744), (20, 45.314_746)];
        for (d, v) in q99 {
            assert!((chi2_q99(d) - v).abs() < 1e-5, "q99 dof {d}: {} vs {v}", chi2_q99(d));
        }
        for (d, v) in q999 {
            assert!((chi2_q999(d) - v).abs() < 1e-5, "q999 dof {d}: {} vs {v}", chi2_q999(d));
        }
        assert!((chi2_quantile(0.95, 3) - 7.814_728).abs() < 1e-5);
        // z = 3.144 for a two-sided 0.01 / 6: z^2 = 9.8846.
        assert!((chi2_quantile(1.0 - 0.01 / 6.0, 1) - 9.884_6).abs() < 1e-3);
        assert_eq!(chi2_q99(0), 0.0);
        assert_eq!(chi2_q999(-3), 0.0);
        // Beyond the table: Wilson-Hilferty is good to ~0.1% here.
        let wh = |k: f64, z: f64| k * (1.0 - 2.0 / (9.0 * k) + z * (2.0 / (9.0 * k)).sqrt()).powi(3);
        assert!((chi2_q99(100) / wh(100.0, 2.326_348) - 1.0).abs() < 2e-3);
    }
}
