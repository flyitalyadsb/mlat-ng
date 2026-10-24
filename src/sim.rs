//! End-to-end simulation of the whole pipeline: synthetic receivers with
//! drifting, offset clocks; ADS-B aircraft whose real DF17 frames drive clock
//! sync; Mode-S-only aircraft whose DF4 replies are multilaterated. The engine
//! is driven with a simulated clock and its SBS output is scored against the
//! simulated truth. This is the test that says the parts work together.

#![cfg(test)]

use std::f64::consts::PI;
use std::sync::Arc;

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use tokio::sync::{broadcast, mpsc};

use crate::engine::{Engine, EngineConfig, Stats};
use crate::geo::{dist, ecef_to_llh, haversine_m, llh_to_ecef, Ecef, Llh, C_AIR, FT_TO_M};
use crate::modes;
use crate::types::{ClockType, Event, ReceiverInfo};

const CPR_SCALE: f64 = 131_072.0;

fn nl(lat: f64) -> f64 {
    if lat.abs() >= 87.0 {
        return 1.0;
    }
    let a = 1.0 - (PI / 30.0).cos();
    let b = (PI / 180.0 * lat).cos().powi(2);
    (2.0 * PI / (1.0 - a / b).acos()).floor()
}

fn cpr_encode(lat: f64, lon: f64, odd: bool) -> (u32, u32) {
    let i = if odd { 1.0 } else { 0.0 };
    let dlat = 360.0 / (60.0 - i);
    let yz = (CPR_SCALE * lat.rem_euclid(dlat) / dlat + 0.5).floor();
    let rlat = dlat * (yz / CPR_SCALE + (lat / dlat).floor());
    let dlon = 360.0 / (nl(rlat) - i).max(1.0);
    let xz = (CPR_SCALE * lon.rem_euclid(dlon) / dlon + 0.5).floor();
    ((yz as u32) & 0x1ffff, (xz as u32) & 0x1ffff)
}

/// Q=1 altitude code, 25 ft steps.
fn alt_n(alt_ft: i32) -> u32 {
    ((alt_ft + 1000) / 25) as u32
}

fn set_parity(frame: &mut [u8], xor: u32) {
    let n = frame.len();
    frame[n - 3] = 0;
    frame[n - 2] = 0;
    frame[n - 1] = 0;
    let crc = modes::crc_residual(frame) ^ xor;
    frame[n - 3] = (crc >> 16) as u8;
    frame[n - 2] = (crc >> 8) as u8;
    frame[n - 1] = crc as u8;
}

fn df17_position(icao: u32, lat: f64, lon: f64, alt_ft: i32, odd: bool) -> Vec<u8> {
    let (yz, xz) = cpr_encode(lat, lon, odd);
    let n = alt_n(alt_ft);
    let ac12 = ((n & 0x7f0) << 1) | 0x010 | (n & 0x00f);
    // ME: TC(5)=11 SS(2)=0 SAF(1)=0 ALT(12) T(1)=0 F(1) LAT(17) LON(17)
    let me: u64 = (11u64 << 51) | ((ac12 as u64) << 36) | ((odd as u64) << 34) | ((yz as u64) << 17) | xz as u64;
    let mut f = vec![0u8; 14];
    f[0] = (17 << 3) | 5;
    f[1] = (icao >> 16) as u8;
    f[2] = (icao >> 8) as u8;
    f[3] = icao as u8;
    for i in 0..7 {
        f[4 + i] = (me >> (48 - 8 * i)) as u8;
    }
    set_parity(&mut f, 0);
    f
}

fn df4_altitude(icao: u32, alt_ft: i32) -> Vec<u8> {
    let n = alt_n(alt_ft);
    let ac13 = ((n & 0x7e0) << 2) | ((n & 0x010) << 1) | 0x0010 | (n & 0x00f);
    let mut f = vec![0u8; 7];
    f[0] = 4 << 3; // DF4, FS=0
    f[1] = 0; // DR, UM
    f[2] = (ac13 >> 8) as u8 & 0x1f;
    f[3] = ac13 as u8;
    set_parity(&mut f, icao);
    f
}

struct SimRx {
    pos: Ecef,
    offset: f64,
    drift: f64,
}

impl SimRx {
    /// Clock counter reading (12 MHz) for a true arrival time, with jitter.
    fn ticks(&self, tau: f64, rng: &mut StdRng, jitter: f64) -> f64 {
        let noise: f64 = (0..12).map(|_| rng.gen::<f64>() - 0.5).sum::<f64>() * jitter;
        ((tau + self.offset) * (1.0 + self.drift) + noise) * 12e6
    }
}

struct SimAc {
    icao: u32,
    start: Llh,
    /// m/s east, north
    vel: (f64, f64),
    alt_ft: i32,
    adsb: bool,
}

impl SimAc {
    fn llh(&self, t: f64) -> Llh {
        let dlat = self.vel.1 * t / 111_320.0;
        let dlon = self.vel.0 * t / (111_320.0 * self.start.lat.to_radians().cos());
        Llh { lat: self.start.lat + dlat, lon: self.start.lon + dlon, alt: self.alt_ft as f64 * FT_TO_M }
    }
}

#[derive(Debug)]
struct Scored {
    errs: Vec<f64>,
}

impl Scored {
    fn quantile(&self, q: f64) -> f64 {
        let mut e = self.errs.clone();
        e.sort_by(|a, b| a.partial_cmp(b).unwrap());
        e[((e.len() - 1) as f64 * q) as usize]
    }
}

fn run_sim(seed: u64, n_rx: usize, jitter: f64, duration: f64) -> (Scored, Arc<Stats>) {
    let mut rng = StdRng::seed_from_u64(seed);
    let epoch = 1_800_000_000.0;
    // Receivers in a ~150 km disc around Milan.
    let rxs: Vec<SimRx> = (0..n_rx)
        .map(|i| {
            let a = 2.0 * PI * i as f64 / n_rx as f64 + rng.gen::<f64>() * 0.3;
            let r = 40e3 + rng.gen::<f64>() * 110e3;
            let lat = 45.46 + r * a.sin() / 111_320.0;
            let lon = 9.19 + r * a.cos() / (111_320.0 * 45.46f64.to_radians().cos());
            SimRx {
                pos: llh_to_ecef(Llh { lat, lon, alt: 100.0 + rng.gen::<f64>() * 300.0 }),
                offset: rng.gen::<f64>() * 5000.0,
                drift: (rng.gen::<f64>() - 0.5) * 80e-6,
            }
        })
        .collect();
    let mut acs = Vec::new();
    for i in 0..8 {
        let a = rng.gen::<f64>() * 2.0 * PI;
        acs.push(SimAc {
            icao: 0x4C0000 + i,
            start: Llh { lat: 45.2 + rng.gen::<f64>() * 0.6, lon: 8.8 + rng.gen::<f64>() * 0.8, alt: 0.0 },
            vel: (230.0 * a.cos(), 230.0 * a.sin()),
            alt_ft: 30_000 + 1000 * i as i32,
            adsb: true,
        });
    }
    for i in 0..3 {
        let a = rng.gen::<f64>() * 2.0 * PI;
        acs.push(SimAc {
            icao: 0x3C1000 + i,
            start: Llh { lat: 45.3 + rng.gen::<f64>() * 0.4, lon: 8.9 + rng.gen::<f64>() * 0.6, alt: 0.0 },
            vel: (200.0 * a.cos(), 200.0 * a.sin()),
            alt_ft: 24_000 + 2000 * i as i32,
            adsb: false,
        });
    }

    // Events, each with its server arrival time.
    let mut events: Vec<(f64, Event)> = Vec::new();
    let (tx, _rx) = mpsc::channel(1);
    for (i, r) in rxs.iter().enumerate() {
        let l = ecef_to_llh(r.pos);
        events.push((
            0.0,
            Event::Connect {
                conn: i as u64,
                info: ReceiverInfo {
                    user: format!("rx{i}"),
                    uuid: None,
                    lat: l.lat,
                    lon: l.lon,
                    alt: l.alt,
                    clock: ClockType::Dump1090,
                    privacy: false,
                    source_ip: "127.0.0.1".into(),
                    client_version: "sim".into(),
                    compress: "none",
                },
                tx: tx.clone(),
            },
        ));
        events.push((0.01, Event::Seen { conn: i as u64, icao: acs.iter().map(|a| a.icao).collect() }));
    }
    let net = |rng: &mut StdRng| 0.02 + rng.gen::<f64>() * 0.08;
    let mut t = 1.0;
    let mut odd = false;
    let mut last_even: Vec<Option<(f64, Vec<u8>)>> = vec![None; acs.len()];
    while t < duration {
        for (ai, ac) in acs.iter().enumerate() {
            let p = ac.llh(t);
            let pe = llh_to_ecef(p);
            if ac.adsb {
                let frame = df17_position(ac.icao, p.lat, p.lon, ac.alt_ft, odd);
                if !odd {
                    last_even[ai] = Some((t, frame));
                } else if let Some((te, fe)) = last_even[ai].clone() {
                    let pe_e = llh_to_ecef(ac.llh(te));
                    for (ri, r) in rxs.iter().enumerate() {
                        let ae = te + dist(&pe_e, &r.pos) / C_AIR;
                        let ao = t + dist(&pe, &r.pos) / C_AIR;
                        events.push((
                            ao + net(&mut rng),
                            Event::Sync {
                                conn: ri as u64,
                                et: r.ticks(ae, &mut rng, jitter),
                                ot: r.ticks(ao, &mut rng, jitter),
                                em: fe.clone(),
                                om: frame.clone(),
                            },
                        ));
                    }
                }
            } else {
                let frame = df4_altitude(ac.icao, ac.alt_ft);
                for (ri, r) in rxs.iter().enumerate() {
                    if rng.gen::<f64>() < 0.15 {
                        continue; // not every receiver hears every reply
                    }
                    let a = t + dist(&pe, &r.pos) / C_AIR;
                    events.push((
                        a + net(&mut rng),
                        Event::Mlat { conn: ri as u64, t: r.ticks(a, &mut rng, jitter), msg: frame.clone(), rx: epoch + a },
                    ));
                }
            }
        }
        odd = !odd;
        t += 0.25;
    }
    events.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let stats = Arc::new(Stats::default());
    let (out, mut sbs) = broadcast::channel(1 << 16);
    let mut eng = Engine::new(EngineConfig { shadow: true, mlat_delay: 0.9, holdout_every: 20 }, stats.clone(), out);
    let mut next_tick = 0.0;
    let mut next_house = 10.0;
    for (at, ev) in events {
        // the Mlat events carry their own wall time; keep `rx` consistent
        let ev = match ev {
            Event::Mlat { conn, t, msg, .. } => Event::Mlat { conn, t, msg, rx: epoch + at },
            e => e,
        };
        while next_tick <= at {
            eng.tick_at(epoch + next_tick, next_tick >= next_house);
            if next_tick >= next_house {
                next_house += 10.0;
            }
            next_tick += 0.05;
        }
        eng.handle_at(ev, epoch + at);
    }
    eng.tick_at(epoch + duration + 5.0, true);

    // Score the Mode-S-only aircraft after warm-up.
    let mut errs = Vec::new();
    while let Ok(line) = sbs.try_recv() {
        let f: Vec<&str> = line.trim_end().split(',').collect();
        let icao = u32::from_str_radix(f[4], 16).unwrap_or(0);
        let Some(ac) = acs.iter().find(|a| a.icao == icao && !a.adsb) else { continue };
        let (lat, lon): (f64, f64) = (f[14].parse().unwrap(), f[15].parse().unwrap());
        // fix time from the SBS fields (date/time of the fix)
        let hms: Vec<f64> = f[7].split(':').map(|x| x.parse().unwrap()).collect();
        let sod = hms[0] * 3600.0 + hms[1] * 60.0 + hms[2];
        let t_fix = sod - (epoch % 86_400.0);
        if t_fix < 60.0 {
            continue;
        }
        let truth = ac.llh(t_fix);
        errs.push(haversine_m(lat, lon, truth.lat, truth.lon));
    }
    (Scored { errs }, stats)
}

#[test]
fn end_to_end_multilateration_is_accurate() {
    let (s, stats) = run_sim(7, 8, 0.3e-6, 180.0);
    let snap = |a: &std::sync::atomic::AtomicU64| a.load(std::sync::atomic::Ordering::Relaxed);
    eprintln!(
        "sync={} points={} pair_updates={} rejects={} resets={} groups={} clusters={} solves={} fixes={} tdoa={} outputs={} holdout={}/{} n_scored={}",
        snap(&stats.sync_msgs),
        snap(&stats.sync_points),
        snap(&stats.pair_updates),
        snap(&stats.pair_rejects),
        snap(&stats.pair_resets),
        snap(&stats.groups),
        snap(&stats.clusters),
        snap(&stats.solves),
        snap(&stats.fixes),
        snap(&stats.track_tdoa_updates),
        snap(&stats.outputs),
        snap(&stats.holdout_solved),
        snap(&stats.holdout_failed),
        s.errs.len()
    );
    assert!(s.errs.len() >= 100, "too few scored outputs: {}", s.errs.len());
    let (p50, p90) = (s.quantile(0.5), s.quantile(0.9));
    eprintln!("Mode-S-only error p50 {p50:.0} m p90 {p90:.0} m");
    assert!(p50 < 250.0, "p50 {p50}");
    assert!(p90 < 800.0, "p90 {p90}");
    assert!(snap(&stats.holdout_solved) > 20);
}
