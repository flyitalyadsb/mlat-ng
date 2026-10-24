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

/// DF5 identity reply with the squawk's ID13 bits (C1 A1 C2 A2 C4 A4 X B1 D1 B2 D2 B4 D4).
fn df5_identity(icao: u32, squawk: u16) -> Vec<u8> {
    let d = |i: u32| ((squawk as u32) >> (12 - 4 * i)) & 0xf; // decimal-looking octal digits A,B,C,D
    let (a, b, c, dd) = (d(0), d(1), d(2), d(3));
    let bit = |v: u32, k: u32| (v >> k) & 1;
    let id13 = (bit(c, 0) << 12) | (bit(a, 0) << 11) | (bit(c, 1) << 10) | (bit(a, 1) << 9) | (bit(c, 2) << 8)
        | (bit(a, 2) << 7) | (bit(b, 0) << 5) | (bit(dd, 0) << 4) | (bit(b, 1) << 3) | (bit(dd, 1) << 2)
        | (bit(b, 2) << 1) | bit(dd, 2);
    let mut f = vec![0u8; 7];
    f[0] = 5 << 3;
    f[2] = (id13 >> 8) as u8 & 0x1f;
    f[3] = id13 as u8;
    set_parity(&mut f, icao);
    f
}

/// mlat-client's false_ac frame for a Mode A/C payload written as 0xABCD.
fn false_ac_frame(code: u16) -> Vec<u8> {
    let digits = format!("{code:04x}");
    let v: u32 = format!("{digits}00").parse::<u32>().unwrap() % 1_000_000;
    vec![0x5d, (v >> 16) as u8, (v >> 8) as u8, v as u8, 0xa4, 0x92, 0x4f]
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
    /// Mode S identity (DF5) and Mode A replies with this code (0xABCD).
    squawk: Option<u16>,
    /// Mode A/C only: no Mode S frames at all.
    modeac_only: bool,
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

fn run_sim(seed: u64, n_rx: usize, jitter: f64, duration: f64, modeac: bool) -> (Scored, Arc<Stats>, Vec<String>) {
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
    let mut acs: Vec<SimAc> = Vec::new();
    for i in 0..8 {
        let a = rng.gen::<f64>() * 2.0 * PI;
        acs.push(SimAc {
            icao: 0x4C0000 + i,
            start: Llh { lat: 45.2 + rng.gen::<f64>() * 0.6, lon: 8.8 + rng.gen::<f64>() * 0.8, alt: 0.0 },
            vel: (230.0 * a.cos(), 230.0 * a.sin()),
            alt_ft: 30_000 + 1000 * i as i32,
            adsb: true,
            squawk: None,
            modeac_only: false,
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
            squawk: if modeac && i == 0 { Some(0x4721) } else { None },
            modeac_only: false,
        });
    }
    if modeac {
        acs.push(SimAc {
            icao: 0,
            start: Llh { lat: 45.5, lon: 9.3, alt: 0.0 },
            vel: (-150.0, 60.0),
            alt_ft: 18_000,
            adsb: false,
            squawk: Some(0x3456),
            modeac_only: true,
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
                let mut frames = Vec::new();
                if !ac.modeac_only {
                    frames.push(df4_altitude(ac.icao, ac.alt_ft));
                }
                if let Some(sq) = ac.squawk {
                    if !ac.modeac_only && odd {
                        frames.push(df5_identity(ac.icao, sq));
                    }
                    frames.push(false_ac_frame(sq));
                }
                for frame in frames {
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
    let mut lines = Vec::new();
    while let Ok(line) = sbs.try_recv() {
        lines.push(line.to_string());
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
    (Scored { errs }, stats, lines)
}

#[test]
fn end_to_end_multilateration_is_accurate() {
    let (s, stats, _) = run_sim(7, 8, 0.3e-6, 180.0, false);
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

#[test]
fn mode_ac_is_associated_and_tracked() {
    let (s, stats, lines) = run_sim(11, 10, 0.3e-6, 180.0, true);
    let get = |a: &std::sync::atomic::AtomicU64| a.load(std::sync::atomic::Ordering::Relaxed);
    eprintln!(
        "modeac groups={} blacklisted={} fixes={} assoc={} outputs={} mode-s scored={}",
        get(&stats.modeac_groups),
        get(&stats.modeac_blacklisted),
        get(&stats.modeac_fixes),
        get(&stats.modeac_assoc),
        get(&stats.modeac_outputs),
        s.errs.len()
    );
    // The Mode S aircraft answering Mode A with its own squawk is recognised
    // as that aircraft, not turned into a second, fake target.
    assert!(get(&stats.modeac_assoc) > 50, "assoc {}", get(&stats.modeac_assoc));
    // The Mode A/C-only aircraft becomes a synthetic track near its truth.
    let ac_lines: Vec<&String> = lines.iter().filter(|l| l.split(',').nth(4).map(|a| a.starts_with('~')).unwrap_or(false)).collect();
    assert!(ac_lines.len() > 20, "Mode A/C outputs {}", ac_lines.len());
    let start = Llh { lat: 45.5, lon: 9.3, alt: 0.0 };
    let epoch = 1_800_000_000.0f64;
    let mut errs: Vec<f64> = ac_lines
        .iter()
        .map(|l| {
            let f: Vec<&str> = l.trim_end().split(',').collect();
            let hms: Vec<f64> = f[7].split(':').map(|x| x.parse().unwrap()).collect();
            let t = hms[0] * 3600.0 + hms[1] * 60.0 + hms[2] - (epoch % 86_400.0);
            let lat = start.lat + 60.0 * t / 111_320.0;
            let lon = start.lon - 150.0 * t / (111_320.0 * start.lat.to_radians().cos());
            haversine_m(f[14].parse().unwrap(), f[15].parse().unwrap(), lat, lon)
        })
        .collect();
    errs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p50 = errs[errs.len() / 2];
    eprintln!("Mode A/C-only track error p50 {p50:.0} m over {} outputs", errs.len());
    assert!(p50 < 1500.0, "p50 {p50}");
    // Mode A/C must never leak into the Mode S stream as a fake ICAO.
    assert!(!lines.iter().any(|l| l.split(',').nth(4).map(|a| a == "000000" || a.ends_with("00") && a.len() == 6 && modes::is_false_ac_address(u32::from_str_radix(a, 16).unwrap_or(1))).unwrap_or(false)));
}

#[test]
fn sparse_noisy_network_still_tracks() {
    // 5 receivers, RTL-SDR-class jitter: the case where TDOA updates and the
    // joint normalisation have to carry the network.
    let (s, stats, _) = run_sim(23, 5, 0.6e-6, 240.0, false);
    let get = |a: &std::sync::atomic::AtomicU64| a.load(std::sync::atomic::Ordering::Relaxed);
    eprintln!(
        "sparse: fixes={} tdoa={} rejects={} outputs={} scored={}",
        get(&stats.fixes),
        get(&stats.track_tdoa_updates),
        get(&stats.track_rejects),
        get(&stats.outputs),
        s.errs.len()
    );
    assert!(s.errs.len() >= 100, "scored {}", s.errs.len());
    let (p50, p90) = (s.quantile(0.5), s.quantile(0.9));
    eprintln!("sparse Mode-S-only error p50 {p50:.0} m p90 {p90:.0} m");
    assert!(p50 < 400.0 && p90 < 1500.0, "p50 {p50} p90 {p90}");
}

/// DF17 airborne position frame, for other modules' tests.
pub(crate) fn test_df17_position(icao: u32, lat: f64, lon: f64, alt_ft: i32, odd: bool) -> Vec<u8> {
    df17_position(icao, lat, lon, alt_ft, odd)
}

/// A clean DF11 acquisition squitter.
fn df11_squitter(icao: u32) -> Vec<u8> {
    let mut f = vec![0x5D, (icao >> 16) as u8, (icao >> 8) as u8, icao as u8, 0, 0, 0];
    set_parity(&mut f, 0);
    f
}

/// The feed path end to end: receivers whose readsb forwards only part of
/// what it hears (as `beast_reduce` does) send receiver-tagged frames; the
/// adapter turns them into engine events with no mlat-client involved.
/// Returns (Mode-S-only errors, bench errors, stats).
fn run_feed_sim(seed: u64, n_rx: usize, forward_p: f64, duration: f64) -> (Scored, Scored, Arc<Stats>) {
    use crate::relay::adapter::{Adapter, AdapterConfig, AdapterStats};
    use crate::relay::beast::Frame;
    use crate::relay::registry::Receiver;
    use std::collections::HashMap;

    let mut rng = StdRng::seed_from_u64(seed);
    let epoch = 1_800_000_000.0;
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
    let mut acs: Vec<SimAc> = Vec::new();
    for i in 0..8 {
        let a = rng.gen::<f64>() * 2.0 * PI;
        acs.push(SimAc {
            icao: 0x4C0000 + i,
            start: Llh { lat: 45.2 + rng.gen::<f64>() * 0.6, lon: 8.8 + rng.gen::<f64>() * 0.8, alt: 0.0 },
            vel: (230.0 * a.cos(), 230.0 * a.sin()),
            alt_ft: 30_000 + 1000 * i as i32,
            adsb: true,
            squawk: None,
            modeac_only: false,
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
            squawk: None,
            modeac_only: false,
        });
    }
    let registry: HashMap<u64, Receiver> = rxs
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let l = ecef_to_llh(r.pos);
            (0x1000 + i as u64, Receiver { user: format!("rx{i}"), lat: l.lat, lon: l.lon, alt: l.alt, clock: ClockType::Dump1090, privacy: false })
        })
        .collect();

    // (arrival wall, frame)
    let mut frames: Vec<(f64, Frame)> = Vec::new();
    let mut push = |rng: &mut StdRng, frames: &mut Vec<(f64, Frame)>, msg: &[u8], t: f64, pe: &Ecef, p: f64| {
        for (ri, r) in rxs.iter().enumerate() {
            if rng.gen::<f64>() >= p {
                continue;
            }
            let a = t + dist(pe, &r.pos) / C_AIR;
            let mut m = [0u8; 14];
            m[..msg.len()].copy_from_slice(msg);
            let f = Frame {
                rid: 0x1000 + ri as u64,
                ts: r.ticks(a, rng, 0.15e-6).round() as u64,
                sig: 0x40,
                kind: if msg.len() == 7 { b'2' } else { b'3' },
                len: msg.len() as u8,
                msg: m,
            };
            // readsb flush + WAN
            frames.push((a + 0.05 + rng.gen::<f64>() * 0.25, f));
        }
    };
    let mut t = 0.5;
    let mut odd = false;
    while t < duration {
        for ac in &acs {
            let p = ac.llh(t);
            let pe = llh_to_ecef(p);
            if ac.adsb {
                push(&mut rng, &mut frames, &df17_position(ac.icao, p.lat, p.lon, ac.alt_ft, odd), t, &pe, 0.9);
                if odd {
                    // a roll-call reply once a second: what the bench multilaterates
                    push(&mut rng, &mut frames, &df4_altitude(ac.icao, ac.alt_ft), t + 0.01, &pe, forward_p);
                }
            } else {
                push(&mut rng, &mut frames, &df4_altitude(ac.icao, ac.alt_ft), t, &pe, forward_p);
                if odd {
                    push(&mut rng, &mut frames, &df11_squitter(ac.icao), t + 0.02, &pe, forward_p);
                }
            }
        }
        odd = !odd;
        t += 0.25;
    }
    frames.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let stats = Arc::new(Stats::default());
    let (out, mut sbs) = broadcast::channel(1 << 16);
    let (bench, mut bench_rx) = broadcast::channel(1 << 16);
    let mut eng = Engine::new(EngineConfig { shadow: true, mlat_delay: 1.5, holdout_every: 20 }, stats.clone(), out).with_bench_output(bench);
    let cfg = AdapterConfig { sync_every: 2, bench_every: 1, probation: 10.0, ..Default::default() };
    let mut ad = Adapter::new(cfg, Arc::new(AdapterStats::default()));
    let mut evs = Vec::new();
    ad.set_registry(registry, &mut evs);
    let (mut next_tick, mut next_second, mut next_house) = (0.0, 0.0, 10.0);
    for (at, f) in frames {
        while next_tick <= at {
            eng.tick_at(epoch + next_tick, next_tick >= next_house);
            if next_tick >= next_house {
                next_house += 10.0;
            }
            if next_tick >= next_second {
                ad.tick(epoch + next_tick, &mut evs);
                next_second += 1.0;
            }
            for ev in evs.drain(..) {
                eng.handle_at(ev, epoch + next_tick);
            }
            next_tick += 0.05;
        }
        ad.on_frame(&f, epoch + at, &mut evs);
        for ev in evs.drain(..) {
            eng.handle_at(ev, epoch + at);
        }
    }
    eng.tick_at(epoch + duration + 5.0, true);

    let score = |rx: &mut broadcast::Receiver<Arc<str>>, want_adsb: bool| {
        let mut errs = Vec::new();
        while let Ok(line) = rx.try_recv() {
            let f: Vec<&str> = line.trim_end().split(',').collect();
            let icao = u32::from_str_radix(f[4], 16).unwrap_or(0);
            let Some(ac) = acs.iter().find(|a| a.icao == icao) else { continue };
            assert_eq!(ac.adsb, want_adsb, "{icao:06X} in the wrong output");
            let (lat, lon): (f64, f64) = (f[14].parse().unwrap(), f[15].parse().unwrap());
            let hms: Vec<f64> = f[7].split(':').map(|x| x.parse().unwrap()).collect();
            let t_fix = hms[0] * 3600.0 + hms[1] * 60.0 + hms[2] - (epoch % 86_400.0);
            if t_fix < 60.0 {
                continue;
            }
            let truth = ac.llh(t_fix);
            errs.push(haversine_m(lat, lon, truth.lat, truth.lon));
        }
        Scored { errs }
    };
    let modes_only = score(&mut sbs, false);
    let bench_s = score(&mut bench_rx, true);
    (modes_only, bench_s, stats)
}

#[test]
fn feed_path_multilaterates_without_mlat_client() {
    let (m, b, stats) = run_feed_sim(11, 8, 0.7, 180.0);
    let snap = |a: &std::sync::atomic::AtomicU64| a.load(std::sync::atomic::Ordering::Relaxed);
    eprintln!(
        "feed sim: sync_points={} pair_updates={} groups={} fixes={} outputs={} bench_outputs={} modes-only n={} bench n={}",
        snap(&stats.sync_points),
        snap(&stats.pair_updates),
        snap(&stats.groups),
        snap(&stats.fixes),
        snap(&stats.outputs),
        snap(&stats.bench_outputs),
        m.errs.len(),
        b.errs.len()
    );
    assert!(m.errs.len() > 100, "too few Mode-S-only positions: {}", m.errs.len());
    assert!(b.errs.len() > 100, "too few bench positions: {}", b.errs.len());
    let (m50, m90, b50) = (m.quantile(0.5), m.quantile(0.9), b.quantile(0.5));
    eprintln!("feed sim: Mode-S-only p50 {m50:.0} m p90 {m90:.0} m; bench p50 {b50:.0} m");
    assert!(m50 < 150.0 && m90 < 600.0, "Mode-S-only p50 {m50:.0} p90 {m90:.0}");
    assert!(b50 < 150.0, "bench p50 {b50:.0}");
}
