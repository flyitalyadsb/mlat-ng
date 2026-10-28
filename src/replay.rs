//! `mlat-ng replay`: run the real feed adapter and engine offline on captures
//! of readsb-ingest's 4906 stream, on a simulated clock, and print what came
//! out: clock sync, the bench (ADS-B aircraft multilaterated from their
//! non-ADS-B replies, scored against their own ADS-B positions) and the
//! Mode-S-only positions. Captures hold feeder identities: keep them local.
//!
//! A capture is the raw byte stream, gzip'd (`cat <&3 | gzip -1` on the
//! ingest pod), given as `PATH@START_UNIX@DURATION_S`. It has no per-frame
//! arrival times, so arrival is interpolated from the compressed offset;
//! readsb flushes and the WAN already jitter arrivals by ~0.1-0.3 s.

use crate::hashing::{HashMap, HashSet};
use std::fs::File;
use std::io::Read;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use clap::Parser as _;
use flate2::read::GzDecoder;
use serde_json::json;
use tokio::sync::broadcast;

use crate::clientsim::ClientSim;
use crate::engine::{fxhash, Engine, EngineConfig, Stats};
use crate::hybrid::{Router, RouterStats, Src};
use crate::relay::adapter::{Adapter, AdapterConfig, AdapterStats};
use crate::relay::beast::{Frame, Parser};
use crate::relay::registry;

#[derive(clap::Parser, Debug)]
#[command(about = "Replay feed captures through the adapter and engine")]
pub struct ReplayArgs {
    /// PATH@START_UNIX@DURATION_S, one per cell.
    #[arg(long = "capture", required = true)]
    captures: Vec<String>,
    /// Receiver registry (the /receivers.json format).
    #[arg(long)]
    receivers: String,
    #[arg(long, default_value_t = 2)]
    sync_every: u64,
    #[arg(long, default_value_t = 10)]
    bench_every: u64,
    #[arg(long, default_value_t = 20.0)]
    modes_only_after: f64,
    #[arg(long, default_value_t = 20.0)]
    probation: f64,
    #[arg(long, default_value_t = 1.5)]
    mlat_delay: f64,
    #[arg(long, default_value_t = 20)]
    holdout_every: u64,
    /// Also write the summary JSON here.
    #[arg(long)]
    out: Option<String>,
    /// Write every Mode-S-only (main output) SBS line here.
    #[arg(long)]
    dump_sbs: Option<String>,
    /// Hybrid test: this share of receivers (by id hash) also gets a stand-in
    /// mlat-client (a second adapter on its frames) through the hybrid router.
    #[arg(long, default_value_t = 0.0)]
    hybrid_clients: f64,
    /// dup: those receivers on both inputs; split: only via the stand-in
    /// client (their feed frames withheld); drop: on neither (the baseline
    /// for what the client path brings back).
    #[arg(long, default_value = "dup")]
    hybrid_mode: String,
    /// Study: the stand-in clients' events reach the engine this many
    /// seconds late (live, mlat-client data comes through the shadow hub and
    /// client-side batching; the feed is faster).
    #[arg(long, default_value_t = 0.0)]
    hybrid_client_delay: f64,
    /// Study: and they are sent in batches this many seconds apart, each
    /// client on its own phase (mlat-client's zlib2 output is flushed ~1 s
    /// after the first line).
    #[arg(long, default_value_t = 0.0)]
    hybrid_client_batch: f64,
    /// Save the receiver coverage maps built during the replay here.
    #[arg(long)]
    coverage_file: Option<String>,
    /// Start from these coverage maps (a file saved by a live engine or an
    /// earlier replay).
    #[arg(long)]
    coverage_from: Option<String>,
    /// Write every 2-site solution of an aircraft with ADS-B here (JSON lines).
    #[arg(long)]
    two_site_dump: Option<String>,
    /// Aircraft heard by only two sites: off | bench | publish.
    #[arg(long, default_value = "off")]
    two_site: crate::engine::TwoSite,
    /// Bench of the 2-site path: aircraft with ADS-B keep the copies of one
    /// pair of receivers only, so the bench scores 2-site positions.
    #[arg(long, default_value_t = false)]
    bench_two_sites: bool,
    /// Serve mode: every receiver in the registry becomes an emulated
    /// mlat-client (clientsim.rs) instead of the feed adapter. `select`: the
    /// engine chooses what each client sends; `all`: every client sends
    /// everything (the no-selection baseline). `off`: the feed path.
    #[arg(long, default_value = "off")]
    serve_sim: String,
    /// Serve selection: seconds without ADS-B before multilaterating.
    #[arg(long, default_value_t = 120.0)]
    serve_no_adsb_s: f64,
    #[arg(long, default_value_t = 12.0)]
    serve_max_sync_rate: f64,
    /// Instead of replaying: write the frames of receivers within
    /// `--extract-km` of `--extract-near LAT,LON` into this directory, one
    /// file per receiver of (arrival time, plain Beast frame) records, for
    /// feeding real mlat-clients. Stops after `--extract-seconds` of capture.
    #[arg(long)]
    extract_dir: Option<String>,
    #[arg(long, default_value = "45.46,9.19")]
    extract_near: String,
    #[arg(long, default_value_t = 120.0)]
    extract_km: f64,
    #[arg(long, default_value_t = 300.0)]
    extract_seconds: f64,
    /// Survey test: this share of receivers is declared `--perturb-m` metres
    /// away from where the registry has them (random direction), and the
    /// summary checks whether the survey finds the offset back.
    #[arg(long, default_value_t = 0.0)]
    perturb_share: f64,
    #[arg(long, default_value_t = 300.0)]
    perturb_m: f64,
    /// Write every receiver's survey estimate (rid, d, sd, ...) here.
    #[arg(long)]
    survey_out: Option<String>,
    /// Apply the estimates of an earlier `--survey-out` to the declared
    /// positions (after any --perturb), for receivers with at least
    /// `--apply-min-windows` windows and a weakest horizontal sigma under
    /// `--apply-max-sd` metres.
    #[arg(long)]
    positions_from: Option<String>,
    #[arg(long, default_value_t = 200)]
    apply_min_windows: u32,
    #[arg(long, default_value_t = 5.0)]
    apply_max_sd: f64,
    /// Let the engine move receivers to their survey estimates as it goes,
    /// every this many seconds (0 = off).
    #[arg(long, default_value_t = 0.0)]
    survey_apply_every: f64,
    /// Seed of every hash map (hashing.rs). The output must not depend on
    /// it: two seeds giving different results is a bug.
    #[arg(long, default_value_t = 0)]
    hash_seed: u64,
}

/// Plain Beast (`0x1A '2'|'3'` + 6-byte timestamp + signal + payload, 0x1A
/// doubled), the format mlat-client reads with `--input-type beast`.
fn beast_plain(f: &Frame, out: &mut Vec<u8>) {
    out.push(0x1A);
    out.push(f.kind);
    let mut push = |b: u8| {
        out.push(b);
        if b == 0x1A {
            out.push(0x1A);
        }
    };
    for i in (0..6).rev() {
        push((f.ts >> (8 * i)) as u8);
    }
    push(f.sig);
    for &b in f.bytes() {
        push(b);
    }
}

fn extract(a: &ReplayArgs, dir: &str, reg: &HashMap<u64, registry::Receiver>) -> anyhow::Result<()> {
    use std::io::Write;
    let p: Vec<f64> = a.extract_near.split(',').map(|x| x.trim().parse()).collect::<Result<_, _>>()?;
    anyhow::ensure!(p.len() == 2, "--extract-near LAT,LON");
    let hav = |lat: f64, lon: f64| {
        let (la1, la2) = (p[0].to_radians(), lat.to_radians());
        let x = ((la2 - la1) / 2.0).sin().powi(2) + la1.cos() * la2.cos() * ((lon - p[1]).to_radians() / 2.0).sin().powi(2);
        12_742.0 * x.sqrt().asin()
    };
    std::fs::create_dir_all(dir)?;
    let mut files: HashMap<u64, (std::io::BufWriter<File>, u64)> = HashMap::default();
    let mut buf = Vec::new();
    for spec in &a.captures {
        let mut src = Source::open(spec)?;
        let end = src.start + a.extract_seconds;
        while let Some((wall, frames)) = src.next()? {
            if wall > end {
                break;
            }
            for f in &frames {
                if f.kind == b'1' {
                    continue;
                }
                let Some(r) = reg.get(&f.rid) else { continue };
                if hav(r.lat, r.lon) > a.extract_km {
                    continue;
                }
                let e = match files.get_mut(&f.rid) {
                    Some(e) => e,
                    None => {
                        let w = std::io::BufWriter::new(File::create(format!("{dir}/{:016x}.rec", f.rid))?);
                        files.entry(f.rid).or_insert((w, 0))
                    }
                };
                // Record: arrival wall time (f64 LE), length (u16 LE), then
                // the frame as plain Beast; the test server paces by the
                // wall time so every receiver's stream stays aligned.
                buf.clear();
                beast_plain(f, &mut buf);
                e.0.write_all(&wall.to_le_bytes())?;
                e.0.write_all(&(buf.len() as u16).to_le_bytes())?;
                e.0.write_all(&buf)?;
                e.1 += 1;
            }
        }
    }
    let mut list: Vec<serde_json::Value> = Vec::new();
    for (rid, (mut w, n)) in files {
        w.flush()?;
        let r = &reg[&rid];
        list.push(json!({"rid": format!("{rid:016x}"), "frames": n, "user": r.user, "lat": r.lat, "lon": r.lon, "alt": r.alt, "clock": r.clock.as_str()}));
    }
    list.sort_by_key(|v| std::cmp::Reverse(v["frames"].as_u64().unwrap_or(0)));
    std::fs::write(format!("{dir}/receivers.json"), serde_json::to_string_pretty(&list)?)?;
    eprintln!("extracted {} receivers into {dir}", list.len());
    Ok(())
}

struct Counting<R> {
    inner: R,
    n: Arc<AtomicU64>,
}

impl<R: Read> Read for Counting<R> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let k = self.inner.read(buf)?;
        self.n.fetch_add(k as u64, Ordering::Relaxed);
        Ok(k)
    }
}

struct Source {
    dec: GzDecoder<Counting<File>>,
    read: Arc<AtomicU64>,
    total: f64,
    start: f64,
    dur: f64,
    parser: Parser,
    buf: Vec<u8>,
}

impl Source {
    fn open(spec: &str) -> anyhow::Result<Source> {
        let p: Vec<&str> = spec.split('@').collect();
        anyhow::ensure!(p.len() == 3, "capture must be PATH@START_UNIX@DURATION_S: {spec}");
        let f = File::open(p[0])?;
        let total = f.metadata()?.len() as f64;
        let read = Arc::new(AtomicU64::new(0));
        Ok(Source {
            dec: GzDecoder::new(Counting { inner: f, n: read.clone() }),
            read,
            total,
            start: p[1].parse()?,
            dur: p[2].parse()?,
            parser: Parser::default(),
            buf: vec![0u8; 64 * 1024],
        })
    }

    /// Next chunk of frames with its interpolated arrival time.
    fn next(&mut self) -> anyhow::Result<Option<(f64, Vec<Frame>)>> {
        loop {
            let n = self.dec.read(&mut self.buf)?;
            if n == 0 {
                return Ok(None);
            }
            let mut frames = Vec::new();
            self.parser.push(&self.buf[..n], &mut frames);
            if !frames.is_empty() {
                let wall = self.start + self.dur * (self.read.load(Ordering::Relaxed) as f64 / self.total).min(1.0);
                return Ok(Some((wall, frames)));
            }
        }
    }
}

fn quantiles(mut v: Vec<f64>) -> serde_json::Value {
    if v.is_empty() {
        return json!({"n": 0});
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let q = |p: f64| v[((v.len() - 1) as f64 * p) as usize].round();
    let over = |x: f64| (v.iter().filter(|e| **e > x).count() as f64 / v.len() as f64 * 1000.0).round() / 10.0;
    json!({"n": v.len(), "p50_m": q(0.5), "p90_m": q(0.9), "p99_m": q(0.99), "over_1km_pct": over(1000.0), "over_5km_pct": over(5000.0)})
}

fn aircraft_of(line: &str) -> Option<String> {
    line.split(',').nth(4).map(|s| s.to_string())
}

pub fn main(argv: Vec<String>) -> anyhow::Result<()> {
    let a = ReplayArgs::parse_from(argv);
    crate::hashing::set_seed(a.hash_seed);
    let doc: registry::Doc = serde_json::from_reader(File::open(&a.receivers)?)?;
    let mut reg = registry::parse(&doc);
    // Survey test: move a share of the declared positions; remember by how
    // much (east, north metres), keyed by receiver id.
    let mut moved: HashMap<u64, (f64, f64)> = HashMap::default();
    if a.perturb_share > 0.0 {
        for (rid, r) in reg.iter_mut() {
            let h = fxhash(&rid.to_le_bytes());
            if (h % 10_000) as f64 >= a.perturb_share * 10_000.0 {
                continue;
            }
            let bearing = ((h >> 20) % 360) as f64;
            let (e, n) = (a.perturb_m * bearing.to_radians().sin(), a.perturb_m * bearing.to_radians().cos());
            r.lat += n / 111_320.0;
            r.lon += e / (111_320.0 * r.lat.to_radians().cos());
            moved.insert(*rid, (e, n));
        }
    }
    let mut applied = 0usize;
    if let Some(p) = &a.positions_from {
        let rows: Vec<serde_json::Value> = serde_json::from_reader(File::open(p)?)?;
        for row in rows {
            let Some(rid) = row["rid"].as_str().and_then(|x| u64::from_str_radix(x, 16).ok()) else { continue };
            let windows = row["windows"].as_u64().unwrap_or(0) as u32;
            let weak = row["sd_weak_h"].as_f64().unwrap_or(f64::INFINITY);
            if windows < a.apply_min_windows || weak >= a.apply_max_sd {
                continue;
            }
            let (Some(r), Some(d)) = (reg.get_mut(&rid), row["d"].as_array()) else { continue };
            let d: Vec<f64> = d.iter().filter_map(|x| x.as_f64()).collect();
            if d.len() != 3 {
                continue;
            }
            r.lat += d[1] / 111_320.0;
            r.lon += d[0] / (111_320.0 * r.lat.to_radians().cos());
            r.alt += d[2];
            applied += 1;
        }
    }
    if let Some(dir) = &a.extract_dir {
        return extract(&a, dir, &reg);
    }
    let mut srcs: Vec<Source> = a.captures.iter().map(|c| Source::open(c)).collect::<anyhow::Result<_>>()?;
    let t0 = srcs.iter().map(|s| s.start).fold(f64::INFINITY, f64::min);
    let t_end = srcs.iter().map(|s| s.start + s.dur).fold(f64::NEG_INFINITY, f64::max);

    let stats = Arc::new(Stats::default());
    let (out, mut out_rx) = broadcast::channel(1 << 16);
    let (bench, mut bench_rx) = broadcast::channel(1 << 16);
    let ecfg = EngineConfig { shadow: true, mlat_delay: a.mlat_delay, holdout_every: a.holdout_every };
    let mut eng = Engine::new(ecfg, stats.clone(), out)
        .with_bench_output(bench)
        .with_survey_apply((a.survey_apply_every > 0.0).then(|| crate::survey::ApplyCfg { every_s: a.survey_apply_every, ..Default::default() }))
        .with_two_site(a.two_site)
        .with_bench_two_sites(a.bench_two_sites)
        .with_serve_params(crate::interest::Params {
            no_adsb_mlat_s: a.serve_no_adsb_s,
            max_sync_rate: a.serve_max_sync_rate,
            ..crate::interest::Params::default()
        });
    if let Some(p) = &a.coverage_from {
        eng = eng.load_coverage(p)?;
    }
    if let Some(p) = &a.two_site_dump {
        eng = eng.with_two_site_dump(p)?;
    }
    let astats = Arc::new(AdapterStats::default());
    let acfg = AdapterConfig {
        sync_every: a.sync_every,
        bench_every: a.bench_every,
        modes_only_after: a.modes_only_after,
        probation: a.probation,
        ..AdapterConfig::default()
    };
    anyhow::ensure!(["off", "select", "all"].contains(&a.serve_sim.as_str()), "--serve-sim: off|select|all");
    let serve_sim = a.serve_sim != "off";
    let mut csim = ClientSim::new(a.serve_sim == "select");
    let mut cs_evs = Vec::new();
    let sim_reg = reg.clone();
    let mut ad = Adapter::new(acfg.clone(), astats.clone());
    let mut evs = Vec::new();
    if !serve_sim {
        ad.set_registry(reg.clone(), &mut evs);
    }
    // Hybrid test: stand-in mlat-clients for a share of receivers.
    anyhow::ensure!(["dup", "split", "drop"].contains(&a.hybrid_mode.as_str()), "--hybrid-mode: dup|split|drop");
    let hyb = a.hybrid_clients > 0.0;
    let in_clients = |rid: u64| fxhash(&rid.to_be_bytes()) % 1000 < (a.hybrid_clients * 1000.0) as u64;
    let feed_takes = |rid: u64| !hyb || a.hybrid_mode == "dup" || !in_clients(rid);
    let emu_on = hyb && a.hybrid_mode != "drop";
    let emu_stats = Arc::new(AdapterStats::default());
    let mut emu = Adapter::new(acfg, emu_stats.clone()).with_conn_base(1 << 61);
    let mut emu_evs = Vec::new();
    emu.set_registry(reg, &mut emu_evs);
    let mut delayed: std::collections::BinaryHeap<std::cmp::Reverse<(u64, u64)>> = std::collections::BinaryHeap::new();
    let mut delayed_ev: HashMap<u64, crate::types::Event> = HashMap::default();
    let mut delayed_seq = 0u64;
    // When a stand-in client's event reaches the engine: its batch's flush
    // (each connection on its own phase) plus the network delay.
    let due_of = |ev: &crate::types::Event, t: f64| -> f64 {
        let conn = match ev {
            crate::types::Event::Sync { conn, .. } | crate::types::Event::Mlat { conn, .. } => *conn,
            _ => 0,
        };
        let b = a.hybrid_client_batch;
        let wait = if b > 0.0 {
            let phase = (fxhash(&conn.to_be_bytes()) % 1000) as f64 / 1000.0 * b;
            (phase - t).rem_euclid(b)
        } else {
            0.0
        };
        t + a.hybrid_client_delay + wait
    };
    let rstats = Arc::new(RouterStats::default());
    let mut router = Router::new(rstats.clone());
    let mut routed: Vec<crate::types::Event> = Vec::new();
    let mut client_rids = HashSet::default();

    let (mut main_lines, mut bench_lines) = (0usize, 0usize);
    let (mut main_ac, mut bench_ac) = (HashSet::default(), HashSet::default());
    let mut dump: Option<std::io::BufWriter<File>> = a.dump_sbs.as_ref().map(File::create).transpose()?.map(std::io::BufWriter::new);
    let mut drain = |out_rx: &mut broadcast::Receiver<Arc<str>>, bench_rx: &mut broadcast::Receiver<Arc<str>>| {
        while let Ok(l) = out_rx.try_recv() {
            main_lines += 1;
            main_ac.extend(aircraft_of(&l));
            if let Some(d) = dump.as_mut() {
                let _ = std::io::Write::write_all(d, l.as_bytes());
            }
        }
        while let Ok(l) = bench_rx.try_recv() {
            bench_lines += 1;
            bench_ac.extend(aircraft_of(&l));
        }
    };

    let started = Instant::now();
    let mut pending: Vec<Option<(f64, Vec<Frame>)>> = srcs.iter_mut().map(|s| s.next()).collect::<anyhow::Result<_>>()?;
    let mut next_tick = t0;
    let mut next_second = t0;
    let mut next_house = t0 + 10.0;
    let mut frames = 0u64;
    let mut events = 0u64;
    loop {
        let Some(i) = (0..pending.len())
            .filter(|&i| pending[i].is_some())
            .min_by(|&x, &y| pending[x].as_ref().unwrap().0.partial_cmp(&pending[y].as_ref().unwrap().0).unwrap())
        else {
            break;
        };
        let (wall, chunk) = pending[i].take().unwrap();
        while next_tick <= wall {
            let house = next_tick >= next_house;
            eng.tick_at(next_tick, house);
            if house {
                next_house += 10.0;
            }
            if next_tick >= next_second {
                ad.tick(next_tick, &mut evs);
                if emu_on {
                    emu.tick(next_tick, &mut emu_evs);
                }
                next_second += 1.0;
                drain(&mut out_rx, &mut bench_rx);
            }
            for ev in evs.drain(..) {
                // Without stand-in clients the router stays out of the way:
                // the baseline is exactly the production feed path.
                if hyb {
                    router.on_event(Src::Feed, ev, &mut routed);
                } else {
                    routed.push(ev);
                }
            }
            for ev in emu_evs.drain(..) {
                let due = due_of(&ev, next_tick);
                delayed_seq += 1;
                delayed.push(std::cmp::Reverse(((due * 1e6) as u64, delayed_seq)));
                delayed_ev.insert(delayed_seq, ev);
            }
            while delayed.peek().is_some_and(|std::cmp::Reverse((due, _))| *due as f64 / 1e6 <= next_tick) {
                let std::cmp::Reverse((due, k)) = delayed.pop().unwrap();
                let mut ev = delayed_ev.remove(&k).unwrap();
                // Live, the server stamps an mlat line when it reads it.
                if let crate::types::Event::Mlat { rx, .. } = &mut ev {
                    *rx = due as f64 / 1e6;
                }
                router.on_event(Src::Client, ev, &mut routed);
            }
            if serve_sim {
                csim.tick(next_tick, &mut cs_evs);
                routed.append(&mut cs_evs);
            }
            for ev in routed.drain(..) {
                events += 1;
                eng.handle_at(ev, next_tick);
            }
            next_tick += 0.05;
        }
        for f in &chunk {
            frames += 1;
            if serve_sim {
                csim.on_frame(f, wall, &sim_reg, 1 << 60, &mut cs_evs);
                continue;
            }
            if feed_takes(f.rid) {
                ad.on_frame(f, wall, &mut evs);
            }
            if emu_on && in_clients(f.rid) {
                client_rids.insert(f.rid);
                emu.on_frame(f, wall, &mut emu_evs);
            }
        }
        for ev in evs.drain(..) {
            if hyb {
                router.on_event(Src::Feed, ev, &mut routed);
            } else {
                routed.push(ev);
            }
        }
        for ev in emu_evs.drain(..) {
            let due = due_of(&ev, wall);
            delayed_seq += 1;
            delayed.push(std::cmp::Reverse(((due * 1e6) as u64, delayed_seq)));
            delayed_ev.insert(delayed_seq, ev);
        }
        while delayed.peek().is_some_and(|std::cmp::Reverse((due, _))| *due as f64 / 1e6 <= wall) {
            let std::cmp::Reverse((due, k)) = delayed.pop().unwrap();
            let mut ev = delayed_ev.remove(&k).unwrap();
            if let crate::types::Event::Mlat { rx, .. } = &mut ev {
                *rx = due as f64 / 1e6;
            }
            router.on_event(Src::Client, ev, &mut routed);
        }
        routed.append(&mut cs_evs);
        for ev in routed.drain(..) {
            events += 1;
            eng.handle_at(ev, wall);
        }
        pending[i] = srcs[i].next()?;
    }
    // let the last groups and tracks settle
    let mut t = next_tick;
    while t < t_end + 5.0 {
        eng.tick_at(t, false);
        t += 0.05;
    }
    eng.tick_at(t, true);
    drain(&mut out_rx, &mut bench_rx);
    eng.publish_at(0, t);
    eng.flush_two_site_dump();
    let wall_s = started.elapsed().as_secs_f64();
    if let Some(p) = &a.coverage_file {
        eng.save_coverage(p)?;
    }

    // 2-site solutions vs the aircraft's own ADS-B (bench aircraft only).
    let two = eng.two_site_errors();
    let qs = |mut v: Vec<f64>| {
        v.sort_by(|a, b| a.total_cmp(b));
        let at = |p: f64| if v.is_empty() { 0.0 } else { (v[((v.len() - 1) as f64 * p) as usize] / 10.0).round() / 100.0 };
        json!({"n": v.len(), "p50_km": at(0.5), "p90_km": at(0.9), "p99_km": at(0.99),
               "under_1km": v.iter().filter(|e| **e < 1000.0).count(), "over_5km": v.iter().filter(|e| **e > 5000.0).count()})
    };
    let two_site = json!({
        "tries": stats.two_site_tries.load(Ordering::Relaxed),
        "solved": stats.two_site_solved.load(Ordering::Relaxed),
        "accepted": stats.two_site_accepted.load(Ordering::Relaxed),
        "tracked": stats.two_site_tracked.load(Ordering::Relaxed),
        "solve_ms_mean": stats.two_site_us.load(Ordering::Relaxed) as f64
            / stats.two_site_tries.load(Ordering::Relaxed).max(1) as f64 / 1000.0,
        "all": qs(two.iter().map(|x| x.1).collect()),
        "accepted_err": qs(two.iter().filter(|x| x.2).map(|x| x.1).collect()),
        "published_err": qs(eng.two_site_published().iter().map(|x| x.1).collect()),
    });
    let errs = eng.adsb_errors();
    let fix_errs: Vec<f64> = errs.iter().filter_map(|e| e.1).collect();
    let out_errs: Vec<f64> = errs.iter().filter_map(|e| e.2).collect();
    let mut by_rx: HashMap<&str, Vec<f64>> = HashMap::default();
    for (n, f, _) in &errs {
        let b = match n {
            0..=3 => "2-3",
            4..=5 => "4-5",
            6..=9 => "6-9",
            _ => "10+",
        };
        if let Some(f) = f {
            by_rx.entry(b).or_default().push(*f);
        }
    }
    // Survey: estimates are by coverage identity, which for feed receivers
    // is the receiver id. A moved receiver should come back with d = -move.
    let est: HashMap<u64, crate::survey::Estimate> = eng.survey_estimates().into_iter().collect();
    let (s_obs, s_win, s_drop) = eng.survey_counts();
    let mut found_err = Vec::new();
    let mut flagged_moved = 0usize;
    let mut still = Vec::new();
    let mut flagged_still = 0usize;
    let flag = |e: &crate::survey::Estimate| {
        let h = e.d[0].hypot(e.d[1]);
        h > 100.0 && h > 4.0 * e.sd[0].hypot(e.sd[1])
    };
    for (rid, e) in &est {
        match moved.get(rid) {
            Some(&(me, mn)) => {
                found_err.push((e.d[0] + me).hypot(e.d[1] + mn));
                flagged_moved += flag(e) as usize;
            }
            None => {
                still.push(e.d[0].hypot(e.d[1]));
                flagged_still += flag(e) as usize;
            }
        }
    }
    if let Some(p) = &a.survey_out {
        let rows: Vec<serde_json::Value> = est
            .iter()
            .map(|(rid, e)| json!({"rid": format!("{rid:016x}"), "moved": moved.get(rid), "d": e.d, "sd": e.sd,
                                   "sigma": e.sigma, "sd_weak_h": e.sd_weak_h, "windows": e.windows, "dropped": e.dropped}))
            .collect();
        std::fs::write(p, serde_json::to_string(&rows)?)?;
    }
    let qm = |mut v: Vec<f64>| {
        v.sort_by(|a, b| a.total_cmp(b));
        let q = |p: f64| if v.is_empty() { 0.0 } else { v[((v.len() - 1) as f64 * p) as usize].round() };
        json!({"n": v.len(), "p50_m": q(0.5), "p90_m": q(0.9), "under_50m": v.iter().filter(|x| **x < 50.0).count()})
    };
    let survey = json!({
        "applied": applied,
        "observations": s_obs, "windows": s_win, "windows_dropped": s_drop, "receivers": est.len(),
        "moved": {"count": moved.len(), "estimated": found_err.len(), "error_vs_move": qm(found_err), "flagged": flagged_moved},
        "not_moved": {"offset": qm(still), "flagged": flagged_still},
    });
    let l = |x: &AtomicU64| x.load(Ordering::Relaxed);
    let snap = stats.snapshot.lock().unwrap().clone();
    let (sent, req_mean) = csim.totals();
    let secs = (t_end - t0).max(1.0);
    let n_cl = csim.len().max(1) as f64;
    let per_client: Vec<f64> = csim.per_client().into_iter().map(|n| n as f64 / secs).collect();
    let pc = {
        let mut v = per_client;
        v.sort_by(|a, b| a.total_cmp(b));
        let q = |p: f64| if v.is_empty() { 0.0 } else { (v[((v.len() - 1) as f64 * p) as usize] * 100.0).round() / 100.0 };
        json!({"p50": q(0.5), "p90": q(0.9), "p99": q(0.99), "max": q(1.0)})
    };
    let serve = json!({
        "mode": a.serve_sim,
        "clients": csim.len(),
        "msgs_per_s_per_client": pc,
        "sync_per_s_per_client": ((sent.sync as f64 / secs / n_cl) * 100.0).round() / 100.0,
        "mlat_per_s_per_client": ((sent.mlat as f64 / secs / n_cl) * 100.0).round() / 100.0,
        "sync_total": sent.sync, "mlat_total": sent.mlat,
        "results_to_clients": sent.results,
        "start_lines": sent.start_lines, "stop_lines": sent.stop_lines,
        "requested_peak_mean": (req_mean * 10.0).round() / 10.0,
        "engine_requested_sync": snap.serve_requested_sync, "engine_requested_mlat": snap.serve_requested_mlat,
        "engine_mlat_wanted": snap.serve_mlat_wanted,
    });
    let summary = json!({
        "survey": survey,
        "serve_sim": serve,
        "two_site": two_site,
        "sim_seconds": t_end - t0,
        "processing_seconds": (wall_s * 10.0).round() / 10.0,
        "frames": frames,
        "events": events,
        "adapter": {
            "frames_unmapped": l(&astats.frames_unmapped), "frames_not_live": l(&astats.frames_not_live),
            "frames_zero_ts": l(&astats.frames_zero_ts), "frames_magic_ts": l(&astats.frames_magic_ts), "frames_modeac": l(&astats.frames_modeac),
            "sync_events": l(&astats.ev_sync), "mlat_modes_only_events": l(&astats.ev_mlat_modes_only),
            "mlat_bench_events": l(&astats.ev_mlat_bench), "clock_jumps": l(&astats.clock_jumps),
            "clock_resets": l(&astats.clock_resets), "receivers_multi_clock": l(&astats.receivers_multi_clock),
            "receivers_live": l(&astats.receivers_live), "receivers_bad_clock": l(&astats.receivers_bad_clock),
            "receivers_probation": l(&astats.receivers_probation), "receivers_unmapped": l(&astats.receivers_unmapped),
        },
        "engine": {
            "sync_points": l(&stats.sync_points), "sync_invalid": l(&stats.sync_invalid),
            "pair_updates": l(&stats.pair_updates), "pair_rejects": l(&stats.pair_rejects),
            "pair_budget_refused": l(&stats.pair_budget_refused),
            "pairs": snap.pairs, "pairs_usable": snap.pairs_usable, "receivers": snap.receivers,
            "receivers_suspect": snap.receivers_suspect,
            "groups": l(&stats.groups), "groups_unknown_aircraft": l(&stats.groups_unknown_ac),
            "groups_unnormalised": l(&stats.groups_no_norm), "clusters": l(&stats.clusters),
            "solves": l(&stats.solves), "fixes": l(&stats.fixes), "tracks_born": l(&stats.tracks_born),
            "track_tdoa_updates": l(&stats.track_tdoa_updates),
            "commb_vel_offered": l(&stats.commb_vel_offered), "commb_vel_accepted": l(&stats.commb_vel_accepted),
            "holdout": {"n": snap.holdout_n, "p50_m": snap.holdout_p50.round(), "p90_m": snap.holdout_p90.round()},
            "coverage_maps": snap.coverage_maps, "coverage_samples": snap.coverage_samples,
            "alt_model": {"cells": snap.alt_model_cells, "samples": snap.alt_model_samples,
                "corrected": l(&stats.alt_geo_corrected), "fallback": l(&stats.alt_geo_fallback),
                "sync_refs_geometric": l(&stats.sync_refs_geometric)},
            "bridging": {"groups": l(&stats.groups_bridged), "receivers": l(&stats.bridge_receivers),
                "copies_gained": l(&stats.bridged_copies)},
        },
        "bench": {
            "fix_error": quantiles(fix_errs),
            "published_error": quantiles(out_errs),
            "fix_error_by_receivers": by_rx.into_iter().map(|(k, v)| (k.to_string(), quantiles(v))).collect::<HashMap<_, _>>(),
            "positions": bench_lines, "aircraft": bench_ac.len(),
        },
        // Aircraft on the main output that never showed up on the bench (no
        // ADS-B position during the whole run): the truly Mode-S-only ones.
        "modes_only": {"positions": main_lines, "aircraft": main_ac.len(),
            "aircraft_never_adsb": main_ac.difference(&bench_ac).count()},
        "hybrid": {
            "clients_share": a.hybrid_clients, "mode": a.hybrid_mode, "client_receivers": client_rids.len(),
            "stand_in_clients_live": l(&emu_stats.receivers_live),
            "merged": l(&rstats.merged), "owner_client": l(&rstats.owner_client), "owner_feed": l(&rstats.owner_feed),
            "remapped": l(&rstats.remapped), "dup_mlat": l(&rstats.dup_mlat), "dup_sync": l(&rstats.dup_sync),
            "secondary_dropped": l(&rstats.secondary_dropped), "handovers": l(&rstats.handovers),
        },
    });
    let text = serde_json::to_string_pretty(&summary)?;
    println!("{text}");
    if let Some(p) = a.out {
        std::fs::write(p, text)?;
    }
    Ok(())
}
