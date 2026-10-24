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

use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::Read;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use clap::Parser as _;
use flate2::read::GzDecoder;
use serde_json::json;
use tokio::sync::broadcast;

use crate::engine::{Engine, EngineConfig, Stats};
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
    let doc: registry::Doc = serde_json::from_reader(File::open(&a.receivers)?)?;
    let reg = registry::parse(&doc);
    let mut srcs: Vec<Source> = a.captures.iter().map(|c| Source::open(c)).collect::<anyhow::Result<_>>()?;
    let t0 = srcs.iter().map(|s| s.start).fold(f64::INFINITY, f64::min);
    let t_end = srcs.iter().map(|s| s.start + s.dur).fold(f64::NEG_INFINITY, f64::max);

    let stats = Arc::new(Stats::default());
    let (out, mut out_rx) = broadcast::channel(1 << 16);
    let (bench, mut bench_rx) = broadcast::channel(1 << 16);
    let ecfg = EngineConfig { shadow: true, mlat_delay: a.mlat_delay, holdout_every: a.holdout_every };
    let mut eng = Engine::new(ecfg, stats.clone(), out).with_bench_output(bench);
    let astats = Arc::new(AdapterStats::default());
    let acfg = AdapterConfig {
        sync_every: a.sync_every,
        bench_every: a.bench_every,
        modes_only_after: a.modes_only_after,
        probation: a.probation,
        ..AdapterConfig::default()
    };
    let mut ad = Adapter::new(acfg, astats.clone());
    let mut evs = Vec::new();
    ad.set_registry(reg, &mut evs);

    let (mut main_lines, mut bench_lines) = (0usize, 0usize);
    let (mut main_ac, mut bench_ac) = (HashSet::new(), HashSet::new());
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
                next_second += 1.0;
                drain(&mut out_rx, &mut bench_rx);
            }
            for ev in evs.drain(..) {
                events += 1;
                eng.handle_at(ev, next_tick);
            }
            next_tick += 0.05;
        }
        for f in &chunk {
            frames += 1;
            ad.on_frame(f, wall, &mut evs);
        }
        for ev in evs.drain(..) {
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
    let wall_s = started.elapsed().as_secs_f64();

    let errs = eng.adsb_errors();
    let fix_errs: Vec<f64> = errs.iter().filter_map(|e| e.1).collect();
    let out_errs: Vec<f64> = errs.iter().filter_map(|e| e.2).collect();
    let mut by_rx: HashMap<&str, Vec<f64>> = HashMap::new();
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
    let l = |x: &AtomicU64| x.load(Ordering::Relaxed);
    let snap = stats.snapshot.lock().unwrap().clone();
    let summary = json!({
        "sim_seconds": t_end - t0,
        "processing_seconds": (wall_s * 10.0).round() / 10.0,
        "frames": frames,
        "events": events,
        "adapter": {
            "frames_unmapped": l(&astats.frames_unmapped), "frames_not_live": l(&astats.frames_not_live),
            "frames_zero_ts": l(&astats.frames_zero_ts), "frames_modeac": l(&astats.frames_modeac),
            "sync_events": l(&astats.ev_sync), "mlat_modes_only_events": l(&astats.ev_mlat_modes_only),
            "mlat_bench_events": l(&astats.ev_mlat_bench), "clock_jumps": l(&astats.clock_jumps),
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
            "holdout": {"n": snap.holdout_n, "p50_m": snap.holdout_p50.round(), "p90_m": snap.holdout_p90.round()},
        },
        "bench": {
            "fix_error": quantiles(fix_errs),
            "published_error": quantiles(out_errs),
            "fix_error_by_receivers": by_rx.into_iter().map(|(k, v)| (k.to_string(), quantiles(v))).collect::<HashMap<_, _>>(),
            "positions": bench_lines, "aircraft": bench_ac.len(),
        },
        "modes_only": {"positions": main_lines, "aircraft": main_ac.len()},
    });
    let text = serde_json::to_string_pretty(&summary)?;
    println!("{text}");
    if let Some(p) = a.out {
        std::fs::write(p, text)?;
    }
    Ok(())
}
