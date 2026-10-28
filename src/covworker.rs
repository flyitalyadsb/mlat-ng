//! The coverage maps on their own thread (docs/MULTICORE.md, step 3).
//!
//! Counting every ADS-B sample into the maps of the receivers in range was
//! ~25% of the engine thread (2026-10-28 profile), and Lightcone's questions
//! were answered there too. Stage 1 now sends the maps what changes them, in
//! the order it happens: a sample with the receivers that report seeing the
//! aircraft at that moment (stage 1 does the per-aircraft sampling, so only
//! samples that count are sent), receivers placed and removed, fixes to
//! count. The maps answer Lightcone, save themselves, and hand out the
//! snapshots the workers' 2-site path reads, each taken exactly where stage 1
//! asked for it in the stream. Inline or threaded, the maps see the same
//! messages in the same order, so they end up the same.

use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::{mpsc as std_mpsc, Arc};

use crate::coverage::{CovId, Coverage};
use crate::engine::{unix_now, CovQuery, CovWhat};

/// What stage 1 sends the maps, in the order it must be applied.
pub(crate) enum ToCoverage {
    /// An ADS-B position to count, with the coverage ids (sorted) of the
    /// receivers that report seeing the aircraft.
    Observe { icao: u32, lat: f64, lon: f64, alt_m: f64, now: f64, seeing: Vec<CovId> },
    Place { id: CovId, lat: f64, lon: f64, now: f64 },
    Unplace(CovId),
    Name { id: CovId, who: String },
    CountFix { id: CovId, now: f64 },
    /// A copy of the maps as they are now, one per reply.
    Snapshot(Vec<std_mpsc::Sender<Arc<Coverage>>>),
    Query(CovQuery),
    /// Save to `path` off this thread (persist::Writer).
    Save(String),
    /// Save to `path` now, then reply (shutdown, replay end).
    SaveNow(String, std_mpsc::Sender<anyhow::Result<()>>),
    Flush(std_mpsc::Sender<()>),
}

/// The maps' gauges, for /metrics.
#[derive(Default)]
pub(crate) struct CovShared {
    pub maps: AtomicUsize,
    pub samples: AtomicU64,
}

pub(crate) struct CovWorker {
    pub(crate) cov: Coverage,
    shared: Arc<CovShared>,
    writer: crate::persist::Writer,
}

impl CovWorker {
    fn handle(&mut self, m: ToCoverage) {
        match m {
            ToCoverage::Observe { icao, lat, lon, alt_m, now, seeing } => {
                self.cov.observe(icao, lat, lon, alt_m, now, |id| seeing.binary_search(&id).is_ok());
            }
            ToCoverage::Place { id, lat, lon, now } => self.cov.place(id, lat, lon, now),
            ToCoverage::Unplace(id) => self.cov.unplace(id),
            ToCoverage::Name { id, who } => self.cov.name(id, &who),
            ToCoverage::CountFix { id, now } => self.cov.count_fix(id, now),
            ToCoverage::Snapshot(replies) => {
                let c = Arc::new(self.cov.clone());
                for r in replies {
                    let _ = r.send(c.clone());
                }
            }
            ToCoverage::Query(q) => {
                let now = unix_now();
                let body = match q.what {
                    CovWhat::Map(id) => self.cov.map_json(id, now).map(|v| v.to_string()),
                    CovWhat::Summary => Some(self.cov.summary_json(now).to_string()),
                };
                let _ = q.reply.send(body);
            }
            ToCoverage::Save(path) => match self.cov.encode() {
                Ok(b) => self.writer.spawn(path, b),
                Err(e) => tracing::warn!(path, error = %e, "coverage maps not encoded"),
            },
            ToCoverage::SaveNow(path, reply) => {
                let _ = reply.send(self.cov.encode().and_then(|b| crate::persist::write_atomic(&path, &b)));
            }
            ToCoverage::Flush(reply) => {
                let _ = reply.send(());
            }
        }
        self.publish();
    }

    fn publish(&self) {
        self.shared.maps.store(self.cov.receivers(), Ordering::Relaxed);
        self.shared.samples.store(self.cov.samples, Ordering::Relaxed);
    }

    /// Start from these maps (loaded from a file).
    pub(crate) fn set(&mut self, cov: Coverage) {
        self.cov = cov;
        self.publish();
    }
}

/// Messages the maps' thread may have queued before stage 1 waits for it.
const QUEUE: usize = 65_536;

pub(crate) enum CovHandle {
    Inline(Box<CovWorker>),
    Thread { tx: Option<std_mpsc::SyncSender<ToCoverage>>, join: Option<std::thread::JoinHandle<()>> },
}

impl CovHandle {
    pub(crate) fn new(cov: Coverage) -> (CovHandle, Arc<CovShared>) {
        let shared = Arc::new(CovShared::default());
        let w = CovWorker { cov, shared: shared.clone(), writer: crate::persist::Writer::default() };
        w.publish();
        (CovHandle::Inline(Box::new(w)), shared)
    }

    /// The maps, while still inline (loading them at start).
    pub(crate) fn inline_mut(&mut self) -> Option<&mut CovWorker> {
        match self {
            CovHandle::Inline(w) => Some(w.as_mut()),
            CovHandle::Thread { .. } => None,
        }
    }

    pub(crate) fn spawn(&mut self) {
        let CovHandle::Inline(_) = self else { return };
        let CovHandle::Inline(mut w) = std::mem::replace(self, CovHandle::Thread { tx: None, join: None }) else { unreachable!() };
        let (tx, rx) = std_mpsc::sync_channel::<ToCoverage>(QUEUE);
        let join = std::thread::Builder::new()
            .name("coverage".into())
            .spawn(move || {
                while let Ok(m) = rx.recv() {
                    w.handle(m);
                }
            })
            .expect("coverage thread");
        *self = CovHandle::Thread { tx: Some(tx), join: Some(join) };
    }

    pub(crate) fn send(&mut self, m: ToCoverage) {
        match self {
            CovHandle::Inline(w) => w.handle(m),
            CovHandle::Thread { tx, .. } => tx.as_ref().unwrap().send(m).expect("coverage thread gone"),
        }
    }

    /// Save to `path` now (after everything sent before).
    pub(crate) fn save_now(&mut self, path: &str) -> anyhow::Result<()> {
        let (tx, rx) = std_mpsc::channel();
        self.send(ToCoverage::SaveNow(path.to_string(), tx));
        rx.recv().map_err(|_| anyhow::anyhow!("coverage thread gone"))?
    }

    pub(crate) fn flush(&mut self) {
        let (tx, rx) = std_mpsc::channel();
        self.send(ToCoverage::Flush(tx));
        let _ = rx.recv();
    }

    pub(crate) fn stop(&mut self) {
        if let CovHandle::Thread { tx, join } = self {
            drop(tx.take());
            if let Some(j) = join.take() {
                let _ = j.join();
            }
        }
    }
}
