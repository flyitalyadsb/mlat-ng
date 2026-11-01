//! Readiness, drain and the gated serve listener (M1 in docs/PLAN.md,
//! "Redundancy").
//!
//! A served region runs on two instances, an active one and a warm standby.
//! mlat-proxy (`mlat_shard_proxy.py`, "pool di backend") polls each one's
//! `/readyz` every 2 s and serves from one that answers 200. The proxy reads
//! the status and the first word of the body only (`check_member`):
//!
//! - `200 ok`: warm and serving.
//! - `503 draining`: SIGTERM came, the process is about to stop. The proxy
//!   leaves it at once (no hysteresis) and closes its feeders' connections,
//!   if the other instance is usable; mlat-client reconnects in 2-4 s.
//! - `503 stalled: ...`, `503 warming: ...`: not usable. The proxy counts a
//!   failed check (3 in a row = down), as for no answer at all. Kubernetes'
//!   readiness probe reads the same endpoint.
//!
//! Warm means all of: not draining; the engine published its snapshot in the
//! last `STALL_S` seconds (it does every second); up for at least
//! `--ready-min-uptime-s` (the hub's reconnect backoff and the feed's
//! registry need that long); with `--feed`, a cell that delivered frames in
//! the last `FEED_FRESH_S` seconds and a non-empty registry; and at least
//! `--ready-min-usable` of the clock pairs usable.
//!
//! Those warming criteria gate only the first 200 (`--ready-latch`, on by
//! default). Once ready, only draining and stalled take the 200 away: a
//! takeover's reconnect wave resets the clock pairs of every feeder that
//! moves here, and the usable share dips under its threshold for a few
//! seconds (the 2026-11-01 failover harness: 0.37-0.44 for about 4 s,
//! starting 4-6 s after the switch). Answering `warming` then made the proxy
//! mark the member that had just taken over down; with a usable peer it
//! would move every served feeder back.
//!
//! The serve listener is bound only once warm, or after
//! `--serve-gate-max-s` anyway: a cold instance refuses, so the proxy's
//! connect fails at once and it tries the other instance within the same
//! client connection. The timeout keeps two cold instances from waiting on
//! each other. On SIGTERM main.rs drains: `/readyz` says draining, the serve
//! listener closes (new connects are refused), the served feeders already
//! connected keep being served until none is left or `--drain-max-s`
//! passes, and then the process stops as before (state saved).

use std::future::Future;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Context as _;
use tokio::net::TcpListener;
use tokio::sync::watch;

use crate::engine::{unix_now, Stats};
use crate::relay::FeedStats;

/// The engine's snapshot is rewritten every second; older than this, the
/// engine is stuck.
pub const STALL_S: f64 = 10.0;
/// A feed cell counts as connected while it delivered frames this recently.
pub const FEED_FRESH_S: f64 = 30.0;
/// How often the gate looks at readiness, and the drain at the sessions.
pub const POLL: Duration = Duration::from_secs(1);

#[derive(Debug, Clone, Copy)]
pub struct ReadyConfig {
    pub min_uptime_s: f64,
    /// Share of the clock pairs that must be usable, 0..=1.
    pub min_usable: f64,
    /// `--feed` is on: a delivering cell and a registry are needed too.
    pub feed: bool,
    /// Once ready, stay ready unless draining or stalled (`Lifecycle::verdict`).
    pub latch: bool,
}

/// What readiness is judged on, read at each question.
#[derive(Debug, Clone, Default)]
pub struct Inputs {
    pub uptime_s: f64,
    /// Seconds since the engine's last snapshot; None before the first.
    pub snapshot_age_s: Option<f64>,
    pub pairs: usize,
    pub pairs_usable: usize,
    /// Feed cells that delivered frames in the last `FEED_FRESH_S` seconds.
    pub cells_fresh: usize,
    /// Receivers in the feed's registry.
    pub registry: u64,
}

impl Inputs {
    pub fn read(uptime_s: f64, stats: &Stats, feed: Option<&FeedStats>) -> Inputs {
        let now_ms = unix_now() * 1000.0;
        let at = stats.snapshot_ms.load(Ordering::Relaxed);
        let (pairs, pairs_usable) = {
            let s = stats.snapshot.lock().unwrap();
            (s.pairs, s.pairs_usable)
        };
        let fresh = |ms: u64| ms > 0 && now_ms - ms as f64 <= FEED_FRESH_S * 1000.0;
        Inputs {
            uptime_s,
            snapshot_age_s: (at > 0).then(|| (now_ms - at as f64).max(0.0) / 1000.0),
            pairs,
            pairs_usable,
            cells_fresh: feed.map_or(0, |f| f.cells.iter().filter(|(_, c)| fresh(c.last_frame_ms.load(Ordering::Relaxed))).count()),
            registry: feed.map_or(0, |f| f.registry_size.load(Ordering::Relaxed)),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Verdict {
    Ready,
    Draining,
    Stalled(String),
    Warming(String),
}

impl Verdict {
    pub fn is_ready(&self) -> bool {
        *self == Verdict::Ready
    }

    /// The `/readyz` answer: Ok = 200 with this body, Err = 503 with it.
    pub fn http(&self) -> Result<String, String> {
        match self {
            Verdict::Ready => Ok("ok\n".into()),
            Verdict::Draining => Err("draining\n".into()),
            Verdict::Stalled(why) => Err(format!("stalled: {why}\n")),
            Verdict::Warming(why) => Err(format!("warming: {why}\n")),
        }
    }
}

/// Is this instance warm? Draining comes first: the proxy must see it even
/// from an instance that is also stalled or cold.
pub fn assess(cfg: &ReadyConfig, draining: bool, i: &Inputs) -> Verdict {
    if draining {
        return Verdict::Draining;
    }
    match i.snapshot_age_s {
        Some(age) if age > STALL_S => return Verdict::Stalled(format!("engine snapshot {age:.0} s old")),
        None if i.uptime_s > STALL_S => return Verdict::Stalled("no engine snapshot".into()),
        None => return Verdict::Warming("no engine snapshot yet".into()),
        Some(_) => {}
    }
    if i.uptime_s < cfg.min_uptime_s {
        return Verdict::Warming(format!("up {:.0} s of {:.0}", i.uptime_s, cfg.min_uptime_s));
    }
    if cfg.feed {
        if i.cells_fresh == 0 {
            return Verdict::Warming("no feed cell delivering".into());
        }
        if i.registry == 0 {
            return Verdict::Warming("receiver registry empty".into());
        }
    }
    let share = if i.pairs > 0 { i.pairs_usable as f64 / i.pairs as f64 } else { 0.0 };
    if share < cfg.min_usable {
        return Verdict::Warming(format!("usable clock pairs {share:.3} < {:.3} ({} of {})", cfg.min_usable, i.pairs_usable, i.pairs));
    }
    Verdict::Ready
}

/// The process as `/readyz` and the serve listener see it: started, ready
/// once, then (once, for good) draining.
pub struct Lifecycle {
    started: Instant,
    draining: watch::Sender<bool>,
    /// Set by the first Ready verdict, for good.
    ready_once: AtomicBool,
    /// Verdicts the latch turned from warming into ready.
    latch_held: AtomicU64,
}

impl Default for Lifecycle {
    fn default() -> Self {
        Lifecycle { started: Instant::now(), draining: watch::Sender::new(false), ready_once: AtomicBool::new(false), latch_held: AtomicU64::new(0) }
    }
}

impl Lifecycle {
    /// What `/readyz` and the serve gate answer: `assess`, with the latch.
    /// The first Ready sets it (and is logged); from then on a `warming`
    /// verdict answers Ready, while draining and stalled still answer 503.
    pub fn verdict(&self, cfg: &ReadyConfig, i: &Inputs) -> Verdict {
        match assess(cfg, self.is_draining(), i) {
            Verdict::Ready => {
                if !self.ready_once.swap(true, Ordering::Relaxed) {
                    tracing::info!(
                        uptime_s = i.uptime_s.round(),
                        pairs = i.pairs,
                        pairs_usable = i.pairs_usable,
                        latch = cfg.latch,
                        "ready: warm for the first time"
                    );
                }
                Verdict::Ready
            }
            Verdict::Warming(_) if cfg.latch && self.ready_once.load(Ordering::Relaxed) => {
                self.latch_held.fetch_add(1, Ordering::Relaxed);
                Verdict::Ready
            }
            v => v,
        }
    }

    /// `mlat_ng_ready_latched` (1 once ready, with the latch on) and
    /// `mlat_ng_ready_latch_held_total`.
    pub fn render_metrics(&self, o: &mut String, cfg: &ReadyConfig) {
        use std::fmt::Write as _;
        let latched = cfg.latch && self.ready_once.load(Ordering::Relaxed);
        let _ = writeln!(o, "# TYPE mlat_ng_ready_latched gauge\nmlat_ng_ready_latched {}", latched as u8);
        let held = self.latch_held.load(Ordering::Relaxed);
        let _ = writeln!(o, "# TYPE mlat_ng_ready_latch_held_total counter\nmlat_ng_ready_latch_held_total {held}");
    }

    pub fn uptime_s(&self) -> f64 {
        self.started.elapsed().as_secs_f64()
    }

    pub fn is_draining(&self) -> bool {
        *self.draining.borrow()
    }

    /// Start draining; true the first time.
    pub fn drain(&self) -> bool {
        !self.draining.send_replace(true)
    }

    /// Resolves once draining has started.
    pub async fn drained(&self) {
        let mut rx = self.draining.subscribe();
        let _ = rx.wait_for(|d| *d).await;
    }
}

/// The serve listener behind its gate: bound on `addr` once `ready` says
/// warm, or after `max` anyway (0 = at once), and served by `serve` until
/// draining starts; then the listener is closed. The connections it
/// accepted run in their own tasks and are not touched. Never bound if
/// draining starts first. A bind error ends it with an error.
pub async fn serve_gated<R, S, F>(addr: String, lc: Arc<Lifecycle>, ready: R, max: Duration, poll: Duration, serve: S) -> anyhow::Result<()>
where
    R: Fn() -> Verdict,
    S: FnOnce(TcpListener) -> F,
    F: Future<Output = anyhow::Result<()>>,
{
    let t0 = Instant::now();
    let why = loop {
        if lc.is_draining() {
            tracing::info!(serve = %addr, "serve listener never opened: draining");
            return Ok(());
        }
        match ready() {
            Verdict::Ready => break "warm",
            _ if max.is_zero() => break "no gate",
            v if t0.elapsed() >= max => {
                tracing::warn!(serve = %addr, verdict = ?v, max_s = max.as_secs_f64(), "serve listener opens without being warm: gate timeout");
                break "gate timeout";
            }
            _ => {}
        }
        tokio::select! {
            _ = tokio::time::sleep(poll) => {}
            _ = lc.drained() => {}
        }
    };
    let l = TcpListener::bind(&addr).await.with_context(|| format!("serve listener {addr}"))?;
    tracing::info!(serve = %addr, why, after_s = t0.elapsed().as_secs(), "serve listener open: feeders here are served by this engine");
    tokio::select! {
        r = serve(l) => r,
        _ = lc.drained() => {
            tracing::info!(serve = %addr, "serve listener closed: draining");
            Ok(())
        }
    }
}

/// The drain's wait: until `active()` (served connections) is 0, or `max`
/// has passed. Returns how many are left.
pub async fn wait_served(active: impl Fn() -> u64, max: Duration, poll: Duration) -> u64 {
    let deadline = tokio::time::Instant::now() + max;
    loop {
        let n = active();
        let now = tokio::time::Instant::now();
        if n == 0 || now >= deadline {
            return n;
        }
        tokio::time::sleep(poll.min(deadline - now)).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser as _;
    use std::sync::atomic::AtomicU64;
    use std::sync::Mutex;
    use tokio::io::AsyncReadExt;
    use tokio::net::TcpStream;

    fn cfg() -> ReadyConfig {
        ReadyConfig { min_uptime_s: 120.0, min_usable: 0.6, feed: true, latch: true }
    }

    fn warm() -> Inputs {
        Inputs { uptime_s: 600.0, snapshot_age_s: Some(0.4), pairs: 1000, pairs_usable: 700, cells_fresh: 2, registry: 2700 }
    }

    /// mlat_shard_proxy.py `check_member`, for an HTTP health URL: 200 is
    /// "ok"; otherwise a body that starts with "draining" (after leading
    /// whitespace) is "draining"; anything else is "fail".
    fn proxy_sees(answer: &Result<String, String>) -> &'static str {
        match answer {
            Ok(_) => "ok",
            Err(body) if body.trim_start().starts_with("draining") => "draining",
            Err(_) => "fail",
        }
    }

    #[test]
    fn warm_is_ok_and_each_criterion_alone_makes_it_warming() {
        let c = cfg();
        assert_eq!(assess(&c, false, &warm()), Verdict::Ready);
        assert_eq!(assess(&c, false, &warm()).http(), Ok("ok\n".into()));
        let cases: Vec<(Inputs, &str)> = vec![
            (Inputs { uptime_s: 30.0, ..warm() }, "warming: up 30 s of 120\n"),
            (Inputs { cells_fresh: 0, ..warm() }, "warming: no feed cell delivering\n"),
            (Inputs { registry: 0, ..warm() }, "warming: receiver registry empty\n"),
            (Inputs { pairs_usable: 599, ..warm() }, "warming: usable clock pairs 0.599 < 0.600 (599 of 1000)\n"),
            (Inputs { pairs: 0, pairs_usable: 0, ..warm() }, "warming: usable clock pairs 0.000 < 0.600 (0 of 0)\n"),
            (Inputs { snapshot_age_s: None, uptime_s: 3.0, ..warm() }, "warming: no engine snapshot yet\n"),
        ];
        for (i, body) in cases {
            let v = assess(&c, false, &i);
            assert_eq!(v.http(), Err(body.to_string()), "{i:?}");
            assert_eq!(proxy_sees(&v.http()), "fail");
        }
        // 600 of 1000 is enough; without --feed the feed is not asked.
        assert!(assess(&c, false, &Inputs { pairs_usable: 600, ..warm() }).is_ready());
        let nofeed = ReadyConfig { feed: false, ..c };
        assert!(assess(&nofeed, false, &Inputs { cells_fresh: 0, registry: 0, ..warm() }).is_ready());
        // min_usable 0 turns the pairs check off, even with no pairs at all.
        let any = ReadyConfig { min_usable: 0.0, ..c };
        assert!(assess(&any, false, &Inputs { pairs: 0, pairs_usable: 0, ..warm() }).is_ready());
    }

    #[test]
    fn a_stuck_engine_is_stalled() {
        let c = cfg();
        let v = assess(&c, false, &Inputs { snapshot_age_s: Some(12.4), ..warm() });
        assert_eq!(v.http(), Err("stalled: engine snapshot 12 s old\n".into()));
        assert_eq!(proxy_sees(&v.http()), "fail");
        let v = assess(&c, false, &Inputs { snapshot_age_s: None, ..warm() });
        assert_eq!(v.http(), Err("stalled: no engine snapshot\n".into()));
        // At the edge it is still fresh.
        assert!(assess(&c, false, &Inputs { snapshot_age_s: Some(STALL_S), ..warm() }).is_ready());
    }

    #[test]
    fn draining_wins_over_everything_and_the_proxy_reads_it_as_draining() {
        let c = cfg();
        for i in [warm(), Inputs { snapshot_age_s: Some(60.0), ..warm() }, Inputs { uptime_s: 1.0, pairs: 0, ..warm() }] {
            let v = assess(&c, true, &i);
            assert_eq!(v, Verdict::Draining);
            assert_eq!(v.http(), Err("draining\n".into()));
            assert_eq!(proxy_sees(&v.http()), "draining");
        }
        assert_eq!(proxy_sees(&Verdict::Ready.http()), "ok");
    }

    /// The state machine as /readyz walks it: warming, ok, then SIGTERM
    /// (`drain`) and draining for good, even with warm inputs.
    #[tokio::test]
    async fn lifecycle_warming_ready_draining() {
        let lc = Lifecycle::default();
        let c = cfg();
        let verdict = |i: &Inputs| assess(&c, lc.is_draining(), i);
        assert!(matches!(verdict(&Inputs { uptime_s: 5.0, ..warm() }), Verdict::Warming(_)));
        assert_eq!(verdict(&warm()), Verdict::Ready);
        assert!(lc.drain(), "the first drain starts it");
        assert!(!lc.drain(), "a second signal does not start it again");
        assert_eq!(verdict(&warm()), Verdict::Draining);
        // drained() resolves, now and for anyone who asks later.
        tokio::time::timeout(Duration::from_secs(1), lc.drained()).await.unwrap();
        assert!(lc.uptime_s() >= 0.0);
    }

    fn metrics(lc: &Lifecycle, c: &ReadyConfig) -> String {
        let mut o = String::new();
        lc.render_metrics(&mut o, c);
        o
    }

    /// The latch (the 2026-11-01 harness's A1): after the first 200, a
    /// takeover's dip of the usable share keeps answering 200; draining and
    /// stalled still answer 503; before the first 200, warming is a 503.
    #[test]
    fn once_ready_a_dip_of_the_usable_share_stays_ready() {
        let lc = Lifecycle::default();
        let c = cfg();
        // 0.394 of the pairs usable, as measured after a takeover.
        let dip = Inputs { pairs: 434, pairs_usable: 171, ..warm() };
        // Before the first 200: warming, and nothing latched.
        let v = lc.verdict(&c, &Inputs { uptime_s: 30.0, ..warm() });
        assert_eq!(v.http(), Err("warming: up 30 s of 120\n".into()));
        assert_eq!(lc.verdict(&c, &dip).http(), Err("warming: usable clock pairs 0.394 < 0.600 (171 of 434)\n".into()));
        assert!(metrics(&lc, &c).contains("mlat_ng_ready_latched 0\n"));
        // The first 200 latches.
        assert_eq!(lc.verdict(&c, &warm()).http(), Ok("ok\n".into()));
        assert!(metrics(&lc, &c).contains("mlat_ng_ready_latched 1\n"));
        // A takeover's dip, and the other warming criteria: still 200.
        for i in [dip.clone(), Inputs { pairs: 0, pairs_usable: 0, ..warm() }, Inputs { cells_fresh: 0, ..warm() }, Inputs { registry: 0, ..warm() }]
        {
            let v = lc.verdict(&c, &i);
            assert_eq!(v.http(), Ok("ok\n".into()), "{i:?}");
            assert_eq!(proxy_sees(&v.http()), "ok");
        }
        assert!(metrics(&lc, &c).contains("mlat_ng_ready_latch_held_total 4\n"));
        // Stalled still takes the 200 away, latched or not, and gives it back.
        let v = lc.verdict(&c, &Inputs { snapshot_age_s: Some(12.0), ..dip.clone() });
        assert_eq!(v.http(), Err("stalled: engine snapshot 12 s old\n".into()));
        assert_eq!(proxy_sees(&v.http()), "fail");
        assert!(lc.verdict(&c, &dip).is_ready());
        // Draining wins over everything, for good.
        lc.drain();
        for i in [warm(), dip.clone(), Inputs { snapshot_age_s: Some(60.0), ..warm() }] {
            let v = lc.verdict(&c, &i);
            assert_eq!(v.http(), Err("draining\n".into()));
            assert_eq!(proxy_sees(&v.http()), "draining");
        }
    }

    #[test]
    fn without_the_latch_a_dip_is_warming_again() {
        let lc = Lifecycle::default();
        let c = ReadyConfig { latch: false, ..cfg() };
        assert!(lc.verdict(&c, &warm()).is_ready());
        let v = lc.verdict(&c, &Inputs { pairs: 434, pairs_usable: 171, ..warm() });
        assert!(matches!(v, Verdict::Warming(_)), "{v:?}");
        let m = metrics(&lc, &c);
        assert!(m.contains("mlat_ng_ready_latched 0\n") && m.contains("mlat_ng_ready_latch_held_total 0\n"), "{m}");
    }

    /// The serve gate goes through the same verdict: its first Ready latches.
    #[tokio::test]
    async fn the_gate_opening_latches_readiness() {
        let addr = free_addr().await;
        let lc = Arc::new(Lifecycle::default());
        let c = cfg();
        let (l2, poll) = (lc.clone(), Duration::from_millis(10));
        let task = tokio::spawn(serve_gated(addr.clone(), lc.clone(), move || l2.verdict(&c, &warm()), Duration::from_secs(60), poll, hold));
        for _ in 0..200 {
            if TcpStream::connect(&addr).await.is_ok() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(lc.verdict(&c, &Inputs { pairs_usable: 0, ..warm() }).is_ready());
        lc.drain();
        task.await.unwrap().unwrap();
    }

    /// A free loopback address (bound, then released).
    async fn free_addr() -> String {
        let l = TcpListener::bind("127.0.0.1:0").await.unwrap();
        l.local_addr().unwrap().to_string()
    }

    /// Accept connections and keep each open until the peer closes it, like
    /// proto::serve does in its own tasks.
    async fn hold(l: TcpListener) -> anyhow::Result<()> {
        hold_counting(l, Arc::new(AtomicU64::new(0))).await
    }

    async fn hold_counting(l: TcpListener, accepted: Arc<AtomicU64>) -> anyhow::Result<()> {
        loop {
            let (mut s, _) = l.accept().await?;
            accepted.fetch_add(1, Ordering::Relaxed);
            tokio::spawn(async move {
                let mut b = [0u8; 64];
                while matches!(s.read(&mut b).await, Ok(n) if n > 0) {}
            });
        }
    }

    async fn refused(addr: &str) -> bool {
        TcpStream::connect(addr).await.is_err()
    }

    /// Real sockets: refused while cold, accepted once warm; on drain new
    /// connects are refused and the connection already there stays open.
    #[tokio::test]
    async fn the_serve_listener_opens_when_warm_and_closes_on_drain() {
        let addr = free_addr().await;
        let lc = Arc::new(Lifecycle::default());
        let warm_now = Arc::new(Mutex::new(false));
        let w = warm_now.clone();
        let ready = move || if *w.lock().unwrap() { Verdict::Ready } else { Verdict::Warming("test".into()) };
        let accepted = Arc::new(AtomicU64::new(0));
        let acc = accepted.clone();
        let serve = move |l| hold_counting(l, acc);
        let task = tokio::spawn(serve_gated(addr.clone(), lc.clone(), ready, Duration::from_secs(60), Duration::from_millis(10), serve));
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert!(refused(&addr).await, "a cold instance must refuse");
        *warm_now.lock().unwrap() = true;
        let mut open = None;
        for _ in 0..200 {
            if let Ok(s) = TcpStream::connect(&addr).await {
                open = Some(s);
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        let mut open = open.expect("the listener never opened once warm");
        // Accepted, not just queued in the backlog (closing the listener
        // resets those; the proxy retries them elsewhere before the reply).
        for _ in 0..200 {
            if accepted.load(Ordering::Relaxed) > 0 {
                break;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert_eq!(accepted.load(Ordering::Relaxed), 1);
        lc.drain();
        tokio::time::timeout(Duration::from_secs(2), task).await.unwrap().unwrap().unwrap();
        assert!(refused(&addr).await, "a draining instance must refuse new connects");
        // The served connection is still there: a read would wait, not EOF.
        let mut b = [0u8; 8];
        assert!(tokio::time::timeout(Duration::from_millis(100), open.read(&mut b)).await.is_err());
    }

    #[tokio::test]
    async fn the_gate_times_out_and_never_opens_after_a_drain() {
        let cold = || Verdict::Warming("never".into());
        // Never warm: opens after `max` anyway.
        let addr = free_addr().await;
        let lc = Arc::new(Lifecycle::default());
        let t0 = Instant::now();
        let task = tokio::spawn(serve_gated(addr.clone(), lc.clone(), cold, Duration::from_millis(150), Duration::from_millis(10), hold));
        let mut opened = false;
        for _ in 0..300 {
            if TcpStream::connect(&addr).await.is_ok() {
                opened = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(opened, "the gate never timed out");
        assert!(t0.elapsed() >= Duration::from_millis(150));
        lc.drain();
        task.await.unwrap().unwrap();
        // A drain before warm-up: never bound.
        let addr = free_addr().await;
        let lc = Arc::new(Lifecycle::default());
        let task = tokio::spawn(serve_gated(addr.clone(), lc.clone(), cold, Duration::from_secs(60), Duration::from_millis(10), hold));
        tokio::time::sleep(Duration::from_millis(30)).await;
        lc.drain();
        tokio::time::timeout(Duration::from_secs(1), task).await.unwrap().unwrap().unwrap();
        assert!(refused(&addr).await);
        // max 0: no gate at all.
        let addr = free_addr().await;
        let lc = Arc::new(Lifecycle::default());
        let task = tokio::spawn(serve_gated(addr.clone(), lc.clone(), cold, Duration::ZERO, Duration::from_millis(10), hold));
        let mut opened = false;
        for _ in 0..100 {
            if TcpStream::connect(&addr).await.is_ok() {
                opened = true;
                break;
            }
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        assert!(opened);
        lc.drain();
        task.await.unwrap().unwrap();
    }

    #[tokio::test]
    async fn a_bind_error_is_an_error() {
        let busy = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = busy.local_addr().unwrap().to_string();
        let lc = Arc::new(Lifecycle::default());
        let r = serve_gated(addr, lc, || Verdict::Ready, Duration::from_secs(1), Duration::from_millis(10), hold).await;
        assert!(r.is_err());
    }

    #[tokio::test]
    async fn the_drain_waits_for_the_served_feeders_or_its_maximum() {
        let poll = Duration::from_millis(20);
        // The proxy moves them: 3, then 1, then none, one poll apart.
        let left = AtomicU64::new(3);
        let t0 = Instant::now();
        let n = wait_served(
            || {
                let n = left.load(Ordering::Relaxed);
                left.store(n.saturating_sub(2), Ordering::Relaxed);
                n
            },
            Duration::from_secs(30),
            poll,
        )
        .await;
        assert_eq!(n, 0);
        let t = t0.elapsed();
        assert!(t >= 2 * poll && t < Duration::from_secs(5), "{t:?}");
        // Nobody served: no wait at all.
        let t0 = Instant::now();
        assert_eq!(wait_served(|| 0, Duration::from_secs(30), poll).await, 0);
        assert!(t0.elapsed() < poll);
        // The proxy cannot move them (no usable standby): the maximum.
        let t0 = Instant::now();
        assert_eq!(wait_served(|| 5, Duration::from_millis(250), poll).await, 5);
        let t = t0.elapsed();
        assert!(t >= Duration::from_millis(250) && t < Duration::from_secs(5), "{t:?}");
    }

    /// main.rs's flags for all this.
    #[test]
    fn flags_defaults_env_names_and_checks() {
        let a = crate::Args::try_parse_from(["mlat-ng"]).unwrap();
        let (c, gate, drain) = crate::redundancy(&a).unwrap();
        assert_eq!((c.min_uptime_s, c.min_usable, c.feed, c.latch), (120.0, 0.6, false, true));
        assert_eq!((gate.as_secs(), drain.as_secs()), (240, 30));
        let a = crate::Args::try_parse_from(["mlat-ng", "--ready-latch", "false"]).unwrap();
        assert!(!crate::redundancy(&a).unwrap().0.latch);
        assert!(crate::Args::try_parse_from(["mlat-ng", "--ready-latch", "maybe"]).is_err());
        let set = ["mlat-ng", "--feed", "a=x:1", "--serve-gate-max-s", "0", "--drain-max-s", "5", "--ready-min-usable", "0.8"];
        let a = crate::Args::try_parse_from(set).unwrap();
        let (c, gate, drain) = crate::redundancy(&a).unwrap();
        assert!(c.feed && c.min_usable == 0.8 && gate.is_zero() && drain.as_secs() == 5);
        for bad in ["--ready-min-usable=1.5", "--drain-max-s=-1", "--serve-gate-max-s=NaN", "--ready-min-uptime-s=-3"] {
            let a = crate::Args::try_parse_from(["mlat-ng", bad]).unwrap();
            assert!(crate::redundancy(&a).is_err(), "{bad:?}");
        }
        // The environment variables (read from the definitions: setting one
        // would race other tests).
        let cmd = <crate::Args as clap::CommandFactory>::command();
        for (id, env) in [
            ("serve_gate_max_s", "MLAT_NG_SERVE_GATE_MAX_S"),
            ("drain_max_s", "MLAT_NG_DRAIN_MAX_S"),
            ("ready_min_uptime_s", "MLAT_NG_READY_MIN_UPTIME_S"),
            ("ready_min_usable", "MLAT_NG_READY_MIN_USABLE"),
            ("ready_latch", "MLAT_NG_READY_LATCH"),
        ] {
            let arg = cmd.get_arguments().find(|a| a.get_id() == id).unwrap();
            assert_eq!(arg.get_env().and_then(|e| e.to_str()), Some(env));
        }
    }

    #[test]
    fn inputs_from_the_engine_and_the_feed() {
        let st = Stats::default();
        let i = Inputs::read(5.0, &st, None);
        assert_eq!(i.snapshot_age_s, None);
        assert_eq!((i.cells_fresh, i.registry), (0, 0));
        let now_ms = (unix_now() * 1000.0) as u64;
        st.snapshot_ms.store(now_ms - 2000, Ordering::Relaxed);
        {
            let mut s = st.snapshot.lock().unwrap();
            s.pairs = 10;
            s.pairs_usable = 7;
        }
        let cfg = crate::relay::FeedConfig {
            cells: vec![("a".into(), "x:1".into()), ("b".into(), "y:1".into())],
            receivers_url: String::new(),
            adapter: Default::default(),
        };
        let fs = FeedStats::new(&cfg);
        fs.cells[0].1.last_frame_ms.store(now_ms - 1000, Ordering::Relaxed);
        fs.cells[1].1.last_frame_ms.store(now_ms - 60_000, Ordering::Relaxed);
        fs.registry_size.store(42, Ordering::Relaxed);
        let i = Inputs::read(5.0, &st, Some(&fs));
        let age = i.snapshot_age_s.unwrap();
        assert!((1.9..5.0).contains(&age), "{age}");
        assert_eq!((i.pairs, i.pairs_usable, i.cells_fresh, i.registry), (10, 7, 1, 42));
    }
}
