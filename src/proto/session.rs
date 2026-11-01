//! Served sessions, timed (M5 in docs/PLAN.md, "Redundancy"): how long a
//! feeder connected to `--serve-listen` waits for its first `start_sending`
//! and its first result line. In a handover between two instances every
//! served feeder reconnects and starts a new session on the instance taking
//! over, so with the proxy's close times these give every served feeder's
//! gap, not just a test feeder's.
//!
//! A session starts when its Connect has gone to the engine and ends with its
//! connection. "Sent" means queued on the connection's writer: zlib2 output
//! leaves up to a second later (writer.rs), and a line the backlog cap drops
//! does not count. Clients that did not ask for results never get one, so
//! they end in `closed_no_result`.

use std::fmt::Write as _;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use super::Counters;
use crate::types::{ConnId, ToClient};

/// Upper bounds (s) of the histogram buckets: a takeover is expected to cost
/// 4-35 s by the redundancy design's estimate, a cold start minutes.
pub const BUCKETS: [f64; 14] = [0.5, 1.0, 2.0, 5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0, 120.0, 180.0, 300.0];

/// A Prometheus histogram on atomics: a count per bucket (not cumulative;
/// the last one is +Inf) and the sum.
#[derive(Debug, Default)]
pub struct Histogram {
    counts: [AtomicU64; BUCKETS.len() + 1],
    sum_us: AtomicU64,
}

impl Histogram {
    pub fn observe(&self, secs: f64) {
        let s = secs.max(0.0);
        let i = BUCKETS.iter().position(|&b| s <= b).unwrap_or(BUCKETS.len());
        self.counts[i].fetch_add(1, Ordering::Relaxed);
        self.sum_us.fetch_add((s * 1e6) as u64, Ordering::Relaxed);
    }

    /// `# TYPE`, the cumulative buckets, `_sum` and `_count`.
    pub fn render(&self, o: &mut String, name: &str) {
        let _ = writeln!(o, "# TYPE {name} histogram");
        let mut n = 0;
        for (i, b) in BUCKETS.iter().enumerate() {
            n += self.counts[i].load(Ordering::Relaxed);
            let _ = writeln!(o, "{name}_bucket{{le=\"{b}\"}} {n}");
        }
        n += self.counts[BUCKETS.len()].load(Ordering::Relaxed);
        let _ = writeln!(o, "{name}_bucket{{le=\"+Inf\"}} {n}");
        let _ = writeln!(o, "{name}_sum {}", self.sum_us.load(Ordering::Relaxed) as f64 / 1e6);
        let _ = writeln!(o, "{name}_count {n}");
    }
}

/// Every served session of the process (part of `Counters`).
#[derive(Debug, Default)]
pub struct ServeSessions {
    pub opened: AtomicU64,
    pub closed: AtomicU64,
    /// Sessions that ended before their first result line.
    pub closed_no_result: AtomicU64,
    /// Seconds from the Connect to the first start_sending / result line.
    pub first_start: Histogram,
    pub first_result: Histogram,
}

impl ServeSessions {
    /// Served connections open right now (what a drain waits on).
    pub fn active(&self) -> u64 {
        self.opened.load(Ordering::Relaxed).saturating_sub(self.closed.load(Ordering::Relaxed))
    }

    pub fn render(&self, o: &mut String) {
        let l = |a: &AtomicU64| a.load(Ordering::Relaxed);
        for (name, kind, v) in [
            ("serve_sessions_opened_total", "counter", l(&self.opened)),
            ("serve_sessions_closed_total", "counter", l(&self.closed)),
            ("serve_sessions_closed_no_result_total", "counter", l(&self.closed_no_result)),
            ("serve_sessions_active", "gauge", self.active()),
        ] {
            let _ = writeln!(o, "# TYPE mlat_ng_{name} {kind}\nmlat_ng_{name} {v}");
        }
        self.first_start.render(o, "mlat_ng_serve_first_start_seconds");
        self.first_result.render(o, "mlat_ng_serve_first_result_seconds");
    }
}

/// What a line queued for a served client was, for its session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Sent {
    Start,
    Result,
    Other,
}

impl Sent {
    pub(super) fn of(m: &ToClient) -> Sent {
        match m {
            ToClient::StartSending(_) => Sent::Start,
            ToClient::ResultLine(_) => Sent::Result,
            _ => Sent::Other,
        }
    }
}

/// One served connection. The writer task holds it, so it is dropped (and
/// counted closed) however the connection ends.
pub(super) struct Session {
    counters: Arc<Counters>,
    conn: ConnId,
    user: String,
    opened: Instant,
    first_start: Option<f64>,
    first_result: Option<f64>,
}

impl Session {
    pub(super) fn open(counters: Arc<Counters>, conn: ConnId, user: &str) -> Session {
        counters.serve.opened.fetch_add(1, Ordering::Relaxed);
        Session { counters, conn, user: user.to_string(), opened: Instant::now(), first_start: None, first_result: None }
    }

    /// A line was queued for the client.
    pub(super) fn sent(&mut self, what: Sent) {
        let s = &self.counters.serve;
        match what {
            Sent::Start if self.first_start.is_none() => {
                let t = self.opened.elapsed().as_secs_f64();
                self.first_start = Some(t);
                s.first_start.observe(t);
            }
            Sent::Result if self.first_result.is_none() => {
                let t = self.opened.elapsed().as_secs_f64();
                self.first_result = Some(t);
                s.first_result.observe(t);
                let ms = |x: f64| (x * 1000.0).round() as i64;
                tracing::info!(
                    conn = self.conn,
                    user = %self.user,
                    first_start_ms = self.first_start.map_or(-1, ms),
                    first_result_ms = ms(t),
                    "served: first result"
                );
            }
            _ => {}
        }
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        let s = &self.counters.serve;
        s.closed.fetch_add(1, Ordering::Relaxed);
        if self.first_result.is_none() {
            s.closed_no_result.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn histogram_buckets_are_cumulative() {
        let h = Histogram::default();
        for s in [0.2, 0.5, 3.0, 12.0, 12.0, 1000.0] {
            h.observe(s);
        }
        let mut o = String::new();
        h.render(&mut o, "x");
        assert!(o.starts_with("# TYPE x histogram\n"), "{o}");
        for want in [
            "x_bucket{le=\"0.5\"} 2\n",
            "x_bucket{le=\"1\"} 2\n",
            "x_bucket{le=\"5\"} 3\n",
            "x_bucket{le=\"10\"} 3\n",
            "x_bucket{le=\"15\"} 5\n",
            "x_bucket{le=\"300\"} 5\n",
            "x_bucket{le=\"+Inf\"} 6\n",
            "x_sum 1027.7\n",
            "x_count 6\n",
        ] {
            assert!(o.contains(want), "missing {want:?} in\n{o}");
        }
        // One line per bucket, +Inf, sum and count, after the TYPE line.
        assert_eq!(o.lines().count(), 1 + BUCKETS.len() + 3);
    }

    #[test]
    fn a_session_counts_its_first_start_and_result_once_and_its_close() {
        let c = Arc::new(Counters::default());
        let mut s = Session::open(c.clone(), 7, "feeder");
        assert_eq!(c.serve.active(), 1);
        s.sent(Sent::Other);
        s.sent(Sent::Start);
        s.sent(Sent::Start);
        s.sent(Sent::Result);
        s.sent(Sent::Result);
        drop(s);
        let mut o = String::new();
        c.serve.render(&mut o);
        for want in [
            "mlat_ng_serve_sessions_opened_total 1\n",
            "mlat_ng_serve_sessions_closed_total 1\n",
            "mlat_ng_serve_sessions_closed_no_result_total 0\n",
            "mlat_ng_serve_sessions_active 0\n",
            "mlat_ng_serve_first_start_seconds_count 1\n",
            "mlat_ng_serve_first_result_seconds_count 1\n",
            "mlat_ng_serve_first_result_seconds_bucket{le=\"0.5\"} 1\n",
        ] {
            assert!(o.contains(want), "missing {want:?} in\n{o}");
        }
        // A session that ends before any result.
        let s = Session::open(c.clone(), 8, "feeder");
        drop(s);
        assert_eq!(c.serve.closed_no_result.load(Ordering::Relaxed), 1);
        assert_eq!(c.serve.active(), 0);
    }

    #[test]
    fn sent_kinds() {
        assert_eq!(Sent::of(&ToClient::StartSending(vec![1])), Sent::Start);
        assert_eq!(Sent::of(&ToClient::ResultLine(String::new())), Sent::Result);
        assert_eq!(Sent::of(&ToClient::StopSending(vec![1])), Sent::Other);
        assert_eq!(Sent::of(&ToClient::StatsLine(String::new())), Sent::Other);
    }
}
