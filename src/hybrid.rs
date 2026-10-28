//! Hybrid input: one engine receiver per feeder, fed by both its mlat-client
//! connection and its slice of the receiver-tagged Beast feed.
//!
//! A feeder's mlat-client reads the timestamps of its own readsb, the same
//! 12 MHz counter readsb forwards on the Beast feed, so the two inputs are one
//! clock and can be merged into one receiver. They are matched by key (the
//! first 64 bits of the UUID, as readsb derives the feed's receiver id).
//!
//! - The owner of a key is its mlat-client connection when there is one (it
//!   is the one that can be asked for data and sent results), else its feed
//!   receiver. Only the owner's Connect reaches the engine.
//! - Timing data from the other input (Sync, Mlat) is remapped to the owner's
//!   connection id. A copy both inputs deliver (same timestamp, same message)
//!   reaches the engine once. ClockReset counts only from the owner.
//! - Seen/Lost/RateReport come from the owner's input only: two inputs
//!   reporting the same aircraft set would otherwise undo each other's Lost.
//! - When the owner disconnects and the other input is still there, the other
//!   one takes over with its own Connect (the engine re-syncs its clock).
//! - Connections without a usable UUID, and a second mlat-client with a key
//!   already taken by another client (a cloned UUID), pass through untouched.

use crate::hashing::{HashMap, HashSet};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

use tokio::sync::mpsc;

use crate::engine::fxhash;
use crate::relay::registry::key_of_uuid;
use crate::types::{ConnId, Event, ReceiverInfo, ToClient};

/// Recent copies kept per receiver for de-duplication, in counter ticks of a
/// 12 MHz clock (other clocks keep proportionally less time, still seconds).
const DEDUP_SPAN_TICKS: u64 = 12_000_000 * 30;
const DEDUP_MAX: usize = 20_000;

#[derive(Default)]
pub struct RouterStats {
    /// Keys with both inputs present right now.
    pub merged: AtomicU64,
    pub owner_client: AtomicU64,
    pub owner_feed: AtomicU64,
    pub passthrough_conns: AtomicU64,
    /// Data events from the non-owner input forwarded under the owner's id.
    pub remapped: AtomicU64,
    pub dup_mlat: AtomicU64,
    pub dup_sync: AtomicU64,
    /// Seen/Lost/RateReport from the non-owner input, not forwarded.
    pub secondary_dropped: AtomicU64,
    pub handovers: AtomicU64,
    /// Events the engine queue could not take.
    pub engine_full: AtomicU64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Src {
    Client,
    Feed,
}

struct Side {
    conn: ConnId,
    info: ReceiverInfo,
    tx: mpsc::Sender<ToClient>,
}

#[derive(Default)]
struct Slot {
    client: Option<Side>,
    feed: Option<Side>,
    owner: Option<(ConnId, Src)>,
    seen: HashSet<(u64, u64)>,
    order: VecDeque<(u64, (u64, u64))>,
    newest: u64,
}

impl Slot {
    fn side(&self, s: Src) -> Option<&Side> {
        match s {
            Src::Client => self.client.as_ref(),
            Src::Feed => self.feed.as_ref(),
        }
    }

    /// True the first time this copy is offered.
    fn first(&mut self, t: u64, h: u64) -> bool {
        if !self.seen.insert((t, h)) {
            return false;
        }
        self.newest = self.newest.max(t);
        self.order.push_back((t, (t, h)));
        while let Some(&(ot, k)) = self.order.front() {
            if self.order.len() > DEDUP_MAX || ot + DEDUP_SPAN_TICKS < self.newest {
                self.order.pop_front();
                self.seen.remove(&k);
            } else {
                break;
            }
        }
        true
    }

    fn forget(&mut self) {
        self.seen.clear();
        self.order.clear();
        self.newest = 0;
    }
}

pub struct Router {
    slots: HashMap<u64, Slot>,
    conn_key: HashMap<ConnId, (u64, Src)>,
    passthrough: HashSet<ConnId>,
    pub stats: Arc<RouterStats>,
}

fn connect(s: &Side) -> Event {
    Event::Connect { conn: s.conn, info: s.info.clone(), tx: s.tx.clone() }
}

impl Router {
    pub fn new(stats: Arc<RouterStats>) -> Router {
        Router { slots: HashMap::default(), conn_key: HashMap::default(), passthrough: HashSet::default(), stats }
    }

    pub fn on_event(&mut self, src: Src, ev: Event, out: &mut Vec<Event>) {
        match ev {
            Event::Connect { conn, info, tx } => self.on_connect(src, Side { conn, info, tx }, out),
            Event::Disconnect { conn } => self.on_disconnect(conn, out),
            // About an aircraft, not a receiver: nothing to route.
            ev @ Event::GeoBaro { .. } => out.push(ev),
            ev => self.on_data(ev, out),
        }
        self.gauges();
    }

    fn on_connect(&mut self, src: Src, side: Side, out: &mut Vec<Event>) {
        let Some(key) = side.info.uuid.as_deref().and_then(key_of_uuid) else {
            self.passthrough.insert(side.conn);
            out.push(connect(&side));
            return;
        };
        let slot = self.slots.entry(key).or_default();
        if slot.side(src).is_some() {
            // A second client (or feed receiver) under a key already taken:
            // a cloned UUID. Keep it a receiver of its own.
            self.passthrough.insert(side.conn);
            out.push(connect(&side));
            return;
        }
        self.conn_key.insert(side.conn, (key, src));
        match (slot.owner, src) {
            (None, _) => {
                slot.owner = Some((side.conn, src));
                out.push(connect(&side));
            }
            (Some((fc, Src::Feed)), Src::Client) => {
                // The client takes over: it can be sent requests and results.
                out.push(Event::Disconnect { conn: fc });
                slot.owner = Some((side.conn, Src::Client));
                slot.forget();
                out.push(connect(&side));
                self.stats.handovers.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
        match src {
            Src::Client => slot.client = Some(side),
            Src::Feed => slot.feed = Some(side),
        }
    }

    fn on_disconnect(&mut self, conn: ConnId, out: &mut Vec<Event>) {
        if self.passthrough.remove(&conn) {
            out.push(Event::Disconnect { conn });
            return;
        }
        let Some((key, src)) = self.conn_key.remove(&conn) else {
            out.push(Event::Disconnect { conn });
            return;
        };
        let Some(slot) = self.slots.get_mut(&key) else { return };
        match src {
            Src::Client => slot.client = None,
            Src::Feed => slot.feed = None,
        }
        if slot.owner.map(|o| o.0) == Some(conn) {
            out.push(Event::Disconnect { conn });
            slot.owner = None;
            slot.forget();
            let other = match src {
                Src::Client => Src::Feed,
                Src::Feed => Src::Client,
            };
            if let Some((c, ev)) = slot.side(other).map(|s| (s.conn, connect(s))) {
                slot.owner = Some((c, other));
                out.push(ev);
                self.stats.handovers.fetch_add(1, Ordering::Relaxed);
            }
        }
        if slot.client.is_none() && slot.feed.is_none() {
            self.slots.remove(&key);
        }
    }

    fn on_data(&mut self, ev: Event, out: &mut Vec<Event>) {
        let conn = match &ev {
            Event::Sync { conn, .. }
            | Event::Mlat { conn, .. }
            | Event::Seen { conn, .. }
            | Event::Lost { conn, .. }
            | Event::RateReport { conn, .. }
            | Event::ClockReset { conn, .. }
            | Event::PositionUpdate { conn, .. } => *conn,
            _ => return,
        };
        let Some(&(key, _)) = self.conn_key.get(&conn) else {
            out.push(ev);
            return;
        };
        let Some(slot) = self.slots.get_mut(&key) else { return };
        let Some((owner, _)) = slot.owner else { return };
        let from_owner = owner == conn;
        let st = &self.stats;
        match ev {
            Event::Mlat { t, msg, rx, .. } => {
                if !slot.first(t as u64, fxhash(&msg)) {
                    st.dup_mlat.fetch_add(1, Ordering::Relaxed);
                    return;
                }
                if !from_owner {
                    st.remapped.fetch_add(1, Ordering::Relaxed);
                }
                out.push(Event::Mlat { conn: owner, t, msg, rx });
            }
            Event::Sync { et, ot, em, om, .. } => {
                // A pair is both halves: consecutive odd messages pair with
                // the same even one and are different sync points.
                let h = fxhash(&em) ^ fxhash(&om).rotate_left(17) ^ (ot as u64).rotate_left(33) ^ 0x5c5c_5c5c;
                if !slot.first(et as u64, h) {
                    st.dup_sync.fetch_add(1, Ordering::Relaxed);
                    return;
                }
                if !from_owner {
                    st.remapped.fetch_add(1, Ordering::Relaxed);
                }
                out.push(Event::Sync { conn: owner, et, ot, em, om });
            }
            // A reset from the owner restarts the clock; one seen only by the
            // other input (a feed reconnect replaying held-back frames, say) is
            // not the owner's to act on, but its old copies must not match.
            Event::ClockReset { reason, .. } => {
                slot.forget();
                if from_owner {
                    out.push(Event::ClockReset { conn: owner, reason });
                } else {
                    st.secondary_dropped.fetch_add(1, Ordering::Relaxed);
                }
            }
            Event::PositionUpdate { lat, lon, alt, .. } => out.push(Event::PositionUpdate { conn: owner, lat, lon, alt }),
            ev if from_owner => out.push(ev),
            _ => {
                st.secondary_dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    fn gauges(&self) {
        let (mut merged, mut oc, mut of) = (0, 0, 0);
        for s in self.slots.values() {
            if s.client.is_some() && s.feed.is_some() {
                merged += 1;
            }
            match s.owner {
                Some((_, Src::Client)) => oc += 1,
                Some((_, Src::Feed)) => of += 1,
                None => {}
            }
        }
        self.stats.merged.store(merged, Ordering::Relaxed);
        self.stats.owner_client.store(oc, Ordering::Relaxed);
        self.stats.owner_feed.store(of, Ordering::Relaxed);
        self.stats.passthrough_conns.store(self.passthrough.len() as u64, Ordering::Relaxed);
    }
}

/// Run the router between the two inputs and the engine. Connect/Disconnect
/// wait for queue space (they must not be lost); data uses try_send, as the
/// connections themselves do.
pub async fn run(
    mut clients: mpsc::Receiver<Event>,
    mut feed: mpsc::Receiver<Event>,
    engine: mpsc::Sender<Event>,
    stats: Arc<RouterStats>,
) {
    let mut r = Router::new(stats.clone());
    let mut out = Vec::with_capacity(64);
    let (mut clients_open, mut feed_open) = (true, true);
    while clients_open || feed_open {
        tokio::select! {
            ev = clients.recv(), if clients_open => match ev {
                Some(ev) => r.on_event(Src::Client, ev, &mut out),
                None => clients_open = false,
            },
            ev = feed.recv(), if feed_open => match ev {
                Some(ev) => r.on_event(Src::Feed, ev, &mut out),
                None => feed_open = false,
            },
        }
        for ev in out.drain(..) {
            let control = matches!(ev, Event::Connect { .. } | Event::Disconnect { .. });
            if control {
                if engine.send(ev).await.is_err() {
                    return;
                }
            } else if engine.try_send(ev).is_err() {
                stats.engine_full.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

pub fn render_metrics(o: &mut String, s: &RouterStats) {
    use std::fmt::Write as _;
    let l = |a: &AtomicU64| a.load(Ordering::Relaxed);
    for (m, v) in [
        ("hybrid_receivers_merged", l(&s.merged)),
        ("hybrid_owner_client", l(&s.owner_client)),
        ("hybrid_owner_feed", l(&s.owner_feed)),
        ("hybrid_passthrough_conns", l(&s.passthrough_conns)),
        ("hybrid_remapped_total", l(&s.remapped)),
        ("hybrid_dup_mlat_total", l(&s.dup_mlat)),
        ("hybrid_dup_sync_total", l(&s.dup_sync)),
        ("hybrid_secondary_dropped_total", l(&s.secondary_dropped)),
        ("hybrid_handovers_total", l(&s.handovers)),
        ("hybrid_engine_full_total", l(&s.engine_full)),
    ] {
        let _ = writeln!(o, "mlat_ng_{m} {v}");
    }
}

#[cfg(test)]
mod tests;
