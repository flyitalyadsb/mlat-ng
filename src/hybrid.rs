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
//!
//! A served feeder (M2 in docs/PLAN.md, "Redundancy") reaches a serving
//! instance twice: on `--serve-listen`, and as the copy mlat-shadow-hub tees
//! to `--listen` so that the standby instance stays warm. Both are
//! mlat-client connections with one key; the listener tells them apart
//! (`ReceiverInfo::serve`, set only off a shadow listener: `--listen` must
//! run with `--shadow`).
//! The served connection owns the key (Serve > shadow client > feed), and
//! everything from its shadow copy is dropped: the served stream carries
//! the same lines and more (the hub forwards them or drops some), so there
//! is nothing to merge. When the served connection goes (the proxy moved the
//! feeder), the shadow copy takes over and the feeder stays a receiver here.

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
    /// Keys owned by a served connection, and of those the ones whose
    /// shadow copy is connected too.
    pub owner_serve: AtomicU64,
    pub served_with_shadow: AtomicU64,
    /// Events from the shadow copy of a served feeder, dropped.
    pub shadow_of_served_dropped: AtomicU64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Src {
    Client,
    Feed,
    /// An mlat-client on `--serve-listen`. Both listeners feed the router as
    /// `Client`; on_connect tells this one by its handshake.
    Serve,
}

impl Src {
    /// Who owns a key that several inputs have: the served connection (it is
    /// asked and answered), the shadow client, the feed.
    fn rank(self) -> u8 {
        match self {
            Src::Feed => 0,
            Src::Client => 1,
            Src::Serve => 2,
        }
    }
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
    serve: Option<Side>,
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
            Src::Serve => self.serve.as_ref(),
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
            Event::Connect { conn, info, tx } => {
                self.on_connect(src, Side { conn, info, tx }, out);
                self.gauges();
            }
            Event::Disconnect { conn } => {
                self.on_disconnect(conn, out);
                self.gauges();
            }
            // About an aircraft, not a receiver: nothing to route.
            ev @ Event::GeoBaro { .. } => out.push(ev),
            ev => self.on_data(ev, out),
        }
    }

    fn on_connect(&mut self, src: Src, side: Side, out: &mut Vec<Event>) {
        let src = if src == Src::Client && side.info.serve.is_some() { Src::Serve } else { src };
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
            (Some((fc, o)), _) if src.rank() > o.rank() => {
                // The client takes over (the served one from its shadow
                // copy too): it can be sent requests and results.
                out.push(Event::Disconnect { conn: fc });
                slot.owner = Some((side.conn, src));
                slot.forget();
                out.push(connect(&side));
                self.stats.handovers.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
        match src {
            Src::Client => slot.client = Some(side),
            Src::Feed => slot.feed = Some(side),
            Src::Serve => slot.serve = Some(side),
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
            Src::Serve => slot.serve = None,
        }
        if slot.owner.map(|o| o.0) == Some(conn) {
            out.push(Event::Disconnect { conn });
            slot.owner = None;
            slot.forget();
            // The best input left takes over.
            let next = [Src::Serve, Src::Client, Src::Feed].into_iter().find_map(|o| slot.side(o).map(|s| (o, s.conn, connect(s))));
            if let Some((other, c, ev)) = next {
                slot.owner = Some((c, other));
                out.push(ev);
                self.stats.handovers.fetch_add(1, Ordering::Relaxed);
            }
        }
        if slot.client.is_none() && slot.feed.is_none() && slot.serve.is_none() {
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
        let Some(&(key, src)) = self.conn_key.get(&conn) else {
            out.push(ev);
            return;
        };
        let Some(slot) = self.slots.get_mut(&key) else { return };
        let Some((owner, osrc)) = slot.owner else { return };
        let from_owner = owner == conn;
        let st = &self.stats;
        // The shadow copy of a served feeder: all of it is in the served
        // stream already.
        if osrc == Src::Serve && src == Src::Client {
            st.shadow_of_served_dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
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
        let (mut merged, mut oc, mut of, mut os, mut sw) = (0, 0, 0, 0, 0);
        for s in self.slots.values() {
            if (s.client.is_some() || s.serve.is_some()) && s.feed.is_some() {
                merged += 1;
            }
            match s.owner {
                Some((_, Src::Client)) => oc += 1,
                Some((_, Src::Feed)) => of += 1,
                Some((_, Src::Serve)) => {
                    os += 1;
                    if s.client.is_some() {
                        sw += 1;
                    }
                }
                None => {}
            }
        }
        self.stats.merged.store(merged, Ordering::Relaxed);
        self.stats.owner_client.store(oc, Ordering::Relaxed);
        self.stats.owner_feed.store(of, Ordering::Relaxed);
        self.stats.owner_serve.store(os, Ordering::Relaxed);
        self.stats.served_with_shadow.store(sw, Ordering::Relaxed);
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
        ("hybrid_owner_serve", l(&s.owner_serve)),
        ("hybrid_served_with_shadow", l(&s.served_with_shadow)),
        ("hybrid_shadow_of_served_dropped_total", l(&s.shadow_of_served_dropped)),
    ] {
        let _ = writeln!(o, "mlat_ng_{m} {v}");
    }
}

#[cfg(test)]
mod tests;
