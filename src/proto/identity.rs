//! Who a served connection is: PR #14's duplicate checks, on the serve
//! listener only. The rules are flyitalyadsb/mlat-server's at f1ff6c5
//! (`mlat/identity.py`, applied in `jsonclient.process_handshake`), the
//! version the production shards run.
//!
//! A new served connection whose user name is already connected here is
//! either the same receiver coming back or a different receiver that picked
//! the same name:
//! - **Same receiver, old connection still sending** (a line or frame in the
//!   last 45 s): the new one is refused, with `reconnect_in` set to when the
//!   old one would stop counting as live, plus 15 s (+-10 %). Two live clients
//!   of one receiver (both ports 30100 and 31090, or a second host) would
//!   otherwise be two receivers here at one spot, with zero-baseline clock
//!   pairs and every result sent twice. The old connection is never touched.
//! - **Same receiver, old connection quiet**: the receiver came back over a
//!   new connection (the old one is half-open). The new one takes over. The
//!   old one is closed first, so the engine sees its Disconnect before the
//!   new Connect (hybrid.rs would otherwise keep the newcomer as a second
//!   receiver under the same key).
//! - **Another receiver** (a different uuid; with no uuid on either side, more
//!   than 100 m away): the new one is renamed `<name>_NNN`, with a note in
//!   its motd. `/clients.json` is keyed by name.
//!
//! "Same receiver": with a uuid on either side, the uuids decide; with none on
//! both sides, the position does. A placeholder uuid (handshake.rs) is none.
//!
//! Shadow listeners are not checked. Their connections are the copies
//! mlat-shadow-hub tees from mlat-proxy, and the primary (a serving instance,
//! or an mlat-server shard) has already refused a duplicate's client. That
//! closes the client's proxy session and its copy with it.
//!
//! The registry is per process, like mlat-server's per-shard
//! `coordinator.usernames`: a feeder that mlat-proxy moves to the other
//! member of the pair is a new connection there.

use std::collections::hash_map::Entry as MapEntry;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Instant;

use tokio::sync::Notify;

use crate::hashing::HashMap;
use crate::types::ConnId;

/// How recently a connection must have sent something to count as live
/// (identity.py `LIVE_S`). mlat-client sends sync and mlat messages many
/// times a second while it sees aircraft, and answers the 30 s heartbeat.
pub const LIVE_S: f64 = 45.0;
/// A refused duplicate retries this long after the old connection would stop
/// counting as live (`RETRY_MARGIN_S`).
pub const RETRY_MARGIN_S: f64 = 15.0;
/// Two receivers with one name and no uuid closer than this are one receiver
/// connected twice (`SAME_PLACE_M`).
pub const SAME_PLACE_M: f64 = 100.0;

/// The refusal mlat-server sends (`DuplicateReceiver`, verbatim).
fn duplicate_reason(user: &str) -> String {
    format!(
        "Receiver {user} is already connected to this server and still sending. Are two MLAT clients of this \
         receiver pointed here (e.g. both ports 30100 and 31090, or a second host)? Keep one."
    )
}

/// The motd note of a renamed receiver (mlat-server's, verbatim).
fn renamed_note(requested: &str, user: &str) -> String {
    format!("The name {requested} is taken by another receiver: you are connected as {user}. Set a name of your own (MLAT_USER).")
}

fn epoch() -> Instant {
    static E: OnceLock<Instant> = OnceLock::new();
    *E.get_or_init(Instant::now)
}

fn now_ms() -> u64 {
    epoch().elapsed().as_millis() as u64
}

/// One connection's `_last_message_time`, shared with the registry. The
/// connection touches it on every complete line or frame (conn.rs).
#[derive(Debug, Default)]
pub(crate) struct Liveness(AtomicU64);

impl Liveness {
    pub fn touch(&self) {
        self.0.store(now_ms(), Ordering::Relaxed);
    }

    fn age_s(&self, now: u64) -> f64 {
        now.saturating_sub(self.0.load(Ordering::Relaxed)) as f64 / 1000.0
    }
}

/// Great-circle distance in metres between two (lat, lon, alt), as
/// identity.py computes it.
pub(crate) fn distance_m(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let (lat1, lon1) = (a.0.to_radians(), a.1.to_radians());
    let (lat2, lon2) = (b.0.to_radians(), b.1.to_radians());
    let h = ((lat2 - lat1) / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * ((lon2 - lon1) / 2.0).sin().powi(2);
    2.0 * 6_371_000.0 * h.sqrt().min(1.0).asin()
}

/// Whether a new connection with an already-connected name is that same
/// receiver (`same_receiver`).
pub(crate) fn same_receiver(old_uuid: Option<&str>, old: (f64, f64, f64), uuid: Option<&str>, new: (f64, f64, f64)) -> bool {
    if uuid.is_some() || old_uuid.is_some() {
        return uuid.is_some() && uuid == old_uuid;
    }
    distance_m(old, new) <= SAME_PLACE_M
}

/// SplitMix64: the few random draws here (rename suffix, retry jitter) need
/// no more, and no dependency.
fn mix(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9e37_79b9_7f4a_7c15);
    x = (x ^ (x >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    x ^ (x >> 31)
}

/// mlat-server's `util.fuzzy`: uniform in +-10 %, rounded to whole seconds.
pub(crate) fn fuzzy(t: f64, draw: u64) -> f64 {
    let u = (draw >> 11) as f64 / (1u64 << 53) as f64;
    (t * (0.9 + 0.2 * u)).round()
}

#[derive(Debug)]
struct Entry {
    conn: ConnId,
    uuid: Option<String>,
    llh: (f64, f64, f64),
    live: Arc<Liveness>,
    /// Tells the connection to close (a quiet duplicate is taking over).
    kick: Arc<Notify>,
    /// Signalled when the connection has released its name, after its
    /// Disconnect went to the engine.
    gone: Arc<Notify>,
}

/// The names connected on one serve listener (mlat-server's
/// `coordinator.usernames`).
#[derive(Debug)]
pub struct Registry {
    names: Mutex<HashMap<String, Entry>>,
    live_s: f64,
    margin_s: f64,
}

impl Default for Registry {
    fn default() -> Self {
        Registry::with_windows(LIVE_S, RETRY_MARGIN_S)
    }
}

/// What to do with a new served connection.
pub(crate) enum Admission {
    /// Connected under `ticket.user()`; `note` when it was renamed.
    Admit { ticket: Ticket, note: Option<String> },
    /// The same receiver, over a new connection, while the old one is quiet.
    /// Tell the old one to close (`kick`), wait for `gone`, then ask again.
    Replace { kick: Arc<Notify>, gone: Arc<Notify> },
    /// A second live client of a connected receiver.
    Refuse { reason: String, retry_s: f64 },
}

impl Registry {
    /// Shorter windows are for tests.
    pub fn with_windows(live_s: f64, margin_s: f64) -> Registry {
        Registry { names: Mutex::new(HashMap::default()), live_s, margin_s }
    }

    /// The handshake's duplicate checks for connection `conn` asking for
    /// `user` (already sanitized, as mlat-server checks it).
    pub(crate) fn admit(self: &Arc<Self>, conn: ConnId, user: &str, uuid: Option<&str>, llh: (f64, f64, f64)) -> Admission {
        let now = now_ms();
        let mut draw = mix(conn ^ now.rotate_left(32));
        let mut names = self.names.lock().unwrap();
        let mut name = user.to_string();
        let mut note = None;
        if let Some(e) = names.get(&name) {
            if same_receiver(e.uuid.as_deref(), e.llh, uuid, llh) {
                let age = e.live.age_s(now);
                if age < self.live_s {
                    let retry_s = fuzzy((self.live_s - age).max(0.0) + self.margin_s, draw).max(1.0);
                    return Admission::Refuse { reason: duplicate_reason(&name), retry_s };
                }
                return Admission::Replace { kick: e.kick.clone(), gone: e.gone.clone() };
            }
            // Another receiver: suffixes accumulate while still taken, as in
            // mlat-server (`user = user + '_' + str(random.randrange(100,999))`).
            let mut tries = 1000;
            while tries > 0 && names.contains_key(&name) {
                tries -= 1;
                draw = mix(draw);
                name = format!("{name}_{}", 100 + draw % 899);
            }
            if names.contains_key(&name) {
                // Unreachable in practice (1000 taken suffixes in a row).
                return Admission::Refuse { reason: duplicate_reason(user), retry_s: fuzzy(900.0, draw) };
            }
            note = Some(renamed_note(user, &name));
        }
        let live = Arc::new(Liveness::default());
        live.touch();
        let kick = Arc::new(Notify::new());
        let entry = Entry { conn, uuid: uuid.map(str::to_string), llh, live: live.clone(), kick: kick.clone(), gone: Arc::new(Notify::new()) };
        match names.entry(name.clone()) {
            MapEntry::Vacant(v) => {
                v.insert(entry);
            }
            MapEntry::Occupied(_) => unreachable!("checked under the same lock"),
        }
        Admission::Admit { ticket: Ticket { reg: self.clone(), user: name, conn, live, kick }, note }
    }

    /// The connection `conn` is gone: free its name (only if it still holds
    /// it) and wake whoever waits to take it over.
    fn release(&self, user: &str, conn: ConnId) {
        let mut names = self.names.lock().unwrap();
        if names.get(user).is_some_and(|e| e.conn == conn) {
            if let Some(e) = names.remove(user) {
                e.gone.notify_waiters();
            }
        }
    }

    #[cfg(test)]
    pub(crate) fn len(&self) -> usize {
        self.names.lock().unwrap().len()
    }
}

/// A served connection's hold on its name. Dropping it releases the name, so
/// a connection keeps it until after its Disconnect went to the engine.
pub(crate) struct Ticket {
    reg: Arc<Registry>,
    user: String,
    conn: ConnId,
    pub live: Arc<Liveness>,
    pub kick: Arc<Notify>,
}

impl Ticket {
    pub fn user(&self) -> &str {
        &self.user
    }
}

impl Drop for Ticket {
    fn drop(&mut self) {
        self.reg.release(&self.user, self.conn);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MILANO: (f64, f64, f64) = (45.4642, 9.19, 120.0);
    const U1: Option<&str> = Some("6a9423e0-7ba6-4672-81a4-81b715ac1ad2");
    const U2: Option<&str> = Some("0b5c6f0e-1d3a-4f6b-9a2e-3c4d5e6f7a8b");

    fn admit(r: &Arc<Registry>, conn: ConnId, user: &str, uuid: Option<&str>, llh: (f64, f64, f64)) -> Admission {
        r.admit(conn, user, uuid, llh)
    }

    fn ticket(a: Admission) -> (Ticket, Option<String>) {
        match a {
            Admission::Admit { ticket, note } => (ticket, note),
            Admission::Replace { .. } => panic!("expected Admit, got Replace"),
            Admission::Refuse { reason, .. } => panic!("expected Admit, got Refuse: {reason}"),
        }
    }

    #[test]
    fn same_receiver_rules() {
        // With a uuid on either side, the uuids decide.
        assert!(same_receiver(U1, MILANO, U1, (10.0, 10.0, 0.0)));
        assert!(!same_receiver(U1, MILANO, U2, MILANO));
        assert!(!same_receiver(U1, MILANO, None, MILANO));
        assert!(!same_receiver(None, MILANO, U1, MILANO));
        // With none, the position: 100 m.
        assert!(same_receiver(None, MILANO, None, (45.4642 + 0.0008, 9.19, 300.0)));
        assert!(!same_receiver(None, MILANO, None, (45.4642 + 0.0012, 9.19, 120.0)));
    }

    #[test]
    fn distance_matches_identity_py() {
        // identity.distance_m((45.4642, 9.19, 0), (45.4642, 9.20, 0)) = 780.2 m.
        let d = distance_m(MILANO, (45.4642, 9.20, 0.0));
        assert!((d - 780.2).abs() < 0.5, "{d}");
        assert_eq!(distance_m(MILANO, MILANO), 0.0);
    }

    #[test]
    fn fuzzy_stays_within_ten_percent() {
        for i in 0..2000u64 {
            let v = fuzzy(60.0, mix(i));
            assert!((54.0..=66.0).contains(&v), "{v}");
        }
        assert_eq!(fuzzy(0.0, 7), 0.0);
    }

    #[test]
    fn a_live_duplicate_is_refused_and_the_first_keeps_its_name() {
        let r = Arc::new(Registry::default());
        let (t1, note) = ticket(admit(&r, 1, "milano-1", U1, MILANO));
        assert_eq!((t1.user(), note), ("milano-1", None));
        match admit(&r, 2, "milano-1", U1, MILANO) {
            Admission::Refuse { reason, retry_s } => {
                assert!(reason.starts_with("Receiver milano-1 is already connected"), "{reason}");
                // Just touched: ~45 s of liveness left + 15 s, +-10 %.
                assert!((54.0..=66.0).contains(&retry_s), "{retry_s}");
            }
            _ => panic!("expected Refuse"),
        }
        assert_eq!(r.len(), 1);
        drop(t1);
        assert_eq!(r.len(), 0);
        let (t3, _) = ticket(admit(&r, 3, "milano-1", U1, MILANO));
        assert_eq!(t3.user(), "milano-1");
    }

    #[test]
    fn the_retry_shrinks_as_the_old_connection_ages() {
        let r = Arc::new(Registry::with_windows(0.4, 0.1));
        let (_t1, _) = ticket(admit(&r, 1, "milano-1", U1, MILANO));
        std::thread::sleep(std::time::Duration::from_millis(250));
        match admit(&r, 2, "milano-1", U1, MILANO) {
            // 0.15 s left + 0.1 s, rounded to whole seconds, at least 1.
            Admission::Refuse { retry_s, .. } => assert_eq!(retry_s, 1.0),
            _ => panic!("expected Refuse"),
        }
    }

    #[test]
    fn a_quiet_duplicate_is_replaced() {
        let r = Arc::new(Registry::with_windows(0.05, 0.05));
        let (t1, _) = ticket(admit(&r, 1, "milano-1", None, MILANO));
        std::thread::sleep(std::time::Duration::from_millis(80));
        let (kick, gone) = match admit(&r, 2, "milano-1", None, (45.4642, 9.1901, 100.0)) {
            Admission::Replace { kick, gone } => (kick, gone),
            _ => panic!("expected Replace"),
        };
        assert!(Arc::ptr_eq(&kick, &t1.kick));
        // Until the old one lets go, the name is still taken.
        assert!(matches!(admit(&r, 2, "milano-1", None, MILANO), Admission::Replace { .. }));
        drop(gone);
        drop(t1);
        let (t2, note) = ticket(admit(&r, 2, "milano-1", None, MILANO));
        assert_eq!((t2.user(), note), ("milano-1", None));
    }

    #[test]
    fn touching_keeps_a_connection_live() {
        let r = Arc::new(Registry::with_windows(0.1, 0.05));
        let (t1, _) = ticket(admit(&r, 1, "milano-1", U1, MILANO));
        for _ in 0..4 {
            std::thread::sleep(std::time::Duration::from_millis(40));
            t1.live.touch();
        }
        assert!(matches!(admit(&r, 2, "milano-1", U1, MILANO), Admission::Refuse { .. }));
    }

    #[test]
    fn another_receiver_with_the_name_is_renamed() {
        let r = Arc::new(Registry::default());
        let (_t1, _) = ticket(admit(&r, 1, "milano-1", U1, MILANO));
        let (t2, note) = ticket(admit(&r, 2, "milano-1", U2, MILANO));
        let n = t2.user();
        assert!(n.starts_with("milano-1_") && n.len() == "milano-1_".len() + 3, "{n}");
        let suffix: u64 = n["milano-1_".len()..].parse().unwrap();
        assert!((100..999).contains(&suffix), "{suffix}");
        assert_eq!(note.unwrap(), format!("The name milano-1 is taken by another receiver: you are connected as {n}. Set a name of your own (MLAT_USER)."));
        // No uuid on either side, far apart: also another receiver.
        let r = Arc::new(Registry::default());
        let (_a, _) = ticket(admit(&r, 1, "my_feeder_name", None, MILANO));
        let (b, note) = ticket(admit(&r, 2, "my_feeder_name", None, (41.9, 12.5, 30.0)));
        assert_ne!(b.user(), "my_feeder_name");
        assert!(note.is_some());
        assert_eq!(r.len(), 2);
    }

    #[test]
    fn a_stale_release_does_not_free_a_newer_holder() {
        let r = Arc::new(Registry::default());
        let (t1, _) = ticket(admit(&r, 1, "milano-1", U1, MILANO));
        // Connection 1 released, 2 took the name; a late release of 1 (or a
        // release under a name it never held) leaves 2 alone.
        drop(t1);
        let (_t2, _) = ticket(admit(&r, 2, "milano-1", U1, MILANO));
        r.release("milano-1", 1);
        r.release("someone-else", 2);
        assert!(matches!(admit(&r, 3, "milano-1", U1, MILANO), Admission::Refuse { .. }));
    }
}
