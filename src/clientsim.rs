//! Emulated mlat-clients for serve-mode replays: what each feeder would send
//! to this server, from what its receiver heard (a 4906 capture), given what
//! the server asks it for. A port of mlat-client 0.4.x (wiedehopf,
//! `mlat/client/coordinator.py` and the reader filter in `modes_reader.c`):
//!
//! - The reader passes every DF17, and DF0/4/5/11/16/20/21 only for the
//!   "interested" set: requested aircraft without a good ADS-B position.
//!   DF11 and CRC-clean DF17/18 mark an aircraft as seen even when filtered.
//! - An aircraft needs 10 messages before anything of it is forwarded.
//! - DF17 airborne positions (TC 9..18, 20, 21) with an altitude, NUCp >= 6
//!   and an even/odd pair within 5 s count towards the rate report and make
//!   the aircraft "ADS-B good" (both halves within 30 s); a requested one
//!   sends the pair as sync.
//! - Other replies of a requested aircraft that is not ADS-B good go out as
//!   mlat.
//! - Every 4.5 s (+ up to 1 s) the client reviews its aircraft; every 4th
//!   review it sends seen / lost and a rate report. Right after the handshake
//!   it sends an empty rate report.
//!
//! `select = false` emulates a client that is asked for everything it sees
//! (no selection): the baseline for traffic and accuracy.

use crate::hashing::{HashMap, HashSet};

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use tokio::sync::mpsc;

use crate::modes::{self, EsKind};
use crate::relay::beast::Frame;
use crate::relay::registry::Receiver;
use crate::types::{ConnId, Event, ReceiverInfo, ResultFormat, ServeOpts, ToClient};

const UPDATE_INTERVAL: f64 = 4.5;
const REPORT_EVERY: f64 = 4.0;
const POSITION_EXPIRY: f64 = 30.0;
const EXPIRY: f64 = 120.0;

#[derive(Default)]
struct Ac {
    messages: u32,
    last_message: f64,
    last_even: f64,
    last_odd: f64,
    adsb_good: bool,
    even: Option<(u64, [u8; 14])>,
    odd: Option<(u64, [u8; 14])>,
    rate_start: f64,
    recent_positions: u32,
}

/// Traffic a client sent, and what came back.
#[derive(Debug, Default, Clone, Copy)]
pub struct Sent {
    pub sync: u64,
    pub mlat: u64,
    pub results: u64,
    pub start_lines: u64,
    pub stop_lines: u64,
}

struct Client {
    conn: ConnId,
    rx: mpsc::Receiver<ToClient>,
    freq: f64,
    aircraft: HashMap<u32, Ac>,
    requested: HashSet<u32>,
    /// The reader's specific filter: requested and not ADS-B good.
    interested: HashSet<u32>,
    /// The reader's seen set since the last review.
    seen: HashSet<u32>,
    reported: HashSet<u32>,
    next_update: f64,
    last_update: f64,
    next_report: f64,
    sent: Sent,
    /// Peak size of the requested set.
    requested_max: usize,
}

pub struct ClientSim {
    clients: HashMap<u64, Client>,
    select: bool,
    rng: StdRng,
}

impl ClientSim {
    pub fn new(select: bool) -> ClientSim {
        ClientSim { clients: HashMap::default(), select, rng: StdRng::seed_from_u64(0xc11e) }
    }

    pub fn len(&self) -> usize {
        self.clients.len()
    }

    /// Connect a client for receiver `rid` (once), as a served receiver that
    /// asked for ecef results and stats.
    fn connect(&mut self, rid: u64, reg: &Receiver, conn: ConnId, wall: f64, out: &mut Vec<Event>) {
        let (tx, rx) = mpsc::channel(1 << 14);
        out.push(Event::Connect {
            conn,
            info: ReceiverInfo {
                user: reg.user.clone(),
                uuid: Some(format!("{rid:016x}")),
                lat: reg.lat,
                lon: reg.lon,
                alt: reg.alt,
                clock: reg.clock,
                privacy: reg.privacy,
                source_ip: "sim".into(),
                source_port: String::new(),
                client_version: "sim".into(),
                compress: "none",
                connection: String::new(),
                msgs: Default::default(),
                serve: Some(ServeOpts { results: ResultFormat::Ecef, stats: true }),
            },
            tx,
        });
        out.push(Event::RateReport { conn, rates: Vec::new() });
        let first = wall + UPDATE_INTERVAL + self.rng.gen::<f64>();
        self.clients.insert(
            rid,
            Client {
                conn,
                rx,
                freq: reg.clock.freq(),
                aircraft: HashMap::default(),
                requested: HashSet::default(),
                interested: HashSet::default(),
                seen: HashSet::default(),
                reported: HashSet::default(),
                next_update: first,
                last_update: wall,
                next_report: self.rng.gen::<f64>() * REPORT_EVERY,
                sent: Sent::default(),
                requested_max: 0,
            },
        );
    }

    /// One frame heard by receiver `f.rid`; receivers not in the registry
    /// have no position and no client.
    pub fn on_frame(&mut self, f: &Frame, wall: f64, registry: &HashMap<u64, Receiver>, conn_base: ConnId, out: &mut Vec<Event>) {
        if f.kind == b'1' {
            return; // Mode A/C: not requested by this server
        }
        if !self.clients.contains_key(&f.rid) {
            let Some(reg) = registry.get(&f.rid) else { return };
            let conn = conn_base + self.clients.len() as ConnId;
            self.connect(f.rid, reg, conn, wall, out);
        }
        let select = self.select;
        let c = self.clients.get_mut(&f.rid).unwrap();
        c.frame(f.bytes(), f.ts, wall, select, out);
    }

    /// Timers, and the server's requests to each client.
    pub fn tick(&mut self, wall: f64, out: &mut Vec<Event>) {
        let select = self.select;
        // In receiver order: each client draws from the shared rng
        // (hashing.rs: nothing may depend on the map's order).
        let mut rids: Vec<u64> = self.clients.keys().copied().collect();
        rids.sort_unstable();
        for rid in rids {
            let c = self.clients.get_mut(&rid).unwrap();
            while let Ok(m) = c.rx.try_recv() {
                match m {
                    ToClient::StartSending(v) => {
                        c.sent.start_lines += 1;
                        c.requested.extend(v);
                    }
                    ToClient::StopSending(v) => {
                        c.sent.stop_lines += 1;
                        for a in v {
                            c.requested.remove(&a);
                        }
                    }
                    ToClient::ResultLine(_) => c.sent.results += 1,
                    ToClient::StatsLine(_) | ToClient::Close => {}
                }
                c.requested_max = c.requested_max.max(c.requested.len());
                c.refilter(select);
            }
            if wall >= c.next_update {
                c.next_update = wall + UPDATE_INTERVAL + self.rng.gen::<f64>();
                c.update(wall, select);
                c.next_report += 1.0;
                if c.next_report >= REPORT_EVERY {
                    c.next_report = 0.0;
                    c.report(wall, out);
                }
            }
        }
    }

    /// Totals over all clients, and the mean of their peak requested sets.
    pub fn totals(&self) -> (Sent, f64) {
        let mut t = Sent::default();
        let mut req = 0usize;
        for c in self.clients.values() {
            t.sync += c.sent.sync;
            t.mlat += c.sent.mlat;
            t.results += c.sent.results;
            t.start_lines += c.sent.start_lines;
            t.stop_lines += c.sent.stop_lines;
            req += c.requested_max;
        }
        (t, if self.clients.is_empty() { 0.0 } else { req as f64 / self.clients.len() as f64 })
    }

    /// Messages sent per client over the run (sync + mlat), for quantiles.
    pub fn per_client(&self) -> Vec<u64> {
        self.clients.values().map(|c| c.sent.sync + c.sent.mlat).collect()
    }
}

impl Client {
    fn wants(&self, icao: u32, select: bool) -> bool {
        !select || self.requested.contains(&icao)
    }

    /// update_receiver_filter
    fn refilter(&mut self, select: bool) {
        if !select {
            return;
        }
        self.interested = self.requested.iter().filter(|a| self.aircraft.get(a).is_none_or(|ac| !ac.adsb_good)).copied().collect();
    }

    fn frame(&mut self, msg: &[u8], ts: u64, now: f64, select: bool, out: &mut Vec<Event>) {
        let Some(d) = modes::decode(msg) else { return };
        // Reader: CRC-checked DF11/17/18 mark the aircraft seen.
        let clean = match d.df {
            11 => d.crc_ok != Some(false),
            17 | 18 => d.crc_ok == Some(true),
            _ => true,
        };
        if !clean {
            return;
        }
        if matches!(d.df, 11 | 17 | 18) {
            self.seen.insert(d.address);
        }
        // Reader filter: DF17 always; the mlat DFs only when interested.
        let passes = match d.df {
            17 => true,
            0 | 4 | 5 | 11 | 16 | 20 | 21 => !select || self.interested.contains(&d.address),
            _ => false,
        };
        if !passes {
            return;
        }
        match d.df {
            17 => self.df17(&d, msg, ts, now, select, out),
            11 => {
                let ac = match self.aircraft.get_mut(&d.address) {
                    Some(ac) => ac,
                    None => {
                        self.aircraft.insert(d.address, Ac { messages: 1, last_message: now, rate_start: now, ..Ac::default() });
                        return;
                    }
                };
                ac.messages += 1;
                ac.last_message = now;
                if ac.messages < 10 || ac.adsb_good || !(!select || self.requested.contains(&d.address)) {
                    return;
                }
                self.sent.mlat += 1;
                out.push(Event::Mlat { conn: self.conn, t: ts as f64, msg: msg.to_vec(), rx: now });
            }
            _ => {
                let wanted = self.wants(d.address, select);
                let Some(ac) = self.aircraft.get_mut(&d.address) else { return };
                ac.messages += 1;
                ac.last_message = now;
                if ac.messages < 10 || !wanted || ac.adsb_good {
                    return;
                }
                self.sent.mlat += 1;
                out.push(Event::Mlat { conn: self.conn, t: ts as f64, msg: msg.to_vec(), rx: now });
            }
        }
    }

    fn df17(&mut self, d: &modes::Decoded, msg: &[u8], ts: u64, now: f64, select: bool, out: &mut Vec<Event>) {
        let wanted = self.wants(d.address, select);
        let freq = self.freq;
        let ac = match self.aircraft.get_mut(&d.address) {
            Some(ac) => ac,
            None => {
                self.aircraft.insert(d.address, Ac { messages: 1, last_message: now, rate_start: now, ..Ac::default() });
                return;
            }
        };
        ac.messages += 1;
        ac.last_message = now;
        if ac.messages < 10 {
            return;
        }
        let Some(es) = d.es.as_ref() else { return };
        // mlat-client: airborne TC 9..18 and 20..21 only.
        if !(es.kind == EsKind::AirbornePosition && (9..=18).contains(&es.tc) || (20..=21).contains(&es.tc)) {
            return;
        }
        if es.lat_cpr == 0 || es.lon_cpr == 0 {
            return; // the reader's "mostly zeros" CPR check, roughly
        }
        let mut b = [0u8; 14];
        b.copy_from_slice(&msg[..14]);
        if es.odd {
            ac.odd = Some((ts, b));
        } else {
            ac.even = Some((ts, b));
        }
        let (Some((te, me)), Some((to, mo))) = (ac.even, ac.odd) else { return };
        if (te as f64 - to as f64).abs() > 5.0 * freq {
            return;
        }
        if d.altitude_ft.is_none() || es.nuc.unwrap_or(0) < 6 {
            return;
        }
        ac.recent_positions += 1;
        if es.odd {
            ac.last_odd = now;
        } else {
            ac.last_even = now;
        }
        ac.adsb_good = now - ac.last_even < POSITION_EXPIRY && now - ac.last_odd < POSITION_EXPIRY;
        if !wanted {
            return;
        }
        self.sent.sync += 1;
        out.push(Event::Sync { conn: self.conn, et: te as f64, ot: to as f64, em: me.to_vec(), om: mo.to_vec() });
    }

    /// update_aircraft
    fn update(&mut self, now: f64, select: bool) {
        for icao in std::mem::take(&mut self.seen) {
            let ac = self.aircraft.entry(icao).or_insert_with(|| Ac { rate_start: now, ..Ac::default() });
            if ac.last_message <= self.last_update {
                ac.messages += 1;
                ac.last_message = now;
            }
            ac.adsb_good = now - ac.last_even < POSITION_EXPIRY && now - ac.last_odd < POSITION_EXPIRY;
        }
        self.aircraft.retain(|_, a| now - a.last_message <= EXPIRY);
        self.last_update = now;
        self.refilter(select);
    }

    /// send_aircraft_report + send_rate_report
    fn report(&mut self, now: f64, out: &mut Vec<Event>) {
        let all: HashSet<u32> = self.aircraft.iter().filter(|(_, a)| a.messages > 1).map(|(k, _)| *k).collect();
        let mut seen: Vec<u32> = all.difference(&self.reported).copied().collect();
        let mut lost: Vec<u32> = self.reported.difference(&all).copied().collect();
        seen.sort_unstable();
        lost.sort_unstable();
        if !seen.is_empty() {
            out.push(Event::Seen { conn: self.conn, icao: seen });
        }
        if !lost.is_empty() {
            out.push(Event::Lost { conn: self.conn, icao: lost });
        }
        self.reported = all;
        let mut rates = Vec::new();
        for (icao, a) in self.aircraft.iter_mut() {
            let interval = now - a.rate_start;
            if interval > 0.0 && a.recent_positions > 0 {
                let r = a.recent_positions as f64 / interval;
                a.rate_start = now;
                a.recent_positions = 0;
                rates.push((*icao, ((r * 100.0).round() / 100.0) as f32));
            }
        }
        if !rates.is_empty() {
            rates.sort_unstable_by_key(|(a, _)| *a);
            out.push(Event::RateReport { conn: self.conn, rates });
        }
    }
}
