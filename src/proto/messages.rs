//! Client message parsing: one JSON line in, one `Msg` out.
//!
//! Two paths that must agree. The fast path recognises, byte for byte, the
//! two lines mlat-client formats by hand and sends tens of thousands of times
//! a second (`send_tcp_mlat`, `send_tcp_sync`: integer timestamps, lowercase
//! hex, fixed key order, no spaces) and decodes them without allocating
//! anything but the frame bytes. Anything else goes through serde_json and
//! the same Python conversions mlat-server applies (`float()`,
//! `bytes.fromhex()`, `int(x, 16)`), dispatching on the first key present in
//! mlat-server's `process_message` order.

use serde_json::{Map, Value};

use super::py;

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Msg {
    Sync { et: f64, ot: f64, em: Vec<u8>, om: Vec<u8> },
    Mlat { t: f64, m: Vec<u8> },
    Seen(Vec<u32>),
    Lost(Vec<u32>),
    RateReport(Vec<(u32, f32)>),
    /// mlat-server's `receiver.clock_reset(reason)` reason string.
    ClockReset(&'static str),
    PositionUpdate { lat: f64, lon: f64, alt: f64 },
    /// Understood, nothing for the engine: heartbeat, quine, split sync,
    /// modeac_messages_count.
    Ignored,
    /// Whitespace only (not counted).
    Empty,
    /// Not JSON, not an object, an unknown message, or a field that fails the
    /// conversions or the 7/14-byte frame rule.
    Bad,
}

/// Parse one line (without its newline; trailing whitespace is fine).
pub(crate) fn parse(line: &[u8]) -> Msg {
    match fast(line) {
        Some(m) => m,
        None => slow(line),
    }
}

/// A Mode S frame is 7 or 14 bytes; anything else is skipped.
#[inline]
fn frame_ok(m: &[u8]) -> bool {
    m.len() == 7 || m.len() == 14
}

// ---------------------------------------------------------------- fast path

/// A cursor over the line for the fast path. Every method returns None on
/// the first byte that is not exactly what mlat-client would write.
struct Cur<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> Cur<'a> {
    #[inline]
    fn lit(&mut self, s: &[u8]) -> Option<()> {
        if self.b.get(self.i..self.i + s.len())? == s {
            self.i += s.len();
            Some(())
        } else {
            None
        }
    }

    /// A JSON integer that fits u64, as f64 (Python's float(int) rounding).
    /// Leading zeros, signs, fractions and exponents fall back to serde.
    #[inline]
    fn uint(&mut self) -> Option<f64> {
        let s = self.i;
        let mut v: u64 = 0;
        while let Some(&c) = self.b.get(self.i) {
            if !c.is_ascii_digit() {
                break;
            }
            v = v.checked_mul(10)?.checked_add(u64::from(c - b'0'))?;
            self.i += 1;
        }
        let n = self.i - s;
        if n == 0 || (n > 1 && self.b[s] == b'0') {
            return None;
        }
        Some(v as f64)
    }

    /// The hex body of a string up to its closing quote (quote consumed).
    /// An escape sequence falls back to serde.
    #[inline]
    fn hex_str(&mut self) -> Option<&'a [u8]> {
        let s = self.i;
        while let Some(&c) = self.b.get(self.i) {
            match c {
                b'"' => {
                    self.i += 1;
                    return Some(&self.b[s..self.i - 1]);
                }
                b'\\' => return None,
                _ => self.i += 1,
            }
        }
        None
    }

    /// Only JSON whitespace left.
    #[inline]
    fn end(&self) -> Option<()> {
        self.b[self.i..].iter().all(|c| matches!(c, b' ' | b'\t' | b'\r' | b'\n')).then_some(())
    }
}

/// Strict hex (either case); invalid hex falls back to the slow path, whose
/// fromhex() decides.
#[inline]
fn unhex(s: &[u8]) -> Option<Vec<u8>> {
    hex::decode(s).ok()
}

fn fast(line: &[u8]) -> Option<Msg> {
    let mut c = Cur { b: line, i: 0 };
    if line.starts_with(b"{\"mlat\":") {
        c.lit(b"{\"mlat\":{\"t\":")?;
        let t = c.uint()?;
        c.lit(b",\"m\":\"")?;
        let m = c.hex_str()?;
        c.lit(b"}}")?;
        c.end()?;
        let m = unhex(m)?;
        return Some(if frame_ok(&m) { Msg::Mlat { t, m } } else { Msg::Bad });
    }
    if line.starts_with(b"{\"sync\":") {
        c.lit(b"{\"sync\":{\"et\":")?;
        let et = c.uint()?;
        c.lit(b",\"em\":\"")?;
        let em = c.hex_str()?;
        c.lit(b",\"ot\":")?;
        let ot = c.uint()?;
        c.lit(b",\"om\":\"")?;
        let om = c.hex_str()?;
        c.lit(b"}}")?;
        c.end()?;
        let (em, om) = (unhex(em)?, unhex(om)?);
        return Some(if frame_ok(&em) && frame_ok(&om) { Msg::Sync { et, ot, em, om } } else { Msg::Bad });
    }
    None
}

// ---------------------------------------------------------------- slow path

fn slow(line: &[u8]) -> Msg {
    if line.iter().all(|c| c.is_ascii_whitespace()) {
        return Msg::Empty;
    }
    let Ok(Value::Object(o)) = serde_json::from_slice::<Value>(line) else {
        return Msg::Bad;
    };
    dispatch(&o).unwrap_or(Msg::Bad)
}

/// mlat-server's `process_message` chain, then the messages it does not
/// know but the module contract lists. None = malformed.
fn dispatch(o: &Map<String, Value>) -> Option<Msg> {
    if let Some(s) = o.get("sync") {
        let s = s.as_object()?;
        let et = ticks(s.get("et")?)?;
        let ot = ticks(s.get("ot")?)?;
        let em = frame(s.get("em")?)?;
        let om = frame(s.get("om")?)?;
        return Some(Msg::Sync { et, ot, em, om });
    }
    if let Some(m) = o.get("mlat") {
        let m = m.as_object()?;
        let t = ticks(m.get("t")?)?;
        let m = frame(m.get("m")?)?;
        return Some(Msg::Mlat { t, m });
    }
    if let Some(v) = o.get("seen") {
        return Some(Msg::Seen(icao_list(v)?));
    }
    if let Some(v) = o.get("lost") {
        return Some(Msg::Lost(icao_list(v)?));
    }
    // The payloads of these four are ignored by mlat-server.
    if o.contains_key("input_connected") {
        return Some(Msg::ClockReset("clientmsg: input connected"));
    }
    if o.contains_key("input_disconnected") {
        return Some(Msg::ClockReset("clientmsg: input disconnected"));
    }
    if o.contains_key("clock_reset") {
        return Some(Msg::ClockReset("clientmsg: clock_reset"));
    }
    if o.contains_key("clock_jump") {
        return Some(Msg::ClockReset("clientmsg: clock_jump"));
    }
    if o.contains_key("heartbeat") {
        return Some(Msg::Ignored);
    }
    if let Some(v) = o.get("rate_report") {
        // {int(k, 16): v}: mlat-client sends {"4CA123": 1.25, ...}.
        let mut rates = Vec::new();
        for (k, v) in v.as_object()? {
            rates.push((py::int16(k)?, v.as_f64()? as f32));
        }
        return Some(Msg::RateReport(rates));
    }
    // mlat-server answers a quine with its own source; we never do.
    if o.contains_key("quine") {
        return Some(Msg::Ignored);
    }
    // Not in mlat-server's chain (it logs "unexpected message"); mlat-client
    // never sends a position update over JSON, the shape here mirrors the
    // handshake's lat/lon/alt with the same rules.
    if let Some(v) = o.get("position_update") {
        let p = v.as_object()?;
        let lat = position_float(p.get("lat")?)?;
        let lon = position_float(p.get("lon")?)?;
        let alt = position_float(p.get("alt")?)?;
        if !(-90.0..=90.0).contains(&lat) || !(-180.0..=360.0).contains(&lon) || !(-1000.0..=10000.0).contains(&alt) {
            return None;
        }
        let lon = if lon > 180.0 { lon - 180.0 } else { lon };
        return Some(Msg::PositionUpdate { lat, lon, alt });
    }
    // modeac_messages_count: a statistic mlat-server ignores. ssync is the
    // split-sync message mlat-client sends only when asked (never).
    if o.contains_key("modeac_messages_count") || o.contains_key("split_sync") || o.contains_key("ssync") {
        return Some(Msg::Ignored);
    }
    None
}

/// `float(x)` of a timestamp; a non-finite result is skipped.
fn ticks(v: &Value) -> Option<f64> {
    py::float(v).ok().filter(|t| t.is_finite())
}

fn position_float(v: &Value) -> Option<f64> {
    py::float(v).ok().filter(|t| t.is_finite())
}

/// `bytes.fromhex(x)` of a frame, 7 or 14 bytes.
fn frame(v: &Value) -> Option<Vec<u8>> {
    py::fromhex(v.as_str()?).filter(|m| frame_ok(m))
}

/// `{int(icao, 16) for icao in x}` over a list of strings.
fn icao_list(v: &Value) -> Option<Vec<u32>> {
    v.as_array()?.iter().map(|s| py::int16(s.as_str()?)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const L7: &str = "5d4ca1239bb7c0";
    const L14: &str = "8d4ca123589b8665a2c7b1e6ba5f";

    fn mlat(t: f64, m: &str) -> Msg {
        Msg::Mlat { t, m: hex::decode(m).unwrap() }
    }

    #[test]
    fn mlat_client_lines_take_the_fast_path() {
        let line = format!("{{\"mlat\":{{\"t\":123456789012,\"m\":\"{L14}\"}}}}");
        assert_eq!(fast(line.as_bytes()), Some(mlat(123456789012.0, L14)));
        let line = format!("{{\"sync\":{{\"et\":1000,\"em\":\"{L14}\",\"ot\":2000,\"om\":\"{L14}\"}}}}");
        let want = Msg::Sync { et: 1000.0, ot: 2000.0, em: hex::decode(L14).unwrap(), om: hex::decode(L14).unwrap() };
        assert_eq!(fast(line.as_bytes()), Some(want));
        // Trailing \r and spaces are still the fast path.
        let line = format!("{{\"mlat\":{{\"t\":0,\"m\":\"{L7}\"}}}} \r");
        assert_eq!(fast(line.as_bytes()), Some(mlat(0.0, L7)));
    }

    #[test]
    fn other_spellings_take_the_slow_path_with_the_same_result() {
        let cases = [
            format!("{{\"mlat\": {{\"t\": 5, \"m\": \"{L7}\"}}}}"),
            format!("{{\"mlat\":{{\"m\":\"{L7}\",\"t\":5}}}}"),
            format!("{{\"mlat\":{{\"t\":5.0,\"m\":\"{L7}\"}}}}"),
            format!("{{\"mlat\":{{\"t\":\"5\",\"m\":\"{L7}\"}}}}"),
            format!("{{\"mlat\":{{\"t\":5e0,\"m\":\"{}\"}}}}", L7.to_uppercase()),
            "{\"mlat\":{\"t\":5,\"m\":\"5d 4c a1 23 9b b7 c0\"}}".to_string(),
            "{\"mlat\":{\"t\":5,\"m\":\"\\u0035d4ca1239bb7c0\"}}".to_string(),
            format!("{{\"mlat\":{{\"t\":5,\"m\":\"{L7}\",\"extra\":[1,2]}}}}"),
        ];
        for c in cases {
            assert_eq!(fast(c.as_bytes()), None, "{c}");
            assert_eq!(parse(c.as_bytes()), mlat(5.0, L7), "{c}");
        }
    }

    #[test]
    fn bad_frames_and_timestamps_are_skipped() {
        for c in [
            "{\"mlat\":{\"t\":5,\"m\":\"5d4ca1\"}}",                       // 3 bytes
            "{\"mlat\":{\"t\":5,\"m\":\"5d4ca1239bb7c\"}}",                // odd hex
            "{\"mlat\":{\"t\":5,\"m\":\"zz4ca1239bb7c0\"}}",               // not hex
            "{\"mlat\":{\"t\":5,\"m\":\"5d4 ca1239bb7c0\"}}",              // space inside a pair
            "{\"mlat\":{\"t\":\"nan\",\"m\":\"5d4ca1239bb7c0\"}}",          // NaN ticks
            "{\"mlat\":{\"t\":null,\"m\":\"5d4ca1239bb7c0\"}}",
            "{\"mlat\":{\"m\":\"5d4ca1239bb7c0\"}}",
            "{\"mlat\":[5,\"5d4ca1239bb7c0\"]}",
            "{\"sync\":{\"et\":1,\"em\":\"5d4ca1239bb7c0\",\"ot\":2}}",
            "{\"sync\":{\"et\":1,\"em\":\"5d4ca1239bb7c0\",\"ot\":2,\"om\":\"5d4c\"}}",
            "{\"mlat\":{\"t\":01,\"m\":\"5d4ca1239bb7c0\"}}",               // leading zero: not JSON
            "{\"mlat\":{\"t\":5,\"m\":\"5d4ca1239bb7c0\"}}x",
            "{\"nope\":1}",
            "[1,2,3]",
            "42",
            "{",
        ] {
            assert_eq!(parse(c.as_bytes()), Msg::Bad, "{c}");
        }
        assert_eq!(parse(b""), Msg::Empty);
        assert_eq!(parse(b"  \r"), Msg::Empty);
    }

    #[test]
    fn big_and_huge_timestamps() {
        // 2^53 + 1 rounds the way CPython's float(int) does, on both paths.
        let f = format!("{{\"mlat\":{{\"t\":9007199254740993,\"m\":\"{L7}\"}}}}");
        assert_eq!(parse(f.as_bytes()), mlat(9007199254740992.0, L7));
        let s = format!("{{\"mlat\": {{\"t\": 9007199254740993, \"m\": \"{L7}\"}}}}");
        assert_eq!(parse(s.as_bytes()), mlat(9007199254740992.0, L7));
        // Past u64: the fast path gives up and serde parses a float,
        // correctly rounded like CPython's float(123456789012345678901234)
        // = 1.2345678901234569e+23 (serde's default parser is 1 ULP off).
        let h = format!("{{\"mlat\":{{\"t\":123456789012345678901234,\"m\":\"{L7}\"}}}}");
        assert_eq!(fast(h.as_bytes()), None);
        assert_eq!(parse(h.as_bytes()), mlat(1.2345678901234569e23, L7));
    }

    #[test]
    fn the_other_messages() {
        assert_eq!(parse(b"{\"seen\":[\"4ca123\",\"3C6444\"]}"), Msg::Seen(vec![0x4ca123, 0x3c6444]));
        assert_eq!(parse(b"{\"lost\":[]}"), Msg::Lost(vec![]));
        assert_eq!(parse(b"{\"lost\":[\"4ca123\",5]}"), Msg::Bad);
        assert_eq!(parse(b"{\"seen\":\"4ca123\"}"), Msg::Bad);
        assert_eq!(parse(b"{\"rate_report\":{}}"), Msg::RateReport(vec![]));
        assert_eq!(parse(b"{\"rate_report\":{\"4CA123\":1.5}}"), Msg::RateReport(vec![(0x4ca123, 1.5)]));
        assert_eq!(parse(b"{\"rate_report\":{\"4CA123\":\"x\"}}"), Msg::Bad);
        assert_eq!(parse(b"{\"input_connected\":\"connected\"}"), Msg::ClockReset("clientmsg: input connected"));
        assert_eq!(parse(b"{\"input_disconnected\":\"disconnected\"}"), Msg::ClockReset("clientmsg: input disconnected"));
        assert_eq!(parse(b"{\"clock_reset\":{\"reason\":\"Epoch rollover detected\"}}"), Msg::ClockReset("clientmsg: clock_reset"));
        assert_eq!(parse(b"{\"clock_jump\":true}"), Msg::ClockReset("clientmsg: clock_jump"));
        assert_eq!(parse(b"{\"heartbeat\":{\"client_time\":1729760000.123}}"), Msg::Ignored);
        assert_eq!(parse(b"{\"modeac_messages_count\":{\"count\":42}}"), Msg::Ignored);
        assert_eq!(parse(b"{\"ssync\":{\"t\":1,\"m\":\"00\"}}"), Msg::Ignored);
        assert_eq!(parse(b"{\"split_sync\":{}}"), Msg::Ignored);
        assert_eq!(parse(b"{\"quine\":\"mlat.jsonclient\"}"), Msg::Ignored);
        assert_eq!(
            parse(b"{\"position_update\":{\"lat\":45.5,\"lon\":200,\"alt\":\"120\"}}"),
            Msg::PositionUpdate { lat: 45.5, lon: 20.0, alt: 120.0 }
        );
        assert_eq!(parse(b"{\"position_update\":{\"lat\":95,\"lon\":9,\"alt\":0}}"), Msg::Bad);
        // mlat-server's precedence when a line carries two keys.
        assert_eq!(parse(b"{\"heartbeat\":{},\"seen\":[\"000001\"]}"), Msg::Seen(vec![1]));
    }

    /// The fast path must agree with the general path on every line it
    /// accepts: generate mlat-client-shaped lines plus random corruptions.
    #[test]
    fn fast_and_slow_agree() {
        let mut x: u64 = 0x1234_5678_9abc_def0;
        let mut next = || {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            x >> 16
        };
        let mut fast_hits = 0;
        for i in 0..20000 {
            let n = if next() % 2 == 0 { 7 } else { 14 };
            let m: String = (0..n).map(|_| format!("{:02x}", next() & 0xff)).collect();
            let t = next() >> (next() % 48);
            let mut line = if i % 2 == 0 {
                format!("{{\"mlat\":{{\"t\":{t},\"m\":\"{m}\"}}}}")
            } else {
                format!("{{\"sync\":{{\"et\":{t},\"em\":\"{m}\",\"ot\":{},\"om\":\"{m}\"}}}}", t + 1)
            }
            .into_bytes();
            if i % 5 == 0 {
                // Flip, drop or insert a byte somewhere.
                let p = (next() as usize) % line.len();
                match next() % 3 {
                    0 => line[p] = (next() & 0x7f) as u8,
                    1 => {
                        line.remove(p);
                    }
                    _ => line.insert(p, b"0aZ\" ,}{\\"[(next() % 9) as usize]),
                }
            }
            if let Some(f) = fast(&line) {
                fast_hits += 1;
                assert_eq!(f, slow(&line), "{}", String::from_utf8_lossy(&line));
            }
        }
        assert!(fast_hits > 15000);
    }

    #[test]
    fn random_bytes_never_panic() {
        let mut x: u64 = 42;
        for _ in 0..20000 {
            x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let len = (x >> 58) as usize;
            let mut b = Vec::with_capacity(len);
            for k in 0..len {
                x = x.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                // Mostly JSON-ish bytes so the parser gets past the first char.
                const ALPHABET: &[u8] = b"{}[]\":,0123456789abcdef synclatmetx\\-.e";
                b.push(if k == 0 { b'{' } else { ALPHABET[(x >> 33) as usize % ALPHABET.len()] });
            }
            let _ = parse(&b);
        }
    }
}
