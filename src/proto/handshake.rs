//! The handshake: optional PROXY v1 line, validation of the client's JSON
//! object, and the reply. Mirrors mlat-server's `process_handshake` (master
//! 33c3361 + PR #14) check by check and in the same order, so that a client
//! the primary accepts is accepted here with the same name, position and
//! compression, and one it refuses is refused here too.

use serde_json::{Map, Value};

use super::py::{self, PyErr};
use super::ListenConfig;
use crate::types::{ClockType, ResultFormat};

/// Note PR #14 adds to the motd of a client sending the sample-.env uuid.
const PLACEHOLDER_NOTE: &str = "Your MLAT client sends the placeholder uuid of the sample .env (XXXXXXXX-...). \
Set your own, from `cat /proc/sys/kernel/random/uuid`, so this server can tell \
your receiver apart from the others that kept the placeholder.";

/// What mlat-server would do with a handshake line.
#[derive(Debug)]
pub(crate) enum Outcome {
    Accept(Box<Accepted>),
    /// mlat-server writes `{"deny": [reason], ...}` and closes: the handshake
    /// raised KeyError or ValueError (or was not JSON / not ASCII).
    Deny(String),
    /// mlat-server raises something else (TypeError on a non-dict handshake or
    /// a null field, NotImplementedError on an unknown clock type) and closes
    /// without replying.
    Fatal(&'static str),
}

/// An accepted handshake; `ReceiverInfo` minus the source address.
#[derive(Debug, Clone)]
pub(crate) struct Accepted {
    pub user: String,
    pub uuid: Option<String>,
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
    pub clock: ClockType,
    pub privacy: bool,
    pub client_version: String,
    pub compress: &'static str,
    pub return_results: bool,
    /// The layout results go out in when `return_results` (default "old").
    pub result_format: ResultFormat,
    /// `hs.get('return_stats', False)`, echoed back verbatim like mlat-server.
    pub return_stats: Value,
    /// Extra motd paragraphs (PR #14's placeholder-uuid note).
    pub notes: Vec<&'static str>,
}

/// The source address of a PROXY v1 line (`PROXY TCP4 src dst sport dport`).
/// mlat-server takes `line.split(' ')[2]` and also reads `[4]`, so a line
/// with fewer than five space-separated fields (`PROXY UNKNOWN`) raises
/// IndexError and the connection is closed: None here.
pub(crate) fn parse_proxy(line: &[u8]) -> Option<String> {
    let s = std::str::from_utf8(line).ok()?;
    let parts: Vec<&str> = s.split(' ').collect();
    if parts.len() < 5 {
        return None;
    }
    Some(parts[2].to_string())
}

/// Judge one handshake line (as read, newline included).
pub(crate) fn evaluate(line: &[u8]) -> Outcome {
    // mlat-server decodes the line as ASCII first; UnicodeDecodeError is a
    // ValueError, hence a deny. mlat-client's json.dumps escapes non-ASCII.
    if !line.is_ascii() {
        return Outcome::Deny("Badly formatted handshake: 'ascii' codec can't decode the handshake".into());
    }
    let hs: Value = match serde_json::from_slice(line) {
        Ok(v) => v,
        Err(e) => return Outcome::Deny(format!("Badly formatted handshake: {e}")),
    };
    match check(&hs) {
        Ok(a) => Outcome::Accept(Box::new(a)),
        Err(Refusal::Key(k)) => Outcome::Deny(format!("Missing field in handshake: '{k}'")),
        Err(Refusal::Value(m)) => Outcome::Deny(format!("Bad values in handshake: {m}")),
        Err(Refusal::Fatal(why)) => Outcome::Fatal(why),
    }
}

enum Refusal {
    Key(&'static str),
    Value(String),
    Fatal(&'static str),
}

impl From<PyErr> for Refusal {
    fn from(e: PyErr) -> Self {
        match e {
            PyErr::Value(m) => Refusal::Value(m),
            PyErr::Type => Refusal::Fatal("TypeError in handshake"),
        }
    }
}

fn get<'a>(hs: &'a Map<String, Value>, k: &'static str) -> Result<&'a Value, Refusal> {
    hs.get(k).ok_or(Refusal::Key(k))
}

fn check(hs: &Value) -> Result<Accepted, Refusal> {
    // hs['version'] on a list / string / number is a TypeError.
    let hs = hs.as_object().ok_or(Refusal::Fatal("handshake is not a JSON object"))?;

    let version = get(hs, "version")?;
    if !py::eq_int(version, 2) && !py::eq_int(version, 3) {
        return Err(Refusal::Value("Unsupported version in handshake".into()));
    }

    // 'compress' must be present, and set(hs['compress']) a subset of the
    // three methods: one unknown method refuses the client outright.
    let compress = hs.get("compress").ok_or_else(|| Refusal::Value("Unsupported: Missing compression".into()))?;
    let offered = offered_methods(compress)?;
    if offered.other {
        return Err(Refusal::Value(format!("Unsupported compression type, got: {}", py::repr(&Value::Object(hs.clone())))));
    }

    let user = py::str(get(hs, "user")?);
    // PR #14: a placeholder uuid identifies nobody. Unlike PR #14 a non-string
    // uuid (some clients send a list) is dropped too: the engine keys on it.
    let mut notes = Vec::new();
    let uuid = match hs.get("uuid") {
        Some(Value::String(s)) if is_placeholder_uuid(s) => {
            notes.push(PLACEHOLDER_NOTE);
            None
        }
        Some(Value::String(s)) => Some(s.clone()),
        _ => None,
    };
    let user = sanitize_user(&user);

    // The first of zlib2, zlib, none that the client offers.
    let compress = if offered.zlib2 {
        "zlib2"
    } else if offered.zlib {
        "zlib"
    } else if offered.none {
        "none"
    } else {
        return Err(Refusal::Value("No mutually usable compression type".into()));
    };

    let lat = py::float(get(hs, "lat")?)?;
    // NaN passes mlat-server's comparisons; we refuse it rather than hand the
    // engine a NaN position (only a string "nan" can produce one).
    if !(-90.0..=90.0).contains(&lat) {
        return Err(Refusal::Value("invalid latitude, should be -90 .. 90".into()));
    }
    let mut lon = py::float(get(hs, "lon")?)?;
    if !(-180.0..=360.0).contains(&lon) {
        return Err(Refusal::Value("invalid longitude, should be -180 .. 360".into()));
    }
    // mlat-server's quirk, kept so both servers place the receiver alike:
    // 0..360 longitudes above 180 lose 180, not 360.
    if lon > 180.0 {
        lon -= 180.0;
    }
    let alt = py::float(get(hs, "alt")?)?;
    if !(-1000.0..=10000.0).contains(&alt) {
        return Err(Refusal::Value("invalid altitude, should be -1000 .. 10000".into()));
    }

    // hs.get('clock_type', 'dump1090'): a present null is str(None) = "None".
    let clock_type = hs.get("clock_type").map_or_else(|| "dump1090".to_string(), py::str);

    let return_results = py::truthy(hs.get("return_results"));
    let mut result_format = ResultFormat::None;
    if return_results {
        result_format = match hs.get("return_result_format") {
            None => ResultFormat::Old, // defaults to 'old'
            Some(Value::String(f)) if f == "old" => ResultFormat::Old,
            Some(Value::String(f)) if f == "ecef" => ResultFormat::Ecef,
            Some(_) => {
                return Err(Refusal::Value("invalid return_result_format, should be one of \"old\" or \"ecef\"".into()));
            }
        };
    }
    let return_stats = hs.get("return_stats").cloned().unwrap_or(Value::Bool(false));
    let privacy = py::truthy(hs.get("privacy"));
    let client_version = hs.get("client_version").map_or_else(|| "unknown".to_string(), py::str);

    // Receiver() calls make_clock(), which raises NotImplementedError for an
    // unknown type: not caught, so no deny is sent.
    let clock = ClockType::parse(&clock_type).ok_or(Refusal::Fatal("unknown clock type"))?;

    Ok(Accepted {
        user,
        uuid,
        lat,
        lon,
        alt,
        clock,
        privacy,
        client_version,
        compress,
        return_results,
        result_format,
        return_stats,
        notes,
    })
}

#[derive(Default)]
struct Offered {
    zlib2: bool,
    zlib: bool,
    none: bool,
    /// Anything that is not one of the three method names.
    other: bool,
}

/// `set(hs['compress'])`: a list's items, a dict's keys, a string's
/// characters; a list holding a list or dict, or a scalar, is a TypeError.
fn offered_methods(v: &Value) -> Result<Offered, Refusal> {
    let mut o = Offered::default();
    let mut add = |s: &str| match s {
        "zlib2" => o.zlib2 = true,
        "zlib" => o.zlib = true,
        "none" => o.none = true,
        _ => o.other = true,
    };
    match v {
        Value::Array(items) => {
            for it in items {
                match it {
                    Value::String(s) => add(s),
                    Value::Array(_) | Value::Object(_) => return Err(Refusal::Fatal("unhashable compress entry")),
                    _ => add(""), // hashable, never a method name
                }
            }
        }
        Value::Object(m) => m.keys().for_each(|k| add(k)),
        // set("none") is {'n', 'o', 'e'}: never a subset unless empty.
        Value::String(s) => {
            if !s.is_empty() {
                add("");
            }
        }
        _ => return Err(Refusal::Fatal("compress is not iterable")),
    }
    Ok(o)
}

/// PR #14's `is_placeholder_uuid`: empty, or only X's, or only zeros once
/// dashes and spaces are removed.
pub(crate) fn is_placeholder_uuid(s: &str) -> bool {
    let mut any = false;
    let mut all_x = true;
    let mut all_0 = true;
    for c in s.chars().filter(|&c| c != '-' && c != ' ') {
        any = true;
        all_x &= c == 'x' || c == 'X';
        all_0 &= c == '0';
    }
    !any || all_x || all_0
}

/// mlat-server's name rules: every character outside [A-Za-z0-9_.-] becomes
/// '_' (per code point, so "ü" is one '_'), cut to 40, and a name shorter
/// than 3 gets '_' plus a random 10..98. The rename of a name already in use
/// is the engine's business.
pub(crate) fn sanitize_user(raw: &str) -> String {
    let mut u: String = raw
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '-') { c } else { '_' })
        .take(40)
        .collect();
    if u.len() < 3 {
        use rand::Rng;
        u.push_str(&format!("_{}", rand::thread_rng().gen_range(10..99)));
    }
    u
}

/// mlat-server's expanded motd (PR #14 layout, notes included).
fn expanded_motd(motd: &str, notes: &[&str], agpl_url: &str) -> String {
    let notes: String = notes.iter().map(|n| format!("\n        {n}\n")).collect();
    format!(
        "\n\n        {motd}\n        {notes}\n        The multilateration server source code is available under\n        \
         the terms of the Affero GPL (v3 or later). You may obtain\n        \
         a copy of this server's source code at the following\n        \
         location: {agpl_url}\n        "
    )
}

/// The accept reply, keys in mlat-server's order (no newline).
pub(crate) fn accept_line(a: &Accepted, cfg: &ListenConfig) -> String {
    let motd = expanded_motd(&cfg.motd, &a.notes, &cfg.source_url);
    format!(
        "{{\"compress\":\"{}\",\"reconnect_in\":10,\"selective_traffic\":true,\"heartbeat\":true,\
         \"return_results\":{},\"return_stats\":{},\"rate_reports\":true,\"forward_all_ac\":false,\"motd\":{}}}",
        a.compress,
        a.return_results,
        a.return_stats,
        Value::String(motd),
    )
}

/// The refusal reply (no newline).
pub(crate) fn deny_line(reason: &str) -> String {
    format!("{{\"deny\":[{}],\"reconnect_in\":900}}", Value::String(reason.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn hs(v: Value) -> Outcome {
        let mut line = crate::proto::tests::ascii_json(&v).into_bytes();
        line.push(b'\n');
        evaluate(&line)
    }

    fn base() -> Value {
        json!({"version": 3, "user": "milano-1", "uuid": "6a9423e0-7ba6-4672-81a4-81b715ac1ad2",
               "compress": ["none", "zlib", "zlib2"], "lat": 45.4642, "lon": 9.19, "alt": 120.0,
               "clock_type": "dump1090", "client_version": "0.3.91", "return_results": false})
    }

    fn with(k: &str, v: Value) -> Value {
        let mut b = base();
        b[k] = v;
        b
    }

    fn without(k: &str) -> Value {
        let mut b = base();
        b.as_object_mut().unwrap().remove(k);
        b
    }

    fn accept(v: Value) -> Accepted {
        match hs(v) {
            Outcome::Accept(a) => *a,
            o => panic!("expected accept, got {o:?}"),
        }
    }

    fn deny(v: Value) -> String {
        match hs(v) {
            Outcome::Deny(r) => r,
            o => panic!("expected deny, got {o:?}"),
        }
    }

    #[test]
    fn real_client_handshake_is_accepted() {
        let a = accept(base());
        assert_eq!(a.user, "milano-1");
        assert_eq!(a.uuid.as_deref(), Some("6a9423e0-7ba6-4672-81a4-81b715ac1ad2"));
        assert_eq!(a.compress, "zlib2");
        assert_eq!((a.lat, a.lon, a.alt), (45.4642, 9.19, 120.0));
        assert_eq!(a.clock, ClockType::Dump1090);
        assert!(!a.return_results && !a.privacy);
        assert_eq!(a.client_version, "0.3.91");
    }

    #[test]
    fn compression_is_chosen_like_mlat_server() {
        assert_eq!(accept(with("compress", json!(["none"]))).compress, "none");
        assert_eq!(accept(with("compress", json!(["none", "zlib"]))).compress, "zlib");
        assert_eq!(accept(with("compress", json!(["zlib2", "none"]))).compress, "zlib2");
        // A dict offers its keys, as set() does.
        assert_eq!(accept(with("compress", json!({"zlib": 1}))).compress, "zlib");
        // One unknown method refuses the whole handshake.
        assert!(deny(with("compress", json!(["zlib2", "lz4"]))).starts_with("Bad values in handshake: Unsupported compression type"));
        assert!(deny(with("compress", json!("zlib2"))).contains("Unsupported compression type"));
        assert!(deny(with("compress", json!([]))).contains("No mutually usable compression type"));
        assert!(deny(without("compress")).contains("Missing compression"));
        assert!(matches!(hs(with("compress", json!(null))), Outcome::Fatal(_)));
        assert!(matches!(hs(with("compress", json!([["none"]]))), Outcome::Fatal(_)));
    }

    #[test]
    fn versions() {
        assert!(matches!(hs(with("version", json!(2))), Outcome::Accept(_)));
        assert!(matches!(hs(with("version", json!(3.0))), Outcome::Accept(_)));
        for v in [json!(1), json!(4), json!("3"), json!(true), json!(null)] {
            assert_eq!(deny(with("version", v)), "Bad values in handshake: Unsupported version in handshake");
        }
        assert_eq!(deny(without("version")), "Missing field in handshake: 'version'");
    }

    #[test]
    fn positions() {
        // The -180 quirk: 200 -> 20, as the primary computes it.
        assert_eq!(accept(with("lon", json!(200.0))).lon, 20.0);
        assert_eq!(accept(with("lon", json!(360))).lon, 180.0);
        assert_eq!(accept(with("lat", json!("45.5"))).lat, 45.5);
        assert_eq!(accept(with("lat", json!(true))).lat, 1.0);
        assert!(deny(with("lat", json!(90.5))).contains("invalid latitude"));
        assert!(deny(with("lon", json!(-180.1))).contains("invalid longitude"));
        assert!(deny(with("lon", json!(360.5))).contains("invalid longitude"));
        assert!(deny(with("alt", json!(10001))).contains("invalid altitude"));
        assert!(deny(with("alt", json!(-1000.5))).contains("invalid altitude"));
        assert!(deny(with("lat", json!("north"))).contains("could not convert string to float"));
        assert!(deny(with("lat", json!("nan"))).contains("invalid latitude"));
        assert_eq!(deny(without("alt")), "Missing field in handshake: 'alt'");
        assert!(matches!(hs(with("lat", json!(null))), Outcome::Fatal(_)));
    }

    #[test]
    fn user_names() {
        assert_eq!(accept(with("user", json!("Zürich feeder!"))).user, "Z_rich_feeder_");
        assert_eq!(accept(with("user", json!("a".repeat(50)))).user, "a".repeat(40));
        assert_eq!(accept(with("user", json!(12345))).user, "12345");
        assert_eq!(accept(with("user", json!(null))).user, "None");
        let short = accept(with("user", json!("ab"))).user;
        assert!(short.len() == 5 && short.starts_with("ab_"), "{short}");
        let n: u32 = short[3..].parse().unwrap();
        assert!((10..99).contains(&n));
        assert_eq!(deny(without("user")), "Missing field in handshake: 'user'");
    }

    #[test]
    fn placeholder_and_non_string_uuids_are_no_uuid() {
        // PR #14's test vectors.
        for u in ["XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX", "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx", "00000000-0000-0000-0000-000000000000", "", " - "] {
            assert!(is_placeholder_uuid(u), "{u}");
            let a = accept(with("uuid", json!(u)));
            assert_eq!(a.uuid, None);
            assert_eq!(a.notes, vec![PLACEHOLDER_NOTE]);
        }
        for u in ["6a9423e0-7ba6-4672-81a4-81b715ac1ad2", "<28710a05-d56e-4ca1-8b1b-12f6d88d2c0f", "b2a3a57040f17a3d", "3f9c‑a1b2‑4d5e‑7f90‑123456789abc"] {
            assert!(!is_placeholder_uuid(u), "{u}");
            assert_eq!(accept(with("uuid", json!(u))).uuid.as_deref(), Some(u));
        }
        let a = accept(with("uuid", json!(["6a9423e0-7ba6-4672-81a4-81b715ac1ad2"])));
        assert_eq!(a.uuid, None);
        assert!(a.notes.is_empty());
        assert_eq!(accept(with("uuid", json!(null))).uuid, None);
        assert_eq!(accept(without("uuid")).uuid, None);
    }

    #[test]
    fn options() {
        assert_eq!(accept(with("clock_type", json!("radarcape_gps"))).clock, ClockType::RadarcapeGps);
        assert_eq!(accept(without("clock_type")).clock, ClockType::Dump1090);
        assert!(matches!(hs(with("clock_type", json!("rtlsdr"))), Outcome::Fatal(_)));
        assert!(matches!(hs(with("clock_type", json!(null))), Outcome::Fatal(_)));
        let mut v = with("return_results", json!(true));
        assert!(accept(v.clone()).return_results);
        v["return_result_format"] = json!("ecef");
        assert!(accept(v.clone()).return_results);
        v["return_result_format"] = json!("xml");
        assert!(deny(v).contains("invalid return_result_format"));
        assert!(accept(with("privacy", json!(1))).privacy);
        assert!(accept(with("privacy", json!("false"))).privacy); // Python truthiness
        assert_eq!(accept(with("return_stats", json!(true))).return_stats, json!(true));
    }

    #[test]
    fn not_json_or_not_ascii_is_denied_not_an_object_is_fatal() {
        assert!(matches!(evaluate(b"hello\n"), Outcome::Deny(r) if r.starts_with("Badly formatted handshake")));
        assert!(matches!(evaluate(b"\n"), Outcome::Deny(_)));
        assert!(matches!(evaluate("{\"user\":\"é\"}\n".as_bytes()), Outcome::Deny(_)));
        assert!(matches!(evaluate(b"[1,2]\n"), Outcome::Fatal(_)));
        assert!(matches!(evaluate(b"\"x\"\n"), Outcome::Fatal(_)));
    }

    #[test]
    fn proxy_lines() {
        assert_eq!(parse_proxy(b"PROXY TCP4 203.0.113.7 10.0.0.1 51234 30005\r\n").as_deref(), Some("203.0.113.7"));
        assert_eq!(parse_proxy(b"PROXY TCP6 2001:db8::1 2001:db8::2 51234 30005\r\n").as_deref(), Some("2001:db8::1"));
        assert_eq!(parse_proxy(b"PROXY UNKNOWN\r\n"), None);
        assert_eq!(parse_proxy(b"PROXY TCP4 1.2.3.4 5.6.7.8\r\n"), None);
    }

    #[test]
    fn replies() {
        let cfg = ListenConfig { shadow: true, motd: "Ciao".into(), source_url: "https://example.org/mlat-ng".into() };
        let a = accept(with("uuid", json!("XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX")));
        let line = accept_line(&a, &cfg);
        let v: Value = serde_json::from_str(&line).unwrap();
        assert_eq!(v["compress"], "zlib2");
        assert_eq!(v["reconnect_in"], 10);
        assert_eq!(v["return_results"], false);
        assert_eq!(v["forward_all_ac"], false);
        let motd = v["motd"].as_str().unwrap();
        assert!(motd.starts_with("\n\n        Ciao\n        \n        Your MLAT client sends the placeholder uuid"));
        assert!(motd.ends_with("location: https://example.org/mlat-ng\n        "));
        // Key order as mlat-server writes it.
        let keys = ["compress", "reconnect_in", "selective_traffic", "heartbeat", "return_results", "return_stats", "rate_reports", "forward_all_ac", "motd"];
        let pos: Vec<usize> = keys.iter().map(|k| line.find(&format!("\"{k}\"")).unwrap()).collect();
        assert!(pos.windows(2).all(|w| w[0] < w[1]));
        let d: Value = serde_json::from_str(&deny_line("Bad \"values\"")).unwrap();
        assert_eq!(d, json!({"deny": ["Bad \"values\""], "reconnect_in": 900}));
    }
}
