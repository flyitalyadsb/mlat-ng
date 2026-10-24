//! The few Python builtins mlat-server applies to client JSON, ported so the
//! handshake and the messages are judged exactly as the primary judges them:
//! `float(x)`, `str(x)`, `bool(x)`, `int(s, 16)` and `bytes.fromhex(s)`.
//!
//! The values come from serde_json; mlat-server parses with ujson, which
//! agrees with it on everything a real client sends (int vs float numbers,
//! last-wins duplicate keys, trailing whitespace).

use serde_json::Value;

/// How a Python builtin failed. mlat-server catches ValueError (and KeyError)
/// in the handshake and turns them into a `deny`; a TypeError escapes and the
/// connection is closed without a reply.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum PyErr {
    Value(String),
    Type,
}

/// `float(x)` for a JSON value: numbers and bools convert, strings are parsed
/// with Python's float syntax, null / list / dict are a TypeError.
pub(crate) fn float(v: &Value) -> Result<f64, PyErr> {
    match v {
        // u64 -> f64 rounds to nearest-even, exactly like float(int).
        Value::Number(n) => n.as_f64().ok_or(PyErr::Type),
        Value::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        Value::String(s) => float_str(s).ok_or_else(|| PyErr::Value(format!("could not convert string to float: {}", repr_str(s)))),
        _ => Err(PyErr::Type),
    }
}

/// Python's float(str): surrounding whitespace ignored, "inf" / "infinity" /
/// "nan" in any case, and single underscores between digits ("1_000.5").
fn float_str(s: &str) -> Option<f64> {
    let t = s.trim();
    if t.contains('_') {
        let b = t.as_bytes();
        for (i, &c) in b.iter().enumerate() {
            if c == b'_' && !(i > 0 && b[i - 1].is_ascii_digit() && b.get(i + 1).is_some_and(|d| d.is_ascii_digit())) {
                return None;
            }
        }
        return t.replace('_', "").parse().ok();
    }
    // Rust's grammar is Python's minus the underscores: both take "1.", ".5",
    // "1e5", signs, and the inf/nan words case-insensitively.
    t.parse().ok()
}

/// `bool(x)`: Python truthiness, with a missing key counting as False (every
/// caller does `bool(hs.get(key, False))`).
pub(crate) fn truthy(v: Option<&Value>) -> bool {
    match v {
        None | Some(Value::Null) => false,
        Some(Value::Bool(b)) => *b,
        Some(Value::Number(n)) => n.as_f64().is_none_or(|f| f != 0.0),
        Some(Value::String(s)) => !s.is_empty(),
        Some(Value::Array(a)) => !a.is_empty(),
        Some(Value::Object(o)) => !o.is_empty(),
    }
}

/// `x == n` with Python's numeric equality (2.0 == 2, True == 1).
pub(crate) fn eq_int(v: &Value, n: i64) -> bool {
    match v {
        Value::Number(x) => {
            if let Some(i) = x.as_i64() {
                i == n
            } else if x.is_u64() {
                false // beyond i64, can't equal a small int
            } else {
                x.as_f64() == Some(n as f64)
            }
        }
        Value::Bool(b) => i64::from(*b) == n,
        _ => false,
    }
}

/// `str(x)`. Exact for scalars, which is all a real client ever sends as a
/// user name or clock type; lists and dicts get Python's repr layout (dict
/// keys come back sorted from serde_json, so their order may differ).
pub(crate) fn str(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        _ => repr(v),
    }
}

/// `repr(x)` for a JSON value.
pub(crate) fn repr(v: &Value) -> String {
    match v {
        Value::Null => "None".into(),
        Value::Bool(true) => "True".into(),
        Value::Bool(false) => "False".into(),
        Value::Number(n) => {
            if n.is_f64() {
                float_repr(n.as_f64().unwrap_or(0.0))
            } else {
                n.to_string()
            }
        }
        Value::String(s) => repr_str(s),
        Value::Array(a) => {
            let items: Vec<String> = a.iter().map(repr).collect();
            format!("[{}]", items.join(", "))
        }
        Value::Object(o) => {
            let items: Vec<String> = o.iter().map(|(k, v)| format!("{}: {}", repr_str(k), repr(v))).collect();
            format!("{{{}}}", items.join(", "))
        }
    }
}

/// Python's repr of a str: single quotes unless the text has a single quote
/// and no double quote; backslash, quote and control characters escaped.
fn repr_str(s: &str) -> String {
    let q = if s.contains('\'') && !s.contains('"') { '"' } else { '\'' };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(q);
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if c == q => {
                out.push('\\');
                out.push(c);
            }
            c if (c as u32) < 0x20 || c as u32 == 0x7f => out.push_str(&format!("\\x{:02x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push(q);
    out
}

/// Python's repr(float): the shortest digits that round-trip (Rust finds the
/// same ones), fixed notation for exponents -4..15, scientific otherwise,
/// and always a ".0" on integral values in fixed notation.
pub(crate) fn float_repr(x: f64) -> String {
    if x.is_nan() {
        return "nan".into();
    }
    if x.is_infinite() {
        return if x > 0.0 { "inf".into() } else { "-inf".into() };
    }
    // "{:e}" is "-1.2345e16" / "1e-5": sign, shortest digits, exponent.
    let e = format!("{:e}", x);
    let (mant, exp) = e.split_once('e').unwrap_or((&e, "0"));
    let exp: i32 = exp.parse().unwrap_or(0);
    let (neg, mant) = match mant.strip_prefix('-') {
        Some(m) => (true, m),
        None => (false, mant),
    };
    let digits: String = mant.chars().filter(|c| *c != '.').collect();
    let mut out = String::new();
    if neg {
        out.push('-');
    }
    if (-4..16).contains(&exp) {
        if exp >= 0 {
            let int_len = exp as usize + 1;
            if digits.len() <= int_len {
                out.push_str(&digits);
                out.extend(std::iter::repeat_n('0', int_len - digits.len()));
                out.push_str(".0");
            } else {
                out.push_str(&digits[..int_len]);
                out.push('.');
                out.push_str(&digits[int_len..]);
            }
        } else {
            out.push_str("0.");
            out.extend(std::iter::repeat_n('0', (-exp - 1) as usize));
            out.push_str(&digits);
        }
    } else {
        out.push_str(&digits[..1]);
        if digits.len() > 1 {
            out.push('.');
            out.push_str(&digits[1..]);
        }
        out.push_str(&format!("e{}{:02}", if exp < 0 { '-' } else { '+' }, exp.abs()));
    }
    out
}

/// `int(s, 16)` restricted to what fits an ICAO-ish u32: whitespace around,
/// an optional '+', an optional 0x prefix, single underscores between digits.
/// Negative or larger values are None (mlat-server would accept them and then
/// track an address no aircraft has).
pub(crate) fn int16(s: &str) -> Option<u32> {
    let t = s.trim();
    let t = t.strip_prefix('+').unwrap_or(t);
    let body = match t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        // Python allows one underscore right after the prefix: 0x_4c.
        Some(r) => r.strip_prefix('_').unwrap_or(r),
        None => t,
    };
    let b = body.as_bytes();
    if b.is_empty() || b[0] == b'_' || b[b.len() - 1] == b'_' {
        return None;
    }
    let mut v: u64 = 0;
    let mut prev_us = false;
    for &c in b {
        if c == b'_' {
            if prev_us {
                return None;
            }
            prev_us = true;
            continue;
        }
        prev_us = false;
        let d = (c as char).to_digit(16)?;
        v = v * 16 + u64::from(d);
        if v > u64::from(u32::MAX) {
            return None;
        }
    }
    Some(v as u32)
}

/// `bytes.fromhex(s)`: pairs of hex digits, ASCII whitespace allowed between
/// (not inside) pairs.
pub(crate) fn fromhex(s: &str) -> Option<Vec<u8>> {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len() / 2);
    let mut i = 0;
    let ws = |c: u8| matches!(c, b' ' | b'\t' | b'\n' | b'\r' | b'\x0b' | b'\x0c');
    while i < b.len() {
        if ws(b[i]) {
            i += 1;
            continue;
        }
        let hi = (b[i] as char).to_digit(16)?;
        let lo = (*b.get(i + 1)? as char).to_digit(16)?;
        out.push((hi * 16 + lo) as u8);
        i += 2;
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn float_matches_python() {
        assert_eq!(float(&json!(45)), Ok(45.0));
        assert_eq!(float(&json!(true)), Ok(1.0));
        assert_eq!(float(&json!(" 45.5 ")), Ok(45.5));
        assert_eq!(float(&json!("1_0.5")), Ok(10.5));
        assert!(float(&json!("nan")).unwrap().is_nan());
        assert_eq!(float(&json!("-Infinity")), Ok(f64::NEG_INFINITY));
        assert!(matches!(float(&json!("4__5")), Err(PyErr::Value(_))));
        assert!(matches!(float(&json!("abc")), Err(PyErr::Value(_))));
        assert_eq!(float(&json!(null)), Err(PyErr::Type));
        assert_eq!(float(&json!([1])), Err(PyErr::Type));
        // float(int) rounds big ints to nearest-even like CPython.
        assert_eq!(float(&json!(9007199254740993u64)), Ok(9007199254740992.0));
    }

    #[test]
    fn str_and_repr_match_python() {
        assert_eq!(str(&json!("abc")), "abc");
        assert_eq!(str(&json!(123)), "123");
        assert_eq!(str(&json!(-7)), "-7");
        assert_eq!(str(&json!(1.5)), "1.5");
        assert_eq!(str(&json!(true)), "True");
        assert_eq!(str(&json!(null)), "None");
        // Same text as CPython's str() of the equivalent list.
        assert_eq!(str(&json!(["a\nb", 1.0, null, true, {"k": "v"}])), "['a\\nb', 1.0, None, True, {'k': 'v'}]");
        assert_eq!(repr(&json!("it's")), "\"it's\"");
    }

    #[test]
    fn float_repr_matches_python() {
        // Expected values printed by CPython 3.8's repr().
        let cases = [
            (1e16, "1e+16"),
            (1.5e-5, "1.5e-05"),
            (123.0, "123.0"),
            (0.0001, "0.0001"),
            (1e-5, "1e-05"),
            (-0.0, "-0.0"),
            (12345678901234567.0, "1.2345678901234568e+16"),
            (1234567890123456.0, "1234567890123456.0"),
            (45.4642, "45.4642"),
            (1729760000.12, "1729760000.12"),
        ];
        for (x, want) in cases {
            assert_eq!(float_repr(x), want, "{x:e}");
        }
    }

    #[test]
    fn truthiness() {
        assert!(!truthy(None));
        assert!(!truthy(Some(&json!(0))));
        assert!(!truthy(Some(&json!(""))));
        assert!(truthy(Some(&json!("false")))); // a non-empty string is True
        assert!(truthy(Some(&json!([0]))));
        assert!(!truthy(Some(&json!({}))));
    }

    #[test]
    fn int16_matches_python() {
        assert_eq!(int16("4ca123"), Some(0x4ca123));
        assert_eq!(int16("4CA123"), Some(0x4ca123));
        assert_eq!(int16(" 0x4CA123 "), Some(0x4ca123));
        assert_eq!(int16("0x_4c"), Some(0x4c));
        assert_eq!(int16("4c_a1"), Some(0x4ca1));
        for bad in ["4c__a1", "_4c", "4c_", "0x", "", "-1", "xyz", "1ffffffff"] {
            assert_eq!(int16(bad), None, "{bad}");
        }
    }

    #[test]
    fn fromhex_matches_python() {
        assert_eq!(fromhex("8d4ca1"), Some(vec![0x8d, 0x4c, 0xa1]));
        assert_eq!(fromhex("8D 4c\ta1\n"), Some(vec![0x8d, 0x4c, 0xa1]));
        assert_eq!(fromhex("8d4 c"), None);
        assert_eq!(fromhex("8d4"), None);
        assert_eq!(fromhex(""), Some(vec![]));
    }

    #[test]
    fn eq_int_is_python_equality() {
        assert!(eq_int(&json!(2), 2));
        assert!(eq_int(&json!(2.0), 2));
        assert!(!eq_int(&json!(true), 2));
        assert!(eq_int(&json!(true), 1));
        assert!(!eq_int(&json!("2"), 2));
        assert!(!eq_int(&json!(18446744073709551615u64), 3));
    }
}
