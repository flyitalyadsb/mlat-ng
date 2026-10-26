//! Outputs: BaseStation (SBS) lines over TCP, and the HTTP status server.

use std::fmt::Write as _;
use std::sync::Arc;
use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio::sync::{broadcast, mpsc, oneshot};

use crate::engine::{cov_id_of, CovQuery, CovWhat};

/// Unix seconds -> ("YYYY/MM/DD", "HH:MM:SS.mmm"), UTC, as mlat-server writes.
pub fn sbs_datetime(t: f64) -> (String, String) {
    let ms_total = (t * 1000.0).round() as i64;
    let secs = ms_total.div_euclid(1000);
    let ms = ms_total.rem_euclid(1000);
    let days = secs.div_euclid(86_400);
    let sod = secs.rem_euclid(86_400);
    // civil_from_days (Howard Hinnant), proleptic Gregorian.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    (
        format!("{y:04}/{m:02}/{d:02}"),
        format!("{:02}:{:02}:{:02}.{ms:03}", sod / 3600, (sod / 60) % 60, sod % 60),
    )
}

pub struct SbsFix<'a> {
    /// "4CA2D1" for Mode S, "~F00012" for synthetic Mode A/C tracks.
    pub addr: &'a str,
    pub t_fix: f64,
    pub t_now: f64,
    pub callsign: Option<&'a str>,
    pub alt_ft: Option<i32>,
    pub speed_kt: Option<f64>,
    pub track_deg: Option<f64>,
    pub lat: f64,
    pub lon: f64,
    pub vrate_fpm: Option<f64>,
    pub squawk: Option<u16>,
    pub receivers: usize,
    pub err_m: Option<f64>,
}

/// A position as mlat-server sends it back to the receivers that heard it
/// (jsonclient.py report_mlat_position_old / _ecef).
pub struct ClientResult<'a> {
    pub addr: u32,
    /// Wall time of the multilaterated message, unix seconds.
    pub t: f64,
    pub ecef: [f64; 3],
    pub lat: f64,
    pub lon: f64,
    /// Height above the ellipsoid, metres.
    pub alt_m: f64,
    /// ECEF position covariance, m^2.
    pub cov: Option<[[f64; 3]; 3]>,
    pub receivers: usize,
    pub distinct: usize,
    /// North / east ground velocity, knots, from a track.
    pub vel_kt: Option<(f64, f64)>,
    pub vrate_fpm: Option<f64>,
    pub callsign: Option<&'a str>,
    pub squawk: Option<u16>,
}

fn round_to(x: f64, d: i32) -> f64 {
    let m = 10f64.powi(d);
    (x * m).round() / m
}

/// `{"result": {...}}` in the "old" layout (lat/lon/alt in feet, dops 0).
pub fn result_old(r: &ClientResult) -> String {
    let v = serde_json::json!({"result": {
        "@": round_to(r.t, 3),
        "addr": format!("{:06x}", r.addr),
        "lat": round_to(r.lat, 5),
        "lon": round_to(r.lon, 5),
        "alt": (r.alt_m / 0.3048).round(),
        "callsign": r.callsign,
        "squawk": r.squawk.map(|q| format!("{q:04}")),
        "hdop": 0.0, "vdop": 0.0, "tdop": 0.0, "gdop": 0.0,
        "nstations": r.receivers,
    }});
    v.to_string()
}

/// `{"result": {...}}` in the "ecef" layout that mlat-client asks for.
pub fn result_ecef(r: &ClientResult) -> String {
    let mut m = serde_json::Map::new();
    m.insert("@".into(), round_to(r.t, 3).into());
    m.insert("addr".into(), format!("{:06x}", r.addr).into());
    m.insert("n".into(), r.receivers.into());
    m.insert("nd".into(), r.distinct.into());
    // Missing 'cov' disconnects client 0.1.7: always present, null if unknown.
    m.insert(
        "cov".into(),
        match r.cov {
            Some(c) => serde_json::json!([c[0][0].round(), c[0][1].round(), c[0][2].round(), c[1][1].round(), c[1][2].round(), c[2][2].round()]),
            None => serde_json::Value::Null,
        },
    );
    if let Some((ns, ew)) = r.vel_kt {
        m.insert("nsvel".into(), round_to(ns, 1).into());
        m.insert("ewvel".into(), round_to(ew, 1).into());
    }
    if let Some(v) = r.vrate_fpm {
        m.insert("vrate".into(), v.round().into());
    }
    m.insert("ecef".into(), serde_json::json!([r.ecef[0].round(), r.ecef[1].round(), r.ecef[2].round()]));
    serde_json::json!({ "result": m }).to_string()
}

/// One MSG,3 line in exactly mlat-server's BasestationClient layout.
pub fn sbs_line(f: &SbsFix) -> String {
    let (rd, rt) = sbs_datetime(f.t_fix);
    let (nd, nt) = sbs_datetime(f.t_now);
    let mut s = String::with_capacity(160);
    let _ = write!(s, "MSG,3,1,1,{},1,{rd},{rt},{nd},{nt},", f.addr);
    if let Some(c) = f.callsign {
        s.push_str(c);
    }
    s.push(',');
    if let Some(a) = f.alt_ft {
        let _ = write!(s, "{a}");
    }
    s.push(',');
    if let Some(v) = f.speed_kt {
        let _ = write!(s, "{}", v.round() as i64);
    }
    s.push(',');
    if let Some(v) = f.track_deg {
        let _ = write!(s, "{}", v.round() as i64);
    }
    let _ = write!(s, ",{:.6},{:.6},", f.lat, f.lon);
    if let Some(v) = f.vrate_fpm {
        let _ = write!(s, "{}", v.round() as i64);
    }
    s.push(',');
    if let Some(q) = f.squawk {
        let _ = write!(s, "{q:04}");
    }
    let _ = write!(s, ",{},", f.receivers);
    if let Some(e) = f.err_m {
        let _ = write!(s, "{}", e.round() as i64);
    }
    s.push_str(",,\n");
    s
}

/// Serve SBS lines to every connected client; slow clients skip lines.
pub async fn sbs_server(listener: TcpListener, tx: broadcast::Sender<Arc<str>>) {
    loop {
        let Ok((mut sock, _)) = listener.accept().await else { continue };
        let mut rx = tx.subscribe();
        tokio::spawn(async move {
            let _ = sock.set_nodelay(true);
            let mut hb = tokio::time::interval(Duration::from_secs(30));
            hb.tick().await;
            let mut buf = [0u8; 256];
            loop {
                tokio::select! {
                    line = rx.recv() => match line {
                        Ok(l) => if sock.write_all(l.as_bytes()).await.is_err() { return },
                        Err(broadcast::error::RecvError::Lagged(_)) => continue,
                        Err(_) => return,
                    },
                    _ = hb.tick() => if sock.write_all(b"\n").await.is_err() { return },
                    r = sock.read(&mut buf) => match r { Ok(0) | Err(_) => return, Ok(_) => {} },
                }
            }
        });
    }
}

/// `%XX` escapes in a path segment (user names may hold spaces and more).
fn percent_decode(s: &str) -> Option<String> {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%' {
            let h = std::str::from_utf8(b.get(i + 1..i + 3)?).ok()?;
            out.push(u8::from_str_radix(h, 16).ok()?);
            i += 3;
        } else {
            out.push(b[i]);
            i += 1;
        }
    }
    String::from_utf8(out).ok()
}

/// The coverage question a path asks, if any:
///   /coverage/uuid/<uuid>     one receiver's map, by its UUID
///   /coverage/user/<name>     one receiver's map, by its MLAT user name
///                             (percent-encoded), for receivers without UUID
///   /coverage/summary.json    every receiver's daily facts
/// Internal only: the HTTP port is not exposed outside the cluster.
pub fn coverage_route(path: &str) -> Option<CovWhat> {
    let rest = path.strip_prefix("/coverage/")?;
    if rest == "summary.json" {
        return Some(CovWhat::Summary);
    }
    if let Some(u) = rest.strip_prefix("uuid/") {
        let k = crate::relay::registry::key_of_uuid(&percent_decode(u)?)?;
        return Some(CovWhat::Map(k));
    }
    let user = percent_decode(rest.strip_prefix("user/")?)?;
    (!user.is_empty()).then(|| CovWhat::Map(cov_id_of(None, &user)))
}

/// Minimal HTTP/1.0 server: GET /metrics, /healthz, /state.json. The render
/// callback is called per request on a worker thread (cheap: it formats
/// counters that the engine publishes once a second). /coverage/* is asked
/// to the engine thread through `queries`, when given.
pub async fn http_server(
    listener: TcpListener,
    render: Arc<dyn Fn(&str) -> Option<(&'static str, String)> + Send + Sync>,
    queries: Option<mpsc::Sender<CovQuery>>,
) {
    loop {
        let Ok((mut sock, _)) = listener.accept().await else { continue };
        let render = render.clone();
        let queries = queries.clone();
        tokio::spawn(async move {
            let mut buf = vec![0u8; 4096];
            let n = match tokio::time::timeout(Duration::from_secs(5), sock.read(&mut buf)).await {
                Ok(Ok(n)) if n > 0 => n,
                _ => return,
            };
            let req = String::from_utf8_lossy(&buf[..n]);
            let path = req.split_whitespace().nth(1).unwrap_or("/").split('?').next().unwrap_or("/").to_string();
            let answer = match (&queries, path.starts_with("/coverage/")) {
                (Some(q), true) => ask_coverage(q, &path).await,
                _ => render(&path),
            };
            let (status, ctype, body) = match answer {
                Some((ct, b)) => ("200 OK", ct, b),
                None => ("404 Not Found", "text/plain", "not found\n".to_string()),
            };
            let head = format!(
                "HTTP/1.0 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                body.len()
            );
            let _ = sock.write_all(head.as_bytes()).await;
            let _ = sock.write_all(body.as_bytes()).await;
        });
    }
}

async fn ask_coverage(q: &mpsc::Sender<CovQuery>, path: &str) -> Option<(&'static str, String)> {
    let what = coverage_route(path)?;
    let (tx, rx) = oneshot::channel();
    q.send(CovQuery { what, reply: tx }).await.ok()?;
    let body = tokio::time::timeout(Duration::from_secs(5), rx).await.ok()?.ok()??;
    Some(("application/json", body))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coverage_routes() {
        assert!(matches!(coverage_route("/coverage/summary.json"), Some(CovWhat::Summary)));
        match coverage_route("/coverage/uuid/01234567-89ab-cdef-0123-456789abcdef") {
            Some(CovWhat::Map(k)) => assert_eq!(k, 0x0123_4567_89ab_cdef),
            _ => panic!("uuid route"),
        }
        match coverage_route("/coverage/user/My%20Station-1") {
            Some(CovWhat::Map(k)) => assert_eq!(k, cov_id_of(None, "My Station-1")),
            _ => panic!("user route"),
        }
        assert!(coverage_route("/coverage/uuid/not-a-uuid").is_none());
        assert!(coverage_route("/coverage/user/").is_none());
        assert!(coverage_route("/coverage/user/bad%zz").is_none());
        assert!(coverage_route("/coverage/other").is_none());
        assert!(coverage_route("/metrics").is_none());
    }

    #[test]
    fn datetime_matches_known_epochs() {
        assert_eq!(sbs_datetime(0.0), ("1970/01/01".into(), "00:00:00.000".into()));
        // 2026-10-24 01:02:03.456 UTC
        assert_eq!(sbs_datetime(1_792_803_723.456), ("2026/10/24".into(), "01:02:03.456".into()));
        assert_eq!(sbs_datetime(951_782_400.0).0, "2000/02/29");
    }

    #[test]
    fn line_layout_matches_mlat_server() {
        let l = sbs_line(&SbsFix {
            addr: "4CA2D1",
            t_fix: 0.5,
            t_now: 1.0,
            callsign: Some("RYR1"),
            alt_ft: Some(35000),
            speed_kt: Some(451.4),
            track_deg: Some(90.0),
            lat: 45.0,
            lon: 9.5,
            vrate_fpm: None,
            squawk: Some(1200),
            receivers: 5,
            err_m: Some(120.4),
        });
        assert_eq!(
            l,
            "MSG,3,1,1,4CA2D1,1,1970/01/01,00:00:00.500,1970/01/01,00:00:01.000,RYR1,35000,451,90,45.000000,9.500000,,1200,5,120,,\n"
        );
        // 22 comma-separated fields like mlat-server's TEMPLATE
        assert_eq!(l.trim_end().split(',').count(), 22);
    }

    #[test]
    fn client_results_parse_as_mlat_client_expects() {
        let r = ClientResult {
            addr: 0x4ca1b2,
            t: 1_792_858_311.12345,
            ecef: [4_400_000.4, 700_000.6, 4_500_000.0],
            lat: 45.123456,
            lon: 9.123456,
            alt_m: 10_000.0,
            cov: Some([[100.0, 1.0, 2.0], [1.0, 200.0, 3.0], [2.0, 3.0, 300.0]]),
            receivers: 6,
            distinct: 5,
            vel_kt: Some((300.04, -120.06)),
            vrate_fpm: Some(-640.4),
            callsign: Some("AZA123"),
            squawk: Some(1200),
        };
        // mlat-client jsonclient.handle_connected_request, "ecef" branch.
        let v: serde_json::Value = serde_json::from_str(&result_ecef(&r)).unwrap();
        let x = &v["result"];
        assert_eq!(x["addr"], "4ca1b2");
        assert_eq!(x["@"], 1_792_858_311.123);
        assert_eq!(x["n"], 6);
        assert_eq!(x["nd"], 5);
        assert_eq!(x["cov"].as_array().unwrap().len(), 6);
        assert_eq!(x["ecef"][0], 4_400_000.0);
        assert_eq!(x["nsvel"], 300.0);
        assert_eq!(x["ewvel"], -120.1);
        assert_eq!(x["vrate"], -640.0);
        // "old" branch: lat/lon/alt/gdop/nstations/callsign/squawk.
        let v: serde_json::Value = serde_json::from_str(&result_old(&r)).unwrap();
        let x = &v["result"];
        assert_eq!(x["lat"], 45.12346);
        assert_eq!(x["alt"], 32808.0);
        assert_eq!(x["gdop"], 0.0);
        assert_eq!(x["nstations"], 6);
        assert_eq!(x["callsign"], "AZA123");
        assert_eq!(x["squawk"], "1200");
        // Without covariance the key is still there (client 0.1.7).
        let v: serde_json::Value = serde_json::from_str(&result_ecef(&ClientResult { cov: None, vel_kt: None, vrate_fpm: None, ..r })).unwrap();
        assert!(v["result"]["cov"].is_null());
        assert!(v["result"].get("nsvel").is_none());
    }
}
