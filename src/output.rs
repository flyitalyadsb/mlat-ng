//! Outputs: BaseStation (SBS) lines over TCP, and the HTTP status server.

use std::fmt::Write as _;
use std::sync::Arc;
use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio::sync::broadcast;

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

/// Minimal HTTP/1.0 server: GET /metrics, /healthz, /state.json. The render
/// callback is called per request on a worker thread (cheap: it formats
/// counters that the engine publishes once a second).
pub async fn http_server(
    listener: TcpListener,
    render: Arc<dyn Fn(&str) -> Option<(&'static str, String)> + Send + Sync>,
) {
    loop {
        let Ok((mut sock, _)) = listener.accept().await else { continue };
        let render = render.clone();
        tokio::spawn(async move {
            let mut buf = vec![0u8; 4096];
            let n = match tokio::time::timeout(Duration::from_secs(5), sock.read(&mut buf)).await {
                Ok(Ok(n)) if n > 0 => n,
                _ => return,
            };
            let req = String::from_utf8_lossy(&buf[..n]);
            let path = req.split_whitespace().nth(1).unwrap_or("/").split('?').next().unwrap_or("/").to_string();
            let (status, ctype, body) = match render(&path) {
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
