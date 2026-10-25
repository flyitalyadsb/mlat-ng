//! Mode S / ADS-B / Mode A/C frame decoding. Pure functions, no I/O.
//!
//! Semantics follow the production mlat-server (`modes_cython/message.pyx`)
//! so that both servers read a frame the same way: which DFs are decoded,
//! when the CRC counts as checked, how the address is obtained, the ES type
//! code / NUCp table and the CPR global decode with its zone checks. The one
//! deliberate difference is Gillham (Q=0) altitudes, where mlat-server has a
//! long-standing bug; see `gillham_ac13`.
//!
//! Bit numbering in comments is the ICAO one: bit 1 is the first bit sent.

#[cfg(test)]
mod tests;

pub mod commb;

/// Downlink formats we decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EsKind {
    AirbornePosition,
    SurfacePosition,
    Identification,
    Velocity,
    Other,
}

/// Fields of a DF17/DF18 extended squitter that the server uses.
#[derive(Debug, Clone, PartialEq)]
pub struct Es {
    pub kind: EsKind,
    /// ME type code (0..31; 0 is "no position", which mlat-server files under
    /// airborne position with NUCp 0).
    pub tc: u8,
    /// NUCp for position messages, mlat-server's table: TC 0 -> 0, surface
    /// TC 5..8 -> 9..6, airborne TC 9..18 -> 9..0, TC 20 -> 9, 21 -> 8,
    /// 22 -> 0. None otherwise.
    pub nuc: Option<u8>,
    /// CPR format bit (false = even, true = odd), position messages only.
    pub odd: bool,
    /// 17-bit CPR latitude / longitude, position messages only (airborne and
    /// surface share the ME layout, so both are filled; mlat-server only
    /// reads them for airborne).
    pub lat_cpr: u32,
    pub lon_cpr: u32,
}

/// A decoded frame. `address` follows mlat-server's semantics: the AA field for
/// DF11/17/18, and the address recovered from the AP (CRC XOR) field for
/// DF0/4/5/16/20/21, where it is only as good as the frame.
#[derive(Debug, Clone, PartialEq)]
pub struct Decoded {
    pub df: u8,
    pub address: u32,
    /// Some(true/false) when the CRC can be checked (DF11 with IID 0, DF17,
    /// DF18), None when the CRC is overlaid by the address (AP formats).
    pub crc_ok: Option<bool>,
    /// Barometric altitude in feet from AC13 (DF0/4/16/20) or AC12 (ES
    /// airborne position); None when absent, zero or invalid. For ES TC 20..22
    /// the field is GNSS height, decoded the same way, as mlat-server does.
    pub altitude_ft: Option<i32>,
    /// Squawk as a 4-digit octal-looking number, e.g. 7000 (DF5/21).
    pub squawk: Option<u16>,
    /// Callsign with its space padding removed: from ES identification
    /// (TC 1..4), and from a DF20/21 Comm-B reply carrying BDS 2,0 when it
    /// passes mlat-server's sanity filter. None when it would be empty.
    pub callsign: Option<String>,
    pub es: Option<Es>,
}

/// Decode a 7- or 14-byte Mode S frame. Returns None for unknown DF, wrong
/// length for the DF, or malformed input. Must never panic on any input.
pub fn decode(frame: &[u8]) -> Option<Decoded> {
    let df = frame.first()? >> 3;
    if frame.len() != frame_len(df) {
        return None;
    }
    let f = frame;
    let mut d = Decoded { df, address: 0, crc_ok: None, altitude_ft: None, squawk: None, callsign: None, es: None };
    // AC13 (altitude) and ID13 (identity) sit in the same place, bits 20..32.
    let field13 = (u16::from(f[2] & 0x1f) << 8) | u16::from(f[3]);
    match df {
        0 | 4 | 16 | 20 => {
            d.altitude_ft = decode_ac13(field13);
            d.address = crc_residual(f);
        }
        5 | 21 => {
            d.squawk = Some(decode_id13(field13));
            d.address = crc_residual(f);
        }
        11 => {
            d.address = u24(&f[1..4]);
            // PI = CRC xor the interrogator's II/SI code (7 bits). Zero is an
            // acquisition squitter or an II=0 reply: checked. A residual that
            // fits in 7 bits is a clean reply to interrogator `r`, or an error
            // that happens to look like one: unknown, as in mlat-server.
            let r = crc_residual(f);
            d.crc_ok = if r == 0 {
                Some(true)
            } else if r & !0x7f == 0 {
                None
            } else {
                Some(false)
            };
        }
        17 | 18 => {
            // DF18 is decoded exactly like DF17 whatever its CF, as
            // mlat-server does (CF only matters to TIS-B/ADS-R consumers).
            d.address = u24(&f[1..4]);
            d.crc_ok = Some(crc_residual(f) == 0);
            decode_es(&mut d, &f[4..11]);
        }
        _ => return None,
    }
    if df == 20 || df == 21 {
        d.callsign = comm_b_callsign(&f[4..11]);
    }
    Some(d)
}

/// DF0..15 are 56-bit frames, DF16..31 112-bit: the first DF bit is the
/// length bit.
fn frame_len(df: u8) -> usize {
    if df >= 16 {
        14
    } else {
        7
    }
}

/// Big-endian 24-bit value from the first 3 bytes (callers pass >= 3).
fn u24(b: &[u8]) -> u32 {
    (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2])
}

// ---------------------------------------------------------------------------
// Extended squitter

/// ES type code -> (kind, NUCp), mlat-server's `es_types` table verbatim.
/// TC 22 is NUCp 0 (DO-260: horizontal protection limit >= 30 m), not 8.
fn es_type(tc: u8) -> (EsKind, Option<u8>) {
    match tc {
        0 => (EsKind::AirbornePosition, Some(0)),
        1..=4 => (EsKind::Identification, None),
        5..=8 => (EsKind::SurfacePosition, Some(14 - tc)),
        9..=18 => (EsKind::AirbornePosition, Some(18 - tc)),
        19 => (EsKind::Velocity, None),
        20 => (EsKind::AirbornePosition, Some(9)),
        21 => (EsKind::AirbornePosition, Some(8)),
        22 => (EsKind::AirbornePosition, Some(0)),
        _ => (EsKind::Other, None),
    }
}

/// Fill the ES fields from the 7-byte ME field.
fn decode_es(d: &mut Decoded, me: &[u8]) {
    let tc = me[0] >> 3;
    let (kind, nuc) = es_type(tc);
    let mut es = Es { kind, tc, nuc, odd: false, lat_cpr: 0, lon_cpr: 0 };
    match kind {
        EsKind::AirbornePosition | EsKind::SurfacePosition => {
            // ME bit 22 = F, 23..39 = lat, 40..56 = lon in both formats.
            es.odd = me[2] & 0x04 != 0;
            es.lat_cpr = (u32::from(me[2] & 0x03) << 15) | (u32::from(me[3]) << 7) | u32::from(me[4] >> 1);
            es.lon_cpr = (u32::from(me[4] & 0x01) << 16) | (u32::from(me[5]) << 8) | u32::from(me[6]);
            if kind == EsKind::AirbornePosition {
                // ME bits 9..20; on the surface those bits are movement/track.
                let ac12 = (u16::from(me[1]) << 4) | u16::from(me[2] >> 4);
                d.altitude_ft = decode_ac12(ac12);
            }
        }
        EsKind::Identification => {
            // Unfiltered, like mlat-server: a '?' from an out-of-charset code
            // is kept, the CRC already vouches for the frame.
            d.callsign = unpad(&ais_chars(&me[1..7]));
        }
        EsKind::Velocity | EsKind::Other => {}
    }
    d.es = Some(es);
}

/// Geometric minus barometric altitude (ft) from a DF17/18 airborne velocity
/// message (TC 19, subtypes 1-4): ME bits 49-56, i.e. frame byte 10, a sign
/// bit (set = geometric below baro) and (value - 1) x 25 ft; 0 = no data.
/// The geometric altitude is the GNSS height above the WGS84 ellipsoid
/// (DO-260A/B). None for anything else; checks the CRC itself.
pub fn es_geo_minus_baro_ft(f: &[u8]) -> Option<i32> {
    if f.len() != 14 || !matches!(f[0] >> 3, 17 | 18) || crc_residual(f) != 0 {
        return None;
    }
    if f[4] >> 3 != 19 || !(1..=4).contains(&(f[4] & 7)) {
        return None;
    }
    let v = i32::from(f[10] & 0x7f);
    if v == 0 {
        return None;
    }
    Some(if f[10] & 0x80 != 0 { -(v - 1) * 25 } else { (v - 1) * 25 })
}

/// mlat-server's 6-bit character set. Index 0 maps to a space (not the usual
/// '#') and every unassigned code to '?', so the Comm-B filter can spot them.
const AIS_CHARSET: &[u8; 64] = b" ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

/// Eight 6-bit characters packed in 6 bytes.
fn ais_chars(b: &[u8]) -> [u8; 8] {
    let v = b.iter().take(6).fold(0u64, |acc, &x| (acc << 8) | u64::from(x));
    let mut out = [b' '; 8];
    for (i, c) in out.iter_mut().enumerate() {
        *c = AIS_CHARSET[((v >> (42 - 6 * i)) & 0x3f) as usize];
    }
    out
}

/// Drop the trailing space padding; None when nothing is left.
fn unpad(chars: &[u8; 8]) -> Option<String> {
    let s = std::str::from_utf8(chars).ok()?.trim_end();
    (!s.is_empty()).then(|| s.to_owned())
}

/// Callsign from a Comm-B MB field holding BDS 2,0 (aircraft identification).
/// The MB field has no register number on the wire: mlat-server takes the
/// 0x20 header byte as the signature and rejects blank or '?'-carrying
/// results, which weeds out other registers that happen to start with 0x20.
fn comm_b_callsign(mb: &[u8]) -> Option<String> {
    if mb.first() != Some(&0x20) {
        return None;
    }
    let chars = ais_chars(&mb[1..]);
    if chars.contains(&b'?') {
        return None;
    }
    unpad(&chars)
}

// ---------------------------------------------------------------------------
// CRC

/// Returned by `crc_residual` when the frame is shorter than its DF needs:
/// not a 24-bit value, so it is never 0 and never equal to an address.
const NO_RESIDUAL: u32 = u32::MAX;

/// Byte-at-a-time table for the Mode S parity generator x^24 + ... (0xFFF409
/// with the x^24 term implicit, Annex 10 §3.1.2.3.3.1), MSB first.
const CRC_TABLE: [u32; 256] = {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = (i as u32) << 16;
        let mut j = 0;
        while j < 8 {
            c = if c & 0x80_0000 != 0 { (c << 1) ^ 0xFF_F409 } else { c << 1 };
            j += 1;
        }
        t[i] = c & 0xFF_FFFF;
        i += 1;
    }
    t
};

/// CRC-24 residual of a Mode S frame (0 for a valid DF11/17/18 frame; the
/// address for AP formats with a clean frame).
///
/// As in mlat-server, the length comes from the DF (7 bytes below DF16, 14
/// from DF16), not from the slice; extra bytes are ignored. A frame shorter
/// than its DF needs returns `u32::MAX`, which is outside 24 bits and so can
/// match neither 0 nor an address.
pub fn crc_residual(frame: &[u8]) -> u32 {
    let Some(&first) = frame.first() else { return NO_RESIDUAL };
    let Some(frame) = frame.get(..frame_len(first >> 3)) else { return NO_RESIDUAL };
    let (data, parity) = frame.split_at(frame.len() - 3);
    let rem = data.iter().fold(0u32, |rem, &b| {
        ((rem & 0xFFFF) << 8) ^ CRC_TABLE[(((rem >> 16) ^ u32::from(b)) & 0xFF) as usize]
    });
    rem ^ u24(parity)
}

// ---------------------------------------------------------------------------
// CPR

/// Latitudes (degrees, ascending) at which the number of longitude zones
/// drops by one: NL is 59 below NL_LAT[0], 59 - i below NL_LAT[i], and 1 from
/// 87 degrees to the pole. Copied from mlat-server rather than computed from
/// the closed form so that a position on the edge (to 1e-8 degrees) lands in
/// the same zone in both servers.
const NL_LAT: [f64; 59] = [
    10.47047130, 14.82817437, 18.18626357, 21.02939493, 23.54504487, 25.82924707, 27.93898710, 29.91135686,
    31.77209708, 33.53993436, 35.22899598, 36.85025108, 38.41241892, 39.92256684, 41.38651832, 42.80914012,
    44.19454951, 45.54626723, 46.86733252, 48.16039128, 49.42776439, 50.67150166, 51.89342469, 53.09516153,
    54.27817472, 55.44378444, 56.59318756, 57.72747354, 58.84763776, 59.95459277, 61.04917774, 62.13216659,
    63.20427479, 64.26616523, 65.31845310, 66.36171008, 67.39646774, 68.42322022, 69.44242631, 70.45451075,
    71.45986473, 72.45884545, 73.45177442, 74.43893416, 75.42056257, 76.39684391, 77.36789461, 78.33374083,
    79.29428225, 80.24923213, 81.19801349, 82.13956981, 83.07199445, 83.99173563, 84.89166191, 85.75541621,
    86.53536998, 87.00000000, 90.00000000,
];

/// Number of longitude zones at `lat`. Strict `<` against each edge, as the
/// Cython `NL()` (the pure-Python bisect version differs exactly on an edge).
fn nl(lat: f64) -> i64 {
    let lat = lat.abs();
    NL_LAT.iter().position(|&edge| lat < edge).map_or(1, |i| 59 - i as i64)
}

/// 2^17: CPR coordinates are 17-bit fractions of a zone.
const CPR_SCALE: f64 = 131072.0;

/// Global airborne CPR decode of an even/odd pair, as mlat-server's
/// `decode_cpr`: returns (even_lat, even_lon, odd_lat, odd_lon) in degrees,
/// or None when the pair straddles a latitude zone boundary / is invalid.
///
/// The arithmetic is written in the same order as mlat-server so results are
/// bit-identical; a coordinate wider than 17 bits is invalid.
pub fn cpr_global_airborne(lat_e: u32, lon_e: u32, lat_o: u32, lon_o: u32) -> Option<(f64, f64, f64, f64)> {
    if [lat_e, lon_e, lat_o, lon_o].iter().any(|&v| v >= 1 << 17) {
        return None;
    }
    let (lat_e, lon_e, lat_o, lon_o) = (f64::from(lat_e), f64::from(lon_e), f64::from(lat_o), f64::from(lon_o));

    // Latitude index j: which of the 60 even / 59 odd zones both lie in.
    let j = ((59.0 * lat_e - 60.0 * lat_o) / CPR_SCALE + 0.5).floor() as i64;
    let mut rlat_e = (360.0 / 60.0) * (j.rem_euclid(60) as f64 + lat_e / CPR_SCALE);
    let mut rlat_o = (360.0 / 59.0) * (j.rem_euclid(59) as f64 + lat_o / CPR_SCALE);
    // Southern hemisphere comes out as 270..360.
    if rlat_e >= 270.0 {
        rlat_e -= 360.0;
    }
    if rlat_o >= 270.0 {
        rlat_o -= 360.0;
    }
    if !(-90.0..=90.0).contains(&rlat_e) || !(-90.0..=90.0).contains(&rlat_o) {
        return None;
    }
    // Both must have the same number of longitude zones, otherwise the pair
    // straddles an NL edge (aircraft crossed it between the two messages).
    let nl_e = nl(rlat_e);
    if nl_e != nl(rlat_o) {
        return None;
    }
    let n_e = nl_e;
    let n_o = (nl_e - 1).max(1);

    let m = ((lon_e * (nl_e - 1) as f64 - lon_o * nl_e as f64) / CPR_SCALE + 0.5).floor() as i64;
    let mut rlon_e = (360.0 / n_e as f64) * (m.rem_euclid(n_e) as f64 + lon_e / CPR_SCALE);
    let mut rlon_o = (360.0 / n_o as f64) * (m.rem_euclid(n_o) as f64 + lon_o / CPR_SCALE);
    // To -180..180.
    rlon_e -= ((rlon_e + 180.0) / 360.0).floor() * 360.0;
    rlon_o -= ((rlon_o + 180.0) / 360.0).floor() * 360.0;
    Some((rlat_e, rlon_e, rlat_o, rlon_o))
}

// ---------------------------------------------------------------------------
// Altitude and identity

/// Gillham-coded 13-bit altitude (AC13, with M and Q bits) to feet.
///
/// MSB first: C1 A1 C2 A2 C4 A4 M B1 Q B2 D2 B4 D4 (Annex 10
/// §3.1.2.6.5.4). Rules as mlat-server: 0 = no altitude, M set (metric) is
/// not decoded, Q set is the 25 ft binary code, Q clear is 100 ft Gillham.
pub fn decode_ac13(ac13: u16) -> Option<i32> {
    let ac13 = ac13 & 0x1fff;
    if ac13 == 0 || ac13 & 0x0040 != 0 {
        return None;
    }
    if ac13 & 0x0010 != 0 {
        // 11-bit count of 25 ft from -1000 ft: the field with M and Q removed.
        let n = ((ac13 & 0x1f80) >> 2) | ((ac13 & 0x0020) >> 1) | (ac13 & 0x000f);
        return Some(i32::from(n) * 25 - 1000);
    }
    gillham_ac13(ac13)
}

/// 100 ft Gillham code (Q = 0). D1 shares the Q position, so it is 0 here and
/// the 500 ft part is the 8-bit Gray code D2 D4 A1 A2 A4 B1 B2 B4; C1 C2 C4 is
/// a Gray code of the 100 ft step inside it, whose only valid values are
/// 001 011 010 110 100 (1..5). The 100 ft count runs backwards on odd 500 ft
/// steps (reflected code: neighbouring altitudes differ in a single bit).
///
/// Deliberately NOT mlat-server: its `_decode_ac13` folds the 100 ft code with
/// `if h & 5: h ^= 5` where Gillham needs `if (h & 5) == 5: h ^= 2` (readsb's
/// `internalModeAToModeC`). That bug, shared with upstream mutability and
/// wiedehopf, gets 1024 of the 1280 valid codes wrong (768 by +-300 ft,
/// 256 rejected) and accepts 511 illegal ones. This implementation agrees
/// with readsb and the published Mode C table on every code (see tests).
fn gillham_ac13(ac13: u16) -> Option<i32> {
    let bit = |n: u16| (ac13 >> n) & 1;
    let g500 = (bit(2) << 7)
        | (bit(0) << 6)
        | (bit(11) << 5)
        | (bit(9) << 4)
        | (bit(7) << 3)
        | (bit(5) << 2)
        | (bit(3) << 1)
        | bit(1);
    let g100 = (bit(12) << 2) | (bit(10) << 1) | bit(8);
    let n500 = i32::from(gray_to_binary(g500));
    let n100 = match gray_to_binary(g100) {
        n @ 1..=4 => i32::from(n),
        7 => 5, // Gray 100
        _ => return None, // 000, 101, 111 are not used
    };
    let n100 = if n500 % 2 == 1 { 6 - n100 } else { n100 };
    Some(n500 * 500 + n100 * 100 - 1300)
}

fn gray_to_binary(mut g: u16) -> u16 {
    let mut b = 0;
    while g != 0 {
        b ^= g;
        g >>= 1;
    }
    b
}

/// 12-bit ES altitude (AC12, Q bit at position 4) to feet.
///
/// AC12 is AC13 without the M bit (Doc 9871 Table A-2-5): put a 0 back in.
pub fn decode_ac12(ac12: u16) -> Option<i32> {
    decode_ac13(((ac12 & 0x0fc0) << 1) | (ac12 & 0x003f))
}

/// 13-bit identity field (DF5/21 ID13) to squawk digits, e.g. 0o7000 -> 7000.
///
/// MSB first: C1 A1 C2 A2 C4 A4 X B1 D1 B2 D2 B4 D4 (Annex 10
/// §3.1.2.6.7.1); each digit is its 4-2-1 bits, X is ignored. Same result as
/// mlat-server's string, read as a decimal number.
pub fn decode_id13(id13: u16) -> u16 {
    let bit = |n: u16| (id13 >> n) & 1;
    let digit = |b1: u16, b2: u16, b4: u16| bit(b1) | (bit(b2) << 1) | (bit(b4) << 2);
    let a = digit(11, 9, 7);
    let b = digit(5, 3, 1);
    let c = digit(12, 10, 8);
    let d = digit(4, 2, 0);
    a * 1000 + b * 100 + c * 10 + d
}

// ---------------------------------------------------------------------------
// Mode A/C via mlat-client's false_ac

/// A Mode A/C reply carried to us inside mlat-client's "false_ac" encoding: a
/// fake DF11 (first byte 0x5d, last three bytes a4 92 4f) whose 24-bit
/// address is `int(hex4.ljust(6, '0')) % 1_000_000` where `hex4` is the
/// 4-hex-digit Mode A/C payload with every digit in 0..=8. See
/// flyitalyadsb/mlat-client coordinator.py (`false_ac`) and
/// nesso-infra k8s/flyitalyadsb/base/acbridge/acbridge.py (`false_ac_message`).
///
/// readsb carries a Mode A/C reply in a Beast type-'1' frame as 2 bytes of
/// hex-coded octal digits (0xABCD for code ABCD). The reply format is the same
/// for a Mode A and a Mode C interrogation, so both readings are returned:
/// `squawk` (the identity) and `altitude_ft` (readsb's `modeAToModeC()` in
/// mode_ac.c applied to the same bits, x100 ft, None when not a valid Gillham
/// altitude). Read flyitalyadsb/readsb mode_ac.c and reproduce it exactly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeAc {
    /// The 2 payload bytes as readsb wrote them, e.g. 0x7000 for code 7000.
    pub code: u16,
    /// The reply read as a Mode A identity, as decimal digits (7000). The
    /// spare bit of each nibble (a digit 8; in practice only the SPI/ident
    /// bit 0x0080 is ever set) is dropped, as readsb's `squawkHex & 0x7777`.
    pub squawk: u16,
    /// The reply read as a Mode C altitude (Gillham), feet, when valid.
    pub altitude_ft: Option<i32>,
}

/// The fixed bytes mlat-client wraps around the fake address.
const FALSE_AC_HEAD: u8 = 0x5d;
const FALSE_AC_TAIL: [u8; 3] = [0xa4, 0x92, 0x4f];

/// Recognise a false_ac frame and recover the Mode A/C payload; None for any
/// genuine Mode S frame. Exact pattern match, not an address-range test.
pub fn false_ac(frame: &[u8]) -> Option<ModeAc> {
    if frame.len() != 7 || frame[0] != FALSE_AC_HEAD || frame[4..] != FALSE_AC_TAIL {
        return None;
    }
    let addr = u24(&frame[1..4]);
    if !is_false_ac_address(addr) {
        return None;
    }
    // The address is the 4 payload digits followed by "00" in decimal; the
    // zero padding to 6 digits restores the leading zeros of codes like 0040.
    let digits = addr / 100;
    let code = ((digits / 1000) << 12) | ((digits / 100 % 10) << 8) | ((digits / 10 % 10) << 4) | (digits % 10);
    let code = code as u16; // every digit <= 8, so code <= 0x8888
    Some(ModeAc {
        code,
        squawk: hex_digits_to_dec(code & 0x7777),
        altitude_ft: mode_a_to_mode_c(code).map(|c| c * 100),
    })
}

/// True when a 24-bit address has the false_ac shape (decimal multiple of 100
/// with every one of its 6 zero-padded decimal digits in 0..=8 and <= 999999).
pub fn is_false_ac_address(addr: u32) -> bool {
    if addr > 999_999 || !addr.is_multiple_of(100) {
        return false;
    }
    let mut rest = addr;
    for _ in 0..6 {
        if rest % 10 > 8 {
            return false;
        }
        rest /= 10;
    }
    true
}

/// readsb `squawkHex2Dec`: 0xABCD -> decimal ABCD.
fn hex_digits_to_dec(v: u16) -> u16 {
    (v >> 12) * 1000 + ((v >> 8) & 0xf) * 100 + ((v >> 4) & 0xf) * 10 + (v & 0xf)
}

/// 500 ft Gray-code bits of a readsb Mode A value and the XOR mask each one
/// contributes. XOR-ing "this bit and every lower one" is a Gray -> binary
/// conversion done bit by bit, D2 being the most significant.
const MODE_A_FIVE_HUNDREDS: [(u32, u32); 8] = [
    (0x0002, 0xFF), // D2
    (0x0004, 0x7F), // D4
    (0x1000, 0x3F), // A1
    (0x2000, 0x1F), // A2
    (0x4000, 0x0F), // A4
    (0x0100, 0x07), // B1
    (0x0200, 0x03), // B2
    (0x0400, 0x01), // B4
];

/// readsb `modeAToModeC()`: a Mode A/C value in readsb's layout
/// (00 A4 A2 A1 | 00 B4 B2 B1 | SPI C4 C2 C1 | 00 D4 D2 D1) to a Gillham
/// altitude in hundreds of feet, None for readsb's INVALID_ALTITUDE.
///
/// readsb looks the value up in a 4096-entry table indexed by
/// `modeAToIndex()`, which drops the spare bit of each nibble (0x8888, the
/// SPI bit among them), and fills the table with `internalModeAToModeC()` of
/// `indexToModeA(i)`: i.e. `internalModeAToModeC(modeA & 0x7777)`. Its "zero
/// bits are zero" test is therefore only ever about D1. (`decodeModeAMessage`
/// also skips the altitude when SPI is set; for false_ac payloads that is
/// moot, because a set SPI bit means a C nibble of 8, i.e. no C bits, which
/// is already invalid.)
fn mode_a_to_mode_c(code: u16) -> Option<i32> {
    let a = u32::from(code) & 0x7777;
    // D1 is never used for altitude; C1..C4 cannot all be zero.
    if a & 0xFFFF_8889 != 0 || a & 0x0000_00F0 == 0 {
        return None;
    }
    let mut one_hundreds = 0u32;
    if a & 0x0010 != 0 {
        one_hundreds ^= 0x007; // C1
    }
    if a & 0x0020 != 0 {
        one_hundreds ^= 0x003; // C2
    }
    if a & 0x0040 != 0 {
        one_hundreds ^= 0x001; // C4
    }
    // Gray 100 decodes to 7 and Gray 111 to 5; Gillham uses the former as 5.
    if one_hundreds & 5 == 5 {
        one_hundreds ^= 2;
    }
    if one_hundreds > 5 {
        return None;
    }
    let five_hundreds = MODE_A_FIVE_HUNDREDS.iter().filter(|&&(bit, _)| a & bit != 0).fold(0, |f, &(_, mask)| f ^ mask);
    // Correct order of OneHundreds (reflected on odd 500 ft steps).
    if five_hundreds & 1 != 0 {
        one_hundreds = 6 - one_hundreds;
    }
    Some((five_hundreds * 5 + one_hundreds) as i32 - 13)
}
