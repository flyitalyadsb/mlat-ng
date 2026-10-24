//! Mode S / ADS-B / Mode A/C frame decoding. Pure functions, no I/O.
//!
//! IMPLEMENTATION OWNER: agent "modes". Fill every `todo!()`, keep the public
//! signatures, add unit tests with real frames (known vectors from the
//! literature / pyModeS / dump1090 tests, and frames you construct and check
//! against an independent computation).

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
    /// ME type code (1..31).
    pub tc: u8,
    /// NUCp for position messages (mlat-server's table: TC 9..18 -> 9..0,
    /// TC 20..22 -> 9..8), None otherwise.
    pub nuc: Option<u8>,
    /// CPR format bit (false = even, true = odd), position messages only.
    pub odd: bool,
    /// 17-bit CPR latitude / longitude, position messages only.
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
    /// airborne position); None when absent, zero or invalid.
    pub altitude_ft: Option<i32>,
    /// Squawk as a 4-digit octal-looking number, e.g. 7000 (DF5/21).
    pub squawk: Option<u16>,
    /// Callsign from ES identification, trimmed.
    pub callsign: Option<String>,
    pub es: Option<Es>,
}

/// Decode a 7- or 14-byte Mode S frame. Returns None for unknown DF, wrong
/// length for the DF, or malformed input. Must never panic on any input.
pub fn decode(_frame: &[u8]) -> Option<Decoded> {
    todo!()
}

/// CRC-24 residual of a Mode S frame (0 for a valid DF11/17/18 frame; the
/// address for AP formats with a clean frame).
pub fn crc_residual(_frame: &[u8]) -> u32 {
    todo!()
}

/// Global airborne CPR decode of an even/odd pair, as mlat-server's
/// `decode_cpr`: returns (even_lat, even_lon, odd_lat, odd_lon) in degrees,
/// or None when the pair straddles a latitude zone boundary / is invalid.
pub fn cpr_global_airborne(_lat_e: u32, _lon_e: u32, _lat_o: u32, _lon_o: u32) -> Option<(f64, f64, f64, f64)> {
    todo!()
}

/// Gillham-coded 13-bit altitude (AC13, with M and Q bits) to feet.
pub fn decode_ac13(_ac13: u16) -> Option<i32> {
    todo!()
}

/// 12-bit ES altitude (AC12, Q bit at position 4) to feet.
pub fn decode_ac12(_ac12: u16) -> Option<i32> {
    todo!()
}

/// 13-bit identity field (DF5/21 ID13) to squawk digits, e.g. 0o7000 -> 7000.
pub fn decode_id13(_id13: u16) -> u16 {
    todo!()
}

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
    /// The reply read as a Mode A identity, as decimal digits (7000).
    pub squawk: u16,
    /// The reply read as a Mode C altitude (Gillham), feet, when valid.
    pub altitude_ft: Option<i32>,
}

/// Recognise a false_ac frame and recover the Mode A/C payload; None for any
/// genuine Mode S frame. Exact pattern match, not an address-range test.
pub fn false_ac(_frame: &[u8]) -> Option<ModeAc> {
    todo!()
}

/// True when a 24-bit address has the false_ac shape (decimal multiple of 100
/// with every one of its 6 zero-padded decimal digits in 0..=8 and <= 999999).
pub fn is_false_ac_address(_addr: u32) -> bool {
    todo!()
}
