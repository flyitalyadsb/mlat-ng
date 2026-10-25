//! Comm-B kinematics: BDS 5,0 (track and turn report) and BDS 6,0 (heading
//! and speed report), the two Enhanced Surveillance registers that carry an
//! aircraft's velocity in the 56-bit MB field of a DF20/DF21 reply. Pure
//! functions, no I/O.
//!
//! The MB field does not say which register it holds (the ground station
//! asked for one; we only hear the reply), so the register is inferred from
//! the bits. The rules follow pyModeS 2.x `bds.infer(msg, mrar=True)` (the
//! v2 checks, including the altitude one that v3 dropped), with the changes
//! marked (*):
//!
//! 1. Status bits. Every field has one; when it is 0 the whole field, sign
//!    included, must be 0 (*pyModeS v2 skips the roll sign bit, v3 checks
//!    it). An all-zero MB is nothing.
//! 2. Ranges (`decode_bds50` / `decode_bds60`): |roll| <= 50 deg, GS and
//!    TAS <= 800 kt (*pyModeS: 600 kt, which loses jet-stream ground
//!    speeds), IAS <= 500 kt, Mach <= 1, |vertical rates| <= 6000 ft/min.
//!    The track rate is not limited (as pyModeS).
//! 3. Cross-checks, as pyModeS: a 5,0 needs |TAS - GS| <= 200 kt (the wind);
//!    a 6,0 needs its IAS within 20 kt of the CAS its Mach gives at the
//!    reply's pressure altitude (ISA), when the altitude is known.
//! 4. Other registers. (*) An MB that also passes pyModeS's BDS 4,0 or 1,7
//!    test is neither: pyModeS lists both candidates, but on live data none
//!    of those replies matched the aircraft's ADS-B velocity as a 5,0 or 6,0
//!    (they read as selected altitudes). Format-ID registers (1,0, 2,0, 3,0)
//!    never get this far: their first byte puts non-zero bits in the roll /
//!    heading field with its status bit clear. Meteorological 4,4 / 4,5 are
//!    ignored, as pyModeS does without `mrar`: their tests are loose, and
//!    96-100% of the 5,0 / 6,0 replies that also pass them were genuine.
//! 5. Tie-break (*, after readsb's comm_b.c plausibility scores), only when
//!    both registers pass 1-4: a 5,0 whose track rate is more than 1 deg/s
//!    off the coordinated-turn rate of its roll (g tan(roll) / TAS) is out;
//!    a 6,0 whose baro and inertial vertical rates differ by more than
//!    1000 ft/min, or whose IAS is more than 10 kt off CAS(Mach), is out. If
//!    exactly one is out the other wins, otherwise the result is
//!    `Kinematics::Ambiguous` and the caller resolves it against its track.
//!
//! On a 60 s capture of the live feed (297k distinct DF20/21 replies) rules
//! 1-3 give pyModeS's answer for all but 8 (6 ground speeds over 600 kt,
//! which ADS-B confirms, and 2 MBs holding nothing but a roll sign bit).
//! 1.8% of the replies pass both registers; rule 5 settles 92% of those,
//! never against the aircraft's own ADS-B velocity.
//!
//! Bit numbers are MB bits, 1..56 (Doc 9871 numbering; MB bit 1 is message
//! bit 33). Signed fields are two's complement over sign bit + magnitude.

#[cfg(test)]
mod tests;

/// BDS 5,0 track and turn report (Doc 9871 Table A-2-80). A field is None
/// when its status bit says it is not available.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bds50 {
    /// Roll angle, degrees, positive = right wing down. Bits 1..11, LSB 45/256.
    pub roll_deg: Option<f64>,
    /// True track angle, degrees clockwise from true north, 0..360. Bits
    /// 12..23, LSB 90/512.
    pub true_track_deg: Option<f64>,
    /// Ground speed, knots. Bits 24..34, LSB 2 kt.
    pub gs_kt: Option<f64>,
    /// Track angle rate, degrees/second, positive = turning right. Bits
    /// 35..45, LSB 8/256.
    pub track_rate_dps: Option<f64>,
    /// True airspeed, knots. Bits 46..56, LSB 2 kt.
    pub tas_kt: Option<f64>,
}

/// BDS 6,0 heading and speed report (Doc 9871 Table A-2-96). A field is None
/// when its status bit says it is not available.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bds60 {
    /// Magnetic heading, degrees clockwise from magnetic north, 0..360. Bits
    /// 1..12, LSB 90/512.
    pub mag_heading_deg: Option<f64>,
    /// Indicated airspeed, knots. Bits 13..23, LSB 1 kt.
    pub ias_kt: Option<f64>,
    /// Mach number. Bits 24..34, LSB 2.048/512.
    pub mach: Option<f64>,
    /// Barometric altitude rate, ft/min, positive = climbing. Bits 35..45,
    /// LSB 32 ft/min.
    pub baro_rate_fpm: Option<f64>,
    /// Inertial vertical velocity, ft/min, positive = climbing. Bits 46..56,
    /// LSB 32 ft/min.
    pub inertial_rate_fpm: Option<f64>,
}

/// What an MB field carries, as far as kinematics go.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Kinematics {
    Bds50(Bds50),
    Bds60(Bds60),
    /// Valid as both and the tie-break could not tell: both readings.
    Ambiguous(Bds50, Bds60),
}

// Ranges (rule 2).
const MAX_ROLL_DEG: f64 = 50.0;
const MAX_GS_KT: f64 = 800.0;
const MAX_TAS_KT: f64 = 800.0;
const MAX_IAS_KT: f64 = 500.0;
const MAX_MACH: f64 = 1.0;
const MAX_VRATE_FPM: f64 = 6000.0;

// Cross-checks (rule 3), pyModeS's values.
const MAX_TAS_GS_KT: f64 = 200.0;
const MAX_IAS_CAS_KT: f64 = 20.0;

// Tie-break (rule 5). On live data a genuine 5,0 is within 0.5 deg/s of
// the turn its roll makes in 99.9% of replies, and a genuine 6,0 has IAS
// within 5 kt of CAS(Mach) in 99.9% and its two rates within 500 ft/min in
// 99%; the misread register is much further off.
const TIE_TURN_DPS: f64 = 1.0;
const TIE_RATES_FPM: f64 = 1000.0;
const TIE_IAS_CAS_KT: f64 = 10.0;

/// Decode the MB field as BDS 5,0. None when a status bit is clear but its
/// field is not zero, when a value is out of range (rule 2), or for an
/// all-zero MB. Says nothing about whether the MB really is a 5,0: see
/// `infer_kinematics`.
pub fn decode_bds50(mb: &[u8; 7]) -> Option<Bds50> {
    let v = mb_bits(mb);
    if v == 0 {
        return None;
    }
    let b = Bds50 {
        roll_deg: gated(v, 2, 11)?.map(|r| signed(r, 10) * 45.0 / 256.0),
        true_track_deg: gated(v, 13, 23)?.map(|r| (signed(r, 11) * 90.0 / 512.0).rem_euclid(360.0)),
        gs_kt: gated(v, 25, 34)?.map(|r| r as f64 * 2.0),
        track_rate_dps: gated(v, 36, 45)?.map(|r| signed(r, 10) * 8.0 / 256.0),
        tas_kt: gated(v, 47, 56)?.map(|r| r as f64 * 2.0),
    };
    let ok = within(b.roll_deg, MAX_ROLL_DEG) && within(b.gs_kt, MAX_GS_KT) && within(b.tas_kt, MAX_TAS_KT);
    ok.then_some(b)
}

/// Decode the MB field as BDS 6,0. None when a status bit is clear but its
/// field is not zero, when a value is out of range (rule 2), or for an
/// all-zero MB. Says nothing about whether the MB really is a 6,0: see
/// `infer_kinematics`.
pub fn decode_bds60(mb: &[u8; 7]) -> Option<Bds60> {
    let v = mb_bits(mb);
    if v == 0 {
        return None;
    }
    let b = Bds60 {
        mag_heading_deg: gated(v, 2, 12)?.map(|r| (signed(r, 11) * 90.0 / 512.0).rem_euclid(360.0)),
        ias_kt: gated(v, 14, 23)?.map(|r| r as f64),
        // Same operation order as pyModeS, so 0.42 comes out as 0.42.
        mach: gated(v, 25, 34)?.map(|r| r as f64 * 2.048 / 512.0),
        baro_rate_fpm: gated(v, 36, 45)?.map(|r| signed(r, 10) * 32.0),
        inertial_rate_fpm: gated(v, 47, 56)?.map(|r| signed(r, 10) * 32.0),
    };
    let ok = within(b.ias_kt, MAX_IAS_KT)
        && within(b.mach, MAX_MACH)
        && within(b.baro_rate_fpm, MAX_VRATE_FPM)
        && within(b.inertial_rate_fpm, MAX_VRATE_FPM);
    ok.then_some(b)
}

/// Decide whether an MB field is a BDS 5,0 or 6,0 (module docs, rules 1-5).
/// `alt_ft` is the pressure altitude for the IAS / Mach checks: the reply's
/// own AC13 for a DF20; for a DF21 (no altitude) the track's, if the caller
/// has one, else None and those checks are skipped. None when the MB is
/// neither register (or something else as well).
pub fn infer_kinematics(mb: &[u8; 7], alt_ft: Option<i32>) -> Option<Kinematics> {
    let v = mb_bits(mb);
    if v == 0 || looks_like_bds40(v) || looks_like_bds17(v) {
        return None;
    }
    let b50 = decode_bds50(mb).filter(|b| b.tas_gs_gap_kt().is_none_or(|d| d <= MAX_TAS_GS_KT));
    let b60 = decode_bds60(mb).filter(|b| b.ias_cas_gap_kt(alt_ft).is_none_or(|d| d <= MAX_IAS_CAS_KT));
    match (b50, b60) {
        (None, None) => None,
        (Some(a), None) => Some(Kinematics::Bds50(a)),
        (None, Some(b)) => Some(Kinematics::Bds60(b)),
        (Some(a), Some(b)) => {
            let out50 = a.turn_gap_dps().is_some_and(|d| d > TIE_TURN_DPS);
            let out60 = b.rates_gap_fpm().is_some_and(|d| d > TIE_RATES_FPM)
                || b.ias_cas_gap_kt(alt_ft).is_some_and(|d| d > TIE_IAS_CAS_KT);
            Some(match (out50, out60) {
                (false, true) => Kinematics::Bds50(a),
                (true, false) => Kinematics::Bds60(b),
                _ => Kinematics::Ambiguous(a, b),
            })
        }
    }
}

impl Bds50 {
    /// Ground velocity (east, north) in m/s from ground speed and true track,
    /// when both are present.
    pub fn ground_velocity_en(&self) -> Option<(f64, f64)> {
        let gs = self.gs_kt? * KT_TO_MS;
        let (s, c) = self.true_track_deg?.to_radians().sin_cos();
        Some((gs * s, gs * c))
    }

    /// |TAS - GS|, knots, when both are present.
    fn tas_gs_gap_kt(&self) -> Option<f64> {
        Some((self.tas_kt? - self.gs_kt?).abs())
    }

    /// How far the track rate is from the rate of a coordinated turn at this
    /// roll, g tan(roll) / V, in deg/s; V is TAS, else GS (wind makes that a
    /// little off, the tolerance covers it). None when a term is missing.
    fn turn_gap_dps(&self) -> Option<f64> {
        let v_kt = self.tas_kt.filter(|&v| v > 0.0).or(self.gs_kt.filter(|&v| v > 0.0))?;
        let turn_dps = (G0 * self.roll_deg?.to_radians().tan() / (v_kt * KT_TO_MS)).to_degrees();
        Some((self.track_rate_dps? - turn_dps).abs())
    }
}

impl Bds60 {
    /// True airspeed, knots, at pressure altitude `alt_ft` in the ISA: from
    /// Mach when present, else from IAS taken as CAS. For checking a 6,0
    /// against a track's speed (the wind is still in between).
    pub fn tas_kt(&self, alt_ft: i32) -> Option<f64> {
        let h = f64::from(alt_ft) * FT_TO_M;
        match (self.mach, self.ias_kt) {
            (Some(m), _) => Some(m * speed_of_sound(h) / KT_TO_MS),
            (None, Some(ias)) => Some(cas_to_tas(ias * KT_TO_MS, h) / KT_TO_MS),
            (None, None) => None,
        }
    }

    /// |IAS - CAS(Mach)| at the altitude, knots, as pyModeS: IAS is taken as
    /// CAS, and the Mach -> CAS conversion depends on pressure only, so ISA
    /// temperature errors do not enter. None without IAS, Mach or altitude.
    fn ias_cas_gap_kt(&self, alt_ft: Option<i32>) -> Option<f64> {
        let (ias, mach, alt) = (self.ias_kt?, self.mach?, alt_ft?);
        let h = f64::from(alt) * FT_TO_M;
        Some((ias - tas_to_cas(mach * speed_of_sound(h), h) / KT_TO_MS).abs())
    }

    /// |baro rate - inertial rate|, ft/min, when both are present.
    fn rates_gap_fpm(&self) -> Option<f64> {
        Some((self.baro_rate_fpm? - self.inertial_rate_fpm?).abs())
    }
}

// ---------------------------------------------------------------------------
// Bits

/// The MB field as a 56-bit number, MB bit 1 most significant.
fn mb_bits(mb: &[u8; 7]) -> u64 {
    mb.iter().fold(0, |acc, &b| (acc << 8) | u64::from(b))
}

/// MB bits `first..=last` (1-based) as an unsigned number.
fn bits(v: u64, first: u32, last: u32) -> u64 {
    (v >> (56 - last)) & ((1 << (last - first + 1)) - 1)
}

/// A field at bits `first..=last` whose status bit is bit `first - 1`, as
/// every 5,0 / 6,0 / 4,0 field is laid out. Some(Some(raw)) when the status
/// bit is set; Some(None) when it is clear and the field is all zero;
/// None when it is clear but the field is not, which rules the register out.
fn gated(v: u64, first: u32, last: u32) -> Option<Option<u64>> {
    let raw = bits(v, first, last);
    match (bits(v, first - 1, first - 1), raw) {
        (1, _) => Some(Some(raw)),
        (_, 0) => Some(None),
        _ => None,
    }
}

/// A `width`-bit two's complement field (sign bit on top) as a number: sign 1
/// with magnitude 0 is the most negative value, not -0.
fn signed(raw: u64, width: u32) -> f64 {
    if (raw >> (width - 1)) & 1 == 1 {
        raw as f64 - (1u64 << width) as f64
    } else {
        raw as f64
    }
}

/// |x| <= max, or absent.
fn within(x: Option<f64>, max: f64) -> bool {
    x.is_none_or(|x| x.abs() <= max)
}

/// pyModeS's BDS 4,0 (selected vertical intention) test: status-gated MCP
/// altitude (bits 1..13), FMS altitude (14..26), baro setting (27..39), mode
/// bits (48..51) and target altitude source (54..56); 40..47 and 52..53 are
/// reserved and must be zero.
fn looks_like_bds40(v: u64) -> bool {
    bits(v, 40, 47) == 0
        && bits(v, 52, 53) == 0
        && [(2, 13), (15, 26), (28, 39), (49, 51), (55, 56)].iter().all(|&(a, b)| gated(v, a, b).is_some())
}

/// pyModeS's BDS 1,7 (GICB capability report) test: bits 25..56 unused, and
/// the 2,0 (identification) capability, bit 7, set.
fn looks_like_bds17(v: u64) -> bool {
    bits(v, 25, 56) == 0 && bits(v, 7, 7) == 1
}

// ---------------------------------------------------------------------------
// ISA, as pyModeS `extra/aero.py` (same constants and formulas, so the 20 kt
// check falls on the same side for the same reply). Mode S altitude is
// pressure altitude, i.e. ISA geopotential height.

const KT_TO_MS: f64 = 1852.0 / 3600.0;
const FT_TO_M: f64 = 0.3048;
const G0: f64 = 9.80665;
const R_AIR: f64 = 287.05287;
const GAMMA: f64 = 1.4;
const P0: f64 = 101_325.0;
const RHO0: f64 = 1.225;
const T0: f64 = 288.15;

/// (pressure Pa, density kg/m^3, temperature K) at height `h` metres:
/// -6.5 K/km to 11 km, isothermal (216.65 K) above.
fn isa(h: f64) -> (f64, f64, f64) {
    let t = (T0 - 0.0065 * h).max(216.65);
    let rho_trop = RHO0 * (t / T0).powf(4.256_848_030_018_761);
    let rho = rho_trop * (-(h - 11_000.0).max(0.0) / 6_341.552_161).exp();
    (rho * R_AIR * t, rho, t)
}

fn speed_of_sound(h: f64) -> f64 {
    (GAMMA * R_AIR * isa(h).2).sqrt()
}

/// True -> calibrated airspeed (m/s) at `h`: the CAS that gives the same
/// impact pressure at sea level (subsonic).
fn tas_to_cas(tas: f64, h: f64) -> f64 {
    let (p, rho, _) = isa(h);
    let qc = p * ((1.0 + rho * tas * tas / (7.0 * p)).powf(3.5) - 1.0);
    (7.0 * P0 / RHO0 * ((qc / P0 + 1.0).powf(2.0 / 7.0) - 1.0)).sqrt()
}

/// Calibrated -> true airspeed (m/s) at `h`, the inverse of `tas_to_cas`.
fn cas_to_tas(cas: f64, h: f64) -> f64 {
    let (p, rho, _) = isa(h);
    let qc = P0 * ((1.0 + RHO0 * cas * cas / (7.0 * P0)).powf(3.5) - 1.0);
    (7.0 * p / rho * ((1.0 + qc / p).powf(2.0 / 7.0) - 1.0)).sqrt()
}
