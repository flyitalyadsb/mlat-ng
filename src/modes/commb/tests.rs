//! Tests for the Comm-B kinematics decoder. Published vectors and reference
//! numbers come from pyModeS (2.22 `bds.infer`, `commb.*50/*60`,
//! `extra.aero`); the encodings are checked against an encoder written here
//! from the Doc 9871 field layouts, exhaustively over every raw value.

use super::*;

fn frame_mb(h: &str) -> [u8; 7] {
    hex::decode(h).unwrap()[4..11].try_into().unwrap()
}

/// MB from (first bit, last bit, value) fields, MB bit numbering (1..56).
fn mb(fields: &[(u32, u32, u64)]) -> [u8; 7] {
    let v = fields.iter().fold(0u64, |acc, &(first, last, x)| {
        assert!(x < 1 << (last - first + 1), "{x} does not fit in bits {first}..{last}");
        acc | (x << (56 - last))
    });
    v.to_be_bytes()[1..].try_into().unwrap()
}

/// (status bit, last bit) of each field in struct order; the value is
/// status + 1 ..= last, sign bit first where there is one.
const FIELDS_50: [(u32, u32); 5] = [(1, 11), (12, 23), (24, 34), (35, 45), (46, 56)];
const FIELDS_60: [(u32, u32); 5] = [(1, 12), (13, 23), (24, 34), (35, 45), (46, 56)];

fn values50(b: &Bds50) -> [Option<f64>; 5] {
    [b.roll_deg, b.true_track_deg, b.gs_kt, b.track_rate_dps, b.tas_kt]
}

fn values60(b: &Bds60) -> [Option<f64>; 5] {
    [b.mag_heading_deg, b.ias_kt, b.mach, b.baro_rate_fpm, b.inertial_rate_fpm]
}

/// Tiny deterministic PRNG (xorshift64), to stay std-only.
struct XorShift(u64);
impl XorShift {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }
    fn mb(&mut self) -> [u8; 7] {
        self.next().to_be_bytes()[1..].try_into().unwrap()
    }
}

// ---------------------------------------------------------------------------
// Published vectors (pyModeS)

#[test]
fn bds50_pymodes_vectors() {
    // pyModeS test_bds50_functions: roll 2.1, track 114.258, GS 438,
    // rate 0.125, TAS 424; the second one carries a negative roll.
    let b = decode_bds50(&frame_mb("A000139381951536E024D4CCF6B5")).unwrap();
    // Every LSB is a binary fraction, so the values are exact.
    assert_eq!(values50(&b), [Some(2.109375), Some(114.2578125), Some(438.0), Some(0.125), Some(424.0)]);
    let b = decode_bds50(&frame_mb("A0001691FFD263377FFCE02B2BF9")).unwrap();
    assert_eq!(values50(&b), [Some(-0.3515625), Some(53.61328125), Some(442.0), Some(-0.03125), Some(448.0)]);
}

#[test]
fn bds60_pymodes_vectors() {
    // pyModeS test_bds60_functions: heading 42.71484, IAS 252, Mach 0.42,
    // both vertical rates -1920 ft/min.
    let b = decode_bds60(&frame_mb("A00004128F39F91A7E27C46ADC21")).unwrap();
    assert_eq!(values60(&b), [Some(42.71484375), Some(252.0), Some(0.42), Some(-1920.0), Some(-1920.0)]);
}

#[test]
fn infer_pymodes_vectors() {
    use Kinematics::*;
    // pyModeS test_bds_infer (+ v3's 4,0, 2,0, 4,4 and 4,5 golden vectors),
    // with the DF20 altitude each frame carries.
    let infer = |h: &str| {
        let d = crate::modes::decode(&hex::decode(h).unwrap()).unwrap();
        infer_kinematics(&frame_mb(h), d.altitude_ft)
    };
    assert!(matches!(infer("A000139381951536E024D4CCF6B5"), Some(Bds50(_)))); // 30275 ft
    assert!(matches!(infer("A0001691FFD263377FFCE02B2BF9"), Some(Bds50(_))));
    assert!(matches!(infer("A800292DFFBBA9383FFCEB903D01"), Some(Bds50(_)))); // DF21
    assert!(matches!(infer("A00004128F39F91A7E27C46ADC21"), Some(Bds60(_)))); // 5450 ft
    for (h, bds) in [
        ("A800178D10010080F50000D5893C", "1,0"),
        ("A0000638FA81C10000000081A92F", "1,7"),
        ("A0001838201584F23468207CDFA5", "2,0"),
        ("A000083E202CC371C31DE0AA1CCF", "2,0"),
        ("A0001839CA3800315800007448D9", "4,0"),
        ("A000029C85E42F313000007047D3", "4,0"),
        ("A0001692185BD5CF400000DFC696", "4,4"),
        ("A00004190001FB80000000000000", "4,5"),
    ] {
        assert_eq!(infer(h), None, "{h} is BDS {bds}");
        let mb = frame_mb(h);
        assert_eq!((decode_bds50(&mb), decode_bds60(&mb)), (None, None), "{h}");
    }
}

#[test]
fn pymodes_ambiguous_vectors_are_settled() {
    // pyModeS test_bds_is50or60: DF21 (no altitude) replies valid as both;
    // against an ADS-B reference of 320 kt / 250 deg / 14000 ft pyModeS
    // picks 5,0. The 6,0 reading has an inertial rate of +5344 ft/min next
    // to a baro rate of 0 (the 5,0 TAS seen through the wrong layout), and
    // the 5,0 reading's track rate matches its roll: 5,0 without a reference.
    for h in ["A8001EBCFFFB23286004A73F6A5B", "A8001EBCFE1B29287FDCA807BCFC"] {
        let mb = frame_mb(h);
        let (b50, b60) = (decode_bds50(&mb).unwrap(), decode_bds60(&mb).unwrap());
        assert_eq!(b50.gs_kt, Some(322.0), "{h}");
        assert!((b50.true_track_deg.unwrap() - 250.75).abs() < 0.5, "{h}");
        assert!(b60.rates_gap_fpm().unwrap() > 5000.0, "{h}");
        assert_eq!(infer_kinematics(&mb, None), Some(Kinematics::Bds50(b50)), "{h}");
        // With the altitude, the 6,0 reading's IAS ~400 kt against
        // CAS(Mach 0.644) ~333 kt already rules it out.
        assert_eq!(infer_kinematics(&mb, Some(14000)), Some(Kinematics::Bds50(b50)), "{h}");
    }
    // pyModeS v3 test_infer's ambiguous payload: 5,0 GS 240 / TAS 228, or
    // 6,0 IAS 336 / Mach 0.48. At the frame's 3300 ft CAS(Mach 0.48) is
    // ~300 kt (v2's altitude check: 5,0); without altitude the vertical
    // rates (0 vs +3648 ft/min) settle it the same way.
    let mb = frame_mb("A000029CFFBAA11E2004727281F1");
    let b50 = decode_bds50(&mb).unwrap();
    assert_eq!((b50.gs_kt, b50.tas_kt), (Some(240.0), Some(228.0)));
    let b60 = decode_bds60(&mb).unwrap();
    assert_eq!((b60.ias_kt, b60.mach), (Some(336.0), Some(0.48)));
    assert!(b60.ias_cas_gap_kt(Some(3300)).unwrap() > MAX_IAS_CAS_KT);
    assert_eq!(infer_kinematics(&mb, Some(3300)), Some(Kinematics::Bds50(b50)));
    assert_eq!(infer_kinematics(&mb, None), Some(Kinematics::Bds50(b50)));
}

#[test]
fn pymodes_sample_data() {
    use Kinematics::*;
    // Replies from pyModeS's tests/data/sample_data_commb_df20/21.csv, with
    // pyModeS 2.22 `bds.infer(msg, mrar=True)` and our answer. Over the
    // whole file (5959 distinct replies) rules 1-3 agree with pyModeS on
    // every one; the differences are the cases below (rules 4 and 5).
    let class = |k| match k {
        None => "-",
        Some(Bds50(_)) => "5,0",
        Some(Bds60(_)) => "6,0",
        Some(Ambiguous(..)) => "5,0|6,0",
    };
    for (h, pymodes, ours) in [
        ("A000019910010080F500004315B2", "BDS10", "-"),
        ("A00006B2FE818100000000D24B2B", "BDS17", "-"),
        ("A00017B0202422F94958208F0A91", "BDS20", "-"),
        ("A00015B7C26E1370AA00005DD34A", "BDS40", "-"),
        ("A0000D92B2C80031BC00000EDA3B", "BDS40,BDS45", "-"),
        ("A0001910FA810300000000DD1204", "BDS17,BDS45", "-"),
        ("A000093F0000000000000026CD84", "EMPTY", "-"),
        ("A00015B4FFB4993A7FFCDFE19E01", "BDS50", "5,0"),
        ("A000149DFFF4FB3A7FFCE2864CDC", "BDS50", "5,0"),
        ("A0000638B699F11BE3846DCA35F9", "BDS60", "6,0"),
        ("A00019108D49D92FE00C0018B267", "BDS60", "6,0"),
        // Meteorological candidates are ignored.
        ("A0001717807D1934BFFCE080D921", "BDS45,BDS50", "5,0"),
        ("A0001910A3D9F532BFEC007AE77D", "BDS45,BDS60", "6,0"),
        // A 4,0 / 1,7 candidate rules kinematics out. These read as selected
        // altitudes: 35008 ft (FL350 in 16 ft steps) heard at 35000 ft;
        // 33008 ft from an aircraft climbing through 26375 ft.
        ("A0001690C46000000000006B27F5", "BDS40,BDS60", "-"),
        ("A0001117C07800000000008CE73D", "BDS40,BDS45,BDS60", "-"),
        ("A00003308FA00000000000D8639F", "BDS17,BDS40,BDS50,BDS60", "-"),
        ("A00012B9BE8000000000002ABA22", "BDS17,BDS40,BDS45,BDS60", "-"),
        // Both pass: the tie-break. A DF21 whose 6,0 reading has baro +416
        // and inertial +4256 ft/min.
        ("A800179C839B1F1F206C85C375F4", "BDS50,BDS60", "5,0"),
        // A 5,0 reading turning at 3.75 deg/s on 15 deg of roll (2.5 deg/s
        // off) against a 6,0 reading with IAS 219 = CAS(Mach 0.396) at
        // 9950 ft and rates +3840 / +3712.
        ("A00006B68AF9B718E3C474A87B83", "BDS50,BDS60", "6,0"),
        // Both readings hold up: 22.5 deg of roll turning 1.75 deg/s at
        // 344 kt GS, or IAS 279 = CAS(Mach 0.688) at 26375 ft with a single
        // vertical rate. Left to the caller.
        ("A0001117901A2F2B21C000B31B62", "BDS50,BDS60", "5,0|6,0"),
    ] {
        let alt = crate::modes::decode(&hex::decode(h).unwrap()).unwrap().altitude_ft;
        assert_eq!(class(infer_kinematics(&frame_mb(h), alt)), ours, "{h}: pyModeS {pymodes}");
    }
}

// ---------------------------------------------------------------------------
// Field encodings

/// Two's complement value of a `width`-bit raw field, written independently
/// of `signed`.
fn twos(raw: u64, width: u32) -> i64 {
    let half = 1i64 << (width - 1);
    let r = raw as i64;
    if r >= half {
        r - 2 * half
    } else {
        r
    }
}

/// One field's encoding: register, index in struct order, LSB as
/// `scale / div` (the decoder's operation order, so values compare exactly),
/// signed, angle wrapped to 0..360, range limit.
struct Enc(u8, usize, f64, f64, bool, bool, f64);

const ENCODINGS: [Enc; 10] = [
    Enc(50, 0, 45.0, 256.0, true, false, 50.0),
    Enc(50, 1, 90.0, 512.0, true, true, f64::INFINITY),
    Enc(50, 2, 2.0, 1.0, false, false, 800.0),
    Enc(50, 3, 8.0, 256.0, true, false, f64::INFINITY),
    Enc(50, 4, 2.0, 1.0, false, false, 800.0),
    Enc(60, 0, 90.0, 512.0, true, true, f64::INFINITY),
    Enc(60, 1, 1.0, 1.0, false, false, 500.0),
    Enc(60, 2, 2.048, 512.0, false, false, 1.0),
    Enc(60, 3, 32.0, 1.0, true, false, 6000.0),
    Enc(60, 4, 32.0, 1.0, true, false, 6000.0),
];

#[test]
fn every_raw_value_decodes_and_reencodes() {
    for &Enc(reg, i, scale, div, is_signed, angle, limit) in &ENCODINGS {
        let (status, last) = if reg == 50 { FIELDS_50[i] } else { FIELDS_60[i] };
        let width = last - status;
        for raw in 0..1u64 << width {
            let m = mb(&[(status, status, 1), (status + 1, last, raw)]);
            let n = if is_signed { twos(raw, width) } else { raw as i64 };
            let value = n as f64 * scale / div;
            let got = if reg == 50 {
                decode_bds50(&m).map(|b| values50(&b))
            } else {
                decode_bds60(&m).map(|b| values60(&b))
            };
            let Some(got) = got else {
                assert!(value.abs() > limit, "BDS {reg} field {i} raw {raw} rejected");
                continue;
            };
            assert!(value.abs() <= limit, "BDS {reg} field {i} raw {raw} accepted");
            for (j, g) in got.iter().enumerate() {
                if j != i {
                    assert_eq!(*g, None, "BDS {reg} field {j} should be absent");
                }
            }
            let expect = if angle { value.rem_euclid(360.0) } else { value };
            assert_eq!(got[i], Some(expect), "BDS {reg} field {i} raw {raw}");
            // And back: value / LSB, modulo the field width (angles wrap).
            let back = (expect * div / scale).round() as i64;
            assert_eq!(back.rem_euclid(1 << width) as u64, raw, "BDS {reg} field {i} raw {raw}");
        }
    }
}

#[test]
fn sign_bits_are_twos_complement() {
    // Sign set, magnitude zero: the most negative value, not -0.
    let b = decode_bds50(&mb(&[(12, 12, 1), (13, 13, 1)])).unwrap();
    assert_eq!(b.true_track_deg, Some(180.0)); // -180
    let b = decode_bds50(&mb(&[(35, 35, 1), (36, 36, 1)])).unwrap();
    assert_eq!(b.track_rate_dps, Some(-16.0));
    assert_eq!(decode_bds50(&mb(&[(1, 1, 1), (2, 2, 1)])), None); // roll -90: out of range
    assert_eq!(decode_bds60(&mb(&[(35, 35, 1), (36, 36, 1)])), None); // -16384 ft/min
    // Sign set, magnitude all ones: one LSB below zero.
    let b = decode_bds50(&mb(&[(1, 1, 1), (2, 11, 0x3ff)])).unwrap();
    assert_eq!(b.roll_deg, Some(-45.0 / 256.0)); // left wing down
    let b = decode_bds60(&mb(&[(1, 1, 1), (2, 12, 0x7ff), (46, 46, 1), (47, 56, 0x3ff)])).unwrap();
    assert_eq!((b.mag_heading_deg, b.inertial_rate_fpm), (Some(360.0 - 90.0 / 512.0), Some(-32.0)));
    // West track: 270 deg is -90, i.e. sign 1 and magnitude 512.
    let b = decode_bds50(&mb(&[(12, 12, 1), (13, 13, 1), (14, 23, 512)])).unwrap();
    assert_eq!(b.true_track_deg, Some(270.0));
}

#[test]
fn status_clear_needs_an_all_zero_field() {
    for (fields, which) in [(FIELDS_50, 50), (FIELDS_60, 60)] {
        let decode = |m: &[u8; 7]| if which == 50 { decode_bds50(m).is_some() } else { decode_bds60(m).is_some() };
        for (i, &(status, last)) in fields.iter().enumerate() {
            // Another field present, so the MB is not all zero.
            let (other_status, _) = fields[(i + 1) % 5];
            let base = (other_status, other_status, 1);
            assert!(decode(&mb(&[base])), "BDS {which} field {i} absent");
            // Any one bit of the field with the status clear, the sign bit
            // included (pyModeS v2 lets a lone roll sign bit through).
            for bit in status + 1..=last {
                assert!(!decode(&mb(&[base, (bit, bit, 1)])), "BDS {which} field {i} bit {bit}");
            }
            // Status set with a zero value is a zero.
            let m = mb(&[base, (status, status, 1)]);
            let v = if which == 50 { values50(&decode_bds50(&m).unwrap()) } else { values60(&decode_bds60(&m).unwrap()) };
            assert_eq!(v[i], Some(0.0), "BDS {which} field {i}");
        }
    }
}

// ---------------------------------------------------------------------------
// Ranges

#[test]
fn all_zero_mb_is_nothing() {
    let z = [0u8; 7];
    assert_eq!((decode_bds50(&z), decode_bds60(&z), infer_kinematics(&z, Some(30000))), (None, None, None));
}

#[test]
fn physical_limits() {
    let roll = |raw| decode_bds50(&mb(&[(1, 1, 1), (2, 11, raw)])).map(|b| b.roll_deg.unwrap());
    assert_eq!(roll(284), Some(284.0 * 45.0 / 256.0)); // 49.92
    assert_eq!(roll(1024 - 284), Some(-284.0 * 45.0 / 256.0));
    assert_eq!((roll(285), roll(1024 - 285)), (None, None)); // 50.1
    let gs = |raw| decode_bds50(&mb(&[(24, 24, 1), (25, 34, raw)])).map(|b| b.gs_kt.unwrap());
    assert_eq!((gs(400), gs(401)), (Some(800.0), None));
    let tas = |raw| decode_bds50(&mb(&[(46, 46, 1), (47, 56, raw)])).map(|b| b.tas_kt.unwrap());
    assert_eq!((tas(400), tas(401)), (Some(800.0), None));
    let ias = |raw| decode_bds60(&mb(&[(13, 13, 1), (14, 23, raw)])).map(|b| b.ias_kt.unwrap());
    assert_eq!((ias(500), ias(501)), (Some(500.0), None));
    let mach = |raw| decode_bds60(&mb(&[(24, 24, 1), (25, 34, raw)])).map(|b| b.mach.unwrap());
    assert_eq!((mach(250), mach(251)), (Some(1.0), None));
    for (status, last) in [(35, 45), (46, 56)] {
        let rate = |raw| decode_bds60(&mb(&[(status, status, 1), (status + 1, last, raw)]));
        let rate = |raw| rate(raw).map(|b| b.baro_rate_fpm.or(b.inertial_rate_fpm).unwrap());
        assert_eq!((rate(187), rate(188)), (Some(5984.0), None));
        assert_eq!((rate(1024 - 187), rate(1024 - 188)), (Some(-5984.0), None));
    }
}

// ---------------------------------------------------------------------------
// Inference

#[test]
fn tas_gs_cross_check() {
    // GS 300 kt, TAS 498 / 502 kt. The TAS bits read as a 6,0 inertial rate
    // are ~8000 ft/min, so only the 5,0 reading is in play.
    let m = |tas_raw| mb(&[(24, 24, 1), (25, 34, 150), (46, 46, 1), (47, 56, tas_raw)]);
    assert!(matches!(infer_kinematics(&m(249), None), Some(Kinematics::Bds50(_))));
    assert!(decode_bds50(&m(251)).is_some());
    assert_eq!(infer_kinematics(&m(251), None), None);
}

/// The pyModeS 6,0 vector's fields at a chosen IAS / Mach.
fn mb60(ias: u64, mach_raw: u64) -> [u8; 7] {
    mb(&[(1, 1, 1), (2, 12, 243), (13, 13, 1), (14, 23, ias), (24, 24, 1), (25, 34, mach_raw), (35, 35, 1), (36, 45, 964), (46, 46, 1), (47, 56, 964)])
}

#[test]
fn ias_mach_cross_check_needs_the_altitude() {
    let m = mb60(252, 105); // IAS 252 kt, Mach 0.42: the pyModeS vector itself
    assert_eq!(m, frame_mb("A00004128F39F91A7E27C46ADC21"));
    assert_eq!(decode_bds50(&m), None);
    let b = decode_bds60(&m).unwrap();
    // CAS(Mach 0.42) is 252.25 kt at 5450 ft (pyModeS aero.mach2cas) ...
    assert!((b.ias_cas_gap_kt(Some(5450)).unwrap() - 0.253153501669).abs() < 1e-3);
    assert_eq!(infer_kinematics(&m, Some(5450)), Some(Kinematics::Bds60(b)));
    // ... but 154 kt at 30000 ft: not a 6,0 at that altitude.
    assert_eq!(infer_kinematics(&m, Some(30000)), None);
    // Unknown altitude (DF21 without a track): not checked.
    assert_eq!(infer_kinematics(&m, None), Some(Kinematics::Bds60(b)));
    // Either speed missing: not checked either.
    let no_ias = mb(&[(1, 1, 1), (2, 12, 243), (24, 24, 1), (25, 34, 105), (35, 35, 1), (36, 45, 964), (46, 46, 1), (47, 56, 964)]);
    assert!(matches!(infer_kinematics(&no_ias, Some(30000)), Some(Kinematics::Bds60(_))));
    // 20 kt is the line: IAS 232 at Mach 0.42 / 5450 ft is 20.25 kt off.
    assert!(matches!(infer_kinematics(&mb60(233, 105), Some(5450)), Some(Kinematics::Bds60(_))));
    assert_eq!(infer_kinematics(&mb60(232, 105), Some(5450)), None);
}

#[test]
fn other_registers_are_not_kinematics() {
    // A BDS 4,0 with only the MCP/FCU selected altitude (3008 ft, raw 188)
    // reads as a roll-only 5,0 and a heading-only 6,0; pyModeS answers
    // "BDS40,BDS50,BDS60". Live replies like this were all 4,0.
    let bds40 = mb(&[(1, 1, 1), (2, 13, 188)]);
    assert!(decode_bds50(&bds40).is_some() && decode_bds60(&bds40).is_some());
    assert!(looks_like_bds40(mb_bits(&bds40)));
    assert_eq!(infer_kinematics(&bds40, None), None);
    // A BDS 1,7 capability report (2,0 bit 7 set, bits 25..56 zero) that
    // is a valid 5,0 too but not a 4,0 (FMS status clear, bit 20 set).
    let bds17 = mb(&[(1, 1, 1), (7, 7, 1), (12, 12, 1), (20, 20, 1)]);
    assert!(decode_bds50(&bds17).is_some());
    let v = mb_bits(&bds17);
    assert!(looks_like_bds17(v) && !looks_like_bds40(v));
    assert_eq!(infer_kinematics(&bds17, None), None);
    // Format-ID registers 1,0 / 2,0 / 3,0 put non-zero bits under a clear
    // roll / heading status bit: never 5,0 or 6,0, whatever follows.
    let mut rng = XorShift(0x5EED_C0DE);
    for _ in 0..30_000 {
        let mut m = rng.mb();
        m[0] = [0x10, 0x20, 0x30][(rng.next() % 3) as usize];
        assert_eq!((decode_bds50(&m), decode_bds60(&m), infer_kinematics(&m, None)), (None, None, None));
    }
}

/// Valid as both: 5,0 roll 0.7 deg, track 225, GS 400, TAS 360 kt; 6,0
/// heading 1.6, IAS 256 kt, Mach 0.8, inertial +5760 ft/min. `rate`: the
/// 5,0 track rate field, which a 6,0 reads as the baro rate.
fn both(rate: Option<u64>) -> [u8; 7] {
    let mut f = vec![(1, 1, 1), (2, 11, 4), (12, 12, 1), (13, 13, 1), (14, 23, 256), (24, 24, 1), (25, 34, 200)];
    if let Some(r) = rate {
        f.extend([(35, 35, 1), (36, 45, r)]);
    }
    f.extend([(46, 46, 1), (47, 56, 180)]);
    mb(&f)
}

#[test]
fn ambiguous_when_nothing_tells() {
    let m = both(None);
    let (b50, b60) = (decode_bds50(&m).unwrap(), decode_bds60(&m).unwrap());
    assert_eq!((b50.true_track_deg, b50.gs_kt, b50.tas_kt), (Some(225.0), Some(400.0), Some(360.0)));
    assert_eq!((b60.ias_kt, b60.mach, b60.inertial_rate_fpm), (Some(256.0), Some(0.8), Some(5760.0)));
    // No track rate, a single vertical rate, no altitude: nothing to go on.
    assert_eq!(infer_kinematics(&m, None), Some(Kinematics::Ambiguous(b50, b60)));
    // At 37000 ft CAS(Mach 0.8) is 259.6 kt: IAS 256 fits, still both.
    assert_eq!(infer_kinematics(&m, Some(37000)), Some(Kinematics::Ambiguous(b50, b60)));
    // At 30000 ft it is 303.9 kt: not a 6,0 (pyModeS's check).
    assert_eq!(infer_kinematics(&m, Some(30000)), Some(Kinematics::Bds50(b50)));
}

#[test]
fn tie_break() {
    use Kinematics::*;
    // Track rate 5.625 deg/s at 0.7 deg of roll: no 5,0 turns like that;
    // read as a 6,0 the same bits are a baro rate of +5760, equal to the
    // inertial one.
    let m = both(Some(180));
    let b50 = decode_bds50(&m).unwrap();
    assert!(b50.turn_gap_dps().unwrap() > 5.0);
    assert_eq!(decode_bds60(&m).unwrap().rates_gap_fpm(), Some(0.0));
    assert!(matches!(infer_kinematics(&m, None), Some(Bds60(_))));
    // Track rate 0 matches the roll; as a 6,0, baro 0 vs inertial +5760.
    let m = both(Some(0));
    assert!(decode_bds50(&m).unwrap().turn_gap_dps().unwrap() < 0.1);
    assert!(matches!(infer_kinematics(&m, None), Some(Bds50(_))));
    // Both off (3 deg/s at 0.7 deg; 3072 vs 5760 ft/min): still ambiguous.
    assert!(matches!(infer_kinematics(&both(Some(96)), None), Some(Ambiguous(..))));
    // The IAS-CAS tie-break is tighter than the 20 kt check: at 35000 ft
    // CAS(Mach 0.8) is 271.9 kt, 16 kt off IAS 256: valid, but out in a tie.
    let m = both(None);
    let gap = decode_bds60(&m).unwrap().ias_cas_gap_kt(Some(35000)).unwrap();
    assert!(gap > TIE_IAS_CAS_KT && gap <= MAX_IAS_CAS_KT, "{gap}");
    assert!(matches!(infer_kinematics(&m, Some(35000)), Some(Bds50(_))));
}

#[test]
fn turn_rate_physics() {
    // 25 deg of bank at 250 kt TAS: g tan(25) / v = 2.037 deg/s.
    let rate = 2.0372173474457114;
    let b = Bds50 { roll_deg: Some(25.0), true_track_deg: None, gs_kt: Some(240.0), track_rate_dps: Some(rate), tas_kt: Some(250.0) };
    assert!(b.turn_gap_dps().unwrap() < 1e-9);
    // Without TAS, GS stands in; without roll, rate or speed, no answer.
    let b = Bds50 { tas_kt: None, ..b };
    assert!((b.turn_gap_dps().unwrap() - rate * (250.0 / 240.0 - 1.0)).abs() < 1e-9);
    assert_eq!(Bds50 { roll_deg: None, ..b }.turn_gap_dps(), None);
    assert_eq!(Bds50 { track_rate_dps: None, ..b }.turn_gap_dps(), None);
    assert_eq!(Bds50 { gs_kt: Some(0.0), ..b }.turn_gap_dps(), None);
}

// ---------------------------------------------------------------------------
// ISA and velocity

#[test]
fn isa_matches_pymodes() {
    // pyModeS 2.22 aero.mach2cas / mach2tas / cas2tas, in knots. The knot
    // differs in the 7th digit (pyModeS 0.514444, here 1852/3600).
    for (mach, alt_ft, cas, tas) in [
        (0.42, 14000, 214.81032603755486, 264.11170306641003),
        (0.78, 35000, 264.3864864991666, 449.60699470111354),
        (0.3, 0, 198.44374980439324, 198.4437497722331),
        (0.85, 41000, 253.32508243715242, 487.5342495640162),
        (0.5, 20000, 227.86683321672368, 307.1586693193074),
    ] {
        let h = f64::from(alt_ft) * FT_TO_M;
        let got_tas = mach * speed_of_sound(h) / KT_TO_MS;
        assert!((got_tas - tas).abs() < 1e-3, "{mach} {alt_ft}: {got_tas}");
        let got_cas = tas_to_cas(mach * speed_of_sound(h), h) / KT_TO_MS;
        assert!((got_cas - cas).abs() < 1e-3, "{mach} {alt_ft}: {got_cas}");
        let b = Bds60 { mag_heading_deg: None, ias_kt: None, mach: Some(mach), baro_rate_fpm: None, inertial_rate_fpm: None };
        assert_eq!(b.tas_kt(alt_ft), Some(got_tas));
    }
    for (cas, alt_ft, tas) in [
        (250.0, 10000, 288.71179552545414),
        (280.0, 35000, 473.4955626513049),
        (150.0, 0, 149.9999999859513),
        (300.0, 45000, 609.9551404620198),
    ] {
        let h = f64::from(alt_ft) * FT_TO_M;
        let got = cas_to_tas(cas * KT_TO_MS, h) / KT_TO_MS;
        assert!((got - tas).abs() < 1e-3, "{cas} {alt_ft}: {got}");
        assert!((tas_to_cas(got * KT_TO_MS, h) / KT_TO_MS - cas).abs() < 1e-6);
        let b = Bds60 { mag_heading_deg: None, ias_kt: Some(cas), mach: None, baro_rate_fpm: None, inertial_rate_fpm: None };
        assert_eq!(b.tas_kt(alt_ft), Some(got));
        assert_eq!(Bds60 { ias_kt: None, ..b }.tas_kt(alt_ft), None);
    }
}

#[test]
fn ground_velocity_en() {
    let gs_ms = 400.0 * 1852.0 / 3600.0;
    let b = |trk: f64| Bds50 { roll_deg: None, true_track_deg: Some(trk), gs_kt: Some(400.0), track_rate_dps: None, tas_kt: None };
    for (trk, east, north) in [(0.0, 0.0, 1.0), (90.0, 1.0, 0.0), (180.0, 0.0, -1.0), (270.0, -1.0, 0.0)] {
        let (e, n) = b(trk).ground_velocity_en().unwrap();
        assert!((e - east * gs_ms).abs() < 1e-9 && (n - north * gs_ms).abs() < 1e-9, "{trk}: {e} {n}");
    }
    // pyModeS 5,0 vector: 438 kt on 114.26 deg, east-south-east.
    let (e, n) = decode_bds50(&frame_mb("A000139381951536E024D4CCF6B5")).unwrap().ground_velocity_en().unwrap();
    assert!((e.hypot(n) - 438.0 * 1852.0 / 3600.0).abs() < 1e-9);
    assert!((e.atan2(n).to_degrees() - 114.2578125).abs() < 1e-9);
    assert_eq!(Bds50 { gs_kt: None, ..b(90.0) }.ground_velocity_en(), None);
    assert_eq!(Bds50 { true_track_deg: None, ..b(90.0) }.ground_velocity_en(), None);
}

// ---------------------------------------------------------------------------
// Robustness

#[test]
fn arbitrary_mb_never_panics_and_stays_in_range() {
    let mut rng = XorShift(0x0DDB_A11C_0FFE_E000);
    let mut seen = [0usize; 4];
    for i in 0..200_000 {
        // Sparse MBs too: random ones almost never pass the status checks.
        let mut m = rng.mb();
        if i % 2 == 0 {
            let keep = rng.next();
            for (j, b) in m.iter_mut().enumerate() {
                *b &= (keep >> (8 * j)) as u8;
            }
        }
        let alt = (i % 3 != 0).then_some((rng.next() % 50_000) as i32 - 1_000);
        let (b50, b60) = (decode_bds50(&m), decode_bds60(&m));
        if let Some(b) = b50 {
            assert!(within(b.roll_deg, 50.0) && within(b.gs_kt, 800.0) && within(b.tas_kt, 800.0));
            assert!(b.true_track_deg.is_none_or(|t| (0.0..360.0).contains(&t)));
        }
        if let Some(b) = b60 {
            assert!(within(b.ias_kt, 500.0) && within(b.mach, 1.0));
            assert!(within(b.baro_rate_fpm, 6000.0) && within(b.inertial_rate_fpm, 6000.0));
            assert!(b.mag_heading_deg.is_none_or(|t| (0.0..360.0).contains(&t)));
        }
        // Whatever is inferred is exactly what the decoders give.
        let k = infer_kinematics(&m, alt);
        match k {
            None => seen[0] += 1,
            Some(Kinematics::Bds50(b)) => {
                assert_eq!(Some(b), b50);
                seen[1] += 1;
            }
            Some(Kinematics::Bds60(b)) => {
                assert_eq!(Some(b), b60);
                seen[2] += 1;
            }
            Some(Kinematics::Ambiguous(a, b)) => {
                assert_eq!((Some(a), Some(b)), (b50, b60));
                seen[3] += 1;
            }
        }
    }
    assert!(seen.iter().all(|&n| n > 0), "{seen:?}");
}
