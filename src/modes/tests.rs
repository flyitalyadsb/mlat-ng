//! Tests for the Mode S decoder. Published vectors come from "The 1090MHz
//! Riddle" and pyModeS; the rest is checked against independent
//! implementations written here (bitwise CRC, CPR encoder, closed-form NL)
//! and against straight ports of mlat-server's own algorithms, so parity with
//! the production server is tested, not assumed.

use super::*;

fn frame(h: &str) -> Vec<u8> {
    hex::decode(h).unwrap()
}

/// Reference CRC: polynomial long division one bit at a time over the whole
/// frame, nothing shared with the table-driven implementation.
fn crc_bitwise(f: &[u8]) -> u32 {
    const GEN: u128 = 0x1FF_F409;
    let nbits = f.len() * 8;
    let v = f.iter().fold(0u128, |acc, &b| (acc << 8) | u128::from(b));
    let parity = (v & 0xFF_FFFF) as u32;
    let mut rem = (v >> 24) << 24;
    for i in (24..nbits).rev() {
        if rem >> i & 1 == 1 {
            rem ^= GEN << (i - 24);
        }
    }
    rem as u32 ^ parity
}

/// Set the parity field so that the residual becomes `overlay` (0 for
/// DF11/17/18, the address for AP formats).
fn with_parity(mut f: Vec<u8>, overlay: u32) -> Vec<u8> {
    let n = f.len();
    f[n - 3..].fill(0);
    let p = crc_residual(&f) ^ overlay;
    f[n - 3..].copy_from_slice(&p.to_be_bytes()[1..]);
    f
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
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next() as u8).collect()
    }
}

// ---------------------------------------------------------------------------
// CRC and address

#[test]
fn crc_table_matches_bitwise_division() {
    let mut rng = XorShift(0x9E37_79B9_7F4A_7C15);
    for _ in 0..20_000 {
        let mut f = rng.bytes(14);
        // both lengths: the DF decides how many bytes are covered
        if rng.next() & 1 == 0 {
            f[0] &= 0x7f;
            f.truncate(7);
        } else {
            f[0] |= 0x80;
        }
        assert_eq!(crc_residual(&f), crc_bitwise(&f), "{}", hex::encode(&f));
    }
}

#[test]
fn crc_of_valid_squitters_is_zero_and_catches_every_single_bit_error() {
    for h in [
        "8D406B902015A678D4D220AA4BDA",
        "8D4840D6202CC371C32CE0576098",
        "8D40621D58C382D690C8AC2863A7",
        "8D40621D58C386435CC412692AD6",
        "8D485020994409940838175B284F",
        "8DA05F219B06B6AF189400CBC33F",
    ] {
        let f = frame(h);
        assert_eq!(crc_residual(&f), 0, "{h}");
        // Flip each bit after the 5 DF bits (flipping those changes the DF).
        for bit in 5..112 {
            let mut g = f.clone();
            g[bit / 8] ^= 0x80 >> (bit % 8);
            assert_ne!(crc_residual(&g), 0, "{h} bit {bit}");
            assert_eq!(decode(&g).unwrap().crc_ok, Some(false), "{h} bit {bit}");
        }
    }
}

#[test]
fn crc_residual_length_comes_from_the_df() {
    let f = frame("8D4840D6202CC371C32CE0576098");
    assert_eq!(crc_residual(&f[..13]), u32::MAX);
    assert_eq!(crc_residual(&[]), u32::MAX);
    // A DF4 is covered over 7 bytes even if more follow (mlat-server's Cython).
    let df4 = frame("20001838CA3804");
    let mut longer = df4.clone();
    longer.extend_from_slice(&[0xde, 0xad, 0xbe, 0xef]);
    assert_eq!(crc_residual(&longer), crc_residual(&df4));
    assert_eq!(crc_residual(&df4[..6]), u32::MAX);
}

#[test]
fn ap_address_recovery_pymodes_vectors() {
    // pyModeS test_icao: DF20 Comm-B altitude replies, address from AP.
    for (h, addr, alt) in [
        ("A0001839CA3800315800007448D9", 0x400940, 38025),
        ("A000139381951536E024D4CCF6B5", 0x3C4DD2, 30275),
        ("A000029CFFBAA11E2004727281F1", 0x4243D0, 3300),
    ] {
        let d = decode(&frame(h)).unwrap();
        assert_eq!(d.df, 20);
        assert_eq!(d.address, addr, "{h}");
        assert_eq!(d.crc_ok, None);
        assert_eq!(d.altitude_ft, Some(alt), "{h}");
        assert_eq!(d.squawk, None);
    }
}

#[test]
fn ap_address_round_trip_for_every_ap_format() {
    let mut rng = XorShift(42);
    for df in [0u8, 4, 5, 16, 20, 21] {
        for _ in 0..200 {
            let addr = (rng.next() as u32) & 0xFF_FFFF;
            let mut f = rng.bytes(frame_len(df));
            f[0] = (df << 3) | (f[0] & 7);
            let f = with_parity(f, addr);
            let d = decode(&f).unwrap();
            assert_eq!((d.df, d.address, d.crc_ok), (df, addr, None));
        }
    }
}

// ---------------------------------------------------------------------------
// DF-specific decoding

#[test]
fn df17_identification() {
    for (h, addr, cs) in [
        ("8D4840D6202CC371C32CE0576098", 0x4840D6, "KLM1023"),
        ("8D406B902015A678D4D220AA4BDA", 0x406B90, "EZY85MH"),
    ] {
        let d = decode(&frame(h)).unwrap();
        assert_eq!(d.df, 17);
        assert_eq!(d.address, addr);
        assert_eq!(d.crc_ok, Some(true));
        assert_eq!(d.callsign.as_deref(), Some(cs));
        assert_eq!(d.altitude_ft, None);
        let es = d.es.unwrap();
        assert_eq!((es.kind, es.tc, es.nuc), (EsKind::Identification, 4, None));
    }
}

#[test]
fn df17_airborne_position_pair_1090_riddle() {
    let even = decode(&frame("8D40621D58C382D690C8AC2863A7")).unwrap();
    let odd = decode(&frame("8D40621D58C386435CC412692AD6")).unwrap();
    for d in [&even, &odd] {
        assert_eq!((d.df, d.address, d.crc_ok), (17, 0x40621D, Some(true)));
        assert_eq!(d.altitude_ft, Some(38000));
        assert_eq!(d.callsign, None);
    }
    let (e, o) = (even.es.unwrap(), odd.es.unwrap());
    for es in [&e, &o] {
        assert_eq!((es.kind, es.tc, es.nuc), (EsKind::AirbornePosition, 11, Some(7)));
    }
    assert_eq!((e.odd, e.lat_cpr, e.lon_cpr), (false, 93000, 51372));
    assert_eq!((o.odd, o.lat_cpr, o.lon_cpr), (true, 74158, 50194));

    let (lat_e, lon_e, lat_o, lon_o) = cpr_global_airborne(e.lat_cpr, e.lon_cpr, o.lat_cpr, o.lon_cpr).unwrap();
    // The riddle's published fix (even message most recent): 52.2572, 3.9194.
    assert!((lat_e - 52.2572).abs() < 5e-5 && (lon_e - 3.9194).abs() < 5e-5);
    // Exact values: 6*(8 + 93000/2^17) and 10*(51372/2^17) are binary fractions.
    assert_eq!(lat_e, 52.2572021484375);
    assert_eq!(lon_e, 3.91937255859375);
    assert!((lat_o - 52.26578017412606).abs() < 1e-12);
    assert!((lon_o - 3.938912527901786).abs() < 1e-12);
}

#[test]
fn df17_velocity_and_other() {
    for h in ["8D485020994409940838175B284F", "8DA05F219B06B6AF189400CBC33F"] {
        let d = decode(&frame(h)).unwrap();
        let es = d.es.unwrap();
        assert_eq!((es.kind, es.tc, es.nuc), (EsKind::Velocity, 19, None));
        assert_eq!((d.altitude_ft, d.callsign), (None, None));
    }
}

#[test]
fn es_type_code_table_matches_mlat_server() {
    use EsKind::*;
    // mlat-server modes_cython/message.pyx `es_types`, transcribed entry by entry.
    let expected: [(EsKind, Option<u8>); 32] = [
        (AirbornePosition, Some(0)), // 0
        (Identification, None),
        (Identification, None),
        (Identification, None),
        (Identification, None), // 4
        (SurfacePosition, Some(9)),
        (SurfacePosition, Some(8)),
        (SurfacePosition, Some(7)),
        (SurfacePosition, Some(6)), // 8
        (AirbornePosition, Some(9)),
        (AirbornePosition, Some(8)),
        (AirbornePosition, Some(7)),
        (AirbornePosition, Some(6)),
        (AirbornePosition, Some(5)),
        (AirbornePosition, Some(4)),
        (AirbornePosition, Some(3)),
        (AirbornePosition, Some(2)),
        (AirbornePosition, Some(1)),
        (AirbornePosition, Some(0)), // 18
        (Velocity, None),            // 19
        (AirbornePosition, Some(9)), // 20
        (AirbornePosition, Some(8)),
        (AirbornePosition, Some(0)), // 22
        (Other, None),
        (Other, None),
        (Other, None),
        (Other, None),
        (Other, None),
        (Other, None),
        (Other, None),
        (Other, None),
        (Other, None),
    ];
    let base = frame("8D40621D58C382D690C8AC2863A7");
    for tc in 0..32u8 {
        let mut f = base.clone();
        f[4] = (tc << 3) | (f[4] & 7);
        let d = decode(&with_parity(f, 0)).unwrap();
        let es = d.es.unwrap();
        assert_eq!((es.kind, es.nuc), expected[tc as usize], "tc {tc}");
        assert_eq!(es.tc, tc);
        // Altitude only from airborne position (same AC12 bits for all TCs).
        let want_alt = (expected[tc as usize].0 == AirbornePosition).then_some(38000);
        assert_eq!(d.altitude_ft, want_alt, "tc {tc}");
    }
}

#[test]
fn surface_position_fills_cpr_but_not_altitude() {
    let mut f = frame("8D40621D58C386435CC412692AD6");
    f[4] = (7 << 3) | (f[4] & 7);
    let d = decode(&with_parity(f, 0)).unwrap();
    let es = d.es.unwrap();
    assert_eq!((es.kind, es.nuc), (EsKind::SurfacePosition, Some(7)));
    assert_eq!((es.odd, es.lat_cpr, es.lon_cpr), (true, 74158, 50194));
    assert_eq!(d.altitude_ft, None);
}

#[test]
fn df18_decodes_like_df17() {
    let mut f = frame("8D4840D6202CC371C32CE0576098");
    f[0] = (18 << 3) | 2; // CF 2 (TIS-B fine) is not looked at
    let d = decode(&with_parity(f.clone(), 0)).unwrap();
    assert_eq!((d.df, d.address, d.crc_ok), (18, 0x4840D6, Some(true)));
    assert_eq!(d.callsign.as_deref(), Some("KLM1023"));
    assert_eq!(decode(&f).unwrap().crc_ok, Some(false)); // DF17 parity on a DF18
}

#[test]
fn es_identification_padding_and_unknown_characters() {
    // Pack 8 six-bit codes into a TC 4 frame.
    let ident = |codes: [u8; 8]| {
        let v = codes.iter().fold(0u64, |acc, &c| (acc << 6) | u64::from(c));
        let mut f = vec![0x8D, 0x48, 0x40, 0xD6, 0x20];
        f.extend_from_slice(&v.to_be_bytes()[2..]);
        f.extend_from_slice(&[0, 0, 0]);
        decode(&with_parity(f, 0)).unwrap()
    };
    assert_eq!(ident([32; 8]).callsign, None); // all spaces
    assert_eq!(ident([0; 8]).callsign, None); // code 0 is a space too in mlat-server's charset
    // 'A', 'B', 27 ('?'), '1', spaces: mlat-server keeps the '?' for ES.
    assert_eq!(ident([1, 2, 27, 49, 32, 32, 32, 32]).callsign.as_deref(), Some("AB?1"));
    // inner spaces survive, only the padding goes
    assert_eq!(ident([1, 32, 2, 32, 32, 32, 32, 32]).callsign.as_deref(), Some("A B"));
}

#[test]
fn df11_all_call() {
    // pyModeS all-call example: PI overlaid with interrogator code 0x16.
    let d = decode(&frame("5D484FDEA248F5")).unwrap();
    assert_eq!((d.df, d.address, d.crc_ok), (11, 0x484FDE, None));
    assert_eq!(crc_residual(&frame("5D484FDEA248F5")), 0x16);

    let squitter = with_parity(frame("5D484FDE000000"), 0);
    assert_eq!(decode(&squitter).unwrap().crc_ok, Some(true));
    for ii in 1..=0x7f {
        assert_eq!(decode(&with_parity(squitter.clone(), ii)).unwrap().crc_ok, None);
    }
    assert_eq!(decode(&with_parity(squitter.clone(), 0x80)).unwrap().crc_ok, Some(false));
    let mut bad = squitter.clone();
    bad[2] ^= 0x01;
    assert_eq!(decode(&bad).unwrap().crc_ok, Some(false));
    assert_eq!(decode(&bad).unwrap().address, 0x484EDE); // AA is taken as is
}

#[test]
fn df0_df4_df16_altitude() {
    let d = decode(&frame("02E197B00179C3")).unwrap();
    assert_eq!((d.df, d.altitude_ft, d.address), (0, Some(37000), 0x4B18FE));

    // AC field 0x1838 is the riddle's 38000 ft (Q=1); AP set for 40621D.
    let df4 = with_parity(frame("20001838000000"), 0x40621D);
    let d = decode(&df4).unwrap();
    assert_eq!((d.df, d.altitude_ft, d.address, d.crc_ok, d.squawk), (4, Some(38000), 0x40621D, None, None));

    let df16 = with_parity(frame("80E1983858B9D53C4D2100000000"), 0x3C4DD2);
    let d = decode(&df16).unwrap();
    assert_eq!((d.df, d.altitude_ft, d.address), (16, Some(38000), 0x3C4DD2));
}

#[test]
fn df5_df21_squawk_and_comm_b() {
    // pyModeS idcode vectors.
    let d = decode(&frame("2A00516D492B80")).unwrap();
    assert_eq!((d.df, d.squawk, d.altitude_ft), (5, Some(356), None));
    let d = decode(&frame("A800292DFFBBA9383FFCEB903D01")).unwrap();
    assert_eq!((d.df, d.squawk, d.callsign), (21, Some(1346), None)); // MB is not BDS 2,0

    // DF20 carrying BDS 2,0 (riddle Comm-B example).
    let d = decode(&frame("A000083E202CC371C31DE0AA1CCF")).unwrap();
    assert_eq!((d.df, d.address), (20, 0x484163));
    assert_eq!(d.callsign.as_deref(), Some("KLM1017"));
    assert_eq!(d.altitude_ft, Some(12550));

    // Same MB in a DF21: callsign plus squawk.
    let mut f = frame("A800083E202CC371C31DE0AA1CCF");
    f = with_parity(f, 0x484163);
    let d = decode(&f).unwrap();
    assert_eq!((d.df, d.callsign.as_deref(), d.address), (21, Some("KLM1017"), 0x484163));
}

#[test]
fn comm_b_callsign_filter() {
    let mb = |chars: [u8; 8]| {
        let v = chars.iter().fold(0u64, |acc, &c| (acc << 6) | u64::from(c));
        let mut f = vec![0xA0, 0x00, 0x08, 0x3E, 0x20];
        f.extend_from_slice(&v.to_be_bytes()[2..]);
        f.extend_from_slice(&[0, 0, 0]);
        decode(&with_parity(f, 0x484163)).unwrap().callsign
    };
    assert_eq!(mb([1, 2, 3, 49, 50, 32, 32, 32]).as_deref(), Some("ABC12"));
    assert_eq!(mb([32; 8]), None); // blank
    assert_eq!(mb([1, 2, 3, 27, 50, 32, 32, 32]), None); // '?' -> not BDS 2,0
    assert_eq!(mb([1, 2, 3, 63, 50, 32, 32, 32]), None);
}

#[test]
fn length_and_df_rejection() {
    let es = frame("8D4840D6202CC371C32CE0576098");
    assert!(decode(&es[..7]).is_none()); // long DF in a short frame
    let mut es15 = es.clone();
    es15.push(0);
    assert!(decode(&es15).is_none());
    let df4 = frame("20001838CA3804");
    let mut df4_long = df4.clone();
    df4_long.extend_from_slice(&[0; 7]);
    assert!(decode(&df4_long).is_none()); // short DF in a long frame
    assert!(decode(&[]).is_none());
    for df in [1u8, 2, 3, 6, 7, 8, 9, 10, 12, 13, 14, 15] {
        assert!(decode(&[df << 3, 0, 0, 0, 0, 0, 0]).is_none(), "DF{df}");
    }
    for df in [19u8, 22, 23, 24, 25, 31] {
        let mut f = vec![df << 3];
        f.extend_from_slice(&[0; 13]);
        assert!(decode(&f).is_none(), "DF{df}");
    }
}

// ---------------------------------------------------------------------------
// Altitude and squawk

/// mlat-server `_decode_ac13`, ported line by line (including its Gillham
/// bug), to pin down where this module agrees with it and where it doesn't.
fn mlat_server_ac13(ac13: u16) -> Option<i32> {
    let ac13 = ac13 & 0x1fff;
    if ac13 == 0 || ac13 & 0x0040 != 0 {
        return None;
    }
    if ac13 & 0x0010 != 0 {
        let n = ((ac13 & 0x1f80) >> 2) | ((ac13 & 0x0020) >> 1) | (ac13 & 0x000f);
        return Some(i32::from(n) * 25 - 1000);
    }
    if ac13 & 0x1500 == 0 {
        return None;
    }
    let mut h = 0i32;
    for (b, x) in [(0x1000, 7), (0x0400, 3), (0x0100, 1)] {
        if ac13 & b != 0 {
            h ^= x;
        }
    }
    if h & 5 != 0 {
        h ^= 5;
    }
    if h > 5 {
        return None;
    }
    let mut f = 0i32;
    let five_hundreds = [
        (0x0010, 0x1ff), // D1
        (0x0004, 0x0ff), // D2
        (0x0001, 0x07f), // D4
        (0x0800, 0x03f), // A1
        (0x0200, 0x01f), // A2
        (0x0080, 0x00f), // A4
        (0x0020, 0x007), // B1
        (0x0008, 0x003), // B2
        (0x0002, 0x001), // B4
    ];
    for (b, x) in five_hundreds {
        if ac13 & b != 0 {
            f ^= x;
        }
    }
    if f & 1 != 0 {
        h = 6 - h;
    }
    let a = 500 * f + 100 * h - 1300;
    (a >= -1200).then_some(a)
}

/// AC13 bit -> readsb Mode A bit (C1 A1 C2 A2 C4 A4 M B1 Q B2 D2 B4 D4 vs
/// 00 A4 A2 A1 | 00 B4 B2 B1 | SPI C4 C2 C1 | 00 D4 D2 D1). M and Q have no
/// Mode A counterpart; the Q position is D1 in Gillham.
const AC13_TO_MODE_A: [(u16, u16); 12] = [
    (0x1000, 0x0010), // C1
    (0x0800, 0x1000), // A1
    (0x0400, 0x0020), // C2
    (0x0200, 0x2000), // A2
    (0x0100, 0x0040), // C4
    (0x0080, 0x4000), // A4
    (0x0020, 0x0100), // B1
    (0x0010, 0x0001), // D1 (Q)
    (0x0008, 0x0200), // B2
    (0x0004, 0x0002), // D2
    (0x0002, 0x0400), // B4
    (0x0001, 0x0004), // D4
];

fn ac13_to_mode_a(ac13: u16) -> u16 {
    AC13_TO_MODE_A.iter().filter(|&&(s, _)| ac13 & s != 0).fold(0, |a, &(_, m)| a | m)
}

fn mode_a_to_ac13(mode_a: u16) -> u16 {
    AC13_TO_MODE_A.iter().filter(|&&(_, m)| mode_a & m != 0).fold(0, |a, &(s, _)| a | s)
}

/// Spot values of the published Mode C table (aeroelectric.com, shipped as
/// mlat-client's mode_c_altitude_by_squawk.py), in readsb's layout. The whole
/// 1280-entry table was checked against `mode_a_to_mode_c` once; these keep it.
const MODE_C_TABLE: [(u16, i32); 21] = [
    (0x0040, -1200),
    (0x0620, 0),
    (0x6610, 8200),
    (0x6520, 10000),
    (0x3560, 17900),
    (0x1560, 27600),
    (0x5124, 35000),
    (0x5614, 37300),
    (0x5424, 38000),
    (0x2024, 47000),
    (0x2534, 50100),
    (0x2514, 50200),
    (0x4244, 56700),
    (0x0136, 66400),
    (0x2736, 76100),
    (0x7446, 85800),
    (0x1422, 95500),
    (0x7312, 105200),
    (0x6162, 114900),
    (0x0362, 124600),
    (0x0042, 126700),
];

#[test]
fn ac13_matches_mlat_server_except_gillham() {
    let mut gillham_diffs = 0;
    for ac13 in 0..0x2000u16 {
        let gillham = ac13 != 0 && ac13 & 0x0050 == 0;
        if gillham {
            gillham_diffs += usize::from(decode_ac13(ac13) != mlat_server_ac13(ac13));
        } else {
            assert_eq!(decode_ac13(ac13), mlat_server_ac13(ac13), "{ac13:#06x}");
        }
    }
    // 768 values off by 300 ft, 256 valid codes rejected, 511 illegal codes
    // accepted by mlat-server. C4 alone is -1200 ft (the lowest Gillham
    // altitude); mlat-server reads -900.
    assert_eq!(gillham_diffs, 1535);
    assert_eq!(decode_ac13(0x0100), Some(-1200));
    assert_eq!(mlat_server_ac13(0x0100), Some(-900));
}

#[test]
fn gillham_ac13_agrees_with_readsb_on_every_code() {
    let mut valid = 0;
    for ac13 in 0..0x2000u16 {
        if ac13 & 0x0050 != 0 {
            continue; // M or Q set: not Gillham
        }
        let readsb = mode_a_to_mode_c(ac13_to_mode_a(ac13)).map(|c| c * 100);
        assert_eq!(decode_ac13(ac13), readsb, "{ac13:#06x}");
        valid += usize::from(readsb.is_some());
    }
    assert_eq!(valid, 1280); // -1200 ..= 126700 ft in 100 ft steps
}

#[test]
fn gillham_published_table() {
    for (mode_a, alt) in MODE_C_TABLE {
        assert_eq!(mode_a_to_mode_c(mode_a), Some(alt / 100), "{mode_a:04x}");
        assert_eq!(decode_ac13(mode_a_to_ac13(mode_a)), Some(alt), "{mode_a:04x}");
    }
}

#[test]
fn ac13_and_ac12_rules() {
    assert_eq!(decode_ac13(0), None);
    assert_eq!(decode_ac13(0x1838 | 0x0040), None); // M bit: metric
    assert_eq!(decode_ac13(0x1838), Some(38000));
    assert_eq!(decode_ac13(0x0010), Some(-1000)); // Q set, count 0
    assert_eq!(decode_ac13(0x1fbf), Some(2047 * 25 - 1000)); // Q set, all ones
    assert_eq!(decode_ac13(0xe000 | 0x1838), Some(38000)); // bits above 13 ignored
    // AC12 0xC38 from the riddle position frames.
    assert_eq!(decode_ac12(0xC38), Some(38000));
    assert_eq!(decode_ac12(0), None);
    for ac12 in 0..0x1000u16 {
        assert_eq!(decode_ac12(ac12), decode_ac13(((ac12 & 0xfc0) << 1) | (ac12 & 0x3f)));
    }
}

/// mlat-server's squawk lookup tables, ported, as a parity oracle.
fn mlat_server_id13(id13: u16) -> u16 {
    let mut ut = 0u16;
    let hi = id13 & 0x1f80;
    let upper = [
        (0x1000, 0x0010), // C1
        (0x0800, 0x1000), // A1
        (0x0400, 0x0020), // C2
        (0x0200, 0x2000), // A2
        (0x0100, 0x0040), // C4
        (0x0080, 0x4000), // A4
    ];
    for (b, v) in upper {
        if hi & b != 0 {
            ut |= v;
        }
    }
    let mut lt = 0u16;
    for (b, v) in [(0x20, 0x0100), (0x10, 0x0001), (0x08, 0x0200), (0x04, 0x0002), (0x02, 0x0400), (0x01, 0x0004)] {
        if id13 & b != 0 {
            lt |= v;
        }
    }
    // '{0:04x}'.format(...) then read as a number
    format!("{:04x}", ut | lt).parse().unwrap()
}

#[test]
fn id13_matches_mlat_server_for_every_code() {
    for id13 in 0..0x2000u16 {
        assert_eq!(decode_id13(id13), mlat_server_id13(id13), "{id13:#06x}");
    }
    assert_eq!(decode_id13(0x0040), 0); // X bit ignored
    // 7000: A = 7 is A1 A2 A4 = 0x0800 | 0x0200 | 0x0080
    assert_eq!(decode_id13(0x0a80), 7000);
    assert_eq!(decode_id13(0x1fbf), 7777);
}

// ---------------------------------------------------------------------------
// CPR

/// Closed-form NL (DO-260B A.1.7.2.d), independent of the NL_LAT table.
fn nl_formula(lat: f64) -> i64 {
    use std::f64::consts::PI;
    let lat = lat.abs();
    if lat == 0.0 {
        return 59;
    }
    if lat == 87.0 {
        return 2;
    }
    if lat > 87.0 {
        return 1;
    }
    let a = 1.0 - (PI / 30.0).cos();
    let b = (PI / 180.0 * lat).cos().powi(2);
    (2.0 * PI / (1.0 - a / b).acos()).floor() as i64
}

/// Airborne CPR encoder (DO-260B A.1.7.3), for round trips.
fn cpr_encode(lat: f64, lon: f64, odd: bool) -> (u32, u32) {
    let i = if odd { 1.0 } else { 0.0 };
    let dlat = 360.0 / (60.0 - i);
    let yz = (CPR_SCALE * lat.rem_euclid(dlat) / dlat + 0.5).floor();
    let rlat = dlat * (yz / CPR_SCALE + (lat / dlat).floor());
    let dlon = 360.0 / ((nl_formula(rlat) as f64) - i).max(1.0);
    let xz = (CPR_SCALE * lon.rem_euclid(dlon) / dlon + 0.5).floor();
    ((yz as u32) & 0x1ffff, (xz as u32) & 0x1ffff)
}

fn lon_diff(a: f64, b: f64) -> f64 {
    ((a - b + 540.0).rem_euclid(360.0) - 180.0).abs()
}

#[test]
fn nl_table_agrees_with_closed_form() {
    for lat in (0..=9000).map(|i| f64::from(i) * 0.01) {
        let near_edge = NL_LAT.iter().any(|&e| (lat - e).abs() < 1e-6);
        if !near_edge {
            assert_eq!(nl(lat), nl_formula(lat), "{lat}");
            assert_eq!(nl(-lat), nl_formula(lat), "-{lat}");
        }
    }
    for (i, &edge) in NL_LAT.iter().enumerate().take(57) {
        assert_eq!(nl(edge - 1e-6), 59 - i as i64);
        assert_eq!(nl(edge), 58 - i as i64);
    }
}

#[test]
fn cpr_round_trip_over_the_globe() {
    let mut checked = 0;
    let mut lat = -84.9;
    while lat < 85.0 {
        let mut lon = -179.95;
        while lon < 180.0 {
            // Stay clear of NL edges, where a pair legitimately fails.
            if nl_formula(lat - 0.01) == nl_formula(lat + 0.01) {
                let (le, ne) = cpr_encode(lat, lon, false);
                let (lo, no) = cpr_encode(lat, lon, true);
                let (rle, rne, rlo, rno) = cpr_global_airborne(le, ne, lo, no).unwrap_or_else(|| panic!("{lat} {lon}"));
                let n = nl_formula(lat) as f64;
                // Half an LSB of rounding in the encoder, plus slack.
                assert!((rle - lat).abs() < 6.0 / CPR_SCALE, "{lat} {lon} -> {rle}");
                assert!((rlo - lat).abs() < 6.2 / CPR_SCALE, "{lat} {lon} -> {rlo}");
                assert!(lon_diff(rne, lon) < 360.0 / n / CPR_SCALE, "{lat} {lon} -> {rne}");
                assert!(lon_diff(rno, lon) < 360.0 / (n - 1.0).max(1.0) / CPR_SCALE, "{lat} {lon} -> {rno}");
                assert!((-180.0..180.0).contains(&rne) && (-180.0..180.0).contains(&rno));
                checked += 1;
            }
            lon += 7.31;
        }
        lat += 0.53;
    }
    assert!(checked > 15_000, "{checked}");
}

#[test]
fn cpr_rejects_zone_straddle_and_bad_input() {
    // Even just south of the NL 37/36 edge, odd just north of it: each half
    // decodes fine but the zone counts differ.
    let edge = NL_LAT[22]; // 51.89342469
    let (le, ne) = cpr_encode(edge - 0.002, 4.0, false);
    let (lo, no) = cpr_encode(edge + 0.002, 4.0, true);
    assert_eq!(cpr_global_airborne(le, ne, lo, no), None);
    // Same pair on one side of the edge is fine.
    let (lo, no) = cpr_encode(edge - 0.002, 4.0, true);
    assert!(cpr_global_airborne(le, ne, lo, no).is_some());

    // j = -30 puts the even latitude at 180 degrees: out of range.
    assert_eq!(cpr_global_airborne(0, 0, 65536, 0), None);
    // Wider than 17 bits cannot come from a frame.
    assert_eq!(cpr_global_airborne(1 << 17, 0, 0, 0), None);
    assert_eq!(cpr_global_airborne(0, 0, 0, u32::MAX), None);
}

#[test]
fn cpr_southern_and_western_hemispheres() {
    for (lat, lon) in [(-33.9461, 151.1772), (-54.8, -68.3), (40.6413, -73.7781), (-0.01, -0.01), (0.01, 179.99)] {
        let (le, ne) = cpr_encode(lat, lon, false);
        let (lo, no) = cpr_encode(lat, lon, true);
        let (rle, rne, _, _) = cpr_global_airborne(le, ne, lo, no).unwrap();
        assert!((rle - lat).abs() < 1e-4 && lon_diff(rne, lon) < 1e-3, "{lat} {lon} -> {rle} {rne}");
    }
}

// ---------------------------------------------------------------------------
// false_ac

/// mlat-client coordinator.py / acbridge.py `false_ac_message`, verbatim:
/// b'\x5d' + (int(hex4.ljust(6, '0')) % 1000000).to_bytes(3, 'big') + b'\xa4\x92\x4f'
fn python_false_ac(hex4: &str) -> (u32, Vec<u8>) {
    let potential_squawk: u32 = format!("{hex4:0<6}").parse().unwrap();
    let addr = potential_squawk % 1_000_000;
    let mut f = vec![0x5d];
    f.extend_from_slice(&addr.to_be_bytes()[1..]);
    f.extend_from_slice(&[0xa4, 0x92, 0x4f]);
    (addr, f)
}

#[test]
fn false_ac_squawk_7000() {
    let (addr, f) = python_false_ac("7000");
    assert_eq!(addr, 700_000);
    assert_eq!(hex::encode(&f), "5d0aae60a4924f");
    let ac = false_ac(&f).unwrap();
    assert_eq!(ac, ModeAc { code: 0x7000, squawk: 7000, altitude_ft: None }); // no C bits: not an altitude
    let d = decode(&f).unwrap();
    assert_eq!((d.df, d.address), (11, 700_000));
}

#[test]
fn false_ac_round_trip_every_code() {
    let mut n = 0;
    for v in 0..0x10000u32 {
        let hex4 = format!("{v:04x}");
        if !hex4.bytes().all(|c| (b'0'..=b'8').contains(&c)) {
            continue;
        }
        n += 1;
        let (addr, f) = python_false_ac(&hex4);
        assert!(is_false_ac_address(addr), "{hex4}");
        let ac = false_ac(&f).unwrap_or_else(|| panic!("{hex4}"));
        assert_eq!(u32::from(ac.code), v);
        assert_eq!(ac.squawk, hex_digits_to_dec((v & 0x7777) as u16));
        assert_eq!(ac.altitude_ft, mode_a_to_mode_c(v as u16).map(|c| c * 100));
        // The server sees an ordinary DF11 for the fake address; its made-up
        // PI never passes for a valid one, so crc_ok is always false.
        let d = decode(&f).unwrap();
        assert_eq!((d.df, d.address, d.crc_ok), (11, addr, Some(false)), "{hex4}");
    }
    assert_eq!(n, 9 * 9 * 9 * 9);
}

#[test]
fn false_ac_mode_c_and_ident_readings() {
    for (mode_a, alt) in MODE_C_TABLE {
        let (_, f) = python_false_ac(&format!("{mode_a:04x}"));
        let ac = false_ac(&f).unwrap();
        assert_eq!(ac.altitude_ft, Some(alt));
        assert_eq!(ac.squawk, hex_digits_to_dec(mode_a));
    }
    // 7000 with the ident (SPI) bit: identity 7000, never an altitude.
    let ac = false_ac(&python_false_ac("7080").1).unwrap();
    assert_eq!(ac, ModeAc { code: 0x7080, squawk: 7000, altitude_ft: None });
    // SPI in a false_ac payload always means a bare C nibble of 8.
    for v in 0..0x10000u32 {
        let hex4 = format!("{v:04x}");
        if v & 0x80 != 0 && hex4.bytes().all(|c| (b'0'..=b'8').contains(&c)) {
            assert_eq!(mode_a_to_mode_c(v as u16), None, "{hex4}");
        }
    }
    // readsb's Beast heartbeat is a Mode A/C frame of zeros: code 0000.
    assert_eq!(false_ac(&python_false_ac("0000").1), Some(ModeAc { code: 0, squawk: 0, altitude_ft: None }));
}

#[test]
fn false_ac_address_shape() {
    let real_looking = [0x0A4061, 0x06A1C9, 0x0D0BBA, 0x484FDE, 0x40621D];
    let wrong_shape = [999_900, 1_000_000, 123_450, 900_000, 0xFF_FFFF, 700_001];
    for a in real_looking.into_iter().chain(wrong_shape) {
        assert!(!is_false_ac_address(a), "{a}");
    }
    for a in [700_000, 123_400, 0, 4_000, 888_800, 100, 80_800] {
        assert!(is_false_ac_address(a), "{a}");
    }
}

#[test]
fn false_ac_rejects_genuine_frames() {
    for h in [
        "5D484FDEA248F5",               // real DF11, same first byte
        "8D4840D6202CC371C32CE0576098", // DF17
        "20001838CA3804",               // DF4
        "5D0A4061A4924F",               // right wrapper, real-looking ICAO
        "5D0AAE61A4924F",               // 700001
        "5D0AAE60A4924E",               // tail off by one
        "5C0AAE60A4924F",               // head off by one
    ] {
        assert_eq!(false_ac(&frame(h)), None, "{h}");
    }
    assert_eq!(false_ac(&frame("5D0AAE60A4924F00")), None); // too long
    assert_eq!(false_ac(&frame("5D0AAE60A492")), None); // too short
}

// ---------------------------------------------------------------------------
// Robustness

#[test]
fn never_panics_on_arbitrary_input() {
    let mut rng = XorShift(0xDEAD_BEEF_CAFE_F00D);
    let check = |buf: &[u8]| {
        let _ = crc_residual(buf);
        let _ = false_ac(buf);
        if let Some(d) = decode(buf) {
            assert_eq!(buf.len(), frame_len(d.df));
            assert_eq!(d.df, buf[0] >> 3);
            assert!(d.crc_ok.is_some() || !matches!(d.df, 17 | 18));
        }
    };
    for len in 0..=40 {
        for _ in 0..3_000 {
            check(&rng.bytes(len));
        }
    }
    for first in 0..=255u8 {
        for len in 0..=30 {
            for fill in [0x00, 0xff, 0x1a, 0x55] {
                let mut buf = vec![fill; len];
                if let Some(b) = buf.first_mut() {
                    *b = first;
                }
                check(&buf);
            }
        }
    }
    for v in 0..=u16::MAX {
        let _ = decode_ac13(v);
        let _ = decode_ac12(v);
        let sq = decode_id13(v);
        assert!(sq <= 7777 && sq.to_string().bytes().all(|c| c <= b'7'));
    }
    for _ in 0..200_000 {
        let r = rng.next();
        let v = [0, 17, 34, 47].map(|s| (r >> s) as u32 & 0x1ffff);
        if let Some((a, b, c, d)) = cpr_global_airborne(v[0], v[1], v[2], v[3]) {
            assert!((-90.0..=90.0).contains(&a) && (-90.0..=90.0).contains(&c));
            assert!((-180.0..180.0).contains(&b) && (-180.0..180.0).contains(&d));
        }
    }
    for v in [0, 1, 0x1ffff, 0x20000, u32::MAX] {
        let _ = cpr_global_airborne(v, v, v, v);
        let _ = cpr_global_airborne(v, 0, 0x1ffff, v);
        let _ = is_false_ac_address(v);
    }
}
