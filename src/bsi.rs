//! AC-3 Bit Stream Information — `bsi()` (§5.3.2 / §5.4.2).
//!
//! The BSI immediately follows the 5-byte syncinfo and describes the
//! service characteristics: stream identification, channel layout,
//! dialogue normalization, compression, language, timecode, etc.
//!
//! This module parses the base (bsid ≤ 8) layout. Annex E (E-AC-3,
//! bsid=16) is a separate syntax not handled here — that's a future
//! `oxideav-eac3` crate.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::tables::acmod_nfchans;

/// Largest `bsid` value accepted by the base AC-3 BSI parser. Streams
/// at higher `bsid` values use the Annex E (E-AC-3) syntax — the
/// top-level decoder dispatches them to [`crate::eac3::decoder`].
///
/// The spec mandates muting for `bsid > 8` in pure AC-3 decoders
/// (§5.4.2.7) but accepts up to 10 as a small safety margin for
/// near-compatible streams (legacy bsid=9..=10 variants of base AC-3
/// that still parse the same syntax). bsid 11..=16 is canonical
/// E-AC-3 territory.
pub const MAX_BSID_BASE: u8 = 10;

/// Parsed BSI — just the fields a decoder actually needs. Optional
/// service-metadata (compression gain, language code, timecodes,
/// `addbsi`) is consumed but not surfaced since the decoder doesn't
/// apply any of it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bsi {
    /// bsid — bit stream identification. Spec mandates decoders built
    /// to A/52 mute for `bsid > 8` (Annex E E-AC-3 is a different
    /// syntax). We surface the raw value and let the decoder decide.
    pub bsid: u8,
    pub bsmod: u8,
    pub acmod: u8,
    /// Number of full-bandwidth channels per Table 5.8.
    pub nfchans: u8,
    /// `true` when the low-frequency-effects channel is on.
    pub lfeon: bool,
    /// Total channel count — `nfchans + lfeon`.
    pub nchans: u8,
    /// Dialogue normalization, 1..=31 dB below reference. 0 is reserved
    /// and spec says to treat it as 31.
    pub dialnorm: u8,
    /// Center mix-level coefficient code (cmixlev) for acmod with 3
    /// front channels; 0xFF when absent.
    pub cmixlev: u8,
    /// Surround mix-level coefficient code (surmixlev); 0xFF when absent.
    pub surmixlev: u8,
    /// Dolby-Surround flag for 2/0 stereo streams; 0xFF when absent.
    pub dsurmod: u8,
    /// Annex D §2.3 "alternate bit stream syntax" mix-level extensions.
    /// `Some` only when `bsid == 6` AND the encoder set `xbsi1e == 1`;
    /// `None` otherwise. The four 3-bit codewords (`ltrtcmixlev` /
    /// `ltrtsurmixlev` / `lorocmixlev` / `lorosurmixlev`) refine the
    /// 2-bit `cmixlev` / `surmixlev` defaults specifically for the
    /// LtRt vs LoRo downmix targets — see [`crate::downmix`].
    pub annex_d_mix_levels: Option<AnnexDMixLevels>,
    /// Annex D §2.3.1.2 preferred-stereo-downmix-mode (`dmixmod`); 0xFF
    /// when absent. `00` = not indicated, `01` = LtRt preferred,
    /// `10` = LoRo preferred, `11` = reserved.
    pub dmixmod: u8,
    /// Absolute bit position (in bits, measured from the first byte of
    /// `bsi()` input) where the BSI ended. Callers use this to skip
    /// straight to the audio-block area.
    pub bits_consumed: u64,
}

/// Annex D §2.3.1.3-6 alternate-syntax mix-level codewords. Each is a
/// 3-bit value; Tables D2.3 / D2.4 / D2.5 / D2.6 map them to linear
/// gains via [`annex_d_lt_rt_clev`] / [`annex_d_lt_rt_slev`] /
/// [`annex_d_lo_ro_clev`] / [`annex_d_lo_ro_slev`].
///
/// These supersede the body-spec 2-bit `cmixlev` / `surmixlev` defaults
/// for the LtRt / LoRo downmix targets specifically — the body fields
/// are still parsed (they sit ahead of the xbsi1 block in the bit
/// stream) but a §7.8 downmix on a `bsid == 6` Annex D stream should
/// prefer the Annex D refinements.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AnnexDMixLevels {
    /// `ltrtcmixlev` (Table D2.3). Defined for acmod ∈ {3, 5, 7}.
    pub ltrtcmixlev: u8,
    /// `ltrtsurmixlev` (Table D2.4). Codes 000..010 are reserved → use
    /// 0.841. Defined for acmod ∈ {4, 5, 6, 7}.
    pub ltrtsurmixlev: u8,
    /// `lorocmixlev` (Table D2.5). Defined for acmod ∈ {3, 5, 7}.
    pub lorocmixlev: u8,
    /// `lorosurmixlev` (Table D2.6). Codes 000..010 are reserved → use
    /// 0.841. Defined for acmod ∈ {4, 5, 6, 7}.
    pub lorosurmixlev: u8,
}

/// Map an Annex D 3-bit "center-channel" mix-level code to a linear
/// gain per Tables D2.3 / D2.5 (the two tables are identical).
///
/// Code → gain (dB):
///  `000` 1.414 (+3.0), `001` 1.189 (+1.5), `010` 1.000 ( 0.0),
///  `011` 0.841 (−1.5), `100` 0.707 (−3.0), `101` 0.595 (−4.5),
///  `110` 0.500 (−6.0), `111` 0.000 (−∞).
pub fn annex_d_center_mix_gain(code: u8) -> f32 {
    match code & 0x7 {
        0 => 1.414,
        1 => 1.189,
        2 => 1.000,
        3 => 0.841,
        4 => 0.707,
        5 => 0.595,
        6 => 0.500,
        _ => 0.000,
    }
}

/// Map an Annex D 3-bit "surround-channel" mix-level code to a linear
/// gain per Tables D2.4 / D2.6 (identical). Codes `000..010` are
/// reserved; per §2.3.1.4 / §2.3.1.6 the decoder shall substitute
/// 0.841 (the next defined code).
pub fn annex_d_surround_mix_gain(code: u8) -> f32 {
    match code & 0x7 {
        0..=3 => 0.841, // 000/001/010 reserved → 0.841; 011 = 0.841
        4 => 0.707,
        5 => 0.595,
        6 => 0.500,
        _ => 0.000,
    }
}

/// Parse the BSI starting at the beginning of `data`. The slice *must*
/// point at the byte immediately following `syncinfo` (i.e. byte 5 of
/// the syncframe).
///
/// On success the returned `Bsi` describes the stream and carries the
/// exact number of bits the parser consumed, so the caller can resume
/// an MSB-first `BitReader` at the right place for the first audio
/// block.
pub fn parse(data: &[u8]) -> Result<Bsi> {
    let mut br = BitReader::new(data);

    let bsid = br.read_u32(5)? as u8;
    let bsmod = br.read_u32(3)? as u8;
    let acmod = br.read_u32(3)? as u8;
    let nfchans = acmod_nfchans(acmod);

    // cmixlev — only present when there are 3 front channels, i.e.
    // the two LSBs of acmod include '1' for centre *and* acmod!=1
    // (the spec's "if ((acmod & 0x1) && (acmod != 0x1))" guard).
    let cmixlev = if (acmod & 0x1) != 0 && acmod != 0x1 {
        br.read_u32(2)? as u8
    } else {
        0xFF
    };

    // surmixlev — present when a surround channel exists (acmod & 0x4).
    let surmixlev = if (acmod & 0x4) != 0 {
        br.read_u32(2)? as u8
    } else {
        0xFF
    };

    // dsurmod — present only in 2/0 mode (acmod == 0x2).
    let dsurmod = if acmod == 0x2 {
        br.read_u32(2)? as u8
    } else {
        0xFF
    };

    let lfeon = br.read_u32(1)? != 0;
    let nchans = nfchans + u8::from(lfeon);

    let dialnorm_raw = br.read_u32(5)? as u8;
    // §5.4.2.8: dialnorm=0 is reserved; decoder shall use -31 dB.
    let dialnorm = if dialnorm_raw == 0 { 31 } else { dialnorm_raw };

    // Optional service metadata (§5.4.2.9 ff). We parse-and-discard —
    // a proper player can tap these later via a second pass.
    let compre = br.read_u32(1)? != 0;
    if compre {
        let _compr = br.read_u32(8)?;
    }
    let langcode = br.read_u32(1)? != 0;
    if langcode {
        let _langcod = br.read_u32(8)?;
    }
    let audprodie = br.read_u32(1)? != 0;
    if audprodie {
        let _mixlevel = br.read_u32(5)?;
        let _roomtyp = br.read_u32(2)?;
    }

    // 1+1 mode (dual mono) carries a second copy of the metadata for Ch2.
    if acmod == 0 {
        let _dialnorm2 = br.read_u32(5)?;
        let compr2e = br.read_u32(1)? != 0;
        if compr2e {
            let _compr2 = br.read_u32(8)?;
        }
        let langcod2e = br.read_u32(1)? != 0;
        if langcod2e {
            let _langcod2 = br.read_u32(8)?;
        }
        let audprodi2e = br.read_u32(1)? != 0;
        if audprodi2e {
            let _mixlevel2 = br.read_u32(5)?;
            let _roomtyp2 = br.read_u32(2)?;
        }
    }

    let _copyrightb = br.read_u32(1)?;
    let _origbs = br.read_u32(1)?;

    // §5.3.2 base syntax has `timecod1e/timecod2e` here; Annex D
    // §2.3 / Table D2.1 reuses the same two 1+14-bit slots as
    // `xbsi1e/xbsi2e` and is identified by `bsid == 6` (§2.1).
    // Both shapes occupy the same fixed 30 bits maximum so the
    // surrounding parse is unchanged.
    let (annex_d_mix_levels, dmixmod) = if bsid == 6 {
        // Annex D xbsi1 block.
        let xbsi1e = br.read_u32(1)? != 0;
        let (mix, dmm) = if xbsi1e {
            let dmm = br.read_u32(2)? as u8;
            let ltrtc = br.read_u32(3)? as u8;
            let ltrts = br.read_u32(3)? as u8;
            let loroc = br.read_u32(3)? as u8;
            let loros = br.read_u32(3)? as u8;
            (
                Some(AnnexDMixLevels {
                    ltrtcmixlev: ltrtc,
                    ltrtsurmixlev: ltrts,
                    lorocmixlev: loroc,
                    lorosurmixlev: loros,
                }),
                dmm,
            )
        } else {
            (None, 0xFFu8)
        };
        // xbsi2: 2 + 2 + 1 + 8 + 1 = 14 bits, none consumed by the
        // round-126 decoder.
        let xbsi2e = br.read_u32(1)? != 0;
        if xbsi2e {
            let _dsurexmod = br.read_u32(2)?;
            let _dheadphonmod = br.read_u32(2)?;
            let _adconvtyp = br.read_u32(1)?;
            let _xbsi2 = br.read_u32(8)?;
            let _encinfo = br.read_u32(1)?;
        }
        (mix, dmm)
    } else {
        // §5.3.2 base syntax — timecod1/timecod2 (never surfaced).
        let timecod1e = br.read_u32(1)? != 0;
        if timecod1e {
            let _t1 = br.read_u32(14)?;
        }
        let timecod2e = br.read_u32(1)? != 0;
        if timecod2e {
            let _t2 = br.read_u32(14)?;
        }
        (None, 0xFFu8)
    };

    // addbsi — up to 64 bytes of trailing info we can safely skip.
    let addbsie = br.read_u32(1)? != 0;
    if addbsie {
        let addbsil = br.read_u32(6)?; // 0..=63, meaning 1..=64 bytes
        let nbits = (addbsil + 1) * 8;
        br.skip(nbits)?;
    }

    let bits_consumed = br.bit_position();

    if bsid > 10 {
        // Per spec, base decoders mute for bsid > 8; we accept ≤10 as
        // a small safety margin for near-compatible streams and defer
        // a hard rejection to the decoder loop so probing still
        // succeeds.
        return Err(Error::Unsupported(format!(
            "ac3: bsid {bsid} > 8 — Annex E E-AC-3 bitstream needs a separate parser"
        )));
    }

    Ok(Bsi {
        bsid,
        bsmod,
        acmod,
        nfchans,
        lfeon,
        nchans,
        dialnorm,
        cmixlev,
        surmixlev,
        dsurmod,
        annex_d_mix_levels,
        dmixmod,
        bits_consumed,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal BSI byte sequence for 2/0 stereo, LFE off, no
    /// optional fields. acmod=2 → surmixlev/cmixlev absent, dsurmod
    /// present.
    ///
    ///   bsid=8 (5 bits)  : 0b01000
    ///   bsmod=0 (3 bits) : 0b000
    ///   acmod=2 (3 bits) : 0b010
    ///   dsurmod=0 (2)    : 0b00
    ///   lfeon=0 (1)      : 0
    ///   dialnorm=27 (5)  : 0b11011
    ///   compre=0         : 0
    ///   langcode=0       : 0
    ///   audprodie=0      : 0
    ///   copyrightb=0     : 0
    ///   origbs=0         : 0
    ///   timecod1e=0      : 0
    ///   timecod2e=0      : 0
    ///   addbsie=0        : 0
    ///
    /// Total = 5+3+3+2+1+5+1+1+1+1+1+1+1+1 = 27 bits → 4 bytes with 5
    /// trailing pad bits.
    #[test]
    fn parses_minimal_2_0_stereo_bsi() {
        // Build via a BitWriter-style manual pack.
        let bits: [(u8, u32); 14] = [
            (5, 0b01000),
            (3, 0b000),
            (3, 0b010),
            (2, 0b00),
            (1, 0),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let mut out = vec![0u8; 8];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                out[byte] |= bit << shift;
                bitpos += 1;
            }
        }

        let b = parse(&out).unwrap();
        assert_eq!(b.bsid, 8);
        assert_eq!(b.bsmod, 0);
        assert_eq!(b.acmod, 2);
        assert_eq!(b.nfchans, 2);
        assert!(!b.lfeon);
        assert_eq!(b.nchans, 2);
        assert_eq!(b.dialnorm, 27);
        assert_eq!(b.dsurmod, 0);
        assert_eq!(b.cmixlev, 0xFF);
        assert_eq!(b.surmixlev, 0xFF);
        assert!(b.annex_d_mix_levels.is_none());
        assert_eq!(b.dmixmod, 0xFF);
        assert_eq!(b.bits_consumed, bitpos as u64);
    }

    #[test]
    fn dialnorm_zero_remaps_to_31() {
        // bsid=8, bsmod=0, acmod=1 (1/0 mono — no cmix / surmix / dsurmod),
        // lfeon=0, dialnorm=0 → should remap.
        let bits: [(u8, u32); 11] = [
            (5, 8),
            (3, 0),
            (3, 1),
            (1, 0), // lfeon
            (5, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let mut last4 = vec![0u8; 8];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                last4[byte] |= bit << shift;
                bitpos += 1;
            }
        }
        // Need addbsie + timecodes bits too — add three trailing zero bits
        // to cover (timecod1e, timecod2e, addbsie) — wait, already in list.
        // Actually this packs 5+3+3+1+5+... re-count:
        //   5+3+3+1+5+1+1+1+1+1+1 = 23 bits. Missing nothing structural?
        // For acmod=1 there's no cmix, surmix, dsurmod. After lfeon and
        // dialnorm it goes compre/langcode/audprodie/copyrightb/origbs/
        // timecod1e/timecod2e/addbsie = 8 flags but we have 6. Add two
        // more zero bits so the addbsie fires false.
        let bits2: [(u8, u32); 2] = [(1, 0), (1, 0)];
        for (n, v) in bits2.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                last4[byte] |= bit << shift;
                bitpos += 1;
            }
        }

        let b = parse(&last4).unwrap();
        assert_eq!(b.dialnorm, 31);
        assert_eq!(b.nfchans, 1);
        assert_eq!(b.nchans, 1);
    }

    /// Annex D §2 / Table D2.1 — `bsid == 6` activates the alternate
    /// syntax: the body's `timecod1e/timecod2e` slots become
    /// `xbsi1e/xbsi2e`. Verify the xbsi1 mix-level fields surface on
    /// [`Bsi::annex_d_mix_levels`] / [`Bsi::dmixmod`].
    #[test]
    fn parses_annex_d_bsid_6_xbsi1_mix_levels() {
        // 3/2 (acmod=7), lfe on. cmixlev = 0b00 (0.707), surmixlev = 0b00
        // (0.707). dialnorm=27. No compre / langcode / audprodie /
        // copyrightb / origbs. xbsi1e = 1 with:
        //   dmixmod        = 0b01 (LtRt preferred)
        //   ltrtcmixlev    = 0b011 (0.841)
        //   ltrtsurmixlev  = 0b100 (0.707)
        //   lorocmixlev    = 0b100 (0.707)
        //   lorosurmixlev  = 0b101 (0.595)
        // xbsi2e = 0. addbsie = 0.
        //
        // Bit layout:
        //   bsid=6 (5)         00110
        //   bsmod=0 (3)        000
        //   acmod=7 (3)        111
        //   cmixlev (2)        00
        //   surmixlev (2)      00
        //   lfeon (1)          1
        //   dialnorm (5)       11011
        //   compre (1)         0
        //   langcode (1)       0
        //   audprodie (1)      0
        //   copyrightb (1)     0
        //   origbs (1)         0
        //   xbsi1e (1)         1
        //   dmixmod (2)        01
        //   ltrtcmixlev (3)    011
        //   ltrtsurmixlev (3)  100
        //   lorocmixlev (3)    100
        //   lorosurmixlev (3)  101
        //   xbsi2e (1)         0
        //   addbsie (1)        0
        let bits: &[(u8, u32)] = &[
            (5, 6),
            (3, 0),
            (3, 7),
            (2, 0),
            (2, 0),
            (1, 1),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 1),
            (2, 0b01),
            (3, 0b011),
            (3, 0b100),
            (3, 0b100),
            (3, 0b101),
            (1, 0),
            (1, 0),
        ];
        let mut out = vec![0u8; 8];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                out[byte] |= bit << shift;
                bitpos += 1;
            }
        }

        let b = parse(&out).unwrap();
        assert_eq!(b.bsid, 6);
        assert_eq!(b.acmod, 7);
        assert!(b.lfeon);
        assert_eq!(b.dmixmod, 0b01);
        let mix = b.annex_d_mix_levels.expect("xbsi1 set → mix levels");
        assert_eq!(mix.ltrtcmixlev, 0b011);
        assert_eq!(mix.ltrtsurmixlev, 0b100);
        assert_eq!(mix.lorocmixlev, 0b100);
        assert_eq!(mix.lorosurmixlev, 0b101);
        assert_eq!(b.bits_consumed, bitpos as u64);
    }

    /// `bsid == 6` with `xbsi1e == 0` should leave the mix-level
    /// payload absent. The xbsi2e slot still needs to be consumed.
    #[test]
    fn parses_annex_d_bsid_6_no_xbsi1() {
        // 2/0 (acmod=2), no LFE. cmixlev absent (acmod & 1 == 0).
        // surmixlev absent (acmod & 4 == 0). dsurmod=0 (2 bits).
        // dialnorm=20. xbsi1e=0. xbsi2e=0. addbsie=0.
        let bits: &[(u8, u32)] = &[
            (5, 6),
            (3, 0),
            (3, 2),
            (2, 0),  // dsurmod
            (1, 0),  // lfeon
            (5, 20), // dialnorm
            (1, 0),  // compre
            (1, 0),  // langcode
            (1, 0),  // audprodie
            (1, 0),  // copyrightb
            (1, 0),  // origbs
            (1, 0),  // xbsi1e
            (1, 0),  // xbsi2e
            (1, 0),  // addbsie
        ];
        let mut out = vec![0u8; 8];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                out[byte] |= bit << shift;
                bitpos += 1;
            }
        }

        let b = parse(&out).unwrap();
        assert_eq!(b.bsid, 6);
        assert!(b.annex_d_mix_levels.is_none());
        assert_eq!(b.dmixmod, 0xFF);
        assert_eq!(b.bits_consumed, bitpos as u64);
    }

    /// Table D2.3 / D2.5 — the 3-bit center mix-level codewords map to
    /// the exact gains the spec tabulates.
    #[test]
    fn annex_d_center_mix_gain_matches_table_d2_3() {
        let expected: [(u8, f32); 8] = [
            (0b000, 1.414),
            (0b001, 1.189),
            (0b010, 1.000),
            (0b011, 0.841),
            (0b100, 0.707),
            (0b101, 0.595),
            (0b110, 0.500),
            (0b111, 0.000),
        ];
        for (code, gain) in expected {
            assert!(
                (annex_d_center_mix_gain(code) - gain).abs() < 1e-6,
                "code 0b{code:03b}: want {gain}, got {}",
                annex_d_center_mix_gain(code)
            );
        }
    }

    /// Table D2.4 / D2.6 — the 3-bit surround mix-level codewords. The
    /// reserved codes `000/001/010` substitute 0.841 per spec.
    #[test]
    fn annex_d_surround_mix_gain_substitutes_reserved_with_0_841() {
        // Reserved codes all map to 0.841.
        for code in 0u8..=2 {
            let g = annex_d_surround_mix_gain(code);
            assert!(
                (g - 0.841).abs() < 1e-6,
                "reserved code 0b{code:03b} should resolve to 0.841, got {g}"
            );
        }
        let expected: [(u8, f32); 5] = [
            (0b011, 0.841),
            (0b100, 0.707),
            (0b101, 0.595),
            (0b110, 0.500),
            (0b111, 0.000),
        ];
        for (code, gain) in expected {
            assert!(
                (annex_d_surround_mix_gain(code) - gain).abs() < 1e-6,
                "code 0b{code:03b}: want {gain}, got {}",
                annex_d_surround_mix_gain(code)
            );
        }
    }
}
