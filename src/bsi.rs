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
    /// Absolute bit position (in bits, measured from the first byte of
    /// `bsi()` input) where the BSI ended. Callers use this to skip
    /// straight to the audio-block area.
    pub bits_consumed: u64,
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

    let timecod1e = br.read_u32(1)? != 0;
    if timecod1e {
        let _t1 = br.read_u32(14)?;
    }
    let timecod2e = br.read_u32(1)? != 0;
    if timecod2e {
        let _t2 = br.read_u32(14)?;
    }

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
}
