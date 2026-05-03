//! E-AC-3 (Annex E) Bit Stream Information parser — `bsi()` per
//! §E.2.2.2 / Table E1.2.
//!
//! Unlike base AC-3, the E-AC-3 syncinfo is just a 16-bit syncword
//! (`0x0B77`) — no `crc1`, no `fscod`, no `frmsizecod`. The
//! sample-rate and frame-size codes have moved into the BSI itself,
//! reordered, and joined by a stream-type tag, substream id, and a
//! variable number-of-blocks code (1, 2, 3, or 6 audio blocks per
//! syncframe instead of AC-3's hard-coded 6).
//!
//! This module parses the **entire** Table E1.2 BSI bit-by-bit,
//! including the optional `mixmdate`, `infomdate`, and `addbsi`
//! chains. Fields that the round-1 decoder does not act on are still
//! consumed exactly so the bit cursor lands on byte/bit position
//! `start + bits_consumed = start of audfrm()`.
//!
//! ## Bit-stream order (Table E1.2 verbatim)
//!
//! ```text
//!   strmtyp       2
//!   substreamid   3
//!   frmsiz       11        // size in 16-bit words minus one
//!   fscod         2
//!   if (fscod == 0x3) {
//!     fscod2      2         // numblkscod implicit = 0x3 (6 blocks)
//!   } else {
//!     numblkscod  2
//!   }
//!   acmod         3
//!   lfeon         1
//!   bsid          5         // ≥ 11 for E-AC-3 (16 = canonical)
//!   dialnorm      5
//!   compre        1
//!   if (compre)        compr        8
//!   if (acmod == 0) {
//!     dialnorm2   5
//!     compr2e     1
//!     if (compr2e)     compr2       8
//!   }
//!   if (strmtyp == 0x1) {           // dependent substream
//!     chanmape    1
//!     if (chanmape)    chanmap     16
//!   }
//!   mixmdate      1
//!   if (mixmdate)         /* parses 0..200 bits per Table E1.2 */
//!   infomdate     1
//!   if (infomdate)        /* parses 0..50 bits  per Table E1.2 */
//!   addbsie       1
//!   if (addbsie)     addbsil 6, addbsi (addbsil+1)*8 bits
//! ```
//!
//! Field semantics are described per §E.2.3.1.x in the spec PDF.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::tables::acmod_nfchans;

/// Largest `bsid` value still served by the base AC-3 parser. Streams
/// at `bsid` 11..=16 use the E-AC-3 (Annex E) syntax.
pub const BSID_BASE_AC3_MAX: u8 = 10;

/// Canonical Annex E stream identification value (§E.2.3.1.6, "10000").
/// Backwards-compatible variants 11..15 share the same syntax.
pub const EAC3_BSID: u8 = 16;

/// Stream type — Table E2.1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamType {
    /// Type 0: independent substream (or sole independent stream).
    Independent,
    /// Type 1: dependent substream (refers back to the immediately
    /// preceding independent substream).
    Dependent,
    /// Type 2: an AC-3 bit-stream wrapped inside an E-AC-3 sync layer
    /// (§E.2.3.1.1 "may not have any dependent substreams associated").
    Ac3Convert,
    /// Type 3: reserved.
    Reserved,
}

impl StreamType {
    fn from_u8(v: u8) -> Self {
        match v & 0x3 {
            0 => StreamType::Independent,
            1 => StreamType::Dependent,
            2 => StreamType::Ac3Convert,
            _ => StreamType::Reserved,
        }
    }

    /// Raw 2-bit value the parser read.
    pub fn raw(self) -> u8 {
        match self {
            StreamType::Independent => 0,
            StreamType::Dependent => 1,
            StreamType::Ac3Convert => 2,
            StreamType::Reserved => 3,
        }
    }
}

/// Parsed E-AC-3 BSI — the subset actually needed by the round-1
/// decoder + dispatcher. Fields not surfaced are still parsed (the
/// bit cursor walk has to land at the start of `audfrm()`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bsi {
    pub strmtyp: StreamType,
    pub substreamid: u8,
    /// `frmsiz` raw value. Frame size in bytes = `(frmsiz + 1) * 2`.
    pub frmsiz: u16,
    /// `fscod` raw value (2 bits). 0x3 indicates a reduced-rate stream
    /// (use `fscod2` for the actual rate).
    pub fscod: u8,
    /// `fscod2` (2 bits). Only valid when `fscod == 0x3`; 0xFF
    /// otherwise.
    pub fscod2: u8,
    /// Sample rate in Hz, derived from `(fscod, fscod2)`.
    pub sample_rate: u32,
    /// `numblkscod` raw value (2 bits). 0x3 = 6 blocks. 0/1/2 = 1/2/3
    /// blocks. When `fscod == 0x3` it is implicitly 0x3.
    pub numblkscod: u8,
    /// Number of audio blocks per syncframe (= 256 PCM samples each).
    /// Derived from `numblkscod`. Always 6 on reduced-rate streams.
    pub num_blocks: u8,
    /// AC-3 audio coding mode (Table 5.8, §5.4.2.3) — channel layout.
    pub acmod: u8,
    /// Number of full-bandwidth channels (`acmod_nfchans(acmod)`).
    pub nfchans: u8,
    /// Whether the LFE channel is coded in this substream.
    pub lfeon: bool,
    /// Total channel count = `nfchans + lfeon`.
    pub nchans: u8,
    /// `bsid` (5 bits). 16 = canonical Annex E; 11..15 = backward-
    /// compatible Annex E variants.
    pub bsid: u8,
    /// Dialogue normalization, 1..=31 dB below reference. 0 in the
    /// stream is reserved → mapped to 31 per §5.4.2.8 (Annex E reuses
    /// the base spec semantics).
    pub dialnorm: u8,
    /// `chanmap` field (16 bits, dependent substream only). `None`
    /// when `strmtyp != Dependent` or `chanmape == 0`.
    ///
    /// Per Table E2.5, bit *i* (counted from the field's MSB → bit 0
    /// = MSB) flags channel location *i* — see the table for the
    /// label assignment.
    pub chanmap: Option<u16>,
    /// Frame size in bytes — `(frmsiz + 1) * 2`. Cached so the
    /// dispatcher can range-check the packet without re-doing
    /// arithmetic.
    pub frame_bytes: u32,
    /// Total number of bits the parser consumed out of the input
    /// slice. Callers seek the audfrm parser to exactly this offset.
    pub bits_consumed: u64,
}

/// Parse the E-AC-3 BSI starting at byte 0 of `data` (the byte *just
/// after* the 16-bit syncword — i.e. the third byte of the syncframe).
///
/// Returns `Err(Error::Invalid)` for reserved / illegal field
/// combinations (`fscod2 == 0x3`, `bsid == 9 || bsid == 10` per
/// §E.2.3.1.6, or a malformed `addbsi` length).
pub fn parse(data: &[u8]) -> Result<Bsi> {
    let mut br = BitReader::new(data);
    parse_with(&mut br)
}

/// Variant that reads from an externally-managed [`BitReader`] so a
/// caller already positioned past the syncword can share its cursor.
pub fn parse_with(br: &mut BitReader<'_>) -> Result<Bsi> {
    let start_bits = br.bit_position();

    // §E.2.3.1.1
    let strmtyp_raw = br.read_u32(2)? as u8;
    let strmtyp = StreamType::from_u8(strmtyp_raw);
    if matches!(strmtyp, StreamType::Reserved) {
        return Err(Error::invalid("eac3 bsi: strmtyp '11' is reserved"));
    }

    // §E.2.3.1.2
    let substreamid = br.read_u32(3)? as u8;

    // §E.2.3.1.3 — 11-bit value, frame_size_in_words = frmsiz + 1.
    let frmsiz = br.read_u32(11)? as u16;
    let frame_words = (frmsiz as u32) + 1;
    let frame_bytes = frame_words * 2;
    if !(64..=4096).contains(&frame_bytes) {
        // Spec note in §E.2.3.1.3: "values at the lower end of this
        // range do not occur as they do not represent enough words to
        // convey a complete syncframe". We still accept anything that
        // can plausibly fit a syncinfo + bsi + crc2; downstream sanity
        // checks reject runts.
        if frame_bytes < 8 {
            return Err(Error::invalid(format!(
                "eac3 bsi: frmsiz {frmsiz} → frame {frame_bytes} bytes is too small"
            )));
        }
    }

    // §E.2.3.1.4
    let fscod = br.read_u32(2)? as u8;
    // §E.2.3.1.5 — fscod2 OR numblkscod
    let (fscod2, numblkscod) = if fscod == 0x3 {
        let f2 = br.read_u32(2)? as u8;
        if f2 == 0x3 {
            return Err(Error::invalid(
                "eac3 bsi: fscod2 '11' is reserved (Table E2.3)",
            ));
        }
        // numblkscod is implicitly 0x3 (six blocks per syncframe) when
        // fscod indicates a reduced-rate stream.
        (f2, 0x3u8)
    } else {
        (0xFFu8, br.read_u32(2)? as u8)
    };
    let num_blocks = match numblkscod {
        0 => 1u8,
        1 => 2,
        2 => 3,
        _ => 6,
    };
    let sample_rate = match (fscod, fscod2) {
        (0, _) => 48_000,
        (1, _) => 44_100,
        (2, _) => 32_000,
        (3, 0) => 24_000,
        (3, 1) => 22_050,
        (3, 2) => 16_000,
        _ => unreachable!("fscod/fscod2 combos covered above"),
    };

    // §E.2.3.1.x acmod / lfeon
    let acmod = br.read_u32(3)? as u8;
    let lfeon = br.read_u32(1)? != 0;
    let nfchans = acmod_nfchans(acmod);
    let nchans = nfchans + u8::from(lfeon);

    // §E.2.3.1.6
    let bsid = br.read_u32(5)? as u8;
    if bsid == 9 || bsid == 10 || bsid > 16 {
        return Err(Error::Unsupported(format!(
            "eac3 bsi: bsid {bsid} is reserved/illegal per §E.2.3.1.6"
        )));
    }
    if bsid <= BSID_BASE_AC3_MAX {
        // Caller should have routed this packet to the base AC-3
        // parser (which itself handles bsid ≤ 8 + the tolerated 9..=10
        // safety margin we permit elsewhere). Surfacing a clear error
        // protects against double-dispatch bugs in the decoder loop.
        return Err(Error::Unsupported(format!(
            "eac3 bsi: bsid {bsid} routes through the base AC-3 parser, not Annex E"
        )));
    }

    // §5.4.2.8 (reused) — dialnorm 0 maps to 31.
    let dialnorm_raw = br.read_u32(5)? as u8;
    let dialnorm = if dialnorm_raw == 0 { 31 } else { dialnorm_raw };

    let compre = br.read_u32(1)? != 0;
    if compre {
        let _compr = br.read_u32(8)?;
    }

    // 1+1 dual-mono (acmod == 0): second copy of dialnorm + compr.
    if acmod == 0 {
        let _dialnorm2_raw = br.read_u32(5)?;
        let compr2e = br.read_u32(1)? != 0;
        if compr2e {
            let _compr2 = br.read_u32(8)?;
        }
    }

    // §E.2.3.1.7-8 — chanmape / chanmap, dependent substream only.
    let chanmap = if matches!(strmtyp, StreamType::Dependent) {
        let chanmape = br.read_u32(1)? != 0;
        if chanmape {
            Some(br.read_u32(16)? as u16)
        } else {
            None
        }
    } else {
        None
    };

    // §E.2.3.1.9-21 — mixing meta-data block.
    let mixmdate = br.read_u32(1)? != 0;
    if mixmdate {
        skip_mixing_metadata(br, acmod, lfeon, strmtyp, numblkscod)?;
    }

    // §E.2.3.1.62 ff — informational meta-data.
    let infomdate = br.read_u32(1)? != 0;
    if infomdate {
        skip_informational_metadata(br, acmod, fscod, strmtyp, numblkscod)?;
    }

    // addbsi — opt-in trailer of up to 64 bytes.
    let addbsie = br.read_u32(1)? != 0;
    if addbsie {
        let addbsil = br.read_u32(6)?;
        let nbits = (addbsil + 1) * 8;
        br.skip(nbits)?;
    }

    let bits_consumed = br.bit_position() - start_bits;

    Ok(Bsi {
        strmtyp,
        substreamid,
        frmsiz,
        fscod,
        fscod2,
        sample_rate,
        numblkscod,
        num_blocks,
        acmod,
        nfchans,
        lfeon,
        nchans,
        bsid,
        dialnorm,
        chanmap,
        frame_bytes,
        bits_consumed,
    })
}

/// Walk the §E.2.3.1.9-61 mixing metadata block exactly per Table
/// E1.2. Every field is consumed; none are surfaced (the round-1
/// decoder ignores all of it). Errors propagate when the bit-reader
/// runs out of input.
fn skip_mixing_metadata(
    br: &mut BitReader<'_>,
    acmod: u8,
    lfeon: bool,
    strmtyp: StreamType,
    numblkscod: u8,
) -> Result<()> {
    // §E.2.3.1 mixing metadata — Table E1.2.
    // dmixmod (2) when acmod > 0x2 (more than 2 channels).
    if acmod > 0x2 {
        let _dmixmod = br.read_u32(2)?;
    }
    // ltrtcmixlev (3) + lorocmixlev (3) when 3 front channels exist.
    if (acmod & 0x1) != 0 && acmod > 0x2 {
        let _ltrtcmixlev = br.read_u32(3)?;
        let _lorocmixlev = br.read_u32(3)?;
    }
    // ltrtsurmixlev (3) + lorosurmixlev (3) when a surround channel exists.
    if (acmod & 0x4) != 0 {
        let _ltrtsurmixlev = br.read_u32(3)?;
        let _lorosurmixlev = br.read_u32(3)?;
    }
    // lfemixlevcode (1) + lfemixlevcod (5) when LFE on.
    if lfeon {
        let lfemixlevcode = br.read_u32(1)? != 0;
        if lfemixlevcode {
            let _lfemixlevcod = br.read_u32(5)?;
        }
    }
    // strmtyp == 0x0 (independent) emits pgmscle/pgmscl + extpgmscle/
    // extpgmscl + mixdef + (mixdef-dependent body).
    if matches!(strmtyp, StreamType::Independent) {
        let pgmscle = br.read_u32(1)? != 0;
        if pgmscle {
            let _pgmscl = br.read_u32(6)?;
        }
        if acmod == 0 {
            let pgmscl2e = br.read_u32(1)? != 0;
            if pgmscl2e {
                let _pgmscl2 = br.read_u32(6)?;
            }
        }
        let extpgmscle = br.read_u32(1)? != 0;
        if extpgmscle {
            let _extpgmscl = br.read_u32(6)?;
        }
        let mixdef = br.read_u32(2)?;
        match mixdef {
            0 => { /* no additional bits */ }
            1 => {
                // premixcmpsel(1) + drcsrc(1) + premixcmpscl(3) = 5
                let _ = br.read_u32(5)?;
            }
            2 => {
                // mixdata = 12 bits (Table E1.2 — "mixing option 3, 12 bits reserved").
                let _ = br.read_u32(12)?;
            }
            _ => {
                // mixdef == 3: variable-length mixing parameter block.
                // mixdeflen(5), mixdata2e(1), if mixdata2e {…}, mixdata3e(1),
                // if mixdata3e {…}, mixdata field (8*(mixdeflen+2) - num_mixdata_bits),
                // mixdatafill (0..7 bits to round to a byte).
                let mixdeflen = br.read_u32(5)?;
                let mut bits_used: u32 = 5; // mixdeflen itself
                let mixdata2e = br.read_u32(1)? != 0;
                bits_used += 1;
                if mixdata2e {
                    bits_used += parse_mixdata2_block(br, acmod, lfeon)?;
                }
                let mixdata3e = br.read_u32(1)? != 0;
                bits_used += 1;
                if mixdata3e {
                    bits_used += parse_mixdata3_block(br)?;
                }
                let mixdata_bits_total = 8 * (mixdeflen + 2);
                if bits_used >= mixdata_bits_total {
                    // Spec note: bits_used must be ≤ mixdata_bits_total.
                    // If we've already consumed more than the budget,
                    // the bit stream is malformed — bail.
                    return Err(Error::invalid(format!(
                        "eac3 bsi: mixdata overrun (used {bits_used} bits, budget {mixdata_bits_total})"
                    )));
                }
                let pad = mixdata_bits_total - bits_used;
                br.skip(pad)?;
                // mixdatafill rounds the field to a whole byte. After
                // the fixed 8*(mixdeflen+2) bits the field is byte-aligned
                // by construction; nothing more to do.
            }
        }
        // paninfoe / paninfo + paninfo2e / paninfo2 — only when acmod < 2
        // (mono or 1+1 dual-mono).
        if acmod < 0x2 {
            let paninfoe = br.read_u32(1)? != 0;
            if paninfoe {
                let _panmean = br.read_u32(8)?;
                let _paninfo = br.read_u32(6)?;
            }
            if acmod == 0 {
                let paninfo2e = br.read_u32(1)? != 0;
                if paninfo2e {
                    let _panmean2 = br.read_u32(8)?;
                    let _paninfo2 = br.read_u32(6)?;
                }
            }
        }
        // frmmixcfginfoe — and per-block blkmixcfginfo.
        let frmmixcfginfoe = br.read_u32(1)? != 0;
        if frmmixcfginfoe {
            if numblkscod == 0 {
                let _blkmixcfginfo0 = br.read_u32(5)?;
            } else {
                let nblks = match numblkscod {
                    1 => 2u32,
                    2 => 3,
                    _ => 6,
                };
                for _ in 0..nblks {
                    let blkmixcfginfoe = br.read_u32(1)? != 0;
                    if blkmixcfginfoe {
                        let _blkmixcfginfo = br.read_u32(5)?;
                    }
                }
            }
        }
    }
    Ok(())
}

/// Parses the body of `mixdata2e` (mixing option 4 with extra channel
/// scale factors). Returns the number of bits consumed.
fn parse_mixdata2_block(br: &mut BitReader<'_>, _acmod: u8, _lfeon: bool) -> Result<u32> {
    let mut bits = 0u32;
    // premixcmpsel(1) + drcsrc(1) + premixcmpscl(3) = 5 bits.
    let _ = br.read_u32(5)?;
    bits += 5;
    // For each of L/C/R/Ls/Rs/LFE: presence(1) + (if set) scale(4) = 5 bits.
    // Plus dmixscle(1) + (if set) dmixscl(4).
    // Plus addche(1) + (if set) extpgmaux1scle(1)+(...4) + extpgmaux2scle(1)+(...4).
    for _ in 0..6 {
        let p = br.read_u32(1)? != 0;
        bits += 1;
        if p {
            let _ = br.read_u32(4)?;
            bits += 4;
        }
    }
    let dmixscle = br.read_u32(1)? != 0;
    bits += 1;
    if dmixscle {
        let _ = br.read_u32(4)?;
        bits += 4;
    }
    let addche = br.read_u32(1)? != 0;
    bits += 1;
    if addche {
        let p1 = br.read_u32(1)? != 0;
        bits += 1;
        if p1 {
            let _ = br.read_u32(4)?;
            bits += 4;
        }
        let p2 = br.read_u32(1)? != 0;
        bits += 1;
        if p2 {
            let _ = br.read_u32(4)?;
            bits += 4;
        }
    }
    Ok(bits)
}

/// Parses the body of `mixdata3e` (speech enhancement processing).
/// Returns the number of bits consumed.
fn parse_mixdata3_block(br: &mut BitReader<'_>) -> Result<u32> {
    let mut bits = 0u32;
    // spchdat(5) + addspchdate(1) + (if set) spchdat1(5) + spchan1att(2) +
    //   addspchdat1e(1) + (if set) spchdat2(5) + spchan2att(3).
    let _ = br.read_u32(5)?;
    bits += 5;
    let addspchdate = br.read_u32(1)? != 0;
    bits += 1;
    if addspchdate {
        let _ = br.read_u32(5)?;
        let _ = br.read_u32(2)?;
        bits += 7;
        let addspchdat1e = br.read_u32(1)? != 0;
        bits += 1;
        if addspchdat1e {
            let _ = br.read_u32(5)?;
            let _ = br.read_u32(3)?;
            bits += 8;
        }
    }
    Ok(bits)
}

/// Walk the §E.2.3.1.62 ff informational metadata block. The body is
/// the same structural shape as base AC-3's `bsmod`/`copyrightb`/
/// `origbs`/`audprodie` chain plus a few Annex E additions
/// (`sourcefscod`, `convsync`, `blkid`/`frmsizecod` for AC-3-converted
/// streams).
fn skip_informational_metadata(
    br: &mut BitReader<'_>,
    acmod: u8,
    fscod: u8,
    strmtyp: StreamType,
    numblkscod: u8,
) -> Result<()> {
    let _bsmod = br.read_u32(3)?;
    let _copyrightb = br.read_u32(1)?;
    let _origbs = br.read_u32(1)?;
    if acmod == 0x2 {
        let _dsurmod = br.read_u32(2)?;
        let _dheadphonmod = br.read_u32(2)?;
    }
    if acmod >= 0x6 {
        let _dsurexmod = br.read_u32(2)?;
    }
    let audprodie = br.read_u32(1)? != 0;
    if audprodie {
        let _mixlevel = br.read_u32(5)?;
        let _roomtyp = br.read_u32(2)?;
        let _adconvtyp = br.read_u32(1)?;
    }
    if acmod == 0 {
        let audprodi2e = br.read_u32(1)? != 0;
        if audprodi2e {
            let _mixlevel2 = br.read_u32(5)?;
            let _roomtyp2 = br.read_u32(2)?;
            let _adconvtyp2 = br.read_u32(1)?;
        }
    }
    if fscod < 0x3 {
        let _sourcefscod = br.read_u32(1)?;
    }
    // convsync is present only for indep substream (strmtyp == 0) when
    // numblkscod != 0x3 (i.e. fewer than 6 blocks per syncframe).
    if matches!(strmtyp, StreamType::Independent) && numblkscod != 0x3 {
        let _convsync = br.read_u32(1)?;
    }
    // strmtyp == 0x2 → AC-3 wrapped in E-AC-3 syncframe; carries
    // `blkid` (only if numblkscod != 0x3) + `frmsizecod` (6 bits).
    if matches!(strmtyp, StreamType::Ac3Convert) {
        if numblkscod != 0x3 {
            let _blkid = br.read_u32(1)?;
        }
        let _frmsizecod = br.read_u32(6)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper — pack a sequence of (n_bits, value) pairs MSB-first
    /// into a fresh byte buffer, padded with zeros to the next byte
    /// boundary.
    fn pack_msb(bits: &[(u32, u32)]) -> (Vec<u8>, u64) {
        let total: u32 = bits.iter().map(|(n, _)| *n).sum();
        let nbytes = total.div_ceil(8);
        let mut out = vec![0u8; nbytes as usize];
        let mut bitpos = 0u32;
        for &(n, v) in bits {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = (bitpos / 8) as usize;
                let shift = 7 - (bitpos % 8);
                out[byte] |= bit << shift;
                bitpos += 1;
            }
        }
        (out, total as u64)
    }

    /// Independent substream, 2/0 stereo, 48 kHz, 6 blocks, 768 byte
    /// frame. dialnorm=27, no compr, no chanmape (indep), no mixmdate,
    /// no infomdate, no addbsi. Mirrors the FFmpeg-encoded fixture
    /// `eac3-stereo-48000-192kbps`.
    #[test]
    fn parses_192kbps_indep_stereo() {
        let bits: &[(u32, u32)] = &[
            (2, 0),    // strmtyp = 0
            (3, 0),    // substreamid = 0
            (11, 383), // frmsiz = 383 → 768 bytes
            (2, 0),    // fscod = 0 → 48 kHz
            (2, 3),    // numblkscod = 3 → 6 blocks
            (3, 2),    // acmod = 2 (2/0)
            (1, 0),    // lfeon
            (5, 16),   // bsid = 16
            (5, 27),   // dialnorm = 27 → -27 dB
            (1, 0),    // compre
            (1, 0),    // mixmdate
            (1, 0),    // infomdate
            (1, 0),    // addbsie
        ];
        let (buf, total_bits) = pack_msb(bits);
        let bsi = parse(&buf).unwrap();
        assert_eq!(bsi.strmtyp, StreamType::Independent);
        assert_eq!(bsi.substreamid, 0);
        assert_eq!(bsi.frmsiz, 383);
        assert_eq!(bsi.frame_bytes, 768);
        assert_eq!(bsi.sample_rate, 48_000);
        assert_eq!(bsi.num_blocks, 6);
        assert_eq!(bsi.acmod, 2);
        assert_eq!(bsi.nfchans, 2);
        assert_eq!(bsi.nchans, 2);
        assert!(!bsi.lfeon);
        assert_eq!(bsi.bsid, 16);
        assert_eq!(bsi.dialnorm, 27);
        assert!(bsi.chanmap.is_none());
        assert_eq!(bsi.bits_consumed, total_bits);
    }

    /// Reduced-rate (24 kHz) variant — fscod=3, fscod2=0, numblkscod
    /// implicit at 6 blocks.
    #[test]
    fn parses_reduced_rate_24khz() {
        let bits: &[(u32, u32)] = &[
            (2, 0),    // strmtyp
            (3, 0),    // substreamid
            (11, 100), // frmsiz
            (2, 3),    // fscod = 0x3 → reduced
            (2, 0),    // fscod2 = 0 → 24 kHz
            (3, 2),    // acmod = 2
            (1, 0),    // lfeon
            (5, 16),   // bsid
            (5, 31),   // dialnorm
            (1, 0),    // compre
            (1, 0),    // mixmdate
            (1, 0),    // infomdate
            (1, 0),    // addbsie
        ];
        let (buf, _) = pack_msb(bits);
        let bsi = parse(&buf).unwrap();
        assert_eq!(bsi.fscod, 3);
        assert_eq!(bsi.fscod2, 0);
        assert_eq!(bsi.sample_rate, 24_000);
        assert_eq!(bsi.num_blocks, 6); // implicit numblkscod=3
        assert_eq!(bsi.numblkscod, 3);
    }

    /// 1-block-per-syncframe variant (`eac3-256-coeff-block` fixture
    /// shape).
    #[test]
    fn parses_one_block_per_syncframe() {
        let bits: &[(u32, u32)] = &[
            (2, 0),   // strmtyp
            (3, 0),   // substreamid
            (11, 50), // frmsiz
            (2, 0),   // fscod = 48 kHz
            (2, 0),   // numblkscod = 0 → 1 block
            (3, 2),   // acmod
            (1, 0),   // lfeon
            (5, 16),  // bsid
            (5, 31),  // dialnorm
            (1, 0),   // compre
            (1, 0),   // mixmdate
            (1, 0),   // infomdate
            (1, 0),   // addbsie
        ];
        let (buf, _) = pack_msb(bits);
        let bsi = parse(&buf).unwrap();
        assert_eq!(bsi.numblkscod, 0);
        assert_eq!(bsi.num_blocks, 1);
    }

    #[test]
    fn rejects_reserved_bsid_9_10() {
        for bad in [9u32, 10] {
            let bits: &[(u32, u32)] = &[
                (2, 0),
                (3, 0),
                (11, 100),
                (2, 0),
                (2, 3),
                (3, 2),
                (1, 0),
                (5, bad),
                (5, 27),
                (1, 0),
                (1, 0),
                (1, 0),
                (1, 0),
            ];
            let (buf, _) = pack_msb(bits);
            let r = parse(&buf);
            assert!(r.is_err(), "expected reject for bsid={bad}, got {r:?}");
        }
    }

    /// Dependent substream with chanmape=1 and chanmap = bit 6
    /// (Lrs/Rrs pair) — matches our 7.1 encoder output's dep payload.
    #[test]
    fn parses_dependent_substream_chanmap() {
        let chanmap_val = 1u32 << (15 - 6); // bit 6 (Table E2.5)
        let bits: &[(u32, u32)] = &[
            (2, 1),    // strmtyp = dependent
            (3, 0),    // substreamid = 0 (first dep)
            (11, 383), // frmsiz
            (2, 0),    // fscod
            (2, 3),    // numblkscod = 3
            (3, 2),    // acmod = 2 (2 channels: Lb, Rb)
            (1, 0),    // lfeon
            (5, 16),   // bsid
            (5, 27),   // dialnorm
            (1, 0),    // compre
            (1, 1),    // chanmape = 1
            (16, chanmap_val),
            (1, 0), // mixmdate
            (1, 0), // infomdate
            (1, 0), // addbsie
        ];
        let (buf, _) = pack_msb(bits);
        let bsi = parse(&buf).unwrap();
        assert_eq!(bsi.strmtyp, StreamType::Dependent);
        assert_eq!(bsi.chanmap, Some(0x0200));
    }

    #[test]
    fn rejects_strmtyp_reserved() {
        let bits: &[(u32, u32)] = &[
            (2, 3), // strmtyp = '11' reserved
            (3, 0),
            (11, 100),
            (2, 0),
            (2, 3),
            (3, 2),
            (1, 0),
            (5, 16),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let (buf, _) = pack_msb(bits);
        assert!(parse(&buf).is_err());
    }

    #[test]
    fn rejects_fscod2_reserved() {
        let bits: &[(u32, u32)] = &[
            (2, 0),
            (3, 0),
            (11, 100),
            (2, 3), // fscod = 0x3 → reduced rate
            (2, 3), // fscod2 = 0x3 reserved
            (3, 2),
            (1, 0),
            (5, 16),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let (buf, _) = pack_msb(bits);
        assert!(parse(&buf).is_err());
    }
}
