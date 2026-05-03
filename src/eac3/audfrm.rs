//! E-AC-3 audio frame element — `audfrm()` per Table E1.3.
//!
//! `audfrm()` sits between `bsi()` and the first `audblk()`. It
//! carries frame-level strategy flags (`expstre`, `ahte`,
//! `snroffststr`, `transproce`, `blkswe`, `dithflage`, `bamode`,
//! `frmfgaincode`, `dbaflde`, `skipflde`, `spxattene`) plus, when the
//! corresponding flag is *cleared*, the per-channel frame-level
//! strategy values that would otherwise have been emitted per block.
//!
//! Round-1 scope (consumed-but-not-acted-upon for everything except
//! the strategy flags themselves):
//!
//! * Strategy flags: stored in [`AudFrm`].
//! * Frame-level exponent strategies: parsed when `expstre == 0`. The
//!   spec packs these as a per-channel run of fixed-width codes (2
//!   bits for fbw, 5 bits for converter exponents on `strmtyp == 0`,
//!   1 bit for LFE).
//! * AHT in-use flags: parsed when `ahte == 1`.
//! * Frame-level SNR offsets: parsed when `snroffststr == 0`.
//! * Transient pre-noise processing: parsed when `transproce == 1`.
//! * Spectral-extension attenuation parameters: parsed when
//!   `spxattene == 1`.
//! * Block-start info: parsed when `numblkscod != 0` and the
//!   `blkstrtinfoe` bit is set.
//!
//! All consumed values are surfaced on [`AudFrm`] so the audblk
//! parser can use them as defaults when the per-block flag says
//! "frame-level value reused".

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use super::bsi::{Bsi, StreamType};

/// Maximum coded channels in a single substream (5 fbw + LFE = 6, but
/// the parser indexes fbw and converter strategies independently).
const MAX_FBW: usize = 5;

/// Maximum blocks per syncframe (Annex E).
pub const MAX_BLOCKS_PER_FRAME: usize = 6;

/// Parsed `audfrm()` snapshot.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AudFrm {
    /// `expstre` (1 bit, only present when `numblkscod == 0x3`).
    /// `true` means each block carries its own exponent strategy
    /// (mirroring AC-3 base behaviour); `false` means the frame
    /// emitted per-channel `frmchexpstr` codes that drive a 6-block
    /// strategy run via Table E2.10.
    pub expstre: bool,
    /// `ahte` (1 bit) — Adaptive Hybrid Transform in use this frame.
    pub ahte: bool,
    /// `snroffststr` (2 bits) — frame-level SNR offset packing mode.
    /// 0 = single frame value; 1/2 = per-block sub-modes (see Table
    /// E1.3 for the exact bit layout).
    pub snroffststr: u8,
    /// `transproce` (1 bit) — transient pre-noise processing in use.
    pub transproce: bool,
    /// `blkswe` (1 bit) — per-block block-switch flags emitted (when
    /// false, `blksw[ch] = 0` for every block of every channel).
    pub blkswe: bool,
    /// `dithflage` (1 bit) — per-block per-channel dither flags
    /// emitted (when false, `dithflag[ch] = 1` always).
    pub dithflage: bool,
    /// `bamode` (1 bit) — per-block bit-allocation parametric info
    /// emitted (when false, the BA params take fixed defaults from
    /// §E.2.2.4 / Table E1.4).
    pub bamode: bool,
    /// `frmfgaincode` (1 bit) — `fgaincode` is per-block (when true)
    /// versus implicit (`fgaincode = 0`) when false.
    pub frmfgaincode: bool,
    /// `dbaflde` (1 bit) — delta bit-allocation info may appear in any
    /// block when true; absent when false.
    pub dbaflde: bool,
    /// `skipflde` (1 bit) — skip-field-exists flag emitted per block
    /// when true.
    pub skipflde: bool,
    /// `spxattene` (1 bit) — spectral-extension attenuation parameters
    /// follow.
    pub spxattene: bool,
    /// Frame-level coupling exponent strategy (`frmcplexpstr`, 5
    /// bits, from Table E2.10). 0xFF when not applicable. Applies
    /// only when `expstre == 0` AND coupling is in use across the
    /// whole syncframe.
    pub frmcplexpstr: u8,
    /// Frame-level per-fbw-channel exponent strategy (`frmchexpstr`,
    /// 5 bits each). 0xFF when not applicable.
    pub frmchexpstr: [u8; MAX_FBW],
    /// `lfeexpstr[blk]` — LFE exponent strategy per block (1 bit each).
    /// Only valid when `lfeon == true`.
    pub lfeexpstr: [u8; MAX_BLOCKS_PER_FRAME],
    /// Frame-level coarse SNR offset (`frmcsnroffst`, 6 bits). Only
    /// when `snroffststr == 0`.
    pub frmcsnroffst: u8,
    /// Frame-level fine SNR offset (`frmfsnroffst`, 4 bits). Only
    /// when `snroffststr == 0`.
    pub frmfsnroffst: u8,
    /// Total bits the parser consumed (handy for callers that share
    /// a bit cursor and want to seek to the start of `audblk[0]`).
    pub bits_consumed: u64,
    /// Per-block `cplinu[blk]` — surfaced from audfrm so the audblk
    /// parser knows whether to read the coupling-coordinate block.
    /// `false` for blocks with no coupling. Always `false` when
    /// `acmod ≤ 1` (no coupling possible).
    pub cplinu_blk: [bool; MAX_BLOCKS_PER_FRAME],
    /// Per-block `cplstre[blk]` — coupling-strategy-exists flag per
    /// block. Block 0 is always implicit `1` per §E.1.2.2 / Table E1.3
    /// (the spec only transmits `cplinu[0]`); subsequent blocks
    /// transmit `cplstre[blk]` explicitly. Surfaced so the round-5
    /// audblk DSP path knows when to expect the coupling-strategy
    /// fields (chincpl[], cplbegf, …) versus reusing the prior block's
    /// strategy with fresh coordinates.
    pub cplstre_blk: [bool; MAX_BLOCKS_PER_FRAME],
    /// Number of blocks with `cplinu[blk] == 1`. Convenient summary
    /// for the round-2 DSP path which rejects any non-zero value.
    pub ncplblks: u32,
}

impl AudFrm {
    fn new() -> Self {
        Self {
            expstre: true,
            ahte: false,
            snroffststr: 0,
            transproce: false,
            blkswe: true,
            dithflage: true,
            bamode: true,
            frmfgaincode: false,
            dbaflde: true,
            skipflde: true,
            spxattene: false,
            frmcplexpstr: 0xFF,
            frmchexpstr: [0xFF; MAX_FBW],
            lfeexpstr: [0; MAX_BLOCKS_PER_FRAME],
            frmcsnroffst: 0,
            frmfsnroffst: 0,
            bits_consumed: 0,
            cplinu_blk: [false; MAX_BLOCKS_PER_FRAME],
            cplstre_blk: [false; MAX_BLOCKS_PER_FRAME],
            ncplblks: 0,
        }
    }
}

/// Parse `audfrm()` per Table E1.3.
///
/// `bsi.num_blocks` must equal the number of audio blocks per
/// syncframe (1, 2, 3, or 6). The parser uses it to walk the
/// per-block lfeexpstr / blkstrtinfoe runs.
pub fn parse_with(br: &mut BitReader<'_>, bsi: &Bsi) -> Result<AudFrm> {
    let start_bits = br.bit_position();
    let mut a = AudFrm::new();

    let num_blocks = bsi.num_blocks as usize;
    let nfchans = bsi.nfchans as usize;
    let lfeon = bsi.lfeon;

    // §E.2.2.3 / §E.2.3.2 — 6-block syncframes carry expstre+ahte; for
    // smaller frames the spec hard-codes expstre=1 and ahte=0.
    if num_blocks == MAX_BLOCKS_PER_FRAME {
        a.expstre = br.read_u32(1)? != 0;
        a.ahte = br.read_u32(1)? != 0;
    } else {
        a.expstre = true;
        a.ahte = false;
    }
    a.snroffststr = br.read_u32(2)? as u8;
    a.transproce = br.read_u32(1)? != 0;
    a.blkswe = br.read_u32(1)? != 0;
    a.dithflage = br.read_u32(1)? != 0;
    a.bamode = br.read_u32(1)? != 0;
    a.frmfgaincode = br.read_u32(1)? != 0;
    a.dbaflde = br.read_u32(1)? != 0;
    a.skipflde = br.read_u32(1)? != 0;
    a.spxattene = br.read_u32(1)? != 0;

    // ---- coupling data ----
    //
    // For acmod > 0x1, block 0 carries an explicit `cplinu[0]` flag
    // and subsequent blocks emit (cplstre[blk], optional cplinu[blk])
    // pairs. The pure parser doesn't need to remember which blocks
    // had coupling — it just consumes the bits.
    //
    // For acmod ≤ 0x1, every block has coupling implicitly off and
    // there is nothing to consume.
    let mut ncplblks = 0u32;
    if bsi.acmod > 0x1 {
        // cplstre[0] is fixed at 1 (not transmitted); cplinu[0] is 1
        // bit.
        let cplinu0 = br.read_u32(1)?;
        ncplblks += cplinu0;
        a.cplinu_blk[0] = cplinu0 != 0;
        a.cplstre_blk[0] = true;
        let mut last_cplinu = cplinu0;
        for blk in 1..num_blocks {
            let cplstre = br.read_u32(1)? != 0;
            a.cplstre_blk[blk] = cplstre;
            if cplstre {
                let v = br.read_u32(1)?;
                last_cplinu = v;
            }
            ncplblks += last_cplinu;
            a.cplinu_blk[blk] = last_cplinu != 0;
        }
    }
    a.ncplblks = ncplblks;

    // ---- exponent strategy data ----
    //
    // §E.1.2.2 / Table E1.3 — when expstre == 1, per-block per-channel
    // strategy codes live in **audblk()**, NOT audfrm. We consume
    // exactly zero bits from audfrm in that branch.
    //
    // When expstre == 0 (frame-level strategies), audfrm carries the
    // 5-bit `frmcplexpstr` (only if coupling is in use somewhere) +
    // 5-bit `frmchexpstr[ch]` per fbw channel. The 5-bit codeword
    // expands per Table E2.10 to a 6-block strategy run via the
    // chexpstr_run table.
    //
    // `lfeexpstr[blk]` — same rule: when expstre == 1, lfe strategies
    // live per-block in audblk(); they only appear in audfrm when
    // expstre == 0 (one bit per block). The previous round-1 parser
    // unconditionally walked these bits — which over-consumed the
    // audfrm by 12 + lfe×6 bits whenever expstre==1, sliding every
    // subsequent field (frmcsnroffst, dba, skip, audblk header) into
    // the wrong bit slots. Round 2 corrects this.
    if !a.expstre {
        if bsi.acmod > 0x1 && ncplblks > 0 {
            a.frmcplexpstr = br.read_u32(5)? as u8;
        }
        for ch in 0..nfchans {
            a.frmchexpstr[ch] = br.read_u32(5)? as u8;
        }
        if lfeon {
            for blk in 0..num_blocks {
                a.lfeexpstr[blk] = br.read_u32(1)? as u8;
            }
        }
    }

    // ---- converter exponent strategy data ----
    //
    // strmtyp == 0 (independent substream): when numblkscod != 0x3 a
    // 1-bit `convexpstre` flag controls whether per-channel 5-bit
    // `convexpstr` codes follow. With numblkscod == 0x3, `convexpstre`
    // is implicit = 1 and the codes are always present.
    if matches!(bsi.strmtyp, StreamType::Independent) {
        let convexpstre_present = if num_blocks != MAX_BLOCKS_PER_FRAME {
            br.read_u32(1)? != 0
        } else {
            true
        };
        if convexpstre_present {
            for _ch in 0..nfchans {
                let _convexpstr = br.read_u32(5)?;
            }
        }
    }

    // ---- AHT data ----
    //
    // Per §E.2.3.5 / Table E1.3, `ahte` is in scope only when
    // `expstre == 1`. When `ahte == 1`, audfrm carries `ahtinu[ch]`
    // (1 bit) for every fbw channel whose 6-block exponent
    // strategies are all REUSE, and one `ahtinu_lfe`. The per-
    // channel "all-REUSE" determination requires reading the audblk
    // strategy bits **before** we get to the AHT block — but those
    // bits live in audblk[0]..audblk[5], past the audfrm boundary.
    //
    // Round 4 stub: we surface `ahte == true` to the DSP path (which
    // bails with `Unsupported`) without attempting to consume the
    // variable-length `ahtinu[ch]` bits. The bit cursor will land in
    // the wrong place for any subsequent fields the dsp path reads,
    // but the caller treats the whole frame as silent on Unsupported
    // so cursor drift is contained.
    //
    // The `eac3-low-bitrate-32kbps` fixture (which uses AHT at low
    // bit budgets per its `notes.md`) will mute. A real round-4
    // implementation requires a 2-pass decode (scan audblks for
    // chexpstr first, then re-walk audfrm + audblks for AHT) plus
    // the §E.2.2.4 Karhunen-Loeve VQ codebooks — substantial work
    // deferred to a follow-up.
    if a.ahte {
        return Err(Error::Unsupported(
            "eac3 audfrm: ahte == 1 (AHT in use) — round 4 stub mutes; \
             real AHT decode (Karhunen-Loeve VQ codebooks per §E.2.2.4) \
             is a follow-up"
                .to_string(),
        ));
    }

    // ---- audio frame SNR offset data ----
    if a.snroffststr == 0 {
        a.frmcsnroffst = br.read_u32(6)? as u8;
        a.frmfsnroffst = br.read_u32(4)? as u8;
    }
    // snroffststr 1 / 2 → per-block values, parsed inside audblk.

    // ---- transient pre-noise processing ----
    if a.transproce {
        for _ch in 0..nfchans {
            let chintransproc = br.read_u32(1)? != 0;
            if chintransproc {
                let _transprocloc = br.read_u32(10)?;
                let _transproclen = br.read_u32(8)?;
            }
        }
    }

    // ---- spectral extension attenuation data ----
    if a.spxattene {
        for _ch in 0..nfchans {
            let chinspxatten = br.read_u32(1)? != 0;
            if chinspxatten {
                let _spxattencod = br.read_u32(5)?;
            }
        }
    }

    // ---- block start information ----
    //
    // Only present for frames with > 1 block (numblkscod != 0); flagged
    // by `blkstrtinfoe`. When set, `blkstrtinfo` follows with
    // `nblkstrtbits` bits. nblkstrtbits is derived from frmsiz per
    // §2.3.2.27. Round-1 parser simply consumes the byte-aligned-up
    // budget when the flag is set; for fixtures we care about
    // (numblkscod=3 → 6 blocks), we follow the spec formula.
    if num_blocks != 1 {
        let blkstrtinfoe = br.read_u32(1)? != 0;
        if blkstrtinfoe {
            // nblkstrtbits = (numblks - 1) * (4 + ceil(log2(frmsiz_bits)))
            // For numblks=6 and frmsiz_bits ≤ 16, log2 ≤ 4 → 8 bits per
            // entry → 5*8 = 40 bits. We use the spec-correct formula.
            let frame_bits = bsi.frame_bytes * 8;
            let log2 = 32 - frame_bits.leading_zeros();
            let bits_per = 4 + log2;
            let total = (num_blocks as u32 - 1) * bits_per;
            br.skip(total)?;
        }
    }

    // ---- per-channel state initialisation flags ----
    //
    // The spec requires the syntax-state init for every channel in the
    // syncframe (firstspxcos[ch] = 1, firstcplcos[ch] = 1, firstcplleak
    // = 1) — these are stateful initialisers, not bit-field reads.
    // Nothing to consume here.

    a.bits_consumed = br.bit_position() - start_bits;
    Ok(a)
}

/// Convenience parser that creates a fresh [`BitReader`] over `data`.
pub fn parse(data: &[u8], bsi: &Bsi) -> Result<AudFrm> {
    let mut br = BitReader::new(data);
    parse_with(&mut br, bsi)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_bsi(acmod: u8, lfeon: bool, num_blocks: u8, strmtyp: StreamType) -> Bsi {
        Bsi {
            strmtyp,
            substreamid: 0,
            frmsiz: 383,
            fscod: 0,
            fscod2: 0xFF,
            sample_rate: 48_000,
            numblkscod: if num_blocks == 6 { 3 } else { num_blocks - 1 },
            num_blocks,
            acmod,
            nfchans: crate::tables::acmod_nfchans(acmod),
            lfeon,
            nchans: crate::tables::acmod_nfchans(acmod) + u8::from(lfeon),
            bsid: 16,
            dialnorm: 27,
            chanmap: None,
            frame_bytes: 768,
            bits_consumed: 0,
        }
    }

    fn pack_msb(bits: &[(u32, u32)]) -> Vec<u8> {
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
        out
    }

    /// 2/0 stereo, 6 blocks, all strategy flags at the encoder's
    /// preferred values (expstre=1, blkswe=1, dithflage=1, bamode=1,
    /// dbaflde=1, skipflde=1; ahte=0, transproce=0, spxattene=0,
    /// snroffststr=0, frmfgaincode=0). acmod=2 has no LFE and (without
    /// a coupling block) no per-block coupling bits.
    #[test]
    fn parses_minimal_indep_stereo_audfrm() {
        let bsi = make_bsi(2, false, 6, StreamType::Independent);
        // Strategy flags (numblkscod==3 → 6 blocks)
        let mut bits: Vec<(u32, u32)> = vec![(1, 1)]; // expstre
        bits.push((1, 0)); // ahte
        bits.push((2, 0)); // snroffststr
        bits.push((1, 0)); // transproce
        bits.push((1, 1)); // blkswe
        bits.push((1, 1)); // dithflage
        bits.push((1, 1)); // bamode
        bits.push((1, 0)); // frmfgaincode
        bits.push((1, 1)); // dbaflde
        bits.push((1, 1)); // skipflde
        bits.push((1, 0)); // spxattene
                           // acmod>1 → block 0 cplinu, then 5*(cplstre[, cplinu]) pairs.
        bits.push((1, 0)); // cplinu[0] = 0
        for _ in 1..6 {
            bits.push((1, 0)); // cplstre[blk] = 0
        }
        // expstre==1 → per-block per-channel chexpstr lives in audblk(),
        // not audfrm. (Round-2 bug fix: round 1 over-consumed 24 bits
        // here.)
        // strmtyp == 0 + numblkscod == 0x3 → convexpstre implicit = 1,
        // followed by per-channel convexpstr (5 bits each).
        for _ in 0..2 {
            bits.push((5, 0));
        }
        // ahte=0 → no AHT block.
        // snroffststr=0 → frmcsnroffst (6) + frmfsnroffst (4)
        bits.push((6, 15));
        bits.push((4, 0));
        // transproce=0, spxattene=0 → nothing.
        // num_blocks > 1 → blkstrtinfoe (1 bit, 0).
        bits.push((1, 0));

        let buf = pack_msb(&bits);
        let af = parse(&buf, &bsi).unwrap();
        assert!(af.expstre);
        assert!(af.blkswe);
        assert!(af.dithflage);
        assert!(af.bamode);
        assert!(af.dbaflde);
        assert!(af.skipflde);
        assert!(!af.ahte);
        assert!(!af.transproce);
        assert!(!af.spxattene);
        assert_eq!(af.snroffststr, 0);
        assert_eq!(af.frmcsnroffst, 15);
        assert_eq!(af.frmfsnroffst, 0);
    }
}
