//! E-AC-3 audio-block DSP pipeline — rounds 2 + 4 stub.
//!
//! Translates the parsed [`super::bsi::Bsi`] + [`super::audfrm::AudFrm`]
//! into the existing AC-3 [`crate::audblk::Ac3State`] shape so the §7
//! DSP helpers (`decode_exponents`, `run_bit_allocation`,
//! `unpack_mantissas`, `dsp_block`) can be reused without modification.
//!
//! ## Round 4 (this commit)
//!
//! AHT and SPX are E-AC-3-specific psychoacoustic features that gate
//! decode for any fixture using them. The round-4 **stub** in this
//! commit:
//!
//! * Surfaces `ahte == 1` as a clear `Error::Unsupported` from the
//!   audfrm parser instead of silently skipping bits (round 1
//!   incorrectly consumed AHT bits as if they were always-emitted).
//! * Tightens the `spxinu == 1` rejection in the audblk parser with
//!   a spec citation (§E.2.2.5.4).
//! * Documents the path forward: a real round-4-bis lands the §E.2.2.4
//!   Karhunen-Loeve VQ codebooks (AHT) and the §E.2.2.5.4 SBR-style
//!   parametric high-frequency reconstruction (SPX). Both are
//!   substantial: AHT requires a 2-pass audblk decode (scan
//!   chexpstr, then re-walk for AHT in-use bits) plus the VQ
//!   codebook tables themselves; SPX needs the spxcoexp/spxcomant
//!   coordinate decoding + the noise-blend / amplitude-fold
//!   reconstruction pipeline.
//!
//! Fixtures unblocked by round 4-bis: `eac3-low-bitrate-32kbps` (AHT
//! at low bit budgets per its `notes.md`). No corpus fixture
//! exercises SPX (FFmpeg's eac3 encoder doesn't emit it).
//!
//! ## Scope (round 2)
//!
//! The round-2 DSP path covers the **simple-encoder happy path** that
//! the bulk of the corpus fixtures actually exercise:
//!
//! * `expstre == 1`  — per-block per-channel chexpstr (no frame-level
//!   strategy run; we don't currently translate the §E.1.3.4.4
//!   chexpstr-codeword runs from frmchexpstr into per-block strategies).
//! * `bamode  == 1`  — per-block bit-allocation parametric info.
//! * `dithflage == 1` — per-block per-channel dithflag (otherwise spec
//!   says implicit 1).
//! * `blkswe   == 1` — per-block blksw (otherwise implicit 0).
//! * `dbaflde  == 1` — delta-bit-allocation may appear per block.
//! * `skipflde == 1` — skip-field may appear per block.
//! * `snroffststr == 0` — single frame-level (csnroffst, fsnroffst).
//! * No coupling, no enhanced coupling.
//! * No spectral extension (`spxinu == 0` always).
//! * No AHT (`ahte == 0` — gated upstream in audfrm parser).
//! * No transient pre-noise processing (`transproce == 0`).
//!
//! When any of these conditions is violated the parser returns
//! [`oxideav_core::Error::Unsupported`] and the caller (the decoder
//! in [`super::decoder`]) falls back to silent emit for that frame.
//!
//! ## Bit syntax — Table E1.4 (audblk()) verbatim, simplified
//!
//! ```text
//!   if (blkswe)   for (ch=0..nfchans) blksw[ch]    1
//!   if (dithflage) for (ch=0..nfchans) dithflag[ch] 1
//!   dynrnge                                          1
//!   if (dynrnge) dynrng                              8
//!   if (acmod==0) {
//!     dynrng2e                                       1
//!     if (dynrng2e) dynrng2                          8
//!   }
//!   if (cplstre[blk])      cplinu[blk]               (parsed in audfrm)
//!   /* coupling block — skipped: cplinu = 0 */
//!   if (acmod==2) {
//!     rematstr                                       1
//!     if (rematstr) rematflg[0..n]                   1 each
//!   }
//!   /* exponent strategies */
//!   if (cplinu[blk]) cplexpstr                       2
//!   for (ch=0..nfchans) chexpstr[ch]                 2  (when expstre==1)
//!   if (lfeon)        lfeexpstr                      1
//!   /* exponents */
//!   if (cplinu[blk] && cplexpstr!=0) — coupling exponents
//!   for (ch) if (chexpstr[ch]!=0)    — chbwcod[ch] (6) + grouped exponents
//!   if (lfeon && lfeexpstr!=0)        — LFE exponents
//!   /* bit-allocation parametric */
//!   if (bamode) {
//!     baie                                            1
//!     if (baie) sdcycod(2) fdcycod(2) sgaincod(2) dbpbcod(2) floorcod(3)
//!   }
//!   if (snroffststr==0) — uses frame-level (csnroffst, fsnroffst).
//!   else                — per-block snroffste etc. (NOT IN ROUND 2)
//!   if (frmfgaincode)   — fgaincode per block. (We treat as no extra bits.)
//!   /* dba */
//!   if (dbaflde) {
//!     deltbaie                                        1
//!     ... (same as AC-3) ...
//!   }
//!   if (skipflde) {
//!     skiple                                          1
//!     if (skiple) skipl(9) + skipfld(skipl*8)
//!   }
//!   /* mantissas — bap-driven, identical to AC-3 unpack_mantissas. */
//! ```

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::audblk::{self, Ac3State, BLOCKS_PER_FRAME, MAX_FBW, SAMPLES_PER_BLOCK};
use crate::bsi::Bsi as Ac3Bsi;
use crate::syncinfo::SyncInfo;

use super::audfrm::AudFrm;
use super::bsi::Bsi as Eac3Bsi;

/// Decode one E-AC-3 independent substream's audblks into interleaved
/// f32 PCM. Returns `Ok(())` on a successful clean walk, or `Err(...)`
/// if any block hits a feature we don't support (caller substitutes
/// silence).
///
/// `out` length must equal `bsi.num_blocks * 256 * bsi.nchans`.
pub fn decode_indep_audblks(
    bsi: &Eac3Bsi,
    audfrm: &AudFrm,
    br: &mut BitReader<'_>,
    state: &mut Ac3State,
    out: &mut [f32],
) -> Result<()> {
    // Reject cases the round-2 parser does not handle.
    reject_unsupported(bsi, audfrm)?;

    // Build a "shim" AC-3 BSI + SyncInfo so the reused helpers see the
    // shape they expect.
    let ac3_bsi = build_ac3_bsi_shim(bsi);
    let si = build_syncinfo_shim(bsi);

    // Top-level frame init mirrors §7.2.2.6: clear delta-segment counts.
    for n in state.deltnseg.iter_mut() {
        *n = 0;
    }

    let nfchans = bsi.nfchans as usize;
    let nchans = bsi.nchans as usize;
    let lfeon = bsi.lfeon;
    let num_blocks = bsi.num_blocks as usize;
    let _ = BLOCKS_PER_FRAME; // unused once cplinu_blk migrated to audfrm

    for blk in 0..num_blocks {
        state.blkidx = blk;

        // ---- §E.1.3.1 blksw[ch] ----
        if audfrm.blkswe {
            for ch in 0..nfchans {
                let v = br.read_u32(1)? != 0;
                state.channels[ch].blksw = v;
            }
        } else {
            for ch in 0..nfchans {
                state.channels[ch].blksw = false;
            }
        }

        // ---- §E.1.3.2 dithflag[ch] ----
        if audfrm.dithflage {
            for ch in 0..nfchans {
                let v = br.read_u32(1)? != 0;
                state.channels[ch].dithflag = v;
            }
        } else {
            for ch in 0..nfchans {
                state.channels[ch].dithflag = true;
            }
        }

        // ---- dynrng ----
        let dynrnge = br.read_u32(1)? != 0;
        if dynrnge {
            let dynrng = br.read_u32(8)? as u8;
            let g = dynrng_to_linear(dynrng);
            for ch in 0..nfchans {
                state.channels[ch].dynrng = g;
            }
        } else if blk == 0 {
            for ch in 0..nfchans {
                state.channels[ch].dynrng = 1.0;
            }
        }
        if bsi.acmod == 0 {
            let dynrng2e = br.read_u32(1)? != 0;
            if dynrng2e {
                let d2 = br.read_u32(8)? as u8;
                state.channels[1].dynrng = dynrng_to_linear(d2);
            } else if blk == 0 {
                state.channels[1].dynrng = 1.0;
            }
        }

        // ---- spectral extension strategy block (§E.1.3.5.1) ----
        //
        // Per Table E1.4, blk 0 has implicit `spxstre = 1` with the
        // 1-bit `spxinu[0]` emitted directly; subsequent blocks emit
        // `spxstre[blk]` (1 bit) + (only if spxstre[blk]) `spxinu[blk]`.
        //
        // Round 4 stub: SPX-active frames mute. The spec (§E.1.3.5.1
        // / §E.2.2.5.4) describes a parametric high-frequency
        // reconstruction (similar to AAC's SBR): bins in the SPX
        // region [spxbegf .. spxendf] are derived from low-frequency
        // bins via per-band amplitude (`spxbndcoeff`) + blending
        // (`spxblnd`) coefficients, plus a coupling-style coordinate
        // table (`mstrspxco`/`spxcoexp`/`spxcomant`). FFmpeg's eac3
        // encoder doesn't emit SPX (per `eac3-low-rate-stereo-64kbps/
        // notes.md` "gap"), so no corpus fixture currently exercises
        // this path.
        let spxstre = if blk == 0 { true } else { br.read_u32(1)? != 0 };
        if spxstre {
            let spxinu = br.read_u32(1)? != 0;
            if spxinu {
                return Err(Error::unsupported(
                    "eac3 audblk: spxinu == 1 (spectral extension active) — \
                     round 4 stub mutes; full §E.2.2.5.4 SPX decode is a follow-up",
                ));
            }
        }
        // No additional SPX coordinate fields when spxinu == 0.

        // ---- coupling strategy block ----
        // Per §E.1.2 Table E1.3, the cplstre/cplinu bits live in
        // **audfrm**, not audblk. Round-1 audfrm parser already
        // consumed them. The audblk emits the coupling coordinate
        // block only when `cplinu[blk] == 1`; we forced that case to
        // return Unsupported via `reject_unsupported`, so for round 2
        // we never enter that path.
        // (Note: the round-1 audfrm doesn't return per-block cplinu;
        // we conservatively rely on `reject_unsupported` to gate.)

        // ---- §E.1.3.4 / §7.5 rematrixing — only for 2/0 (acmod==2) ----
        // Block 0 is special: encoder emits rematflg directly without
        // a rematstr gate; subsequent blocks emit rematstr first and
        // only if set do rematflg follow. AC-3 §5.4.3.19 has the same
        // shape for base AC-3.
        if bsi.acmod == 0x2 {
            let rematstr = if blk == 0 { true } else { br.read_u32(1)? != 0 };
            if rematstr {
                let n_remat = crate::audblk::remat_band_count(false, 0);
                for rbnd in 0..n_remat {
                    let v = br.read_u32(1)? != 0;
                    state.rematflg[rbnd] = v;
                }
            }
        }

        // ---- §E.1.3.4 exponent strategy ----
        // Round 2 requires expstre == 1 (per-block strategies live here).
        // No coupling, so no cplexpstr. fbw chexpstr (2 each) + LFE (1).
        let mut chexpstr = [0u8; MAX_FBW];
        for ch in 0..nfchans {
            chexpstr[ch] = br.read_u32(2)? as u8;
        }
        let mut lfeexpstr = 0u8;
        if lfeon {
            lfeexpstr = br.read_u32(1)? as u8;
        }

        // chbwcod — only when chexpstr != REUSE and channel uncoupled.
        let mut chbwcod = [0u8; MAX_FBW];
        for ch in 0..nfchans {
            if chexpstr[ch] != 0 {
                chbwcod[ch] = br.read_u32(6)? as u8;
                if chbwcod[ch] > 60 {
                    return Err(Error::invalid(
                        "eac3 audblk: chbwcod > 60 (E.1.3.4.6 invalid)",
                    ));
                }
            }
        }

        // ---- exponents ----
        for ch in 0..nfchans {
            if chexpstr[ch] != 0 {
                let end = 37 + 3 * (chbwcod[ch] as usize + 12);
                state.channels[ch].end_mant = end;
                let absexp = br.read_u32(4)? as i32;
                let grpsize = match chexpstr[ch] {
                    1 => 1,
                    2 => 2,
                    3 => 4,
                    _ => 1,
                };
                let nchgrps = match chexpstr[ch] {
                    1 => (end - 1) / 3,
                    2 => (end - 1).div_ceil(6),
                    3 => (end - 1 + 9) / 12,
                    _ => 0,
                };
                let mut raw_exp = vec![0i32; nchgrps * 3];
                audblk::decode_exponents(br, absexp, nchgrps, chexpstr[ch] as usize, &mut raw_exp)?;
                state.channels[ch].exp[0] = absexp.clamp(0, 24) as u8;
                for (i, e) in raw_exp.iter().enumerate() {
                    let base = i * grpsize + 1;
                    for j in 0..grpsize {
                        if base + j < end {
                            state.channels[ch].exp[base + j] = (*e).clamp(0, 24) as u8;
                        }
                    }
                }
                // E-AC-3 audblk does NOT carry gainrng (that field is
                // base-AC-3 only). Skip.
            } else if blk == 0 {
                return Err(Error::invalid(
                    "eac3 audblk: chexpstr == 0 in block 0 (no prior exponents to reuse)",
                ));
            }
        }
        if lfeon && lfeexpstr != 0 {
            let lfe_ch = MAX_FBW + 1;
            state.channels[lfe_ch].end_mant = 7;
            let absexp = br.read_u32(4)? as i32;
            let nlfegrps = 2usize;
            let mut raw_exp = vec![0i32; nlfegrps * 3];
            audblk::decode_exponents(br, absexp, nlfegrps, 1, &mut raw_exp)?;
            state.channels[lfe_ch].exp[0] = absexp.clamp(0, 24) as u8;
            for (i, e) in raw_exp.iter().enumerate() {
                if i + 1 < 7 {
                    state.channels[lfe_ch].exp[i + 1] = (*e).clamp(0, 24) as u8;
                }
            }
        }

        // ---- §E.1.3.5 bit-allocation parametric info ----
        if audfrm.bamode {
            let baie = br.read_u32(1)? != 0;
            if baie {
                state.sdcycod = br.read_u32(2)? as u8;
                state.fdcycod = br.read_u32(2)? as u8;
                state.sgaincod = br.read_u32(2)? as u8;
                state.dbpbcod = br.read_u32(2)? as u8;
                state.floorcod = br.read_u32(3)? as u8;
            } else if blk == 0 {
                // §E.2.2.4 — "if bamode == 0 the encoder uses default
                // BA params". We use the spec's default codewords
                // (Table E1.4 footnote / §E.2.2.4).
                state.sdcycod = 0x2;
                state.fdcycod = 0x1;
                state.sgaincod = 0x1;
                state.dbpbcod = 0x2;
                state.floorcod = 0x7;
            }
        } else if blk == 0 {
            // Same defaults when bamode == 0.
            state.sdcycod = 0x2;
            state.fdcycod = 0x1;
            state.sgaincod = 0x1;
            state.dbpbcod = 0x2;
            state.floorcod = 0x7;
        }

        // ---- SNR offset ----
        // Round 2 supports snroffststr == 0 (single frame-level).
        // Block 0 of each frame initialises from frmcsnroffst /
        // frmfsnroffst; subsequent blocks reuse the same value.
        if blk == 0 {
            state.snroffst_coarse = audfrm.frmcsnroffst;
            for ch in 0..nfchans {
                state.fsnroffst[ch] = audfrm.frmfsnroffst;
            }
            if lfeon {
                state.lfefsnroffst = audfrm.frmfsnroffst;
            }
            // §E.2.2.4: "If bamode == 0 the encoder uses fast-gain
            // codeword 0x4 (mid)". Carry through for fgaincod when
            // frmfgaincode == 0 (no per-block fgaincod).
            for ch in 0..nfchans {
                state.fgaincod[ch] = 0x4;
            }
            if lfeon {
                state.lfefgaincod = 0x4;
            }
        }

        // ---- §E.1.3.5.4 frmfgaincode (per-block fgain override) ----
        if audfrm.frmfgaincode {
            let fgaincode = br.read_u32(1)? != 0;
            if fgaincode {
                for ch in 0..nfchans {
                    state.fgaincod[ch] = br.read_u32(3)? as u8;
                }
                if lfeon {
                    state.lfefgaincod = br.read_u32(3)? as u8;
                }
            }
        }

        // ---- §E.1.3.5.5 dba ----
        if audfrm.dbaflde {
            let dbaie = br.read_u32(1)? != 0;
            if dbaie {
                let mut deltbae = [0u32; MAX_FBW];
                for ch in 0..nfchans {
                    deltbae[ch] = br.read_u32(2)?;
                }
                for ch in 0..nfchans {
                    match deltbae[ch] {
                        1 => {
                            let nseg = (br.read_u32(3)? + 1) as usize;
                            state.deltnseg[ch] = nseg.min(8);
                            for seg in 0..state.deltnseg[ch] {
                                state.deltoffst[ch][seg] = br.read_u32(5)? as u8;
                                state.deltlen[ch][seg] = br.read_u32(4)? as u8;
                                state.deltba[ch][seg] = br.read_u32(3)? as u8;
                            }
                        }
                        2 => {
                            state.deltnseg[ch] = 0;
                        }
                        _ => {}
                    }
                }
            } else if blk == 0 {
                for ch in 0..MAX_FBW + 1 {
                    state.deltnseg[ch] = 0;
                }
            }
        } else if blk == 0 {
            for ch in 0..MAX_FBW + 1 {
                state.deltnseg[ch] = 0;
            }
        }

        // ---- §E.1.3.5.6 skip ----
        if audfrm.skipflde {
            let skiple = br.read_u32(1)? != 0;
            if skiple {
                let skipl = br.read_u32(9)?;
                br.skip(skipl * 8)?;
            }
        }

        // ---- bit allocation ----
        for ch in 0..nfchans {
            let end = state.channels[ch].end_mant;
            audblk::run_bit_allocation(
                state,
                ch,
                0,
                end,
                si.fscod,
                state.fsnroffst[ch],
                state.fgaincod[ch],
                false,
            );
        }
        if lfeon {
            let lfe_ch = MAX_FBW + 1;
            audblk::run_bit_allocation(
                state,
                lfe_ch,
                0,
                7,
                si.fscod,
                state.lfefsnroffst,
                state.lfefgaincod,
                false,
            );
        }

        // ---- mantissas ----
        audblk::unpack_mantissas(state, &ac3_bsi, br)?;

        // ---- DSP (decouple+rematrix+dynrng+IMDCT+overlap-add) ----
        audblk::dsp_block(state, &si, &ac3_bsi);

        // Write block PCM into `out`.
        let base = blk * SAMPLES_PER_BLOCK * nchans;
        for n in 0..SAMPLES_PER_BLOCK {
            for ch in 0..nfchans {
                let s = state.channels[ch].coeffs[n];
                out[base + n * nchans + ch] = s;
            }
            if lfeon {
                let s = state.channels[MAX_FBW + 1].coeffs[n];
                out[base + n * nchans + nfchans] = s;
            }
        }
    }
    Ok(())
}

/// Decide whether the round-2 DSP path can handle this frame.
fn reject_unsupported(bsi: &Eac3Bsi, audfrm: &AudFrm) -> Result<()> {
    if !audfrm.expstre {
        return Err(Error::unsupported(
            "eac3 dsp: frame-level exponent strategy (expstre==0) — round 2 only does per-block",
        ));
    }
    if audfrm.snroffststr != 0 {
        return Err(Error::unsupported(format!(
            "eac3 dsp: snroffststr={} — round 2 only handles 0 (single frame value)",
            audfrm.snroffststr
        )));
    }
    if audfrm.transproce {
        return Err(Error::unsupported(
            "eac3 dsp: transient pre-noise processing in use — round 2 mutes",
        ));
    }
    if audfrm.spxattene {
        return Err(Error::unsupported(
            "eac3 dsp: spectral-extension attenuation in use — round 2 mutes",
        ));
    }
    if audfrm.ahte {
        // Defensive — the audfrm parser bails before returning a
        // valid `AudFrm` when ahte == 1, so we should never see this
        // case. Kept as a safety net for any future audfrm path that
        // surfaces ahte without bailing.
        return Err(Error::unsupported(
            "eac3 dsp: AHT in use — round 4 stub mutes",
        ));
    }
    if audfrm.ncplblks > 0 {
        return Err(Error::unsupported(format!(
            "eac3 dsp: coupling in use ({} of {} blocks) — round 2 mutes",
            audfrm.ncplblks, bsi.num_blocks,
        )));
    }
    if bsi.frmsiz == 0 {
        return Err(Error::invalid("eac3 dsp: frmsiz=0 (would be 2-byte frame)"));
    }
    Ok(())
}

/// Build a synthetic [`Ac3Bsi`] from the parsed [`Eac3Bsi`] so the
/// AC-3 helpers (which read `bsi.nfchans`/`bsi.nchans`/`bsi.lfeon`) see
/// the shape they expect.
fn build_ac3_bsi_shim(bsi: &Eac3Bsi) -> Ac3Bsi {
    Ac3Bsi {
        bsid: bsi.bsid,
        bsmod: 0,
        acmod: bsi.acmod,
        nfchans: bsi.nfchans,
        lfeon: bsi.lfeon,
        nchans: bsi.nchans,
        dialnorm: bsi.dialnorm,
        cmixlev: 0xFF,
        surmixlev: 0xFF,
        dsurmod: 0xFF,
        bits_consumed: 0,
    }
}

/// Build a [`SyncInfo`] shim — only `fscod` is consumed downstream.
fn build_syncinfo_shim(bsi: &Eac3Bsi) -> SyncInfo {
    let fscod_for_ba = match bsi.sample_rate {
        48_000 => 0,
        44_100 => 1,
        32_000 => 2,
        // Reduced-rate (24/22.05/16 kHz) — round 2 maps these to the
        // closest base-AC-3 fscod for the masking-curve table HTH lookup.
        // §E.2.2.4 says "the masking model uses the (fscod, fscod2)
        // pair to index a doubled-row HTH"; we approximate with the
        // closest non-reduced row. PSNR will be a bit off on reduced
        // streams; round-3 follow-up to fix.
        24_000 | 22_050 | 16_000 => 2,
        _ => 0,
    };
    SyncInfo {
        crc1: 0,
        fscod: fscod_for_ba,
        frmsizecod: 0,
        sample_rate: bsi.sample_rate,
        frame_length: bsi.frame_bytes,
    }
}

/// Convert an 8-bit dynrng word to linear gain (§7.7.1.2). Same as
/// AC-3 (§E.2 reuses the base spec). Duplicated here to avoid making
/// the AC-3 private helper public.
fn dynrng_to_linear(dynrng: u8) -> f32 {
    let x = ((dynrng >> 5) & 0x7) as i32;
    let x_signed = if x >= 4 { x - 8 } else { x };
    let y = (dynrng & 0x1F) as i32;
    let y_val = (32 + y) as f32 / 64.0;
    let shift = x_signed + 1;
    let base = 2f32.powi(shift);
    base * y_val
}
