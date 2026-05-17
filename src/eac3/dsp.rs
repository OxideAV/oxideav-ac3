//! E-AC-3 audio-block DSP pipeline — rounds 2 / 4-stub / 5 / 6.
//!
//! Translates the parsed [`super::bsi::Bsi`] + [`super::audfrm::AudFrm`]
//! into the existing AC-3 [`crate::audblk::Ac3State`] shape so the §7
//! DSP helpers (`decode_exponents`, `run_bit_allocation`,
//! `unpack_mantissas`, `dsp_block`) can be reused without modification.
//!
//! ## Round 6 (this commit) — Adaptive Hybrid Transform (AHT)
//!
//! Mono-only AHT decode. The audblk loop now keeps a per-channel
//! AHT-coefficient cache (`aht_coeffs[ch][blk][bin]`) populated by
//! [`unpack_mixed_mantissas`] on the FIRST AHT-active block per
//! channel; subsequent blocks load coefficients from the cache and
//! emit zero mantissa bits for that channel. The audfrm parser is
//! split into phase A ([`super::audfrm::parse_with`]) + phase B
//! ([`super::audfrm::parse_phase_b`]) so the dsp can hand it the
//! pre-walked `nchregs[ch]` hints. For mono streams (`nfchans == 1`,
//! no LFE, no coupling), `nchregs[0]` is implicitly `1` whenever
//! `audfrm.ahte == 1`, so the dsp short-circuits the pre-walk.
//! Multichannel / LFE / coupled AHT is round-7 work — those frames
//! still mute via an `Unsupported` early return.
//!
//! Per-bin AHT decode flow:
//!
//! 1. Derive `hebap[bin]` from `psd[bin]` / `mask[masktab[bin]]` via
//!    [`super::aht::hebap_from_address`] (Table E3.1).
//! 2. Read 2-bit `chgaqmod` + `chgaqsections` 1- or 5-bit gain words.
//! 3. Per bin:
//!    * `hebap == 0` → zero coefficients across all 6 blocks.
//!    * `1 ≤ hebap ≤ 7` → 2..9-bit VQ codeword indexes into Tables
//!      E4.1..E4.7, returns 6 dequantised mantissas.
//!    * `hebap ≥ 8` → 6 scalar / GAQ-tagged mantissa reads (with the
//!      Gk gain factor applied per Table E3.5).
//! 4. Apply the §3.4.5 inverse DCT-II over the 6 mantissas to
//!    recover per-block C(k, m).
//! 5. Multiply by `2^-exp` and stash in `aht_coeffs[ch][blk][bin]`.
//!
//! ## Round 5 — standard coupling
//!
//! The audblk syntax for **standard** (non-enhanced) coupling per
//! Table E1.4 is wired end-to-end:
//!
//! * `cplstre[blk]` + `cplinu[blk]` come from [`AudFrm::cplstre_blk`]
//!   / [`AudFrm::cplinu_blk`] (audfrm-resident in Annex E, vs.
//!   audblk-resident in base AC-3).
//! * When `cplstre[blk] && cplinu[blk]`: parse `ecplinu` (1 bit),
//!   `chincpl[ch]` (per fbw channel, 1 bit each — implicit `1` for
//!   2/0), `phsflginu` (1 bit, only in 2/0), `cplbegf`/`cplendf`
//!   (4 bits each), `cplbndstrce` (1 bit) + per-subband
//!   `cplbndstrc[bnd]` (1 bit).
//! * When `cplinu[blk]`: parse `cplcoe[ch]` (1 bit, implicit `1` if
//!   `firstcplcos[ch]`) + `mstrcplco`/`cplcoexp`/`cplcomant` (4+4
//!   bits per band) + `phsflg[bnd]` (1 bit, only in 2/0).
//! * Coupling-channel exponents (`cplabsexp` 4 bits + grouped exps),
//!   `chbwcod[ch]` only for un-coupled channels, `cplleake` +
//!   `cplfleak`/`cplsleak`, `cpldeltbae` (delta-BA for the cpl
//!   channel) — all wired through to the existing [`Ac3State`] slots
//!   so [`audblk::dsp_block`] runs the §7.4 decouple step unchanged.
//!
//! `ecplinu == 1` (enhanced coupling) is rejected as `Unsupported` —
//! none of the FFmpeg eac3 encoder's outputs in the corpus exercise
//! that path; standard coupling covers all four 5.1 / low-rate
//! fixtures (eac3-5.1-48000-384kbps, eac3-5.1-side-768kbps,
//! eac3-low-rate-stereo-64kbps, eac3-from-ac3-bitstream-recombination).
//!
//! Other newly-handled fields:
//! * `convsnroffste` (1 bit, always present for `strmtyp == 0`) +
//!   optional 10-bit `convsnroffst` — was silently missing from
//!   round 2. Most fixtures had it = 0 so the missing bit aliased
//!   onto the next field cleanly, but coupled fixtures hit `cplleake`
//!   right after which made the misalignment visible.
//!
//! ## Round 4 (prior commit)
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
//! * Standard coupling (round 5); enhanced coupling still rejected.
//! * No spectral extension (`spxinu == 0` always).
//! * No AHT (`ahte == 0` — gated upstream in audfrm parser).
//! * No transient pre-noise processing (`transproce == 0`).
//!
//! When any of these conditions is violated the parser returns
//! [`oxideav_core::Error::Unsupported`] and the caller (the decoder
//! in [`super::decoder`]) falls back to silent emit for that frame.
//!
//! ## Bit syntax — Table E.1.4 (audblk()) verbatim, simplified
//!
//! Per ETSI TS 102 366 V1.4.1 §E.1.2.4 / ATSC A/52:2018 Table E1.4 the
//! per-block-per-channel `chexpstr`, per-block `cplexpstr`, and
//! per-block `lfeexpstr` strategy codes are emitted in **audfrm**
//! (Table E.1.3, gated by `expstre`), NOT in audblk. Audblk merely
//! consumes them as state via the `chexpstr[blk][ch] != reuse` /
//! `cplexpstr[blk] != reuse` / `lfeexpstr[blk] != reuse` gates that
//! decide whether the bandwidth code + exponent payload follow.
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
//!   if (blk == 0) spxstre = 1
//!   else          spxstre                            1
//!   if (spxstre)  spxinu (+ spx fields when set)     1+
//!   if (cplstre[blk])  ... coupling strategy fields ...   (cplstre/cplinu in audfrm)
//!   if (cplinu[blk])   ... coupling coordinates ...
//!   if (acmod==2) {
//!     if (blk == 0) rematstr = 1                       (implicit)
//!     else          rematstr                          1
//!     if (rematstr) rematflg[0..n]                    1 each
//!   }
//!   /* §E.1.2.4 chbwcod — gated by audfrm-supplied chexpstr */
//!   for (ch) if (chexpstr[blk][ch] != reuse && !chincpl && !chinspx)
//!                       chbwcod[ch]                  6
//!   /* exponents */
//!   if (cplinu[blk] && cplexpstr[blk] != reuse) — coupling exponents (cplabsexp + groups)
//!   for (ch) if (chexpstr[blk][ch] != reuse)    — exps[ch][0] + groups + gainrng[ch] (2)
//!   if (lfeon && lfeexpstr[blk] != reuse)       — LFE exponents (no gainrng)
//!   /* bit-allocation parametric */
//!   if (bamode) {
//!     baie                                            1
//!     if (baie) sdcycod(2) fdcycod(2) sgaincod(2) dbpbcod(2) floorcod(3)
//!   }
//!   if (snroffststr==0) — uses frame-level (csnroffst, fsnroffst).
//!   else                — per-block snroffste etc. (NOT IN ROUND 2)
//!   if (frmfgaincode)   — fgaincode (1 bit) per block, then 3 bits per channel if set.
//!   if (strmtyp == 0)   — convsnroffste (1 bit) [+10-bit convsnroffst when set]
//!   if (cplinu[blk]) — cplleak block (cplleake + cplfleak/cplsleak).
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

use crate::audblk::{self, Ac3State, BLOCKS_PER_FRAME, MAX_FBW, N_COEFFS, SAMPLES_PER_BLOCK};
use crate::bsi::Bsi as Ac3Bsi;
use crate::syncinfo::SyncInfo;

use super::aht::{self, AHT_BLOCKS};
use super::audfrm::{self, AhtRegsHints, AudFrm};
use super::bsi::{Bsi as Eac3Bsi, StreamType};

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
    // Phase-B audfrm finalisation when AHT is in use. The audfrm
    // parser stopped at the AHT anchor so the dsp can compute
    // nchregs[ch]/ncplregs/nlferegs from per-block exponent
    // strategies. For round 6 we restrict AHT support to **mono
    // streams with no LFE and no coupling** — the only configuration
    // exercised by the corpus's `eac3-low-bitrate-32kbps` fixture
    // (1 fbw channel, lfeon=0, ncplblks=0). In that case the AHT bit
    // count is fixed at 1 (chahtinu[0]) IFF `ahte == 1`, since the
    // spec mandates that ahte can only be set when at least one
    // chahtinu/cplahtinu/lfeahtinu emit a `1` — and the only candidate
    // is chahtinu[0]. Multichannel / LFE / coupling AHT requires the
    // 2-pass nchregs probe described in §3.4.2 and lands in round 7.
    let mut audfrm_local;
    let audfrm: &AudFrm = if audfrm.aht_phase_b_pending {
        if !(bsi.nfchans == 1 && !bsi.lfeon && audfrm.ncplblks == 0) {
            return Err(Error::unsupported(
                "eac3 dsp: AHT in use on multichannel / lfe / coupled stream — \
                 round 6 implements mono-only AHT (single chahtinu[0] bit). \
                 nchregs probe for the multichannel case is round-7 work.",
            ));
        }
        audfrm_local = audfrm.clone();
        let hints = AhtRegsHints {
            nchregs: {
                let mut h = [0u8; MAX_FBW];
                h[0] = 1; // mono-only: chahtinu[0] presence implies nchregs[0]==1.
                h
            },
            ncplregs: 0,
            nlferegs: 0,
        };
        audfrm::parse_phase_b(br, &mut audfrm_local, bsi, &hints)?;
        &audfrm_local
    } else {
        audfrm
    };

    // Reject cases the parser does not handle.
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
    let strmtyp_indep = matches!(bsi.strmtyp, StreamType::Independent);
    let _ = BLOCKS_PER_FRAME; // unused once cplinu_blk migrated to audfrm

    // §E.2.3.2 / §E.1.2.4 — per-frame syntax-state initialisation. The
    // audfrm finishes by setting `firstcplcos[ch] = 1` for every fbw
    // channel and `firstcplleak = 1`; both are stateful "have we seen
    // a coupling-coordinate / leak-init block yet this frame" flags
    // that gate whether the audblk reads the explicit `cplcoe[ch]` /
    // `cplleake` bit or substitutes an implicit `1`. They reset every
    // syncframe so we keep them as locals here rather than on `state`.
    let mut firstcplcos: [bool; MAX_FBW] = [true; MAX_FBW];
    let mut firstcplleak = true;

    // §7.2.2.6 — clear leftover coupling state at the top of every
    // frame so a previous frame's `cpl_in_use` doesn't leak forward
    // when this frame's blk 0 has `cplinu == 0`.
    state.cpl_in_use = false;
    for ch in 0..MAX_FBW {
        state.channels[ch].in_coupling = false;
    }

    // ---- §3.4 AHT pre-buffered coefficients ----
    //
    // When `chahtinu[ch] == 1`, the audblk that decodes the FIRST
    // non-`REUSE` exponent strategy for channel `ch` reads ALL 6×nmant
    // AHT mantissas + GAQ side info up front. Subsequent audblks for
    // that channel emit no mantissa bits — they pull their per-block
    // coefficient values from this `aht_coeffs[ch][blk][bin]` cache,
    // which holds the post-IDCT / post-`*2^-exp` floating coefficients.
    //
    // `aht_pending[ch] == true` for channels that have `chahtinu == 1`
    // AND haven't emitted their AHT mantissa block yet this frame.
    // `aht_filled[ch] == true` once the cache is populated.
    // 30 KB total (5 chs × 6 blks × 256 bins × 4 B). Heap-allocate so
    // we don't blow the audio thread's modest stack budget.
    let mut aht_coeffs: Vec<[[f32; N_COEFFS]; AHT_BLOCKS]> =
        vec![[[0.0; N_COEFFS]; AHT_BLOCKS]; MAX_FBW];
    let mut aht_pending: [bool; MAX_FBW] = [false; MAX_FBW];
    let mut aht_filled: [bool; MAX_FBW] = [false; MAX_FBW];
    if audfrm.ahte {
        aht_pending[..nfchans].copy_from_slice(&audfrm.chahtinu[..nfchans]);
    }

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

        // ---- coupling strategy block (Table E1.4 + §E.1.3.3.5) ----
        //
        // Per §E.1.2 / Table E1.3, `cplstre[blk]` + `cplinu[blk]` are
        // emitted in **audfrm** (already parsed; surfaced as
        // [`AudFrm::cplstre_blk`] / [`AudFrm::cplinu_blk`]). The audblk
        // only carries the **strategy details** (chincpl, cplbegf,
        // cplendf, cplbndstrc) when `cplstre[blk] && cplinu[blk]`,
        // and the **coordinate block** (cplcoe, mstrcplco, cplcoexp,
        // cplcomant, phsflg) whenever `cplinu[blk]`.
        let cplinu = audfrm.cplinu_blk[blk];
        let cplstre = audfrm.cplstre_blk[blk];
        if cplstre {
            if cplinu {
                // §E.1.3.3.6 ecplinu — enhanced coupling flag.
                let ecplinu = br.read_u32(1)? != 0;
                if ecplinu {
                    return Err(Error::unsupported(
                        "eac3 audblk: ecplinu == 1 (enhanced coupling) — \
                         round 5 standard coupling only; \
                         enhanced coupling per §E.1.3.3.7-26 deferred",
                    ));
                }
                // §E.1.3.3.7 chincpl[ch] — implicit 1 for both channels
                // in 2/0; explicit per-fbw-channel bit otherwise.
                if bsi.acmod == 0x2 {
                    state.channels[0].in_coupling = true;
                    state.channels[1].in_coupling = true;
                } else {
                    for ch in 0..nfchans {
                        let v = br.read_u32(1)? != 0;
                        state.channels[ch].in_coupling = v;
                    }
                }
                // §E.1.3.3.8 phsflginu — only in 2/0.
                state.phsflginu = if bsi.acmod == 0x2 {
                    br.read_u32(1)? != 0
                } else {
                    false
                };
                // §E.1.3.3.9 cplbegf (4 bits).
                state.cpl_begf = br.read_u32(4)? as u8;
                // §E.1.3.3.10 cplendf (4 bits when SPX is off; we
                // already required spxinu == 0 above).
                state.cpl_endf = br.read_u32(4)? as u8;
                // §5.4.3.12 spec envelope: the upper sub-band index is
                // `cplendf + 2`, so `ncplsubnd = 3 + cplendf - cplbegf
                // >= 1` is the actual validity test (equivalently
                // `cplbegf <= cplendf + 2`). The earlier "cplbegf >
                // cplendf" rejection mirrored the AC-3 round-7 bug —
                // FFmpeg's E-AC-3 encoder picks narrow configs like
                // `(cplbegf=11, cplendf=10)` for high-bandwidth
                // multichannel frames. Use signed arithmetic so the
                // `3 + cplendf - cplbegf` term can't underflow.
                let ncplsubnd_signed = 3i32 + state.cpl_endf as i32 - state.cpl_begf as i32;
                if ncplsubnd_signed < 1 {
                    return Err(Error::invalid(
                        "eac3 audblk: cplbegf > cplendf+2 — malformed coupling range",
                    ));
                }
                state.cpl_nsubbnd = ncplsubnd_signed as usize;
                // §E.1.3.3.11 cplbndstrce — gates the cplbndstrc[]
                // array. When 0, all subbands stay un-merged (i.e.
                // cplbndstrc[bnd] = 0 for every band).
                let cplbndstrce = br.read_u32(1)? != 0;
                state.cpl_bndstrc[0] = false;
                if cplbndstrce {
                    for bnd in 1..state.cpl_nsubbnd.min(18) {
                        let v = br.read_u32(1)? != 0;
                        state.cpl_bndstrc[bnd] = v;
                    }
                    // Any remaining (in case nsubbnd capped at 18) stay
                    // at the default 0.
                    for bnd in state.cpl_nsubbnd.min(18)..18 {
                        state.cpl_bndstrc[bnd] = false;
                    }
                } else {
                    for bnd in 1..18 {
                        state.cpl_bndstrc[bnd] = false;
                    }
                }
                // Mantissa-domain coupling range: bins [37+12·begf,
                // 37+12·(endf+3)) per §7.4.2.
                state.cpl_begf_mant = 37 + 12 * state.cpl_begf as usize;
                state.cpl_endf_mant = 37 + 12 * (state.cpl_endf as usize + 3);
                // Derive ncplbnd by merging sub-bands whose
                // cplbndstrc=1 (same algorithm as base AC-3).
                let mut n = state.cpl_nsubbnd;
                for bnd in 1..state.cpl_nsubbnd {
                    if state.cpl_bndstrc[bnd] {
                        n -= 1;
                    }
                }
                state.cpl_nbnd = n;
                state.cpl_in_use = true;
            } else {
                // !cplinu[blk] — clear all per-channel coupling flags
                // and reset the per-frame state-init markers per
                // Table E1.4 ("if !cplinu[blk] { firstcplcos[ch] = 1;
                // firstcplleak = 1; phsflginu = 0; ecplinu = 0; }").
                for ch in 0..nfchans {
                    state.channels[ch].in_coupling = false;
                    firstcplcos[ch] = true;
                }
                firstcplleak = true;
                state.phsflginu = false;
                state.cpl_in_use = false;
            }
        }
        // When !cplstre[blk], every persistent coupling-strategy field
        // (cpl_begf/cpl_endf/cpl_nsubbnd/cpl_nbnd/cpl_begf_mant/
        // cpl_endf_mant/cpl_in_use/in_coupling) keeps its prior value
        // from the last block where cplstre[blk] == 1 (that's the
        // whole point of `cplstre` — strategy reuse). Coordinates
        // (cplcoe, mstrcplco, cplcoexp, cplcomant) ARE re-emitted per
        // block via the cplcoe[ch] gate below.

        // ---- coupling coordinates (Table E1.4) ----
        let mut any_cplcoe_this_block = false;
        if cplinu {
            // ecplinu == 0 path (we already rejected ecplinu == 1).
            for ch in 0..nfchans {
                if state.channels[ch].in_coupling {
                    // §E.1.3.3.13 cplcoe[ch] — implicit 1 on the very
                    // first block this channel enters coupling per
                    // frame (firstcplcos[ch]); explicit 1-bit field
                    // otherwise.
                    let cplcoe = if firstcplcos[ch] {
                        firstcplcos[ch] = false;
                        true
                    } else {
                        br.read_u32(1)? != 0
                    };
                    if cplcoe {
                        any_cplcoe_this_block = true;
                        // §E.1.3.3.14 mstrcplco[ch] — 2 bits.
                        let mstrcplco = br.read_u32(2)? as i32;
                        for bnd in 0..state.cpl_nbnd {
                            // §E.1.3.3.15-16 cplcoexp + cplcomant.
                            let cplcoexp = br.read_u32(4)? as i32;
                            let cplcomant = br.read_u32(4)? as i32;
                            let mant = if cplcoexp == 15 {
                                cplcomant as f32 / 16.0
                            } else {
                                (cplcomant + 16) as f32 / 32.0
                            };
                            let shift = cplcoexp + 3 * mstrcplco;
                            state.cpl_coord[ch][bnd] = mant * 2f32.powi(-shift);
                        }
                        state.cpl_coord_valid[ch] = true;
                    }
                } else {
                    // Channel is not part of the coupling group; reset
                    // the firstcplcos marker so a later block that
                    // brings this channel back into coupling treats
                    // its first cplcoe as implicit 1.
                    firstcplcos[ch] = true;
                }
            }
            // §E.1.3.3.17 phsflg[bnd] — only in 2/0 + phsflginu + at
            // least one channel emitted coordinates this block (the
            // spec's `cplcoe[0] || cplcoe[1]` test, which is THIS
            // block's cplcoe — not a sticky any-block flag).
            if bsi.acmod == 0x2 && state.phsflginu && any_cplcoe_this_block {
                for bnd in 0..state.cpl_nbnd {
                    state.cpl_phsflg[bnd] = br.read_u32(1)? != 0;
                }
            }
        }

        // ---- §E.1.3.4 / §7.5 rematrixing — only for 2/0 (acmod==2) ----
        // Block 0 is special: encoder emits rematflg directly without
        // a rematstr gate; subsequent blocks emit rematstr first and
        // only if set do rematflg follow. AC-3 §5.4.3.19 has the same
        // shape for base AC-3.
        if bsi.acmod == 0x2 {
            let rematstr = if blk == 0 { true } else { br.read_u32(1)? != 0 };
            if rematstr {
                let n_remat = crate::audblk::remat_band_count(cplinu, state.cpl_begf);
                for rbnd in 0..n_remat {
                    let v = br.read_u32(1)? != 0;
                    state.rematflg[rbnd] = v;
                }
            }
        }

        // ---- §E.1.2.4 exponent strategy lookup ----
        //
        // §E.1.2.3 / Table E.1.3 emit `chexpstr[blk][ch]` (2 bits),
        // `cplexpstr[blk]` (2 bits when cplinu[blk]), and per-block
        // `lfeexpstr[blk]` (1 bit, when lfeon) IN audfrm — NOT in
        // audblk. The audblk only carries the bandwidth code +
        // exponent payload that those strategies gate. Round-29.5
        // moves the strategy reads back to where the spec puts them
        // (audfrm); audblk just looks them up.
        let cplexpstr = if cplinu {
            audfrm.cplexpstr_blk[blk]
        } else {
            0u8
        };
        let mut chexpstr = [0u8; MAX_FBW];
        chexpstr[..nfchans].copy_from_slice(&audfrm.chexpstr_blk_ch[blk][..nfchans]);
        let lfeexpstr = if lfeon { audfrm.lfeexpstr[blk] } else { 0u8 };

        // §E.1.3.4.5 chbwcod — only when chexpstr != REUSE AND the
        // channel is not in coupling AND not in spectral extension
        // (we already require !spxinu).
        let mut chbwcod = [0u8; MAX_FBW];
        for ch in 0..nfchans {
            if chexpstr[ch] != 0 && !state.channels[ch].in_coupling {
                chbwcod[ch] = br.read_u32(6)? as u8;
                if chbwcod[ch] > 60 {
                    return Err(Error::invalid(
                        "eac3 audblk: chbwcod > 60 (E.1.3.4.6 invalid)",
                    ));
                }
            }
        }

        // ---- coupling-channel exponents (§E.1.3.4.4) ----
        if cplinu && cplexpstr != 0 {
            let cplabsexp = br.read_u32(4)? as i32;
            let cpl_start = state.cpl_begf_mant;
            let cpl_end = state.cpl_endf_mant;
            let grpsize = match cplexpstr {
                1 => 1,
                2 => 2,
                3 => 4,
                _ => 1,
            };
            // Number of groups: (cpl_end - cpl_start) / (grpsize · 3).
            // cpl_end - cpl_start = 12 · (cplendf + 3 - cplbegf) =
            // 12 · ncplsubnd which is divisible by 3 for grpsize=1
            // and by 6/12 for grpsize=2/4 only when ncplsubnd is even
            // (D2/D4). FFmpeg encoders generally pick a strategy that
            // makes this divisible; if not we'd round down and miss
            // bins, but this is the spec's `(cpl_end - cpl_start) /
            // (grpsize × 3)` formula verbatim.
            let ncplgrps = (cpl_end - cpl_start) / (grpsize * 3);
            let mut raw_exp = vec![0i32; ncplgrps * 3];
            audblk::decode_exponents(
                br,
                cplabsexp << 1,
                ncplgrps,
                cplexpstr as usize,
                &mut raw_exp,
            )?;
            let cpl_ch = MAX_FBW;
            for (i, e) in raw_exp.iter().enumerate() {
                let idx = cpl_start + i * grpsize;
                for j in 0..grpsize {
                    if idx + j < N_COEFFS {
                        state.channels[cpl_ch].exp[idx + j] = (*e).clamp(0, 24) as u8;
                    }
                }
            }
        }

        // ---- fbw exponents ----
        for ch in 0..nfchans {
            if chexpstr[ch] != 0 {
                // Coupled channels stop at cpl_begf_mant; un-coupled
                // channels go up to 37 + 3·(chbwcod+12).
                let end = if state.channels[ch].in_coupling {
                    state.cpl_begf_mant
                } else {
                    37 + 3 * (chbwcod[ch] as usize + 12)
                };
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
                // §E.1.2.4 / Table E.1.4 — `gainrng[ch]` (2 bits)
                // immediately after the per-channel exponent payload.
                // We don't currently use it in the DSP path (the round-2
                // bit-allocation reuses base-AC-3 sgain logic that
                // doesn't consult gainrng), but the bit MUST be
                // consumed or every subsequent field slides. The
                // earlier "Annex E dropped gainrng" comment was wrong;
                // Table E.1.4 emits it for every fbw channel whose
                // strategy this block is non-REUSE.
                let _gainrng = br.read_u32(2)?;
            } else if blk == 0 {
                return Err(Error::invalid(
                    "eac3 audblk: chexpstr == 0 in block 0 (no prior exponents to reuse)",
                ));
            } else if state.channels[ch].in_coupling {
                // Reuse path: end_mant follows the coupled channel's
                // bandwidth, which is cpl_begf_mant. Re-set in case a
                // prior block had this channel un-coupled with a
                // different end_mant.
                state.channels[ch].end_mant = state.cpl_begf_mant;
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

        // ---- SNR offset (§E.1.3.5.2) ----
        // Round 5 supports snroffststr == 0 (single frame-level value
        // applied to every block). Block 0 of each frame initialises
        // every channel's csnroffst/fsnroffst from the audfrm frame
        // values; later blocks reuse them.
        if blk == 0 {
            state.snroffst_coarse = audfrm.frmcsnroffst;
            for ch in 0..nfchans {
                state.fsnroffst[ch] = audfrm.frmfsnroffst;
            }
            if lfeon {
                state.lfefsnroffst = audfrm.frmfsnroffst;
            }
            state.cpl_fsnroffst = audfrm.frmfsnroffst;
            // §E.2.2.4: "If bamode == 0 the encoder uses fast-gain
            // codeword 0x4 (mid)". Carry through for fgaincod when
            // frmfgaincode == 0 (no per-block fgaincod).
            for ch in 0..nfchans {
                state.fgaincod[ch] = 0x4;
            }
            if lfeon {
                state.lfefgaincod = 0x4;
            }
            state.cpl_fgaincod = 0x4;
        }

        // ---- §E.1.3.5.4 fgaincode (per-block fgain override) ----
        // Per Table E1.4, when `frmfgaincode == 1` a 1-bit `fgaincode`
        // field follows; if set, the per-channel fgaincod (3 bits each)
        // are emitted including the cpl-channel slot (only when
        // cplinu[blk]).
        if audfrm.frmfgaincode {
            let fgaincode = br.read_u32(1)? != 0;
            if fgaincode {
                if cplinu {
                    state.cpl_fgaincod = br.read_u32(3)? as u8;
                }
                for ch in 0..nfchans {
                    state.fgaincod[ch] = br.read_u32(3)? as u8;
                }
                if lfeon {
                    state.lfefgaincod = br.read_u32(3)? as u8;
                }
            }
        }

        // ---- §E.1.3.5.3 convsnroffste (always present for strmtyp == 0) ----
        // Optional 10-bit `convsnroffst` follows; we don't use it for
        // playback (it adjusts the SNR offset for downstream AC-3
        // converter modes), but we must consume the bit(s).
        if strmtyp_indep {
            let convsnroffste = br.read_u32(1)? != 0;
            if convsnroffste {
                let _convsnroffst = br.read_u32(10)?;
            }
        }

        // ---- §E.1.3.5.4 cplleake (only when cplinu[blk]) ----
        // First-block (per frame) emits `cplleake = 1` implicitly; later
        // blocks emit it explicitly. When set, `cplfleak` + `cplsleak`
        // (3 bits each) follow.
        if cplinu {
            let cplleake = if firstcplleak {
                firstcplleak = false;
                true
            } else {
                br.read_u32(1)? != 0
            };
            if cplleake {
                state.cpl_fleak = br.read_u32(3)? as u8;
                state.cpl_sleak = br.read_u32(3)? as u8;
            }
        }

        // ---- §E.1.3.5.5 dba ----
        if audfrm.dbaflde {
            let dbaie = br.read_u32(1)? != 0;
            if dbaie {
                let cpl_idx = MAX_FBW;
                let mut cpldeltbae = 0u32;
                if cplinu {
                    cpldeltbae = br.read_u32(2)?;
                }
                let mut deltbae = [0u32; MAX_FBW];
                for ch in 0..nfchans {
                    deltbae[ch] = br.read_u32(2)?;
                }
                if cplinu {
                    match cpldeltbae {
                        1 => {
                            let nseg = (br.read_u32(3)? + 1) as usize;
                            state.deltnseg[cpl_idx] = nseg.min(8);
                            for seg in 0..state.deltnseg[cpl_idx] {
                                state.deltoffst[cpl_idx][seg] = br.read_u32(5)? as u8;
                                state.deltlen[cpl_idx][seg] = br.read_u32(4)? as u8;
                                state.deltba[cpl_idx][seg] = br.read_u32(3)? as u8;
                            }
                        }
                        2 => {
                            state.deltnseg[cpl_idx] = 0;
                        }
                        _ => {}
                    }
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
        if cplinu {
            let start = state.cpl_begf_mant;
            let end = state.cpl_endf_mant;
            audblk::run_bit_allocation(
                state,
                MAX_FBW,
                start,
                end,
                si.fscod,
                state.cpl_fsnroffst,
                state.cpl_fgaincod,
                true,
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
        //
        // Standard path: walk every (ch, bin) pair reading bap-coded
        // mantissas (`audblk::unpack_mantissas`).
        //
        // AHT path: when chahtinu[ch] == 1, the FIRST audblk that
        // would emit channel exponents (i.e. the block where chexpstr
        // != REUSE — for AHT-eligible streams that's always block 0)
        // reads instead chgaqmod + chgaqgain + 6×nmant AHT mantissas,
        // applies the §3.4.5 IDCT-II to recover per-block transform
        // coefficients, and caches them in `aht_coeffs[ch][blk][bin]`.
        // Subsequent blocks for AHT channels skip the mantissa read
        // and load coefficients from the cache.
        if audfrm.ahte && (aht_pending.iter().any(|&p| p) || aht_filled.iter().any(|&p| p)) {
            // AHT in use for at least one channel in this frame.
            // unpack_mixed_mantissas walks per-channel: AHT-active
            // channels skip bit reads on blocks 1..5 (their mantissas
            // were front-loaded in block 0); standard channels are
            // walked as usual via the per-channel scalar fallback.
            unpack_mixed_mantissas(
                state,
                &ac3_bsi,
                br,
                &mut aht_coeffs,
                &mut aht_pending,
                &mut aht_filled,
                nfchans,
            )?;
        } else {
            audblk::unpack_mantissas(state, &ac3_bsi, br)?;
        }
        // For AHT-active channels, overwrite coeffs[bin] with the
        // pre-cached value for THIS block index.
        for ch in 0..nfchans {
            if aht_filled[ch] {
                let end = state.channels[ch].end_mant;
                for bin in 0..end {
                    state.channels[ch].coeffs[bin] = aht_coeffs[ch][blk][bin];
                }
                // Clear bins past end_mant so stale data can't leak.
                for bin in end..N_COEFFS {
                    state.channels[ch].coeffs[bin] = 0.0;
                }
            }
        }

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

/// AHT-aware mantissa unpacker.
///
/// Mirrors [`audblk::unpack_mantissas`] but routes per-channel reads
/// through the AHT path when `aht_pending[ch] == true`. For AHT-active
/// channels we read 6×nmant mantissas + GAQ side info, dequantise via
/// VQ (Tables E4.1..E4.7) or scalar/GAQ (Table E3.5), apply the
/// §3.4.5 inverse DCT-II to recover per-block coefficients, multiply
/// by `2^-exp`, and cache the per-block coefficients in
/// `aht_coeffs[ch][blk][bin]` for the per-block dispatch loop above.
///
/// Coupling/LFE AHT (`cplahtinu`/`lfeahtinu`) is **not** handled here —
/// the round-6 dispatch only sets `aht_pending[ch]` for fbw channels.
fn unpack_mixed_mantissas(
    state: &mut Ac3State,
    _bsi: &Ac3Bsi,
    br: &mut BitReader<'_>,
    aht_coeffs: &mut [[[f32; N_COEFFS]; AHT_BLOCKS]],
    aht_pending: &mut [bool; MAX_FBW],
    aht_filled: &mut [bool; MAX_FBW],
    nfchans: usize,
) -> Result<()> {
    // Clear per-block transform-coefficient state for every channel —
    // standard mantissas overwrite bins 0..end_mant, AHT mantissas
    // populate via the cache below; bins outside those ranges must
    // read as zero (matches the base AC-3 unpacker).
    for ch in 0..crate::audblk::MAX_CHANNELS {
        for v in state.channels[ch].coeffs.iter_mut() {
            *v = 0.0;
        }
    }

    // Standard channels (and the AHT-skip blocks for AHT channels)
    // pull from the bit stream; AHT channels on their FIRST appearance
    // pull the mantissa block and IDCT it. We walk channels in order
    // (matching the spec's `for ch in 0..nfchans` loop) so the bit
    // cursor advances in the same order whether AHT is in use or not.
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        if aht_filled[ch] {
            // AHT cache populated on a prior block — no bits to read
            // here. The per-block dispatch loop in
            // `decode_indep_audblks` will load coefficients from
            // `aht_coeffs[ch][blk]` after this function returns.
            continue;
        }
        if aht_pending[ch] {
            // First AHT-active block for this channel — read GAQ side
            // info + 6×nmant mantissas + IDCT into the coefficient
            // cache.
            decode_aht_channel_mantissas(state, ch, end, br, &mut aht_coeffs[ch])?;
            aht_filled[ch] = true;
            aht_pending[ch] = false;
            continue;
        }
        // Standard scalar mantissa path. Walks this channel inline
        // using the same fetch shape as `audblk::unpack_mantissas`
        // (single-channel grouping state — multichannel sharing of
        // bap-1/2/4 triplet/pair buffers is round-7 work).
        unpack_one_channel_scalar(state, ch, end, br)?;
    }
    Ok(())
}

/// Walk one channel's scalar mantissa path (standard, non-AHT). This
/// mirrors the per-channel inner loop of [`audblk::unpack_mantissas`]
/// without the cross-channel grouping state — for round-6 AHT the
/// only "mixed" use-case is mono (1 fbw channel + AHT in use), so
/// the bap-1/2/4 cross-channel triplet/pair sharing isn't reachable.
/// For multichannel AHT (round 7) the grouping state must be threaded
/// across channels.
fn unpack_one_channel_scalar(
    state: &mut Ac3State,
    ch: usize,
    end: usize,
    br: &mut BitReader<'_>,
) -> Result<()> {
    use crate::tables::QUANTIZATION_BITS;
    // Per-channel ephemeral grouping state; safe for the mono case.
    let mut grp1 = [0.0f32; 3];
    let mut grp1_n = 0usize;
    let mut grp2 = [0.0f32; 3];
    let mut grp2_n = 0usize;
    let mut grp4 = [0.0f32; 2];
    let mut grp4_n = 0usize;
    let dith = state.channels[ch].dithflag;
    for bin in 0..end {
        let bap = state.channels[ch].bap[bin];
        let v = match bap {
            0 => 0.0f32,
            1 => {
                if grp1_n == 0 {
                    let code = br.read_u32(5)? as i32;
                    let m1 = code / 9;
                    let m2 = (code % 9) / 3;
                    let m3 = code % 3;
                    let lvl = crate::tables::MANT_LEVEL_3;
                    grp1 = [
                        lvl[m1.clamp(0, 2) as usize],
                        lvl[m2.clamp(0, 2) as usize],
                        lvl[m3.clamp(0, 2) as usize],
                    ];
                    grp1_n = 3;
                }
                let r = grp1[3 - grp1_n];
                grp1_n -= 1;
                r
            }
            2 => {
                if grp2_n == 0 {
                    let code = br.read_u32(7)? as i32;
                    let m1 = code / 25;
                    let m2 = (code % 25) / 5;
                    let m3 = code % 5;
                    let lvl = crate::tables::MANT_LEVEL_5;
                    grp2 = [
                        lvl[m1.clamp(0, 4) as usize],
                        lvl[m2.clamp(0, 4) as usize],
                        lvl[m3.clamp(0, 4) as usize],
                    ];
                    grp2_n = 3;
                }
                let r = grp2[3 - grp2_n];
                grp2_n -= 1;
                r
            }
            3 => {
                let code = br.read_u32(3)? as usize;
                crate::tables::MANT_LEVEL_7[code.min(6)]
            }
            4 => {
                if grp4_n == 0 {
                    let code = br.read_u32(7)? as i32;
                    let m1 = code / 11;
                    let m2 = code % 11;
                    let lvl = crate::tables::MANT_LEVEL_11;
                    grp4 = [lvl[m1.clamp(0, 10) as usize], lvl[m2.clamp(0, 10) as usize]];
                    grp4_n = 2;
                }
                let r = grp4[2 - grp4_n];
                grp4_n -= 1;
                r
            }
            5 => {
                let code = br.read_u32(4)? as usize;
                crate::tables::MANT_LEVEL_15[code.min(14)]
            }
            b if (6..=15).contains(&b) => {
                let nbits = QUANTIZATION_BITS[b as usize] as u32;
                let raw = br.read_u32(nbits)? as i32;
                let shift = 32 - nbits;
                let signed = (raw << shift) >> shift;
                let scale = 2f32.powi(-(nbits as i32 - 1));
                signed as f32 * scale
            }
            _ => 0.0,
        };
        let final_v = if bap == 0 && dith { 0.0 } else { v };
        let e = state.channels[ch].exp[bin] as i32;
        state.channels[ch].coeffs[bin] = final_v * 2f32.powi(-e);
    }
    Ok(())
}

/// Decode the AHT mantissa block for one fbw channel and fill its
/// 6×N coefficient cache. Per §3.4 / §3.4.4 / §3.4.5 / §3.4.4.2.
///
/// 1. **hebap** — per-bin high-efficiency bap, derived in the §3.4.3.1
///    pseudo-code from psd/mask. Reuses the masking curve already
///    computed by [`audblk::run_bit_allocation`] (so we walk
///    `state.channels[ch].psd`/`mask` rather than re-derive them).
/// 2. **chgaqmod**: 2 bits.
/// 3. **chgaqbin[bin]**: derived from hebap (Table E3.3 logic).
/// 4. **chgaqgain[n]** for `n in 0..chgaqsections`: 1 or 5 bits each
///    depending on chgaqmod (mode 3 packs 3 gains in 5 bits).
/// 5. **mantissas**: per bin, per AHT-block (j in 0..6):
///    * `hebap == 0`           → mantissa = 0 (zero bin).
///    * `1 <= hebap <= 7`      → 6-element VQ codeword shared across all 6 j's.
///    * `hebap >= 8`           → scalar/GAQ per j (with optional gain word).
/// 6. **IDCT-II §3.4.5**       → reconstruct per-block C(k, m) from X(k, j).
/// 7. **`coeff = mant · 2^-exp`** stored in `cache[blk][bin]`.
fn decode_aht_channel_mantissas(
    state: &mut Ac3State,
    ch: usize,
    end: usize,
    br: &mut BitReader<'_>,
    cache: &mut [[f32; N_COEFFS]; AHT_BLOCKS],
) -> Result<()> {
    // ---- 1. derive hebap[] from psd / mask via §3.4.3.1 ----
    //
    // The masking curve `state.channels[ch].mask` is in the spec's
    // banded representation (50 entries indexed by masktab[bin]). We
    // reproduce the per-band post-processing inline (`mask_after_floor`)
    // so our hebap lookup matches the encoder's choice exactly.
    let mut hebap = vec![0u8; end.max(1)];
    {
        use crate::tables::{BNDSZ, BNDTAB, FLOORTAB, MASKTAB};
        let snroffset =
            (((state.snroffst_coarse as i32 - 15) << 4) + state.fsnroffst[ch] as i32) << 2;
        let floor = FLOORTAB[state.floorcod as usize];
        let mut i = 0usize;
        let mut j = MASKTAB[i] as usize;
        while i < end {
            let lastbin = (BNDTAB[j] as usize + BNDSZ[j] as usize).min(end);
            let mut m = state.channels[ch].mask[j] as i32;
            m -= snroffset;
            m -= floor;
            if m < 0 {
                m = 0;
            }
            m &= 0x1fe0;
            m += floor;
            while i < lastbin {
                hebap[i] = aht::hebap_from_address(state.channels[ch].psd[i], m);
                i += 1;
            }
            j += 1;
        }
    }

    // ---- 2. chgaqmod (2 bits) ----
    let chgaqmod = br.read_u32(2)? as u8;

    // ---- 3. compute chgaqbin[bin] (Table E3.3 logic) ----
    let mut gaqbin = vec![0i8; end.max(1)];
    let active = aht::fill_gaqbin(&hebap, chgaqmod, &mut gaqbin);

    // ---- 4. read chgaqgain[n] for nsections sections ----
    let nsections = aht::gaq_sections(chgaqmod, active);
    let mut gain_words = vec![0u8; active];
    aht::read_gaq_gains(br, chgaqmod, nsections, &mut gain_words)?;

    // ---- 5/6/7. per-bin mantissa decode + IDCT + scale ----
    let mut gain_iter = gain_words.into_iter();
    let mut x = [0.0f32; 6];
    for bin in 0..end {
        let h = hebap[bin];
        if h == 0 {
            // Zero-mantissa bin — coefficients are 0 across all blocks.
            for blk in 0..AHT_BLOCKS {
                cache[blk][bin] = 0.0;
            }
            continue;
        }
        if (1..=7).contains(&h) {
            // VQ regime — single codeword shared across the 6 AHT blocks.
            let nb = aht::VQ_BITS[h as usize] as u32;
            let idx = br.read_u32(nb)? as usize;
            x = aht::vq_lookup(h, idx);
        } else {
            // Scalar / GAQ regime.
            let gain_code = if gaqbin[bin] == 1 {
                gain_iter.next().unwrap_or(0)
            } else {
                0
            };
            aht::read_scalar_aht_mantissas(br, h, chgaqmod, gaqbin[bin], gain_code, &mut x)?;
        }
        // Inverse DCT-II to recover per-block C(k, m) (§3.4.5).
        let c = aht::idct_ii_6(x);
        // `coeff = mantissa · 2^(-exp)` for each block.
        let exp = state.channels[ch].exp[bin] as i32;
        let scale = 2f32.powi(-exp);
        for blk in 0..AHT_BLOCKS {
            cache[blk][bin] = c[blk] * scale;
        }
    }

    Ok(())
}

/// Decide whether the round-2 DSP path can handle this frame.
fn reject_unsupported(bsi: &Eac3Bsi, audfrm: &AudFrm) -> Result<()> {
    // expstre handling — both per-block (expstre==1) and frame-based
    // (expstre==0) strategies are supported. Round 72 (this commit)
    // landed the frame-based path: `audfrm::parse_with` expands the
    // 5-bit `frmcplexpstr` + per-channel `frmchexpstr[ch]` codewords
    // via Table E2.10 into `cplexpstr_blk[]` + `chexpstr_blk_ch[]` so
    // the dsp body sees the same per-block-per-channel shape it
    // already consumes for the expstre==1 case. Every FFmpeg-produced
    // E-AC-3 fixture in the corpus picks expstre==0.
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
    // ahte is now handled by the round-6 phase-B path (mono-only).
    // Defensive: reject any case where phase B was supposed to run
    // but didn't get a chance (caller forgot to call parse_phase_b).
    if audfrm.aht_phase_b_pending {
        return Err(Error::invalid(
            "eac3 dsp: audfrm phase-B AHT bits not consumed — caller must \
             invoke audfrm::parse_phase_b before decode_indep_audblks",
        ));
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
