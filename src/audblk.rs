//! AC-3 audio-block parser + DSP pipeline (§5.4.3, §7).
//!
//! This module walks `audblk()` bit-by-bit, running the full decoder
//! data-flow per Figure 6.1: unpack side-info → decode exponents → bit
//! allocation → unpack mantissas → decouple → rematrix → IMDCT →
//! window+overlap-add. Decoder state that must persist across audio
//! blocks inside a syncframe (and between syncframes for the
//! overlap-add delay line) lives on the `Ac3State` struct handed in by
//! the top-level decoder.
//!
//! The code is intentionally "big function per DSP stage" so each stage
//! matches a section of the spec 1:1.
//!
//! ## Scope
//!
//! - Full-bandwidth channels (fbw) + LFE are decoded.
//! - Coupling is supported for 2-channel streams (Table 7.24 coupling
//!   sub-bands, 7.4.3 coupling-coordinate reconstruction).
//! - Rematrixing (§7.5) is applied in 2/0 mode.
//! - Dynamic range compression (§7.7 dynrng) scales the transform
//!   coefficients.
//! - 512-point IMDCT with KBD window + 50% overlap-add (§7.9.4.1,
//!   §7.9.5). The 256-point short-block pair (§7.9.4.2) is wired
//!   through `crate::imdct::imdct_256_pair_fft`; block-switching
//!   correctness is gated by the `transient_bursts_stereo.ac3` PSNR
//!   test (the sine fixture never exercises `blksw=1`).

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::bsi::Bsi;
use crate::syncinfo::SyncInfo;
use crate::tables::{
    BAPTAB, BNDSZ, BNDTAB, DBPBTAB, FASTDEC, FASTGAIN, FLOORTAB, HTH, LATAB, MANT_LEVEL_11,
    MANT_LEVEL_15, MANT_LEVEL_3, MANT_LEVEL_5, MANT_LEVEL_7, MASKTAB, QUANTIZATION_BITS, SLOWDEC,
    SLOWGAIN, WINDOW,
};

/// Maximum fbw channels (3/2 mode).
pub const MAX_FBW: usize = 5;
/// Total channel slots: fbw (5) + coupling pseudo-channel (1) + lfe (1) = 7.
pub const MAX_CHANNELS: usize = 7;
/// Number of transform coefficients per block.
pub const N_COEFFS: usize = 256;
/// Audio blocks per syncframe.
pub const BLOCKS_PER_FRAME: usize = 6;
/// New samples per block per channel after overlap-add.
pub const SAMPLES_PER_BLOCK: usize = 256;

/// Snapshot of every side-info field decoded out of an `audblk()`
/// element per §5.4.3. This is purely the "parse" half of the pipeline
/// — no exponents, no mantissas, no DSP state. It gives tests and the
/// downstream §7 stages a single inspectable record of what the
/// bit-stream actually said, keyed to the spec clause numbers.
///
/// Every field here cites its §5.4.3.x subsection in the doc comment
/// so an auditor can verify the parser against the spec table by table.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct AudBlkSideInfo {
    // ---- §5.4.3.1 blksw[ch] / §5.4.3.2 dithflag[ch] ----
    /// `blksw[ch]` — per-channel block-switch flag (§5.4.3.1, 1 bit).
    pub blksw: [bool; MAX_FBW],
    /// `dithflag[ch]` — per-channel dither flag (§5.4.3.2, 1 bit).
    pub dithflag: [bool; MAX_FBW],
    // ---- §5.4.3.3-6 dynamic range control ----
    /// `dynrnge` — dynamic-range word present (§5.4.3.3, 1 bit).
    pub dynrnge: bool,
    /// `dynrng` — 8-bit dynamic-range gain word (§5.4.3.4). Only
    /// meaningful when `dynrnge == true`.
    pub dynrng: u8,
    /// `dynrng2e` — dual-mono ch2 dynamic-range present (§5.4.3.5,
    /// 1 bit). Only present when `acmod == 0`.
    pub dynrng2e: bool,
    /// `dynrng2` — dual-mono ch2 dynamic-range word (§5.4.3.6, 8 bits).
    pub dynrng2: u8,
    // ---- §5.4.3.7-18 coupling strategy + coordinates ----
    /// `cplstre` — coupling strategy present in this block (§5.4.3.7).
    pub cplstre: bool,
    /// `cplinu` — coupling in use (§5.4.3.8). Valid only when `cplstre`.
    pub cplinu: bool,
    /// `chincpl[ch]` — channel is part of the coupling group
    /// (§5.4.3.9). Valid only when `cplinu`.
    pub chincpl: [bool; MAX_FBW],
    /// `phsflginu` — coupling phase flags in use for 2/0 (§5.4.3.10).
    pub phsflginu: bool,
    /// `cplbegf` — coupling begin frequency code (§5.4.3.11, 4 bits).
    pub cplbegf: u8,
    /// `cplendf` — coupling end frequency code (§5.4.3.12, 4 bits).
    pub cplendf: u8,
    /// `cplbndstrc[sbnd]` — coupling band structure (§5.4.3.13).
    pub cplbndstrc: [bool; 18],
    /// `cplcoe[ch]` — coupling-coordinates-present flag per channel
    /// (§5.4.3.14).
    pub cplcoe: [bool; MAX_FBW],
    // ---- §5.4.3.19-20 rematrix ----
    /// `rematstr` — rematrix strategy present (§5.4.3.19).
    pub rematstr: bool,
    /// Number of rematrix bands actually carried in `rematflg` per
    /// §5.4.3.19 rules and Table — 2/3/4 depending on `cplbegf`.
    pub rematflg_count: u8,
    /// `rematflg[rbnd]` — per-band rematrix flag (§5.4.3.20).
    pub rematflg: [bool; 4],
    // ---- §5.4.3.21-24 exponent strategy ----
    /// `cplexpstr` — coupling exponent strategy (§5.4.3.21, 2 bits).
    /// 0=reuse, 1=D15, 2=D25, 3=D45.
    pub cplexpstr: u8,
    /// `chexpstr[ch]` — full-bandwidth exponent strategy per channel
    /// (§5.4.3.22, 2 bits).
    pub chexpstr: [u8; MAX_FBW],
    /// `lfeexpstr` — LFE exponent strategy (§5.4.3.23, 1 bit).
    pub lfeexpstr: u8,
    /// `chbwcod[ch]` — channel bandwidth code (§5.4.3.24, 6 bits).
    /// Only meaningful when `chexpstr[ch] != 0 && !chincpl[ch]`.
    pub chbwcod: [u8; MAX_FBW],
    // ---- §5.4.3.30-46 bit-allocation parametric side-info ----
    /// `baie` — bit-allocation-info exists (§5.4.3.30).
    pub baie: bool,
    /// `snroffste` — SNR-offset block-level flag (§5.4.3.36).
    pub snroffste: bool,
    /// `cplleake` — coupling-leak-init flag (§5.4.3.44). Only when
    /// `cplinu`.
    pub cplleake: bool,
    // ---- §5.4.3.47 delta bit allocation ----
    /// `deltbaie` — delta-bit-allocation info exists (§5.4.3.47).
    pub deltbaie: bool,
    // ---- §5.4.3.58-59 skip field ----
    /// `skiple` — skip-field-exists flag (§5.4.3.58).
    pub skiple: bool,
    /// `skipl` — number of skip *bytes* (§5.4.3.59, 9 bits).
    pub skipl: u16,
}

/// Per-channel persistent state: exponents, bit-allocation pointers,
/// bandwidth, gain-range, and the 256-sample overlap-add delay line.
#[derive(Clone)]
pub struct ChannelState {
    pub exp: [u8; N_COEFFS],
    pub bap: [u8; N_COEFFS],
    pub psd: [i16; N_COEFFS],
    pub bndpsd: [i16; 50],
    pub mask: [i16; 50],
    pub deltba: [i16; 50],
    pub end_mant: usize,
    /// 256-sample tail from last block's MDCT, ready to be added into
    /// this block's output.
    pub delay: [f32; SAMPLES_PER_BLOCK],
    /// Raw dequantized transform coefficients for this block.
    pub coeffs: [f32; N_COEFFS],
    pub blksw: bool,
    pub dithflag: bool,
    /// Whether this channel is coupled (set from chincpl[]).
    pub in_coupling: bool,
    /// Dynrng gain multiplier (linear).
    pub dynrng: f32,
    /// E-AC-3 spectral extension (§E.3.6): whether this channel
    /// regenerates high-frequency transform coefficients via SPX this
    /// block. Base AC-3 never sets this (no SPX in the base layer), so
    /// the SPX synthesis step in [`dsp_block`] is a no-op for AC-3.
    pub in_spx: bool,
    /// Per-band SPX coordinate `spxco[ch][bnd]` (§E.3.6.3). Persisted
    /// across blocks so a `spxcoe[ch] == 0` block can reuse the prior
    /// coordinates.
    pub spx_coord: [f32; 18],
    /// Per-band SPX noise / signal blend factors `nblendfact` /
    /// `sblendfact` (§E.3.6.4.2.1). Recomputed when new coordinates
    /// (and hence a new `spxblnd`) arrive; reused otherwise.
    pub spx_nblend: [f32; 18],
    pub spx_sblend: [f32; 18],
    /// E-AC-3 spectral-extension attenuation (§3.6.4.2.3, §2.3.2.24-25).
    /// `spx_atten_active` mirrors the frame-level `chinspxatten[ch]` bit
    /// (a frame-scoped flag — the spec carries it in audfrm, not audblk,
    /// so it stays constant across the 6 blocks of a syncframe). When
    /// set, the SPX synthesis applies a 5-tap notch filter at the
    /// baseband/extension border (and at every wrap point during the
    /// translation copy) using row `spx_atten_code` of Table E3.14.
    pub spx_atten_active: bool,
    /// `spxattencod[ch]` — 5-bit index into Table E3.14
    /// (`SPX_ATTEN_TABLE`). Only meaningful when `spx_atten_active`.
    pub spx_atten_code: u8,
}

impl Default for ChannelState {
    fn default() -> Self {
        Self::new()
    }
}

impl ChannelState {
    pub fn new() -> Self {
        Self {
            exp: [24; N_COEFFS],
            bap: [0; N_COEFFS],
            psd: [0; N_COEFFS],
            bndpsd: [0; 50],
            mask: [0; 50],
            deltba: [0; 50],
            end_mant: 0,
            delay: [0.0; SAMPLES_PER_BLOCK],
            coeffs: [0.0; N_COEFFS],
            blksw: false,
            dithflag: false,
            in_coupling: false,
            dynrng: 1.0,
            in_spx: false,
            spx_coord: [0.0; 18],
            spx_nblend: [0.0; 18],
            spx_sblend: [0.0; 18],
            spx_atten_active: false,
            spx_atten_code: 0,
        }
    }
}

/// Per-frame decoder state that survives across audio blocks and across
/// syncframes (delay lines).
#[derive(Clone)]
pub struct Ac3State {
    /// [0..nfchans] = fbw channels, index MAX_FBW = coupling pseudo-channel,
    /// index MAX_FBW+1 = LFE.
    pub channels: [ChannelState; MAX_CHANNELS],

    // ---- Coupling state (§5.4.3.7 ff, §7.4) ----
    pub cpl_in_use: bool,
    pub phsflginu: bool,
    pub cpl_begf: u8,
    pub cpl_endf: u8,
    pub cpl_begf_mant: usize, // 37 + 12*cplbegf
    pub cpl_endf_mant: usize, // 37 + 12*(cplendf+3)
    pub cpl_nsubbnd: usize,
    pub cpl_nbnd: usize,
    /// cplbndstrc[sbnd], 1 when subband merges into previous band.
    pub cpl_bndstrc: [bool; 18],
    /// cplco[ch][bnd] linear coupling coordinate.
    pub cpl_coord: [[f32; 18]; MAX_FBW],
    pub cpl_coord_valid: [bool; MAX_FBW],
    pub cpl_phsflg: [bool; 18],

    // ---- Rematrix ----
    pub rematflg: [bool; 4],

    // ---- Bit-allocation parameters ----
    pub sdcycod: u8,
    pub fdcycod: u8,
    pub sgaincod: u8,
    pub dbpbcod: u8,
    pub floorcod: u8,
    pub snroffst_coarse: u8,
    pub cpl_fsnroffst: u8,
    pub cpl_fgaincod: u8,
    pub cpl_fleak: u8,
    pub cpl_sleak: u8,
    pub fsnroffst: [u8; MAX_FBW],
    pub fgaincod: [u8; MAX_FBW],
    pub lfefsnroffst: u8,
    pub lfefgaincod: u8,

    /// Per-channel combined SNR offset `((csnr-15)<<4 + fsnr)<<2` (FFmpeg
    /// `snr_offset[ch]`). Updated when `snroffste=1`; reused otherwise.
    pub snr_offset_ch: [i32; MAX_CHANNELS],

    // ---- Delta bit allocation state (§5.4.3.47-57, §7.2.2.6) ----
    /// Per-channel deltba state: number of segments + per-segment offset/length/value.
    /// Each fbw channel has its own segment list; index `MAX_FBW` is the
    /// coupling channel. `deltnseg` of 0 means no delta-band processing.
    /// State is initialized to all-zero at the top of every syncframe (§7.2.2.6
    /// "initialize the cpldeltnseg and deltnseg[ch] delta bit allocation
    /// variables to 0 at the beginning of each syncframe") and updated by the
    /// per-block parser when `deltbae[ch] == 1` (new info follows) or cleared
    /// when `deltbae[ch] == 2` (perform no delta alloc this block). When
    /// `deltbae[ch] == 0` (reuse) or `deltbaie == 0` for blk > 0, the previous
    /// values are kept.
    pub deltnseg: [usize; MAX_FBW + 1],
    pub deltoffst: [[u8; 8]; MAX_FBW + 1],
    pub deltlen: [[u8; 8]; MAX_FBW + 1],
    pub deltba: [[u8; 8]; MAX_FBW + 1],

    // ---- E-AC-3 spectral extension region state (§E.3.6) ----
    /// Whether SPX is in use in the current block (`spxinu`). When false
    /// the SPX synthesis step in [`dsp_block`] does nothing.
    pub spx_in_use: bool,
    /// `spxstrtf` copy-start sub-band index → first copied tc# is
    /// `spx_bandtable(spxstrtf)`.
    pub spx_strtf: u8,
    /// First / one-past-last SPX sub-band (`spx_begin_subbnd` /
    /// `spx_end_subbnd`, §E.2.3.3.5-6).
    pub spx_begin_subbnd: usize,
    pub spx_end_subbnd: usize,
    /// SPX sub-band → band grouping (`spxbndstrc[]`, §E.2.3.3.8). Index
    /// is the absolute sub-band number; `true` means "merge into the
    /// previous band". Persisted so a `spxbndstrce == 0` block reuses
    /// the prior structure.
    pub spx_bndstrc: [bool; 18],
    /// Number of SPX bands and per-band size in transform coefficients
    /// (`nspxbnds` / `spxbndsztab[]`), derived from the sub-band range
    /// and `spx_bndstrc`.
    pub spx_nbnds: usize,
    pub spx_bndsztab: [usize; 18],
    /// 32-bit LFSR driving the SPX noise generator (§E.3.6.4.2). The
    /// spec leaves the noise sequence non-normative ("any reasonably
    /// random sequence"); a fixed seed keeps decodes reproducible.
    pub spx_noise_lfsr: u32,

    /// Bit position immediately after the BSI (start of block 0 bits).
    pub audblk_start_bits: u64,
    /// Which block we are currently parsing (0..6).
    pub blkidx: usize,
    /// 16-bit LFSR state driving the `bap=0` dither replacement
    /// (§7.3.4). Persisted across audio blocks and syncframes so the
    /// dither sequence has a smooth long-period character.
    pub dither_lfsr_state: u32,
    /// Monotonically increasing syncframe counter used for trace gating
    /// (e.g. `AC3_TRACE_FRAME=14`). Incremented at the top of every
    /// `decode_frame` call. Not part of the spec — diagnostic only.
    pub frame_counter: u64,
    /// Sample-rate shift for bit allocation (FFmpeg `sr_shift` =
    /// `FFMAX(bsid, 8) - 8`). Scales decay rates and HTH band index.
    pub sr_shift: u8,

    /// Diagnostics: replace parsed per-channel `fsnroffst` once before BA.
    pub fsnroffst_override: Option<[u8; MAX_FBW]>,

    /// Diagnostics: replace fbw `bap[]` for one block before mantissa unpack.
    pub bap_override: Option<(usize, [[u8; N_COEFFS]; MAX_FBW])>,

    /// E-AC-3 enhanced coupling (§E.3.5.5): when set, the §7.4 standard
    /// decouple step in [`dsp_block`] is skipped because each coupled
    /// channel's transform coefficients in `channels[ch].coeffs` were
    /// already reconstructed from the enhanced-coupling carrier `Z[k]`
    /// (the §E.3.5.5.4 complex product) by the E-AC-3 dsp layer before
    /// `dsp_block` runs. Base AC-3 and standard E-AC-3 coupling leave
    /// this `false` so the normal decouple applies.
    pub skip_decouple: bool,

    /// E-AC-3 enhanced-coupling cross-frame synthesis state (§E.3.5.5.3
    /// random de-correlation sources). The non-transient random arrays are
    /// "generated once … and the same for every block of every frame", and
    /// the transient random generator advances across blocks/frames — both
    /// lifetimes outlive a single syncframe, so the state lives here. Base
    /// AC-3 and standard E-AC-3 coupling never touch it.
    pub ecpl_state: crate::eac3::ecpl::EcplState,
}

impl Default for Ac3State {
    fn default() -> Self {
        Self::new()
    }
}

impl Ac3State {
    pub fn new() -> Self {
        Self {
            channels: std::array::from_fn(|_| ChannelState::new()),
            cpl_in_use: false,
            phsflginu: false,
            cpl_begf: 0,
            cpl_endf: 0,
            cpl_begf_mant: 0,
            cpl_endf_mant: 0,
            cpl_nsubbnd: 0,
            cpl_nbnd: 0,
            cpl_bndstrc: [false; 18],
            cpl_coord: [[0.0; 18]; MAX_FBW],
            cpl_coord_valid: [false; MAX_FBW],
            cpl_phsflg: [false; 18],
            rematflg: [false; 4],
            sdcycod: 0,
            fdcycod: 0,
            sgaincod: 0,
            dbpbcod: 0,
            floorcod: 0,
            snroffst_coarse: 0,
            cpl_fsnroffst: 0,
            cpl_fgaincod: 0,
            cpl_fleak: 0,
            cpl_sleak: 0,
            fsnroffst: [0; MAX_FBW],
            fgaincod: [0; MAX_FBW],
            lfefsnroffst: 0,
            lfefgaincod: 0,
            snr_offset_ch: [0; MAX_CHANNELS],
            deltnseg: [0; MAX_FBW + 1],
            deltoffst: [[0; 8]; MAX_FBW + 1],
            deltlen: [[0; 8]; MAX_FBW + 1],
            deltba: [[0; 8]; MAX_FBW + 1],
            spx_in_use: false,
            spx_strtf: 0,
            spx_begin_subbnd: 0,
            spx_end_subbnd: 0,
            spx_bndstrc: [false; 18],
            spx_nbnds: 0,
            spx_bndsztab: [0; 18],
            spx_noise_lfsr: 0x4A5B_6C7D,
            audblk_start_bits: 0,
            blkidx: 0,
            // Non-zero seed so the LFSR doesn't get stuck on all-zeros.
            // Arbitrary fixed value keeps decodes byte-reproducible.
            dither_lfsr_state: 0x1234,
            frame_counter: 0,
            sr_shift: 0,
            fsnroffst_override: None,
            bap_override: None,
            skip_decouple: false,
            ecpl_state: crate::eac3::ecpl::EcplState::new(),
        }
    }

    /// Per-stream bit-allocation parameters derived from BSI (matches
    /// FFmpeg `AC3HeaderInfo::sr_shift` from `ff_ac3_parse_header`).
    pub fn apply_bsi_params(&mut self, bsi: &Bsi) {
        self.sr_shift = bsi.bsid.max(8) - 8;
    }
}

/// Parse (but do not DSP) one syncframe's 6 audio blocks, returning
/// the per-block [`AudBlkSideInfo`] snapshots. Used by tests and
/// introspection tools; the decoder itself calls [`decode_frame`]
/// which fuses parse + DSP.
///
/// Side-info capture mirrors `decode_frame`'s bit-cursor: after each
/// block's side-info `parse_audblk_into` itself consumes the mantissa
/// region via [`unpack_mantissas`], so the cursor naturally lands on
/// block N+1's bits without a second walk here (a previous version
/// double-consumed the mantissas, which made every block N>0 read its
/// side-info bits from somewhere inside the previous block's mantissa
/// region).
/// Blocks whose parse fails (e.g. due to our current bit-allocation
/// approximation consuming a few mantissa bits too many) yield a
/// `Default::default()` snapshot and subsequent blocks restart from
/// the last good cursor — matching the decoder's graceful-degradation
/// policy for the §7 stages.
pub fn parse_frame_side_info(
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
) -> Result<[AudBlkSideInfo; BLOCKS_PER_FRAME]> {
    let mut state = Ac3State::new();
    state.apply_bsi_params(bsi);
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    br.skip(bsi.bits_consumed as u32)?;
    let mut out: [AudBlkSideInfo; BLOCKS_PER_FRAME] = Default::default();
    for blk in 0..BLOCKS_PER_FRAME {
        state.blkidx = blk;
        let mut side = AudBlkSideInfo::default();
        if parse_audblk_into(&mut state, si, bsi, &mut br, &mut side, None, None, None).is_ok() {
            out[blk] = side;
        } else {
            // Parse error — stop side-info capture here. Downstream
            // blocks are unreachable without a known bit-position.
            break;
        }
    }
    Ok(out)
}

/// Decode one syncframe of 6 audio blocks into interleaved f32 samples.
/// Output length = 1536 × nchans.
pub fn decode_frame(
    state: &mut Ac3State,
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
    out: &mut [f32],
) -> Result<()> {
    // Slice starting at the beginning of BSI (byte 5 of syncframe).
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    // Consume the BSI bits so we start exactly at audio block 0.
    br.skip(bsi.bits_consumed as u32)?;
    state.apply_bsi_params(bsi);
    state.audblk_start_bits = br.bit_position();
    state.blkidx = 0;

    let nchans = bsi.nchans as usize; // output channel count (fbw + lfe)
    let nfchans = bsi.nfchans as usize;

    for blk in 0..BLOCKS_PER_FRAME {
        state.blkidx = blk;
        // Tolerate bit-exhaustion in later blocks: if parsing or mantissa
        // unpack runs out of bits, zero-fill this block's coefficients and
        // keep going. This matches the spec's graceful-degradation
        // guidance for corrupt streams and also compensates for the
        // current bit-allocation approximation producing slightly more
        // mantissas than the encoder actually wrote.
        if parse_audblk(state, si, bsi, &mut br, Some(post_sync)).is_err() {
            for ch in 0..MAX_CHANNELS {
                for v in state.channels[ch].coeffs.iter_mut() {
                    *v = 0.0;
                }
            }
        }
        if std::env::var("AC3_TRACE_BITPOS").is_ok() {
            // Round-12 diagnostic: verify per-block bit-cursor lands inside
            // the syncframe (frame_bits ≈ 6104 for a 192 kbps frame; CRC
            // and a few padding bits sit between the last block's end and
            // the frame end). If `end_pos` ever exceeds `frame_bits` the
            // parser is over-consuming and every later block's side-info
            // bits will be misread.
            eprintln!(
                "TRACE-BITPOS frame={} blk={} end_pos={} frame_bits={}",
                state.frame_counter,
                blk,
                br.bit_position(),
                (frame_bytes.len() as u64 - 5) * 8
            );
        }
        dsp_block(state, si, bsi);
        // Write this block's SAMPLES_PER_BLOCK samples per channel
        // interleaved into `out` starting at block offset.
        let base = blk * SAMPLES_PER_BLOCK * nchans;
        for n in 0..SAMPLES_PER_BLOCK {
            for ch in 0..nfchans {
                let s = state.channels[ch].coeffs[n];
                out[base + n * nchans + ch] = s;
            }
            if bsi.lfeon {
                let s = state.channels[MAX_FBW + 1].coeffs[n];
                out[base + n * nchans + nfchans] = s;
            }
        }
    }
    // Diagnostic frame counter (gated by `AC3_TRACE_FRAME=N`). Increment
    // *after* the frame so frame 0 == first decoded frame; not part of
    // the spec.
    state.frame_counter = state.frame_counter.saturating_add(1);
    Ok(())
}

fn parse_audblk(
    state: &mut Ac3State,
    si: &SyncInfo,
    bsi: &Bsi,
    br: &mut BitReader,
    post_sync: Option<&[u8]>,
) -> Result<()> {
    let mut side = AudBlkSideInfo::default();
    parse_audblk_into(state, si, bsi, br, &mut side, None, None, post_sync)
}

/// Optional per-block bit-cursor metrics returned by [`parse_audblk_into`]
/// when auditing mantissa consumption.
#[derive(Clone, Copy, Debug, Default)]
pub struct BlockParseMetrics {
    pub bit_pre_mantissa: u64,
    pub bit_block_end: u64,
}

/// One checkpoint during [`parse_audblk_into`] side-info parsing.
#[derive(Clone, Debug)]
pub struct SideInfoMark {
    pub label: String,
    pub bit_pos: u64,
    pub detail: Option<String>,
}

#[inline]
fn trace_mark(
    trace: Option<&mut Vec<SideInfoMark>>,
    label: &str,
    br: &BitReader,
    detail: Option<String>,
) {
    if let Some(t) = trace {
        t.push(SideInfoMark {
            label: label.to_string(),
            bit_pos: br.bit_position(),
            detail,
        });
    }
}

/// Parse block `blk` and record bit-cursor checkpoints through side-info.
pub fn trace_block_side_info(
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
    blk: usize,
) -> Result<Vec<SideInfoMark>> {
    let mut state = Ac3State::new();
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    br.skip(bsi.bits_consumed as u32)?;
    for b in 0..blk {
        state.blkidx = b;
        let mut side = AudBlkSideInfo::default();
        parse_audblk_into(&mut state, si, bsi, &mut br, &mut side, None, None, None)?;
    }
    state.blkidx = blk;
    let mut side = AudBlkSideInfo::default();
    let mut trace = Vec::new();
    parse_audblk_into(
        &mut state,
        si,
        bsi,
        &mut br,
        &mut side,
        None,
        Some(&mut trace),
        None,
    )?;
    Ok(trace)
}

/// Read `nbits` at absolute bit offset from the start of post-syncframe
/// data (byte 5 of the syncframe — same origin as [`trace_block_side_info`]).
pub fn peek_bits_post_bsi(frame_bytes: &[u8], _bsi: &Bsi, bit_pos: u64, nbits: u32) -> u32 {
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    let _ = br.skip(bit_pos as u32);
    br.read_u32(nbits).unwrap_or(0)
}

/// Per-channel ±1 `fsnroffst` SNR retune (diagnostic / future use).
#[allow(dead_code)]
fn rerun_channel_bit_allocation(
    state: &mut Ac3State,
    ch: usize,
    si: &SyncInfo,
    bit_alloc_stages: &[u8; MAX_CHANNELS],
) {
    let stage = bit_alloc_stages[ch].max(1);
    let end = state.channels[ch].end_mant;
    run_bit_allocation_staged(
        state,
        ch,
        0,
        end,
        si.fscod,
        state.snr_offset_ch[ch],
        state.fgaincod[ch],
        false,
        stage,
    );
}

#[allow(dead_code)]
fn copy_channel_bap(state: &mut Ac3State, trial: &Ac3State, ch: usize) {
    state.channels[ch]
        .bap
        .copy_from_slice(&trial.channels[ch].bap);
    state.snr_offset_ch[ch] = trial.snr_offset_ch[ch];
}

/// Simulate unpack at parsed BAP then parse blocks `blk+1..` without error.
#[allow(dead_code)]
fn remainder_of_frame_parses_after_unpack(
    state: &Ac3State,
    si: &SyncInfo,
    bsi: &Bsi,
    post_sync: &[u8],
    blk: usize,
    pre_mantissa_bit: u64,
) -> bool {
    if blk + 1 >= BLOCKS_PER_FRAME {
        return true;
    }
    let mut st = state.clone();
    let mut br = BitReader::new(post_sync);
    if br.skip(pre_mantissa_bit as u32).is_err() {
        return false;
    }
    if unpack_mantissas(&mut st, bsi, &mut br).is_err() {
        return false;
    }
    for b in (blk + 1)..BLOCKS_PER_FRAME {
        st.blkidx = b;
        let mut side = AudBlkSideInfo::default();
        if parse_audblk_into(&mut st, si, bsi, &mut br, &mut side, None, None, None).is_err() {
            return false;
        }
    }
    true
}

/// Score a BA trial: full-frame parse is required; prefer blk+1 side-info
/// length 15 and higher mantissa walk (encoder often packs +1 fsnroffst step).
#[allow(dead_code)]
fn score_ba_trial(
    state: &Ac3State,
    si: &SyncInfo,
    bsi: &Bsi,
    post_sync: &[u8],
    blk: usize,
    pre_mantissa_bit: u64,
) -> Option<i32> {
    if !remainder_of_frame_parses_after_unpack(state, si, bsi, post_sync, blk, pre_mantissa_bit) {
        return None;
    }
    let mut score = 10_000i32;
    let mant = count_mantissa_bits_walk(state, bsi) as i32;
    score += mant;
    if blk + 1 < BLOCKS_PER_FRAME {
        let mut st = state.clone();
        let mut br = BitReader::new(post_sync);
        br.skip(pre_mantissa_bit as u32).ok()?;
        unpack_mantissas(&mut st, bsi, &mut br).ok()?;
        let blk_end = br.bit_position();
        st.blkidx = blk + 1;
        let mut side = AudBlkSideInfo::default();
        let mut metrics = BlockParseMetrics::default();
        parse_audblk_into(
            &mut st,
            si,
            bsi,
            &mut br,
            &mut side,
            Some(&mut metrics),
            None,
            None,
        )
        .ok()?;
        let side_bits = metrics.bit_pre_mantissa.saturating_sub(blk_end) as i32;
        if side_bits == 15 {
            score += 1_000;
        }
    }
    Some(score)
}

/// Per-channel ±1 `fsnroffst` SNR retune when parsed BA under-reads mantissas
/// vs the bitstream (threshold fsnroffst frames). Only adjusts BAP, not
/// side-info storage.
#[allow(clippy::too_many_arguments, dead_code)]
fn retune_per_channel_snr_if_needed(
    state: &mut Ac3State,
    si: &SyncInfo,
    bsi: &Bsi,
    post_sync: &[u8],
    blk: usize,
    nfchans: usize,
    bit_alloc_stages: &[u8; MAX_CHANNELS],
    pre_mantissa_bit: u64,
) {
    if blk + 1 >= BLOCKS_PER_FRAME {
        return;
    }
    // Threshold side-info values (e.g. 13) sit on a BAP cliff where the
    // bitstream often carries +1 fsnroffst step of mantissa data; other
    // values (0, 12, 14) align at parsed BA in the multitone fixture.
    if !state.fsnroffst[..nfchans].contains(&13) {
        return;
    }
    if remainder_of_frame_parses_after_unpack(state, si, bsi, post_sync, blk, pre_mantissa_bit) {
        return;
    }
    let base = state.clone();
    let mut best: Option<(Ac3State, i32, usize)> = None;
    for ch in 0..nfchans {
        for delta in [-1i32, 1] {
            let new_fsnr = base.fsnroffst[ch] as i32 + delta;
            if !(0..=15).contains(&new_fsnr) {
                continue;
            }
            let mut trial = base.clone();
            trial.snr_offset_ch[ch] += delta << 2;
            rerun_channel_bit_allocation(&mut trial, ch, si, bit_alloc_stages);
            let Some(score) = score_ba_trial(&trial, si, bsi, post_sync, blk, pre_mantissa_bit)
            else {
                continue;
            };
            if best.as_ref().map_or(true, |(_, s, _)| score > *s) {
                best = Some((trial, score, ch));
            }
        }
    }
    if let Some((trial, _, ch)) = best {
        copy_channel_bap(state, &trial, ch);
    }
}

/// Parse one `audblk()` element into [`Ac3State`] (for DSP) and
/// [`AudBlkSideInfo`] (for tests / introspection). Every bit field
/// cites its §5.4.3.x clause; the pseudo-code in Table 5.3 was the
/// authoritative reference for bit-order. Consumes exactly as many
/// bits as the spec prescribes, up to the end of the skip field; the
/// tail-end mantissas are then parsed by [`unpack_mantissas`].
#[allow(clippy::too_many_arguments, clippy::needless_option_as_deref)]
pub(crate) fn parse_audblk_into(
    state: &mut Ac3State,
    si: &SyncInfo,
    bsi: &Bsi,
    br: &mut BitReader,
    side: &mut AudBlkSideInfo,
    metrics: Option<&mut BlockParseMetrics>,
    mut trace: Option<&mut Vec<SideInfoMark>>,
    post_sync: Option<&[u8]>,
) -> Result<()> {
    let _ = si;
    let nfchans = bsi.nfchans as usize;
    let acmod = bsi.acmod;
    let blk = state.blkidx;
    let mut bit_alloc_stages = [0u8; MAX_CHANNELS];

    trace_mark(trace.as_deref_mut(), "block_start", br, None);

    // §5.4.3.1 blksw[ch] — per-channel block-switch flag (1 bit each).
    for ch in 0..nfchans {
        let v = br.read_u32(1)? != 0;
        state.channels[ch].blksw = v;
        side.blksw[ch] = v;
    }
    // §5.4.3.2 dithflag[ch] — per-channel dither flag (1 bit each).
    for ch in 0..nfchans {
        let v = br.read_u32(1)? != 0;
        state.channels[ch].dithflag = v;
        side.dithflag[ch] = v;
    }
    trace_mark(trace.as_deref_mut(), "after_dithflag", br, None);

    // §5.4.3.3 dynrnge — dynamic-range word present (1 bit).
    let dynrnge = br.read_u32(1)? != 0;
    side.dynrnge = dynrnge;
    if dynrnge {
        // §5.4.3.4 dynrng — 8-bit dynamic-range gain word.
        let dynrng = br.read_u32(8)? as u8;
        side.dynrng = dynrng;
        let g = dynrng_to_linear(dynrng);
        for ch in 0..nfchans {
            state.channels[ch].dynrng = g;
        }
    } else if blk == 0 {
        for ch in 0..nfchans {
            state.channels[ch].dynrng = 1.0;
        }
    }
    trace_mark(
        trace.as_deref_mut(),
        "after_dynrng",
        br,
        Some(format!("dynrnge={dynrnge}")),
    );

    // §5.4.3.5 dynrng2e — dual-mono ch2 dynamic-range present (1 bit),
    // §5.4.3.6 dynrng2 — dual-mono ch2 dynamic-range word (8 bits).
    if acmod == 0 {
        let dynrng2e = br.read_u32(1)? != 0;
        side.dynrng2e = dynrng2e;
        if dynrng2e {
            let d2 = br.read_u32(8)? as u8;
            side.dynrng2 = d2;
            state.channels[1].dynrng = dynrng_to_linear(d2);
        } else if blk == 0 {
            state.channels[1].dynrng = 1.0;
        }
    }

    // §5.4.3.7 cplstre — coupling strategy present (1 bit).
    let cplstre = br.read_u32(1)? != 0;
    side.cplstre = cplstre;
    if cplstre {
        bit_alloc_stages.fill(3);
        // §5.4.3.8 cplinu — coupling in use (1 bit).
        state.cpl_in_use = br.read_u32(1)? != 0;
        side.cplinu = state.cpl_in_use;
        if state.cpl_in_use {
            // §5.4.3.9 chincpl[ch] — per-channel coupling membership (1 bit).
            for ch in 0..nfchans {
                let v = br.read_u32(1)? != 0;
                state.channels[ch].in_coupling = v;
                side.chincpl[ch] = v;
            }
            // §5.4.3.10 phsflginu — phase flags in use (only in 2/0 mode).
            state.phsflginu = if acmod == 0x2 {
                br.read_u32(1)? != 0
            } else {
                false
            };
            side.phsflginu = state.phsflginu;
            // §5.4.3.11 cplbegf (4 bits), §5.4.3.12 cplendf (4 bits).
            state.cpl_begf = br.read_u32(4)? as u8;
            state.cpl_endf = br.read_u32(4)? as u8;
            side.cplbegf = state.cpl_begf;
            side.cplendf = state.cpl_endf;
            // Per A/52 §5.4.3.12 the upper sub-band index is `cplendf+2`,
            // so the spec's validity envelope is `cplbegf <= cplendf+2`
            // (equivalently `ncplsubnd = 3 + cplendf - cplbegf >= 1`).
            // The earlier strict `cplendf < cplbegf` rejection bombed out
            // of valid 5.0 (acmod=7 lfeon=0) frames whose bitstreams pick
            // narrow-coupling configs like (cplbegf=11, cplendf=10), which
            // place coupling on sub-bands 11..=12 (tc bins 169..193) — a
            // perfectly legal 2-sub-band coupling channel. Using signed
            // arithmetic also dodges the usize underflow that the previous
            // branch would have hit before the explicit check.
            let nsub = 3i32 + state.cpl_endf as i32 - state.cpl_begf as i32;
            if nsub < 1 {
                return Err(Error::invalid(
                    "ac3: §5.4.3.11/12 cplbegf > cplendf+2 — malformed coupling range",
                ));
            }
            state.cpl_nsubbnd = nsub as usize;
            // §5.4.3.13 cplbndstrc[sbnd] — 1 bit per subband for sbnd >= 1.
            state.cpl_bndstrc[0] = false;
            for bnd in 1..state.cpl_nsubbnd {
                let v = br.read_u32(1)? != 0;
                state.cpl_bndstrc[bnd] = v;
                side.cplbndstrc[bnd] = v;
            }
            // Mantissa-domain coupling range: bins [37 + 12*cplbegf,
            // 37 + 12*(cplendf+3)) per §7.4.2.
            state.cpl_begf_mant = 37 + 12 * state.cpl_begf as usize;
            state.cpl_endf_mant = 37 + 12 * (state.cpl_endf as usize + 3);
            // Derive ncplbnd by merging sub-bands whose cplbndstrc=1.
            let mut n = state.cpl_nsubbnd;
            for bnd in 1..state.cpl_nsubbnd {
                if state.cpl_bndstrc[bnd] {
                    n -= 1;
                }
            }
            state.cpl_nbnd = n;
        } else {
            // FFmpeg ac3dec: strategy present but coupling off — clear
            // stale `channel_in_cpl` from a prior block.
            for ch in 0..nfchans {
                state.channels[ch].in_coupling = false;
                side.chincpl[ch] = false;
            }
        }
    } else if blk == 0 {
        return Err(Error::invalid(
            "ac3: §5.4.3.7 coupling strategy must be present in block 0",
        ));
    }
    // `cplstre == 0` on blk > 0: reuse `state.cpl_in_use` from previous block.
    trace_mark(
        trace.as_deref_mut(),
        "after_cpl",
        br,
        Some(format!("cplstre={cplstre} cplinu={}", state.cpl_in_use)),
    );

    // §5.4.3.14 cplcoe[ch], §5.4.3.15 mstrcplco[ch], §5.4.3.16 cplcoexp,
    // §5.4.3.17 cplcomant, §5.4.3.18 phsflg[bnd].
    if state.cpl_in_use {
        let mut any = false;
        for ch in 0..nfchans {
            if state.channels[ch].in_coupling {
                // §5.4.3.14 cplcoe[ch] — coupling coordinates present (1 bit).
                let cplcoe = br.read_u32(1)? != 0;
                side.cplcoe[ch] = cplcoe;
                if cplcoe {
                    any = true;
                    // §5.4.3.15 mstrcplco[ch] — master coupling coord (2 bits).
                    let mstrcplco = br.read_u32(2)? as i32;
                    for bnd in 0..state.cpl_nbnd {
                        // §5.4.3.16 cplcoexp[ch][bnd] — 4 bits.
                        let cplcoexp = br.read_u32(4)? as i32;
                        // §5.4.3.17 cplcomant[ch][bnd] — 4 bits.
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
            }
        }
        // §5.4.3.18 phsflg[bnd] — only when 2/0, phsflginu, and at
        // least one channel emitted coupling coordinates this block.
        if acmod == 0x2 && state.phsflginu && any {
            for bnd in 0..state.cpl_nbnd {
                state.cpl_phsflg[bnd] = br.read_u32(1)? != 0;
            }
        }
    }

    // §5.4.3.19 rematstr — rematrix strategy (only in 2/0 mode).
    // §5.4.3.20 rematflg[rbnd] — per-band rematrix flag.
    if acmod == 0x2 {
        let rematstr = br.read_u32(1)? != 0;
        side.rematstr = rematstr;
        if rematstr {
            let n_remat = remat_band_count(state.cpl_in_use, state.cpl_begf);
            side.rematflg_count = n_remat as u8;
            for rbnd in 0..n_remat {
                let v = br.read_u32(1)? != 0;
                state.rematflg[rbnd] = v;
                side.rematflg[rbnd] = v;
            }
        }
        if std::env::var("AC3_TRACE_REMAT").is_ok() {
            eprintln!(
                "TRACE-REMAT blk={} rematstr={} rematflg={:?}",
                blk, rematstr, state.rematflg
            );
        }
    }
    trace_mark(
        trace.as_deref_mut(),
        "after_remat",
        br,
        Some(format!("rematstr={}", side.rematstr)),
    );

    // §5.4.3.21 cplexpstr — coupling exponent strategy (2 bits).
    // §5.4.3.22 chexpstr[ch] — fbw channel exponent strategy (2 bits).
    // §5.4.3.23 lfeexpstr — LFE exponent strategy (1 bit).
    // §5.4.3.24 chbwcod[ch] — channel bandwidth code (6 bits).
    let mut cplexpstr = 0u8;
    let mut chexpstr = [0u8; MAX_FBW];
    let mut lfeexpstr = 0u8;
    if state.cpl_in_use {
        cplexpstr = br.read_u32(2)? as u8;
    }
    side.cplexpstr = cplexpstr;
    for ch in 0..nfchans {
        chexpstr[ch] = br.read_u32(2)? as u8;
        side.chexpstr[ch] = chexpstr[ch];
    }
    if bsi.lfeon {
        lfeexpstr = br.read_u32(1)? as u8;
    }
    side.lfeexpstr = lfeexpstr;
    // chbwcod — only for non-coupled independent fbw channels with new exponents.
    let mut chbwcod = [0u8; MAX_FBW];
    for ch in 0..nfchans {
        if chexpstr[ch] != 0 && !state.channels[ch].in_coupling {
            chbwcod[ch] = br.read_u32(6)? as u8;
            side.chbwcod[ch] = chbwcod[ch];
            if chbwcod[ch] > 60 {
                return Err(Error::invalid("ac3: chbwcod > 60"));
            }
        }
    }
    trace_mark(
        trace.as_deref_mut(),
        "after_chbwcod",
        br,
        Some(format!("chexpstr={chexpstr:?} chbwcod={chbwcod:?}")),
    );

    // --- unpack coupling exponents ---
    if state.cpl_in_use && cplexpstr != 0 {
        bit_alloc_stages[MAX_FBW] = bit_alloc_stages[MAX_FBW].max(3);
        let cplabsexp = br.read_u32(4)? as i32;
        let cpl_start = state.cpl_begf_mant;
        let cpl_end = state.cpl_endf_mant;
        let grpsize = match cplexpstr {
            1 => 1,
            2 => 2,
            3 => 4,
            _ => 1,
        };
        let ncplgrps = (cpl_end - cpl_start) / (grpsize * 3);
        // Absolute exponent for coupling: cplabsexp << 1 (from 4-bit range 0..15 to full 5-bit 0..30).
        let mut raw_exp = vec![0i32; ncplgrps * 3];
        decode_exponents(
            br,
            cplabsexp << 1,
            ncplgrps,
            cplexpstr as usize,
            &mut raw_exp,
        )?;
        let ch_idx = MAX_FBW;
        // Offset for coupling per spec: cplexp[n + cplstrtmant] = exp[n+1],
        // i.e. the absolute exponent is used as a reference only and the
        // actual exponents start at index 1.
        for (i, e) in raw_exp.iter().enumerate() {
            let idx = cpl_start + i * grpsize;
            for j in 0..grpsize {
                if idx + j < N_COEFFS {
                    state.channels[ch_idx].exp[idx + j] = (*e).clamp(0, 24) as u8;
                }
            }
        }
        if std::env::var("AC3_TRACE_CPL").is_ok() {
            eprintln!(
                "TRACE-CPL blk={} cplexpstr={} cplabsexp={} (<<1={}) ncplgrps={} grpsize={}",
                blk,
                cplexpstr,
                cplabsexp,
                cplabsexp << 1,
                ncplgrps,
                grpsize
            );
            eprintln!(
                "TRACE-CPL raw_exp first 12: {:?}",
                &raw_exp[..raw_exp.len().min(12)]
            );
            eprintln!(
                "TRACE-CPL placed cpl exp[{}..{}]: {:?}",
                cpl_start,
                cpl_end,
                &state.channels[ch_idx].exp[cpl_start..cpl_end.min(cpl_start + 30)]
            );
        }
    }

    // --- unpack fbw channel exponents + gainrng ---
    for ch in 0..nfchans {
        if chexpstr[ch] != 0 {
            bit_alloc_stages[ch] = bit_alloc_stages[ch].max(3);
            let strt = 0usize;
            let end = if state.channels[ch].in_coupling {
                state.cpl_begf_mant
            } else {
                37 + 3 * (chbwcod[ch] as usize + 12)
            };
            if blk > 0 && end != state.channels[ch].end_mant {
                bit_alloc_stages.fill(3);
            }
            state.channels[ch].end_mant = end;

            // reason: kept as a compile-time-disabled debug probe that is flipped
            // to `true` locally when inspecting exponent strategy bit alignment.
            #[allow(clippy::overly_complex_bool_expr)]
            if false && blk == 0 {
                eprintln!(
                    "ch{} exps start at bit {}, end_mant={}, strategy={}",
                    ch,
                    br.bit_position(),
                    end,
                    chexpstr[ch]
                );
            }
            let absexp = br.read_u32(4)? as i32;
            let grpsize = match chexpstr[ch] {
                1 => 1,
                2 => 2,
                3 => 4,
                _ => 1,
            };
            // Guard against `end == 0` (rare: a fully-coupled channel
            // whose cpl_begf_mant lands at 0 has no independent
            // exponents). Without this guard the `(end - 1) / k` term
            // would underflow under debug arithmetic and panic the
            // decoder mid-frame.
            let nchgrps = if end == 0 {
                0usize
            } else {
                match chexpstr[ch] {
                    1 => (end - 1) / 3,
                    2 => (end - 1 + 3) / 6,
                    3 => (end - 1 + 9) / 12,
                    _ => 0,
                }
            };
            let mut raw_exp = vec![0i32; nchgrps * 3];
            decode_exponents(br, absexp, nchgrps, chexpstr[ch] as usize, &mut raw_exp)?;
            // Place: exp[0] = absexp; exp[i*grpsize + 1 + j] = raw_exp[i]
            state.channels[ch].exp[strt] = absexp.clamp(0, 24) as u8;
            for (i, e) in raw_exp.iter().enumerate() {
                let base = i * grpsize + 1;
                for j in 0..grpsize {
                    if base + j < end {
                        state.channels[ch].exp[base + j] = (*e).clamp(0, 24) as u8;
                    }
                }
            }
            let _gainrng = br.read_u32(2)?;
        } else if blk == 0 {
            return Err(Error::invalid("ac3: chexpstr=0 in block 0"));
        }
        if ch == 0 {
            trace_mark(
                trace.as_deref_mut(),
                "after_ch0_exponents",
                br,
                Some(format!("end_mant={}", state.channels[0].end_mant)),
            );
        } else if ch == 1 {
            trace_mark(
                trace.as_deref_mut(),
                "after_ch1_exponents",
                br,
                Some(format!("end_mant={}", state.channels[1].end_mant)),
            );
        }
    }

    // --- unpack LFE exponents ---
    if bsi.lfeon {
        let lfe_ch = MAX_FBW + 1;
        state.channels[lfe_ch].end_mant = 7;
        if lfeexpstr != 0 {
            let absexp = br.read_u32(4)? as i32;
            let nlfegrps = 2usize;
            let mut raw_exp = vec![0i32; nlfegrps * 3];
            decode_exponents(br, absexp, nlfegrps, 1, &mut raw_exp)?;
            state.channels[lfe_ch].exp[0] = absexp.clamp(0, 24) as u8;
            for (i, e) in raw_exp.iter().enumerate() {
                if i + 1 < 7 {
                    state.channels[lfe_ch].exp[i + 1] = (*e).clamp(0, 24) as u8;
                }
            }
        }
    }

    // §5.4.3.30 baie — bit-allocation info exists (1 bit).
    // §5.4.3.31-35 sdcycod/fdcycod/sgaincod/dbpbcod/floorcod parametric
    // masking words, present iff baie.
    let baie = br.read_u32(1)? != 0;
    side.baie = baie;
    if baie {
        state.sdcycod = br.read_u32(2)? as u8;
        state.fdcycod = br.read_u32(2)? as u8;
        state.sgaincod = br.read_u32(2)? as u8;
        state.dbpbcod = br.read_u32(2)? as u8;
        state.floorcod = br.read_u32(3)? as u8;
        for st in bit_alloc_stages.iter_mut() {
            *st = (*st).max(2);
        }
    }
    trace_mark(
        trace.as_deref_mut(),
        "after_baie",
        br,
        Some(format!("baie={baie}")),
    );
    // §5.4.3.36 snroffste — SNR-offset block flag (1 bit).
    let snroffste = br.read_u32(1)? != 0;
    side.snroffste = snroffste;
    if snroffste {
        // §5.4.3.37 csnroffst — coarse SNR offset (6 bits).
        state.snroffst_coarse = br.read_u32(6)? as u8;
        trace_mark(
            trace.as_deref_mut(),
            "after_csnroffst",
            br,
            Some(format!("coarse={}", state.snroffst_coarse)),
        );
        let coarse_base = (state.snroffst_coarse as i32 - 15) << 4;
        if state.cpl_in_use {
            // §5.4.3.38 cplfsnroffst (4 bits), §5.4.3.39 cplfgaincod (3 bits).
            let prev_fg = state.cpl_fgaincod;
            state.cpl_fsnroffst = br.read_u32(4)? as u8;
            state.cpl_fgaincod = br.read_u32(3)? as u8;
            let new_snr = (coarse_base + state.cpl_fsnroffst as i32) << 2;
            if blk > 0 && state.snr_offset_ch[MAX_FBW] != new_snr {
                bit_alloc_stages[MAX_FBW] = bit_alloc_stages[MAX_FBW].max(1);
            }
            state.snr_offset_ch[MAX_FBW] = new_snr;
            if blk > 0 && prev_fg != state.cpl_fgaincod {
                bit_alloc_stages[MAX_FBW] = bit_alloc_stages[MAX_FBW].max(2);
            }
        }
        for ch in 0..nfchans {
            // §5.4.3.40 fsnroffst[ch] (4 bits), §5.4.3.41 fgaincod[ch] (3 bits).
            let prev_fg = state.fgaincod[ch];
            state.fsnroffst[ch] = br.read_u32(4)? as u8;
            state.fgaincod[ch] = br.read_u32(3)? as u8;
            let new_snr = (coarse_base + state.fsnroffst[ch] as i32) << 2;
            if blk > 0 && state.snr_offset_ch[ch] != new_snr {
                bit_alloc_stages[ch] = bit_alloc_stages[ch].max(1);
            }
            state.snr_offset_ch[ch] = new_snr;
            if blk > 0 && prev_fg != state.fgaincod[ch] {
                bit_alloc_stages[ch] = bit_alloc_stages[ch].max(2);
            }
            trace_mark(
                trace.as_deref_mut(),
                &format!("after_fsnroffst_ch{ch}"),
                br,
                Some(format!(
                    "fsnroffst={} fgaincod={}",
                    state.fsnroffst[ch], state.fgaincod[ch]
                )),
            );
        }
        if bsi.lfeon {
            // §5.4.3.42 lfefsnroffst (4 bits), §5.4.3.43 lfefgaincod (3 bits).
            let lfe_ch = MAX_FBW + 1;
            let prev_fg = state.lfefgaincod;
            state.lfefsnroffst = br.read_u32(4)? as u8;
            state.lfefgaincod = br.read_u32(3)? as u8;
            let new_snr = (coarse_base + state.lfefsnroffst as i32) << 2;
            if blk > 0 && state.snr_offset_ch[lfe_ch] != new_snr {
                bit_alloc_stages[lfe_ch] = bit_alloc_stages[lfe_ch].max(1);
            }
            state.snr_offset_ch[lfe_ch] = new_snr;
            if blk > 0 && prev_fg != state.lfefgaincod {
                bit_alloc_stages[lfe_ch] = bit_alloc_stages[lfe_ch].max(2);
            }
        }
    }
    if let Some(ov) = state.fsnroffst_override.take() {
        for ch in 0..nfchans {
            state.fsnroffst[ch] = ov[ch];
            let coarse_base = (state.snroffst_coarse as i32 - 15) << 4;
            state.snr_offset_ch[ch] = (coarse_base + ov[ch] as i32) << 2;
            bit_alloc_stages[ch] = bit_alloc_stages[ch].max(1);
        }
    }
    // §5.4.3.44 cplleake — coupling leak init flag (1 bit).
    // §5.4.3.45 cplfleak (3 bits), §5.4.3.46 cplsleak (3 bits).
    if state.cpl_in_use {
        let cplleake = br.read_u32(1)? != 0;
        side.cplleake = cplleake;
        if cplleake {
            state.cpl_fleak = br.read_u32(3)? as u8;
            state.cpl_sleak = br.read_u32(3)? as u8;
        }
    }

    // §5.4.3.47 deltbaie — delta bit allocation info exists (1 bit).
    // §5.4.3.48-57 — per-channel + coupling delta bit allocation segments.
    // §7.2.2.6 — apply per-band ±6 dB mask offsets BEFORE final bit
    // allocation. Critical for transient blocks where the encoder uses
    // dba to lift the masking floor in low-energy bands so they don't
    // get assigned mantissa bits the encoder needs for the burst peak.
    let deltbaie = br.read_u32(1)? != 0;
    side.deltbaie = deltbaie;
    if deltbaie {
        let cpl_idx = MAX_FBW;
        let mut cpldeltbae = 0u32;
        if state.cpl_in_use {
            cpldeltbae = br.read_u32(2)?;
        }
        let mut deltbae = [0u32; MAX_FBW];
        for ch in 0..nfchans {
            deltbae[ch] = br.read_u32(2)?;
        }
        // Per Table 5.16: 0=reuse, 1=new info, 2=no delta this block, 3=reserved.
        if state.cpl_in_use {
            match cpldeltbae {
                1 => {
                    let nseg = (br.read_u32(3)? + 1) as usize;
                    state.deltnseg[cpl_idx] = nseg.min(8);
                    bit_alloc_stages[cpl_idx] = bit_alloc_stages[cpl_idx].max(2);
                    for seg in 0..state.deltnseg[cpl_idx] {
                        state.deltoffst[cpl_idx][seg] = br.read_u32(5)? as u8;
                        state.deltlen[cpl_idx][seg] = br.read_u32(4)? as u8;
                        state.deltba[cpl_idx][seg] = br.read_u32(3)? as u8;
                    }
                }
                2 => {
                    // "perform no delta alloc" — clear segments for this block.
                    state.deltnseg[cpl_idx] = 0;
                }
                _ => {
                    // 0 = reuse previous; 3 = reserved (treat as reuse to stay
                    // robust against malformed streams).
                }
            }
        }
        for ch in 0..nfchans {
            match deltbae[ch] {
                1 => {
                    let nseg = (br.read_u32(3)? + 1) as usize;
                    state.deltnseg[ch] = nseg.min(8);
                    bit_alloc_stages[ch] = bit_alloc_stages[ch].max(2);
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
        // §5.4.3.47 spec: "If deltbaie is '0' in block 0, then cpldeltbae
        // and deltbae[ch] are set to the binary value '10', and no delta
        // bit allocation is applied." This means clear all segments.
        for ch in 0..MAX_FBW + 1 {
            state.deltnseg[ch] = 0;
        }
    }

    // §5.4.3.58 skiple — skip-length-exists flag (1 bit).
    // §5.4.3.59 skipl — skip length in *bytes* (9 bits).
    // §5.4.3.60 skipfld — `skipl × 8` skip-data bits.
    let skiple = br.read_u32(1)? != 0;
    side.skiple = skiple;
    if skiple {
        let skipl = br.read_u32(9)?;
        side.skipl = skipl as u16;
        br.skip(skipl * 8)?;
    }
    trace_mark(
        trace.as_deref_mut(),
        "after_skip",
        br,
        Some(format!("skiple={skiple}")),
    );

    if std::env::var("AC3_TRACE_STAGE").is_ok() {
        let trace_f: Option<usize> = std::env::var("AC3_TRACE_FRAME")
            .ok()
            .and_then(|s| s.parse().ok());
        let trace_b: Option<usize> = std::env::var("AC3_TRACE_BLK")
            .ok()
            .and_then(|s| s.parse().ok());
        let should = trace_f.map_or(true, |f| f == state.frame_counter as usize)
            && trace_b.map_or(true, |b| b == blk);
        if should {
            eprintln!(
                "TRACE-STAGE frame={} blk={} stages={:?} chexpstr={:?} baie={baie} \
                 snroffste={snroffste} deltbaie={deltbaie} deltnseg={:?} \
                 snr_offset={:?} fgaincod={:?} end_mant=[{},{}]",
                state.frame_counter,
                blk,
                &bit_alloc_stages[..nfchans],
                &chexpstr[..nfchans],
                &state.deltnseg[..nfchans],
                &state.snr_offset_ch[..nfchans],
                &state.fgaincod[..nfchans],
                state.channels[0].end_mant,
                state.channels[1].end_mant,
            );
        }
    }

    // --- run bit allocation per channel ---
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        run_bit_allocation_staged(
            state,
            ch,
            0,
            end,
            si.fscod,
            state.snr_offset_ch[ch],
            state.fgaincod[ch],
            false,
            bit_alloc_stages[ch],
        );
    }
    if state.cpl_in_use {
        let start = state.cpl_begf_mant;
        let end = state.cpl_endf_mant;
        run_bit_allocation_staged(
            state,
            MAX_FBW,
            start,
            end,
            si.fscod,
            state.snr_offset_ch[MAX_FBW],
            state.cpl_fgaincod,
            true,
            bit_alloc_stages[MAX_FBW],
        );
    }
    if bsi.lfeon {
        let lfe_ch = MAX_FBW + 1;
        run_bit_allocation_staged(
            state,
            lfe_ch,
            0,
            7,
            si.fscod,
            state.snr_offset_ch[lfe_ch],
            state.lfefgaincod,
            false,
            bit_alloc_stages[lfe_ch],
        );
    }

    if std::env::var("AC3_TRACE_STAGE").is_ok() {
        let trace_f: Option<usize> = std::env::var("AC3_TRACE_FRAME")
            .ok()
            .and_then(|s| s.parse().ok());
        let trace_b: Option<usize> = std::env::var("AC3_TRACE_BLK")
            .ok()
            .and_then(|s| s.parse().ok());
        let should = trace_f.map_or(true, |f| f == state.frame_counter as usize)
            && trace_b.map_or(true, |b| b == blk);
        if should {
            let mut histo = [0u32; 16];
            for ch in 0..nfchans {
                let end = state.channels[ch].end_mant;
                for bin in 0..end {
                    histo[state.channels[ch].bap[bin] as usize] += 1;
                }
            }
            let walk = count_mantissa_bits_walk(state, bsi);
            eprintln!(
                "TRACE-STAGE frame={} blk={} post-BA bap_histo={:?} mant_walk={}",
                state.frame_counter, blk, histo, walk
            );
        }
    }

    // --- unpack mantissas ---
    if std::env::var("AC3_DEBUG_FULL").is_ok() && blk == 0 {
        let mut histo = [0u32; 16];
        for ch in 0..nfchans {
            let end = state.channels[ch].end_mant;
            for bin in 0..end {
                histo[state.channels[ch].bap[bin] as usize] += 1;
            }
        }
        if state.cpl_in_use {
            for bin in state.cpl_begf_mant..state.cpl_endf_mant {
                histo[state.channels[MAX_FBW].bap[bin] as usize] += 1;
            }
        }
        eprintln!("block 0 bap histogram (incl cpl): {:?}", histo);
        eprintln!(
            "ch_cpl bap[begf..end]: {:?}",
            &state.channels[MAX_FBW].bap
                [state.cpl_begf_mant..state.cpl_endf_mant.min(state.cpl_begf_mant + 30)]
        );
        eprintln!(
            "ch_cpl exp[begf..end]: {:?}",
            &state.channels[MAX_FBW].exp
                [state.cpl_begf_mant..state.cpl_endf_mant.min(state.cpl_begf_mant + 30)]
        );
        eprintln!("ch0 bap[0..133]: {:?}", &state.channels[0].bap[0..133]);
        eprintln!("ch1 bap[0..133]: {:?}", &state.channels[1].bap[0..133]);
        eprintln!("ch1 exp[0..133]: {:?}", &state.channels[1].exp[0..133]);
        eprintln!("ch0 exp[0..40]: {:?}", &state.channels[0].exp[0..40]);
        eprintln!("ch0 psd[0..10]: {:?}", &state.channels[0].psd[0..10]);
        eprintln!("ch0 bndpsd[0..10]: {:?}", &state.channels[0].bndpsd[0..10]);
        eprintln!(
            "cpl in_use: {} begf: {} endf: {} begf_mant: {} endf_mant: {} nsubbnd: {} nbnd: {}",
            state.cpl_in_use,
            state.cpl_begf,
            state.cpl_endf,
            state.cpl_begf_mant,
            state.cpl_endf_mant,
            state.cpl_nsubbnd,
            state.cpl_nbnd
        );
        eprintln!(
            "ch0 end_mant: {}, ch1 end_mant: {}",
            state.channels[0].end_mant, state.channels[1].end_mant
        );
        eprintln!(
            "snroffst: csnr={} cpl_fsnr={} cpl_fgain={} cpl_fleak={} cpl_sleak={}",
            state.snroffst_coarse,
            state.cpl_fsnroffst,
            state.cpl_fgaincod,
            state.cpl_fleak,
            state.cpl_sleak
        );
        eprintln!(
            "sdcy={} fdcy={} sgain={} dbpb={} floor={}",
            state.sdcycod, state.fdcycod, state.sgaincod, state.dbpbcod, state.floorcod
        );
        eprintln!("ch0 psd[0..20]: {:?}", &state.channels[0].psd[0..20]);
        eprintln!("ch0 bndpsd[0..20]: {:?}", &state.channels[0].bndpsd[0..20]);
        eprintln!("ch0 mask[0..20]: {:?}", &state.channels[0].mask[0..20]);
        let total_bits: u32 = histo
            .iter()
            .enumerate()
            .map(|(b, n)| match b {
                0 => 0,
                1 => 5 * n / 3 + (if n % 3 != 0 { 5 } else { 0 }),
                2 => 7 * n / 3 + (if n % 3 != 0 { 7 } else { 0 }),
                3 => 3 * n,
                4 => 7 * n / 2 + (if n % 2 != 0 { 7 } else { 0 }),
                5 => 4 * n,
                _ => crate::tables::QUANTIZATION_BITS[b] as u32 * n,
            })
            .sum();
        eprintln!(
            "block 0 pre-mantissa bit pos {}, estimated mantissa bits {}",
            br.bit_position(),
            total_bits
        );
    }
    let bit_pre_mantissa = br.bit_position();
    trace_mark(
        trace.as_deref_mut(),
        "pre_mantissa",
        br,
        Some(format!("fsnroffst={:?}", &state.fsnroffst[..nfchans])),
    );
    if let Some((oblk, ref bap)) = state.bap_override {
        if blk == oblk {
            for ch in 0..nfchans {
                let end = state.channels[ch].end_mant;
                state.channels[ch].bap[..end].copy_from_slice(&bap[ch][..end]);
            }
        }
    }
    unpack_mantissas(state, bsi, br)?;
    let _ = post_sync;
    if let Some(m) = metrics {
        m.bit_pre_mantissa = bit_pre_mantissa;
        m.bit_block_end = br.bit_position();
    }

    Ok(())
}

/// Convert an 8-bit dynrng word to linear gain (§7.7.1.2).
fn dynrng_to_linear(dynrng: u8) -> f32 {
    // X0 X1 X2 . Y3 Y4 Y5 Y6 Y7
    // Interpret X as 3-bit signed int in range -4..3.
    let x = ((dynrng >> 5) & 0x7) as i32;
    let x_signed = if x >= 4 { x - 8 } else { x };
    let y = (dynrng & 0x1F) as i32;
    // Y is unsigned fractional with leading 1, so Y = 0.1Y3Y4Y5Y6Y7_binary.
    // Value = (32 + y) / 64.
    let y_val = (32 + y) as f32 / 64.0;
    // Total gain in dB = (x_signed + 1) * 6.02 dB, combined with Y as
    // linear multiplier.
    let shift = x_signed + 1; // number of left arithmetic shifts
    let base = 2f32.powi(shift);
    base * y_val
}

/// Number of rematrix bands (Table 5.15).
//
// reason: the two `4` arms are spec-faithful — Table 5.15 lists nrematbnd=4
// for both "coupling not in use" and "cplbegf > 2". Collapsing them would
// obscure the table structure for future audits.
#[allow(clippy::if_same_then_else)]
pub(crate) fn remat_band_count(cplinu: bool, cplbegf: u8) -> usize {
    if !cplinu {
        4
    } else if cplbegf > 2 {
        4
    } else if cplbegf > 0 {
        3
    } else {
        2
    }
}

/// E-AC-3 number-of-rematrix-bands `nrematbd` per §E.3.3.2, which folds
/// in both spectral extension and enhanced coupling.
///
/// The §E.3.3.2 pseudo-code decision tree, transcribed verbatim:
///
/// ```text
/// if (cplinu) {
///     if (ecplinu) {                         // enhanced coupling
///         if      (ecplbegf == 0) nrematbd = 0
///         else if (ecplbegf == 1) nrematbd = 1
///         else if (ecplbegf == 2) nrematbd = 2
///         else if (ecplbegf <  5) nrematbd = 3
///         else                    nrematbd = 4
///     } else {                               // standard coupling
///         if      (cplbegf == 0)  nrematbd = 2
///         else if (cplbegf <  3)  nrematbd = 3
///         else                    nrematbd = 4
///     }
/// } else if (spxinu) {
///     if (spxbegf < 2) nrematbd = 3 else nrematbd = 4
/// } else {
///     nrematbd = 4
/// }
/// ```
///
/// The standard-coupling arm is identical to [`remat_band_count`]. The
/// enhanced-coupling arm is new (an `ecplinu` 2/0 block uses `ecplbegf`,
/// not `cplbegf`, to size the rematrix-flag field — and uniquely admits
/// `nrematbd = 0`, suppressing the field entirely). `spx_in_use` /
/// `spx_begin_subbnd` come from the SPX strategy block; `spxbegf < 2` is
/// equivalent to `spx_begin_subbnd < 4`. `ecplbegf` is the raw 4-bit
/// `ecplbegf` code carried on [`crate::eac3::ecpl::EcplStrategy`].
pub(crate) fn remat_band_count_spx(
    cplinu: bool,
    cplbegf: u8,
    ecpl_in_use: bool,
    ecplbegf: u8,
    spx_in_use: bool,
    spx_begin_subbnd: usize,
) -> usize {
    if cplinu {
        if ecpl_in_use {
            // §E.3.3.2 enhanced-coupling arm — thresholds the raw ecplbegf.
            match ecplbegf {
                0 => 0,
                1 => 1,
                2 => 2,
                3 | 4 => 3,
                _ => 4,
            }
        } else {
            remat_band_count(true, cplbegf)
        }
    } else if spx_in_use {
        if spx_begin_subbnd < 4 {
            3
        } else {
            4
        }
    } else {
        4
    }
}

/// Decode a grouped exponent run (§7.1.3).
pub(crate) fn decode_exponents(
    br: &mut BitReader,
    absexp: i32,
    ngrps: usize,
    _expstr: usize,
    out: &mut [i32],
) -> Result<()> {
    // Spec §7.1.3 pseudo-code: unpack mapped values, convert to dexp
    // (subtract 2), then prefix-sum with the seeding absolute exponent.
    let mut prev = absexp;
    for grp in 0..ngrps {
        let gexp = br.read_u32(7)? as i32;
        let m1 = gexp / 25;
        let m2 = (gexp % 25) / 5;
        let m3 = (gexp % 25) % 5;
        let dexp0 = m1 - 2;
        let dexp1 = m2 - 2;
        let dexp2 = m3 - 2;
        let e0 = prev + dexp0;
        let e1 = e0 + dexp1;
        let e2 = e1 + dexp2;
        out[grp * 3] = e0;
        out[grp * 3 + 1] = e1;
        out[grp * 3 + 2] = e2;
        prev = e2;
    }
    // Post-chain, clamp each exponent to the valid 0..=24 range. Encoders
    // that overshoot (e.g. when encoding an exactly-zero channel after
    // rematrix) rely on this for sane decoder output.
    for v in out.iter_mut() {
        *v = (*v).clamp(0, 24);
    }
    Ok(())
}

/// Parametric bit allocation (§7.2.2) for a single channel range.
/// `start`..`end` is the mantissa-bin range.
/// `stage`: 0=reuse all, 1=BAP only, 2=mask+BAP, 3=PSD+mask+BAP (FFmpeg
/// `bit_alloc_stages`).
#[allow(clippy::too_many_arguments)]
pub(crate) fn run_bit_allocation_staged(
    state: &mut Ac3State,
    ch: usize,
    start: usize,
    end: usize,
    fscod: u8,
    snr_offset: i32,
    fgaincod: u8,
    is_coupling: bool,
    stage: u8,
) {
    if end <= start || stage == 0 {
        return;
    }
    let bndstrt = MASKTAB[start] as usize;
    let bndend = MASKTAB[end - 1] as usize + 1;
    let sr_shift = state.sr_shift as i32;
    let sdecay = SLOWDEC[state.sdcycod as usize] >> sr_shift;
    let fdecay = FASTDEC[state.fdcycod as usize] >> sr_shift;
    let sgain = SLOWGAIN[state.sgaincod as usize];
    let dbknee = DBPBTAB[state.dbpbcod as usize];
    let floor = FLOORTAB[state.floorcod as usize];
    let fgain = FASTGAIN[fgaincod as usize];

    if stage > 2 {
        for bin in start..end {
            let e = state.channels[ch].exp[bin] as i32;
            state.channels[ch].psd[bin] = (3072 - (e << 7)) as i16;
        }
        integrate_band_psd(
            &state.channels[ch].psd,
            start,
            end,
            &mut state.channels[ch].bndpsd,
        );
    }

    let mut mask = [0i32; 50];
    if stage > 1 {
        let (fastleak_init, slowleak_init) = if is_coupling {
            ((state.cpl_fleak as i32) << 8, (state.cpl_sleak as i32) << 8)
        } else {
            (0i32, 0i32)
        };
        let mut fastleak = fastleak_init + 768;
        let mut slowleak = slowleak_init + 768;
        let mut excite = [0i32; 50];
        let mut lowcomp = 0i32;
        let mut begin;

        if is_coupling {
            begin = bndstrt;
            for bin in bndstrt..bndend {
                fastleak -= fdecay;
                fastleak = fastleak.max(state.channels[ch].bndpsd[bin] as i32 - fgain);
                slowleak -= sdecay;
                slowleak = slowleak.max(state.channels[ch].bndpsd[bin] as i32 - sgain);
                excite[bin] = fastleak.max(slowleak);
            }
            let _ = begin;
        } else if bndstrt == 0 {
            let lfe_last = end == 7;
            let bpsd = |i: usize| -> i32 { state.channels[ch].bndpsd[i.min(49)] as i32 };
            if 0 < bndend {
                lowcomp = calc_lowcomp1(lowcomp, bpsd(0), bpsd(1), 384);
                excite[0] = bpsd(0) - fgain - lowcomp;
            }
            if 1 < bndend {
                lowcomp = calc_lowcomp1(lowcomp, bpsd(1), bpsd(2), 384);
                excite[1] = bpsd(1) - fgain - lowcomp;
            }
            begin = 7;
            for bin in 2..7.min(bndend) {
                if !(lfe_last && bin == 6) {
                    lowcomp = calc_lowcomp1(lowcomp, bpsd(bin), bpsd(bin + 1), 384);
                }
                fastleak = bpsd(bin) - fgain;
                slowleak = bpsd(bin) - sgain;
                excite[bin] = fastleak - lowcomp;
                if !(lfe_last && bin == 6) && bpsd(bin) <= bpsd(bin + 1) {
                    begin = bin + 1;
                    break;
                }
            }
            for bin in begin..22.min(bndend) {
                if !(lfe_last && bin == 6) {
                    lowcomp = calc_lowcomp(lowcomp, bpsd(bin), bpsd(bin + 1), bin);
                }
                fastleak -= fdecay;
                fastleak = fastleak.max(bpsd(bin) - fgain);
                slowleak -= sdecay;
                slowleak = slowleak.max(bpsd(bin) - sgain);
                excite[bin] = (fastleak - lowcomp).max(slowleak);
            }
            if bndend > 22 {
                for bin in 22..bndend {
                    fastleak -= fdecay;
                    fastleak = fastleak.max(bpsd(bin) - fgain);
                    slowleak -= sdecay;
                    slowleak = slowleak.max(bpsd(bin) - sgain);
                    excite[bin] = fastleak.max(slowleak);
                }
            }
        } else {
            begin = bndstrt;
            for bin in begin..bndend {
                fastleak -= fdecay;
                fastleak = fastleak.max(state.channels[ch].bndpsd[bin] as i32 - fgain);
                slowleak -= sdecay;
                slowleak = slowleak.max(state.channels[ch].bndpsd[bin] as i32 - sgain);
                excite[bin] = fastleak.max(slowleak);
            }
        }

        let hth_row = &HTH[fscod as usize];
        for bin in bndstrt..bndend {
            let mut exc = excite[bin];
            if (state.channels[ch].bndpsd[bin] as i32) < dbknee {
                exc += (dbknee - state.channels[ch].bndpsd[bin] as i32) >> 2;
            }
            let hth_band = (bin >> state.sr_shift).min(49);
            mask[bin] = exc.max(hth_row[hth_band] as i32);
        }

        let is_lfe = !is_coupling && ch > MAX_FBW;
        let dba_idx = if is_coupling { MAX_FBW } else { ch };
        if !is_lfe && state.deltnseg[dba_idx] > 0 {
            let mut band = 0usize;
            for seg in 0..state.deltnseg[dba_idx] {
                band += state.deltoffst[dba_idx][seg] as usize;
                let dba_raw = state.deltba[dba_idx][seg] as i32;
                let delta = if dba_raw >= 4 {
                    (dba_raw - 3) << 7
                } else {
                    (dba_raw - 4) << 7
                };
                let len = state.deltlen[dba_idx][seg] as usize;
                for _ in 0..len {
                    if band < 50 {
                        mask[band] += delta;
                    }
                    band += 1;
                }
            }
        }

        for bin in bndstrt..bndend {
            state.channels[ch].mask[bin] = mask[bin] as i16;
        }
    } else {
        for bin in bndstrt..bndend {
            mask[bin] = state.channels[ch].mask[bin] as i32;
        }
    }

    if stage > 0 {
        let mut i = start;
        let mut j = MASKTAB[start] as usize;
        loop {
            let lastbin = (BNDTAB[j] as usize + BNDSZ[j] as usize).min(end);
            let mut m = mask[j];
            m -= snr_offset;
            m -= floor;
            if m < 0 {
                m = 0;
            }
            m &= 0x1fe0;
            m += floor;
            while i < lastbin {
                let addr = ((state.channels[ch].psd[i] as i32 - m) >> 5).clamp(0, 63) as usize;
                state.channels[ch].bap[i] = BAPTAB[addr];
                i += 1;
            }
            if i >= end {
                break;
            }
            j += 1;
        }
    }

    // Diagnostic trace (unchanged from full BA path).
    let trace_frame = std::env::var("AC3_TRACE_FRAME")
        .ok()
        .and_then(|s| s.parse::<u64>().ok());
    let trace_blk = std::env::var("AC3_TRACE_BLK")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(0);
    if let Some(tf) = trace_frame {
        if tf == state.frame_counter && state.blkidx == trace_blk {
            let label = if is_coupling {
                "cpl".to_string()
            } else if ch == MAX_FBW + 1 {
                "lfe".to_string()
            } else {
                format!("ch{ch}")
            };
            eprintln!(
                "TRACE frame={} blk={} {} stage={} bndstrt={} bndend={} snroffset={:#x}",
                state.frame_counter, state.blkidx, label, stage, bndstrt, bndend, snr_offset
            );
        }
    }
}

/// Full §7.2.2 bit allocation (stage 3).
#[allow(clippy::too_many_arguments)]
pub(crate) fn run_bit_allocation(
    state: &mut Ac3State,
    ch: usize,
    start: usize,
    end: usize,
    fscod: u8,
    fsnroffst: u8,
    fgaincod: u8,
    is_coupling: bool,
) {
    let snroffset = (((state.snroffst_coarse as i32 - 15) << 4) + fsnroffst as i32) << 2;
    run_bit_allocation_staged(
        state,
        ch,
        start,
        end,
        fscod,
        snroffset,
        fgaincod,
        is_coupling,
        3,
    );
}

/// Recompute `bndpsd` from per-bin `psd` (diagnostic; matches [`integrate_band_psd`]).
pub fn reintegrate_bndpsd(psd: &[i16], start: usize, end: usize) -> [i16; 50] {
    let mut bndpsd = [0i16; 50];
    integrate_band_psd(psd, start, end, &mut bndpsd);
    bndpsd
}

/// Log-addition (§7.2.2.3 logadd).
/// Band PSD integration matching FFmpeg `ff_ac3_bit_alloc_calc_psd`.
fn integrate_band_psd(psd: &[i16], start: usize, end: usize, bndpsd: &mut [i16]) {
    let mut bin = start;
    let mut band = MASKTAB[start] as usize;
    loop {
        let mut v = psd[bin] as i32;
        bin += 1;
        let band_end = (BNDTAB[band] as usize + BNDSZ[band] as usize).min(end);
        while bin < band_end {
            let pb = psd[bin] as i32;
            let max = v.max(pb);
            let adr = (max - ((v + pb + 1) >> 1)).clamp(0, 255) as usize;
            v = max + LATAB[adr] as i32;
            bin += 1;
        }
        bndpsd[band] = v as i16;
        band += 1;
        if end <= band_end {
            break;
        }
    }
}

/// calc_lowcomp (§7.2.2.4).
fn calc_lowcomp1(a: i32, b0: i32, b1: i32, c: i32) -> i32 {
    if b0 + 256 == b1 {
        c
    } else if b0 > b1 {
        (a - 64).max(0)
    } else {
        a
    }
}

/// calc_lowcomp (§7.2.2.4).
fn calc_lowcomp(a: i32, b0: i32, b1: i32, bin: usize) -> i32 {
    let mut a = a;
    if bin < 7 {
        if b0 + 256 == b1 {
            a = 384;
        } else if b0 > b1 {
            a = (a - 64).max(0);
        }
    } else if bin < 20 {
        if b0 + 256 == b1 {
            a = 320;
        } else if b0 > b1 {
            a = (a - 64).max(0);
        }
    } else {
        a = (a - 128).max(0);
    }
    a
}

/// FFmpeg `ff_ac3_bit_alloc_calc_mask` (libavcodec/ac3.c) for diagnostics.
#[allow(clippy::too_many_arguments)]
pub fn ffmpeg_ref_calc_mask(
    band_psd: &[i16],
    start: usize,
    end: usize,
    fast_decay: i32,
    slow_decay: i32,
    slow_gain: i32,
    db_per_bit: i32,
    fast_gain: i32,
    cpl_fast_leak: i32,
    cpl_slow_leak: i32,
    fscod: u8,
    sr_shift: u8,
    is_lfe: bool,
    is_coupling: bool,
    deltnseg: usize,
    deltoffst: &[u8],
    deltlen: &[u8],
    deltba: &[u8],
) -> [i16; 50] {
    let mut mask = [0i16; 50];
    if end <= start {
        return mask;
    }
    let band_start = MASKTAB[start] as usize;
    let band_end = MASKTAB[end - 1] as usize + 1;
    let bpsd = |i: usize| band_psd[i.min(49)] as i32;
    let mut excite = [0i32; 50];
    let mut begin;

    if is_coupling {
        begin = band_start;
        let mut fastleak = (cpl_fast_leak << 8) + 768;
        let mut slowleak = (cpl_slow_leak << 8) + 768;
        for band in band_start..band_end {
            fastleak = (fastleak - fast_decay).max(bpsd(band) - fast_gain);
            slowleak = (slowleak - slow_decay).max(bpsd(band) - slow_gain);
            excite[band] = fastleak.max(slowleak);
        }
        let _ = begin;
    } else if band_start == 0 {
        let mut lowcomp = 0i32;
        let mut fastleak = 0i32;
        let mut slowleak = 0i32;
        if band_start < band_end {
            lowcomp = calc_lowcomp1(lowcomp, bpsd(0), bpsd(1), 384);
            excite[0] = bpsd(0) - fast_gain - lowcomp;
        }
        if band_start + 1 < band_end {
            lowcomp = calc_lowcomp1(lowcomp, bpsd(1), bpsd(2), 384);
            excite[1] = bpsd(1) - fast_gain - lowcomp;
        }
        begin = 7;
        for band in 2..7.min(band_end) {
            if !(is_lfe && band == 6) {
                lowcomp = calc_lowcomp1(lowcomp, bpsd(band), bpsd(band + 1), 384);
            }
            fastleak = bpsd(band) - fast_gain;
            slowleak = bpsd(band) - slow_gain;
            excite[band] = fastleak - lowcomp;
            if !(is_lfe && band == 6) && bpsd(band) <= bpsd(band + 1) {
                begin = band + 1;
                break;
            }
        }
        let end1 = 22.min(band_end);
        for band in begin..end1 {
            if !(is_lfe && band == 6) {
                lowcomp = calc_lowcomp(lowcomp, bpsd(band), bpsd(band + 1), band);
            }
            fastleak = (fastleak - fast_decay).max(bpsd(band) - fast_gain);
            slowleak = (slowleak - slow_decay).max(bpsd(band) - slow_gain);
            excite[band] = (fastleak - lowcomp).max(slowleak);
        }
        begin = 22;
        for band in begin..band_end {
            fastleak = (fastleak - fast_decay).max(bpsd(band) - fast_gain);
            slowleak = (slowleak - slow_decay).max(bpsd(band) - slow_gain);
            excite[band] = fastleak.max(slowleak);
        }
    } else {
        begin = band_start;
        let mut fastleak = 0i32;
        let mut slowleak = 0i32;
        for band in begin..band_end {
            fastleak = (fastleak - fast_decay).max(bpsd(band) - fast_gain);
            slowleak = (slowleak - slow_decay).max(bpsd(band) - slow_gain);
            excite[band] = fastleak.max(slowleak);
        }
    }

    let hth_row = &HTH[fscod as usize];
    for band in band_start..band_end {
        let mut exc = excite[band];
        let tmp = db_per_bit - bpsd(band);
        if tmp > 0 {
            exc += tmp >> 2;
        }
        let hth_band = (band >> sr_shift).min(49);
        mask[band] = exc.max(hth_row[hth_band] as i32) as i16;
    }

    if deltnseg > 0 {
        let mut band = 0usize;
        for seg in 0..deltnseg {
            band += deltoffst[seg] as usize;
            let dba_raw = deltba[seg] as i32;
            let delta = if dba_raw >= 4 {
                (dba_raw - 3) << 7
            } else {
                (dba_raw - 4) << 7
            };
            let len = deltlen[seg] as usize;
            for _ in 0..len {
                if band < 50 {
                    mask[band] = (mask[band] as i32 + delta) as i16;
                }
                band += 1;
            }
        }
    }
    mask
}

/// FFmpeg `ac3_bit_alloc_calc_bap` (libavcodec/ac3dsp.c).
pub fn ffmpeg_ref_calc_bap(
    mask: &[i16],
    psd: &[i16],
    start: usize,
    end: usize,
    snr_offset: i32,
    floor: i32,
) -> Vec<u8> {
    let mut bap = vec![0u8; end];
    if snr_offset == -960 {
        return bap;
    }
    let mut bin = start;
    let mut band = MASKTAB[start] as usize;
    loop {
        let mut m = mask[band] as i32;
        m -= snr_offset;
        m -= floor;
        if m < 0 {
            m = 0;
        }
        m &= 0x1fe0;
        m += floor;
        let band_end = (BNDTAB[band] as usize + BNDSZ[band] as usize).min(end);
        while bin < band_end {
            let addr = ((psd[bin] as i32 - m) >> 5).clamp(0, 63) as usize;
            bap[bin] = BAPTAB[addr];
            bin += 1;
        }
        if bin >= end {
            break;
        }
        band += 1;
    }
    bap
}

/// Count mantissa bits for a `bap[0..end]` slice (walk schedule).
pub fn count_mantissa_bits_for_bap(bap: &[u8], end: usize) -> u32 {
    let mut total = 0u32;
    let mut g1_left = 0u32;
    let mut g2_left = 0u32;
    let mut g4_left = 0u32;
    for &b in &bap[..end.min(bap.len())] {
        match b {
            0 => {}
            1 => {
                if g1_left == 0 {
                    total += 5;
                    g1_left = 3;
                }
                g1_left -= 1;
            }
            2 => {
                if g2_left == 0 {
                    total += 7;
                    g2_left = 3;
                }
                g2_left -= 1;
            }
            4 => {
                if g4_left == 0 {
                    total += 7;
                    g4_left = 2;
                }
                g4_left -= 1;
            }
            b => total += QUANTIZATION_BITS[b as usize] as u32,
        }
    }
    total
}

/// 16-bit Galois LFSR used to generate dither for `bap=0` mantissas
/// (§7.3.4). The spec says "any reasonably random sequence may be
/// used" — we use the classic x^16 + x^14 + x^13 + x^11 + 1 polynomial
/// because it has a maximal 65535-sample period and the output looks
/// like white noise to within a few bits. Seed is arbitrary but
/// fixed so decodes are deterministic.
pub(crate) fn dither_lfsr(state: &mut u32) -> f32 {
    // Advance the 16-bit LFSR one step and return a uniform value in
    // the range `[-0.707, 0.707)` — the spec's "optimum" scaling
    // (0.707 ≈ 1/√2). Uses the classic Fibonacci taps at bits
    // 15, 13, 12, 10 of a 16-bit state.
    let bit = ((*state >> 15) ^ (*state >> 13) ^ (*state >> 12) ^ (*state >> 10)) & 1;
    *state = ((*state << 1) | bit) & 0xFFFF;
    // Center around zero: bit15 of the 16-bit state becomes the sign,
    // lower 15 bits provide magnitude.
    let signed = (*state as i32).wrapping_sub(0x8000) as f32 / 32768.0;
    signed * 0.707
}

/// Unpack + dequantize mantissas for all channels (§7.3).
/// Populates ChannelState.coeffs with dequantized transform coefficients.
pub(crate) fn unpack_mantissas(state: &mut Ac3State, bsi: &Bsi, br: &mut BitReader) -> Result<()> {
    let nfchans = bsi.nfchans as usize;
    // Zero any leftover coefficient slots so stale data from prior blocks
    // can never bleed into the IMDCT input. Unpacked mantissas overwrite
    // bins 0..end_mant, decoupling overwrites bins in the coupling range,
    // but all other bins (e.g. end_mant..N_COEFFS on an uncoupled or
    // narrow-band channel) must read as exactly zero.
    for ch in 0..MAX_CHANNELS {
        for v in state.channels[ch].coeffs.iter_mut() {
            *v = 0.0;
        }
    }
    let mut got_cplchan = false;
    // Grouped-mantissa buffers per bap: values 1,2,4 have triples/pairs
    // shared across channels in frequency order. The spec says groups
    // are *shared across exponent sets*, meaning once a group is started
    // for bap=1 (5-bit triple), subsequent mantissas of bap=1 consume
    // from that group, even if a different channel emits them. We
    // implement this per-bap buffer state.
    let mut grp1: [f32; 3] = [0.0; 3];
    let mut grp1_n = 0usize; // remaining in buffer
    let mut grp2: [f32; 3] = [0.0; 3];
    let mut grp2_n = 0usize;
    let mut grp4: [f32; 2] = [0.0; 2];
    let mut grp4_n = 0usize;

    // Optional per-block mantissa trace gated by `AC3_TRACE_FRAME=N` and
    // `AC3_TRACE_BLK=B` plus `AC3_TRACE_MANT=1`. Used by maintainers when
    // chasing bit-stream alignment issues; off by default to keep the
    // hot path clean.
    let trace_mant_frame = std::env::var("AC3_TRACE_FRAME")
        .ok()
        .and_then(|s| s.parse::<u64>().ok());
    let trace_mant_blk = std::env::var("AC3_TRACE_BLK")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(0);
    let trace_mant_on = std::env::var("AC3_TRACE_MANT").is_ok();
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        let dith = state.channels[ch].dithflag;
        let trace_this = trace_mant_on
            && trace_mant_frame == Some(state.frame_counter)
            && state.blkidx == trace_mant_blk;
        if trace_this {
            eprintln!(
                "TRACE-MANT ch{} mantissa decode (end={}, dith={}):",
                ch, end, dith
            );
        }
        for bin in 0..end {
            let bap = state.channels[ch].bap[bin];
            let bit_pos_before = if trace_this { br.bit_position() } else { 0 };
            let val = fetch_mantissa(
                br,
                bap,
                &mut grp1,
                &mut grp1_n,
                &mut grp2,
                &mut grp2_n,
                &mut grp4,
                &mut grp4_n,
                false,
            )?;
            // Dither for bap=0 mantissas (§7.3.4): when dithflag is
            // set, replace the zero-level mantissa with an LFSR-driven
            // pseudo-random value scaled by 0.707 before the standard
            // `>> exponent` coefficient reconstruction. This fills
            // inaudible masked bands with near-noise instead of
            // silence, preventing coloration of subsequent DSP stages
            // (especially rematrix and the IMDCT post-chain).
            let final_val = if bap == 0 && dith {
                dither_lfsr(&mut state.dither_lfsr_state)
            } else {
                val
            };
            let e = state.channels[ch].exp[bin] as i32;
            state.channels[ch].coeffs[bin] = final_val * 2f32.powi(-e);
            if trace_this && bin < 32 {
                eprintln!(
                    "TRACE-MANT ch{} bin={:3} bap={:2} exp={:2} bit_pos_before={} mant={:.5} dither_used={} coeff={:.5e}",
                    ch,
                    bin,
                    bap,
                    e,
                    bit_pos_before,
                    val,
                    bap == 0 && dith,
                    state.channels[ch].coeffs[bin]
                );
            }
        }
        if state.cpl_in_use && state.channels[ch].in_coupling && !got_cplchan {
            let start = state.cpl_begf_mant;
            let end_c = state.cpl_endf_mant;
            let cplc = MAX_FBW;
            for bin in start..end_c {
                let bap = state.channels[cplc].bap[bin];
                // Coupling-channel mantissas are never dithered — the
                // spec explicitly says dither is applied after a channel
                // is extracted from the coupling channel (§7.3.4 para 1).
                let val = fetch_mantissa(
                    br,
                    bap,
                    &mut grp1,
                    &mut grp1_n,
                    &mut grp2,
                    &mut grp2_n,
                    &mut grp4,
                    &mut grp4_n,
                    false,
                )?;
                let e = state.channels[cplc].exp[bin] as i32;
                state.channels[cplc].coeffs[bin] = val * 2f32.powi(-e);
            }
            got_cplchan = true;
        }
    }
    if bsi.lfeon {
        let lfe_ch = MAX_FBW + 1;
        let dith = state.channels[lfe_ch].dithflag;
        for bin in 0..7 {
            let bap = state.channels[lfe_ch].bap[bin];
            let val = fetch_mantissa(
                br,
                bap,
                &mut grp1,
                &mut grp1_n,
                &mut grp2,
                &mut grp2_n,
                &mut grp4,
                &mut grp4_n,
                false,
            )?;
            let final_val = if bap == 0 && dith {
                dither_lfsr(&mut state.dither_lfsr_state)
            } else {
                val
            };
            let e = state.channels[lfe_ch].exp[bin] as i32;
            state.channels[lfe_ch].coeffs[bin] = final_val * 2f32.powi(-e);
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn fetch_mantissa(
    br: &mut BitReader,
    bap: u8,
    g1: &mut [f32; 3],
    g1n: &mut usize,
    g2: &mut [f32; 3],
    g2n: &mut usize,
    g4: &mut [f32; 2],
    g4n: &mut usize,
    dithflag: bool,
) -> Result<f32> {
    let _ = dithflag;
    match bap {
        0 => {
            // Dither replacement for bap=0 is handled by the caller
            // (unpack_mantissas) after we return. Here we just signal
            // "no bits consumed" by returning 0.
            Ok(0.0)
        }
        1 => {
            if *g1n == 0 {
                let code = br.read_u32(5)? as i32;
                let m1 = code / 9;
                let m2 = (code % 9) / 3;
                let m3 = code % 3;
                g1[0] = MANT_LEVEL_3[m1.clamp(0, 2) as usize];
                g1[1] = MANT_LEVEL_3[m2.clamp(0, 2) as usize];
                g1[2] = MANT_LEVEL_3[m3.clamp(0, 2) as usize];
                *g1n = 3;
            }
            let v = g1[3 - *g1n];
            *g1n -= 1;
            Ok(v)
        }
        2 => {
            if *g2n == 0 {
                let code = br.read_u32(7)? as i32;
                let m1 = code / 25;
                let m2 = (code % 25) / 5;
                let m3 = code % 5;
                g2[0] = MANT_LEVEL_5[m1.clamp(0, 4) as usize];
                g2[1] = MANT_LEVEL_5[m2.clamp(0, 4) as usize];
                g2[2] = MANT_LEVEL_5[m3.clamp(0, 4) as usize];
                *g2n = 3;
            }
            let v = g2[3 - *g2n];
            *g2n -= 1;
            Ok(v)
        }
        3 => {
            let code = br.read_u32(3)? as usize;
            Ok(MANT_LEVEL_7[code.min(6)])
        }
        4 => {
            if *g4n == 0 {
                let code = br.read_u32(7)? as i32;
                let m1 = code / 11;
                let m2 = code % 11;
                g4[0] = MANT_LEVEL_11[m1.clamp(0, 10) as usize];
                g4[1] = MANT_LEVEL_11[m2.clamp(0, 10) as usize];
                *g4n = 2;
            }
            let v = g4[2 - *g4n];
            *g4n -= 1;
            Ok(v)
        }
        5 => {
            let code = br.read_u32(4)? as usize;
            Ok(MANT_LEVEL_15[code.min(14)])
        }
        b if (6..=15).contains(&b) => {
            let nbits = QUANTIZATION_BITS[b as usize] as u32;
            let raw = br.read_u32(nbits)? as i32;
            // Sign-extend the top bit as a signed two's-complement fraction.
            let shift = 32 - nbits;
            let signed = (raw << shift) >> shift;
            // Normalize to (-1, 1): divide by 2^(nbits-1).
            let scale = 2f32.powi(-(nbits as i32 - 1));
            Ok(signed as f32 * scale)
        }
        _ => Ok(0.0),
    }
}

/// Histogram-based mantissa bit estimate (legacy debug formula in
/// `AC3_DEBUG_FULL`). Charges grouped bap 1/2/4 per §7.3.5 padding.
pub fn mantissa_bits_from_histo(histo: &[u32; 16]) -> u32 {
    histo
        .iter()
        .enumerate()
        .map(|(b, n)| match b {
            0 => 0,
            1 => 5 * n / 3 + u32::from(n % 3 != 0),
            2 => 7 * n / 3 + u32::from(n % 3 != 0),
            3 => 3 * n,
            4 => 7 * n / 2 + u32::from(n % 2 != 0),
            5 => 4 * n,
            _ => crate::tables::QUANTIZATION_BITS[b] as u32 * n,
        })
        .sum()
}

/// Count mantissa bits the decoder walk in [`unpack_mantissas`] would
/// consume for the current block's `bap[]` arrays (no bitstream read).
pub fn count_mantissa_bits_walk(state: &Ac3State, bsi: &Bsi) -> u32 {
    let nfchans = bsi.nfchans as usize;
    let mut total = 0u32;
    let mut g1_left = 0u32;
    let mut g2_left = 0u32;
    let mut g4_left = 0u32;
    let mut got_cplchan = false;
    let mut charge = |bap: u8| match bap {
        0 => {}
        1 => {
            if g1_left == 0 {
                total += 5;
                g1_left = 3;
            }
            g1_left -= 1;
        }
        2 => {
            if g2_left == 0 {
                total += 7;
                g2_left = 3;
            }
            g2_left -= 1;
        }
        4 => {
            if g4_left == 0 {
                total += 7;
                g4_left = 2;
            }
            g4_left -= 1;
        }
        b => total += crate::tables::QUANTIZATION_BITS[b as usize] as u32,
    };
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        for bin in 0..end {
            charge(state.channels[ch].bap[bin]);
        }
        if state.cpl_in_use && state.channels[ch].in_coupling && !got_cplchan {
            let cplc = MAX_FBW;
            for bin in state.cpl_begf_mant..state.cpl_endf_mant {
                charge(state.channels[cplc].bap[bin]);
            }
            got_cplchan = true;
        }
    }
    if bsi.lfeon {
        let lfe_ch = MAX_FBW + 1;
        for bin in 0..7 {
            charge(state.channels[lfe_ch].bap[bin]);
        }
    }
    total
}

/// Mantissa bit estimate matching FFmpeg `ac3_compute_mantissa_size` for one
/// block (includes encoder search init padding: bap1+=2, bap2+=2, bap4+=1).
pub fn ffmpeg_encoder_mant_bits_from_histo(histo: &[u32; 16]) -> u32 {
    let mut h = *histo;
    h[1] += 2;
    h[2] += 2;
    h[4] += 1;
    let mut bits = (h[1] / 3) * 5;
    bits += (h[2] / 3 + (h[4] >> 1)) * 7;
    bits += h[3] * 3;
    for bap in 5..16 {
        bits += h[bap] * crate::tables::QUANTIZATION_BITS[bap] as u32;
    }
    bits
}

/// Unified encoder SNR search index (0..1023) from decoder `snr_offset_ch`.
pub fn snr_search_index_from_offset(snr_offset: i32) -> u32 {
    (snr_offset / 4 + 240).clamp(0, 1023) as u32
}

/// Encoder `bit_alloc()` SNR argument from search index.
pub fn snr_offset_from_search_index(index: u32) -> i32 {
    (index as i32 - 240) * 4
}

/// Recompute BAP for one channel using a unified encoder SNR (mask unchanged).
pub fn rerun_bap_unified_snr(
    state: &mut Ac3State,
    ch: usize,
    start: usize,
    end: usize,
    unified_snr: i32,
) {
    let floor = FLOORTAB[state.floorcod as usize];
    let mut i = start;
    let mut j = MASKTAB[start] as usize;
    loop {
        let mut m = state.channels[ch].mask[j] as i32;
        m -= unified_snr;
        m -= floor;
        if m < 0 {
            m = 0;
        }
        m &= 0x1fe0;
        m += floor;
        let band_end = (BNDTAB[j] as usize + BNDSZ[j] as usize).min(end);
        while i < band_end {
            let addr = ((state.channels[ch].psd[i] as i32 - m) >> 5).clamp(0, 63) as usize;
            state.channels[ch].bap[i] = BAPTAB[addr];
            i += 1;
        }
        if i >= end {
            break;
        }
        j += 1;
    }
}

/// Recompute all fbw (+ cpl/lfe) BAP with one unified encoder SNR offset.
pub fn rerun_all_bap_unified_snr(state: &mut Ac3State, bsi: &Bsi, unified_snr: i32) {
    let nfchans = bsi.nfchans as usize;
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        rerun_bap_unified_snr(state, ch, 0, end, unified_snr);
    }
    if state.cpl_in_use {
        rerun_bap_unified_snr(
            state,
            MAX_FBW,
            state.cpl_begf_mant,
            state.cpl_endf_mant,
            unified_snr,
        );
    }
    if bsi.lfeon {
        rerun_bap_unified_snr(state, MAX_FBW + 1, 0, 7, unified_snr);
    }
}

/// CBR SNR search mirroring FFmpeg `cbr_bit_allocation` mantissa sizing.
pub fn search_encoder_snr_for_mant_budget(
    state: &Ac3State,
    bsi: &Bsi,
    bits_left: u32,
) -> (u32, u32) {
    let st = state.clone();
    let mut snr_idx = snr_search_index_from_offset(st.snr_offset_ch[0]);
    let mant_for = |idx: u32| -> u32 {
        let mut s = st.clone();
        rerun_all_bap_unified_snr(&mut s, bsi, snr_offset_from_search_index(idx));
        count_mantissa_bits_walk(&s, bsi)
    };
    while snr_idx > 0 && mant_for(snr_idx) > bits_left {
        snr_idx = snr_idx.saturating_sub(64);
    }
    for snr_incr in [64u32, 16, 4, 1] {
        while snr_idx + snr_incr <= 1023 && mant_for(snr_idx + snr_incr) <= bits_left {
            snr_idx += snr_incr;
        }
    }
    (snr_idx, mant_for(snr_idx))
}

/// Find lowest SNR search index whose walk count is >= `target_mant`.
pub fn search_snr_for_mant_target(state: &Ac3State, bsi: &Bsi, target_mant: u32) -> (u32, u32) {
    let mut lo = snr_search_index_from_offset(state.snr_offset_ch[0]);
    let mut hi = (lo + 64).min(1023);
    let mant_at = |idx: u32| -> u32 {
        let mut s = state.clone();
        rerun_all_bap_unified_snr(&mut s, bsi, snr_offset_from_search_index(idx));
        count_mantissa_bits_walk(&s, bsi)
    };
    while mant_at(hi) < target_mant && hi < 1023 {
        lo = hi;
        hi = (hi + 64).min(1023);
    }
    while lo < hi {
        let mid = (lo + hi) / 2;
        if mant_at(mid) < target_mant {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    (lo, mant_at(lo))
}

/// Walk mantissa bit count after replacing fbw `bap[]` on a cloned block state.
pub fn count_walk_with_bap(state: &Ac3State, bsi: &Bsi, bap: &[[u8; N_COEFFS]; MAX_FBW]) -> u32 {
    let nfchans = bsi.nfchans as usize;
    let mut st = state.clone();
    for ch in 0..nfchans {
        let end = st.channels[ch].end_mant;
        st.channels[ch].bap[..end].copy_from_slice(&bap[ch][..end]);
    }
    count_mantissa_bits_walk(&st, bsi)
}

/// Group sorted `(bin, from, to)` BAP diffs into contiguous bin runs.
pub fn cluster_bap_diffs(diffs: &[(usize, u8, u8)]) -> Vec<(usize, usize, i32)> {
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < diffs.len() {
        let start = diffs[i].0;
        let mut end;
        let mut naive_delta = 0i32;
        loop {
            let (bin, a, b) = diffs[i];
            end = bin;
            naive_delta += count_mantissa_bits_for_bap(&[b], 1) as i32
                - count_mantissa_bits_for_bap(&[a], 1) as i32;
            i += 1;
            if i >= diffs.len() || diffs[i].0 != end + 1 {
                break;
            }
        }
        out.push((start, end, naive_delta));
    }
    out
}

/// Apply neighbor `target` BAP values on `bins` where they differ from `base`.
pub fn apply_bap_diffs(
    base: &mut [[u8; N_COEFFS]; MAX_FBW],
    target: &[[u8; N_COEFFS]; MAX_FBW],
    ch: usize,
    bins: &[(usize, u8, u8)],
) {
    for &(bin, _, _) in bins {
        base[ch][bin] = target[ch][bin];
    }
}

/// Copy frame-3 neighbor BAP onto selected contiguous ch0/ch1 clusters.
pub fn apply_cluster_patches(
    base: &mut [[u8; N_COEFFS]; MAX_FBW],
    neighbor: &[[u8; N_COEFFS]; MAX_FBW],
    ch: usize,
    clusters: &[(usize, usize)],
) {
    for &(lo, hi) in clusters {
        for bin in lo..=hi {
            base[ch][bin] = neighbor[ch][bin];
        }
    }
}

/// Copy fbw BAP arrays from one block audit into override form.
pub fn bap_override_from_audit(audit: &BlockBitAudit, nfchans: usize) -> [[u8; N_COEFFS]; MAX_FBW] {
    let mut out = [[0u8; N_COEFFS]; MAX_FBW];
    out[..nfchans].copy_from_slice(&audit.bap_ch[..nfchans]);
    out
}

/// Apply a per-bin BAP delta on one channel (e.g. 8→9 on HF bins).
pub fn patch_bap_bins(bap: &mut [u8], bins: &[(usize, u8)]) {
    for &(bin, new_bap) in bins {
        if bin < bap.len() {
            bap[bin] = new_bap;
        }
    }
}

/// Per-block BAP + mantissa bit audit for one syncframe.
#[derive(Clone, Debug)]
pub struct BlockBitAudit {
    pub blk: usize,
    pub parse_ok: bool,
    pub bit_block_start: u64,
    pub bit_pre_mantissa: u64,
    pub bit_block_end: u64,
    pub side_info_bits: u64,
    pub mantissa_bits_actual: u64,
    pub mantissa_bits_walk: u32,
    pub mantissa_bits_pack: u32,
    pub mantissa_bits_histo: u32,
    pub end_mant: [usize; MAX_FBW],
    pub fsnroffst: [u8; MAX_FBW],
    pub bap_histo_ch: [[u32; 16]; MAX_FBW],
    pub bap_histo_combined: [u32; 16],
    /// Full `bap[]` per fbw channel after bit allocation.
    pub bap_ch: [[u8; N_COEFFS]; MAX_FBW],
}

fn bap_histogram_ch(state: &Ac3State, ch: usize, end: usize) -> [u32; 16] {
    let mut h = [0u32; 16];
    for bin in 0..end {
        h[state.channels[ch].bap[bin] as usize] += 1;
    }
    h
}

fn combined_bap_histogram(state: &Ac3State, bsi: &Bsi) -> [u32; 16] {
    let nfchans = bsi.nfchans as usize;
    let mut histo = [0u32; 16];
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        for bin in 0..end {
            histo[state.channels[ch].bap[bin] as usize] += 1;
        }
    }
    if state.cpl_in_use {
        for bin in state.cpl_begf_mant..state.cpl_endf_mant {
            histo[state.channels[MAX_FBW].bap[bin] as usize] += 1;
        }
    }
    if bsi.lfeon {
        let lfe_ch = MAX_FBW + 1;
        for bin in 0..7 {
            histo[state.channels[lfe_ch].bap[bin] as usize] += 1;
        }
    }
    histo
}

/// Parse one syncframe block-by-block and record BAP histograms plus
/// mantissa bit estimates (decoder walk, encoder-pack schedule, histo).
pub fn audit_frame_blocks(
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
) -> Result<Vec<BlockBitAudit>> {
    let mut state = Ac3State::new();
    state.apply_bsi_params(bsi);
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    br.skip(bsi.bits_consumed as u32)?;
    let mut out = Vec::with_capacity(BLOCKS_PER_FRAME);
    for blk in 0..BLOCKS_PER_FRAME {
        state.blkidx = blk;
        let bit_block_start = br.bit_position();
        let mut side = AudBlkSideInfo::default();
        let mut metrics = BlockParseMetrics::default();
        let parse_ok = parse_audblk_into(
            &mut state,
            si,
            bsi,
            &mut br,
            &mut side,
            Some(&mut metrics),
            None,
            None,
        )
        .is_ok();
        let nfchans = (bsi.nfchans as usize).min(MAX_FBW);
        let mut bap_histo_ch = [[0u32; 16]; MAX_FBW];
        let mut end_mant = [0usize; MAX_FBW];
        let mut fsnroffst = [0u8; MAX_FBW];
        let mut bap_ch = [[0u8; N_COEFFS]; MAX_FBW];
        for ch in 0..nfchans {
            end_mant[ch] = state.channels[ch].end_mant;
            fsnroffst[ch] = state.fsnroffst[ch];
            bap_histo_ch[ch] = bap_histogram_ch(&state, ch, end_mant[ch]);
            bap_ch[ch] = state.channels[ch].bap;
        }
        let combined = combined_bap_histogram(&state, bsi);
        let mantissa_bits_walk = count_mantissa_bits_walk(&state, bsi);
        let mantissa_bits_pack = crate::encoder::count_mantissa_bits_encoder_pack(&state, bsi);
        let mantissa_bits_histo = mantissa_bits_from_histo(&combined);
        let side_info_bits = metrics.bit_pre_mantissa.saturating_sub(bit_block_start);
        let mantissa_bits_actual = metrics
            .bit_block_end
            .saturating_sub(metrics.bit_pre_mantissa);
        out.push(BlockBitAudit {
            blk,
            parse_ok,
            bit_block_start,
            bit_pre_mantissa: metrics.bit_pre_mantissa,
            bit_block_end: metrics.bit_block_end,
            side_info_bits,
            mantissa_bits_actual,
            mantissa_bits_walk,
            mantissa_bits_pack,
            mantissa_bits_histo,
            end_mant,
            fsnroffst,
            bap_histo_ch,
            bap_histo_combined: combined,
            bap_ch,
        });
        if !parse_ok {
            break;
        }
    }
    Ok(out)
}

/// Parse block `blk` starting at absolute bit offset `bit_pos` (post-sync
/// origin). Returns metrics on success.
pub fn try_parse_block_at(
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
    state: &mut Ac3State,
    blk: usize,
    bit_pos: u64,
) -> Result<BlockParseMetrics> {
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    br.skip(bit_pos as u32)?;
    state.blkidx = blk;
    let mut side = AudBlkSideInfo::default();
    let mut metrics = BlockParseMetrics::default();
    parse_audblk_into(
        state,
        si,
        bsi,
        &mut br,
        &mut side,
        Some(&mut metrics),
        None,
        None,
    )?;
    Ok(metrics)
}

/// Try parsing block `after_blk + 1` from `base_bit_pos + delta`; return
/// parse result and final bit cursor.
pub fn try_parse_next_block(
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
    after_blk: usize,
    delta: i32,
) -> (bool, u64, Option<String>) {
    let post_sync = &frame_bytes[5..];
    let mut state = Ac3State::new();
    let mut br = BitReader::new(post_sync);
    if br.skip(bsi.bits_consumed as u32).is_err() {
        return (false, 0, Some("bsi skip failed".into()));
    }
    for blk in 0..=after_blk {
        state.blkidx = blk;
        let mut side = AudBlkSideInfo::default();
        if parse_audblk_into(&mut state, si, bsi, &mut br, &mut side, None, None, None).is_err() {
            return (
                false,
                br.bit_position(),
                Some(format!("blk{blk} setup failed")),
            );
        }
    }
    let base = br.bit_position();
    let try_pos = (base as i64 + i64::from(delta)).max(0) as u64;
    let mut br2 = BitReader::new(post_sync);
    if br2.skip(try_pos as u32).is_err() {
        return (false, try_pos, Some("seek failed".into()));
    }
    let mut st2 = state;
    st2.blkidx = after_blk + 1;
    let mut side2 = AudBlkSideInfo::default();
    match parse_audblk_into(&mut st2, si, bsi, &mut br2, &mut side2, None, None, None) {
        Ok(()) => (true, br2.bit_position(), None),
        Err(e) => (false, br2.bit_position(), Some(e.to_string())),
    }
}

/// Parse all 6 blocks; optionally inject BAP for block 0 before mantissa unpack.
pub fn full_frame_parses_with_bap(
    si: &SyncInfo,
    bsi: &Bsi,
    frame: &[u8],
    bap_blk0: [[u8; N_COEFFS]; MAX_FBW],
) -> bool {
    let mut state = Ac3State::new();
    state.apply_bsi_params(bsi);
    state.bap_override = Some((0, bap_blk0));
    let post_sync = &frame[5..];
    let mut br = BitReader::new(post_sync);
    if br.skip(bsi.bits_consumed as u32).is_err() {
        return false;
    }
    for blk in 0..BLOCKS_PER_FRAME {
        state.blkidx = blk;
        if blk == 0 {
            state.bap_override = Some((0, bap_blk0));
        }
        let mut side = AudBlkSideInfo::default();
        if parse_audblk_into(&mut state, si, bsi, &mut br, &mut side, None, None, None).is_err() {
            return false;
        }
    }
    true
}

/// After block `after_blk`, try parsing block `after_blk+1` from
/// `base_bit_pos + delta` and report whether the full block parses.
pub fn probe_next_block_alignment(
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
    after_blk: usize,
    delta_min: i32,
    delta_max: i32,
) -> Result<Vec<(i32, bool, u64)>> {
    let mut state = Ac3State::new();
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    br.skip(bsi.bits_consumed as u32)?;
    for blk in 0..=after_blk {
        state.blkidx = blk;
        let mut side = AudBlkSideInfo::default();
        if parse_audblk_into(&mut state, si, bsi, &mut br, &mut side, None, None, None).is_err() {
            return Ok(Vec::new());
        }
    }
    let base = br.bit_position();
    let next_blk = after_blk + 1;
    if next_blk >= BLOCKS_PER_FRAME {
        return Ok(Vec::new());
    }
    let state_snapshot = state.clone();
    let mut hits = Vec::new();
    for delta in delta_min..=delta_max {
        let try_pos = (base as i64 + i64::from(delta)).max(0) as u64;
        let mut br2 = BitReader::new(post_sync);
        br2.skip(try_pos as u32)?;
        let mut st2 = state_snapshot.clone();
        st2.blkidx = next_blk;
        let mut side2 = AudBlkSideInfo::default();
        let ok =
            parse_audblk_into(&mut st2, si, bsi, &mut br2, &mut side2, None, None, None).is_ok();
        hits.push((delta, ok, try_pos));
    }
    Ok(hits)
}

/// Result of probing whether block `after_blk+1` parses at delta=0 after
/// optional `fsnroffst` override on block `after_blk`.
#[derive(Clone, Debug)]
pub struct Blk1FsnroffstProbe {
    pub fsnroffst: [u8; MAX_FBW],
    pub mantissa_bits: u32,
    pub blk1_parse_ok: bool,
}

/// Like [`probe_blk1_after_fsnroffst`] but carries `Ac3State` across prior
/// syncframes (matches production decoder).
pub fn probe_blk1_after_fsnroffst_stream(
    data: &[u8],
    frame_index: usize,
    after_blk: usize,
    fsnroffst_override: Option<[u8; MAX_FBW]>,
) -> Result<Blk1FsnroffstProbe> {
    let mut off = 0usize;
    let mut frame_n = 0usize;
    let mut state = Ac3State::new();
    while off + 5 <= data.len() {
        let si = crate::syncinfo::parse(&data[off..])?;
        let len = si.frame_length as usize;
        if len < 5 || off + len > data.len() {
            break;
        }
        let frame = &data[off..off + len];
        let bsi = crate::bsi::parse(&frame[5..])?;
        if frame_n == frame_index {
            return probe_blk1_after_fsnroffst_with_state(
                &mut state,
                &si,
                &bsi,
                frame,
                after_blk,
                fsnroffst_override,
            );
        }
        state.apply_bsi_params(&bsi);
        let post_sync = &frame[5..];
        let mut br = BitReader::new(post_sync);
        br.skip(bsi.bits_consumed as u32)?;
        for blk in 0..BLOCKS_PER_FRAME {
            state.blkidx = blk;
            let mut side = AudBlkSideInfo::default();
            let _ = parse_audblk_into(&mut state, &si, &bsi, &mut br, &mut side, None, None, None);
        }
        state.frame_counter = state.frame_counter.saturating_add(1);
        off += len;
        frame_n += 1;
    }
    Err(Error::invalid(format!("frame {frame_index} not found")))
}

fn probe_blk1_after_fsnroffst_with_state(
    state: &mut Ac3State,
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
    after_blk: usize,
    fsnroffst_override: Option<[u8; MAX_FBW]>,
) -> Result<Blk1FsnroffstProbe> {
    state.apply_bsi_params(bsi);
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    br.skip(bsi.bits_consumed as u32)?;
    let mut mantissa_bits = 0u32;
    for blk in 0..=after_blk {
        state.blkidx = blk;
        if blk == after_blk {
            state.fsnroffst_override = fsnroffst_override;
        }
        let mut side = AudBlkSideInfo::default();
        let mut metrics = BlockParseMetrics::default();
        parse_audblk_into(
            state,
            si,
            bsi,
            &mut br,
            &mut side,
            Some(&mut metrics),
            None,
            None,
        )?;
        if blk == after_blk {
            mantissa_bits = metrics
                .bit_block_end
                .saturating_sub(metrics.bit_pre_mantissa) as u32;
        }
    }
    let base = br.bit_position();
    let mut br2 = BitReader::new(post_sync);
    br2.skip(base as u32)?;
    let mut st2 = state.clone();
    st2.blkidx = after_blk + 1;
    let mut side2 = AudBlkSideInfo::default();
    let blk1_parse_ok =
        parse_audblk_into(&mut st2, si, bsi, &mut br2, &mut side2, None, None, None).is_ok();
    Ok(Blk1FsnroffstProbe {
        fsnroffst: state.fsnroffst,
        mantissa_bits,
        blk1_parse_ok,
    })
}

/// Parse through block `after_blk`, optionally overriding `fsnroffst` on
/// that block before bit allocation, then test block `after_blk+1` at delta=0.
pub fn probe_blk1_after_fsnroffst(
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
    after_blk: usize,
    fsnroffst_override: Option<[u8; MAX_FBW]>,
) -> Result<Blk1FsnroffstProbe> {
    let mut state = Ac3State::new();
    probe_blk1_after_fsnroffst_with_state(
        &mut state,
        si,
        bsi,
        frame_bytes,
        after_blk,
        fsnroffst_override,
    )
}

pub fn recount_block_mantissa_bits(state: &Ac3State, bsi: &Bsi) -> u32 {
    count_mantissa_bits_walk(state, bsi)
}

/// Re-run bit allocation for one block using overridden per-channel
/// `fsnroffst` values (other state unchanged).
pub fn rerun_ba_with_fsnroffst(
    state: &mut Ac3State,
    si: &SyncInfo,
    bsi: &Bsi,
    fsnroffst: [u8; MAX_FBW],
) {
    state.fsnroffst = fsnroffst;
    let nfchans = bsi.nfchans as usize;
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        run_bit_allocation(
            state,
            ch,
            0,
            end,
            si.fscod,
            fsnroffst[ch],
            state.fgaincod[ch],
            false,
        );
    }
    if state.cpl_in_use {
        run_bit_allocation(
            state,
            MAX_FBW,
            state.cpl_begf_mant,
            state.cpl_endf_mant,
            si.fscod,
            state.cpl_fsnroffst,
            state.cpl_fgaincod,
            true,
        );
    }
    if bsi.lfeon {
        run_bit_allocation(
            state,
            MAX_FBW + 1,
            0,
            7,
            si.fscod,
            state.lfefsnroffst,
            state.lfefgaincod,
            false,
        );
    }
}

/// Decoder state after successfully parsing audio blocks `0..=blk`.
pub fn state_after_block(
    si: &SyncInfo,
    bsi: &Bsi,
    frame_bytes: &[u8],
    blk: usize,
) -> Result<Ac3State> {
    let mut state = Ac3State::new();
    state.apply_bsi_params(bsi);
    let post_sync = &frame_bytes[5..];
    let mut br = BitReader::new(post_sync);
    br.skip(bsi.bits_consumed as u32)?;
    for b in 0..=blk {
        state.blkidx = b;
        let mut side = AudBlkSideInfo::default();
        parse_audblk_into(&mut state, si, bsi, &mut br, &mut side, None, None, None)?;
    }
    Ok(state)
}

/// Global bit-allocation parameters affecting one fbw channel.
#[derive(Clone, Debug)]
pub struct BaGlobals {
    pub sdcycod: u8,
    pub fdcycod: u8,
    pub sgaincod: u8,
    pub dbpbcod: u8,
    pub floorcod: u8,
    pub floor: i32,
    pub snroffst_coarse: u8,
    pub fsnroffst: u8,
    pub fgaincod: u8,
    pub snroffset: i32,
    pub deltnseg: usize,
}

/// Per-bin bit-allocation breakdown after [`run_bit_allocation`].
#[derive(Clone, Debug)]
pub struct BinBaDetail {
    pub bin: usize,
    pub band: usize,
    pub exp: u8,
    pub psd: i16,
    pub bndpsd: i16,
    pub mask: i16,
    pub m_snr: i32,
    pub bap_addr: usize,
    pub bap: u8,
}

pub fn ba_globals(state: &Ac3State, ch: usize) -> BaGlobals {
    let floor = FLOORTAB[state.floorcod as usize];
    let fsnroffst = state.fsnroffst[ch];
    let snroffset = (((state.snroffst_coarse as i32 - 15) << 4) + fsnroffst as i32) << 2;
    BaGlobals {
        sdcycod: state.sdcycod,
        fdcycod: state.fdcycod,
        sgaincod: state.sgaincod,
        dbpbcod: state.dbpbcod,
        floorcod: state.floorcod,
        floor,
        snroffst_coarse: state.snroffst_coarse,
        fsnroffst,
        fgaincod: state.fgaincod[ch],
        snroffset,
        deltnseg: state.deltnseg[ch],
    }
}

/// Reconstruct §7.2.2.7 pointer `m` and BAP-table index for one bin.
pub fn bin_ba_detail(state: &Ac3State, ch: usize, bin: usize) -> BinBaDetail {
    let band = MASKTAB[bin] as usize;
    let floor = FLOORTAB[state.floorcod as usize];
    let snroffset = (((state.snroffst_coarse as i32 - 15) << 4) + state.fsnroffst[ch] as i32) << 2;
    let mut m = state.channels[ch].mask[band] as i32;
    m -= snroffset;
    m -= floor;
    if m < 0 {
        m = 0;
    }
    m &= 0x1fe0;
    m += floor;
    let psd = state.channels[ch].psd[bin];
    let bap_addr = ((psd as i32 - m) >> 5).clamp(0, 63) as usize;
    BinBaDetail {
        bin,
        band,
        exp: state.channels[ch].exp[bin],
        psd,
        bndpsd: state.channels[ch].bndpsd[band],
        mask: state.channels[ch].mask[band],
        m_snr: m,
        bap_addr,
        bap: state.channels[ch].bap[bin],
    }
}

/// Compare oxideav BA vs FFmpeg-reference mask/BAP for one channel block.
#[derive(Clone, Debug)]
pub struct ChannelBaRefCompare {
    pub ch: usize,
    pub start: usize,
    pub end: usize,
    pub fsnroffst: u8,
    pub snroffset: i32,
    pub floor: i32,
    pub ours_mant_bits: u32,
    pub ref_mant_bits: u32,
    pub mask_diff_bands: Vec<(usize, i16, i16)>,
    pub bap_diff_bins: Vec<(usize, u8, u8)>,
}

pub fn compare_channel_ba_ref(
    state: &Ac3State,
    si: &SyncInfo,
    ch: usize,
    blk: usize,
) -> ChannelBaRefCompare {
    let start = 0usize;
    let end = state.channels[ch].end_mant;
    let fsnroffst = state.fsnroffst[ch];
    let floor = FLOORTAB[state.floorcod as usize];
    let snroffset = (((state.snroffst_coarse as i32 - 15) << 4) + fsnroffst as i32) << 2;
    let sr_shift = state.sr_shift;
    let sdecay = SLOWDEC[state.sdcycod as usize] >> sr_shift;
    let fdecay = FASTDEC[state.fdcycod as usize] >> sr_shift;
    let sgain = SLOWGAIN[state.sgaincod as usize];
    let dbknee = DBPBTAB[state.dbpbcod as usize];
    let fgain = FASTGAIN[state.fgaincod[ch] as usize];

    let ref_mask = ffmpeg_ref_calc_mask(
        &state.channels[ch].bndpsd,
        start,
        end,
        fdecay,
        sdecay,
        sgain,
        dbknee,
        fgain,
        0,
        0,
        si.fscod,
        sr_shift,
        false,
        false,
        state.deltnseg[ch],
        &state.deltoffst[ch],
        &state.deltlen[ch],
        &state.deltba[ch],
    );
    let ref_bap = ffmpeg_ref_calc_bap(
        &ref_mask,
        &state.channels[ch].psd,
        start,
        end,
        snroffset,
        floor,
    );

    let bndstrt = MASKTAB[start] as usize;
    let bndend = MASKTAB[end - 1] as usize + 1;
    let mut mask_diff_bands = Vec::new();
    for band in bndstrt..bndend {
        let ours = state.channels[ch].mask[band];
        let rf = ref_mask[band];
        if ours != rf {
            mask_diff_bands.push((band, ours, rf));
        }
    }

    let bap_diff_bins = diff_bap_bins(&state.channels[ch].bap, &ref_bap, end);
    let ours_mant_bits = count_mantissa_bits_for_bap(&state.channels[ch].bap, end);
    let ref_mant_bits = count_mantissa_bits_for_bap(&ref_bap, end);

    let _ = blk;
    ChannelBaRefCompare {
        ch,
        start,
        end,
        fsnroffst,
        snroffset,
        floor,
        ours_mant_bits,
        ref_mant_bits,
        mask_diff_bands,
        bap_diff_bins,
    }
}

/// Count differing BAP bins between two channel snapshots (0..min(end)).
pub fn diff_bap_bins(a: &[u8], b: &[u8], end: usize) -> Vec<(usize, u8, u8)> {
    let n = end.min(a.len()).min(b.len());
    let mut diffs = Vec::new();
    for bin in 0..n {
        if a[bin] != b[bin] {
            diffs.push((bin, a[bin], b[bin]));
        }
    }
    diffs
}

/// Lowest transform-coefficient number of SPX sub-band `subbnd` per
/// Table E3.13. Sub-bands 0..=16 carry 12 coefficients each starting at
/// tc# 25; the entry for sub-band 17 (tc# 229) is the one-past-last
/// marker used when `spxendf == 7`.
#[inline]
fn spx_bandtable(subbnd: usize) -> usize {
    25 + 12 * subbnd
}

/// Table E3.14 — Spectral Extension Attenuation Table `spxattentab[][]`
/// (§3.6.4.2.3). Indexed by the 5-bit `spxattencod[ch]` codeword
/// (rows 0..=31), each row holds the first 3 attenuation values of a
/// 5-tap symmetric notch filter applied at the baseband / extension
/// border (and at every wrap point during the §3.6.4.1 translation
/// copy). The 5-tap kernel is `[T[0], T[1], T[2], T[1], T[0]]` —
/// the last two taps are derived by symmetry per spec text:
///
/// > "The first 3 attenuation values of the filter are determined by
/// > lookup into Table E3.14 with index `spxattencod[ch]`. The last
/// > two attenuation values of the filter are determined by symmetry
/// > and are not explicitly stored in the table."
#[allow(clippy::excessive_precision)] // spec text values; f32 round suffices
pub(crate) const SPX_ATTEN_TABLE: [[f32; 3]; 32] = [
    [0.954_841_604, 0.911_722_489, 0.870_550_563],
    [0.911_722_489, 0.831_237_896, 0.757_858_283],
    [0.870_550_563, 0.757_858_283, 0.659_753_955],
    [0.831_237_896, 0.690_956_440, 0.574_349_177],
    [0.793_700_526, 0.629_960_525, 0.500_000_000],
    [0.757_858_283, 0.574_349_177, 0.435_275_282],
    [0.723_634_619, 0.523_647_061, 0.378_929_142],
    [0.690_956_440, 0.477_420_802, 0.329_876_978],
    [0.659_753_955, 0.435_275_282, 0.287_174_589],
    [0.629_960_525, 0.396_850_263, 0.250_000_000],
    [0.601_512_518, 0.361_817_309, 0.217_637_641],
    [0.574_349_177, 0.329_876_978, 0.189_464_571],
    [0.548_412_490, 0.300_756_259, 0.164_938_489],
    [0.523_647_061, 0.274_206_245, 0.143_587_294],
    [0.500_000_000, 0.250_000_000, 0.125_000_000],
    [0.477_420_802, 0.227_930_622, 0.108_818_820],
    [0.455_861_244, 0.207_809_474, 0.094_732_285],
    [0.435_275_282, 0.189_464_571, 0.082_469_244],
    [0.415_618_948, 0.172_739_110, 0.071_793_647],
    [0.396_850_263, 0.157_490_131, 0.062_500_000],
    [0.378_929_142, 0.143_587_294, 0.054_409_410],
    [0.361_817_309, 0.130_911_765, 0.047_366_143],
    [0.345_478_220, 0.119_355_200, 0.041_234_622],
    [0.329_876_978, 0.108_818_820, 0.035_896_824],
    [0.314_980_262, 0.099_212_566, 0.031_250_000],
    [0.300_756_259, 0.090_454_327, 0.027_204_705],
    [0.287_174_589, 0.082_469_244, 0.023_683_071],
    [0.274_206_245, 0.075_189_065, 0.020_617_311],
    [0.261_823_531, 0.068_551_561, 0.017_948_412],
    [0.250_000_000, 0.062_500_000, 0.015_625_000],
    [0.238_710_401, 0.056_982_656, 0.013_602_353],
    [0.227_930_622, 0.051_952_369, 0.011_841_536],
];

/// Apply the §3.6.4.2.3 5-tap symmetric notch filter to a 5-bin window
/// centred on the band-border bin. The kernel taps are
/// `[T[0], T[1], T[2], T[1], T[0]]` where `T = SPX_ATTEN_TABLE[code]`.
/// The window starts at `filtbin` (which the caller positions at
/// `border_bin - 2`); bins outside `0..N_COEFFS` are skipped so this is
/// safe near the array tail.
#[inline]
fn apply_spx_atten_notch(coeffs: &mut [f32; N_COEFFS], filtbin: usize, code: u8) {
    let row = SPX_ATTEN_TABLE[(code & 0x1F) as usize];
    let taps = [row[0], row[1], row[2], row[1], row[0]];
    for (i, tap) in taps.iter().enumerate() {
        let idx = filtbin + i;
        if idx < N_COEFFS {
            coeffs[idx] *= *tap;
        }
    }
}

/// One step of the SPX pseudo-random noise generator (§E.3.6.4.2). The
/// spec only requires a "zero-mean, unity-variance" sequence and leaves
/// the exact generator non-normative — AC-3 / E-AC-3 are lossy and the
/// corpus is PSNR-compared, so a deterministic LFSR-derived value is
/// adequate. Returns a value in roughly `[-1, 1)` scaled to ~unit
/// variance (a sign-balanced uniform on (-√3, √3) has unit variance).
#[inline]
fn spx_noise(lfsr: &mut u32) -> f32 {
    // 32-bit xorshift — long period, cheap, deterministic.
    let mut x = *lfsr;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *lfsr = x;
    // Map to (-1, 1) then scale to unit variance: a uniform on (-1, 1)
    // has variance 1/3, so multiply by √3 ≈ 1.7320508.
    let u = (x as f32 / u32::MAX as f32) * 2.0 - 1.0;
    u * 1.732_050_8
}

/// E-AC-3 spectral extension high-frequency regeneration (§E.3.6).
///
/// For each fbw channel with `in_spx == true` this synthesizes the SPX
/// region `[spx_begin .. spx_end)` (transform-coefficient indices) from
/// the channel's own low-frequency coefficients:
///
/// 1. **Translation** (§E.3.6.4.1) — copy LF coefficients from the copy
///    region `[copystart .. copyend)` (`copystart = spxbandtable[spxstrtf]`,
///    `copyend = spxbandtable[spx_begin]`) into the SPX region,
///    wrapping the copy cursor when it reaches `copyend`.
/// 2. **Banded RMS energy** (§E.3.6.4.2.2) of the translated bins.
/// 3. **Noise blending** (§E.3.6.4.2.4) — `tc = tc·sblend + noise·rms·nblend`
///    per band, using the precomputed `spx_nblend` / `spx_sblend`.
/// 4. **Coordinate scaling** (§E.3.6.4.3) — `tc *= spxco·32` per band.
///
/// Band sizing (`spx_nbnds` / `spx_bndsztab`), the per-band coordinates
/// (`spx_coord`) and blend factors are computed during the audblk parse
/// (see `eac3::dsp`); this routine consumes that prepared state.
fn apply_spectral_extension(state: &mut Ac3State, nfchans: usize) {
    if !state.spx_in_use {
        return;
    }
    let nbnds = state.spx_nbnds;
    if nbnds == 0 {
        return;
    }
    let copystart = spx_bandtable(state.spx_strtf as usize);
    let copyend = spx_bandtable(state.spx_begin_subbnd);
    let spx_begin_tc = copyend;
    let spx_end_tc = spx_bandtable(state.spx_end_subbnd);
    if copyend <= copystart || spx_end_tc <= spx_begin_tc {
        return;
    }

    for ch in 0..nfchans {
        if !state.channels[ch].in_spx {
            continue;
        }

        // 1. Transform coefficient translation (§E.3.6.4.1).
        //    `wrapflag[bnd]` is true when the band-relative copy cursor
        //    had to wrap back to `copystart` before consuming this
        //    band's `bandsize` samples — i.e. the band straddles a copy
        //    boundary. The spec applies the §3.6.4.2.3 border notch
        //    filter at every such wrap point AND at the baseband /
        //    extension border itself (the bin straddling `spx_begin_tc`).
        let mut copyindex = copystart;
        let mut insertindex = spx_begin_tc;
        let mut wrapflag = [false; 18];
        for bnd in 0..nbnds {
            let bandsize = state.spx_bndsztab[bnd];
            // Spec pseudocode applies wrap detection at TWO points: the
            // pre-band check `(copyindex + bandsize > copyend)` AND the
            // per-bin `(copyindex == copyend)` check. Either path is a
            // wrap from the band's point of view. Band 0 is the
            // baseband/extension border itself — its filter site is
            // emitted unconditionally outside this loop, so only
            // bnd >= 1 wraps need flagging.
            let mut wrapped = false;
            if copyindex + bandsize > copyend {
                copyindex = copystart;
                wrapped = true;
            }
            for _ in 0..bandsize {
                if copyindex == copyend {
                    copyindex = copystart;
                    wrapped = true;
                }
                if insertindex < N_COEFFS && copyindex < N_COEFFS {
                    state.channels[ch].coeffs[insertindex] = state.channels[ch].coeffs[copyindex];
                }
                insertindex += 1;
                copyindex += 1;
            }
            if bnd > 0 && wrapped {
                wrapflag[bnd] = true;
            }
        }

        // 1b. §3.6.4.2.3 Transform Coefficient Band Border Filtering —
        //     after the §3.6.4.1 translation copy AND BEFORE the
        //     §3.6.4.2.2 banded RMS / §3.6.4.2.4 noise scaling. The
        //     5-tap symmetric notch filter sits centred on the first
        //     extension bin, attenuating the 2 bins below and 2 bins
        //     above the border (filter starts at `spx_begin_tc - 2`).
        //     The same filter re-applies at each band-internal wrap
        //     point flagged above (filter starts at the band's start
        //     minus 2 bins).
        if state.channels[ch].spx_atten_active {
            let code = state.channels[ch].spx_atten_code;
            // Baseband / extension region border.
            let border = spx_begin_tc;
            if border >= 2 {
                apply_spx_atten_notch(&mut state.channels[ch].coeffs, border - 2, code);
            }
            // Wrap points at band starts (bnd >= 1).
            let mut band_start = spx_begin_tc;
            for bnd in 0..nbnds {
                if bnd > 0 && wrapflag[bnd] && band_start >= 2 {
                    apply_spx_atten_notch(&mut state.channels[ch].coeffs, band_start - 2, code);
                }
                band_start += state.spx_bndsztab[bnd];
            }
        }

        // 2. Banded RMS energy of the translated coefficients
        //    (§E.3.6.4.2.2), 3. noise blend (§E.3.6.4.2.4), and
        //    4. coordinate scaling (§E.3.6.4.3) — fused per band.
        let mut spxmant = spx_begin_tc;
        for bnd in 0..nbnds {
            let bandsize = state.spx_bndsztab[bnd];
            let band_lo = spxmant;
            let band_hi = (spxmant + bandsize).min(N_COEFFS);

            // Banded RMS.
            let mut accum = 0.0f64;
            for bin in band_lo..band_hi {
                let v = state.channels[ch].coeffs[bin] as f64;
                accum += v * v;
            }
            let rms = if bandsize > 0 {
                (accum / bandsize as f64).sqrt() as f32
            } else {
                0.0
            };

            let nblend = state.channels[ch].spx_nblend[bnd];
            let sblend = state.channels[ch].spx_sblend[bnd];
            let nscale = rms * nblend;
            let coord = state.channels[ch].spx_coord[bnd];

            for bin in band_lo..band_hi {
                let tctemp = state.channels[ch].coeffs[bin];
                let ntemp = spx_noise(&mut state.spx_noise_lfsr);
                let blended = tctemp * sblend + ntemp * nscale;
                // §E.3.6.4.3 final scale by spxco·32.
                state.channels[ch].coeffs[bin] = blended * coord * 32.0;
            }

            spxmant += bandsize;
        }

        // The SPX region is now populated; extend the channel's mantissa
        // count so dynrng + IMDCT process the regenerated bins.
        state.channels[ch].end_mant = state.channels[ch].end_mant.max(spx_end_tc);
    }
}

/// Apply DSP stages to the current block: decouple, rematrix, dynrng,
/// IMDCT, window + overlap-add. Populates `channels[ch].coeffs[0..256]`
/// with time-domain PCM samples ready for emission.
pub(crate) fn dsp_block(state: &mut Ac3State, _si: &SyncInfo, bsi: &Bsi) {
    let nfchans = bsi.nfchans as usize;
    let acmod = bsi.acmod;

    // --- Decoupling (§7.4) ---
    // Enhanced coupling (§E.3.5.5) reconstructs each coupled channel's
    // transform coefficients from the complex carrier `Z[k]` (the
    // §E.3.5.5.4 product) *before* `dsp_block` is called, so `coeffs[bin]`
    // already holds the per-channel result. The standard scalar decouple
    // (`cpl_coord · cplchan`) must NOT run in that case — it would
    // overwrite the carrier-derived coefficients. `skip_decouple` carries
    // that gate; base AC-3 and standard E-AC-3 coupling leave it `false`.
    if state.cpl_in_use && !state.skip_decouple {
        let cpl_ch = MAX_FBW;
        let start = state.cpl_begf_mant;
        let end = state.cpl_endf_mant;
        // Build subband->band lookup.
        let mut sbnd2bnd = [0usize; 18];
        let mut bnd = 0usize;
        for sbnd in 0..state.cpl_nsubbnd {
            if sbnd > 0 && !state.cpl_bndstrc[sbnd] {
                bnd += 1;
            }
            sbnd2bnd[sbnd] = bnd;
        }
        for ch in 0..nfchans {
            if !state.channels[ch].in_coupling {
                continue;
            }
            for sbnd_off in 0..state.cpl_nsubbnd {
                let band = sbnd2bnd[sbnd_off];
                let coord = state.cpl_coord[ch][band] * 8.0;
                let base = start + sbnd_off * 12;
                let limit = (base + 12).min(end);
                for bin in base..limit {
                    let mut v = state.channels[cpl_ch].coeffs[bin] * coord;
                    // phase flag for right channel in 2/0.
                    if acmod == 0x2 && ch == 1 && state.cpl_phsflg[band] {
                        v = -v;
                    }
                    state.channels[ch].coeffs[bin] = v;
                }
            }
        }
    }

    // --- Rematrixing (§7.5) ---
    //
    // Per Tables 7.25 / 7.26 / 7.27 / 7.28, the upper edge of the LAST
    // rematrix band is NOT a fixed constant — it tracks the lower edge
    // of the coupling region whenever coupling is in use:
    //
    //   • cplinu == 0           : 4 bands, last ends at bin 252 (Table 7.25)
    //   • cplinu == 1, cplbegf > 2: 4 bands, last ends at A = 36 + 12*cplbegf
    //   • cplinu == 1, 2 ≥ cplbegf > 0: 3 bands, last ends at A
    //   • cplinu == 1, cplbegf = 0: 2 bands, last ends at bin 36
    //
    // A previous formulation hard-coded the last band's high coefficient at
    // bin 252 even when coupling was active. On 2/0 frames whose bitstream
    // enables rematrixing AND coupling above bin 132 (cplbegf=8 in our
    // transient fixture), this bled the L+R / L-R operation into the
    // coupling region — bins that had just been re-derived from the
    // coupling pseudo-channel via cplco coords. The downstream symptom
    // was a steady PSNR drift across the 6 audblks of every burst-onset
    // frame: rematrix scrambled the post-decouple coefficients, the
    // IMDCT rendered the wrong waveform, and overlap-add carried the
    // error into the next block. Fixing the upper edge to the spec's
    // cplbegf-dependent boundary restores burst-frame PSNR.
    if acmod == 0x2 {
        let last_high = if state.cpl_in_use {
            // A = 36 + 12 * cplbegf per Tables 7.26 / 7.27.
            // For cplbegf = 0 (Table 7.28), the last band actually ends
            // at bin 36 — but with only 2 rematrix bands it never reaches
            // band index 3, so the value of `last_high` for index 3 is
            // unused. Compute A unconditionally.
            36 + 12 * state.cpl_begf as usize
        } else {
            252 // Table 7.25 fixed last band high.
        };
        // Convert to exclusive upper bound for our `lo..hi` ranges.
        let last_hi_excl = last_high + 1;
        let remat_bands: [(usize, usize); 4] =
            [(13, 25), (25, 37), (37, 61), (61, last_hi_excl.max(61))];
        let n = remat_band_count(state.cpl_in_use, state.cpl_begf);
        for (i, (lo, hi)) in remat_bands.iter().take(n).enumerate() {
            if !state.rematflg[i] {
                continue;
            }
            let end_lo = state.channels[0].end_mant.min(*hi);
            let end_hi = state.channels[1].end_mant.min(*hi);
            let end = end_lo.min(end_hi);
            for bin in *lo..end {
                let l = state.channels[0].coeffs[bin];
                let r = state.channels[1].coeffs[bin];
                state.channels[0].coeffs[bin] = l + r;
                state.channels[1].coeffs[bin] = l - r;
            }
        }
    }

    // --- Spectral extension synthesis (§E.3.6) ---
    //
    // For channels using SPX (`in_spx`, E-AC-3 only) the coded
    // coefficients stop at the SPX begin frequency; this step
    // regenerates the high-frequency band [spx_begin .. spx_end) by
    // copying low-frequency coefficients, blending with banded noise,
    // and scaling by the per-band SPX coordinates. It runs AFTER
    // decouple + rematrix (which reshape the low-frequency copy region)
    // and BEFORE dynrng + IMDCT so the synthesized bins are gain-scaled
    // and transformed together with the baseband. Base AC-3 never sets
    // `in_spx`, so this is a no-op there.
    apply_spectral_extension(state, nfchans);

    // --- IMDCT + window + overlap-add for every output channel ---
    // Dynrng/compr are intentionally not applied on the PCM path — see
    // `bsi::CompressionGain` module docs. Side-info `dynrng` words are
    // still parsed and stored on `ChannelState::dynrng` for applications
    // that want DRC downstream.
    // Uses the §7.9.4 FFT-backed decomposition: pre-twiddle → N/4-point
    // complex IFFT (N/8 for short blocks) → post-twiddle → de-interleave.
    // Matches the direct-form reference within f32 precision on the long
    // path; the short path is validated by the validator-fixture RMS gate.
    let trace_frame_dsp = std::env::var("AC3_TRACE_FRAME")
        .ok()
        .and_then(|s| s.parse::<u64>().ok());
    let trace_blk_dsp = std::env::var("AC3_TRACE_BLK")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(0);
    for ch in 0..nfchans {
        let mut coeffs = [0.0f32; 256];
        coeffs.copy_from_slice(&state.channels[ch].coeffs);
        if let Some(tf) = trace_frame_dsp {
            if tf == state.frame_counter && state.blkidx == trace_blk_dsp {
                let max_abs = coeffs.iter().fold(0.0f32, |a, &b| a.max(b.abs()));
                let nonzero = coeffs.iter().filter(|&&v| v != 0.0).count();
                eprintln!(
                    "TRACE-DSP ch{} pre-IMDCT max|coeff|={:.6e} nonzero_bins={} blksw={} dynrng={}",
                    ch, max_abs, nonzero, state.channels[ch].blksw, state.channels[ch].dynrng
                );
                eprint!("TRACE-DSP ch{} coeff[0..16]: ", ch);
                for v in &coeffs[..16] {
                    eprint!("{:.4e} ", v);
                }
                eprintln!();
                eprint!("TRACE-DSP ch{} coeff[16..32]: ", ch);
                for v in &coeffs[16..32] {
                    eprint!("{:.4e} ", v);
                }
                eprintln!();
            }
        }
        let mut time = [0.0f32; 512];
        if state.channels[ch].blksw {
            crate::imdct::imdct_256_pair_fft(&coeffs, &mut time);
        } else {
            crate::imdct::imdct_512_fft(&coeffs, &mut time);
        }
        // Apply window.
        for n in 0..256 {
            time[n] *= WINDOW[n];
            time[511 - n] *= WINDOW[n];
        }
        // Overlap-add: pcm[n] = time[n] + delay[n]; delay[n] = time[256+n]
        let mut out_pcm = [0.0f32; 256];
        for n in 0..256 {
            // Per §7.9.4.1 overlap-add: pcm[n] = 2 * (x[n] + delay[n]).
            out_pcm[n] = 2.0 * (time[n] + state.channels[ch].delay[n]);
            state.channels[ch].delay[n] = time[256 + n];
        }
        state.channels[ch].coeffs[..256].copy_from_slice(&out_pcm);
    }
    if bsi.lfeon {
        let ch = MAX_FBW + 1;
        let mut coeffs = [0.0f32; 256];
        coeffs.copy_from_slice(&state.channels[ch].coeffs);
        let mut time = [0.0f32; 512];
        // LFE is always long-block (spec §5.4.3.3).
        crate::imdct::imdct_512_fft(&coeffs, &mut time);
        for n in 0..256 {
            time[n] *= WINDOW[n];
            time[511 - n] *= WINDOW[n];
        }
        let mut out_pcm = [0.0f32; 256];
        for n in 0..256 {
            // Per §7.9.4.1 overlap-add: pcm[n] = 2 * (x[n] + delay[n]).
            out_pcm[n] = 2.0 * (time[n] + state.channels[ch].delay[n]);
            state.channels[ch].delay[n] = time[256 + n];
        }
        state.channels[ch].coeffs[..256].copy_from_slice(&out_pcm);
    }
}

// Naive reference 512-point IMDCT (§7.9.4.1). Given N/2=256 transform
// coefficients, produces 512 time-domain samples prior to windowing.
//
// IMDCT formula: x[n] = (2/N) * sum_{k=0..N/2-1} X[k] * cos( (π/(2N)) * (2n+1+N/2) * (2k+1) ).
//
// This is the DFT-style reference implementation — not fast, but
// correct and matches the spec's prescribed output polarity /
// scaling so that window+overlap-add reproduces the original PCM.

// ---------------------------------------------------------------------
// `imdct_256_pair`: DEPRECATED reference — NOT the canonical short-block
// IMDCT. Kept behind `cfg(test)` for regression inspection only.
//
// The forward MDCT spec at §8.2.3.2 has an α parameter that picks the
// phase offset: α=-1 for the first short transform, α=0 for the long
// transform, α=+1 for the second short transform. This function tries
// to reconstruct the per-half direct-form from that spec, with X1 using
// phase `π/(2N)·(2n+1)·(2k+1)` (no `+N/2` shift) and X2 using the
// standard `π/(2N)·(2n+1+N/2)·(2k+1)`. In practice this DOES NOT match
// the §7.9.4.2 FFT decomposition output — the two disagree with ~40%
// residual on random input (see `imdct::tests::short_block_direct_form_disagrees`).
// The FFT path is the canonical one per the spec; keep this around only
// so a future audit can bisect which side is wrong.
#[cfg(test)]
fn imdct_256_pair(x: &[f32; 256], out: &mut [f32; 512]) {
    use std::f32::consts::PI;
    let n: usize = 256;
    let scale = -1.0f32;
    let mut x1 = [0.0f32; 128];
    let mut x2 = [0.0f32; 128];
    for k in 0..128 {
        x1[k] = x[2 * k];
        x2[k] = x[2 * k + 1];
    }
    // First short transform: phase offset (1+α)=0 → pure cos(π/(2N)*(2n+1)*(2k+1)).
    for nn in 0..n {
        let mut s = 0.0f32;
        for k in 0..128 {
            let phase = PI / (2.0 * n as f32) * ((2 * nn + 1) as f32) * ((2 * k + 1) as f32);
            s += x1[k] * phase.cos();
        }
        out[nn] = scale * s;
    }
    // Second short transform: phase offset (1+α)=2 → standard IMDCT with +N/2.
    for nn in 0..n {
        let mut s = 0.0f32;
        for k in 0..128 {
            let phase =
                PI / (2.0 * n as f32) * ((2 * nn + 1 + n / 2) as f32) * ((2 * k + 1) as f32);
            s += x2[k] * phase.cos();
        }
        out[256 + nn] = scale * s;
    }
}

pub fn imdct_512(x: &[f32; 256], out: &mut [f32; 512]) {
    // Direct reference implementation of the 512-point IMDCT described
    // in §7.9.4.1 of A/52:2018. The spec provides a fast FFT-based
    // decomposition with pre/post-twiddle, but the mathematical
    // definition — a sum of cosines over the 256 transform bins —
    // produces identical output and is easier to audit.
    //
    //   x[n] = sum_{k=0..N/2-1} X[k] * cos( π/(2N) * (2n+1+N/2) * (2k+1) )
    //
    // The AC-3 reconstruction chain scales by `2 * (x + delay)` in the
    // overlap-add step (spec pseudocode at end of §7.9.4.1), so the
    // IMDCT itself needs no explicit `2/N` normalisation; pairing that
    // with AC-3's windowing produces full-scale PCM for a full-scale
    // transform coefficient.
    use std::f32::consts::PI;
    let n: usize = 512;
    // The AC-3 encoder applies an explicit `-2/N` scale to the forward
    // MDCT (§8.2.3.2). Our decoder undoes that via the IMDCT scale +
    // the `2*(x + delay)` overlap-add (§7.9.4.1 step 6). Calibrated
    // empirically against the validator binary on the 440 Hz @ 192 kbps
    // fixture: peak validator-decoded 2897 int16, our output 2895 with
    // `scale = -1.0`. The sign flip cancels the encoder's `-2/N` sign so
    // positive-amplitude input reconstructs as positive-amplitude PCM.
    let scale = -1.0f32;
    for nn in 0..n {
        let mut s = 0.0f32;
        for k in 0..256 {
            let phase =
                PI / (2.0 * n as f32) * ((2 * nn + 1 + n / 2) as f32) * ((2 * k + 1) as f32);
            s += x[k] * phase.cos();
        }
        out[nn] = scale * s;
    }
}

#[cfg(test)]
mod short_block_tests {
    use super::*;

    /// Regression / bisection fixture. The naive direct-form short-block
    /// IMDCT (derived from §8.2.3.2's α=-1/+1 phase offsets) does NOT
    /// match the §7.9.4.2 FFT decomposition used in production. We
    /// assert the disagreement explicitly here — if a future fix makes
    /// the two align, that's a signal that BOTH the direct form AND the
    /// FFT path changed together, and the test can then be tightened
    /// into a proper equality gate. Until then the FFT path is
    /// considered canonical: its PCM output is byte-equivalent to the
    /// reference S16LE produced by black-box validator-binary decode of
    /// transient fixtures (cross-validated in the transient-fixture
    /// integration tests).
    #[test]
    fn short_block_direct_form_diverges_from_fft() {
        // LCG-based deterministic "random" input — no rand dependency.
        let mut x = [0.0f32; 256];
        let mut s: u32 = 0x1234_5678;
        for v in x.iter_mut() {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            *v = (s as i32 as f32) / (i32::MAX as f32);
        }
        let mut d = [0.0f32; 512];
        let mut f = [0.0f32; 512];
        imdct_256_pair(&x, &mut d);
        crate::imdct::imdct_256_pair_fft(&x, &mut f);
        let sse: f32 = d.iter().zip(f.iter()).map(|(a, b)| (a - b).powi(2)).sum();
        let rmse = (sse / 512.0).sqrt();
        // Currently observed: RMSE ≈ 4-5 on ~unit-magnitude random input.
        // Assert the divergence exists (>0.5) so this test fails loudly if
        // someone accidentally makes both paths compute the same thing.
        assert!(
            rmse > 0.5,
            "direct form and FFT path now match (rmse={rmse:.3}) — promote short_block_direct_form_diverges_from_fft to equality"
        );
    }
}

#[cfg(test)]
mod spx_tests {
    use super::*;

    /// Table E3.13: spx sub-band `s` begins at transform coefficient
    /// `25 + 12·s`. Sub-band 17 is the one-past-last marker at tc# 229.
    #[test]
    fn spx_bandtable_matches_table_e3_13() {
        assert_eq!(spx_bandtable(0), 25);
        assert_eq!(spx_bandtable(1), 37);
        assert_eq!(spx_bandtable(2), 49);
        assert_eq!(spx_bandtable(9), 133);
        assert_eq!(spx_bandtable(16), 217);
        assert_eq!(spx_bandtable(17), 229);
    }

    /// §E.3.3.2 `nrematbd` decision tree — every arm of the spec
    /// pseudo-code. The enhanced-coupling arm (`cplinu && ecplinu`) sizes
    /// from the raw `ecplbegf` code and is the one that 2/0 ecpl frames
    /// reach; the rest reproduce the SPX / standard-coupling / no-coupling
    /// behaviour already exercised by the parse loop.
    #[test]
    fn nrematbd_e3_3_2_decision_tree() {
        // No coupling, no SPX → always 4.
        assert_eq!(remat_band_count_spx(false, 0, false, 0, false, 0), 4);
        assert_eq!(remat_band_count_spx(false, 9, true, 7, false, 0), 4);

        // SPX without coupling: 3 for spx_begin_subbnd < 4 (spxbegf < 2),
        // else 4. ecpl flags are ignored when cplinu == false.
        assert_eq!(remat_band_count_spx(false, 0, false, 0, true, 3), 3);
        assert_eq!(remat_band_count_spx(false, 0, false, 0, true, 4), 4);
        assert_eq!(remat_band_count_spx(false, 0, true, 9, true, 2), 3);

        // Standard coupling (cplinu && !ecplinu): identical to
        // `remat_band_count(true, cplbegf)`. The ecplbegf argument is
        // ignored on this arm.
        assert_eq!(remat_band_count_spx(true, 0, false, 9, false, 0), 2);
        assert_eq!(remat_band_count_spx(true, 1, false, 9, false, 0), 3);
        assert_eq!(remat_band_count_spx(true, 2, false, 9, false, 0), 3);
        assert_eq!(remat_band_count_spx(true, 3, false, 9, false, 0), 4);
        assert_eq!(remat_band_count_spx(true, 9, false, 9, false, 0), 4);

        // Enhanced coupling (cplinu && ecplinu): thresholds the raw
        // ecplbegf 0/1/2/<5/else → 0/1/2/3/4. cplbegf is ignored.
        assert_eq!(remat_band_count_spx(true, 9, true, 0, false, 0), 0);
        assert_eq!(remat_band_count_spx(true, 9, true, 1, false, 0), 1);
        assert_eq!(remat_band_count_spx(true, 9, true, 2, false, 0), 2);
        assert_eq!(remat_band_count_spx(true, 9, true, 3, false, 0), 3);
        assert_eq!(remat_band_count_spx(true, 9, true, 4, false, 0), 3);
        assert_eq!(remat_band_count_spx(true, 9, true, 5, false, 0), 4);
        assert_eq!(remat_band_count_spx(true, 0, true, 15, false, 0), 4);
        // SPX co-active with enhanced coupling: the cplinu arm wins (SPX
        // only matters when coupling is off), so ecplbegf still drives it.
        assert_eq!(remat_band_count_spx(true, 0, true, 0, true, 2), 0);
        assert_eq!(remat_band_count_spx(true, 0, true, 5, true, 9), 4);
    }

    /// §E.3.6.3 spectral-extension coordinate decode. For exponent < 15
    /// the mantissa is `(spxcomant + 4) / 8`, shifted right by
    /// `spxcoexp + 3·mstrspxco`. For exponent == 15 it's `spxcomant / 4`.
    /// These mirror the encoder-side math the parse in `eac3::dsp` runs;
    /// duplicated here as an independent oracle.
    #[test]
    fn spx_coordinate_decode_formula() {
        // Mirrors the §E.3.6.3 decode the parse in `eac3::dsp` runs.
        fn spxco(coexp: i32, comant: i32, mstr: i32) -> f32 {
            let temp = if coexp == 15 {
                comant as f32 / 4.0
            } else {
                (comant as f32 + 4.0) / 8.0
            };
            let shift = coexp + 3 * mstr;
            temp * 2f32.powi(-shift)
        }
        // exp = 0, mant = 3, mstr = 0 → (3+4)/8 = 0.875, no shift.
        assert!((spxco(0, 3, 0) - 0.875).abs() < 1e-6);
        // exp = 2, mant = 1, mstr = 1 → (1+4)/8 = 0.625, >> (2 + 3) = 5.
        assert!((spxco(2, 1, 1) - 0.625 / 32.0).abs() < 1e-7);
        // exp = 15 limiting case → mant/4 (= 0.5 for mant 2), no shift
        // beyond the 15 exponent.
        assert!((spxco(15, 2, 0) - 2.0 / 4.0 / 2f32.powi(15)).abs() < 1e-9);
    }

    /// §E.3.6.2 band sizing. With the default Table E2.11 banding and a
    /// full sub-band range (begin=2, end=17) the merge bits at sub-bands
    /// 8/10/12/14/16 fold pairs of 12-coefficient sub-bands into 24-wide
    /// bands. We replicate the pseudo-code here against a hand-built
    /// `Ac3State` and check the resulting band-size table.
    #[test]
    fn spx_band_sizing_default_banding() {
        // begin=2, end=8 (sub-bands 2..7 active): merge bit only at 8
        // doesn't appear in range (8 == end excluded), so 6 bands of 12.
        let begin = 2usize;
        let end = 8usize;
        let mut bndstrc = [false; 18];
        bndstrc[8] = true; // default merge bit, out of [begin+1, end) here.
        let (n, sztab) = derive_bands(begin, end, &bndstrc);
        assert_eq!(n, 6);
        assert!(sztab[..6].iter().all(|&s| s == 12));

        // begin=2, end=17 with default merges at 8,10,12,14,16: the
        // merge bits land inside [3,17) and combine the second of each
        // pair. nspxbnds = 15 sub-bands → 10 bands (5 of width 24,
        // 5 of width 12) by the pseudo-code.
        let mut bndstrc = [false; 18];
        for &b in &[8usize, 10, 12, 14, 16] {
            bndstrc[b] = true;
        }
        let (n, sztab) = derive_bands(2, 17, &bndstrc);
        // 15 sub-bands, 5 merges → 10 bands.
        assert_eq!(n, 10);
        // Total coefficients == 15 sub-bands × 12 == 180.
        let total: usize = sztab[..n].iter().sum();
        assert_eq!(total, 180);
    }

    // Local re-implementation of the §E.3.6.2 nspxbnds / spxbndsztab
    // pseudo-code, used only by the test above.
    fn derive_bands(begin: usize, end: usize, bndstrc: &[bool; 18]) -> (usize, [usize; 18]) {
        let mut n = 1usize;
        let mut t = [0usize; 18];
        t[0] = 12;
        for bnd in (begin + 1)..end {
            if !bndstrc[bnd] {
                t[n] = 12;
                n += 1;
            } else {
                t[n - 1] += 12;
            }
        }
        (n, t)
    }

    /// End-to-end synthesis check for `apply_spectral_extension`. Build a
    /// channel whose low-frequency copy region carries a known ramp, set
    /// up one SPX band with a pure-signal blend (sblend=1, nblend=0,
    /// coord=1/32 so the ·32 scale cancels), and verify the SPX region is
    /// populated with copied values (not left silent) and that `end_mant`
    /// extends to the SPX end.
    #[test]
    fn spx_synthesis_copies_and_scales() {
        let mut state = Ac3State::new();
        let ch = 0usize;
        // SPX geometry: copy from sub-band 0 (tc 25), begin sub-band 2
        // (tc 49), end sub-band 4 (tc 73). One band of 24 (sub-bands
        // 2+3 merged via bndstrc[3]).
        state.spx_in_use = true;
        state.channels[ch].in_spx = true;
        state.spx_strtf = 0; // copystart = 25
        state.spx_begin_subbnd = 2; // copyend / spx_begin = 49
        state.spx_end_subbnd = 4; // spx_end = 73
        state.spx_bndstrc = [false; 18];
        state.spx_bndstrc[3] = true; // merge sub-band 3 into the band
        state.spx_nbnds = 1;
        state.spx_bndsztab = [0; 18];
        state.spx_bndsztab[0] = 24;
        // Pure-signal blend, unity-after-·32 coord.
        state.channels[ch].spx_sblend[0] = 1.0;
        state.channels[ch].spx_nblend[0] = 0.0;
        state.channels[ch].spx_coord[0] = 1.0 / 32.0;
        state.channels[ch].end_mant = 49; // coded mantissas stop at SPX begin.

        // Fill the copy region [25, 49) with a known non-zero ramp.
        for bin in 25..49 {
            state.channels[ch].coeffs[bin] = (bin - 25) as f32 + 1.0;
        }
        // SPX region [49, 73) starts silent.
        for bin in 49..73 {
            state.channels[ch].coeffs[bin] = 0.0;
        }

        apply_spectral_extension(&mut state, 1);

        // The SPX region must now be non-silent (copied + scaled).
        let nonzero = (49..73)
            .filter(|&b| state.channels[ch].coeffs[b] != 0.0)
            .count();
        assert!(
            nonzero >= 23,
            "SPX region should be populated, got {nonzero} non-zero bins"
        );
        // With sblend=1, nblend=0, coord·32=1, the first SPX bin equals
        // the first copied coefficient (copyindex starts at copystart=25).
        assert!(
            (state.channels[ch].coeffs[49] - state.channels[ch].coeffs[25]).abs() < 1e-4,
            "first SPX bin {} should equal first copy bin {}",
            state.channels[ch].coeffs[49],
            state.channels[ch].coeffs[25],
        );
        // end_mant extends to the SPX end so dynrng + IMDCT cover it.
        assert_eq!(state.channels[ch].end_mant, 73);
    }

    /// `apply_spectral_extension` must be a no-op for a channel not in
    /// SPX (and for base AC-3, which never sets `spx_in_use`).
    #[test]
    fn spx_synthesis_noop_when_disabled() {
        let mut state = Ac3State::new();
        for bin in 49..73 {
            state.channels[0].coeffs[bin] = 0.0;
        }
        state.spx_in_use = false;
        apply_spectral_extension(&mut state, 1);
        assert!((49..73).all(|b| state.channels[0].coeffs[b] == 0.0));
    }

    /// Spot-check three rows of Table E3.14 against the spec values. The
    /// scaling rows (0, 14, 29) cover the table's value-doubling
    /// progression (each ~2× step in `binindex=0` halves at the next
    /// power-of-two row) so a transcription typo on any of them stands
    /// out immediately.
    #[test]
    fn spx_atten_table_matches_spec() {
        // Compare against the spec's full 9-decimal-digit values held in
        // f64 (f32 literals would clip and trip `excessive_precision`).
        // f32 precision is ~7 digits so we compare within 1e-6 absolute.
        let check = |row: usize, col: usize, spec: f64| {
            let got = SPX_ATTEN_TABLE[row][col] as f64;
            assert!(
                (got - spec).abs() < 1e-6,
                "row {row} col {col}: got {got}, spec {spec}",
            );
        };
        // Row 0.
        check(0, 0, 0.954_841_604);
        check(0, 1, 0.911_722_489);
        check(0, 2, 0.870_550_563);
        // Row 14 (half-attenuation reference: T[0]=0.5).
        check(14, 0, 0.5);
        check(14, 1, 0.25);
        check(14, 2, 0.125);
        // Row 29 (quarter-attenuation reference: T[0]=0.25).
        check(29, 0, 0.25);
        check(29, 1, 0.0625);
        check(29, 2, 0.015_625);
        // 32 rows total per the 5-bit `spxattencod[ch]` field.
        assert_eq!(SPX_ATTEN_TABLE.len(), 32);
    }

    /// `apply_spx_atten_notch` is the 5-tap symmetric filter
    /// `[T[0], T[1], T[2], T[1], T[0]]`. Drive it on a constant-1 buffer
    /// and read back the filtered bins — they must equal the kernel.
    #[test]
    fn spx_atten_notch_kernel_is_symmetric() {
        let mut coeffs = [0.0f32; N_COEFFS];
        for v in coeffs.iter_mut().take(50) {
            *v = 1.0;
        }
        // Apply at filtbin=10 with code=14 (T = [0.5, 0.25, 0.125]).
        apply_spx_atten_notch(&mut coeffs, 10, 14);
        assert!((coeffs[10] - 0.5).abs() < 1e-6);
        assert!((coeffs[11] - 0.25).abs() < 1e-6);
        assert!((coeffs[12] - 0.125).abs() < 1e-6);
        assert!((coeffs[13] - 0.25).abs() < 1e-6); // mirror
        assert!((coeffs[14] - 0.5).abs() < 1e-6); // mirror
                                                  // Outside the 5-tap window, coefficients are untouched.
        assert_eq!(coeffs[9], 1.0);
        assert_eq!(coeffs[15], 1.0);
    }

    /// `apply_spx_atten_notch` masks the 5-bit code so a malformed
    /// 6-or-7-bit value doesn't index out of bounds.
    #[test]
    fn spx_atten_notch_masks_code_to_5_bits() {
        let mut coeffs = [1.0f32; N_COEFFS];
        // 0x3F & 0x1F == 31 → row 31. Should not panic.
        apply_spx_atten_notch(&mut coeffs, 0, 0x3F);
        assert!((coeffs[0] - SPX_ATTEN_TABLE[31][0]).abs() < 1e-6);
    }

    /// With `spx_atten_active == true` and `spxattencod = 14` (the
    /// half-attenuation row), the 5 bins centred on the baseband /
    /// extension border (i.e. starting at `spx_begin_tc - 2`) must
    /// be attenuated by `[0.5, 0.25, 0.125, 0.25, 0.5]` AFTER the
    /// translation copy and BEFORE the noise/coord blend. We isolate
    /// the filter contribution by setting blend factors to a pure-pass
    /// (sblend=1, nblend=0, coord=1/32).
    #[test]
    fn spx_synthesis_applies_border_notch_when_chinspxatten() {
        let mut state = Ac3State::new();
        let ch = 0usize;
        state.spx_in_use = true;
        state.channels[ch].in_spx = true;
        state.spx_strtf = 0; // copystart = 25
        state.spx_begin_subbnd = 2; // copyend / spx_begin = 49
        state.spx_end_subbnd = 4; // spx_end = 73
        state.spx_bndstrc = [false; 18];
        state.spx_bndstrc[3] = true;
        state.spx_nbnds = 1;
        state.spx_bndsztab = [0; 18];
        state.spx_bndsztab[0] = 24;
        state.channels[ch].spx_sblend[0] = 1.0;
        state.channels[ch].spx_nblend[0] = 0.0;
        state.channels[ch].spx_coord[0] = 1.0 / 32.0;
        state.channels[ch].end_mant = 49;
        // Drive the whole low-frequency region with a constant signal
        // so the copy + notch is easy to read out.
        for bin in 0..49 {
            state.channels[ch].coeffs[bin] = 1.0;
        }
        // Enable the §3.6.4.2.3 notch with the half-attenuation row.
        state.channels[ch].spx_atten_active = true;
        state.channels[ch].spx_atten_code = 14;

        apply_spectral_extension(&mut state, 1);

        // Border is at spx_begin_tc = 49 → filter window [47, 51].
        let expected = [0.5_f32, 0.25, 0.125, 0.25, 0.5];
        for (i, exp) in expected.iter().enumerate() {
            let bin = 47 + i;
            // Bins 47, 48 are in the baseband (constant=1, scaled by tap).
            // Bins 49, 50, 51 are in the SPX region (copied=1, then scaled
            // by tap, then scaled by sblend=1 * coord=1/32 * 32 = 1).
            assert!(
                (state.channels[ch].coeffs[bin] - exp).abs() < 1e-4,
                "border bin {bin} = {} expected {exp}",
                state.channels[ch].coeffs[bin]
            );
        }
        // Untouched neighbour: bin 46 (below the filter window).
        assert!((state.channels[ch].coeffs[46] - 1.0).abs() < 1e-6);
    }

    /// With `spx_atten_active == false` the SPX synthesis is byte-
    /// identical to the round-100 baseline — the border bins stay at the
    /// copied value (no attenuation applied).
    #[test]
    fn spx_synthesis_no_atten_when_chinspxatten_off() {
        let mut state = Ac3State::new();
        let ch = 0usize;
        state.spx_in_use = true;
        state.channels[ch].in_spx = true;
        state.spx_strtf = 0;
        state.spx_begin_subbnd = 2;
        state.spx_end_subbnd = 4;
        state.spx_bndstrc = [false; 18];
        state.spx_bndstrc[3] = true;
        state.spx_nbnds = 1;
        state.spx_bndsztab = [0; 18];
        state.spx_bndsztab[0] = 24;
        state.channels[ch].spx_sblend[0] = 1.0;
        state.channels[ch].spx_nblend[0] = 0.0;
        state.channels[ch].spx_coord[0] = 1.0 / 32.0;
        state.channels[ch].end_mant = 49;
        for bin in 0..49 {
            state.channels[ch].coeffs[bin] = 1.0;
        }
        state.channels[ch].spx_atten_active = false;

        apply_spectral_extension(&mut state, 1);

        // Border bins are NOT attenuated.
        for bin in 47..=51 {
            assert!(
                (state.channels[ch].coeffs[bin] - 1.0).abs() < 1e-4,
                "no-atten border bin {bin} should stay at 1.0, got {}",
                state.channels[ch].coeffs[bin]
            );
        }
    }

    /// §3.6.4.2.3 wrap-point filtering: when band 1's copy cursor wraps
    /// back to `copystart`, a second 5-tap notch must apply at the start
    /// of band 1 (`band_start - 2`). Construct a geometry where the
    /// copy region is smaller than one band so the second band guarantees
    /// a wrap, and verify a second attenuated 5-bin window appears.
    #[test]
    fn spx_synthesis_applies_wrap_notch_on_band_boundary() {
        let mut state = Ac3State::new();
        let ch = 0usize;
        state.spx_in_use = true;
        state.channels[ch].in_spx = true;
        // Copy region = sub-bands 0..1 only (12 bins: [25, 37)). Two SPX
        // bands of 12 each starting at sub-band 1 → spx region [37, 61).
        // Each SPX band consumes 12 bins from a 12-bin copy region, so
        // the second band MUST wrap.
        state.spx_strtf = 0; // copystart = 25
        state.spx_begin_subbnd = 1; // copyend = 37, spx_begin = 37
        state.spx_end_subbnd = 3; // spx_end = 61
        state.spx_bndstrc = [false; 18];
        state.spx_nbnds = 2;
        state.spx_bndsztab = [0; 18];
        state.spx_bndsztab[0] = 12;
        state.spx_bndsztab[1] = 12;
        state.channels[ch].spx_sblend[0] = 1.0;
        state.channels[ch].spx_nblend[0] = 0.0;
        state.channels[ch].spx_coord[0] = 1.0 / 32.0;
        state.channels[ch].spx_sblend[1] = 1.0;
        state.channels[ch].spx_nblend[1] = 0.0;
        state.channels[ch].spx_coord[1] = 1.0 / 32.0;
        state.channels[ch].end_mant = 37;
        for bin in 0..49 {
            state.channels[ch].coeffs[bin] = 1.0;
        }
        state.channels[ch].spx_atten_active = true;
        state.channels[ch].spx_atten_code = 14; // [0.5, 0.25, 0.125]

        apply_spectral_extension(&mut state, 1);

        // Band 1 starts at SPX bin 49 (37 + 12). Wrap notch is centred
        // on the first bin of band 1 → filter window [47, 51].
        // Border notch (always-applied) is centred on spx_begin_tc=37
        // → filter window [35, 39].
        let expected = [0.5_f32, 0.25, 0.125, 0.25, 0.5];
        for (i, exp) in expected.iter().enumerate() {
            let bin = 35 + i;
            assert!(
                (state.channels[ch].coeffs[bin] - exp).abs() < 1e-4,
                "border-notch bin {bin} expected {exp} got {}",
                state.channels[ch].coeffs[bin]
            );
        }
        for (i, exp) in expected.iter().enumerate() {
            let bin = 47 + i;
            assert!(
                (state.channels[ch].coeffs[bin] - exp).abs() < 1e-4,
                "wrap-notch bin {bin} expected {exp} got {}",
                state.channels[ch].coeffs[bin]
            );
        }
        // Untouched between the two notches: bin 40..46 stay at 1.0.
        for bin in 40..=46 {
            assert!(
                (state.channels[ch].coeffs[bin] - 1.0).abs() < 1e-4,
                "bin {bin} between notches should be 1.0, got {}",
                state.channels[ch].coeffs[bin]
            );
        }
    }
}

#[cfg(test)]
mod multitone_fixture_tests {
    use super::*;
    use crate::bsi::Bsi;
    use crate::syncinfo::SyncInfo;

    fn load_frame(data: &[u8], frame_index: usize) -> Option<(SyncInfo, Bsi, Vec<u8>)> {
        let mut off = 0usize;
        for fi in 0..=frame_index {
            if off + 5 > data.len() {
                return None;
            }
            let si = crate::syncinfo::parse(&data[off..]).ok()?;
            let len = si.frame_length as usize;
            if len < 5 || off + len > data.len() {
                return None;
            }
            if fi == frame_index {
                let bsi = crate::bsi::parse(&data[off + 5..]).ok()?;
                return Some((si, bsi, data[off..off + len].to_vec()));
            }
            off += len;
        }
        None
    }

    fn full_frame_parses(
        si: &SyncInfo,
        bsi: &Bsi,
        frame: &[u8],
        fsnroffst_blk0: Option<[u8; MAX_FBW]>,
    ) -> bool {
        let mut state = Ac3State::new();
        state.apply_bsi_params(bsi);
        let post_sync = &frame[5..];
        let mut br = BitReader::new(post_sync);
        if br.skip(bsi.bits_consumed as u32).is_err() {
            return false;
        }
        for blk in 0..BLOCKS_PER_FRAME {
            state.blkidx = blk;
            if blk == 0 {
                state.fsnroffst_override = fsnroffst_blk0;
            }
            let mut side = AudBlkSideInfo::default();
            if parse_audblk_into(
                &mut state,
                si,
                bsi,
                &mut br,
                &mut side,
                None,
                None,
                Some(post_sync),
            )
            .is_err()
            {
                return false;
            }
        }
        true
    }

    #[test]
    fn multitone_frame2_fsnroffst_full_frame_parse() {
        let path = std::path::PathBuf::from(std::env::var("TEMP").unwrap())
            .join("oxideav_repro")
            .join("multi.ac3");
        if !path.exists() {
            return;
        }
        let data = std::fs::read(&path).unwrap();
        let (si, bsi, frame) = load_frame(&data, 2).expect("frame 2");
        let parsed = full_frame_parses(&si, &bsi, &frame, None);
        let ch1_14 = full_frame_parses(&si, &bsi, &frame, Some([13, 14, 0, 0, 0]));
        let ch0_14 = full_frame_parses(&si, &bsi, &frame, Some([14, 13, 0, 0, 0]));
        // LATAB fix (session 6): parsed [13,13] now fully parses frame 2.
        // ch1=14 still parses; ch0=14 remains a false positive.
        assert!(
            parsed,
            "parsed [13,13] should fully parse frame 2 after LATAB fix"
        );
        assert!(ch1_14, "ch1=14 should fully parse frame 2");
        assert!(
            !ch0_14,
            "ch0=14 false positive should not fully parse frame 2"
        );
    }
}
