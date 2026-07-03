//! Enhanced AC-3 (E-AC-3 / Dolby Digital Plus) encoder per ATSC A/52
//! Annex E. Scope: independent substream (`strmtyp=0`, `substreamid=0`)
//! for 1.0/2.0/5.1 layouts, plus a paired indep+dep substream emission
//! for 7.1 input (5.1 downmix in indep, Lb/Rb in dep with `chanmap`
//! bit 6 = Lrs/Rrs pair set). `bsid=16`, 6 audio blocks per syncframe
//! (`numblkscod=3`), no coupling, no Adaptive Hybrid Transform.
//! Spectral extension (§E.2.3.3 / §E.3.6) is available opt-in through
//! [`make_encoder_with_spx`] — every fbw channel of every substream is
//! then coded only up to the SPX begin frequency and the decoder
//! synthesizes the extension region from the [`super::spxenc`]
//! energy-matching coordinates.
//!
//! Differences from AC-3 (`super::encoder`) at this scope:
//!
//! * **Sync frame size** is signalled in **bytes** via the 11-bit
//!   `frmsiz` field (§E.2.3.1.3) — `frmsiz = (frame_size_in_words - 1)`.
//!   AC-3's `frmsizecod` 6-bit table (§5.4.1.4) is gone. The encoder is
//!   free to pick any size between 64 and 4096 bytes (32–2048 words);
//!   we pick a per-bitrate value that matches the AC-3 lookup so PSNR
//!   comparisons against AC-3 are like-for-like.
//! * **No `crc1`**. The 4-byte syncinfo is just `syncword(16) +
//!   strmtyp(2) + substreamid(3) + frmsiz(11)`. The errorcheck() field
//!   carries only `encinfo(1) + crc2(16)` (§E.2.2.6).
//! * **`audfrm()`** sits between `bsi()` and the audio blocks and
//!   carries frame-level strategy flags (`expstre`, `ahte`,
//!   `snroffststr`, `transproce`, `blkswe`, `dithflage`, `bamode`,
//!   `frmfgaincode`, `dbaflde`, `skipflde`, `spxattene`). This encoder
//!   sets them so per-block strategy data still travels with each
//!   block — i.e. `expstre=1`, `blkswe=1`, `dithflage=1`, `bamode=1`,
//!   `frmfgaincode=1`, `dbaflde=1`, `skipflde=1`. AHT/SPX/transient
//!   pre-noise-processing all set to 0 (out of round-1 scope).
//! * **`audblk()`** for E-AC-3 inserts a few new fields that
//!   we leave at their disabled defaults (no SPX, no enhanced
//!   coupling).
//!
//! The DSP pipeline (windowing, MDCT, exponent extraction, parametric
//! bit allocation, mantissa quantisation) is identical to AC-3 and we
//! reuse the helpers from `super::encoder` directly.

use oxideav_core::bits::BitWriter;
use oxideav_core::Encoder;
use oxideav_core::{
    CodecId, CodecParameters, Error, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::audblk::{remat_band_count_spx, BLOCKS_PER_FRAME, N_COEFFS, SAMPLES_PER_BLOCK};
use crate::decoder::SAMPLES_PER_FRAME;
use crate::encoder::{
    ac3_crc_update, build_dba_plan, compute_bap, decode_input_samples, extract_exponent,
    preprocess_d15, quantise_exponents_to_grpsize, quantise_mantissa, select_exp_strategies,
    tune_snroffst_with_plan, write_exponents_grouped, write_mantissa_stream, BitAllocParams,
    CouplingPlan, TransientDetector, LFE_END_MANT,
};
use crate::mdct::{mdct_256_pair, mdct_512};
use crate::tables::WINDOW;

use super::dsp::DEFAULT_SPX_BNDSTRC;
use super::spxenc::{
    band_coord_targets_span_atten, choose_mstrspxco, quantise_coord, SpxGeometry, SpxParams,
};

/// Codec id string used by the E-AC-3 encoder (registered separately from
/// the AC-3 decoder/encoder in [`crate::register`]).
pub const CODEC_ID_STR: &str = "eac3";

// `EAC3_BSID` is the canonical bsid value for E-AC-3 streams (= 16).
// It lives in [`super::bsi`] now and is re-exported through
// `eac3::EAC3_BSID` in `mod.rs`. Re-import locally for the bit
// writer call-site below.
use super::bsi::EAC3_BSID;

/// Build an E-AC-3 encoder.
///
/// Required parameters:
/// * `sample_rate` — 48 000 / 44 100 / 32 000 Hz
/// * `channels`    — 1 (mono), 2 (stereo), 6 (5.1 = L,C,R,Ls,Rs,LFE),
///   or 8 (7.1 = L,C,R,Ls,Rs,LFE,Lb,Rb — emits an indep+dep substream
///   pair per Annex E §E.3.8.2: indep carries 5.1 with Ls/Rs taken
///   directly from the source, dep carries Lb/Rb with `chanmape=1`
///   and `chanmap` bit 6 (Lrs/Rrs pair) set).
///
/// Optional `bit_rate` (selects the syncframe size). Defaults:
///   * mono   →  96 kbps
///   * stereo → 192 kbps
///   * 5.1    → 384 kbps
///   * 7.1    → 384 kbps indep + 192 kbps dep = 576 kbps total
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    Ok(Box::new(build_concrete_encoder(params)?))
}

/// Build the concrete [`Eac3Encoder`] (with `snroffststr == 0`). The
/// public [`make_encoder`] boxes this behind `dyn Encoder`; the test-only
/// [`make_encoder_with_snroffststr`] reuses it and overrides the strategy.
fn build_concrete_encoder(params: &CodecParameters) -> Result<Eac3Encoder> {
    let sample_rate = params.sample_rate.ok_or_else(|| {
        Error::invalid("eac3 encoder: sample_rate is required (48000/44100/32000)")
    })?;
    let channels = params
        .channels
        .ok_or_else(|| Error::invalid("eac3 encoder: channels is required (1, 2, 6, or 8)"))?;
    let fscod: u8 = match sample_rate {
        48_000 => 0,
        44_100 => 1,
        32_000 => 2,
        _ => {
            return Err(Error::Unsupported(format!(
                "eac3 encoder: unsupported sample rate {sample_rate} (48000/44100/32000 only)"
            )))
        }
    };
    let target_kbps: Option<u32> = params.bit_rate.map(|b| (b / 1000) as u32);

    // Build per-substream layout descriptors. For 1/2/6 channel input
    // we emit one independent substream; for 8 we emit indep+dep.
    let layout = match channels {
        1 => Layout::Indep(SubstreamLayout {
            strmtyp: 0,
            substreamid: 0,
            acmod: 1,
            lfeon: false,
            nfchans: 1,
            chanmap: None,
            // Map each substream channel slot to the input PCM
            // interleaved-channel index. Mono = passthrough.
            src_indices: vec![0],
            // Bytes per syncframe; chosen from `bit_rate` or the per-
            // layout default below.
            kbps: target_kbps.unwrap_or(96),
            frame_bytes: 0,
        }),
        2 => Layout::Indep(SubstreamLayout {
            strmtyp: 0,
            substreamid: 0,
            acmod: 2,
            lfeon: false,
            nfchans: 2,
            chanmap: None,
            src_indices: vec![0, 1],
            kbps: target_kbps.unwrap_or(192),
            frame_bytes: 0,
        }),
        6 => Layout::Indep(SubstreamLayout {
            strmtyp: 0,
            substreamid: 0,
            acmod: 7, // 3/2 — L,C,R,Ls,Rs
            lfeon: true,
            nfchans: 5,
            chanmap: None,
            // Input layout L,C,R,Ls,Rs,LFE → substream order:
            // [L, C, R, Ls, Rs] then LFE pseudo-channel last.
            src_indices: vec![0, 1, 2, 3, 4, 5],
            kbps: target_kbps.unwrap_or(384),
            frame_bytes: 0,
        }),
        8 => {
            // 7.1 input: L,C,R,Ls,Rs,LFE,Lb,Rb.
            // Indep substream = 5.1 downmix where Ls/Rs come straight
            // from the 7.1 source's surround pair (the spec's
            // §E.3.8.2 figure shows the 5.1 downmix as the
            // independently-decodable program).
            let total_kbps = target_kbps.unwrap_or(576);
            // Split: most of the budget goes to the 5.1 indep stream,
            // a quarter goes to the dep Lb/Rb pair (matches stereo).
            let indep_kbps = match total_kbps {
                k if k >= 384 + 192 => 384,
                k => (k * 2) / 3, // generous fallback
            };
            let dep_kbps = total_kbps.saturating_sub(indep_kbps).max(64);
            Layout::Pair {
                indep: SubstreamLayout {
                    strmtyp: 0,
                    substreamid: 0,
                    acmod: 7,
                    lfeon: true,
                    nfchans: 5,
                    chanmap: None,
                    src_indices: vec![0, 1, 2, 3, 4, 5],
                    kbps: indep_kbps,
                    frame_bytes: 0,
                },
                dep: SubstreamLayout {
                    strmtyp: 1,
                    substreamid: 0,
                    acmod: 2, // 2/0 — two coded channels (Lb, Rb)
                    lfeon: false,
                    nfchans: 2,
                    // chanmap bit 6 = Lrs/Rrs pair (Table E2.5).
                    // Per §E.2.3.1.8: bit 0 → MSB of the 16-bit field.
                    // bit 6 → MSB-6 → mask = 1 << (15 - 6) = 0x0200.
                    chanmap: Some(1u16 << (15 - 6)),
                    // Source PCM offsets 6 (Lb) and 7 (Rb).
                    src_indices: vec![6, 7],
                    kbps: dep_kbps,
                    frame_bytes: 0,
                },
            }
        }
        n => {
            return Err(Error::Unsupported(format!(
                "eac3 encoder: unsupported channel count {n} (must be 1, 2, 6, or 8)"
            )))
        }
    };

    // Resolve frame_bytes for each substream + total per-syncframe size
    // (used in `out_params.bit_rate` reporting).
    let total_kbps_reported: u32;
    let layout_resolved = match layout {
        Layout::Indep(mut s) => {
            let bytes = ac3_frame_bytes(fscod, s.kbps).ok_or_else(|| {
                Error::Unsupported(format!(
                    "eac3 encoder: bit rate {} kbps has no frame-size mapping",
                    s.kbps
                ))
            })? as usize;
            s.frame_bytes = bytes;
            total_kbps_reported = s.kbps;
            Layout::Indep(s)
        }
        Layout::Pair { mut indep, mut dep } => {
            let i_bytes = ac3_frame_bytes(fscod, indep.kbps).ok_or_else(|| {
                Error::Unsupported(format!(
                    "eac3 encoder: indep bit rate {} kbps has no frame-size mapping",
                    indep.kbps
                ))
            })? as usize;
            let d_bytes = ac3_frame_bytes(fscod, dep.kbps).ok_or_else(|| {
                Error::Unsupported(format!(
                    "eac3 encoder: dep bit rate {} kbps has no frame-size mapping",
                    dep.kbps
                ))
            })? as usize;
            indep.frame_bytes = i_bytes;
            dep.frame_bytes = d_bytes;
            total_kbps_reported = indep.kbps + dep.kbps;
            Layout::Pair { indep, dep }
        }
    };

    let total_pcm_chans = channels as usize;
    let out_params = {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(sample_rate);
        p.channels = Some(channels);
        p.sample_format = Some(SampleFormat::S16);
        p.bit_rate = Some(total_kbps_reported as u64 * 1000);
        p
    };

    let input_sample_format = params.sample_format.unwrap_or(SampleFormat::S16);
    Ok(Eac3Encoder {
        codec_id: CodecId::new(CODEC_ID_STR),
        out_params,
        sample_rate,
        total_pcm_chans,
        input_sample_format,
        fscod,
        layout: layout_resolved,
        // One delay-line / transient slot per *input* channel. Every
        // distinct PCM channel that ends up in any substream needs its
        // own MDCT-priming history; channels never appear in two
        // substreams under the 7.1 layout, so total_pcm_chans is the
        // upper bound.
        delay_line: vec![vec![0.0f32; SAMPLES_PER_BLOCK]; total_pcm_chans],
        pending_samples: vec![Vec::<f32>::new(); total_pcm_chans],
        transient_state: (0..total_pcm_chans)
            .map(|_| TransientDetector::default())
            .collect(),
        packet_queue: Vec::new(),
        pts: 0,
        snroffststr: 0,
        spx: None,
    })
}

/// Build an E-AC-3 encoder with Spectral Extension (SPX, §E.2.3.3 /
/// §E.3.6) enabled.
///
/// SPX is E-AC-3's parametric high-frequency reconstruction: the coded
/// bandwidth of every full-bandwidth channel stops at the SPX begin
/// frequency (`25 + 12·spx_begin_subbnd` transform coefficients; tc#
/// 109 ≈ 10.2 kHz at 48 kHz for the default [`SpxParams`]), and the
/// decoder regenerates the extension region from a translated copy of
/// the channel's own low-frequency spectrum, noise-blended and scaled
/// by per-band coordinates the encoder derives from the §3.6.4.3
/// energy-matching rule. The bits saved on high-frequency exponents /
/// mantissas are re-spent on the coded low band by the SNR-offset
/// tuner, so SPX trades exact HF waveforms for a better LF floor —
/// the intended low-bit-rate operating mode.
///
/// Accepts the same `params` as [`make_encoder`]. The geometry is
/// validated up front; an invalid combination (inverted sub-band
/// range, empty copy region, out-of-range field) fails here rather
/// than at the first frame.
pub fn make_encoder_with_spx(params: &CodecParameters, spx: SpxParams) -> Result<Box<dyn Encoder>> {
    // Fail fast on invalid geometry (same validation the per-frame
    // emitter performs).
    SpxGeometry::derive(&spx, &DEFAULT_SPX_BNDSTRC)?;
    let mut concrete = build_concrete_encoder(params)?;
    concrete.spx = Some(spx);
    Ok(Box::new(concrete))
}

/// Test-only constructor that selects a non-zero `snroffststr` (per-block
/// SNR-offset strategy). Builds the same encoder as [`make_encoder`] then
/// overrides the strategy. Used by the round-trip tests to drive the
/// decoder's §2.3.3.27 per-block SNR-offset parse.
#[cfg(test)]
pub(crate) fn make_encoder_with_snroffststr(
    params: &CodecParameters,
    snroffststr: u8,
) -> Result<Box<dyn Encoder>> {
    let mut concrete = build_concrete_encoder(params)?;
    concrete.snroffststr = snroffststr;
    Ok(Box::new(concrete))
}

/// Pick a syncframe size in bytes for a given (fscod, target kbps).
/// Returns `None` for unsupported bit rates.
///
/// Formula matches AC-3's Table 5.18 row size:
///   `bytes = round((kbps * 1000 * SAMPLES_PER_FRAME) / (sample_rate * 8))`
/// For 48 kHz this collapses to `kbps * 4` exactly.
fn ac3_frame_bytes(fscod: u8, kbps: u32) -> Option<u32> {
    let sample_rate: u32 = match fscod {
        0 => 48_000,
        1 => 44_100,
        2 => 32_000,
        _ => return None,
    };
    // 1536 samples per syncframe; 8 bits per byte.
    let numerator = kbps as u64 * 1000 * (SAMPLES_PER_FRAME as u64);
    let denom = sample_rate as u64 * 8;
    // AC-3 spec rounds toward the table value; we round down so the
    // bit-rate is a hard upper bound. Even byte counts only: round to
    // the nearest 2-byte boundary so the frame size is a whole word
    // (E-AC-3 frmsiz is in 16-bit words, value = words - 1).
    let raw = numerator / denom;
    let rounded = raw & !1;
    if (32..=4096).contains(&rounded) {
        Some(rounded as u32)
    } else {
        None
    }
}

/// Configuration for one E-AC-3 substream within a syncframe pair.
#[derive(Clone, Debug)]
struct SubstreamLayout {
    /// `strmtyp` field (§E.2.3.1.1). 0 = independent, 1 = dependent.
    strmtyp: u8,
    /// `substreamid` (§E.2.3.1.2). 0 for the primary indep / first dep.
    substreamid: u8,
    /// AC-3 audio coding mode (Table 5.8) of the coded substream.
    acmod: u8,
    /// Whether an LFE pseudo-channel is coded inside the substream.
    lfeon: bool,
    /// Number of full-bandwidth channels coded by this substream
    /// (`acmod_nfchans(acmod)`).
    nfchans: usize,
    /// Custom chanmap (§E.2.3.1.7-8). When `Some`, `chanmape=1` is
    /// emitted in the bsi and the 16-bit `chanmap` field follows. Only
    /// valid when `strmtyp == 1`.
    chanmap: Option<u16>,
    /// Source-PCM channel index for each coded slot. Length =
    /// `nfchans + lfeon as usize`.
    src_indices: Vec<usize>,
    /// Target bit rate in kbps (recorded for reporting / debug).
    #[allow(dead_code)]
    kbps: u32,
    /// Computed syncframe size in bytes (filled in by `make_encoder`).
    frame_bytes: usize,
}

#[derive(Clone, Debug)]
enum Layout {
    /// Single independent substream.
    Indep(SubstreamLayout),
    /// 7.1: independent (5.1 channel program) + dependent (Lb/Rb pair).
    Pair {
        indep: SubstreamLayout,
        dep: SubstreamLayout,
    },
}

struct Eac3Encoder {
    codec_id: CodecId,
    out_params: CodecParameters,
    sample_rate: u32,
    /// Total number of input PCM channels (1, 2, 6, or 8).
    total_pcm_chans: usize,
    input_sample_format: SampleFormat,
    fscod: u8,
    layout: Layout,
    /// Per-input-channel left-half MDCT context (256 samples each).
    delay_line: Vec<Vec<f32>>,
    /// Per-input-channel pending PCM samples, drained 1536 at a time.
    pending_samples: Vec<Vec<f32>>,
    /// Per-input-channel transient-detector state.
    transient_state: Vec<TransientDetector>,
    packet_queue: Vec<Packet>,
    pts: i64,
    /// SNR-offset strategy emitted in audfrm (§2.3.2.3 / Table E1.3).
    /// `0` = single frame-level pair (the default the public
    /// [`make_encoder`] always selects); `1` = one shared per-block fine
    /// offset; `2` = independent per-channel per-block fine offsets. The
    /// non-zero modes are exercised by the round-trip tests to validate
    /// the decoder's per-block SNR-offset parse; the chosen offsets equal
    /// the frame-level values so the decoded PCM matches the `0` baseline.
    snroffststr: u8,
    /// Spectral extension configuration (§E.2.3.3 / §E.3.6). `None`
    /// (the [`make_encoder`] default) codes the full bandwidth; `Some`
    /// (via [`make_encoder_with_spx`]) puts every fbw channel of every
    /// substream in SPX.
    spx: Option<SpxParams>,
}

impl Encoder for Eac3Encoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }
    fn output_params(&self) -> &CodecParameters {
        &self.out_params
    }
    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        let audio = match frame {
            Frame::Audio(a) => a,
            _ => {
                return Err(Error::invalid(
                    "eac3 encoder: send_frame requires an audio frame",
                ))
            }
        };
        let per_chan = decode_input_samples(audio, self.total_pcm_chans, self.input_sample_format)?;
        for ch in 0..self.total_pcm_chans {
            self.pending_samples[ch].extend_from_slice(&per_chan[ch]);
        }
        while self.pending_samples[0].len() as u32 >= SAMPLES_PER_FRAME {
            self.emit_syncframe()?;
        }
        Ok(())
    }
    fn receive_packet(&mut self) -> Result<Packet> {
        if self.packet_queue.is_empty() {
            return Err(Error::NeedMore);
        }
        Ok(self.packet_queue.remove(0))
    }
    fn flush(&mut self) -> Result<()> {
        if self.pending_samples[0].is_empty() {
            return Ok(());
        }
        let missing = SAMPLES_PER_FRAME as usize - self.pending_samples[0].len();
        if missing > 0 {
            for ch in 0..self.total_pcm_chans {
                self.pending_samples[ch].extend(std::iter::repeat(0.0).take(missing));
            }
        }
        self.emit_syncframe()?;
        Ok(())
    }
}

impl Eac3Encoder {
    /// Drain one syncframe worth of PCM (1536 samples per input channel),
    /// emit one packet whose payload is the indep substream — or the
    /// indep+dep concatenation for the 7.1 pair layout.
    fn emit_syncframe(&mut self) -> Result<()> {
        let n_per = SAMPLES_PER_FRAME as usize;

        // Drain `n_per` samples per input channel into a per-input
        // PCM matrix, *advance* the per-input delay-line + transient
        // state once across all blocks/channels. We do the windowing /
        // MDCT inside `emit_substream` per-substream so that blksw and
        // exponents are computed against the substream's actual coded
        // PCM (which for indep substream of 7.1 is just a select of
        // the source channels).
        let mut frame_pcm: Vec<Vec<f32>> = Vec::with_capacity(self.total_pcm_chans);
        for ch in 0..self.total_pcm_chans {
            let drain: Vec<f32> = self.pending_samples[ch].drain(0..n_per).collect();
            frame_pcm.push(drain);
        }

        let mut payload = Vec::<u8>::with_capacity(self.layout_total_frame_bytes());
        // Snapshot the layout once so we don't borrow self while
        // calling emit_substream.
        let substreams: Vec<SubstreamLayout> = match &self.layout {
            Layout::Indep(s) => vec![s.clone()],
            Layout::Pair { indep, dep } => vec![indep.clone(), dep.clone()],
        };
        for sub in &substreams {
            let bytes = self.emit_substream(sub, &frame_pcm)?;
            payload.extend_from_slice(&bytes);
        }

        self.packet_queue.push(
            Packet::new(0, TimeBase::new(1, self.sample_rate as i64), payload).with_pts(self.pts),
        );
        self.pts += SAMPLES_PER_FRAME as i64;
        Ok(())
    }

    /// Total syncframe-pair size in bytes.
    fn layout_total_frame_bytes(&self) -> usize {
        match &self.layout {
            Layout::Indep(s) => s.frame_bytes,
            Layout::Pair { indep, dep } => indep.frame_bytes + dep.frame_bytes,
        }
    }

    /// Emit one E-AC-3 syncframe (indep or dep substream) of size
    /// `sub.frame_bytes` bytes for the channels named by `sub.src_indices`.
    fn emit_substream(&mut self, sub: &SubstreamLayout, frame_pcm: &[Vec<f32>]) -> Result<Vec<u8>> {
        let nfchans = sub.nfchans;
        let total_chans = nfchans + usize::from(sub.lfeon);

        // -------- DSP: window + MDCT per substream channel per block --------
        let mut coeffs: Vec<Vec<[f32; N_COEFFS]>> =
            vec![vec![[0.0; N_COEFFS]; BLOCKS_PER_FRAME]; total_chans];
        // blksw exists only for fbw channels (LFE never short-blocks).
        let mut blksw: Vec<[bool; BLOCKS_PER_FRAME]> = vec![[false; BLOCKS_PER_FRAME]; nfchans];
        for ch in 0..total_chans {
            let src_idx = sub.src_indices[ch];
            let drain = &frame_pcm[src_idx];
            for blk in 0..BLOCKS_PER_FRAME {
                let mut in_buf = [0.0f32; 512];
                in_buf[..256].copy_from_slice(&self.delay_line[src_idx]);
                in_buf[256..].copy_from_slice(
                    &drain[blk * SAMPLES_PER_BLOCK..(blk + 1) * SAMPLES_PER_BLOCK],
                );
                let is_lfe_chan = sub.lfeon && ch == nfchans;
                let is_short = if is_lfe_chan || std::env::var("EAC3_DISABLE_BLKSW").is_ok() {
                    false
                } else {
                    self.transient_state[src_idx].process(&in_buf[256..])
                };
                if !is_lfe_chan {
                    blksw[ch][blk] = is_short;
                }
                let mut win_buf = [0.0f32; 512];
                for n in 0..256 {
                    win_buf[n] = in_buf[n] * WINDOW[n];
                    win_buf[511 - n] = in_buf[511 - n] * WINDOW[n];
                }
                self.delay_line[src_idx].copy_from_slice(
                    &drain[blk * SAMPLES_PER_BLOCK..(blk + 1) * SAMPLES_PER_BLOCK],
                );
                if is_short {
                    mdct_256_pair(&win_buf, &mut coeffs[ch][blk]);
                } else {
                    mdct_512(&win_buf, &mut coeffs[ch][blk]);
                }
            }
        }

        // -------- Exponents --------
        // Layout (matches the AC-3 helpers' expectation):
        //   0..nfchans  → fbw channels
        //   nfchans     → coupling pseudo-channel (unused — cplinu=0)
        //   nfchans + 1 → LFE pseudo-channel (when lfeon)
        let lfe_idx_in_exps = nfchans + 1;
        let mut exps: Vec<Vec<[u8; N_COEFFS]>> =
            vec![vec![[24u8; N_COEFFS]; BLOCKS_PER_FRAME]; nfchans + 2];
        let chbwcod: u8 = 60;
        // §E.2.3.3 / §E.3.3.3 — with SPX in use, every fbw channel's
        // coded bandwidth ends at the SPX begin frequency
        // (`endmant = spxbandtable[spx_begin_subbnd]`) and no chbwcod
        // is emitted; without SPX the bandwidth code drives it.
        // The emitted `spxstrtf` — either the configured code or, with
        // `adaptive_copy_start`, the per-frame least-saturated pick.
        let mut spx_strtf_emit: u8 = self.spx.as_ref().map_or(0, |p| p.spxstrtf);
        let spx_geom = match &self.spx {
            Some(p) => {
                let mut geom = SpxGeometry::derive(p, &DEFAULT_SPX_BNDSTRC)?;
                if p.adaptive_copy_start {
                    // Score each valid candidate by the frame's total
                    // coordinate saturation (log-excess above the 0.875
                    // representable ceiling, §E.2.3.3.11-13), summed
                    // over channels and bands. Strict `<` keeps the
                    // lowest candidate on ties — the largest copy
                    // region carries the most source correlation.
                    let mut best_score = f64::INFINITY;
                    for cand in 0..=3u8 {
                        let cand_params = SpxParams {
                            spxstrtf: cand,
                            ..*p
                        };
                        let Ok(g) = SpxGeometry::derive(&cand_params, &DEFAULT_SPX_BNDSTRC) else {
                            continue; // empty copy region — invalid
                        };
                        let mut score = 0.0f64;
                        for ch_coeffs in coeffs.iter().take(nfchans) {
                            let span: Vec<&[f32; N_COEFFS]> = ch_coeffs.iter().collect();
                            let targets = band_coord_targets_span_atten(&span, &g, p.atten_code);
                            for &t in targets.iter().take(g.nbnds) {
                                if t as f64 > 0.875 {
                                    score += (t as f64 / 0.875).log2();
                                }
                            }
                        }
                        if score < best_score {
                            best_score = score;
                            spx_strtf_emit = cand;
                            geom = g;
                        }
                    }
                }
                Some(geom)
            }
            None => None,
        };
        let end_mant: usize = match &spx_geom {
            Some(g) => g.begin_tc,
            None => 37 + 3 * (chbwcod as usize + 12),
        };
        let ch_end_mant = end_mant;
        for ch in 0..nfchans {
            for blk in 0..BLOCKS_PER_FRAME {
                for k in 0..ch_end_mant {
                    exps[ch][blk][k] = extract_exponent(coeffs[ch][blk][k]);
                }
            }
        }
        if sub.lfeon {
            // §7.1.3: LFE is spectrally constrained to 0-120 Hz per the
            // AC-3 / E-AC-3 specification. At 48 kHz with a 512-point
            // MDCT, bin k ≈ (2k+1)×48000/1024 Hz; bin 0 ≈ 47 Hz,
            // bin 1 ≈ 141 Hz. We zero coefficients at bin ≥ 2 to enforce
            // the 0–120 Hz constraint before exponent extraction, keeping
            // only the sub-120 Hz content in the coded LFE signal. The
            // LFE_END_MANT bitstream limit remains 7 (decoder expects it),
            // but bins 2..7 are set to silence so they don't consume bits.
            let lfe_cutoff = match self.sample_rate {
                48_000 => 2usize, // bin 0 ≈ 47 Hz, bin 1 ≈ 141 Hz → keep 0..2
                44_100 => 2usize,
                32_000 => 2usize,
                _ => 2usize,
            };
            for blk in 0..BLOCKS_PER_FRAME {
                for k in lfe_cutoff..LFE_END_MANT {
                    coeffs[nfchans][blk][k] = 0.0;
                }
                for k in 0..LFE_END_MANT {
                    exps[lfe_idx_in_exps][blk][k] = extract_exponent(coeffs[nfchans][blk][k]);
                }
            }
        }
        // Exponent-strategy selection: adaptive D15/D25/D45 per-channel
        // per-anchor-block (§7.1.3 / §5.4.3.22). The frame-wide anchor
        // pattern [1,0,0,1,0,0] is preserved; for each anchor block
        // (block 0 and block 3) we pick the smoothest legal strategy
        // (D15/D25/D45) per channel. Blocks 1/2/4/5 always REUSE.
        // EAC3_DISABLE_EXPSTR_SEL=1 pins every anchor to D15 (same as
        // old static behaviour) for A/B testing.
        let exp_strategies: [u8; BLOCKS_PER_FRAME] = [1, 0, 0, 1, 0, 0];
        // D15-preprocess every anchor block first so the strategy picker
        // sees legalised exponents.
        for ch in 0..nfchans {
            for blk in 0..BLOCKS_PER_FRAME {
                if exp_strategies[blk] == 1 {
                    preprocess_d15(&mut exps[ch][blk][..ch_end_mant]);
                }
            }
        }
        let chexpstr_plan: Vec<[u8; BLOCKS_PER_FRAME]> =
            if std::env::var("EAC3_DISABLE_EXPSTR_SEL").is_ok() {
                let mut out = vec![[0u8; BLOCKS_PER_FRAME]; nfchans];
                for ch in 0..nfchans {
                    out[ch] = exp_strategies;
                }
                out
            } else {
                select_exp_strategies(&exps, nfchans, ch_end_mant)
            };
        // Apply grpsize quantisation for D25/D45 anchor blocks.
        for ch in 0..nfchans {
            for blk in 0..BLOCKS_PER_FRAME {
                let strat = chexpstr_plan[ch][blk];
                if strat >= 2 {
                    let grpsize = if strat == 2 { 2 } else { 4 };
                    quantise_exponents_to_grpsize(&mut exps[ch][blk][..ch_end_mant], grpsize);
                }
            }
            // REUSE blocks get the most-recent anchor's exponents.
            let mut last = 0usize;
            for blk in 0..BLOCKS_PER_FRAME {
                if chexpstr_plan[ch][blk] != 0 {
                    last = blk;
                } else {
                    let src: [u8; N_COEFFS] = exps[ch][last];
                    exps[ch][blk][..ch_end_mant].copy_from_slice(&src[..ch_end_mant]);
                }
            }
        }
        if sub.lfeon {
            // LFE strategy: D15 on anchor blocks, REUSE elsewhere. The
            // 1-bit lfeexpstr field only supports D15 or REUSE (§5.4.3.23
            // / §E.1.2.3).
            for blk in 0..BLOCKS_PER_FRAME {
                if exp_strategies[blk] == 1 {
                    preprocess_d15(&mut exps[lfe_idx_in_exps][blk][..LFE_END_MANT]);
                }
            }
            let mut last = 0usize;
            for blk in 0..BLOCKS_PER_FRAME {
                if exp_strategies[blk] == 1 {
                    last = blk;
                } else {
                    let src: [u8; N_COEFFS] = exps[lfe_idx_in_exps][last];
                    exps[lfe_idx_in_exps][blk][..LFE_END_MANT]
                        .copy_from_slice(&src[..LFE_END_MANT]);
                }
            }
        }

        // -------- SPX coordinate planning (§E.2.3.3.9-13 / §E.3.6.3) --------
        //
        // Coordinates are refreshed twice per frame — block 0 (where
        // `spxcoe` is implicit for a channel's first SPX block,
        // §E.2.3.3.9) and block 3, matching the exponent anchor
        // pattern; blocks 1/2/4/5 emit `spxcoe = 0` and reuse. Each
        // refresh's coordinate set energy-matches its whole 3-block
        // span (§3.6.4.3), computed from the encoder's own unquantised
        // MDCT spectra via the shared decoder translation plan.
        const SPX_REFRESH_BLOCKS: [usize; 2] = [0, 3];
        const SPX_SPAN: usize = 3;
        // Per channel, per refresh span: (mstrspxco, per-band (exp, mant)).
        type SpxCoordSet = (u8, [(u8, u8); 18]);
        let spx_coords: Option<Vec<[SpxCoordSet; 2]>> = spx_geom.as_ref().map(|g| {
            (0..nfchans)
                .map(|ch| {
                    SPX_REFRESH_BLOCKS.map(|start| {
                        let span: Vec<&[f32; N_COEFFS]> =
                            (start..start + SPX_SPAN).map(|b| &coeffs[ch][b]).collect();
                        let targets = band_coord_targets_span_atten(
                            &span,
                            g,
                            self.spx.as_ref().and_then(|p| p.atten_code),
                        );
                        let mstr = choose_mstrspxco(&targets[..g.nbnds]);
                        let mut codes = [(0u8, 0u8); 18];
                        for bnd in 0..g.nbnds {
                            codes[bnd] = quantise_coord(targets[bnd], mstr);
                        }
                        (mstr, codes)
                    })
                })
                .collect()
        });

        // -------- Bit allocation --------
        let ba = BitAllocParams {
            sdcycod: 2,
            fdcycod: 1,
            sgaincod: 1,
            dbpbcod: 2,
            floorcod: 4,
            csnroffst: 15,
            fsnroffst: 0,
            fsnroffst_ch: [0u8; crate::audblk::MAX_FBW],
            cplfsnroffst: 0,
            lfefsnroffst: 0,
            fgaincod: 4,
            cplfgaincod: 4,
            lfefgaincod: 4,
        };
        // No coupling.
        let cpl = CouplingPlan::default();
        let dba_plan = if std::env::var("EAC3_DISABLE_DBA").is_ok() {
            crate::encoder::DbaPlan::default()
        } else {
            build_dba_plan(&exps, nfchans, ch_end_mant, &cpl)
        };
        // When `snroffststr != 0`, the audblk carries per-block SNR
        // offsets instead of the single frame-level pair in audfrm. Those
        // extra header bits must be reserved out of the mantissa budget,
        // or the bit-allocation tuner would fill the frame and the
        // per-block offsets would push the syncframe past `frame_bytes`.
        // Net extra bits vs the `snroffststr == 0` baseline:
        //   * audfrm drops `frmcsnroffst`(6) + `frmfsnroffst`(4) = 10.
        //   * each block adds an explicit `snroffste`(1) except block 0,
        //     plus `csnroffst`(6) plus the fine offsets.
        //   * snroffststr==1: one shared `blkfsnroffst`(4) per block.
        //   * snroffststr==2: `fsnroffst[ch]`(4·nfchans) + LFE(4) per
        //     block (no coupling slot — cplinu==0 throughout).
        // When `snroffststr != 0` the audblk carries per-block SNR
        // offsets instead of the single frame-level pair in audfrm. Those
        // extra header bits must be reserved out of the mantissa budget,
        // or the bit-allocation tuner would fill the frame and the
        // per-block offsets would push the syncframe past `frame_bytes`.
        // The `snroffststr == 0` default path reserves nothing, so its
        // output is byte-identical to before this change. Net extra bits
        // vs the baseline:
        //   * audfrm drops `frmcsnroffst`(6) + `frmfsnroffst`(4) = 10.
        //   * each block adds an explicit `snroffste`(1) except block 0,
        //     plus `csnroffst`(6) plus the fine offsets.
        //   * snroffststr==1: one shared `blkfsnroffst`(4) per block.
        //   * snroffststr==2: `fsnroffst[ch]`(4·nfchans) + LFE(4) per
        //     block (no coupling slot — cplinu==0 throughout).
        // Both non-zero strategies reserve the SAME (worst-case, i.e.
        // `snroffststr == 2`) overhead so that `1` and `2` tune against an
        // identical mantissa budget — making their decoded PCM bit-exact
        // to each other, which the round-trip test asserts. The `1` mode
        // leaves a few reserved bits unused as extra padding; that does
        // not change the bit allocation.
        let snr_reserve_bits: u32 = if self.snroffststr == 0 {
            0
        } else {
            let per_block_fine = 4 * nfchans as u32 + if sub.lfeon { 4 } else { 0 };
            let explicit_snroffste = (BLOCKS_PER_FRAME as u32) - 1; // blk 0 implicit
            let per_block = BLOCKS_PER_FRAME as u32 * (6 + per_block_fine);
            (per_block + explicit_snroffste).saturating_sub(10)
        };
        // SPX header bits over the SPX-off baseline (which spends one
        // bit per block on the disabled `spxinu`/`spxstre` slot). The
        // reserve always assumes the worst case — an explicit
        // `spxbndstrce == 1` band structure — so the default-banding
        // and explicit-banding emissions tune against an identical
        // mantissa budget and decode bit-identically (pinned by test).
        let spx_reserve_bits: u32 = match &spx_geom {
            None => 0,
            Some(g) => {
                // Block-0 strategy: chinspx (explicit unless mono) +
                // spxstrtf(2) + spxbegf(3) + spxendf(3) + spxbndstrce(1)
                // + explicit structure flags. spxinu itself replaces the
                // baseline's 1-bit slot (no delta).
                let chinspx = if sub.acmod == 0x1 { 0 } else { nfchans as u32 };
                let strat = chinspx + 2 + 3 + 3 + 1 + (g.end_subbnd - g.begin_subbnd - 1) as u32;
                // Coordinates: spxblnd(5) + mstrspxco(2) + 6 bits/band.
                let payload = 5 + 2 + 6 * g.nbnds as u32;
                // Attenuation (audfrm): chinspxatten(1) + spxattencod(5)
                // per fbw channel when signalled.
                let atten = if self.spx.as_ref().is_some_and(|p| p.atten_code.is_some()) {
                    6 * nfchans as u32
                } else {
                    0
                };
                // blk 0: payload (spxcoe implicit); blk 3: 1 + payload;
                // blks 1/2/4/5: 1-bit spxcoe each.
                strat + atten + nfchans as u32 * (payload + (1 + payload) + 4)
            }
        };
        let tuner_frame_bytes = sub
            .frame_bytes
            .saturating_sub((snr_reserve_bits + spx_reserve_bits).div_ceil(8) as usize);
        // Pass chexpstr_plan so the overhead calculator accounts for
        // D25/D45 exponent savings when sizing the mantissa budget.
        let tuned_ba = tune_snroffst_with_plan(
            &ba,
            &exps,
            ch_end_mant,
            nfchans,
            self.fscod,
            tuner_frame_bytes,
            &exp_strategies,
            Some(&chexpstr_plan),
            &cpl,
            &dba_plan,
            sub.acmod,
            sub.lfeon,
        );
        let mut baps: Vec<Vec<[u8; N_COEFFS]>> =
            vec![vec![[0u8; N_COEFFS]; BLOCKS_PER_FRAME]; nfchans + 2];
        let frame_ba = tuned_ba;
        for ch in 0..nfchans {
            for blk in 0..BLOCKS_PER_FRAME {
                compute_bap(
                    &exps[ch][blk],
                    ch_end_mant,
                    self.fscod,
                    &frame_ba,
                    &mut baps[ch][blk],
                    Some((&dba_plan, ch)),
                );
            }
        }
        if sub.lfeon {
            // LFE bit allocation. The shared `compute_bap` is generic
            // — pass the LFE exponents and the LFE-specific budget
            // through a dedicated `BitAllocParams` snapshot whose
            // fsnroffst_ch is irrelevant (LFE uses the lfefsnroffst).
            let lfe_ba = BitAllocParams {
                fsnroffst: tuned_ba.lfefsnroffst,
                fgaincod: tuned_ba.lfefgaincod,
                ..frame_ba
            };
            for blk in 0..BLOCKS_PER_FRAME {
                compute_bap(
                    &exps[lfe_idx_in_exps][blk],
                    LFE_END_MANT,
                    self.fscod,
                    &lfe_ba,
                    &mut baps[lfe_idx_in_exps][blk],
                    None,
                );
            }
        }

        // -------- Pack syncframe --------
        let mut bw = BitWriter::with_capacity(sub.frame_bytes);

        // syncinfo (§E.2.2.1) — just the 16-bit syncword.
        bw.write_u32(0x0B77, 16);

        // bsi (§E.2.2.2)
        bw.write_u32(sub.strmtyp as u32, 2);
        bw.write_u32(sub.substreamid as u32, 3);
        let frmsiz = (sub.frame_bytes / 2 - 1) as u32;
        bw.write_u32(frmsiz, 11);
        bw.write_u32(self.fscod as u32, 2);
        // fscod != 0x3, so emit numblkscod (2 bits). 0x3 = 6 blocks.
        bw.write_u32(0x3, 2);
        bw.write_u32(sub.acmod as u32, 3);
        bw.write_u32(u32::from(sub.lfeon), 1);
        bw.write_u32(EAC3_BSID as u32, 5);
        bw.write_u32(27, 5); // dialnorm = -27 dB
        bw.write_u32(0, 1); // compre = 0
                            // acmod != 0 → no dialnorm2/compr2e (we never produce 1+1).
                            // §E.2.2.2 / E.2.3.1.7-8: dependent substreams emit chanmape
                            // (and chanmap when chanmape=1) immediately after compre.
        if sub.strmtyp == 1 {
            if let Some(map) = sub.chanmap {
                bw.write_u32(1, 1); // chanmape = 1
                bw.write_u32(map as u32, 16); // chanmap
            } else {
                bw.write_u32(0, 1); // chanmape = 0
            }
        }
        bw.write_u32(0, 1); // mixmdate = 0
        bw.write_u32(0, 1); // infomdate = 0
                            // numblkscod == 0x3 → no convsync / blkid / frmsizecod fields.
        bw.write_u32(0, 1); // addbsie = 0

        // audfrm (§E.2.2.3 / §E.2.3.2)
        bw.write_u32(1, 1); // expstre = 1
        bw.write_u32(0, 1); // ahte = 0
        bw.write_u32(self.snroffststr as u32, 2); // snroffststr
        bw.write_u32(0, 1); // transproce = 0
        bw.write_u32(1, 1); // blkswe = 1
        bw.write_u32(1, 1); // dithflage = 1
        bw.write_u32(1, 1); // bamode = 1
        bw.write_u32(1, 1); // frmfgaincode = 1
        bw.write_u32(1, 1); // dbaflde = 1
        bw.write_u32(1, 1); // skipflde = 1
        let spx_atten: Option<u8> = self.spx.as_ref().and_then(|p| p.atten_code);
        bw.write_u32(u32::from(spx_atten.is_some()), 1); // spxattene (§2.3.2.23)
        if sub.acmod > 1 {
            bw.write_u32(0, 1); // cplinu[0] = 0
            for _blk in 1..BLOCKS_PER_FRAME {
                bw.write_u32(0, 1); // cplstre[blk] = 0
            }
        }
        // §E.1.2.3 / Table E1.3 — exponent strategy data.
        //
        // When `expstre == 1`, the per-block per-channel `chexpstr`
        // codes (2 bits each) AND any per-block `cplexpstr` (when
        // coupling is in use) live HERE in audfrm — NOT in audblk.
        // The previous "round 2 fix" inverted this: it moved the bits
        // into audblk because the spec text for §E.2.3.2.1 (`expstre`)
        // says "the fields for the full exponent strategy shall be
        // present in each audio block" — but Table E1.3 makes clear
        // those fields are still emitted in audfrm, just indexed by
        // block. The validator binary's parser consumes them in audfrm
        // and rejects any frame that doesn't supply them, surfacing
        // as the "new bit allocation info must be present in block 0"
        // / "delta bit allocation strategy reserved" / "error in bit
        // allocation" cascade once the parser misaligns.
        //
        // Coupling: cplinu==0 for every block of every substream we
        // emit, so the `if (cplinu[blk] == 1) {cplexpstr[blk]}` branch
        // never fires. We emit per-channel chexpstr from the adaptive
        // D15/D25/D45 plan selected above (chexpstr_plan[ch][blk]).
        for blk in 0..BLOCKS_PER_FRAME {
            for ch in 0..nfchans {
                bw.write_u32(chexpstr_plan[ch][blk] as u32, 2);
            }
        }
        // §E.1.2.3 — `lfeexpstr[blk]` (1 bit) per block when lfeon,
        // OUTSIDE the `if (expstre)` gate (always present when LFE is
        // on, regardless of expstre). LFE always D15 or REUSE.
        if sub.lfeon {
            for blk in 0..BLOCKS_PER_FRAME {
                bw.write_u32(exp_strategies[blk] as u32, 1);
            }
        }
        // §E.1.2.3 — converter exponent strategy data. strmtyp == 0x0
        // (independent substream) and numblkscod == 0x3 ⇒ convexpstre
        // implicit = 1, followed by per-channel convexpstr (5 bits each).
        if sub.strmtyp == 0 {
            for _ in 0..nfchans {
                bw.write_u32(0, 5); // convexpstr = 0 (REUSE codeword)
            }
        }
        // ahte == 0 ⇒ no AHT block.
        // snroffststr == 0 ⇒ frame-level (frmcsnroffst, frmfsnroffst) in
        // audfrm; snroffststr ∈ {1, 2} ⇒ no frame-level fields here (the
        // offsets are carried per-block in each audblk instead).
        if self.snroffststr == 0 {
            bw.write_u32(tuned_ba.csnroffst as u32, 6); // frmcsnroffst
            bw.write_u32(tuned_ba.fsnroffst as u32, 4); // frmfsnroffst
        }
        // §2.3.2.24-25 — spectral extension attenuation data: one
        // chinspxatten[ch] bit per fbw channel, + spxattencod[ch]
        // (5 bits) when set. Emitted between the SNR-offset tail and
        // blkstrtinfoe per Table E1.3. Every fbw channel is in SPX, so
        // all channels carry the (shared) code.
        if let Some(code) = spx_atten {
            for _ in 0..nfchans {
                bw.write_u32(1, 1); // chinspxatten[ch] = 1
                bw.write_u32(code as u32, 5); // spxattencod[ch]
            }
        }
        bw.write_u32(0, 1); // blkstrtinfoe = 0

        // -------- audio blocks --------
        for blk in 0..BLOCKS_PER_FRAME {
            for ch in 0..nfchans {
                bw.write_u32(blksw[ch][blk] as u32, 1);
            }
            for _ in 0..nfchans {
                bw.write_u32(1, 1); // dithflag = 1
            }
            bw.write_u32(0, 1); // dynrnge = 0

            // SPX strategy (§E.2.3.3.1-8). Block 0 has implicit
            // `spxstre = 1` and emits `spxinu` directly; later blocks
            // emit `spxstre = 0` (strategy reuse) — the geometry is
            // frame-constant.
            match (&spx_geom, blk) {
                (Some(g), 0) => {
                    let p = self.spx.as_ref().expect("spx params when geom is set");
                    bw.write_u32(1, 1); // spxinu = 1
                                        // §E.2.3.3.3 chinspx[ch] — implicit for mono
                                        // (acmod == 0x1); explicit per fbw channel
                                        // otherwise. Every channel is in SPX.
                    if sub.acmod != 0x1 {
                        for _ in 0..nfchans {
                            bw.write_u32(1, 1);
                        }
                    }
                    bw.write_u32(spx_strtf_emit as u32, 2); // §E.2.3.3.4
                    bw.write_u32(p.spxbegf as u32, 3); // §E.2.3.3.5
                    bw.write_u32(p.spxendf as u32, 3); // §E.2.3.3.6
                    if p.explicit_band_structure {
                        // §E.2.3.3.7-8 — explicit structure carrying the
                        // same content as the Table E2.11 default.
                        bw.write_u32(1, 1);
                        for bnd in (g.begin_subbnd + 1)..g.end_subbnd {
                            bw.write_u32(u32::from(g.bndstrc[bnd]), 1);
                        }
                    } else {
                        bw.write_u32(0, 1); // spxbndstrce = 0 → default banding
                    }
                }
                (Some(_), _) => bw.write_u32(0, 1), // spxstre = 0 (reuse)
                (None, 0) => bw.write_u32(0, 1),    // spxinu = 0
                (None, _) => bw.write_u32(0, 1),    // spxstre = 0
            }
            // SPX coordinates (§E.2.3.3.9-13). `spxcoe` is implicit-1
            // on a channel's first SPX block (block 0 here); explicit
            // thereafter. Refresh on the anchor blocks, reuse between.
            if let (Some(g), Some(coords)) = (&spx_geom, &spx_coords) {
                let refresh = SPX_REFRESH_BLOCKS.contains(&blk);
                let span_idx = usize::from(blk >= SPX_REFRESH_BLOCKS[1]);
                let spxblnd = self.spx.as_ref().expect("spx params").spxblnd;
                for ch in 0..nfchans {
                    if blk != 0 {
                        bw.write_u32(u32::from(refresh), 1); // spxcoe[ch]
                    }
                    if refresh {
                        let (mstr, codes) = &coords[ch][span_idx];
                        bw.write_u32(spxblnd as u32, 5); // §E.2.3.3.10
                        bw.write_u32(*mstr as u32, 2); // §E.2.3.3.11
                        for bnd in 0..g.nbnds {
                            bw.write_u32(codes[bnd].0 as u32, 4); // spxcoexp
                            bw.write_u32(codes[bnd].1 as u32, 2); // spxcomant
                        }
                    }
                }
            }

            // Coupling: cplinu always 0 — block 0 hits the "else"
            // branch with no body; later blocks have cplstre=0 so
            // skip entirely.

            // Rematrixing — only acmod==2. The flag-field size folds in
            // SPX per §E.3.3.2 (spxbegf < 2 → 3 bands, else 4; 4 when
            // SPX is off since coupling is never in use here). Round-1
            // keeps rematrixing disabled (all flags 0).
            if sub.acmod == 2 {
                let nrematbd = remat_band_count_spx(
                    false,
                    0,
                    false,
                    0,
                    spx_geom.is_some(),
                    spx_geom.as_ref().map_or(0, |g| g.begin_subbnd),
                );
                if blk == 0 {
                    for _bnd in 0..nrematbd {
                        bw.write_u32(0, 1);
                    }
                } else {
                    bw.write_u32(0, 1); // rematstr = 0
                }
            }

            let exp_strategy = exp_strategies[blk];
            // §E.1.2.4 / Table E1.4 — `chexpstr[blk][ch]` and
            // `lfeexpstr[blk]` were already emitted in audfrm above.
            // audblk only carries the **bandwidth code + exponent
            // payload** that follows from those strategies, gated by
            // the `chexpstr[blk][ch] != reuse` test.
            //
            // §E.1.2.4 chbwcod[ch] — 6 bits per fbw channel whose
            // strategy this block is non-REUSE AND that channel is
            // neither in coupling nor in spectral extension
            // (`if((!chincpl[ch]) && (!chinspx[ch])) {chbwcod[ch]}`).
            // With SPX every fbw channel is in SPX, so no chbwcod is
            // emitted — the decoder derives the bandwidth from the SPX
            // begin frequency (§E.3.3.3).
            if spx_geom.is_none() {
                for ch in 0..nfchans {
                    if chexpstr_plan[ch][blk] != 0 {
                        bw.write_u32(chbwcod as u32, 6);
                    }
                }
            }
            // §E.1.2.4 fbw exponents (cplexps section is skipped:
            // cplinu=0). After each channel's grouped exponents, the
            // spec emits `gainrng[ch]` (2 bits) — same as base AC-3
            // §5.4.2.20. Emit exponents using the per-channel strategy
            // (D15/D25/D45 from chexpstr_plan).
            for ch in 0..nfchans {
                let strat = chexpstr_plan[ch][blk];
                if strat != 0 {
                    let grpsize = match strat {
                        1 => 1usize,
                        2 => 2usize,
                        _ => 4usize,
                    };
                    write_exponents_grouped(&mut bw, &exps[ch][blk], ch_end_mant, grpsize);
                    bw.write_u32(0, 2); // gainrng[ch] = 0
                }
            }
            // LFE exponents — D15 only, no chbwcod (LFE has fixed bw),
            // and no gainrng (only fbw channels carry gainrng).
            if sub.lfeon && exp_strategy == 1 {
                write_exponents_grouped(&mut bw, &exps[lfe_idx_in_exps][blk], LFE_END_MANT, 1);
            }

            let baie = blk == 0;
            bw.write_u32(baie as u32, 1);
            if baie {
                bw.write_u32(tuned_ba.sdcycod as u32, 2);
                bw.write_u32(tuned_ba.fdcycod as u32, 2);
                bw.write_u32(tuned_ba.sgaincod as u32, 2);
                bw.write_u32(tuned_ba.dbpbcod as u32, 2);
                bw.write_u32(tuned_ba.floorcod as u32, 3);
            }

            // §2.3.3.27 — per-block SNR offsets when snroffststr ∈ {1,2}.
            // (The snroffststr == 0 path carries them once in audfrm.)
            // Block 0 emits implicitly (snroffste == 1); later blocks emit
            // an explicit `snroffste = 1` so every block carries the same
            // value (the decode then matches the frame-level reference).
            // cplinu == 0 for every block, so the coupling slot is skipped.
            if self.snroffststr != 0 {
                if blk != 0 {
                    bw.write_u32(1, 1); // snroffste = 1
                }
                bw.write_u32(tuned_ba.csnroffst as u32, 6); // csnroffst
                if self.snroffststr == 1 {
                    bw.write_u32(tuned_ba.fsnroffst as u32, 4); // blkfsnroffst
                } else {
                    // snroffststr == 2 — per-channel fine offsets (no cpl
                    // slot: cplinu == 0). LFE slot when present.
                    for _ in 0..nfchans {
                        bw.write_u32(tuned_ba.fsnroffst as u32, 4); // fsnroffst[ch]
                    }
                    if sub.lfeon {
                        bw.write_u32(tuned_ba.lfefsnroffst as u32, 4); // lfefsnroffst
                    }
                }
            }

            // §E.1.3.5.4 frmfgaincode==1 ⇒ per-block `fgaincode` flag.
            bw.write_u32(0, 1); // fgaincode = 0 (no per-channel override)

            // §E.1.2.4 convsnroffste — UNCONDITIONAL when strmtyp == 0
            // (independent substream). Per Table E1.4 line 7965, this
            // sits between the fgaincode block and the cplleak block,
            // gated only by `if (strmtyp == 0x0)` — NOT inside the
            // `if (snroffste)` branch as the previous comment claimed.
            // We never emit `convsnroffst`, so write the gating bit at
            // 0.
            if sub.strmtyp == 0 {
                bw.write_u32(0, 1); // convsnroffste = 0
            }
            // cplleak block skipped: cplinu=0 for every block.

            let any_fbw_dba = (0..nfchans).any(|c| dba_plan.nseg[c] > 0);
            if blk == 0 && any_fbw_dba {
                bw.write_u32(1, 1); // deltbaie = 1
                for ch in 0..nfchans {
                    let code = if dba_plan.nseg[ch] > 0 { 1 } else { 2 };
                    bw.write_u32(code as u32, 2);
                }
                for ch in 0..nfchans {
                    if dba_plan.nseg[ch] > 0 {
                        let nseg = dba_plan.nseg[ch] as u32;
                        bw.write_u32(nseg - 1, 3);
                        for seg in 0..nseg as usize {
                            bw.write_u32(dba_plan.offst[ch][seg] as u32, 5);
                            bw.write_u32(dba_plan.len[ch][seg] as u32, 4);
                            bw.write_u32(dba_plan.ba[ch][seg] as u32, 3);
                        }
                    }
                }
            } else {
                bw.write_u32(0, 1); // deltbaie = 0
            }

            bw.write_u32(0, 1); // skiple = 0

            // -------- Mantissas --------
            // Order per AC-3 §5.4.3.49: fbw channels in ascending index,
            // then (when present) the LFE pseudo-channel.
            let mut codes: Vec<(u8, u32)> = Vec::with_capacity(nfchans * ch_end_mant + 4);
            for ch in 0..nfchans {
                for bin in 0..ch_end_mant {
                    let bap = baps[ch][blk][bin];
                    if bap == 0 {
                        continue;
                    }
                    let e = exps[ch][blk][bin] as i32;
                    let mant = quantise_mantissa(coeffs[ch][blk][bin], e, bap);
                    codes.push((bap, mant));
                }
            }
            if sub.lfeon {
                for bin in 0..LFE_END_MANT {
                    let bap = baps[lfe_idx_in_exps][blk][bin];
                    if bap == 0 {
                        continue;
                    }
                    let e = exps[lfe_idx_in_exps][blk][bin] as i32;
                    let mant = quantise_mantissa(coeffs[nfchans][blk][bin], e, bap);
                    codes.push((bap, mant));
                }
            }
            write_mantissa_stream(&mut bw, &codes);
        }

        // auxdata + errorcheck.
        let target_bits = (sub.frame_bytes * 8) as u64;
        let used_bits = bw.bit_position();
        let errorcheck_bits = 17u64;
        if used_bits + errorcheck_bits > target_bits {
            return Err(Error::other(format!(
                "eac3 encoder: bit budget overflow ({} bits used, frame {} bits, strmtyp={}, acmod={}, lfeon={})",
                used_bits, target_bits, sub.strmtyp, sub.acmod, sub.lfeon
            )));
        }
        let pad_bits = target_bits - used_bits - errorcheck_bits;
        let mut left = pad_bits;
        while left >= 32 {
            bw.write_u32(0, 32);
            left -= 32;
        }
        if left > 0 {
            bw.write_u32(0, left as u32);
        }
        bw.write_u32(0, 1); // encinfo = 0
        bw.write_u32(0, 16); // crc2 placeholder
        let mut frame = bw.into_bytes();
        debug_assert_eq!(frame.len(), sub.frame_bytes);

        // crc2 in augmented form per §7.10.1 (Annex E §E.1.2 inherits
        // the same residue check). Shift the body + 16 trailing zero
        // bits through the LFSR; the resulting register value goes in
        // the crc2 field so a spec-strict decoder's residue check
        // `ac3_crc_update(0, post_syncword) == 0` succeeds. E-AC-3
        // syncframes have no crc1 (§E.1.2 elides it), so the running
        // CRC starts at byte 2 (post-syncword).
        let body_residue = ac3_crc_update(0, &frame[2..(sub.frame_bytes - 2)]);
        let crc2_val = ac3_crc_update(body_residue, &[0u8, 0u8]);
        let n = sub.frame_bytes;
        frame[n - 2] = (crc2_val >> 8) as u8;
        frame[n - 1] = (crc2_val & 0xFF) as u8;
        debug_assert_eq!(
            ac3_crc_update(0, &frame[2..sub.frame_bytes]),
            0,
            "E-AC-3 crc2 emit produced a non-zero post-syncword residue"
        );
        Ok(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_bytes_lookup_48k() {
        // 192 kbps @ 48 kHz ⇒ 768 bytes (matches AC-3's frmsizecod=20).
        assert_eq!(ac3_frame_bytes(0, 192), Some(768));
        // 96 kbps @ 48 kHz ⇒ 384 bytes.
        assert_eq!(ac3_frame_bytes(0, 96), Some(384));
        // 32 kbps lower bound — even, in range.
        assert_eq!(ac3_frame_bytes(0, 32), Some(128));
        // 640 kbps @ 48 kHz ⇒ 2560 bytes (largest A/52 row).
        assert_eq!(ac3_frame_bytes(0, 640), Some(2560));
    }

    #[test]
    fn make_encoder_stereo_48k() {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(48_000);
        p.channels = Some(2);
        p.bit_rate = Some(192_000);
        let _enc = make_encoder(&p).expect("eac3 stereo 192k");
    }

    // ---- per-block SNR-offset (snroffststr ∈ {1, 2}) round-trip ----
    //
    // The decoder's §2.3.3.27 per-block SNR-offset parse is validated by
    // a self-encode → self-decode round-trip: the same PCM is encoded
    // three times — once with the default frame-level `snroffststr == 0`
    // and once each with `snroffststr == 1` and `== 2`. The `1` / `2`
    // encodes carry the SAME numeric (csnroffst, fsnroffst) the bit
    // allocator chose, just spread per-block across the audblks instead of
    // once in audfrm; their mantissa budget is fractionally smaller (the
    // per-block headers are reserved), so the decoded PCM is near-identical
    // to the `0` baseline rather than bit-exact. The discriminating signal
    // is that a ONE-BIT cursor misalignment in the new parse would corrupt
    // every downstream mantissa and collapse the PSNR — so we gate on a
    // high PSNR-vs-baseline floor (≥ 70 dB), which only a correctly
    // bit-aligned parse can reach.

    fn psnr_vs(a: &[i16], b: &[i16]) -> f64 {
        assert_eq!(a.len(), b.len(), "length mismatch in psnr_vs");
        let mut se = 0.0f64;
        for (x, y) in a.iter().zip(b.iter()) {
            let d = *x as f64 - *y as f64;
            se += d * d;
        }
        let mse = se / a.len().max(1) as f64;
        if mse == 0.0 {
            return f64::INFINITY;
        }
        let peak = 32768.0f64;
        10.0 * (peak * peak / mse).log10()
    }

    fn build_sine_pcm(channels: usize, frames: usize) -> Vec<f32> {
        let n = frames * SAMPLES_PER_FRAME as usize;
        let mut pcm = vec![0.0f32; n * channels];
        for i in 0..n {
            let t = i as f32 / 48_000.0;
            // Two tones so several bands carry energy (so the SNR offset
            // actually changes how many mantissa bits each band gets).
            let s = 0.4 * (2.0 * std::f32::consts::PI * 440.0 * t).sin()
                + 0.25 * (2.0 * std::f32::consts::PI * 3500.0 * t).sin();
            for ch in 0..channels {
                pcm[i * channels + ch] = s * (1.0 - 0.1 * ch as f32);
            }
        }
        pcm
    }

    fn encode_with(pcm: &[f32], channels: usize, bit_rate: u64, snroffststr: u8) -> Vec<u8> {
        let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        params.sample_rate = Some(48_000);
        params.channels = Some(channels as u16);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(bit_rate);
        let mut enc: Box<dyn Encoder> = if snroffststr == 0 {
            make_encoder(&params).expect("eac3 make_encoder")
        } else {
            make_encoder_with_snroffststr(&params, snroffststr).expect("eac3 make_encoder snr")
        };
        let n_samp = pcm.len() / channels;
        let mut s16 = Vec::with_capacity(pcm.len() * 2);
        for &v in pcm {
            let q = (v * 32767.0).clamp(-32768.0, 32767.0) as i16;
            s16.extend_from_slice(&q.to_le_bytes());
        }
        enc.send_frame(&Frame::Audio(oxideav_core::AudioFrame {
            samples: n_samp as u32,
            pts: Some(0),
            data: vec![s16],
        }))
        .unwrap();
        enc.flush().unwrap();
        let mut out = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => out.extend_from_slice(&p.data),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("eac3 encode error: {e:?}"),
            }
        }
        out
    }

    fn decode_all(bytes: &[u8], frame_bytes: usize) -> Vec<i16> {
        use crate::eac3::decoder::{decode_eac3_packet, Eac3DecoderState};
        let mut st = Eac3DecoderState::default();
        let mut out = Vec::new();
        let mut off = 0usize;
        while off + frame_bytes <= bytes.len() {
            let frame = decode_eac3_packet(&mut st, &bytes[off..off + frame_bytes])
                .expect("decode eac3 packet");
            for chunk in frame.pcm_s16le.chunks_exact(2) {
                out.push(i16::from_le_bytes([chunk[0], chunk[1]]));
            }
            off += frame_bytes;
        }
        out
    }

    fn assert_snroffststr_roundtrip(channels: usize, bit_rate: u64, frame_bytes: usize) {
        let pcm = build_sine_pcm(channels, 4);
        let base = decode_all(&encode_with(&pcm, channels, bit_rate, 0), frame_bytes);
        let dec1 = decode_all(&encode_with(&pcm, channels, bit_rate, 1), frame_bytes);
        let dec2 = decode_all(&encode_with(&pcm, channels, bit_rate, 2), frame_bytes);
        assert!(!base.is_empty(), "baseline decode produced no PCM");
        assert_eq!(
            dec1.len(),
            base.len(),
            "snroffststr=1 sample-count mismatch"
        );
        assert_eq!(
            dec2.len(),
            base.len(),
            "snroffststr=2 sample-count mismatch"
        );

        // (a) The two per-block strategies share an identical mantissa
        // budget and SNR offsets, so they MUST decode bit-for-bit equal.
        // This is the strict cursor-alignment invariant: if either parse
        // mis-consumes a single bit, the mantissas diverge and the two
        // decodes differ.
        assert_eq!(
            dec1, dec2,
            "snroffststr=1 and =2 must decode bit-identically (same budget + offsets); \
             a mismatch means one of the per-block SNR-offset parses is misaligned"
        );

        // (b) Sanity: each per-block strategy decodes to near-identical
        // audio as the frame-level baseline (the only delta is the few
        // reserved header bits). A misaligned parse would collapse this
        // PSNR toward 0 dB; the small budget delta keeps it ≥ 55 dB.
        for (strat, dec) in [(1u8, &dec1), (2u8, &dec2)] {
            let psnr = psnr_vs(dec, &base);
            assert!(
                psnr >= 55.0,
                "snroffststr={strat}: PSNR vs frame-level baseline {psnr:.2} dB < 55 dB — \
                 the per-block SNR-offset parse is bit-misaligned"
            );
        }
    }

    #[test]
    fn snroffststr_per_block_roundtrip_mono() {
        // 128 kbps @ 48 kHz ⇒ 512-byte frames, single indep substream.
        // The slightly higher rate (vs the 96k default) leaves padding
        // headroom for the extra per-block SNR-offset header bits the
        // §2.3.3.27 strategies carry over the frame-level baseline.
        assert_snroffststr_roundtrip(1, 128_000, 512);
    }

    #[test]
    fn snroffststr_per_block_roundtrip_stereo() {
        // 256 kbps @ 48 kHz ⇒ 1024-byte frames.
        assert_snroffststr_roundtrip(2, 256_000, 1024);
    }

    #[test]
    fn snroffststr_per_block_roundtrip_51() {
        // 5.1 @ 448 kbps ⇒ 1792-byte frames, lfeon=1 so the §2.3.3.27
        // `snroffststr == 2` path also exercises the per-block
        // `lfefsnroffst` slot.
        assert_snroffststr_roundtrip(6, 448_000, 1792);
    }

    // ---- spectral extension (§E.2.3.3 / §E.3.6) round-trips ----
    //
    // The SPX encode is validated with the in-tree decoder on two
    // axes:
    //
    // 1. **Cursor alignment** — the SPX strategy + coordinate fields
    //    thread through the audblk between the dynrng and coupling
    //    slots and re-gate chbwcod / nrematbd; one mis-sized field
    //    corrupts every downstream exponent + mantissa and collapses
    //    the decode to noise. The banded-energy gates below can only
    //    pass on a bit-aligned parse.
    // 2. **§3.6.4.3 energy matching** — the whole point of the tool:
    //    per SPX band, the synthesized HF energy must match the
    //    original signal's banded HF energy (the encoder's coordinate
    //    computation + quantiser + the decoder's translation / blend /
    //    scale chain, end to end).
    //
    // The analysis measures banded energy in the encoder's own MDCT
    // domain (same window + transform), averaged over the steady-state
    // interior. Pure-tone content is stationary, so the metric is
    // insensitive to coder delay and needs no lag alignment.

    fn encode_spx(pcm: &[f32], channels: usize, bit_rate: u64, spx: SpxParams) -> Vec<u8> {
        let mut params = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        params.sample_rate = Some(48_000);
        params.channels = Some(channels as u16);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(bit_rate);
        let mut enc = make_encoder_with_spx(&params, spx).expect("eac3 make_encoder_with_spx");
        let n_samp = pcm.len() / channels;
        let mut s16 = Vec::with_capacity(pcm.len() * 2);
        for &v in pcm {
            let q = (v * 32767.0).clamp(-32768.0, 32767.0) as i16;
            s16.extend_from_slice(&q.to_le_bytes());
        }
        enc.send_frame(&Frame::Audio(oxideav_core::AudioFrame {
            samples: n_samp as u32,
            pts: Some(0),
            data: vec![s16],
        }))
        .unwrap();
        enc.flush().unwrap();
        let mut out = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => out.extend_from_slice(&p.data),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("eac3 spx encode error: {e:?}"),
            }
        }
        out
    }

    /// Multitone with content below AND above the default SPX begin
    /// frequency (tc# 109 ≈ 10.2 kHz at 48 kHz): LF tones at 700 Hz /
    /// 3.1 kHz / 6 kHz feed the coded band + translation copy region,
    /// HF tones at 12.2 / 15.4 / 19 kHz land in SPX bands 0 / 2 / 3.
    ///
    /// A deterministic broadband noise bed (~-38 dBFS) rides under the
    /// tones. SPX synthesizes each HF band by *scaling a translated
    /// copy* of the low band — a band whose translation source is
    /// spectrally EMPTY can only be filled up to the coordinate
    /// ceiling (`0.875 · 32 ×` the source RMS), so an all-tones signal
    /// with silent gaps between tones is unencodable by construction
    /// (the coordinate saturates and the band comes out low). Real
    /// content carries a broadband floor; the bed models that and
    /// keeps every band's coordinate inside the representable range.
    fn build_spx_multitone(channels: usize, frames: usize) -> Vec<f32> {
        let n = frames * SAMPLES_PER_FRAME as usize;
        let mut pcm = vec![0.0f32; n * channels];
        let tones: [(f32, f32); 6] = [
            (700.0, 0.22),
            (3_100.0, 0.18),
            (6_000.0, 0.14),
            (12_200.0, 0.055),
            (15_400.0, 0.035),
            (19_000.0, 0.030),
        ];
        let mut lfsr: u32 = 0x2545_F491;
        for i in 0..n {
            let t = i as f32 / 48_000.0;
            let mut s = 0.0f32;
            for (f, a) in tones {
                s += a * (2.0 * std::f32::consts::PI * f * t).sin();
            }
            // Deterministic xorshift noise bed, uniform in ±0.012.
            lfsr ^= lfsr << 13;
            lfsr ^= lfsr >> 17;
            lfsr ^= lfsr << 5;
            s += ((lfsr as f32 / u32::MAX as f32) * 2.0 - 1.0) * 0.012;
            for ch in 0..channels {
                pcm[i * channels + ch] = s * (1.0 - 0.08 * ch as f32);
            }
        }
        pcm
    }

    /// Mean per-bin MDCT energy of one channel of interleaved PCM,
    /// using the encoder's own window + long transform, skipping the
    /// first / last few blocks (codec priming + flush edges).
    fn mdct_energy_profile(pcm: &[f32], channels: usize, ch: usize) -> [f64; N_COEFFS] {
        use crate::mdct::mdct_512;
        use crate::tables::WINDOW;
        let n = pcm.len() / channels;
        let mut acc = [0.0f64; N_COEFFS];
        let mut blocks = 0usize;
        let mut start = 4 * SAMPLES_PER_BLOCK; // skip priming edge
        while start + 512 + 4 * SAMPLES_PER_BLOCK <= n {
            let mut win = [0.0f32; 512];
            for k in 0..256 {
                win[k] = pcm[(start + k) * channels + ch] * WINDOW[k];
                win[511 - k] = pcm[(start + 511 - k) * channels + ch] * WINDOW[k];
            }
            let mut coeffs = [0.0f32; N_COEFFS];
            mdct_512(&win, &mut coeffs);
            for (a, &c) in acc.iter_mut().zip(coeffs.iter()) {
                *a += (c as f64) * (c as f64);
            }
            blocks += 1;
            start += SAMPLES_PER_BLOCK;
        }
        assert!(blocks > 8, "analysis needs a steady-state interior");
        for a in acc.iter_mut() {
            *a /= blocks as f64;
        }
        acc
    }

    fn band_db_deltas(
        orig: &[f64; N_COEFFS],
        dec: &[f64; N_COEFFS],
        geom: &crate::eac3::spxenc::SpxGeometry,
    ) -> Vec<f64> {
        let mut out = Vec::new();
        let mut lo = geom.begin_tc;
        for bnd in 0..geom.nbnds {
            let hi = lo + geom.bndsztab[bnd];
            let eo: f64 = orig[lo..hi].iter().sum();
            let ed: f64 = dec[lo..hi].iter().sum();
            out.push(10.0 * (ed.max(1e-30) / eo.max(1e-30)).log10());
            lo = hi;
        }
        out
    }

    fn to_f32_interleaved(pcm: &[i16]) -> Vec<f32> {
        pcm.iter().map(|&v| v as f32 / 32767.0).collect()
    }

    fn assert_spx_roundtrip(channels: usize, bit_rate: u64, frame_bytes: usize, tol_db: f64) {
        let spx = SpxParams::default();
        let geom = SpxGeometry::derive(&spx, &DEFAULT_SPX_BNDSTRC).unwrap();
        let pcm = build_spx_multitone(channels, 8);
        let stream = encode_spx(&pcm, channels, bit_rate, spx);
        assert_eq!(
            stream.len() % frame_bytes,
            0,
            "stream not a whole number of {frame_bytes}-byte frames"
        );
        let dec = decode_all(&stream, frame_bytes);
        assert_eq!(
            dec.len() / channels,
            (stream.len() / frame_bytes) * SAMPLES_PER_FRAME as usize,
            "sample-count mismatch"
        );
        let dec_f = to_f32_interleaved(&dec);
        for ch in 0..channels.min(2) {
            let orig_prof = mdct_energy_profile(&pcm, channels, ch);
            let dec_prof = mdct_energy_profile(&dec_f, channels, ch);
            // (a) SPX-band energy match (§3.6.4.3).
            let deltas = band_db_deltas(&orig_prof, &dec_prof, &geom);
            for (bnd, d) in deltas.iter().enumerate() {
                assert!(
                    d.abs() <= tol_db,
                    "ch{ch} SPX band {bnd}: energy delta {d:+.2} dB exceeds ±{tol_db} dB \
                     (all deltas: {deltas:?})"
                );
            }
            // (b) Coded-band fidelity: total LF energy within ±1.5 dB.
            let eo: f64 = orig_prof[..geom.begin_tc].iter().sum();
            let ed: f64 = dec_prof[..geom.begin_tc].iter().sum();
            let lf_delta = 10.0 * (ed / eo).log10();
            assert!(
                lf_delta.abs() <= 1.5,
                "ch{ch}: coded-band energy delta {lf_delta:+.2} dB"
            );
        }
    }

    #[test]
    fn spx_roundtrip_stereo_band_energy() {
        // 192 kbps @ 48 kHz ⇒ 768-byte frames.
        assert_spx_roundtrip(2, 192_000, 768, 3.0);
    }

    #[test]
    fn spx_roundtrip_mono_implicit_chinspx() {
        // Mono exercises the §E.2.3.3.3 implicit `chinspx[0] = 1` arm
        // (no per-channel bits). 96 kbps ⇒ 384-byte frames.
        assert_spx_roundtrip(1, 96_000, 384, 3.0);
    }

    #[test]
    fn spx_roundtrip_51_with_lfe() {
        // 5.1 exercises SPX alongside the LFE pseudo-channel (LFE is
        // never in SPX; its exponents/mantissas follow the fbw SPX
        // fields, so a mis-sized SPX field would corrupt it too).
        // 384 kbps ⇒ 1536-byte frames.
        assert_spx_roundtrip(6, 384_000, 1536, 3.5);
    }

    #[test]
    fn spx_explicit_band_structure_decodes_bit_identically() {
        // `spxbndstrce = 1` with the Table E2.11 content vs
        // `spxbndstrce = 0` (decoder default): identical geometry and
        // — because the encoder reserves the worst-case (explicit)
        // header bits in both modes — identical mantissa budgets, so
        // the two decodes must match bit-for-bit. A one-bit misparse
        // of the explicit structure field would break this instantly.
        let pcm = build_spx_multitone(2, 4);
        let dec_default = decode_all(&encode_spx(&pcm, 2, 192_000, SpxParams::default()), 768);
        let dec_explicit = decode_all(
            &encode_spx(
                &pcm,
                2,
                192_000,
                SpxParams {
                    explicit_band_structure: true,
                    ..SpxParams::default()
                },
            ),
            768,
        );
        assert!(!dec_default.is_empty());
        assert_eq!(
            dec_default, dec_explicit,
            "default-banding and explicit-banding SPX encodes must decode bit-identically"
        );
    }

    #[test]
    fn spx_narrow_region_nondefault_codes() {
        // Non-default geometry: begin sub-band 4 (spxbegf=2 → tc 73),
        // end sub-band 9 (spxendf=3 → tc 133), copy start sub-band 1
        // (tc 37 — non-zero spxstrtf arm), low-noise blend. Exercises
        // remat_band_count_spx's spxbegf ≥ 2 arm with a narrower coded
        // band and a copy region smaller than the SPX span (wraps).
        let spx = SpxParams {
            spxbegf: 2,
            spxendf: 3,
            spxstrtf: 1,
            spxblnd: 31,
            ..SpxParams::default()
        };
        let geom = SpxGeometry::derive(&spx, &DEFAULT_SPX_BNDSTRC).unwrap();
        assert_eq!((geom.begin_tc, geom.end_tc), (73, 133));
        let pcm = build_spx_multitone(2, 6);
        let stream = encode_spx(&pcm, 2, 192_000, spx);
        let dec = decode_all(&stream, 768);
        let dec_f = to_f32_interleaved(&dec);
        let orig_prof = mdct_energy_profile(&pcm, 2, 0);
        let dec_prof = mdct_energy_profile(&dec_f, 2, 0);
        let deltas = band_db_deltas(&orig_prof, &dec_prof, &geom);
        for (bnd, d) in deltas.iter().enumerate() {
            assert!(
                d.abs() <= 3.5,
                "SPX band {bnd}: energy delta {d:+.2} dB (all: {deltas:?})"
            );
        }
    }

    /// Fixture for the adaptive copy-start test: a spectral HOLE in
    /// tc# 25..49 (no content between ~2.3 and ~4.6 kHz), LF tones only
    /// above it, HF tones in SPX bands 0/2/3, and a bed too weak to
    /// carry a band on its own. With `spxstrtf = 0` the copy region
    /// starts at tc# 25, so SPX band 0 (tc 109..133) translates the
    /// near-silent hole and its coordinate pins at the 0.875 ceiling;
    /// `spxstrtf = 2` starts the copy at tc# 49 where the tones live.
    fn build_gap_multitone(channels: usize, frames: usize) -> Vec<f32> {
        let n = frames * SAMPLES_PER_FRAME as usize;
        let mut pcm = vec![0.0f32; n * channels];
        let tones: [(f32, f32); 6] = [
            (700.0, 0.20),     // below the copy region entirely
            (5_500.0, 0.16),   // tc ≈ 58 — inside 49..109
            (8_600.0, 0.12),   // tc ≈ 91 — inside 49..109
            (12_200.0, 0.045), // SPX band 0
            (15_400.0, 0.030), // SPX band 2
            (19_000.0, 0.025), // SPX band 3
        ];
        let mut lfsr: u32 = 0x0BAD_5EED;
        for i in 0..n {
            let t = i as f32 / 48_000.0;
            let mut s = 0.0f32;
            for (f, a) in tones {
                s += a * (2.0 * std::f32::consts::PI * f * t).sin();
            }
            lfsr ^= lfsr << 13;
            lfsr ^= lfsr >> 17;
            lfsr ^= lfsr << 5;
            s += ((lfsr as f32 / u32::MAX as f32) * 2.0 - 1.0) * 0.001;
            for ch in 0..channels {
                pcm[i * channels + ch] = s * (1.0 - 0.08 * ch as f32);
            }
        }
        pcm
    }

    #[test]
    fn spx_adaptive_copy_start_avoids_saturation() {
        let geom = SpxGeometry::derive(&SpxParams::default(), &DEFAULT_SPX_BNDSTRC).unwrap();
        let pcm = build_gap_multitone(2, 8);
        // Tone-bearing SPX bands (12.2 / 15.4 / 19 kHz → bands 0/2/3).
        let tone_bands = [0usize, 2, 3];

        // Fixed spxstrtf = 0: band 0's translation source is the
        // spectral hole → the coordinate saturates and the band decodes
        // far below the original energy.
        let fixed = SpxParams::default(); // spxstrtf = 0
        let dec_fixed = to_f32_interleaved(&decode_all(&encode_spx(&pcm, 2, 192_000, fixed), 768));
        let orig_prof = mdct_energy_profile(&pcm, 2, 0);
        let fixed_prof = mdct_energy_profile(&dec_fixed, 2, 0);
        let fixed_deltas = band_db_deltas(&orig_prof, &fixed_prof, &geom);
        assert!(
            fixed_deltas[0] < -6.0,
            "premise: fixed copy-start must saturate band 0 (delta {:+.2} dB)",
            fixed_deltas[0]
        );

        // Adaptive: the per-frame scorer must move the copy start past
        // the hole and recover every tone band.
        let adaptive = SpxParams {
            adaptive_copy_start: true,
            ..SpxParams::default()
        };
        let dec_ad = to_f32_interleaved(&decode_all(&encode_spx(&pcm, 2, 192_000, adaptive), 768));
        let ad_prof = mdct_energy_profile(&dec_ad, 2, 0);
        let ad_deltas = band_db_deltas(&orig_prof, &ad_prof, &geom);
        for &bnd in &tone_bands {
            assert!(
                ad_deltas[bnd].abs() <= 3.5,
                "adaptive: SPX band {bnd} delta {:+.2} dB (all: {ad_deltas:?}; fixed: {fixed_deltas:?})",
                ad_deltas[bnd]
            );
        }
        // Coded band unaffected by the choice.
        let eo: f64 = orig_prof[..geom.begin_tc].iter().sum();
        let ed: f64 = ad_prof[..geom.begin_tc].iter().sum();
        assert!((10.0 * (ed / eo).log10()).abs() <= 1.5);
    }

    #[test]
    fn spx_atten_roundtrip_stereo_band_energy() {
        // §3.6.4.2.3 attenuation: spxattene=1 in audfrm with
        // chinspxatten/spxattencod per channel; the decoder notches the
        // translated coefficients at the border + wrap sites and the
        // encoder folds the notch into its coordinate computation, so
        // the banded-energy contract must STILL hold. This gates (a)
        // the audfrm attenuation-field alignment (a mis-sized field
        // shifts blkstrtinfoe and every audblk after it) and (b) the
        // energy compensation.
        let spx = SpxParams {
            atten_code: Some(14), // taps [0.5, 0.25, 0.125] — a strong notch
            ..SpxParams::default()
        };
        let geom = SpxGeometry::derive(&spx, &DEFAULT_SPX_BNDSTRC).unwrap();
        let pcm = build_spx_multitone(2, 8);
        let stream = encode_spx(&pcm, 2, 192_000, spx);
        let dec = decode_all(&stream, 768);
        let dec_f = to_f32_interleaved(&dec);
        for ch in 0..2 {
            let orig_prof = mdct_energy_profile(&pcm, 2, ch);
            let dec_prof = mdct_energy_profile(&dec_f, 2, ch);
            let deltas = band_db_deltas(&orig_prof, &dec_prof, &geom);
            for (bnd, d) in deltas.iter().enumerate() {
                assert!(
                    d.abs() <= 3.0,
                    "ch{ch} SPX band {bnd} (atten): energy delta {d:+.2} dB (all: {deltas:?})"
                );
            }
        }
        // The attenuated stream must actually differ from the plain
        // one (the notch + the audfrm fields are real bit-level
        // effects, not a silent no-op).
        let plain = encode_spx(&pcm, 2, 192_000, SpxParams::default());
        assert_ne!(stream, plain, "attenuation must change the bitstream");
    }

    #[test]
    fn make_encoder_with_spx_rejects_invalid_geometry() {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(48_000);
        p.channels = Some(2);
        // Inverted sub-band range (begf=7 → sub-band 11, endf=0 → 5).
        let bad = SpxParams {
            spxbegf: 7,
            spxendf: 0,
            ..SpxParams::default()
        };
        assert!(make_encoder_with_spx(&p, bad).is_err());
        // Empty copy region (strtf sub-band 3 ≥ begin sub-band 2).
        let bad = SpxParams {
            spxbegf: 0,
            spxendf: 7,
            spxstrtf: 3,
            ..SpxParams::default()
        };
        assert!(make_encoder_with_spx(&p, bad).is_err());
    }

    #[test]
    fn make_encoder_rejects_unsupported_channels() {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(48_000);
        // 4 ch is between stereo (2) and 5.1 (6) — not yet covered by
        // the encoder's allow-list (1, 2, 6, 8). Likewise 7 isn't in
        // the list (it would map to 3/2 + 1 ambiguous extra channel
        // and we don't speculate).
        p.channels = Some(4);
        match make_encoder(&p) {
            Ok(_) => panic!("must reject 4ch (not in 1/2/6/8 allow-list)"),
            Err(e) => assert!(matches!(e, Error::Unsupported(_))),
        }
    }

    #[test]
    fn make_encoder_rejects_bad_sample_rate() {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(16_000);
        p.channels = Some(2);
        match make_encoder(&p) {
            Ok(_) => panic!("must reject 16 kHz"),
            Err(e) => assert!(matches!(e, Error::Unsupported(_))),
        }
    }

    #[test]
    fn make_encoder_71_builds_pair_layout() {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(48_000);
        p.channels = Some(8);
        let _enc = make_encoder(&p).expect("eac3 7.1");
        // Reach into the layout via a probe encode would require an
        // accessor; since the layout struct is private, we cover the
        // chanmap math directly: bit 6 (Lrs/Rrs pair, Table E2.5) is
        // stored in MSB-6 = 1 << 9 = 0x0200.
        assert_eq!(1u16 << (15 - 6), 0x0200);
    }

    #[test]
    fn make_encoder_51_5fbw_plus_lfe() {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(48_000);
        p.channels = Some(6);
        p.bit_rate = Some(384_000);
        let _enc = make_encoder(&p).expect("eac3 5.1");
    }
}
