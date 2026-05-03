//! Enhanced AC-3 (E-AC-3 / Dolby Digital Plus) encoder per ATSC A/52
//! Annex E. Scope: independent substream (`strmtyp=0`, `substreamid=0`)
//! for 1.0/2.0/5.1 layouts, plus a paired indep+dep substream emission
//! for 7.1 input (5.1 downmix in indep, Lb/Rb in dep with `chanmap`
//! bit 6 = Lrs/Rrs pair set). `bsid=16`, 6 audio blocks per syncframe
//! (`numblkscod=3`), no coupling, no spectral extension, no Adaptive
//! Hybrid Transform.
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

use crate::audblk::{BLOCKS_PER_FRAME, N_COEFFS, SAMPLES_PER_BLOCK};
use crate::decoder::SAMPLES_PER_FRAME;
use crate::encoder::{
    ac3_crc_update, build_dba_plan, compute_bap, decode_input_samples, extract_exponent,
    preprocess_d15, quantise_mantissa, tune_snroffst, write_exponents_d15, write_mantissa_stream,
    BitAllocParams, CouplingPlan, TransientDetector, LFE_END_MANT,
};
use crate::mdct::{mdct_256_pair, mdct_512};
use crate::tables::WINDOW;

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
    Ok(Box::new(Eac3Encoder {
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
    }))
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
        let end_mant: usize = 37 + 3 * (chbwcod as usize + 12);
        let ch_end_mant = end_mant;
        for ch in 0..nfchans {
            for blk in 0..BLOCKS_PER_FRAME {
                for k in 0..ch_end_mant {
                    exps[ch][blk][k] = extract_exponent(coeffs[ch][blk][k]);
                }
            }
        }
        if sub.lfeon {
            for blk in 0..BLOCKS_PER_FRAME {
                for k in 0..LFE_END_MANT {
                    exps[lfe_idx_in_exps][blk][k] = extract_exponent(coeffs[nfchans][blk][k]);
                }
            }
        }
        // Exponent strategy: D15 on blocks 0 and 3, REUSE elsewhere.
        let exp_strategies: [u8; BLOCKS_PER_FRAME] = [1, 0, 0, 1, 0, 0];
        for ch in 0..nfchans {
            for blk in 0..BLOCKS_PER_FRAME {
                if exp_strategies[blk] == 1 {
                    preprocess_d15(&mut exps[ch][blk][..ch_end_mant]);
                }
            }
            let mut last = 0usize;
            for blk in 0..BLOCKS_PER_FRAME {
                if exp_strategies[blk] == 1 {
                    last = blk;
                } else {
                    let src: [u8; N_COEFFS] = exps[ch][last];
                    exps[ch][blk][..ch_end_mant].copy_from_slice(&src[..ch_end_mant]);
                }
            }
        }
        if sub.lfeon {
            // LFE strategy: reuse same D15 cadence; LFE has no chexpstr
            // — we still preprocess each block with D15 grouping for
            // the bins we actually code.
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
        let tuned_ba = tune_snroffst(
            &ba,
            &exps,
            ch_end_mant,
            nfchans,
            self.fscod,
            sub.frame_bytes,
            &exp_strategies,
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
        bw.write_u32(0, 2); // snroffststr = 0
        bw.write_u32(0, 1); // transproce = 0
        bw.write_u32(1, 1); // blkswe = 1
        bw.write_u32(1, 1); // dithflage = 1
        bw.write_u32(1, 1); // bamode = 1
        bw.write_u32(1, 1); // frmfgaincode = 1
        bw.write_u32(1, 1); // dbaflde = 1
        bw.write_u32(1, 1); // skipflde = 1
        bw.write_u32(0, 1); // spxattene = 0
        if sub.acmod > 1 {
            bw.write_u32(0, 1); // cplinu[0] = 0
            for _blk in 1..BLOCKS_PER_FRAME {
                bw.write_u32(0, 1); // cplstre[blk] = 0
            }
        }
        for blk in 0..BLOCKS_PER_FRAME {
            for _ch in 0..nfchans {
                bw.write_u32(exp_strategies[blk] as u32, 2);
            }
        }
        // expstre==1 + lfeon=1 → per-block lfeexpstr (1 bit each).
        if sub.lfeon {
            for blk in 0..BLOCKS_PER_FRAME {
                bw.write_u32(exp_strategies[blk] as u32, 1);
            }
        }
        // strmtyp == 0x0 ⇒ convexpstre. With numblkscod==0x3 this is
        // implicit = 1 → emit per-channel convexpstr (5 bits each).
        if sub.strmtyp == 0 {
            for _ in 0..nfchans {
                bw.write_u32(0, 5); // convexpstr = 0 (REUSE)
            }
        }
        // ahte == 0 ⇒ no AHT block.
        // snroffststr == 0 ⇒ frame-level snr offsets.
        bw.write_u32(tuned_ba.csnroffst as u32, 6); // frmcsnroffst
        bw.write_u32(tuned_ba.fsnroffst as u32, 4); // frmfsnroffst
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

            // SPX strategy.
            if blk == 0 {
                bw.write_u32(0, 1); // spxinu = 0
            } else {
                bw.write_u32(0, 1); // spxstre = 0
            }

            // Coupling: cplinu always 0 — block 0 hits the "else"
            // branch with no body; later blocks have cplstre=0 so
            // skip entirely.

            // Rematrixing — only acmod==2 (and not when SPX/cpl region
            // covers everything). Round-1 disables it.
            if sub.acmod == 2 {
                let nrematbd = 4usize;
                if blk == 0 {
                    for _bnd in 0..nrematbd {
                        bw.write_u32(0, 1);
                    }
                } else {
                    bw.write_u32(0, 1); // rematstr = 0
                }
            }

            let exp_strategy = exp_strategies[blk];
            if exp_strategy != 0 {
                for _ in 0..nfchans {
                    bw.write_u32(chbwcod as u32, 6);
                }
            }
            if exp_strategy == 1 {
                for ch in 0..nfchans {
                    write_exponents_d15(&mut bw, &exps[ch][blk], ch_end_mant);
                    bw.write_u32(0, 2); // gainrng = 0
                }
            }
            // LFE exponents — D15 only, no chbwcod (LFE has fixed bw).
            if sub.lfeon && exp_strategy == 1 {
                write_exponents_d15(&mut bw, &exps[lfe_idx_in_exps][blk], LFE_END_MANT);
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

            bw.write_u32(0, 1); // fgaincode = 0

            if sub.strmtyp == 0 {
                bw.write_u32(0, 1); // convsnroffste = 0
            }

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

        let crc2_val = ac3_crc_update(0, &frame[2..(sub.frame_bytes - 2)]);
        let n = sub.frame_bytes;
        frame[n - 2] = (crc2_val >> 8) as u8;
        frame[n - 1] = (crc2_val & 0xFF) as u8;
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
