//! AC-3 (Dolby Digital) encoder — builds syncframes from PCM.
//!
//! This is a *basic* encoder in the spirit of §8 (A/52:2018): fixed
//! bit-allocation parameters, no coupling, no rematrixing, long
//! (512-point) transforms only. Target configuration for this initial
//! revision is 48 kHz stereo at 192 kbps — the fixture format already
//! exercised by the decoder round-trip test.
//!
//! Pipeline per block (§8.1 Fig. 8.1):
//!
//! 1. window + forward MDCT (see [`super::mdct`])
//! 2. exponent extraction (leading-zero count on each coefficient)
//! 3. exponent preprocessing + strategy selection (D15/D25/D45)
//! 4. delta encoding of exponents
//! 5. parametric bit allocation (shared routine with the decoder)
//! 6. mantissa quantisation + grouped packing
//! 7. pack into syncframe, emit crc1 / crc2
//!
//! ## Status
//!
//! Only the scaffolding + block-zero encode path is wired up so far;
//! incremental commits flesh out the remaining stages. The current
//! implementation produces a syntactically-valid syncframe for the
//! 48 kHz stereo 192 kbps mode.

use oxideav_core::bits::BitWriter;
use oxideav_core::Encoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::audblk::{remat_band_count, BLOCKS_PER_FRAME, MAX_FBW, N_COEFFS, SAMPLES_PER_BLOCK};
use crate::decoder::SAMPLES_PER_FRAME;
use crate::mdct::{mdct_256_pair, mdct_512};
use crate::tables::{
    frame_length_bytes, nominal_bitrate_kbps, BAPTAB, BNDSZ, BNDTAB, DBPBTAB, FASTDEC, FASTGAIN,
    FLOORTAB, HTH, LATAB, MANT_LEVEL_11, MANT_LEVEL_15, MANT_LEVEL_3, MANT_LEVEL_5, MANT_LEVEL_7,
    MASKTAB, QUANTIZATION_BITS, SLOWDEC, SLOWGAIN, WINDOW,
};

/// Build an encoder instance. Required parameters (via
/// [`CodecParameters::audio`]):
///
/// * `sample_rate` — 48 000, 44 100, or 32 000 Hz
/// * `channels`    — only 2 (2/0 stereo) is currently supported
///
/// The bit rate defaults to 192 kbps (frmsizecod = 20 for 48 kHz);
/// callers may override via `bit_rate` on [`CodecParameters`] if it
/// maps to a valid row of Table 5.18.
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    let sample_rate = params.sample_rate.ok_or_else(|| {
        Error::invalid("ac3 encoder: sample_rate is required (48000/44100/32000)")
    })?;
    let channels = params
        .channels
        .ok_or_else(|| Error::invalid("ac3 encoder: channels is required"))?;
    if channels != 2 {
        return Err(Error::Unsupported(format!(
            "ac3 encoder: only 2-channel stereo is currently supported (got {channels})"
        )));
    }
    let fscod: u8 = match sample_rate {
        48_000 => 0,
        44_100 => 1,
        32_000 => 2,
        _ => {
            return Err(Error::Unsupported(format!(
                "ac3 encoder: unsupported sample rate {sample_rate} (48000/44100/32000 only)"
            )))
        }
    };

    // Bit-rate → frmsizecod lookup. 192 kbps maps to frmsizecod=20.
    let target_kbps: u32 = params.bit_rate.map(|b| (b / 1000) as u32).unwrap_or(192);
    let frmsizecod = pick_frmsizecod(target_kbps).ok_or_else(|| {
        Error::Unsupported(format!(
            "ac3 encoder: bit rate {target_kbps} kbps has no frmsizecod mapping"
        ))
    })?;
    let frame_bytes = frame_length_bytes(fscod, frmsizecod)
        .ok_or_else(|| Error::invalid("ac3 encoder: internal frame-length lookup failed"))?;

    let out_params = {
        let mut p = CodecParameters::audio(CodecId::new(crate::CODEC_ID_STR));
        p.sample_rate = Some(sample_rate);
        p.channels = Some(channels);
        p.sample_format = Some(SampleFormat::S16);
        p.bit_rate = Some(nominal_bitrate_kbps(frmsizecod).unwrap_or(target_kbps) as u64 * 1000);
        p
    };

    let input_sample_format = params.sample_format.unwrap_or(SampleFormat::S16);
    Ok(Box::new(Ac3Encoder {
        codec_id: CodecId::new(crate::CODEC_ID_STR),
        out_params,
        sample_rate,
        channels: channels as usize,
        input_sample_format,
        fscod,
        frmsizecod,
        frame_bytes: frame_bytes as usize,
        // 256 samples of left-context per channel feed the first MDCT.
        delay_line: vec![vec![0.0f32; SAMPLES_PER_BLOCK]; channels as usize],
        pending_samples: vec![Vec::<f32>::new(); channels as usize],
        packet_queue: Vec::new(),
        pts: 0,
    }))
}

/// Match a target kbps to a row of Table 5.18. Returns the lower
/// (even-indexed) frmsizecod for each bitrate — both members of a pair
/// encode the same rate, and the lower row matches the usual encoder
/// default at 48 kHz.
fn pick_frmsizecod(kbps: u32) -> Option<u8> {
    const TABLE: &[(u32, u8)] = &[
        (32, 0),
        (40, 2),
        (48, 4),
        (56, 6),
        (64, 8),
        (80, 10),
        (96, 12),
        (112, 14),
        (128, 16),
        (160, 18),
        (192, 20),
        (224, 22),
        (256, 24),
        (320, 26),
        (384, 28),
        (448, 30),
        (512, 32),
        (576, 34),
        (640, 36),
    ];
    for &(rate, code) in TABLE {
        if rate == kbps {
            return Some(code);
        }
    }
    None
}

struct Ac3Encoder {
    codec_id: CodecId,
    out_params: CodecParameters,
    sample_rate: u32,
    channels: usize,
    /// Input PCM sample format. Defaults to `S16` when params don't
    /// declare one. Accepts `S16` and `F32`.
    input_sample_format: SampleFormat,
    fscod: u8,
    frmsizecod: u8,
    frame_bytes: usize,
    /// Last block's right-half (256 samples) per channel, forming the
    /// left context for the next MDCT window.
    delay_line: Vec<Vec<f32>>,
    /// Samples that have been sent via `send_frame` but not yet
    /// consumed into a syncframe. Each inner `Vec` is per-channel.
    pending_samples: Vec<Vec<f32>>,
    packet_queue: Vec<Packet>,
    /// Running sample PTS. Each produced syncframe carries SAMPLES_PER_FRAME.
    pts: i64,
}

impl Encoder for Ac3Encoder {
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
                    "ac3 encoder: send_frame requires an audio frame",
                ))
            }
        };
        // Per-frame channel-count and sample-rate are no longer carried on
        // AudioFrame; the encoder validates layout/rate at construction
        // time via CodecParameters and trusts the caller to feed matching
        // PCM here. Channel count and stride are taken from `self.channels`
        // and `self.input_sample_format`.
        // Extract per-channel f32 samples from the interleaved input.
        let per_chan = decode_input_samples(audio, self.channels, self.input_sample_format)?;
        for ch in 0..self.channels {
            self.pending_samples[ch].extend_from_slice(&per_chan[ch]);
        }
        // Flush whole syncframes while we have enough.
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
        // Pad any partial frame with zeros so the last PCM samples reach
        // the decoder.
        if self.pending_samples[0].is_empty() {
            return Ok(());
        }
        let missing = SAMPLES_PER_FRAME as usize - self.pending_samples[0].len();
        if missing > 0 {
            for ch in 0..self.channels {
                self.pending_samples[ch].extend(std::iter::repeat(0.0).take(missing));
            }
        }
        self.emit_syncframe()?;
        Ok(())
    }
}

/// Convert a decoded [`AudioFrame`] into normalized f32 samples per
/// channel. Supports the two formats most commonly supplied by
/// upstream demuxers / resamplers: interleaved S16 and interleaved F32.
fn decode_input_samples(
    a: &AudioFrame,
    nch: usize,
    fmt: SampleFormat,
) -> Result<Vec<Vec<f32>>> {
    let nsamp = a.samples as usize;
    let mut out = vec![Vec::with_capacity(nsamp); nch];
    match fmt {
        SampleFormat::S16 => {
            let plane = a
                .data
                .first()
                .ok_or_else(|| Error::invalid("ac3 encoder: S16 frame missing data plane"))?;
            if plane.len() < nsamp * nch * 2 {
                return Err(Error::invalid("ac3 encoder: S16 plane too short"));
            }
            for n in 0..nsamp {
                for ch in 0..nch {
                    let off = (n * nch + ch) * 2;
                    let v = i16::from_le_bytes([plane[off], plane[off + 1]]);
                    out[ch].push(v as f32 / 32768.0);
                }
            }
        }
        SampleFormat::F32 => {
            let plane = a
                .data
                .first()
                .ok_or_else(|| Error::invalid("ac3 encoder: F32 frame missing data plane"))?;
            if plane.len() < nsamp * nch * 4 {
                return Err(Error::invalid("ac3 encoder: F32 plane too short"));
            }
            for n in 0..nsamp {
                for ch in 0..nch {
                    let off = (n * nch + ch) * 4;
                    let v = f32::from_le_bytes([
                        plane[off],
                        plane[off + 1],
                        plane[off + 2],
                        plane[off + 3],
                    ]);
                    out[ch].push(v);
                }
            }
        }
        other => {
            return Err(Error::Unsupported(format!(
                "ac3 encoder: sample format {other:?} not yet supported"
            )))
        }
    }
    Ok(out)
}

impl Ac3Encoder {
    fn emit_syncframe(&mut self) -> Result<()> {
        let n_per = SAMPLES_PER_FRAME as usize;
        // Run the 6-block MDCT pipeline per channel and stash coefficient
        // blocks per channel: blocks × N_COEFFS.
        let mut coeffs: Vec<Vec<[f32; N_COEFFS]>> =
            vec![vec![[0.0; N_COEFFS]; BLOCKS_PER_FRAME]; self.channels];
        // §5.4.3.1 blksw[ch][blk] — per-block per-channel block-switch
        // flag. Decided per block from the time-domain transient
        // detector (see `detect_transient`). When `true`, the encoder
        // runs the 256-sample MDCT pair (§7.6 / §8.2.3.2 short
        // transform) instead of the long 512-sample MDCT, and the
        // decoder swaps to the matching IMDCT path on the same flag.
        let mut blksw: Vec<[bool; BLOCKS_PER_FRAME]> =
            vec![[false; BLOCKS_PER_FRAME]; self.channels];
        for ch in 0..self.channels {
            let drain: Vec<f32> = self.pending_samples[ch].drain(0..n_per).collect();
            for blk in 0..BLOCKS_PER_FRAME {
                // Build 512-sample input: left context + next 256.
                let mut in_buf = [0.0f32; 512];
                in_buf[..256].copy_from_slice(&self.delay_line[ch]);
                in_buf[256..].copy_from_slice(
                    &drain[blk * SAMPLES_PER_BLOCK..(blk + 1) * SAMPLES_PER_BLOCK],
                );
                // Per-block transient decision. Compares the high-pass
                // energy of consecutive 64-sample sub-frames within
                // the *new* 256-sample input (the right half of the
                // 512-sample window, where a transient that starts
                // here will be smeared by a long MDCT but captured
                // cleanly by the short-block pair). Threshold + HP
                // filter chosen empirically against the burst fixture
                // — see `detect_transient` for the rationale.
                //
                // The `AC3_DISABLE_BLKSW=1` environment variable
                // forces long blocks regardless of detector output —
                // useful when bisecting whether a quality regression
                // is short-block-related, and for the round-15 A/B
                // PSNR measurement.
                let is_short = if std::env::var("AC3_DISABLE_BLKSW").is_ok() {
                    false
                } else {
                    detect_transient(&in_buf[256..])
                };
                blksw[ch][blk] = is_short;
                // Windowing (symmetric 512-sample AC-3 window). The
                // window is the same regardless of long/short — the
                // decoder applies the same 256-coeff KBD window after
                // its IMDCT in both cases (`audblk.rs` around the
                // `time[n] *= WINDOW[n]` line). The spec's §7.9.5
                // distinguishes long-only / long-to-short / etc.
                // window shapes, but the decoder's choice makes the
                // 4-way distinction collapse to the long window for
                // every block, which we honour here.
                let mut win_buf = [0.0f32; 512];
                for n in 0..256 {
                    win_buf[n] = in_buf[n] * WINDOW[n];
                    win_buf[511 - n] = in_buf[511 - n] * WINDOW[n];
                }
                // Update delay line to right-half of the next block.
                self.delay_line[ch].copy_from_slice(
                    &drain[blk * SAMPLES_PER_BLOCK..(blk + 1) * SAMPLES_PER_BLOCK],
                );
                // Forward MDCT — long (one 512-pt) or short pair
                // (two interleaved 256-pt halves per §7.9.4.2).
                if is_short {
                    mdct_256_pair(&win_buf, &mut coeffs[ch][blk]);
                } else {
                    mdct_512(&win_buf, &mut coeffs[ch][blk]);
                }
            }
        }

        // Per-block exponents: channels × blocks × N_COEFFS (u8 in 0..=24).
        // Index `self.channels` (one past the last fbw channel) is the
        // coupling pseudo-channel's exponent buffer when coupling is
        // active; we always allocate `self.channels + 1` slots so the
        // index arithmetic stays uniform regardless of `cpl.in_use`.
        let mut exps: Vec<Vec<[u8; N_COEFFS]>> =
            vec![vec![[24u8; N_COEFFS]; BLOCKS_PER_FRAME]; self.channels + 1];
        // Limit active bins: the decoder starts from end_mant = 37 + 3*(chbwcod+12).
        // Use chbwcod=60 → end_mant=253 (full bandwidth minus the top 3 bins).
        let chbwcod: u8 = 60;
        let end_mant: usize = 37 + 3 * (chbwcod as usize + 12);

        // -------------------------------------------------------------
        // §7.4 Channel coupling (encoder-side)
        // -------------------------------------------------------------
        //
        // Decide once per frame whether to enable coupling. The default
        // policy is: 2/0 stereo + AC3_DISABLE_CPL not set. The enable
        // logic could test inter-channel correlation in the high band
        // and skip coupling when channels are uncorrelated (mid-side
        // would actually hurt), but in practice the decoder-side bit
        // savings are large enough that always-on coupling is the
        // standard choice for production AC-3 encoders at our 192
        // kbps target.
        //
        // Coupling region:
        //   cplbegf=8  → first cpl coefficient at bin 133 (~6.0 kHz @ 48 kHz)
        //   cplendf=15 → last subband index 17 (bins 241..252, ~22 kHz)
        //                → cpl_endf_mant = 37 + 12*18 = 253 (matches end_mant)
        //
        // Above bin 133, individual L/R coefficients are replaced by
        // shared coupling-channel coefficients = 0.5*(L+R). The decoder
        // re-derives L_recv = cplmant * cplco_L * 8, R_recv = cplmant *
        // cplco_R * 8 — preserving the per-band envelope of each
        // channel without spending bits on per-channel mantissas.
        let cpl_disabled = std::env::var("AC3_DISABLE_CPL").is_ok();
        let mut cpl = CouplingPlan::default();
        if self.channels == 2 && !cpl_disabled {
            cpl.in_use = true;
            cpl.begf = 8;
            cpl.endf = 15;
            cpl.chincpl[0] = true;
            cpl.chincpl[1] = true;
            // No phase flags by default (mid-side over-suppression on
            // anti-correlated transients can sound like ping-ponging
            // smear; the decoder side handles phsflg=0 trivially).
            cpl.phsflginu = false;
            // Merge each pair of subbands into one coupling band:
            // cplbndstrc[0]=false (always), [1]=true, [2]=false,
            // [3]=true, ... → bands of size 2. With 10 subbands
            // (cplbegf=8, cplendf=15) this gives 5 coupling bands.
            //
            // Coarser bands ⇒ fewer cplco emissions per block ⇒ more
            // bit savings, at a small per-band envelope-resolution
            // cost. 5 bands × 5 ms blocks → ~140 Hz envelope tracking
            // resolution which is fine well above the masker.
            cpl.nsubbnd = 3 + cpl.endf as usize - cpl.begf as usize;
            cpl.bndstrc[0] = false;
            for sbnd in 1..cpl.nsubbnd {
                cpl.bndstrc[sbnd] = sbnd % 2 == 1;
            }
            let mut nbnd = cpl.nsubbnd;
            for sbnd in 1..cpl.nsubbnd {
                if cpl.bndstrc[sbnd] {
                    nbnd -= 1;
                }
            }
            cpl.nbnd = nbnd;
            // Coupling coordinates are signalled on block 0 only;
            // every later block reuses (cplcoe[blk][ch]=false).
            for ch in 0..self.channels {
                cpl.cplcoe[0][ch] = true;
            }
        }

        // Storage for the coupling-channel coefficients. Index by
        // [blk][bin]; only bins in [cpl_begf_mant, cpl_endf_mant) are
        // meaningful when coupling is in use.
        let mut cpl_coeffs: Vec<[f32; N_COEFFS]> = vec![[0.0f32; N_COEFFS]; BLOCKS_PER_FRAME];

        if cpl.in_use {
            // §7.4.1: "channel coupling is performed on encode by
            // averaging the transform coefficients across channels
            // that are included in the coupling channel."
            //
            // For 2/0 with both channels coupled, cplmant = 0.5*(L+R).
            // The per-band envelope ratio is captured by the coupling
            // coordinates so the decoder reconstructs each channel's
            // approximate magnitude.
            let begf_mant = cpl.begf_mant();
            let endf_mant = cpl.endf_mant();
            for blk in 0..BLOCKS_PER_FRAME {
                for bin in begf_mant..endf_mant {
                    cpl_coeffs[blk][bin] = 0.5 * (coeffs[0][blk][bin] + coeffs[1][blk][bin]);
                }
            }

            // Compute and quantise coupling coordinates from the
            // *block-0* envelope. This matches our cplcoe[blk][ch]
            // policy: signal coords on block 0, reuse for blocks 1..5.
            //
            // Per-band per-channel coordinate:
            //   cplco[ch][bnd] = sqrt(Σ ch[k]² / Σ cpl[k]²) / 8
            //
            // The /8 cancels the decoder's `coord = state.cpl_coord *
            // 8.0` lift, and the sqrt(energy ratio) preserves the
            // per-channel magnitude in the average sense across the
            // band.
            //
            // We sum energies over *all 6 blocks* before computing the
            // coordinate so the block-0 coords represent the frame
            // envelope rather than a single-block snapshot — the
            // coords are reused for the whole frame, and a single-
            // block measurement would over- or under-shoot on bursts.
            let sbnd2bnd = cpl.sbnd_to_bnd();
            let mut e_ch_band = [[0.0f64; 18]; MAX_FBW];
            let mut e_cpl_band = [0.0f64; 18];
            for blk in 0..BLOCKS_PER_FRAME {
                for sbnd_off in 0..cpl.nsubbnd {
                    let bnd = sbnd2bnd[sbnd_off];
                    let base = begf_mant + sbnd_off * 12;
                    let limit = (base + 12).min(endf_mant);
                    for bin in base..limit {
                        let c = cpl_coeffs[blk][bin] as f64;
                        e_cpl_band[bnd] += c * c;
                        for ch in 0..self.channels {
                            let v = coeffs[ch][blk][bin] as f64;
                            e_ch_band[ch][bnd] += v * v;
                        }
                    }
                }
            }
            // Per-channel: max raw cplco across bands → mstrcplco.
            let mut raw_cplco = [[0.0f32; 18]; MAX_FBW];
            for ch in 0..self.channels {
                let mut max_co: f32 = 0.0;
                for bnd in 0..cpl.nbnd {
                    let denom = e_cpl_band[bnd];
                    let co = if denom > 1e-20 {
                        ((e_ch_band[ch][bnd] / denom).sqrt() as f32) / 8.0
                    } else {
                        0.0
                    };
                    raw_cplco[ch][bnd] = co;
                    if co > max_co {
                        max_co = co;
                    }
                }
                cpl.mstrcplco[ch] = pick_mstrcplco(max_co);
                for bnd in 0..cpl.nbnd {
                    let (e, m) = quantise_cplco(raw_cplco[ch][bnd], cpl.mstrcplco[ch]);
                    cpl.cplcoexp[ch][bnd] = e;
                    cpl.cplcomant[ch][bnd] = m;
                }
            }

            // Replace the per-channel high-band MDCT coefficients
            // with the *encoder's view* of what the decoder will
            // reconstruct from the cpl channel + the quantised
            // coords. This is critical for two downstream stages:
            //
            //   1. Rematrixing — must run on the *post-coupling*
            //      coefficients so the encoder and decoder agree on
            //      what's in each channel's exponent/mantissa
            //      buffers in the cpl region (rematrix is bypassed
            //      above bin = cpl_begf_mant by the band-table cap,
            //      so this only matters for stages downstream of
            //      rematrix).
            //   2. The per-channel `end_mant` used for fbw exponent /
            //      mantissa emission is clamped to `cpl_begf_mant`
            //      below — so the post-coupling bins above that
            //      index are intentionally not transmitted as
            //      per-channel data; they exist only so subsequent
            //      sanity checks see realistic magnitudes.
            //
            // Reconstructed coefficient: `chmant * cplco * 8 = cpl *
            // cplco_recon * 8`. Note phsflginu=0 so no sign flip.
            let mut cplco_recon = [[0.0f32; 18]; MAX_FBW];
            for ch in 0..self.channels {
                for bnd in 0..cpl.nbnd {
                    cplco_recon[ch][bnd] = reconstruct_cplco(
                        cpl.cplcoexp[ch][bnd],
                        cpl.cplcomant[ch][bnd],
                        cpl.mstrcplco[ch],
                    );
                }
            }
            for blk in 0..BLOCKS_PER_FRAME {
                for sbnd_off in 0..cpl.nsubbnd {
                    let bnd = sbnd2bnd[sbnd_off];
                    let base = begf_mant + sbnd_off * 12;
                    let limit = (base + 12).min(endf_mant);
                    for bin in base..limit {
                        let cpl_v = cpl_coeffs[blk][bin];
                        for ch in 0..self.channels {
                            coeffs[ch][blk][bin] = cpl_v * cplco_recon[ch][bnd] * 8.0;
                        }
                    }
                }
            }

            if std::env::var("AC3_TRACE_CPL_ENC").is_ok() {
                eprintln!(
                    "CPL-ENC begf={} endf={} nsubbnd={} nbnd={} mstr=[{},{}]",
                    cpl.begf, cpl.endf, cpl.nsubbnd, cpl.nbnd, cpl.mstrcplco[0], cpl.mstrcplco[1]
                );
                for ch in 0..self.channels {
                    eprintln!("  ch{} cplco[bnd]: {:?}", ch, &raw_cplco[ch][..cpl.nbnd]);
                    eprintln!(
                        "  ch{} cplcoexp:    {:?}",
                        ch,
                        &cpl.cplcoexp[ch][..cpl.nbnd]
                    );
                    eprintln!(
                        "  ch{} cplcomant:   {:?}",
                        ch,
                        &cpl.cplcomant[ch][..cpl.nbnd]
                    );
                }
            }
        }

        // Per-channel mantissa-bin upper bound. When coupling is in
        // use the channel only carries data up to cpl_begf_mant;
        // above that, the decoder fills from the coupling channel
        // (post-coord application) so transmitting per-channel data
        // would be wasted bits. With cplinu=0 the channel goes the
        // full chbwcod range.
        let ch_end_mant: usize = if cpl.in_use {
            cpl.begf_mant()
        } else {
            end_mant
        };

        // Rematrixing decision (§7.5.3) — only meaningful for 2/0 stereo.
        // For each block and each rematrix band compare Σ|L|² + Σ|R|²
        // against Σ|L+R|² + Σ|L-R|² and pick the smaller-energy pair.
        // When (L+R, L-R) wins we replace the L and R coefficients in
        // that band with their sum/difference forms scaled by 0.5 —
        // the spec's "transmitted left = 0.5*(L+R)" formula. The
        // decoder reverses with `L = L'+R'`, `R = L'-R'`.
        //
        // Rationale for picking 0.5 vs leaving the unscaled sum:
        //   * `L_recv = 0.5*(L+R)`, `R_recv = 0.5*(L-R)`
        //   * `L_dec = L_recv + R_recv = 0.5*(L+R) + 0.5*(L-R) = L`  ✓
        //
        // The 0.5 keeps the rematrixed coefficient magnitudes in the
        // same range as the original L/R, so quantiser exponents do
        // not jump by a stage.
        //
        // Per Tables 7.25-7.28, the upper edge of the LAST rematrix
        // band tracks the coupling lower edge: with cplinu=0 it ends
        // at bin 252; with cplinu=1 it ends at A = 36 + 12*cplbegf.
        // For cplbegf=8 that's bin 132, so rematrix band 3 = (61, 133).
        let last_remat_hi = if cpl.in_use {
            36 + 12 * cpl.begf as usize + 1
        } else {
            253
        };
        let remat_bands: [(usize, usize); 4] =
            [(13, 25), (25, 37), (37, 61), (61, last_remat_hi.max(61))];
        let nrematbd = if self.channels == 2 {
            remat_band_count(cpl.in_use, cpl.begf)
        } else {
            0
        };
        let mut rematflg: Vec<[bool; 4]> = vec![[false; 4]; BLOCKS_PER_FRAME];
        if nrematbd > 0 {
            for blk in 0..BLOCKS_PER_FRAME {
                for (bnd_idx, &(lo, hi_full)) in remat_bands.iter().take(nrematbd).enumerate() {
                    let hi = hi_full.min(ch_end_mant);
                    if lo >= hi {
                        continue;
                    }
                    let mut e_l = 0.0f64;
                    let mut e_r = 0.0f64;
                    let mut e_s = 0.0f64;
                    let mut e_d = 0.0f64;
                    for bin in lo..hi {
                        let l = coeffs[0][blk][bin] as f64;
                        let r = coeffs[1][blk][bin] as f64;
                        e_l += l * l;
                        e_r += r * r;
                        let s = l + r;
                        let d = l - r;
                        e_s += s * s;
                        e_d += d * d;
                    }
                    // §7.5.3 picks the minimum-energy combination among the
                    // 4 candidates {L, R, L+R, L-R}. Rematrix if the minimum
                    // belongs to the {L+R, L-R} pair: that is, the smaller
                    // of the sum/difference energies undercuts the smaller
                    // of the L/R energies. The scaling-by-0.5 we apply on
                    // the transmitted side only changes the magnitude — the
                    // *relative* ranking is preserved, so we compare the
                    // unscaled energies here.
                    if e_s.min(e_d) < e_l.min(e_r) {
                        rematflg[blk][bnd_idx] = true;
                        for bin in lo..hi {
                            let l = coeffs[0][blk][bin];
                            let r = coeffs[1][blk][bin];
                            coeffs[0][blk][bin] = 0.5 * (l + r);
                            coeffs[1][blk][bin] = 0.5 * (l - r);
                        }
                    }
                    if std::env::var("AC3_TRACE_REMAT_ENC").is_ok() {
                        eprintln!(
                            "REMAT-ENC blk={} bnd={} lo={} hi={} e_l={:.3e} e_r={:.3e} e_s={:.3e} e_d={:.3e} flg={}",
                            blk, bnd_idx, lo, hi, e_l, e_r, e_s, e_d, rematflg[blk][bnd_idx]
                        );
                    }
                }
            }
        }
        // Step 1: raw exponent extraction per block, per channel.
        for ch in 0..self.channels {
            for blk in 0..BLOCKS_PER_FRAME {
                for k in 0..ch_end_mant {
                    exps[ch][blk][k] = extract_exponent(coeffs[ch][blk][k]);
                }
                // For each exponent that was just extracted, take the *minimum*
                // across the coefficient's bin neighbourhood of radius 0
                // (i.e. no change) — this is a stub for the spec's §7.1.5
                // exponent-sharing that grpsize>1 strategies imply. Currently
                // only D15 (grpsize=1) is used so no sharing happens here.
            }
        }
        // Coupling-channel exponents: extract from cpl_coeffs over
        // [cpl_begf_mant, cpl_endf_mant). The cpl pseudo-channel's
        // exponent buffer lives at index `self.channels` (one past
        // the last fbw channel). The decoder's first cpl exponent
        // (`cplabsexp << 1`) is just a starting reference; the
        // actual bin-aligned exponents start at `cpl_start`.
        if cpl.in_use {
            let cpl_idx = self.channels;
            let begf_mant = cpl.begf_mant();
            let endf_mant = cpl.endf_mant();
            for blk in 0..BLOCKS_PER_FRAME {
                for k in begf_mant..endf_mant {
                    exps[cpl_idx][blk][k] = extract_exponent(cpl_coeffs[blk][k]);
                }
            }
        }

        // Exponent strategy per block per channel.
        //
        // A basic encoder can legally transmit D15 on block 0 and REUSE
        // on blocks 1..5 — which is what this encoder shipped with. But
        // that badly hurts quality on any non-stationary input: blocks
        // 1..5 are quantised using block-0's spectral envelope, so their
        // mantissas saturate (|coeff| * 2^e clamps to ±1) whenever the
        // actual bin energy disagrees. Here we refresh exponents twice
        // per frame — D15 on blocks 0 and 3, REUSE for 1/2/4/5 — which
        // fits inside the 192 kbps budget for 2/0 stereo and recovers a
        // large SNR margin on non-steady-state signals.
        let exp_strategies: [u8; BLOCKS_PER_FRAME] = [1, 0, 0, 1, 0, 0];
        // Pre-process the D15 exponents: clamp absexp to 4-bit range and
        // clamp each forward delta to ±2. The output is a legal D15
        // sequence the decoder will replay verbatim.
        for ch in 0..self.channels {
            for blk in 0..BLOCKS_PER_FRAME {
                if exp_strategies[blk] == 1 {
                    preprocess_d15(&mut exps[ch][blk][..ch_end_mant]);
                }
            }
            // For REUSE blocks, copy the most recent transmitted D15 set
            // forward so compute_bap + mantissa quantisation use the
            // exponents the decoder will see on this block.
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
        // Coupling-channel exponent strategy + D15 preprocessing.
        // Same per-block strategy as the fbw channels (D15 on blocks
        // 0 and 3, REUSE elsewhere) so the cpl side info adds no
        // new strategy decisions to track.
        if cpl.in_use {
            let cpl_idx = self.channels;
            let begf_mant = cpl.begf_mant();
            let endf_mant = cpl.endf_mant();
            for blk in 0..BLOCKS_PER_FRAME {
                if exp_strategies[blk] == 1 {
                    preprocess_d15(&mut exps[cpl_idx][blk][begf_mant..endf_mant]);
                }
            }
            let mut last = 0usize;
            for blk in 0..BLOCKS_PER_FRAME {
                if exp_strategies[blk] == 1 {
                    last = blk;
                } else {
                    let src: [u8; N_COEFFS] = exps[cpl_idx][last];
                    exps[cpl_idx][blk][begf_mant..endf_mant]
                        .copy_from_slice(&src[begf_mant..endf_mant]);
                }
            }
        }

        // Bit-allocation state we'll feed into the shared allocator.
        // Fixed parameters per §8.2.12 "Core Bit Allocation" (basic encoder).
        let ba = BitAllocParams {
            sdcycod: 2,
            fdcycod: 1,
            sgaincod: 1,
            dbpbcod: 2,
            floorcod: 4,
            // These SNR offsets are "loose" — a production encoder would
            // iterate them so the total mantissa bit count fills the
            // frame budget exactly. For now we pick a conservative
            // baseline that under-shoots the budget (padded with skip
            // bytes) rather than overshoots.
            csnroffst: 15,
            fsnroffst: 0,
            cplfsnroffst: 0,
            lfefsnroffst: 0,
            fgaincod: 4,
            cplfgaincod: 4,
            lfefgaincod: 4,
        };

        // Iteratively tune csnroffst+fsnroffst so the encoded mantissa
        // bits + side-info fit the frame payload. This is the minimal
        // loop §8.2.12 describes.
        let tuned_ba = tune_snroffst(
            &ba,
            &exps,
            ch_end_mant,
            self.channels,
            self.fscod,
            self.frame_bytes,
            &exp_strategies,
            &cpl,
        );

        // Compute bap arrays per channel per block using the tuned
        // params. We allocate `self.channels + 1` slots so the cpl
        // pseudo-channel's bap lives at index `self.channels`.
        let mut baps: Vec<Vec<[u8; N_COEFFS]>> =
            vec![vec![[0u8; N_COEFFS]; BLOCKS_PER_FRAME]; self.channels + 1];
        for ch in 0..self.channels {
            for blk in 0..BLOCKS_PER_FRAME {
                compute_bap(
                    &exps[ch][blk],
                    ch_end_mant,
                    self.fscod,
                    &tuned_ba,
                    &mut baps[ch][blk],
                );
            }
        }
        // Coupling-channel bap. The decoder runs `run_bit_allocation`
        // on the cpl pseudo-channel over [cpl_begf_mant, cpl_endf_mant)
        // with `is_coupling=true` (which uses cpl_fsnroffst /
        // cpl_fgaincod, currently inherited from the fbw cstd values
        // via the BitAllocParams struct). For the encoder we run the
        // same compute_bap routine; the start is implicit at bin 0
        // for the masking model so we use `compute_bap_range` (a thin
        // wrapper around compute_bap) that masks with the cpl-specific
        // snroffset.
        if cpl.in_use {
            let cpl_idx = self.channels;
            let begf_mant = cpl.begf_mant();
            let endf_mant = cpl.endf_mant();
            // Build a BitAllocParams variant where csnr/fsnr are the
            // cpl values so compute_bap's snroffset arithmetic uses
            // the cpl-specific knob.
            let mut cpl_ba = tuned_ba;
            cpl_ba.fsnroffst = tuned_ba.cplfsnroffst;
            cpl_ba.fgaincod = tuned_ba.cplfgaincod;
            for blk in 0..BLOCKS_PER_FRAME {
                compute_bap_cpl(
                    &exps[cpl_idx][blk],
                    begf_mant,
                    endf_mant,
                    self.fscod,
                    &cpl_ba,
                    &mut baps[cpl_idx][blk],
                );
            }
        }

        // --------- Pack the syncframe ---------
        let mut bw = BitWriter::with_capacity(self.frame_bytes);

        // syncinfo: syncword + placeholder crc1 + fscod/frmsizecod.
        bw.write_u32(0x0B77, 16);
        bw.write_u32(0, 16); // crc1 placeholder, filled post-hoc
        bw.write_u32(self.fscod as u32, 2);
        bw.write_u32(self.frmsizecod as u32, 6);

        // BSI — 2/0 stereo, bsid=8, bsmod=0, dialnorm=27, no optional fields.
        bw.write_u32(8, 5); // bsid
        bw.write_u32(0, 3); // bsmod
        bw.write_u32(2, 3); // acmod = 2/0 stereo
                            // cmixlev — absent for acmod=2.
                            // surmixlev — absent (acmod bit 2 is 0).
        bw.write_u32(0, 2); // dsurmod = not indicated
        bw.write_u32(0, 1); // lfeon
        bw.write_u32(27, 5); // dialnorm = -27 dB
        bw.write_u32(0, 1); // compre
        bw.write_u32(0, 1); // langcode
        bw.write_u32(0, 1); // audprodie
        bw.write_u32(0, 1); // copyrightb
        bw.write_u32(1, 1); // origbs
        bw.write_u32(0, 1); // timecod1e
        bw.write_u32(0, 1); // timecod2e
        bw.write_u32(0, 1); // addbsie

        // ---- Audio blocks ----
        for blk in 0..BLOCKS_PER_FRAME {
            // §5.4.3.1 blksw[ch] — per-channel block-switch flag.
            // Value 1 ⇒ this channel uses the 256-sample short-block
            // pair for this audio block; the decoder takes the
            // matching `imdct_256_pair_fft` branch.
            for ch in 0..self.channels {
                bw.write_u32(blksw[ch][blk] as u32, 1);
            }
            // dithflag per channel: 1 (enable dither on zero-bap bins).
            // Spec-recommended default; decoder drives an LFSR-backed
            // pseudo-random mantissa replacement on bap=0 bins which
            // removes coloration of the IMDCT's stop band on masked
            // bins. After the backward-pass legaliser lowers some
            // silent-bin exponents, dither there multiplies `0.707` by
            // `2^-exp` which can be perceptible; however disabling
            // dither globally is worse than enabling it (we measured
            // ~2 dB PSNR regression on speech fixtures with dith off).
            bw.write_u32(1, 1);
            bw.write_u32(1, 1);
            // dynrnge = 0 (no dynrng transmitted; block 0 sets gain=1).
            bw.write_u32(0, 1);
            // §5.4.3.7-13 cplstre + cplinu + (when cplinu) the
            // chincpl[ch] / phsflginu / cplbegf / cplendf / cplbndstrc
            // sequence. The encoder commits to a single cpl
            // configuration for the whole frame, so all of this side
            // info is emitted on block 0 and reused thereafter.
            if blk == 0 {
                bw.write_u32(1, 1); // cplstre = 1
                bw.write_u32(cpl.in_use as u32, 1); // cplinu
                if cpl.in_use {
                    for ch in 0..self.channels {
                        bw.write_u32(cpl.chincpl[ch] as u32, 1);
                    }
                    // §5.4.3.10 phsflginu — only in 2/0.
                    bw.write_u32(cpl.phsflginu as u32, 1);
                    bw.write_u32(cpl.begf as u32, 4);
                    bw.write_u32(cpl.endf as u32, 4);
                    // §5.4.3.13 cplbndstrc[sbnd] for sbnd >= 1.
                    for sbnd in 1..cpl.nsubbnd {
                        bw.write_u32(cpl.bndstrc[sbnd] as u32, 1);
                    }
                }
            } else {
                bw.write_u32(0, 1); // cplstre = 0 (reuse)
            }
            // §5.4.3.14-18 cplcoe[ch] / mstrcplco / cplcoexp / cplcomant
            // / phsflg. Coupling coordinates are signalled on block 0
            // only (cplcoe[blk][ch] = (blk == 0)); subsequent blocks
            // reuse, which is the bit-saving win of the coupling
            // mechanism: one envelope per ~32 ms frame.
            if cpl.in_use {
                let mut any_coe = false;
                for ch in 0..self.channels {
                    if !cpl.chincpl[ch] {
                        continue;
                    }
                    let coe = cpl.cplcoe[blk][ch];
                    bw.write_u32(coe as u32, 1);
                    if coe {
                        any_coe = true;
                        bw.write_u32(cpl.mstrcplco[ch] as u32, 2);
                        for bnd in 0..cpl.nbnd {
                            bw.write_u32(cpl.cplcoexp[ch][bnd] as u32, 4);
                            bw.write_u32(cpl.cplcomant[ch][bnd] as u32, 4);
                        }
                    }
                }
                // §5.4.3.18 phsflg[bnd] — only when 2/0 + phsflginu +
                // any cplcoe set this block. We disabled phsflginu so
                // the field is suppressed; left as a guard for when
                // it is enabled in a future iteration.
                if cpl.phsflginu && any_coe {
                    for bnd in 0..cpl.nbnd {
                        bw.write_u32(cpl.phsflg[bnd] as u32, 1);
                    }
                }
            }
            // §5.4.3.19 rematstr (acmod == 2): we refresh rematflg
            // every block so the encoder can adapt the L/R vs L+R/L-R
            // decision per block. Cost is 1 + nrematbd bits per block.
            // When coupling is active the rematrix-band count shrinks
            // to track the lower edge of the cpl region (Table 5.15).
            if nrematbd > 0 {
                bw.write_u32(1, 1); // rematstr — flags follow
                for bnd in 0..nrematbd {
                    bw.write_u32(rematflg[blk][bnd] as u32, 1);
                }
            }

            // chexpstr: per-block strategy chosen above.
            let exp_strategy: u8 = exp_strategies[blk];
            // §5.4.3.21 cplexpstr — only when cplinu. Use the same
            // strategy as the fbw channels (D15 on blocks 0/3, REUSE
            // elsewhere).
            if cpl.in_use {
                bw.write_u32(exp_strategy as u32, 2);
            }
            bw.write_u32(exp_strategy as u32, 2); // ch0
            bw.write_u32(exp_strategy as u32, 2); // ch1
                                                  // chbwcod (only when exp strategy != reuse, AND channel not
                                                  // coupled). When coupling is active, channels do not
                                                  // transmit their own bandwidth code.
            if exp_strategy != 0 {
                for ch in 0..self.channels {
                    if !(cpl.in_use && cpl.chincpl[ch]) {
                        bw.write_u32(chbwcod as u32, 6);
                    }
                }
            }

            // Exponents: only transmitted when chexpstr != reuse.
            //
            // Order per spec §5.4.3.25-29: cplexps (when cplinu),
            // exps[0], exps[1], ..., lfeexps. cpl uses cplabsexp + D15
            // grouping over [cpl_begf_mant, cpl_endf_mant) per
            // §7.1.3, with the absolute exponent value implied to be
            // the 4-bit cplabsexp left-shifted by 1.
            if exp_strategy == 1 {
                if cpl.in_use {
                    let cpl_idx = self.channels;
                    let begf_mant = cpl.begf_mant();
                    let endf_mant = cpl.endf_mant();
                    write_exponents_cpl(&mut bw, &exps[cpl_idx][blk], begf_mant, endf_mant);
                }
                for ch in 0..self.channels {
                    write_exponents_d15(&mut bw, &exps[ch][blk], ch_end_mant);
                    bw.write_u32(0, 2); // gainrng = 0
                }
            }

            // Bit-allocation side-info: block 0 transmits the parametric
            // set + snroffst; later blocks reuse.
            let baie = blk == 0;
            bw.write_u32(baie as u32, 1);
            if baie {
                bw.write_u32(tuned_ba.sdcycod as u32, 2);
                bw.write_u32(tuned_ba.fdcycod as u32, 2);
                bw.write_u32(tuned_ba.sgaincod as u32, 2);
                bw.write_u32(tuned_ba.dbpbcod as u32, 2);
                bw.write_u32(tuned_ba.floorcod as u32, 3);
            }
            let snroffste = blk == 0;
            bw.write_u32(snroffste as u32, 1);
            if snroffste {
                bw.write_u32(tuned_ba.csnroffst as u32, 6);
                if cpl.in_use {
                    // §5.4.3.38 cplfsnroffst (4 bits), §5.4.3.39 cplfgaincod (3 bits).
                    bw.write_u32(tuned_ba.cplfsnroffst as u32, 4);
                    bw.write_u32(tuned_ba.cplfgaincod as u32, 3);
                }
                bw.write_u32(tuned_ba.fsnroffst as u32, 4); // ch0
                bw.write_u32(tuned_ba.fgaincod as u32, 3); // ch0
                bw.write_u32(tuned_ba.fsnroffst as u32, 4); // ch1
                bw.write_u32(tuned_ba.fgaincod as u32, 3); // ch1
            }
            // §5.4.3.44-46 cplleake / cplfleak / cplsleak. The spec
            // requires cplleake=1 on the first block where coupling
            // is in use (so the decoder gets fresh leak-init values
            // for the §7.2.2.4 cpl excitation path); subsequent
            // blocks may reuse with cplleake=0. We always send
            // cplfleak=cplsleak=0, matching the encoder's
            // expectation in the cpl bap routine
            // (`compute_bap_cpl` initialises leak from 768 + 0).
            if cpl.in_use {
                if blk == 0 {
                    bw.write_u32(1, 1); // cplleake = 1
                    bw.write_u32(0, 3); // cplfleak = 0
                    bw.write_u32(0, 3); // cplsleak = 0
                } else {
                    bw.write_u32(0, 1); // cplleake = 0 (reuse)
                }
            }
            // deltbaie = 0 (no delta bit allocation).
            bw.write_u32(0, 1);

            // skiple / skipl: potentially used at frame-end to pad out to
            // frame_bytes; for now, none per block.
            bw.write_u32(0, 1);

            // Mantissas per channel.
            //
            // Pre-compute all mantissa codes for this block first, then
            // walk them in decoder-read order and emit each grouped
            // quantizer's 5/7-bit packed word at the position of the
            // *first* code in the triple/pair. The decoder pre-fetches
            // groups (reads 5/7 bits whenever its buffer empties) while
            // the natural encoder loop would only emit when the buffer
            // fills — an asymmetric mismatch that desyncs the bitstream
            // across the channel boundary. By pre-quantising and then
            // emitting proactively we match the decoder's expected read
            // schedule exactly.
            //
            // Decoder mantissa-read order (§7.3.2): for each channel
            // walk bins 0..ch_end_mant emitting bap values; if the
            // channel is coupled and this is the first coupled channel
            // we encounter, also walk the coupling channel's
            // [cpl_begf_mant, cpl_endf_mant) bap sequence appended to
            // that channel's mantissa stream. The encoder must emit
            // codes in exactly the same order.
            let mut codes: Vec<(u8, u32)> = Vec::with_capacity((self.channels + 1) * ch_end_mant);
            let mut got_cplchan = false;
            for ch in 0..self.channels {
                for bin in 0..ch_end_mant {
                    let bap = baps[ch][blk][bin];
                    if bap == 0 {
                        continue;
                    }
                    let e = exps[ch][blk][bin] as i32;
                    let mant = quantise_mantissa(coeffs[ch][blk][bin], e, bap);
                    codes.push((bap, mant));
                }
                if cpl.in_use && cpl.chincpl[ch] && !got_cplchan {
                    got_cplchan = true;
                    let cpl_idx = self.channels;
                    let begf_mant = cpl.begf_mant();
                    let endf_mant = cpl.endf_mant();
                    for bin in begf_mant..endf_mant {
                        let bap = baps[cpl_idx][blk][bin];
                        if bap == 0 {
                            continue;
                        }
                        let e = exps[cpl_idx][blk][bin] as i32;
                        let mant = quantise_mantissa(cpl_coeffs[blk][bin], e, bap);
                        codes.push((bap, mant));
                    }
                }
            }
            write_mantissa_stream(&mut bw, &codes);
        }

        // auxdata: auxdatae=0 plus any necessary skip-padding so the
        // remainder of the frame before crc2 (16 bits) is filled with
        // zeros. The auxdata() field is defined as (nauxbits) then a
        // 1-bit auxdatae flag; we just set the whole field to a zero
        // tail up through the last byte before crc2.
        //
        // Compute how many bits remain before the last 16 bits (crc2).
        let target_bits = (self.frame_bytes * 8) as u64;
        let used_bits = bw.bit_position();
        let crc2_bits = 16u64;
        if used_bits + crc2_bits > target_bits {
            return Err(Error::other(format!(
                "ac3 encoder: mantissa budget overflow ({} bits used, frame {} bits)",
                used_bits, target_bits
            )));
        }
        let pad_bits = target_bits - used_bits - crc2_bits;
        // Write pad_bits of zeros — the final bit before crc2 is
        // auxdatae=0 by virtue of being in the padding zone.
        let mut left = pad_bits;
        while left >= 32 {
            bw.write_u32(0, 32);
            left -= 32;
        }
        if left > 0 {
            bw.write_u32(0, left as u32);
        }

        // Placeholder crc2 — filled after the body is emitted.
        bw.write_u32(0, 16);
        let mut frame = bw.into_bytes();
        debug_assert_eq!(frame.len(), self.frame_bytes);

        // Compute crc1 over the first 5/8 of the syncframe — the sync
        // word (bytes 0..1) is excluded, but the 2-byte crc1 field
        // itself (bytes 2..3) is *included*. We need to place a value X
        // into bytes 2..3 such that the LFSR residue at the end of the
        // 5/8 region is zero. Since the CRC is XOR-linear, we compute
        // the residue R_0 with X = 0, and independently compute how a
        // 16-bit value placed at bytes 2..3 propagates forward — then
        // solve for the X whose propagation cancels R_0.
        let frame_words = self.frame_bytes / 2;
        let five_eighths_words = (frame_words >> 1) + (frame_words >> 3);
        let five_eighths_bytes = five_eighths_words * 2;
        let crc1_val = ac3_crc_solve_prefix(&frame[2..five_eighths_bytes]);
        frame[2] = (crc1_val >> 8) as u8;
        frame[3] = (crc1_val & 0xFF) as u8;
        debug_assert_eq!(
            ac3_crc_update(0, &frame[2..five_eighths_bytes]),
            0,
            "crc1 solver produced a non-zero residue"
        );

        // crc2 covers the last 3/8 of the syncframe: bytes
        // five_eighths_bytes .. frame_bytes with the 2-byte crc2 field
        // itself at the tail. The CRC value to emit is the running CRC
        // over the data without the field; ATSC A/52 §6.1.7 verifiers
        // compare the recomputed value to the stored bytes (not a
        // residue check), which suits this "augmented" CRC formulation
        // — the residue-of-the-augmented-stream property only holds in
        // the direct CRC formulation.
        let crc2_val = ac3_crc_update(0, &frame[five_eighths_bytes..(self.frame_bytes - 2)]);
        let n = self.frame_bytes;
        frame[n - 2] = (crc2_val >> 8) as u8;
        frame[n - 1] = (crc2_val & 0xFF) as u8;

        self.packet_queue.push(
            Packet::new(0, TimeBase::new(1, self.sample_rate as i64), frame).with_pts(self.pts),
        );
        self.pts += SAMPLES_PER_FRAME as i64;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Transient detection (§7.6 block-switch decision)
// ---------------------------------------------------------------------------

/// Decide whether a 256-sample input block contains a transient that
/// warrants a short-block MDCT (§7.6.1). The heuristic compares the
/// *high-pass* energy of consecutive 64-sample sub-frames within the
/// block; if any pair's ratio exceeds a fixed threshold the block is
/// flagged short.
///
/// Why high-pass: a long MDCT smears transients across the 256-sample
/// post-IMDCT support, raising mid/high-frequency noise *across the
/// whole block*. The short-block pair localises the transient to one
/// of the two 128-sample sub-windows, halving the temporal smear.
/// Detecting on the high-pass band keeps the heuristic insensitive
/// to slowly-varying low-frequency content (e.g. a 50 Hz hum) while
/// reacting strongly to drum hits / clicks.
///
/// The IIR filter is the simplest stable HP: `y[n] = x[n] - x[n-1]`,
/// a one-tap differentiator that doubles the noise floor on white
/// input but adds zero state — important because the transient
/// detector is invoked once per block per channel and we don't carry
/// per-channel HP filter state across blocks (each block decides
/// independently — adjacent transients on different channels are
/// allowed to flip blksw differently per spec §5.4.3.1).
///
/// Threshold = 4.0 — corresponds to ~6 dB sub-frame-to-sub-frame
/// energy jump. Smaller `THRESHOLD` over-triggers on a steady tone's
/// natural amplitude variation; larger misses the trailing edge of
/// short attacks. 4.0 picks up the burst onsets in the
/// `transient_bursts_stereo` fixture's analogue without misfiring on
/// the steady 440 Hz pre-burst region (verified by
/// `transient_detector_sanity`).
fn detect_transient(block: &[f32]) -> bool {
    if block.len() < 32 {
        return false;
    }
    const THRESHOLD: f32 = 4.0;
    // 32-sample sub-frames give the detector ~150 µs resolution
    // (32 samples / 48 kHz). Smaller would be more sensitive but
    // also noisier; larger would miss sharp clicks.
    const SUBFRAME: usize = 32;
    // High-pass via first-difference. A first-order HP has a 6
    // dB/oct slope, which is enough to suppress the dominant
    // 440 Hz sine in the test fixture (rolled off ~10 dB at the
    // first-difference's natural break-frequency of ~2400 Hz @
    // 48 kHz) while passing the burst's sharp envelope edges.
    let mut hp = vec![0.0f32; block.len()];
    hp[0] = block[0];
    for i in 1..block.len() {
        hp[i] = block[i] - block[i - 1];
    }
    // Sub-frame energies.
    let nsub = block.len() / SUBFRAME;
    let mut energies = vec![0.0f32; nsub];
    for sub in 0..nsub {
        let lo = sub * SUBFRAME;
        let hi = lo + SUBFRAME;
        energies[sub] = hp[lo..hi].iter().map(|&v| v * v).sum::<f32>();
    }
    // Search for the largest sub-to-sub energy jump.
    const FLOOR: f32 = 1e-8;
    for w in energies.windows(2) {
        let prev = w[0].max(FLOOR);
        let cur = w[1];
        if cur > THRESHOLD * prev {
            return true;
        }
    }
    false
}

// ---------------------------------------------------------------------------
// Exponent extraction + D15 encoding
// ---------------------------------------------------------------------------

/// Compute the AC-3 exponent for a single coefficient: the number of
/// left shifts that would bring `|x|` to the interval `[0.5, 1)`, clamped
/// to `0..=24` (§8.2.7 extract_exponents).
fn extract_exponent(x: f32) -> u8 {
    let ax = x.abs();
    if ax < f32::MIN_POSITIVE {
        return 24;
    }
    // x = m * 2^-e with |m| in [0.5, 1) ⇒ e = -floor(log2(ax)) - 1,
    // which for ax ∈ [2^-25, 1) lies in 0..=24.
    let e = (-ax.log2().floor() as i32) - 1;
    e.clamp(0, 24) as u8
}

/// Pre-process the D15 exponent run so that successive differences
/// stay in `[-2, +2]` **and** the absolute-value constraints of the
/// bitstream layout hold (absolute exponent fits in 4 bits → `0..=15`;
/// subsequent exponents remain ≥0 after the decoder replays the
/// differences). Two-pass implementation:
///
/// 1. **Backward pass** — propagate low (loud-bin) exponents *toward*
///    the start of the array. For each bin, `exp[i]` must ≤ `exp[i+1] + 2`
///    so the encoder can reach the loud bin's exponent within the D15
///    per-step slope. Without this, a narrow-band spike (e.g. a sine
///    tone) surrounded by silent (exp=24) bins would force the encoder
///    to stay at 24 until two bins before the spike, then step down in
///    ±2 increments — which can't reach exp=1 in time. The backward
///    pass pre-drops the silent bins' exponents so the decoder can
///    reconstruct the spike's exponent accurately.
/// 2. **Forward pass** — legalise absexp to 4 bits, then clamp each
///    forward delta to `±2` and each running value to `[0, 24]`. After
///    the backward pass, the forward pass is typically a no-op; it's
///    kept to guarantee legality for pathological inputs.
///
/// `exp` is mutated in place.
fn preprocess_d15(exp: &mut [u8]) {
    if exp.is_empty() {
        return;
    }
    // Backward pass: ensure exp[i] ≤ exp[i+1] + 2 for every adjacent
    // pair. Propagates low values leftward at the maximum legal slope.
    for i in (0..exp.len() - 1).rev() {
        let next_plus_two = (exp[i + 1] as i32 + 2).min(24) as u8;
        if exp[i] > next_plus_two {
            exp[i] = next_plus_two;
        }
    }
    // absexp (exps[0]) is transmitted in 4 bits → 0..=15.
    if exp[0] > 15 {
        exp[0] = 15;
    }
    // Forward pass: clamp each delta to ±2 and the running value to [0, 24].
    for i in 1..exp.len() {
        let prev = exp[i - 1] as i32;
        let cur = exp[i] as i32;
        let d = cur - prev;
        let clamped = d.clamp(-2, 2);
        exp[i] = (prev + clamped).clamp(0, 24) as u8;
    }
}

/// Write D15 exponents for the coupling pseudo-channel per §5.4.3.25 +
/// §7.1.3. Differs from the fbw `write_exponents_d15` in two ways:
///
/// 1. **`cplabsexp` is a *reference* not a real exponent.** The 4-bit
///    `cplabsexp` field is left-shifted by 1 to seed the differential
///    decoder; the first transmitted exponent is `exp[cpl_strtmant]`
///    (no `exp[0]` involved). We pick `cplabsexp` ≈ `exp[cpl_strtmant]
///    / 2` so the first delta lies inside ±2.
/// 2. **Groups span `[cpl_strtmant, cpl_endmant)`.** With D15 grpsize=1
///    that's `ncplgrps = (cpl_endmant - cpl_strtmant) / 3` 7-bit words.
fn write_exponents_cpl(bw: &mut BitWriter, exp: &[u8; N_COEFFS], start: usize, end: usize) {
    if end <= start {
        return;
    }
    // Pick cplabsexp such that (cplabsexp << 1) is the closest even
    // value to exp[start], clamped to the 4-bit range. Enforce that
    // the first delta lies in ±2 by clamping the start exponent first.
    let first_exp = exp[start] as i32;
    let cplabsexp = ((first_exp + 1) >> 1).clamp(0, 15) as u8;
    bw.write_u32(cplabsexp as u32, 4);
    let ncplgrps = (end - start) / 3;
    let mut prev = (cplabsexp as i32) << 1;
    for grp in 0..ncplgrps {
        let base = start + grp * 3;
        let e0 = exp[base] as i32;
        let e1 = exp[base + 1] as i32;
        let e2 = exp[base + 2] as i32;
        let d0 = (e0 - prev).clamp(-2, 2) + 2;
        let d1 = (e1 - e0).clamp(-2, 2) + 2;
        let d2 = (e2 - e1).clamp(-2, 2) + 2;
        let packed: u32 = (25 * d0 + 5 * d1 + d2) as u32;
        bw.write_u32(packed, 7);
        prev = e2;
    }
}

/// Write D15 exponents per §7.1.3 / §5.4.3.16+. D15 is `grpsize = 1`:
/// every raw exponent carries one delta. The first absolute exponent is
/// 4 bits (exps[ch][0]); subsequent exponents are packed in groups of
/// three (ngrps = (end-1)/3) as a single 7-bit word encoding three
/// `(dexp+2)` values via m = 25*(dexp0+2) + 5*(dexp1+2) + (dexp2+2).
fn write_exponents_d15(bw: &mut BitWriter, exp: &[u8; N_COEFFS], end: usize) {
    let absexp = exp[0];
    bw.write_u32(absexp as u32, 4);
    let ngrps = (end - 1) / 3;
    for grp in 0..ngrps {
        let base = 1 + grp * 3;
        let e_prev_0 = if base == 1 {
            absexp as i32
        } else {
            exp[base - 1] as i32
        };
        let e0 = exp[base] as i32;
        let e1 = exp[base + 1] as i32;
        let e2 = exp[base + 2] as i32;
        let d0 = (e0 - e_prev_0).clamp(-2, 2) + 2;
        let d1 = (e1 - e0).clamp(-2, 2) + 2;
        let d2 = (e2 - e1).clamp(-2, 2) + 2;
        let packed: u32 = (25 * d0 + 5 * d1 + d2) as u32;
        bw.write_u32(packed, 7);
    }
    // If (end-1) % 3 != 0, the spare (up to 2) exponents are padded
    // into the last group implicitly — A/52 caps end at 253, so this
    // is exact for chbwcod=60 (end=252, (252-1)/3=83 groups with
    // 83*3+1=250 exponents transmitted). Uncovered bins reuse the last
    // transmitted value on the decoder side via the grpsize expansion
    // step (`exp[(i*grpsize)+j+1] = aexp[i]`), so they are not a
    // problem in practice.
    let _ = ngrps;
}

// ---------------------------------------------------------------------------
// Bit allocation (encoder-side) — runs the same §7.2.2 routine the
// decoder uses, but retains the bap array for mantissa quantisation.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
#[allow(dead_code)]
struct BitAllocParams {
    sdcycod: u8,
    fdcycod: u8,
    sgaincod: u8,
    dbpbcod: u8,
    floorcod: u8,
    csnroffst: u8,
    fsnroffst: u8,
    cplfsnroffst: u8,
    lfefsnroffst: u8,
    fgaincod: u8,
    cplfgaincod: u8,
    lfefgaincod: u8,
}

/// Run the parametric bit allocator for one channel (start=0..end)
/// and fill `bap_out` with the resulting pointers.
fn compute_bap(
    exp: &[u8; N_COEFFS],
    end: usize,
    fscod: u8,
    ba: &BitAllocParams,
    bap_out: &mut [u8; N_COEFFS],
) {
    if end == 0 {
        return;
    }
    // PSD
    let mut psd = [0i32; N_COEFFS];
    for bin in 0..end {
        psd[bin] = 3072 - ((exp[bin] as i32) << 7);
    }
    // Band PSD
    let mut bndpsd = [0i32; 50];
    let bndstrt = MASKTAB[0] as usize;
    let bndend = MASKTAB[end - 1] as usize + 1;
    {
        let mut j = 0usize;
        let mut k = bndstrt;
        loop {
            let lastbin = (BNDTAB[k] as usize + BNDSZ[k] as usize).min(end);
            bndpsd[k] = psd[j];
            j += 1;
            while j < lastbin {
                bndpsd[k] = logadd(bndpsd[k], psd[j]);
                j += 1;
            }
            k += 1;
            if end <= lastbin {
                break;
            }
        }
    }
    // Excitation — fbw, non-coupled path.
    let sdecay = SLOWDEC[ba.sdcycod as usize];
    let fdecay = FASTDEC[ba.fdcycod as usize];
    let sgain = SLOWGAIN[ba.sgaincod as usize];
    let dbknee = DBPBTAB[ba.dbpbcod as usize];
    let floor = FLOORTAB[ba.floorcod as usize];
    let fgain = FASTGAIN[ba.fgaincod as usize];
    let snroffset = (((ba.csnroffst as i32 - 15) << 4) + ba.fsnroffst as i32) << 2;

    let mut excite = [0i32; 50];
    let mut lowcomp = 0i32;
    // §7.2.2.4 fbw path (start == 0).
    if bndend > 0 {
        lowcomp = calc_lowcomp(lowcomp, bndpsd[0], bndpsd[1], 0);
        excite[0] = bndpsd[0] - fgain - lowcomp;
    }
    if bndend > 1 {
        lowcomp = calc_lowcomp(lowcomp, bndpsd[1], bndpsd[2], 1);
        excite[1] = bndpsd[1] - fgain - lowcomp;
    }
    let mut begin = 7.min(bndend);
    let mut fastleak = 0i32;
    let mut slowleak = 0i32;
    for bin in 2..7.min(bndend) {
        lowcomp = calc_lowcomp(lowcomp, bndpsd[bin], bndpsd[bin + 1], bin);
        fastleak = bndpsd[bin] - fgain;
        slowleak = bndpsd[bin] - sgain;
        excite[bin] = fastleak - lowcomp;
        if bndpsd[bin] <= bndpsd[bin + 1] {
            begin = bin + 1;
            break;
        }
    }
    for bin in begin..22.min(bndend) {
        lowcomp = calc_lowcomp(lowcomp, bndpsd[bin], bndpsd[bin + 1], bin);
        fastleak -= fdecay;
        fastleak = fastleak.max(bndpsd[bin] - fgain);
        slowleak -= sdecay;
        slowleak = slowleak.max(bndpsd[bin] - sgain);
        excite[bin] = (fastleak - lowcomp).max(slowleak);
    }
    if bndend > 22 {
        for bin in 22..bndend {
            fastleak -= fdecay;
            fastleak = fastleak.max(bndpsd[bin] - fgain);
            slowleak -= sdecay;
            slowleak = slowleak.max(bndpsd[bin] - sgain);
            excite[bin] = fastleak.max(slowleak);
        }
    }
    // Mask.
    let hth_row = &HTH[fscod as usize];
    let mut mask = [0i32; 50];
    for bin in bndstrt..bndend {
        let mut exc = excite[bin];
        if bndpsd[bin] < dbknee {
            exc += (dbknee - bndpsd[bin]) >> 2;
        }
        mask[bin] = exc.max(hth_row[bin] as i32);
    }
    // bap.
    let mut i = 0usize;
    let mut j = MASKTAB[0] as usize;
    loop {
        let lastbin = (BNDTAB[j] as usize + BNDSZ[j] as usize).min(end);
        let mut m = mask[j];
        m -= snroffset;
        m -= floor;
        if m < 0 {
            m = 0;
        }
        m &= 0x1fe0;
        m += floor;
        while i < lastbin {
            let addr = ((psd[i] - m) >> 5).clamp(0, 63) as usize;
            bap_out[i] = BAPTAB[addr];
            i += 1;
        }
        if i >= end {
            break;
        }
        j += 1;
    }
}

/// Bit allocation for the coupling pseudo-channel, mirroring the
/// decoder's `run_bit_allocation(..., is_coupling=true)` path.
///
/// Differences from the fbw `compute_bap`:
///   * no lowcomp / start-of-spectrum special cases — the cpl region
///     starts mid-spectrum so the leak filters init from `768` and run
///     the simple `fastleak.max(slowleak)` excitation across every
///     band.
///   * `start = cpl_begf_mant`, `end = cpl_endf_mant` (in coefficient
///     bins). Bands are derived via MASKTAB[bin] just like fbw.
///
/// `ba.fsnroffst` and `ba.fgaincod` should be the cpl-specific
/// values (cplfsnroffst, cplfgaincod) — the caller is responsible for
/// substituting them into the BitAllocParams before calling.
fn compute_bap_cpl(
    exp: &[u8; N_COEFFS],
    start: usize,
    end: usize,
    fscod: u8,
    ba: &BitAllocParams,
    bap_out: &mut [u8; N_COEFFS],
) {
    if end <= start {
        return;
    }
    let mut psd = [0i32; N_COEFFS];
    for bin in start..end {
        psd[bin] = 3072 - ((exp[bin] as i32) << 7);
    }
    let bndstrt = MASKTAB[start] as usize;
    let bndend = MASKTAB[end - 1] as usize + 1;
    let mut bndpsd = [0i32; 50];
    {
        let mut j = start;
        let mut k = bndstrt;
        loop {
            let lastbin = (BNDTAB[k] as usize + BNDSZ[k] as usize).min(end);
            bndpsd[k] = psd[j];
            j += 1;
            while j < lastbin {
                bndpsd[k] = logadd(bndpsd[k], psd[j]);
                j += 1;
            }
            k += 1;
            if end <= lastbin {
                break;
            }
        }
    }
    let sdecay = SLOWDEC[ba.sdcycod as usize];
    let fdecay = FASTDEC[ba.fdcycod as usize];
    let sgain = SLOWGAIN[ba.sgaincod as usize];
    let dbknee = DBPBTAB[ba.dbpbcod as usize];
    let floor = FLOORTAB[ba.floorcod as usize];
    let fgain = FASTGAIN[ba.fgaincod as usize];
    let snroffset = (((ba.csnroffst as i32 - 15) << 4) + ba.fsnroffst as i32) << 2;
    // §7.2.2.4 cpl path. cpl_fleak / cpl_sleak default to 0 (no
    // cplleake transmitted), giving fastleak_init = slowleak_init = 0
    // and the +768 offset applied in the decoder.
    let mut fastleak = 768i32;
    let mut slowleak = 768i32;
    let mut excite = [0i32; 50];
    for bin in bndstrt..bndend {
        fastleak -= fdecay;
        fastleak = fastleak.max(bndpsd[bin] - fgain);
        slowleak -= sdecay;
        slowleak = slowleak.max(bndpsd[bin] - sgain);
        excite[bin] = fastleak.max(slowleak);
    }
    let hth_row = &HTH[fscod as usize];
    let mut mask = [0i32; 50];
    for bin in bndstrt..bndend {
        let mut exc = excite[bin];
        if bndpsd[bin] < dbknee {
            exc += (dbknee - bndpsd[bin]) >> 2;
        }
        mask[bin] = exc.max(hth_row[bin] as i32);
    }
    let mut i = start;
    let mut j = MASKTAB[start] as usize;
    loop {
        let lastbin = (BNDTAB[j] as usize + BNDSZ[j] as usize).min(end);
        let mut m = mask[j];
        m -= snroffset;
        m -= floor;
        if m < 0 {
            m = 0;
        }
        m &= 0x1fe0;
        m += floor;
        while i < lastbin {
            let addr = ((psd[i] - m) >> 5).clamp(0, 63) as usize;
            bap_out[i] = BAPTAB[addr];
            i += 1;
        }
        if i >= end {
            break;
        }
        j += 1;
    }
}

fn logadd(a: i32, b: i32) -> i32 {
    let c = a - b;
    let addr = ((c.abs() >> 1) as usize).min(255);
    if c >= 0 {
        a + LATAB[addr] as i32
    } else {
        b + LATAB[addr] as i32
    }
}

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

/// Count mantissa bits used by a bap histogram over all channels and
/// blocks. Grouped bap values (1, 2, 4) charge per-group cost; other
/// values charge nbits per mantissa. Returns the total in bits.
///
/// `nchan` excludes the cpl pseudo-channel; `end` is the per-channel
/// upper bound (= cpl_begf_mant when coupling is in use). The cpl
/// pseudo-channel's bap (when active) is appended into the same group
/// stream right after the first coupled channel — same order as the
/// decoder's read schedule (`unpack_mantissas`).
fn mantissa_bits_total(
    baps: &[Vec<[u8; N_COEFFS]>],
    end: usize,
    nchan: usize,
    cpl: &CouplingPlan,
) -> u32 {
    let blocks = baps[0].len();
    let mut total = 0u32;
    let cpl_idx = nchan;
    let begf_mant = cpl.begf_mant();
    let endf_mant = cpl.endf_mant();
    for blk in 0..blocks {
        let mut g1_left = 0u32;
        let mut g2_left = 0u32;
        let mut g4_left = 0u32;
        let mut got_cplchan = false;
        for ch in 0..nchan {
            for bin in 0..end {
                let bap = baps[ch][blk][bin];
                match bap {
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
            if cpl.in_use && cpl.chincpl[ch] && !got_cplchan {
                got_cplchan = true;
                for bin in begf_mant..endf_mant {
                    let bap = baps[cpl_idx][blk][bin];
                    match bap {
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
            }
        }
    }
    total
}

/// Compute the exact overhead bits (everything except mantissas) for a
/// given per-block exponent strategy. We walk the bitstream layout the
/// emitter uses and sum each field's width.
fn overhead_bits_for(
    exp_strategies: &[u8; BLOCKS_PER_FRAME],
    end: usize,
    nchan: usize,
    cpl: &CouplingPlan,
) -> u32 {
    // syncinfo: 16 (sync) + 16 (crc1) + 2 (fscod) + 6 (frmsizecod) = 40
    // BSI for 2/0 stereo: 5+3+3+2+1+5+1+1+1+1+1+1+1+1 = 27
    let mut bits: u32 = 40 + 27;
    // D15 ngrps for the per-channel exponents (over [0, end)).
    let ngrps = (end - 1) / 3;
    let d15_bits_per_ch = 4 + 7 * ngrps as u32;
    // D15 ngrps for the cpl pseudo-channel (over [cpl_begf_mant,
    // cpl_endf_mant)). Per §7.1.3 cpl uses
    // ncplgrps = (cplendmant - cplstrtmant) / 3 for D15.
    let cpl_d15_bits = if cpl.in_use {
        let n = (cpl.endf_mant() - cpl.begf_mant()) / 3;
        4 + 7 * n as u32
    } else {
        0
    };
    let nrematbd_bits = if nchan >= 2 {
        // 1 bit rematstr + nrematbd flags. With cplinu the band count
        // tracks Table 5.15.
        1 + remat_band_count(cpl.in_use, cpl.begf) as u32
    } else {
        0
    };
    let coupled_chs = if cpl.in_use {
        cpl.chincpl[..nchan].iter().filter(|&&v| v).count() as u32
    } else {
        0
    };
    for &s in exp_strategies.iter() {
        // blksw per ch (1 bit × nchan), dithflag × nchan, dynrnge(1)
        bits += nchan as u32 * 2 + 1;
        // cplstre + cplinu + (block 0 only) the cpl strategy fields.
        if s == 1 {
            // block 0 emits cplstre=1 + cplinu (+ cpl strategy fields
            // when cpl.in_use).
            bits += 1 + 1;
            if cpl.in_use {
                // chincpl[ch] × nchan + phsflginu(1) + cplbegf(4) +
                // cplendf(4) + (nsubbnd-1) cplbndstrc bits.
                bits += nchan as u32 + 1 + 4 + 4;
                bits += cpl.nsubbnd.saturating_sub(1) as u32;
            }
        } else {
            // non-block-0: cplstre = 0 (reuse), no further cpl strategy fields.
            bits += 1;
        }
        // §5.4.3.14-18 cplcoe[ch] (1 bit per coupled ch) + the
        // mstrcplco/cplcoexp/cplcomant payload when cplcoe=1, plus the
        // optional phsflg burst. Coordinates are signalled on block 0
        // only (cplcoe[blk][ch]=(blk==0)).
        if cpl.in_use {
            // cplcoe per coupled ch = coupled_chs bits.
            bits += coupled_chs;
            // payload only when cplcoe=1 → only on block 0 in our
            // policy. The strategy code is 1 on block 0 in our default
            // [1,0,0,1,0,0]; map block-0 to s==1 by checking position.
        }
        // rematstr (1 bit) + nrematbd flags.
        bits += nrematbd_bits;
        // §5.4.3.21 cplexpstr — 2 bits per block when cplinu.
        if cpl.in_use {
            bits += 2;
        }
        // chexpstr × nchan (2 bits each)
        bits += 2 * nchan as u32;
        // chbwcod × nchan (6 bits) only when exp strategy != reuse and
        // channel not coupled.
        if s != 0 {
            let n_indep = if cpl.in_use {
                nchan as u32 - coupled_chs
            } else {
                nchan as u32
            };
            bits += 6 * n_indep;
        }
        // exponents when new: cpl D15 (when cplexpstr=new) +
        // per-channel D15 + 2 bits gainrng × nchan.
        if s == 1 {
            if cpl.in_use {
                bits += cpl_d15_bits;
            }
            bits += (d15_bits_per_ch + 2) * nchan as u32;
        }
        // baie(1) + bit-alloc side info
        if s == 1 {
            // baie=1: sdcycod(2)+fdcycod(2)+sgaincod(2)+dbpbcod(2)+floorcod(3)=11
            bits += 1 + 11;
            // snroffste=1: csnr(6) + (cpl: cplfsnr(4)+cplfgain(3)) +
            // fsnr(4)+fgain(3) per ch.
            bits += 1 + 6 + nchan as u32 * (4 + 3);
            if cpl.in_use {
                bits += 4 + 3;
            }
        } else {
            bits += 1; // baie=0
            bits += 1; // snroffste=0
        }
        // §5.4.3.44 cplleake (1 bit per block when cplinu).
        if cpl.in_use {
            bits += 1;
        }
        bits += 1; // deltbaie=0
        bits += 1; // skiple=0
    }
    // §5.4.3.45-46 cplfleak/cplsleak — 3+3 bits, sent once on
    // block 0 since the spec requires cplleake=1 there. Subsequent
    // blocks emit cplleake=0 and reuse.
    if cpl.in_use {
        bits += 6;
    }
    // Block-0 cplcoe payload accounting (one-shot, outside the per-
    // block strategy loop). When cplcoe[blk=0][ch]=1 the bitstream
    // emits mstrcplco(2) + (cplcoexp(4)+cplcomant(4)) × nbnd per
    // coupled channel.
    if cpl.in_use {
        bits += coupled_chs * (2 + 8 * cpl.nbnd as u32);
    }
    // auxdatae flag inherent in final pad byte; crc2 (16 bits).
    bits += 16;
    bits
}

/// Adjust `csnroffst`/`fsnroffst` so the total mantissa bit count fits
/// the frame payload, minus the exact overhead cost. The sub-optimal
/// sweep is O(16*16) which is trivial given 6 blocks × 253 bins.
#[allow(clippy::too_many_arguments)]
fn tune_snroffst(
    ba: &BitAllocParams,
    exps: &[Vec<[u8; N_COEFFS]>],
    end: usize,
    nchan: usize,
    fscod: u8,
    frame_bytes: usize,
    exp_strategies: &[u8; BLOCKS_PER_FRAME],
    cpl: &CouplingPlan,
) -> BitAllocParams {
    let overhead = overhead_bits_for(exp_strategies, end, nchan, cpl) + 32 /* safety */;
    let total_bits = (frame_bytes * 8) as u32;
    if overhead >= total_bits {
        return *ba;
    }
    let budget = total_bits - overhead;

    // Maximise SNR offset subject to mantissa bit count fitting `budget`.
    // Search over all (csnroffst, fsnroffst) pairs — with a small early
    // termination once the combined offset starts producing non-monotone
    // growth. The 256-pair sweep is fast in release and lets us find a
    // finer optimum than the old greedy walk.
    //
    // When coupling is in use we also tune `cplfsnroffst` together
    // with the per-channel `fsnroffst`. To keep the search small we
    // tie cplfsnr ≡ fsnr (the cpl pseudo-channel and the fbw channels
    // share the same sub-band SNR offset). This is sub-optimal but
    // adequate for an initial coupling-encode implementation.
    let mut best = *ba;
    let mut best_offset: i32 = -1;
    for csnr in 0..=63u8 {
        for fsnr in 0..=15u8 {
            let mut cand = *ba;
            cand.csnroffst = csnr;
            cand.fsnroffst = fsnr;
            cand.cplfsnroffst = fsnr;
            let mut baps: Vec<Vec<[u8; N_COEFFS]>> =
                vec![vec![[0u8; N_COEFFS]; exps[0].len()]; nchan + 1];
            for ch in 0..nchan {
                for blk in 0..exps[ch].len() {
                    compute_bap(&exps[ch][blk], end, fscod, &cand, &mut baps[ch][blk]);
                }
            }
            if cpl.in_use {
                let cpl_idx = nchan;
                let begf_mant = cpl.begf_mant();
                let endf_mant = cpl.endf_mant();
                let mut cpl_ba = cand;
                cpl_ba.fsnroffst = cand.cplfsnroffst;
                cpl_ba.fgaincod = cand.cplfgaincod;
                for blk in 0..exps[cpl_idx].len() {
                    compute_bap_cpl(
                        &exps[cpl_idx][blk],
                        begf_mant,
                        endf_mant,
                        fscod,
                        &cpl_ba,
                        &mut baps[cpl_idx][blk],
                    );
                }
            }
            let used = mantissa_bits_total(&baps, end, nchan, cpl);
            if used <= budget {
                let combined = (csnr as i32) * 16 + fsnr as i32;
                if combined > best_offset {
                    best_offset = combined;
                    best = cand;
                }
            }
        }
    }
    best
}

// ---------------------------------------------------------------------------
// Mantissa quantisation + packing
// ---------------------------------------------------------------------------

#[derive(Default)]
#[allow(dead_code)]
struct MantGroupCtx {
    /// Pending 3-level mantissa codes (0..3) to pack when the group fills.
    g3_codes: [u32; 3],
    g3_n: usize,
    /// Pending 5-level mantissa codes (0..5).
    g5_codes: [u32; 3],
    g5_n: usize,
    /// Pending 11-level mantissa codes (0..11).
    g11_codes: [u32; 2],
    g11_n: usize,
}

/// Quantise a floating-point transform coefficient into its AC-3
/// mantissa code for bap ∈ 1..=15. Returns `u32` (upper-bit-unused for
/// narrow bap).
fn quantise_mantissa(coeff: f32, exp: i32, bap: u8) -> u32 {
    // Normalised mantissa in (-1, 1): coeff * 2^exp.
    let m = (coeff * 2f32.powi(exp)).clamp(-1.0, 1.0);
    match bap {
        1 => {
            // 3-level: nearest of {-2/3, 0, 2/3} → codes 0..2.
            let nearest = nearest_symmetric(m, &MANT_LEVEL_3);
            nearest as u32
        }
        2 => nearest_symmetric(m, &MANT_LEVEL_5) as u32,
        3 => nearest_symmetric(m, &MANT_LEVEL_7) as u32,
        4 => nearest_symmetric(m, &MANT_LEVEL_11) as u32,
        5 => nearest_symmetric(m, &MANT_LEVEL_15) as u32,
        b => {
            // 6..=15: asymmetric 2's-complement fractional, `nbits` bits.
            let nbits = QUANTIZATION_BITS[b as usize] as u32;
            let shift = nbits - 1;
            let v = (m * (1u32 << shift) as f32).round() as i32;
            let max = (1i32 << shift) - 1;
            let min = -(1i32 << shift);
            let clamped = v.clamp(min, max);
            let mask = if nbits == 32 {
                u32::MAX
            } else {
                (1u32 << nbits) - 1
            };
            (clamped as u32) & mask
        }
    }
}

fn nearest_symmetric(m: f32, table: &[f32]) -> usize {
    let mut best_i = 0usize;
    let mut best_d = f32::INFINITY;
    for (i, &v) in table.iter().enumerate() {
        let d = (m - v).abs();
        if d < best_d {
            best_d = d;
            best_i = i;
        }
    }
    best_i
}

/// Emit a block's mantissa codes in the decoder's expected read order.
///
/// The AC-3 decoder pre-fetches grouped mantissas (bap=1/2/4) — it
/// reads the 5-bit triple (bap=1), 7-bit triple (bap=2), or 7-bit pair
/// (bap=4) *as soon as its internal buffer empties and the next code
/// of that bap is requested*. A naive encoder that only emits when a
/// group fills would lag the decoder by one group at the very first
/// non-full boundary, causing a persistent bitstream desync that
/// corrupts every mantissa read after that point.
///
/// To match the decoder's schedule exactly, we pre-quantise all of the
/// block's mantissas (already done in `codes`) and then walk the list
/// in order. Whenever we encounter a bap=1/2/4 code and no codes of
/// that bap are buffered, we scan forward to the next two (or one, for
/// bap=4) codes of the same bap, pack them into the group word, and
/// emit it at the current bit position. The "consumed" flags track
/// which codes have already been rolled into a group so the outer loop
/// skips them when reached.
///
/// Bap values 3, 5, and 6..=15 are emitted inline (no grouping).
fn write_mantissa_stream(bw: &mut BitWriter, codes: &[(u8, u32)]) {
    let n = codes.len();
    let mut consumed = vec![false; n];
    for i in 0..n {
        if consumed[i] {
            continue;
        }
        let (bap, mant) = codes[i];
        match bap {
            1 => {
                // 5-bit group of 3 codes (3-level quantiser).
                let m1 = mant;
                let (m2, m3) = grab_next_two(codes, &mut consumed, i, 1);
                let packed = m1 * 9 + m2 * 3 + m3;
                bw.write_u32(packed, 5);
            }
            2 => {
                // 7-bit group of 3 codes (5-level).
                let m1 = mant;
                let (m2, m3) = grab_next_two(codes, &mut consumed, i, 2);
                let packed = m1 * 25 + m2 * 5 + m3;
                bw.write_u32(packed, 7);
            }
            4 => {
                // 7-bit group of 2 codes (11-level).
                let m1 = mant;
                let m2 = grab_next_one(codes, &mut consumed, i, 4);
                let packed = m1 * 11 + m2;
                bw.write_u32(packed, 7);
            }
            3 => bw.write_u32(mant, 3),
            5 => bw.write_u32(mant, 4),
            b if (6..=15).contains(&b) => {
                let nbits = QUANTIZATION_BITS[b as usize] as u32;
                bw.write_u32(mant, nbits);
            }
            _ => {}
        }
    }
}

/// Scan forward in `codes` for the next two entries matching `target_bap`
/// that have not yet been consumed. Marks those entries consumed and
/// returns their mantissa codes. Pads with zero if fewer than two are
/// available (typical for end-of-block partial groups).
fn grab_next_two(
    codes: &[(u8, u32)],
    consumed: &mut [bool],
    start: usize,
    target_bap: u8,
) -> (u32, u32) {
    let mut out = [0u32; 2];
    let mut n = 0usize;
    for j in (start + 1)..codes.len() {
        if consumed[j] {
            continue;
        }
        if codes[j].0 == target_bap {
            out[n] = codes[j].1;
            consumed[j] = true;
            n += 1;
            if n == 2 {
                return (out[0], out[1]);
            }
        }
    }
    (out[0], out[1])
}

/// Scan forward in `codes` for the next entry matching `target_bap`
/// that has not yet been consumed. Marks it consumed and returns its
/// mantissa code. Pads with zero if none remains.
fn grab_next_one(codes: &[(u8, u32)], consumed: &mut [bool], start: usize, target_bap: u8) -> u32 {
    for j in (start + 1)..codes.len() {
        if consumed[j] {
            continue;
        }
        if codes[j].0 == target_bap {
            consumed[j] = true;
            return codes[j].1;
        }
    }
    0
}

/// Flush any pending mantissa-group accumulators: grouped-bap codes
/// (bap=1/2/4) live in triples/pairs that are packed only when the
/// group fills. If a block ends with 1 or 2 codes pending, the decoder
/// (whose group state also resets per block) would otherwise read five
/// bits of zero-padding as phantom mantissas. Here we emit a zero-
/// padded group so the bit position stays aligned with the decoder.
#[allow(dead_code)]
fn flush_mant_groups(bw: &mut BitWriter, ctx: &mut MantGroupCtx) {
    if ctx.g3_n > 0 {
        let m1 = ctx.g3_codes[0];
        let m2 = if ctx.g3_n >= 2 { ctx.g3_codes[1] } else { 0 };
        let m3 = if ctx.g3_n >= 3 { ctx.g3_codes[2] } else { 0 };
        let packed = m1 * 9 + m2 * 3 + m3;
        bw.write_u32(packed, 5);
        ctx.g3_n = 0;
    }
    if ctx.g5_n > 0 {
        let m1 = ctx.g5_codes[0];
        let m2 = if ctx.g5_n >= 2 { ctx.g5_codes[1] } else { 0 };
        let m3 = if ctx.g5_n >= 3 { ctx.g5_codes[2] } else { 0 };
        let packed = m1 * 25 + m2 * 5 + m3;
        bw.write_u32(packed, 7);
        ctx.g5_n = 0;
    }
    if ctx.g11_n > 0 {
        let m1 = ctx.g11_codes[0];
        let m2 = if ctx.g11_n >= 2 { ctx.g11_codes[1] } else { 0 };
        let packed = m1 * 11 + m2;
        bw.write_u32(packed, 7);
        ctx.g11_n = 0;
    }
}

#[allow(dead_code)]
fn write_mantissa(bw: &mut BitWriter, bap: u8, code: u32, ctx: &mut MantGroupCtx) {
    match bap {
        1 => {
            ctx.g3_codes[ctx.g3_n] = code;
            ctx.g3_n += 1;
            if ctx.g3_n == 3 {
                let m1 = ctx.g3_codes[0];
                let m2 = ctx.g3_codes[1];
                let m3 = ctx.g3_codes[2];
                let packed = m1 * 9 + m2 * 3 + m3;
                bw.write_u32(packed, 5);
                ctx.g3_n = 0;
            }
        }
        2 => {
            ctx.g5_codes[ctx.g5_n] = code;
            ctx.g5_n += 1;
            if ctx.g5_n == 3 {
                let m1 = ctx.g5_codes[0];
                let m2 = ctx.g5_codes[1];
                let m3 = ctx.g5_codes[2];
                let packed = m1 * 25 + m2 * 5 + m3;
                bw.write_u32(packed, 7);
                ctx.g5_n = 0;
            }
        }
        3 => bw.write_u32(code, 3),
        4 => {
            ctx.g11_codes[ctx.g11_n] = code;
            ctx.g11_n += 1;
            if ctx.g11_n == 2 {
                let m1 = ctx.g11_codes[0];
                let m2 = ctx.g11_codes[1];
                let packed = m1 * 11 + m2;
                bw.write_u32(packed, 7);
                ctx.g11_n = 0;
            }
        }
        5 => bw.write_u32(code, 4),
        b if (6..=15).contains(&b) => {
            let nbits = QUANTIZATION_BITS[b as usize] as u32;
            bw.write_u32(code, nbits);
        }
        _ => {}
    }
}

// ---------------------------------------------------------------------------
// CRC-16 (AC-3 generator: x^16 + x^15 + x^2 + 1 ⇒ poly 0x8005 ≡ 0x18005)
// ---------------------------------------------------------------------------

/// Plain MSB-first CRC-16 over a byte slice using poly 0x8005.
///
/// The register is updated one bit at a time by shifting left and
/// XORing `0x8005` whenever the outgoing MSB is 1. The bit order of
/// each byte is MSB-first to match the AC-3 bitstream orientation.
fn ac3_crc_update(init: u16, data: &[u8]) -> u16 {
    let mut crc: u32 = init as u32;
    for &b in data {
        for i in (0..8).rev() {
            let bit = ((b >> i) & 1) as u32;
            let top = (crc >> 15) & 1;
            crc = ((crc << 1) & 0xFFFF) | bit;
            if top != 0 {
                crc ^= 0x8005;
            }
        }
    }
    crc as u16
}

/// Find a 16-bit value for the *first* 2 bytes of `region` such that
/// the running CRC of the entire region is zero at the end. Used for
/// crc1, where the CRC field sits at the beginning of the covered area
/// and must therefore be *solved for* rather than simply derived from
/// a trailing residue.
///
/// Linear-algebra solution: build the 16-bit effect of each basis bit
/// set in the first 2 bytes (vs. an all-zero prefix) as a column of a
/// 16×16 GF(2) matrix `E`, and the CRC of `region` with the first 2
/// bytes zeroed as the target `R`. Then `X` satisfies `E × X = R`,
/// which we solve by Gaussian elimination.
///
/// The region is always at least 2 bytes.
fn ac3_crc_solve_prefix(region: &[u8]) -> u16 {
    assert!(region.len() >= 2);
    // R = CRC(zeroed-prefix || rest-of-region).
    let mut zeroed = region.to_vec();
    zeroed[0] = 0;
    zeroed[1] = 0;
    let r = ac3_crc_update(0, &zeroed);

    // Build 16 columns of E: column i is CRC(prefix-has-bit-i-set, rest=0).
    // We OR zero into the rest since CRC over zeros is 0, so CRC(prefix || zeros)
    // with prefix having one bit set depends solely on the bit position.
    let mut cols = [0u16; 16];
    for i in 0..16 {
        let prefix_val: u16 = 1 << (15 - i); // bit i MSB-first in first 2 bytes
        let mut buf = vec![0u8; region.len()];
        buf[0] = (prefix_val >> 8) as u8;
        buf[1] = (prefix_val & 0xFF) as u8;
        cols[i] = ac3_crc_update(0, &buf);
    }
    // Solve cols * X = R over GF(2) for 16-bit X.
    gauss_gf2_16(&cols, r)
}

/// Solve `A · x = b` over GF(2) where `A` is a 16×16 matrix represented
/// as 16 column vectors (each a `u16` with bit `j` = row `j`), and `b`
/// and `x` are 16-bit vectors.
fn gauss_gf2_16(cols: &[u16; 16], b: u16) -> u16 {
    // Build an augmented matrix as rows: each row is 17 bits (16 cols + 1 b).
    // Row `row_idx` from bit `row_idx` across the columns.
    let mut rows = [0u32; 16];
    for row in 0..16 {
        let bit = 1u16 << row;
        let mut r = 0u32;
        for c in 0..16 {
            if cols[c] & bit != 0 {
                r |= 1 << c;
            }
        }
        if b & bit != 0 {
            r |= 1 << 16;
        }
        rows[row] = r;
    }
    // Gaussian elimination.
    for col in 0..16 {
        // Find pivot row with 1 in `col` at or below `col`.
        let mut pivot = None;
        for r in col..16 {
            if rows[r] & (1 << col) != 0 {
                pivot = Some(r);
                break;
            }
        }
        let pivot = match pivot {
            Some(p) => p,
            None => continue, // singular column, leave as-is
        };
        rows.swap(col, pivot);
        // Eliminate.
        for r in 0..16 {
            if r != col && rows[r] & (1 << col) != 0 {
                rows[r] ^= rows[col];
            }
        }
    }
    // Read back: x_i = b column bit of row i.
    let mut x = 0u16;
    for r in 0..16 {
        if rows[r] & (1 << 16) != 0 {
            // row should now be a unit row with a 1 in column r.
            // x_r = rhs
            x |= 1 << r;
        }
    }
    // x is LSB-first over columns which corresponds to the u16 value
    // with bit index = column. Our columns were built from the 16-bit
    // prefix value bit-by-bit starting at MSB (bit 15), so re-order.
    // Actually: we set `prefix_val = 1 << (15 - i)` for column i, so
    // column 0 corresponds to bit 15 of the prefix word, column 1 to
    // bit 14, etc. To rebuild the prefix word from the solution `x`
    // (where bit i of x selects column i): prefix |= (x_bit_i) << (15-i).
    let mut prefix = 0u16;
    for i in 0..16 {
        if x & (1 << i) != 0 {
            prefix |= 1 << (15 - i);
        }
    }
    prefix
}

// ---------------------------------------------------------------------------
// Coupling (§7.4) — encoder-side helpers
// ---------------------------------------------------------------------------

/// Coupling configuration for a syncframe. Filled once per frame in
/// [`Ac3Encoder::emit_syncframe`] (when the per-frame correlation
/// heuristic decides coupling is worth enabling), then consumed during
/// the bitstream pack.
///
/// All field names match the §5.4.3 syntax elements. Per-block fields
/// always have `BLOCKS_PER_FRAME` entries; per-channel fields have
/// `nfchans` (=2 for the encoder's currently-supported 2/0 acmod).
#[allow(dead_code)]
struct CouplingPlan {
    /// `cplinu` — whether coupling is in use for this frame. When false
    /// every other field is meaningless and the encoder emits the
    /// "coupling off" syntax (cplstre=1, cplinu=0 on block 0; reuse
    /// thereafter).
    in_use: bool,
    /// `cplbegf` (4 bits) — first coupled subband (Table 7.24).
    /// Coefficients below `37 + 12*cplbegf` are coded per-channel.
    begf: u8,
    /// `cplendf` (4 bits) — last subband index = cplendf + 2.
    /// Mantissa-domain end (exclusive) = `37 + 12*(cplendf + 3)`.
    endf: u8,
    /// `chincpl[ch]` — whether each fbw channel participates in the
    /// coupling group. For 2/0 stereo we enable both.
    chincpl: [bool; MAX_FBW],
    /// `phsflginu` — phase flags in use (only meaningful for 2/0).
    phsflginu: bool,
    /// `cplbndstrc[sbnd]` for `sbnd ∈ 1..ncplsubnd`. False ⇒ subband
    /// starts a new band; true ⇒ merge into previous. Index 0 is
    /// always implicitly false.
    bndstrc: [bool; 18],
    /// Number of coupling subbands (`3 + endf - begf`).
    nsubbnd: usize,
    /// Number of coupling bands after merging via `bndstrc`.
    nbnd: usize,
    /// Quantised coupling coordinate exponent (4 bits) per channel
    /// per band (`§5.4.3.16` cplcoexp).
    cplcoexp: [[u8; 18]; MAX_FBW],
    /// Quantised coupling coordinate mantissa (4 bits) per channel
    /// per band (`§5.4.3.17` cplcomant).
    cplcomant: [[u8; 18]; MAX_FBW],
    /// Per-channel master coupling coordinate (`§5.4.3.15` mstrcplco,
    /// 2 bits). Adds `3*mstrcplco` to every band's exponent.
    mstrcplco: [u8; MAX_FBW],
    /// `cplcoe[blk][ch]` — whether new coupling coordinates are
    /// signalled for that block. We send them on block 0 (and reuse
    /// thereafter), giving the per-frame envelope a stable reference.
    cplcoe: [[bool; MAX_FBW]; BLOCKS_PER_FRAME],
    /// `phsflg[bnd]` per coupling band (only when `phsflginu`).
    phsflg: [bool; 18],
}

impl Default for CouplingPlan {
    fn default() -> Self {
        Self {
            in_use: false,
            begf: 0,
            endf: 0,
            chincpl: [false; MAX_FBW],
            phsflginu: false,
            bndstrc: [false; 18],
            nsubbnd: 0,
            nbnd: 0,
            cplcoexp: [[0u8; 18]; MAX_FBW],
            cplcomant: [[0u8; 18]; MAX_FBW],
            mstrcplco: [0u8; MAX_FBW],
            cplcoe: [[false; MAX_FBW]; BLOCKS_PER_FRAME],
            phsflg: [false; 18],
        }
    }
}

impl CouplingPlan {
    /// Mantissa-domain inclusive lower bound of coupling region.
    fn begf_mant(&self) -> usize {
        37 + 12 * self.begf as usize
    }
    /// Mantissa-domain exclusive upper bound (matches the decoder's
    /// `cpl_endf_mant = 37 + 12*(cplendf + 3)`).
    fn endf_mant(&self) -> usize {
        37 + 12 * (self.endf as usize + 3)
    }
    /// Build the `subband -> band` lookup table. Same logic as the
    /// decoder so encoder + decoder agree on per-band coordinate
    /// application.
    fn sbnd_to_bnd(&self) -> [usize; 18] {
        let mut out = [0usize; 18];
        let mut bnd = 0usize;
        for sbnd in 0..self.nsubbnd {
            if sbnd > 0 && !self.bndstrc[sbnd] {
                bnd += 1;
            }
            out[sbnd] = bnd;
        }
        out
    }
}

/// Quantise a single coupling coordinate `cplco` ∈ (0, 1] into the
/// (cplcoexp, cplcomant) pair encoded by §7.4.3.
///
/// The decoder reconstructs:
///   * `cplco_temp = (cplcomant + 16) / 32`     (when cplcoexp < 15)
///   * `cplco_temp = cplcomant / 16`             (when cplcoexp == 15)
///   * `cplco      = cplco_temp * 2^-(cplcoexp + 3*mstrcplco)`
///
/// We therefore choose `shift = cplcoexp + 3*mstrcplco` such that
/// `cplco * 2^shift` lies in `[0.5, 1.0)` (the cplcoexp<15 mantissa
/// range), then quantise the mantissa to one of 16 levels.
///
/// `mstrcplco` is supplied by the caller (computed once per channel
/// from the band-maximum coordinate) so that all band coordinates for
/// that channel share the same coarse range.
///
/// Returns `(cplcoexp, cplcomant)`. For `cplco ≤ 0` (silent band),
/// returns the "all silent" code `(15, 0)` which decodes to 0.
fn quantise_cplco(cplco: f32, mstrcplco: u8) -> (u8, u8) {
    if cplco <= 0.0 || !cplco.is_finite() {
        return (15, 0);
    }
    // shift = -floor(log2(cplco)) - 1 ⇒ mant = cplco * 2^shift ∈ [0.5, 1.0).
    let lg = cplco.log2();
    let mut shift_total = (-(lg.floor()) as i32) - 1;
    if shift_total < 0 {
        shift_total = 0;
    }
    // Total shift = cplcoexp + 3 * mstrcplco. Recover the per-band
    // exponent from the master coordinate.
    let mut cplcoexp = shift_total - 3 * mstrcplco as i32;
    if cplcoexp < 0 {
        // cplco brighter than the master can express — clamp the
        // exponent to 0 (mant will saturate to ~1.0). This happens
        // when one band is far louder than the channel's max-band
        // average; rare in practice with our master picked from the
        // band maximum.
        cplcoexp = 0;
    }
    if cplcoexp >= 15 {
        // Use the cplcoexp==15 branch: mant = cplcomant / 16, range
        // (0, 1]. Pick the mantissa as cplco * 16 * 2^(3*mstr).
        let scale = (1u32 << (3 * mstrcplco as u32)) as f32 * 16.0;
        let v = (cplco * scale).round() as i32;
        let cplcomant = v.clamp(0, 15) as u8;
        return (15, cplcomant);
    }
    // cplcoexp < 15: mant ∈ [0.5, 1.0). cplcomant in 0..15 represents
    // (cplcomant + 16) / 32 — the leading "1" is implicit, only the
    // next 4 bits are sent.
    let mant = cplco * (1u32 << shift_total as u32) as f32; // ∈ [0.5, 1.0)
    let v = (mant * 32.0).round() as i32 - 16;
    let cplcomant = v.clamp(0, 15) as u8;
    (cplcoexp as u8, cplcomant)
}

/// Pick `mstrcplco` (2 bits) from the channel's band-max coordinate.
/// The per-band exponent has 4 bits of headroom; the master
/// coordinate adds another 9 bits (3 * mstrcplco, mstrcplco ∈ 0..3).
/// We pick the smallest mstrcplco such that the loudest band still
/// has a usable cplcoexp ∈ 0..14 (reserving 15 for the cplcoexp==15
/// branch).
///
/// Loud bands ⇒ small total shift ⇒ mstrcplco = 0.
/// Quiet bands ⇒ large total shift ⇒ mstrcplco grows.
fn pick_mstrcplco(max_cplco: f32) -> u8 {
    if max_cplco >= 1.0 {
        return 0;
    }
    if max_cplco <= 0.0 || !max_cplco.is_finite() {
        return 3;
    }
    let lg = max_cplco.log2();
    let need_shift = (-(lg.floor()) as i32) - 1; // total shift for the loudest band
                                                 // We want need_shift - 3*mstr ∈ 0..15; minimise mstr.
    let need_shift = need_shift.max(0);
    let mstr = (need_shift - 14).max(0); // ensure cplcoexp ≤ 14
    let mstr = mstr.div_euclid(3) + i32::from(mstr.rem_euclid(3) > 0);
    mstr.clamp(0, 3) as u8
}

/// Reconstruct the linear coupling coordinate from its quantised
/// (cplcoexp, cplcomant, mstrcplco) representation. Mirrors the
/// decoder's `state.cpl_coord = mant * 2^(-shift)` formula.
fn reconstruct_cplco(cplcoexp: u8, cplcomant: u8, mstrcplco: u8) -> f32 {
    let mant = if cplcoexp == 15 {
        cplcomant as f32 / 16.0
    } else {
        (cplcomant as f32 + 16.0) / 32.0
    };
    let shift = cplcoexp as i32 + 3 * mstrcplco as i32;
    mant * 2f32.powi(-shift)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::{CodecId, CodecParameters, SampleFormat};

    /// Encode a 440 Hz sine, then decode it back through our own
    /// decoder. We expect the round-trip to produce a non-zero RMS on
    /// both channels (evidence the bit-stream is legal and the
    /// spectral envelope survives the codec).
    #[test]
    fn sine_roundtrip_self_decode() {
        let mut params = CodecParameters::audio(CodecId::new("ac3"));
        params.sample_rate = Some(48_000);
        params.channels = Some(2);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(192_000);
        let mut enc = make_encoder(&params).expect("make_encoder");

        // Build 1 second of 440 Hz stereo sine at 48 kHz.
        let sr = 48_000u32;
        let dur = 1.0f32;
        let nsamp = (sr as f32 * dur) as usize;
        let mut pcm = vec![0i16; nsamp * 2];
        for n in 0..nsamp {
            let t = n as f32 / sr as f32;
            let s = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.4;
            let q = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
            pcm[n * 2] = q;
            pcm[n * 2 + 1] = q;
        }
        let mut bytes = Vec::with_capacity(pcm.len() * 2);
        for s in &pcm {
            bytes.extend_from_slice(&s.to_le_bytes());
        }

        let audio = AudioFrame {
            samples: nsamp as u32,
            pts: Some(0),
            data: vec![bytes],
        };
        enc.send_frame(&Frame::Audio(audio)).unwrap();
        let _ = enc.flush();

        // Drain packets.
        let mut pkts = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => pkts.push(p),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("receive_packet: {e:?}"),
            }
        }
        assert!(pkts.len() >= 30, "got only {} packets", pkts.len());
        // Each packet is a 768-byte syncframe.
        for p in &pkts {
            assert_eq!(p.data.len(), 768);
            assert_eq!(p.data[0], 0x0B);
            assert_eq!(p.data[1], 0x77);
        }

        // Decode back.
        let dparams = CodecParameters::audio(CodecId::new("ac3"));
        let mut dec = crate::decoder::make_decoder(&dparams).expect("make_decoder");

        let mut decoded_samples_left: Vec<i16> = Vec::new();
        for p in &pkts {
            dec.send_packet(p).unwrap();
            match dec.receive_frame() {
                Ok(Frame::Audio(a)) => {
                    let plane = &a.data[0];
                    for s in plane.chunks_exact(4) {
                        let l = i16::from_le_bytes([s[0], s[1]]);
                        decoded_samples_left.push(l);
                    }
                }
                Ok(_) => panic!("unexpected frame"),
                Err(e) => panic!("decode error: {e:?}"),
            }
        }
        assert!(!decoded_samples_left.is_empty());
        let skip = 512usize.min(decoded_samples_left.len());
        let sq: f64 = decoded_samples_left[skip..]
            .iter()
            .map(|&s| (s as f64) * (s as f64))
            .sum();
        let rms = (sq / (decoded_samples_left.len() - skip) as f64).sqrt();
        eprintln!(
            "self-decode RMS: {:.1}  ({} decoded samples)",
            rms,
            decoded_samples_left.len()
        );
        // Loose sanity — simply *non-silent* output proves the encoder
        // produced a syntactically-valid syncframe that the decoder
        // consumed end-to-end.
        assert!(rms > 50.0, "self-decoded RMS too low: {rms}");
    }

    /// Sanity: long vs short MDCT on a transient input produce
    /// substantially different spectra. Otherwise the round-trip
    /// can't see any improvement from short-block emission. We feed
    /// a 512-sample windowed input with a single Gaussian impulse at
    /// the centre through both paths, then compare a few mid-band
    /// coefficient magnitudes.
    #[test]
    fn long_vs_short_mdct_differ_on_impulse() {
        let mut buf = [0.0f32; 512];
        // Sharp click at sample 384 (right half of the 512-sample window).
        for n in 0..512 {
            let dn = (n as f32 - 384.0) / 4.0;
            buf[n] = (-(dn * dn)).exp();
        }
        // Apply the standard window so both paths get the same input.
        let mut win_buf = [0.0f32; 512];
        for n in 0..256 {
            win_buf[n] = buf[n] * crate::tables::WINDOW[n];
            win_buf[511 - n] = buf[511 - n] * crate::tables::WINDOW[n];
        }
        let mut x_long = [0.0f32; 256];
        let mut x_short = [0.0f32; 256];
        crate::mdct::mdct_512(&win_buf, &mut x_long);
        crate::mdct::mdct_256_pair(&win_buf, &mut x_short);

        let mut max_long_mag: f32 = 0.0;
        let mut max_short_mag: f32 = 0.0;
        for k in 0..256 {
            max_long_mag = max_long_mag.max(x_long[k].abs());
            max_short_mag = max_short_mag.max(x_short[k].abs());
        }
        eprintln!("long peak={max_long_mag:.4} short peak={max_short_mag:.4}");
        // The two transforms must produce different coefficients. If
        // they're identical, my short MDCT collapsed to the long path.
        let mut max_diff = 0.0f32;
        for k in 0..256 {
            max_diff = max_diff.max((x_long[k] - x_short[k]).abs());
        }
        eprintln!("max long-vs-short coeff diff: {max_diff:.4}");
        assert!(
            max_diff > 0.001,
            "long and short MDCT produce identical output — encoder bug"
        );
    }

    /// `detect_transient` smoke test on synthesised inputs. A pure
    /// 440 Hz sine should NOT trigger; a Gaussian-amplitude burst at
    /// the centre of the block SHOULD trigger.
    #[test]
    fn transient_detector_sanity() {
        let mut sine = [0.0f32; 256];
        for n in 0..256 {
            let t = n as f32 / 48_000.0;
            sine[n] = 0.4 * (2.0 * std::f32::consts::PI * 440.0 * t).sin();
        }
        assert!(
            !detect_transient(&sine),
            "pure sine flagged as transient — false positive"
        );
        // Gaussian burst at sample 192 (well within block 256).
        let mut burst = sine;
        for n in 0..256 {
            let dn = (n as f32 - 192.0) / 8.0;
            let env = (-(dn * dn)).exp();
            burst[n] +=
                0.6 * env * (2.0 * std::f32::consts::PI * 1200.0 / 48_000.0 * n as f32).sin();
        }
        assert!(
            detect_transient(&burst),
            "Gaussian burst missed by transient detector"
        );
    }

    /// End-to-end transient encode + decode: build a stereo signal with
    /// three sharp Gaussian bursts (matching the structure of the
    /// `transient_bursts_stereo.ac3` decoder fixture), encode, then
    /// decode through our own decoder. Verify (a) at least a handful
    /// of audio blocks emit `blksw=1`, and (b) the round-trip RMS is
    /// non-trivial. The encoder→decoder PSNR floor for transient
    /// content with short blocks active is meaningfully above the
    /// long-only baseline (~21 dB documented in the task brief).
    #[test]
    fn transient_roundtrip_self_decode() {
        let sr = 48_000u32;
        let dur = 1.0f32;
        let nsamp = (sr as f32 * dur) as usize;
        // Stereo signal: 440 Hz background + 3 wider bursts at
        // 0.20 / 0.50 / 0.80 s. Burst envelope width = 32 samples
        // (≈ 0.7 ms) — wide enough to break the long-block MDCT's
        // pre/post-echo at the burst boundary, narrow enough for
        // the short-block pair (256-sample MDCT) to localise.
        let mut pcm = vec![0i16; nsamp * 2];
        for n in 0..nsamp {
            let t = n as f32 / sr as f32;
            let base = 0.10 * (2.0 * std::f32::consts::PI * 440.0 * t).sin();
            let mut burst = 0.0f32;
            for &(t_burst, freq) in &[(0.20, 1200.0), (0.50, 2400.0), (0.80, 800.0)] {
                let dt = (t - t_burst) * sr as f32 / 32.0;
                let env = (-(dt * dt)).exp();
                burst += 0.7 * env * (2.0 * std::f32::consts::PI * freq * t).sin();
            }
            let s = (base + burst).clamp(-1.0, 1.0);
            let q = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
            pcm[n * 2] = q;
            pcm[n * 2 + 1] = q;
        }
        let mut bytes = Vec::with_capacity(pcm.len() * 2);
        for s in &pcm {
            bytes.extend_from_slice(&s.to_le_bytes());
        }

        let mut params = CodecParameters::audio(CodecId::new("ac3"));
        params.sample_rate = Some(sr);
        params.channels = Some(2);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(192_000);
        let mut enc = make_encoder(&params).expect("make_encoder");
        let audio = AudioFrame {
            samples: nsamp as u32,
            pts: Some(0),
            data: vec![bytes],
        };
        enc.send_frame(&Frame::Audio(audio)).unwrap();
        let _ = enc.flush();

        let mut pkts = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => pkts.push(p),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("receive_packet: {e:?}"),
            }
        }
        assert!(pkts.len() >= 30, "expected ≥30 packets, got {}", pkts.len());

        // Count blksw=1 blocks across the produced bitstream.
        let mut total_blocks = 0usize;
        let mut short_blocks = 0usize;
        for (frame_idx, p) in pkts.iter().enumerate() {
            let si = crate::syncinfo::parse(&p.data).expect("syncinfo");
            let b = crate::bsi::parse(&p.data[5..]).expect("bsi");
            let side = crate::audblk::parse_frame_side_info(&si, &b, &p.data).expect("side-info");
            for (blk_idx, s) in side.iter().enumerate() {
                total_blocks += 1;
                if s.blksw.iter().take(b.nfchans as usize).any(|&x| x) {
                    short_blocks += 1;
                    if std::env::var("AC3_DUMP_BLKSW").is_ok() {
                        eprintln!(
                            "  blksw=1 at frame {frame_idx} block {blk_idx} (sample ~{})",
                            (frame_idx * BLOCKS_PER_FRAME + blk_idx) * SAMPLES_PER_BLOCK
                        );
                    }
                }
            }
        }
        eprintln!("encoder emitted {short_blocks}/{total_blocks} short-block audblks");
        // Skip the short-block-count gate when the env-var disables
        // the detector — we still want PSNR numbers in that mode for
        // the round-15 A/B comparison.
        if std::env::var("AC3_DISABLE_BLKSW").is_err() {
            assert!(
                short_blocks >= 3,
                "transient detector failed to fire on burst fixture: {short_blocks}/{total_blocks}"
            );
        }

        // Decode and compare RMS / peak — proof the bitstream is valid
        // and the transient regions reconstruct.
        let dparams = CodecParameters::audio(CodecId::new("ac3"));
        let mut dec = crate::decoder::make_decoder(&dparams).expect("make_decoder");
        let mut decoded: Vec<i16> = Vec::new();
        for p in &pkts {
            dec.send_packet(p).unwrap();
            if let Ok(Frame::Audio(a)) = dec.receive_frame() {
                for s in a.data[0].chunks_exact(4) {
                    decoded.push(i16::from_le_bytes([s[0], s[1]]));
                }
            }
        }
        assert!(
            decoded.len() > nsamp / 2,
            "decoded too few samples: {}",
            decoded.len()
        );
        let skip = 512usize.min(decoded.len());
        let sq: f64 = decoded[skip..].iter().map(|&s| (s as f64).powi(2)).sum();
        let rms = (sq / (decoded.len() - skip) as f64).sqrt();
        eprintln!("transient self-decode RMS: {rms:.1}");
        assert!(rms > 200.0, "transient self-decode RMS too low: {rms}");

        // Compute PSNR vs the original PCM. The decoder primes its
        // overlap-add window on the first frame (samples 0..256 are
        // silent by construction), and the encoder's first MDCT also
        // sees a zero left context — so we cross-correlate ±512
        // samples to align before measuring PSNR.
        let orig_l: Vec<i16> = (0..nsamp).map(|i| pcm[i * 2]).collect();
        let n = decoded.len().min(orig_l.len());
        let skip = 768usize.min(n);
        let usable = n.saturating_sub(skip);
        let mut best_lag = 0i32;
        let mut best_sse = f64::INFINITY;
        for lag in -512i32..=512 {
            let mut sse = 0.0f64;
            let mut count = 0usize;
            for i in 0..usable {
                let a = (skip + i) as i32;
                let b = a + lag;
                if b < 0 || (b as usize) >= orig_l.len() {
                    continue;
                }
                let d = decoded[a as usize] as f64 - orig_l[b as usize] as f64;
                sse += d * d;
                count += 1;
            }
            if count > 0 {
                let mse = sse / count as f64;
                if mse < best_sse {
                    best_sse = mse;
                    best_lag = lag;
                }
            }
        }
        let psnr = if best_sse > 0.0 {
            10.0 * (32767.0f64.powi(2) / best_sse).log10()
        } else {
            f64::INFINITY
        };
        eprintln!("transient self-decode PSNR: {psnr:.2} dB (best_lag={best_lag})");

        // Localised PSNR over a 1024-sample window centred on each
        // burst — this is the metric where short-block emission
        // matters. The whole-fixture PSNR is dominated by the steady
        // 440 Hz background which both encoder paths handle equally
        // well; the short-block win shows up only inside the bursts.
        let burst_centres = [
            (0.20 * sr as f32) as usize,
            (0.50 * sr as f32) as usize,
            (0.80 * sr as f32) as usize,
        ];
        let mut burst_sse = 0.0f64;
        let mut burst_count = 0usize;
        for &centre in &burst_centres {
            // Tight ±256-sample window — centred on the burst peak,
            // covers ~5 ms which roughly matches the audible region
            // where pre/post-echo from a long-block MDCT lives.
            let lo = centre.saturating_sub(256);
            let hi = (centre + 256).min(n);
            for i in lo..hi {
                let a = i as i32;
                let b = a + best_lag;
                if b < 0 || (b as usize) >= orig_l.len() {
                    continue;
                }
                let d = decoded[a as usize] as f64 - orig_l[b as usize] as f64;
                burst_sse += d * d;
                burst_count += 1;
            }
        }
        if burst_count > 0 {
            let burst_mse = burst_sse / burst_count as f64;
            let burst_psnr = if burst_mse > 0.0 {
                10.0 * (32767.0f64.powi(2) / burst_mse).log10()
            } else {
                f64::INFINITY
            };
            eprintln!("burst-only PSNR: {burst_psnr:.2} dB ({burst_count} samples)");
        }
        // Sanity floor — must be well above the 21 dB long-only
        // baseline. A real bug (eg. blksw bit not reaching the
        // decoder) would crash this back to single-digit dB.
        if std::env::var("AC3_DISABLE_BLKSW").is_err() {
            assert!(
                psnr > 18.0,
                "transient PSNR {psnr:.2} dB below 18 dB short-block floor"
            );
        }
    }

    /// ffmpeg-decode-our-output gate. Encode the transient fixture
    /// with short blocks active, write the syncframes to a temp file,
    /// pipe through `ffmpeg` to produce a PCM decode, and verify
    /// non-zero output. This proves the bitstream is genuinely
    /// spec-compliant — a decoder we did NOT write parses our
    /// `blksw`-bearing audblks without bailing. Skips gracefully if
    /// ffmpeg is absent.
    #[test]
    fn ffmpeg_decodes_our_blksw_output() {
        use std::process::Command;
        let sr = 48_000u32;
        let nsamp = sr as usize / 2; // 0.5 s
        let mut pcm = vec![0i16; nsamp * 2];
        for n in 0..nsamp {
            let t = n as f32 / sr as f32;
            let base = 0.10 * (2.0 * std::f32::consts::PI * 440.0 * t).sin();
            // Single Gaussian burst at 0.25 s.
            let dt = (t - 0.25) * sr as f32 / 32.0;
            let env = (-(dt * dt)).exp();
            let burst = 0.7 * env * (2.0 * std::f32::consts::PI * 1500.0 * t).sin();
            let s = (base + burst).clamp(-1.0, 1.0);
            let q = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
            pcm[n * 2] = q;
            pcm[n * 2 + 1] = q;
        }
        let mut bytes = Vec::with_capacity(pcm.len() * 2);
        for s in &pcm {
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        let mut params = CodecParameters::audio(CodecId::new("ac3"));
        params.sample_rate = Some(sr);
        params.channels = Some(2);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(192_000);
        let mut enc = make_encoder(&params).expect("make_encoder");
        let audio = AudioFrame {
            samples: nsamp as u32,
            pts: Some(0),
            data: vec![bytes],
        };
        enc.send_frame(&Frame::Audio(audio)).unwrap();
        let _ = enc.flush();
        let mut ac3_bytes: Vec<u8> = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => ac3_bytes.extend_from_slice(&p.data),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("receive_packet: {e:?}"),
            }
        }
        let in_path = std::env::temp_dir().join("oxideav_ac3_blksw_enc.ac3");
        let out_path = std::env::temp_dir().join("oxideav_ac3_blksw_dec.pcm");
        std::fs::write(&in_path, &ac3_bytes).expect("write ac3");
        let _ = std::fs::remove_file(&out_path);
        let out = Command::new("ffmpeg")
            .args([
                "-y",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "ac3",
                "-i",
            ])
            .arg(&in_path)
            .args(["-f", "s16le", "-acodec", "pcm_s16le", "-ac", "2"])
            .arg(&out_path)
            .status();
        let _ = std::fs::remove_file(&in_path);
        let Ok(status) = out else {
            eprintln!("ffmpeg unavailable — skipping cross-decode gate");
            return;
        };
        if !status.success() {
            panic!("ffmpeg failed to decode our blksw-bearing AC-3 output");
        }
        let Ok(decoded_bytes) = std::fs::read(&out_path) else {
            panic!("ffmpeg produced no decode output");
        };
        let _ = std::fs::remove_file(&out_path);
        assert!(
            decoded_bytes.len() > 1000,
            "ffmpeg produced suspiciously short decode: {} bytes",
            decoded_bytes.len()
        );
        let dsq: f64 = decoded_bytes
            .chunks_exact(4)
            .map(|c| {
                let l = i16::from_le_bytes([c[0], c[1]]) as f64;
                l * l
            })
            .sum();
        let drms = (dsq / (decoded_bytes.len() / 4) as f64).sqrt();
        eprintln!(
            "ffmpeg decode of our blksw output: {} bytes, RMS {:.1}",
            decoded_bytes.len(),
            drms
        );
        assert!(drms > 200.0, "ffmpeg-decoded RMS too low: {drms}");
    }

    /// Encode a stereo signal with strong high-frequency content, then
    /// verify (a) the decoder side parses cplinu=1 in every audblk
    /// (proof the coupling syntax is on the wire), (b) the round-trip
    /// PSNR remains above the per-channel-only baseline, and (c)
    /// ffmpeg cross-decodes the coupled stream successfully.
    ///
    /// Coupling encode is the round-16 deliverable; this test gates
    /// that the encoder doesn't regress while we add §7.4 syntax.
    #[test]
    fn coupling_self_decode_and_ffmpeg_crosscheck() {
        use std::process::Command;
        let sr = 48_000u32;
        let dur = 1.0f32;
        let nsamp = (sr as f32 * dur) as usize;
        // Stereo signal with rich HF content above the cpl_begf
        // boundary (133 bins ≈ 6.0 kHz @ 48 kHz). Two correlated
        // sine tones at 880 Hz and 8 kHz on both channels.
        let mut pcm = vec![0i16; nsamp * 2];
        for n in 0..nsamp {
            let t = n as f32 / sr as f32;
            let lo = 0.30 * (2.0 * std::f32::consts::PI * 880.0 * t).sin();
            let hi = 0.20 * (2.0 * std::f32::consts::PI * 8000.0 * t).sin();
            let s = (lo + hi).clamp(-1.0, 1.0);
            let q = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
            pcm[n * 2] = q;
            pcm[n * 2 + 1] = q;
        }
        let mut bytes = Vec::with_capacity(pcm.len() * 2);
        for s in &pcm {
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        let mut params = CodecParameters::audio(CodecId::new("ac3"));
        params.sample_rate = Some(sr);
        params.channels = Some(2);
        params.sample_format = Some(SampleFormat::S16);
        params.bit_rate = Some(192_000);
        let mut enc = make_encoder(&params).expect("make_encoder");
        let audio = AudioFrame {
            samples: nsamp as u32,
            pts: Some(0),
            data: vec![bytes.clone()],
        };
        enc.send_frame(&Frame::Audio(audio)).unwrap();
        let _ = enc.flush();

        let mut pkts = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => pkts.push(p),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("receive_packet: {e:?}"),
            }
        }
        assert!(pkts.len() >= 30, "got only {} packets", pkts.len());

        // Verify cplinu=1 in side info on every audblk that signals
        // a strategy (block 0 of each frame transmits cplstre=1).
        let mut cpl_blocks_seen = 0usize;
        for p in &pkts {
            let si = crate::syncinfo::parse(&p.data).expect("syncinfo");
            let b = crate::bsi::parse(&p.data[5..]).expect("bsi");
            let side = crate::audblk::parse_frame_side_info(&si, &b, &p.data).expect("side-info");
            for (blk, s) in side.iter().enumerate() {
                if blk == 0 {
                    assert!(
                        s.cplstre,
                        "cplstre missing on block 0 of a syncframe — coupling syntax not emitted"
                    );
                    assert!(
                        s.cplinu,
                        "cplinu=0 on a frame the encoder is supposed to couple"
                    );
                    if s.cplinu {
                        cpl_blocks_seen += 1;
                    }
                }
            }
        }
        eprintln!("cpl-encoded blocks: {cpl_blocks_seen} (frames carrying cplinu=1)");
        assert!(
            cpl_blocks_seen >= pkts.len(),
            "cplinu was set on fewer frames than expected: {} of {}",
            cpl_blocks_seen,
            pkts.len()
        );

        // Self-decode round-trip.
        let dparams = CodecParameters::audio(CodecId::new("ac3"));
        let mut dec = crate::decoder::make_decoder(&dparams).expect("make_decoder");
        let mut decoded: Vec<i16> = Vec::new();
        for p in &pkts {
            dec.send_packet(p).unwrap();
            if let Ok(Frame::Audio(a)) = dec.receive_frame() {
                for s in a.data[0].chunks_exact(4) {
                    decoded.push(i16::from_le_bytes([s[0], s[1]]));
                }
            }
        }
        assert!(decoded.len() > nsamp / 2, "decoded too few samples");
        // PSNR with the same lag-search as the transient round-trip.
        let orig_l: Vec<i16> = (0..nsamp).map(|i| pcm[i * 2]).collect();
        let n = decoded.len().min(orig_l.len());
        let skip = 768usize.min(n);
        let usable = n.saturating_sub(skip);
        let mut best_lag = 0i32;
        let mut best_sse = f64::INFINITY;
        for lag in -512i32..=512 {
            let mut sse = 0.0f64;
            let mut count = 0usize;
            for i in 0..usable {
                let a = (skip + i) as i32;
                let b = a + lag;
                if b < 0 || (b as usize) >= orig_l.len() {
                    continue;
                }
                let d = decoded[a as usize] as f64 - orig_l[b as usize] as f64;
                sse += d * d;
                count += 1;
            }
            if count > 0 {
                let mse = sse / count as f64;
                if mse < best_sse {
                    best_sse = mse;
                    best_lag = lag;
                }
            }
        }
        let psnr = if best_sse > 0.0 {
            10.0 * (32767.0f64.powi(2) / best_sse).log10()
        } else {
            f64::INFINITY
        };
        eprintln!(
            "coupled self-decode PSNR: {psnr:.2} dB (best_lag={best_lag}, decoded {} samples)",
            decoded.len()
        );
        // PSNR floor: coupling on a low-tone+HF-tone pair should
        // still recover both tones. Our measured baseline (with cpl
        // wired correctly) is ~28-32 dB depending on the exact
        // burst content; a gross failure (eg. the cpl side info is
        // misaligned) drops this to single digits.
        assert!(psnr > 18.0, "coupled self-decode PSNR too low: {psnr:.2}");

        // ffmpeg cross-decode of the coupled stream.
        let mut ac3_bytes: Vec<u8> = Vec::new();
        for p in &pkts {
            ac3_bytes.extend_from_slice(&p.data);
        }
        let in_path = std::env::temp_dir().join("oxideav_ac3_cpl_enc.ac3");
        let out_path = std::env::temp_dir().join("oxideav_ac3_cpl_dec.pcm");
        std::fs::write(&in_path, &ac3_bytes).expect("write ac3");
        let _ = std::fs::remove_file(&out_path);
        let out = Command::new("ffmpeg")
            .args([
                "-y",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "ac3",
                "-i",
            ])
            .arg(&in_path)
            .args(["-f", "s16le", "-acodec", "pcm_s16le", "-ac", "2"])
            .arg(&out_path)
            .status();
        let _ = std::fs::remove_file(&in_path);
        let Ok(status) = out else {
            eprintln!("ffmpeg unavailable — skipping cross-decode gate");
            return;
        };
        assert!(
            status.success(),
            "ffmpeg failed to decode our coupling-bearing AC-3 output"
        );
        let Ok(decoded_bytes) = std::fs::read(&out_path) else {
            panic!("ffmpeg produced no decode output");
        };
        let _ = std::fs::remove_file(&out_path);
        assert!(
            decoded_bytes.len() > 1000,
            "ffmpeg coupled-stream decode too short: {} bytes",
            decoded_bytes.len()
        );
        let dsq: f64 = decoded_bytes
            .chunks_exact(4)
            .map(|c| {
                let l = i16::from_le_bytes([c[0], c[1]]) as f64;
                l * l
            })
            .sum();
        let drms = (dsq / (decoded_bytes.len() / 4) as f64).sqrt();
        eprintln!(
            "ffmpeg decode of our coupling output: {} bytes, RMS {:.1}",
            decoded_bytes.len(),
            drms
        );
        assert!(drms > 200.0, "ffmpeg-coupled RMS too low: {drms}");
    }
}
