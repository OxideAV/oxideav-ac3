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

use oxideav_codec::Encoder;
use oxideav_core::bits::BitWriter;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::audblk::{BLOCKS_PER_FRAME, N_COEFFS, SAMPLES_PER_BLOCK};
use crate::decoder::SAMPLES_PER_FRAME;
use crate::mdct::mdct_512;
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

    Ok(Box::new(Ac3Encoder {
        codec_id: CodecId::new(crate::CODEC_ID_STR),
        out_params,
        sample_rate,
        channels: channels as usize,
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
        if audio.channels as usize != self.channels {
            return Err(Error::invalid(format!(
                "ac3 encoder: expected {} channels, got {}",
                self.channels, audio.channels
            )));
        }
        if audio.sample_rate != self.sample_rate {
            return Err(Error::invalid(format!(
                "ac3 encoder: expected {} Hz input, got {} Hz",
                self.sample_rate, audio.sample_rate
            )));
        }
        // Extract per-channel f32 samples from the interleaved input.
        let per_chan = decode_input_samples(audio)?;
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
fn decode_input_samples(a: &AudioFrame) -> Result<Vec<Vec<f32>>> {
    let nch = a.channels as usize;
    let nsamp = a.samples as usize;
    let mut out = vec![Vec::with_capacity(nsamp); nch];
    match a.format {
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
        for ch in 0..self.channels {
            let drain: Vec<f32> = self.pending_samples[ch].drain(0..n_per).collect();
            for blk in 0..BLOCKS_PER_FRAME {
                // Build 512-sample input: left context + next 256.
                let mut in_buf = [0.0f32; 512];
                in_buf[..256].copy_from_slice(&self.delay_line[ch]);
                in_buf[256..].copy_from_slice(
                    &drain[blk * SAMPLES_PER_BLOCK..(blk + 1) * SAMPLES_PER_BLOCK],
                );
                // Windowing (symmetric 512-sample AC-3 window).
                let mut win_buf = [0.0f32; 512];
                for n in 0..256 {
                    win_buf[n] = in_buf[n] * WINDOW[n];
                    win_buf[511 - n] = in_buf[511 - n] * WINDOW[n];
                }
                // Update delay line to right-half of the next block.
                self.delay_line[ch].copy_from_slice(
                    &drain[blk * SAMPLES_PER_BLOCK..(blk + 1) * SAMPLES_PER_BLOCK],
                );
                // Forward MDCT.
                mdct_512(&win_buf, &mut coeffs[ch][blk]);
            }
        }

        // Per-block exponents: channels × blocks × N_COEFFS (u8 in 0..=24).
        let mut exps: Vec<Vec<[u8; N_COEFFS]>> =
            vec![vec![[24u8; N_COEFFS]; BLOCKS_PER_FRAME]; self.channels];
        // Limit active bins: the decoder starts from end_mant = 37 + 3*(chbwcod+12).
        // Use chbwcod=60 → end_mant=253 (full bandwidth minus the top 3 bins).
        let chbwcod: u8 = 60;
        let end_mant: usize = 37 + 3 * (chbwcod as usize + 12);
        for ch in 0..self.channels {
            for blk in 0..BLOCKS_PER_FRAME {
                for k in 0..end_mant {
                    exps[ch][blk][k] = extract_exponent(coeffs[ch][blk][k]);
                }
            }
        }

        // Choose a single exponent strategy per channel for this whole
        // syncframe: block 0 always D15, blocks 1..=5 "reuse". This is
        // the simplest legal choice and matches what a very basic
        // encoder does on stationary signals (§8.2.8).
        //
        // Pre-process exponents so that D15 differentials stay within
        // the ±2 per-step legal range (Table 7.1 / §8.2.10).
        for ch in 0..self.channels {
            // Only block 0 transmits its exponents; blocks 1..5 reuse.
            // Propagate block 0's preprocessed exponents into the other
            // blocks so their dequantised coefficients use the same set
            // (the decoder will apply them verbatim).
            preprocess_d15(&mut exps[ch][0][..end_mant]);
            let blk0: [u8; N_COEFFS] = exps[ch][0];
            for blk in 1..BLOCKS_PER_FRAME {
                exps[ch][blk][..end_mant].copy_from_slice(&blk0[..end_mant]);
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
            end_mant,
            self.channels,
            self.fscod,
            self.frame_bytes,
        );

        // Compute bap arrays per channel per block using the tuned params.
        let mut baps: Vec<Vec<[u8; N_COEFFS]>> =
            vec![vec![[0u8; N_COEFFS]; BLOCKS_PER_FRAME]; self.channels];
        for ch in 0..self.channels {
            for blk in 0..BLOCKS_PER_FRAME {
                compute_bap(
                    &exps[ch][blk],
                    end_mant,
                    self.fscod,
                    &tuned_ba,
                    &mut baps[ch][blk],
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
            // blksw per channel: 0 (long blocks only).
            bw.write_u32(0, 1);
            bw.write_u32(0, 1);
            // dithflag per channel: 1 (enable dither).
            bw.write_u32(1, 1);
            bw.write_u32(1, 1);
            // dynrnge = 0 (no dynrng transmitted; block 0 sets gain=1).
            bw.write_u32(0, 1);
            // cplstre: block 0 sends "coupling off, do not use"; others reuse.
            if blk == 0 {
                bw.write_u32(1, 1); // cplstre
                bw.write_u32(0, 1); // cplinu = 0 (no coupling)
            } else {
                bw.write_u32(0, 1); // reuse
            }
            // rematstr (acmod == 2): block 0 sends "no rematrix" (rematstr=1, all flags=0).
            if blk == 0 {
                bw.write_u32(1, 1); // rematstr
                                    // With cplinu=0 → 4 rematrix bands (Table 5.15).
                for _ in 0..4 {
                    bw.write_u32(0, 1);
                }
            } else {
                bw.write_u32(0, 1); // no rematrix this block
            }

            // chexpstr: block 0 = D15 (=1), blocks 1..5 = reuse (=0).
            let exp_strategy: u8 = if blk == 0 { 1 } else { 0 };
            bw.write_u32(exp_strategy as u32, 2); // ch0
            bw.write_u32(exp_strategy as u32, 2); // ch1
                                                  // chbwcod (only when exp strategy != reuse, and channel not coupled).
            if exp_strategy != 0 {
                bw.write_u32(chbwcod as u32, 6); // ch0
                bw.write_u32(chbwcod as u32, 6); // ch1
            }

            // Exponents: only transmitted when chexpstr != reuse (block 0).
            if blk == 0 {
                for ch in 0..self.channels {
                    write_exponents_d15(&mut bw, &exps[ch][0], end_mant);
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
                bw.write_u32(tuned_ba.fsnroffst as u32, 4); // ch0
                bw.write_u32(tuned_ba.fgaincod as u32, 3); // ch0
                bw.write_u32(tuned_ba.fsnroffst as u32, 4); // ch1
                bw.write_u32(tuned_ba.fgaincod as u32, 3); // ch1
            }
            // deltbaie = 0 (no delta bit allocation).
            bw.write_u32(0, 1);

            // skiple / skipl: potentially used at frame-end to pad out to
            // frame_bytes; for now, none per block.
            bw.write_u32(0, 1);

            // Mantissas per channel.
            let mut ctx = MantGroupCtx::default();
            for ch in 0..self.channels {
                for bin in 0..end_mant {
                    let bap = baps[ch][blk][bin];
                    if bap == 0 {
                        continue;
                    }
                    let e = exps[ch][blk][bin] as i32;
                    let mant = quantise_mantissa(coeffs[ch][blk][bin], e, bap);
                    write_mantissa(&mut bw, bap, mant, &mut ctx);
                }
            }
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
        // itself at the tail. Here the crc field is at the END, so the
        // residue-after-zero-tail value IS the value we write.
        let crc2_val = ac3_crc_update(0, &frame[five_eighths_bytes..(self.frame_bytes - 2)]);
        let n = self.frame_bytes;
        frame[n - 2] = (crc2_val >> 8) as u8;
        frame[n - 1] = (crc2_val & 0xFF) as u8;
        debug_assert_eq!(
            ac3_crc_update(0, &frame[five_eighths_bytes..self.frame_bytes]),
            0,
            "crc2 placement produced a non-zero residue"
        );

        self.packet_queue.push(
            Packet::new(0, TimeBase::new(1, self.sample_rate as i64), frame).with_pts(self.pts),
        );
        self.pts += SAMPLES_PER_FRAME as i64;
        Ok(())
    }
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
/// differences). Walks forward clamping both axes; this is the
/// "legalise" step §8.2.10 describes. Mutates `exp` in-place.
///
/// The clamping is conservative — when the raw exponent sequence
/// descends faster than D15 can represent, this routine floors it at
/// the minimum representable slope, which causes some high-frequency
/// coefficients to be reconstructed with a slightly larger-than-ideal
/// exponent. That costs SNR on those bins but keeps the stream legal
/// for every A/52 decoder.
fn preprocess_d15(exp: &mut [u8]) {
    if exp.is_empty() {
        return;
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
fn mantissa_bits_total(baps: &[Vec<[u8; N_COEFFS]>], end: usize) -> u32 {
    // Because groups for bap=1/2/4 are shared across channels in frequency
    // order within a block (spec §7.3.5), we walk channels within each
    // block and accumulate a running count of pending group slots.
    let nchan = baps.len();
    let blocks = baps[0].len();
    let mut total = 0u32;
    for blk in 0..blocks {
        let mut g1_left = 0u32;
        let mut g2_left = 0u32;
        let mut g4_left = 0u32;
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
        }
    }
    total
}

/// Adjust `csnroffst`/`fsnroffst` so the total mantissa bit count fits
/// the frame payload, minus a safety budget for syncinfo/BSI/side-info.
/// Uses a simple bisection on the 11-bit combined SNR offset range.
fn tune_snroffst(
    ba: &BitAllocParams,
    exps: &[Vec<[u8; N_COEFFS]>],
    end: usize,
    nchan: usize,
    fscod: u8,
    frame_bytes: usize,
) -> BitAllocParams {
    // Budget: total frame bits minus a conservative fixed-cost estimate
    // for everything besides mantissas. We model that overhead with
    // generous margin so later revisions can tighten it without risking
    // corruption.
    //
    //   syncinfo + BSI ≈ 40 + 28     = 68   bits
    //   per-block fixed overhead     ≈ 40  bits × 6 = 240
    //   block-0 exponents (D15)      : per channel (4 + 7*ngrps); for
    //     end=252 that's 4 + 7*83 = 585 bits × 2 chans = 1170
    //   crc2 = 16
    // Total overhead ~ 1494 bits; round to 1800 for safety.
    let overhead_bits: u32 = 1800 + 16 /* crc2 */;
    let total_bits = (frame_bytes * 8) as u32;
    if overhead_bits >= total_bits {
        return *ba;
    }
    let budget = total_bits - overhead_bits;

    // Evaluate `bits_used(csnr, fsnr)` — higher snroffst → more bits.
    let mut best = *ba;
    // Search order: try decreasing csnroffst steps of 1, then increase
    // fsnroffst up to 15 to squeeze a little extra.
    for csnr in (0..=15u8).rev() {
        for fsnr in 0..=15u8 {
            let mut cand = *ba;
            cand.csnroffst = csnr;
            cand.fsnroffst = fsnr;
            let mut baps: Vec<Vec<[u8; N_COEFFS]>> =
                vec![vec![[0u8; N_COEFFS]; exps[0].len()]; nchan];
            for ch in 0..nchan {
                for blk in 0..exps[ch].len() {
                    compute_bap(&exps[ch][blk], end, fscod, &cand, &mut baps[ch][blk]);
                }
            }
            let used = mantissa_bits_total(&baps, end);
            if used <= budget {
                best = cand;
            } else {
                break;
            }
        }
    }
    best
}

// ---------------------------------------------------------------------------
// Mantissa quantisation + packing
// ---------------------------------------------------------------------------

#[derive(Default)]
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
            format: SampleFormat::S16,
            channels: 2,
            sample_rate: sr,
            samples: nsamp as u32,
            pts: Some(0),
            time_base: TimeBase::new(1, sr as i64),
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
}
