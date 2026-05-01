//! Enhanced AC-3 (E-AC-3 / Dolby Digital Plus) encoder per ATSC A/52
//! Annex E. Round-1 scope: a single independent substream
//! (`strmtyp=0`, `substreamid=0`), `bsid=16`, 6 audio blocks per
//! syncframe (`numblkscod=3`), 1.0/2.0 layouts only, no coupling, no
//! spectral extension, no Adaptive Hybrid Transform.
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

/// `bsid` value for E-AC-3 streams compliant with ATSC A/52 Annex E
/// (§E.2.3.1.6: bsid = 16 = '10000').
pub const EAC3_BSID: u8 = 16;

/// Build an E-AC-3 encoder (round-1 scope).
///
/// Required parameters:
/// * `sample_rate` — 48 000 / 44 100 / 32 000 Hz
/// * `channels`    — 1 (mono / acmod=1) or 2 (stereo / acmod=2)
///
/// Optional `bit_rate` (selects the syncframe size). Defaults: 96 kbps
/// for mono, 192 kbps for stereo.
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    let sample_rate = params.sample_rate.ok_or_else(|| {
        Error::invalid("eac3 encoder: sample_rate is required (48000/44100/32000)")
    })?;
    let channels = params
        .channels
        .ok_or_else(|| Error::invalid("eac3 encoder: channels is required (1 or 2)"))?;
    let (acmod, nfchans) = match channels {
        1 => (1u8, 1usize),
        2 => (2u8, 2usize),
        n => {
            return Err(Error::Unsupported(format!(
                "eac3 encoder: round-1 scope is mono/stereo only, got {n} channels"
            )))
        }
    };
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
    let default_kbps: u32 = match channels {
        1 => 96,
        2 => 192,
        _ => 192,
    };
    let target_kbps: u32 = params
        .bit_rate
        .map(|b| (b / 1000) as u32)
        .unwrap_or(default_kbps);
    // Frame size in bytes. AC-3 picks frame size from Table 5.18 so the
    // bit-rate is exact across frames at 48 kHz; we mirror that lookup
    // (E-AC-3 frame size is byte-granular, AC-3 is word-granular —
    // every AC-3 size happens to be even so we can reuse the table).
    let frame_bytes = ac3_frame_bytes(fscod, target_kbps).ok_or_else(|| {
        Error::Unsupported(format!(
            "eac3 encoder: bit rate {target_kbps} kbps has no frame-size mapping"
        ))
    })? as usize;

    let out_params = {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID_STR));
        p.sample_rate = Some(sample_rate);
        p.channels = Some(channels);
        p.sample_format = Some(SampleFormat::S16);
        p.bit_rate = Some(target_kbps as u64 * 1000);
        p
    };

    let input_sample_format = params.sample_format.unwrap_or(SampleFormat::S16);
    Ok(Box::new(Eac3Encoder {
        codec_id: CodecId::new(CODEC_ID_STR),
        out_params,
        sample_rate,
        channels: nfchans,
        acmod,
        input_sample_format,
        fscod,
        frame_bytes,
        delay_line: vec![vec![0.0f32; SAMPLES_PER_BLOCK]; nfchans],
        pending_samples: vec![Vec::<f32>::new(); nfchans],
        transient_state: (0..nfchans).map(|_| TransientDetector::default()).collect(),
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

struct Eac3Encoder {
    codec_id: CodecId,
    out_params: CodecParameters,
    sample_rate: u32,
    /// Number of full-bandwidth channels (1 or 2 in round-1 scope).
    channels: usize,
    /// AC-3 audio coding mode (Table 5.8). One of {1, 2}.
    acmod: u8,
    input_sample_format: SampleFormat,
    fscod: u8,
    frame_bytes: usize,
    delay_line: Vec<Vec<f32>>,
    pending_samples: Vec<Vec<f32>>,
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
        let per_chan = decode_input_samples(audio, self.channels, self.input_sample_format)?;
        for ch in 0..self.channels {
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
            for ch in 0..self.channels {
                self.pending_samples[ch].extend(std::iter::repeat(0.0).take(missing));
            }
        }
        self.emit_syncframe()?;
        Ok(())
    }
}

impl Eac3Encoder {
    fn emit_syncframe(&mut self) -> Result<()> {
        let n_per = SAMPLES_PER_FRAME as usize;
        // -------- DSP: window + MDCT per channel per block --------
        let mut coeffs: Vec<Vec<[f32; N_COEFFS]>> =
            vec![vec![[0.0; N_COEFFS]; BLOCKS_PER_FRAME]; self.channels];
        let mut blksw: Vec<[bool; BLOCKS_PER_FRAME]> =
            vec![[false; BLOCKS_PER_FRAME]; self.channels];
        for ch in 0..self.channels {
            let drain: Vec<f32> = self.pending_samples[ch].drain(0..n_per).collect();
            for blk in 0..BLOCKS_PER_FRAME {
                let mut in_buf = [0.0f32; 512];
                in_buf[..256].copy_from_slice(&self.delay_line[ch]);
                in_buf[256..].copy_from_slice(
                    &drain[blk * SAMPLES_PER_BLOCK..(blk + 1) * SAMPLES_PER_BLOCK],
                );
                let is_short = if std::env::var("EAC3_DISABLE_BLKSW").is_ok() {
                    false
                } else {
                    self.transient_state[ch].process(&in_buf[256..])
                };
                blksw[ch][blk] = is_short;
                let mut win_buf = [0.0f32; 512];
                for n in 0..256 {
                    win_buf[n] = in_buf[n] * WINDOW[n];
                    win_buf[511 - n] = in_buf[511 - n] * WINDOW[n];
                }
                self.delay_line[ch].copy_from_slice(
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
        // Round-1 layout matches AC-3 sans coupling/LFE: just per-channel.
        // The shared `compute_bap` / `tune_snroffst` helpers expect a
        // `channels + 2` layout (slots [nfchans] = cpl, [nfchans+1] = LFE)
        // even when neither is in use — we allocate empty slots to satisfy
        // their bounds checks.
        let cpl_idx_in_exps = self.channels;
        let lfe_idx_in_exps = self.channels + 1;
        let _ = (cpl_idx_in_exps, lfe_idx_in_exps);
        let mut exps: Vec<Vec<[u8; N_COEFFS]>> =
            vec![vec![[24u8; N_COEFFS]; BLOCKS_PER_FRAME]; self.channels + 2];
        let chbwcod: u8 = 60;
        let end_mant: usize = 37 + 3 * (chbwcod as usize + 12);
        let ch_end_mant = end_mant;
        for ch in 0..self.channels {
            for blk in 0..BLOCKS_PER_FRAME {
                for k in 0..ch_end_mant {
                    exps[ch][blk][k] = extract_exponent(coeffs[ch][blk][k]);
                }
            }
        }
        // Exponent strategy: D15 on blocks 0 and 3, REUSE elsewhere.
        let exp_strategies: [u8; BLOCKS_PER_FRAME] = [1, 0, 0, 1, 0, 0];
        for ch in 0..self.channels {
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
        // No coupling, no LFE — the helpers still want a CouplingPlan
        // and a DbaPlan to thread through their shared bookkeeping.
        let cpl = CouplingPlan::default();
        let dba_plan = if std::env::var("EAC3_DISABLE_DBA").is_ok() {
            crate::encoder::DbaPlan::default()
        } else {
            build_dba_plan(&exps, self.channels, ch_end_mant, &cpl)
        };
        let tuned_ba = tune_snroffst(
            &ba,
            &exps,
            ch_end_mant,
            self.channels,
            self.fscod,
            self.frame_bytes,
            &exp_strategies,
            &cpl,
            &dba_plan,
            self.acmod,
            false, /* lfeon */
        );
        // For E-AC-3 with snroffststr=0 the frame-level frmfsnroffst is
        // applied to every fbw channel (and to cpl/lfe). Force every
        // channel's compute_bap to use the *base* fsnroffst rather than
        // the per-channel fsnroffst_ch[ch] that the AC-3 path uses, so
        // encoder and decoder agree on the bap[] arrays. Otherwise the
        // mantissa quantiser overshoots/undershoots the bit budget and
        // ffmpeg (correctly) reports per-block exponent / bap mismatches.
        let mut baps: Vec<Vec<[u8; N_COEFFS]>> =
            vec![vec![[0u8; N_COEFFS]; BLOCKS_PER_FRAME]; self.channels + 2];
        let mut frame_ba = tuned_ba;
        frame_ba.fsnroffst = tuned_ba.fsnroffst; // base value reused for all
        for ch in 0..self.channels {
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

        // -------- Pack syncframe --------
        let mut bw = BitWriter::with_capacity(self.frame_bytes);

        // syncinfo (§E.2.2.1) — just the 16-bit syncword.
        bw.write_u32(0x0B77, 16);

        // bsi (§E.2.2.2) — round-1: independent substream id 0.
        bw.write_u32(0, 2); // strmtyp = 0 (Type 0 = independent)
        bw.write_u32(0, 3); // substreamid = 0
                            // frmsiz = (frame_size_in_words - 1) per §E.2.3.1.3.
        let frmsiz = (self.frame_bytes / 2 - 1) as u32;
        bw.write_u32(frmsiz, 11);
        bw.write_u32(self.fscod as u32, 2);
        // fscod != 0x3, so emit numblkscod (2 bits). 0x3 = 6 blocks.
        bw.write_u32(0x3, 2); // numblkscod
        bw.write_u32(self.acmod as u32, 3);
        bw.write_u32(0, 1); // lfeon (round-1: no LFE)
        bw.write_u32(EAC3_BSID as u32, 5); // bsid = 16
        bw.write_u32(27, 5); // dialnorm = -27 dB
        bw.write_u32(0, 1); // compre = 0
                            // acmod != 0 → no dialnorm2/compr2e
                            // strmtyp == 0 (independent), not Type 1 → no chanmape
        bw.write_u32(0, 1); // mixmdate = 0 (no mixing metadata)
        bw.write_u32(0, 1); // infomdate = 0 (no informational metadata)
                            // strmtyp == 0 && numblkscod != 0x3 ? convsync. Here
                            // numblkscod == 0x3, so the convsync field is omitted.
                            // strmtyp != 0x2, so no blkid/frmsizecod.
        bw.write_u32(0, 1); // addbsie = 0

        // audfrm (§E.2.2.3 / §E.2.3.2). numblkscod == 0x3 ⇒ expstre + ahte
        // are present; otherwise they are implied to {1, 0}.
        bw.write_u32(1, 1); // expstre = 1 — per-block strategy
        bw.write_u32(0, 1); // ahte = 0 — no Adaptive Hybrid Transform
        bw.write_u32(0, 2); // snroffststr = 0 — frame-level snroffst
        bw.write_u32(0, 1); // transproce = 0 — no TPNP
        bw.write_u32(1, 1); // blkswe = 1 — per-block blksw[ch] in audblk
        bw.write_u32(1, 1); // dithflage = 1 — per-block dithflag in audblk
        bw.write_u32(1, 1); // bamode = 1 — per-block bit-allocation params
        bw.write_u32(1, 1); // frmfgaincode = 1 — per-block fgaincod
        bw.write_u32(1, 1); // dbaflde = 1 — delta-bit-allocation in audblk
        bw.write_u32(1, 1); // skipflde = 1 — skip fields in audblk
        bw.write_u32(0, 1); // spxattene = 0 — no SPX attenuation
                            // acmod > 1 ⇒ cplstre[0] = 1 implied; emit cplinu[0] then per-block
                            // cplstre[blk] / cplinu[blk]. Round-1: cplinu = 0 for every block.
        if self.acmod > 1 {
            // cplinu[0] = 0
            bw.write_u32(0, 1);
            // For blocks 1..5: cplstre[blk] (1 bit), and only when
            // cplstre[blk]==1 do we emit cplinu[blk]. Since cpl is off
            // for the whole frame, cplstre[blk]=0 for blocks 1..5
            // (reuse from prior block, which is 0).
            for _blk in 1..BLOCKS_PER_FRAME {
                bw.write_u32(0, 1); // cplstre[blk] = 0
            }
        }
        // expstre==1 ⇒ per-block exponent-strategy data is signalled
        // here (NOT in the audblk). For each block, emit cplexpstr
        // (only when cplinu[blk]==1; we keep cpl off so omit), then
        // per-channel chexpstr[blk][ch] at 2 bits each.
        for blk in 0..BLOCKS_PER_FRAME {
            // cplinu[blk] = 0 — no cplexpstr.
            for _ch in 0..self.channels {
                bw.write_u32(exp_strategies[blk] as u32, 2);
            }
        }
        // expstre==1 + lfeon=0 → no lfeexpstr emission here either.
        // strmtyp == 0x0 ⇒ convexpstre. With numblkscod==0x3, convexpstre==1
        // implied (see syntax: "if (numblkscod != 0x3) {convexpstre} else {convexpstre = 1}").
        // Then per-channel convexpstr (5 bits each).
        for _ in 0..self.channels {
            bw.write_u32(0, 5); // convexpstr = 0 (REUSE) — no AC-3 conversion
        }
        // ahte == 0 ⇒ skip AHT block entirely.
        // snroffststr == 0 ⇒ frame-level snr offsets.
        bw.write_u32(tuned_ba.csnroffst as u32, 6); // frmcsnroffst
        bw.write_u32(tuned_ba.fsnroffst as u32, 4); // frmfsnroffst
                                                    // transproce == 0 ⇒ no per-channel transproc.
                                                    // spxattene == 0 ⇒ no per-channel spxatten.
                                                    // blkstrtinfoe — only when numblkscod != 0. Here numblkscod==3,
                                                    // so the conditional `if (numblkscod != 0)` is true and we emit
                                                    // a 1-bit blkstrtinfoe; we set it to 0 (no block-start info).
        bw.write_u32(0, 1); // blkstrtinfoe

        // -------- audio blocks --------
        for blk in 0..BLOCKS_PER_FRAME {
            // blkswe == 1 ⇒ per-channel blksw[ch].
            for ch in 0..self.channels {
                bw.write_u32(blksw[ch][blk] as u32, 1);
            }
            // dithflage == 1 ⇒ per-channel dithflag (set 1 = enable).
            for _ in 0..self.channels {
                bw.write_u32(1, 1);
            }
            // dynrnge / dynrng. Round-1: no dynrng signalled.
            bw.write_u32(0, 1); // dynrnge = 0
                                // acmod==0 (1+1) would emit dynrng2e, but our acmods are 1/2.

            // SPX strategy: spxstre is implicit on blk==0 (=1); else 1 bit.
            // We always set spxinu=0 → spxstre transmitted as 1 with
            // spxinu=0 on block 0, then `0` for subsequent blocks.
            if blk == 0 {
                // spxstre implicit = 1 — emit spxinu=0 directly.
                bw.write_u32(0, 1); // spxinu = 0
            } else {
                bw.write_u32(0, 1); // spxstre = 0 (reuse — keeps spxinu=0)
            }

            // Coupling strategy / coupling info. cplstre[0] was emitted in
            // audfrm; cplinu[blk] is reused at 0. Since cplinu==0 for the
            // whole frame, we skip the cplstre body. But cplstre[blk] for
            // blk>=1 was already emitted in audfrm — for blk==0 here we
            // are inside `if(cplstre[blk])` of the audblk syntax.
            //
            // The audblk grammar (§E.2.2.4):
            //   if (cplstre[blk]) {
            //       if (cplinu[blk]) { ... ecplinu, chincpl, ... }
            //       else { ... }
            //   }
            // For block 0: cplstre[0] = 1 implied (because audfrm emits
            // cplinu[0]) — so we ARE inside the body; cplinu[0] = 0
            // means we hit the `else` branch which has no syntax (just
            // sets state vars). Nothing to emit here.
            // For blk>=1: cplstre[blk] = 0 (we wrote it in audfrm), so
            // the body is skipped entirely. Nothing to emit.

            // Coupling coordinates — only when cplinu[blk]. Skipped (=0).

            // Rematrixing (Table 5.15 / §E.2 inherits §5.4.3.19 form).
            // Only acmod==2. With no coupling and no SPX, nrematbd=4.
            // Round-1 disables rematrix — emit `rematstr=1` (implicit on
            // blk==0; explicit elsewhere) followed by nrematbd zero
            // flags so the alignment is correct. Block 0's rematstr is
            // implicitly 1 per the spec grammar; subsequent blocks
            // emit `rematstr=1` so we re-transmit the all-zero flags
            // (cheap; alternative is rematstr=0 = "reuse from prior").
            if self.acmod == 2 {
                let nrematbd = 4usize; // no cpl, no spx
                if blk == 0 {
                    // rematstr implicit = 1; emit rematflg[bnd] zeros.
                    for _bnd in 0..nrematbd {
                        bw.write_u32(0, 1);
                    }
                } else {
                    // rematstr = 0 → reuse prior block's flags (which
                    // are all zero — disabled — from block 0).
                    bw.write_u32(0, 1);
                }
            }

            // chbwcod for channels not coupled / SPX'd, when chexpstr != reuse.
            let exp_strategy = exp_strategies[blk];
            if exp_strategy != 0 {
                for _ in 0..self.channels {
                    bw.write_u32(chbwcod as u32, 6);
                }
            }
            // Exponents per channel (D15, no coupling, no LFE).
            if exp_strategy == 1 {
                for ch in 0..self.channels {
                    write_exponents_d15(&mut bw, &exps[ch][blk], ch_end_mant);
                    bw.write_u32(0, 2); // gainrng = 0
                }
            }

            // Bit-allocation params. bamode=1 → emit baie + (when set)
            // sdcycod/.../floorcod. We send a fresh set on block 0; reuse
            // afterwards.
            let baie = blk == 0;
            bw.write_u32(baie as u32, 1);
            if baie {
                bw.write_u32(tuned_ba.sdcycod as u32, 2);
                bw.write_u32(tuned_ba.fdcycod as u32, 2);
                bw.write_u32(tuned_ba.sgaincod as u32, 2);
                bw.write_u32(tuned_ba.dbpbcod as u32, 2);
                bw.write_u32(tuned_ba.floorcod as u32, 3);
            }

            // snroffst — snroffststr=0 ⇒ values come from the frame-level
            // (frmcsnroffst/frmfsnroffst) and there is NO per-block
            // snroffste/csnroffst/fsnroffst emission in the audblk.

            // fgaincode (frmfgaincode=1 → per-block fgaincode flag).
            // value=0 ⇒ all fbw channels use the default fgaincod=4.
            bw.write_u32(0, 1); // fgaincode = 0

            // strmtyp==0 ⇒ convsnroffste. 0 ⇒ no convsnroffst.
            bw.write_u32(0, 1); // convsnroffste = 0

            // cplleake — only when cplinu[blk]. Skipped.

            // Delta bit allocation. dbaflde=1 → deltbaie. Round-1: emit
            // deltbaie=1 on block 0 with one segment per channel
            // (mirrors AC-3 path), deltbaie=0 elsewhere.
            let any_fbw_dba = (0..self.channels).any(|c| dba_plan.nseg[c] > 0);
            if blk == 0 && any_fbw_dba {
                bw.write_u32(1, 1); // deltbaie = 1
                for ch in 0..self.channels {
                    let code = if dba_plan.nseg[ch] > 0 { 1 } else { 2 };
                    bw.write_u32(code as u32, 2); // deltbae[ch]
                }
                for ch in 0..self.channels {
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

            // skipflde=1 → skiple flag, value=0 ⇒ no skip data.
            bw.write_u32(0, 1); // skiple = 0

            // -------- Mantissas --------
            // Same per-channel order as AC-3 (no coupling, no LFE).
            let mut codes: Vec<(u8, u32)> = Vec::with_capacity(self.channels * ch_end_mant);
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
            }
            // Suppress LFE — we have no LFE in round-1 scope.
            let _ = LFE_END_MANT;
            write_mantissa_stream(&mut bw, &codes);
        }

        // auxdata: pad up to (frame_bytes - errorcheck_bytes) with zeros.
        // errorcheck() = encinfo(1) + crc2(16) = 17 bits.
        let target_bits = (self.frame_bytes * 8) as u64;
        let used_bits = bw.bit_position();
        let errorcheck_bits = 17u64;
        if used_bits + errorcheck_bits > target_bits {
            return Err(Error::other(format!(
                "eac3 encoder: bit budget overflow ({} bits used, frame {} bits)",
                used_bits, target_bits
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
        // errorcheck: encinfo (1 bit reserved) + crc2 placeholder (16 bits).
        bw.write_u32(0, 1); // encinfo = 0
        bw.write_u32(0, 16); // crc2 placeholder
        let mut frame = bw.into_bytes();
        debug_assert_eq!(frame.len(), self.frame_bytes);

        // crc2 covers everything from byte 2 onward (skip the 16-bit
        // syncword) up through the byte before the crc2 field. The CRC
        // value emitted is the running CRC over that data; the same
        // polynomial / initial value as AC-3 (§6.1.7) per Annex E
        // (E-AC-3 reuses §6.1.7 — Annex E doesn't redefine CRC).
        let crc2_val = ac3_crc_update(0, &frame[2..(self.frame_bytes - 2)]);
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
        p.channels = Some(6);
        match make_encoder(&p) {
            Ok(_) => panic!("must reject 6ch in round-1"),
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
}
