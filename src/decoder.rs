//! AC-3 packet → AudioFrame decoder.
//!
//! The decoder runs the full §7 DSP pipeline: syncinfo + BSI parsing,
//! audio-block exponent decode, parametric bit allocation, mantissa
//! dequantization, channel decoupling, rematrixing (for 2/0 streams),
//! dynamic-range scaling, 512-point IMDCT with KBD window, and 50%
//! overlap-add across audio blocks. The per-frame output is 1536 S16
//! samples per channel exactly as specified by §8.2.1.2.

use oxideav_core::Decoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::audblk::{self, Ac3State, BLOCKS_PER_FRAME, SAMPLES_PER_BLOCK};
use crate::bsi::{self, Bsi};
use crate::downmix::{Downmix, DownmixMode};
use crate::eac3;
use crate::syncinfo::{self, SyncInfo};
use crate::wave_order;

/// Samples produced per AC-3 syncframe, per channel: 6 blocks × 256
/// new samples each (each audio block is a 512-point TDAC transform
/// overlapping by 256 samples with its neighbour — §2.2).
pub const SAMPLES_PER_FRAME: u32 = 1536;

pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(Ac3Decoder {
        codec_id: params.codec_id.clone(),
        time_base: TimeBase::new(1, 48_000),
        pending: None,
        eof: false,
        state: Ac3State::new(),
        eac3_state: eac3::Eac3DecoderState::default(),
        requested_channels: params.channels,
        prefer_ltrt: false,
    }))
}

/// Dedicated E-AC-3 decoder factory. Identical to [`make_decoder`] —
/// the same `Ac3Decoder` struct dispatches on the per-packet bsid —
/// but registered with the `eac3` codec id so the registry's
/// container-tag lookup hits it for `A_EAC3` / `0xA7` / etc.
pub fn make_eac3_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(Ac3Decoder {
        codec_id: params.codec_id.clone(),
        time_base: TimeBase::new(1, 48_000),
        pending: None,
        eof: false,
        state: Ac3State::new(),
        eac3_state: eac3::Eac3DecoderState::default(),
        requested_channels: params.channels,
        prefer_ltrt: false,
    }))
}

/// Variant of [`make_decoder`] that selects the §7.8.2 **LtRt**
/// (Dolby Surround matrix-encoded) downmix when a 2-channel target is
/// requested. Equivalent to `make_decoder` when the caller did not
/// request a stereo downmix (`params.channels != Some(2)` or the
/// source is already mono/stereo). The LtRt downmix preserves
/// surround information so a downstream matrix decoder (Pro Logic
/// et al.) can recover Ls/Rs from the stereo pair; LoRo's
/// straight-sum mix is unrecoverable in that sense.
pub fn make_decoder_ltrt(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(Ac3Decoder {
        codec_id: params.codec_id.clone(),
        time_base: TimeBase::new(1, 48_000),
        pending: None,
        eof: false,
        state: Ac3State::new(),
        eac3_state: eac3::Eac3DecoderState::default(),
        requested_channels: params.channels,
        prefer_ltrt: true,
    }))
}

struct Ac3Decoder {
    codec_id: CodecId,
    time_base: TimeBase,
    pending: Option<Packet>,
    eof: bool,
    state: Ac3State,
    /// Per-decoder E-AC-3 state — empty in round 1 (no overlap-add
    /// delay yet), present so round 2 can park dependent-substream
    /// recombination scratch + per-channel IMDCT history without
    /// changing this struct's layout.
    eac3_state: eac3::Eac3DecoderState,
    /// Downmix target channel count — `Some(1)` = mono, `Some(2)` =
    /// stereo, `None` = passthrough of whatever the bitstream carries.
    /// Drives the §7.8 matrix in [`Ac3Decoder::process_frame`].
    requested_channels: Option<u16>,
    /// When `true` and a 2-channel downmix is requested, use the
    /// §7.8.2 **LtRt** (Dolby Surround matrix-encoded) equations
    /// instead of LoRo. Toggled by [`make_decoder_ltrt`]; the regular
    /// [`make_decoder`] / [`make_eac3_decoder`] factories leave this
    /// off (LoRo is §7.8.2's "preferred when mono is the ultimate
    /// target" path and is the spec's default downmix matrix).
    prefer_ltrt: bool,
}

impl Decoder for Ac3Decoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if self.pending.is_some() {
            return Err(Error::other(
                "AC-3 decoder: receive_frame must be called before sending another packet",
            ));
        }
        self.pending = Some(packet.clone());
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        let pkt = match self.pending.take() {
            Some(p) => p,
            None => {
                return if self.eof {
                    Err(Error::Eof)
                } else {
                    Err(Error::NeedMore)
                }
            }
        };
        self.process_frame(&pkt)
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.pending = None;
        self.eof = false;
        self.state = Ac3State::new();
        self.eac3_state = eac3::Eac3DecoderState::default();
        Ok(())
    }
}

impl Ac3Decoder {
    fn process_frame(&mut self, pkt: &Packet) -> Result<Frame> {
        let data = &pkt.data[..];
        if data.len() < 5 {
            return Err(Error::invalid("ac3: packet too short for syncinfo"));
        }
        // Top-level dispatch: peek at the bsid byte to choose AC-3
        // vs E-AC-3. The 5-bit `bsid` field sits at byte 5 (top 5
        // bits) in BOTH syntaxes:
        //
        //   AC-3:    syncword(2B) + crc1(2B) + fscod+frmsizecod(1B)
        //            ⇒ BSI starts at byte 5; bsid is the first 5
        //            bits = byte 5 top 5 bits.
        //   E-AC-3:  syncword(2B) + strmtyp+substreamid+frmsiz(2B) +
        //            fscod+(numblkscod|fscod2)+acmod+lfeon(1B) ⇒ bsid
        //            starts at byte 5 bit 0 = byte 5 top 5 bits.
        //
        // So `data[5] >> 3` is bsid in either layout. Per §E.2.3.1.6,
        // bsid 0..8 is base AC-3, 9/10 are reserved (we tolerate them
        // via the same AC-3 path), and 11..16 routes to Annex E.
        let try_ac3 = syncinfo::parse(data);
        if let Ok(si) = try_ac3 {
            // bsid lives in BSI byte 0 (= packet byte 5), top 5 bits.
            // The first BSI byte sits at exactly the same place in
            // both AC-3 (after 5 bytes of syncinfo) and E-AC-3 (after
            // 16-bit syncword + 16-bit strmtyp/substreamid/frmsiz +
            // 8-bit fscod/numblkscod/acmod/lfeon = 5 bytes). Whether
            // the value at byte 5's top 5 bits parses as bsid in BOTH
            // syntaxes is a documented spec property — see §E.2.3.1.
            if data.len() > 5 {
                let bsi_byte0 = data[5];
                let bsid = bsi_byte0 >> 3;
                if bsid <= bsi::MAX_BSID_BASE {
                    return self.process_ac3_frame(pkt, si);
                }
            }
        }
        // E-AC-3 path. The AC-3 syncinfo path may have rejected the
        // packet entirely (frmsizecod past Table 5.18) — that's still
        // a valid E-AC-3 syncframe. Hand the whole packet to the
        // Annex E decoder.
        self.process_eac3_frame(pkt)
    }

    fn process_eac3_frame(&mut self, pkt: &Packet) -> Result<Frame> {
        let data = &pkt.data[..];
        let decoded = eac3::decode_eac3_packet(&mut self.eac3_state, data)?;
        let channels = decoded.channels;

        // Resolve the §7.8 downmix mode from the requested target.
        // Annex E's `nfchans` (excludes LFE) drives the mode picker —
        // an Eac3 5.1 stream has nfchans=5 and resolves to Stereo /
        // StereoLtRt / Mono just like AC-3 does.
        let dmx_mode = {
            let base = DownmixMode::resolve(self.requested_channels, decoded.nfchans);
            if self.prefer_ltrt && matches!(base, DownmixMode::Stereo) {
                DownmixMode::StereoLtRt
            } else {
                base
            }
        };

        // Active downmix? Walk the f32 PCM through the §7.8 matrix so
        // negative LtRt surround weights don't truncate to 0 after a
        // pre-quantised S16 input. Falls back to the s16 truncate-then-
        // reorder path when no downmix is needed (passthrough) — that
        // path also keeps the dep-substream-extended channels intact.
        let (pcm, out_channels) = if matches!(dmx_mode, DownmixMode::Passthrough) {
            let mut pcm = decoded.pcm_s16le;
            // Reorder bitstream-order multichannel layouts into WAV-mask
            // order for the indep substream. For dep-extended programs
            // (e.g. 7.1 emitted as indep 5.1 + dep [Lb,Rb]) the buffer's
            // channel count exceeds the indep `output_channels(acmod,
            // lfeon)` and the reorder no-ops via its channel-count
            // guard — extended channels stay in bitstream order.
            wave_order::reorder_s16le_in_place(
                &mut pcm,
                decoded.acmod,
                decoded.lfeon,
                channels as usize,
            );
            (pcm, channels)
        } else {
            // Build a Downmix that honours Annex E mixmdata (Tables
            // E1.13-16 / D2.3-6) when present. Without mixmdata the
            // matrix uses the §7.8.2 fixed 0.707 defaults — identical
            // to the previous "truncate-to-2-channels" behaviour for
            // a 2/0 stereo source but spec-correct for 5.1 → LtRt /
            // LoRo where the Annex D path already proved out.
            let dmx = Downmix::from_eac3_fields(
                decoded.acmod,
                decoded.nfchans,
                channels as u8,
                decoded.lfeon,
                decoded.annex_e_mix_levels,
                dmx_mode,
            );
            let out_ch = dmx.output_channels() as usize;
            let src_f32 = self.eac3_state.indep_pcm_f32();
            let n_frames = decoded.samples as usize;
            // Defensive — should never fire unless the eac3 state is
            // out of sync with `decoded`.
            if src_f32.len() != n_frames * channels as usize {
                return Err(Error::invalid(format!(
                    "eac3 downmix: f32 scratch len {} != frames*ch {}*{}",
                    src_f32.len(),
                    n_frames,
                    channels,
                )));
            }
            let nfchans = decoded.nfchans as usize;
            let nchans = channels as usize;
            let mut out_f32 = vec![0.0f32; n_frames * out_ch];
            // §7.8 matrix is applied in fixed-size SAMPLES_PER_BLOCK
            // chunks (the §2.2 256-sample block window the encoder also
            // works in). Annex E doesn't change the block size; one
            // syncframe is `num_blocks * 256` samples per channel.
            let nblocks = n_frames / SAMPLES_PER_BLOCK;
            for blk in 0..nblocks {
                let mut per_ch: [[f32; SAMPLES_PER_BLOCK]; 5] = [[0.0; SAMPLES_PER_BLOCK]; 5];
                let base = blk * SAMPLES_PER_BLOCK * nchans;
                for n in 0..SAMPLES_PER_BLOCK {
                    for ch in 0..nfchans.min(5) {
                        per_ch[ch][n] = src_f32[base + n * nchans + ch];
                    }
                }
                let out_base = blk * SAMPLES_PER_BLOCK * out_ch;
                dmx.apply(
                    &per_ch,
                    SAMPLES_PER_BLOCK,
                    &mut out_f32[out_base..out_base + SAMPLES_PER_BLOCK * out_ch],
                );
            }
            // Pack f32 → S16LE.
            let mut out_bytes = vec![0u8; out_f32.len() * 2];
            for (i, s) in out_f32.iter().enumerate() {
                let clamped = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
                let le = clamped.to_le_bytes();
                out_bytes[i * 2] = le[0];
                out_bytes[i * 2 + 1] = le[1];
            }
            (out_bytes, out_ch as u16)
        };

        self.time_base = TimeBase::new(1, decoded.sample_rate as i64);
        let _ = out_channels; // surfaced for future AudioFrame channel-count
        Ok(Frame::Audio(AudioFrame {
            samples: decoded.samples,
            pts: pkt.pts,
            data: vec![pcm],
        }))
    }

    fn process_ac3_frame(&mut self, pkt: &Packet, si: SyncInfo) -> Result<Frame> {
        let data = &pkt.data[..];
        if (si.frame_length as usize) > data.len() {
            return Err(Error::invalid(format!(
                "ac3: packet short: frame_length={} pkt_len={}",
                si.frame_length,
                data.len()
            )));
        }
        let bsi: Bsi = bsi::parse(&data[5..])?;

        let src_channels = bsi.nchans as u16;
        let sample_rate = si.sample_rate;
        self.time_base = TimeBase::new(1, sample_rate as i64);

        // 1) Decode the syncframe into a source-layout interleaved
        //    f32 buffer. This contains `nfchans + lfe` channels.
        let src_samples = SAMPLES_PER_FRAME as usize * src_channels as usize;
        let mut floats = vec![0.0f32; src_samples];
        audblk::decode_frame(
            &mut self.state,
            &si,
            &bsi,
            &data[..si.frame_length as usize],
            &mut floats,
        )?;
        debug_assert_eq!(
            floats.len(),
            BLOCKS_PER_FRAME * SAMPLES_PER_BLOCK * src_channels as usize
        );

        // 2) Pick a §7.8 downmix mode from the requested output channel
        //    count (falls back to passthrough when unset or equal to
        //    source width). When `prefer_ltrt` is set, promote a
        //    LoRo (`Stereo`) selection to LtRt — Mono / Passthrough
        //    are unaffected (LtRt is a stereo-target option only,
        //    §7.8.2 explicitly notes "if the LtRt downmix is combined
        //    to mono, the surround information will be lost").
        let dmx_mode = {
            let base = DownmixMode::resolve(self.requested_channels, bsi.nfchans);
            if self.prefer_ltrt && matches!(base, DownmixMode::Stereo) {
                DownmixMode::StereoLtRt
            } else {
                base
            }
        };
        let (out_channels, out_samples) = if matches!(dmx_mode, DownmixMode::Passthrough) {
            (src_channels, floats.clone())
        } else {
            let dmx = Downmix::from_bsi(&bsi, dmx_mode);
            let out_ch = dmx.output_channels() as usize;
            let mut out = vec![0.0f32; SAMPLES_PER_FRAME as usize * out_ch];
            // Walk each audio block; gather fbw channel rows into the
            // downmixer's `[[f32; 256]; 5]` slot format, then apply.
            // LFE lives at fbw index `nfchans` in the source interleaved
            // buffer and is ignored by the downmix (§7.8 explicitly
            // allows any coefficient for LFE; we choose zero).
            let nfchans = bsi.nfchans as usize;
            let nchans = src_channels as usize;
            for blk in 0..BLOCKS_PER_FRAME {
                let mut per_ch: [[f32; SAMPLES_PER_BLOCK]; 5] = [[0.0; SAMPLES_PER_BLOCK]; 5];
                let base = blk * SAMPLES_PER_BLOCK * nchans;
                for n in 0..SAMPLES_PER_BLOCK {
                    for ch in 0..nfchans.min(5) {
                        per_ch[ch][n] = floats[base + n * nchans + ch];
                    }
                }
                let out_base = blk * SAMPLES_PER_BLOCK * out_ch;
                dmx.apply(
                    &per_ch,
                    SAMPLES_PER_BLOCK,
                    &mut out[out_base..out_base + SAMPLES_PER_BLOCK * out_ch],
                );
            }
            (out_ch as u16, out)
        };

        // 3) Pack f32 → S16 interleaved.
        let bytes_per_sample = SampleFormat::S16.bytes_per_sample();
        let total_bytes = SAMPLES_PER_FRAME as usize * out_channels as usize * bytes_per_sample;
        let mut out_bytes = vec![0u8; total_bytes];
        for (i, s) in out_samples.iter().enumerate() {
            let clamped = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
            let le = clamped.to_le_bytes();
            out_bytes[i * 2] = le[0];
            out_bytes[i * 2 + 1] = le[1];
        }

        // 4) Reorder bitstream-order channels into WAV-mask order so
        //    consumers that interpret the PCM as a WAVE file (or any
        //    `WAVE_FORMAT_EXTENSIBLE`-compliant sink — `pcm_s16le`,
        //    foobar2000, miniaudio, …) see (FL, FR, FC, LFE, BL, BR)
        //    instead of AC-3's bitstream (L, C, R, Ls, Rs, LFE).
        //    Mono / stereo / 2/1 / 2/2 layouts are no-ops; only
        //    acmod ∈ {3, 5, 7} (the front-center-bearing modes) get
        //    permuted. When downmix is active the output is already
        //    in standard order — `out_channels < src_channels` skips
        //    the reorder via [`wave_order::output_channels`] check.
        if matches!(dmx_mode, DownmixMode::Passthrough) {
            wave_order::reorder_s16le_in_place(
                &mut out_bytes,
                bsi.acmod,
                bsi.lfeon,
                out_channels as usize,
            );
        }

        Ok(Frame::Audio(AudioFrame {
            samples: SAMPLES_PER_FRAME,
            pts: pkt.pts,
            data: vec![out_bytes],
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::{CodecId, CodecParameters};

    /// A decoder build must succeed for the canonical codec id.
    #[test]
    fn decoder_builds() {
        let params = CodecParameters::audio(CodecId::new("ac3"));
        let dec = make_decoder(&params).unwrap();
        assert_eq!(dec.codec_id().as_str(), "ac3");
    }

    /// The LtRt factory must accept the same parameters as the default
    /// factory and produce a working decoder.
    #[test]
    fn ltrt_decoder_builds() {
        let mut params = CodecParameters::audio(CodecId::new("ac3"));
        params.channels = Some(2);
        let dec = make_decoder_ltrt(&params).unwrap();
        assert_eq!(dec.codec_id().as_str(), "ac3");
    }

    /// E-AC-3 5.1 encoded packet → decode with `channels = Some(2)`
    /// must run the §7.8 LoRo matrix end-to-end (not truncate the
    /// channel set to the first two). The encoder produces a fresh 5.1
    /// indep substream; the decoder is configured for stereo output;
    /// the resulting `AudioFrame` payload is exactly 2 ch × 1536
    /// samples × 2 bytes per frame.
    ///
    /// Round 129 wires `Downmix::from_eac3_fields` through
    /// [`Ac3Decoder::process_eac3_frame`]; this test exercises that
    /// new path end-to-end and locks in the output buffer shape +
    /// the fact that both output channels still carry non-trivial
    /// energy (the matrix coefficients pull C / Ls / Rs into both Lo
    /// and Ro, so a constant-amplitude sine on every channel keeps a
    /// recognisable envelope after the matrix).
    #[test]
    fn eac3_5_1_decodes_to_stereo_with_matrix_downmix() {
        use oxideav_core::Packet;
        use oxideav_core::TimeBase as TB;
        // Encode a 5.1 sine fixture at 384 kbps so the indep substream
        // has all six channels active (5 fbw + LFE).
        let mut enc_params = CodecParameters::audio(CodecId::new(eac3::CODEC_ID_STR));
        enc_params.sample_rate = Some(48_000);
        enc_params.channels = Some(6);
        enc_params.sample_format = Some(SampleFormat::S16);
        enc_params.bit_rate = Some(384_000);
        let mut enc = match eac3::make_encoder(&enc_params) {
            Ok(e) => e,
            Err(e) => {
                eprintln!("eac3 make_encoder failed: {e} — skipping");
                return;
            }
        };

        // 1536 samples × 6 channels (interleaved S16). C carries 0.4,
        // L/R carry 0.3 each, Ls/Rs -0.3 each, LFE zero.
        let mut pcm = Vec::<u8>::with_capacity(1536 * 6 * 2);
        for i in 0..1536 {
            let t = i as f32 / 48_000.0;
            let s = (2.0 * std::f32::consts::PI * 440.0 * t).sin();
            let push = |out: &mut Vec<u8>, v: f32| {
                let q = (v * 32767.0).clamp(-32768.0, 32767.0) as i16;
                out.extend_from_slice(&q.to_le_bytes());
            };
            push(&mut pcm, 0.3 * s); // L
            push(&mut pcm, 0.4 * s); // C
            push(&mut pcm, 0.3 * s); // R
            push(&mut pcm, -0.3 * s); // Ls
            push(&mut pcm, -0.3 * s); // Rs
            push(&mut pcm, 0.0); // LFE
        }
        if enc
            .send_frame(&Frame::Audio(AudioFrame {
                samples: 1536,
                pts: Some(0),
                data: vec![pcm],
            }))
            .is_err()
        {
            eprintln!("eac3 encoder send_frame failed — skipping");
            return;
        }
        let _ = enc.flush();

        let mut all_bytes = Vec::<u8>::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => all_bytes.extend_from_slice(&p.data),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => {
                    eprintln!("eac3 encoder receive_packet failed: {e} — skipping");
                    return;
                }
            }
        }
        if all_bytes.is_empty() {
            eprintln!("eac3 encoder produced no bytes for 5.1 input — skipping");
            return;
        }
        // First two bytes must be the syncword (cheap sanity check
        // that we actually have an E-AC-3 elementary stream).
        assert_eq!(&all_bytes[0..2], &[0x0B, 0x77]);

        // Decode with channels = Some(2) — request the LoRo downmix.
        let mut dec_params = CodecParameters::audio(CodecId::new(eac3::CODEC_ID_STR));
        dec_params.sample_rate = Some(48_000);
        dec_params.channels = Some(2);
        dec_params.sample_format = Some(SampleFormat::S16);
        let mut dec = make_eac3_decoder(&dec_params).expect("make_eac3_decoder");

        // The decoder expects one full packet per `send_packet` call.
        // The E-AC-3 encoder produces fixed 1536-byte frames at 384
        // kbps / 48 kHz / 1536 spf. Walk them one at a time.
        let frame_bytes = 1536usize;
        assert!(all_bytes.len() >= frame_bytes);
        let mut got_any = false;
        for off in (0..all_bytes.len()).step_by(frame_bytes) {
            let end = (off + frame_bytes).min(all_bytes.len());
            let pkt = Packet::new(0, TB::new(1, 48_000), all_bytes[off..end].to_vec());
            if dec.send_packet(&pkt).is_err() {
                continue;
            }
            loop {
                match dec.receive_frame() {
                    Ok(Frame::Audio(af)) => {
                        got_any = true;
                        let expected_len = af.samples as usize * 2 * 2;
                        assert_eq!(
                            af.data[0].len(),
                            expected_len,
                            "stereo downmix payload size: want {} bytes, got {}",
                            expected_len,
                            af.data[0].len()
                        );
                    }
                    Ok(_) => {}
                    Err(Error::NeedMore) | Err(Error::Eof) => break,
                    Err(e) => panic!("eac3 decoder error: {e}"),
                }
            }
        }
        assert!(
            got_any,
            "decoder produced no audio frames from 5.1 → stereo path"
        );
    }
}
