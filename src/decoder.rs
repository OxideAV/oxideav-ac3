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
        let mut pcm = decoded.pcm_s16le;
        // Reorder bitstream-order multichannel layouts into WAV-mask
        // order for the indep substream. The indep `acmod`/`lfeon`
        // are surfaced on `DecodedFrame`; when there's a dep substream
        // the buffer's channel count exceeds `output_channels(acmod,
        // lfeon)` and `reorder_s16le_in_place` no-ops via its
        // channel-count guard. Round 6 leaves the dep-substream-extended
        // 7.1 case (indep 5.1 + dep [Lb,Rb]) in pure bitstream order;
        // routing the chanmap-tagged dep channels into the WAV 7.1
        // slots requires a bigger refactor and no fixture exercises it.
        wave_order::reorder_s16le_in_place(
            &mut pcm,
            decoded.acmod,
            decoded.lfeon,
            channels as usize,
        );
        // Apply downmix request only if the decoded program has more
        // channels than requested. Round 1 emits silence so the
        // downmix is a no-op anyway, but threading the request
        // through keeps round-2 work minimal.
        if let Some(req) = self.requested_channels {
            if req < channels {
                let bytes_per_sample = 2usize;
                let in_stride = channels as usize * bytes_per_sample;
                let out_stride = req as usize * bytes_per_sample;
                let n_frames = decoded.samples as usize;
                let mut out = vec![0u8; n_frames * out_stride];
                for i in 0..n_frames {
                    let src = &pcm[i * in_stride..i * in_stride + out_stride];
                    out[i * out_stride..i * out_stride + out_stride].copy_from_slice(src);
                }
                pcm = out;
            }
        }
        self.time_base = TimeBase::new(1, decoded.sample_rate as i64);
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
        //    source width).
        let dmx_mode = DownmixMode::resolve(self.requested_channels, bsi.nfchans);
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
}
