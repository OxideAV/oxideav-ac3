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
use crate::syncinfo::{self, SyncInfo};

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
        requested_channels: params.channels,
    }))
}

struct Ac3Decoder {
    codec_id: CodecId,
    time_base: TimeBase,
    pending: Option<Packet>,
    eof: bool,
    state: Ac3State,
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
        Ok(())
    }
}

impl Ac3Decoder {
    fn process_frame(&mut self, pkt: &Packet) -> Result<Frame> {
        let data = &pkt.data[..];
        if data.len() < 5 {
            return Err(Error::invalid("ac3: packet too short for syncinfo"));
        }
        let si: SyncInfo = syncinfo::parse(data)?;
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
