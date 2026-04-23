//! AC-3 packet → AudioFrame decoder.
//!
//! ## Status
//!
//! This is the skeleton. It *correctly* parses `syncinfo` and `bsi`
//! out of each incoming packet (so stream inspection / muxing
//! round-trip works), but the DSP pipeline — exponent decode, bit
//! allocation, mantissa dequant, IMDCT, window/overlap-add, downmix
//! — is still TODO. Until those land the decoder emits a silent
//! audio frame of the correct shape (256 samples per block × 6
//! blocks = 1536 samples per channel) so upstream code can be
//! exercised end-to-end.
//!
//! The internal layout is deliberately structured so that each DSP
//! step drops into `process_frame` without rewriting packet glue.

use oxideav_codec::Decoder;
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::bsi::{self, Bsi};
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
    }))
}

struct Ac3Decoder {
    codec_id: CodecId,
    time_base: TimeBase,
    pending: Option<Packet>,
    eof: bool,
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

        // Skeleton: emit a silent frame of the correct shape.
        let channels = bsi.nchans as u16;
        let sample_rate = si.sample_rate;
        self.time_base = TimeBase::new(1, sample_rate as i64);
        let bytes_per_sample = SampleFormat::S16.bytes_per_sample();
        let total_bytes =
            SAMPLES_PER_FRAME as usize * channels as usize * bytes_per_sample;
        let out_bytes = vec![0u8; total_bytes];

        Ok(Frame::Audio(AudioFrame {
            format: SampleFormat::S16,
            channels,
            sample_rate,
            samples: SAMPLES_PER_FRAME,
            pts: pkt.pts,
            time_base: self.time_base,
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
