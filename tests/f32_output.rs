//! Opt-in F32 output (`CodecParameters::sample_format = Some(F32)`)
//! across every decoder output path: AC-3 and E-AC-3, multichannel
//! passthrough (bitstream → WAVE reorder) and the §7.8 stereo downmix.
//!
//! The streams come from the crate's own encoders so no fixture is
//! needed; the property under test is the contract the F32 option
//! makes with the default output: the S16 output is exactly the F32
//! output quantised as the S16 path does, frame for frame, channel for
//! channel, so a consumer that switches format gets the same decode
//! with the truncation removed.

use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, SampleFormat, TimeBase,
};

const RATE: u32 = 48_000;

/// Encode a one-second multitone through the in-tree AC-3 (`eac3 =
/// false`) or E-AC-3 encoder and return the elementary stream as
/// syncframe packets.
fn encode(eac3: bool, channels: u16, bit_rate: u64) -> Vec<Vec<u8>> {
    let codec = if eac3 {
        oxideav_ac3::CODEC_ID_STR_EAC3
    } else {
        oxideav_ac3::CODEC_ID_STR
    };
    let mut params = CodecParameters::audio(CodecId::new(codec));
    params.sample_rate = Some(RATE);
    params.channels = Some(channels);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(bit_rate);
    let mut enc = if eac3 {
        oxideav_ac3::eac3::make_encoder(&params).expect("eac3 make_encoder")
    } else {
        oxideav_ac3::encoder::make_encoder(&params).expect("ac3 make_encoder")
    };
    let n = RATE as usize;
    let mut s16 = Vec::with_capacity(n * channels as usize * 2);
    for i in 0..n {
        let t = i as f32 / RATE as f32;
        for ch in 0..channels {
            let f0 = 330.0 * (1.0 + 0.07 * ch as f32);
            let v = 0.3 * (2.0 * std::f32::consts::PI * f0 * t).sin()
                + 0.1 * (2.0 * std::f32::consts::PI * (3100.0 + 150.0 * ch as f32) * t).sin();
            let q = (v * 32767.0).clamp(-32768.0, 32767.0) as i16;
            s16.extend_from_slice(&q.to_le_bytes());
        }
    }
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![s16],
    }))
    .expect("send_frame");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(pkt) => out.push(pkt.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("receive_packet: {e:?}"),
        }
    }
    assert!(out.len() > 4, "encoder emitted {} syncframes", out.len());
    out
}

/// Decode `packets` with the given output format and requested channel
/// count; returns every frame's payload.
fn decode(packets: &[Vec<u8>], format: SampleFormat, channels: Option<u16>) -> Vec<Vec<u8>> {
    let mut params = CodecParameters::audio(CodecId::new(oxideav_ac3::CODEC_ID_STR));
    params.sample_rate = Some(RATE);
    params.channels = channels;
    params.sample_format = Some(format);
    let mut dec = oxideav_ac3::decoder::make_decoder(&params).expect("make_decoder");
    let mut frames = Vec::new();
    for pkt in packets {
        dec.send_packet(&Packet::new(0, TimeBase::new(1, RATE as i64), pkt.clone()))
            .expect("send_packet");
        while let Ok(Frame::Audio(af)) = dec.receive_frame() {
            frames.push(af.data[0].clone());
        }
    }
    assert_eq!(frames.len(), packets.len(), "one frame per syncframe");
    frames
}

/// The S16 decode must be the F32 decode quantised; `out_channels` pins
/// the payload width of both.
fn assert_f32_is_unquantised_s16(packets: &[Vec<u8>], requested: Option<u16>, out_channels: usize) {
    let s16 = decode(packets, SampleFormat::S16, requested);
    let f32 = decode(packets, SampleFormat::F32, requested);
    assert_eq!(s16.len(), f32.len());
    let mut fractional = 0usize;
    let mut nonzero = 0usize;
    for (a, b) in s16.iter().zip(&f32) {
        assert_eq!(a.len(), 1536 * out_channels * 2, "S16 payload width");
        assert_eq!(b.len(), 1536 * out_channels * 4, "F32 payload width");
        for (q, f) in a.as_chunks::<2>().0.iter().zip(b.as_chunks::<4>().0) {
            let v = f32::from_le_bytes(*f);
            let expect = (v * 32767.0).clamp(-32768.0, 32767.0) as i16;
            assert_eq!(i16::from_le_bytes(*q), expect);
            if (v * 32767.0).fract() != 0.0 {
                fractional += 1;
            }
            if v != 0.0 {
                nonzero += 1;
            }
        }
    }
    assert!(nonzero > 0, "decoded audio is not silence");
    assert!(fractional > 0, "F32 keeps the precision S16 drops");
}

#[test]
fn ac3_stereo_passthrough() {
    let es = encode(false, 2, 192_000);
    assert_f32_is_unquantised_s16(&es, None, 2);
}

#[test]
fn ac3_51_passthrough_reorders_like_s16() {
    let es = encode(false, 6, 448_000);
    assert_f32_is_unquantised_s16(&es, None, 6);
}

#[test]
fn ac3_51_stereo_downmix() {
    let es = encode(false, 6, 448_000);
    assert_f32_is_unquantised_s16(&es, Some(2), 2);
}

#[test]
fn eac3_stereo_passthrough() {
    let es = encode(true, 2, 192_000);
    assert_f32_is_unquantised_s16(&es, None, 2);
}

#[test]
fn eac3_51_passthrough_reorders_like_s16() {
    let es = encode(true, 6, 384_000);
    assert_f32_is_unquantised_s16(&es, None, 6);
}

#[test]
fn eac3_51_stereo_downmix() {
    let es = encode(true, 6, 384_000);
    assert_f32_is_unquantised_s16(&es, Some(2), 2);
}

#[test]
fn eac3_51_mono_downmix() {
    let es = encode(true, 6, 384_000);
    assert_f32_is_unquantised_s16(&es, Some(1), 1);
}
