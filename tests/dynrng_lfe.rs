//! §7.7.1 dynamic range control reaches the LFE channel.
//!
//! §5.4.3.4 says the `dynrng` word "is applied to scale the reproduced
//! audio" and §6.1.9 has the decoder alter the block's coefficients by
//! it; nothing exempts the LFE channel (only 1+1 mode splits the word
//! per channel). The decoders used to scale the fbw channels only, so a
//! 5.1 stream carrying `dynrng` played its LFE up to 24 dB off the rest
//! of the program (found against a black-box decoder on the r480
//! broadcast-shape matrix: every channel ≥ 89 dB, LFE 17 dB).
//!
//! Both encoders write a constant `dynrng` word on request, so the
//! stream pair (unity word / −9.5 dB word) differs only in that word;
//! the decoded LFE must move by the same linear gain as the fbw
//! channels, which `drc::dynrng_to_linear` resolves from Table 7.29.

use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, SampleFormat, TimeBase,
};

const RATE: u32 = 48_000;
const CHANNELS: usize = 6;

/// Encode a one-second 5.1 multitone (LFE = 50 Hz) with the given
/// `dynrng` option through the AC-3 (`eac3 = false`) or E-AC-3 encoder.
fn encode(eac3: bool, dynrng: &str) -> Vec<Vec<u8>> {
    let codec = if eac3 {
        oxideav_ac3::CODEC_ID_STR_EAC3
    } else {
        oxideav_ac3::CODEC_ID_STR
    };
    let mut params = CodecParameters::audio(CodecId::new(codec));
    params.sample_rate = Some(RATE);
    params.channels = Some(CHANNELS as u16);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(448_000);
    params
        .options
        .insert("dynrng".to_string(), dynrng.to_string());
    let mut enc = if eac3 {
        oxideav_ac3::eac3::make_encoder(&params).expect("eac3 make_encoder")
    } else {
        oxideav_ac3::encoder::make_encoder(&params).expect("ac3 make_encoder")
    };
    let n = RATE as usize;
    let mut s16 = Vec::with_capacity(n * CHANNELS * 2);
    for i in 0..n {
        let t = i as f32 / RATE as f32;
        for ch in 0..CHANNELS {
            // WAV order: FL FR FC LFE BL BR — the LFE gets a 50 Hz tone.
            let v = if ch == 3 {
                0.3 * (2.0 * std::f32::consts::PI * 50.0 * t).sin()
            } else {
                0.3 * (2.0 * std::f32::consts::PI * (330.0 + 70.0 * ch as f32) * t).sin()
            };
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

/// Decode (passthrough, S16) and return the per-channel RMS of the
/// output in WAV order, skipping the first frame's transform ramp.
fn channel_rms(packets: &[Vec<u8>]) -> [f64; CHANNELS] {
    let mut params = CodecParameters::audio(CodecId::new(oxideav_ac3::CODEC_ID_STR));
    params.sample_rate = Some(RATE);
    let mut dec = oxideav_ac3::decoder::make_decoder(&params).expect("make_decoder");
    let mut acc = [0f64; CHANNELS];
    let mut n = 0usize;
    for (i, pkt) in packets.iter().enumerate() {
        dec.send_packet(&Packet::new(0, TimeBase::new(1, RATE as i64), pkt.clone()))
            .expect("send_packet");
        while let Ok(Frame::Audio(af)) = dec.receive_frame() {
            if i == 0 {
                continue;
            }
            for frame in af.data[0].as_chunks::<2>().0.as_chunks::<CHANNELS>().0 {
                for (ch, s) in frame.iter().enumerate() {
                    let v = i16::from_le_bytes(*s) as f64 / 32768.0;
                    acc[ch] += v * v;
                }
                n += 1;
            }
        }
    }
    assert!(n > 0);
    acc.map(|a| (a / n as f64).sqrt())
}

fn assert_lfe_follows_dynrng(eac3: bool) {
    // 0xc8 = X 110 (−6.02 dB) . Y 01000 → −9.54 dB (Table 7.29).
    let word = 0xc8u8;
    let gain = oxideav_ac3::drc::dynrng_to_linear(word) as f64;
    let unity = channel_rms(&encode(eac3, "0"));
    let scaled = channel_rms(&encode(eac3, "0xc8"));
    for ch in 0..CHANNELS {
        assert!(unity[ch] > 0.05, "ch{ch} silent: {}", unity[ch]);
        let ratio = scaled[ch] / unity[ch];
        let db = 20.0 * (ratio / gain).log10();
        assert!(
            db.abs() < 0.5,
            "ch{ch}: dynrng 0x{word:02x} moved the level by {ratio:.4}× (expected {gain:.4}×, off by {db:.2} dB)"
        );
    }
}

#[test]
fn ac3_lfe_follows_dynrng() {
    assert_lfe_follows_dynrng(false);
}

#[test]
fn eac3_lfe_follows_dynrng() {
    assert_lfe_follows_dynrng(true);
}
