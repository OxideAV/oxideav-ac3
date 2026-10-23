//! Table 7.15's 44.1 kHz hearing threshold row, through streams whose
//! bit allocation reaches it.
//!
//! Each fixture is six syncframes of a black-box encoder's 44.1 kHz
//! stereo AC-3 stream of a sine tone that starts at phase 0:
//!
//! - `hth44_band34_stereo.ac3`: 220 Hz left and 330 Hz right at
//!   amplitude 1/16, 192 kb/s (the first six frames of a 1 s tone);
//! - `hth44_band28_stereo.ac3`: 2153 Hz at amplitude 1/(250 sqrt 2) on
//!   both channels for 0.2 s (8820 samples), 448 kb/s;
//! - `hth44_band25_stereo.ac3`: the same at half the amplitude.
//!
//! Where a band's mask is the hearing threshold a wrong threshold entry
//! changes `bap[]`, the mantissa unpack loses sync with the encoder and
//! the rest of the frame decodes to noise (bands 34, 28 and 25
//! respectively for these streams: each stream decodes at 10 dB or less
//! with its band's entry one step off). A correct decode is the tone
//! 256 samples late (the transform overlap with the frame before the
//! first), well above 30 dB.

use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

const RATE: f64 = 44_100.0;

/// The stream's interleaved stereo samples (S16, scaled to [-1, 1)).
fn decode(stream: &[u8]) -> Vec<f64> {
    let params = CodecParameters::audio(CodecId::new("ac3"));
    let mut decoder = oxideav_ac3::decoder::make_decoder(&params).expect("make_decoder");
    let (mut pcm, mut at) = (Vec::new(), 0);
    while at < stream.len() {
        let si = oxideav_ac3::syncinfo::parse(&stream[at..]).expect("syncinfo");
        assert_eq!(si.sample_rate, 44_100);
        let len = si.frame_length as usize;
        decoder
            .send_packet(&Packet::new(
                0,
                TimeBase::new(1, 44_100),
                stream[at..at + len].to_vec(),
            ))
            .expect("send_packet");
        let Frame::Audio(frame) = decoder.receive_frame().expect("receive_frame") else {
            panic!("not audio")
        };
        assert_eq!(
            frame.data[0].len(),
            frame.samples as usize * 2 * 2,
            "stereo S16"
        );
        pcm.extend(
            frame.data[0]
                .as_chunks::<2>()
                .0
                .iter()
                .map(|b| f64::from(i16::from_le_bytes(*b)) / 32768.0),
        );
        at += len;
    }
    pcm
}

/// Per-channel SNR of the decode against `tone(channel, n)` 256 samples late.
fn assert_tone(stream: &[u8], tone: impl Fn(usize, usize) -> f64) {
    let pcm = decode(stream);
    for ch in 0..2 {
        let (mut signal, mut error) = (0.0, 0.0);
        for (i, frame) in pcm.as_chunks::<2>().0.iter().enumerate() {
            let expected = if i < 256 { 0.0 } else { tone(ch, i - 256) };
            signal += expected * expected;
            error += (frame[ch] - expected).powi(2);
        }
        let snr = 10.0 * (signal / error).log10();
        assert!(snr > 30.0, "channel {ch}: {snr:.1} dB against the tone");
    }
}

fn sine(frequency: f64, amplitude: f64, n: usize) -> f64 {
    amplitude * (2.0 * std::f64::consts::PI * frequency * n as f64 / RATE).sin()
}

#[test]
fn band_34() {
    assert_tone(
        include_bytes!("fixtures/hth44_band34_stereo.ac3"),
        |ch, n| sine([220.0, 330.0][ch], 1.0 / 16.0, n),
    );
}

#[test]
fn band_28() {
    assert_tone(
        include_bytes!("fixtures/hth44_band28_stereo.ac3"),
        |_, n| {
            if n < 8820 {
                sine(2153.0, 1.0 / 250.0 / 2f64.sqrt(), n)
            } else {
                0.0
            }
        },
    );
}

#[test]
fn band_25() {
    assert_tone(
        include_bytes!("fixtures/hth44_band25_stereo.ac3"),
        |_, n| {
            if n < 8820 {
                sine(2153.0, 1.0 / 500.0 / 2f64.sqrt(), n)
            } else {
                0.0
            }
        },
    );
}
