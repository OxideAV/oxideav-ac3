//! E-AC-3 rate/quality curves — encode a fixture at a ladder of bit
//! rates with the standard, AHT, SPX, and enhanced-coupling coding
//! paths, decode with the in-tree decoder, and print the per-rate PSNR
//! table.
//!
//! Each row exposes the outcome of the encoder's per-frame SNR-offset
//! search: at every rate the tuner picks the largest `(csnroffst,
//! fsnroffst)` whose exact mantissa cost fits the frame, so the PSNR
//! column IS the quality curve traced by that search across the rate
//! ladder.
//!
//! Usage:
//!
//! ```sh
//! cargo run --release --example eac3_rate_curves -- sine
//! cargo run --release --example eac3_rate_curves -- multitone
//! cargo run --release --example eac3_rate_curves -- modulated
//! ```

use std::env;

use oxideav_ac3::eac3;
use oxideav_ac3::eac3::decoder::{decode_eac3_packet, Eac3DecoderState};
use oxideav_ac3::eac3::ecplenc::EcplParams;
use oxideav_ac3::eac3::spxenc::SpxParams;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Encoder, Error, Frame, SampleFormat};

const SR: u32 = 48_000;
const CHANNELS: usize = 2;
const DUR: f32 = 2.0;

fn build_pcm(kind: &str) -> Vec<f32> {
    let n = (SR as f32 * DUR) as usize;
    let mut pcm = vec![0.0f32; n * CHANNELS];
    let mut lfsr: u32 = 0x2545_F491;
    for i in 0..n {
        let t = i as f32 / SR as f32;
        let s = match kind {
            "multitone" => {
                let tones: [(f32, f32); 6] = [
                    (700.0, 0.22),
                    (3_100.0, 0.18),
                    (6_000.0, 0.14),
                    (12_200.0, 0.055),
                    (15_400.0, 0.035),
                    (19_000.0, 0.030),
                ];
                let mut acc = 0.0f32;
                for (f, a) in tones {
                    acc += a * (2.0 * std::f32::consts::PI * f * t).sin();
                }
                lfsr ^= lfsr << 13;
                lfsr ^= lfsr >> 17;
                lfsr ^= lfsr << 5;
                acc + ((lfsr as f32 / u32::MAX as f32) * 2.0 - 1.0) * 0.012
            }
            "modulated" => {
                let m = 1.0 + 0.8 * (2.0 * std::f32::consts::PI * 40.0 * t).sin();
                (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.2 * m
            }
            _ => {
                (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.4
                    + (2.0 * std::f32::consts::PI * 3500.0 * t).sin() * 0.25
            }
        };
        for ch in 0..CHANNELS {
            pcm[i * CHANNELS + ch] = s * (1.0 - 0.1 * ch as f32);
        }
    }
    pcm
}

fn encode(pcm: &[f32], kbps: u32, mode: &str) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new(eac3::CODEC_ID_STR));
    params.sample_rate = Some(SR);
    params.channels = Some(CHANNELS as u16);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(kbps as u64 * 1000);
    let mut enc: Box<dyn Encoder> = match mode {
        "aht" => eac3::make_encoder_with_aht(&params).expect("aht encoder"),
        "spx" => eac3::make_encoder_with_spx(&params, SpxParams::default()).expect("spx encoder"),
        "ecpl" => {
            eac3::make_encoder_with_ecpl(&params, EcplParams::default()).expect("ecpl encoder")
        }
        _ => eac3::make_encoder(&params).expect("standard encoder"),
    };
    let n_samp = pcm.len() / CHANNELS;
    let mut s16 = Vec::with_capacity(pcm.len() * 2);
    for &v in pcm {
        let q = (v * 32767.0).clamp(-32768.0, 32767.0) as i16;
        s16.extend_from_slice(&q.to_le_bytes());
    }
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: n_samp as u32,
        pts: Some(0),
        data: vec![s16],
    }))
    .unwrap();
    enc.flush().unwrap();
    let mut out = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(p) => out.extend_from_slice(&p.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("encode error: {e:?}"),
        }
    }
    out
}

fn decode(stream: &[u8], frame_bytes: usize) -> Vec<i16> {
    let mut st = Eac3DecoderState::default();
    let mut out = Vec::new();
    let mut off = 0usize;
    while off + frame_bytes <= stream.len() {
        let f = decode_eac3_packet(&mut st, &stream[off..off + frame_bytes]).expect("decode");
        for c in f.pcm_s16le.chunks_exact(2) {
            out.push(i16::from_le_bytes([c[0], c[1]]));
        }
        off += frame_bytes;
    }
    out
}

/// PSNR vs the source, aligned past the 256-sample MDCT priming delay.
fn psnr(pcm: &[f32], dec: &[i16]) -> f64 {
    let delay = 256 * CHANNELS;
    let input: Vec<i16> = pcm
        .iter()
        .map(|&v| (v * 32767.0).clamp(-32768.0, 32767.0) as i16)
        .collect();
    let n = dec.len().saturating_sub(delay).min(input.len());
    let mut se = 0.0f64;
    for i in 0..n {
        let d = dec[i + delay] as f64 - input[i] as f64;
        se += d * d;
    }
    let mse = se / n.max(1) as f64;
    10.0 * (32768.0f64 * 32768.0 / mse.max(1e-30)).log10()
}

fn main() {
    let kind = env::args().nth(1).unwrap_or_else(|| "sine".into());
    let pcm = build_pcm(&kind);
    println!("fixture: {kind} (stereo, {DUR} s @ {SR} Hz)");
    println!(
        "{:>6} | {:>12} | {:>12} | {:>12} | {:>12}",
        "kbps", "standard", "aht", "spx", "ecpl"
    );
    println!(
        "{:->6}-+-{:->12}-+-{:->12}-+-{:->12}-+-{:->12}",
        "", "", "", "", ""
    );
    for kbps in [96u32, 128, 160, 192, 256, 320, 384, 448] {
        let frame_bytes = (kbps * 4) as usize / 2 * 2;
        let mut row = format!("{kbps:>6}");
        for mode in ["std", "aht", "spx", "ecpl"] {
            let stream = encode(&pcm, kbps, mode);
            assert_eq!(stream.len() % frame_bytes, 0);
            let dec = decode(&stream, frame_bytes);
            row.push_str(&format!(" | {:>9.2} dB", psnr(&pcm, &dec)));
        }
        println!("{row}");
    }
}
