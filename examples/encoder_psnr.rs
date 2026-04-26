//! Encode a PCM test vector, decode with our own decoder, and report PSNR.
//!
//! Usage:
//!
//! ```sh
//! cargo run --release --example encoder_psnr -- sine
//! cargo run --release --example encoder_psnr -- transient
//! cargo run --release --example encoder_psnr -- speech
//! cargo run --release --example encoder_psnr -- chirp
//! ```

use std::env;

use oxideav_ac3::decoder;
use oxideav_ac3::encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Error, Frame, SampleFormat};

const SR: u32 = 48_000;

fn build_pcm(kind: &str) -> (Vec<f32>, Vec<f32>, f32) {
    // (left, right, duration_seconds)
    let dur = 2.0f32;
    let n = (SR as f32 * dur) as usize;
    let mut l = vec![0.0f32; n];
    let mut r = vec![0.0f32; n];
    match kind {
        "sine" => {
            for i in 0..n {
                let t = i as f32 / SR as f32;
                let s = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.4;
                l[i] = s;
                r[i] = s;
            }
        }
        "chirp" => {
            // 100 Hz → 8 kHz log sweep, mono dup'd.
            let f0 = 100.0f32;
            let f1 = 8000.0f32;
            let k = (f1 / f0).ln() / dur;
            for i in 0..n {
                let t = i as f32 / SR as f32;
                let phase = 2.0 * std::f32::consts::PI * f0 * ((k * t).exp() - 1.0) / k;
                let s = phase.sin() * 0.35;
                l[i] = s;
                r[i] = s;
            }
        }
        "speech" => {
            // Formant-ish synthetic speech: three formants modulated.
            for i in 0..n {
                let t = i as f32 / SR as f32;
                let fund = 140.0 + 20.0 * (2.0 * std::f32::consts::PI * 3.0 * t).sin();
                let f0 = 1.0 * fund;
                let f1 = 5.0 * fund;
                let f2 = 10.0 * fund;
                let env = (1.0 + 0.5 * (2.0 * std::f32::consts::PI * 6.0 * t).sin()) * 0.15;
                let s = env
                    * ((2.0 * std::f32::consts::PI * f0 * t).sin() * 0.6
                        + (2.0 * std::f32::consts::PI * f1 * t).sin() * 0.3
                        + (2.0 * std::f32::consts::PI * f2 * t).sin() * 0.1);
                l[i] = s;
                r[i] = s;
            }
        }
        "transient" => {
            // Silence + click + decay pattern: tests block handling.
            let click_period = SR as usize / 8;
            for i in 0..n {
                let phase = i % click_period;
                if phase < 64 {
                    let env = (1.0 - phase as f32 / 64.0).max(0.0);
                    let s = (2.0 * std::f32::consts::PI * 2000.0 * i as f32 / SR as f32).sin()
                        * env
                        * 0.5;
                    l[i] = s;
                    r[i] = s;
                }
            }
        }
        "stereo" => {
            // Different signals per channel to stress stereo.
            for i in 0..n {
                let t = i as f32 / SR as f32;
                l[i] = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.3;
                r[i] = (2.0 * std::f32::consts::PI * 660.0 * t).sin() * 0.3;
            }
        }
        _ => panic!("unknown kind {kind}"),
    }
    (l, r, dur)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let kind = env::args().nth(1).unwrap_or_else(|| "sine".to_string());
    let (l, r, _dur) = build_pcm(&kind);
    let n = l.len();

    // Encode.
    let mut enc_params = CodecParameters::audio(CodecId::new("ac3"));
    enc_params.sample_rate = Some(SR);
    enc_params.channels = Some(2);
    enc_params.sample_format = Some(SampleFormat::S16);
    enc_params.bit_rate = Some(192_000);
    let mut enc = encoder::make_encoder(&enc_params)?;

    let mut pcm_bytes = Vec::with_capacity(n * 4);
    for i in 0..n {
        let li = (l[i] * 32767.0).clamp(-32768.0, 32767.0) as i16;
        let ri = (r[i] * 32767.0).clamp(-32768.0, 32767.0) as i16;
        pcm_bytes.extend_from_slice(&li.to_le_bytes());
        pcm_bytes.extend_from_slice(&ri.to_le_bytes());
    }
    let audio = AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![pcm_bytes],
    };
    enc.send_frame(&Frame::Audio(audio))?;
    enc.flush()?;
    let mut packets = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(p) => packets.push(p),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => return Err(e.into()),
        }
    }
    eprintln!(
        "encoded {} syncframes ({} bytes)",
        packets.len(),
        packets.iter().map(|p| p.data.len()).sum::<usize>()
    );

    // Decode with our own decoder.
    let dparams = CodecParameters::audio(CodecId::new("ac3"));
    let mut dec = decoder::make_decoder(&dparams)?;
    let mut dec_l: Vec<f32> = Vec::new();
    let mut dec_r: Vec<f32> = Vec::new();
    for p in &packets {
        dec.send_packet(p)?;
        if let Ok(Frame::Audio(a)) = dec.receive_frame() {
            let plane = &a.data[0];
            for c in plane.chunks_exact(4) {
                let li = i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0;
                let ri = i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0;
                dec_l.push(li);
                dec_r.push(ri);
            }
        }
    }
    eprintln!("decoded {} samples", dec_l.len());

    // Align: AC-3 has 256 samples of overlap-add latency (one block).
    // Pin the lag to exactly 256 which is the expected IMDCT + OLA
    // latency — scanning introduces false minima on periodic signals.
    let skip = 2048usize.min(dec_l.len().saturating_sub(1));
    let best_lag: i32 = 256;
    let _ = (dec_r.len(),); // silence unused-var for early exits
                            // Full-length PSNR using best_lag.
    let usable = l
        .len()
        .saturating_sub(skip)
        .min(dec_l.len().saturating_sub(skip + best_lag as usize));
    let mut sse_l = 0.0f64;
    let mut sse_r = 0.0f64;
    let mut ref_e_l = 0.0f64;
    let mut ref_e_r = 0.0f64;
    for i in 0..usable {
        let dl = l[skip + i] as f64 - dec_l[skip + best_lag as usize + i] as f64;
        let dr = r[skip + i] as f64 - dec_r[skip + best_lag as usize + i] as f64;
        sse_l += dl * dl;
        sse_r += dr * dr;
        ref_e_l += (l[skip + i] as f64).powi(2);
        ref_e_r += (r[skip + i] as f64).powi(2);
    }
    let mse_l = sse_l / usable as f64;
    let mse_r = sse_r / usable as f64;
    let psnr_l = 10.0 * (1.0f64 / mse_l.max(1e-30)).log10();
    let psnr_r = 10.0 * (1.0f64 / mse_r.max(1e-30)).log10();
    let snr_l = 10.0 * (ref_e_l / sse_l.max(1e-30)).log10();
    let snr_r = 10.0 * (ref_e_r / sse_r.max(1e-30)).log10();
    println!(
        "kind={kind} lag={best_lag} samples PSNR_L={psnr_l:.2}dB PSNR_R={psnr_r:.2}dB SNR_L={snr_l:.2}dB SNR_R={snr_r:.2}dB"
    );
    Ok(())
}
