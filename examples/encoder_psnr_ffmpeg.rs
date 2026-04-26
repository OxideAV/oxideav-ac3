//! Encode a PCM test vector, decode with ffmpeg as an external reference,
//! and report PSNR. Complements `encoder_psnr` which uses our own decoder.
//!
//! Usage:
//!
//! ```sh
//! cargo run --release --example encoder_psnr_ffmpeg -- sine
//! ```

use std::env;
use std::fs;
use std::io::Write;
use std::process::Command;

use oxideav_ac3::encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Error, Frame, SampleFormat};

const SR: u32 = 48_000;

fn build_pcm(kind: &str) -> (Vec<f32>, Vec<f32>, f32) {
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
            for i in 0..n {
                let t = i as f32 / SR as f32;
                let fund = 140.0 + 20.0 * (2.0 * std::f32::consts::PI * 3.0 * t).sin();
                let env = (1.0 + 0.5 * (2.0 * std::f32::consts::PI * 6.0 * t).sin()) * 0.15;
                let s = env
                    * ((2.0 * std::f32::consts::PI * fund * t).sin() * 0.6
                        + (2.0 * std::f32::consts::PI * 5.0 * fund * t).sin() * 0.3
                        + (2.0 * std::f32::consts::PI * 10.0 * fund * t).sin() * 0.1);
                l[i] = s;
                r[i] = s;
            }
        }
        "transient" => {
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
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![pcm_bytes],
    }))?;
    enc.flush()?;

    // Write encoded bytes to a temp ac3 file.
    let tmp_ac3 = std::env::temp_dir().join(format!("oxideav_enc_{kind}.ac3"));
    let tmp_pcm = std::env::temp_dir().join(format!("oxideav_enc_{kind}.pcm"));
    {
        let mut f = fs::File::create(&tmp_ac3)?;
        loop {
            match enc.receive_packet() {
                Ok(p) => f.write_all(&p.data)?,
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => return Err(e.into()),
            }
        }
    }

    // ffmpeg decode.
    let status = Command::new("ffmpeg")
        .args(["-y", "-v", "error", "-i"])
        .arg(&tmp_ac3)
        .args(["-f", "s16le", "-acodec", "pcm_s16le", "-ac", "2"])
        .arg(&tmp_pcm)
        .status()?;
    if !status.success() {
        return Err("ffmpeg decode failed".into());
    }
    let dec = fs::read(&tmp_pcm)?;
    let nd = dec.len() / 4;
    let mut dec_l = Vec::with_capacity(nd);
    let mut dec_r = Vec::with_capacity(nd);
    for c in dec.chunks_exact(4) {
        let li = i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0;
        let ri = i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0;
        dec_l.push(li);
        dec_r.push(ri);
    }

    // Lag search in 0..=2048 using correlation on both channels; PSNR
    // with peak amplitude 1.0.
    let skip = 2048.min(dec_l.len().saturating_sub(1));
    let mut best_lag = 0i32;
    let mut best_mse = f64::INFINITY;
    for lag in 0..=2048i32 {
        if skip + lag as usize + 1024 > dec_l.len() {
            continue;
        }
        if skip + 1024 > l.len() {
            continue;
        }
        let mut acc = 0.0f64;
        for i in 0..1024 {
            let dl = l[skip + i] as f64 - dec_l[skip + lag as usize + i] as f64;
            let dr = r[skip + i] as f64 - dec_r[skip + lag as usize + i] as f64;
            acc += dl * dl + dr * dr;
        }
        if acc < best_mse {
            best_mse = acc;
            best_lag = lag;
        }
    }
    let usable = l
        .len()
        .saturating_sub(skip)
        .min(dec_l.len().saturating_sub(skip + best_lag as usize));
    let mut sse_l = 0.0f64;
    let mut sse_r = 0.0f64;
    for i in 0..usable {
        let dl = l[skip + i] as f64 - dec_l[skip + best_lag as usize + i] as f64;
        let dr = r[skip + i] as f64 - dec_r[skip + best_lag as usize + i] as f64;
        sse_l += dl * dl;
        sse_r += dr * dr;
    }
    let mse_l = sse_l / usable as f64;
    let mse_r = sse_r / usable as f64;
    let psnr_l = 10.0 * (1.0f64 / mse_l.max(1e-30)).log10();
    let psnr_r = 10.0 * (1.0f64 / mse_r.max(1e-30)).log10();
    println!("kind={kind} lag={best_lag} PSNR_L={psnr_l:.2}dB PSNR_R={psnr_r:.2}dB");
    let _ = fs::remove_file(tmp_ac3);
    let _ = fs::remove_file(tmp_pcm);
    Ok(())
}
