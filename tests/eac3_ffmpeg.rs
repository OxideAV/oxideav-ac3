//! End-to-end E-AC-3 encode → ffmpeg decode test.
//!
//! Runs the round-1 E-AC-3 encoder on a synthetic 440 Hz sine fixture
//! and pipes the resulting elementary stream through `ffmpeg -i …
//! -f s16le …`. We then compute PSNR vs. the original PCM and assert
//! the decoded audio is non-trivial and reasonably faithful.
//!
//! The test is `#[ignore]`-gated by absence of ffmpeg: when ffmpeg
//! isn't on PATH we skip rather than fail, since the binary isn't a
//! hard build dep.

use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::process::Command;

use oxideav_ac3::eac3;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Error, Frame, SampleFormat};

const SR: u32 = 48_000;
const DUR_SEC: f32 = 1.0;

fn ffmpeg_present() -> bool {
    Command::new("ffmpeg")
        .args(["-version"])
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

fn build_sine_pcm(channels: usize, freq: f32) -> Vec<f32> {
    let n = (SR as f32 * DUR_SEC) as usize;
    let mut out = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f32 / SR as f32;
        let s = (2.0 * std::f32::consts::PI * freq * t).sin() * 0.4;
        for _ in 0..channels {
            out.push(s);
        }
    }
    out
}

fn encode_eac3(pcm: &[f32], channels: usize, bit_rate: u64) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new(eac3::CODEC_ID_STR));
    params.sample_rate = Some(SR);
    params.channels = Some(channels as u16);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(bit_rate);
    let mut enc = eac3::make_encoder(&params).expect("eac3 make_encoder");

    // Convert interleaved f32 PCM to interleaved S16 bytes.
    let n_samp = pcm.len() / channels;
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
            Err(e) => panic!("eac3 encode error: {e:?}"),
        }
    }
    out
}

fn ffmpeg_decode(eac3_bytes: &[u8], channels: usize) -> Option<Vec<f32>> {
    if !ffmpeg_present() {
        return None;
    }
    let dir = std::env::temp_dir();
    let in_path: PathBuf = dir.join(format!("oxideav_eac3_test_{}.ec3", std::process::id()));
    let out_path: PathBuf = dir.join(format!("oxideav_eac3_test_{}.pcm", std::process::id()));
    {
        let mut f = fs::File::create(&in_path).unwrap();
        f.write_all(eac3_bytes).unwrap();
    }
    let status = Command::new("ffmpeg")
        .args(["-y", "-v", "error", "-f", "eac3", "-i"])
        .arg(&in_path)
        .args([
            "-f",
            "s16le",
            "-acodec",
            "pcm_s16le",
            "-ac",
            &channels.to_string(),
            "-ar",
            &SR.to_string(),
        ])
        .arg(&out_path)
        .status()
        .expect("ffmpeg invocation failed");
    if !status.success() {
        let _ = fs::remove_file(&in_path);
        let _ = fs::remove_file(&out_path);
        return None;
    }
    let bytes = fs::read(&out_path).unwrap();
    let _ = fs::remove_file(&in_path);
    let _ = fs::remove_file(&out_path);
    let mut out = Vec::with_capacity(bytes.len() / 2);
    for c in bytes.chunks_exact(2) {
        let v = i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0;
        out.push(v);
    }
    Some(out)
}

/// PSNR between two channel streams — independently per-channel,
/// returns the worse-case PSNR. Skips a 2048-sample priming region
/// and lag-searches up to 2048 samples to align ffmpeg's decode delay
/// with the source.
fn psnr_min(orig: &[f32], dec: &[f32], channels: usize) -> f64 {
    let n_orig = orig.len() / channels;
    let n_dec = dec.len() / channels;
    let skip = 2048usize.min(n_dec.saturating_sub(1));
    // Align via lag search on first channel.
    let mut best_lag = 0i32;
    let mut best_mse = f64::INFINITY;
    for lag in 0..=2048i32 {
        if skip + lag as usize + 1024 > n_dec {
            continue;
        }
        if skip + 1024 > n_orig {
            continue;
        }
        let mut acc = 0.0f64;
        for i in 0..1024 {
            let d = orig[(skip + i) * channels] as f64
                - dec[(skip + lag as usize + i) * channels] as f64;
            acc += d * d;
        }
        if acc < best_mse {
            best_mse = acc;
            best_lag = lag;
        }
    }
    let usable = n_orig
        .saturating_sub(skip)
        .min(n_dec.saturating_sub(skip + best_lag as usize));
    let mut worst = f64::INFINITY;
    for ch in 0..channels {
        let mut sse = 0.0f64;
        for i in 0..usable {
            let o = orig[(skip + i) * channels + ch] as f64;
            let d = dec[(skip + best_lag as usize + i) * channels + ch] as f64;
            let e = o - d;
            sse += e * e;
        }
        let mse = sse / usable as f64;
        let psnr = 10.0 * (1.0f64 / mse.max(1e-30)).log10();
        if psnr < worst {
            worst = psnr;
        }
    }
    worst
}

#[test]
fn eac3_stereo_192k_decodes_through_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg not in PATH — skipping interop test");
        return;
    }
    let pcm = build_sine_pcm(2, 440.0);
    let frame = encode_eac3(&pcm, 2, 192_000);
    assert!(!frame.is_empty(), "encoder produced no bytes");
    // Sanity-check syncword presence at the start of the stream.
    assert_eq!(
        &frame[0..2],
        &[0x0B, 0x77],
        "first frame must start with the AC-3/E-AC-3 syncword"
    );
    let decoded = ffmpeg_decode(&frame, 2).expect("ffmpeg decode failed");
    assert!(
        decoded.len() >= 1024,
        "ffmpeg returned only {} samples — expected ≥ 1024",
        decoded.len()
    );
    let psnr = psnr_min(&pcm, &decoded, 2);
    eprintln!("E-AC-3 stereo 192k → ffmpeg PSNR = {psnr:.2} dB");
    // 18 dB matches the AC-3 baseline encoder's PSNR vs ffmpeg on
    // pure-sine input (the encoder's loose snroffst tuning + reference
    // mismatch in the lag-search window keeps PSNR around 20 dB even
    // for the AC-3 path — see `examples/encoder_psnr_ffmpeg.rs`).
    assert!(
        psnr >= 18.0,
        "PSNR {psnr:.2} dB below 18 dB acceptance floor"
    );
}

#[test]
fn eac3_mono_96k_decodes_through_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg not in PATH — skipping interop test");
        return;
    }
    let pcm = build_sine_pcm(1, 440.0);
    let frame = encode_eac3(&pcm, 1, 96_000);
    assert!(!frame.is_empty(), "encoder produced no bytes");
    assert_eq!(&frame[0..2], &[0x0B, 0x77]);
    let decoded = ffmpeg_decode(&frame, 1).expect("ffmpeg decode failed");
    assert!(decoded.len() >= 1024);
    let psnr = psnr_min(&pcm, &decoded, 1);
    eprintln!("E-AC-3 mono 96k → ffmpeg PSNR = {psnr:.2} dB");
    // 18 dB matches the AC-3 baseline encoder's PSNR vs ffmpeg on
    // pure-sine input (the encoder's loose snroffst tuning + reference
    // mismatch in the lag-search window keeps PSNR around 20 dB even
    // for the AC-3 path — see `examples/encoder_psnr_ffmpeg.rs`).
    assert!(
        psnr >= 18.0,
        "PSNR {psnr:.2} dB below 18 dB acceptance floor"
    );
}

#[test]
fn eac3_first_frame_is_syncframe() {
    let pcm = build_sine_pcm(2, 440.0);
    let frame = encode_eac3(&pcm, 2, 192_000);
    assert!(frame.len() >= 768);
    // Each frame is 768 bytes at 192 kbps / 48 kHz / 1536 samples.
    assert_eq!(
        frame.len() % 768,
        0,
        "concatenated frames should sum to whole number of 768-byte syncframes"
    );
    // Check that every 768-byte boundary starts with 0x0B 0x77.
    for off in (0..frame.len()).step_by(768) {
        assert_eq!(
            &frame[off..off + 2],
            &[0x0B, 0x77],
            "missing syncword at frame offset {off}"
        );
    }
}
