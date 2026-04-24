//! Per-frame PSNR diagnostic. Decode the named AC-3 stream with our
//! decoder and with ffmpeg, then print PSNR per 1536-sample frame so we
//! can locate which frames disagree the most.

use std::env;
use std::fs;
use std::process::Command;

use oxideav_ac3::{bsi, decoder::SAMPLES_PER_FRAME, syncinfo};
use oxideav_codec::CodecRegistry;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args()
        .nth(1)
        .ok_or("usage: psnr_per_frame <file.ac3>")?;
    let data = fs::read(&path)?;

    // ffmpeg reference decode.
    let tmp = std::env::temp_dir().join("oxideav_psnr_ref.pcm");
    let status = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error", "-i"])
        .arg(&path)
        .args(["-f", "s16le", "-acodec", "pcm_s16le", "-ac", "2"])
        .arg(&tmp)
        .status()?;
    if !status.success() {
        return Err("ffmpeg failed".into());
    }
    let ref_pcm = fs::read(&tmp)?;
    let _ = fs::remove_file(&tmp);

    // Our decoder.
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register(&mut reg);
    let params = CodecParameters::audio(CodecId::new("ac3"));
    let mut dec = reg.make_decoder(&params).expect("make_decoder");

    let mut our_pcm: Vec<u8> = Vec::new();
    let mut offset = 0;
    let mut frame_idx: i64 = 0;
    while offset < data.len() {
        let si = syncinfo::parse(&data[offset..])?;
        let _ = bsi::parse(&data[offset + 5..])?;
        let flen = si.frame_length as usize;
        let pkt = Packet::new(
            0,
            TimeBase::new(1, 48_000),
            data[offset..offset + flen].to_vec(),
        )
        .with_pts(frame_idx * SAMPLES_PER_FRAME as i64);
        dec.send_packet(&pkt)?;
        if let Ok(Frame::Audio(a)) = dec.receive_frame() {
            our_pcm.extend_from_slice(&a.data[0]);
        }
        offset += flen;
        frame_idx += 1;
    }

    let extract_ch = |buf: &[u8], ch: usize| -> Vec<i16> {
        buf.chunks_exact(4)
            .map(|c| {
                let o = ch * 2;
                i16::from_le_bytes([c[o], c[o + 1]])
            })
            .collect()
    };
    let ch_pick: usize = env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let our_l = extract_ch(&our_pcm, ch_pick);
    let ref_l = extract_ch(&ref_pcm, ch_pick);
    eprintln!("examining channel {ch_pick}");

    // Find global lag on first 2 frames post-prime.
    let skip = 768usize;
    let usable = our_l.len().min(ref_l.len()).saturating_sub(skip);
    let mut best_lag = 0i32;
    let mut best_sse = f64::INFINITY;
    for lag in -512i32..=512 {
        let mut sse = 0.0f64;
        let mut count = 0;
        for i in 0..usable.min(4096) {
            let a_idx = (skip + i) as i32;
            let b_idx = a_idx + lag;
            if b_idx < 0 || (b_idx as usize) >= ref_l.len() {
                continue;
            }
            let d = our_l[a_idx as usize] as f64 - ref_l[b_idx as usize] as f64;
            sse += d * d;
            count += 1;
        }
        if count > 0 && sse / (count as f64) < best_sse {
            best_sse = sse / (count as f64);
            best_lag = lag;
        }
    }
    println!("global best_lag = {best_lag}, sse/sample = {:.1}", best_sse);

    // Per-frame PSNR with best_lag.
    let frame_samples = SAMPLES_PER_FRAME as usize;
    let nframes = usable / frame_samples;
    for f in 0..nframes {
        let base = skip + f * frame_samples;
        let mut sse = 0.0f64;
        let mut count = 0;
        for i in 0..frame_samples {
            let a_idx = base + i;
            let b_idx = a_idx as i32 + best_lag;
            if b_idx < 0 || (b_idx as usize) >= ref_l.len() {
                continue;
            }
            let d = our_l[a_idx] as f64 - ref_l[b_idx as usize] as f64;
            sse += d * d;
            count += 1;
        }
        if count == 0 {
            continue;
        }
        let mse = sse / count as f64;
        let peak = 32767.0f64;
        let psnr = if mse > 0.0 {
            10.0 * (peak * peak / mse).log10()
        } else {
            f64::INFINITY
        };
        println!("frame {f}: PSNR = {:.2} dB (mse={:.1})", psnr, mse);
    }
    Ok(())
}
