//! Per-block PSNR + peak diff between our decoder's PCM and ffmpeg's
//! reference decode of the same AC-3 stream. Used to localise residual
//! drift on burst frames where the per-frame PSNR floor (currently
//! ≈15 dB on `transient_bursts_stereo.ac3`) hides which sub-block is
//! actually breaking.
//!
//! Usage: `sample_compare [<file.ac3>] [<first_frame>] [<last_frame>]`
//!
//! Defaults to the bundled transient fixture, frames 14-15.

use std::env;
use std::fs;
use std::process::Command;

use oxideav_ac3::{bsi, decoder::SAMPLES_PER_FRAME, syncinfo};
use oxideav_core::CodecRegistry;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let path = args.get(1).cloned().unwrap_or_else(|| {
        "crates/oxideav-ac3/tests/fixtures/transient_bursts_stereo.ac3".to_string()
    });
    let first: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(14);
    let last: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(15);

    let data = fs::read(&path)?;

    let tmp = std::env::temp_dir().join("oxideav_sample_cmp.pcm");
    Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error", "-i"])
        .arg(&path)
        .args(["-f", "s16le", "-acodec", "pcm_s16le", "-ac", "2"])
        .arg(&tmp)
        .status()?;
    let ref_pcm = fs::read(&tmp)?;
    let _ = fs::remove_file(&tmp);

    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    let params = CodecParameters::audio(CodecId::new("ac3"));
    let mut dec = reg.first_decoder(&params).expect("make_decoder");
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
    let extract = |buf: &[u8], ch: usize| -> Vec<i16> {
        buf.chunks_exact(4)
            .map(|c| i16::from_le_bytes([c[ch * 2], c[ch * 2 + 1]]))
            .collect()
    };
    let our_l = extract(&our_pcm, 0);
    let ref_l = extract(&ref_pcm, 0);

    let skip = 768usize;
    let usable = our_l.len().min(ref_l.len()).saturating_sub(skip);
    let mut best_lag = 0i32;
    let mut best_sse = f64::INFINITY;
    for lag in -512i32..=512 {
        let mut sse = 0.0f64;
        let mut count = 0;
        for i in 0..usable.min(2048) {
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
    println!("global best_lag = {best_lag}");

    for f in first..=last {
        let frame_base = skip + f * 1536;
        for blk in 0..6 {
            let base = frame_base + blk * 256;
            let mut sse = 0.0f64;
            let mut count = 0usize;
            let mut peak_diff: i64 = 0;
            let mut peak_ours: i64 = 0;
            let mut peak_ref: i64 = 0;
            for i in 0..256 {
                let a_idx = base + i;
                let b_idx = a_idx as i32 + best_lag;
                if b_idx < 0 || (b_idx as usize) >= ref_l.len() {
                    continue;
                }
                if a_idx >= our_l.len() {
                    break;
                }
                let o = our_l[a_idx] as i64;
                let r = ref_l[b_idx as usize] as i64;
                let d = o - r;
                sse += (d * d) as f64;
                count += 1;
                if d.abs() > peak_diff.abs() {
                    peak_diff = d;
                }
                if o.abs() > peak_ours.abs() {
                    peak_ours = o;
                }
                if r.abs() > peak_ref.abs() {
                    peak_ref = r;
                }
            }
            if count == 0 {
                continue;
            }
            let mse = sse / count as f64;
            let psnr = if mse > 0.0 {
                10.0 * ((32767.0f64 * 32767.0) / mse).log10()
            } else {
                f64::INFINITY
            };
            println!(
                "frame {f} blk {blk}: PSNR = {:6.2} dB, peak_diff = {:6}, peak_ours = {:6}, peak_ref = {:6}",
                psnr, peak_diff, peak_ours, peak_ref
            );
        }
    }
    Ok(())
}
