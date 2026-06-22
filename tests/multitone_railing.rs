//! Regression test for issue #10 — railed ±32767/±32768 sample bursts on
//! multi-tone / real content.
//!
//! Root cause was a run-length error in the §7.2.2.3 log-addition table
//! (`LATAB`, A/52 Table 7.14): several mid-table runs (A=12/13/14/17) were
//! one entry too short, so `latab[]` under-counted, `bndpsd` PSD
//! integration under-estimated the masking curve, and high-frequency bins
//! were allocated too few bits → quantiser overflow → full-scale railing.
//! A pure single tone stays in the clean part of the table and never
//! exposed the bug, so the original tone fixture passed.
//!
//! This test uses the `ffmpeg` CLI **purely as a black-box encoder/oracle**
//! to produce a multi-tone AC-3 stream, decodes it through the crate's
//! public API, and asserts the fraction of full-scale samples is ≈0. It
//! also decodes a single-tone control and confirms it remains clean.
//!
//! Skips silently when `ffmpeg` is unavailable so minimal builders still
//! pass.

use std::process::Command;

use oxideav_ac3::decoder::make_decoder;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

fn ffmpeg_available() -> bool {
    Command::new("ffmpeg")
        .args(["-hide_banner", "-version"])
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

/// Encode a lavfi `aevalsrc`/`sine` source to an AC-3 elementary stream.
/// Returns the raw `.ac3` bytes, or `None` if ffmpeg failed.
fn encode_ac3(lavfi_src: &str, bitrate: &str, path: &std::path::Path) -> Option<Vec<u8>> {
    let status = Command::new("ffmpeg")
        .args([
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "lavfi",
            "-i",
            lavfi_src,
            "-c:a",
            "ac3",
            "-b:a",
            bitrate,
            "-f",
            "ac3",
        ])
        .arg(path)
        .status()
        .ok()?;
    if !status.success() {
        return None;
    }
    std::fs::read(path).ok()
}

/// Decode an AC-3 elementary stream through the crate's public API,
/// returning interleaved S16 samples. Frame-splitting via `syncinfo::parse`
/// exactly as in the issue's minimal repro.
fn decode_ac3(data: &[u8]) -> Vec<i16> {
    let mut params = CodecParameters::audio(CodecId::new("ac3"));
    params.channels = Some(2);
    let mut dec = make_decoder(&params).expect("decoder factory");

    let mut offset = 0;
    let mut pcm: Vec<i16> = Vec::new();
    while offset < data.len() {
        let si = match oxideav_ac3::syncinfo::parse(&data[offset..]) {
            Ok(s) => s,
            Err(_) => break,
        };
        let flen = si.frame_length as usize;
        if flen == 0 || offset + flen > data.len() {
            break;
        }
        let pkt = Packet::new(
            0,
            TimeBase::new(1, 48_000),
            data[offset..offset + flen].to_vec(),
        );
        dec.send_packet(&pkt).unwrap();
        if let Ok(Frame::Audio(a)) = dec.receive_frame() {
            for s in a.data[0].chunks_exact(2) {
                pcm.push(i16::from_le_bytes([s[0], s[1]]));
            }
        }
        offset += flen;
    }
    pcm
}

/// Count of |sample| >= 32767 (full-scale rail) and the total.
fn full_scale_stats(pcm: &[i16]) -> (usize, usize) {
    let railed = pcm.iter().filter(|&&s| s.unsigned_abs() >= 32767).count();
    (railed, pcm.len())
}

#[test]
fn multitone_does_not_rail() {
    if !ffmpeg_available() {
        eprintln!("ffmpeg unavailable — skipping multitone railing test");
        return;
    }

    let dir = std::env::temp_dir();

    // Four-tone source (issue #10 repro): 300/900/3000/7000 Hz summed.
    // The 3 kHz + 7 kHz partials land in the high-frequency bins whose
    // bit allocation was corrupted by the bad `LATAB` runs.
    let multi_src = "aevalsrc=0.15*(sin(2*PI*300*t)+sin(2*PI*900*t)\
        +sin(2*PI*3000*t)+sin(2*PI*7000*t)):s=48000:d=4:c=stereo";
    // 384 kbps: high enough that the bit pool reaches the high-frequency
    // bins whose masking the corrupted `LATAB` runs distorted. At this rate
    // the pre-fix decoder railed ~9% of all samples on this content; the
    // fixed table (and ffmpeg) decode it with zero full-scale samples.
    let multi_path = dir.join("oxideav_ac3_multitone.ac3");
    let multi = match encode_ac3(multi_src, "384k", &multi_path) {
        Some(b) => b,
        None => {
            eprintln!("ffmpeg multitone encode failed — skipping");
            return;
        }
    };

    let pcm = decode_ac3(&multi);
    assert!(
        !pcm.is_empty(),
        "multitone decode produced no samples ({} bytes in)",
        multi.len()
    );

    let (railed, total) = full_scale_stats(&pcm);
    let frac = railed as f64 / total as f64;
    // ffmpeg decodes this same file with zero full-scale samples. With the
    // corrected §7.2.2.3 log-addition table our decoder must too; allow a
    // tiny tolerance for legitimate peaks but reject the burst behaviour
    // (pre-fix this was a large fraction of all samples).
    assert!(
        frac < 1e-3,
        "multitone railing not eliminated: {railed}/{total} = {frac:.5} full-scale samples \
         (issue #10 regression — check LATAB / §7.2.2.3 PSD integration)"
    );

    let _ = std::fs::remove_file(&multi_path);
}

#[test]
fn single_tone_control_stays_clean() {
    if !ffmpeg_available() {
        eprintln!("ffmpeg unavailable — skipping single-tone control");
        return;
    }

    let dir = std::env::temp_dir();
    let tone_path = dir.join("oxideav_ac3_singletone.ac3");
    let tone_src = "sine=frequency=997:sample_rate=48000:duration=4:samples_per_frame=1024";
    let tone = match encode_ac3(tone_src, "640k", &tone_path) {
        Some(b) => b,
        None => {
            eprintln!("ffmpeg single-tone encode failed — skipping");
            return;
        }
    };

    let pcm = decode_ac3(&tone);
    assert!(!pcm.is_empty(), "single-tone decode produced no samples");

    let (railed, total) = full_scale_stats(&pcm);
    let frac = railed as f64 / total as f64;
    assert!(
        frac < 1e-3,
        "single-tone control railed: {railed}/{total} = {frac:.5} full-scale samples"
    );

    let _ = std::fs::remove_file(&tone_path);
}
