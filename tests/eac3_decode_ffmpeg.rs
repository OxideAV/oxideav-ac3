//! E-AC-3 **decode**-side conformance vs ffmpeg (issue #13 class).
//!
//! The historical E-AC-3 ffmpeg tests only validated the ENCODER
//! (our encode → ffmpeg decode). The issue-#13 broadcast report showed
//! real streams decoding uncorrelated with full-second zero-fill
//! dropouts, so this suite closes the decode-side gap from two
//! directions, with ffmpeg as a black-box producer / reference:
//!
//! 1. `generated_*` — ffmpeg's own E-AC-3 encoder produces elementary
//!    streams (several rates / channel modes / coupling configs /
//!    BSI metadata shapes) and we require our decode of those bytes to
//!    match ffmpeg's decode of the same bytes at ≥ 50 dB per channel.
//!    This exercises a *foreign* encoder's bit patterns end to end —
//!    exactly what the synthetic encoder round-trips never covered.
//! 2. `crafted_*` — our own encoder emits broadcast-shaped BSI/audfrm
//!    configurations that ffmpeg's encoder never produces (blk-0
//!    implicit SPX strategy, `compre = 1`, mixing + informational
//!    metadata blocks, per-block `dynrng`, AHT), and ffmpeg's DECODER
//!    is the reference for the resulting bytes.
//! 3. `local_eac3_vector_matches_ffmpeg` — env-gated hook: point
//!    `OXIDEAV_AC3_LOCAL_EAC3` at any local E-AC-3 elementary stream
//!    (e.g. a captured broadcast vector that must never be committed)
//!    and the same decode-vs-ffmpeg PSNR gate runs on it.
//!
//! Every test is skip-gated on ffmpeg availability, mirroring the
//! existing `eac3_ffmpeg.rs` policy.

use std::process::Command;

use oxideav_ac3::eac3::decoder::{decode_eac3_packet, Eac3DecoderState};

const PSNR_FLOOR_DB: f64 = 50.0;

fn ffmpeg_present() -> bool {
    Command::new("ffmpeg")
        .args(["-version"])
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

fn tmp_path(tag: &str, ext: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!(
        "oxideav_eac3_decode_{}_{}.{ext}",
        tag,
        std::process::id()
    ))
}

/// Encode a lavfi-generated source to an E-AC-3 elementary stream via
/// ffmpeg (black-box producer). Returns the raw stream bytes.
fn ffmpeg_generate(tag: &str, lavfi: &str, extra: &[&str]) -> Option<Vec<u8>> {
    let out = tmp_path(tag, "eac3");
    let mut cmd = Command::new("ffmpeg");
    cmd.args([
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-f",
        "lavfi",
        "-i",
        lavfi,
    ]);
    cmd.args(extra);
    cmd.args(["-c:a", "eac3"]);
    cmd.arg(&out);
    let st = cmd.output().ok()?;
    if !st.status.success() {
        eprintln!(
            "ffmpeg generate failed for {tag}: {}",
            String::from_utf8_lossy(&st.stderr)
        );
        return None;
    }
    let bytes = std::fs::read(&out).ok()?;
    let _ = std::fs::remove_file(&out);
    Some(bytes)
}

/// Reference decode of an E-AC-3 elementary stream via ffmpeg → s16le.
fn ffmpeg_decode(tag: &str, es: &[u8], channels: usize, rate: u32) -> Option<Vec<i16>> {
    let inp = tmp_path(&format!("{tag}_ref_in"), "eac3");
    let out = tmp_path(&format!("{tag}_ref_out"), "pcm");
    std::fs::write(&inp, es).ok()?;
    let st = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error", "-i"])
        .arg(&inp)
        .args([
            "-f",
            "s16le",
            "-acodec",
            "pcm_s16le",
            "-ar",
            &rate.to_string(),
            "-ac",
            &channels.to_string(),
        ])
        .arg(&out)
        .output()
        .ok()?;
    let _ = std::fs::remove_file(&inp);
    if !st.status.success() {
        eprintln!(
            "ffmpeg decode failed for {tag}: {}",
            String::from_utf8_lossy(&st.stderr)
        );
        return None;
    }
    let bytes = std::fs::read(&out).ok()?;
    let _ = std::fs::remove_file(&out);
    Some(
        bytes
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect(),
    )
}

/// Walk `data` syncframe by syncframe (E-AC-3 `frmsiz` sizing),
/// grouping an independent substream with any dependent substreams
/// that follow it into one packet for `decode_eac3_packet`.
/// Returns (interleaved s16, channels, sample_rate, zero_filled).
fn our_decode(data: &[u8]) -> Option<(Vec<i16>, usize, u32, u64)> {
    let mut frames: Vec<(usize, usize)> = Vec::new();
    let mut off = 0usize;
    while off + 6 <= data.len() {
        if data[off] != 0x0B || data[off + 1] != 0x77 {
            off += 1;
            continue;
        }
        let frmsiz = (((data[off + 2] & 0x07) as u32) << 8) | data[off + 3] as u32;
        let flen = ((frmsiz + 1) * 2) as usize;
        if flen < 6 || off + flen > data.len() {
            break;
        }
        frames.push((off, flen));
        off += flen;
    }
    if frames.is_empty() {
        return None;
    }
    // Group indep + following dep substreams into packets.
    let strmtyp = |o: usize| data[o + 2] >> 6;
    let mut packets: Vec<(usize, usize)> = Vec::new();
    let mut i = 0usize;
    while i < frames.len() {
        let (start, mut len) = frames[i];
        let mut j = i + 1;
        while j < frames.len() && strmtyp(frames[j].0) == 1 {
            len += frames[j].1;
            j += 1;
        }
        packets.push((start, len));
        i = j;
    }
    let mut state = Eac3DecoderState::default();
    let mut pcm: Vec<i16> = Vec::new();
    let mut channels = 0usize;
    let mut rate = 0u32;
    for &(start, len) in &packets {
        match decode_eac3_packet(&mut state, &data[start..start + len]) {
            Ok(f) => {
                channels = f.channels as usize;
                rate = f.sample_rate;
                let mut frame_pcm: Vec<i16> = f
                    .pcm_s16le
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect();
                // `decode_eac3_packet` emits the natural bitstream
                // order (Table 5.8 fbw order + LFE last). ffmpeg's
                // s16 output uses WAV order; remap the 3/2+LFE case
                // (L C R Ls Rs LFE → FL FR FC LFE SL SR) so the
                // per-channel comparison lines up.
                if channels == 6 {
                    const MAP: [usize; 6] = [0, 2, 1, 5, 3, 4];
                    frame_pcm = frame_pcm
                        .chunks_exact(6)
                        .flat_map(|fr| MAP.map(|i| fr[i]))
                        .collect();
                }
                pcm.extend(frame_pcm);
            }
            Err(e) => {
                eprintln!("our decode error at offset {start}: {e}");
                return None;
            }
        }
    }
    Some((pcm, channels, rate, state.frames_zero_filled))
}

/// Full-window cross-correlation lag search (the round-14 form: no
/// window cap, so periodic content can't alias the lag) on channel 0,
/// then per-channel PSNR at the winning lag.
fn per_channel_psnr(ours: &[i16], theirs: &[i16], channels: usize) -> Option<(i32, Vec<f64>)> {
    let n_ours = ours.len() / channels;
    let n_ref = theirs.len() / channels;
    let skip = 1536usize; // decoder priming + encoder delay region
    let usable = n_ours.min(n_ref).checked_sub(skip + 512)?;
    if usable < 4096 {
        return None;
    }
    let mut best_lag = 0i32;
    let mut best_sse = f64::INFINITY;
    for lag in -512i32..=512 {
        let mut sse = 0.0f64;
        let mut count = 0u64;
        for i in 0..usable {
            let a_idx = (skip + i) as i32;
            let b_idx = a_idx + lag;
            if b_idx < 0 || b_idx as usize >= n_ref {
                continue;
            }
            let a = ours[a_idx as usize * channels] as f64;
            let b = theirs[b_idx as usize * channels] as f64;
            sse += (a - b) * (a - b);
            count += 1;
        }
        if count > 0 {
            let norm = sse / count as f64;
            if norm < best_sse {
                best_sse = norm;
                best_lag = lag;
            }
        }
    }
    let mut psnrs = Vec::with_capacity(channels);
    for ch in 0..channels {
        let mut sse = 0.0f64;
        let mut count = 0u64;
        for i in 0..usable {
            let a_idx = (skip + i) as i32;
            let b_idx = a_idx + best_lag;
            if b_idx < 0 || b_idx as usize >= n_ref {
                continue;
            }
            let a = ours[a_idx as usize * channels + ch] as f64;
            let b = theirs[b_idx as usize * channels + ch] as f64;
            sse += (a - b) * (a - b);
            count += 1;
        }
        let mse = if count > 0 { sse / count as f64 } else { 0.0 };
        let psnr = if mse <= f64::EPSILON {
            120.0
        } else {
            10.0 * (32767.0f64 * 32767.0 / mse).log10()
        };
        psnrs.push(psnr);
    }
    Some((best_lag, psnrs))
}

/// Run the decode-vs-ffmpeg gate on raw E-AC-3 elementary stream bytes.
fn assert_decode_matches_ffmpeg(tag: &str, es: &[u8], want_channels: usize, want_rate: u32) {
    assert_decode_matches_ffmpeg_floor(tag, es, want_channels, want_rate, PSNR_FLOOR_DB);
}

/// Same gate with an explicit per-stream PSNR floor (used by the
/// dense-noise config where decoder-discretionary dither bounds the
/// achievable agreement).
fn assert_decode_matches_ffmpeg_floor(
    tag: &str,
    es: &[u8],
    want_channels: usize,
    want_rate: u32,
    floor_db: f64,
) {
    let (ours, channels, rate, zero_filled) =
        our_decode(es).unwrap_or_else(|| panic!("{tag}: our decoder failed on the stream"));
    assert_eq!(channels, want_channels, "{tag}: channel count");
    assert_eq!(rate, want_rate, "{tag}: sample rate");
    assert_eq!(
        zero_filled, 0,
        "{tag}: decoder zero-filled {zero_filled} frames (the issue-#13 dropout symptom)"
    );
    let Some(reference) = ffmpeg_decode(tag, es, channels, rate) else {
        panic!("{tag}: ffmpeg failed to decode the stream");
    };
    let (lag, psnrs) = per_channel_psnr(&ours, &reference, channels)
        .unwrap_or_else(|| panic!("{tag}: not enough overlapping samples to compare"));
    eprintln!("{tag}: lag={lag} per-channel PSNR vs ffmpeg: {psnrs:.2?}");
    for (ch, p) in psnrs.iter().enumerate() {
        assert!(
            *p >= floor_db,
            "{tag}: ch{ch} PSNR {p:.2} dB below the {floor_db} dB floor (lag {lag})"
        );
    }
}

// ---------------------------------------------------------------------------
// 1. ffmpeg-generated streams (foreign encoder → our decoder)
// ---------------------------------------------------------------------------

const MULTITONE_STEREO: &str = "aevalsrc=0.35*sin(2*PI*440*t)+0.2*sin(2*PI*3200*t)+0.1*sin(2*PI*9000*t)|0.35*sin(2*PI*554*t)+0.2*sin(2*PI*4100*t)+0.1*sin(2*PI*11000*t):s=48000:d=3";

#[test]
fn generated_stereo_128k_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    let Some(es) = ffmpeg_generate("st128", MULTITONE_STEREO, &["-b:a", "128k"]) else {
        eprintln!("ffmpeg could not generate — skipping");
        return;
    };
    assert_decode_matches_ffmpeg("generated stereo 128k", &es, 2, 48000);
}

#[test]
fn generated_stereo_noise_192k_with_metadata_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    // Pink noise (dense spectrum) + every BSI metadata knob ffmpeg's
    // encoder exposes: infomdate (dsur/original/copyright/mixing_level/
    // room_type) + mixmdate (dmix mode + Lt/Rt / Lo/Ro levels) +
    // non-default dialnorm. This is the broadcast-BSI shape from a
    // foreign producer.
    let Some(es) = ffmpeg_generate(
        "noise192meta",
        "anoisesrc=color=pink:seed=7:r=48000:d=3,volume=0.25,aformat=channel_layouts=stereo",
        &[
            "-b:a",
            "192k",
            "-dialnorm",
            "-24",
            "-dsur_mode",
            "on",
            "-original",
            "1",
            "-copyright",
            "1",
            "-mixing_level",
            "80",
            "-room_type",
            "large",
            "-dmix_mode",
            "ltrt",
            "-ltrt_cmixlev",
            "0.707",
            "-loro_cmixlev",
            "0.707",
        ],
    ) else {
        eprintln!("ffmpeg could not generate — skipping");
        return;
    };
    // Dense-noise content leaves many masked bap-0 bins to DITHER,
    // and the §7.3.4 dither sequence is decoder-discretionary — our
    // LFSR differs from the reference's, so those bins are
    // uncorrelated noise-in-noise at roughly −47 dB. Structural
    // decode correctness still gates: 40 dB is unreachable when any
    // bit-accounting drift corrupts the frame (the pre-fix issue-#13
    // state measured < 1 dB here).
    assert_decode_matches_ffmpeg_floor("generated noise 192k + metadata", &es, 2, 48000, 40.0);
}

#[test]
fn generated_stereo_coupling_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    for (rate, band) in [("96k", "1"), ("64k", "3")] {
        let Some(es) = ffmpeg_generate(
            &format!("cpl{rate}"),
            MULTITONE_STEREO,
            &[
                "-b:a",
                rate,
                "-channel_coupling",
                "1",
                "-cpl_start_band",
                band,
            ],
        ) else {
            eprintln!("ffmpeg could not generate — skipping");
            return;
        };
        assert_decode_matches_ffmpeg(
            &format!("generated stereo {rate} coupling band {band}"),
            &es,
            2,
            48000,
        );
    }
}

#[test]
fn generated_mono_96k_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    let Some(es) = ffmpeg_generate(
        "mono96",
        "aevalsrc=0.35*sin(2*PI*440*t)+0.2*sin(2*PI*2500*t):s=48000:d=3",
        &["-b:a", "96k"],
    ) else {
        eprintln!("ffmpeg could not generate — skipping");
        return;
    };
    assert_decode_matches_ffmpeg("generated mono 96k", &es, 1, 48000);
}

#[test]
fn generated_51_384k_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    let lavfi = "aevalsrc=0.3*sin(2*PI*440*t)|0.3*sin(2*PI*554*t)|0.3*sin(2*PI*659*t)|0.2*sin(2*PI*80*t)|0.25*sin(2*PI*880*t)|0.25*sin(2*PI*1108*t):c=5.1:s=48000:d=3";
    let Some(es) = ffmpeg_generate("s51", lavfi, &["-b:a", "384k", "-channel_coupling", "1"])
    else {
        eprintln!("ffmpeg could not generate — skipping");
        return;
    };
    assert_decode_matches_ffmpeg("generated 5.1 384k coupling", &es, 6, 48000);
}

#[test]
fn generated_stereo_44100_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    let Some(es) = ffmpeg_generate(
        "st441",
        "aevalsrc=0.35*sin(2*PI*440*t)+0.15*sin(2*PI*5000*t)|0.35*sin(2*PI*554*t)+0.15*sin(2*PI*6000*t):s=44100:d=3",
        &["-b:a", "128k"],
    ) else {
        eprintln!("ffmpeg could not generate — skipping");
        return;
    };
    assert_decode_matches_ffmpeg("generated stereo 44.1kHz 128k", &es, 2, 44100);
}

// ---------------------------------------------------------------------------
// 2. Our encoder as the producer of broadcast BSI shapes ffmpeg's
//    encoder never emits (blk-0 implicit SPX, compre, mix/info metadata
//    blocks, per-block dynrng, AHT) — ffmpeg's DECODER is the oracle.
// ---------------------------------------------------------------------------

fn encode_ours(channels: usize, bit_rate: u64, opts: &[(&str, &str)]) -> Vec<u8> {
    use oxideav_core::{AudioFrame, CodecId, CodecParameters, Error, Frame, SampleFormat};
    let mut params = CodecParameters::audio(CodecId::new(oxideav_ac3::CODEC_ID_STR_EAC3));
    params.sample_rate = Some(48000);
    params.channels = Some(channels as u16);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(bit_rate);
    for (k, v) in opts {
        params.options.insert((*k).to_string(), (*v).to_string());
    }
    let mut enc = oxideav_ac3::eac3::make_encoder(&params).expect("make_encoder");

    // 3 s multitone with per-channel detune so coupling/SPX have work.
    let n = 48000 * 3;
    let mut s16 = Vec::with_capacity(n * channels * 2);
    for i in 0..n {
        let t = i as f32 / 48000.0;
        for ch in 0..channels {
            let f0 = 440.0 * (1.0 + 0.05 * ch as f32);
            let v = 0.3 * (2.0 * std::f32::consts::PI * f0 * t).sin()
                + 0.15 * (2.0 * std::f32::consts::PI * (2500.0 + 200.0 * ch as f32) * t).sin()
                + 0.08 * (2.0 * std::f32::consts::PI * (9000.0 + 500.0 * ch as f32) * t).sin();
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
            Ok(pkt) => out.extend_from_slice(&pkt.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("receive_packet: {e:?}"),
        }
    }
    out
}

#[test]
fn crafted_spx_compre_metadata_stereo_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    // Broadcast shape: blk-0 implicit SPX on, compre = 1 (8-bit compr),
    // per-block dynrng, informational metadata block present.
    let es = encode_ours(
        2,
        128_000,
        &[
            ("spx", "1"),
            ("compr", "0x0b"),
            ("dialnorm", "24"),
            ("dynrng", "0xc8"),
            ("bsmod", "0"),
            ("copyright", "1"),
            ("origbs", "1"),
            ("dsurmod", "2"),
            ("mixlevel", "25"),
            ("roomtyp", "1"),
        ],
    );
    assert_decode_matches_ffmpeg("crafted SPX+compre+infomd stereo", &es, 2, 48000);
}

#[test]
fn crafted_spx_low_begf_stereo_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    // Low SPX begin frequencies (spxbegf 0 and 1) — the geometry the
    // issue-#13 class cares about (a co-active coupling region would
    // have a NEGATIVE derived cplendf here).
    for begf in ["0", "1"] {
        let es = encode_ours(
            2,
            96_000,
            &[("spx", "1"), ("spx_begf", begf), ("spx_endf", "4")],
        );
        assert_decode_matches_ffmpeg(&format!("crafted SPX begf={begf} stereo"), &es, 2, 48000);
    }
}

#[test]
fn crafted_spx_mixmd_51_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    // 5.1 with the mixing-metadata block (dmixmod + mix levels + LFE
    // mix level + program scale) and SPX — a Table E1.2 shape close to
    // broadcast practice.
    let es = encode_ours(
        6,
        256_000,
        &[
            ("spx", "1"),
            ("compr", "0x10"),
            ("dmixmod", "1"),
            ("ltrtcmixlev", "4"),
            ("lorocmixlev", "4"),
            ("ltrtsurmixlev", "5"),
            ("lorosurmixlev", "5"),
            ("lfemixlevcod", "10"),
            ("pgmscl", "51"),
        ],
    );
    assert_decode_matches_ffmpeg("crafted SPX+mixmd 5.1", &es, 6, 48000);
}

#[test]
fn crafted_aht_compre_stereo_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    let es = encode_ours(2, 192_000, &[("aht", "1"), ("compr", "0x0b")]);
    assert_decode_matches_ffmpeg("crafted AHT+compre stereo", &es, 2, 48000);
}

// ---------------------------------------------------------------------------
// 3. Env-gated local vector (e.g. the unpublishable issue-#13 capture)
// ---------------------------------------------------------------------------

/// Set `OXIDEAV_AC3_LOCAL_EAC3=/path/to/stream.eac3` to run the same
/// decode-vs-ffmpeg PSNR gate on a local elementary stream that cannot
/// be committed (broadcast captures). Skips silently when unset.
#[test]
fn local_eac3_vector_matches_ffmpeg() {
    let Ok(path) = std::env::var("OXIDEAV_AC3_LOCAL_EAC3") else {
        eprintln!("OXIDEAV_AC3_LOCAL_EAC3 unset — skipping local vector gate");
        return;
    };
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    let es = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let (ours, channels, rate, zero_filled) =
        our_decode(&es).unwrap_or_else(|| panic!("local vector: our decoder failed"));
    assert_eq!(
        zero_filled, 0,
        "local vector: decoder zero-filled {zero_filled} frames (dropouts)"
    );
    let Some(reference) = ffmpeg_decode("local", &es, channels, rate) else {
        panic!("local vector: ffmpeg failed to decode");
    };
    let (lag, psnrs) = per_channel_psnr(&ours, &reference, channels)
        .expect("local vector: not enough overlap to compare");
    eprintln!("local vector: lag={lag} per-channel PSNR vs ffmpeg: {psnrs:.2?}");
    for (ch, p) in psnrs.iter().enumerate() {
        assert!(
            *p >= PSNR_FLOOR_DB,
            "local vector: ch{ch} PSNR {p:.2} dB below {PSNR_FLOOR_DB} dB (lag {lag})"
        );
    }
}
