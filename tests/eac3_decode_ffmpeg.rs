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

use oxideav_ac3::eac3::decoder::{Eac3DecoderState, decode_eac3_packet};

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
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect(),
    )
}

/// Walk `data` syncframe by syncframe (E-AC-3 `frmsiz` sizing),
/// grouping an independent substream with any dependent substreams
/// that follow it into one packet for `decode_eac3_packet`.
/// Returns (interleaved s16, channels, sample_rate, zero_filled).
fn our_decode(data: &[u8]) -> Option<(Vec<i16>, usize, u32, u64)> {
    our_decode_seeded(data, None)
}

/// [`our_decode`] with the decoder-private noise generators (§7.3.4
/// dither, §E.3.6.4.2 SPX noise) re-seeded when `seed` is set.
fn our_decode_seeded(data: &[u8], seed: Option<u32>) -> Option<(Vec<i16>, usize, u32, u64)> {
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
    if let Some(seed) = seed {
        state.reseed_noise(seed);
    }
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
                    .as_chunks::<2>()
                    .0
                    .iter()
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
                        .as_chunks::<6>()
                        .0
                        .iter()
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

/// Per-channel PSNR between two of OUR decodes of one stream that
/// differ only in the decoder-private noise seed (sample-aligned, same
/// skip as [`per_channel_psnr`]). The spec leaves the §7.3.4 dither and
/// §E.3.6.4.2 SPX noise sequences to the decoder, so this is the
/// agreement ceiling ANY two conformant decoders can reach on the
/// stream: their difference is two uncorrelated noise fields of the
/// same power, exactly like ours-vs-ours here. 120 dB when the stream
/// carries no decoder-private noise.
fn self_noise_psnr(data: &[u8], ours: &[i16], channels: usize) -> Vec<f64> {
    let (alt, ..) = our_decode_seeded(data, Some(0x9E37_79B9)).expect("reseeded decode");
    let n = ours.len().min(alt.len()) / channels;
    (0..channels)
        .map(|ch| {
            let mut sse = 0.0f64;
            let mut count = 0u64;
            for i in 1536.min(n)..n {
                let d = ours[i * channels + ch] as f64 - alt[i * channels + ch] as f64;
                sse += d * d;
                count += 1;
            }
            let mse = sse / count.max(1) as f64;
            if mse <= f64::EPSILON {
                120.0
            } else {
                10.0 * (32767.0f64 * 32767.0 / mse).log10()
            }
        })
        .collect()
}

/// Margin below the self-noise ceiling a cross-decoder comparison may
/// sit: the two decoders' noise generators need not share our level
/// exactly (§7.3.4 allows a 0.5–0.75 uniform scale) and the noise
/// estimate itself fluctuates. A desynced or tool-mishandled decode
/// lands tens of dB lower, far outside it.
const NOISE_MARGIN_DB: f64 = 4.0;

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
    let ceiling = self_noise_psnr(es, &ours, channels);
    eprintln!(
        "{tag}: lag={lag} per-channel PSNR vs ffmpeg: {psnrs:.2?} (decoder-private noise ceiling {ceiling:.2?})"
    );
    for (ch, p) in psnrs.iter().enumerate() {
        // The fixed floor, lowered only as far as the stream's own
        // decoder-private noise forces it.
        let floor = floor_db.min(ceiling[ch] - NOISE_MARGIN_DB);
        assert!(
            *p >= floor,
            "{tag}: ch{ch} PSNR {p:.2} dB below the {floor:.2} dB floor (lag {lag}, noise ceiling {:.2} dB)",
            ceiling[ch]
        );
    }
    assert_no_segment_dropouts(tag, &ours, &reference, channels, lag);
}

/// Localised-dropout check (the issue-#13 "full-second silence gaps"
/// symptom, which a whole-stream PSNR can average away): in every
/// 1536-sample segment where the reference is above −50 dBFS RMS, our
/// decode's power must be within 6 dB of it.
fn assert_no_segment_dropouts(
    tag: &str,
    ours: &[i16],
    reference: &[i16],
    channels: usize,
    lag: i32,
) {
    const SEG: usize = 1536;
    let n_ours = ours.len() / channels;
    let n_ref = reference.len() / channels;
    let floor_pow = (32767.0f64 * 10f64.powf(-50.0 / 20.0)).powi(2);
    for ch in 0..channels {
        let mut start = SEG;
        while start + SEG <= n_ours {
            let (mut p_o, mut p_r, mut cnt) = (0.0f64, 0.0f64, 0u64);
            for i in start..start + SEG {
                let j = i as i64 + lag as i64;
                if j < 0 || j as usize >= n_ref {
                    continue;
                }
                let o = ours[i * channels + ch] as f64;
                let r = reference[j as usize * channels + ch] as f64;
                p_o += o * o;
                p_r += r * r;
                cnt += 1;
            }
            if cnt == SEG as u64 && p_r / cnt as f64 > floor_pow {
                assert!(
                    p_o >= p_r / 4.0,
                    "{tag}: ch{ch} dropout at sample {start}: ours {:.1} dB below the reference",
                    10.0 * (p_r / p_o.max(1e-9)).log10()
                );
            }
            start += SEG;
        }
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
    encode_ours_at(channels, 48000, 3.0, bit_rate, opts)
}

/// [`encode_ours`] at a chosen sample rate and duration.
fn encode_ours_at(
    channels: usize,
    rate: u32,
    seconds: f32,
    bit_rate: u64,
    opts: &[(&str, &str)],
) -> Vec<u8> {
    // Multitone with per-channel detune so coupling/SPX have work.
    let n = (rate as f32 * seconds) as usize;
    let mut pcm = Vec::with_capacity(n * channels);
    for i in 0..n {
        let t = i as f32 / rate as f32;
        for ch in 0..channels {
            let f0 = 440.0 * (1.0 + 0.05 * ch as f32);
            let v = 0.3 * (2.0 * std::f32::consts::PI * f0 * t).sin()
                + 0.15 * (2.0 * std::f32::consts::PI * (2500.0 + 200.0 * ch as f32) * t).sin()
                + 0.08 * (2.0 * std::f32::consts::PI * (9000.0 + 500.0 * ch as f32) * t).sin();
            pcm.push((v * 32767.0).clamp(-32768.0, 32767.0) as i16);
        }
    }
    encode_ours_pcm(channels, rate, bit_rate, opts, &pcm)
}

/// Encode interleaved s16 PCM with our E-AC-3 encoder.
fn encode_ours_pcm(
    channels: usize,
    rate: u32,
    bit_rate: u64,
    opts: &[(&str, &str)],
    pcm: &[i16],
) -> Vec<u8> {
    use oxideav_core::{AudioFrame, CodecId, CodecParameters, Error, Frame, SampleFormat};
    let mut params = CodecParameters::audio(CodecId::new(oxideav_ac3::CODEC_ID_STR_EAC3));
    params.sample_rate = Some(rate);
    params.channels = Some(channels as u16);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(bit_rate);
    for (k, v) in opts {
        params.options.insert((*k).to_string(), (*v).to_string());
    }
    let mut enc = oxideav_ac3::eac3::make_encoder(&params).expect("make_encoder");
    let n = pcm.len() / channels;
    let s16: Vec<u8> = pcm.iter().flat_map(|v| v.to_le_bytes()).collect();
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
fn crafted_std_coupling_spx_geometry_sweep_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    // The issue-#13 geometry end to end: Annex E standard coupling
    // (`ecplinu = 0`) co-active with SPX, so `cplendf` is DERIVED from
    // spxbegf (Table E1.4) — negative for spxbegf 0/1 — and the
    // coupling region sits between `cplbegf` and the SPX begin. Every
    // spxbegf code, at a broadcast-like stereo rate and on 5.1, plus
    // the lowest legal cplbegf for each (one-sub-band regions).
    for begf in 0..=7u8 {
        let b = begf.to_string();
        let derived_endf: i32 = if begf < 6 {
            begf as i32 - 2
        } else {
            begf as i32 * 2 - 7
        };
        // Narrowest legal region: a single sub-band ending at the SPX
        // begin (cplbegf = cplendf + 2).
        let single = (derived_endf + 2).to_string();
        for (channels, kbps) in [(2usize, 128u64), (6, 384)] {
            // The default cplbegf (min(8, cplendf + 2)) coincides with
            // the single-sub-band region except at spxbegf = 7.
            let regions: &[Option<&str>] = if begf == 7 {
                &[None, Some("0"), Some(single.as_str())]
            } else {
                &[Some("0"), Some(single.as_str())]
            };
            for &cpl_begf in regions {
                let mut opts: Vec<(&str, &str)> = vec![
                    ("cpl", "1"),
                    ("spx", "1"),
                    ("spx_begf", &b),
                    ("spx_blnd", "31"),
                    ("compr", "0x0b"),
                    ("dynrng", "0xc8"),
                ];
                if let Some(c) = cpl_begf {
                    opts.push(("cpl_begf", c));
                }
                let es = encode_ours_at(channels, 48_000, 0.25, kbps * 1000, &opts);
                assert_decode_matches_ffmpeg(
                    &format!(
                        "cpl+spx spxbegf={begf} cplbegf={} {channels}ch {kbps}k",
                        cpl_begf.unwrap_or("default")
                    ),
                    &es,
                    channels,
                    48_000,
                );
            }
        }
    }
}

#[test]
fn crafted_std_coupling_region_sweep_decodes_like_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    // Standard coupling without SPX: transmitted cplendf, the Table
    // E2.12 default banding over every begin/end shape (narrow,
    // single-sub-band, full-width), stereo and 5.1, 48/44.1/32 kHz.
    for (begf, endf) in [
        (0u8, 15u8),
        (2, 12),
        (5, 15),
        (8, 15),
        (11, 10),
        (12, 12),
        (15, 15),
    ] {
        for (channels, kbps, rate) in [
            (2usize, 128u64, 48_000u32),
            (6, 384, 48_000),
            (2, 96, 44_100),
            (2, 96, 32_000),
        ] {
            let (b, e) = (begf.to_string(), endf.to_string());
            let es = encode_ours_at(
                channels,
                rate,
                0.25,
                kbps * 1000,
                // dither = 0: zero-bit mantissas decode as true zeros,
                // so wide low-rate coupling regions compare
                // deterministically (§7.3.4 dither is decoder-private).
                &[
                    ("cpl", "1"),
                    ("cpl_begf", &b),
                    ("cpl_endf", &e),
                    ("dither", "0"),
                ],
            );
            assert_decode_matches_ffmpeg(
                &format!("cpl begf={begf} endf={endf} {channels}ch {kbps}k {rate} Hz"),
                &es,
                channels,
                rate,
            );
        }
    }
}

/// Mean windowed-DFT power per channel over the bins in
/// `[f_lo, f_hi)` Hz (512-sample Hann frames, hop 512), at `lag`.
fn band_power_db(
    pcm: &[i16],
    channels: usize,
    rate: u32,
    lag: i32,
    f_lo: f64,
    f_hi: f64,
) -> Vec<f64> {
    const N: usize = 512;
    let n = pcm.len() / channels;
    let k_lo = (f_lo * N as f64 / rate as f64).ceil() as usize;
    let k_hi = (f_hi * N as f64 / rate as f64).floor() as usize;
    let win: Vec<f64> = (0..N)
        .map(|i| 0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / N as f64).cos())
        .collect();
    let mut out = Vec::with_capacity(channels);
    for ch in 0..channels {
        let mut acc = 0.0f64;
        let mut frames = 0u64;
        let mut start = 2048usize;
        while start + N + 2048 < n {
            for k in k_lo..k_hi {
                let (mut re, mut im) = (0.0f64, 0.0f64);
                for (i, w) in win.iter().enumerate() {
                    let idx = (start + i) as i32 + lag;
                    let v = pcm[idx as usize * channels + ch] as f64 * w;
                    let ph = 2.0 * std::f64::consts::PI * (k * i) as f64 / N as f64;
                    re += v * ph.cos();
                    im -= v * ph.sin();
                }
                acc += re * re + im * im;
            }
            frames += 1;
            start += N;
        }
        out.push(10.0 * (acc / (frames.max(1) as f64 * (k_hi - k_lo) as f64)).log10());
    }
    out
}

/// Stereo program for the coupled-dither gates: a loud low tone per
/// channel plus near-threshold high-band content (48 tones from 10
/// kHz, 12 dB lower in R), one second at 48 kHz.
fn near_threshold_hf_stereo() -> Vec<i16> {
    let rate = 48_000u32;
    let n = rate as usize;
    let mut pcm = Vec::with_capacity(n * 2);
    let mut seed = 0x2468_ace1u32;
    let mut tones: Vec<(f32, f32)> = Vec::new();
    for j in 0..48 {
        seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
        let phase = (seed >> 8) as f32 / (1u32 << 24) as f32 * std::f32::consts::TAU;
        tones.push((10_000.0 + 180.0 * j as f32, phase));
    }
    for i in 0..n {
        let t = i as f32 / rate as f32;
        let hf: f32 = tones
            .iter()
            .map(|&(f, ph)| (std::f32::consts::TAU * f * t + ph).sin())
            .sum::<f32>()
            * 0.000_15;
        for (f0, g) in [(440.0f32, 1.0f32), (523.0, 0.25)] {
            let v = 0.3 * (std::f32::consts::TAU * f0 * t).sin() + g * hf;
            pcm.push((v * 32767.0).round().clamp(-32768.0, 32767.0) as i16);
        }
    }
    pcm
}

/// Assert per-channel 10.5–18 kHz band power of `ours` within 1.5 dB
/// of the ffmpeg decode `reference` (lag-aligned).
fn assert_coupled_band_power_matches(tag: &str, ours: &[i16], reference: &[i16]) {
    let (lag, _) = per_channel_psnr(ours, reference, 2).expect("overlap");
    let p_ours = band_power_db(ours, 2, 48_000, 0, 10_500.0, 18_000.0);
    let p_ref = band_power_db(reference, 2, 48_000, lag, 10_500.0, 18_000.0);
    eprintln!("{tag}: coupled-band power ours {p_ours:.2?} dB, ffmpeg {p_ref:.2?} dB (lag {lag})");
    for ch in 0..2 {
        assert!(
            (p_ours[ch] - p_ref[ch]).abs() < 1.5,
            "{tag} ch{ch}: coupled-band power {:.2} dB vs reference {:.2} dB",
            p_ours[ch],
            p_ref[ch]
        );
    }
}

#[test]
fn coupled_zero_bit_dither_level_matches_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    // §7.3.4: zero-bit (bap = 0) coupling-channel mantissas are
    // replaced by dither AFTER each channel is extracted from the
    // coupling channel, so every coupled channel with dithflag = 1
    // carries its own noise there — at the level the coupling
    // exponent and that channel's coordinate set. The program is
    // coupled from 9.1 kHz at a low rate so most of the coupling
    // channel is allocated zero bits. The dither is decoder-private
    // noise, so the waveforms cannot match; the band POWER must, per
    // channel (a decoder leaving those bins at zero sits 27–39 dB low).
    let pcm = near_threshold_hf_stereo();
    let es = encode_ours_pcm(2, 48_000, 64_000, &[("cpl", "1"), ("cpl_begf", "5")], &pcm);
    let (ours, channels, _, zero_filled) = our_decode(&es).expect("our decode");
    assert_eq!((channels, zero_filled), (2, 0));
    let reference = ffmpeg_decode("cpl dither", &es, 2, 48_000).expect("ffmpeg decode");
    assert_coupled_band_power_matches("E-AC-3 cpl 64k", &ours, &reference);
}

#[test]
fn ac3_coupled_zero_bit_dither_level_matches_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    // The same §7.3.4 gate on base AC-3 (shared decouple path): our
    // AC-3 encoder couples 2/0 from cplbegf 8 by default; at 48 kb/s
    // a decoder leaving the zero-bit coupled bins at zero sits ~5 dB
    // low in the band.
    use oxideav_core::{
        AudioFrame, CodecId, CodecParameters, CodecRegistry, Error, Frame, Packet, SampleFormat,
        TimeBase,
    };
    let pcm = near_threshold_hf_stereo();
    let mut params = CodecParameters::audio(CodecId::new("ac3"));
    params.sample_rate = Some(48_000);
    params.channels = Some(2);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(48_000);
    let mut enc = oxideav_ac3::encoder::make_encoder(&params).expect("ac3 make_encoder");
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: (pcm.len() / 2) as u32,
        pts: Some(0),
        data: vec![pcm.iter().flat_map(|v| v.to_le_bytes()).collect()],
    }))
    .expect("send_frame");
    enc.flush().expect("flush");
    let mut packets = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(p) => packets.push(p.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("receive_packet: {e:?}"),
        }
    }
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    let mut dec = reg.first_decoder(&params).expect("ac3 decoder");
    let mut ours: Vec<i16> = Vec::new();
    for (i, data) in packets.iter().enumerate() {
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), data.clone()).with_pts(i as i64 * 1536);
        dec.send_packet(&pkt).expect("send_packet");
        while let Ok(f) = dec.receive_frame() {
            if let Frame::Audio(a) = f {
                ours.extend(
                    a.data[0]
                        .as_chunks::<2>()
                        .0
                        .iter()
                        .map(|c| i16::from_le_bytes(*c)),
                );
            }
        }
    }
    let es: Vec<u8> = packets.concat();
    let reference = ffmpeg_decode("ac3 cpl dither", &es, 2, 48_000).expect("ffmpeg decode");
    assert_coupled_band_power_matches("AC-3 cpl 48k", &ours, &reference);
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
// 2b. Broadcast-shape matrix — every coding tool our encoder can switch
//     on, at every sample rate, on stereo and 5.1, through ffmpeg's
//     decoder as the oracle. Cells are chosen where the oracle is
//     meaningful: the per-channel budget keeps the §7.3.4 dither floor
//     (bap = 0 bins are filled with decoder-private pseudo-random noise,
//     uncorrelated between implementations) well under the 50 dB gate,
//     SPX runs with `spxblnd = 31` so the §E.3.6.4.2 noise component
//     (likewise a decoder-private generator) is off, and enhanced
//     coupling / dependent substreams are left out because the oracle
//     does not render them (recorded in README + the fixture notes).
// ---------------------------------------------------------------------------

/// The coding-tool axis of the matrix: (tag, encoder options).
const MATRIX_TOOLS: &[(&str, &[(&str, &str)])] = &[
    ("plain", &[]),
    ("cpl", &[("cpl", "1")]),
    ("cpl+spx", &[("cpl", "1"), ("spx", "1"), ("spx_blnd", "31")]),
    ("spx", &[("spx", "1"), ("spx_blnd", "31")]),
    ("aht", &[("aht", "1")]),
    ("tpnp", &[("tpnp", "1")]),
    (
        "spx+tpnp+meta",
        &[
            ("spx", "1"),
            ("spx_blnd", "31"),
            ("tpnp", "1"),
            ("compr", "0x0b"),
            ("dialnorm", "24"),
            ("dynrng", "0xc8"),
            ("dmixmod", "1"),
            ("lfemixlevcod", "10"),
            ("pgmscl", "51"),
            ("mixlevel", "25"),
            ("roomtyp", "1"),
            ("copyright", "1"),
            ("origbs", "1"),
        ],
    ),
    (
        "aht+meta",
        &[
            ("aht", "1"),
            ("compr", "0x10"),
            ("dialnorm", "27"),
            ("dmixmod", "2"),
            ("ltrtcmixlev", "4"),
            ("lorocmixlev", "4"),
            ("ltrtsurmixlev", "5"),
            ("lorosurmixlev", "5"),
            ("lfemixlevcod", "10"),
        ],
    ),
];

/// Run the matrix at one sample rate: stereo 192 kb/s and 5.1 448 kb/s
/// six-block frames across every tool, plus the fractional-frame shapes
/// (`numblkscod` 0..2) the tools cannot combine with.
fn broadcast_matrix_at(rate: u32) {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    for (tag, opts) in MATRIX_TOOLS {
        for (channels, kbps) in [(2usize, 192u64), (6, 448)] {
            let es = encode_ours_at(channels, rate, 0.5, kbps * 1000, opts);
            assert_decode_matches_ffmpeg(
                &format!("matrix {rate} Hz {channels}ch {kbps}k {tag}"),
                &es,
                channels,
                rate,
            );
        }
    }
    for (channels, blocks, kbps, tool) in [
        (2usize, "3", 192u64, "plain"),
        (2, "2", 448, "plain"),
        (2, "1", 448, "tpnp"),
        (6, "3", 448, "plain"),
    ] {
        let opts: &[(&str, &str)] = if tool == "tpnp" {
            &[("blocks", blocks), ("tpnp", "1")]
        } else {
            &[("blocks", blocks)]
        };
        let es = encode_ours_at(channels, rate, 0.5, kbps * 1000, opts);
        assert_decode_matches_ffmpeg(
            &format!("matrix {rate} Hz {channels}ch {blocks}blk {kbps}k {tool}"),
            &es,
            channels,
            rate,
        );
    }
}

#[test]
fn crafted_broadcast_matrix_48k_decodes_like_ffmpeg() {
    broadcast_matrix_at(48_000);
}

#[test]
fn crafted_broadcast_matrix_44k_decodes_like_ffmpeg() {
    broadcast_matrix_at(44_100);
}

#[test]
fn crafted_broadcast_matrix_32k_decodes_like_ffmpeg() {
    broadcast_matrix_at(32_000);
}

// ---------------------------------------------------------------------------
// 3. Env-gated local vector (e.g. the unpublishable issue-#13 capture)
// ---------------------------------------------------------------------------

/// Set `OXIDEAV_AC3_LOCAL_EAC3=/path/to/stream.eac3` to run the same
/// decode-vs-ffmpeg gate on a local elementary stream that cannot be
/// committed (broadcast captures): no zero-filled frames, no localised
/// dropouts, and per-channel PSNR ≥ 50 dB — or, when the stream carries
/// decoder-private noise (§7.3.4 dither, §E.3.6.4.2 SPX noise blending,
/// both left to the decoder by the spec, and both routine in broadcast
/// streams), within 4 dB of the ceiling two conformant decoders can
/// reach on it. Skips silently when unset.
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
    let (_, channels, rate, _) =
        our_decode(&es).unwrap_or_else(|| panic!("local vector: our decoder failed"));
    assert_decode_matches_ffmpeg_floor("local vector", &es, channels, rate, PSNR_FLOOR_DB);
}
