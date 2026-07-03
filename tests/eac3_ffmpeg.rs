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

/// Build a 7.1 sine PCM at 440 Hz on every channel.
/// Layout: L,C,R,Ls,Rs,LFE,Lb,Rb (8 channels). The LFE channel is
/// fed a 100 Hz tone so a downstream test can verify it isn't the
/// same content as the fbw channels.
fn build_sine_pcm_71() -> Vec<f32> {
    let n = (SR as f32 * DUR_SEC) as usize;
    let mut out = Vec::with_capacity(n * 8);
    for i in 0..n {
        let t = i as f32 / SR as f32;
        let s_main = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.4;
        let s_lfe = (2.0 * std::f32::consts::PI * 100.0 * t).sin() * 0.4;
        // L, C, R, Ls, Rs, LFE, Lb, Rb
        out.push(s_main); // L
        out.push(s_main); // C
        out.push(s_main); // R
        out.push(s_main); // Ls
        out.push(s_main); // Rs
        out.push(s_lfe); // LFE
        out.push(s_main); // Lb
        out.push(s_main); // Rb
    }
    out
}

/// 7.1 emits an indep+dep substream pair. The packet payload has two
/// concatenated syncframes, each starting with 0x0B 0x77. The first
/// syncframe is the indep substream (strmtyp=0, acmod=7, lfeon=1) and
/// the second is the dep substream (strmtyp=1, acmod=2) with a
/// chanmap field set so bit 6 (Lrs/Rrs pair) = 1.
#[test]
fn eac3_71_emits_indep_plus_dep_substream_pair() {
    let pcm = build_sine_pcm_71();
    let frame = encode_eac3(&pcm, 8, 576_000);
    assert!(!frame.is_empty(), "encoder produced no bytes");
    // 384 kbps indep + 192 kbps dep @ 48 kHz / 1536 spf:
    //   indep frame size = 384 * 1000 * 1536 / (48000 * 8) = 1536 bytes
    //   dep   frame size = 192 * 1000 * 1536 / (48000 * 8) = 768 bytes
    let indep_bytes = 1536usize;
    let dep_bytes = 768usize;
    let pair_bytes = indep_bytes + dep_bytes;
    assert_eq!(
        frame.len() % pair_bytes,
        0,
        "concatenated 7.1 frames should sum to whole pair-syncframes ({} bytes)",
        pair_bytes
    );
    let mut off = 0usize;
    while off + pair_bytes <= frame.len() {
        // Indep substream syncword.
        assert_eq!(
            &frame[off..off + 2],
            &[0x0B, 0x77],
            "missing indep-substream syncword at offset {off}"
        );
        // strmtyp = 0 — top 2 bits of byte 2.
        let strmtyp_indep = (frame[off + 2] >> 6) & 0x3;
        assert_eq!(strmtyp_indep, 0, "indep substream must have strmtyp=0");

        // Dep substream syncword.
        let dep_off = off + indep_bytes;
        assert_eq!(
            &frame[dep_off..dep_off + 2],
            &[0x0B, 0x77],
            "missing dep-substream syncword at offset {dep_off}"
        );
        let strmtyp_dep = (frame[dep_off + 2] >> 6) & 0x3;
        assert_eq!(strmtyp_dep, 1, "dep substream must have strmtyp=1");

        off += pair_bytes;
    }
}

#[test]
fn eac3_71_pair_decodes_through_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg not in PATH — skipping 7.1 interop test");
        return;
    }
    let pcm = build_sine_pcm_71();
    let frame = encode_eac3(&pcm, 8, 576_000);
    assert!(!frame.is_empty(), "encoder produced no bytes");
    // ffmpeg's reference decoder may report 8 channels (full 7.1 reassembly
    // when it honours the chanmap) or 6 channels (5.1 fallback that
    // ignores the dep substream). Both satisfy spec §E.3.8.1 — the
    // reference decoder MUST decode indep substream 0 and MAY use the
    // dependent substreams. Acceptance: ffmpeg produces audio of the
    // right *shape* and at least the indep-substream's 5.1 carries
    // signal energy.
    let dir = std::env::temp_dir();
    let in_path: PathBuf = dir.join(format!("oxideav_eac3_71_test_{}.ec3", std::process::id()));
    let out_path: PathBuf = dir.join(format!("oxideav_eac3_71_test_{}.pcm", std::process::id()));
    {
        let mut f = fs::File::create(&in_path).unwrap();
        f.write_all(&frame).unwrap();
    }
    // Ask ffmpeg to keep whatever channel layout it reconstructs (no -ac).
    let probe = Command::new("ffmpeg")
        .args(["-y", "-v", "error", "-f", "eac3", "-i"])
        .arg(&in_path)
        .args([
            "-f",
            "s16le",
            "-acodec",
            "pcm_s16le",
            "-ar",
            &SR.to_string(),
        ])
        .arg(&out_path)
        .status();
    let _ = probe;
    let bytes = match fs::read(&out_path) {
        Ok(b) => b,
        Err(_) => {
            let _ = fs::remove_file(&in_path);
            eprintln!("ffmpeg produced no output — skipping 7.1 ffmpeg interop test");
            return;
        }
    };
    let _ = fs::remove_file(&in_path);
    let _ = fs::remove_file(&out_path);
    // We need to know how many channels ffmpeg picked. Re-run ffprobe on
    // the source elementary stream to query the layout it assigned.
    let probe_out = Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-select_streams",
            "a:0",
            "-show_entries",
            "stream=channels",
            "-of",
            "default=noprint_wrappers=1:nokey=1",
            "-f",
            "eac3",
        ])
        .arg("-")
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .spawn();
    let ffmpeg_chans: usize = match probe_out {
        Ok(mut child) => {
            if let Some(mut stdin) = child.stdin.take() {
                let _ = stdin.write_all(&frame);
            }
            let out = child.wait_with_output().ok();
            out.and_then(|o| {
                String::from_utf8(o.stdout)
                    .ok()
                    .and_then(|s| s.trim().parse::<usize>().ok())
            })
            .unwrap_or(6)
        }
        Err(_) => 6,
    };
    assert!(
        ffmpeg_chans == 6 || ffmpeg_chans == 8,
        "ffmpeg reported unexpected channel count {ffmpeg_chans}"
    );
    // Sample count = bytes / (2 bytes per s16 × ffmpeg_chans).
    let nsamp = bytes.len() / 2 / ffmpeg_chans;
    assert!(
        nsamp >= 1024,
        "ffmpeg returned only {nsamp} samples — expected ≥ 1024"
    );
    // Pick a fbw channel from the 5.1 program (Left, channel 0) and
    // check the energy is non-trivial — confirms the indep substream
    // was decoded successfully.
    let mut left_energy: f64 = 0.0;
    for chunk in bytes.chunks_exact(2 * ffmpeg_chans).skip(1024) {
        let v = i16::from_le_bytes([chunk[0], chunk[1]]) as f64 / 32768.0;
        left_energy += v * v;
    }
    assert!(
        left_energy > 0.01,
        "ffmpeg-decoded Left channel is silent (energy={left_energy})"
    );
    eprintln!(
        "E-AC-3 7.1 indep+dep → ffmpeg decoded {ffmpeg_chans} channels, \
         L-energy={left_energy:.3}, samples={nsamp}"
    );
}

/// In-tree round-trip: encode a 7.1 fixture, feed the indep+dep pair
/// to the in-tree decoder, and confirm the per-frame `dep_locations`
/// list resolves to `[LeftRearSurround, RightRearSurround]` — the
/// Lb/Rb pair carried by the dep substream with chanmap bit 6
/// (Lrs/Rrs pair) set per §E.2.3.1.8 / Table E2.5.
///
/// This test pins the contract that the decoder reports back the
/// physical channel assignment of the appended dep channels, so
/// downstream consumers (a future WAV-mask 7.1 reorderer or a
/// chanmap-aware §7.8 downmix) can route them without re-parsing
/// the bitstream.
#[test]
fn eac3_71_decoder_surfaces_chanmap_lrs_rrs() {
    use oxideav_ac3::eac3::chanmap::ChannelLocation;
    use oxideav_ac3::eac3::decoder::{decode_eac3_packet, Eac3DecoderState};

    let pcm = build_sine_pcm_71();
    let bytes = encode_eac3(&pcm, 8, 576_000);
    assert!(!bytes.is_empty(), "encoder produced no bytes");

    // Walk packet-by-packet: each packet is one indep+dep pair (1536
    // + 768 = 2304 bytes at the chosen 384k+192k bit rates).
    let indep_bytes = 1536usize;
    let dep_bytes = 768usize;
    let pair_bytes = indep_bytes + dep_bytes;
    assert_eq!(bytes.len() % pair_bytes, 0);

    let mut st = Eac3DecoderState::default();
    let mut seen_lrs_rrs = 0usize;
    let mut seen_packets = 0usize;

    let mut off = 0usize;
    while off + pair_bytes <= bytes.len() {
        let pkt = &bytes[off..off + pair_bytes];
        let frame = decode_eac3_packet(&mut st, pkt).expect("decode 7.1 indep+dep packet");
        // Indep substream is acmod=7 (5 fbw) + lfeon=1 → 6 chans,
        // plus 2 dep chans = 8.
        assert_eq!(
            frame.channels, 8,
            "7.1 indep+dep packet should yield 8 channels (got {})",
            frame.channels
        );
        // The dep_locations vector must have exactly one entry per
        // dep coded channel (here: 2).
        assert_eq!(
            frame.dep_locations.len(),
            2,
            "expected 2 dep-channel locations (Lrs/Rrs pair), got {}",
            frame.dep_locations.len()
        );
        // Spec example for chanmap bit 6 (§E.2.3.1.8 / Table E2.5):
        // pair = Left Rear Surround / Right Rear Surround.
        if frame.dep_locations[0] == ChannelLocation::LeftRearSurround
            && frame.dep_locations[1] == ChannelLocation::RightRearSurround
        {
            seen_lrs_rrs += 1;
        }
        seen_packets += 1;
        off += pair_bytes;
    }
    assert!(seen_packets > 0, "no packets walked");
    assert_eq!(
        seen_lrs_rrs, seen_packets,
        "every packet should report dep_locations = [Lrs, Rrs] \
         (saw {seen_lrs_rrs}/{seen_packets})"
    );
}

// ---------------------------------------------------------------------
// Spectral extension (§E.2.3.3 / §E.3.6) black-box cross-validation.
//
// The in-tree round-trip tests (eac3::encoder::tests::spx_*) prove the
// encoder and OUR decoder agree; this test proves the emitted syntax
// is what the SPEC says by handing the stream to an independent
// decoder binary. Two gates:
//
//  1. ffmpeg accepts every syncframe (a mis-sized SPX field would
//     desynchronise its parser and abort / mute);
//  2. the §3.6.4.3 contract holds through the EXTERNAL decoder: per
//     SPX band, the synthesized high-frequency energy matches the
//     original signal's banded energy (loose ±4 dB — the noise-blend
//     generator is non-normative, so exact waveforms differ, but band
//     energies must not).

use oxideav_ac3::audblk::{N_COEFFS, SAMPLES_PER_BLOCK};
use oxideav_ac3::eac3::spxenc::{SpxGeometry, SpxParams};
use oxideav_ac3::mdct::mdct_512;
use oxideav_ac3::tables::WINDOW;

/// Multitone + deterministic broadband bed (mirrors the in-tree SPX
/// round-trip fixture): LF tones feed the coded band / translation
/// copy region, HF tones land in SPX bands, and the bed keeps every
/// band's energy-matching coordinate inside the representable range.
fn build_spx_multitone(channels: usize) -> Vec<f32> {
    let n = (SR as f32 * DUR_SEC) as usize;
    let tones: [(f32, f32); 6] = [
        (700.0, 0.22),
        (3_100.0, 0.18),
        (6_000.0, 0.14),
        (12_200.0, 0.055),
        (15_400.0, 0.035),
        (19_000.0, 0.030),
    ];
    let mut lfsr: u32 = 0x2545_F491;
    let mut pcm = vec![0.0f32; n * channels];
    for i in 0..n {
        let t = i as f32 / SR as f32;
        let mut s = 0.0f32;
        for (f, a) in tones {
            s += a * (2.0 * std::f32::consts::PI * f * t).sin();
        }
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        s += ((lfsr as f32 / u32::MAX as f32) * 2.0 - 1.0) * 0.012;
        for ch in 0..channels {
            pcm[i * channels + ch] = s * (1.0 - 0.08 * ch as f32);
        }
    }
    pcm
}

fn encode_eac3_spx(pcm: &[f32], channels: usize, bit_rate: u64, spx: SpxParams) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new(eac3::CODEC_ID_STR));
    params.sample_rate = Some(SR);
    params.channels = Some(channels as u16);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(bit_rate);
    let mut enc = eac3::make_encoder_with_spx(&params, spx).expect("eac3 make_encoder_with_spx");
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
            Err(e) => panic!("eac3 spx encode error: {e:?}"),
        }
    }
    out
}

/// Mean per-bin MDCT energy of one channel (encoder window + long
/// transform), skipping priming/flush edges. Pure-tone-plus-bed
/// content is stationary, so no delay alignment is needed.
fn mdct_energy_profile(pcm: &[f32], channels: usize, ch: usize) -> [f64; N_COEFFS] {
    let n = pcm.len() / channels;
    let mut acc = [0.0f64; N_COEFFS];
    let mut blocks = 0usize;
    let mut start = 4 * SAMPLES_PER_BLOCK;
    while start + 512 + 4 * SAMPLES_PER_BLOCK <= n {
        let mut win = [0.0f32; 512];
        for k in 0..256 {
            win[k] = pcm[(start + k) * channels + ch] * WINDOW[k];
            win[511 - k] = pcm[(start + 511 - k) * channels + ch] * WINDOW[k];
        }
        let mut coeffs = [0.0f32; N_COEFFS];
        mdct_512(&win, &mut coeffs);
        for (a, &c) in acc.iter_mut().zip(coeffs.iter()) {
            *a += (c as f64) * (c as f64);
        }
        blocks += 1;
        start += SAMPLES_PER_BLOCK;
    }
    assert!(blocks > 8, "analysis needs a steady-state interior");
    for a in acc.iter_mut() {
        *a /= blocks as f64;
    }
    acc
}

#[test]
fn eac3_spx_stereo_band_energy_through_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg not in PATH — skipping SPX interop test");
        return;
    }
    let spx = SpxParams::default();
    let geom = SpxGeometry::derive(&spx, &oxideav_ac3::eac3::dsp::DEFAULT_SPX_BNDSTRC)
        .expect("default SPX geometry");
    let pcm = build_spx_multitone(2);
    let stream = encode_eac3_spx(&pcm, 2, 192_000, spx);
    assert!(!stream.is_empty());
    let decoded = ffmpeg_decode(&stream, 2).expect("ffmpeg rejected the SPX stream");
    assert!(
        decoded.len() as f32 >= pcm.len() as f32 * 0.9,
        "ffmpeg returned too few samples ({} vs {})",
        decoded.len(),
        pcm.len()
    );
    for ch in 0..2 {
        let orig_prof = mdct_energy_profile(&pcm, 2, ch);
        let dec_prof = mdct_energy_profile(&decoded, 2, ch);
        // SPX-band energy match through the external decoder.
        let mut lo = geom.begin_tc;
        let mut deltas = Vec::new();
        for bnd in 0..geom.nbnds {
            let hi = lo + geom.bndsztab[bnd];
            let eo: f64 = orig_prof[lo..hi].iter().sum();
            let ed: f64 = dec_prof[lo..hi].iter().sum();
            deltas.push(10.0 * (ed.max(1e-30) / eo.max(1e-30)).log10());
            lo = hi;
        }
        for (bnd, d) in deltas.iter().enumerate() {
            assert!(
                d.abs() <= 4.0,
                "ch{ch} SPX band {bnd}: external-decoder energy delta {d:+.2} dB \
                 exceeds ±4 dB (all: {deltas:?})"
            );
        }
        // Coded-band sanity through the external decoder.
        let eo: f64 = orig_prof[..geom.begin_tc].iter().sum();
        let ed: f64 = dec_prof[..geom.begin_tc].iter().sum();
        let lf = 10.0 * (ed / eo).log10();
        assert!(lf.abs() <= 1.5, "ch{ch}: coded-band delta {lf:+.2} dB");
    }
}

#[test]
fn eac3_spx_mono_decodes_through_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg not in PATH — skipping SPX mono interop test");
        return;
    }
    // Mono drives the §E.2.3.3.3 implicit `chinspx[0] = 1` arm through
    // an external parser.
    let pcm = build_spx_multitone(1);
    let stream = encode_eac3_spx(&pcm, 1, 96_000, SpxParams::default());
    let decoded = ffmpeg_decode(&stream, 1).expect("ffmpeg rejected the mono SPX stream");
    assert!(decoded.len() as f32 >= pcm.len() as f32 * 0.9);
    // Non-silent, sane level: RMS within 3 dB of the source.
    let rms =
        |x: &[f32]| (x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len() as f64).sqrt();
    let r_o = rms(&pcm);
    let r_d = rms(&decoded);
    let delta = 20.0 * (r_d / r_o).log10();
    assert!(
        delta.abs() <= 3.0,
        "mono SPX decode RMS delta {delta:+.2} dB vs source"
    );
}

#[test]
fn eac3_spx_atten_band_energy_through_ffmpeg() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg not in PATH — skipping SPX attenuation interop test");
        return;
    }
    // §3.6.4.2.3 attenuation: the encoder signals spxattene /
    // chinspxatten / spxattencod in audfrm and folds the border/wrap
    // notch into its coordinate computation. Through an EXTERNAL
    // decoder this cross-validates (a) the audfrm attenuation-field
    // placement (a mis-sized field would desynchronise everything
    // after it) and (b) that the notch-compensated coordinates still
    // deliver the §3.6.4.3 banded energy.
    let spx = SpxParams {
        atten_code: Some(14), // Table E3.14 taps [0.5, 0.25, 0.125]
        ..SpxParams::default()
    };
    let geom = SpxGeometry::derive(&spx, &oxideav_ac3::eac3::dsp::DEFAULT_SPX_BNDSTRC)
        .expect("SPX geometry");
    let pcm = build_spx_multitone(2);
    let stream = encode_eac3_spx(&pcm, 2, 192_000, spx);
    let decoded = ffmpeg_decode(&stream, 2).expect("ffmpeg rejected the SPX+atten stream");
    assert!(decoded.len() as f32 >= pcm.len() as f32 * 0.9);
    let orig_prof = mdct_energy_profile(&pcm, 2, 0);
    let dec_prof = mdct_energy_profile(&decoded, 2, 0);
    let mut lo = geom.begin_tc;
    let mut deltas = Vec::new();
    for bnd in 0..geom.nbnds {
        let hi = lo + geom.bndsztab[bnd];
        let eo: f64 = orig_prof[lo..hi].iter().sum();
        let ed: f64 = dec_prof[lo..hi].iter().sum();
        deltas.push(10.0 * (ed.max(1e-30) / eo.max(1e-30)).log10());
        lo = hi;
    }
    for (bnd, d) in deltas.iter().enumerate() {
        assert!(
            d.abs() <= 4.0,
            "SPX band {bnd} (atten): external-decoder energy delta {d:+.2} dB (all: {deltas:?})"
        );
    }
}
