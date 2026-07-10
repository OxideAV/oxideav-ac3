//! Decoder robustness / panic-safety against adversarial input.
//!
//! A decoder must never panic on malformed, truncated, or hostile bytes —
//! a panic in a media decoder is a denial-of-service vector when the bytes
//! come from an untrusted source. This harness drives the AC-3 and E-AC-3
//! decoders through three families of corruption derived deterministically
//! from the real fixture corpus and asserts the process survives every
//! feed (the decoder is free to return `Err`, emit a frame, or emit
//! nothing — it just must not abort).
//!
//! Corruption families:
//!  1. **Truncations** — every prefix length of a real frame, exercising
//!     the bit-reader's end-of-buffer handling at every field boundary.
//!  2. **Bit flips** — deterministic multi-bit flips on a real frame,
//!     reaching field values (coupling sub-band ranges, exponent
//!     strategies, SPX/ecpl geometry) that a valid encoder never emits.
//!  3. **Sync-prefixed garbage** — random payloads tagged with the
//!     `0x0B77` syncword so the parser commits to a frame body that is
//!     pure noise.
//!
//! This pins two specific panics fixed in r373:
//!  * `eac3::ecpl` enhanced-coupling sub-band range — an inverted
//!    `begin..end` slice (`ecplbegf`/`ecplendf` decoding to `begin >= end`)
//!    panicked `necplbnd` / `band_bin_counts` with "slice index starts at
//!    N but ends at M". Now rejected at parse time (§E.2.3.3.16-17).
//!  * `audblk::dsp_block` coupling decouple — a malformed `cpl_nsubbnd`
//!    above the §7.4.2 maximum of 18 sub-bands indexed `sbnd2bnd[18]` out
//!    of a length-18 array. Now clamped to the valid prefix.
//!
//! The test is skipped gracefully (returns success) when the workspace
//! `docs/` corpus is absent (standalone-repo CI checkout).

use std::fs;
use std::path::PathBuf;

use oxideav_ac3::syncinfo;
use oxideav_core::{CodecId, CodecParameters, CodecRegistry, Decoder, Packet, TimeBase};

fn fixtures_root() -> PathBuf {
    PathBuf::from("../../docs/audio/ac3/fixtures")
}

/// Feed one packet and pump the decoder once. The only contract under test
/// is that this returns without panicking; any `Err` / empty / frame is
/// acceptable. The decoder is reset afterwards so the next feed starts from
/// a clean slot regardless of how this one terminated.
fn feed_no_panic(dec: &mut Box<dyn Decoder>, data: &[u8]) {
    let pkt = Packet::new(0, TimeBase::new(1, 48_000), data.to_vec());
    if dec.send_packet(&pkt).is_ok() {
        let _ = dec.receive_frame();
    }
    let _ = dec.reset();
}

/// Deterministic LCG — no `rand` dependency, identical sequence run-to-run.
fn lcg(s: &mut u32) -> u32 {
    *s = s.wrapping_mul(1664525).wrapping_add(1013904223);
    *s
}

fn first_frame_len(data: &[u8]) -> usize {
    syncinfo::parse(data)
        .map(|s| s.frame_length as usize)
        .unwrap_or(768)
        .min(data.len())
        .max(1)
}

/// Drive one fixture through the three corruption families.
fn sweep_one(id: &str, base: &[u8]) {
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    let mut dec = reg
        .first_decoder(&CodecParameters::audio(CodecId::new(id)))
        .expect("make_decoder");

    // 1. Every truncation prefix (capped so the test stays quick).
    for t in 0..base.len().min(800) {
        feed_no_panic(&mut dec, &base[..t]);
    }

    let flen = first_frame_len(base);
    let bits = (flen * 8).min(2400);
    // Seed varies per fixture so the corruption patterns differ, but is a
    // pure function of the input (deterministic).
    let mut s: u32 = 0xACE1u32 ^ (base.len() as u32);

    // 2. Triple-bit flips on the first real frame.
    for _ in 0..2500 {
        let mut m = base[..flen].to_vec();
        for _ in 0..3 {
            let b = (lcg(&mut s) as usize) % bits;
            m[b / 8] ^= 1 << (b % 8);
        }
        feed_no_panic(&mut dec, &m);
    }

    // 3. Sync-prefixed random garbage.
    for _ in 0..1200 {
        let len = (lcg(&mut s) % 1800 + 2) as usize;
        let mut v = vec![0u8; len];
        for byte in v.iter_mut() {
            *byte = (lcg(&mut s) >> 20) as u8;
        }
        v[0] = 0x0B;
        v[1] = 0x77;
        feed_no_panic(&mut dec, &v);
    }
}

/// Walk the whole corpus and sweep every fixture. Skips cleanly when the
/// docs corpus is not checked out alongside the crate.
fn run_corpus_sweep() {
    let root = fixtures_root();
    let entries = match fs::read_dir(&root) {
        Ok(e) => e,
        Err(_) => {
            eprintln!("skip robustness sweep: {} absent", root.display());
            return;
        }
    };
    let mut swept = 0usize;
    for entry in entries.flatten() {
        let dir = entry.path();
        let (id, file) = if dir.join("input.ac3").exists() {
            ("ac3", dir.join("input.ac3"))
        } else if dir.join("input.eac3").exists() {
            ("eac3", dir.join("input.eac3"))
        } else {
            continue;
        };
        let base = match fs::read(&file) {
            Ok(b) if b.len() >= 8 => b,
            _ => continue,
        };
        sweep_one(id, &base);
        swept += 1;
    }
    eprintln!("robustness sweep: {swept} fixtures, no panic");
}

#[test]
fn decoder_survives_adversarial_corpus_input() {
    run_corpus_sweep();
}

/// Targeted regression for the r373 `eac3::ecpl` inverted-range panic: a
/// hand-built E-AC-3 frame whose enhanced-coupling sub-band range decodes
/// to `begin >= end` must yield a recoverable error, never a slice-index
/// abort. We synthesise the trigger from the `eac3-stereo` fixture by
/// flipping bits in the coupling-strategy region across a deterministic
/// sweep; the corpus sweep above already covers this, but this focused
/// case documents the specific failure mode and runs even if the broad
/// sweep is later trimmed.
#[test]
fn eac3_malformed_coupling_range_does_not_panic() {
    let dir = fixtures_root().join("eac3-stereo-48000-192kbps");
    let base = match fs::read(dir.join("input.eac3")) {
        Ok(b) if b.len() >= 8 => b,
        _ => {
            eprintln!("skip: eac3-stereo fixture absent");
            return;
        }
    };
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    let mut dec = reg
        .first_decoder(&CodecParameters::audio(CodecId::new("eac3")))
        .expect("make_eac3_decoder");

    // The first ~40 bytes after the sync/BSI carry the per-block coupling
    // strategy fields; an exhaustive double-bit-flip sweep over that window
    // reaches the inverted ecpl sub-band range that used to panic.
    let flen = base.len().min(256);
    for i in 16..flen.min(64) {
        for j in (i + 1)..flen.min(64) {
            for bi in 0..8 {
                for bj in 0..8 {
                    let mut m = base[..flen].to_vec();
                    m[i] ^= 1 << bi;
                    m[j] ^= 1 << bj;
                    feed_no_panic(&mut dec, &m);
                }
            }
        }
    }
}

/// SPX-bearing stream corruption sweep. The encoder-side spectral
/// extension (r386) emits syntax the corpus fixtures only partially
/// exercise (explicit band structures, per-channel attenuation codes,
/// non-zero copy-start offsets), and the decoder's SPX strategy /
/// coordinate parse carries several derived-geometry slices
/// (`spx_bndsztab`, translation plan, blend tables). Generate an SPX +
/// attenuation stream IN-PROCESS (no corpus dependency — this runs on
/// the standalone-repo CI too) and drive it through the same three
/// corruption families as the corpus sweep.
#[test]
fn eac3_spx_encoded_stream_survives_corruption() {
    use oxideav_core::{AudioFrame, Encoder, Error, Frame, SampleFormat};

    // Build ~4 frames of HF-rich stereo PCM and encode with SPX +
    // attenuation + explicit band structure (the densest SPX syntax).
    let n = 4 * 1536usize;
    let mut pcm_s16 = Vec::with_capacity(n * 2 * 2);
    let mut lfsr: u32 = 0xC0FF_EE01;
    for i in 0..n {
        let t = i as f32 / 48_000.0;
        let mut s = 0.16 * (2.0 * std::f32::consts::PI * 3_100.0 * t).sin()
            + 0.05 * (2.0 * std::f32::consts::PI * 14_000.0 * t).sin();
        lfsr ^= lfsr << 13;
        lfsr ^= lfsr >> 17;
        lfsr ^= lfsr << 5;
        s += ((lfsr as f32 / u32::MAX as f32) * 2.0 - 1.0) * 0.01;
        let q = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
        pcm_s16.extend_from_slice(&q.to_le_bytes()); // L
        pcm_s16.extend_from_slice(&q.to_le_bytes()); // R
    }
    let mut params = CodecParameters::audio(CodecId::new("eac3"));
    params.sample_rate = Some(48_000);
    params.channels = Some(2);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(192_000);
    let spx = oxideav_ac3::eac3::SpxParams {
        atten_code: Some(14),
        explicit_band_structure: true,
        ..Default::default()
    };
    let mut enc: Box<dyn Encoder> =
        oxideav_ac3::eac3::make_encoder_with_spx(&params, spx).expect("spx encoder");
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![pcm_s16],
    }))
    .unwrap();
    enc.flush().unwrap();
    let mut stream = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(p) => stream.extend_from_slice(&p.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("spx encode error: {e:?}"),
        }
    }
    assert!(stream.len() >= 768, "expected at least one 768-byte frame");

    // Same three corruption families as the corpus sweep.
    sweep_one("eac3", &stream);
}

/// r406: an enhanced-coupling encoded stream (§E.2.3.3.16-26) swept
/// through the same three corruption families. Enhanced coupling opens
/// decode paths the corpus never reaches — the ecpl strategy /
/// coordinate parse, the deferred §E.3.5.5 carrier synthesis and the
/// coupling-channel exponent/bap walk over the ecpl region — so bit
/// flips here reach field combinations (inverted sub-band ranges,
/// stale coordinate reuse after a strategy mutation, corrupted
/// carrier exponents) no other robustness input produces.
#[test]
fn eac3_ecpl_encoded_stream_survives_corruption() {
    use oxideav_core::{AudioFrame, Encoder, Error, Frame, SampleFormat};

    // ~4 frames of correlated in-region stereo content (the coupled
    // region starts at tc 37 ≈ 3.5 kHz), phase-offset on the right
    // channel so amplitude AND angle coordinates carry live values.
    let n = 4 * 1536usize;
    let mut pcm_s16 = Vec::with_capacity(n * 2 * 2);
    for i in 0..n {
        let t = i as f32 / 48_000.0;
        let mk = |phase: f32| -> i16 {
            let s = 0.18 * (2.0 * std::f32::consts::PI * 700.0 * t).sin()
                + 0.15 * (2.0 * std::f32::consts::PI * 4_300.0 * t + phase).sin()
                + 0.10 * (2.0 * std::f32::consts::PI * 9_800.0 * t + phase).sin()
                + 0.06 * (2.0 * std::f32::consts::PI * 14_500.0 * t + phase).sin();
            (s * 32767.0).clamp(-32768.0, 32767.0) as i16
        };
        pcm_s16.extend_from_slice(&mk(0.0).to_le_bytes()); // L
        pcm_s16.extend_from_slice(&mk(0.7).to_le_bytes()); // R
    }
    let mut params = CodecParameters::audio(CodecId::new("eac3"));
    params.sample_rate = Some(48_000);
    params.channels = Some(2);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(192_000);
    let mut enc: Box<dyn Encoder> = oxideav_ac3::eac3::make_encoder_with_ecpl(
        &params,
        oxideav_ac3::eac3::EcplParams::default(),
    )
    .expect("ecpl encoder");
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![pcm_s16],
    }))
    .unwrap();
    enc.flush().unwrap();
    let mut stream = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(p) => stream.extend_from_slice(&p.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("ecpl encode error: {e:?}"),
        }
    }
    assert!(stream.len() >= 768, "expected at least one 768-byte frame");

    // Same three corruption families as the corpus sweep.
    sweep_one("eac3", &stream);
}

/// r409: metadata-bearing streams (the encoder-side §5.4.2 / Table E1.2
/// metadata surfaces) swept through the same three corruption families.
/// The optional BSI words open parse paths the corpus never reaches —
/// the base compr/langcod/audprodie chains, the Annex E mixing-metadata
/// walk (mix levels, pgmscl chains, `mixdef` bodies including the
/// variable-length `mixdef == 3` arm a bit flip can select) and the
/// informational block — so bit flips here reach field combinations no
/// other robustness input produces.
#[test]
fn metadata_bearing_streams_survive_corruption() {
    use oxideav_core::{AudioFrame, Encoder, Error, Frame};

    let encode = |codec: &str, channels: u16, enc: &mut Box<dyn Encoder>| -> Vec<u8> {
        let n = 3 * 1536usize;
        let nch = channels as usize;
        let mut pcm_s16 = Vec::with_capacity(n * nch * 2);
        for i in 0..n {
            let t = i as f32 / 48_000.0;
            for ch in 0..nch {
                let s = 0.25 * (2.0 * std::f32::consts::PI * (300.0 + 150.0 * ch as f32) * t).sin();
                let q = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
                pcm_s16.extend_from_slice(&q.to_le_bytes());
            }
        }
        enc.send_frame(&Frame::Audio(AudioFrame {
            samples: n as u32,
            pts: Some(0),
            data: vec![pcm_s16],
        }))
        .unwrap();
        enc.flush().unwrap();
        let mut stream = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => stream.extend_from_slice(&p.data),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("{codec} metadata encode error: {e:?}"),
            }
        }
        assert!(!stream.is_empty(), "{codec}: no metadata stream produced");
        stream
    };

    // Base AC-3 5.1 with every optional word live.
    let mut params = CodecParameters::audio(CodecId::new("ac3"));
    params.sample_rate = Some(48_000);
    params.channels = Some(6);
    params.options = oxideav_core::CodecOptions::new()
        .set("dialnorm", "24")
        .set("compr", "0xC5")
        .set("dynrng", "0xC0")
        .set("bsmod", "2")
        .set("langcod", "255")
        .set("mixlevel", "21")
        .set("roomtyp", "2")
        .set("copyright", "1");
    let mut aenc: Box<dyn Encoder> =
        oxideav_ac3::encoder::make_encoder(&params).expect("ac3 metadata encoder");
    let ac3_stream = encode("ac3", 6, &mut aenc);
    sweep_one("ac3", &ac3_stream);

    // E-AC-3 5.1 with the mixing + informational blocks live.
    let mut params = CodecParameters::audio(CodecId::new("eac3"));
    params.sample_rate = Some(48_000);
    params.channels = Some(6);
    params.bit_rate = Some(384_000);
    params.options = oxideav_core::CodecOptions::new()
        .set("dialnorm", "22")
        .set("compr", "0xB3")
        .set("dynrng", "0xC0")
        .set("dmixmod", "2")
        .set("ltrtcmixlev", "5")
        .set("lorocmixlev", "4")
        .set("ltrtsurmixlev", "6")
        .set("lorosurmixlev", "5")
        .set("lfemixlevcod", "15")
        .set("pgmscl", "51")
        .set("extpgmscl", "45")
        .set("bsmod", "2")
        .set("dsurexmod", "2")
        .set("mixlevel", "18")
        .set("roomtyp", "1")
        .set("adconvtyp", "1")
        .set("copyright", "1")
        .set("origbs", "0");
    let mut eenc: Box<dyn Encoder> =
        oxideav_ac3::eac3::make_encoder(&params).expect("eac3 metadata encoder");
    let eac3_stream = encode("eac3", 6, &mut eenc);
    sweep_one("eac3", &eac3_stream);
}
