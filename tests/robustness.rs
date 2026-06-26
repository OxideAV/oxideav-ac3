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
