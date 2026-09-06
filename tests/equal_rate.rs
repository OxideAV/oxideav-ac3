//! Equal-rate rate-distortion gates.
//!
//! Every corpus clip is encoded by our AC-3 / E-AC-3 encoder at a
//! fixed nominal rate, decoded by **our** decoder and by the black-box
//! reference decoder, and scored (worst-channel SNR + mean NMR) against
//! the source. Three things are gated:
//!
//! 1. **Conformance** — the two decoders agree on our stream (the SNR
//!    they reach differs by < 1.5 dB and the mean NMR by < 6 dB — see
//!    the dither note in `run_cell`).
//! 2. **Position** — per-cell SNR / NMR floors pinned a few dB under
//!    the measured value at the time the last tool landed, so a
//!    rate-distortion regression fails CI.
//! 3. **Reference distance** — when the black-box encoder is available,
//!    our worst-channel SNR stays within the pinned distance of the
//!    reference encoder's at the same rate (both decoded by the
//!    reference decoder).
//!
//! The black-box parts skip gracefully when `ffmpeg` is absent; the
//! in-tree decode + position gates always run.
//!
//! `cargo run --release --example equal_rate_report` prints the full
//! ladder (README "Equal-rate position" table).

mod common;

use common::rd::{self, Codec, Dec, Enc};

/// One gated cell: signal, codec, rate, pinned floors. The pins sit
/// ≈ 3 dB under the r457 measurement on the 1.5 s clips (worst-channel
/// SNR / mean NMR through our decoder) and ≈ 3 dB over the measured
/// distance to the reference encoder.
struct Cell {
    signal: &'static str,
    codec: Codec,
    kbps: u32,
    /// Worst-channel SNR floor (dB) for our stream through our decoder.
    snr_floor: f64,
    /// Mean-NMR ceiling (dB) for our stream through our decoder.
    nmr_ceil: f64,
    /// Max allowed SNR shortfall (dB) vs the reference encoder at the
    /// same rate (both through the reference decoder).
    ref_gap: f64,
}

const CELLS: &[Cell] = &[
    Cell {
        signal: "speech",
        codec: Codec::Ac3,
        kbps: 96,
        snr_floor: 12.0,
        nmr_ceil: -16.0,
        ref_gap: 6.0,
    },
    Cell {
        signal: "music",
        codec: Codec::Ac3,
        kbps: 192,
        snr_floor: 30.0,
        nmr_ceil: -1.0,
        ref_gap: 4.0,
    },
    Cell {
        signal: "transients",
        codec: Codec::Ac3,
        kbps: 192,
        snr_floor: 17.5,
        nmr_ceil: 4.0,
        ref_gap: 6.0,
    },
    Cell {
        signal: "tones",
        codec: Codec::Ac3,
        kbps: 192,
        snr_floor: 60.0,
        nmr_ceil: -40.0,
        ref_gap: 4.0,
    },
    Cell {
        signal: "pink",
        codec: Codec::Ac3,
        kbps: 192,
        snr_floor: 11.5,
        nmr_ceil: 10.0,
        ref_gap: 4.0,
    },
    Cell {
        signal: "mix-5.1",
        codec: Codec::Ac3,
        kbps: 448,
        snr_floor: 18.5,
        nmr_ceil: 2.0,
        ref_gap: 6.0,
    },
    Cell {
        signal: "speech",
        codec: Codec::Eac3,
        kbps: 96,
        snr_floor: 12.0,
        nmr_ceil: -16.0,
        ref_gap: 6.0,
    },
    Cell {
        signal: "music",
        codec: Codec::Eac3,
        kbps: 192,
        snr_floor: 28.5,
        nmr_ceil: 0.5,
        ref_gap: 6.0,
    },
    Cell {
        signal: "transients",
        codec: Codec::Eac3,
        kbps: 192,
        snr_floor: 17.0,
        nmr_ceil: 7.0,
        ref_gap: 6.0,
    },
    Cell {
        signal: "mix-5.1",
        codec: Codec::Eac3,
        kbps: 448,
        snr_floor: 16.0,
        nmr_ceil: 5.0,
        ref_gap: 6.0,
    },
];

const CLIP_SECONDS: f32 = 1.5;

fn run_cell(cell: &Cell) -> Vec<String> {
    let sig = rd::signal_by_name(CLIP_SECONDS, cell.signal);
    let enc = Enc::Ours(cell.codec, Vec::new());
    let es = rd::encode(&sig, &enc, cell.kbps).expect("our encoder");
    let mut failures = Vec::new();
    let tag = format!("{}/{}/{}k", cell.signal, cell.codec.id(), cell.kbps);

    // In-tree decode + position gate.
    let pcm_ours = rd::decode(&es, &enc, Dec::Ours, sig.channels).expect("our decoder");
    let lag_ours = rd::path_lag(&enc, Dec::Ours).expect("our-path lag");
    let ours = rd::score(&sig.pcm, &pcm_ours, sig.channels, sig.lfe, lag_ours);
    eprintln!("{tag:<24} ours→ours {}", rd::fmt_score(&ours));
    if ours.snr_min() < cell.snr_floor {
        failures.push(format!(
            "{tag}: worst-channel SNR {:.2} dB under the pinned floor {:.2}",
            ours.snr_min(),
            cell.snr_floor
        ));
    }
    if ours.nmr_mean > cell.nmr_ceil {
        failures.push(format!(
            "{tag}: mean NMR {:.2} dB above the pinned ceiling {:.2}",
            ours.nmr_mean, cell.nmr_ceil
        ));
    }

    if !rd::ffmpeg_present() {
        eprintln!("{tag}: reference binary absent — black-box gates skipped");
        return failures;
    }

    // Conformance: the reference decoder reaches the same quality.
    let pcm_ref = rd::decode(&es, &enc, Dec::Reference, sig.channels)
        .unwrap_or_else(|| panic!("{tag}: the reference decoder rejected our stream"));
    let lag_ref = rd::path_lag(&enc, Dec::Reference).expect("reference-path lag");
    let theirs = rd::score(&sig.pcm, &pcm_ref, sig.channels, sig.lfe, lag_ref);
    eprintln!("{tag:<24} ours→ref  {}", rd::fmt_score(&theirs));
    if (ours.snr_min() - theirs.snr_min()).abs() > 1.5 {
        failures.push(format!(
            "{tag}: decoders disagree on our stream — SNR {:.2} (ours) vs {:.2} (reference)",
            ours.snr_min(),
            theirs.snr_min()
        ));
    }
    // The NMR agreement is looser than the SNR one: the mean NMR is
    // dominated by masked (bap-0) bands, which each decoder fills with
    // its own dither sequence — on the 96 kbps speech cell the two
    // decoders sit ~5.5 dB apart there while their SNRs agree within
    // 0.4 dB (a decoder-side dither-level question, recorded as a
    // follow-up, not an encoder conformance signal).
    if (ours.nmr_mean - theirs.nmr_mean).abs() > 6.0 {
        failures.push(format!(
            "{tag}: decoders disagree on our stream — NMR {:.2} (ours) vs {:.2} (reference)",
            ours.nmr_mean, theirs.nmr_mean
        ));
    }

    // Reference distance at equal rate.
    let ref_enc = Enc::Reference(cell.codec);
    if let Some((reference, _)) = rd::measure(&sig, &ref_enc, Dec::Reference, cell.kbps) {
        eprintln!("{tag:<24} ref→ref   {}", rd::fmt_score(&reference));
        let gap = reference.snr_min() - theirs.snr_min();
        if gap > cell.ref_gap {
            failures.push(format!(
                "{tag}: {:.2} dB behind the reference encoder (pinned max {:.2})",
                gap, cell.ref_gap
            ));
        }
    } else {
        eprintln!("{tag}: reference encoder unavailable for this configuration — gap gate skipped");
    }
    failures
}

#[test]
fn equal_rate_position_ac3() {
    let mut failures = Vec::new();
    for cell in CELLS.iter().filter(|c| c.codec == Codec::Ac3) {
        failures.extend(run_cell(cell));
    }
    assert!(
        failures.is_empty(),
        "equal-rate gates failed:\n{}",
        failures.join("\n")
    );
}

#[test]
fn equal_rate_position_eac3() {
    let mut failures = Vec::new();
    for cell in CELLS.iter().filter(|c| c.codec == Codec::Eac3) {
        failures.extend(run_cell(cell));
    }
    assert!(
        failures.is_empty(),
        "equal-rate gates failed:\n{}",
        failures.join("\n")
    );
}

/// The masking model used for NMR must sit below the signal on a
/// loud tone (sanity of the scorer itself, independent of any codec).
#[test]
fn scorer_mask_sits_under_a_loud_tone() {
    let mut psd = [0.0f64; 256];
    for p in psd.iter_mut() {
        *p = 3072.0 - 24.0 * 128.0;
    }
    psd[20] = 3072.0 - 128.0; // one loud bin, exponent 1
    let mask = rd::spec_mask(&psd, 253, 0);
    let band = oxideav_ac3::tables::MASKTAB[20] as usize;
    assert!(
        mask[band] < psd[20] && mask[band] > psd[20] - 60.0 * (128.0 / 6.02),
        "mask {} out of the expected window under psd {}",
        mask[band],
        psd[20]
    );
    // Silence is floored at the hearing threshold.
    assert!(mask[45] >= oxideav_ac3::tables::HTH[0][45] as f64);
}
