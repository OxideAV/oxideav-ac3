//! Independent spec-transcribed mask/excitation vs the live decoder.
//!
//! Session 5: the BA *tables* are audited clean (`tables::ba_table_audit`)
//! and the addr/quantizer step is verified spec-faithful by inspection, so
//! the cold stage-3 divergence from ffmpeg — if any — must be in the
//! excitation→mask recursion or upstream in PSD/exponents. This example
//! reimplements §7.2.2.4–7.2.2.6 *directly from the A/52 pseudocode* (a
//! single unified `calc_lowcomp`, different loop structure than the live
//! `run_bit_allocation_staged`), then diffs its `mask[band]` against the
//! live decoder's stored mask on a given frame/block.
//!
//! It reuses only the constant tables (already audited), not the live
//! algorithm — so agreement is real evidence the mask path is correct and
//! the fault is upstream (PSD/exponents); disagreement localizes it here.
//!
//! Run: cargo run --example mask_spec_diff -- white 0 0
#![allow(clippy::needless_range_loop)]
use oxideav_ac3::audblk::{compare_channel_ba_ref, state_after_block};
use oxideav_ac3::tables::{DBPBTAB, FASTDEC, FASTGAIN, HTH, MASKTAB, SLOWDEC, SLOWGAIN};
use oxideav_ac3::{bsi, syncinfo};

/// A/52 §7.2.2.4 `calc_lowcomp` — single unified form from the spec.
fn calc_lowcomp(a: i32, b0: i32, b1: i32, bin: usize) -> i32 {
    if bin < 7 {
        if b0 + 256 == b1 {
            384
        } else if b0 > b1 {
            (a - 64).max(0)
        } else {
            a
        }
    } else if bin < 20 {
        if b0 + 256 == b1 {
            320
        } else if b0 > b1 {
            (a - 64).max(0)
        } else {
            a
        }
    } else {
        (a - 128).max(0)
    }
}

/// §7.2.2.4–7.2.2.6 mask for an fbw/LFE channel (bndstrt == 0), transcribed
/// straight from the spec pseudocode. Returns mask[0..bndend].
#[allow(clippy::too_many_arguments)]
fn spec_calc_mask_fbw(
    bndpsd: &[i32],
    bndend: usize,
    fdecay: i32,
    sdecay: i32,
    fgain: i32,
    sgain: i32,
    dbknee: i32,
    fscod: usize,
    sr_shift: usize,
    deltnseg: usize,
    deltoffst: &[u8],
    deltlen: &[u8],
    deltba: &[u8],
) -> [i32; 50] {
    let bp = |i: usize| bndpsd[i.min(49)];
    let is_lfe = bndend == 7;
    let mut excite = [0i32; 50];
    let mut lowcomp = 0i32;

    lowcomp = calc_lowcomp(lowcomp, bp(0), bp(1), 0);
    excite[0] = bp(0) - fgain - lowcomp;
    lowcomp = calc_lowcomp(lowcomp, bp(1), bp(2), 1);
    excite[1] = bp(1) - fgain - lowcomp;

    let mut fastleak = 0i32;
    let mut slowleak = 0i32;
    let mut begin = 7usize;
    for bin in 2..7.min(bndend) {
        if !(is_lfe && bin == 6) {
            lowcomp = calc_lowcomp(lowcomp, bp(bin), bp(bin + 1), bin);
        }
        fastleak = bp(bin) - fgain;
        slowleak = bp(bin) - sgain;
        excite[bin] = fastleak - lowcomp;
        if !(is_lfe && bin == 6) && bp(bin) <= bp(bin + 1) {
            begin = bin + 1;
            break;
        }
    }
    for bin in begin..22.min(bndend) {
        if !(is_lfe && bin == 6) {
            lowcomp = calc_lowcomp(lowcomp, bp(bin), bp(bin + 1), bin);
        }
        fastleak -= fdecay;
        fastleak = fastleak.max(bp(bin) - fgain);
        slowleak -= sdecay;
        slowleak = slowleak.max(bp(bin) - sgain);
        excite[bin] = (fastleak - lowcomp).max(slowleak);
    }
    for bin in 22..bndend {
        fastleak -= fdecay;
        fastleak = fastleak.max(bp(bin) - fgain);
        slowleak -= sdecay;
        slowleak = slowleak.max(bp(bin) - sgain);
        excite[bin] = fastleak.max(slowleak);
    }

    // §7.2.2.5 masking curve.
    let mut mask = [0i32; 50];
    for bin in 0..bndend {
        let mut exc = excite[bin];
        if bp(bin) < dbknee {
            exc += (dbknee - bp(bin)) >> 2;
        }
        let hth = HTH[fscod][(bin >> sr_shift).min(49)] as i32;
        mask[bin] = exc.max(hth);
    }

    // §7.2.2.6 delta bit allocation (mask offsets).
    if deltnseg > 0 {
        let mut band = 0usize;
        for seg in 0..deltnseg {
            band += deltoffst[seg] as usize;
            let raw = deltba[seg] as i32;
            let delta = if raw >= 4 {
                (raw - 3) << 7
            } else {
                (raw - 4) << 7
            };
            for _ in 0..deltlen[seg] {
                if band < 50 {
                    mask[band] += delta;
                }
                band += 1;
            }
        }
    }
    mask
}

fn nth_frame(data: &[u8], index: usize) -> Option<(usize, usize)> {
    let mut off = 0usize;
    let mut n = 0usize;
    while off + 5 <= data.len() {
        let si = syncinfo::parse(&data[off..]).ok()?;
        let len = si.frame_length as usize;
        if len < 5 || off + len > data.len() {
            break;
        }
        if n == index {
            return Some((off, len));
        }
        off += len;
        n += 1;
    }
    None
}

fn main() {
    let dir = std::env::var("TEMP").unwrap() + r"\oxideav_repro";
    let stem = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "white".to_string());
    let frame_idx: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let blk: usize = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    let path = format!("{dir}\\{stem}.ac3");
    let data = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let (off, len) = nth_frame(&data, frame_idx).unwrap_or_else(|| panic!("frame {frame_idx}"));
    let frame = &data[off..off + len];
    let si = syncinfo::parse(frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();
    let nfchans = bsi.nfchans as usize;

    let state = state_after_block(&si, &bsi, frame, blk).unwrap_or_else(|e| panic!("{e}"));
    let sr_shift = state.sr_shift as usize;
    let sdecay = SLOWDEC[state.sdcycod as usize] >> sr_shift;
    let fdecay = FASTDEC[state.fdcycod as usize] >> sr_shift;
    let sgain = SLOWGAIN[state.sgaincod as usize];
    let dbknee = DBPBTAB[state.dbpbcod as usize];

    println!(
        "stream={stem}.ac3 frame={frame_idx} blk={blk} nfchans={nfchans} fscod={} sr_shift={sr_shift}",
        si.fscod
    );

    for ch in 0..nfchans {
        if state.channels[ch].in_coupling {
            println!("ch{ch}: coupled — spec path here is fbw-only, skipping");
            continue;
        }
        let end = state.channels[ch].end_mant;
        if end == 0 {
            continue;
        }
        let bndend = MASKTAB[end - 1] as usize + 1;
        let bndpsd: Vec<i32> = (0..50)
            .map(|b| state.channels[ch].bndpsd[b] as i32)
            .collect();

        let fgain = FASTGAIN[state.fgaincod[ch] as usize];
        let spec_mask = spec_calc_mask_fbw(
            &bndpsd,
            bndend,
            fdecay,
            sdecay,
            fgain,
            sgain,
            dbknee,
            si.fscod as usize,
            sr_shift,
            state.deltnseg[ch],
            &state.deltoffst[ch],
            &state.deltlen[ch],
            &state.deltba[ch],
        );

        let mut diffs: Vec<(usize, i32, i32)> = Vec::new();
        for band in 0..bndend {
            let live = state.channels[ch].mask[band] as i32;
            if live != spec_mask[band] {
                diffs.push((band, live, spec_mask[band]));
            }
        }

        // Three-way context: live-vs-ffmpeg-ref (existing port).
        let cmp = compare_channel_ba_ref(&state, &si, ch, blk);

        println!(
            "\nch{ch}: end_mant={end} bndend={bndend} fgaincod={} fsnroffst={}",
            state.fgaincod[ch], state.fsnroffst[ch]
        );
        if diffs.is_empty() {
            println!("  spec-vs-live mask: MATCH (all {bndend} bands)");
        } else {
            println!("  spec-vs-live mask: {} band(s) DIFFER:", diffs.len());
            for (band, live, spec) in diffs.iter().take(30) {
                println!(
                    "    band {band:2}: live={live} spec={spec} delta={}",
                    live - spec
                );
            }
        }
        println!(
            "  ref-vs-live mask: {} band(s) differ; mant bits ours={} ref={}",
            cmp.mask_diff_bands.len(),
            cmp.ours_mant_bits,
            cmp.ref_mant_bits
        );
    }
}
