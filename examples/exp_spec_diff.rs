//! Independent §7.1.3 exponent decode vs the live decoder, on a chosen
//! frame/block. Session 5: mask formula and BA tables are exonerated, so
//! the cold stage-3 divergence (if real) must trace to the per-bin PSD,
//! i.e. the exponents. This re-decodes exponents straight from the
//! bitstream — reading raw 4-bit absexp + 7-bit grouped codes at the
//! side-info-verified field offsets — and diffs against the live
//! `state.channels[ch].exp`.
//!
//! It also flags any running exponent that leaves [0,24]: oxideav clamps
//! such values; ffmpeg treats them as a hard bitstream error. A clamp that
//! fires here would be a silent divergence from ffmpeg.
//!
//! Run: cargo run --example exp_spec_diff -- white 0 0
use oxideav_ac3::audblk::{peek_bits_post_bsi, state_after_block, trace_block_side_info};
use oxideav_ac3::{bsi, syncinfo};

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

/// Independent §7.1.3 group expansion. Returns (exps_by_bin, out_of_range)
/// where exps include bin 0 = absexp and `out_of_range` records any running
/// absolute exponent that fell outside [0,24] before clamping.
fn decode_exp_independent(
    absexp: i32,
    groups: &[u32],
    grpsize: usize,
    end: usize,
) -> (Vec<i32>, Vec<(usize, i32)>) {
    let mut exps = Vec::with_capacity(end);
    let mut oor = Vec::new();
    exps.push(absexp); // bin 0
    let mut prev = absexp;
    'outer: for &g in groups {
        let g = g as i32;
        let mapped = [g / 25, (g % 25) / 5, (g % 25) % 5];
        for m in mapped {
            prev += m - 2;
            if !(0..=24).contains(&prev) {
                oor.push((exps.len(), prev));
            }
            for _ in 0..grpsize {
                exps.push(prev);
                if exps.len() >= end {
                    break 'outer;
                }
            }
        }
    }
    (exps, oor)
}

fn mark<'a>(marks: &'a [oxideav_ac3::audblk::SideInfoMark], label: &str) -> Option<&'a u64> {
    marks.iter().find(|m| m.label == label).map(|m| &m.bit_pos)
}

fn main() {
    let dir = std::env::var("TEMP").unwrap() + r"\oxideav_repro";
    let stem = std::env::args().nth(1).unwrap_or_else(|| "white".to_string());
    let frame_idx: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let blk: usize = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(0);

    let path = format!("{dir}\\{stem}.ac3");
    let data = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let (off, len) = nth_frame(&data, frame_idx).unwrap_or_else(|| panic!("frame {frame_idx}"));
    let frame = &data[off..off + len];
    let si = syncinfo::parse(frame).unwrap();
    let b = bsi::parse(&frame[5..]).unwrap();
    let nfchans = b.nfchans as usize;

    let state = state_after_block(&si, &b, frame, blk).unwrap_or_else(|e| panic!("{e}"));
    let marks = trace_block_side_info(&si, &b, frame, blk).unwrap_or_else(|e| panic!("{e}"));

    // Exponent field start: ch0 begins at `after_chbwcod` (no coupling exps
    // when cplinu=0). Each fbw channel's exp field ends at after_chN_exponents.
    let start_label = "after_chbwcod";
    let end_labels = ["after_ch0_exponents", "after_ch1_exponents"];

    println!("stream={stem}.ac3 frame={frame_idx} blk={blk} nfchans={nfchans} cplinu={}", state.cpl_in_use);
    if state.cpl_in_use {
        println!("NOTE: coupling in use — ch0 field start offset would include cpl exps; skipping.");
        return;
    }

    let mut field_start = *mark(&marks, start_label).expect("after_chbwcod");
    for ch in 0..nfchans.min(2) {
        let field_end = *mark(&marks, end_labels[ch]).expect("ch exp end mark");
        let end = state.channels[ch].end_mant;
        // Field layout: 4 (absexp) + ngrps*7 + 2 (gainrng).
        let field_bits = field_end - field_start;
        let group_bits = field_bits.saturating_sub(4 + 2);
        let ngrps = (group_bits / 7) as usize;
        let absexp = peek_bits_post_bsi(frame, &b, field_start, 4) as i32;
        let groups: Vec<u32> = (0..ngrps)
            .map(|i| peek_bits_post_bsi(frame, &b, field_start + 4 + (i as u64) * 7, 7))
            .collect();
        // Infer grpsize from coverage: ngrps*3 differentials cover end-1 bins.
        let diffs = ngrps * 3;
        let grpsize = if diffs == 0 { 1 } else { ((end - 1) + diffs - 1) / diffs };

        let (exps, oor) = decode_exp_independent(absexp, &groups, grpsize, end);

        let mut diffs_found: Vec<(usize, i32, i32)> = Vec::new();
        for bin in 0..end {
            let live = state.channels[ch].exp[bin] as i32;
            let indep = exps.get(bin).copied().unwrap_or(-999).clamp(0, 24);
            if live != indep {
                diffs_found.push((bin, live, indep));
            }
        }

        println!(
            "\nch{ch}: end_mant={end} field_bits={field_bits} ngrps={ngrps} grpsize={grpsize} absexp={absexp}",
        );
        if diffs_found.is_empty() {
            println!("  indep-vs-live exp: MATCH (all {end} bins)");
        } else {
            println!("  indep-vs-live exp: {} bin(s) DIFFER:", diffs_found.len());
            for (bin, live, indep) in diffs_found.iter().take(30) {
                println!("    bin {bin:3}: live={live} indep={indep}");
            }
        }
        if oor.is_empty() {
            println!("  running exponents in [0,24]: yes (no clamp; matches ffmpeg's error gate)");
        } else {
            println!(
                "  ⚠ {} running exponent(s) OUT OF [0,24] — oxideav clamps, ffmpeg would ERROR:",
                oor.len()
            );
            for (bin, val) in oor.iter().take(10) {
                println!("    bin {bin:3}: raw running exp = {val}");
            }
        }
        field_start = field_end;
    }
}
