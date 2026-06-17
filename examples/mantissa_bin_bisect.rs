//! Single-bin, grouping-aware mantissa walk bisect for one audio block.
//!
//! For each coefficient in decoder read order, measures how much
//! [`count_mantissa_bits_walk`] changes when that bin's BAP is toggled
//! (±1 or to a neighbor frame's value). Flags bins whose flip closes the
//! gap between our walk and the bitstream-inferred blk0 mantissa budget.
//!
//! Usage:
//!   cargo run --release --example mantissa_bin_bisect -- <file.ac3> [frame] [neighbor]
//!   cargo run --release --example mantissa_bin_bisect -- white.ac3 0 1
//!
//! `neighbor` defaults to `frame + 1` when present, else omitted.

use oxideav_ac3::audblk::{
    audit_frame_blocks, count_mantissa_bits_walk, probe_next_block_alignment, state_after_block,
    try_parse_block_at, Ac3State, MAX_FBW, N_COEFFS,
};
use oxideav_ac3::bsi;
use oxideav_ac3::syncinfo;
use oxideav_ac3::tables::QUANTIZATION_BITS;

#[derive(Clone, Copy)]
struct BinPos {
    order: usize,
    ch: usize,
    bin: usize,
    bap: u8,
}

#[derive(Clone)]
struct FlipHit {
    order: usize,
    ch: usize,
    bin: usize,
    from_bap: u8,
    to_bap: u8,
    walk_delta: i32,
    naive_delta: i32,
    label: &'static str,
}

fn nth_frame(data: &[u8], index: usize) -> Option<Vec<u8>> {
    let mut off = 0usize;
    let mut n = 0usize;
    while off + 5 <= data.len() {
        let si = syncinfo::parse(&data[off..]).ok()?;
        let len = si.frame_length as usize;
        if len < 5 || off + len > data.len() {
            break;
        }
        if n == index {
            return Some(data[off..off + len].to_vec());
        }
        off += len;
        n += 1;
    }
    None
}

fn ch_label(ch: usize) -> String {
    if ch < MAX_FBW {
        format!("fbw{ch}")
    } else if ch == MAX_FBW {
        "cpl".into()
    } else {
        "lfe".into()
    }
}

/// Coefficients in the same order as [`oxideav_ac3::audblk::unpack_mantissas`].
fn decode_order_bins(state: &Ac3State, bsi: &bsi::Bsi) -> Vec<BinPos> {
    let nfchans = bsi.nfchans as usize;
    let mut out = Vec::new();
    let mut got_cplchan = false;
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        for bin in 0..end {
            out.push(BinPos {
                order: out.len(),
                ch,
                bin,
                bap: state.channels[ch].bap[bin],
            });
        }
        if state.cpl_in_use && state.channels[ch].in_coupling && !got_cplchan {
            let cplc = MAX_FBW;
            for bin in state.cpl_begf_mant..state.cpl_endf_mant {
                out.push(BinPos {
                    order: out.len(),
                    ch: cplc,
                    bin,
                    bap: state.channels[cplc].bap[bin],
                });
            }
            got_cplchan = true;
        }
    }
    if bsi.lfeon {
        let lfe_ch = MAX_FBW + 1;
        for bin in 0..7 {
            out.push(BinPos {
                order: out.len(),
                ch: lfe_ch,
                bin,
                bap: state.channels[lfe_ch].bap[bin],
            });
        }
    }
    out
}

fn naive_bits_delta(from_bap: u8, to_bap: u8) -> i32 {
    if from_bap == 0 && to_bap == 0 {
        return 0;
    }
    let from = if from_bap == 0 {
        0
    } else {
        QUANTIZATION_BITS[from_bap as usize] as i32
    };
    let to = if to_bap == 0 {
        0
    } else {
        QUANTIZATION_BITS[to_bap as usize] as i32
    };
    to - from
}

fn walk_delta(
    state: &Ac3State,
    bsi: &bsi::Bsi,
    ch: usize,
    bin: usize,
    new_bap: u8,
    base: u32,
) -> i32 {
    let mut st = state.clone();
    st.channels[ch].bap[bin] = new_bap;
    count_mantissa_bits_walk(&st, bsi) as i32 - base as i32
}

/// Nearest blk1 parse alignment (same probe as `bap_audit`).
fn inferred_budget_nearest_parse(
    si: &syncinfo::SyncInfo,
    bsi: &bsi::Bsi,
    frame: &[u8],
    pre_mantissa: u64,
) -> Option<(u32, i32)> {
    let hits = probe_next_block_alignment(si, bsi, frame, 0, -48, 48).ok()?;
    let (delta, _, pos) = hits
        .iter()
        .filter(|(_, ok, _)| *ok)
        .min_by_key(|(d, _, _)| d.abs())?;
    Some(((pos - pre_mantissa) as u32, *delta))
}

/// Structured blk1 header: `side_info_bits == 15` and blk1 mantissa walk matches blk0.
fn inferred_budget_structured(
    si: &syncinfo::SyncInfo,
    bsi: &bsi::Bsi,
    frame: &[u8],
    blk0_end: u64,
    pre_mantissa: u64,
    walk: u32,
) -> Option<(u32, i32)> {
    let mut best: Option<(u32, i32)> = None;
    for delta in -48i32..=48 {
        let pos = (blk0_end as i64 + i64::from(delta)).max(0) as u64;
        let mut st = state_after_block(si, bsi, frame, 0).ok()?;
        if let Ok(m) = try_parse_block_at(si, bsi, frame, &mut st, 1, pos) {
            let side = m.bit_pre_mantissa.saturating_sub(pos);
            let mant = m.bit_block_end.saturating_sub(m.bit_pre_mantissa);
            if side == 15 && mant == u64::from(walk) {
                let budget = (pos - pre_mantissa) as u32;
                if best.as_ref().map_or(true, |(_, d)| delta.abs() < d.abs()) {
                    best = Some((budget, delta));
                }
            }
        }
    }
    best
}

fn neighbor_bap_at(neighbor_state: Option<&Ac3State>, ch: usize, bin: usize) -> Option<u8> {
    let st = neighbor_state?;
    if ch >= st.channels.len() || bin >= N_COEFFS {
        return None;
    }
    Some(st.channels[ch].bap[bin])
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: mantissa_bin_bisect <file.ac3> [frame] [neighbor]");
    let frame_index: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let neighbor_index: Option<usize> = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .or_else(|| Some(frame_index + 1));

    let data = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let frame =
        nth_frame(&data, frame_index).unwrap_or_else(|| panic!("frame {frame_index} missing"));
    let si = syncinfo::parse(&frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();
    let audits = audit_frame_blocks(&si, &bsi, &frame).expect("audit");
    let b0 = audits
        .first()
        .filter(|a| a.parse_ok)
        .expect("blk0 must parse");

    let state = state_after_block(&si, &bsi, &frame, 0).expect("state after blk0");
    let base_walk = count_mantissa_bits_walk(&state, &bsi);
    let nearest = inferred_budget_nearest_parse(&si, &bsi, &frame, b0.bit_pre_mantissa);
    let structured = inferred_budget_structured(
        &si,
        &bsi,
        &frame,
        b0.bit_block_end,
        b0.bit_pre_mantissa,
        base_walk,
    );
    let budget = nearest.map(|(b, _)| b);
    let align_delta = nearest.map(|(_, d)| d);
    let gap = budget.map(|b| b as i32 - base_walk as i32);

    let neighbor_state = neighbor_index.and_then(|ni| {
        let nf = nth_frame(&data, ni)?;
        let si_n = syncinfo::parse(&nf).ok()?;
        let bsi_n = bsi::parse(&nf[5..]).ok()?;
        state_after_block(&si_n, &bsi_n, &nf, 0).ok()
    });

    println!("file: {path}");
    println!("frame: {frame_index}  neighbor: {:?}", neighbor_index);
    println!(
        "cpl_in_use={}  nfchans={}  lfeon={}  fsnroffst={:?}",
        state.cpl_in_use,
        bsi.nfchans,
        bsi.lfeon,
        &b0.fsnroffst[..bsi.nfchans as usize],
    );
    println!(
        "blk0: pre_mant={} walk={} actual={} end={}",
        b0.bit_pre_mantissa, base_walk, b0.mantissa_bits_actual, b0.bit_block_end,
    );
    println!(
        "inferred budget (nearest blk1 parse_ok): {:?}  align delta {:?}  gap: {:?}",
        budget, align_delta, gap
    );
    if let Some((sb, sd)) = structured {
        let sg = sb as i32 - base_walk as i32;
        println!("structured budget (side=15, blk1 mant==walk): {sb}  delta {sd:+}  gap: {sg:+}");
    } else {
        println!("structured budget (side=15, blk1 mant==walk): (none in [-48,+48])");
    }
    if let Some(g) = gap {
        if g == 0 {
            println!("walk already matches inferred budget");
        } else if g > 0 {
            println!("need +{g} walk bits (or find bins to bump BAP)");
        } else {
            println!("over-read by {} bits — find bins to lower BAP", -g);
        }
    }
    println!();

    let bins = decode_order_bins(&state, &bsi);
    let mut hits: Vec<FlipHit> = Vec::new();
    let mut nonzero: Vec<FlipHit> = Vec::new();

    for pos in &bins {
        let mut candidates: Vec<(u8, &'static str)> = Vec::new();
        if pos.bap > 0 {
            candidates.push((pos.bap - 1, "bap-1"));
        }
        if pos.bap < 15 {
            candidates.push((pos.bap + 1, "bap+1"));
        }
        if let Some(nb) = neighbor_bap_at(neighbor_state.as_ref(), pos.ch, pos.bin) {
            if nb != pos.bap {
                candidates.push((nb, "neighbor"));
            }
        }

        for (to_bap, label) in candidates {
            let wd = walk_delta(&state, &bsi, pos.ch, pos.bin, to_bap, base_walk);
            if wd == 0 {
                continue;
            }
            let hit = FlipHit {
                order: pos.order,
                ch: pos.ch,
                bin: pos.bin,
                from_bap: pos.bap,
                to_bap,
                walk_delta: wd,
                naive_delta: naive_bits_delta(pos.bap, to_bap),
                label,
            };
            nonzero.push(hit.clone());
            if gap == Some(wd) {
                hits.push(hit);
            }
        }
    }

    println!("--- exact gap closers (single-bin flip, grouping-aware walkΔ == gap) ---");
    if let Some(g) = gap {
        if hits.is_empty() {
            println!("  (none — gap {g:+} may need multiple bins or is not a pure BAP walk issue)");
        } else {
            println!(
                "{:>5} {:>6} {:>4} {:>4} {:>4} {:>4} {:>8} {:>8} note",
                "ord", "ch", "bin", "from", "to", "kind", "walkΔ", "naiveΔ"
            );
            for h in &hits {
                let note = if h.walk_delta != h.naive_delta {
                    "grouping"
                } else {
                    ""
                };
                println!(
                    "{:>5} {:>6} {:>4} {:>4} {:>4} {:>4} {:>+8} {:>+8} {note}",
                    h.order,
                    ch_label(h.ch),
                    h.bin,
                    h.from_bap,
                    h.to_bap,
                    h.label,
                    h.walk_delta,
                    h.naive_delta,
                );
            }
        }
    } else {
        println!("  (no inferred budget — skip)");
    }

    println!();
    println!("--- closest single-bin flips (|walkΔ − gap|) ---");
    if let Some(g) = gap {
        nonzero.sort_by_key(|h| (h.walk_delta - g).abs());
        let show = nonzero.len().min(24);
        println!(
            "{:>5} {:>6} {:>4} {:>4} {:>4} {:>4} {:>8} {:>8} {:>6}",
            "ord", "ch", "bin", "from", "to", "kind", "walkΔ", "naiveΔ", "|Δ−gap|"
        );
        for h in nonzero.iter().take(show) {
            println!(
                "{:>5} {:>6} {:>4} {:>4} {:>4} {:>4} {:>+8} {:>+8} {:>6}",
                h.order,
                ch_label(h.ch),
                h.bin,
                h.from_bap,
                h.to_bap,
                h.label,
                h.walk_delta,
                h.naive_delta,
                (h.walk_delta - g).abs(),
            );
        }
        if nonzero.len() > show {
            println!("... {} more nonzero flips", nonzero.len() - show);
        }
    }

    println!();
    println!("--- decode-order bins where naiveΔ ≠ walkΔ for bap±1 (grouping-sensitive) ---");
    let mut grouping_sensitive = 0usize;
    for pos in &bins {
        for (to_bap, label) in [
            (pos.bap.saturating_sub(1), "bap-1"),
            (pos.bap.saturating_add(1).min(15), "bap+1"),
        ] {
            if label == "bap-1" && pos.bap == 0 {
                continue;
            }
            if label == "bap+1" && pos.bap >= 15 {
                continue;
            }
            if to_bap == pos.bap {
                continue;
            }
            let wd = walk_delta(&state, &bsi, pos.ch, pos.bin, to_bap, base_walk);
            let nd = naive_bits_delta(pos.bap, to_bap);
            if wd != nd && wd != 0 {
                grouping_sensitive += 1;
                if grouping_sensitive <= 20 {
                    println!(
                        "  ord={} {} bin={} {}→{} {} walkΔ={wd:+} naiveΔ={nd:+}",
                        pos.order,
                        ch_label(pos.ch),
                        pos.bin,
                        pos.bap,
                        to_bap,
                        label,
                    );
                }
            }
        }
    }
    if grouping_sensitive > 20 {
        println!("  ... {grouping_sensitive} total grouping-sensitive flips");
    } else if grouping_sensitive == 0 {
        println!("  (none in this block)");
    }

    println!();
    println!("--- neighbor-only diffs with walkΔ (grouping-aware) ---");
    if neighbor_state.is_some() {
        let mut ndiffs: Vec<FlipHit> = Vec::new();
        for pos in &bins {
            let Some(nb) = neighbor_bap_at(neighbor_state.as_ref(), pos.ch, pos.bin) else {
                continue;
            };
            if nb == pos.bap {
                continue;
            }
            let wd = walk_delta(&state, &bsi, pos.ch, pos.bin, nb, base_walk);
            ndiffs.push(FlipHit {
                order: pos.order,
                ch: pos.ch,
                bin: pos.bin,
                from_bap: pos.bap,
                to_bap: nb,
                walk_delta: wd,
                naive_delta: naive_bits_delta(pos.bap, nb),
                label: "neighbor",
            });
        }
        ndiffs.sort_by_key(|h| h.order);
        let mut cum = 0i32;
        println!(
            "{:>5} {:>6} {:>4} {:>4} {:>4} {:>8} {:>8} {:>8}",
            "ord", "ch", "bin", "f0", "f1", "walkΔ", "naiveΔ", "cumΔ"
        );
        for h in &ndiffs {
            cum += h.walk_delta;
            let mark = if gap == Some(h.walk_delta) {
                " ← closes gap"
            } else {
                ""
            };
            println!(
                "{:>5} {:>6} {:>4} {:>4} {:>4} {:>+8} {:>+8} {:>+8}{mark}",
                h.order,
                ch_label(h.ch),
                h.bin,
                h.from_bap,
                h.to_bap,
                h.walk_delta,
                h.naive_delta,
                cum,
            );
        }
        println!("cumulative neighbor walkΔ if applied in isolation: {cum:+}");
        if let Some(g) = gap {
            println!("target gap: {g:+}");
        }
    } else {
        println!("  (no neighbor frame)");
    }
}
