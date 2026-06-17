//! Search for the true blk0 mantissa length by parsing blocks 1–5 from
//! candidate bit offsets (no ffmpeg required).
//!
//! Usage:
//!   cargo run --release --example frame_align_solve -- <file.ac3> [frame] [mant_lo] [mant_hi]
//!
//! Defaults: frame 0, sweep `[walk-50, walk+50]` from blk0 audit.

use oxideav_ac3::audblk::{
    audit_frame_blocks, peek_bits_post_bsi, recount_block_mantissa_bits, state_after_block,
    try_parse_block_at, BLOCKS_PER_FRAME,
};
use oxideav_ac3::bsi;
use oxideav_ac3::syncinfo;

#[derive(Clone, Debug)]
struct BlockTail {
    blk: usize,
    side_info_bits: u64,
    mant_actual: u64,
    mant_walk: u32,
    bit_end: u64,
}

#[derive(Clone, Debug)]
struct Candidate {
    mant_end: u32,
    blk1_pos: u64,
    delta_vs_actual: i32,
    delta_vs_walk: i32,
    blocks_parsed: usize,
    tail: Vec<BlockTail>,
    frame_end: u64,
    unused: u64,
    cplstre_first_bit: u32,
    score: i32,
    err: Option<String>,
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

fn reference_unused_for_frame(data: &[u8], frame_index: usize) -> Option<u64> {
    let frame = nth_frame(data, frame_index)?;
    let si = syncinfo::parse(&frame).ok()?;
    let bsi = bsi::parse(&frame[5..]).ok()?;
    let frame_bits = (frame.len() - 5) as u64 * 8;
    let audits = audit_frame_blocks(&si, &bsi, &frame).ok()?;
    if audits.len() == BLOCKS_PER_FRAME && audits.iter().all(|a| a.parse_ok) {
        Some(frame_bits - audits[BLOCKS_PER_FRAME - 1].bit_block_end)
    } else {
        None
    }
}

fn score_candidate(c: &Candidate, ref_unused: Option<u64>) -> i32 {
    if c.tail.is_empty() {
        return -(c.mant_end as i32);
    }

    let mut s = 0i32;
    let t0 = &c.tail[0];

    // blk1 must look like a normal reuse block (frame 1 template: side=15).
    if t0.side_info_bits != 15 {
        return 100 + c.blocks_parsed as i32;
    }
    s += 5_000;

    if t0.mant_actual != u64::from(t0.mant_walk) {
        return 200 + c.blocks_parsed as i32;
    }
    s += 2_000;

    if !(2_800..=3_400).contains(&t0.mant_walk) {
        s -= 1_000;
    }

    if c.cplstre_first_bit != 0 {
        s -= 800;
    } else {
        s += 400;
    }

    for t in c.tail.iter().skip(1) {
        if t.side_info_bits == 15 {
            s += 600;
        }
        if t.mant_actual == u64::from(t.mant_walk) {
            s += 300;
        }
        if !(2_800..=3_400).contains(&t.mant_walk) {
            s -= 400;
        }
    }

    if c.blocks_parsed == BLOCKS_PER_FRAME - 1 {
        s += 3_000;
    } else {
        s += c.blocks_parsed as i32 * 200;
    }

    if let Some(ref_u) = ref_unused {
        let diff = (c.unused as i64 - ref_u as i64).unsigned_abs();
        if diff <= 200 {
            s += 2_000 - diff as i32;
        } else {
            s -= diff.min(10_000) as i32;
        }
    }

  s
}

fn try_candidate(
    si: &syncinfo::SyncInfo,
    bsi: &bsi::Bsi,
    frame: &[u8],
    base_state: &oxideav_ac3::audblk::Ac3State,
    pre_mantissa: u64,
    mant_end: u32,
    frame_bits: u64,
    actual_mant: u32,
    walk_mant: u32,
    ref_unused: Option<u64>,
) -> Candidate {
    let blk1_pos = pre_mantissa + u64::from(mant_end);
    let cplstre_first_bit = peek_bits_post_bsi(frame, bsi, blk1_pos, 1);

    let mut state = base_state.clone();
    let mut pos = blk1_pos;
    let mut tail = Vec::new();
    let mut err = None;

    for blk in 1..BLOCKS_PER_FRAME {
        let block_start = pos;
        match try_parse_block_at(si, bsi, frame, &mut state, blk, pos) {
            Ok(m) => {
                let walk = recount_block_mantissa_bits(&state, bsi);
                let side_info_bits = m.bit_pre_mantissa.saturating_sub(block_start);
                let mant_actual = m.bit_block_end.saturating_sub(m.bit_pre_mantissa);
                tail.push(BlockTail {
                    blk,
                    side_info_bits,
                    mant_actual,
                    mant_walk: walk,
                    bit_end: m.bit_block_end,
                });
                pos = m.bit_block_end;
            }
            Err(e) => {
                err = Some(e.to_string());
                break;
            }
        }
    }

    let frame_end = tail.last().map(|t| t.bit_end).unwrap_or(blk1_pos);
    let unused = frame_bits.saturating_sub(frame_end);

    let mut c = Candidate {
        mant_end,
        blk1_pos,
        delta_vs_actual: mant_end as i32 - actual_mant as i32,
        delta_vs_walk: mant_end as i32 - walk_mant as i32,
        blocks_parsed: tail.len(),
        tail,
        frame_end,
        unused,
        cplstre_first_bit,
        score: 0,
        err,
    };
    c.score = score_candidate(&c, ref_unused);
    c
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: frame_align_solve <file.ac3> [frame] [mant_lo] [mant_hi]");
    let frame_index: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    let data = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let frame = nth_frame(&data, frame_index)
        .unwrap_or_else(|| panic!("frame {frame_index} missing in {path}"));
    let si = syncinfo::parse(&frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();
    let frame_bits = (frame.len() - 5) as u64 * 8;

    let audits = audit_frame_blocks(&si, &bsi, &frame).expect("audit blk0");
    let a0 = audits
        .first()
        .filter(|a| a.parse_ok)
        .expect("blk0 must parse for alignment search");

    let pre_mantissa = a0.bit_pre_mantissa;
    let actual_mant = a0.mantissa_bits_actual as u32;
    let walk_mant = a0.mantissa_bits_walk;
    let blk0_end = a0.bit_block_end;

    let mant_lo: u32 = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(walk_mant.saturating_sub(50));
    let mant_hi: u32 = std::env::args()
        .nth(4)
        .and_then(|s| s.parse().ok())
        .unwrap_or(walk_mant + 50);

    let ref_unused = if frame_index == 0 {
        reference_unused_for_frame(&data, 1)
    } else {
        reference_unused_for_frame(&data, frame_index)
    };

    let base_state =
        state_after_block(&si, &bsi, &frame, 0).expect("state after blk0");

    println!("file: {path}");
    println!("frame: {frame_index}  frame_bits: {frame_bits}");
    println!(
        "blk0: pre_mantissa={pre_mantissa} actual_mant={actual_mant} walk={walk_mant} end={blk0_end}"
    );
    println!("sweep mant_end in [{mant_lo}, {mant_hi}]");
    if let Some(u) = ref_unused {
        println!("reference unused tail (next clean frame): {u} bits");
    } else {
        println!("reference unused tail: (no clean reference frame in file)");
    }
    println!();

    let mut candidates: Vec<Candidate> = (mant_lo..=mant_hi)
        .map(|mant_end| {
            try_candidate(
                &si,
                &bsi,
                &frame,
                &base_state,
                pre_mantissa,
                mant_end,
                frame_bits,
                actual_mant,
                walk_mant,
                ref_unused,
            )
        })
        .collect();

    candidates.sort_by(|a, b| b.score.cmp(&a.score).then_with(|| a.mant_end.cmp(&b.mant_end)));

    println!("--- ranked candidates (top 25) ---");
    println!(
        "{:>6} {:>6} {:>6} {:>5} {:>5} {:>6} {:>7} {:>7} {:>4} {}",
        "score", "mant", "d_act", "d_wlk", "blk1@", "parsed", "unused", "side15", "cpl0", "notes"
    );
    for c in candidates.iter().take(25) {
        let side15 = c
            .tail
            .iter()
            .filter(|t| t.side_info_bits == 15)
            .count();
        let note = if c.mant_end == actual_mant {
            "← live decoder".to_string()
        } else if c.tail.first().is_some_and(|t| t.side_info_bits == 15)
            && c.blocks_parsed == BLOCKS_PER_FRAME - 1
        {
            "plausible tail".to_string()
        } else if let Some(ref e) = c.err {
            e.chars().take(48).collect()
        } else if let Some(t0) = c.tail.first() {
            format!("blk1 side={}", t0.side_info_bits)
        } else {
            String::new()
        };
        println!(
            "{:>6} {:>6} {:>+6} {:>+5} {:>5} {:>6} {:>7} {:>7} {:>4} {note}",
            c.score,
            c.mant_end,
            c.delta_vs_actual,
            c.delta_vs_walk,
            c.blk1_pos,
            c.blocks_parsed,
            c.unused,
            side15,
            c.cplstre_first_bit,
        );
    }

    if let Some(best) = candidates.first() {
        println!();
        println!(
            "best: mant_end={} blk1@={} frame_end={} score={}",
            best.mant_end, best.blk1_pos, best.frame_end, best.score
        );
        if best.mant_end != actual_mant {
            println!(
                "live decoder uses mant_end={actual_mant} (delta {:+} vs best)",
                best.mant_end as i32 - actual_mant as i32
            );
        }
        for t in &best.tail {
            println!(
                "  blk{}: side={} mant_actual={} mant_walk={} end={}",
                t.blk, t.side_info_bits, t.mant_actual, t.mant_walk, t.bit_end
            );
        }
        if let Some(e) = &best.err {
            println!("  tail stopped: {e}");
        }
    }

    let strict_blk1: Vec<_> = candidates
        .iter()
        .filter(|c| {
            c.tail.first().is_some_and(|t| {
                t.side_info_bits == 15 && t.mant_actual == u64::from(t.mant_walk)
            })
        })
        .collect();
    if !strict_blk1.is_empty() {
        println!();
        println!("--- candidates with valid blk1 (side=15, mant walk=actual) ---");
        println!(
            "{:>6} {:>6} {:>6} {:>6} {:>5} {:>7} {:>4}",
            "mant", "d_act", "blk1@", "parsed", "unused", "cpl0", "note"
        );
        for c in strict_blk1.iter().take(20) {
            let note = if c.mant_end == actual_mant {
                "live"
            } else if c.unused <= 500 {
                "tight unused"
            } else {
                ""
            };
            println!(
                "{:>6} {:>+6} {:>6} {:>5} {:>7} {:>4} {note}",
                c.mant_end,
                c.delta_vs_actual,
                c.blk1_pos,
                c.blocks_parsed,
                c.unused,
                c.cplstre_first_bit,
            );
        }
    }

    let tight: Vec<_> = strict_blk1
        .iter()
        .filter(|c| {
            c.blocks_parsed == BLOCKS_PER_FRAME - 1
                && c.unused <= ref_unused.unwrap_or(500) + 200
        })
        .collect();
    println!();
    if tight.is_empty() {
        println!("no candidate in sweep achieves full tail + tight unused (±200 of ref).");
        println!("→ likely no single mant_end fixes frame 0; investigate blk0 bap/unpack or state.");
    } else {
        println!("tight full-tail candidates: {}", tight.len());
    }
}
