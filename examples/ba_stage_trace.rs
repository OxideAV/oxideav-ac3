//! Dump per-block side-info, bit_alloc_stages (via AC3_TRACE_STAGE), and
//! bit-cursor audit for one frame.
//!
//! Usage:
//!   cargo run --example ba_stage_trace -- <file.ac3> [frame_index]
//!   AC3_TRACE_STAGE=1 AC3_TRACE_FRAME=0 cargo run --example ba_stage_trace -- white.ac3 0

use oxideav_ac3::audblk::{audit_frame_blocks, parse_frame_side_info, peek_bits_post_bsi, try_parse_next_block};
use oxideav_ac3::bsi;
use oxideav_ac3::syncinfo;

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

fn main() {
    let path = std::env::args().nth(1).expect("usage: ba_stage_trace <file.ac3> [frame]");
    let fi: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let data = std::fs::read(&path).unwrap();
    let frame = nth_frame(&data, fi).unwrap_or_else(|| panic!("frame {fi} missing"));
    let si = syncinfo::parse(&frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();
    let frame_bits = (frame.len() - 5) * 8;
    let sides = parse_frame_side_info(&si, &bsi, &frame).expect("side-info parse");

    println!("frame {fi} side-info:");
    for (blk, s) in sides.iter().enumerate() {
        println!(
            "  blk{blk}: cplstre={} chexpstr={:?} baie={} snroffste={} deltbaie={} blksw={:?}",
            s.cplstre,
            &s.chexpstr[..bsi.nfchans as usize],
            s.baie,
            s.snroffste,
            s.deltbaie,
            &s.blksw[..bsi.nfchans as usize],
        );
    }

    let audits = audit_frame_blocks(&si, &bsi, &frame).expect("audit");
    println!();
    println!("--- bit audit ---");
    for a in &audits {
        let delta = if a.blk == 0 {
            a.bit_block_end
        } else {
            a.bit_block_end - audits[a.blk - 1].bit_block_end
        };
        println!(
            "  blk{}: ok={} end={} delta={delta} side={} mant_actual={} mant_walk={} fsnroffst={:?}",
            a.blk,
            a.parse_ok,
            a.bit_block_end,
            a.side_info_bits,
            a.mantissa_bits_actual,
            a.mantissa_bits_walk,
            &a.fsnroffst[..bsi.nfchans as usize],
        );
    }
    if let Some(last) = audits.last() {
        println!(
            "  frame_bits={frame_bits} unused={}",
            frame_bits as u64 - last.bit_block_end
        );
    }

    if let Some(a0) = audits.first() {
        let pos = a0.bit_block_end;
        println!();
        println!("--- bits at frame {fi} blk0 end ({pos}) ---");
        for d in -4i32..=8 {
            let p = (pos as i64 + i64::from(d)) as u64;
            let w = peek_bits_post_bsi(&frame, &bsi, p, 16);
            let first = (w >> 15) & 1;
            println!("  @{p}: first_bit={first} word={w:#018b}");
        }
    }
    let (ok, end, err) = try_parse_next_block(&si, &bsi, &frame, 0, 0);
    println!();
    println!("blk1 probe delta=0: ok={ok} end_bit={end} err={}", err.unwrap_or_default());
    for d in [-26i32, -1, 0, 23, 24, 25, 26, 27] {
        let (ok, end, err) = try_parse_next_block(&si, &bsi, &frame, 0, d);
        println!(
            "blk1 probe delta={d:+3}: ok={ok} end_bit={end} err={}",
            err.unwrap_or_default()
        );
    }
}
