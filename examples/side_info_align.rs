//! Bisect true blk0 pre-mantissa bit offset by parsing tail blocks.
//!
//! Usage: cargo run --release --example side_info_align -- <file.ac3> [frame]

use oxideav_ac3::audblk::{
    peek_bits_post_bsi, recount_block_mantissa_bits, state_after_block, try_parse_block_at,
    BLOCKS_PER_FRAME,
};
use oxideav_ac3::{bsi, syncinfo};

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
    let path = std::env::args().nth(1).expect("usage: side_info_align <file.ac3> [frame]");
    let fi: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let data = std::fs::read(&path).unwrap();
    let frame = nth_frame(&data, fi).unwrap_or_else(|| panic!("frame {fi} missing"));
    let si = syncinfo::parse(&frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();
    let frame_bits = (frame.len() - 5) as u64 * 8;

    let base_state = state_after_block(&si, &bsi, &frame, 0).expect("blk0 state");
    let marks = oxideav_ac3::audblk::trace_block_side_info(&si, &bsi, &frame, 0).unwrap();
    let reported = marks
        .iter()
        .find(|m| m.label == "pre_mantissa")
        .map(|m| m.bit_pos)
        .unwrap_or(0);

    println!("file: {path}  frame: {fi}");
    println!("reported pre_mantissa={reported}  frame_bits={frame_bits}");
    println!();
    println!("pre_mant  mant_end  blk1@  cpl0  side1  walk1  parsed  unused  ok");

    for pre_delta in -40i32..=10 {
        let pre_mant = (reported as i64 + i64::from(pre_delta)).max(0) as u64;
        for mant_end in [3220u32, 3194, 3175] {
            let blk1_pos = pre_mant + u64::from(mant_end);
            let cpl0 = peek_bits_post_bsi(&frame, &bsi, blk1_pos, 1);
            let mut state = base_state.clone();
            let mut pos = blk1_pos;
            let mut tail_ok = true;
            let mut side1 = 0u64;
            let mut walk1 = 0u32;
            for blk in 1..BLOCKS_PER_FRAME {
                match try_parse_block_at(&si, &bsi, &frame, &mut state, blk, pos) {
                    Ok(m) => {
                        if blk == 1 {
                            side1 = m.bit_pre_mantissa.saturating_sub(pos);
                            walk1 = recount_block_mantissa_bits(&state, &bsi);
                        }
                        pos = m.bit_block_end;
                    }
                    Err(_) => {
                        tail_ok = false;
                        break;
                    }
                }
            }
            let unused = frame_bits.saturating_sub(pos);
            let good = tail_ok && side1 == 15 && walk1 == 3220 || walk1 == 3197;
            if good || pre_delta == 0 || pre_delta == -26 {
                println!(
                    "{pre_mant:8}  {mant_end:7}  {blk1_pos:5}  {cpl0:4}  {side1:5}  {walk1:5}  {}  {unused:6}  {}",
                    if tail_ok { BLOCKS_PER_FRAME - 1 } else { 0 },
                    if tail_ok { "ok" } else { "fail" }
                );
            }
        }
    }
}
