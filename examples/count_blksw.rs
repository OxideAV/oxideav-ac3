//! Count per-channel `blksw` (block-switch) bits in every audio block of
//! an AC-3 elementary stream. Useful for confirming that a fixture
//! actually exercises the short-block IMDCT path (§7.9.4.2) rather
//! than only the long path (§7.9.4.1).
//!
//! Usage: `cargo run --example count_blksw -- path/to/file.ac3`

// reason: channel loops use an index both to read `blksw[ch]` and update
// `short_by_ch[ch]`; rewriting to zipped iterators is noisier than it's worth
// for a debug example.
#![allow(clippy::needless_range_loop)]

use std::env;
use std::fs;

use oxideav_ac3::{audblk, bsi, syncinfo};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args().nth(1).ok_or("usage: count_blksw <file.ac3>")?;
    let data = fs::read(&path)?;

    let mut offset = 0usize;
    let mut frame_idx = 0usize;
    let mut total_blocks = 0usize;
    let mut short_blocks = 0usize;
    let mut short_by_ch = [0usize; 5];
    while offset < data.len() {
        let si = syncinfo::parse(&data[offset..])?;
        let flen = si.frame_length as usize;
        if offset + flen > data.len() {
            break;
        }
        let b = bsi::parse(&data[offset + 5..])?;
        let frame = &data[offset..offset + flen];
        let side = audblk::parse_frame_side_info(&si, &b, frame)?;
        for (i, s) in side.iter().enumerate() {
            total_blocks += 1;
            let mut any_short = false;
            for ch in 0..b.nfchans as usize {
                if s.blksw[ch] {
                    any_short = true;
                    short_by_ch[ch] += 1;
                }
            }
            if any_short {
                short_blocks += 1;
                eprintln!(
                    "frame {frame_idx} blk {i}: blksw = {:?}",
                    &s.blksw[..b.nfchans as usize]
                );
            }
        }
        offset += flen;
        frame_idx += 1;
    }
    println!("frames={frame_idx} total_blocks={total_blocks} short_blocks={short_blocks}");
    println!("short by channel: {:?}", &short_by_ch[..]);
    Ok(())
}
