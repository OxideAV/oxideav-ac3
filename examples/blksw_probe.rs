//! Block-switch (blksw) census per stem: counts how many (frame,blk,ch)
//! audio blocks use short blocks vs long. If white uses short blocks where
//! sine/pink/multitone use long, the bug is in the short-block IMDCT path.
//!
//! Run: cargo run --example blksw_probe
use oxideav_ac3::audblk::parse_frame_side_info;
use oxideav_ac3::{bsi, syncinfo};

fn census(stem: &str) {
    let dir = std::env::var("TEMP").unwrap() + r"\oxideav_repro";
    let path = format!("{dir}\\{stem}.ac3");
    let Ok(data) = std::fs::read(&path) else {
        println!("{stem}: (no file)");
        return;
    };
    let (mut off, mut frames, mut blocks, mut short, mut frames_with_short) = (0, 0u32, 0u32, 0u32, 0u32);
    while off + 5 <= data.len() {
        let Ok(si) = syncinfo::parse(&data[off..]) else { break };
        let len = si.frame_length as usize;
        if len < 5 || off + len > data.len() {
            break;
        }
        let frame = &data[off..off + len];
        if let Ok(b) = bsi::parse(&frame[5..]) {
            if let Ok(blks) = parse_frame_side_info(&si, &b, frame) {
                let nf = b.nfchans as usize;
                let mut this_frame_short = 0u32;
                for sb in blks.iter() {
                    for &sw in sb.blksw[..nf].iter() {
                        blocks += 1;
                        if sw {
                            short += 1;
                            this_frame_short += 1;
                        }
                    }
                }
                if this_frame_short > 0 {
                    frames_with_short += 1;
                }
            }
        }
        frames += 1;
        off += len;
    }
    println!(
        "{stem:10} frames={frames:4} ch-blocks={blocks:5} short={short:5} ({:.1}%) frames_with_short={frames_with_short}",
        100.0 * short as f64 / blocks.max(1) as f64
    );
}

fn per_frame(stem: &str, nframes: usize) {
    let dir = std::env::var("TEMP").unwrap() + r"\oxideav_repro";
    let path = format!("{dir}\\{stem}.ac3");
    let Ok(data) = std::fs::read(&path) else { return };
    let mut off = 0usize;
    print!("{stem} short-block frames (of first {nframes}): ");
    for f in 0..nframes {
        if off + 5 > data.len() {
            break;
        }
        let Ok(si) = syncinfo::parse(&data[off..]) else { break };
        let len = si.frame_length as usize;
        let frame = &data[off..off + len];
        let b = bsi::parse(&frame[5..]).unwrap();
        let blks = parse_frame_side_info(&si, &b, frame).unwrap();
        let nf = b.nfchans as usize;
        let short: u32 = blks.iter().map(|sb| sb.blksw[..nf].iter().filter(|&&x| x).count() as u32).sum();
        if short > 0 {
            print!("{f}({short}) ");
        }
        off += len;
    }
    println!();
}

fn main() {
    for stem in ["sine", "pink", "multi", "white"] {
        census(stem);
    }
    println!();
    per_frame("white", 12);
}
