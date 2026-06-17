//! Per-block per-channel blksw map for the first 12 frames (S=short, .=long).
use oxideav_ac3::audblk::parse_frame_side_info;
use oxideav_ac3::{bsi, syncinfo};
fn main() {
    let dir = std::env::var("TEMP").unwrap() + r"\oxideav_repro";
    let stem = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "white".to_string());
    let data = std::fs::read(format!("{dir}\\{stem}.ac3")).unwrap();
    let mut off = 0usize;
    for f in 0..12 {
        if off + 5 > data.len() {
            break;
        }
        let si = syncinfo::parse(&data[off..]).unwrap();
        let len = si.frame_length as usize;
        let frame = &data[off..off + len];
        let b = bsi::parse(&frame[5..]).unwrap();
        let blks = parse_frame_side_info(&si, &b, frame).unwrap();
        let nf = b.nfchans as usize;
        let s: Vec<String> = blks
            .iter()
            .enumerate()
            .map(|(bi, sb)| {
                format!(
                    "b{bi}[{}]",
                    sb.blksw[..nf]
                        .iter()
                        .map(|&x| if x { 'S' } else { '.' })
                        .collect::<String>()
                )
            })
            .collect();
        println!("frame {f:2}: {}", s.join(" "));
        off += len;
    }
}
