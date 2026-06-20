//! Diagnostic: decode an E-AC-3 elementary stream frame-by-frame and
//! dump per-frame shape. Set `EAC3_DUMP_BLK=1` for per-block detail.
//! Usage: `cargo run --example eac3_dump -- <file.eac3>`

use oxideav_ac3::eac3::{decode_eac3_packet, Eac3DecoderState};

fn find_frames(data: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut off = 0usize;
    while off + 4 <= data.len() {
        if data[off] != 0x0B || data[off + 1] != 0x77 {
            off += 1;
            continue;
        }
        let frmsiz = (((data[off + 2] & 0x07) as u32) << 8) | data[off + 3] as u32;
        let flen = ((frmsiz + 1) * 2) as usize;
        if flen < 6 || off + flen > data.len() {
            break;
        }
        out.push((off, flen));
        off += flen;
    }
    out
}

fn main() {
    let path = std::env::args().nth(1).expect("usage: eac3_dump <file>");
    let data = std::fs::read(&path).expect("read input");
    let frames = find_frames(&data);
    eprintln!("frames={}", frames.len());
    let mut state = Eac3DecoderState::default();
    let max = std::env::var("EAC3_MAX_FRAMES")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(2);
    for (i, &(start, len)) in frames.iter().enumerate().take(max) {
        eprintln!("=== FRAME {i} @ {start} len {len} ===");
        match decode_eac3_packet(&mut state, &data[start..start + len]) {
            Ok(f) => eprintln!(
                "  ok ch={} sr={} samples={} pcm_len={}",
                f.channels,
                f.sample_rate,
                f.samples,
                f.pcm_s16le.len()
            ),
            Err(e) => eprintln!("  err {e}"),
        }
    }
}
