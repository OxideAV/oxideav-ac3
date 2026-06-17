//! Field-by-field side-info bit cursor trace (§5.4.3).
//!
//! Usage:
//!   cargo run --example side_info_trace -- <file.ac3> [frame_a] [frame_b]
//!
//! Default: compare frames 2 (bad) vs 3 (good), block 0.

use oxideav_ac3::audblk::{peek_bits_post_bsi, trace_block_side_info, try_parse_next_block, SideInfoMark};
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

fn print_trace(label: &str, marks: &[SideInfoMark]) {
    println!("--- {label} ---");
    for m in marks {
        let detail = m.detail.as_deref().unwrap_or("");
        if detail.is_empty() {
            println!("  {:>6}  {}", m.bit_pos, m.label);
        } else {
            println!("  {:>6}  {}  ({detail})", m.bit_pos, m.label);
        }
    }
}

fn diff_traces(a: &[SideInfoMark], b: &[SideInfoMark]) {
    println!("--- cursor delta (B - A) ---");
    let n = a.len().min(b.len());
    for i in 0..n {
        let d = b[i].bit_pos as i64 - a[i].bit_pos as i64;
        if d != 0 || a[i].label != b[i].label {
            println!(
                "  {:>6}  {} vs {}  Δ={d:+}",
                a[i].bit_pos, a[i].label, b[i].label
            );
        }
    }
    if a.len() != b.len() {
        println!("  (trace lengths differ: {} vs {})", a.len(), b.len());
    }
}

fn main() {
    let path = std::env::args().nth(1).expect("usage: side_info_trace <file.ac3> [a] [b]");
    let fa: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(2);
    let fb: usize = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(3);
    let data = std::fs::read(&path).unwrap();

    let frame_a = nth_frame(&data, fa).unwrap_or_else(|| panic!("frame {fa} missing"));
    let frame_b = nth_frame(&data, fb).unwrap_or_else(|| panic!("frame {fb} missing"));
    let si_a = syncinfo::parse(&frame_a).unwrap();
    let bsi_a = bsi::parse(&frame_a[5..]).unwrap();
    let si_b = syncinfo::parse(&frame_b).unwrap();
    let bsi_b = bsi::parse(&frame_b[5..]).unwrap();

    let trace_a = trace_block_side_info(&si_a, &bsi_a, &frame_a, 0).expect("trace A");
    let trace_b = trace_block_side_info(&si_b, &bsi_b, &frame_b, 0).expect("trace B");

    print_trace(&format!("frame {fa}"), &trace_a);
    println!();
    print_trace(&format!("frame {fb}"), &trace_b);
    println!();
    diff_traces(&trace_a, &trace_b);

    for (fi, frame, bsi, trace) in [
        (fa, &frame_a, &bsi_a, &trace_a),
        (fb, &frame_b, &bsi_b, &trace_b),
    ] {
        if let Some(m) = trace.iter().find(|m| m.label == "after_fsnroffst_ch0") {
            let pos = m.bit_pos.saturating_sub(7);
            let raw7 = peek_bits_post_bsi(frame, bsi, pos, 7);
            let fsnr = raw7 >> 3;
            let fgain = raw7 & 0x7;
            println!();
            println!(
                "frame {fi}: bits@{pos} raw7={raw7:#09b} → fsnroffst={fsnr} fgaincod={fgain} (parser after_ch0 mark at {})",
                m.bit_pos
            );
        }
        if let Some(m) = trace.iter().find(|m| m.label == "after_fsnroffst_ch1") {
            let pos = m.bit_pos.saturating_sub(7);
            let raw7 = peek_bits_post_bsi(frame, bsi, pos, 7);
            let fsnr = raw7 >> 3;
            let fgain = raw7 & 0x7;
            println!(
                "frame {fi}: bits@{pos} raw7={raw7:#09b} → fsnroffst={fsnr} fgaincod={fgain} (parser after_ch1 mark at {})",
                m.bit_pos
            );
        }
    }

    println!();
    println!("--- blk1 parse probe (after blk0) ---");
    for (fi, frame, si, bsi) in [
        (fa, &frame_a, &si_a, &bsi_a),
        (fb, &frame_b, &si_b, &bsi_b),
    ] {
        for delta in [0i32, 1, 36] {
            let (ok, end, err) = try_parse_next_block(si, bsi, frame, 0, delta);
            let base = trace_block_side_info(si, bsi, frame, 0)
                .ok()
                .and_then(|t| t.last().map(|_| 0))
                .unwrap_or(0);
            let _ = base;
            if ok {
                println!("frame {fi} delta={delta:+3}: OK end_bit={end}");
            } else {
                println!(
                    "frame {fi} delta={delta:+3}: FAIL end_bit={end} err={}",
                    err.unwrap_or_default()
                );
            }
        }
    }
}
