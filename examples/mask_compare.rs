//! Compare oxideav mask/BAP vs FFmpeg-reference on a frame block.
//!
//! Usage:
//!   cargo run --example mask_compare -- <file.ac3> [frame] [blk]

use oxideav_ac3::audblk::{
    compare_channel_ba_ref, probe_blk1_after_fsnroffst, probe_blk1_after_fsnroffst_stream,
    state_after_block,
};
use oxideav_ac3::bsi;
use oxideav_ac3::syncinfo;

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

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: mask_compare <file.ac3> [frame] [blk]");
        std::process::exit(1);
    });
    let frame_idx: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(2);
    let blk: usize = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);

    let data = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let (off, len) = nth_frame(&data, frame_idx).unwrap_or_else(|| panic!("frame {frame_idx}"));
    let frame = &data[off..off + len];
    let si = syncinfo::parse(frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();

    let state = state_after_block(&si, &bsi, frame, blk).unwrap_or_else(|e| panic!("{e}"));
    let nfchans = bsi.nfchans as usize;

    println!(
        "frame {frame_idx} blk {blk} nfchans={nfchans} fsnroffst={:?}",
        &state.fsnroffst[..nfchans]
    );

    for ch in 0..nfchans {
        let cmp = compare_channel_ba_ref(&state, &si, ch, blk);
        println!(
            "ch{ch}: end={} fsnroffst={} snroffset={:#x} floor={:#x}",
            cmp.end, cmp.fsnroffst, cmp.snroffset, cmp.floor
        );
        println!(
            "  mantissa bits: ours={} ref={} delta={}",
            cmp.ours_mant_bits,
            cmp.ref_mant_bits,
            cmp.ours_mant_bits as i32 - cmp.ref_mant_bits as i32
        );
        if cmp.mask_diff_bands.is_empty() {
            println!("  mask: MATCH (all bands)");
        } else {
            println!("  mask diffs ({} bands):", cmp.mask_diff_bands.len());
            for (band, ours, rf) in cmp.mask_diff_bands.iter().take(20) {
                println!("    band {band}: ours={ours} ref={rf} delta={}", ours - rf);
            }
            if cmp.mask_diff_bands.len() > 20 {
                println!("    ... {} more", cmp.mask_diff_bands.len() - 20);
            }
        }
        if cmp.bap_diff_bins.is_empty() {
            println!("  bap: MATCH (all bins)");
        } else {
            println!("  bap diffs ({} bins):", cmp.bap_diff_bins.len());
            for (bin, ours, rf) in cmp.bap_diff_bins.iter().take(30) {
                println!("    bin {bin}: ours={ours} ref={rf}");
            }
            if cmp.bap_diff_bins.len() > 30 {
                println!("    ... {} more", cmp.bap_diff_bins.len() - 30);
            }
        }
    }

    println!();
    println!("--- blk1 delta=0 probe vs fsnroffst (frame {frame_idx} blk{blk}) ---");
    let base = probe_blk1_after_fsnroffst(&si, &bsi, frame, blk, None).unwrap();
    println!(
        "baseline {:?} → mant={} blk1_ok={}",
        &base.fsnroffst[..nfchans],
        base.mantissa_bits,
        base.blk1_parse_ok
    );
    for ch1_fsnr in 13u8..=15 {
        let mut fsnr = state.fsnroffst;
        fsnr[1] = ch1_fsnr;
        let r = probe_blk1_after_fsnroffst(&si, &bsi, frame, blk, Some(fsnr)).unwrap();
        println!(
            "  ch1={ch1_fsnr} (isolated) → mant={} blk1_ok={}",
            r.mantissa_bits, r.blk1_parse_ok
        );
    }

    println!();
    println!("--- with stream state (frames 0..{}) ---", frame_idx);
    let data = std::fs::read(&path).unwrap();
    let stream_base = probe_blk1_after_fsnroffst_stream(&data, frame_idx, blk, None).unwrap();
    println!(
        "baseline {:?} → mant={} blk1_ok={}",
        &stream_base.fsnroffst[..nfchans],
        stream_base.mantissa_bits,
        stream_base.blk1_parse_ok
    );
    for ch1_fsnr in 13u8..=15 {
        let mut fsnr = stream_base.fsnroffst;
        fsnr[1] = ch1_fsnr;
        let r = probe_blk1_after_fsnroffst_stream(&data, frame_idx, blk, Some(fsnr)).unwrap();
        println!(
            "  ch1={ch1_fsnr} (stream) → mant={} blk1_ok={}",
            r.mantissa_bits, r.blk1_parse_ok
        );
    }
}
