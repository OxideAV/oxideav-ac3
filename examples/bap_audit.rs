//! Per-block BAP + mantissa bit audit vs ffmpeg encoder pack schedule.
//!
//! Usage:
//!   cargo run --example bap_audit -- <file.ac3> [frame_index]
//!
//! Default `frame_index` is 2 (first bad PCM frame on multitone @ 640k).

use oxideav_ac3::audblk::{
    audit_frame_blocks, diff_bap_bins, probe_next_block_alignment, state_after_block,
    try_parse_block_at, BlockBitAudit,
};
use oxideav_ac3::bsi;
use oxideav_ac3::syncinfo;

fn print_bap_histo(label: &str, h: &[u32; 16]) {
    print!("{label}:");
    for (b, n) in h.iter().enumerate() {
        if *n > 0 {
            print!(" bap{b}={n}");
        }
    }
    println!();
}

fn print_block(a: &BlockBitAudit) {
    println!(
        "blk{} parse_ok={} bits: start={} pre_mant={} end={} | side={} mant_actual={} walk={} pack={} histo={} | delta(actual-pack)={}",
        a.blk,
        a.parse_ok,
        a.bit_block_start,
        a.bit_pre_mantissa,
        a.bit_block_end,
        a.side_info_bits,
        a.mantissa_bits_actual,
        a.mantissa_bits_walk,
        a.mantissa_bits_pack,
        a.mantissa_bits_histo,
        a.mantissa_bits_actual as i64 - i64::from(a.mantissa_bits_pack),
    );
    for ch in 0..2 {
        println!(
            "  ch{ch}: end_mant={} fsnroffst={}",
            a.end_mant[ch], a.fsnroffst[ch]
        );
        print!("  ");
        print_bap_histo(&format!("ch{ch} histo"), &a.bap_histo_ch[ch]);
    }
    print!("  ");
    print_bap_histo("combined", &a.bap_histo_combined);
}

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
    let path = std::env::args()
        .nth(1)
        .expect("usage: bap_audit <file.ac3> [frame]");
    let frame_index: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(2);
    let data = std::fs::read(&path).unwrap();
    let (off, len) = nth_frame(&data, frame_index)
        .unwrap_or_else(|| panic!("frame {frame_index} not found in {path}"));
    let frame = &data[off..off + len];
    let si = syncinfo::parse(frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();
    let frame_bits = (len as u64 - 5) * 8;

    println!("{path} frame {frame_index}: {len} bytes ({frame_bits} bits post-sync)");
    println!(
        "acmod={} nfchans={} lfeon={} bsi_bits={}",
        bsi.acmod, bsi.nfchans, bsi.lfeon, bsi.bits_consumed
    );

    let audits = audit_frame_blocks(&si, &bsi, frame).unwrap();
    for a in &audits {
        print_block(a);
    }

    if audits.len() >= 2 {
        let b0 = &audits[0];
        let b1 = &audits[1];
        println!();
        println!(
            "blk0→blk1 cursor: {} → {} (blk1 consumed {} bits, expected pack {} from blk0 mantissa end)",
            b0.bit_block_end,
            b1.bit_block_end,
            b1.bit_block_end.saturating_sub(b0.bit_block_end),
            b0.mantissa_bits_pack,
        );
        println!(
            "blk0 mantissa gap: actual={} pack={} walk={} histo={} (pack-actual={})",
            b0.mantissa_bits_actual,
            b0.mantissa_bits_pack,
            b0.mantissa_bits_walk,
            b0.mantissa_bits_histo,
            i64::from(b0.mantissa_bits_pack) - b0.mantissa_bits_actual as i64,
        );
    }

    if let Some((off2, len2)) = nth_frame(&data, frame_index + 1) {
        let frame2 = &data[off2..off2 + len2];
        let si2 = syncinfo::parse(frame2).unwrap();
        let bsi2 = bsi::parse(&frame2[5..]).unwrap();
        let audits2 = audit_frame_blocks(&si2, &bsi2, frame2).unwrap();
        if let (Some(a_bad), Some(a_good)) = (audits.first(), audits2.first()) {
            println!();
            println!(
                "--- frame {frame_index} blk0 vs frame {} blk0 ---",
                frame_index + 1
            );
            println!("end_mant: {:?} vs {:?}", a_bad.end_mant, a_good.end_mant);
            println!(
                "mantissa: actual {} vs {} | pack {} vs {} | walk {} vs {}",
                a_bad.mantissa_bits_actual,
                a_good.mantissa_bits_actual,
                a_bad.mantissa_bits_pack,
                a_good.mantissa_bits_pack,
                a_bad.mantissa_bits_walk,
                a_good.mantissa_bits_walk,
            );
            for ch in 0..2 {
                let end = a_bad.end_mant[ch].min(a_good.end_mant[ch]);
                let diffs = diff_bap_bins(&a_bad.bap_ch[ch], &a_good.bap_ch[ch], end);
                print!("  ch{ch}: {} bins differ", diffs.len());
                if !diffs.is_empty() {
                    print!(" e.g.");
                    for (bin, a, b) in diffs.iter().take(20) {
                        print!(" [{bin}] {a}→{b}");
                    }
                }
                println!();
            }
        }
    }

    if let Some(b0) = audits.first() {
        let h = &b0.bap_histo_combined;
        let c1 = h[1];
        let c2 = h[2];
        let c4 = h[4];
        let pad1 = u32::from(c1 % 3 != 0) * 5;
        let pad2 = u32::from(c2 % 3 != 0) * 7;
        let pad4 = u32::from(c4 % 2 != 0) * 7;
        let pad_total = pad1 + pad2 + pad4;
        println!();
        println!(
            "blk0 grouped padding est: bap1={c1} bap2={c2} bap4={c4} → +{pad1}/+{pad2}/+{pad4} (upper bound {pad_total})"
        );
        println!(
            "  walk={} + padding ≤ {}",
            b0.mantissa_bits_walk,
            b0.mantissa_bits_walk + pad_total
        );
    }

    println!();
    println!("--- block 1 alignment probe (delta from blk0 end) ---");
    let hits = probe_next_block_alignment(&si, &bsi, frame, 0, -48, 48).unwrap();
    if let Some((_, ok0, pos0)) = hits.iter().find(|(d, _, _)| *d == 0) {
        println!("delta=0 bit_pos={pos0} parse_ok={ok0}");
    }
    let ok_hits: Vec<_> = hits.iter().filter(|(_, ok, _)| *ok).collect();
    if ok_hits.is_empty() {
        println!("no clean parse in [-48,+48]");
    } else {
        for (d, _, pos) in &ok_hits {
            println!("  delta={d:+4} bit_pos={pos} parse_ok=true");
        }
        if let Some(b0) = audits.first() {
            let walk = b0.mantissa_bits_walk;
            let base = b0.bit_block_end;
            println!();
            println!("structured blk1 candidates (side=15, mant={walk}):");
            for delta in 0i32..=48 {
                let pos = base + delta as u64;
                let mut st = state_after_block(&si, &bsi, frame, 0).unwrap();
                if let Ok(m) = try_parse_block_at(&si, &bsi, frame, &mut st, 1, pos) {
                    let side = m.bit_pre_mantissa.saturating_sub(pos);
                    let mant = m.bit_block_end.saturating_sub(m.bit_pre_mantissa);
                    if side == 15 && mant == u64::from(walk) {
                        let target_mant = pos - b0.bit_pre_mantissa;
                        let gap = target_mant as i64 - walk as i64;
                        println!(
                            "  delta={delta:+3} pos={pos} → inferred blk0 mantissa budget {target_mant} (gap {gap:+} vs walk)"
                        );
                    }
                }
            }
        }
        let best = ok_hits.iter().min_by_key(|(d, _, _)| d.abs()).unwrap();
        println!(
            "nearest parse_ok hit: delta={:+} (bit {}) — positive delta => blk0 under-read by that many bits",
            best.0, best.2
        );
    }

    if let Ok(side) = oxideav_ac3::audblk::parse_frame_side_info(&si, &bsi, frame) {
        let s = &side[0];
        println!();
        println!(
            "blk0 side-info: blksw={:?} chexpstr={:?} baie={} snroffste={} deltbaie={}",
            &s.blksw[..2],
            &s.chexpstr[..2],
            s.baie,
            s.snroffste,
            s.deltbaie,
        );
    }

    if frame_index > 0 {
        if let (Some((off1, len1)), Some((off2, len2))) = (
            nth_frame(&data, frame_index - 1),
            nth_frame(&data, frame_index),
        ) {
            let f1 = &data[off1..off1 + len1];
            let f2 = &data[off2..off2 + len2];
            let si1 = syncinfo::parse(f1).unwrap();
            let bsi1 = bsi::parse(&f1[5..]).unwrap();
            let si2 = syncinfo::parse(f2).unwrap();
            let bsi2 = bsi::parse(&f2[5..]).unwrap();
            let side1 = oxideav_ac3::audblk::parse_frame_side_info(&si1, &bsi1, f1).unwrap();
            let side2 = oxideav_ac3::audblk::parse_frame_side_info(&si2, &bsi2, f2).unwrap();
            println!();
            println!(
                "--- frame {} vs {} blk0 side-info ---",
                frame_index - 1,
                frame_index
            );
            for ch in 0..2 {
                println!(
                    "  ch{ch}: blksw {}→{} chexpstr {}→{} dithflag {}→{} chbwcod {}→{}",
                    side1[0].blksw[ch],
                    side2[0].blksw[ch],
                    side1[0].chexpstr[ch],
                    side2[0].chexpstr[ch],
                    side1[0].dithflag[ch],
                    side2[0].dithflag[ch],
                    side1[0].chbwcod[ch],
                    side2[0].chbwcod[ch],
                );
            }
            println!(
                "  dynrnge {}→{} cplstre {}→{} baie {}→{} deltbaie {}→{}",
                side1[0].dynrnge,
                side2[0].dynrnge,
                side1[0].cplstre,
                side2[0].cplstre,
                side1[0].baie,
                side2[0].baie,
                side1[0].deltbaie,
                side2[0].deltbaie,
            );
        }
    }
}
