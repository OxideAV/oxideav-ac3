//! Bisect mask / exponent / BAP for a bin range across syncframes.
//!
//! Usage:
//!   cargo run --example mask_bisect -- <file.ac3> [frame_bad] [frame_good]
//!
//! Defaults: frame_bad=2, frame_good=1, ch=1, bins=133..=151, blk=0.

use oxideav_ac3::audblk::{
    ba_globals, bin_ba_detail, recount_block_mantissa_bits, rerun_ba_with_fsnroffst,
    state_after_block, BinBaDetail,
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

fn load_frame(data: &[u8], index: usize) -> (oxideav_ac3::syncinfo::SyncInfo, oxideav_ac3::bsi::Bsi, Vec<u8>) {
    let (off, len) = nth_frame(data, index).unwrap_or_else(|| panic!("frame {index} missing"));
    let frame = data[off..off + len].to_vec();
    let si = syncinfo::parse(&frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();
    (si, bsi, frame)
}

fn print_globals(label: &str, g: &oxideav_ac3::audblk::BaGlobals) {
    println!(
        "{label}: sdcy={} fdcy={} sgain={} dbpb={} floorcod={} (floor={:#x}) snr_coarse={} fsnroffst={} fgain={} snroffset={:#x} deltnseg={}",
        g.sdcycod,
        g.fdcycod,
        g.sgaincod,
        g.dbpbcod,
        g.floorcod,
        g.floor,
        g.snroffst_coarse,
        g.fsnroffst,
        g.fgaincod,
        g.snroffset,
        g.deltnseg,
    );
}

fn print_row(bad: &BinBaDetail, good: &BinBaDetail) {
    let exp_d = good.exp as i16 - bad.exp as i16;
    let psd_d = good.psd - bad.psd;
    let mask_d = good.mask - bad.mask;
    let m_d = good.m_snr - bad.m_snr;
    println!(
        "bin {:3} band {:2} | exp {:2}→{:2} ({:+3}) psd {:5}→{:5} ({:+5}) | mask {:5}→{:5} ({:+5}) m {:5}→{:5} ({:+5}) | addr {:2}→{:2} bap {:2}→{:2}",
        bad.bin,
        bad.band,
        bad.exp,
        good.exp,
        exp_d,
        bad.psd,
        good.psd,
        psd_d,
        bad.mask,
        good.mask,
        mask_d,
        bad.m_snr,
        good.m_snr,
        m_d,
        bad.bap_addr,
        good.bap_addr,
        bad.bap,
        good.bap,
    );
}

fn main() {
    let path = std::env::args().nth(1).expect("usage: mask_bisect <file.ac3> [bad] [good]");
    let frame_bad: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(2);
    let frame_good: usize = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(1);
    let ch: usize = std::env::args().nth(4).and_then(|s| s.parse().ok()).unwrap_or(1);
    let blk: usize = std::env::args().nth(5).and_then(|s| s.parse().ok()).unwrap_or(0);
    let bin_lo: usize = 133;
    let bin_hi: usize = 151;

    let data = std::fs::read(&path).unwrap();
    let (si_b, bsi_b, frame_b) = load_frame(&data, frame_bad);
    let (si_g, bsi_g, frame_g) = load_frame(&data, frame_good);

    let st_b = state_after_block(&si_b, &bsi_b, &frame_b, blk).expect("bad frame parse");
    let st_g = state_after_block(&si_g, &bsi_g, &frame_g, blk).expect("good frame parse");

    println!("{path}");
    println!(
        "ch{ch} bins {bin_lo}..={bin_hi} blk{blk}: frame {frame_bad} (bad) vs {frame_good} (good)"
    );
    println!();
    print_globals("bad ", &ba_globals(&st_b, ch));
    print_globals("good", &ba_globals(&st_g, ch));

    let mut exp_diffs = 0usize;
    let mut mask_diffs = 0usize;
    let mut bap_diffs = 0usize;
    let mut bits_lost = 0i32;

    println!();
    println!(
        "bin  band | exp      psd           | mask          m_snr         | addr    bap"
    );
    for bin in bin_lo..=bin_hi {
        let b = bin_ba_detail(&st_b, ch, bin);
        let g = bin_ba_detail(&st_g, ch, bin);
        if b.exp != g.exp {
            exp_diffs += 1;
        }
        if b.mask != g.mask {
            mask_diffs += 1;
        }
        if b.bap != g.bap {
            bap_diffs += 1;
            bits_lost += oxideav_ac3::tables::QUANTIZATION_BITS[g.bap as usize] as i32
                - oxideav_ac3::tables::QUANTIZATION_BITS[b.bap as usize] as i32;
        }
        print_row(&b, &g);
    }

    println!();
    println!(
        "summary: {bap_diffs}/{} bins differ in BAP, {mask_diffs} mask, {exp_diffs} exponent",
        bin_hi - bin_lo + 1
    );
    println!("estimated mantissa bits lost on differing bins (ch{ch} only): {bits_lost}");

    // Wider context: all ch1 bins where bap differs between frames.
    let end = st_b.channels[ch].end_mant.min(st_g.channels[ch].end_mant);
    let mut all_bap_diffs = 0usize;
    let mut all_bits = 0i32;
    for bin in 0..end {
        let b = bin_ba_detail(&st_b, ch, bin);
        let g = bin_ba_detail(&st_g, ch, bin);
        if b.bap != g.bap {
            all_bap_diffs += 1;
            all_bits += oxideav_ac3::tables::QUANTIZATION_BITS[g.bap as usize] as i32
                - oxideav_ac3::tables::QUANTIZATION_BITS[b.bap as usize] as i32;
        }
    }
    println!(
        "ch{ch} full 0..{end}: {all_bap_diffs} BAP diffs, net {all_bits:+} mantissa bits vs good frame"
    );

    // Side-info fields that size the bitstream before fsnroffst.
    for (label, fi) in [("bad", frame_bad), ("good", frame_good)] {
        let (si, bsi, frame) = load_frame(&data, fi);
        let s = oxideav_ac3::audblk::parse_frame_side_info(&si, &bsi, &frame).unwrap()[0].clone();
        println!(
            "{label} frame {fi} blk0 side: cplstre={} cplinu={} rematstr={} nremat={} chexpstr={:?} chbwcod={:?} baie={} snroffste={}",
            s.cplstre,
            s.cplinu,
            s.rematstr,
            s.rematflg_count,
            &s.chexpstr[..2],
            &s.chbwcod[..2],
            s.baie,
            s.snroffste,
        );
    }

    // Band-level mask dump for bands covering 133..151.
    let bands: Vec<usize> = (bin_lo..=bin_hi)
        .map(|b| oxideav_ac3::tables::MASKTAB[b] as usize)
        .collect();
    let bmin = *bands.iter().min().unwrap();
    let bmax = *bands.iter().max().unwrap();
    println!();
    println!("bands {bmin}..={bmax} bndpsd + mask:");
    print!("  band   ");
    for band in bmin..=bmax {
        print!(" {:>6}", band);
    }
    println!();
    print!("  bndpsd ");
    for band in bmin..=bmax {
        print!(" {:>6}", st_b.channels[ch].bndpsd[band]);
    }
    println!("  (bad)");
    print!("  bndpsd ");
    for band in bmin..=bmax {
        print!(" {:>6}", st_g.channels[ch].bndpsd[band]);
    }
    println!("  (good)");
    print!("  mask   ");
    for band in bmin..=bmax {
        print!(" {:>6}", st_b.channels[ch].mask[band]);
    }
    println!("  (bad)");
    print!("  mask   ");
    for band in bmin..=bmax {
        print!(" {:>6}", st_g.channels[ch].mask[band]);
    }
    println!("  (good)");
    print!("  Δmask  ");
    for band in bmin..=bmax {
        let d = st_g.channels[ch].mask[band] - st_b.channels[ch].mask[band];
        print!(" {:>+6}", d);
    }
    println!();

  // fsnroffst sweep on bad frame to match bitstream mantissa budget (~3206).
    const TARGET_MANT: u32 = 3206;
    let mut st_sweep = state_after_block(&si_b, &bsi_b, &frame_b, blk).expect("sweep parse");
    let base_fsnr = st_sweep.fsnroffst;
    println!();
    println!("fsnroffst sweep on frame {frame_bad} blk{blk} (target mantissa bits ~{TARGET_MANT}):");
    for f1 in 0..=15u8 {
        let mut fsnr = base_fsnr;
        fsnr[1] = f1;
        rerun_ba_with_fsnroffst(&mut st_sweep, &si_b, &bsi_b, fsnr);
        let bits = recount_block_mantissa_bits(&st_sweep, &bsi_b);
        let b133 = bin_ba_detail(&st_sweep, 1, 133);
        println!(
            "  ch1 fsnroffst={f1:2} → total_mant={bits} (Δ{}), ch1 bin133 bap={}",
            bits as i32 - TARGET_MANT as i32,
            b133.bap
        );
    }
    for f0 in 0..=15u8 {
        let mut st_sweep = state_after_block(&si_b, &bsi_b, &frame_b, blk).expect("sweep ch0");
        let mut fsnr = base_fsnr;
        fsnr[0] = f0;
        rerun_ba_with_fsnroffst(&mut st_sweep, &si_b, &bsi_b, fsnr);
        let bits = recount_block_mantissa_bits(&st_sweep, &bsi_b);
        println!(
            "  ch0 fsnroffst={f0:2} → total_mant={bits} (Δ{})",
            bits as i32 - TARGET_MANT as i32
        );
    }
    for f0 in 12..=15u8 {
        for f1 in 12..=15u8 {
            let mut st_sweep = state_after_block(&si_b, &bsi_b, &frame_b, blk).expect("2d sweep");
            let mut fsnr = base_fsnr;
            fsnr[0] = f0;
            fsnr[1] = f1;
            rerun_ba_with_fsnroffst(&mut st_sweep, &si_b, &bsi_b, fsnr);
            let bits = recount_block_mantissa_bits(&st_sweep, &bsi_b);
            if (bits as i32 - TARGET_MANT as i32).abs() <= 12 {
                println!(
                    "  ch0={f0} ch1={f1} → total_mant={bits} (Δ{})",
                    bits as i32 - TARGET_MANT as i32
                );
            }
        }
    }

    for coarse in 47..=52u8 {
        let mut st_c = state_after_block(&si_b, &bsi_b, &frame_b, blk).expect("coarse sweep");
        st_c.snroffst_coarse = coarse;
        rerun_ba_with_fsnroffst(&mut st_c, &si_b, &bsi_b, base_fsnr);
        let bits = recount_block_mantissa_bits(&st_c, &bsi_b);
        println!(
            "  snr_coarse={coarse} (fsnroffst={:?}) → total_mant={bits} (Δ{})",
            base_fsnr,
            bits as i32 - TARGET_MANT as i32
        );
    }
}
