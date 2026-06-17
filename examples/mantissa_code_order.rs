//! Dump mantissa `(bap, bin)` read order for one block — matches
//! [`mantissa_codes_from_decoder_state`] / ffmpeg `decode_transform_coeffs`.
//!
//! Use this to compare against `ac3dec.c` `decode_transform_coeffs` channel
//! loop and upstream `ac3enc.c` mantissa pack order.
//!
//! Usage:
//!   cargo run --release --example mantissa_code_order -- <file.ac3> [frame] [blk]
//!   cargo run --release --example mantissa_code_order -- white.ac3 0 0

use oxideav_ac3::audblk::{count_mantissa_bits_walk, state_after_block, MAX_FBW};
use oxideav_ac3::bsi;
use oxideav_ac3::encoder::{count_mantissa_bits_encoder_pack, count_mantissa_stream_bits, mantissa_codes_from_decoder_state};
use oxideav_ac3::syncinfo;
use oxideav_ac3::tables::QUANTIZATION_BITS;

#[derive(Clone, Debug)]
struct CodeEntry {
    idx: usize,
  /// Oxideav fbw index (0..nfchans-1) or `MAX_FBW` (cpl) or `MAX_FBW+1` (lfe).
    ch: usize,
    ch_label: String,
    bin: usize,
    bap: u8,
}

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

/// Labeled walk matching `mantissa_codes_from_decoder_state` iteration.
fn labeled_codes(state: &oxideav_ac3::audblk::Ac3State, bsi: &bsi::Bsi) -> Vec<CodeEntry> {
    let nfchans = bsi.nfchans as usize;
    let mut out = Vec::new();
    let mut got_cplchan = false;
    for ch in 0..nfchans {
        let end = state.channels[ch].end_mant;
        for bin in 0..end {
            let bap = state.channels[ch].bap[bin];
            if bap != 0 {
                out.push(CodeEntry {
                    idx: out.len(),
                    ch,
                    ch_label: format!("fbw{ch}"),
                    bin,
                    bap,
                });
            }
        }
        if state.cpl_in_use && state.channels[ch].in_coupling && !got_cplchan {
            let cplc = MAX_FBW;
            for bin in state.cpl_begf_mant..state.cpl_endf_mant {
                let bap = state.channels[cplc].bap[bin];
                if bap != 0 {
                    out.push(CodeEntry {
                        idx: out.len(),
                        ch: cplc,
                        ch_label: "cpl".into(),
                        bin,
                        bap,
                    });
                }
            }
            got_cplchan = true;
        }
    }
    if bsi.lfeon {
        let lfe_ch = MAX_FBW + 1;
        for bin in 0..7 {
            let bap = state.channels[lfe_ch].bap[bin];
            if bap != 0 {
                out.push(CodeEntry {
                    idx: out.len(),
                    ch: lfe_ch,
                    ch_label: "lfe".into(),
                    bin,
                    bap,
                });
            }
        }
    }
    out
}

/// Simulate encoder `write_mantissa_stream` group emissions on code indices.
fn emit_groups(codes: &[(u8, u32)]) -> Vec<(usize, u8, u32)> {
    let n = codes.len();
    let mut consumed = vec![false; n];
    let mut emits = Vec::new();
    for i in 0..n {
        if consumed[i] {
            continue;
        }
        let (bap, _) = codes[i];
        match bap {
            1 => {
                consumed[i] = true;
                let mut group = vec![i];
                let mut got = 1usize;
                for j in (i + 1)..n {
                    if consumed[j] || codes[j].0 != 1 {
                        continue;
                    }
                    consumed[j] = true;
                    group.push(j);
                    got += 1;
                    if got == 3 {
                        break;
                    }
                }
                emits.push((i, 1, 5));
                let _ = group;
            }
            2 => {
                consumed[i] = true;
                for j in (i + 1)..n {
                    if consumed[j] || codes[j].0 != 2 {
                        continue;
                    }
                    consumed[j] = true;
                    break;
                }
                for j in (i + 1)..n {
                    if consumed[j] || codes[j].0 != 2 {
                        continue;
                    }
                    consumed[j] = true;
                    break;
                }
                emits.push((i, 2, 7));
            }
            4 => {
                consumed[i] = true;
                for j in (i + 1)..n {
                    if consumed[j] || codes[j].0 != 4 {
                        continue;
                    }
                    consumed[j] = true;
                    break;
                }
                emits.push((i, 4, 7));
            }
            3 => emits.push((i, 3, 3)),
            5 => emits.push((i, 5, 4)),
            b if (6..=15).contains(&b) => emits.push((i, b, QUANTIZATION_BITS[b as usize] as u32)),
            _ => {}
        }
    }
    emits
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: mantissa_code_order <file.ac3> [frame] [blk]");
    let frame_index: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let blk: usize = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(0);

    let data = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let frame = nth_frame(&data, frame_index).unwrap_or_else(|| panic!("frame {frame_index} missing"));
    let si = syncinfo::parse(&frame).unwrap();
    let bsi = bsi::parse(&frame[5..]).unwrap();

    let state = state_after_block(&si, &bsi, &frame, blk)
        .unwrap_or_else(|e| panic!("parse through blk{blk}: {e}"));

    let codes = mantissa_codes_from_decoder_state(&state, &bsi);
    let labeled = labeled_codes(&state, &bsi);
    assert_eq!(codes.len(), labeled.len());
    for (a, b) in codes.iter().zip(labeled.iter()) {
        assert_eq!(a.0, b.bap);
    }

    let walk = count_mantissa_bits_walk(&state, &bsi);
    let pack = count_mantissa_bits_encoder_pack(&state, &bsi);
    let stream = count_mantissa_stream_bits(&codes);
    let emits = emit_groups(&codes);
    let emit_bits: u32 = emits.iter().map(|(_, _, n)| n).sum();

    println!("file: {path}");
    println!("frame: {frame_index}  blk: {blk}");
    println!(
        "cpl_in_use={}  nfchans={}  lfeon={}",
        state.cpl_in_use, bsi.nfchans, bsi.lfeon
    );
    for ch in 0..bsi.nfchans as usize {
        println!(
            "  fbw{ch}: end_mant={} in_coupling={}",
            state.channels[ch].end_mant, state.channels[ch].in_coupling
        );
    }
    println!(
        "codes={}  walk={}  pack={}  stream={}  emit_sim={}",
        codes.len(),
        walk,
        pack,
        stream,
        emit_bits
    );
    println!(
        "ffmpeg decode loop: for ch=1..channels {{ ac3_decode_transform_coeffs_ch(ch); maybe CPL_CH }}"
    );
    println!();

    // Histogram in code-stream order (non-zero bap bins only).
    let mut histo = [0u32; 16];
    for &(bap, _) in &codes {
        histo[bap as usize] += 1;
    }
    print!("code-stream bap histo:");
    for (b, n) in histo.iter().enumerate() {
        if *n > 0 {
            print!(" bap{b}={n}");
        }
    }
    println!();
    println!();

    let show = std::env::var("MANT_ORDER_FULL").is_ok();
    let limit = if show {
        labeled.len()
    } else {
        labeled.len().min(80)
    };

    println!("--- code index (compare to ac3dec mantissa read order) ---");
    println!("{:>5} {:>6} {:>4} {:>4}  {}", "idx", "ch", "bin", "bap", "notes");
    let mut group_id = 0u32;
    let mut emit_iter = emits.iter().peekable();
    for e in labeled.iter().take(limit) {
        let ff_ch = if e.ch <= 1 {
            e.ch + 1
        } else if e.ch == MAX_FBW {
            0 // CPL_CH in ffmpeg
        } else {
            state.channels.len().min(MAX_FBW + 2) // lfe_ch approx
        };
        let mut note = String::new();
        if let Some(em) = emit_iter.peek() {
            if em.0 == e.idx {
                let em = emit_iter.next().unwrap();
                group_id += 1;
                note = format!("GROUP#{group_id} bap{} +{}bits", em.1, em.2);
            }
        }
        println!(
            "{:>5} {:>6} {:>4} {:>4}  ff_ch≈{ff_ch} {}",
            e.idx, e.ch_label, e.bin, e.bap, note
        );
    }
    if limit < labeled.len() {
        println!("... {} more (set MANT_ORDER_FULL=1 for all)", labeled.len() - limit);
    }

    println!();
    println!("--- channel transitions in code stream ---");
    let mut prev = "";
    for e in &labeled {
        if e.ch_label != prev {
            println!("  @idx {} → {}", e.idx, e.ch_label);
            prev = &e.ch_label;
        }
    }

    println!();
    println!("--- last 12 codes (block tail — grouping boundary sensitive) ---");
    for e in labeled.iter().skip(labeled.len().saturating_sub(12)) {
        println!("  idx={} {} bin={} bap={}", e.idx, e.ch_label, e.bin, e.bap);
    }
}
