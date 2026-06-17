//! BAP hypothesis testing without ffmpeg instrumentation.
//!
//! Compares decoder walk vs FFmpeg-encoder mantissa counting, replays CBR
//! SNR search, and scores BAP patches by PCM diff vs ffmpeg reference.
//!
//! Usage:
//!   cargo run --example bap_hypothesis -- [file.ac3]

use oxideav_ac3::audblk::{
    apply_cluster_patches, audit_frame_blocks, ba_globals, bap_override_from_audit,
    bin_ba_detail, cluster_bap_diffs, compare_channel_ba_ref, count_mantissa_bits_walk,
    count_walk_with_bap, diff_bap_bins, ffmpeg_encoder_mant_bits_from_histo, patch_bap_bins,
    probe_blk1_after_fsnroffst, rerun_all_bap_unified_snr, search_encoder_snr_for_mant_budget,
    search_snr_for_mant_target, snr_offset_from_search_index, snr_search_index_from_offset,
    state_after_block, Ac3State, MAX_FBW, N_COEFFS,
};
use oxideav_ac3::bsi::{self, Bsi};
use oxideav_ac3::syncinfo;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

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

fn load_ffmpeg_pcm(frame: usize, nsamp: usize) -> Option<Vec<f32>> {
    let path = std::path::PathBuf::from(std::env::var("TEMP").ok()?)
        .join("oxideav_repro")
        .join("ffmpeg.pcm");
    let bytes = std::fs::read(path).ok()?;
    let base = frame * 3072 * 4;
    if base + nsamp * 4 > bytes.len() {
        return None;
    }
    Some(
        bytes[base..base + nsamp * 4]
            .chunks_exact(4)
            .map(|c| f32::from_le_bytes(c.try_into().unwrap()))
            .collect(),
    )
}

fn pcm_stats(ours: &[f32], ff: &[f32]) -> (f32, usize, usize) {
    let mut max_diff = 0f32;
    let mut bad = 0usize;
    let mut spikes = 0usize;
    for (o, f) in ours.iter().zip(ff.iter()) {
        let d = (o - f).abs();
        if d > 1e-3 {
            bad += 1;
        }
        max_diff = max_diff.max(d);
        if o.abs() >= 0.999 {
            spikes += 1;
        }
    }
    (max_diff, bad, spikes)
}

fn decode_frame_with_bap(
    data: &[u8],
    frame_idx: usize,
    bap_override: Option<(usize, [[u8; N_COEFFS]; 5])>,
) -> Option<Vec<f32>> {
    let mut off = 0usize;
    let mut fi = 0usize;
    while off + 5 <= data.len() {
        let si = syncinfo::parse(&data[off..]).ok()?;
        let len = si.frame_length as usize;
        if len < 5 || off + len > data.len() {
            break;
        }
        if fi == frame_idx {
            let mut p = CodecParameters::audio(CodecId::new("ac3"));
            p.channels = Some(2);
            p.sample_rate = Some(48_000);
            let mut dec = oxideav_ac3::decoder::make_decoder(&p).ok()?;
            let tb = TimeBase::new(1, 48_000);
            if let Some((blk, bap)) = bap_override {
                // Reach into decoder state via audblk hook on next parse.
                // Ac3Decoder doesn't expose state; use low-level path with
                // the same s16 pack the public API applies.
                let frame = &data[off..off + len];
                let bsi = bsi::parse(&frame[5..]).ok()?;
                let nchans = bsi.nchans as usize;
                let mut state = Ac3State::new();
                state.bap_override = Some((blk, bap));
                let mut raw = vec![0f32; 1536 * nchans];
                oxideav_ac3::audblk::decode_frame(&mut state, &si, &bsi, frame, &mut raw)
                    .ok()?;
                return Some(
                    raw.iter()
                        .map(|s| {
                            let clamped = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
                            clamped as f32 / 32768.0
                        })
                        .collect(),
                );
            }
            dec.send_packet(&Packet::new(0, tb, data[off..off + len].to_vec()))
                .ok()?;
            if let Ok(Frame::Audio(af)) = dec.receive_frame() {
                return Some(
                    af.data[0]
                        .chunks_exact(2)
                        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                        .collect(),
                );
            }
            return None;
        }
        off += len;
        fi += 1;
    }
    None
}

fn patch_a_base(a2: &oxideav_ac3::audblk::BlockBitAudit) -> [[u8; N_COEFFS]; MAX_FBW] {
    let mut bap = bap_override_from_audit(a2, 2);
    let patches: Vec<(usize, u8)> = (133..=151).map(|b| (b, 9)).collect();
    patch_bap_bins(&mut bap[1], &patches);
    bap
}

fn patch_a_from_neighbor(
    bad: &oxideav_ac3::audblk::BlockBitAudit,
    good: &oxideav_ac3::audblk::BlockBitAudit,
    nfchans: usize,
) -> [[u8; N_COEFFS]; MAX_FBW] {
    let mut bap = bap_override_from_audit(bad, nfchans);
    for bin in 133..=151 {
        if bad.bap_ch[1][bin] != good.bap_ch[1][bin] {
            bap[1][bin] = good.bap_ch[1][bin];
        }
    }
    bap
}

fn patch_a_plus_ch0_cluster(
    bad: &oxideav_ac3::audblk::BlockBitAudit,
    good: &oxideav_ac3::audblk::BlockBitAudit,
    lo: usize,
    hi: usize,
    nfchans: usize,
) -> [[u8; N_COEFFS]; MAX_FBW] {
    let mut bap = patch_a_from_neighbor(bad, good, nfchans);
    let neighbor_bap = bap_override_from_audit(good, nfchans);
    apply_cluster_patches(&mut bap, &neighbor_bap, 0, &[(lo, hi)]);
    bap
}

fn mant_bits_delta(bap_from: u8, bap_to: u8) -> i32 {
    use oxideav_ac3::tables::QUANTIZATION_BITS;
    QUANTIZATION_BITS[bap_to as usize] as i32 - QUANTIZATION_BITS[bap_from as usize] as i32
}

fn print_bin_ba_trace(
    label: &str,
    st: &Ac3State,
    ch: usize,
    bins: &[usize],
    ref_bap: Option<&[u8]>,
) {
    let g = ba_globals(st, ch);
    println!(
        "  {label}: fsnroffst={} snroffset={:#x} floor={} fgaincod={} deltnseg={}",
        g.fsnroffst, g.snroffset, g.floor, g.fgaincod, g.deltnseg
    );
    println!(
        "  {:>4} {:>4} {:>6} {:>6} {:>6} {:>8} {:>5} {:>4} {:>4} {:>4}",
        "bin", "band", "exp", "psd", "bndpsd", "mask", "m_snr", "addr", "bap", "ref"
    );
    for &bin in bins {
        let d = bin_ba_detail(st, ch, bin);
        let rf = ref_bap.map(|r| r[bin]).unwrap_or(255);
        let rf_s = if rf == 255 {
            "-".to_string()
        } else {
            format!("{rf}")
        };
        println!(
            "  {:>4} {:>4} {:>6} {:>6} {:>6} {:>6} {:>8} {:>5} {:>4} {:>4}",
            d.bin, d.band, d.exp, d.psd, d.bndpsd, d.mask, d.m_snr, d.bap_addr, d.bap, rf_s
        );
    }
}

fn full_frame_parses_with_bap(data: &[u8], frame_idx: usize, bap: [[u8; N_COEFFS]; MAX_FBW]) -> bool {
    let (off, len) = match nth_frame(data, frame_idx) {
        Some(v) => v,
        None => return false,
    };
    let frame = &data[off..off + len];
    let si = match syncinfo::parse(frame) {
        Ok(v) => v,
        Err(_) => return false,
    };
    let bsi = match bsi::parse(&frame[5..]) {
        Ok(v) => v,
        Err(_) => return false,
    };
    oxideav_ac3::audblk::full_frame_parses_with_bap(&si, &bsi, frame, bap)
}

#[derive(Clone)]
struct PatchScore {
    label: String,
    walk: u32,
    spikes: usize,
    max_diff: f32,
    parse_ok: bool,
}

fn score_patch(
    data: &[u8],
    frame_idx: usize,
    bap: [[u8; N_COEFFS]; MAX_FBW],
    ff: &[f32],
    st: &Ac3State,
    bsi: &Bsi,
    label: &str,
) -> PatchScore {
    let walk = count_walk_with_bap(st, bsi, &bap);
    let parse_ok = full_frame_parses_with_bap(data, frame_idx, bap);
    let out = decode_frame_with_bap(data, frame_idx, Some((0, bap))).expect("decode");
    let (max_diff, _, spikes) = pcm_stats(&out, ff);
    PatchScore {
        label: label.to_string(),
        walk,
        spikes,
        max_diff,
        parse_ok,
    }
}

fn print_score(s: &PatchScore, budget: Option<u32>) {
    let budget_s = budget
        .map(|b| format!(" Δwalk={}", s.walk as i32 - b as i32))
        .unwrap_or_default();
    println!(
        "  {}: walk={}{} parse={} spikes={} max_diff={:.4}",
        s.label, s.walk, budget_s, s.parse_ok, s.spikes, s.max_diff
    );
}

/// One contiguous BAP cluster copied from a neighbor frame.
#[derive(Clone)]
struct ClusterUnit {
    ch: usize,
    lo: usize,
    hi: usize,
    naive_delta: i32,
    walk_delta: i32,
}

impl ClusterUnit {
    fn label(&self) -> String {
        format!("ch{}[{lo}..={hi}]", self.ch, lo = self.lo, hi = self.hi)
    }
}

fn build_cluster_units(
    base_bap: [[u8; N_COEFFS]; MAX_FBW],
    neighbor_bap: &[[u8; N_COEFFS]; MAX_FBW],
    bad_audit: &oxideav_ac3::audblk::BlockBitAudit,
    st: &Ac3State,
    bsi: &Bsi,
    ch: usize,
) -> Vec<ClusterUnit> {
    let diffs = diff_bap_bins(&bad_audit.bap_ch[ch], &neighbor_bap[ch], 217);
    let clusters = cluster_bap_diffs(&diffs);
    clusters
        .into_iter()
        .map(|(lo, hi, naive)| {
            let mut trial = base_bap;
            apply_cluster_patches(&mut trial, neighbor_bap, ch, &[(lo, hi)]);
            let walk_delta =
                count_walk_with_bap(st, bsi, &trial) as i32 - count_walk_with_bap(st, bsi, &base_bap) as i32;
            ClusterUnit {
                ch,
                lo,
                hi,
                naive_delta: naive,
                walk_delta,
            }
        })
        .collect()
}

fn apply_units(
    base: &mut [[u8; N_COEFFS]; MAX_FBW],
    neighbor: &[[u8; N_COEFFS]; MAX_FBW],
    units: &[ClusterUnit],
) {
    for u in units {
        apply_cluster_patches(base, neighbor, u.ch, &[(u.lo, u.hi)]);
    }
}

fn bisect_frame4_gap(data: &[u8]) {
    const FI: usize = 4;
    const NEIGHBOR: usize = 3; // fsnroffst=14, moderate neighbor

    println!("\n=== frame 4 +24-bit gap bisect (baseline vs frame {NEIGHBOR}) ===");
    let ff = load_ffmpeg_pcm(FI, 3072).expect("ffmpeg.pcm frame 4");
    let budget = inferred_blk0_mant_budget(data, FI).expect("frame 4 budget");
    let (off, len) = nth_frame(data, FI).unwrap();
    let (no, nl) = nth_frame(data, NEIGHBOR).unwrap();
    let frame = &data[off..off + len];
    let nframe = &data[no..no + nl];
    let si = syncinfo::parse(frame).unwrap();
    let bsi_p = bsi::parse(&frame[5..]).unwrap();
    let ab = audit_frame_blocks(&si, &bsi_p, frame).unwrap();
    let an = audit_frame_blocks(
        &syncinfo::parse(nframe).unwrap(),
        &bsi::parse(&nframe[5..]).unwrap(),
        nframe,
    )
    .unwrap();
    let st = state_after_block(&si, &bsi_p, frame, 0).unwrap();
    let base_bap = bap_override_from_audit(&ab[0], 2);
    let neighbor_bap = bap_override_from_audit(&an[0], 2);
    let base_walk = count_walk_with_bap(&st, &bsi_p, &base_bap);
    let gap = budget as i32 - base_walk as i32;

    println!(
        "baseline: walk={base_walk} budget={budget} gap={gap} fsnr {:?} → neighbor fsnr {:?}",
        &ab[0].fsnroffst[..2],
        &an[0].fsnroffst[..2],
    );
    for ch in 0..2 {
        let diffs = diff_bap_bins(&ab[0].bap_ch[ch], &an[0].bap_ch[ch], 217);
        println!("  ch{ch}: {} bin diffs vs frame {NEIGHBOR}", diffs.len());
    }

    let mut units: Vec<ClusterUnit> = Vec::new();
    for ch in 0..2 {
        units.extend(build_cluster_units(
            base_bap,
            &neighbor_bap,
            &ab[0],
            &st,
            &bsi_p,
            ch,
        ));
    }
    units.sort_by_key(|u| (-u.walk_delta, u.ch, u.lo));

    println!("\n--- per-cluster (parsed baseline, patches from f{NEIGHBOR}) ---");
    println!(
        "{:<18} {:>7} {:>7} {:>7} {:>7} {:>8}",
        "cluster", "naiveΔ", "walkΔ", "walk", "parse", "spikes"
    );
    let mut scored: Vec<(ClusterUnit, PatchScore)> = Vec::new();
    for u in &units {
        let mut bap = base_bap;
        apply_units(&mut bap, &neighbor_bap, std::slice::from_ref(u));
        let s = score_patch(
            data,
            FI,
            bap,
            &ff,
            &st,
            &bsi_p,
            &u.label(),
        );
        println!(
            "{:<18} {:>7} {:>7} {:>7} {:>7} {:>8}",
            u.label(),
            u.naive_delta,
            u.walk_delta,
            s.walk,
            s.parse_ok,
            s.spikes
        );
        scored.push((u.clone(), s));
    }

    // Clusters that increase walk (toward budget).
    let positive: Vec<&ClusterUnit> = units.iter().filter(|u| u.walk_delta > 0).collect();
    println!(
        "\n--- positive walkΔ clusters: {} (total naiveΔ={}) ---",
        positive.len(),
        positive.iter().map(|u| u.naive_delta).sum::<i32>()
    );

    // Greedy cumulative: add largest walkΔ first until budget met.
    {
        let mut bap = base_bap;
        let mut picked: Vec<String> = Vec::new();
        let mut ranked = positive.clone();
        ranked.sort_by_key(|u| -u.walk_delta);
        for u in ranked {
            apply_units(&mut bap, &neighbor_bap, std::slice::from_ref(u));
            picked.push(u.label());
            let w = count_walk_with_bap(&st, &bsi_p, &bap);
            if w >= budget {
                let s = score_patch(data, FI, bap, &ff, &st, &bsi_p, "greedy+");
                println!("greedy (largest walkΔ first), {} units:", picked.len());
                print_score(&s, Some(budget));
                print!("    ");
                for p in &picked {
                    print!("{p} ");
                }
                println!();
                break;
            }
        }
    }

    // Greedy by best spike count per walk bit (only positive Δ).
    {
        let mut candidates: Vec<(&ClusterUnit, usize, u32)> = scored
            .iter()
            .filter(|(u, _)| u.walk_delta > 0)
            .map(|(u, s)| (u, s.spikes, s.walk))
            .collect();
        candidates.sort_by_key(|(_, spikes, walk)| (*spikes, (budget as i32 - *walk as i32).abs()));
        let mut bap = base_bap;
        let mut picked: Vec<String> = Vec::new();
        for (u, _, _) in &candidates {
            apply_units(&mut bap, &neighbor_bap, std::slice::from_ref(u));
            picked.push(u.label());
            let w = count_walk_with_bap(&st, &bsi_p, &bap);
            if w >= budget {
                let s = score_patch(data, FI, bap, &ff, &st, &bsi_p, "greedy-low-spikes+");
                println!("greedy (lowest spikes per cluster), {} units:", picked.len());
                print_score(&s, Some(budget));
                print!("    ");
                for p in &picked {
                    print!("{p} ");
                }
                println!();
                break;
            }
        }
    }

    // Pair search among positive-walkΔ clusters (disjoint bin ranges per ch).
    println!("\n--- pair search (positive walkΔ, target walk≈{budget}) ---");
    let pos_scored: Vec<(&ClusterUnit, &PatchScore)> = scored
        .iter()
        .filter(|(u, _)| u.walk_delta > 0)
        .map(|(u, s)| (u, s))
        .collect();
    let mut best_pairs: Vec<(i32, PatchScore, String)> = Vec::new();
    for i in 0..pos_scored.len() {
        for j in (i + 1)..pos_scored.len() {
            let (u1, _) = pos_scored[i];
            let (u2, _) = pos_scored[j];
            if u1.ch == u2.ch && ranges_overlap(u1.lo, u1.hi, u2.lo, u2.hi) {
                continue;
            }
            let mut bap = base_bap;
            apply_units(&mut bap, &neighbor_bap, &[u1.clone(), u2.clone()]);
            let walk = count_walk_with_bap(&st, &bsi_p, &bap);
            if walk < budget.saturating_sub(2) || walk > budget + 6 {
                continue;
            }
            let label = format!("{} + {}", u1.label(), u2.label());
            let s = score_patch(data, FI, bap, &ff, &st, &bsi_p, &label);
            let dist = (s.walk as i32 - budget as i32).abs();
            best_pairs.push((dist, s, label));
        }
    }
    best_pairs.sort_by_key(|(d, s, _)| (*d, s.spikes));
    println!("top pair combos (by |walk-budget|, then spikes):");
    for (d, s, label) in best_pairs.iter().take(12) {
        println!(
            "  |Δwalk|={d} walk={} parse={} spikes={} max_diff={:.4}  {label}",
            s.walk, s.parse_ok, s.spikes, s.max_diff
        );
    }

    // Triple search on top walkΔ clusters (cap n for runtime).
    let mut top_units: Vec<ClusterUnit> = positive
        .iter()
        .take(18)
        .map(|u| (*u).clone())
        .collect();
    top_units.sort_by_key(|u| -u.walk_delta);
    println!("\n--- triple search (top {} positive clusters) ---", top_units.len());
    let mut best_triples: Vec<(i32, PatchScore)> = Vec::new();
    for i in 0..top_units.len() {
        for j in (i + 1)..top_units.len() {
            for k in (j + 1)..top_units.len() {
                let u1 = &top_units[i];
                let u2 = &top_units[j];
                let u3 = &top_units[k];
                if clusters_conflict(u1, u2) || clusters_conflict(u1, u3) || clusters_conflict(u2, u3) {
                    continue;
                }
                let mut bap = base_bap;
                apply_units(&mut bap, &neighbor_bap, &[u1.clone(), u2.clone(), u3.clone()]);
                let walk = count_walk_with_bap(&st, &bsi_p, &bap);
                if walk < budget.saturating_sub(2) || walk > budget + 6 {
                    continue;
                }
                let label = format!("{} + {} + {}", u1.label(), u2.label(), u3.label());
                let s = score_patch(data, FI, bap, &ff, &st, &bsi_p, &label);
                let dist = (s.walk as i32 - budget as i32).abs();
                best_triples.push((dist, s));
            }
        }
    }
    best_triples.sort_by_key(|(d, s)| (*d, s.spikes));
    for (d, s) in best_triples.iter().take(10) {
        println!(
            "  |Δwalk|={d} {} walk={} parse={} spikes={} max_diff={:.4}",
            s.label, s.walk, s.parse_ok, s.spikes, s.max_diff
        );
    }

    // Report zero-spike hits at or above budget.
    println!("\n--- zero-spike solutions (walk >= {budget}) ---");
    let mut zero_hits: Vec<PatchScore> = scored
        .into_iter()
        .map(|(_, s)| s)
        .filter(|s| s.spikes == 0 && s.parse_ok && s.walk >= budget)
        .collect();
    // Re-score combos from best pairs/triples with 0 spikes
    for (_, s, _) in best_pairs.iter().filter(|(_, s, _)| s.spikes == 0 && s.parse_ok) {
        zero_hits.push(s.clone());
    }
    for (_, s) in best_triples.iter().filter(|(_, s)| s.spikes == 0 && s.parse_ok) {
        zero_hits.push(s.clone());
    }
    zero_hits.sort_by_key(|s| (s.walk as i32 - budget as i32).abs());
    if zero_hits.is_empty() {
        println!("  (none found — try subset-sum over positive clusters next)");
        // Subset-sum DP on positive clusters for exact gap
        subset_sum_bisect(
            data,
            FI,
            &ff,
            &st,
            &bsi_p,
            base_bap,
            &neighbor_bap,
            &positive.iter().map(|u| (*u).clone()).collect::<Vec<_>>(),
            base_walk,
            budget,
        );
    } else {
        for s in zero_hits.iter().take(8) {
            print_score(s, Some(budget));
        }
    }

    // Compare: frame 4 vs frame 2 (same fsnroffst=13, different walk).
    println!("\n--- frame 4 vs frame 2 (both fsnroffst=13) ---");
    let (o2, l2) = nth_frame(data, 2).unwrap();
    let f2 = &data[o2..o2 + l2];
    let a2 = audit_frame_blocks(
        &syncinfo::parse(f2).unwrap(),
        &bsi::parse(&f2[5..]).unwrap(),
        f2,
    )
    .unwrap();
    for ch in 0..2 {
        let d = diff_bap_bins(&ab[0].bap_ch[ch], &a2[0].bap_ch[ch], 217);
        println!("  ch{ch}: {} diffs f4 vs f2", d.len());
    }
    let combo_f2 = patch_a_plus_ch0_cluster(&ab[0], &a2[0], 116, 118, 2);
    let s_f2patch = score_patch(data, FI, combo_f2, &ff, &st, &bsi_p, "f2-style A+ch0[116..118]");
    print_score(&s_f2patch, Some(budget));

    println!("\n--- winning cluster ch1[133..=156] per-bin BAP (f4 / f3 / Δbits) ---");
    for bin in 133..=156 {
        let b4 = ab[0].bap_ch[1][bin];
        let b3 = an[0].bap_ch[1][bin];
        if b4 != b3 {
            println!("  bin {bin}: f4={b4} f3={b3} Δ={}", mant_bits_delta(b4, b3));
        }
    }

  bisect_frame_gap(data, 9, 8);
}

/// Same bisect harness for another catastrophic frame / neighbor pair.
fn bisect_frame_gap(data: &[u8], fi: usize, neighbor: usize) {
    if fi != 4 {
        println!("\n=== frame {fi} gap bisect (vs frame {neighbor}) ===");
    }
    let ff = match load_ffmpeg_pcm(fi, 3072) {
        Some(v) => v,
        None => return,
    };
    let budget = match inferred_blk0_mant_budget(data, fi) {
        Some(b) => b,
        None => return,
    };
    let (off, len) = match nth_frame(data, fi) {
        Some(v) => v,
        None => return,
    };
    let (no, nl) = match nth_frame(data, neighbor) {
        Some(v) => v,
        None => return,
    };
    let frame = &data[off..off + len];
    let nframe = &data[no..no + nl];
    let si = syncinfo::parse(frame).unwrap();
    let bsi_p = bsi::parse(&frame[5..]).unwrap();
    let ab = audit_frame_blocks(&si, &bsi_p, frame).unwrap();
    let an = audit_frame_blocks(
        &syncinfo::parse(nframe).unwrap(),
        &bsi::parse(&nframe[5..]).unwrap(),
        nframe,
    )
    .unwrap();
    let st = state_after_block(&si, &bsi_p, frame, 0).unwrap();
    let base_bap = bap_override_from_audit(&ab[0], 2);
    let neighbor_bap = bap_override_from_audit(&an[0], 2);
    let base_walk = count_walk_with_bap(&st, &bsi_p, &base_bap);
    let gap = budget as i32 - base_walk as i32;
    println!(
        "baseline walk={base_walk} budget={budget} gap={gap} fsnr {:?} → {:?}",
        &ab[0].fsnroffst[..2],
        &an[0].fsnroffst[..2],
    );
    let mut bap = base_bap;
    apply_cluster_patches(&mut bap, &neighbor_bap, 1, &[(133, 156)]);
    let s = score_patch(
        data,
        fi,
        bap,
        &ff,
        &st,
        &bsi_p,
        "ch1[133..=156] from neighbor",
    );
    print_score(&s, Some(budget));
}

/// Why ch1 bins 152–156 need patching on subtype B (4,9) but not subtype A (2,7).
fn investigate_subtype_ab_ch1_hf(data: &[u8]) {
    println!("\n=== subtype A vs B: ch1 bins 133–156 (blk0) ===");
    println!("Subtype A = frames 2,7  |  Subtype B = frames 4,9");
    println!("Reference = fsnroffst=14 neighbor (frame fi+1 for A, fi-1 for B)\n");

    struct FrameCtx {
        fi: usize,
        subtype: &'static str,
        fsnr14: usize,
    }
    let frames = [
        FrameCtx { fi: 2, subtype: "A", fsnr14: 3 },
        FrameCtx { fi: 7, subtype: "A", fsnr14: 8 },
        FrameCtx { fi: 4, subtype: "B", fsnr14: 3 },
        FrameCtx { fi: 9, subtype: "B", fsnr14: 8 },
    ];

    println!(
        "{:<4} {:>3} {:>4} {:>4} {:>4} {:>4} {:>6} {:>6} {:>5} {:>4} {:>4} {:>4} {:>5}",
        "fr", "sub", "bin", "bap", "n14", "Δ", "exp", "psd", "mask", "m", "addr", "n_m", "n_addr"
    );

    let mut audits: std::collections::HashMap<usize, oxideav_ac3::audblk::BlockBitAudit> =
        std::collections::HashMap::new();
    let mut states: std::collections::HashMap<usize, Ac3State> = std::collections::HashMap::new();
    let mut neighbor_bap: std::collections::HashMap<usize, [[u8; N_COEFFS]; MAX_FBW]> =
        std::collections::HashMap::new();
    let mut neighbor_state: std::collections::HashMap<usize, Ac3State> =
        std::collections::HashMap::new();

    for fc in &frames {
        let (off, len) = nth_frame(data, fc.fi).unwrap();
        let frame = &data[off..off + len];
        let si = syncinfo::parse(frame).unwrap();
        let bsi_p = bsi::parse(&frame[5..]).unwrap();
        let audit = audit_frame_blocks(&si, &bsi_p, frame).unwrap();
        let st = state_after_block(&si, &bsi_p, frame, 0).unwrap();
        audits.insert(fc.fi, audit[0].clone());
        states.insert(fc.fi, st);

        let (no, nl) = nth_frame(data, fc.fsnr14).unwrap();
        let nf = &data[no..no + nl];
        let an = audit_frame_blocks(
            &syncinfo::parse(nf).unwrap(),
            &bsi::parse(&nf[5..]).unwrap(),
            nf,
        )
        .unwrap();
        neighbor_bap.insert(fc.fi, bap_override_from_audit(&an[0], 2));
        let stn = state_after_block(
            &syncinfo::parse(nf).unwrap(),
            &bsi::parse(&nf[5..]).unwrap(),
            nf,
            0,
        )
        .unwrap();
        neighbor_state.insert(fc.fi, stn);
    }

    for fc in &frames {
        let a = audits.get(&fc.fi).unwrap();
        let st = states.get(&fc.fi).unwrap();
        let nb = neighbor_bap.get(&fc.fi).unwrap();
        let stn = neighbor_state.get(&fc.fi).unwrap();
        for bin in 133..=156 {
            let bap = a.bap_ch[1][bin];
            let n14 = nb[1][bin];
            let d = bin_ba_detail(st, 1, bin);
            let dn = bin_ba_detail(stn, 1, bin);
            let delta = if bap == n14 {
                "=".to_string()
            } else {
                mant_bits_delta(bap, n14).to_string()
            };
            println!(
                "{:<4} {:>3} {:>4} {:>4} {:>4} {:>4} {:>6} {:>6} {:>5} {:>4} {:>4} {:>4} {:>5}",
                fc.fi,
                fc.subtype,
                bin,
                bap,
                n14,
                delta,
                d.exp,
                d.psd,
                d.mask,
                d.m_snr,
                d.bap_addr,
                dn.m_snr,
                dn.bap_addr,
            );
        }
        println!();
    }

    // Direct A vs B at the critical tail bins.
    println!("--- bins 152–156: subtype A (f2,f7) vs B (f4,f9) ---");
    println!(
        "{:<4} {:>4} {:>4} {:>4} {:>4} {:>6} {:>6} {:>6} {:>6}",
        "bin", "f2", "f7", "f4", "f9", "e2", "e4", "psd2", "psd4"
    );
    for bin in 152..=156 {
        let a2 = &audits[&2];
        let a4 = &audits[&4];
        let a7 = &audits[&7];
        let a9 = &audits[&9];
        let s2 = states.get(&2).unwrap();
        let s4 = states.get(&4).unwrap();
        println!(
            "{bin:<4} {:>4} {:>4} {:>4} {:>4} {:>6} {:>6} {:>6} {:>6}",
            a2.bap_ch[1][bin],
            a7.bap_ch[1][bin],
            a4.bap_ch[1][bin],
            a9.bap_ch[1][bin],
            s2.channels[1].exp[bin],
            s4.channels[1].exp[bin],
            s2.channels[1].psd[bin],
            s4.channels[1].psd[bin],
        );
    }

    // BAP cliff: what psd-m threshold separates bap 8 vs 9 at fsnroffst=13?
    println!("\n--- BAP cliff at ch1 band for fsnroffst=13 (frame 2 state) ---");
    let st2 = states.get(&2).unwrap();
    let g2 = ba_globals(st2, 1);
    for bin in [151usize, 152, 155, 156] {
        let d = bin_ba_detail(st2, 1, bin);
        let psd_minus_m = d.psd as i32 - d.m_snr;
        println!(
            "  f2 bin {bin}: bap={} addr={} psd-m={} (addr 31→bap9, addr 30→bap8)",
            d.bap,
            d.bap_addr,
            psd_minus_m
        );
    }
    let st4 = states.get(&4).unwrap();
    for bin in [151usize, 152, 155, 156] {
        let d = bin_ba_detail(st4, 1, bin);
        let psd_minus_m = d.psd as i32 - d.m_snr;
        println!(
            "  f4 bin {bin}: bap={} addr={} psd-m={}",
            d.bap,
            d.bap_addr,
            psd_minus_m
        );
    }
    println!("  snroffset f2={:#x} f4={:#x}", g2.snroffset, ba_globals(st4, 1).snroffset);

    // Walk impact: patch only 152–156 on subtype A frames.
    println!("\n--- walk impact: patch ch1[152..=156] only (from fsnr=14 neighbor) ---");
    for fc in &frames {
        let (off, len) = nth_frame(data, fc.fi).unwrap();
        let frame = &data[off..off + len];
        let si = syncinfo::parse(frame).unwrap();
        let bsi_p = bsi::parse(&frame[5..]).unwrap();
        let st = states.get(&fc.fi).unwrap();
        let a = audits.get(&fc.fi).unwrap();
        let nb = neighbor_bap.get(&fc.fi).unwrap();
        let base = bap_override_from_audit(a, 2);
        let budget = inferred_blk0_mant_budget(data, fc.fi);
        let base_walk = count_walk_with_bap(st, &bsi_p, &base);
        let mut patch152 = base;
        apply_cluster_patches(&mut patch152, nb, 1, &[(152, 156)]);
        let w152 = count_walk_with_bap(st, &bsi_p, &patch152);
        let mut patch133 = base;
        apply_cluster_patches(&mut patch133, nb, 1, &[(133, 151)]);
        let w133 = count_walk_with_bap(st, &bsi_p, &patch133);
        let mut patch156 = base;
        apply_cluster_patches(&mut patch156, nb, 1, &[(133, 156)]);
        let w156 = count_walk_with_bap(st, &bsi_p, &patch156);
        println!(
            "  frame {} ({}): base={base_walk} +133..151→{w133} +152..156→{w152} +133..156→{w156} budget={budget:?}",
            fc.fi, fc.subtype
        );
    }

    // blk0 side-info: exponent strategy / block switch (ch1).
    println!("\n--- blk0 side-info ch1 (exponent path) ---");
    for fc in &frames {
        let (off, len) = nth_frame(data, fc.fi).unwrap();
        let frame = &data[off..off + len];
        let si = syncinfo::parse(frame).unwrap();
        let bsi_p = bsi::parse(&frame[5..]).unwrap();
        let side = oxideav_ac3::audblk::parse_frame_side_info(&si, &bsi_p, frame)
            .ok()
            .and_then(|v| v.into_iter().next());
        let st = states.get(&fc.fi).unwrap();
        if let Some(s) = side {
            println!(
                "  frame {} ({}): fsnroffst={} blksw={} chexpstr={} chbwcod={} exp[152..156]={:?}",
                fc.fi,
                fc.subtype,
                audits.get(&fc.fi).unwrap().fsnroffst[1],
                s.blksw[1],
                s.chexpstr[1],
                s.chbwcod[1],
                &st.channels[1].exp[152..=156],
            );
        }
    }

    // FFmpeg-ref ch1 compare: mask/bap diffs for f2 vs f4.
    println!("\n--- compare_channel_ba_ref ch1 (parsed vs ffmpeg mask path) ---");
    for fc in &[2usize, 4, 7, 9] {
        let (off, len) = nth_frame(data, *fc).unwrap();
        let frame = &data[off..off + len];
        let si = syncinfo::parse(frame).unwrap();
        let st = states.get(fc).unwrap();
        let cmp = compare_channel_ba_ref(st, &si, 1, 0);
        let tail_diffs: Vec<_> = cmp
            .bap_diff_bins
            .iter()
            .filter(|(b, _, _)| (152..=156).contains(b))
            .collect();
        println!(
            "  frame {fc}: ref_mant={} ours_mant={} mask_diff_bands={} bap_diffs_total={} tail152-156={tail_diffs:?}",
            cmp.ref_mant_bits,
            cmp.ours_mant_bits,
            cmp.mask_diff_bands.len(),
            cmp.bap_diff_bins.len(),
        );
    }

    // Bins 152–156 do NOT differ between A and B — same parsed BA on all four frames.
    println!("\n--- conclusion probe: where does subtype B +8 walk come from? ---");
    for (fi_a, fi_b, label) in [(2usize, 4, "f2 vs f4"), (7, 9, "f7 vs f9")] {
        let a_a = audits.get(&fi_a).unwrap();
        let a_b = audits.get(&fi_b).unwrap();
        let ch0_diffs = diff_bap_bins(&a_a.bap_ch[0], &a_b.bap_ch[0], 217);
        let ch1_diffs = diff_bap_bins(&a_a.bap_ch[1], &a_b.bap_ch[1], 217);
        let st_a = states.get(&fi_a).unwrap();
        let st_b = states.get(&fi_b).unwrap();
        let bsi_a = {
            let (o, l) = nth_frame(data, fi_a).unwrap();
            bsi::parse(&data[o..o + l][5..]).unwrap()
        };
        let bsi_b = {
            let (o, l) = nth_frame(data, fi_b).unwrap();
            bsi::parse(&data[o..o + l][5..]).unwrap()
        };
        let walk_a = count_mantissa_bits_walk(st_a, &bsi_a);
        let walk_b = count_mantissa_bits_walk(st_b, &bsi_b);
        println!(
            "  {label}: walk {walk_a}→{walk_b} (Δ{}), ch0_bap_diffs={} ch1_bap_diffs={}",
            walk_b as i32 - walk_a as i32,
            ch0_diffs.len(),
            ch1_diffs.len(),
        );
        if !ch0_diffs.is_empty() {
            println!("    ch0 BAP diffs (first 12): {:?}", &ch0_diffs[..ch0_diffs.len().min(12)]);
        }
        if !ch1_diffs.is_empty() {
            println!("    ch1 BAP diffs: {:?}", &ch1_diffs);
        }
    }

    // Same ch1 fix on both subtypes: 133..156 from fsnr=14 neighbor.
    println!("\n--- unified ch1[133..=156] patch (both subtypes) ---");
    for fc in &frames {
        let (off, len) = nth_frame(data, fc.fi).unwrap();
        let frame = &data[off..off + len];
        let si = syncinfo::parse(frame).unwrap();
        let bsi_p = bsi::parse(&frame[5..]).unwrap();
        let st = states.get(&fc.fi).unwrap();
        let a = audits.get(&fc.fi).unwrap();
        let nb = neighbor_bap.get(&fc.fi).unwrap();
        let ff = load_ffmpeg_pcm(fc.fi, 3072).expect("pcm");
        let mut bap = bap_override_from_audit(a, 2);
        apply_cluster_patches(&mut bap, nb, 1, &[(133, 156)]);
        let s = score_patch(
            data,
            fc.fi,
            bap,
            &ff,
            st,
            &bsi_p,
            "ch1[133..=156]",
        );
        print_score(&s, inferred_blk0_mant_budget(data, fc.fi));
    }

    // Subtype A alternative: 133..151 + 152..156 (should equal 133..156).
    println!("\n--- subtype A: 133..151 only vs 152..156 only (frame 2) ---");
    {
        let fc = &frames[0];
        let (off, len) = nth_frame(data, fc.fi).unwrap();
        let frame = &data[off..off + len];
        let si = syncinfo::parse(frame).unwrap();
        let bsi_p = bsi::parse(&frame[5..]).unwrap();
        let st = states.get(&fc.fi).unwrap();
        let a = audits.get(&fc.fi).unwrap();
        let nb = neighbor_bap.get(&fc.fi).unwrap();
        let ff = load_ffmpeg_pcm(2, 3072).unwrap();
        let base = bap_override_from_audit(a, 2);
        for (label, ranges) in [
            ("133..151", vec![(133usize, 151)]),
            ("152..156", vec![(152usize, 156)]),
            ("133..156", vec![(133usize, 156)]),
        ] {
            let mut bap = base;
            for &(lo, hi) in &ranges {
                apply_cluster_patches(&mut bap, nb, 1, &[(lo, hi)]);
            }
            let s = score_patch(data, 2, bap, &ff, st, &bsi_p, label);
            print_score(&s, inferred_blk0_mant_budget(data, 2));
        }
    }
}

fn ranges_overlap(a0: usize, a1: usize, b0: usize, b1: usize) -> bool {
    a0 <= b1 && b0 <= a1
}

fn clusters_conflict(a: &ClusterUnit, b: &ClusterUnit) -> bool {
    a.ch == b.ch && ranges_overlap(a.lo, a.hi, b.lo, b.hi)
}

/// Small subset-sum: pick disjoint clusters whose walkΔ sums to `target_gap`.
fn subset_sum_bisect(
    data: &[u8],
    fi: usize,
    ff: &[f32],
    st: &Ac3State,
    bsi: &Bsi,
    base_bap: [[u8; N_COEFFS]; MAX_FBW],
    neighbor_bap: &[[u8; N_COEFFS]; MAX_FBW],
    units: &[ClusterUnit],
    base_walk: u32,
    budget: u32,
) {
    let target_gap = budget as i32 - base_walk as i32;
    let n = units.len().min(20);
    let units = &units[..n];
    println!("  subset-sum over {n} positive clusters (target walkΔ={target_gap}):");

    let limit = 1usize << n;
    let mut walk_hits: Vec<(usize, u32, Vec<ClusterUnit>)> = Vec::new();
    for mask in 1..limit {
        let mut chosen: Vec<&ClusterUnit> = Vec::new();
        for (i, u) in units.iter().enumerate() {
            if (mask >> i) & 1 == 1 {
                chosen.push(u);
            }
        }
        if clusters_pairwise_conflict(&chosen) {
            continue;
        }
        let sum: i32 = chosen.iter().map(|u| u.walk_delta).sum();
        if sum < target_gap - 1 || sum > target_gap + 3 {
            continue;
        }
        let mut bap = base_bap;
        let owned: Vec<ClusterUnit> = chosen.iter().map(|u| (*u).clone()).collect();
        apply_units(&mut bap, neighbor_bap, &owned);
        let walk = count_walk_with_bap(st, bsi, &bap);
        if walk >= budget.saturating_sub(1) && walk <= budget + 4 {
            walk_hits.push((mask, walk, owned));
        }
    }
    walk_hits.sort_by_key(|(_, walk, units)| {
        (
            (*walk as i32 - budget as i32).abs(),
            units.len(),
        )
    });
    walk_hits.truncate(40);

    let mut best: Option<PatchScore> = None;
    for (_, _, chosen) in walk_hits {
        let mut bap = base_bap;
        apply_units(&mut bap, neighbor_bap, &chosen);
        let label = chosen
            .iter()
            .map(|u| u.label())
            .collect::<Vec<_>>()
            .join(" + ");
        let s = score_patch(data, fi, bap, ff, st, bsi, &label);
        let key = (s.spikes, (s.walk as i32 - budget as i32).abs());
        if best.as_ref().map_or(true, |b| {
            let bk = (b.spikes, (b.walk as i32 - budget as i32).abs());
            key < bk
        }) {
            best = Some(s);
        }
    }
    if let Some(s) = best {
        print_score(&s, Some(budget));
    } else {
        println!("  no subset hit budget (tried {} masks)", limit - 1);
    }
}

fn clusters_pairwise_conflict(units: &[&ClusterUnit]) -> bool {
    for i in 0..units.len() {
        for j in (i + 1)..units.len() {
            if clusters_conflict(units[i], units[j]) {
                return true;
            }
        }
    }
    false
}

fn inferred_blk0_mant_budget(data: &[u8], frame_idx: usize) -> Option<u32> {
    let (off, len) = nth_frame(data, frame_idx)?;
    let frame = &data[off..off + len];
    let si = syncinfo::parse(frame).ok()?;
    let bsi = bsi::parse(&frame[5..]).ok()?;
    let audits = audit_frame_blocks(&si, &bsi, frame).ok()?;
    let b0 = audits.first()?;
    let base = b0.bit_block_end;
    for delta in 0i32..=48 {
        let pos = base + delta as u64;
        let mut st = state_after_block(&si, &bsi, frame, 0).ok()?;
        if let Ok(m) = oxideav_ac3::audblk::try_parse_block_at(&si, &bsi, frame, &mut st, 1, pos) {
            let side = m.bit_pre_mantissa.saturating_sub(pos);
            if side == 15 {
                return Some((pos - b0.bit_pre_mantissa) as u32);
            }
        }
    }
    None
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| format!("{}\\oxideav_repro\\multi.ac3", std::env::var("TEMP").unwrap()));
    let data = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));

    println!("=== mantissa budget analysis (blk0) ===\n");
    for fi in [2usize, 3, 4, 5] {
        let (off, len) = nth_frame(&data, fi).expect("frame");
        let frame = &data[off..off + len];
        let si = syncinfo::parse(frame).unwrap();
        let bsi = bsi::parse(&frame[5..]).unwrap();
        let audits = audit_frame_blocks(&si, &bsi, frame).unwrap();
        let b0 = &audits[0];
        let enc_histo = ffmpeg_encoder_mant_bits_from_histo(&b0.bap_histo_combined);
        let inferred = inferred_blk0_mant_budget(&data, fi);
        let probe = probe_blk1_after_fsnroffst(&si, &bsi, frame, 0, None).unwrap();
        let st = state_after_block(&si, &bsi, frame, 0).unwrap();
        let _snr_idx = snr_search_index_from_offset(st.snr_offset_ch[0]);
        println!(
            "frame {fi}: fsnroffst={:?} walk={} pack={} enc_histo={} inferred_budget={inferred:?} blk1_ok={}",
            &b0.fsnroffst[..2],
            b0.mantissa_bits_walk,
            b0.mantissa_bits_pack,
            enc_histo,
            probe.blk1_parse_ok,
        );
        if let Some(budget) = inferred {
            let (idx, mant) = search_encoder_snr_for_mant_budget(&st, &bsi, budget);
            let (idx_tgt, mant_tgt) = search_snr_for_mant_target(&st, &bsi, budget);
            let mant_hi = {
                let mut s = st.clone();
                rerun_all_bap_unified_snr(
                    &mut s,
                    &bsi,
                    snr_offset_from_search_index(idx.saturating_add(64).min(1023)),
                );
                count_mantissa_bits_walk(&s, &bsi)
            };
            println!(
                "  CBR replay for budget {budget}: snr_idx={idx} unified_snr={:#x} → mant={mant} (idx+64 → {mant_hi} bits)",
                snr_offset_from_search_index(idx)
            );
            println!(
                "  binary search for mant>={budget}: snr_idx={idx_tgt} snr={:#x} → mant={mant_tgt}",
                snr_offset_from_search_index(idx_tgt)
            );
        }
    }

    score_ch0_clusters(&data);
    bisect_frame4_gap(&data);
    investigate_subtype_ab_ch1_hf(&data);
    run_pcm_hypothesis_tests(&data);
}

fn score_ch0_clusters(data: &[u8]) {
    println!("\n=== ch0 cluster scoring (frame2 blk0, base = patch A) ===");
    let ff = load_ffmpeg_pcm(2, 3072).expect("ffmpeg.pcm for frame 2");
    let budget = inferred_blk0_mant_budget(data, 2);
    let (o2, l2) = nth_frame(data, 2).unwrap();
    let (o3, l3) = nth_frame(data, 3).unwrap();
    let f2 = &data[o2..o2 + l2];
    let f3 = &data[o3..o3 + l3];
    let si2 = syncinfo::parse(f2).unwrap();
    let bsi2 = bsi::parse(&f2[5..]).unwrap();
    let a2 = audit_frame_blocks(&si2, &bsi2, f2).unwrap();
    let a3 = audit_frame_blocks(
        &syncinfo::parse(f3).unwrap(),
        &bsi::parse(&f3[5..]).unwrap(),
        f3,
    )
    .unwrap();
    let st2 = state_after_block(&si2, &bsi2, f2, 0).unwrap();
    let base_bap = patch_a_base(&a2[0]);
    let neighbor_bap = bap_override_from_audit(&a3[0], 2);
    let base_walk = count_walk_with_bap(&st2, &bsi2, &base_bap);
    println!(
        "patch A base: walk={base_walk} budget={budget:?} ch1 HF 133-151→bap9"
    );

    let ch0_diffs = diff_bap_bins(&a2[0].bap_ch[0], &a3[0].bap_ch[0], 217);
    let clusters = cluster_bap_diffs(&ch0_diffs);
    println!("\nch0: {} diffs in {} contiguous clusters:", ch0_diffs.len(), clusters.len());
    println!("{:<22} {:>8} {:>8} {:>8} {:>7} {:>7} {:>8}", "cluster", "naiveΔ", "walkΔ", "walk", "parse", "spikes", "max_diff");
    let mut cluster_scores: Vec<(usize, PatchScore)> = Vec::new();
    for (ci, (lo, hi, naive)) in clusters.iter().enumerate() {
        let mut bap = base_bap;
        apply_cluster_patches(&mut bap, &neighbor_bap, 0, &[(*lo, *hi)]);
        let walk = count_walk_with_bap(&st2, &bsi2, &bap);
        let walk_delta = walk as i32 - base_walk as i32;
        let s = score_patch(data, 2, bap, &ff, &st2, &bsi2, &format!("A+ch0[{lo}..={hi}]"));
        println!(
            "{:<22} {:>8} {:>8} {:>8} {:>7} {:>7} {:>8.4}",
            format!("[{lo}..={hi}]"),
            naive,
            walk_delta,
            walk,
            s.parse_ok,
            s.spikes,
            s.max_diff
        );
        cluster_scores.push((ci, s));
    }

    {
        let mut bap = base_bap;
        for &(bin, _, _) in &ch0_diffs {
            bap[0][bin] = neighbor_bap[0][bin];
        }
        let s = score_patch(data, 2, bap, &ff, &st2, &bsi2, "A+all ch0 f3 diffs");
        print_score(&s, budget);
    }

    {
        let mut bap = base_bap;
        let mut applied: Vec<(usize, usize)> = Vec::new();
        let target = budget.unwrap_or(base_walk);
        let mut ranked: Vec<(usize, i32)> = clusters
            .iter()
            .enumerate()
            .map(|(i, (lo, hi, _))| {
                let mut trial = bap;
                apply_cluster_patches(&mut trial, &neighbor_bap, 0, &[(*lo, *hi)]);
                let w = count_walk_with_bap(&st2, &bsi2, &trial);
                (i, w as i32 - base_walk as i32)
            })
            .collect();
        ranked.sort_by_key(|(_, d)| -d);
        for (i, _) in &ranked {
            let (lo, hi, _) = clusters[*i];
            apply_cluster_patches(&mut bap, &neighbor_bap, 0, &[(lo, hi)]);
            applied.push((lo, hi));
            let w = count_walk_with_bap(&st2, &bsi2, &bap);
            if w >= target {
                break;
            }
        }
        let s = score_patch(
            data,
            2,
            bap,
            &ff,
            &st2,
            &bsi2,
            &format!("A+greedy {} clusters", applied.len()),
        );
        print_score(&s, budget);
        print!("    clusters applied:");
        for (lo, hi) in &applied {
            print!(" [{lo}..={hi}]");
        }
        println!();
    }

    if let Some((_, best)) = cluster_scores.iter().min_by_key(|(_, s)| s.spikes) {
        println!(
            "best single ch0 cluster by spikes: {} (spikes={})",
            best.label, best.spikes
        );
    }

    print_bin_trace_and_multi(data, &a2, &a3, &st2, &si2, &bsi2, f3, &ff, budget);
}

fn print_bin_trace_and_multi(
    data: &[u8],
    a2: &[oxideav_ac3::audblk::BlockBitAudit],
    a3: &[oxideav_ac3::audblk::BlockBitAudit],
    st2: &Ac3State,
    si2: &oxideav_ac3::syncinfo::SyncInfo,
    bsi2: &Bsi,
    f3: &[u8],
    ff: &[f32],
    budget: Option<u32>,
) {
    println!("\n=== ch0 bin BAP trace (frame2 blk0 vs frame3 neighbor) ===");
    let key_bins: Vec<usize> = [57, 58, 59, 60, 116, 117, 118].into_iter().collect();
    println!("{:<6} {:>6} {:>6} {:>6}", "bin", "f2_bap", "f3_bap", "Δbits");
    for &bin in &key_bins {
        let b2 = a2[0].bap_ch[0][bin];
        let b3 = a3[0].bap_ch[0][bin];
        println!("{bin:<6} {b2:>6} {b3:>6} {:>6}", mant_bits_delta(b2, b3));
    }
    let st3 = state_after_block(
        &syncinfo::parse(f3).unwrap(),
        &bsi::parse(&f3[5..]).unwrap(),
        f3,
        0,
    )
    .unwrap();
    let ref_cmp = compare_channel_ba_ref(&st2, &si2, 0, 0);
    println!(
        "\nch0 BA ref compare: ours_mant={} ref_mant={} mask_diff_bands={} bap_diff_bins={}",
        ref_cmp.ours_mant_bits,
        ref_cmp.ref_mant_bits,
        ref_cmp.mask_diff_bands.len(),
        ref_cmp.bap_diff_bins.len()
    );
    if !ref_cmp.mask_diff_bands.is_empty() {
        println!(
            "  mask band diffs: {:?}",
            &ref_cmp.mask_diff_bands[..ref_cmp.mask_diff_bands.len().min(15)]
        );
    }
    print_bin_ba_trace("frame2 ch0 (parsed)", &st2, 0, &key_bins, None);
    print_bin_ba_trace("frame3 ch0 (neighbor)", &st3, 0, &key_bins, None);
    for &bin in &key_bins {
        let d2 = bin_ba_detail(&st2, 0, bin);
        let d3 = bin_ba_detail(&st3, 0, bin);
        let in_ref_diff = ref_cmp.bap_diff_bins.iter().any(|(b, _, _)| *b == bin);
        if d2.bap != a3[0].bap_ch[0][bin] || in_ref_diff {
            let rf = ref_cmp
                .bap_diff_bins
                .iter()
                .find(|(b, _, _)| *b == bin)
                .map(|(_, _, r)| *r);
            println!(
                "  bin {bin}: f2={} f3={} ref={} band={} exp2/3={}/{} psd2/3={}/{} mask2/3={}/{} addr2={} (psd2-m2={})",
                d2.bap,
                a3[0].bap_ch[0][bin],
                rf.map(|v| v.to_string()).unwrap_or_else(|| "-".into()),
                d2.band,
                d2.exp,
                d3.exp,
                d2.psd,
                d3.psd,
                d2.mask,
                d3.mask,
                d2.bap_addr,
                d2.psd as i32 - d2.m_snr
            );
        }
    }

    // Explicit score for leading combo patch.
    {
        let bap = patch_a_plus_ch0_cluster(&a2[0], &a3[0], 116, 118, 2);
        let s = score_patch(
            &data,
            2,
            bap,
            &ff,
            &st2,
            &bsi2,
            "A+ch0[116..=118] (leading)",
        );
        print_score(&s, budget);
    }

    // --- Multi-frame scoring on catastrophic frames ---
    println!("\n=== multi-frame patch scoring (catastrophic frames) ===");
    println!(
        "{:<8} {:>6} {:>6} {:>8} {:>7} {:>7} {:>8}  neighbor ch0 clusters w/ 0 spikes",
        "frame", "fsnr", "budget", "walk", "parse", "spikes", "max_diff"
    );
    for &fi in &[2usize, 4, 7, 9] {
        let ni = fi + 1;
        let (off, len) = nth_frame(&data, fi).expect("bad frame");
        let (no, nl) = nth_frame(&data, ni).expect("neighbor frame");
        let frame = &data[off..off + len];
        let nframe = &data[no..no + nl];
        let si = syncinfo::parse(frame).unwrap();
        let bsi_p = bsi::parse(&frame[5..]).unwrap();
        let ab = audit_frame_blocks(&si, &bsi_p, frame).unwrap();
        let an = audit_frame_blocks(
            &syncinfo::parse(nframe).unwrap(),
            &bsi::parse(&nframe[5..]).unwrap(),
            nframe,
        )
        .unwrap();
        let st = state_after_block(&si, &bsi_p, frame, 0).unwrap();
        let budget = inferred_blk0_mant_budget(&data, fi);
        let ff_n = load_ffmpeg_pcm(fi, 3072).expect("ffmpeg pcm");
        let bap_a = patch_a_from_neighbor(&ab[0], &an[0], 2);
        let walk_a = count_walk_with_bap(&st, &bsi_p, &bap_a);
        let s_a = score_patch(&data, fi, bap_a, &ff_n, &st, &bsi_p, "A");
        let bap_combo = patch_a_plus_ch0_cluster(&ab[0], &an[0], 116, 118, 2);
        let s_combo = score_patch(&data, fi, bap_combo, &ff_n, &st, &bsi_p, "A+116..118");
        let bap_combo2 = patch_a_plus_ch0_cluster(&ab[0], &an[0], 57, 60, 2);
        let s_combo2 = score_patch(&data, fi, bap_combo2, &ff_n, &st, &bsi_p, "A+57..60");
        println!(
            "frame {fi} [{:?}] budget={budget:?}:",
            &ab[0].fsnroffst[..2]
        );
        print_score(&s_a, budget);
        print_score(&s_combo, budget);
        print_score(&s_combo2, budget);
        // Per-frame: which ch0 clusters from neighbor give 0 spikes on top of A?
        let base_bap = patch_a_from_neighbor(&ab[0], &an[0], 2);
        let base_walk = count_walk_with_bap(&st, &bsi_p, &base_bap);
        let neighbor_bap = bap_override_from_audit(&an[0], 2);
        let ch0_diffs = diff_bap_bins(&ab[0].bap_ch[0], &an[0].bap_ch[0], 217);
        let clusters = cluster_bap_diffs(&ch0_diffs);
        let mut zero_spike: Vec<String> = Vec::new();
        for (lo, hi, _) in &clusters {
            let mut bap = base_bap;
            apply_cluster_patches(&mut bap, &neighbor_bap, 0, &[(*lo, *hi)]);
            let s = score_patch(
                &data,
                fi,
                bap,
                &ff_n,
                &st,
                &bsi_p,
                &format!("[{lo}..={hi}]"),
            );
            if s.spikes == 0 && s.parse_ok {
                let w = count_walk_with_bap(&st, &bsi_p, &bap);
                zero_spike.push(format!("[{lo}..={hi}] walk={w} md={:.3}", s.max_diff));
            }
        }
        println!(
            "    patch A alone: walk={walk_a} parse={} spikes={}",
            s_a.parse_ok, s_a.spikes
        );
        if zero_spike.is_empty() {
            println!("    zero-spike clusters: (none)");
        } else {
            println!("    zero-spike clusters: {}", zero_spike.join(", "));
        }
        let _ = base_walk;
    }

    // Frame 4/9: neighbor fi+1 has fsnroffst=12 (wrong direction). Try fi-1 (fsnr=14).
    println!("\n=== frame 4/9 alt neighbor (fi-1, fsnroffst=14) ===");
    for &fi in &[4usize, 9] {
        let ni = fi - 1;
        let (off, len) = nth_frame(&data, fi).unwrap();
        let (no, nl) = nth_frame(&data, ni).unwrap();
        let frame = &data[off..off + len];
        let nframe = &data[no..no + nl];
        let si = syncinfo::parse(frame).unwrap();
        let bsi_p = bsi::parse(&frame[5..]).unwrap();
        let ab = audit_frame_blocks(&si, &bsi_p, frame).unwrap();
        let an = audit_frame_blocks(
            &syncinfo::parse(nframe).unwrap(),
            &bsi::parse(&nframe[5..]).unwrap(),
            nframe,
        )
        .unwrap();
        let st = state_after_block(&si, &bsi_p, frame, 0).unwrap();
        let budget = inferred_blk0_mant_budget(&data, fi);
        let ff_n = load_ffmpeg_pcm(fi, 3072).expect("ffmpeg pcm");
        let mut bap = bap_override_from_audit(&ab[0], 2);
        for ch in 0..2 {
            for bin in 0..217 {
                if ab[0].bap_ch[ch][bin] != an[0].bap_ch[ch][bin] {
                    bap[ch][bin] = an[0].bap_ch[ch][bin];
                }
            }
        }
        let walk = count_walk_with_bap(&st, &bsi_p, &bap);
        let s = score_patch(
            &data,
            fi,
            bap,
            &ff_n,
            &st,
            &bsi_p,
            &format!("all diffs vs frame {ni}"),
        );
        println!(
            "frame {fi} vs neighbor {ni} fsnr {:?}->{:?}:",
            &ab[0].fsnroffst[..2],
            &an[0].fsnroffst[..2]
        );
        print_score(&s, budget);
        println!("    walk={walk} ch0_diffs={} ch1_diffs={}",
            diff_bap_bins(&ab[0].bap_ch[0], &an[0].bap_ch[0], 217).len(),
            diff_bap_bins(&ab[0].bap_ch[1], &an[0].bap_ch[1], 217).len(),
        );
    }
}

fn run_pcm_hypothesis_tests(data: &[u8]) {
    let ff = load_ffmpeg_pcm(2, 3072).expect("ffmpeg.pcm for frame 2");
    println!("\n=== PCM hypothesis tests (frame 2, isolated) ===");
    let baseline = decode_frame_with_bap(&data, 2, None).expect("decode");
    let (md, bad, spikes) = pcm_stats(&baseline, &ff);
    println!("baseline parsed BAP: max_diff={md:.4} bad={bad}/3072 spikes={spikes}");

    // Patch A: ch1 HF bins 133-151 bap 8→9 (frame3 neighbor on threshold band).
    {
        let (o, l) = nth_frame(&data, 2).unwrap();
        let f = &data[o..o + l];
        let audits = audit_frame_blocks(
            &syncinfo::parse(f).unwrap(),
            &bsi::parse(&f[5..]).unwrap(),
            f,
        )
        .unwrap();
        let mut bap = bap_override_from_audit(&audits[0], 2);
        let patches: Vec<(usize, u8)> = (133..=151).map(|b| (b, 9)).collect();
        patch_bap_bins(&mut bap[1], &patches);
        let mut st = state_after_block(
            &syncinfo::parse(f).unwrap(),
            &bsi::parse(&f[5..]).unwrap(),
            f,
            0,
        )
        .unwrap();
        for ch in 0..2 {
            let end = st.channels[ch].end_mant;
            st.channels[ch].bap[..end].copy_from_slice(&bap[ch][..end]);
        }
        let walk = count_mantissa_bits_walk(&st, &bsi::parse(&f[5..]).unwrap());
        let out = decode_frame_with_bap(&data, 2, Some((0, bap))).expect("decode");
        let (md, bad, spikes) = pcm_stats(&out, &ff);
        println!(
            "patch A (ch1 bins 133-151 → bap 9): max_diff={md:.4} bad={bad}/3072 spikes={spikes} walk={walk}"
        );
    }

    // Patch B: full blk0 BAP from frame 3 (good neighbor).
    {
        let (o3, l3) = nth_frame(&data, 3).unwrap();
        let f3 = &data[o3..o3 + l3];
        let audits3 = audit_frame_blocks(
            &syncinfo::parse(f3).unwrap(),
            &bsi::parse(&f3[5..]).unwrap(),
            f3,
        )
        .unwrap();
        let bap = bap_override_from_audit(&audits3[0], 2);
        let out = decode_frame_with_bap(&data, 2, Some((0, bap))).expect("decode");
        let (md, bad, spikes) = pcm_stats(&out, &ff);
        println!(
            "patch B (frame3 blk0 BAP on frame2): max_diff={md:.4} bad={bad}/3072 spikes={spikes}"
        );
    }

    // Patch C: unified SNR from CBR replay at inferred budget ~3196.
    {
        let (o, l) = nth_frame(&data, 2).unwrap();
        let f = &data[o..o + l];
        let si = syncinfo::parse(f).unwrap();
        let bsi_p = bsi::parse(&f[5..]).unwrap();
        let st = state_after_block(&si, &bsi_p, f, 0).unwrap();
        if let Some(budget) = inferred_blk0_mant_budget(&data, 2) {
            let (idx, _) = search_encoder_snr_for_mant_budget(&st, &bsi_p, budget);
            let mut bap = bap_override_from_audit(
                &audit_frame_blocks(&si, &bsi_p, f).unwrap()[0],
                2,
            );
            let mut st2 = st.clone();
            rerun_all_bap_unified_snr(&mut st2, &bsi_p, snr_offset_from_search_index(idx));
            for ch in 0..2 {
                let end = st2.channels[ch].end_mant;
                bap[ch][..end].copy_from_slice(&st2.channels[ch].bap[..end]);
            }
            let out = decode_frame_with_bap(&data, 2, Some((0, bap))).expect("decode");
            let (md, bad, spikes) = pcm_stats(&out, &ff);
            println!(
                "patch C (CBR replay snr_idx={idx} at budget {budget}): max_diff={md:.4} bad={bad}/3072 spikes={spikes}"
            );
        }
    }

    // Patch D: all frame2→frame3 blk0 BAP diffs (72+25 bins).
    {
        let (o2, l2) = nth_frame(&data, 2).unwrap();
        let (o3, l3) = nth_frame(&data, 3).unwrap();
        let f2 = &data[o2..o2 + l2];
        let f3 = &data[o3..o3 + l3];
        let a2 = audit_frame_blocks(
            &syncinfo::parse(f2).unwrap(),
            &bsi::parse(&f2[5..]).unwrap(),
            f2,
        )
        .unwrap();
        let a3 = audit_frame_blocks(
            &syncinfo::parse(f3).unwrap(),
            &bsi::parse(&f3[5..]).unwrap(),
            f3,
        )
        .unwrap();
        let mut bap = bap_override_from_audit(&a2[0], 2);
        for ch in 0..2 {
            for bin in 0..217 {
                if a2[0].bap_ch[ch][bin] != a3[0].bap_ch[ch][bin] {
                    bap[ch][bin] = a3[0].bap_ch[ch][bin];
                }
            }
        }
        let out = decode_frame_with_bap(&data, 2, Some((0, bap))).expect("decode");
        let (md, bad, spikes) = pcm_stats(&out, &ff);
        println!(
            "patch D (all f2→f3 BAP diffs on blk0): max_diff={md:.4} bad={bad}/3072 spikes={spikes}"
        );
    }

    // Patch E: binary-search unified SNR for inferred mant budget on frame 2.
    if let Some(budget) = inferred_blk0_mant_budget(&data, 2) {
        let (o, l) = nth_frame(&data, 2).unwrap();
        let f = &data[o..o + l];
        let si = syncinfo::parse(f).unwrap();
        let bsi_p = bsi::parse(&f[5..]).unwrap();
        let st = state_after_block(&si, &bsi_p, f, 0).unwrap();
        let (idx_tgt, _mant_tgt) = search_snr_for_mant_target(&st, &bsi_p, budget);
        let mut bap = bap_override_from_audit(&audit_frame_blocks(&si, &bsi_p, f).unwrap()[0], 2);
        let mut st2 = st.clone();
        rerun_all_bap_unified_snr(&mut st2, &bsi_p, snr_offset_from_search_index(idx_tgt));
        for ch in 0..2 {
            let end = st2.channels[ch].end_mant;
            bap[ch][..end].copy_from_slice(&st2.channels[ch].bap[..end]);
        }
        let out = decode_frame_with_bap(&data, 2, Some((0, bap))).expect("decode");
        let (md, bad, spikes) = pcm_stats(&out, &ff);
        println!(
            "patch E (binary-search snr_idx={idx_tgt} for budget {budget}): max_diff={md:.4} bad={bad}/3072 spikes={spikes}"
        );
    }
}
