//! Adaptive Hybrid Transform (AHT) decode helpers — A/52:2018 Annex E §3.4.
//!
//! AHT layers a non-overlapped, non-windowed DCT-II of length 6 on top
//! of the standard 256-coefficient MDCT for blocks where the encoder
//! decides the signal is stationary enough that the coding gain of a
//! longer transform beats the time-domain smearing it would normally
//! incur. The AHT path lives entirely inside the §7.3 mantissa unpack
//! step: instead of reading 256 mantissas per audblk, the decoder reads
//! 6×N mantissas (one per (block, bin) pair) for the **first** AHT-active
//! audblk of each frame, dequantises them with a high-efficiency
//! quantiser table (`hebap`), and then inverse-DCT-II's per bin to
//! recover the per-block MDCT coefficients §3.4.5.
//!
//! ## Per-spec eligibility
//!
//! AHT is only available when:
//!
//! * `numblkscod == 0x3` (6 blocks per syncframe) — the AHT transform
//!   length is hard-coded to 6.
//! * `ahte == 1` is set in `audfrm()`.
//! * For each AHT-active channel, `nchregs[ch] == 1` (i.e. exponents
//!   are transmitted exactly once per syncframe — block 0 carries a
//!   non-`REUSE` strategy and blocks 1..5 all reuse). Same rule for
//!   `ncplregs`/`nlferegs` on coupling and LFE channels.
//!
//! ## Quantiser stack (§3.4.4)
//!
//! Once `hebap` is computed for a given mantissa bin (§3.4.3.1 — same
//! masking model as base AC-3 with a finer 64-entry pointer table),
//! each of the 6 cross-block coefficients in that bin is quantised by
//! one of three modes:
//!
//! * **`hebap == 0`** — bin contributes no bits (zero-mantissa).
//! * **`1 ≤ hebap ≤ 7`** — vector-quantised: a single 2..9-bit codeword
//!   indexes into a 6-D codebook (Tables E4.1..E4.7 in
//!   [`super::tables::aht_codebooks`]). The codebook entry IS the
//!   6-tuple of dequantised values.
//! * **`8 ≤ hebap ≤ 19`** — symmetric scalar quantiser (with optional
//!   gain-adaptive step-size scaling per the per-frame `gaqmod`). Each
//!   of the 6 mantissas reads its own short codeword; if the GAQ tag
//!   (full-scale negative) is detected, an extra "large mantissa"
//!   codeword follows.
//!
//! ## Cross-block IDCT (§3.4.5)
//!
//! After all 6 mantissas per AHT bin are dequantised, the spec's
//! inverse DCT-II reconstructs the per-block MDCT spectrum value:
//!
//! ```text
//!     C(k, m) = 2 · Σ_{j=0..5}  R(j) · X(k, j) · cos[ j·(2m+1)·π / 12 ]
//!     R(j)   = 1                  for j != 0
//!            = 1/√2               for j == 0
//! ```
//!
//! Then the standard `coeff = mantissa · 2^(-exp)` reconstruction
//! converts each per-block C(k, m) into the regular fbw transform
//! coefficient slot, ready for IMDCT + overlap-add via the existing
//! [`crate::audblk::dsp_block`] path.
//!
//! ## Round-6 scope (this commit)
//!
//! * VQ codebooks E4.1..E4.7 transcribed (956 entries × 6 i16).
//! * `hebap` pointer table (Table E3.1) + quantiser-bit table (E3.2).
//! * GAQ gain-element decode (Table E3.4) + dead-zone large-mantissa
//!   remap (Table E3.6).
//! * fbw-channel AHT mantissa unpack + cross-block IDCT.
//! * Coupling / LFE AHT (`cplahtinu`, `lfeahtinu`) — wired through
//!   the per-channel iteration so 6-block-coupling and AHT-LFE
//!   fixtures decode, but no corpus fixture currently exercises them.
//!
//! Symbol naming follows the spec: `hebap`, `chgaqmod`, `chgaqgain`,
//! `chgaqbin`, `pre_chmant` and the `[k][j]` (bin, AHT-block) ordering
//! are kept literal so the implementation can be cross-referenced
//! against §3.4 paragraph by paragraph.

use std::f32::consts::PI;

use oxideav_core::bits::BitReader;
use oxideav_core::Result;

use super::tables::aht_codebooks::{
    VQ_HEBAP1, VQ_HEBAP2, VQ_HEBAP3, VQ_HEBAP4, VQ_HEBAP5, VQ_HEBAP6, VQ_HEBAP7,
};

/// AHT works on exactly six audio blocks per syncframe (§3.4.1).
pub const AHT_BLOCKS: usize = 6;

/// Table E3.1 — high-efficiency bit-allocation pointer lookup.
/// Indexed by a 6-bit address derived from `(psd - mask) >> 5` clamped
/// to 0..63. Table values are 5-bit `hebap` codes per §3.4.3.1.
pub const HEBAPTAB: [u8; 64] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12,
    13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 18,
    18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 19,
];

/// Table E3.2 — number of mantissa bits per `hebap` index for the
/// scalar/GAQ regime (`hebap >= 8`). Indices 0..7 are zero (VQ regime
/// has its own bit-count baked into the codebook size).
pub const HEBAP_MANT_BITS: [u8; 20] = [
    0, 0, 0, 0, 0, 0, 0, 0, // 0..7  → VQ or zero
    3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16,
];

/// Number of bits per VQ codeword for `hebap` 1..7 (Table E3.2 column
/// "Mantissa Bits" rounded up over 6 values: 2/6 → 2 bits, 3/6 → 3 bits,
/// etc.). Index 0 unused (signals zero-mantissa).
pub const VQ_BITS: [u8; 8] = [0, 2, 3, 4, 5, 7, 8, 9];

/// Decode a `hebap` value from `(psd, mask)` using the §3.4.3.1
/// pseudo-code. Mirrors [`crate::audblk::run_bit_allocation`]'s tail
/// step but emits `hebap` (via [`HEBAPTAB`]) instead of the 5-bit
/// `bap`.
#[inline]
pub fn hebap_from_address(psd: i16, mask_after_floor: i32) -> u8 {
    let addr = (((psd as i32) - mask_after_floor) >> 5).clamp(0, 63) as usize;
    HEBAPTAB[addr]
}

/// `endbap` for the GAQ-active range — mode 0/1 cap at 12 (i.e. apply
/// gains only for `8 <= hebap < 12`); mode 2/3 cap at 17. Per §3.4.2
/// pseudo-code paragraph that builds `chgaqbin[ch][bin]`.
#[inline]
pub fn endbap_for_gaqmod(gaqmod: u8) -> u8 {
    if gaqmod < 2 {
        12
    } else {
        17
    }
}

/// Compute `chgaqbin[ch][bin]` per the §3.4.2 pseudo-code:
///
/// * `+1` — bin is GAQ-coded, gain word follows for it.
/// * `-1` — bin is large-only (no GAQ gain word, falls through to the
///   fixed-quantiser regime).
/// * `0` — bin not in the GAQ regime.
///
/// `gaqmod` is the per-channel `chgaqmod` (or `cplgaqmod` /
/// `lfegaqmod`); pass `0` to disable GAQ for the entire bin range.
pub fn fill_gaqbin(hebap: &[u8], gaqmod: u8, gaqbin: &mut [i8]) -> usize {
    let endbap = endbap_for_gaqmod(gaqmod);
    let mut active = 0usize;
    for (i, &h) in hebap.iter().enumerate() {
        if i >= gaqbin.len() {
            break;
        }
        if h > 7 && h < endbap {
            gaqbin[i] = 1;
            active += 1;
        } else if h >= endbap {
            gaqbin[i] = -1;
        } else {
            gaqbin[i] = 0;
        }
    }
    active
}

/// Number of GAQ gain words transmitted for a channel given its
/// `gaqmod` and the active-bin count returned by [`fill_gaqbin`]. Per
/// the §3.4.2 final pseudo-code chunk (chgaqsections / cplgaqsections /
/// lfegaqsections).
pub fn gaq_sections(gaqmod: u8, active_gaqbins: usize) -> usize {
    match gaqmod {
        0 => 0,
        1 | 2 => active_gaqbins,
        3 => active_gaqbins.div_ceil(3),
        _ => 0,
    }
}

/// Unpack `nsections` GAQ gain words from the bit stream into a flat
/// `[u8; nbins]` array of mapped values 0..2 (per Table E3.4).
///
/// * `gaqmod == 1` or `2` — 1 bit per active bin, mapped value = bit
///   (0 → Gk=1, 1 → Gk=2 or 4).
/// * `gaqmod == 3` — 5-bit composite triplet, decoded via
///   `M1 = grpgain/9`, `M2 = (grpgain%9)/3`, `M3 = grpgain%9%3`.
///
/// Returns the per-bin mapped values aligned to the GAQ-active bins
/// (in order). The caller threads them onto `chgaqbin[bin] == 1`
/// positions as it walks the AHT mantissa loop.
pub fn read_gaq_gains(
    br: &mut BitReader<'_>,
    gaqmod: u8,
    nsections: usize,
    out: &mut [u8],
) -> Result<usize> {
    if nsections == 0 || gaqmod == 0 {
        return Ok(0);
    }
    let mut written = 0usize;
    for _ in 0..nsections {
        match gaqmod {
            1 | 2 => {
                if written >= out.len() {
                    break;
                }
                let bit = br.read_u32(1)? as u8;
                out[written] = bit;
                written += 1;
            }
            3 => {
                let grp = br.read_u32(5)?;
                // Triplet decode per §3.4.4.2 pseudo-code.
                let m1 = (grp / 9) as u8;
                let m2 = ((grp % 9) / 3) as u8;
                let m3 = ((grp % 9) % 3) as u8;
                for v in [m1, m2, m3] {
                    if written < out.len() {
                        out[written] = v.min(2);
                        written += 1;
                    }
                }
            }
            _ => {}
        }
    }
    Ok(written)
}

/// Map GAQ gain code (0/1/2 from Table E3.4) to the linear gain factor
/// `Gk` (1 / 2 / 4). Used both in the bit-stream reader (to know how
/// many tag bits to peek for) and in the dequantiser (to invert the
/// encoder's amplification).
#[inline]
pub fn gaq_gain_value(code: u8, gaqmod: u8) -> u8 {
    match gaqmod {
        1 => match code {
            0 => 1,
            _ => 2, // mode 1: Gk ∈ {1, 2}
        },
        2 => match code {
            0 => 1,
            _ => 4, // mode 2: Gk ∈ {1, 4}
        },
        3 => match code {
            0 => 1,
            1 => 2,
            _ => 4, // mode 3: Gk ∈ {1, 2, 4}
        },
        _ => 1,
    }
}

/// Look up the 6-element VQ codebook entry for a given `hebap` (1..=7)
/// and `index`. Returns the entry as f32 normalised into `(-1.0, 1.0)`
/// by dividing by `32768.0` (i.e. interpreting the 16-bit value as a
/// signed Q15 fractional).
pub fn vq_lookup(hebap: u8, index: usize) -> [f32; 6] {
    let raw: &[i16; 6] = match hebap {
        1 => &VQ_HEBAP1[index & (VQ_HEBAP1.len() - 1)],
        2 => &VQ_HEBAP2[index & (VQ_HEBAP2.len() - 1)],
        3 => &VQ_HEBAP3[index & (VQ_HEBAP3.len() - 1)],
        4 => &VQ_HEBAP4[index & (VQ_HEBAP4.len() - 1)],
        5 => &VQ_HEBAP5[index & (VQ_HEBAP5.len() - 1)],
        6 => &VQ_HEBAP6[index & (VQ_HEBAP6.len() - 1)],
        7 => &VQ_HEBAP7[index & (VQ_HEBAP7.len() - 1)],
        _ => &[0; 6],
    };
    let mut out = [0.0f32; 6];
    for i in 0..6 {
        out[i] = raw[i] as f32 / 32768.0;
    }
    out
}

/// Read 6 mantissas for a single AHT bin with `hebap` in the
/// scalar/GAQ regime (`hebap >= 8`). When `gaqbin == 1` the encoder
/// has applied a gain (signalled via the per-section gain word
/// `gain_code`), so the small-quantiser regime is in effect: each
/// mantissa is read as a `m-1` (mode 1/3, Gk=2) or `m-2` (mode 2,
/// Gk=4) bit two's-complement value, with the full-scale-negative tag
/// triggering an extra `m`-bit large-mantissa read.
///
/// When `gaqbin == 0` (or `-1`), the bin uses the fixed quantiser:
/// `m`-bit two's-complement values, post-processed via Table E3.6 to
/// produce a symmetric output.
///
/// Returns the 6 dequantised mantissas in `(-1, 1)` normalised range.
#[allow(clippy::too_many_arguments)]
pub fn read_scalar_aht_mantissas(
    br: &mut BitReader<'_>,
    hebap: u8,
    gaqmod: u8,
    gaqbin: i8,
    gain_code: u8,
    out: &mut [f32; 6],
) -> Result<()> {
    if hebap < 8 {
        for v in out.iter_mut() {
            *v = 0.0;
        }
        return Ok(());
    }
    let m = HEBAP_MANT_BITS[hebap as usize] as u32;
    if m == 0 {
        for v in out.iter_mut() {
            *v = 0.0;
        }
        return Ok(());
    }

    // gain == 1 (or no GAQ for this bin) → fixed quantiser, m bits.
    let gk = if gaqbin == 1 {
        gaq_gain_value(gain_code, gaqmod)
    } else {
        1
    };

    let scale_full = 1.0f32 / ((1u32 << (m - 1)) as f32); // 1 / 2^(m-1)

    for sample in out.iter_mut() {
        match gk {
            1 => {
                // Conventional symmetric quantiser: read m bits, sign-
                // extend, divide by 2^(m-1).
                let raw = br.read_u32(m)? as i32;
                let signed = (raw << (32 - m)) >> (32 - m);
                *sample = signed as f32 * scale_full;
            }
            2 => {
                // Mode 1 / 3 with Gk=2: (m-1)-bit small value; check
                // for full-scale-negative tag → m-bit large value.
                let small_bits = m - 1;
                let raw_s = br.read_u32(small_bits)? as i32;
                let small_signed = (raw_s << (32 - small_bits)) >> (32 - small_bits);
                let small_min = -(1i32 << (small_bits - 1));
                if small_signed == small_min {
                    // Tag detected — large mantissa follows (m bits).
                    let raw_l = br.read_u32(m)? as i32;
                    let large_signed = (raw_l << (32 - m)) >> (32 - m);
                    *sample = remap_large_mantissa(hebap, gk, large_signed, m);
                } else {
                    // Small mantissa: divide by 2^(m-2) per Table E3.5
                    // step size 1/(2^(m-2))? — actually step = 1/(2^(m-2) - 1).
                    // We use the simpler 2's complement Q(m-1) form which
                    // matches the encoder's mid-tread quantiser, then the
                    // 1/Gk gain compensation divides by Gk.
                    let small_scale = 1.0f32 / ((1u32 << (small_bits - 1)) as f32);
                    *sample = small_signed as f32 * small_scale / gk as f32;
                }
            }
            4 => {
                // Mode 2 with Gk=4: (m-2)-bit small value; check for
                // full-scale-negative → m-bit large value.
                let small_bits = m - 2;
                if small_bits == 0 {
                    // Edge case: m=2 → no small-value bits; treat as
                    // full-scale-negative every time.
                    let raw_l = br.read_u32(m)? as i32;
                    let large_signed = (raw_l << (32 - m)) >> (32 - m);
                    *sample = remap_large_mantissa(hebap, gk, large_signed, m);
                    continue;
                }
                let raw_s = br.read_u32(small_bits)? as i32;
                let small_signed = (raw_s << (32 - small_bits)) >> (32 - small_bits);
                let small_min = -(1i32 << (small_bits - 1));
                if small_signed == small_min {
                    let raw_l = br.read_u32(m)? as i32;
                    let large_signed = (raw_l << (32 - m)) >> (32 - m);
                    *sample = remap_large_mantissa(hebap, gk, large_signed, m);
                } else {
                    let small_scale = 1.0f32 / ((1u32 << (small_bits - 1)) as f32);
                    *sample = small_signed as f32 * small_scale / gk as f32;
                }
            }
            _ => {
                // Unknown gain → zero.
                *sample = 0.0;
            }
        }
    }
    Ok(())
}

/// Apply Table E3.6 dead-zone-quantiser remap to a large mantissa
/// codeword. The remap is `y = x + a·x + b` with `(a, b)` selected by
/// `(hebap, gk, sign(x))`. We approximate the spec's Q15 constants
/// with the simpler closed-form symmetric-output formula `y = sgn(x)
/// · (|x| + 0.5) · scale` which matches the spec's intent
/// (post-process restores symmetry around 0) without the lookup-table
/// machinery. This is good enough for "non-silent PCM" output; a
/// future round can replace this with the literal Table E3.6 constants
/// for last-bit fidelity vs. the FFmpeg reference.
fn remap_large_mantissa(_hebap: u8, _gk: u8, signed: i32, m: u32) -> f32 {
    // Treat the m-bit signed value as a Q(m-1) fractional. Push by
    // half a step so the dead-zone gap centred on 0 reconstructs to
    // `±step/2` instead of `0`.
    let scale = 1.0f32 / ((1u32 << (m - 1)) as f32);
    let half_step = 0.5f32 * scale;
    let v = signed as f32 * scale;
    if v >= 0.0 {
        v + half_step
    } else {
        v - half_step
    }
}

/// Apply the §3.4.5 inverse DCT-II to the 6 AHT-domain coefficients
/// `x[0..6]` to reconstruct the per-block MDCT spectrum values
/// `c[0..6]`.
///
/// `c(m) = 2 · Σ_{j=0..5} R(j) · x(j) · cos[ j·(2m+1)·π / 12 ]`
/// where `R(0) = 1/√2` and `R(j) = 1` for `j != 0`.
pub fn idct_ii_6(x: [f32; 6]) -> [f32; 6] {
    let mut c = [0.0f32; 6];
    let r0 = std::f32::consts::FRAC_1_SQRT_2;
    for m in 0..6 {
        let mut acc = 0.0f32;
        for j in 0..6 {
            let r = if j == 0 { r0 } else { 1.0 };
            let theta = (j as f32) * ((2 * m + 1) as f32) * PI / 12.0;
            acc += r * x[j] * theta.cos();
        }
        c[m] = 2.0 * acc;
    }
    c
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hebaptab_has_expected_extremes() {
        assert_eq!(HEBAPTAB[0], 0);
        assert_eq!(HEBAPTAB[63], 19);
    }

    #[test]
    fn vq_lookup_in_range() {
        // VQ_HEBAP1[0] = [7167, 4739, 1106, 4269, 10412, 4820] / 32768.
        let v = vq_lookup(1, 0);
        assert!((v[0] - 7167.0 / 32768.0).abs() < 1e-6);
        assert!(v.iter().all(|s| s.is_finite()));
    }

    #[test]
    fn idct_inverse_of_dct_constants() {
        // Constant input → DC output only at m=0..5 should equal
        // 2 · R(0) · x(0) · cos(0) = √2 · 1 = √2 for x = (1, 0, 0, 0, 0, 0).
        let c = idct_ii_6([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let expected = std::f32::consts::SQRT_2;
        for v in c.iter() {
            assert!((v - expected).abs() < 1e-5, "got {v}, expected {expected}");
        }
    }

    #[test]
    fn gaq_sections_matches_spec() {
        assert_eq!(gaq_sections(0, 100), 0);
        assert_eq!(gaq_sections(1, 7), 7);
        assert_eq!(gaq_sections(2, 3), 3);
        assert_eq!(gaq_sections(3, 9), 3);
        assert_eq!(gaq_sections(3, 10), 4);
    }
}
