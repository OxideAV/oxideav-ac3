//! Enhanced-coupling sub-band / band geometry — ATSC A/52:2018 Annex E
//! §E.2.3.3.16-19 + §E.3.5.2.
//!
//! Enhanced coupling (`ecplinu == 1`) reconstructs the high-frequency
//! transform coefficients of the fbw channels from a single shared
//! *enhanced coupling channel* plus per-band amplitude / angle / chaos
//! parameters (§E.3.5.5). Before any of that synthesis can run, the
//! decoder has to know the **band geometry**: which transform
//! coefficients (bins) belong to the enhanced-coupling region, how the
//! 22 fixed sub-bands of Table E3.7 are grouped into the variable
//! coupling *bands* that carry one coordinate each, and how many such
//! bands (`necplbnd`) exist for the current block.
//!
//! This module is the pure, spec-tabulated geometry layer. It carries:
//!
//! * [`begin_subbnd`] / [`end_subbnd`] — the Table E3.8 derivations of
//!   `ecpl_begin_subbnd` (from `ecplbegf`) and `ecpl_end_subbnd` (from
//!   `ecplendf`, or from the SPX begin when SPX is co-active).
//! * [`ECPL_SUBBND_TAB`] — Table E3.9 `ecplsubbndtab[]`, the starting
//!   transform-coefficient number of each of the 22 sub-bands (plus the
//!   one-past-the-end sentinel at index 22).
//! * [`DEFAULT_ECPL_BNDSTRC`] — Table E2.14 `defecplbndstrc[]`, the
//!   default banding used the first time enhanced coupling is active in
//!   a frame when `ecplbndstrce == 0`.
//! * [`necplbnd`] — §E.2.3.3.19 band-count derivation from the
//!   per-sub-band `ecplbndstrc[]` merge bits.
//! * [`band_bin_counts`] — the §E.3.5.5.1 `nbins_per_bnd_array[]`
//!   population: how many transform coefficients each enhanced coupling
//!   band spans.
//!
//! It deliberately does **not** perform any synthesis: amplitude /
//! angle / chaos parameter decode (Tables E3.10-E3.12) and the complex
//! coordinate reconstruction (§E.3.5.5) are a separate step still under
//! construction. Keeping the geometry isolated and unit-tested gives the
//! synthesis step a verified foundation.
//!
//! As of round 300 the module also carries the **bitstream-syntax** layer
//! that sits on top of the geometry: [`parse_strategy`] reads the
//! §E.2.3.3.16-19 strategy fields and [`parse_coords`] reads the
//! §E.2.3.3.20-26 per-band amplitude / angle / chaos coordinates. These
//! advance the bit cursor exactly per the reference syntax so an
//! enhanced-coupling block can be walked without desync.
//!
//! Round 306 adds the **§E.3.5.5.2 / §E.3.5.5.3 parameter-processing**
//! layer: [`ampbnd`] / [`angle_value`] / [`chaos_value`] decode the
//! Table E3.10-E3.12 index triples, [`process_band_amplitudes`] applies
//! the §E.3.5.5.2 chaos amplitude modification, [`expand_bands_to_bins`]
//! fans the per-band values out to per-bin `ampbin[]`, and
//! [`interpolate_bin_angles`] implements the `ecplangleintrp == 1`
//! linear-interpolation path (§E.3.5.5.3). These turn the decoded index
//! triples into the per-bin amplitude / angle arrays the synthesis
//! consumes — pure tabulated arithmetic with no multi-block state.
//!
//! The two remaining deferred pieces of §E.3.5.5 are the
//! §E.3.5.5.1 prev/curr/next IMDCT + overlap-add + FFT that produces the
//! non-aliased complex coupling channel, and the §E.3.5.5.4 per-bin
//! complex product + `rand[]` de-correlation arrays — both stateful
//! across blocks (and the latter needs an init-time RNG), so they stay
//! out of this pure layer.
//!
//! **Spec note (erratum):** the default-banding table is captioned
//! "Table E2.14" in the document's table-of-contents (and list of
//! tables) but is cross-referenced as "Table E2.13" from the body of
//! §E.2.3.3.18 — the latter collides with the *standard* coupling
//! default at the genuine Table E2.13. The two tables hold different
//! values; the enhanced-coupling values used here are those listed in
//! full under the §E.2.3.3.18 heading.

use oxideav_core::bits::BitReader;
use oxideav_core::Result;

/// Table E3.9 — `ecplsubbndtab[]`. The starting transform-coefficient
/// number of each of the 22 enhanced-coupling sub-bands, with a
/// one-past-the-end sentinel (`253`) at index 22 so the half-open span
/// of sub-band `s` is `ECPL_SUBBND_TAB[s] .. ECPL_SUBBND_TAB[s + 1]`.
///
/// Sub-bands 0..=3 are 6 bins wide (13..18, 19..24, 25..30, 31..36);
/// sub-bands 4..=21 are 12 bins wide. The enhanced-coupling region thus
/// spans transform coefficients 13..=252.
pub const ECPL_SUBBND_TAB: [usize; 23] = [
    13, 19, 25, 31, 37, 49, 61, 73, 85, 97, 109, 121, 133, 145, 157, 169, 181, 193, 205, 217, 229,
    241, 253,
];

/// Number of enhanced-coupling sub-bands defined by Table E3.7.
pub const N_ECPL_SUBBND: usize = 22;

/// Table E2.14 — `defecplbndstrc[]`. The default enhanced-coupling
/// banding structure, indexed by absolute sub-band number. A `true`
/// ('1') entry means "merge sub-band `s` into the previous band". Per
/// §E.2.3.3.19 the merge bits for sub-bands `<= max(ecpl_begin_subbnd,
/// 8)` are always zero (and not transmitted); the table reflects that —
/// sub-bands 0..=8 are all `false`, with the first merge at sub-band 9.
pub const DEFAULT_ECPL_BNDSTRC: [bool; N_ECPL_SUBBND] = {
    let mut t = [false; N_ECPL_SUBBND];
    // §E.2.3.3.18 default: sub-bands 0..8 → 0; then the per-row values.
    t[9] = true;
    // t[10] = false (12 → 0)
    t[11] = true;
    // t[12] = false
    t[13] = true;
    // t[14] = false
    t[15] = true;
    t[16] = true;
    t[17] = true;
    // t[18] = false
    t[19] = true;
    t[20] = true;
    t[21] = true;
    t
};

/// §E.2.3.3.16 — derive `ecpl_begin_subbnd` from the 4-bit `ecplbegf`
/// code (Table E3.8).
///
/// ```text
/// if (ecplbegf < 3)       ecpl_begin_subbnd = ecplbegf * 2
/// else if (ecplbegf < 13) ecpl_begin_subbnd = ecplbegf + 2
/// else                    ecpl_begin_subbnd = ecplbegf * 2 - 10
/// ```
#[inline]
pub fn begin_subbnd(ecplbegf: u8) -> usize {
    let f = ecplbegf as usize;
    if f < 3 {
        f * 2
    } else if f < 13 {
        f + 2
    } else {
        f * 2 - 10
    }
}

/// §E.2.3.3.17 — derive `ecpl_end_subbnd` (one greater than the highest
/// active enhanced-coupling sub-band), per Table E3.8.
///
/// When spectral extension is **not** in use the end sub-band is taken
/// directly from the 4-bit `ecplendf` code (`ecplendf + 7`). When SPX
/// **is** co-active the enhanced-coupling region is instead bounded by
/// the SPX begin so the two regions abut: `spxbegf + 5` for
/// `spxbegf < 6`, else `spxbegf * 2`. In the SPX-active case `ecplendf`
/// is not transmitted.
#[inline]
pub fn end_subbnd(spxinu: bool, ecplendf: u8, spxbegf: usize) -> usize {
    if !spxinu {
        ecplendf as usize + 7
    } else if spxbegf < 6 {
        spxbegf + 5
    } else {
        spxbegf * 2
    }
}

/// §E.2.3.3.19 — number of enhanced-coupling bands.
///
/// ```text
/// necplbnd  = ecpl_end_subbnd - ecpl_begin_subbnd;
/// necplbnd -= sum(ecplbndstrc[ecpl_begin_subbnd ..= ecpl_end_subbnd-1])
/// ```
///
/// Each set merge bit collapses one sub-band into the previous band, so
/// the count is the number of sub-bands in the active span minus the
/// number of merges. `bndstrc` is indexed by absolute sub-band number.
#[inline]
pub fn necplbnd(begin: usize, end: usize, bndstrc: &[bool; N_ECPL_SUBBND]) -> usize {
    let span = end.saturating_sub(begin);
    let merges = bndstrc[begin..end.min(N_ECPL_SUBBND)]
        .iter()
        .filter(|&&b| b)
        .count();
    span - merges
}

/// §E.3.5.5.1 — populate `nbins_per_bnd_array[]`: the number of
/// transform-coefficient bins spanned by each enhanced-coupling band.
///
/// Walking the active sub-band span, a `0` merge bit opens a new band
/// and a `1` merge bit extends the current band; each sub-band
/// contributes `ecplsubbndtab[s+1] - ecplsubbndtab[s]` bins (6 for the
/// narrow low sub-bands, 12 for the rest). The returned vector has
/// length [`necplbnd`].
pub fn band_bin_counts(begin: usize, end: usize, bndstrc: &[bool; N_ECPL_SUBBND]) -> Vec<usize> {
    let mut counts: Vec<usize> = Vec::new();
    for sbnd in begin..end.min(N_ECPL_SUBBND) {
        let bins = ECPL_SUBBND_TAB[sbnd + 1] - ECPL_SUBBND_TAB[sbnd];
        if !bndstrc[sbnd] {
            // New band.
            counts.push(bins);
        } else if let Some(last) = counts.last_mut() {
            // Merge into the current band.
            *last += bins;
        } else {
            // Defensive: a leading merge bit with no open band. The spec
            // guarantees `ecplbndstrc[begin] == 0`, so this can only be
            // reached on malformed input; treat it as a new band.
            counts.push(bins);
        }
    }
    counts
}

/// First transform-coefficient number of the enhanced-coupling region
/// for the given begin sub-band (Table E3.9 lookup). This is where the
/// shared enhanced-coupling channel starts; below it every channel is
/// independently coded.
#[inline]
pub fn begin_bin(begin: usize) -> usize {
    ECPL_SUBBND_TAB[begin.min(N_ECPL_SUBBND)]
}

/// One-past-the-last transform-coefficient number of the
/// enhanced-coupling region for the given end sub-band (Table E3.9
/// lookup).
#[inline]
pub fn end_bin(end: usize) -> usize {
    ECPL_SUBBND_TAB[end.min(N_ECPL_SUBBND)]
}

/// Maximum number of enhanced-coupling bands. Equals [`N_ECPL_SUBBND`]
/// (no merge bits → every sub-band is its own band), so a fixed-size
/// per-band buffer never overflows.
pub const MAX_ECPL_BND: usize = N_ECPL_SUBBND;

/// The decoded **enhanced-coupling strategy** for a block (§E.2.3.3.16-19).
///
/// This is the resolved geometry the coordinate parse + the eventual
/// §E.3.5.5 synthesis consume: the active sub-band span, the per-sub-band
/// merge structure, and the derived band count. It is produced by
/// [`parse_strategy`] from the `cplstre[blk] && cplinu[blk] && ecplinu`
/// branch of Table E1.4, or carried forward unchanged on a block whose
/// `cplstre[blk]` is `0` (strategy reuse).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct EcplStrategy {
    /// `ecpl_begin_subbnd` — index of the first active sub-band
    /// (§E.2.3.3.16).
    pub begin_subbnd: usize,
    /// `ecpl_end_subbnd` — one greater than the highest active sub-band
    /// (§E.2.3.3.17).
    pub end_subbnd: usize,
    /// Resolved `ecplbndstrc[]`, indexed by absolute sub-band number: a
    /// `true` entry merges that sub-band into the previous band.
    pub bndstrc: [bool; N_ECPL_SUBBND],
    /// `necplbnd` — number of enhanced-coupling bands (§E.2.3.3.19).
    pub necplbnd: usize,
}

impl EcplStrategy {
    /// First transform-coefficient (bin) of the enhanced-coupling region.
    #[inline]
    pub fn begin_bin(&self) -> usize {
        begin_bin(self.begin_subbnd)
    }

    /// One-past-the-last transform-coefficient (bin) of the region.
    #[inline]
    pub fn end_bin(&self) -> usize {
        end_bin(self.end_subbnd)
    }

    /// Per-band bin counts (`nbins_per_bnd_array[]`, §E.3.5.5.1).
    #[inline]
    pub fn band_bin_counts(&self) -> Vec<usize> {
        band_bin_counts(self.begin_subbnd, self.end_subbnd, &self.bndstrc)
    }
}

/// Parse the enhanced-coupling **strategy** block (§E.2.3.3.16-19 / the
/// `ecplinu` arm of Table E1.4, reached only when
/// `cplstre[blk] && cplinu[blk] && ecplinu`).
///
/// Field order (each `read_u32` advances the cursor exactly as the
/// reference syntax does):
///
/// 1. `ecplbegf` (4 bits) → `ecpl_begin_subbnd` via [`begin_subbnd`].
/// 2. `ecplendf` (4 bits) **only when SPX is off**; when SPX is active
///    `ecpl_end_subbnd` is derived from `spxbegf` and `ecplendf` is *not*
///    transmitted ([`end_subbnd`]).
/// 3. `ecplbndstrce` (1 bit). When set, the per-sub-band merge bits
///    `ecplbndstrc[sbnd]` follow for
///    `sbnd in [max(9, ecpl_begin_subbnd + 1), ecpl_end_subbnd)` — the
///    sub-bands up to and including `max(8, ecpl_begin_subbnd)` are known
///    to be `0` and are never sent (§E.2.3.3.19).
///
/// `ecplbndstrce == 0` means *use the default / reuse the previous*
/// structure. The caller supplies `prev_bndstrc`: pass
/// [`DEFAULT_ECPL_BNDSTRC`] on the first block of the frame that enables
/// enhanced coupling, or the previously-decoded structure on a later
/// block (§E.2.3.3.18). When `ecplbndstrce == 1` the supplied default is
/// ignored and a fresh all-`false` base is populated from the wire bits.
pub fn parse_strategy(
    br: &mut BitReader<'_>,
    spxinu: bool,
    spxbegf: usize,
    prev_bndstrc: &[bool; N_ECPL_SUBBND],
) -> Result<EcplStrategy> {
    let ecplbegf = br.read_u32(4)? as u8;
    let begin = begin_subbnd(ecplbegf);
    let ecplendf = if spxinu { 0 } else { br.read_u32(4)? as u8 };
    let end = end_subbnd(spxinu, ecplendf, spxbegf);

    let ecplbndstrce = br.read_u32(1)? != 0;
    let bndstrc = if ecplbndstrce {
        // Fresh structure from the wire. Sub-bands up to and including
        // max(8, begin) are implicitly 0 and not transmitted; the loop
        // starts at max(9, begin + 1). The merge bit for the very first
        // active sub-band is therefore never sent — it always starts a
        // band (§E.2.3.3.19).
        let mut t = [false; N_ECPL_SUBBND];
        let lo = (begin + 1).max(9);
        for sbnd in lo..end.min(N_ECPL_SUBBND) {
            t[sbnd] = br.read_u32(1)? != 0;
        }
        t
    } else {
        // Reuse the default (first block) or the previous block's
        // structure (later block) — no bits consumed.
        *prev_bndstrc
    };

    let necplbnd = necplbnd(begin, end, &bndstrc);
    Ok(EcplStrategy {
        begin_subbnd: begin,
        end_subbnd: end,
        bndstrc,
        necplbnd,
    })
}

/// Per-channel decoded enhanced-coupling **parameters** for a block
/// (§E.2.3.3.21-26). Only channels in coupling carry an entry; the angle
/// and chaos arrays of the *first* coupled channel are spec-fixed to `0`
/// and not transmitted, so they stay empty on that channel.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EcplChannelParams {
    /// `ecplparam1e[ch]` — amplitudes present this block.
    pub param1e: bool,
    /// `ecplparam2e[ch]` — angle + chaos present this block.
    pub param2e: bool,
    /// `ecplamp[ch][bnd]` — 5-bit amplitude index per band (present iff
    /// `param1e`).
    pub amp: Vec<u8>,
    /// `ecplangle[ch][bnd]` — 6-bit angle index per band (present iff
    /// `param2e` and not the first coupled channel).
    pub angle: Vec<u8>,
    /// `ecplchaos[ch][bnd]` — 3-bit chaos index per band (present iff
    /// `param2e` and not the first coupled channel).
    pub chaos: Vec<u8>,
    /// `ecpltrans[ch]` — transient-present flag (not transmitted for the
    /// first coupled channel, where it is `false`).
    pub trans: bool,
}

/// The decoded enhanced-coupling **coordinate** block for one audio block
/// (§E.2.3.3.20-26 / the `ecplinu` arm of the coupling-coordinate loop in
/// Table E1.4, reached when `cplinu[blk] && ecplinu`).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EcplCoords {
    /// `ecplangleintrp` — angle-interpolation flag (§E.2.3.3.20).
    pub angleintrp: bool,
    /// Per-front-channel parameters, indexed by channel number. Channels
    /// not in coupling carry a default (all-`false`) entry.
    pub channels: Vec<EcplChannelParams>,
}

/// Parse the enhanced-coupling **coordinate** block (§E.2.3.3.20-26).
///
/// `nfchans` is the number of full-bandwidth channels; `chincpl[ch]`
/// flags which of them are in coupling. `firstcplcos[ch]` is the
/// per-channel "first coupling block this frame" marker (the same one the
/// standard-coupling `cplcoe` gate uses): on the first block a channel
/// enters coupling, its `ecplparam1e`/`ecplparam2e` are *implicit* (not
/// read from the wire) and all parameters are forced present — the spec
/// guarantees every channel transmits its full parameter set the first
/// time enhanced coupling is enabled. The marker is cleared in place so
/// later blocks read the explicit exist bits.
///
/// `necplbnd` is the band count from the active [`EcplStrategy`]. The
/// first coupled channel (`firstchincpl`) never carries `ecplparam2e`,
/// `ecplangle`, `ecplchaos`, or `ecpltrans` — its angle/chaos are
/// spec-fixed to `0` (§E.2.3.3.24-26).
pub fn parse_coords(
    br: &mut BitReader<'_>,
    nfchans: usize,
    chincpl: &[bool],
    firstcplcos: &mut [bool],
    necplbnd: usize,
) -> Result<EcplCoords> {
    let angleintrp = br.read_u32(1)? != 0;
    let mut channels = vec![EcplChannelParams::default(); nfchans];

    // firstchincpl = -1 → the first channel actually in coupling.
    let mut firstchincpl: Option<usize> = None;

    for ch in 0..nfchans {
        if !chincpl[ch] {
            // §E.2.3.3 "!chincpl[ch]" arm: re-arm the first-coupling
            // marker so a later block re-entering coupling treats its
            // parameters as implicit-present again.
            firstcplcos[ch] = true;
            continue;
        }
        let is_first = firstchincpl.is_none();
        if is_first {
            firstchincpl = Some(ch);
        }

        let (param1e, param2e) = if firstcplcos[ch] {
            // First block this channel is in coupling: parameters are
            // implicit. param1e is always 1; param2e is 1 only for
            // channels after the first coupled channel.
            firstcplcos[ch] = false;
            (true, !is_first)
        } else {
            let p1 = br.read_u32(1)? != 0;
            // param2e is transmitted only for channels after the first
            // coupled channel; the first coupled channel's angle/chaos
            // are fixed to 0, so it has no param2e bit.
            let p2 = if !is_first {
                br.read_u32(1)? != 0
            } else {
                false
            };
            (p1, p2)
        };

        let mut params = EcplChannelParams {
            param1e,
            param2e,
            ..Default::default()
        };

        if param1e {
            params.amp.reserve_exact(necplbnd);
            for _ in 0..necplbnd {
                params.amp.push(br.read_u32(5)? as u8);
            }
        }
        if param2e {
            params.angle.reserve_exact(necplbnd);
            params.chaos.reserve_exact(necplbnd);
            for _ in 0..necplbnd {
                params.angle.push(br.read_u32(6)? as u8);
                params.chaos.push(br.read_u32(3)? as u8);
            }
        }
        // ecpltrans[ch] is transmitted only for channels after the first
        // coupled channel.
        if !is_first {
            params.trans = br.read_u32(1)? != 0;
        }

        channels[ch] = params;
    }

    Ok(EcplCoords {
        angleintrp,
        channels,
    })
}

// ===========================================================================
// §E.3.5.5.2 / §E.3.5.5.3 — enhanced-coupling parameter processing
// ===========================================================================
//
// This layer turns the decoded per-band index triples (`ecplamp` /
// `ecplangle` / `ecplchaos` carried in [`EcplChannelParams`]) into the
// per-*bin* amplitude and angle arrays the §E.3.5.5.4 complex-product
// synthesis consumes. It is pure tabulated arithmetic over the active
// band geometry — no multi-block state, no FFT.
//
// The two remaining (deferred) pieces of §E.3.5.5 are:
//   * §E.3.5.5.1 — the prev/curr/next IMDCT + overlap-add + FFT that
//     produces the non-aliased complex coupling channel `Z[k]`, and
//   * §E.3.5.5.4 — the per-bin complex product + the `rand[]`
//     de-correlation arrays.
// Both are stateful across blocks (and need an init-time RNG for the
// `rand_notrans` array), so they stay out of this pure layer.

/// Table E3.10 — `ecplampexptab[]`. The binary exponent (right-shift
/// count) applied to the mantissa for each 5-bit `ecplamp` index. Index
/// 31 (minus-infinity dB) has no exponent and is handled specially in
/// [`ampbnd`]; its slot here is `0` and never read on that path.
pub const ECPL_AMP_EXP_TAB: [u8; 32] = [
    0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 0,
];

/// Table E3.10 — `ecplampmanttab[]`. The 6-bit mantissa for each 5-bit
/// `ecplamp` index (the `/32` in [`ampbnd`] applies the implicit binary
/// point). Index 31 is `0x00` (minus-infinity dB → amplitude 0).
pub const ECPL_AMP_MANT_TAB: [u8; 32] = [
    0x20, 0x1b, 0x17, 0x13, 0x10, 0x1b, 0x17, 0x13, 0x10, 0x1b, 0x17, 0x13, 0x10, 0x1b, 0x17, 0x13,
    0x10, 0x1b, 0x17, 0x13, 0x10, 0x1b, 0x17, 0x13, 0x10, 0x1b, 0x17, 0x13, 0x10, 0x1b, 0x17, 0x00,
];

/// Table E3.11 — `ecplangletab[]`. The band phase angle (in the spec's
/// normalised `-1.0 ..= 1.0` units, representing `-pi ..= pi`) for each
/// 6-bit `ecplangle` index. Codes 0..=31 are the non-negative angles
/// `0.0, 0.03125, … 0.96875`; codes 32..=63 are the negative angles
/// `-1.0, -0.96875, … -0.03125`.
pub const ECPL_ANGLE_TAB: [f32; 64] = {
    let mut t = [0.0f32; 64];
    let mut i = 0;
    while i < 32 {
        t[i] = i as f32 / 32.0;
        i += 1;
    }
    let mut i = 32;
    while i < 64 {
        t[i] = -1.0 + (i - 32) as f32 / 32.0;
        i += 1;
    }
    t
};

/// Table E3.12 — `ecplchaostab[]`. The chaos scaling factor (in
/// `0.0 ..= -1.0`) for each 3-bit `ecplchaos` index. The values are the
/// eighths `-k/7` for `k = 0..=7`.
pub const ECPL_CHAOS_TAB: [f32; 8] = [
    0.0, -0.142857, -0.285714, -0.428571, -0.571429, -0.714286, -0.857143, -1.0,
];

/// §E.3.5.5.2 — the band amplitude `ampbnd[ch][bnd]` for a single 5-bit
/// `ecplamp` index, *before* the chaos modification.
///
/// ```text
/// if (ecplamp == 31)  ampbnd = 0
/// else                ampbnd = (ecplampmanttab[ecplamp] / 32) >> ecplampexptab[ecplamp]
/// ```
///
/// The spec's `>> exp` is a fixed-point right shift, i.e. division by
/// `2^exp`; we evaluate it in floating point as `(mant / 32) / 2^exp`.
/// Index 0 (`mant = 0x20`, `exp = 0`) yields `1.0` (0 dB); index 30
/// (`mant = 0x17`, `exp = 7`) yields `≈ 0.005615` (≈ -45.01 dB); index
/// 31 yields `0.0` (minus-infinity dB).
#[inline]
pub fn ampbnd(ecplamp: u8) -> f32 {
    let idx = (ecplamp & 0x1f) as usize;
    if idx == 31 {
        return 0.0;
    }
    let mant = ECPL_AMP_MANT_TAB[idx] as f32 / 32.0;
    let exp = ECPL_AMP_EXP_TAB[idx] as i32;
    mant / 2f32.powi(exp)
}

/// §E.3.5.5.2 — the chaos value `chaos[ch][bnd]` for a single 3-bit
/// `ecplchaos` index. The first coupled channel always reads `0.0`
/// regardless of the index (its chaos/angle are spec-fixed to zero).
#[inline]
pub fn chaos_value(ecplchaos: u8, is_first_coupled: bool) -> f32 {
    if is_first_coupled {
        0.0
    } else {
        ECPL_CHAOS_TAB[(ecplchaos & 0x07) as usize]
    }
}

/// §E.3.5.5.3 — the band angle `angle[ch][bnd]` for a single 6-bit
/// `ecplangle` index. The first coupled channel always reads `0.0`.
#[inline]
pub fn angle_value(ecplangle: u8, is_first_coupled: bool) -> f32 {
    if is_first_coupled {
        0.0
    } else {
        ECPL_ANGLE_TAB[(ecplangle & 0x3f) as usize]
    }
}

/// §E.3.5.5.2 — the fully-processed per-band amplitudes for one channel,
/// including the chaos modification.
///
/// For each band, [`ampbnd`] converts the `ecplamp` index to a linear
/// gain, then the chaos modification
///
/// ```text
/// if (ecpltrans == 0 && ch != firstchincpl)
///     ampbnd[bnd] *= 1 + 0.38 * chaos[bnd]
/// ```
///
/// scales it (note `chaos` is `<= 0`, so this *reduces* the amplitude of
/// non-transient, non-first coupled channels). `is_first_coupled` carries
/// the `ch == firstchincpl` test; `trans` carries `ecpltrans[ch]`.
///
/// Returns a vector of length `necplbnd` (the length of `params.amp`).
pub fn process_band_amplitudes(params: &EcplChannelParams, is_first_coupled: bool) -> Vec<f32> {
    let mut out = Vec::with_capacity(params.amp.len());
    for (bnd, &amp_idx) in params.amp.iter().enumerate() {
        let mut a = ampbnd(amp_idx);
        // The chaos modification applies only to non-first coupled
        // channels with no transient. `chaos`/`angle` are only present
        // for those channels (param2e), so a missing entry means 0.
        if !params.trans && !is_first_coupled {
            let chaos_idx = params.chaos.get(bnd).copied().unwrap_or(0);
            let chaos = chaos_value(chaos_idx, is_first_coupled);
            a *= 1.0 + 0.38 * chaos;
        }
        out.push(a);
    }
    out
}

/// §E.3.5.5.2 — expand per-band values to per-sub-band then per-bin.
///
/// `band_vals` holds one value per enhanced-coupling *band*; this fans
/// each band's value out across every transform-coefficient bin it spans,
/// using the merge structure `bndstrc[]` to walk the same band boundaries
/// the parse used. The returned vector is indexed by *bin offset from
/// `begin_bin`* (i.e. element `0` is transform coefficient
/// `ecplsubbndtab[begin_subbnd]`), with length
/// `end_bin(end) - begin_bin(begin)`.
///
/// This is the §E.3.5.5.2 `ampbin[ch][bin]` reconstruction; the same
/// fan-out applies to the no-interpolation angle path (§E.3.5.5.3) and to
/// the per-bin chaos / random expansion.
pub fn expand_bands_to_bins(
    begin: usize,
    end: usize,
    bndstrc: &[bool; N_ECPL_SUBBND],
    band_vals: &[f32],
) -> Vec<f32> {
    let total = end_bin(end).saturating_sub(begin_bin(begin));
    let mut out = Vec::with_capacity(total);
    // `bnd` tracks the current band index into `band_vals`. The first
    // active sub-band always opens band 0 (its merge bit is never sent).
    let mut bnd: isize = -1;
    for sbnd in begin..end.min(N_ECPL_SUBBND) {
        if !bndstrc[sbnd] {
            bnd += 1;
        }
        let val = if bnd >= 0 {
            band_vals.get(bnd as usize).copied().unwrap_or(0.0)
        } else {
            // Defensive: a leading merge bit (spec guarantees the first
            // active sub-band starts a band, so this is malformed input).
            band_vals.first().copied().unwrap_or(0.0)
        };
        let nbins = ECPL_SUBBND_TAB[sbnd + 1] - ECPL_SUBBND_TAB[sbnd];
        for _ in 0..nbins {
            out.push(val);
        }
    }
    out
}

/// §E.3.5.5.3 — per-bin angle reconstruction with linear interpolation
/// between band centres (the `ecplangleintrp == 1` path).
///
/// `band_angles` holds one angle per band (the [`angle_value`] outputs);
/// `nbins_per_bnd` holds the bin count of each band (from
/// [`band_bin_counts`]). The result is one angle per bin across the
/// active region, length `sum(nbins_per_bnd)`. Angles wrap into the
/// `-1.0 ..= 1.0` interval after each interpolation step, exactly as the
/// reference pseudo-code's `while` guards do.
///
/// The single-band case (`nbands < 2`) has no inter-band slope, so the
/// one band's angle is simply fanned across all its bins.
pub fn interpolate_bin_angles(band_angles: &[f32], nbins_per_bnd: &[usize]) -> Vec<f32> {
    let nbands = band_angles.len().min(nbins_per_bnd.len());
    let total: usize = nbins_per_bnd.iter().take(nbands).sum();
    let mut out = vec![0.0f32; total];
    if nbands == 0 {
        return out;
    }
    if nbands == 1 {
        for slot in out.iter_mut() {
            *slot = band_angles[0];
        }
        return out;
    }

    let mut bin: usize = 0;
    for bnd in 1..nbands {
        let nbins_prev = nbins_per_bnd[bnd - 1];
        let nbins_curr = nbins_per_bnd[bnd];
        let angle_prev = band_angles[bnd - 1];
        let mut angle_curr = band_angles[bnd];
        // Unwrap the current band angle to within one step of the prev.
        while (angle_curr - angle_prev) > 1.0 {
            angle_curr -= 2.0;
        }
        while (angle_prev - angle_curr) > 1.0 {
            angle_curr += 2.0;
        }
        let slope = (angle_curr - angle_prev) / ((nbins_curr + nbins_prev) as f32 / 2.0);

        // Lower half of the first band (walks downward from the centre).
        if bnd == 1 && nbins_prev > 1 {
            let (mut y, mut down_bin): (f32, usize);
            if nbins_prev % 2 == 0 {
                y = angle_prev - slope / 2.0;
                down_bin = nbins_prev / 2 - 1;
            } else {
                y = angle_prev - slope;
                down_bin = (nbins_prev - 3) / 2;
            }
            let count = down_bin + 1;
            for _ in 0..count {
                let ytmp = y;
                while y > 1.0 {
                    y -= 2.0;
                }
                while y < -1.0 {
                    y += 2.0;
                }
                out[down_bin] = y;
                down_bin = down_bin.saturating_sub(1);
                y = ytmp - slope;
            }
            bin = count;
        }

        let (mut y, count): (f32, usize) = if nbins_prev % 2 == 0 {
            (angle_prev + slope / 2.0, nbins_curr / 2 + nbins_prev / 2)
        } else {
            (angle_prev, nbins_curr / 2 + nbins_prev.div_ceil(2))
        };
        for _ in 0..count {
            let ytmp = y;
            while y > 1.0 {
                y -= 2.0;
            }
            while y < -1.0 {
                y += 2.0;
            }
            if bin < total {
                out[bin] = y;
                bin += 1;
            }
            y = ytmp + slope;
        }

        // Finish the last band when this is the final iteration. The
        // reference carries `y`/`slope` out of the loop and runs one
        // closing pass; we mirror that by detecting the last band here.
        if bnd == nbands - 1 {
            let last_count = if nbins_curr % 2 == 0 {
                nbins_curr / 2
            } else {
                nbins_curr / 2 + 1
            };
            for _ in 0..last_count {
                let ytmp = y;
                while y > 1.0 {
                    y -= 2.0;
                }
                while y < -1.0 {
                    y += 2.0;
                }
                if bin < total {
                    out[bin] = y;
                    bin += 1;
                }
                y = ytmp + slope;
            }
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::{BitReader, BitWriter};

    #[test]
    fn ecplsubbndtab_matches_table_e3_9() {
        // Spot-check the Table E3.9 values + the 6/12-bin widths.
        assert_eq!(ECPL_SUBBND_TAB[0], 13);
        assert_eq!(ECPL_SUBBND_TAB[4], 37);
        assert_eq!(ECPL_SUBBND_TAB[21], 241);
        assert_eq!(ECPL_SUBBND_TAB[22], 253); // sentinel
                                              // Sub-bands 0..=3 are 6 bins wide.
        for s in 0..4 {
            assert_eq!(ECPL_SUBBND_TAB[s + 1] - ECPL_SUBBND_TAB[s], 6);
        }
        // Sub-bands 4..=21 are 12 bins wide.
        for s in 4..N_ECPL_SUBBND {
            assert_eq!(ECPL_SUBBND_TAB[s + 1] - ECPL_SUBBND_TAB[s], 12);
        }
        // Region spans tc 13..=252.
        assert_eq!(begin_bin(0), 13);
        assert_eq!(end_bin(22), 253);
    }

    #[test]
    fn begin_subbnd_table_e3_8() {
        // ecplbegf < 3 → *2.
        assert_eq!(begin_subbnd(0), 0);
        assert_eq!(begin_subbnd(1), 2);
        assert_eq!(begin_subbnd(2), 4);
        // 3..=12 → +2.
        assert_eq!(begin_subbnd(3), 5);
        assert_eq!(begin_subbnd(7), 9);
        assert_eq!(begin_subbnd(12), 14);
        // >=13 → *2 - 10.
        assert_eq!(begin_subbnd(13), 16);
        assert_eq!(begin_subbnd(14), 18);
        assert_eq!(begin_subbnd(15), 20);
    }

    #[test]
    fn end_subbnd_table_e3_8_no_spx() {
        // SPX off: ecplendf + 7.
        assert_eq!(end_subbnd(false, 0, 0), 7);
        assert_eq!(end_subbnd(false, 8, 0), 15);
        assert_eq!(end_subbnd(false, 15, 0), 22);
    }

    #[test]
    fn end_subbnd_table_e3_8_with_spx() {
        // SPX on, spxbegf < 6 → spxbegf + 5.
        assert_eq!(end_subbnd(true, 0, 0), 5);
        assert_eq!(end_subbnd(true, 0, 5), 10);
        // SPX on, spxbegf >= 6 → spxbegf * 2.
        assert_eq!(end_subbnd(true, 0, 6), 12);
        assert_eq!(end_subbnd(true, 0, 7), 14);
        // ecplendf is ignored when SPX is active.
        assert_eq!(end_subbnd(true, 15, 6), 12);
    }

    #[test]
    fn default_bndstrc_table_e2_14() {
        // Sub-bands 0..=8 are all zero (never transmitted; always merge=0).
        for s in 0..=8 {
            assert!(!DEFAULT_ECPL_BNDSTRC[s], "sbnd {s} should be 0");
        }
        // The Table E2.14 merge rows.
        assert!(DEFAULT_ECPL_BNDSTRC[9]);
        assert!(!DEFAULT_ECPL_BNDSTRC[10]);
        assert!(DEFAULT_ECPL_BNDSTRC[11]);
        assert!(!DEFAULT_ECPL_BNDSTRC[12]);
        assert!(DEFAULT_ECPL_BNDSTRC[13]);
        assert!(!DEFAULT_ECPL_BNDSTRC[14]);
        assert!(DEFAULT_ECPL_BNDSTRC[15]);
        assert!(DEFAULT_ECPL_BNDSTRC[16]);
        assert!(DEFAULT_ECPL_BNDSTRC[17]);
        assert!(!DEFAULT_ECPL_BNDSTRC[18]);
        assert!(DEFAULT_ECPL_BNDSTRC[19]);
        assert!(DEFAULT_ECPL_BNDSTRC[20]);
        assert!(DEFAULT_ECPL_BNDSTRC[21]);
    }

    #[test]
    fn necplbnd_no_merges() {
        // All-zero banding → every sub-band is its own band.
        let bndstrc = [false; N_ECPL_SUBBND];
        // begin=9, end=22 → 13 sub-bands, no merges → 13 bands.
        assert_eq!(necplbnd(9, 22, &bndstrc), 13);
        // A small span: begin=5, end=8 → 3 sub-bands.
        assert_eq!(necplbnd(5, 8, &bndstrc), 3);
    }

    #[test]
    fn necplbnd_with_default_banding() {
        // Default banding over the full high span begin=9, end=22.
        // 13 sub-bands; merges at 9,11,13,15,16,17,19,20,21 that fall in
        // [9,22) → 9 merges → 13 - 9 = 4 bands.
        let n = necplbnd(9, 22, &DEFAULT_ECPL_BNDSTRC);
        assert_eq!(n, 4);
    }

    #[test]
    fn band_bin_counts_no_merges_low_subbands() {
        // begin=0, end=4 → narrow 6-bin sub-bands, no merges → four
        // 6-bin bands.
        let bndstrc = [false; N_ECPL_SUBBND];
        let counts = band_bin_counts(0, 4, &bndstrc);
        assert_eq!(counts, vec![6, 6, 6, 6]);
    }

    #[test]
    fn band_bin_counts_default_banding_high() {
        // begin=9, end=22 under the default banding. Sub-bands here are
        // all 12-bin wide. Bands open at sub-bands with merge==0
        // (9 opens? no — 9 has merge=1 but it is the first sub-band of
        // the span, so per band_bin_counts the leading merge starts a new
        // band defensively). To match the spec's guarantee that the
        // first sub-band of the active span is never a merge, test a span
        // whose first sub-band has merge=0.
        //
        // begin=10, end=22 → first sub-band 10 has merge=0 (new band).
        // Merge bits in [10,22): 11,13,15,16,17,19,20,21 → 8 merges.
        // 12 sub-bands → 4 bands. Each band's bin count = 12 * (sub-bands
        // in band).
        let counts = band_bin_counts(10, 22, &DEFAULT_ECPL_BNDSTRC);
        // Bands: [10,11]=24, [12,13]=24, [14,15,16,17]=48, [18,19,20,21]=48.
        assert_eq!(counts, vec![24, 24, 48, 48]);
        // Total bins == span bins (12 each * 12 sub-bands = 144).
        let total: usize = counts.iter().sum();
        assert_eq!(total, 144);
        assert_eq!(total, end_bin(22) - begin_bin(10));
    }

    #[test]
    fn band_bin_counts_length_equals_necplbnd() {
        // The vector length must equal necplbnd for any banding.
        let begin = 10;
        let end = 22;
        let n = necplbnd(begin, end, &DEFAULT_ECPL_BNDSTRC);
        let counts = band_bin_counts(begin, end, &DEFAULT_ECPL_BNDSTRC);
        assert_eq!(counts.len(), n);
    }

    // ---- §E.2.3.3.16-19 strategy parse ----

    #[test]
    fn parse_strategy_no_spx_default_banding() {
        // ecplbegf = 7 → begin = 9; SPX off, ecplendf = 15 → end = 22;
        // ecplbndstrce = 0 → reuse the supplied default banding (no merge
        // bits on the wire).
        let mut w = BitWriter::new();
        w.write_u32(7, 4); // ecplbegf
        w.write_u32(15, 4); // ecplendf
        w.write_u32(0, 1); // ecplbndstrce = 0
        let bytes = w.finish();

        let mut br = BitReader::new(&bytes);
        let strat = parse_strategy(&mut br, false, 0, &DEFAULT_ECPL_BNDSTRC).unwrap();
        assert_eq!(strat.begin_subbnd, 9);
        assert_eq!(strat.end_subbnd, 22);
        assert_eq!(strat.bndstrc, DEFAULT_ECPL_BNDSTRC);
        // begin=9, end=22, default banding → 4 bands (see geometry test).
        assert_eq!(strat.necplbnd, 4);
        assert_eq!(strat.begin_bin(), 97);
        assert_eq!(strat.end_bin(), 253);
        // Exactly 9 bits consumed (4 + 4 + 1).
        assert_eq!(br.bit_position(), 9);
    }

    #[test]
    fn parse_strategy_spx_active_skips_ecplendf() {
        // SPX on, spxbegf = 5 → end = spxbegf + 5 = 10; ecplendf is NOT
        // transmitted. ecplbegf = 5 → begin = 7. ecplbndstrce = 0.
        let mut w = BitWriter::new();
        w.write_u32(5, 4); // ecplbegf
        w.write_u32(0, 1); // ecplbndstrce = 0 (no ecplendf field)
        let bytes = w.finish();

        let mut br = BitReader::new(&bytes);
        let strat = parse_strategy(&mut br, true, 5, &DEFAULT_ECPL_BNDSTRC).unwrap();
        assert_eq!(strat.begin_subbnd, 7);
        assert_eq!(strat.end_subbnd, 10);
        // Only 5 bits consumed because ecplendf is omitted under SPX.
        assert_eq!(br.bit_position(), 5);
    }

    #[test]
    fn parse_strategy_explicit_banding_bits() {
        // ecplbegf = 7 → begin = 9; ecplendf = 15 → end = 22.
        // ecplbndstrce = 1 → merge bits for sbnd in [max(9,10), 22) =
        // [10, 22): that's 12 bits. Set every other bit so we can verify
        // placement: 10=1, 11=0, 12=1, ... merge bits transmitted from
        // sbnd 10 upward. The first active sub-band (9) is never a merge
        // bit (not transmitted) and stays false.
        let mut w = BitWriter::new();
        w.write_u32(7, 4); // ecplbegf
        w.write_u32(15, 4); // ecplendf
        w.write_u32(1, 1); // ecplbndstrce = 1
        let pattern = [
            true, false, true, false, true, false, true, false, true, false, true, false,
        ];
        for &b in &pattern {
            w.write_u32(b as u32, 1);
        }
        let bytes = w.finish();

        let mut br = BitReader::new(&bytes);
        let strat = parse_strategy(&mut br, false, 0, &DEFAULT_ECPL_BNDSTRC).unwrap();
        // Sub-band 9 is never transmitted → false.
        assert!(!strat.bndstrc[9]);
        // Sub-bands 10..22 match the pattern.
        for (i, &b) in pattern.iter().enumerate() {
            assert_eq!(strat.bndstrc[10 + i], b, "sbnd {}", 10 + i);
        }
        // 6 merges in the pattern → 13 sub-bands − 6 = 7 bands.
        assert_eq!(strat.necplbnd, 13 - 6);
        // 4 + 4 + 1 + 12 = 21 bits.
        assert_eq!(br.bit_position(), 21);
    }

    // ---- §E.2.3.3.20-26 coordinate parse ----

    #[test]
    fn parse_coords_first_block_implicit_present() {
        // 2/0: both channels in coupling, both firstcplcos (first block).
        // necplbnd = 2 for a compact test. ch0 = first coupled channel:
        // param1e implicit 1 (amps follow), param2e = 0 (angle/chaos fixed
        // to 0, not sent), no ecpltrans. ch1: param1e + param2e implicit
        // 1, then ecpltrans (1 bit).
        let necplbnd = 2usize;
        let mut w = BitWriter::new();
        w.write_u32(1, 1); // ecplangleintrp = 1
                           // ch0: implicit param1e=1 → 2 amps (5 bits each).
        w.write_u32(3, 5);
        w.write_u32(7, 5);
        // ch1: implicit param1e=1 → 2 amps; implicit param2e=1 → 2×(angle
        // 6 + chaos 3); ecpltrans = 1.
        w.write_u32(11, 5);
        w.write_u32(15, 5);
        w.write_u32(20, 6);
        w.write_u32(4, 3);
        w.write_u32(33, 6);
        w.write_u32(2, 3);
        w.write_u32(1, 1); // ecpltrans[1]
        let bytes = w.finish();

        let mut br = BitReader::new(&bytes);
        let chincpl = [true, true];
        let mut firstcplcos = [true, true];
        let c = parse_coords(&mut br, 2, &chincpl, &mut firstcplcos, necplbnd).unwrap();
        assert!(c.angleintrp);
        // firstcplcos cleared for both.
        assert_eq!(firstcplcos, [false, false]);

        // ch0 (first coupled): param1e, no param2e, no trans.
        assert!(c.channels[0].param1e);
        assert!(!c.channels[0].param2e);
        assert_eq!(c.channels[0].amp, vec![3, 7]);
        assert!(c.channels[0].angle.is_empty());
        assert!(c.channels[0].chaos.is_empty());
        assert!(!c.channels[0].trans);

        // ch1: param1e + param2e + trans.
        assert!(c.channels[1].param1e);
        assert!(c.channels[1].param2e);
        assert_eq!(c.channels[1].amp, vec![11, 15]);
        assert_eq!(c.channels[1].angle, vec![20, 33]);
        assert_eq!(c.channels[1].chaos, vec![4, 2]);
        assert!(c.channels[1].trans);

        // 1 + (2×5) + (2×5 + 2×(6+3) + 1) = 1 + 10 + 29 = 40 bits.
        assert_eq!(br.bit_position(), 40);
    }

    #[test]
    fn parse_coords_later_block_explicit_exist_bits() {
        // Later block (firstcplcos already cleared): exist bits are read
        // from the wire. ch0 first coupled: param1e bit only (no param2e,
        // no trans). ch1: param1e + param2e bits + trans.
        let necplbnd = 1usize;
        let mut w = BitWriter::new();
        w.write_u32(0, 1); // ecplangleintrp = 0
                           // ch0: param1e = 1 → 1 amp.
        w.write_u32(1, 1); // param1e
        w.write_u32(9, 5); // amp
                           // ch1: param1e = 0 (reuse), param2e = 1 → angle+chaos; trans = 0.
        w.write_u32(0, 1); // param1e
        w.write_u32(1, 1); // param2e
        w.write_u32(42, 6); // angle
        w.write_u32(5, 3); // chaos
        w.write_u32(0, 1); // ecpltrans[1]
        let bytes = w.finish();

        let mut br = BitReader::new(&bytes);
        let chincpl = [true, true];
        let mut firstcplcos = [false, false];
        let c = parse_coords(&mut br, 2, &chincpl, &mut firstcplcos, necplbnd).unwrap();
        assert!(!c.angleintrp);
        assert!(c.channels[0].param1e);
        assert!(!c.channels[0].param2e);
        assert_eq!(c.channels[0].amp, vec![9]);
        assert!(!c.channels[1].param1e);
        assert!(c.channels[1].param2e);
        assert!(c.channels[1].amp.is_empty());
        assert_eq!(c.channels[1].angle, vec![42]);
        assert_eq!(c.channels[1].chaos, vec![5]);
        assert!(!c.channels[1].trans);
        // 1 + (1 + 5) + (1 + 1 + 6 + 3 + 1) = 1 + 6 + 12 = 19 bits.
        assert_eq!(br.bit_position(), 19);
    }

    #[test]
    fn parse_coords_channel_not_in_coupling_rearms_marker() {
        // 3/0: ch1 not in coupling. firstcplcos[1] must be re-armed to
        // true; its params stay default. ch0 + ch2 are coupled.
        let necplbnd = 1usize;
        let mut w = BitWriter::new();
        w.write_u32(0, 1); // ecplangleintrp
                           // ch0 (first coupled, first block): implicit param1e → 1 amp.
        w.write_u32(8, 5);
        // ch1 skipped (not in coupling).
        // ch2 (second coupled, first block): implicit param1e + param2e +
        // trans.
        w.write_u32(12, 5); // amp
        w.write_u32(30, 6); // angle
        w.write_u32(6, 3); // chaos
        w.write_u32(0, 1); // trans
        let bytes = w.finish();

        let mut br = BitReader::new(&bytes);
        let chincpl = [true, false, true];
        let mut firstcplcos = [true, true, true];
        let c = parse_coords(&mut br, 3, &chincpl, &mut firstcplcos, necplbnd).unwrap();
        // ch1 re-armed, ch0 + ch2 cleared.
        assert_eq!(firstcplcos, [false, true, false]);
        assert_eq!(c.channels[0].amp, vec![8]);
        assert_eq!(c.channels[1], EcplChannelParams::default());
        assert_eq!(c.channels[2].amp, vec![12]);
        assert_eq!(c.channels[2].angle, vec![30]);
        // ch2 is the *second* coupled channel → carries param2e + trans.
        assert!(c.channels[2].param2e);
        // 1 + 5 + (5 + 6 + 3 + 1) = 21 bits.
        assert_eq!(br.bit_position(), 21);
    }

    // ---- §E.3.5.5.2 / §E.3.5.5.3 parameter processing ----

    fn db(linear: f32) -> f32 {
        20.0 * linear.log10()
    }

    #[test]
    fn ampbnd_endpoints_table_e3_10() {
        // Index 0: mant 0x20 (=32) / 32 = 1.0, exp 0 → 0 dB.
        assert!((ampbnd(0) - 1.0).abs() < 1e-6);
        assert!(db(ampbnd(0)).abs() < 1e-4);
        // Index 31: minus-infinity dB → amplitude 0.
        assert_eq!(ampbnd(31), 0.0);
        // Index 30: mant 0x17 (=23)/32 = 0.71875, exp 7 → /128 ≈ 0.005615
        // → ≈ -45.01 dB (the spec's documented lowest finite gain).
        let a30 = ampbnd(30);
        assert!((a30 - (23.0 / 32.0) / 128.0).abs() < 1e-7);
        assert!((db(a30) - (-45.01)).abs() < 0.05);
        // Monotone non-increasing across the finite range 0..=30.
        for i in 1..=30u8 {
            assert!(
                ampbnd(i) <= ampbnd(i - 1) + 1e-7,
                "amp[{i}] not <= amp[{}]",
                i - 1
            );
        }
        // Each exponent step (every 4 indices) approximately halves the
        // mantissa-1.0 anchor: index 4 (mant 0x10/32 = 0.5, exp 0) and
        // index 8 (mant 0x10/32, exp 1 → 0.25).
        assert!((ampbnd(4) - 0.5).abs() < 1e-6);
        assert!((ampbnd(8) - 0.25).abs() < 1e-6);
    }

    #[test]
    fn angle_and_chaos_tables() {
        // Table E3.11: code 0 → 0.0; code 32 → -1.0; code 63 → -0.03125.
        assert_eq!(ECPL_ANGLE_TAB[0], 0.0);
        assert!((ECPL_ANGLE_TAB[31] - 0.96875).abs() < 1e-6);
        assert!((ECPL_ANGLE_TAB[32] - (-1.0)).abs() < 1e-6);
        assert!((ECPL_ANGLE_TAB[63] - (-0.03125)).abs() < 1e-6);
        // Table E3.12: eighths -k/7.
        assert_eq!(ECPL_CHAOS_TAB[0], 0.0);
        assert!((ECPL_CHAOS_TAB[7] - (-1.0)).abs() < 1e-6);
        assert!((ECPL_CHAOS_TAB[3] - (-3.0 / 7.0)).abs() < 1e-5);
        // First-coupled channel forces angle/chaos to 0 regardless of code.
        assert_eq!(angle_value(40, true), 0.0);
        assert_eq!(chaos_value(5, true), 0.0);
        assert!((angle_value(40, false) - (-0.75)).abs() < 1e-6);
    }

    #[test]
    fn chaos_modification_reduces_amplitude() {
        // A non-first, non-transient channel with chaos applied: the
        // §E.3.5.5.2 modification `*= 1 + 0.38*chaos` with chaos <= 0
        // reduces the gain.
        let params = EcplChannelParams {
            param1e: true,
            param2e: true,
            amp: vec![0, 4],   // gains 1.0, 0.5
            chaos: vec![7, 0], // chaos -1.0, 0.0
            angle: vec![0, 0],
            trans: false,
        };
        let amps = process_band_amplitudes(&params, false);
        // band 0: 1.0 * (1 + 0.38 * -1.0) = 0.62.
        assert!((amps[0] - 0.62).abs() < 1e-6);
        // band 1: 0.5 * (1 + 0.38 * 0.0) = 0.5.
        assert!((amps[1] - 0.5).abs() < 1e-6);

        // Same params but transient present → no chaos modification.
        let mut tp = params.clone();
        tp.trans = true;
        let amps_t = process_band_amplitudes(&tp, false);
        assert!((amps_t[0] - 1.0).abs() < 1e-6);

        // First coupled channel → no chaos modification even without
        // transient (its chaos is fixed to 0).
        let amps_first = process_band_amplitudes(&params, true);
        assert!((amps_first[0] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn expand_bands_to_bins_fans_out_per_subband() {
        // begin sub-band 0, end sub-band 2: sub-bands 0 (6 bins) and 1
        // (6 bins). No merge → two bands of 6 bins each. begin_bin = 13,
        // end_bin = ECPL_SUBBND_TAB[2] = 25 → 12 bins total.
        let bndstrc = [false; N_ECPL_SUBBND];
        let out = expand_bands_to_bins(0, 2, &bndstrc, &[2.0, 5.0]);
        assert_eq!(out.len(), 12);
        assert!(out[..6].iter().all(|&v| (v - 2.0).abs() < 1e-9));
        assert!(out[6..].iter().all(|&v| (v - 5.0).abs() < 1e-9));

        // Merge sub-band 1 into 0 → single band of 12 bins, one value.
        let mut merged = [false; N_ECPL_SUBBND];
        merged[1] = true;
        let out2 = expand_bands_to_bins(0, 2, &merged, &[3.0]);
        assert_eq!(out2.len(), 12);
        assert!(out2.iter().all(|&v| (v - 3.0).abs() < 1e-9));
    }

    #[test]
    fn interpolate_single_band_is_constant() {
        // One band → angle fanned across all bins, no slope.
        let out = interpolate_bin_angles(&[0.25], &[6]);
        assert_eq!(out.len(), 6);
        assert!(out.iter().all(|&v| (v - 0.25).abs() < 1e-9));
    }

    #[test]
    fn interpolate_two_bands_wraps_and_fills() {
        // Two equal-size bands; verify output length == sum of bin counts
        // and every angle stays within the wrapped -1.0 ..= 1.0 interval.
        let band_angles = [0.0, 0.5];
        let nbins = [6usize, 6usize];
        let out = interpolate_bin_angles(&band_angles, &nbins);
        assert_eq!(out.len(), 12);
        for &v in &out {
            assert!((-1.0..=1.0).contains(&v), "angle {v} out of range");
        }
        // The interpolation walks from below the first band centre up
        // through the second; values should be (weakly) increasing in the
        // interior where no wrap occurs.
        // First band centre region near 0.0, last band region near 0.5.
        assert!(out[0] < out[out.len() - 1] + 1e-6);
    }

    #[test]
    fn interpolate_wrap_across_pi() {
        // angle_prev = 0.9, angle_curr = -0.9: the raw difference is
        // -1.8 but the spec unwraps (-0.9 + 2.0 = 1.1) so the slope is
        // small + positive, crossing the +1/-1 boundary which the wrap
        // guards fold back. Output must stay in range.
        let out = interpolate_bin_angles(&[0.9, -0.9], &[6, 6]);
        assert_eq!(out.len(), 12);
        for &v in &out {
            assert!((-1.0..=1.0).contains(&v), "wrapped angle {v} out of range");
        }
    }
}
