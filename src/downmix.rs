//! AC-3 channel-layout downmix (§7.8).
//!
//! Maps an arbitrary source `acmod` channel layout onto a 2- or 1-channel
//! output using the §7.8.1 / §7.8.2 matrix equations. The per-channel
//! weights come from §5.4.2.4 `cmixlev` / §5.4.2.5 `surmixlev` (Tables
//! 5.9 / 5.10), remapped to linear gains via the existing
//! [`crate::tables::CENTER_MIX_LEVEL`] / [`SURROUND_MIX_LEVEL`] LUTs.
//!
//! ## Scope
//!
//! - **Target layouts:** 2-channel `LoRo` stereo and 1-channel mono.
//!   `LtRt` (Dolby Surround matrix) is not yet implemented — the
//!   decoder advertises `LoRo` which spec §7.8.2 calls the "preferred"
//!   downmix when the ultimate target is mono anyway.
//! - **Source layouts:** every `acmod` ≥ 1 (1/0, 2/0, 3/0, 2/1, 3/1,
//!   2/2, 3/2). `acmod = 0` (dual-mono 1+1) is handled by routing Ch1
//!   into the Left output and Ch2 into the Right, which is the "Stereo"
//!   dualmode path from §7.8.1.
//! - **LFE:** always dropped. §7.8 allows any coefficient; adding LFE
//!   to the stereo sum is risky (speakers crossed-over to a sub bus
//!   will double-tap the bass) so we do what liba52 and ffmpeg's
//!   `ac3_downmix` default both do and route LFE to nothing.
//! - **Overload scaling:** §7.8.2 mandates attenuating the matrix so
//!   `sum-of-coefficients ≤ 1`. We compute the per-output sum exactly
//!   and divide — for stereo 3/2 this lands at the spec's 0.4143 worst
//!   case; for narrower layouts the scaling is looser, preserving
//!   envelope.
//!
//! Construction is cheap (just a 5×2 matrix of `f32`) so callers can
//! cache a `Downmix` across syncframes or build one per block.

use crate::bsi::Bsi;
use crate::tables::{CENTER_MIX_LEVEL, SURROUND_MIX_LEVEL};

/// Output layout requested from the decoder.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DownmixMode {
    /// Leave the source channels untouched.
    Passthrough,
    /// Mix every source channel into a 2-channel LoRo pair.
    Stereo,
    /// Mix every source channel into a single mono channel.
    Mono,
}

impl DownmixMode {
    /// Resolve from a user-requested output channel count (`None`
    /// meaning "pass through"). A requested count that matches the
    /// source `nfchans` also becomes `Passthrough`, even when LFE is
    /// on — AC-3 never downmixes LFE explicitly.
    pub fn resolve(requested: Option<u16>, source_nfchans: u8) -> Self {
        match requested {
            None => Self::Passthrough,
            Some(1) => Self::Mono,
            Some(2) if source_nfchans > 2 || source_nfchans == 0 => Self::Stereo,
            Some(2) if source_nfchans == 2 => Self::Passthrough,
            Some(2) if source_nfchans == 1 => Self::Stereo,
            _ => Self::Passthrough,
        }
    }
}

/// Pre-computed per-channel coefficients for one syncframe. The source
/// layout slot order matches `acmod`'s Table 5.8 ordering
/// `[L, C, R, Ls/S, Rs]`, with missing channels having a zero weight.
///
/// Each `DownmixGains` row lists the weight applied to *that source slot*
/// when summed into an output channel. For stereo the two rows are
/// Left-output-coeffs and Right-output-coeffs. For mono there is one
/// row (Center-output-coeffs).
#[derive(Clone, Debug)]
pub struct Downmix {
    mode: DownmixMode,
    /// `out_coeffs[out_ch][src_slot]` — up to 5 slots per output.
    out_coeffs: [[f32; 5]; 2],
    /// Number of active output channels (1 or 2).
    out_channels: u8,
    /// Source acmod for spec diagnostics.
    src_acmod: u8,
    /// Source nfchans for arithmetic in `apply`.
    src_nfchans: u8,
    /// Whether LFE exists on the source (never mixed in).
    src_lfe: bool,
}

impl Downmix {
    /// Build a downmix matrix from BSI state and a target mode. When
    /// `mode` is [`DownmixMode::Passthrough`] the returned `Downmix`
    /// still records the source layout but `out_channels` is set to
    /// `nfchans + lfe`; [`Downmix::apply`] short-circuits.
    pub fn from_bsi(bsi: &Bsi, mode: DownmixMode) -> Self {
        let mut out = Self {
            mode,
            out_coeffs: [[0.0; 5]; 2],
            out_channels: bsi.nchans,
            src_acmod: bsi.acmod,
            src_nfchans: bsi.nfchans,
            src_lfe: bsi.lfeon,
        };
        if matches!(mode, DownmixMode::Passthrough) {
            return out;
        }

        // §5.4.2.4 / §5.4.2.5 — reserved code 0b11 maps to the
        // "intermediate" coefficient per spec. Our `CENTER_MIX_LEVEL`
        // table already repeats the middle value at index 3 so the
        // reserved code resolves to 0.595 / 0.500.
        let clev = if bsi.cmixlev == 0xFF {
            0.707
        } else {
            CENTER_MIX_LEVEL[(bsi.cmixlev & 0x3) as usize]
        };
        let slev = if bsi.surmixlev == 0xFF {
            0.707
        } else {
            SURROUND_MIX_LEVEL[(bsi.surmixlev & 0x3) as usize]
        };

        match mode {
            DownmixMode::Stereo => Self::fill_stereo(&mut out, bsi.acmod, clev, slev),
            DownmixMode::Mono => Self::fill_mono(&mut out, bsi.acmod, clev, slev),
            DownmixMode::Passthrough => unreachable!(),
        }
        out
    }

    /// LoRo 2-channel downmix per §7.8.2. Source slots are indexed as
    /// `L=0, C=1, R=2, Ls/S=3, Rs=4` per Table 5.8; missing channels
    /// leave their coefficient at zero. Output slot 0 = Lo, slot 1 = Ro.
    fn fill_stereo(out: &mut Self, acmod: u8, clev: f32, slev: f32) {
        // Start with the §7.8.2 3/2 LoRo equations:
        //     Lo = 1·L + clev·C + slev·Ls
        //     Ro = 1·R + clev·C + slev·Rs
        // with Table 5.8 dropping channels as the acmod narrows.

        // Source-channel presence per Table 5.8.
        // acmod 0 (1+1 dual mono) is handled separately below — for all
        // other acmods the bit pattern is:
        //   bit 0 (center): acmod == 1, 3, 5, 7
        //   bit 2 (surround): acmod >= 4
        //   two surround channels: acmod == 6, 7
        //   two front (L/R): acmod != 1 (i.e. acmod != 1/0 mono)
        match acmod {
            0 => {
                // 1+1 dual mono — §7.8.1 'dualmode == Stereo' path:
                // Ch1 → Lo, Ch2 → Ro. Slot layout [Ch1, _, Ch2, _, _]:
                // Ch1 at slot 0, Ch2 at slot 2.
                out.out_coeffs[0][0] = 1.0;
                out.out_coeffs[1][2] = 1.0;
            }
            1 => {
                // 1/0 — pure center → both outputs get -3 dB of C.
                // §7.8.1 `output_nfront == 2` path with `input_nfront==1`:
                //   mix center into left with –3 dB
                //   mix center into right with –3 dB
                out.out_coeffs[0][0] = 0.707;
                out.out_coeffs[1][0] = 0.707;
            }
            2 => {
                // 2/0 — pass through. L at slot 0, R at slot 2 per
                // the [L, C, R, Ls/S, Rs] layout.
                out.out_coeffs[0][0] = 1.0;
                out.out_coeffs[1][2] = 1.0;
            }
            3 => {
                // 3/0 — L, C, R into Lo, Ro using clev.
                out.out_coeffs[0][0] = 1.0;
                out.out_coeffs[0][1] = clev;
                out.out_coeffs[1][1] = clev;
                out.out_coeffs[1][2] = 1.0;
            }
            4 => {
                // 2/1 — L, R, S. Single surround is folded into both
                // outputs with slev and, per spec "0.7 * slev" factor
                // for single-surround LoRo.
                out.out_coeffs[0][0] = 1.0;
                out.out_coeffs[0][3] = 0.7 * slev;
                out.out_coeffs[1][2] = 1.0;
                out.out_coeffs[1][3] = 0.7 * slev;
            }
            5 => {
                // 3/1 — L, C, R, S.
                out.out_coeffs[0][0] = 1.0;
                out.out_coeffs[0][1] = clev;
                out.out_coeffs[0][3] = 0.7 * slev;
                out.out_coeffs[1][1] = clev;
                out.out_coeffs[1][2] = 1.0;
                out.out_coeffs[1][3] = 0.7 * slev;
            }
            6 => {
                // 2/2 — L, R, Ls, Rs.
                out.out_coeffs[0][0] = 1.0;
                out.out_coeffs[0][3] = slev;
                out.out_coeffs[1][2] = 1.0;
                out.out_coeffs[1][4] = slev;
            }
            _ => {
                // 3/2 (acmod=7) and any defensive fall-through — full
                // L, C, R, Ls, Rs.
                out.out_coeffs[0][0] = 1.0;
                out.out_coeffs[0][1] = clev;
                out.out_coeffs[0][3] = slev;
                out.out_coeffs[1][1] = clev;
                out.out_coeffs[1][2] = 1.0;
                out.out_coeffs[1][4] = slev;
            }
        }

        out.out_channels = 2;
        // Normalise per §7.8.2: divide each row so its sum is ≤ 1.
        Self::normalise(&mut out.out_coeffs);
    }

    /// Mono downmix: derive from the stereo LoRo pair and sum to mono
    /// per §7.8.2 ("a simple summation of the 2 channels"). The spec
    /// also gives the explicit 3/2 mono formula:
    ///     M = L + 2·clev·C + R + slev·Ls + slev·Rs
    /// and the 3/1 form:
    ///     M = L + 2·clev·C + R + 1.4·slev·S
    /// which we build from scratch rather than composing stereo, so we
    /// can apply spec's "further scaling of 1/2" exactly once.
    fn fill_mono(out: &mut Self, acmod: u8, clev: f32, slev: f32) {
        // Slot layout [L, C, R, Ls/S, Rs].
        let (l, c, r, ls, rs) = match acmod {
            0 => {
                // 1+1 dual mono — sum Ch1 + Ch2 with -6 dB each per
                // §7.8.1 (dualmode=Stereo 1-front path).
                (0.5, 0.0, 0.5, 0.0, 0.0)
            }
            1 => (0.0, 1.0, 0.0, 0.0, 0.0),               // 1/0
            2 => (1.0, 0.0, 1.0, 0.0, 0.0),               // 2/0
            3 => (1.0, 2.0 * clev, 1.0, 0.0, 0.0),        // 3/0
            4 => (1.0, 0.0, 1.0, 1.4 * slev, 0.0),        // 2/1
            5 => (1.0, 2.0 * clev, 1.0, 1.4 * slev, 0.0), // 3/1
            6 => (1.0, 0.0, 1.0, slev, slev),             // 2/2
            _ => (1.0, 2.0 * clev, 1.0, slev, slev),      // 3/2 (acmod=7)
        };
        out.out_coeffs[0][0] = l;
        out.out_coeffs[0][1] = c;
        out.out_coeffs[0][2] = r;
        out.out_coeffs[0][3] = ls;
        out.out_coeffs[0][4] = rs;
        out.out_channels = 1;

        // Normalise so the sum of coefficients is ≤ 1 (§7.8.2 overload
        // guard — effectively the "further scaling of 1/2" for mono
        // when the stereo sum already saturates).
        let sum: f32 = out.out_coeffs[0].iter().sum();
        if sum > 1.0 {
            let k = 1.0 / sum;
            for v in out.out_coeffs[0].iter_mut() {
                *v *= k;
            }
        }
    }

    /// Normalise each output row so `sum(|coeff|) ≤ 1`. §7.8.2 calls
    /// this "attenuating all downmix coefficients equally". For the 3/2
    /// LoRo case with default clev=slev=0.707 this matches the spec's
    /// worst-case 0.4143 factor (1 / 2.414) to 3-sig-fig.
    fn normalise(rows: &mut [[f32; 5]; 2]) {
        for row in rows.iter_mut() {
            let sum: f32 = row.iter().map(|c| c.abs()).sum();
            if sum > 1.0 {
                let k = 1.0 / sum;
                for v in row.iter_mut() {
                    *v *= k;
                }
            }
        }
    }

    /// How many channels the downmix produces, including LFE if any.
    /// Passthrough returns the source count.
    pub fn output_channels(&self) -> u8 {
        match self.mode {
            DownmixMode::Passthrough => self.src_nfchans + u8::from(self.src_lfe),
            _ => self.out_channels,
        }
    }

    /// Whether this downmix will touch the samples at all.
    pub fn is_passthrough(&self) -> bool {
        matches!(self.mode, DownmixMode::Passthrough)
    }

    /// Apply this downmix to one block of per-source-channel PCM.
    ///
    /// `src` is indexed as `src[ch][n]` with `ch` in the decoder's
    /// internal fbw order (Table 5.8) — so for 3/2 the channels are
    /// `[L, C, R, Ls, Rs]`. `lfe` is the optional 7th channel;
    /// currently ignored.
    ///
    /// Writes `nsamples × output_channels()` samples into `dst` in
    /// channel-interleaved layout. `dst.len()` must be at least
    /// `nsamples × output_channels()` — extra trailing space is left
    /// untouched.
    pub fn apply(&self, src: &[[f32; 256]; 5], nsamples: usize, dst: &mut [f32]) {
        debug_assert!(self.mode != DownmixMode::Passthrough);
        // Map the decoder's fbw channel index to our [L, C, R, Ls/S, Rs]
        // slot layout. For 1+1 dual-mono Ch1 and Ch2 live at fbw 0 / 1
        // — we alias them to slots L and R since the matrix was built
        // with L/R gains for them in `fill_stereo`/`fill_mono`. For all
        // other acmods we follow Table 5.8 directly.
        let slot_of: [Option<usize>; 5] = match self.src_acmod {
            0 => [Some(0), None, Some(1), None, None], // Ch1, Ch2 → L, R
            1 => [Some(0), None, None, None, None],    // C only at fbw 0 → slot 1? see below
            2 => [Some(0), None, Some(1), None, None], // L, R
            3 => [Some(0), Some(1), Some(2), None, None], // L, C, R
            4 => [Some(0), None, Some(1), Some(2), None], // L, R, S
            5 => [Some(0), Some(1), Some(2), Some(3), None], // L, C, R, S
            6 => [Some(0), None, Some(1), Some(2), Some(3)], // L, R, Ls, Rs
            _ => [Some(0), Some(1), Some(2), Some(3), Some(4)], // 3/2 (acmod=7)
        };
        // Special case acmod=1 (1/0): our single source channel is the
        // center, so its matrix weights sit in slot 1. Rebuild the
        // mapping with slot 1 pointing to fbw 0 and slot 0 nothing.
        let slot_of = if self.src_acmod == 1 {
            [None, Some(0), None, None, None]
        } else {
            slot_of
        };

        let nch = self.out_channels as usize;
        for n in 0..nsamples {
            for out_ch in 0..nch {
                let coeffs = &self.out_coeffs[out_ch];
                let mut acc = 0.0f32;
                for (slot, fbw) in slot_of.iter().enumerate() {
                    let Some(fbw_idx) = *fbw else { continue };
                    let c = coeffs[slot];
                    if c == 0.0 {
                        continue;
                    }
                    acc += c * src[fbw_idx][n];
                }
                dst[n * nch + out_ch] = acc;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fake_bsi(acmod: u8, cmixlev: u8, surmixlev: u8, lfeon: bool) -> Bsi {
        let nfchans = crate::tables::acmod_nfchans(acmod);
        Bsi {
            bsid: 8,
            bsmod: 0,
            acmod,
            nfchans,
            lfeon,
            nchans: nfchans + u8::from(lfeon),
            dialnorm: 27,
            cmixlev,
            surmixlev,
            dsurmod: 0xFF,
            bits_consumed: 0,
        }
    }

    #[test]
    fn stereo_downmix_3_2_sum_bounded() {
        // acmod=7, default clev/slev.
        let bsi = fake_bsi(7, 0, 0, true);
        let d = Downmix::from_bsi(&bsi, DownmixMode::Stereo);
        // Both output rows must sum to ≤ 1.
        let sum_l: f32 = d.out_coeffs[0].iter().map(|c| c.abs()).sum();
        let sum_r: f32 = d.out_coeffs[1].iter().map(|c| c.abs()).sum();
        assert!(sum_l <= 1.0 + 1e-6);
        assert!(sum_r <= 1.0 + 1e-6);
        // §7.8.2 says the 3/2 LoRo worst-case scale is 1/2.414 ≈ 0.4143
        // when clev=slev=0.707. The unnormalised L-row is
        // [1, 0.707, 0, 0.707, 0] summing to 2.414, so after normalise
        // the L-coeff (originally 1.0) should read ≈ 0.4143.
        assert!((d.out_coeffs[0][0] - 0.4143).abs() < 1e-3);
    }

    #[test]
    fn stereo_downmix_2_0_is_identity() {
        let bsi = fake_bsi(2, 0xFF, 0xFF, false);
        let d = Downmix::from_bsi(&bsi, DownmixMode::Stereo);
        assert_eq!(d.output_channels(), 2);
        // Lo-row picks up source slot 0 (L); Ro-row picks up slot 2 (R).
        assert_eq!(d.out_coeffs[0][0], 1.0);
        assert_eq!(d.out_coeffs[1][2], 1.0);
        // Other slots zero on both rows.
        for slot in 1..5 {
            assert_eq!(d.out_coeffs[0][slot], 0.0);
        }
        for slot in [0, 1, 3, 4] {
            assert_eq!(d.out_coeffs[1][slot], 0.0);
        }
    }

    #[test]
    fn stereo_downmix_3_1() {
        // 3/1 with cmixlev=0 (0.707), surmixlev=0 (0.707).
        let bsi = fake_bsi(5, 0, 0, false);
        let d = Downmix::from_bsi(&bsi, DownmixMode::Stereo);
        assert_eq!(d.output_channels(), 2);
        // Each row should have L/R, C (clev) and S (0.7·slev) slots populated.
        // Pre-normalise the row is [1, 0.707, 0, 0.4949, 0] summing to 2.2019.
        let sum: f32 = d.out_coeffs[0].iter().map(|c| c.abs()).sum();
        assert!(sum <= 1.0 + 1e-6);
        // Centre weight is non-zero on both rows; surround weight too.
        assert!(d.out_coeffs[0][1] > 0.0);
        assert!(d.out_coeffs[1][1] > 0.0);
        assert!(d.out_coeffs[0][3] > 0.0);
        assert!(d.out_coeffs[1][3] > 0.0);
    }

    #[test]
    fn mono_from_3_2_has_all_sources() {
        let bsi = fake_bsi(7, 0, 0, false);
        let d = Downmix::from_bsi(&bsi, DownmixMode::Mono);
        assert_eq!(d.output_channels(), 1);
        // All five source slots must contribute.
        for slot in 0..5 {
            assert!(
                d.out_coeffs[0][slot] > 0.0,
                "mono slot {} should be >0",
                slot
            );
        }
        // Sum bounded.
        let sum: f32 = d.out_coeffs[0].iter().sum();
        assert!(sum <= 1.0 + 1e-6);
    }

    #[test]
    fn mono_from_mono_routes_center() {
        let bsi = fake_bsi(1, 0xFF, 0xFF, false);
        let d = Downmix::from_bsi(&bsi, DownmixMode::Mono);
        assert_eq!(d.output_channels(), 1);
        // Only the center slot carries weight.
        assert_eq!(d.out_coeffs[0][1], 1.0);
    }

    #[test]
    fn apply_stereo_passes_identity_for_2_0() {
        let bsi = fake_bsi(2, 0xFF, 0xFF, false);
        let d = Downmix::from_bsi(&bsi, DownmixMode::Stereo);
        let mut src: [[f32; 256]; 5] = [[0.0; 256]; 5];
        for n in 0..256 {
            src[0][n] = 0.5;
            src[1][n] = -0.25;
        }
        let mut out = vec![0.0f32; 256 * 2];
        d.apply(&src, 256, &mut out);
        for n in 0..256 {
            assert!((out[n * 2] - 0.5).abs() < 1e-6);
            assert!((out[n * 2 + 1] - -0.25).abs() < 1e-6);
        }
    }

    #[test]
    fn apply_stereo_on_3_2_full_scale_clamps() {
        // Hand a full-scale signal on every source channel; worst-case
        // normalisation must keep the output within ±1.
        let bsi = fake_bsi(7, 0, 0, false);
        let d = Downmix::from_bsi(&bsi, DownmixMode::Stereo);
        let mut src: [[f32; 256]; 5] = [[1.0; 256]; 5];
        // Make R (slot 2) negative so its path doesn't fully phase-cancel.
        for n in 0..256 {
            src[2][n] = 1.0;
        }
        let mut out = vec![0.0f32; 256 * 2];
        d.apply(&src, 256, &mut out);
        for n in 0..256 {
            assert!(out[n * 2].abs() <= 1.0 + 1e-6);
            assert!(out[n * 2 + 1].abs() <= 1.0 + 1e-6);
        }
    }

    #[test]
    fn resolve_common_cases() {
        assert_eq!(DownmixMode::resolve(None, 5), DownmixMode::Passthrough);
        assert_eq!(DownmixMode::resolve(Some(2), 5), DownmixMode::Stereo);
        assert_eq!(DownmixMode::resolve(Some(2), 2), DownmixMode::Passthrough);
        assert_eq!(DownmixMode::resolve(Some(1), 2), DownmixMode::Mono);
        assert_eq!(DownmixMode::resolve(Some(1), 5), DownmixMode::Mono);
        assert_eq!(DownmixMode::resolve(Some(6), 5), DownmixMode::Passthrough);
    }
}
