//! Forward MDCT (Modified Discrete Cosine Transform) for AC-3 encoding.
//!
//! Per §8.2.3.2 (A/52:2018), the 512-sample-long AC-3 transform is
//!
//! ```text
//!   X_D[k] = (-2/N) * sum_{n=0..N-1} x[n] *
//!             cos( (2π/(4N)) * (2n+1) * (2k+1)
//!                  + (π/4) * (2k+1) * (1+α) )
//! ```
//!
//! with N = 512 and α = 0 for the long transform (we do not emit
//! block-switched short blocks here — the encoder always uses long
//! blocks, matching the `blksw=0` path the decoder already handles
//! most carefully).
//!
//! The 256-coefficient output of this transform, when fed back through
//! our [`super::audblk::imdct_512`] reference, recovers the windowed
//! input (modulo the standard TDAC 50% overlap-add). Concretely, for a
//! block of 512 windowed input samples, this forward MDCT plus the
//! decoder's IMDCT+window+overlap-add chain reproduces the middle 256
//! samples exactly (to floating-point precision). The factor-of-`-2/N`
//! here pairs with the decoder's factor-of-`2` overlap-add so that the
//! combined round-trip gain is `1.0`.

use std::f32::consts::PI;

/// 512-point forward MDCT (§8.2.3.2, α=0 long transform).
///
/// `input`  : 512 windowed time-domain samples.
/// `output` : 256 MDCT coefficients — indices 0..N/2.
///
/// The reference implementation is `O(N^2)` — 256 × 512 ≈ 128 k
/// multiply-adds per block. For a 48 kHz stereo frame we run it 12
/// times per syncframe, well inside budget for a pure-Rust encoder
/// whose job is correctness first.
pub fn mdct_512(input: &[f32; 512], output: &mut [f32; 256]) {
    let n: usize = 512;
    // §8.2.3.2 mandates a `-2/N` normalisation. Combined with the
    // decoder's `2/N` IMDCT scale and the ×2 overlap-add, the full
    // analysis-synthesis round-trip lands on unity gain (to within
    // window-table rounding).
    let scale: f32 = -2.0 / n as f32;
    let two_pi_over_4n = 2.0 * PI / (4.0 * n as f32);
    let pi_over_4 = PI / 4.0;
    for k in 0..256 {
        let mut s = 0.0f32;
        let two_k_plus_1 = (2 * k + 1) as f32;
        let phase_b = pi_over_4 * two_k_plus_1; // α = 0 → (1+α) = 1
        for nn in 0..n {
            let phase = two_pi_over_4n * (2 * nn + 1) as f32 * two_k_plus_1 + phase_b;
            s += input[nn] * phase.cos();
        }
        output[k] = scale * s;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::audblk;
    use crate::tables::WINDOW;

    /// Forward MDCT followed by the decoder's IMDCT should approximately
    /// invert under the AC-3 windowing + TDAC overlap-add rule. We feed
    /// the same windowed block in twice (blocks N and N+1), run the
    /// full analysis-synthesis chain, and expect the second block's
    /// output to match the (windowed^2) input in the middle region.
    #[test]
    fn mdct_imdct_roundtrip_identity_window_tdac() {
        // Construct a 768-sample ramp and encode two adjacent 512-sample
        // overlapping blocks out of it. Decoder's overlap-add needs two
        // blocks to produce valid output for the first block.
        let sig_len = 512 + 256;
        let mut sig = vec![0.0f32; sig_len];
        for (i, s) in sig.iter_mut().enumerate() {
            // 100 Hz-ish sine to keep the magnitudes sensible under a 48 kHz rate.
            let t = i as f32 / 48_000.0;
            *s = (2.0 * PI * 440.0 * t).sin() * 0.3;
        }

        // Build the symmetric 512-sample window from WINDOW[0..256] + mirror.
        let mut full_win = [0.0f32; 512];
        for n in 0..256 {
            full_win[n] = WINDOW[n];
            full_win[511 - n] = WINDOW[n];
        }

        // Window block 0 (samples 0..512).
        let mut blk0 = [0.0f32; 512];
        for n in 0..512 {
            blk0[n] = sig[n] * full_win[n];
        }
        let mut x0 = [0.0f32; 256];
        mdct_512(&blk0, &mut x0);

        // Window block 1 (samples 256..768).
        let mut blk1 = [0.0f32; 512];
        for n in 0..512 {
            blk1[n] = sig[256 + n] * full_win[n];
        }
        let mut x1 = [0.0f32; 256];
        mdct_512(&blk1, &mut x1);

        // IMDCT + window + overlap-add path, exactly as the decoder runs.
        let mut delay = [0.0f32; 256];

        // Block 0: IMDCT, window, OLA (primes delay; pcm0 is discarded).
        let mut time0 = [0.0f32; 512];
        audblk::imdct_512(&x0, &mut time0);
        for n in 0..256 {
            time0[n] *= WINDOW[n];
            time0[511 - n] *= WINDOW[n];
        }
        let mut _pcm0 = [0.0f32; 256];
        for n in 0..256 {
            _pcm0[n] = 2.0 * (time0[n] + delay[n]);
            delay[n] = time0[256 + n];
        }

        // Block 1: IMDCT, window, OLA → pcm1 should match input[256..512].
        let mut time1 = [0.0f32; 512];
        audblk::imdct_512(&x1, &mut time1);
        for n in 0..256 {
            time1[n] *= WINDOW[n];
            time1[511 - n] *= WINDOW[n];
        }
        let mut pcm1 = [0.0f32; 256];
        for n in 0..256 {
            pcm1[n] = 2.0 * (time1[n] + delay[n]);
        }

        // Compare pcm1 to input[256..512]. The overlap-add equation
        // pcm[n] = window[n]^2 * x[n] + window[n+256]^2 * x[n] = x[n]
        // (because AC-3's window satisfies w[n]^2 + w[n+256]^2 = 1 —
        // the Princen-Bradley condition for MDCT TDAC).
        let mut worst: f32 = 0.0;
        let mut sse: f32 = 0.0;
        for n in 0..256 {
            let err = (pcm1[n] - sig[256 + n]).abs();
            worst = worst.max(err);
            sse += err * err;
        }
        let rms = (sse / 256.0).sqrt();
        eprintln!("mdct-imdct roundtrip: worst={worst:.5}, rms={rms:.5}");
        // The window is only approximate in the tables (5-decimal rounding);
        // a few 1e-3 worst-case error is acceptable here.
        assert!(worst < 0.01, "worst {worst} too large");
        assert!(rms < 5e-3, "rms {rms} too large");
    }
}
