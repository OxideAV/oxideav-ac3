# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- §7.2.2.6 delta bit allocation: persistent per-channel + coupling deltba
  segment state on `Ac3State`, parsed per Table 5.16 (`new info / reuse /
  no delta` semantics) and applied to the masking curve before bap[]
  computation. Dormant on the current transient-burst fixture (the
  encoder there sets cpldeltbae and per-fbw deltbae[] to '10' = no
  delta) but required-by-spec for any stream that does signal mask
  offsets — without it our bap[] would diverge from the encoder's and
  desync mantissa unpacking.
- `examples/sample_compare.rs`: per-block PSNR + peak-diff diagnostic for
  drilling into burst frames where the per-frame floor hides which
  audblk is breaking. Used to characterise the round-7 drift pattern
  (errors ramp through frame 14 blk 0..5, peak in frame 15 blk 0-1,
  unwind through frame 15 blk 2-5).

### Fixed

- §7.5.2.2 / §7.5.2.3 rematrix band 3 upper bound: was hard-coded to
  bin 252 even when coupling was active, allowing the L+R / L-R operation
  to bleed into the just-decoupled coupling region. Now tracks
  `36 + 12*cplbegf` per Tables 7.26 / 7.27 when `cplinu == 1`. No PSNR
  movement on the current fixture (end_mant capped the bleed there
  anyway) but a latent correctness fix for streams where the per-channel
  bandwidth code reaches further.

### Investigation notes (transient fixture, round 7)

Root-causing the residual ≈15 dB transient PSNR floor — outcome: not
cracked this round, but the bug is now bracketed substantially tighter
than the round-6 notes left it.

Per-block sample_compare on frame 14 of the bursts fixture shows the
error grows monotonically through the frame (28, 22, 16, 11, 8, 6 dB
across blocks 0..5), peaks in frame 15 blk 0-1 (4.4 / 4.7 dB), then
unwinds symmetrically. Peak amplitudes: ours ≈ 0.36 × ffmpeg's on the
worst blocks — i.e. our reconstruction is roughly the right *shape*
but the wrong *magnitude*, with a partial sign inversion appearing
specifically on burst-peak blocks. Probes that did NOT move the floor:

- spec-literal vs. swapped §7.9.4.2 short2 de-interleave (round 6's
  deviation makes no PSNR difference on this fixture either way)
- disabling coupling decoupling entirely (`AC3_DISABLE_DECOUPLE=1`)
- disabling rematrix entirely (`AC3_DISABLE_REMAT=1`)
- the new dba application (encoder sends only "no delta" markers in
  this fixture's burst blocks)

These eliminate IMDCT short-block, coupling decode, rematrix matrix,
AND dba as the dominant error sources. What remains in the per-frame
error budget is bit allocation (bap[]) on the burst-onset blocks
themselves — specifically the masking-curve computation around bins
4-5 (the dominant 440 Hz tone) where calc_lowcomp's break condition
`bndpsd[bin] <= bndpsd[bin+1]` fires under burst conditions and the
spec-vs-implementation behaviour at that exact moment is hardest to
verify without a known-correct reference trace.

Next round should: (a) instrument bndpsd[0..7], excite[0..7], mask[0..7],
and bap[0..7] for ch0 in frame 14 blk 0 (28 dB — easiest to reverse-
engineer the divergence) and compare against a hand-calculated trace
from §7.2.2.4 pseudo-code with the same exponents we decoded; and
(b) consider whether the burst-frame bap[] divergence might come from
our exponent decode chain itself (D15 → cumulative sum of M-2 deltas)
when consecutive deltas straddle the 5-level map saturation boundary,
since the burst fixture is where extreme M=0 / M=4 codes appear.

## [0.0.2](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.1...v0.0.2) - 2026-04-25

### Fixed

- drop crc2 residue debug_assert (wrong invariant)

### Other

- drop oxideav-codec/oxideav-container shims, import from oxideav-core
- encoder quality lift — group-synced mantissa emit + per-block D15 refresh
- clippy sweep — allows for approx_constant + needless_range_loop
- clippy sweep in audblk — auto-fix + spec-faithful allows
- clippy sweep — unnecessary_cast, identity_op, needless_range_loop
- cargo fmt sweep (14 files)
- document round-6 transient-burst PSNR investigation
- fix §7.9.4.2 short2 IMDCT TDAC — restore antisymmetric upper half
- fix BAPTAB/MASKTAB/LATAB off-by-ones, add transient PSNR gate
- FFT-backed 512-pt + 256-pt IMDCT (§7.9.4) + PSNR gate
- §7.8 downmix matrix (3/2, 3/1, 2/1 → stereo/mono)
- parse_frame_side_info helper + fixture-backed §5.4.3 tests
- annotate audblk parser with §5.4.3 clause citations + side-info struct
- ac3 encoder: solve crc1 via GF(2) Gaussian elimination
- ac3 encoder: encode_sine + check_stream examples, preprocess comment
- short-block 256-point IMDCT + bap=0 dither
- ac3 encoder: register in CodecRegistry as make_encoder
- fix IMDCT scale + zero stale coeffs, RMS within 3% of ffmpeg
- IMDCT overlap-add applies spec factor-of-2 scaling
- add audio-block DSP pipeline (exp, bit-alloc, mantissa, IMDCT)
- add bit-allocation / mantissa / window / hth tables
- switch workflows to master branch

### Fixed

- `BAPTAB` (Table 7.16): off-by-one at positions 26 and 30 that was
  shifting every bap in the 23..=34 range by one, mis-quantizing
  mid-band mantissas. PSNR vs ffmpeg on the 440 Hz sine fixture
  jumps from 35.7 dB to 92.0 dB as a direct result.
- `MASKTAB` (Table 7.13): off-by-one at rows A=8, 9, 10, 15 that was
  mapping four high-frequency mantissa bins into the wrong 1/6 octave
  band, skewing bit-allocation masking on wide-bandwidth content.
- `LATAB` (Table 7.14) entry 151: `0x0002` → `0x0003` to match spec
  Table 7.14, row A=15 column B=1.

### Added

- `tests/fixtures/transient_bursts_stereo.ac3`: three Gaussian tone
  bursts, 62 short-block audblks across 63 frames, exercising the
  `blksw=1` IMDCT path that the sine fixture never touches.
- `decoder_matches_ffmpeg_on_transient_fixture` PSNR gate for the
  short-block / block-switching regression.
- `decoder_matches_ffmpeg_within_psnr_floor` gate ratcheted from
  25 dB to 80 dB now that the BAPTAB fix lands.
- `examples/count_blksw`, `examples/psnr_per_frame` diagnostic
  tools for inspecting short-block coverage and per-frame decode
  quality.

### Changed

- `audblk::imdct_256_pair` direct-form short-block IMDCT is now
  `#[cfg(test)]`-only. The §7.9.4.2 FFT decomposition
  (`imdct::imdct_256_pair_fft`) is canonical; the naive per-half
  reference disagrees with it (see the
  `short_block_direct_form_diverges_from_fft` regression test).

### Investigation notes (transient fixture drift)

Round 6: investigated the remaining 15.5 dB PSNR floor on the
transient-burst fixture (bursts at frames 14-17, 29-32, 45-48).
Outcome: no code change lands this round — the previously-suspected
`run_bit_allocation` divergence on bins 4-5 at burst onset was
traced line-by-line against ATSC A/52:2018 §7.2.2.4 and every
reachable value (`lowcomp`, `fastleak`, `slowleak`, `excite[]`,
`mask[]`, `bap[]`) agrees with the spec's pseudocode for the
`bndstrt==0` fbw path (including the break at bin=2 when
`bndpsd[2] <= bndpsd[3]` as bursts rise).

Remaining symptom on burst frames: the time-domain output at the
dominant 440 Hz MDCT bin emerges with roughly the wrong sign (our
output is ≈ −0.77 × reference across the burst). Localised probes
(zero bins 4-5, flip bin 4, flip bin 5, zero entire burst blocks,
disable dither, force long blocks) each shift burst-frame PSNR by
≤4 dB — i.e. the error is not concentrated in any single bin or
DSP stage that the probes touched. Rematrix flags are all zero in
this fixture and coupling coefficients (bins 133-216) are all
`exp=24` / `bap=0`, so neither pathway is contributing.

Further investigation should: (a) compare our decoded
pre-IMDCT coefficient array against a reference MDCT of the
ffmpeg-decoded PCM for burst frame 15 block 0 to pinpoint which
bin's magnitude or sign disagrees, and (b) re-check the §7.1.3
grouped-exponent prefix sum when consecutive deltas sit near the
boundary of the 5-level map (M1=0 or M1=4), since the burst psd
profile is where those extremes actually appear.
