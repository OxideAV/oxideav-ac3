# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- E-AC-3 (Enhanced AC-3) encoder per ATSC A/52:2018 Annex E, round-1
  scope: a single independent substream (`strmtyp=0`, `substreamid=0`,
  `bsid=16`) carrying mono (acmod=1) or stereo (acmod=2) audio at
  32 kHz / 44.1 kHz / 48 kHz, 6 audio blocks per syncframe
  (`numblkscod=3`), no coupling, no spectral extension (`spxinu=0`),
  no Adaptive Hybrid Transform (`ahte=0`), no transient pre-noise
  processing. New `eac3::make_encoder` constructor and
  `crate::CODEC_ID_STR_EAC3 = "eac3"` registration. The DSP pipeline
  (windowing, MDCT, exponent extraction + D15 strategy, parametric bit
  allocation, dba, mantissa quantisation) is shared with the AC-3
  encoder via newly-pub(crate) helpers (`extract_exponent`,
  `preprocess_d15`, `compute_bap`, `tune_snroffst`, `build_dba_plan`,
  `quantise_mantissa`, `write_exponents_d15`, `write_mantissa_stream`,
  `ac3_crc_update`, `BitAllocParams`, `CouplingPlan`, `DbaPlan`,
  `TransientDetector`, `decode_input_samples`). Framing diverges:
    - **syncinfo** (§E.2.2.1) is just the 16-bit syncword `0x0B77`;
      no `crc1` field.
    - **bsi** (§E.2.2.2) replaces AC-3's `bsid≤8` layout with
      `strmtyp(2) + substreamid(3) + frmsiz(11) + fscod(2) +
      numblkscod(2) + acmod(3) + lfeon(1) + bsid=16(5) + dialnorm(5)
      + compre(1) + mixmdate(1) + infomdate(1) + addbsie(1)`.
      `frmsiz = (frame_size_in_words - 1)` per §E.2.3.1.3 — the size
      table from AC-3's §5.4.1.4 is gone.
    - **audfrm** (§E.2.2.3) sits between bsi and the audblks and
      carries frame-level strategy flags (`expstre=1, ahte=0,
      snroffststr=0, transproce=0, blkswe=1, dithflage=1, bamode=1,
      frmfgaincode=1, dbaflde=1, skipflde=1, spxattene=0`), per-block
      coupling-strategy flags (`cplinu[0..5]=0`), per-block per-channel
      `chexpstr[blk][ch]` (2 bits each), per-channel `convexpstr[ch]`
      (5 bits each, value=0 = D15 + 5×REUSE per Table E2.10), and the
      shared frame-level `frmcsnroffst(6) + frmfsnroffst(4)`.
    - **audblk** (§E.2.2.4) emits `blksw + dithflag + dynrnge=0 +
      spxinu=0 + (rematrix flags when acmod==2) + chbwcod (D15 blocks
      only) + exponents (D15 blocks only) + bamode params (block 0)
      + fgaincode=0 (default fgaincod=4 for all chans) + convsnroffste=0
      + dba + skiple=0 + mantissas`. The `snroffststr=0` choice means
      every channel reads the same `frmfsnroffst` from the audfrm —
      `compute_bap` is fed the base `fsnroffst` (NOT the per-channel
      `fsnroffst_ch[ch]` array the AC-3 path uses) so encoder and
      decoder derive identical bap[] arrays.
    - **errorcheck** (§E.2.2.6) is just `encinfo(1) + crc2(16) = 17
      bits`. crc2 covers bytes `[2..frame_bytes-2]` with the same
      polynomial and initial value as AC-3 §6.1.7 (Annex E doesn't
      redefine the CRC).
  Test gates (3 new tests, total = 56):
    - `eac3_first_frame_is_syncframe` — every 768-byte boundary in
      a 192 kbps stereo stream starts with `0x0B 0x77`.
    - `eac3_stereo_192k_decodes_through_ffmpeg` — encode 1 s of
      440 Hz stereo, decode through `ffmpeg -f eac3 -i …`, assert
      PSNR ≥ 18 dB. Measured: **20.21 dB** (matches the AC-3 baseline
      encoder's PSNR-vs-ffmpeg on the same input — ~20.7 dB).
    - `eac3_mono_96k_decodes_through_ffmpeg` — same shape for mono,
      96 kbps. Measured: **20.21 dB**.

### Fixed

- Round 24 (task #103) — replaced the ad-hoc first-difference + 4×
  energy-ratio transient detector with a spec-faithful §8.2.2
  implementation: a 4th-order Butterworth high-pass at 8 kHz cutoff
  (cascaded direct-form-I biquads) followed by the hierarchical
  three-level peak-ratio test (T₁=0.1, T₂=0.075, T₃=0.05) with a
  `100/32768` silence threshold. Per-channel state holds the biquad
  memory and the previous block's last-segment peaks so the cross-
  block "k=1" comparisons of step 4 work as written. The previous
  detector mis-fired on low-frequency pure tones (e.g. 220 Hz sine):
  its 32-sample sub-frame energy ratio crossed 4× whenever a sub-
  frame happened to land near the sine's zero-crossing, triggering
  the 256-point short MDCT on a steady-state signal. Short MDCT on
  a pure tone smears the bin energy across multiple bins, dropping
  the ffmpeg-cross-decode L-channel PSNR on the 5.1 fixture from a
  spec-expected ~24 dB down to **14.36 dB**. After the fix the
  L-channel reads **24.54 dB** (+10.2 dB), matching the other fbw
  channels' bit-allocation ceiling. The decoder side was already
  correct; this was strictly an encoder transient-decision bug.
  `transient_roundtrip_self_decode` had its synthetic-burst fixture
  re-shaped (sharper σ=12 sample envelope at 4/8 kHz carrier) so the
  bursts carry meaningful HF content for the 8 kHz HPF to pass — the
  old σ=32 / 800-2400 Hz bursts were below the spec detector's
  threshold and would trip a regression test that documents real
  detector behaviour.

### Added

- Round 24 (task #103) — per-channel `fsnroffst[ch]` tuning
  (§5.4.3.40). `BitAllocParams` now carries a `[u8; MAX_FBW]` array
  of per-channel fine-SNR offsets; after the global `(csnr, fsnr)`
  selection the tuner does a greedy per-channel sweep that bumps
  individual channels' `fsnroffst[ch]` as long as the residual frame
  budget allows. Previously every fbw channel emitted the same
  `fsnroffst` value, leaving budget on the table when one channel's
  mask had more headroom than its peers. The bitstream syntax always
  allowed per-channel emission; the encoder just wasn't using it.

- Round 19 — multichannel encoder. The encoder now accepts `channels`
  ∈ 1..=6 with per-channel-count acmod selection per A/52 Table 5.8:
    - `1` → acmod=1 (1/0 mono)
    - `2` → acmod=2 (2/0 L,R)             — unchanged from earlier rounds
    - `3` → acmod=3 (3/0 L,C,R)
    - `4` → acmod=6 (2/2 L,R,Ls,Rs)
    - `5` → acmod=7 (3/2 L,C,R,Ls,Rs)
    - `6` → acmod=7 + lfeon=1 (3/2 + LFE — canonical 5.1 layout
      L,C,R,Ls,Rs,LFE)
  BSI emission switches on acmod for the `cmixlev` / `surmixlev` /
  `dsurmod` optional fields per §5.4.2.4-7 + Tables 5.9 / 5.10 (we
  emit the spec-default `01` = -3 dB centre/surround coefficient when
  applicable). LFE pipeline added end-to-end: separate exponent
  extraction over bins 0..7 (§5.4.3.29), `lfeexpstr` 1-bit per block
  per §5.4.3.23, LFE-specific bap routine (LFE never short-blocks
  per §5.4.3.1, no DBA per §5.4.3.47, dedicated `lfefsnroffst /
  lfefgaincod` SNR knobs per §5.4.3.42-43), and LFE mantissas
  emitted last per the §7.3.2 read order. Coupling and rematrix
  remain 2/0-only as the spec requires (`acmod==2` gate). Default
  bit rates per channel count: 1→96, 2→192, 3→256, 4→320, 5→384,
  6→448 kbps. Test gates:
    - `mono_self_decode_roundtrip`
    - `three_zero_self_decode_roundtrip`
    - `three_two_self_decode_roundtrip`
    - `five_one_self_decode_roundtrip`
    - `five_one_ffmpeg_crossdecode` — encodes 5.1 input with a
      unique tone per channel + an 80 Hz sub-bass on LFE, decodes
      via ffmpeg, asserts every channel survived (per-channel RMS
      gate) and reports per-channel PSNR after WAVEEX channel
      reorder. Measured per-channel PSNR on 0.5s tonal fixture at
      448 kbps: L 14.4 dB, C 24.1 dB, R 44.7 dB, Ls 24.2 dB,
      Rs 24.7 dB, LFE 28.4 dB.
- Round 18 — encoder-side §7.2.2.6 / §5.4.3.47-57 delta bit allocation.
  Encoder now emits `deltbaie=1` on block 0 of every syncframe with a
  per-fbw-channel single-band segment (`deltbae[ch]=1`, `deltnseg=1`),
  picked greedily from the lowest-energy 1/6th-octave band in the 25..45
  range with `deltba=4` (+6 dB mask boost). Coupling channel signals
  `cpldeltbae=2` (no delta this block) when coupling is active. Blocks
  1..5 emit `deltbaie=0` (reuse) — block-0's segment list applies for
  the rest of the syncframe. `tune_snroffst` accounts for the dba
  syntax cost (≈17 bits per fbw channel + 2 bits per channel for
  `deltbae[ch]` + 2 bits for `cpldeltbae` when cpl is in use), and
  `compute_bap` / `compute_bap_cpl` apply the dba mask offsets before
  bap[] computation so encoder and decoder derive identical bap[]
  arrays. ffmpeg cross-decodes the dba-bearing stream cleanly.
  `AC3_DISABLE_DBA=1` reverts to the round-17 deltbaie=0 behaviour for
  A/B comparison. Test gate: `dba_self_decode_and_ffmpeg_crosscheck`.
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
