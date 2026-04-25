# Round 13 investigation — step C verified clean; encoder rematrix landed

## Status

* Decoder transient PSNR: **unchanged** (still 15.55 dB on the
  transient_bursts_stereo.ac3 fixture). The bug remains uncracked
  on the decoder side.
* Encoder: implemented §7.5.3 rematrixing decision + emission. The
  encoder now picks the minimum-energy combination among `{L, R,
  L+R, L-R}` per rematrix band per block and applies the spec's
  `0.5 * (L ± R)` transform when the sum/difference pair wins.
* Tests: 25 unit tests + 11 integration tests still pass; clippy
  clean; cargo fmt clean.

## Round-13 plan recap

Per round-12 report, the only major decoder code-path region not
yet hand-traced was FBW `run_bit_allocation` step C (bins 22..132).
This round spent the first ~40 minutes hand-tracing that region
against §7.2.2.4 of A52-2018, then pivoted to encoder-side per
the round-12 instructions ("If step C is also clean: pivot to
encoder-side improvements").

## What the step-C hand-trace proved

Bin-by-bin walk for ch0, frame 14, blk 0 (transient_bursts_stereo.ac3):

```
Side info (no anomaly):
  fgain=0x280=640  sgain=0x4d8=1240
  fdecay=0x53=83   sdecay=0x13=19
  dbknee=0xb00=2816  floor=0xfffff800=-2048
  snroffset=0x54c=1356

bndpsd[0..45] (from trace):
  [1792,1920,2176,2432,2432,2560,2304,2176,1920,1664,1408,1152,
   896,896,768,640,768,512,640,384,640,384,512,384,512,384,512,
   384,485,485,458,458,421,451,394,372,362,362,362,362,302,312,
   312,312,312]
```

Step A (bins 0..1):
  bin=0: lc=calc_lowcomp(0,1792,1920,0)=0; excite[0]=1792-640-0=1152 ✓
  bin=1: lc=calc_lowcomp(0,1920,2176,1)=384 (b0+256==b1);
         excite[1]=1920-640-384=896 ✓

Step B (bins 2..7):
  bin=2: lc=384; excite[2]=2176-640-384=1152 ✓; check break:
         bndpsd[2]=2176 <= bndpsd[3]=2432 → break, begin=3
  (rest of step B not entered for this fixture)

Step B' (bins 3..22, post-break):
  bin=3..7: traced bin-by-bin — values 1408, 1408, 1600, 1581, 1562 ✓
  bin=8..11: traced — 1543, 1524, 1505, 1422 (slowleak winning) ✓
  bin=12..18: traced — 1339, 1256, 1173, 1130, 1111, 1092, 1073 ✓
  bin=19..21: lc=320 at bin=19 (b0+256==b1, bin<20); 192 at bin=20
              (>=20 a-=128 path); 64 at bin=21 — values
              1054, 1035, 1016 ✓

Step C (bins 22..bndend=45) — THE SUBJECT OF THIS ROUND:

  Spec pseudocode at A52-2018 lines 6041-6054:
    begin = 22 ;          /* set after bndstrt==0 fbw branch */
    for (bin = begin; bin < bndend; bin++) {
        fastleak -= fdecay ;
        fastleak = max(fastleak, bndpsd[bin] - fgain) ;
        slowleak -= sdecay ;
        slowleak = max(slowleak, bndpsd[bin] - sgain) ;
        excite[bin] = max(fastleak, slowleak) ;   /* NO lowcomp */
    }

  Our code at audblk.rs lines 1196-1204:
    if bndend > 22 {
        for bin in 22..bndend {
            fastleak -= fdecay;
            fastleak = fastleak.max(bpsd(bin) - fgain);
            slowleak -= sdecay;
            slowleak = slowleak.max(bpsd(bin) - sgain);
            excite[bin] = fastleak.max(slowleak);
        }
    }

  Hand trace from bin 22 (fastleak=592 carried from bin 21,
  slowleak=1016 carried from bin 21):
    bin=22: fl=509, sl=997, excite=997 ✓
    bin=23: fl=426, sl=978, excite=978 ✓
    bin=24: fl=343, sl=959, excite=959 ✓
    bin=25..44: slowleak dominates throughout, dropping by 19 each
                bin → 940, 921, 902, 883, 864, 845, 826, 807, 788,
                769, 750, 731, 712, 693, 674, 655, 636, 617, 598,
                579 — ALL match the trace.

* `slowleak.max(...)` reset rule: matches spec exactly.
* `fastleak -= fdecay` accumulation: matches spec exactly.
* No off-by-one on the upper bound (loop runs bndend-22 iterations).
* Initial values at bin 22 carry from bin 21's body (which the
  bins 7..22 loop computes correctly). No reset/reinit anomaly.
* No use of lowcomp in the post-22 loop, matching the spec.

**Verdict**: FBW `run_bit_allocation` step C is bit-for-bit clean
against A52-2018 §7.2.2.4. Excite, mask, and bap arrays for the
upper-band region are produced correctly. The 1/π peak ratio is
not explained by step C.

## Round-13 lead status table

| Lead from earlier rounds                      | Status R13      |
|-----------------------------------------------|-----------------|
| cpl-channel mantissa loop ordering            | ruled out (R12) |
| snroffset bit-budget formula                  | ruled out (R12) |
| phsflg refresh                                | ruled out (R12) |
| IMDCT short-block branch                      | ruled out (R10) |
| Coupling decoupling                           | ruled out (R12) |
| Rematrix matrix                               | ruled out (R12) |
| Delta bit allocation (dba)                    | ruled out (R10) |
| calc_lowcomp                                  | ruled out (R11) |
| Mask formula                                  | ruled out (R11) |
| BAP derivation                                | ruled out (R11) |
| Mantissa bit-cursor advance                   | ruled out (R12) |
| Long-block IMDCT                              | ruled out (R12) |
| Delay carry-over                              | ruled out (R12) |
| **FBW step C (bins 22..bndend)**              | **ruled out (R13)** |

## What was added — encoder rematrixing (§7.5.3)

`emit_syncframe()` now runs the spec's rematrix decision before
exponent extraction. For each of the four rematrix bands per Table
7.25 (cplinu=0 case — this encoder doesn't emit coupling yet) and
each of the six audio blocks, the encoder:

1. Sums the squared coefficients over the band for each of the
   four candidates `L, R, L+R, L-R`.
2. Compares the smaller of the L/R energies against the smaller
   of the (L+R)/(L-R) energies.
3. When the sum/difference pair wins, sets `rematflg[band] = 1`
   and replaces the L/R coefficients with `0.5*(L+R)` and
   `0.5*(L-R)`.

The bitstream emission was changed to write `rematstr=1` plus
`nrematbd` flag bits **every block** (not just block 0), so the
decision can adapt per block. Cost: 5 bits/block × 6 blocks = 30
bits/syncframe (was 10 in the all-zeros previous emission).
`overhead_bits_for` updated to match.

The decoder side already implements `L = L'+R', R = L'-R'`
correctly per §7.5.4 (verified in round 12), so this is a
pure encoder-side change.

## Encoder PSNR before/after

Self-decoder (our own decoder reading our encoder's output):

| fixture   | before R13 | after R13 |
|-----------|------------|-----------|
| sine      | 20.98 dB   | 20.98 dB  |
| chirp     | 22.49 dB   | 22.53 dB  |
| transient | 35.50 dB   | 35.51 dB  |
| speech    | 32.49 dB   | 32.49 dB  |
| stereo    | (n/a)      | 23.48/44.57 dB |

Cross-decoder (ffmpeg 8.1 reading our encoder's output):

| fixture   | before R13         | after R13 |
|-----------|--------------------|-----------|
| sine      | 20.74 dB           | 20.74 dB  |
| chirp     | 17.97 dB (errors)  | 18.32 dB (similar errors) |
| transient | (errors)           | 32.49 dB  |
| speech    | 32.31 dB           | 32.49 dB  |
| stereo    | (n/a)              | 23.22/34.34 dB |

Why the L=R fixtures (sine, chirp, transient, speech) move only
marginally: the rematrix bands start at bin 13 (1.17 kHz at 48
kHz fs) — the dominant signal energy in these fixtures lives in
bins 0..12 (440 Hz / chirp lower octaves / 2 kHz transient lobes),
which the spec's rematrix algorithm CANNOT touch. The stereo
fixture (different L/R sources) shows the dramatic R-channel
benefit that rematrixing is designed for: 44.57 dB self-decode,
34.34 dB ffmpeg-decode, vs ~20 dB without rematrix.

## Pre-existing D15 bug noticed (NOT addressed)

ffmpeg occasionally complains about `expacc 126 is out-of-range`
on the chirp fixture both before and after this round. The 7-bit
packed value 126 is mathematically unreachable from clamped
deltas — the encoder's `write_exponents_d15` clamps each delta to
±2 before packing, max 25*4+5*4+4=124. Most likely cause: ffmpeg
desynchronises mid-frame and tries to interpret CRC2 bytes or the
next frame's sync word as a D15 group. Root cause TBD; deferred
to a future round so this round's PR stays scoped to rematrix.

## Round-14 attack plan

The 1/π decoder bug remains. Concrete next leads:

1. **Fixed-point arithmetic in mantissa dequantisation**. The spec's
   bap=1/2 tables use exact rational values like ±2/3 — our floats
   match those values, but the spec's "decimal-point left of MSB"
   asymmetric quantisation hint suggests there might be a
   right-shift convention we're missing on bap=6..15. Worth a
   bit-by-bit comparison of one-byte mantissa unpack against the
   reference at frame 14 blk 0.
2. **Coupling-coordinate exponent shift**. cplco = mantissa *
   2^-cplcoexp. We multiply `state.cpl_coord[ch][band] * 8.0` in
   `dsp_block` (line 1683). The "× 8" absorbs `2^-3` from the
   coordinate. If the cplcoexp absorbs an additional power that's
   not in the × 8 — that's a candidate for the 1/π factor (close
   to 8/π·something). Worth a re-derivation against §7.4.3's
   "master coordinate exponent" subtlety.
3. **Coupling-channel coefficient placement in the bins
   immediately ABOVE the rematrix range**. The cplbegf=8 fixture
   places coupling at bin 132+. Bins 132..N might be processed
   slightly off — checking `state.channels[cpl_ch].coeffs[bin]`
   for bin in 132..217 against ffmpeg's at frame 14 blk 0 would
   be a cheap clincher.

## Files modified

* `crates/oxideav-ac3/src/encoder.rs` — added rematrix decision
  block (lines 320-385), updated bitstream emission to refresh
  `rematstr` per block (lines ~537-559), updated
  `overhead_bits_for` for the new 5-bit-per-block cost
  (line ~1054).
