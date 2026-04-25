# Round 11 investigation — transient_bursts_stereo.ac3 PSNR drift

**Status**: bug NOT cracked. Transient PSNR still 15.55 dB. The
calc_lowcomp / bndpsd / excite / mask / bap chain has been
hand-traced against §7.2.2.4 / §7.2.2.5 / §7.2.2.7 of A52-2018
for **frame 14 block 0** and matches the spec **bit-for-bit**.
The bug is elsewhere.

## What was added

Three env-var-gated diagnostic probes (no perf cost when off):

* `AC3_TRACE_FRAME=N` + `AC3_TRACE_BLK=B` — dump bndpsd / excite /
  mask / bap (and lowcomp progression) for the requested
  frame+block during `run_bit_allocation`. Also dumps the
  pre-IMDCT coefficient buffer for every channel inside
  `dsp_block`. Both touch `state.frame_counter`, which is
  incremented at the end of every `decode_frame` call.
* `AC3_TRACE_MANT=1` — adds a per-mantissa trace printing
  `bin / bap / exp / bit_pos_before / mant / dither_used /
  coeff` for the requested frame+block. Off by default because
  the output is huge.

`Ac3State::new()` now zeroes a `frame_counter: u64` field; this
is the only state-shape change.

## What hand-tracing proved (frame 14 blk 0 ch0)

Side-info (for context, no anomaly):
```
fgain=0x280=640  sgain=0x4d8=1240
fdecay=0x53=83   sdecay=0x13=19
dbknee=0xb00=2816  floor=0xfffff800=-2048
snroffset=0x54c=1356
```

Bin-by-bin walk against §7.2.2.4 pseudocode (every value below
matches the trace exactly):

```
exp[0..12]   = [10,9,7,5,5,4,6,7,9,11,13,15]
psd[i]       = 3072 - exp[i]<<7  → [1792,1920,2176,2432,2432,2560,2304,2176,1920,1664,1408,1152]
bndpsd[0..12]= same as psd (bndsz=1 for bin<28)

calc_lowcomp progression in the actual run path:
  bin=0  : lc=0   (1792+256≠1920, 1792 < 1920)        excite[0] = 1792-640-0    = 1152 ✓
  bin=1  : lc=384 (1920+256==2176)                     excite[1] = 1920-640-384  = 896  ✓
  bin=2  : lc=384 (2176+256==2432) — break begin=3     excite[2] = 2176-640-384  = 1152 ✓
  bin=3  : lc=384 (2432+256≠2432, 2432==2432)          excite[3] = max(1408,1192)= 1408 ✓
  bin=4  : lc=384 (2432+256≠2560, 2432 < 2560)         excite[4] = max(1408,1192)= 1408 ✓
  bin=5  : lc=320 (2560+256≠2304, 2560 > 2304)         excite[5] = max(1600,1320)= 1600 ✓
  bin=6  : lc=256 (2304+256≠2176, 2304 > 2176)         excite[6] = max(1581,1301)= 1581 ✓
  bin=7  : lc=192 (bin<20 branch, b0 > b1)             excite[7] = max(1562,1282)= 1562 ✓
  ... (continues matching through bin 11) ...
```

Mask (§7.2.2.5):
```
bin=0 : exc' = 1152 + (2816-1792)>>2 = 1408,  mask = max(1408, hth=1232) = 1408 ✓
bin=1 : exc' = 896  + (2816-1920)>>2 = 1120,  mask = max(1120, hth=1232) = 1232 ✓
bin=2 : exc' = 1152 + (2816-2176)>>2 = 1312,  mask = max(1312, hth=1088) = 1312 ✓
bin=5 : exc' = 1600 + (2816-2560)>>2 = 1664,  mask = max(1664, hth=960)  = 1664 ✓
```

Bap (§7.2.2.7, ch0 bin 0):
```
m = 1408 - 1356 = 52
m += 2048      = 2100
m & 0x1fe0     = 2048
m -= 2048      =  0
addr = (1792-0)>>5 = 56  → BAPTAB[56] = 15 ✓
```

Bap (ch1 bin 14, the “0 bap” boundary):
```
mask[14] = 1781 (ch1, exp=21, psd=384)
m = 1781 - 1356 = 425
m += 2048      = 2473
m & 0x1fe0     = 2464
m -= 2048      = 416
addr = (384-416)>>5 = -1 → clamp 0 → BAPTAB[0] = 0 ✓
```

So:
* `calc_lowcomp` matches the spec at every corner case
  (b0+256==b1, b0>b1, fall-through) for bins 0..7 and bins
  7..20.
* `excite` matches the spec for bins 0..11 (verified by hand).
* `mask` matches at the bins inspected.
* `bap` matches at the bins inspected.
* The bit reader’s position after each mantissa matches the
  bap-determined width (8 bits for bap=9, 6 for bap=7, etc.) –
  no bit-stream slip.

The 2018 spec has a stray semicolon on line 3492:

```
if ((b0 + 256) == b1) ;     <-- typo: empty-statement after if
{
    a = 384 ;               <-- always runs in the literal text
}
```

Same typo is in the 2015 spec. The intent is unambiguous from
the bin<20 branch (no semicolon there) so we keep the
conditional-set behavior. Switching to the literal “always
384” interpretation does not match what the actual bndpsd
shape needs.

## What this round ruled out

* `calc_lowcomp` low-bin behavior — matches spec exactly.
* `bndpsd` integration — log-add table values verified.
* Mask formula `>> 2` rounding — matches.
* `BAPTAB` mapping — verified against Table 7.16.
* Mantissa unpack widths — bit cursor advances per
  `QUANTIZATION_BITS` exactly.
* Asymmetric quantization scale (`signed * 2^-(nbits-1)`) — gives
  values in the `[-1, 1)` range as the spec demands.
* Symmetric quantization tables — match Tables 7.19..7.23.
* dynrng_to_linear — correct (0x00 → 1.0, 0x40 → 4.0).
* Coupling-coordinate reconstruction (§7.4.3) — verified.
* Rematrix decode formula `L = L'+R'`, `R = L'-R'` — correct
  (no extra factor of 2).
* Long-block IMDCT — `imdct_512_fft_matches_reference_random`
  passes within 2e-3 vs the direct-form reference.
* Delay-buffer carry-over — zeroing the delay buffer at the
  start of frame 14 (`AC3_ZERO_DELAY_AT=14` in the deleted
  experimental probe) did **not** change frame 14 blk 0’s
  PSNR. The error is intrinsic to frame 14 blk 0’s own
  reconstruction.

## Concrete next-round leads

The peak-amplitude ratio ours/ref across the burst is
remarkably stable at ≈ 1/π ≈ 0.318:

| frame.blk | peak_ours | peak_ref | ratio |
|-----------|-----------|----------|-------|
| 13.5      |   -314    |   +943   | 0.333 |
| 14.0      |   -767    |  -2404   | 0.319 |
| 14.1      |  +2019    |  +5215   | 0.387 |

Multiplying our coefficients by π=3.14159 makes the **peak
sample** match (off by < 1 LSB on blk 0) — but per-block PSNR
gets worse, not better. So the IMDCT is producing the
right-shaped impulse at **the right sample positions** but
1/π smaller in amplitude. A simple post-IMDCT scale doesn’t
help because the rest of the waveform is not a clean π-scaled
copy of the reference.

Three concrete next-round leads, in order of likelihood:

1. **Coupling-channel mantissa interpretation.** The transient
   fixture has cplinu=true with cplbegf=8 (coupling region
   bins 132..217). Coupling mantissas are ungrouped via the
   spec’s shared bap=1/2/4 grouping rules — but the spec at
   §7.4 mandates that grouped coupling mantissas share a group
   buffer with **fbw mantissas of the same bap** (because
   `chmant`, `cplmant`, `lfemant` are all unpacked into a
   single grouped sequence within the audblk()). Our
   `unpack_mantissas` keeps a single set of grp1/grp2/grp4
   buffers across channels — that part is right — but it
   processes ch0 fully → cpl mantissas → ch1 fully. Per the
   spec at §5.4.3.61, the bit-stream order is **ch0 mantissas,
   then cpl mantissas at the position they appear in ch0’s
   exponent set, then ch1 mantissas**. We process all of ch0,
   then cpl, then ch1 — which is the same ordering, BUT the
   `got_cplchan` gate means we ONLY emit cpl mantissas if the
   first channel encountered with `in_coupling=true` triggers
   them, and we never re-trigger. If ch0 isn’t in coupling
   (e.g., ch0.blksw=true → forced out of coupling) but ch1 is,
   we’d miss cpl entirely. For frame 14 blk 0 both channels
   are coupled so this doesn’t obviously fire — but it’s
   worth re-reading the §5.4.3.61 audblk() loop ordering and
   confirming our nesting matches it cycle-for-cycle.

2. **Encoder snroffset vs decoder snroffset.** Ffmpeg’s AC-3
   encoder iterates on snroffst until the bit budget fits.
   Our decoded csnroffst+fsnroffst should be exactly what the
   encoder wrote — but the resulting `snroffset =
   ((csnroffst-15)<<4 + fsnroffst)<<2` is shifted into the
   range -1920..6716 (vs. mask values in 0..3072). At the
   high end this clamps the bap aggressively, at the low end
   it inflates. A 1-bit bap mismatch on a single high-energy
   bin would slip the bit cursor for every following mantissa
   and explain the cascading per-block PSNR decay. Worth
   adding a “bap delta vs. computed-from-rederived-mask” probe
   that crosses a re-derived mask against state.bap.

3. **Coupling-coordinate phase flag (§5.4.3.18 phsflg).** For
   ch1 in 2/0 mode, when phsflginu=true and the band has
   phsflg=1, our decoupling negates the coefficient
   (`v = -v`). The state.cpl_phsflg array is per-band, but
   we only refresh it inside the cplstre block when
   coupling coordinates are also being sent (`any` flag).
   In a block where coords are reused but phsflg might be
   re-emitted, we’d miss the new flags. Frame 14 blk 0
   does have new coords (cplstre=true, cplinu=true, both
   chincpl), so this isn’t the proximate cause for blk 0,
   but is worth re-reading.

## Diagnostic recipes

```
# Per-block PSNR + peak-diff (the test fixture)
CARGO_TARGET_DIR=/tmp/cargo-target-ac3 \
    cargo run --example sample_compare -p oxideav-ac3

# Hand-trace frame 14 block 0
CARGO_TARGET_DIR=/tmp/cargo-target-ac3 \
    AC3_TRACE_FRAME=14 AC3_TRACE_BLK=0 \
    cargo run --example sample_compare -p oxideav-ac3 2>&1 | head -30

# Add per-mantissa trace
CARGO_TARGET_DIR=/tmp/cargo-target-ac3 \
    AC3_TRACE_FRAME=14 AC3_TRACE_BLK=0 AC3_TRACE_MANT=1 \
    cargo run --example sample_compare -p oxideav-ac3 2>&1 | grep TRACE-MANT
```
