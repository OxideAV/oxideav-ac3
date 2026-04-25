# Round 12 investigation — transient_bursts_stereo.ac3 PSNR drift

**Status**: bug still NOT cracked. Transient PSNR still 15.55 dB.
Round 12 ruled out **all three** of the round-11 leads (cpl
mantissa loop ordering, snroffset bit-budget, phsflg refresh) by
proving that the per-block bit-cursor in `decode_frame` lands at
the right position every block — and by direct hand-verification
of the bit at frame 0, blk 0, ch0 blksw against the raw bytes.

A genuine bug WAS found and fixed in `parse_frame_side_info`
(double-call of `unpack_mantissas`), but it does not affect
`decode_frame`'s PCM output — only the side-info introspection
helper used by tests. The fix is committed; the transient PSNR
is unchanged because the live decoder never had the bug.

Round-11's hand-trace (§7.2.2 chain) still holds bit-for-bit.

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

# Round-12 additions: dump cpl-channel exponents + rematrix flags +
# per-block bit positions (any one of these env vars is enough to
# turn the corresponding probe on; combine freely with the trace
# vars above).
CARGO_TARGET_DIR=/tmp/cargo-target-ac3 AC3_TRACE_CPL=1 \
    cargo run --example sample_compare -p oxideav-ac3 2>&1 | grep TRACE-CPL
CARGO_TARGET_DIR=/tmp/cargo-target-ac3 AC3_TRACE_REMAT=1 \
    cargo run --example sample_compare -p oxideav-ac3 2>&1 | grep TRACE-REMAT
CARGO_TARGET_DIR=/tmp/cargo-target-ac3 AC3_TRACE_BITPOS=1 \
    cargo run --example sample_compare -p oxideav-ac3 2>&1 | grep TRACE-BITPOS
```

## Round 12 — what was tried + ruled out

### Attack 1: cpl-channel mantissa loop ordering (round-11 lead 1)

`parse_audblk_into` at lines 1431..1525 follows §5.4.3.61 exactly:
`for ch { chmant; if (cplinu && chincpl[ch] && !got_cplchan)
{ cplmant; got_cplchan=1; } }`. The grouped-mantissa buffers
(`grp1`/`grp2`/`grp4`) are persistent across the inner cpl walk
and across channels — matching the §7.3.5 "groups shared across
exponent sets" requirement. Verified by `AC3_TRACE_MANT` trace
on frame 14 blk 0 ch0 vs ch1: every chmant bit position lines
up with `bap`-implied stride. **Loop ordering is NOT the bug.**

### Attack 2: snroffset bit-budget (round-11 lead 2)

`run_bit_allocation` at line 1092 implements the §7.2.2.7
formula `((csnroffst-15)<<4 + fsnroffst)<<2` exactly. Verified
against the audblk syntax table (lines 1798..1808 of A52-2018).
The cpl-channel branch passes `state.cpl_fsnroffst` /
`state.cpl_fgaincod` correctly. Per-block bap delta against a
re-derived mask is well within ±1 step everywhere — the burst
spike does not push any bin into a saturating bap. **Bit-budget
is NOT the bug.**

### Attack 3: phsflg refresh (round-11 lead 3)

`state.cpl_phsflg` is set inside the `if any` block (line 561+).
For frame 14 blk 0 the per-block trace shows `cplstre=true` AND
both `cplcoe[ch]=true` (so `any=true`), so `phsflg` is freshly
read. For the next blocks, when coords are reused, `phsflg` is
also correctly NOT re-read — but we DO continue to apply the
stale state.cpl_phsflg, which is what the spec requires per
§5.4.3.18 ("when phase flags are not transmitted, the previous
values shall be reused"). **Phsflg refresh is NOT the bug.**

### Bonus find: `parse_frame_side_info` double-consume

While instrumenting per-block bit positions to verify Attack 1,
discovered that `parse_frame_side_info` was calling
`unpack_mantissas` once explicitly *after* `parse_audblk_into`
already invoked it internally — a double-consume that advanced
the cursor by ~1.5× per block in the side-info helper.

This means **every test that read `side.blksw` via
`parse_frame_side_info` was reading garbage bits** from inside
the previous block's mantissa region. Two consequences:

1. `tests/ffmpeg_fixture.rs::transient_fixture_has_short_blocks`
   passed with `short_blocks=59/378` despite the actual fixture
   carrying **zero** `blksw=true` bits. After the fix the same
   test reports 0 short blocks. We `#[ignore]` it pending a
   re-encoded fixture that does exercise short blocks.
2. The `examples/count_blksw` example reported up to 39 short
   blocks on a clean sine-wave .ac3; that was also garbage.

**The live decoder (`decode_frame`) was never affected** — it
calls `parse_audblk` (one mantissa walk per block, correct).
The transient PSNR did not move because of this fix.

### What the bit-cursor verification proved

`AC3_TRACE_BITPOS` shows `decode_frame` lands at exactly:

```
frame=0 blk=5 end_pos=6085 frame_bits=6104
```

(763 bytes × 8 − 27 BSI bits = 6077 audio bits available; we
land 19 bits short — fits crc16 + a few padding bits, exactly
as the spec mandates). So `parse_audblk_into` consumes the
spec-prescribed number of bits per block. The per-block start
positions in the live decoder (`27, 1825, 2677, 3529, 4381,
5233`) are the correct positions for blksw — and at each one,
both ch0 and ch1 read FALSE for every frame in the fixture.

### What this means for the 1/π gap

The transient fixture's encoder (`ffmpeg -c:a ac3`) genuinely
ENCODED ALL LONG BLOCKS — the psy-acoustic block-switch decision
threshold inside ffmpeg's encoder did not fire on these
particular Gaussian bursts. Both ffmpeg's decoder AND ours run
the long IMDCT path on every block. ffmpeg's PCM still reaches
peak 5215 on frame 14 blk 1 while ours peaks at 2019 — same
bit stream, same IMDCT branch, same long-block formula. The
1/π factor must therefore be in **coefficient decoding**, not
in IMDCT branching.

This pivot rules out the entire short-block hypothesis tree
(every IMDCT-pair / TDAC-symmetry / window-half investigation
done in rounds 5–10 was looking at the wrong path). The remaining
search space is much narrower:

* **Mantissa decoding** (asymmetric quantisation: signed >> nbits-1
  scale, or symmetric quantisation table values) — but our
  `MANT_LEVEL_*` tables match Tables 7.19–7.23 exactly, and the
  asymmetric scale (`signed * 2^-(nbits-1)`) matches the spec's
  "fractional 2's complement, decimal-point left of MSB"
  definition.
* **Exponent decoding** — D15 differential prefix-sum + clamp to
  `0..=24`. Verified against Table 7.4 for D15 / D25 / D45 grouping.
* **Coupling-channel exponent placement** — `cplabsexp << 1` is
  the *seed* (not stored), differential exponents start at
  `cpl_start`. Matches §7.1.3 lines 3095–3128 exactly.

What WASN'T looked at this round but is suspect for round 13:

1. **The `(2/N)` IMDCT scale is documented in §8.2.3.2 as a
   forward MDCT factor** but our `imdct_512_fft` has scale=1.
   The round-trip test `mdct_imdct_roundtrip_identity_window_tdac`
   DOES pass with our scale=1 IMDCT and our mdct.rs scale=-2/N
   forward — but ffmpeg's encoder also uses -2/N forward, so
   ffmpeg's pre-IMDCT coefficients should match ours bit-for-bit.
   The mystery is why our reconstruction is 1/π smaller.
2. **`run_bit_allocation` for the FBW channel uses
   `is_coupling=false` and falls through `bndstrt==0` branch.**
   The §7.2.2.4 fast-leak / slow-leak initialisation when not in
   coupling sets `(fastleak, slowleak) = (768, 768)` — confirmed
   correct, but the decay loop at bins 22..bndend uses the same
   `fastleak.max(slowleak)` rule as the coupling path, which the
   spec uses ONLY for the coupling channel. For FBW channels the
   bins above 22 should ALSO use the lowcomp-driven excite
   formula. Need to re-read §7.2.2.4 step "C: bin >= 22" carefully.
3. **The per-bin lowcomp recurrence terminates at bin 21 in our
   code** (line 1141: `for bin in begin..22.min(bndend)`).
   For FBW channels with bndend > 22 we then enter the
   coupling-style fastleak.max(slowleak) loop. This may be
   wrong for the bands above 22 in the FBW channel.

The 1/π factor is suspicious as a *coupling-coordinate-related*
artifact (cplco includes `2^-cplcoexp` divisions, and the
spec's "× 8" in §7.4.3 absorbs a `2^-3 = 1/8` from the
coordinate). But cpl bap is all 0 in frame 14, so cpl
coefficients are 0 — decoupling produces 0 added to bins
133..217. Those bins do NOT contribute to the IMDCT peak we're
looking at — which lives in bin 4. So coupling can't be it.

Most likely remaining lead: **bit allocation for FBW channels
above bin 22.** That code path exists ONLY for the FBW channel
when its bandwidth extends above bin 22 (which is the case here:
end_mant=133), and we have two distinct masking-curve regions
(0..22 with lowcomp + fast/slow leak, and 22..end_mant with
just fast/slow leak). The bap values in this upper region
directly drive the mantissa-bit budget for bins 22..132 — most
of the channel's energy. A wrong bap here would change the
mantissa magnitudes for the very bins that contribute most to
the IMDCT peak.

## Round 13 plan

1. Hand-trace `run_bit_allocation` for ch0 bins 22..132 against
   §7.2.2.4 step C, with snroffset / mask intermediates dumped
   to compare against a hand-derivation. Specifically check the
   `fastleak -= fdecay; slowleak -= sdecay` decay rates — these
   should accumulate over bins, but the spec might require
   reset to `bpsd[bin] - fgain` / `bpsd[bin] - sgain` whenever
   `bpsd > fastleak` (we do that for fastleak in line 1146 but
   may be wrong about slowleak).
2. If §7.2.2.4 step C reads correctly, hand-trace the asymmetric
   mantissa-bit budget across bins 0..132 with `AC3_TRACE_MANT`
   and verify the per-bit budget at the encoder side matches by
   regenerating the fixture with `ffmpeg -strict experimental`
   plus `-trace_headers 1`.
