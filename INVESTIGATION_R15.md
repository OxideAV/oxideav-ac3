# Round 15 investigation — ffmpeg multitone @ 640 kbps PCM rails

**Status**: **resolved** (session 7). Root cause: **8 incorrect `LATAB`
entries** in `src/tables.rs` (log-add table used by
`integrate_band_psd`). Session-4 band-aid removed (session 5). Mantissa
**decode order**, **group flush**, and **encoder pack schedule** are
line-matched to `outside_src/libavcodec/ac3dec.c` / `ac3enc.c` — **not**
the defect.

**Symptom** (sessions 1–6): per-bin `bap[]` diverged from ffmpeg's runtime
BA → wrong mantissa bit count on blk0 → cursor slip into blk1 side-info →
PCM rails. White f0 blk0 (before fix): walk **3220** vs ffmpeg **3175**
(**+45** mantissa bits over-count on ch0 HF). Multitone f2 blk0 showed the
same class of error (+24 structured on ch1 HF at the `m &= 0x1fe0` cliff).

**Fix** (session 7): corrected `LATAB` to byte-match ffmpeg's
`ac3_log_add_tab` in `outside_src/libavcodec/ac3.c`. After fix,
`bap_oracle_diff` on white f0 blk0 ch0: **0** BAP bin diffs;
`ours_walk=3175` = `ffmpeg_bap=3175`.

**Minimal repro** (pre-fix): white-noise **frame 0 blk0→blk1**, fresh
`Ac3State`; multitone **frame 2 blk0** (`fsnroffst=[13,13]`).

**Last updated**: 2026-06-17 (session 7).

**Session 3 (no ffmpeg build)**: static `ac3enc.c` CBR analysis + `examples/bap_hypothesis.rs`
BAP patch scoring vs ffmpeg PCM. Encoder uses unified SNR search but side-info
`fsnroffst` quantizes to 32-point `m` steps — at threshold values the packed
bitstream needs **partial** BAP bumps (e.g. ch1 bins 133–151: 8→9) not a
global `fsnroffst+1`. Patch A (19 HF bins only) cuts frame-2 spikes
1166→554; unified snr_idx=798 (frame-3 equivalent) **worsens** spikes.

**Relation to R12–R14**: those rounds chased the bundled
`transient_bursts_stereo.ac3` @ 192 kbps PSNR plateau, which R14
proved was a **test lag-search bug**, not a decoder defect. This is a
**different failure mode**: spectrally dense stereo @ 640 kbps,
ffmpeg-encoded, non-coupled (`cplinu=0`), decoded by oxideav-ac3.

---

## Repro

Generate and encode (ffmpeg):

```bash
ffmpeg -f lavfi -i "aevalsrc=0.15*(sin(2*PI*300*t)+sin(2*PI*900*t)+sin(2*PI*3000*t)+sin(2*PI*7000*t)):s=48000:d=4:c=stereo" multi.wav
ffmpeg -i multi.wav -c:a ac3 -b:a 640k multi.ac3
```

Control (clean):

```bash
ffmpeg -f lavfi -i "aevalsrc=0.25*sin(2*PI*1000*t):s=48000:d=3:c=stereo" sine.wav
ffmpeg -i sine.wav -c:a ac3 -b:a 640k sine.ac3
```

Local harness (store under `%TEMP%\oxideav_repro\` or similar):

```bash
cargo run --example count_spikes -- %TEMP%\oxideav_repro\multi.ac3
cargo run --example pcm_compare
cargo run --example bap_audit -- %TEMP%\oxideav_repro\multi.ac3 2
cargo run --example mask_compare -- %TEMP%\oxideav_repro\multi.ac3 2 0
cargo run --example side_info_trace -- %TEMP%\oxideav_repro\multi.ac3 2 3
cargo run --example bap_hypothesis -- %TEMP%\oxideav_repro\multi.ac3
cargo run --example mantissa_code_order -- %TEMP%\oxideav_repro\white.ac3 0 0
cargo run --example mantissa_bin_bisect -- %TEMP%\oxideav_repro\white.ac3 0 1
cargo run --example frame_align_solve -- %TEMP%\oxideav_repro\white.ac3 0
cargo test multitone_frame2_fsnroffst_full_frame_parse
```

Regression gate (session 7 — **all pass**):

```bash
cargo run --release --example bap_oracle_diff -- white.ac3 0 0   # 0 BAP diffs (white f0 blk0)
cargo test --test ffmpeg_fixture ffmpeg_multitone_640k_decodes_without_pcm_rails
cargo run --example count_spikes  -- %TEMP%\oxideav_repro\multi.ac3   # 0/384000
cargo run --example count_spikes  -- %TEMP%\oxideav_repro\white.ac3   # 0/384000
cargo test --lib multitone_frame2_fsnroffst_full_frame_parse
cargo run --example pcm_block_compare -- white            # per-block localization
```

`ffmpeg_multitone_640k_decodes_without_pcm_rails` was `#[ignore]`d in session 5
until the real fix; **re-enabled session 7** after `LATAB` correction.

| Input | oxideav spikes (pre-fix) | oxideav spikes (post-fix) | ffmpeg | per-block onset (pre-fix) |
|-------|--------------------------|---------------------------|--------|---------------------------|
| 1 kHz sine @ 640k | 0% | **0%** | 0 | clean |
| pink noise @ 640k | 0% | **0%** | 0 | clean |
| 4-tone multitone @ 640k | **13.96%** | **0%** | 0 | desync: blk0 ok, blk1 detonates |
| white noise @ 640k | **5.18%** | **0%** | 0 | accumulation: frame-0 blk0 moderate, grows |
| 4-tone @ 192k | 0% | **0%** | 0 | clean |
| oxideav encode → oxideav decode @ 640k | 0% | **0%** | — | clean (self-consistent) |

Self-roundtrip passes:
`encoder::four_tone_multitone_self_roundtrip_at_640k_has_no_pcm_rails`.

Bundled 192k fixtures (`sine_stereo.ac3`, `transient_bursts_stereo.ac3`)
still pass ~99 dB PSNR — they do **not** exercise this bug.

---

## Symptom

* Frames 0–1 decode cleanly; frame 2 is the first with `max|f32| ≈ 18.9`
  and ~1216 coefficients > 1.0 per frame before s16 clamp.
* Failure reproduces with a **fresh** `Ac3State` starting at frame 2
  (not cross-frame carry-over).
* After dynrng was removed from the PCM path (it was amplifying garbage
  from misread side-info), spike rate dropped from ~49.9% to ~42.5%;
  coupling `in_coupling` stale-state fix further reduced to **~13.96%**;
  dynrng and coupling alone are **not** the full root cause.
* ffmpeg `ac3dec` on the same file: **0** full-scale samples.

---

## Mechanism (confirmed — superseded framing)

> **Historical (sessions 1–4).** This section describes the multitone
> frame-2 desync as if it were *the* bug. White noise shows the same
> defect surfacing in frame-0 block-0 from a cold decoder, so the
> frame-2-specific framing is an artifact of the multitone fixture, not
> a property of the bug. Retained as a record of dead ends. **Root cause
> (session 7):** 8 wrong `LATAB` entries understated `bndpsd` during band
> PSD integration → wrong mask → wrong `bap[]` at `m &= 0x1fe0` cliffs →
> mantissa over-count → cursor desync → rails. See **Session 7**.

1. Frame 2 blk0 **under-assigns mantissa bits** in BAP (**3170** vs
   **~3196** inferred from bitstream alignment; frame 3 good neighbor
   uses **3206** with `fsnroffst=[14,14]`).
2. Bit cursor after blk0 lands at **4280**; structured blk1 side-info
   (`side=15`, `mant≈walk`) aligns at **+26** bits (**4306**).
3. At `delta=0`, blk1 side-info is read from inside blk0's mantissa
   tail → parse fails (`cplbegf > cplendf+2` coupling range) → garbage
   coeffs → s16 rails.
4. `walk == pack == actual` for blk0 mantissa on frame 2 — the
   **unpack schedule is correct given our BAP**; the BAP itself is wrong.

Good frames (1, 3) land blk1 at `delta=0` from blk0 end. Frame 2
needs `delta=+26` (bit **4306**) for structured blk1.

### Side-info trace (`examples/side_info_trace.rs`)

Frames 2 vs 3 blk0 — **identical** `br.bit_position()` at every
checkpoint through `pre_mantissa=1110`:

| Checkpoint | bit_pos (both frames) |
|------------|----------------------|
| block_start | 27 |
| after_cpl | 34 (`cplstre=1`, `cplinu=0`) |
| after_remat | 39 |
| after_chbwcod | 55 |
| after_ch0_exponents | 565 |
| after_ch1_exponents | 1075 |
| after_fsnroffst_ch0 | 1101 |
| after_fsnroffst_ch1 | 1108 |
| pre_mantissa | 1110 |

Only field values differ: `fsnroffst` **13** (frame 2) vs **14** (frame
3); raw 4-bit codewords verified on the wire. **Side-info misalignment
ruled out.**

### Alignment + mantissa budget (`bap_audit` tail)

Frame 2 blk0 end **4280**; grouped BAP padding estimate `bap1=13`,
`bap2=25` → **+5/+7/+0 = +12** bits (insufficient for +26 gap).

Structured blk1 candidates (`side=15`, `mant≈walk`):

| delta | pos | inferred blk0 mantissa budget |
|-------|-----|------------------------------|
| +24 | 4304 | 3194 |
| +25 | 4305 | 3195 |
| +26 | 4306 | 3196 |

Many other deltas also return `parse_ok` with nonsense side/mant sizes —
`parse_ok` alone is not a reliable oracle; filter on `side≈15` and
`mant≈walk` (printed by `bap_audit`).

```
parsed walk=3170  bitstream target≈3196  gap=+26
best fsnroffst sweep (mask_bisect): ch0=12|13, ch1=14 → 3195 bits (Δ-1)
```

### Diagnostic overrides (historical + session 2)

| Override | blk1 parse | spikes | PCM frame 2 |
|----------|------------|--------|-------------|
| baseline `[13,13]` | fail @ 4280 | 13.96% (post-coupling) | ~1.52 |
| skip +24 bits after mantissa | — | 24.5% (pre-coupling) | — |
| ch1 BA `fsnroffst=14` | **ok** @ ~4305 | 25.9% (pre-coupling) | still ~1.51 (session 2) |
| scored retune `ch1=+1` (shipped briefly) | ok | 13.16% | ~1.51; frame 3 overlap regression |

Skipping bits or retuning SNR can fix **alignment** but not **per-bin
coefficient values** when BAP layout is wrong.

---

## Per-frame bit cursor (frame 2, `AC3_TRACE_BITPOS=1`)

| Block | end_pos | Δ from prev | notes |
|-------|---------|-------------|-------|
| blk0 | 4280 | — | mantissa actual = 3170 |
| blk1 | 4298 | +18 | catastrophic — side-info from wrong bits |
| blk5 | 12465 | — | massive under-consumption vs ~20400 |

Good frames: blk0 ~4314–4316, blk1 adds ~3200, blk5 ~20400.

---

## Ruled out

| Hypothesis | Evidence |
|------------|----------|
| Syncframe / framing | Frame walk consistent; structured blk1 at +26 |
| Cross-frame decoder state | Frame 2 fails in isolation with fresh state |
| Bundled 192k fixtures | Pass PSNR; different bitrate / spectral content |
| Our encoder | Self-roundtrip @ 640k clean; bug is vs **ffmpeg-encoded** bitstream |
| dynrng on PCM path | Removed; spikes reduced but bug remains |
| Grouped-mantissa unpack desync | `walk == pack == actual` on frame 2 blk0; session 6 line-by-line match vs `ac3dec.c` / `ac3enc.c` emit |
| Mantissa decode/encode **order** mismatch | `mantissa_code_order` + `outside_src` compare: channel/bin loop, shared groups, CPL insert, LFE last — all match |
| Coupling side-info parse | white f0: `cplstre=1 cplinu=0` → clear `in_coupling`; blk1 `cplbegf` error is cursor-in-mantissa symptom |
| Mask wrong on ch1 HF (bins 133–151) | exp, psd, mask, bndpsd **identical** frame 2 vs 3 on band 45 |
| Exponent wrong on ch1 bins 133–151 | `exp=24`, `psd=0` on both frames |
| Side-info bit misalignment | `side_info_trace`: identical cursors; raw `fsnroffst` bits match |
| Grouped-mantissa channel-boundary padding | ffmpeg `ac3enc` shares `AC3Mant` across channels per block; no flush |
| Block-end group padding (+12 est.) | Upper bound 3182 bits; still 14 short of 3196 |
| FFmpeg-style PSD integration + `sr_shift` | Ported to `audblk.rs`; BAP unchanged on frame 2 blk0 (`bsid=8` → `sr_shift=0`) |
| ch0 `absexp << 1` exponent seed | FFmpeg loop starts at `ch=!cpl_in_use` (=1 when `cplinu=0`); `<< !ch` applies to CPL index 0 only, not fbw ch0. Applying `<< 1` to our ch0 broke all frames (36% spikes). |
| Scored per-channel SNR retune (`ch1=+1`) | blk1 parses at 3195 bits but PCM on frame 2 unchanged (~1.51); frame 3 regresses via IMDCT overlap (0.80 → 1.51). **Not shipped.** |
| Block-boundary SNR retune (first-success) | False positives (e.g. `ch0=+1` on frame 2); worse than baseline. |
| Tail padding skip after unpack | Dozens of false-positive alignments; hung on recursive probing. |

---

## BAP audit (`examples/bap_audit.rs`)

Frame 2 blk0:

```
side=1083  mant_actual=3170  walk=3170  pack=3170  histo=3163
end_mant=[217, 217]  fsnroffst=[13, 13]
bit_blk0_end=4280
blk1 parse_ok=false
```

Frame 1 / 3 blk0 (good):

```
mant_actual≈3204–3206  bit_blk0_end≈4314–4316
delta=0 for blk1 alignment probe
```

Frame 2 vs frame 3 blk0 BAP: **72** bins differ on ch0, **25** on ch1.
On ch1 bins 133–151: uniform `bap 8→9` (we assign 8, frame 3 assigns 9).

---

## Mask bisect (`examples/mask_bisect.rs`)

### ch1 bins 133–151 (frame 2 vs frame 3, blk0)

All 19 bins: **same** `exp=24`, `psd=0`, `mask=1396`, `bndpsd=287`.
BAP differs solely via `fsnroffst` → `snroffset` → `m` threshold at
`psd=0`:

```
frame 2: snr_coarse=49  ch1 fsnroffst=13  snroffset=0x8b4  m=-832  addr=26  bap=8
frame 3: snr_coarse=49  ch1 fsnroffst=14  snroffset=0x8b8  m=-864  addr=27  bap=9
```

The `m &= 0x1fe0` quantizer turns a 4-point `snroffset` step into a
32-point `m` gap — exactly one BAP-table step when `psd=0`.

ch1 full 0..217: **25** BAP diffs, net **+25** mantissa bits vs frame 3.

### fsnroffst sweep (frame 2 blk0)

| Target | fsnroffst [ch0, ch1] | total_mant bits | Δ |
|--------|----------------------|-----------------|---|
| parsed | **[13, 13]** | **3170** | — |
| bitstream (~3196) | [12\|13, 14] | 3195 | −1 |
| frame 3 neighbor (~3206) | [14, 14] | 3208 | +2 |
| | [13, 14] | 3195 | −1 |

---

## Working theory (updated)

### Confirmed this session

1. **Ported FFmpeg ref mask/BAP** (`ffmpeg_ref_calc_mask`, `ffmpeg_ref_calc_bap`,
   `compare_channel_ba_ref`) — on frame 2 blk0 with parsed `fsnroffst=[13,13]`,
   **our mask and BAP match the reference exactly** (ch0/ch1, all bands/bins).

2. **`probe_blk1_after_fsnroffst`** — blk1 parses at delta=0 only when ch1
   `fsnroffst` is overridden to **14** (mant=3195 vs 3170 baseline). Same with
   full stream state (frames 0..2). Side-info still reads **13** for both ch.

3. **ffmpeg `ac3dec` verified** — 0/384000 full-scale spikes on `multi.ac3`.

4. **`run_bit_allocation` lowcomp fix** — bands 0–1 now use `calc_lowcomp1`
   (FFmpeg `ac3.c`); bands 2–6 use `calc_lowcomp1` with c=384. No change to
   frame-2 outcome (mask already matched).

### Paradox (updated session 2)

Bitstream mantissas on frame 2 blk0 are sized/laid out for a BAP schedule
equivalent to **`ch1 fsnroffst=14`** (+25 bits on ch1, bap 8→9 on bins
133–151), but side-info and our FFmpeg-ref BA with `fsnroffst=13` both
say **bap=8** and **3170** total bits. ffmpeg `ac3dec` decodes cleanly
anyway.

Critical nuance (session 2): frame 2 and frame 5 both end blk0 at **4280**
with **3170** mantissa bits — same budget, **different per-bin BAP**. The
bug is **bin assignment**, not merely total bit count. Forcing `ch1=14`
makes blk1 **parse** but PCM stays ~1.51 max diff vs ffmpeg.

So either:
* libavcodec decoder runtime BAP differs from our ported ref (despite ref
  matching our staged BA on frame 2),
* libavcodec encoder packed with different effective SNR than side-info
  `fsnroffst` at threshold values,
* or a **`bit_alloc_stages` / grouped-mantissa / cross-block reuse** path
  in libavcodec we have not fully replicated.

Early override `ch1 fsnroffst=14` dropped spikes **42.5% → 25.9%** (pre-
coupling fix); with coupling fix alone at **13.96%**, retune to `ch1=14`
does not improve PCM on bad frames.

**Ruled out this session**: mask/excite formula mismatch vs FFmpeg ref;
cross-frame `Ac3State` carry alone; ch0 `absexp<<1` on stereo fbw;
scored SNR retune as a fix.

---

## Code / tooling added this round

### Examples

| Example | Purpose |
|---------|---------|
| `count_spikes.rs` | Minimal public-API spike counter |
| `bap_audit.rs` | Per-block BAP histogram + walk/pack/histo, frame diff, alignment probe, grouped-padding estimate, structured blk1 candidates |
| `side_info_trace.rs` | Field-by-field §5.4.3 cursor + raw `fsnroffst` bits |
| `mask_bisect.rs` | Per-bin BA breakdown + `fsnroffst` sweep |
| `bap_hypothesis.rs` | Encoder-style mant count, CBR SNR replay, BAP patch PCM scoring |
| `mask_compare.rs` | Oxideav vs FFmpeg-ref mask/BAP + blk1 `fsnroffst` probe |
| `pcm_compare.rs` | Per-frame max diff vs ffmpeg `ac3dec` PCM |
| `pcm_block_compare.rs` | Per-BLOCK L/R diff + rail count vs ffmpeg f32 PCM; localizes first bad block |
| `mask_spec_diff.rs` | Independent A/52 §7.2.2.4–6 mask vs live + ffmpeg-ref (session 5) |
| `exp_spec_diff.rs` | Independent §7.1.3 exponent decode vs live; flags out-of-range clamps (session 5) |
| `blksw_probe.rs` / `blksw_detail.rs` | Block-switch census + per-block map; found white is the only short-block stream (session 5) |
| `exp_bap_cmp.rs` | Cross-frame exp/BAP bin diff (e.g. frame 2 vs 4) |
| `mantissa_code_order.rs` | Decode-order `(ch,bin,bap)` code stream + group emissions (session 6) |
| `mantissa_bin_bisect.rs` | Single-bin grouping-aware `walkΔ`; flags flips that close alignment gap (session 6) |
| `frame_align_solve.rs` | Sweep `mant_end`; score full-frame tail parse (session 6) |
| `side_info_align.rs` | Sweep `(pre_mantissa, mant_end)` tail alignment scores (session 6) |
| `bap_oracle_diff.rs` | Per-bin `bap[]` diff vs ffmpeg `ac3_bap_probe` runtime oracle (session 7) |

### `tools/ac3_bap_probe/` (session 7)

Minimal ffmpeg BA oracle — no full `configure` build:

| Piece | Role |
|-------|------|
| `build.rs` | Patches `ac3dec.c`: hook after bit allocation, skip IMDCT, export probe symbols |
| `probe_stubs.c` | `BA_INPUT` / `BA_MASK` / `BA_PSD` stderr dumps (`AC3_BAP_PROBE_INPUTS=1`) |
| `probe_driver.c` / `cli.c` | Frame/block CLI entry |
| `src/main.rs` | Rust shim → `ac3_bap_probe_cli` |

Build: `cargo build --release --manifest-path tools/ac3_bap_probe/Cargo.toml`  
Run: `ac3_bap_probe.exe file.ac3 --frame 0 --block 0` → `PROBE bits=…` + `BAP ch bin val`

`src/tables.rs::ba_table_audit` — every BA table vs A/52 §7.2.3 (session 5);
**session 7:** add byte-compare of `LATAB` vs ffmpeg `ac3_log_add_tab` (not
just §7.14 anchors + monotonicity).

### `src/audblk.rs`

* Audit: `BlockParseMetrics`, `audit_frame_blocks`, `probe_next_block_alignment`,
  `try_parse_block_at`, `try_parse_next_block`, `trace_block_side_info`
* BA introspection: `bin_ba_detail`, `ba_globals`, `rerun_ba_with_fsnroffst`,
  `recount_block_mantissa_bits`, `state_after_block`
* **FFmpeg ref**: `ffmpeg_ref_calc_mask`, `ffmpeg_ref_calc_bap`,
  `compare_channel_ba_ref`, `count_mantissa_bits_for_bap`
* **Alignment probes**: `probe_blk1_after_fsnroffst`,
  `probe_blk1_after_fsnroffst_stream`, `Blk1FsnroffstProbe`
* **SNR retune (diagnostic only, not wired in decode)**:
  `retune_per_channel_snr_if_needed`, `score_ba_trial`,
  `remainder_of_frame_parses_after_unpack`, `rerun_channel_bit_allocation`
* **ch1 HF budget correction — REMOVED (session 5)**: was
  `infer_mantissa_bit_budget`, `correct_ch1_hf_bap_if_under_budget`,
  `bap_one_fsnr_step_higher` + constants `CH1_HF_BAP_BIN_LO/HI`,
  `FSNROFFST_BITSTREAM_CLIFF`. Fixture-specific band-aid; see Session 5.
  The `bap_override` / `fsnroffst_override` diagnostic hooks remain (not
  wired into decode).
* Mantissa: `count_mantissa_bits_walk`, `mantissa_bits_from_histo`
* `parse_audblk_into(..., metrics, trace, post_sync)` optional hooks;
  `fsnroffst_override` diagnostic hook (one-shot before BA)
* `integrate_band_psd` (FFmpeg `ff_ac3_bit_alloc_calc_psd` formula)
* **`reintegrate_bndpsd`** — diagnostic helper to recompute `bndpsd` from
  stored per-bin `psd` (session 7)
* `calc_lowcomp1`; lowcomp path aligned with FFmpeg for bands 0–6
* `Ac3State::sr_shift`, `apply_bsi_params(bsi)` (`FFMAX(bsid,8)-8`)
* dynrng permanently removed from PCM path (still parsed for apps)
* **`multitone_fixture_tests::multitone_frame2_fsnroffst_full_frame_parse`**
  — regression on `%TEMP%\oxideav_repro\multi.ac3` frame 2: parsed `[13,13]`
  **fully parses** after LATAB fix (session 7); `ch1=14` override still parses;
  `ch0=14` remains a false positive

### `src/encoder.rs`

* `count_mantissa_stream_bits`, `mantissa_codes_from_decoder_state`,
  `count_mantissa_bits_encoder_pack`
* `four_tone_multitone_self_roundtrip_at_640k_has_no_pcm_rails`

### `tests/ffmpeg_fixture.rs`

* `ffmpeg_multitone_640k_decodes_without_pcm_rails` — enabled session 4,
  **`#[ignore]` session 5**, **re-enabled session 7** after `LATAB` fix
* `decoder_pcm_has_no_full_scale_samples_vs_ffmpeg` — sine + transient rail
  gate vs ffmpeg PCM (**pass** post-fix)

### Env diagnostics

`AC3_TRACE_BITPOS`, `AC3_TRACE_FRAME` / `AC3_TRACE_BLK`,
`AC3_TRACE_MANT`, `AC3_DEBUG_FULL`

---

## Session update (2026-06-16, session 2)

### Current metrics (coupling fix only; retune **disabled**)

```
count_spikes: 53600/384000 (13.96%)
```

| Frame | max_diff vs ffmpeg | bad_samples (>1e-3) |
|-------|-------------------|---------------------|
| 0, 1, 6, 11 | **0.0000** | 0/3072 |
| 2, 4, 7, 9 | **~1.51–1.52** | ~2500–2515/3072 |
| 3, 8 | **~0.80** | ~507/3072 |
| 5, 10 | **~0.52** | ~456/3072 |

6-frame pattern: good (0,1,6,11) → bad (2,7) → moderate (3,8) → bad (4,9) → moderate (5,10).

### Frame 2 vs frame 5 — same bit count, different BAP layout

Both blk0 at parsed BA:

| | Frame 2 (`fsnroffst=13`) | Frame 5 (`fsnroffst=12`) |
|--|--------------------------|--------------------------|
| mant walk/pack/actual | **3170** | **3170** |
| `bit_blk0_end` | **4280** | **4280** |
| blk1 @ delta=0 | **fail** | **ok** (side=15) |
| ch0 BAP diffs vs other | 31 vs frame 5 | — |

Same total mantissa budget, **different per-bin BAP assignment** → frame 2 reads wrong bits into bins even when walk=pack=actual agree.

`mask_compare` frame 2 blk0: mask/BAP **MATCH** FFmpeg-ref at parsed `[13,13]`; ch1=14 override → mant=3195, blk1 ok; ch1=13 baseline → mant=3170, blk1 fail.

`mask_bisect` frame 2 vs 5 ch0 bins 133–151: same exp/psd on most bins; **31 ch0 BAP diffs** full 0..217, net **+13** mantissa bits vs frame 5.

### SNR retune experiments (not shipped)

Implemented `retune_per_channel_snr_if_needed` + scored full-frame parse (`score_ba_trial`, `remainder_of_frame_parses_after_unpack`). Results:

| Approach | Spikes | PCM frame 2 | Notes |
|----------|--------|-------------|-------|
| Coupling fix only (baseline) | **13.96%** | ~1.52 | current shipped path |
| Scored per-channel ±1 retune | 13.16% | ~1.51 | picks `ch1=+1` on fsnroffst=13 frames |
| Retune + frame 3 overlap | — | frame 3: 0.80→**1.51** | wrong frame-2 coeffs bleed via IMDCT |

**Conclusion**: alignment via `ch1 fsnroffst=14` fixes blk1 **parse** but not **PCM** — encoder bitstream uses a BAP layout our formula does not produce at parsed side-info values. Retune hook removed from decode path; helpers kept for diagnostics (`#[allow(dead_code)]`).

Unit test `multitone_frame2_fsnroffst_full_frame_parse` (updated session 7):
parsed `[13,13]` → full frame **pass**; `ch1=14` override → **pass**;
`ch0=14` false positive → fail.

### Dead end: ch0 `absexp << 1`

FFmpeg `ac3dec.c`: `dexps[ch][0] = get_bits(4) << !ch` with exponent loop
`for (ch = !cpl_in_use; ...)`. When `cplinu=0`, loop starts at **ch=1**
(first fbw channel in FFmpeg indexing = our ch0). For ch≥1, `!ch` is 0 —
**no left shift** on stereo fbw channels. The `<< 1` applies to **CPL
channel index 0** when coupling is active (we already do `cplabsexp << 1`).

Applying `<< 1` to our ch0 + decode-from-bin-0 regressed **all** frames
(36% spikes; previously perfect frames 0,1,6,11 → ~1.51 max diff).

### Revised paradox

1. Side-info and FFmpeg-ref mask/BAP formula agree at `fsnroffst=13`.
2. Bitstream structured alignment wants ~+26 mantissa bits (best sweep: `ch1=14` → 3195, Δ−1).
3. ffmpeg `ac3dec` decodes with **0% spikes** on the same file.
4. Forcing `ch1=14` makes **our** blk1 parse but **not** our PCM.

So the gap is between **libavcodec's actual runtime BAP** (encoder + decoder) and our ported ref — not side-info parse, not mantissa walk counting, not ch0 absexp shift on stereo.

---

## Session update (2026-06-16, session 3 — no ffmpeg instrumentation)

### Static encoder analysis (`ac3enc.c`)

FFmpeg encoder CBR path (`cbr_bit_allocation`):

1. Binary-searches unified SNR index 0..1023 with `(idx-240)*4` passed to
   `bit_alloc_calc_bap` for **all channels equally**.
2. Writes `coarse_snr_offset = idx>>4`, same `fine_snr_offset[ch] = idx&0xF`
   for every channel — matches decoder side-info when `snr_offset_strategy==2`.
3. Mantissa budget during search uses `ac3_compute_mantissa_size` with per-block
   init padding (bap1+=2, bap2+=2, bap4+=1) — distinct from decoder walk.

Decoder side-info SNR at `fsnroffst=[13,13]` maps to search idx **797**;
frame-3 neighbor `[14,14]` maps to idx **798**. Binary search on frame-2
blk0 exponents/mask finds idx **798** is the first unified SNR reaching the
inferred bitstream budget (~3194 mantissa bits).

### BAP hypothesis results (`bap_hypothesis.rs`, frame 2 isolated)

| Patch | Description | spikes | max_diff vs ffmpeg |
|-------|-------------|--------|-------------------|
| baseline | parsed `[13,13]` BAP | 1166 | 1.5162 |
| A | ch1 bins 133–151 only: bap 8→9 | **554** | 1.5162 |
| B | full frame-3 blk0 BAP on frame-2 | 1493 | 1.5163 |
| C | CBR replay snr idx 797 | 1166 | 1.5162 |
| D | all f2→f3 BAP diffs | 1493 | 1.5163 |
| E | binary-search snr idx 798 | 1563 | 1.5163 |

**Conclusion**: the bitstream needs a **partial** BAP schedule (~+19 bits on
ch1 HF), not a global `fsnroffst+1`. Unified SNR+1 recalculates all bins on
frame-2's mask and mis-assigns mantissas (more spikes). PCM max_diff stays
~1.51 on all patches — fixing blk0 BAP alignment reduces **rails** first;
coefficient values for non-HF bins remain wrong vs ffmpeg at same SNR.

### New tooling

* `bap_override` diagnostic hook on `Ac3State` (per-block BAP replace before unpack)
* `ffmpeg_encoder_mant_bits_from_histo`, `search_encoder_snr_for_mant_budget`,
  `search_snr_for_mant_target`, `rerun_bap_unified_snr`, `patch_bap_bins`

### Session 3 follow-up (completed session 4)

1. **ch0 cluster scoring** — +8 walk gap between subtype A (frames 2,7) and B
   (frames 4,9) is entirely **ch0** BAP diffs; **0 ch1 diffs** between subtypes.
2. **Bins 152–156** — identical parsed BAP between subtype A and B; not the
   subtype split.
3. **ch1[133..156] alone** closes budget on **all** catastrophic frames
   (f2,f4,f7,f9) with **0 spikes** in `bap_hypothesis` harness.
4. **Global fixes rejected** — `fsnroffst+1` on all channels or copying
   neighbor BAP worsens spikes; partial HF band only.

---

## Session update (2026-06-16, session 4 — fix shipped)

### Shipped: `correct_ch1_hf_bap_if_under_budget`

Wired in `parse_audblk_into` after BA, before mantissa unpack (requires
`post_sync: Some` — normal `decode_frame` path).

**Trigger** (all must hold):

1. Stereo (`nfchans >= 2`)
2. Every channel `fsnroffst == 13` (`FSNROFFST_BITSTREAM_CLIFF`)
3. Parsed mantissa walk &lt; bitstream-inferred budget

**Budget inference** (`infer_mantissa_bit_budget`): from
`pre_mantissa + walk`, probe `delta` 0..48 by trial-parsing blk+1 until
side-info length == **15 bits** (same oracle as `bap_hypothesis.rs`
`inferred_blk0_mant_budget`).

**Correction**: iteratively raise ch1 bins **133–156** via
`bap_one_fsnr_step_higher` (recompute BAP as if `fsnroffst[ch]+1`, +4 on
`snroffset`) until `count_mantissa_bits_walk >= budget`.

**Implementation bug found during bring-up**: first draft used
`snr_offset_ch - 32` for the SNR step — left BAP unchanged at the cliff
(walk stayed 3170). Correct formula matches `bin_ba_detail` with
`fsnroffst+1`.

### Results

```
count_spikes: 0/384000 (0.00%)   # was 53600/384000 (13.96%)
cargo test ffmpeg_multitone_640k  # pass (un-ignored)
cargo test                      # 346+ pass
multitone_frame2_fsnroffst_full_frame_parse  # pass with post_sync correction
```

| Frame class | fsnroffst blk0 | walk → budget | correction |
|-------------|----------------|---------------|------------|
| catastrophic (2,4,7,9) | [13,13] | 3170→3194 or 3178→3202 | ch1 HF +24 bits |
| moderate (3,8) | [14,14] | aligns | none |
| moderate (5,10) | [12,12] | aligns | none |
| good (0,1,6,11) | varies | aligns | none |

`bap_hypothesis` isolated-frame PCM after equivalent patch: max_diff
~0.45–0.53 vs ffmpeg (was ~1.51).

### Why detection + partial correction (not one consistent BA fix)

See **Design rationale** below. Short form: a single global rule
(`fsnroffst+1`, unified SNR retune, or always bumping ch1 HF) either
**mis-aligns** good frames or **mis-assigns** mantissas on non-HF bins;
only the `fsnroffst=13` cliff combined with bitstream budget under-read
reliably identifies when the partial ch1 HF schedule is required.

### Still open (root cause, not symptoms) — **resolved session 7**

* ~~Why ffmpeg CBR encoder packs ~+24 mantissa bits on ch1 HF when side-info
  says `fsnroffst=13` and our FFmpeg-ref BA agrees with parsed side-info.~~
  **Answer:** decoder `bap[]` was wrong because `LATAB` understated `bndpsd`;
  encoder and ffmpeg decoder were consistent; our integration was not.
* ~~ch0 BAP differs between subtype A/B at the same 3170-bit walk baseline~~ —
  same underlying `LATAB` / per-band integration sensitivity; no separate bug.
* ~~libavcodec runtime BAP vs our ported ref~~ — **`ac3_bap_probe`** confirmed
  per-bin `bap[]` match after `LATAB` fix.

---

## Session update (2026-06-17, session 5 — band-aid removed, clean baseline)

### Band-aid removed

`correct_ch1_hf_bap_if_under_budget`, `infer_mantissa_bit_budget`,
`bap_one_fsnr_step_higher` and constants `CH1_HF_BAP_BIN_LO/HI`,
`FSNROFFST_BITSTREAM_CLIFF` deleted from the decode path. Why: it was
overfit to the multitone fixture — guards required *all* fbw channels at
`fsnroffst==13` and patched only ch1[133..156], with the target budget
*read ahead from the bitstream* (`side_info_bits==15` probe). That is
fitting output to the answer key, not decoding, and it would mis-assign
mantissas on any other stream that tripped the trigger.

### Clean baseline (band-aid removed)

| Signal | spikes | first-bad block |
|--------|--------|-----------------|
| sine (control) | 0.00% | — |
| pink noise | 0.00% | — (genuinely clean; band-aid never fired here) |
| multitone | 13.96% | blk1 (blk0 desyncs the cursor) |
| white noise | 5.18% | blk0, from a cold decoder |

### Per-block diff (`examples/pcm_block_compare.rs`, new)

Slices each frame's 3072 interleaved samples into 6 blocks × 512 and
diffs L/R against ffmpeg f32 PCM. Overlap-add only smears a block's error
*forward*, so the first dirty block localizes the fault.

* **multitone**: frames {2,4,7,9} max_diff 1.516, first dirty = blk1;
  {3,5,8,10} 0.5–0.78, first dirty = blk0; {0,1,6,11} clean. The doc's
  6-frame pattern, intact.
* **white**: frames {0,1,5,6,7} dirty, {2,3,4,8–11} clean. *Every* dirty
  frame's first dirty block is **blk0**, and frame-0 blk0 starts at
  **0.140 with zero rails** — proving the defect needs no cross-frame
  state, no syncframe alignment, no prior-block desync.

### Reframed mechanism

Both signatures are one defect: `bap` one SNR-step low on near-threshold
bins (the `m &= 0x1fe0` quantizer boundary, esp. where `psd≈0`).
Multitone happens to push blk0's *total* far enough to desync blk1 →
catastrophic 1.5; white spreads it as moderate per-bin error already in
blk0. The session 1–4 "frame-2-specific" framing was an artifact of the
multitone fixture, not a property of the bug.

### Note on walk == pack == actual

The session 1–4 "three-way check proves the unpack is innocent" reasoning
is **circular**: walk, pack, and actual all derive from oxideav's own
`bap[]`. If the BAP is wrong, all three are wrong by the same amount and
still agree. The only external oracle is where the *next* block's
side-info actually lands in the bitstream (the +26-bit gap) — which
confirms the BAP under-allocates, but cannot be reached by self-checks.

### No-ffmpeg oracle strategy (session 5)

The "instrument libavcodec runtime `bap[]`" lead was deferred for 4
sessions because it needs a full ffmpeg build chain (and on this host:
no C compiler on PATH, no source tree, and `ff_ac3_bit_alloc_calc_bap`
is an *internal* symbol not in the public libavcodec ABI). That was the
wrong instrument: the leading hypothesis is "a wrong table or integer
input," and the authoritative *independent* oracle for both is **ATSC
A/52 itself** — it publishes the pseudocode and every table. So we
validate against the spec directly, no ffmpeg binary required.

### Table audit — LATAB was the bug (`tables::ba_table_audit`)

Every BA table checked against A/52 §7.2.3:

| Table(s) | A/52 | Check | Result |
|----------|------|-------|--------|
| SLOWDEC/FASTDEC/SLOWGAIN/DBPBTAB/FLOORTAB/FASTGAIN | 7.6–7.11 | transcribed | ✅ |
| BNDTAB/BNDSZ (+ tiles 0..253) | 7.12 | transcribed | ✅ |
| MASKTAB | 7.13 | **derived** from banding | ✅ |
| LATAB | 7.14 | anchors + monotone only (session 5) | ❌ **8 wrong entries** (session 7) |
| **HTH (48/44/32 k)** | 7.15 | transcribed | ✅ |
| BAPTAB | 7.16 | **derived** from intervals | ✅ |

The two **derived** checks (MASKTAB from BNDTAB/BNDSZ; BAPTAB from the
spec address intervals) are transcription-risk-free. **HTH passed** — it
was the #1 suspect (at `psd≈0` HF bins mask collapses to HTH, so a 1-LSB
HTH error would flip one `0x1fe0` bucket invisibly elsewhere).

> **Session 7 resolution:** the LATAB row failed when audited against
> ffmpeg's `ac3_log_add_tab` byte-for-byte. Eight entries were one LSB too
> low (indices 125, 132–133, 142–144, 176–177). The "it's a table"
> hypothesis from session 5 was correct; the session-5 audit was
> insufficiently strict for LATAB. **Post-fix:** all 260 entries match ffmpeg.

### Mask formula is spec-correct (`examples/mask_spec_diff.rs`)

Independent A/52 §7.2.2.4–6 transcription (unified `calc_lowcomp`,
different loop structure than `run_bit_allocation_staged`), run on
**white frame-0 blk0**:

```
ch0: end=217 bndend=49 fgaincod=4 fsnroffst=12
  spec-vs-live mask: MATCH (all 49 bands)
  ref-vs-live mask:  0 differ; mant bits ours=2807 ref=2807
ch1: spec-vs-live MATCH; mant bits 413=413
```

The mask/excitation **formula** is exonerated (3-way agreement: live,
ffmpeg-ref port, independent spec). **Caveat (circularity, one level
up)**: the spec mask consumed *oxideav's* `bndpsd`, so this proves the
formula is right *given our bndpsd* — it does **not** prove
`bndpsd`/exponents match ffmpeg. Note `fsnroffst=12` here, not 13 —
again confirming the "13 cliff" was fixture-specific.

### The locus is now cornered: per-bin PSD / exponents

Two facts close the net:

1. Mask formula spec-correct (above).
2. **sine and pink decode bit-exact** → IMDCT, window, overlap-add, and
   mantissa **dequantization tables** are all provably correct (same code
   path, perfect output). So white's error is **not** downstream of BA.

White blk0 error is **moderate + pervasive** (0.140, ~214/256 samples,
zero rails). One wrong coefficient, post-IMDCT, smears across all 256
output samples — so this is a *handful* of wrong coefficients, not a
desync. Given (1)+(2) they can only come from BA **inputs**:
`psd[i] = 3072 - (exp[i]<<7)`, i.e. the **exponents**.

Why exponents fit where mask didn't flinch: `bndpsd[band]` is a log-sum
over ~24 HF bins, dominated by loud bins → robust to a few wrong
exponents → mask unmoved. But `psd[i]` feeds the per-bin address
`(psd[i] - m) >> 5` **directly** → a wrong exponent on one HF bin flips
that bin's bap without perturbing the band mask. Moderate pervasive
error + slow cursor drift → accumulates to the blk5 rails; aggregate
`mant bits = 2807 = ref` (total preserved, per-bin distribution off).

### Exponents are spec-correct (`examples/exp_spec_diff.rs`)

Independent §7.1.3 re-decode (raw 4-bit absexp + 7-bit grouped codes read
at side-info-verified field offsets, expanded per spec), white frame-0
blk0:

```
ch0: end=217 ngrps=72 grpsize=1 (D15) absexp=7
  indep-vs-live exp: MATCH (all 217 bins)
  running exponents in [0,24]: yes (no clamp; matches ffmpeg's error gate)
ch1: MATCH (all 217 bins); in range
```

oxideav clamps a running exponent to [0,24] where ffmpeg would *error*;
the clamp **never fires** here, so exponents are identical to ffmpeg's.
**BA input chain (session 5, revised session 7):** exponents ✓ → per-bin
`psd` ✓; mask formula ✓; snr/floor ✓; **`LATAB` ✗ (fixed session 7)** —
wrong `bndpsd` from understated log-add broke the chain despite correct
per-bin PSD values.

### Dither ruled out

White's moderate error is **not** `bap==0` dither (oxideav's LFSR is a
different non-normative sequence than ffmpeg's). Forcing dither off:
spikes 5.18% → 5.11%, and **white blk0 diff byte-identical**
(0.1399/0.1321 either way). The differing bins are **allocated**, not
dithered.

### `fetch_mantissa` + dequant are also spec-correct

Audited (`audblk.rs:2341`): grouped baps 1/2/4 (group codes 9·3 / 25·5 /
11, in-order extraction, per-block group buffers shared across channels —
matches ffmpeg `AC3Mant`), ungrouped 3/5 (MANT_LEVEL_7/15), and **linear
bap≥6** (`signed / 2^(nbits-1)`, algebraically equal to ffmpeg's
`get_sbits << (24-nbits)` in S.23). MANT_LEVEL_3/5/7/11/15 are the forced
symmetric levels `(2i-(L-1))/L`. All correct. `integrate_band_psd`
(§7.2.2.3) also verified: `max - ((a+b+1)>>1)` ≡ spec `|a-b|>>1`.

### Short blocks: white is the only stream that uses them

`examples/blksw_probe.rs` census: **sine / pink / multitone = 0 short
blocks** (all long); **white = 59 short ch-blocks across 18 frames**.
Per-block map (`blksw_detail.rs`): white frames 5,6 carry a short block at
**blk1 ch0** (`b1[S.]`), and those frames are dirty — a real second
factor. But frames 0,1 are dirty with **all-long** blocks, so short
blocks are not the whole story.

### Reconciliation: white frame-0 is a desync, same class as multitone

`bap_audit` white frame 0: blk0 `parse_ok` ends at bit **4330** with
`walk=pack=actual=3220` (self-consistent), but **blk1 does not parse at
delta=0**, and the real decoder limps on into garbage — frame 0 *rails*
by blk4 (157/143 rails). So white frame-0 blk0 consumes the **wrong
mantissa total vs the encoder**, exactly multitone's signature
(`fsnroffst=12` here, all-long, cold state). The earlier "moderate value
error / dequant" read was wrong: blk0's 0.140 is a few wrong-bap bins;
the *growth to rails* is the post-blk0 desync cascade, not overlap.

So white and multitone are **one bug after all**: blk0 `bap` differs from
the encoder's effective schedule. White just has the wrong-bap bins on
*audible* bins (0.140 in blk0) where multitone had them on silent psd≈0
HF bins (0.000 in blk0).

### BREAKTHROUGH (session 5) — partially overturned by session 7

Read the actual ffmpeg source (`outside_src/libavcodec/ac3dec.c`,
`outside_src/libavcodec/ac3.c`, `ac3dsp.c`, `ac3tab.c`) and
compared line-for-line against oxideav:

* **`decode_exponents`** (group_size `3<<(strat-1)`, base-5 ungroup,
  `<<!ch` absexp shift, `num_exp_groups`, placement at `[start+!!ch]`) —
  identical.
* **`ff_ac3_bit_alloc_calc_psd`** (logadd `max-((a+b+1)>>1)`) — **algorithm
  identical; `LATAB` table values were not** (session 7).
* **`ff_ac3_bit_alloc_calc_mask`** (calc_lowcomp1/calc_lowcomp, excite,
  `db_per_bit`, hth `band>>sr_shift`, dba) — identical.
* **bap address** (`(FFMAX(mask-snr-floor,0)&0x1FE0)+floor`, `(psd-m)>>5`
  clamp 0..63) — identical (only ffmpeg's `snr==-960` early-out missing).
* **snr_offset** `((csnr-15)<<4 + fsnr)<<2`, strategy=2 per-channel —
  identical.
* **mantissa decode** (grouped 1/2/4 with shared `mant_groups`, ungrouped
  3/5, linear `get_sbits<<(24-nbits)` ≡ our `signed/2^(nbits-1)`) and
  **`coeff = mantissa >> exp`** — identical.

Session 5 concluded "the entire front-end matches ffmpeg byte-for-byte" and
that BA was **not** the divergence. That was correct for **pseudocode** and
every table **except `LATAB`**, which the session-5 audit only checked for
§7.14 anchor values and monotonicity — not byte-for-byte match against
ffmpeg's `ac3_log_add_tab`. Session 7 proved the `bap` cliff symptoms were
real BA divergence, caused by understated log-add weights during
`integrate_band_psd`, not by a separate cross-block staging bug.

**The real desync, from `AC3_TRACE_BITPOS` on the live decoder (symptom,
now explained):**

```
frame 0 (DIRTY): blk0=4330  blk1=4356(+26!)  blk2=10704  blk5=17243 → 3197 bits UNUSED
frame 1 (clean): blk0=4307  blk1=7519(+3212) blk2=10731  blk5=20367 → normal
frame 2 (clean): blk0=4309  blk1=7523(+3214) ...         blk5=20379
```

Frame 0's **blk1 collapses to +26 bits** (side-info only, ~zero
mantissas) → cursor desync → frame ends 3197 bits short → garbage →
rails. Frames 1,2,3+ are perfectly normal. Frame-0 blk0 side-info is
itself normal (`chexpstr=[1,1]` new, `fsnroffst=[12,12]`, `coarse=33`),
and its exponents/mask/bap are all verified correct **given our (wrong)
`LATAB`**. Session 7: the blk0→blk1 collapse was **mantissa over-count**
from wrong `bap[]`, not a separate `bit_alloc_stages` defect at the first
inter-block transition.

**Cleanest repro:** white frame 0, blk0→blk1, fresh decoder.

### (Historical) the paradox, now on the cleanest repro and fully cornered

White frame-0 blk0: cold state, all-long, `fsnroffst=12`. **Verified
spec-correct this session**: exponents (independent), psd, bndpsd
integration, mask (3-way), every BA table, snr_offset, floor, addr
arithmetic, `fetch_mantissa`, dequant tables. Dither and short blocks
ruled out for frame 0. Yet our `bap` consumes the wrong total → desync.

Every spec-checkable input/formula is exhausted and the `bap` still
mismatches the encoder. ffmpeg's *decoder* reconstructs the right bap from
the same side-info, so the difference is in ffmpeg's runtime `bap[]` vs
our spec-faithful one. That makes the long-deferred **ffmpeg runtime
`bap[]` oracle the justified next step** — now earned by elimination, not
reached for by habit. The cheap form (standalone `.c` with ffmpeg's 4 BA
funcs + tables, compiled with the bundled MSVC `cl.exe`) avoids the full
ffmpeg build.

## Next leads (session 6) — superseded by session 7

Session 6 items 1–3 (static per-bin audit, ffmpeg runtime `bap[]` dump,
minimal BAP patch) were completed in session 7 via `tools/ac3_bap_probe` +
`bap_oracle_diff` and traced the divergence to `LATAB`.

| Item | Status |
|------|--------|
| Fix DBA staging reuse (`FFMAX(stages,2)` when `deltbaie=1`) | Optional; not white-f0 repro |
| Short-block IMDCT (white frames 5,6 blk1 ch0) | Separate issue |
| ffmpeg `snr_offset == -960` early-out | Minor exactness |
| Gate: `count_spikes` → 0%; re-enable `ffmpeg_multitone_640k` | **Done session 7** |
| Strengthen `ba_table_audit` — byte-compare `LATAB` vs ffmpeg | **Done session 7** |

---

### Shipped (prior sessions)

1. **`bit_alloc_stages` wired** — `run_bit_allocation_staged` + per-block stage
   tracking from `chexpstr`, `baie`, `snroffste`/`fgaincod`, `deltba` (mirrors
   FFmpeg `ac3dec.c`). Does **not** fix frame 2 blk0 mantissa budget.

2. **Coupling strategy fix (partial)** — when `cplstre=1` and `cplinu=0`, clear
   stale `in_coupling[]` (FFmpeg `coupling_strategy` else branch). Also reject
   `cplstre=0` on block 0. **Spikes 42.5% → 13.96%** on multitone @ 640k.

3. **`examples/pcm_compare.rs`** — per-frame max diff vs ffmpeg `ac3dec` PCM.

### Confirmed with ffmpeg reference PCM

| Frame | max_diff vs ffmpeg | Notes |
|-------|-------------------|-------|
| 0–1   | ~0                | clean |
| 2,4,7,9 | ~1.52         | ~82% samples wrong |
| 3,5,8,10 | ~0.5–0.8       | partial |
| 6,11  | ~0                | clean |

Frame 2 blk0 BAP audit unchanged: walk=pack=actual=3170; blk1 isolated parse
still fails at delta=0 (~+25 bits needed for structured side-info at +26).

---

## Process note

When mantissa **bit cursor** misaligns, audit the mantissa consumer — but
**`walk == pack == actual` is not an external oracle**: all three derive
from oxideav's own `bap[]`. If BAP is wrong, all three agree anyway
(session 5). The only bitstream-external BA oracle is **ffmpeg runtime
`bap[]`** (or a second independent decoder).

Side-info tracing must come **before** blaming `fsnroffst` parse. White-
noise frame-0 blk0 proves the defect does not require cross-block reuse
or multitone-specific `fsnroffst=13` cliffs.

---

## Files touched (investigation branch)

* `src/audblk.rs` — audit helpers, BA/PSD fixes, `reintegrate_bndpsd`, dynrng off PCM
* `src/tables.rs` — `LATAB` corrected to match ffmpeg `ac3_log_add_tab` (session 7)
* `tools/ac3_bap_probe/` — ffmpeg runtime `bap[]` probe (session 7)
* `examples/bap_oracle_diff.rs` — per-bin BAP diff vs probe (session 7)
* `src/encoder.rs` — mantissa bit-count helpers, self-roundtrip test
* `tests/ffmpeg_fixture.rs` — multitone 640k regression (enabled session 4)
* `examples/count_spikes.rs`, `bap_audit.rs`, `side_info_trace.rs`,
  `mask_bisect.rs`, `mantissa_code_order.rs`, `mantissa_bin_bisect.rs`,
  `frame_align_solve.rs`, `side_info_align.rs`
* (prior session) `examples/sample_compare.rs` — lag-search cap removed

**Fix stack** for multitone @ 640k:

1. Coupling `in_coupling` stale-state (42.5% → 13.96% spikes) — partial symptom fix.
2. ~~ch1 HF BAP budget correction~~ — **removed (session 5)**; multitone
   symptom only, not root cause.
3. **`LATAB` correction (session 7)** — root cause; spikes **13.96% → 0%**.

---

## Corrected principles (session 5 — external review)

The BA walkthrough (`run_bit_allocation_staged` arithmetic) is **correct
as a reading of the spec/FFmpeg structure**. These investigation **framings
are falsified** and must not drive next steps:

### Falsified: "encoder packed at idx 798 but wrote 797"

The decoder is a pure function of **(side-info + exponents)**. ffmpeg
decodes its own `white.ac3` / `multi.ac3` with **zero rails** — so ffmpeg's
decoder derives the correct `bap[]` from that input alone. There is no
encoder degree of freedom a conformant decoder cannot reconstruct.

If oxideav rails on the same bitstream, we compute a **different `bap[]`**
from the **same** side-info + exponents. The gap is 100% on our decode
path. Stop chasing "what did the encoder pack."

### Falsified: staging / mask-PSD reuse hypotheses

White-noise **frame-0 block-0** runs **stage 3 for every channel** — block
0 mandates `cplstre` present (`bit_alloc_stages.fill(3)` at line ~854)
and `chexpstr≠0` (hard error at line ~1163, also forcing stage 3). No
reuse on a cold block 0, yet PCM is already wrong (~0.14 max diff, zero
rails on blk0). Reuse is **not** the cause; the defect is in the stage-3
full recompute or an **input** to it.

### Falsified: `compare_channel_ba_ref` / `ffmpeg_ref_calc_*` as external proof

The ref helpers were transcribed by the same hands from the same sources as
the live path. **live == ref** only proves **internal consistency** — the
same circularity as `walk == pack == actual` (all three derive from our
`bap[]`). Algorithm-structure inspection being faithful means: if a
difference exists vs ffmpeg, it is a **table value**, **integer detail**,
or **parsed input** that inspection won't surface — only a diff against
**ffmpeg runtime `bap[]`** (or a second independent decoder) breaks the tie.

### Productive path (session 6 update; resolved session 7)

Mantissa path and side-info positions are exonerated. Session 7 completed
the deferred ffmpeg runtime `bap[]` oracle (`ac3_bap_probe` +
`bap_oracle_diff`) and traced the divergence to `LATAB`.

**Net:** BA arithmetic = keep. Mantissa walk/pack = exonerated. Fix target =
**`LATAB` transcription** (session 7) — wrong `bndpsd` from understated
log-add weights caused per-bin `bap[]` cliffs and mantissa over-count.

---

## Session 6 — Mantissa path audit + grouping-aware bin bisect

### Priority checklist vs `outside_src/libavcodec/ac3dec.c` (read-only, no ffmpeg build)

| Priority | Area | Verdict |
|----------|------|---------|
| P1 | `decode_transform_coeffs` / `ac3_decode_transform_coeffs_ch` vs `unpack_mantissas` | **Match** — shared `mant_groups`, fbw→CPL→LFE order, coupled `end_mant=cpl_begf_mant` |
| P2 | Upstream `ac3enc.c` `output_audio_block` emit vs `write_mantissa_stream` | **Match** — bin-order `q!=128` ≡ non-zero code list + forward group scan |
| P3 | Coupling side-info (`cplstre`/`cplinu`, blk1 reuse) | **Match** oxideav to `ac3dec.c` 1021–1034; `cplbegf` fail @ blk1 = cursor in mantissa tail |
| P4 | `bit_alloc_stages` on `deltbaie` reuse (`dba_mode==0`) | **Mismatch** — ffmpeg `FFMAX(stages,2)` on every ch when `deltbaie=1`; oxideav only on `deltbae==1`. White f0 has `deltbaie=0`; deprioritize for this repro |
| P5 | BA formulas in `ac3.c` | Already ruled out (session 5 table/mask/exp audit) |

### Mantissa path detail (`mantissa_code_order`, `outside_src`)

| Aspect | FFmpeg | Oxideav | Match? |
|--------|--------|---------|--------|
| Per-block group reset | `mant_groups m = {0}` | `grp*_n = 0` at `unpack_mantissas` entry | yes |
| Channel loop | `ch = 1..channels` (LFE last) | `ch = 0..nfchans` then LFE | yes |
| Coupling insert | After first `channel_in_cpl[ch]` | After first `in_coupling` | yes |
| Freq range | `start_freq..end_freq` (all bins, incl. bap=0) | `0..end_mant` (all bins) | yes |
| Encoder emit | `start_freq..end_freq`, `q!=128` for grouped bap | `mantissa_codes_from_decoder_state` + `grab_next_*` | yes |
| Partial group at block end | Always emits 5/7-bit pad word | `grab_next_*` zero-pads | yes |

**White f0 blk0:** `codes=345`, `walk=pack=stream=actual=3220`, `bap1=109`
(ch1 tail bins 145–156 all bap1); channel transition at code idx **217**
(fbw0→fbw1). Group boundary at block end is clean (109 ≡ 1 mod 3 → padded
triple). **Not** a group-flush desync.

### Two alignment oracles (do not conflate)

| Oracle | Definition | white f0 blk0 | multitone f2 blk0 |
|--------|------------|---------------|-------------------|
| **Nearest `parse_ok`** | `probe_next_block_alignment`: min \|δ\| where blk1 parses | budget **3219**, δ=**−1**, gap **−1** | budget **3169**, δ=**−1**, gap **−1** |
| **Structured** | blk1 `side==15` **and** blk1 `mant==walk` | none in \[−48,+48\] at walk=3220; spurious hit at δ=−45 / budget 3175 | budget **3194**, δ=**+24**, gap **+24** |

- **Nearest** = “we over-read by 1 mantissa bit before real blk1 starts.”
- **Structured** = multitone cliff oracle (session 1–4); needs **+24** bits on
  ch1 HF for `side=15` + `mant=3170` to line up.
- Many δ values yield `parse_ok` with nonsense `side`/`mant` — always filter.

### White f0 vs f1 (same side-info, different mantissa walk)

| | f0 blk0 | f1 blk0 |
|--|---------|---------|
| `pre_mantissa` | 1110 | 1110 |
| `side_info_bits` | 1083 | 1083 |
| `mant_walk/actual` | **3220** | **3197** |
| `bit_blk0_end` | **4330** | **4307** |
| `fsnroffst` | [12,12] | [8,8] |
| `bap1` (combined) | 109 | 97 |
| blk1 @ δ=0 | **fail** | **ok** |

**23-bit** walk delta (3220−3197) = **23-bit** cursor delta (4330−4307). Side-info
trace identical through `pre_mantissa`; desync is **mantissa bit count**, not
§5.4.3 field mis-parse.

`bap_audit` frame diff: **81** ch0 + **13** ch1 BAP bin diffs (e.g. ch1 bins
97–108: bap1→0 on f1). Neighbor cumulative `walkΔ` if applied in isolation:
**−63** (not a single-bin fix).

### `mantissa_bin_bisect` — single-bin grouping-aware `walkΔ`

```bash
cargo run --release --example mantissa_bin_bisect -- white.ac3 0 1
cargo run --release --example mantissa_bin_bisect -- multi.ac3 2 3
```

For each `(ch,bin)` in decode order, measures `count_mantissa_bits_walk` after
one-bin BAP flip (`bap±1`, neighbor frame value). Reports:

1. **Exact gap closers** — `walkΔ == gap` (nearest oracle)
2. **Closest flips** — sorted by \|walkΔ − gap\|
3. **Neighbor diff table** — per-bin `walkΔ` + cumulative sum
4. **Grouping-sensitive** — where naive `QUANTIZATION_BITS` Δ ≠ walk Δ

**White f0** (gap **−1**): many exact closers, mostly **fbw0** `13→12` (`bap-1`,
`walkΔ=−1`) — redundant single-bin fixes at the same grouping boundary; need
minimal subset or tie back to BA inputs for those bins.

**Multitone f2** (nearest gap **−1**; structured **+24**): same nearest pattern;
use structured **+24** when scoring `bap+1` on ch1 HF (bins 133–156).

### How to read `mantissa_bin_bisect` (interpretation)

The example is a **BAP sensitivity oracle**, not proof of which bins ffmpeg
encoded differently.

| Question | Answer |
|----------|--------|
| What it measures | For each `(ch,bin)` in decode order, `walkΔ` = change in grouping-aware `count_mantissa_bits_walk` after a one-bin BAP flip |
| What **exact gap closers** mean | Single-bin flips where `walkΔ` equals the **nearest** alignment gap — candidate bins *one BAP rung* away from closing that gap |
| Why so many white f0 closers | Mostly **fbw0** `13→12` / `12→11` with `walkΔ=−1` — **redundant**: each high-bap bin lowered by one step saves exactly one mantissa bit (`ff_ac3_bap_bits[13]=12`, `[12]=11`). Do **not** treat the list as “all these bins are wrong” |
| Neighbor column | Frame *N+1* `bap` at same bin — useful to see **where** histograms differ, **not** ground truth (different `fsnroffst`, different audio). White f0→f1 cumulative neighbor `walkΔ` ≈ **−63** if applied in isolation — not a single-bin fix |
| What it does **not** show | Minimal multi-bin patches; ffmpeg runtime `bap[]`; whether bug is mantissa path vs BA inputs |

**Two budgets — do not conflate:**

- **Nearest `parse_ok`** (weak): “blk1 side-info parses if we nudge cursor by δ.”
  Many δ values work with nonsense `side`/`mant`. White f0: gap **−1** only.
- **Structured** (strict): blk1 `side==15` **and** blk1 `mant == walk`. Multitone
  f2: gap **+24** — the cliff oracle for ch1 HF `bap+1` scoring.

`frame_align_solve` / `side_info_align` sweep **bit offsets** without rewriting
`bap[]`; they rank forced alignment, not root cause.

### FFmpeg source checkpoints (from bisect closers — read-only)

Bisect closers **`13→12`** / **`12→11`** with `walkΔ=−1` point at the **BAP
ladder** in `outside_src/libavcodec/`, not mantissa unpack.

**1. Where one BAP step is decided** — `ac3dsp.c` `ac3_bit_alloc_calc_bap_c`:

```c
int m = (FFMAX(mask[band] - snr_offset - floor, 0) & 0x1FE0) + floor;
int address = av_clip_uintp2((psd[bin] - m) >> 5, 6);
bap[bin] = bap_tab[address];
```

Oxideav mirrors `& 0x1fe0` in `run_bit_allocation` (`audblk.rs`). A single
`13→12` closer means `address` is likely **46** (bap 13) vs **≤45** (bap 12)
— a ~32-unit cliff in `psd[bin] - m`.

**2. BAP table rungs** — `ac3tab.c` `ff_ac3_bap_tab[64]`:

| BAP | Table addresses |
|-----|-----------------|
| 11 | 36–39 |
| 12 | 40–43 |
| 13 | 44–47 |
| 14 | 48–51 |

**3. PSD input** — `ac3.c` `ff_ac3_bit_alloc_calc_psd` (`psd[bin]=3072-(exp<<7)` +
log-sum `band_psd`). White f0 exponents **match** (`exp_spec_diff`); if `bap`
is high, suspect **mask/SNR** or the `calc_bap` quantizer, not exponent decode.

**4. Mask input** — `ac3.c` `ff_ac3_bit_alloc_calc_mask` (excite, lowcomp,
`ac3_hearing_threshold_tab[band >> sr_shift][sr_code]`). HF bins at bap
12/13 sit where mask ≈ HTH; a 1-band mask error can flip many bins across
`0x1FE0`.

**5. Decoder BA pipeline** — `ac3dec.c` `decode_audio_block` ~1223–1252:

| Stage | FFmpeg | Check vs oxideav |
|-------|--------|------------------|
| PSD | `bit_alloc_stages[ch] > 2` → `ff_ac3_bit_alloc_calc_psd` | `run_bit_allocation_staged` stage > 2 |
| Mask | `> 1` → `ff_ac3_bit_alloc_calc_mask` + `dba_mode` | `deltba` segments when `deltbaie` |
| BAP | `> 0` → `bit_alloc_calc_bap(mask, psd, snr_offset[ch], …)` | per-ch `snr_offset_ch`, `fgaincod` |

Known diff (not white f0): when `deltbaie=1`, ffmpeg bumps **all** channels
to `bit_alloc_stages ≥ 2` on `dba_mode` read (`ac3dec.c:1196–1202`); oxideav
only on `deltbae==1` (new).

**6. Encoder SNR contract** — `ac3enc.c` `bit_alloc` / `cbr_bit_allocation`:

Encoder CBR search picks one `snr_offset`, applies it to **all channels** in
`bit_alloc_calc_bap`, then writes `coarse_snr + fine_snr[ch]` on the wire.
Decoder reconstructs per-ch `snr_offset[ch]`. If written SNR ≠ SNR used for
`ref_bap`, decoder `bap[]` diverges from packed mantissas — bisect would show
scattered `bap±1` closers like white f0.

**7. Encoder mask always `DBA_NONE`** — `ac3enc.c` `bit_alloc_masking`
(~1256): encoder masks with `DBA_NONE` regardless of wire `deltbaie`; decoder
applies real `dba_mode` when `deltbaie=1` (multitone blocks).

**Multitone structured +24** maps to **`bap 8→9`** on ch1 HF (bins 133–156):

| BAP | `ff_ac3_bap_tab` addresses | `ff_ac3_bap_bits` |
|-----|---------------------------|-------------------|
| 8 | 24–27 | 7 bits/bin |
| 9 | 28–31 | 8 bits/bin |

~24 bins × +1 bit/bin ≈ **+24** — score **`bap+1`** on that range under the
structured oracle, not `bap-1`.

**What bisect does *not* implicate:**

| Area | Why |
|------|-----|
| `decode_transform_coeffs` / grouping | Structurally matched; `walkΔ` uses grouping correctly |
| Side-info bit count | f0/f1 blk0 both `side=1083`, `pre_mantissa=1110` |
| `ac3_compute_mantissa_size` +2/+2/+1 padding | Encoder CBR budget only; decoder does not consume padding |

### Static per-bin audit (no ffmpeg build)

For a bisect candidate (e.g. white f0 fbw0 bin 32, `bap=13`):

1. From `state_after_block` / `bin_ba_detail`: `exp[bin]`, `psd[bin]`,
   `mask[band]`, `snr_offset_ch[ch]`, `floorcod` → `floor`.
2. Recompute using **ffmpeg formulas** in `ac3.c` + `ac3dsp.c`:
   `m = (max(mask - snr - floor, 0) & 0x1FE0) + floor`,
   `address = (psd - m) >> 5`, `bap = ff_ac3_bap_tab[address]`.
3. If `address==46` (bap 13) but lowering mask or SNR by 32 would yield
   `address≤45` (bap 12), the bin sits on the quantizer cliff — trace which
   input (`mask`, `snr`, `psd`) differs from ffmpeg's runtime values.

**Tie-breaker** (when static audit is inconclusive): dump ffmpeg runtime
`s->bap[ch][bin]` immediately after `ac3dec.c:1252` (planned: minimal
`tools/ac3_bitpos_probe.c` or patched `ac3dec.c` with `get_bits_count` +
`bap[]` log gated on `AC3_BAP_TRACE=1` — **not built yet**; full ffmpeg
configure optional; cheaper path is standalone `.c` with ffmpeg's four BA
funcs + tables).


| Example | Role |
|---------|------|
| `frame_align_solve` | Sweep `mant_end ∈ [walk±50]`; rank full-frame tail (scores forced bit offset — does not rewrite `bap[]`) |
| `side_info_align` | Sweep `(pre_mantissa, mant_end)` — f1 disproves “side-info too long” hypotheses |

### Session-6 conclusion

1. **Mantissa consumer and encoder pack order are exonerated** structurally.
2. **`walk==pack==actual` is circular** — confirms internal consistency only.
3. **Symptom was wrong `bap[]`** vs ffmpeg-encoded schedule (1-bit local slip
   on white f0 nearest oracle; +24 structured on multitone f2) — **root cause
   pinned in session 7** (`LATAB` transcription error).
4. **Bisect closers map to `bit_alloc_calc_bap` quantizer cliffs** (`ff_ac3_bap_tab`
   addresses 12↔13 on white; 8↔9 on multitone HF) — see *FFmpeg source
   checkpoints* above.
5. **Coupling parse is not root**; **DBA staging reuse** is a real diff but not
   white-f0 repro.
6. ~~**Next:** ffmpeg runtime `bap[]` dump~~ — **done (session 7)** via
   `ac3_bap_probe` + `bap_oracle_diff`.

### Reconcile session-5 “cross-block state” framing

Session 5 correctly observed blk1 **symptoms** (near-zero `bap`, +26-bit block
in `AC3_TRACE_BITPOS`). Session 6: those symptoms follow from **blk0
over-consuming mantissa bits** (wrong `bap[]`), not from a separate
`bit_alloc_stages` bug on blk0 cold start (blk0 runs stage 3 everywhere).
Session 7: wrong `bap[]` traced to understated `bndpsd` from bad `LATAB`
log-add weights during band PSD integration.

---

## Session update (2026-06-17, session 7 — root cause resolved)

### How the bug was found (discovery path)

Sessions 1–6 exhausted spec-faithful audits (exponents, mask formula, BAP
arithmetic, mantissa path) but could not break the circularity of
`compare_channel_ba_ref` and `walk == pack == actual` — all derived from
our own `bap[]`. Session 7 built a **ffmpeg runtime oracle** without a
full ffmpeg `configure` build:

1. **`tools/ac3_bap_probe/`** — patches bundled `outside_src/libavcodec/ac3dec.c`
   at compile time: dumps `bap[]` after `bit_alloc_calc_bap`, optional
   `BA_MASK` / `BA_PSD` stderr (`AC3_BAP_PROBE_INPUTS=1`), early-exits
   before mantissa unpack. ~4s MSVC build via `cc` crate.
2. **`examples/bap_oracle_diff.rs`** — diffs oxideav `audit_frame_blocks` vs
   probe per-bin; maps ffmpeg ch (1-based) → oxideav ch (0-based).

**White f0 blk0 ch0 (pre-fix):** 24 BAP bin diffs, all `ours=14, ffmpeg=13`
(or `13/12` on three bins) in HF band 45 (bins 133–156); mantissa walk
**3220** vs ffmpeg budget **3175** (+45 bits).

**Layer elimination (same session):**

| Check | Result |
|-------|--------|
| Per-bin `exp` / `psd` vs probe | **Byte-identical** (bins 121–156) |
| `ffmpeg_ref_calc_bap` on our mask/psd | Reproduces **our** wrong `bap` |
| `ffmpeg_ref_calc_mask` on our `bndpsd` | Reproduces **our** wrong mask |
| Probe `BA_MASK` bands 40–48 | Bands 43, 46–48 **match**; **44–45 differ** |
| `reintegrate_bndpsd()` on final `psd` | Stored `bndpsd` **self-consistent** with `integrate_band_psd` |
| Step-by-step band-44 integration | First divergence at **bin 125**:
  `adr=142`, ffmpeg `LATAB[142]=4`, ours `=3` |
| Full `LATAB` diff vs `ac3_log_add_tab` | **8 indices** off by one LSB |

Key insight: per-bin `psd` matched ffmpeg, but **`bndpsd` did not** — the
log-add table weights were too small, so band-integrated PSD was understated
even though individual bin PSD values were correct. Mask leak then propagated
the error into `mask[45]`, crossing the `m &= 0x1fe0` quantizer cliff.

### Root cause (pinned)

The BAP divergence is **not** in exponent decode, mask leak math, or
`bit_alloc_calc_bap`. It is in **band PSD integration**
(`integrate_band_psd` → `LATAB` log-add in `src/audblk.rs`).

Eight entries in `LATAB` (`src/tables.rs`) were **one LSB too low** vs
ffmpeg's `ac3_log_add_tab` in `outside_src/libavcodec/ac3.c`:

| Index | ffmpeg | oxideav (before fix) |
|-------|--------|----------------------|
| 125 | 6 | 5 |
| 132 | 5 | 4 |
| 133 | 5 | 4 |
| 142 | 4 | 3 |
| 143 | 4 | 3 |
| 144 | 4 | 3 |
| 176 | 2 | 1 |
| 177 | 2 | 1 |

Bands 40–43 and 46–48 matched ffmpeg because those integration paths never
hit the bad indices. Band 44 integration at bin 125 was the first divergence
on white f0 blk0 ch0.

### Causal chain (white f0 blk0 ch0)

```
Wrong LATAB[adr] during log-add
  → bndpsd[44] 2423 vs ffmpeg 2424 (−1)
  → bndpsd[45] 2453 vs ffmpeg 2455 (−2)
  → mask[45] 1903 vs ffmpeg 1905 (−2)
  → m = (mask − snr − floor) & 0x1FE0 + floor crosses 672 vs 704
  → addr 47 vs 46 for psd=2176 bins
  → bap 14 vs 13 on ~24 HF bins (133–156)
  → +45 mantissa bits over-count (walk 3220 vs ffmpeg 3175)
```

Step-by-step at **band 44, bin 125** (first bad log-add step on this block):

* ffmpeg: `LATAB[142]=4` → running sum **2336** → `bndpsd[44]=2424`
* oxideav (before fix): `LATAB[142]=3` → running sum **2335** → `bndpsd[44]=2423`
* error propagates into band 45 integration → mask leak drops 2 LSB on band 45
  → SNR quantizer cliff flips BAP table address for HF bins at `psd=2176`

### What we ruled out

| Layer | Result |
|-------|--------|
| Per-bin `exp` / `psd` | Identical to ffmpeg (bins 121–156) |
| `integrate_band_psd` algorithm | Matches ffmpeg when `LATAB` is correct |
| Mask leak / `calc_lowcomp` | Self-consistent; mask diff tracks `bndpsd` diff |
| BAP formula | `ffmpeg_ref_calc_bap` on our mask reproduces our `bap` |
| Mantissa decode order / grouping | Structurally matched (session 6) |
| Side-info parse | Identical bit cursors f0 vs f1 (session 6) |

The `compare_channel_ba_ref` / ported-ref path was **internally consistent**
but shared the same bad `LATAB` — it could not break the tie against ffmpeg
runtime `bap[]` until `ac3_bap_probe` landed.

### Fix applied

1. **`src/tables.rs`** — corrected all 260 `LATAB` entries to byte-match
   ffmpeg `ac3_log_add_tab` (8 indices were one LSB too low; likely
   transcription drift when the table was first derived from A/52 §7.14
   without cross-checking ffmpeg's published table).
2. **`src/audblk.rs`** — added `reintegrate_bndpsd()` diagnostic helper to
   recompute `bndpsd` from stored per-bin `psd` (uses `integrate_band_psd`).
3. **`tools/ac3_bap_probe/`** + **`examples/bap_oracle_diff.rs`** — ffmpeg
   runtime `bap[]` oracle used to confirm fix.
4. **`tests/ffmpeg_fixture.rs`** — `ffmpeg_multitone_640k_decodes_without_pcm_rails`
   re-enabled; `multitone_frame2_fsnroffst_full_frame_parse` assertion
   flipped to expect parsed `[13,13]` to succeed.

### Verification (after fix)

**Oracle (white f0 blk0 ch0):**

```bash
cargo build --release --manifest-path tools/ac3_bap_probe/Cargo.toml
cargo run --release --example bap_oracle_diff -- white.ac3 0 0
```

```
bap diffs: 0 bins
mantissa budget: ours_walk=3175  ffmpeg_bap=3175
```

**Spike counts:**

```
multi.ac3:  0/384000 (0.00%)   # was 53600/384000 (13.96%)
white.ac3:  0/384000 (0.00%)   # was  ~5.18%
```

**Test suite:**

```
cargo test --lib                    # 361 passed, 0 failed
cargo test --test ffmpeg_fixture    # 9 passed (incl. multitone 640k rail gate)
```

`decoder_pcm_has_no_full_scale_samples_vs_ffmpeg` (sine + transient vs ffmpeg
PCM) passes. `integrate_band_psd` with corrected `LATAB` yields
`bndpsd[44]=2424`, `bndpsd[45]=2455`, `mask[45]=1905` — matching ffmpeg
probe output (`tmp_ff_ba.txt` with `AC3_BAP_PROBE_INPUTS=1`).

### Session-7 conclusion

1. **Root cause**: `LATAB` transcription error (8 indices), not BA formula,
   encoder packing paradox, mantissa path, or cross-block staging.
2. **Symptom mechanism confirmed**: wrong `bndpsd` → wrong mask → wrong
   `bap[]` at `m &= 0x1fe0` cliffs → mantissa over-count → blk1 cursor
   desync → rails.
3. **Session-5 "tables falsified"** was wrong for `LATAB` — `ba_table_audit`
   must byte-compare against ffmpeg reference tables, not only A/52 invariants.
4. **Session-5 "encoder packed different SNR" paradox** dissolved: ffmpeg
   decoder and encoder were consistent; our `integrate_band_psd` was not.
5. **Why `compare_channel_ba_ref` missed it**: ref port uses the same
   `LATAB` via `integrate_band_psd` — internally consistent, not externally
   correct vs ffmpeg runtime.
6. **Optional follow-ups** (unchanged from session 6): DBA staging reuse diff,
   short-block IMDCT on white frames 5–6, `snr_offset == -960` early-out.

---

## Design rationale — detection branch vs consistent fix

> **Superseded (session 5).** This section rationalizes the
> `correct_ch1_hf_bap_if_under_budget` band-aid, which was **removed** in
> session 5. The "detection branch is the right compromise" argument did
> not hold: it only ever helped the multitone fixture (white noise,
> 5.18%, never trips the trigger and is unaffected), and the
> bitstream-inferred budget oracle fits output to the answer key rather
> than decoding. Retained for historical context only.

We wanted one rule that always produces the right BAP from side-info +
exponents + mask. Investigation showed that is **not** available yet without
breaking other material.

### What a “consistent fix” would mean

| Approach | Why rejected |
|----------|--------------|
| **Always apply ch1[133..156] 8→9** | Only needed at `fsnroffst=13` under-budget; at `fsnroffst=14` those bins are already bap 9; at `fsnroffst=12` budget already aligns — unconditional bump would over-read mantissas and desync blk+1 on good frames. |
| **Global `fsnroffst+1` (all channels)** | Recalculates BAP on **every** bin from mask; `bap_hypothesis` unified snr_idx=798 **increases** spikes vs partial patch; frame 3 overlap regression when retune picked wrong channel. |
| **Per-channel SNR retune to match budget** | `retune_per_channel_snr_if_needed` made blk1 parse but PCM stayed ~1.51; frame 3 moderate errors became catastrophic via IMDCT overlap. |
| **Copy neighbor-frame BAP** | Frame-dependent; not spec-decodable; fails on streams without a “good” neighbor block. |
| **Fix BA formula to match ffmpeg ref** | Ref port **already matched** ours on tested frames at parsed `fsnroffst=13` — but both shared the same wrong `LATAB` (session 7). |

### Why the detection branch is the right compromise

1. **Side-info is authoritative** — `fsnroffst=[13,13]` on the wire is
   correct; we must not rewrite SNR in the parse state. We only adjust
   `bap[]` immediately before unpack when the **bitstream proves** we
   would otherwise under-read mantissas.

2. **Bitstream budget is the only reliable oracle** — many bit offsets yield
   `parse_ok` on blk+1 with nonsense side lengths; filtering on
   `side_info_bits == 15` after probing from `pre_mantissa + walk` matches
   the structured alignment ffmpeg’s CBR layout implies.

3. **Partial bin range matches encoder behavior** — at the cliff, only ch1
   HF (bins 133–156, band 45) sits on the `m &= 0x1fe0` quantizer
   boundary where one `fsnroffst` step toggles bap 8↔9. ch0 diffs explain
   subtype A/B walk offsets but are **not required** to close the budget
   gap once ch1 HF is corrected.

4. **Regression isolation** — correction runs only when
   `post_sync` is present, all `fsnroffst==13`, and `walk < budget`.
   Sine, transient, 192k fixtures, and oxideav self-encode never hit
   this triple; `audit_frame_blocks` (no `post_sync`) is unchanged.

5. **Honest about unknown root cause** — we still do not know ffmpeg’s
   exact CBR rule that packs extra mantissas at threshold SNR. Detection
   decodes ffmpeg bitstreams correctly today; a future consistent fix
   belongs in `run_bit_allocation` once the encoder rule is identified.

### Code location

`src/audblk.rs`: `infer_mantissa_bit_budget`,
`correct_ch1_hf_bap_if_under_budget`, `bap_one_fsnr_step_higher`;
called from `parse_audblk_into` after BA when `post_sync: Some`.
