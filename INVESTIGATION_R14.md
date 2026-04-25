# Round 14 — the "transient PSNR bug" was a TEST BUG, not a decoder bug

## TL;DR

After 6 rounds chasing a phantom decoder defect characterised by a
"15.55 dB transient PSNR plateau" and a "1/π peak ratio across burst
blocks", round-14 traced the symptom to its actual source: the
PSNR-vs-ffmpeg integration test in
`tests/ffmpeg_fixture.rs::decoder_matches_ffmpeg_on_transient_fixture`
had a buggy cross-correlation lag-search that locked onto a
periodic-sine local minimum at lag = 252 samples instead of the true
lag = 0.

With the lag-search fixed (search window widened from
`usable.min(4096)` to `usable`), the **same decoder binary** now
reports **92.77 dB** transient PSNR vs ffmpeg — essentially 1-LSB-level
agreement on s16 PCM. The decoder was always correct. The "1/π" ratio
that recurred across rounds 8-13 was an artefact of comparing aligned
PCM regions through a 252-sample shift.

## How the test was lying

The transient fixture is 3 Gaussian-modulated tone bursts (440 / 1200
/ 2400 Hz) embedded in a steady-state 440 Hz background sine. The
post-priming window starts at sample 768 — 16 ms into the file, well
before the first burst at frame 14 (~448 ms in).

The lag-search loop was:

```rust
for lag in -256i32..=256 {
    for i in 0..usable.min(4096) {
        ...
    }
    if sse < best_sse { best_lag = lag; }
}
```

With `usable.min(4096)`, only ~85 ms of pre-burst signal was being
compared. That window contains **~37 cycles of 440 Hz** (period ≈ 109
samples). For a near-pure sine, any lag that is an integer number of
cycles + the true lag gives the same SSE — so the search picks the
first lag in the range that hits a local minimum. On this fixture
that's `lag = 252` (≈ 2.3 cycles), not the true `lag = 0`.

Once `best_lag = 252` was wired into the global PSNR computation, the
test compared `our_pcm[skip+i]` against `ref_pcm[skip+i+252]` over
the **whole** fixture — including the burst region. Through the
burst, this 252-sample shift slides ref's burst transient *forward*
relative to ours, and the per-block RMS of `(our - ref)` blows up.
Result: 15.55 dB "transient PSNR" while the decoder was producing
PCM that, point-by-point at the correct lag, agrees with ffmpeg to
better than 1 LSB.

## How the deception was uncovered

A diagnostic example (`peak_probe.rs`, removed before commit) ran
the per-block cross-correlation with a per-block lag search of
±512 samples. Two readings made the bug pop out:

1. At every burst block, the per-block-best-lag was always `+0`,
   not `+252` — locally the two streams *were* perfectly aligned.
2. The per-block correlation at lag = 0 was `1.0000`, and the
   per-block RMS ratio between our and ref at lag = 0 was within
   1 part in 10^4.

That contradicted the "lag = 252" assumption baked into the global
test. Re-running the global lag search across the entire usable
region (not just `min(4096)`) settled on `lag = 0` with
`sse/sample = 0.567` over **96 000** samples, giving:

```
PSNR = 10 * log10(32767² / 0.567) ≈ 92.77 dB
```

## The fix

`tests/ffmpeg_fixture.rs::decoder_matches_ffmpeg_on_transient_fixture`:
the lag-search now scans the **full** usable region (drop the
`.min(4096)` cap). The transient burst breaks the sinusoidal
periodicity, the search settles on `lag = 0`, and the PSNR
gate's floor is raised from **10 dB** to **80 dB** — the level a
spec-faithful AC-3 decoder owes ffmpeg's reference on this fixture.

Same fix applied prophylactically to
`decoder_matches_ffmpeg_within_psnr_floor` (the sine fixture, which
was passing only because pure-sine PSNR is high at almost any
sub-cycle alignment).

## Numbers

|                              | before R14 | after R14 |
|------------------------------|------------|-----------|
| transient fixture PSNR vs ffmpeg (test report) | **15.55 dB** | **92.77 dB** |
| sine fixture PSNR vs ffmpeg    | ~93 dB | ~93 dB (unchanged) |
| transient burst-block RMS error vs ref @ lag 0 | undetected (test compared at lag 252) | < 1 LSB |
| decoder source changes         | none       | none      |
| test source changes            | n/a        | 7 lines   |

## What we now know about previous "leads"

Every "ruled-out" item from rounds 8-13 — `calc_lowcomp`, the mask
formula, BAP derivation, mantissa cursor advance, IMDCT, dba apply,
phsflg refresh, the snroffset budget formula, FBW step C
(slowleak/fastleak/sdecay) — was correctly ruled out. The decoder was
never wrong on any of those. The 1/π peak ratio was the
periodic-alignment-artefact at the *block envelope* level: at the
start of a burst, our_rms[blk N] ≈ ref_rms[blk N - 1], because the
test's lag = 252 ≈ slightly less than one MDCT block aligned ref's
envelope curve about one block ahead of ours. The ratio
`our_rms / ref_rms` evolves smoothly through the burst, hitting ≈ 1/π
at the leading-edge sub-block where `e^(-1)` of the Gaussian envelope
sits.

## Encoder side

No encoder changes this round. Round-13 already landed the §7.5.3
rematrix decision on the encoder side; that work stands.

## Files modified

* `tests/ffmpeg_fixture.rs` — lag-search window widened in two PSNR
  tests (sine + transient). Transient PSNR floor raised to 80 dB.

That's it. No `src/` changes — the decoder was correct.

## What this should have looked like as a process lesson

Round 8 noted the "1/π peak ratio" datapoint and round-after-round
treated it as a constraint on the *decoder*. The right move at round
8 would have been to ask: "what does that ratio mean *as a pattern*?
Does it survive a different alignment?" A 30-line probe like the one
that finally cracked it (per-block lag search) would have surfaced
the test bug in <1 hour. Instead 6 rounds were spent hand-tracing
spec sections at bin granularity. **Lesson**: when chasing a numeric
anomaly, audit the *measurement* before audiing the *measurement
target*. Especially when the measurement is a custom cross-correlation
whose window-size constant falls inside the periodic regime of the
input signal.
