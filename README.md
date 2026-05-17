# oxideav-ac3

Pure-Rust **AC-3 (Dolby Digital)** + **E-AC-3 (Enhanced AC-3 / Dolby
Digital Plus)** audio decoder + encoder — elementary streams per
ATSC A/52:2018 (= ETSI TS 102 366). Zero C dependencies.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Status

Early WIP. Implementation follows the A/52 spec incrementally:

- [x] Sync frame + BSI parse (§5.3 / §5.4.1-2)
- [x] Audio-block parse (§5.4.3) — every §5.4.3.x field cited and
      captured into `AudBlkSideInfo` for introspection
- [x] Exponent decode (§7.1) + parametric bit allocation (§7.2)
- [x] Mantissa decode (§7.3) with bap=0 dither (§7.3.4)
- [x] IMDCT synthesis (§7.9) — 512-point long-block path;
      256-point short-block still uses a reference (non-FFT) IMDCT
- [x] Channel coupling (§7.4) + rematrix (§7.5) + dynrng (§7.7) —
      coupling now spans up to 5 fbw channels (encoder + decoder),
      matching the spec's nfchans limit (5.1 minus LFE). At 320 kbps
      on HF-rich 5.1 content the multichannel cpl path lifts the
      average self-decode PSNR by **+3.12 dB** over the no-coupling
      baseline at matched bitstream size (round 25 / task #155).
- [x] Delta bit allocation (§7.2.2.6) — encoder + decoder, with
      tonal-vs-noise psy classification (round 30): `band_is_tonal` measures
      exponent spread (min vs. mean) per band across 6 blocks; DBA band
      picker steers toward spectrally flat (noise-like) bands, avoiding
      bands containing a dominant tone where raising the mask costs quality
- [x] Multichannel encode — 1/0, 2/0, 3/0, 2/2, 3/2, and 3/2 + LFE
      (the canonical 5.1 layout: L,C,R,Ls,Rs,LFE) with per-acmod BSI
      emission, LFE exponent + bap + mantissa pipeline (§5.4.3.23
      / §5.4.3.29 / §5.4.3.42-43), and ffmpeg cross-decode validation.
      LFE spectrally constrained to 0–120 Hz per §7.1.3 (round 30):
      MDCT bins ≥ 2 are zeroed before exponent extraction; `LFE_END_MANT=7`
      is unchanged for bitstream compatibility
- [x] Spec-§8.2.2 transient detector — 4th-order Butterworth 8 kHz
      cascaded-biquad HPF + hierarchical 3-level peak-ratio test
      (T₁=0.1 / T₂=0.075 / T₃=0.05); per-channel state. Replaces the
      earlier first-difference + 4× energy-ratio detector that
      mis-fired on low-frequency pure tones (round 24 / task #103).
- [x] Per-channel `fsnroffst[ch]` tuning (§5.4.3.40) — greedy bumps
      after the global `(csnr, fsnr)` selection so individual fbw
      channels can spend residual budget bits matching their mask
      headroom. Bitstream syntax always allowed it; the encoder now
      uses it.
- [x] Per-channel exponent strategy selection (§7.1.3 / §5.4.3.22,
      round 28 + 29) — encoder anchor blocks (block 0 / 3) pick
      D15 (grpsize=1), D25 (grpsize=2), or D45 (grpsize=4) per channel
      based on the smoothness of the exponent envelope. Smooth-spectrum
      bass / mid-band channels emit D25 or D45 instead of D15, shrinking
      the per-channel exponent payload (D45 = `4 + 7 × ((end-1+9)/12)`
      vs D15 = `4 + 7 × ((end-1)/3)`). With end_mant=253 D45 saves
      **~430 bits/channel/anchor block** over D15 that the snr-offset
      tuner reinvests in mantissa precision. Round 29 unblocked D45 by
      capping the dba-segment search at band 31 (the 5-bit `deltoffst`
      field range per §5.4.3.51) — previously the search reached up to
      band 44 and the wire write silently truncated, mis-targeting the
      mask delta on the decoder side. `AC3_DISABLE_D45=1` falls back to
      D25-only for A/B sweeps. ffmpeg cross-decodes both D25 and D45
      streams cleanly.
- [x] Per-block SNR-offset bit-pool tuning (§5.4.3.37-43, round 26 /
      task #170) — encoder runs a redistribution pass after the global
      tuner that moves mantissa bits between blocks based on per-block
      masking demand, emitting `snroffste=1` on the boundary block
      when the redistribution fits the budget. On a 96 kbps stereo
      fixture with a HF-rich chord burst on block 3 of each frame,
      block-3 PSNR rises from **31.84 dB** (flat allocation) to
      **32.91 dB** (per-block tuned) at matched bitstream size
      (+1.07 dB). When the demand spread is small or the budget is
      tight the pass is a no-op and the bitstream stays
      byte-identical to the previous encoder.
- [x] **Bitstream → WAV channel reorder** (round 6 / r6) — multichannel
      decoder output now lands in `WAVE_FORMAT_EXTENSIBLE` `dwChannelMask`
      order `(FL, FR, FC, LFE, BL, BR)` instead of the bitstream's
      `acmod` slot order `(L, C, R, Ls, Rs, LFE)`. Only the
      front-center-bearing layouts (`acmod ∈ {3, 5, 7}`) need the
      permutation; mono / stereo / 2/1 / 2/2 paths short-circuit.
      Applied on the passthrough path of both AC-3 and E-AC-3 decoders;
      downmix outputs skip the reorder because the matrix already emits
      in standard order. Boost on `ac3-3-0-48000`: **10.56 dB → 88.99 dB**
      PSNR vs FFmpeg `pcm_s16le`. Boost on `ac3-3-2-lfe-48000-448kbps`
      (5.1): **11.97 dB → 90.42 dB**.
- [x] **Narrow-coupling validity envelope per §5.4.3.12** (round 7 / r7).
      The audblk parser used to reject any block whose `cplbegf > cplendf`
      with `malformed coupling range`. The spec's actual envelope is
      `cplbegf <= cplendf+2` (since the upper sub-band index is
      `cplendf+2` per §5.4.3.12) — equivalently `ncplsubnd = 3 + cplendf -
      cplbegf >= 1`. ffmpeg picks narrow configs like
      `(cplbegf=11, cplendf=10)` (sub-bands 11..=12, transform-coefficient
      bins 169..193) on 5.0 (acmod=7, lfeon=0) frames; the strict check
      bombed every block-0 of every frame, the catch in `decode_frame`
      zeroed the coefficients, and the bit cursor drifted from there.
      Boost on `ac3-3-2-48000-384kbps` (5.0): **6.49 dB → 88.85 dB**
      PSNR (+82.36 dB).
- [ ] Downmix (§7.8) — 3/2 and 3/1 modes still pending
- [x] E-AC-3 (bsid=16, Annex E) — encoder. Independent substream
      (`strmtyp=0`, `substreamid=0`) for 1.0/2.0/5.1 layouts (acmod
      ∈ {1, 2, 7}, with `lfeon=1` for 5.1). 7.1 input emits an
      indep+dep substream pair (round 27 / task #187): the indep
      substream carries the 5.1 program (acmod=7, lfeon=1); the
      dep substream (`strmtyp=1`, `substreamid=0`, acmod=2) carries
      the back-surround pair Lb/Rb with `chanmape=1` and `chanmap`
      bit 6 (`Lrs/Rrs pair`, Table E2.5) set. Per ATSC A/52 §E.3.8.2
      a reference 5.1 decoder ignores the dep substream and reads
      only indep substream 0 — extended decoders that honour the
      chanmap field reassemble all 8 channels. 6 blocks per
      syncframe (`numblkscod=3`), no coupling, no spectral
      extension, no Adaptive Hybrid Transform. Task #467 corrected
      the audfrm-vs-audblk placement of `chexpstr[blk][ch]` /
      `cplexpstr[blk]` / `lfeexpstr[blk]` (audfrm per ETSI §E.1.2.3
      / Table E.1.3, gated by `expstre`), restored the per-channel
      `gainrng` (2 bits) emit in audblk, and added the unconditional
      `convsnroffste` bit when `strmtyp == 0` — ffmpeg now decodes
      every output cleanly at PSNR **20.21 dB** (mono 96k / stereo
      192k) and reconstructs the full 8-channel program for the 7.1
      indep+dep pair. Codec id = `"eac3"`.
- [x] E-AC-3 adaptive exponent strategy (round 30) — encoder now calls
      `select_exp_strategies` per-channel on each anchor block (0/3),
      replacing the static D15-only pattern with D15/D25/D45 chosen from
      spectral smoothness, matching the AC-3 encoder's strategy. D45
      saves ~430 bits/channel/anchor-block that the SNR-offset tuner
      redirects to mantissa precision. `EAC3_DISABLE_EXPSTR_SEL=1`
      reverts to static D15 for A/B testing.
- [x] **E-AC-3 frame-based exponent strategy** (`expstre == 0`,
      round 72). The audfrm parser now expands the 5-bit
      `frmcplexpstr` + per-channel `frmchexpstr[ch]` codewords (and
      `convexpstr[ch]` on indep substreams) via **Table E2.10** into the
      32 spec-defined `[D15/D25/D45/REUSE] × 6` per-block strategy
      runs. The audblk dsp consumes the expanded `chexpstr_blk_ch[blk][ch]`
      / `cplexpstr_blk[blk]` arrays unchanged from the `expstre == 1`
      path — Table E2.10 also stocks `cplexpstr_blk[]` on blocks where
      coupling is in use; entries for non-cplinu blocks are harmlessly
      left at the lookup value (the dsp gates them on `cplinu_blk[blk]`).
      Also widens the E-AC-3 coupling validity check to the §5.4.3.12
      envelope (`cplbegf <= cplendf+2`, equivalently `ncplsubnd >= 1`)
      so FFmpeg's narrow-coupling configs (e.g. `(cplbegf=11,
      cplendf=10)` on 5.0 frames) no longer trip `malformed coupling
      range`. Corpus deltas vs round-6 baseline (all FFmpeg-encoded
      fixtures use `expstre == 0` so were silent before):
      `eac3-5.1-48000-384kbps` **13.57 → 90.01 dB** (+76.4 dB),
      `eac3-low-rate-stereo-64kbps` **13.57 → 71.74 dB** (+58.2 dB),
      `eac3-low-bitrate-32kbps` **13.57 → 66.32 dB** (+52.7 dB),
      `eac3-5.1-side-768kbps` **13.57 → 21.32 dB** (+7.7 dB; remaining
      ceiling is SPX-blocked frames muting and bleeding into the
      overlap-add delay line). Stereo-192k / 256-coeff-block / from-ac3
      fixtures hit SPX-active blocks in mid-frame and stay near the
      silent floor (SPX decode is the next blocker).
- [x] E-AC-3 decoder — **round 1** (task #285): full BSI parser
      (Table E1.2) covering strmtyp / substreamid / frmsiz / fscod
      / fscod2 / numblkscod / acmod / lfeon / bsid / dialnorm /
      chanmape+chanmap / mixmdate / infomdate / addbsi; full audfrm
      parser (Table E1.3) covering the 11 strategy flags +
      coupling-block run + frame-level exponent strategies +
      converter exponents + frame SNR offsets + transient pre-noise
      params + spectral-extension attenuation + per-block-start
      info. Top-level dispatch in the AC-3 decoder routes packets
      with `bsid > 10` to the Annex E path. Round-1 PCM output is
      silent (zero S16) of the correct shape (`num_blocks × 256 ×
      nchans`); real DSP (decouple + IMDCT + overlap-add) is
      deferred to round 2 along with dependent-substream
      recombination, AHT, and spectral extension.

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-codec = "0.1"
oxideav-ac3 = "0.0"
```

## Codec ID

- Codec: `"ac3"` (decoder + encoder) and `"eac3"` (decoder + encoder);
  output sample format `S16` interleaved.

## License

MIT — see [LICENSE](LICENSE).
