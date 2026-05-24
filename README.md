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
- [x] Multichannel encode — 1/0, 2/0, **2/0 + LFE (2.1)**, 3/0, 2/2,
      3/2, and 3/2 + LFE (the canonical 5.1 layout: L,C,R,Ls,Rs,LFE)
      with per-acmod BSI emission, LFE exponent + bap + mantissa
      pipeline (§5.4.3.23 / §5.4.3.29 / §5.4.3.42-43), and ffmpeg
      cross-decode validation. The 2.1 layout (round 78 / r78) is
      reached by setting `CodecParameters.channel_layout =
      Some(ChannelLayout::Stereo21)` on a 3-channel input — without
      the explicit layout, 3 channels default to acmod=3 (3/0 = L,C,R).
      ffmpeg cross-decodes our 2.1 output at within 0.2% per-channel
      RMS of the input (`two_one_lfe_ffmpeg_crossdecode`). LFE
      spectrally constrained to 0–120 Hz per §7.1.3 (round 30):
      MDCT bins ≥ 2 are zeroed before exponent extraction;
      `LFE_END_MANT=7` is unchanged for bitstream compatibility.
      Round 91 added a self-decode roundtrip for the previously
      untested 2/2 (acmod=6 = L,R,Ls,Rs, 4 fbw channels) path plus
      per-channel PSNR-floored regression tests for 2/2 (4ch),
      3/2 (5ch), and 5.1 (6ch) layouts — each fbw slot is asserted
      `>= 10 dB` PSNR vs the source PCM after a per-channel lag
      search (1024-sample correlator, ±2048-sample window). Headline
      figures on the synthesised 220×n Hz multitone fixture: 2/2
      24-32 dB per ch (320 kbps default), 5.0 10-33 dB per ch
      (384 kbps default), 5.1 10-33 dB per ch (448 kbps default)
- [x] Spec-§8.2.2 transient detector — 4th-order Butterworth 8 kHz
      cascaded-biquad HPF + hierarchical 3-level peak-ratio test
      (T₁=0.1 / T₂=0.075 / T₃=0.05); per-channel state. Replaces the
      earlier first-difference + 4× energy-ratio detector that
      mis-fired on low-frequency pure tones (round 24 / task #103).
- [x] Per-channel `fsnroffst[ch]` tuning (§5.4.3.40) — greedy bumps
      after the global `(csnr, fsnr)` selection so individual fbw
      channels can spend residual budget bits matching their mask
      headroom. Bitstream syntax always allowed it; the encoder now
      uses it. **Round 95** retired the round-23 index-order
      round-robin in favour of a two-stage **equalise + spread-cap**
      greedy: an equalisation pass bumps minimum-`fsnroffst_ch`
      channels until none fit, then a residual pass spreads any
      remaining slack subject to `max(fsnroffst_ch) - min(...) ≤ 2`.
      Closes the long-standing imbalance where a low-frequency tone
      on slot 0 (cheap per-bump mantissa cost) ran away to
      `fsnroffst_ch=15` while higher-frequency siblings stayed at
      the global baseline. Encoder-policy only — §5.4.3.40 defines
      the wire field, the choice of value is non-normative
      (`encoder::tune_snroffst_per_channel_spread_bounded`).
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
- [x] Downmix (§7.8) — LoRo 2-channel, LtRt 2-channel
      (Dolby Surround matrix-encoded — round 120 / r120), and mono
      target paths cover every source acmod (1/0 / 2/0 / 3/0 / 2/1 /
      3/1 / 2/2 / 3/2 / 1+1 dual-mono). LtRt implements §7.8.2's
      3/2 form `Lt = L + 0.707·C − 0.707·Ls − 0.707·Rs` /
      `Rt = R + 0.707·C + 0.707·Ls + 0.707·Rs` (plus 3/1's
      single-surround variant and the 2/1 / 2/2 C-drop case),
      normalised by Table 7.32's 0.3204 / 0.2265 coefficients
      (1/3.121 worst-case). Selected via the new
      `decoder::make_decoder_ltrt` factory (`make_decoder` keeps
      LoRo, matching FFmpeg's default). On a surround-only 5.1 AC-3
      source the LoRo L/R correlation lands at +0.002 (uncorrelated
      independent surround tones summed in-phase) while LtRt lands
      at **−0.972** — the matrix encoder's defining anti-phase
      surround signature, recoverable by a downstream Pro Logic
      decoder. Annex D §2.3.1.3-4 (`ltrtcmixlev` / `ltrtsurmixlev`,
      E-AC-3 metadata override for the fixed 0.707 coefficients) is
      a documented followup; base-spec §7.8.2 uses the fixed form
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
- [x] E-AC-3 decoder — **spectral extension (SPX)** decode (round 100 /
      r100). The audblk parser decodes the full §E.2.3.3 SPX strategy +
      coordinate syntax (`chinspx` / `spxstrtf` / `spxbegf` / `spxendf` /
      `spxbndstrce`+`spxbndstrc[]` with Table E2.11 default banding /
      `spxcoe` / `spxblnd` / `mstrspxco` / `spxcoexp` / `spxcomant`),
      replacing the round-4 `spxinu == 1` mute. The §E.3.6 high-frequency
      regeneration runs in `audblk::dsp_block`: per SPX band it (1) copies
      low-frequency transform coefficients into the SPX region with the
      §E.3.6.4.1 wrapping copy cursor, (2) measures banded RMS energy,
      (3) blends the copies with banded noise via the spxblnd-derived
      `nblendfact`/`sblendfact` (§E.3.6.4.2), and (4) scales by the
      per-band coordinate `spxco·32` (§E.3.6.4.3); SPX-channel `endmant`
      is then extended to the SPX end so dynrng + IMDCT cover the
      regenerated bins. Three derivations that previously drifted the bit
      cursor on SPX frames are now spec-correct: `endmant[ch] =
      spxbandtable[spx_begin_subbnd]` (§E.3.3.3), `cplendf` derived from
      `spxbegf` when SPX is active (§E.3.3.1), and `nrematbd` folding in
      SPX (§E.3.3.2). The noise generator is non-normative per spec
      ("any reasonably random sequence"); a deterministic xorshift LFSR
      keeps decodes reproducible. Also fixes the E-AC-3 D25 exponent-group
      count to the spec's `(endmant−1+3)/6` (§7.1.3) — it used
      `(endmant−1).div_ceil(6)`, over-counting one group when
      `(endmant−1) mod 6 ∈ {2,3}`, which read an extra 7-bit exponent
      word. SPX synthesis math is covered by 5 unit tests
      (`audblk::spx_tests`). Note: the three SPX-active corpus fixtures
      (`eac3-stereo-48000-192kbps`, `eac3-256-coeff-block`,
      `eac3-from-ac3-bitstream-recombination`) remain floor-bound on a
      pre-existing bit-allocation cursor drift affecting a subset of their
      non-SPX frames — the same drift that mutes a few AC-3 fixtures
      (`ac3-32000hz-stereo`) — so end-to-end PSNR there awaits that
      separate fix. For `ac3-32000hz-stereo` specifically, the first
      syncframe decodes bit-exactly (≤2 LSB) but the second over-reads
      mantissas — the bap array for its D15/full-bandwidth/no-coupling
      blocks comes out more generous than the reference encoder budgeted,
      so blocks explode then exhaust the frame. Pinpointing it needs a
      per-block reference, which the shipped `trace.txt` cannot provide
      (it was captured from a 12-frame stream while `input.ac3` carries
      11 frames — see followups).
- [x] E-AC-3 decoder — **transient pre-noise processing (TPNP)** decode
      (round 103 / r103). Implements the §E.3.7.2 PCM-domain time-scaling
      synthesis, replacing the round-2 whole-frame reject that errored any
      syncframe with `transproce == 1`. The audfrm parser now stores the
      per-fbw-channel `chintransproc` / `transprocloc` (4-sample units) /
      `transproclen` fields; after overlap-add, `apply_transient_prenoise`
      reconstructs the pre-transient region for each TPNP channel from a
      `2·TC1 + pnlen`-sample synthesis buffer copied from earlier audio
      and cross-fades it over the noisy original (fade window `TC1 = 256`,
      overwrite middle, fade window `TC2 = 128`; complementary Hann
      windows per §E.3.7.2's "any constant-amplitude cross-fade pair").
      LFE never carries TPNP and the baseband decode is unchanged — TPNP
      is a quality enhancement on already-valid samples. The §E.3.7.1
      cross-frame case (a frame-N transient referencing frame-(N-1) tail)
      is clamped to the current frame (single-frame conservative path);
      no corpus fixture carries `transproce == 1`, so the synthesis math
      is covered by 4 unit tests (`eac3::dsp::tpnp_tests`) rather than an
      end-to-end PSNR gate.
- [x] E-AC-3 decoder — **Adaptive Hybrid Transform (AHT)** decode,
      multichannel full-bandwidth + LFE + coupling (round 6 mono →
      round 110 fbw → round 113 LFE → round 117 coupling / r117). The
      §3.4 AHT path front-loads 6×N high-efficiency mantissas (VQ Tables
      E4.1-E4.7 for `1 ≤ hebap ≤ 7`, scalar/GAQ for `hebap ≥ 8`),
      inverse DCT-II's per bin (§3.4.5), and caches the per-block
      coefficients for the standard IMDCT + overlap-add. Round 110 lifts
      the round-6 mono-only restriction: the §3.4.2 helper variables
      `nchregs[ch]` / `ncplregs` / `nlferegs` are derived directly from
      the already-parsed per-block exponent strategies (no audblk
      pre-walk), so every fbw channel with `nchregs[ch] == 1` decodes via
      AHT. Non-AHT channels in a mixed frame share the §7.3.5 bap-1/2/4
      grouping buffers across channels (round 6's per-channel grouping
      was correct only for mono). Round 113 wires the LFE channel into
      the AHT mantissa path: the `lfeahtinu == 1` LFE-AHT block
      (`lfegaqmod` + gains + 6×7 mantissas + IDCT) decodes, and the
      previously-skipped *standard* LFE mantissas (`lfeahtinu == 0`) are
      now read — fixing a latent bit-cursor desync that hit any AHT frame
      carrying an LFE channel. **Round 117** wires the coupling
      pseudo-channel: the `cplahtinu == 1` coupling-AHT block
      (`cplgaqmod` + gains + 6×ncplmant VQ/GAQ mantissas + IDCT) is read
      interleaved with the first coupled fbw channel — gated by
      `got_cplchan` exactly as the base-AC-3 mantissa loop (Table E1.4) —
      over the coupling range `[cpl_begf_mant, cpl_endf_mant)`, and its
      per-block coefficients are loaded into the coupling pseudo-channel
      slot before the §7.4 decouple step scatters them into the fbw
      channels via the cplco coordinates. The standard
      (`cplahtinu == 0`) coupling read in an AHT frame is also wired at
      that interleave point (it was previously skipped — only the blanket
      reject saved the cursor). No AHT flag is rejected by the decoder
      any more. Coupling/LFE AHT have no corpus fixture (FFmpeg's eac3
      encoder emits neither), so the synthesis math is covered by unit
      tests (`eac3::dsp::cpl_aht_tests`, 3 tests).

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
