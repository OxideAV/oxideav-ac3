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
- [x] Delta bit allocation (§7.2.2.6) — encoder + decoder
- [x] Multichannel encode — 1/0, 2/0, 3/0, 2/2, 3/2, and 3/2 + LFE
      (the canonical 5.1 layout: L,C,R,Ls,Rs,LFE) with per-acmod BSI
      emission, LFE exponent + bap + mantissa pipeline (§5.4.3.23
      / §5.4.3.29 / §5.4.3.42-43), and ffmpeg cross-decode validation
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
      round 28) — encoder anchor blocks (block 0 / 3) now pick
      D15 (grpsize=1) or D25 (grpsize=2) per channel based on the
      smoothness of the exponent envelope. Smooth-spectrum bass /
      mid-band channels emit D25 instead of D15, halving the per-channel
      exponent payload (`4 + 7 × ((end-1+3)/6)` bits vs `4 + 7 × ((end-1)/3)`).
      With end_mant=253 the saving is **~290 bits/channel/anchor block**
      that the snr-offset tuner reinvests in mantissa precision. The
      decoder side already supported all three "new" strategies; this
      round wires the encoder to actually emit them when the spectrum
      allows. ffmpeg cross-decodes the D25-bearing stream cleanly. D45
      (grpsize=4) emission is gated behind `AC3_ENABLE_D45=1` —
      first-frame mantissa-stream desync follow-up.
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
      extension, no Adaptive Hybrid Transform. Cross-decodes
      cleanly through ffmpeg. Codec id = `"eac3"`.
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
