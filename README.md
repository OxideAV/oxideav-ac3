# oxideav-ac3

Pure-Rust **AC-3 (Dolby Digital)** audio decoder + encoder — elementary
streams per ATSC A/52:2018 (= ETSI TS 102 366). Zero C dependencies.

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
- [x] Channel coupling (§7.4) + rematrix (§7.5) + dynrng (§7.7)
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
- [ ] Downmix (§7.8) — 3/2 and 3/1 modes still pending
- [ ] E-AC-3 (bsid=16, Annex E) — separate crate, not in scope here

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-codec = "0.1"
oxideav-ac3 = "0.0"
```

## Codec ID

- Codec: `"ac3"`; output sample format `S16` interleaved.

## License

MIT — see [LICENSE](LICENSE).
