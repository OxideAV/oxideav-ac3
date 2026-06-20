# oxideav-ac3

Pure-Rust **AC-3 (Dolby Digital)** + **E-AC-3 (Enhanced AC-3 / Dolby
Digital Plus)** audio decoder + encoder — elementary streams per
ATSC A/52:2018 (= ETSI TS 102 366). Zero C dependencies.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Architecture

The pipeline follows the spec's natural ordering; each module owns one
slice of §5..§7 (base AC-3) or §E (E-AC-3):

1. [`syncinfo`] — sync word 0x0B77, crc1, fscod, frmsizecod,
   frame-length lookup (§5.3.1 / §5.4.1 / Table 5.18).
2. [`bsi`] — Bit Stream Information: bsid, bsmod, acmod → channel layout
   + lfeon + dialnorm + the optional timecode / Annex D alternate-syntax
   metadata blocks (§5.4.2).
3. [`audblk`] — per-block exponent decode (§7.1), parametric bit
   allocation (§7.2 with §7.2.2.6 delta-bit-allocation), mantissa decode
   (§7.3), channel coupling (§7.4), rematrixing (§7.5), dynamic-range
   compression (§7.7).
4. [`imdct`] + [`mdct`] — §7.9.4 FFT-backed 512-point IMDCT and
   256-point short-block pair, plus the forward transforms the encoder
   uses.
5. [`downmix`] — §7.8 LoRo + §7.8.2 LtRt downmix matrices for every
   source acmod, with Annex D / E-AC-3 mix-level extension routing.
6. [`wave_order`] — channel reorder for front-centre-bearing layouts
   (`acmod ∈ {3, 5, 7}`).
7. [`encoder`] — base AC-3 encoder.
8. [`eac3`] — Annex E decoder + encoder.
9. [`crc`] — §7.10.1 CRC-16 over poly 0x8005, shared between the encoder
   and the opt-in `decoder::verify_packet_crc` residue check.

## Capabilities

### AC-3 decoder

- Sync frame + BSI parse (§5.3 / §5.4). All §5.4.2 metadata words —
  bit-stream mode, compression gain, dialogue normalisation, mix
  levels, Dolby Surround mode, timecodes, copyright/original flags,
  language code, audio-production info, the Annex D alternate-syntax
  informational blocks, and the `addbsi` trailer — are parsed and
  surfaced as typed accessors (advisory metadata; the PCM path is
  unchanged).
- Audio-block parse (§5.4.3), exponent decode (§7.1) + parametric bit
  allocation (§7.2), mantissa decode (§7.3) with bap=0 dither (§7.3.4),
  delta bit allocation (§7.2.2.6).
- IMDCT synthesis (§7.9) — both 512-point long-block and 256-point
  short-block paths.
- Channel coupling (§7.4) + rematrix (§7.5) + dynrng (§7.7).
- Downmix (§7.8) — LoRo and LtRt 2-channel.
- Bitstream → WAV channel reorder for multichannel layouts.

### AC-3 encoder

- Multichannel encode — 1/0, 2/0, 2/0+LFE (2.1), 3/0, 2/2, 3/2, 3/2.1
  (5.1) and other acmod layouts, with per-channel D15/D25/D45 exponent
  strategy selection (§7.1.3), 5-fbw channel coupling within the
  §5.4.3.12 narrow-coupling validity envelope, a §8.2.2 transient
  detector (4th-order Butterworth 8 kHz split for short-block
  switching), per-channel `fsnroffst[ch]` tuning (§5.4.3.40), per-block
  SNR-offset bit-pool redistribution, and §7.10.1 dual-CRC emission.

### E-AC-3 (Annex E)

- Decoder — BSI, audfrm (Tables E1.2 / E1.3), audblk DSP, the §3.4
  Adaptive Hybrid Transform on fbw / LFE / coupling channels, §3.6
  spectral extension with the §3.6.4.2.3 SPXATTEN border notch, and
  §3.7.2 transient pre-noise processing. Enhanced coupling
  (`ecplinu == 1`, §E.2.3.3.16-26 / §E.3.5.5) decodes end-to-end: the
  audblk parser reads the strategy + per-channel amplitude/angle/chaos
  coordinates, decodes the enhanced-coupling channel through the shared
  exponent / bit-allocation / mantissa path, and a deferred second pass
  reconstructs the non-aliased complex carrier `Z[k]` from the
  previous / current / next blocks (§E.3.5.5.1), processes the per-bin
  amplitudes + de-correlated angles, and emits each coupled channel's
  transform coefficients via the §E.3.5.5.4 complex product — replacing
  the standard §7.4 decouple. Block 0's "previous block" carrier source
  is threaded across the frame boundary from the prior frame's last
  enhanced-coupling block (carried on `EcplState`, §E.3.5.5.1), so the
  prior-frame edge no longer collapses to a zero carrier; the frame's
  last block's "next block" still uses a zero carrier (it lives in a
  not-yet-decoded frame — streaming lookahead is out of scope). The
  §E.3.3.2 `nrematbd` derivation now folds in enhanced coupling: a 2/0
  `ecplinu` block sizes its rematrix-flag field from the raw `ecplbegf`
  code (0/1/2/<5 → 0/1/2/3 bands, else 4) rather than `cplbegf`, keeping
  the bit cursor aligned on enhanced-coupling 2/0 frames. Standard coupling
  now applies the §E.2.3.3.15 **default coupling banding structure**
  (`defcplbndstrc[]`, Table E2.12, indexed by absolute sub-band) when
  `cplbndstrce == 0` in a frame's first coupling block, instead of leaving
  every sub-band un-merged — the prior all-zeros behaviour collapsed a
  7-subband region to 7 bands instead of 3, corrupting the §7.4
  coupling-coordinate scatter on every basic stereo-coupled frame. Three
  corpus stereo fixtures (`eac3-stereo-48000-192kbps`, `eac3-256-coeff-block`,
  `eac3-from-ac3-bitstream-recombination`) jumped from ~8-14 dB to ~91 dB
  PSNR and are now CI-gated at an 80 dB floor (`Tier::MinPsnr` in
  `tests/docs_corpus.rs`).
- Encoder — independent + dependent substream pairs for 1.0 / 2.0 / 5.1
  / 7.1 layouts, with adaptive / frame-based exponent strategies. SPX
  and AHT are out of scope on the encoder side.

### CRC

§7.10.1 CRC-16 (poly 0x8005), shared between the encoder (forward
generation, augmented form for crc2) and the opt-in decoder residue
check.

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-codec = "0.1"
oxideav-ac3 = "0.0"
```

## Codec ID

- Codecs: `"ac3"` (decoder + encoder) and `"eac3"` (decoder + encoder);
  output sample format `S16` interleaved.

## License

MIT — see [LICENSE](LICENSE).
