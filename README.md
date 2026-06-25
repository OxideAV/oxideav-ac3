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
10. [`drc`] — §6.1.9 / §7.6 / §7.7 dynamic-range-control + dialogue-
    normalisation control surface (`DrcSettings`: partial-compression
    cut/boost, heavy-compression "RF mode", dialnorm playback target).

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
- **Dynamic-range-control + dialogue-normalisation control surface**
  (§6.1.9 / §7.6 / §7.7, [`drc`]). The mandatory §7.7.1 full-`dynrng`
  decode is the default ("line out"); a listener-facing
  [`DrcSettings`] steers the §7.7.1.2 *partial-compression* cut/boost
  factors (apply a fraction of each gain reduction / increase, with
  independent directions), §7.7.2 *heavy compression* ("RF mode" —
  substitutes the BSI `compr` word, ±48 dB, falling back to `dynrng`
  when a frame carries no `compr` per §7.7.2.1), and §7.6 dialogue
  normalisation (an opt-in playback scalar `10^((target − dialnorm)/20)`
  toward a chosen headroom target). Build a configured decoder with
  `decoder::make_decoder_with_drc(params, DrcSettings)`. Applies to both
  the AC-3 and E-AC-3 paths.
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
  §3.7.2 transient pre-noise processing. All three §2.3.2.3 SNR-offset
  strategies decode: the frame-level `snroffststr == 0` pair plus the
  §2.3.3.27 per-block modes `0x1` (one shared `blkfsnroffst`) and `0x2`
  (independent per-coupling/channel/LFE fine offsets), each gated by the
  per-block `snroffste` reuse flag. Enhanced coupling
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

## Conformance corpus

`tests/docs_corpus.rs` decodes the AC-3 / E-AC-3 fixture set under
`docs/audio/ac3/fixtures/` (each a raw elementary stream paired with a
reference PCM decode) and scores per-channel PSNR. The decode is
floating-point in the IMDCT, so it is not bit-exact against the
reference, but it is **deterministic** (identical PSNR run-to-run).

Fixtures whose decode is known-good are gated at a `Tier::MinPsnr`
floor so a regression fails CI; the rest log deltas without gating:

| Tier | AC-3 | E-AC-3 |
| --- | --- | --- |
| `MinPsnr` (CI-gated) | 10 fixtures — mono / stereo / 2/1 / 3/0 / 3/2 (±LFE) at 48 / 44.1 / 32 kHz, 96-448 kbps, ~86-92 dB (80 dB floor, 96 kbps mono at 78) | 4 fixtures — stereo + 5.1 + 256-coeff, ~91 dB (80 dB floor) |
| `ReportOnly` | `ac3-low-bitrate-32kbps-mono` (~62 dB — the 32 kbps lossy floor, residual confined to the tone attack/release blocks) | the torture-grade low-rate `eac3-low-bitrate-32kbps` (~66 dB) and `eac3-low-rate-stereo-64kbps` (~72 dB) |

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
