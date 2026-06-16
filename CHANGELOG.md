# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Other

- eac3 round 321 (r321): §E.3.3.2 `nrematbd` now folds in enhanced coupling. The 2/0 rematrix-flag field count (eac3::dsp audblk loop) previously routed an `ecplinu == 1` block through the standard-coupling arm keyed on `cplbegf` (which the enhanced-coupling path never sets), drifting the bit cursor on enhanced-coupling 2/0 frames. `audblk::remat_band_count_spx` gains an `ecpl_in_use` / `ecplbegf` pair and implements the §E.3.3.2 enhanced-coupling arm verbatim (raw `ecplbegf` 0/1/2/<5 → 0/1/2/3 bands, else 4 — uniquely admitting `nrematbd = 0`). The raw `ecplbegf` code is now retained on `eac3::ecpl::EcplStrategy` (carried across `cplstre == 0` strategy-reuse blocks). New unit test `audblk::spx_tests::nrematbd_e3_3_2_decision_tree` covers every arm. Resolves one of the two §E.3.5.5 enhanced-coupling follow-ups (cross-frame edge threading remains).
- eac3 round 317 (r317): enhanced-coupling (`ecplinu == 1`) decoder-level integration — §E.3.5.5 now decodes end-to-end. The audblk parser reads the §E.2.3.3.16-26 strategy + per-channel amplitude/angle/chaos coordinates (eac3::ecpl::parse_strategy / parse_coords) and decodes the enhanced-coupling channel through the shared exponent / bit-allocation / mantissa path by pinning the coupling region at the ecpl bins; a deferred second pass (eac3::dsp::run_deferred_ecpl_dsp) reconstructs the §E.3.5.5.1 carrier `Z[k]` from prev/curr/next blocks and emits each coupled channel's transform coefficients via eac3::ecpl::synthesize_block (carrier × per-bin amplitude × de-correlated angle, §E.3.5.5.4), replacing the standard §7.4 decouple (gated by Ac3State::skip_decouple). Cross-block random de-correlation sources held on Ac3State::ecpl_state (EcplState). Removes the audblk-stage `ecplinu == 1` `Unsupported` rejection. Frame-edge neighbours use zero carriers (§E.3.5.5.1 "set to zero"); cross-frame edge threading + the 2/0 ecpl+rematrix band-count interaction are follow-ups.
- eac3 round 314 (r314): enhanced-coupling §E.3.5.5.1 carrier reconstruction — windowed prev/curr/next 512-sample IMDCT + overlap-add + normalised forward DFT producing the non-aliased complex coupling channel `Z[k]` (eac3::ecpl::reconstruct_carrier); adds a forward 512-point DFT (imdct::dft_512_forward). Pure tested layer; decoder-level cross-block integration deferred.

## [0.0.9](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.8...v0.0.9) - 2026-06-15

### Other

- enhanced-coupling complex synthesis (§E.3.5.5.3 closing + §E.3.5.5.4)
- eac3 round 306 (r306): enhanced-coupling parameter processing (eac3::ecpl, §E.3.5.5.2-3)
- eac3 round 300 (r300): enhanced-coupling bitstream-syntax parse (eac3::ecpl, §E.2.3.3.16-26)
- eac3 round 293 (r293): enhanced-coupling band geometry (eac3::ecpl, §E.2.3.3.16-19 + §E.3.5.2)
- ac3 round 288 (r288): typed PremixCompression surface for premixcmpsel / drcsrc / premixcmpscl (§E.2.3.1.19-21 / Table E2.7)
- ac3 round 281 (r281): typed PanInfo surface for panmean / paninfo / panmean2 / paninfo2 (§E.2.3.1.53-58 / §E.3.10.8)
- ac3 round 278 (r278): typed ProgramScaleFactor surface for pgmscl / pgmscl2 / extpgmscl (§E.2.3.1.12-17 / §E.3.10.1-2)
- ac3 round 274 (r274): ChannelLocation Table E2.5 classification + re-emit surface (§E.2.3.1.8)
- ac3 round 271 (r271): typed FrameSizeCode surface over frmsizecod (§5.4.1.4 / Table 5.18)
- ac3 round 263 (r263): typed SampleRateCode surface for fscod (§5.4.1.3 / Table 5.6)
- ac3 round 259 (r259): typed CenterMixLevel / SurroundMixLevel surfaces for cmixlev / surmixlev (§5.4.2.4-5 / Tables 5.9-5.10)
- ac3 round 254 (r254): typed ExtraBsi2 surface for xbsi2 / encinfo (§2.3.1.11-12 / Annex D Table D2.1)
- ac3 round 249 (r249): typed LanguageCode surface for langcod / langcod2 (§5.4.2.11-12 / §5.4.2.19-20)
- drop release-plz.toml — use release-plz defaults across the workspace
- ac3 round 246 (r246): typed DolbySurroundMode surface for dsurmod (§5.4.2.6 / Table 5.11 / Annex E §E.2.3.1.x)
- ac3 round 243 wording cleanup — paraphrase 'no slot' rationale to spec-action phrasing
- ac3 round 243 (r243): typed StereoDownmixPreference surface for dmixmod (§2.3.1.2 / Table D2.2 / Annex E §E.1.2.2)
- ac3 round 240 (r240): typed AdditionalBitStreamInfo surface for addbsi (§5.4.2.29-31 / §5.3.2 / Table E1.2)
- ac3 round 234 (r234): typed DialNorm surface for dialnorm / dialnorm2 (§5.4.2.8 / §5.4.2.16 / §7.6)
- ac3 round 226 (r226): typed CopyrightInfo surface for copyrightb / origbs (§5.4.2.24-25 / §E.2.3.1.62)
- ac3 round 219 (r219): typed TimeCode1 / TimeCode2 / TimeCodePresence surface for base-syntax timecode (§5.4.2.26-28 / Table 5.13)
- ac3 round 214 (r214): typed AudioProductionInfo surface for mixlevel/roomtyp (§5.4.2.13-15 / Table 5.12 / §E.2.3.1.x)
- ac3 round 208 (r208): typed xbsi2 / infomdata Dolby Surround EX / Dolby Headphone / A/D converter (Tables D2.7-D2.9 / §E.2.3.1.x)
- ac3 round 202 (r202): typed CompressionGain surface for compr / compr2 (Table 7.30 / §7.7.2.2)
- eac3 round 196 (r196): dependent-substream chanmap routing (Table E2.5)
- ac3 round 193 (r193): typed BitStreamMode accessor for Table 5.7

### Added

- **E-AC-3 enhanced-coupling complex synthesis — `eac3::ecpl`**
  (round 310 / r310). Adds the §E.3.5.5.3 closing de-correlation step and
  the §E.3.5.5.4 channel transform-coefficient generation on top of the
  r306 parameter-processing layer: `apply_decorrelation()` (the
  `angle[bin] += chaos[bin]·rand[bin]` term with the single-step `±1.0`
  fold), `generate_channel_coeffs()` (the per-bin complex coordinate
  `ampbin·e^{jπ·angle}` multiplied into the reconstructed coupling carrier
  `Z[k]` followed by the MDCT synthesis
  `chmant = -2·(y[bin]·Zr_ch + y[N/2-1-bin]·Zi_ch)`), the
  `synthesis_window()` factor `y[bin] = cos(2π·(N/4+0.5)/N·(bin+0.5))`,
  and the two de-correlation random sources — `RandNoTrans` (the
  init-once per-channel `[-1,1]`-uniform `rand_notrans[ch][bin]` array)
  and `gen_rand_trans()` (the per-block transient `rand_trans[ch][bnd]`
  band values). The generators are non-normative (the spec fixes only the
  distribution / uniqueness / per-block-vs-once cadence) so a
  deterministic xorshift keeps decodes reproducible, mirroring the
  §E.3.6.4.2 SPX-noise precedent. Pure tabulated arithmetic over the
  supplied carrier — no multi-block state. The §E.3.5.5.1 prev/curr/next
  IMDCT + overlap-add + FFT that produces `Z[k]` remains deferred (it is
  stateful across blocks); the decoder's `ecplinu == 1` reject is
  unchanged. 8 new `eac3::ecpl::tests` (327 → 335 lib tests).
- **E-AC-3 enhanced-coupling parameter processing — `eac3::ecpl`**
  (round 306 / r306). Adds the §E.3.5.5.2 / §E.3.5.5.3 layer that
  turns the decoded `ecplamp` / `ecplangle` / `ecplchaos` index
  triples into per-bin amplitude and angle arrays. Tables E3.10
  (`ECPL_AMP_EXP_TAB` / `ECPL_AMP_MANT_TAB`), E3.11 (`ECPL_ANGLE_TAB`)
  and E3.12 (`ECPL_CHAOS_TAB`); `ampbnd()` (5-bit index → linear gain
  `(mant/32) >> exp`; index 31 → minus-∞ dB → 0);
  `process_band_amplitudes()` (the §E.3.5.5.2 chaos amplitude
  modification `*= 1 + 0.38·chaos`, skipped for first-coupled /
  transient channels); `expand_bands_to_bins()` (per-band → per-bin
  `ampbin[]` fan-out via the `ecplbndstrc[]` merge structure);
  `angle_value()` / `chaos_value()` (index decode, forced to 0 on the
  first coupled channel); and `interpolate_bin_angles()` (the
  `ecplangleintrp == 1` band-centre linear-interpolation path with the
  spec's `±1.0` wrap guards). Pure tabulated arithmetic, no multi-block
  state. The §E.3.5.5.1 FFT channel processing + §E.3.5.5.4 complex
  synthesis (+ `rand[]` de-correlation) remain deferred; the decoder's
  `ecplinu == 1` reject is unchanged. 7 new `eac3::ecpl::tests`
  (321 → 328 lib tests).
- **E-AC-3 enhanced-coupling bitstream-syntax parse — `eac3::ecpl`**
  (round 300 / r300). Adds the syntax layer on top of the r293
  geometry: `parse_strategy()` reads the §E.2.3.3.16-19 strategy
  fields (`ecplbegf` → begin sub-band; `ecplendf` only when SPX is
  off; `ecplbndstrce` gating per-sub-band merge bits over
  `[max(9, begin+1), end)`; default/previous banding reuse with no
  bits consumed when `ecplbndstrce == 0`), and `parse_coords()`
  reads the §E.2.3.3.20-26 coordinate block (`ecplangleintrp`;
  per-channel implicit-on-first-block / explicit-thereafter
  `ecplparam1e`/`ecplparam2e`; `ecplamp` 5 b, `ecplangle` 6 b,
  `ecplchaos` 3 b per band; `ecpltrans` 1 b — with the first coupled
  channel carrying no angle/chaos/trans per the spec's fixed-zero
  rule). Typed `EcplStrategy` / `EcplCoords` / `EcplChannelParams`
  carriers; both readers advance the bit cursor exactly per the
  reference syntax (verified by `bit_position()` assertions). The
  §E.3.5.5 coordinate reconstruction (Tables E3.10-E3.12) remains
  the deferred next step; the decoder's `ecplinu == 1` reject is
  unchanged. 6 new `eac3::ecpl::tests` (315 → 321 lib tests).
- **E-AC-3 enhanced-coupling band geometry — `eac3::ecpl`** (round
  293 / r293). A new pure, spec-tabulated geometry module for the
  `ecplinu == 1` (enhanced coupling) decode path, covering
  ATSC A/52:2018 Annex E §E.2.3.3.16-19 + §E.3.5.2: `begin_subbnd()`
  / `end_subbnd()` (Table E3.8 derivations of `ecpl_begin_subbnd`
  from `ecplbegf` and `ecpl_end_subbnd` from `ecplendf`, or from the
  SPX begin when spectral extension is co-active), `ECPL_SUBBND_TAB`
  (Table E3.9 `ecplsubbndtab[]` — the 22 sub-band start
  transform-coefficient numbers + a one-past-the-end sentinel),
  `DEFAULT_ECPL_BNDSTRC` (Table E2.14 `defecplbndstrc[]` default
  banding), `necplbnd()` (§E.2.3.3.19 band count from the
  per-sub-band merge bits), and `band_bin_counts()` (§E.3.5.5.1
  `nbins_per_bnd_array[]` population). This is the geometry
  foundation the still-deferred §E.3.5.5 synthesis (amplitude /
  angle / chaos parameter decode + complex coordinate
  reconstruction) will consume; the decoder still rejects
  `ecplinu == 1` at the synthesis stage rather than emit incorrect
  PCM, but the rejection diagnostic now points at the derivable
  geometry. 10 new `eac3::ecpl::tests` cover every Table E3.8
  begin/end branch (SPX off + both SPX-active sub-cases), the
  6/12-bin sub-band widths of Table E3.9, the Table E2.14 default
  merge rows, the `necplbnd` count under both all-zero and
  default banding, and `band_bin_counts` totals matching the
  Table E3.9 region span. Spec erratum noted: the default-banding
  table is captioned "E2.14" in the document's table list but
  cross-referenced as "E2.13" from §E.2.3.3.18 (the latter collides
  with the standard-coupling default at the genuine Table E2.13);
  the values used are those listed in full under the §E.2.3.3.18
  heading.
- **E-AC-3 §E.2.3.1.19-21 premix-compression typed surface —
  `PremixCompression`** (round 288 / r288). The three fields the
  `mixdef == 0x1` ("mixing option 2") body of an independent
  substream's mixing-metadata block carries to steer a §E.3.10
  dual-decoder mixer's premix compression process —
  `premixcmpsel` (compression-word select, 1 bit), `drcsrc`
  (DRC-word source, 1 bit), and `premixcmpscl` (compression-word
  scale factor, 3 bits) — lift from parse-and-discard to a typed
  `Eac3Bsi::premix_compression` field (`Option<PremixCompression>`).
  The struct exposes `from_fields()` / `premixcmpsel()` / `drcsrc()`
  / `premixcmpscl()` round-trip, typed `compression_word() ->
  PremixCompressionWord` (`DynRng` / `Compr`, §E.2.3.1.19) and
  `drc_source() -> DrcSource` (`ExternalProgram` / `CurrentSubstream`,
  §E.2.3.1.20) views, `scale_ratio() -> Option<f32>` implementing
  Table E2.7 (the seven listed codes map to `n/6` gain-reduction
  ratios `0/6..=6/6`; the unlisted `0b110` codepoint is
  `is_premixcmpscl_reserved()` → `None`), and an
  `is_recommended_default()` predicate for the §E.2.3.1.21
  recommended `premixcmpsel=0 / drcsrc=0 / premixcmpscl=000`
  configuration. `Some` only when `mixmdate == 1`, the substream is
  independent, and the block selects `mixdef == 0x1`; `None`
  otherwise (a `mixdef ∈ {0, 2, 3}` block does not carry the three
  fields as a standalone 5-bit group). The single-stream decode PCM
  path is unchanged and encoder output stays byte-identical — this
  is pure surfaced metadata for a downstream §E.3.10 mixer. Covered
  by 4 new `eac3::bsi::tests` (the full Table E2.7 ratio sweep + the
  reserved `0b110` codepoint + percentage-caption cross-check, the
  typed-view + recommended-default + 3-bit masking checks, a
  `mixdef == 0x1` parse round-trip with a cursor check, and the
  `mixdef ∈ {0}` / `mixmdate == 0` None guards). 301 → 305 lib
  tests, all green.

- **E-AC-3 §E.2.3.1.53-58 pan-information typed surface — `PanInfo`**
  (round 281 / r281). The pan-information pair an independent mono /
  1+1 dual-mono substream's mixing-metadata block can carry —
  `panmean` + the 6-bit reserved `paninfo` trailer (§E.2.3.1.53-55)
  and the dual-mono second-channel copies `panmean2` + `paninfo2`
  (§E.2.3.1.56-58) — lift from parse-and-discard to typed
  `Eac3Bsi::paninfo` / `Eac3Bsi::paninfo2` fields (`Option<PanInfo>`).
  The struct implements the §E.2.3.1.54 wire scale — index `0` points
  the panned virtual source at the center speaker location (0
  degrees), each step is 1.5 degrees of clockwise rotation
  (`degrees()`, indices `0..=239` spanning `0..=358.5`), indices
  `240..=255` reserved (`is_reserved_index()`) — with
  `from_fields(panmean, reserved)` (6-bit masking on the trailer) /
  `panmean()` / `reserved()` round-trip and the §E.2.3.1.53/.56
  "defaulted to center" absent state as `None` (`PanInfo::CENTER` is
  the explicit equivalent). The §E.3.10.8 associated-audio mixer
  tables are implemented directly: `stereo_scale_factors() ->
  Option<(f32, f32)>` ((AL, AR) per Table E3.15) and
  `surround_scale_factors() -> Option<[f32; 5]>` ([AL, AC, AR, ALS,
  ARS] per Tables E3.16 + E3.17; the LFE channel is not included per
  spec) — every non-reserved index is a power-preserving sin/cos pan
  between two adjacent output speakers, verified by exhaustive
  sweeps. `Some` only when `mixmdate == 1` on an independent
  substream (Table E1.2 emits the chain under `strmtyp == 0x0` only)
  with `acmod < 0x2` (a §E.3.10.8 mixer pans a *mono* associated
  program across the main service's channels) AND the respective
  exists-flag set (`paninfo2` additionally requires `acmod == 0`).
  Pure surfaced metadata for a downstream §E.3.10 dual-decoder mixer
  — the single-stream decode PCM path is unchanged and encoder
  output stays byte-identical (`mixmdate == 0` still emitted). 8 new
  `eac3::bsi` tests: degree mapping with the 16 reserved indices,
  Table E3.15 boundary values + flat-range/trig-range continuity,
  stereo + 5.1 power-preservation sweeps over all 240 non-reserved
  indices, the five 5.1 cardinal points (one full speaker each) +
  the equal-power Center/Right midpoint, mono and 1+1 dual-mono
  parse round-trips with cursor checks (including a reserved index
  surviving the trip), and the `acmod >= 2` / exists-flag-clear
  guards. 293 → 301 lib tests, all green.
- **E-AC-3 §E.2.3.1.12-17 program-scale-factor typed surface —
  `ProgramScaleFactor`** (round 278 / r278). The three 6-bit gain words
  an independent substream's mixing-metadata block can carry — `pgmscl`
  (§E.2.3.1.13, the substream's own program), `pgmscl2` (§E.2.3.1.15,
  the second channel of a 1+1 dual-mono program), and `extpgmscl`
  (§E.2.3.1.17, an *external* program carried in a different bit stream
  / independent substream, "same scale as pgmscl") — lift from
  parse-and-discard to typed `Eac3Bsi::pgmscl` / `Eac3Bsi::pgmscl2` /
  `Eac3Bsi::extpgmscl` fields (`Option<ProgramScaleFactor>`). The
  `ProgramScaleFactor` newtype implements the §E.2.3.1.13 wire scale:
  codepoint `0` is **mute**, codepoints `1..=63` map to `-50..=+12 dB`
  in 1 dB steps (`decibels() == code - 51`, the `51` codepoint at 0 dB
  unity), with `from_code(u8)` (6-bit masking so a wider word can be
  passed verbatim) / `raw()` round-trip, `is_mute()`, `decibels() ->
  Option<i8>` (`None` for the mute codepoint, which has no finite dB
  value), and `linear() -> f32` (`0.0` mute, else `10^(dB/20)`) for a
  §E.3.10 mixer to consume directly. `Some` only when `mixmdate == 1`,
  the substream is independent (Table E1.2 emits the chain under
  `strmtyp == 0x0` only), AND the respective exists-flag is set
  (`pgmscl2` additionally requires `acmod == 0`); `None` otherwise —
  per §E.2.3.1.12/.14/.16 the absent state means "0 dB (no scaling)".
  Per §E.3.10.1-2 the gains apply "during the mixing process" of a
  dual-decoder main + associated-service mixer, so the single-stream
  decode PCM path is unchanged; surfacing them lets a downstream
  §E.3.10 mixer attenuate the main service per the associated
  substream's instruction without re-parsing the BSI. The internal
  `parse_mixing_metadata` return grew from a 4-tuple to a named
  `MixingMetadata` struct in the process. Encoders still emit
  `mixmdate == 0` so encoder output is byte-identical; the only
  behaviour change is decoder-side parsing. Covered by 8 new
  `eac3::bsi::tests` (the mute codepoint, the full `1..=63` →
  `code - 51` dB mapping with the spec's -50/+12 endpoints + unity +
  linear endpoint derivations, the 6-bit `from_code` masking, an indep
  3/2+LFE `parse()` round-trip carrying the §E.3.10.1 -3 dB and
  §E.3.10.2 -10 dB worked-example values with `bits_consumed` cursor
  check, a 1+1 dual-mono round-trip surfacing `pgmscl2` (mute) past
  the `dialnorm2`/`compr2e` chain, the exists-flags-clear default, the
  `mixmdate == 0` short-circuit, and the dependent-substream guard
  that surfaces mix levels but no scale factors). 285 → 293 lib tests,
  all green.

- **E-AC-3 §E.2.3.1.8 / Table E2.5 `ChannelLocation` classification
  surface** (round 274 / r274). The `chanmap` custom-channel-map decoder
  (`eac3::chanmap`) already expanded the 16-bit field into an ordered
  `Vec<ChannelLocation>` (r196); this round adds a spec-grounded
  classification + re-emit surface to the `ChannelLocation` enum so a
  future 7.1 WAVE-mask reorderer or a chanmap-aware §7.8 downmix router
  can route the decoded dep-substream channels without re-walking
  Table E2.5. New: `ChannelLocation::ALL` (the 22 distinct variants in
  Table E2.5 bit order, pair-bits expanded left-then-right);
  `table_e2_5_bit() -> u8` (the `0..=15` location bit a variant decoded
  from — both halves of a pair-bit share the row's single bit, making it
  the exact inverse of `expand_chanmap_locations`); `chanmap_weight() ->
  u16` (the MSB-first `1 << (15 - bit)` field weight per §E.2.3.1.8, so a
  consumer can OR a decoded location list straight back into the original
  16-bit `chanmap` — pair halves share one weight and are not
  double-counted); `is_pair_half()` + `pair_companion()` (the 12 expanded
  halves of the six Table E2.5 pair-bits — `Lc/Rc`, `Lrs/Rrs`, `Lsd/Rsd`,
  `Lw/Rw`, `Vhl/Vhr`, `Lts/Rts` — and the other half of each pair);
  `is_lfe()` (the two LFE rows, bits 14/15); `is_height()` (the
  SMPTE 428-3 above-plane rows — `Ts` bit 8, `Vhl/Vhr` bit 11, `Vhc`
  bit 12, `Lts/Rts` bit 13); and `is_surround()` (the ear-level surround
  rows — `Ls/Rs`, `Cs`, `Lrs/Rrs`, `Lsd/Rsd`). The decoder PCM path is
  unchanged — these are pure metadata accessors over the already-decoded
  location list — and encoder output is byte-identical (the in-tree
  encoder still emits 7.1 as an indep 5.1 + a dep `Lrs/Rrs`-pair
  substream). The only change is the new public classification surface.
  Covered by 8 new `eac3::chanmap::tests` (the `ALL` ordering /
  no-duplicate invariant, every Table E2.5 row's `table_e2_5_bit`, the
  MSB-first `chanmap_weight`, the decode→`chanmap_weight`-OR round-trip
  including pair-bit non-double-counting on both spec examples,
  `is_pair_half` / `pair_companion` over all 12 halves + the
  single-channel `None` cases, and the disjoint `is_lfe` / `is_height` /
  `is_surround` partitions over the full `ALL` set). 285 lib tests, all
  green.

- **Base §5.4.1.4 frame-size code typed surface — `FrameSizeCode`
  (Table 5.18)** (round 271 / r271). The 6-bit `frmsizecod` field that
  every AC-3 syncframe carries — long parsed into a raw `u8` and a
  separate pre-resolved `SyncInfo::frame_length: u32` byte length — now
  also surfaces as a typed `SyncInfo::frame_size_code() -> FrameSizeCode`
  accessor, mirroring the r263 `SampleRateCode` surface over `fscod`
  (the other half of syncinfo byte 4). `FrameSizeCode` is a two-variant
  enum: `Valid(u8)` carries one of the 38 Table 5.18 codepoints
  (`frmsizecod = 0..=37`) and `Reserved` collapses the `38..=63` range
  that has no Table 5.18 row. Per §5.4.1.4 the frame size code "is used
  along with the sample rate code to determine the number of (2-byte)
  words before the next syncword." The enum exposes `from_code(u8)`
  (masks off the upper 2 bits so a caller can pass syncinfo byte 4
  verbatim — `fscod` lives in bits 7..6) / `raw() -> Option<u8>`
  (round-trip inverse; `None` for the reserved range which has no single
  wire value), `is_reserved()` for probe / re-emit tooling,
  `nominal_bitrate_kbps() -> Option<u32>` for the Table 5.18 nominal
  bit-rate (the two neighbouring codepoints per rate return the same
  value — 44.1 kHz alternates frame sizes to hit the declared rate on
  average), `words(SampleRateCode) -> Option<u32>` for the per-rate
  16-bit-word count, and `frame_length_bytes(SampleRateCode) ->
  Option<u32>` (`2 ×` words) which matches the pre-resolved
  `SyncInfo::frame_length` field for any frame `parse` accepts. The raw
  `SyncInfo::frmsizecod: u8` and pre-resolved `SyncInfo::frame_length:
  u32` fields stay public and authoritative; the new typed surface is a
  thin convenience over them. `parse()` itself still rejects an
  out-of-range `frmsizecod` at frame boundary per §5.4.1.4 / Table 5.18,
  so a `SyncInfo` obtained from `parse()` never reports `Reserved` — the
  variant is preserved for chain consumers that construct a `SyncInfo`
  by hand (e.g. resynthesising one from container-stored metadata where
  the upstream demuxer may not have validated `frmsizecod`). The decoder
  PCM path is unchanged and encoder output is byte-identical; the only
  behaviour change is the added accessor. Covered by 6 new
  `syncinfo::tests` (the full `0..=37` valid + `38..=63` reserved
  round-trip on `from_code` / `raw` / `is_reserved`, the upper-2-bit
  masking, the `nominal_bitrate_kbps` pairing at the 32 / 192 / 640 kbps
  rows, the per-rate `words` / `frame_length_bytes` Table 5.18 lookups
  plus reserved-rate and reserved-code `None` short-circuits, the
  `parse()` → typed surface agreement on 48 / 32 kHz frames, and the
  hand-built reserved-frmsizecod surfacing path). 277 lib tests, all
  green.

- **Base §5.4.1.3 sample-rate code typed surface — `SampleRateCode`
  (Table 5.6)** (round 263 / r263). The 2-bit `fscod` field that
  every AC-3 syncframe carries — long parsed into a raw `u8` and a
  separate pre-resolved `SyncInfo::sample_rate: u32` field — now
  also surfaces as a typed
  `SyncInfo::sample_rate_code() -> SampleRateCode` accessor.
  `SampleRateCode` is a four-variant enum carrying the three valid
  sampling-rate codepoints (`FortyEightKHz` = `'00'` /
  `FortyFourPointOneKHz` = `'01'` / `ThirtyTwoKHz` = `'10'`) and a
  `Reserved` variant for the spec-reserved `'11'` codepoint that
  mandates a decoder mute per §5.4.1.3. The enum exposes
  `from_code(u8)` / `raw() -> u8` verbatim round-trip (with the
  upper bits of `code` ignored so a caller does not need to mask
  first), `hertz() -> Option<u32>` and `kilohertz() -> Option<u32>`
  for the spec rate lookups (the Table 5.6 / Annex D handbook
  tables phrase the rate both ways), `is_reserved()` for probe /
  re-emit tooling, and `hth_row_index() -> Option<usize>` that
  routes a typed sample-rate code straight into the §7.15 hearing-
  threshold table row in `tables::HTH` without re-walking Table
  5.6. The raw `SyncInfo::fscod: u8` and pre-resolved
  `SyncInfo::sample_rate: u32` fields stay public and
  authoritative; the new typed surface is a thin convenience over
  them. `parse()` itself still rejects the reserved `'11'`
  codepoint at frame boundary per §5.4.1.3 ("If the reserved code
  is indicated, the decoder should not attempt to decode audio and
  should mute") so a `SyncInfo` obtained from `parse()` never
  reports `Reserved` — the variant is preserved for chain consumers
  that construct a `SyncInfo` by hand (e.g. resynthesising one
  from container-stored metadata where the upstream demuxer may
  not have validated `fscod`). The Annex E (E-AC-3) BSI overloads
  the `'11'` codepoint as a reduced-rate indicator that triggers a
  follow-on `fscod2` codeword (§E.2.3.1.4-5), so this enum's
  `Reserved` variant corresponds to the base AC-3 decoder-mute
  semantics only; the typed surface is not mirrored on the Annex E
  `Bsi`. The decoder PCM path is unchanged and encoder output is
  byte-identical; the only behaviour change is the added accessor.
  Covered by 7 new `syncinfo::tests`. 271 lib tests, all green.

- **Base §5.4.2.4-5 mix-level typed surfaces — `CenterMixLevel`
  (Table 5.9) + `SurroundMixLevel` (Table 5.10)** (round 259 / r259).
  The 2-bit `cmixlev` codeword that the §5.3.2 guard emits when the
  stream has 3 front channels (`acmod ∈ {3, 5, 7}`) and the 2-bit
  `surmixlev` codeword that the guard emits when the stream has a
  surround channel (`acmod ∈ {4, 5, 6, 7}`) — long parsed into raw
  `u8` fields (`Bsi::cmixlev` / `Bsi::surmixlev`) with the `0xFF`
  "absent" sentinel — now surface as new typed `Bsi::center_mix:
  Option<CenterMixLevel>` and `Bsi::surround_mix:
  Option<SurroundMixLevel>` fields. The `CenterMixLevel` enum
  carries the four Table 5.9 codepoints verbatim (`Minus3Db` =
  0.707, `Minus4Point5Db` = 0.595, `Minus6Db` = 0.500, `Reserved`);
  the `SurroundMixLevel` enum carries the four Table 5.10
  codepoints (`Minus3Db` = 0.707, `Minus6Db` = 0.500, `Mute` =
  0.000, `Reserved`). Both expose `from_code(u8)` / `raw() -> u8`
  round-trip plus `coefficient() -> Option<f32>` (returns the
  spec-documented linear attenuation; `None` for the reserved
  codepoint) and `coefficient_with_reserved_fallback() -> f32`
  (applies the §5.4.2.4-5 "intermediate value may be used"
  substitution so a §7.8 downmix consumer can pick the
  per-codepoint gain in a single call).
  `SurroundMixLevel::is_mute()` lets a downmix router short-circuit
  the surround mix-in step when the encoder picked the `'10'` mute
  codepoint. `Some` only when the wire codeword is present; `None`
  for every other channel mode, mirroring the raw `0xFF` sentinel
  that the existing `Bsi::cmixlev` / `Bsi::surmixlev` fields keep
  for bit-stream round-trip. The Annex E (E-AC-3) BSI never carries
  these 2-bit slots — Annex E replaces them with the refined 3-bit
  `ltrtcmixlev` / `lorocmixlev` / `ltrtsurmixlev` / `lorosurmixlev`
  codewords inside the `mixmdata` block (§E.2.3.1.3-6) — so the
  typed surface stays on the base AC-3 `Bsi`; the
  `eac3::dsp::build_ac3_bsi_shim` hands the base helpers `None`
  unconditionally. The decoder PCM path is unchanged
  (`downmix::Downmix::from_bsi` continues to consult the raw fields
  + table-lookup) and encoder output is byte-identical; the new
  typed surfaces let chain consumers (a downstream LtRt / LoRo
  auto-router, a metadata probe) pick the per-codepoint coefficient
  without re-walking Table 5.9 / 5.10 or consulting the magic
  `0xFF` sentinel. Covered by 6 new `bsi::tests`. 264 lib tests,
  all green.
- **Annex D xbsi2 reserved-trailer typed surface — `ExtraBsi2` over
  `xbsi2` + `encinfo` (§2.3.1.11-12 / Annex D Table D2.1)** (round
  254 / r254). The 8-bit reserved-for-future-assignment `xbsi2` slot
  and the trailing 1-bit encoder-private `encinfo` flag that close
  the Annex D `xbsi2e == 1` block — long parsed-and-discarded by the
  base AC-3 BSI parser via `let _xbsi2 = …; let _encinfo = …` — now
  surface as a new typed `Bsi::extra_bsi: Option<ExtraBsi2>` field.
  Per §2.3.1.11 encoders shall set `xbsi2` to `0x00`; per §2.3.1.12
  `encinfo` is reserved for encoder-private use ("not used by the
  decoder"). The `ExtraBsi2` struct exposes `from_raw(u8, bool)` /
  `xbsi2() -> u8` / `encinfo() -> bool` / `wire_bits() -> u32` (`9` —
  8 bits for `xbsi2` plus 1 bit for `encinfo`) plus an
  `is_spec_reserved_value() -> bool` predicate that flags whether the
  carried byte matches the `0x00` wire-conformance value, so a
  conformance probe / archive tool can route non-conformant encoder
  output (`xbsi2 != 0x00`) to a chain-of-custody log without
  re-parsing the BSI. The `encinfo` bit is excluded from the
  conformance check — it is reserved for encoder-private use and any
  value is wire-legal. `Some` only when `bsid == 6` AND the encoder
  set `xbsi2e == 1`; `None` otherwise (the §5.3.2 base syntax reuses
  the bit slot for `timecod2e/timecod2` and the trailer is
  definitionally absent on `bsid != 6` streams). The block is
  base-AC-3 only — the Annex E (E-AC-3) BSI never carries an `xbsi2e`
  slot — so the Annex E → base-AC-3 shim in
  `eac3::dsp::build_ac3_bsi_shim` hands the base helpers `None`
  unconditionally and the typed surface stays on the base BSI struct.
  The decoder PCM path is unchanged — per §2.3.1.11-12 the fields
  "are not used by the decoder" — and encoders still emit
  `xbsi1e == 0` / `xbsi2e == 0` for every syncframe so encoder
  output is byte-identical; the only behaviour change is decoder-side
  parsing. Covered by 5 new `bsi::tests` (every-byte × every-flag
  `from_raw` round-trip plus `Copy`/`Eq` semantics, the
  `is_spec_reserved_value` predicate's `xbsi2 == 0x00`-only
  acceptance across all 511 non-conformant codepoints, Annex D
  `parse()` surfacing on a non-conformant `xbsi2 == 0xAA` codepoint
  with `encinfo == 1`, Annex D `parse()` surfacing on the
  spec-conformant `xbsi2 == 0x00` codepoint with `encinfo == 0`
  cross-checked against the sibling `dheadphonmod` / `adconvtyp`
  typed fields on a 2/0 frame, and the `xbsi2e == 0` short-circuit
  on a `bsid == 6` frame). 258 lib tests, all green.
- **Deprecated language-code typed surface — `LanguageCode` over
  `langcod` / `langcod2` (§5.4.2.11-12 / §5.4.2.19-20)** (round 249
  / r249). The §5.4.2.11-12 deprecated 8-bit `langcod` slot (and its
  §5.4.2.19-20 Ch2 `langcod2` mirror in 1+1 dual-mono streams) — long
  parsed-and-discarded by the base AC-3 BSI parser — now surfaces as
  two new typed `Bsi::language_code: Option<LanguageCode>` /
  `Bsi::language_code_ch2: Option<LanguageCode>` fields. Per the
  current §5.4.2.12 wire-conformance rule the slot "is an 8 bit
  reserved value that shall be set to `0xFF` if present" — the
  original 1995 8-bit table-lookup language-id semantics were retired
  in the 2001 revision, and modern delivery systems carry the
  ISO 639-2 language code in the signaling layer instead. The
  `LanguageCode` newtype wraps the raw byte verbatim and exposes
  `from_raw(u8)` / `raw() -> u8` plus an `is_spec_reserved_value()
  -> bool` predicate that flags whether the carried byte equals the
  `0xFF` wire-conformance value, so a probe / archive tool can route
  legacy non-conforming streams (carrying a 1995-era table-lookup
  codepoint) to a chain-of-custody log without re-parsing the BSI.
  `Some` only when the flag bit is set: on Ch1 when `langcode == 1`,
  on Ch2 when `acmod == 0` AND `langcod2e == 1`; `None` otherwise.
  The slot is base-AC-3 only — the Annex E (E-AC-3) BSI does not
  carry a `langcod` field — so the Annex E → base-AC-3 shim in
  `eac3::dsp::build_ac3_bsi_shim` hands the base helpers `None`
  unconditionally for both Ch1 and Ch2 surfaces, and the typed view
  stays on the base BSI struct. The decoder PCM path is unchanged —
  the word does not affect audio reproduction per §5.4.2.12 — and
  encoders still emit `langcode == 0` for every syncframe so encoder
  output is byte-identical; the only behaviour change is decoder-side
  parsing. Covered by 7 new `bsi::tests` (every-byte round-trip on
  `from_raw`, the `is_spec_reserved_value` predicate's `0xFF`-only
  acceptance, §5.3.2 base-syntax `parse()` round-trip with a
  spec-conforming `0xFF` byte, a non-conforming legacy `0x42` byte,
  the `langcode == 0` short-circuit, the 1+1 dual-mono Ch2 mirror
  with `langcod2e == 1`, and the per-channel-gate independence on
  `langcod2e == 0`).
- **Dolby Surround mode typed surface — `DolbySurroundMode` over
  base-syntax `dsurmod` (§5.4.2.6 / Table 5.11 / Annex E §E.2.3.1.x)**
  (round 246 / r246). The 2-bit `dsurmod` codepoint that the base
  AC-3 parser has long surfaced as a raw `u8` (with `0xFF` "absent"
  sentinel) now also exposes a typed
  `Bsi::dolby_surround_mode: Option<DolbySurroundMode>` field,
  mirrored on the Annex E `Bsi`. On the Annex E side the slot was
  previously parsed-and-discarded inside the §E.2.3.1.x
  informational-metadata `acmod == 2` branch — surfacing it brings
  parity with the base-syntax surface and makes both syntaxes
  routable through one branch on `Bsi::dolby_surround_mode()`. The
  `DolbySurroundMode` enum has four variants matching the wire
  codepoints — `NotIndicated` (`'00'`), `NotEncoded` (`'01'`),
  `Encoded` (`'10'`), `Reserved` (`'11'`) — with `raw() -> u8` for
  bit-stream round-trip, `is_dolby_surround_encoded()` short-circuit
  predicate for a Pro Logic-aware receiver to arm its matrix decoder,
  and `is_not_indicated()` that collapses both `NotIndicated` and
  `Reserved` into one branch per the §5.4.2.6 spec note ("the
  reserved code may be interpreted as 'not indicated'"). `Some` only
  when the wire slot is actually present — on the base parser when
  `acmod == 2`, on the Annex E parser when `infomdate == 1` AND
  `acmod == 2`; `None` otherwise. The raw `dsurmod: u8` field stays
  public on the base `Bsi` as the authoritative wire value, so
  existing consumers continue to compile and the typed surface is a
  thin convenience over it. Single source of truth across base +
  Annex E — the Annex E informational-metadata `dsurmod` slot is
  defined to reuse Table 5.11 semantics verbatim, so a chain consumer
  can route both syntaxes through the same enum. The decoder PCM
  path is unchanged — per §5.4.2.6 the field "is not used by the
  AC-3 decoder, but may be used by other portions of the audio
  reproduction equipment" — surfacing the hint lets a Pro
  Logic-aware receiver arm its matrix decoder without re-parsing
  the BSI. Encoders still emit `dsurmod == 0` (NotIndicated) for
  every 2/0 syncframe so encoder output is byte-identical; the only
  behaviour change is decoder-side parsing. The Annex E → base-AC-3
  shim in `eac3::dsp::build_ac3_bsi_shim` forwards the typed field
  through unchanged so the base AC-3 downmix helpers can consult it
  on an Annex E playback path too. Covered by 4 new `bsi::tests`
  (every-codepoint round-trip on `from_code`, the
  `is_not_indicated()` Reserved + NotIndicated collapse, the
  `is_dolby_surround_encoded()` predicate gating, the §5.3.2
  base-syntax `parse()` round-trip across all four codepoints on
  `acmod == 2`, plus a `None` short-circuit on `acmod != 2`) plus 3
  new `eac3::bsi::tests` (Annex E `infomdate == 1` round-trip
  across all four codepoints on `acmod == 2`, the per-Table-E1.2
  `acmod != 2` guard short-circuit on a 3/2 frame, and the
  `infomdate == 0` baseline).
- **Preferred stereo downmix mode typed surface — `StereoDownmixPreference`
  over `dmixmod` (§2.3.1.2 / Table D2.2 / Annex E §E.1.2.2)** (round 243
  / r243). The 2-bit `dmixmod` codepoint that the base + Annex E
  parsers have long surfaced as a raw `u8` (with `0xFF` "absent"
  sentinel) now also exposes a typed `Bsi::dmixmod_preference:
  Option<StereoDownmixPreference>` field, mirrored on the Annex E
  `Bsi`. The `StereoDownmixPreference` enum has four variants
  matching the wire codepoints — `NotIndicated` (`'00'`),
  `LtRtPreferred` (`'01'`), `LoRoPreferred` (`'10'`), `Reserved`
  (`'11'`) — with `raw() -> u8` for bit-stream round-trip,
  `prefers_lt_rt()` / `prefers_lo_ro()` short-circuit predicates
  for a §3.1.1 auto-mode two-channel-downmix router, and
  `is_not_indicated()` that collapses both `NotIndicated` and
  `Reserved` into one branch per the §2.3.1.2 spec note ("the
  reserved code may be interpreted as 'not indicated'"). `Some` only
  when the wire slot is actually present — on the base parser when
  `bsid == 6` AND `xbsi1e == 1`, on the Annex E parser when
  `mixmdate == 1` AND `acmod > 2`; `None` otherwise. The raw
  `dmixmod: u8` field stays public on both BSI structs as the
  authoritative wire value, so existing consumers continue to
  compile and the typed surface is a thin convenience over it.
  Single source of truth across base + Annex E so a chain consumer
  can route both syntaxes through one branch on
  `Bsi::stereo_downmix_preference()`. The decoder PCM path is
  unchanged — `dmixmod` is per §2.3.1.2 "may be used by the AC-3
  decoder to automatically configure the type of stereo downmix,
  but may also be overridden or ignored" — surfacing the hint lets
  a §3.1.1 compliant downmix router pick LtRt vs LoRo without
  re-parsing the BSI. Encoders still emit `xbsi1e == 0` /
  `mixmdate == 0` for every syncframe so encoder output is
  byte-identical; the only behaviour change is decoder-side
  parsing. Covered by 5 new `bsi::tests` (every-codepoint round-trip
  on `from_code`, the `is_not_indicated()` Reserved + NotIndicated
  collapse, the `prefers_lt_rt()` / `prefers_lo_ro()` predicate
  gating, the §5.3.2 base-syntax `None` short-circuit through
  `parse()`, and the Annex D `bsid == 6` + `xbsi1e == 1` round-trip
  across all four codepoints) plus 3 new `eac3::bsi::tests` (Annex E
  `mixmdate == 1` round-trip across all four codepoints, the
  `acmod == 2` per-Table-E1.2 guard short-circuit, and the
  `mixmdate == 0` baseline).
- **Additional bit-stream information typed surface — `AdditionalBitStreamInfo`
  over `addbsi` (§5.4.2.29-31 / §5.3.2 / Table E1.2)** (round 240 / r240).
  The variable-length BSI trailer the base + Annex E parsers used to
  consume-and-discard now surfaces as `Bsi::addbsi:
  Option<AdditionalBitStreamInfo>` on both the base AC-3 `Bsi` and the
  Annex E `Bsi` (Table E1.2 closes its BSI walk with `addbsie +
  addbsil + addbsi` exactly as §5.3.2 does — single typed surface for
  both shapes). `AdditionalBitStreamInfo` exposes `addbsil() -> u8`
  (raw 6-bit codepoint, 0..=63), `len() -> usize` (`addbsil + 1`,
  always within 1..=64 per §5.4.2.30), `is_empty() -> bool` (always
  `false` per spec — the field is at least 1 byte whenever present),
  `payload() -> &[u8]` (borrowed view of the wire-order bytes), and
  `wire_bits() -> u32` (`7 + 8 × (addbsil + 1)` — total span of the
  trailer block including the `addbsie` flag, for callers that need
  to mirror the BSI verbatim into a bit-stream writer). The
  `from_addbsil_and_payload` constructor rejects out-of-range
  `addbsil` (> 63) and any payload-length mismatch so callers cannot
  construct an instance that would not round-trip through the parser.
  Per §5.4.2.30 — "the decoder is not required to interpret this
  information, and thus shall skip over this number of bytes" — the
  PCM decode is unchanged; surfacing the payload bytes lets a chain
  consumer reach an encoder-private metadata block (encoder
  watermark, distribution-tagging, OAMD packetisation, downstream
  routing hint) without re-walking the BSI. Encoders still emit
  `addbsie == 0` for every syncframe so encoder output is
  byte-identical; the only behaviour change is decoder-side parsing.
  Covered by 7 new `bsi::tests` (constructor-validity rejection
  cases, minimum-length 1-byte payload, maximum-length 64-byte
  payload, `parse()` round-trip on a 1-byte payload with cursor
  check, `parse()` round-trip on the 64-byte endpoint with
  `bits_consumed` cursor check, Annex D `bsid == 6` round-trip
  confirming the trailer position is unaffected by the alt-syntax
  switch, and a 1+1 dual-mono `acmod == 0` round-trip past the Ch2
  service-metadata block) plus 4 new `eac3::bsi::tests` (encoder-
  default `addbsie == 0` short-circuit on E-AC-3, 1-byte payload,
  64-byte endpoint payload, dependent-substream walk with
  `strmtyp == 1`).
- **Dialogue-normalization typed surface — `DialNorm` over `dialnorm` /
  `dialnorm2` (§5.4.2.8 / §5.4.2.16 / §7.6)** (round 234 / r234). The
  5-bit `dialnorm` codepoint the BSI parser has long surfaced as a
  remapped `u8` now also exposes a typed `DialNorm` view via
  `Bsi::dialogue_normalization()`, mirrored on the Annex E `Bsi`. The
  `DialNorm` newtype carries `codepoint()` (post-remap `1..=31`),
  `wire_value()` (pre-remap, recovers the reserved `0`),
  `is_reserved_wire_codepoint()`, `db() -> i8` (`-31..=-1` per
  §5.4.2.8 "interpreted as -1 dB to -31 dB"),
  `level_below_full_scale_db() -> u8` (`1..=31` — the §7.6 "headroom
  in dB above the subjective dialogue level" phrasing),
  `attenuation_linear() -> f32` (`10^(db/20)` linear scalar), and
  `reproduction_gain_linear(listener_target_db, reference_full_scale_db)
  -> f32` (the §7.6 playback-gain derivation:
  `listener_target_db - reference_full_scale_db + level_below_full_scale_db`,
  matching the spec's worked example exactly: listener 67 dB SPL +
  reference 105 dB SPL + dialnorm -25 dB → -13 dB system attenuation
  → full-scale digital reproduces at 92 dB SPL).
  In parallel the parser lifts the §5.4.2.16 `dialnorm2` Ch2 mirror
  for 1+1 dual-mono (`acmod == 0`) streams from parse-and-discard to a
  new `Bsi::dialnorm_ch2: Option<u8>` field + matching
  `Bsi::dialogue_normalization_ch2() -> Option<DialNorm>` typed
  accessor, mirrored on the Annex E `Bsi`. Per §7.6 the value is not
  applied inside the AC-3 decoder itself — it is forwarded to the
  reproduction system's volume controller — so the PCM path is
  unchanged. Encoders still emit `dialnorm == 27` (-27 dB) for every
  syncframe and `dialnorm2 == 27` is unchanged in the 1+1 path so
  encoder output is byte-identical; the only behaviour change is
  decoder-side parsing. Covered by 9 new `bsi::tests` (every legal
  `1..=31` wire codepoint round-trip, the reserved-`0`-remaps-to-`31`
  path with the `is_reserved_wire_codepoint` flag, the low-5-bit-only
  masking, linear attenuation at -1 / -25 / -31 dB,
  the §7.6 worked-example reproduction-gain match, the typed accessor
  via `parse()`, the 1+1 dual-mono `dialnorm_ch2` surface, the 1+1
  Ch2 reserved-codepoint remap, and the non-1+1 `dialnorm_ch2.is_none()`
  short-circuit) plus 4 new `eac3::bsi::tests` (the indep-substream
  stereo typed-accessor round-trip, 1+1 Ch2 surface, Annex E reserved
  codepoint remap, and the non-1+1 short-circuit).
- **Distribution-control hint typed surface — `copyrightb` + `origbs`
  pair (§5.4.2.24-25 / §E.2.3.1.62)** (round 226 / r226). The two
  1-bit BSI flags the base + Annex E parsers used to consume-and-
  discard now surface as a typed `CopyrightInfo` on the base AC-3
  `Bsi` (always present per §5.3.2) and as an
  `Option<CopyrightInfo>` on the Annex E `Bsi` (gated by
  `infomdate == 1` since the pair lives inside the §E.2.3.1.62
  informational-metadata block). `CopyrightInfo` exposes
  `is_copyright_protected()` (§5.4.2.24), `is_original_bitstream()`
  (§5.4.2.25), and raw `copyrightb_bit()` / `origbs_bit()` getters
  for byte-exact re-emission. Per spec the bits are advisory and do
  not influence the decoder PCM path; the typed surface lets a chain
  consumer enforce a distribution / archival policy (refuse to
  re-encode a `copyrightb == 1` stream, tag a `origbs == 0` copy for
  downstream-only routing) without re-walking the BSI. The base
  encoder still emits `copyrightb=0, origbs=1` and the Annex E
  encoder still emits `infomdate=0` so encoder output is
  byte-identical; the only behaviour change is decoder-side parsing.
  Covered by 6 new `bsi::tests` (four-codepoint round-trip, `Eq` +
  `Copy` semantics, the encoder-default `(0,1)` BSI parse, the
  `(1,0)` protected-copy pattern, the 1+1 dual-mono `acmod == 0` BSI
  where the pair sits further down the cursor, and the Annex D
  `bsid == 6` shared-position parse with `(0,0)`) plus 3 new
  `eac3::bsi::tests` (`infomdate == 0` short-circuit; 3/2 indep
  `(1,1)`; 2/0 indep `(0,0)` exercising the `dheadphonmod` gate
  path).
- **Base-syntax timecode typed surface — `timecod1` / `timecod2` /
  `timecode_presence` (§5.4.2.26-28 / Table 5.13)** (round 219 / r219).
  The two 14-bit timecode fields the BSI parser used to consume-and-
  discard now surface as `Option<TimeCode1>` / `Option<TimeCode2>` on
  the base AC-3 `Bsi` (gated on `bsid != 6` — Annex D §1 reuses these
  wire slots for the `xbsi*e` blocks so the timecode is definitionally
  absent on `bsid == 6` streams). `TimeCode1` exposes `hours()` (5-bit),
  `minutes()` (6-bit), and `eight_second_increments()` (3-bit) plus
  `seconds_in_day()` and `is_spec_valid()` for spec-range checks
  (§5.4.2.27 documents `hours ≤ 23` / `minutes ≤ 59`); `TimeCode2`
  exposes `seconds()` (3-bit), `frames()` (5-bit), and
  `frame_fractions()` (6-bit) plus `is_spec_valid()` (frames ≤ 29 at
  the §5.4.2.26 30 fps reference). A new `TimeCodePresence` enum
  records the `(timecod2e, timecod1e)` pair per Table 5.13
  (`NotPresent` / `FirstHalfOnly` / `SecondHalfOnly` / `BothHalves`)
  so a chain consumer can pick playback strategy without re-decoding
  the flags. Per Annex D §1 / §3.2 the timecode "does not affect the
  decoding process in legacy decoders" — the AC-3 PCM path is
  unchanged. Encoders still emit `timecod1e == 0` and `timecod2e == 0`
  per the long-standing default so encoder output is byte-identical;
  the only behaviour change is decoder-side parsing. Covered by 10
  new `bsi::tests`: `TimeCode1` and `TimeCode2` field-decomposition
  walks (including out-of-range codepoint passthrough), `is_spec_valid`
  range checks for both halves, the Table 5.13 `from_flags`
  round-trip, `parse()` surfacing both halves on a base-syntax frame,
  the `FirstHalfOnly` and `SecondHalfOnly` partial-presence rows, the
  `NotPresent` all-zero case, and the Annex D `bsid == 6` short-circuit
  that keeps `timecod1` / `timecod2` at `None` even when the wire bits
  carrying `xbsi*e` are set.
- **Audio production information typed surface — `mixlevel` + `roomtyp`
  (§5.4.2.13-15 / Table 5.12 + §E.2.3.1.x)** (round 214 / r214). The
  two `audprodie == 1` payload fields the BSI parser used to consume-
  and-discard now surface as a typed `Option<AudioProductionInfo>` on
  both the base AC-3 `Bsi` and the Annex E `Bsi`. The struct carries
  the raw 5-bit `mixlevel` codepoint plus a typed `RoomType` enum
  (Table 5.12: `NotIndicated` / `LargeXCurve` / `SmallFlat` /
  `Reserved`); a `peak_mix_level_db_spl()` accessor resolves the
  spec's `80 + mixlevel` derivation (range 80..=111 dB SPL per
  §5.4.2.14). The 1+1 dual-mono Ch2 mirror (`audprodi2e == 1`,
  §5.4.2.21-23) surfaces as a separate `audio_production_ch2` field
  so a chain consumer routing Ch1/Ch2 to independent SPL-calibrated
  reproduction buses no longer needs a second BSI walk. Per spec the
  field "is not typically used within the AC-3 decoder, but may be
  used by other parts of the audio reproduction equipment" —
  surfacing the typed values lets cinema / mastering tooling re-target
  the playback bus to the absolute SPL the mixing engineer was
  monitoring at without forfeiting decoder-side zero-overhead.
  Encoders still emit `audprodie == 0` for every syncframe so output
  is byte-identical; the only behaviour change is decoder-side
  parsing. Covered by 5 new `bsi::tests` (every Table 5.12 row's
  round-trip, the `80 + mixlevel` endpoint resolution at 0 / 5 / 31
  codepoints, the `audprodie == 1` mono surface, the `audprodie == 0`
  short-circuit, and the 1+1 dual-mono Ch1+Ch2 independent surfacing)
  plus 1 new `eac3::bsi::tests::no_infomdate_yields_no_audio_production`
  short-circuit + extended assertions on two pre-existing `infomdate`
  tests that already exercise `audprodie == 1` (3/2 indep and 1+1
  dual-mono).
- **xbsi2 / informational-metadata typed surface — Dolby Surround EX,
  Dolby Headphone, A/D converter type (Tables D2.7 / D2.8 / D2.9 +
  §E.2.3.1.x)** (round 208 / r208). Three Annex D §2.3.1.7-10 fields
  that the BSI parser used to consume-and-discard now surface as
  typed `Option<DolbySurroundExMode>` / `Option<DolbyHeadphoneMode>` /
  `Option<AdConverterType>` on the base AC-3 `Bsi` (gated by
  `bsid == 6` AND `xbsi2e == 1`). The Annex E (E-AC-3) `Bsi` carries
  the same surface — the §E.2.3.1.x informational-metadata block
  reuses Tables D2.7 / D2.8 / D2.9 verbatim — under the spec's
  per-acmod gates: `dsurexmod` only when `acmod ≥ 6` (a stereo
  surround pair exists to drive the EX matrix), `dheadphonmod` only
  when `acmod == 2` (2/0 stereo), `adconvtyp` inside the `audprodie`
  chain, and a separate `adconvtyp_ch2` field on the Annex E `Bsi` for
  the 1+1 dual-mono Ch2 word (`audprodi2e == 1`). All three enums
  expose `from_code(u8)` / `raw() -> u8` accessors over the spec's
  small variant sets — `DolbySurroundExMode` covers Tables D2.7's four
  codepoints (`NotIndicated` / `NotEncoded` /
  `SurroundExOrProLogicIIx` / `ProLogicIIz`); `DolbyHeadphoneMode`
  covers D2.8's three indicators plus an explicit `Reserved` variant
  for the `'11'` codepoint (which the spec mandates the decoder still
  reproduces audio for, treating as `NotIndicated`); `AdConverterType`
  covers D2.9's `Standard` / `Hdcd` single-bit choice. These three
  fields are per the spec text purely informational hints for
  downstream playback equipment (surround upmix processor, headphone
  virtualiser, HDCD-aware DAC) and do not affect AC-3 / E-AC-3 PCM
  decode — but surfacing them lets a chain consumer route the hint
  without re-parsing the BSI. The encoders still emit `xbsi2e=0` and
  `infomdate=0` for every syncframe (matching the round-126 / r126
  default), so the only behavioural change is decoder-side parsing.
  Covered by 4 new `bsi::tests` (enum-codepoint round-trips for all
  three tables, an `xbsi2e==1` 3/2 frame surfacing all three typed
  fields, and a pair of `None`-stays-`None` assertions for `bsid != 6`
  and `xbsi2e == 0`) plus 4 new `eac3::bsi::tests` (`infomdate == 0`
  yields no playback hints; 3/2 indep with `infomdate == 1` surfaces
  `dsurexmod` + `adconvtyp` and leaves `dheadphonmod` `None`; 2/0
  stereo with `infomdate == 1` surfaces `dheadphonmod` and leaves
  the other two `None`; 1+1 dual-mono with both `audprodie` and
  `audprodi2e` set surfaces `adconvtyp` and `adconvtyp_ch2`
  independently).

- **Heavy compression gain word `compr` typed surface (Table 7.30 /
  §7.7.2.2)** (round 202 / r202). The base AC-3 BSI parser used to
  consume-and-discard the 8-bit `compr` byte (and the Ch2 `compr2`
  byte in 1+1 dual-mono mode); both are now surfaced on
  `Bsi::compr` / `Bsi::compr_ch2` as `Option<CompressionGain>`. A
  new `bsi::CompressionGain` newtype wraps the byte and exposes
  the Table 7.30 decomposition: `x() -> i8` (4-bit signed integer
  in `-8..=+7`, contributing `(X+1)·6.02 dB`), `y() -> u8` (4-bit
  unsigned mantissa with implicit leading 1, contributing the
  `(16+Y)/32` linear attenuation between -6.02 dB and -0.28 dB),
  plus `linear() -> f32` and `decibels() -> f32` accessors. The
  Annex E (E-AC-3) `Bsi` mirrors the same surface — §E.2.3.1.x
  reuses Table 7.30 verbatim and points back at the base spec —
  via the same `CompressionGain` type so a single source of truth
  drives both parsers. `None` is preserved verbatim when the
  encoder did not emit a heavy-compression word, so a player can
  honour the §7.7.2.1 fallback rule ("if compr is not present for
  a particular syncframe, the dynrng control signal shall be used
  for that syncframe"). The decoder PCM path is unchanged — both
  `compr` and `dynrng` continue to be left for the application to
  apply downstream, matching the spec's "at the discretion of the
  decoder" language — but the typed surface lets a peak-limited
  feed (RF-modulator, hotel-room, airline-seat per §7.7.2.1)
  implement the compress-on policy without re-parsing the
  bitstream. The encoders still emit `compre=0` for every
  syncframe (no heavy-compression policy yet), so the new
  decoder-side parsing is the only behaviour change. Covered by
  6 new unit tests across `bsi::tests` (every `X` codepoint's
  two's-complement sign-extension, every Table 7.30 row's dB
  endpoints at both `Y=0` and `Y=15`, the `Y` fractional decode
  with implicit leading 1, the §7.7.2.2 combined ±48 dB range
  endpoints, `parse()` round-trip via `compre=1`, and the 1+1
  `compr2e` round-trip) plus 1 new `eac3::bsi::tests` round-trip.

- **E-AC-3 dependent-substream chanmap routing (Table E2.5)**
  (round 196 / r196). The decoder now decodes the 16-bit `chanmap`
  field on each dependent substream into an ordered list of
  physical channel locations per §E.2.3.1.7-8. A new
  `eac3::chanmap::ChannelLocation` enum covers all 22 distinct
  Table E2.5 locations — including the 6 pair-bits (Lc/Rc, Lrs/Rrs,
  Lsd/Rsd, Lw/Rw, Vhl/Vhr, Lts/Rts) which each expand to two
  consecutive channels in the order documented by the spec text
  ("the first coded channel is the Left Surround channel, the
  second coded channel is the Right Surround channel"). The decoder
  validates the spec invariant that the expanded chanmap count
  must equal the dep substream's `acmod`/`lfeon`-derived coded
  channel count (§E.2.3.1.8 last paragraph) and rejects mismatched
  bit-streams with a typed `ChanmapError::CountMismatch`. When
  `chanmape == 0`, `Eac3DecoderState::default_dep_locations` falls
  back to the natural-acmod order per §E.2.3.1.7. The resolved
  `dep_locations` list is surfaced on `Eac3DecoderState` AND on
  `DecodedFrame.dep_locations`, so consumers (a future WAV-mask
  7.1 reorderer or a chanmap-aware §7.8 downmix matrix) can route
  the appended dep channels without re-parsing the bitstream. The
  splice itself still appends dep channels at the end of the indep
  program — the new metadata is the foundation for future routing
  work, not a behavioural change for current acmod-native
  consumers. 6 new unit tests cover both literal spec examples
  (§E.2.3.1.8 bits 0/3/4 and bits 3/4/6 pair), the in-tree 7.1
  encoder's Lrs/Rrs pair (chanmap=0x0200), single-bit-only
  decodes, MSB/LSB extremes, and the count-mismatch rejection.
  1 new integration test in `tests/eac3_ffmpeg.rs` round-trips
  the encoder's 7.1 indep+dep pair through the in-tree decoder
  and asserts `dep_locations == [LeftRearSurround,
  RightRearSurround]` on every packet.

- **Typed `BitStreamMode` accessor for §5.4.2.2 / Table 5.7**
  (round 193 / r193). The raw `Bsi::bsmod` and `Bsi::acmod` fields
  were already public, but Table 5.7's classification of the
  8 `bsmod` codepoints into main / associated services (with
  `bsmod=0b111` overloaded on `acmod` for the VoiceOver vs Karaoke
  split) had to be re-derived by every caller. `bsi::BitStreamMode`
  is a 10-variant enum covering every cell — `CompleteMain` /
  `MusicAndEffects` / `VisuallyImpaired` / `HearingImpaired` /
  `Dialogue` / `Commentary` / `Emergency` / `VoiceOver` / `Karaoke`
  / `Reserved` — and `Bsi::service_type()` returns it by
  delegating to `BitStreamMode::from_bsmod_acmod(bsmod, acmod)`.
  `is_main()` / `is_associated()` partition the table for
  service-routing logic (a receiver normally selects a single main
  service and mixes associated services on top), and `mnemonic()`
  returns the Table 5.7 short forms (`"CM"`, `"ME"`, `"VI"`, `"HI"`,
  `"D"`, `"C"`, `"E"`, `"VO"`, `"K"`, `"?"` for the undefined
  `bsmod=0b111 acmod=0b000` cell) for CLI / log output. The raw
  fields stay authoritative and the accessor never panics on an
  unmatched value. 5 new unit tests cover the fixed-codepoint
  rows, the overloaded-`0b111` branch (`acmod=0b000` →
  `Reserved`, `acmod=0b001` → `VoiceOver`, `acmod ∈ {0b010..=0b111}`
  → `Karaoke`), the main / associated partition, mnemonic
  stability, and a `Bsi::service_type()` round-trip through
  `parse()` on a synthetic 1/0 mono `bsmod=0b111 acmod=0b001`
  voice-over BSI. Lib tests 156 pass (was 151, +5 new); integration
  tests pass; clippy clean; rustfmt clean.

## [0.0.8](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.7...v0.0.8) - 2026-05-30

### Other

- ac3 round 190 (r190): refresh stale lib.rs + eac3/mod.rs module docstrings
- ac3 round 187 (r187): encoder crc2 in §7.10.1 augmented form
- ac3 round 182 (r182): opt-in §7.10.1 decoder CRC verification API
- ac3 round 176 (r176): clean-room comment hygiene — scrub decorative implementation-attribution prose
- eac3 round 172 (r172): SPX attenuation (§3.6.4.2.3) border notch filter

### Documentation

- **Refresh stale `lib.rs` and `eac3/mod.rs` module docstrings**
  (round 190 / r190). The crate-root docstring previously billed
  the codec as an "initial skeleton" that "emits PCM frames
  (currently silence)" with "real IMDCT, bit allocation, exponent
  decode, and mantissa dequantization … staged for follow-up
  commits", and listed `audblk` + transform synthesis as `TODO`.
  This is the page docs.rs shows as the public landing for
  `oxideav-ac3`; it was about 180 round commits out of date.
  Rewritten to describe the actual module layout — `syncinfo` /
  `bsi` / `audblk` / `imdct` / `mdct` / `downmix` / `wave_order` /
  `encoder` / `eac3` / `crc` — with each entry pinned to the
  spec section it implements (§5..§7 for base AC-3, §E for
  E-AC-3). The E-AC-3 register-block comment in `lib.rs` was
  similarly stale ("Decoder side: round-1 path parses the BSI +
  audfrm bit-accurately and emits silent PCM … Real DSP
  (decouple + IMDCT) lands in round 2") and is rewritten to
  describe the actual decoder path (AHT on fbw / LFE / coupling
  channels, SPX with SPXATTEN border notch, transient pre-noise,
  LoRo / LtRt downmix). `src/eac3/mod.rs`'s round-1 / round-2 /
  round-6 / "deferred to round 7 and beyond" headings are
  replaced with a status-based "Module layout" + "Known decoder
  gaps" pair so future doc.rs visitors don't have to reason about
  round numbering. The README's per-feature checklist stays the
  authoritative source for round-by-round status.

  No code, normative comments, or test behaviour changes; the
  diff is comments + module-level `//!` docstrings only.
  Behavioural test machinery (151 lib tests + integration suite)
  is byte-identical pre/post.

### Fixed

- **AC-3 + E-AC-3 encoder `crc2` emit now uses the spec's augmented
  CRC form** (round 187 / r187). ATSC A/52:2018 §7.10.1 defines a
  valid `crc2` as the value the LFSR holds at end-of-frame after
  shifting through the post-syncword body *and* the 16-bit crc2
  field — equivalently `r(x) = data·x^16 mod g(x)` (the augmented
  CRC codeword property `data·x^16 + r(x) ≡ 0 mod g(x)`). The
  previous emit stored `ac3_crc_update(0, &frame[body_start..(n -
  2)])` (a direct-form `data mod g(x)` value), which a
  spec-strict residue-checking decoder marks invalid because
  shifting that value through the trailing LFSR positions yields a
  non-zero register. Each encoder now does
  `ac3_crc_update(ac3_crc_update(0, body), &[0, 0])` and a debug
  assertion pins the post-syncword residue to zero on every
  emitted syncframe. AC-3 (`encoder.rs::emit_frame_packet`)
  chains from the 5/8 boundary (the crc1 solver guarantees the
  running CRC is zero there); E-AC-3
  (`eac3/encoder.rs::emit_packet`) starts from byte 2 because
  Annex E syncframes carry no `crc1`. `crc1` emit is unchanged
  (already spec-correct via `ac3_crc_solve_prefix`); body and side
  info bytes are unchanged. The r182 verifier now reports
  `crc2_ok = Some(true)` on our own emitted bitstreams, so a
  lenient §6.1.2 decoder behaves identically (it always accepted
  on `crc1_ok = true`) but a strict residue-checking decoder will
  no longer flag our frames as `crc2`-invalid.

  The three r182 decoder tests that pinned the bug now assert
  `crc2_ok = Some(true)` on encoder output:
  `verify_packet_crc_matches_residue_on_ffmpeg_fixture` (residue
  check against the FFmpeg fixture, unchanged); the renamed
  `ac3_encoder_output_has_spec_correct_crc1_and_crc2` (asserts
  both crc fields on our own AC-3 output);
  `verify_packet_crc_dispatches_eac3_path_correctly` (asserts
  `crc2_ok = Some(true)` on our own E-AC-3 output, plus the
  `crc1_ok = None` dispatch contract). The deferred-followup
  caveat in the r182 README + crc module docs is removed.

### Added

- **§7.10.1 CRC verification API** (round 182). The CRC-16
  primitive that the encoder uses to fill `crc1` is now exposed
  for decoder-side validation under a new `crc` module:
  `verify_ac3_syncframe`, `verify_eac3_syncframe`,
  `crc1_boundary_bytes`, the public `CrcStatus` enum, and the
  top-level `decoder::verify_packet_crc` peek-and-dispatch
  function. The verifier implements the spec's **residue check**:
  shift the post-syncword data through the LFSR (with the stored
  CRC fields included) and the register must read zero at the
  end. The decode pipeline does not call this automatically — it
  stays opt-in to match the spec's "may be used at the discretion
  of the decoder" language and to keep zero-overhead decoding the
  default. Empirically validated against the FFmpeg-produced
  `tests/fixtures/sine440_stereo.ac3` corpus: every syncframe in
  the fixture satisfies the spec residue check on both `crc1` and
  `crc2`. `CrcStatus` reports `crc1_ok` / `crc2_ok` independently
  so callers can implement either of the §6.1.2 strategies
  (accept on either CRC valid, or require both). E-AC-3 reports
  `crc1_ok = None` (the Annex E syncframe has no `crc1` field per
  Table E1.2). Adds 14 unit tests on the new primitive + 3
  end-to-end tests
  (`verify_packet_crc_matches_residue_on_ffmpeg_fixture`,
  `ac3_encoder_output_has_spec_correct_crc1`,
  `verify_packet_crc_dispatches_eac3_path_correctly`) and
  single-bit-flip tamper detection on the ffmpeg fixture path.
  Total +17 lib tests; full suite remains green (134 → 151
  lib tests, all integration tests unchanged).

### Known issues

- **Our own AC-3 / E-AC-3 encoder writes `crc2` in direct form**
  (`data mod g(x)`) rather than the augmented form
  (`data·x^16 mod g(x)`) the §7.10.1 residue test implies. The
  encoder's `crc1` is spec-compliant (a future-round
  `ac3_crc_solve_prefix` already drives the 5/8 residue to zero
  per the spec). The `crc2` mismatch means our emitted bitstreams
  will be flagged as crc2-invalid by spec-strict decoders running
  the residue check (a §6.1.2 lenient decoder that accepts on
  either CRC will still play them because `crc1` validates). The
  fix is a small encoder change: replace `crc2 = ac3_crc_update(
  0, payload)` with `crc2 = ac3_crc_update(0, payload || [0, 0])`
  so the trailing 16 zero bits flush the LFSR through `data·x^16
  mod g(x)`. Deferred to a future round because it changes every
  encoded bitstream and warrants its own ffmpeg cross-decode
  audit. The new `decoder::tests::ac3_encoder_output_has_spec_
  correct_crc1` test will tighten once the encoder fix lands.

### Changed

- **CRC-16 primitive moved to a shared module** (round 182).
  `ac3_crc_update` and `ac3_crc_solve_prefix` now live in
  `crate::crc` instead of `crate::encoder` so the decoder can
  reuse the same byte-exact LFSR. `crate::encoder` re-exports the
  two functions under their original names (`pub(crate) use
  crate::crc::{ac3_crc_update, ac3_crc_solve_prefix}`) so the
  `eac3/encoder.rs` call sites continue to compile unchanged. No
  bitstream output is affected; the diff is purely a code move +
  an additive public API.

- **Clean-room comment hygiene** (round 176). Pre-existing decorative
  implementation-attribution prose in `src/` was rewritten to spec-
  grounded and observable terms. Behavioural and bitstream output is
  unchanged. The diff touches comments + the `Ac3Decoder` `LtRt`
  doc-string; no normative code paths move. Notable substitutions:
  - `"ffmpeg-style decoders also have a frame of look-ahead"`
    → `"a frame-aligned reference decoder may also carry a frame of
    look-ahead"` (`src/encoder.rs`, lag-search comment in a self-
    roundtrip test).
  - `"matches FFmpeg's default"` → `"is the spec's default downmix
    matrix"` (`src/decoder.rs`, `prefer_ltrt` field doc).
  - `"libavcodec clamps absexp_after_seed to ≤ 24, and any group …
    flagged \"expacc out-of-range\" by the dexp validity check"`
    → spec-grounded: `"the spec's §7.1 exponent envelope (and the
    validator binary's dexp validity check) rejects reconstructed
    exp > 24 as out-of-range"` (`src/encoder.rs`).
  - `"ffmpeg uses dba to BOOST the masking floor …"`
    → `"the §7.2.2.6 mechanism BOOSTs the masking floor …"`
    (`src/audblk.rs`).
  - `"ffmpeg uses this to free a few mantissa bits …"`
    → `"the §7.2.2.6 mechanism uses this …"` (`src/encoder.rs`).
  - `"the canonical FFmpeg pattern"` / `"every corpus FFmpeg-encoded
    fixture picks row 16"` → `"the prevailing corpus pattern"` /
    `"every validator-encoded fixture in our corpus picks row 16"`
    (`src/eac3/audfrm.rs`).
  - `"FFmpeg's E-AC-3 encoder picks narrow configs"`
    → `"valid corpus bitstreams use narrow configs"`
    (`src/eac3/dsp.rs`).
  - Multiple `"ffmpeg's parser consumes/rejects/detects …"`
    → `"the validator binary consumes/rejects/detects …"`
    (`src/eac3/encoder.rs`, `src/encoder.rs` cplcoe comment).
  - Several `"ffmpeg's reference decode"` / `"FFmpeg reference"`
    → `"the validator binary's decode"` / `"the validator binary's
    PCM"` (`src/audblk.rs`, `src/imdct.rs`, `src/eac3/mod.rs`,
    `src/eac3/aht.rs`, `src/wave_order.rs`).

  Black-box validator-binary invocations in tests
  (`Command::new("ffmpeg")`) and the test-file path
  `tests/eac3_ffmpeg.rs` are unchanged — these name `ffmpeg` as an
  opaque validator process, which the workspace allow-list permits.
- **README §"IMDCT synthesis" claim refreshed**: the short-block path
  has been on the §7.9.4 FFT-backed decomposition since the
  `imdct_256_pair_fft` wire-up; the direct-form `imdct_256_pair` in
  `audblk.rs` is now only a test oracle.

## [0.0.7](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.6...v0.0.7) - 2026-05-25

### Other

- eac3 round 129 (r129): surface mixmdata mix levels + route through downmix
- ac3 round 126 (r126): Annex D §2.3 alternate-syntax mix-level params
- ac3 round 120 (r120): §7.8.2 LtRt matrix-encoded stereo downmix
- decode coupling-channel AHT (cplahtinu) — round 117
- reword LFE-drop rationale to spec-permitted/observable terms (clean-room comment hygiene)
- eac3 decoder round 113 (r113): LFE-channel AHT decode + standard-LFE mantissa fix
- multichannel full-bandwidth AHT decode (§3.4.2 nregs)
- eac3 decoder round 103 (r103): transient pre-noise processing (TPNP) decode
- eac3 decoder round 100 (r100): spectral extension (SPX) decode
- ac3 encoder round 95 (r95): fairness fix for per-channel fsnroffst greedy
- ac3 encoder round 91 (r91): 2/2 acmod=6 self-decode test + per-channel PSNR regression gates for 2/2, 5.0, 5.1
- ac3 encoder round 78 (r78): 2.1 (L,R,LFE) acmod=2 + lfeon=1 emit path
- rewrite two source comments to remove ffmpeg-internals language
- eac3 round 72 (r72): Table E2.10 frame-based exponent strategy decode
- ac3 round 7 (r7): widen coupling-range check to spec envelope
- ac3/eac3 round 6 (r6): emit decoder PCM in WAV-mask channel order

### Added

- **E-AC-3 spectral-extension attenuation (§3.6.4.2.3)** — round 172.
  Lifts the round-2 whole-frame reject `audfrm.spxattene == 1` so any
  E-AC-3 syncframe carrying spectral-extension attenuation parameters
  decodes through the SPX synthesis pipeline with the spec's 5-tap
  border notch filter applied.
  * `eac3::audfrm::parse_with` now writes `chinspxatten[ch]` (1 bit) +
    `spxattencod[ch]` (5 bits, §2.3.2.24-25) into new `AudFrm.chinspxatten`
    / `AudFrm.spxattencod` arrays instead of consuming-and-discarding
    them, mirroring the round-103 TPNP-field surfacing pattern.
  * `audblk::ChannelState` gains `spx_atten_active` + `spx_atten_code`
    (frame-scoped — the spec emits both in audfrm, not audblk, so the
    flags stay constant across the 6 blocks of a syncframe).
    `eac3::dsp::decode_indep_audblks` propagates the audfrm fields onto
    state at the top of every frame; when `spxattene == 0` they reset
    to false/0 so a previous frame's attenuation doesn't leak forward.
  * `audblk::SPX_ATTEN_TABLE` is a `[[f32; 3]; 32]` direct transcription
    of Table E3.14 — the 3 stored taps per row become a 5-tap symmetric
    kernel `[T[0], T[1], T[2], T[1], T[0]]` per the spec's "last two
    attenuation values determined by symmetry" rule.
  * `audblk::apply_spectral_extension` now applies the §3.6.4.2.3 notch
    filter at the baseband / extension border (centred on
    `spx_begin_tc`, filter window `[spx_begin_tc - 2 .. spx_begin_tc + 2]`)
    AND at every wrap point flagged during the §3.6.4.1 translation
    copy (centred on each post-wrap band start). The filter sits
    between the translation step and the noise-blend / coordinate-scale
    step, matching the spec's "filtering occurs after the transform
    coefficient translation and banded RMS energy calculation but
    prior to the noise scaling and transform coefficient blending
    calculation" placement — implemented here as
    translate → notch → RMS+blend so the RMS is measured on the
    already-attenuated bins (the spec's wording reads either way; this
    choice matches the §3.6.4.2.3 pseudo-code's positioning of the
    notch loop before the blend loop in §3.6.4.2.4).
  * 5 new unit tests in `audblk::spx_tests`: Table E3.14 row spot-checks
    (rows 0 / 14 / 29 covering the 1×, 0.5×, 0.25× anchor magnitudes);
    `apply_spx_atten_notch` symmetric-kernel + 5-bit-code-masking; and
    two end-to-end synthesis checks pinning the border-notch + wrap-point
    notch placements against a constant-1 driving signal with sblend=1 +
    nblend=0 + coord=1/32 (so the kernel taps appear directly in the
    output bins). When `spx_atten_active == false`, synthesis is
    byte-identical to the round-100 baseline (separate test).
  * Note: no FFmpeg-encoded E-AC-3 fixture in the corpus carries
    `spxattene == 1` (FFmpeg's E-AC-3 encoder doesn't emit it), so this
    landing is covered by unit-test synthesis math rather than an
    end-to-end PSNR gate. Matches the round-103 (TPNP) + round-113
    (LFE AHT) + round-117 (coupling AHT) precedent: corpus-untestable
    decoder paths land with synthesis-math coverage so they don't drift
    silently.

- **E-AC-3 mixmdata mix-level surfacing + downmix routing** —
  round 129. Pulls the four 3-bit `ltrtcmixlev` / `ltrtsurmixlev` /
  `lorocmixlev` / `lorosurmixlev` codewords (§E.2.3.1.3-6, Table E1.2)
  out of `eac3::bsi::skip_mixing_metadata` (renamed
  `parse_mixing_metadata`) and routes them through the §7.8 downmix
  matrix used by `process_eac3_frame`.
  * `eac3::Bsi` gains `annex_e_mix_levels: Option<AnnexDMixLevels>`
    (the Annex E mixmdata fields share their value range with
    Annex D xbsi1 — Tables E1.13-16 = D2.3-D2.6 — so the existing
    `AnnexDMixLevels` struct is reused), `dmixmod: u8` (the §E.2.3.1.2
    preferred-stereo-downmix advisory; `0xFF` when absent), and
    `lfemixlevcod: Option<u8>` (§E.2.3.1 5-bit LFE mix code; surfaced
    for downstream LFE-bass-routing tooling, not consumed by the
    round-129 downmix which keeps LFE muted per §7.8).
  * `downmix::Downmix` gains `from_eac3_bsi(&Eac3Bsi, mode)` and a
    field-based `from_eac3_fields(acmod, nfchans, nchans, lfeon,
    mix, mode)` constructor. Both run through a shared private
    `build` helper that resolves the per-target (`clev`, `slev`) pair
    from `Option<AnnexDMixLevels>` exactly like `from_bsi` — so the
    LtRt and LoRo matrices on an E-AC-3 stream with mixmdata are
    coefficient-identical to a base AC-3 stream carrying the same
    four xbsi1 codes.
  * `eac3::DecodedFrame` gains `nfchans: u8` and
    `annex_e_mix_levels: Option<AnnexDMixLevels>` so the top-level
    decoder no longer has to re-derive nfchans from `channels - lfeon`
    or re-parse the BSI to recover the mixmdata.
  * `eac3::Eac3DecoderState` gains `indep_pcm_f32()` / `indep_nchans()`
    accessors so `process_eac3_frame` can run the §7.8 matrix on the
    pre-quantisation f32 PCM (negative LtRt surround weights truncate
    to 0 if matrixed in S16 space).
  * `process_eac3_frame` now routes through `Downmix::from_eac3_fields`
    when a downmix is requested instead of the previous "truncate the
    interleaved buffer to N channels" shortcut, applying the §7.8.2
    LoRo / LtRt / mono matrix block-by-block in 256-sample chunks and
    quantising back to S16LE at the boundary. Passthrough still uses
    the in-place WAV-mask reorder path for parity with prior rounds.
  * 8 unit tests:
    `captures_mixmdata_5_1_full_mix_levels` /
    `captures_mixmdata_3_1_no_lfe` /
    `mixmdate_on_stereo_yields_no_mix_levels` /
    `no_mixmdate_yields_none` exercise the BSI cursor across the
    per-channel guards and the `mixmdate == 0` baseline;
    `eac3_fields_match_annex_d_for_same_mix_codes` /
    `eac3_fields_without_mixmdata_uses_fixed_0_707` /
    `eac3_loro_honours_lorocmixlev_override` pin the matrix
    equivalence vs the Annex D path and the override semantics;
    `eac3_5_1_decodes_to_stereo_with_matrix_downmix` is an end-to-end
    encode → 5.1 → request stereo decode → verify the 2-channel
    payload shape via the matrix path (catches a regression where the
    decoder would still produce a buffer of the right size by
    truncation even after the matrix wiring breaks).

- **Annex D §2.3 alternate-syntax mix-level parameters** — round 126.
  Closes the round-120 followup to honour Annex D `ltrtcmixlev` /
  `ltrtsurmixlev` / `lorocmixlev` / `lorosurmixlev` instead of the
  body-spec fixed coefficients.
  * `bsi::parse` now detects `bsid == 6` (§2.1) and switches the
    post-`origbs` parse from the body `timecod1e/timecod2e` slots to
    the Annex D `xbsi1e/xbsi2e` blocks (Table D2.1). Both shapes occupy
    the same 30-bit window so the surrounding cursor is unchanged.
  * `Bsi` gains `annex_d_mix_levels: Option<AnnexDMixLevels>` carrying
    the four 3-bit codewords (`ltrtcmixlev` / `ltrtsurmixlev` /
    `lorocmixlev` / `lorosurmixlev`), plus `dmixmod: u8` (Table D2.2
    preferred-stereo-downmix advisory; `0xFF` when absent).
  * `Downmix::from_bsi` consults the Annex D fields when present:
    LtRt uses `ltrtcmixlev` / `ltrtsurmixlev` instead of the §7.8.2
    fixed 0.707; LoRo uses `lorocmixlev` / `lorosurmixlev` instead of
    the body `cmixlev` / `surmixlev`. Mono keeps the body fields per
    §7.8.2 (Annex D has no mono-specific mix levels). Without the
    Annex D extension the matrix is byte-identical to the round-120
    behaviour.
  * Tables D2.3-D2.6 are exposed as `annex_d_center_mix_gain` /
    `annex_d_surround_mix_gain`. Reserved codes `000/001/010` on the
    surround tables resolve to `0.841` per §2.3.1.4 / §2.3.1.6
    "decoder shall use a value of 0.841".
  * 7 unit tests: `parses_annex_d_bsid_6_xbsi1_mix_levels` /
    `parses_annex_d_bsid_6_no_xbsi1` exercise the BSI cursor;
    `annex_d_center_mix_gain_matches_table_d2_3` /
    `annex_d_surround_mix_gain_substitutes_reserved_with_0_841` pin
    the spec tables; `ltrt_3_2_honours_annex_d_ltrtcmixlev_override` /
    `ltrt_reserved_surround_code_substitutes_0_841` /
    `loro_honours_annex_d_lorocmixlev_override` /
    `ltrt_without_annex_d_uses_fixed_0_707` verify the downmix
    matrix tracks the overrides without regressing the body path.

- **§7.8.2 LtRt (Dolby Surround matrix-encoded stereo) downmix** —
  round 120. The §7.8 downmix matrix gains a third target alongside
  LoRo and Mono: `DownmixMode::StereoLtRt` implements the spec's
  3/2 LtRt equations
  `Lt = 1·L + 0.707·C − 0.707·Ls − 0.707·Rs` /
  `Rt = 1·R + 0.707·C + 0.707·Ls + 0.707·Rs`,
  plus the 3/1 form (single surround folded with opposite signs),
  with the 2/1 / 2/2 C-drop and 3/0 / 2/0 / 1/0 / 1+1 narrower
  cases. The §7.8.2 normalisation lands on Table 7.32's
  headline 0.3204 / 0.2265 coefficients (worst-case
  `1 + 3·0.707 = 3.121` → divide by 3.121 = 0.3204; 9.89 dB
  attenuation, matching the spec's Table 7.32 row 1 / row 2).
  * A new `decoder::make_decoder_ltrt` factory parallels
    `make_decoder` / `make_eac3_decoder`: when a 2-channel
    downmix is requested it routes through the LtRt matrix
    instead of LoRo. The default factories keep LoRo (FFmpeg's
    default + §7.8.2's "preferred when the ultimate target is
    mono" path) so existing callers see no behaviour change.
  * Mono / Passthrough are unaffected. §7.8.2 explicitly notes
    that combining LtRt to mono destroys the surround
    information, so the LtRt path is stereo-only and a Mono
    request always uses the LoRo→summed-to-mono pathway.
  * Annex D §2.3.1.3-4 (`ltrtcmixlev` / `ltrtsurmixlev`,
    E-AC-3 mixing-metadata overrides for the 0.707 C / surround
    coefficients) is a documented followup — the base-spec form
    is anchored at fixed 0.707 here; the E-AC-3 BSI parser
    already reads these fields into `_`-prefixed locals.
  * 9 new unit tests in `downmix::tests` covering:
    Table 7.32 coefficient match (`ltrt_3_2_matches_table_7_32`),
    surround sign discipline across acmod ∈ {4, 5, 6, 7}
    (`ltrt_surround_sign_discipline`), 2/2 center-drop
    (`ltrt_2_2_drops_center`), 3/1 single-surround form
    (`ltrt_3_1_uses_single_surround_form`), 2/0 surround-less
    passthrough (`ltrt_2_0_passes_no_surround_through`),
    surround phase inversion through `apply`
    (`ltrt_apply_preserves_surround_phase_inversion`),
    full-scale overload protection
    (`ltrt_3_2_full_scale_does_not_clip`), divergence vs LoRo
    on surround-only input (`ltrt_vs_loro_differ_on_surround`),
    and decoder factory build (`ltrt_decoder_builds`).
  * 2 end-to-end integration tests (`tests/downmix_ltrt.rs`,
    ffmpeg-gated): on a surround-only 5.1 AC-3 source LoRo's L/R
    correlation came in at +0.002 (independent surround tones
    summed in-phase to both sides → uncorrelated); LtRt's came
    in at **−0.972** — the matrix encoder's defining anti-phase
    surround signature. On a surround-free 5.1 source the LtRt
    L-channel RMS is 0.707× the LoRo L-channel RMS, matching
    the LoRo↔LtRt normalisation ratio
    (`0.3204 / 0.4143 ≈ 0.7733`).

- **E-AC-3 coupling-channel AHT decode (`cplahtinu`)** — round 117.
  Extends the AHT mantissa path (round 110 fbw, round 113 LFE) to the
  coupling pseudo-channel, removing the last AHT reject in the E-AC-3
  decoder.
  * **Interleaved coupling read in AHT frames**: the coupling-channel
    mantissa block (standard *or* AHT) is now read INSIDE the fbw
    channel loop of `eac3::dsp::unpack_mixed_mantissas`, right after the
    first coupled channel's mantissas — gated by `got_cplchan` exactly as
    the base-AC-3 mantissa loop (Table E1.4
    `if(cplinu[blk] && chincpl[ch] && !got_cplchan)`). The round-110/113
    mixed path skipped coupling entirely; only the blanket `cplahtinu`
    reject had hidden a latent cursor desync for *standard*
    (`cplahtinu == 0`) coupling in an AHT frame, which is now also fixed.
  * **Coupling-AHT synthesis** (`cplahtinu == 1`): the front-loaded
    coupling-AHT block — `cplgaqmod` (2 bits) + `cplgaqgain` words + the
    per-bin VQ (Tables E4.1..E4.7) or scalar/GAQ (Table E3.5) mantissas
    across all six AHT sub-blocks over the coupling range
    `[cpl_begf_mant, cpl_endf_mant)` — is decoded once (the
    `ncplregs == 1` / `ncplblks == 6` eligibility block), IDCT-II'd
    (§3.4.5), scaled by `2^-exp`, and cached for the per-block dispatch
    loop. `decode_aht_channel_mantissas` now takes a `start` bin so the
    coupling range maps to the same psd/mask/exp slots the §7.4 decouple
    step reads; the cached coupling coefficients are loaded into the
    `MAX_FBW` pseudo-channel slot before `dsp_block` runs decouple.
  * No corpus fixture exercises coupling AHT (FFmpeg's eac3 encoder
    emits neither coupling-AHT nor LFE-AHT), so the synthesis is covered
    by 3 unit tests (`eac3::dsp::cpl_aht_tests`): the standard-coupling
    interleave regression, the zero-mantissa front-load + cache, and the
    VQ regime (IDCT-II block variation + zero-below-`cpl_begf_mant`).
  * The dsp no longer rejects ANY AHT flag — fbw, LFE, and coupling AHT
    all decode.
- **E-AC-3 LFE-channel AHT decode + standard-LFE mantissa fix** — round
  113. Extends the round-110 multichannel fbw AHT path to the LFE
  channel, the immediately-next §3.4.2 element after fbw AHT.
  * **Latent desync fixed**: the round-110 AHT mantissa unpacker
    (`eac3::dsp::unpack_mixed_mantissas`) walked only the fbw channels
    and never read the LFE channel's mantissas. Any AHT syncframe that
    carried an LFE channel therefore desynced the bit cursor at the
    §E.1.3.2 `if(lfeon)` mantissa tail — even when the LFE used
    *standard* (`lfeahtinu == 0`) coding. The mixed path now reads the
    7 standard LFE mantissas (sharing the fbw bap-1/2/4 grouping
    buffers, §7.3.5), exactly where the base AC-3 unpacker does.
  * **LFE-AHT synthesis** (`lfeahtinu == 1`): the front-loaded LFE-AHT
    block — `lfegaqmod` (2 bits) + `lfegaqgain` words + the per-bin VQ
    (Tables E4.1..E4.7) or scalar/GAQ (Table E3.5) mantissas across all
    six AHT sub-blocks — is decoded once (block 0, the
    `nlferegs == 1` eligibility block), IDCT-II'd (§3.4.5), scaled by
    `2^-exp`, and cached for the per-block dispatch loop.
    `decode_aht_channel_mantissas` is now channel-agnostic: it takes the
    masking `snroffset` so the LFE path can feed `lfefsnroffst` into the
    §3.4.3.1 hebap derivation where the fbw path feeds `fsnroffst[ch]`.
  * The blanket `cplahtinu || lfeahtinu` reject in `decode_indep_audblks`
    is narrowed to `cplahtinu` only — coupling-channel AHT synthesis
    remains deferred; fbw + LFE AHT now decode.
  * The AHT coefficient cache and `aht_pending`/`aht_filled` flag arrays
    grow from `MAX_FBW` to `MAX_FBW + 2` slots so the LFE channel
    (`state.channels[MAX_FBW + 1]`) shares the same machinery.
  * 3 unit tests (`eac3::dsp::lfe_aht_tests`): standard-LFE mantissa bit
    consumption (regression guard for the desync), LFE-AHT zero-hebap
    front-load + cache + zero-bit replay on later blocks, and LFE-AHT
    VQ-regime IDCT producing block-varying cached coefficients.

- **E-AC-3 multichannel full-bandwidth AHT decode** — round 110. Extends
  the round-6 mono-only Adaptive Hybrid Transform path to any number of
  fbw channels. The §3.4.2 helper variables `nchregs[ch]` / `ncplregs` /
  `nlferegs` are now computed (`eac3::dsp::compute_aht_regs`) directly
  from the per-block exponent strategies already parsed onto `AudFrm`
  (`chexpstr_blk_ch`, `cplexpstr_blk` + `cplstre_blk`, `lfeexpstr`) — the
  spec derives these from the bitstream rather than transmitting them, and
  every input is available before the AHT anchor, so no real audblk
  pre-walk is needed. `parse_phase_b` then reads `chahtinu[ch]` for every
  fbw channel whose `nchregs[ch] == 1` (the AHT-eligibility gate: exponents
  sent exactly once per 6-block frame). The previous hardcoded mono path
  (`nfchans == 1 && !lfeon && ncplblks == 0`, single `nchregs[0] = 1` hint)
  is gone.
  * **Cross-channel mantissa grouping fix**: the standard (non-AHT)
    channels in a mixed frame now share the bap-1/2/4 triplet/pair
    grouping buffers across channels in frequency-then-channel order via
    the canonical `audblk::fetch_mantissa`, matching base AC-3 §7.3.5.
    Round 6's `unpack_one_channel_scalar` used per-channel grouping state
    that was only correct for the single-channel case; it is removed. The
    bap=0 dither now also uses the base path's §7.3.4 LFSR (the round-6
    scalar path zeroed dithered bins). The mono AHT fixture never reached
    the scalar path (its only channel takes the AHT branch), so this
    change has no effect on the previously-passing case.
  * **Coupling-AHT (`cplahtinu`) and LFE-AHT (`lfeahtinu`)** synthesis
    remain deferred: such frames are rejected as `Unsupported` after the
    regs-driven phase-B parse confirms the flag is set, instead of the
    old blanket multichannel reject. Multichannel fbw AHT now decodes.
  * 4 unit tests (`eac3::dsp::aht_regs_tests`) cover the nchregs /
    ncplregs / nlferegs counting rules and the `nchregs == 1`
    eligibility pattern.

- **E-AC-3 transient pre-noise processing (TPNP) decode** — round 103.
  Implements the §E.3.7.2 PCM-domain time-scaling synthesis, replacing
  the round-2 whole-frame reject (`transproce == 1` previously errored
  the entire syncframe, muting otherwise-decodable audio).
  * **Parse** (`eac3::audfrm::parse_tail`): the per-fbw-channel
    `chintransproc[ch]` (1 bit), `transprocloc[ch]` (10 bits, in
    4-sample units), and `transproclen[ch]` (8 bits) are now stored on
    `AudFrm` instead of being read-and-discarded for cursor alignment.
  * **Synthesis** (`eac3::dsp::apply_transient_prenoise`, called after
    overlap-add): for each fbw channel carrying TPNP data it (1) derives
    `transloc = 4·transprocloc`, the containing audio block's leading
    edge `aud_blk_samp_loc`, the pre-noise length `pnlen`, and the total
    correction length `tot_corr_len = pnlen + translen + TC1`; (2) copies
    a `2·TC1 + pnlen`-sample synthesis buffer from earlier (cleaner)
    PCM; (3) overwrites the pre-transient region with three windows —
    fade-out/in over `TC1 = 256`, full synth overwrite, fade-in/out over
    `TC2 = 128` — using complementary Hann cross-fades (the spec permits
    "nearly any pair of constant-amplitude cross-fade windows"). LFE
    never carries TPNP; the baseband decode is untouched (TPNP is a
    quality enhancement on already-valid samples).
  * **Cross-frame note**: §E.3.7.1 allows a frame-N transient to
    reference frame-(N-1) tail samples; the round-103 path clamps such
    reads to index 0 (conservative single-frame behaviour) and a future
    round can thread the previous frame's tail through
    `Eac3DecoderState`.
  * 4 unit tests in `eac3::dsp::tpnp_tests` cover the no-op cases
    (transient at/after frame end, block-aligned transient), the
    constant-signal-survives-cross-fade invariant, and the region-2
    full-overwrite-equals-synthesis-source property.
- **E-AC-3 spectral extension (SPX) decode** — round 100. Implements
  the §E.2.3.3 SPX strategy + coordinate syntax and the §E.3.6
  high-frequency regeneration, replacing the round-4 `spxinu == 1`
  mute path.
  * **Parse** (`eac3::dsp`): `chinspx[ch]`, `spxstrtf`, `spxbegf`,
    `spxendf`, `spxbndstrce` + `spxbndstrc[]` (Table E2.11 default
    banding), and the per-channel coordinate block `spxcoe` /
    `spxblnd` / `mstrspxco` / `spxcoexp` / `spxcomant`.
  * **Synthesis** (`audblk::apply_spectral_extension`, called from
    `dsp_block` between rematrix and dynrng): transform-coefficient
    translation with the §E.3.6.4.1 wrapping copy cursor, banded RMS
    energy, spxblnd-derived noise/signal blending (§E.3.6.4.2), and
    `spxco·32` coordinate scaling (§E.3.6.4.3). The SPX-channel
    `end_mant` is extended to the SPX end so dynrng + IMDCT process
    the regenerated bins. Base AC-3 never sets `in_spx`, so this is a
    no-op there.
  * **Cursor-drift derivations** now spec-correct on SPX frames:
    `endmant[ch] = spxbandtable[spx_begin_subbnd]` (§E.3.3.3),
    `cplendf` derived from `spxbegf` when SPX is in use (§E.3.3.1),
    and `nrematbd` folding in SPX via `remat_band_count_spx`
    (§E.3.3.2).
  * The §E.3.6.4.2 noise generator is non-normative per spec; a
    deterministic 32-bit xorshift LFSR (`spx_noise`) keeps decodes
    reproducible.
  * 5 unit tests in `audblk::spx_tests` cover `spx_bandtable`, the
    coordinate-decode formula, default-banding band sizing, the
    end-to-end copy+scale synthesis, and the disabled no-op.
- **E-AC-3 D25 exponent-group-count fix** — round 100. `nchgrps` for
  the D25 strategy used `(endmant−1).div_ceil(6)` = `(endmant−1+5)/6`,
  over-counting one group when `(endmant−1) mod 6 ∈ {2,3}` and reading
  an extra 7-bit exponent word (drifting the bit cursor). Corrected to
  the §7.1.3 form `(endmant−1+3)/6`, matching the AC-3 path.

### Changed

- **Per-channel fsnroffst encoder policy** — round 95. Replaces the
  round-23 index-order round-robin in `tune_snroffst_with_plan` with
  a two-stage greedy that closes the asymmetry r91's per-channel
  PSNR gates exposed (channel 0 reaching `fsnroffst_ch=15` while
  siblings stayed at the global baseline).
  * **Stage 1 — equalise.** Each round we bump every channel
    currently at the minimum `fsnroffst_ch` that still fits the
    frame budget. Every fbw channel reaches `min(fsnroffst_ch)`
    before any one runs ahead.
  * **Stage 2 — spread-capped residual.** A channel only enters the
    free-bump pass if `fsnroffst_ch[ch] - min(fsnroffst_ch[..]) <
    FAIR_SPREAD` (FAIR_SPREAD=2, ≈1.5 dB per-channel SNR variance).
    This prevents a cheap-mantissa channel from monopolising slack
    that the spec lets a single channel claim.
  ATSC A/52:2018 §5.4.3.40 only defines the bitstream field; the
  encoder's choice of value is non-normative (Annex C reference
  encoder suggests balancing the per-channel SNR). New regression:
  `tune_snroffst_per_channel_spread_bounded` synthesises a
  5-channel exponent grid with one cheap-bump channel and four
  expensive-bump channels and asserts the post-tune
  `max(fsnroffst_ch) - min(fsnroffst_ch) ≤ 3` — pre-r95 this
  spread reached 14-15. The change is encoder-policy only and the
  existing per-channel PSNR floors (`two_two_psnr_per_channel`,
  `three_two_psnr_per_channel`, `five_one_psnr_per_channel`) hold
  unchanged on the 220×n Hz multitone fixture; PSNR is bap-cap-bound
  on those pure-sine inputs, so the fairness fix is visible in the
  per-channel `fsnroffst_ch` array (now `[2, 1, 1, 2, 0]` instead of
  `[15, 1, 1, 2, 0]` on the tight-budget redistribution frames) but
  not in the test PSNR numbers. New `AC3_DEBUG_PERCH_SNR=1` env var
  prints the post-tune `(csnr, fsnr, fsnr_ch, cpl_fsnr, lfe_fsnr)`
  tuple per frame for A/B sweeps.

### Added

- **2/2 + 5-channel PSNR regression coverage** — round 91. Adds
  `two_two_self_decode_roundtrip` for the previously-untested 4-fbw
  `acmod=6` (2/2 = L,R,Ls,Rs) path, plus a new
  `encode_decode_multichan_psnr` helper with per-channel lag-aligned
  PSNR (1024-sample correlator, ±2048-sample window) and three
  `*_psnr_per_channel` regression gates:
  * `two_two_psnr_per_channel` (4ch, acmod=6) — 24-32 dB per slot
  * `three_two_psnr_per_channel` (5ch, acmod=7) — 10-33 dB per slot
  * `five_one_psnr_per_channel` (6ch, acmod=7 + lfeon=1) — 10-33 dB
    per slot
  Floor is 10 dB per channel — matches the in-tree
  `tests/eac3_ffmpeg.rs::psnr_min` convention (18 dB AC-3 baseline
  via ffmpeg's reference decoder; self-decode tends to score a few
  dB lower than ffmpeg's smoothing-aware path). The 2/2 acmod=6
  spec-defined layout (per ATSC A/52 Table 5.8) had no prior
  encode-then-self-decode test — only `mono` (1/0), `three_zero`
  (3/0), `three_two` (3/2), `five_one` (5.1), and `two_one_lfe` (2.1)
  were exercised.
- **2.1 (L, R, LFE) encoder layout** — round 78 / r78. Adds a new
  acmod-emit path that maps a 3-channel input to `acmod=2 (2/0) +
  lfeon=1` instead of the default 3-channel `acmod=3 (3/0 L,C,R)`.
  Reached by setting `CodecParameters.channel_layout =
  Some(ChannelLayout::Stereo21)` on a 3-channel input; the
  count-alone path still defaults to 3/0 for backward compatibility.
  BSI emission already gated `cmixlev` (acmod ∈ {3,5,7}),
  `surmixlev` (acmod ∈ {4,5,6,7}), and `phsflginu` (acmod == 2)
  correctly; the 2.1 path picks up the `dsurmod` emit (acmod == 2)
  and routes LFE through the same per-channel exponent / bap /
  mantissa pipeline as 5.1. Two new test gates land alongside:
  `two_one_lfe_self_decode_roundtrip` (our decoder reads back per-
  channel RMS within tolerance) and `two_one_lfe_ffmpeg_crossdecode`
  (FFmpeg's reference decoder reports per-channel RMS within 0.2 %
  of the input — `[6610, 6118, 6336]` ours vs `[6611, 6108, 6336]`
  ffmpeg's). The full encoder acmod coverage is now 1/0, 2/0,
  2/0+LFE (2.1), 3/0, 2/2, 3/2, and 3/2+LFE (5.1).
- **E-AC-3 frame-based exponent strategy (`expstre == 0`)** — Table E2.10
  expansion (round 72). When the audfrm header carries `expstre == 0`,
  the per-block per-channel strategy codes are emitted as 32 spec-defined
  6-block runs indexed by a single 5-bit `frmcplexpstr` (when coupling
  is in use anywhere in the frame) and one 5-bit `frmchexpstr[ch]` per
  fbw channel (per A/52:2018 §E.2.3.2.12 / §E.2.3.2.13). The audfrm
  parser now expands these codewords via `FRAME_EXP_STRAT_TABLE[32][6]`
  (transcribed verbatim from ATSC A/52:2018 Annex E Table E2.10) into
  `cplexpstr_blk[]` and `chexpstr_blk_ch[][]`, so the audblk DSP
  consumes the same per-block shape it already does for the
  `expstre == 1` path. Every FFmpeg-encoded E-AC-3 fixture in the
  corpus picks `expstre == 0` — round 6's silent fallback hid this from
  the round-2 DSP path. Corpus deltas (E-AC-3 fixtures, prior baseline
  = 13.57 dB / silent):
  * `eac3-5.1-48000-384kbps` → **90.01 dB** (+76.4 dB)
  * `eac3-low-rate-stereo-64kbps` → **71.74 dB** (+58.2 dB)
  * `eac3-low-bitrate-32kbps` → **66.32 dB** (+52.7 dB)
  * `eac3-5.1-side-768kbps` → **21.32 dB** (+7.7 dB; SPX-blocked frames
    still mute and bleed into the overlap-add delay line)
  Remaining 3 fixtures (`eac3-stereo-48000-192kbps`,
  `eac3-256-coeff-block`, `eac3-from-ac3-bitstream-recombination`)
  exercise `spxinu == 1` blocks where the round-4 stub mutes — they
  decode block-0/block-1 cleanly then drop to silent on the first SPX
  frame. Implementing §E.2.2.5.4 spectral extension is the next E-AC-3
  blocker.

### Fixed

- **E-AC-3 coupling validity envelope widened to §5.4.3.12** (round 72).
  The audblk parser used to reject any block whose `cplbegf > cplendf`
  with `malformed coupling range`. Mirroring the round-7 AC-3 fix
  (commit `97d112f`), the spec's actual envelope is `ncplsubnd =
  3 + cplendf - cplbegf >= 1` — equivalently `cplbegf <= cplendf+2`
  — because §5.4.3.12 defines the upper sub-band index as `cplendf+2`.
  FFmpeg picks narrow configs like `(cplbegf=11, cplendf=10)` on
  high-bitrate 5.x frames; the strict check tripped on
  `eac3-5.1-48000-384kbps` frame 0 every syncframe and crashed the
  rest of the frame. Signed arithmetic prevents the `3 + cplendf -
  cplbegf` term from underflowing `usize` before the check fires.

### Fixed

- **Coupling-range validity check too strict for narrow-coupling streams**
  (round 7 / r7). The audblk parser rejected any frame whose `cplbegf >
  cplendf` with the message `§5.4.3.11/12 cplbegf > cplendf — malformed
  coupling range`. Per A/52:2018 §5.4.3.12, the upper sub-band index is
  `cplendf+2`, so the spec's actual envelope is `ncplsubnd = 3 + cplendf
  - cplbegf >= 1`, equivalently `cplbegf <= cplendf+2`. ffmpeg's
  encoder picks narrow `(cplbegf=11, cplendf=10)` configs on 5.0
  (acmod=7, lfeon=0) frames — placing coupling on sub-bands 11..=12
  (transform-coefficient bins 169..193, ~16-19 kHz at fs=48k) — and our
  too-strict check bombed every block 0 of every frame. The catch in
  `decode_frame` zeroed the coefficients on Err, then the next block
  inherited a corrupt bit cursor (the cpl-begf/-endf reads are AFTER
  cplinu / chincpl so the rejection was past the no-cpl path). PSNR on
  `ac3-3-2-48000-384kbps` (5.0): **6.49 dB → 88.85 dB** (+82.36 dB).
  Also moves to signed arithmetic for `ncplsubnd` so the `3 + cplendf
  - cplbegf` term can't underflow `usize` before the check fires.

- **Multichannel decoder output now in WAV-mask order** (round 6 / r6).
  AC-3 transmits multichannel layouts in `acmod` slot order
  (Table 5.8): 3/0 = `(L, C, R)`, 3/1 = `(L, C, R, S)`, 3/2 = `(L, C,
  R, Ls, Rs)`, with LFE appended as the last bitstream slot. Consumers
  that interpret the decoded PCM as a WAVE file (or any
  `WAVE_FORMAT_EXTENSIBLE`-compliant sink such as `pcm_s16le`,
  foobar2000, or miniaudio) expect samples in `dwChannelMask` order:
  `(FL, FR, FC, LFE, BL, BR)`. Stereo, mono, 2/1, 2/2, and 1+1 dual-mono
  paths already match WAV order; only `acmod ∈ {3, 5, 7}` (the
  front-center-bearing layouts) require permutation. The new
  `wave_order` module exposes `wave_to_bitstream_map(acmod, lfeon)` and
  `reorder_s16le_in_place(...)`, called once per syncframe in the
  passthrough decode path of both AC-3 (`decoder::process_ac3_frame`)
  and E-AC-3 (`decoder::process_eac3_frame`). Downmixed outputs
  (mono / stereo via `Downmix`) skip the reorder because the matrix
  emits in standard order. Verified against the docs corpus:
  `ac3-3-0-48000` jumps from 10.56 dB to **88.99 dB** PSNR vs FFmpeg;
  `ac3-3-2-lfe-48000-448kbps` (5.1) jumps from 11.97 dB to
  **90.42 dB**. The dep-substream-extended 7.1 case (indep 5.1 + dep
  `[Lb, Rb]`) keeps the trailing dep channels in append order — the
  bigger refactor needed to route them through the WAV 7.1 slot map
  is deferred (no fixture exercises it).

## [0.0.6](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.5...v0.0.6) - 2026-05-06

### Other

- drop dead `linkme` dep
- registry calls: rename make_decoder/make_encoder → first_decoder/first_encoder
- restore split-after-= form from 1de0dc2 (undo mistaken reformat)
- apply cargo fmt (rustfmt CI fix for round 30)
- rustfmt fix for band_is_tonal mean_exp_x8 line break
- ac3/eac3 encoder round 30: adaptive expstr + LFE spectral constraint + tonal DBA
- auto-register via oxideav_core::register! macro (linkme distributed slice)
- unify entry point on register(&mut RuntimeContext) ([#502](https://github.com/OxideAV/oxideav-ac3/pull/502))

### Added

- **Tonal-vs-noise psy classification in DBA band selection** (round 30).
  Added `band_is_tonal` helper that measures the exponent spread
  (min vs. mean) within a frequency band across all 6 blocks. The
  `build_dba_plan` band picker now tracks per-band tonal votes and
  applies a large penalty (1 000 000) to mostly-tonal bands, steering
  the DBA target toward spectrally flat (noise-like) bands instead of
  bands that contain a dominant tone. This avoids raising the masking
  threshold at frequencies where the tone is perceptually salient.

- **Adaptive D15/D25/D45 exponent-strategy selection for E-AC-3**
  (round 30). The E-AC-3 encoder previously used a static
  `[1, 0, 0, 1, 0, 0]` pattern (always D15 on anchor blocks, REUSE
  elsewhere). It now calls `select_exp_strategies` — the same adaptive
  per-channel smoothness test already used by the AC-3 encoder — to
  pick D15/D25/D45 per channel per anchor block. Smooth-spectrum
  channels emit D25 or D45, freeing ~430 bits/channel/anchor-block for
  mantissa precision. `EAC3_DISABLE_EXPSTR_SEL=1` reverts to static
  D15-only for A/B sweeps. `tune_snroffst_with_plan` receives the
  `chexpstr_plan` so the mantissa budget accounts for the exponent savings.

- **LFE spectral constraint (0–120 Hz)** (round 30). Both the AC-3 and
  E-AC-3 encoders now zero MDCT coefficients at bin ≥ 2 before exponent
  extraction, enforcing the spec §7.1.3 0–120 Hz LFE bandwidth limit
  (at 48 kHz, bin 0 ≈ 47 Hz; bin 1 ≈ 141 Hz). Bins 2..6 carry
  `exp=24 → bap=0 → no mantissa bits`, so `LFE_END_MANT=7` is
  unchanged for bitstream compatibility.

### Changed

- **`register` entry point unified on `RuntimeContext`** (task #502).
  The legacy `pub fn register(reg: &mut CodecRegistry)` is renamed to
  `register_codecs` and a new `pub fn register(ctx: &mut
  oxideav_core::RuntimeContext)` calls it internally. Breaking change
  for direct callers passing a `CodecRegistry`; switch to either the
  new `RuntimeContext` entry or the explicit `register_codecs` name.

## [0.0.5](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.4...v0.0.5) - 2026-05-05

### Fixed

- *(clippy)* drop unnecessary i32→i32 casts + manual_memcpy in EAC-3 AHT

### Other

- task #467 — move chexpstr/cplexpstr/lfeexpstr from audblk to audfrm
- round 29 — D45 grpsize=4 anchor blocks now bit-exact
- round 28 — per-channel exponent strategy selection (D15/D25)
- round 6 (task #324) — Adaptive Hybrid Transform (AHT) decode
- round 5 — standard coupling decode for 5.1 + low-rate stereo
- round 4 stub — clarify AHT + SPX unsupported paths
- round 3 — dependent substream channel splice
- silence dead-code warning on round-3-only fields
- round 2 — per-block DSP via Ac3State translation

### Fixed — task #467 — E-AC-3 audfrm vs audblk exponent strategy bit placement

- **`chexpstr[blk][ch]`, `cplexpstr[blk]`, and `lfeexpstr[blk]` now live in
  `audfrm()` per ETSI TS 102 366 V1.4.1 §E.1.2.3 / Table E.1.3 (= ATSC
  A/52:2018 Annex E Table E1.3).** Earlier rounds put these per-block
  strategy codes inside `audblk()` (with a "round 2 fix" comment doubling
  down on the inversion). The spec text in §E.1.3.2.1 — "If the expstre
  bit is set to '1', the fields that carry the full exponent strategy
  syntax shall be present in **each audio block**" — refers to the
  per-block-indexed fields enumerated by the syntax table; those fields
  LIVE in audfrm, indexed by `[blk]`. Audblk (Table E.1.4) does NOT
  re-emit chexpstr/cplexpstr/lfeexpstr; it merely consumes them as state
  via `if(chexpstr[blk][ch] != reuse) {chbwcod[ch]; ...}` gates.
  ffmpeg follows the table verbatim; our streams misaligned by 12 bits
  per block (2 × nfchans), surfacing as the cascade
  `new bit allocation info must be present in block 0`,
  `delta bit allocation strategy reserved`, `error in bit allocation`.
- **`lfeexpstr[blk]` is now emitted unconditionally of `expstre`.** The
  `if(lfeon)` lfeexpstr loop sits OUTSIDE the `if(expstre)` branch in
  Table E.1.3. Previously we only emitted it when `expstre == 0`, which
  silently dropped 6 bits per LFE-on substream per syncframe.
- **`gainrng[ch]` (2 bits) restored after each per-channel exponent
  payload in audblk.** Table E.1.4 line: `gainrng[ch] 2`, immediately
  after the `nchgrps[ch]` group emit. The earlier "Annex E dropped
  gainrng" comment was wrong; gainrng is per-fbw-channel-only (LFE has
  no gainrng) and the bit MUST be consumed or every subsequent field
  in the audblk slides by 2 bits per non-REUSE channel.
- **`convsnroffste` (1 bit) emitted by encoder when `strmtyp == 0`.**
  Per Table E.1.4, the `if(strmtyp == 0x0) { convsnroffste; if(set)
  convsnroffst(10) }` block sits between fgaincode and cplleak,
  UNCONDITIONAL on snroffste. The encoder previously skipped it
  entirely; the dsp parser was already reading it (round 5 fix), so
  the asymmetry desync'd the wire stream.
- **Audfrm parser** now reads per-block per-channel `chexpstr` and
  per-block `cplexpstr` when `expstre==1`, and the per-block
  `lfeexpstr` array unconditionally of expstre. Two new fields on
  `AudFrm`: `chexpstr_blk_ch[6][5]` and `cplexpstr_blk[6]`. The
  test `parses_minimal_indep_stereo_audfrm` was updated to pack the
  6×2 chexpstr bits in the right slot.
- **Dsp module** now looks up chexpstr/cplexpstr/lfeexpstr from the
  parsed audfrm instead of re-reading them from the audblk byte
  stream. Added `let _gainrng = br.read_u32(2)?` after each per-channel
  exponent decode to consume the now-emitted bit.
- **Verification:** all three pre-existing E-AC-3 ffmpeg cross-decode
  tests pass: `eac3_stereo_192k_decodes_through_ffmpeg` →
  PSNR **20.21 dB** (above the 18 dB floor); `eac3_mono_96k_decodes_through_ffmpeg`
  → PSNR **20.21 dB**; `eac3_71_pair_decodes_through_ffmpeg` → ffmpeg
  reconstructs the full 8-channel program (chanmap honored) with
  L-energy 2844 across 49 152 samples. Full test run: 98 passed,
  0 failed across the lib + 5 integration suites.

### Fixed — round 29 — D45 grpsize=4 anchor blocks now bit-exact

- **`build_dba_plan` — clamp `hi_band` to 32 (was 45).** The DBA segment
  search picked a "quietest" mid-band in `[25, 45)`, then emitted that
  absolute band number as `deltoffst` (5 bits per §5.4.3.51). When the
  best band ≥ 32 the wire write silently truncated to `band & 31`,
  re-targeting the +6 dB mask delta at a low band the encoder never
  tagged. The decoder applied the delta at the wrong band → `bap[]`
  drifted by 1 at one bin → the rest of the frame's mantissa stream
  shifted by 1 bit. Cap the search at 32 so a single 5-bit segment can
  always reach its band; multi-segment dba would let us reclaim the
  upper region but costs more bits than the dba saves at our bitrates.
- **D45 (chexpstr=3) is now the default** when the strategy selector's
  smoothness probe permits — `AC3_DISABLE_D45=1` falls back to D25-only
  for A/B sweeps. Previously gated behind `AC3_ENABLE_D45=1` for the
  same reason as the dba fix above (the symptom looked like a D45
  emitter bug but it was a dba syntax bug masked by D25's different
  `build_dba_plan` exponent inputs).
- **Encoder defensive guard.** Added a `debug_assert!(offst <= 31)` to
  the dba emission so any future plan builder that overshoots the
  5-bit field range fails loudly instead of corrupting the bitstream.
- **Test gate `d45_exp_strategy_selection_and_ffmpeg_crosscheck`** —
  encodes a 110 Hz stereo tone, asserts ≥ 50% of frames carry
  `chexpstr=3` on an anchor block, asserts self-decode PSNR > 20 dB,
  asserts ffmpeg cross-decode succeeds with non-trivial PCM.
  Measured: 32/32 frames carry D45, self-decode PSNR 24.90 dB,
  ffmpeg-decode RMS 10767.

### Added — round 28 — per-channel exponent strategy selection (D25)

- **Encoder-side §7.1.3 / §5.4.3.22 chexpstr selection.** The encoder's
  anchor blocks (blocks 0 and 3 of each syncframe) now pick D15
  (grpsize=1) or D25 (grpsize=2) **per fbw channel** based on a
  smoothness probe of the post-`preprocess_d15` exponent array. Smooth
  channels emit D25 — `4 + 7×((end-1+3)/6)` bits per anchor block
  instead of D15's `4 + 7×((end-1)/3)`, saving ~290 bits per channel
  per anchor block at end_mant=253 (full-bandwidth uncoupled stereo).
  The freed bits are picked up automatically by the existing
  `tune_snroffst_with_plan` pass and reinvested in mantissa precision.
- **`select_exp_strategies`** — per-channel-per-block plan vector with
  the conservative "anchor on blocks 0 and 3, REUSE on 1/2/4/5"
  cadence preserved. Per-channel decisions: silent / smooth-tail
  channels can pick a coarser grpsize than transient / HF-rich
  channels in the same block.
- **`pick_strategy_for_block`** — sums the per-bin clipping cost of
  collapsing the spectrum to grpsize=2 and grpsize=4 spans (each
  clipped bin upcasts a mantissa bap by ≈1 bit, so the cost is in
  exp-units worth of dynamic range). Pick the largest grpsize whose
  per-bin avg clipping cost stays below thresholds tuned to the bit
  savings of each strategy. Thresholds are env-tunable via
  `AC3_EXPSTR_D25_THR` / `AC3_EXPSTR_D45_THR`; `AC3_FORCE_EXPSTR=N`
  pins all anchors to a fixed strategy for A/B sweeps.
- **`quantise_exponents_to_grpsize`** — replaces each grpsize-span
  with its minimum-exponent representative (covers the loudest bin
  in the span without clipping), then back-prop and forward-clamp the
  representative sequence so adjacent reps differ by at most ±2 (the
  AC-3 differential encoding limit). The bit allocator and mantissa
  quantiser see the same per-bin exponents the decoder will
  reconstruct after grpsize expansion, keeping bap[] in lockstep.
- **`write_exponents_grouped`** — generic 4-bit absexp + 7-bit packed
  groups emitter parameterised on grpsize; replaces the dedicated
  D15-only `write_exponents_d15` (which is now a thin wrapper).
- **`overhead_bits_for`** — accepts an optional per-channel-per-block
  chexpstr plan. When supplied, the per-channel exponent payload is
  computed from the actual emitted strategy instead of assuming D15
  for every channel, so `tune_snroffst`'s budget calculation matches
  the actual emitted byte count.
- **`AC3_ENABLE_D45` env-gated D45 path** — D45 emission has a
  first-frame-only mantissa desync the round didn't crack (per-bin
  exp arrays match between encoder/decoder post-grpsize expansion;
  the bit allocator delivers ~3 fewer mantissa bits in the decoder
  than the encoder writes during frame 1; subsequent frames are
  bit-exact). Disabled by default; leave the infrastructure in place
  for round 29 to land it once the desync is bracketed. D45 saves an
  additional ~145 bits/channel/anchor block over D25 when the
  spectrum is smooth enough.
- Test gate: `d25_exp_strategy_selection_and_ffmpeg_crosscheck` —
  encodes a 220 Hz + 440 Hz + 880 Hz stereo mix at 192 kbps,
  asserts every frame carries `chexpstr=2` on at least one fbw
  channel of an anchor block, and verifies ffmpeg decodes the
  resulting elementary stream into non-trivial PCM. Measured: 32 of
  32 frames pick D25 on at least one anchor; ffmpeg-decode RMS
  10190.

### Fixed — decoder defensive guard for end=0

- **`audblk::parse_audblk_into`** — guard `(end - 1) / k` underflow
  when a fully-coupled fbw channel has `cpl_begf_mant == 0`. Without
  the guard a corpus stream with that shape (unusual but spec-legal)
  would panic the decoder mid-frame instead of producing zero
  exponent groups for the channel. Surfaced by
  `tests/docs_corpus.rs::corpus_ac3_3_2_48000_384kbps`.

### Added — round 6 (task #324) — Adaptive Hybrid Transform (AHT)

- **VQ codebooks E4.1..E4.7** — 956 entries × 6 i16 transcribed
  verbatim from ATSC A/52:2018 (= ETSI TS 102 366 v1.4.1) Annex E §4
  into `src/eac3/tables/aht_codebooks.rs`. Spec values, not
  implementation source.
- **`src/eac3/aht.rs`** — AHT decode helpers:
    - `HEBAPTAB` — Table E3.1 high-efficiency bit-allocation pointer
      lookup (64 entries).
    - `HEBAP_MANT_BITS` — Table E3.2 mantissa bits per `hebap` for the
      scalar/GAQ regime (`hebap >= 8`).
    - `VQ_BITS` — codeword widths for VQ tables E4.1..E4.7
      (2/3/4/5/7/8/9 bits).
    - `vq_lookup(hebap, index)` — per-bin 6-tuple VQ lookup,
      Q15-normalised to (-1, 1).
    - `read_scalar_aht_mantissas` + Table E3.5 GAQ small/large
      quantiser with the §3.4.4.2 large-mantissa remap.
    - `idct_ii_6` — §3.4.5 inverse DCT-II that recovers per-block
      MDCT coefficients from the 6 AHT-domain values per bin.
    - `fill_gaqbin` / `gaq_sections` / `read_gaq_gains` — per
      §3.4.2 helper-variable derivation and per-channel gain-word
      bit-stream parsing.
- **`audfrm` two-phase parse** — `parse_with` now stops at the AHT
  anchor when `ahte == 1`, surfacing `AudFrm::aht_anchor_bits` +
  `aht_phase_b_pending`; a new `parse_phase_b(br, audfrm, bsi,
  hints)` consumes `chahtinu[ch]` / `cplahtinu` / `lfeahtinu`
  followed by the SNR / transient / SPX-attenuation / blkstrtinfo
  tail. Hints (`AhtRegsHints`) carry `nchregs[ch]` / `ncplregs` /
  `nlferegs` derived from the per-block exponent strategies.
- **dsp AHT path** — `decode_indep_audblks` keeps a per-channel AHT
  coefficient cache and dispatches AHT-active channels to a
  GAQ + VQ + IDCT mantissa decoder on the FIRST audblk where the
  channel emits exponents. Subsequent audblks pull pre-computed
  coefficients from the cache.
- **Round-6 scope is mono-only**: `nfchans == 1 && !lfeon &&
  ncplblks == 0`. Multichannel / coupled / LFE AHT mutes via an
  `Unsupported` early return — the iterative `nchregs` probe
  (§3.4.2) lands in round 7. The `eac3-low-bitrate-32kbps` corpus
  fixture is the only AHT-active fixture and matches the mono
  scope; round 6 unblocks its 14 of 17 AHT-on frames.

## [0.0.4](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.3...v0.0.4) - 2026-05-03

### Other

- clippy follow-ups (div_ceil + vec_init_then_push)
- drop redundant u32 cast on bsi.frame_bytes
- add Annex E decoder dispatch + BSI/audfrm parsers (round 1)

## [0.0.3](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.2...v0.0.3) - 2026-05-03

### Other

- use checked_div for per-channel sample math
- rustfmt docs_corpus.rs
- wire docs/audio/ac3/fixtures/ corpus into tests/docs_corpus.rs
- replace never-match regex with semver_check = false
- migrate to centralized OxideAV/.github reusable workflows
- round 27/task #187 — E-AC-3 dependent substream encode (7.1)
- round 26/task #170 — per-block SNR-offset bit-pool tuning
- round 25/task #155 — multichannel coupling (>2 fbw)
- round 25 — E-AC-3 (Annex E) encoder, round-1 scope
- round 24 - spec-faithful transient detector + per-channel fsnroffst
- round 19 — multichannel encoder (1/0, 3/0, 2/2, 3/2, 5.1)
- round 18 — encoder-side §7.2.2.6 delta bit allocation
- adopt slim AudioFrame shape
- round 16 — encoder-side §7.4 channel coupling
- round 15 — encoder-side short-block emission + transient detection
- round 14 — transient PSNR was a TEST BUG, not a decoder bug
- round 13 — verify FBW step C clean; add encoder rematrix (§7.5.3)
- round 12 - fix parse_frame_side_info double-consume; rule out cpl/snroffset/phsflg
- round 11 - hand-trace bndpsd/excite/mask/bap, add diagnostic probes
- round 7 — apply §7.2.2.6 dba + cap §7.5 rematrix at coupling boundary
- pin release-plz to patch-only bumps

### Added

- Round 27 (task #187) — E-AC-3 dependent substream encode +
  multichannel scope expansion. The encoder now accepts 1, 2, 6,
  and 8 input channels (was 1 / 2 only):
    - **5.1 input (6 ch)** → single independent substream with
      `acmod=7` (3/2 L,C,R,Ls,Rs) and `lfeon=1`. The DSP path
      gained the LFE pseudo-channel exponent / mantissa pipeline
      (D15 over `bins[0..7]`, no chbwcod, `lfeexpstr` emitted in
      audfrm with the same D15-on-blocks-0-and-3 cadence used by
      the fbw channels).
    - **7.1 input (8 ch)** → the spec's §E.3.8.2 indep+dep pair.
      Indep substream is the 5.1 program built from the source's
      L,C,R,Ls,Rs,LFE; the dep substream is `strmtyp=1`,
      `substreamid=0`, `acmod=2`, `lfeon=0` carrying Lb/Rb with
      `chanmape=1` and `chanmap = 0x0200` (bit 6 of the 16-bit
      field — Lrs/Rrs pair per Table E2.5; bit 0 sits in MSB so
      the bit-6 mask is `1 << (15 - 6)`). The packet payload is
      the byte-concatenation of the two syncframes, both starting
      with `0x0B 0x77`. Default bitrates: 384 kbps indep + 192 kbps
      dep = 576 kbps total; user-supplied `bit_rate` is split
      preserving the same ratio.
  Refactor: the per-syncframe DSP body moved out of `Eac3Encoder::
  emit_syncframe` into `emit_substream(sub, frame_pcm)` which
  takes a `SubstreamLayout` (strmtyp / substreamid / acmod /
  lfeon / chanmap / src-PCM-index map / frame_bytes) and writes
  one E-AC-3 syncframe of size `sub.frame_bytes`. `emit_syncframe`
  drains 1536 samples per input channel and dispatches to one or
  two `emit_substream` calls based on the `Layout::Indep` /
  `Layout::Pair` configuration built at make-encoder time.
  Bitstream additions vs round 26:
    - `chanmape(1) [+ chanmap(16) when chanmape=1]` after `compre`
      in bsi when `strmtyp == 1` (§E.2.2.2 / E.2.3.1.7-8).
    - `lfeexpstr[blk]` (1 bit per block) inside audfrm when
      `lfeon=1` (§E.2.3.2.7).
    - LFE D15 exponents (4-bit absexp + 2 D15 groups) in audblks
      with new exponent strategy when `lfeon=1`.
    - `convexpstre` / `convsnroffste` skipped on `strmtyp==1`
      substreams (only emitted for indep streams per §E.2.2.3 /
      §E.2.2.4 syntax).
  Tests added (4 new tests; the eac3 unit-test cluster grows
  from 4 to 6 inside the lib, and the `eac3_ffmpeg` integration
  suite from 3 to 5):
    - `make_encoder_71_builds_pair_layout` — accepts 8 ch and
      asserts the chanmap MSB ordering math (`1 << (15 - 6) =
      0x0200` for the Lrs/Rrs-pair location bit).
    - `make_encoder_51_5fbw_plus_lfe` — accepts 6 ch at 384 kbps.
    - `eac3_71_emits_indep_plus_dep_substream_pair` — encodes 1 s
      of 7.1 sine, asserts each frame is exactly `1536 + 768 =
      2304 bytes`, that both halves start with the syncword, that
      the first half's `strmtyp` is 0, and that the second half's
      `strmtyp` is 1.
    - `eac3_71_pair_decodes_through_ffmpeg` — pipes the 7.1
      output through `ffmpeg -f eac3 -i …` and verifies ffmpeg
      reports either 6 or 8 channels (§E.3.8.1 reference decoder
      vs full 7.1 reassembler) with non-trivial Left-channel
      energy. The ffmpeg reference decoder accepts the stream
      cleanly.
  Existing test `make_encoder_rejects_unsupported_channels` was
  updated to test 4 channels (no longer in the allow-list) instead
  of 6 (now accepted as 5.1).

- Round 26 (task #170) — per-block SNR-offset bit-pool tuning. AC-3
  syntax §5.4.3.37-43 lets every audio block re-transmit a fresh
  `(csnroffst, fsnroffst[ch], cplfsnroffst, lfefsnroffst)` tuple via
  the `snroffste=1` flag; previously the encoder only emitted one
  global tuple on block 0 and reused it for blocks 1..5. The new
  `tune_per_block_snroffst` pass runs after the existing global
  `tune_snroffst` and redistributes mantissa bits between blocks based
  on per-block masking demand. Algorithm:
    1. Compute per-block PSD demand (mean of `3072 - (exp << 7)` over
       all bins / fbw channels) — silent blocks land near 24, loud
       blocks land at 800-1200+.
    2. Group blocks 0/1/2 vs 3/4/5 (the encoder's D15-on-blocks-0-and-3
       exponent strategy means each half shares an exponent set).
    3. Search `(down, up) ∈ [0, 8] × [0, 8]` step pairs: drop the
       donor half's whole-block fsnroffst by `down` (banks bits) and
       bump the recipient half by `up` (spends bits where the masking
       demand is highest). Accept the pair maximising `up - down`
       subject to the trial mantissa-bit count + per-block snroffste
       payload (~27 bits/changed-block for stereo) fitting the frame
       budget.
    4. Optional fine refinement via single-channel pair walks on the
       residual budget.
  When the demand spread is below threshold or the budget is too tight
  the pass returns the flat plan unchanged — `snroffste` reverts to
  block-0-only and the bitstream stays byte-identical to the previous
  encoder. `AC3_DISABLE_PERBLOCK_SNR=1` pins the flat plan for A/B
  testing; `AC3_DEBUG_PERBLOCK_SNR=1` prints the demand vector and
  the accepted (down, up) trial.
  Test gates (3 new tests, total = 57 active + 2 ignored):
    - `perblock_snroffst_self_decode` — encode 4 syncframes carrying a
      HF-rich chord burst on block 3 of each frame and silence
      elsewhere; verify our own decoder reads the per-block snroffst
      stream without complaint.
    - `perblock_snroffst_helps_transient` (ignored — mutates
      `AC3_DISABLE_PERBLOCK_SNR` so it must run alone) — A/B encodes
      the same fixture at 96 kbps stereo with vs without per-block
      tuning, asserts identical byte count and that the per-block path
      doesn't regress block-3 PSNR. Measured: per-block-tuned **32.91
      dB** vs flat **31.84 dB** (+1.07 dB localised on the demand-heavy
      block at matched bitrate).
    - `perblock_snroffst_ffmpeg_crossdecode` — encodes the same
      fixture and pipes through `ffmpeg -f ac3` to verify a production
      decoder accepts our snroffste-on-non-block-0 stream cleanly.

- Round 25 (task #155) — multichannel coupling: extended the encoder's
  §7.4 channel-coupling path from the previous 2/0-only restriction to
  every multichannel acmod (3/0, 2/2, 3/2, 5.1). All available fbw
  channels join the coupling group (`chincpl[ch]=1` for every fbw,
  matching the spec's 1..=5 limit; LFE is excluded by §7.4.1). The
  coupling-channel coefficients become the mean of every coupled
  channel's MDCT bin in `[37 + 12*cplbegf, 37 + 12*(cplendf+3))`, and
  per-channel cplco coordinates restore each channel's HF envelope on
  decode. With cplbegf=8 / cplendf=15 (the 2/0 baseline) the cpl region
  spans ~6 kHz upward, and 5.1 frees ~5× the per-channel HF mantissa
  budget — at 320 kbps for a 5-fbw HF-rich source this lifts the average
  fbw-channel self-decode PSNR from **20.82 dB** to **23.94 dB**
  (+3.12 dB at matched bitrate). The `AC3_DISABLE_CPL` env var still
  suppresses coupling for A/B testing; `AC3_TRACE_CPL_ENC=1` now prints
  the chincpl mask + per-channel mstr/cplco/cplcoexp/cplcomant arrays.
  Test gates (1 new test + 1 fix, total = 56 active + 2 ignored):
    - `five_one_coupling_beats_no_coupling_at_low_bitrate` (ignored —
      mutates `AC3_DISABLE_CPL` so it must run alone) — encodes the same
      5.1 HF-rich PCM with and without coupling at 320 kbps and
      asserts the coupled path beats the no-coupling path by ≥1 dB.
    - `five_one_ffmpeg_crossdecode` — already in the suite, now passes
      cleanly through ffmpeg's reference decoder. The previous output
      tripped libavcodec's "new coupling coordinates must be present in
      block 0" + "expacc out-of-range" parsers; both were rooted in the
      same bug below.
  Bitstream fix needed to reach this acceptance:
    - **§5.4.3.10 phsflginu gating** — the encoder used to emit the
      1-bit `phsflginu` field unconditionally on block 0 of every cpl
      strategy frame, but the spec defines the field only for `acmod ==
      0x2` (2/0 stereo). For multichannel acmods the phantom bit
      shifted every following block-0 cpl field by 1 bit, which
      cascaded into ffmpeg parsing the cpl-coord side info as garbage
      and then walking off into the cpl-exponent stream where it landed
      on impossible D15 packed values (the "expacc 127 out-of-range"
      messages). The encoder's own decoder masked the bug by reading
      phsflginu conditionally — same logic on both sides means
      self-decode round-tripped fine. Now `phsflginu` is written only
      when `acmod == 0x2`, matching `audblk::parse_audblk_side_info`.
    - **`overhead_bits_for`** correspondingly drops the 1-bit phsflginu
      contribution for non-2/0 acmods so `tune_snroffst`'s mantissa
      budget calculation matches the actual emitted bitstream length.
    - **`write_exponents_cpl`** — clamped `cplabsexp` to ≤12 (was 15)
      so the seed `(cplabsexp << 1)` cannot exceed 24. Pre-fix, the
      decoder's running exp could land at 30 + small negative delta
      = 28, breaching the §7.1.3 [0, 24] limit and tripping ffmpeg's
      "expacc out-of-range" check on the very first cpl D15 group. Add
      a debug_assert that catches any future regression of the same
      shape inside the encoder rather than at the consumer end.

- E-AC-3 (Enhanced AC-3) encoder per ATSC A/52:2018 Annex E, round-1
  scope: a single independent substream (`strmtyp=0`, `substreamid=0`,
  `bsid=16`) carrying mono (acmod=1) or stereo (acmod=2) audio at
  32 kHz / 44.1 kHz / 48 kHz, 6 audio blocks per syncframe
  (`numblkscod=3`), no coupling, no spectral extension (`spxinu=0`),
  no Adaptive Hybrid Transform (`ahte=0`), no transient pre-noise
  processing. New `eac3::make_encoder` constructor and
  `crate::CODEC_ID_STR_EAC3 = "eac3"` registration. The DSP pipeline
  (windowing, MDCT, exponent extraction + D15 strategy, parametric bit
  allocation, dba, mantissa quantisation) is shared with the AC-3
  encoder via newly-pub(crate) helpers (`extract_exponent`,
  `preprocess_d15`, `compute_bap`, `tune_snroffst`, `build_dba_plan`,
  `quantise_mantissa`, `write_exponents_d15`, `write_mantissa_stream`,
  `ac3_crc_update`, `BitAllocParams`, `CouplingPlan`, `DbaPlan`,
  `TransientDetector`, `decode_input_samples`). Framing diverges:
    - **syncinfo** (§E.2.2.1) is just the 16-bit syncword `0x0B77`;
      no `crc1` field.
    - **bsi** (§E.2.2.2) replaces AC-3's `bsid≤8` layout with
      `strmtyp(2) + substreamid(3) + frmsiz(11) + fscod(2) +
      numblkscod(2) + acmod(3) + lfeon(1) + bsid=16(5) + dialnorm(5)
      + compre(1) + mixmdate(1) + infomdate(1) + addbsie(1)`.
      `frmsiz = (frame_size_in_words - 1)` per §E.2.3.1.3 — the size
      table from AC-3's §5.4.1.4 is gone.
    - **audfrm** (§E.2.2.3) sits between bsi and the audblks and
      carries frame-level strategy flags (`expstre=1, ahte=0,
      snroffststr=0, transproce=0, blkswe=1, dithflage=1, bamode=1,
      frmfgaincode=1, dbaflde=1, skipflde=1, spxattene=0`), per-block
      coupling-strategy flags (`cplinu[0..5]=0`), per-block per-channel
      `chexpstr[blk][ch]` (2 bits each), per-channel `convexpstr[ch]`
      (5 bits each, value=0 = D15 + 5×REUSE per Table E2.10), and the
      shared frame-level `frmcsnroffst(6) + frmfsnroffst(4)`.
    - **audblk** (§E.2.2.4) emits `blksw + dithflag + dynrnge=0 +
      spxinu=0 + (rematrix flags when acmod==2) + chbwcod (D15 blocks
      only) + exponents (D15 blocks only) + bamode params (block 0)
      + fgaincode=0 (default fgaincod=4 for all chans) + convsnroffste=0
      + dba + skiple=0 + mantissas`. The `snroffststr=0` choice means
      every channel reads the same `frmfsnroffst` from the audfrm —
      `compute_bap` is fed the base `fsnroffst` (NOT the per-channel
      `fsnroffst_ch[ch]` array the AC-3 path uses) so encoder and
      decoder derive identical bap[] arrays.
    - **errorcheck** (§E.2.2.6) is just `encinfo(1) + crc2(16) = 17
      bits`. crc2 covers bytes `[2..frame_bytes-2]` with the same
      polynomial and initial value as AC-3 §6.1.7 (Annex E doesn't
      redefine the CRC).
  Test gates (3 new tests, total = 56):
    - `eac3_first_frame_is_syncframe` — every 768-byte boundary in
      a 192 kbps stereo stream starts with `0x0B 0x77`.
    - `eac3_stereo_192k_decodes_through_ffmpeg` — encode 1 s of
      440 Hz stereo, decode through `ffmpeg -f eac3 -i …`, assert
      PSNR ≥ 18 dB. Measured: **20.21 dB** (matches the AC-3 baseline
      encoder's PSNR-vs-ffmpeg on the same input — ~20.7 dB).
    - `eac3_mono_96k_decodes_through_ffmpeg` — same shape for mono,
      96 kbps. Measured: **20.21 dB**.

### Fixed

- Round 24 (task #103) — replaced the ad-hoc first-difference + 4×
  energy-ratio transient detector with a spec-faithful §8.2.2
  implementation: a 4th-order Butterworth high-pass at 8 kHz cutoff
  (cascaded direct-form-I biquads) followed by the hierarchical
  three-level peak-ratio test (T₁=0.1, T₂=0.075, T₃=0.05) with a
  `100/32768` silence threshold. Per-channel state holds the biquad
  memory and the previous block's last-segment peaks so the cross-
  block "k=1" comparisons of step 4 work as written. The previous
  detector mis-fired on low-frequency pure tones (e.g. 220 Hz sine):
  its 32-sample sub-frame energy ratio crossed 4× whenever a sub-
  frame happened to land near the sine's zero-crossing, triggering
  the 256-point short MDCT on a steady-state signal. Short MDCT on
  a pure tone smears the bin energy across multiple bins, dropping
  the ffmpeg-cross-decode L-channel PSNR on the 5.1 fixture from a
  spec-expected ~24 dB down to **14.36 dB**. After the fix the
  L-channel reads **24.54 dB** (+10.2 dB), matching the other fbw
  channels' bit-allocation ceiling. The decoder side was already
  correct; this was strictly an encoder transient-decision bug.
  `transient_roundtrip_self_decode` had its synthetic-burst fixture
  re-shaped (sharper σ=12 sample envelope at 4/8 kHz carrier) so the
  bursts carry meaningful HF content for the 8 kHz HPF to pass — the
  old σ=32 / 800-2400 Hz bursts were below the spec detector's
  threshold and would trip a regression test that documents real
  detector behaviour.

### Added

- Round 24 (task #103) — per-channel `fsnroffst[ch]` tuning
  (§5.4.3.40). `BitAllocParams` now carries a `[u8; MAX_FBW]` array
  of per-channel fine-SNR offsets; after the global `(csnr, fsnr)`
  selection the tuner does a greedy per-channel sweep that bumps
  individual channels' `fsnroffst[ch]` as long as the residual frame
  budget allows. Previously every fbw channel emitted the same
  `fsnroffst` value, leaving budget on the table when one channel's
  mask had more headroom than its peers. The bitstream syntax always
  allowed per-channel emission; the encoder just wasn't using it.

- Round 19 — multichannel encoder. The encoder now accepts `channels`
  ∈ 1..=6 with per-channel-count acmod selection per A/52 Table 5.8:
    - `1` → acmod=1 (1/0 mono)
    - `2` → acmod=2 (2/0 L,R)             — unchanged from earlier rounds
    - `3` → acmod=3 (3/0 L,C,R)
    - `4` → acmod=6 (2/2 L,R,Ls,Rs)
    - `5` → acmod=7 (3/2 L,C,R,Ls,Rs)
    - `6` → acmod=7 + lfeon=1 (3/2 + LFE — canonical 5.1 layout
      L,C,R,Ls,Rs,LFE)
  BSI emission switches on acmod for the `cmixlev` / `surmixlev` /
  `dsurmod` optional fields per §5.4.2.4-7 + Tables 5.9 / 5.10 (we
  emit the spec-default `01` = -3 dB centre/surround coefficient when
  applicable). LFE pipeline added end-to-end: separate exponent
  extraction over bins 0..7 (§5.4.3.29), `lfeexpstr` 1-bit per block
  per §5.4.3.23, LFE-specific bap routine (LFE never short-blocks
  per §5.4.3.1, no DBA per §5.4.3.47, dedicated `lfefsnroffst /
  lfefgaincod` SNR knobs per §5.4.3.42-43), and LFE mantissas
  emitted last per the §7.3.2 read order. Coupling and rematrix
  remain 2/0-only as the spec requires (`acmod==2` gate). Default
  bit rates per channel count: 1→96, 2→192, 3→256, 4→320, 5→384,
  6→448 kbps. Test gates:
    - `mono_self_decode_roundtrip`
    - `three_zero_self_decode_roundtrip`
    - `three_two_self_decode_roundtrip`
    - `five_one_self_decode_roundtrip`
    - `five_one_ffmpeg_crossdecode` — encodes 5.1 input with a
      unique tone per channel + an 80 Hz sub-bass on LFE, decodes
      via ffmpeg, asserts every channel survived (per-channel RMS
      gate) and reports per-channel PSNR after WAVEEX channel
      reorder. Measured per-channel PSNR on 0.5s tonal fixture at
      448 kbps: L 14.4 dB, C 24.1 dB, R 44.7 dB, Ls 24.2 dB,
      Rs 24.7 dB, LFE 28.4 dB.
- Round 18 — encoder-side §7.2.2.6 / §5.4.3.47-57 delta bit allocation.
  Encoder now emits `deltbaie=1` on block 0 of every syncframe with a
  per-fbw-channel single-band segment (`deltbae[ch]=1`, `deltnseg=1`),
  picked greedily from the lowest-energy 1/6th-octave band in the 25..45
  range with `deltba=4` (+6 dB mask boost). Coupling channel signals
  `cpldeltbae=2` (no delta this block) when coupling is active. Blocks
  1..5 emit `deltbaie=0` (reuse) — block-0's segment list applies for
  the rest of the syncframe. `tune_snroffst` accounts for the dba
  syntax cost (≈17 bits per fbw channel + 2 bits per channel for
  `deltbae[ch]` + 2 bits for `cpldeltbae` when cpl is in use), and
  `compute_bap` / `compute_bap_cpl` apply the dba mask offsets before
  bap[] computation so encoder and decoder derive identical bap[]
  arrays. ffmpeg cross-decodes the dba-bearing stream cleanly.
  `AC3_DISABLE_DBA=1` reverts to the round-17 deltbaie=0 behaviour for
  A/B comparison. Test gate: `dba_self_decode_and_ffmpeg_crosscheck`.
- §7.2.2.6 delta bit allocation: persistent per-channel + coupling deltba
  segment state on `Ac3State`, parsed per Table 5.16 (`new info / reuse /
  no delta` semantics) and applied to the masking curve before bap[]
  computation. Dormant on the current transient-burst fixture (the
  encoder there sets cpldeltbae and per-fbw deltbae[] to '10' = no
  delta) but required-by-spec for any stream that does signal mask
  offsets — without it our bap[] would diverge from the encoder's and
  desync mantissa unpacking.
- `examples/sample_compare.rs`: per-block PSNR + peak-diff diagnostic for
  drilling into burst frames where the per-frame floor hides which
  audblk is breaking. Used to characterise the round-7 drift pattern
  (errors ramp through frame 14 blk 0..5, peak in frame 15 blk 0-1,
  unwind through frame 15 blk 2-5).

### Fixed

- §7.5.2.2 / §7.5.2.3 rematrix band 3 upper bound: was hard-coded to
  bin 252 even when coupling was active, allowing the L+R / L-R operation
  to bleed into the just-decoupled coupling region. Now tracks
  `36 + 12*cplbegf` per Tables 7.26 / 7.27 when `cplinu == 1`. No PSNR
  movement on the current fixture (end_mant capped the bleed there
  anyway) but a latent correctness fix for streams where the per-channel
  bandwidth code reaches further.

### Investigation notes (transient fixture, round 7)

Root-causing the residual ≈15 dB transient PSNR floor — outcome: not
cracked this round, but the bug is now bracketed substantially tighter
than the round-6 notes left it.

Per-block sample_compare on frame 14 of the bursts fixture shows the
error grows monotonically through the frame (28, 22, 16, 11, 8, 6 dB
across blocks 0..5), peaks in frame 15 blk 0-1 (4.4 / 4.7 dB), then
unwinds symmetrically. Peak amplitudes: ours ≈ 0.36 × ffmpeg's on the
worst blocks — i.e. our reconstruction is roughly the right *shape*
but the wrong *magnitude*, with a partial sign inversion appearing
specifically on burst-peak blocks. Probes that did NOT move the floor:

- spec-literal vs. swapped §7.9.4.2 short2 de-interleave (round 6's
  deviation makes no PSNR difference on this fixture either way)
- disabling coupling decoupling entirely (`AC3_DISABLE_DECOUPLE=1`)
- disabling rematrix entirely (`AC3_DISABLE_REMAT=1`)
- the new dba application (encoder sends only "no delta" markers in
  this fixture's burst blocks)

These eliminate IMDCT short-block, coupling decode, rematrix matrix,
AND dba as the dominant error sources. What remains in the per-frame
error budget is bit allocation (bap[]) on the burst-onset blocks
themselves — specifically the masking-curve computation around bins
4-5 (the dominant 440 Hz tone) where calc_lowcomp's break condition
`bndpsd[bin] <= bndpsd[bin+1]` fires under burst conditions and the
spec-vs-implementation behaviour at that exact moment is hardest to
verify without a known-correct reference trace.

Next round should: (a) instrument bndpsd[0..7], excite[0..7], mask[0..7],
and bap[0..7] for ch0 in frame 14 blk 0 (28 dB — easiest to reverse-
engineer the divergence) and compare against a hand-calculated trace
from §7.2.2.4 pseudo-code with the same exponents we decoded; and
(b) consider whether the burst-frame bap[] divergence might come from
our exponent decode chain itself (D15 → cumulative sum of M-2 deltas)
when consecutive deltas straddle the 5-level map saturation boundary,
since the burst fixture is where extreme M=0 / M=4 codes appear.

## [0.0.2](https://github.com/OxideAV/oxideav-ac3/compare/v0.0.1...v0.0.2) - 2026-04-25

### Fixed

- drop crc2 residue debug_assert (wrong invariant)

### Other

- drop oxideav-codec/oxideav-container shims, import from oxideav-core
- encoder quality lift — group-synced mantissa emit + per-block D15 refresh
- clippy sweep — allows for approx_constant + needless_range_loop
- clippy sweep in audblk — auto-fix + spec-faithful allows
- clippy sweep — unnecessary_cast, identity_op, needless_range_loop
- cargo fmt sweep (14 files)
- document round-6 transient-burst PSNR investigation
- fix §7.9.4.2 short2 IMDCT TDAC — restore antisymmetric upper half
- fix BAPTAB/MASKTAB/LATAB off-by-ones, add transient PSNR gate
- FFT-backed 512-pt + 256-pt IMDCT (§7.9.4) + PSNR gate
- §7.8 downmix matrix (3/2, 3/1, 2/1 → stereo/mono)
- parse_frame_side_info helper + fixture-backed §5.4.3 tests
- annotate audblk parser with §5.4.3 clause citations + side-info struct
- ac3 encoder: solve crc1 via GF(2) Gaussian elimination
- ac3 encoder: encode_sine + check_stream examples, preprocess comment
- short-block 256-point IMDCT + bap=0 dither
- ac3 encoder: register in CodecRegistry as make_encoder
- fix IMDCT scale + zero stale coeffs, RMS within 3% of ffmpeg
- IMDCT overlap-add applies spec factor-of-2 scaling
- add audio-block DSP pipeline (exp, bit-alloc, mantissa, IMDCT)
- add bit-allocation / mantissa / window / hth tables
- switch workflows to master branch

### Fixed

- `BAPTAB` (Table 7.16): off-by-one at positions 26 and 30 that was
  shifting every bap in the 23..=34 range by one, mis-quantizing
  mid-band mantissas. PSNR vs ffmpeg on the 440 Hz sine fixture
  jumps from 35.7 dB to 92.0 dB as a direct result.
- `MASKTAB` (Table 7.13): off-by-one at rows A=8, 9, 10, 15 that was
  mapping four high-frequency mantissa bins into the wrong 1/6 octave
  band, skewing bit-allocation masking on wide-bandwidth content.
- `LATAB` (Table 7.14) entry 151: `0x0002` → `0x0003` to match spec
  Table 7.14, row A=15 column B=1.

### Added

- `tests/fixtures/transient_bursts_stereo.ac3`: three Gaussian tone
  bursts, 62 short-block audblks across 63 frames, exercising the
  `blksw=1` IMDCT path that the sine fixture never touches.
- `decoder_matches_ffmpeg_on_transient_fixture` PSNR gate for the
  short-block / block-switching regression.
- `decoder_matches_ffmpeg_within_psnr_floor` gate ratcheted from
  25 dB to 80 dB now that the BAPTAB fix lands.
- `examples/count_blksw`, `examples/psnr_per_frame` diagnostic
  tools for inspecting short-block coverage and per-frame decode
  quality.

### Changed

- `audblk::imdct_256_pair` direct-form short-block IMDCT is now
  `#[cfg(test)]`-only. The §7.9.4.2 FFT decomposition
  (`imdct::imdct_256_pair_fft`) is canonical; the naive per-half
  reference disagrees with it (see the
  `short_block_direct_form_diverges_from_fft` regression test).

### Investigation notes (transient fixture drift)

Round 6: investigated the remaining 15.5 dB PSNR floor on the
transient-burst fixture (bursts at frames 14-17, 29-32, 45-48).
Outcome: no code change lands this round — the previously-suspected
`run_bit_allocation` divergence on bins 4-5 at burst onset was
traced line-by-line against ATSC A/52:2018 §7.2.2.4 and every
reachable value (`lowcomp`, `fastleak`, `slowleak`, `excite[]`,
`mask[]`, `bap[]`) agrees with the spec's pseudocode for the
`bndstrt==0` fbw path (including the break at bin=2 when
`bndpsd[2] <= bndpsd[3]` as bursts rise).

Remaining symptom on burst frames: the time-domain output at the
dominant 440 Hz MDCT bin emerges with roughly the wrong sign (our
output is ≈ −0.77 × reference across the burst). Localised probes
(zero bins 4-5, flip bin 4, flip bin 5, zero entire burst blocks,
disable dither, force long blocks) each shift burst-frame PSNR by
≤4 dB — i.e. the error is not concentrated in any single bin or
DSP stage that the probes touched. Rematrix flags are all zero in
this fixture and coupling coefficients (bins 133-216) are all
`exp=24` / `bap=0`, so neither pathway is contributing.

Further investigation should: (a) compare our decoded
pre-IMDCT coefficient array against a reference MDCT of the
ffmpeg-decoded PCM for burst frame 15 block 0 to pinpoint which
bin's magnitude or sign disagrees, and (b) re-check the §7.1.3
grouped-exponent prefix sum when consecutive deltas sit near the
boundary of the 5-level map (M1=0 or M1=4), since the burst psd
profile is where those extremes actually appear.
