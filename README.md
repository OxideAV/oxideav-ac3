# oxideav-ac3

Pure-Rust **AC-3 (Dolby Digital)** + **E-AC-3 (Enhanced AC-3 / Dolby
Digital Plus)** audio decoder + encoder — elementary streams per
ATSC A/52:2018 (= ETSI TS 102 366). Zero C dependencies.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Status

Early WIP. Implementation follows the A/52 spec incrementally:

- [x] Sync frame + BSI parse (§5.3 / §5.4.1-2). **Round 193**
      surfaces a typed `BitStreamMode` enum + `Bsi::service_type()`
      accessor that decodes Table 5.7 ("Bit Stream Mode") — the
      eight `bsmod` codepoints map to `CompleteMain` /
      `MusicAndEffects` / `VisuallyImpaired` / `HearingImpaired` /
      `Dialogue` / `Commentary` / `Emergency`, and the overloaded
      `bsmod=0b111` resolves via `acmod` to `VoiceOver`
      (`acmod=0b001`) or `Karaoke` (`acmod ∈ {0b010..=0b111}`); the
      undefined `bsmod=0b111 acmod=0b000` cell falls into
      `Reserved`. `is_main()` / `is_associated()` partition the
      table for routing (a receiver picks at most one main service
      and mixes associated services on top), and a stable
      `mnemonic()` returns the Table 5.7 short forms
      (`"CM"/"ME"/"VI"/"HI"/"D"/"C"/"E"/"VO"/"K"/"?"`) for
      CLI / logging. The raw `bsmod` / `acmod` fields stay public
      and authoritative; the new surface is a thin convenience over
      them. 5 new tests cover the fixed-codepoint rows, the
      overloaded-`0b111` branch, the main/associated partition,
      mnemonic stability, and the `Bsi::service_type()` accessor
      round-trip through `parse()`. **Round 202** lifts the BSI's
      heavy-compression gain word from parse-and-discard to a typed
      `Bsi::compr` / `Bsi::compr_ch2` (`Option<CompressionGain>`)
      surface, mirrored on the Annex E `Bsi`. The `CompressionGain`
      newtype splits the 8-bit wire field per Table 7.30 + §7.7.2.2
      into `x() -> i8` (4-bit signed integer, `-8..=+7`, contributing
      `(X+1)·6.02 dB`) and `y() -> u8` (4-bit unsigned mantissa with
      implicit leading 1, contributing the `(16+Y)/32` attenuation
      between -6.02 dB and -0.28 dB), with `linear() -> f32` /
      `decibels() -> f32` derivatives. Endpoint codepoints land at
      the spec's documented combined range: `raw=0b0111_1111` (X=7,
      Y=15) ≈ +47.89 dB, `raw=0b1000_0000` (X=-8, Y=0) ≈
      -48.16 dB. Annex E reuses Table 7.30 verbatim per §E.2.3.1.x,
      so the eac3 BSI surfaces the same type — single source of
      truth for both parsers. `None` is preserved verbatim when the
      encoder omitted the word, letting a peak-limited player honour
      the §7.7.2.1 "use dynrng for that syncframe" fallback. The
      decoder PCM path is unchanged — compr/dynrng remain at the
      decoder's "discretion" per spec — so the typed surface is
      pure metadata for downstream RF-modulator / hotel-room feeds
      that need to bound peak output level without re-parsing the
      BSI. Encoders still emit `compre=0` (no heavy-compression
      policy yet). Covered by 6 new `bsi::tests` (every X
      codepoint's two's-complement sign-extension, every Table 7.30
      row's dB endpoints at both `Y=0` and `Y=15`, the Y fractional
      decode with implicit leading 1, the §7.7.2.2 combined ±48 dB
      range endpoints, `parse()` round-trip via `compre=1`, and the
      1+1 `compr2e` Ch2 round-trip) plus 1 new `eac3::bsi::tests`
      round-trip. **Round 208** lifts the Annex D xbsi2 informational
      block + the §E.2.3.1.x informational metadata from
      parse-and-discard to three typed fields:
      `Bsi::dsurexmod : Option<DolbySurroundExMode>` (Table D2.7 —
      `NotIndicated` / `NotEncoded` / `SurroundExOrProLogicIIx` /
      `ProLogicIIz`), `Bsi::dheadphonmod : Option<DolbyHeadphoneMode>`
      (Table D2.8 — `NotIndicated` / `NotEncoded` / `Encoded` /
      `Reserved`), and `Bsi::adconvtyp : Option<AdConverterType>`
      (Table D2.9 — `Standard` / `Hdcd`). Surfaces mirror on the
      E-AC-3 `Bsi` (the spec's §E.2.3.1.x informational block reuses
      D2.7-D2.9 verbatim) with the spec's per-acmod gates — `dsurexmod`
      only when `acmod ≥ 6`, `dheadphonmod` only when `acmod == 2`,
      `adconvtyp` inside the `audprodie` chain — plus a separate
      `Eac3Bsi::adconvtyp_ch2` for the 1+1 dual-mono Ch2 `audprodi2e`
      word. The three fields are per spec purely informational hints
      for downstream playback equipment (surround upmix processor,
      headphone virtualiser, HDCD-aware DAC) and do not affect AC-3 /
      E-AC-3 PCM decode — but surfacing them lets a chain consumer
      route the hint without re-parsing the BSI. The encoders still
      emit `xbsi2e=0` / `infomdate=0` for every syncframe so encoder
      output is byte-identical; the only behaviour change is
      decoder-side parsing. Covered by 4 new `bsi::tests`
      (enum-codepoint round-trips for all three tables, `xbsi2e==1`
      surfacing all three on a 3/2 frame, `None`-stays-`None`
      assertions for `bsid != 6` and `xbsi2e == 0`) plus 4 new
      `eac3::bsi::tests` (`infomdate == 0` yields no hints; 3/2 indep
      `infomdate == 1` surfaces `dsurexmod` + `adconvtyp`; 2/0 indep
      surfaces `dheadphonmod`; 1+1 dual-mono surfaces both `adconvtyp`
      and `adconvtyp_ch2` independently). **Round 214** extends the
      same parse-and-surface treatment to the §5.4.2.13-15 audio
      production information block — `mixlevel` (5 bits) + `roomtyp`
      (2 bits, Table 5.12) lift from parse-and-discard to a typed
      `Bsi::audio_production: Option<AudioProductionInfo>` (plus an
      independent `Bsi::audio_production_ch2` mirror for the 1+1
      dual-mono `audprodi2e` chain per §5.4.2.21-23). The
      `AudioProductionInfo` struct exposes the raw 5-bit `mixlevel`
      codepoint, a typed `RoomType` enum (Table 5.12: `NotIndicated`
      / `LargeXCurve` / `SmallFlat` / `Reserved`), and a
      `peak_mix_level_db_spl()` accessor resolving the spec's
      `80 + mixlevel` derivation (range 80..=111 dB SPL per §5.4.2.14).
      The Annex E `Bsi` carries the same surface — §E.2.3.1.x's
      informational-metadata block reuses §5.4.2.13-15 verbatim under
      the `infomdate == 1` gate — so single source of truth for both
      parsers. Per spec the fields are advisory ("not typically used
      within the AC-3 decoder, but may be used by other parts of the
      audio reproduction equipment") so the decoder PCM path is
      unchanged; the typed surface lets cinema / mastering tooling
      re-target the playback bus to the absolute mixing-session SPL
      without re-walking the BSI. Encoders still emit `audprodie=0`
      so encoder output is byte-identical. Covered by 5 new
      `bsi::tests` (every Table 5.12 row's round-trip; the
      `80 + mixlevel` endpoint resolution at 0 / 5 / 31 codepoints;
      `audprodie == 1` mono surface; `audprodie == 0` short-circuit;
      and the 1+1 dual-mono Ch1+Ch2 independent surfacing) plus 1
      new `eac3::bsi::tests::no_infomdate_yields_no_audio_production`
      and extended assertions on two pre-existing `infomdate` tests
      that already exercised `audprodie == 1` (3/2 indep + 1+1).
      **Round 219** lifts the base-syntax timecode fields from parse-
      and-discard to a typed surface: `Bsi::timecod1: Option<TimeCode1>`
      + `Bsi::timecod2: Option<TimeCode2>` +
      `Bsi::timecode_presence: TimeCodePresence` decode the
      §5.4.2.26-28 / Table 5.13 timecode pair when the base syntax is
      in use (`bsid != 6` — Annex D §1 reuses these wire bits for the
      `xbsi*e` blocks so the timecode is definitionally absent on
      `bsid == 6` streams). `TimeCode1` exposes the 5-bit `hours()`,
      6-bit `minutes()`, and 3-bit `eight_second_increments()` halves
      of the low-resolution 14-bit field (range 0..=86 336 s within
      the 24-hour day at 8-second granularity per §5.4.2.27);
      `TimeCode2` exposes the 3-bit `seconds()`, 5-bit `frames()`, and
      6-bit `frame_fractions()` halves of the high-resolution 14-bit
      field (≈ 521 µs resolution at the §5.4.2.26 30 fps reference).
      Both types carry a `is_spec_valid()` predicate that flags
      out-of-range codepoints (hours > 23, minutes > 59, frames > 29)
      without rejecting the stream — per Annex D §3.2 the timecode
      "does not affect the decoding process in legacy decoders" so the
      parser passes the raw codepoint through verbatim. The
      `TimeCodePresence` enum (`NotPresent` / `FirstHalfOnly` /
      `SecondHalfOnly` / `BothHalves`) records the
      `(timecod2e, timecod1e)` pair per Table 5.13 so a chain consumer
      can pick playback strategy without re-decoding the flags.
      Encoders still emit `timecod1e == 0` / `timecod2e == 0` for every
      syncframe so encoder output is byte-identical; the only behaviour
      change is decoder-side parsing. Covered by 10 new `bsi::tests`
      (`TimeCode1` and `TimeCode2` field decomposition including
      out-of-range pass-through, `is_spec_valid` range checks for both
      halves, the Table 5.13 `from_flags` round-trip, `parse()`
      surfacing both halves on base-syntax frames, the `FirstHalfOnly`
      and `SecondHalfOnly` partial-presence rows, the `NotPresent`
      all-zero case, and the Annex D `bsid == 6` short-circuit that
      keeps `timecod1` / `timecod2` at `None` even when the wire bits
      carrying `xbsi*e` are set). **Round 226** lifts the §5.4.2.24-25
      distribution-control hint pair (`copyrightb` + `origbs`) from
      parse-and-discard to a typed `Bsi::copyright_info: CopyrightInfo`
      surface, mirrored on the Annex E `Bsi` as
      `Option<CopyrightInfo>` (gated by `infomdate == 1` per
      §E.2.3.1.62). The `CopyrightInfo` struct exposes
      `is_copyright_protected()` (§5.4.2.24 — "the information in the
      bit stream is indicated as protected by copyright") and
      `is_original_bitstream()` (§5.4.2.25 — "this is an original bit
      stream", `false` ⇒ this is a copy) accessors plus raw 1-bit
      `copyrightb_bit()` / `origbs_bit()` getters for byte-exact
      re-emission. The base-syntax pair is always present per §5.3.2 so
      the field is unconditional `CopyrightInfo`; the Annex E pair sits
      inside the §E.2.3.1.62 informational metadata block so the
      surface is `Option<CopyrightInfo>` — `None` reflects the
      encoder-default `infomdate == 0` path. Per spec the bits are
      purely advisory ("does not affect the decoding process") so the
      PCM path is unchanged; surfacing them lets a chain consumer
      enforce a distribution / archival policy (refuse to re-encode a
      `copyrightb == 1` stream, tag a `origbs == 0` copy for
      downstream-only routing) without re-walking the BSI. The base
      encoder still emits `copyrightb=0, origbs=1` and the Annex E
      encoder still emits `infomdate=0` so encoder output is
      byte-identical; the only behaviour change is decoder-side
      parsing. Covered by 6 new `bsi::tests` (the four-codepoint
      round-trip, `Eq` + `Copy` semantics, the encoder-default
      `(0,1)` BSI parse, the `(1,0)` protected-copy pattern, the 1+1
      dual-mono `acmod == 0` BSI where the pair sits further down the
      cursor past the Ch2 metadata chain, and the Annex D `bsid == 6`
      shared-position parse with `(0,0)`) plus 3 new
      `eac3::bsi::tests` (`infomdate == 0` short-circuit; 3/2 indep
      with `(1,1)`; 2/0 indep with `(0,0)` exercising the
      `dheadphonmod` gate path). **Round 234** lifts the §5.4.2.8
      dialogue-normalization word from a remapped `u8` to a typed
      `Bsi::dialogue_normalization() -> DialNorm` accessor (mirrored
      on Annex E) and adds a parallel `Bsi::dialnorm_ch2: Option<u8>`
      surface for the §5.4.2.16 Ch2 mirror in 1+1 dual-mono
      (`acmod == 0`) streams plus
      `Bsi::dialogue_normalization_ch2() -> Option<DialNorm>`. The
      `DialNorm` newtype exposes `codepoint()` / `wire_value()` /
      `is_reserved_wire_codepoint()` (so the §5.4.2.8 reserved-`0`
      remap is observable), `db() -> i8` (`-31..=-1`),
      `level_below_full_scale_db() -> u8` (`1..=31` — the §7.6
      "headroom in dB above the subjective dialogue level"
      phrasing), `attenuation_linear()`, and
      `reproduction_gain_linear(listener_target_db,
      reference_full_scale_db)` — the §7.6 playback-gain derivation
      `listener_target_db − reference_full_scale_db +
      level_below_full_scale_db` that lets a reproduction system
      apply the dialnorm normalization without re-parsing the BSI.
      Validated against the §7.6 worked example verbatim (listener
      67 dB SPL, reference 105 dB SPL, dialnorm -25 dB → -13 dB
      system gain → 92 dB SPL full-scale reproduction). Per §7.6
      the value is not consumed inside the AC-3 decoder itself —
      it is forwarded to the reproduction system's volume
      controller — so the PCM path is unchanged. Encoders still
      emit `dialnorm == 27` (-27 dB) so encoder output is
      byte-identical; the only behaviour change is decoder-side
      parsing. Covered by 9 new `bsi::tests` (every legal
      `1..=31` wire codepoint round-trip, the reserved-`0` remap
      observable via `is_reserved_wire_codepoint`, the
      low-5-bit-only masking, linear attenuation at -1 / -25 / -31
      dB, the §7.6 reproduction-gain worked example, the typed
      accessor via `parse()`, 1+1 dual-mono `dialnorm_ch2`
      surfacing, the Ch2 reserved-codepoint remap, and the
      `acmod != 0` short-circuit) plus 4 new `eac3::bsi::tests`
      (stereo indep surface, 1+1 Ch2 surface, Annex E reserved
      codepoint remap, and the non-1+1 short-circuit). 218 lib
      tests, all green. **Round 240** lifts the §5.4.2.29-31
      `addbsi` trailer (the variable-length additional bit-stream
      information block that closes both `bit_stream_info()` in the
      base syntax and Table E1.2's BSI walk on Annex E) from
      parse-and-discard to a typed `Bsi::addbsi:
      Option<AdditionalBitStreamInfo>` surface, mirrored across
      base AC-3 and Annex E (`eac3::Bsi::addbsi` reuses the same
      type — Table E1.2 carries `addbsie + addbsil + addbsi`
      verbatim from §5.3.2). The `AdditionalBitStreamInfo` struct
      exposes `addbsil() -> u8` (raw 6-bit codepoint, 0..=63),
      `len() -> usize` (`addbsil + 1`, always within 1..=64 per
      §5.4.2.30), `payload() -> &[u8]` (the wire-order bytes the
      encoder placed in the trailer), and `wire_bits() -> u32`
      (`7 + 8 × (addbsil + 1)` — total span of the trailer block
      including the `addbsie` flag, for callers that need to
      mirror the BSI verbatim back into a bit-stream writer). The
      `from_addbsil_and_payload` constructor enforces the
      §5.4.2.30 length-byte relationship (rejects out-of-range
      `addbsil` and payload-length mismatches) so callers cannot
      construct an instance that would not round-trip. Per
      §5.4.2.30 the decoder "is not required to interpret this
      information, and thus shall skip over this number of bytes"
      so the PCM path is unchanged; surfacing the payload bytes
      lets a chain consumer reach an encoder-private metadata
      block (encoder watermark, distribution-tagging, OAMD
      packetisation, downstream routing hint) without re-walking
      the BSI. Encoders still emit `addbsie == 0` for every
      syncframe so encoder output is byte-identical; the only
      behaviour change is decoder-side parsing. Covered by 7 new
      `bsi::tests` (constructor-validity rejection cases, minimum-
      length 1-byte payload, maximum-length 64-byte payload,
      `parse()` round-trip on a 1-byte payload with cursor check,
      `parse()` round-trip on the 64-byte endpoint with
      `bits_consumed` cursor check, Annex D `bsid == 6` round-trip
      confirming the trailer position is unaffected by the
      alt-syntax switch, and a 1+1 dual-mono `acmod == 0`
      round-trip past the Ch2 service-metadata block) plus 4 new
      `eac3::bsi::tests` (encoder-default `addbsie == 0`
      short-circuit on E-AC-3, 1-byte payload, 64-byte endpoint
      payload, dependent-substream walk with `strmtyp == 1`).
      230 lib tests, all green. **Round 243** lifts the §2.3.1.2 /
      Table D2.2 preferred-stereo-downmix advisory (`dmixmod`, 2
      bits) from a raw-sentinel codepoint to a typed
      `Bsi::dmixmod_preference: Option<StereoDownmixPreference>`
      surface, mirrored on the Annex E `Bsi`
      (`mixmdate == 1` mixing-metadata block reuses Annex D
      §2.3.1.2 verbatim per Table E1.2 §E.1.2.2). The new
      `StereoDownmixPreference` enum covers the four wire codepoints
      — `NotIndicated` (`'00'`), `LtRtPreferred` (`'01'`),
      `LoRoPreferred` (`'10'`), `Reserved` (`'11'`) — with `raw() ->
      u8` for bit-stream round-trip, `prefers_lt_rt() / prefers_lo_ro()`
      short-circuit predicates for a §3.1.1 auto-mode
      two-channel-downmix router, and `is_not_indicated()` collapsing
      both `NotIndicated` and `Reserved` into one branch per the
      §2.3.1.2 spec note ("the reserved code may be interpreted as
      'not indicated'"). `Some` only when the wire slot is actually
      present: on the base parser when `bsid == 6` AND `xbsi1e == 1`,
      on the Annex E parser when `mixmdate == 1` AND `acmod > 2`;
      `None` otherwise (the §5.3.2 base timecode syntax reuses the
      bit slot for `timecod*e/timecod*`, and the Table E1.2 mixmdata
      guard skips the slot for mono / 2/0 streams). The raw
      `dmixmod: u8` field
      with the `0xFF` "absent" sentinel stays public on both BSI
      structs as the authoritative wire value, so the typed surface
      is a thin convenience for chain consumers and existing
      consumers continue to compile. Single source of truth across
      base + Annex E so a chain consumer can route both syntaxes
      through one branch on
      `Bsi::stereo_downmix_preference()`. The decoder PCM path is
      unchanged — `dmixmod` is per §2.3.1.2 "may be used by the AC-3
      decoder to automatically configure the type of stereo
      downmix, but may also be overridden or ignored" — surfacing
      the hint lets a §3.1.1 compliant downmix router pick LtRt vs
      LoRo without re-parsing the BSI. Encoders still emit
      `xbsi1e == 0` / `mixmdate == 0` for every syncframe so encoder
      output is byte-identical; the only behaviour change is
      decoder-side parsing. Covered by 5 new `bsi::tests`
      (every-codepoint round-trip on `from_code`, the
      `is_not_indicated()` Reserved + NotIndicated collapse, the
      `prefers_lt_rt()` / `prefers_lo_ro()` predicate gating, the
      §5.3.2 base-syntax `None` short-circuit through `parse()`, and
      the Annex D `bsid == 6` + `xbsi1e == 1` round-trip across all
      four codepoints) plus 3 new `eac3::bsi::tests` (Annex E
      `mixmdate == 1` round-trip across all four codepoints, the
      `acmod == 2` per-Table-E1.2 guard short-circuit, and the
      `mixmdate == 0` baseline). 239 lib tests, all green.
      **Round 246** brings the §5.4.2.6 base-syntax Dolby Surround
      mode (`dsurmod`, 2 bits, Table 5.11) up to the same typed
      treatment via a new
      `Bsi::dolby_surround_mode: Option<DolbySurroundMode>` field,
      mirrored on the Annex E `Bsi` (the §E.2.3.1.x
      informational-metadata `acmod == 2` branch had been parsing
      the slot and discarding it — surfacing it brings parity with
      the base-syntax view). `DolbySurroundMode` covers the four
      wire codepoints — `NotIndicated` (`'00'`), `NotEncoded`
      (`'01'`), `Encoded` (`'10'`), `Reserved` (`'11'`) — with
      `raw() -> u8` for bit-stream round-trip,
      `is_dolby_surround_encoded()` short-circuit predicate for a
      Pro Logic-aware receiver to arm its matrix decoder, and
      `is_not_indicated()` collapsing both `NotIndicated` and
      `Reserved` into one branch per the §5.4.2.6 spec note ("the
      reserved code may be interpreted as 'not indicated'"). `Some`
      only when the wire slot is actually present: on the base
      parser when `acmod == 2`, on the Annex E parser when
      `infomdate == 1` AND `acmod == 2`; `None` otherwise. The raw
      `dsurmod: u8` field with the `0xFF` "absent" sentinel stays
      public on the base BSI struct as the authoritative wire value
      so existing consumers continue to compile. Single source of
      truth across base + Annex E — the Annex E
      informational-metadata `dsurmod` slot reuses Table 5.11
      semantics verbatim, so a chain consumer can route both
      syntaxes through one branch on `Bsi::dolby_surround_mode()`.
      The Annex E → base-AC-3 shim in
      `eac3::dsp::build_ac3_bsi_shim` forwards the typed field
      through unchanged so the base AC-3 downmix helpers can consult
      it on an Annex E playback path too. The decoder PCM path is
      unchanged — per §5.4.2.6 the field "is not used by the AC-3
      decoder, but may be used by other portions of the audio
      reproduction equipment" — surfacing the hint lets a Pro
      Logic-aware receiver arm its matrix decoder without re-parsing
      the BSI. Encoders still emit `dsurmod == 0` (NotIndicated) for
      every 2/0 syncframe so encoder output is byte-identical; the
      only behaviour change is decoder-side parsing. Covered by 4
      new `bsi::tests` (every-codepoint round-trip on `from_code`,
      the `is_not_indicated()` Reserved + NotIndicated collapse, the
      `is_dolby_surround_encoded()` predicate gating, the §5.3.2
      base-syntax `parse()` round-trip across all four codepoints on
      `acmod == 2`, plus a `None` short-circuit on `acmod != 2`)
      plus 3 new `eac3::bsi::tests` (Annex E `infomdate == 1`
      round-trip across all four codepoints on `acmod == 2`, the
      per-Table-E1.2 `acmod != 2` guard short-circuit on a 3/2
      frame, and the `infomdate == 0` baseline). 246 lib tests, all
      green. **Round 249** lifts the §5.4.2.11-12 deprecated
      `langcod` slot (and its §5.4.2.19-20 Ch2 `langcod2` mirror) from
      parse-and-discard to two new typed `Bsi::language_code:
      Option<LanguageCode>` / `Bsi::language_code_ch2:
      Option<LanguageCode>` fields. Per §5.4.2.12 the slot "is an 8
      bit reserved value that shall be set to `0xFF` if present" —
      the original 1995 table-lookup language id was retired in 2001,
      and modern delivery systems carry the ISO 639-2 language code
      in the signaling layer instead. The `LanguageCode` newtype wraps
      the raw byte verbatim and exposes `raw() -> u8` plus an
      `is_spec_reserved_value() -> bool` predicate that flags whether
      the carried byte matches the `0xFF` wire-conformance value, so a
      probe / archive tool can route legacy non-conforming streams
      (carrying a 1995-era table-lookup codepoint) to a chain-of-
      custody log without re-parsing the BSI. `Some` only when the
      flag bit is set: on Ch1 when `langcode == 1`, on Ch2 when
      `acmod == 0` AND `langcod2e == 1`; `None` otherwise. The slot
      is base-AC-3 only — the Annex E (E-AC-3) BSI never carried a
      `langcod` field — so the Annex E → base-AC-3 shim in
      `eac3::dsp::build_ac3_bsi_shim` hands the base helpers `None`
      unconditionally and the typed surface stays on the base BSI
      struct. The decoder PCM path is unchanged — the word does not
      affect audio reproduction — and encoders still emit
      `langcode == 0` for every syncframe so encoder output is
      byte-identical; the only behaviour change is decoder-side
      parsing. Covered by 7 new `bsi::tests` (every-byte round-trip
      on `from_raw`, the `is_spec_reserved_value` predicate's
      `0xFF`-only acceptance, base-syntax `parse()` round-trip with a
      spec-conforming `0xFF` byte, a non-conforming legacy `0x42`
      byte, the `langcode == 0` short-circuit, the 1+1 dual-mono Ch2
      mirror with `langcod2e == 1`, and the per-channel-gate
      independence on `langcod2e == 0`). 253 lib tests, all green.
      **Round 254** lifts the §2.3.1.11-12 reserved-trailer slot
      (8-bit `xbsi2` + 1-bit `encinfo`) that closes the Annex D
      `xbsi2e == 1` block from parse-and-discard to a typed
      `Bsi::extra_bsi: Option<ExtraBsi2>` field. Per §2.3.1.11
      encoders shall set `xbsi2` to `0x00` (reserved for future
      assignment); per §2.3.1.12 `encinfo` is reserved for
      encoder-private use ("not used by the decoder"). The
      `ExtraBsi2` newtype wraps both fields verbatim and exposes
      `from_raw(u8, bool)` / `xbsi2() -> u8` / `encinfo() -> bool` /
      `wire_bits() -> u32` (`9` — 8 bits for `xbsi2` plus 1 bit for
      `encinfo`) plus an `is_spec_reserved_value() -> bool`
      predicate that flags whether the carried byte matches the
      `0x00` wire-conformance value, so a conformance probe /
      archive tool can route non-conformant encoder output
      (`xbsi2 != 0x00`) to a chain-of-custody log without re-parsing
      the BSI. `encinfo` is excluded from the conformance check —
      any value is wire-legal per §2.3.1.12. `Some` only when
      `bsid == 6` AND the encoder set `xbsi2e == 1`; `None`
      otherwise (the §5.3.2 base syntax reuses the bit slot for
      `timecod2e/timecod2` and the trailer is definitionally absent
      on `bsid != 6` streams). The block is base-AC-3 only — the
      Annex E (E-AC-3) BSI never carries an `xbsi2e` slot — so the
      Annex E → base-AC-3 shim in `eac3::dsp::build_ac3_bsi_shim`
      hands the base helpers `None` unconditionally and the typed
      surface stays on the base BSI struct. The decoder PCM path is
      unchanged — per §2.3.1.11-12 the fields "are not used by the
      decoder" — and encoders still emit `xbsi1e == 0` /
      `xbsi2e == 0` for every syncframe so encoder output is
      byte-identical; the only behaviour change is decoder-side
      parsing. Covered by 5 new `bsi::tests` (every-byte ×
      every-flag `from_raw` round-trip plus `Copy`/`Eq` semantics,
      the `is_spec_reserved_value` predicate's `xbsi2 == 0x00`-only
      acceptance across all 511 non-conformant codepoints, Annex D
      `parse()` surfacing on a non-conformant `xbsi2 == 0xAA`
      codepoint with `encinfo == 1`, Annex D `parse()` surfacing on
      the spec-conformant `xbsi2 == 0x00` codepoint with
      `encinfo == 0` cross-checked against the sibling `dheadphonmod`
      / `adconvtyp` typed fields on a 2/0 frame, and the
      `xbsi2e == 0` short-circuit on a `bsid == 6` frame).
      258 lib tests, all green. **Round 259** lifts the §5.4.2.4
      `cmixlev` (Table 5.9) + §5.4.2.5 `surmixlev` (Table 5.10) 2-bit
      mix-level codewords from parse-into-`u8` to typed
      `Bsi::center_mix: Option<CenterMixLevel>` /
      `Bsi::surround_mix: Option<SurroundMixLevel>` surfaces. The
      `CenterMixLevel` enum carries the four Table 5.9 codepoints
      verbatim (`Minus3Db` = 0.707, `Minus4Point5Db` = 0.595,
      `Minus6Db` = 0.500, `Reserved`); the `SurroundMixLevel` enum
      carries the four Table 5.10 codepoints (`Minus3Db` = 0.707,
      `Minus6Db` = 0.500, `Mute` = 0.000, `Reserved`). Both expose
      `from_code(u8)` / `raw() -> u8` round-trip plus
      `coefficient() -> Option<f32>` (returns the spec-documented
      linear attenuation; `None` for the reserved codepoint) and
      `coefficient_with_reserved_fallback() -> f32` (applies the
      §5.4.2.4-5 "intermediate value may be used" substitution so a
      §7.8 downmix consumer can pick the per-codepoint gain in a
      single call). `SurroundMixLevel::is_mute()` lets a downmix
      router short-circuit the surround mix-in step when the encoder
      picked the `'10'` mute codepoint. `Some` only when the
      §5.3.2 guards emit the codeword on the wire — `center_mix`
      requires 3 front channels (`acmod ∈ {3, 5, 7}`), `surround_mix`
      requires a surround channel (`acmod ∈ {4, 5, 6, 7}`); both
      stay `None` for every other channel mode, mirroring the raw
      `0xFF` "absent" sentinel that the existing `Bsi::cmixlev` /
      `Bsi::surmixlev` fields keep for bit-stream round-trip. The
      Annex E (E-AC-3) BSI never carries these 2-bit slots — Annex E
      replaces them with the refined 3-bit `ltrtcmixlev` /
      `lorocmixlev` / `ltrtsurmixlev` / `lorosurmixlev` codewords
      inside the `mixmdata` block (§E.2.3.1.3-6) — so the typed
      surface stays on the base AC-3 `Bsi`; the
      `eac3::dsp::build_ac3_bsi_shim` hands the base helpers `None`
      unconditionally. The decoder PCM path is unchanged — the
      existing `downmix::Downmix::from_bsi` consumer continues to
      read `bsi.cmixlev` / `bsi.surmixlev` and index the
      `CENTER_MIX_LEVEL` / `SURROUND_MIX_LEVEL` tables — and
      encoders still emit `cmixlev == 0b01` / `surmixlev == 0b01`
      for every syncframe so encoder output is byte-identical. The
      only behaviour change is decoder-side parsing: the new typed
      surfaces let chain consumers (a downstream LtRt / LoRo
      auto-router, a metadata probe) pick the per-codepoint
      coefficient without re-walking Table 5.9 / 5.10 or consulting
      the magic `0xFF` sentinel. Covered by 6 new `bsi::tests`
      (every Table 5.9 codepoint × `from_code` / `raw` / `coefficient`
      / fallback / `is_reserved` round-trip including the 2-bit
      truncation of `0xFF`; the same coverage for Table 5.10 plus
      `is_mute`; `parse()` surfacing on a 3/2 frame with non-default
      codepoints; the 2/0 stereo case keeping both surfaces `None`;
      the 1/0 mono case keeping both surfaces `None`; and the 2/2
      asymmetric case where only `surround_mix` surfaces).
      264 lib tests, all green. **Round 263** lifts the
      §5.4.1.3 `fscod` sample-rate code (Table 5.6) from raw `u8` to
      a typed `SyncInfo::sample_rate_code() -> SampleRateCode`
      accessor. `SampleRateCode` is a four-variant enum carrying the
      three valid sampling-rate codepoints (`FortyEightKHz` /
      `FortyFourPointOneKHz` / `ThirtyTwoKHz`) and a `Reserved`
      variant for the spec-reserved `'11'` codepoint that mandates a
      decoder mute. The enum exposes `from_code(u8)` / `raw() -> u8`
      verbatim round-trip (with the upper bits of `code` ignored so a
      caller does not need to mask first), `hertz() -> Option<u32>` /
      `kilohertz() -> Option<u32>` for the spec rate lookups, an
      `is_reserved()` predicate for probe / re-emit tooling, and an
      `hth_row_index() -> Option<usize>` accessor that routes a typed
      sample-rate code straight into the §7.15 hearing-threshold table
      row in [`tables::HTH`] without re-walking Table 5.6. The raw
      `SyncInfo::fscod: u8` and pre-resolved `SyncInfo::sample_rate:
      u32` fields stay public and authoritative; the new typed
      surface is a thin convenience over them. `parse()` itself still
      rejects the reserved `'11'` codepoint at frame boundary per
      §5.4.1.3 ("If the reserved code is indicated, the decoder
      should not attempt to decode audio and should mute") so a
      `SyncInfo` obtained from `parse()` never reports `Reserved` —
      the variant is preserved for chain consumers that construct a
      `SyncInfo` by hand (e.g. resynthesising one from
      container-stored metadata where the upstream demuxer may not
      have validated `fscod`). The Annex E (E-AC-3) BSI overloads the
      `'11'` codepoint as a reduced-rate indicator that triggers a
      follow-on `fscod2` codeword (§E.2.3.1.4-5), so this enum's
      `Reserved` variant corresponds to the base AC-3 decoder-mute
      semantics only; the typed surface is not mirrored on the Annex
      E `Bsi`. The decoder PCM path is unchanged and encoder output
      is byte-identical; the only behaviour change is the added
      accessor. Covered by 7 new `syncinfo::tests` (all four Table
      5.6 codepoints' `from_code` / `raw` round-trip; the upper-bit
      truncation of `from_code` arguments; `hertz()` + `kilohertz()`
      Table 5.6 lookups for the three valid codepoints and `None` for
      reserved; `is_reserved` only-true-for-`'11'`; `hth_row_index`
      matching the Table 7.15 row order; the `parse()` → typed
      surface agreement on 48 / 44.1 / 32 kHz frames; and the
      hand-built reserved-fscod surfacing path). 271 lib tests, all
      green. **Round 271** lifts the §5.4.1.4 `frmsizecod` frame-size
      code (Table 5.18) — the other half of syncinfo byte 4 — from raw
      `u8` to a typed `SyncInfo::frame_size_code() -> FrameSizeCode`
      accessor, mirroring the r263 `SampleRateCode` surface.
      `FrameSizeCode` is a two-variant enum: `Valid(u8)` carries one of
      the 38 Table 5.18 codepoints (`frmsizecod = 0..=37`) and
      `Reserved` collapses the `38..=63` range with no Table 5.18 row.
      Per §5.4.1.4 the code "is used along with the sample rate code to
      determine the number of (2-byte) words before the next syncword."
      The enum exposes `from_code(u8)` (masks the upper 2 bits so a
      caller can pass syncinfo byte 4 verbatim — `fscod` lives in
      bits 7..6), `raw() -> Option<u8>` (`None` for the reserved range),
      `is_reserved()`, `nominal_bitrate_kbps() -> Option<u32>` (the two
      neighbouring codepoints per rate return the same value),
      `words(SampleRateCode) -> Option<u32>` for the per-rate 16-bit-word
      count, and `frame_length_bytes(SampleRateCode) -> Option<u32>`
      (`2 ×` words) which matches the pre-resolved `SyncInfo::frame_length`
      field for any frame `parse()` accepts. The raw
      `SyncInfo::frmsizecod` and pre-resolved `SyncInfo::frame_length`
      fields stay public and authoritative; the typed surface is a thin
      convenience over them. `parse()` still rejects an out-of-range
      `frmsizecod` at frame boundary so a `SyncInfo` from `parse()` never
      reports `Reserved` — the variant is preserved for chain consumers
      that hand-build a `SyncInfo` from container-stored metadata. The
      decoder PCM path is unchanged and encoder output is byte-identical;
      the only behaviour change is the added accessor. Covered by 6 new
      `syncinfo::tests` (the full `0..=37` valid + `38..=63` reserved
      round-trip, the upper-2-bit masking, the `nominal_bitrate_kbps`
      pairing at the 32 / 192 / 640 kbps rows, the per-rate `words` /
      `frame_length_bytes` Table 5.18 lookups plus reserved-rate /
      reserved-code `None` short-circuits, the `parse()` → typed surface
      agreement on 48 / 32 kHz frames, and the hand-built
      reserved-frmsizecod surfacing path). 277 lib tests, all green.
      **Round 278** lifts the Annex E §E.2.3.1.12-17 program-scale-factor
      trio — `pgmscl` (the substream's own program), `pgmscl2` (the
      second channel of a 1+1 dual-mono program), and `extpgmscl` (an
      *external* program carried in a different bit stream / independent
      substream, "same scale as pgmscl" per §E.2.3.1.17) — from
      parse-and-discard to typed `Eac3Bsi::pgmscl` / `pgmscl2` /
      `extpgmscl` fields (`Option<ProgramScaleFactor>`). The newtype
      implements the §E.2.3.1.13 wire scale: codepoint `0` = mute,
      `1..=63` → `-50..=+12 dB` in 1 dB steps (`decibels() = code − 51`,
      unity at `51`), with `from_code` / `raw()` round-trip, `is_mute()`,
      `decibels() -> Option<i8>`, and `linear() -> f32` for a §E.3.10
      dual-decoder mixer to consume directly. `Some` only when
      `mixmdate == 1` on an independent substream (Table E1.2 emits the
      chain under `strmtyp == 0x0` only) with the respective exists-flag
      set; `None` means "0 dB (no scaling)" per §E.2.3.1.12/.14/.16. Per
      §E.3.10.1-2 the gains apply during the main + associated-service
      mixing process, so the single-stream decode PCM path is unchanged
      and encoder output stays byte-identical (`mixmdate == 0`). Covered
      by 8 new `eac3::bsi::tests` including both §E.3.10 worked-example
      gains (-3 dB / -10 dB) round-tripped through `parse()` with cursor
      checks. 285 → 293 lib tests, all green.
      **Round 281** lifts the Annex E §E.2.3.1.53-58 pan-information
      pair — `panmean`+`paninfo` (the substream's mono program) and
      `panmean2`+`paninfo2` (the second channel of a 1+1 dual-mono
      program) — from parse-and-discard to typed `Eac3Bsi::paninfo` /
      `paninfo2` fields (`Option<PanInfo>`). The `PanInfo` struct
      implements the §E.2.3.1.54 wire scale: index `0` points the
      panned virtual source at the center speaker (0°), each step is
      1.5° clockwise (`degrees()`, `0..=239` → `0..=358.5°`),
      `240..=255` reserved (`is_reserved_index()`), with the 6-bit
      reserved trailer preserved verbatim for re-emit. The §E.3.10.8
      mixer tables are implemented directly: `stereo_scale_factors()
      -> Option<(AL, AR)>` (Table E3.15) and `surround_scale_factors()
      -> Option<[AL, AC, AR, ALS, ARS]>` (Tables E3.16-E3.17, LFE
      excluded per spec) — both power-preserving sin/cos pans between
      adjacent speakers, verified over every non-reserved index.
      `Some` only when `mixmdate == 1` on an independent substream
      with `acmod < 0x2` (a §E.3.10.8 mixer pans a *mono* associated
      program) and the exists-flag set; `None` defaults the pan word
      to "center" per §E.2.3.1.53/.56 (`PanInfo::CENTER`). Decode PCM
      path unchanged; encoder output byte-identical. Covered by 8 new
      `eac3::bsi::tests` (degree mapping + reserved indices, Table
      E3.15 boundaries + continuity, both power-preservation sweeps,
      5.1 cardinal points, mono + dual-mono parse round-trips with
      cursor checks, and the acmod≥2 / exists-flag-clear guards).
      293 → 301 lib tests, all green.
      **Round 288** lifts the Annex E §E.2.3.1.19-21 premix-compression
      trio — `premixcmpsel` (compression-word select), `drcsrc`
      (DRC-word source), `premixcmpscl` (compression-word scale
      factor) — carried in the `mixdef == 0x1` ("mixing option 2")
      body of an independent substream's mixing-metadata block, from
      parse-and-discard to a typed `Eac3Bsi::premix_compression` field
      (`Option<PremixCompression>`). The struct exposes raw
      `premixcmpsel()` / `drcsrc()` / `premixcmpscl()` round-trip,
      typed `compression_word() -> PremixCompressionWord` (`DynRng` /
      `Compr`, §E.2.3.1.19), `drc_source() -> DrcSource`
      (`ExternalProgram` / `CurrentSubstream`, §E.2.3.1.20), a Table
      E2.7 `scale_ratio() -> Option<f32>` (seven listed codes → `n/6`
      gain-reduction ratios `0/6..=6/6`; the unlisted `0b110`
      codepoint is `is_premixcmpscl_reserved()` → `None`), and an
      `is_recommended_default()` predicate (§E.2.3.1.21 recommended
      `0 / 0 / 000`). `Some` only when `mixmdate == 1`, the substream
      is independent, AND the block selects `mixdef == 0x1`; a
      `mixdef ∈ {0, 2, 3}` block does not carry the three fields as a
      standalone 5-bit group so they stay `None` there (the same
      §E.2.3.1.19-21 fields inside the `mixdef == 0x3` `mixdata` body
      remain an opaque skip). The single-stream decode PCM path is
      unchanged and encoder output stays byte-identical — pure
      surfaced metadata for a downstream §E.3.10 mixer. Covered by 4
      new `eac3::bsi::tests` (the full Table E2.7 ratio sweep + the
      reserved `0b110` codepoint + percentage-caption cross-check; the
      typed-view + recommended-default + 3-bit-masking checks; a
      `mixdef == 0x1` parse round-trip with a cursor check; and the
      `mixdef ∈ {0}` / `mixmdate == 0` None guards). 301 → 305 lib
      tests, all green.

      **Round 293** lays the geometry foundation for the
      enhanced-coupling (`ecplinu == 1`) decode path in a new pure,
      spec-tabulated `eac3::ecpl` module covering A/52:2018 Annex E
      §E.2.3.3.16-19 + §E.3.5.2. `begin_subbnd()` derives
      `ecpl_begin_subbnd` from the 4-bit `ecplbegf` (Table E3.8: `<3`
      → `·2`, `<13` → `+2`, else `·2−10`); `end_subbnd()` derives
      `ecpl_end_subbnd` either from `ecplendf` (`+7`, SPX off) or from
      the SPX begin when spectral extension is co-active (`spxbegf+5`
      for `spxbegf<6`, else `spxbegf·2`) so the two high-frequency
      regions abut. `ECPL_SUBBND_TAB` transcribes Table E3.9
      `ecplsubbndtab[]` — the 22 sub-band start transform-coefficient
      numbers (sub-bands 0-3 are 6 bins wide, 4-21 are 12 bins,
      region 13..=252) plus a one-past-the-end sentinel.
      `DEFAULT_ECPL_BNDSTRC` is Table E2.14 `defecplbndstrc[]`.
      `necplbnd()` implements the §E.2.3.3.19 band count
      (`end−begin` minus the set merge bits) and `band_bin_counts()`
      the §E.3.5.5.1 `nbins_per_bnd_array[]` population. This is the
      verified geometry the still-deferred §E.3.5.5 synthesis
      (amplitude / angle / chaos parameter decode + complex
      coordinate reconstruction) will consume; the decoder still
      rejects `ecplinu == 1` at the synthesis stage rather than emit
      incorrect PCM, but the diagnostic now points at the derivable
      geometry in `eac3::ecpl`. Spec erratum noted: the
      default-banding table is captioned "E2.14" in the document's
      table list but cross-referenced as "E2.13" from §E.2.3.3.18
      (which collides with the standard-coupling default at the
      genuine Table E2.13); the values used are those listed in full
      under the §E.2.3.3.18 heading. 10 new `eac3::ecpl::tests`;
      305 → 315 lib tests, all green.

      **Round 300** lifts the second layer onto that geometry: the
      enhanced-coupling **bitstream syntax** parse. `eac3::ecpl`
      grows `parse_strategy()` (§E.2.3.3.16-19) and `parse_coords()`
      (§E.2.3.3.20-26) plus the typed `EcplStrategy` /
      `EcplCoords` / `EcplChannelParams` carriers. `parse_strategy()`
      reads `ecplbegf` (4 b) → `ecpl_begin_subbnd`, then `ecplendf`
      (4 b) **only when SPX is off** (under SPX the field is omitted
      and the end is derived from `spxbegf`), then `ecplbndstrce`
      (1 b) gating the per-sub-band merge bits over
      `sbnd ∈ [max(9, begin+1), end)` — the sub-bands through
      `max(8, begin)` are known-zero and never sent, so the first
      active sub-band always opens a band. `ecplbndstrce == 0`
      reuses the caller-supplied banding (the Table E2.14 default on
      the first enhanced-coupling block, the previous block's
      structure afterward) with zero bits consumed.
      `parse_coords()` reads `ecplangleintrp` (1 b) then walks the
      fbw channels: a channel's first enhanced-coupling block has
      *implicit* `ecplparam1e`/`ecplparam2e` (all parameters forced
      present per §E.2.3.3.21-22), later blocks read the explicit
      exist bits; `ecplamp[bnd]` (5 b) follows when `param1e`,
      `ecplangle[bnd]` (6 b) + `ecplchaos[bnd]` (3 b) when `param2e`.
      The **first coupled channel** carries no `ecplparam2e`,
      `ecplangle`, `ecplchaos`, or `ecpltrans` (its angle/chaos are
      spec-fixed to 0, §E.2.3.3.24-26); `ecpltrans` (1 b) closes
      each later channel. Both readers advance the bit cursor exactly
      per the reference syntax — verified by bit-exact
      `bit_position()` assertions on hand-built bitstreams — so an
      enhanced-coupling block can now be walked without desync. The
      §E.3.5.5 coordinate reconstruction (turning the decoded indices
      into complex gains via Tables E3.10-E3.12) stays the deferred
      next step; the decoder's `ecplinu == 1` reject is unchanged for
      now (parsing is a pure tested layer, not yet wired into the
      audblk walk). 6 new `eac3::ecpl::tests` (315 → 321 lib tests),
      all green.

      **Round 306** lifts the third layer onto the geometry + syntax:
      the §E.3.5.5.2 / §E.3.5.5.3 **parameter processing** that turns
      the decoded `ecplamp` / `ecplangle` / `ecplchaos` index triples
      into per-bin amplitude and angle arrays. `eac3::ecpl` grows the
      three lookup tables — `ECPL_AMP_EXP_TAB` / `ECPL_AMP_MANT_TAB`
      (Table E3.10), `ECPL_ANGLE_TAB` (Table E3.11, codes 0-31 →
      `0..0.96875`, codes 32-63 → `-1.0..-0.03125`), `ECPL_CHAOS_TAB`
      (Table E3.12, the eighths `-k/7`) — plus the processing
      functions. `ampbnd()` converts a 5-bit amplitude index to a
      linear gain `(ecplampmanttab/32) >> ecplampexptab` (index 0 →
      0 dB, index 30 → ≈ -45.01 dB, index 31 → minus-∞ dB → 0).
      `process_band_amplitudes()` applies the §E.3.5.5.2 chaos
      modification `ampbnd *= 1 + 0.38·chaos` (skipped for the first
      coupled channel and for transient channels, where chaos is 0).
      `expand_bands_to_bins()` fans the per-band values out to the
      per-bin `ampbin[]` using the `ecplbndstrc[]` merge structure.
      `angle_value()` / `chaos_value()` decode the angle/chaos indices
      (forced to 0 on the first coupled channel per §E.3.5.5.3), and
      `interpolate_bin_angles()` implements the `ecplangleintrp == 1`
      linear-interpolation path (band-centre slope, the lower-half-of-
      first-band downward walk, and the closing last-band pass, with
      the spec's `±1.0` wrap guards). These are pure tabulated
      arithmetic over the decoded indices — no multi-block state. The
      two remaining deferred pieces of §E.3.5.5 are the §E.3.5.5.1
      prev/curr/next IMDCT + overlap-add + FFT that produces the
      non-aliased complex coupling channel and the §E.3.5.5.4 per-bin
      complex product + `rand[]` de-correlation arrays (both stateful
      across blocks; the latter needs an init-time RNG). The decoder's
      `ecplinu == 1` synthesis-stage reject is unchanged — this layer
      is pure + tested but not yet wired into the audblk walk. 7 new
      `eac3::ecpl::tests` (321 → 328 lib tests), all green.

      **Round 310** lifts the fourth layer onto the geometry + syntax +
      parameter processing: the §E.3.5.5.3 **closing de-correlation** step
      and the §E.3.5.5.4 **channel transform-coefficient generation** —
      the per-bin complex product that turns the reconstructed
      enhanced-coupling carrier `Z[k]` into each coupled channel's
      coefficients. `eac3::ecpl` grows `apply_decorrelation()` (the
      §E.3.5.5.3 `angle[bin] += chaos[bin]·rand[bin]` term with the
      single-step `±1.0` fold — `chaos ∈ [0,-1]` and `rand ∈ [-1,1]` keep
      the product within `±1.0` so one fold suffices),
      `generate_channel_coeffs()` (the §E.3.5.5.4 complex coordinate
      `ampbin·e^{jπ·angle}` multiplied into `Z`, then the MDCT synthesis
      `chmant = -2·(y[bin]·Zr_ch + y[N/2-1-bin]·Zi_ch)`), and the
      `synthesis_window()` factor `y[bin] = cos(2π·(N/4+0.5)/N·(bin+0.5))`.
      The two de-correlation random sources are also implemented per the
      spec's distinct requirements: `RandNoTrans` caches an init-once
      per-channel `[-1,1]`-uniform array (the §E.3.5.5.3 non-transient
      `rand_notrans[ch][bin]` — "generated once … stay the same for every
      block"), while `gen_rand_trans()` advances a per-block LFSR for the
      transient `rand_trans[ch][bnd]` band values ("new values for each
      block"); both generators are non-normative (the spec only fixes the
      uniform distribution / uniqueness / per-block-vs-once cadence) so a
      deterministic xorshift keeps decodes reproducible, mirroring the
      §E.3.6.4.2 SPX-noise precedent. These remain pure tabulated
      arithmetic over the supplied carrier — no multi-block state. The one
      remaining deferred piece of §E.3.5.5 is §E.3.5.5.1: the
      prev/curr/next IMDCT + overlap-add + FFT that produces `Z[k]` from
      the de-normalised enhanced-coupling mantissas (stateful across
      blocks, wired at the decoder level rather than in this pure layer).
      The decoder's `ecplinu == 1` synthesis-stage reject is unchanged —
      this layer is pure + tested but not yet wired into the audblk walk.
      8 new `eac3::ecpl::tests` (`rand_notrans` distribution / uniqueness /
      generate-once / per-channel-distinct properties, `rand_trans`
      per-block advance, the de-correlation single-step wrap including both
      overflow directions, the `synthesis_window` mirror symmetry, and
      three `generate_channel_coeffs` checks — unity-coordinate
      pure-carrier passthrough, amplitude-linearity, and the angle=π/2
      real→imaginary rotation). 327 → 335 lib tests, all green.
- [x] **§7.10.1 CRC verification API** (round 182). Opt-in
      decoder side: `decoder::verify_packet_crc(syncframe) ->
      CrcStatus` peeks the bsid byte to dispatch AC-3 (double CRC)
      vs E-AC-3 (single `crc2`) and returns `crc1_ok` / `crc2_ok`
      independently so callers can implement either §6.1.2
      strategy (accept on either CRC valid, or require both). The
      verifier implements the spec's **residue check**: shift the
      post-syncword data through the LFSR (with the stored CRC
      fields included) and the register must read zero at the end.
      Validated end-to-end against the existing FFmpeg-encoded
      `tests/fixtures/sine440_stereo.ac3` corpus — every syncframe
      satisfies the residue check; a single body-bit flip then
      breaks at least one CRC. The decode pipeline does not call
      the verifier automatically; it stays opt-in to match the
      spec's "may be used at the discretion of the decoder"
      language and to keep zero-overhead decoding the default. The
      CRC-16 primitive (poly 0x8005, MSB-first) moved from
      `src/encoder.rs` to a new `src/crc.rs` module so the encoder
      + decoder share one byte-exact implementation. **Round 187**
      closes the r182 follow-up: both encoders now emit `crc2` in
      the §7.10.1 **augmented form** (`r(x) = data·x^16 mod g(x)`,
      computed as `ac3_crc_update(ac3_crc_update(0, body), &[0,
      0])`), so a spec-strict residue-checking decoder accepts our
      own bitstreams on `crc2_ok` as well as `crc1_ok`. The three
      decoder tests that previously pinned the r182 deferral now
      assert `crc2_ok = Some(true)` on encoder output.
- [x] Audio-block parse (§5.4.3) — every §5.4.3.x field cited and
      captured into `AudBlkSideInfo` for introspection
- [x] Exponent decode (§7.1) + parametric bit allocation (§7.2)
- [x] Mantissa decode (§7.3) with bap=0 dither (§7.3.4)
- [x] IMDCT synthesis (§7.9) — both 512-point long-block and
      256-point short-block paths use the §7.9.4 FFT-backed
      decomposition (pre-twiddle → N/4-point complex IFFT [N/8 for
      short blocks] → post-twiddle → de-interleave). The direct-form
      reference is kept only as a test oracle in `imdct.rs`.
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
      3/2 form `Lt = L + clev·C − slev·Ls − slev·Rs` /
      `Rt = R + clev·C + slev·Ls + slev·Rs` (plus 3/1's
      single-surround variant and the 2/1 / 2/2 C-drop case),
      normalised by Table 7.32's 0.3204 / 0.2265 coefficients
      (1/3.121 worst-case at default clev=slev=0.707). Selected via
      the new `decoder::make_decoder_ltrt` factory (`make_decoder`
      keeps LoRo, matching FFmpeg's default). On a surround-only 5.1
      AC-3 source the LoRo L/R correlation lands at +0.002
      (uncorrelated independent surround tones summed in-phase)
      while LtRt lands at **−0.972** — the matrix encoder's defining
      anti-phase surround signature, recoverable by a downstream Pro
      Logic decoder. **Round 126** wires Annex D §2.3 (alternate
      bit-stream syntax, `bsid == 6`) into the BSI parser + downmix:
      the `xbsi1` block surfaces `ltrtcmixlev` / `ltrtsurmixlev` /
      `lorocmixlev` / `lorosurmixlev` (Tables D2.3-D2.6, 3-bit
      codewords; reserved surround codes `000/001/010` resolve to
      0.841 per spec) plus the `dmixmod` preferred-target advisory,
      and `Downmix::from_bsi` consults them for the per-target gain
      instead of the fixed §7.8.2 0.707 (LtRt) / body
      `cmixlev`/`surmixlev` (LoRo). Without the Annex D extension the
      matrix is byte-identical to round-120 behaviour. **Round 129**
      extends the same plumbing to E-AC-3: `eac3::bsi` now captures
      the four mixmdata mix-level codewords + `dmixmod` +
      `lfemixlevcod` instead of consuming-and-discarding them, the
      new `Downmix::from_eac3_bsi` / `from_eac3_fields` constructors
      share a private `build` helper with `from_bsi` (matrix is
      coefficient-identical to base AC-3 with the same codes), and
      `process_eac3_frame` runs the §7.8 matrix on the pre-quantised
      f32 PCM via `Eac3DecoderState::indep_pcm_f32()` instead of
      truncating the interleaved buffer to N channels.
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
- [x] E-AC-3 decoder — **spectral-extension attenuation (SPXATTEN)**
      decode (round 172 / r172). Lifts the round-2 `audfrm.spxattene == 1`
      whole-frame reject: the audfrm parser now surfaces
      `chinspxatten[ch]` (1 bit) + `spxattencod[ch]` (5 bits,
      §2.3.2.24-25) into per-channel state, and
      `audblk::apply_spectral_extension` applies the §3.6.4.2.3 5-tap
      symmetric notch filter `[T[0], T[1], T[2], T[1], T[0]]` at the
      baseband / extension border AND at every wrap point flagged
      during the §3.6.4.1 translation copy. The 32-row attenuation
      table (Table E3.14, `SPX_ATTEN_TABLE`) is transcribed directly
      from the spec text; the kernel's last two taps come from
      symmetry per spec ("last two attenuation values are determined
      by symmetry and are not explicitly stored in the table"). When
      `chinspxatten[ch] == 0` for a channel (or `spxattene == 0` for
      the whole frame), synthesis is byte-identical to the round-100
      baseline. No FFmpeg-encoded fixture in the corpus carries
      `spxattene == 1`, so the landing is covered by 5 unit tests in
      `audblk::spx_tests` rather than an end-to-end PSNR gate; matches
      the round-103 (TPNP) / round-113 (LFE-AHT) / round-117
      (coupling-AHT) precedent for corpus-untestable decoder paths.
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
- [x] E-AC-3 decoder — **dependent-substream chanmap routing**
      (round 196 / r196). The 16-bit `chanmap` field (§E.2.3.1.8 /
      Table E2.5) now decodes into an ordered list of physical channel
      locations on every dep substream: a new
      `eac3::chanmap::ChannelLocation` enum covers all 22 distinct
      Table E2.5 locations including the 6 pair-bits (Lc/Rc, Lrs/Rrs,
      Lsd/Rsd, Lw/Rw, Vhl/Vhr, Lts/Rts) which each expand to two
      consecutive channels per the spec text. The decoder enforces the
      §E.2.3.1.8 invariant that the expanded chanmap count equal the
      dep substream's coded channel count (`acmod_nfchans + lfeon`)
      and surfaces the resolved location list on
      `Eac3DecoderState.dep_locations` and `DecodedFrame.dep_locations`
      — so a future 7.1 WAV-mask reorderer or a chanmap-aware §7.8
      downmix can route the appended dep channels without re-parsing
      the bitstream. The splice itself still appends dep channels at
      the end of the indep program; the new metadata is the foundation
      for future routing work, not a behavioural change for current
      acmod-native consumers. When `chanmape == 0` the locations
      default to the natural-acmod order per §E.2.3.1.7. Covered by
      6 new unit tests (both spec examples from §E.2.3.1.8 verbatim,
      the in-tree 7.1 encoder's Lrs/Rrs pair, single-bit-only decodes,
      MSB/LSB extremes, and the count-mismatch rejection) and a new
      integration test that round-trips the encoder's 7.1 indep+dep
      pair through the in-tree decoder asserting
      `dep_locations == [LeftRearSurround, RightRearSurround]` on
      every packet. **Round 274** adds the spec-grounded classification
      + re-emit surface the routing foundation was waiting on:
      `ChannelLocation::ALL` (the 22 distinct Table E2.5 variants in bit
      order, pair-bits expanded left-then-right), `table_e2_5_bit()` (the
      `0..=15` location bit a variant decoded from — the exact inverse of
      `expand_chanmap_locations`, both pair halves sharing the row's
      single bit), `chanmap_weight()` (the MSB-first `1 << (15 - bit)`
      field weight per §E.2.3.1.8, so a decoded location list OR's
      straight back into the original 16-bit `chanmap` without
      double-counting pair-bits), `is_pair_half()` + `pair_companion()`
      (the 12 expanded halves of the six pair-bits and the other half of
      each), `is_lfe()` (bits 14/15), `is_height()` (the SMPTE 428-3
      above-plane rows — `Ts` / `Vhl/Vhr` / `Vhc` / `Lts/Rts`), and
      `is_surround()` (the ear-level `Ls/Rs` / `Cs` / `Lrs/Rrs` /
      `Lsd/Rsd` rows). Pure metadata over the already-decoded location
      list — the PCM path is unchanged and encoder output is
      byte-identical — covered by 8 new `eac3::chanmap::tests` (285 lib
      tests, all green).
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
