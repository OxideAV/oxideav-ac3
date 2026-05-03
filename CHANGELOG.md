# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
