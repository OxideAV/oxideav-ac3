//! Enhanced AC-3 (E-AC-3 / Dolby Digital Plus) — ATSC A/52 Annex E.
//!
//! E-AC-3 is **not** backwards-compatible with AC-3 at the bit-stream
//! level: the syncinfo loses crc1, the bsi grows new fields
//! (`strmtyp`, `substreamid`, `frmsiz`, `numblkscod`), the audio frame
//! gains a new `audfrm()` element with frame-level strategy flags, and
//! every audio block carries SPX, AHT, and enhanced-coupling fields
//! that don't exist in the base spec. The bsid value (16, or 11..15
//! for backward-compatible variants) selects the syntax — base-AC-3
//! decoders MUST mute on bsid > 10 per A/52 §E.2.3.1.6.
//!
//! ## Round-1 scope (this commit)
//!
//! * **BSI parser** ([`bsi`]) covers Table E1.2 in full: stream type,
//!   substream id, frame size, sample-rate code (incl. fscod2 reduced
//!   rates), number of blocks, channel layout, dialnorm, compression,
//!   `chanmape`/`chanmap` for dependent substreams, the entire
//!   `mixmdate`/`infomdate`/`addbsi` opt-in chain. Values not used
//!   by the round-1 decoder are still consumed bit-accurately so the
//!   bit cursor lands on the start of `audfrm()`.
//! * **audfrm parser** ([`audfrm`]) covers Table E1.3: the 11 strategy
//!   flags, frame-level exponent strategies (`frmcplexpstr`,
//!   `frmchexpstr`, `lfeexpstr` runs), AHT in-use flags, frame-level
//!   SNR offsets, transient pre-noise + spectral-extension attenuation
//!   parameters, and the per-block start info. Like BSI, every field
//!   is consumed even when its value is ignored.
//! * **audblk parsing** is deferred to round 2 — the round-1 decoder
//!   does not walk per-block bits; instead, after parsing BSI +
//!   audfrm, it advances by `bsi.frame_bytes` and emits silent PCM
//!   of shape `bsi.num_blocks × 256 × nchans` S16. Coupling,
//!   rematrixing, IMDCT, overlap-add all land in round 2.
//! * **Decoder** ([`decoder`]) wires BSI + audfrm + silent emit into
//!   a top-level per-substream decode that produces `1536 × nchans`
//!   S16 zeros per syncframe (or `numblks × 256 × nchans` for short-
//!   block frames). This is enough to unblock the corpus tests,
//!   which now report measurable per-channel diffs against the
//!   FFmpeg reference (low PSNR because round-1 is silent, but the
//!   test machinery runs end-to-end instead of bailing at `bsi.rs`).
//! * **Encoder** ([`encoder`]) is the existing round-0 emitter for
//!   1.0 / 2.0 / 5.1 indep substreams + 7.1 indep+dep pair. Untouched
//!   by this commit.
//!
//! ## Round 6 (this commit) — Adaptive Hybrid Transform (AHT)
//!
//! * **VQ codebooks E4.1..E4.7** — 956 entries × 6 i16 transcribed
//!   from A/52:2018 Annex E §4 into [`tables::aht_codebooks`]. Plus
//!   the `hebap` pointer table (E3.1) and quantiser-bit table (E3.2)
//!   in [`aht`].
//! * **Phase-B audfrm parse** — [`audfrm::parse_with`] now stops at
//!   the AHT anchor when `ahte == 1`, leaving the variable-width
//!   `chahtinu` / `cplahtinu` / `lfeahtinu` bits for [`audfrm::parse_phase_b`]
//!   once the dsp pre-walk has produced `nchregs[ch]` / `ncplregs` /
//!   `nlferegs` from the per-block exponent strategies.
//! * **AHT mantissa decode** ([`aht::vq_lookup`] /
//!   [`aht::read_scalar_aht_mantissas`]) + **§3.4.5 inverse DCT-II**
//!   ([`aht::idct_ii_6`]) routed through a per-frame coefficient
//!   cache in [`dsp::decode_indep_audblks`]. AHT-active channels read
//!   their full 6×nmant mantissa block in audblk[0]'s mantissa step;
//!   audblks 1..5 pull pre-computed coefficients from the cache.
//! * **Round-6 scope is mono-only** — multichannel / LFE / coupled AHT
//!   needs the 2-pass nchregs probe (round 7). The
//!   `eac3-low-bitrate-32kbps` fixture is the only AHT-active fixture
//!   in the corpus.
//!
//! ## Deferred to round 7 and beyond
//!
//! * Multichannel / coupled / LFE AHT — needs the iterative nchregs
//!   probe described in §3.4.2 (or a deferred audfrm parse with full
//!   audblk pre-walk). The `cplahtinu` / `lfeahtinu` bit streams
//!   themselves are already wired through [`audfrm::AudFrm`].
//! * **Spectral Extension (SPX)** decode — **landed**. `spxinu == 1`
//!   blocks now parse the full §E.2.3.3 strategy + coordinate fields
//!   (chinspx / spxstrtf / spxbegf / spxendf / spxbndstrc / spxcoe /
//!   spxblnd / mstrspxco / spxcoexp / spxcomant) and run the §E.3.6
//!   high-frequency regeneration (translate → noise-blend → coordinate
//!   scale) in [`crate::audblk::dsp_block`]. The `nrematbd` (§E.3.3.2)
//!   and `cplendf`-when-SPX (§E.3.3.1) derivations are wired so the bit
//!   cursor no longer drifts on the SPX-strategy fields. The corpus
//!   fixtures `eac3-stereo-48000-192kbps`, `eac3-256-coeff-block`, and
//!   `eac3-from-ac3-bitstream-recombination` are *additionally* gated
//!   by a pre-existing coupling/bit-allocation cursor drift on a subset
//!   of their non-SPX frames (the same drift that leaves a handful of
//!   AC-3 fixtures muted), so end-to-end PSNR on those three is still
//!   floor-bound until that drift is fixed. The non-SPX fixtures decode
//!   cleanly: `eac3-5.1-48000-384kbps` at **90 dB**,
//!   `eac3-low-rate-stereo-64kbps` at **72 dB**,
//!   `eac3-low-bitrate-32kbps` at **66 dB**.
//! * **Per-block SNR-offset** (`snroffststr != 0`) — needs the
//!   audblk-level `snroffste` parser. Same situation as above.
//! * **Transient pre-noise processing** (`transproce == 1`) —
//!   **landed** (round 103). The per-channel `chintransproc` /
//!   `transprocloc` / `transproclen` fields are stored on
//!   [`audfrm::AudFrm`] and the §E.3.7.2 PCM-domain time-scaling
//!   synthesis runs in [`dsp::decode_indep_audblks`] after overlap-add
//!   (see `dsp::apply_transient_prenoise`). The cross-frame reference
//!   case (§E.3.7.1) is clamped to the current frame for now.
//! * **256-coeff-block-per-syncframe variants** (`numblkscod < 3`).
//!   The parser handles them; the silent-PCM path produces an output
//!   of the right length only.

pub mod aht;
pub mod audfrm;
pub mod bsi;
pub mod decoder;
pub mod dsp;
pub mod encoder;
pub mod tables;

// Re-exports — keep the public surface identical to the old single-
// file `eac3.rs` so external callers (the encoder integration test in
// `tests/eac3_ffmpeg.rs` and the workspace registration in
// `crate::lib::register`) don't need to change.
pub use bsi::{Bsi as Eac3Bsi, BSID_BASE_AC3_MAX, EAC3_BSID};
pub use decoder::{decode_eac3_packet, Eac3DecoderState};
pub use encoder::{make_encoder, CODEC_ID_STR};
