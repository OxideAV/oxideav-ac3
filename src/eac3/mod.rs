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
//! ## Deferred to round 2 and beyond
//!
//! * Real DSP (exponent decode, parametric bit allocation, mantissa
//!   dequantization, decoupling, IMDCT, overlap-add) for an indep
//!   substream — most of the AC-3 helpers in [`crate::audblk`] can
//!   be reused once the BSI/audfrm fields are translated into the
//!   shape `Ac3State`/`run_bit_allocation`/`unpack_mantissas` expect.
//! * Dependent substream recombination (the `eac3-from-ac3-bitstream-
//!   recombination` fixture and the `eac3-5.1-side-768kbps` 5.1+side
//!   case) — needs a multi-substream syncframe assembler that
//!   threads `chanmap` channels into the indep substream's output
//!   PCM matrix.
//! * Adaptive Hybrid Transform (AHT) and Spectral Extension (SPX) —
//!   conditional on `ahte` / `spxinu` flags surfaced by the audfrm /
//!   audblk parsers. Both are §E features the round-1 parser will
//!   correctly skip past on streams that opt out.
//! * 256-coeff-block-per-syncframe variants (`numblkscod < 3`).
//!   The `eac3-256-coeff-block` fixture uses `numblkscod=0` (1 block
//!   per syncframe = 256 samples) — the parser handles it, but the
//!   silent-PCM path produces an output of the right length only.

pub mod audfrm;
pub mod bsi;
pub mod decoder;
pub mod dsp;
pub mod encoder;

// Re-exports — keep the public surface identical to the old single-
// file `eac3.rs` so external callers (the encoder integration test in
// `tests/eac3_ffmpeg.rs` and the workspace registration in
// `crate::lib::register`) don't need to change.
pub use bsi::{Bsi as Eac3Bsi, BSID_BASE_AC3_MAX, EAC3_BSID};
pub use decoder::{decode_eac3_packet, Eac3DecoderState};
pub use encoder::{make_encoder, CODEC_ID_STR};
