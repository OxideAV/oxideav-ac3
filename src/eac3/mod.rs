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
//! ## Module layout
//!
//! * **[`bsi`]** — Table E1.2 parser: stream type, substream id,
//!   frame size, sample-rate code (incl. fscod2 reduced rates),
//!   number of blocks, channel layout, dialnorm, compression,
//!   `chanmape`/`chanmap` for dependent substreams, plus the full
//!   `mixmdate`/`infomdate`/`addbsi` opt-in chain.
//! * **[`audfrm`]** — Table E1.3 parser: the 11 strategy flags,
//!   frame-level exponent strategies (`frmcplexpstr`,
//!   `frmchexpstr`, `lfeexpstr` runs), AHT in-use flags, frame-level
//!   SNR offsets, transient pre-noise + spectral-extension attenuation
//!   parameters, per-block start info. Two-phase: [`audfrm::parse_with`]
//!   stops at the AHT anchor when `ahte == 1`; once the dsp pre-walk has
//!   produced `nchregs[ch]` / `ncplregs` / `nlferegs` from the per-block
//!   exponent strategies, [`audfrm::parse_phase_b`] consumes the
//!   variable-width `chahtinu` / `cplahtinu` / `lfeahtinu` bits.
//! * **[`aht`]** — Adaptive Hybrid Transform (§3.4). VQ codebooks
//!   E4.1..E4.7 (956 × 6 i16) + `hebap` pointer table (E3.1) +
//!   quantiser-bit table (E3.2). [`aht::vq_lookup`] /
//!   [`aht::read_scalar_aht_mantissas`] plus the §3.4.5 inverse
//!   DCT-II ([`aht::idct_ii_6`]).
//! * **[`dsp`]** — per-frame DSP: §7.4 decouple, AHT mantissa cache,
//!   §3.6 spectral extension (translate → noise-blend → coordinate
//!   scale + §3.6.4.2.3 SPXATTEN border notch), §3.7.2 transient
//!   pre-noise processing (PCM-domain time-scaling synthesis).
//! * **[`decoder`]** — top-level per-substream decode. Routes packets
//!   with `bsid ∈ {11..=16}` through BSI → audfrm phase-A → dsp
//!   pre-walk → audfrm phase-B → audblk DSP → IMDCT → overlap-add →
//!   §7.8 downmix.
//! * **[`encoder`]** — Annex E encoder. Indep substream for
//!   1.0 / 2.0 / 5.1 layouts (acmod ∈ {1, 2, 7} with `lfeon=1` for
//!   5.1); 7.1 input emits an indep+dep substream pair (indep
//!   carries the 5.1 program, dep 0 carries Lb/Rb back surrounds
//!   with chanmap bit 6 set per §E.2.3.1.7-8 / §E.3.8.2). Encoder-
//!   side SPX, AHT, and enhanced coupling are out of scope.
//!
//! ## Known decoder gaps
//!
//! * **Enhanced coupling** (`ecplinu == 1`, §E.1.3.3.7-26) is
//!   rejected as `Unsupported`. Standard coupling is in.
//! * **Cross-frame transient pre-noise reference** (§E.3.7.1) is
//!   clamped to the current frame; intra-frame transients (§E.3.7.2)
//!   are fully synthesised.
//! * Three corpus fixtures (`eac3-stereo-48000-192kbps`,
//!   `eac3-256-coeff-block`, `eac3-from-ac3-bitstream-recombination`)
//!   remain floor-bound on a pre-existing coupling/bit-allocation
//!   cursor drift that also leaves a handful of base-AC-3 fixtures
//!   muted; non-affected fixtures decode at 66-90 dB PSNR (see
//!   crate `README.md` for the per-fixture numbers).

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
