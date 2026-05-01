//! Pure-Rust **AC-3 (Dolby Digital)** audio decoder.
//!
//! Implements ATSC A/52:2018 (= ETSI TS 102 366) elementary AC-3 streams.
//!
//! # Status
//!
//! Initial skeleton: sync-frame + BSI parsing is complete and the decoder
//! emits PCM frames (currently silence) with correct shape — real IMDCT,
//! bit allocation, exponent decode, and mantissa dequantization are
//! staged for follow-up commits.
//!
//! Architecture follows the spec's natural pipeline:
//!
//! 1. [`syncinfo`] — 16-bit sync word 0x0B77, crc1, fscod, frmsizecod,
//!    frame-length table lookup (§5.3.1 / §5.4.1 / Table 5.18).
//! 2. [`bsi`] — Bit Stream Information: bsid (≤8 for base AC-3), bsmod,
//!    acmod → channel-layout + lfeon + dialnorm + optional timecodes.
//! 3. `audblk` + transform synthesis — TODO.

#![allow(clippy::needless_range_loop)]

pub mod audblk;
pub mod bsi;
pub mod decoder;
pub mod downmix;
pub mod eac3;
pub mod encoder;
pub mod imdct;
pub mod mdct;
pub mod syncinfo;
pub mod tables;

use oxideav_core::{CodecCapabilities, CodecId, CodecParameters, CodecTag, Result};
use oxideav_core::{CodecInfo, CodecRegistry, Decoder, Encoder};

pub const CODEC_ID_STR: &str = "ac3";
pub const CODEC_ID_STR_EAC3: &str = "eac3";

/// Register the AC-3 + E-AC-3 decoder + encoder with the supplied codec
/// registry.
pub fn register(reg: &mut CodecRegistry) {
    let cid = CodecId::new(CODEC_ID_STR);
    let dec_caps = CodecCapabilities::audio("ac3_sw_dec")
        .with_lossy(true)
        .with_intra_only(true)
        .with_max_channels(6)
        .with_max_sample_rate(48_000);
    let enc_caps = CodecCapabilities::audio("ac3_sw_enc")
        .with_lossy(true)
        .with_intra_only(true)
        .with_max_channels(6)
        .with_max_sample_rate(48_000);
    // Container tag claims. AC-3 is identified by:
    //   - WAVEFORMATEX::wFormatTag = 0x2000 (AVI / WAV)
    //   - MP4 ObjectTypeIndication 0xA5 (for MP4/ISOBMFF carriage)
    //   - Matroska CodecID "A_AC3"
    reg.register(
        CodecInfo::new(cid.clone())
            .capabilities(dec_caps.clone())
            .decoder(make_decoder)
            .tag(CodecTag::wave_format(0x2000)),
    );
    reg.register(
        CodecInfo::new(cid.clone())
            .capabilities(dec_caps.clone())
            .decoder(make_decoder)
            .tag(CodecTag::mp4_object_type(0xA5)),
    );
    reg.register(
        CodecInfo::new(cid.clone())
            .capabilities(dec_caps)
            .decoder(make_decoder)
            .tag(CodecTag::matroska("A_AC3")),
    );
    // Encoder registration — keyed on codec id only (no container tag
    // so it gets picked up regardless of output muxer).
    reg.register(
        CodecInfo::new(cid)
            .capabilities(enc_caps)
            .encoder(make_encoder),
    );

    // E-AC-3 encoder (Annex E independent substream, round-1 scope:
    // mono / stereo, no coupling / SPX / AHT). No decoder yet.
    let eac3_cid = CodecId::new(CODEC_ID_STR_EAC3);
    let eac3_enc_caps = CodecCapabilities::audio("eac3_sw_enc")
        .with_lossy(true)
        .with_intra_only(true)
        .with_max_channels(2)
        .with_max_sample_rate(48_000);
    reg.register(
        CodecInfo::new(eac3_cid)
            .capabilities(eac3_enc_caps)
            .encoder(make_eac3_encoder),
    );
}

fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    decoder::make_decoder(params)
}

fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    encoder::make_encoder(params)
}

fn make_eac3_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    eac3::make_encoder(params)
}
