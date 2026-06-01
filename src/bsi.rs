//! AC-3 Bit Stream Information — `bsi()` (§5.3.2 / §5.4.2).
//!
//! The BSI immediately follows the 5-byte syncinfo and describes the
//! service characteristics: stream identification, channel layout,
//! dialogue normalization, compression, language, timecode, etc.
//!
//! This module parses the base (bsid ≤ 8) layout. Annex E (E-AC-3,
//! bsid=16) is a separate syntax not handled here — that's a future
//! `oxideav-eac3` crate.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::tables::acmod_nfchans;

/// Largest `bsid` value accepted by the base AC-3 BSI parser. Streams
/// at higher `bsid` values use the Annex E (E-AC-3) syntax — the
/// top-level decoder dispatches them to [`crate::eac3::decoder`].
///
/// The spec mandates muting for `bsid > 8` in pure AC-3 decoders
/// (§5.4.2.7) but accepts up to 10 as a small safety margin for
/// near-compatible streams (legacy bsid=9..=10 variants of base AC-3
/// that still parse the same syntax). bsid 11..=16 is canonical
/// E-AC-3 territory.
pub const MAX_BSID_BASE: u8 = 10;

/// Parsed BSI — just the fields a decoder actually needs. Optional
/// service-metadata (compression gain, language code, timecodes,
/// `addbsi`) is consumed but not surfaced since the decoder doesn't
/// apply any of it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Bsi {
    /// bsid — bit stream identification. Spec mandates decoders built
    /// to A/52 mute for `bsid > 8` (Annex E E-AC-3 is a different
    /// syntax). We surface the raw value and let the decoder decide.
    pub bsid: u8,
    pub bsmod: u8,
    pub acmod: u8,
    /// Number of full-bandwidth channels per Table 5.8.
    pub nfchans: u8,
    /// `true` when the low-frequency-effects channel is on.
    pub lfeon: bool,
    /// Total channel count — `nfchans + lfeon`.
    pub nchans: u8,
    /// Dialogue normalization, 1..=31 dB below reference. 0 is reserved
    /// and spec says to treat it as 31.
    pub dialnorm: u8,
    /// Center mix-level coefficient code (cmixlev) for acmod with 3
    /// front channels; 0xFF when absent.
    pub cmixlev: u8,
    /// Surround mix-level coefficient code (surmixlev); 0xFF when absent.
    pub surmixlev: u8,
    /// Dolby-Surround flag for 2/0 stereo streams; 0xFF when absent.
    pub dsurmod: u8,
    /// Annex D §2.3 "alternate bit stream syntax" mix-level extensions.
    /// `Some` only when `bsid == 6` AND the encoder set `xbsi1e == 1`;
    /// `None` otherwise. The four 3-bit codewords (`ltrtcmixlev` /
    /// `ltrtsurmixlev` / `lorocmixlev` / `lorosurmixlev`) refine the
    /// 2-bit `cmixlev` / `surmixlev` defaults specifically for the
    /// LtRt vs LoRo downmix targets — see [`crate::downmix`].
    pub annex_d_mix_levels: Option<AnnexDMixLevels>,
    /// Annex D §2.3.1.2 preferred-stereo-downmix-mode (`dmixmod`); 0xFF
    /// when absent. `00` = not indicated, `01` = LtRt preferred,
    /// `10` = LoRo preferred, `11` = reserved.
    pub dmixmod: u8,
    /// Heavy compression gain word (`compr`, §5.4.2.10 / §7.7.2.2). For
    /// 1+1 dual-mono (`acmod == 0`) this is the Ch1 word; Ch2 is
    /// surfaced separately as [`Bsi::compr_ch2`]. `Some` when
    /// `compre == 1` in the bitstream; `None` when the encoder did not
    /// emit a heavy-compression word for this syncframe (the spec's
    /// "use `dynrng` instead for this frame" branch).
    pub compr: Option<CompressionGain>,
    /// Ch2 heavy compression gain word for 1+1 dual-mono only. `None`
    /// outside `acmod == 0`, or inside `acmod == 0` when `compr2e == 0`.
    pub compr_ch2: Option<CompressionGain>,
    /// Annex D §2.3.1.8 Dolby Surround EX mode (`dsurexmod`, 2 bits,
    /// Table D2.7). `Some` only when `bsid == 6` and the `xbsi2e` block
    /// is present; `None` otherwise. Per the spec note the field's
    /// semantics are only defined for `acmod ∈ {6, 7}` (2/2 or 3/2) —
    /// the parser still surfaces the raw decoded variant for other
    /// `acmod` values so a caller can decide whether to honour the
    /// hint (encoders treat reserved-combination codes as advisory).
    pub dsurexmod: Option<DolbySurroundExMode>,
    /// Annex D §2.3.1.9 Dolby Headphone mode (`dheadphonmod`, 2 bits,
    /// Table D2.8). `Some` only when `bsid == 6` and the `xbsi2e`
    /// block is present; `None` otherwise. Per the spec note the
    /// field's semantics are only defined for `acmod == 2` (2/0
    /// stereo); the parser still surfaces the raw decoded variant for
    /// other `acmod` values.
    pub dheadphonmod: Option<DolbyHeadphoneMode>,
    /// Annex D §2.3.1.10 A/D converter type (`adconvtyp`, 1 bit, Table
    /// D2.9). `Some` only when `bsid == 6` and the `xbsi2e` block is
    /// present; `None` otherwise. `Standard` = generic 24-bit PCM
    /// converter; `Hdcd` = HDCD-encoded source.
    pub adconvtyp: Option<AdConverterType>,
    /// Absolute bit position (in bits, measured from the first byte of
    /// `bsi()` input) where the BSI ended. Callers use this to skip
    /// straight to the audio-block area.
    pub bits_consumed: u64,
}

impl Bsi {
    /// Decode the raw `bsmod` value into a typed [`BitStreamMode`]
    /// per Table 5.7. `bsmod == 0b111` is overloaded by `acmod` and
    /// returns either [`BitStreamMode::VoiceOver`] (acmod=0b001) or
    /// [`BitStreamMode::Karaoke`] (acmod ∈ {0b010..=0b111}); the
    /// `bsmod==0b111 && acmod==0b000` combination is not defined by
    /// the spec and maps to [`BitStreamMode::Reserved`].
    ///
    /// This is a thin convenience over [`Bsi::bsmod`] + [`Bsi::acmod`]
    /// — the raw fields stay authoritative and an unmatched value
    /// never panics here. A player can use the typed result to drive
    /// service-routing (e.g. mute the dialogue-only `Dialogue` track
    /// when also playing a main service, or surface the
    /// `VisuallyImpaired` track to a screen-reader bus).
    pub fn service_type(&self) -> BitStreamMode {
        BitStreamMode::from_bsmod_acmod(self.bsmod, self.acmod)
    }
}

/// Service-type classification of an AC-3 bit stream — Table 5.7
/// "Bit Stream Mode". The encoding is keyed on `bsmod`; the `'111'`
/// codepoint is overloaded and resolves with `acmod`'s help.
///
/// Spec §5.4.2.2: `bsmod` indicates whether the bit stream carries a
/// main audio service (CM, ME, karaoke), an associated service
/// (VI, HI, D, C, E, VO), or — for the unused `bsmod==0b111`
/// /`acmod==0b000` combination — nothing defined.
///
/// Routing recommendations (from §5.4.2.2 and Table 5.7):
///
/// * **Main** services (`CompleteMain` / `MusicAndEffects` / `Karaoke`):
///   the primary playback target. A receiver normally selects exactly
///   one main service at a time.
/// * **Associated** services may be mixed *on top of* a main service
///   (e.g. `VisuallyImpaired` and `HearingImpaired` are descriptive
///   narration / cleaned-dialogue mixes intended to substitute or
///   augment the main mix; `Commentary` / `Emergency` / `VoiceOver`
///   typically mix on top of a separate main).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitStreamMode {
    /// `bsmod=0b000` — main audio service: complete main (CM).
    CompleteMain,
    /// `bsmod=0b001` — main audio service: music and effects (ME).
    MusicAndEffects,
    /// `bsmod=0b010` — associated service: visually impaired (VI).
    VisuallyImpaired,
    /// `bsmod=0b011` — associated service: hearing impaired (HI).
    HearingImpaired,
    /// `bsmod=0b100` — associated service: dialogue (D).
    Dialogue,
    /// `bsmod=0b101` — associated service: commentary (C).
    Commentary,
    /// `bsmod=0b110` — associated service: emergency (E).
    Emergency,
    /// `bsmod=0b111` + `acmod=0b001` (mono) — associated service:
    /// voice over (VO).
    VoiceOver,
    /// `bsmod=0b111` + `acmod ∈ {0b010..=0b111}` — main audio
    /// service: karaoke.
    Karaoke,
    /// `bsmod=0b111` + `acmod=0b000` — undefined by Table 5.7
    /// (`bsmod==0b111` collides with the 1+1 dual-mono `acmod`).
    /// Decoders should treat this as malformed metadata, not error.
    Reserved,
}

impl BitStreamMode {
    /// Resolve a `(bsmod, acmod)` pair into a typed service-type per
    /// Table 5.7. Only the low 3 bits of each input are consulted.
    pub fn from_bsmod_acmod(bsmod: u8, acmod: u8) -> Self {
        match bsmod & 0x7 {
            0b000 => BitStreamMode::CompleteMain,
            0b001 => BitStreamMode::MusicAndEffects,
            0b010 => BitStreamMode::VisuallyImpaired,
            0b011 => BitStreamMode::HearingImpaired,
            0b100 => BitStreamMode::Dialogue,
            0b101 => BitStreamMode::Commentary,
            0b110 => BitStreamMode::Emergency,
            0b111 => match acmod & 0x7 {
                0b000 => BitStreamMode::Reserved,
                0b001 => BitStreamMode::VoiceOver,
                _ => BitStreamMode::Karaoke,
            },
            _ => unreachable!(),
        }
    }

    /// `true` for a main audio service (CM, ME, or karaoke). A
    /// receiver picking a default playback target should normally
    /// route a main service first.
    pub fn is_main(self) -> bool {
        matches!(
            self,
            BitStreamMode::CompleteMain | BitStreamMode::MusicAndEffects | BitStreamMode::Karaoke
        )
    }

    /// `true` for an associated service (VI / HI / D / C / E / VO).
    /// These are typically mixed on top of a separately-decoded main
    /// service.
    pub fn is_associated(self) -> bool {
        matches!(
            self,
            BitStreamMode::VisuallyImpaired
                | BitStreamMode::HearingImpaired
                | BitStreamMode::Dialogue
                | BitStreamMode::Commentary
                | BitStreamMode::Emergency
                | BitStreamMode::VoiceOver
        )
    }

    /// Short ASCII mnemonic per Table 5.7 (e.g. "CM", "ME", "VI",
    /// "HI", "D", "C", "E", "VO", "K"). Stable for UI / logging.
    /// Returns "?" for [`BitStreamMode::Reserved`].
    pub fn mnemonic(self) -> &'static str {
        match self {
            BitStreamMode::CompleteMain => "CM",
            BitStreamMode::MusicAndEffects => "ME",
            BitStreamMode::VisuallyImpaired => "VI",
            BitStreamMode::HearingImpaired => "HI",
            BitStreamMode::Dialogue => "D",
            BitStreamMode::Commentary => "C",
            BitStreamMode::Emergency => "E",
            BitStreamMode::VoiceOver => "VO",
            BitStreamMode::Karaoke => "K",
            BitStreamMode::Reserved => "?",
        }
    }
}

/// Heavy compression gain word per Table 7.30 + §7.7.2.2.
///
/// The wire field is 8 bits, split as `X0 X1 X2 X3 . Y4 Y5 Y6 Y7`:
///
/// * The upper nibble `X` is a 4-bit signed integer in the range
///   `-8..=+7` (transmitted MSB-first). It contributes a gain of
///   `(X + 1) * 6.02 dB` — i.e. an arithmetic shift on the PCM
///   sample. The 16 `X` codepoints span `+48.16 dB` (`X=7`) down to
///   `-42.14 dB` (`X=-8`).
/// * The lower nibble `Y` is an unsigned fractional value with an
///   implicit leading `1`, read as `0.1 Y4 Y5 Y6 Y7` in base 2 — i.e.
///   `(16 + Y) / 32`, ranging from `16/32 = 0.5` to `31/32`. It
///   represents a linear *attenuation* between `0` dB and `-6.02` dB.
///
/// The combined linear gain is `linear = 2^(X+1) * (16 + Y) / 32`;
/// the combined dB gain runs from `-48.16 dB` (`X=-8`, `Y=0`,
/// linear `0.5 * 0.5 = 0.25`) up to `+47.89 dB` (`X=7`, `Y=15`,
/// linear `256 * 31/32`).
///
/// Per §7.7.2 the `compr` element is intended to bound the **peak**
/// playback level for downstream feeds with restricted dynamic range
/// (RF modulators, hotel-room feeds, etc.). Decoders that have been
/// instructed to "compress on" SHOULD apply `compr` when present, and
/// fall back to `dynrng` for syncframes that omit it (§7.7.2.1).
/// `oxideav-ac3`'s current PCM path does neither — both `compr` and
/// `dynrng` are left for the application to apply downstream — but
/// surfacing the typed value here lets a player implement the policy
/// without re-parsing the BSI.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CompressionGain {
    raw: u8,
}

impl CompressionGain {
    /// Wrap the 8-bit wire value verbatim. Every byte pattern is valid
    /// per Table 7.30 (all 256 codepoints map to a defined gain).
    pub fn from_byte(raw: u8) -> Self {
        Self { raw }
    }

    /// Underlying 8-bit wire value — `X0 X1 X2 X3 Y4 Y5 Y6 Y7` packed
    /// MSB-first.
    pub fn raw(self) -> u8 {
        self.raw
    }

    /// Signed `X` field, in `-8..=+7`. Per the §7.7.2.2 description
    /// the four upper bits encode `X` as a 4-bit signed integer
    /// (two's-complement convention: `0b1111 → -1`, `0b1000 → -8`).
    pub fn x(self) -> i8 {
        let x4 = (self.raw >> 4) & 0xF;
        // Sign-extend the 4-bit field.
        if x4 & 0x8 != 0 {
            (x4 as i16 - 16) as i8
        } else {
            x4 as i8
        }
    }

    /// Unsigned `Y` field, in `0..=15`. Combined with the implicit
    /// leading `1`, it represents `(16 + Y) / 32` per §7.7.2.2.
    pub fn y(self) -> u8 {
        self.raw & 0xF
    }

    /// Linear-domain gain coefficient — multiply the decoded PCM by
    /// this scalar. Equals `2^(X+1) * (16 + Y) / 32`.
    pub fn linear(self) -> f32 {
        let x_shift = (self.x() as i32) + 1; // -7..=+8
        let y_frac = (16.0 + self.y() as f32) / 32.0; // 0.5..=31/32
                                                      // 2^x_shift via direct floating multiply: x_shift fits in i32 well
                                                      // within f32 exponent range (-7..=+8).
        let two_pow = 2.0f32.powi(x_shift);
        two_pow * y_frac
    }

    /// dB-domain gain — `20 * log10(linear())`. Range
    /// `-48.16 dB ..= +47.89 dB` per Table 7.30 + §7.7.2.2.
    pub fn decibels(self) -> f32 {
        20.0 * self.linear().log10()
    }
}

/// Annex D §2.3.1.8 Dolby Surround EX mode (Table D2.7).
///
/// Surfaced on [`Bsi::dsurexmod`] when `bsid == 6` and the `xbsi2e`
/// block is present. The spec note constrains the meaningful range of
/// the field to `acmod ∈ {6, 7}` (2/2 and 3/2 — the only layouts that
/// carry a stereo surround pair); for other `acmod` values the field
/// is "reserved" but encoders still emit one of the four codepoints,
/// so the parser surfaces the raw decoded variant and leaves the
/// caller to honour the spec gating.
///
/// "Dolby Pro Logic IIx" is a back-compatible matrix decoder that
/// recovers a 5.1 or 6.1/7.1 program from a Dolby Surround EX-encoded
/// stream; "Dolby Pro Logic IIz" is the matrix variant that recovers a
/// front-height pair.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DolbySurroundExMode {
    /// `'00'` — encoding not indicated.
    NotIndicated,
    /// `'01'` — explicitly NOT Dolby Surround EX, Pro Logic IIx, or
    /// Pro Logic IIz encoded.
    NotEncoded,
    /// `'10'` — Dolby Surround EX or Pro Logic IIx encoded.
    SurroundExOrProLogicIIx,
    /// `'11'` — Dolby Pro Logic IIz encoded.
    ProLogicIIz,
}

impl DolbySurroundExMode {
    /// Decode the 2-bit wire value verbatim per Table D2.7.
    pub fn from_code(code: u8) -> Self {
        match code & 0x3 {
            0 => DolbySurroundExMode::NotIndicated,
            1 => DolbySurroundExMode::NotEncoded,
            2 => DolbySurroundExMode::SurroundExOrProLogicIIx,
            _ => DolbySurroundExMode::ProLogicIIz,
        }
    }

    /// Raw 2-bit code as it appeared on the wire.
    pub fn raw(self) -> u8 {
        match self {
            DolbySurroundExMode::NotIndicated => 0,
            DolbySurroundExMode::NotEncoded => 1,
            DolbySurroundExMode::SurroundExOrProLogicIIx => 2,
            DolbySurroundExMode::ProLogicIIz => 3,
        }
    }
}

/// Annex D §2.3.1.9 Dolby Headphone mode (Table D2.8).
///
/// Surfaced on [`Bsi::dheadphonmod`] when `bsid == 6` and the `xbsi2e`
/// block is present. The spec note constrains the meaningful range of
/// the field to `acmod == 2` (2/0 stereo); for other `acmod` values
/// the field is "reserved" but the parser still surfaces the raw
/// decoded variant.
///
/// The `'11'` reserved codepoint is mapped to [`DolbyHeadphoneMode::Reserved`];
/// per the spec a decoder receiving the reserved code "should still
/// reproduce audio" and is encouraged to treat it as `NotIndicated`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DolbyHeadphoneMode {
    /// `'00'` — encoding not indicated.
    NotIndicated,
    /// `'01'` — explicitly NOT Dolby Headphone encoded.
    NotEncoded,
    /// `'10'` — Dolby Headphone encoded.
    Encoded,
    /// `'11'` — reserved (treat as [`NotIndicated`](Self::NotIndicated)
    /// per §2.3.1.9; the decoder must still reproduce audio).
    Reserved,
}

impl DolbyHeadphoneMode {
    /// Decode the 2-bit wire value verbatim per Table D2.8.
    pub fn from_code(code: u8) -> Self {
        match code & 0x3 {
            0 => DolbyHeadphoneMode::NotIndicated,
            1 => DolbyHeadphoneMode::NotEncoded,
            2 => DolbyHeadphoneMode::Encoded,
            _ => DolbyHeadphoneMode::Reserved,
        }
    }

    /// Raw 2-bit code as it appeared on the wire.
    pub fn raw(self) -> u8 {
        match self {
            DolbyHeadphoneMode::NotIndicated => 0,
            DolbyHeadphoneMode::NotEncoded => 1,
            DolbyHeadphoneMode::Encoded => 2,
            DolbyHeadphoneMode::Reserved => 3,
        }
    }
}

/// Annex D §2.3.1.10 A/D converter type (Table D2.9). A single bit:
/// `'0'` indicates a generic / standard PCM A/D converter; `'1'`
/// indicates an HDCD-encoded source (HDCD packs a "hidden" 4 bits in
/// the 16-bit PCM LSBs, and downstream equipment may decode them for a
/// 20-bit dynamic range). The AC-3 decoder treats both identically.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdConverterType {
    /// `'0'` — Standard (generic 24-bit PCM).
    Standard,
    /// `'1'` — HDCD-encoded source.
    Hdcd,
}

impl AdConverterType {
    /// Decode the 1-bit wire value verbatim per Table D2.9.
    pub fn from_code(code: u8) -> Self {
        if code & 0x1 == 0 {
            AdConverterType::Standard
        } else {
            AdConverterType::Hdcd
        }
    }

    /// Raw 1-bit code as it appeared on the wire.
    pub fn raw(self) -> u8 {
        match self {
            AdConverterType::Standard => 0,
            AdConverterType::Hdcd => 1,
        }
    }
}

/// Annex D §2.3.1.3-6 alternate-syntax mix-level codewords. Each is a
/// 3-bit value; Tables D2.3 / D2.4 / D2.5 / D2.6 map them to linear
/// gains via [`annex_d_lt_rt_clev`] / [`annex_d_lt_rt_slev`] /
/// [`annex_d_lo_ro_clev`] / [`annex_d_lo_ro_slev`].
///
/// These supersede the body-spec 2-bit `cmixlev` / `surmixlev` defaults
/// for the LtRt / LoRo downmix targets specifically — the body fields
/// are still parsed (they sit ahead of the xbsi1 block in the bit
/// stream) but a §7.8 downmix on a `bsid == 6` Annex D stream should
/// prefer the Annex D refinements.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AnnexDMixLevels {
    /// `ltrtcmixlev` (Table D2.3). Defined for acmod ∈ {3, 5, 7}.
    pub ltrtcmixlev: u8,
    /// `ltrtsurmixlev` (Table D2.4). Codes 000..010 are reserved → use
    /// 0.841. Defined for acmod ∈ {4, 5, 6, 7}.
    pub ltrtsurmixlev: u8,
    /// `lorocmixlev` (Table D2.5). Defined for acmod ∈ {3, 5, 7}.
    pub lorocmixlev: u8,
    /// `lorosurmixlev` (Table D2.6). Codes 000..010 are reserved → use
    /// 0.841. Defined for acmod ∈ {4, 5, 6, 7}.
    pub lorosurmixlev: u8,
}

/// Map an Annex D 3-bit "center-channel" mix-level code to a linear
/// gain per Tables D2.3 / D2.5 (the two tables are identical).
///
/// Code → gain (dB):
///  `000` 1.414 (+3.0), `001` 1.189 (+1.5), `010` 1.000 ( 0.0),
///  `011` 0.841 (−1.5), `100` 0.707 (−3.0), `101` 0.595 (−4.5),
///  `110` 0.500 (−6.0), `111` 0.000 (−∞).
pub fn annex_d_center_mix_gain(code: u8) -> f32 {
    match code & 0x7 {
        0 => 1.414,
        1 => 1.189,
        2 => 1.000,
        3 => 0.841,
        4 => 0.707,
        5 => 0.595,
        6 => 0.500,
        _ => 0.000,
    }
}

/// Map an Annex D 3-bit "surround-channel" mix-level code to a linear
/// gain per Tables D2.4 / D2.6 (identical). Codes `000..010` are
/// reserved; per §2.3.1.4 / §2.3.1.6 the decoder shall substitute
/// 0.841 (the next defined code).
pub fn annex_d_surround_mix_gain(code: u8) -> f32 {
    match code & 0x7 {
        0..=3 => 0.841, // 000/001/010 reserved → 0.841; 011 = 0.841
        4 => 0.707,
        5 => 0.595,
        6 => 0.500,
        _ => 0.000,
    }
}

/// Parse the BSI starting at the beginning of `data`. The slice *must*
/// point at the byte immediately following `syncinfo` (i.e. byte 5 of
/// the syncframe).
///
/// On success the returned `Bsi` describes the stream and carries the
/// exact number of bits the parser consumed, so the caller can resume
/// an MSB-first `BitReader` at the right place for the first audio
/// block.
pub fn parse(data: &[u8]) -> Result<Bsi> {
    let mut br = BitReader::new(data);

    let bsid = br.read_u32(5)? as u8;
    let bsmod = br.read_u32(3)? as u8;
    let acmod = br.read_u32(3)? as u8;
    let nfchans = acmod_nfchans(acmod);

    // cmixlev — only present when there are 3 front channels, i.e.
    // the two LSBs of acmod include '1' for centre *and* acmod!=1
    // (the spec's "if ((acmod & 0x1) && (acmod != 0x1))" guard).
    let cmixlev = if (acmod & 0x1) != 0 && acmod != 0x1 {
        br.read_u32(2)? as u8
    } else {
        0xFF
    };

    // surmixlev — present when a surround channel exists (acmod & 0x4).
    let surmixlev = if (acmod & 0x4) != 0 {
        br.read_u32(2)? as u8
    } else {
        0xFF
    };

    // dsurmod — present only in 2/0 mode (acmod == 0x2).
    let dsurmod = if acmod == 0x2 {
        br.read_u32(2)? as u8
    } else {
        0xFF
    };

    let lfeon = br.read_u32(1)? != 0;
    let nchans = nfchans + u8::from(lfeon);

    let dialnorm_raw = br.read_u32(5)? as u8;
    // §5.4.2.8: dialnorm=0 is reserved; decoder shall use -31 dB.
    let dialnorm = if dialnorm_raw == 0 { 31 } else { dialnorm_raw };

    // Optional service metadata (§5.4.2.9 ff). `compr` is surfaced
    // (Table 7.30); langcode / audprodie are parse-and-discard — they
    // carry deprecated mix-level/room-type hints and ISO-639 codes
    // that the decoder doesn't act on. A proper player can tap them
    // later via a second pass if needed.
    let compre = br.read_u32(1)? != 0;
    let compr = if compre {
        Some(CompressionGain::from_byte(br.read_u32(8)? as u8))
    } else {
        None
    };
    let langcode = br.read_u32(1)? != 0;
    if langcode {
        let _langcod = br.read_u32(8)?;
    }
    let audprodie = br.read_u32(1)? != 0;
    if audprodie {
        let _mixlevel = br.read_u32(5)?;
        let _roomtyp = br.read_u32(2)?;
    }

    // 1+1 mode (dual mono) carries a second copy of the metadata for Ch2.
    let compr_ch2 = if acmod == 0 {
        let _dialnorm2 = br.read_u32(5)?;
        let compr2e = br.read_u32(1)? != 0;
        let c2 = if compr2e {
            Some(CompressionGain::from_byte(br.read_u32(8)? as u8))
        } else {
            None
        };
        let langcod2e = br.read_u32(1)? != 0;
        if langcod2e {
            let _langcod2 = br.read_u32(8)?;
        }
        let audprodi2e = br.read_u32(1)? != 0;
        if audprodi2e {
            let _mixlevel2 = br.read_u32(5)?;
            let _roomtyp2 = br.read_u32(2)?;
        }
        c2
    } else {
        None
    };

    let _copyrightb = br.read_u32(1)?;
    let _origbs = br.read_u32(1)?;

    // §5.3.2 base syntax has `timecod1e/timecod2e` here; Annex D
    // §2.3 / Table D2.1 reuses the same two 1+14-bit slots as
    // `xbsi1e/xbsi2e` and is identified by `bsid == 6` (§2.1).
    // Both shapes occupy the same fixed 30 bits maximum so the
    // surrounding parse is unchanged.
    let (annex_d_mix_levels, dmixmod, dsurexmod, dheadphonmod, adconvtyp) = if bsid == 6 {
        // Annex D xbsi1 block.
        let xbsi1e = br.read_u32(1)? != 0;
        let (mix, dmm) = if xbsi1e {
            let dmm = br.read_u32(2)? as u8;
            let ltrtc = br.read_u32(3)? as u8;
            let ltrts = br.read_u32(3)? as u8;
            let loroc = br.read_u32(3)? as u8;
            let loros = br.read_u32(3)? as u8;
            (
                Some(AnnexDMixLevels {
                    ltrtcmixlev: ltrtc,
                    ltrtsurmixlev: ltrts,
                    lorocmixlev: loroc,
                    lorosurmixlev: loros,
                }),
                dmm,
            )
        } else {
            (None, 0xFFu8)
        };
        // xbsi2 block — §2.3.1.7-12. 14 bits total: dsurexmod(2) +
        // dheadphonmod(2) + adconvtyp(1) + xbsi2(8) + encinfo(1). The
        // last two are reserved-for-future-assignment / encoder-private
        // respectively and stay discarded.
        let xbsi2e = br.read_u32(1)? != 0;
        let (dsex, dhpm, adcv) = if xbsi2e {
            let dsex_raw = br.read_u32(2)? as u8;
            let dhpm_raw = br.read_u32(2)? as u8;
            let adcv_raw = br.read_u32(1)? as u8;
            let _xbsi2 = br.read_u32(8)?;
            let _encinfo = br.read_u32(1)?;
            (
                Some(DolbySurroundExMode::from_code(dsex_raw)),
                Some(DolbyHeadphoneMode::from_code(dhpm_raw)),
                Some(AdConverterType::from_code(adcv_raw)),
            )
        } else {
            (None, None, None)
        };
        (mix, dmm, dsex, dhpm, adcv)
    } else {
        // §5.3.2 base syntax — timecod1/timecod2 (never surfaced).
        let timecod1e = br.read_u32(1)? != 0;
        if timecod1e {
            let _t1 = br.read_u32(14)?;
        }
        let timecod2e = br.read_u32(1)? != 0;
        if timecod2e {
            let _t2 = br.read_u32(14)?;
        }
        (None, 0xFFu8, None, None, None)
    };

    // addbsi — up to 64 bytes of trailing info we can safely skip.
    let addbsie = br.read_u32(1)? != 0;
    if addbsie {
        let addbsil = br.read_u32(6)?; // 0..=63, meaning 1..=64 bytes
        let nbits = (addbsil + 1) * 8;
        br.skip(nbits)?;
    }

    let bits_consumed = br.bit_position();

    if bsid > 10 {
        // Per spec, base decoders mute for bsid > 8; we accept ≤10 as
        // a small safety margin for near-compatible streams and defer
        // a hard rejection to the decoder loop so probing still
        // succeeds.
        return Err(Error::Unsupported(format!(
            "ac3: bsid {bsid} > 8 — Annex E E-AC-3 bitstream needs a separate parser"
        )));
    }

    Ok(Bsi {
        bsid,
        bsmod,
        acmod,
        nfchans,
        lfeon,
        nchans,
        dialnorm,
        cmixlev,
        surmixlev,
        dsurmod,
        annex_d_mix_levels,
        dmixmod,
        compr,
        compr_ch2,
        dsurexmod,
        dheadphonmod,
        adconvtyp,
        bits_consumed,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal BSI byte sequence for 2/0 stereo, LFE off, no
    /// optional fields. acmod=2 → surmixlev/cmixlev absent, dsurmod
    /// present.
    ///
    ///   bsid=8 (5 bits)  : 0b01000
    ///   bsmod=0 (3 bits) : 0b000
    ///   acmod=2 (3 bits) : 0b010
    ///   dsurmod=0 (2)    : 0b00
    ///   lfeon=0 (1)      : 0
    ///   dialnorm=27 (5)  : 0b11011
    ///   compre=0         : 0
    ///   langcode=0       : 0
    ///   audprodie=0      : 0
    ///   copyrightb=0     : 0
    ///   origbs=0         : 0
    ///   timecod1e=0      : 0
    ///   timecod2e=0      : 0
    ///   addbsie=0        : 0
    ///
    /// Total = 5+3+3+2+1+5+1+1+1+1+1+1+1+1 = 27 bits → 4 bytes with 5
    /// trailing pad bits.
    #[test]
    fn parses_minimal_2_0_stereo_bsi() {
        // Build via a BitWriter-style manual pack.
        let bits: [(u8, u32); 14] = [
            (5, 0b01000),
            (3, 0b000),
            (3, 0b010),
            (2, 0b00),
            (1, 0),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let mut out = vec![0u8; 8];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                out[byte] |= bit << shift;
                bitpos += 1;
            }
        }

        let b = parse(&out).unwrap();
        assert_eq!(b.bsid, 8);
        assert_eq!(b.bsmod, 0);
        assert_eq!(b.acmod, 2);
        assert_eq!(b.nfchans, 2);
        assert!(!b.lfeon);
        assert_eq!(b.nchans, 2);
        assert_eq!(b.dialnorm, 27);
        assert_eq!(b.dsurmod, 0);
        assert_eq!(b.cmixlev, 0xFF);
        assert_eq!(b.surmixlev, 0xFF);
        assert!(b.annex_d_mix_levels.is_none());
        assert_eq!(b.dmixmod, 0xFF);
        assert_eq!(b.bits_consumed, bitpos as u64);
    }

    #[test]
    fn dialnorm_zero_remaps_to_31() {
        // bsid=8, bsmod=0, acmod=1 (1/0 mono — no cmix / surmix / dsurmod),
        // lfeon=0, dialnorm=0 → should remap.
        let bits: [(u8, u32); 11] = [
            (5, 8),
            (3, 0),
            (3, 1),
            (1, 0), // lfeon
            (5, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let mut last4 = vec![0u8; 8];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                last4[byte] |= bit << shift;
                bitpos += 1;
            }
        }
        // Need addbsie + timecodes bits too — add three trailing zero bits
        // to cover (timecod1e, timecod2e, addbsie) — wait, already in list.
        // Actually this packs 5+3+3+1+5+... re-count:
        //   5+3+3+1+5+1+1+1+1+1+1 = 23 bits. Missing nothing structural?
        // For acmod=1 there's no cmix, surmix, dsurmod. After lfeon and
        // dialnorm it goes compre/langcode/audprodie/copyrightb/origbs/
        // timecod1e/timecod2e/addbsie = 8 flags but we have 6. Add two
        // more zero bits so the addbsie fires false.
        let bits2: [(u8, u32); 2] = [(1, 0), (1, 0)];
        for (n, v) in bits2.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                last4[byte] |= bit << shift;
                bitpos += 1;
            }
        }

        let b = parse(&last4).unwrap();
        assert_eq!(b.dialnorm, 31);
        assert_eq!(b.nfchans, 1);
        assert_eq!(b.nchans, 1);
    }

    /// Annex D §2 / Table D2.1 — `bsid == 6` activates the alternate
    /// syntax: the body's `timecod1e/timecod2e` slots become
    /// `xbsi1e/xbsi2e`. Verify the xbsi1 mix-level fields surface on
    /// [`Bsi::annex_d_mix_levels`] / [`Bsi::dmixmod`].
    #[test]
    fn parses_annex_d_bsid_6_xbsi1_mix_levels() {
        // 3/2 (acmod=7), lfe on. cmixlev = 0b00 (0.707), surmixlev = 0b00
        // (0.707). dialnorm=27. No compre / langcode / audprodie /
        // copyrightb / origbs. xbsi1e = 1 with:
        //   dmixmod        = 0b01 (LtRt preferred)
        //   ltrtcmixlev    = 0b011 (0.841)
        //   ltrtsurmixlev  = 0b100 (0.707)
        //   lorocmixlev    = 0b100 (0.707)
        //   lorosurmixlev  = 0b101 (0.595)
        // xbsi2e = 0. addbsie = 0.
        //
        // Bit layout:
        //   bsid=6 (5)         00110
        //   bsmod=0 (3)        000
        //   acmod=7 (3)        111
        //   cmixlev (2)        00
        //   surmixlev (2)      00
        //   lfeon (1)          1
        //   dialnorm (5)       11011
        //   compre (1)         0
        //   langcode (1)       0
        //   audprodie (1)      0
        //   copyrightb (1)     0
        //   origbs (1)         0
        //   xbsi1e (1)         1
        //   dmixmod (2)        01
        //   ltrtcmixlev (3)    011
        //   ltrtsurmixlev (3)  100
        //   lorocmixlev (3)    100
        //   lorosurmixlev (3)  101
        //   xbsi2e (1)         0
        //   addbsie (1)        0
        let bits: &[(u8, u32)] = &[
            (5, 6),
            (3, 0),
            (3, 7),
            (2, 0),
            (2, 0),
            (1, 1),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 1),
            (2, 0b01),
            (3, 0b011),
            (3, 0b100),
            (3, 0b100),
            (3, 0b101),
            (1, 0),
            (1, 0),
        ];
        let mut out = vec![0u8; 8];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                out[byte] |= bit << shift;
                bitpos += 1;
            }
        }

        let b = parse(&out).unwrap();
        assert_eq!(b.bsid, 6);
        assert_eq!(b.acmod, 7);
        assert!(b.lfeon);
        assert_eq!(b.dmixmod, 0b01);
        let mix = b.annex_d_mix_levels.expect("xbsi1 set → mix levels");
        assert_eq!(mix.ltrtcmixlev, 0b011);
        assert_eq!(mix.ltrtsurmixlev, 0b100);
        assert_eq!(mix.lorocmixlev, 0b100);
        assert_eq!(mix.lorosurmixlev, 0b101);
        assert_eq!(b.bits_consumed, bitpos as u64);
    }

    /// `bsid == 6` with `xbsi1e == 0` should leave the mix-level
    /// payload absent. The xbsi2e slot still needs to be consumed.
    #[test]
    fn parses_annex_d_bsid_6_no_xbsi1() {
        // 2/0 (acmod=2), no LFE. cmixlev absent (acmod & 1 == 0).
        // surmixlev absent (acmod & 4 == 0). dsurmod=0 (2 bits).
        // dialnorm=20. xbsi1e=0. xbsi2e=0. addbsie=0.
        let bits: &[(u8, u32)] = &[
            (5, 6),
            (3, 0),
            (3, 2),
            (2, 0),  // dsurmod
            (1, 0),  // lfeon
            (5, 20), // dialnorm
            (1, 0),  // compre
            (1, 0),  // langcode
            (1, 0),  // audprodie
            (1, 0),  // copyrightb
            (1, 0),  // origbs
            (1, 0),  // xbsi1e
            (1, 0),  // xbsi2e
            (1, 0),  // addbsie
        ];
        let mut out = vec![0u8; 8];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                out[byte] |= bit << shift;
                bitpos += 1;
            }
        }

        let b = parse(&out).unwrap();
        assert_eq!(b.bsid, 6);
        assert!(b.annex_d_mix_levels.is_none());
        assert_eq!(b.dmixmod, 0xFF);
        assert_eq!(b.bits_consumed, bitpos as u64);
    }

    /// Table D2.3 / D2.5 — the 3-bit center mix-level codewords map to
    /// the exact gains the spec tabulates.
    #[test]
    fn annex_d_center_mix_gain_matches_table_d2_3() {
        let expected: [(u8, f32); 8] = [
            (0b000, 1.414),
            (0b001, 1.189),
            (0b010, 1.000),
            (0b011, 0.841),
            (0b100, 0.707),
            (0b101, 0.595),
            (0b110, 0.500),
            (0b111, 0.000),
        ];
        for (code, gain) in expected {
            assert!(
                (annex_d_center_mix_gain(code) - gain).abs() < 1e-6,
                "code 0b{code:03b}: want {gain}, got {}",
                annex_d_center_mix_gain(code)
            );
        }
    }

    /// Table D2.4 / D2.6 — the 3-bit surround mix-level codewords. The
    /// reserved codes `000/001/010` substitute 0.841 per spec.
    #[test]
    fn annex_d_surround_mix_gain_substitutes_reserved_with_0_841() {
        // Reserved codes all map to 0.841.
        for code in 0u8..=2 {
            let g = annex_d_surround_mix_gain(code);
            assert!(
                (g - 0.841).abs() < 1e-6,
                "reserved code 0b{code:03b} should resolve to 0.841, got {g}"
            );
        }
        let expected: [(u8, f32); 5] = [
            (0b011, 0.841),
            (0b100, 0.707),
            (0b101, 0.595),
            (0b110, 0.500),
            (0b111, 0.000),
        ];
        for (code, gain) in expected {
            assert!(
                (annex_d_surround_mix_gain(code) - gain).abs() < 1e-6,
                "code 0b{code:03b}: want {gain}, got {}",
                annex_d_surround_mix_gain(code)
            );
        }
    }

    /// Table 5.7 — every `bsmod` codepoint except `0b111` resolves to a
    /// fixed service type independent of `acmod`. Spot-check each row
    /// with a couple of `acmod` values to confirm the resolver doesn't
    /// peek at `acmod` when `bsmod != 0b111`.
    #[test]
    fn bsmod_table_5_7_fixed_codepoints() {
        use BitStreamMode::*;
        let rows: [(u8, BitStreamMode); 7] = [
            (0b000, CompleteMain),
            (0b001, MusicAndEffects),
            (0b010, VisuallyImpaired),
            (0b011, HearingImpaired),
            (0b100, Dialogue),
            (0b101, Commentary),
            (0b110, Emergency),
        ];
        for (bsmod, want) in rows {
            for acmod in 0u8..=7 {
                let got = BitStreamMode::from_bsmod_acmod(bsmod, acmod);
                assert_eq!(
                    got, want,
                    "bsmod=0b{bsmod:03b} acmod=0b{acmod:03b}: want {want:?}, got {got:?}"
                );
            }
        }
    }

    /// Table 5.7 — `bsmod==0b111` is overloaded: acmod=0b001 → VoiceOver,
    /// acmod ∈ {0b010..=0b111} → Karaoke, acmod=0b000 (the 1+1 dual-mono
    /// slot) → Reserved (no Table 5.7 row defines it).
    #[test]
    fn bsmod_0b111_resolves_with_acmod() {
        assert_eq!(
            BitStreamMode::from_bsmod_acmod(0b111, 0b000),
            BitStreamMode::Reserved
        );
        assert_eq!(
            BitStreamMode::from_bsmod_acmod(0b111, 0b001),
            BitStreamMode::VoiceOver
        );
        for acmod in 0b010u8..=0b111 {
            assert_eq!(
                BitStreamMode::from_bsmod_acmod(0b111, acmod),
                BitStreamMode::Karaoke,
                "acmod=0b{acmod:03b}"
            );
        }
    }

    /// `is_main` / `is_associated` partition Table 5.7 cleanly. CM, ME,
    /// and karaoke are main; VI/HI/D/C/E/VO are associated; the unused
    /// `bsmod=0b111 acmod=0b000` cell is neither.
    #[test]
    fn main_vs_associated_partition() {
        use BitStreamMode::*;
        let main = [CompleteMain, MusicAndEffects, Karaoke];
        let assoc = [
            VisuallyImpaired,
            HearingImpaired,
            Dialogue,
            Commentary,
            Emergency,
            VoiceOver,
        ];
        for m in main {
            assert!(m.is_main(), "{m:?} should be main");
            assert!(!m.is_associated(), "{m:?} should not be associated");
        }
        for a in assoc {
            assert!(a.is_associated(), "{a:?} should be associated");
            assert!(!a.is_main(), "{a:?} should not be main");
        }
        assert!(!Reserved.is_main());
        assert!(!Reserved.is_associated());
    }

    /// Mnemonics are stable per Table 5.7 — used in CLI / log output.
    /// "?" is reserved for the Reserved case so downstream code can
    /// rely on a single sentinel for "no service type".
    #[test]
    fn mnemonics_are_table_5_7_short_forms() {
        use BitStreamMode::*;
        let rows: [(BitStreamMode, &str); 10] = [
            (CompleteMain, "CM"),
            (MusicAndEffects, "ME"),
            (VisuallyImpaired, "VI"),
            (HearingImpaired, "HI"),
            (Dialogue, "D"),
            (Commentary, "C"),
            (Emergency, "E"),
            (VoiceOver, "VO"),
            (Karaoke, "K"),
            (Reserved, "?"),
        ];
        for (mode, mnem) in rows {
            assert_eq!(mode.mnemonic(), mnem, "{mode:?}");
        }
    }

    /// `Bsi::service_type()` round-trips the raw bsmod/acmod into the
    /// typed enum. Reuses the minimal 2/0 stereo fixture (acmod=2,
    /// bsmod=0) and a custom 1/0 mono bsmod=0b111 builder to cover
    /// both the simple and overloaded branches end-to-end through the
    /// `Bsi` accessor.
    #[test]
    fn bsi_service_type_accessor_routes_through_table_5_7() {
        // The minimal 2/0 stereo fixture sets bsmod=0, acmod=2 →
        // CompleteMain. Re-built locally so the test stays
        // self-contained.
        let stereo_bits: [(u8, u32); 14] = [
            (5, 0b01000),
            (3, 0b000),
            (3, 0b010),
            (2, 0b00),
            (1, 0),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let stereo_bytes = pack_bits(&stereo_bits);
        let bsi = parse(&stereo_bytes).expect("parse minimal 2/0");
        assert_eq!(bsi.bsmod, 0b000);
        assert_eq!(bsi.acmod, 0b010);
        assert_eq!(bsi.service_type(), BitStreamMode::CompleteMain);

        // 1/0 mono BSI with bsmod=0b111 + acmod=0b001 → VoiceOver.
        // acmod=1 means no cmix / no surmix / no dsurmod optional fields.
        //   bsid=8       (5)  : 0b01000
        //   bsmod=0b111  (3)  : 0b111
        //   acmod=0b001  (3)  : 0b001
        //   lfeon=0      (1)  : 0
        //   dialnorm=27  (5)  : 0b11011
        //   compre=0     (1)  : 0
        //   langcode=0   (1)  : 0
        //   audprodie=0  (1)  : 0
        //   copyrightb=0 (1)  : 0
        //   origbs=0     (1)  : 0
        //   timecod1e=0  (1)  : 0
        //   timecod2e=0  (1)  : 0
        //   addbsie=0    (1)  : 0
        let voiceover_bits: [(u8, u32); 13] = [
            (5, 0b01000),
            (3, 0b111),
            (3, 0b001),
            (1, 0),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let voiceover_bytes = pack_bits(&voiceover_bits);
        let bsi = parse(&voiceover_bytes).expect("parse 1/0 voiceover");
        assert_eq!(bsi.bsmod, 0b111);
        assert_eq!(bsi.acmod, 0b001);
        assert_eq!(bsi.service_type(), BitStreamMode::VoiceOver);
    }

    /// MSB-first bit packer matching the AC-3 `BitReader` order — used
    /// by the Table 5.7 service-type tests to build synthetic BSIs.
    fn pack_bits(bits: &[(u8, u32)]) -> Vec<u8> {
        let total_bits: usize = bits.iter().map(|(n, _)| *n as usize).sum();
        let mut out = vec![0u8; total_bits.div_ceil(8) + 1];
        let mut bitpos = 0usize;
        for (n, v) in bits.iter().copied() {
            for i in (0..n).rev() {
                let bit = ((v >> i) & 1) as u8;
                let byte = bitpos / 8;
                let shift = 7 - (bitpos % 8);
                out[byte] |= bit << shift;
                bitpos += 1;
            }
        }
        out
    }

    // ---------------------------------------------------------------
    // Heavy compression gain (`compr`) — Table 7.30 / §7.7.2.2.
    // ---------------------------------------------------------------

    /// X is a 4-bit signed integer with values in `-8..=+7`. Walk every
    /// `X` codepoint with `Y = 0b1111` (max-Y) and assert the decoded
    /// `(X, Y)` round-trip matches the bit layout described in §7.7.2.2.
    #[test]
    fn compression_gain_x_field_sign_extends_correctly() {
        // (raw_x_nibble, expected signed value) — every codepoint.
        let cases = [
            (0b0000u8, 0i8),
            (0b0001, 1),
            (0b0010, 2),
            (0b0011, 3),
            (0b0100, 4),
            (0b0101, 5),
            (0b0110, 6),
            (0b0111, 7),
            (0b1000, -8),
            (0b1001, -7),
            (0b1010, -6),
            (0b1011, -5),
            (0b1100, -4),
            (0b1101, -3),
            (0b1110, -2),
            (0b1111, -1),
        ];
        for (xn, x) in cases {
            // Y = 0b1010 (arbitrary) — verify X decoding is independent of Y.
            let cg = CompressionGain::from_byte((xn << 4) | 0b1010);
            assert_eq!(cg.x(), x, "X mismatch for raw nibble {xn:#06b}");
            assert_eq!(cg.y(), 0b1010);
            assert_eq!(cg.raw(), (xn << 4) | 0b1010);
        }
    }

    /// Table 7.30 row checks: the dB gain of each `(X, Y=0)` codepoint
    /// must match the table's "Gain Indicated" column to within 0.005 dB.
    /// At `Y=0`, the contribution from `Y` is exactly `-6.02 dB`, so the
    /// table's "X alone = (X+1)*6.02 dB" sums with the Y attenuation to
    /// `linear = 2^(X+1) * 0.5`, i.e. `(X+1)*6.02 - 6.02 = X*6.02 dB`.
    /// Therefore the dB at `Y=0` equals `X * 6.02` (Table 7.30 minus
    /// 6.02 dB across the board).
    ///
    /// Equivalently the table's headline rows (e.g. `X=7 → +48.16 dB`)
    /// describe the X contribution **without** the Y attenuation; the
    /// effective decoder gain when `Y = 0b1111` (`(16+15)/32 = 31/32 ≈
    /// -0.28 dB`) drops the headline by 0.276 dB. This test checks both
    /// the headline (max-Y) and the bottom (Y=0) of every X row.
    #[test]
    fn compression_gain_table_7_30_db_endpoints() {
        // (X, Y=15 dB ≈ headline - 0.276; Y=0 dB = headline - 6.02).
        let cases = [
            (7i8, 48.16f32),
            (6, 42.14),
            (5, 36.12),
            (4, 30.10),
            (3, 24.08),
            (2, 18.06),
            (1, 12.04),
            (0, 6.02),
            (-1, 0.0),
            (-2, -6.02),
            (-3, -12.04),
            (-4, -18.06),
            (-5, -24.08),
            (-6, -30.10),
            (-7, -36.12),
            (-8, -42.14),
        ];
        for (x, headline_db) in cases {
            // Pack X into the upper nibble (two's-complement 4-bit).
            let xn = (x as i16 & 0xF) as u8;
            // Y = 0b1111 → top of row, dB ≈ headline - 0.276.
            let max_y = CompressionGain::from_byte((xn << 4) | 0b1111);
            let max_y_db = max_y.decibels();
            assert!(
                (max_y_db - (headline_db - 0.276)).abs() < 0.01,
                "X={x} Y=15: got {max_y_db:.3} dB, want {:.3} dB",
                headline_db - 0.276
            );
            // Y = 0b0000 → bottom of row, dB = headline - 6.02.
            let min_y = CompressionGain::from_byte(xn << 4);
            let min_y_db = min_y.decibels();
            assert!(
                (min_y_db - (headline_db - 6.02)).abs() < 0.01,
                "X={x} Y=0: got {min_y_db:.3} dB, want {:.3} dB",
                headline_db - 6.02
            );
        }
    }

    /// Y is a 4-bit unsigned mantissa with an implicit leading 1, read
    /// as `(16 + Y) / 32`. Spot-check the four boundary values per
    /// §7.7.2.2 ("Y can represent values between 0.111112 (or 31/32) and
    /// 0.100002 (or 1/2)").
    #[test]
    fn compression_gain_y_field_is_fractional_with_leading_one() {
        // With X = -1 (= 0b1111, gain = 0 dB), linear = 1.0 * (16+Y)/32.
        let cases = [
            (0u8, 16.0 / 32.0), // 0.5
            (1, 17.0 / 32.0),   // 0.53125
            (15, 31.0 / 32.0),  // 0.96875
            (8, 24.0 / 32.0),   // 0.75
        ];
        for (y, expected) in cases {
            let cg = CompressionGain::from_byte(0b1111_0000 | y);
            let lin = cg.linear();
            assert!(
                (lin - expected).abs() < 1e-6,
                "X=-1 Y={y}: got linear={lin}, want {expected}"
            );
        }
    }

    /// Combined-range sanity per §7.7.2.2:
    /// "The combination of X and Y values allows compr to indicate gain
    /// changes from 48.16 – 0.28 = +47.89 dB, to –42.14 – 6.02 =
    /// –48.16 dB."
    #[test]
    fn compression_gain_extreme_codepoints_match_spec_range() {
        let top = CompressionGain::from_byte(0b0111_1111); // X=7, Y=15
        let bottom = CompressionGain::from_byte(0b1000_0000); // X=-8, Y=0

        assert_eq!(top.x(), 7);
        assert_eq!(top.y(), 15);
        // Linear = 2^8 * 31/32 = 248.
        assert!((top.linear() - 248.0).abs() < 1e-3);
        // dB = 20*log10(248) ≈ +47.884 dB.
        assert!((top.decibels() - 47.884).abs() < 0.01);

        assert_eq!(bottom.x(), -8);
        assert_eq!(bottom.y(), 0);
        // Linear = 2^-7 * 0.5 = 1/256.
        assert!((bottom.linear() - 1.0 / 256.0).abs() < 1e-6);
        // dB = 20*log10(1/256) ≈ -48.165 dB.
        assert!((bottom.decibels() - (-48.165)).abs() < 0.01);
    }

    /// `parse()` surfaces `compr` as `Some(CompressionGain)` when the
    /// `compre` flag is set, and `None` otherwise. Build a 1/0 mono
    /// BSI with `compre=1` and `compr=0b0001_0000` (X=1, Y=0, linear
    /// `2^2 * 0.5 = 2.0`, ≈ +6.02 dB), then verify the parser routes
    /// the byte verbatim into the typed surface.
    #[test]
    fn parse_surfaces_compr_when_compre_set() {
        // 1/0 mono (acmod=1) → no cmixlev / surmixlev / dsurmod.
        // bsid=8, bsmod=0, acmod=1, lfeon=0, dialnorm=27,
        //   compre=1, compr=0b0001_0000, langcode=0, audprodie=0,
        //   copyrightb=0, origbs=0, timecod1e=0, timecod2e=0,
        //   addbsie=0.
        let bits: [(u8, u32); 13] = [
            (5, 8),
            (3, 0),
            (3, 1),
            (1, 0),  // lfeon
            (5, 27), // dialnorm
            (1, 1),  // compre
            (8, 0b0001_0000),
            (1, 0), // langcode
            (1, 0), // audprodie
            (1, 0), // copyrightb
            (1, 0), // origbs
            (1, 0), // timecod1e
            (1, 0), // timecod2e + addbsie folded as separate bits below
        ];
        let mut bytes = pack_bits(&bits);
        // Append one more zero bit for addbsie.
        bytes.push(0);
        let bsi = parse(&bytes).unwrap();
        assert_eq!(bsi.acmod, 1);
        let cg = bsi.compr.expect("compre=1 should surface compr");
        assert_eq!(cg.raw(), 0b0001_0000);
        assert_eq!(cg.x(), 1);
        assert_eq!(cg.y(), 0);
        assert!((cg.linear() - 2.0).abs() < 1e-6);
        // 1+1 mode is acmod==0; for acmod==1 the Ch2 word stays None.
        assert!(bsi.compr_ch2.is_none());
    }

    /// `parse()` leaves `compr` as `None` when `compre == 0`.
    #[test]
    fn parse_leaves_compr_none_when_compre_clear() {
        // Reuse the minimal 2/0 BSI from `parses_minimal_2_0_stereo_bsi`
        // — it has compre=0 by construction.
        let bits: [(u8, u32); 14] = [
            (5, 0b01000),
            (3, 0b000),
            (3, 0b010),
            (2, 0b00),
            (1, 0),
            (5, 27),
            (1, 0), // compre
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let bytes = pack_bits(&bits);
        let bsi = parse(&bytes).unwrap();
        assert!(bsi.compr.is_none());
        assert!(bsi.compr_ch2.is_none());
    }

    /// 1+1 dual-mono (`acmod == 0`) carries a second `compr2` word for
    /// Ch2 with identical Table 7.30 semantics per §5.4.2.18 ("This
    /// 8-bit word has the same meaning as compr, except that it applies
    /// to the second audio channel"). Build a 1+1 BSI with `compre=1`
    /// (X=-1, Y=15 ≈ -0.276 dB on Ch1) and `compr2e=1` (X=-8, Y=0 ≈
    /// -48.16 dB on Ch2) and verify both surface independently.
    #[test]
    fn parse_surfaces_compr_ch2_in_dual_mono() {
        // acmod=0 (1+1 dual mono): no cmix/surmix/dsurmod, lfeon possible.
        //   bsid=8, bsmod=0, acmod=0, lfeon=0, dialnorm=27,
        //     compre=1, compr=0b1111_1111,
        //     langcode=0, audprodie=0,
        //   /* 1+1 second block */
        //     dialnorm2=27, compr2e=1, compr2=0b1000_0000,
        //     langcod2e=0, audprodi2e=0,
        //   copyrightb=0, origbs=0, timecod1e=0, timecod2e=0, addbsie=0.
        let bits: [(u8, u32); 18] = [
            (5, 8),
            (3, 0),
            (3, 0),
            (1, 0),  // lfeon
            (5, 27), // dialnorm
            (1, 1),  // compre
            (8, 0b1111_1111),
            (1, 0), // langcode
            (1, 0), // audprodie
            (5, 27),
            (1, 1), // compr2e
            (8, 0b1000_0000),
            (1, 0), // langcod2e
            (1, 0), // audprodi2e
            (1, 0), // copyrightb
            (1, 0), // origbs
            (1, 0), // timecod1e
            (1, 0), // timecod2e
        ];
        let mut bytes = pack_bits(&bits);
        bytes.push(0); // addbsie + pad
        let bsi = parse(&bytes).unwrap();
        assert_eq!(bsi.acmod, 0);
        let c1 = bsi.compr.expect("compre=1");
        assert_eq!(c1.raw(), 0b1111_1111);
        assert!((c1.decibels() - (-0.276)).abs() < 0.01);
        let c2 = bsi.compr_ch2.expect("compr2e=1");
        assert_eq!(c2.raw(), 0b1000_0000);
        assert!((c2.decibels() - (-48.165)).abs() < 0.01);
    }

    // ---------------------------------------------------------------
    // Annex D §2.3.1.7-10 — xbsi2 informational metadata.
    // ---------------------------------------------------------------

    /// Table D2.7 — `dsurexmod` decodes verbatim across all 4 codepoints.
    #[test]
    fn dsurexmod_decodes_all_4_codepoints() {
        use DolbySurroundExMode::*;
        assert_eq!(DolbySurroundExMode::from_code(0b00), NotIndicated);
        assert_eq!(DolbySurroundExMode::from_code(0b01), NotEncoded);
        assert_eq!(
            DolbySurroundExMode::from_code(0b10),
            SurroundExOrProLogicIIx
        );
        assert_eq!(DolbySurroundExMode::from_code(0b11), ProLogicIIz);
        // raw() round-trip.
        for code in 0u8..4 {
            assert_eq!(DolbySurroundExMode::from_code(code).raw(), code);
        }
    }

    /// Table D2.8 — `dheadphonmod` decodes verbatim. The `'11'`
    /// codepoint is `Reserved`; the spec instructs decoders to keep
    /// reproducing audio when it appears.
    #[test]
    fn dheadphonmod_decodes_all_4_codepoints() {
        use DolbyHeadphoneMode::*;
        assert_eq!(DolbyHeadphoneMode::from_code(0b00), NotIndicated);
        assert_eq!(DolbyHeadphoneMode::from_code(0b01), NotEncoded);
        assert_eq!(DolbyHeadphoneMode::from_code(0b10), Encoded);
        assert_eq!(DolbyHeadphoneMode::from_code(0b11), Reserved);
        for code in 0u8..4 {
            assert_eq!(DolbyHeadphoneMode::from_code(code).raw(), code);
        }
    }

    /// Table D2.9 — `adconvtyp` is a single bit (`Standard` vs `Hdcd`).
    #[test]
    fn adconvtyp_decodes_both_codepoints() {
        assert_eq!(AdConverterType::from_code(0), AdConverterType::Standard);
        assert_eq!(AdConverterType::from_code(1), AdConverterType::Hdcd);
        // Defensive — `from_code` masks the low bit.
        assert_eq!(AdConverterType::from_code(2), AdConverterType::Standard);
        assert_eq!(AdConverterType::from_code(3), AdConverterType::Hdcd);
        assert_eq!(AdConverterType::Standard.raw(), 0);
        assert_eq!(AdConverterType::Hdcd.raw(), 1);
    }

    /// Annex D §2.3.1.7 — `bsid == 6` with `xbsi2e == 1` surfaces the
    /// three typed playback hints on the parsed [`Bsi`]. Build a 3/2
    /// frame (acmod=7) with `xbsi1e == 0` (mix-level extensions
    /// absent), `xbsi2e == 1`, and Table D2.7 / D2.8 / D2.9 codepoints
    /// `(0b10, 0b00, 0b1)` — Dolby Surround EX on, headphone hint not
    /// indicated, HDCD source. The body `xbsi2(8)` + `encinfo(1)`
    /// reserved fields are populated with non-zero bits to verify the
    /// parser skips them but still surfaces the three typed fields.
    #[test]
    fn parse_surfaces_xbsi2_dsurexmod_dheadphonmod_adconvtyp() {
        // bsid=6 (5), bsmod=0 (3), acmod=7 (3), cmixlev=0 (2),
        // surmixlev=0 (2), lfeon=0 (1), dialnorm=27 (5),
        //   compre=0, langcode=0, audprodie=0, copyrightb=0, origbs=0,
        //   xbsi1e=0,
        //   xbsi2e=1, dsurexmod=0b10 (Surround EX / PLIIx),
        //              dheadphonmod=0b00 (NotIndicated),
        //              adconvtyp=0b1 (Hdcd),
        //              xbsi2=0b1010_1010 (reserved garbage — must be
        //                                 parsed-and-discarded),
        //              encinfo=0b1,
        //   addbsie=0.
        let bits: [(u8, u32); 20] = [
            (5, 6),
            (3, 0),
            (3, 7),
            (2, 0),
            (2, 0),
            (1, 0),           // lfeon
            (5, 27),          // dialnorm
            (1, 0),           // compre
            (1, 0),           // langcode
            (1, 0),           // audprodie
            (1, 0),           // copyrightb
            (1, 0),           // origbs
            (1, 0),           // xbsi1e=0
            (1, 1),           // xbsi2e=1
            (2, 0b10),        // dsurexmod = Surround EX / PLIIx
            (2, 0b00),        // dheadphonmod = NotIndicated
            (1, 0b1),         // adconvtyp = HDCD
            (8, 0b1010_1010), // xbsi2 (reserved garbage)
            (1, 0b1),         // encinfo (encoder-private)
            (1, 0),           // addbsie
        ];
        let bytes = pack_bits(&bits);
        let b = parse(&bytes).unwrap();
        assert_eq!(b.bsid, 6);
        assert_eq!(b.acmod, 7);
        assert!(b.annex_d_mix_levels.is_none());
        assert_eq!(
            b.dsurexmod,
            Some(DolbySurroundExMode::SurroundExOrProLogicIIx)
        );
        assert_eq!(b.dheadphonmod, Some(DolbyHeadphoneMode::NotIndicated));
        assert_eq!(b.adconvtyp, Some(AdConverterType::Hdcd));
    }

    /// `bsid != 6` falls through the §5.3.2 base syntax — the
    /// `xbsi2e` block doesn't exist, so the three Annex D fields stay
    /// `None`. Use the round-202 `parses_minimal_2_0_stereo_bsi`
    /// fixture (bsid=8, 2/0 stereo) and just assert the new fields.
    #[test]
    fn parse_leaves_xbsi2_fields_none_outside_bsid_6() {
        // Identical layout to `parses_minimal_2_0_stereo_bsi`.
        let bits: [(u8, u32); 14] = [
            (5, 0b01000),
            (3, 0b000),
            (3, 0b010),
            (2, 0b00),
            (1, 0),
            (5, 27),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
        ];
        let bytes = pack_bits(&bits);
        let b = parse(&bytes).unwrap();
        assert_eq!(b.bsid, 8);
        assert!(b.dsurexmod.is_none());
        assert!(b.dheadphonmod.is_none());
        assert!(b.adconvtyp.is_none());
    }

    /// `bsid == 6` with `xbsi2e == 0` keeps the three Annex D fields at
    /// `None` even though the alternate syntax is active — the encoder
    /// chose to omit the playback metadata. The xbsi1 block is also
    /// disabled here to keep the bit string short.
    #[test]
    fn parse_leaves_xbsi2_fields_none_when_xbsi2e_zero() {
        // bsid=6, acmod=2 (2/0 stereo): no cmix, surmixlev=0xFF guard,
        // dsurmod present. Skip xbsi1e/xbsi2e/addbsie.
        let bits: [(u8, u32); 16] = [
            (5, 6),
            (3, 0),
            (3, 2),
            (2, 0),  // dsurmod
            (1, 0),  // lfeon
            (5, 27), // dialnorm
            (1, 0),  // compre
            (1, 0),  // langcode
            (1, 0),  // audprodie
            (1, 0),  // copyrightb
            (1, 0),  // origbs
            (1, 0),  // xbsi1e=0
            (1, 0),  // xbsi2e=0
            (1, 0),  // addbsie=0
            (1, 0),  // pad
            (1, 0),  // pad
        ];
        let bytes = pack_bits(&bits);
        let b = parse(&bytes).unwrap();
        assert_eq!(b.bsid, 6);
        assert!(b.dsurexmod.is_none());
        assert!(b.dheadphonmod.is_none());
        assert!(b.adconvtyp.is_none());
    }
}
