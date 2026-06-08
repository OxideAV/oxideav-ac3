//! AC-3 synchronization frame header — `syncinfo` (§5.3.1 / §5.4.1).
//!
//! The syncinfo is the first field of every AC-3 syncframe. It is
//! exactly **5 bytes** of fixed-width fields (no variable bits):
//!
//! ```text
//!   syncword     16 bits  (always 0x0B77, transmission is MSB-first)
//!   crc1         16 bits  (CRC over the first 5/8 of the syncframe)
//!   fscod         2 bits  (sample rate: Table 5.6)
//!   frmsizecod    6 bits  (frame length: Table 5.18)
//! ```
//!
//! `parse` does zero CRC validation — the spec allows either crc1 or
//! crc2 (or neither) to be checked; we defer that to an optional
//! verification pass once the whole frame is buffered.

use oxideav_core::{Error, Result};

use crate::tables::{frame_length_bytes, sample_rate_hz};

/// The 16-bit AC-3 syncword (§5.4.1.1).
pub const SYNCWORD: u16 = 0x0B77;

/// A fully-parsed syncinfo header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SyncInfo {
    pub crc1: u16,
    pub fscod: u8,
    pub frmsizecod: u8,
    /// Sample rate in Hz as derived from `fscod` (Table 5.6).
    pub sample_rate: u32,
    /// Full frame length in **bytes** (including syncinfo itself, BSI,
    /// all 6 audio blocks, auxdata, and crc2). Derived from
    /// (fscod, frmsizecod) via Table 5.18.
    pub frame_length: u32,
}

impl SyncInfo {
    /// Typed view over [`Self::fscod`] per §5.4.1.3 / Table 5.6. The
    /// returned [`SampleRateCode`] decodes the 2-bit `fscod` field
    /// into one of the three valid sampling-rate codepoints
    /// ([`SampleRateCode::FortyEightKHz`] /
    /// [`SampleRateCode::FortyFourPointOneKHz`] /
    /// [`SampleRateCode::ThirtyTwoKHz`]) or
    /// [`SampleRateCode::Reserved`] for the spec-reserved `'11'`
    /// codepoint (per §5.4.1.3 the decoder must mute on receipt).
    ///
    /// [`parse`] already rejects the reserved codepoint at frame
    /// boundary — a [`SyncInfo`] handed back from [`parse`] therefore
    /// always reports a non-reserved [`SampleRateCode`]. The accessor
    /// remains a thin wrapper over [`Self::fscod`] for chain consumers
    /// that construct a [`SyncInfo`] by hand (e.g. resynthesising one
    /// from container-stored metadata) and want the typed surface
    /// without re-walking Table 5.6.
    pub fn sample_rate_code(&self) -> SampleRateCode {
        SampleRateCode::from_code(self.fscod)
    }
}

/// §5.4.1.3 sample-rate code (Table 5.6). A 2-bit codeword carried in
/// every AC-3 syncframe that selects one of the three supported
/// sampling rates — 48 kHz, 44.1 kHz, or 32 kHz — or the reserved
/// `'11'` codepoint that mandates a decoder mute.
///
/// Surfaced via [`SyncInfo::sample_rate_code`]. Callers that just
/// want the rate in Hz can keep reading the pre-resolved
/// [`SyncInfo::sample_rate`] field; the typed enum is for chain
/// consumers that branch on the codepoint itself (e.g. a §7.15
/// hearing-threshold table lookup that indexes into the per-`fscod`
/// row, or a §7.2.2.5 masking-curve evaluator that needs to flag a
/// reserved `fscod` separately from an out-of-range `dbpbcod`).
///
/// Per §5.4.1.3: "If the reserved code is indicated, the decoder
/// should not attempt to decode audio and should mute." [`parse`]
/// enforces that rule at frame boundary by returning
/// [`oxideav_core::Error::Invalid`] for the `'11'` codepoint, so a
/// [`SyncInfo`] obtained from [`parse`] never reports
/// [`Self::Reserved`]; the variant is preserved so a chain consumer
/// constructing a [`SyncInfo`] from container-stored metadata (where
/// the upstream demuxer may not have validated `fscod`) can detect
/// the reserved code without re-walking Table 5.6.
///
/// Annex E (E-AC-3) overloads the `'11'` codepoint as a reduced-rate
/// indicator that triggers a follow-on `fscod2` codeword (§E.2.3.1.4
/// / §E.2.3.1.5), so this enum's [`Self::Reserved`] variant
/// corresponds to the **base AC-3** decoder-mute semantics only. The
/// Annex E `bsi` carries its own `fscod` + `fscod2` raw pair and the
/// reduced-rate paths are surfaced via the E-AC-3 BSI; this base-AC-3
/// enum is not mirrored on the Annex E `Bsi`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SampleRateCode {
    /// `'00'` — 48 kHz.
    FortyEightKHz,
    /// `'01'` — 44.1 kHz.
    FortyFourPointOneKHz,
    /// `'10'` — 32 kHz.
    ThirtyTwoKHz,
    /// `'11'` — reserved. Per §5.4.1.3 the decoder "should not attempt
    /// to decode audio and should mute" on receipt; [`parse`] rejects
    /// this codepoint at frame boundary so a [`SyncInfo`] obtained
    /// from [`parse`] never carries it.
    Reserved,
}

impl SampleRateCode {
    /// Decode the 2-bit wire value verbatim per Table 5.6. Only the
    /// low 2 bits of `code` are consulted; the upper bits are
    /// ignored so a caller that passes a full `bsid`-style byte does
    /// not need to mask first.
    pub fn from_code(code: u8) -> Self {
        match code & 0x3 {
            0 => SampleRateCode::FortyEightKHz,
            1 => SampleRateCode::FortyFourPointOneKHz,
            2 => SampleRateCode::ThirtyTwoKHz,
            _ => SampleRateCode::Reserved,
        }
    }

    /// Raw 2-bit code as it appeared on the wire — the round-trip
    /// inverse of [`Self::from_code`].
    pub fn raw(self) -> u8 {
        match self {
            SampleRateCode::FortyEightKHz => 0,
            SampleRateCode::FortyFourPointOneKHz => 1,
            SampleRateCode::ThirtyTwoKHz => 2,
            SampleRateCode::Reserved => 3,
        }
    }

    /// Sample rate in **Hz** per Table 5.6, or `None` for the
    /// reserved codepoint (the spec mandates a decoder mute, with no
    /// associated playback rate).
    pub fn hertz(self) -> Option<u32> {
        match self {
            SampleRateCode::FortyEightKHz => Some(48_000),
            SampleRateCode::FortyFourPointOneKHz => Some(44_100),
            SampleRateCode::ThirtyTwoKHz => Some(32_000),
            SampleRateCode::Reserved => None,
        }
    }

    /// Sample rate in **kHz** per Table 5.6, or `None` for the
    /// reserved codepoint. Equivalent to [`Self::hertz`] divided by
    /// 1 000; kept as a separate accessor since the spec text /
    /// Table 5.6 / Annex D handbook tables phrase the rate in kHz
    /// throughout.
    pub fn kilohertz(self) -> Option<u32> {
        match self {
            SampleRateCode::FortyEightKHz => Some(48),
            SampleRateCode::FortyFourPointOneKHz => Some(44),
            SampleRateCode::ThirtyTwoKHz => Some(32),
            SampleRateCode::Reserved => None,
        }
    }

    /// `true` only for [`Self::Reserved`] — lets a probe / re-emit
    /// tool flag streams that carry the `'11'` reserved codepoint
    /// (per §5.4.1.3 such streams must trigger a decoder mute)
    /// without re-walking Table 5.6.
    pub fn is_reserved(self) -> bool {
        matches!(self, SampleRateCode::Reserved)
    }

    /// Row index into the §7.15 hearing-threshold table
    /// (`tables::HTH[fscod]`) and into [`crate::tables::HTH`] in
    /// general. Returns `None` for [`Self::Reserved`] since the spec
    /// only defines the three valid rows (fscod 0..=2). The index is
    /// equal to [`Self::raw`] for the three valid codepoints but
    /// returning it from a dedicated accessor keeps Table 5.6 →
    /// Table 7.15 routing one call away from the typed surface.
    pub fn hth_row_index(self) -> Option<usize> {
        match self {
            SampleRateCode::FortyEightKHz => Some(0),
            SampleRateCode::FortyFourPointOneKHz => Some(1),
            SampleRateCode::ThirtyTwoKHz => Some(2),
            SampleRateCode::Reserved => None,
        }
    }
}

/// Parse the 5-byte syncinfo out of the front of `data`.
///
/// Returns `Error::Invalid` if the syncword is missing or if the
/// fscod/frmsizecod pair lands on a reserved entry (in which case the
/// spec mandates the decoder mute — see §5.4.1.3 and §5.4.1.4).
pub fn parse(data: &[u8]) -> Result<SyncInfo> {
    if data.len() < 5 {
        return Err(Error::invalid(format!(
            "ac3: syncinfo needs 5 bytes, got {}",
            data.len()
        )));
    }
    let syncword = u16::from_be_bytes([data[0], data[1]]);
    if syncword != SYNCWORD {
        return Err(Error::invalid(format!(
            "ac3: bad syncword 0x{syncword:04X} (expected 0x0B77)"
        )));
    }
    let crc1 = u16::from_be_bytes([data[2], data[3]]);
    // Byte 4: fscod (2 bits, MSB) + frmsizecod (6 bits, LSB).
    let b4 = data[4];
    let fscod = (b4 >> 6) & 0x03;
    let frmsizecod = b4 & 0x3F;
    let sample_rate = sample_rate_hz(fscod)
        .ok_or_else(|| Error::invalid("ac3: reserved fscod '11' — decoder must mute"))?;
    let frame_length = frame_length_bytes(fscod, frmsizecod)
        .ok_or_else(|| Error::invalid(format!("ac3: invalid frmsizecod {frmsizecod}")))?;
    Ok(SyncInfo {
        crc1,
        fscod,
        frmsizecod,
        sample_rate,
        frame_length,
    })
}

/// Find the byte offset of the next AC-3 syncword starting from `offset`.
/// Returns `None` if no syncword is found.
///
/// Used by demuxers that don't already know where frame boundaries are
/// (e.g. raw / corrupted elementary streams) — but well-formed
/// containers hand us frames pre-cut and we can call `parse` directly.
pub fn find_syncword(data: &[u8], start: usize) -> Option<usize> {
    let mut i = start;
    while i + 1 < data.len() {
        if data[i] == 0x0B && data[i + 1] == 0x77 {
            return Some(i);
        }
        i += 1;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Happy path: a hand-crafted 5-byte header for a 48 kHz, 192 kbps
    /// stream (frmsizecod = 20 → 384 words = 768 bytes).
    #[test]
    fn parses_valid_header() {
        let mut buf = [0u8; 5];
        buf[0] = 0x0B;
        buf[1] = 0x77;
        buf[2] = 0xAB;
        buf[3] = 0xCD;
        // fscod=0 (48 kHz), frmsizecod=20
        buf[4] = 20;
        let si = parse(&buf).unwrap();
        assert_eq!(si.sample_rate, 48_000);
        assert_eq!(si.frame_length, 768);
        assert_eq!(si.crc1, 0xABCD);
        assert_eq!(si.frmsizecod, 20);
    }

    #[test]
    fn rejects_bad_syncword() {
        let buf = [0xFF, 0x00, 0, 0, 0];
        assert!(parse(&buf).is_err());
    }

    #[test]
    fn rejects_reserved_fscod() {
        let buf = [0x0B, 0x77, 0, 0, 0xC0]; // fscod = 0b11
        assert!(parse(&buf).is_err());
    }

    #[test]
    fn rejects_reserved_frmsizecod() {
        // frmsizecod = 38 is past the end of Table 5.18.
        let buf = [0x0B, 0x77, 0, 0, 38];
        assert!(parse(&buf).is_err());
    }

    #[test]
    fn finds_syncword() {
        let mut buf = vec![0u8; 32];
        buf[17] = 0x0B;
        buf[18] = 0x77;
        assert_eq!(find_syncword(&buf, 0), Some(17));
        assert_eq!(find_syncword(&buf, 18), None);
    }

    /// Round-trip every Table 5.6 row through `SampleRateCode` —
    /// the four codepoints `'00'`/`'01'`/`'10'`/`'11'` decode to
    /// the four enum variants and `raw()` returns the original
    /// codepoint verbatim.
    #[test]
    fn sample_rate_code_round_trip_all_codepoints() {
        let rows = [
            (0u8, SampleRateCode::FortyEightKHz),
            (1, SampleRateCode::FortyFourPointOneKHz),
            (2, SampleRateCode::ThirtyTwoKHz),
            (3, SampleRateCode::Reserved),
        ];
        for (code, expected) in rows {
            let got = SampleRateCode::from_code(code);
            assert_eq!(got, expected, "code 0x{code:02X}");
            assert_eq!(got.raw(), code, "raw() round-trip for 0x{code:02X}");
        }
    }

    /// `SampleRateCode::from_code` ignores the upper bits of the
    /// argument so a caller can pass a full `bsid`-style byte without
    /// masking first.
    #[test]
    fn sample_rate_code_ignores_upper_bits() {
        assert_eq!(
            SampleRateCode::from_code(0xFC),
            SampleRateCode::FortyEightKHz
        );
        assert_eq!(
            SampleRateCode::from_code(0b1111_1101),
            SampleRateCode::FortyFourPointOneKHz,
        );
        assert_eq!(
            SampleRateCode::from_code(0x06),
            SampleRateCode::ThirtyTwoKHz
        );
        assert_eq!(SampleRateCode::from_code(0xFF), SampleRateCode::Reserved);
    }

    /// `hertz()` and `kilohertz()` return the Table 5.6 rates for the
    /// three valid codepoints and `None` for the reserved codepoint.
    #[test]
    fn sample_rate_code_hertz_and_kilohertz() {
        assert_eq!(SampleRateCode::FortyEightKHz.hertz(), Some(48_000));
        assert_eq!(SampleRateCode::FortyFourPointOneKHz.hertz(), Some(44_100));
        assert_eq!(SampleRateCode::ThirtyTwoKHz.hertz(), Some(32_000));
        assert_eq!(SampleRateCode::Reserved.hertz(), None);

        assert_eq!(SampleRateCode::FortyEightKHz.kilohertz(), Some(48));
        assert_eq!(SampleRateCode::FortyFourPointOneKHz.kilohertz(), Some(44));
        assert_eq!(SampleRateCode::ThirtyTwoKHz.kilohertz(), Some(32));
        assert_eq!(SampleRateCode::Reserved.kilohertz(), None);
    }

    /// `is_reserved()` flags only the `'11'` codepoint.
    #[test]
    fn sample_rate_code_is_reserved_only_for_eleven() {
        assert!(!SampleRateCode::FortyEightKHz.is_reserved());
        assert!(!SampleRateCode::FortyFourPointOneKHz.is_reserved());
        assert!(!SampleRateCode::ThirtyTwoKHz.is_reserved());
        assert!(SampleRateCode::Reserved.is_reserved());
    }

    /// `hth_row_index()` matches the Table 7.15 row order so callers
    /// can route a typed sample-rate code straight into a §7.2.2.5
    /// hearing-threshold lookup.
    #[test]
    fn sample_rate_code_hth_row_index_matches_table_7_15() {
        assert_eq!(SampleRateCode::FortyEightKHz.hth_row_index(), Some(0));
        assert_eq!(
            SampleRateCode::FortyFourPointOneKHz.hth_row_index(),
            Some(1)
        );
        assert_eq!(SampleRateCode::ThirtyTwoKHz.hth_row_index(), Some(2));
        assert_eq!(SampleRateCode::Reserved.hth_row_index(), None);
    }

    /// `SyncInfo::sample_rate_code()` mirrors the resolved
    /// `sample_rate_hz()` lookup on every valid codepoint — for any
    /// frame `parse` accepts, the typed surface and the raw
    /// `sample_rate` field always agree.
    #[test]
    fn syncinfo_sample_rate_code_matches_resolved_rate() {
        // fscod=0 / frmsizecod=20 = 48 kHz, 384 words.
        let buf = [0x0B, 0x77, 0xAB, 0xCD, 20];
        let si = parse(&buf).unwrap();
        let src = si.sample_rate_code();
        assert_eq!(src, SampleRateCode::FortyEightKHz);
        assert_eq!(src.hertz(), Some(si.sample_rate));
        assert!(!src.is_reserved());

        // fscod=1 / frmsizecod=0 = 44.1 kHz, 69 words.
        let buf = [0x0B, 0x77, 0, 0, 0x40];
        let si = parse(&buf).unwrap();
        let src = si.sample_rate_code();
        assert_eq!(src, SampleRateCode::FortyFourPointOneKHz);
        assert_eq!(src.hertz(), Some(si.sample_rate));

        // fscod=2 / frmsizecod=0 = 32 kHz, 96 words.
        let buf = [0x0B, 0x77, 0, 0, 0x80];
        let si = parse(&buf).unwrap();
        let src = si.sample_rate_code();
        assert_eq!(src, SampleRateCode::ThirtyTwoKHz);
        assert_eq!(src.hertz(), Some(si.sample_rate));
    }

    /// A caller constructing a `SyncInfo` by hand (e.g. from
    /// container-stored metadata) with the reserved `fscod = 3`
    /// code still gets the `SampleRateCode::Reserved` typed surface
    /// — `parse` itself never lets the reserved code through, but
    /// the typed accessor on a hand-built `SyncInfo` does flag it.
    #[test]
    fn syncinfo_sample_rate_code_surfaces_reserved_for_hand_built() {
        let si = SyncInfo {
            crc1: 0,
            fscod: 3,
            frmsizecod: 0,
            sample_rate: 0,
            frame_length: 0,
        };
        let src = si.sample_rate_code();
        assert_eq!(src, SampleRateCode::Reserved);
        assert!(src.is_reserved());
        assert_eq!(src.hertz(), None);
        assert_eq!(src.hth_row_index(), None);
    }
}
