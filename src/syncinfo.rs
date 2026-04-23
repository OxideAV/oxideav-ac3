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
    let frame_length = frame_length_bytes(fscod, frmsizecod).ok_or_else(|| {
        Error::invalid(format!("ac3: invalid frmsizecod {frmsizecod}"))
    })?;
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
        buf[4] = (0 << 6) | 20;
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
        let buf = [0x0B, 0x77, 0, 0, 0 | 38];
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
}
