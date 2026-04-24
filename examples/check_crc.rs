//! Verify AC-3 CRC consistency on an elementary stream. Walks each
//! syncframe, recomputes crc1 (over frame bytes 2..5/8-size, with the
//! stored crc1 field included) and crc2 (over frame bytes 5/8-size..end),
//! and reports frames where the residue is non-zero.
//!
//! Per §7.10.1: "Checking for valid CRC [...] consists of resetting all
//! registers to zero, and then shifting the AC-3 data bits serially into
//! the circuit [...] crc1 is considered valid if the above register
//! contains all zeros after the first 5/8 of the syncframe has been
//! shifted in." So we run the entire region through the LFSR and expect
//! zero.

use std::env;
use std::fs;

fn ac3_crc(init: u16, data: &[u8]) -> u16 {
    let mut crc: u32 = init as u32;
    for &b in data {
        for i in (0..8).rev() {
            let bit = ((b >> i) & 1) as u32;
            let top = (crc >> 15) & 1;
            crc = ((crc << 1) & 0xFFFF) | bit;
            if top != 0 {
                crc ^= 0x8005;
            }
        }
    }
    crc as u16
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args().nth(1).unwrap_or_else(|| "/tmp/sine440.ac3".into());
    let data = fs::read(&path)?;

    let mut offset = 0;
    let mut frame_idx = 0;
    let mut fail1 = 0;
    let mut fail2 = 0;
    while offset + 5 <= data.len() {
        if data[offset] != 0x0B || data[offset + 1] != 0x77 {
            eprintln!("no sync at offset {offset}");
            break;
        }
        let fscod = (data[offset + 4] >> 6) & 0x3;
        let frmsizecod = data[offset + 4] & 0x3F;
        let words_48k: &[u32] = &[
            64, 64, 80, 80, 96, 96, 112, 112, 128, 128, 160, 160, 192, 192, 224, 224,
            256, 256, 320, 320, 384, 384, 448, 448, 512, 512, 640, 640, 768, 768, 896,
            896, 1024, 1024, 1152, 1152, 1280, 1280,
        ];
        let words = words_48k[frmsizecod as usize];
        let _ = fscod;
        let frame_bytes = (words * 2) as usize;
        let five_eighths_words = (words >> 1) + (words >> 3);
        let five_eighths_bytes = (five_eighths_words * 2) as usize;
        let frame = &data[offset..offset + frame_bytes];

        // crc1 covers bytes [2..5/8_bytes]. The crc1 field is *included*
        // as its stored value. Residue should be zero.
        let c1 = ac3_crc(0, &frame[2..five_eighths_bytes]);
        // crc2 covers bytes [5/8_bytes..frame_end].
        let c2 = ac3_crc(0, &frame[five_eighths_bytes..frame_bytes]);
        if c1 != 0 {
            fail1 += 1;
            if fail1 < 3 {
                eprintln!("frame {frame_idx}: crc1 residue = 0x{c1:04x}");
            }
        }
        if c2 != 0 {
            fail2 += 1;
            if fail2 < 3 {
                eprintln!("frame {frame_idx}: crc2 residue = 0x{c2:04x}");
            }
        }
        offset += frame_bytes;
        frame_idx += 1;
    }
    eprintln!("checked {frame_idx} frames; crc1 fail {fail1}, crc2 fail {fail2}");
    Ok(())
}
