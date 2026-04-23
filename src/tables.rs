//! Static lookup tables taken directly from ATSC A/52:2018.
//!
//! Every table here is line-for-line from a numbered table in the spec —
//! call sites always cite the section/table so it stays greppable.

/// Table 5.6 — sampling frequency code (fscod) → sample rate in Hz.
///
/// fscod `'11'` is **reserved** and the decoder must mute on receipt
/// (spec §5.4.1.3); this table returns `None` for that case.
pub fn sample_rate_hz(fscod: u8) -> Option<u32> {
    match fscod {
        0 => Some(48_000),
        1 => Some(44_100),
        2 => Some(32_000),
        _ => None,
    }
}

/// Table 5.18 — Frame Size Code Table (1 word = 16 bits).
///
/// Each entry is `(nominal_kbps, words_32k, words_44k, words_48k)`.
/// frmsizecod ranges 0..=37 (6 bits, but values 38..=63 are reserved /
/// invalid); each nominal bitrate has two neighbouring frmsizecod values
/// because 44.1 kHz encoding must alternate frame sizes to hit the
/// declared bit-rate on average (Table 5.18 shows both sizes).
pub const FRAME_SIZE_TABLE: &[(u32, u32, u32, u32)] = &[
    (32, 96, 69, 64),
    (32, 96, 70, 64),
    (40, 120, 87, 80),
    (40, 120, 88, 80),
    (48, 144, 104, 96),
    (48, 144, 105, 96),
    (56, 168, 121, 112),
    (56, 168, 122, 112),
    (64, 192, 139, 128),
    (64, 192, 140, 128),
    (80, 240, 174, 160),
    (80, 240, 175, 160),
    (96, 288, 208, 192),
    (96, 288, 209, 192),
    (112, 336, 243, 224),
    (112, 336, 244, 224),
    (128, 384, 278, 256),
    (128, 384, 279, 256),
    (160, 480, 348, 320),
    (160, 480, 349, 320),
    (192, 576, 417, 384),
    (192, 576, 418, 384),
    (224, 672, 487, 448),
    (224, 672, 488, 448),
    (256, 768, 557, 512),
    (256, 768, 558, 512),
    (320, 960, 696, 640),
    (320, 960, 697, 640),
    (384, 1152, 835, 768),
    (384, 1152, 836, 768),
    (448, 1344, 975, 896),
    (448, 1344, 976, 896),
    (512, 1536, 1114, 1024),
    (512, 1536, 1115, 1024),
    (576, 1728, 1253, 1152),
    (576, 1728, 1254, 1152),
    (640, 1920, 1393, 1280),
    (640, 1920, 1394, 1280),
];

/// Compute the total frame length in **bytes** from (fscod, frmsizecod).
///
/// Returns `None` for reserved fscod (=3) or out-of-range frmsizecod
/// (>= 38). The table is indexed in "16-bit words per syncframe"; we
/// multiply by 2 so the result is the byte length the demuxer / parser
/// expects to consume before the next syncword.
pub fn frame_length_bytes(fscod: u8, frmsizecod: u8) -> Option<u32> {
    let frmsizecod = frmsizecod as usize;
    if frmsizecod >= FRAME_SIZE_TABLE.len() {
        return None;
    }
    let (_, w32, w44, w48) = FRAME_SIZE_TABLE[frmsizecod];
    let words = match fscod {
        0 => w48,
        1 => w44,
        2 => w32,
        _ => return None,
    };
    Some(words * 2)
}

/// Return the nominal bit rate in kbps for a given frmsizecod. The two
/// frmsizecod entries per bitrate are collapsed here since the bitrate
/// is identical. Returns `None` when out of range.
pub fn nominal_bitrate_kbps(frmsizecod: u8) -> Option<u32> {
    let i = frmsizecod as usize;
    if i >= FRAME_SIZE_TABLE.len() {
        return None;
    }
    Some(FRAME_SIZE_TABLE[i].0)
}

/// Table 5.8 — Audio Coding Mode (acmod) → (nfchans, channel ordering).
///
/// `nfchans` here is the number of *full-bandwidth* channels; the total
/// channel count (`nchans`) is `nfchans + 1` when the LFE channel is on.
pub fn acmod_nfchans(acmod: u8) -> u8 {
    match acmod {
        0 => 2, // 1+1 (Ch1, Ch2 — dual mono)
        1 => 1, // 1/0 (C)
        2 => 2, // 2/0 (L, R)
        3 => 3, // 3/0 (L, C, R)
        4 => 3, // 2/1 (L, R, S)
        5 => 4, // 3/1 (L, C, R, S)
        6 => 4, // 2/2 (L, R, SL, SR)
        7 => 5, // 3/2 (L, C, R, SL, SR)
        _ => 0,
    }
}
