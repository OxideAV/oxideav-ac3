#![no_main]

//! Annex E substream walker: splits the input into syncframes via the
//! syncinfo frame-length table and feeds each to `decode_eac3_packet`
//! on a persistent `Eac3DecoderState` — the path that accumulates
//! independent + dependent substreams into one program and applies
//! the §E.3.8.2 replace-or-extend chanmap channel combination.
//! Bounded to 32 frames.

use libfuzzer_sys::fuzz_target;
use oxideav_ac3::eac3::{decode_eac3_packet, Eac3DecoderState};
use oxideav_ac3::syncinfo;

fuzz_target!(|data: &[u8]| {
    let mut st = Eac3DecoderState::default();
    let mut off = 0usize;
    for _ in 0..32 {
        if off >= data.len() {
            break;
        }
        let rest = &data[off..];
        let flen = syncinfo::parse(rest)
            .map(|s| s.frame_length as usize)
            .unwrap_or(rest.len())
            .clamp(1, rest.len());
        let _ = decode_eac3_packet(&mut st, &rest[..flen]);
        off += flen;
    }
});
