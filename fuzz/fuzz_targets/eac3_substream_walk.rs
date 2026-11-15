#![no_main]

//! Annex E substream walker: splits the input into syncframes via the
//! syncinfo frame-length table and feeds each to `decode_eac3_packet`
//! on a persistent `Eac3DecoderState` — the path that accumulates
//! independent + dependent substreams into one program and applies
//! the §E.3.8.2 replace-or-extend chanmap channel combination, with
//! the following bytes offered as the look-ahead access unit.
//! Bounded to 32 frames.

use libfuzzer_sys::fuzz_target;
use oxideav_ac3::eac3::{Eac3DecoderState, decode_eac3_packet_with_next};
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
        // The bytes after the frame stand in for the next access unit
        // (the enhanced-coupling look-ahead parses its first block on a
        // scratch state).
        let next = (flen < rest.len()).then(|| &rest[flen..]);
        let _ = decode_eac3_packet_with_next(&mut st, &rest[..flen], next);
        off += flen;
    }
});
