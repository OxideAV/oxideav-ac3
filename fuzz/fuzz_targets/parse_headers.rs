#![no_main]

//! Header-parser harness: syncinfo (§5.3.1), base-AC-3 BSI (§5.4.2 —
//! incl. the Annex D alternate-syntax metadata chain) and Annex E BSI
//! (Table E1.2 — incl. the mixing / informational opt-in chains, the
//! dependent-substream chanmap, and the fractional-frame convsync /
//! blkid tail). Every byte sequence must return Ok or Err without
//! panicking; header field values are additionally consistency-checked
//! where the parse promises derived invariants.

use libfuzzer_sys::fuzz_target;
use oxideav_ac3::{bsi, eac3, syncinfo};

fuzz_target!(|data: &[u8]| {
    if let Ok(si) = syncinfo::parse(data) {
        // frame_length is a table value — never zero, always even.
        assert!(si.frame_length > 0 && si.frame_length % 2 == 0);
    }
    // Base AC-3 BSI starts right after the 5-byte syncinfo.
    if data.len() > 5 {
        if let Ok(b) = bsi::parse(&data[5..]) {
            assert!(b.nfchans <= 5);
        }
    }
    // Annex E BSI starts right after the 2-byte syncword.
    if data.len() > 2 {
        if let Ok(b) = eac3::bsi::parse(&data[2..]) {
            assert!(b.num_blocks == 1 || b.num_blocks == 2 || b.num_blocks == 3 || b.num_blocks == 6);
            assert!(b.nfchans <= 5);
            // convsync only exists on fractional independent frames.
            if b.convsync.is_some() {
                assert_eq!(b.numblkscod & 0x3 == 0x3, false);
            }
        }
    }
});
