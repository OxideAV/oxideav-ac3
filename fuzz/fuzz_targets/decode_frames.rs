#![no_main]

//! Whole-frame decode harness: the first byte routes the payload to
//! the AC-3 or E-AC-3 registry decoder and picks a carve pattern; the
//! rest is fed as up to four consecutive packets on ONE stateful
//! decoder, so cross-frame state (enhanced-coupling carrier
//! threading, coupling-coordinate reuse, downmix state) sees
//! adversarial sequences, not just single frames. Bounded: at most 4
//! packets and 8 receive_frame pumps per packet.

use libfuzzer_sys::fuzz_target;
use oxideav_core::{CodecId, CodecParameters, CodecRegistry, Packet, TimeBase};

fuzz_target!(|data: &[u8]| {
    let Some((&sel, body)) = data.split_first() else {
        return;
    };
    let codec = if sel & 1 == 0 { "ac3" } else { "eac3" };
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    let Ok(mut dec) = reg.first_decoder(&CodecParameters::audio(CodecId::new(codec))) else {
        return;
    };
    // Carve the body into 1, 2, or 4 packets (prefix splits keep the
    // corpus small while exercising resync-after-error paths).
    let parts: usize = match (sel >> 1) & 3 {
        0 => 1,
        1 => 2,
        _ => 4,
    };
    let chunk = (body.len() / parts).max(1);
    for part in body.chunks(chunk).take(parts) {
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), part.to_vec());
        if dec.send_packet(&pkt).is_ok() {
            for _ in 0..8 {
                if dec.receive_frame().is_err() {
                    break;
                }
            }
        }
    }
});
