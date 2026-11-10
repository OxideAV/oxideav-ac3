#![no_main]

//! Whole-frame decode harness: the first byte routes the payload to
//! the AC-3 or E-AC-3 registry decoder and picks a carve pattern; the
//! rest is fed as up to four consecutive packets on ONE stateful
//! decoder, so cross-frame state (enhanced-coupling carrier
//! threading, coupling-coordinate reuse, downmix state) sees
//! adversarial sequences, not just single frames. Bits 3..6 of the
//! selector pick the E-AC-3 `joc_layout` decoder option (none, or one
//! of the eight object-presentation layouts) and bit 7 the F32 output
//! format, so the fixed-layout channel-bed routing and — on inputs
//! whose skip field parses as JOC/OAMD — the object renderer run on
//! hostile frames too. Bounded: at most 4 packets and 8 receive_frame
//! pumps per packet.

use libfuzzer_sys::fuzz_target;
use oxideav_core::{
    CodecId, CodecParameters, CodecRegistry, Frame, Packet, SampleFormat, TimeBase,
};

const LAYOUTS: [&str; 8] = [
    "stereo", "5.1", "5.1.2", "5.1.4", "7.1", "7.1.2", "7.1.4", "9.1.6",
];

fuzz_target!(|data: &[u8]| {
    let Some((&sel, body)) = data.split_first() else {
        return;
    };
    let codec = if sel & 1 == 0 { "ac3" } else { "eac3" };
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    let mut params = CodecParameters::audio(CodecId::new(codec));
    let layout = match (sel >> 3) & 15 {
        0 => None,
        pick => Some(LAYOUTS[(pick as usize - 1) % LAYOUTS.len()]),
    };
    if let Some(layout) = layout {
        params.options.insert("joc_layout", layout);
    }
    if sel & 0x80 != 0 {
        params.sample_format = Some(SampleFormat::F32);
    }
    let Ok(mut dec) = reg.first_decoder(&params) else {
        return;
    };
    // A fixed multichannel layout promises its shape on every frame.
    let fixed_channels = dec.output_params().and_then(|out| out.channels);
    let bytes_per_sample = if sel & 0x80 != 0 { 4 } else { 2 };
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
                match dec.receive_frame() {
                    Ok(Frame::Audio(frame)) => {
                        if let Some(channels) = fixed_channels {
                            assert_eq!(
                                frame.data[0].len(),
                                frame.samples as usize * usize::from(channels) * bytes_per_sample
                            );
                        }
                    }
                    Ok(_) => {}
                    Err(_) => break,
                }
            }
        }
    }
});
