//! Count near-full-scale PCM samples — repro harness from bug report.
//!
//! Usage: `cargo run --example count_spikes -- <file.ac3>`

use oxideav_core::{CodecId, CodecParameters, Decoder, Frame, Packet, TimeBase};

fn main() {
    let path = std::env::args().nth(1).expect("usage: count_spikes <file.ac3>");
    let data = std::fs::read(&path).unwrap();
    let mut p = CodecParameters::audio(CodecId::new("ac3"));
    p.channels = Some(2);
    p.sample_rate = Some(48_000);
    let mut dec = oxideav_ac3::decoder::make_decoder(&p).unwrap();
    let tb = TimeBase::new(1, 48_000);

    let (mut total, mut spikes, mut off) = (0u64, 0u64, 0usize);
    while off + 5 <= data.len() {
        let n = match oxideav_ac3::syncinfo::parse(&data[off..]) {
            Ok(si) => si.frame_length as usize,
            Err(_) => break,
        };
        if n < 5 || off + n > data.len() {
            break;
        }
        dec.send_packet(&Packet::new(0, tb, data[off..off + n].to_vec()))
            .unwrap();
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            for c in af.data[0].chunks_exact(2) {
                let s = i16::from_le_bytes([c[0], c[1]]);
                total += 1;
                if s.unsigned_abs() >= 32760 {
                    spikes += 1;
                }
            }
        }
        off += n;
    }
    println!(
        "{path}: {spikes}/{total} full-scale ({:.2}%)",
        100.0 * spikes as f64 / total as f64
    );
}
