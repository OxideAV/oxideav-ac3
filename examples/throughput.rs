//! Decode / encode / object-render throughput, as multiples of real
//! time, on deterministic synthetic 5.1 programme material.
//!
//! `cargo run --release --example throughput [seconds]`
//!
//! Every stage also prints an FNV-1a hash of its output so a
//! performance change can be checked for bit-identical results.

use std::time::Instant;

use oxideav_ac3::eac3::JocLayout;
use oxideav_ac3::eac3::joc::JocMetadata;
use oxideav_ac3::eac3::joc_renderer::JocRenderer;
use oxideav_ac3::eac3::oamd::synth::{TestObject, TestProgram, payload};
use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Decoder, Error, Frame, Packet, SampleFormat, TimeBase,
};

const RATE: u32 = 48_000;

fn fnv(hash: &mut u64, bytes: &[u8]) {
    for &byte in bytes {
        *hash ^= u64::from(byte);
        *hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
}

/// Six channels of harmonic material with slow level motion, a noise
/// bed and sparse transients (L, R, C, LFE, Ls, Rs).
fn programme(seconds: usize) -> Vec<i16> {
    let n = seconds * RATE as usize;
    let mut noise = 0x1357_9bdf_u32;
    let mut out = Vec::with_capacity(n * 6);
    for i in 0..n {
        let t = i as f32 / RATE as f32;
        for ch in 0..6 {
            noise ^= noise << 13;
            noise ^= noise >> 17;
            noise ^= noise << 5;
            let hiss = (noise >> 8) as f32 / (1 << 23) as f32 - 1.0;
            let v = if ch == 3 {
                0.3 * (std::f32::consts::TAU * 45.0 * t).sin()
            } else {
                let f = 110.0 * (1.0 + 0.31 * ch as f32);
                let level = 0.5 + 0.4 * (std::f32::consts::TAU * (0.7 + 0.2 * ch as f32) * t).sin();
                let click = if (i + ch * 2400) % 17_777 < 24 {
                    0.5
                } else {
                    0.0
                };
                level
                    * (0.16 * (std::f32::consts::TAU * f * t).sin()
                        + 0.10 * (std::f32::consts::TAU * f * 3.0 * t).sin()
                        + 0.07 * (std::f32::consts::TAU * f * 7.0 * t).sin()
                        + 0.04 * (std::f32::consts::TAU * f * 29.0 * t).sin())
                    + 0.02 * hiss
                    + click
            };
            out.push((v * 32767.0).clamp(-32768.0, 32767.0) as i16);
        }
    }
    out
}

fn encode(codec: &str, pcm: &[i16], bit_rate: u64, options: &[(&str, &str)]) -> Vec<Vec<u8>> {
    let mut params = CodecParameters::audio(CodecId::new(codec));
    params.sample_rate = Some(RATE);
    params.channels = Some(6);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(bit_rate);
    for (key, value) in options {
        params.options.insert(*key, *value);
    }
    let mut enc = if codec == "ac3" {
        oxideav_ac3::encoder::make_encoder(&params).expect("ac3 encoder")
    } else {
        oxideav_ac3::eac3::make_encoder(&params).expect("eac3 encoder")
    };
    let mut packets = Vec::new();
    for frame in pcm.chunks(1536 * 6) {
        let bytes: Vec<u8> = frame.iter().flat_map(|s| s.to_le_bytes()).collect();
        enc.send_frame(&Frame::Audio(AudioFrame {
            samples: (frame.len() / 6) as u32,
            pts: None,
            data: vec![bytes],
        }))
        .expect("send_frame");
        while let Ok(packet) = enc.receive_packet() {
            packets.push(packet.data);
        }
    }
    enc.flush().expect("flush");
    loop {
        match enc.receive_packet() {
            Ok(packet) => packets.push(packet.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(error) => panic!("receive_packet: {error:?}"),
        }
    }
    packets
}

/// Decode `packets` (again and again when `THROUGHPUT_REPEAT` asks for
/// a longer measurement); returns the samples, the seconds spent
/// decoding and the output hash of the first pass (hashed off the
/// clock).
fn decode(dec: &mut dyn Decoder, packets: &[Vec<u8>]) -> (u64, f64, u64) {
    let repeat: usize = std::env::var("THROUGHPUT_REPEAT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(1);
    let start = Instant::now();
    let (mut samples, frames) = decode_once(dec, packets);
    let mut elapsed = start.elapsed().as_secs_f64();
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    for frame in &frames {
        fnv(&mut hash, frame);
    }
    drop(frames);
    for _ in 1..repeat {
        let _ = dec.reset();
        let start = Instant::now();
        samples += decode_once(dec, packets).0;
        elapsed += start.elapsed().as_secs_f64();
    }
    (samples, elapsed, hash)
}

fn decode_once(dec: &mut dyn Decoder, packets: &[Vec<u8>]) -> (u64, Vec<Vec<u8>>) {
    let mut frames = Vec::with_capacity(packets.len());
    let mut samples = 0_u64;
    for packet in packets {
        dec.send_packet(&Packet::new(
            0,
            TimeBase::new(1, i64::from(RATE)),
            packet.clone(),
        ))
        .expect("send_packet");
        while let Ok(Frame::Audio(mut frame)) = dec.receive_frame() {
            samples += u64::from(frame.samples);
            frames.push(std::mem::take(&mut frame.data[0]));
        }
    }
    (samples, frames)
}

fn report(stage: &str, samples: u64, seconds: f64, hash: u64) {
    let audio = samples as f64 / f64::from(RATE);
    println!(
        "{stage:<44} {:>8.1} x realtime  ({audio:.1} s audio in {:.3} s)  hash {hash:016x}",
        audio / seconds,
        seconds
    );
}

fn main() {
    let seconds: usize = std::env::args()
        .nth(1)
        .and_then(|arg| arg.parse().ok())
        .unwrap_or(20);
    let pcm = programme(seconds);
    let total = (pcm.len() / 6) as u64;

    // --- encode ---
    let mut streams = Vec::new();
    for (label, codec, rate, options) in [
        ("encode AC-3 5.1 448k", "ac3", 448_000_u64, &[][..]),
        ("encode E-AC-3 5.1 384k", "eac3", 384_000, &[][..]),
        (
            "encode E-AC-3 5.1 384k spx+cpl",
            "eac3",
            384_000,
            &[("spx", "1"), ("cpl", "1")][..],
        ),
        (
            "encode E-AC-3 5.1 384k aht",
            "eac3",
            384_000,
            &[("aht", "1")][..],
        ),
    ] {
        let start = Instant::now();
        let packets = encode(codec, &pcm, rate, options);
        let elapsed = start.elapsed().as_secs_f64();
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        for packet in &packets {
            fnv(&mut hash, packet);
        }
        report(label, total, elapsed, hash);
        streams.push((label.replacen("encode", "decode", 1), codec, packets));
    }

    // --- decode ---
    for (label, codec, packets) in &streams {
        let params = CodecParameters::audio(CodecId::new(*codec));
        let mut dec = if *codec == "ac3" {
            oxideav_ac3::decoder::make_decoder(&params).expect("decoder")
        } else {
            oxideav_ac3::decoder::make_eac3_decoder(&params).expect("decoder")
        };
        let (samples, elapsed, hash) = decode(&mut *dec, packets);
        report(label, samples, elapsed, hash);
    }
    {
        let (_, _, packets) = &streams[1];
        let mut params = CodecParameters::audio(CodecId::new("eac3"));
        params.channels = Some(2);
        let mut dec = oxideav_ac3::decoder::make_eac3_decoder(&params).expect("decoder");
        let (samples, elapsed, hash) = decode(&mut *dec, packets);
        report(
            "decode E-AC-3 5.1 384k -> stereo downmix",
            samples,
            elapsed,
            hash,
        );

        let mut params = CodecParameters::audio(CodecId::new("eac3"));
        params.options.insert("joc_layout", "7.1.4");
        let mut dec = oxideav_ac3::decoder::make_eac3_decoder(&params).expect("decoder");
        let (samples, elapsed, hash) = decode(&mut *dec, packets);
        report(
            "decode E-AC-3 5.1 384k -> 7.1.4 bed route",
            samples,
            elapsed,
            hash,
        );
    }

    // --- JOC object rendering (16 moving objects from the 5.1 bed) ---
    let frames = pcm.len() / (1536 * 6);
    // Bitstream order L, C, R, Ls, Rs, LFE.
    let bitstream: Vec<f32> = pcm
        .as_chunks::<6>()
        .0
        .iter()
        .flat_map(|s| [s[0], s[2], s[1], s[4], s[5], s[3]])
        .map(|s| f32::from(s) / 32768.0)
        .collect();
    let routing: Vec<usize> = (0..15).collect();
    for layout in [
        JocLayout::Stereo,
        JocLayout::Surround51,
        JocLayout::Surround714,
        JocLayout::Surround916,
    ] {
        let mut renderer = JocRenderer::with_layout(layout);
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        let mut elapsed = 0.0_f64;
        let mut rendered = 0_u64;
        for frame in 0..frames {
            // Every object ramps to a new position each frame.
            let objects: Vec<TestObject> = (0..15)
                .map(|object| {
                    let step = (frame * 3 + object * 11) as i16;
                    let mut test =
                        TestObject::at([step % 63, (step * 5) % 63, (step * 7) % 31 - 15]);
                    test.size = (object as u32 * 5) % 32;
                    test
                })
                .collect();
            let oamd = payload(TestProgram::Dynamic { lfe: true }, 1, &objects, 2);
            let metadata = JocMetadata::synthetic((frame % 1024) as u16, oamd, &routing);
            let pcm = &bitstream[frame * 1536 * 6..(frame + 1) * 1536 * 6];
            let start = Instant::now();
            let result = renderer.render(&metadata, pcm, 6, 7, true);
            elapsed += start.elapsed().as_secs_f64();
            match result {
                Ok(samples) => {
                    rendered += 1536;
                    for sample in &samples {
                        fnv(&mut hash, &sample.to_le_bytes());
                    }
                }
                Err(error) => panic!("render: {error}"),
            }
        }
        report(
            &format!("JOC render 15 objects + LFE -> {layout}"),
            rendered,
            elapsed,
            hash,
        );
    }
}
