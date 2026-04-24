//! Encode PCM, decode with our decoder, dump decoded L-channel to stdout
//! as f32 LE so we can inspect externally.

use oxideav_ac3::{decoder, encoder};
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Error, Frame, SampleFormat, TimeBase};
use std::io::Write;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sr = 48_000u32;
    let n = 4800usize;
    let mut pcm = vec![0i16; n * 2];
    for i in 0..n {
        let t = i as f32 / sr as f32;
        let s = (2.0 * std::f32::consts::PI * 440.0 * t).sin() * 0.4;
        let q = (s * 32767.0) as i16;
        pcm[i * 2] = q;
        pcm[i * 2 + 1] = q;
    }
    let mut bytes = Vec::with_capacity(pcm.len() * 2);
    for s in &pcm {
        bytes.extend_from_slice(&s.to_le_bytes());
    }
    let mut p = CodecParameters::audio(CodecId::new("ac3"));
    p.sample_rate = Some(sr);
    p.channels = Some(2);
    p.sample_format = Some(SampleFormat::S16);
    p.bit_rate = Some(192_000);
    let mut enc = encoder::make_encoder(&p)?;
    enc.send_frame(&Frame::Audio(AudioFrame {
        format: SampleFormat::S16,
        channels: 2,
        sample_rate: sr,
        samples: n as u32,
        pts: Some(0),
        time_base: TimeBase::new(1, sr as i64),
        data: vec![bytes],
    }))?;
    enc.flush()?;
    let mut packets = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(pk) => packets.push(pk),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => return Err(e.into()),
        }
    }

    let dp = CodecParameters::audio(CodecId::new("ac3"));
    let mut dec = decoder::make_decoder(&dp)?;
    let ch: usize = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let mut out = std::io::stdout().lock();
    for pk in &packets {
        dec.send_packet(pk)?;
        if let Ok(Frame::Audio(a)) = dec.receive_frame() {
            let plane = &a.data[0];
            for c in plane.chunks_exact(4) {
                let off = ch * 2;
                let li = i16::from_le_bytes([c[off], c[off + 1]]);
                let f = li as f32 / 32768.0;
                out.write_all(&f.to_le_bytes())?;
            }
        }
    }
    Ok(())
}
