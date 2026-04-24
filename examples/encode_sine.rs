//! Encode a 440 Hz stereo sine wave to `/tmp/sine440.ac3` using our
//! encoder. Run with:
//!
//! ```sh
//! cargo run --example encode_sine -- /tmp/sine440.ac3
//! ```
//!
//! Then decode + verify with ffmpeg:
//!
//! ```sh
//! ffmpeg -y -i /tmp/sine440.ac3 /tmp/sine440.wav
//! ffplay /tmp/sine440.wav
//! ```

use std::env;
use std::fs::File;
use std::io::Write;

use oxideav_ac3::encoder;
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Error, Frame, SampleFormat, TimeBase};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();
    let out_path = args
        .get(1)
        .cloned()
        .unwrap_or_else(|| "/tmp/sine440.ac3".to_string());

    let sr = 48_000u32;
    let dur_s = 2.0f32;
    let nsamp = (sr as f32 * dur_s) as usize;
    let freq = 440.0f32;

    let mut pcm = vec![0i16; nsamp * 2];
    for n in 0..nsamp {
        let t = n as f32 / sr as f32;
        let s = (2.0 * std::f32::consts::PI * freq * t).sin() * 0.4;
        let q = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
        pcm[n * 2] = q;
        pcm[n * 2 + 1] = q;
    }
    let mut bytes = Vec::with_capacity(pcm.len() * 2);
    for s in &pcm {
        bytes.extend_from_slice(&s.to_le_bytes());
    }

    let mut params = CodecParameters::audio(CodecId::new("ac3"));
    params.sample_rate = Some(sr);
    params.channels = Some(2);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(192_000);
    let mut enc = encoder::make_encoder(&params)?;

    let audio = AudioFrame {
        format: SampleFormat::S16,
        channels: 2,
        sample_rate: sr,
        samples: nsamp as u32,
        pts: Some(0),
        time_base: TimeBase::new(1, sr as i64),
        data: vec![bytes],
    };
    enc.send_frame(&Frame::Audio(audio))?;
    let _ = enc.flush();

    let mut out = File::create(&out_path)?;
    let mut nframes = 0usize;
    loop {
        match enc.receive_packet() {
            Ok(p) => {
                out.write_all(&p.data)?;
                nframes += 1;
            }
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => return Err(e.into()),
        }
    }
    eprintln!("wrote {} syncframes to {}", nframes, out_path);
    Ok(())
}
