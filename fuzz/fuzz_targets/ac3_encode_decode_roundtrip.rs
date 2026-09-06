#![no_main]

//! Structure-aware base-AC-3 encoder round-trip (the r457 twin of
//! `encode_decode_roundtrip`): the fuzz input picks a layout (1/0 …
//! 3/2+LFE), a Table 5.18 bit rate, a sample rate and the §5.4.2
//! metadata words (dialnorm / compr / per-block dynrng) through the
//! registry options path, then supplies the PCM payload. For every
//! configuration the constructor accepts, arbitrary PCM must encode
//! without error and every emitted syncframe must decode through our
//! own AC-3 decoder with the exact 1536-sample frame count. A panic,
//! an encode-side bit-budget overflow, or a packet our decoder
//! rejects is an encoder (or decoder) bug.

use libfuzzer_sys::fuzz_target;
use oxideav_core::{
    AudioFrame, CodecId, CodecOptions, CodecParameters, CodecRegistry, Error, Frame, SampleFormat,
};

fuzz_target!(|data: &[u8]| {
    let mut it = data.iter().copied();
    let (Some(c0), Some(c1), Some(c2), Some(c3)) = (it.next(), it.next(), it.next(), it.next())
    else {
        return;
    };
    let channels: u16 = [1, 2, 3, 4, 5, 6][(c0 % 6) as usize];
    let kbps: u64 = [32, 48, 64, 96, 128, 160, 192, 256, 320, 384, 448, 640][(c1 % 12) as usize];
    let sample_rate = [48_000u32, 44_100, 32_000][((c0 >> 4) % 3) as usize];
    let mut opts = CodecOptions::new();
    if c3 & 1 == 1 {
        opts = opts.set("dynrng", c2.to_string());
    }
    if c3 & 2 == 2 {
        opts = opts.set("dialnorm", (c2 & 31).max(1).to_string());
    }
    if c3 & 4 == 4 {
        opts = opts.set("compr", c2.to_string());
    }
    let mut params = CodecParameters::audio(CodecId::new("ac3"));
    params.sample_rate = Some(sample_rate);
    params.channels = Some(channels);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(kbps * 1000);
    params.options = opts;
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    // Construction rejections are the documented contract, not findings.
    let Ok(mut enc) = reg.first_encoder(&params) else {
        return;
    };

    // Two syncframes of PCM from the remaining bytes (16-bit LE,
    // pattern-extended when the input runs short).
    let spf = 1536usize;
    let n = spf * 2 * channels as usize;
    let mut s16 = Vec::with_capacity(n * 2);
    for i in 0..n {
        s16.push(it.next().unwrap_or(0));
        s16.push(it.next().unwrap_or((i & 0x3f) as u8));
    }
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: (n / channels as usize) as u32,
        pts: Some(0),
        data: vec![s16],
    }))
    .expect("send_frame on a contract-valid config");
    enc.flush().expect("flush");

    let mut dec = reg
        .first_decoder(&CodecParameters::audio(CodecId::new("ac3")))
        .expect("registry decoder");
    let mut samples = 0usize;
    loop {
        match enc.receive_packet() {
            Ok(p) => {
                dec.send_packet(&p).expect("own decode of an encoder-emitted syncframe");
                match dec.receive_frame() {
                    Ok(Frame::Audio(a)) => samples += a.samples as usize,
                    Ok(_) => {}
                    Err(e) => panic!("receive_frame on our own syncframe: {e:?}"),
                }
            }
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("receive_packet: {e:?}"),
        }
    }
    assert_eq!(samples, 2 * spf, "decoded sample count per channel");
});
