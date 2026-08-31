#![no_main]

//! Structure-aware encoder round-trip: the fuzz input picks a
//! contract-valid encoder configuration — layout (1.0/2.0/5.1/7.1
//! pair), bit rate, blocks-per-syncframe (§E.2.3.1.5 fractional
//! shapes), coding tool (SPX / AHT / enhanced coupling / SPX+ecpl)
//! and metadata words (dialnorm / per-block dynrng) — through the
//! registry options path, then supplies the PCM payload. Invalid
//! combinations are rejected at construction (contract behaviour);
//! for every accepted configuration, arbitrary PCM must encode
//! without error and every emitted packet must decode cleanly through
//! our own Annex E decoder with the exact per-frame sample count. A
//! panic, an encode-side bit-budget overflow, or a packet our decoder
//! rejects is an encoder (or decoder) bug.

use libfuzzer_sys::fuzz_target;
use oxideav_ac3::eac3::{decode_eac3_packet, Eac3DecoderState};
use oxideav_core::{
    AudioFrame, CodecId, CodecOptions, CodecParameters, Error, Frame, SampleFormat,
};

fuzz_target!(|data: &[u8]| {
    let mut it = data.iter().copied();
    let (Some(c0), Some(c1), Some(c2), Some(c3)) = (it.next(), it.next(), it.next(), it.next())
    else {
        return;
    };
    let channels: u16 = [1, 2, 6, 8][(c0 & 3) as usize];
    let blocks: usize = [1, 2, 3, 6][((c0 >> 2) & 3) as usize];
    let kbps: u64 = [96, 128, 192, 256, 384, 448, 576, 640][(c1 & 7) as usize];
    let mut opts = CodecOptions::new().set("blocks", blocks.to_string());
    match (c0 >> 4) & 7 {
        1 => opts = opts.set("spx", "1"),
        2 => opts = opts.set("aht", "1"),
        3 => opts = opts.set("ecpl", "1"),
        4 => opts = opts.set("spx", "1").set("ecpl", "1"),
        _ => {}
    }
    if c3 & 1 == 1 {
        opts = opts.set("dynrng", c2.to_string());
    }
    if c3 & 2 == 2 {
        opts = opts.set("dialnorm", (c2 & 31).to_string());
    }
    if c3 & 4 == 4 {
        opts = opts.set("tpnp", "1");
    }
    let mut params = CodecParameters::audio(CodecId::new("eac3"));
    params.sample_rate = Some([48_000u32, 44_100, 32_000][(((c1 >> 3) & 3) % 3) as usize]);
    params.channels = Some(channels);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(kbps * 1000);
    params.options = opts;
    // Construction rejections (tool × blocks × layout × rate
    // mapping) are the documented contract — not findings.
    let Ok(mut enc) = oxideav_ac3::eac3::make_encoder(&params) else {
        return;
    };

    // Two syncframes of PCM from the remaining bytes (16-bit LE,
    // zero/pattern-extended when the input runs short).
    let spf = 256 * blocks;
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

    let mut st = Eac3DecoderState::default();
    let mut samples = 0usize;
    loop {
        match enc.receive_packet() {
            Ok(p) => {
                let f = decode_eac3_packet(&mut st, &p.data)
                    .expect("own decode of an encoder-emitted syncframe");
                samples += f.pcm_s16le.len() / 2 / channels as usize;
            }
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("receive_packet: {e:?}"),
        }
    }
    assert_eq!(samples, 2 * spf, "decoded sample count per channel");
});
