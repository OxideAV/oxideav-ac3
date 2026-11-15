//! Regression inputs from the coverage-guided fuzz targets, replayed
//! through the same logic as the target that found them.

use oxideav_ac3::eac3::{Eac3DecoderState, decode_eac3_packet};
use oxideav_core::{
    AudioFrame, CodecId, CodecOptions, CodecParameters, Error, Frame, SampleFormat,
};

/// Mirror of `fuzz/fuzz_targets/encode_decode_roundtrip.rs`: a
/// contract-valid configuration picked by the first four bytes, PCM
/// from the rest. Returns `false` when construction rejects the
/// configuration (the documented contract), panics on any encoder
/// error or own-decoder rejection.
fn eac3_roundtrip(data: &[u8]) -> bool {
    let mut it = data.iter().copied();
    let (Some(c0), Some(c1), Some(c2), Some(c3)) = (it.next(), it.next(), it.next(), it.next())
    else {
        return false;
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
    let Ok(mut enc) = oxideav_ac3::eac3::make_encoder(&params) else {
        return false;
    };
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
    true
}

/// Scheduled fuzz run of 2026-11-14: 7.1 pair, 96 kbps total at
/// 44.1 kHz, SPX on, near-silent PCM with one loud sample. The
/// independent 5.1 substream's elected exponent plan left its
/// 2224-bit syncframe 210 bits short.
#[test]
fn eac3_71_spx_96k_quiet_pcm_fits_its_frame() {
    assert!(eac3_roundtrip(&[
        0x1f, 0x68, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x74
    ]));
}
