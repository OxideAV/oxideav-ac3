//! Elementary-stream helpers for the E-AC-3 integration tests: our
//! encoder as a stream source, a syncframe walker, and bit-level frame
//! surgery (header field patches with the CRC word recomputed).

use oxideav_ac3::eac3::{Eac3DecoderState, decode_eac3_packet};
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Error, Frame, SampleFormat};

/// Deterministic programme-like test signal: per-channel detuned tones
/// over a low noise bed, with a level step every 4096 samples so
/// consecutive frames differ. `seed` separates programmes.
pub fn test_pcm(channels: usize, rate: u32, samples: usize, seed: u32) -> Vec<i16> {
    let mut s = seed.wrapping_mul(2_654_435_761).wrapping_add(12345) | 1;
    let mut pcm = Vec::with_capacity(samples * channels);
    for i in 0..samples {
        let t = i as f32 / rate as f32;
        let step = if (i / 4096) % 2 == 0 { 1.0 } else { 0.6 };
        for ch in 0..channels {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            let noise = (s as i32 as f32) / (i32::MAX as f32);
            let f0 = (330.0 + 37.0 * seed as f32) * (1.0 + 0.07 * ch as f32);
            let v = step
                * (0.28 * (std::f32::consts::TAU * f0 * t).sin()
                    + 0.12 * (std::f32::consts::TAU * (2300.0 + 310.0 * ch as f32) * t).sin()
                    + 0.06 * (std::f32::consts::TAU * (8700.0 + 450.0 * ch as f32) * t).sin())
                + 0.01 * noise;
            pcm.push((v * 32767.0).clamp(-32768.0, 32767.0) as i16);
        }
    }
    pcm
}

/// Encode interleaved s16 PCM with our E-AC-3 encoder; one `Vec` per
/// emitted packet (a presentation period: the independent syncframe
/// and any dependent syncframes of the same period).
pub fn encode_packets(
    channels: usize,
    rate: u32,
    bit_rate: u64,
    opts: &[(&str, &str)],
    pcm: &[i16],
) -> Vec<Vec<u8>> {
    let mut params = CodecParameters::audio(CodecId::new(oxideav_ac3::CODEC_ID_STR_EAC3));
    params.sample_rate = Some(rate);
    params.channels = Some(channels as u16);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(bit_rate);
    for (k, v) in opts {
        params.options.insert((*k).to_string(), (*v).to_string());
    }
    let mut enc = oxideav_ac3::eac3::make_encoder(&params).expect("make_encoder");
    let s16: Vec<u8> = pcm.iter().flat_map(|v| v.to_le_bytes()).collect();
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: (pcm.len() / channels) as u32,
        pts: Some(0),
        data: vec![s16],
    }))
    .expect("send_frame");
    enc.flush().expect("flush");
    let mut out = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(pkt) => out.push(pkt.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("receive_packet: {e:?}"),
        }
    }
    out
}

/// Byte ranges of the syncframes in `data` (E-AC-3 `frmsiz` sizing).
pub fn syncframes(data: &[u8]) -> Vec<std::ops::Range<usize>> {
    let mut out = Vec::new();
    let mut off = 0usize;
    while off + 6 <= data.len() {
        assert_eq!(&data[off..off + 2], &[0x0B, 0x77], "syncword at {off}");
        let frmsiz = (usize::from(data[off + 2] & 0x07) << 8) | usize::from(data[off + 3]);
        let len = (frmsiz + 1) * 2;
        assert!(off + len <= data.len(), "syncframe overruns the buffer");
        out.push(off..off + len);
        off += len;
    }
    out
}

/// `strmtyp` of the syncframe starting at `frame[0]`.
pub fn strmtyp(frame: &[u8]) -> u8 {
    frame[2] >> 6
}

/// CRC-16 with generator x^16 + x^15 + x^2 + 1, MSB first, zero
/// initial register (§7.10.1).
fn crc16(data: &[u8]) -> u16 {
    let mut reg = 0u16;
    for &byte in data {
        reg ^= u16::from(byte) << 8;
        for _ in 0..8 {
            reg = if reg & 0x8000 != 0 {
                (reg << 1) ^ 0x8005
            } else {
                reg << 1
            };
        }
    }
    reg
}

/// Recompute the trailing CRC word of an E-AC-3 syncframe (§E.2.2.6:
/// one CRC over everything after the syncword).
pub fn fix_crc(frame: &mut [u8]) {
    let n = frame.len();
    let crc = crc16(&frame[2..n - 2]);
    frame[n - 2..].copy_from_slice(&crc.to_be_bytes());
}

/// Overwrite `width` bits at bit offset `bit` (from the start of the
/// syncframe, MSB first) with `value`. The CRC is NOT touched.
pub fn set_bits(frame: &mut [u8], bit: usize, width: usize, value: u32) {
    for i in 0..width {
        let b = (value >> (width - 1 - i)) & 1;
        let pos = bit + i;
        let mask = 0x80u8 >> (pos % 8);
        if b != 0 {
            frame[pos / 8] |= mask;
        } else {
            frame[pos / 8] &= !mask;
        }
    }
}

/// Re-label a syncframe's `substreamid` (bits 18..21) and fix its CRC.
pub fn with_substreamid(frame: &[u8], id: u8) -> Vec<u8> {
    let mut f = frame.to_vec();
    set_bits(&mut f, 18, 3, u32::from(id));
    fix_crc(&mut f);
    f
}

/// Decode a packet sequence through one decoder state; returns the
/// interleaved s16 PCM of every packet (bit-stream channel order), the
/// channel count of the last packet and the state.
pub fn decode_packets<P: AsRef<[u8]>>(
    state: &mut Eac3DecoderState,
    packets: &[P],
) -> (Vec<Vec<i16>>, usize) {
    let mut out = Vec::with_capacity(packets.len());
    let mut channels = 0usize;
    for p in packets {
        let f = decode_eac3_packet(state, p.as_ref()).expect("decode_eac3_packet");
        channels = f.channels as usize;
        out.push(
            f.pcm_s16le
                .as_chunks::<2>()
                .0
                .iter()
                .map(|c| i16::from_le_bytes(*c))
                .collect(),
        );
    }
    (out, channels)
}

/// De-interleave channel `ch` of `channels`.
pub fn channel(pcm: &[i16], channels: usize, ch: usize) -> Vec<i16> {
    pcm.iter().skip(ch).step_by(channels).copied().collect()
}

/// PSNR in dB of `a` against `b` (16-bit full scale), `f64::INFINITY`
/// when identical.
pub fn psnr(a: &[i16], b: &[i16]) -> f64 {
    assert_eq!(a.len(), b.len());
    let se: f64 = a
        .iter()
        .zip(b)
        .map(|(&x, &y)| {
            let d = f64::from(x) - f64::from(y);
            d * d
        })
        .sum();
    if se == 0.0 {
        return f64::INFINITY;
    }
    10.0 * (32768.0f64 * 32768.0 * a.len() as f64 / se).log10()
}
