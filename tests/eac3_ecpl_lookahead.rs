//! Enhanced coupling across the syncframe boundary (Annex E §3.5.5.1).
//!
//! "The non-aliased enhanced coupling channel must be reconstructed
//! using the enhanced coupling transform coefficients from the
//! previous, current and next blocks." For the last block of a
//! syncframe the next block is the first block of the following
//! syncframe, so the decoder has to look one access unit ahead.

mod common;

use common::es::{channel, encode_packets};
use oxideav_ac3::eac3::{Eac3DecoderState, decode_eac3_packet, decode_eac3_packet_with_next};
use oxideav_core::{CodecId, CodecParameters, Error, Frame, Packet, TimeBase};

const RATE: u32 = 48_000;
const FRAMES: usize = 24;

/// Stereo: a low tone below the coupling region and steady tones inside
/// it (the default region starts near 3.5 kHz), the second channel at
/// another level and phase.
fn pcm() -> Vec<i16> {
    let n = FRAMES * 1536;
    let mut out = Vec::with_capacity(2 * n);
    for i in 0..n {
        let t = i as f32 / RATE as f32;
        let tone = |f: f32, ph: f32| (std::f32::consts::TAU * f * t + ph).sin();
        let l = 0.20 * tone(410.0, 0.0) + 0.22 * tone(5130.0, 0.3) + 0.15 * tone(8270.0, 1.1);
        let r = 0.18 * tone(530.0, 0.7) + 0.12 * tone(5130.0, 1.9) + 0.20 * tone(8270.0, 0.2);
        out.push((l * 32767.0) as i16);
        out.push((r * 32767.0) as i16);
    }
    out
}

fn stream() -> Vec<Vec<u8>> {
    encode_packets(
        2,
        RATE,
        256_000,
        &[("ecpl", "1"), ("ecpl_chaos", "0"), ("dither", "0")],
        &pcm(),
    )
}

fn s16(bytes: &[u8]) -> Vec<i16> {
    bytes
        .as_chunks::<2>()
        .0
        .iter()
        .map(|c| i16::from_le_bytes(*c))
        .collect()
}

/// Decode every packet, handing each the packet after it when
/// `look_ahead`.
fn decode(packets: &[Vec<u8>], look_ahead: bool) -> Vec<Vec<i16>> {
    let mut st = Eac3DecoderState::default();
    let out = packets
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let next = packets.get(i + 1).map(|n| &n[..]).filter(|_| look_ahead);
            let f = decode_eac3_packet_with_next(&mut st, p, next).expect("decode");
            s16(&f.pcm_s16le)
        })
        .collect();
    assert_eq!(st.frames_zero_filled, 0);
    out
}

/// Signal-to-error ratio in dB of `got` against `want` over the given
/// sample ranges.
fn snr(got: &[i16], want: &[i16], ranges: impl Iterator<Item = std::ops::Range<usize>>) -> f64 {
    let (mut sig, mut err) = (0.0f64, 0.0f64);
    for r in ranges {
        for i in r {
            let (g, w) = (f64::from(got[i]), f64::from(want[i]));
            sig += w * w;
            err += (g - w) * (g - w);
        }
    }
    10.0 * (sig / err.max(1e-9)).log10()
}

#[test]
fn last_block_of_a_frame_uses_the_next_frames_first_block() {
    let packets = stream();
    let plain = decode(&packets, false);
    let ahead = decode(&packets, true);
    // The look-ahead changes the spectrum of block 5 only; through the
    // overlap that reaches the output of block 5 and of the next
    // frame's block 0. Blocks 1..=4 are untouched.
    let mut changed = 0;
    for (i, (a, b)) in plain.iter().zip(&ahead).enumerate() {
        assert!(
            a[2 * 256..2 * 5 * 256] == b[2 * 256..2 * 5 * 256],
            "frame {i}: blocks 1..=4"
        );
        if i + 1 < plain.len() && a[2 * 5 * 256..] != b[2 * 5 * 256..] {
            changed += 1;
        }
    }
    assert!(
        changed >= plain.len() - 2,
        "block 5 changed in {changed} frames"
    );
    // The stream's last frame has nothing after it: past its first
    // block (which overlaps the frame before) it is the same decode.
    let (a, b) = (plain.last().unwrap(), ahead.last().unwrap());
    assert!(a[2 * 256..] == b[2 * 256..], "last frame");

    // Against the source (the decode is the source delayed by one
    // block). Channel 0 is the channel the carrier is locked to (angle
    // zero): its coefficients do not depend on the aliasing terms.
    // Channel 1 is rotated against the carrier, which only works on a
    // non-aliased carrier: without the next block its two blocks at
    // every frame boundary are 10 dB worse than the blocks inside the
    // frame; with it they match.
    let quality = |dec: &[Vec<i16>], ch: usize| -> (f64, f64) {
        let src = channel(&pcm(), 2, ch);
        let flat: Vec<i16> = dec.iter().flat_map(|f| channel(f, 2, ch)).collect();
        let delayed: Vec<i16> = std::iter::repeat_n(0, 256)
            .chain(src.iter().copied())
            .take(flat.len())
            .collect();
        let frames = 2..FRAMES - 2;
        let inner = snr(
            &flat,
            &delayed,
            frames.clone().map(|f| f * 1536 + 512..f * 1536 + 1024),
        );
        let edge = snr(
            &flat,
            &delayed,
            frames.map(|f| f * 1536 + 1280..f * 1536 + 1536 + 256),
        );
        (inner, edge)
    };
    let (inner0, edge0) = quality(&ahead, 0);
    assert!(
        (inner0 - edge0).abs() < 2.0,
        "channel 0: {inner0:.1} / {edge0:.1} dB"
    );
    let (inner_plain, edge_plain) = quality(&plain, 1);
    let (inner, edge) = quality(&ahead, 1);
    eprintln!(
        "channel 1, inner / boundary blocks: {inner_plain:.1} / {edge_plain:.1} dB without the next block, {inner:.1} / {edge:.1} dB with it"
    );
    assert!(inner > 20.0, "channel 1 inside the frame: {inner:.1} dB");
    assert!(
        edge > inner - 2.5,
        "channel 1 at the frame boundary: {edge:.1} dB, {inner:.1} dB inside the frame"
    );
    assert!(
        edge > edge_plain + 6.0,
        "the next block must matter: {edge:.1} dB with, {edge_plain:.1} dB without"
    );
}

fn registry_decoder() -> Box<dyn oxideav_core::Decoder> {
    let params = CodecParameters::audio(CodecId::new("eac3"));
    oxideav_ac3::decoder::make_eac3_decoder(&params).unwrap()
}

/// The registry decoder gets the look-ahead by itself: a period whose
/// last block uses enhanced coupling is held until the next period has
/// arrived (one packet of latency, no shift of the audio), and the end
/// of the stream releases the last one.
#[test]
fn registry_decoder_waits_for_the_next_access_unit() {
    let packets = stream();
    let want = decode(&packets, true);

    // One access unit per packet.
    let mut dec = registry_decoder();
    let mut got: Vec<(Option<i64>, Vec<i16>)> = Vec::new();
    for (i, p) in packets.iter().enumerate() {
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), p.clone()).with_pts(i as i64 * 1536);
        dec.send_packet(&pkt).expect("send_packet");
        let mut frames_now = 0;
        loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(af)) => {
                    frames_now += 1;
                    got.push((af.pts, s16(&af.data[0])));
                }
                Ok(_) => {}
                Err(Error::NeedMore) => break,
                Err(e) => panic!("receive_frame: {e:?}"),
            }
        }
        // The first packet is held; from then on each packet releases
        // the one before it.
        assert_eq!(frames_now, usize::from(i > 0), "packet {i}");
    }
    dec.flush().unwrap();
    while let Ok(Frame::Audio(af)) = dec.receive_frame() {
        got.push((af.pts, s16(&af.data[0])));
    }
    assert_eq!(got.len(), packets.len());
    for (i, ((pts, pcm), w)) in got.iter().zip(&want).enumerate() {
        assert_eq!(*pts, Some(i as i64 * 1536), "frame {i} pts");
        assert!(pcm == w, "frame {i}: the look-ahead decode");
    }

    // The whole stream in one packet: the same audio, no waiting.
    let mut dec = registry_decoder();
    dec.send_packet(&Packet::new(0, TimeBase::new(1, 48_000), packets.concat()).with_pts(0))
        .unwrap();
    dec.flush().unwrap();
    let mut all = Vec::new();
    while let Ok(Frame::Audio(af)) = dec.receive_frame() {
        all.push(s16(&af.data[0]));
    }
    assert!(all == want, "one-packet decode");

    // A stream without enhanced coupling is never held.
    let plain = encode_packets(2, RATE, 192_000, &[("cpl", "1")], &pcm());
    let mut dec = registry_decoder();
    for p in &plain {
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 48_000), p.clone()))
            .unwrap();
        assert!(matches!(dec.receive_frame(), Ok(Frame::Audio(_))));
    }
}

/// The frame-level entry point without a next access unit is the
/// decode with the frame's last block followed by nothing.
#[test]
fn without_a_next_access_unit_the_decode_is_unchanged() {
    let packets = stream();
    let mut st = Eac3DecoderState::default();
    let direct: Vec<Vec<i16>> = packets
        .iter()
        .map(|p| s16(&decode_eac3_packet(&mut st, p).unwrap().pcm_s16le))
        .collect();
    assert!(direct == decode(&packets, false));
}
