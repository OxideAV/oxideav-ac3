//! The registry decoder on a byte stream instead of tidy packets:
//! syncframes split across packets, a stream joined part-way through a
//! frame, bytes between frames that belong to no frame, and frames
//! damaged in transit (§7.10 / Annex E §E.3.2).
//!
//! E-AC-3 streams are encoded with `dither = 0` so two decodes that
//! see different frame histories still agree sample for sample once
//! the overlap has recovered; the AC-3 fixture carries dither, so its
//! post-recovery comparison is a PSNR bound.

mod common;

use common::es::{encode_packets, psnr, syncframes, test_pcm};
use oxideav_core::{AudioFrame, CodecId, CodecParameters, Decoder, Error, Frame, Packet, TimeBase};

const AC3_FIXTURE: &[u8] = include_bytes!("fixtures/sine440_stereo.ac3");

fn decoder(id: &str) -> Box<dyn Decoder> {
    let params = CodecParameters::audio(CodecId::new(id));
    if id == "ac3" {
        oxideav_ac3::decoder::make_decoder(&params).unwrap()
    } else {
        oxideav_ac3::decoder::make_eac3_decoder(&params).unwrap()
    }
}

fn drain(dec: &mut dyn Decoder, out: &mut Vec<AudioFrame>) {
    loop {
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => out.push(af),
            Ok(_) => {}
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("receive_frame: {e:?}"),
        }
    }
}

/// Feed `chunks` as consecutive packets (the first carrying `pts` 0 in
/// a 90 kHz time base), flush, and collect every frame.
fn decode_chunks<C: AsRef<[u8]>>(id: &str, chunks: &[C]) -> Vec<AudioFrame> {
    let mut dec = decoder(id);
    let mut out = Vec::new();
    for (i, c) in chunks.iter().enumerate() {
        let mut pkt = Packet::new(0, TimeBase::new(1, 90_000), c.as_ref().to_vec());
        if i == 0 {
            pkt = pkt.with_pts(0);
        }
        dec.send_packet(&pkt).expect("send_packet");
        drain(dec.as_mut(), &mut out);
    }
    dec.flush().unwrap();
    drain(dec.as_mut(), &mut out);
    out
}

fn s16(frame: &AudioFrame) -> Vec<i16> {
    frame.data[0]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|c| i16::from_le_bytes(*c))
        .collect()
}

/// Deterministic chunk sizes in `1..=max`.
fn chop(data: &[u8], max: usize, mut seed: u32) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut off = 0usize;
    while off < data.len() {
        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let n = (1 + (seed >> 8) as usize % max).min(data.len() - off);
        out.push(data[off..off + n].to_vec());
        off += n;
    }
    out
}

struct Stream {
    id: &'static str,
    /// One entry per presentation period.
    periods: Vec<Vec<u8>>,
    channels: usize,
}

impl Stream {
    fn bytes(&self) -> Vec<u8> {
        self.periods.concat()
    }
}

fn eac3_stream(channels: usize, bit_rate: u64, opts: &[(&str, &str)]) -> Stream {
    let pcm = test_pcm(channels, 48_000, 12 * 1536, 11);
    let mut all = vec![("dither", "0")];
    all.extend_from_slice(opts);
    Stream {
        id: "eac3",
        periods: encode_packets(channels, 48_000, bit_rate, &all, &pcm),
        channels,
    }
}

fn ac3_stream() -> Stream {
    Stream {
        id: "ac3",
        periods: AC3_FIXTURE
            .as_chunks::<768>()
            .0
            .iter()
            .take(12)
            .map(|frame| frame.to_vec())
            .collect(),
        channels: 2,
    }
}

fn streams() -> Vec<Stream> {
    vec![
        ac3_stream(),
        eac3_stream(2, 192_000, &[]),
        eac3_stream(6, 448_000, &[]),
        eac3_stream(8, 768_000, &[]),
        eac3_stream(2, 192_000, &[("blocks", "2")]),
    ]
}

/// Any packetisation of the same bytes decodes to the same frames with
/// the same timestamps: one period per packet (a file container), the
/// whole stream in one packet, and arbitrary cuts that split syncframes
/// (a transport stream).
#[test]
fn packet_boundaries_do_not_change_the_decode() {
    for st in streams() {
        let want = decode_chunks(st.id, &st.periods);
        assert_eq!(
            want.len(),
            st.periods.len(),
            "{}: one frame per period",
            st.id
        );
        let bytes = st.bytes();
        let cuts: Vec<Vec<Vec<u8>>> = vec![
            vec![bytes.clone()],
            chop(&bytes, 188, 1),
            chop(&bytes, 7, 2),
            chop(&bytes, 3000, 3),
        ];
        for (n, chunks) in cuts.iter().enumerate() {
            let got = decode_chunks(st.id, chunks);
            assert_eq!(got.len(), want.len(), "{} cut {n}: frame count", st.id);
            for (i, (g, w)) in got.iter().zip(&want).enumerate() {
                assert_eq!(g.samples, w.samples);
                if g.data != w.data {
                    let (a, b) = (s16(g), s16(w));
                    let first = a.iter().zip(&b).position(|(x, y)| x != y).unwrap();
                    let last = a.iter().zip(&b).rposition(|(x, y)| x != y).unwrap();
                    panic!(
                        "{} cut {n}: frame {i} PCM differs in samples {first}..={last} of {} (psnr {:.1})",
                        st.id,
                        a.len(),
                        psnr(&a, &b)
                    );
                }
                // 48 kHz samples in a 90 kHz time base.
                let ticks: i64 = want[..i]
                    .iter()
                    .map(|f| i64::from(f.samples) * 15 / 8)
                    .sum();
                assert_eq!(g.pts, Some(ticks), "{} cut {n}: frame {i} pts", st.id);
            }
        }
    }
}

/// One syncframe per packet — the independent and the dependent frame
/// of each period in packets of their own. The first period is
/// released before its dependent frame is seen (nothing says one is
/// coming); from the second period on the decode is the full program.
#[test]
fn one_syncframe_per_packet_keeps_dependent_substreams() {
    let st = eac3_stream(8, 768_000, &[]);
    let want = decode_chunks(st.id, &st.periods);
    let frames: Vec<Vec<u8>> = st
        .periods
        .iter()
        .flat_map(|p| syncframes(p).into_iter().map(|r| p[r].to_vec()))
        .collect();
    assert_eq!(frames.len(), 2 * st.periods.len());
    let got = decode_chunks(st.id, &frames);
    assert_eq!(got.len(), want.len());
    assert_eq!(got[0].data[0].len(), 1536 * 6 * 2, "first period: 5.1 only");
    for (i, (g, w)) in got.iter().zip(&want).enumerate().skip(1) {
        assert_eq!(g.data[0].len(), 1536 * 8 * 2, "frame {i}: 7.1");
        if i == 1 {
            // The dependent substream starts one frame late: its
            // channels lack the overlap of the frame before.
            let (g, w) = (s16(g), s16(w));
            assert!(psnr(&g[256 * 8..], &w[256 * 8..]) > 60.0, "frame 1");
        } else {
            assert!(g.data == w.data, "frame {i} PCM");
        }
    }
}

/// A stream joined in the middle of a syncframe: the partial frame is
/// skipped, decoding starts at the next syncframe, and from the second
/// decoded frame on the output equals the full decode (the first only
/// lacks the overlap of the frame before it).
#[test]
fn joining_mid_frame_resynchronises_at_the_next_syncframe() {
    for st in streams() {
        let want = decode_chunks(st.id, &st.periods);
        let bytes = st.bytes();
        let skip = st.periods[0].len() + st.periods[1].len() / 3;
        let got = decode_chunks(st.id, &chop(&bytes[skip..], 1500, 5));
        assert_eq!(
            got.len(),
            want.len() - 2,
            "{}: frames after the join",
            st.id
        );
        let tail = &want[2..];
        for (i, (g, w)) in got.iter().zip(tail).enumerate() {
            let (g, w) = (s16(g), s16(w));
            if i == 0 {
                let from = 256 * st.channels;
                assert!(
                    psnr(&g[from..], &w[from..]) > 60.0,
                    "{}: first frame after the join, past its first block",
                    st.id
                );
            } else if st.id == "eac3" {
                assert!(g == w, "{}: frame {i} after the join", st.id);
            } else {
                assert!(psnr(&g, &w) > 60.0, "{}: frame {i} after the join", st.id);
            }
        }
    }
}

/// Bytes between syncframes that belong to no frame — including a
/// stray syncword with a plausible header — are skipped without losing
/// or disturbing a frame.
#[test]
fn bytes_between_frames_are_skipped() {
    for st in streams() {
        let want = decode_chunks(st.id, &st.periods);
        let mut dirty = Vec::new();
        let mut seed = 0x1234_5678u32;
        for (i, p) in st.periods.iter().enumerate() {
            dirty.extend_from_slice(p);
            match i % 3 {
                0 => {}
                1 => {
                    for _ in 0..37 {
                        seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                        dirty.push((seed >> 24) as u8);
                    }
                }
                _ => {
                    // The header of a real frame, cut short and
                    // followed by noise.
                    dirty.extend_from_slice(&p[..9]);
                    dirty.extend_from_slice(&[0xA5; 21]);
                }
            }
        }
        let got = decode_chunks(st.id, &chop(&dirty, 900, 9));
        assert_eq!(got.len(), want.len(), "{}: frame count", st.id);
        for (i, (g, w)) in got.iter().zip(&want).enumerate() {
            assert!(g.data == w.data, "{}: frame {i}", st.id);
        }
    }
}

/// Largest step between adjacent samples of one channel.
fn max_step(x: &[i16]) -> i32 {
    x.windows(2)
        .map(|w| (i32::from(w[1]) - i32::from(w[0])).abs())
        .max()
        .unwrap_or(0)
}

/// A frame damaged in transit (CRC mismatch) is replaced by the
/// concealment signal — the previous block's overlap tail, then
/// silence — without a step at the join, the frame count and timing
/// are kept, and the frames after it decode as if nothing happened once
/// the overlap has recovered.
#[test]
fn damaged_frame_is_concealed_and_the_stream_recovers() {
    for st in streams() {
        let want = decode_chunks(st.id, &st.periods);
        let hit = 5usize;
        for which in 0..2 {
            let mut periods = st.periods.clone();
            // Damage the first syncframe of the period, or its last
            // (the dependent frame of a 7.1 pair).
            let whole = 0..periods[hit].len();
            let r = if st.id != "eac3" {
                whole
            } else {
                let frames = syncframes(&periods[hit]);
                if which == 0 {
                    frames[0].clone()
                } else {
                    frames[frames.len() - 1].clone()
                }
            };
            for k in 0..24 {
                periods[hit][r.start + 40 + 7 * k] ^= 0x5A;
            }
            let got = decode_chunks(st.id, &periods);
            assert_eq!(got.len(), want.len(), "{}: frame count kept", st.id);
            let ch = st.channels;
            let spf = want[hit].samples as usize;
            for (i, (g, w)) in got.iter().zip(&want).enumerate() {
                assert_eq!(g.pts, w.pts, "{}: frame {i} pts", st.id);
                assert_eq!(
                    g.data[0].len(),
                    w.data[0].len(),
                    "{}: frame {i} shape",
                    st.id
                );
                let (g, w) = (s16(g), s16(w));
                if i < hit {
                    assert!(g == w, "{}: frame {i} before the damage", st.id);
                } else if i == hit {
                    // Channels of an undamaged substream decode as
                    // usual; a concealed channel is silent after its
                    // first block and joins the previous frame
                    // without a step.
                    let prev = s16(&got[hit - 1]);
                    let mut concealed = 0;
                    for c in 0..ch {
                        let gc: Vec<i16> = g.iter().skip(c).step_by(ch).copied().collect();
                        let wc: Vec<i16> = w.iter().skip(c).step_by(ch).copied().collect();
                        if gc == wc {
                            continue;
                        }
                        concealed += 1;
                        assert!(
                            gc[256..].iter().all(|&v| v == 0),
                            "{}: concealed channel {c} silent after the tail",
                            st.id
                        );
                        let pc: Vec<i16> = prev.iter().skip(c).step_by(ch).copied().collect();
                        let join = (i32::from(gc[0]) - i32::from(pc[spf - 1])).abs();
                        let usual = max_step(&pc[spf - 128..]).max(max_step(&wc[..128]));
                        assert!(
                            join <= 2 * usual + 64,
                            "{}: channel {c} steps {join} into the concealed frame (usual {usual})",
                            st.id
                        );
                    }
                    assert!(concealed > 0, "{}: the damage was detected", st.id);
                } else if i == hit + 1 {
                    let from = 256 * ch;
                    assert!(
                        psnr(&g[from..], &w[from..]) > 60.0,
                        "{}: frame after the damage, past its first block",
                        st.id
                    );
                } else if st.id == "eac3" {
                    assert!(g == w, "{}: frame {i} after recovery", st.id);
                } else {
                    assert!(psnr(&g, &w) > 60.0, "{}: frame {i} after recovery", st.id);
                }
            }
        }
    }
}

/// With the CRC check switched off (`crc = false`) a damaged frame is
/// parsed as it stands; the decoder still returns a frame of the right
/// shape for every period.
#[test]
fn crc_check_can_be_switched_off() {
    let st = eac3_stream(2, 192_000, &[]);
    let mut periods = st.periods.clone();
    for k in 0..24 {
        periods[5][40 + 7 * k] ^= 0x5A;
    }
    let mut params = CodecParameters::audio(CodecId::new("eac3"));
    params.options.insert("crc", "false");
    let mut dec = oxideav_ac3::decoder::make_eac3_decoder(&params).unwrap();
    let mut out = Vec::new();
    for p in &periods {
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 48_000), p.clone()))
            .unwrap();
        drain(dec.as_mut(), &mut out);
    }
    assert_eq!(out.len(), periods.len());
    assert!(out.iter().all(|f| f.data[0].len() == 1536 * 2 * 2));
}

/// Input that holds no syncframe is skipped silently for a while (a
/// stream may be joined anywhere), then reported.
#[test]
fn input_without_any_syncframe_is_reported() {
    let mut dec = decoder("eac3");
    let junk = vec![0x55u8; 4096];
    let mut reported = false;
    for _ in 0..12 {
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 48_000), junk.clone()))
            .unwrap();
        match dec.receive_frame() {
            Err(Error::NeedMore) => {}
            Err(_) => {
                reported = true;
                break;
            }
            Ok(_) => panic!("junk decoded to a frame"),
        }
    }
    assert!(reported, "48 KiB without a syncframe must surface an error");
}
