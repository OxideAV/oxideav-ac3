//! Raw `.ac3` / `.eac3` elementary streams through the registry: probe,
//! demux (one packet per presentation period, counted against the
//! black-box `ffprobe -count_frames`), decode, seek — the playback
//! corpus cells 120–122 (AC-3 stereo / 5.1, E-AC-3 5.1) — plus the
//! decoder's handling of packets that carry several consecutive
//! syncframes (what an MPEG-TS PES payload looks like).

use std::process::Command;

use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error, Frame, Packet, RuntimeContext, SampleFormat,
    TimeBase,
};

fn ffmpeg_present() -> bool {
    Command::new("ffmpeg")
        .arg("-version")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}

fn tmp(tag: &str, ext: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("oxideav_ac3_es_{tag}_{}.{ext}", std::process::id()))
}

/// Generate a corpus stream with ffmpeg (black-box producer); `args`
/// is the manifest's `ffmpeg_args` column.
fn generate(tag: &str, ext: &str, args: &str) -> Option<std::path::PathBuf> {
    let out = tmp(tag, ext);
    let st = Command::new("ffmpeg")
        .args(["-y", "-hide_banner", "-loglevel", "error"])
        .args(args.split(' '))
        .arg(&out)
        .status()
        .ok()?;
    st.success().then_some(out)
}

/// `ffprobe -count_frames` on the stream: the black-box syncframe count.
fn ffprobe_frames(path: &std::path::Path) -> Option<usize> {
    let out = Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-count_frames",
            "-select_streams",
            "a:0",
            "-show_entries",
            "stream=nb_read_frames",
            "-of",
            "default=noprint_wrappers=1:nokey=1",
        ])
        .arg(path)
        .output()
        .ok()?;
    String::from_utf8(out.stdout).ok()?.trim().parse().ok()
}

fn ctx() -> RuntimeContext {
    let mut ctx = RuntimeContext::new();
    oxideav_ac3::register(&mut ctx);
    ctx
}

struct Cell {
    id: u32,
    ext: &'static str,
    channels: u16,
    codec: &'static str,
    args: &'static str,
}

const CELLS: &[Cell] = &[
    Cell {
        id: 120,
        ext: "ac3",
        channels: 2,
        codec: "ac3",
        args: "-f lavfi -i aevalsrc=sin(440*2*PI*t)|sin(660*2*PI*t):s=48000:c=stereo -c:a ac3 -b:a 192k -f ac3 -t 2",
    },
    Cell {
        id: 121,
        ext: "ac3",
        channels: 6,
        codec: "ac3",
        args: "-f lavfi -i aevalsrc=sin(440*2*PI*t)|sin(660*2*PI*t)|sin(880*2*PI*t)|sin(1100*2*PI*t)|sin(1320*2*PI*t)|sin(1540*2*PI*t):s=48000:c=5.1 -c:a ac3 -b:a 384k -f ac3 -t 2",
    },
    Cell {
        id: 122,
        ext: "eac3",
        channels: 6,
        codec: "eac3",
        args: "-f lavfi -i aevalsrc=sin(440*2*PI*t)|sin(660*2*PI*t)|sin(880*2*PI*t)|sin(1100*2*PI*t)|sin(1320*2*PI*t)|sin(1540*2*PI*t):s=48000:c=5.1 -c:a eac3 -b:a 384k -f eac3 -t 2",
    },
];

#[test]
fn corpus_cells_probe_demux_decode_and_seek() {
    if !ffmpeg_present() {
        eprintln!("ffmpeg unavailable — skipping");
        return;
    }
    let ctx = ctx();
    for cell in CELLS {
        let Some(path) = generate(&format!("c{}", cell.id), cell.ext, cell.args) else {
            eprintln!("ffmpeg could not generate cell {} — skipping", cell.id);
            continue;
        };
        let expect_frames = ffprobe_frames(&path).expect("ffprobe -count_frames");
        let mut file = std::fs::File::open(&path).unwrap();

        // Probe: with the extension hint and without it.
        let name = ctx
            .containers
            .probe_input(&mut file, Some(cell.ext))
            .unwrap();
        assert_eq!(name, "ac3", "cell {}: probe with ext", cell.id);
        let name = ctx.containers.probe_input(&mut file, None).unwrap();
        assert_eq!(name, "ac3", "cell {}: probe without ext", cell.id);

        // Demux: one packet per syncframe, timestamps in samples.
        let mut dmx = ctx
            .containers
            .open_demuxer("ac3", Box::new(file), &ctx.codecs)
            .unwrap();
        let stream = dmx.streams()[0].clone();
        assert_eq!(stream.params.codec_id, CodecId::new(cell.codec));
        assert_eq!(stream.params.sample_rate, Some(48_000));
        assert_eq!(stream.params.channels, Some(cell.channels));
        assert_eq!(stream.time_base, TimeBase::new(1, 48_000));
        let mut packets = Vec::new();
        loop {
            match dmx.next_packet() {
                Ok(p) => packets.push(p),
                Err(Error::Eof) => break,
                Err(e) => panic!("cell {}: next_packet: {e}", cell.id),
            }
        }
        assert_eq!(
            packets.len(),
            expect_frames,
            "cell {}: packet count vs ffprobe -count_frames",
            cell.id
        );
        for (i, p) in packets.iter().enumerate() {
            assert_eq!(p.pts, Some(i as i64 * 1536), "cell {}: pts", cell.id);
            assert_eq!(p.duration, Some(1536));
        }
        assert_eq!(stream.duration, Some(expect_frames as i64 * 1536));
        assert_eq!(
            dmx.duration_micros(),
            Some(expect_frames as i64 * 1536 * 1_000_000 / 48_000)
        );

        // Decode every packet through the registry decoder.
        let mut dec = ctx.codecs.first_decoder(&stream.params).unwrap();
        let mut samples = 0usize;
        let mut energy = 0f64;
        for p in &packets {
            dec.send_packet(p).unwrap();
            while let Ok(Frame::Audio(af)) = dec.receive_frame() {
                samples += af.samples as usize;
                for s in af.data[0].as_chunks::<2>().0 {
                    let v = i16::from_le_bytes(*s) as f64 / 32768.0;
                    energy += v * v;
                }
            }
        }
        assert_eq!(
            samples,
            expect_frames * 1536,
            "cell {}: decoded samples",
            cell.id
        );
        assert!(
            (energy / (samples * cell.channels as usize) as f64).sqrt() > 0.1,
            "cell {}: decode is silent",
            cell.id
        );

        // Seek to the middle and read on: the packet there is the
        // one the sequential pass produced.
        let mid = (expect_frames / 2) as i64 * 1536;
        assert_eq!(dmx.seek_to(0, mid + 7).unwrap(), mid);
        let p = dmx.next_packet().unwrap();
        assert_eq!(p.pts, Some(mid));
        assert_eq!(p.data, packets[expect_frames / 2].data);

        let _ = std::fs::remove_file(&path);
    }
}

// ---------------------------------------------------------------------------
// Packets carrying several consecutive syncframes.
// ---------------------------------------------------------------------------

const FIXTURE: &[u8] = include_bytes!("fixtures/sine440_stereo.ac3");

fn decode_all(dec: &mut dyn oxideav_core::Decoder, pkt: &Packet) -> Vec<AudioFrame> {
    dec.send_packet(pkt).unwrap();
    let mut out = Vec::new();
    while let Ok(Frame::Audio(af)) = dec.receive_frame() {
        out.push(af);
    }
    out
}

#[test]
fn ac3_packet_with_three_syncframes_decodes_three_frames() {
    let params = CodecParameters::audio(CodecId::new("ac3"));
    // Reference: one frame per packet.
    let mut one = oxideav_ac3::decoder::make_decoder(&params).unwrap();
    let mut expect = Vec::new();
    for i in 0..3 {
        let pkt = Packet::new(
            0,
            TimeBase::new(1, 48_000),
            FIXTURE[i * 768..(i + 1) * 768].to_vec(),
        )
        .with_pts(i as i64 * 1536);
        expect.extend(decode_all(one.as_mut(), &pkt));
    }
    // Three syncframes in one packet, in a 90 kHz (MPEG-TS) time base.
    let mut multi = oxideav_ac3::decoder::make_decoder(&params).unwrap();
    let pkt =
        Packet::new(0, TimeBase::new(1, 90_000), FIXTURE[..3 * 768].to_vec()).with_pts(900_000);
    let got = decode_all(multi.as_mut(), &pkt);
    assert_eq!(got.len(), 3);
    for (i, (g, e)) in got.iter().zip(&expect).enumerate() {
        assert_eq!(g.samples, 1536);
        assert_eq!(
            g.pts,
            Some(900_000 + i as i64 * 2880),
            "frame {i}: pts (1536 samples = 2880 ticks)"
        );
        assert_eq!(
            g.data, e.data,
            "frame {i}: PCM identical to the one-per-packet decode"
        );
    }
    // A second send before draining is still refused.
    let mut d = oxideav_ac3::decoder::make_decoder(&params).unwrap();
    d.send_packet(&pkt).unwrap();
    assert!(d.send_packet(&pkt).is_err());
    // A single frame plus trailing junk stays one period and decodes.
    let mut tail = FIXTURE[..768].to_vec();
    tail.extend_from_slice(&[0x0B, 0x77, 0x00]);
    let mut d = oxideav_ac3::decoder::make_decoder(&params).unwrap();
    let got = decode_all(d.as_mut(), &Packet::new(0, TimeBase::new(1, 48_000), tail));
    assert_eq!(got.len(), 1);
}

/// Our E-AC-3 encoder's 7.1 output: each period is an independent 5.1
/// frame followed by a dependent frame; two periods in one packet must
/// decode to two 8-channel frames, identical to decoding them one by one.
#[test]
fn eac3_packet_with_two_indep_dep_periods_decodes_two_frames() {
    let mut params = CodecParameters::audio(CodecId::new(oxideav_ac3::CODEC_ID_STR_EAC3));
    params.sample_rate = Some(48_000);
    params.channels = Some(8);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(576_000);
    let mut enc = oxideav_ac3::eac3::make_encoder(&params).unwrap();
    let n = 1536 * 4;
    let mut s16 = Vec::with_capacity(n * 8 * 2);
    for i in 0..n {
        let t = i as f32 / 48_000.0;
        for ch in 0..8 {
            let v = 0.3 * (2.0 * std::f32::consts::PI * (300.0 + 90.0 * ch as f32) * t).sin();
            s16.extend_from_slice(&((v * 32767.0) as i16).to_le_bytes());
        }
    }
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: n as u32,
        pts: Some(0),
        data: vec![s16],
    }))
    .unwrap();
    enc.flush().unwrap();
    let mut periods = Vec::new();
    loop {
        match enc.receive_packet() {
            Ok(p) => periods.push(p.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("{e:?}"),
        }
    }
    assert!(periods.len() >= 2);
    // Each encoder packet is one period: indep (program start) + dep.
    let heads: Vec<_> = periods
        .iter()
        .map(|p| oxideav_ac3::container::parse_sync_frame(p).unwrap())
        .collect();
    assert!(heads.iter().all(|h| h.eac3 && h.program_start));
    assert!(
        periods.iter().zip(&heads).all(|(p, h)| p.len() > h.len),
        "indep + dep per period"
    );

    let dparams = CodecParameters::audio(CodecId::new("eac3"));
    let mut one = oxideav_ac3::decoder::make_decoder(&dparams).unwrap();
    let mut expect = Vec::new();
    for (i, p) in periods.iter().take(2).enumerate() {
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), p.clone()).with_pts(i as i64 * 1536);
        expect.extend(decode_all(one.as_mut(), &pkt));
    }
    let mut both = periods[0].clone();
    both.extend_from_slice(&periods[1]);
    let mut multi = oxideav_ac3::decoder::make_decoder(&dparams).unwrap();
    let got = decode_all(
        multi.as_mut(),
        &Packet::new(0, TimeBase::new(1, 48_000), both).with_pts(0),
    );
    assert_eq!(got.len(), 2);
    for (i, (g, e)) in got.iter().zip(&expect).enumerate() {
        assert_eq!(g.pts, Some(i as i64 * 1536));
        assert_eq!(g.data[0].len(), 1536 * 8 * 2, "frame {i}: 8 channels");
        assert_eq!(g.data, e.data, "frame {i}: identical PCM");
    }
}
