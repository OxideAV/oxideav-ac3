//! The caller-selectable object-presentation layout of the E-AC-3
//! decoder (`joc_layout` option / `Eac3DecoderOptions`), on streams the
//! crate's own encoders produce. Those streams carry no object metadata,
//! so every frame takes the channel-bed route: the properties pinned
//! here are the output shape contract (channel count, order,
//! `output_params`), the registry and direct construction paths, and
//! that each coded channel reaches its own speaker of the layout
//! unchanged. Object rendering itself is covered by the unit tests next
//! to the renderer (synthetic JOC/OAMD metadata).

use oxideav_ac3::decoder::{
    Eac3DecoderOptions, make_eac3_decoder, make_eac3_decoder_with_joc_layout,
    make_eac3_decoder_with_options,
};
use oxideav_ac3::eac3::{JocLayout, JocSpeaker};
use oxideav_core::{
    AudioFrame, ChannelLayout, CodecId, CodecParameters, CodecRegistry, Decoder, Error, Frame,
    Packet, SampleFormat, TimeBase,
};

const RATE: u32 = 48_000;

/// Encode half a second of per-channel tones; one packet per
/// presentation period.
fn encode(eac3: bool, channels: u16, bit_rate: u64) -> Vec<Vec<u8>> {
    let codec = if eac3 {
        oxideav_ac3::CODEC_ID_STR_EAC3
    } else {
        oxideav_ac3::CODEC_ID_STR
    };
    let mut params = CodecParameters::audio(CodecId::new(codec));
    params.sample_rate = Some(RATE);
    params.channels = Some(channels);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(bit_rate);
    let mut enc = if eac3 {
        oxideav_ac3::eac3::make_encoder(&params).expect("eac3 make_encoder")
    } else {
        oxideav_ac3::encoder::make_encoder(&params).expect("ac3 make_encoder")
    };
    let n = RATE as usize / 2;
    let mut s16 = Vec::with_capacity(n * channels as usize * 2);
    for i in 0..n {
        let t = i as f32 / RATE as f32;
        for ch in 0..channels {
            let f0 = 90.0 * (1.0 + 0.61 * ch as f32);
            let v = 0.3 * (2.0 * std::f32::consts::PI * f0 * t).sin();
            let q = (v * 32767.0).clamp(-32768.0, 32767.0) as i16;
            s16.extend_from_slice(&q.to_le_bytes());
        }
    }
    enc.send_frame(&Frame::Audio(AudioFrame {
        samples: n as u32,
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
    assert!(out.len() > 4, "encoder emitted {} packets", out.len());
    out
}

fn f32_params(codec: &str) -> CodecParameters {
    let mut params = CodecParameters::audio(CodecId::new(codec));
    params.sample_rate = Some(RATE);
    params.sample_format = Some(SampleFormat::F32);
    params
}

/// Decode every packet; returns the frames as f32 samples.
fn decode(dec: &mut dyn Decoder, packets: &[Vec<u8>]) -> Vec<Vec<f32>> {
    let mut frames = Vec::new();
    for pkt in packets {
        dec.send_packet(&Packet::new(0, TimeBase::new(1, RATE as i64), pkt.clone()))
            .expect("send_packet");
        while let Ok(Frame::Audio(af)) = dec.receive_frame() {
            let samples = af.data[0]
                .as_chunks::<4>()
                .0
                .iter()
                .map(|bytes| f32::from_le_bytes(*bytes))
                .collect();
            frames.push(samples);
        }
    }
    assert_eq!(frames.len(), packets.len(), "one frame per packet");
    frames
}

fn channel_of(layout: JocLayout, speaker: JocSpeaker) -> usize {
    layout
        .speakers()
        .iter()
        .position(|&candidate| candidate == speaker)
        .expect("speaker in layout")
}

/// A 5.1 program on the 5.1 layout is the plain multichannel decode.
#[test]
fn layout_5_1_on_a_5_1_stream_is_the_passthrough_decode() {
    let packets = encode(true, 6, 384_000);
    let params = f32_params(oxideav_ac3::CODEC_ID_STR_EAC3);
    let mut plain = make_eac3_decoder(&params).expect("plain decoder");
    let mut layout = make_eac3_decoder_with_joc_layout(&params, JocLayout::Surround51)
        .expect("5.1 layout decoder");
    assert_eq!(
        decode(&mut *layout, &packets),
        decode(&mut *plain, &packets)
    );

    let out = layout.output_params().expect("fixed layout is described");
    assert_eq!(out.channels, Some(6));
    assert_eq!(out.channel_layout, Some(ChannelLayout::Surround51));
    assert_eq!(out.sample_format, Some(SampleFormat::F32));
    assert_eq!(out.sample_rate, Some(RATE));
    assert_eq!(layout.output_params_generation(), 0);
    // The historical factories keep describing nothing up front.
    assert!(plain.output_params().is_none());
}

/// Every layout: fixed channel count, coded channels on their own
/// speakers, nothing invented on the others.
#[test]
fn every_layout_carries_the_5_1_bed_on_its_own_speakers() {
    let packets = encode(true, 6, 384_000);
    let params = f32_params(oxideav_ac3::CODEC_ID_STR_EAC3);
    let reference = decode(&mut *make_eac3_decoder(&params).expect("plain"), &packets);
    let wav_order = JocLayout::Surround51.speakers();
    for layout in JocLayout::ALL {
        if layout == JocLayout::Stereo {
            continue;
        }
        let mut dec = make_eac3_decoder_with_joc_layout(&params, layout).expect("layout decoder");
        let frames = decode(&mut *dec, &packets);
        let channels = usize::from(layout.channel_count());
        let out = dec.output_params().expect("described");
        assert_eq!(out.channels, Some(layout.channel_count()));
        assert_eq!(out.channel_layout, Some(layout.channel_layout()));
        for (frame, plain) in frames.iter().zip(&reference) {
            assert_eq!(frame.len(), plain.len() / 6 * channels, "{layout}");
            for (index, &speaker) in layout.speakers().iter().enumerate() {
                let source = wav_order.iter().position(|&s| s == speaker);
                for (out, plain) in frame.chunks_exact(channels).zip(plain.as_chunks::<6>().0) {
                    let want = source.map_or(0.0, |source| plain[source]);
                    assert_eq!(out[index], want, "{layout} {speaker}");
                }
            }
        }
    }
}

/// The registry path: the `joc_layout` option on the registered factory.
#[test]
fn registry_option_selects_the_layout() {
    let mut registry = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut registry);
    let id = CodecId::new(oxideav_ac3::CODEC_ID_STR_EAC3);
    let schema = registry
        .decoder_options_schema(&id)
        .expect("eac3 decoder option schema");
    assert!(schema.iter().any(|field| field.name == "joc_layout"));

    let packets = encode(true, 6, 384_000);
    let mut params = f32_params(oxideav_ac3::CODEC_ID_STR_EAC3);
    params.options.insert("joc_layout", "7.1.4");
    // Keys the decoder does not know stay ignored, as before.
    params.options.insert("some_demuxer_hint", "1");
    let mut by_registry = registry.first_decoder(&params).expect("registry decoder");
    let typed = Eac3DecoderOptions::new().with_joc_layout(JocLayout::Surround714);
    let mut by_type =
        make_eac3_decoder_with_options(&f32_params(id.as_str()), &typed).expect("typed decoder");
    let frames = decode(&mut *by_registry, &packets);
    assert_eq!(frames, decode(&mut *by_type, &packets));
    assert_eq!(frames[0].len(), 1536 * 12);
    let out = by_registry.output_params().expect("described");
    assert_eq!(out.channels, Some(12));
    assert_eq!(out.channel_layout, Some(ChannelLayout::DiscreteN(12)));

    // A value outside the schema is a construction error.
    let mut bad = f32_params(id.as_str());
    bad.options.insert("joc_layout", "22.2");
    assert!(registry.first_decoder(&bad).is_err());
    // So is a channel request that contradicts the layout.
    let mut clash = f32_params(id.as_str());
    clash.channels = Some(2);
    assert!(make_eac3_decoder_with_joc_layout(&clash, JocLayout::Surround514).is_err());
    clash.channels = Some(10);
    assert!(make_eac3_decoder_with_joc_layout(&clash, JocLayout::Surround514).is_ok());
}

/// `joc_layout = stereo` is the historical opt-in stereo presentation:
/// without object metadata it is the §7.8 downmix.
#[test]
fn stereo_layout_keeps_the_compatibility_downmix() {
    let packets = encode(true, 6, 384_000);
    let mut stereo_params = f32_params(oxideav_ac3::CODEC_ID_STR_EAC3);
    stereo_params.channels = Some(2);
    let mut downmix = make_eac3_decoder(&stereo_params).expect("downmix decoder");
    let reference = decode(&mut *downmix, &packets);

    let mut by_option = f32_params(oxideav_ac3::CODEC_ID_STR_EAC3);
    by_option.options.insert("joc_layout", "stereo");
    let mut dec = make_eac3_decoder(&by_option).expect("stereo layout decoder");
    assert_eq!(decode(&mut *dec, &packets), reference);
    let mut legacy =
        oxideav_ac3::decoder::make_eac3_decoder_with_joc(&stereo_params).expect("legacy factory");
    assert_eq!(decode(&mut *legacy, &packets), reference);
}

/// A 7.1 program (independent 5.1 + dependent back pair) fills the back
/// speakers of a 7.1.x layout and folds onto the surrounds of a 5.1.x.
#[test]
fn dependent_substream_channels_reach_the_back_speakers() {
    let packets = encode(true, 8, 768_000);
    let params = f32_params(oxideav_ac3::CODEC_ID_STR_EAC3);
    let reference = decode(&mut *make_eac3_decoder(&params).expect("plain"), &packets);
    assert_eq!(reference[0].len(), 1536 * 8, "7.1 program");
    // Plain decode order of the extended program: L, C, R, Ls, Rs, LFE,
    // then the appended Lrs / Rrs.
    let plain_order = [
        JocSpeaker::Left,
        JocSpeaker::Center,
        JocSpeaker::Right,
        JocSpeaker::LeftSurround,
        JocSpeaker::RightSurround,
        JocSpeaker::Lfe,
        JocSpeaker::LeftBack,
        JocSpeaker::RightBack,
    ];
    let layout = JocLayout::Surround714;
    let mut dec = make_eac3_decoder_with_joc_layout(&params, layout).expect("7.1.4 decoder");
    let frames = decode(&mut *dec, &packets);
    for (frame, plain) in frames.iter().zip(&reference) {
        for (out, plain) in frame
            .as_chunks::<12>()
            .0
            .iter()
            .zip(plain.as_chunks::<8>().0)
        {
            for (source, &speaker) in plain_order.iter().enumerate() {
                assert_eq!(out[channel_of(layout, speaker)], plain[source], "{speaker}");
            }
            assert!(out[8..].iter().all(|&top| top == 0.0));
        }
    }

    let layout = JocLayout::Surround512;
    let mut dec = make_eac3_decoder_with_joc_layout(&params, layout).expect("5.1.2 decoder");
    let frames = decode(&mut *dec, &packets);
    for (frame, plain) in frames.iter().zip(&reference) {
        for (out, plain) in frame
            .as_chunks::<8>()
            .0
            .iter()
            .zip(plain.as_chunks::<8>().0)
        {
            let ls = out[channel_of(layout, JocSpeaker::LeftSurround)];
            assert!((ls - (plain[3] + plain[6])).abs() < 1.0e-6);
            let rs = out[channel_of(layout, JocSpeaker::RightSurround)];
            assert!((rs - (plain[4] + plain[7])).abs() < 1.0e-6);
        }
    }
}

/// Plain AC-3 through a fixed-layout decoder keeps the layout's shape,
/// in both sample formats.
#[test]
fn ac3_frames_keep_the_selected_layout() {
    let packets = encode(false, 2, 192_000);
    let layout = JocLayout::Surround512;
    let params = f32_params(oxideav_ac3::CODEC_ID_STR);
    let reference = decode(
        &mut *oxideav_ac3::decoder::make_decoder(&params).expect("plain"),
        &packets,
    );
    let mut dec = make_eac3_decoder_with_joc_layout(&params, layout).expect("layout decoder");
    let frames = decode(&mut *dec, &packets);
    for (frame, plain) in frames.iter().zip(&reference) {
        assert_eq!(frame.len(), 1536 * 8);
        for (out, plain) in frame
            .as_chunks::<8>()
            .0
            .iter()
            .zip(plain.as_chunks::<2>().0)
        {
            assert_eq!(out[..2], plain[..]);
            assert!(out[2..].iter().all(|&other| other == 0.0));
        }
    }

    // S16 output is the F32 output quantised.
    let mut s16_params = params.clone();
    s16_params.sample_format = Some(SampleFormat::S16);
    let mut dec = make_eac3_decoder_with_joc_layout(&s16_params, layout).expect("s16 decoder");
    dec.send_packet(&Packet::new(
        0,
        TimeBase::new(1, RATE as i64),
        packets[2].clone(),
    ))
    .expect("send_packet");
    let Ok(Frame::Audio(af)) = dec.receive_frame() else {
        panic!("no frame");
    };
    assert_eq!(af.data[0].len(), 1536 * 8 * 2);
    assert_eq!(
        dec.output_params().and_then(|p| p.sample_format),
        Some(SampleFormat::S16)
    );
}

/// Fuzz regression (`decode_frames`): a syncframe the Annex E decoder
/// conceals (zero-fills) has no f32 program to route, and a fixed-layout
/// decoder must still return the layout's frame shape — silent.
#[test]
fn concealed_frames_keep_the_layout_shape() {
    // Truncated syncframe headers whose audio blocks cannot be parsed.
    const CONCEALED: [u8; 28] = [
        0x0b, 0x77, 0x00, 0x06, 0x35, 0x87, 0xc0, 0x6f, 0x00, 0x00, 0x00, 0x00, 0xa1, 0x00, 0x00,
        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3d,
    ];
    for layout in JocLayout::ALL {
        if layout == JocLayout::Stereo {
            continue;
        }
        for format in [SampleFormat::S16, SampleFormat::F32] {
            let mut params = f32_params(oxideav_ac3::CODEC_ID_STR_EAC3);
            params.sample_format = Some(format);
            let mut dec =
                make_eac3_decoder_with_joc_layout(&params, layout).expect("layout decoder");
            // The fuzzer fed the bytes as two 14-byte packets.
            let mut frames = Vec::new();
            for chunk in CONCEALED.as_chunks::<14>().0 {
                let packet = Packet::new(0, TimeBase::new(1, RATE as i64), chunk.to_vec());
                if dec.send_packet(&packet).is_ok() {
                    while let Ok(Frame::Audio(af)) = dec.receive_frame() {
                        frames.push(af);
                    }
                }
            }
            let Some(af) = frames.first() else {
                panic!("{layout}: the concealed frame must still be returned");
            };
            assert!(af.samples > 0);
            let width = if format == SampleFormat::F32 { 4 } else { 2 };
            assert_eq!(
                af.data[0].len(),
                af.samples as usize * usize::from(layout.channel_count()) * width,
                "{layout}"
            );
            assert!(af.data[0].iter().all(|&byte| byte == 0));
        }
    }
}
