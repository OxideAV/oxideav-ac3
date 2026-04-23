//! Round-trip test against an `ffmpeg`-generated AC-3 fixture.
//!
//! The fixture is a 0.5-second 440 Hz sine tone encoded as 48 kHz
//! stereo AC-3 at 192 kbps. We verify:
//!
//! - The elementary stream is a clean concatenation of syncframes.
//! - Every frame's syncinfo parses and declares the expected sample
//!   rate and frame length.
//! - Every frame's BSI parses and reports 2 channels / no LFE.
//! - The decoder accepts the packets and emits audio frames of the
//!   expected sample count and channel layout.
//!
//! DSP correctness (i.e. that the output approximates a 440 Hz sine)
//! is deferred until the mantissa/IMDCT pipeline lands — the
//! placeholder decoder currently outputs silence.

use oxideav_ac3::{bsi, decoder::SAMPLES_PER_FRAME, syncinfo};
use oxideav_codec::CodecRegistry;
use oxideav_core::{CodecId, CodecParameters, Error, Frame, Packet, TimeBase};

const FIXTURE: &[u8] = include_bytes!("fixtures/sine440_stereo.ac3");

#[test]
fn fixture_is_pure_ac3_syncframes() {
    let mut offset = 0;
    let mut frames = 0;
    while offset < FIXTURE.len() {
        let si = syncinfo::parse(&FIXTURE[offset..]).unwrap_or_else(|e| {
            panic!("syncinfo parse at offset {offset} failed: {e:?}")
        });
        assert_eq!(si.sample_rate, 48_000);
        assert_eq!(si.frame_length, 768, "192 kbps @ 48 kHz ⇒ 768-byte frames");
        assert_eq!(si.frmsizecod, 20);

        let b = bsi::parse(&FIXTURE[offset + 5..]).unwrap();
        assert_eq!(b.acmod, 2, "stereo 2/0");
        assert_eq!(b.nfchans, 2);
        assert!(!b.lfeon);
        assert_eq!(b.nchans, 2);
        assert_eq!(b.bsid, 8);

        offset += si.frame_length as usize;
        frames += 1;
    }
    assert_eq!(frames, FIXTURE.len() / 768);
    assert!(frames >= 10, "short fixture? got {frames} frames");
}

#[test]
fn decoder_produces_frames_of_correct_shape() {
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register(&mut reg);
    let params = CodecParameters::audio(CodecId::new("ac3"));
    let mut dec = reg.make_decoder(&params).expect("make_decoder");

    let mut offset = 0;
    let mut produced = 0;
    while offset < FIXTURE.len() {
        let si = syncinfo::parse(&FIXTURE[offset..]).unwrap();
        let flen = si.frame_length as usize;
        let packet_data = FIXTURE[offset..offset + flen].to_vec();
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), packet_data)
            .with_pts(produced as i64 * SAMPLES_PER_FRAME as i64);
        dec.send_packet(&pkt).unwrap();
        match dec.receive_frame() {
            Ok(Frame::Audio(a)) => {
                assert_eq!(a.channels, 2);
                assert_eq!(a.sample_rate, 48_000);
                assert_eq!(a.samples, SAMPLES_PER_FRAME);
                produced += 1;
            }
            Ok(other) => panic!("unexpected frame kind: {other:?}"),
            Err(Error::NeedMore) => panic!("decoder asked for more on a full packet"),
            Err(e) => panic!("decode error: {e:?}"),
        }
        offset += flen;
    }
    assert!(produced >= 10);
}
