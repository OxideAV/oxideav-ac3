#![no_main]

//! JOC/OAMD metadata surface (TS 103 420): feeds raw bytes to the
//! EC-3 Extension Type A signal parser, the EMDF container walk
//! (payload-config validation, OAMD + JOC payload extraction, sparse /
//! dense Huffman matrix decode), the standalone OAMD object-element
//! decoder, and — whenever a container parses — the QMF object
//! reconstruction + object renderer (stereo and one selector-picked
//! multichannel layout: panner, bed objects, zone constraints, channel
//! lock, extent) with PCM synthesized from the input tail. All of these paths receive at most one declared 511-byte
//! skip field in real streams, so the harness asserts panic-freedom
//! and bounded allocation on hostile bytes well past that envelope.

//!
//! No JOC bitstream exists to seed the corpus, and a random byte string
//! almost never forms a valid EMDF container, so the harness also
//! builds object metadata directly: the input drives the program
//! assignment (dynamic / bed), every object's position, size, zone
//! constraints, elevation and snap flags and the ramp, optionally
//! corrupts the resulting OAMD payload, and renders two frames through
//! the selector-picked layout.

use libfuzzer_sys::fuzz_target;
use oxideav_ac3::eac3::JocLayout;
use oxideav_ac3::eac3::joc::{JocMetadata, parse_ec3_extension_type_a, parse_joc_emdf};
use oxideav_ac3::eac3::joc_renderer::JocRenderer;
use oxideav_ac3::eac3::oamd::OamdDecoder;
use oxideav_ac3::eac3::oamd::synth::{TestObject, TestProgram, payload};

fuzz_target!(|data: &[u8]| {
    let Some((&selector, body)) = data.split_first() else {
        return;
    };

    // The Extension Type A parser on raw bytes.
    let _ = parse_ec3_extension_type_a(body);

    // A standalone OAMD walk (the renderer only reaches it behind a
    // fully valid EMDF container, so cover it directly too), run on a
    // persistent decoder to exercise the reuse/differential state.
    let mut oamd = OamdDecoder::default();
    let _ = oamd.decode(body);
    let _ = oamd.decode(body);

    structured_render(selector, body);

    // The EMDF container walk, with every valid complexity index.
    let signal = match parse_ec3_extension_type_a(&[1, selector % 16 + 1]) {
        Ok(Some(signal)) => signal,
        _ => return,
    };
    if let Ok(metadata) = parse_joc_emdf(body, signal) {
        // One 3/2+LFE E-AC-3 frame of PCM seeded from the input so the
        // QMF analysis/synthesis and OAMD gain ramps see signal.
        let mut pcm = vec![0.0f32; 1536 * 6];
        for (index, value) in pcm.iter_mut().enumerate() {
            let byte = body[index % body.len()];
            *value = (f32::from(byte) - 127.5) / 127.5;
        }
        let mut renderer = JocRenderer::default();
        // Two passes: an initialization frame and a sequence follow-up
        // (the second pass exercises the discontinuity reset).
        let _ = renderer.render_stereo(&metadata, &pcm, 6, 7, true);
        let _ = renderer.render_stereo(&metadata, &pcm, 6, 7, true);

        let layout = JocLayout::ALL[usize::from(selector >> 4) % JocLayout::ALL.len()];
        let channels = usize::from(layout.channel_count());
        let mut renderer = JocRenderer::with_layout(layout);
        for _ in 0..2 {
            if let Ok(rendered) = renderer.render(&metadata, &pcm, 6, 7, true) {
                assert_eq!(rendered.len(), 1536 * channels);
                assert!(rendered.iter().all(|sample| sample.is_finite()));
            }
        }
        let bed = renderer.render_channel_bed(&pcm, 6, 7, true, &[]);
        assert_eq!(bed.len(), 1536 * channels);
    }
});

/// Structure-aware leg: synthesize OAMD from the input and render it.
fn structured_render(selector: u8, body: &[u8]) {
    let [program, ramp, flips, rest @ ..] = body else {
        return;
    };
    let objects: Vec<TestObject> = rest
        .as_chunks::<5>()
        .0
        .iter()
        .take(15)
        .map(|bytes| TestObject {
            position: [
                i16::from(bytes[0] & 63),
                i16::from(bytes[1] & 63),
                i16::from(bytes[2] & 31) - 15,
            ],
            zone_constraints: u32::from(bytes[3] & 7),
            elevation: bytes[3] & 8 != 0,
            size: u32::from(bytes[4] & 31),
            snap: bytes[4] & 32 != 0,
        })
        .collect();
    let (assignment, fixed) = if program & 1 == 0 {
        (TestProgram::Dynamic { lfe: true }, 1)
    } else {
        // A standard bed: the low bits pick the speaker pairs.
        let mask = u32::from(*program >> 1) << 1 | 0b10_0000_0000;
        let mut count = 0;
        for (bit, speakers) in [1, 2, 2, 2, 2, 2, 2, 1, 1, 2].into_iter().enumerate() {
            if mask & (1 << bit) != 0 {
                count += speakers;
            }
        }
        (
            TestProgram::Bed {
                assignment: mask,
                objects: objects.len(),
            },
            count,
        )
    };
    let mut oamd = payload(assignment, fixed, &objects, u32::from(*ramp & 3));
    if oamd.is_empty() {
        return;
    }
    // Optional corruption of the well-formed payload.
    for flip in 0..(flips & 3) {
        let bit = (usize::from(*flips) * 31 + usize::from(flip) * 97) % (oamd.len() * 8);
        oamd[bit / 8] ^= 1 << (bit % 8);
    }
    let layout = JocLayout::ALL[usize::from(selector >> 4) % JocLayout::ALL.len()];
    let channels = usize::from(layout.channel_count());
    // One JOC object per non-LFE OAMD object (the bed mask may hold up
    // to two LFE labels).
    let joc_objects = (fixed + objects.len()).saturating_sub(1).clamp(1, 16);
    let routing: Vec<usize> = (0..joc_objects).collect();
    let mut pcm = vec![0.0f32; 1536 * 6];
    for (index, value) in pcm.iter_mut().enumerate() {
        *value = (f32::from(body[index % body.len()]) - 127.5) / 127.5;
    }
    let mut renderer = JocRenderer::with_layout(layout);
    for sequence in 0..2 {
        let metadata = JocMetadata::synthetic(sequence, oamd.clone(), &routing);
        if let Ok(rendered) = renderer.render(&metadata, &pcm, 6, 7, true) {
            assert_eq!(rendered.len(), 1536 * channels);
            assert!(rendered.iter().all(|sample| sample.is_finite()));
        }
    }
}
