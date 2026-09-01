#![no_main]

//! JOC/OAMD metadata surface (TS 103 420): feeds raw bytes to the
//! EC-3 Extension Type A signal parser, the EMDF container walk
//! (payload-config validation, OAMD + JOC payload extraction, sparse /
//! dense Huffman matrix decode), the standalone OAMD object-element
//! decoder, and — whenever a container parses — the QMF object
//! reconstruction + stereo renderer with PCM synthesized from the
//! input tail. All of these paths receive at most one declared 511-byte
//! skip field in real streams, so the harness asserts panic-freedom
//! and bounded allocation on hostile bytes well past that envelope.

use libfuzzer_sys::fuzz_target;
use oxideav_ac3::eac3::joc::{parse_ec3_extension_type_a, parse_joc_emdf};
use oxideav_ac3::eac3::joc_renderer::JocRenderer;
use oxideav_ac3::eac3::oamd::OamdDecoder;

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
    }
});
