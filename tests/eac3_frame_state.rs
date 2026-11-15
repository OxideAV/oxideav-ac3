//! What an E-AC-3 decoder carries from one syncframe to the next.
//!
//! Annex E restarts every strategy at block 0 of a syncframe (`cplstre`,
//! `spxstre`, `rematstr`, `snroffste`, `cplleake` and the coordinate
//! "first" states are implicit there; exponents, bit-allocation
//! parameters and delta bit allocation may not be reused across the
//! boundary), so a syncframe decodes without any side information from
//! the frame before it. What does cross the boundary is signal history:
//! the overlap tail of the last block, the enhanced-coupling carrier of
//! the last block (§3.5.5.1) and the PCM the transient pre-noise
//! synthesis copies from (§3.7.2).
//!
//! The tests decode each stream twice — from its first frame, and from
//! a later frame on a fresh decoder (a receiver tuning in mid-stream) —
//! and require the two decodes to agree sample for sample once that
//! signal history has been rebuilt. A decoder that let any parsed state
//! leak across frames (or needed it) would not agree. Streams carry
//! `dither = 0` and no noise-blended spectral extension so the decode
//! is deterministic.

mod common;

use common::es::{decode_packets, encode_packets, psnr, syncframes, test_pcm};
use oxideav_ac3::eac3::Eac3DecoderState;

struct Case {
    name: &'static str,
    channels: usize,
    bit_rate: u64,
    opts: &'static [(&'static str, &'static str)],
    /// Frames a fresh decoder needs before it must agree exactly.
    settle: usize,
    transients: bool,
}

const fn case(
    name: &'static str,
    channels: usize,
    bit_rate: u64,
    opts: &'static [(&'static str, &'static str)],
) -> Case {
    Case {
        name,
        channels,
        bit_rate,
        opts,
        settle: 1,
        transients: false,
    }
}

const CASES: &[Case] = &[
    case("stereo", 2, 192_000, &[]),
    case("mono", 1, 96_000, &[]),
    case("5.1", 6, 448_000, &[]),
    case("7.1 independent + dependent", 8, 768_000, &[]),
    case("standard coupling", 2, 128_000, &[("cpl", "1")]),
    case("standard coupling 5.1", 6, 384_000, &[("cpl", "1")]),
    case(
        "spectral extension",
        2,
        128_000,
        &[("spx", "1"), ("spx_blnd", "31")],
    ),
    case(
        "spectral extension with attenuation",
        2,
        128_000,
        &[("spx", "1"), ("spx_blnd", "31"), ("spx_atten", "9")],
    ),
    case(
        "coupling + spectral extension",
        2,
        128_000,
        &[("cpl", "1"), ("spx", "1"), ("spx_blnd", "31")],
    ),
    case("adaptive hybrid transform", 2, 192_000, &[("aht", "1")]),
    case("adaptive hybrid transform 5.1", 6, 448_000, &[("aht", "1")]),
    case(
        "enhanced coupling",
        2,
        192_000,
        &[("ecpl", "1"), ("ecpl_chaos", "0")],
    ),
    case("three blocks per syncframe", 2, 192_000, &[("blocks", "3")]),
    case("two blocks per syncframe", 2, 192_000, &[("blocks", "2")]),
    case("one block per syncframe", 2, 384_000, &[("blocks", "1")]),
    // Block switching (short transforms on the transient blocks) with
    // the channels coupled.
    Case {
        name: "standard coupling across block switches",
        channels: 2,
        bit_rate: 192_000,
        opts: &[("cpl", "1")],
        settle: 1,
        transients: true,
    },
    Case {
        name: "block switching 5.1",
        channels: 6,
        bit_rate: 448_000,
        opts: &[],
        settle: 1,
        transients: true,
    },
    Case {
        name: "transient pre-noise processing",
        channels: 2,
        bit_rate: 128_000,
        opts: &[("tpnp", "1")],
        // The synthesis buffer of a transient early in a frame reaches
        // into the first block of the frame before it.
        settle: 2,
        transients: true,
    },
];

/// Tones over a noise bed, with a decaying noise burst every 2000
/// samples when `transients`.
fn pcm_for(c: &Case, frames: usize) -> Vec<i16> {
    let n = frames * 1536;
    let mut pcm = test_pcm(c.channels, 48_000, n, 3);
    if c.transients {
        let mut s = 0x9E37_79B9u32;
        for i in 0..n {
            let since = i % 2000;
            if since < 500 {
                s ^= s << 13;
                s ^= s >> 17;
                s ^= s << 5;
                let burst =
                    0.7 * (-(since as f32) / 90.0).exp() * (s as i32 as f32) / (i32::MAX as f32);
                for ch in 0..c.channels {
                    let v = &mut pcm[i * c.channels + ch];
                    *v = (f32::from(*v) * 0.2 + burst * 32767.0).clamp(-32768.0, 32767.0) as i16;
                }
            }
        }
    }
    pcm
}

fn encode(c: &Case, frames: usize) -> Vec<Vec<u8>> {
    let mut opts = vec![("dither", "0")];
    opts.extend_from_slice(c.opts);
    encode_packets(c.channels, 48_000, c.bit_rate, &opts, &pcm_for(c, frames))
}

#[test]
fn a_decoder_joining_mid_stream_agrees_with_one_that_saw_everything() {
    for c in CASES {
        let packets = encode(c, 12);
        let mut full_state = Eac3DecoderState::default();
        let (full, ch) = decode_packets(&mut full_state, &packets);
        assert_eq!(ch, c.channels, "{}", c.name);
        if c.name.contains("block switch") {
            assert!(
                uses_short_blocks(&packets),
                "{}: the stream must switch to short transforms",
                c.name
            );
        }
        assert_eq!(full_state.frames_zero_filled, 0, "{}", c.name);
        let per_frame = full[0].len() / ch;
        // Settling is counted in 1536-sample periods.
        let settle = c.settle * 1536 / per_frame;
        for join in [1usize, packets.len() / 3, packets.len() / 2 + 1] {
            let mut st = Eac3DecoderState::default();
            let (late, _) = decode_packets(&mut st, &packets[join..]);
            assert_eq!(st.frames_zero_filled, 0, "{}: join at {join}", c.name);
            for (i, got) in late.iter().enumerate() {
                let want = &full[join + i];
                if i >= settle {
                    assert!(
                        got == want,
                        "{}: joined at frame {join}, frame {} differs (psnr {:.1} dB)",
                        c.name,
                        join + i,
                        psnr(got, want)
                    );
                }
            }
            // The first frame after the join only lacks the overlap
            // into its first block (enhanced coupling: also the
            // carrier of the block before it).
            if !c.opts.iter().any(|(k, _)| *k == "ecpl") && !c.transients {
                let from = 256 * ch;
                if late[0].len() > from {
                    assert!(
                        late[0][from..] == full[join][from..],
                        "{}: joined at frame {join}, first frame past its first block",
                        c.name
                    );
                }
            }
        }
    }
}

/// §3.7.2 across a frame boundary: a transient in the first block of a
/// syncframe has its pre-noise overwritten with audio that was output
/// with the frame before. In the full-overwrite part of the corrected
/// window every sample equals the output `TC1 + PN − transproclen`
/// samples earlier.
#[test]
fn transient_pre_noise_synthesis_copies_from_the_previous_frame() {
    let c = &CASES[CASES.len() - 1];
    assert!(c.transients);
    let packets = encode(c, 12);
    let (frames, ch) = decode_packets(&mut Eac3DecoderState::default(), &packets);
    let pcm: Vec<i16> = frames.concat();
    let mut checked = 0usize;
    for (n, p) in packets.iter().enumerate().skip(1) {
        let frame = &p[syncframes(p)[0].clone()];
        let mut br = oxideav_core::bits::BitReader::new(&frame[2..]);
        let bsi = oxideav_ac3::eac3::bsi::parse_with(&mut br).unwrap();
        let af = oxideav_ac3::eac3::audfrm::parse_with(&mut br, &bsi).unwrap();
        if !af.transproce {
            continue;
        }
        for c in 0..ch {
            if !af.chintransproc[c] {
                continue;
            }
            let t = 4 * af.transprocloc[c] as isize;
            let pn = t % 256 + 256;
            let len = af.transproclen[c] as isize;
            let tot = pn + len + 256;
            let shift = 256 + pn - len;
            let base = (n * 1536) as isize;
            for s in 256..tot - 128 {
                let d = t - tot + s;
                // Only destinations in this frame whose source lies in
                // the frame before it.
                if d < 0 || d - shift >= 0 {
                    continue;
                }
                let at = |i: isize| pcm[((base + i) as usize) * ch + c];
                assert_eq!(
                    at(d),
                    at(d - shift),
                    "frame {n} channel {c}: sample {d} is a copy of sample {}",
                    d - shift
                );
                checked += 1;
            }
        }
    }
    assert!(
        checked > 200,
        "the stream must place transients early in a frame ({checked} samples checked)"
    );
}

/// Does any block of any channel of the stream signal `blksw = 1`?
/// (Reads the first block's flags of every syncframe: they directly
/// follow the audfrm when `blkswe` is set.)
fn uses_short_blocks(packets: &[Vec<u8>]) -> bool {
    packets.iter().any(|p| {
        syncframes(p).into_iter().any(|r| {
            let frame = &p[r];
            let mut br = oxideav_core::bits::BitReader::new(&frame[2..]);
            let bsi = oxideav_ac3::eac3::bsi::parse_with(&mut br).unwrap();
            let af = oxideav_ac3::eac3::audfrm::parse_with(&mut br, &bsi).unwrap();
            af.blkswe && (0..bsi.nfchans).any(|_| br.read_u32(1).unwrap() != 0)
        })
    })
}
