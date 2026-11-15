//! Annex E §E.3.8 substream handling: program selection among several
//! independent substreams, dependent substreams associated with the
//! independent frame they follow, and one history per substream.
//!
//! The streams are built from our encoder's output by re-labelling
//! `substreamid` / `chanmap` (CRC recomputed) and interleaving the
//! syncframes in the §E.2.3.1.2 order.

mod common;

use common::es::{
    channel, decode_packets, encode_packets, fix_crc, set_bits, strmtyp, syncframes, test_pcm,
    with_substreamid,
};
use oxideav_ac3::eac3::Eac3DecoderState;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

const RATE: u32 = 48_000;
const FRAMES: usize = 10;

fn stereo_program(seed: u32) -> Vec<Vec<u8>> {
    let pcm = test_pcm(2, RATE, FRAMES * 1536, seed);
    encode_packets(2, RATE, 192_000, &[], &pcm)
}

fn seven_one_program(seed: u32) -> Vec<Vec<u8>> {
    let pcm = test_pcm(8, RATE, FRAMES * 1536, seed);
    encode_packets(8, RATE, 768_000, &[], &pcm)
}

/// Every frame of `packets` re-labelled as program `id`: independent
/// frames take the id, dependent frames keep theirs (they are numbered
/// per program, §E.2.3.1.2).
fn as_program(packets: &[Vec<u8>], id: u8) -> Vec<Vec<u8>> {
    packets
        .iter()
        .map(|p| {
            let mut out = Vec::with_capacity(p.len());
            for r in syncframes(p) {
                let f = &p[r];
                if strmtyp(f) == 1 {
                    out.extend_from_slice(f);
                } else {
                    out.extend(with_substreamid(f, id));
                }
            }
            out
        })
        .collect()
}

fn interleave(a: &[Vec<u8>], b: &[Vec<u8>]) -> Vec<Vec<u8>> {
    a.iter()
        .zip(b)
        .map(|(x, y)| [x.as_slice(), y.as_slice()].concat())
        .collect()
}

/// §E.3.8.3: "The default program selection should always be Program
/// 1" — a second independent substream in the same access unit is
/// skipped and leaves the decode of program 0 untouched.
#[test]
fn default_selection_decodes_program_zero_only() {
    let p0 = stereo_program(1);
    let p1 = stereo_program(2);
    let (want, ch) = decode_packets(&mut Eac3DecoderState::default(), &p0);
    let mixed = interleave(&p0, &as_program(&p1, 1));
    let mut st = Eac3DecoderState::default();
    let (got, got_ch) = decode_packets(&mut st, &mixed);
    assert_eq!(got_ch, ch);
    assert_eq!(
        got, want,
        "program 0 must decode as if program 1 were absent"
    );
    assert_eq!(st.frames_zero_filled, 0);
}

/// §E.3.8.3: selecting another independent substream id decodes that
/// program, with program 0 skipped.
#[test]
fn selected_program_decodes_alone() {
    let p0 = stereo_program(1);
    let p1 = stereo_program(2);
    let (want, _) = decode_packets(&mut Eac3DecoderState::default(), &p1);
    let mixed = interleave(&p0, &as_program(&p1, 1));
    let mut st = Eac3DecoderState::default();
    st.set_program(1);
    let (got, _) = decode_packets(&mut st, &mixed);
    assert_eq!(
        got, want,
        "program 1 must decode as if program 0 were absent"
    );
    assert_eq!(st.frames_zero_filled, 0);
}

/// §E.3.8.4: a dependent substream belongs to the independent
/// substream it follows. Program 1's dependent substream must not be
/// spliced into program 0, and program 0's own dependent substream
/// still is.
#[test]
fn dependent_substreams_stay_with_their_program() {
    let p0 = seven_one_program(3);
    let p1 = seven_one_program(4);
    let (want, ch) = decode_packets(&mut Eac3DecoderState::default(), &p0);
    assert_eq!(ch, 8);
    let mixed = interleave(&p0, &as_program(&p1, 1));
    let (got, got_ch) = decode_packets(&mut Eac3DecoderState::default(), &mixed);
    assert_eq!(
        got_ch, 8,
        "program 1's dependent channels are not part of program 0"
    );
    assert_eq!(got, want);

    let (want1, _) = decode_packets(&mut Eac3DecoderState::default(), &p1);
    let mut st = Eac3DecoderState::default();
    st.set_program(1);
    let (got1, got1_ch) = decode_packets(&mut st, &mixed);
    assert_eq!(got1_ch, 8);
    assert_eq!(got1, want1);
}

/// §E.2.3.1.2: an independent substream may carry up to eight
/// dependent substreams. Each keeps its own exponent / overlap
/// history: a second dependent substream (id 1, mapped to the Lc/Rc
/// pair) decodes exactly as it does as the only dependent substream
/// of its source program.
#[test]
fn two_dependent_substreams_keep_separate_history() {
    let a = seven_one_program(5);
    let b = seven_one_program(6);
    let (want_a, _) = decode_packets(&mut Eac3DecoderState::default(), &a);
    let (want_b, _) = decode_packets(&mut Eac3DecoderState::default(), &b);

    // Access unit: A's independent frame, A's dependent frame (id 0,
    // Lrs/Rrs), then B's dependent frame re-labelled id 1 on the
    // Lc/Rc pair (Table E2.5 bit 5).
    let packets: Vec<Vec<u8>> = a
        .iter()
        .zip(&b)
        .map(|(pa, pb)| {
            let mut out = pa.clone();
            let dep = syncframes(pb)
                .into_iter()
                .map(|r| &pb[r])
                .find(|f| strmtyp(f) == 1)
                .expect("7.1 packet carries a dependent frame");
            let mut dep = dep.to_vec();
            // compre (bit 50) clear, so chanmape sits at bit 51 and
            // chanmap at bits 52..68.
            assert_eq!(dep[6] & 0x20, 0, "dependent BSI carries no compr word");
            assert_ne!(dep[6] & 0x10, 0, "dependent BSI carries a chanmap");
            set_bits(&mut dep, 18, 3, 1);
            set_bits(&mut dep, 52, 16, 1 << (15 - 5));
            fix_crc(&mut dep);
            out.extend(dep);
            out
        })
        .collect();

    let mut st = Eac3DecoderState::default();
    let (got, ch) = decode_packets(&mut st, &packets);
    assert_eq!(ch, 10, "5.1 + Lrs/Rrs + Lc/Rc");
    assert_eq!(st.frames_zero_filled, 0);
    for (i, frame) in got.iter().enumerate() {
        for c in 0..8 {
            assert_eq!(
                channel(frame, 10, c),
                channel(&want_a[i], 8, c),
                "frame {i} channel {c} (program A)"
            );
        }
        for c in 0..2 {
            assert_eq!(
                channel(frame, 10, 8 + c),
                channel(&want_b[i], 8, 6 + c),
                "frame {i} second dependent substream channel {c}"
            );
        }
    }
}

/// An access unit that carries only frames of other programs is an
/// error for the selected program, not silence from a foreign one.
#[test]
fn packet_without_the_selected_program_is_rejected() {
    let p1 = as_program(&stereo_program(7), 1);
    let mut st = Eac3DecoderState::default();
    assert!(oxideav_ac3::eac3::decode_eac3_packet(&mut st, &p1[0]).is_err());
}

/// The dependent frame of each packet of `packets`, re-mapped by
/// `chanmap` (Table E2.5 bit weights), appended to the matching packet
/// of `onto` in place of its own dependent frame.
fn with_dependent_from(onto: &[Vec<u8>], packets: &[Vec<u8>], chanmap: u16) -> Vec<Vec<u8>> {
    onto.iter()
        .zip(packets)
        .map(|(a, b)| {
            let indep = syncframes(a)
                .into_iter()
                .map(|r| &a[r])
                .find(|f| strmtyp(f) != 1)
                .unwrap();
            let dep = syncframes(b)
                .into_iter()
                .map(|r| &b[r])
                .find(|f| strmtyp(f) == 1)
                .unwrap();
            let mut dep = dep.to_vec();
            assert_eq!(dep[6] & 0x30, 0x10, "chanmap present, no compr word");
            set_bits(&mut dep, 52, 16, u32::from(chanmap));
            fix_crc(&mut dep);
            [indep, &dep[..]].concat()
        })
        .collect()
}

const LS_RS: u16 = (1 << (15 - 3)) | (1 << (15 - 4));

/// §E.3.8.2: dependent-substream channels at a location the
/// independent substream already carries REPLACE that channel. A
/// dependent substream mapped to Ls / Rs leaves the program at six
/// channels with its surrounds taken from the dependent substream —
/// in the 16-bit output too.
#[test]
fn replacing_dependent_channels_reach_the_output() {
    let a = seven_one_program(8);
    let b = seven_one_program(9);
    let (want_a, _) = decode_packets(&mut Eac3DecoderState::default(), &a);
    let (want_b, _) = decode_packets(&mut Eac3DecoderState::default(), &b);
    let packets = with_dependent_from(&a, &b, LS_RS);
    let mut st = Eac3DecoderState::default();
    let (got, ch) = decode_packets(&mut st, &packets);
    assert_eq!(ch, 6, "replacement does not grow the program");
    assert_eq!(st.frames_zero_filled, 0);
    for (i, frame) in got.iter().enumerate() {
        // Bit-stream order L C R Ls Rs LFE.
        for c in [0, 1, 2, 5] {
            assert_eq!(
                channel(frame, 6, c),
                channel(&want_a[i], 8, c),
                "frame {i} ch {c}"
            );
        }
        for c in 0..2 {
            assert_eq!(
                channel(frame, 6, 3 + c),
                channel(&want_b[i], 8, 6 + c),
                "frame {i}: surround {c} comes from the dependent substream"
            );
        }
    }
}

fn registry_decode(packets: &[Vec<u8>], channels: Option<u16>) -> Vec<Vec<u8>> {
    let mut params = CodecParameters::audio(CodecId::new("eac3"));
    params.channels = channels;
    let mut dec = oxideav_ac3::decoder::make_eac3_decoder(&params).unwrap();
    packets
        .iter()
        .map(|p| {
            dec.send_packet(&Packet::new(0, TimeBase::new(1, 48_000), p.clone()))
                .unwrap();
            match dec.receive_frame().unwrap() {
                Frame::Audio(af) => af.data.into_iter().next().unwrap(),
                _ => panic!("audio frame expected"),
            }
        })
        .collect()
}

/// §E.3.8.2: "Decoders reproducing 5.1 or fewer channels from a
/// program containing greater than 5.1 channels shall decode only
/// independent substream 0 and skip all associated dependent
/// substreams." A stereo or 5.1 target must not see the surrounds a
/// dependent substream substitutes for the 5.1 program's.
#[test]
fn small_targets_decode_the_independent_substream_only() {
    let a = seven_one_program(8);
    let b = seven_one_program(9);
    let with_dep = with_dependent_from(&a, &b, LS_RS);
    let indep_only: Vec<Vec<u8>> = a
        .iter()
        .map(|p| {
            let r = syncframes(p)
                .into_iter()
                .find(|r| strmtyp(&p[r.clone()]) != 1)
                .unwrap();
            p[r].to_vec()
        })
        .collect();
    for target in [1u16, 2, 6] {
        assert!(
            registry_decode(&with_dep, Some(target)) == registry_decode(&indep_only, Some(target)),
            "{target}-channel target ignores dependent substreams"
        );
    }
    // No target: the complete program (here with replaced surrounds).
    assert!(registry_decode(&with_dep, None) != registry_decode(&indep_only, None));
}

/// §E.3.8.5: when the complete program is reproduced, "the compr and
/// dynrng metadata words carried in independent substream 0 shall be
/// ignored. The decoder shall instead use the compr and dynrng
/// metadata words carried by the associated dependent substream" —
/// for every substream of the program.
#[test]
fn program_dynamic_range_words_come_from_the_dependent_substream() {
    let pcm = test_pcm(8, RATE, FRAMES * 1536, 12);
    // Same audio, same budget; only the per-block dynrng word differs
    // (0x00 = unity, 0xE0 = one 6 dB step down).
    let unity = encode_packets(8, RATE, 768_000, &[("dynrng", "0")], &pcm);
    let cut = encode_packets(8, RATE, 768_000, &[("dynrng", "224")], &pcm);
    let (want_unity, _) = decode_packets(&mut Eac3DecoderState::default(), &unity);
    let (want_cut, _) = decode_packets(&mut Eac3DecoderState::default(), &cut);
    assert_ne!(
        want_unity, want_cut,
        "the two words must give different gains"
    );

    // Independent frame asks for the cut, dependent frame for unity.
    let mixed = with_dependent_from(&cut, &unity, 1 << (15 - 6));
    let (got, ch) = decode_packets(&mut Eac3DecoderState::default(), &mixed);
    assert_eq!(ch, 8);
    assert_eq!(
        got, want_unity,
        "the dependent substream's word governs all eight channels"
    );

    // The 5.1 program on its own keeps the independent substream's word.
    let mut st = Eac3DecoderState::default();
    st.set_dependent_substreams(false);
    let (got51, ch51) = decode_packets(&mut st, &mixed);
    assert_eq!(ch51, 6);
    for (i, frame) in got51.iter().enumerate() {
        for c in 0..6 {
            assert_eq!(
                channel(frame, 6, c),
                channel(&want_cut[i], 8, c),
                "frame {i} ch {c}"
            );
        }
    }
}
