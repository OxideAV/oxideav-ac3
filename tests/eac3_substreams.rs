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
