//! E-AC-3 syncframe decoder — round 1.
//!
//! Round-1 path (this commit):
//!
//! 1. Verify the 16-bit syncword (`0x0B77`).
//! 2. Parse the [`super::bsi`] BSI — channel layout, sample rate,
//!    frame size, dialnorm, etc.
//! 3. Parse the [`super::audfrm`] audio-frame element — strategy
//!    flags only (no DSP yet).
//! 4. Emit `bsi.num_blocks × 256 × nchans` interleaved S16 zeros.
//!
//! This is enough to unblock the corpus tests at
//! `tests/docs_corpus.rs`, which previously bailed at
//! `bsi.rs:152-158` ("bsid > 10"). PCM output is silent, so the
//! per-channel match% / PSNR numbers will be poor — but the test
//! framework now runs end-to-end and a numeric baseline exists for
//! round 2 (decouple + IMDCT) to improve against.
//!
//! The dependent-substream recombination case
//! (`eac3-from-ac3-bitstream-recombination`) is detected and noted
//! but only the independent substream is emitted. Dependent
//! substreams are still parsed (BSI + audfrm) so the decoder's
//! output PCM is the right length — round 2 will add the per-channel
//! `chanmap`-driven recombination into the indep program.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use super::audfrm::{self, AudFrm};
use super::bsi::{self, Bsi as Eac3Bsi, StreamType};

/// E-AC-3 syncword — same value as base AC-3 (§E.2.2.1).
pub const SYNCWORD: u16 = 0x0B77;

/// Per-decoder state that persists across packets. Round-1 carries
/// no state (every syncframe is decoded independently and emits
/// silent PCM), but the struct exists so round-2 can attach a
/// per-channel overlap-add delay line + the current independent
/// substream's program PCM that dependent substreams will fold in.
#[derive(Default, Clone, Debug)]
pub struct Eac3DecoderState {
    /// Last successfully-decoded indep substream parameters. Used by
    /// dependent substreams to know how many channels to extend.
    pub last_indep: Option<IndepProgramShape>,
}

/// Snapshot of the most recent independent substream's output shape
/// — channels, sample rate, samples per frame.
#[derive(Clone, Debug)]
pub struct IndepProgramShape {
    pub nchans: u16,
    pub sample_rate: u32,
    pub samples_per_frame: u32,
}

/// Result of decoding one E-AC-3 packet.
#[derive(Clone, Debug)]
pub struct DecodedFrame {
    pub sample_rate: u32,
    pub channels: u16,
    /// PCM samples per channel (= `bsi.num_blocks * 256`).
    pub samples: u32,
    /// Interleaved S16LE bytes — `samples * channels * 2` total.
    pub pcm_s16le: Vec<u8>,
}

/// Decode one or more concatenated E-AC-3 syncframes contained in a
/// single packet (a "transport syncframe" pair: indep + dep).
///
/// The packet must begin with a 16-bit `0x0B77` syncword. Subsequent
/// syncframes are located via the BSI's `frame_bytes` field. Returns
/// the **independent** substream's program PCM (silent in round 1);
/// dependent substreams are parsed and skipped.
pub fn decode_eac3_packet(state: &mut Eac3DecoderState, data: &[u8]) -> Result<DecodedFrame> {
    if data.len() < 4 {
        return Err(Error::invalid("eac3: packet too short for syncinfo"));
    }
    let mut indep_pcm: Option<DecodedFrame> = None;
    let mut off = 0usize;
    while off + 4 <= data.len() {
        // §E.2.2.1 — syncword.
        let sync = u16::from_be_bytes([data[off], data[off + 1]]);
        if sync != SYNCWORD {
            return Err(Error::invalid(format!(
                "eac3: bad syncword 0x{sync:04X} at offset {off} (expected 0x0B77)"
            )));
        }
        // BSI starts at byte off+2.
        let bsi_data = &data[off + 2..];
        let mut br = BitReader::new(bsi_data);
        let bsi = bsi::parse_with(&mut br)?;
        let frame_bytes = bsi.frame_bytes as usize;
        if off + frame_bytes > data.len() {
            return Err(Error::invalid(format!(
                "eac3: syncframe at offset {off} claims {frame_bytes} bytes but packet has only {}",
                data.len() - off
            )));
        }

        // Audfrm follows BSI in the same bit cursor.
        let audfrm = audfrm::parse_with(&mut br, &bsi)?;

        match bsi.strmtyp {
            StreamType::Independent | StreamType::Ac3Convert => {
                let pcm = build_silent_indep(&bsi, &audfrm)?;
                state.last_indep = Some(IndepProgramShape {
                    nchans: pcm.channels,
                    sample_rate: pcm.sample_rate,
                    samples_per_frame: pcm.samples,
                });
                indep_pcm = Some(pcm);
            }
            StreamType::Dependent => {
                // Round 1: parsed but skipped. Round 2 will fold the
                // dep substream's `chanmap` channels into the indep
                // PCM when one exists in `state.last_indep`.
            }
            StreamType::Reserved => {
                return Err(Error::invalid("eac3: strmtyp '11' is reserved"));
            }
        }

        off += frame_bytes;
    }

    indep_pcm.ok_or_else(|| {
        Error::invalid(
            "eac3: packet contains no independent substream (only dependent or ac3-convert frames)",
        )
    })
}

/// Build a silent S16 PCM buffer of the right shape for one
/// independent substream. Round-1 placeholder for the real
/// audblk-decoding pipeline.
fn build_silent_indep(bsi: &Eac3Bsi, _audfrm: &AudFrm) -> Result<DecodedFrame> {
    // Annex E does not change the §2.2 transform: each audio block
    // produces 256 PCM samples per channel. Total per syncframe =
    // num_blocks × 256.
    let samples = bsi.num_blocks as u32 * 256;
    // Interleaved S16LE: 2 bytes per sample, channels per audio
    // sample.
    let nchans = bsi.nchans as usize;
    let total_bytes = samples as usize * nchans * 2;
    let pcm_s16le = vec![0u8; total_bytes];
    Ok(DecodedFrame {
        sample_rate: bsi.sample_rate,
        channels: bsi.nchans as u16,
        samples,
        pcm_s16le,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    /// Build a minimal valid E-AC-3 syncframe of `frame_bytes` bytes:
    /// syncword + the BSI from `bsi_bits` + zero-padding through the
    /// payload. Used by parser smoke tests; not bit-stream conformant
    /// past the BSI/audfrm boundary.
    fn build_syncframe(frame_bytes: usize, bsi_bits: &[(u32, u32)]) -> Vec<u8> {
        let mut bw = BitWriter::with_capacity(frame_bytes);
        bw.write_u32(SYNCWORD as u32, 16);
        for &(n, v) in bsi_bits {
            bw.write_u32(v, n);
        }
        let mut buf = bw.into_bytes();
        if buf.len() < frame_bytes {
            buf.resize(frame_bytes, 0);
        } else {
            buf.truncate(frame_bytes);
        }
        buf
    }

    /// Build a stereo 6-block 768-byte indep syncframe + a stripped
    /// audfrm with zero strategy flags — enough that the round-1
    /// decoder produces a silent buffer of the right shape.
    fn stereo_768_indep() -> Vec<u8> {
        let bsi_bits: &[(u32, u32)] = &[
            (2, 0),    // strmtyp = indep
            (3, 0),    // substreamid
            (11, 383), // frmsiz → 768 bytes
            (2, 0),    // fscod = 48 kHz
            (2, 3),    // numblkscod = 3 → 6 blocks
            (3, 2),    // acmod = 2 (2/0)
            (1, 0),    // lfeon
            (5, 16),   // bsid
            (5, 27),   // dialnorm
            (1, 0),    // compre
            (1, 0),    // mixmdate
            (1, 0),    // infomdate
            (1, 0),    // addbsie
            // ----- audfrm -----
            (1, 1), // expstre
            (1, 0), // ahte
            (2, 0), // snroffststr
            (1, 0), // transproce
            (1, 1), // blkswe
            (1, 1), // dithflage
            (1, 1), // bamode
            (1, 0), // frmfgaincode
            (1, 1), // dbaflde
            (1, 1), // skipflde
            (1, 0), // spxattene
            // acmod>1 → cplinu[0] (1) + 5 × cplstre (1 each)
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            (1, 0),
            // expstre=1 → 6 blocks × 2 chans × chexpstr 2 bits each (no cpl).
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            (2, 0),
            // strmtyp=0 + numblkscod=3 → convexpstre implicit 1
            // → 2 × convexpstr (5 bits)
            (5, 0),
            (5, 0),
            // snroffststr=0 → frmcsnroffst (6) + frmfsnroffst (4)
            (6, 15),
            (4, 0),
            // num_blocks > 1 → blkstrtinfoe (1, 0)
            (1, 0),
        ];
        build_syncframe(768, bsi_bits)
    }

    #[test]
    fn decode_silent_indep_smoke() {
        let pkt = stereo_768_indep();
        let mut st = Eac3DecoderState::default();
        let frm = decode_eac3_packet(&mut st, &pkt).unwrap();
        assert_eq!(frm.sample_rate, 48_000);
        assert_eq!(frm.channels, 2);
        assert_eq!(frm.samples, 6 * 256);
        assert_eq!(frm.pcm_s16le.len(), (6 * 256) * 2 * 2);
        assert!(frm.pcm_s16le.iter().all(|&b| b == 0));
    }

    #[test]
    fn rejects_bad_syncword() {
        let mut data = stereo_768_indep();
        data[0] = 0xFF;
        let mut st = Eac3DecoderState::default();
        assert!(decode_eac3_packet(&mut st, &data).is_err());
    }
}
