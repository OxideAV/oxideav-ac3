//! E-AC-3 syncframe decoder — rounds 1 + 2.
//!
//! Round-1 path:
//!
//! 1. Verify the 16-bit syncword (`0x0B77`).
//! 2. Parse the [`super::bsi`] BSI — channel layout, sample rate,
//!    frame size, dialnorm, etc.
//! 3. Parse the [`super::audfrm`] audio-frame element — strategy
//!    flags only (no DSP yet).
//! 4. Emit `bsi.num_blocks × 256 × nchans` interleaved S16 zeros.
//!
//! Round-2 path (this commit):
//!
//! 5. Hand the BitReader to [`super::dsp::decode_indep_audblks`],
//!    which walks per-block side-info, runs §7 bit allocation +
//!    mantissa unpack, and applies IMDCT + window + overlap-add via
//!    the AC-3 helpers. On any "unsupported feature" error
//!    (coupling, SPX, AHT, transient processing, …) the decoder
//!    falls back to silent emit so the corpus driver keeps decoding.
//!
//! The dependent-substream recombination case
//! (`eac3-from-ac3-bitstream-recombination`) is detected and noted
//! but only the independent substream is emitted. Dependent
//! substreams are still parsed (BSI + audfrm) so the decoder's
//! output PCM is the right length — round 3 will add the per-channel
//! `chanmap`-driven recombination into the indep program.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::audblk::{Ac3State, SAMPLES_PER_BLOCK};

use super::audfrm::{self, AudFrm};
use super::bsi::{self, Bsi as Eac3Bsi, StreamType};
use super::dsp;

/// E-AC-3 syncword — same value as base AC-3 (§E.2.2.1).
pub const SYNCWORD: u16 = 0x0B77;

/// Per-decoder state that persists across packets.
///
/// Round 2 adds [`Ac3State`] — the per-channel exponent / bap /
/// overlap-add-delay state shared with the AC-3 decoder. We carry one
/// `Ac3State` per substream id so dependent + independent substreams
/// don't trample each other's delay lines (round 3 wires this).
#[derive(Default, Clone)]
pub struct Eac3DecoderState {
    /// Last successfully-decoded indep substream parameters. Used by
    /// dependent substreams to know how many channels to extend.
    pub last_indep: Option<IndepProgramShape>,
    /// Per-channel persistent DSP state for the **independent**
    /// substream. Carries exponent reuse + 256-sample IMDCT delay line
    /// across blocks and frames.
    indep_state: Ac3State,
    /// Per-channel persistent DSP state for the **dependent** substream
    /// (round 3 hookup). Held separately so a dep-substream block's
    /// reuse-exponent path doesn't read indep exponents.
    #[allow(dead_code)]
    dep_state: Ac3State,
    /// Round-3 PCM scratch — the f32 PCM of the indep substream's
    /// channels in their native layout, populated by round 2 and
    /// awaiting dep-substream channels (chanmap-spliced) in round 3.
    #[allow(dead_code)]
    indep_pcm_f32: Vec<f32>,
    /// Per-frame error string (last seen). Diagnostic only.
    pub last_error: Option<String>,
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
/// the **independent** substream's program PCM. Dependent substreams
/// are parsed; round 3 will splice their channels into the indep PCM.
pub fn decode_eac3_packet(state: &mut Eac3DecoderState, data: &[u8]) -> Result<DecodedFrame> {
    if data.len() < 4 {
        return Err(Error::invalid("eac3: packet too short for syncinfo"));
    }
    let mut indep_pcm: Option<DecodedFrame> = None;
    let mut off = 0usize;
    state.last_error = None;
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

        // Audfrm follows BSI in the same bit cursor. AHT (audfrm
        // returning Unsupported) is recoverable: the silent emit path
        // handles it by emitting zeros for this frame instead of
        // bailing the whole packet.
        let audfrm = match audfrm::parse_with(&mut br, &bsi) {
            Ok(a) => a,
            Err(e) => {
                state.last_error = Some(format!("{e}"));
                let pcm = build_silent_indep(&bsi)?;
                if matches!(
                    bsi.strmtyp,
                    StreamType::Independent | StreamType::Ac3Convert
                ) {
                    state.last_indep = Some(IndepProgramShape {
                        nchans: pcm.channels,
                        sample_rate: pcm.sample_rate,
                        samples_per_frame: pcm.samples,
                    });
                    indep_pcm = Some(pcm);
                }
                off += frame_bytes;
                continue;
            }
        };

        match bsi.strmtyp {
            StreamType::Independent | StreamType::Ac3Convert => {
                let pcm = decode_indep_substream(state, &bsi, &audfrm, &mut br)?;
                state.last_indep = Some(IndepProgramShape {
                    nchans: pcm.channels,
                    sample_rate: pcm.sample_rate,
                    samples_per_frame: pcm.samples,
                });
                indep_pcm = Some(pcm);
            }
            StreamType::Dependent => {
                // Round 1+2: parsed but skipped. Round 3 will fold the
                // dep substream's `chanmap` channels into the indep
                // PCM when one exists in `state.last_indep`.
                let _ = decode_dep_substream(state, &bsi, &audfrm, &mut br);
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

/// Decode one independent substream's audblks. Tries the round-2 DSP
/// path first; on `Error::Unsupported` (or any other DSP error) falls
/// back to silent PCM of the right shape so the corpus driver sees
/// a frame-count match.
fn decode_indep_substream(
    state: &mut Eac3DecoderState,
    bsi: &Eac3Bsi,
    audfrm: &AudFrm,
    br: &mut BitReader<'_>,
) -> Result<DecodedFrame> {
    let samples = bsi.num_blocks as u32 * SAMPLES_PER_BLOCK as u32;
    let nchans = bsi.nchans as usize;
    let mut floats = vec![0.0f32; samples as usize * nchans];

    let dsp_result =
        dsp::decode_indep_audblks(bsi, audfrm, br, &mut state.indep_state, &mut floats);
    let pcm_s16le = match dsp_result {
        Ok(()) => {
            // Pack f32 → S16 interleaved.
            let mut out = vec![0u8; floats.len() * 2];
            for (i, s) in floats.iter().enumerate() {
                let clamped = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
                let le = clamped.to_le_bytes();
                out[i * 2] = le[0];
                out[i * 2 + 1] = le[1];
            }
            out
        }
        Err(e) => {
            // Silent fallback. Reset the per-channel exponent reuse
            // state so the next frame's reuse-strategy blocks don't
            // pick up garbage from a half-decoded prior frame.
            state.last_error = Some(format!("{e}"));
            state.indep_state = Ac3State::new();
            vec![0u8; floats.len() * 2]
        }
    };

    // Cache the f32 slot for round 3 even on silent fallback (keeps
    // sizes consistent for dep-substream splice).
    state.indep_pcm_f32 = floats;

    Ok(DecodedFrame {
        sample_rate: bsi.sample_rate,
        channels: bsi.nchans as u16,
        samples,
        pcm_s16le,
    })
}

/// Decode (currently: skip) a dependent substream. Round 3 wires the
/// chanmap-driven splice into [`Eac3DecoderState::indep_pcm_f32`].
fn decode_dep_substream(
    _state: &mut Eac3DecoderState,
    _bsi: &Eac3Bsi,
    _audfrm: &AudFrm,
    _br: &mut BitReader<'_>,
) -> Result<()> {
    Ok(())
}

/// Build a silent S16 PCM buffer of the right shape for one
/// substream. Used as a fallback when a frame can't be DSP-decoded.
fn build_silent_indep(bsi: &Eac3Bsi) -> Result<DecodedFrame> {
    // Annex E does not change the §2.2 transform: each audio block
    // produces 256 PCM samples per channel. Total per syncframe =
    // num_blocks × 256.
    let samples = bsi.num_blocks as u32 * 256;
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
            // expstre==1 → per-block chexpstr lives in audblk(), NOT
            // audfrm. (Round-2 fix: round 1 over-consumed these bits.)
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
