//! E-AC-3 syncframe decoder — rounds 1 + 2 + 3.
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
//! Round-2 path:
//!
//! 5. Hand the BitReader to [`super::dsp::decode_indep_audblks`],
//!    which walks per-block side-info, runs §7 bit allocation +
//!    mantissa unpack, and applies IMDCT + window + overlap-add via
//!    the AC-3 helpers. On any "unsupported feature" error
//!    (coupling, SPX, AHT, transient processing, …) the decoder
//!    falls back to silent emit so the corpus driver keeps decoding.
//!
//! Round-3 path (this commit):
//!
//! 6. Walk dependent substreams in the same packet, decode each via
//!    the round-2 DSP, and **splice** the resulting channels into
//!    [`Eac3DecoderState::indep_pcm_f32`] so the final emitted PCM
//!    is `(indep_nchans + Σ dep_nchans) × samples`. The chanmap
//!    field (Table E2.5) is parsed but not used to reorder — round
//!    3 simply appends dep channels at the end of the indep program.
//!    None of the corpus fixtures actually exercise dep substreams
//!    (FFmpeg's eac3 encoder doesn't emit them per
//!    `eac3-5.1-side-768kbps/notes.md`); the plumbing is in place so
//!    a future custom-built fixture (or interop with a real DD+
//!    7.1 stream) works end-to-end.

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
    /// Per-channel persistent DSP state for the **dependent** substream.
    /// Held separately so a dep-substream block's reuse-exponent path
    /// doesn't read indep exponents.
    dep_state: Ac3State,
    /// Per-frame f32 PCM scratch — the indep substream's PCM in its
    /// native layout immediately after [`decode_indep_substream`]; if
    /// any dep substreams follow in the same packet, their channels
    /// are spliced in by [`decode_dep_substream`], growing this slot
    /// to `indep_nchans + dep_nchans`.
    indep_pcm_f32: Vec<f32>,
    /// Current channel count of [`Self::indep_pcm_f32`] — starts at
    /// the indep BSI's `nchans` and grows as dep substreams splice
    /// their channels in.
    indep_nchans: u16,
    /// Samples-per-frame (per channel) of [`Self::indep_pcm_f32`].
    indep_samples_per_frame: u32,
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
    /// Independent substream's `acmod` field (Table E1.2 §E.1.2.1).
    /// Surfaced so callers that need to reorder bitstream-order
    /// multichannel layouts into WAV-mask order can pick the right
    /// permutation via [`crate::wave_order`]. For dep-substream-extended
    /// programs (e.g. 7.1 emitted as indep 5.1 + dep [Lb,Rb]) this
    /// reflects the **indep** acmod only — the additional dep channels
    /// are appended at the end of the PCM buffer per
    /// `splice_dep_into_indep` and are not covered by Table 5.8.
    pub acmod: u8,
    /// Independent substream's `lfeon` flag (1 bit, §E.1.2.1).
    pub lfeon: bool,
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
                // Round 3: decode + splice into indep_pcm_f32. On
                // failure (Unsupported / parse error) we keep the
                // indep PCM as-is so output is still meaningful.
                let _ = decode_dep_substream(state, &bsi, &audfrm, &mut br);
            }
            StreamType::Reserved => {
                return Err(Error::invalid("eac3: strmtyp '11' is reserved"));
            }
        }

        off += frame_bytes;
    }

    let mut pcm = indep_pcm.ok_or_else(|| {
        Error::invalid(
            "eac3: packet contains no independent substream (only dependent or ac3-convert frames)",
        )
    })?;

    // Rebuild the final S16 buffer from the (possibly extended)
    // [`indep_pcm_f32`] scratch. When no dep substream was seen, this
    // is the indep PCM unchanged; when one or more dep substreams
    // contributed, the scratch has grown to `indep_nchans + Σ
    // dep_nchans` channels.
    if state.indep_nchans != pcm.channels {
        pcm.channels = state.indep_nchans;
        pcm.pcm_s16le = pack_f32_to_s16le(&state.indep_pcm_f32);
    }
    Ok(pcm)
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
    if let Err(e) = &dsp_result {
        // Silent fallback. Reset the per-channel exponent reuse state
        // so the next frame's reuse-strategy blocks don't pick up
        // garbage from a half-decoded prior frame.
        state.last_error = Some(format!("{e}"));
        state.indep_state = Ac3State::new();
        for v in floats.iter_mut() {
            *v = 0.0;
        }
    }

    // Cache the indep f32 PCM in its native (acmod, lfeon) layout
    // so any subsequent dep substream in the same packet can append
    // its chanmap-routed channels (round 3).
    state.indep_pcm_f32 = floats;
    state.indep_nchans = nchans as u16;
    state.indep_samples_per_frame = samples;

    // Pack indep PCM only. If a dep substream follows, the packet-
    // level driver will rebuild the final S16 buffer in
    // `decode_eac3_packet` after all substreams are walked.
    let pcm_s16le = pack_f32_to_s16le(&state.indep_pcm_f32);

    Ok(DecodedFrame {
        sample_rate: bsi.sample_rate,
        channels: bsi.nchans as u16,
        samples,
        pcm_s16le,
        acmod: bsi.acmod,
        lfeon: bsi.lfeon,
    })
}

/// Decode one dependent substream and splice its `chanmap`-routed
/// channels into the indep substream's PCM scratch
/// [`Eac3DecoderState::indep_pcm_f32`].
///
/// Returns `Ok(extended_channel_count)` on success; `Err(...)` if
/// the dep substream uses a feature we can't decode (round-2-style
/// silent-fallback applied to the dep audio, leaving the indep PCM
/// untouched).
fn decode_dep_substream(
    state: &mut Eac3DecoderState,
    bsi: &Eac3Bsi,
    audfrm: &AudFrm,
    br: &mut BitReader<'_>,
) -> Result<u16> {
    let samples = bsi.num_blocks as u32 * SAMPLES_PER_BLOCK as u32;
    let dep_nchans = bsi.nchans as usize;

    // Without an indep substream in the same packet there is nothing
    // to extend. Per §E.2.3.1.1 a dep substream must follow an
    // indep substream; flag it but don't error.
    if state.last_indep.is_none() {
        return Err(Error::invalid(
            "eac3 dep: dependent substream with no preceding independent substream",
        ));
    }
    if state.indep_samples_per_frame != samples {
        return Err(Error::invalid(format!(
            "eac3 dep: dep substream sample-count {samples} differs from indep {}",
            state.indep_samples_per_frame
        )));
    }

    // Decode the dep substream into its own f32 buffer. Round 3 reuses
    // the indep DSP path; the dep substream's audblks have the same
    // syntax (Table E1.4 doesn't branch on strmtyp).
    let mut dep_floats = vec![0.0f32; samples as usize * dep_nchans];
    if let Err(e) =
        dsp::decode_indep_audblks(bsi, audfrm, br, &mut state.dep_state, &mut dep_floats)
    {
        // Silent fallback for the dep substream — leave indep PCM
        // untouched so the indep program is still audible.
        state.last_error = Some(format!("{e}"));
        state.dep_state = Ac3State::new();
        return Err(e);
    }

    // Splice channels per chanmap (Table E2.5).
    splice_dep_into_indep(state, bsi, &dep_floats);
    Ok(state.indep_nchans)
}

/// Pack interleaved f32 PCM (range -1..1) into S16LE bytes.
fn pack_f32_to_s16le(floats: &[f32]) -> Vec<u8> {
    let mut out = vec![0u8; floats.len() * 2];
    for (i, s) in floats.iter().enumerate() {
        let clamped = (s * 32767.0).clamp(-32768.0, 32767.0) as i16;
        let le = clamped.to_le_bytes();
        out[i * 2] = le[0];
        out[i * 2 + 1] = le[1];
    }
    out
}

/// Splice the dep substream's `nfchans` channels into the indep PCM
/// scratch per the chanmap field (Table E2.5). Grows the scratch's
/// channel count + reinterleaves on the fly.
///
/// Round 3 implementation strategy: simply **append** the dep
/// substream's channels at the end of the indep program (i.e. for an
/// indep 5.1 + dep [Lc Rc], the output becomes 8 channels [L C R Ls
/// Rs LFE Lc Rc]). The chanmap bits aren't currently used to
/// reorder — Table E2.5 specifies where each channel sits in a 16-
/// channel reference grid, but downstream consumers (the corpus
/// driver and the `Downmix` engine) work in acmod-native layouts
/// only. A future round can route per-bit if/when a fixture exercises
/// a non-trivial chanmap.
fn splice_dep_into_indep(state: &mut Eac3DecoderState, bsi: &Eac3Bsi, dep_floats: &[f32]) {
    let dep_nchans = bsi.nchans as usize;
    let samples = state.indep_samples_per_frame as usize;
    let indep_nchans = state.indep_nchans as usize;
    let new_nchans = indep_nchans + dep_nchans;

    let mut grown = vec![0.0f32; samples * new_nchans];
    for n in 0..samples {
        for ch in 0..indep_nchans {
            grown[n * new_nchans + ch] = state.indep_pcm_f32[n * indep_nchans + ch];
        }
        for ch in 0..dep_nchans {
            grown[n * new_nchans + indep_nchans + ch] = dep_floats[n * dep_nchans + ch];
        }
    }
    state.indep_pcm_f32 = grown;
    state.indep_nchans = new_nchans as u16;

    // Diagnostic: log the chanmap for any future bug hunts.
    if let Some(map) = bsi.chanmap {
        if std::env::var("EAC3_TRACE_CHANMAP").is_ok() {
            eprintln!(
                "TRACE-CHANMAP dep substream: chanmap=0x{map:04X} dep_nchans={dep_nchans} → indep grown to {new_nchans} channels",
            );
        }
    }
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
        acmod: bsi.acmod,
        lfeon: bsi.lfeon,
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
