//! Integration tests against the `docs/audio/ac3/fixtures/` corpus.
//!
//! Each fixture under `../../docs/audio/ac3/fixtures/<name>/` ships:
//! * `input.ac3` (or `input.eac3`) — a raw elementary stream, i.e. a
//!   plain concatenation of AC-3 / E-AC-3 syncframes starting with the
//!   16-bit syncword 0x0B77 (no container, no padding).
//! * `expected.wav` — reference 16-bit signed PCM produced by FFmpeg's
//!   native AC-3 / E-AC-3 decoder. The PCM byte count typically maps
//!   1:1 to `frames × 1536 × channels × 2` after the 44-byte RIFF/WAVE
//!   header.
//! * `expected.sha256` / `notes.md` / `trace.txt` — informational; not
//!   consumed by this driver.
//!
//! For every fixture we:
//! 1. Walk `input.{ac3,eac3}` syncframe-by-syncframe (driven by the
//!    `syncinfo` table for AC-3, or the `frmsiz` field for E-AC-3).
//! 2. Hand each syncframe to a fresh in-tree AC-3 decoder via
//!    `Decoder::send_packet` / `receive_frame`, accumulating PCM bytes.
//! 3. Parse `expected.wav` to extract the reference S16 PCM.
//! 4. Report per-channel RMS / exact-match-pct / near<=1LSB-pct / PSNR
//!    over the overlapping prefix of decoded vs reference samples.
//!
//! Tiering:
//! * `Tier::BitExact` — must decode bit-exactly; CI fails on divergence.
//!   Reserved for future use — AC-3 is lossy and floating-point
//!   IMDCT rounding makes bit-exactness vs FFmpeg unrealistic.
//! * `Tier::ReportOnly` — log deltas without failing CI. Every
//!   fixture starts here; a follow-up round can promote any that turn
//!   out to round-trip cleanly.
//!
//! The current AC-3 decoder rejects E-AC-3 syncframes (BSI parser
//! bails on `bsid > 10`); E-AC-3 fixtures are still wired in at
//! `Tier::ReportOnly`, with the per-frame error count reported so a
//! follow-up implementing the Annex E parser has a clear baseline.
//!
//! Note: `oxideav-ac3` is its own repository and `docs/` is checked
//! into the workspace umbrella, not the standalone crate. When the
//! fixtures are missing the test logs `skip <name>: missing ...` and
//! returns success, so CI stays clean for both layouts.

use std::fs;
use std::path::PathBuf;

use oxideav_ac3::syncinfo;
use oxideav_core::{
    CodecId, CodecParameters, CodecRegistry, Decoder, Frame, Packet, TimeBase,
};

// ---------------------------------------------------------------------------
// Fixture path resolution
// ---------------------------------------------------------------------------

/// Locate `docs/audio/ac3/fixtures/<name>/`. Tests run with CWD at the
/// crate root, so we walk two levels up to reach the workspace root.
/// Standalone-repo CI checkouts skip gracefully (no `../../docs/`).
fn fixture_dir(name: &str) -> PathBuf {
    PathBuf::from("../../docs/audio/ac3/fixtures").join(name)
}

// ---------------------------------------------------------------------------
// Tiny WAV parser — same shape as the g711/vorbis docs_corpus tests.
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct RefPcm {
    sample_rate: u32,
    channels: u16,
    /// Interleaved S16LE samples extracted from `expected.wav`'s `data`
    /// chunk.
    samples: Vec<i16>,
}

/// Parse a minimal RIFF/WAVE file. Walks chunks until it finds `fmt `
/// and `data`. Accepts WAVE_FORMAT_PCM (1) or WAVE_FORMAT_EXTENSIBLE
/// (0xFFFE) at 16-bit S16LE — that's all the FFmpeg-generated
/// `expected.wav` files in this corpus carry.
fn parse_wav(bytes: &[u8]) -> Option<RefPcm> {
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    let mut off = 12usize;
    let mut channels: u16 = 0;
    let mut sample_rate: u32 = 0;
    let mut bits_per_sample: u16 = 0;
    let mut format_tag: u16 = 0;
    let mut data: Option<&[u8]> = None;
    while off + 8 <= bytes.len() {
        let id = &bytes[off..off + 4];
        let sz = u32::from_le_bytes([
            bytes[off + 4],
            bytes[off + 5],
            bytes[off + 6],
            bytes[off + 7],
        ]) as usize;
        let body_start = off + 8;
        let body_end = body_start.checked_add(sz)?;
        if body_end > bytes.len() {
            break;
        }
        match id {
            b"fmt " => {
                if sz < 16 {
                    return None;
                }
                format_tag = u16::from_le_bytes([bytes[body_start], bytes[body_start + 1]]);
                channels = u16::from_le_bytes([bytes[body_start + 2], bytes[body_start + 3]]);
                sample_rate = u32::from_le_bytes([
                    bytes[body_start + 4],
                    bytes[body_start + 5],
                    bytes[body_start + 6],
                    bytes[body_start + 7],
                ]);
                bits_per_sample =
                    u16::from_le_bytes([bytes[body_start + 14], bytes[body_start + 15]]);
            }
            b"data" => {
                data = Some(&bytes[body_start..body_end]);
                break;
            }
            _ => {}
        }
        off = body_end + (sz & 1);
    }
    let data = data?;
    if format_tag != 1 && format_tag != 0xFFFE {
        return None;
    }
    if channels == 0 || sample_rate == 0 || bits_per_sample != 16 {
        return None;
    }
    let mut samples = Vec::with_capacity(data.len() / 2);
    for chunk in data.chunks_exact(2) {
        samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
    }
    Some(RefPcm {
        sample_rate,
        channels,
        samples,
    })
}

// ---------------------------------------------------------------------------
// Syncframe iteration — AC-3 (table-driven) + E-AC-3 (frmsiz field).
// ---------------------------------------------------------------------------

/// One syncframe extracted from a raw elementary stream.
struct Frm {
    /// Inclusive byte range within the original buffer.
    start: usize,
    len: usize,
    /// `bsid` field. <=8 means standard AC-3, 16 means E-AC-3 Annex E.
    bsid: u8,
}

/// Iterate the syncframes inside `data`. Tries the AC-3 syncinfo path
/// first; on failure (which is normal for E-AC-3 because A/52 §5.4.1.4
/// frmsizecod table stops at index 37 but Annex E uses a direct
/// frmsiz field) we fall back to reading bytes 2-3 as
/// `strmtyp(2)|substreamid(3)|frmsiz(11)` — frame_size = (frmsiz+1)*2.
///
/// `bsid` for both paths sits at the same place: 5 bits at bit
/// offset 40 in standard AC-3, and at bit offset 35 (after
/// `strmtyp`+`substreamid`+`frmsiz`+`fscod`+`numblkscod`+`acmod`+`lfeon`)
/// in E-AC-3. We don't need to extract bsid from E-AC-3 — its
/// presence (i.e. AC-3 syncinfo failed but we hit a 0x0B77 word) is
/// a sufficient marker for our purposes.
fn iter_syncframes(data: &[u8]) -> Vec<Frm> {
    let mut out = Vec::new();
    let mut off = 0usize;
    while off + 5 <= data.len() {
        if data[off] != 0x0B || data[off + 1] != 0x77 {
            // Misalignment — search for the next syncword.
            match syncinfo::find_syncword(data, off + 1) {
                Some(next) => {
                    off = next;
                    continue;
                }
                None => break,
            }
        }
        // Try AC-3 path first.
        if let Ok(si) = syncinfo::parse(&data[off..]) {
            let flen = si.frame_length as usize;
            if off + flen > data.len() {
                break;
            }
            // bsid lives in BSI byte 0, top 5 bits.
            let bsi_byte0 = data[off + 5];
            let bsid = bsi_byte0 >> 3;
            if bsid <= 10 {
                out.push(Frm {
                    start: off,
                    len: flen,
                    bsid,
                });
                off += flen;
                continue;
            }
            // bsid > 10 falls through to E-AC-3 parsing.
        }
        // E-AC-3 path: byte 2 = strmtyp(2)|substreamid(3)|frmsiz_hi(3),
        //              byte 3 = frmsiz_lo(8). frame_size = (frmsiz+1) * 2.
        if off + 4 > data.len() {
            break;
        }
        let frmsiz_hi = (data[off + 2] & 0x07) as u32;
        let frmsiz_lo = data[off + 3] as u32;
        let frmsiz = (frmsiz_hi << 8) | frmsiz_lo;
        let flen = ((frmsiz + 1) * 2) as usize;
        if flen < 6 || off + flen > data.len() {
            break;
        }
        // For E-AC-3 we don't bother extracting bsid here — the iter
        // only cares about the frame range so the caller can feed
        // each frame to the decoder. Tag with the canonical Annex E
        // value (16, A/52 §E.2.3.1.6) so reports differentiate the
        // bitstream variant.
        out.push(Frm {
            start: off,
            len: flen,
            bsid: 16,
        });
        off += flen;
    }
    out
}

// ---------------------------------------------------------------------------
// Decode-and-score
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
enum Tier {
    /// Must decode bit-exactly. Reserved for future use; AC-3's
    /// floating-point IMDCT path makes vs-FFmpeg bit-exactness
    /// unrealistic.
    BitExact,
    /// Logged but never gates CI. Every fixture in this driver starts
    /// here; promote individual cases once they're shown to round-trip
    /// cleanly through both decoders.
    ReportOnly,
}

/// Per-channel diff numbers + aggregate match percentage and PSNR.
struct ChannelStat {
    rms_ref: f64,
    rms_ours: f64,
    /// Sum of squared per-sample errors. Kept as a sum until the final
    /// log line (PSNR uses MSE = sum / total, the printed RMS divides
    /// by total then takes sqrt).
    sse_err: f64,
    exact: usize,
    near: usize, // |delta| <= 1 LSB
    total: usize,
    max_abs_err: i32,
}

impl ChannelStat {
    fn new() -> Self {
        Self {
            rms_ref: 0.0,
            rms_ours: 0.0,
            sse_err: 0.0,
            exact: 0,
            near: 0,
            total: 0,
            max_abs_err: 0,
        }
    }

    fn match_pct(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            self.exact as f64 / self.total as f64 * 100.0
        }
    }

    fn near_pct(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            self.near as f64 / self.total as f64 * 100.0
        }
    }

    /// PSNR over the 16-bit signed full scale (peak = 32767). Returns
    /// `f64::INFINITY` on perfect match.
    fn psnr_db(&self) -> f64 {
        if self.total == 0 || self.sse_err == 0.0 {
            return f64::INFINITY;
        }
        let mse = self.sse_err / self.total as f64;
        let peak = 32767.0_f64;
        10.0 * (peak * peak / mse).log10()
    }

    fn rms_err_disp(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            (self.sse_err / self.total as f64).sqrt()
        }
    }
}

/// Decoded output from a single fixture run.
struct DecodedPcm {
    /// Interleaved S16LE samples (already converted from byte-buffer
    /// to `i16`).
    samples: Vec<i16>,
    /// Channels as advertised by the AC-3 BSI of the FIRST decoded
    /// frame (downmix may have been applied — we don't request one,
    /// so this matches the bitstream's nchans).
    channels: u16,
    sample_rate: u32,
    /// Number of syncframes we found.
    frames_total: usize,
    /// Number that decoded successfully.
    frames_ok: usize,
    /// First decoder error string (if any) — for the report.
    first_error: Option<String>,
}

fn decode_stream(input: &[u8]) -> DecodedPcm {
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register(&mut reg);
    let params = CodecParameters::audio(CodecId::new("ac3"));
    let mut dec: Box<dyn Decoder> = match reg.make_decoder(&params) {
        Ok(d) => d,
        Err(e) => {
            return DecodedPcm {
                samples: Vec::new(),
                channels: 0,
                sample_rate: 0,
                frames_total: 0,
                frames_ok: 0,
                first_error: Some(format!("make_decoder: {e:?}")),
            };
        }
    };

    let frms = iter_syncframes(input);
    let mut samples: Vec<i16> = Vec::new();
    let mut frames_ok = 0usize;
    let mut first_error: Option<String> = None;
    let mut channels: u16 = 0;
    let mut sample_rate: u32 = 0;

    for (i, f) in frms.iter().enumerate() {
        let payload = input[f.start..f.start + f.len].to_vec();
        let pkt = Packet::new(0, TimeBase::new(1, 48_000), payload).with_pts(i as i64 * 1536);
        if let Err(e) = dec.send_packet(&pkt) {
            if first_error.is_none() {
                first_error = Some(format!(
                    "send_packet@frame{i} (bsid={}): {e:?}",
                    f.bsid
                ));
            }
            // Reset state so the next frame's send_packet doesn't see
            // a stale `pending` slot.
            let _ = dec.reset();
            continue;
        }
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => {
                // af.data[0] is interleaved S16LE bytes.
                let plane = &af.data[0];
                let n = plane.len() / 2;
                if channels == 0 && af.samples > 0 {
                    // Channels = total samples / per-channel samples.
                    let per_ch = af.samples as usize;
                    if per_ch > 0 {
                        channels = (n / per_ch) as u16;
                    }
                    sample_rate = 48_000; // AC-3 max; we don't have a per-frame field
                }
                samples.reserve(n);
                for chunk in plane.chunks_exact(2) {
                    samples.push(i16::from_le_bytes([chunk[0], chunk[1]]));
                }
                frames_ok += 1;
            }
            Ok(other) => {
                if first_error.is_none() {
                    first_error =
                        Some(format!("frame{i}: unexpected non-audio frame: {other:?}"));
                }
                let _ = dec.reset();
            }
            Err(e) => {
                if first_error.is_none() {
                    first_error = Some(format!(
                        "receive_frame@frame{i} (bsid={}): {e:?}",
                        f.bsid
                    ));
                }
                let _ = dec.reset();
            }
        }
    }

    DecodedPcm {
        samples,
        channels,
        sample_rate,
        frames_total: frms.len(),
        frames_ok,
        first_error,
    }
}

fn compare(ours: &DecodedPcm, refp: &RefPcm) -> Vec<ChannelStat> {
    let chs = ours.channels.min(refp.channels) as usize;
    if chs == 0 {
        return Vec::new();
    }
    let frames_ours = ours.samples.len() / ours.channels.max(1) as usize;
    let frames_ref = refp.samples.len() / refp.channels.max(1) as usize;
    let n = frames_ours.min(frames_ref);

    let mut stats: Vec<ChannelStat> = (0..chs).map(|_| ChannelStat::new()).collect();
    for f in 0..n {
        for (ch, s) in stats.iter_mut().enumerate() {
            let our = ours.samples[f * ours.channels as usize + ch] as i64;
            let r = refp.samples[f * refp.channels as usize + ch] as i64;
            let err = (our - r).abs();
            s.total += 1;
            if err == 0 {
                s.exact += 1;
            }
            if err <= 1 {
                s.near += 1;
            }
            if err as i32 > s.max_abs_err {
                s.max_abs_err = err as i32;
            }
            s.rms_ref += (r * r) as f64;
            s.rms_ours += (our * our) as f64;
            s.sse_err += (err * err) as f64;
        }
    }
    for s in &mut stats {
        if s.total > 0 {
            s.rms_ref = (s.rms_ref / s.total as f64).sqrt();
            s.rms_ours = (s.rms_ours / s.total as f64).sqrt();
        }
    }
    stats
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

struct CorpusCase {
    /// Fixture directory name under `docs/audio/ac3/fixtures/`.
    name: &'static str,
    /// Input filename inside that directory — `input.ac3` or
    /// `input.eac3`.
    input_filename: &'static str,
    /// Expected channels (sanity-check vs `expected.wav`). None to
    /// skip the assertion.
    channels: Option<u16>,
    /// Expected sample rate. None to skip.
    sample_rate: Option<u32>,
    /// True if this fixture is E-AC-3 and the in-tree decoder is
    /// expected to bail at BSI (bsid=16). The driver still records
    /// the per-frame error count.
    eac3: bool,
    tier: Tier,
}

fn evaluate(case: &CorpusCase) {
    eprintln!(
        "--- {} (eac3={} tier={:?}) ---",
        case.name, case.eac3, case.tier
    );
    let dir = fixture_dir(case.name);
    let input_path = dir.join(case.input_filename);
    let input_bytes = match fs::read(&input_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!(
                "skip {}: missing {} ({e})",
                case.name,
                input_path.display()
            );
            return;
        }
    };
    let expected_path = dir.join("expected.wav");
    let expected_bytes = match fs::read(&expected_path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!(
                "skip {}: missing {} ({e})",
                case.name,
                expected_path.display()
            );
            return;
        }
    };
    let refp = match parse_wav(&expected_bytes) {
        Some(p) => p,
        None => {
            eprintln!("skip {}: expected.wav unparseable", case.name);
            return;
        }
    };

    let ours = decode_stream(&input_bytes);
    eprintln!(
        "{}: input={} bytes, syncframes={} ({} decoded ok), expected ch={} sr={} samples={}",
        case.name,
        input_bytes.len(),
        ours.frames_total,
        ours.frames_ok,
        refp.channels,
        refp.sample_rate,
        refp.samples.len(),
    );
    if let Some(err) = &ours.first_error {
        eprintln!("{}: first decoder error: {err}", case.name);
    }

    if let Some(want_ch) = case.channels {
        if refp.channels != want_ch {
            eprintln!(
                "{}: WARN expected.wav says {} channels, case requested {}",
                case.name, refp.channels, want_ch
            );
        }
    }
    if let Some(want_sr) = case.sample_rate {
        if refp.sample_rate != want_sr {
            eprintln!(
                "{}: WARN expected.wav sample rate {} != case-requested {}",
                case.name, refp.sample_rate, want_sr
            );
        }
    }

    if ours.frames_ok == 0 {
        eprintln!(
            "{}: no frames decoded — nothing to compare (typical for E-AC-3 against \
             the AC-3-only decoder).",
            case.name
        );
        match case.tier {
            Tier::BitExact => {
                panic!(
                    "{}: BitExact requires successful decode; first_error={:?}",
                    case.name, ours.first_error
                );
            }
            Tier::ReportOnly => return,
        }
    }

    eprintln!(
        "{}: decoded ch={} sr={} samples={} ({} frames)",
        case.name,
        ours.channels,
        ours.sample_rate,
        ours.samples.len(),
        ours.samples.len() / ours.channels.max(1) as usize,
    );
    if ours.channels != refp.channels {
        eprintln!(
            "{}: WARN channel-count mismatch (decoded {} vs reference {})",
            case.name, ours.channels, refp.channels
        );
    }
    if ours.sample_rate != refp.sample_rate {
        eprintln!(
            "{}: WARN sample-rate mismatch (decoded {} vs reference {})",
            case.name, ours.sample_rate, refp.sample_rate
        );
    }

    let stats = compare(&ours, &refp);
    if stats.is_empty() {
        eprintln!("{}: no overlapping channels to compare", case.name);
        return;
    }

    let mut total_exact = 0usize;
    let mut total_near = 0usize;
    let mut total_samples = 0usize;
    let mut max_err_overall = 0i32;
    let mut psnr_min: f64 = f64::INFINITY;
    for (i, s) in stats.iter().enumerate() {
        let psnr = s.psnr_db();
        if psnr < psnr_min {
            psnr_min = psnr;
        }
        eprintln!(
            "  ch{i}: rms_ref={:.1} rms_ours={:.1} rms_err={:.2} match={:.4}% \
             near<=1LSB={:.4}% max_abs_err={} psnr={:.2} dB",
            s.rms_ref,
            s.rms_ours,
            s.rms_err_disp(),
            s.match_pct(),
            s.near_pct(),
            s.max_abs_err,
            psnr,
        );
        total_exact += s.exact;
        total_near += s.near;
        total_samples += s.total;
        if s.max_abs_err > max_err_overall {
            max_err_overall = s.max_abs_err;
        }
    }
    let agg_pct = if total_samples > 0 {
        total_exact as f64 / total_samples as f64 * 100.0
    } else {
        0.0
    };
    let near_pct = if total_samples > 0 {
        total_near as f64 / total_samples as f64 * 100.0
    } else {
        0.0
    };
    eprintln!(
        "{}: aggregate match={:.4}% near<=1LSB={:.4}% max_abs_err={} min_psnr={:.2} dB",
        case.name, agg_pct, near_pct, max_err_overall, psnr_min,
    );

    match case.tier {
        Tier::BitExact => {
            assert_eq!(
                total_exact, total_samples,
                "{}: not bit-exact (max_abs_err={} match={:.4}%)",
                case.name, max_err_overall, agg_pct,
            );
        }
        Tier::ReportOnly => {
            // Logged; never gates CI.
        }
    }
}

// ---------------------------------------------------------------------------
// Per-fixture tests — every entry maps 1:1 to a directory under
// docs/audio/ac3/fixtures/. All start `Tier::ReportOnly` per the brief.
// ---------------------------------------------------------------------------

#[test]
fn corpus_ac3_2_1_48000_256kbps() {
    evaluate(&CorpusCase {
        name: "ac3-2-1-48000-256kbps",
        input_filename: "input.ac3",
        channels: Some(3),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_3_0_48000() {
    evaluate(&CorpusCase {
        name: "ac3-3-0-48000",
        input_filename: "input.ac3",
        channels: Some(3),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_3_2_48000_384kbps() {
    evaluate(&CorpusCase {
        name: "ac3-3-2-48000-384kbps",
        input_filename: "input.ac3",
        channels: Some(5),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_3_2_lfe_48000_448kbps() {
    evaluate(&CorpusCase {
        name: "ac3-3-2-lfe-48000-448kbps",
        input_filename: "input.ac3",
        channels: Some(6),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_32000hz_stereo() {
    evaluate(&CorpusCase {
        name: "ac3-32000hz-stereo",
        input_filename: "input.ac3",
        channels: Some(2),
        sample_rate: Some(32_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_44100hz_stereo() {
    evaluate(&CorpusCase {
        name: "ac3-44100hz-stereo",
        input_filename: "input.ac3",
        channels: Some(2),
        sample_rate: Some(44_100),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_low_bitrate_32kbps_mono() {
    evaluate(&CorpusCase {
        name: "ac3-low-bitrate-32kbps-mono",
        input_filename: "input.ac3",
        channels: Some(1),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_mono_48000_96kbps() {
    evaluate(&CorpusCase {
        name: "ac3-mono-48000-96kbps",
        input_filename: "input.ac3",
        channels: Some(1),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_stereo_48000_192kbps() {
    evaluate(&CorpusCase {
        name: "ac3-stereo-48000-192kbps",
        input_filename: "input.ac3",
        channels: Some(2),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_with_audprodie() {
    evaluate(&CorpusCase {
        name: "ac3-with-audprodie",
        input_filename: "input.ac3",
        channels: Some(2),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_ac3_with_dialnorm_set() {
    evaluate(&CorpusCase {
        name: "ac3-with-dialnorm-set",
        input_filename: "input.ac3",
        channels: Some(2),
        sample_rate: Some(48_000),
        eac3: false,
        tier: Tier::ReportOnly,
    });
}

// ---------------------------------------------------------------------------
// E-AC-3 fixtures — the in-tree decoder rejects bsid > 10 (Annex E
// Annex parser is a future task). Wired in at `Tier::ReportOnly` so the
// per-frame error count is captured for the follow-up that adds
// E-AC-3 decode.
// ---------------------------------------------------------------------------

#[test]
fn corpus_eac3_256_coeff_block() {
    evaluate(&CorpusCase {
        name: "eac3-256-coeff-block",
        input_filename: "input.eac3",
        channels: Some(2),
        sample_rate: Some(48_000),
        eac3: true,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_eac3_5_1_48000_384kbps() {
    evaluate(&CorpusCase {
        name: "eac3-5.1-48000-384kbps",
        input_filename: "input.eac3",
        channels: Some(6),
        sample_rate: Some(48_000),
        eac3: true,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_eac3_5_1_side_768kbps() {
    evaluate(&CorpusCase {
        name: "eac3-5.1-side-768kbps",
        input_filename: "input.eac3",
        channels: Some(6),
        sample_rate: Some(48_000),
        eac3: true,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_eac3_from_ac3_bitstream_recombination() {
    evaluate(&CorpusCase {
        name: "eac3-from-ac3-bitstream-recombination",
        input_filename: "input.eac3",
        channels: Some(2),
        sample_rate: Some(48_000),
        eac3: true,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_eac3_low_bitrate_32kbps() {
    evaluate(&CorpusCase {
        name: "eac3-low-bitrate-32kbps",
        input_filename: "input.eac3",
        channels: Some(1),
        sample_rate: Some(48_000),
        eac3: true,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_eac3_low_rate_stereo_64kbps() {
    evaluate(&CorpusCase {
        name: "eac3-low-rate-stereo-64kbps",
        input_filename: "input.eac3",
        channels: Some(2),
        sample_rate: Some(48_000),
        eac3: true,
        tier: Tier::ReportOnly,
    });
}

#[test]
fn corpus_eac3_stereo_48000_192kbps() {
    evaluate(&CorpusCase {
        name: "eac3-stereo-48000-192kbps",
        input_filename: "input.eac3",
        channels: Some(2),
        sample_rate: Some(48_000),
        eac3: true,
        tier: Tier::ReportOnly,
    });
}

// ---------------------------------------------------------------------------
// Sanity test for the syncframe iterator on a known-good fixture.
// Guards against regressions in `iter_syncframes` itself, which is the
// load-bearing piece outside the decoder under test.
// ---------------------------------------------------------------------------

#[test]
fn syncframe_iter_matches_table_5_18_for_192kbps_48k_stereo() {
    let dir = fixture_dir("ac3-stereo-48000-192kbps");
    let bytes = match fs::read(dir.join("input.ac3")) {
        Ok(b) => b,
        Err(_) => return,
    };
    // Per A/52 §5.4.1.4, frmsizecod=24 at fscod=0 ⇒ 384 words = 768
    // bytes. The fixture is 12288 bytes, so we expect exactly 16
    // syncframes, all 768 bytes long, all with bsid=8.
    let frms = iter_syncframes(&bytes);
    assert_eq!(frms.len(), 16, "expected 16 syncframes, got {}", frms.len());
    for f in &frms {
        assert_eq!(f.len, 768, "frame at offset {} not 768 bytes", f.start);
        assert_eq!(f.bsid, 8, "frame at offset {} has unexpected bsid", f.start);
    }
}

/// Sanity test for the E-AC-3 path of the iterator. The fixture is 12288
/// bytes of 192 kbps E-AC-3; `frmsiz` = 383 ⇒ frame_size = 768 bytes.
#[test]
fn syncframe_iter_handles_eac3_frmsiz_field() {
    let dir = fixture_dir("eac3-stereo-48000-192kbps");
    let bytes = match fs::read(dir.join("input.eac3")) {
        Ok(b) => b,
        Err(_) => return,
    };
    let frms = iter_syncframes(&bytes);
    // Some of the AC-3 frmsizecod values DO accidentally produce a
    // valid `frame_length` for the leading bytes of an E-AC-3 frame
    // (Table 5.18 covers all 6-bit values 0..38 for fscod=0). When that
    // happens iter_syncframes takes the AC-3 path and tags the frame
    // with bsid extracted from byte 5 — which for an E-AC-3 frame sits
    // at a different bit offset and so is meaningless. This is fine
    // for the driver — we only use the frame ranges to feed the
    // decoder, and AC-3 BSI parse will reject bsid>10 deterministically.
    assert!(
        !frms.is_empty(),
        "iter_syncframes returned 0 frames for a known E-AC-3 fixture"
    );
    // Total bytes covered ≈ input size (within a small slack — the
    // iterator may bail before the last partial frame in malformed
    // streams; for well-formed FFmpeg output the slack should be 0).
    let total: usize = frms.iter().map(|f| f.len).sum();
    assert!(
        total + 16 >= bytes.len(),
        "iter_syncframes left {} bytes uncovered (got {} of {})",
        bytes.len() - total,
        total,
        bytes.len(),
    );
}

