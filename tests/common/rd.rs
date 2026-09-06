//! Equal-rate rate-distortion measurement harness.
//!
//! Encodes a deterministic synthetic corpus (speech, music, transients,
//! tones, pink noise, 5.1 with LFE) with our AC-3 / E-AC-3 encoder and
//! with the black-box reference encoder at the same nominal bit rate,
//! decodes every stream with **both** our decoder and the black-box
//! decoder, and scores each decode against the source PCM:
//!
//! * `snr` — per-channel signal-to-noise ratio (dB, energy ratio over
//!   the whole clip); the table reports the worst channel.
//! * `nmr` — mean noise-to-mask ratio (dB) across every (block, band)
//!   cell whose signal energy sits above the §7.2.2.5 hearing
//!   threshold. The mask is the A/52 §7.2.2 parametric model
//!   (band psd → excitation with the fast/slow leak filters → dbknee
//!   correction → hearing-threshold floor) evaluated on the source's
//!   MDCT with the §8.2.12 basic-encoder parameter set. Negative is
//!   "noise below the mask"; the reference encoder and ours are
//!   scored by the same model so the comparison is fair.
//!
//! Everything here is black-box: the reference encoder/decoder is an
//! external binary invoked through `Command`; no diagnostics of it are
//! read.
#![allow(clippy::needless_range_loop)]

use std::collections::HashMap;
use std::io::Write;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Mutex, OnceLock};

use oxideav_ac3::mdct::mdct_512;
use oxideav_ac3::tables::{
    BNDSZ, BNDTAB, DBPBTAB, FASTDEC, FASTGAIN, HTH, MASKTAB, SLOWDEC, SLOWGAIN, WINDOW,
};
use oxideav_core::{
    AudioFrame, CodecId, CodecOptions, CodecParameters, CodecRegistry, Error, Frame, Packet,
    SampleFormat, TimeBase,
};

pub const SR: u32 = 48_000;

// ---------------------------------------------------------------------------
// Corpus
// ---------------------------------------------------------------------------

/// One corpus clip: interleaved f32 PCM in WAV channel order
/// (mono / L R / FL FR FC LFE BL BR).
#[derive(Clone)]
pub struct Signal {
    pub name: &'static str,
    pub channels: usize,
    pub lfe: bool,
    pub pcm: Vec<f32>,
}

impl Signal {
    pub fn samples(&self) -> usize {
        self.pcm.len() / self.channels
    }
}

struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed | 1)
    }
    fn next_u32(&mut self) -> u32 {
        // xorshift64*
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32
    }
    /// Uniform in [-1, 1).
    fn uni(&mut self) -> f32 {
        (self.next_u32() as f32 / 2_147_483_648.0) - 1.0
    }
    fn gauss(&mut self) -> f32 {
        // Sum of 4 uniforms — plenty for test noise.
        (self.uni() + self.uni() + self.uni() + self.uni()) * 0.5
    }
}

/// One-pole low-pass state.
fn lp1(x: f32, state: &mut f32, coef: f32) -> f32 {
    *state += coef * (x - *state);
    *state
}

/// Pink-ish noise: white noise through three parallel one-pole
/// low-passes (a classic -3 dB/oct approximation).
fn pink(rng: &mut Rng, st: &mut [f32; 3]) -> f32 {
    let w = rng.gauss();
    let a = lp1(w, &mut st[0], 0.0033);
    let b = lp1(w, &mut st[1], 0.032);
    let c = lp1(w, &mut st[2], 0.29);
    (a * 3.2 + b * 1.4 + c * 0.7 + w * 0.15) * 0.25
}

/// Two-pole resonator (formant) — direct-form II transposed.
struct Reson {
    b0: f32,
    a1: f32,
    a2: f32,
    z1: f32,
    z2: f32,
}
impl Reson {
    fn new(freq: f32, bw: f32) -> Self {
        let r = (-std::f32::consts::PI * bw / SR as f32).exp();
        let a1 = -2.0 * r * (2.0 * std::f32::consts::PI * freq / SR as f32).cos();
        let a2 = r * r;
        Reson {
            b0: 1.0 - r,
            a1,
            a2,
            z1: 0.0,
            z2: 0.0,
        }
    }
    fn set(&mut self, freq: f32, bw: f32) {
        let r = (-std::f32::consts::PI * bw / SR as f32).exp();
        self.a1 = -2.0 * r * (2.0 * std::f32::consts::PI * freq / SR as f32).cos();
        self.a2 = r * r;
        self.b0 = 1.0 - r;
    }
    fn run(&mut self, x: f32) -> f32 {
        let y = self.b0 * x - self.a1 * self.z1 - self.a2 * self.z2;
        self.z2 = self.z1;
        self.z1 = y;
        y
    }
}

/// Synthetic speech: a glottal pulse train with pitch contour through
/// three time-varying formant resonators, syllabic amplitude envelope,
/// and unvoiced fricative bursts between syllables. Mono.
pub fn synth_speech(dur_s: f32, seed: u64) -> Vec<f32> {
    let n = (SR as f32 * dur_s) as usize;
    let mut rng = Rng::new(seed);
    let vowels: [[(f32, f32); 3]; 5] = [
        [(730.0, 80.0), (1090.0, 90.0), (2440.0, 120.0)], // a
        [(270.0, 60.0), (2290.0, 100.0), (3010.0, 150.0)], // i
        [(300.0, 60.0), (870.0, 80.0), (2240.0, 120.0)],  // u
        [(530.0, 70.0), (1840.0, 90.0), (2480.0, 130.0)], // e
        [(570.0, 70.0), (840.0, 80.0), (2410.0, 120.0)],  // o
    ];
    let mut f = [
        Reson::new(730.0, 80.0),
        Reson::new(1090.0, 90.0),
        Reson::new(2440.0, 120.0),
    ];
    let mut out = vec![0.0f32; n];
    let syl = (SR as f32 * 0.22) as usize;
    let mut phase = 0.0f32;
    let mut hp_prev = 0.0f32;
    for i in 0..n {
        let t = i as f32 / SR as f32;
        let s_idx = i / syl;
        let s_pos = (i % syl) as f32 / syl as f32;
        let v = &vowels[(s_idx * 7 + 3) % 5];
        if i % syl == 0 {
            for k in 0..3 {
                f[k].set(v[k].0 * (1.0 + 0.05 * rng.uni()), v[k].1);
            }
        }
        // Voiced part: first 70 % of the syllable, raised-cosine on/off.
        let voiced_env = if s_pos < 0.7 {
            let e = (s_pos / 0.7 * std::f32::consts::PI).sin();
            e.powf(0.6)
        } else {
            0.0
        };
        let f0 = 120.0 + 35.0 * (2.0 * std::f32::consts::PI * 0.9 * t).sin() - 25.0 * s_pos
            + 8.0 * (2.0 * std::f32::consts::PI * 5.5 * t).sin();
        phase += f0 / SR as f32;
        if phase >= 1.0 {
            phase -= 1.0;
        }
        // Rosenberg-like pulse: -(1-cos) derivative shape.
        let pulse = if phase < 0.4 {
            let p = phase / 0.4;
            (std::f32::consts::PI * p).sin() * (1.0 - p)
        } else {
            0.0
        };
        let excitation = pulse * voiced_env + rng.gauss() * 0.004;
        let mut y = 0.0;
        let mut x = excitation;
        for r in f.iter_mut() {
            x = r.run(x);
            y = x;
        }
        // Unvoiced burst in the last 30 % of every other syllable.
        let fric = if s_pos >= 0.72 && s_idx % 2 == 1 {
            let w = rng.gauss();
            let hp = w - hp_prev;
            hp_prev = w;
            hp * 0.05 * ((s_pos - 0.72) / 0.28 * std::f32::consts::PI).sin()
        } else {
            0.0
        };
        out[i] = (y * 2.2 + fric).clamp(-0.95, 0.95);
    }
    // Normalise to -12 dBFS peak-ish.
    let peak = out.iter().fold(0.0f32, |m, v| m.max(v.abs())).max(1e-6);
    let g = 0.5 / peak;
    for v in out.iter_mut() {
        *v *= g;
    }
    out
}

/// Synthetic music: chords of harmonic notes with vibrato and ADSR,
/// a bass line, and a quiet pink-noise bed. Stereo (notes are panned).
pub fn synth_music(dur_s: f32, seed: u64) -> Vec<f32> {
    let n = (SR as f32 * dur_s) as usize;
    let mut rng = Rng::new(seed);
    let mut out = vec![0.0f32; n * 2];
    let scale = [261.63f32, 329.63, 392.0, 440.0, 523.25, 659.25, 783.99];
    let note_len = (SR as f32 * 0.4) as usize;
    let n_notes = n / note_len + 1;
    let mut pink_st = [[0.0f32; 3]; 2];
    // Pre-pick the chord per note slot.
    let chords: Vec<[usize; 3]> = (0..n_notes)
        .map(|k| {
            let root = (k * 3 + (k / 2)) % 4;
            [root, root + 2, (root + 4) % 7]
        })
        .collect();
    for i in 0..n {
        let t = i as f32 / SR as f32;
        let slot = i / note_len;
        let pos = (i % note_len) as f32 / note_len as f32;
        let env = if pos < 0.02 {
            pos / 0.02
        } else if pos < 0.1 {
            1.0 - 0.3 * (pos - 0.02) / 0.08
        } else if pos < 0.85 {
            0.7
        } else {
            0.7 * (1.0 - (pos - 0.85) / 0.15)
        };
        let vib = 1.0 + 0.004 * (2.0 * std::f32::consts::PI * 5.2 * t).sin();
        let mut l = 0.0f32;
        let mut r = 0.0f32;
        for (v, &deg) in chords[slot].iter().enumerate() {
            let f = scale[deg] * vib;
            let pan = [0.3, 0.5, 0.7][v];
            let mut s = 0.0f32;
            for h in 1..=8u32 {
                let a = 1.0 / (h as f32).powf(1.3);
                s += a * (2.0 * std::f32::consts::PI * f * h as f32 * t).sin();
            }
            s *= env * 0.11;
            l += s * (1.0 - pan);
            r += s * pan;
        }
        // Bass: root two octaves down, longer notes.
        let bslot = i / (note_len * 2);
        let bf = scale[chords[(bslot * 2).min(n_notes - 1)][0]] / 4.0;
        let bpos = (i % (note_len * 2)) as f32 / (note_len * 2) as f32;
        let benv = if bpos < 0.01 {
            bpos / 0.01
        } else {
            (1.0 - bpos).powf(0.5)
        };
        let bass = benv
            * 0.18
            * ((2.0 * std::f32::consts::PI * bf * t).sin()
                + 0.4 * (2.0 * std::f32::consts::PI * bf * 2.0 * t).sin());
        l += bass;
        r += bass;
        l += pink(&mut rng, &mut pink_st[0]) * 0.01;
        r += pink(&mut rng, &mut pink_st[1]) * 0.01;
        out[2 * i] = l.clamp(-0.95, 0.95);
        out[2 * i + 1] = r.clamp(-0.95, 0.95);
    }
    out
}

/// Drum-like transients: kick (pitch-swept sine, hard attack), snare
/// (noise burst + 190 Hz body), hi-hat (short high-passed noise), in a
/// fixed 130 BPM pattern. Stereo with slight pan/decorrelation.
pub fn synth_transients(dur_s: f32, seed: u64) -> Vec<f32> {
    let n = (SR as f32 * dur_s) as usize;
    let mut rng = Rng::new(seed);
    let mut out = vec![0.0f32; n * 2];
    let step = (SR as f32 * 60.0 / 130.0 / 4.0) as usize; // 16th notes
    let mut hp_prev = 0.0f32;
    for i in 0..n {
        let s = i / step;
        let pos = i % step;
        let tp = pos as f32 / SR as f32;
        let mut l = 0.0f32;
        let mut r = 0.0f32;
        // Kick on beats 0 and 8 of 16.
        if s % 16 == 0 || s % 16 == 8 || s % 16 == 11 {
            let fk = 40.0 + 90.0 * (-tp * 40.0).exp();
            let env = (-tp * 9.0).exp();
            let ph =
                2.0 * std::f32::consts::PI * (40.0 * tp + 90.0 / 40.0 * (1.0 - (-tp * 40.0).exp()));
            let _ = fk;
            let k = ph.sin() * env * 0.8;
            l += k;
            r += k;
        }
        // Snare on 4 and 12.
        if s % 16 == 4 || s % 16 == 12 {
            let env = (-tp * 22.0).exp();
            let body = (2.0 * std::f32::consts::PI * 190.0 * tp).sin() * (-tp * 35.0).exp() * 0.5;
            let nz = rng.gauss() * env * 0.35;
            l += body + nz;
            r += body + rng.gauss() * env * 0.35;
        }
        // Hi-hat on every even 16th.
        if s % 2 == 0 {
            let env = (-tp * 90.0).exp();
            let w = rng.gauss();
            let hp = w - hp_prev;
            hp_prev = w;
            let h = hp * env * 0.25;
            l += h * 0.8;
            r += h * 1.2;
        }
        out[2 * i] = l.clamp(-0.95, 0.95);
        out[2 * i + 1] = r.clamp(-0.95, 0.95);
    }
    out
}

/// The README two-tone fixture (0.3·sin 440 Hz + 0.18·sin 3517 Hz),
/// right channel with the second partial phase-shifted. Stereo.
pub fn synth_tones(dur_s: f32) -> Vec<f32> {
    let n = (SR as f32 * dur_s) as usize;
    let mut out = vec![0.0f32; n * 2];
    for i in 0..n {
        let t = i as f32 / SR as f32;
        let a = 0.3 * (2.0 * std::f32::consts::PI * 440.0 * t).sin();
        let b = 0.18 * (2.0 * std::f32::consts::PI * 3517.0 * t).sin();
        let b2 = 0.18 * (2.0 * std::f32::consts::PI * 3517.0 * t + 1.1).sin();
        out[2 * i] = a + b;
        out[2 * i + 1] = a + b2;
    }
    out
}

/// Pink noise at about -20 dBFS RMS, partially correlated stereo.
pub fn synth_pink(dur_s: f32, seed: u64) -> Vec<f32> {
    let n = (SR as f32 * dur_s) as usize;
    let mut rng = Rng::new(seed);
    let mut st = [[0.0f32; 3]; 3];
    let mut out = vec![0.0f32; n * 2];
    for i in 0..n {
        let m = pink(&mut rng, &mut st[0]);
        let a = pink(&mut rng, &mut st[1]);
        let b = pink(&mut rng, &mut st[2]);
        out[2 * i] = ((m + 0.5 * a) * 0.35).clamp(-0.95, 0.95);
        out[2 * i + 1] = ((m + 0.5 * b) * 0.35).clamp(-0.95, 0.95);
    }
    out
}

/// 5.1 mix in WAV order (FL FR FC LFE BL BR): music front, speech
/// centre, delayed/low-passed transients + pink ambience in the
/// surrounds, low-passed kick in the LFE.
pub fn synth_51(dur_s: f32, seed: u64) -> Vec<f32> {
    let n = (SR as f32 * dur_s) as usize;
    let music = synth_music(dur_s, seed);
    let speech = synth_speech(dur_s, seed + 7);
    let drums = synth_transients(dur_s, seed + 11);
    let mut rng = Rng::new(seed + 99);
    let mut pst = [[0.0f32; 3]; 2];
    let mut lfe_st = [0.0f32; 3];
    let mut sur_st = [0.0f32; 2];
    let mut out = vec![0.0f32; n * 6];
    let delay = 480usize; // 10 ms surround delay
    for i in 0..n {
        let fl = music[2 * i] * 0.8 + drums[2 * i] * 0.5;
        let fr = music[2 * i + 1] * 0.8 + drums[2 * i + 1] * 0.5;
        let fc = speech[i] * 0.9;
        let d = i.saturating_sub(delay);
        let sl_raw = drums[2 * d] * 0.35 + music[2 * d + 1] * 0.2;
        let sr_raw = drums[2 * d + 1] * 0.35 + music[2 * d] * 0.2;
        let sl = lp1(sl_raw, &mut sur_st[0], 0.35) + pink(&mut rng, &mut pst[0]) * 0.03;
        let sr = lp1(sr_raw, &mut sur_st[1], 0.35) + pink(&mut rng, &mut pst[1]) * 0.03;
        // LFE: kick content low-passed hard (three cascaded poles ≈ 90 Hz).
        let mut k = (drums[2 * i] + drums[2 * i + 1]) * 0.5;
        let c = 2.0 * std::f32::consts::PI * 90.0 / SR as f32;
        for s in lfe_st.iter_mut() {
            k = lp1(k, s, c);
        }
        out[6 * i] = fl.clamp(-0.95, 0.95);
        out[6 * i + 1] = fr.clamp(-0.95, 0.95);
        out[6 * i + 2] = fc.clamp(-0.95, 0.95);
        out[6 * i + 3] = (k * 1.5).clamp(-0.95, 0.95);
        out[6 * i + 4] = sl.clamp(-0.95, 0.95);
        out[6 * i + 5] = sr.clamp(-0.95, 0.95);
    }
    out
}

/// 4th-order Butterworth low-pass (two cascaded RBJ biquads, Q =
/// 0.5412 / 1.3066) at `fc` Hz applied in place to interleaved PCM.
/// Keeps the synthetic clips inside a natural ~16 kHz audio band so the
/// scores are not dominated by the encoders' bandwidth choices.
pub fn lowpass_in_place(pcm: &mut [f32], channels: usize, fc: f32) {
    let w0 = 2.0 * std::f32::consts::PI * fc / SR as f32;
    let (sw, cw) = (w0.sin(), w0.cos());
    let stages: Vec<[f32; 5]> = [0.5412f32, 1.3066]
        .iter()
        .map(|q| {
            let alpha = sw / (2.0 * q);
            let a0 = 1.0 + alpha;
            [
                (1.0 - cw) / 2.0 / a0,
                (1.0 - cw) / a0,
                (1.0 - cw) / 2.0 / a0,
                -2.0 * cw / a0,
                (1.0 - alpha) / a0,
            ]
        })
        .collect();
    for c in 0..channels {
        let mut st = [[0.0f32; 4]; 2];
        let n = pcm.len() / channels;
        for i in 0..n {
            let mut x = pcm[i * channels + c];
            for (k, b) in stages.iter().enumerate() {
                let y = b[0] * x + b[1] * st[k][0] + b[2] * st[k][1]
                    - b[3] * st[k][2]
                    - b[4] * st[k][3];
                st[k][1] = st[k][0];
                st[k][0] = x;
                st[k][3] = st[k][2];
                st[k][2] = y;
                x = y;
            }
            pcm[i * channels + c] = x;
        }
    }
}

/// The full corpus. `dur_s` seconds per clip, band-limited to 16 kHz.
pub fn corpus(dur_s: f32) -> Vec<Signal> {
    let mut v = corpus_raw(dur_s);
    for s in v.iter_mut() {
        lowpass_in_place(&mut s.pcm, s.channels, 16_000.0);
    }
    v
}

fn corpus_raw(dur_s: f32) -> Vec<Signal> {
    vec![
        Signal {
            name: "speech",
            channels: 1,
            lfe: false,
            pcm: synth_speech(dur_s, 0x5EED),
        },
        Signal {
            name: "music",
            channels: 2,
            lfe: false,
            pcm: synth_music(dur_s, 0x1234),
        },
        Signal {
            name: "transients",
            channels: 2,
            lfe: false,
            pcm: synth_transients(dur_s, 0x9876),
        },
        Signal {
            name: "tones",
            channels: 2,
            lfe: false,
            pcm: synth_tones(dur_s),
        },
        Signal {
            name: "pink",
            channels: 2,
            lfe: false,
            pcm: synth_pink(dur_s, 0x4242),
        },
        Signal {
            name: "mix-5.1",
            channels: 6,
            lfe: true,
            pcm: synth_51(dur_s, 0x51),
        },
    ]
}

pub fn signal_by_name(dur_s: f32, name: &str) -> Signal {
    corpus(dur_s)
        .into_iter()
        .find(|s| s.name == name)
        .unwrap_or_else(|| panic!("no corpus signal named {name}"))
}

// ---------------------------------------------------------------------------
// Encoders / decoders
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Codec {
    Ac3,
    Eac3,
}

impl Codec {
    pub fn id(self) -> &'static str {
        match self {
            Codec::Ac3 => "ac3",
            Codec::Eac3 => "eac3",
        }
    }
}

#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub enum Enc {
    /// Our encoder with registry options.
    Ours(Codec, Vec<(String, String)>),
    /// The black-box reference encoder.
    Reference(Codec),
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Dec {
    Ours,
    Reference,
}

pub fn ours(codec: Codec) -> Enc {
    Enc::Ours(codec, Vec::new())
}

pub fn ours_with(codec: Codec, opts: &[(&str, &str)]) -> Enc {
    Enc::Ours(
        codec,
        opts.iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
    )
}

pub fn ffmpeg_present() -> bool {
    static PRESENT: OnceLock<bool> = OnceLock::new();
    *PRESENT.get_or_init(|| {
        Command::new("ffmpeg")
            .arg("-version")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .map(|s| s.success())
            .unwrap_or(false)
    })
}

fn tmp_path(tag: &str, ext: &str) -> std::path::PathBuf {
    static CTR: AtomicUsize = AtomicUsize::new(0);
    let n = CTR.fetch_add(1, Ordering::Relaxed);
    std::env::temp_dir().join(format!(
        "oxideav_ac3_rd_{}_{}_{}.{}",
        std::process::id(),
        n,
        tag,
        ext
    ))
}

/// WAV-order interleaved f32 → the encoder's input order (AC-3 / E-AC-3
/// both take 5.1 as L,C,R,Ls,Rs,LFE) as S16LE bytes.
fn to_encoder_s16(sig: &Signal) -> Vec<u8> {
    let nch = sig.channels;
    let map: Vec<usize> = match (nch, sig.lfe) {
        (6, true) => vec![0, 2, 1, 4, 5, 3], // FL FR FC LFE BL BR → L C R Ls Rs LFE
        _ => (0..nch).collect(),
    };
    let mut out = Vec::with_capacity(sig.pcm.len() * 2);
    for fr in sig.pcm.chunks_exact(nch) {
        for &src in &map {
            let v = (fr[src] * 32767.0).round().clamp(-32768.0, 32767.0) as i16;
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    out
}

/// WAV-order S16LE bytes (what the reference tools consume/produce).
fn to_wav_s16(sig: &Signal) -> Vec<u8> {
    let mut out = Vec::with_capacity(sig.pcm.len() * 2);
    for &s in &sig.pcm {
        let v = (s * 32767.0).round().clamp(-32768.0, 32767.0) as i16;
        out.extend_from_slice(&v.to_le_bytes());
    }
    out
}

fn s16_to_f32(bytes: &[u8]) -> Vec<f32> {
    bytes
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
        .collect()
}

/// Encode with our encoder through the registry path.
pub fn encode_ours(sig: &Signal, codec: Codec, kbps: u32, opts: &[(String, String)]) -> Vec<u8> {
    let mut params = CodecParameters::audio(CodecId::new(codec.id()));
    params.sample_rate = Some(SR);
    params.channels = Some(sig.channels as u16);
    params.sample_format = Some(SampleFormat::S16);
    params.bit_rate = Some(kbps as u64 * 1000);
    let mut o = CodecOptions::new();
    for (k, v) in opts {
        o = o.set(k.as_str(), v.as_str());
    }
    params.options = o;
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    let mut enc = reg.first_encoder(&params).expect("registry encoder");
    let bytes = to_encoder_s16(sig);
    // Feed in ~0.25 s chunks so the encoder's pending-sample path is
    // exercised like a streaming caller would.
    let chunk_samples: usize = std::env::var("OXIDEAV_RD_CHUNK")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(12_000);
    let chunk = chunk_samples * sig.channels * 2;
    let mut pos = 0usize;
    let mut pts = 0i64;
    let mut out = Vec::new();
    while pos < bytes.len() {
        let end = (pos + chunk).min(bytes.len());
        let nsamp = (end - pos) / (sig.channels * 2);
        enc.send_frame(&Frame::Audio(AudioFrame {
            samples: nsamp as u32,
            pts: Some(pts),
            data: vec![bytes[pos..end].to_vec()],
        }))
        .expect("send_frame");
        pts += nsamp as i64;
        pos = end;
        loop {
            match enc.receive_packet() {
                Ok(p) => out.extend_from_slice(&p.data),
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("receive_packet: {e:?}"),
            }
        }
    }
    enc.flush().expect("flush");
    loop {
        match enc.receive_packet() {
            Ok(p) => out.extend_from_slice(&p.data),
            Err(Error::NeedMore) | Err(Error::Eof) => break,
            Err(e) => panic!("receive_packet: {e:?}"),
        }
    }
    out
}

/// Encode with the black-box reference encoder. `None` when the
/// binary is unavailable or refuses the configuration.
pub fn encode_reference(sig: &Signal, codec: Codec, kbps: u32) -> Option<Vec<u8>> {
    if !ffmpeg_present() {
        return None;
    }
    let inp = tmp_path("ref_in", "pcm");
    let outp = tmp_path("ref_out", codec.id());
    std::fs::write(&inp, to_wav_s16(sig)).ok()?;
    let mut cmd = Command::new("ffmpeg");
    cmd.args(["-y", "-hide_banner", "-loglevel", "error", "-f", "s16le"])
        .args(["-ar", &SR.to_string(), "-ac", &sig.channels.to_string()]);
    if sig.channels == 6 {
        cmd.args(["-channel_layout", "5.1"]);
    }
    cmd.arg("-i")
        .arg(&inp)
        .args([
            "-c:a",
            codec.id(),
            "-b:a",
            &format!("{kbps}k"),
            "-f",
            codec.id(),
        ])
        .arg(&outp);
    let st = cmd.stdin(Stdio::null()).status().ok()?;
    let _ = std::fs::remove_file(&inp);
    if !st.success() {
        let _ = std::fs::remove_file(&outp);
        return None;
    }
    let bytes = std::fs::read(&outp).ok()?;
    let _ = std::fs::remove_file(&outp);
    Some(bytes)
}

/// Decode with the black-box reference decoder to WAV-order f32.
pub fn decode_reference(es: &[u8], codec: Codec, channels: usize) -> Option<Vec<f32>> {
    if !ffmpeg_present() {
        return None;
    }
    let inp = tmp_path("dec_in", codec.id());
    let outp = tmp_path("dec_out", "pcm");
    std::fs::write(&inp, es).ok()?;
    let st = Command::new("ffmpeg")
        .args([
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            codec.id(),
            "-i",
        ])
        .arg(&inp)
        .args(["-f", "s16le", "-acodec", "pcm_s16le"])
        .args(["-ac", &channels.to_string(), "-ar", &SR.to_string()])
        .arg(&outp)
        .stdin(Stdio::null())
        .status()
        .ok()?;
    let _ = std::fs::remove_file(&inp);
    if !st.success() {
        let _ = std::fs::remove_file(&outp);
        return None;
    }
    let bytes = std::fs::read(&outp).ok()?;
    let _ = std::fs::remove_file(&outp);
    Some(s16_to_f32(&bytes))
}

/// Split an AC-3 / E-AC-3 elementary stream into decoder packets
/// (E-AC-3 dependent substreams are grouped with their independent
/// frame).
fn split_packets(es: &[u8], codec: Codec) -> Vec<Vec<u8>> {
    let mut frames: Vec<(usize, usize)> = Vec::new();
    let mut off = 0usize;
    while off + 6 <= es.len() {
        if es[off] != 0x0B || es[off + 1] != 0x77 {
            off += 1;
            continue;
        }
        let flen = match codec {
            Codec::Ac3 => {
                let si = match oxideav_ac3::syncinfo::parse(&es[off..]) {
                    Ok(si) => si,
                    Err(_) => break,
                };
                si.frame_length as usize
            }
            Codec::Eac3 => {
                let frmsiz = (((es[off + 2] & 0x07) as u32) << 8) | es[off + 3] as u32;
                ((frmsiz + 1) * 2) as usize
            }
        };
        if flen < 6 || off + flen > es.len() {
            break;
        }
        frames.push((off, flen));
        off += flen;
    }
    let strmtyp = |o: usize| es[o + 2] >> 6;
    let mut packets = Vec::new();
    let mut i = 0usize;
    while i < frames.len() {
        let (start, mut len) = frames[i];
        let mut j = i + 1;
        if codec == Codec::Eac3 {
            while j < frames.len() && strmtyp(frames[j].0) == 1 {
                len += frames[j].1;
                j += 1;
            }
        }
        packets.push(es[start..start + len].to_vec());
        i = j;
    }
    packets
}

/// Decode with our registry decoder to WAV-order f32.
pub fn decode_ours(es: &[u8], codec: Codec, channels: usize) -> Vec<f32> {
    let mut reg = CodecRegistry::new();
    oxideav_ac3::register_codecs(&mut reg);
    let params = CodecParameters::audio(CodecId::new(codec.id()));
    let mut dec = reg.first_decoder(&params).expect("registry decoder");
    let mut out = Vec::new();
    let mut pts = 0i64;
    for pkt in split_packets(es, codec) {
        let p = Packet::new(0, TimeBase::new(1, SR as i64), pkt).with_pts(pts);
        dec.send_packet(&p).expect("send_packet");
        loop {
            match dec.receive_frame() {
                Ok(Frame::Audio(a)) => {
                    assert_eq!(
                        a.data[0].len(),
                        a.samples as usize * channels * 2,
                        "decoder channel count mismatch"
                    );
                    pts += a.samples as i64;
                    out.extend(s16_to_f32(&a.data[0]));
                }
                Ok(_) => {}
                Err(Error::NeedMore) | Err(Error::Eof) => break,
                Err(e) => panic!("receive_frame: {e:?}"),
            }
        }
    }
    out
}

/// Run one encoder on one signal at one rate.
pub fn encode(sig: &Signal, enc: &Enc, kbps: u32) -> Option<Vec<u8>> {
    match enc {
        Enc::Ours(codec, opts) => Some(encode_ours(sig, *codec, kbps, opts)),
        Enc::Reference(codec) => encode_reference(sig, *codec, kbps),
    }
}

pub fn decode(es: &[u8], enc: &Enc, dec: Dec, channels: usize) -> Option<Vec<f32>> {
    let codec = match enc {
        Enc::Ours(c, _) | Enc::Reference(c) => *c,
    };
    match dec {
        Dec::Ours => Some(decode_ours(es, codec, channels)),
        Dec::Reference => decode_reference(es, codec, channels),
    }
}

// ---------------------------------------------------------------------------
// Alignment
// ---------------------------------------------------------------------------

/// Whole-clip cross-correlation lag search (0..=1024 samples) on the
/// channel sum. Broadband inputs only — tonal clips alias.
pub fn find_lag(orig: &[f32], dec: &[f32], channels: usize) -> usize {
    let n_o = orig.len() / channels;
    let n_d = dec.len() / channels;
    let mut best = 0usize;
    let mut best_c = f64::NEG_INFINITY;
    let sum = |pcm: &[f32], i: usize| -> f64 {
        (0..channels).map(|c| pcm[i * channels + c] as f64).sum()
    };
    let so: Vec<f64> = (0..n_o).map(|i| sum(orig, i)).collect();
    let sd: Vec<f64> = (0..n_d).map(|i| sum(dec, i)).collect();
    for lag in 0..=1024usize {
        let n = n_o.min(n_d.saturating_sub(lag));
        if n < 4096 {
            continue;
        }
        let mut acc = 0.0f64;
        let mut eo = 0.0f64;
        let mut ed = 0.0f64;
        for i in 1024..n {
            acc += so[i] * sd[i + lag];
            eo += so[i] * so[i];
            ed += sd[i + lag] * sd[i + lag];
        }
        let c = acc / (eo.sqrt() * ed.sqrt()).max(1e-30);
        if c > best_c {
            best_c = c;
            best = lag;
        }
    }
    best
}

/// Decoder-chain delay per (encoder kind, decoder), calibrated once on
/// the broadband pink clip and cached for the process.
pub fn path_lag(enc: &Enc, dec: Dec) -> Option<usize> {
    static CACHE: OnceLock<Mutex<HashMap<(String, Dec), usize>>> = OnceLock::new();
    let key_enc = match enc {
        Enc::Ours(c, _) => format!("ours-{}", c.id()),
        Enc::Reference(c) => format!("ref-{}", c.id()),
    };
    let key = (key_enc, dec);
    let cache = CACHE.get_or_init(|| Mutex::new(HashMap::new()));
    if let Some(v) = cache.lock().unwrap().get(&key) {
        return Some(*v);
    }
    let probe = Signal {
        name: "probe",
        channels: 2,
        lfe: false,
        pcm: synth_pink(1.0, 0xC0FFEE),
    };
    let calib_enc = match enc {
        Enc::Ours(c, _) => Enc::Ours(*c, Vec::new()),
        Enc::Reference(c) => Enc::Reference(*c),
    };
    let es = encode(&probe, &calib_enc, 192)?;
    let pcm = decode(&es, &calib_enc, dec, 2)?;
    let lag = find_lag(&probe.pcm, &pcm, 2);
    cache.lock().unwrap().insert(key, lag);
    Some(lag)
}

// ---------------------------------------------------------------------------
// Scoring
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub struct Score {
    /// Per-channel SNR (dB), WAV order.
    pub snr_ch: Vec<f64>,
    /// Mean NMR (dB) over audible (block, band) cells, fbw channels.
    pub nmr_mean: f64,
    /// Worst per-band mean NMR (dB) across the 50 §7.2 bands.
    pub nmr_band_max: f64,
    /// Per-band mean NMR (dB), 50 §7.2 bands (NaN where never audible).
    pub nmr_band: [f64; 50],
}

impl Score {
    pub fn snr_min(&self) -> f64 {
        self.snr_ch.iter().cloned().fold(f64::INFINITY, f64::min)
    }
    pub fn snr_mean(&self) -> f64 {
        self.snr_ch.iter().sum::<f64>() / self.snr_ch.len() as f64
    }
}

/// psd units per dB (§7.2.2.2: one exponent step = 128 units = 6.02 dB).
const UNITS_PER_DB: f64 = 128.0 / 6.020_599_913;

/// Energy → §7.2 psd units. A coefficient of magnitude `m·2^-e`
/// (`m ∈ [0.5, 1)`) carries `psd = 3072 − 128e`; we map through the
/// band-centre magnitude `m ≈ 0.75`.
fn energy_to_psd(e: f64) -> f64 {
    let db = 10.0 * e.max(1e-30).log10();
    3072.0 + (db + 2.5) * UNITS_PER_DB
}

fn logadd_f(a: f64, b: f64) -> f64 {
    // 10·log10(10^(a/10) + 10^(b/10)) in psd units.
    let a_db = a / UNITS_PER_DB;
    let b_db = b / UNITS_PER_DB;
    let m = a_db.max(b_db);
    let s = m + 10.0 * (10f64.powf((a_db - m) / 10.0) + 10f64.powf((b_db - m) / 10.0)).log10();
    s * UNITS_PER_DB
}

/// §7.2.2 parametric mask (basic-encoder parameter set: sdcycod=2,
/// fdcycod=1, sgaincod=1, dbpbcod=2, floorcod=4, fgaincod=4) for one
/// block of one fbw channel. `psd[bin]` in psd units; returns the
/// band mask (50 entries, psd units) for bins `0..end`.
pub fn spec_mask(psd: &[f64; 256], end: usize, fscod: usize) -> [f64; 50] {
    let sdecay = SLOWDEC[2] as f64;
    let fdecay = FASTDEC[1] as f64;
    let sgain = SLOWGAIN[1] as f64;
    let dbknee = DBPBTAB[2] as f64;
    let fgain = FASTGAIN[4] as f64;
    let mut bndpsd = [f64::NEG_INFINITY; 50];
    let bndend = MASKTAB[end - 1] as usize + 1;
    for k in 0..bndend {
        let lo = BNDTAB[k] as usize;
        let hi = (lo + BNDSZ[k] as usize).min(end);
        let mut acc = psd[lo];
        for &p in &psd[lo + 1..hi] {
            acc = logadd_f(acc, p);
        }
        bndpsd[k] = acc;
    }
    let mut excite = [0.0f64; 50];
    // §7.2.2.4 fbw path — lowcomp per the spec's `calc_lowcomp`.
    let calc_lowcomp = |a: f64, b0: f64, b1: f64, bin: usize| -> f64 {
        if bin < 7 {
            if b0 + 256.0 == b1 {
                384.0
            } else if b0 > b1 {
                (a - 64.0).max(0.0)
            } else {
                a
            }
        } else if bin < 20 {
            if b0 + 256.0 == b1 {
                320.0
            } else if b0 > b1 {
                (a - 64.0).max(0.0)
            } else {
                a
            }
        } else {
            (a - 128.0).max(0.0)
        }
    };
    let mut lowcomp = 0.0f64;
    if bndend > 0 {
        lowcomp = calc_lowcomp(lowcomp, bndpsd[0], bndpsd[1], 0);
        excite[0] = bndpsd[0] - fgain - lowcomp;
    }
    if bndend > 1 {
        lowcomp = calc_lowcomp(lowcomp, bndpsd[1], bndpsd[2], 1);
        excite[1] = bndpsd[1] - fgain - lowcomp;
    }
    let mut begin = 7.min(bndend);
    let mut fastleak = 0.0f64;
    let mut slowleak = 0.0f64;
    for bin in 2..7.min(bndend) {
        lowcomp = calc_lowcomp(lowcomp, bndpsd[bin], bndpsd[bin + 1], bin);
        fastleak = bndpsd[bin] - fgain;
        slowleak = bndpsd[bin] - sgain;
        excite[bin] = fastleak - lowcomp;
        if bndpsd[bin] <= bndpsd[bin + 1] {
            begin = bin + 1;
            break;
        }
    }
    for bin in begin..22.min(bndend) {
        lowcomp = calc_lowcomp(lowcomp, bndpsd[bin], bndpsd[bin + 1], bin);
        fastleak = (fastleak - fdecay).max(bndpsd[bin] - fgain);
        slowleak = (slowleak - sdecay).max(bndpsd[bin] - sgain);
        excite[bin] = (fastleak - lowcomp).max(slowleak);
    }
    for bin in 22.min(bndend)..bndend {
        fastleak = (fastleak - fdecay).max(bndpsd[bin] - fgain);
        slowleak = (slowleak - sdecay).max(bndpsd[bin] - sgain);
        excite[bin] = fastleak.max(slowleak);
    }
    let mut mask = [f64::NEG_INFINITY; 50];
    for bin in 0..bndend {
        let mut exc = excite[bin];
        if bndpsd[bin] < dbknee {
            exc += (dbknee - bndpsd[bin]) / 4.0;
        }
        mask[bin] = exc.max(HTH[fscod][bin] as f64);
    }
    mask
}

/// Band energies of a windowed 512-sample MDCT block.
fn block_band_psd(buf: &[f32; 512], end: usize) -> ([f64; 256], [f64; 50]) {
    let mut win = [0.0f32; 512];
    for n in 0..256 {
        win[n] = buf[n] * WINDOW[n];
        win[511 - n] = buf[511 - n] * WINDOW[n];
    }
    let mut coef = [0.0f32; 256];
    mdct_512(&win, &mut coef);
    let mut psd = [0.0f64; 256];
    for k in 0..256 {
        psd[k] = energy_to_psd((coef[k] as f64).powi(2));
    }
    let mut band = [f64::NEG_INFINITY; 50];
    let bndend = MASKTAB[end - 1] as usize + 1;
    for k in 0..bndend {
        let lo = BNDTAB[k] as usize;
        let hi = (lo + BNDSZ[k] as usize).min(end);
        let e: f64 = coef[lo..hi].iter().map(|c| (*c as f64).powi(2)).sum();
        band[k] = energy_to_psd(e);
    }
    (psd, band)
}

/// Score `dec` against `orig` (both WAV-order interleaved, same
/// channel count) with the decoder chain delay `lag`. The first and
/// last 1024 samples are excluded.
pub fn score(orig: &[f32], dec: &[f32], channels: usize, lfe: bool, lag: usize) -> Score {
    let n_o = orig.len() / channels;
    let n_d = dec.len() / channels;
    let n = n_o.min(n_d.saturating_sub(lag));
    assert!(n > 4096, "decoded clip too short ({n} samples)");
    let lo = 1024usize;
    let hi = n - 1024;
    let mut snr_ch = Vec::with_capacity(channels);
    for c in 0..channels {
        let mut es = 0.0f64;
        let mut ee = 0.0f64;
        for i in lo..hi {
            let s = orig[i * channels + c] as f64;
            let d = dec[(i + lag) * channels + c] as f64;
            es += s * s;
            ee += (s - d) * (s - d);
        }
        snr_ch.push(10.0 * (es / ee.max(1e-30)).log10());
    }
    // NMR over fbw channels (WAV index 3 is the LFE in 5.1).
    let end = 253usize;
    let mut sum = [0.0f64; 50];
    let mut cnt = [0usize; 50];
    let mut buf_s = [0.0f32; 512];
    let mut buf_e = [0.0f32; 512];
    for c in 0..channels {
        if lfe && channels == 6 && c == 3 {
            continue;
        }
        let mut start = lo;
        while start + 512 <= hi {
            for i in 0..512 {
                let s = orig[(start + i) * channels + c];
                let d = dec[(start + i + lag) * channels + c];
                buf_s[i] = s;
                buf_e[i] = s - d;
            }
            let (psd_s, _) = block_band_psd(&buf_s, end);
            let (_, band_e) = block_band_psd(&buf_e, end);
            let mask = spec_mask(&psd_s, end, 0);
            let bndend = MASKTAB[end - 1] as usize + 1;
            // Band psd of the signal for the audibility gate.
            for k in 0..bndend {
                let lo_b = BNDTAB[k] as usize;
                let hi_b = (lo_b + BNDSZ[k] as usize).min(end);
                let mut sig_band = psd_s[lo_b];
                for &p in &psd_s[lo_b + 1..hi_b] {
                    sig_band = logadd_f(sig_band, p);
                }
                if sig_band <= HTH[0][k] as f64 {
                    continue;
                }
                let nmr_db = (band_e[k] - mask[k]) / UNITS_PER_DB;
                sum[k] += nmr_db;
                cnt[k] += 1;
            }
            start += 256;
        }
    }
    let mut nmr_band = [f64::NAN; 50];
    let mut total = 0.0f64;
    let mut total_n = 0usize;
    let mut worst = f64::NEG_INFINITY;
    for k in 0..50 {
        if cnt[k] > 0 {
            nmr_band[k] = sum[k] / cnt[k] as f64;
            total += sum[k];
            total_n += cnt[k];
            worst = worst.max(nmr_band[k]);
        }
    }
    Score {
        snr_ch,
        nmr_mean: if total_n > 0 {
            total / total_n as f64
        } else {
            f64::NAN
        },
        nmr_band_max: worst,
        nmr_band,
    }
}

/// Worst-channel SNR (dB) at a given lag — cheap alignment probe.
pub fn snr_min_at(orig: &[f32], dec: &[f32], channels: usize, lag: usize) -> f64 {
    let n_o = orig.len() / channels;
    let n_d = dec.len() / channels;
    let n = n_o.min(n_d.saturating_sub(lag));
    if n < 4096 {
        return f64::NEG_INFINITY;
    }
    let mut worst = f64::INFINITY;
    for c in 0..channels {
        let mut es = 0.0f64;
        let mut ee = 0.0f64;
        for i in 1024..n - 1024 {
            let s = orig[i * channels + c] as f64;
            let d = dec[(i + lag) * channels + c] as f64;
            es += s * s;
            ee += (s - d) * (s - d);
        }
        worst = worst.min(10.0 * (es / ee.max(1e-30)).log10());
    }
    worst
}

/// Ungated per-band energy profile (dB re full scale) of the source
/// and of the error for channel `c` — a diagnostic, not a score.
pub fn band_profile(
    orig: &[f32],
    dec: &[f32],
    channels: usize,
    c: usize,
    lag: usize,
) -> ([f64; 50], [f64; 50]) {
    let n_o = orig.len() / channels;
    let n_d = dec.len() / channels;
    let n = n_o.min(n_d.saturating_sub(lag));
    let end = 253usize;
    let mut es = [0.0f64; 50];
    let mut ee = [0.0f64; 50];
    let mut buf_s = [0.0f32; 512];
    let mut buf_e = [0.0f32; 512];
    let mut start = 1024usize;
    let mut blocks = 0usize;
    while start + 512 <= n - 1024 {
        for i in 0..512 {
            let s = orig[(start + i) * channels + c];
            let d = dec[(start + i + lag) * channels + c];
            buf_s[i] = s;
            buf_e[i] = s - d;
        }
        let (_, bs) = block_band_psd(&buf_s, end);
        let (_, be) = block_band_psd(&buf_e, end);
        for k in 0..50 {
            if bs[k].is_finite() {
                es[k] += 10f64.powf((bs[k] - 3072.0) / UNITS_PER_DB / 10.0);
                ee[k] += 10f64.powf((be[k] - 3072.0) / UNITS_PER_DB / 10.0);
            }
        }
        start += 256;
        blocks += 1;
    }
    let mut out_s = [f64::NAN; 50];
    let mut out_e = [f64::NAN; 50];
    for k in 0..50 {
        if es[k] > 0.0 {
            out_s[k] = 10.0 * (es[k] / blocks as f64).log10();
            out_e[k] = 10.0 * (ee[k] / blocks as f64).log10();
        }
    }
    (out_s, out_e)
}

/// Encode + decode + score one cell. `None` when the black-box tool is
/// unavailable / refuses the configuration.
pub fn measure(sig: &Signal, enc: &Enc, dec: Dec, kbps: u32) -> Option<(Score, usize)> {
    let es = encode(sig, enc, kbps)?;
    let pcm = decode(&es, enc, dec, sig.channels)?;
    let lag = path_lag(enc, dec)?;
    Some((score(&sig.pcm, &pcm, sig.channels, sig.lfe, lag), es.len()))
}

/// Human-readable one-line summary.
pub fn fmt_score(s: &Score) -> String {
    format!(
        "snr_min={:6.2} snr_mean={:6.2} nmr={:6.2} nmr_bandmax={:6.2}",
        s.snr_min(),
        s.snr_mean(),
        s.nmr_mean,
        s.nmr_band_max
    )
}

/// Write a WAV-order f32 clip to a temp file as S16LE (debug aid).
pub fn dump_pcm(path: &std::path::Path, pcm: &[f32]) {
    let mut f = std::fs::File::create(path).expect("create pcm dump");
    for &s in pcm {
        let v = (s * 32767.0).round().clamp(-32768.0, 32767.0) as i16;
        f.write_all(&v.to_le_bytes()).unwrap();
    }
}
