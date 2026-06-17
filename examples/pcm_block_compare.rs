//! Per-BLOCK PCM diff vs ffmpeg reference (cf. `pcm_compare.rs`, which is
//! per-FRAME only). AC-3 emits 6 audio blocks/frame, 256 samples/channel each.
//! With 50%% overlap-add a block's coefficient error smears FORWARD into the
//! next block's output but never backward, so slicing the already-decoded,
//! frame-aligned PCM into 6 sub-ranges localizes the first bad block.
//!
//! Frame layout in `af.data[0]`: 1536 samples/channel * 2 ch interleaved =
//! 3072 i16 samples = 6 blocks * 512 interleaved samples (256/ch).
//!
//! Reference: raw f32le stereo from `ffmpeg -i multi.ac3 -f f32le ffmpeg.pcm`.
//!
//! Run: cargo run --example pcm_block_compare
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

const BLOCKS: usize = 6;
const SAMP_PER_BLK_CH: usize = 256;
const CH: usize = 2;
const INTERLEAVED_PER_BLK: usize = SAMP_PER_BLK_CH * CH; // 512
const INTERLEAVED_PER_FRAME: usize = INTERLEAVED_PER_BLK * BLOCKS; // 3072

fn main() {
    let dir = std::env::var("TEMP").unwrap() + r"\oxideav_repro";
    // arg1 = stem (default "multi"): reads <stem>.ac3 and <stem>.pcm
    let stem = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "multi".to_string());
    let path = format!("{dir}\\{stem}.ac3");
    let refp = format!("{dir}\\{stem}.pcm");
    let data = std::fs::read(&path).unwrap_or_else(|_| panic!("read {path}"));
    let refpcm = std::fs::read(&refp).unwrap_or_else(|_| panic!("read {refp}"));
    println!("stream={stem}.ac3  ref={stem}.pcm");

    let mut p = CodecParameters::audio(CodecId::new("ac3"));
    p.channels = Some(2);
    p.sample_rate = Some(48000);
    let mut dec = oxideav_ac3::decoder::make_decoder(&p).unwrap();
    let tb = TimeBase::new(1, 48000);

    println!(
        "{:>5} {:>3} | {:>8} {:>8} | {:>8} {:>8} | {:>6} {:>6}",
        "frame", "blk", "maxd_L", "maxd_R", "bad_L", "bad_R", "rail_L", "rail_R"
    );

    let (mut off, mut frame) = (0usize, 0usize);
    while off + 5 <= data.len() && frame < 12 {
        let n = oxideav_ac3::syncinfo::parse(&data[off..])
            .unwrap()
            .frame_length as usize;
        dec.send_packet(&Packet::new(0, tb, data[off..off + n].to_vec()))
            .unwrap();
        if let Ok(Frame::Audio(af)) = dec.receive_frame() {
            let frame_base = frame * INTERLEAVED_PER_FRAME * 4; // f32 bytes in refpcm
            for blk in 0..BLOCKS {
                let mut maxd = [0f32; CH];
                let mut bad = [0u32; CH];
                let mut rail = [0u32; CH];
                for i in 0..INTERLEAVED_PER_BLK {
                    let interleaved_idx = blk * INTERLEAVED_PER_BLK + i;
                    let ch = i & 1; // 0 = L, 1 = R
                    let b = interleaved_idx * 2;
                    let ours_i16 = i16::from_le_bytes([af.data[0][b], af.data[0][b + 1]]);
                    let ours = ours_i16 as f32 / 32768.0;
                    let rb = frame_base + interleaved_idx * 4;
                    let ff = f32::from_le_bytes(refpcm[rb..rb + 4].try_into().unwrap());
                    let d = (ours - ff).abs();
                    if d > 1e-3 {
                        bad[ch] += 1;
                    }
                    if ours_i16.unsigned_abs() >= 32760 {
                        rail[ch] += 1;
                    }
                    maxd[ch] = maxd[ch].max(d);
                }
                let flag = if maxd[0].max(maxd[1]) > 0.1 {
                    " <<<"
                } else {
                    ""
                };
                println!(
                    "{frame:>5} {blk:>3} | {:>8.4} {:>8.4} | {:>4}/256 {:>4}/256 | {:>6} {:>6}{flag}",
                    maxd[0], maxd[1], bad[0], bad[1], rail[0], rail[1]
                );
            }
            frame += 1;
        }
        off += n;
    }
}
