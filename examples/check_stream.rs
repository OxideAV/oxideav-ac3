//! Standalone AC-3 exp-stream sanity checker. Reads an elementary AC-3
//! stream and verifies that every D15 exp group reconstructs into the
//! [0..=24] range — matching the consistency check ffmpeg performs
//! (`prevexp > 24U` rejects both >24 and < 0 values).

use std::env;
use std::fs;

use oxideav_ac3::{bsi, syncinfo};
use oxideav_core::bits::BitReader;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = env::args().nth(1).unwrap_or_else(|| "/tmp/sine440.ac3".into());
    let data = fs::read(&path)?;

    let mut offset = 0;
    let mut frame_idx = 0;
    while offset < data.len() {
        let si = syncinfo::parse(&data[offset..])?;
        let frame_bytes = si.frame_length as usize;
        let frame = &data[offset..offset + frame_bytes];

        let b = bsi::parse(&frame[5..])?;
        let post_sync = &frame[5..];
        let mut br = BitReader::new(post_sync);
        br.skip(b.bits_consumed as u32)?;

        // We are now positioned at the beginning of audblk 0.
        // Parse enough to reach chbwcod + exponents.
        let nfchans = b.nfchans as usize;
        let acmod = b.acmod;

        // blksw × nfchans + dithflag × nfchans
        br.skip(nfchans as u32)?;
        br.skip(nfchans as u32)?;
        // dynrnge
        let dynrnge = br.read_u32(1)? != 0;
        if dynrnge { br.skip(8)?; }
        if acmod == 0 { // 1+1 dual-mono, second dynrng
            let d2 = br.read_u32(1)? != 0;
            if d2 { br.skip(8)?; }
        }

        // cplstre / cplinu — block 0 always sends cplstre.
        let cplstre = br.read_u32(1)? != 0;
        let cplinu = if cplstre { br.read_u32(1)? != 0 } else { false };
        if cplinu {
            eprintln!("frame {frame_idx}: coupling not supported in this checker; skipping");
            offset += frame_bytes;
            frame_idx += 1;
            continue;
        }

        // Rematrix (2/0 only).
        if acmod == 0x2 {
            let rematstr = br.read_u32(1)? != 0;
            if rematstr {
                // cplinu=0 → 4 rematflg bits
                br.skip(4)?;
            }
        }

        // Exp strategy.
        let mut chexpstr = [0u8; 5];
        for ch in 0..nfchans {
            chexpstr[ch] = br.read_u32(2)? as u8;
        }
        // lfeexpstr.
        if b.lfeon {
            br.skip(1)?;
        }
        // chbwcod for non-coupled, not-reuse channels.
        let mut chbwcod = [0u8; 5];
        for ch in 0..nfchans {
            if chexpstr[ch] != 0 {
                chbwcod[ch] = br.read_u32(6)? as u8;
            }
        }

        // Exp unpack.
        for ch in 0..nfchans {
            if chexpstr[ch] == 0 {
                continue;
            }
            // fbw not-coupled: end = 37 + 3*(chbwcod+12)
            let end = 37 + 3 * (chbwcod[ch] as usize + 12);
            let group_size = match chexpstr[ch] {
                1 => 3usize,
                2 => 6,
                3 => 12,
                _ => panic!(),
            };
            let ngrps = (end + group_size - 4) / group_size;
            let absexp = br.read_u32(4)? as i32;
            let mut prev = absexp;
            let bit_before = br.bit_position();
            for grp in 0..ngrps {
                let packed = br.read_u32(7)? as i32;
                if packed >= 125 {
                    eprintln!("frame {frame_idx} ch{ch} grp {grp}: expacc {packed} >= 125");
                    break;
                }
                let m1 = packed / 25;
                let m2 = (packed % 25) / 5;
                let m3 = packed % 5;
                let d0 = m1 - 2;
                let d1 = m2 - 2;
                let d2 = m3 - 2;
                prev += d0;
                if !(0..=24).contains(&prev) {
                    eprintln!(
                        "frame {frame_idx} ch{ch} grp {grp} (bit {bit_before}): exp OOR after d0={d0}, prev={prev}, packed={packed}"
                    );
                    break;
                }
                prev += d1;
                if !(0..=24).contains(&prev) {
                    eprintln!(
                        "frame {frame_idx} ch{ch} grp {grp}: exp OOR after d1={d1}, prev={prev}, packed={packed}"
                    );
                    break;
                }
                prev += d2;
                if !(0..=24).contains(&prev) {
                    eprintln!(
                        "frame {frame_idx} ch{ch} grp {grp}: exp OOR after d2={d2}, prev={prev}, packed={packed}"
                    );
                    break;
                }
            }
            // gainrng
            br.skip(2)?;
        }

        offset += frame_bytes;
        frame_idx += 1;
    }
    eprintln!("checked {frame_idx} frames");
    Ok(())
}
