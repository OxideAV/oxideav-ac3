fn load(data: &[u8], fi: usize) -> oxideav_ac3::audblk::Ac3State {
    let mut off = 0;
    let mut n = 0;
    loop {
        let si = oxideav_ac3::syncinfo::parse(&data[off..]).unwrap();
        let len = si.frame_length as usize;
        if n == fi {
            let frame = &data[off..off + len];
            let bsi = oxideav_ac3::bsi::parse(&frame[5..]).unwrap();
            return oxideav_ac3::audblk::state_after_block(&si, &bsi, frame, 0).unwrap();
        }
        off += len;
        n += 1;
    }
}
fn main() {
    let data = std::fs::read(std::env::var("TEMP").unwrap() + r"\oxideav_repro\multi.ac3").unwrap();
    for fi in [2usize, 5] {
        let st = load(&data, fi);
        println!(
            "frame {fi}: fsnr={:?} fgain={:?} mask[40..46]={:?}",
            st.fsnroffst[..2].to_vec(),
            st.fgaincod[..2].to_vec(),
            &st.channels[1].mask[40..46]
        );
        for bin in [22usize, 46, 79] {
            let d = oxideav_ac3::audblk::bin_ba_detail(&st, 1, bin);
            println!(
                "  bin {bin}: exp={} psd={} mask={} m={:#x} addr={} bap={}",
                d.exp, d.psd, d.mask, d.m_snr, d.bap_addr, d.bap
            );
        }
    }
}
