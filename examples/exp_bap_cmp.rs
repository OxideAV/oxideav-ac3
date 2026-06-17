fn st(data: &[u8], fi: usize) -> oxideav_ac3::audblk::Ac3State {
    let mut off = 0;
    let mut n = 0;
    loop {
        let si = oxideav_ac3::syncinfo::parse(&data[off..]).unwrap();
        let len = si.frame_length as usize;
        if n == fi {
            let f = &data[off..off + len];
            let b = oxideav_ac3::bsi::parse(&f[5..]).unwrap();
            return oxideav_ac3::audblk::state_after_block(&si, &b, f, 0).unwrap();
        }
        off += len;
        n += 1;
    }
}
fn main() {
    let data = std::fs::read(std::env::var("TEMP").unwrap() + r"\oxideav_repro\multi.ac3").unwrap();
    let s2 = st(&data, 2);
    let s4 = st(&data, 4);
    let mut ed = 0;
    for b in 0..217 {
        if s2.channels[0].exp[b] != s4.channels[0].exp[b] {
            ed += 1;
        }
    }
    println!("ch0 exp diffs: {ed}");
    for b in 0..217 {
        if s2.channels[0].bap[b] != s4.channels[0].bap[b] {
            println!(
                "bin {b}: exp {} vs {} bap {} vs {}",
                s2.channels[0].exp[b],
                s4.channels[0].exp[b],
                s2.channels[0].bap[b],
                s4.channels[0].bap[b]
            );
        }
    }
}
