fn load(data:&[u8], fi:usize)->(oxideav_ac3::syncinfo::SyncInfo, oxideav_ac3::bsi::Bsi, Vec<u8>) {
    let mut off=0; let mut n=0;
    loop { let si=oxideav_ac3::syncinfo::parse(&data[off..]).unwrap(); let len=si.frame_length as usize;
        if n==fi { return (si, oxideav_ac3::bsi::parse(&data[off+5..]).unwrap(), data[off..off+len].to_vec()); }
        off+=len; n+=1; }
}
fn main() {
    let data = std::fs::read(std::env::var("TEMP").unwrap()+r"\oxideav_repro\multi.ac3").unwrap();
    for fi in [2usize,4] {
        let (si,bsi,frame)=load(&data,fi);
        for ch1 in [12u8,13,14] {
            let mut o=[0u8;5]; o[0]=13; o[1]=ch1;
            let iso = oxideav_ac3::audblk::probe_blk1_after_fsnroffst(&si,&bsi,&frame,0,Some(o)).unwrap();
            let strm = oxideav_ac3::audblk::probe_blk1_after_fsnroffst_stream(&data,fi,0,Some(o)).unwrap();
            println!("frame {fi} ch1={ch1} isolated ok={} mant={} | stream ok={} mant={}", iso.blk1_parse_ok, iso.mantissa_bits, strm.blk1_parse_ok, strm.mantissa_bits);
        }
    }
}
