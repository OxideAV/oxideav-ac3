fn main() {
    let data = std::fs::read(std::env::var("TEMP").unwrap() + r"\oxideav_repro\multi.ac3").unwrap();
    let si = oxideav_ac3::syncinfo::parse(&data).unwrap();
    let bsi = oxideav_ac3::bsi::parse(&data[5..]).unwrap();
    println!(
        "fscod={} bsid={} acmod={} sr_shift={}",
        si.fscod,
        bsi.bsid,
        bsi.acmod,
        bsi.bsid.max(8) - 8
    );
}
