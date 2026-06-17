use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};
fn main() {
    let path = std::env::var("TEMP").unwrap() + r"\oxideav_repro\multi.ac3";
    let data = std::fs::read(&path).unwrap();
    let refpcm=std::fs::read(std::env::var("TEMP").unwrap()+r"\oxideav_repro\ffmpeg.pcm").unwrap();
    let mut p = CodecParameters::audio(CodecId::new("ac3")); p.channels=Some(2); p.sample_rate=Some(48000);
    let mut dec = oxideav_ac3::decoder::make_decoder(&p).unwrap();
    let tb = TimeBase::new(1,48000);
    let mut off=0; let mut frame=0;
    while off+5<=data.len() && frame<12 {
        let n=oxideav_ac3::syncinfo::parse(&data[off..]).unwrap().frame_length as usize;
        dec.send_packet(&Packet::new(0,tb,data[off..off+n].to_vec())).unwrap();
        if let Ok(Frame::Audio(af))=dec.receive_frame() {
            let base=frame*3072*4;
            let mut max_diff=0f32; let mut n_diff=0u32;
            for (i,c) in af.data[0].chunks_exact(2).enumerate() {
                let ours=i16::from_le_bytes([c[0],c[1]]) as f32/32768.0;
                let ff=f32::from_le_bytes(refpcm[base+i*4..base+i*4+4].try_into().unwrap());
                let d=(ours-ff).abs();
                if d>1e-3 { n_diff+=1; }
                max_diff=max_diff.max(d);
            }
            println!("frame {frame}: max_diff={max_diff:.4} bad_samples(>1e-3)={n_diff}/3072");
            frame+=1;
        }
        off+=n;
    }
}
