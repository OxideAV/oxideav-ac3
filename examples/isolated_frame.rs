use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};
fn decode_frames(data:&[u8], start:usize, count:usize) -> Vec<f32> {
    let mut p = CodecParameters::audio(CodecId::new("ac3")); p.channels=Some(2); p.sample_rate=Some(48000);
    let mut dec = oxideav_ac3::decoder::make_decoder(&p).unwrap();
    let tb = TimeBase::new(1,48000);
    let mut off=0; let mut fi=0; let mut out=Vec::new();
    while off+5<=data.len() {
        let n=oxideav_ac3::syncinfo::parse(&data[off..]).unwrap().frame_length as usize;
        if fi>=start { dec.send_packet(&Packet::new(0,tb,data[off..off+n].to_vec())).unwrap();
            if let Ok(Frame::Audio(af))=dec.receive_frame() {
                for c in af.data[0].chunks_exact(2) { out.push(i16::from_le_bytes([c[0],c[1]]) as f32/32768.0); }
            }
        }
        off+=n; fi+=1; if fi>=start+count { break; }
    }
    out
}
fn max_diff(a:&[f32], b:&[f32]) -> (f32,usize) {
    let mut md=0f32; let mut nd=0usize;
    for (x,y) in a.iter().zip(b.iter()) { let d=(x-y).abs(); if d>1e-3 {nd+=1;} md=md.max(d); }
    (md,nd)
}
fn main() {
    let data = std::fs::read(std::env::var("TEMP").unwrap()+r"\oxideav_repro\multi.ac3").unwrap();
    let refpcm=std::fs::read(std::env::var("TEMP").unwrap()+r"\oxideav_repro\ffmpeg.pcm").unwrap();
    let ref3: Vec<f32> = refpcm[3*3072*4..4*3072*4].chunks_exact(4).map(|c| f32::from_le_bytes(c.try_into().unwrap())).collect();
    let stream = decode_frames(&data, 0, 4);
    let s3 = &stream[3*3072..4*3072];
    let (md,nd)=max_diff(s3, &ref3);
    println!("frame3 stream decode: max_diff={md:.4} bad={nd}/3072");
    let isolated = decode_frames(&data, 3, 1);
    let (md2,nd2)=max_diff(&isolated, &ref3);
    println!("frame3 isolated decode: max_diff={md2:.4} bad={nd2}/3072");
}
