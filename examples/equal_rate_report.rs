//! Equal-rate rate-distortion ladder: our encoder vs the black-box
//! reference encoder, every stream decoded by both decoders.
//!
//! ```sh
//! cargo run --release --example equal_rate_report            # full ladder
//! cargo run --release --example equal_rate_report -- music 192 ac3 aht=1
//! OXIDEAV_RD_BANDS=1 cargo run --release --example equal_rate_report -- tones 192
//! ```
//!
//! Positional args: `<signal|all> <kbps|all> <ac3|eac3|all> [key=val ...]`
//! (the key=val pairs are registry options for our encoder).

#[path = "../tests/common/rd.rs"]
#[allow(dead_code)]
mod rd;

use rd::{Codec, Dec, Enc};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let sig_sel = args.first().map(String::as_str).unwrap_or("all");
    let rate_sel = args.get(1).map(String::as_str).unwrap_or("all");
    let codec_sel = args.get(2).map(String::as_str).unwrap_or("all");
    let opts: Vec<(String, String)> = args
        .iter()
        .skip(3)
        .filter_map(|a| {
            a.split_once('=')
                .map(|(k, v)| (k.to_string(), v.to_string()))
        })
        .collect();
    let bands = std::env::var("OXIDEAV_RD_BANDS").is_ok();
    let dur: f32 = std::env::var("OXIDEAV_RD_DUR")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(3.0);

    let corpus = rd::corpus(dur);
    let codecs: Vec<Codec> = match codec_sel {
        "ac3" => vec![Codec::Ac3],
        "eac3" => vec![Codec::Eac3],
        _ => vec![Codec::Ac3, Codec::Eac3],
    };
    println!(
        "{:<11} {:<5} {:>4} | {:<22} | {:>8} {:>8} {:>8} | {:>8} {:>8} {:>8}",
        "signal",
        "codec",
        "kbps",
        "encoder",
        "snr_min",
        "snr_mean",
        "nmr",
        "snr_min",
        "snr_mean",
        "nmr"
    );
    println!(
        "{:<11} {:<5} {:>4} | {:<22} | {:^26} | {:^26}",
        "", "", "", "", "decoded by ours", "decoded by reference"
    );
    for sig in &corpus {
        if sig_sel != "all" && sig.name != sig_sel {
            continue;
        }
        let rates: Vec<u32> = if rate_sel != "all" {
            vec![rate_sel.parse().expect("kbps")]
        } else {
            match sig.channels {
                1 => vec![64, 96, 192],
                2 => vec![96, 192, 384],
                _ => vec![256, 448, 640],
            }
        };
        for &codec in &codecs {
            for &kbps in &rates {
                let mut encs: Vec<(&str, Enc)> = vec![
                    ("reference", Enc::Reference(codec)),
                    ("ours", Enc::Ours(codec, opts.clone())),
                ];
                if std::env::var("OXIDEAV_RD_OURS_ONLY").is_ok() {
                    encs.remove(0);
                }
                for (label, enc) in encs {
                    let a = rd::measure(sig, &enc, Dec::Ours, kbps);
                    let b = if std::env::var("OXIDEAV_RD_OURS_ONLY").is_ok() {
                        None
                    } else {
                        rd::measure(sig, &enc, Dec::Reference, kbps)
                    };
                    let cell = |m: &Option<(rd::Score, usize)>| match m {
                        Some((s, _)) => format!(
                            "{:8.2} {:8.2} {:8.2}",
                            s.snr_min(),
                            s.snr_mean(),
                            s.nmr_mean
                        ),
                        None => format!("{:>8} {:>8} {:>8}", "-", "-", "-"),
                    };
                    println!(
                        "{:<11} {:<5} {:>4} | {:<22} | {} | {}",
                        sig.name,
                        codec.id(),
                        kbps,
                        label,
                        cell(&a),
                        cell(&b)
                    );
                    if bands {
                        if let Some((s, _)) = &b {
                            let mut line = String::from("    nmr/band:");
                            for (k, v) in s.nmr_band.iter().enumerate() {
                                if !v.is_nan() {
                                    line.push_str(&format!(" {k}:{v:.0}"));
                                }
                            }
                            println!("{line}");
                        }
                        if let Some((s, _)) = &a {
                            println!(
                                "    snr/ch (ours dec): {:?}",
                                s.snr_ch
                                    .iter()
                                    .map(|v| (v * 10.0).round() / 10.0)
                                    .collect::<Vec<_>>()
                            );
                        }
                        if let Some((s, _)) = &b {
                            println!(
                                "    snr/ch (ref dec):  {:?}",
                                s.snr_ch
                                    .iter()
                                    .map(|v| (v * 10.0).round() / 10.0)
                                    .collect::<Vec<_>>()
                            );
                        }
                        if let Some(es) = rd::encode(sig, &enc, kbps) {
                            let pcm = rd::decode(&es, &enc, Dec::Reference, sig.channels).unwrap();
                            let lag = rd::path_lag(&enc, Dec::Reference).unwrap();
                            let (bs, be) = rd::band_profile(&sig.pcm, &pcm, sig.channels, 0, lag);
                            let mut line = String::from("    ch0 sig/err dB:");
                            for k in 0..50 {
                                if bs[k].is_finite() {
                                    line.push_str(&format!(" {k}:{:.0}/{:.0}", bs[k], be[k]));
                                }
                            }
                            println!("{line}");
                        }
                        println!(
                            "    lag ours={:?} ref={:?}",
                            rd::path_lag(&enc, Dec::Ours),
                            rd::path_lag(&enc, Dec::Reference)
                        );
                        if std::env::var("OXIDEAV_RD_LAGSCAN").is_ok() {
                            if let Some(es) = rd::encode(sig, &enc, kbps) {
                                let pcm =
                                    rd::decode(&es, &enc, Dec::Reference, sig.channels).unwrap();
                                let mut best = (0usize, f64::NEG_INFINITY);
                                for lag in 0..=1024usize {
                                    let v = rd::snr_min_at(&sig.pcm, &pcm, sig.channels, lag);
                                    if v > best.1 {
                                        best = (lag, v);
                                    }
                                }
                                println!("    lagscan best={:?}", best);
                            }
                        }
                    }
                }
            }
        }
    }
}
