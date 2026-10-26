//! The Ion over-the-air ladder test.
//!
//!   cargo run --release -p ion --example sounding -- make test.wav N0CALL
//!       writes the test sequence
//!   cargo run --release -p ion --example sounding -- decode rec.wav [N0CALL]
//!       decodes a recording of it (a KiwiSDR/WebSDR WAV, any rate); with
//!       the call, lists every burst sent and whether it was heard

use std::io::Write;

use ion::sounding;

fn write_wav(path: &str, a: &[f32]) {
    let mut w = std::io::BufWriter::new(std::fs::File::create(path).expect("create wav"));
    let data_len = (a.len() * 2) as u32;
    w.write_all(b"RIFF").unwrap();
    w.write_all(&(36 + data_len).to_le_bytes()).unwrap();
    w.write_all(b"WAVEfmt ").unwrap();
    w.write_all(&16u32.to_le_bytes()).unwrap();
    w.write_all(&1u16.to_le_bytes()).unwrap();
    w.write_all(&1u16.to_le_bytes()).unwrap();
    w.write_all(&48_000u32.to_le_bytes()).unwrap();
    w.write_all(&96_000u32.to_le_bytes()).unwrap();
    w.write_all(&2u16.to_le_bytes()).unwrap();
    w.write_all(&16u16.to_le_bytes()).unwrap();
    w.write_all(b"data").unwrap();
    w.write_all(&data_len.to_le_bytes()).unwrap();
    for &s in a {
        w.write_all(&((s.clamp(-1.0, 1.0) * 32767.0) as i16).to_le_bytes())
            .unwrap();
    }
    w.flush().unwrap();
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("make") => {
            let path = args.get(1).map_or("ion_test.wav", String::as_str);
            let call = args.get(2).map_or("N0CALL", String::as_str);
            let (a, sent) = sounding::sequence(call);
            write_wav(path, &a);
            println!(
                "wrote {path}: {:.0} s, {} bursts",
                a.len() as f64 / 48_000.0,
                sent.len()
            );
        }
        Some("decode") => {
            let path = args.get(1).expect("decode <file.wav> [CALL]");
            let bytes = std::fs::read(path).expect("read wav");
            let audio = sounding::read_wav(&bytes).expect("wav");
            println!("{path}: {:.0} s", audio.len() as f64 / 48_000.0);
            let heard = sounding::analyze(&audio);
            println!("\n{}", sounding::report_heard(&heard));
            if let Some(call) = args.get(2) {
                let (_, sent) = sounding::sequence(call);
                println!("{}", sounding::report(&sent, &heard));
                let got = sent
                    .iter()
                    .filter(|b| heard.iter().any(|h| h.seq == Some(b.seq)))
                    .count();
                println!("{got} of {} bursts heard", sent.len());
            }
        }
        _ => eprintln!("usage: sounding make <out.wav> <CALL> | decode <rec.wav> [CALL]"),
    }
}
