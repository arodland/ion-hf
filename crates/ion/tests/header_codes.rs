// Short codes for headers and tiny ACKs (`ion::fec::conv`), on ideal
// synthetic channels: which code to put in the MFSK and OFDM headers. The
// numbers in `docs/fec.md` ("Header codes") come from the ignored
// tests here:
//
// cargo test -j 2 -p ion --release --test header_codes -- --test-threads 2 --ignored --nocapture

use ion::fec::conv::{ConvCode, search};
use ion_dsp::dsp::NoiseGen;

fn random_bits(n: usize, ng: &mut NoiseGen) -> Vec<u8> {
    (0..n).map(|_| (ng.uniform() < 0.5) as u8).collect()
}

fn ln_i0(x: f64) -> f64 {
    // Abramowitz & Stegun 9.8.1 / 9.8.2.
    if x < 3.75 {
        let t = (x / 3.75).powi(2);
        (1.0 + t
            * (3.515_622_9
                + t * (3.089_942_4 + t * (1.206_749_2 + t * (0.265_973_2 + t * (0.036_076_8 + t * 0.004_581_3))))))
            .ln()
    } else {
        let t = 3.75 / x;
        x - 0.5 * x.ln()
            + (0.398_942_28
                + t * (0.013_285_92
                    + t * (0.002_253_19
                        + t * (-0.001_575_65
                            + t * (0.009_162_81
                                + t * (-0.020_577_06 + t * (0.026_355_37 + t * (-0.016_476_33 + t * 0.003_923_77))))))))
                .ln()
    }
}

fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// One noncoherent 32-FSK symbol per step at Es/N0 `g` (linear): the log-
/// likelihood of every tone value. `fading`: independent Rayleigh per
/// symbol (ideal interleaving), else AWGN with a random phase.
fn fsk_metrics(label: usize, m: usize, g: f64, fading: bool, ng: &mut NoiseGen) -> Vec<f32> {
    let (hr, hi) = if fading {
        (
            ng.gaussian() as f64 * core::f64::consts::FRAC_1_SQRT_2,
            ng.gaussian() as f64 * core::f64::consts::FRAC_1_SQRT_2,
        )
    } else {
        let ph = ng.uniform() as f64 * core::f64::consts::TAU;
        (ph.cos(), ph.sin())
    };
    (0..m)
        .map(|k| {
            let mut re = ng.gaussian() as f64 * core::f64::consts::FRAC_1_SQRT_2;
            let mut im = ng.gaussian() as f64 * core::f64::consts::FRAC_1_SQRT_2;
            if k == label {
                re += g.sqrt() * hr;
                im += g.sqrt() * hi;
            }
            let y = re * re + im * im;
            (if fading {
                y * g / (1.0 + g)
            } else {
                ln_i0(2.0 * (g * y).sqrt())
            }) as f32
        })
        .collect()
}

/// A candidate MFSK header code: constraint length, generators, tail.
struct ToneCode {
    name: &'static str,
    k: usize,
    gens: Vec<u32>,
    tb: bool,
}

/// Success (all 48 bits right) against Es/N0 per 32-ary symbol for the
/// candidate tone codes of the MFSK header (`MFSK_HDR_TRIALS` headers per
/// point, default 400). Generators: `HDR_GENS_K13` etc. from `search_codes`.
#[test]
#[ignore]
fn mfsk_header_codes() {
    let trials = env_usize("MFSK_HDR_TRIALS", 400);
    let codes = [
        ToneCode {
            name: "K9 zero-tail",
            k: 9,
            gens: vec![0o522, 0o363, 0o570, 0o505, 0o765],
            tb: false,
        },
        ToneCode {
            name: "K9 tail-biting",
            k: 9,
            gens: vec![0o522, 0o363, 0o570, 0o505, 0o765],
            tb: true,
        },
        ToneCode {
            name: "K13 zero-tail",
            k: 13,
            gens: TONE13.to_vec(),
            tb: false,
        },
        ToneCode {
            name: "K13 tail-biting",
            k: 13,
            gens: TONE13.to_vec(),
            tb: true,
        },
        ToneCode {
            name: "K15 zero-tail",
            k: 15,
            gens: TONE15.to_vec(),
            tb: false,
        },
        ToneCode {
            name: "K15 tail-biting (used)",
            k: 15,
            gens: TONE15.to_vec(),
            tb: true,
        },
    ];
    let nbits = 48;
    for fading in [false, true] {
        println!("\n{}", if fading { "Rayleigh (per symbol)" } else { "AWGN" });
        for c in &codes {
            let code = ConvCode::new(c.k, &c.gens);
            let steps = if c.tb { nbits } else { nbits + c.k - 1 };
            let mut line = format!("{:<20} {steps:>2} sym:", c.name);
            let mut t_dec = 0f64;
            let grid: Vec<f64> = if fading {
                (40..=90).step_by(5).map(|x| x as f64 / 10.0).collect()
            } else {
                (20..=60).step_by(5).map(|x| x as f64 / 10.0).collect()
            };
            for es in grid {
                let g = 10f64.powf(es / 10.0);
                let mut ng = NoiseGen::new(1234 + (es * 10.0) as u64);
                let mut ok = 0;
                for _ in 0..trials {
                    let b = random_bits(nbits, &mut ng);
                    let labels = if c.tb {
                        code.encode_tb(&b)
                    } else {
                        code.encode_zt(&b, steps)
                    };
                    let met: Vec<Vec<f32>> = labels
                        .iter()
                        .map(|&l| fsk_metrics(l as usize, 32, g, fading, &mut ng))
                        .collect();
                    let mut mf = |j: usize, out: &mut [f32]| out.copy_from_slice(&met[j]);
                    let t = std::time::Instant::now();
                    let got = if c.tb {
                        code.decode_tb(steps, 3, &mut mf).0
                    } else {
                        code.decode_zt(steps, nbits, &mut mf).0
                    };
                    t_dec += t.elapsed().as_secs_f64();
                    ok += usize::from(got[..nbits] == b[..]);
                }
                line += &format!(" {es:.1}:{:.3}", ok as f64 / trials as f64);
            }
            println!(
                "{line}  ({:.2} ms/decode)",
                1e3 * t_dec / (trials as f64 * if fading { 11.0 } else { 9.0 })
            );
        }
    }
}

/// The OFDM header's candidate codes: (name, constraint length, generators,
/// tail-biting). The 3GPP generators are in MSB = newest order; reversing
/// every generator gives the time-reversed code (the same distances), and
/// `ConvCode` takes bit 0 = newest, so they are used as they are.
fn ofdm_codes() -> Vec<(&'static str, usize, Vec<u32>, bool)> {
    vec![
        ("K7 r1/3 zero-tail", 7, vec![0o133, 0o171, 0o165], false),
        ("K11 r1/8 tail-biting", 11, BIN11.to_vec(), true),
        ("K13 r1/8 tail-biting (used)", 13, BIN13.to_vec(), true),
        ("K15 r1/8 tail-biting", 15, BIN15.to_vec(), true),
    ]
}

/// Header success against Es/N0 per coded bit (BPSK; QPSK is two of
/// them) for the OFDM header: 48 bits into `HDR_SLOTS` slots (default 352,
/// 704 = a two-copy header-only burst). The code's bits go out in
/// stream-major order (all steps of output 0, then output 1, ...), repeated
/// from the start when the slots outnumber them, truncated when not; the
/// receiver adds the LLRs of repeated bits.
#[test]
#[ignore]
fn ofdm_header_codes() {
    let trials = env_usize("HDR_TRIALS", 400);
    let slots = env_usize("HDR_SLOTS", 352);
    let nbits = 48;
    for fading in [false, true] {
        println!("\n{} {slots} slots", if fading { "Rayleigh (per bit)" } else { "AWGN" });
        for (name, k, gens, tb) in ofdm_codes() {
            let code = ConvCode::new(k, &gens);
            let n = code.nout();
            let steps = if tb { nbits } else { nbits + k - 1 };
            let len = steps * n;
            let mut line = format!("{name:<24}:");
            let grid: Vec<i32> = if fading {
                (-95..=-45).step_by(5).collect()
            } else {
                (-115..=-65).step_by(5).collect()
            };
            let mut t_dec = 0f64;
            for &es10 in &grid {
                let es = es10 as f64 / 10.0 - if slots > 400 { 3.0 } else { 0.0 };
                let g = 10f64.powf(es / 10.0);
                let mut ng = NoiseGen::new(77 + (es10 + 1000) as u64);
                let mut ok = 0;
                for _ in 0..trials {
                    let b = random_bits(nbits, &mut ng);
                    let labels = if tb {
                        code.encode_tb(&b)
                    } else {
                        code.encode_zt(&b, steps)
                    };
                    // Stream-major coded bits.
                    let coded: Vec<u8> = (0..n)
                        .flat_map(|i| labels.iter().map(move |&l| ((l >> i) & 1) as u8))
                        .collect();
                    let mut acc = vec![0f32; len];
                    for j in 0..slots {
                        let x = 1.0 - 2.0 * coded[j % len] as f64;
                        let h = if fading {
                            let (a, c) = (ng.gaussian() as f64, ng.gaussian() as f64);
                            ((a * a + c * c) / 2.0).sqrt()
                        } else {
                            1.0
                        };
                        let y = h * x * (2.0 * g).sqrt() + ng.gaussian() as f64;
                        acc[j % len] += (2.0 * (2.0 * g).sqrt() * h * y) as f32;
                    }
                    let mut mf = |t: usize, out: &mut [f32]| {
                        for (l, o) in out.iter_mut().enumerate() {
                            let mut s = 0.0;
                            for i in 0..n {
                                let v = acc[i * steps + t];
                                s += if (l >> i) & 1 == 0 { 0.5 * v } else { -0.5 * v };
                            }
                            *o = s;
                        }
                    };
                    let tt = std::time::Instant::now();
                    let got = if tb {
                        code.decode_tb(steps, 3, &mut mf).0
                    } else {
                        code.decode_zt(steps, nbits, &mut mf).0
                    };
                    t_dec += tt.elapsed().as_secs_f64();
                    ok += usize::from(got[..nbits] == b[..]);
                }
                line += &format!(" {:.1}:{:.3}", es, ok as f64 / trials as f64);
            }
            println!("{line}  ({:.2} ms/decode)", 1e3 * t_dec / (trials * grid.len()) as f64);
        }
    }
}

/// Generator search (prints the best sets): tone codes (symbol distance)
/// for the MFSK header and binary rate-1/8 codes (Hamming distance) for the
/// OFDM header. `SEARCH_TRIALS` candidates each (default 300).
#[test]
#[ignore]
fn search_codes() {
    let trials = env_usize("SEARCH_TRIALS", 300);
    let sym = |l: u16| usize::from(l != 0);
    let ham = |l: u16| l.count_ones() as usize;
    for k in [13usize, 15] {
        let (g, d, m) = search(k, 5, &sym, trials, 0x5EED + k as u64);
        println!(
            "tone K{k}: {:?} (octal {}) d_s {d} x{m}",
            g,
            g.iter().map(|x| format!("0o{x:o}")).collect::<Vec<_>>().join(", ")
        );
    }
    for k in [11usize, 13, 15] {
        let (g, d, m) = search(k, 8, &ham, trials, 0xB1A + k as u64);
        println!(
            "binary K{k} r1/8: (octal {}) d_free {d} x{m}",
            g.iter().map(|x| format!("0o{x:o}")).collect::<Vec<_>>().join(", ")
        );
    }
}

// From `search_codes` (300 candidates each).
const TONE13: [u32; 5] = [0o17747, 0o14205, 0o13277, 0o11333, 0o11663];
const TONE15: [u32; 5] = [0o42101, 0o77127, 0o60225, 0o42073, 0o64667];
const BIN11: [u32; 8] = [0o2531, 0o3741, 0o3673, 0o3111, 0o2537, 0o2767, 0o3465, 0o3071];
const BIN13: [u32; 8] = [0o14623, 0o10771, 0o17567, 0o10433, 0o13351, 0o17265, 0o15237, 0o17221];
const BIN15: [u32; 8] = [0o66727, 0o70421, 0o76357, 0o53277, 0o66725, 0o57211, 0o52631, 0o71475];
