// GF(32) LDPC for the MFSK floor (`ion::fec::nb`) on ideal synthetic
// noncoherent 32-FSK: degree profiles against the BICM-ID decoder of the
// binary short code. Numbers in `docs/fec.md` ("GF(32) short code"):
//
// cargo test -j 2 -p ion --release --test nb -- --test-threads 2 --ignored --nocapture

use ion::fec::nb::{self, NbCode, Q};
use ion_dsp::dsp::NoiseGen;

fn ln_i0(x: f64) -> f64 {
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

/// Tone log-likelihoods of one noncoherent 32-FSK symbol at Es/N0 `g`.
fn metrics(sym: u8, g: f64, fading: bool, ng: &mut NoiseGen, out: &mut [f32]) {
    let (hr, hi) = if fading {
        (
            ng.gaussian() as f64 * core::f64::consts::FRAC_1_SQRT_2,
            ng.gaussian() as f64 * core::f64::consts::FRAC_1_SQRT_2,
        )
    } else {
        let ph = ng.uniform() as f64 * core::f64::consts::TAU;
        (ph.cos(), ph.sin())
    };
    for (k, o) in out.iter_mut().enumerate() {
        let mut re = ng.gaussian() as f64 * core::f64::consts::FRAC_1_SQRT_2;
        let mut im = ng.gaussian() as f64 * core::f64::consts::FRAC_1_SQRT_2;
        if k == sym as usize {
            re += g.sqrt() * hr;
            im += g.sqrt() * hi;
        }
        let y = re * re + im * im;
        *o = (if fading {
            y * g / (1.0 + g)
        } else {
            ln_i0(2.0 * (g * y).sqrt())
        }) as f32;
    }
}

#[test]
fn clean_and_noisy_round_trip() {
    let mut ng = NoiseGen::new(3);
    let info: Vec<u8> = (0..nb::K_BITS).map(|i| ((i * 13 + 1) % 3 % 2) as u8).collect();
    let x = nb::encode_bits(&info);
    let mut m = vec![0f32; nb::N * Q];
    for (v, &s) in x.iter().enumerate() {
        metrics(s, 10f64.powf(0.6), false, &mut ng, &mut m[v * Q..(v + 1) * Q]);
    }
    assert_eq!(nb::decode_bits(&m).as_deref(), Some(&info[..]));
}

/// Every GF(32) rate encodes to its length and decodes through the link's
/// layout.
#[test]
fn every_rate_round_trips() {
    let mut ng = NoiseGen::new(5);
    let info: Vec<u8> = (0..nb::K_BITS).map(|i| ((i * 13 + 1) % 3 % 2) as u8).collect();
    for rate in nb::RATES {
        let x = nb::encode_bits_at(rate, nb::link_layout(), &info);
        assert_eq!(x.len(), nb::n_for(rate), "{rate:?}");
        let mut m = vec![0f32; x.len() * Q];
        for (v, &s) in x.iter().enumerate() {
            metrics(s, 10f64.powf(1.0), false, &mut ng, &mut m[v * Q..(v + 1) * Q]);
        }
        let got = nb::decode_bits_prior_at(rate, nb::link_layout(), &m, &[], nb::link_fallback());
        assert_eq!(got.as_deref(), Some(&info[..]), "{rate:?}");
    }
}

/// Codeword success against Es/N0 for degree profiles of a GF(32) code
/// (N = 205, 153 checks; `NB_N` for another length with the same 52
/// information symbols, e.g. 104 for rate 1/2), `NB_TRIALS` codewords per
/// point (default 100), `NB_GRID` = `lo,hi` Es/N0 × 10 (AWGN; fading 1 dB
/// higher, both in 0.5 dB steps).
#[test]
#[ignore]
fn nb_profiles() {
    let trials = env_usize("NB_TRIALS", 100);
    let n = env_usize("NB_N", nb::N);
    let m = n - nb::K;
    let (lo, hi) = std::env::var("NB_GRID")
        .ok()
        .and_then(|v| v.split_once(',').map(|(a, b)| (a.parse().unwrap(), b.parse().unwrap())))
        .unwrap_or((35i32, 45i32));
    let mut profiles: Vec<(String, Vec<usize>)> = vec![
        ("dv 2/3, dc 3".into(), nb::even_degrees(n, 3 * m)),
        ("dv 2, dc 2/3".into(), nb::even_degrees(n, 2 * n)),
        ("dv 3, dc 4".into(), nb::even_degrees(n, 3 * n)),
        ("dv 2/3, dc 3.5".into(), nb::even_degrees(n, 7 * m / 2)),
    ];
    // Mostly degree 2 with a few heavy columns (low-rate style).
    let mut heavy = vec![2usize; n];
    heavy[..20].fill(6);
    profiles.push(("dv 2 + 20×6".into(), heavy));
    // `NB_SPECS`: extra profiles, e.g. `20x6,30x5`: degree 2 everywhere but
    // the first A variables, which get degree B (groups separated by `+`).
    if let Ok(v) = std::env::var("NB_SPECS") {
        profiles.clear();
        for spec in v.split(',') {
            let mut d = vec![2usize; n];
            let mut at = 0;
            for g in spec.split('+') {
                let (a, b) = g.split_once('x').unwrap();
                let (a, b): (usize, usize) = (a.parse().unwrap(), b.parse().unwrap());
                for x in d.iter_mut().skip(at).take(a) {
                    *x = b;
                }
                at += a;
            }
            profiles.push((format!("dv 2 + {spec}"), d));
        }
    }
    let sel = std::env::var("NB_PROFILES").ok();
    for (pi, (name, deg)) in profiles.iter().enumerate() {
        if sel.as_ref().is_some_and(|s| !s.split(',').any(|x| x.parse() == Ok(pi))) {
            continue;
        }
        let code = NbCode::new(n, m, deg, 0x4E42_4C44_5043);
        let k = code.k();
        for fading in [false, true] {
            let grid: Vec<i32> = if fading {
                (lo + 10..=hi + 15).step_by(5).collect()
            } else {
                (lo..=hi).step_by(5).collect()
            };
            let mut line = format!("{name:<16} k {k} {}:", if fading { "Rayleigh" } else { "AWGN    " });
            let t0 = std::time::Instant::now();
            let mut decodes = 0;
            for &es10 in &grid {
                let g = 10f64.powf(es10 as f64 / 100.0);
                let mut ng = NoiseGen::new(77 + es10 as u64);
                let mut ok = 0;
                let mut mbuf = vec![0f32; n * Q];
                for t in 0..trials {
                    let u: Vec<u8> = (0..nb::K)
                        .map(|i| ((t * 31 + i * 7) as u64 ^ ng.uniform().to_bits() as u64) as u8 & 31)
                        .collect();
                    let x = code.encode(&u);
                    for (v, &s) in x.iter().enumerate() {
                        metrics(s, g, fading, &mut ng, &mut mbuf[v * Q..(v + 1) * Q]);
                    }
                    decodes += 1;
                    if let Some(y) = code.decode(&mbuf, nb::K, nb::MAX_ITER) {
                        ok += usize::from(y == x);
                    }
                }
                line += &format!(" {:.1}:{:.2}", es10 as f64 / 10.0, ok as f64 / trials as f64);
            }
            println!(
                "{line}  ({:.1} ms/decode)",
                1e3 * t0.elapsed().as_secs_f64() / decodes as f64
            );
        }
    }
}

/// Undetected errors of the link's code: decodes that satisfy every check
/// but are not the sent codeword, at low Es/N0 on AWGN where most decodes
/// fail, and the error floor above (`NB_ES`: Es/N0 × 10, comma-separated;
/// `NB_TRIALS` per point, default 1000). The link frames carry a CRC-32 on
/// top.
#[test]
#[ignore]
fn nb_undetected() {
    let trials = env_usize("NB_TRIALS", 1000);
    let code = nb::code();
    println!("link code: n {} k {}", code.n(), code.k());
    let grid: Vec<i32> = std::env::var("NB_ES")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.parse().ok()).collect())
        .unwrap_or_else(|| vec![20, 25, 30, 35]);
    for es10 in grid {
        let g = 10f64.powf(es10 as f64 / 100.0);
        let mut ng = NoiseGen::new(500 + es10 as u64);
        let (mut ok, mut wrong) = (0, 0);
        let mut mbuf = vec![0f32; nb::N * Q];
        for t in 0..trials {
            let info: Vec<u8> = (0..nb::K_BITS)
                .map(|i| ((t * 7 + i * 13) % 5 % 2) as u8 ^ (ng.uniform() < 0.5) as u8)
                .collect();
            let x = nb::encode_bits(&info);
            for (v, &s) in x.iter().enumerate() {
                metrics(s, g, false, &mut ng, &mut mbuf[v * Q..(v + 1) * Q]);
            }
            match nb::decode_bits(&mbuf) {
                Some(d) if d == info => ok += 1,
                Some(_) => wrong += 1,
                None => {}
            }
        }
        println!(
            "Es/N0 {:.1} dB: {ok} decoded, {wrong} wrong, of {trials}",
            es10 as f64 / 10.0
        );
    }
}
