// A priori decoding with the GF(32) short code (`ion::fec::nb`): the
// prior path's sanity, and the information layouts (`nb::Layout`) on ideal
// synthetic noncoherent 32-FSK with net check-in priors (`link::apriori`).
// The tables come from the ignored generators:
//
// cargo test -j 2 -p ion --release --test nb_prior -- --test-threads 2 --ignored --nocapture

use ion::fec::nb::{self, BitMap, InfoSet, Layout, Q};
use ion::link::apriori::{Assume, checkin_prior};
use ion::link::audio::bytes_to_bits;
use ion::link::net::{Caps, checkin_frame_bytes};
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

/// Tone log-likelihoods of one noncoherent 32-FSK symbol at Es/N0 `g`
/// (AWGN, or Rayleigh per tone).
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

fn channel(x: &[u8], es_db: f64, fading: bool, ng: &mut NoiseGen) -> Vec<f32> {
    let g = 10f64.powf(es_db / 10.0);
    let mut m = vec![0f32; nb::N * Q];
    for (v, &s) in x.iter().enumerate() {
        metrics(s, g, fading, ng, &mut m[v * Q..(v + 1) * Q]);
    }
    m
}

const ROSTER: [&str; 4] = ["N0JLF", "AG5B", "KD9QRS", "W1AW"];

/// A check-in with its variable fields drawn from `t`.
fn checkin_bits(t: usize) -> (Vec<u8>, &'static str) {
    let call = ROSTER[t % ROSTER.len()];
    let caps = Caps {
        qsy: t & 1 == 1,
        relay: t & 2 == 2,
        ..Caps::default()
    };
    let bytes = checkin_frame_bytes(call, (t % 37) as f32 - 18.0, caps, (t % 5) as u8);
    (bytes_to_bits(&bytes), call)
}

/// Sure priors on every information bit but `free`.
fn prior_all_but(info: &[u8], free: &[usize]) -> Vec<f32> {
    info.iter()
        .enumerate()
        .map(|(i, &b)| {
            if free.contains(&i) {
                0.0
            } else if b == 0 {
                20.0
            } else {
                -20.0
            }
        })
        .collect()
}

/// Every layout round-trips clean, blind and with priors, and
/// `echelon-plain` is the code's own encoding (bits in order, symbol j in
/// information slot j).
#[test]
fn layouts_round_trip() {
    for l in Layout::all() {
        for t in 0..4 {
            let (info, call) = checkin_bits(t);
            let x = nb::encode_bits_with(l, &info);
            assert!(nb::code().checks_ok(&x));
            let mut m = vec![0f32; nb::N * Q];
            for (v, &s) in x.iter().enumerate() {
                m[v * Q + s as usize] = 4.0;
            }
            assert_eq!(
                nb::decode_bits_prior_with(l, &m, &[]).as_deref(),
                Some(&info[..]),
                "{l:?}"
            );
            let p = checkin_prior(Assume::Call(call));
            assert_eq!(
                nb::decode_bits_prior_with(l, &m, &p).as_deref(),
                Some(&info[..]),
                "{l:?}"
            );
        }
    }
    let (info, _) = checkin_bits(0);
    let plain = nb::code_with(InfoSet::Echelon).encode(&nb::bits_to_syms(&info));
    assert_eq!(
        nb::encode_bits_with(Layout::new(InfoSet::Echelon, BitMap::Plain), &info),
        plain
    );
}

/// The prior path: (a) information tones with no channel at all and sure
/// priors, parity tones clean: the priors alone must give the information
/// (a wrong bit-to-symbol conversion could not satisfy the checks);
/// (b) every information symbol sure, the parity tones clean but for a
/// dozen erased ones: the check update must carry the certainty into
/// them.
#[test]
fn priors_reach_the_checks() {
    for l in Layout::all() {
        let c = nb::code_with(l.info);
        for t in 0..4 {
            let (info, _) = checkin_bits(t);
            let x = nb::encode_bits_with(l, &info);
            let mut m = vec![0f32; nb::N * Q];
            for (v, &s) in x.iter().enumerate() {
                if !c.info_positions().contains(&v) {
                    m[v * Q + s as usize] = 4.0;
                }
            }
            let p = prior_all_but(&info, &[]);
            assert_eq!(
                nb::decode_bits_prior_with(l, &m, &p).as_deref(),
                Some(&info[..]),
                "(a) {}",
                l.name()
            );
            let mut m = vec![0f32; nb::N * Q];
            let parity: Vec<usize> = (0..nb::N).filter(|v| !c.info_positions().contains(v)).collect();
            for (v, &s) in x.iter().enumerate() {
                m[v * Q + s as usize] = 4.0;
            }
            for k in 0..12 {
                let v = parity[(37 * k + 11 * t) % parity.len()];
                m[v * Q..(v + 1) * Q].fill(0.0);
            }
            assert_eq!(
                nb::decode_bits_prior_with(l, &m, &p).as_deref(),
                Some(&info[..]),
                "(b) {}",
                l.name()
            );
        }
    }
}

/// A check-in with its caller known decodes at +2 dB Es/N0 on AWGN, below
/// the blind threshold, in every layout.
#[test]
fn call_prior_decodes_below_the_blind_threshold() {
    for l in Layout::all() {
        let mut ng = NoiseGen::new(21);
        let mut ok = 0;
        for t in 0..20 {
            let (info, call) = checkin_bits(t);
            let x = nb::encode_bits_with(l, &info);
            let m = channel(&x, 2.0, false, &mut ng);
            let p = checkin_prior(Assume::Call(call));
            ok += usize::from(nb::decode_bits_prior_with(l, &m, &p).as_deref() == Some(&info[..]));
        }
        assert!(ok >= 15, "{}: {ok} of 20 at +2 dB Es/N0", l.name());
    }
}

/// Why belief propagation cannot use the known symbols fully: with every
/// information symbol sure and no channel at all, does it find the parity
/// symbols? Prints, per information set, the degree-6 columns among the
/// information symbols, the answer, and how many symbols greedy peeling has
/// to be given to solve the rest one check at a time (the code's dimension
/// is 52).
#[test]
#[ignore]
fn nb_info_sets() {
    for i in [InfoSet::Echelon, InfoSet::Peeled] {
        let c = nb::code_with(i);
        let p = c.info_positions();
        let heavy = p.iter().filter(|&&v| c.degree(v) > 2).count();
        let l = Layout::new(i, BitMap::Plain);
        let (info, _) = checkin_bits(3);
        let got = nb::decode_bits_prior_with(l, &vec![0f32; nb::N * Q], &prior_all_but(&info, &[]));
        println!(
            "{i:?}: {heavy} of 20 degree-6 columns are information; certain information alone decodes: {}; positions {p:?}",
            got.as_deref() == Some(&info[..])
        );
    }
    let c = nb::code_with(InfoSet::Echelon);
    println!(
        "greedy peeling picks {} symbols",
        c.peel_picks(|v| c.degree(v) as i64).len()
    );
}

/// Check-in success against Es/N0 for every layout and pass (blind, the
/// format known, the call known too), `NB_TRIALS` check-ins per point
/// (default 200), AWGN and Rayleigh per tone. `NB_LAYOUTS`: comma-separated
/// names to run; `ION_NB_FALLBACK` the fallback.
#[test]
#[ignore]
fn nb_layout_table() {
    let trials: usize = std::env::var("NB_TRIALS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(200);
    let sel: Vec<Layout> = std::env::var("NB_LAYOUTS")
        .ok()
        .map(|v| v.split(',').filter_map(Layout::from_name).collect())
        .unwrap_or_else(Layout::all);
    for fading in [false, true] {
        let grid: Vec<i32> = if fading { (-2..=6).collect() } else { (-4..=4).collect() };
        println!(
            "\n{}, {trials} check-ins per point, success fraction\n",
            if fading { "Rayleigh" } else { "AWGN" }
        );
        println!(
            "| Layout | Pass | {} |",
            grid.iter().map(|e| format!("{e:+} dB")).collect::<Vec<_>>().join(" | ")
        );
        println!("|---|---|{}", "---:|".repeat(grid.len()));
        for &l in &sel {
            for (pass, name) in [
                (Assume::Nothing, "blind"),
                (Assume::Format, "format"),
                (Assume::Call("x"), "call"),
            ] {
                let mut row = Vec::new();
                for &es in &grid {
                    // The same channel noise for every layout and pass.
                    let mut ng = NoiseGen::new(1000 + 31 * (es + 10) as u64 + 7 * fading as u64);
                    let mut ok = 0;
                    for t in 0..trials {
                        let (info, call) = checkin_bits(t);
                        let assume = match pass {
                            Assume::Call(_) => Assume::Call(call),
                            a => a,
                        };
                        let x = nb::encode_bits_with(l, &info);
                        let m = channel(&x, es as f64, fading, &mut ng);
                        let p = checkin_prior(assume);
                        ok += usize::from(nb::decode_bits_prior_with(l, &m, &p).as_deref() == Some(&info[..]));
                    }
                    row.push(format!("{:.2}", ok as f64 / trials as f64));
                }
                println!("| {} | {name} | {} |", l.name(), row.join(" | "));
            }
        }
    }
}
