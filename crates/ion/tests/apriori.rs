// A priori decoding of net check-ins (`ion::link::apriori`).
//
// A check-in goes through the real MFSK R0 modulator, the Watterson
// simulator and the real receiver; the burst it delivers is then decoded
// blind, with the check-in format as a prior, and with a roster of calls.
// The table comes from the ignored generator:
//
// cargo test -p ion --release --test apriori -- --ignored --nocapture

use ion::fec;
use ion::link::apriori::{Assume, Pass, decode_checkin, try_checkin};
use ion::link::audio::bytes_to_bits;
use ion::link::net::{Caps, checkin_frame_bytes, checkin_user};
use ion::phy_mfsk::{MfskBurst, MfskMode, MfskRx, modulate, modulate_symbols};
use ion_dsp::channel::{Channel, Profile, add_awgn};
use ion_dsp::dsp::{AUDIO_RATE, NoiseGen, noise_sigma};

const CENTER: f32 = 1500.0;
const NET: u8 = 0x5A;

/// A roster of regulars; the caller is one of them unless a test says not.
const ROSTER: [&str; 20] = [
    "N0JLF", "AG5B", "W1AW", "K4XYZ", "N5ABC", "KD9QRS", "W7DEF", "KB0GHI", "AE4JKL", "N8MNO", "K0PQR", "W5STU",
    "KC3VWX", "N2YZA", "AB1BCD", "K6EFG", "W9HIJ", "KE7KLM", "N4NOP", "AD0QRS",
];

/// `MFSK_CODE=nb`: the table generators send check-ins in the GF(32) short
/// code (`fec::nb`, symbol-mapped bursts), as the link does; by default in
/// the binary short code.
fn nb_env() -> bool {
    std::env::var("MFSK_CODE").is_ok_and(|v| v == "nb")
}

/// Data LLRs of a one-check-in burst: one binary short codeword, or one
/// GF(32) codeword (five bits per symbol).
fn checkin_len(len: usize) -> bool {
    len == fec::n_short() || len == 5 * fec::nb::N
}

fn checkin_audio(call: &str, seed: u64) -> Vec<f32> {
    checkin_audio_code(call, seed, nb_env())
}

/// A check-in burst in the binary short code or (`nb`) the GF(32) one.
fn checkin_audio_code(call: &str, seed: u64, nb: bool) -> Vec<f32> {
    // The variable fields change from burst to burst, as on the air.
    let snr = (seed % 30) as f32 - 15.0;
    let caps = Caps {
        qsy: seed & 1 == 1,
        ..Caps::default()
    };
    let info = bytes_to_bits(&checkin_frame_bytes(call, snr, caps, 0x01));
    if nb {
        let syms = fec::nb::encode_bits(&info);
        modulate_symbols(MfskMode::R0, checkin_user(NET), 0, &syms, CENTER)
    } else {
        let bits = fec::encode_short(&info);
        modulate(MfskMode::R0, checkin_user(NET), 0, &bits, CENTER)
    }
}

fn power(x: &[f32]) -> f32 {
    x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32
}

/// Burst after 0.5 s of silence, through `profile` at `snr` dB (in
/// 2.4 kHz, against the burst's own power), then received.
fn receive(burst: &[f32], profile: Profile, snr: f32, seed: u64) -> Vec<MfskBurst> {
    let lead = (0.5 * AUDIO_RATE) as usize;
    let mut audio = vec![0.0; lead];
    audio.extend_from_slice(burst);
    audio.extend(std::iter::repeat_n(0.0, lead));
    let mut ch = Channel::from_profile(profile, seed);
    let mut out = vec![0.0; audio.len()];
    ch.process(&audio, &mut out);
    let sigma = noise_sigma(power(burst), snr, 2400.0, AUDIO_RATE);
    add_awgn(&mut out, sigma, &mut NoiseGen::new(seed ^ 0xA5A5));
    let mut rx = MfskRx::new(CENTER);
    let mut got = Vec::new();
    for c in out.chunks(4800) {
        rx.process(c, &mut got);
    }
    got
}

/// The check-in burst among the receiver's output, if its header decoded.
fn checkin_burst(got: Vec<MfskBurst>) -> Option<MfskBurst> {
    got.into_iter()
        .find(|b| b.user == checkin_user(NET) && checkin_len(b.llrs.len()))
}

#[test]
fn clean_checkin_decodes_on_every_pass() {
    let b = checkin_burst(receive(&checkin_audio("AG5B", 3), Profile::Awgn, 20.0, 3)).unwrap();
    for a in [Assume::Nothing, Assume::Format, Assume::Call("AG5B")] {
        let (_, call) = try_checkin(&b, a).unwrap_or_else(|| panic!("{a:?}"));
        assert_eq!(call, "AG5B");
    }
    let c = decode_checkin(&b, &ROSTER).unwrap();
    assert_eq!((c.call.as_str(), c.pass), ("AG5B", Pass::Blind));
}

/// The same with the GF(32) short code the link sends: the priors reach its
/// decoder (`fec::nb::decode_bits_prior`).
#[test]
fn gf32_clean_checkin_decodes_on_every_pass() {
    let a = checkin_audio_code("AG5B", 3, true);
    let b = checkin_burst(receive(&a, Profile::Awgn, 20.0, 3)).unwrap();
    assert!(b.symbols);
    for a in [Assume::Nothing, Assume::Format, Assume::Call("AG5B")] {
        let (_, call) = try_checkin(&b, a).unwrap_or_else(|| panic!("{a:?}"));
        assert_eq!(call, "AG5B");
    }
}

/// GF(32): a caller missing from the roster, decoded with every roster
/// call assumed, fails or comes out as itself.
#[test]
fn gf32_wrong_roster_never_misdecodes() {
    let mut runs = 0;
    for seed in 0..6u64 {
        let a = checkin_audio_code("VE3ZZZ", seed, true);
        let Some(b) = checkin_burst(receive(&a, Profile::Awgn, -14.0, 100 + seed)) else {
            continue;
        };
        runs += 1;
        for call in ROSTER {
            if let Some((_, c)) = try_checkin(&b, Assume::Call(call)) {
                assert_eq!(c, "VE3ZZZ", "assumed {call}, seed {seed}");
            }
        }
    }
    assert!(runs >= 3, "too few headers decoded to test ({runs})");
}

/// GF(32): pure-noise tone metrics with a full call prior must not pass
/// the CRC.
#[test]
fn gf32_noise_with_a_prior_decodes_nothing() {
    let mut g = NoiseGen::new(8);
    for t in 0..100 {
        let m: Vec<f32> = (0..fec::nb::N * fec::nb::Q).map(|_| 2.0 * g.gaussian()).collect();
        let prior = ion::link::apriori::checkin_prior(Assume::Call(ROSTER[t % ROSTER.len()]));
        if let Some(info) = fec::nb::decode_bits_prior(&m, &prior) {
            let bytes = ion::link::audio::bits_to_bytes(&info);
            assert!(
                ion::link::frame::Frame::decode(&bytes).is_none(),
                "trial {t}: noise passed the CRC"
            );
        }
    }
}

/// A wrong guess must not invent a check-in: a caller missing from the
/// roster, decoded with every roster call assumed, either fails or comes
/// out as itself.
#[test]
fn wrong_roster_never_misdecodes() {
    let mut runs = 0;
    for seed in 0..6u64 {
        let got = receive(&checkin_audio("VE3ZZZ", seed), Profile::Awgn, -13.0, 100 + seed);
        let Some(b) = checkin_burst(got) else {
            continue;
        };
        runs += 1;
        for call in ROSTER {
            if let Some((_, c)) = try_checkin(&b, Assume::Call(call)) {
                assert_eq!(c, "VE3ZZZ", "assumed {call}, seed {seed}");
            }
        }
    }
    assert!(runs >= 3, "too few headers decoded to test ({runs})");
}

/// Pure noise LLRs with a full call prior: the decoder must not converge
/// on the assumed call's codeword (the unknown bytes and the CRC still have
/// to come out right).
#[test]
fn noise_with_a_prior_decodes_nothing() {
    let mut g = NoiseGen::new(7);
    for t in 0..200 {
        let llrs: Vec<f32> = (0..fec::n_short()).map(|_| 2.0 * g.gaussian()).collect();
        let prior = ion::link::apriori::checkin_prior(Assume::Call(ROSTER[t % ROSTER.len()]));
        if let Some(info) = fec::decode_short_prior(&llrs, &prior) {
            let bytes = ion::link::audio::bits_to_bytes(&info);
            assert!(
                ion::link::frame::Frame::decode(&bytes).is_none(),
                "trial {t}: noise passed the CRC"
            );
        }
    }
}

/// Below the blind threshold the priors still decode: on AWGN at −14 dB
/// the roster pass gets more check-ins through than blind decoding.
#[test]
fn priors_decode_below_the_blind_threshold() {
    let (mut blind, mut roster, mut headers) = (0, 0, 0);
    for seed in 0..8u64 {
        let call = ROSTER[seed as usize % ROSTER.len()];
        let Some(b) = checkin_burst(receive(&checkin_audio(call, seed), Profile::Awgn, -14.0, 40 + seed)) else {
            continue;
        };
        headers += 1;
        blind += try_checkin(&b, Assume::Nothing).is_some() as usize;
        if let Some(c) = decode_checkin(&b, &ROSTER) {
            assert_eq!(c.call, call);
            roster += 1;
        }
    }
    assert!(roster > blind, "headers {headers}/8: roster {roster} vs blind {blind}");
}

// ---------------------------------------------------------------------
// Table generator (ignored)
// ---------------------------------------------------------------------

/// Per cell: header decoded, then check-in decoded blind, with the format,
/// with a 20-call roster holding the caller; and wrong calls (must be 0).
#[derive(Default, Clone, Copy)]
struct Cell {
    n: usize,
    header: usize,
    blind: usize,
    format: usize,
    roster: usize,
    wrong: usize,
}

fn run_cell(profile: Profile, snr: f32, n: usize) -> Cell {
    // ION_THREADS: worker threads (default 2; results do not depend
    // on it, every check-in has its own seed).
    let threads = std::env::var("ION_THREADS")
        .ok()
        .and_then(|v| v.parse::<usize>().ok())
        .unwrap_or(2)
        .max(1);
    let cells: Vec<Cell> = std::thread::scope(|s| {
        let hs: Vec<_> = (0..threads)
            .map(|t| {
                s.spawn(move || {
                    let mut c = Cell::default();
                    for i in (t..n).step_by(threads) {
                        let seed = 1000 * (snr as i64 + 100) as u64 + i as u64;
                        let call = ROSTER[i % ROSTER.len()];
                        c.n += 1;
                        let Some(b) = checkin_burst(receive(&checkin_audio(call, seed), profile, snr, seed)) else {
                            continue;
                        };
                        c.header += 1;
                        c.blind += try_checkin(&b, Assume::Nothing).is_some() as usize;
                        let f = try_checkin(&b, Assume::Format);
                        c.format += f.is_some() as usize;
                        match decode_checkin(&b, &ROSTER) {
                            Some(k) if k.call == call => c.roster += 1,
                            Some(_) => c.wrong += 1,
                            None => {}
                        }
                    }
                    c
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).collect()
    });
    cells.iter().fold(Cell::default(), |a, c| Cell {
        n: a.n + c.n,
        header: a.header + c.header,
        blind: a.blind + c.blind,
        format: a.format + c.format,
        roster: a.roster + c.roster,
        wrong: a.wrong + c.wrong,
    })
}

#[test]
#[ignore]
fn apriori_table() {
    let n: usize = std::env::var("APRIORI_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40);
    let grid = [
        (Profile::Awgn, "AWGN", -17..=-10),
        (Profile::CcirPoor, "CCIR poor", -15..=-8),
        (Profile::MidDisturbedNvis, "NVIS disturbed", -13..=-6),
    ];
    for (profile, name, snrs) in grid {
        if !table_profile(name) {
            continue;
        }
        println!("\n{name}, {n} check-ins per SNR (fraction of all sent)\n");
        println!("| SNR dB | header | blind | + format | + roster (20) | wrong |");
        println!("|---:|---:|---:|---:|---:|---:|");
        for snr in snrs.filter(|&s| table_snr(s)) {
            let c = run_cell(profile, snr as f32, n);
            let f = |x: usize| x as f32 / c.n as f32;
            println!(
                "| {snr} | {:.2} | {:.2} | {:.2} | {:.2} | {} |",
                f(c.header),
                f(c.blind),
                f(c.format),
                f(c.roster),
                c.wrong
            );
        }
    }
}

/// `TABLE_PROFILES` (channel names as printed, e.g. `AWGN,CCIR poor`) and
/// `TABLE_SNR_MIN` / `TABLE_SNR_MAX` run part of a table; every check-in
/// keeps its seed.
fn table_profile(name: &str) -> bool {
    std::env::var("TABLE_PROFILES").map_or(true, |v| v.split(',').any(|p| p.trim() == name))
}

fn table_snr(s: i32) -> bool {
    let env = |k: &str, d: i32| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
    (env("TABLE_SNR_MIN", i32::MIN)..=env("TABLE_SNR_MAX", i32::MAX)).contains(&s)
}
