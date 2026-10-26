// Ion MFSK floor (`ion::phy_mfsk`): round trips, acquisition,
// streaming, false triggers and coded checks. The tables in
// `docs/mfsk.md` come from the `#[ignore]`d generators at the end:
//
// cargo test -j 2 -p ion --release --test phy_mfsk -- --test-threads 2 --ignored --nocapture tables

use ion::fec;
use ion::phy_mfsk::{
    MfskBurst, MfskMode, MfskRx, burst_samples, decode_short_frames, header_only_s, info, max_bits, modulate,
    modulate_symbols,
};
use ion_dsp::TX_LEVEL;
use ion_dsp::channel::{Channel, Profile, add_awgn};
use ion_dsp::dsp::{AUDIO_RATE, NoiseGen, noise_sigma};

const CENTER: f32 = 1500.0;
/// The channel simulator's Hilbert delay.
const CH_DELAY: u64 = 200;

fn random_bits(n: usize, seed: u64) -> Vec<u8> {
    let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    (0..n)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 33) as u8 & 1
        })
        .collect()
}

fn power(x: &[f32]) -> f32 {
    x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32
}

fn pad(burst: &[f32], lead: f32, tail: f32) -> Vec<f32> {
    let mut v = vec![0.0; (lead * AUDIO_RATE) as usize];
    v.extend_from_slice(burst);
    v.extend(std::iter::repeat_n(0.0, (tail * AUDIO_RATE) as usize));
    v
}

#[allow(clippy::too_many_arguments)]
fn impair(
    audio: &[f32],
    profile: Profile,
    seed: u64,
    snr_db: Option<f32>,
    sig_power: f32,
    offset_hz: f32,
    drift: f32,
) -> Vec<f32> {
    let mut ch = Channel::from_profile(profile, seed).with_offset(offset_hz, drift);
    let mut out = vec![0.0; audio.len()];
    ch.process(audio, &mut out);
    if let Some(snr) = snr_db {
        let sigma = noise_sigma(sig_power, snr, 2400.0, AUDIO_RATE);
        add_awgn(&mut out, sigma, &mut NoiseGen::new(seed ^ 0xABCD));
    }
    out
}

fn rx_all(audio: &[f32], chunk: usize) -> Vec<MfskBurst> {
    let mut rx = MfskRx::new(CENTER);
    let mut out = Vec::new();
    for c in audio.chunks(chunk.max(1)) {
        rx.process(c, &mut out);
    }
    out
}

fn hard(llrs: &[f32]) -> Vec<u8> {
    llrs.iter().map(|&l| u8::from(l < 0.0)).collect()
}

fn bit_errors(a: &[u8], b: &[u8]) -> usize {
    a.iter().zip(b).filter(|(x, y)| x != y).count()
}

#[test]
fn numerology() {
    for mode in MfskMode::ALL {
        let i = info(mode);
        for nbits in [0usize, 1, 13, 1024, 5760] {
            let a = modulate(mode, 1, 2, &random_bits(nbits, 3), CENTER);
            assert_eq!(a.len(), burst_samples(mode, nbits), "{mode:?} {nbits}");
            let peak = a.iter().fold(0f32, |m, v| m.max(v.abs()));
            assert!(
                peak <= TX_LEVEL * 1.0001 && peak > 0.9 * TX_LEVEL,
                "{mode:?} peak {peak}"
            );
        }
        assert!(i.raw_bps > 0.0 && i.overhead_s > 0.0);
        println!(
            "{mode:?}: {:.1} bit/s raw, overhead {:.3} s, header-only {:.3} s, max {} bits",
            i.raw_bps,
            i.overhead_s,
            header_only_s(mode),
            max_bits(mode)
        );
    }
}

#[test]
fn clean_round_trip_both_modes() {
    for mode in MfskMode::ALL {
        for (nbits, seed) in [(1024usize, 1u64), (5760, 2), (7, 3)] {
            let bits = random_bits(nbits, seed);
            let audio = pad(&modulate(mode, 0xA5C3, 0x1234, &bits, CENTER), 0.3, 0.5);
            let got = rx_all(&audio, 4800);
            assert_eq!(got.len(), 1, "{mode:?} {nbits}");
            let b = &got[0];
            assert_eq!((b.mode, b.user, b.extra), (mode, 0xA5C3, 0x1234));
            assert_eq!(b.llrs.len(), nbits);
            assert_eq!(hard(&b.llrs), bits, "{mode:?} {nbits}");
            let lead = (0.3 * AUDIO_RATE) as i64;
            assert!(
                (b.start_sample as i64 - lead).abs() <= 12,
                "start {} vs {lead}",
                b.start_sample
            );
            assert!(b.freq_offset_hz.abs() < 0.5, "{}", b.freq_offset_hz);
            println!("{mode:?} {nbits}: snr {:.1} dB", b.snr_db);
        }
    }
}

#[test]
fn header_only_bursts() {
    for mode in MfskMode::ALL {
        let burst = modulate(mode, 0xBEEF, 0x0042, &[], CENTER);
        assert_eq!(burst.len(), burst_samples(mode, 0));
        let p = power(&burst);
        for (snr, seed) in [(None, 1u64), (Some(0.0), 2), (Some(-6.0), 3)] {
            let audio = impair(&pad(&burst, 0.4, 0.4), Profile::Awgn, seed, snr, p, 0.0, 0.0);
            let got = rx_all(&audio, 960);
            assert_eq!(got.len(), 1, "{mode:?} {snr:?}");
            let b = &got[0];
            assert_eq!((b.mode, b.user, b.extra), (mode, 0xBEEF, 0x0042));
            assert!(b.llrs.is_empty());
            let lead = (0.4 * AUDIO_RATE) as i64 + CH_DELAY as i64;
            assert!(
                (b.start_sample as i64 - lead).abs() <= 24,
                "start {} vs {lead}",
                b.start_sample
            );
        }
        println!("{mode:?} header-only: {:.3} s", header_only_s(mode));
    }
}

#[test]
fn frequency_offsets_and_drift() {
    for mode in MfskMode::ALL {
        let snr = if mode == MfskMode::R0 { -4.0 } else { 2.0 };
        for (k, off) in [-50.0f32, -25.0, 0.0, 25.0, 50.0].into_iter().enumerate() {
            let info = random_bits(fec::K_SHORT, k as u64);
            let cw = fec::encode_short(&info);
            let burst = modulate(mode, 7, 8, &cw, CENTER);
            let audio = impair(
                &pad(&burst, 0.3, 0.5),
                Profile::Awgn,
                40 + k as u64,
                Some(snr),
                power(&burst),
                off,
                0.0,
            );
            let got = rx_all(&audio, 4800);
            assert_eq!(got.len(), 1, "{mode:?} {off} Hz");
            let b = &got[0];
            assert!(
                (b.freq_offset_hz - off).abs() < 1.0,
                "{mode:?}: {} vs {off}",
                b.freq_offset_hz
            );
            assert_eq!(
                fec::decode_short(&b.llrs).as_deref(),
                Some(&info[..]),
                "{mode:?} {off} Hz"
            );
        }
        // A long burst drifting 0.5 Hz/s from +35 Hz.
        let bits = random_bits(5760, 9);
        let burst = modulate(mode, 7, 8, &bits, CENTER);
        let dur = burst.len() as f32 / AUDIO_RATE;
        let audio = impair(
            &pad(&burst, 0.3, 0.5),
            Profile::Awgn,
            99,
            Some(snr + 4.0),
            power(&burst),
            35.0,
            0.5,
        );
        let got = rx_all(&audio, 4800);
        assert_eq!(got.len(), 1, "{mode:?} drift");
        let b = &got[0];
        let ber = bit_errors(&hard(&b.llrs), &bits) as f32 / bits.len() as f32;
        let mean = 35.0 + 0.5 * (0.3 + dur / 2.0);
        println!(
            "{mode:?} drift over {dur:.1} s: BER {ber:.4}, offset {:.2} Hz (mean {mean:.2})",
            b.freq_offset_hz
        );
        assert!(ber < 0.02, "{mode:?} drift BER {ber}");
        assert!((b.freq_offset_hz - mean).abs() < 2.0);
    }
}

#[test]
fn arbitrary_chunking_gives_identical_output() {
    let bits = random_bits(1024, 5);
    let burst = modulate(MfskMode::R1, 3, 4, &bits, CENTER);
    let mut audio = pad(&modulate(MfskMode::R0, 1, 2, &[], CENTER), 0.2, 0.1);
    audio.extend(pad(&burst, 0.0, 0.4));
    let audio = impair(&audio, Profile::CcirFlutter, 5, Some(4.0), power(&burst), 10.0, 0.0);
    let reference = rx_all(&audio, audio.len());
    assert_eq!(reference.len(), 2);
    let mut seed = 7u64;
    // 0 = random sizes.
    for size in [1usize, 7, 480, 4801, 0] {
        let mut rx = MfskRx::new(CENTER);
        let mut out = Vec::new();
        let mut i = 0;
        while i < audio.len() {
            let n = if size == 0 {
                seed = seed
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                1 + (seed >> 33) as usize % 3000
            } else {
                size
            };
            let j = (i + n).min(audio.len());
            rx.process(&audio[i..j], &mut out);
            i = j;
        }
        assert_eq!(out.len(), reference.len(), "chunk {size}");
        for (a, b) in out.iter().zip(&reference) {
            assert_eq!((a.user, a.extra, a.start_sample), (b.user, b.extra, b.start_sample));
            assert_eq!(a.llrs, b.llrs, "chunk {size}");
            assert_eq!(a.snr_db.to_bits(), b.snr_db.to_bits());
        }
    }
}

#[test]
fn back_to_back_bursts() {
    let plan: [(MfskMode, usize, f32); 5] = [
        (MfskMode::R0, 0, 0.0),
        (MfskMode::R1, 1024, 0.0),
        (MfskMode::R0, 300, 0.3),
        (MfskMode::R1, 0, 0.0),
        (MfskMode::R0, 1024, 0.05),
    ];
    let mut audio = vec![0f32; 9600];
    let mut sent = Vec::new();
    for (i, &(mode, n, gap)) in plan.iter().enumerate() {
        let bits = random_bits(n, 50 + i as u64);
        audio.extend(modulate(mode, i as u16, 100 + i as u16, &bits, CENTER));
        audio.extend(std::iter::repeat_n(0.0, (gap * AUDIO_RATE) as usize));
        sent.push((mode, bits));
    }
    audio.extend(std::iter::repeat_n(0.0, 24_000));
    for snr in [None, Some(12.0)] {
        let a = impair(&audio, Profile::Awgn, 3, snr, 0.125, 0.0, 0.0);
        for chunk in [a.len(), 4800] {
            let got = rx_all(&a, chunk);
            assert_eq!(got.len(), plan.len(), "{snr:?}");
            for (i, (b, (mode, bits))) in got.iter().zip(&sent).enumerate() {
                assert_eq!((b.mode, b.user, b.extra), (*mode, i as u16, 100 + i as u16));
                assert_eq!(hard(&b.llrs), *bits, "burst {i} {snr:?}");
            }
        }
    }
}

#[test]
fn busy_during_a_burst_only() {
    let burst = modulate(MfskMode::R0, 1, 2, &random_bits(512, 1), CENTER);
    let audio = pad(&burst, 1.0, 1.0);
    let mut rx = MfskRx::new(CENTER);
    let mut out = Vec::new();
    let mut busy_at = Vec::new();
    for c in audio.chunks(480) {
        rx.process(c, &mut out);
        busy_at.push(rx.busy());
    }
    // 10 ms chunks: the burst starts at chunk 100.
    let end = 100 + burst.len() / 480;
    assert!(busy_at[..100].iter().all(|b| !b), "busy before the burst");
    // Busy once the preamble (0.45 s) has been synced.
    assert!(busy_at[160..end - 5].iter().all(|b| *b), "not busy mid-burst");
    assert!(!busy_at.last().unwrap(), "busy after delivery");
    assert_eq!(out.len(), 1);
}

#[test]
fn noise_only_gives_no_bursts() {
    let mut audio = vec![0f32; 60 * 48_000];
    add_awgn(&mut audio, 0.05, &mut NoiseGen::new(77));
    audio.extend(std::iter::repeat_n(0.0, 48_000));
    audio.extend(std::iter::repeat_n(0.3, 48_000));
    let mut rx = MfskRx::new(CENTER);
    let mut out = Vec::new();
    let (mut busy, mut chunks) = (0, 0);
    for c in audio.chunks(480) {
        rx.process(c, &mut out);
        busy += usize::from(rx.busy());
        chunks += 1;
    }
    println!("noise: {:?}, busy {busy}/{chunks}", rx.stats());
    assert!(out.is_empty());
    assert!(busy * 200 <= chunks, "busy {busy}/{chunks}");
}

#[test]
fn ofdm_bursts_give_no_mfsk_bursts() {
    use ion::phy::{self, Constellation};
    let mut rx = MfskRx::new(CENTER);
    let mut out = Vec::new();
    let mut total = 0.0;
    for (i, c) in Constellation::ALL.into_iter().enumerate() {
        for (j, snr) in [None, Some(20.0), Some(8.0), Some(0.0)].into_iter().enumerate() {
            let bits = random_bits(6000, (10 * i + j) as u64);
            let burst = phy::modulate(c, 0x1234, &bits, CENTER);
            let profile = if j % 2 == 0 { Profile::Awgn } else { Profile::CcirPoor };
            let audio = impair(
                &pad(&burst, 0.3, 0.3),
                profile,
                (i * 4 + j) as u64,
                snr,
                power(&burst),
                0.0,
                0.0,
            );
            total += audio.len() as f32 / AUDIO_RATE;
            for ch in audio.chunks(4800) {
                rx.process(ch, &mut out);
            }
        }
    }
    println!("{total:.0} s of OFDM: {:?}", rx.stats());
    assert!(out.is_empty(), "{} MFSK bursts from OFDM", out.len());
}

/// A sound card running `ppm` slow against the sender's: the burst is
/// stretched by 1 + ppm·10⁻⁶ (16-tap windowed-sinc interpolation).
fn resample(x: &[f32], ppm: f64) -> Vec<f32> {
    const HALF: i64 = 8;
    let r = 1.0 + ppm * 1e-6;
    let n_out = (x.len() as f64 * r) as usize;
    (0..n_out)
        .map(|n| {
            let t = n as f64 / r;
            let i0 = t.floor() as i64;
            let mut acc = 0f64;
            for i in i0 - HALF + 1..=i0 + HALF {
                if i < 0 || i >= x.len() as i64 {
                    continue;
                }
                let d = t - i as f64;
                let s = if d.abs() < 1e-12 {
                    1.0
                } else {
                    (core::f64::consts::PI * d).sin() / (core::f64::consts::PI * d)
                };
                let w = 0.5 + 0.5 * (core::f64::consts::PI * d / HALF as f64).cos();
                acc += x[i as usize] as f64 * s * w;
            }
            acc as f32
        })
        .collect()
}

/// A burst of about `secs` seconds, resampled by `ppm`, through `profile`
/// at `snr` dB: (raw BER of the LLRs' hard decisions, tracked ppm).
fn drift_run(
    mode: MfskMode,
    secs: f32,
    ppm: f64,
    profile: Profile,
    snr: f32,
    seed: u64,
    track: bool,
) -> Option<(f64, f32)> {
    let nbits = ((secs * info(mode).raw_bps) as usize).min(max_bits(mode));
    let bits = random_bits(nbits, seed);
    let burst = modulate(mode, 3, 4, &bits, CENTER);
    let x = resample(&pad(&burst, 0.3, 0.3), ppm);
    let p = power(&burst);
    let a = impair(&x, profile, seed ^ 0x55, Some(snr), p, 0.0, 0.0);
    let mut rx = MfskRx::new(CENTER);
    rx.set_drift_tracking(track);
    let mut out = Vec::new();
    for c in a.chunks(4800) {
        rx.process(c, &mut out);
    }
    let b = out.iter().find(|b| b.user == 3 && !b.llrs.is_empty())?;
    let ber = bit_errors(&hard(&b.llrs), &bits) as f64 / bits.len() as f64;
    Some((ber, b.clock_ppm))
}

/// Long bursts survive ±100 ppm of sample-clock offset: a 60 s burst
/// drifts 6 ms, most of the 8 ms guard, and the offset is tracked. At
/// +200 ppm (12 ms) only tracking keeps the burst.
#[test]
fn sample_clock_drift_100ppm() {
    for (mode, snr) in [(MfskMode::R0, -4.0f32), (MfskMode::R1, 4.0)] {
        for ppm in [-100.0, 100.0, 200.0] {
            let (ber, est) = drift_run(mode, 60.0, ppm, Profile::Awgn, snr, 7, true).expect("burst found");
            eprintln!("{mode:?} {ppm:+} ppm: BER {ber:.2e}, tracked {est:+.1} ppm");
            // The raw BER of a 0 ppm burst here is up to ~5e-4.
            assert!(ber < 3e-3, "{mode:?} {ppm} ppm: BER {ber}");
            assert!((est as f64 - ppm).abs() < 25.0, "{mode:?} {ppm} ppm: tracked {est}");
        }
    }
}

/// Raw BER with and without drift tracking against clock offset (the
/// numbers in `docs/mfsk.md`).
#[test]
#[ignore]
fn diag_clock_drift() {
    let ppms = [0.0f64, 50.0, -100.0, 100.0, 200.0];
    for (mode, profile, snr, label) in [
        (MfskMode::R0, Profile::Awgn, -4.0f32, "AWGN"),
        (MfskMode::R0, Profile::MidDisturbedNvis, 0.0, "NVIS disturbed"),
        (MfskMode::R1, Profile::Awgn, 4.0, "AWGN"),
        (MfskMode::R1, Profile::CcirPoor, 10.0, "CCIR poor"),
    ] {
        for &ppm in &ppms {
            for track in [true, false] {
                let rs: Vec<(f64, f32)> = (0..3u64)
                    .filter_map(|s| drift_run(mode, 60.0, ppm, profile, snr, 60 + s, track))
                    .collect();
                let ber = rs.iter().map(|r| r.0).sum::<f64>() / rs.len().max(1) as f64;
                let est: Vec<String> = rs.iter().map(|r| format!("{:+.0}", r.1)).collect();
                println!(
                    "| {mode:?} | {label} {snr:+} dB | {ppm:+} | {} | {}/3 | {ber:.1e} | {} |",
                    if track { "on" } else { "off" },
                    rs.len(),
                    est.join(" ")
                );
            }
        }
    }
}

/// Symbol-mapped bursts (`modulate_symbols`, the link's GF(32) frames):
/// clean round trip of two codewords in one burst with the right flag,
/// symbol bits and metrics order, and decoding at −8 dB (R0) / −5 dB (R1)
/// on AWGN.
#[test]
fn symbol_mapped_round_trip() {
    for (mode, snr) in [(MfskMode::R0, -8.0f32), (MfskMode::R1, -5.0)] {
        let infos: Vec<Vec<u8>> = (0..2).map(|i| random_bits(fec::K_SHORT, 40 + i)).collect();
        let syms: Vec<u8> = infos.iter().flat_map(|i| fec::nb::encode_bits(i)).collect();
        let burst = modulate_symbols(mode, 9, 10, &syms, CENTER);
        assert_eq!(burst.len(), burst_samples(mode, 5 * syms.len()));
        let got = rx_all(&pad(&burst, 0.3, 0.5), 4800);
        assert_eq!(got.len(), 1);
        let b = &got[0];
        assert!(b.symbols && b.user == 9 && b.extra == 10);
        let bits: Vec<u8> = syms
            .iter()
            .flat_map(|&x| (0..5).rev().map(move |k| (x >> k) & 1))
            .collect();
        assert_eq!(hard(&b.llrs), bits);
        assert_eq!(b.metrics.len(), syms.len() * 32);
        for (p, &x) in syms.iter().enumerate() {
            assert_eq!(b.metrics[p * 32 + x as usize], 0.0, "symbol {p}");
        }
        let dec = decode_short_frames(b);
        assert_eq!(dec.len(), 2);
        assert!(dec.iter().zip(&infos).all(|(d, i)| d.as_deref() == Some(&i[..])));
        let p = power(&burst);
        let a = impair(&pad(&burst, 0.3, 0.5), Profile::Awgn, 3, Some(snr), p, 0.0, 0.0);
        let got = rx_all(&a, 4800);
        let dec = decode_short_frames(&got[0]);
        assert!(
            dec.iter().zip(&infos).all(|(d, i)| d.as_deref() == Some(&i[..])),
            "{mode:?} at {snr} dB"
        );
    }
}

#[test]
fn coded_ccir_flutter() {
    for (mode, snr) in [(MfskMode::R0, -6.0f32), (MfskMode::R1, 0.0)] {
        let ok: usize = (0..10u64)
            .map(|seed| run_burst(Cell::Short(mode), Profile::CcirFlutter, snr, 500 + seed).0)
            .sum();
        println!("{mode:?} CCIR flutter {snr} dB: {ok}/10 short codewords");
        assert!(ok >= 9, "{mode:?}: {ok}/10");
    }
}

#[test]
fn llrs_are_calibrated_on_awgn() {
    for (mode, snr) in [(MfskMode::R0, -10.0f32), (MfskMode::R1, -5.0)] {
        let mut bins = [(0usize, 0usize, 0f64); 5];
        for seed in 0..12u64 {
            let bits = random_bits(1024, seed);
            let burst = modulate(mode, 1, 1, &bits, CENTER);
            let audio = impair(
                &pad(&burst, 0.3, 0.4),
                Profile::Awgn,
                300 + seed,
                Some(snr),
                power(&burst),
                0.0,
                0.0,
            );
            let got = rx_all(&audio, 4800);
            let Some(b) = got.first() else { continue };
            for (l, &c) in b.llrs.iter().zip(&bits) {
                let a = l.abs();
                let k = [1.0f32, 2.0, 3.0, 4.5, f32::MAX].iter().position(|&t| a < t).unwrap();
                bins[k].0 += 1;
                bins[k].1 += usize::from(u8::from(*l < 0.0) != c);
                bins[k].2 += 1.0 / (1.0 + (a as f64).exp());
            }
        }
        for (k, b) in bins.iter().enumerate() {
            let (meas, expect) = (b.1 as f64, b.2);
            println!("{mode:?} bin {k}: n {} errors {meas} expected {expect:.1}", b.0);
            if expect >= 30.0 {
                let r = meas / expect;
                assert!((0.6..=1.6).contains(&r), "{mode:?} bin {k}: {meas} vs {expect:.1}");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Table generators (ignored)
// ---------------------------------------------------------------------------

const PROFILES: [(Profile, &str); 6] = [
    (Profile::Awgn, "AWGN"),
    (Profile::CcirPoor, "CCIR poor (2 ms, 1 Hz)"),
    (Profile::CcirFlutter, "CCIR flutter (0.5 ms, 10 Hz)"),
    (Profile::LowDisturbed, "ITU low-lat disturbed (6 ms, 10 Hz)"),
    (Profile::MidDisturbedNvis, "NVIS disturbed (7 ms, 1 Hz)"),
    (Profile::HighDisturbed, "ITU high-lat disturbed (7 ms, 30 Hz)"),
];

fn env_usize(name: &str, default: usize) -> usize {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

/// What one table cell measures.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Cell {
    /// Short-code codewords (one per burst): fraction decoded.
    Short(MfskMode),
    /// Header-only bursts: fraction delivered with the right fields.
    HeaderOnly(MfskMode),
}

/// The decoder the tables measure (`MFSK_DEC`): `nb` (default) the GF(32)
/// code on symbol-mapped bursts, as the link sends them; `id` the binary
/// short code with iterative demapping; `bicm` its bit LLRs alone.
fn dec() -> String {
    std::env::var("MFSK_DEC").unwrap_or_else(|_| "nb".into())
}

/// The burst's one short codeword (see [`dec`]).
fn decode_one(b: &MfskBurst) -> Option<Vec<u8>> {
    if dec() == "bicm" {
        fec::decode_short(&b.llrs)
    } else {
        decode_short_frames(b).into_iter().next().flatten()
    }
}

/// One burst through `profile` at `snr`; returns (successes, trials).
fn run_burst(cell: Cell, profile: Profile, snr: f32, seed: u64) -> (usize, usize) {
    let user = (seed as u16).wrapping_mul(7919);
    let extra = (seed as u16) ^ 0x5A5A;
    let (mode, info_bits, bits) = match cell {
        Cell::Short(m) => {
            let info = random_bits(fec::K_SHORT, seed);
            let cw = if dec() == "nb" {
                fec::nb::encode_bits(&info)
            } else {
                fec::encode_short(&info)
            };
            (m, info, cw)
        }
        Cell::HeaderOnly(m) => (m, Vec::new(), Vec::new()),
    };
    let burst = if dec() == "nb" {
        modulate_symbols(mode, user, extra, &bits, CENTER)
    } else {
        modulate(mode, user, extra, &bits, CENTER)
    };
    let nbits = if dec() == "nb" { 5 * bits.len() } else { bits.len() };
    // `MFSK_PEP=1`: SNR against the mean power of a constant-envelope tone
    // at the same peak (a PEP-limited transmitter), not the burst's own
    // mean power.
    let p = if std::env::var("MFSK_PEP").is_ok_and(|v| v == "1") {
        TX_LEVEL * TX_LEVEL / 2.0
    } else {
        power(&burst)
    };
    let audio = pad(&burst, 0.5, 0.6);
    let rx_audio = impair(&audio, profile, seed, Some(snr), p, 0.0, 0.0);
    let got = rx_all(&rx_audio, 4800);
    let ok = got
        .iter()
        .find(|b| b.mode == mode && b.user == user && b.extra == extra);
    let success = match cell {
        Cell::Short(_) => ok.is_some_and(|b| b.llrs.len() == nbits && decode_one(b).as_deref() == Some(&info_bits[..])),
        Cell::HeaderOnly(_) => ok.is_some_and(|b| b.llrs.is_empty()),
    };
    (usize::from(success), 1)
}

/// Success fractions for every (profile, SNR) of `cells`, on at most two
/// threads (`ION_THREADS`, capped at 2).
fn sweep(cell: Cell, profiles: &[(Profile, &str)], snrs: &[i32], runs: usize) -> Vec<Vec<f32>> {
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicUsize, Ordering};
    let jobs: Vec<(usize, usize, usize)> = (0..profiles.len())
        .flat_map(|p| (0..snrs.len()).flat_map(move |s| (0..runs).map(move |r| (p, s, r))))
        .collect();
    let acc = Mutex::new(vec![vec![(0usize, 0usize); snrs.len()]; profiles.len()]);
    let next = AtomicUsize::new(0);
    let threads = env_usize("ION_THREADS", 2).clamp(1, 2);
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(&(p, k, r)) = jobs.get(i) else { break };
                    let seed = 1000 * p as u64 + 37 * k as u64 + 7919 * r as u64 + 11;
                    let (ok, n) = run_burst(cell, profiles[p].0, snrs[k] as f32, seed);
                    let mut a = acc.lock().unwrap();
                    a[p][k].0 += ok;
                    a[p][k].1 += n;
                }
            });
        }
    });
    acc.into_inner()
        .unwrap()
        .iter()
        .map(|row| row.iter().map(|&(ok, n)| ok as f32 / n.max(1) as f32).collect())
        .collect()
}

fn print_table(title: &str, profiles: &[(Profile, &str)], snrs: &[i32], t: &[Vec<f32>]) {
    println!(
        "
{title}
"
    );
    println!(
        "| Channel | {} |",
        snrs.iter().map(|s| format!("{s:+}")).collect::<Vec<_>>().join(" | ")
    );
    println!("|---|{}", "---:|".repeat(snrs.len()));
    for (p, row) in t.iter().enumerate() {
        println!(
            "| {} | {} |",
            profiles[p].1,
            row.iter().map(|v| format!("{v:.2}")).collect::<Vec<_>>().join(" | ")
        );
    }
}

/// The tables of `docs/mfsk.md`. `MFSK_RUNS` bursts per cell
/// (default 20); `MFSK_PROFILES` = comma-separated indices into PROFILES;
/// `MFSK_CELLS` = any of r0,r1,h0,h1; `MFSK_SNRS` = `lo,hi,step` (default
/// `-14,6,2`); `MFSK_PEP=1` references the SNR to peak power.
#[test]
#[ignore]
fn tables() {
    let runs = env_usize("MFSK_RUNS", 20);
    let grid: Vec<i32> = std::env::var("MFSK_SNRS")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .filter(|g: &Vec<i32>| g.len() == 3 && g[2] > 0)
        .unwrap_or_else(|| vec![-14, 6, 2]);
    let snrs: Vec<i32> = (grid[0]..=grid[1]).step_by(grid[2] as usize).collect();
    let sel: Vec<usize> = std::env::var("MFSK_PROFILES")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| (0..PROFILES.len()).collect());
    let profiles: Vec<(Profile, &str)> = sel.iter().map(|&i| PROFILES[i]).collect();
    let cells = std::env::var("MFSK_CELLS").unwrap_or_else(|_| "r0,r1,h0,h1".into());
    for c in cells.split(',') {
        let (cell, title) = match c.trim() {
            "r0" => (
                Cell::Short(MfskMode::R0),
                "R0, short-code codeword success (1 per burst)",
            ),
            "r1" => (
                Cell::Short(MfskMode::R1),
                "R1, short-code codeword success (1 per burst)",
            ),
            "h0" => (Cell::HeaderOnly(MfskMode::R0), "R0 header-only burst success"),
            "h1" => (Cell::HeaderOnly(MfskMode::R1), "R1 header-only burst success"),
            _ => continue,
        };
        let t = sweep(cell, &profiles, &snrs, runs);
        print_table(title, &profiles, &snrs, &t);
    }
}

#[test]
#[ignore]
fn diag_one() {
    for mode in MfskMode::ALL {
        let bits = random_bits(env_usize("MFSK_NBITS", 1024), 1);
        let audio = pad(&modulate(mode, 0xA5C3, 0x1234, &bits, CENTER), 0.3, 0.5);
        let mut rx = MfskRx::new(CENTER);
        let mut out = Vec::new();
        rx.process(&audio, &mut out);
        println!("{mode:?}: {:?} bursts {}", rx.stats(), out.len());

        for b in &out {
            let h = hard(&b.llrs);
            let errpos: Vec<usize> = (0..bits.len()).filter(|&i| h[i] != bits[i]).take(20).collect();
            println!(
                "  {:?} {} {} start {} f {} snr {} errs {} at {:?}",
                b.mode,
                b.user,
                b.extra,
                b.start_sample,
                b.freq_offset_hz,
                b.snr_db,
                bit_errors(&h, &bits),
                errpos
            );
        }
    }
}

/// Mean reported `snr_db` against the channel SNR (AWGN and CCIR poor, 8
/// data bursts of 1024 bits per point): the gear shifter compares it with
/// `level::THRESHOLDS_DB`.
#[test]
#[ignore]
fn diag_snr_report() {
    for mode in MfskMode::ALL {
        for (profile, name) in [(Profile::Awgn, "AWGN"), (Profile::CcirPoor, "CCIR poor")] {
            let mut line = format!("{mode:?} {name}:");
            for snr in (-12..=12).step_by(3) {
                let mut got = Vec::new();
                for seed in 0..8u64 {
                    let bits = random_bits(1024, seed);
                    let burst = modulate(mode, 5, 6, &bits, CENTER);
                    let audio = pad(&burst, 0.3, 0.4);
                    let a = impair(&audio, profile, seed + 77, Some(snr as f32), power(&burst), 0.0, 0.0);
                    if let Some(b) = rx_all(&a, 4800).iter().find(|b| b.user == 5) {
                        got.push(b.snr_db);
                    }
                }
                let m = got.iter().sum::<f32>() / got.len().max(1) as f32;
                line += &format!(" {snr:+}:{m:+.1}({})", got.len());
            }
            println!("{line}");
        }
    }
}

#[test]
#[ignore]
fn diag_noise() {
    let secs = env_usize("MFSK_NOISE_S", 60);
    let mut noise = vec![0f32; secs * 48_000];
    add_awgn(&mut noise, 0.05, &mut NoiseGen::new(77));
    let mut rx = MfskRx::new(CENTER);
    let mut out = Vec::new();
    let mut busy = 0;
    let mut chunks = 0;
    for c in noise.chunks(480) {
        rx.process(c, &mut out);
        busy += usize::from(rx.busy());
        chunks += 1;
    }
    println!(
        "{secs} s noise: {:?}, bursts {}, busy {busy}/{chunks}",
        rx.stats(),
        out.len()
    );
}

/// One burst in detail: `MFSK_DIAG` = "mode,profile,snr,seeds" (e.g.
/// "1,5,6,10").
#[test]
#[ignore]
fn diag_burst() {
    let spec = std::env::var("MFSK_DIAG").unwrap_or_else(|_| "1,5,6,10".into());
    let v: Vec<f32> = spec.split(',').map(|x| x.trim().parse().unwrap()).collect();
    let mode = if v[0] == 0.0 { MfskMode::R0 } else { MfskMode::R1 };
    let profile = PROFILES[v[1] as usize].0;
    let snr = v[2];
    let mut bins = [(0usize, 0usize, 0f64); 8];
    for seed in 0..v[3] as u64 {
        let info = random_bits(fec::K_SHORT, seed);
        let cw = fec::encode_short(&info);
        let burst = modulate(mode, 1, 2, &cw, CENTER);
        let p = power(&burst);
        let audio = impair(&pad(&burst, 0.5, 0.6), profile, seed + 100, Some(snr), p, 0.0, 0.0);
        let mut rx = MfskRx::new(CENTER);
        let mut out = Vec::new();
        rx.process(&audio, &mut out);
        let st = rx.stats();
        match out.iter().find(|b| b.user == 1 && b.llrs.len() == cw.len()) {
            None => println!("seed {seed}: no burst ({st:?})"),
            Some(b) => {
                let errs = bit_errors(&hard(&b.llrs), &cw);
                let ok = fec::decode_short(&b.llrs).as_deref() == Some(&info[..]);
                for (l, &c) in b.llrs.iter().zip(&cw) {
                    let a = l.abs();
                    let k = [0.5, 1.0, 2.0, 3.0, 4.5, 6.0, 9.0, f32::MAX]
                        .iter()
                        .position(|&t| a < t)
                        .unwrap();
                    bins[k].0 += 1;
                    bins[k].1 += usize::from(u8::from(*l < 0.0) != c);
                    bins[k].2 += 1.0 / (1.0 + (a as f64).exp());
                }
                println!(
                    "seed {seed}: ok {ok} errs {errs} snr {:.1} f {:.2} W {} mi {:?} d {:?} g {:?}",
                    b.snr_db, b.freq_offset_hz, st.last_window, st.last_mi, st.last_d, st.last_gtot
                );
            }
        }
    }
    for (k, b) in bins.iter().enumerate() {
        if b.0 > 0 {
            println!(
                "bin {k}: n {} measured {:.4} claimed {:.4}",
                b.0,
                b.1 as f64 / b.0 as f64,
                b.2 / b.0 as f64
            );
        }
    }
}

/// Detection vs header decoding of header-only bursts: `MFSK_DIAG` =
/// "mode,profile,snr,seeds".
#[test]
#[ignore]
fn diag_detect() {
    let spec = std::env::var("MFSK_DIAG").unwrap_or_else(|_| "0,0,-14,20".into());
    let v: Vec<f32> = spec.split(',').map(|x| x.trim().parse().unwrap()).collect();
    let mode = if v[0] == 0.0 { MfskMode::R0 } else { MfskMode::R1 };
    let profile = PROFILES[v[1] as usize].0;
    let (mut det, mut acc, mut hdr) = (0, 0, 0);
    for seed in 0..v[3] as u64 {
        let burst = modulate(mode, 1, 2, &[], CENTER);
        let audio = impair(
            &pad(&burst, 0.5, 0.6),
            profile,
            seed + 300,
            Some(v[2]),
            power(&burst),
            0.0,
            0.0,
        );
        let mut rx = MfskRx::new(CENTER);
        let mut out = Vec::new();
        rx.process(&audio, &mut out);
        let st = rx.stats();
        det += usize::from(st.detections > 0);
        acc += usize::from(st.accepted > 0);
        hdr += usize::from(out.iter().any(|b| b.user == 1 && b.extra == 2));
    }
    println!("{spec}: detected {det}, accepted {acc}, decoded {hdr}");
}

/// Envelope PAPR, audio crest factor, mean power against the fixed peak,
/// and the spectrum outside the band. `cargo test ... -- --ignored
/// --nocapture papr_and_spectrum`
#[test]
#[ignore]
fn papr_and_spectrum() {
    use ion_dsp::dsp::{Complex, Fir, Nco, lowpass_taps};
    for mode in MfskMode::ALL {
        let (mut peak_env, mut env_sum, mut n_env) = (0f64, 0f64, 0usize);
        let (mut peak_audio, mut audio_sum, mut n_audio) = (0f32, 0f64, 0usize);
        let mut env_hist = vec![0usize; 200];
        let mut psd = vec![0f64; 161];
        let table: Vec<(f64, f64)> = (0..161)
            .flat_map(|fi| {
                (0..4096).map(move |n| {
                    let w = 0.5 - 0.5 * (core::f64::consts::TAU * n as f64 / 4096.0).cos();
                    let a = core::f64::consts::TAU * fi as f64 * 25.0 * n as f64 / 48_000.0;
                    (w * a.cos(), w * a.sin())
                })
            })
            .collect();
        for seed in 0..20u64 {
            let burst = modulate(mode, 1, 2, &random_bits(2048, seed), CENTER);
            // Envelope: mix down by the centre, low-pass, |z|.
            let mut nco = Nco::new(CENTER, AUDIO_RATE);
            let mut lp = Fir::new(lowpass_taps(481, 1500.0, AUDIO_RATE));
            let delay = 240;
            for (i, &x) in burst.iter().enumerate() {
                let z: Complex = lp.filter(nco.mix_down(x).scale(2.0));
                peak_audio = peak_audio.max(x.abs());
                audio_sum += (x * x) as f64;
                n_audio += 1;
                if i >= delay + 1344 && i + 1344 < burst.len() {
                    let e = z.norm_sqr() as f64;
                    peak_env = peak_env.max(e);
                    env_sum += e;
                    n_env += 1;
                    let db = 10.0 * (e / 0.125 / 2.0).max(1e-12).log10();
                    let k = ((db + 20.0) * 5.0).clamp(0.0, 199.0) as usize;
                    env_hist[k] += 1;
                }
            }
            // Averaged periodogram, 25 Hz steps, Hann-windowed 4096 blocks
            // (first 3 bursts).
            if seed < 3 {
                for blk in burst.as_chunks::<4096>().0 {
                    for (fi, p) in psd.iter_mut().enumerate() {
                        let tw = &table[fi * 4096..(fi + 1) * 4096];
                        let (mut re, mut im) = (0f64, 0f64);
                        for (&x, &(c, s)) in blk.iter().zip(tw) {
                            re += x as f64 * c;
                            im -= x as f64 * s;
                        }
                        *p += re * re + im * im;
                    }
                }
            }
        }
        let mean_env = env_sum / n_env as f64;
        let mean_audio = audio_sum / n_audio as f64;
        // 99.9% point of the envelope CCDF.
        let total: usize = env_hist.iter().sum();
        let mut acc = 0;
        let mut p999 = 0.0;
        for (k, &c) in env_hist.iter().enumerate().rev() {
            acc += c;
            if acc as f64 >= 0.001 * total as f64 {
                p999 = k as f64 / 5.0 - 20.0;
                break;
            }
        }
        let p999 = p999 - 10.0 * (mean_env / 0.125 / 2.0).log10();
        println!(
            "{mode:?}: envelope PAPR {:.2} dB (99.9% {:.1} dB), audio crest {:.2} dB, mean power {:.2} dB vs TX_LEVEL² peak",
            10.0 * (peak_env / mean_env).log10(),
            p999,
            10.0 * ((peak_audio as f64).powi(2) / mean_audio).log10(),
            10.0 * (mean_audio / (TX_LEVEL as f64).powi(2)).log10()
        );
        let pmax = psd.iter().cloned().fold(0.0, f64::max);
        let tot: f64 = psd.iter().sum();
        let inband: f64 = psd
            .iter()
            .enumerate()
            .filter(|(i, _)| (12..=108).contains(i))
            .map(|(_, p)| p)
            .sum();
        print!("  PSD dB rel. peak:");
        for f in [100, 200, 250, 275, 300, 2700, 2725, 2750, 2800, 2900, 3000, 3500] {
            print!(" {f}:{:.0}", 10.0 * (psd[f / 25] / pmax).log10());
        }
        println!("\n  power outside 300-2700 Hz: {:.2}%", 100.0 * (1.0 - inband / tot));
    }
}

/// Receiver and modulator CPU time on one core.
#[test]
#[ignore]
fn cpu_cost() {
    use std::time::Instant;
    for mode in MfskMode::ALL {
        let mut audio = vec![0f32; 48_000];
        let mut tx_time = 0.0;
        let mut tx_audio = 0usize;
        let mut k = 0;
        while audio.len() < 60 * 48_000 {
            let bits = random_bits(2048, k);
            let t = Instant::now();
            let b = modulate(mode, k as u16, 0, &bits, CENTER);
            tx_time += t.elapsed().as_secs_f64();
            tx_audio += b.len();
            audio.extend(b);
            audio.extend(std::iter::repeat_n(0.0, 9600));
            k += 1;
        }
        let p = power(&modulate(mode, 0, 0, &random_bits(2048, 0), CENTER));
        let audio = impair(&audio, Profile::CcirPoor, 1, Some(6.0), p, 0.0, 0.0);
        let mut rx = MfskRx::new(CENTER);
        let mut out = Vec::new();
        let mut worst = 0f64;
        let t0 = Instant::now();
        for c in audio.chunks(480) {
            let t = Instant::now();
            rx.process(c, &mut out);
            worst = worst.max(t.elapsed().as_secs_f64());
        }
        let total = t0.elapsed().as_secs_f64();
        let secs = audio.len() as f64 / 48_000.0;
        let users: Vec<u16> = out.iter().map(|b| b.user).collect();
        println!("  delivered users {users:?}, {:?}", rx.stats());
        println!(
            "{mode:?}: RX {:.1} ms per s of audio ({} bursts of {k}), worst 10 ms chunk {:.1} ms; TX {:.1} ms per s of burst",
            1000.0 * total / secs,
            out.len(),
            1000.0 * worst,
            1000.0 * tx_time / (tx_audio as f64 / 48_000.0)
        );
    }
}

/// ln I0(x) (Abramowitz & Stegun 9.8.1 / 9.8.2).
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

/// Tone log-likelihoods of one ideal noncoherent `m`-FSK symbol carrying
/// `tone` at Es/N0 `g` (linear): AWGN with a random phase, or Rayleigh
/// fading independent per symbol.
fn fsk_metrics(tone: usize, m: usize, g: f64, fading: bool, ng: &mut NoiseGen) -> Vec<f64> {
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
            if k == tone {
                re += g.sqrt() * hr;
                im += g.sqrt() * hi;
            }
            let y = re * re + im * im;
            if fading {
                y * g / (1.0 + g)
            } else {
                ln_i0(2.0 * (g * y).sqrt())
            }
        })
        .collect()
}

/// The "Design numbers" of `docs/mfsk.md`: short-code success
/// against Es/N0 on synthetic noncoherent M-FSK LLRs (exact metrics, Gray
/// labels, ideal symbol-by-symbol channel: AWGN with random phase, or
/// independent Rayleigh fading per symbol).
#[test]
#[ignore]
fn diag_ldpc_threshold() {
    fn lse(a: f64, b: f64) -> f64 {
        let (hi, lo) = if a > b { (a, b) } else { (b, a) };
        if lo == f64::NEG_INFINITY {
            hi
        } else {
            hi + (lo - hi).exp().ln_1p()
        }
    }
    let mut ng = NoiseGen::new(5);
    for (m_bits, fading) in [(5usize, false), (5, true), (4, false), (4, true)] {
        let m = 1usize << m_bits;
        let mut line = format!("M={m} {}:", if fading { "Rayleigh" } else { "AWGN" });
        for es10 in (30..=90).step_by(5) {
            let g = 10f64.powf(es10 as f64 / 100.0);
            let trials = 60;
            let mut ok = 0;
            for t in 0..trials {
                let info = random_bits(fec::K_SHORT, t as u64);
                let cw = fec::encode_short(&info);
                let nsym = cw.len().div_ceil(m_bits);
                let mut llr = vec![0f32; nsym * m_bits];
                for s in 0..nsym {
                    let label = (0..m_bits).fold(0usize, |a, b| {
                        a | (cw.get(s * m_bits + b).copied().unwrap_or(0) as usize) << b
                    });
                    let tone = (0..m).find(|&k| (k ^ (k >> 1)) == label).unwrap();
                    let lam = fsk_metrics(tone, m, g, fading, &mut ng);
                    for b in 0..m_bits {
                        let (mut l0, mut l1) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
                        for (k, &l) in lam.iter().enumerate() {
                            if ((k ^ (k >> 1)) >> b) & 1 == 0 {
                                l0 = lse(l0, l);
                            } else {
                                l1 = lse(l1, l);
                            }
                        }
                        llr[s * m_bits + b] = (l0 - l1).clamp(-40.0, 40.0) as f32;
                    }
                }
                if fec::decode_short(&llr[..cw.len()]).as_deref() == Some(&info[..]) {
                    ok += 1;
                }
            }
            line += &format!(" {:.1}:{:.2}", es10 as f64 / 10.0, ok as f64 / trials as f64);
        }
        println!("{line}");
    }
}

/// BICM against iterative demapping (BICM-ID) on synthetic noncoherent
/// 32-FSK: the PHY's own scrambler, interleaver and Gray labels
/// (`phy_mfsk::debug::data_tones`), exact tone metrics, ideal channel
/// (AWGN with a random phase, or independent Rayleigh per symbol).
/// `MFSK_FRAMES` short codewords per burst (default 1), `MFSK_TRIALS`
/// bursts per point (default 60). Prints the codeword success of each.
#[test]
#[ignore]
fn diag_bicm_id() {
    use ion::phy_mfsk::debug::data_tones;
    use ion::phy_mfsk::demap;
    let frames = env_usize("MFSK_FRAMES", 1);
    let trials = env_usize("MFSK_TRIALS", 60);
    for fading in [false, true] {
        let mut l_b = format!("{} BICM   :", if fading { "Rayleigh" } else { "AWGN    " });
        let mut l_i = format!("{} BICM-ID:", if fading { "Rayleigh" } else { "AWGN    " });
        let grid: Vec<i32> = if fading {
            (40..=80).step_by(5).collect()
        } else {
            (25..=65).step_by(5).collect()
        };
        for &es10 in &grid {
            let g = 10f64.powf(es10 as f64 / 100.0);
            let mut ng = NoiseGen::new(9 + es10 as u64);
            let (mut ok_b, mut ok_i) = (0, 0);
            for t in 0..trials {
                let infos: Vec<Vec<u8>> = (0..frames)
                    .map(|f| random_bits(fec::K_SHORT, (t * frames + f) as u64 * 31 + 7))
                    .collect();
                let bits: Vec<u8> = infos.iter().flat_map(|i| fec::encode_short(i)).collect();
                let tones = data_tones(MfskMode::R0, &bits);
                let mut metrics = Vec::with_capacity(tones.len() * 32);
                for &v in &tones {
                    let lam = fsk_metrics(v as usize, 32, g, fading, &mut ng);
                    let mx = lam.iter().cloned().fold(f64::MIN, f64::max);
                    metrics.extend(lam.iter().map(|l| (l - mx).max(-40.0) as f32));
                }
                let mut b = MfskBurst {
                    symbols: false,
                    mode: MfskMode::R0,
                    user: 0,
                    extra: 0,
                    llrs: vec![0.0; bits.len()],
                    metrics,
                    snr_db: 0.0,
                    freq_offset_hz: 0.0,
                    start_sample: 0,
                    clock_ppm: 0.0,
                    chan: Default::default(),
                };
                b.llrs = demap(&b, &vec![0.0; bits.len()]);
                let n = fec::n_short();
                for (f, info) in infos.iter().enumerate() {
                    ok_b += usize::from(fec::decode_short(&b.llrs[f * n..(f + 1) * n]).as_deref() == Some(&info[..]));
                }
                for (got, info) in decode_short_frames(&b).iter().zip(&infos) {
                    ok_i += usize::from(got.as_deref() == Some(&info[..]));
                }
            }
            let tot = (trials * frames) as f64;
            l_b += &format!(" {:.1}:{:.2}", es10 as f64 / 10.0, ok_b as f64 / tot);
            l_i += &format!(" {:.1}:{:.2}", es10 as f64 / 10.0, ok_i as f64 / tot);
        }
        println!(
            "{l_b}
{l_i}"
        );
    }
}

/// Drift tracking at the floor: R0 link-sized bursts (8 s) at −12 dB with
/// no offset, tracking on and off (it must not hurt).
#[test]
#[ignore]
fn diag_clock_floor() {
    for snr in [-12.0f32, -10.0] {
        for track in [true, false] {
            let rs: Vec<(f64, f32)> = (0..16u64)
                .filter_map(|s| drift_run(MfskMode::R0, 8.0, 0.0, Profile::Awgn, snr, 300 + s, track))
                .collect();
            let ber = rs.iter().map(|r| r.0).sum::<f64>() / rs.len().max(1) as f64;
            let worst = rs.iter().map(|r| r.1.abs()).fold(0f32, f32::max);
            println!(
                "R0 AWGN {snr} dB track {track}: {}/16 found, BER {ber:.3e}, worst |ppm| {worst:.0}",
                rs.len()
            );
        }
    }
}
