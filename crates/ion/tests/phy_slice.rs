// OFDM PHY: the 650 Hz Slice numerology (three side by side in an SSB
// channel). Round trips at the three slice centres, AWGN decoding with the
// FEC, rejection of the neighbouring slices, false detections, and the
// threshold table in `docs/phy.md` (Slice numerology):
//
// cargo test -p ion --release --test phy_slice -- --ignored --nocapture slice_table

use ion::fec::{self, Rate};
use ion::phy::{
    Constellation, Numerology, PhyRx, Pilots, RxBurst, TxOptions, burst_samples_with, capacity_bits_with,
    data_symbols_with, info_for, modulate_full,
};
use ion_dsp::channel::{Channel, Profile, add_awgn};
use ion_dsp::dsp::{AUDIO_RATE, NoiseGen, noise_sigma};

const SLICE: Numerology = Numerology::Slice;
/// The three slice centres in a 2.4 kHz SSB channel.
const CENTERS: [f32; 3] = [750.0, 1500.0, 2250.0];

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

/// `lead` seconds of silence, the burst, `tail` seconds of silence.
fn pad(burst: &[f32], lead: f32, tail: f32) -> Vec<f32> {
    let mut v = vec![0.0; (lead * AUDIO_RATE) as usize];
    v.extend_from_slice(burst);
    v.extend(std::iter::repeat_n(0.0, (tail * AUDIO_RATE) as usize));
    v
}

fn bit_errors(llrs: &[f32], bits: &[u8]) -> usize {
    bits.iter().zip(llrs).filter(|(b, l)| (**l < 0.0) as u8 != **b).count()
}

/// A Slice receiver at `center` over `audio` in 4800-sample chunks.
fn rx_slice(center: f32, audio: &[f32]) -> (Vec<RxBurst>, PhyRx) {
    let mut rx = PhyRx::with_numerology(SLICE, center);
    let mut out = Vec::new();
    for c in audio.chunks(4800) {
        rx.process(c, &mut out);
    }
    (out, rx)
}

/// Through `profile` (with a frequency offset) plus AWGN at `snr_db`
/// against `sig_power` in 2.4 kHz.
fn impair(audio: &[f32], profile: Profile, seed: u64, snr_db: Option<f32>, sig_power: f32, offset_hz: f32) -> Vec<f32> {
    let mut ch = Channel::from_profile(profile, seed).with_offset(offset_hz, 0.0);
    let mut out = vec![0.0; audio.len()];
    ch.process(audio, &mut out);
    if let Some(snr) = snr_db {
        let sigma = noise_sigma(sig_power, snr, 2400.0, AUDIO_RATE);
        add_awgn(&mut out, sigma, &mut NoiseGen::new(seed ^ 0xABCD));
    }
    out
}

/// Runs `jobs` on `ION_THREADS` threads (default 2), results in job
/// order.
fn par_map<J: Sync, R: Send>(jobs: &[J], f: impl Fn(&J) -> R + Sync) -> Vec<R> {
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicUsize, Ordering};
    let next = AtomicUsize::new(0);
    let out: Mutex<Vec<Option<R>>> = Mutex::new((0..jobs.len()).map(|_| None).collect());
    let threads: usize = std::env::var("ION_THREADS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(2);
    std::thread::scope(|s| {
        for _ in 0..threads {
            s.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    if i >= jobs.len() {
                        break;
                    }
                    let r = f(&jobs[i]);
                    out.lock().unwrap()[i] = Some(r);
                }
            });
        }
    });
    out.into_inner().unwrap().into_iter().map(|r| r.unwrap()).collect()
}

#[test]
fn slice_layout() {
    let i = info_for(SLICE);
    assert_eq!((i.data_carriers, i.sparse_data_carriers), (10, 11));
    assert!((i.bandwidth_hz - 650.0).abs() < 1e-3);
    assert!((i.symbol_s - 0.024).abs() < 1e-6);
    assert!(SLICE.min_center_hz() <= 750.0);
    assert!(!Numerology::ALL.contains(&SLICE));
    for nbits in [0usize, 1, 2000] {
        let b = random_bits(nbits, 3);
        let a = modulate_full(SLICE, Constellation::Qpsk, 1, 0, &b, 1500.0, &TxOptions::default());
        assert_eq!(
            a.len(),
            burst_samples_with(SLICE, Pilots::Robust, Constellation::Qpsk, nbits)
        );
        if nbits > 0 {
            let secs =
                i.overhead_s + data_symbols_with(SLICE, Pilots::Robust, Constellation::Qpsk, nbits) as f32 * i.symbol_s;
            assert!((a.len() as f32 / AUDIO_RATE - secs).abs() < 1e-4);
        }
        let peak = a.iter().fold(0f32, |m, v| m.max(v.abs()));
        assert!((peak - ion_dsp::TX_LEVEL).abs() < 1e-4);
    }
}

/// The slice's 48 kHz filter: flat over the carriers (±300 Hz), ≥ 70 dB
/// down where the neighbouring slice starts (±450 Hz) and beyond.
#[test]
fn slice_filter_response() {
    let taps = ion::phy::front_end_taps_for(SLICE);
    for f in (0..=300).step_by(10) {
        let g = 20.0 * ion_dsp::dsp::fir_response(&taps, f as f32, AUDIO_RATE).log10();
        assert!(g.abs() < 0.05, "{f} Hz: {g} dB");
    }
    for f in (450..=24_000).step_by(10) {
        let g = 20.0 * ion_dsp::dsp::fir_response(&taps, f as f32, AUDIO_RATE).log10();
        assert!(g < -70.0, "{f} Hz: {g} dB");
    }
}

/// Clean round trips at the three slice centres, both pilot patterns,
/// QPSK and 16QAM, data and header-only. QPSK with the link's clip and
/// 16QAM unclipped must be error-free; 16QAM with the link's clip shows the
/// clip's distortion (concentrated on few carriers), so only ≤ 1% there.
#[test]
fn slice_clean_round_trip() {
    let qam_none = TxOptions {
        clip_db: None,
        pilots: Pilots::Robust,
        ..TxOptions::default()
    };
    for (ci, &center) in CENTERS.iter().enumerate() {
        for p in [Pilots::Robust, Pilots::Sparse] {
            let cases = [
                (Constellation::Qpsk, TxOptions::for_burst(Constellation::Qpsk, p), 0.0),
                (Constellation::Qam16, TxOptions { pilots: p, ..qam_none }, 0.0),
                (
                    Constellation::Qam16,
                    TxOptions::for_burst(Constellation::Qam16, p),
                    0.01,
                ),
            ];
            for (k, (c, o, max_ber)) in cases.into_iter().enumerate() {
                let bits = random_bits(1500 + 37 * k, 40 + k as u64 + 10 * ci as u64);
                let user = 0x3C00 + (ci * 16 + k) as u16;
                let burst = modulate_full(SLICE, c, user, 0, &bits, center, &o);
                assert_eq!(burst.len(), burst_samples_with(SLICE, p, c, bits.len()));
                let (got, _) = rx_slice(center, &pad(&burst, 0.3, 0.3));
                assert_eq!(got.len(), 1, "{center} {p:?} {c:?}");
                let b = &got[0];
                assert_eq!((b.numerology, b.constellation, b.pilots, b.user), (SLICE, c, p, user));
                let ns = data_symbols_with(SLICE, p, c, bits.len());
                assert_eq!(b.llrs.len(), capacity_bits_with(SLICE, p, c, ns));
                let errs = bit_errors(&b.llrs, &bits);
                assert!(
                    errs as f64 <= max_ber * bits.len() as f64,
                    "{center} {p:?} {c:?} clip {:?}: {errs} errors",
                    o.clip_db
                );
                assert!(b.llrs[bits.len()..].iter().all(|&l| l > 0.0));
                let err = b.start_sample as i64 - (0.3 * AUDIO_RATE) as i64;
                assert!(err.abs() <= 48, "{center} start error {err}");
            }
        }
        // Header-only.
        let burst = modulate_full(
            SLICE,
            Constellation::Qpsk,
            0xBEEF,
            0x1234,
            &[],
            center,
            &TxOptions::default(),
        );
        let (got, _) = rx_slice(center, &pad(&burst, 0.3, 0.3));
        assert_eq!(got.len(), 1);
        assert!(got[0].header_only);
        assert_eq!((got[0].user, got[0].extra), (0xBEEF, 0x1234));
    }
}

/// Frequency offsets within the ±50 Hz pull-in of A (10 ms period).
#[test]
fn slice_frequency_offsets() {
    let c = Constellation::Qpsk;
    for (k, off) in [-30.0f32, -12.5, 0.0, 7.0, 30.0].into_iter().enumerate() {
        let bits = random_bits(2000, 70 + k as u64);
        let burst = modulate_full(SLICE, c, 9, 0, &bits, 1500.0, &TxOptions::default());
        let p = power(&burst);
        let a = impair(
            &pad(&burst, 0.3, 0.3),
            Profile::Awgn,
            300 + k as u64,
            Some(10.0),
            p,
            off,
        );
        let (got, _) = rx_slice(1500.0, &a);
        assert_eq!(got.len(), 1, "offset {off}");
        let b = &got[0];
        let ber = bit_errors(&b.llrs, &bits) as f32 / bits.len() as f32;
        assert!((b.freq_offset_hz - off).abs() < 0.5);
        assert!(ber < 1e-3);
    }
}

/// Per-carrier error of the equalised data cells on a clean 64QAM burst
/// (unclipped), against the nearest point: where the slice's floor is.
#[test]
#[ignore]
fn diag_slice_cells() {
    let c = Constellation::Qam64;
    let pts: Vec<ion_dsp::dsp::Complex> = {
        let k = 1.0 / 42f32.sqrt();
        let mut v = Vec::new();
        for i in 0..8 {
            for q in 0..8 {
                v.push(ion_dsp::dsp::Complex::new(
                    k * (2 * i - 7) as f32,
                    k * (2 * q - 7) as f32,
                ));
            }
        }
        v
    };
    for num in [Numerology::Normal, SLICE] {
        for clip in [None, Some(8.0f32), Some(6.0), Some(5.0), Some(4.5)] {
            let bits = random_bits(12000, 5);
            let o = TxOptions {
                clip_db: clip,
                pilots: Pilots::Robust,
                ..TxOptions::default()
            };
            let burst = modulate_full(num, c, 1, 0, &bits, 1500.0, &o);
            let mut rx = PhyRx::with_numerology(num, 1500.0);
            rx.set_debug(true);
            let mut out = Vec::new();
            rx.process(&pad(&burst, 0.3, 0.3), &mut out);
            let mut e = vec![0f64; 120];
            let mut m = vec![0f64; 120];
            let mut n = vec![0f64; 120];
            for d in rx.debug_cells() {
                let z = d.y * d.h.conj() * (1.0 / d.h.norm_sqr());
                let best = pts.iter().map(|p| (z - *p).norm_sqr()).fold(f32::INFINITY, f32::min);
                e[d.c] += best as f64;
                m[d.c] += (d.mse / d.h.norm_sqr()) as f64;
                n[d.c] += 1.0;
            }
            let line: Vec<String> = (0..120)
                .filter(|&k| n[k] > 0.0)
                .map(|k| {
                    format!(
                        "{k}:{:.0}/{:.0}",
                        -10.0 * (e[k] / n[k]).log10(),
                        -10.0 * (m[k] / n[k]).log10()
                    )
                })
                .collect();
            eprintln!("{num:?} {clip:?} model {:?}: {}", rx.debug_model(), line.join(" "));
        }
    }
}

/// Equalised-cell error (dB below the cell power) on two-path channels
/// without noise, unclipped 16QAM, genie-free: per carrier and over the
/// burst, for Normal and Slice; where the slice's fading floor comes from.
#[test]
#[ignore]
fn diag_slice_fading() {
    use ion_dsp::channel::ChannelSpec;
    let k = 1.0 / 10f32.sqrt();
    let pts: Vec<ion_dsp::dsp::Complex> = (0..16)
        .map(|m| ion_dsp::dsp::Complex::new(k * (2 * (m / 4) - 3) as f32, k * (2 * (m % 4) - 3) as f32))
        .collect();
    let c = Constellation::Qam16;
    let specs = [
        ChannelSpec::two_path("static 1 ms", 1.0, 0.0),
        ChannelSpec::two_path("static 2 ms", 2.0, 0.0),
        ChannelSpec::two_path("static 3 ms", 3.0, 0.0),
        ChannelSpec::two_path("2 ms 0.1 Hz", 2.0, 0.1),
        ChannelSpec::two_path("2 ms 1 Hz", 2.0, 1.0),
    ];
    for spec in &specs {
        for num in [Numerology::Normal, SLICE] {
            let jobs: Vec<u64> = (0..8).collect();
            let res = par_map(&jobs, |&s| {
                let nbits = capacity_bits_with(num, Pilots::Robust, c, 48);
                let bits = random_bits(nbits, 11 + s);
                let o = TxOptions {
                    clip_db: None,
                    pilots: Pilots::Robust,
                    ..TxOptions::default()
                };
                let burst = modulate_full(num, c, 1, 0, &bits, 1500.0, &o);
                let audio = pad(&burst, 0.25, 0.3);
                let mut ch = Channel::new(spec, 50 + s);
                let mut a = vec![0.0; audio.len()];
                ch.process(&audio, &mut a);
                let mut rx = PhyRx::with_numerology(num, 1500.0);
                rx.set_debug(true);
                let mut out = Vec::new();
                rx.process(&a, &mut out);
                let mut e = vec![0f64; 64];
                let mut n = vec![0f64; 64];
                for d in rx.debug_cells() {
                    let z = d.y * d.h.conj() * (1.0 / d.h.norm_sqr());
                    let best = pts.iter().map(|p| (z - *p).norm_sqr()).fold(f32::INFINITY, f32::min);
                    // Weight by |h|²: error power against the mean cell.
                    e[d.c] += (best * d.h.norm_sqr()) as f64;
                    n[d.c] += d.h.norm_sqr() as f64;
                }
                let ber = out
                    .first()
                    .map_or(-1.0, |b| bit_errors(&b.llrs, &bits) as f32 / nbits as f32);
                (e, n, ber, rx.debug_model())
            });
            let mut e = vec![0f64; 64];
            let mut n = vec![0f64; 64];
            for r in &res {
                for i in 0..64 {
                    e[i] += r.0[i];
                    n[i] += r.1[i];
                }
            }
            let tot = -10.0 * (e.iter().sum::<f64>() / n.iter().sum::<f64>()).log10();
            let per: Vec<String> = (0..64)
                .filter(|&i| n[i] > 0.0)
                .map(|i| format!("{:.0}", -10.0 * (e[i] / n[i]).log10()))
                .collect();
            let bers: Vec<String> = res.iter().map(|r| format!("{:.0e}", r.2)).collect();
            let models: Vec<String> = res
                .iter()
                .map(|r| format!("{}/{}/{:.1}", r.3.0, r.3.1, r.3.2))
                .collect();
            eprintln!(
                "{} {num:?}: {tot:.1} dB | per carrier {} | BER {} | models {}",
                spec.name,
                per.join(" "),
                bers.join(" "),
                models.join(" ")
            );
        }
    }
}

/// One codeword of `rate` (`fec::K` info bits) in one Slice burst at
/// `center`, constellation `c` with the link's clip, through `profile` at
/// `snr` (2.4 kHz, against the burst's own power): (header right,
/// codeword decoded).
fn codeword_run(c: Constellation, rate: Rate, profile: Profile, snr: f32, seed: u64, center: f32) -> (bool, bool) {
    let info_bits = random_bits(fec::K, seed);
    let coded = fec::encode(rate, &info_bits);
    let user = (seed % 65521) as u16;
    let o = TxOptions::for_burst(c, Pilots::Robust);
    let burst = modulate_full(SLICE, c, user, 0, &coded, center, &o);
    let p = power(&burst);
    let a = impair(&pad(&burst, 0.25, 0.3), profile, seed, Some(snr), p, 0.0);
    let (got, _) = rx_slice(center, &a);
    let Some(b) = got
        .iter()
        .find(|b| b.user == user && !b.header_only && b.constellation == c)
    else {
        return (false, false);
    };
    let ok = fec::decode(rate, &b.llrs[..rate.n()]).is_some_and(|d| d == info_bits);
    (true, ok)
}

/// AWGN: QPSK codewords at rate 1/2 and 1/4 decode 2 dB above the 90%
/// points of `slice_table`, 6 bursts each.
#[test]
fn slice_awgn_qpsk_codewords() {
    for (rate, snr) in [(Rate::R1_4, -4.0f32), (Rate::R1_2, 0.0)] {
        let jobs: Vec<u64> = (0..6).collect();
        let res = par_map(&jobs, |&k| {
            codeword_run(Constellation::Qpsk, rate, Profile::Awgn, snr, 4000 + k, 1500.0)
        });
        let ok = res.iter().filter(|r| r.1).count();
        assert!(ok >= 5, "{rate:?} at {snr} dB: {ok}/6");
    }
}

/// A QPSK rate-1/2 codeword burst at `center`, clipped at `clip`:
/// (audio, info bits, user).
fn slice_codeword(center: f32, seed: u64, clip: Option<f32>) -> (Vec<f32>, Vec<u8>, u16) {
    let info = random_bits(fec::K, seed);
    let coded = fec::encode(Rate::R1_2, &info);
    let user = 0x4000 + (seed % 4096) as u16;
    let o = TxOptions {
        clip_db: clip,
        pilots: Pilots::Robust,
        ..TxOptions::default()
    };
    let burst = modulate_full(SLICE, Constellation::Qpsk, user, 0, &coded, center, &o);
    (burst, info, user)
}

/// Adds `x` into `out` starting at `at` seconds.
fn mix_in(out: &mut Vec<f32>, x: &[f32], at: f32, gain: f32) {
    let s = (at * AUDIO_RATE) as usize;
    if out.len() < s + x.len() {
        out.resize(s + x.len(), 0.0);
    }
    for (o, v) in out[s..].iter_mut().zip(x) {
        *o += gain * v;
    }
}

/// The codeword of `user` among `got`, decoded: (found, decoded).
fn find_decode(got: &[RxBurst], user: u16, info: &[u8]) -> (bool, bool) {
    match got.iter().find(|b| b.user == user) {
        Some(b) => {
            let ok = fec::decode(Rate::R1_2, &b.llrs[..Rate::R1_2.n()]).is_some_and(|d| d == info);
            (true, ok)
        }
        None => (false, false),
    }
}

/// Three slices at full power side by side (750, 1500, 2250 Hz),
/// overlapping in time, different data: each slice's receiver decodes its
/// own codeword (QPSK 1/2 at +2 dB against one slice's power) and nothing
/// else.
#[test]
fn slice_adjacent_slices() {
    // Without noise and unclipped: the two full-power neighbours' leakage
    // into the middle slice, as its reported SNR.
    {
        let (w, _, wu) = slice_codeword(1500.0, 11, None);
        let (lo, _, _) = slice_codeword(750.0, 12, None);
        let (hi, _, _) = slice_codeword(2250.0, 13, None);
        let mut all = Vec::new();
        mix_in(&mut all, &w, 0.5, 1.0);
        mix_in(&mut all, &[0.0; 48000], 4.5, 1.0);
        mix_in(&mut all, &lo, 0.8, 1.0);
        mix_in(&mut all, &hi, 0.1, 1.0);
        let (g_all, _) = rx_slice(1500.0, &all);
        let s_all = g_all.iter().find(|b| b.user == wu).unwrap().snr_db;
        assert!(s_all > 30.0, "neighbours leak: {s_all:.1} dB");
    }
    let mut ok_all = 0;
    let runs = 4u64;
    for r in 0..runs {
        let mut mix = Vec::new();
        let mut want = Vec::new();
        // The neighbours start 0.3 s after and 0.6 s before the middle
        // burst (3.5 s long), so all three overlap for most of it.
        for (k, (&center, at)) in CENTERS.iter().zip([1.1f32, 0.8, 0.2]).enumerate() {
            let (b, info, user) = slice_codeword(center, 100 * r + k as u64, Some(4.5));
            mix_in(&mut mix, &b, at, 1.0);
            want.push((center, info, user));
        }
        mix.extend(std::iter::repeat_n(0.0, 24000));
        // Noise against one slice burst's power.
        let (b, _, _) = slice_codeword(1500.0, 100 * r + 1, Some(4.5));
        let sigma = noise_sigma(power(&b), 2.0, 2400.0, AUDIO_RATE);
        add_awgn(&mut mix, sigma, &mut NoiseGen::new(77 + r));
        for (center, info, user) in &want {
            let (got, _) = rx_slice(*center, &mix);
            let (found, ok) = find_decode(&got, *user, info);
            assert_eq!(got.len(), 1, "{center} Hz receiver delivered {}", got.len());
            assert!(found, "{center} Hz: own burst not found");
            ok_all += ok as usize;
        }
    }
    assert!(ok_all >= 3 * runs as usize - 1, "{ok_all} of {} decoded", 3 * runs);
}

/// Steady carriers inside both neighbouring slices (off the 50 Hz grid),
/// each 20 dB stronger than the wanted middle slice: its codeword still
/// decodes as it does without them (QPSK 1/2 at +2 dB), and the carriers
/// trigger no bursts.
#[test]
fn slice_strong_neighbour_carriers() {
    let tones = [763.0f64, 1037.0, 1963.0, 2237.0];
    let add_tones = |x: &mut [f32], p: f32| {
        // Each carrier 20 dB above the burst: amplitude √(2·100·p).
        let amp = (2.0 * 100.0 * p as f64).sqrt();
        for (i, v) in x.iter_mut().enumerate() {
            let t = i as f64 / AUDIO_RATE as f64;
            for (k, &f) in tones.iter().enumerate() {
                *v += (amp * (core::f64::consts::TAU * f * t + k as f64).sin()) as f32;
            }
        }
    };
    // Without noise and unclipped: what reaches the middle slice.
    {
        let (b, _, user) = slice_codeword(1500.0, 299, None);
        let mut a = pad(&b, 0.4, 0.5);
        add_tones(&mut a, power(&b));
        let (g1, _) = rx_slice(1500.0, &a);
        let s1 = g1.iter().find(|b| b.user == user).unwrap().snr_db;
        assert!(s1 > 30.0);
    }
    let runs = 4u64;
    let mut ok_with = 0;
    let mut ok_without = 0;
    for r in 0..runs {
        let (b, info, user) = slice_codeword(1500.0, 300 + r, Some(4.5));
        let p = power(&b);
        let mut clean = Vec::new();
        mix_in(&mut clean, &b, 0.4, 1.0);
        clean.extend(std::iter::repeat_n(0.0, 24000));
        let sigma = noise_sigma(p, 2.0, 2400.0, AUDIO_RATE);
        add_awgn(&mut clean, sigma, &mut NoiseGen::new(900 + r));
        let mut with = clean.clone();
        add_tones(&mut with, p);
        let (g0, _) = rx_slice(1500.0, &clean);
        let (g1, _) = rx_slice(1500.0, &with);
        let (_, ok0) = find_decode(&g0, user, &info);
        let (_, ok1) = find_decode(&g1, user, &info);
        assert_eq!(g1.len(), 1);
        ok_with += ok1 as usize;
        ok_without += ok0 as usize;
    }
    assert!(ok_with >= ok_without && ok_with >= runs as usize - 1);
}

/// Table for `docs/phy.md` (Slice numerology): for each level the
/// lowest SNR on a 2 dB grid (2.4 kHz, against the burst's own power) at
/// which ≥ 90% of bursts deliver the right header, and at which ≥ 90%
/// decode their codeword; `PHY_RUNS` bursts per point (default 20), one
/// codeword per burst, link clip, centre 1500 Hz. `ION_THREADS`
/// threads (default 2).
///
/// cargo test -p ion --release --test phy_slice -- --ignored --nocapture slice_table
#[test]
#[ignore]
fn slice_table() {
    let runs: u64 = std::env::var("PHY_RUNS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(20);
    let levels = [
        (Constellation::Qpsk, Rate::R1_4, "QPSK 1/4"),
        (Constellation::Qpsk, Rate::R1_2, "QPSK 1/2"),
        (Constellation::Psk8, Rate::R2_3, "8PSK 2/3"),
        (Constellation::Qam16, Rate::R3_4, "16QAM 3/4"),
    ];
    let mut rows = Vec::new();
    for (profile, start) in [(Profile::Awgn, -16i32), (Profile::CcirPoor, -14)] {
        for &(c, rate, name) in &levels {
            let (mut hdr90, mut data90) = (None, None);
            let mut curve = Vec::new();
            let mut snr = start;
            // Up the 2 dB grid until both reach 90% twice in a row.
            let mut both = 0;
            while snr <= 36 && both < 2 {
                let jobs: Vec<u64> = (0..runs).collect();
                let res = par_map(&jobs, |&k| {
                    codeword_run(
                        c,
                        rate,
                        profile,
                        snr as f32,
                        7 + 1000 * k + snr.unsigned_abs() as u64,
                        1500.0,
                    )
                });
                let h = res.iter().filter(|r| r.0).count() as f32 / runs as f32;
                let d = res.iter().filter(|r| r.1).count() as f32 / runs as f32;
                curve.push(format!("{snr:+}: {h:.2}/{d:.2}"));
                if h >= 0.9 && hdr90.is_none() {
                    hdr90 = Some(snr);
                }
                if d >= 0.9 && data90.is_none() {
                    data90 = Some(snr);
                }
                both = if h >= 0.9 && d >= 0.9 { both + 1 } else { 0 };
                // Skip the hopeless low end quickly.
                snr += if h < 0.05 && d < 0.05 && snr < 10 { 4 } else { 2 };
            }
            let show = |v: Option<i32>| v.map_or("> 36".to_string(), |s| format!("{s:+}"));
            eprintln!(
                "{:?} {name}: header ≥ 90% at {} dB, codeword ≥ 90% at {} dB; header/codeword: {}",
                profile,
                show(hdr90),
                show(data90),
                curve.join(", ")
            );
            rows.push((profile, name, show(hdr90), show(data90)));
        }
    }
    println!("| Level | AWGN header | AWGN codeword | CCIR poor header | CCIR poor codeword |");
    println!("|---|---:|---:|---:|---:|");
    for &(_, name, _) in levels.iter().map(|l| (l.0, l.2, l.1)).collect::<Vec<_>>().iter() {
        let a = rows.iter().find(|r| r.0 == Profile::Awgn && r.1 == name).unwrap();
        let p = rows.iter().find(|r| r.0 == Profile::CcirPoor && r.1 == name).unwrap();
        println!("| {name} | {} | {} | {} | {} |", a.2, a.3, p.2, p.3);
    }
}

/// 60 s of white noise into a slice receiver (one stream, 10 ms chunks):
/// detector triggers, preamble checks passed, bursts.
fn noise_run(secs: usize, seed: u64, center: f32) -> (ion::phy::RxStats, usize, usize, f32) {
    let mut rx = PhyRx::with_numerology(SLICE, center);
    rx.set_debug(true);
    let mut out = Vec::new();
    let mut ng = NoiseGen::new(seed);
    let mut buf = vec![0f32; 480];
    let mut busy = 0;
    for _ in 0..secs * 100 {
        for v in buf.iter_mut() {
            *v = 0.1 * ng.gaussian();
        }
        rx.process(&buf, &mut out);
        busy += rx.busy() as usize;
    }
    let worst = rx.debug_rho().iter().fold(0f32, |m, x| m.max(*x));
    (rx.stats(), out.len(), busy, worst)
}

/// False detections on 60 s of noise: the whole-preamble check rejects
/// (nearly) all detector triggers, and no burst comes out.
#[test]
fn slice_noise_only() {
    let mut total_syncs = 0;
    for (k, &center) in CENTERS.iter().enumerate() {
        let (st, bursts, busy, _) = noise_run(60, 99 + k as u64, center);
        assert_eq!(bursts, 0);
        assert!(st.syncs <= 1, "{} false syncs", st.syncs);
        assert!(busy <= 100, "busy in {busy} of 6000 chunks");
        total_syncs += st.syncs;
    }
    assert!(total_syncs <= 1);
    // Silence and a hard step must not trigger either.
    let mut a = vec![0f32; 48000];
    a.extend(std::iter::repeat_n(0.3, 48000));
    a.extend(std::iter::repeat_n(0.0, 48000));
    let (got, _) = rx_slice(1500.0, &a);
    assert!(got.is_empty());
}

/// Slice bursts into the full-width receivers and full-width bursts into
/// the slice receivers (20 dB, AWGN and CCIR poor): preamble checks
/// passed (header attempts) and bursts delivered by the wrong receiver.
#[test]
#[ignore]
fn diag_slice_cross() {
    let mut a = Vec::new();
    let mut b = Vec::new();
    for (k, profile) in [Profile::Awgn, Profile::CcirPoor].into_iter().enumerate() {
        for s in 0..6u64 {
            let bits = random_bits(1000, s);
            let c = Constellation::ALL[s as usize % 4];
            let o = TxOptions::default();
            let center = CENTERS[s as usize % 3];
            let x = modulate_full(SLICE, c, 1, 0, &bits, center, &o);
            a.extend(pad(
                &impair(&x, profile, 10 * k as u64 + s, Some(20.0), power(&x), 0.0),
                0.3,
                0.0,
            ));
            for num in Numerology::ALL {
                let y = modulate_full(num, c, 1, 0, &bits, num.center_hz(), &o);
                b.extend(pad(
                    &impair(&y, profile, 10 * k as u64 + s, Some(20.0), power(&y), 0.0),
                    0.3,
                    0.0,
                ));
            }
        }
    }
    for num in Numerology::ALL {
        let mut rx = PhyRx::with_numerology(num, num.center_hz());
        rx.set_debug(true);
        let mut out = Vec::new();
        rx.process(&a, &mut out);
        let worst = rx.debug_rho().iter().fold(0f32, |m, x| m.max(*x));
        eprintln!(
            "12 Slice bursts → {num:?} receiver: {:?}, {} bursts, worst rho {worst:.3}",
            rx.stats(),
            out.len()
        );
    }
    for &center in &CENTERS {
        let mut rx = PhyRx::with_numerology(SLICE, center);
        rx.set_debug(true);
        let mut out = Vec::new();
        rx.process(&b, &mut out);
        let worst = rx.debug_rho().iter().fold(0f32, |m, x| m.max(*x));
        eprintln!(
            "48 full-width bursts → Slice {center} receiver: {:?}, {} bursts, worst rho {worst:.3}",
            rx.stats(),
            out.len()
        );
    }
}

/// 10 minutes of noise (20 streams of 30 s).
#[test]
#[ignore]
fn slice_noise_long() {
    let jobs: Vec<u64> = (0..20).collect();
    let res = par_map(&jobs, |&k| noise_run(30, 1000 + k, 1500.0));
    let cand: u64 = res.iter().map(|r| r.0.candidates).sum();
    let syncs: u64 = res.iter().map(|r| r.0.syncs).sum();
    let bursts: usize = res.iter().map(|r| r.1).sum();
    let worst = res.iter().map(|r| r.3).fold(0f32, f32::max);
    println!(
        "Slice, 600 s of noise: {cand} detector triggers, {syncs} syncs (header attempts), {bursts} bursts, worst preamble correlation {worst:.3}"
    );
    assert_eq!(bursts, 0);
}

/// Preamble statistics for the thresholds: whole-preamble correlation of
/// noise-only candidates (120 s) against real bursts at low SNR.
#[test]
#[ignore]
fn diag_slice_rho() {
    let jobs: Vec<u64> = (0..4).collect();
    let res = par_map(&jobs, |&k| {
        let mut rx = PhyRx::with_numerology(SLICE, 1500.0);
        rx.set_debug(true);
        let mut out = Vec::new();
        let mut ng = NoiseGen::new(1234 + k);
        let mut buf = vec![0f32; 4800];
        for _ in 0..300 {
            for v in buf.iter_mut() {
                *v = 0.1 * ng.gaussian();
            }
            rx.process(&buf, &mut out);
        }
        (rx.stats(), rx.debug_rho().to_vec(), out.len())
    });
    let mut r: Vec<f32> = res.iter().flat_map(|x| x.1.iter().copied()).collect();
    r.sort_by(|a, b| b.total_cmp(a));
    let cand: u64 = res.iter().map(|x| x.0.candidates).sum();
    let syncs: u64 = res.iter().map(|x| x.0.syncs).sum();
    eprintln!(
        "noise 120 s: {cand} candidates, {} past the B pre-filter, {syncs} syncs, {} bursts; top rho {:?}",
        r.len(),
        res.iter().map(|x| x.2).sum::<usize>(),
        &r[..r.len().min(12)]
    );
    let c = Constellation::Qpsk;
    for profile in [Profile::Awgn, Profile::CcirPoor] {
        for snr in [-16.0f32, -14.0, -12.0, -10.0, -8.0, -6.0] {
            let jobs: Vec<u64> = (0..40).collect();
            let res = par_map(&jobs, |&seed| {
                let bits = random_bits(20 * 40, seed);
                let burst = modulate_full(SLICE, c, 1, 0, &bits, 1500.0, &TxOptions::default());
                let p = power(&burst);
                let a = impair(&pad(&burst, 0.25, 0.25), profile, 900 + seed, Some(snr), p, 0.0);
                let mut rx = PhyRx::with_numerology(SLICE, 1500.0);
                rx.set_debug(true);
                let mut out = Vec::new();
                rx.process(&a, &mut out);
                (
                    rx.debug_rho().iter().fold(0f32, |m, x| m.max(*x)),
                    rx.stats().candidates > 0,
                    rx.stats().syncs > 0,
                    out.len() == 1,
                )
            });
            let mut v: Vec<f32> = res.iter().map(|x| x.0).collect();
            v.sort_by(|a, b| a.total_cmp(b));
            eprintln!(
                "{profile:?} {snr} dB: rho min {:.3}, 10% {:.3}, median {:.3}; detected {}, synced {}, header ok {} / 40",
                v[0],
                v[4],
                v[20],
                res.iter().filter(|x| x.1).count(),
                res.iter().filter(|x| x.2).count(),
                res.iter().filter(|x| x.3).count()
            );
        }
    }
}

/// Per-burst uncoded BER of 16QAM on CCIR poor at high SNR (where the
/// codeword table shows a loss floor), real header, with and without the
/// clip; and the same for Normal for comparison.
#[test]
#[ignore]
fn diag_slice_floor() {
    let snr: f32 = std::env::var("SLICE_SNR")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(30.0);
    let c = Constellation::Qam16;
    for num in [Numerology::Normal, SLICE] {
        for clip in [None, Some(5.0f32)] {
            let jobs: Vec<u64> = (0..20).collect();
            let res = par_map(&jobs, |&k| {
                let seed = 7 + 1000 * k + 30;
                let nbits = capacity_bits_with(num, Pilots::Robust, c, 48);
                let bits = random_bits(nbits, seed);
                let o = TxOptions {
                    clip_db: clip,
                    pilots: Pilots::Robust,
                    ..TxOptions::default()
                };
                let burst = modulate_full(num, c, 1, 0, &bits, 1500.0, &o);
                let p = power(&burst);
                let a = impair(&pad(&burst, 0.25, 0.3), Profile::CcirPoor, seed, Some(snr), p, 0.0);
                let mut rx = PhyRx::with_numerology(num, 1500.0);
                rx.set_debug(true);
                let mut out = Vec::new();
                rx.process(&a, &mut out);
                match out.first() {
                    Some(b) => {
                        let mut ber = bit_errors(&b.llrs, &bits) as f32 / nbits as f32;
                        if b.constellation != c {
                            ber = -1.0;
                        }
                        (ber, b.snr_db, rx.debug_model())
                    }
                    None => (-2.0, 0.0, (0.0, 0.0, 0.0)),
                }
            });
            let line: Vec<String> = res.iter().map(|r| format!("{:.0e}/{:.0}", r.0, r.1)).collect();
            eprintln!("{num:?} clip {clip:?} {snr} dB: {}", line.join(" "));
            let models: Vec<String> = res
                .iter()
                .map(|r| format!("{}/{}/{:.1}", r.2.0, r.2.1, r.2.2))
                .collect();
            eprintln!("   models: {}", models.join(" "));
        }
    }
}

/// Audio crest factor (peak over RMS of the 48 kHz audio, dB; the
/// envelope PAPR is ~3 dB less) by clip level: 20 bursts of ~1 s.
#[test]
#[ignore]
fn diag_slice_papr() {
    for num in [Numerology::Normal, SLICE] {
        for c in [Constellation::Qpsk, Constellation::Qam16] {
            let mut line = format!("{num:?} {c:?}:");
            for clip in [None, Some(8.0f32), Some(7.0), Some(6.0), Some(5.0), Some(4.5)] {
                let mut crest = 0f64;
                for seed in 0..20u64 {
                    let nbits = capacity_bits_with(num, Pilots::Robust, c, 42);
                    let bits = random_bits(nbits, 900 + seed);
                    let o = TxOptions {
                        clip_db: clip,
                        pilots: Pilots::Robust,
                        ..TxOptions::default()
                    };
                    let a = modulate_full(num, c, 1, 0, &bits, 1500.0, &o);
                    let peak = a.iter().fold(0f32, |m, v| m.max(v.abs()));
                    crest += 10.0 * (peak as f64 * peak as f64 / power(&a) as f64).log10() / 20.0;
                }
                line += &format!(" {clip:?} {crest:.1} dB |");
            }
            eprintln!("{line}");
        }
    }
}

#[test]
#[ignore]
fn diag_slice_clip() {
    for num in [Numerology::Normal, SLICE] {
        for c in Constellation::ALL {
            let mut line = format!("{num:?} {c:?}:");
            for clip in [None, Some(8.0f32), Some(6.0), Some(5.0), Some(4.5)] {
                let (mut e, mut n, mut snr) = (0, 0, 0.0);
                for seed in 0..4u64 {
                    let bits = random_bits(6000, 500 + seed);
                    let o = TxOptions {
                        clip_db: clip,
                        pilots: Pilots::Robust,
                        ..TxOptions::default()
                    };
                    let burst = modulate_full(num, c, 1, 0, &bits, 1500.0, &o);
                    let mut rx = PhyRx::with_numerology(num, 1500.0);
                    let mut out = Vec::new();
                    rx.process(&pad(&burst, 0.3, 0.3), &mut out);
                    if let Some(b) = out.first() {
                        e += bit_errors(&b.llrs, &bits);
                        n += bits.len();
                        snr += b.snr_db / 4.0;
                    }
                }
                line += &format!(" {clip:?}: {:.1e} {snr:.1} dB |", e as f32 / n.max(1) as f32);
            }
            eprintln!("{line}");
        }
    }
}
