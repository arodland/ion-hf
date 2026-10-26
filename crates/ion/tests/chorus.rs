// Chorus check-ins: many stations at once, each in its own sub-band
// (`ion::phy_chorus`).
//
// Every station has its own fading channel, frequency offset (±15 Hz),
// clock error (±20 ms) and level; NCS knows the slot to ±50 ms. The table
// comes from the ignored generator:
//
// cargo test -p ion --release --test chorus -- --ignored --nocapture

use ion::phy_chorus::{ChorusCheckin, ChorusFmt, modulate, receive, receive_subband};
use ion::phy_mfsk::{MfskMode, burst_samples};
use ion_dsp::channel::{Channel, Profile, add_awgn};
use ion_dsp::dsp::{AUDIO_RATE, NoiseGen, noise_sigma};

const CENTER: f32 = 1500.0;
const CH_DELAY: u64 = 200;
const LEAD_S: f32 = 0.5;
const TOL_S: f32 = 0.050;
const ERR_S: f32 = 0.020;
const OFFSET_HZ: f32 = 15.0;

const CALLS: [&str; 9] = [
    "N0JLF", "AG5B", "W1AW", "K4XYZ", "N5ABC", "KD9QRS", "W7DEF", "KB0GHI", "AE4JKL",
];

fn unit(seed: u64) -> f32 {
    let mut z = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    ((z ^ (z >> 31)) >> 40) as f32 / (1u64 << 24) as f32
}

fn sym(seed: u64) -> f32 {
    2.0 * unit(seed) - 1.0
}

fn checkin(i: usize, seed: u64) -> ChorusCheckin {
    ChorusCheckin {
        call: CALLS[i % CALLS.len()].to_string(),
        snr_ncs_db: (seed % 40) as i8 - 20,
        caps: (seed % 16) as u8,
        version: 0x01,
    }
}

/// One slot on the air: station `i` (in sub-band `i`) at `snrs[i]` dB
/// (None: silent), each through its own channel, with its own frequency
/// offset and clock error, plus noise.
fn slot(fmt: ChorusFmt, snrs: &[Option<f32>], profile: Profile, seed: u64) -> Vec<f32> {
    let len = ((LEAD_S + 2.0 * ERR_S + 0.5) * AUDIO_RATE) as usize + fmt.burst_samples();
    let mut sum = vec![0f32; len];
    let p_ref = 0.5 * ion_dsp::TX_LEVEL * ion_dsp::TX_LEVEL;
    for (i, snr) in snrs.iter().enumerate() {
        let Some(snr) = snr else { continue };
        let s = seed.wrapping_mul(31).wrapping_add(i as u64);
        let burst = modulate(fmt, i, &checkin(i, s), CENTER);
        let start = ((LEAD_S + ERR_S * sym(s ^ 0x77)) * AUDIO_RATE) as usize;
        let mut a = vec![0f32; len];
        a[start..start + burst.len()].copy_from_slice(&burst);
        let mut ch = Channel::from_profile(profile, s).with_offset(OFFSET_HZ * sym(s ^ 0x99), 0.0);
        let mut out = vec![0f32; len];
        ch.process(&a, &mut out);
        // Level against the 0 dB reference.
        let g = 10f32.powf(snr / 20.0);
        for (o, x) in sum.iter_mut().zip(&out) {
            *o += g * x;
        }
    }
    let sigma = noise_sigma(p_ref, 0.0, 2400.0, AUDIO_RATE);
    add_awgn(&mut sum, sigma, &mut NoiseGen::new(seed ^ 0xC0C0));
    sum
}

fn slot_start() -> u64 {
    (LEAD_S * AUDIO_RATE) as u64 + CH_DELAY
}

fn tol() -> u64 {
    (TOL_S * AUDIO_RATE) as u64
}

#[test]
fn full_chorus_at_good_snr() {
    for fmt in [ChorusFmt::T8, ChorusFmt::T4] {
        let n = fmt.subbands();
        let audio = slot(fmt, &vec![Some(3.0); n], Profile::Awgn, 11);
        let heard = receive(fmt, &audio, slot_start(), tol(), CENTER);
        assert_eq!(heard.len(), n, "{fmt:?}: {} of {n}", heard.len());
        for h in &heard {
            assert_eq!(
                h.checkin,
                checkin(h.sub, 11u64.wrapping_mul(31).wrapping_add(h.sub as u64))
            );
        }
    }
}

/// A weak station between two loud neighbours (6 dB louder) still gets
/// through at −6 dB.
#[test]
fn weak_station_between_loud_neighbours() {
    let fmt = ChorusFmt::T8;
    let mid = fmt.subbands() / 2;
    let mut ok = 0;
    for seed in 0..6u64 {
        let mut snrs = vec![Some(0.0); fmt.subbands()];
        snrs[mid] = Some(-6.0);
        let audio = slot(fmt, &snrs, Profile::Awgn, 300 + seed);
        if let Some(h) = receive_subband(fmt, &audio, mid, slot_start(), tol(), CENTER) {
            assert_eq!(h.checkin.call, CALLS[mid]);
            ok += 1;
        }
    }
    assert!(ok >= 5, "{ok} of 6");
}

#[test]
fn empty_slot_decodes_nothing() {
    for fmt in [ChorusFmt::T8, ChorusFmt::T4] {
        for seed in 0..10u64 {
            let audio = slot(fmt, &vec![None; fmt.subbands()], Profile::Awgn, 900 + seed);
            assert!(receive(fmt, &audio, slot_start(), tol(), CENTER).is_empty());
        }
    }
}

// ---------------------------------------------------------------------
// Table generator (ignored)
// ---------------------------------------------------------------------

/// Fraction of slots in which the middle station decodes, at `snr`, with
/// its neighbours absent (`None`) or `loud` dB louder than it.
fn cell(fmt: ChorusFmt, profile: Profile, snr: f32, loud: Option<f32>, n: usize) -> f32 {
    let threads = std::thread::available_parallelism().map_or(2, |p| p.get()).min(4);
    let mid = fmt.subbands() / 2;
    let ok: usize = std::thread::scope(|s| {
        let hs: Vec<_> = (0..threads)
            .map(|t| {
                s.spawn(move || {
                    (t..n)
                        .step_by(threads)
                        .filter(|&i| {
                            let seed =
                                50_000 * (snr as i64 + 100) as u64 + 1000 * loud.map_or(99, |l| l as u64) + i as u64;
                            let mut snrs = vec![loud.map(|l| snr + l); fmt.subbands()];
                            snrs[mid] = Some(snr);
                            let audio = slot(fmt, &snrs, profile, seed);
                            receive_subband(fmt, &audio, mid, slot_start(), tol(), CENTER)
                                .is_some_and(|h| h.checkin.call == CALLS[mid])
                        })
                        .count()
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).sum()
    });
    ok as f32 / n as f32
}

#[test]
#[ignore]
fn chorus_table() {
    let n: usize = std::env::var("CHORUS_N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(40);
    let r0 = burst_samples(MfskMode::R0, 1024) as f32 / AUDIO_RATE;
    println!("\nAir time of the check-ins alone (R0 control frame: {r0:.2} s each)\n");
    println!("| Stations | one after another (R0) | chorus 8 tones | chorus 4 tones |");
    println!("|---:|---:|---:|---:|");
    for k in [1usize, 5, 9, 20] {
        let ch = |f: ChorusFmt| k.div_ceil(f.subbands()) as f32 * f.burst_s();
        println!(
            "| {k} | {:.1} s | {:.1} s | {:.1} s |",
            k as f32 * r0,
            ch(ChorusFmt::T8),
            ch(ChorusFmt::T4)
        );
    }
    let grid = [
        (Profile::Awgn, "AWGN", -14..=-6),
        (Profile::CcirPoor, "CCIR poor", -12..=-4),
        (Profile::MidDisturbedNvis, "NVIS disturbed", -10..=-2),
    ];
    for fmt in [ChorusFmt::T8, ChorusFmt::T4] {
        println!(
            "\n{} tones × {} sub-bands, {:.2} s burst. Middle station decoded, {n} slots per cell; \
             neighbours absent, as loud, 6 dB and 12 dB louder\n",
            fmt.tones,
            fmt.subbands(),
            fmt.burst_s()
        );
        for (profile, name, snrs) in grid.clone() {
            println!("{name}\n");
            println!("| SNR dB | alone | neighbours 0 dB | +6 dB | +12 dB |");
            println!("|---:|---:|---:|---:|---:|");
            for snr in snrs.step_by(2) {
                let s = snr as f32;
                println!(
                    "| {snr} | {:.2} | {:.2} | {:.2} | {:.2} |",
                    cell(fmt, profile, s, None, n),
                    cell(fmt, profile, s, Some(0.0), n),
                    cell(fmt, profile, s, Some(6.0), n),
                    cell(fmt, profile, s, Some(12.0), n)
                );
            }
            println!();
        }
    }
}
