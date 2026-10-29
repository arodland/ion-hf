//! Receiver lab: codeword loss of the ladder's OFDM levels on IONOS SIM
//! channels for receiver variants that all decode the same received audio
//! (paired, so small differences between them show).
//!
//! ```text
//! cargo test -p ion --release --test rx_lab -- --ignored --nocapture --test-threads 1
//! ```
//!
//! `RX_LEVEL` (ladder index, default 10: Normal QPSK 1/2), `RX_MODES`
//! (default `WGN,MPG,MPP,MPD`), `RX_SNRS` (IONOS S:N in 3 kHz against a
//! sine at the burst's peak; default a range per channel), `RX_BURSTS`
//! per point (default 40), `RX_SECS` (burst length as the link sizes it,
//! default 8), `RX_VARIANTS`, `ION_THREADS` (default 2).

use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};

use ion::fec::{self, Rate};
use ion::level::{LADDER, Wave};
use ion::link::audio;
use ion::phy::{self, Constellation, Numerology, PhyRx, Pilots, RxStats};
use ion_dsp::ionos::{Ionos, Mode, Reference, reference_power};

fn env<T: std::str::FromStr>(k: &str) -> Option<T> {
    std::env::var(k).ok().and_then(|v| v.parse().ok())
}

fn list<T: std::str::FromStr>(k: &str, default: &str) -> Vec<T> {
    std::env::var(k)
        .unwrap_or_else(|_| default.into())
        .split(',')
        .filter_map(|x| x.trim().parse().ok())
        .collect()
}

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

/// Lead-in and tail of silence around a burst, samples.
const LEAD: usize = 4800;

fn modes() -> Vec<Mode> {
    std::env::var("RX_MODES")
        .unwrap_or_else(|_| "WGN,MPG,MPP,MPD".into())
        .split(',')
        .map(|m| {
            *Mode::ALL
                .iter()
                .find(|x| x.name().eq_ignore_ascii_case(m.trim()))
                .expect("mode")
        })
        .collect()
}

/// Frames in a burst of about `secs`, by the link's rule.
fn frames_for(num: Numerology, p: Pilots, c: Constellation, r: Rate, secs: f64) -> usize {
    let dur = |f: usize| phy::burst_samples_with(num, p, c, f * r.n()) as f64 / 48_000.0;
    let one = dur(1);
    let frame = dur(2) - one;
    (((secs - (one - frame)) / frame.max(1e-3)).floor() as usize).max(1)
}

/// One data burst of level `l` as the link puts it on the air, through
/// IONOS (2 paths) at `snr`: the received audio, the frames' information
/// bits, and the header's `user`.
struct Burst {
    audio: Vec<f32>,
    infos: Vec<Vec<u8>>,
    user: u16,
    num: Numerology,
    center: f32,
}

fn burst(l: usize, mode: Mode, snr: f32, seed: u64, secs: f64) -> Burst {
    let lv = LADDER[l];
    let Wave::Ofdm(g) = lv.wave else {
        panic!("level {l} is not OFDM")
    };
    let num = audio::numerology(g);
    let (c, r, pilots) = (lv.constellation, lv.rate, lv.pilots);
    let center = num.center_hz();
    let nframes = frames_for(num, pilots, c, r, secs);
    let infos: Vec<Vec<u8>> = (0..nframes)
        .map(|f| random_bits(fec::K, seed * 1000 + f as u64))
        .collect();
    let coded: Vec<u8> = infos.iter().flat_map(|i| fec::encode(r, i)).collect();
    let user = 0x2000 | (seed % 0x1000) as u16;
    let mut tx = phy::modulate_full(num, c, user, 0, &coded, center, &audio::tx_options(l));
    // As `audio::transmit_with` puts it on the air.
    let gain = (audio::ofdm_gain(audio::OFDM_AIR_GAIN_DB) * 10f32.powf(audio::level_trim_db(l) / 20.0))
        .min(10f32.powf(audio::OFDM_AIR_GAIN_MAX_DB / 20.0));
    for x in &mut tx {
        *x = (*x * gain).clamp(-1.0, 1.0);
    }
    let mut ch = Ionos::new(mode, 2, 3000, seed);
    let mut input = vec![0.0; LEAD];
    input.extend_from_slice(&tx);
    input.resize(input.len() + ch.latency() + LEAD, 0.0);
    let mut out = vec![0.0; input.len()];
    ch.process(&input, Some((reference_power(&tx, Reference::Peak), snr)), &mut out);
    Burst {
        audio: out,
        infos,
        user,
        num,
        center,
    }
}

/// Frames of `b` decoded from `llrs` (by the burst's own code).
fn decoded(r: Rate, llrs: &[f32], b: &Burst) -> usize {
    b.infos
        .iter()
        .zip(llrs.chunks_exact(r.n()))
        .filter(|(info, w)| fec::decode(r, w).as_deref() == Some(&info[..]))
        .count()
}

/// S:N where the loss `curve` (S:N, loss) first falls to `p`,
/// interpolated in log loss.
fn crossing(curve: &[(f32, f32)], p: f32) -> String {
    let lg = |x: f32| x.max(1e-4).ln();
    if curve.first().is_some_and(|c| c.1 <= p) {
        return format!("<{}", curve[0].0);
    }
    for w in curve.windows(2) {
        let ((s0, l0), (s1, l1)) = (w[0], w[1]);
        if l0 > p && l1 <= p {
            let t = (lg(l0) - lg(p)) / (lg(l0) - lg(l1));
            return format!("{:.1}", s0 + t * (s1 - s0));
        }
    }
    format!(">{}", curve.last().map_or(0.0, |c| c.0))
}

/// Per-channel default S:N range around the Normal QPSK 1/2 crossings.
fn default_snrs(m: Mode) -> &'static str {
    match m {
        Mode::Wgn => "2,3,4,5,6,7,8",
        Mode::Mpg => "5,6,7,8,9,10,11,12,13,14",
        Mode::Mpm | Mode::Mpp => "4,5,6,7,8,9,10,11,12",
        Mode::Mpd => "8,9,10,11,12,13,14,15,16,18,20",
    }
}

/// Runs `jobs` over `ION_THREADS` threads.
fn parallel<J: Sync, R: Send + Clone>(jobs: &[J], empty: R, f: impl Fn(&J) -> R + Sync) -> Vec<R> {
    let threads: usize = env("ION_THREADS").unwrap_or(2);
    let out = Mutex::new(vec![empty; jobs.len()]);
    let next = AtomicUsize::new(0);
    std::thread::scope(|sc| {
        for _ in 0..threads {
            sc.spawn(|| {
                loop {
                    let i = next.fetch_add(1, Ordering::Relaxed);
                    let Some(j) = jobs.get(i) else { break };
                    let r = f(j);
                    out.lock().unwrap()[i] = r;
                }
            });
        }
    });
    out.into_inner().unwrap()
}

/// FFT window placement: today's rule (`legacy`: the energy window's
/// plateau, no re-timing) against the receiver's default (`fix`: Normal
/// guard, placement for echoes past the energy window and in-burst
/// re-timing); `noretime` and `norule` take one part out of `fix`.
#[test]
#[ignore]
fn timing_fer() {
    let l: usize = env("RX_LEVEL").unwrap_or(10);
    let secs: f64 = env("RX_SECS").unwrap_or(8.0);
    let bursts: u64 = env("RX_BURSTS").unwrap_or(40);
    let variants: Vec<String> = list("RX_VARIANTS", "legacy,fix");
    let r = LADDER[l].rate;
    for m in modes() {
        let snrs: Vec<f32> = list("RX_SNRS", default_snrs(m));
        let jobs: Vec<(usize, f32, u64)> = snrs
            .iter()
            .enumerate()
            .flat_map(|(si, &s)| (0..bursts).map(move |b| (si, s, b)))
            .collect();
        // Per variant: frames decoded, header found, receiver counters.
        let res = parallel(&jobs, Vec::new(), |&(_, s, b)| {
            let seed = 11 + b + 1000 * l as u64 + 100_000 * m as u64;
            let bu = burst(l, m, s, seed, secs);
            variants
                .iter()
                .map(|v| {
                    let mut rx = PhyRx::with_numerology(bu.num, bu.center);
                    match v.as_str() {
                        "legacy" => rx.set_legacy_timing(true),
                        "fix" => {}
                        "noretime" => rx.set_retiming(false),
                        "norule" => {
                            rx.set_legacy_timing(true);
                            rx.set_retiming(true);
                        }
                        _ => panic!("variant {v}"),
                    }
                    let mut got = Vec::new();
                    rx.process(&bu.audio, &mut got);
                    let n = match got.iter().find(|x| x.user == bu.user && !x.header_only) {
                        Some(x) => decoded(r, &x.llrs, &bu),
                        None => 0,
                    };
                    (n, bu.infos.len(), rx.stats())
                })
                .collect::<Vec<(usize, usize, RxStats)>>()
        });
        println!("\n### L{l} {}, {bursts} bursts per point", m.name());
        println!(
            "| S:N | {} |",
            variants
                .iter()
                .map(|v| format!("{v} loss | placed | steps"))
                .collect::<Vec<_>>()
                .join(" | ")
        );
        let mut curves = vec![Vec::new(); variants.len()];
        for (si, &s) in snrs.iter().enumerate() {
            let mut row = format!("| {s} |");
            for (vi, curve) in curves.iter_mut().enumerate() {
                let (mut ok, mut all, mut placed, mut steps) = (0, 0, 0, 0);
                for (j, rr) in jobs.iter().zip(&res) {
                    if j.0 == si {
                        ok += rr[vi].0;
                        all += rr[vi].1;
                        placed += rr[vi].2.placed;
                        steps += rr[vi].2.retime_steps;
                    }
                }
                let loss = 1.0 - ok as f32 / all.max(1) as f32;
                curve.push((s, loss));
                row += &format!(" {loss:.3} | {placed} | {steps} |");
            }
            println!("{row}");
        }
        for (v, c) in variants.iter().zip(&curves) {
            println!(
                "{} {v}: 10% {} 30% {} 50% {}",
                m.name(),
                crossing(c, 0.1),
                crossing(c, 0.3),
                crossing(c, 0.5)
            );
        }
    }
}
