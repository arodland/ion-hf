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
use std::time::Instant;

use ion::fec::{self, Rate};
use ion::level::{EFF_THRESHOLDS_DB, LADDER, Wave};
use ion::link::audio;
use ion::link::harq::{HarqRx, Soft};
use ion::phy::{self, Constellation, DataMode, ExpectedHeader, Numerology, PhyRx, Pilots, REFINE_K, RxStats};
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
    /// Data symbols.
    nsym: usize,
    infos: Vec<Vec<u8>>,
    user: u16,
    num: Numerology,
    center: f32,
}

fn burst(l: usize, mode: Mode, snr: f32, seed: u64, secs: f64) -> Burst {
    burst_as(l, mode, snr, seed, secs, 0x2000 | (seed % 0x1000) as u16)
}

/// [`burst`] with the header's `user` given.
fn burst_as(l: usize, mode: Mode, snr: f32, seed: u64, secs: f64, user: u16) -> Burst {
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
        nsym: phy::data_symbols_with(num, pilots, c, coded.len()),
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

/// Mutual information per bit (bits) that LLRs `llrs` carry about the
/// coded bits `truth`, assuming they are calibrated: what a combining
/// receiver gets from them. Confident wrong LLRs cost more than they give.
fn llr_info(llrs: &[f32], truth: &[u8]) -> f64 {
    let n = llrs.len().min(truth.len());
    let loss: f64 = (0..n)
        .map(|i| {
            let s = if truth[i] == 0 { 1.0 } else { -1.0 };
            (1.0 + (-(s * llrs[i]) as f64).exp()).log2()
        })
        .sum();
    1.0 - loss / n.max(1) as f64
}

/// The iterative receiver (`HarqRx::receive_turbo`, the station's path)
/// against one plain decode, on the same bursts: codeword loss per
/// `RX_PASSES` (default `0,1,2,4`; 0 is the plain receiver), CPU per
/// pass, and the information the stored LLRs of codewords that failed
/// either way carry about their bits (HARQ combines them with the next
/// increment: the posteriors fed back must not make them worse).
#[test]
#[ignore]
fn turbo_fer() {
    let l: usize = env("RX_LEVEL").unwrap_or(10);
    let secs: f64 = env("RX_SECS").unwrap_or(8.0);
    let bursts: u64 = env("RX_BURSTS").unwrap_or(30);
    let passes: Vec<usize> = list("RX_PASSES", "0,1,2,4");
    let r = LADDER[l].rate;
    for m in modes() {
        let snrs: Vec<f32> = list("RX_SNRS", default_snrs(m));
        let jobs: Vec<(usize, f32, u64)> = snrs
            .iter()
            .enumerate()
            .flat_map(|(si, &s)| (0..bursts).map(move |b| (si, s, b)))
            .collect();
        // Per pass count: frames decoded, frames, per frame decoded or not
        // and its stored LLRs' information, refine seconds and passes run.
        // Wrong codewords (passing every check) last.
        type Out = Vec<(usize, usize, Vec<Option<f64>>, f64, usize, usize)>;
        let res: Vec<Out> = parallel(&jobs, Vec::new(), |&(_, s, b)| {
            let seed = 11 + b + 1000 * l as u64 + 100_000 * m as u64;
            let bu = burst(l, m, s, seed, secs);
            let mut rx = PhyRx::with_numerology(bu.num, bu.center);
            let mut got = Vec::new();
            rx.process(&bu.audio, &mut got);
            let Some(x) = got.into_iter().find(|x| x.user == bu.user && !x.header_only) else {
                return passes
                    .iter()
                    .map(|_| (0, bu.infos.len(), Vec::new(), 0.0, 0, 0))
                    .collect();
            };
            let grid = x.grid.clone().expect("data burst");
            let soft = Soft::Llrs(x.llrs.clone());
            passes
                .iter()
                .map(|&p| {
                    let mut h = HarqRx::new();
                    let mut t = 0.0;
                    let rcv = h.receive_turbo(l, 0, &soft, p, &mut |prior| {
                        let t0 = Instant::now();
                        let o = grid.refine(prior, REFINE_K);
                        t += t0.elapsed().as_secs_f64();
                        o
                    });
                    let mut stored = h.stored_llrs().into_iter();
                    let mut wrong = 0;
                    let per: Vec<Option<f64>> = rcv
                        .frames
                        .iter()
                        .zip(&bu.infos)
                        .map(|(f, info)| match f {
                            Some(bytes) => {
                                wrong += usize::from(audio::bytes_to_bits(bytes)[..fec::K] != info[..]);
                                None
                            }
                            None => stored.next().map(|llrs| llr_info(&llrs, &fec::encode(r, info))),
                        })
                        .collect();
                    let ok = per.iter().filter(|v| v.is_none()).count() - wrong;
                    (ok, bu.infos.len(), per, t, rcv.turbo_passes, wrong)
                })
                .collect()
        });
        println!("\n### turbo L{l} {}, {bursts} bursts per point", m.name());
        println!(
            "| S:N | {} |",
            passes
                .iter()
                .map(|p| format!("{p} passes: loss | ms/pass | wrong"))
                .collect::<Vec<_>>()
                .join(" | ")
        );
        let mut curves = vec![Vec::new(); passes.len()];
        let mut info = vec![(0f64, 0usize); passes.len()];
        for (si, &s) in snrs.iter().enumerate() {
            let mut row = format!("| {s} |");
            for (pi, curve) in curves.iter_mut().enumerate() {
                let (mut ok, mut all, mut t, mut np, mut bad) = (0, 0, 0.0, 0, 0);
                for (j, rr) in jobs.iter().zip(&res) {
                    if j.0 == si {
                        ok += rr[pi].0;
                        all += rr[pi].1;
                        t += rr[pi].3;
                        np += rr[pi].4;
                        bad += rr[pi].5;
                        // Codewords failed under every pass count.
                        for (f, v) in rr[pi].2.iter().enumerate() {
                            if let Some(v) = v
                                && rr.iter().all(|o| o.2.get(f).is_some_and(|x| x.is_some()))
                            {
                                info[pi].0 += v;
                                info[pi].1 += 1;
                            }
                        }
                    }
                }
                let loss = 1.0 - ok as f32 / all.max(1) as f32;
                curve.push((s, loss));
                row += &format!(" {loss:.3} | {:.0} | {bad} |", 1e3 * t / np.max(1) as f64);
            }
            println!("{row}");
        }
        for ((p, c), (isum, n)) in passes.iter().zip(&curves).zip(&info) {
            println!(
                "{} {p} passes: 10% {} 30% {} 50% {}; stored LLRs of {n} codewords failed every way: {:.4} bit/bit",
                m.name(),
                crossing(c, 0.1),
                crossing(c, 0.3),
                crossing(c, 0.5),
                isum / (*n).max(1) as f64
            );
        }
    }
}

/// Session byte of the link the expected-header tests receive in.
const SESSION: u16 = 0x5A;

/// A data header's `user` as the station builds it: the level's air
/// code (OFDM levels from 2), the session, the burst number.
fn user_for(l: usize, session: u16, id: u16) -> u16 {
    let first_ofdm = LADDER.iter().position(|x| matches!(x.wave, Wave::Ofdm(_))).unwrap();
    (l - first_ofdm + 2) as u16 | session << 4 | id << 12
}

/// What a receiving station expects of a data burst in level `l`'s
/// guard (`Station::expected`, data part, nothing stored): every level
/// of the guard whose threshold is at most 5 dB, every frame count up to
/// an 8 s burst, the next two burst numbers.
fn expected_after(l: usize, session: u16) -> Vec<ExpectedHeader> {
    let Wave::Ofdm(g) = LADDER[l].wave else {
        panic!("not OFDM")
    };
    let num = audio::numerology(g);
    let mut out = Vec::new();
    for (lv, &x) in LADDER.iter().enumerate() {
        if x.wave != LADDER[l].wave || EFF_THRESHOLDS_DB[lv] > 5.0 {
            continue;
        }
        let opt = audio::tx_options(lv);
        let fmax = frames_for(num, x.pilots, x.constellation, x.rate, 8.0);
        for f in 1..=fmax {
            for id in [1, 2] {
                out.push(ExpectedHeader {
                    constellation: x.constellation,
                    data_symbols: phy::data_symbols_with(num, x.pilots, x.constellation, f * x.rate.n()),
                    user: user_for(lv, session, id),
                    extra: 0,
                    pilots: x.pilots,
                    mode: DataMode::new(opt.spread, opt.data_gain_db),
                });
            }
        }
    }
    out
}

/// Header outcome of one burst through one receiver.
#[derive(Clone, Copy, Default)]
struct HdrOut {
    synced: bool,
    /// Delivered with the true header.
    right: bool,
    /// Delivered with a header of our session that is not the true one.
    wrong: bool,
    /// Taken from the expected set.
    expected: bool,
    margin: f32,
}

fn receive_hdr(b: &Burst, want: (u16, usize), expect: Option<&[ExpectedHeader]>, margin: f32) -> HdrOut {
    let mut rx = PhyRx::with_numerology(b.num, b.center);
    if let Some(e) = expect {
        rx.expect(e);
    }
    rx.set_hyp_margin(margin);
    let mut got = Vec::new();
    rx.process(&b.audio, &mut got);
    let st = rx.stats();
    let mut o = HdrOut {
        synced: st.syncs > 0,
        expected: st.header_expected > 0,
        margin: st.hyp_margin,
        ..HdrOut::default()
    };
    for x in &got {
        let nsym = phy::data_symbols_with(b.num, x.pilots, x.constellation, x.llrs.len().max(1));
        if (x.user, nsym) == want && !x.header_only {
            o.right = true;
        } else if x.user >> 4 & 0xFF == SESSION {
            o.wrong = true;
        }
    }
    o
}

/// Expected-header decoding: per guard (`RX_HDR_LEVELS`, default `9,7`:
/// Normal and Long QPSK) and channel, the share of bursts lost before
/// the data with and without the expected set of a station that heard
/// the level last (a few hundred headers), and wrong headers taken.
/// `RX_HYP_MARGIN` sets the margin (default `HYP_MARGIN`; a large value
/// prints the margin's spread for calibration).
#[test]
#[ignore]
fn hdr_floor() {
    let levels: Vec<usize> = list("RX_HDR_LEVELS", "9,7");
    let snrs: Vec<f32> = list("RX_SNRS", "-4,-3,-2,-1,0,1,2,3,4,5,6");
    let bursts: u64 = env("RX_BURSTS").unwrap_or(60);
    let margin: f32 = env("RX_HYP_MARGIN").unwrap_or(ion::phy::HYP_MARGIN);
    for &l in &levels {
        let exp = expected_after(l, SESSION);
        for m in modes() {
            let jobs: Vec<(usize, f32, u64)> = snrs
                .iter()
                .enumerate()
                .flat_map(|(si, &s)| (0..bursts).map(move |b| (si, s, b)))
                .collect();
            let res = parallel(&jobs, [HdrOut::default(); 2], |&(_, s, b)| {
                let seed = 7 + b + 1000 * l as u64 + 100_000 * m as u64;
                let user = user_for(l, SESSION, 1);
                let bu = burst_as(l, m, s, seed, 8.0, user);
                let want = (user, bu.nsym);
                [
                    receive_hdr(&bu, want, None, margin),
                    receive_hdr(&bu, want, Some(&exp), margin),
                ]
            });
            println!(
                "\n### L{l} {}, {} expected headers, {bursts} bursts per point",
                m.name(),
                exp.len()
            );
            println!("| S:N | no sync | plain: header lost | expected: header lost | taken from the set | wrong |");
            println!("|---:|---:|---:|---:|---:|---:|");
            let mut curves = [Vec::new(), Vec::new()];
            let (mut ok_m, mut bad_m) = (Vec::new(), Vec::new());
            for (si, &s) in snrs.iter().enumerate() {
                let v: Vec<&[HdrOut; 2]> = jobs
                    .iter()
                    .zip(&res)
                    .filter(|(j, _)| j.0 == si)
                    .map(|(_, r)| r)
                    .collect();
                let n = v.len() as f32;
                let nosync = v.iter().filter(|r| !r[0].synced).count() as f32 / n;
                let lost0 = v.iter().filter(|r| !r[0].right).count() as f32 / n;
                let lost1 = v.iter().filter(|r| !r[1].right).count() as f32 / n;
                let taken = v.iter().filter(|r| r[1].expected).count();
                let wrong = v.iter().filter(|r| r[1].wrong).count();
                for r in &v {
                    if r[1].expected && r[1].right {
                        ok_m.push(r[1].margin);
                    } else if r[1].expected {
                        bad_m.push(r[1].margin);
                    }
                }
                curves[0].push((s, lost0));
                curves[1].push((s, lost1));
                println!("| {s} | {nosync:.2} | {lost0:.2} | {lost1:.2} | {taken} | {wrong} |");
            }
            println!(
                "{} L{l} 10% of bursts lost before the data: plain {}, expected {}",
                m.name(),
                crossing(&curves[0], 0.1),
                crossing(&curves[1], 0.1)
            );
            let q = |v: &mut Vec<f32>| {
                v.sort_by(f32::total_cmp);
                let at = |p: f32| {
                    v.get(((v.len() as f32 - 1.0) * p) as usize)
                        .copied()
                        .unwrap_or(f32::NAN)
                };
                format!(
                    "n {} median {:.1} 90% {:.1} 99% {:.1} max {:.1}",
                    v.len(),
                    at(0.5),
                    at(0.9),
                    at(0.99),
                    at(1.0)
                )
            };
            println!("margin, right header taken: {}", q(&mut ok_m));
            println!("margin, wrong header taken: {}", q(&mut bad_m));
        }
    }
}

/// False headers from expected-header decoding: bursts of another link
/// (another session) at every S:N, bursts of our own link whose header
/// is not among those expected (HARQ check bits), and noise alone, all through receivers
/// expecting our link's next burst. `RX_BURSTS` per S:N (default 40),
/// `RX_NOISE_S` seconds of noise per guard (default 1200).
#[test]
#[ignore]
fn hdr_false_long() {
    false_accepts(
        env("RX_BURSTS").unwrap_or(40),
        env("RX_NOISE_S").unwrap_or(1200.0),
        &list("RX_SNRS", "-3,-1,0,1,2,3,4,6,9"),
    );
}

/// [`hdr_false_long`] in brief, for the regular test run.
#[test]
fn expected_headers_take_no_false_ones() {
    false_accepts(4, 60.0, &[-1.0, 1.0, 3.0]);
}

fn false_accepts(bursts: u64, noise_s: f32, snrs: &[f32]) {
    for l in [9usize, 7] {
        let exp = expected_after(l, SESSION);
        let Wave::Ofdm(g) = LADDER[l].wave else { unreachable!() };
        let num = audio::numerology(g);
        let jobs: Vec<(Mode, f32, u64, bool)> = [Mode::Wgn, Mode::Mpp]
            .into_iter()
            .flat_map(|m| {
                snrs.iter()
                    .flat_map(move |&s| (0..bursts).flat_map(move |b| [(m, s, b, false), (m, s, b, true)]))
            })
            .collect();
        let margin: f32 = env("RX_HYP_MARGIN").unwrap_or(ion::phy::HYP_MARGIN);
        let res = parallel(&jobs, (0usize, 0usize, f32::NAN), |&(m, s, b, own)| {
            let seed = 900 + b + 1000 * l as u64 + 100_000 * m as u64;
            // Our link's burst with HARQ check bits nobody expects, or
            // another link's.
            let user = if own {
                user_for(l, SESSION, 1 | 1 << 2)
            } else {
                user_for(l, 0x33, 1)
            };
            let bu = burst_as(l, m, s, seed, 8.0, user);
            let o = receive_hdr(&bu, (user, bu.nsym), Some(&exp), margin);
            (
                o.wrong as usize,
                o.expected as usize,
                if o.wrong { o.margin } else { f32::NAN },
            )
        });
        let wrong: usize = res.iter().map(|r| r.0).sum();
        let mut wm: Vec<f32> = res.iter().map(|r| r.2).filter(|m| m.is_finite()).collect();
        wm.sort_by(f32::total_cmp);
        println!("L{l}: margins of the wrong headers taken: {wm:.1?}");
        let tested = jobs.len();
        // Noise alone.
        let mut rx = PhyRx::with_numerology(num, num.center_hz());
        rx.expect(&exp);
        let mut noise = ion_dsp::dsp::NoiseGen::new(99 + l as u64);
        let mut got = Vec::new();
        let n = (noise_s * 48_000.0) as usize;
        let mut done = 0;
        while done < n {
            let mut a = vec![0f32; 48_000];
            ion_dsp::channel::add_awgn(&mut a, 0.1, &mut noise);
            rx.process(&a, &mut got);
            done += a.len();
        }
        let st = rx.stats();
        println!(
            "L{l}: {tested} bursts of another link or an unexpected length, {wrong} taken as ours; \
             {noise_s} s of noise: {} syncs, {} headers failed, {} taken from the set",
            st.syncs, st.header_fail, st.header_expected
        );
        assert_eq!(wrong, 0, "L{l}: a burst that is not the expected one was taken for it");
        assert_eq!(st.header_expected, 0, "L{l}: noise taken for an expected header");
    }
}
