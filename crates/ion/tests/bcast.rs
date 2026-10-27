//! Broadcast groups on the simulated air: frames reach every station of the
//! group once, tags come back when sent, stations share the channel, and
//! the level follows the weakest station heard.

use std::collections::BTreeMap;
use std::sync::Arc;

use ion::link::Air;
use ion::link::bcast::{BcastConfig, BcastEvent, BcastStation};
use ion::link::net_sim::NetSim;
use ion::link::sim::ModelAir;

const GROUP: &str = "D2G";

fn call(i: usize) -> String {
    format!("N{i}AAA")
}

fn sim(n: usize, snr: f32, seed: u64) -> (NetSim<BcastStation>, Vec<usize>) {
    sim_with(n, snr, seed, |_| {})
}

fn sim_with(n: usize, snr: f32, seed: u64, tweak: impl Fn(&mut BcastConfig)) -> (NetSim<BcastStation>, Vec<usize>) {
    let air: Arc<dyn Air> = Arc::new(ModelAir::default());
    let mut groups = Vec::new();
    let st = (0..n)
        .map(|i| {
            let mut c = BcastConfig::new(&call(i));
            c.seed = seed * 7919 + i as u64 + 1;
            tweak(&mut c);
            let mut s = BcastStation::new(c, air.clone());
            groups.push(s.open(GROUP));
            s
        })
        .collect();
    (NetSim::new(st, snr, air, seed), groups)
}

/// Everything each station saw.
fn run(sim: &mut NetSim<BcastStation>, ev: &mut [Vec<BcastEvent>], secs: f64) {
    let end = sim.t + secs;
    while sim.t < end {
        for (i, s) in sim.stations.iter_mut().enumerate() {
            ev[i].extend(s.take_events());
        }
        for _ in 0..100 {
            sim.step();
        }
    }
    for (i, s) in sim.stations.iter_mut().enumerate() {
        ev[i].extend(s.take_events());
    }
}

fn frames(ev: &[BcastEvent]) -> Vec<Vec<u8>> {
    ev.iter()
        .filter_map(|e| match e {
            BcastEvent::Frame { data, .. } => Some(data.clone()),
            _ => None,
        })
        .collect()
}

fn line(i: usize, k: usize) -> Vec<u8> {
    let mut v = format!("{} line {k}: ", call(i)).into_bytes();
    // Zeros and KISS's special bytes travel too.
    v.extend([0, 0xC0, 0xDB, 0]);
    v.extend(std::iter::repeat_n(b'x', 20 + 17 * k));
    v
}

#[test]
fn frames_reach_every_station_once_and_tags_come_back() {
    let (mut sim, g) = sim(4, 8.0, 1);
    let mut ev = vec![Vec::new(); 4];
    for k in 0..5 {
        sim.stations[0].send(g[0], &line(0, k), Some(100 + k as u16));
    }
    run(&mut sim, &mut ev, 300.0);
    let sent: Vec<Vec<u8>> = (0..5).map(|k| line(0, k)).collect();
    for e in &ev[1..] {
        assert_eq!(frames(e), sent);
    }
    let tags: Vec<u16> = ev[0]
        .iter()
        .filter_map(|e| match e {
            BcastEvent::Sent { tag } => Some(*tag),
            _ => None,
        })
        .collect();
    assert_eq!(tags, vec![100, 101, 102, 103, 104]);
    // The others know who sent it.
    assert!(
        ev[1]
            .iter()
            .any(|e| matches!(e, BcastEvent::Heard { call: Some(c), .. } if *c == call(0)))
    );
}

/// Six stations, each sending a line at random moments over 20 minutes:
/// how many of the lines reach how many of the others.
fn shared(seed: u64, snr: f32) -> (usize, usize) {
    let (got, want, _) = shared_with(seed, snr, 0.0, None);
    (got, want)
}

/// [`shared`] with slow fading on every path and a level margin (`None`:
/// the default); also the airtime.
fn shared_with(seed: u64, snr: f32, fade_db: f32, margin_db: Option<f32>) -> (usize, usize, f64) {
    let n = 6;
    let (mut sim, g) = sim_with(n, snr, seed, |c| {
        if let Some(m) = margin_db {
            c.margin_db = m;
        }
    });
    sim.fade_db = fade_db;
    sim.fade_tau_s = 20.0;
    let mut ev = vec![Vec::new(); n];
    let mut rng = seed;
    let mut at: BTreeMap<u64, usize> = BTreeMap::new();
    for k in 0..30u64 {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        at.insert((rng >> 33) % 1200 * 10 + k, (k as usize) % n);
    }
    let mut sent = Vec::new();
    for (t, i) in at {
        let t = t as f64 / 10.0;
        let wait = t - sim.t;
        if wait > 0.0 {
            run(&mut sim, &mut ev, wait);
        }
        let k = sent.len();
        sim.stations[i].send(g[i], &line(i, k), None);
        sent.push((i, line(i, k)));
    }
    run(&mut sim, &mut ev, 300.0);
    let mut got = 0;
    for (i, f) in &sent {
        got += (0..n).filter(|&j| j != *i && frames(&ev[j]).contains(f)).count();
    }
    let air = sim.stations.iter().map(|s| s.stats.airtime_s).sum();
    (got, sent.len() * (n - 1), air)
}

#[test]
fn stations_share_the_channel() {
    let (got, want) = shared(2, 8.0);
    assert!(got as f64 >= 0.95 * want as f64, "{got} of {want}");
}

#[test]
fn the_level_follows_the_weakest_station_heard() {
    // Strong neighbours: after hearing them, faster than the fallback.
    let (mut sim, g) = sim(3, 15.0, 3);
    let mut ev = vec![Vec::new(); 3];
    for (i, &gi) in g.iter().enumerate() {
        sim.stations[i].send(gi, &line(i, 0), None);
        run(&mut sim, &mut ev, 60.0);
    }
    let fallback = BcastConfig::new("X").fallback_level;
    let before = sim.stations[0].stats.airtime_s;
    sim.stations[0].send(g[0], &line(0, 9), None);
    run(&mut sim, &mut ev, 60.0);
    let fast = sim.stations[0].stats.airtime_s - before;
    // The same frame at the fallback level, from a station that heard
    // nobody.
    let (mut lone, lg) = self::sim(2, 15.0, 4);
    let mut lev = vec![Vec::new(); 2];
    lone.stations[0].send(lg[0], &line(0, 9), None);
    run(&mut lone, &mut lev, 60.0);
    let slow = lone.stations[0].stats.airtime_s;
    assert!(
        fast < 0.6 * slow,
        "after hearing 15 dB stations {fast:.1} s, at the fallback {slow:.1} s (level {fallback})"
    );
    // A weak station heard pulls the level down for everyone it hears.
    let (mut sim, g) = self::sim(3, 15.0, 5);
    sim.set_path(0, 2, -4.0);
    let mut ev = vec![Vec::new(); 3];
    for (i, &gi) in g.iter().enumerate() {
        sim.stations[i].send(gi, &line(i, 0), None);
        run(&mut sim, &mut ev, 60.0);
    }
    sim.stations[0].send(g[0], &line(0, 9), None);
    run(&mut sim, &mut ev, 120.0);
    assert!(frames(&ev[2]).contains(&line(0, 9)), "the weak station missed it");
}

#[test]
fn a_full_kiss_frame_goes_whole_at_the_fallback() {
    let (mut sim, g) = sim(2, 0.0, 6);
    let mut ev = vec![Vec::new(); 2];
    let big: Vec<u8> = (0..255u32).map(|i| (i * 7) as u8).collect();
    sim.stations[0].send(g[0], &big, Some(1));
    run(&mut sim, &mut ev, 120.0);
    assert_eq!(frames(&ev[1]), vec![big]);
    assert_eq!(sim.stations[0].stats.bursts, 1);
}

#[test]
fn another_groups_frames_are_not_ours() {
    let air: Arc<dyn Air> = Arc::new(ModelAir::default());
    let mut a = BcastStation::new(BcastConfig::new("N0AAA"), air.clone());
    let mut b = BcastStation::new(BcastConfig::new("N1AAA"), air.clone());
    let ga = a.open("D2G");
    b.open("NETS");
    a.send(ga, b"for D2G only", None);
    let mut sim = NetSim::new(vec![a, b], 10.0, air, 7);
    let mut ev = vec![Vec::new(); 2];
    run(&mut sim, &mut ev, 60.0);
    assert!(frames(&ev[1]).is_empty());
}

/// Prints delivery when stations share the channel, over many seeds, by
/// fading and level margin: `cargo test --release --test bcast --
/// --ignored --nocapture`.
#[test]
#[ignore]
fn sharing_table() {
    println!("| S/N | fading | margin | delivered | airtime |");
    println!("|---|---|---|---:|---:|");
    for (snr, fade) in [(8.0, 0.0), (0.0, 0.0), (8.0, 6.0), (0.0, 6.0), (15.0, 6.0)] {
        for margin in [3.0, 6.0, 9.0] {
            let (mut got, mut want, mut air) = (0, 0, 0.0);
            for seed in 1..=20 {
                let (g, w, a) = shared_with(seed * 101, snr, fade, Some(margin));
                got += g;
                want += w;
                air += a;
            }
            println!(
                "| {snr} dB | {fade} dB | {margin} dB | {:.1}% | {:.0} s |",
                100.0 * got as f64 / want as f64,
                air / 20.0
            );
        }
    }
}
