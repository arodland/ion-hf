//! A small net over real modulated audio: every burst goes through the
//! Ion PHYs, every station hears the sum of the others (each at its
//! path's gain) plus its own noise, so simultaneous check-ins really
//! collide.

use ion::link::net::{NetConfig, NetEvent, NetNode};
use ion_dsp::channel::add_awgn;
use ion_dsp::dsp::NoiseGen;

const CHUNK: usize = 480;

struct Station {
    node: NetNode,
    out: Vec<f32>,
    events: Vec<NetEvent>,
}

struct Air {
    st: Vec<Station>,
    sigma: f32,
    /// Amplitude from `[from][to]`.
    gain: Vec<Vec<f32>>,
    noise: NoiseGen,
    t: f64,
    /// (start, station, seconds) of every burst.
    timeline: Vec<(f64, usize, f64)>,
}

impl Air {
    fn new(calls: &[&str], sigma: f32) -> Air {
        Self::with(calls, sigma, |_| {})
    }

    fn with(calls: &[&str], sigma: f32, tweak: impl Fn(&mut NetConfig)) -> Air {
        let st: Vec<Station> = calls
            .iter()
            .enumerate()
            .map(|(i, c)| {
                let mut cfg = NetConfig::new(c, "TNARES", i == 0);
                tweak(&mut cfg);
                Station {
                    node: NetNode::new(cfg, 0.2),
                    out: Vec::new(),
                    events: Vec::new(),
                }
            })
            .collect();
        let n = st.len();
        Air {
            st,
            sigma,
            gain: vec![vec![1.0; n]; n],
            noise: NoiseGen::new(42),
            t: 0.0,
            timeline: Vec::new(),
        }
    }

    fn step(&mut self) {
        self.t += CHUNK as f64 / 48_000.0;
        let t = self.t;
        let n = self.st.len();
        let mut played = vec![vec![0.0f32; CHUNK]; n];
        for (i, s) in self.st.iter_mut().enumerate() {
            if s.out.is_empty()
                && let Some(a) = s.node.poll(t)
            {
                self.timeline.push((t, i, a.len() as f64 / 48_000.0));
                s.out.extend(a);
            }
            let k = s.out.len().min(CHUNK);
            played[i][..k].copy_from_slice(&s.out[..k]);
            s.out.drain(..k);
            if k > 0 && s.out.is_empty() {
                s.node.tx_done(t);
            }
        }
        for i in 0..n {
            let mut heard = vec![0.0f32; CHUNK];
            for (j, p) in played.iter().enumerate() {
                let g = self.gain[j][i];
                if j != i && g > 0.0 {
                    for (h, x) in heard.iter_mut().zip(p) {
                        *h += g * x;
                    }
                }
            }
            add_awgn(&mut heard, self.sigma, &mut self.noise);
            self.st[i].node.push_audio(&heard, t);
            let ev = self.st[i].node.station_mut().take_events();
            self.st[i].events.extend(ev);
        }
    }

    fn run_until(&mut self, secs: f64, mut done: impl FnMut(&Air) -> bool) -> bool {
        let end = self.t + secs;
        while self.t < end {
            if done(self) {
                return true;
            }
            self.step();
        }
        done(self)
    }
}

fn got(ev: &[NetEvent], from: &str) -> bool {
    ev.iter()
        .any(|e| matches!(e, NetEvent::Message { from: f, overheard: false, .. } if f == from))
}

#[test]
fn a_net_over_real_audio() {
    let mut air = Air::new(&["N0JLF", "KJ5ET", "AG5B"], 0.02);
    air.st[1].node.station_mut().send(b"KJ5ET: no traffic, all well");
    air.st[2]
        .node
        .station_mut()
        .send(b"AG5B: generator test complete, 2 hrs fuel");
    air.st[0]
        .node
        .station_mut()
        .send(b"Net closes at 2130. Thanks all. de N0JLF");
    let ok = air.run_until(900.0, |a| {
        got(&a.st[0].events, "KJ5ET")
            && got(&a.st[0].events, "AG5B")
            && a.st[1..]
                .iter()
                .all(|s| s.events.iter().any(|e| matches!(e, NetEvent::Message { to: None, .. })))
    });
    let roster = air.st[0].node.station().roster(air.t);
    assert!(
        ok,
        "after {:.0} s:\nNCS {:#?}\nKJ5ET {:#?}\nAG5B {:#?}\nroster {roster:#?}",
        air.t, air.st[0].events, air.st[1].events, air.st[2].events
    );
}

/// W5FAR hears only AG5B (and only AG5B hears it): it finds the net by
/// calling "looking for a net", and its traffic and the bulletin go through
/// AG5B, all over real audio.
#[test]
fn a_leaf_joins_through_a_relay_over_real_audio() {
    let calls = ["N0JLF", "KJ5ET", "AG5B", "W5FAR"];
    let mut air = Air::with(&calls, 0.02, |c| {
        c.seek_after_s = 60.0;
        c.seek_every_s = 40.0;
    });
    for j in [0, 1] {
        air.gain[3][j] = 0.0;
        air.gain[j][3] = 0.0;
    }
    air.st[3]
        .node
        .station_mut()
        .send(b"W5FAR: bridge out on CR 12, detour via Hwy 80");
    air.st[0]
        .node
        .station_mut()
        .send(b"Net closes at 2130. Thanks all. de N0JLF");
    let ok = air.run_until(1500.0, |a| {
        got(&a.st[0].events, "W5FAR")
            && a.st[3]
                .events
                .iter()
                .any(|e| matches!(e, NetEvent::Message { to: None, .. }))
    });
    let roster = air.st[0].node.station().roster(air.t);
    if !ok {
        for (t, i, d) in air.timeline.iter().take(120) {
            eprintln!("  {t:6.1} s  {:5}  {d:4.1} s", calls[*i]);
        }
    }
    assert!(
        ok,
        "after {:.0} s:\nNCS {:#?}\nAG5B {:#?}\nW5FAR {:#?}\nroster {roster:#?}",
        air.t, air.st[0].events, air.st[2].events, air.st[3].events
    );
    assert!(
        air.st[0]
            .events
            .iter()
            .any(|e| matches!(e, NetEvent::Relayed { source, via } if source == "W5FAR" && via == "AG5B")),
        "{:#?}",
        air.st[0].events
    );
    assert_eq!(air.st[3].node.station().relay_call().as_deref(), Some("AG5B"));
}
