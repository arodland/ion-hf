//! CW (Morse code): the station ID sender, and a receiver with adaptive
//! timing.
//!
//! Timing is PARIS: a dot is 1.2/wpm s, a dash three dots, the gap inside a
//! character one dot, between characters three and between words seven.

use std::collections::VecDeque;

use crate::dsp::AUDIO_RATE;
use crate::{Modulator, TX_LEVEL};

/// Speed range the modulator accepts.
pub const CW_MIN_WPM: f32 = 5.0;
pub const CW_MAX_WPM: f32 = 60.0;

/// Text and Morse pattern. Where two entries share a pattern the decoder
/// shows the first, so `.-.-.` reads as `<AR>` and `-...-` as `=`.
const MORSE: &[(&str, &str)] = &[
    ("A", ".-"),
    ("B", "-..."),
    ("C", "-.-."),
    ("D", "-.."),
    ("E", "."),
    ("F", "..-."),
    ("G", "--."),
    ("H", "...."),
    ("I", ".."),
    ("J", ".---"),
    ("K", "-.-"),
    ("L", ".-.."),
    ("M", "--"),
    ("N", "-."),
    ("O", "---"),
    ("P", ".--."),
    ("Q", "--.-"),
    ("R", ".-."),
    ("S", "..."),
    ("T", "-"),
    ("U", "..-"),
    ("V", "...-"),
    ("W", ".--"),
    ("X", "-..-"),
    ("Y", "-.--"),
    ("Z", "--.."),
    ("0", "-----"),
    ("1", ".----"),
    ("2", "..---"),
    ("3", "...--"),
    ("4", "....-"),
    ("5", "....."),
    ("6", "-...."),
    ("7", "--..."),
    ("8", "---.."),
    ("9", "----."),
    ("<AR>", ".-.-."),
    ("<KN>", "-.--."),
    ("<AS>", ".-..."),
    ("<SK>", "...-.-"),
    ("<BK>", "-...-.-"),
    ("<SOS>", "...---..."),
    ("=", "-...-"),
    ("<BT>", "-...-"),
    (".", ".-.-.-"),
    (",", "--..--"),
    ("?", "..--.."),
    ("/", "-..-."),
    ("+", ".-.-."),
    ("-", "-....-"),
    ("'", ".----."),
    ("(", "-.--."),
    (")", "-.--.-"),
    (":", "---..."),
    (";", "-.-.-."),
    ("\"", ".-..-."),
    ("@", ".--.-."),
    ("!", "-.-.--"),
    ("&", ".-..."),
    ("_", "..--.-"),
    ("$", "...-..-"),
];

/// Longest pattern in [`MORSE`].
const MAX_ELEMENTS: usize = 9;
/// Longest prosign text including the angle brackets (`<SOS>`).
const MAX_PROSIGN: usize = 5;

/// Pattern for a single (upper-case) character.
fn pattern_for_char(c: char) -> Option<&'static str> {
    MORSE.iter().find_map(|&(t, p)| {
        let mut it = t.chars();
        (it.next() == Some(c) && it.next().is_none()).then_some(p)
    })
}

/// Pattern for a prosign written as `<..>` (already upper case).
fn pattern_for_prosign(name: &[char]) -> Option<&'static str> {
    MORSE
        .iter()
        .find_map(|&(t, p)| (t.starts_with('<') && t.chars().eq(name.iter().copied())).then_some(p))
}

/// Raised-cosine key edge, samples (5 ms). Each edge starts at the nominal key
/// time, so every half-amplitude crossing is shifted by the same 2.5 ms and
/// the durations between them are exact.
const RAMP: usize = 240;
/// Silence emitted per step while keyed with nothing to send (1 ms).
const IDLE_CHUNK: u32 = 48;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TxState {
    Idle,
    Running,
}

enum Next {
    /// A character's elements are loaded.
    Loaded,
    /// A character was taken but has no Morse code.
    Skipped,
    /// A `<` might still become a prosign: wait for more text.
    Wait,
    Empty,
}

/// Sends CW at `wpm` words per minute with shaped (click-free) keying.
pub struct CwMod {
    center: f32,
    /// Samples per dot.
    unit: u32,
    phase: f64,
    shape: Vec<f32>,
    ramp: usize,
    state: TxState,
    finishing: bool,
    queue: VecDeque<char>,
    /// Elements of the current character: (key down, length in dots).
    elems: [(bool, u8); 2 * MAX_ELEMENTS + 1],
    nelems: usize,
    ielem: usize,
    key: bool,
    samples_left: u32,
    /// Part of a final gap cut short because the transmission was ending.
    trimmed: u32,
    sent: usize,
}

impl CwMod {
    pub fn new(center_hz: f32, wpm: f32) -> Self {
        let shape = (0..=RAMP)
            .map(|k| 0.5 - 0.5 * (core::f32::consts::PI * k as f32 / RAMP as f32).cos())
            .collect();
        let wpm = wpm.clamp(CW_MIN_WPM, CW_MAX_WPM);
        Self {
            center: center_hz,
            unit: (AUDIO_RATE * 1.2 / wpm).round() as u32,
            phase: 0.0,
            shape,
            ramp: 0,
            state: TxState::Idle,
            finishing: false,
            queue: VecDeque::with_capacity(1024),
            elems: [(false, 0); 2 * MAX_ELEMENTS + 1],
            nelems: 0,
            ielem: 0,
            key: false,
            samples_left: 0,
            trimmed: 0,
            sent: 0,
        }
    }

    fn begin(&mut self) {
        self.state = TxState::Running;
        self.finishing = false;
        self.nelems = 0;
        self.ielem = 0;
        self.key = false;
        self.samples_left = 0;
        self.trimmed = 0;
    }

    fn take(&mut self, n: usize) {
        for _ in 0..n {
            self.queue.pop_front();
        }
        self.sent += n;
    }

    fn load_pattern(&mut self, p: &str) {
        self.nelems = 0;
        for (i, b) in p.bytes().enumerate() {
            if i > 0 {
                self.elems[self.nelems] = (false, 1);
                self.nelems += 1;
            }
            self.elems[self.nelems] = (true, if b == b'-' { 3 } else { 1 });
            self.nelems += 1;
        }
        self.elems[self.nelems] = (false, 3);
        self.nelems += 1;
        self.ielem = 0;
    }

    fn load_next(&mut self) -> Next {
        let Some(&c) = self.queue.front() else {
            return Next::Empty;
        };
        if c == '<' {
            let mut name = ['\0'; MAX_PROSIGN];
            let mut seen = 0;
            let mut close = None;
            for (i, &ch) in self.queue.iter().take(MAX_PROSIGN).enumerate() {
                name[i] = ch.to_ascii_uppercase();
                seen = i + 1;
                if ch == '>' {
                    close = Some(i);
                    break;
                }
            }
            return match close {
                Some(i) => match pattern_for_prosign(&name[..=i]) {
                    Some(p) => {
                        self.take(i + 1);
                        self.load_pattern(p);
                        Next::Loaded
                    }
                    None => {
                        self.take(1);
                        Next::Skipped
                    }
                },
                None if seen < MAX_PROSIGN && !self.finishing => Next::Wait,
                None => {
                    self.take(1);
                    Next::Skipped
                }
            };
        }
        self.take(1);
        if c.is_whitespace() {
            // With the 3-dot gap after the previous character: 7 dots.
            self.elems[0] = (false, 4);
            self.nelems = 1;
            self.ielem = 0;
            return Next::Loaded;
        }
        match pattern_for_char(c.to_ascii_uppercase()) {
            Some(p) => {
                self.load_pattern(p);
                Next::Loaded
            }
            None => Next::Skipped,
        }
    }

    /// Moves to the next element; false when there is nothing left and the
    /// transmission is finishing.
    fn advance(&mut self) -> bool {
        loop {
            if self.ielem < self.nelems {
                let (on, dots) = self.elems[self.ielem];
                self.ielem += 1;
                self.key = on;
                let full = dots as u32 * self.unit;
                self.samples_left = full;
                self.trimmed = 0;
                if !on && self.ielem == self.nelems && self.finishing && self.queue.is_empty() {
                    // Last gap of the transmission: only wait for the edge.
                    let short = (RAMP as u32).min(full);
                    self.trimmed = full - short;
                    self.samples_left = short;
                }
                return true;
            }
            match self.load_next() {
                Next::Loaded | Next::Skipped => continue,
                Next::Wait => {}
                Next::Empty if self.finishing => return false,
                Next::Empty => {}
            }
            self.key = false;
            self.samples_left = IDLE_CHUNK;
            return true;
        }
    }
}

impl Modulator for CwMod {
    fn set_center_hz(&mut self, hz: f32) {
        self.center = hz;
    }

    fn push_text(&mut self, text: &str) {
        self.queue.extend(text.chars());
        match self.state {
            TxState::Idle => self.begin(),
            TxState::Running => {
                self.finishing = false;
                // A gap cut short for the end gets its full length back.
                self.samples_left += self.trimmed;
                self.trimmed = 0;
            }
        }
    }

    fn fill(&mut self, out: &mut [f32]) -> usize {
        let mut n = 0;
        let inc = self.center as f64 / AUDIO_RATE as f64;
        for o in out.iter_mut() {
            if self.state == TxState::Idle {
                break;
            }
            if self.samples_left == 0 && !self.advance() {
                self.key = false;
                if self.ramp == 0 {
                    self.state = TxState::Idle;
                    break;
                }
            }
            self.samples_left = self.samples_left.saturating_sub(1);
            if self.key {
                if self.ramp < RAMP {
                    self.ramp += 1;
                }
            } else if self.ramp > 0 {
                self.ramp -= 1;
            }
            let env = self.shape[self.ramp];
            *o = if env > 0.0 {
                TX_LEVEL * env * (self.phase * core::f64::consts::TAU).sin() as f32
            } else {
                0.0
            };
            self.phase += inc;
            self.phase -= self.phase.floor();
            n += 1;
        }
        n
    }

    fn pending(&self) -> usize {
        self.queue.len()
    }

    fn start(&mut self) {
        if self.state == TxState::Idle {
            self.begin();
        }
        self.finishing = false;
    }

    fn finish(&mut self) {
        if self.state == TxState::Running {
            self.finishing = true;
        }
    }

    fn abort(&mut self) {
        self.queue.clear();
        self.state = TxState::Idle;
        self.finishing = false;
        self.ramp = 0;
        self.key = false;
        self.samples_left = 0;
        self.trimmed = 0;
        self.nelems = 0;
        self.ielem = 0;
    }

    fn is_active(&self) -> bool {
        self.state != TxState::Idle
    }

    fn chars_sent(&self) -> usize {
        self.sent
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prosign_waits_for_closing_bracket() {
        let mut m = CwMod::new(700.0, 30.0);
        let mut buf = vec![0.0; 4800];
        m.push_text("<A");
        assert_eq!(m.fill(&mut buf), buf.len());
        assert_eq!(m.chars_sent(), 0, "must wait: <A could become <AR>");
        m.push_text("R>");
        m.fill(&mut buf);
        assert_eq!(m.chars_sent(), 4);
        assert!(buf.iter().any(|x| x.abs() > 0.1));
    }
}
