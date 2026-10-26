//! TCI wire format: `name:arg,arg,...;` text commands and 64-byte-header
//! binary audio frames.

use std::fmt;

/// One `name:arg,arg;` command as it appeared on the wire.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TextCommand {
    /// Lower-cased command name.
    pub name: String,
    pub args: Vec<String>,
}

impl fmt::Display for TextCommand {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.args.is_empty() {
            write!(f, "{};", self.name)
        } else {
            write!(f, "{}:{};", self.name, self.args.join(","))
        }
    }
}

/// Streaming parser: a text message may hold several commands, or end in the
/// middle of one.
#[derive(Debug, Default)]
pub struct Parser {
    buf: String,
}

impl Parser {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn feed(&mut self, text: &str, out: &mut Vec<TextCommand>) {
        for ch in text.chars() {
            if ch == ';' {
                if let Some(cmd) = parse_one(&self.buf) {
                    out.push(cmd);
                }
                self.buf.clear();
            } else if !(ch == '\r' || ch == '\n') {
                self.buf.push(ch);
            }
        }
        // A runaway fragment is never a real command.
        if self.buf.len() > 64 * 1024 {
            self.buf.clear();
        }
    }
}

fn parse_one(s: &str) -> Option<TextCommand> {
    let s = s.trim();
    if s.is_empty() {
        return None;
    }
    let (name, rest) = match s.split_once(':') {
        Some((n, r)) => (n, Some(r)),
        None => (s, None),
    };
    let args = match rest {
        Some(r) if !r.is_empty() => r.split(',').map(|a| a.trim().to_string()).collect(),
        _ => Vec::new(),
    };
    Some(TextCommand {
        name: name.trim().to_ascii_lowercase(),
        args,
    })
}

#[derive(Clone, Debug, PartialEq)]
pub enum Event {
    /// Socket connected (before `ready`).
    Connected,
    Disconnected(String),
    Protocol {
        name: String,
        version: String,
    },
    Device(String),
    ReceiveOnly(bool),
    TrxCount(u32),
    ChannelsCount(u32),
    ModulationsList(Vec<String>),
    AudioSampleRate(u32),
    Vfo {
        trx: u32,
        channel: u32,
        hz: i64,
    },
    Modulation {
        trx: u32,
        mode: String,
    },
    Trx {
        trx: u32,
        on: bool,
    },
    Tune {
        trx: u32,
        on: bool,
    },
    /// TCI 2.0: `drive:trx,N;`; TCI 1.x `drive:N;` is reported as trx 0.
    Drive {
        trx: u32,
        value: u32,
    },
    Start,
    Stop,
    Ready,
    /// Anything else; logged and ignored.
    Other(TextCommand),
}

fn arg<T: std::str::FromStr>(c: &TextCommand, i: usize) -> Option<T> {
    c.args.get(i)?.parse().ok()
}

fn arg_bool(c: &TextCommand, i: usize) -> Option<bool> {
    match c.args.get(i)?.to_ascii_lowercase().as_str() {
        "true" | "1" => Some(true),
        "false" | "0" => Some(false),
        _ => None,
    }
}

impl Event {
    pub fn parse(c: &TextCommand) -> Event {
        let parsed = match c.name.as_str() {
            "protocol" => Some(Event::Protocol {
                name: c.args.first().cloned().unwrap_or_default(),
                version: c.args.get(1).cloned().unwrap_or_default(),
            }),
            "device" => c.args.first().map(|d| Event::Device(d.clone())),
            "receive_only" => arg_bool(c, 0).map(Event::ReceiveOnly),
            "trx_count" => arg(c, 0).map(Event::TrxCount),
            "channels_count" => arg(c, 0).map(Event::ChannelsCount),
            "modulations_list" => Some(Event::ModulationsList(c.args.clone())),
            "audio_samplerate" => arg(c, 0).map(Event::AudioSampleRate),
            "vfo" => (|| {
                Some(Event::Vfo {
                    trx: arg(c, 0)?,
                    channel: arg(c, 1)?,
                    hz: arg(c, 2)?,
                })
            })(),
            "modulation" => (|| {
                Some(Event::Modulation {
                    trx: arg(c, 0)?,
                    mode: c.args.get(1)?.to_ascii_lowercase(),
                })
            })(),
            "trx" => (|| {
                Some(Event::Trx {
                    trx: arg(c, 0)?,
                    on: arg_bool(c, 1)?,
                })
            })(),
            "tune" => (|| {
                Some(Event::Tune {
                    trx: arg(c, 0)?,
                    on: arg_bool(c, 1)?,
                })
            })(),
            // TCI 1.x sends `drive:N;`, 2.0 sends `drive:trx,N;`.
            "drive" => match c.args.len() {
                1 => arg(c, 0).map(|value| Event::Drive { trx: 0, value }),
                _ => (|| {
                    Some(Event::Drive {
                        trx: arg(c, 0)?,
                        value: arg(c, 1)?,
                    })
                })(),
            },
            "start" => Some(Event::Start),
            "stop" => Some(Event::Stop),
            "ready" => Some(Event::Ready),
            _ => None,
        };
        parsed.unwrap_or_else(|| Event::Other(c.clone()))
    }
}

/// What the server has told us so far.
#[derive(Clone, Debug, Default)]
pub struct RadioState {
    pub connected: bool,
    pub ready: bool,
    pub protocol: Option<(String, String)>,
    pub device: Option<String>,
    pub receive_only: bool,
    pub trx_count: u32,
    pub channels_count: u32,
    pub modulations: Vec<String>,
    pub audio_samplerate: Option<u32>,
    /// `vfo[trx][channel]` in Hz.
    pub vfo: Vec<[i64; 2]>,
    pub modulation: Vec<String>,
    pub trx: Vec<bool>,
    pub tune: Vec<bool>,
    /// Drive % per transceiver.
    pub drive: Vec<Option<u32>>,
    pub started: bool,
}

fn slot<T: Default + Clone>(v: &mut Vec<T>, i: u32) -> &mut T {
    let i = i as usize;
    if v.len() <= i {
        v.resize(i + 1, T::default());
    }
    &mut v[i]
}

impl RadioState {
    pub fn apply(&mut self, ev: &Event) {
        match ev {
            Event::Connected => {
                *self = RadioState::default();
                self.connected = true;
            }
            Event::Disconnected(_) => {
                *self = RadioState::default();
            }
            Event::Protocol { name, version } => self.protocol = Some((name.clone(), version.clone())),
            Event::Device(d) => self.device = Some(d.clone()),
            Event::ReceiveOnly(b) => self.receive_only = *b,
            Event::TrxCount(n) => self.trx_count = *n,
            Event::ChannelsCount(n) => self.channels_count = *n,
            Event::ModulationsList(l) => self.modulations = l.clone(),
            Event::AudioSampleRate(r) => self.audio_samplerate = Some(*r),
            Event::Vfo { trx, channel, hz } => {
                if *channel < 2 {
                    slot(&mut self.vfo, *trx)[*channel as usize] = *hz;
                }
            }
            Event::Modulation { trx, mode } => *slot(&mut self.modulation, *trx) = mode.clone(),
            Event::Trx { trx, on } => *slot(&mut self.trx, *trx) = *on,
            Event::Tune { trx, on } => *slot(&mut self.tune, *trx) = *on,
            Event::Drive { trx, value } => *slot(&mut self.drive, *trx) = Some(*value),
            Event::Start => self.started = true,
            Event::Stop => self.started = false,
            Event::Ready => self.ready = true,
            Event::Other(_) => {}
        }
    }

    pub fn dial_hz(&self, trx: u32) -> Option<i64> {
        self.vfo.get(trx as usize).map(|v| v[0])
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Command {
    AudioSampleRate(u32),
    AudioStart(u32),
    AudioStop(u32),
    Vfo {
        trx: u32,
        channel: u32,
        hz: i64,
    },
    Modulation {
        trx: u32,
        mode: String,
    },
    /// PTT. `tci_audio` selects TCI as the TX audio source.
    Trx {
        trx: u32,
        on: bool,
        tci_audio: bool,
    },
    Tune {
        trx: u32,
        on: bool,
    },
    /// TX drive % (TCI 2.0 form, per transceiver).
    Drive {
        trx: u32,
        pct: u32,
    },
    RxFilterBand {
        trx: u32,
        lo: i32,
        hi: i32,
    },
    /// Anything not modelled above, sent verbatim (must end with `;`).
    Raw(String),
}

impl Command {
    pub fn to_wire(&self) -> String {
        match self {
            Command::AudioSampleRate(r) => format!("audio_samplerate:{r};"),
            Command::AudioStart(t) => format!("audio_start:{t};"),
            Command::AudioStop(t) => format!("audio_stop:{t};"),
            Command::Vfo { trx, channel, hz } => format!("vfo:{trx},{channel},{hz};"),
            Command::Modulation { trx, mode } => format!("modulation:{trx},{mode};"),
            Command::Trx {
                trx,
                on: true,
                tci_audio: true,
            } => format!("trx:{trx},true,tci;"),
            Command::Trx {
                trx,
                on: true,
                tci_audio: false,
            } => format!("trx:{trx},true;"),
            Command::Trx { trx, on: false, .. } => format!("trx:{trx},false;"),
            Command::Tune { trx, on } => format!("tune:{trx},{on};"),
            Command::Drive { trx, pct } => format!("drive:{trx},{pct};"),
            Command::RxFilterBand { trx, lo, hi } => format!("rx_filter_band:{trx},{lo},{hi};"),
            Command::Raw(s) => s.clone(),
        }
    }
}

pub const HEADER_LEN: usize = 64;

pub mod frame_type {
    pub const IQ: u32 = 0;
    pub const RX_AUDIO: u32 = 1;
    pub const TX_AUDIO: u32 = 2;
    pub const TX_CHRONO: u32 = 3;
    pub const LINE_OUT: u32 = 4;
}

pub mod sample_format {
    pub const INT16: u32 = 0;
    pub const INT24: u32 = 1;
    pub const INT32: u32 = 2;
    pub const FLOAT32: u32 = 3;
}

/// Sixteen little-endian `u32`s, as the TCI specification lays them out.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FrameHeader {
    pub receiver: u32,
    pub sample_rate: u32,
    pub format: u32,
    pub codec: u32,
    pub crc: u32,
    pub length: u32,
    pub frame_type: u32,
    pub reserved: [u32; 9],
}

impl FrameHeader {
    pub fn parse(b: &[u8]) -> Option<FrameHeader> {
        if b.len() < HEADER_LEN {
            return None;
        }
        let w = |i: usize| u32::from_le_bytes([b[4 * i], b[4 * i + 1], b[4 * i + 2], b[4 * i + 3]]);
        let mut reserved = [0u32; 9];
        for (i, r) in reserved.iter_mut().enumerate() {
            *r = w(7 + i);
        }
        Some(FrameHeader {
            receiver: w(0),
            sample_rate: w(1),
            format: w(2),
            codec: w(3),
            crc: w(4),
            length: w(5),
            frame_type: w(6),
            reserved,
        })
    }

    pub fn write(&self, out: &mut Vec<u8>) {
        for v in [
            self.receiver,
            self.sample_rate,
            self.format,
            self.codec,
            self.crc,
            self.length,
            self.frame_type,
        ]
        .into_iter()
        .chain(self.reserved)
        {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
}

/// Decodes a frame payload to `f32` samples (interleaved as received).
/// Returns false for an unsupported format.
pub fn decode_samples(format: u32, payload: &[u8], out: &mut Vec<f32>) -> bool {
    out.clear();
    match format {
        sample_format::FLOAT32 => {
            out.extend(
                payload
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])),
            );
        }
        sample_format::INT16 => {
            out.extend(
                payload
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0),
            );
        }
        sample_format::INT32 => {
            out.extend(
                payload
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|c| i32::from_le_bytes([c[0], c[1], c[2], c[3]]) as f32 / 2_147_483_648.0),
            );
        }
        _ => return false,
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse_all(chunks: &[&str]) -> Vec<TextCommand> {
        let mut p = Parser::new();
        let mut out = Vec::new();
        for c in chunks {
            p.feed(c, &mut out);
        }
        out
    }

    #[test]
    fn parses_multiple_and_split_commands() {
        let cmds = parse_all(&["protocol:ExpertSDR3,2.0;device:SunSDR2DX;vfo:0,0,140", "80000;ready;"]);
        assert_eq!(cmds.len(), 4);
        assert_eq!(
            Event::parse(&cmds[0]),
            Event::Protocol {
                name: "ExpertSDR3".into(),
                version: "2.0".into()
            }
        );
        assert_eq!(Event::parse(&cmds[1]), Event::Device("SunSDR2DX".into()));
        assert_eq!(
            Event::parse(&cmds[2]),
            Event::Vfo {
                trx: 0,
                channel: 0,
                hz: 14_080_000
            }
        );
        assert_eq!(Event::parse(&cmds[3]), Event::Ready);
    }

    #[test]
    fn survives_unknown_and_garbage() {
        let cmds = parse_all(&["SPLIT_ENABLE:0,false;;  \r\n;weird;trx:0;modulation:0,DIGU;"]);
        assert_eq!(cmds.len(), 4);
        assert!(matches!(Event::parse(&cmds[0]), Event::Other(_)));
        assert_eq!(cmds[0].name, "split_enable");
        assert!(matches!(Event::parse(&cmds[1]), Event::Other(_)));
        assert!(matches!(Event::parse(&cmds[2]), Event::Other(_))); // malformed trx
        assert_eq!(
            Event::parse(&cmds[3]),
            Event::Modulation {
                trx: 0,
                mode: "digu".into()
            }
        );
    }

    #[test]
    fn state_tracks_events() {
        let mut s = RadioState::default();
        for line in [
            "trx_count:2;",
            "vfo:1,0,7040000;",
            "trx:0,true;",
            "drive:0,45;",
            "modulations_list:am,usb,digu;",
            "ready;",
        ] {
            for c in parse_all(&[line]) {
                s.apply(&Event::parse(&c));
            }
        }
        assert_eq!(s.trx_count, 2);
        assert_eq!(s.dial_hz(1), Some(7_040_000));
        assert_eq!(s.trx, vec![true]);
        assert_eq!(s.drive, vec![Some(45)]);
        assert_eq!(s.modulations, vec!["am", "usb", "digu"]);
        assert!(s.ready);
    }

    #[test]
    fn drive_is_per_transceiver() {
        // As ExpertSDR3 2.0 sends it: TRX 0 at 0 %, TRX 1 at 100 %.
        let mut s = RadioState::default();
        for c in parse_all(&["drive:0,0;tune_drive:0,0;drive:1,100;"]) {
            s.apply(&Event::parse(&c));
        }
        assert_eq!(s.drive, vec![Some(0), Some(100)]);
        let one = parse_all(&["drive:40;"]);
        assert_eq!(Event::parse(&one[0]), Event::Drive { trx: 0, value: 40 });
    }

    #[test]
    fn command_wire_format() {
        assert_eq!(
            Command::Vfo {
                trx: 0,
                channel: 0,
                hz: 14_080_000
            }
            .to_wire(),
            "vfo:0,0,14080000;"
        );
        assert_eq!(
            Command::Trx {
                trx: 0,
                on: true,
                tci_audio: true
            }
            .to_wire(),
            "trx:0,true,tci;"
        );
        assert_eq!(
            Command::Trx {
                trx: 0,
                on: false,
                tci_audio: true
            }
            .to_wire(),
            "trx:0,false;"
        );
        assert_eq!(
            Command::Modulation {
                trx: 0,
                mode: "digu".into()
            }
            .to_wire(),
            "modulation:0,digu;"
        );
        assert_eq!(
            Command::RxFilterBand {
                trx: 0,
                lo: 100,
                hi: 3000
            }
            .to_wire(),
            "rx_filter_band:0,100,3000;"
        );
        assert_eq!(Command::AudioStart(0).to_wire(), "audio_start:0;");
        assert_eq!(Command::Drive { trx: 0, pct: 30 }.to_wire(), "drive:0,30;");
    }

    #[test]
    fn header_round_trip() {
        let h = FrameHeader {
            receiver: 0,
            sample_rate: 48_000,
            format: sample_format::FLOAT32,
            codec: 0,
            crc: 0,
            length: 2048,
            frame_type: frame_type::TX_CHRONO,
            reserved: [0; 9],
        };
        let mut b = Vec::new();
        h.write(&mut b);
        assert_eq!(b.len(), HEADER_LEN);
        assert_eq!(&b[4..8], &48_000u32.to_le_bytes());
        assert_eq!(FrameHeader::parse(&b), Some(h));
        assert_eq!(FrameHeader::parse(&b[..63]), None);
    }

    #[test]
    fn decodes_float_and_int16() {
        let mut out = Vec::new();
        let payload: Vec<u8> = [0.5f32, -0.25].iter().flat_map(|f| f.to_le_bytes()).collect();
        assert!(decode_samples(sample_format::FLOAT32, &payload, &mut out));
        assert_eq!(out, vec![0.5, -0.25]);
        let payload: Vec<u8> = [16384i16, -32768].iter().flat_map(|v| v.to_le_bytes()).collect();
        assert!(decode_samples(sample_format::INT16, &payload, &mut out));
        assert_eq!(out, vec![0.5, -1.0]);
    }
}
