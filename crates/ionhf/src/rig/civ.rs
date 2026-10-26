//! Icom CI-V, from Icom's published CI-V reference manuals.
//!
//! A frame is `FE FE <to> <from> <cmd> [sub] [data] FD`; we are the
//! controller at E0h. The radio answers `FB` (OK), `FA` (NG) or the data
//! asked for. On the one-wire CI-V bus (and over USB with "CI-V USB Echo
//! Back" on) the radio also echoes our own frame, and with CI-V Transceive
//! on it broadcasts changes to address 00h: both are skipped.
//!
//! | what              | send                 | reply                       |
//! |-------------------|----------------------|-----------------------------|
//! | read frequency    | `03`                 | `03` + 5 bytes BCD, LSB first |
//! | read mode         | `04`                 | `04 <mode> <filter 1-3>`    |
//! | read data mode    | `1A 06`              | `1A 06 <0/1-3> <filter>`    |
//! | read filter width | `1A 03`              | `1A 03 <index, BCD>`        |
//! | PTT               | `1C 00 <00/01>`      | `FB`                        |
//! | RF power          | `14 0A` + 2 bytes BCD 0000-0255 | `FB`             |

use std::time::{Duration, Instant};

use super::{Line, ModeInfo, Port, Rig, RigError, RigResult, pct_to_scale, scale_to_pct};

pub const CONTROLLER: u8 = 0xE0;
const PREAMBLE: u8 = 0xFE;
const END: u8 = 0xFD;
const OK: u8 = 0xFB;
const NG: u8 = 0xFA;
const REPLY_TIMEOUT: Duration = Duration::from_millis(300);

/// Icom models and their factory CI-V addresses.
pub const MODELS: &[(&str, u8)] = &[
    ("IC-7300", 0x94),
    ("IC-705", 0xA4),
    ("IC-7100", 0x88),
    ("IC-7610", 0x98),
    ("IC-9700", 0xA2),
    ("IC-7600", 0x7A),
    ("IC-7851", 0x8E),
    ("IC-9100", 0x7C),
    ("IC-7410", 0x80),
];

/// `v` as `n` bytes of BCD, least significant pair first.
pub fn bcd_le(mut v: u64, n: usize) -> Vec<u8> {
    (0..n)
        .map(|_| {
            let lo = (v % 10) as u8;
            v /= 10;
            let hi = (v % 10) as u8;
            v /= 10;
            (hi << 4) | lo
        })
        .collect()
}

pub fn from_bcd_le(bytes: &[u8]) -> Option<u64> {
    let mut v = 0u64;
    for &b in bytes.iter().rev() {
        let (hi, lo) = (b >> 4, b & 0x0F);
        if hi > 9 || lo > 9 {
            return None;
        }
        v = v * 100 + (hi * 10 + lo) as u64;
    }
    Some(v)
}

/// Big-endian BCD (levels such as RF power: 0255 = `02 55`).
pub fn bcd_be(v: u32, n: usize) -> Vec<u8> {
    let mut out = bcd_le(v as u64, n);
    out.reverse();
    out
}

pub fn from_bcd_be(bytes: &[u8]) -> Option<u32> {
    let rev: Vec<u8> = bytes.iter().rev().copied().collect();
    from_bcd_le(&rev).map(|v| v as u32)
}

pub fn frame(to: u8, from: u8, body: &[u8]) -> Vec<u8> {
    let mut f = vec![PREAMBLE, PREAMBLE, to, from];
    f.extend_from_slice(body);
    f.push(END);
    f
}

/// A parsed frame: (to, from, body without the FD).
#[derive(Debug, PartialEq)]
pub struct Frame {
    pub to: u8,
    pub from: u8,
    pub body: Vec<u8>,
}

/// Takes every complete frame off the front of `buf`; junk before a
/// preamble is dropped.
pub fn take_frames(buf: &mut Vec<u8>) -> Vec<Frame> {
    let mut out = Vec::new();
    loop {
        let Some(start) = buf.windows(2).position(|w| w == [PREAMBLE, PREAMBLE]) else {
            // Keep a lone trailing FE: it may be half a preamble.
            let keep = usize::from(buf.last() == Some(&PREAMBLE));
            buf.drain(..buf.len() - keep);
            return out;
        };
        buf.drain(..start);
        // Extra preamble bytes are allowed.
        let mut i = 2;
        while buf.get(i) == Some(&PREAMBLE) {
            i += 1;
        }
        let Some(end) = buf[i..].iter().position(|&b| b == END) else {
            return out;
        };
        let f = &buf[i..i + end];
        if f.len() >= 2 {
            out.push(Frame {
                to: f[0],
                from: f[1],
                body: f[2..].to_vec(),
            });
        }
        buf.drain(..i + end + 1);
    }
}

/// IC-7300-class filter width index (`1A 03`): 0-9 are 50-500 Hz in 50 Hz
/// steps, 10-40 are 600-3600 Hz in 100 Hz steps (SSB, data and RTTY).
pub fn filter_width_hz(index: u32) -> u32 {
    if index <= 9 {
        (index + 1) * 50
    } else {
        (index - 4) * 100
    }
}

pub fn mode_name(code: u8) -> &'static str {
    match code {
        0x00 => "LSB",
        0x01 => "USB",
        0x02 => "AM",
        0x03 => "CW",
        0x04 => "RTTY",
        0x05 => "FM",
        0x06 => "WFM",
        0x07 => "CW-R",
        0x08 => "RTTY-R",
        0x17 => "DV",
        _ => "?",
    }
}

pub struct Civ {
    port: Box<dyn Port>,
    port_name: String,
    model: String,
    addr: u8,
    buf: Vec<u8>,
    /// Commands the radio answered NG to once (`1A 03`, `1A 06`): not asked again.
    no_width: bool,
    no_data: bool,
}

impl Civ {
    /// Talks to the radio once (reads the frequency) so a wrong port, baud
    /// rate or address fails here with a useful message.
    pub fn new(port: Box<dyn Port>, port_name: &str, model: &str, addr: u8) -> RigResult<Civ> {
        let mut civ = Civ {
            port,
            port_name: port_name.to_string(),
            model: model.to_string(),
            addr,
            buf: Vec::new(),
            no_width: false,
            no_data: false,
        };
        match civ.freq() {
            Ok(_) => Ok(civ),
            Err(RigError::Timeout) => Err(RigError::Link(format!(
                "no CI-V reply on {port_name} from address {addr:02X}h: check the baud rate \
                 (the radio's CI-V Baud Rate), the CI-V address, and that no other program has the port"
            ))),
            Err(e) => Err(e),
        }
    }

    /// Sends `body` and waits for the radio's answer (data, or empty for
    /// FB). One retry: a CI-V bus can lose a frame to a collision.
    fn call(&mut self, body: &[u8]) -> RigResult<Vec<u8>> {
        match self.exchange(body) {
            Err(RigError::Timeout) => self.exchange(body),
            r => r,
        }
    }

    fn exchange(&mut self, body: &[u8]) -> RigResult<Vec<u8>> {
        let _ = self.port.clear_input();
        self.buf.clear();
        self.port.write_all(&frame(self.addr, CONTROLLER, body))?;
        self.port.flush()?;
        let deadline = Instant::now() + REPLY_TIMEOUT;
        let mut chunk = [0u8; 256];
        loop {
            match self.port.read(&mut chunk) {
                Ok(0) => {}
                Ok(n) => self.buf.extend_from_slice(&chunk[..n]),
                Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock) => {}
                Err(e) => return Err(e.into()),
            }
            for f in take_frames(&mut self.buf) {
                // Our own echo, a broadcast, or another controller's traffic.
                if f.to != CONTROLLER || f.from != self.addr {
                    continue;
                }
                match f.body.first() {
                    Some(&OK) => return Ok(Vec::new()),
                    Some(&NG) => {
                        return Err(RigError::Rejected(format!("CI-V {}", hex(body))));
                    }
                    _ => {}
                }
                // A data reply repeats the command (and sub-command).
                let head = if body[0] == 0x1A || body[0] == 0x14 || body[0] == 0x1C {
                    2
                } else {
                    1
                };
                if f.body.len() >= head && f.body[..head] == body[..head] {
                    return Ok(f.body[head..].to_vec());
                }
            }
            if Instant::now() >= deadline {
                return Err(RigError::Timeout);
            }
        }
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(" ")
}

impl Rig for Civ {
    fn describe(&self) -> String {
        format!("{} (CI-V {:02X}h, {})", self.model, self.addr, self.port_name)
    }

    fn freq(&mut self) -> RigResult<i64> {
        let d = self.call(&[0x03])?;
        // 5 bytes up to 9.99 GHz; some VHF/UHF rigs send 6.
        if !(4..=6).contains(&d.len()) {
            return Err(RigError::Protocol(format!("frequency {}", hex(&d))));
        }
        from_bcd_le(&d)
            .map(|v| v as i64)
            .ok_or_else(|| RigError::Protocol(format!("frequency {}", hex(&d))))
    }

    fn set_ptt(&mut self, on: bool) -> RigResult<()> {
        self.call(&[0x1C, 0x00, u8::from(on)]).map(drop)
    }

    fn power(&mut self) -> RigResult<u32> {
        let d = self.call(&[0x14, 0x0A])?;
        from_bcd_be(&d)
            .map(|v| scale_to_pct(v, 255))
            .ok_or_else(|| RigError::Protocol(format!("RF power {}", hex(&d))))
    }

    fn set_power(&mut self, pct: u32) -> RigResult<()> {
        let mut body = vec![0x14, 0x0A];
        body.extend(bcd_be(pct_to_scale(pct, 255), 2));
        self.call(&body).map(drop)
    }

    fn mode(&mut self) -> RigResult<ModeInfo> {
        let d = self.call(&[0x04])?;
        let code = *d.first().ok_or_else(|| RigError::Protocol("empty mode reply".into()))?;
        let mut name = mode_name(code).to_string();
        if !self.no_data {
            match self.call(&[0x1A, 0x06]) {
                Ok(dd) if dd.first().is_some_and(|&x| x != 0) => name.push_str("-D"),
                Ok(_) => {}
                Err(RigError::Rejected(_)) => self.no_data = true,
                Err(e) => return Err(e),
            }
        }
        let mut width_hz = None;
        if !self.no_width {
            match self.call(&[0x1A, 0x03]) {
                Ok(dd) => width_hz = from_bcd_be(&dd).map(filter_width_hz),
                Err(RigError::Rejected(_)) => self.no_width = true,
                Err(e) => return Err(e),
            }
        }
        if width_hz.is_none()
            && let Some(&fil) = d.get(1)
        {
            name.push_str(&format!(" FIL{fil}"));
        }
        Ok(ModeInfo {
            name,
            usb: code == 0x01,
            width_hz,
        })
    }

    fn set_line(&mut self, line: Line, on: bool) -> RigResult<()> {
        self.port.set_line(line, on).map_err(RigError::from)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::emu::{CivEmu, EmuPort};

    fn rig(emu: CivEmu) -> (Civ, EmuPort<CivEmu>) {
        let port = EmuPort::new(emu);
        let civ = Civ::new(Box::new(port.clone()), "EMU", "IC-7300", 0x94).unwrap();
        (civ, port)
    }

    #[test]
    fn bcd_both_ways() {
        assert_eq!(bcd_le(14_074_000, 5), vec![0x00, 0x40, 0x07, 0x14, 0x00]);
        assert_eq!(from_bcd_le(&[0x00, 0x40, 0x07, 0x14, 0x00]), Some(14_074_000));
        assert_eq!(bcd_be(255, 2), vec![0x02, 0x55]);
        assert_eq!(from_bcd_be(&[0x01, 0x28]), Some(128));
        assert_eq!(from_bcd_le(&[0x1A]), None);
    }

    #[test]
    fn frames_survive_junk_echo_and_splits() {
        let mut buf = vec![0x00, 0x13];
        buf.extend(frame(0x94, 0xE0, &[0x03]));
        buf.extend(frame(0xE0, 0x94, &[0x03, 0, 0x40, 0x07, 0x14, 0]));
        buf.extend([0xFE, 0xFE, 0xE0]);
        let fs = take_frames(&mut buf);
        assert_eq!(fs.len(), 2);
        assert_eq!(fs[0].to, 0x94);
        assert_eq!(fs[1].body[0], 0x03);
        assert_eq!(buf, vec![0xFE, 0xFE, 0xE0]);
        buf.extend([0x94, 0xFB, 0xFD]);
        assert_eq!(take_frames(&mut buf)[0].body, vec![0xFB]);
    }

    #[test]
    fn filter_index_table() {
        assert_eq!(filter_width_hz(0), 50);
        assert_eq!(filter_width_hz(9), 500);
        assert_eq!(filter_width_hz(10), 600);
        assert_eq!(filter_width_hz(40), 3600);
    }

    #[test]
    fn frequency_ptt_and_power() {
        let (mut civ, port) = rig(CivEmu::new(0x94));
        assert_eq!(civ.freq().unwrap(), 14_074_000);

        civ.set_power(40).unwrap();
        assert_eq!(port.with(|e| e.power), 102);
        assert_eq!(civ.power().unwrap(), 40);

        assert!(!port.with(|e| e.ptt));
        civ.set_ptt(true).unwrap();
        assert!(port.with(|e| e.ptt));
        civ.set_ptt(false).unwrap();
        assert!(!port.with(|e| e.ptt));
    }

    #[test]
    fn mode_data_and_filter() {
        let (mut civ, port) = rig(CivEmu::new(0x94));
        let m = civ.mode().unwrap();
        assert_eq!(m.name, "USB-D");
        assert_eq!(m.width_hz, Some(3000));
        assert_eq!(m.problem(), None);
        port.with(|e| {
            e.mode = 0x00;
            e.data = 0;
            e.width_index = 32; // 2.8 kHz
        });
        let m = civ.mode().unwrap();
        assert_eq!(m.name, "LSB");
        assert!(m.problem().unwrap().contains("LSB"));
        port.with(|e| {
            e.mode = 0x01;
            e.width_index = 24; // 2.0 kHz
        });
        assert!(civ.mode().unwrap().problem().unwrap().contains("2000"));
    }

    #[test]
    fn older_rigs_without_width_or_data_commands() {
        let mut emu = CivEmu::new(0x88);
        emu.supports_1a = false;
        let port = EmuPort::new(emu);
        let mut civ = Civ::new(Box::new(port.clone()), "EMU", "IC-7100", 0x88).unwrap();
        let m = civ.mode().unwrap();
        assert_eq!(m.name, "USB FIL1");
        assert_eq!(m.width_hz, None);
        assert_eq!(m.problem(), None);
        // NG once is remembered: the second read asks only for the mode.
        let before = port.with(|e| e.frames_seen);
        civ.mode().unwrap();
        assert_eq!(port.with(|e| e.frames_seen) - before, 1);
    }

    #[test]
    fn echo_and_transceive_broadcasts_are_skipped() {
        let mut emu = CivEmu::new(0x94);
        emu.echo = true;
        emu.broadcast_noise = true;
        let (mut civ, _port) = rig(emu);
        assert_eq!(civ.freq().unwrap(), 14_074_000);
        civ.set_power(100).unwrap();
        assert_eq!(civ.power().unwrap(), 100);
    }

    #[test]
    fn ng_is_rejected_and_wrong_address_times_out_with_advice() {
        let (mut civ, _port) = rig(CivEmu::new(0x94));
        // The emulator answers NG to commands it does not know.
        assert!(matches!(civ.call(&[0x99]), Err(RigError::Rejected(_))));

        let port = EmuPort::new(CivEmu::new(0x94));
        let err = Civ::new(Box::new(port), "EMU", "IC-705", 0xA4).err().unwrap();
        assert!(err.to_string().contains("A4h"), "{err}");
    }

    #[test]
    fn reading_never_keys() {
        let (mut civ, port) = rig(CivEmu::new(0x94));
        civ.freq().unwrap();
        civ.mode().unwrap();
        civ.power().unwrap();
        assert!(port.with(|e| e.log.iter().all(|b| b.first() != Some(&0x1C))));
        assert!(!port.with(|e| e.ptt));
    }
}
