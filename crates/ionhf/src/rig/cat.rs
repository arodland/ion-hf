//! The Kenwood-style ASCII CAT family, from the published PC command
//! references of Kenwood (TS-590/890/990/480/2000), Elecraft (K3/KX3/KX2/K4
//! Programmer's References) and Yaesu (FT-991A/891/710, FTDX10/101 CAT
//! Operation Reference Manuals). Commands are two letters, parameters, `;`.
//! A read is the bare command (`FA;`); the radio answers with the command
//! and its value (`FA00014074000;`). Sets get no answer; a bad command gets
//! `?;`.
//!
//! What differs by brand:
//!
//! | | Kenwood | Elecraft | Yaesu |
//! |---|---|---|---|
//! | PTT on / off | `TX1;` (data send) or `TX;` (mic) / `RX;` | `TX;` / `RX;` | `TX1;` / `TX0;` |
//! | mode | `MD;` → `MD2;` + `DA;` | `MD;` + `DT;` | `MD0;` → `MD0C;` (hex) |
//! | filter | – | `BW;` (10 Hz units) | – |
//! | power `PC` | watts | watts | watts |
//!
//! Note `TX;` on Kenwood and Elecraft *keys the radio* (on Yaesu it is a
//! read): nothing here sends it except [`Rig::set_ptt`].

use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

use super::{Line, ModeInfo, Port, Rig, RigError, RigResult};
use crate::config::TxAudio;

const REPLY_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Brand {
    Kenwood,
    Elecraft,
    Yaesu,
}

impl Brand {
    pub fn name(self) -> &'static str {
        match self {
            Brand::Kenwood => "Kenwood",
            Brand::Elecraft => "Elecraft",
            Brand::Yaesu => "Yaesu",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Model {
    pub name: &'static str,
    pub brand: Brand,
    /// PTT-on command.
    pub tx: &'static str,
    /// Reads the data sub-mode (`DA;` Kenwood, `DT;` Elecraft).
    pub data: Option<&'static str>,
    /// `BW;` reads the filter width (Elecraft).
    pub bw: bool,
    pub max_watts: u32,
    pub min_watts: u32,
    /// Factory (or usual) CAT baud rate; must match the radio's menu.
    pub baud: u32,
}

const fn kenwood(name: &'static str, data: bool, max: u32, baud: u32) -> Model {
    Model {
        name,
        brand: Brand::Kenwood,
        tx: "TX;",
        data: if data { Some("DA;") } else { None },
        bw: false,
        max_watts: max,
        min_watts: 5,
        baud,
    }
}

const fn elecraft(name: &'static str, max: u32) -> Model {
    Model {
        name,
        brand: Brand::Elecraft,
        tx: "TX;",
        data: Some("DT;"),
        bw: true,
        max_watts: max,
        min_watts: 0,
        baud: 38_400,
    }
}

const fn yaesu(name: &'static str, max: u32) -> Model {
    Model {
        name,
        brand: Brand::Yaesu,
        tx: "TX1;",
        data: None,
        bw: false,
        max_watts: max,
        min_watts: 5,
        baud: 38_400,
    }
}

pub const MODELS: &[Model] = &[
    kenwood("TS-590S/SG", true, 100, 115_200),
    kenwood("TS-890S", true, 100, 115_200),
    kenwood("TS-990S", true, 200, 115_200),
    kenwood("TS-480", false, 100, 9_600),
    kenwood("TS-2000", false, 100, 9_600),
    elecraft("K3/K3S", 100),
    elecraft("K4", 100),
    elecraft("KX3", 15),
    elecraft("KX2", 10),
    yaesu("FT-991/991A", 100),
    yaesu("FT-891", 100),
    yaesu("FT-710", 100),
    yaesu("FTDX10", 100),
    yaesu("FTDX101D", 100),
    yaesu("FTDX101MP", 200),
];

pub fn model(name: &str) -> Option<&'static Model> {
    MODELS.iter().find(|m| m.name == name)
}

impl Model {
    /// On a Kenwood, `TX1;` is DATA SEND: transmit audio from USB/ACC2,
    /// not the mic.
    pub fn ptt_on(&self, audio: TxAudio) -> &'static str {
        match (self.brand, audio) {
            (Brand::Kenwood, TxAudio::Data) => "TX1;",
            _ => self.tx,
        }
    }

    pub fn ptt_off(&self) -> &'static str {
        if self.brand == Brand::Yaesu { "TX0;" } else { "RX;" }
    }

    fn mode_query(&self) -> &'static str {
        if self.brand == Brand::Yaesu { "MD0;" } else { "MD;" }
    }

    /// Yaesu wants 2 stop bits on its older menus; 2 is harmless to the
    /// rest, but only Yaesu needs it.
    pub fn stop_bits(&self) -> u8 {
        if self.brand == Brand::Yaesu { 2 } else { 1 }
    }

    pub fn watts(&self, pct: u32) -> u32 {
        let w = (pct.min(100) * self.max_watts + 50) / 100;
        w.clamp(self.min_watts, self.max_watts)
    }

    pub fn pct(&self, watts: u32) -> u32 {
        ((watts.min(self.max_watts) * 100 + self.max_watts / 2) / self.max_watts.max(1)).min(100)
    }
}

/// Mode name and whether it is upper sideband, from the `MD` code (plus
/// the data sub-mode where the radio has one).
pub fn mode_info(brand: Brand, code: char, data: Option<u32>) -> (String, bool) {
    let (name, usb) = match brand {
        Brand::Kenwood => match code {
            '1' => ("LSB", false),
            '2' => ("USB", true),
            '3' => ("CW", false),
            '4' => ("FM", false),
            '5' => ("AM", false),
            '6' => ("FSK", false),
            '7' => ("CW-R", false),
            '9' => ("FSK-R", false),
            _ => ("?", false),
        },
        Brand::Elecraft => match (code, data) {
            ('1', _) => ("LSB", false),
            ('2', _) => ("USB", true),
            ('3', _) => ("CW", false),
            ('4', _) => ("FM", false),
            ('5', _) => ("AM", false),
            // DATA A is the sound-card data mode; AFSK A, FSK D and
            // PSK D use the radio's own encoders.
            ('6', Some(0) | None) => ("DATA A", true),
            ('6', Some(1)) => ("AFSK A", false),
            ('6', Some(2)) => ("FSK D", false),
            ('6', Some(3)) => ("PSK D", false),
            ('7', _) => ("CW-R", false),
            ('9', _) => ("DATA-R", false),
            _ => ("?", false),
        },
        Brand::Yaesu => match code.to_ascii_uppercase() {
            '1' => ("LSB", false),
            '2' => ("USB", true),
            '3' => ("CW-U", false),
            '4' => ("FM", false),
            '5' => ("AM", false),
            '6' => ("RTTY-L", false),
            '7' => ("CW-L", false),
            '8' => ("DATA-L", false),
            '9' => ("RTTY-U", false),
            'A' => ("DATA-FM", false),
            'B' => ("FM-N", false),
            'C' => ("DATA-U", true),
            'D' => ("AM-N", false),
            _ => ("?", false),
        },
    };
    let mut name = name.to_string();
    if brand == Brand::Kenwood && usb && data.is_some_and(|d| d != 0) {
        name.push_str("-DATA");
    }
    (name, usb)
}

pub struct Cat {
    port: Box<dyn Port>,
    port_name: String,
    model: &'static Model,
    buf: Vec<u8>,
    no_data: bool,
    no_bw: bool,
    tx_audio: TxAudio,
}

fn is_error(tok: &str) -> bool {
    matches!(tok, "?" | "E" | "O")
}

impl Cat {
    /// Reads the frequency once so a wrong port or baud rate fails here.
    pub fn new(port: Box<dyn Port>, port_name: &str, model: &'static Model) -> RigResult<Cat> {
        let mut cat = Cat {
            port,
            port_name: port_name.to_string(),
            model,
            buf: Vec::new(),
            no_data: false,
            no_bw: false,
            tx_audio: TxAudio::Data,
        };
        match cat.freq() {
            Ok(_) => Ok(cat),
            Err(RigError::Timeout) => Err(RigError::Link(format!(
                "no CAT reply on {port_name}: check the baud rate (the radio's CAT/COM rate menu) \
                 and that no other program has the port"
            ))),
            Err(e) => Err(e),
        }
    }

    pub fn with_tx_audio(mut self, audio: TxAudio) -> Cat {
        self.tx_audio = audio;
        self
    }

    /// Next `;`-terminated token, without the `;`.
    fn next_token(&mut self, deadline: Instant) -> RigResult<String> {
        let mut chunk = [0u8; 128];
        loop {
            if let Some(i) = self.buf.iter().position(|&b| b == b';') {
                let tok: Vec<u8> = self.buf.drain(..=i).collect();
                let s = String::from_utf8_lossy(&tok[..tok.len() - 1]);
                // Some radios put CR/LF or spaces between replies.
                return Ok(s.trim().to_string());
            }
            if Instant::now() >= deadline {
                return Err(RigError::Timeout);
            }
            match self.port.read(&mut chunk) {
                Ok(n) => self.buf.extend_from_slice(&chunk[..n]),
                Err(e) if matches!(e.kind(), std::io::ErrorKind::TimedOut | std::io::ErrorKind::WouldBlock) => {}
                Err(e) => return Err(e.into()),
            }
        }
    }

    /// Sends `pre` (sets, possibly empty) then the read `q`, and returns
    /// the value of the answer to `q`. A `?;` before it means the radio
    /// refused something.
    fn transact(&mut self, pre: &str, q: &str) -> RigResult<String> {
        let _ = self.port.clear_input();
        self.buf.clear();
        self.port.write_all(pre.as_bytes())?;
        self.port.write_all(q.as_bytes())?;
        self.port.flush()?;
        let prefix = q.trim_end_matches(';');
        let deadline = Instant::now() + REPLY_TIMEOUT;
        let mut refused = false;
        loop {
            let tok = self.next_token(deadline)?;
            if is_error(&tok) {
                if refused || pre.is_empty() {
                    return Err(RigError::Rejected(format!("{pre}{q}")));
                }
                // `pre` was refused; the read may still answer.
                refused = true;
                continue;
            }
            // Anything else is Auto Information traffic: skip it.
            if let Some(v) = tok.strip_prefix(prefix) {
                if refused {
                    return Err(RigError::Rejected(pre.to_string()));
                }
                return Ok(v.to_string());
            }
        }
    }

    fn query(&mut self, q: &str) -> RigResult<String> {
        match self.transact("", q) {
            Err(RigError::Timeout) => self.transact("", q),
            r => r,
        }
    }

    /// A set, confirmed by reading the frequency after it.
    fn set(&mut self, cmd: &str) -> RigResult<()> {
        self.transact(cmd, "FA;").map(drop)
    }

    fn number(v: &str, what: &str) -> RigResult<u32> {
        let digits: String = v.chars().take_while(char::is_ascii_digit).collect();
        digits.parse().map_err(|_| RigError::Protocol(format!("{what} {v:?}")))
    }
}

impl Rig for Cat {
    fn describe(&self) -> String {
        format!("{} {} ({})", self.model.brand.name(), self.model.name, self.port_name)
    }

    fn freq(&mut self) -> RigResult<i64> {
        let v = self.query("FA;")?;
        v.parse::<i64>()
            .map_err(|_| RigError::Protocol(format!("frequency {v:?}")))
    }

    fn set_ptt(&mut self, on: bool) -> RigResult<()> {
        if !on {
            return self.set(self.model.ptt_off());
        }
        let cmd = self.model.ptt_on(self.tx_audio);
        match self.set(cmd) {
            // A radio without DATA SEND: key it the one way it has.
            Err(RigError::Rejected(_)) if cmd != self.model.tx => {
                self.tx_audio = TxAudio::Mic;
                self.set(self.model.tx)
            }
            r => r,
        }
    }

    fn power(&mut self) -> RigResult<u32> {
        let v = self.query("PC;")?;
        Ok(self.model.pct(Self::number(&v, "power")?))
    }

    fn set_power(&mut self, pct: u32) -> RigResult<()> {
        let cmd = format!("PC{:03};", self.model.watts(pct));
        self.set(&cmd)
    }

    fn mode(&mut self) -> RigResult<ModeInfo> {
        let q = self.model.mode_query();
        let v = self.query(q)?;
        let code = v
            .chars()
            .next()
            .ok_or_else(|| RigError::Protocol("empty mode reply".into()))?;
        let mut data = None;
        if let Some(dq) = self.model.data.filter(|_| !self.no_data) {
            match self.query(dq) {
                Ok(d) => data = Self::number(&d, "data mode").ok(),
                Err(RigError::Rejected(_)) => self.no_data = true,
                Err(e) => return Err(e),
            }
        }
        let mut width_hz = None;
        if self.model.bw && !self.no_bw {
            match self.query("BW;") {
                Ok(w) => width_hz = Self::number(&w, "bandwidth").ok().map(|x| x * 10),
                Err(RigError::Rejected(_)) => self.no_bw = true,
                Err(e) => return Err(e),
            }
        }
        let (name, usb) = mode_info(self.model.brand, code, data);
        Ok(ModeInfo { name, usb, width_hz })
    }

    fn set_line(&mut self, line: Line, on: bool) -> RigResult<()> {
        self.port.set_line(line, on).map_err(RigError::from)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::emu::{CatEmu, EmuPort};

    fn open(name: &str) -> (Cat, EmuPort<CatEmu>) {
        let m = model(name).unwrap();
        let port = EmuPort::new(CatEmu::new(m.brand));
        let cat = Cat::new(Box::new(port.clone()), "EMU", m).unwrap();
        (cat, port)
    }

    #[test]
    fn watts_mapping() {
        let ts = model("TS-590S/SG").unwrap();
        assert_eq!(ts.watts(50), 50);
        assert_eq!(ts.watts(0), 5); // its minimum
        let kx3 = model("KX3").unwrap();
        assert_eq!(kx3.watts(100), 15);
        assert_eq!(kx3.pct(15), 100);
        let mp = model("FTDX101MP").unwrap();
        assert_eq!(mp.watts(50), 100);
        assert_eq!(mp.pct(100), 50);
    }

    #[test]
    fn every_brand_reads_frequency_and_round_trips_ptt_and_power() {
        for name in ["TS-590S/SG", "TS-2000", "K3/K3S", "KX3", "FT-991/991A", "FTDX101MP"] {
            let (mut cat, port) = open(name);
            let m = model(name).unwrap();
            assert_eq!(cat.freq().unwrap(), 14_074_000, "{name}");

            cat.set_power(50).unwrap();
            assert_eq!(port.with(|e| e.watts), m.watts(50), "{name}");
            assert_eq!(cat.power().unwrap(), m.pct(m.watts(50)), "{name}");

            cat.set_ptt(true).unwrap();
            assert!(port.with(|e| e.ptt), "{name}");
            if m.brand == Brand::Kenwood {
                assert!(port.with(|e| e.data_send), "{name} should key DATA SEND");
            }
            cat.set_ptt(false).unwrap();
            assert!(!port.with(|e| e.ptt), "{name}");
        }
    }

    #[test]
    fn a_kenwood_keys_the_mic_input_when_asked() {
        let (cat, port) = open("TS-480");
        let mut cat = cat.with_tx_audio(TxAudio::Mic);
        cat.set_ptt(true).unwrap();
        assert!(port.with(|e| e.ptt && !e.data_send));
        cat.set_ptt(false).unwrap();
        assert!(!port.with(|e| e.ptt));
    }

    #[test]
    fn modes_per_brand() {
        let (mut cat, port) = open("TS-590S/SG");
        let m = cat.mode().unwrap();
        assert_eq!(m.name, "USB-DATA");
        assert_eq!(m.problem(), None);
        port.with(|e| e.mode = '1');
        assert!(cat.mode().unwrap().problem().unwrap().contains("LSB"));

        let (mut cat, port) = open("K3/K3S");
        port.with(|e| {
            e.mode = '6';
            e.data = 0;
            e.bw_10hz = 280;
        });
        let m = cat.mode().unwrap();
        assert_eq!((m.name.as_str(), m.width_hz), ("DATA A", Some(2800)));
        assert_eq!(m.problem(), None);
        port.with(|e| e.bw_10hz = 180);
        assert!(cat.mode().unwrap().problem().unwrap().contains("1800"));
        port.with(|e| e.data = 2);
        assert!(cat.mode().unwrap().problem().unwrap().contains("FSK D"));

        let (mut cat, port) = open("FT-991/991A");
        assert_eq!(cat.mode().unwrap().name, "DATA-U");
        port.with(|e| e.mode = '8');
        assert!(cat.mode().unwrap().problem().unwrap().contains("DATA-L"));
    }

    #[test]
    fn refused_commands_and_auto_info_noise() {
        let (mut cat, port) = open("TS-590S/SG");
        port.with(|e| e.no_power = true);
        assert!(matches!(cat.set_power(50), Err(RigError::Rejected(_))));
        assert_eq!(cat.freq().unwrap(), 14_074_000);
        // Unsolicited AI traffic between replies is skipped.
        port.with(|e| {
            e.no_power = false;
            e.auto_info = true;
        });
        cat.set_power(30).unwrap();
        assert_eq!(port.with(|e| e.watts), 30);
        assert_eq!(cat.freq().unwrap(), 14_074_000);
        assert_eq!(cat.mode().unwrap().name, "USB-DATA");
    }

    #[test]
    fn rigs_without_power_control_report_rejected() {
        let (mut cat, port) = open("FT-891");
        port.with(|e| e.no_power = true);
        assert!(matches!(cat.power(), Err(RigError::Rejected(_))));
        assert!(matches!(cat.set_power(30), Err(RigError::Rejected(_))));
    }

    #[test]
    fn silent_port_times_out_with_advice() {
        let m = model("TS-590S/SG").unwrap();
        let mut emu = CatEmu::new(m.brand);
        emu.silent = true;
        let err = Cat::new(Box::new(EmuPort::new(emu)), "EMU", m).err().unwrap();
        assert!(err.to_string().contains("baud"), "{err}");
    }

    #[test]
    fn reading_never_sends_tx() {
        for name in ["TS-590S/SG", "K3/K3S", "FT-991/991A"] {
            let (mut cat, port) = open(name);
            cat.freq().unwrap();
            cat.mode().unwrap();
            cat.power().unwrap();
            let log = port.with(|e| e.log.clone());
            assert!(log.iter().all(|c| !c.starts_with("TX")), "{name}: {log:?}");
            assert!(!port.with(|e| e.ptt));
        }
    }
}
