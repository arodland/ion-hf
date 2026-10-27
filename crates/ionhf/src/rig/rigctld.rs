//! Hamlib's `rigctld` network daemon, through its documented text protocol
//! (the `rigctld(1)` manual). Every command goes in the extended form (`+f`)
//! so each reply ends with an `RPRT n` line and parsing never guesses where
//! a reply stops:
//!
//! ```text
//! +f            get_freq:\nFrequency: 14074000\nRPRT 0\n
//! +T 1          set_ptt: 1\nRPRT 0\n
//! +m            get_mode:\nMode: PKTUSB\nPassband: 3000\nRPRT 0\n
//! +l RFPOWER    get_level: RFPOWER\nRFPOWER: 0.500000\nRPRT 0\n
//! +L RFPOWER 0.5
//! ```

use std::io::{BufRead, BufReader, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::Duration;

use super::{ModeInfo, Rig, RigError, RigResult};
use crate::config::TxAudio;

pub const DEFAULT_ADDR: &str = "127.0.0.1:4532";

pub struct Rigctld {
    addr: String,
    out: TcpStream,
    input: BufReader<TcpStream>,
    /// A reply was cut short: the stream is out of step and must reconnect.
    broken: bool,
    tx_audio: TxAudio,
}

/// What to tell the user when the daemon is not there.
pub fn not_running(addr: &str) -> String {
    format!(
        "rigctld is not running at {addr}: start it with `rigctld -m <model> -r <port>` \
         (`rigctl -l` lists model numbers, e.g. `rigctld -m 3073 -r COM3 -s 19200` for an IC-7300)"
    )
}

impl Rigctld {
    pub fn with_tx_audio(mut self, audio: TxAudio) -> Rigctld {
        self.tx_audio = audio;
        self
    }

    pub fn connect(addr: &str) -> RigResult<Rigctld> {
        let sock: SocketAddr = addr
            .to_socket_addrs()
            .map_err(|e| RigError::Link(format!("rigctld address {addr}: {e}")))?
            .next()
            .ok_or_else(|| RigError::Link(format!("rigctld address {addr}: no such host")))?;
        let out = TcpStream::connect_timeout(&sock, Duration::from_millis(1500)).map_err(|e| match e.kind() {
            std::io::ErrorKind::ConnectionRefused | std::io::ErrorKind::TimedOut => RigError::Link(not_running(addr)),
            _ => RigError::Link(format!("rigctld at {addr}: {e}")),
        })?;
        out.set_nodelay(true)?;
        out.set_read_timeout(Some(Duration::from_secs(2)))?;
        out.set_write_timeout(Some(Duration::from_secs(2)))?;
        let input = BufReader::new(out.try_clone()?);
        Ok(Rigctld {
            addr: addr.to_string(),
            out,
            input,
            broken: false,
            tx_audio: TxAudio::Data,
        })
    }

    /// Sends `+cmd` and returns the value lines between the header and
    /// `RPRT 0`.
    fn call(&mut self, cmd: &str) -> RigResult<Vec<String>> {
        if self.broken {
            return Err(RigError::Link(format!("rigctld at {}: connection lost", self.addr)));
        }
        let result = self.exchange(cmd);
        if let Err(RigError::Link(_) | RigError::Timeout | RigError::Protocol(_)) = &result {
            self.broken = true;
        }
        result
    }

    fn exchange(&mut self, cmd: &str) -> RigResult<Vec<String>> {
        self.out.write_all(format!("+{cmd}\n").as_bytes())?;
        let mut lines = Vec::new();
        loop {
            let mut line = String::new();
            let n = self.input.read_line(&mut line)?;
            if n == 0 {
                return Err(RigError::Link(format!(
                    "rigctld at {} closed the connection",
                    self.addr
                )));
            }
            let line = line.trim_end_matches(['\r', '\n']);
            if let Some(code) = line.strip_prefix("RPRT ") {
                let code: i32 = code.trim().parse().map_err(|_| RigError::Protocol(line.to_string()))?;
                return match code {
                    0 => Ok(lines.into_iter().skip(1).collect()),
                    // ENIMPL, ENAVAIL: this rig has no such function.
                    -4 | -11 => Err(RigError::Unsupported),
                    -5 => Err(RigError::Timeout),
                    n => Err(RigError::Rejected(format!("{cmd}: Hamlib error {n}"))),
                };
            }
            if lines.len() > 32 {
                return Err(RigError::Protocol(format!("{cmd}: no RPRT")));
            }
            lines.push(line.to_string());
        }
    }

    /// The value of `Key: value` (or a bare value) on line `i`.
    fn value(lines: &[String], i: usize, cmd: &str) -> RigResult<String> {
        let line = lines
            .get(i)
            .ok_or_else(|| RigError::Protocol(format!("{cmd}: short reply")))?;
        Ok(match line.rsplit_once(':') {
            Some((_, v)) => v.trim().to_string(),
            None => line.trim().to_string(),
        })
    }
}

pub fn parse_mode(name: &str, passband: Option<u32>) -> ModeInfo {
    ModeInfo {
        name: name.to_string(),
        usb: matches!(name, "USB" | "PKTUSB"),
        width_hz: passband,
    }
}

impl Rig for Rigctld {
    fn describe(&self) -> String {
        format!("rigctld {}", self.addr)
    }

    fn freq(&mut self) -> RigResult<i64> {
        let lines = self.call("f")?;
        let v = Self::value(&lines, 0, "f")?;
        // Hamlib prints whole hertz, some builds as "14074000.000000".
        v.parse::<f64>()
            .map(|hz| hz.round() as i64)
            .map_err(|_| RigError::Protocol(format!("frequency {v:?}")))
    }

    /// `T 3` is Hamlib's RIG_PTT_ON_DATA (the data input on a rig with
    /// both); a rig that refuses it gets `T 1`.
    fn set_ptt(&mut self, on: bool) -> RigResult<()> {
        if on && self.tx_audio == TxAudio::Data {
            match self.call("T 3") {
                Err(RigError::Rejected(_) | RigError::Unsupported) => self.tx_audio = TxAudio::Mic,
                r => return r.map(drop),
            }
        }
        self.call(&format!("T {}", u8::from(on))).map(drop)
    }

    fn power(&mut self) -> RigResult<u32> {
        let lines = self.call("l RFPOWER")?;
        let v = Self::value(&lines, 0, "l RFPOWER")?;
        let f: f32 = v.parse().map_err(|_| RigError::Protocol(format!("RFPOWER {v:?}")))?;
        Ok((f.clamp(0.0, 1.0) * 100.0).round() as u32)
    }

    fn set_power(&mut self, pct: u32) -> RigResult<()> {
        let f = pct.min(100) as f32 / 100.0;
        self.call(&format!("L RFPOWER {f:.2}")).map(drop)
    }

    fn mode(&mut self) -> RigResult<ModeInfo> {
        let lines = self.call("m")?;
        let name = Self::value(&lines, 0, "m")?;
        let passband = Self::value(&lines, 1, "m").ok().and_then(|p| p.parse().ok());
        Ok(parse_mode(&name, passband))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::emu::FakeRigctld;

    #[test]
    fn frequency_ptt_power_and_mode_round_trip() {
        let fake = FakeRigctld::start();
        let mut rig = Rigctld::connect(&fake.addr()).unwrap();
        assert_eq!(rig.freq().unwrap(), 14_074_000);
        rig.set_power(35).unwrap();
        assert_eq!(rig.power().unwrap(), 35);
        let m = rig.mode().unwrap();
        assert_eq!(m.name, "PKTUSB");
        assert_eq!(m.width_hz, Some(3000));
        assert_eq!(m.problem(), None);
        assert!(!fake.state().ptt);
        rig.set_ptt(true).unwrap();
        assert!(fake.state().ptt && fake.state().data_ptt);
        rig.set_ptt(false).unwrap();
        assert!(!fake.state().ptt);
    }

    #[test]
    fn data_ptt_falls_back_to_plain_ptt() {
        let fake = FakeRigctld::start();
        fake.state_mut(|s| s.no_data_ptt = true);
        let mut rig = Rigctld::connect(&fake.addr()).unwrap();
        rig.set_ptt(true).unwrap();
        assert!(fake.state().ptt && !fake.state().data_ptt);
        rig.set_ptt(false).unwrap();
        assert!(!fake.state().ptt);
    }

    #[test]
    fn mic_ptt_is_plain_ptt() {
        let fake = FakeRigctld::start();
        let mut rig = Rigctld::connect(&fake.addr()).unwrap().with_tx_audio(TxAudio::Mic);
        rig.set_ptt(true).unwrap();
        assert!(fake.state().ptt && !fake.state().data_ptt);
    }

    #[test]
    fn errors_map_to_unsupported_and_rejected() {
        let fake = FakeRigctld::start();
        fake.state_mut(|s| s.no_power = true);
        let mut rig = Rigctld::connect(&fake.addr()).unwrap();
        assert_eq!(rig.power(), Err(RigError::Unsupported));
        assert_eq!(rig.set_power(50), Err(RigError::Unsupported));
        // An out-of-range frequency is refused, and the link stays usable.
        assert!(matches!(rig.call("F -5"), Err(RigError::Rejected(_))));
        assert_eq!(rig.freq().unwrap(), 14_074_000);
    }

    #[test]
    fn wrong_mode_is_a_problem() {
        let fake = FakeRigctld::start();
        fake.state_mut(|s| {
            s.mode = "LSB".into();
            s.passband = 2400;
        });
        let mut rig = Rigctld::connect(&fake.addr()).unwrap();
        assert!(rig.mode().unwrap().problem().unwrap().contains("LSB"));
    }

    #[test]
    fn daemon_not_running_says_how_to_start_it() {
        // Bind and close a port so nothing is listening there.
        let addr = {
            let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            l.local_addr().unwrap().to_string()
        };
        let err = Rigctld::connect(&addr).err().unwrap();
        assert!(err.to_string().contains("rigctld -m"), "{err}");
    }

    #[test]
    fn daemon_going_away_is_a_link_error() {
        let fake = FakeRigctld::start();
        let mut rig = Rigctld::connect(&fake.addr()).unwrap();
        assert!(rig.freq().is_ok());
        fake.stop();
        assert!(matches!(rig.freq(), Err(RigError::Link(_))));
        // Stays failed until the radio thread reconnects.
        assert!(matches!(rig.freq(), Err(RigError::Link(_))));
    }

    /// Against a real Hamlib dummy rig, when Hamlib is installed.
    #[test]
    fn hamlib_dummy_rig_if_installed() {
        let port = {
            let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            l.local_addr().unwrap().port()
        };
        let child = std::process::Command::new("rigctld")
            .args(["-m", "1", "-T", "127.0.0.1", "-t", &port.to_string()])
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .spawn();
        let Ok(mut child) = child else {
            eprintln!("rigctld not installed; skipped");
            return;
        };
        let addr = format!("127.0.0.1:{port}");
        let mut rig = None;
        for _ in 0..50 {
            std::thread::sleep(Duration::from_millis(100));
            if let Ok(r) = Rigctld::connect(&addr) {
                rig = Some(r);
                break;
            }
        }
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut rig = rig.expect("rigctld -m 1 did not start");
            rig.freq().unwrap();
            rig.set_power(40).unwrap();
            assert_eq!(rig.power().unwrap(), 40);
            rig.set_ptt(true).unwrap();
            rig.set_ptt(false).unwrap();
            rig.mode().unwrap();
        }));
        let _ = child.kill();
        let _ = child.wait();
        result.unwrap();
    }
}
