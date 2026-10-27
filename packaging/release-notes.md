**Alpha.** Ion HF has been measured in simulation but not yet proven on the air. It is released for on-air testing; expect rough edges and changes that break compatibility between versions.

## Download

| System | File |
|---|---|
| Windows 10/11 | `IonHF-…-windows.zip`: unzip, run `ionhf.exe` |
| macOS 11+ (Apple silicon and Intel) | `IonHF-…-macos.dmg`: drag Ion HF to Applications |
| Debian 12+, Ubuntu 22.04+ | `ionhf_…_amd64.deb`: `sudo apt install ./ionhf_…_amd64.deb` |
| Other Linux (x86_64) | `IonHF-…-linux-x86_64.tar.gz`: unpack, run `ionhf` |

`SHA256SUMS.txt` lists each file's checksum.

The builds are not signed:

- **Windows:** at the SmartScreen warning choose *More info*, then *Run anyway*.
- **macOS:** the first time, right-click Ion HF and choose *Open*. Allow the microphone when asked; that is how Ion hears the radio.

## First run

1. **Setup:** your call, then the radio link (TCI, CI-V, CAT, rigctld, or a sound card with RTS/DTR/VOX keying) and the audio devices.
2. Try it without a radio first: start it with `--loopback` and connect to `LOOP`.
3. On the air, Ion sends a CW ID at the end of every link and every 9.5 minutes during one. *Esc* or *Abort* stops transmitting at any time.

All stations in a test need the same Ion HF version.

## Report problems

Open an issue with what you did, what happened, your system and radio, and `ion.log` from the settings folder (see `README.txt` in the download).
