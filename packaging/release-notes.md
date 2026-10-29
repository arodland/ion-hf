**Alpha.** Ion HF has been measured in simulation but not yet proven on the air. It is released for on-air testing; expect rough edges and changes that break compatibility between versions.

## New in this version

- **Ion HF is now just the modem.** The window shows what the modem is doing: S/N, level, rate, the link's counts, the band (one third for a slice level, the whole band otherwise), the waterfall, the latest bursts and notes, and whether it is in an ARQ link or sending to a broadcast group. Chat, files and nets are gone from Ion; use a host program on Ion's VARA-style ports: Ragtime for chat and files, Pat or Winlink Express for Winlink.
- **Broadcast groups** for Ragtime's group chat, as Data2G's host offers them (`BCAST OPEN`, KISS frames on port 8100).
- **Disconnect** sends what is queued before closing (30 s at most).
- **`ionhf --sim`** runs several Ion stations with no window and no radio, linked through a simulated HF channel, for testing host programs.
- A host port that is busy at start no longer switches the host off in the settings.

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
2. Try it without a radio first: `ionhf --loopback --connect LOOP` calls a simulated station that sends back what it receives; or point a host program at Ion with `--loopback` and connect to `LOOP`.
3. On the air, Ion sends a CW ID at the end of every link and every 9.5 minutes during one. *Esc* or *Abort* stops transmitting at any time.

Ion answers calls on its own while no link is up. All stations in a test need the same Ion HF version.

## Report problems

Open an issue with what you did, what happened, your system and radio, and `ion.log` from the settings folder (see `README.txt` in the download).

## Licence

Ion HF is now licensed under Apache-2.0 (LICENSE, with the credit in NOTICE). Releases up to 0.1.0-alpha.4 were GPL-3.0-or-later and stay so.
