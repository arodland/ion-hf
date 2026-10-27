# Ion HF, the program

`ionhf` (`crates/ionhf`) is the Ion modem as a desktop program: Winlink
through a VARA-style host interface (Pat, Winlink Express), keyboard links
with chat, files and pictures, and nets. It drives the radio over TCI
(ExpertSDR / SunSDR), Icom CI-V, Kenwood / Elecraft / Yaesu CAT or Hamlib
`rigctld`, or uses a sound card alone with PTT by CAT, RTS, DTR or VOX.

```text
ionhf [--loopback] [--loop-snr DB] [--tci ws://host:port] [--connect CALL]
      [--send TEXT] [--file PATH] [--net NAME] [--tab link|listen|net|setup]
      [--view chat|charts|log] [--mini] [--shots DIR] [--quit S]
```

- `--loopback`: no radio; simulated stations answer (below). `--loop-snr`
  sets the simulated channel's SNR (default 10 dB, the `loopback_snr_db`
  setting).
- `--tci`: use TCI at this address instead of the configured radio.
- `--connect`, `--send`, `--file`: call a station at start, and send the
  text or file once linked. `--net NAME`: run that net as NCS at start.
- `--tab`, `--view`: the tab shown at start, and what the Link tab's centre
  shows. `--mini`: start in the small window.
- `--shots DIR`, `--quit S`: save the window to `DIR` every 15 s and close
  after `S` seconds (screenshots, scripted checks).

## Settings

`ionhf.toml` holds the settings, `window.ron` the window, and `ion.log`
the log (written by a release build started without a terminal):

| System | Folder |
|---|---|
| Windows | `%APPDATA%\Ion HF` |
| macOS | `~/Library/Application Support/Ion HF` |
| Linux | `$XDG_CONFIG_HOME/ionhf`, else `~/.config/ionhf` |

**Portable:** with an `ionhf.toml` (even an empty one) next to the program,
Ion keeps everything in that folder instead.

## Screens

| Tab | What it shows |
|---|---|
| Link | Each side's level and SNR report (Rpt Sent, Rpt Rcvd), Aim (the level the reports allow), Queue, Rate, Data and Time; the band meter (how clean each third of the passband is here). The centre is **Chat** (each line under its sender's call; files and pictures in the conversation with progress, a picture filling in as it arrives; drop a file on the window or pick one with **Browse** to send it), **Charts** (level, SNR and data over the last minutes) or **Log** (every burst sent and heard). |
| Listen | Call and Connect; Recent; Answer (Auto: answer calls whenever no link is up; Off: receive only); Heard (stations heard calling); Band and the bursts heard lately; the log. |
| Net | Start as NCS or Join; Open, Close net, Leave; Caps (Relay, Wide, QSY, Attended); the roster and the traffic. |
| Setup | Station (call, grid), Radio (link type, port, model, sound cards with **Rescan**; applies on restart), Levels (drive and the OFDM level), Keying (PTT, **TX audio** and lead time), Host (the VARA-style ports). |
| Mini | A small window for running behind a Winlink client: state, SNR, level, rate, progress, Abort. |

## Files and pictures

Text and files share the link's byte stream (`crates/ionhf/src/ion_files.rs`).
A file is a header line and its bytes:

```text
0x1E "HF1 " kind " " size " " name "\n" <size bytes>
```

0x1E does not occur in typed text, so the receiving side sorts files back
out of the stream. A picture (jpg, png, bmp, gif, webp) is scaled to fit
320 × 240 and sent as eight horizontal JPEG strips, so the other side sees
it fill in; any other file goes as it is, up to 16 MB. Received files are
saved to `Documents/Ion` (a name already there gets `-2`, `-3`, ...). A
Winlink client on the host ports has the stream to itself and never sees
this framing.

## Winlink host interface

`crates/ionhf/src/host.rs` follows "VARA Protocol Native TNC Commands"
(2022): a command port (default 8300) and a data port one above, on
127.0.0.1 only, one client at a time. `VERSION` is answered
`VERSION IonHF <version>`. Settings that mean nothing to Ion (`BW…`,
`COMPRESSION …`, `CWID …`) are answered `OK`. `BUFFER n` counts bytes the
far end has not yet acknowledged (Pat waits for 0 before closing).
`PTT ON/OFF` is for information only: Ion keys the radio itself. `SN`
gives the S/N of the last burst heard. A KISS port 200 below the command
port accepts a connection and discards what it is sent, for clients that
open it alongside the command port.

## On the air

Connect, Answer (when called), and net Start, Join and Close all key the
radio. Answer is on by default: with a call set, Ion answers a station
that calls it. Turn Answer off to receive only. Abort (Esc) stops a link
without transmitting.

**Identification.** Ion sends a CW ID at least every 570 s during a link
and once at its end (§97.119). The station that did not close the link
waits for the other's CW ID plus 1.5 s before it transmits. Both ends time
the mid-link IDs from the same instant, so each waits out the other's ID
instead of polling into it (`cw_id_breaks_do_not_cost_polls` in
`crates/ion/tests/node.rs`).

**Radio notes.**

- TCI: Ion sets DIGU on connect. The RX filter is left to ExpertSDR3 and
  must pass 0–3000 Hz; a narrower one is reported as a problem.
- **TX audio** picks the input a CAT key takes on radios with two: Data
  (the default) keys a Kenwood with `TX1;` (DATA SEND, audio from USB or
  ACC2) and rigctld with `T 3` (Hamlib's data PTT); Mic keys `TX;` and
  `T 1`. A radio that refuses the data command gets the plain one. `TX;`
  keys Kenwood and Elecraft but is a read on Yaesu.
  Yaesu wants 2 stop bits on its older menus.
- The 0.2 s PTT-to-RF lead (`DEFAULT_LEAD_S`) has not been measured over
  TCI; sound-card radios set the lead under Keying.

## Loopback

`ionhf --loopback` runs Ion against simulated stations on a simulated
channel, with no radio:

- **Link:** the station `LOOP` answers calls, sends every line back
  ("LOOP copies: ...") and acknowledges every file ("LOOP got ...").
- **Net, Start:** W5AAA, W5BBB and W5HID join the net you run. Only W5BBB
  hears W5HID, so W5HID's traffic arrives relayed.
- **Net, Join:** `LOOPNC` runs net `LOOPNET` with two members and a
  bulletin.

## Not in this version

- Calling CQ or beaconing outside a net.
- Setting the radio's frequency: the operator tunes the radio.
