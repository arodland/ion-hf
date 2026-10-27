# Broadcast groups

Frames to every station of a named group, once each, with no
acknowledgement and no resend: Data2G's broadcast (its
`docs/broadcast.md`), on Ion's waveforms. It is what a host program such as
Ragtime uses for group chat; reliability is the program's (Ragtime's Acks,
Pings and repair rounds).

Code: `crates/ion/src/link/bcast.rs` (`BcastStation`), beside the link in
`crates/ionhf/src/ion_engine.rs`; the host commands in
`crates/ionhf/src/host.rs` ([app.md](app.md), Host interface). Tests:
`crates/ion/tests/bcast.rs` (model channel), the framing tests in
`bcast.rs`.

## On the air

- **A burst** is an ordinary Ion burst at one ladder level, the group's
  8-bit id (a hash of its name) in the PHY header's session field.
- **Header:** the first codeword starts with a version byte, the sender's
  packed call and the group's 16-bit key (FNV-1a of the name), which
  settles the rare 8-bit id clash.
- **Frames** are COBS-encoded (no zero byte left in them) and each ended
  by a zero, back to back across the codewords; each codeword's field says
  its place. A frame that would start a codeword gets a zero before it, so
  its start is known when the codeword before is lost. A lost codeword
  loses only the frames it touches.
- **Header lost:** the frames still go to the group when only one open
  group has the burst's id; the burst comes with no call.
- **Size:** a burst takes the first queued frame's group, then every later
  frame of that group that fits 8 s; the first always goes, however long
  (a 255-byte frame goes whole at any level).

## Channel access and level

- **Sessions first:** broadcast bursts go only while the link is idle, as
  on Data2G's host.
- **Access:** a frame queued while the channel is free goes at once. One
  that had to wait (a burst on the air, or one heard since) takes each 1 s
  slot with probability (P + 1) / 256: KISS persistence P, 63 by default.
  After 60 s of unbroken busy it goes anyway.
- **Level:** the fastest level the weakest station heard in the last 15
  minutes in the group copies, 3 dB over its threshold; level 3 (MFSK,
  about −6.5 dB) until anyone is heard.
- **Identification:** a CW ID after the first broadcast burst and at least
  every 9.5 minutes while broadcast traffic goes on.

## Measured

Six stations in one group, each sending lines (60 to 550 bytes) at random
moments, 30 lines in 20 minutes, 20 seeds: the share of lines that reached
each other station, and the airtime (all stations, one run):

| S/N | Fading | Margin | Delivered | Airtime |
|---|---|---|---:|---:|
| 8 dB | none | 3 dB | 97.7% | 87 s |
| 8 dB | none | 6 dB | 96.5% | 246 s |
| 0 dB | none | 3 dB | 98.2% | 381 s |
| 0 dB | none | 9 dB | 33.8% | 2533 s |
| 8 dB | 6 dB, 20 s | 3 dB | 90.7% | 296 s |
| 8 dB | 6 dB, 20 s | 9 dB | 68.9% | 1328 s |
| 0 dB | 6 dB, 20 s | 3 dB | 62.9% | 1292 s |
| 15 dB | 6 dB, 20 s | 3 dB | 92.3% | 85 s |
| 15 dB | 6 dB, 20 s | 9 dB | 96.2% | 316 s |

A wider margin makes bursts longer, and on a shared channel the extra
collisions cost more than the robustness gains; 3 dB stays. At 0 dB with
fading the channel is close to full: losses are collisions more than
noise.

**Ragtime over Ion** (`ionhf --sim`, three stations, IONOS MPG at 10 dB,
Ragtime 0.2.0-beta's own `HostLink` and `BroadcastChat`): nine lines, three
of them addressed; 17 of 18 deliveries (one lost, reported to Ragtime as
`BCAST 1 LOST 1`), none twice, every line Sent on its ACKMODE tags, all
three addressed lines Acknowledged by the far Ragtime within 2 s.

## Not done

- **Several groups' bursts at once:** a burst carries one group's frames.
- **Slices:** bursts are full width; three slices side by side would carry
  three stations at once (the PHY already decodes overlapping slice
  bursts, [phy.md](phy.md), Neighbouring slices).
- **Soft combining of a program's resends** (Data2G lists it as future
  work too).
