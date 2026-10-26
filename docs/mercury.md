# Ion against Mercury on IONOS SIM

Mercury (Rhizomatica, GPL-3, release 1.9.21) is the open-source HF ARQ
modem closest to Ion: FreeDV data modes plus QAM16C2 and an MFSK floor,
behind a VARA-style TNC interface. Both run here through the same port of
IONOS SIM ([ionos.md](ionos.md)), measured the same way.

## Method

- **Ion:** `crates/ion/tests/ionos.rs`: 10-minute sessions from the
  connect, incompressible payload, compression off, two paths, two seeds
  per cell, S:N in 3 kHz against a sine at each burst's peak. The Ion
  column below is the Results table of [ionos.md](ionos.md).
- **Mercury:** two `mercury` daemons on Linux (or WSL) on their `-x sock`
  lockstep transport (a virtual clock, paced in real time), each direction
  through its own IONOS box (`crates/ion-dsp/examples/ionos_pipe.rs`, with
  streaming resamplers either side of the box's 44.1 kHz).
  `tools/mercury-bench/bench.py` drives them as a TNC client: `MYCALL`,
  `COMPRESSION OFF`, `BW2300`, `LISTEN`, `CONNECT`; incompressible bytes
  fed as Mercury's TX buffer drains; bytes counted at B's data port for 10
  minutes from the `CONNECT`. `tools/mercury-bench/sweep.py` runs the grid.
  One seed per cell.
- **S:N:** Mercury sends every mode at the same peak (about −5.8 dBFS,
  measured per burst), so a fixed noise level against a sine at that peak
  is the same definition as Ion's per-burst peak reference. QAM16C2's
  peak sits 12.3 dB above its mean (its control frames' 6.8 dB), which
  costs it on a peak-levelled channel, as it would on a peak-limited
  transmitter.
- The audio reaches Mercury 6 dB down (so fades do not clip 16-bit
  samples); signal and noise drop alike.

Mercury was built from source with its ALSA and Pulse back ends stubbed
out; only the `sock` audio path is used. No Mercury code is in this
repository: the bench drives Mercury as a program.

## Validation

Against Mercury's own figures (the CHANNEL-MODEL document in its repository:
4 KB transfers on AWGN, in-burst mean S:N):

| In-burst S:N, theirs / ours | Their link time | Ours (CONNECT to last byte) |
|---|---:|---:|
| 13.7 / 13.4 dB | 31 s | 63 s |
| 5.4 / 5.5 dB | 41 s | 70 s |
| 1.2 / 1.0 dB | 43 s | 70 s |
| −0.7 / −0.9 dB | 197 s | 195 s |

The floor agrees. Our times include the 7.8 s connect, and at the top our
run probed QAM16C2, lost the probe and fell back to DATAC17. The AWGN
plateau (about 2.1 kbit/s, 16.5 kB/min) matches the one they report.

## Results

Bytes per minute, 10-minute sessions:

| S:N dB | Ion WGN | Mercury WGN | Ion MPG | Mercury MPG | Ion MPP | Mercury MPP |
|---:|---:|---:|---:|---:|---:|---:|
| 30 | 55 624 | 16 450 | 47 132 | 15 870 | 47 864 | 13 228 |
| 25 | 47 158 | 16 450 | 32 776 | 7 099 | 33 906 | 8 417 |
| 20 | 33 626 | 16 450 | 25 832 | 6 263 | 28 926 | 7 245 |
| 15 | 22 092 | 5 132 | 15 598 | 5 058 | 16 686 | 7 782 |
| 10 | 14 807 | 5 360 | 10 888 | 3 661 | 11 203 | 1 581 |
| 5 | 8 109 | 1 658 | 3 477 | 800 | 3 120 | 1 267 |
| 0 | 1 390 | 283 | 708 | 199 | 720 | 200 |
| −5 | 688 | 283 | 439 | 228 | 484 | 265 |
| −10 | 463 | 283 | 208 | 110 | 219 | 0 |

Ion is ahead in every cell: 2–5 times over most of the range, 1.6–1.9
times at −10 dB, where Mercury's MFSK floor is at its best; at MPP −10 dB
Mercury never connected. Mercury tops out at QAM16C2 (3135 bit/s raw,
about 2.1 kbit/s delivered); Ion goes to 64QAM.

## Caveats

- Mercury has one seed per cell, and its fading cells scatter (its own
  document reports 857–1379 bit/s for four repeats at 25 dB on CCIR poor);
  Ion has two.
- Ion is 2.45 kHz wide (2.58–2.61 kHz occupied) against Mercury's 2.1 kHz
  top modes; not normalised.
- Both are software against software: no radios, clock skew, frequency
  offset or ALC. That favours neither by design, but neither has been
  through a real box.
