# Ion on IONOS SIM

IONOS SIM is the Teensy HF channel simulator that ARSFI's Winlink team used
to compare PACTOR 3 and 4, VARA HF 2300, ARDOP 2000 and WINMOR 1600
(Muething et al., QEX March/April 2022; `ARSFI/HFSimulator` on GitHub, MIT
licence). `ion_dsp::ionos` (`crates/ion-dsp/src/ionos/`) is a port of its
firmware (`HFSim_BFD_2_05_Proto.ino`, rev 2.05), so Ion can be measured on
the same channels and the same S:N scale as the published modems. The
measurement is `crates/ion/tests/ionos.rs`.

## The port

Kept from the firmware:

- the 44.1 kHz rate and its filter tables (3 and 6 kHz low-pass, 119-tap
  Hilbert, 128-tap Gaussian Doppler filter), extracted by script into
  `crates/ion-dsp/src/ionos/coeffs.rs`;
- the path model `gI·x(t−τ) − gQ·x̂(t−τ)`, with the I taps delayed 59
  samples (1.3379 ms) to match the Hilbert filter's group delay, and path
  delays rounded as the Teensy delay object rounds them;
- the gain generator: polar Box–Muller with the firmware's
  0.1 ≤ |z|² < 1 rejection and 1e-5 quantisation, scale 4 (two paths) or
  4/√2 (four paths, the firmware's literal `4.0 / 1.41421`), through the
  Gaussian filter, updated every 156.25 / 31.25 / 15.625 / 7.812 ms (64 ×
  the spread) and applied at the next 128-sample block;
- WGN's single path at a gain of 0.8498, and white noise through the same
  low-pass.

The channels: WGN (noise only), MPG (0.1 Hz spread, 0.5 ms delay), MPM
(0.5 Hz, 1 ms), MPP (1 Hz, 2 ms) and MPD (2 Hz, 4 ms).

**S:N.** The firmware's mixer gains are fixed and calibrated (+4.28 dB) so
that a sine at the nominal input level reads the dialled S:N on a spectrum
analyser, in 3000 or 6000 Hz; the box is levelled on each modem's peaks. So
here S:N is the power of a sine at each burst's own peak over the noise in
3000 Hz (`ion_dsp::ionos::Reference::Peak`), and the bursts are the ones
sent on the air (`AudioMedium::ionos` in `crates/ion/src/link/audio.rs`).
Against the mean-power SNR in 2.4 kHz that the other documents use, this
scale is harder by the burst's envelope PAPR less 3 dB, plus 1 dB of
bandwidth: about 1 dB for MFSK, 2 dB for the low-PAPR QPSK levels, 4 dB
for 16QAM and 6.5 dB for 64QAM (PAPR in [phy.md](phy.md)).
`ION_IONOS_REF=mean` measures against each burst's mean power instead.

**Checks** (`cargo test -p ion-dsp --release ionos`): the Gaussian
filter's documented response (−9.1 dB at 1 Hz, −37.1 dB at 2 Hz), unity
gain on WGN, the low-pass stop band, the Teensy delay rounding, the
streaming resampler, and the mean level of the multipath channels. With the
firmware's gain constants a multipath channel runs about 1 dB above WGN on
average (+1.4 dB with two paths, +0.9 dB with four); this is kept, since
the box does the same to every modem.

Not ported: flat fading, frequency offset and FM, the busy detector.

## Results

10-minute sessions from the connect, incompressible payload, Ion's
compression off, two paths, two seeds per cell, the 2.45 kHz layout.
Bytes per minute (bit/s):

| S:N dB | WGN | MPG | MPP |
|---:|---:|---:|---:|
| 30 | 55 624 (7417) | 47 132 (6284) | 47 864 (6382) |
| 25 | 47 158 (6288) | 32 776 (4370) | 33 906 (4521) |
| 20 | 33 626 (4483) | 25 832 (3444) | 28 926 (3857) |
| 15 | 22 092 (2946) | 15 598 (2080) | 16 686 (2225) |
| 10 | 14 807 (1974) | 10 888 (1452) | 11 203 (1494) |
| 5 | 8 109 (1081) | 3 477 (464) | 3 120 (416) |
| 0 | 1 390 (185) | 708 (94) | 720 (96) |
| −5 | 688 (92) | 439 (59) | 484 (65) |
| −10 | 463 (62) | 208 (28) | 219 (29) |

Seeds differ most at 0–5 dB on the fading channels (MPG 5 dB: 2 789 and
4 165 B/min). To reproduce:

```text
ION_SEEDS=2 ION_SNRS=-10,-5,0,5,10,15,20,25,30 \
  cargo test -p ion --release --test ionos -- --ignored --nocapture
```

`ION_IONOS_MODES`, `ION_PATHS`, `ION_MINUTES` and `ION_THREADS` select the
channels, paths, session length and threads; `ION_TRACE=1` prints every
data burst's level. The test's header lists the other switches.

Against the QEX curves (figures 12 and 13, values read off the charts,
about ±10%), bytes per minute:

| Channel, S:N | PACTOR 4 | VARA 2300 | PACTOR 3 | ARDOP 2000 | Ion |
|---|---:|---:|---:|---:|---:|
| MPG 30 | 37 500 | 29 000 | 14 000 | 4 000 | 47 132 |
| MPG 20 | 21 300 | 11 800 | 9 500 | 1 500 | 25 832 |
| MPG 10 | 7 700 | 3 200 | 4 500 | 500 | 10 888 |
| MPG 5 | 2 200 | 1 300 | 2 000 | — | 3 477 |
| MPG 0 | 1 200 | 500 | 700 | — | 708 |
| MPP 30 | 34 200 | 17 900 | 8 200 | 3 600 | 47 864 |
| MPP 20 | 22 000 | 10 900 | 6 700 | 1 500 | 28 926 |
| MPP 10 | 10 100 | 3 600 | 3 800 | 300 | 11 203 |
| MPP 5 | 2 800 | 1 300 | 1 700 | — | 3 120 |
| MPP 0 | 900 | 900 | 1 100 | — | 720 |

From 5 dB up Ion is ahead of every published modem on both channels:
11–58% ahead of PACTOR 4, and 1.6–3.4 times VARA 2300 from 10 dB up. At 0 dB it
is behind PACTOR 4 on both channels, level with PACTOR 3 and ahead of VARA
2300 on MPG, and behind all three on MPP.

## What this does and does not show

- The same channel model and S:N scale as the published runs, so the
  shape is comparable.
- Not like for like. The published numbers are Winlink sessions: real
  modems and sound cards through the hardware box, B2F handshakes and
  message overhead, real PTT and turnaround times, 5–15 minute runs. Ion
  here is its ARQ alone in software: no session protocol, 0.3 s
  turnarounds, no clock skew, frequency offset or ALC. That favours Ion, by
  an amount not measured.
- Ion is 2.45 kHz wide (2.58–2.61 kHz occupied by FCC §97.3(a)(8)),
  against 2–2.4 kHz for the others; not normalised.
- The fair comparison is Ion through a real IONOS box with Pat or Winlink
  Express on the host port, the way the others were run.

[mercury.md](mercury.md) runs Mercury, an open-source modem, through the
same port for a software-against-software comparison.
