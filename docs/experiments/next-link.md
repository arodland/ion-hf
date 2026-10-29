# next-link: goodput gear shift and MFSK cue, for main

Branch `next-link` (off `befad71`, Ion HF 0.1.0-alpha.4). The finished
link-layer experiments made production: defaults on, the old behaviour one
switch away, and nothing new on the air for a peer that has not said it
understands it. Raw outputs in `lab_runs/next-link/`.

## What changed

| Piece | From | What |
|---|---|---|
| Doppler class clears | `fix-sticky-doppler` (396a52f) | a fast-fading class clears after four bursts without a Doppler measurement (MFSK bursts report none) |
| IONOS S:N note | `fix-sticky-doppler` (8bf15e9) | `docs/ionos.md`: the S:N scale against mean SNR in 2.4 kHz |
| Goodput gear shift | `climb` (bfaf729, 4e3e261) | `Config::gear = Gear::Goodput`: each burst's level by expected throughput ([link.md](../link.md), Gear shifting); `harq_stretch`, `merge_pieces` on |
| Dead waveforms | new | goodput: a whole OFDM burst lost with 4 dB to spare counts as a loss at the other levels of its waveform and the faster ones whose guard is no longer (the ladder's holds did this; without it `a_dead_waveform_is_not_retried_over_and_over` spent 260 of 468 s on dead levels) |
| MFSK cue, PHY | `exp-mfskhdr` (08fc374, 360f690) | `phy_mfsk::modulate_cue`, `Kind::Cue`, `MfskBurst::cue`; `PhyRx::cue`; weak-sync bursts report no spreads |
| MFSK cue, link | new (lab: `CuedAir`, `AudioMedium::with_cue`) | `Outgoing::cue`, sent by `audio::transmit` / `transmit_with` (so `Node`, the app and the simulators), heard by `Receivers`, counted in every air time; capability bit; the low-S:N rule; `harq_partial` |
| Capabilities | new | `frame::caps`: 4 bits in the ConnReq's flags (4–7) and the ConnAck's `extra` (11–14); `caps::CUE` the first |
| App | new | `gear = "goodput" \| "ladder"` in `ionhf.toml`, `--gear` (also `ionhf --sim`) |

Dropped from the labs: `keep_clean_db`, `goodput_measured`, `harq_prior`,
the HARQ credit (all measured no better), `CuedAir` and
`AudioMedium::with_cue` (replaced by the per-burst flag). `pin_level` stays,
`#[doc(hidden)]`, for pinned lab runs (`ION_PIN`).

## Defaults and switches

| Setting | Default | Off / other |
|---|---|---|
| `Config::gear` | `Goodput` | `Ladder` (ionhf: `gear = "ladder"`, `--gear ladder`; tests: `ION_GEAR=ladder`) |
| `harq_stretch`, `merge_pieces` | on, on | `ION_STRETCH=0`, `ION_MERGE=0` |
| `cue` | on (offered and accepted) | `ION_CUE=0` |
| `cue_below_db` | 7 dB | `ION_CUE_DB=X` (`inf`: every full-band burst) |
| `harq_partial` | on (with a cue) | `ION_PARTIAL=0` |

`gear = "ladder"` changes only the choice of level: the cue, the HARQ
stretch and the rejoined pieces stay. So `ladder` is not alpha.4 exactly;
alpha.4 itself is the `befad71` column below. Each end's gear governs the
bursts it sends, so the A/B needs no agreement.

## Compatibility

- **On the air.** Only the cue and the partial HARQ count are new, and
  both are sent only on a link whose ConnAck agreed `caps::CUE`. An alpha.4
  station sends a ConnReq with flags bits 4–7 clear and a ConnAck with
  `extra` bits 11–14 clear (its decoder never reads them), so it agrees to
  nothing. The goodput shift, the stretch and the rejoined pieces are the
  sender's own choices inside the alpha.4 protocol: longer bursts and
  pieces the receiver places by offset, as alpha.4 receivers already do.
- **Test** (`crates/ion/tests/cue.rs`, real PHYs through IONOS WGN 0 dB, a
  medium that records every burst): a `next-link` station and one with
  `cue` off (the stand-in for alpha.4: it neither offers nor accepts),
  first as caller then as called station, each sending the other data.
  Both transfers complete; `caps()` is 0 on both ends; no burst goes out
  behind a cue and no full-band data burst carries a parity field; neither
  side's receivers hear a cue. The twin with both ends on agrees
  `caps::CUE`, sends cued bursts (full-band only) and hears them.
  Passing. The stand-in is the same code with the bit off, not the alpha.4
  binary: an alpha.4 build was not run against `next-link`.
- **Missed cue.** The partial count rides only in the cue, so a burst
  whose OFDM part is heard without its cue would be misread. A partial
  burst's check bits count one more than the stored set
  (`harq::burst_check_bits`): such a receiver finds them wrong and keeps
  nothing, as on any HARQ disagreement (tested in
  `harq::tests::a_partial_burst_continues_the_first_stored_and_keeps_the_rest`).
- **Power.** The cue is built by the MFSK modulator at `TX_LEVEL`, like
  every MFSK burst, and `transmit_with` applies the OFDM gain and trim to
  the OFDM part only: the cue's peak is the MFSK header's, the OFDM part is
  sample for sample what it is without a cue, at 0 dB and at the 3 dB
  maximum gain (`a_cue_is_no_louder_than_mfsk_and_leaves_the_ofdm_as_it_was`).
  In the simulators the S:N reference is the OFDM part's, as before.

## The cue rule

A full-band OFDM data burst goes behind a cue on a link that agreed, while
the SNR the gear aims by (the effective SNR the ACKs report, smoothed) is
below `cue_below_db`. An absolute bound, not one relative to the level's
threshold: what the cue fixes is the OFDM header, whose failure depends on
the S:N, not on the level. The goodput shift runs every level close to its
threshold, so a threshold-relative rule would cue the 16QAM levels at
+10 dB, where the header is heard anyway.

Swept on WGN, MPG and MPP, 0 to +10 dB, 5 min, 2 seeds, goodput gear
(`lab_runs/next-link/sweep_*.txt`), B/min, WGN / MPG / MPP:

| S:N dB | no cue | below 4 dB | below 7 dB | always |
|---:|---|---|---|---|
| +10 | 14586 / 9707 / 10897 | 14586 / 9911 / 10897 | 14620 / 9860 / 11084 | 14416 / 9418 / 10999 |
| +6 | 10812 / 7004 / 5593 | 10302 / 7293 / 6239 | 10472 / 6494 / 6324 | 10472 / 6460 / 6324 |
| +3 | 7140 / 1348 / 2499 | 7038 / 2329 / 3791 | 6936 / 2933 / 3910 | 6936 / 2933 / 3910 |
| 0 | 1513 / 733 / 840 | 1513 / 836 / 877 | 1513 / 836 / 877 | 1513 / 836 / 877 |

Below 7 dB the cue does everything always-on does from 0 to +6 dB and
costs nothing at +10 dB (always-on: −1 to −3%). The IONOS S:N reads about
2.7 dB below the reported SNR for the QPSK levels ([ionos.md](../ionos.md)),
so 7 dB is about +9 to +10 dB S:N on WGN, more on fading (the report is
the effective SNR). At +6 dB the cue costs WGN 3% and MPG 7% and gains
MPP 13%: within two seeds' noise. 4 dB gives up a third of the MPG +3 dB
gain. Default 7 dB.

## Sessions

IONOS SIM, two paths, S:N in 3 kHz against a sine at each burst's peak, 5
minutes from the connect, 2 seeds per cell (the same seeds in every
column), incompressible payload, 6 threads. B/min, WGN / MPG / MPP / MPD.
`lab_runs/next-link/run.sh`.

| S:N dB | alpha.4 (`befad71`) | `next-link` (defaults) | `next-link`, `gear = ladder` |
|---:|---|---|---|
| +20 | 32572 / 23868 / 28152 / 3309 | 32572 / 23358 / 28747 / 5644 | 32572 / 23358 / 28152 / 5916 |
| +10 | 14586 / 10098 / 10846 / 3468 | 14620 / 9860 / 11084 / 3944 | 14620 / 9537 / 10523 / 3400 |
| +6 | 10523 / 5115 / 4913 / 825 | 10472 / 6494 / 6324 / 1836 | 10166 / 6562 / 4522 / 2363 |
| +3 | 3638 / 528 / 1904 / 651 | **6936 / 2933 / 3910 / 1225** | 4012 / 1156 / 1802 / 718 |
| 0 | 1054 / 499 / 788 / 324 | **1513 / 836 / 877 / 540** | 1220 / 673 / 863 / 477 |
| −3 | 663 / 403 / 577 / 463 | 702 / 377 / 554 / 514 | 663 / 403 / 577 / 463 |
| −6 | 562 / 396 / 442 / 440 | 783 / 373 / 436 / 394 | 562 / 396 / 442 / 440 |

`next-link` against alpha.4:

| S:N dB | WGN | MPG | MPP | MPD |
|---:|---:|---:|---:|---:|
| +20 | 1.00 | 0.98 | 1.02 | 1.71 |
| +10 | 1.00 | 0.98 | 1.02 | 1.14 |
| +6 | 1.00 | 1.27 | 1.29 | 2.23 |
| +3 | 1.91 | 5.55 | 2.05 | 1.88 |
| 0 | 1.44 | 1.68 | 1.11 | 1.67 |
| −3 | 1.06 | 0.94 | 0.96 | 1.11 |
| −6 | 1.39 | 0.94 | 0.99 | 0.90 |

- **0 to +6 dB is where it moves**: ×1.9–5.6 at +3 dB, +11–68% at 0 dB,
  +27–29% at +6 dB on MPG/MPP. Of the +3 dB gain the goodput shift brings
  WGN all of it (3638 → 7140 without the cue), MPG 528 → 1348 and MPP
  1904 → 2499; the cue takes MPG on to 2933 and MPP to 3910 (sweep).
- **+10 dB and up unchanged** on WGN/MPG/MPP (±2%). The cue is not sent
  there.
- **MPD gains everywhere above −3 dB**, at +20 dB with the ladder too
  (3309 → 5916): the Doppler class now clears after MFSK bursts
  (396a52f), where alpha.4 stayed on MFSK once one OFDM burst had
  measured fast fading.
- **−3 and −6 dB**: WGN −6 dB +39%; MPG, MPP and MPD −6 dB 1–10% down.
  The ladder column is byte-identical to alpha.4 there (both run MFSK,
  which gets no cue), so the difference is the goodput shift probing
  levels above the floor; within two seeds' noise at these rates (one
  burst is 2–4% of a cell), but five of the six fading cells are down.
- **The ladder column** isolates the gear: on the ladder the cue, the
  stretch and the Doppler fix add +10% at WGN +3 dB and +28–47% at MPG 0
  and +6 dB and MPD 0 dB, and cost 8% at MPP +6 dB (seed noise, or the cue
  on the ladder's levels).

## What is uncertain

- Two seeds, 5 minutes: ratios within ±15% are not significant, least at
  0–6 dB on the fading channels.
- The real-radio questions of the cue from the `exp-mfskhdr` write-up
  (`docs/experiments/mfsk-header.md` on that branch) stand: a
  receiver AGC settling on the cue's steady carrier before the OFDM part
  (2.4 dB lower mean for QPSK, 7.3 dB for 64QAM), no frequency offset or
  clock drift in the simulation, a false cue on noise (CRC-16).
- The app path (`Node` with 0.1 s audio blocks into `Receivers`) is the
  same `transmit_with` and `Receivers::process` the tests use, but no
  `tests/node.rs` case runs at an S:N low enough for cues.
- Not checked against an alpha.4 binary on the air or in a two-program
  loopback.

## Reproduce

```text
# sweep (about 5 min each on 6 threads)
ION_IONOS_MODES=WGN,MPG,MPP ION_SNRS=0,3,6,10 bash lab_runs/next-link/run.sh sweep_cue7 ION_CUE_DB=7
# sessions (about 10 min each); the baseline from its own worktree:
#   git worktree add --detach ../ion-hf-base befad71
ION_TREE=../ion-hf-base CARGO_TARGET_DIR=../ion-hf-base/target bash lab_runs/next-link/run.sh full_alpha4
bash lab_runs/next-link/run.sh full_next
bash lab_runs/next-link/run.sh full_ladder ION_GEAR=ladder
```
