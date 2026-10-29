# next: next-link and next-rx together

Branch `next` (off `main` at `6b96c22`): `next-link`
([next-link.md](next-link.md): goodput gear shift, MFSK cue behind
`caps::CUE`, sticky Doppler fix, 20 ms block receive loop) merged first,
then `next-rx` ([next-rx.md](next-rx.md): window placement and re-timing,
turbo receiver, expected headers). Both are real merges, so each branch's
history stays intact. Raw outputs are in `lab_runs/next/`.

## Where the two meet

| Where | Resolution |
|---|---|
| `phy/rx.rs`, `phy/rx/demod.rs`, `phy.rs`, `link/audio.rs`, `link/mod.rs`, `station.rs`, `docs/link.md` | Both sides kept: `RxStats`, `PhyRx` and `Receivers` hold the cue state and the expected set; a cue-only burst reports no spreads and still carries its `BurstGrid` |
| Turbo × partial HARQ | `HarqRx::receive_turbo` takes the cue's parity field and receives through `receive_partial`. Without it, a cued partial burst with a failed codeword would have gone to `receive` with field 0, failed the check bits and kept nothing. Unit test `the_turbo_receiver_keeps_a_partial_layout` |
| Expected headers × cue | A cued burst's header comes from the cue (`decode_header` returns it before the Viterbi and expected-header path). A partial burst whose cue is missed fails its check bits and keeps nothing, so listing partial layouts would gain nothing. Nothing is added to `Station::expected`. `tests/expect.rs` counts uncued bursts only, and also runs links with cues off (a peer without them sends every burst uncued): 196 of 196 data bursts and 97 of 97 tiny bursts were expected, and 110 cued bursts were left out |
| Expected headers × goodput gear | The list is built from the ladder and the thresholds, not from the gear: every long-frame OFDM level with a threshold of 5 dB or less, at every frame count up to `max_burst_s`, and the increments alone. That covers the goodput gear's jumps after a loss and a stretched (`harq_stretch`) burst of increments only. `channel_failed` only marks records and sends nothing. Levels above 5 dB that the gear may probe at low S:N are not listed, as on next-rx, because their data is lost there anyway |
| Window placement × cue | New commit: a burst taken on a trusted cue below the preamble threshold keeps the plateau window, because its lag profile is noise. Every burst that next-rx would have accepted is handled as before |
| Turbo × 20 ms block loop | Turbo runs in `Station::on_rx` after `Receivers::run` has returned the whole chunk. The block loop changes when a cue reaches the OFDM receiver, not the per-burst budget (`TURBO_SYMBOLS`). `Receivers::expect` (Node) and `Medium::expect` (Sim) are called once before each chunk or burst, ahead of `run` |
| Capability bits | next-rx does not touch `link/frame.rs`. ConnReq flags 4–7 and ConnAck `extra` 11–14 stay next-link's |
| Tests pinned to the ladder | next-link already pinned the ladder-specific tests (`Gear::Ladder` in `bursts_grow_on_a_steady_floor_and_it_pays`). next-rx's tests pass on the goodput default |
| Power | `a_cue_is_no_louder_than_mfsk_and_leaves_the_ofdm_as_it_was` passes |

## Sessions

`tests/ionos.rs` `ionos_table` on IONOS SIM with 2 paths. S:N is in 3 kHz
against a sine at each burst's peak. Each cell is 5 minutes from the
connect, averaged over 2 seeds (the same seeds in every column), with an
incompressible payload on 6 threads. Figures are B/min, WGN / MPG / MPP /
MPD. main is `befad71`. main and next-link reproduce their published
tables to the byte.

| S:N dB | main | next-link | next-rx | next |
|---:|---|---|---|---|
| +20 | 32572 / 23868 / 28152 / 3309 | 32572 / 23358 / 28747 / 5644 | 32572 / 23902 / 27948 / 20893 | 32572 / 23375 / 28951 / 23664 |
| +10 | 14586 / 10098 / 10846 / 3468 | 14620 / 9860 / 11084 / 3944 | 14586 / 10404 / 12733 / 9894 | 14620 / 10319 / 12478 / 9520 |
| +6 | 10523 / 5115 / 4913 / 825 | 10472 / 6494 / 6324 / 1836 | 10523 / 5914 / 5134 / 1055 | 10710 / 7446 / 6052 / 6307 |
| +3 | 3638 / 528 / 1904 / 651 | 6936 / 2933 / 3910 / 1225 | 3638 / 728 / 1666 / 483 | 6936 / 3239 / 3893 / 1190 |
| 0 | 1054 / 499 / 788 / 324 | 1513 / 836 / 877 / 540 | 1071 / 499 / 836 / 396 | 1513 / 856 / 1180 / 562 |
| −3 | 663 / 403 / 577 / 463 | 702 / 377 / 554 / 514 | 663 / 447 / 577 / 511 | 702 / 427 / 554 / 517 |
| −6 | 562 / 396 / 442 / 440 | 783 / 373 / 436 / 394 | 575 / 396 / 442 / 440 | 783 / 379 / 467 / 394 |

next ÷ main, and next ÷ the better of next-link and next-rx:

| S:N dB | WGN | MPG | MPP | MPD | | WGN | MPG | MPP | MPD |
|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|
| +20 | 1.00 | 0.98 | 1.03 | 7.15 | | 1.00 | 0.98 | 1.01 | 1.13 |
| +10 | 1.00 | 1.02 | 1.15 | 2.75 | | 1.00 | 0.99 | 0.98 | 0.96 |
| +6 | 1.02 | 1.46 | 1.23 | 7.64 | | 1.02 | 1.15 | 0.96 | 3.44 |
| +3 | 1.91 | 6.13 | 2.04 | 1.83 | | 1.00 | 1.10 | 1.00 | 0.97 |
| 0 | 1.44 | 1.72 | 1.50 | 1.73 | | 1.00 | 1.02 | 1.35 | 1.04 |
| −3 | 1.06 | 1.06 | 0.96 | 1.12 | | 1.00 | 0.96 | 0.96 | 1.01 |
| −6 | 1.39 | 0.96 | 1.06 | 0.90 | | 1.00 | 0.96 | 1.06 | 0.90 |

- **The gains add up.** On MPD at +6 dB, next-link's Doppler fix keeps
  the link on OFDM and next-rx's window placement makes those bursts
  decode: 825 → 6307 B/min, 3.4× the better parent. On MPP at 0 dB each
  branch alone gains 6–11% and together they gain 50%, 1.35× the better
  parent (two seeds; not re-run). MPD gains 7× at +20 dB.
- **WGN matches next-link** in every cell, because next-rx leaves WGN as
  it was.
- **No cell is clearly worse than the better parent.** The lowest cells,
  0.90 to 0.98, were run again with 6 seeds on all three branches (MPG,
  MPP and MPD at −6, −3, +6 and +10 dB; `*6.txt`):

| S:N dB | next-link | next-rx | next | next ÷ better |
|---:|---|---|---|---|
| +10 | 10234 / 10693 / 4936 | 10733 / 13011 / 8603 | 10563 / 13277 / 8784 | 0.98 / 1.02 / 1.02 |
| +6 | 6330 / 6273 / 2896 | 6046 / 6035 / 1386 | 7316 / 7117 / 6148 | 1.16 / 1.13 / 2.12 |
| −3 | 444 / 576 / 518 | 425 / 542 / 505 | 466 / 583 / 512 | 1.05 / 1.01 / 0.99 |
| −6 | 348 / 429 / 393 | 361 / 417 / 421 | 371 / 440 / 396 | 1.03 / 1.03 / 0.94 |

  (MPG / MPP / MPD.) With 6 seeds, MPP +6 dB is up 13% and MPD +10 dB
  up 2%. MPD −6 dB is the one cell still down: 0.94 against next-rx,
  where next-link alone has 393 and next has 396. This is next-link's
  trade-off, already in its write-up: the goodput gear probes levels
  above the MFSK floor. The merge adds nothing to it.

## Open

- The placement guard on cue-only bursts was not measured on its own. It
  only affects bursts below the preamble threshold, which next-rx never
  took.
- The questions both write-ups leave open stay open: the real-time cost
  of turbo on slow machines (it now also follows a 1.12 s cue, which
  adds no CPU), the cue on a real radio (AGC, frequency offset, false
  cues), and no run against an alpha.4 binary.
- In-burst re-timing still runs on cue-only bursts. The CP correlation
  there is on the burst's own OFDM signal. MPD from −3 to +3 dB is within
  3% of next-link (517 / 562 / 1190 against 514 / 540 / 1225).

## Reproduce

```text
# from this worktree; the baseline in its own: git worktree add --detach ../ion-hf-base befad71
bash lab_runs/next/run.sh ../ion-hf-base main
bash lab_runs/next/run.sh ../ion-hf-next-link next-link
bash lab_runs/next/run.sh ../ion-hf-next-rx next-rx
bash lab_runs/next/run.sh . next
ION_IONOS_MODES=MPG,MPP,MPD ION_SNRS=-6,-3,6,10 ION_SEEDS=6 bash lab_runs/next/run.sh . next6
```
