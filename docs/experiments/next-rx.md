# next-rx: receiver improvements from the exp-otfs and exp-rx experiments

Branch `next-rx` (from `befad71`). Three receiver-side changes built out
from finished experiments; nothing on the air changes, and a station
with them talks to one without them as before.

## What changed

1. **Window placement for echoes near the CP** (Normal guard;
   `phy/rx.rs`, `phy/rx/retime.rs`; from `exp-otfs` `0267f4b`,
   [phy.md](../phy.md#echoes-near-the-cyclic-prefix)). At sync the FFT
   window is moved when the energy window leaves an echo outside the CP;
   during the burst the CP correlation follows both paths and steps the
   window; the grid is re-referenced to the mean window; Normal's Wiener
   delay grid goes to 6 ms. Long and the slice keep the plateau rule.
   - The 0.1–0.2 dB cost exp-otfs saw on WGN/MPG/MPP had two causes, both
     removed: the re-timing stepped the window on noise (it took the
     first and last threshold crossing anywhere in the CP correlation;
     now only the run around the peak), and the sync rule moved the
     window on noise lags near threshold on MPP (paths within 10 dB and a
     one-sample tolerance; now within 6 dB and 3 samples). The cost was
     not the sync rule on WGN, as first thought: there it fires only on
     candidates that are rejected anyway.
2. **Turbo receiver** (`phy/rx/refine.rs`, `HarqRx::receive_turbo`; from
   `exp-rx` `e169876`, [phy.md](../phy.md#iterative-data-stage),
   [link.md](../link.md#harq)). Each data `RxBurst` carries a snapshot
   (`BurstGrid`) of its data stage, so refinement never reads a grid the
   next burst has overwritten (the experiment's `Arc<Mutex<PhyRx>>`
   `Refiner` is gone). Soft re-estimation and soft interference
   cancellation from the decoder's posteriors, on long-frame bursts with
   a failed codeword.
3. **Expected-header decoding** (`PhyRx::expect`, `Station::expected`,
   `Receivers::expect`, [phy.md](../phy.md#expected-headers),
   [link.md](../link.md#expected-headers)). Wired for real: the station
   lists the peer's possible next bursts while a link is up (none
   outside), each OFDM receiver keeps those of its guard, and the
   simulator and `Node` pass the list before each burst. Besides the
   5σ score, a margin test against the Viterbi decision (30 nats) guards
   against bursts whose own header is not expected.

## Defaults

| Setting | Default | Why |
|---|---|---|
| Placement and re-timing | Normal and NormalWide on; Long and slice off | Long's CP holds the echoes; the slice cannot resolve paths. `set_legacy_timing` (test hook) |
| `Config::turbo` | 2 passes | 2 passes take most of the gain (4 add 0.5 dB at MPG's 10% point only) |
| `TURBO_SYMBOLS` | 1000 symbol passes per burst | A pass costs ~0.5 ms of CPU per data symbol (~180 ms on an 8 s Normal burst here): two passes on 8 s bursts, one on 16 s, none on the longest; at most ~0.5 s before the ACK, inside the peer's 2 s reply slack. Slow machines should lower `turbo` |
| `Config::expect_headers` | on | |
| `HYP_Z`, `HYP_MARGIN` | 5σ, 30 nats | True headers taken lay at most 24.4 nats from the Viterbi decision; no wrong header was taken even with no margin limit |
| Expected data levels | OFDM levels with effective-SNR threshold ≤ 5 dB, frame counts up to `max_burst_s` | Headers fail only below ~+5 dB; the sender jumps several levels after a loss. About 190 headers per receiver |
| Expected tiny bursts | poll, idle; turn only while our ACKs ask for it; goodbyes only while saying goodbye | A turn or goodbye taken from a guess would hand the link over or end it |

## Measured gains and costs

Codeword loss crossings, Normal QPSK 1/2, IONOS 2 paths, S:N in 3 kHz
against the burst's peak (`tests/rx_lab.rs`, every variant on the same
audio), 10% / 30% / 50%:

| | WGN | MPG | MPP | MPD |
|---|---:|---:|---:|---:|
| main | 4.8 / 4.4 / 4.2 | 9.8 / 6.7 / 5.6 | 7.7 / 6.8 / 6.2 | > 20 |
| + placement, re-timing (60 bursts) | 4.8 / 4.4 / 4.2 | 9.8 / 6.7 / 5.6 | 7.7 / 6.8 / 6.2 | 10.6 / 8.6 / < 8 |
| + turbo, 2 passes (30 bursts) | 4.9 / 4.4 / 4.3 | 9.7 / 7.1 / 6.2 | 7.0 / 5.7 / 5.3 | 10.0 / < 8 / < 8 |

(The turbo row's own baseline, on its 30 bursts: 4.9 / 4.5 / 4.3,
10.1 / 7.7 / 6.5, 7.3 / 6.7 / 6.2, 10.0 / 8.2 / < 8.) Turbo leaves the
stored LLRs of codewords that still fail as informative as before (0.422
bit per coded bit either way on MPP; MPG 0.4054 → 0.4058; WGN 0.4245 →
0.4239), so HARQ combining is not hurt by the posteriors fed back.

Header loss (bursts lost before the data), 10% point, `hdr_floor`:

| | WGN | MPG | MPP |
|---|---:|---:|---:|
| Normal QPSK 1/3 | 1.4 → 0.0 | 6.0 → 3.5 | 5.5 → 3.2 |
| Long QPSK 1/4 | 1.3 → −0.8 | 5.3 → 2.0 | 5.0 → 3.0 |

False accepts: none from 2880 bursts per guard of another link or with
unexpected header fields at −3 to +9 dB (with every tiny kind in the set,
tried on the header-only retry), none from 20 minutes of noise per guard
(which never synced), none from 1000 noise LLR vectors against 500
headers (unit test).

CPU: a turbo pass ~180 ms per 8 s Normal burst; only bursts with a
failed codeword pay it. Expected headers: a few hundred correlations of
384 values per failed header.

## Sessions

`tests/ionos.rs` `ionos_table`, 2 paths, S:N in 3 kHz against the
burst's peak, 5-minute sessions, mean of two seeds, bytes per minute,
main (`befad71`) → next-rx (all three, defaults; the same to the byte
before and after the turn and goodbyes were gated):

| S:N dB | WGN | MPG | MPP | MPD |
|---:|---:|---:|---:|---:|
| 20 | 32 572 → 32 572 (+0%) | 23 868 → 23 902 (+0%) | 28 152 → 27 948 (-1%) | 3 309 → 20 893 (+531%) |
| 10 | 14 586 → 14 586 (+0%) | 10 098 → 10 404 (+3%) | 10 846 → 12 733 (+17%) | 3 468 → 9 894 (+185%) |
| 6 | 10 523 → 10 523 (+0%) | 5 115 → 5 914 (+16%) | 4 913 → 5 134 (+4%) | 825 → 1 055 (+28%) |
| 3 | 3 638 → 3 638 (+0%) | 528 → 728 (+38%) | 1 904 → 1 666 (-12%) | 651 → 483 (-26%) |
| 0 | 1 054 → 1 071 (+2%) | 499 → 499 (+0%) | 788 → 836 (+6%) | 324 → 396 (+22%) |
| -3 | 663 → 663 (+0%) | 403 → 447 (+11%) | 577 → 577 (+0%) | 463 → 511 (+10%) |

MPD: 6.3× at 20 dB and 2.9× at 10 dB. MPP +17% at 10 dB, MPG +16% at
6 dB. Below 6 dB a session's path through the ladder is chaotic (MPP
0 dB: 521 / 1054 bytes/min on main's two seeds, 1100 / 572 on next-rx's),
so the cells that went down (MPP 3 dB −12%, MPD 3 dB −26%) were run
again with six seeds, with piece 1 alone (`ION_TURBO=0 ION_EXPECT=0`)
beside them:

| S:N dB | MPP main | next-rx | piece 1 only | MPD main | next-rx | piece 1 only |
|---:|---:|---:|---:|---:|---:|---:|
| 6 | 4698 | 6035 | 4346 | 1326 | 1386 | 1326 |
| 3 | 2136 | 2080 | 2068 | 508 | 517 | 552 |
| 0 | 631 | 657 | 631 | 437 | 495 | 447 |

Within seed noise at 3 dB (MPP per seed: 8840–14110 bytes on main,
6800–15470 on next-rx); +28% on MPP at 6 dB, mostly from turbo and the
expected headers. WGN is unchanged at every S:N (its levels' headers and
data are clean by the time the link uses them).

Raw outputs: `lab_runs/next-rx/` (`timing_final`, `turbo_fer`,
`hdr_final`, `hdr_false`, `ionos_main`, `ionos_nextrx`, `low_*`).

## Open

- **Real-time cost of turbo on slow machines.** The budget is in symbol
  passes (deterministic, so simulations reproduce); on a machine several
  times slower than this one two passes on an 8 s burst approach the
  peer's 2 s reply slack. A wall-clock cap, or running the refinement
  off the audio thread, would remove the question.
- **Noise never synced** in the 20-minute runs, so the noise half of the
  false-accept test exercises the detector more than the header test;
  the unit test covers the header test on noise LLRs directly. Band noise
  with impulses and other modems' bursts were not tried.
- **Tiny ACKs are not expected** (bitmap and report: too many values);
  the sending side gains only on full ACK frames. The first data burst
  of a link or after a turn is not expected either, nor a goodbye the
  peer starts or a turn we did not ask for (gated on purpose: those
  would end the link or hand it over if taken from a guess).
- One plain LDPC decode in `turbo_fer` (WGN 5 dB, 0 passes, main's
  receiver) passed every check with wrong bits; the link's frame check
  above it catches such frames. Not chased.
- Long keeps the plateau rule; its MPD floor (4–5% of codewords in
  exp-otfs) and turbo on the Long and slice levels were not re-measured
  here.
