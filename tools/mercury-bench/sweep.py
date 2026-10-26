#!/usr/bin/env python3
"""Runs bench.py over a grid, a few sessions at a time.

  python3 sweep.py --mercury ... --pipe ... --out <out dir> \
      --cells MPP:0,MPP:10,MPG:20 [--minutes 10] [--jobs 2] [--seeds 1] [--rt 1]

Each cell is MODE:SNR (SNR "clean" for none). Results land in
OUT/MODE_SNR_sSEED/result.json; a summary table is printed at the end.
"""

import argparse
import json
import os
import subprocess
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mercury", required=True)
    p.add_argument("--pipe", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--cells", required=True)
    p.add_argument("--minutes", type=float, default=10.0)
    p.add_argument("--jobs", type=int, default=2)
    p.add_argument("--seeds", type=int, default=1)
    p.add_argument("--seed0", type=int, default=0, help="first seed index (to add seeds to a grid)")
    p.add_argument("--rt", type=float, default=1.0)
    p.add_argument("--peak", type=float, default=0.51)
    p.add_argument("--bw", type=int, default=2300)
    p.add_argument("--extra", default="", help="more bench.py arguments, space-separated")
    args = p.parse_args()

    jobs = []
    for cell in args.cells.split(","):
        mode, snr = cell.split(":")
        for s in range(args.seed0, args.seed0 + args.seeds):
            jobs.append((mode, snr, s))
    lock = threading.Lock()
    nxt = [0]

    def worker(slot):
        while True:
            with lock:
                if nxt[0] >= len(jobs):
                    return
                mode, snr, s = jobs[nxt[0]]
                nxt[0] += 1
            seed = 900 + (0 if snr == "clean" else int(float(snr))) + 1000 * s
            d = os.path.join(args.out, "%s_%s_s%d" % (mode, snr, s))
            cmd = [sys.executable, "-I", os.path.join(HERE, "bench.py"), "--mercury", args.mercury,
                   "--pipe", args.pipe, "--mode", mode, "--seed", str(seed), "--minutes", str(args.minutes),
                   "--rt", str(args.rt), "--port", str(9300 + 100 * slot), "--bw", str(args.bw),
                   "--peak-a", str(args.peak), "--peak-b", str(args.peak), "--out", d]
            cmd += args.extra.split()
            if snr != "clean":
                cmd += ["--snr", snr]
            r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            with lock:
                print(r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "%s %s: no output" % (mode, snr),
                      flush=True)

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(args.jobs)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    rows = {}
    for mode, snr, s in jobs:
        f = os.path.join(args.out, "%s_%s_s%d" % (mode, snr, s), "result.json")
        try:
            r = json.load(open(f))
        except (OSError, ValueError):
            continue
        rows.setdefault((mode, snr), []).append(r.get("done_s") or r["bytes_per_min"])
    print("\n| mode | S:N | B/min | runs |")
    print("|---|---:|---:|---:|")
    for (mode, snr), v in rows.items():
        print("| %s | %s | %.0f | %d |" % (mode, snr, sum(v) / len(v), len(v)))


if __name__ == "__main__":
    main()
