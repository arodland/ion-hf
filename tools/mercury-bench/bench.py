#!/usr/bin/env python3
"""Two Mercury stations through IONOS SIM, measured the way Ion's IONOS
tests in crates/ion measure Ion: an ARQ session A -> B from the CONNECT,
incompressible payload, compression off, bytes delivered at B's data port
in N minutes of signal time.

Runs on Linux or WSL: Mercury's -x sock backend is an AF_UNIX lockstep
transport on a virtual clock. The channel is the ionos_pipe example of
crates/ion-dsp (under WSL the Windows build runs through interop). This
script only drives Mercury as a program (its sim socket and TNC ports); no
Mercury code is in this repository.

  python3 bench.py --mercury <mercury binary> --pipe <ionos_pipe binary> \
      --mode MPP --snr 10 --peak-a 0.5 --peak-b 0.5 --minutes 10 --out <run dir>

Without --snr the channel is clean (for calibrating the peaks). --rt is the
pace against real time (1 = the radio's own).
"""

import argparse
import json
import os
import queue
import random
import socket
import struct
import subprocess
import threading
import time

BLOCK_MS = 20
BLOCK = 160  # 8 kHz
QUEUE_MAX = 24 * 1024  # bytes we let wait in Mercury's TX buffer

state = {"vnow": 0, "stop": False}
# Every TNC socket stays open for the whole run: Mercury treats a closed
# control port as the client leaving.
keep = []
events = []  # (vtime_s, who, line)
ev_lock = threading.Lock()


def log_event(who, line):
    with ev_lock:
        events.append((state["vnow"] / 1000.0, who, line))


def recv_full(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise EOFError("closed")
        buf.extend(chunk)
    return bytes(buf)


class Station:
    def __init__(self, conn):
        self.conn = conn

    def send_sim(self, seq, vnow, samples):
        n = len(samples) // 2
        self.conn.sendall(struct.pack("<IQQH", 18 + 2 * n, seq, vnow, n) + samples)

    def recv_station(self):
        (length,) = struct.unpack("<I", recv_full(self.conn, 4))
        body = recv_full(self.conn, length)
        _seq, ptt, n = struct.unpack("<QBH", body[:11])
        return ptt, body[11:11 + 2 * n]


class Control:
    """A TNC control port: CR-terminated lines, commands answered OK/WRONG."""

    def __init__(self, who, port):
        self.who = who
        self.replies = queue.Queue()
        self.lines = queue.Queue()
        # Mercury's TX buffer as it last reported it, and what we have
        # written since.
        self.buffer = 0
        self.since_report = 0
        self.sock = connect_retry(port)
        self.sock.settimeout(None)
        keep.append(self)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buf = b""
        while not state["stop"]:
            try:
                chunk = self.sock.recv(4096)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            while b"\r" in buf:
                line, buf = buf.split(b"\r", 1)
                line = line.decode("latin-1").strip()
                if not line:
                    continue
                if line.startswith("BUFFER"):
                    try:
                        self.buffer = int(line.split()[1])
                        self.since_report = 0
                    except (IndexError, ValueError):
                        pass
                if line not in ("IAMALIVE",) and not line.startswith("BUFFER"):
                    log_event(self.who, line)
                if line.startswith("OK") or line.startswith("WRONG"):
                    self.replies.put(line)
                self.lines.put(line)

    def cmd(self, text, timeout=30):
        log_event(self.who, "> " + text)
        self.sock.sendall((text + "\r").encode())
        r = self.replies.get(timeout=timeout)
        if not r.startswith("OK"):
            raise RuntimeError("%s %r -> %r" % (self.who, text, r))
        return r


def connect_retry(port, timeout=60):
    t0 = time.time()
    while True:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=5)
        except OSError:
            if time.time() - t0 > timeout:
                raise
            time.sleep(0.2)


def payload(n, seed):
    rnd = random.Random(seed)
    return bytes(rnd.getrandbits(8) for _ in range(n))


def session(args, ports, out):
    """The TNC side: set up, connect, send; B's data port counts arrivals."""
    a_port, b_port = ports
    try:
        a = Control("A", a_port)
        b = Control("B", b_port)
        for c, call in ((a, "TESTA"), (b, "TESTB")):
            c.cmd("MYCALL " + call)
            c.cmd("COMPRESSION OFF")
            c.cmd("BW%d" % args.bw)
        b.cmd("LISTEN ON")
        data_a = connect_retry(a_port + 1)
        data_b = connect_retry(b_port + 1)
        keep.extend([data_a, data_b])
        data_a.settimeout(None)
        data_b.settimeout(1.0)

        def rx():
            while not state["stop"]:
                try:
                    chunk = data_b.recv(65536)
                except socket.timeout:
                    continue
                except OSError:
                    return
                if not chunk:
                    return
                with ev_lock:
                    out["rx"].append((state["vnow"] / 1000.0, len(chunk)))

        threading.Thread(target=rx, daemon=True).start()
        out["connect_cmd_s"] = state["vnow"] / 1000.0
        a.cmd("CONNECT TESTA TESTB")
        # Queue the payload as Mercury's TX buffer drains: what it cannot
        # take (64 KB) it drops ("Failed to queue ARQ data frame(s)").
        data = payload(args.payload, args.seed)
        i = 0
        while i < len(data) and not state["stop"]:
            if a.buffer + a.since_report > QUEUE_MAX:
                time.sleep(0.05)
                continue
            data_a.sendall(data[i:i + 4096])
            a.since_report += len(data[i:i + 4096])
            i += 4096
    except Exception as e:  # noqa: BLE001 - recorded, the run is then void
        out["error"] = repr(e)
        log_event("bench", "error " + repr(e))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mercury", required=True)
    p.add_argument("--pipe", required=True)
    p.add_argument("--mode", default="WGN")
    p.add_argument("--paths", type=int, default=2)
    p.add_argument("--snr", type=float)
    p.add_argument("--peak-a", type=float, default=0.5)
    p.add_argument("--peak-b", type=float, default=0.5)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--minutes", type=float, default=10.0)
    p.add_argument("--bw", type=int, default=2300)
    p.add_argument("--payload", type=int, default=400_000)
    p.add_argument("--rt", type=float, default=1.0)
    p.add_argument("--port", type=int, default=9300)
    p.add_argument("--until-done", action="store_true",
                   help="stop once the whole payload is in (for timed transfers)")
    p.add_argument("--out", required=True)
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)
    tmp = "/tmp/mb%d" % os.getpid()
    os.makedirs(tmp, exist_ok=True)
    socks = [os.path.join(tmp, n) for n in ("a.sock", "b.sock")]
    listeners = []
    for path in socks:
        if os.path.exists(path):
            os.unlink(path)
        ln = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        ln.bind(path)
        ln.listen(1)
        ln.settimeout(60)
        listeners.append(ln)

    pipe_args = [args.pipe, "--mode", args.mode, "--paths", str(args.paths), "--seed", str(args.seed),
                 "--block", str(BLOCK), "--peak-a", str(args.peak_a), "--peak-b", str(args.peak_b)]
    if args.snr is not None:
        pipe_args += ["--snr", str(args.snr)]
    pipe_err = open(os.path.join(args.out, "pipe.txt"), "w")
    pipe = subprocess.Popen(pipe_args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=pipe_err, bufsize=0)

    ports = (args.port, args.port + 10)
    procs = []
    for i, name in enumerate("AB"):
        lf = open(os.path.join(args.out, "mercury_%s.txt" % name), "w")
        cmd = [args.mercury, "-L", os.path.join(args.out, "timing_%s.log" % name), "-x", "sock", "-i", socks[i],
               "-p", str(ports[i]), "-b", str(ports[i] + 5), "-m", "1", "-C", os.path.join(tmp, "missing.ini")]
        procs.append(subprocess.Popen(cmd, stdout=lf, stderr=subprocess.STDOUT, cwd=tmp))
    stations = [Station(ln.accept()[0]) for ln in listeners]

    out = {"args": vars(args), "rx": []}
    threading.Thread(target=session, args=(args, ports, out), daemon=True).start()

    silence = bytes(2 * BLOCK)
    rx = [silence, silence]
    seq = 0
    keyed = [0, 0]
    end_ms = None
    t0 = time.time()
    wall_hist = []
    try:
        while True:
            if end_ms is None and "connect_cmd_s" in out:
                end_ms = out["connect_cmd_s"] * 1000 + args.minutes * 60_000
            if end_ms is not None and state["vnow"] >= end_ms:
                break
            if args.until_done and sum(n for _, n in out["rx"]) >= args.payload:
                out["done_s"] = out["rx"][-1][0] - out["connect_cmd_s"]
                break
            if state["vnow"] > (args.minutes + 5) * 60_000:
                out.setdefault("error", "never connected")
                break
            if args.rt > 0:
                d = t0 + seq * BLOCK_MS / 1000.0 / args.rt - time.time()
                if d > 0:
                    time.sleep(d)
            for s in range(2):
                stations[s].send_sim(seq, state["vnow"], rx[s])
            frames = [stations[s].recv_station() for s in range(2)]
            ptt = [f[0] for f in frames]
            tx = [f[1].ljust(2 * BLOCK, b"\0")[:2 * BLOCK] for f in frames]
            pipe.stdin.write(bytes([1 if ptt[0] == 1 else 0, 1 if ptt[1] == 1 else 0]) + tx[0] + tx[1])
            heard = recv_full_pipe(pipe.stdout, 4 * BLOCK)
            new = [heard[:2 * BLOCK], heard[2 * BLOCK:]]
            for s in range(2):
                # Half duplex: nothing heard while keyed.
                rx[s] = silence if ptt[s] == 1 else new[s]
                if ptt[s] == 1:
                    keyed[s] += 1
            seq += 1
            state["vnow"] += BLOCK_MS
            if seq % 3000 == 0:
                wall_hist.append((state["vnow"] / 1000.0, time.time() - t0))
    finally:
        state["stop"] = True
        for pr in procs:
            pr.terminate()
        for pr in procs:
            try:
                pr.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pr.kill()
        pipe.stdin.close()
        pipe.wait(timeout=30)
        pipe_err.close()

    start = out.get("connect_cmd_s", 0.0)
    end = end_ms / 1000.0 if end_ms else state["vnow"] / 1000.0
    got = sum(n for t, n in out["rx"] if t <= end)
    out.update({
        "bytes": got,
        "bytes_per_min": got / args.minutes,
        "end_s": end,
        "keyed_s": [k * BLOCK_MS / 1000.0 for k in keyed],
        "wall_s": time.time() - t0,
        "wall_hist": wall_hist,
        "events": events,
    })
    connected = [t for t, w, l in events if w == "A" and l.startswith("CONNECTED")]
    out["connected_s"] = connected[0] - start if connected else None
    for name in "AB":
        try:
            with open(os.path.join(args.out, "mercury_%s.txt" % name)) as f:
                text = f.read()
                if "did not drain below high water" in text:
                    out.setdefault("warnings", []).append("%s: demod backpressure cap hit" % name)
        except OSError:
            pass
        try:
            with open(os.path.join(args.out, "timing_%s.log" % name)) as f:
                n = f.read().count("Failed to queue")
                if n:
                    out.setdefault("warnings", []).append("%s: %d frames dropped at the data port" % (name, n))
        except OSError:
            pass
    with open(os.path.join(args.out, "result.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("%s %s dB: %d B in %.1f min = %.0f B/min (connected after %s s, wall %.0f s)%s" % (
        args.mode, "clean" if args.snr is None else "%+g" % args.snr, got, args.minutes, got / args.minutes,
        "%.1f" % out["connected_s"] if out["connected_s"] is not None else "-", out["wall_s"],
        "  ERROR " + out["error"] if "error" in out else ""))


def recv_full_pipe(f, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = f.read(n - len(buf))
        if not chunk:
            raise EOFError("pipe closed")
        buf.extend(chunk)
    return bytes(buf)


if __name__ == "__main__":
    main()
