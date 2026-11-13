#!/usr/bin/env python3
"""Boot an ESP32-S3 flash image in Espressif QEMU and talk AT over emulated UART1.

UART0 (serial_hd(0)) and UART1 (serial_hd(1)) are both TCP server chardevs. QEMU starts
paused (-S); the harness connects to both sockets and QMP, then issues "cont", so no early
boot byte is lost and t=0 is the instant the CPU starts.

Usage: run_sim.py <label> <flash.bin> <efuse.bin> [--idle SECONDS] [--no-begin] [--cmds stock|fork]
"""
import argparse, json, os, socket, subprocess, sys, threading, time

W = os.path.dirname(os.path.abspath(__file__))
QEMU = os.path.join(W, "qemu/bin/qemu-system-xtensa")


def free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p


def connect(port, tries=200):
    for _ in range(tries):
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=1)
        except OSError:
            time.sleep(0.05)
    raise RuntimeError(f"cannot connect to {port}")


class Reader(threading.Thread):
    """Collects every byte from a socket with a timestamp relative to t0."""
    def __init__(self, sock, rawpath, t0ref):
        super().__init__(daemon=True)
        self.sock, self.t0ref = sock, t0ref
        self.chunks = []           # (t, bytes)
        self.lock = threading.Lock()
        self.raw = open(rawpath, "wb")
        self.stop = False

    def run(self):
        self.sock.settimeout(0.05)
        while not self.stop:
            try:
                d = self.sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not d:
                break
            t = time.monotonic() - self.t0ref[0]
            with self.lock:
                self.chunks.append((t, d))
            self.raw.write(d); self.raw.flush()

    def snapshot(self):
        with self.lock:
            return list(self.chunks)

    def total(self):
        with self.lock:
            return b"".join(d for _, d in self.chunks)


def hexdump(b):
    return " ".join(f"{x:02X}" for x in b)


def crc8(data):
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label"); ap.add_argument("flash"); ap.add_argument("efuse")
    ap.add_argument("--idle", type=float, default=30.0, help="seconds of idle after first AT reply")
    ap.add_argument("--boot-timeout", type=float, default=60.0)
    ap.add_argument("--cmds", default="fork")
    ap.add_argument("--extra", default="", help="extra qemu args, space separated")
    ap.add_argument("--probe-ms", type=float, default=250.0)
    ap.add_argument("--seq", default="", help="custom: semicolon-separated commands sent after idle (cmds=custom)")
    a = ap.parse_args()

    out = os.path.join(W, "logs", a.label); os.makedirs(out, exist_ok=True)
    p0, p1, pq = free_port(), free_port(), free_port()
    cmd = [QEMU, "-nographic", "-S", "-M", "esp32s3",
           "-drive", f"file={a.flash},if=mtd,format=raw",
           "-drive", f"file={a.efuse},if=none,format=raw,id=efuse",
           "-global", "driver=nvram.esp32s3.efuse,property=drive,value=efuse",
           "-serial", f"tcp:127.0.0.1:{p0},server=on,wait=off",
           "-serial", f"tcp:127.0.0.1:{p1},server=on,wait=off",
           "-qmp", f"tcp:127.0.0.1:{pq},server=on,wait=off",
           "-monitor", "none",
           "-d", "unimp,guest_errors", "-D", os.path.join(out, "qemu-debug.log")]
    if a.extra:
        cmd += a.extra.split()
    with open(os.path.join(out, "command.txt"), "w") as f:
        f.write(" ".join(cmd) + "\n")
    print("CMD:", " ".join(cmd), flush=True)
    qerr = open(os.path.join(out, "qemu-stderr.log"), "wb")
    proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=qerr, stderr=subprocess.STDOUT)

    s0, s1, sq = connect(p0), connect(p1), connect(pq)
    qf = sq.makefile("rwb")
    def qmp(obj):
        qf.write((json.dumps(obj) + "\n").encode()); qf.flush()
        while True:
            line = qf.readline()
            if not line:
                return None
            r = json.loads(line)
            if "return" in r or "error" in r:
                return r
    qf.readline()  # greeting
    qmp({"execute": "qmp_capabilities"})

    t0ref = [time.monotonic()]
    r0 = Reader(s0, os.path.join(out, "uart0.raw"), t0ref); r0.start()
    r1 = Reader(s1, os.path.join(out, "uart1.raw"), t0ref); r1.start()
    t0ref[0] = time.monotonic()
    print("QMP cont:", qmp({"execute": "cont"}), flush=True)
    t0ref[0] = time.monotonic()

    events = []
    def ev(msg):
        t = time.monotonic() - t0ref[0]
        events.append(f"[{t:8.3f}] {msg}")
        print(events[-1], flush=True)

    # Phase 1: probe "AT\r\n" every 250 ms until any OK/ERROR arrives on UART1.
    first_reply = None
    deadline = time.monotonic() + a.boot_timeout
    probes = 0
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            ev(f"QEMU exited rc={proc.returncode}"); break
        s1.sendall(b"AT\r\n"); probes += 1
        time.sleep(a.probe_ms / 1000.0)
        if b"OK\r\n" in r1.total() or b"ERROR" in r1.total():
            first_reply = r1.snapshot()[0][0]
            break
    if first_reply is None:
        ev(f"no AT reply on UART1 within {a.boot_timeout}s after {probes} probes")
    else:
        ev(f"first UART1 byte at t={first_reply:.3f}s after {probes} probes; drained: {r1.total()!r}")
    time.sleep(1.0)
    pre = len(r1.total())

    results = []
    def at(line, wait=2.0):
        before = len(r1.total())
        ts = time.monotonic()
        s1.sendall(line)
        time.sleep(wait)
        allb = r1.total()
        reply = allb[before:]
        # latency to first reply byte
        lat = None
        for t, d in r1.snapshot():
            if t + t0ref[0] >= ts:
                lat = t + t0ref[0] - ts; break
        results.append({"cmd": line.decode(errors="replace"), "reply_hex": hexdump(reply),
                        "reply_repr": repr(reply), "len": len(reply),
                        "first_byte_ms": None if lat is None else round(lat * 1000, 1),
                        "t_sent": round(ts - t0ref[0], 3)})
        ev(f"SEND {line!r} -> {len(reply)} B {reply!r}")
        return reply

    if first_reply is not None:
        if a.cmds == "fork":
            pre = [b"AT\r\n", b"AT+GMR\r\n", b"AT+FWVERSION?\r\n", b"AT+XBOXVERSION?\r\n",
                   b"AT+XBOXSTATE?\r\n", b"AT+XBOXADDR?\r\n", b"AT+XBOXLOG=1\r\n",
                   b"AT+XBOXLOG?\r\n", b"AT+HCIAVAILABLE?\r\n", b"AT+XBOXFORGET\r\n",
                   b"AT+XBOXSTATE?\r\n"]
            post = [b"AT+FWVERSION?\r\n", b"AT+XBOXBEGIN\r\n", b"AT+XBOXSTATE?\r\n",
                    b"AT+XBOXLOG?\r\n", b"AT+XBOXADDR?\r\n"]
        elif a.cmds == "custom":
            pre = [b"AT+FWVERSION?\r\n"]
            post = [c.encode() + b"\r\n" for c in a.seq.split(";") if c]
        else:
            pre = [b"AT\r\n", b"AT+GMR\r\n", b"AT+FWVERSION?\r\n", b"AT+XBOXVERSION?\r\n",
                   b"AT+XBOXSTATE?\r\n"]
            post = [b"AT+FWVERSION?\r\n", b"AT+HCIAVAILABLE?\r\n", b"AT+FWVERSION?\r\n"]
        for line in pre:
            if proc.poll() is not None:
                ev(f"QEMU exited rc={proc.returncode}"); break
            at(line)
        # Idle phase (no AT traffic), then the post list
        ev(f"idle {a.idle}s")
        u0_before = len(r0.total())
        time.sleep(a.idle)
        ev(f"UART0 bytes during idle: {len(r0.total()) - u0_before}")
        for line in post:
            at(line)
        time.sleep(3)
        at(b"AT+FWVERSION?\r\n")
    else:
        time.sleep(5)

    # dump PC of both CPUs for the record
    try:
        hm = qmp({"execute": "human-monitor-command", "arguments": {"command-line": "info registers -a"}})
        with open(os.path.join(out, "registers.txt"), "w") as f:
            f.write(hm.get("return", str(hm)) if hm else "none")
    except Exception as e:
        ev(f"register dump failed: {e}")
    try:
        qmp({"execute": "quit"})
    except Exception:
        pass
    time.sleep(0.5)
    if proc.poll() is None:
        proc.kill()
    r0.stop = r1.stop = True
    time.sleep(0.2)

    # UART0 log with timestamps per chunk
    with open(os.path.join(out, "uart0-timed.log"), "w") as f:
        for t, d in r0.snapshot():
            f.write(f"[{t:8.3f}] {d.decode('latin-1')}")
    with open(os.path.join(out, "uart1-timed.log"), "w") as f:
        for t, d in r1.snapshot():
            f.write(f"[{t:8.3f}] {d!r}\n")
    with open(os.path.join(out, "events.log"), "w") as f:
        f.write("\n".join(events) + "\n")
    with open(os.path.join(out, "at_results.json"), "w") as f:
        json.dump(results, f, indent=1)
    print("OUT:", out)


if __name__ == "__main__":
    main()
