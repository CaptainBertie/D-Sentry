# fake_esp32.py -- plays the ESP32-S3 side of the UNO R4 WiFi AT link (docs/protocol.md), so the
# RA4M1 sketch can run in Renode with no ESP32 emulator. Two ways to use it:
#
#  1. Inside Renode (IronPython 2.7): a UART line hook on sysbus.sci1 (Serial2, the AT link) calls
#     on_line() for every line the RA4M1 sends; the reply goes straight back into sci1's receiver
#     at the same virtual instant. Times are Renode virtual time. This is what run.sh does.
#  2. As a plain CPython 3 program on the host: python3 fake_esp32.py <port> connects to a Renode
#     server socket terminal wired to sci1 (run.sh ... --socket) and answers over TCP. Times are
#     host time, so this mode also shows the delays a real second emulator would add.
#
# Code shared by both stays Python 2/3 compatible (no f-strings).
#
# Log: FAKE_ESP32_LOG names a file that gets one line per command (polls once per 50 unless
# FAKE_ESP32_LOG_ALL=1).
# Behaviour (the mode comes from the environment variable FAKE_ESP32_MODE):
#   fork   (default) the forked firmware 0.6.99: answers every +XBOX* command; after AT+XBOXBEGIN
#          the scripted controller is scanning (state 3) for 1 s, found (2) for 0.5 s, waiting for
#          the first report (1) for 0.5 s, then connected (0) from 2 s on. While connected the left
#          stick turns a slow circle (one turn per 4 s) and A is held for the first half of every
#          second. One HID report per 20 ms, so seq advances 50 per second.
#   stock  the stock firmware: ERROR to every +XBOX* command, answers AT+FWVERSION?.
#   silent nothing answers (ESP32-S3 not booted).
#
# SPDX-License-Identifier: MIT
import math

try:
    import System            # IronPython inside Renode
    getenv = System.Environment.GetEnvironmentVariable
except ImportError:          # CPython on the host
    System = None
    import os
    getenv = os.environ.get

LOG_PATH = getenv("FAKE_ESP32_LOG")
MODE = getenv("FAKE_ESP32_MODE") or "fork"
LOG_ALL = getenv("FAKE_ESP32_LOG_ALL") == "1"

FW_VERSION = "0.6.99"
PROTOCOL_VERSION = 1
ADDR = "98:7a:14:3c:5e:01"   # made-up controller address
CONNECT_AFTER_MS = 2000.0
STICK_PERIOD_MS = 4000.0
STICK_RADIUS = 30000.0
REPORT_PERIOD_MS = 20.0

state = {"begin_ms": None, "log": 0, "lines": 0}


def now_ms(machine):
    return machine.ElapsedVirtualTime.TimeElapsed.TotalMilliseconds


def log(text):
    if LOG_PATH:
        with open(LOG_PATH, "a") as f:
            f.write(text + "\n")


def crc8(data):
    # CRC-8, polynomial 0x07, init 0x00, no reflection, no xor-out (xbox_r4_protocol.h)
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def u16le(v):
    v = int(max(0, min(65535, v)))
    return [v & 0xFF, (v >> 8) & 0xFF]


def neutral_report():
    return u16le(32767) * 4 + [0] * 8


def conn_state(t):
    if state["begin_ms"] is None:
        return 4
    dt = t - state["begin_ms"]
    if dt < 1000.0:
        return 3
    if dt < 1500.0:
        return 2
    if dt < CONNECT_AFTER_MS:
        return 1
    return 0


def state_bytes(t):
    st = conn_state(t)
    if st != 0:
        report, battery, seq = neutral_report(), 0xFF, 0
    else:
        dt = t - state["begin_ms"] - CONNECT_AFTER_MS
        angle = 2.0 * math.pi * dt / STICK_PERIOD_MS
        x = 32768.0 + STICK_RADIUS * math.cos(angle)
        y = 32768.0 - STICK_RADIUS * math.sin(angle)   # wire Y: 0 = up
        a_held = (dt % 1000.0) < 500.0
        report = u16le(x) + u16le(y) + u16le(32767) + u16le(32767) + u16le(0) + u16le(0)
        report += [0, 0x01 if a_held else 0x00, 0, 0]
        battery = 87
        seq = int(dt / REPORT_PERIOD_MS) & 0xFF
    body = report + [st, battery, seq]
    return body + [crc8(body)]


def reply_for(cmd, t):
    """Returns the reply as a list of byte values, or None for no reply."""
    if MODE == "silent":
        return None
    if cmd == "AT+SOFTRESETWIFI":
        return text("+SOFTRESETWIFI: \r\nOK\r\n")
    if cmd == "AT+FWVERSION?":
        return text("+FWVERSION: %s\r\nOK\r\n" % (FW_VERSION if MODE == "fork" else "0.6.0"))
    if MODE == "stock" or not cmd.startswith("AT+XBOX"):
        return text("ERROR\r\n")
    if cmd == "AT+XBOXVERSION?":
        return text("+XBOXVERSION: %d,%s\r\nOK\r\n" % (PROTOCOL_VERSION, FW_VERSION))
    if cmd == "AT+XBOXBEGIN":
        if state["begin_ms"] is None:
            state["begin_ms"] = t
        return text("+XBOXBEGIN: 0\r\nOK\r\n")
    if cmd == "AT+XBOXSTATE?":
        return text("+XBOXSTATE: 20|") + state_bytes(t) + text("\r\nOK\r\n")
    if cmd == "AT+XBOXADDR?":
        return text("+XBOXADDR: %s\r\nOK\r\n" % (ADDR if state["begin_ms"] is not None else "00:00:00:00:00:00"))
    if cmd == "AT+XBOXFORGET":
        return text("+XBOXFORGET: 0\r\nOK\r\n")
    if cmd.startswith("AT+XBOXLOG="):
        value = 1 if cmd[len("AT+XBOXLOG="):].strip() not in ("", "0") else 0
        state["log"] = value
        return text("+XBOXLOG: %d\r\nOK\r\n" % value)
    return text("ERROR\r\n")


def text(s):
    return [ord(c) for c in s]


def handle(line, t):
    """One line from the RA4M1 at time t (ms); returns the reply bytes or None."""
    cmd = line.strip()
    i = cmd.find("AT")
    if i < 0:
        return None
    cmd = cmd[i:]
    reply = reply_for(cmd, t)
    state["lines"] += 1
    # Log every command except the 50 Hz poll, which is logged once per 50 polls.
    if LOG_ALL or cmd != "AT+XBOXSTATE?" or state["lines"] % 50 == 0:
        shown = "(none)" if reply is None else " ".join("%02X" % b for b in reply)
        log("%10.1f ms  %-18s -> %s" % (t, cmd, shown))
    return reply


def on_line(machine, uart, line):
    """Renode UART line hook entry point (IronPython)."""
    reply = handle(line, now_ms(machine))
    if reply is not None:
        for b in reply:
            uart.WriteChar(System.Byte(b))


def main(argv):
    """CPython entry point: connect to Renode's server socket terminal on sci1 and answer."""
    import socket
    import time
    port = int(argv[1]) if len(argv) > 1 else 3456
    t0 = time.time()
    sock = None
    while sock is None:
        try:
            sock = socket.create_connection(("127.0.0.1", port))
        except OSError:
            if time.time() - t0 > 60:
                raise
            time.sleep(0.2)
    t0 = time.time()
    buf = b""
    while True:
        data = sock.recv(4096)
        if not data:
            return 0
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            reply = handle(line.decode("latin-1"), (time.time() - t0) * 1000.0)
            if reply is not None:
                sock.sendall(bytes(bytearray(reply)))


if __name__ == "__main__":
    import sys
    sys.exit(main(sys.argv))
