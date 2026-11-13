# SPDX-License-Identifier: MIT
"""dsentry.transports over real pipes: a pty pair through real pyserial (exact baud,
write_timeout, frames and text both ways, monitor never writes), reopen after every kind of
error, loopback UDP against a fake turret (reply to sender, foreign senders ignored), beacon
discovery and the AP-subnet fallback, and --sim spawning a stand-in simulator.

Key safety property: test_monitor_never_writes fails if --monitor (BT mode) could put a second
control source on the USB port."""

import os
import pty
import select
import signal
import socket
import subprocess
import sys
import tempfile
import termios
import textwrap
import threading
import time
import types
import unittest
from pathlib import Path

import _support as S

from dsentry import protocol as P
from dsentry import settings as ST
from dsentry import transports as T

try:
    import serial
    HAVE_SERIAL = True
except ImportError:      # pragma: no cover
    HAVE_SERIAL = False

SET = ST.Settings.load(S.CONFIG_H)
WAIT = 5.0


class Sink:
    def __init__(self):
        self.cond = threading.Condition()
        self.frames, self.texts, self.notes = [], [], []

    def on_frame(self, frame, via=""):
        with self.cond:
            self.frames.append(frame)
            self.cond.notify_all()

    def on_text(self, text, via=""):
        with self.cond:
            self.texts.append(text)
            self.cond.notify_all()

    def note(self, kind, text):
        with self.cond:
            self.notes.append(text)
            self.cond.notify_all()

    def wait(self, pred, timeout=WAIT):
        with self.cond:
            return self.cond.wait_for(lambda: pred(self), timeout)


def telemetry_frame(**kw):
    return P.TelemetryPacket(**kw).encode()


def control_frame(seq=1, flags=P.DS_CTRL_FLAG_CONNECTED):
    return P.ControlPacket(flags=flags, seq=seq).encode()


def read_master(fd, seconds):
    out = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], 0.05)
        if r:
            try:
                out += os.read(fd, 4096)
            except OSError:
                break
    return bytes(out)


class DetectPortTests(unittest.TestCase):
    @staticmethod
    def port(device, vid, pid):
        return types.SimpleNamespace(device=device, vid=vid, pid=pid)

    def test_one_match(self):
        ports = [self.port("/dev/cu.Bluetooth", None, None), self.port("/dev/cu.usbmodemA", 0x2341, 0x1002),
                 self.port("/dev/cu.usbmodemB", 0x303A, 0x1001)]
        self.assertEqual(T.detect_port(ports), "/dev/cu.usbmodemA")

    def test_none_or_several(self):
        with self.assertRaisesRegex(T.PortError, "no UNO R4 WiFi"):
            T.detect_port([self.port("/dev/cu.x", 1, 2)])
        with self.assertRaisesRegex(T.PortError, "several"):
            T.detect_port([self.port("/dev/a", 0x2341, 0x1002), self.port("/dev/b", 0x2341, 0x1002)])


@unittest.skipUnless(HAVE_SERIAL, "pyserial not installed")
class SerialPtyTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.name = os.ttyname(self.slave)
        self.stop = threading.Event()
        self.sink = Sink()

    def tearDown(self):
        self.stop.set()
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def start(self, **kw):
        tr = T.SerialTransport(SET, self.sink, port=self.name, **kw)
        tr.start(self.stop)
        self.assertTrue(self.sink.wait(lambda s: any("open at" in n for n in s.notes)), self.sink.notes)
        return tr

    def test_round_trip_frames_and_text(self):
        tr = self.start()
        link = tr._link
        self.assertEqual(link.baudrate, SET.serial_baud)                       # exactly DEBUG_SERIAL_BAUD
        self.assertAlmostEqual(link.write_timeout, SET.serial_write_timeout_s)
        self.assertTrue(tr.send(control_frame(seq=42)))
        got = P.Deframer().feed(read_master(self.master, 0.3))
        self.assertEqual([f.packet.seq for f in got], [42])
        stream = (b"[2] STATE PAIRING -> DISARMED\r\n" + telemetry_frame(seq=9, ammo=7) + b"Zz ZAP\n"
                  + P.StatusPacket(rumbleSeq=3).encode() + b"tail without newline")
        for i in range(0, len(stream), 7):                       # split reads
            os.write(self.master, stream[i:i + 7])
        self.assertTrue(self.sink.wait(lambda s: len(s.frames) >= 2 and len(s.texts) >= 2))
        self.assertEqual(self.sink.texts[:2], ["[2] STATE PAIRING -> DISARMED", "Zz ZAP"])
        self.assertEqual([f.type for f in self.sink.frames], [P.PacketType.TELEMETRY, P.PacketType.STATUS])
        self.assertEqual(self.sink.frames[0].packet.ammo, 7)
        self.assertEqual(tr.crc_errors, 0)

    def test_corrupt_frame_counted(self):
        tr = self.start()
        bad = bytearray(telemetry_frame(seq=1))
        bad[20] ^= 0x40
        os.write(self.master, bytes(bad) + telemetry_frame(seq=2))
        self.assertTrue(self.sink.wait(lambda s: s.frames))
        self.assertEqual([f.packet.seq for f in self.sink.frames], [2])
        self.assertEqual(tr.crc_errors, 1)

    def test_monitor_never_writes(self):
        tr = self.start(read_only=True)
        self.assertFalse(tr.sends)
        for seq in range(20):
            self.assertFalse(tr.send(control_frame(seq=seq)))
        self.assertEqual(read_master(self.master, 0.3), b"")
        os.write(self.master, telemetry_frame(seq=5))             # but it reads
        self.assertTrue(self.sink.wait(lambda s: s.frames))


class FakePort:
    """Stands in for serial.Serial: hands out chunks, then fails like an unplugged board."""

    def __init__(self, chunks, error, write_error=None):
        self.chunks = list(chunks)
        self.error = error
        self.write_error = write_error
        self.written = []
        self.cancelled = threading.Event()
        self.closed = False
        self.in_waiting = 0

    def read(self, size):
        if self.cancelled.is_set():
            return b""
        if not self.chunks:
            if self.error is None:
                time.sleep(0.01)
                return b""
            raise self.error
        return self.chunks.pop(0)

    def write(self, data):
        if self.write_error is not None:
            raise self.write_error
        self.written.append(bytes(data))
        return len(data)

    def cancel_read(self):
        self.cancelled.set()

    def close(self):
        self.closed = True


@unittest.skipUnless(HAVE_SERIAL, "pyserial not installed")
class SerialReopenTests(unittest.TestCase):
    def setUp(self):
        self.stop = threading.Event()
        self.sink = Sink()
        self.opened = []

    def tearDown(self):
        self.stop.set()

    def run_transport(self, open_port, reopen_s=0.01):
        fast = ST.Settings.from_text(S.CONFIG_H.read_text().replace(
            "#define RELAY_REOPEN_MS", "#define RELAY_REOPEN_MS 10 //", 1))
        self.assertAlmostEqual(fast.reopen_s, reopen_s)
        tr = T.SerialTransport(fast, self.sink, port="/dev/fake", open_port=open_port)
        tr.start(self.stop)
        return tr

    def test_unplug_busy_then_back(self):
        def open_port(name):
            self.opened.append(name)
            if len(self.opened) == 1:
                return FakePort([telemetry_frame(seq=1)], serial.SerialException("device disconnected"))
            if len(self.opened) == 2:
                raise serial.SerialException("port busy")
            return FakePort([telemetry_frame(seq=2)], None)
        self.run_transport(open_port)
        self.assertTrue(self.sink.wait(lambda s: len(s.frames) == 2))
        self.assertEqual(self.opened, ["/dev/fake"] * 3)
        joined = "\n".join(self.sink.notes)
        self.assertIn("device disconnected", joined)
        self.assertIn("port busy", joined)

    def test_termios_and_other_errors_are_retried(self):
        # termios.error is not an OSError; pyserial lets it out when a USB device is half
        # re-enumerated. It must not kill the reader thread.
        def open_port(name):
            self.opened.append(name)
            if len(self.opened) == 1:
                raise termios.error(6, "Device not configured")
            if len(self.opened) == 2:
                raise ImportError("No module named 'serial'")
            return FakePort([telemetry_frame(seq=3)], None)
        self.run_transport(open_port)
        self.assertTrue(self.sink.wait(lambda s: s.frames))
        self.assertTrue(any("error" in n for n in self.sink.notes))

    def test_write_timeout_closes_and_reopens(self):
        ports = []

        def open_port(name):
            self.opened.append(name)
            p = FakePort([], None, write_error=serial.SerialTimeoutException("Write timeout") if not ports else None)
            ports.append(p)
            return p
        tr = self.run_transport(open_port)
        self.assertTrue(self.sink.wait(lambda s: any("open at" in n for n in s.notes)))
        self.assertFalse(tr.send(control_frame()))                # the bridge stalled
        self.assertTrue(ports[0].cancelled.wait(1.0), "a failed write must wake the reader")
        self.assertTrue(self.sink.wait(lambda s: len(self.opened) >= 2))
        self.assertTrue(self.sink.wait(lambda s: ports[0].closed))
        deadline = time.monotonic() + WAIT
        while not tr.send(control_frame(seq=7)) and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(P.ControlPacket.decode(ports[1].written[-1]).seq, 7)
        self.assertTrue(any("Write timeout" in n for n in self.sink.notes))

    def test_send_while_closed_is_dropped(self):
        tr = T.SerialTransport(SET, self.sink, port="/dev/none")
        self.assertFalse(tr.send(control_frame()))
        self.assertEqual(tr.dropped, 1)


class FakeTurret:
    """UDP turret: answers each valid ControlPacket's source address with a TelemetryPacket
    (echoing seq as lastCtrlSeq) and a StatusPacket, like the ESP32 in WiFi mode."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.peers = set()
        self.got = []
        self.stop = threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while not self.stop.is_set():
            try:
                data, peer = self.sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                pkt = P.ControlPacket.decode(data)
            except ValueError:
                continue
            self.got.append(pkt)
            self.peers.add(peer)
            self.sock.sendto(telemetry_frame(seq=len(self.got) & 0xFF, lastCtrlSeq=pkt.seq,
                                             linkSource=P.DS_LINK_SRC_WIFI), peer)
            self.sock.sendto(P.StatusPacket(rumbleSeq=1).encode(), peer)

    def close(self):
        self.stop.set()
        self.sock.close()


class UdpTests(unittest.TestCase):
    def setUp(self):
        self.stop = threading.Event()
        self.sink = Sink()
        self.turret = FakeTurret()

    def tearDown(self):
        self.stop.set()
        self.turret.close()

    def test_explicit_target_and_reply_to_sender(self):
        tr = T.UdpTransport(SET, self.sink, target=("127.0.0.1", self.turret.port), bind_host="127.0.0.1")
        tr.start(self.stop)
        for seq in range(10):
            self.assertTrue(tr.send(control_frame(seq=seq)))
            time.sleep(0.005)
        self.assertTrue(self.sink.wait(lambda s: len(s.frames) >= 20))
        self.assertEqual(self.turret.peers, {("127.0.0.1", tr.local_port())})
        self.assertEqual(sorted(f.packet.lastCtrlSeq for f in self.sink.frames if f.type == P.PacketType.TELEMETRY),
                         list(range(10)))
        self.assertEqual(tr.how, "explicit")
        tr.close()

    def test_foreign_datagrams_ignored(self):
        tr = T.UdpTransport(SET, self.sink, target=("192.0.2.1", SET.udp_port), bind_host="127.0.0.1")
        tr.start(self.stop)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as other:
            for _ in range(3):
                other.sendto(telemetry_frame(seq=1), ("127.0.0.1", tr.local_port()))
        deadline = time.monotonic() + 1.0
        while tr.foreign < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(tr.foreign, 3)
        self.assertEqual(self.sink.frames, [])
        tr.close()

    def test_no_target_sends_nothing(self):
        tr = T.UdpTransport(SET, self.sink, bind_host="127.0.0.1", discovery=False, ap_fallback=False)
        tr.start(self.stop)
        self.assertFalse(tr.send(control_frame()))
        self.assertEqual(self.turret.got, [])
        tr.close()

    def test_beacon_discovery_and_retarget(self):
        disc = T.free_udp_port()
        tr = T.UdpTransport(SET, self.sink, bind_host="127.0.0.1", discovery_port=disc, ap_fallback=False)
        tr.start(self.stop)
        self.assertTrue(tr.discovery_bound.wait(WAIT))
        self.assertIsNone(tr.target)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as b:
            b.sendto(b"\xd6\x08garbage!!!", ("127.0.0.1", disc))                   # bad CRC: ignored
            b.sendto(P.BeaconPacket(ip=(127, 0, 0, 1), udpPort=self.turret.port, bootId=9, mode=3).encode(),
                     ("127.0.0.1", disc))
            self.assertTrue(self.sink.wait(lambda s: tr.target is not None))
            self.assertEqual((tr.target, tr.how), (("127.0.0.1", self.turret.port), "beacon"))
            tr.send(control_frame(seq=5))
            self.assertTrue(self.sink.wait(lambda s: s.frames))
            other = FakeTurret()
            try:
                b.sendto(P.BeaconPacket(ip=(127, 0, 0, 1), udpPort=other.port, bootId=10, mode=2).encode(),
                         ("127.0.0.1", disc))
                self.assertTrue(self.sink.wait(lambda s: tr.target == ("127.0.0.1", other.port)))
            finally:
                other.close()
        self.assertEqual(tr.beacon["bootId"], 10)
        tr.close()

    def test_ap_fallback_only_on_the_ap_subnet(self):
        ap_host = SET.ap_ip.rsplit(".", 1)[0] + ".23"
        tr = T.UdpTransport(SET, self.sink, bind_host="127.0.0.1", discovery=False, local_ip=lambda h: ap_host)
        tr.start(self.stop)
        self.assertTrue(self.sink.wait(lambda s: tr.target is not None))
        self.assertEqual((tr.target, tr.how), ((SET.ap_ip, SET.udp_port), "ap"))
        tr.close()
        for mine in ("10.0.0.5", None):
            sink = Sink()
            tr = T.UdpTransport(SET, sink, bind_host="127.0.0.1", discovery=False, local_ip=lambda h, m=mine: m)
            tr.start(self.stop)
            time.sleep(0.3)
            self.assertIsNone(tr.target, mine)
            tr.close()

    def test_helpers(self):
        self.assertEqual(T.parse_hostport("10.1.2.3", 4210), ("10.1.2.3", 4210))
        self.assertEqual(T.parse_hostport("turret.local:99", 4210), ("turret.local", 99))
        self.assertEqual(T.parse_hostport("[::1]:5", 4210), ("::1", 5))
        for bad in ("host:0", "host:70000", ":5", "host:x"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                T.parse_hostport(bad, 4210)
        self.assertTrue(T.on_subnet("192.168.4.200", "192.168.4.1", "255.255.255.0"))
        self.assertFalse(T.on_subnet("192.168.5.2", "192.168.4.1", "255.255.255.0"))
        self.assertFalse(T.on_subnet(None, "192.168.4.1", "255.255.255.0"))


# ---- --sim with a stand-in simulator ----------------------------------------------------------
STANDIN = textwrap.dedent('''\
    # stand-in for dsentry_sim (the C++ sim is finished in a later stage): --udp-port N, then acts
    # as the ESP32 in WiFi mode on 127.0.0.1:N and prints RA4M1-style text on stdout.
    import signal, socket, sys
    sys.path.insert(0, {relay!r})
    from dsentry import protocol as P
    port = int(sys.argv[sys.argv.index("--udp-port") + 1])
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", port))
    s.settimeout(0.05)
    print("D-Sentry RA4M1 boot (stand-in) on UDP", port, flush=True)
    stop = []
    signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
    n = 0
    while not stop:
        try:
            data, peer = s.recvfrom(256)
        except socket.timeout:
            continue
        except OSError:
            break
        for f in P.Deframer().feed(data):
            n += 1
            s.sendto(P.TelemetryPacket(seq=n & 0xFF, lastCtrlSeq=f.packet.seq, linkSource=P.DS_LINK_SRC_WIFI,
                                       state=P.DS_STATE_DISARMED, lcd="STAND-IN").encode(), peer)
    print("stand-in stopped after", n, "packets", flush=True)
''')


def write_standin(directory):
    """An executable stand-in: a /bin/sh wrapper (a shebang cannot hold the venv path, which has a
    space in it) that runs the Python stand-in with the same interpreter as the tests."""
    script = Path(directory) / "fake_sim.py"
    script.write_text(STANDIN.format(relay=str(S.RELAY)))
    path = Path(directory) / "fake_sim"
    path.write_text('#!/bin/sh\nexec "%s" "%s" "$@"\n' % (sys.executable, script))
    path.chmod(0o755)
    return path


class SimTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.bin = write_standin(self.tmp.name)
        self.stop = threading.Event()
        self.sink = Sink()

    def tearDown(self):
        self.stop.set()
        self.tmp.cleanup()

    def test_spawn_talk_and_stop(self):
        sim = T.SimProcess(SET, self.sink, binary=str(self.bin), port=0)
        self.assertEqual(sim.command()[1:3], ["--udp-port", "0"])
        sim.start()
        try:
            self.assertNotEqual(sim.port, 0)
            self.assertEqual(sim.command()[1:3], ["--udp-port", str(sim.port)])
            tr = T.UdpTransport(SET, self.sink, target=(T.LOOPBACK, sim.port), bind_host=T.LOOPBACK,
                                discovery=False, ap_fallback=False)
            tr.start(self.stop)
            deadline = time.monotonic() + WAIT
            seq = 0
            while not self.sink.frames and time.monotonic() < deadline:
                tr.send(control_frame(seq=seq & 0xFF))
                seq += 1
                time.sleep(0.02)
            self.assertTrue(self.sink.frames, "no telemetry from the stand-in")
            self.assertTrue(self.sink.wait(lambda s: any("boot (stand-in)" in t for t in s.texts)))
            tr.close()
        finally:
            sim.stop()
        self.assertIsNotNone(sim.returncode())
        self.assertTrue(self.sink.wait(lambda s: any("stand-in stopped" in t for t in s.texts)))

    def test_env_var_selects_the_binary(self):
        old = os.environ.get(T.SIM_BIN_ENV)
        os.environ[T.SIM_BIN_ENV] = str(self.bin)
        try:
            self.assertEqual(T.SimProcess(SET, self.sink).binary, self.bin)
        finally:
            if old is None:
                del os.environ[T.SIM_BIN_ENV]
            else:
                os.environ[T.SIM_BIN_ENV] = old
        self.assertEqual(T.SimProcess(SET, self.sink, binary=None).port, SET.udp_port)

    def test_missing_binary_is_a_clear_error(self):
        sim = T.SimProcess(SET, self.sink, binary=str(Path(self.tmp.name) / "nope"))
        with self.assertRaisesRegex(T.SimError, "sim/build.sh"):
            sim.start()

    def test_relay_cli_with_sim(self):
        port = free_tcp_port()
        proc = subprocess.Popen(
            [sys.executable, str(S.RELAY / "dsentry_relay.py"), "--sim", "--sim-bin", str(self.bin),
             "--sim-port", "0", "--no-pad", "--no-browser", "--quiet", "--http-port", str(port)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            env=dict(os.environ, PYTHONUNBUFFERED="1", PYTHONDONTWRITEBYTECODE="1"))
        try:
            banner = proc.stdout.readline()
            self.assertIn("mode: sim", banner)
            import json
            import urllib.request
            state = None
            deadline = time.monotonic() + WAIT
            while time.monotonic() < deadline:
                with urllib.request.urlopen("http://127.0.0.1:%d/state" % port, timeout=2) as r:
                    state = json.loads(r.read())
                if state["telemetryState"] == "live":
                    break
                time.sleep(0.1)
            self.assertEqual(state["telemetryState"], "live")
            self.assertEqual(state["telemetry"]["lcdRows"][0], "STAND-IN        ")
            self.assertEqual(state["mode"], "sim")
            self.assertTrue(any("boot (stand-in)" in t["text"] for t in state["text"]))
            self.assertIsNotNone(state["relay"]["latencyMs"])
        finally:
            proc.send_signal(signal.SIGTERM)
            out, err = proc.communicate(timeout=10)
        self.assertEqual(proc.returncode, 0, err)
        self.assertIn("relay: stopped", out)
        self.assertNotIn("Traceback", err)


def free_tcp_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


if __name__ == "__main__":
    unittest.main()
