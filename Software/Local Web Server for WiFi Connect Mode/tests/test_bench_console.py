# SPDX-License-Identifier: MIT
"""tools/bench_console.py against a pty pair standing in for the RA4M1's USB port: neutral
connected ControlPackets at CONTROL_PACKET_HZ, tap/hold/stick/trigger/connect commands, '!'
shell lines, printing of the board's text and decoded Status/Telemetry, a clean exit that leaves
the turret with connected=0, and the command parser on its own."""

import io
import os
import pty
import select
import subprocess
import sys
import threading
import time
import unittest

import _support as S

from dsentry import protocol as P
from dsentry import settings as ST

SET = ST.Settings.load(S.CONFIG_H)
CONSOLE = S.TOOLS / "bench_console.py"
bench = S.load_module("bench_console_under_test", CONSOLE)


class Board:
    """The 'RA4M1' end of the pty: records ControlPackets (with arrival times) and text lines."""

    def __init__(self, master):
        self.master = master
        self.deframer = P.Deframer()
        self.packets = []
        self.junk = bytearray()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(2)

    def run(self):
        while not self.stop.is_set():
            try:
                r, _, _ = select.select([self.master], [], [], 0.05)
            except (OSError, ValueError):
                return
            if not r:
                continue
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            now = time.monotonic()
            with self.lock:
                for item in self.deframer.feed_ordered(data):
                    if isinstance(item, P.Frame):
                        self.packets.append((now, item.packet))
                    else:
                        self.junk += item

    def since(self, t0):
        with self.lock:
            return [p for t, p in self.packets if t >= t0]

    def write(self, data):
        os.write(self.master, data)


class ConsoleProcessTests(unittest.TestCase):
    def setUp(self):
        self.master, self.slave = pty.openpty()
        self.board = Board(self.master)
        self.proc = subprocess.Popen([sys.executable, str(CONSOLE), "--port", os.ttyname(self.slave)],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, bufsize=1, env=dict(os.environ, PYTHONUNBUFFERED="1"))
        self.out = []
        self.out_lock = threading.Condition()
        threading.Thread(target=self._read_out, daemon=True).start()
        self.assertTrue(self.wait_out("open at"), self.out)

    def _read_out(self):
        for line in self.proc.stdout:
            with self.out_lock:
                self.out.append(line.rstrip("\n"))
                self.out_lock.notify_all()

    def wait_out(self, needle, timeout=5.0):
        with self.out_lock:
            return self.out_lock.wait_for(lambda: any(needle in l for l in self.out), timeout)

    def send(self, line):
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()

    def tearDown(self):
        if self.proc.poll() is None:
            self.proc.kill()
        self.proc.communicate(timeout=5)
        self.board.close()
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def test_session(self):
        t0 = time.monotonic()
        time.sleep(1.0)
        pkts = self.board.since(t0)
        self.assertGreater(len(pkts), SET.control_hz * 0.8)
        self.assertTrue(all(p.flags & P.DS_CTRL_FLAG_CONNECTED for p in pkts))
        self.assertTrue(all((p.rx, p.ry, p.rt, p.buttons) == (0, 0, 0, 0) for p in pkts))
        seqs = [p.seq for p in pkts]
        self.assertEqual(seqs, [(seqs[0] + i) % 256 for i in range(len(seqs))])

        # tap a: A held for about KEYBOARD_PRESS_MS, then released
        t1 = time.monotonic()
        self.send("tap a")
        self.assertTrue(self.wait_out("# tap a"))
        time.sleep(0.4)
        with_a = [p for p in self.board.since(t1) if p.buttons & P.DS_BTN_A]
        expected = SET.keyboard_press_s * SET.control_hz
        self.assertAlmostEqual(len(with_a), expected, delta=expected * 0.4 + 2)
        self.assertFalse(self.board.since(time.monotonic() - 0.05)[-1].buttons & P.DS_BTN_A)

        # stick and trigger (y up positive on the console, up negative on the wire)
        self.send("stick 1 1")
        self.send("trigger 0.5")
        self.send("hold lb")
        self.assertTrue(self.wait_out("# hold lb"))
        time.sleep(0.1)
        p = self.board.since(time.monotonic() - 0.03)[-1]
        self.assertEqual((p.rx, p.ry, p.rt), (SET.joy_axis_range - 1, -SET.joy_axis_range,
                                              round(0.5 * SET.trigger_axis_range)))
        self.assertTrue(p.buttons & P.DS_BTN_LB)
        self.send("neutral")
        self.assertTrue(self.wait_out("# neutral"))

        # shell passthrough reaches the board as one text line
        self.send("!status")
        self.assertTrue(self.wait_out("sent shell line: status"))
        time.sleep(0.1)
        with self.board.lock:
            self.assertIn(b"status\n", bytes(self.board.junk))

        # the board's text and packets are printed
        self.board.write(b"[2] hello from the board\r\n")
        self.board.write(P.TelemetryPacket(state=P.DS_STATE_DISARMED, ammo=15, magCapacity=15,
                                           lcd="DISARMED AMMO:15BT     Connected").encode())
        self.board.write(P.StatusPacket(wantMode=0, state=P.DS_STATE_DISARMED, rumbleSeq=1).encode())
        self.assertTrue(self.wait_out("< [2] hello from the board"))
        self.assertTrue(self.wait_out("telemetry: DISARMED"))
        self.assertTrue(self.wait_out("status: want BLE state DISARMED"))
        self.send("status")
        self.assertTrue(self.wait_out("telemetry (full):"))

        # refused commands do not stop the console
        self.send("warp 9")
        self.assertTrue(self.wait_out("? unknown command 'warp'"))
        self.send("stick 2 0")
        self.assertTrue(self.wait_out("? stick values are -1..1"))

        # disconnect, then quit: the last packet leaves the turret disconnected
        self.send("disconnect")
        self.assertTrue(self.wait_out("# connected=0"))
        time.sleep(0.1)
        self.assertFalse(self.board.since(time.monotonic() - 0.03)[-1].flags & P.DS_CTRL_FLAG_CONNECTED)
        self.send("connect")
        self.send("quit")
        self.assertEqual(self.proc.wait(5), 0)
        time.sleep(0.1)
        last = self.board.since(0)[-1]
        self.assertFalse(last.flags & P.DS_CTRL_FLAG_CONNECTED, "the console must leave connected=0 behind")


class ParserTests(unittest.TestCase):
    def setUp(self):
        self.out = io.StringIO()
        self.printer = bench.Printer(SET, out=self.out)
        self.inp = bench.ConsoleInput(SET)
        self.sent = []
        self.transport = type("T", (), {"send": lambda _, data: self.sent.append(data) or True})()

    def run_cmd(self, line):
        return bench.run_command(line, self.inp, self.printer, self.transport, SET)

    def test_commands(self):
        self.assertTrue(self.run_cmd("hold rt"))
        self.assertEqual(self.inp.build(0.0).rt, SET.trigger_axis_range)
        self.run_cmd("release rt")
        self.assertEqual(self.inp.build(0.0).rt, 0)
        self.run_cmd("hold down")
        self.assertEqual(self.inp.build(0.0).buttons, P.DS_BTN_DPAD_DOWN)
        self.run_cmd("release all")
        self.assertEqual(self.inp.build(0.0).buttons, 0)
        self.run_cmd("disconnect")
        self.assertFalse(self.inp.build(0.0).connected)
        self.run_cmd("!pan 30")
        self.assertEqual(self.sent, [b"pan 30\n"])
        self.assertFalse(self.run_cmd("quit"))

    def test_long_shell_line_refused(self):
        limit = SET.value("SHELL_LINE_MAX")
        self.run_cmd("!" + "x" * (limit + 1))
        self.assertEqual(self.sent, [])
        self.assertIn("SHELL_LINE_MAX", self.out.getvalue())

    def test_bad_input(self):
        for line in ("tap", "tap zz", "stick 1", "trigger 2", "watch maybe", "frobnicate"):
            with self.subTest(line=line):
                self.assertTrue(self.run_cmd(line))
        self.assertEqual(self.out.getvalue().count("? "), 6)

    def test_an_older_boards_telemetry_is_read_through_the_link_reader(self):
        # an RA4M1 build from before the controller block sends version 1 (91-byte payload)
        from dsentry import framing
        old = P.TelemetryPacket(version=1, state=P.DS_STATE_DISARMED, lcd="DISARMED")
        raw = P._frame(P.DS_TELEMETRY_START, old.payload()[:P.DS_TELEMETRY_V1_LEN])
        items = framing.LinkReader().feed(b"boot text\r\n" + raw)
        frames = [i for i in items if isinstance(i, P.Frame)]
        self.assertEqual(len(frames), 1)
        self.printer.on_frame(frames[0])
        self.assertIn("telemetry: DISARMED", self.out.getvalue())
        self.assertNotIn("ignored", self.out.getvalue())
        bad = P.TelemetryPacket(version=7)
        self.printer.on_frame(P.Frame(P.PacketType.TELEMETRY, bad.encode(), bad))
        self.assertIn("version 7, expected %d: ignored" % P.DS_TELEMETRY_VERSION, self.out.getvalue())


if __name__ == "__main__":
    unittest.main()
