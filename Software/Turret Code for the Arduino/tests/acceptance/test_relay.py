# SPDX-License-Identifier: MIT
"""The PC tools against dsentry_sim, as the owner runs them:
  - `dsentry_relay.py --sim` spawns the simulator itself, serves /events with the sim's live
    telemetry, and the dashboard's keyboard POST path pans the simulated turret;
  - `tools/bench_console.py --port <pty>` sends a shell line through the sim's pseudo-terminal and
    gets the RA4M1's reply; its ControlPackets take the turret to DISARMED over USB;
  - `dsentry_relay.py --serial <pty>` shows the sim's USB telemetry live."""

import http.client
import json
import os
import signal
import subprocess
import threading
import time
import unittest
import urllib.error
import urllib.request

import _sim as S
from _sim import P, cfg

RELAY_PY = S.RELAY / "dsentry_relay.py"
CONSOLE_PY = S.TOOLS / "bench_console.py"
ENV = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONDONTWRITEBYTECODE="1")
TOKEN_HEADER = "X-DSentry-Token"


class Relay:
    def __init__(self, *args) -> None:
        self.port = S.free_tcp_port()
        self.proc = subprocess.Popen([S.PYTHON, str(RELAY_PY), *args, "--no-pad", "--no-browser", "--quiet",
                                      "--http-port", str(self.port)],
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=ENV)
        self.base = "http://127.0.0.1:%d" % self.port

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as r:
            return json.loads(r.read())

    def post(self, path, body, token):
        req = urllib.request.Request(self.base + path, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json", TOKEN_HEADER: token or ""})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"{}")

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        try:
            out, err = self.proc.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            out, err = self.proc.communicate()
        return self.proc.returncode, out, err


class Events:
    """GET /events on a thread: every SSE data frame, in order."""

    def __init__(self, port: int) -> None:
        self.frames = []
        self.cond = threading.Condition()
        self.conn = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        self.conn.request("GET", "/events")
        self.resp = self.conn.getresponse()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self) -> None:
        try:
            while True:
                line = self.resp.fp.readline()
                if not line:
                    return
                if line.startswith(b"data: "):
                    with self.cond:
                        self.frames.append(json.loads(line[6:]))
                        self.cond.notify_all()
        except (OSError, ValueError):
            return

    def mark(self) -> int:
        with self.cond:
            return len(self.frames)

    def wait(self, pred, timeout: float, after: int = 0, what: str = "frame"):
        end = time.monotonic() + timeout
        with self.cond:
            i = after
            while True:
                while i < len(self.frames):
                    if pred(self.frames[i]):
                        return self.frames[i]
                    i += 1
                left = end - time.monotonic()
                if left <= 0:
                    last = self.frames[-1] if self.frames else None
                    raise AssertionError("%s not seen in %.1f s; last frame: %s" % (
                        what, timeout, json.dumps(last)[:800] if last else None))
                self.cond.wait(min(left, 0.1))

    def since(self, mark: int):
        with self.cond:
            return list(self.frames[mark:])

    def close(self) -> None:
        self.conn.close()


def tel(frame):
    return frame.get("telemetry") if frame.get("telemetryState") == "live" else None


class RelaySimTests(unittest.TestCase):
    SPEED = 2   # the relay's own timing (sender, keyboard deadman) is wall-clock

    def test_relay_spawns_the_sim_and_the_keyboard_pans_it(self):
        relay = Relay("--sim", "--sim-bin", S.sim_binary(), "--sim-port", "0",
                      "--sim-arg=--speed", "--sim-arg=%g" % self.SPEED)
        events = None
        try:
            banner = relay.proc.stdout.readline()
            self.assertIn("mode: sim", banner)
            events = Events(relay.port)
            boot_s = (cfg("TILT_CAL_MAX_TRAVEL_DEG") / cfg("TILT_CAL_SPEED_DEG_PER_S") + 3.0) / self.SPEED
            f = events.wait(lambda f: tel(f) is not None, boot_s + S.WALL_SLACK_S, what="live telemetry")
            self.assertEqual(f["mode"], "sim")
            self.assertFalse(f["demo"])
            self.assertIn(tel(f)["stateName"], ("CALIBRATING", "PAIRING"))
            self.assertEqual(f["relay"]["transport"]["kind"], "udp")
            f = events.wait(lambda f: tel(f) is not None and tel(f)["stateName"] == "PAIRING",
                            boot_s + S.WALL_SLACK_S, what="PAIRING")
            # The sim's stdout (the RA4M1's text) reaches the dashboard's text panel.
            state = relay.get("/state")
            texts = [e["text"] for e in state["text"]]
            self.assertTrue(any("state PAIRING" in t for t in texts), texts[-5:])
            self.assertTrue(any("simulator started" in e["text"] for e in state["log"]))

            # The page's keyboard path: token from /config, enable, then key state at KEYBOARD_POST_HZ.
            cfg_page = relay.get("/config")
            token = cfg_page["token"]
            self.assertEqual(relay.post("/keyboard/enable", {}, None)[0], 403)
            st, body = relay.post("/keyboard/enable", {}, token)
            self.assertEqual(st, 200)
            sid, seq = body["session"], 0
            posting = threading.Event()

            def page():
                nonlocal seq
                while not posting.is_set():
                    seq += 1
                    relay.post("/keyboard/state", {"session": sid, "seq": seq, "levels": {"right": True}}, token)
                    posting.wait(1.0 / S.SET.keyboard_post_hz)
            th = threading.Thread(target=page, daemon=True)
            th.start()
            try:
                # Keyboard input is a source change: connected=0 for RELAY_SOURCE_CHANGE_HOLD_MS, then
                # connected -> DISARMED, and D (right) pans the turret clockwise (negative).
                hold_s = cfg("RELAY_SOURCE_CHANGE_HOLD_MS") / 1000.0
                m = events.mark()
                f = events.wait(lambda f: tel(f) is not None and tel(f)["stateName"] == "DISARMED"
                                and tel(f)["panVelCdegS"] < 0, hold_s + S.WALL_SLACK_S, m, "the keyboard panning")
                self.assertEqual(f["input"]["source"], "keyboard")
                self.assertTrue(f["input"]["flagBits"]["keyboard"])
                self.assertEqual(tel(f)["linkSourceName"], "WIFI")
                pan0 = tel(f)["panCdeg"]
                m = events.mark()
                f = events.wait(lambda f: tel(f) is not None and tel(f)["panCdeg"] < pan0 - 5 * P.DS_ANGLE_SCALE,
                                S.WALL_SLACK_S + 5.0 / self.SPEED, m, "pan to move 5 deg")
                speed = abs(tel(f)["panVelCdegS"]) / P.DS_ANGLE_SCALE
                self.assertLessEqual(speed, cfg("PAN_MAX_DEG_PER_S"))
                self.assertGreater(speed, 0.0)
            finally:
                posting.set()
                th.join(2)
            # POSTs stopped: the relay's deadman goes neutral, the turret stops panning.
            m = events.mark()
            events.wait(lambda f: tel(f) is not None and tel(f)["panVelCdegS"] == 0, S.WALL_SLACK_S + 2.0, m,
                        "the deadman to stop the pan")
            for f in events.since(m):
                if tel(f):
                    self.assertNotEqual(tel(f)["stateName"], "ARMED")
            sim_pid = int(next(e["text"] for e in relay.get("/state")["log"]
                               if "simulator started" in e["text"]).rsplit("pid ", 1)[1].rstrip(")"))
        finally:
            if events is not None:
                events.close()
            rc, out, err = relay.stop()
        self.assertEqual(rc, 0, err)
        self.assertIn("relay: stopped", out)
        self.assertNotIn("Traceback", err)
        self.assertNotIn("simulator exited", err)
        # The relay stopped (and reaped) its simulator: no orphan left running.
        with self.assertRaises(ProcessLookupError):
            os.kill(sim_pid, 0)


class PtyTests(unittest.TestCase):
    SPEED = 5

    def test_bench_console_and_relay_serial_on_the_pty(self):
        sim = S.Sim("--pty", speed=self.SPEED)
        self.addCleanup(lambda: self.assertEqual(sim.stop(), 0))
        self.assertTrue(os.path.exists(sim.pty_path))
        console = subprocess.Popen([S.PYTHON, str(CONSOLE_PY), "--port", sim.pty_path], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=ENV)
        lines = S.Lines(console.stdout)

        def type_line(text):
            console.stdin.write((text + "\n").encode())
            console.stdin.flush()

        try:
            lines.wait(r"^# bench console", S.WALL_SLACK_S)
            lines.wait(r"^# serial .* open at %d baud" % cfg("DEBUG_SERIAL_BAUD"), S.WALL_SLACK_S)
            # The USB passthrough runs on the ESP32: until it has booted, PC bytes go nowhere (on
            # the board the port does not even exist yet).
            sim.wait_log(r"link: ESP32 answered", 2.0)
            # A shell line through the pty, the RA4M1's reply back through it.
            mark = lines.mark()
            type_line("!status")
            lines.wait(r"^# sent shell line: status", S.WALL_SLACK_S, mark)
            m, _ = lines.wait(r"^< \d+ status state=(\w+) fault=", S.WALL_SLACK_S, mark)
            self.assertIn(m.group(1), ("CALIBRATING", "PAIRING", "DISARMED"))
            lines.wait(r"^< \d+ status link=", S.WALL_SLACK_S, mark)
            # A refused action still answers (the shell obeys TurretState's permissions).
            mark = lines.mark()
            type_line("!pan 10")
            lines.wait(r"^< \d+ pan 10\.0: (refused: not in this state|ok)", S.WALL_SLACK_S, mark)
            # Its neutral connected ControlPackets over USB: after the boot the turret is DISARMED
            # with USB as the source (DEBUG_ACCEPT_USB_CONTROL), and the console prints it.
            self.assertTrue(cfg("DEBUG_ACCEPT_USB_CONTROL"))
            boot_s = cfg("TILT_CAL_MAX_TRAVEL_DEG") / cfg("TILT_CAL_SPEED_DEG_PER_S") + 3.0
            lines.wait(r"^telemetry: DISARMED .* link USB", sim.wall(boot_s))
            type_line("quit")
            console.wait(10)
        finally:
            if console.poll() is None:
                console.kill()
                console.wait()
            lines.join(2)
            console.stdout.close()
            console.stdin.close()
            err = console.stderr.read().decode(errors="replace")
            console.stderr.close()
        self.assertEqual(console.returncode, 0, err)
        self.assertNotIn("Traceback", err)
        # The console's last packet was connected=0: the turret saw the link go.
        sim.wait_log(r"link lost", 2.0)

        # The relay on the same pty (the console has let go of it).
        relay = Relay("--serial", sim.pty_path)
        try:
            relay.proc.stdout.readline()
            # Live USB telemetry, and the relay's own (connected=0) packets make USB the source.
            end = time.monotonic() + S.WALL_SLACK_S + 2.0
            state = None
            while time.monotonic() < end:
                state = relay.get("/state")
                if state["telemetryState"] == "live" and state["telemetry"]["linkSourceName"] == "USB":
                    break
                time.sleep(0.05)
            self.assertEqual(state["telemetryState"], "live", json.dumps(state)[:600])
            self.assertEqual(state["telemetry"]["linkSourceName"], "USB", json.dumps(state)[:600])
            self.assertEqual(state["mode"], "serial")
            self.assertEqual(state["relay"]["transport"]["kind"], "serial")
            self.assertEqual(state["telemetry"]["version"], P.DS_TELEMETRY_VERSION)
            self.assertFalse(state["telemetry"]["flagBits"]["linkConnected"])
        finally:
            rc, out, err = relay.stop()
        self.assertEqual(rc, 0, err)
        self.assertIn("relay: stopped", out)


if __name__ == "__main__":
    unittest.main()
