# SPDX-License-Identifier: MIT
"""dsentry.web and dsentry_relay.py: the dashboard server in-process (keyboard POST auth: token,
Host header, loopback client, JSON content type; CORS preflight never granted; SSE frames,
keepalive ping, coalescing) and the whole relay as a subprocess in --demo mode (keyboard drives
the stand-in turret; the deadman makes it neutral when POSTs stop; SIGTERM stops it cleanly).

Key safety properties: test_post_requires_the_token, test_post_requires_a_loopback_host_header,
test_post_from_a_non_loopback_client_refused, test_preflight_is_never_granted, and the
end-to-end test_demo_keyboard_drives_then_deadman_goes_neutral."""

import http.client
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import unittest
import urllib.request
from unittest import mock

import _support as S

from dsentry import demo as D
from dsentry import hub as H
from dsentry import keyboard as K
from dsentry import protocol as P
from dsentry import settings as ST
from dsentry import web as W

SET = ST.Settings.load(S.CONFIG_H)
DEFL = round(SET.keyboard_deflection * SET.joy_axis_range)
# what the CONTROLLER MAP panel's labels and refusals are built from (all from config.h via /config)
CMAP_CONFIG_NAMES = ("FLYWHEEL_HOLD_MODE", "FLYWHEEL_REQUIRES_ARMED", "FIRE_REQUIRES_FLYWHEEL",
                     "FIRE_LOCKOUT_WHEN_EMPTY", "ARM_REQUIRES_TRIGGER_RELEASED", "LCD_TXT_SERVO_ADJUST",
                     "SERVO_ADJUST_STEP_SMALL_DEG", "SERVO_ADJUST_STEP_LARGE_DEG", "TRIGGER_REST_ANGLE",
                     "TRIGGER_FIRE_ANGLE", "TRIGGER_PRESS_THRESHOLD", "TRIGGER_RELEASE_THRESHOLD", "JOY_DEADZONE",
                     "JOY_AXIS_RANGE", "TRIGGER_AXIS_RANGE", "MAG_CAPACITY")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.hub = H.Hub(SET, "test")
        self.kb = K.KeyboardSession(SET, on_event=self.hub.note)
        self.hub.attach(keyboard=self.kb)
        self.stop = threading.Event()
        self.server = W.serve("127.0.0.1", 0, W.make_handler(self.hub, self.kb, SET, self.stop))
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def tearDown(self):
        self.stop.set()
        self.server.shutdown()
        self.server.server_close()

    def request(self, method, path, body=None, headers=None, host=None, raw=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        hdrs = {"Host": host or "127.0.0.1:%d" % self.port}
        if body is not None:
            raw = json.dumps(body).encode()
            hdrs["Content-Type"] = "application/json"
        hdrs.update(headers or {})
        conn.request(method, path, body=raw, headers=hdrs)
        r = conn.getresponse()
        data = r.read()
        conn.close()
        try:
            parsed = json.loads(data) if data else None
        except ValueError:
            parsed = data
        return r.status, dict(r.getheaders()), parsed

    def auth(self):
        return {K.TOKEN_HEADER: self.kb.token}

    def test_page_and_state(self):
        st, hdrs, body = self.request("GET", "/")
        self.assertEqual(st, 200)
        self.assertTrue(hdrs["Content-Type"].startswith("text/html"))
        self.assertIn(b"EventSource", body)
        self.assertIn(b"/keyboard/state", body)
        st, _, state = self.request("GET", "/state")
        self.assertEqual((st, state["telemetryState"], state["boot"]), (200, "none", self.hub.boot))
        self.assertEqual(self.request("GET", "/nope")[0], 404)

    def test_state_carries_the_controller_block(self):
        _, _, state = self.request("GET", "/state")
        c = state["controller"]
        self.assertEqual((c["origin"], c["reported"], c["note"]), ("turret", False, "no telemetry yet"))
        self.hub.on_frame(P.Frame(P.PacketType.TELEMETRY, b"", P.TelemetryPacket(
            linkSource=P.DS_LINK_SRC_BLE, padState=P.DS_PAD_STATE_CONNECTED, padBattery=55, padLx=-7,
            padRt=1000, padButtons=P.DS_BTN_RS_CLICK)))
        _, _, state = self.request("GET", "/state")
        c = state["controller"]
        self.assertEqual((c["label"], c["lx"], c["rt"], c["batteryText"]), ("pad on the turret (BLE)", -7, 1000, "55 %"))
        self.assertTrue(c["buttonBits"]["rsClick"])
        self.assertEqual(state["telemetry"]["padStateName"], "CONNECTED")
        _, _, cfg = self.request("GET", "/config")
        self.assertEqual(cfg["enums"]["PadState"]["255"], "NONE")
        self.assertEqual([k for k, _m in cfg["bits"]["Buttons"]][12:], ["lsClick", "rsClick", "xbox", "share"])
        for name in ("JOY_DEADZONE", "TRIGGER_PRESS_THRESHOLD", "TRIGGER_RELEASE_THRESHOLD", "JOY_AXIS_RANGE",
                     "TRIGGER_AXIS_RANGE"):
            self.assertIsNotNone(cfg["config"][name], name)

    def test_page_has_the_controller_panel(self):
        page = W.PAGE_PATH.read_text()
        for needle in ('id="ctrlPanel"', 'id="ctrlStickL"', 'id="ctrlStickR"', 'id="ctrlLt"', 'id="ctrlRt"',
                       'id="ctrlButtons"', 'id="ctrlDpad"', 'id="ctrlBattery"', 'id="ctrlState"', 'id="ctrlSource"',
                       "TRIGGER_PRESS_THRESHOLD", "TRIGGER_RELEASE_THRESHOLD", "JOY_DEADZONE",
                       "not reported by this firmware"):
            self.assertIn(needle, page, needle)

    def test_page_has_the_controller_map_panel(self):
        """The CONTROLLER MAP sits in the turret's row, right after the 3D view, and builds its
        labels from /config (servo adjust text included) instead of copies in the page."""
        page = W.PAGE_PATH.read_text(encoding="utf-8")
        row = page.index('<div class="top-row">')
        end = page.index("</section>\n  </div>", row)                  # the row's last panel closes it
        turret, cmap = page.index('<canvas id="view"'), page.index('id="cmapPanel"')
        self.assertTrue(row < turret < cmap < end, (row, turret, cmap, end))
        for needle in ('id="cmapSvg"', 'id="cmapWrap"', 'id="cmapMode"', 'id="cmapLegend"', 'id="cmapSource"',
                       ">Controller map <", "function renderCmap", "renderCmap(f);", "buildCmap();",
                       "prefers-reduced-motion", ".panel.cmap-dim"):
            self.assertIn(needle, page, needle)
        for name in CMAP_CONFIG_NAMES:
            self.assertIn(name, page, name)
        # adjust mode is detected from config.h's text, never a copy of it
        self.assertNotIn(SET.cfg["LCD_TXT_SERVO_ADJUST"], page)
        self.assertTrue(page.isascii())

    def test_config_carries_the_controller_map_values(self):
        _, _, cfg = self.request("GET", "/config")
        for name in CMAP_CONFIG_NAMES:
            with self.subTest(name=name):
                self.assertIn(name, W.PAGE_CONFIG_NAMES)
                self.assertEqual(cfg["config"][name], SET.cfg[name])
        self.assertIsInstance(cfg["config"]["FLYWHEEL_HOLD_MODE"], bool)
        self.assertIsInstance(cfg["config"]["LCD_TXT_SERVO_ADJUST"], str)
        self.assertTrue(cfg["config"]["LCD_TXT_SERVO_ADJUST"])
        for name in ("SERVO_ADJUST_STEP_SMALL_DEG", "SERVO_ADJUST_STEP_LARGE_DEG", "TRIGGER_REST_ANGLE", "TRIGGER_FIRE_ANGLE"):
            self.assertIsInstance(cfg["config"][name], int, name)
        for name in ("TRIGGER_PRESS_THRESHOLD", "TRIGGER_RELEASE_THRESHOLD", "JOY_DEADZONE"):
            self.assertTrue(0.0 < cfg["config"][name] < 1.0, name)

    def cmap_js(self, script):
        """Runs the page's own CONTROLLER MAP logic (CMAP_LEGEND .. CMAP_NONE, cfg, cmapMode,
        cmapActions, cmapTip, cut out of index.html) in node against this server's /config; `script`
        prints one JSON value. Skipped where node is not installed."""
        node = shutil.which("node")
        if not node:
            self.skipTest("node not installed: the CONTROLLER MAP logic is not run")
        page = W.PAGE_PATH.read_text(encoding="utf-8")

        def function(name):
            start = page.index("function %s(" % name)
            depth, i = 0, page.index("{", start)
            while True:
                depth += {"{": 1, "}": -1}.get(page[i], 0)
                i += 1
                if depth == 0:
                    return page[start:i]

        consts = page[page.index("const CMAP_LEGEND = "):page.index("\n", page.index("const CMAP_NONE = "))]
        _, _, cfg = self.request("GET", "/config")
        js = "\n".join(["const S = {live: true, serverUp: true, frozen: false, conf: %s};" % json.dumps(cfg), consts] +
                       [function(n) for n in ("cfg", "cmapMode", "cmapActions", "cmapTip")] + [script])
        r = subprocess.run([node, "-e", js], capture_output=True, text=True, timeout=30)
        self.assertEqual(r.returncode, 0, r.stderr)
        return json.loads(r.stdout)

    def test_controller_map_servo_adjust_labels(self):
        """Owner, 2026-11-03: SERVO ADJUST on Menu. Normal mapping: Menu 'Servo adjust' (refused
        outside DISARMED), View unused. In adjust mode (LCD row 0 starts with LCD_TXT_SERVO_ADJUST
        while DISARMED): Menu 'Save & exit', View 'Reset to config.h', A / B record REST / FIRE,
        X test stroke, the d-pad and bumpers nudge by the SERVO_ADJUST_STEP_* steps, Y not used."""
        adjust = SET.cfg["LCD_TXT_SERVO_ADJUST"]
        out = self.cmap_js("""
          const t = (state, row0) => ({telemetry: {stateName: state, lcdRows: [row0, "R:1 F:2"], fireModeName: "SEMI",
                                      ammo: 3, faultName: "NONE"}, telemetryState: "live"});
          const adj = cmapMode(t("DISARMED", %s + " 23"));
          const res = {adj, adjActs: cmapActions(adj), shown: cmapMode(t("ARMED", %s + " 23")).kind};
          for (const st of ["DISARMED", "ARMED"]) res[st] = cmapActions(cmapMode(t(st, "DISARMED")));
          res.saveTip = cmapTip("Menu", res.adjActs.start);
          res.none = CMAP_NONE;
          console.log(JSON.stringify(res));""" % (json.dumps(adjust), json.dumps(adjust)))
        none, a = out["none"], out["adjActs"]
        self.assertEqual(out["adj"]["kind"], "adjust")
        self.assertTrue(out["adj"]["caption"].startswith(adjust))
        self.assertEqual(out["shown"], "normal")                     # the text alone is not enough: DISARMED too
        for bid, act in (("start", "Save & exit"), ("back", "Reset to config.h"), ("a", "Record REST"),
                         ("b", "Record FIRE"), ("x", "Test stroke"), ("rstick", "Aim (pan / tilt)")):
            with self.subTest(button=bid):
                self.assertEqual(a[bid]["act"], act)
                self.assertTrue(a[bid]["use"] and a[bid]["ok"])
        small, large = SET.cfg["SERVO_ADJUST_STEP_SMALL_DEG"], SET.cfg["SERVO_ADJUST_STEP_LARGE_DEG"]
        for bid, step in (("dpadUp", "+%d" % small), ("dpadDown", "−%d" % small), ("rb", "+%d" % large),
                          ("lb", "−%d" % large)):
            self.assertTrue(a[bid]["act"].startswith(step + "°"), (bid, a[bid]))
        for bid in ("y", "dpadLeft", "dpadRight", "rt"):
            self.assertFalse(a[bid]["use"], bid)                     # Y (and the rest) "not used"
        self.assertEqual(a["y"]["act"], none)
        self.assertIn("refused while the turret moves", out["saveTip"])
        for st in ("DISARMED", "ARMED"):
            with self.subTest(state=st):
                n = out[st]
                self.assertEqual(n["start"]["act"], "Servo adjust")
                self.assertEqual(n["start"]["ok"], st == "DISARMED")
                self.assertEqual(n["back"]["act"], none)
                self.assertEqual(n["y"]["act"], none)
                self.assertEqual(n["a"]["act"], "Arm / disarm")

    def test_page_has_the_servo_adjust_mapping_text(self):
        """Without node: the adjust mapping's labels are in the page and the old teach ones are gone."""
        page = W.PAGE_PATH.read_text(encoding="utf-8")
        for needle in ('put("start", "Save & exit"', 'put("back", "Reset to config.h"', 'put("start", "Servo adjust"',
                       'kind: "adjust"', 'cfg("LCD_TXT_SERVO_ADJUST")', 'put("a", "Record REST")',
                       'put("b", "Record FIRE")', 'put("x", "Test stroke")'):
            self.assertIn(needle, page, needle)
        for gone in ("Exit teach", "LCD_TXT_SERVO_TEACH", "SERVO_TEACH_", "m-teach"):
            self.assertNotIn(gone, page, gone)

    def test_state_carries_what_the_map_reads(self):
        """Adjust mode is read from telemetry LCD row 0 and the turret's state; the lights from the
        controller view."""
        adjust = SET.cfg["LCD_TXT_SERVO_ADJUST"]
        lcd = adjust.ljust(P.DS_TELEMETRY_LCD_COLS - 2) + "23"
        self.hub.on_frame(P.Frame(P.PacketType.TELEMETRY, b"", P.TelemetryPacket(
            state=P.DS_STATE_DISARMED, linkSource=P.DS_LINK_SRC_BLE, padState=P.DS_PAD_STATE_CONNECTED,
            padButtons=P.DS_BTN_DPAD_UP | P.DS_BTN_RB, padRt=700, lcd=lcd.ljust(P.DS_TELEMETRY_LCD_COLS * P.DS_TELEMETRY_LCD_ROWS))))
        _, _, state = self.request("GET", "/state")
        t, c = state["telemetry"], state["controller"]
        self.assertEqual(t["stateName"], "DISARMED")
        self.assertTrue(t["lcdRows"][0].startswith(adjust))
        self.assertIn(t["fireModeName"], ("SEMI", "FULL"))
        self.assertTrue(c["live"] and c["reported"])
        self.assertTrue(c["buttonBits"]["dpadUp"] and c["buttonBits"]["rb"])
        self.assertEqual(c["rt"], 700)

    def test_page_has_no_network_resources(self):
        page = W.PAGE_PATH.read_text()
        for needle in ("http://", "https://", "//cdn", "<link rel=\"stylesheet\"", "src=\""):
            self.assertNotIn(needle, page.replace("http://127.0.0.1:${location.port}/", ""), needle)

    def test_config_gives_the_token_only_to_a_local_host(self):
        _, _, cfg = self.request("GET", "/config")
        self.assertEqual(cfg["token"], self.kb.token)
        self.assertTrue(cfg["keyboardAllowed"])
        self.assertEqual(cfg["config"]["TILT_RANGE_DEG"], SET.cfg["TILT_RANGE_DEG"])
        self.assertEqual(cfg["config"]["MAG_CAPACITY"], SET.cfg["MAG_CAPACITY"])
        self.assertEqual(cfg["enums"]["TurretState"]["5"], "ARMED")
        for host in ("evil.example:%d" % self.port, "127.0.0.1:1", "dsentry.local"):
            with self.subTest(host=host):
                _, _, cfg = self.request("GET", "/config", host=host)
                self.assertIsNone(cfg["token"])
                self.assertFalse(cfg["keyboardAllowed"])

    def test_post_requires_the_token(self):
        for headers in ({}, {K.TOKEN_HEADER: "wrong"}, {K.TOKEN_HEADER: ""}):
            with self.subTest(headers=headers):
                st, _, body = self.request("POST", "/keyboard/enable", {}, headers)
                self.assertEqual(st, 403)
        self.assertFalse(self.kb.enabled)
        st, _, body = self.request("POST", "/keyboard/enable", {}, self.auth())
        self.assertEqual(st, 200)
        self.assertTrue(body["session"])
        self.assertTrue(self.kb.enabled)

    def test_post_requires_a_loopback_host_header(self):
        st, _, _ = self.request("POST", "/keyboard/enable", {}, self.auth(), host="attacker.example:%d" % self.port)
        self.assertEqual(st, 403)
        self.assertFalse(self.kb.enabled)

    def test_post_from_a_non_loopback_client_refused(self):
        with mock.patch.object(W.K, "client_is_loopback", lambda addr: False):
            st, _, body = self.request("POST", "/keyboard/enable", {}, self.auth())
            self.assertEqual(st, 403)
            self.assertIn("this computer", body["error"])
            _, _, cfg = self.request("GET", "/config")
            self.assertIsNone(cfg["token"])
        self.assertFalse(self.kb.enabled)

    def test_post_requires_json(self):
        st, _, _ = self.request("POST", "/keyboard/enable", None, dict(self.auth(), **{"Content-Type": "text/plain"}),
                                raw=b"{}")
        self.assertEqual(st, 415)
        st, _, _ = self.request("POST", "/keyboard/enable", None, dict(self.auth(), **{"Content-Type": "application/json"}),
                                raw=b"{not json")
        self.assertEqual(st, 400)
        st, _, _ = self.request("POST", "/keyboard/enable", None, dict(self.auth(), **{"Content-Type": "application/json"}),
                                raw=b"x" * (W.MAX_POST_BYTES + 1))
        self.assertEqual(st, 413)
        self.assertFalse(self.kb.enabled)

    def test_preflight_is_never_granted(self):
        st, hdrs, _ = self.request("OPTIONS", "/keyboard/enable", headers={
            "Origin": "https://evil.example", "Access-Control-Request-Method": "POST",
            "Access-Control-Request-Headers": K.TOKEN_HEADER})
        self.assertEqual(st, 405)
        self.assertFalse([h for h in hdrs if h.lower().startswith("access-control")], hdrs)
        for path in ("/", "/config", "/state"):
            _, hdrs, _ = self.request("GET", path, headers={"Origin": "https://evil.example"})
            self.assertFalse([h for h in hdrs if h.lower().startswith("access-control")], (path, hdrs))

    def test_state_post_drives_the_keyboard_session(self):
        _, _, body = self.request("POST", "/keyboard/enable", {}, self.auth())
        sid = body["session"]
        st, _, ack = self.request("POST", "/keyboard/state", {"session": sid, "seq": 1, "levels": {"right": True},
                                                              "presses": {"a": 1}}, self.auth())
        self.assertEqual((st, ack["ok"]), (200, True))
        out = self.kb.output()
        self.assertEqual(out.rx, DEFL)
        self.assertTrue(out.buttons & P.DS_BTN_A)
        st, _, _ = self.request("POST", "/keyboard/state", {"session": sid, "seq": 2, "levels": {"warp": True}}, self.auth())
        self.assertEqual(st, 400)
        st, _, _ = self.request("POST", "/keyboard/disable", {"session": sid}, self.auth())
        self.assertEqual(st, 200)
        st, _, _ = self.request("POST", "/keyboard/state", {"session": sid, "seq": 3}, self.auth())
        self.assertEqual(st, 409)

    def test_replaced_session_gets_409(self):
        _, _, a = self.request("POST", "/keyboard/enable", {}, self.auth())
        _, _, b = self.request("POST", "/keyboard/enable", {}, self.auth())
        self.assertEqual(self.request("POST", "/keyboard/state", {"session": a["session"], "seq": 1}, self.auth())[0], 409)
        self.assertEqual(self.request("POST", "/keyboard/state", {"session": b["session"], "seq": 1}, self.auth())[0], 200)

    def test_keep_alive_posts(self):
        """HTTP/1.1: many key-state POSTs over one connection (the page's 30 Hz)."""
        _, _, body = self.request("POST", "/keyboard/enable", {}, self.auth())
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=5)
        hdrs = dict(self.auth(), **{"Host": "127.0.0.1:%d" % self.port, "Content-Type": "application/json"})
        for seq in range(1, 31):
            conn.request("POST", "/keyboard/state", json.dumps({"session": body["session"], "seq": seq}), hdrs)
            r = conn.getresponse()
            r.read()
            self.assertEqual(r.status, 200)
        conn.close()
        self.assertEqual(self.kb.posts, 30)

    def read_events(self, until, timeout=5.0):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=timeout)
        conn.request("GET", "/events", headers={"Host": "127.0.0.1:%d" % self.port})
        r = conn.getresponse()
        self.assertEqual(r.getheader("Content-Type"), "text/event-stream; charset=utf-8")
        frames, pings, start = [], [], time.monotonic()
        try:
            while not until(frames, pings) and time.monotonic() - start < timeout:
                line = r.fp.readline().decode()
                if line.startswith("data: {\"version\""):
                    frames.append((time.monotonic() - start, json.loads(line[6:])))
                elif line == "event: ping\n":
                    pings.append(time.monotonic() - start)
        finally:
            conn.close()
        return frames, pings

    def test_events_first_frame_then_ping_when_idle(self):
        frames, pings = self.read_events(lambda f, p: p)
        self.assertEqual(len(frames), 1)
        self.assertTrue(pings)
        self.assertLess(pings[0], SET.keepalive_s * 1.5)

    def test_events_carry_changes_and_log_once(self):
        def feed():
            time.sleep(0.2)
            for i in range(3):
                self.hub.on_frame(P.Frame(P.PacketType.TELEMETRY, b"", P.TelemetryPacket(seq=i, state=P.DS_STATE_PAIRING)))
                time.sleep(0.1)
        threading.Thread(target=feed, daemon=True).start()
        frames, _ = self.read_events(lambda f, p: len(f) >= 4)
        self.assertEqual(frames[-1][1]["telemetryState"], "live")
        logged = [e["text"] for _, f in frames for e in f["log"]]
        self.assertEqual(logged.count("telemetry: first packet, state PAIRING, uptime 0.0 s"), 1)

    def test_events_are_coalesced_to_max_fps(self):
        stop = threading.Event()
        burst_s = 1.0

        def flood():
            end = time.monotonic() + burst_s
            while time.monotonic() < end and not stop.is_set():
                self.hub.note("relay", "x")
                time.sleep(0.001)
        threading.Thread(target=flood, daemon=True).start()
        frames, _ = self.read_events(lambda f, p: f and f[-1][0] > burst_s)
        stop.set()
        in_burst = [t for t, _ in frames if t <= burst_s]
        self.assertLessEqual(len(in_burst), SET.dashboard_max_fps * burst_s + 2)
        self.assertGreater(len(in_burst), SET.dashboard_max_fps * burst_s * 0.5)


class RelayProcess:
    def __init__(self, *args):
        self.port = free_port()
        self.proc = subprocess.Popen(
            [sys.executable, str(S.RELAY / "dsentry_relay.py"), *args, "--no-browser", "--quiet",
             "--http-port", str(self.port)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            env=dict(os.environ, PYTHONUNBUFFERED="1", PYTHONDONTWRITEBYTECODE="1"))
        self.base = "http://127.0.0.1:%d" % self.port

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as r:
            return json.loads(r.read())

    def post(self, path, body, token):
        req = urllib.request.Request(self.base + path, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json", K.TOKEN_HEADER: token or ""})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read() or b"{}")

    def wait(self, pred, timeout=8.0, poll=0.02):
        end = time.monotonic() + timeout
        last = None
        while time.monotonic() < end:
            last = self.get("/state")
            if pred(last):
                return last
            time.sleep(poll)
        raise AssertionError("condition not reached; last state: %s" % json.dumps(last)[:600])

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        try:
            out, err = self.proc.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            out, err = self.proc.communicate()
        return self.proc.returncode, out, err


class EndToEndTests(unittest.TestCase):
    def test_demo_keyboard_drives_then_deadman_goes_neutral(self):
        relay = RelayProcess("--demo", "--no-pad", "--no-demo-input")
        try:
            banner = relay.proc.stdout.readline()
            self.assertIn("http://127.0.0.1:%d/" % relay.port, banner)
            self.assertIn("mode: demo", banner)
            cfg = relay.get("/config")
            token = cfg["token"]
            self.assertTrue(cfg["demo"])
            relay.wait(lambda s: s["telemetryState"] == "live" and s["telemetry"]["stateName"] == "PAIRING")
            self.assertEqual(relay.post("/keyboard/enable", {}, None)[0], 403)          # no token
            st, body = relay.post("/keyboard/enable", {}, token)
            self.assertEqual(st, 200)
            sid, seq = body["session"], 0
            stop_posting = threading.Event()

            def page():                                   # what the page does at KEYBOARD_POST_HZ
                nonlocal seq
                while not stop_posting.is_set():
                    seq += 1
                    relay.post("/keyboard/state", {"session": sid, "seq": seq, "levels": {"right": True}}, token)
                    stop_posting.wait(1.0 / SET.keyboard_post_hz)
            th = threading.Thread(target=page, daemon=True)
            th.start()
            s = relay.wait(lambda s: s["input"]["source"] == "keyboard" and s["input"]["flagBits"]["connected"])
            self.assertEqual(s["input"]["rx"], DEFL)
            self.assertTrue(s["input"]["flagBits"]["keyboard"])
            # stick right turns the turret clockwise seen from above: pan velocity negative
            s = relay.wait(lambda s: s["telemetry"]["stateName"] == "DISARMED" and s["telemetry"]["panVelCdegS"] < 0)
            stop_posting.set()
            th.join(2)
            stopped_at = time.monotonic()
            s = relay.wait(lambda s: s["input"]["rx"] == 0, timeout=2.0, poll=0.01)
            took = time.monotonic() - stopped_at
            self.assertTrue(s["input"]["deadman"] or not s["keyboard"]["enabled"])
            # deadman LINK_TIMEOUT_MS, plus one POST period that may already have been in flight,
            # plus polling slack
            self.assertLess(took, SET.link_timeout_s + 1.0 / SET.keyboard_post_hz + 0.25)
            relay.wait(lambda s: not s["keyboard"]["enabled"], timeout=SET.keyboard_revoke_s + 2.0)
            self.assertEqual(relay.post("/keyboard/state", {"session": sid, "seq": seq + 1}, token)[0], 409)
            # SSE: frames with distinct versions
            conn = http.client.HTTPConnection("127.0.0.1", relay.port, timeout=5)
            conn.request("GET", "/events")
            r = conn.getresponse()
            frames, deadline = [], time.monotonic() + 5
            while len(frames) < 5 and time.monotonic() < deadline:
                line = r.fp.readline().decode()
                if line.startswith("data: "):
                    frames.append(json.loads(line[6:]))
            conn.close()
            self.assertGreaterEqual(len(frames), 5)
            self.assertEqual(len({f["version"] for f in frames}), len(frames))
        finally:
            rc, out, err = relay.stop()
        self.assertEqual(rc, 0, err)
        self.assertIn("relay: stopped", out)
        self.assertNotIn("Traceback", err)

    def test_demo_with_script_moves_by_itself(self):
        relay = RelayProcess("--demo", "--no-pad")
        try:
            relay.proc.stdout.readline()
            relay.wait(lambda s: s["input"]["source"] == "demo" and s["telemetry"] is not None
                       and s["telemetry"]["stateName"] == "ARMED", timeout=15)
            # the CONTROLLER panel: the scripted pad in full, and the stand-in's own report of it
            s = relay.wait(lambda s: s["controller"]["origin"] == "relay" and s["controller"]["lx"] != 0
                           and s["telemetry"]["padState"] == P.DS_PAD_STATE_CONNECTED)
            self.assertEqual((s["controller"]["label"], s["controller"]["batteryText"]),
                             ("demo script (scripted pad)", "%d %%" % D.DEMO_PAD_BATTERY_PERCENT))
            self.assertTrue(s["telemetry"]["controllerReported"])
        finally:
            rc, _, err = relay.stop()
        self.assertEqual(rc, 0, err)

    def test_bad_arguments_are_clean_errors(self):
        for args, needle in ((["--http-port", "99999"], "--http-port"),
                             (["--udp", "host:0"], "port 0 out of range"),
                             (["--sim", "--sim-bin", "/nonexistent/dsentry_sim"], "simulator not found")):
            with self.subTest(args=args):
                port = [] if "--http-port" in args else ["--http-port", "0"]
                r = subprocess.run([sys.executable, str(S.RELAY / "dsentry_relay.py"), "--no-browser", "--no-pad",
                                    "--quiet", *port, *args], capture_output=True, text=True, timeout=20)
                self.assertEqual(r.returncode, 2, r.stderr)
                self.assertIn(needle, r.stderr)
                self.assertNotIn("Traceback", r.stderr)

    def test_busy_http_port(self):
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            s.listen()
            port = s.getsockname()[1]
            r = subprocess.run([sys.executable, str(S.RELAY / "dsentry_relay.py"), "--demo", "--no-browser",
                                "--no-pad", "--http-port", str(port)], capture_output=True, text=True, timeout=20)
        self.assertEqual(r.returncode, 2)
        self.assertIn("cannot listen", r.stderr)


if __name__ == "__main__":
    unittest.main()
