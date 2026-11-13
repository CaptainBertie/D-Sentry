# SPDX-License-Identifier: MIT
"""The dashboard's HTTP server (architecture.md 7): stdlib ThreadingHTTPServer, HTTP/1.1.

Routes
  GET  /                 relay-pc/web/index.html (read per request, so edits show on reload)
  GET  /config           config.h values the page needs, enum names, the key map, and the per-boot
                         keyboard token -- the token only for a loopback client whose Host header
                         names this server (DNS-rebinding guard)
  GET  /state            one JSON frame (hub.frame)
  GET  /events           server-sent events: a frame per change, coalesced to DASHBOARD_MAX_FPS,
                         "event: ping" after DASHBOARD_KEEPALIVE_MS of quiet
  POST /keyboard/enable  -> {"session": id}
  POST /keyboard/state   {"session", "seq", "levels": {...}, "presses": {...}}
  POST /keyboard/disable {"session"}
Every POST needs: a loopback client, an allowed Host header, the X-DSentry-Token header equal to
the boot token, and Content-Type application/json. OPTIONS is answered 405 with no CORS headers,
so a cross-origin page can never pass the preflight that the custom header forces.
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from . import keyboard as K
from . import protocol as P

PAGE_PATH = Path(__file__).resolve().parents[1] / "web" / "index.html"
MAX_POST_BYTES = 4096            # a key-state body is ~300 bytes
HANDLER_SOCKET_TIMEOUT_S = 30    # an idle keep-alive or stalled client frees its thread after this
SSE_WAIT_SLICE_S = 0.5           # how often an idle SSE loop re-checks stop and the keepalive

# config.h names the page reads from /config (presentation and display ranges only).
PAGE_CONFIG_NAMES = (
    "CONTROL_PACKET_HZ", "LINK_TIMEOUT_MS", "TELEMETRY_USB_HZ", "TELEMETRY_WIFI_HZ",
    "KEYBOARD_POST_HZ", "KEYBOARD_DEFLECTION", "KEYBOARD_PRESS_MS", "KEYBOARD_REVOKE_MS",
    "DASHBOARD_MAX_FPS", "DASHBOARD_KEEPALIVE_MS", "DASHBOARD_EVENT_LOG_KEEP", "DASHBOARD_STALE_MS",
    "TILT_RANGE_DEG", "PAN_LIMITS_ENABLED", "PAN_MIN_DEG", "PAN_MAX_DEG", "SOFT_LIMIT_SLOWDOWN_DEG",
    "PAN_MAX_DEG_PER_S", "TILT_MAX_DEG_PER_S", "MAG_CAPACITY", "LED_COUNT", "LED_MAX_BRIGHTNESS",
    "LED_CHANNEL_MAX", "LCD_COLS", "LCD_ROWS", "FLYWHEEL_DUTY_255", "FLYWHEEL_SPINUP_MS",
    "PWM_MAX", "STALL_DELTA_COUNTS", "ADC_MAX_COUNTS", "STEP_JITTER_LIMIT_PERMILLE", "PERMILLE",
    "JOY_AXIS_RANGE", "TRIGGER_AXIS_RANGE", "JOY_DEADZONE", "TRIGGER_PRESS_THRESHOLD",
    "TRIGGER_RELEASE_THRESHOLD", "TRIGGER_REST_ANGLE", "TRIGGER_FIRE_ANGLE", "TRIGGER_PREFIRE_ANGLE", "TRIGGER_CLEAR_ANGLE",
    "SERVO_SPAN_DEG",
    # the CONTROLLER MAP panel's labels and which actions the turret's state refuses
    "FLYWHEEL_HOLD_MODE", "FLYWHEEL_REQUIRES_ARMED", "FIRE_REQUIRES_FLYWHEEL", "FIRE_LOCKOUT_WHEN_EMPTY",
    "ARM_REQUIRES_TRIGGER_RELEASED", "LCD_TXT_SERVO_TEACH", "SERVO_TEACH_STEP_SMALL_DEG",
    "SERVO_TEACH_STEP_LARGE_DEG",
)


def page_config(settings, hub, kb, local: bool) -> dict:
    cfg = {}
    for name in PAGE_CONFIG_NAMES:
        value = settings.value(name)
        cfg[name] = value if isinstance(value, (bool, int, float)) or value is None else str(value)
    enums = {e.__name__: {m.value: m.name for m in e} for e in
             (P.TurretState, P.Fault, P.FireMode, P.LinkSource, P.EspMode, P.RadioMode, P.PadState)}
    return {
        "boot": hub.boot,
        "demo": hub.demo,
        "mode": hub.mode,
        "token": kb.token if (local and kb is not None) else None,
        "tokenHeader": K.TOKEN_HEADER,
        "keyboardAllowed": bool(local and kb is not None),
        "config": cfg,
        "enums": enums,
        "bits": {name: list(bits) for name, bits in P.BIT_KEYS.items()},
        "keys": [{"codes": list(codes), "action": action, "key": key} for codes, action, key in K.KEY_MAP],
        "angleScale": P.DS_ANGLE_SCALE,
    }


def make_handler(hub, kb, settings, stop: threading.Event, page_path: Path = PAGE_PATH):
    min_gap = 1.0 / settings.dashboard_max_fps
    keepalive = settings.keepalive_s

    class Handler(BaseHTTPRequestHandler):
        server_version = "DSentryRelay/1"
        protocol_version = "HTTP/1.1"
        timeout = HANDLER_SOCKET_TIMEOUT_S

        def log_message(self, fmt, *args):   # the terminal is for the relay's own lines
            pass

        # -- helpers --------------------------------------------------------------------------
        def _port(self) -> int:
            return self.server.server_address[1]

        def _local(self) -> bool:
            return (K.client_is_loopback(self.client_address[0])
                    and K.host_allowed(self.headers.get("Host"), self._port()))

        def _send(self, status: int, content_type: str, body: bytes, extra=()) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            for k, v in extra:
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status: int, obj) -> None:
            self._send(status, "application/json", json.dumps(obj, separators=(",", ":")).encode())

        def _error(self, status: int, message: str) -> None:
            self._json(status, {"ok": False, "error": message})

        # -- GET ------------------------------------------------------------------------------
        def do_GET(self):
            path = self.path.split("?", 1)[0]
            try:
                if path == "/":
                    try:
                        body = page_path.read_bytes()
                    except OSError as error:
                        self._send(500, "text/plain; charset=utf-8",
                                   ("cannot read %s: %s\n" % (page_path, error)).encode())
                        return
                    self._send(200, "text/html; charset=utf-8", body)
                elif path == "/config":
                    self._json(200, page_config(settings, hub, kb, self._local()))
                elif path == "/state":
                    self._json(200, hub.frame())
                elif path == "/events":
                    self._events()
                else:
                    self._send(404, "text/plain; charset=utf-8", b"not found\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError):
                self.close_connection = True

        def _events(self) -> None:
            self.close_connection = True          # the stream ends when the connection does
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(b"retry: 1000\n\n")
            self.wfile.flush()
            seen, log_after, text_after = -1, 0, 0
            last_frame = last_write = time.monotonic()
            while not stop.is_set():
                idle_left = keepalive - (time.monotonic() - last_write)
                version = hub.wait_for_change(seen, min(SSE_WAIT_SLICE_S, max(0.0, idle_left)))
                now = time.monotonic()
                if version != seen:
                    stop.wait(max(0.0, min_gap - (now - last_frame)))   # coalesce to MAX_FPS
                    frame = hub.frame(log_after=log_after, text_after=text_after)
                    seen = frame["version"]
                    if frame["log"]:
                        log_after = frame["log"][-1]["id"]
                    if frame["text"]:
                        text_after = frame["text"][-1]["id"]
                    self.wfile.write(("data: %s\n\n" % json.dumps(frame, separators=(",", ":"))).encode())
                    self.wfile.flush()
                    last_frame = last_write = time.monotonic()
                elif now - last_write >= keepalive:
                    # A named event, not a comment: comments never reach page script, and the page
                    # times out on silence to tell a frozen relay from a quiet turret.
                    self.wfile.write(b"event: ping\ndata: {}\n\n")
                    self.wfile.flush()
                    last_write = now

        # -- POST -----------------------------------------------------------------------------
        def do_POST(self):
            path = self.path.split("?", 1)[0]
            try:
                length = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                length = -1
            if length < 0 or length > MAX_POST_BYTES:
                self.close_connection = True
                self._error(413, "body too large or bad Content-Length")
                return
            raw = self.rfile.read(length) if length else b""
            try:
                if path not in ("/keyboard/enable", "/keyboard/state", "/keyboard/disable"):
                    self._error(404, "not found")
                    return
                if kb is None:
                    self._error(403, "keyboard control is not available in this mode")
                    return
                if not K.client_is_loopback(self.client_address[0]):
                    self._error(403, "keyboard control is only accepted from this computer")
                    return
                if not K.host_allowed(self.headers.get("Host"), self._port()):
                    self._error(403, "Host header must be 127.0.0.1:%d or localhost:%d" % (self._port(), self._port()))
                    return
                if not K.token_matches(self.headers.get(K.TOKEN_HEADER), kb.token):
                    self._error(403, "missing or wrong %s" % K.TOKEN_HEADER)
                    return
                ctype = (self.headers.get("Content-Type") or "").split(";", 1)[0].strip().lower()
                if ctype != "application/json":
                    self._error(415, "Content-Type must be application/json")
                    return
                try:
                    body = json.loads(raw.decode("utf-8")) if raw else {}
                except (UnicodeDecodeError, ValueError):
                    self._error(400, "body is not JSON")
                    return
                try:
                    if path == "/keyboard/enable":
                        self._json(200, {"ok": True, "session": kb.enable()})
                    elif path == "/keyboard/state":
                        self._json(200, kb.apply(body.get("session") if isinstance(body, dict) else None, body))
                    else:
                        kb.disable(body.get("session") if isinstance(body, dict) else None,
                                   str(body.get("why", "disabled in the page"))[:80] if isinstance(body, dict) else "")
                        self._json(200, {"ok": True})
                except K.KeyboardError as error:
                    self._error(error.status, str(error))
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError):
                self.close_connection = True

        def do_OPTIONS(self):
            # Never grant a CORS preflight: no Access-Control-* headers, ever.
            self._send(405, "text/plain; charset=utf-8", b"method not allowed\n", (("Allow", "GET, POST"),))

        def do_PUT(self):
            self._send(405, "text/plain; charset=utf-8", b"method not allowed\n", (("Allow", "GET, POST"),))

        do_DELETE = do_PUT
        do_PATCH = do_PUT

    return Handler


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def serve(host: str, port: int, handler) -> Server:
    """Binds (port 0 = any free port); the caller runs serve_forever on a thread."""
    return Server((host, port), handler)
