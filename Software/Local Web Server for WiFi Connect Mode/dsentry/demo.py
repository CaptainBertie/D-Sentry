# SPDX-License-Identifier: MIT
"""--demo: a SMALL PYTHON STAND-IN for the turret, for previewing the dashboard without hardware
or the C++ simulator. It is NOT the RA4M1 firmware and proves nothing about it: the real logic
runs on the board, and on the Mac in dsentry_sim (--sim). Every frame it produces says so in the
dashboard (the page shows a DEMO badge) and its ESP32/stall/loop figures are made up.

It reads the same config.h values as the firmware, so what it shows has the right shape: tilt
calibration, PAIRING -> DISARMED on a connected packet, arm/fire-mode/magazine/home buttons,
velocity control with deadzone, expo, acceleration limits and soft limits, flywheel ramp and
spin-up, semi/full strokes with ammo, rumble per stroke (StatusPacket with a new rumbleSeq),
LED colour and LCD text, link loss after LINK_TIMEOUT_MS.

DemoScript is a scripted "pad" (lowest-priority input) so the page moves on its own; a real pad
or the dashboard keyboard takes over from it. It is a whole pad for the CONTROLLER panel: left
stick, left trigger, a battery level and every button, display-only ones (Y, RB, Menu, View,
d-pad up/right, stick clicks, Xbox, Share) in a "button tour" each period. Its TelemetryPacket
carries the controller block the way the RA4M1 fills it from a relay's ControlPackets: the
latest packet's right stick, right trigger and buttons while it says connected, neutral with
padState 3 while it does not, padState NONE once nothing arrived for LINK_TIMEOUT_MS; no left
stick, left trigger or battery (a ControlPacket has none).
"""

import collections
import math
import random
import threading
import time
from typing import List, Optional

from . import protocol as P
from .pad import PadState
from .transports import Transport

# Presentation-only values (not tunables; the demo is not the turret).
DEMO_START_TILT_DEG = 20.0      # where the barrel "is" at boot, before calibration drives it down
DEMO_LOOP_HZ = 9800             # made-up loop rate figure
DEMO_STALL_BASELINE = 512       # made-up A2 reading at rest (counts)
DEMO_STALL_NOISE = 3            # +- counts of fake noise
DEMO_STALL_PER_DEG_S = 0.25     # fake supply sag per deg/s of motion (counts)
DEMO_JITTER_MOVING_PERMILLE = 20
DEMO_SCRIPT_PERIOD_S = 32.0
DEMO_SCRIPT_LEAD_S = 5.0        # neutral (connected) input first, so calibration has finished
DEMO_PAD_BATTERY_PERCENT = 76   # made-up battery level of the scripted pad
DEMO_LEFT_STICK_PERIOD_S = 6.0  # the left stick circles (display only: never sent)
DEMO_LEFT_STICK_FRACTION = 0.6
DEMO_TAP_S = 0.15               # one scripted button tap
DEMO_TOUR_START_S = 23.0        # the display-only buttons, one after another
DEMO_TOUR_STEP_S = 0.3
DEMO_TOUR = (P.DS_BTN_Y, P.DS_BTN_RB, P.DS_BTN_DPAD_UP, P.DS_BTN_DPAD_RIGHT, P.DS_BTN_START,
             P.DS_BTN_BACK, P.DS_BTN_LS_CLICK, P.DS_BTN_RS_CLICK, P.DS_BTN_XBOX, P.DS_BTN_SHARE)
DEMO_LT_RAMP = (16.0, 17.5, 18.0)   # the left trigger: ramp from released, full at, let go at


def _shape(value: int, c) -> float:
    """Stick shaping exactly as architecture.md 5 (Inputs): normalise, clamp, deadzone rescale,
    expo, sign."""
    x = max(-1.0, min(1.0, value / c["JOY_AXIS_RANGE"]))
    dz = c["JOY_DEADZONE"]
    a = abs(x)
    if a <= dz:
        return 0.0
    s = (a - dz) / (1.0 - dz)
    e = c["JOY_EXPO"]
    s = (1.0 - e) * s + e * s * s * s
    return math.copysign(s, x)


class _Axis:
    def __init__(self, vmax, accel, lo, hi, slow):
        self.pos, self.vel = 0.0, 0.0
        self.vmax, self.accel, self.lo, self.hi, self.slow = vmax, accel, lo, hi, slow

    def brake_cap(self, direction: float) -> float:
        bound = self.hi if direction > 0 else self.lo
        if bound is None:
            return self.vmax
        d = max(0.0, (bound - self.pos) * direction)
        a_brake = min(self.accel, self.vmax * self.vmax / (2.0 * self.slow))
        return math.sqrt(2.0 * a_brake * d)

    def step(self, target: float, dt: float) -> None:
        if target:
            target = math.copysign(min(abs(target), self.brake_cap(math.copysign(1.0, target))), target)
        dv = max(-self.accel * dt, min(self.accel * dt, target - self.vel))
        self.vel += dv
        self.pos += self.vel * dt
        if self.hi is not None and self.pos > self.hi:
            self.pos, self.vel = self.hi, min(0.0, self.vel)
        if self.lo is not None and self.pos < self.lo:
            self.pos, self.vel = self.lo, max(0.0, self.vel)


class DemoTurret:
    """See the module docstring. step(now) advances the model and returns frames to emit."""

    def __init__(self, cfg, now: float, rng: Optional[random.Random] = None) -> None:
        c = self.c = cfg
        self.rng = rng or random.Random()
        self.t0 = now
        self.last = now
        self.state = P.DS_STATE_BOOT
        self.fault = P.DS_FAULT_NONE
        self.calibrated = False
        self.cal_phase = "down"
        self.cal_until = 0.0
        pan_lim = c["PAN_LIMITS_ENABLED"]
        self.pan = _Axis(c["PAN_MAX_DEG_PER_S"], c["PAN_ACCEL_DEG_PER_S2"],
                         c["PAN_MIN_DEG"] if pan_lim else None, c["PAN_MAX_DEG"] if pan_lim else None,
                         c["SOFT_LIMIT_SLOWDOWN_DEG"])
        self.tilt = _Axis(c["TILT_MAX_DEG_PER_S"], c["TILT_ACCEL_DEG_PER_S2"],
                          -c["TILT_CAL_BACKOFF_DEG"], None, c["SOFT_LIMIT_SLOWDOWN_DEG"])
        self.tilt.pos = DEMO_START_TILT_DEG
        self.home = (0.0, 0.0)
        self.homing = False
        self.armed = False
        self.fire_mode = P.DS_FIRE_MODE_SEMI
        self.ammo = c["MAG_CAPACITY"]
        self.shots = 0
        self.fly_since: Optional[float] = None
        self.duty = 0
        self.stroke: Optional[str] = None
        self.stroke_until = 0.0
        self.stroke_started = -1e9
        self.pulled = False
        self.pull_armed = False      # the current pull started while ARMED
        self.semi_spent = False
        self.ctrl: Optional[P.ControlPacket] = None
        self.ctrl_at = -1e9               # last packet that said connected
        self.ctrl_rx_at = -1e9            # last packet of any kind
        self.ctrl_times = collections.deque()
        self.seq_gaps = 0
        self.prev_buttons = 0
        self.baseline_pending = True
        self.lockout = {}
        self.lost_at: Optional[float] = None
        self.rumble_seq = 0
        self.rumble = (0, 0, 0)
        self.status_key = None
        self.status_at = -1e9
        self.telem_seq = 0
        self.telem_at = -1e9
        self.stall_peak = 0
        self.stall_sample = DEMO_STALL_BASELINE
        self.deframer = P.Deframer()
        self.text: List[bytes] = [b"D-Sentry DEMO boot (Python stand-in, not the RA4M1 firmware)\r\n"]
        self._logged_state = self.state

    # -- input ------------------------------------------------------------------------------
    def receive(self, data: bytes, now: float) -> None:
        for frame in self.deframer.feed(data):
            if frame.type != P.PacketType.CONTROL:
                continue
            pkt = frame.packet
            if self.ctrl is not None and (pkt.seq - self.ctrl.seq) % 256 != 1:
                self.seq_gaps += 1
            self.ctrl_times.append(now)
            if pkt.flags & P.DS_CTRL_FLAG_CONNECTED:
                self.ctrl_at = now
            self.ctrl_rx_at = now
            self.ctrl = pkt

    def connected(self, now: float) -> bool:
        return now - self.ctrl_at <= self.c["LINK_TIMEOUT_MS"] / 1000.0

    def pad_block(self, now: float) -> dict:
        """TelemetryPacket controller fields, as Telemetry::padBlock() fills them from the
        active source's latest frame (a relay's ControlPackets: no left stick, no battery)."""
        heard = self.ctrl is not None and now - self.ctrl_rx_at <= self.c["LINK_TIMEOUT_MS"] / 1000.0
        if not heard:
            return {"padState": P.DS_PAD_STATE_NONE}
        p = self.ctrl
        if not p.flags & P.DS_CTRL_FLAG_CONNECTED:
            return {"padState": P.DS_PAD_STATE_SCANNING}
        return {"padState": P.DS_PAD_STATE_CONNECTED, "padRx": p.rx, "padRy": p.ry, "padRt": p.rt,
                "padButtons": p.buttons}

    # -- model ------------------------------------------------------------------------------
    def _edge(self, buttons: int, mask: int, now: float) -> bool:
        if not buttons & mask or self.prev_buttons & mask:
            return False
        if now < self.lockout.get(mask, 0.0):
            return False
        self.lockout[mask] = now + self.c["BUTTON_DEBOUNCE_MS"] / 1000.0
        return True

    def _abort_stroke(self) -> None:
        self.stroke = None

    def _lose_link(self, now: float) -> None:
        self.state = P.DS_STATE_PAIRING
        self.armed = False
        self.homing = False
        self._abort_stroke()
        self.fly_since = None
        self.lost_at = now
        self.baseline_pending = True

    def step(self, now: float) -> List[bytes]:
        c = self.c
        dt = max(0.0, min(0.1, now - self.last))
        self.last = now
        pan_cmd = tilt_cmd = 0.0
        if self.state == P.DS_STATE_BOOT:
            self.state = P.DS_STATE_CALIBRATING
        elif self.state == P.DS_STATE_CALIBRATING:
            tilt_cmd = self._calibrate(now)
        else:
            live = self.connected(now) and self.ctrl is not None
            if self.state in (P.DS_STATE_DISARMED, P.DS_STATE_ARMED) and not live:
                self._lose_link(now)
            elif self.state == P.DS_STATE_PAIRING and live:
                self.state = P.DS_STATE_DISARMED
                self.lost_at = None
            if self.state in (P.DS_STATE_DISARMED, P.DS_STATE_ARMED):
                pan_cmd, tilt_cmd = self._controls(now)
        if self.homing:
            pan_cmd = self._home_speed(self.pan, self.home[0])
            tilt_cmd = self._home_speed(self.tilt, self.home[1])
            if abs(self.pan.pos - self.home[0]) < 0.05 and abs(self.tilt.pos - self.home[1]) < 0.05:
                self.homing = False
        self.pan.step(pan_cmd, dt)
        self.tilt.step(tilt_cmd, dt)
        self._fire(now)
        self._stall(now)
        return self._outputs(now)

    def _calibrate(self, now: float) -> float:
        c = self.c
        speed = c["TILT_CAL_SPEED_DEG_PER_S"]
        if self.cal_phase == "down":
            if self.tilt.pos <= self.tilt.lo + 1e-6:
                self.cal_phase, self.cal_until = "settle", now + c["TILT_CAL_SETTLE_MS"] / 1000.0
                return 0.0
            return -speed
        if self.cal_phase == "settle":
            if now >= self.cal_until:
                self.cal_phase = "up"
            return 0.0
        if self.tilt.pos >= 0.0:
            self.tilt.pos, self.tilt.vel = 0.0, 0.0
            self.tilt.lo, self.tilt.hi = 0.0, c["TILT_RANGE_DEG"]
            self.calibrated = True
            self.state = P.DS_STATE_PAIRING
            return 0.0
        return speed

    def _home_speed(self, axis: _Axis, target: float) -> float:
        d = target - axis.pos
        v = math.sqrt(2.0 * axis.accel * abs(d))
        return math.copysign(min(self.c["HOME_SPEED_DEG_PER_S"], v), d) if abs(d) > 1e-3 else 0.0

    def _controls(self, now: float):
        c = self.c
        pkt = self.ctrl
        b = pkt.buttons
        if self.baseline_pending:            # first connected frame: levels, no edges
            self.prev_buttons = b
            self.baseline_pending = False
        rt_frac = pkt.rt / c["TRIGGER_AXIS_RANGE"]
        was = self.pulled
        if rt_frac > c["TRIGGER_PRESS_THRESHOLD"]:
            self.pulled = True
        elif rt_frac < c["TRIGGER_RELEASE_THRESHOLD"]:
            self.pulled = False
        if self.pulled and not was:
            self.pull_armed = self.armed
            self.semi_spent = False
        if self._edge(b, P.DS_BTN_A, now):
            if self.armed:
                self.armed = False
                self._abort_stroke()
            elif not (self.pulled and c["ARM_REQUIRES_TRIGGER_RELEASED"]):
                self.armed = True
                self.pull_armed = False
        if self._edge(b, P.DS_BTN_B, now):
            self.fire_mode ^= 1
        if self._edge(b, P.DS_BTN_X, now):
            self.ammo = c["MAG_CAPACITY"]
        if self._edge(b, P.DS_BTN_DPAD_LEFT, now):
            self.home = (self.pan.pos, self.tilt.pos)
        if self._edge(b, P.DS_BTN_DPAD_DOWN, now):
            self.homing = True
        self.prev_buttons = b
        self.state = P.DS_STATE_ARMED if self.armed else P.DS_STATE_DISARMED
        sx, sy = _shape(pkt.rx, c), _shape(pkt.ry, c)
        if sx or sy:
            self.homing = False
        lb = bool(b & P.DS_BTN_LB) and (self.armed or not c["FLYWHEEL_REQUIRES_ARMED"])
        if lb and self.fly_since is None:
            self.fly_since = now
        elif not lb:
            self.fly_since = None
        tilt_sign = -1.0 if c["JOY_INVERT_Y"] else 1.0
        # stick right turns the turret clockwise seen from above = negative pan (contract 4.1)
        return -sx * self.pan.vmax, tilt_sign * sy * self.tilt.vmax

    def _fire(self, now: float) -> None:
        c = self.c
        on = self.fly_since is not None
        ramp = c["FLYWHEEL_RAMP_MS"] / 1000.0
        self.duty = 0 if not on else round(c["FLYWHEEL_DUTY_255"] * min(1.0, (now - self.fly_since) / ramp))
        if self.stroke == "forward" and now >= self.stroke_until:
            self.stroke, self.stroke_until = "return", now + c["TRIGGER_RETURN_MS"] / 1000.0
        elif self.stroke == "return" and now >= self.stroke_until:
            self.stroke = None
            self.ammo = max(0, self.ammo - 1)
            self.shots = (self.shots + 1) & 0xFFFF
        if self.stroke is not None or self.state != P.DS_STATE_ARMED:
            return
        ready = on and now - self.fly_since >= c["FLYWHEEL_SPINUP_MS"] / 1000.0
        allowed = (self.ammo > 0 or not c["FIRE_LOCKOUT_WHEN_EMPTY"]) and (ready or not c["FIRE_REQUIRES_FLYWHEEL"])
        want = self.pulled and self.pull_armed and (self.fire_mode == P.DS_FIRE_MODE_FULL or not self.semi_spent)
        if want and allowed:
            self.semi_spent = True
            self.stroke, self.stroke_until = "forward", now + c["TRIGGER_FORWARD_MS"] / 1000.0
            self.stroke_started = now
            self.rumble_seq = (self.rumble_seq + 1) & 0xFF
            self.rumble = (c["RUMBLE_STRONG_255"], c["RUMBLE_WEAK_255"], c["RUMBLE_DURATION_MS"])

    def _stall(self, now: float) -> None:
        speed = abs(self.pan.vel) + abs(self.tilt.vel)
        noise = self.rng.randint(-DEMO_STALL_NOISE, DEMO_STALL_NOISE)
        self.stall_sample = DEMO_STALL_BASELINE + round(speed * DEMO_STALL_PER_DEG_S) + noise
        if speed > 0.0:
            self.stall_peak = max(self.stall_peak, abs(self.stall_sample - DEMO_STALL_BASELINE))

    # -- outputs ----------------------------------------------------------------------------
    def _led(self, now: float):
        c = self.c
        t = (now - self.t0) * 1000.0
        blink_on = int(t // c["LED_FAULT_BLINK_MS"]) % 2 == 0
        pulse = 0.5 - 0.5 * math.cos(2.0 * math.pi * t / c["LED_PULSE_PERIOD_MS"])
        floor = c["LED_PULSE_MIN_FRACTION"]
        k = floor + (1.0 - floor) * pulse
        firing = self.stroke is not None or now - self.stroke_started < c["LED_FIRE_FLASH_MIN_MS"] / 1000.0
        if firing:
            rgb, k = c.rgb("LED_COLOR_FIRING"), 1.0
        elif self._empty_blink():
            rgb, k = c.rgb("LED_COLOR_EMPTY_MAG"), 1.0 if blink_on else 0.0
        elif self.state in (P.DS_STATE_BOOT, P.DS_STATE_CALIBRATING):
            rgb = c.rgb("LED_COLOR_STARTUP")
        elif self.state == P.DS_STATE_PAIRING:
            a, b = c.rgb("LED_COLOR_PAIRING_A"), c.rgb("LED_COLOR_PAIRING_B")
            rgb, k = tuple(round(x + (y - x) * pulse) for x, y in zip(a, b)), 1.0
        elif self.state == P.DS_STATE_ARMED:
            rgb, k = c.rgb("LED_COLOR_ARMED_FULL" if self.fire_mode else "LED_COLOR_ARMED_SEMI"), 1.0
        else:
            rgb, k = c.rgb("LED_COLOR_DISARMED"), 1.0
        scale = c["LED_MAX_BRIGHTNESS"] / c["LED_CHANNEL_MAX"]
        return tuple(round(v * k * scale) for v in rgb)

    def _empty_blink(self) -> bool:
        return (self.state == P.DS_STATE_ARMED and self.ammo == 0 and self.c["FIRE_LOCKOUT_WHEN_EMPTY"]
                and self.pulled)

    def _row(self, left: str, right: str) -> str:
        cols = self.c["LCD_COLS"]
        room = cols - len(left) - 1
        if right and room > 0:
            right = right[:room]
            return (left + " " * (cols - len(left) - len(right)) + right)[:cols]
        return left[:cols].ljust(cols)

    def _lcd(self, now: float) -> str:
        c = self.c
        if self.state in (P.DS_STATE_BOOT, P.DS_STATE_CALIBRATING):
            return self._row(c["LCD_TXT_CALIBRATING"], "") + self._row("", "")
        arm = c["LCD_TXT_ARMED"] if self.armed else c["LCD_TXT_DISARMED"]
        if self.armed and c["LCD_TXT_SHOW_FIRE_MODE"]:
            arm += " F" if self.fire_mode else " S"
        row0 = self._row(arm, "%s%d" % (c["LCD_TXT_AMMO_PREFIX"], self.ammo))
        if self.state == P.DS_STATE_PAIRING:
            grace = c["LINK_RECONNECT_GRACE_MS"] / 1000.0
            if self.lost_at is not None and now - self.lost_at > grace:
                status = c["LCD_TXT_NO_CONNECTION"]
            else:
                dots = ("", ".", ". .", ". . .")[int((now - self.t0) * 1000.0 // c["LCD_PAIRING_DOT_MS"]) % 4]
                status = c["LCD_TXT_PAIRING"] + (" " + dots if dots else "")
        else:
            status = c["LCD_TXT_CONNECTED"]
        return row0 + self._row(c["LCD_TXT_MODE_WIFI"], status)

    def telemetry(self, now: float) -> P.TelemetryPacket:
        c = self.c
        while self.ctrl_times and self.ctrl_times[0] <= now - 1.0:
            self.ctrl_times.popleft()
        live = self.connected(now)
        ready = self.fly_since is not None and now - self.fly_since >= c["FLYWHEEL_SPINUP_MS"] / 1000.0
        moving = abs(self.pan.vel) + abs(self.tilt.vel) > 0.0
        flags = (P.DS_TELEM_FLAG_POWER_TOGGLE_ON | P.DS_TELEM_FLAG_MODE_TOGGLE_WIFI | P.DS_TELEM_FLAG_DRIVERS_ENABLED
                 | P.DS_TELEM_FLAG_LCD_PRESENT | P.DS_TELEM_FLAG_ESP_ALIVE)
        for cond, bit in ((live and self.state not in (P.DS_STATE_BOOT, P.DS_STATE_CALIBRATING), P.DS_TELEM_FLAG_LINK_CONNECTED),
                          (self.fly_since is not None, P.DS_TELEM_FLAG_FLYWHEEL_ON),
                          (ready, P.DS_TELEM_FLAG_FLYWHEEL_READY),
                          (self.stroke is not None, P.DS_TELEM_FLAG_STROKE_ACTIVE),
                          (self.pulled, P.DS_TELEM_FLAG_TRIGGER_PULLED),
                          (self.homing, P.DS_TELEM_FLAG_HOMING),
                          (self.calibrated, P.DS_TELEM_FLAG_CALIBRATED),
                          (bool(c["STALL_DETECT_ENABLED"]), P.DS_TELEM_FLAG_STALL_ENABLED),
                          (self._empty_blink(), P.DS_TELEM_FLAG_EMPTY_MAG_BLINK)):
            if cond:
                flags |= bit
        r, g, b = self._led(now)
        scale = P.DS_ANGLE_SCALE
        self.telem_seq = (self.telem_seq + 1) & 0xFF
        return P.TelemetryPacket(
            seq=self.telem_seq, uptimeMs=round((now - self.t0) * 1000.0) & 0xFFFFFFFF,
            state=self.state, fault=self.fault, fireMode=self.fire_mode,
            linkSource=P.DS_LINK_SRC_WIFI if live else P.DS_LINK_SRC_NONE, flags=flags,
            panCdeg=round(self.pan.pos * scale), tiltCdeg=round(self.tilt.pos * scale),
            panVelCdegS=round(self.pan.vel * scale), tiltVelCdegS=round(self.tilt.vel * scale),
            homePanCdeg=round(self.home[0] * scale), homeTiltCdeg=round(self.home[1] * scale),
            ammo=self.ammo, magCapacity=c["MAG_CAPACITY"], shotsTotal=self.shots,
            flywheelDuty=self.duty,
            servoDeg=c["TRIGGER_FIRE_ANGLE"] if self.stroke == "forward" else c["TRIGGER_REST_ANGLE"],
            stallSample=self.stall_sample, stallBaseline=DEMO_STALL_BASELINE, stallPeak=self.stall_peak,
            loopHz=DEMO_LOOP_HZ + self.rng.randint(-DEMO_STALL_NOISE, DEMO_STALL_NOISE),
            ctrlRxHz=len(self.ctrl_times), crcErrors=0, seqGaps=self.seq_gaps & 0xFFFF,
            stepJitterPermille=DEMO_JITTER_MOVING_PERMILLE if moving else 0,
            lastCtrlSeq=self.ctrl.seq if self.ctrl is not None else 0,
            espMode=P.DS_ESP_MODE_WIFI, ledR=r, ledG=g, ledB=b, lcd=self._lcd(now),
            **self.pad_block(now))

    def _outputs(self, now: float) -> List[bytes]:
        c = self.c
        if self.state != self._logged_state:
            self.text.append(("[2] STATE %s -> %s\r\n" % (P.TurretState(self._logged_state).name,
                                                        P.TurretState(self.state).name)).encode())
            self._logged_state = self.state
        out, self.text = self.text, []
        status = P.StatusPacket(wantMode=P.DS_RADIO_WIFI, rumbleStrong=self.rumble[0], rumbleWeak=self.rumble[1],
                                rumbleMs=self.rumble[2], state=self.state, rumbleSeq=self.rumble_seq)
        key = status.payload()
        if key != self.status_key or now - self.status_at >= c["STATUS_HEARTBEAT_MS"] / 1000.0:
            self.status_key, self.status_at = key, now
            out.append(status.encode())
        period = 1.0 / c["TELEMETRY_USB_HZ"]
        if now - self.telem_at >= period:
            # deadline schedule: the rate holds even though step() runs on a coarser clock
            self.telem_at = max(self.telem_at + period, now - period)
            out.append(self.telemetry(now).encode())
        return out


class DemoScript:
    """A scripted pad that loops every DEMO_SCRIPT_PERIOD_S: arm, flywheel, semi shots, full-auto
    burst, set home, wander, go home, a left-trigger ramp, magazine reset, disarm, a tour of the
    display-only buttons, then a link drop. The left stick circles throughout."""

    def __init__(self, cfg, start: float) -> None:
        self.c = cfg
        self.start = start

    def __call__(self, now: float) -> PadState:
        c = self.c
        r = c["JOY_AXIS_RANGE"]
        clamp = lambda v: max(-r, min(r - 1, v))   # noqa: E731
        # the left stick is display only (a ControlPacket has no left stick): a slow circle
        a = 2.0 * math.pi * (now - self.start) / DEMO_LEFT_STICK_PERIOD_S
        lx = clamp(round(DEMO_LEFT_STICK_FRACTION * r * math.cos(a)))
        ly = clamp(round(DEMO_LEFT_STICK_FRACTION * r * math.sin(a)))
        idle = PadState(True, name="demo script", lx=lx, ly=ly, battery=DEMO_PAD_BATTERY_PERCENT)
        if now - self.start < DEMO_SCRIPT_LEAD_S:
            return idle
        t = (now - self.start - DEMO_SCRIPT_LEAD_S) % DEMO_SCRIPT_PERIOD_S
        if 26.0 <= t < 29.0:
            return idle._replace(connected=False, lx=0, ly=0)   # link-loss demonstration
        buttons = 0
        taps = {2.0: P.DS_BTN_A, 4.0: P.DS_BTN_DPAD_LEFT, 9.0: P.DS_BTN_B, 12.5: P.DS_BTN_B,
                15.0: P.DS_BTN_DPAD_DOWN, 19.0: P.DS_BTN_X, 22.0: P.DS_BTN_A}
        for i, mask in enumerate(DEMO_TOUR):        # buttons the turret does nothing with
            taps[DEMO_TOUR_START_S + i * DEMO_TOUR_STEP_S] = mask
        for at, mask in taps.items():
            if at <= t < at + DEMO_TAP_S:
                buttons |= mask
        if 3.0 <= t < 13.0:
            buttons |= P.DS_BTN_LB
        rt = 0
        for a0, b0 in ((6.0, 6.3), (7.5, 7.8), (10.0, 11.2)):
            if a0 <= t < b0:
                rt = c["TRIGGER_AXIS_RANGE"]
        lt = 0
        start, full, release = DEMO_LT_RAMP
        if start <= t < release:
            lt = round(c["TRIGGER_AXIS_RANGE"] * min(1.0, (t - start) / (full - start)))
        rx = ry = 0
        if 4.5 <= t < 15.0 or 18.0 <= t < 22.0:
            phase = 2.0 * math.pi * t / 8.0
            rx = round(0.7 * r * math.sin(phase))
            ry = round(0.5 * r * math.sin(2.0 * phase))
        return idle._replace(rx=clamp(rx), ry=clamp(ry), rt=rt, buttons=buttons, lt=lt)


class DemoTransport(Transport):
    """In-process 'link' to a DemoTurret: send() feeds it ControlPacket bytes; a thread steps it
    at CONTROL_LOOP_HZ and its frames go through the same LinkReader as a real port's bytes."""

    kind = "demo"
    expected_link_source = P.DS_LINK_SRC_WIFI

    def __init__(self, settings, sink, clock=time.monotonic) -> None:
        super().__init__(settings, sink)
        self._clock = clock
        self.turret = DemoTurret(settings.cfg, clock())
        self._lock = threading.Lock()

    def send(self, data: bytes) -> bool:
        with self._lock:
            self.turret.receive(data, self._clock())
        self.sent += 1
        return True

    def _run(self, stop: threading.Event) -> None:
        period = 1.0 / self.s.cfg["CONTROL_LOOP_HZ"]
        self.sink.note("relay", "DEMO: a Python stand-in turret (not the RA4M1 logic; use --sim for that)")
        due = self._clock()
        while not stop.is_set():
            with self._lock:
                frames = self.turret.step(self._clock())
            for raw in frames:
                self._deliver(self.reader.feed(raw))
            due += period
            now = self._clock()
            if now - due > period:
                due = now
            stop.wait(max(0.0, due - now))

    def up(self) -> bool:
        return True

    def describe(self) -> str:
        return "DEMO turret (Python stand-in)"

