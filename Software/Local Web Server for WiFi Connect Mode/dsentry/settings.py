# SPDX-License-Identifier: MIT
"""Every config.h value the PC side uses, read once into one typed object (architecture.md rule 1:
Python reads tunables from config.h at run time; nothing here is a copy of a tunable).

    from dsentry import config, settings
    s = settings.Settings.load()             # D-SentryCode/config.h, or $DSENTRY_CONFIG_H
    s.control_period_s                       # 1 / CONTROL_PACKET_HZ
    s.cfg                                    # the full config.Config for anything else

Every name is required: a config.h without one is refused (SettingsError names it), so the relay
never runs on a value config.h does not hold.
"""

import dataclasses
from typing import Optional

from . import config as _config

# Opening the UNO R4 WiFi's USB port at these bauds resets the RA4M1 (the ESP32 bridge firmware,
# UNOR4USBBridge.ino:224-254); every PC tool must open it at exactly DEBUG_SERIAL_BAUD instead.
BRIDGE_RESET_BAUDS = (1200, 2400)
# USB ID of the UNO R4 WiFi's ESP32-S3 bridge (detect_port).
UNO_R4_WIFI_VID = 0x2341
UNO_R4_WIFI_PID = 0x1002


class SettingsError(ValueError):
    """config.h holds a value the PC side cannot work with."""


def _ms(value) -> float:
    return float(value) / 1000.0


@dataclasses.dataclass(frozen=True)
class Settings:
    cfg: _config.Config
    # link and timing
    control_hz: float
    link_timeout_s: float
    source_change_hold_s: float
    serial_baud: int
    serial_write_timeout_s: float
    reopen_s: float
    stats_s: float
    pad_poll_hz: float
    status_heartbeat_s: float
    telemetry_usb_hz: float
    telemetry_wifi_hz: float
    # WiFi
    udp_port: int
    discovery_port: int
    ap_ip: str
    ap_netmask: str
    beacon_s: float
    # input ranges
    joy_axis_range: int
    trigger_axis_range: int
    button_debounce_s: float
    # keyboard
    keyboard_deflection: float
    keyboard_press_s: float
    keyboard_press_queue_max: int
    keyboard_revoke_s: float
    keyboard_post_hz: float
    # dashboard
    http_port: int
    dashboard_max_fps: float
    keepalive_s: float
    event_log_keep: int
    stale_s: float

    @property
    def control_period_s(self) -> float:
        return 1.0 / self.control_hz

    @classmethod
    def load(cls, path=None) -> "Settings":
        return cls.from_config(_config.load(path))

    @classmethod
    def from_text(cls, text: str) -> "Settings":
        return cls.from_config(_config.from_text(text))

    @classmethod
    def from_config(cls, cfg: _config.Config) -> "Settings":
        def get(name):
            if name in cfg:
                return cfg[name]
            raise SettingsError("config.h has no #define %s" % name)

        s = cls(
            cfg=cfg,
            control_hz=float(get("CONTROL_PACKET_HZ")),
            link_timeout_s=_ms(get("LINK_TIMEOUT_MS")),
            source_change_hold_s=_ms(get("RELAY_SOURCE_CHANGE_HOLD_MS")),
            serial_baud=int(get("DEBUG_SERIAL_BAUD")),
            serial_write_timeout_s=_ms(get("RELAY_SERIAL_WRITE_TIMEOUT_MS")),
            reopen_s=_ms(get("RELAY_REOPEN_MS")),
            stats_s=_ms(get("RELAY_STATS_MS")),
            pad_poll_hz=float(get("RELAY_PAD_POLL_HZ")),
            status_heartbeat_s=_ms(get("STATUS_HEARTBEAT_MS")),
            telemetry_usb_hz=float(get("TELEMETRY_USB_HZ")),
            telemetry_wifi_hz=float(get("TELEMETRY_WIFI_HZ")),
            udp_port=int(get("WIFI_UDP_PORT")),
            discovery_port=int(get("WIFI_DISCOVERY_PORT")),
            ap_ip=str(get("WIFI_AP_IP")),
            ap_netmask=str(get("WIFI_AP_NETMASK")),
            beacon_s=_ms(get("WIFI_BEACON_MS")),
            joy_axis_range=int(get("JOY_AXIS_RANGE")),
            trigger_axis_range=int(get("TRIGGER_AXIS_RANGE")),
            button_debounce_s=_ms(get("BUTTON_DEBOUNCE_MS")),
            keyboard_deflection=float(get("KEYBOARD_DEFLECTION")),
            keyboard_press_s=_ms(get("KEYBOARD_PRESS_MS")),
            keyboard_press_queue_max=int(get("KEYBOARD_PRESS_QUEUE_MAX")),
            keyboard_revoke_s=_ms(get("KEYBOARD_REVOKE_MS")),
            keyboard_post_hz=float(get("KEYBOARD_POST_HZ")),
            http_port=int(get("DASHBOARD_HTTP_PORT")),
            dashboard_max_fps=float(get("DASHBOARD_MAX_FPS")),
            keepalive_s=_ms(get("DASHBOARD_KEEPALIVE_MS")),
            event_log_keep=int(get("DASHBOARD_EVENT_LOG_KEEP")),
            stale_s=_ms(get("DASHBOARD_STALE_MS")),
        )
        s.validate()
        return s

    def validate(self) -> None:
        """Refuses values that would make the PC side unsafe or meaningless."""
        problems = []
        if self.serial_baud in BRIDGE_RESET_BAUDS:
            problems.append("DEBUG_SERIAL_BAUD %d resets the RA4M1 through the USB bridge"
                            % self.serial_baud)
        if self.source_change_hold_s <= self.link_timeout_s:
            problems.append("RELAY_SOURCE_CHANGE_HOLD_MS must be > LINK_TIMEOUT_MS, or a source "
                            "change would not look like a link loss to the RA4M1")
        if self.keyboard_press_s <= self.button_debounce_s:
            problems.append("KEYBOARD_PRESS_MS must be > BUTTON_DEBOUNCE_MS, or a key press could "
                            "be eaten by the RA4M1's debounce lockout")
        if self.keyboard_revoke_s <= self.link_timeout_s:
            problems.append("KEYBOARD_REVOKE_MS must be > LINK_TIMEOUT_MS (the deadman goes "
                            "neutral first, the revoke comes later)")
        for name in ("control_hz", "pad_poll_hz", "keyboard_post_hz", "dashboard_max_fps",
                     "telemetry_usb_hz", "telemetry_wifi_hz"):
            if getattr(self, name) <= 0:
                problems.append("%s must be > 0" % name)
        if not 0.0 < self.keyboard_deflection <= 1.0:
            problems.append("KEYBOARD_DEFLECTION must be in (0, 1]")
        if self.keyboard_press_queue_max < 1:
            problems.append("KEYBOARD_PRESS_QUEUE_MAX must be >= 1")
        if problems:
            raise SettingsError("; ".join(problems))

    def value(self, name: str, default: Optional[object] = None):
        """Any other config.h value, or `default` when config.h does not define it."""
        if name in self.cfg:
            return self.cfg[name]
        return default
