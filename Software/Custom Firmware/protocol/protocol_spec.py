#!/usr/bin/env python3
# protocol_spec.py -- the single source of truth for the D-Sentry wire format.
#
# Everything that crosses a wire between the RA4M1, the ESP32-S3 fork and the PC relay is
# described here as plain Python data: the four framed packets (field order, types, meaning),
# every enum and bit mask they carry, the scale constants, the CRC, and the D-Sentry AT commands.
# tools/gen_protocol.py turns this file into
#   protocol/dsentry_protocol.h                    C/C++ header (RA4M1 sketch + host tests)
#   ../firmware/UNOR4USBBridge/dsentry_protocol.h  byte-identical copy for the ESP32-S3 fork
#   relay-pc/dsentry/protocol.py                   Python codec for the relay, bench console, tests
#   docs/protocol.md                               human documentation with example frames
#   tests/unit/test_protocol_c.cpp                 C++ check of the header against the examples
# so the three implementations cannot drift. After editing this file run
#   "<repo>/.venv/bin/python" D-SentryCode/tools/gen_protocol.py
# and commit every output; `gen_protocol.py --check` (run by the tests) fails while any is stale.
#
# The contract this encodes is docs/architecture.md section 4 (it overrides D-SENTRY_SPEC.md §5).
# Field names are the contract's camelCase names, kept identical in C, Python and the docs so one
# grep finds all three. Types: u8 i8 u16 i16 u32 i32, and arrays u8[N] / char[N]. All multi-byte
# values are little-endian. Only the standard library may be imported here (nothing is).
#
# SPDX-License-Identifier: MIT

# ---------------------------------------------------------------------------------------------
# Versions and scales. Each becomes a #define in C and a module constant in Python.
# ---------------------------------------------------------------------------------------------
PROTO_VERSION = 1        # reported by AT+DSCAPS?; bump when an AT command changes shape. A new
                         # TelemetryPacket layout bumps TELEMETRY_VERSION only: the ESP32 forwards
                         # AT+DST frames without reading their fields (it checks start, len, CRC)
TELEMETRY_VERSION = 2    # first byte of every TelemetryPacket; bump when its fields change.
                         # 2 (2026-11-03): the controller block (padState..padButtons) appended
ANGLE_SCALE = 100        # wire units per degree: angles travel as centidegrees (i32/i16)
LCD_MIRROR_ROWS = 2      # TelemetryPacket.lcd holds the 16x2 LCD: row 0 then row 1
LCD_MIRROR_COLS = 16

CONSTANTS = [
    # (C/Python name, value, meaning)
    ("DS_PROTO_VERSION", PROTO_VERSION,
     "D-Sentry protocol version; AT+DSCAPS? reports it as dsProto."),
    ("DS_TELEMETRY_VERSION", TELEMETRY_VERSION,
     "TelemetryPacket.version; a receiver that sees another value must not trust the fields."),
    ("DS_ANGLE_SCALE", ANGLE_SCALE,
     "Wire units per degree. Angles are centidegrees (deg * 100), angular rates centidegrees/s."),
    ("DS_TELEMETRY_LCD_ROWS", LCD_MIRROR_ROWS,
     "Rows of the LCD mirrored in TelemetryPacket.lcd."),
    ("DS_TELEMETRY_LCD_COLS", LCD_MIRROR_COLS,
     "Characters per mirrored LCD row (space padded)."),
    ("DS_PAD_BATTERY_UNKNOWN", 255,
     "TelemetryPacket.padBattery when the source reports no level (= XBOX_R4_BATTERY_UNKNOWN)."),
]

# ---------------------------------------------------------------------------------------------
# Framing and CRC (architecture.md 4.1)
# ---------------------------------------------------------------------------------------------
FRAMING = {
    "layout": "start | len | payload[len] | crc",
    "len": "number of payload bytes: everything after len, excluding crc",
    "byte_order": "little-endian",
    "accept": [
        "the first byte is one of the four start bytes",
        "len equals that packet type's exact payload length (never a range)",
        "the CRC over len and the payload matches the crc byte",
    ],
    "resync": ("on any mismatch the deframer drops only the first held byte and searches again "
               "from the next one, so a frame hidden inside a rejected candidate is still found"),
    "why_exact_len": ("ASCII debug text shares the USB port with binary frames. StatusPacket's "
                      "start byte 0x5A is the letter 'Z'; it is safe only because its len byte "
                      "(7) is a control character that text never contains. Research measured "
                      "0 false frames in 2 MB of random printable text with the exact-length "
                      "rule, and about 1 in 256 candidates accepted when any length was allowed."),
}

CRC = {
    "name": "CRC-8/SMBUS",
    "width": 8,
    "poly": 0x07,
    "init": 0x00,
    "refin": False,
    "refout": False,
    "xorout": 0x00,
    "check": 0xF4,          # CRC over the ASCII bytes "123456789"
    "covers": "the len byte and the payload (every byte after start, before crc)",
}

# ---------------------------------------------------------------------------------------------
# Enums: (NAME, value, meaning). C macro = prefix + NAME; Python = IntEnum member NAME.
# ---------------------------------------------------------------------------------------------
ENUMS = [
    {
        "name": "TurretState",
        "prefix": "DS_STATE_",
        "c_name_func": "ds_turret_state_name",
        "doc": "Top-level state of the RA4M1 state machine (spec 3.1, architecture.md 6).",
        "values": [
            ("OFF", 0, "power toggle open: steppers disabled, flywheel 0, servo detached, LEDs dark"),
            ("BOOT", 1, "peripherals starting; lasts one tick after begin()"),
            ("CALIBRATING", 2, "tilt homing"),
            ("PAIRING", 3, "waiting for a connected control source"),
            ("DISARMED", 4, "connected; motion allowed, trigger and flywheels locked"),
            ("ARMED", 5, "connected; flywheels and firing allowed"),
            ("FAULT", 6, "calibration or stall fault (see Fault)"),
        ],
    },
    {
        "name": "Fault",
        "prefix": "DS_FAULT_",
        "c_name_func": "ds_fault_name",
        "doc": "Why the turret is in FAULT (TelemetryPacket.fault).",
        "values": [
            ("NONE", 0, "no fault"),
            ("CAL", 1, "calibration failed; clears only by toggling power"),
            ("STALL", 2, "stall detected; clears after STALL_FAULT_HOLD_MS"),
        ],
    },
    {
        "name": "RadioMode",
        "prefix": "DS_RADIO_",
        "c_name_func": "ds_radio_mode_name",
        "doc": "Radio the RA4M1 wants the ESP32 to run (StatusPacket.wantMode, AT+DSS wantMode). "
               "Mirrors the BT/WiFi toggle on A1.",
        "values": [
            ("BLE", 0, "the turret pairs directly with the pad over BLE"),
            ("WIFI", 1, "the PC relay holds the pad and sends ControlPackets over UDP"),
        ],
    },
    {
        "name": "EspMode",
        "prefix": "DS_ESP_MODE_",
        "c_name_func": "ds_esp_mode_name",
        "doc": "Radio the ESP32 reports running (AT+DSS reply espMode, TelemetryPacket.espMode).",
        "values": [
            ("BLE", 0, "BLE client running"),
            ("WIFI", 1, "WiFi station or access point running"),
            ("NONE", 2, "no D-Sentry radio started yet"),
            ("RESTARTING", 3, "the other radio was running; the ESP32 stored the wanted mode and "
                              "restarts into it about 50 ms after this reply"),
            ("UNKNOWN", 255, "RA4M1 side only: no DSS reply yet, or legacy firmware"),
        ],
    },
    {
        "name": "LinkSource",
        "prefix": "DS_LINK_SRC_",
        "c_name_func": "ds_link_source_name",
        "doc": "Which input the RA4M1 is obeying (TelemetryPacket.linkSource). USB wins while a "
               "valid USB ControlPacket arrived within LINK_TIMEOUT_MS (architecture.md 4.3).",
        "values": [
            ("NONE", 0, "no live source"),
            ("BLE", 1, "the pad over BLE, through AT+XBOXSTATE?"),
            ("WIFI", 2, "the PC relay over UDP, through AT+XBOXSTATE?"),
            ("USB", 3, "ControlPackets over the USB serial port (bench override)"),
        ],
    },
    {
        "name": "PadState",
        "prefix": "DS_PAD_STATE_",
        "c_name_func": "ds_pad_state_name",
        "doc": "Controller connection state (TelemetryPacket.padState). BLE source: the "
               "AT+XBOXSTATE? state byte (byte 16, enum xbox_r4_conn_state in "
               "protocol/xbox_r4_protocol.h) as sent; WiFi source: the same byte, which the "
               "ESP32 sets to 0 or 3; USB source: 0 when the ControlPacket says connected, else 3.",
        "values": [
            ("CONNECTED", 0, "reports flowing (BLE), or the source says connected (WiFi, USB)"),
            ("WAITING_FIRST_REPORT", 1, "BLE link up, encryption or subscription in progress"),
            ("FOUND", 2, "BLE: the pad was seen in a scan, connecting"),
            ("SCANNING", 3, "BLE: no pad connected, scanning; WiFi or USB: the source is not "
                            "connected"),
            ("OFF", 4, "BLE: the ESP32's Xbox client is not running"),
            ("BLOCKED_BY_HCI", 5, "BLE: the ArduinoBLE HCI bridge owns the radio until AT+RESET"),
            ("NONE", 255, "no control source: no frame arrived within LINK_TIMEOUT_MS"),
        ],
    },
    {
        "name": "FireMode",
        "prefix": "DS_FIRE_MODE_",
        "c_name_func": "ds_fire_mode_name",
        "doc": "Trigger behaviour, toggled by B (TelemetryPacket.fireMode).",
        "values": [
            ("SEMI", 0, "one stroke per pull"),
            ("FULL", 1, "strokes back-to-back while the trigger is held"),
        ],
    },
    {
        "name": "WifiStatus",
        "prefix": "DS_WIFI_STATUS_",
        "c_name_func": "ds_wifi_status_name",
        "doc": "ESP32 WiFi state (AT+DSS reply wifiStatus, BeaconPacket.mode).",
        "values": [
            ("OFF", 0, "WiFi not running"),
            ("CONNECTING", 1, "station connecting"),
            ("STATION_UP", 2, "joined WIFI_SSID as a station"),
            ("AP_UP", 3, "hosting its own access point WIFI_AP_SSID"),
            ("RELAY_HEARD", 4, "a valid ControlPacket arrived from the relay within LINK_TIMEOUT_MS"),
        ],
    },
    {
        "name": "DstResult",
        "prefix": "DS_DST_",
        "c_name_func": "ds_dst_result_name",
        "doc": "AT+DST reply.",
        "values": [
            ("NOT_SENT", 0, "not forwarded: not in WiFi mode, no relay address yet, or the hex "
                            "was not exactly one valid TelemetryPacket"),
            ("SENT", 1, "forwarded by UDP to the relay"),
        ],
    },
]

# ---------------------------------------------------------------------------------------------
# Bit groups: (key, bit, meaning). key is camelCase as in the contract; the C macro is
# prefix + UPPER_SNAKE(key) with value (1 << bit); Python gets an IntFlag and a key table.
# ---------------------------------------------------------------------------------------------
BITS = [
    {
        "name": "ControlFlags",
        "prefix": "DS_CTRL_FLAG_",
        "width": 8,
        "doc": "ControlPacket.flags.",
        "bits": [
            ("connected", 0, "a controller (pad or keyboard) is live; 0 = neutral input, no source"),
            ("sourceWifi", 1, "source: 0 = BLE, 1 = WiFi (the relay)"),
            ("espPairing", 2, "the ESP32 is in pairing mode"),
            ("keyboard", 3, "the relay's dashboard keyboard is driving, not a pad"),
        ],
    },
    {
        "name": "Buttons",
        "prefix": "DS_BTN_",
        "width": 16,
        "doc": "ControlPacket.buttons and TelemetryPacket.padButtons (spec 3.2 mapping: A arm, "
               "B semi/full, X magazine, d-pad left set home, d-pad down go home). Bits 12-15 "
               "exist for the dashboard's controller view: a ControlPacket may carry them and "
               "the RA4M1 never acts on them.",
        "bits": [
            ("a", 0, "A: toggle ARMED/DISARMED"),
            ("b", 1, "B: toggle semi/full"),
            ("x", 2, "X: reset magazine to MAG_CAPACITY"),
            ("y", 3, "Y: unused"),
            ("lb", 4, "left bumper: flywheels"),
            ("rb", 5, "right bumper: unused"),
            ("dpadUp", 6, "d-pad up: unused"),
            ("dpadDown", 7, "d-pad down: go home"),
            ("dpadLeft", 8, "d-pad left: set home"),
            ("dpadRight", 9, "d-pad right: unused"),
            ("start", 10, "Menu (start): unused"),
            ("back", 11, "View (back): unused"),
            ("lsClick", 12, "left stick click: shown on the dashboard, no turret action"),
            ("rsClick", 13, "right stick click: shown, no turret action"),
            ("xbox", 14, "Xbox (guide) button: shown, no turret action"),
            ("share", 15, "Share button: shown, no turret action"),
        ],
    },
    {
        "name": "TelemetryFlags",
        "prefix": "DS_TELEM_FLAG_",
        "width": 16,
        "doc": "TelemetryPacket.flags.",
        "bits": [
            ("linkConnected", 0, "a control source is connected"),
            ("flywheelOn", 1, "flywheel commanded on (ramping or at duty)"),
            ("flywheelReady", 2, "flywheel on for at least FLYWHEEL_SPINUP_MS"),
            ("strokeActive", 3, "a trigger stroke is in progress"),
            ("triggerPulled", 4, "trigger reads pulled (hysteresis applied)"),
            ("homing", 5, "a go-home position move is running"),
            ("calibrated", 6, "tilt calibration finished; tilt bounds active"),
            ("powerToggleOn", 7, "power toggle (A0) reads on"),
            ("modeToggleWifi", 8, "mode toggle (A1) reads WiFi"),
            ("stallEnabled", 9, "stall detector active"),
            ("emptyMagBlink", 10, "empty-magazine blink showing"),
            ("driversEnabled", 11, "stepper drivers enabled (EN low)"),
            ("espLegacyFirmware", 12, "the ESP32 answered AT+DSCAPS? with ERROR (old fork/stock)"),
            ("lcdPresent", 13, "an LCD answered on I2C"),
            ("espAlive", 14, "the ESP32 answered within LINK_TIMEOUT_MS"),
        ],
    },
    {
        "name": "Caps",
        "prefix": "DS_CAP_",
        "width": 8,
        "doc": "AT+DSCAPS? capBits: what the ESP32 firmware supports.",
        "bits": [
            ("rumble", 0, "rumble in BLE mode (AT+DSS rumble fields reach the pad)"),
            ("wifi", 1, "WiFi mode (station/AP, UDP ControlPackets)"),
            ("mode", 2, "radio switching through AT+DSS wantMode"),
            ("telemetry", 3, "AT+DST forwarding to the relay"),
        ],
    },
]

# ---------------------------------------------------------------------------------------------
# Packets. Field: name, type, doc; optional "ref" (an ENUMS/BITS name, for the docs), "default"
# (a number, a CONSTANTS name, an enum macro such as DS_PAD_STATE_NONE, or the fill character
# for a char array) and "since" (the packet version that appended the field, default 1).
# Fields with a "since" are integers at the end, in rising order, and the packet then names its
# version constant in "version". Each older version is an exact length the PC deframer accepts
# when asked (Deframer(accept_legacy=True)), decoded with the newer fields at their defaults;
# the C deframer accepts the current layout only. "example" gives one value per field;
# gen_protocol.py encodes it for docs/protocol.md and the C++ test.
# ---------------------------------------------------------------------------------------------
PACKETS = [
    {
        "name": "ControlPacket",
        "id": "control",
        "start": 0xA5,
        "doc": "Controller input (spec 5). Stick up is negative (the Bluepad32/SDL convention); "
               "the RA4M1 applies JOY_INVERT_Y.",
        "carried": [
            "PC relay -> ESP32 by UDP to WIFI_UDP_PORT at CONTROL_PACKET_HZ, always sent "
            "(connected = 0 when there is no input source) so the ESP32 knows where to reply",
            "PC (relay or bench console) -> RA4M1 over USB serial, accepted only when "
            "DEBUG_ACCEPT_USB_CONTROL; wins over the radio while fresh",
            "the ESP32 does not forward it raw: in WiFi mode it turns the latest valid one "
            "into the 20-byte AT+XBOXSTATE? state the RA4M1 polls",
        ],
        "fields": [
            {"name": "flags", "type": "u8", "ref": "ControlFlags", "doc": "DS_CTRL_FLAG_* bits"},
            {"name": "rx", "type": "i16", "doc": "right stick X, -512..511, right positive"},
            {"name": "ry", "type": "i16", "doc": "right stick Y, -512..511, up negative"},
            {"name": "rt", "type": "u16", "doc": "right trigger, 0..1023"},
            {"name": "buttons", "type": "u16", "ref": "Buttons", "doc": "DS_BTN_* bits"},
            {"name": "seq", "type": "u8", "doc": "increments per packet, wraps; the RA4M1 counts "
                                                 "gaps and echoes it in TelemetryPacket.lastCtrlSeq"},
        ],
        # Same values as research/pc/udp_probe_codec.py, whose frame was
        # a5 0a 03 00 fe ff 01 ff 03 11 08 07 ac -- an independent check of this generator.
        "example": {"flags": 0x03, "rx": -512, "ry": 511, "rt": 1023, "buttons": 0x0811,
                    "seq": 7},
    },
    {
        "name": "StatusPacket",
        "id": "status",
        "start": 0x5A,
        "doc": "Turret status and rumble requests (spec 5, plus rumbleSeq).",
        "carried": [
            "RA4M1 -> PC over USB serial, on change and every STATUS_HEARTBEAT_MS",
            "ESP32 -> PC relay by UDP (WiFi mode), to the source of the last valid ControlPacket, "
            "so the relay rumbles the pad it holds",
            "RA4M1 -> ESP32 carries the same values as AT+DSS arguments, not as a frame",
        ],
        "fields": [
            {"name": "wantMode", "type": "u8", "ref": "RadioMode", "doc": "radio the RA4M1 wants "
                                                                         "(mirrors the A1 toggle)"},
            {"name": "rumbleStrong", "type": "u8", "doc": "strong (low-frequency) motor, 0..255"},
            {"name": "rumbleWeak", "type": "u8", "doc": "weak (high-frequency) motor, 0..255"},
            {"name": "rumbleMs", "type": "u16", "doc": "rumble length in ms; 0 = stop"},
            {"name": "state", "type": "u8", "ref": "TurretState", "doc": "turret state"},
            {"name": "rumbleSeq", "type": "u8", "doc": "increments on every new rumble command, "
                                                       "so a heartbeat repeat is not a new shot"},
        ],
        "example": {"wantMode": 1, "rumbleStrong": 200, "rumbleWeak": 120, "rumbleMs": 80,
                    "state": 5, "rumbleSeq": 42},
    },
    {
        "name": "TelemetryPacket",
        "id": "telemetry",
        "start": 0xD5,
        "version": "DS_TELEMETRY_VERSION",
        "doc": "Everything the dashboard shows. Start byte >= 0x80 so it is never ASCII. "
               "Pan: positive = counter-clockwise seen from above (stick right turns the turret "
               "clockwise, i.e. negative; PAN_DIR_INVERT fixes the hardware side). Tilt: 0 = "
               "calibrated bottom plus backoff, positive = barrel up.",
        "carried": [
            "RA4M1 -> PC over USB serial every 1000/TELEMETRY_USB_HZ ms",
            "RA4M1 -> ESP32 as AT+DST=<hex> every 1000/TELEMETRY_WIFI_HZ ms in WiFi mode; the "
            "ESP32 forwards the bytes unchanged by UDP to the relay",
        ],
        "fields": [
            {"name": "version", "type": "u8", "default": "DS_TELEMETRY_VERSION",
             "doc": "DS_TELEMETRY_VERSION"},
            {"name": "seq", "type": "u8", "doc": "increments per TelemetryPacket, wraps"},
            {"name": "uptimeMs", "type": "u32", "doc": "RA4M1 millis() when built"},
            {"name": "state", "type": "u8", "ref": "TurretState", "doc": "turret state"},
            {"name": "fault", "type": "u8", "ref": "Fault", "doc": "fault code"},
            {"name": "fireMode", "type": "u8", "ref": "FireMode", "doc": "0 semi, 1 full"},
            {"name": "linkSource", "type": "u8", "ref": "LinkSource", "doc": "active control "
                                                                           "source"},
            {"name": "flags", "type": "u16", "ref": "TelemetryFlags", "doc": "DS_TELEM_FLAG_* bits"},
            {"name": "panCdeg", "type": "i32", "doc": "pan position, centidegrees, + = CCW from "
                                                      "above"},
            {"name": "tiltCdeg", "type": "i32", "doc": "tilt position, centidegrees, + = up"},
            {"name": "panVelCdegS", "type": "i16", "doc": "pan velocity, centidegrees/s"},
            {"name": "tiltVelCdegS", "type": "i16", "doc": "tilt velocity, centidegrees/s"},
            {"name": "homePanCdeg", "type": "i32", "doc": "stored home pan, centidegrees"},
            {"name": "homeTiltCdeg", "type": "i32", "doc": "stored home tilt, centidegrees"},
            {"name": "ammo", "type": "u8", "doc": "darts left"},
            {"name": "magCapacity", "type": "u8", "doc": "MAG_CAPACITY"},
            {"name": "shotsTotal", "type": "u16", "doc": "strokes completed since boot, wraps"},
            {"name": "flywheelDuty", "type": "u8", "doc": "ENA duty now, 0..255"},
            {"name": "servoDeg", "type": "u8", "doc": "trigger servo commanded angle, degrees"},
            {"name": "stallSample", "type": "u16", "doc": "latest A2 reading, ADC counts"},
            {"name": "stallBaseline", "type": "u16", "doc": "stall EMA baseline, ADC counts"},
            {"name": "stallPeak", "type": "u16", "doc": "largest |sample - baseline| of the "
                                                        "current or last move, ADC counts"},
            {"name": "loopHz", "type": "u16", "doc": "loop() iterations per second, saturates"},
            {"name": "ctrlRxHz", "type": "u16", "doc": "valid control frames per second from the "
                                                       "active source"},
            {"name": "crcErrors", "type": "u16", "doc": "control-path CRC errors since boot, wraps"},
            {"name": "seqGaps", "type": "u16", "doc": "control seq gaps since boot, wraps"},
            {"name": "stepJitterPermille", "type": "u16", "doc": "worst step jitter of the last "
                                                                 "stats window, permille of the "
                                                                 "step period"},
            {"name": "lastCtrlSeq", "type": "u8", "doc": "seq of the last accepted ControlPacket "
                                                         "(echo for round-trip latency)"},
            {"name": "espMode", "type": "u8", "ref": "EspMode", "doc": "radio the ESP32 reports"},
            {"name": "ledR", "type": "u8", "doc": "LED 0 red as sent, after the brightness clamp"},
            {"name": "ledG", "type": "u8", "doc": "LED 0 green as sent"},
            {"name": "ledB", "type": "u8", "doc": "LED 0 blue as sent"},
            {"name": "lcd", "type": "char[%d]" % (LCD_MIRROR_ROWS * LCD_MIRROR_COLS), "default": " ",
             "doc": "LCD text, row 0 then row 1, space padded"},
            # Version 2: the controller block, from the active source's latest frame. Sticks,
            # triggers and buttons are neutral while that frame is not connected; with no
            # source at all padState is NONE and the rest neutral / unknown.
            {"name": "padState", "type": "u8", "ref": "PadState", "since": 2,
             "default": "DS_PAD_STATE_NONE", "doc": "controller connection state"},
            {"name": "padBattery", "type": "u8", "since": 2, "default": "DS_PAD_BATTERY_UNKNOWN",
             "doc": "pad battery percent 0..100 (AT+XBOXSTATE? byte 17), 255 = unknown"},
            {"name": "padLx", "type": "i16", "since": 2,
             "doc": "left stick X, -512..511, right positive (BLE only, else 0)"},
            {"name": "padLy", "type": "i16", "since": 2,
             "doc": "left stick Y, -512..511, up negative (BLE only, else 0)"},
            {"name": "padRx", "type": "i16", "since": 2,
             "doc": "right stick X as received, right positive"},
            {"name": "padRy", "type": "i16", "since": 2, "doc": "right stick Y as received, up negative"},
            {"name": "padLt", "type": "u16", "since": 2,
             "doc": "left trigger, 0..1023 (BLE only, else 0)"},
            {"name": "padRt", "type": "u16", "since": 2, "doc": "right trigger as received, 0..1023"},
            {"name": "padButtons", "type": "u16", "ref": "Buttons", "since": 2,
             "doc": "DS_BTN_* levels as received, bits 12-15 included"},
        ],
        "example": {
            "version": TELEMETRY_VERSION, "seq": 200, "uptimeMs": 123456, "state": 5, "fault": 0,
            "fireMode": 1, "linkSource": 1,
            "flags": 0x68C7,   # linkConnected flywheelOn flywheelReady calibrated powerToggleOn
                               # driversEnabled lcdPresent espAlive
            "panCdeg": -4550, "tiltCdeg": 1234, "panVelCdegS": -9000, "tiltVelCdegS": 0,
            "homePanCdeg": 0, "homeTiltCdeg": 300, "ammo": 12, "magCapacity": 15,
            "shotsTotal": 3, "flywheelDuty": 113, "servoDeg": 20, "stallSample": 512,
            "stallBaseline": 510, "stallPeak": 7, "loopHz": 9876, "ctrlRxHz": 100,
            "crcErrors": 2, "seqGaps": 1, "stepJitterPermille": 35, "lastCtrlSeq": 7,
            "espMode": 0, "ledR": 127, "ledG": 47, "ledB": 0,
            "lcd": "ARMED F  AMMO:12" + "BT     Connected",
            "padState": 0, "padBattery": 80, "padLx": 128, "padLy": -300, "padRx": -512,
            "padRy": 511, "padLt": 512, "padRt": 1023,
            "padButtons": 0x6011,   # A, LB, right stick click, Xbox
        },
    },
    {
        "name": "BeaconPacket",
        "id": "beacon",
        "start": 0xD6,
        "doc": "WiFi discovery. The relay listens on WIFI_DISCOVERY_PORT when no --udp host is "
               "given.",
        "carried": [
            "ESP32 -> UDP broadcast on WIFI_DISCOVERY_PORT every WIFI_BEACON_MS while in WiFi mode",
        ],
        "fields": [
            {"name": "ip", "type": "u8[4]", "doc": "the turret's IPv4 address, first octet first "
                                                   "(192.168.4.1 = c0 a8 04 01)"},
            {"name": "udpPort", "type": "u16", "doc": "where to send ControlPackets "
                                                      "(WIFI_UDP_PORT)"},
            {"name": "bootId", "type": "u8", "doc": "random non-zero byte chosen at ESP32 boot"},
            {"name": "mode", "type": "u8", "ref": "WifiStatus", "doc": "WiFi status: 2 station, "
                                                                      "3 own AP, 4 relay heard"},
        ],
        "example": {"ip": (192, 168, 4, 1), "udpPort": 4210, "bootId": 0x5C, "mode": 3},
    },
]

# ---------------------------------------------------------------------------------------------
# D-Sentry AT commands (architecture.md 4.2), served by the ESP32 fork's chAT server on Serial2.
# mode: "read" = AT+NAME?, "write" = AT+NAME=<args>, "run" = AT+NAME.
# Every command answers with its prompt and OK; failures go in the payload.
# args/reply: (name, type, meaning). type "hex" = 2 hex digits per byte, either case.
# ---------------------------------------------------------------------------------------------
AT_COMMANDS = [
    {
        "id": "DSCAPS",
        "name": "+DSCAPS",
        "mode": "read",
        "args": [],
        "reply": [
            ("dsProto", "u8", "the firmware's DS_PROTO_VERSION"),
            ("capBits", "u8", "DS_CAP_* bits"),
        ],
        "doc": "Capability probe. The old fork and stock firmware answer a bare ERROR: the RA4M1 "
               "then treats the ESP32 as legacy (BT mode only, AT+XBOXBEGIN when the state says "
               "OFF).",
        "example": {"args": [], "reply": [PROTO_VERSION, 0x0F]},
    },
    {
        "id": "DSS",
        "name": "+DSS",
        "mode": "write",
        "args": [
            ("wantMode", "u8", "DS_RADIO_*: radio to run"),
            ("rumbleSeq", "u8", "StatusPacket.rumbleSeq; a new value starts the rumble below"),
            ("strong", "u8", "StatusPacket.rumbleStrong, 0..255"),
            ("weak", "u8", "StatusPacket.rumbleWeak, 0..255"),
            ("rumbleMs", "u16", "StatusPacket.rumbleMs; 0 = stop"),
            ("state", "u8", "DS_STATE_*: turret state, for the ESP32's own log"),
        ],
        "reply": [
            ("espMode", "u8", "DS_ESP_MODE_*: radio running now (3 = restarting into wantMode)"),
            ("bootId", "u8", "random non-zero byte chosen at ESP32 boot; a change means it rebooted"),
            ("wifiStatus", "u8", "DS_WIFI_STATUS_*"),
        ],
        "doc": "Status: the StatusPacket fields as decimal arguments. The ESP32 starts the wanted "
               "radio (idempotent). If the other radio is running it replies espMode = 3, stores "
               "the wanted mode in RTC_NOINIT memory, restarts about 50 ms later and boots "
               "straight into it. A new rumbleSeq triggers rumble: BLE mode -> the pad (client "
               "task, 0x2A4D output report); WiFi mode -> a StatusPacket by UDP to the relay. "
               "Sent when its content changes or every STATUS_HEARTBEAT_MS.",
        "example": {"args": [1, 42, 200, 120, 80, 5], "reply": [1, 92, 4]},
    },
    {
        "id": "DST",
        "name": "+DST",
        "mode": "write",
        "args": [
            ("hex", "hex", "one complete TelemetryPacket frame (start..crc), 2 hex digits per byte"),
        ],
        "reply": [
            ("sent", "u8", "DS_DST_*: 1 = forwarded by UDP to the relay, 0 = not"),
        ],
        "doc": "Telemetry. WiFi mode forwards the bytes unchanged by UDP to the relay. The line is "
               "7 + 2 * TelemetryPacket frame length + 2 bytes; the RA4M1 writes Serial2 directly "
               "(D-Sentry never uses the WiFiS3 Modem, whose 127-byte line limit would cut it), "
               "and chAT accepts arguments of 5000 hex characters (research test_at_limits.cpp).",
        # a "hex" argument's example names a packet: the docs show that packet's example frame
        "example": {"args": ["TelemetryPacket"], "reply": [1]},
    },
]

# Existing fork commands D-Sentry uses unchanged (docs only; their wire format is
# protocol/xbox_r4_protocol.h and the repo root's docs/protocol.md).
AT_COMMANDS_EXISTING = [
    ("AT+XBOXSTATE?", "+XBOXSTATE: 20|<20 bytes>",
     "Controller state, polled every 1000/CONTROL_PACKET_HZ ms in both modes. In WiFi mode the "
     "fork builds it from the relay's latest valid ControlPacket (state 0 when connected and "
     "fresher than LINK_TIMEOUT_MS, else 3 scanning)."),
    ("AT+XBOXBEGIN", "+XBOXBEGIN: <code>",
     "Legacy firmware only: start the BLE client whenever a state frame says OFF, rate-limited "
     "by LINK_BEGIN_RETRY_MS."),
    ("AT+SOFTRESETWIFI", "OK",
     "Never sent by D-Sentry. The fork turns it into a prompt+OK no-op while a D-Sentry radio "
     "mode is active, because it would tear WiFi down."),
]
