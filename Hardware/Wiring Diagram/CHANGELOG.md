# Wiring diagram history

Ten revisions, v00 through v09. `turret-wiring.png` at the top level is v09, the current
wiring. `versions/` holds v00-v08.

| Version | What it covers |
|---|---|
| `v00` | The original design: an IRLZ44N MOSFET switches the flywheels, with a Schottky flyback diode. Superseded by an L298N driver board — `../../DECISIONS.md` D21. |
| `v01` | First L298N pass, both channels bridged together for more current, plus a resistor-divider battery-sense line on A2. |
| `v02` | L298N narrowed to channel A alone; adds the LCD1602, a KY-024 hall-effect sensor for a tilt limit, and the POWER/WiFi-BLE toggles. |
| `v03` | Shield header pins relabeled with their CNC-shield silkscreen names (SpnEn, CoolEn, the limit-switch pins) alongside the Arduino pin numbers. |
| `v04` | Flywheels moved to the L298N's channel B. |
| `v05` | The hall-effect sensor removed; the POWER/WiFi-BLE toggles reworked as a signal-only pair; IN3/IN4 pin assignment settled. |
| `v06` | Servo moved to D12; IN4 moved to D11. |
| `v07` | LED data line moved to D13. |
| `v08` | Flywheels moved back to the L298N's channel A. |
| `v09` (current) | The 5V rail changes from the board's own regulator to a dedicated UBEC, feeding a breadboard power rail shared by the servo, LED module, LCD and L298N logic. |

Every one of those pin and channel moves tracks a real decision recorded in `../../DECISIONS.md`.
