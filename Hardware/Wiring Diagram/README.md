# Wiring diagram

`turret-wiring.png` (v09) is the current wiring: the RA4M1 pin map, both steppers, the trigger
servo, the LEDs, the LCD, both toggles, the 6S LiPo and 9 V supplies, the 5 V UBEC rail and the
L298N flywheel driver. Its pins should match the "Final pin map" in `../../D-SENTRY_SPEC.md`.

`versions/` holds v00-v08. Each change below tracks a decision in `../../DECISIONS.md`.

| Version | Change |
|---|---|
| v00 | The first design: an IRLZ44N MOSFET switches the flywheels, with a Schottky flyback diode. The L298N replaced it (D21). |
| v01 | First L298N version: both channels bridged for more current, plus a resistor-divider battery-sense line on A2. |
| v02 | L298N cut to channel A alone; adds the LCD1602, a KY-024 hall sensor as a tilt limit, and the POWER and WiFi/BLE toggles. |
| v03 | Shield header pins carry their CNC-shield silkscreen names (SpnEn, CoolEn, the limit-switch pins) beside the Arduino pin numbers. |
| v04 | Flywheels moved to L298N channel B. |
| v05 | Hall sensor removed; the toggles rewired as a signal-only pair; IN3/IN4 pins settled. |
| v06 | Servo moved to D12; IN4 moved to D11. |
| v07 | LED data moved to D13. |
| v08 | Flywheels moved back to channel A. |
| v09 (current) | The 5 V rail moves from the board's regulator to a dedicated UBEC, feeding a breadboard rail shared by the servo, LEDs, LCD and L298N logic. |
