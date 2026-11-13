# Bill of Materials

Compiled from `D-SENTRY_SPEC.md`'s hardware table and the reference part models in
`Mechanical/CAD/vendor-parts/`. Exact fastener/bearing sizes and purchase sources were not
recorded anywhere in the project files, so fill those in yourself rather than relying on this
table alone.

## Electronics

| Qty | Part | Notes |
|---|---|---|
| 1 | Arduino UNO R4 WiFi | Main board. RA4M1 runs turret logic, on-board ESP32-S3-MINI-1-N8 runs the custom firmware in `Software/Custom Firmware/`. |
| 2 | NEMA17 stepper motor | Pan and tilt axes. |
| 1 | CNC Shield V3 | Carries the stepper drivers; see `Mechanical/CAD/vendor-parts/cnc-shield-v3/`. |
| 2 | A4988 or DRV8825 stepper driver | Socketed into the CNC Shield. Read the microstep jumpers before wiring — see `D-SENTRY_SPEC.md` §8. |
| 1 | L298N dual H-bridge motor driver | One channel (OUT1/OUT2) drives both flywheel motors in parallel. See `Mechanical/CAD/vendor-parts/l298n-stepper-driver/`. |
| 2 | DC motor (flywheels) | Wired as one pair on the L298N's A channel. |
| 1 | Hobby servo (trigger) | Drives the dart pusher; D12. Reference part in CAD is a Fitec FS90R-style 9 g servo — confirm against what you actually installed. |
| 1 | Freenove 8× WS2812 addressable RGB LED module | Status/state indicator; data on D13. |
| 1 | 16×2 character LCD + I2C backpack | A4/A5 (SDA/SCL). Run `tools/i2c_scan` to find its address. |
| 2 | SPST toggle switch | Power and BT/WiFi mode select, A0 and A1, both to GND with internal pull-ups. |
| 1 | Xbox Wireless Controller, BLE-capable | Series X\|S (model 1914), or model 1708 updated to firmware 5.x. Pre-5.x 1708 controllers are Bluetooth Classic only and will never pair. |

## Power

| Qty | Part | Notes |
|---|---|---|
| 1 | 9 V battery | Powers the Arduino board itself via its barrel jack. |
| 1 | 6S LiPo, 22.2 V 5200 mAh 60C | Stepper + flywheel rail, landed on the CNC Shield's terminal block. See `Mechanical/CAD/vendor-parts/battery-6s-5200mah/`. |

## Mechanical / printed

All structural parts are 3D printed — see `Mechanical/CAD/` for the source meshes and
`Mechanical/Gcode/final/` for the print files actually used. Printed in PETG, with the pan
pinion gear (`CFPinion`) in nylon. Gear profiles were generated with GearForge, a Fusion 360
add-in built alongside this project — see `Mechanical/CAD/README.md`.

| Qty | Part | Notes |
|---|---|---|
| ? | Bolts / nuts | Sizes were tested in `Mechanical/Gcode/testing/BoltFit`, `NutTest`, `BBFit`, `BBOD` — fill in the sizes that passed. |
| ? | Ball bearing | Fit tested in `Mechanical/Gcode/testing/BearingFitment` — fill in the bearing size that passed. |
| ? | Magnets | Fit tested in `Mechanical/Gcode/testing/MagnetTest`, `MagTest`, `MagTestV2` — fill in size/grade. |
| — | PTFE liner | Low-friction liner referenced in `Mechanical/CAD/meshes/Enclosure_top-ptfe.stl`. |

See `Mechanical/Fitment/` for anything you measured during these fit tests.
