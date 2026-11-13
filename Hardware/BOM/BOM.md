# Bill of Materials

Compiled from `D-SENTRY_SPEC.md`'s hardware table, the reference part models in
`Mechanical/CAD/vendor-parts/`, and a 2026-11-13 handoff from a separate Claude.ai Project
that had chat history about actual purchases. "Not recorded" means no source found it, not
that it doesn't exist — check before assuming it's missing.

## Electronics

| Qty | Part | Notes |
|---|---|---|
| 1 | Arduino UNO R4 WiFi | Main board. RA4M1 runs turret logic, on-board ESP32-S3-MINI-1-N8 runs the custom firmware in `Software/Custom Firmware/`. |
| 2 | NEMA17 stepper motor | **Conflicting sources:** this repo's own BOM notes said "Amazon"; the handoff's chat history says Jaycar **YM2756**, 1.7 A/phase, 5 mm shaft. Unresolved — confirm which is actually installed. |
| 1 | CNC Shield V3 | Protoneer-style V3.00 clone. 4 onboard capacitors (marked "VEHT", one per driver socket) — voltage rating never read, needs to be ≥35 V. Has its own onboard fuse. See `Mechanical/CAD/vendor-parts/cnc-shield-v3/`. |
| 2 | A4988 stepper driver | Confirmed via Amazon AU order history. (DRV8825 is compatible but wasn't what was bought.) Set Vref before connecting motors — see `../../DECISIONS.md` D25; the driver's Rsense marking was never read. |
| 1 | L298N dual H-bridge motor driver | Amazon AU. Its own onboard capacitor must read ≥35 V (never confirmed). 5V-EN, ENA and ENB jumpers all removed. Flywheels on channel A (OUT1/OUT2) — see `Mechanical/CAD/vendor-parts/l298n-stepper-driver/`. An earlier MOSFET-based flywheel driver was built and set aside in favor of this board — `../../DECISIONS.md` D21. |
| 2 | DC motor (flywheels) | Salvaged from a **Zuru XSHOT Insanity** blaster (brushed). Ran on 6×AA in the original toy (~9 V nominal, ~7.2 V under load). Stall current never measured. |
| 1 | Hobby servo (trigger) | "Already owned" — exact model never confirmed. A Jaycar YM2760 (4.8-6 V, 5.5-6.5 kg·cm, 47 g) was discussed as a candidate but not confirmed as the one installed. The CAD reference part (`vendor-parts/servo-fs90r/`) is a different model again (FS90R) — treat all three as unconfirmed against each other. |
| 1 | Freenove 8× WS2812 addressable RGB LED module | Data on D13. Use Adafruit NeoPixel ≥1.12.1 — Freenove's own library isn't confirmed to run on the UNO R4's Renesas core. |
| 1 | 16×2 character LCD + I2C backpack | SDA→A4, SCL→A5. Address 0x27 or 0x3F — never actually scanned; run `tools/i2c_scan`. Currently blank with the backlight on after the 5 V rail changed (D20/D25) — unresolved. |
| 1 | Power toggle (A0) | Candidate: a Jaycar 15 A SPST marine toggle, 150 mm leads — its listing shows both 12 V and 24 V ratings, unresolved which applies, and the final purchase was never confirmed. Now wired as a signal switch to GND, not inline on the battery. |
| 1 | BT/WiFi mode toggle (A1) | Candidate: an AliExpress illuminated latching push button, 5 A, item 1005008024309525 — final purchase not confirmed. |
| 1 | Xbox Wireless Controller, BLE-capable | Series X\|S (model 1914), or model 1708 updated to firmware 5.x. Pre-5.x 1708 controllers are Bluetooth Classic only and will never pair. |

## Power

| Qty | Part | Notes |
|---|---|---|
| 1 | 9 V battery | Snap-to-barrel lead, Arduino board logic only (not the 5 V rail below). |
| 1 | 6S LiPo, 22.2 V nominal (25.2 V full / 19.8 V cutoff) | Stepper + flywheel rail, after an external ~10 A fuse (the CNC Shield's own fuse only covers its own branch) on 16 AWG silicone wire. Brand/capacity/C-rating not recorded; `Mechanical/CAD/vendor-parts/battery-6s-5200mah/`'s reference model (5200 mAh 60C) may or may not be the actual pack. |
| 1 | LiPo charger | Not recorded anywhere. |
| 1 | UBEC, 5 V 5 A (e.g. Durlo), input 2S-7S | Added after `D-SENTRY_SPEC.md` was written — not in the original spec. Fed from the 6S rail after the main fuse; powers a breadboard 5 V rail for the servo, LED module, LCD and L298N logic. Output should read 4.75-5.5 V before anything else is connected. See `../../DECISIONS.md` D20. |

## Mechanical / printed

All structural parts are 3D printed — see `Mechanical/CAD/` for the source (native Fusion
360 in `source/`, sliceable meshes in `meshes/`) and `Mechanical/Gcode/final/` for the print
files actually used. Printed in PETG, with the pan pinion gear (`CFPinion`) in nylon.

Gear tooth counts, where recorded: **tilt** 8T/40T, module 2, +0.6/−0.6 profile shift, 48.2 mm
centre distance (this is a 5:1 ratio, not the "4:1" the rest of this repo calls it — see
`../../DECISIONS.md` D23). **Pan** 5:1 planetary, module 2.5, 72T ring, 3 planets, built with
Fusion's own Spur Gear tool. Whether GearForge (`Mechanical/CAD/README.md`) was used for
either is not confirmed — see that file.

| Qty | Part | Notes |
|---|---|---|
| 2 | 608 ball bearing (22 mm OD × 7 mm wide) | One each side of the tilt gear, on an 8 mm axle. Pan-axis bearing size not recorded. |
| ? | M3 set screw | Secures the tilt pinion on its D-flat shaft. |
| ? | Bolts / nuts (other) | No list beyond the set screw above. Sizes were fit-tested in `Mechanical/Gcode/testing/BoltFit`, `NutTest`, `BBFit`, `BBOD`, but no result was recorded — see `Mechanical/Fitment/`. |
| — | Magnets | Not discussed anywhere found. (A hall-sensor + magnet tilt limit was built and later removed — `../../DECISIONS.md` D24 — unrelated to the `MagnetTest`/`MagTest`/`MagTestV2` fit prints, whose purpose is unrecorded.) |
| — | PTFE liner | Low-friction liner referenced in `Mechanical/CAD/meshes/Enclosure_top-ptfe.stl`. |

See `Mechanical/Fitment/` for anything actually measured during the fit tests above.
