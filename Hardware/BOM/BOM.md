# Bill of Materials

Compiled from `D-SENTRY_SPEC.md`'s hardware table, the reference part models in
`Mechanical/CAD/vendor-parts/`, a 2026-11-13 handoff from a separate Claude.ai Project
(chat history), and the user's own Notion "Meche A4" workspace, which turned out to hold
the actual purchase records and resolved a few conflicts the chat-history handoff couldn't.
"Not recorded" means no source found it, not that it doesn't exist.

## Electronics

| Qty | Part | Notes |
|---|---|---|
| 1 | Arduino UNO R4 WiFi | Main board. RA4M1 runs turret logic, on-board ESP32-S3-MINI-1-N8 runs the custom firmware in `Software/Custom Firmware/`. Already owned. |
| 2 | NEMA17 stepper motor | **Resolved:** [Amazon AU, ASIN B0CB3459ZH](https://www.amazon.com.au/gp/product/B0CB3459ZH/ref=ox_sc_act_title_7?smid=A2A43OOEZ0V29O&psc=1) — the user's own Notion purchase record. (A separate chat-history handoff had guessed Jaycar YM2756; that guess was wrong.) |
| 1 | CNC Shield V3 | [Amazon AU, ASIN B07DMSWR92](https://www.amazon.com.au/gp/product/B07DMSWR92/ref=ox_sc_act_title_8?smid=A3OCK02B53OJFV&psc=1). Protoneer-style V3.00 clone. 4 onboard capacitors (marked "VEHT", one per driver socket) — voltage rating never read, needs to be ≥35 V. Has its own onboard fuse. See `Mechanical/CAD/vendor-parts/cnc-shield-v3/`. |
| 2 | A4988 stepper driver | Confirmed via Amazon AU order history, but no listing link recorded. Set Vref before connecting motors — see `../../DECISIONS.md` D25; the driver's Rsense marking was never read. |
| 1 | L298N dual H-bridge motor driver | Amazon AU, link not recorded. Its own onboard capacitor must read ≥35 V (never confirmed). 5V-EN, ENA and ENB jumpers all removed. Flywheels on channel A (OUT1/OUT2) — see `Mechanical/CAD/vendor-parts/l298n-stepper-driver/`. An earlier MOSFET-based flywheel driver was bought and built first, then set aside in favor of this board — [Amazon AU, ASIN B0H6WVNZ9Y](https://www.amazon.com.au/gp/product/B0H6WVNZ9Y/ref=ox_sc_act_title_3?smid=A1AR0TTSXUY21A&psc=1), `../../DECISIONS.md` D21. |
| 2 | DC motor (flywheels) | From a [Zuru XSHOT Insanity Motorised Clip Blaster & 72 Darts (Amazon AU)](https://www.amazon.com.au/gp/product/B0DYKXYM5X/ref=ox_sc_act_title_4?smid=ANEGB3WVEVKZB&psc=1) — the same blaster also supplied the dart magazines and darts below. Ran on 6×AA in the original toy (~9 V nominal, ~7.2 V under load). Stall current never measured. |
| 1 | Hobby servo (trigger) | Already owned — exact model never confirmed anywhere, including the user's own Notion BOM. A Jaycar YM2760 (4.8-6 V, 5.5-6.5 kg·cm, 47 g) was discussed as a candidate in chat but not confirmed. The CAD reference part (`vendor-parts/servo-fs90r/`) is a third, different model (FS90R) — treat all as unconfirmed against each other. |
| 1 | Freenove 8× WS2812 addressable RGB LED module | Already owned. Data on D13. Kept at half brightness or less to limit draw on the 5 V rail. Use Adafruit NeoPixel ≥1.12.1 — Freenove's own library isn't confirmed to run on the UNO R4's Renesas core. |
| 1 | 16×2 character LCD + I2C backpack | SDA→A4, SCL→A5. Address 0x27 or 0x3F — never actually scanned; run `tools/i2c_scan`. Fits the printed frame pocket at a −0.1 mm press fit (`Mechanical/Fitment/`). Currently blank with the backlight on after the 5 V rail changed (D20/D25) — unresolved. |
| 1 | Power toggle (A0) | Candidate: a Jaycar 15 A SPST marine toggle, 150 mm leads — its listing shows both 12 V and 24 V ratings, unresolved which applies, and the final purchase was never confirmed. Now wired as a signal switch to GND, not inline on the battery. |
| 1 | BT/WiFi mode toggle (A1) | Candidate: an AliExpress illuminated latching push button, 5 A, item 1005008024309525 — final purchase not confirmed. |
| 1 | Xbox Wireless Controller, BLE-capable | Already owned. Series X\|S (model 1914), or model 1708 updated to firmware 5.x. Pre-5.x 1708 controllers are Bluetooth Classic only and will never pair. |
| — | Dupont jumper wires, resistors, 16 AWG wire, XT60 connector, barrel jack (9 V) | Already owned. |

## Power

| Qty | Part | Notes |
|---|---|---|
| 1 | 9 V battery | Already owned. Snap-to-barrel lead, Arduino board logic only (not the 5 V rail below). |
| 1 | 6S LiPo, 22.2 V nominal (25.2 V full / 19.8 V cutoff) | Already owned. Stepper + flywheel rail, after an external fuse ([Amazon AU, ASIN B07RCG7RQX](https://www.amazon.com.au/gp/product/B07RCG7RQX/ref=ox_sc_act_title_1?smid=A16H68OXWLBV7Z&psc=1) — "fuse to stop exploding") on 16 AWG silicone wire (the CNC Shield's own fuse only covers its own branch). Brand/capacity/C-rating not recorded; `Mechanical/CAD/vendor-parts/battery-6s-5200mah/`'s reference model (5200 mAh 60C) may or may not be the actual pack. |
| 1 | LiPo charger | Not recorded anywhere, including Notion. |
| 1 | UBEC, 5 V 5 A (e.g. Durlo), input 2S-7S | Added after both `D-SENTRY_SPEC.md` and the Notion BOM were written — link not recorded. Fed from the 6S rail after the main fuse; powers a breadboard 5 V rail for the servo, LED module, LCD and L298N logic. Output should read 4.75-5.5 V before anything else is connected. See `../../DECISIONS.md` D20. |
| 0-1 | Capacitor, input smoothing | [Jaycar RE6350, 470 µF 63 V low-ESR](https://www.jaycar.com.au/470uf-63vdc-low-esr-electrolytic-capacitor/p/RE6350) — bought, but skipped on the final build by choice. |
| 0 | Flyback diode | [Jaycar ZR1048, 1N5822 Schottky 40 V 3 A](https://www.jaycar.com.au/diode-1n5822-schottky-40v-3a-do-27/p/ZR1048) — bought for the original MOSFET flywheel driver; not needed with the L298N, which has its own diodes. |
| — | Cables for stepper motors | [Amazon AU, ASIN B0B2PD9MN4](https://www.amazon.com.au/gp/product/B0B2PD9MN4/ref=ox_sc_act_title_5?smid=A11XU916U27XD7&psc=1). |

## Mechanical / printed

All structural parts are 3D printed — see `Mechanical/CAD/` for the source (native Fusion
360 in `source/`, sliceable meshes in `meshes/`) and `Mechanical/Gcode/final/` for the print
files actually used. Printed in PETG, with the pan pinion gear (`CFPinion`) in nylon. The
flywheel window is laser-cut acrylic, not printed (`Mechanical/Fitment/`).

Gear tooth counts, where recorded: **tilt** 8T/40T, module 2, +0.6/−0.6 profile shift, 48.2 mm
centre distance (this is a 5:1 ratio, not the "4:1" the rest of this repo calls it — see
`../../DECISIONS.md` D23). **Pan** 5:1 planetary, module 2.5, 72T ring, 3 planets. All of this
project's gears — the tilt pair, the pan planetary, and the shooter's rack-and-pinion — were
generated with **GearForge** (`Mechanical/CAD/README.md`), per the project owner directly.

| Qty | Part | Notes |
|---|---|---|
| 2 | 608 ball bearing | Measured 22.3 mm OD × 7.1 mm ID on the actual print (nominal 608 is 22 × 7). [Amazon AU, ASIN B0G2BHHX5C](https://www.amazon.com.au/gp/product/B0G2BHHX5C/ref=ox_sc_act_title_6?smid=A102Q921QY46T4&psc=1). One each side of the tilt gear, on an 8 mm axle. |
| ? | M2 / M3 / M4 / M5 fasteners | Clearance/self-tap/counterbore sizes measured per fastener size — see `Mechanical/Fitment/README.md`. No head style, length, or purchase source recorded for the fasteners themselves. |
| — | Magnets | Not discussed anywhere found, Notion included. (A hall-sensor + magnet tilt limit was planned and built, mounting points were even added to the frame, then later removed — `../../DECISIONS.md` D24.) |

See `Mechanical/Fitment/` for the full measured-fit table.
