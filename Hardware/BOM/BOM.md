# Bill of Materials

## Electronics

| Qty | Part | Notes |
|---|---|---|
| 1 | Arduino UNO R4 WiFi | Main board. RA4M1 runs turret logic, on-board ESP32-S3-MINI-1-N8 runs the custom firmware in `Software/Custom Firmware/`. |
| 2 | NEMA17 stepper motor | [Amazon AU, ASIN B0CB3459ZH](https://www.amazon.com.au/gp/product/B0CB3459ZH/ref=ox_sc_act_title_7?smid=A2A43OOEZ0V29O&psc=1). |
| 1 | CNC Shield V3 | [Amazon AU, ASIN B07DMSWR92](https://www.amazon.com.au/gp/product/B07DMSWR92/ref=ox_sc_act_title_8?smid=A3OCK02B53OJFV&psc=1). See `Mechanical/CAD/vendor-parts/cnc-shield-v3/`. |
| 2 | A4988 stepper driver | Set Vref before connecting motors: `Vref = Imax × 8 × Rsense`. |
| 1 | L298N dual H-bridge motor driver | 5V-EN, ENA and ENB jumpers removed. Flywheels on channel A (OUT1/OUT2) — see `Mechanical/CAD/vendor-parts/l298n-stepper-driver/` and `Hardware/Wiring Diagram/`. |
| 2 | DC motor (flywheels) | From a [Zuru XSHOT Insanity Motorised Clip Blaster & 72 Darts](https://www.amazon.com.au/gp/product/B0DYKXYM5X/ref=ox_sc_act_title_4?smid=ANEGB3WVEVKZB&psc=1) — the same blaster also supplied the dart magazines and darts. |
| 1 | Hobby servo (trigger) | Drives the dart pusher through a three-position REST/PRE-FIRE/FIRE sequence — `DECISIONS.md` D27. |
| 1 | Freenove 8× WS2812 addressable RGB LED module | Data on D13. Kept at half brightness or less. |
| 1 | 16×2 character LCD + I2C backpack | SDA→A4, SCL→A5. Fits the printed frame pocket at a −0.1 mm press fit — `Mechanical/Fitment/`. |
| 1 | Power toggle (A0) | Signal switch to GND, not inline on the battery. |
| 1 | BT/WiFi mode toggle (A1) | AliExpress illuminated latching push button, 5 A. |
| 1 | Xbox Wireless Controller, BLE-capable | Series X\|S (model 1914), or model 1708 updated to firmware 5.x. |
| — | Dupont jumper wires, resistors, 16 AWG wire, XT60 connector, barrel jack (9 V) | |

## Power

| Qty | Part | Notes |
|---|---|---|
| 1 | 9 V battery | Snap-to-barrel lead, Arduino board logic only. |
| 1 | 6S LiPo, 22.2 V nominal (25.2 V full / 19.8 V cutoff) | Stepper + flywheel rail, after an external fuse ([Amazon AU, ASIN B07RCG7RQX](https://www.amazon.com.au/gp/product/B07RCG7RQX/ref=ox_sc_act_title_1?smid=A16H68OXWLBV7Z&psc=1)) on 16 AWG silicone wire. |
| 1 | UBEC, 5 V 5 A (Durlo), input 2S-7S | Fed from the 6S rail after the main fuse; powers a breadboard 5 V rail for the servo, LED module, LCD and L298N logic — `DECISIONS.md` D20. |
| 1 | Capacitor, input smoothing | [Jaycar RE6350, 470 µF 63 V low-ESR](https://www.jaycar.com.au/470uf-63vdc-low-esr-electrolytic-capacitor/p/RE6350). |
| — | Cables for stepper motors | [Amazon AU, ASIN B0B2PD9MN4](https://www.amazon.com.au/gp/product/B0B2PD9MN4/ref=ox_sc_act_title_5?smid=A11XU916U27XD7&psc=1). |

## Mechanical / printed

All structural parts are 3D printed — see `Mechanical/CAD/` for the source (native Fusion
360 in `source/`, sliceable meshes in `meshes/`) and `Mechanical/Gcode/final/` for the print
files actually used. Printed in PETG, with the pan pinion gear (`CFPinion`) in nylon. The
flywheel window is laser-cut acrylic.

Every gear — the tilt pair (8T/40T, module 2, 5:1), the pan planetary (module 2.5, 72T ring,
3 planets, 5:1), and the shooter's rack-and-pinion — was generated with GearForge, the
Fusion 360 add-in built alongside this project (`Mechanical/CAD/README.md`).

| Qty | Part | Notes |
|---|---|---|
| 2 | 608 ball bearing | 22.3 mm OD × 7.1 mm ID as printed. [Amazon AU, ASIN B0G2BHHX5C](https://www.amazon.com.au/gp/product/B0G2BHHX5C/ref=ox_sc_act_title_6?smid=A102Q921QY46T4&psc=1). One each side of the tilt gear, on an 8 mm axle. |
| ? | M2 / M3 / M4 / M5 fasteners | Clearance/self-tap/counterbore sizes per `Mechanical/Fitment/README.md`. |

See `Mechanical/Fitment/` for the full measured-fit table.
