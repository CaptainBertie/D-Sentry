# Bill of materials

## Electronics

| Qty | Part | Notes |
|---|---|---|
| 1 | Arduino UNO R4 WiFi | The RA4M1 runs the turret; the on-board ESP32-S3-MINI-1-N8 runs the firmware in `Software/Custom Firmware/`. |
| 2 | NEMA17 stepper motor | [Amazon AU, ASIN B0CB3459ZH](https://www.amazon.com.au/gp/product/B0CB3459ZH/ref=ox_sc_act_title_7?smid=A2A43OOEZ0V29O&psc=1). |
| 1 | CNC Shield V3 | [Amazon AU, ASIN B07DMSWR92](https://www.amazon.com.au/gp/product/B07DMSWR92/ref=ox_sc_act_title_8?smid=A3OCK02B53OJFV&psc=1). Model in `Mechanical/CAD/vendor-parts/cnc-shield-v3/`. |
| 2 | A4988 stepper driver | Set Vref before you connect the motors: `Vref = Imax × 8 × Rsense`. |
| 1 | L298N dual H-bridge driver | 5V-EN, ENA and ENB jumpers removed. Flywheels on channel A (OUT1/OUT2). |
| 2 | DC motor (flywheels) | From a [Zuru XSHOT Insanity Motorised Clip Blaster & 72 Darts](https://www.amazon.com.au/gp/product/B0DYKXYM5X/ref=ox_sc_act_title_4?smid=ANEGB3WVEVKZB&psc=1), which also supplied the magazines and darts. |
| 1 | Hobby servo (trigger) | Pushes each dart through a REST/PRE-FIRE/FIRE sequence (`DECISIONS.md` D27). |
| 1 | Freenove 8× WS2812 RGB LED module | Data on D13. Keep it at half brightness or less. |
| 1 | 16×2 character LCD + I2C backpack | SDA → A4, SCL → A5. Press-fits the frame pocket at −0.1 mm (`Mechanical/fitment.md`). |
| 1 | Power toggle (A0) | A signal switch to GND, not inline on the battery. |
| 1 | BT/WiFi mode toggle (A1) | AliExpress illuminated latching push button, 5 A. |
| 1 | Xbox Wireless Controller with BLE | Series X\|S (model 1914), or model 1708 on firmware 5.x. |
| — | Dupont jumpers, resistors, 16 AWG wire, XT60 connector, 9 V barrel jack | |

## Power

| Qty | Part | Notes |
|---|---|---|
| 1 | 9 V battery | Snap-to-barrel lead; powers only the Arduino. |
| 1 | 6S LiPo, 22.2 V nominal (25.2 V full, 19.8 V cutoff) | The stepper and flywheel rail, through an external fuse ([Amazon AU, ASIN B07RCG7RQX](https://www.amazon.com.au/gp/product/B07RCG7RQX/ref=ox_sc_act_title_1?smid=A16H68OXWLBV7Z&psc=1)) on 16 AWG silicone wire. |
| 1 | UBEC, 5 V 5 A (Durlo), 2S-7S input | Fed from the 6S rail after the fuse; powers a breadboard 5 V rail for the servo, LEDs, LCD and L298N logic (`DECISIONS.md` D20). |
| 1 | 470 µF 63 V low-ESR capacitor | Input smoothing. [Jaycar RE6350](https://www.jaycar.com.au/470uf-63vdc-low-esr-electrolytic-capacitor/p/RE6350). |
| — | Stepper motor cables | [Amazon AU, ASIN B0B2PD9MN4](https://www.amazon.com.au/gp/product/B0B2PD9MN4/ref=ox_sc_act_title_5?smid=A11XU916U27XD7&psc=1). |

## Printed and mechanical

Every structural part is printed in PETG, except the pan pinion (`CFPinion`) in nylon. The
flywheel window is laser-cut acrylic. Sources are in `Mechanical/CAD/`, print files in
`Mechanical/Gcode/final/`.

Gears: tilt 8T/40T, module 2, 5:1; pan planetary, module 2.5, 72T ring, 3 planets, 5:1; and the
shooter's rack and pinion.

| Qty | Part | Notes |
|---|---|---|
| 2 | 608 ball bearing | 22.3 mm OD × 7.1 mm ID as printed. [Amazon AU, ASIN B0G2BHHX5C](https://www.amazon.com.au/gp/product/B0G2BHHX5C/ref=ox_sc_act_title_6?smid=A102Q921QY46T4&psc=1). One each side of the tilt gear, on an 8 mm axle. |
| ? | M2 / M3 / M4 / M5 fasteners | Hole sizes and where each is used: `Mechanical/fitment.md`. |
