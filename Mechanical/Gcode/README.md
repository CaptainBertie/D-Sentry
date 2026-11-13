# Gcode

Sliced on a Prusa MK3S, PETG (0.6 mm nozzle, 0.35 mm layers unless noted), nylon for the pan
pinion gear.

- **`final/`** — gcode for parts that match the turret's named structural components: `Base`,
  `Foot`, `Frame`, `Gears`, `LeftLid`, `LeftMount`, `RightMount`, `RLid+BC`, `Shooter`,
  `ShooterBase`, `ShooterPinion`, `TiltGear`, `TiltPinion`, `CFPinion`, `LCD+NEMA`.
- **`testing/`** — fit and calibration prints: bearing/bolt/bolt-OD/nut/pin fit tests, magnet
  tests, switch and mount-width tests, and six iterations of a gear rack (`Rack` through
  `Rack6`).

**One open question:** no file in this project records which rack iteration (`Rack` –
`Rack6`) ended up on the built turret, if any did — they're all filed under `testing/`. There
are also two `Frame` gcode files with very different print times (27 minutes vs. 18 hours 23
minutes); both are kept under `final/` rather than guessing which one is current. If you know
the answers, move the right files and delete the rest.
