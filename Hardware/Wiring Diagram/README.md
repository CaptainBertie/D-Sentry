# Wiring Diagram

`turret-wiring.png` is the current wiring: RA4M1 pin map, both steppers, the trigger servo,
the LED ring, the LCD, both toggles, the 6S LiPo and 9V supplies, the 5V UBEC rail, and the
L298N flywheel driver.

Pin assignments should match `../../D-SENTRY_SPEC.md`'s "Final pin map".

## Version history

`versions/` holds every earlier revision, v00 through v08 — the flywheel driver moving from
a MOSFET switch to an L298N board, the L298N channel changing twice, a hall-effect tilt
limit added and later removed, and the 5V rail's evolution from the board's own regulator
to a dedicated UBEC. `CHANGELOG.md` has the detail on each one.
