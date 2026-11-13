# Fitment

This folder is for the measured values from the fit-test prints in `../Gcode/testing/`
(bearing bore, bolt/nut fit, pin diameter, magnet pocket size, mount width, switch cutout).

**Confirmed empty, not just missing locally.** A 2026-11-13 handoff searched a separate
Claude.ai Project's full chat history for these twelve test names (`BearingFitment`,
`BoltFit`, `BBFit`, `BBOD`, `NutTest`, `PinTest`, `PinTestV2`, `MagnetTest`, `MagTest`,
`MagTestV2`, `MountWidthTest`, `SwitchTest`) and found no mention of any of them, and no
measured fit value anywhere. Whatever was learned from printing these parts was never
written down. A plan existed — print a clearance coupon at 0.10 / 0.15 / 0.20 / 0.25 /
0.30 mm per side and record which one fit — but no result from running it was ever reported
back either.

## What exists instead (generic guidance, not a measurement — don't cite it as one)

For a Prusa MK3 at a 0.4 mm nozzle:

| Fit | Clearance per side |
|---|---|
| Press | 0 to −0.1 mm |
| Snug / slide | 0.1-0.15 mm |
| Sliding | 0.2-0.3 mm |
| Loose | 0.3-0.5 mm |

Printer accuracy is roughly ±0.1-0.2 mm; holes tend to print 0.1-0.2 mm undersized, so a
nominal 3 mm hole can come out 0.1-0.2 mm tight.

Design intent values that were stated but never measured against a printed part: gear
backlash 0.15 mm; tilt gear centre distance +0.15-0.2 mm over nominal; 608 bearing (22 mm
OD × 7 mm wide) on an 8 mm axle; 5 mm motor shaft bore.

## Fill this in

| Test print | Nominal (CAD) | Printed result | Fits? | Notes |
|---|---|---|---|---|
| BearingFitment | | | | |
| BoltFit | | | | |
| BBFit / BBOD | | | | |
| NutTest | | | | |
| PinTest / PinTestV2 | | | | |
| MagnetTest / MagTest / MagTestV2 | | | | |
| MountWidthTest | | | | |
| SwitchTest | | | | |
