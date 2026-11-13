# Fitment

Measured values from the fit-test prints in `../Gcode/testing/`, recovered from a Notion
workspace ("Meche A4") on 2026-11-13 — a separate source from the chat-history handoff
logged in `../../handoff-inbox/`, which had searched chat transcripts and found nothing
because these were written down in Notion instead.

## Fastener fits (confirmed, printed and measured)

| Fastener | Clearance | Self-tap hole | Counterbore |
|---|---|---|---|
| M2 | 2.7 mm (not yet confirmed printed) | not measured | not measured |
| M3 | 3.6 mm | 3.1 mm | 6.2 mm |
| M4 | 4.5 mm | 4.1 mm | 8.1 mm |
| M5 | not measured | — | — |

Same values for both vertical and horizontal holes (printed and checked both ways).

## Other confirmed measurements

| Fit | Measured value | Notes |
|---|---|---|
| Bearing (608) | 22.3 mm OD, 7.1 mm ID | Nominal 608 bearing is 22 mm × 7 mm — the print runs about 0.3 mm over on OD. |
| BB cartridge | −0.3 mm, bottom removed, 0.6 mm stickout | |
| Foot well | 21 mm | |
| Window width | 3.175 mm | These are **laser cut acrylic**, not 3D printed — see `../../README.md`'s build order. |
| LCD pocket | −0.1 mm (press fit) | From the pre-frame checklist, confirmed on the printed frame. |
| NEMA D-shaft bore | 5.20 mm Ø, 4.7 mm D-width | Logged but not yet checked off as confirmed — treat as the design target, not a verified fit. |

## What's tracked but has no number recorded anywhere found

A Notion "Fitment Tracker" database has 33 entries (bolts, nuts, bearings, electronics
mounts, gearing, general fits) with a status column, but most individual entries are blank
templates — the actual numbers above were kept in one consolidated note instead. Tracked
as "recorded" with no value found in either place: **Planetary Gears** backlash, **NEMA D
Shaft Profile** (beyond the two numbers above), **Base Alignment Pins**, **NEMA Mount**,
**LCD Mount** (beyond the −0.1 mm above).

Tracked and explicitly **not yet tested** as of the last update (2026-11-02): rack-and-pinion
backlash and slider fit, tilt gear backlash and mount profile, magnet pocket (shooter),
mag well/mag release fit, servo mount, flywheel cage mount, switch mounts (left lid),
wiring mounts on the shooter, mount-to-frame fit (both left and right mount).

## Fastener list, by where each size is used (from the pre-print dependency checklist)

- **M4**: frame (vertical self-tap, vertical+horizontal clearance, vertical push-in nut),
  left/right mount (horizontal clearance, horizontal push-in nut), left/right lid (vertical
  clearance), bottom panel (vertical clearance), base (vertical clearance).
- **M3**: frame (horizontal self-tap), left mount (vertical self-tap).
- **M2**: frame (vertical self-tap).
- **M5**: shooter (clearance).

Nothing beyond fastener size was recorded (no head style, length, or purchase source for
fasteners specifically — see `../../Hardware/BOM/BOM.md`).
