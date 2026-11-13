# CAD

## What's here

- **`vendor-parts/`** — reference models of purchased components, used for fit-checking the
  turret design around them: the 6S battery pack, the 9 g servo, the 1602 LCD, the L298N
  driver board, the CNC Shield V3, and the control-panel icon set.
- **`meshes/`** — the turret's own parts, but only as sliceable meshes (STL/3MF) and one DXF
  profile. These are print-ready, not editable.
- **`nameplate/`** — the D-Sentry nameplate artwork (SVG + preview render).

## What's missing

The turret's own parametric design — the actual frame, gears, enclosure, and mount source
files — lives in Fusion 360, not in this folder. I don't have a way to pull files out of
Fusion 360's cloud project on your behalf, so `meshes/` only has the exported STL/3MF output,
which you can 3D print from but can't edit.

To make this folder complete: export the native design (F3D, or STEP if you want something
tool-agnostic) from Fusion 360 and drop it in a `source/` folder here.

## GearForge

The gear parts in this project — the planetary pan reduction, the tilt spur gear, the
rack-and-pinion shooter mechanism, the nylon pan pinion (`CFPinion`) — were modeled with
**GearForge**, a Fusion 360 add-in we built alongside D-Sentry for generating parametric ISO
gear profiles (spur, helical, herringbone, internal, worm+wheel, rack) instead of hand-drawing
involute teeth. It's a separate project, not vendored into this repo.
