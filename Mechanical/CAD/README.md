# CAD

## What's here

- **`source/`** — the turret's own native Fusion 360 design: `Turret.f3z` (the main
  assembly — frame, mounts, enclosure), `Shooter.f3z` (the dart-pusher subassembly),
  `Flywheel cage.f3d`, and `Fit Panel.f3d` (the control panel: switch cutouts + the icon
  set in `vendor-parts/panel-icons/`). `.f3z` is a Fusion Archive (an assembly plus every
  document it links to, zipped); `.f3d` is a single design. Both open directly in Fusion
  360 — **File → Open**, or drag onto the Fusion window.
- **`vendor-parts/`** — reference models of purchased components, used for fit-checking the
  turret design around them: the 6S battery pack, the 9 g servo, the 1602 LCD, the L298N
  driver board, the CNC Shield V3, the NEMA17 stepper, and the control-panel icon set.
- **`meshes/`** — the turret's own parts as sliceable meshes (STL/3MF) and one DXF profile —
  derived from `source/`, kept so anyone can print from this repo without owning Fusion.
- **`nameplate/`** — the D-Sentry nameplate artwork (SVG + preview render).

## GearForge

The gear parts in this project — the planetary pan reduction, the tilt spur gear, the
rack-and-pinion shooter mechanism, the nylon pan pinion (`CFPinion`) — were modeled with
**GearForge**, a Fusion 360 add-in we built alongside D-Sentry for generating parametric ISO
gear profiles (spur, helical, herringbone, internal, worm+wheel, rack) instead of hand-drawing
involute teeth. It's a separate project, not vendored into this repo.
