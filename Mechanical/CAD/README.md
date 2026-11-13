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

**GearForge** is a Fusion 360 add-in we built alongside this project, for generating
parametric ISO gear profiles (spur, helical, herringbone, internal, worm+wheel, rack)
instead of hand-drawing involute teeth. It's a separate project, not vendored into this
repo.

Earlier revisions of this page claimed it was used for every gear part here. A 2026-11-13
handoff checked that against chat history and found it's weaker than that:

- **Tilt pair (8T/40T):** probable, not confirmed. Settings matching a "Gear Forge" add-in
  were requested and given (module 2, 20°, ISO 53, +0.6/−0.6 shift), but no chat confirms the
  gears were actually generated that way, and a *different* free add-in, "GF Gear Generator",
  was also named in a related chat with a dialog that matches just as well. "Gear Forge" in
  that history may mean either tool.
- **Pan planetary (5:1, 72T ring):** built with Fusion's own built-in Spur Gear tool plus a
  Combine-cut for the ring, per the instructions on record — not GearForge.
- **Rack-and-pinion shooter mechanism, `CFPinion`:** not mentioned in any chat found.

Treat "GearForge made this turret's gears" as unconfirmed, not as this repo's history.
