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

**Confirmed by the project owner directly (2026-11-13): every gear in this turret — the
tilt pair, the pan planetary, and the shooter's rack-and-pinion — was generated with
GearForge.** GearForge is a Fusion 360 add-in built alongside this project for generating
parametric ISO gear profiles (spur, helical, herringbone, internal, worm+wheel, rack)
instead of hand-drawing involute teeth. It's a separate project, not vendored into this
repo.

This corrects an earlier, more hedged version of this note. A chat-history handoff (see
`../../handoff-inbox/`) had searched a separate Claude.ai Project and come back with only
"probable, not confirmed" for the tilt pair, and "not GearForge" for the pan planetary and
the shooter rack — chat transcripts just hadn't captured what the owner actually did in
Fusion. Recorded here in case it's useful: that search also surfaced a different free
add-in, "GF Gear Generator," named in a related chat, in case "Gear Forge" ever meant that
tool instead in some other context — but for this repo's own gears, the owner's direct word
above is the answer.
