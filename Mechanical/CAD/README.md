# CAD

- **`source/`**: the native Fusion 360 design. `Turret.f3z` is the main assembly (frame,
  mounts, enclosure); `Shooter.f3z` the dart pusher; `Flywheel cage.f3d`; and `Fit Panel.f3d`
  the control panel, using the icons in `vendor-parts/panel-icons/`. An `.f3z` is a zipped
  assembly with every design it links to; an `.f3d` is one design. Open either in Fusion with
  **File → Open**.
- **`meshes/`**: printable STL/3MF exports of `source/`, plus one DXF profile, so you can print
  without Fusion.
- **`vendor-parts/`**: models of the bought parts, for fit checks: the 6S battery, the 9 g
  servo, the 1602 LCD, the L298N, the CNC Shield V3, the NEMA17 and the panel icons.
- **`nameplate/`**: the D-Sentry nameplate (SVG and preview).

## GearForge

We generated every gear (the tilt pair, the pan planetary and the shooter's rack and pinion)
with GearForge, a Fusion 360 add-in we built alongside this project. It draws parametric ISO
spur, helical, herringbone, internal, worm and rack profiles, so nobody has to draw involute
teeth by hand. It is a separate project and does not live in this repo.
