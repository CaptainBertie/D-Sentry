# System Block Diagram

- `architecture.md` — the original build contract: every module, every decision the owner
  made that overrides `D-SENTRY_SPEC.md`, the full folder layout. Dense, text-first.
- `architecture-diagram.png` / `.svg` — the diagram referenced by `architecture.md`.
- `firmware-architecture.html` — a second, later pass at the same system: an interactive
  page with the control-path block diagram, the `App::begin()`/`App::loop()` flowchart, and
  the turret's state diagram, built straight from the firmware source. Open it in a browser.

Both diagrams describe the same system from slightly different angles and were built at
different points — if they ever disagree, check the firmware source
(`../Turret Code for the Arduino/dsentry-sketch/dsentry/src/`) directly rather than assuming
either one.
