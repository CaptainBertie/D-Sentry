# Licensing notice

This repository mixes two different licenses because it builds on top of someone else's
firmware fork. Read this before you reuse, flash, or redistribute anything here.

## The short version

| Part | License | Why |
|---|---|---|
| Mechanical design, wiring, the RA4M1 turret sketch, the PC relay/dashboard, D-Sentry's own protocol code | **MIT** | Original work for this project. |
| `Software/Custom Firmware/bridge-fork/` — the ESP32-S3 firmware, as a whole compiled binary | **AGPL-3.0** | It links SudoMaker's `chAT` AT-command server, which is AGPL-3.0-or-later. Linking AGPL code into a binary makes the whole binary AGPL-3.0, even though most of the surrounding code is MIT/Apache/BSD. |

**What this means in practice:** you can build and flash this firmware for your own turret
with no obligation at all. If you give the compiled firmware image to anyone else — even one
other person — the AGPL requires you to also offer them the complete corresponding source,
which is exactly what this repository already does (the fork's source sits right next to its
prebuilt images in `Software/Custom Firmware/artifacts/`).

The RA4M1-side code (the turret sketch, the `XboxControllerR4` library) is a separate program
that never links `chAT` and is MIT, independent of the ESP32 binary's license.

## The full breakdown

`THIRD_PARTY.md` lists every third-party component in this repo, its exact version, and its
license, read from each component's own LICENSE file or source headers. The reasoning behind
the AGPL determination is in `DECISIONS.md`, entry D04.
