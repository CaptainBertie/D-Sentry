# Handoff inbox

A staging area for material pulled in from elsewhere — right now, a separate Claude.ai
Project that has more context on this build than made it into local files.

## How material gets in here

Claude.ai Artifacts are the bridge: they're visible across every session on the same
account, including this one. The process is:

1. In the other Claude session, ask it to publish its answer as an Artifact titled starting
   with **`D-Sentry Handoff:`** — a text/Markdown writeup for notes, or an HTML page (or an
   uploaded asset) for anything visual, like a wiring diagram.
2. Back here, list this account's artifacts and look for that title prefix, then read the
   matching one(s) in full — including, where an artifact has been republished more than
   once, its earlier versions. That's exactly how a diagram iterated as an Artifact over
   several sessions gets its real version history back, rather than only the latest snapshot.
3. Each handoff gets its own entry below and its own commit when folded into the real repo
   location (`Mechanical/Fitment/`, `Hardware/BOM/`, `DECISIONS.md`, `Hardware/Wiring
   Diagram/`, etc.) — never a silent overwrite.

## Log

| Date | What came in | Folded into |
|---|---|---|
| 2026-11-13 | [D-Sentry Handoff: BOM and Sources](https://claude.ai/artifact/8HwwP86NgFGLKoiiMxhWYn) | `Hardware/BOM/BOM.md` |
| 2026-11-13 | [D-Sentry Handoff: Mechanical Findings](https://claude.ai/artifact/Dep7RSri2VmU5ZspZRsshG) (fitment, rack iteration, GearForge) | `Mechanical/Fitment/README.md`, `Mechanical/Gcode/README.md`, `Mechanical/CAD/README.md` |
| 2026-11-13 | [D-Sentry Handoff: Hardware Decisions](https://claude.ai/artifact/EzHPWaFktqU3dXSaSUNg28) (UBEC, flywheel driver history, gear ratio label mismatch, open items, wiring-diagram provenance) | `DECISIONS.md` D20-D25, `Hardware/Wiring Diagram/CHANGELOG.md` |
| 2026-11-13 | [D-Sentry Architecture](https://claude.ai/artifact/VsKxo5CPpywQSYmbRMB9uv) — a pre-existing artifact surfaced while searching, not a handoff writeup | `Software/System Block Diagram/firmware-architecture.html` |

None of the three handoff artifacts have been revised more than once (each is "v1 only" per
the session that made them), so there was no multi-version history to pull for those. The
wiring diagrams specifically turned out to have real prior versions, just not ones
reachable this way — see `../Hardware/Wiring Diagram/CHANGELOG.md` for what's known about
them.
