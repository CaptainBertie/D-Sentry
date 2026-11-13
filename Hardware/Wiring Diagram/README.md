# Wiring Diagram

- `turret-wiring.svg` / `.png` — the full turret wiring: RA4M1 pin map, steppers, LCD, LEDs,
  switches, servo.
- `turret-wiring-l298n.png` — the flywheel driver sub-circuit (L298N channel A, the two
  flywheel motors, the 6S rail).

Pin assignments these diagrams should match: `../../D-SENTRY_SPEC.md` §"Final pin map".

## Version history

These diagrams don't have any history from before this repo — they were never committed
anywhere until now, so there's nothing earlier to show. From here on, every revision gets its
own commit (never overwrite-and-amend), which means two ways to see what changed:

- On GitHub, open the file and click **History** to see every past version and diff.
- `CHANGELOG.md` in this folder is the human-readable summary: what changed and why, in
  plain words, one entry per revision.

If an updated diagram comes in later (including from the project handoff — see
`../../handoff-inbox/README.md`), add it as its own commit and log it in `CHANGELOG.md`
rather than replacing the file silently.
