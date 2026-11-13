# Software

| Folder | Runs on | What it is |
|---|---|---|
| `Turret Code for the Arduino/` | RA4M1 | The sketch that runs the turret. |
| `Custom Firmware/` | ESP32-S3 | The forked USB-bridge firmware: Xbox BLE host and the BT/WiFi radio modes. |
| `Local Web Server for WiFi Connect Mode/` | PC | The relay and live dashboard for WiFi mode. |

## Architecture

- `architecture.md`: the build contract. Every module, the owner's overrides of
  `D-SENTRY_SPEC.md`, and the wire protocol. The code cites it by section number.
- `architecture-diagram.png`: the control flow, from boot through one loop tick.
- `firmware-architecture.html`: a later, interactive view built from the firmware source: the
  control-path block diagram, the `App::begin()`/`App::loop()` flowchart and the turret's state
  diagram. Open it in a browser.

The two were drawn at different times. Where they disagree, trust the source in
`Turret Code for the Arduino/dsentry-sketch/dsentry/src/`.
