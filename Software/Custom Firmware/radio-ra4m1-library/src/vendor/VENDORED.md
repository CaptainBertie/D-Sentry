# Vendored code

## XboxControllerNotificationParser

| | |
|---|---|
| Upstream | https://github.com/asukiaaa/arduino-XboxControllerNotificationParser |
| Version | 1.0.5 (git tag `1.0.5`) |
| Commit | 77a449d77970166a01c25ef7c26c084a42c3a5d6 |
| Licence | MIT, Copyright (c) 2022 Asuki Kono (`LICENSE-XboxControllerNotificationParser.txt`) |
| Copied from | `firmware/libraries/XboxControllerNotificationParser/src/` in this project (the same pinned commit the ESP32 firmware uses) |

Files, copied byte for byte with no edits:

- `XboxControllerNotificationParser.h`
- `XboxControllerNotificationParser.cpp`
- `XboxControllerElite2NotificationParser.h`
- `XboxControllerElite2NotificationParser.cpp`

Why: copied so the library installs as one folder. The RA4M1 decodes bytes 0..15 of
`AT+XBOXSTATE?` with the same parser the ESP32 uses to encode them (`toArr()`), so both
ends agree on the field names and bit positions by construction.

Include paths: no change was needed. Each `.cpp` includes its own header by quotes from
the same folder, and the base header includes `<Arduino.h>`. `XboxControllerR4.h` includes
`"vendor/XboxControllerNotificationParser.h"`.

The Elite 2 parser is not used by `XboxControllerR4`. It is kept so the vendored folder is
the complete upstream `src/`; the linker drops it unless a sketch uses it.

<!-- SPDX-License-Identifier: MIT -->
