# Upstream

Pinned copy of https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32

- Version 1.1.3, commit 00234325be391543e7df914fc85a82bcb274c045 (2026-09-10)
- Licence: MIT (see LICENSE)
- Copied as plain files rather than a submodule because the fork carries small, marked
  patches to `src/XboxSeriesXControllerESP32_asukiaaa.hpp`. Every patch is wrapped in
  `// --- R4 fork patch N ... ---` / `// --- end R4 fork patch N ---` comments and listed
  below. Everything else is verbatim. To see the exact diff:
  `git diff b1e79fb -- firmware/libraries/XboxSeriesXControllerESP32_asukiaaa/src`

## Patches

1. `ClientCallbacks::onAuthenticationComplete`: null-check `NimBLEDevice::getClientByHandle()`
   before calling `disconnect()`. Upstream dereferences the pointer blindly; a null there would
   reboot the bridge and take the USB passthrough down with it.
2. `ScanCallbacks::clearTargetAddress`: set `targetDeviceAddress = nullptr` after `delete`.
   Upstream leaves a dangling pointer that the next `onResult()` dereferences.
3. `Core`: add `volatile uint32_t notificationCount` (incremented in `notifyCB` for HID
   reports; the bridge sends its low byte as the `seq` field), `bool
   deleteBondOnFailedConnection` (default `true` = upstream behaviour; the bridge sets it to
   `false` so a transient connect failure does not throw the bond away), and
   `connectionStateValue()` (the private `connectionState` as a number).

4. `ScanCallbacks::onResult`, pairing-mode branch: also require appearance 964 (gamepad), and
   the HID service UUID 0x1812 when the advert carries a service UUID at all. The manufacturer
   data `0600030080` the branch keys on is Microsoft's generic Swift Pair "pairing over LE"
   beacon, so without the appearance check any Swift Pair keyboard or mouse in pairing mode
   would be accepted. The UUID is optional because the Xbox One S pad (model 1708, firmware
   5.23.6.0) was observed on hardware advertising appearance 964 + the beacon with no service
   list in the advertising packet (it is in the scan response); upstream's default branch,
   which demands the UUID, never matches that pad with a passive scan.

5. `ScanCallbacks`: an optional `onAdvertHook` function pointer called at the top of
   `onResult()` with every advert, so the bridge can answer `AT+XBOXSCAN?` with what the scanner
   actually sees (a pairing diagnostic). Null by default, so behaviour is unchanged.

Nothing else changed. Reported upstream: not yet (the developer may want to open PRs for 1 and 2).
