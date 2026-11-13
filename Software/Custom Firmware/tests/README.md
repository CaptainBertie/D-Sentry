<!-- SPDX-License-Identifier: MIT -->
# Host tests

These tests run on the Mac with `clang++ -std=gnu++17`. They need no board, no serial port and
no arduino-cli. They compile the real code that sits on each end of the AT link, so a pass
means the real parsers and the real RA4M1 library accept the bytes the protocol defines.

## Run

```
./tests/run.sh        # verdicts, info lines and failures
./tests/run.sh -v     # also every passing check
```

The script prints one `PASS`, `FAIL` or `SKIP` line per test and a summary. The exit status is 0
when nothing failed, 1 if any test fails, and 2 for an unknown argument. Build output and logs
go to `tests/build/` (git-ignored); set `TESTS_BUILD_DIR` to use another folder, for example
when two runs happen at the same time.

Requirements:

- `clang++` (Xcode command line tools). Set `CXX` to use another compiler.
- The Renesas core `arduino:renesas_uno` **1.6.0**, installed in the usual place
  (`~/Library/Arduino15/packages/arduino/hardware/renesas_uno/1.6.0`). Three tests compile
  its `libraries/WiFiS3/src/Modem.cpp`. If the core is missing, those tests fail with an
  install hint. To test another copy, set `WIFIS3_SRC` to its `WiFiS3/src` folder. Other core
  versions are not picked up automatically, because the tests pin the 1.6.0 behaviour.

## What each test proves

| Test | Code under test | What a pass means |
|---|---|---|
| `test_protocol.cpp` | `protocol/xbox_r4_protocol.h` (canonical) | The CRC-8 gives the check value `0xF4` for `123456789`. Encode then decode returns every field, for 5 fixed and 200 random states. Decode rejects lengths 0, 1, 15, 16, 19 and 21. Decode rejects every single-bit flip in all 20 bytes (160 cases) and leaves the output untouched. The neutral report has sticks 32767 and nothing pressed. The constants and the byte offsets match the table in `docs/protocol.md`; the test also checks that the table rows are still in the doc, with the full d-pad map, state list and battery range. |
| `test_parser_replay.cpp` | `firmware/libraries/XboxControllerNotificationParser/src/*.cpp` (asukiaaa 1.0.5) | Each documented button bit sets only its own flag, and undocumented bits set none. D-pad values 0..8 give the direction flags in `docs/protocol.md`; values 9..255 give none. Stick values 0, 32767 and 65535 and triggers 0 and 1023 decode little-endian at the documented offsets. A 19-byte Elite report is accepted and parsed from its first 16 bytes. A 15-byte report is rejected with `XBOX_CONTROLLER_ERROR_INVALID_LENGTH` and changes nothing. `update()` then `toArr()` returns the same 16 bytes for every d-pad value and for 200 random reports inside the documented domain. |
| `test_modem_e2e.cpp` | RA4M1 `WiFiS3/src/Modem.cpp` from the installed core, with the stub UART | The RA4M1 sends exactly `41 54 2B 58 42 4F 58 53 54 41 54 45 3F 0D 0A`. The 41-byte reply is read back byte-identical, including CR, LF, `\|`, `OK`, NUL and 0xFF in the payload, and so are 200 random payloads. The `+HCIREAD` form (no CRLF before `OK`) also works. A bare `OK`, a bare `ERROR` and a prompt followed by `ERROR` each return false only when the 50 ms timeout expires. The test makes the same modem calls as the library's `poll()` and applies a copy of its acceptance rule (write() true, 20 bytes, CRC good) to stale, truncated, wrong-size and bad-CRC replies; see below. |
| `test_ra_library.cpp` | `ra4m1/libraries/XboxControllerR4/src/XboxControllerR4.cpp` and its vendored parser, linked with the real `Modem.cpp` | The library's own `poll()` accepts the documented reply and decodes every field; an unchanged `seq` is not a new report and a new one is. A bare `ERROR` (what a stock bridge sends) and a prompt then `ERROR` fail at the 50 ms timeout. After a good poll and after a timeout, the modem timeout is back to `MODEM_TIMEOUT`. A stale reply in rx is drained; a late reply after the drain is returned one poll late, never older. A truncated payload, a stale sized reply from another command, `0\|`, `19\|`, `21\|` and a bad CRC are each rejected. Every rejected poll adds one to `pollErrors()` and leaves `seq()`, `connectionState()`, `battery()`, `hasNewReport()` and the fields unchanged. Dropping to scanning loads the neutral report and sets `hasNewReport()`; reconnecting counts as a new report. |
| `test_chat_e2e.cpp` | chAT `parser.cpp` and `server.cpp` from `firmware/UNOR4USBBridge`, linked with the real `Modem.cpp` | A copy of the firmware's `+XBOXSTATE` handler makes chAT send exactly the 41 bytes in `docs/protocol.md`, also when the request arrives one byte at a time. Those bytes, passed through the stub UART into the real `Modem.cpp`, decode to the published state, for 1 + 64 polls. `AT+XBOXSTATE` (Run), `AT+XBOXSTATE=?` and `AT+NOPE?` get `ERROR\r\n`, and so does `AT+XBOXSTATE?` on a stock bridge. The RA4M1 sees that `ERROR` only as the 50 ms timeout. The real handler cannot be built on the host, so the test reads `cmds_xbox.h` and fails if the `srv` calls in its Read branch differ from the ones the copy makes. |
| `test_protocol_copies.sh` | the files kept in two places | The firmware and RA4M1 library copies of the header are byte-identical to `protocol/xbox_r4_protocol.h`. The four parser files vendored into `ra4m1/libraries/XboxControllerR4/src/vendor/` are byte-identical to `firmware/libraries/XboxControllerNotificationParser/src/`, the copy `test_parser_replay` tests. A file that does not exist yet is reported as `SKIP`; a file that differs is `FAIL`, with the start of the diff. If every file is missing, the test prints only the `SKIP` lines and does not count as passed. |

## Behaviour the tests record (printed as `info` lines)

- **Stale reply.** If the late reply to a timed-out poll is still in `Serial2`, the next
  `modem.write()` returns that stale frame, because the prompt is the same. Every later poll then
  lags by one. Draining `Serial2` before each poll fixes this.
- **Late reply after the drain.** A whole late reply that lands after the drain is returned one
  poll late. It is never older than the last accepted frame: the UART delivers bytes in order
  and chAT answers requests in order. The next drained poll catches up, and `seq` flags a repeat.
- **Tail of a late reply after the drain.** At 115200 baud the drain usually stops part-way
  through a frame that is still arriving. Over every split point of 16 stale frames, the poll
  either returned the fresh frame or failed; none accepted another frame. A failed poll costs
  the full 50 ms timeout.
- **Truncated sized reply.** With `20|` and only 15 bytes, `write()` returns false at the
  timeout. With 15 bytes followed by the normal `\r\nOK\r\n`, the sized read takes `\r\nOK\r` as
  payload, so `res.size()` is 20 but `write()` returns false and the CRC fails. The library must
  check the return value, the size **and** the CRC.
- **Counter carried over.** A stale sized reply to another command (for example
  `+PREFGET: 5|abcde`) leaves `Modem.cpp`'s sized-read counter at 5, so the next `+XBOXSTATE`
  read stops 5 bytes early. `write()` then returns either true with 15 bytes, or false at the
  timeout, depending on the last payload bytes (an `O` there sends the parser into its error
  path). The size check rejects the first, the result check the second, and draining first
  avoids both.
- **Longer stale sized reply.** After `+PREFGET: 30|<30 bytes>`, the counter is already past 20
  and never equals 20 again, so an undrained `+XBOXSTATE` read runs to the timeout.
- **Wrong sizes.** `Modem.cpp` copies whatever size is announced and returns true for `0|`,
  `19|`, `21|` and a 20-byte payload with a bad CRC. Only the library's size and CRC check
  rejects them.
- **Bare `ERROR` has two causes.** `Modem.cpp:7` defines `RESULT_ERROR` as `"OK\r\n"`. But a
  bare result code with no prompt also sets `restart` (`Modem.cpp:243-255`). So even with line 7
  corrected, a bare `ERROR\r\n` (which is what a stock bridge sends) would still only end at the
  timeout. Only the prompt-then-`ERROR` form depends on line 7 alone.
- **`toArr()` is lossy outside the documented domain.** D-pad values 9..255 become 0. Byte 13
  keeps mask `0xDB`, byte 14 keeps `0x7C` and byte 15 keeps `0x01`. Bytes 0-11 always survive,
  and triggers are not clamped to 1023.
- **Elite 2 parser quirk.** `XboxControllerElite2NotificationParser::update()` returns 0 for a
  15-byte report instead of the length error. The bridge uses the base parser.

## Files

- `stubs/Arduino.h`: the host stand-in for the Arduino core: `millis()`, `Stream`, a `UART`
  with an rx queue, a tx capture, an `on_write` hook and a delayed feed, and a small `String`.
  It is adapted from the recon harness `modemsim/stub/Arduino.h`.
- `stubs/WiFiS3.h`: stands in for the WiFiS3 library header when the RA4M1 library is built on
  the host. It includes only the real `Modem.h`; `run.sh` puts `stubs/` first on the include
  path.
- The test sources started from the recon harnesses `modemsim/sim.cpp`, `modemsim/e2e_ra.cpp`,
  `chatsim/sim.cpp` and `chatsim/e2e_esp.cpp`. Nothing at run time depends on the recon
  scratchpad.

## Licences

The test files are MIT. They compile third-party code in place, without copying it:
`Modem.cpp` (LGPL-2.1, from the installed core), chAT (AGPL-3.0-or-later, from
`firmware/UNOR4USBBridge`) and the notification parser (MIT). They also compile this project's
RA4M1 library (MIT) in place. The test binaries are built locally and never distributed.
