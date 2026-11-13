<!-- SPDX-License-Identifier: MIT -->
# Bench checklist

Nothing in D-Sentry has run on the turret yet. This page is the first real run, in an order where
each step adds one source of power or motion, so that a mistake costs as little as possible.

- **Part A**: first power-up, step by step, with what you should see and what to do if you do not.
- **Part B**: spec section 11, items 1-20, as a checklist for the real turret, with what the Mac
  tests already prove for each.
- **Part C**: the checks only hardware can answer (timing, reset behaviour, the radio).

Bring: a multimeter, tape flags for the barrel and the pan direction, a phone that films slow
motion, and a 10 kΩ potentiometer (Part B item 15). The bench console and the dashboard are
described in `tuning.md`, "How you see what the turret is doing". Commands below written
`!status` are shell lines typed into the bench console; `tap a`, `stick 0.5 0`, `trigger 0.7`
are bench-console commands.

**Three rules for the whole session**

1. **The power toggle (A0) is the stop switch.** Open = OFF: the stepper drivers release, the
   flywheels stop, the servo goes limp. Keep a hand near it whenever something can move.
2. **Never plug or unplug a stepper motor while its driver is powered** (LiPo connected). That can
   destroy the driver.
3. **No darts until Part B item 9.**

## Part A. First power-up

### A0. Before any power

- [ ] Microstep jumpers on both driver sockets: first two closed, third open (1/8, matches
      `MICROSTEPS 8`).
- [ ] Each driver sits the right way round: its EN pin on the shield's EN mark. A driver plugged
      in backwards burns out at power-up.
- [ ] The L298N module's **ENA jumper is off** and the wire from D10 sits on the ENA pin. The
      jumper ties ENA to the module's 5 V: with it on, the wheels run at full pack voltage
      whatever the duty, and D10 fights the 5 V rail every time it drives low.
- [ ] The L298N module: most modules say to remove the "5V EN" jumper when the motor supply is
      above 12 V, because their 5 V regulator is not meant for more; a 6S pack is 22-25 V. Check
      your module's notes and its 5 V logic supply.
- [ ] Power toggle (A0) **open**, mode toggle (A1) **open** (BT).
- [ ] LiPo disconnected. Stepper motors unplugged. No darts.

### A1. Board on USB only

1. Plug the board into the Mac. `"$PY" tools/r4flash.py ports` must list `2341:1002 bridge mode`.
   - Not listed: try another cable (some carry power only).
2. If the servo is powered from the board's 5 V, unplug its lead for this step: a servo starting
   up can pull a USB port's voltage down and reset the board.
3. Upload the sketch (`../README.md`, "Upload the sketch"). Then start the bench console and press
   the board's RESET button once.
   - Expect: `D-Sentry RA4M1 boot: pan timer 1 tilt timer 1 servo 1 lcd 1 0x27`, a line naming
     the cut LCD text and `warning: 1 LCD text combination(s) will be cut` (the WiFi pairing
     row, `tuning.md` Part 2 item 2), and `state OFF`. LCD: `SYSTEM OFF`, backlight off. LEDs
     dark.
   - `lcd 0` and `warning: no LCD`: run the I2C scanner (`tuning.md` Part 1 step 3).
   - `pan timer 0` or `tilt timer 0`: a hardware timer was not claimed. Report it with the
     whole boot text; that axis cannot move.
   - No text at all: check the console found the port (`# ` lines); close any other program
     holding it.
4. Close the power toggle.
   - Expect: `state BOOT`, `state CALIBRATING` (LEDs pulse white, LCD `Calibrating`). With no
     motors connected, calibration still counts its steps and finishes in about 10 s:
     `calibration done`, `state PAIRING`, then `state DISARMED` at once (the console sends
     connected packets). LEDs solid green, LCD `DISARMED AMMO:15` / `BT Connected`.
   - LEDs show the wrong colour (red instead of green): `LED_PIXEL_ORDER NEO_RGB`.
5. `!status` must show `state=DISARMED`, `power=on`. Open the toggle again: `state OFF`.

### A2. Add the 9 V battery

1. Toggle open. Plug the 9 V battery into the barrel jack.
   - Expect: nothing changes; the board keeps running. The 5 V pin reads about 5 V.
2. Plug the servo lead in. Close the toggle: the servo moves to its rest angle and holds. Open it:
   the servo goes limp.
   - The board resets when the servo moves (the uptime in `!status` replies starts again): the
     5 V rail cannot feed the servo. Give the servo its own 5-6 V supply with a shared GND.

### A3. Add the LiPo, steppers still unplugged

1. Toggle open. Connect the LiPo to the shield's terminal block and to the L298N.
   - Expect: shield terminal and L298N supply terminal read the pack voltage (22.2 V nominal,
     25.2 V full). The L298N outputs (OUT1/OUT2) read 0 V.
   - After a minute, nothing is warm (drivers, L298N, regulator).
2. First flywheel spin, **no darts**: close the toggle, wait for DISARMED, then `!arm`, `!fly on`.
   - Expect: the wheels ramp up over 0.4 s. Measure the motor voltage (`tuning.md` Part 1 step 9;
     about 9 V at nominal pack voltage, about 10.4 V on a full pack with the default duty 113).
   - Wheels spin backwards: `FLYWHEEL_FORWARD_IN1_HIGH false`.
   - `fly on: refused: not in this state`: you are not ARMED (`!status`).
3. `!fly off`, `!disarm`, toggle open, LiPo disconnected.

### A4. Plug the steppers in and set their current

1. LiPo disconnected, toggle open. Plug the pan motor into X and the tilt motor into Y.
2. Set Vref on both drivers with the hold sketch (`tuning.md` Part 1 step 4), then disconnect the
   LiPo and upload D-Sentry again.

### A5. First motion: the tilt calibration

1. Put the barrel roughly level. Bench console open. LiPo connected. Hand on the power toggle.
2. Close the toggle.
   - Expect: the barrel drives **down** at 10°/s, meets the bottom panel, the motor buzzes
     softly while it skips steps for the rest of the 90° travel, it stops, lifts 3°, and the log
     says `calibration done: tilt 0 = bottom stop + TILT_CAL_BACKOFF_DEG`. About 10 s.
   - Barrel goes **up**: toggle open at once, `TILT_DIR_INVERT true`, upload, repeat.
   - The turret **pans** instead: toggle open at once; X and Y are swapped (`tuning.md` Part 1
     step 2).
   - Nothing moves: `!status` must show `drv=1`. Check Vref is not near 0 and the EN jumper.
   - A loud knock at the panel: lower `TILT_CAL_SPEED_DEG_PER_S` or Vref.
3. Pan direction, then microsteps: `stick 0.3 0` must turn the turret clockwise seen from above
   (`tuning.md` Part 1 step 5); `!pan 90` must turn exactly a quarter turn.
4. Set the limits (`tuning.md` Part 1 step 6) before driving at speed.

### A6. The ESP32 image, then the pad

Do this once the USB bench steps pass, before the radio items in Part B.

1. Flash the D-Sentry ESP32 image: `flashing.md`. It makes a full backup first and refuses to
   flash without one. Then its step 5: Blink must still upload.
2. Upload D-Sentry again. `!status` must show `esp=alive` (not `legacy`) and `espMode=BLE` with
   the A1 toggle open.
3. Pair the pad: hold its Pair button for 3 s until the Xbox logo flashes fast. Start the relay
   with `--monitor` (read-only, so it does not override the pad). LCD: `BT Connected`.
   - The pad never pairs: it must be a Bluetooth Low Energy model (Series X|S model 1914, or a
     1708 on firmware 5.x). `../../DECISIONS.md` D18 describes the scan diagnostic.

## Part B. Spec section 11, on the real turret

"Mac:" names the host test that already proves the logic: `tests/unit/test_turretstate.cpp` runs
every RA4M1 module together on a fake board (`tests/run.sh`). The Mac cannot prove anything about
motors, power, real timing or the radio; those are what the bench is for. Over USB means the
bench console or the relay on the USB cable; *(radio)* items need Part A6.

- [ ] **1. Power toggle.** Open: LCD `SYSTEM OFF`, backlight off, LEDs dark, motors limp (pan
  turns by hand), wheels stop, servo limp; `rx=` in the stats line keeps counting (packets are
  still read). Close: `state BOOT`, `CALIBRATING`, `PAIRING` again.
  Mac: `accept_01_power_off_idles_everything_and_power_on_reruns_boot`.
- [ ] **2. Boot.** LEDs pulse white; tilt drives down, stops at the panel, lifts 3°; dashboard
  *Motion* "Tilt" reads 0.0 and `!status` shows `cal=1`; LCD `Calibrating`, then the pairing row.
  Mac: `accept_02_boot_pulses_white_and_calibrates_against_the_panel`.
- [ ] **3. Pairing.** Start the bench console with `--disconnected` (A1 open), then open and close
  the power toggle, so PAIRING starts fresh from a boot. LEDs pulse blue and white; LCD row 1
  `BT Pairing` with dots stepping every 0.4 s (`.`, `. .`, `. . .`, none), for as long as no
  packet says connected; then `connect`: green, `Connected`, `state DISARMED`.
  Mac: `accept_03_pairing_pulses_blue_white_with_dots_then_connects`.
- [ ] **4. Stick.** `stick 0.25 0`, `stick 0.5 0`, `stick 1 0`: dashboard *Motion* velocity grows
  with deflection along the expo curve (at half stick about 26°/s); `stick 0.1 0` does not move
  (deadzone); full stick never exceeds 90°/s pan and 60°/s tilt; hold `stick 0 1`: the velocity
  falls before the top and tilt stops at `TILT_RANGE_DEG` (60.0), never past it.
  Mac: `accept_04_stick_speed_follows_the_curve_and_respects_limits`.
- [ ] **5. Home.** Move somewhere, `tap left` (log `home set: pan ... tilt ...`), move away,
  `tap down` (`going home`): it returns at 45°/s and stops on the stored spot. Again, and
  `stick 0.3 0` halfway: the move stops (dashboard *State* "Homing" clears).
  Mac: `accept_05_home_set_go_home_and_stick_cancels`.
- [ ] **6. Arm.** `tap a`: ARMED, yellow (the flywheels are off, so a pull would not fire; red
  once `hold lb` has spun them up), LCD `ARMED S`; `tap a`: DISARMED, green. `hold rt`,
  `tap a`: refused (`A (arm): refused: trigger held`), stays green; `release rt`.
  Mac: `accept_06_a_arms_and_disarms_and_arm_refused_while_the_trigger_is_held`.
- [ ] **7. Fire mode.** ARMED: `tap b` turns the LCD `ARMED F`; again: `ARMED S`. The LEDs do not
  change with the mode (they show red / yellow for ready / not ready). B also works while
  DISARMED (LEDs stay green).
  Mac: `accept_07_b_toggles_semi_full_in_any_state`.
- [ ] **8. Flywheels.** DISARMED: `hold lb` does nothing. ARMED: `hold lb`, the wheels ramp over
  0.4 s; `release lb` stops them. ENA duty ≈ 113 ÷ 255 = 44 %: a multimeter on D10 reads about
  2.2 V, a scope shows 490 Hz at 44 % high. Then measure the motor voltage (`tuning.md` step 9).
  Mac: `accept_08_flywheels_only_when_permitted_and_ramped`.
- [ ] **9. Semi auto.** First without darts, then with. ARMED, semi, `hold lb` for 1 s,
  `trigger 0.7`: one stroke, ammo 15 to 14. Hold it: no second stroke. `trigger 0.5`: still none
  (between the thresholds). `trigger 0.3`, then `trigger 0.7`: a second stroke, ammo 13.
  Mac: `accept_09_semi_auto_one_stroke_per_pull_with_hysteresis`.
- [ ] **10. Full auto.** `tap b` (full), `hold lb`, `trigger 1`: strokes back to back, about 0.84
  a second (1194 ms each at REST 180 / FIRE 55), ammo down one per stroke; `trigger 0`
  mid-stroke: that stroke finishes, then it stops.
  Mac: `accept_10_full_auto_back_to_back_and_release_finishes_the_stroke`.
- [ ] **11. Per-stroke feedback.** ARMED and spun up the LEDs are red. Slow-motion video: all 8
  LEDs yellow from each stroke's start for the whole stroke (at least 120 ms), then red again;
  in full auto with the trigger held they stay yellow, with no red between strokes. The bench
  console prints one `status: ... rumble #N (200/120, 80 ms)` line per stroke.
  *(radio)*: the pad rumbles (Part C4).
  Mac: `accept_11_each_stroke_shows_yellow_then_red_and_sends_one_rumble`,
  `full_auto_held_stays_yellow_between_strokes`.
- [ ] **12. Empty magazine.** Fire until ammo 0: the LEDs flash purple twice every second over
  red / yellow (green once disarmed), and the trigger still fires (`FIRE_LOCKOUT_WHEN_EMPTY`
  false; the count stays 0). `tap x`: ammo 15, the flashing stops.
  Mac: `accept_12_empty_magazine_keeps_firing_flashes_purple_and_x_restores`.
- [ ] **13. Link loss.** ARMED, wheels on, stick held. Then `disconnect` (or stop the relay, or
  pull the USB cable with the 9 V battery in): within 0.25 s (`LINK_TIMEOUT_MS`) the turret
  starts braking at its acceleration limit (from full pan speed that takes about another 0.25 s),
  the wheels are cut, the LEDs pulse blue and white, LCD row 1 shows `Pairing` for 3 s then
  `No Link`. Reconnect: DISARMED, green, never ARMED. In BT mode, switching the pad off is the
  same test.
  Mac: `accept_13_link_loss_stops_everything_within_link_timeout_and_reconnects_disarmed`,
  `connected_zero_frames_are_a_link_loss`, `source_change_is_a_link_loss`.
- [ ] **14. Mode toggle.** Flip A1: the console prints `status: want WIFI ...`, LCD row 1 says
  `WIFI`, the state goes to PAIRING. *(radio)*: the ESP32 restarts into WiFi; the USB port
  disappears for about a second and the console reopens it (Part C3).
  Mac: `accept_14_mode_toggle_changes_want_mode_lcd_and_goes_through_pairing`.
- [ ] **15. Stall.** Pot on A2 (`tuning.md` Part 1 step 12), `STALL_DETECT_ENABLED true`. While
  moving, twist it fast: both axes stop and back off 2°, red hard blink, LCD `FAULT: STALL`, and
  after 1.5 s the previous state returns.
  Mac: `accept_15_stall_twist_stops_backs_off_blinks_and_clears`.
- [ ] **16. LED brightness.** The dashboard's LED ring mirrors LED 0 as sent: no channel ever above
  127. A current meter in the LED module's 5 V lead, BOOT white at its brightest: about half of
  the module's full-white current (WS2812s are usually rated about 20 mA per colour at full, so
  expect no more than about 240 mA for 8). The sketch refuses to compile with
  `LED_MAX_BRIGHTNESS` above 127.
  Mac: `accept_16_brightness_never_exceeds_the_limit_in_a_long_random_session`.
- [ ] **17. Loop timing.** Part C1. The Mac test cannot measure step jitter (its fake timers are
  perfect); only the board can.
  Mac (logic only): `accept_17_loop_never_blocks_and_the_stats_prove_it`.
- [ ] **18. *(radio)* BT end to end.** The pad pairs over BLE and does items 4-14 with no USB
  control; then Blink and D-Sentry still upload over USB-C through the forked bridge
  (`flashing.md` step 5). Mac: `tests/chat/run.sh` runs the RA4M1's link code against the real
  AT server of the fork (60 s at 100 frames/s with no timeout), `tests/esp32/run.sh` the fork's
  D-Sentry handlers.
- [ ] **19. *(radio)* WiFi end to end.** A1 closed, pad paired with the Mac, relay `--udp`: items
  4-14 again. Mac: `tests/chat/run.sh` (the mode switch through an ESP32 restart), the relay's
  UDP tests against a stand-in turret.
- [ ] **20. Literals.** `"$PY" D-SentryCode/tools/check_literals.py --ra4m1` and `--esp32` report
  every file clean. Fully proven on the Mac (part of `tools/build.sh`).

## Part C. Checks only the hardware can answer

### C1. Loop and step timing (spec item 17)

The step timing is measured inside the step interrupt from the chip's cycle counter, so it does
not need the motors to turn. The safest run is with the LiPo disconnected (the drivers' logic is
fed by the board, so they just ignore the steps).

1. Bench console, DISARMED, `!log 2`. Hold `stick 1 1`, and every second or so swap
   `stick 1 -1` / `stick 1 1`, so pan runs at full speed and tilt crosses its range at full speed.
   With the motors connected, do this only if pan may turn freely for a minute.
2. Read 60 stats lines. Pass: `jit=` worst under 100 (10 %) and `loss=` under 1.0 % on every line,
   no `!` marks, `drop=0`. Note `loop=` (passes per second) and `!stats` `loopMax=` (the longest
   single pass, in µs).
3. Repeat in WiFi mode (relay `--udp`): `rx=` drops to about 76-79 with telemetry at 10 a second
   (`tuning.md` Part 2 item 4); `loss=` must still stay under 1 %.
4. Failing `jit=`: note which state and mode, and whether `led=` (forced LED updates) is high at
   the same time.

### C2. Serial2 write timing

The link code assumes that a write to Serial2 (the UART to the ESP32) returns at once when its
16-byte FIFO is empty, and would wait about 1.4 ms when it is not. That finding comes from
disassembling the core, not from a board, so measure it once. Upload this on its own (it sends
`AT` pings, which the ESP32 answers with `OK`):

```cpp
// serial2_timing.ino -- bench check, not part of D-Sentry.
#include <Arduino.h>
const uint8_t BURST[] = "AT\r\nAT\r\nAT\r\nAT\r\n";   // 16 bytes = one FIFO
const size_t BURST_LEN = sizeof(BURST) - 1;
void setup() { Serial.begin(115200); Serial2.begin(115200); delay(2000); }
void loop() {
  uint32_t t0 = micros(); Serial2.write(BURST, BURST_LEN);
  uint32_t t1 = micros(); Serial2.write(BURST, BURST_LEN);
  uint32_t t2 = micros();
  Serial.print("first "); Serial.print(t1 - t0); Serial.print(" us, second ");
  Serial.print(t2 - t1); Serial.println(" us");
  delay(1000);
  while (Serial2.available()) Serial2.read();
}
```

- First under about 100 µs, second about 1,000-1,400 µs: the finding holds; nothing to do.
- First also slow: a write into an empty FIFO blocks, so D-Sentry's loop can stall. Report it.
- Second also fast: the FIFO takes more than assumed; D-Sentry is merely cautious. Note it.

Upload D-Sentry again afterwards.

### C3. Does an ESP32 restart reset the RA4M1?

Every BT/WiFi switch restarts the ESP32. The fork no longer drives the RA4M1's reset line low at
ESP32 boot, but that line floats for a few hundred ms while the ESP32 restarts.

1. D-Sentry ESP32 image, relay `--monitor`, dashboard *Health* "RA4M1 uptime". Note it.
2. Flip A1. The port disappears and comes back; the relay reopens it.
3. Pass: the uptime kept counting, no `D-Sentry RA4M1 boot` line, and **no new tilt calibration**
   (a reset would ram the panel again). Flip back and forth ten times.
4. Fail: report it with the count of resets in ten flips; the fork's reset-line handling then
   needs a change.

### C4. Rumble on the pad

Use the pad's own buttons here, with the relay in `--monitor` (or nothing) on the USB port: packets
from the bench console would take over as the USB source and disarm the turret.

1. BT mode, pad paired, no darts. ARMED (A), wheels on (hold LB), one semi pull: the pad rumbles
   once, briefly (80 ms).
2. Full auto (B) for 2 s, then release: the rumble runs while it cycles and **stops** within
   about half a second. A rumble that never stops means the pad ignores the "stop" report (both
   motors at 0), which is unverified.
3. Press A in the middle of a full-auto burst (disarm mid-stroke): any rumble stops.
4. WiFi mode: the same through the relay; the pad rumbles on the Mac.

### C5. BLE reconnect after a power cycle

1. Pad paired in BT mode, working.
2. Remove all power (USB and 9 V) for 10 s, power up, then switch the pad on. Pass: it reconnects
   without the Pair button, LCD `BT Connected`. Note how long it takes.
3. Again with the pad already on before the board powers up.
4. Fail: note whether the ESP32 still sees the pad at all (D18's scan diagnostic).

### C6. Real WiFi timings

Time each with a phone stopwatch and write the numbers down:

- [ ] A1 to WiFi with `CHANGE_ME` credentials: seconds until the Mac can see the `D-SENTRY`
      network (expected: about 10 s of failed join, then the access point).
- [ ] With your own credentials: seconds until the station joins.
- [ ] Relay `--udp`: seconds until it finds the turret (the beacon comes once a second).
- [ ] Dashboard *Link and latency* "Round trip (lastCtrlSeq)" in WiFi mode, against USB.
- [ ] *Link and latency* "Turret control rate" in WiFi mode (expected 76-79 a second).
- [ ] A1 back to BT, pad on: seconds until `BT Connected`.
- [ ] A sketch upload while in WiFi mode: the station drops and rejoins by itself.
- [ ] Does `No Link` show during a switch that is otherwise healthy (`tuning.md` Part 2)?

### C7. LCD failure and repair on the real bus

The LCD restart sequence and the I2C error detection have only run against the Mac's model.

1. While running, unplug the LCD's four-wire lead (its 5 V and GND come out with it). Pass: the
   turret keeps working, `!stats`
   timing is unchanged, `!status` `lcdErrors=` rises.
2. Plug it back in. Pass: within a second or so the LCD redraws itself (it waits 0.5 s, restarts
   the display controller, then rewrites both rows).

### C8. Expected noise after a mode switch

The ESP32's boot banner reaches the RA4M1's USB input at each restart. The bench shell may answer
a few `unknown command: ... (help lists them)` lines. Nothing runs: the shell only acts on whole
command words.
