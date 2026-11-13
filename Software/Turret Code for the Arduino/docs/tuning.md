<!-- SPDX-License-Identifier: MIT -->
# Tuning D-Sentry

Every number the turret uses lives in `D-SentryCode/config.h`. Each line carries a tag:

- **[HW]** must match the physical build (a jumper, a wire, a chip). Get it right once.
- **[TUNE]** a value you adjust on the bench until the turret behaves.
- **[CHOICE]** a behaviour switch: either setting works, you pick one.

Part 1 is the order to tune in, with the bench procedure for each step. Part 2 is the list of
decisions only you can make. Part 3 is the reference: every tagged value by subsystem, with its
default, safe range, where to see it and what goes wrong when it is off.

**After any change to `config.h`, upload the sketch again** (`../README.md`, "Upload the
sketch"); every value is compiled in. Values marked **ESP32** in Part 3 are also compiled into
the ESP32 image: after changing one, run `"$PY" D-SentryCode/tools/sync_config.py`, rebuild the
ESP32 image (`flashing.md`) and flash it again. `tools/build.sh` fails until the copy is
current. The PC relay reads `config.h` when it starts, so restart it too.

## How you see what the turret is doing

Three windows onto the RA4M1, all over the USB cable:

- **Bench console**: `"$PY" D-SentryCode/tools/bench_console.py`. It sends neutral control
  packets at 100 Hz, so the turret leaves PAIRING (type `connect` if it was started
  `--disconnected`). Lines starting `< ` are the RA4M1's own text. `!<command>` sends a shell
  command: `!status`, `!stats`, `!stall?`, `!log 3`, `!pan 30`, `!fire` (all listed in
  `../README.md`, "The bench shell").
- **Dashboard**: `"$PY" D-SentryCode/relay-pc/dsentry_relay.py` (or `--monitor` in BT mode),
  then `http://127.0.0.1:8766/`. Panels named below: *State*, *Motion*, *Link and latency*,
  *Health*, *Stall sensor (A2)*, *LCD and LEDs*, *Input being sent*. Only one program can hold
  the port, so run the dashboard or the bench console, not both.
- **The stats line**, printed once a second at log level 2 (the default):

  ```
  52000 stats loop=4210 rx=100 to=0 crc=0 gap=0 loss=0.0% jit=12/3pm led=0 drop=0
  ```

  `loop` = main-loop passes per second. `rx` = valid control packets in the last second.
  `to` = AT replies from the ESP32 that timed out in that second. `crc` = USB frames with a bad
  checksum. `gap` = missing USB packet sequence numbers. `loss` = lost control packets in
  percent (from `gap` over USB, from timed-out or failed polls over the radio). `jit` = step
  timing error, worst and mean, in permille (thousandths) of the step period: `jit=12/3pm`
  means the worst step landed 1.2 % early or late. `led` = LED updates forced into a busy
  moment. `drop` = debug lines dropped because the USB queue was full. A `!` after `loss` or
  `jit` means that value is over its limit (1 % and 10 %, spec acceptance 17). The numbers in
  the example are illustrative, not measured.

Log levels (`!log N`, or `DEBUG_LOG_LEVEL` for the boot value): 0 silent, 1 errors and
warnings, 2 also state changes and the stats line, 3 also the stall trace (20 lines a second).

## Part 1. The order to tune in

Hardware facts first, because every later number depends on them. Then directions, limits and
calibration, because the turret moves on its own at every power-on. Then the feel of the
controls, then the parts that throw darts.

### 1. Microsteps (`MICROSTEPS`)

A microstep is a fraction of the motor's natural 1.8° step that the driver chip creates by
sharing current between the two coils. You set it with three jumpers under each driver.

You set the drivers to 1/8: the first two jumpers closed, the third open (you called them
MS0+MS1; the A4988 datasheet calls the pins MS1/MS2/MS3, the DRV8825 M0/M1/M2, and on both
chips "first two high, third low" is 1/8). `MICROSTEPS 8` matches that. Every driver socket used
must be set the same way.

Check it: once the turret reaches DISARMED with the motors connected (`bench-checklist.md` A5),
send `!pan 90` (the shell takes position moves in DISARMED or ARMED). The turret must turn exactly a
quarter turn. Half a quarter turn means the jumpers are at 1/16; double means 1/4.

The step rate grows with the microstep count, and `STEP_MAX_RATE_HZ` (3,000 steps per second)
caps it. At 1/8 the pan gear (5:1) gives 22.2 steps per degree, so 90°/s is 2,000 steps/s. At
1/16 the same 90°/s would need 4,000 steps/s and the sketch refuses to compile until you lower
`PAN_MAX_DEG_PER_S` to 67 or less.

### 2. Which motor is on which driver (X/Y)

`config.h` assumes the X socket drives pan and Y drives tilt. The first power-on with the motors
connected runs the tilt calibration, which drives the Y motor slowly for about 10 s. If the turret
**pans** instead of tilting, flip the power toggle off at once and swap. Two ways:

- swap the four pin numbers in `config.h` section 1 (`PIN_PAN_STEP 3`, `PIN_TILT_STEP 2`,
  `PIN_PAN_DIR 6`, `PIN_TILT_DIR 5`), or
- with every power source off, swap the two motor plugs between the X and Y sockets. Never plug
  or unplug a stepper while its driver is powered: that can destroy the driver.

The gears differ (pan 5:1, tilt 4:1), so a swap left uncorrected also gives wrong angles.

### 3. LCD address (`LCD_I2C_ADDR`)

The LCD's I2C backpack answers on one address, usually 0x27 (PCF8574T chip) or 0x3F (PCF8574AT).
D-Sentry tries `LCD_I2C_ADDR` (0x27), then `LCD_I2C_ADDR_FALLBACK` (0x3F), and runs without an
LCD if neither answers. The boot line says which: `lcd 1 0x27`.

Only when the LCD stays blank, upload the scanner on its own and read its report:

```sh
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi --upload -p "$PORT" D-SentryCode/tools/i2c_scan
arduino-cli monitor -p "$PORT" -c baudrate=115200
```

It prints the idle level of SDA and SCL (both must read HIGH), every address that answers, and
the exact `#define LCD_I2C_ADDR` line to put in `config.h`. Upload D-Sentry again afterwards. A
blank LCD that the scanner does find usually needs its contrast trimmer (the small blue
potentiometer on the backpack) turned.

### 4. Driver current (Vref)

Each stepper driver limits the coil current to a value set by its trimmer. You read that
setting as a voltage, Vref, between the trimmer's metal top and GND.

- Upload `CODE/src/StepperHoldTest.cpp` (`pio run -d CODE -e stepper_hold_test -t upload
  --upload-port "$PORT"`). It enables the drivers and never steps, so the motors hold still.
- LiPo connected, motors connected (plugged in before power was applied). Measure Vref with a
  multimeter, turn the trimmer with an insulated screwdriver.
- **A4988**: current limit = Vref ÷ (8 × R), where R is the current-sense resistor on the driver
  board (often marked R050 = 0.050 Ω, R068 or R100). With R050 that is Vref × 2.5, as the
  sketch's comment says: 0.4 V gives 1.0 A. **DRV8825**: current limit = Vref × 2.
- Set it no higher than the motor's rated phase current (on its label or datasheet). Starting
  at about 70 % of the rating keeps the motors and drivers cooler; raise it if the motor skips.

Too low: the motor skips steps under load (the barrel droops, or the turret ends up off its mark
after fast moves). Too high: motor and driver run hot and the calibration ram hits the panel
harder. Upload D-Sentry again when done.

### 5. Directions (`TILT_DIR_INVERT`, `PAN_DIR_INVERT`)

**Tilt first**, because calibration uses it. At power-on (toggle closed) the barrel must drive
**down** at 10°/s. Keep a hand on the power toggle for this run: if the barrel goes **up**, flip
the toggle off at once (the motors stop and the drivers release), then set
`TILT_DIR_INVERT true`. The barrel may sag under gravity while the drivers are off.

**Pan** next: in DISARMED, push the right stick right (bench console: `stick 0.3 0`). Seen from
above, the turret must turn clockwise, and the dashboard's *Motion* "Pan" value must go
negative (positive pan = counter-clockwise, the contract's sign). If it turns the other way,
set `PAN_DIR_INVERT true`.

Then **stick up = barrel up** (`stick 0 0.3`). If the barrel goes down while calibration went
the right way, the pad's Y axis is the problem, not the motor: check `JOY_INVERT_Y` (true for
every source D-Sentry knows).

### 6. Limits (`TILT_RANGE_DEG`, `TILT_CAL_BACKOFF_DEG`, `PAN_LIMITS_ENABLED`, `PAN_MIN_DEG`, `PAN_MAX_DEG`, `SOFT_LIMIT_SLOWDOWN_DEG`)

After calibration, tilt 0 is the bottom stop plus `TILT_CAL_BACKOFF_DEG` (3°), and the turret
never steps past `TILT_RANGE_DEG` (60°) above that. To find the real range, power off (the
drivers release), move the barrel by hand from the bottom stop to the top stop and measure the
angle (a phone inclinometer works). Set `TILT_RANGE_DEG` to that angle minus the 3° back-off
minus a margin of a few degrees. Check it: `!tilt 60` (your new value) must stop short of the
top. A target outside the limits is refused, so the shell cannot drive past them either.

Pan has no stop and no calibration: **pan 0 is wherever the turret points when the RA4M1
starts** (board power-up, a sketch upload or the RESET button). The power toggle does not reset
it, and in OFF the drivers release, so a turret turned by hand while OFF keeps a wrong pan count
until the next restart. If the wiring limits rotation, set `PAN_LIMITS_ENABLED true` with
`PAN_MIN_DEG`/`PAN_MAX_DEG` inside the safe span, and always start the board with the turret
facing the same marked direction, or the limits move with it.

The turret starts braking before every limit so it stops on it, never past it. At the default
speeds and accelerations it starts braking about 10° before a pan limit and 6-7° before a tilt
limit (more than `SOFT_LIMIT_SLOWDOWN_DEG`, which is the minimum).

### 7. Calibration (`TILT_CAL_*`)

`CAL_RAM` (the default until A2 is wired) drives tilt down 90° (`TILT_CAL_MAX_TRAVEL_DEG`) at
10°/s whatever the start height. The motor skips harmlessly against the bottom panel for the rest
of the travel, waits 300 ms, calls that the stop, lifts 3° and calls that tilt 0. It takes about
10 s; the LCD shows `Calibrating`.

- Repeatability: put a tape flag on the barrel, then `!cal` (allowed from DISARMED or PAIRING)
  several times. The flag must come back to the same height each time.
- The ram must be gentle: a hard knock means lower `TILT_CAL_SPEED_DEG_PER_S` or Vref.
- `TILT_CAL_MAX_TRAVEL_DEG` must be more than the whole range plus the back-off (60 + 3 = 63°).
  The default 90° leaves room for a start at the very top.

### 8. Feel of the controls (`JOY_*`, `*_MAX_DEG_PER_S`, `*_ACCEL_DEG_PER_S2`, `HOME_SPEED_DEG_PER_S`)

The stick passes three stages. Inside the deadzone (12 % of travel) it reads zero. Outside it,
the travel is rescaled to 0-1, then bent by the expo: output = (1 − e) × s + e × s³ with
e = `JOY_EXPO`. Worked example at half stick: (0.50 − 0.12) ÷ 0.88 = 0.43; with e = 0.4 the
output is 0.6 × 0.43 + 0.4 × 0.08 = 0.29, so pan runs at 0.29 × 90 = 26°/s. Full stick is always
the full 90°/s. More expo = finer aim near the centre, same top speed.

Speed and acceleration trade against **skipped steps**, which the step counter cannot see (there
is no encoder). The test: mark the pan direction with tape, `!sethome`, drive hard for a minute
(full-stick reversals), then `!home`. The turret must line up with the mark again. If it does not,
lower the max speed or the acceleration, or raise Vref. For tilt, `!tilt 0` after hard driving
must sit 3° above the panel.

The dashboard's *Motion* panel shows both velocities; *Input being sent* shows the raw stick.

### 9. Flywheel voltage (`FLYWHEEL_DUTY_255`)

The L298N switches the LiPo on and off 490 times a second; the duty (0-255) sets the average
voltage the flywheel motors see: about duty ÷ 255 × (pack voltage − L298N drop). The drop is
about 1.8 V, and up to about 3 V at 1 A, which is why you measure instead of trusting the sum.

Measure: no darts, ARMED, hold LB (bench console: `!arm`, then `!fly on`), wait for the 400 ms
ramp, multimeter on DC volts across the motor terminals (L298N OUT1/OUT2). A plain multimeter
averages the switching, so it reads close to the average voltage. The dashboard *State* panel
shows the duty being sent.

| Pack | Per cell | Duty 113 gives | Duty for 9 V |
|---|---|---|---|
| full, 25.2 V | 4.20 V | about 10.4 V | 98 |
| nominal, 22.2 V | 3.70 V | about 9.0 V | 113 |
| low, 21.0 V | 3.50 V | about 8.5 V | 120 |

The default 113 gives 9 V on a pack at its nominal voltage, which means about 10.4 V on a full
pack. Your choice: tune at full charge for "never above 9 V" (about 98, darts get weaker as the
pack drains), or keep 113 for "9 V on average". Spec acceptance 8 also wants the ENA pin's duty
≈ 113 ÷ 255 = 44 %: a multimeter on ENA (D10) reads about 0.44 × 5 V = 2.2 V.

If the wheels spin backwards, flip `FLYWHEEL_FORWARD_IN1_HIGH`.

### 10. Trigger servo stroke (`TRIGGER_REST_ANGLE`, `TRIGGER_FIRE_ANGLE`, `TRIGGER_SWEEP_DEG_PER_S`, `TRIGGER_FORWARD_MS`, `TRIGGER_RETURN_MS`, `TRIGGER_COOLDOWN_MS`)

**No darts in the magazine.** The flywheels spin during this test because a shot needs them
ready; if you would rather they did not, unplug their leads from the L298N with the LiPo off.

`!arm`, `!fly on`, wait a second, `!fire`: one stroke. Watch the pusher (a 240 fps phone video
helps). At REST (180°) it must sit clear of the dart path so the next dart can drop in. At FIRE
(55°) it must push a dart fully into the wheels without the servo stalling against the end
(buzzing or getting warm). The quickest way to set both is SERVO ADJUST (Menu from DISARMED,
below). The dashboard *State* "Servo" shows the commanded angle.

A stroke has four parts. The commanded angle glides from REST to FIRE at
`TRIGGER_SWEEP_DEG_PER_S` (300°/s), stays at FIRE for `TRIGGER_FORWARD_MS` (180 ms), glides back
at the same rate, and stays at REST for `TRIGGER_RETURN_MS` (180 ms). Only then is the stroke
complete and the ammo count goes down by one. A hobby servo reports no position, so the glide is
what keeps the pusher on the commanded angle: keep `TRIGGER_SWEEP_DEG_PER_S` below the servo's own
top speed (a servo rated 0.12 s per 60° turns 500°/s with no load). With REST 180 and FIRE 55 the
glide covers 125°, so each sweep takes 417 ms and a stroke takes 417 + 180 + 417 + 180 = 1194 ms.
If the pusher lags behind the glide (it reaches FIRE late, or turns back short of it), lower the
rate. `TRIGGER_SWEEP_DEG_PER_S 0` turns the glide off: the servo jumps to each angle and the two
holds alone must cover its travel.

In semi auto the turret then waits `TRIGGER_COOLDOWN_MS` (500 ms) before the next stroke may
start: 1194 + 500 = 1694 ms per shot, about 0.6 shots per second. A pull held through the wait
fires the moment it ends; a tap inside it is ignored, and `!fire` answers `refused: semi-auto
cooldown after the last shot`. Full auto has no wait: one stroke every 1194 ms, about 0.84 shots
per second.

You can also set REST and FIRE with the pad instead of editing numbers: while DISARMED, press
**Menu** (the three-line button) to enter SERVO ADJUST. The LCD shows `SERVO ADJUST <angle>` over
`R:<rest> F:<fire>` (the angles in force), the LEDs go cyan, and the servo goes to REST.

| Control | In SERVO ADJUST |
|---|---|
| D-pad up / down | servo +1° / −1° (`SERVO_ADJUST_STEP_SMALL_DEG`) |
| RB / LB | servo +5° / −5° (`SERVO_ADJUST_STEP_LARGE_DEG`) |
| A | record the angle as REST |
| B | record the angle as FIRE |
| X | one test stroke between REST and FIRE (no dart counted) |
| View (two squares) | put `config.h`'s REST and FIRE back in force, servo to that REST |
| Menu | save and leave |
| Right stick | still aims |
| Y, RT, d-pad left / right | nothing |

**Menu saves the angles in the RA4M1's data flash** (owner, 2026-11-03), so they survive the power
toggle, a reset and a power cut: every boot uses them in place of `TRIGGER_REST_ANGLE` /
`TRIGGER_FIRE_ANGLE`, and the boot log says which it uses (`servo adjust: loaded saved REST 175 FIRE
30 (config.h has REST 180 FIRE 25, not used)` or `servo adjust: no saved angles, using config.h
...`). When nothing changed, Menu writes nothing (`... unchanged, nothing written`). Menu is refused
while the turret moves (let go of the stick first): the save pauses the turret for about 50 ms.
Leaving any other way (link lost, the mode or power toggle, a fault, `!cal`) does not save; the
angles then last until the power toggle opens. Every exit prints the two `#define` lines to paste
into `config.h`. After changing `config.h`'s angles on a turret that has saved ones, enter SERVO
ADJUST, press View, then Menu: that saves `config.h`'s angles over the old ones.
From the bench console: `!servo adjust` (or `!servo teach`) enters or saves and leaves,
`!servo <deg>`, `!servo rest`, `!servo fire`, `!servo reset` (View), `!servo stroke`, `!servo off`
(save and leave). Whether an upload of a new sketch keeps the data flash was not checked on this
board; the boot log line answers it after the first upload.

### 11. Trigger thresholds (`TRIGGER_PRESS_THRESHOLD`, `TRIGGER_RELEASE_THRESHOLD`)

The trigger counts as pulled above 60 % of its travel (614 of 1023) and released below 40 %
(409). The gap between them, called hysteresis, stops a finger resting near 60 % from firing
twice in semi auto. Squeeze slowly and watch the dashboard *Input being sent* trigger bar (it
marks both thresholds) and *State* "Trigger". Keep at least 0.1 between the two.

### 12. Stall threshold (`STALL_*`), once A2 is wired

The stall sensor on A2 is not decided yet (spec section 8). To develop and test it, wire a 10 kΩ
potentiometer: its ends to 5 V and GND, its wiper to A2 (the shield's Resume header).

You can choose the threshold **before** enabling the fault: with `STALL_DETECT_ENABLED false`
the detector still measures and counts the stalls it would have declared.

1. `!log 3`. The trace prints 20 lines a second:
   `stall s=512 b=510 lo=505 hi=518 d=8 n=50 run=0 mv1 ig0 on0`. `s` = latest reading (0-1023,
   so one count is about 5 mV), `b` = baseline (a slow rolling average, frozen while nothing
   moves), `lo`/`hi` = lowest and highest reading in that 50 ms, `d` = largest distance from the
   baseline, `n` = readings taken, `run` = readings in a row beyond the threshold, `mv` = an axis
   is moving, `ig` = inside the start-up ignore window, `on` = detection enabled.
2. Drive normally: full-speed starts, stops and reversals on both axes. Note the largest `d`
   outside the ignore window. That is your noise.
3. Set `STALL_DELTA_COUNTS` to about twice that noise. `!stall?` shows `would=`: the stalls the
   detector would have declared. It must stay 0 through normal driving.
4. Set `STALL_DETECT_ENABLED true`, upload, and twist the pot quickly while the turret moves:
   both axes stop, back off 2°, the LEDs blink red, the LCD shows `FAULT: STALL`, and after 1.5 s
   the turret is back in its previous state. The error line logs the peak delta.

`CAL_STALL_SENSE` calibration uses the same detector: it drives down until the stall event, and
gives up with `FAULT: CAL` after 20 s. Switch `TILT_CAL_MODE` to it only once the real sensor
reliably sees the panel.

### 13. Feedback: LEDs, LCD, rumble

The LED colours, pulse and blink times, LCD texts and rumble strength are taste. The one hard
rule: `LED_MAX_BRIGHTNESS` never above 127 (the sketch refuses to compile above it). If DISARMED
shows red instead of green, the LED module wants the other byte order: set
`LED_PIXEL_ORDER NEO_RGB`.

What the LEDs say (the owner's scheme of 2026-11-03), highest priority first: FAULT blinks red;
SERVO ADJUST is cyan; at ammo 0 (DISARMED or ARMED) two short purple flashes repeat every second
over the normal colour until X reloads; otherwise DISARMED is green, and ARMED is **red when a
pull would fire right now** and **yellow while the next shot is not available** (a stroke
running, at least `LED_FIRE_FLASH_MIN_MS` per shot, or the flywheels still spinning up). Semi /
full is on the LCD (`ARMED S` / `ARMED F`), not the LEDs. In full auto with the trigger held
the LEDs stay yellow from stroke to stroke.

### 14. Link timing (rarely)

`LINK_TIMEOUT_MS` (250 ms) is how long without a valid packet before the turret stops, cuts the
flywheels and drops to PAIRING. Raise it only if `!status` counts link losses (`losses=`) while
the controller is healthy and close; every extra millisecond is time a runaway turret keeps
moving. The rest of Part 3's link section is plumbing that rarely needs a change.

## Part 2. Decisions only you can make

1. **`FAILSAFE_DISARM_ON_LOSS` cannot do anything.** The spec says a reconnection always lands
   in DISARMED, never ARMED, so setting it false cannot restore ARMED. With it false, the boot
   prints `warning: FAILSAFE_DISARM_ON_LOSS false is ignored`. Remove the flag, or keep it as a
   reminder.
2. **WiFi + "Pairing . . ." does not fit 16 columns.** Row 1 in WiFi mode would be
   `WIFI Pairing . . .`, 18 characters, so the third dot is cut and the boot prints one
   "LCD text combination(s) will be cut" warning. BT mode fits exactly. Fix by shortening
   `LCD_TXT_MODE_WIFI` (for example `"WF"`) or `LCD_TXT_PAIRING` (for example `"Pair"`), or
   accept the two missing dots.
3. **Brake mode draws a current spike.** `FLYWHEEL_BRAKE_ON_STOP true` stops the wheels by
   shorting the motors through the L298N (both inputs high, ENA held fully on) for
   `FLYWHEEL_BRAKE_MS` (1 s), then coasts. The L298N only brakes while ENA is high. The
   short-circuit braking current is a spike much like the start-up inrush the soft start exists
   to limit. With braking on, power-off also holds ENA fully on for that second instead of
   going to 0 at once. The default is coast.
4. **WiFi telemetry slows the control polling.** In WiFi mode each telemetry line to the ESP32
   (197 bytes at 115200 baud, 10 a second) holds the shared link for 20-30 ms, so the RA4M1 polls
   the controller about 76-79 times a second instead of 100. That is still far inside
   `LINK_TIMEOUT_MS`. Lower `TELEMETRY_WIFI_HZ` (for example 5) if aiming feels late in WiFi
   mode; the dashboard then updates less often.
5. **`WIFI_SSID` / `WIFI_PASSWORD` are `CHANGE_ME`.** The prebuilt ESP32 image
   (`../../artifacts/dsentry-esp32/`) carries them, so in WiFi mode it tries to join a network
   called CHANGE_ME for 10 s and then hosts its own access point `D-SENTRY` (password
   `dsentry123`). To use your own network, set both, run `sync_config.py`, rebuild and flash.
   Change `WIFI_AP_PASSWORD` too if you use the access point anywhere public: the default is in
   this repo.

Also worth a decision (from the build reports):

- After a mode toggle, the LCD shows `Pairing` for the 3 s grace time, then `No Link`. The ESP32
  restart and WiFi join can take longer than 3 s, so `No Link` may show while all is well.
  Should it show `Pairing` until connected instead?
- A stall fault returns to ARMED by itself after 1.5 s (spec 3.1). Confirm you want that, rather
  than landing in DISARMED.
- `BLE_DEVICE_NAME` is not used by the ESP32 fork: in BT mode the turret is the one that connects
  (the central), so the pad never sees a name.
- A button or trigger held through a reconnect is ignored until it is released and pressed again.

## Part 3. Reference: every tagged value

Columns: **Setting** (default) · **Does** · **Safe range** · **See it** (where the effect shows)
· **If wrong** (too low / too high). "Leave" means the value is a fact of the hardware or the
protocol, and changing it alone breaks something.

### Pins and switches (section 1)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `PIN_PAN_STEP` (2), `PIN_PAN_DIR` (5) [HW] | X driver's step and direction pins | the shield's X pins; swap with tilt's for an X/Y swap | the axis that moves | wrong axis moves |
| `PIN_TILT_STEP` (3), `PIN_TILT_DIR` (6) [HW] | Y driver's step and direction pins | the shield's Y pins | as above | as above |
| `PIN_STEPPER_EN` (8) [HW] | enable line of all drivers, LOW = on | leave | `!status` `drv=`; motors hold or go limp | motors never energise |
| `PIN_FLY_IN1` (9), `PIN_FLY_IN2` (11) [HW] | L298N direction inputs | leave; never `analogWrite` D11 (it shares a timer with the servo) | wheel direction | wheels never spin |
| `PIN_FLY_ENA` (10) [HW] | L298N enable, the PWM pin | a PWM pin; D10 as wired | ENA duty | no speed control |
| `PIN_TRIGGER_SERVO` (12) [HW] | servo signal, hardware 50 Hz PWM | leave (the code drives this pin's timer channel) | servo moves | no firing |
| `PIN_LED_DATA` (13) [HW] | WS2812 data | leave; the bootloader's D13 blink flashes the LEDs at reset (harmless) | LEDs | LEDs dark |
| `PIN_TOGGLE_POWER` (A0), `PIN_TOGGLE_MODE` (A1) [HW] | the two toggles, internal pull-ups | leave | `!status` `power=`, `modeSwitch=` | switch does nothing |
| `TOGGLE_ACTIVE_LOW` (true) [HW] | a closed switch reads LOW | true for switches to GND | as above | both switches read backwards |
| `TOGGLE_MODE_WIFI_WHEN_CLOSED` (true) [CHOICE] | closed = WiFi, open = BT | either | LCD row 1 `BT`/`WIFI` | swapped labels on the panel |
| `PIN_STALL_SENSE` (A2) [HW] | stall sensor input, 0-5 V | an analog pin | `!stall?` `s=` | no stall readings |
| `LINK_SERIAL` (`Serial2`) [HW] | the RA4M1 UART wired to the ESP32 | leave (verified in the core) | `!status` `esp=alive` | ESP32 never answers |
| `LINK_BAUD` (115200) [HW] | its baud rate | leave: the fork's AT server is fixed at 115200 | as above | link dead |

### Mechanics (section 2)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `MOTOR_FULL_STEPS_PER_REV` (200) [HW] | full steps per motor turn | 200 (1.8° motor) or 400 (0.9°) | `!pan 90` turns 90° | every angle off by 2× |
| `MICROSTEPS` (8) [HW] | microsteps per full step, = the jumpers | 1, 2, 4, 8, 16 | `!pan 90` | angles and speeds off by the ratio; 16 needs a lower pan speed (Part 1 step 1) |
| `PAN_GEAR_RATIO` (5.0) [HW] | motor turns per pan turn | the gearbox | `!pan 90` | angles off |
| `TILT_GEAR_RATIO` (4.0) [HW] | motor turns per tilt turn | the spur pair | `!tilt 30` | angles off |
| `PAN_DIR_INVERT` (false) [TUNE] | flips pan direction | true/false | stick right = clockwise from above | stick right turns left |
| `TILT_DIR_INVERT` (false) [TUNE] | flips tilt direction | true/false | calibration drives down | calibration drives **up**: power off at once |

### Motion (section 3)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `JOY_DEADZONE` (0.12) [TUNE] | stick travel ignored around centre | 0.05-0.25 (code: 0 to under 1) | *Motion* velocities with the stick released | low: turret creeps on a released stick; high: dead, then jumpy |
| `JOY_EXPO` (0.40) [TUNE] | curve: 0 linear, 1 cubic | 0-1 | worked example, Part 1 step 8 | low: twitchy aim; high: sluggish near centre |
| `PAN_MAX_DEG_PER_S` (90) [TUNE] | pan speed at full stick | up to 135 at 1/8 (compile cap from `STEP_MAX_RATE_HZ`) | *Motion* "Pan velocity" | high: skipped steps (mark test) |
| `TILT_MAX_DEG_PER_S` (60) [TUNE] | tilt speed at full stick | up to 168 at 1/8 | *Motion* "Tilt velocity" | high: skipped steps, the barrel droops |
| `PAN_ACCEL_DEG_PER_S2` (400) [TUNE] | pan acceleration and braking | above 0; 100-800 to try | starts and stops | low: sluggish, brakes far before limits; high: skips at start/stop |
| `TILT_ACCEL_DEG_PER_S2` (300) [TUNE] | tilt acceleration and braking | above 0; 100-600 to try | as above | as above |
| `HOME_SPEED_DEG_PER_S` (45) [TUNE] | speed of go-home and shell `pan`/`tilt` | above 0, ≤ the max speeds | d-pad down, `!home` | high: skips on the way home |
| `CONTROL_LOOP_HZ` (100) [TUNE] | how often the motion planner runs | leave; must stay above `MOTION_CREEP_STEPS_PER_S` (40) | `!stats` | low: coarse speed changes |
| `TILT_RANGE_DEG` (60) [TUNE] | soft top limit above tilt 0 | measured range − back-off − margin | `!tilt <value>` stops short of the top | high: barrel hits the top stop |
| `TILT_CAL_BACKOFF_DEG` (3) [TUNE] | lift after calibration; this height is tilt 0 | 1-10 | flag height after `!cal` | low: barrel rubs the panel at tilt 0 |
| `PAN_LIMITS_ENABLED` (false) [CHOICE] | enforce pan limits | true if cables limit rotation | *Motion* "Pan limits" | false with tight cables: cables wrap |
| `PAN_MIN_DEG` (-170), `PAN_MAX_DEG` (170) [TUNE] | pan limits from the direction at RA4M1 start | inside the cable's safe span | *Motion* "Pan limits", 3D view | too wide: cables pull |
| `SOFT_LIMIT_SLOWDOWN_DEG` (5) [TUNE] | least braking distance before a limit | above 0; 2-15 | the stop at a limit | low: harsh stop; high: slows early |
| `STEPPER_HOLD_WHEN_IDLE` (true) [CHOICE] | keep drivers on at rest so tilt holds | true for this tilt arm | `!status` `drv=` | false: the barrel sags and tilt 0 is lost |
| `STEPPER_IDLE_DISABLE_MS` (0) [TUNE] | with HOLD false: ms still before release | any; ignored while HOLD is true | as above | short: the barrel sags between moves |

### Calibration (section 4)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `TILT_CAL_MODE` (`CAL_RAM`) [CHOICE] | ram the panel, or stop on the stall sensor | `CAL_STALL_SENSE` only once A2 senses the panel | LCD `Calibrating`; log `calibration done` | sense mode without a sensor: `FAULT: CAL` after 20 s |
| `TILT_CAL_SPEED_DEG_PER_S` (10) [TUNE] | calibration speed | 5-20 | how hard the ram sounds | high: hard knock; low: slow boot |
| `TILT_CAL_MAX_TRAVEL_DEG` (90) [TUNE] | CAL_RAM: how far it drives down | more than range + back-off (63°) | the ram always reaches the panel | low: zero set in mid-air if it starts high |
| `TILT_CAL_TIMEOUT_MS` (20000) [TUNE] | CAL_STALL_SENSE: give up after | longer than a full descent (63° at 10°/s = 6.3 s) | `FAULT: CAL` | low: false CAL faults |
| `TILT_CAL_SETTLE_MS` (300) [TUNE] | pause at the stop before zeroing | 100-1000 | flag repeatability | low: zeroed while still bouncing |

### Firing (section 5)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `TRIGGER_REST_ANGLE` (180) [TUNE] | pusher back, dart not pushed | 0-180, clear of the dart path | *State* "Servo"; look | darts jam or cannot drop in |
| `TRIGGER_FIRE_ANGLE` (55) [TUNE] | pusher forward, dart pushed | 0-180, short of the servo's end | look; servo must not buzz | dart not fed / servo stalls hot |
| `TRIGGER_SWEEP_DEG_PER_S` (300) [TUNE] | rate the commanded angle glides REST → FIRE and back while shooting (Part 1 step 10); 0 = jump | 100 to below the servo's no-load speed (often 500) | slow-motion video: the pusher follows the glide | high: pusher lags, the stroke counts before it is back; low: slow shots |
| `TRIGGER_FORWARD_MS` (180) [TUNE] | hold at FIRE after the sweep arrives | 50-500 | slow-motion video | low: pusher turns back before the dart is fed |
| `TRIGGER_RETURN_MS` (180) [TUNE] | hold at REST after the sweep back arrives; then ammo − 1 | 50-500 | as above | low: next dart blocked |
| `TRIGGER_COOLDOWN_MS` (500) [TUNE] | semi auto: least time from one completed stroke to the next start | 0-2000 | `!fire` right after a shot: `refused: semi-auto cooldown after the last shot` | low: shots too close together; high: slow follow-up shots |
| `TRIGGER_SERVO_MIN_US` (544), `TRIGGER_SERVO_MAX_US` (2400) [HW] | servo pulse at 0° and 180° (Servo.h defaults) | the servo's datasheet range, often 500-2500 | angles match | too wide: buzzing at the ends |
| `TRIGGER_PRESS_THRESHOLD` (0.60) [TUNE] | pulled above this fraction | 0.3-0.9 | *Input being sent* trigger bar | low: fires on a light touch |
| `TRIGGER_RELEASE_THRESHOLD` (0.40) [TUNE] | released below this | below PRESS by ≥ 0.1 | as above | too close to PRESS: double shots in semi |
| `MAG_CAPACITY` (15) [TUNE] | ammo after X | 1-255 | *State* "Ammo", LCD | count disagrees with the magazine |
| `FIRE_LOCKOUT_WHEN_EMPTY` (false) [CHOICE] | true: no strokes at ammo 0 | false (owner, 2026-11-03: the count is only an estimate) | the purple double flash at 0 either way | true: the turret stops firing whenever the count runs out early |
| `FIRE_REQUIRES_FLYWHEEL` (true) [CHOICE] | no shot until the wheels have spun up | true for real darts | `!fire` refused: not ready | false: darts pushed into still wheels jam |
| `FLYWHEEL_SPINUP_MS` (600) [TUNE] | wheels on this long before a shot | ≥ `FLYWHEEL_RAMP_MS` + time to full speed | first shot's range | low: weak first dart |
| `ARM_REQUIRES_TRIGGER_RELEASED` (true) [CHOICE] | refuse to arm with RT held | true | `arm: refused: trigger held` | false: still never fires that pull |

### Flywheels (section 6, plus `FLYWHEEL_BRAKE_MS`)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `FLYWHEEL_DUTY_255` (113) [TUNE] | motor voltage (Part 1 step 9) | 0-255; 98-120 for about 9 V | meter on the motor; *State* "Flywheel" | high: over-volted motors; low: weak darts |
| `FLYWHEEL_RAMP_MS` (400) [TUNE] | soft start 0 to full duty | 200-1000 | listen; LiPo voltage dip | low: 3-4 A inrush |
| `FLYWHEEL_HOLD_MODE` (true) [CHOICE] | true: spin while LB held; false: LB toggles | either | *State* "Flywheel" | taste |
| `FLYWHEEL_REQUIRES_ARMED` (true) [CHOICE] | LB ignored while disarmed | true | LB in DISARMED | false: wheels spin disarmed |
| `FLYWHEEL_BRAKE_ON_STOP` (false) [CHOICE] | coast, or brake for `FLYWHEEL_BRAKE_MS` | false unless you accept the spike (Part 2) | stop time | true: current spike |
| `FLYWHEEL_BRAKE_MS` (1000) [TUNE] | brake time, then coast | 200-2000 | stop time | long: longer spike |
| `FLYWHEEL_FORWARD_IN1_HIGH` (true) [HW] | which input high = forward | either | wheel direction | darts pulled back in |

### Stall detection (section 7)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `STALL_DETECT_ENABLED` (false) [CHOICE] | turn the stall fault on | true only with A2 wired | `!stall?` `on=` | true with A2 floating: random faults |
| `STALL_SAMPLE_HZ` (1000) [TUNE] | A2 readings per second | 200-2000; the loop must keep up | `!stall?` `skipped=` | high: samples skipped |
| `STALL_BASELINE_ALPHA` (0.02) [TUNE] | weight of each new reading in the baseline (0.02 at 1 kHz ≈ 50 ms average) | 0.005-0.1 | trace `b=` | high: baseline chases the stall; low: slow drift reads as a stall |
| `STALL_DELTA_COUNTS` (80) [TUNE] | departure from baseline that counts (80 ≈ 0.39 V) | about 2× the trace's normal `d` | trace `d=`; `!stall?` `would=` | low: false faults; high: misses stalls |
| `STALL_CONFIRM_SAMPLES` (5) [TUNE] | readings in a row beyond the threshold | 2-20 | trace `run=` | low: one spike faults; high: slow to react |
| `STALL_IGNORE_AFTER_START_MS` (150) [TUNE] | blind time at every move start | 50-500 | trace `ig1` | low: start kick faults; high: misses early stalls |
| `STALL_FAULT_HOLD_MS` (1500) [TUNE] | FAULT time after a stall | 500-5000 | LCD `FAULT: STALL` | taste |
| `STALL_BACKOFF_DEG` (2.0) [TUNE] | reverse this far after a stall | 0-5 | *Motion* | high: backs into something else |
| `STALL_LOG_HZ` (20) [TUNE] | trace lines per second at level 3 | 1-50 | `!log 3` | high: `drop=` rises |

### LEDs (section 8, plus the show guard)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `LED_MAX_BRIGHTNESS` (127) [HW] | ceiling for every channel byte | 0-127, **never raise** (spec; protects the 5 V rail; compile refuses above 127) | *LCD and LEDs* ring mirror | - |
| `LED_PULSE_PERIOD_MS` (1200) [TUNE] | one dim-bright-dim cycle | 500-3000 | BOOT and PAIRING pulse | taste |
| `LED_PULSE_MIN_FRACTION` (0.05) [TUNE] | floor of the pulse | 0-1 | as above | taste |
| `LED_FIRE_FLASH_MIN_MS` (120) [TUNE] | ARMED yellow at least this long per shot | 50-300 | yellow between red | low: a short stroke invisible |
| `LED_REFRESH_HZ` (50) [TUNE] | LED frames per second | 20-100 | smooth pulse | high: more forced shows (`led=`) |
| `LED_FAULT_BLINK_MS` (250) [TUNE] | fault blink half-period | 100-500 | blink | taste |
| `LED_EMPTY_FLASH_ON_MS` (120), `LED_EMPTY_FLASH_GAP_MS` (120) [TUNE] | each purple flash at ammo 0, and the normal colour between the two | 50-300 | ammo 0 | taste |
| `LED_EMPTY_FLASH_PERIOD_MS` (1000) [TUNE] | the double flash repeats this often | more than 2 × ON + GAP (the build refuses otherwise) | ammo 0 | low: red / yellow underneath hard to read |
| `LED_PIXEL_ORDER` (`NEO_GRB`) [HW] | byte order of the module | `NEO_GRB` or `NEO_RGB` | DISARMED must be green | red and green swapped |
| `LED_SHOW_GUARD_US` (300) [TUNE] | an LED update only when no step is due this soon (an update blocks interrupts about 241 µs) | above 241 | stats `led=`, `jit=` | low: step jitter; high: more forced shows |
| `LED_SHOW_MAX_DEFER_MS` (100) [TUNE] | after this long without a gap, update anyway (counted) | 50-200 | stats `led=` | high: LEDs lag at full speed |

The colours (`LED_COLOR_*`, 0xRRGGBB) are untagged: change them freely. Brightness is applied on
top, so `0xFFFFFF` is already limited to half.

### LCD (section 9, plus the I2C settings and fire-mode texts)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `LCD_I2C_ADDR` (0x27) [HW] | backpack address | the scanner's answer | boot line `lcd 1 0x27` | `warning: no LCD`; runs without it |
| `LCD_I2C_ADDR_FALLBACK` (0x3F) [HW] | second address tried | the other common one | as above | as above |
| `LCD_I2C_CLOCK_HZ` (100000) [HW] | I2C speed | leave (PCF8574 rated 100 kHz) | `!status` `lcdErrors=` | high: garbled text |
| `LCD_I2C_TIMEOUT_US` (3000) [TUNE] | longest wait on a stuck bus per write | 1000-10000 | `lcdErrors=`, `!stats` `loopMax=` | low: false errors; high: a stuck bus costs loop time |
| `LCD_RETRY_MS` (500) [TUNE] | rest after a failed write, then restart the LCD | 200-2000 | unplug and replug the LCD: it redraws | low: busy retrying an absent LCD |
| `LCD_REFRESH_MS` (100) [TUNE] | rows recomputed at most this often; one cell rewritten each time, so a full self-repair sweep is 32 × 100 ms = 3.2 s | 50-500 | *LCD and LEDs* mirror vs the real LCD | high: slow updates and repair |
| `LCD_PAIRING_DOT_MS` (400) [TUNE] | pairing dots step | 200-1000 | PAIRING row 1 | taste |
| `LCD_TXT_SHOW_FIRE_MODE` (true) [CHOICE] | `ARMED S` / `ARMED F` | either | row 0 | taste |
| `LCD_TXT_FIRE_SEMI` (`" S"`), `LCD_TXT_FIRE_FULL` (`" F"`) [CHOICE] | those suffixes | short; dropped when they do not fit | row 0 | boot warns when cut |
| `LCD_TXT_FAULT_OTHER` (`"FAULT"`) [CHOICE] | row 0 for an unnamed fault (should never show) | ≤ 16 chars | - | - |
| `LCD_BACKLIGHT_OFF_WHEN_SYSTEM_OFF` (true) [CHOICE] | backlight off in OFF | either | toggle off | taste |

The other texts (`LCD_TXT_*`) are untagged: edit freely, each row 16 characters at most. The boot
names every combination that will be cut (Part 2 item 2).

### Controller and link (section 10)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `CONTROL_PACKET_HZ` (100) [TUNE] | relay send rate and RA4M1 poll rate | 50-100 | stats `rx=` | high: the AT link cannot keep up; low: laggy aim |
| `LINK_TIMEOUT_MS` (250) [TUNE] **ESP32** | no valid packet this long = link lost | 150-500; `RELAY_SOURCE_CHANGE_HOLD_MS` must stay above it | `!status` `losses=` | low: false losses (disarms); high: longer runaway |
| `LINK_RECONNECT_GRACE_MS` (3000) [TUNE] | `Pairing` shown this long after a loss, then `No Link` | 1000-10000 | LCD row 1 | taste (Part 2) |
| `BUTTON_DEBOUNCE_MS` (40) [TUNE] | ignore chatter after an accepted press | 20-100; `KEYBOARD_PRESS_MS` must stay above it | double-tap A | low: one press counts twice; high: fast taps lost |
| `TOGGLE_DEBOUNCE_MS` (50) [TUNE] | toggle switch settle time | 20-200 | `!status` `power=` | low: a bouncing switch reboots twice |
| `RUMBLE_STRONG_255` (200), `RUMBLE_WEAK_255` (120) [TUNE] | per-shot rumble, heavy and light motor | 0-255 | the pad | taste |
| `RUMBLE_DURATION_MS` (80) [TUNE] | per-shot rumble length | 30-300 | the pad | taste |
| `RUMBLE_FULL_AUTO_CONTINUOUS` (true) [CHOICE] | rumble without gaps in full auto | either | the pad | taste |
| `JOY_AXIS_RANGE` (512) [HW] **ESP32** | stick range of the packet format | leave (protocol) | - | everything scaled wrong |
| `TRIGGER_AXIS_RANGE` (1023) [HW] **ESP32** | trigger range of the packet format | leave (protocol) | - | as above |
| `JOY_INVERT_Y` (true) [HW] | packet's up is negative; true makes up = barrel up | leave | stick up = barrel up | tilt inverted on the stick |
| `FAILSAFE_DISARM_ON_LOSS` (true) [CHOICE] | disarm on link loss | true (false is ignored, Part 2) | boot warning | - |
| `FAILSAFE_STOP_FLYWHEEL` (true) [CHOICE] | cut the wheels on link loss | true | wheels after a loss | false: wheels keep spinning through PAIRING, cut on reconnect |

### ESP32 radio (section 11 and the ESP32 part of section 13); every one is **ESP32**

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `WIFI_SSID`, `WIFI_PASSWORD` (`CHANGE_ME`) [HW] | the network to join in WiFi mode | a 2.4 GHz network; empty SSID = go straight to the access point | *Link and latency* "ESP32 radio" | never joins (Part 2 item 5) |
| `WIFI_MODE_AP_FALLBACK` (true) [CHOICE] | host its own access point if the join fails | true | `D-SENTRY` network appears | false: no network at all on a failed join |
| `WIFI_AP_PASSWORD` (`dsentry123`) [HW] | access point password | 8-63 characters | joining it | under 8: the access point does not start |
| `WIFI_UDP_PORT` (4210) [HW] | the turret's UDP port | any free port 1024-65535 | relay `--udp` connects | relay and turret disagree until both rebuilt/restarted |
| `BLE_FORGET_PAIRINGS_ON_BOOT` (false) [CHOICE] | wipe the pad's pairing at each BT start | false | pad reconnects by itself | true: pair again after every boot |
| `ESP_RESTART_DELAY_MS` (50) [TUNE] | wait between the "restarting" reply and the restart | 20-200 | mode switch | low: reply lost |
| `WIFI_STA_CONNECT_TIMEOUT_MS` (10000) [TUNE] | how long a first join may take before the access point | 5000-30000 | time to `D-SENTRY` | low: falls back on a slow router |
| `WIFI_STA_RETRY_MS` (30000) [TUNE] | gap between join attempts | ≥ the connect timeout (compile check) | - | - |
| `WIFI_AP_IP` (`192.168.4.1`), `WIFI_AP_NETMASK` [HW] | access point address | change both together | relay finds it | relay fallback misses |
| `WIFI_AP_CHANNEL` (6) [CHOICE] | access point's 2.4 GHz channel | 1, 6 or 11 | WiFi scanner app | busy channel: lost packets |
| `WIFI_DISCOVERY_PORT` (4211) [HW] | beacon broadcast port | any free port | relay finds the turret | relay needs `--udp <ip>` |
| `WIFI_BEACON_MS` (1000) [TUNE] | beacon interval | 500-5000 | relay start-up time | - |
| `WIFI_TASK_POLL_MS` (5) [TUNE] | worst delay before a reply leaves the ESP32 | 1-10 | round-trip latency | high: more latency |
| `ESP_WIFI_TASK_STACK_BYTES` (6144), `ESP_WIFI_TASK_PRIORITY` (1), `ESP_WIFI_TASK_CORE` (0) [HW] | the WiFi task's stack, priority and core | leave | - | crash or stalls |

`WIFI_AP_SSID` (`D-SENTRY`) is untagged and can be renamed. `BLE_DEVICE_NAME` is not used
(Part 2).

### Debug and USB (sections 12 and 13)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `DEBUG_ACCEPT_USB_CONTROL` (true) [CHOICE] | accept control packets over USB (the bench override) | true for the bench | relay `--serial` drives | false: only the radio drives; the shell still works |
| `TELEMETRY_USB_HZ` (20) [TUNE] | telemetry frames per second on USB (94 bytes, 16 % of the line) | 5-50 | dashboard *Health* "Telemetry seq" | high: debug lines dropped (`drop=`) |
| `TELEMETRY_WIFI_HZ` (10) [TUNE] | telemetry lines per second to the ESP32 in WiFi mode | 1-10 | dashboard in WiFi mode; stats `rx=` | high: fewer control polls (Part 2 item 4) |
| `DEBUG_TX_BUFFER_BYTES` (1024) [TUNE] | USB output queue | 512-4096 (RAM) | stats `drop=` | low: lines dropped |
| `DEBUG_LOG_LINE_MAX` (96) [TUNE] | longest text line | 64-160 | long lines cut | - |
| `SHELL_LINE_MAX` (48) [TUNE] | longest shell command | 32-96 | over-long commands dropped | - |
| `SHELL_QUEUE_LINES` (4) [TUNE] | shell lines waiting (one runs per loop) | 2-8 | `!stats` | low: pasted commands lost |

`DEBUG_SERIAL_BAUD` (115200), `DEBUG_LOG_LEVEL` (2) and `DEBUG_STATS_INTERVAL_MS` (1000) are
untagged. Keep the baud at 115200: the PC tools use it, and 1200 or 2400 would reset the board.

### AT link plumbing (section 13)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `AT_REPLY_TIMEOUT_MS` (20) [TUNE] | wait for an ESP32 reply after the request's last byte | 15-50 | stats `to=` | low: timeouts with a healthy ESP32 |
| `AT_SIZED_MAX_DIGITS` (7) [HW] | longest size field in a reply | leave (protocol) | - | - |
| `LINK_ESP_PING_MS` (100) [TUNE] | "are you there" pings until the ESP32 answers | 50-500 | stats `to=10` on a board whose ESP32 is silent: these pings | - |
| `LINK_BEGIN_RETRY_MS` (1000) [TUNE] | legacy firmware only: retry starting BLE | 500-5000 | - | - |
| `STATUS_HEARTBEAT_MS` (500) [TUNE] | status repeat when nothing changed | 200-1000 | bench console `status:` | - |
| `LINK_TX_BUFFER_BYTES` (256) [TUNE] | link output queue | ≥ 197 (compile check) | - | - |
| `LINK_RX_PAYLOAD_BYTES` (64) [TUNE] | largest reply kept | ≥ 20 (compile check) | - | - |

### Step generator (section 13)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `STEP_TIMER_PAN_CHANNEL` (0), `STEP_TIMER_TILT_CHANNEL` (1) [HW] | hardware timers that make the steps | leave | boot line `pan timer 1 tilt timer 1` | an axis cannot move |
| `STEP_TIMER_IRQ_PRIORITY` (4) [HW] | step interrupt priority, above the clock and UARTs | leave | `jit=` | jitter |
| `STEP_PULSE_US` (3) [HW] | STEP high time | ≥ 2 (DRV8825 needs 1.9 µs, A4988 1 µs) | - | missed steps |
| `STEP_MAX_RATE_HZ` (3000) [TUNE] | safety cap on step rate | leave; raise only with a jitter check | compile check on the max speeds | - |
| `STEP_MIN_RATE_HZ` (25) [TUNE] | slower requests stop the axis | leave; below `MOTION_CREEP_STEPS_PER_S` | - | - |
| `STEP_JITTER_LIMIT_PERMILLE` (100) [TUNE] | marks `jit` with `!` above 10 % | the spec's 10 % | stats line | only the marker changes |
| `PACKET_LOSS_LIMIT_PERMILLE` (10) [TUNE] | marks `loss` with `!` from 1 % | the spec's 1 % | stats line | only the marker changes |

### Motion planner internals (section 13)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `MOTION_CREEP_STEPS_PER_S` (40) [TUNE] | last steps of every position move run this slowly (1.8°/s pan) | above 25, below 100 (compile check) | end of `!home` | - |
| `MOTION_BRAKE_MARGIN_STEPS` (2) [TUNE] | steps kept in reserve before a limit or target | leave | - | 0: leans on the hard bound |
| `MOTION_BRAKE_PLAN_FRACTION` (0.90) [TUNE] | plan braking at this share of the allowed deceleration | 0.8-1.0 | - | 1.0: drifts past the curve |

### PC relay and dashboard (section 13; read by Python at start-up, no upload needed)

| Setting | Does | Safe range | See it | If wrong |
|---|---|---|---|---|
| `RELAY_PAD_POLL_HZ` (200) [TUNE] | pad read rate | ≥ 200 | - | low: missed taps |
| `RELAY_SERIAL_WRITE_TIMEOUT_MS` (50) [TUNE] | a stuck USB write gives up after | 20-200 | relay log | - |
| `RELAY_REOPEN_MS` (1000) [TUNE] | port reopen interval after an error | 500-5000 | reconnect after a mode switch | - |
| `RELAY_STATS_MS` (1000) [TUNE] | terminal stats line interval | any | relay terminal | - |
| `RELAY_SOURCE_CHANGE_HOLD_MS` (500) [TUNE] | "not connected" sent this long when the input source changes, so the turret disarms | > `LINK_TIMEOUT_MS` (relay refuses to start otherwise) | turret drops to PAIRING | - |
| `KEYBOARD_DEFLECTION` (0.50) [TUNE] | stick fraction for a key without Shift | above 0, up to 1 | dashboard keyboard | - |
| `KEYBOARD_PRESS_MS` (100) [TUNE] | shortest key press sent | > `BUTTON_DEBOUNCE_MS` (checked) | - | - |
| `KEYBOARD_REVOKE_MS` (2000) [TUNE] | keyboard session ends after this long without input | > `LINK_TIMEOUT_MS` (checked) | KEYBOARD LIVE banner | - |
| `KEYBOARD_POST_HZ` (30) [TUNE] | page to relay key-state rate | 10-60 | - | - |
| `KEYBOARD_PRESS_QUEUE_MAX` (4) [TUNE] | most queued taps per key | ≥ 1 | - | - |
| `DASHBOARD_HTTP_PORT` (8766) [CHOICE] | dashboard port | any free port | the URL | - |
| `DASHBOARD_MAX_FPS` (30) [TUNE] | page updates per second | 5-30 | - | - |
| `DASHBOARD_KEEPALIVE_MS` (1000) [TUNE] | keep-alive on an idle stream | 500-5000 | - | - |
| `DASHBOARD_EVENT_LOG_KEEP` (50) [TUNE] | event log length | any | *Events* | - |
| `DASHBOARD_STALE_MS` (500) [TUNE] | no telemetry this long greys every value | > 1000 ÷ `TELEMETRY_WIFI_HZ` (100 ms) | STALE stamps | low: flickers stale in WiFi mode |

Section 14 of `config.h` holds fixed facts (units, chip registers, the LCD controller's
commands, the AT state layout). Nothing there is a tunable.
