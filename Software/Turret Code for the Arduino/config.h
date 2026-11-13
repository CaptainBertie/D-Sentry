// =============================================================================
//  D-SENTRY  —  config.h
//  Every number you will ever want to tune lives in this file and only here.
//  No other source file may contain a hard-coded pin, speed, angle, colour,
//  timeout or threshold. Claude Code: enforce that rule.
//
//  Board : Arduino UNO R4 WiFi (RA4M1 main MCU + ESP32-S3 radio coprocessor)
//  Shield: CNC Shield V3 with A4988/DRV8825 stepper drivers (X = pan, Y = tilt)
//  Units : degrees, mm, milliseconds, 0..255 PWM, 0..1 fractions — as labelled
//
//  Every line marked  [TUNE]  is a value you are expected to adjust on the
//  bench. Lines marked  [HW]  must match the physical build. Lines marked
//  [CHOICE] are behaviour decisions you may want to flip.
// =============================================================================
#pragma once
#include <Arduino.h>

// -----------------------------------------------------------------------------
// 1. PIN MAP  (final assignment from the wiring diagram, 3 Nov 2026)
// -----------------------------------------------------------------------------
// CNC Shield V3 stepper lines ---------------------------------------------------
#define PIN_PAN_STEP        2    // [HW] X.STEP
#define PIN_TILT_STEP       3    // [HW] Y.STEP
#define PIN_PAN_DIR         5    // [HW] X.DIR
#define PIN_TILT_DIR        6    // [HW] Y.DIR
#define PIN_STEPPER_EN      8    // [HW] EN, shared by all drivers, LOW = enabled

// L298N flywheel driver, channel A, via the shield's limit headers -----------
#define PIN_FLY_IN1         9    // [HW] X± limit header -> L298N IN1
#define PIN_FLY_ENA        10    // [HW] Y± limit header -> L298N ENA (PWM)
#define PIN_FLY_IN2        11    // [HW] Z± limit header -> L298N IN2

// Trigger servo ----------------------------------------------------------------
#define PIN_TRIGGER_SERVO  12    // [HW] SpnEn header

// Freenove 8x WS2812 LED module ----------------------------------------------
#define PIN_LED_DATA       13    // [HW] SpnDir header. Bootloader blink on D13
                                 //      causes a harmless flash at reset.
// Toggle switches (INPUT_PULLUP, switch closes to GND) -----------------------
#define PIN_TOGGLE_POWER   A0    // [HW] Abort header. "Toggle switch 2" in spec
#define PIN_TOGGLE_MODE    A1    // [HW] Hold header.  "Toggle switch 1" in spec
#define TOGGLE_ACTIVE_LOW  true  // [HW] true = closed switch reads LOW
#define TOGGLE_MODE_WIFI_WHEN_CLOSED true // [CHOICE] closed = WiFi, open = BT

// Stall / crash sense (see section 7; hardware tap still to be decided) ------
#define PIN_STALL_SENSE    A2    // [HW] Resume header. Analog input, 0..5 V

// Spare: A3 (CoolEn). LCD uses A4/A5 (I2C). D0/D1 (Serial1) are free.

// Inter-chip link to the radio coprocessor -----------------------------------
// LINK_SERIAL must be the RA4M1 UART that is physically wired to the ESP32-S3.
// Claude Code: confirm the object name from the ArduinoCore-renesas variant
// file for UNOWIFIR4 and the uno-r4-wifi-usb-bridge repo before building.
// Fallback: an external ESP32 dev board on D0/D1 uses  Serial1  here instead.
#define LINK_SERIAL        Serial2      // [HW] verify
#define LINK_BAUD          115200       // [HW] must match ESP32-S3 firmware

// -----------------------------------------------------------------------------
// 2. MECHANICS  ->  steps per degree
// -----------------------------------------------------------------------------
#define MOTOR_FULL_STEPS_PER_REV  200   // [HW] 1.8° NEMA17
#define MICROSTEPS                 8    // [HW] MUST match MS1/MS2/MS3 jumpers on
                                        //      the shield (1,2,4,8,16)
#define PAN_GEAR_RATIO            5.0f  // [HW] 5:1 planetary
#define TILT_GEAR_RATIO           4.0f  // [HW] 4:1 module-2 spur pair

// Derived — do not edit:
#define PAN_STEPS_PER_DEG   (MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * PAN_GEAR_RATIO  / 360.0f)
#define TILT_STEPS_PER_DEG  (MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * TILT_GEAR_RATIO / 360.0f)

#define PAN_DIR_INVERT      false  // [TUNE] flip if stick-right turns turret left
#define TILT_DIR_INVERT     false  // [TUNE] flip if stick-up tilts barrel down

// -----------------------------------------------------------------------------
// 3. MOTION  (joystick -> velocity)
// -----------------------------------------------------------------------------
#define JOY_DEADZONE          0.12f   // [TUNE] fraction of full stick travel ignored
#define JOY_EXPO              0.40f   // [TUNE] 0 = linear, 1 = fully cubic feel
#define PAN_MAX_DEG_PER_S    90.0f    // [TUNE] speed at full stick deflection
#define TILT_MAX_DEG_PER_S   60.0f    // [TUNE]
#define PAN_ACCEL_DEG_PER_S2  400.0f  // [TUNE] limits jerk; also the decel rate
#define TILT_ACCEL_DEG_PER_S2 300.0f  // [TUNE]
#define HOME_SPEED_DEG_PER_S  45.0f   // [TUNE] speed used for "go to home"
#define CONTROL_LOOP_HZ        100    // [TUNE] how often joystick is re-read

// Soft limits (degrees from calibrated zero) ---------------------------------
#define TILT_RANGE_DEG        43.87f  // [TUNE] usable upward travel from bottom stop (owner,
                                      //        bench 2026-11-03; was 60.0)
#define TILT_CAL_BACKOFF_DEG   3.0f   // [TUNE] lift off the bottom panel after cal;
                                      //        this becomes tilt = 0 (soft minimum)
#define PAN_LIMITS_ENABLED    false   // [CHOICE] true if wiring restricts rotation
#define PAN_MIN_DEG         -170.0f   // [TUNE] used only when PAN_LIMITS_ENABLED
#define PAN_MAX_DEG          170.0f   // [TUNE]
#define PAN_HOME_SHORTEST_PATH true   // [CHOICE] go-home turns pan the short way, at most half a
                                      //          turn (370 deg away -> 10 deg), instead of
                                      //          unwinding every full turn. Only while
                                      //          PAN_LIMITS_ENABLED is false. Owner 2026-11-03:
                                      //          the electronics ride above the pivot, so no
                                      //          cable crosses the pan joint.
#define SOFT_LIMIT_SLOWDOWN_DEG 5.0f  // [TUNE] start decelerating this far from a limit

// Stepper idle behaviour ------------------------------------------------------
#define STEPPER_HOLD_WHEN_IDLE  true  // [CHOICE] keep drivers enabled so tilt
                                      //          holds against gravity
#define STEPPER_IDLE_DISABLE_MS 0     // [TUNE] if HOLD is false: ms of no motion
                                      //        before EN is released

// -----------------------------------------------------------------------------
// 4. CALIBRATION  (tilt homing at startup)
// -----------------------------------------------------------------------------
// CAL_STALL_SENSE : drive down until the stall sensor fires (needs A2 tap)
// CAL_RAM         : drive down a fixed overshoot distance at low speed and let
//                   the stepper skip steps against the panel, then zero there
#define CAL_STALL_SENSE     1
#define CAL_RAM             2
#define TILT_CAL_MODE       CAL_RAM        // [CHOICE] switch to CAL_STALL_SENSE
                                           //          once A2 is wired
#define TILT_CAL_SPEED_DEG_PER_S  10.0f    // [TUNE] slow, so a ram is gentle
#define TILT_CAL_MAX_TRAVEL_DEG   90.0f    // [TUNE] CAL_RAM: distance commanded
                                           //        downward (> real range)
#define TILT_CAL_TIMEOUT_MS      20000     // [TUNE] CAL_STALL_SENSE: give up and
                                           //        fault if no stall seen
#define TILT_CAL_SETTLE_MS         300     // [TUNE] pause after hitting stop

// -----------------------------------------------------------------------------
// 5. FIRING  (trigger servo)
// -----------------------------------------------------------------------------
#define TRIGGER_SERVO_REVERSED   true    // [HW] owner, 2026-11-03: the trigger servo turns the
                                         //      other way. Every angle below (and in SERVO ADJUST)
                                         //      is mirrored, SERVO_SPAN_DEG - angle, just before it
                                         //      reaches the servo. Flipping this mirrors where the
                                         //      servo goes: re-adjust REST/FIRE after changing it.
#define TRIGGER_REST_ANGLE       180     // [TUNE] servo degrees, dart not pushed: the racked
                                         //        position at the servo's end of travel (set by
                                         //        the owner with SERVO ADJUST, 2026-11-03).
                                         //        Angles saved by SERVO ADJUST replace this one
                                         //        and FIRE below at boot (section 13)
#define TRIGGER_FIRE_ANGLE        25     // [TUNE] servo degrees, dart fully pushed (owner,
                                         //        2026-11-03; was 55 from SERVO TEACH, now ADJUST)
#define TRIGGER_SWEEP_DEG_PER_S  300.0f  // [TUNE] servo degrees per second: while shooting, the
                                         //        commanded angle glides from REST to FIRE and
                                         //        back at this rate, so the pusher moves slower
                                         //        than the servo's own top speed and really
                                         //        follows the command. A hobby servo has no
                                         //        position feedback, so this is what makes "the
                                         //        stroke completed" (and the ammo count) true.
                                         //        0 = jump straight to each angle, as before
#define TRIGGER_FORWARD_MS       180     // [TUNE] hold at FIRE after the sweep arrives
#define TRIGGER_RETURN_MS        180     // [TUNE] hold at REST after the sweep back arrives;
                                         //        then the stroke is complete and ammo - 1.
                                         //        Stroke = sweep out + FORWARD + sweep back +
                                         //        RETURN: 417 + 180 + 417 + 180 = 1194 ms at
                                         //        REST 180 / FIRE 55 / 300 deg/s
#define TRIGGER_COOLDOWN_MS      500     // [TUNE] semi auto: after a stroke completes, no new
                                         //        stroke starts for this long; a pull still held
                                         //        when it ends fires then, one let go before it
                                         //        ends never fires. Full auto has no gap
#define TRIGGER_SERVO_MIN_US     544     // [HW]  Servo.attach pulse limits
#define TRIGGER_SERVO_MAX_US    2400     // [HW]

#define TRIGGER_PRESS_THRESHOLD   0.60f  // [TUNE] spec: >60% counts as pulled
#define TRIGGER_RELEASE_THRESHOLD 0.40f  // [TUNE] must drop below this to re-arm
                                         //        a semi-auto shot (hysteresis)
#define MAG_CAPACITY              15     // [TUNE] ammo count after X-button reset
#define FIRE_LOCKOUT_WHEN_EMPTY   false  // [CHOICE] false = keep cycling at 0 ammo (the count
                                         //          stays at 0), true = no stroke at 0. Owner,
                                         //          2026-11-03: the count is only the darts the
                                         //          turret thinks are left, so it never stops
                                         //          firing; the LEDs flash purple at 0 instead
#define FIRE_REQUIRES_FLYWHEEL    true   // [CHOICE] false = trigger works without LB
#define FLYWHEEL_SPINUP_MS        600    // [TUNE] min time flywheels must be on
                                         //        before a shot is allowed
#define ARM_REQUIRES_TRIGGER_RELEASED true // [CHOICE] refuse to arm while RT held

// -----------------------------------------------------------------------------
// 6. FLYWHEELS  (L298N channel A, PWM on ENA)
// -----------------------------------------------------------------------------
// Target 9 V from a 6S LiPo through the L298N:
//   duty = 9 V / (V_batt - V_L298N_drop)  ~  9 / (22.2 - 1.8)  ~  0.44
#define FLYWHEEL_DUTY_255         113    // [TUNE] 0..255 (0.44 * 255 = 113)
#define FLYWHEEL_RAMP_MS          400    // [TUNE] soft-start ramp 0 -> duty,
                                         //        limits the 3-4 A inrush
#define FLYWHEEL_HOLD_MODE        true   // [CHOICE] true = spin while LB held,
                                         //          false = LB toggles on/off
#define FLYWHEEL_REQUIRES_ARMED   true   // [CHOICE] LB ignored while disarmed
#define FLYWHEEL_BRAKE_ON_STOP    false  // [CHOICE] false = coast (IN1=IN2=LOW, ENA 0),
                                         //          true = brake: IN1=IN2=HIGH with ENA held
                                         //          at PWM_MAX for FLYWHEEL_BRAKE_MS, then
                                         //          coast (the L298N brakes only while ENA is
                                         //          high; braking draws a current spike)
#define FLYWHEEL_FORWARD_IN1_HIGH false  // [HW] flip if flywheels spin backwards (owner flipped it
                                         //      on the bench 2026-11-03: they spun backwards)

// -----------------------------------------------------------------------------
// 7. STALL / CRASH DETECTION  (analog spike on PIN_STALL_SENSE)
// -----------------------------------------------------------------------------
// The hardware tap for this signal is NOT yet defined — see spec section "Open
// hardware decisions". The software reads A2 at STALL_SAMPLE_HZ, keeps a rolling
// baseline, and declares a stall when the reading departs from the baseline by
// more than STALL_DELTA_COUNTS for STALL_CONFIRM_SAMPLES consecutive samples
// while a stepper is moving.
#define STALL_DETECT_ENABLED      false  // [CHOICE] off until A2 is wired
#define STALL_SAMPLE_HZ           1000   // [TUNE]
#define STALL_BASELINE_ALPHA      0.02f  // [TUNE] 0..1 rolling-average weight
#define STALL_DELTA_COUNTS        80     // [TUNE] 10-bit ADC counts (0..1023)
#define STALL_CONFIRM_SAMPLES     5      // [TUNE] consecutive samples above delta
#define STALL_IGNORE_AFTER_START_MS 150  // [TUNE] mask the normal start-up kick
#define STALL_FAULT_HOLD_MS       1500   // [TUNE] motion locked out after a stall
#define STALL_BACKOFF_DEG          2.0f  // [TUNE] reverse this far after a stall

// -----------------------------------------------------------------------------
// 8. LED MODULE  (8x WS2812, data on PIN_LED_DATA)
// -----------------------------------------------------------------------------
#define LED_COUNT                 8
#define LED_MAX_BRIGHTNESS        127    // [HW] spec: half brightness max,
                                         //      protects the 5 V rail. Never raise.
#define LED_PULSE_PERIOD_MS      1200    // [TUNE] full dim-bright-dim cycle
#define LED_PULSE_MIN_FRACTION    0.05f  // [TUNE] floor of the pulse (0 = fully off)
#define LED_FIRE_FLASH_MIN_MS     120    // [TUNE] ARMED yellow (next shot not ready) stays
                                         //        on at least this long per shot, so even
                                         //        a very short stroke is visible
#define LED_REFRESH_HZ             50    // [TUNE]
// Colours as 0xRRGGBB, applied under LED_MAX_BRIGHTNESS ----------------------
#define LED_COLOR_DISARMED        0x00FF00  // green
// ARMED: the owner changed the scheme on 2026-11-03. The LEDs say whether a pull fires right
// now, no longer semi / full (the LCD still shows ARMED S / F); these two replace
// LED_COLOR_ARMED_SEMI, LED_COLOR_ARMED_FULL and LED_COLOR_FIRING.
#define LED_COLOR_ARMED_READY     0xFF0000  // red: a pull would fire right now
#define LED_COLOR_ARMED_WAIT      0xFFFF00  // yellow: the next shot is not available yet
                                            // (stroke running, flywheels spinning up, ...)
#define LED_COLOR_STARTUP         0xFFFFFF  // white, pulses to off
#define LED_COLOR_PAIRING_A       0x0000FF  // blue  }  pulses between the two
#define LED_COLOR_PAIRING_B       0xFFFFFF  // white }
#define LED_COLOR_FAULT           0xFF0000  // red, hard blink (stall / cal fail)
#define LED_FAULT_BLINK_MS        250       // [TUNE]
// Empty magazine (owner, 2026-11-03): ammo 0 in DISARMED or ARMED shows a repeating double
// flash over the colour the LEDs show anyway (green, or ARMED red / yellow) until X reloads:
// flash, gap, flash, then that colour for the rest of LED_EMPTY_FLASH_PERIOD_MS.
#define LED_COLOR_EMPTY_MAG       0x8000FF  // purple
#define LED_EMPTY_FLASH_ON_MS     120       // [TUNE] length of each of the two flashes
#define LED_EMPTY_FLASH_GAP_MS    120       // [TUNE] the colour underneath between the flashes
#define LED_EMPTY_FLASH_PERIOD_MS 1000      // [TUNE] the pattern repeats this often; must be
                                            //        longer than two flashes and the gap
#define LED_PIXEL_ORDER           NEO_GRB   // [HW] swap to NEO_RGB if colours wrong

// -----------------------------------------------------------------------------
// 9. LCD  (16x2, I2C backpack)
// -----------------------------------------------------------------------------
#define LCD_I2C_ADDR              0x27   // [HW] 0x27 or 0x3F; run an I2C scan
#define LCD_COLS                  16
#define LCD_ROWS                  2
#define LCD_REFRESH_MS            100    // [TUNE] redraw only on change, and no
                                         //        faster than this
#define LCD_PAIRING_DOT_MS        400    // [TUNE] "" -> "." -> ". ." -> ". . ." -> ""
// Row 0 : left = arm state, right = ammo.  Row 1 : left = link mode, right = status
// 16 columns is tight: "DISARMED" + "AMMO COUNT: xx" is 22 chars and cannot fit.
// Strings below are the default compromise — edit freely, keep each row <= 16.
#define LCD_TXT_ARMED             "ARMED"
#define LCD_TXT_DISARMED          "DISARMED"
#define LCD_TXT_AMMO_PREFIX       "AMMO:"      // row 0 right-justified, "AMMO:15"
#define LCD_TXT_MODE_BT           "BT"
#define LCD_TXT_MODE_WIFI         "WIFI"
#define LCD_TXT_CONNECTED         "Connected"
#define LCD_TXT_PAIRING           "Pairing"    // dots appended, spaces between
#define LCD_TXT_NO_CONNECTION     "No Link"
#define LCD_TXT_OFF_ROW0          "SYSTEM OFF"
#define LCD_TXT_OFF_ROW1          ""
#define LCD_TXT_CALIBRATING       "Calibrating"
#define LCD_TXT_FAULT_STALL       "FAULT: STALL"
#define LCD_TXT_FAULT_CAL         "FAULT: CAL"
#define LCD_TXT_SHOW_FIRE_MODE    true   // [CHOICE] append " S"/" F" after ARMED
#define LCD_BACKLIGHT_OFF_WHEN_SYSTEM_OFF true // [CHOICE]

// -----------------------------------------------------------------------------
// 10. CONTROLLER / LINK
// -----------------------------------------------------------------------------
#define CONTROL_PACKET_HZ          100   // [TUNE] ESP32 -> RA4M1 send rate
#define LINK_TIMEOUT_MS            250   // [TUNE] no packet for this long =
                                         //        controller lost -> failsafe
#define LINK_RECONNECT_GRACE_MS   3000   // [TUNE] after a drop, stay in
                                         //        "Pairing" this long before
                                         //        showing "No Link"
#define BUTTON_DEBOUNCE_MS          40   // [TUNE] A/B/X/dpad edge filtering
#define TOGGLE_DEBOUNCE_MS          50   // [TUNE] physical toggle switches
#define RUMBLE_STRONG_255          200   // [TUNE] per-shot rumble, 0..255
#define RUMBLE_WEAK_255            120   // [TUNE]
#define RUMBLE_DURATION_MS          80   // [TUNE] per-shot rumble length
#define RUMBLE_FULL_AUTO_CONTINUOUS true // [CHOICE] keep rumbling while cycling
#define JOY_AXIS_RANGE             512   // [HW] Bluepad32 axis range is -512..511
#define TRIGGER_AXIS_RANGE        1023   // [HW] Bluepad32 trigger range 0..1023
#define JOY_INVERT_Y              true   // [HW] Bluepad32 reports stick-up as
                                         //      negative; true makes up = +tilt

// Failsafe on controller loss --------------------------------------------------
#define FAILSAFE_DISARM_ON_LOSS    true  // [CHOICE]
#define FAILSAFE_STOP_FLYWHEEL     true  // [CHOICE]

// -----------------------------------------------------------------------------
// 11. ESP32-S3 RADIO FIRMWARE  (mirrored into the ESP32 build's config)
// -----------------------------------------------------------------------------
#define WIFI_SSID                 ""            // [HW] station mode credentials. Empty (owner,
                                                //      2026-11-03) = never join a network: the
                                                //      turret goes straight to its own AP below
#define WIFI_PASSWORD             ""            // [HW]
#define WIFI_MODE_AP_FALLBACK     true          // [CHOICE] host own AP if STA fails
#define WIFI_AP_SSID              "D-SENTRY"
#define WIFI_AP_PASSWORD          "brockpurdy"  // [HW] >= 8 chars (owner, 2026-11-03)
#define WIFI_UDP_PORT             4210          // [HW] PC relay sends here
#define BLE_DEVICE_NAME           "D-SENTRY"
#define BLE_FORGET_PAIRINGS_ON_BOOT false       // [CHOICE] true wipes bond keys

// -----------------------------------------------------------------------------
// 12. DEBUG / BENCH
// -----------------------------------------------------------------------------
#define DEBUG_SERIAL_BAUD        115200
#define DEBUG_LOG_LEVEL               2   // 0 off, 1 errors, 2 state changes, 3 verbose
#define DEBUG_ACCEPT_USB_CONTROL   true   // [CHOICE] RA4M1 also accepts control
                                          //          packets over USB serial, so
                                          //          the PC relay can drive the
                                          //          turret with no radio at all
#define DEBUG_STATS_INTERVAL_MS    1000   // loop-rate / link-rate printout

// -----------------------------------------------------------------------------
// 13. ADDED DURING THE BUILD
// -----------------------------------------------------------------------------
// Tunables that docs/architecture.md names but sections 1-12 did not have. Same
// rules as above: [TUNE] adjust on the bench, [HW] must match the hardware,
// [CHOICE] behaviour switch.

// RA4M1 <-> ESP32 AT link (Serial2, docs/architecture.md 4.2) -----------------
#define AT_REPLY_TIMEOUT_MS         20   // [TUNE] reply deadline, counted from the
                                         //        request's last byte on the wire
#define AT_SIZED_MAX_DIGITS          7   // [HW] longest "<len>" before '|' in a
                                         //      sized chAT reply (+XBOXSTATE: 20|)
#define LINK_ESP_PING_MS           100   // [TUNE] "AT" ping interval until the ESP32
                                         //        answers (and after it goes silent)
#define LINK_BEGIN_RETRY_MS       1000   // [TUNE] legacy firmware: least gap between
                                         //        two AT+XBOXBEGIN retries
#define STATUS_HEARTBEAT_MS        500   // [TUNE] AT+DSS / USB StatusPacket repeat
                                         //        when nothing changed (spec 5)
#define LINK_TX_BUFFER_BYTES       256   // [TUNE] paced Serial2 TX ring; must hold
                                         //        one AT+DST line (229 bytes)
#define LINK_RX_PAYLOAD_BYTES       64   // [TUNE] largest AT reply payload kept
                                         //        (AT+XBOXSTATE? carries 20)

// USB serial: telemetry, debug text and the bench shell (4.3) -----------------
#define TELEMETRY_USB_HZ            20   // [TUNE] TelemetryPackets/s on USB (110 B
                                         //        each = 19 % of 115200 baud)
#define TELEMETRY_WIFI_HZ           10   // [TUNE] AT+DST lines/s in WiFi mode (229 B
                                         //        each on the 115200 AT link)
#define DEBUG_TX_BUFFER_BYTES     1024   // [TUNE] paced USB TX ring; a message that
                                         //        does not fit is dropped whole
#define DEBUG_LOG_LINE_MAX         128   // [TUNE] longest debug text line, chars
                                         //        (longer lines are cut). At least 110:
                                         //        the servo adjust summary (uptime + both
                                         //        #define lines) must fit one line
#define SHELL_LINE_MAX              48   // [TUNE] longest shell command; longer
                                         //        lines are dropped and counted
#define SHELL_QUEUE_LINES            4   // [TUNE] complete shell lines ControlLink keeps until
                                         //        Debug takes them (one per loop); more are
                                         //        dropped and counted (LinkStats.shellQueueFull)

// Step generator (GPT timers, hal_arduino.cpp; architecture.md 3) -------------
#define STEP_TIMER_PAN_CHANNEL       0   // [HW] GPT0 (32-bit), PWM-reserved for D5,
                                         //      claimed by force in setup()
#define STEP_TIMER_TILT_CHANNEL      1   // [HW] GPT1 (32-bit), PWM-reserved for D3
#define STEP_TIMER_IRQ_PRIORITY      4   // [HW] above millis (8), UART/I2C (12);
                                         //      0 = most urgent, 15 = least
#define STEP_PULSE_US                3   // [HW] STEP high time; A4988 needs >= 1 us,
                                         //      DRV8825 >= 1.9 us
#define STEP_MAX_RATE_HZ        3000.0f  // [TUNE] safety cap: no axis is ever stepped
                                         //        faster (pan max is 2000 steps/s)
#define STEP_MIN_RATE_HZ          25.0f  // [TUNE] slower requests stop the axis; this
                                         //        bounds how late a buffered speed
                                         //        change can land (1/25 s)
#define STEP_JITTER_LIMIT_PERMILLE 100   // [TUNE] acceptance 17: step jitter must stay
                                         //        under 10 % (stats line flags it)
#define PACKET_LOSS_LIMIT_PERMILLE  10   // [TUNE] acceptance 17: control packet loss must stay
                                         //        under 1 % (stats line flags it)

// Motion planner (Motion.cpp) -------------------------------------------------
#define MOTION_CREEP_STEPS_PER_S   40.0f // [TUNE] final approach of every position move (go-home,
                                         //        shell pan/tilt, stall back-off, calibration):
                                         //        the last MOTION_BRAKE_MARGIN_STEPS steps run at
                                         //        this rate. Must be above STEP_MIN_RATE_HZ (slower
                                         //        stops the axis short of its target) and below
                                         //        CONTROL_LOOP_HZ (at most one step between two
                                         //        planner ticks, so the move stops on its step)
#define MOTION_BRAKE_MARGIN_STEPS     2  // [TUNE] distance the braking planner keeps in reserve
                                         //        before a soft limit or a move target: one step
                                         //        for the buffered rate change (the old rate runs
                                         //        until its next step) and one for the step
                                         //        counter's whole-step resolution
#define MOTION_BRAKE_PLAN_FRACTION 0.90f // [TUNE] the planner plans every braking curve (soft
                                         //        limits, move targets) for this fraction of
                                         //        the deceleration it may use, so the accel
                                         //        limiter keeps the rest to pull the speed back
                                         //        onto the curve after loop jitter or a late
                                         //        step. 1 = no slack (the axis then drifts past
                                         //        its curve and leans on the hard bound)

// Flywheels (Flywheel.cpp) ----------------------------------------------------
#define FLYWHEEL_BRAKE_MS         1000   // [TUNE] FLYWHEEL_BRAKE_ON_STOP only: how long ENA is
                                         //        held at PWM_MAX with IN1=IN2=HIGH (the L298N
                                         //        brakes only while ENA is high), then coast

// LED module ------------------------------------------------------------------
#define LED_SHOW_GUARD_US          300   // [TUNE] show() masks interrupts ~241 us, so
                                         //        it runs only when no step edge is
                                         //        due within this window
#define LED_SHOW_MAX_DEFER_MS      100   // [TUNE] no such window for this long ->
                                         //        show anyway (counted as forced)

// LCD -------------------------------------------------------------------------
#define LCD_I2C_ADDR_FALLBACK     0x3F   // [HW] probed when LCD_I2C_ADDR is silent
#define LCD_I2C_CLOCK_HZ        100000   // [HW] PCF8574 backpacks are rated 100 kHz
#define LCD_I2C_TIMEOUT_US        3000   // [TUNE] a stuck I2C bus costs at most this
                                         //        per transaction
#define LCD_RETRY_MS               500   // [TUNE] after a failed I2C transaction (LCD unplugged,
                                         //        NAK) the LCD is left alone this long; the next
                                         //        transaction starts its re-initialisation
#define LCD_TXT_FIRE_SEMI         " S"   // [CHOICE] suffix after LCD_TXT_ARMED in semi auto
                                         //          (spec 3.9; shown only if it fits)
#define LCD_TXT_FIRE_FULL         " F"   // [CHOICE] suffix after LCD_TXT_ARMED in full auto
#define LCD_TXT_FAULT_OTHER    "FAULT"   // [CHOICE] row 0 in FAULT for a fault code with no
                                         //          text of its own (should never show)

// Stall detector ----------------------------------------------------------------
#define STALL_LOG_HZ                20   // [TUNE] level-3 sample/baseline log lines/s

// Servo adjust mode (TurretState; Menu from DISARMED enters it, Menu again saves and leaves) ---
// The owner sets the trigger servo's REST and FIRE angles with the pad. A hobby servo reports no
// position, so what is recorded is the angle being commanded. Owner, 2026-11-03 (overrides the
// spec's "settings RAM only"): leaving with Menu saves the angles in the RA4M1's data flash, and
// every boot then uses them in place of TRIGGER_REST_ANGLE / TRIGGER_FIRE_ANGLE (the boot log
// says which). View in adjust mode puts config.h's two angles back in force (Menu saves them).
// The turret still logs ready-to-paste "#define TRIGGER_REST_ANGLE n" / FIRE lines.
#define SERVO_ADJUST_STEP_SMALL_DEG  1   // [TUNE] d-pad up / down nudge, whole servo degrees
#define SERVO_ADJUST_STEP_LARGE_DEG  5   // [TUNE] RB / LB nudge, whole servo degrees
#define SERVO_ADJUST_MIN_DEG         0   // [TUNE] adjust mode never commands below this (>= 0);
                                         //        raise it if the pusher meets a hard stop
#define SERVO_ADJUST_MAX_DEG SERVO_SPAN_DEG // [TUNE] ... nor above this (<= SERVO_SPAN_DEG)
#define LCD_TXT_SERVO_ADJUST "SERVO ADJUST" // [CHOICE] row 0 left in adjust mode; the commanded
                                            //          angle right-justified ("SERVO ADJUST 180")
#define LCD_TXT_SERVO_REST        "R:"   // [CHOICE] row 1 in adjust mode, the angles in force:
#define LCD_TXT_SERVO_FIRE        "F:"   // [CHOICE] "R:180 F:25"
#define LED_COLOR_SERVO_ADJUST 0x00FFFF  // [CHOICE] cyan while in adjust mode (under
                                         //          LED_MAX_BRIGHTNESS; it wins over the empty-
                                         //          magazine flash, only FAULT wins over it)

// ESP32-S3 radio firmware (mirrored by tools/sync_config.py) --------------------
#define ESP_RESTART_DELAY_MS        50   // [TUNE] after replying espMode=3 the ESP32
                                         //        waits this long, then restarts
#define WIFI_STA_CONNECT_TIMEOUT_MS 10000 // [TUNE] station join attempt before the AP
                                          //        fallback (WIFI_MODE_AP_FALLBACK)
#define WIFI_STA_RETRY_MS         30000  // [TUNE] gap between station join attempts
                                         //        while not joined
#define WIFI_AP_IP          "192.168.4.1"   // [HW] own-AP address, set explicitly
                                            //      with WiFi.softAPConfig
#define WIFI_AP_NETMASK     "255.255.255.0" // [HW]
#define WIFI_AP_CHANNEL              6   // [CHOICE] 2.4 GHz channel of the own AP
#define WIFI_DISCOVERY_PORT       4211   // [HW] BeaconPacket broadcast port (the PC
                                         //      relay listens here)
#define WIFI_BEACON_MS            1000   // [TUNE] BeaconPacket interval in WiFi mode
#define WIFI_TASK_POLL_MS            5   // [TUNE] UDP receive timeout of the ESP32's WiFi task
                                         //        loop: worst-case delay before a StatusPacket
                                         //        or an AT+DST frame goes out to the relay
#define ESP_WIFI_TASK_STACK_BYTES 6144   // [HW] stack of the D-Sentry WiFi task (WiFi.begin,
                                         //      lwIP sockets); the research prototype ran in 6 KB
#define ESP_WIFI_TASK_PRIORITY       1   // [HW] FreeRTOS priority, same as the AT task and the
                                         //      Xbox client task
#define ESP_WIFI_TASK_CORE           0   // [HW] core 0 with the AT task and the WiFi driver;
                                         //      core 1 runs the USB passthrough loop

// PC relay (relay-pc/dsentry_relay.py) -----------------------------------------
#define RELAY_PAD_POLL_HZ          200   // [TUNE] SDL event pump rate (>= 200 Hz)
#define RELAY_SERIAL_WRITE_TIMEOUT_MS 50 // [TUNE] pyserial write_timeout; any error
                                         //        closes and reopens the port
#define RELAY_REOPEN_MS           1000   // [TUNE] serial / UDP reopen interval
#define RELAY_STATS_MS            1000   // [TUNE] packet-rate printout interval
#define RELAY_SOURCE_CHANGE_HOLD_MS 500  // [TUNE] after the relay's input source changes (pad
                                         //        <-> keyboard, or either appearing or going)
                                         //        it sends connected=0 this long, so the RA4M1
                                         //        runs its link-loss failsafe and comes back
                                         //        DISARMED; must be > LINK_TIMEOUT_MS

// Dashboard keyboard control (relay-pc/web/index.html) -------------------------
#define KEYBOARD_DEFLECTION       0.50f  // [TUNE] stick fraction for a key without
                                         //        Shift (Shift = full deflection)
#define KEYBOARD_PRESS_MS          100   // [TUNE] shortest press a toggle key makes,
                                         //        > BUTTON_DEBOUNCE_MS
#define KEYBOARD_REVOKE_MS        2000   // [TUNE] no POST for this long -> the
                                         //        keyboard session is revoked
#define KEYBOARD_POST_HZ            30   // [TUNE] page -> relay key state rate
#define KEYBOARD_PRESS_QUEUE_MAX     4   // [TUNE] most queued presses per toggle key; a burst
                                         //        of taps faster than KEYBOARD_PRESS_MS presses
                                         //        is cut to this many

// Dashboard page ------------------------------------------------------------------
#define DASHBOARD_HTTP_PORT       8766   // [CHOICE] 8765 is tools/dashboard.py's
#define DASHBOARD_MAX_FPS           30   // [TUNE] SSE telemetry frames/s per client
#define DASHBOARD_KEEPALIVE_MS    1000   // [TUNE] SSE ping on an idle stream
#define DASHBOARD_EVENT_LOG_KEEP    50   // [TUNE] event log entries kept
#define DASHBOARD_STALE_MS         500   // [TUNE] no TelemetryPacket for this long -> the page
                                         //        marks every turret value STALE

// -----------------------------------------------------------------------------
// 14. FIXED CONSTANTS (units and hardware facts — not tunables, do not edit)
// -----------------------------------------------------------------------------
// Named here so no D-Sentry source writes a bare number (architecture.md rule 1).

// Units -------------------------------------------------------------------------
#define MS_PER_S                1000UL
#define US_PER_MS               1000UL
#define US_PER_S             1000000UL
#define DEG_PER_REV             360.0f
#define PERCENT                    100   // a fraction of 1 as a percentage
#define PERMILLE                  1000   // a fraction of 1 in thousandths
#define DEC_BASE                    10   // decimal digits (number formatting)
#define HEX_BASE                    16   // hex digits
#define HEX_DIGIT_BITS               4   // bits per hex digit
#define BITS_PER_BYTE                8
#define BYTE_MASK                 0xFFu
#define MATH_HALF                 0.5f
#define MATH_TWO_PI        6.28318531f   // one full turn in radians
#define KINEMATICS_2AD_FACTOR     2.0f   // the 2 in v^2 = 2*a*d (stopping distance)

// Servo, PWM, LEDs, ADC ---------------------------------------------------------
#define SERVO_SPAN_DEG             180   // Servo.h: 0..180 deg = MIN_US..MAX_US
#define SERVO_PERIOD_US          20000UL // 50 Hz servo frame
#define PWM_MAX                    255   // analogWrite() at the default 8 bits
#define LED_CHANNEL_MAX            255   // one WS2812 colour channel
#define LED_BRIGHTNESS_LIMIT       127   // spec 3.8 / acceptance 16: the ceiling
                                         // LED_MAX_BRIGHTNESS may never exceed
#define COLOR_SHIFT_R               16   // 0xRRGGBB
#define COLOR_SHIFT_G                8
#define COLOR_BYTE_MASK           0xFF
#define ADC_RESOLUTION_BITS         10   // analogReadResolution(); 0..ADC_MAX_COUNTS
#define ADC_MAX_COUNTS            1023

// UARTs (core arduino:renesas_uno 1.6.0) ------------------------------------------
#define UART_BITS_PER_BYTE          10   // 8N1: start + 8 data + stop
#define SERIAL_RX_BUFFER_BYTES     512   // core RX ring per UART; drops when full
#define SERIAL_TX_SLACK_BYTES        1   // Serial (SCI9, no FIFO): bytes the UART
                                         // takes without UART::write() blocking
#define SCI_FIFO_DEPTH              16   // Serial2 (SCI1) TX FIFO depth

// Clocks and registers (RA4M1) ----------------------------------------------------
#define CPU_HZ               48000000UL  // ICLK; also the DWT cycle counter rate
#define GPT_CLOCK_HZ         48000000UL  // PCLKD, GPT prescaler /1
#define RA_IO_PORT_SHIFT             8   // bsp_io_port_pin_t = port << 8 | pin
#define RA_PCNTR3_RESET_SHIFT       16   // PCNTR3: bits 15:0 set, bits 31:16 clear

// HD44780 LCD behind a PCF8574 I2C backpack ---------------------------------------
#define LCD_PCF_RS                0x01   // P0: register select (1 = data)
#define LCD_PCF_RW                0x02   // P1: read/write (always 0 = write)
#define LCD_PCF_EN                0x04   // P2: enable strobe
#define LCD_PCF_BL                0x08   // P3: backlight
#define LCD_NIBBLE_MASK           0xF0   // P4-P7 carry D4-D7
#define LCD_NIBBLE_SHIFT             4
#define HD44780_CMD_SET_DDRAM     0x80   // set-cursor command
#define HD44780_ROW1_DDRAM        0x40   // DDRAM address of row 1, column 0
// Re-initialisation after a failed transaction (datasheet "initializing by instruction",
// 4-bit interface; the same sequence LiquidCrystal_I2C::init() sends in setup()):
#define HD44780_POWER_ON_MS         50   // wait after power returns (datasheet: > 40 ms)
#define HD44780_RESET_NIBBLE      0x30   // "function set, 8-bit", sent as one nibble ...
#define HD44780_RESET_REPEATS        3   // ... this many times
#define HD44780_RESET_FIRST_US    4500   // wait after the first one (datasheet: > 4.1 ms)
#define HD44780_RESET_NEXT_US      150   // wait after the others (datasheet: > 100 us)
#define HD44780_4BIT_NIBBLE       0x20   // "function set, 4-bit", one nibble
#define HD44780_CMD_FUNCTION_2LINE 0x28  // 4-bit, 2 lines, 5x8 dots
#define HD44780_CMD_DISPLAY_ON    0x0C   // display on, cursor off, blink off
#define HD44780_CMD_CLEAR         0x01   // clear, cursor to (0,0)
#define HD44780_CLEAR_US          2000   // clear's execution time (datasheet: 1.52 ms)
#define HD44780_CMD_ENTRY_INC     0x06   // entry mode: cursor moves right, no shift
#define HD44780_CMD_US             100   // other commands' execution time (datasheet: 37 us)

// SERVO ADJUST's record in the RA4M1's 8 KB data flash (core EEPROM library, hal_arduino.cpp):
// magic (2 bytes, low first), version, REST, FIRE (whole degrees), CRC-8 (ds_crc8) of the bytes
// before it. Blank flash reads 0xFF; blank or any mismatch means config.h's angles at boot.
#define SETTINGS_STORE_ADDR          0   // [HW] EEPROM.h index of the record's first byte (in the
                                         //      first 1 KB erase block; nothing else uses the store)
#define SETTINGS_STORE_MAX_BYTES    16   // [HW] most bytes one hal::storeSave takes (its buffer)
#define SERVO_STORE_MAGIC       0xD5A7   // marks a record written by this firmware
#define SERVO_STORE_VERSION          1   // the record layout above; any other value is not read

// ESP32-S3 radio firmware (mirrored by tools/sync_config.py) ----------------------
#define ESP_MODE_REQUEST_MAGIC 0xD5E27701u // marks the RTC_NOINIT mode request as written by
                                           // this firmware; any value power-on garbage is
                                           // unlikely to hold

// AT+XBOXSTATE? 20-byte payload (protocol/xbox_r4_protocol.h comments) ---------------
#define XSTATE_OFS_JOY_LH            0   // u16 LE left stick X, 0 = left
#define XSTATE_OFS_JOY_LV            2   // u16 LE left stick Y, 0 = up
#define XSTATE_OFS_JOY_RH            4   // u16 LE right stick X, 0 = left
#define XSTATE_OFS_JOY_RV            6   // u16 LE right stick Y, 0 = up
#define XSTATE_OFS_TRIG_LT           8   // u16 LE left trigger, 0..1023
#define XSTATE_OFS_TRIG_RT          10   // u16 LE right trigger, 0..1023
#define XSTATE_OFS_DPAD             12   // d-pad code, XSTATE_DPAD_*
#define XSTATE_OFS_BTN0             13   // XSTATE_BTN0_* bits
#define XSTATE_OFS_BTN1             14   // XSTATE_BTN1_* bits
#define XSTATE_OFS_BTN2             15   // XSTATE_BTN2_* bits
#define XSTATE_OFS_CONN             16   // enum xbox_r4_conn_state, 0 = connected
#define XSTATE_OFS_BATTERY          17   // battery percent 0..100, 0xFF unknown
#define XSTATE_OFS_SEQ              18   // HID report counter, low byte
#define XSTATE_BTN0_A             0x01
#define XSTATE_BTN0_B             0x02
#define XSTATE_BTN0_X             0x08
#define XSTATE_BTN0_Y             0x10
#define XSTATE_BTN0_LB            0x40
#define XSTATE_BTN0_RB            0x80
#define XSTATE_BTN1_VIEW          0x04   // View = back
#define XSTATE_BTN1_MENU          0x08   // Menu = start
#define XSTATE_BTN1_XBOX          0x10   // Xbox (guide) button
#define XSTATE_BTN1_LS            0x20   // left stick click
#define XSTATE_BTN1_RS            0x40   // right stick click
#define XSTATE_BTN2_SHARE         0x01   // Share button
#define XSTATE_DPAD_NONE             0
#define XSTATE_DPAD_N                1
#define XSTATE_DPAD_NE               2
#define XSTATE_DPAD_E                3
#define XSTATE_DPAD_SE               4
#define XSTATE_DPAD_S                5
#define XSTATE_DPAD_SW               6
#define XSTATE_DPAD_W                7
#define XSTATE_DPAD_NW               8
#define XSTATE_AXIS_CENTRE       32768   // stick centre in 0..65535
#define XSTATE_AXIS_SHIFT            6   // (raw - centre) >> 6 = -512..511
