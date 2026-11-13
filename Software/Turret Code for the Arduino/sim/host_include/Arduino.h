// Arduino.h -- host stand-in for the Arduino core header, so config.h (which includes
// <Arduino.h> for the pin and pixel-order names) compiles with clang++ on the Mac.
//
// Only what config.h needs, with the values of arduino:renesas_uno 1.6.0
// (variants/UNOWIFIR4/pins_arduino.h) and Adafruit NeoPixel 1.15.5 (Adafruit_NeoPixel.h).
// Deliberately NOT here: ARDUINO (it selects hal_arduino.cpp), Serial objects (only
// hal_arduino.cpp names LINK_SERIAL), and every Arduino function: portable modules talk to the
// hardware through hal.h only (architecture.md rule 4), and a module that calls millis() or
// digitalWrite() must fail to compile on the host.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// UNO R4 WiFi analog pins: PIN_A0 (14u) .. PIN_A5 (19u)
static const uint8_t A0 = 14;
static const uint8_t A1 = 15;
static const uint8_t A2 = 16;
static const uint8_t A3 = 17;
static const uint8_t A4 = 18;
static const uint8_t A5 = 19;

// Adafruit NeoPixel colour orders and speed (LED_PIXEL_ORDER in config.h)
#define NEO_RGB ((0 << 6) | (0 << 4) | (1 << 2) | (2))
#define NEO_GRB ((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define NEO_KHZ800 0x0000
