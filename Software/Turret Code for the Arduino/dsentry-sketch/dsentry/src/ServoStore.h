// ServoStore.h -- SERVO ADJUST's REST / FIRE record in the RA4M1's data flash (owner, 2026-11-03,
// overriding the spec's "settings RAM only"; docs/architecture.md 0 and 6). TurretState is the only
// user: it loads the record once at boot and saves it when SERVO ADJUST is left with Menu.
//
// The record, at SETTINGS_STORE_ADDR through hal::storeLoad / hal::storeSave (config.h section 14):
//   byte 0, 1  SERVO_STORE_MAGIC, low byte first
//   byte 2     SERVO_STORE_VERSION
//   byte 3     REST, whole degrees 0..SERVO_SPAN_DEG
//   byte 4     FIRE, whole degrees 0..SERVO_SPAN_DEG
//   byte 5     ds_crc8() of bytes 0..4 (the protocol's CRC-8)
// Every byte 0xFF is a blank store (never written); anything else that does not check out (magic,
// version, CRC, an angle past SERVO_SPAN_DEG) is INVALID. Both mean config.h's angles.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "Types.h"

namespace servostore {

// The two stroke angles, whole servo degrees in config.h's terms (before TRIGGER_SERVO_REVERSED).
struct Angles {
  int16_t rest = TRIGGER_REST_ANGLE;
  int16_t fire = TRIGGER_FIRE_ANGLE;
};
inline bool sameAngles(const Angles& a, const Angles& b) { return a.rest == b.rest && a.fire == b.fire; }
inline Angles configAngles() { return Angles(); }   // config.h's TRIGGER_REST_ANGLE / _FIRE_ANGLE

enum class Load : uint8_t {
  OK,        // a valid record: `out` holds it
  BLANK,     // every byte 0xFF: never saved
  INVALID    // written, but not a record this firmware wrote (or damaged)
};

// The record's bytes.
enum RecordByte : uint8_t { REC_MAGIC_LO, REC_MAGIC_HI, REC_VERSION, REC_REST, REC_FIRE, REC_CRC, REC_BYTES };

void encode(const Angles& a, uint8_t out[REC_BYTES]);
Load decode(const uint8_t in[REC_BYTES], Angles& out);   // out untouched unless OK
bool validAngle(int32_t deg);                                // 0..SERVO_SPAN_DEG

// hal::storeLoad + decode (a read failure counts as INVALID).
Load load(Angles& out);
// encode + hal::storeSave. BLOCKS for the flash write (hal.h: about 50 ms); true when the record
// read back. The caller (TurretState) only saves in DISARMED, no stroke running, no axis moving.
bool save(const Angles& a);

const char* loadName(Load l);

}  // namespace servostore
