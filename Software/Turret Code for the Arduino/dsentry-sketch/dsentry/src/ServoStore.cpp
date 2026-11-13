// ServoStore.cpp -- see ServoStore.h.
//
// SPDX-License-Identifier: MIT
#include "ServoStore.h"

#include "hal.h"

static_assert(servostore::REC_BYTES <= SETTINGS_STORE_MAX_BYTES,
              "the SERVO ADJUST record must fit SETTINGS_STORE_MAX_BYTES");
static_assert(SERVO_SPAN_DEG <= BYTE_MASK, "an angle is stored in one byte");
static_assert(SERVO_STORE_VERSION <= BYTE_MASK, "the record version is one byte");
static_assert(SERVO_STORE_MAGIC <= (BYTE_MASK << BITS_PER_BYTE | BYTE_MASK), "the magic is two bytes");

namespace servostore {

bool validAngle(int32_t deg) { return deg >= 0 && deg <= SERVO_SPAN_DEG; }

void encode(const Angles& a, uint8_t out[REC_BYTES]) {
  out[REC_MAGIC_LO] = (uint8_t)(SERVO_STORE_MAGIC & BYTE_MASK);
  out[REC_MAGIC_HI] = (uint8_t)((SERVO_STORE_MAGIC >> BITS_PER_BYTE) & BYTE_MASK);
  out[REC_VERSION] = (uint8_t)SERVO_STORE_VERSION;
  out[REC_REST] = (uint8_t)a.rest;
  out[REC_FIRE] = (uint8_t)a.fire;
  out[REC_CRC] = ds_crc8(out, REC_CRC);
}

Load decode(const uint8_t in[REC_BYTES], Angles& out) {
  bool blank = true;
  for (uint8_t i = 0; i < REC_BYTES; i++) blank = blank && in[i] == BYTE_MASK;
  if (blank) return Load::BLANK;
  const uint16_t magic = (uint16_t)(in[REC_MAGIC_LO] | in[REC_MAGIC_HI] << BITS_PER_BYTE);
  if (magic != SERVO_STORE_MAGIC || in[REC_VERSION] != SERVO_STORE_VERSION || ds_crc8(in, REC_CRC) != in[REC_CRC] ||
      !validAngle(in[REC_REST]) || !validAngle(in[REC_FIRE]))
    return Load::INVALID;
  out.rest = in[REC_REST];
  out.fire = in[REC_FIRE];
  return Load::OK;
}

Load load(Angles& out) {
  uint8_t rec[REC_BYTES];
  if (!hal::storeLoad(rec, sizeof rec)) return Load::INVALID;
  return decode(rec, out);
}

bool save(const Angles& a) {
  if (!validAngle(a.rest) || !validAngle(a.fire)) return false;
  uint8_t rec[REC_BYTES];
  encode(a, rec);
  return hal::storeSave(rec, sizeof rec);
}

const char* loadName(Load l) {
  switch (l) {
    case Load::OK: return "ok";
    case Load::BLANK: return "blank";
    case Load::INVALID: return "invalid";
  }
  return "?";
}

}  // namespace servostore
