#include "XboxControllerElite2NotificationParser.h"

#define XBOX_CONTROLLER_INDEX_PROFILE 16
#define XBOX_CONTROLLER_INDEX_PADDLES 18

uint8_t XboxControllerElite2NotificationParser::update(uint8_t *data, size_t length) {
  XboxControllerNotificationParser::update(data, length);
  uint8_t btnBits;

  if (length > XBOX_CONTROLLER_INDEX_PROFILE) {
    profile = data[XBOX_CONTROLLER_INDEX_PROFILE];
  } else {
    profile = 0;
  }

  if (length > XBOX_CONTROLLER_INDEX_PADDLES) {
    btnBits = data[XBOX_CONTROLLER_INDEX_PADDLES];
    btnP1 = btnBits & 0b00000001;
    btnP2 = btnBits & 0b00000010;
    btnP3 = btnBits & 0b00000100;
    btnP4 = btnBits & 0b00001000;
  } else {
    btnP1 = false;
    btnP2 = false;
    btnP3 = false;
    btnP4 = false;
  }
  return 0;
}

String XboxControllerElite2NotificationParser::toString() {
  String str = XboxControllerNotificationParser::toString();
  str += "btnPaddle1: " + String(btnP1) + " " + "btnPaddle2: " + String(btnP2) +
         " " + "btnPaddle3: " + String(btnP3) + " " +
         "btnPaddle4: " + String(btnP4) + "\n" + "profile: " + String(profile) +
         "\n";
  return str;
}

uint8_t XboxControllerElite2NotificationParser::toArr(uint8_t *data, size_t length) {
  XboxControllerNotificationParser::toArr(data, length);

  if (length > XBOX_CONTROLLER_INDEX_PROFILE) {
    data[XBOX_CONTROLLER_INDEX_PROFILE] = profile;
  }

  if (length > XBOX_CONTROLLER_INDEX_PADDLES) {
    uint8_t btnBits = 0;
    if (btnP1) btnBits |= 0b1 << 0;
    if (btnP2) btnBits |= 0b1 << 1;
    if (btnP3) btnBits |= 0b1 << 2;
    if (btnP4) btnBits |= 0b1 << 3;

    data[XBOX_CONTROLLER_INDEX_PADDLES] = btnBits;
  }
  return 0;
}

XboxControllerElite2NotificationParser::XboxControllerElite2NotificationParser()
    : XboxControllerNotificationParser() {
  btnP1 = btnP2 = btnP3 = btnP4 = 0;
  profile = 0;
}
