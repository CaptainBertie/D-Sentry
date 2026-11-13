#ifndef __XBOX_CONTROLLER_ELITE2_NOTIFICATION_PARSER_H__
#define __XBOX_CONTROLLER_ELITE2_NOTIFICATION_PARSER_H__
#include "XboxControllerNotificationParser.h"

class XboxControllerElite2NotificationParser
  : public XboxControllerNotificationParser {
 private:
 public:
  XboxControllerElite2NotificationParser();
  uint8_t update(uint8_t *data, size_t length);
  String toString();
  uint8_t toArr(uint8_t *data, size_t length);
  bool btnP1, btnP2, btnP3, btnP4;
  uint8_t profile;
};

#endif
