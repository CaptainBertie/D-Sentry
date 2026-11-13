/*
 * WiFiS3.h -- host stand-in for the WiFiS3 library header, used only by the host tests.
 *
 * XboxControllerR4.cpp includes <WiFiS3.h> only to reach the `modem` global, MODEM_TIMEOUT and
 * the CMD / CMD_READ / CMD_WRITE / PROMPT macros, which all come from Modem.h. The real
 * WiFiS3.h also pulls in WiFi, sockets and the rest of the Renesas core, which the host cannot
 * build. run.sh puts tests/stubs before the installed WiFiS3/src on the include path, so this
 * file wins and the real Modem.h (and WiFiCommands.h) are still the ones compiled.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once
#include "Modem.h"
