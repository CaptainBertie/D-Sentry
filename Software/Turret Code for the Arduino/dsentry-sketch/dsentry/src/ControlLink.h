// ControlLink.h -- every control source and the AT link to the ESP32 (docs/architecture.md
// 4.2, 4.3, 5).
//
// Owns:
//   - the USB byte stream: a ds_deframer for ControlPackets (obeyed while usbControlEnabled(),
//     default DEBUG_ACCEPT_USB_CONTROL) and the shell line splitter. Every received byte goes
//     to the deframer (polled until DS_PKT_NONE after each feed); the bytes the deframer drops
//     as "not part of a frame", and only those, go on to the line splitter, in order, so a
//     frame's bytes (its len byte 0x0A is '\n') can never cut or corrupt a shell line. Lines
//     are '\n'-terminated, '\r' is ignored, at most SHELL_LINE_MAX characters (longer ones are
//     dropped and counted); a line holding any non-text byte is binary residue and dropped
//     (counted). Complete lines wait in a queue of SHELL_QUEUE_LINES for takeShellLine().
//   - the AT client (AsyncAt + scheduling), one request outstanding:
//       PING        "AT" every LINK_ESP_PING_MS until any final arrives (OK or ERROR: both
//                   prove the ESP32 is alive and its line buffer is in sync), then
//       CAPS        AT+DSCAPS?, asked until two answers in a row agree (AT text has no CRC):
//                   "+DSCAPS: <dsProto>,<capBits>" with dsProto == DS_PROTO_VERSION -> RUN_DS;
//                   a bare ERROR (stock bridge / pre-D-Sentry fork) or another dsProto ->
//                   RUN_LEGACY; anything else retried every LINK_ESP_PING_MS.
//       RUN_DS      each tick, by priority: AT+DSS when its content differs from the last one
//                   the ESP32 acknowledged, on request (new bootId, or XBOXSTATE OFF while BLE
//                   is wanted, the latter at most every LINK_BEGIN_RETRY_MS), or every
//                   STATUS_HEARTBEAT_MS; then AT+XBOXSTATE? on a 1000/CONTROL_PACKET_HZ ms grid;
//                   then AT+DST every 1000/TELEMETRY_WIFI_HZ ms while a frame is on offer, the
//                   ESP32 reports espMode WIFI and has DS_CAP_TELEMETRY. A failed DSS waits one
//                   poll period before its retry so the control polls keep flowing. A DSS reply
//                   outside the enums is a failure; one that changes espMode or bootId is
//                   asked again at once and adopted only when the repeat agrees.
//       RUN_LEGACY  AT+XBOXSTATE? on the same grid; AT+XBOXBEGIN when a state frame says OFF,
//                   at most every LINK_BEGIN_RETRY_MS. BT only: with WiFi wanted the radio never
//                   connects, and an error is logged once per mode change.
//     ESP32 silent (no final of any kind) for LINK_TIMEOUT_MS -> back to PING and CAPS.
//     Request deadline: AT_REPLY_TIMEOUT_MS plus the wire time of everything queued ahead of
//     and including the request, and never less than AT_REPLY_TIMEOUT_MS after its last byte
//     left the UART (AsyncAt::extend), so a 229-byte AT+DST line is not timed out by its own
//     transmission.
//   - XBOXSTATE -> ControlFrame (architecture.md 4.2): rx = (joyRHori - 32768) >> 6,
//     ry = (joyRVert - 32768) >> 6 (0 = up -> negative), rt = trigRT, A/B/X/Y/LB/RB from byte
//     13, start = Menu (byte 14 bit 3), back = View (byte 14 bit 2), d-pad code -> the four
//     d-pad bits, connected = (state byte == 0), espPairing = state 1..3 (waiting / found /
//     scanning), seq = byte 18. For the dashboard only (TelemetryPacket controller block):
//     lx / ly the same way from joyLHori / joyLVert, lt = trigLT, LS / RS click and Xbox from
//     byte 14 bits 5 / 6 / 4, Share from byte 15 bit 0, battery = byte 17, padState = the state
//     byte 16. USB ControlPackets: lx / ly / lt 0, battery unknown, padState 0 when the packet
//     says connected, else 3. A bad CRC is counted in crcErrors and the frame dropped. Radio
//     frames are obeyed only while the ESP32 runs the wanted radio (D-Sentry firmware: espMode
//     matches wantMode; legacy: BLE wanted), so a mode change drops the radio link at once.
//   - source priority: a valid USB ControlPacket within LINK_TIMEOUT_MS makes USB the active
//     source (bench override, even when its flags say not connected), otherwise a fresh radio
//     frame, otherwise none. A frame older than LINK_TIMEOUT_MS is never "connected". A change
//     between two sources while connected counts as a link loss (takeSourceChanged(),
//     LinkStats.sourceChanges).
//   - USB StatusPackets: when their content changes and every STATUS_HEARTBEAT_MS.
//   - stats (LinkStats) and status (LinkStatus), refreshed every update().
// It never decides turret behaviour: TurretState reads frame()/status() and calls the setters.
//
// Integration notes (the next stage wires these; none of them is called yet):
//   App::loop / Debug::update: while (link.takeShellLine(buf, sizeof buf))
//                                debug.handleShellLine(buf);
//   TurretState: setWantMode() on the A1 toggle, setTurretState() every update,
//                requestRumble() per stroke, takeSourceChanged() as a link loss.
//   Telemetry:   offerTelemetry(frame, len) for each USB TelemetryPacket it builds.
//   Debug shell: setUsbControlEnabled() if a command wants to lock USB control out.
//
// SPDX-License-Identifier: MIT
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "AsyncAt.h"
#include "Types.h"

class Debug;

class ControlLink {
 public:
  // Phase of the AT client, for Debug and the tests.
  enum class AtPhase : uint8_t {
    PING,       // sending "AT" every LINK_ESP_PING_MS until any final arrives
    CAPS,       // probing AT+DSCAPS?
    RUN_DS,     // D-Sentry firmware: DSS / XBOXSTATE / DST schedule
    RUN_LEGACY  // stock / old fork: XBOXSTATE (+ XBOXBEGIN when OFF), BT only
  };
  // The AT request outstanding.
  enum class Request : uint8_t { NONE, PING, CAPS, DSS, STATE, DST, BEGIN };

  ControlLink();
  // Not copyable: at_ points into this object's own atPayload_.
  ControlLink(const ControlLink&) = delete;
  ControlLink& operator=(const ControlLink&) = delete;
  // Resets everything except the USB-control flag; the first ping goes out on update().
  void begin(Debug& debug, uint32_t nowMs, uint32_t nowUs);

  // Once per loop(), first: reads USB and link bytes, advances the AT client, refreshes
  // frame() / status() / stats(), queues a due StatusPacket. Never blocks.
  void update(uint32_t nowMs, uint32_t nowUs);

  // ---- outputs ----
  const ControlFrame& frame() const { return frame_; }   // the active source's latest input
  const LinkStatus& status() const { return status_; }
  const LinkStats& stats() const { return stats_; }
  bool connected() const { return status_.connected; }
  AtPhase atPhase() const { return atPhase_; }
  Request outstanding() const { return req_; }
  // The active source changed while connected since the last call (cleared by reading).
  bool takeSourceChanged();
  uint8_t rumbleSeq() const { return rumbleSeq_; }

  // ---- inputs from TurretState / Telemetry / Debug ----
  // DS_RADIO_*; out-of-range values are ignored. A change voids the radio frame at once (the
  // radio is obeyed again only when the ESP32 reports the new mode) and goes out in AT+DSS.
  void setWantMode(uint8_t radioMode);
  void setTurretState(uint8_t dsState);   // DS_STATE_*, carried in AT+DSS / StatusPacket
  // One rumble command (per stroke). Increments rumbleSeq so a heartbeat repeat is not a shot.
  void requestRumble(uint8_t strong, uint8_t weak, uint16_t ms);
  // A complete TelemetryPacket frame (start..crc, DS_TELEMETRY_FRAME_LEN bytes, CRC valid).
  // Returns true when it was kept as the next AT+DST payload (WiFi path open, see
  // wantsWifiTelemetry()); false when invalid or there is no WiFi path. ControlLink alone
  // decides when the kept frame goes out (TELEMETRY_WIFI_HZ); a newer offer replaces it.
  bool offerTelemetry(const uint8_t* frame, size_t len);
  bool sendTelemetry(const uint8_t* frame, size_t len) { return offerTelemetry(frame, len); }
  // D-Sentry firmware, espMode WIFI and DS_CAP_TELEMETRY: offered frames become AT+DST.
  bool wantsWifiTelemetry() const;
  // The oldest complete shell line (no newline, NUL-terminated, cut to cap - 1 chars); false
  // when none is waiting.
  bool takeShellLine(char* out, size_t cap);
  size_t shellLinesWaiting() const { return shellCount_; }
  // USB ControlPackets obeyed (default DEBUG_ACCEPT_USB_CONTROL). Off: they are still
  // deframed (and counted in usbIgnored) but never become the active source.
  void setUsbControlEnabled(bool on);
  bool usbControlEnabled() const { return usbControlEnabled_; }

  // XBOXSTATE payload -> ControlFrame (valid, connected, espPairing, rx, ry, rt, buttons,
  // seq, lx, ly, lt, battery, padState; source and rxMs are left for the caller). False when
  // len != XBOX_R4_STATE_LEN or the CRC is bad. connState: byte 16 (enum xbox_r4_conn_state).
  // Public for the tests.
  static bool decodeXboxState(const uint8_t* payload, size_t len, ControlFrame& out,
                              uint8_t& connState);
  static const char* phaseName(AtPhase p);
  static const char* requestName(Request r);

 private:
  struct SourceFrame {
    bool have = false;      // a frame arrived and is not yet older than LINK_TIMEOUT_MS
    uint32_t rxMs = 0;
    ControlFrame f;
  };

  // USB
  void usbByte(uint8_t b, uint32_t nowMs);
  void shellByte(uint8_t c);
  void usbFrame(uint8_t type, const uint8_t* frame, size_t len, uint32_t nowMs);
  // AT client
  bool issue(Request kind, const char* prompt, const char* line, size_t len, uint32_t nowUs);
  void schedule(uint32_t nowMs, uint32_t nowUs);
  void onResult(AsyncAt::Result r, uint32_t nowMs);
  void onCaps(AsyncAt::Result r, uint32_t nowMs);
  void onDss(AsyncAt::Result r, uint32_t nowMs);
  void onState(AsyncAt::Result r, uint32_t nowMs);
  void onDst(AsyncAt::Result r);
  void onBegin(AsyncAt::Result r);
  void enterPing(uint32_t nowMs);
  void enterCaps(uint32_t nowMs);
  void enterRunDs(uint32_t nowMs);
  void enterRunLegacy(uint32_t nowMs);
  bool dssDue(uint32_t nowMs) const;
  bool pollDue(uint32_t nowMs) const;
  bool dstDue(uint32_t nowMs) const;
  bool beginDue(uint32_t nowMs) const;
  void sendDss(uint32_t nowMs, uint32_t nowUs);
  void sendPoll(uint32_t nowMs, uint32_t nowUs);
  void sendDst(uint32_t nowMs, uint32_t nowUs);
  void sendBegin(uint32_t nowMs, uint32_t nowUs);
  ds_status_packet_t currentStatus() const;
  bool radioPathOpen() const;
  uint8_t radioSource() const;
  // outputs
  void refreshSources(uint32_t nowMs);
  void refreshWifiAvailability();
  void rateWindow(uint32_t nowMs);
  void sendUsbStatus(uint32_t nowMs);
  void refreshStats(uint32_t nowMs);
  void log(uint8_t level, const char* text);

  Debug* debug_ = nullptr;
  ControlFrame frame_;
  LinkStatus status_;
  LinkStats stats_;
  AtPhase atPhase_ = AtPhase::PING;
  bool usbControlEnabled_ = DEBUG_ACCEPT_USB_CONTROL;

  // USB deframer and its shadow: the held bytes not yet reported as a frame (see usbByte).
  ds_deframer_t usbDeframer_;
  uint8_t shadow_[DS_MAX_FRAME_LEN + 1] = {};
  size_t shadowLen_ = 0;
  // shell line splitter and queue
  char line_[SHELL_LINE_MAX + 1] = {};
  size_t lineLen_ = 0;
  bool lineOverflow_ = false;
  bool linePoison_ = false;
  char shellQ_[SHELL_QUEUE_LINES][SHELL_LINE_MAX + 1] = {};
  size_t shellHead_ = 0;
  size_t shellCount_ = 0;
  uint32_t usbJunk_ = 0;

  // sources
  SourceFrame usb_;
  SourceFrame radio_;
  bool sourceChanged_ = false;
  uint32_t windowStartMs_ = 0;
  uint32_t windowFrames_[DS_LINK_SRC_COUNT] = {};

  // AT client
  uint8_t atPayload_[LINK_RX_PAYLOAD_BYTES] = {};
  AsyncAt at_;
  char reqLine_[DS_AT_DST_LINE_LEN + 1] = {};
  Request req_ = Request::NONE;
  bool reqOnWire_ = false;          // the request's last byte has left the UART
  uint32_t lastEspReplyMs_ = 0;
  bool pingSent_ = false;
  uint32_t lastPingMs_ = 0;
  // The previous AT+DSCAPS? outcome, waiting for an identical one (see onCaps).
  struct CapsOutcome {
    enum Kind : uint8_t { NONE, ANSWER, BARE_ERROR } kind = NONE;
    uint8_t proto = 0;
    uint8_t caps = 0;
  };
  bool capsNow_ = false;
  CapsOutcome capsPrev_;
  uint32_t lastCapsMs_ = 0;
  uint32_t lastPollMs_ = 0;
  // AT+DSS
  ds_status_packet_t dssInFlight_ = {};
  ds_status_packet_t dssAcked_ = {};
  bool dssAckedValid_ = false;
  bool dssFailed_ = false;
  bool resendDss_ = false;
  bool dssConfirming_ = false;      // a reply changed espMode/bootId: waiting for the repeat
  uint8_t dssPendingMode_ = DS_ESP_MODE_UNKNOWN;
  uint8_t dssPendingBoot_ = 0;
  uint32_t lastDssMs_ = 0;
  bool offDssEver_ = false;
  uint32_t lastOffDssMs_ = 0;
  // AT+XBOXBEGIN (legacy)
  bool beginWanted_ = false;
  bool beginEver_ = false;
  uint32_t lastBeginMs_ = 0;
  uint8_t lastBeginCode_ = 0;
  // AT+DST
  bool telemetryPending_ = false;
  uint8_t telemetryFrame_[DS_TELEMETRY_FRAME_LEN] = {};
  uint32_t lastDstMs_ = 0;
  bool wifiWarned_ = false;
  uint32_t xboxCrcErrors_ = 0;

  // StatusPacket content
  uint8_t rumbleStrong_ = 0;
  uint8_t rumbleWeak_ = 0;
  uint16_t rumbleMs_ = 0;
  uint8_t rumbleSeq_ = 0;
  uint8_t turretState_ = DS_STATE_OFF;
  ds_status_packet_t usbStatusSent_ = {};
  bool usbStatusValid_ = false;
  uint32_t lastUsbStatusMs_ = 0;
  bool usbStatusRetry_ = false;
  uint32_t lastUsbStatusTryMs_ = 0;
};
