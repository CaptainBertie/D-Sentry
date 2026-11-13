// sim_main.cpp -- dsentry_sim: the REAL RA4M1 firmware (App and every module of
// firmware-ra4m1/dsentry/src) on the host HAL (sim/hal_host.cpp), in real time or N times faster,
// next to a faithful ESP32 (sim/sim_esp32.cpp: the fork's chAT server and D-Sentry AT handlers)
// on the AT link. docs/architecture.md 8.4.
//
// Wiring inside the simulator:
//   RA4M1 Serial2 TX --(paced at LINK_BAUD)--> ESP32 AT server --(paced)--> RA4M1 Serial2 RX
//   RA4M1 Serial  TX --> stdout (text lines only: Status/Telemetry frames are cut out)
//                    --> the pty (--pty), all bytes, while the ESP32's USB passthrough runs
//   the pty / "shell" on stdin / the ESP32's ROM banner --(paced at DEBUG_SERIAL_BAUD)--> Serial RX
//   ESP32 WiFi mode: UDP 127.0.0.1:<--udp-port> (ControlPackets in; StatusPackets and the
//   RA4M1's AT+DST telemetry out to the sender of the last valid ControlPacket)
// Time: virtual. Each App::loop() advances it by --loop-us; loops run whenever virtual time is
// behind the wall clock times --speed. The step model, the UART pacing, the ESP32's 1 ms AT
// task, its millis() and every RA4M1 time stamp use that one clock. If the Mac cannot keep up
// for more than kMaxLagUs the backlog is dropped and counted (lagResets), never replayed in a burst.
//
// Usage: dsentry_sim [options]          (the relay runs: <bin> --udp-port N [--sim-arg ...])
//   --udp-port N       WiFi mode UDP port on 127.0.0.1 (default WIFI_UDP_PORT; 0 = a free one)
//   --speed X          virtual time = X times the wall clock (default 1; >0, at most kMaxSpeed)
//   --pty              expose the RA4M1's USB serial as a pseudo-terminal; its path goes to stderr
//   --pty-link PATH    also a symlink to it at PATH (removed on exit)
//   --seconds S        stop after S virtual seconds (default: run until SIGTERM / SIGINT / quit)
//   --mode bt|wifi     the mode toggle at power-on (default wifi, so the relay can drive it)
//   --power on|off     the power toggle at power-on (default on)
//   --tilt-stop DEG    the barrel's bottom stop, degrees from its power-on position, + = up
//                      (default kDefaultTiltStopDeg); "off" = no stop
//   --a2 COUNTS        the stall-sense input A2 (default 0)
//   --fake-pad         BLE mode: a pad the stdin "pad" commands drive (default: no pad)
//   --beacon           WiFi mode: BeaconPackets to 127.0.0.1:<--beacon-port>
//   --beacon-port N    (default WIFI_DISCOVERY_PORT)
//   --esp-boot-ms N    ESP32 reset to first AT reply (default kDefaultEspBootMs)
//   --wifi-join-ms N   station join time (default kDefaultWifiJoinMs; -1 = never: AP fallback)
//   --no-esp-banner    no ROM banner on the RA4M1's Serial RX at ESP32 boots
//   --loop-us N        virtual time per App::loop() (default kDefaultLoopUs)
//   --seed N           the ESP32's random source (bootId)
// stdin (one command per line; replies and the simulator's own lines go to stderr as "sim: t=<s> ..."):
//   power on|off   mode bt|wifi   a2 <counts>   tiltstop <deg>|off   shell <line>   speed <x>
//   pad on|off | pad set <rx> <ry> <rt> <buttons>   status   help   quit
// stdout carries the RA4M1's text lines and nothing else. SIGTERM / SIGINT: clean exit, status 0.
// A closed stdin (the relay passes /dev/null) only stops command input.
//
// SPDX-License-Identifier: MIT (the combined dsentry_sim binary: AGPL-3.0, see sim_esp32.h)
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "App.h"
#include "hal_host.h"
#include "sim_esp32.h"
#include "sim_wire.h"

namespace {

// ---- model constants of the simulator (not firmware tunables) --------------------------------
constexpr uint32_t kDefaultLoopUs = 100;        // one App::loop() of virtual time (10 kHz)
constexpr uint32_t kDefaultEspBootMs = 400;     // QEMU: 0.4 s from reset to the first AT reply
                                                // (sim/qemu/README.md, research-esp32-fork.md Q4)
constexpr int32_t kDefaultWifiJoinMs = 1000;    // stand-in station join (DHCP included)
constexpr float kDefaultTiltStopDeg = -30.0f;   // the barrel starts 30 deg above its bottom stop
constexpr double kMaxSpeed = 1000.0;
constexpr long kMaxOptionMs = 100000;           // upper bound of the --*-ms options
constexpr uint64_t kMaxLagUs = 100 * US_PER_MS; // further behind than this: drop the backlog
constexpr int kPollTimeoutMs = 1;               // wall-clock wait between batches of loops
constexpr size_t kBridgeRxBytes = 8192;         // the bridge's UART0 RX buffer, PC -> RA4M1
                                                // (UNOR4USBBridge.ino:158)
constexpr size_t kPtyOutMaxBytes = 1024;        // RA4M1 -> PC bytes held while the PC is not reading
constexpr size_t kTextLineCap = 1024;           // a "line" this long without '\n' is printed as one
constexpr size_t kCommandLineCap = 1024;
constexpr size_t kReadChunk = 4096;
constexpr uint64_t kNsPerUs = 1000;
constexpr uint64_t kUsbByteNs = (uint64_t)UART_BITS_PER_BYTE * US_PER_S * kNsPerUs / DEBUG_SERIAL_BAUD;

volatile sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }

uint64_t g_nowUs = 0;   // virtual time

void simLog(const char* text) {
  std::fprintf(stderr, "sim: t=%.3f %s\n", (double)g_nowUs / (double)US_PER_S, text);
  std::fflush(stderr);
}

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

// ---- the RA4M1's Serial TX, split into frames (cut) and text lines (stdout) ------------------
// Same acceptance rule as every D-Sentry deframer (architecture.md 4.1): one of the four start
// bytes, that type's exact len, a matching CRC; on a mismatch only the first held byte is text.
class UsbTap {
 public:
  void feed(const std::string& bytes) {
    for (unsigned char c : bytes) {
      held_.push_back(c);
      settle();
    }
  }
  // At exit: held bytes are the start of a frame the stop cut off (a start byte and its len byte
  // are never both text); only a lone printable one ('Z') can still be text.
  void finish() {
    if (held_.size() == 1 && held_[0] >= ' ' && held_[0] < 0x7F) text(held_[0]);
    held_.clear();
    if (!line_.empty()) emit();
  }
  uint64_t frames(uint8_t type) const { return type < kTypes ? frames_[type] : 0; }
  uint64_t framesTotal() const {
    uint64_t n = 0;
    for (uint64_t f : frames_) n += f;
    return n;
  }
  uint64_t lines() const { return lines_; }
  uint64_t junk() const { return junk_; }
  bool stdoutGone() const { return gone_; }

 private:
  static constexpr uint8_t kTypes = 8;
  void settle() {
    while (!held_.empty()) {
      const uint8_t type = ds_packet_type_for_start(held_[0]);
      if (type == DS_PKT_NONE) {
        dropFirst();
        continue;
      }
      if (held_.size() < DS_FRAME_HEADER_LEN) return;
      if (held_[1] != ds_payload_len_for_type(type)) {
        dropFirst();
        continue;
      }
      const size_t n = ds_frame_len_for_type(type);
      if (held_.size() < n) return;
      if (ds_crc8(held_.data() + 1, n - 2) != held_[n - 1]) {
        dropFirst();
        continue;
      }
      if (type < kTypes) frames_[type]++;
      held_.erase(held_.begin(), held_.begin() + (long)n);
    }
  }
  void dropFirst() {
    text(held_[0]);
    held_.erase(held_.begin());
  }
  void text(unsigned char c) {
    if (c == '\n') {
      emit();
    } else if (c == '\r') {
      // dropped, like the relay's clean_text()
    } else if (c == '\t' || (c >= ' ' && c < 0x7F)) {
      line_.push_back((char)c);
      if (line_.size() >= kTextLineCap) emit();
    } else {
      junk_++;   // never printed: stdout is text only
    }
  }
  void emit() {
    line_.push_back('\n');
    lines_++;
    size_t off = 0;
    while (!gone_ && off < line_.size()) {
      const ssize_t n = write(STDOUT_FILENO, line_.data() + off, line_.size() - off);
      if (n > 0) {
        off += (size_t)n;
      } else if (n < 0 && errno == EINTR) {
        continue;
      } else {
        gone_ = true;   // EPIPE: whoever read stdout is gone
      }
    }
    line_.clear();
  }
  std::vector<uint8_t> held_;
  std::string line_;
  uint64_t frames_[kTypes] = {};
  uint64_t lines_ = 0;
  uint64_t junk_ = 0;
  bool gone_ = false;
};

// ---- options -------------------------------------------------------------------------------
struct Options {
  int udpPort = -1;   // -1 = WIFI_UDP_PORT
  double speed = 1.0;
  bool pty = false;
  std::string ptyLink;
  double seconds = -1.0;
  bool wifi = true;
  bool power = true;
  bool tiltStop = true;
  float tiltStopDeg = kDefaultTiltStopDeg;
  int a2 = 0;
  bool fakePad = false;
  bool beacon = false;
  int beaconPort = 0;
  long espBootMs = kDefaultEspBootMs;
  long wifiJoinMs = kDefaultWifiJoinMs;
  bool banner = true;
  long loopUs = kDefaultLoopUs;
  unsigned long seed = 0;
};

const char kUsage[] =
    "usage: dsentry_sim [--udp-port N] [--speed X] [--pty] [--pty-link PATH] [--seconds S]\n"
    "                   [--mode bt|wifi] [--power on|off] [--tilt-stop DEG|off] [--a2 COUNTS]\n"
    "                   [--fake-pad] [--beacon] [--beacon-port N] [--esp-boot-ms N]\n"
    "                   [--wifi-join-ms N] [--no-esp-banner] [--loop-us N] [--seed N]\n"
    "stdin: power on|off, mode bt|wifi, a2 <counts>, tiltstop <deg>|off, shell <line>, speed <x>,\n"
    "       pad on|off|set <rx> <ry> <rt> <buttons>, status, help, quit\n";

bool parseLong(const char* s, long lo, long hi, long& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s, &end, 10);
  if (errno || *end || v < lo || v > hi) return false;
  out = v;
  return true;
}

bool parseDouble(const char* s, double& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s, &end);
  if (errno || *end || !std::isfinite(v)) return false;
  out = v;
  return true;
}

// Returns 0 on success, 2 on a usage error (after printing why), 1 for --help.
int parseArgs(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    const char* v = i + 1 < argc ? argv[i + 1] : nullptr;
    long n = 0;
    double d = 0.0;
    auto bad = [&](const char* why) {
      std::fprintf(stderr, "dsentry_sim: %s %s\n%s", a.c_str(), why, kUsage);
      return 2;
    };
    if (a == "-h" || a == "--help") {
      std::fputs(kUsage, stdout);
      return 1;
    } else if (a == "--pty") {
      o.pty = true;
    } else if (a == "--fake-pad") {
      o.fakePad = true;
    } else if (a == "--beacon") {
      o.beacon = true;
    } else if (a == "--no-esp-banner") {
      o.banner = false;
    } else if (!v) {
      return bad(a.rfind("--", 0) == 0 ? "needs a value (or is unknown)" : "is not an option");
    } else if (a == "--udp-port") {
      if (!parseLong(v, 0, UINT16_MAX, n)) return bad("must be 0..65535");
      o.udpPort = (int)n;
      i++;
    } else if (a == "--speed") {
      if (!parseDouble(v, d) || !(d > 0.0) || d > kMaxSpeed) return bad("must be > 0 and <= 1000");
      o.speed = d;
      i++;
    } else if (a == "--pty-link") {
      o.pty = true;
      o.ptyLink = v;
      i++;
    } else if (a == "--seconds") {
      if (!parseDouble(v, d) || d < 0.0) return bad("must be >= 0");
      o.seconds = d;
      i++;
    } else if (a == "--mode") {
      if (std::strcmp(v, "bt") && std::strcmp(v, "wifi")) return bad("must be bt or wifi");
      o.wifi = std::strcmp(v, "wifi") == 0;
      i++;
    } else if (a == "--power") {
      if (std::strcmp(v, "on") && std::strcmp(v, "off")) return bad("must be on or off");
      o.power = std::strcmp(v, "on") == 0;
      i++;
    } else if (a == "--tilt-stop") {
      if (std::strcmp(v, "off") == 0) {
        o.tiltStop = false;
      } else if (parseDouble(v, d) && d <= 0.0) {
        o.tiltStop = true;
        o.tiltStopDeg = (float)d;
      } else {
        return bad("must be <= 0 (degrees below the power-on position) or off");
      }
      i++;
    } else if (a == "--a2") {
      if (!parseLong(v, 0, ADC_MAX_COUNTS, n)) return bad("must be 0..ADC_MAX_COUNTS");
      o.a2 = (int)n;
      i++;
    } else if (a == "--beacon-port") {
      if (!parseLong(v, 1, UINT16_MAX, n)) return bad("must be 1..65535");
      o.beaconPort = (int)n;
      i++;
    } else if (a == "--esp-boot-ms") {
      if (!parseLong(v, 0, kMaxOptionMs, n)) return bad("must be 0..100000");
      o.espBootMs = n;
      i++;
    } else if (a == "--wifi-join-ms") {
      if (!parseLong(v, -1, kMaxOptionMs, n)) return bad("must be -1..100000");
      o.wifiJoinMs = n;
      i++;
    } else if (a == "--loop-us") {
      if (!parseLong(v, 1, (long)US_PER_MS, n)) return bad("must be 1..1000");
      o.loopUs = n;
      i++;
    } else if (a == "--seed") {
      if (!parseLong(v, 0, INT32_MAX, n)) return bad("must be 0..2147483647");
      o.seed = (unsigned long)n;
      i++;
    } else {
      return bad("is not an option");
    }
  }
  return 0;
}

// ---- the physical inputs ---------------------------------------------------------------------
bool levelFor(bool closed) { return TOGGLE_ACTIVE_LOW ? !closed : closed; }
void setPowerToggle(bool on) { hal_host::setDigitalInput(PIN_TOGGLE_POWER, levelFor(on)); }   // closed = on
void setModeToggle(bool wifi) {
  hal_host::setDigitalInput(PIN_TOGGLE_MODE, levelFor(wifi == TOGGLE_MODE_WIFI_WHEN_CLOSED));
}
int8_t tiltSign() { return TILT_DIR_INVERT ? -1 : 1; }
int8_t panSign() { return PAN_DIR_INVERT ? -1 : 1; }
float tiltTrueUpDeg() { return (float)(hal_host::trueSteps(AXIS_TILT) * tiltSign()) / TILT_STEPS_PER_DEG; }
float panTrueDeg() { return (float)(hal_host::trueSteps(AXIS_PAN) * panSign()) / PAN_STEPS_PER_DEG; }
void setTiltStop(bool on, float deg) {
  hal_host::setTiltBottomStop(on, (int64_t)std::llround(deg * TILT_STEPS_PER_DEG) * tiltSign());
}

// ---- the pseudo-terminal (the PC's end of the USB serial port) -------------------------------
struct Pty {
  int master = -1;
  int slave = -1;   // kept open: no EOF/EIO on the master while no PC tool has the port open
  std::string path;
  std::string link;
  std::string out;  // RA4M1 -> PC bytes not yet written
  uint64_t dropped = 0;

  bool open(const std::string& linkPath) {
    master = posix_openpt(O_RDWR | O_NOCTTY);
    if (master < 0 || grantpt(master) != 0 || unlockpt(master) != 0) return false;
    const char* name = ptsname(master);
    if (!name) return false;
    path = name;
    slave = ::open(name, O_RDWR | O_NOCTTY);
    if (slave < 0) return false;
    termios t;
    if (tcgetattr(slave, &t) != 0) return false;
    cfmakeraw(&t);   // no echo, no line editing: the port carries binary frames
    cfsetispeed(&t, DEBUG_SERIAL_BAUD);
    cfsetospeed(&t, DEBUG_SERIAL_BAUD);
    if (tcsetattr(slave, TCSANOW, &t) != 0) return false;
    const int fl = fcntl(master, F_GETFL);
    if (fl < 0 || fcntl(master, F_SETFL, fl | O_NONBLOCK) != 0) return false;
    if (!linkPath.empty()) {
      struct stat st;
      if (lstat(linkPath.c_str(), &st) == 0 && S_ISLNK(st.st_mode)) unlink(linkPath.c_str());
      if (symlink(path.c_str(), linkPath.c_str()) != 0) return false;
      link = linkPath;
    }
    return true;
  }
  void queue(const std::string& bytes) {
    out += bytes;
    if (out.size() > kPtyOutMaxBytes) {   // the PC is not reading: the bridge drops, never stalls
      dropped += out.size() - kPtyOutMaxBytes;
      out.erase(0, out.size() - kPtyOutMaxBytes);
    }
  }
  void flush() {
    while (!out.empty()) {
      const ssize_t n = write(master, out.data(), out.size());
      if (n > 0) {
        out.erase(0, (size_t)n);
      } else if (n < 0 && errno == EINTR) {
        continue;
      } else {
        return;   // EAGAIN: try again next batch
      }
    }
  }
  void close() {
    if (!link.empty()) unlink(link.c_str());
    if (slave >= 0) ::close(slave);
    if (master >= 0) ::close(master);
    master = slave = -1;
  }
};

// ---- the simulator -----------------------------------------------------------------------------
struct Sim {
  Options opt;
  App app;
  UsbTap tap;
  Pty pty;
  SimWire usbIn{kUsbByteNs, kBridgeRxBytes};   // -> the RA4M1's Serial RX
  uint64_t ptyInDropped = 0;                    // PC bytes while the passthrough was down
  uint64_t ptyOutDown = 0;                      // RA4M1 bytes while the passthrough was down
  bool stdinOpen = true;
  std::string stdinLine;
  bool power = true;
  bool wifi = true;
  bool tiltStopOn = true;
  float tiltStopDeg = kDefaultTiltStopDeg;
  int a2 = 0;
  uint64_t lagResets = 0;
  bool quit = false;
  std::chrono::steady_clock::time_point wallBase;
  uint64_t virtBase = 0;

  void rebase() {
    wallBase = std::chrono::steady_clock::now();
    virtBase = g_nowUs;
  }

  // One App::loop() at virtual time `us`, with both chips' I/O around it.
  void step(uint64_t us) {
    g_nowUs = us;
    hal_host::setTimeUs(us);
    hal_host::linkInject(simesp::toRa(us));
    usbIn.send(us, simesp::takeUart0(us));
    hal_host::usbInject(usbIn.take(us));
    app.loop();
    simesp::fromRa(us, hal_host::linkTakeTx());
    simesp::run(us);
    const std::string usb = hal_host::usbTakeTx();
    if (!usb.empty()) {
      tap.feed(usb);
      if (pty.master >= 0) {
        if (simesp::bridgeUp())
          pty.queue(usb);
        else
          ptyOutDown += usb.size();
      }
    }
  }

  std::string statusLine() {
    const Rgb led = hal_host::ledShown(0);
    std::string s = fmt(
        "status speed=%g loops=%u power=%s mode=%s a2=%d tiltUpDeg=%.2f tiltStop=%s tiltAboveStopDeg=%.2f "
        "panDeg=%.2f lcd0=\"%s\" lcd1=\"%s\" backlight=%d led0=%u,%u,%u stdoutLines=%llu framesCut=%llu "
        "telemetryCut=%llu statusCut=%llu junk=%llu lagResets=%llu ",
        opt.speed, (unsigned)app.loops(), power ? "on" : "off", wifi ? "wifi" : "bt", a2, tiltTrueUpDeg(),
        tiltStopOn ? fmt("%.2f", tiltStopDeg).c_str() : "off", tiltStopOn ? tiltTrueUpDeg() - tiltStopDeg : 0.0f,
        panTrueDeg(), hal_host::lcdRow(0).c_str(), hal_host::lcdRow(1).c_str(), hal_host::lcdBacklightOn() ? 1 : 0,
        (unsigned)led.r, (unsigned)led.g, (unsigned)led.b, (unsigned long long)tap.lines(),
        (unsigned long long)tap.framesTotal(), (unsigned long long)tap.frames(DS_PKT_TELEMETRY),
        (unsigned long long)tap.frames(DS_PKT_STATUS), (unsigned long long)tap.junk(),
        (unsigned long long)lagResets);
    if (pty.master >= 0)
      s += fmt("pty=%s ptyDropped=%llu ptyDown=%llu/%llu ", pty.path.c_str(), (unsigned long long)pty.dropped,
               (unsigned long long)ptyInDropped, (unsigned long long)ptyOutDown);
    return s + simesp::status();
  }

  void command(const std::string& raw) {
    std::string line = raw;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
    size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos) return;
    line = line.substr(start);
    const size_t sp = line.find(' ');
    const std::string cmd = line.substr(0, sp);
    std::string arg = sp == std::string::npos ? "" : line.substr(sp + 1);
    const size_t a0 = arg.find_first_not_of(" \t");
    arg = a0 == std::string::npos ? "" : arg.substr(a0);
    double d = 0.0;
    long n = 0;
    if (cmd == "power" && (arg == "on" || arg == "off")) {
      power = arg == "on";
      setPowerToggle(power);
      simLog(fmt("power toggle %s", arg.c_str()).c_str());
    } else if (cmd == "mode" && (arg == "bt" || arg == "wifi")) {
      wifi = arg == "wifi";
      setModeToggle(wifi);
      simLog(fmt("mode toggle %s", arg.c_str()).c_str());
    } else if (cmd == "a2" && parseLong(arg.c_str(), 0, ADC_MAX_COUNTS, n)) {
      a2 = (int)n;
      hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)n);
      simLog(fmt("a2 %ld counts", n).c_str());
    } else if (cmd == "tiltstop" && arg == "off") {
      tiltStopOn = false;
      setTiltStop(false, 0.0f);
      simLog("tilt bottom stop off");
    } else if (cmd == "tiltstop" && parseDouble(arg.c_str(), d)) {
      tiltStopOn = true;
      tiltStopDeg = (float)d;
      setTiltStop(true, tiltStopDeg);
      simLog(fmt("tilt bottom stop at %.2f deg (barrel now %.2f deg)", d, tiltTrueUpDeg()).c_str());
    } else if (cmd == "shell" && !arg.empty()) {
      usbIn.send(g_nowUs, arg + "\n");
      simLog(fmt("shell line to the RA4M1: %s", arg.c_str()).c_str());
    } else if (cmd == "speed" && parseDouble(arg.c_str(), d) && d > 0.0 && d <= kMaxSpeed) {
      opt.speed = d;
      rebase();
      simLog(fmt("speed %g", d).c_str());
    } else if (cmd == "pad") {
      std::string reply;
      simesp::padCommand(arg, &reply);
      simLog(reply.c_str());
    } else if (cmd == "status" && arg.empty()) {
      simLog(statusLine().c_str());
    } else if (cmd == "help") {
      simLog("commands: power on|off, mode bt|wifi, a2 <counts>, tiltstop <deg>|off, shell <line>, "
             "speed <x>, pad on|off|set <rx> <ry> <rt> <buttons>, status, quit");
    } else if (cmd == "quit" || cmd == "exit") {
      quit = true;
    } else {
      simLog(fmt("unknown or malformed command: %s (help lists them)", line.c_str()).c_str());
    }
  }

  void readStdin() {
    char buf[kReadChunk];
    const ssize_t n = read(STDIN_FILENO, buf, sizeof buf);
    if (n == 0 || (n < 0 && errno != EINTR && errno != EAGAIN)) {
      stdinOpen = false;   // /dev/null (the relay) or a closed pipe: keep running
      return;
    }
    if (n < 0) return;
    for (ssize_t i = 0; i < n; i++) {
      if (buf[i] == '\n') {
        command(stdinLine);
        stdinLine.clear();
      } else if (stdinLine.size() < kCommandLineCap) {
        stdinLine.push_back(buf[i]);
      }
    }
  }

  void readPty() {
    char buf[kReadChunk];
    for (;;) {
      const ssize_t n = read(pty.master, buf, sizeof buf);
      if (n <= 0) return;
      if (simesp::bridgeUp())
        usbIn.send(g_nowUs, reinterpret_cast<const uint8_t*>(buf), (size_t)n);
      else
        ptyInDropped += (uint64_t)n;   // the passthrough is down while the ESP32 resets
    }
  }

  // Wait up to kPollTimeoutMs of wall time for stdin or the pty, then serve them.
  void waitIo(bool caughtUp) {
    pollfd fds[2];
    nfds_t n = 0;
    int iStdin = -1, iPty = -1;
    if (stdinOpen) {
      fds[n].fd = STDIN_FILENO;
      fds[n].events = POLLIN;
      iStdin = (int)n++;
    }
    if (pty.master >= 0) {
      fds[n].fd = pty.master;
      fds[n].events = POLLIN;
      iPty = (int)n++;
    }
    const int rc = poll(fds, n, caughtUp ? kPollTimeoutMs : 0);
    if (rc <= 0) return;
    if (iStdin >= 0 && (fds[iStdin].revents & (POLLIN | POLLHUP | POLLERR))) readStdin();
    if (iPty >= 0 && (fds[iPty].revents & POLLIN)) readPty();
  }

  int run() {
    rebase();
    const uint64_t loopUs = (uint64_t)opt.loopUs;
    const uint64_t endUs = opt.seconds >= 0.0 ? (uint64_t)std::llround(opt.seconds * (double)US_PER_S) : UINT64_MAX;
    while (!g_stop && !quit && !tap.stdoutGone() && g_nowUs < endUs) {
      const double wallUs =
          std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - wallBase).count();
      uint64_t target = virtBase + (uint64_t)(wallUs * opt.speed);
      if (target > g_nowUs + kMaxLagUs) {   // the Mac fell behind: drop the backlog, count it
        lagResets++;
        virtBase = g_nowUs;
        wallBase = std::chrono::steady_clock::now();
        target = g_nowUs;
      }
      if (target > endUs) target = endUs;
      while (g_nowUs + loopUs <= target && !g_stop) step(g_nowUs + loopUs);
      if (pty.master >= 0) pty.flush();
      waitIo(g_nowUs + loopUs > target);
    }
    tap.finish();
    return 0;
  }
};

}  // namespace

int main(int argc, char** argv) {
  static Sim sim;   // App is large; keep it off the stack
  const int rc = parseArgs(argc, argv, sim.opt);
  if (rc == 1) return 0;
  if (rc != 0) return rc;

  struct sigaction sa;
  std::memset(&sa, 0, sizeof sa);
  sa.sa_handler = onSignal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  signal(SIGPIPE, SIG_IGN);

  simesp::Options eo;
  if (sim.opt.udpPort == 0) {   // a free port, chosen now so it stays the same across ESP32 restarts
    const int probe = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a;
    std::memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t len = sizeof a;
    if (probe < 0 || bind(probe, (sockaddr*)&a, sizeof a) != 0 || getsockname(probe, (sockaddr*)&a, &len) != 0) {
      std::fprintf(stderr, "dsentry_sim: cannot find a free UDP port: %s\n", std::strerror(errno));
      return 1;
    }
    close(probe);
    eo.udpPort = ntohs(a.sin_port);
  } else {
    eo.udpPort = sim.opt.udpPort < 0 ? 0 : (uint16_t)sim.opt.udpPort;
  }
  eo.beacon = sim.opt.beacon;
  eo.beaconPort = (uint16_t)sim.opt.beaconPort;
  eo.bootMs = (uint32_t)sim.opt.espBootMs;
  eo.wifiJoinMs = (int32_t)sim.opt.wifiJoinMs;
  eo.romBanner = sim.opt.banner;
  eo.fakePad = sim.opt.fakePad;
  eo.seed = (uint32_t)sim.opt.seed;

  if (sim.opt.pty && !sim.pty.open(sim.opt.ptyLink)) {
    std::fprintf(stderr, "dsentry_sim: cannot create the pty%s: %s\n", sim.opt.ptyLink.empty() ? "" : " link",
                 std::strerror(errno));
    sim.pty.close();
    return 1;
  }

  // Power-on: both chips at once. The toggles and A2 as the bench has them.
  hal_host::reset();
  sim.power = sim.opt.power;
  sim.wifi = sim.opt.wifi;
  sim.tiltStopOn = sim.opt.tiltStop;
  sim.tiltStopDeg = sim.opt.tiltStopDeg;
  sim.a2 = sim.opt.a2;
  setPowerToggle(sim.power);
  setModeToggle(sim.wifi);
  setTiltStop(sim.tiltStopOn, sim.tiltStopDeg);
  hal_host::setAnalog(PIN_STALL_SENSE, (uint16_t)sim.a2);
  simesp::begin(eo, simLog);
  simLog(fmt("dsentry_sim: RA4M1 firmware on the host HAL, ESP32 chAT + D-Sentry handlers; speed %g, "
             "loop %ld us, power %s, mode %s, udp 127.0.0.1:%u, tilt stop %s",
             sim.opt.speed, sim.opt.loopUs, sim.power ? "on" : "off", sim.wifi ? "wifi" : "bt",
             (unsigned)simesp::udpPort(), sim.tiltStopOn ? fmt("%.1f deg", sim.tiltStopDeg).c_str() : "off")
             .c_str());
  if (sim.pty.master >= 0)
    simLog(fmt("pty %s (the RA4M1's USB serial%s%s)", sim.pty.path.c_str(), sim.pty.link.empty() ? "" : ", link ",
               sim.pty.link.c_str())
               .c_str());
  sim.app.begin();   // setup(): nothing after it blocks

  sim.run();

  simesp::end();
  simLog(fmt("stopped%s: %u loops, %llu text lines on stdout, %llu frames cut (%llu telemetry, %llu status), "
             "%llu junk bytes, %llu lag resets",
             g_stop ? " by signal" : sim.quit ? " by quit" : sim.tap.stdoutGone() ? " (stdout closed)" : "",
             (unsigned)sim.app.loops(), (unsigned long long)sim.tap.lines(), (unsigned long long)sim.tap.framesTotal(),
             (unsigned long long)sim.tap.frames(DS_PKT_TELEMETRY), (unsigned long long)sim.tap.frames(DS_PKT_STATUS),
             (unsigned long long)sim.tap.junk(), (unsigned long long)sim.lagResets)
             .c_str());
  sim.pty.close();
  return 0;
}
