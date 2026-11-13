# SPDX-License-Identifier: MIT
"""How the relay reaches the turret (architecture.md 4.3, 4.4, 7).

- SerialTransport: the UNO R4 WiFi's USB port. Auto-detect = the one port with USB ID 2341:1002.
  Opened at exactly DEBUG_SERIAL_BAUD (1200 or 2400 would reset the RA4M1), write_timeout
  RELAY_SERIAL_WRITE_TIMEOUT_MS. Always reading (the bridge stops forwarding PC->RA4M1 while the
  PC holds the port without reading). Any error, on read or write, closes the port and it is
  reopened every RELAY_REOPEN_MS. read_only=True is --monitor: nothing is ever written.
- UdpTransport: WiFi mode. ControlPackets go to the turret; Status/Telemetry come back to this
  socket (the ESP32 replies to the source address of the last valid ControlPacket). Target, in
  order: explicit --udp HOST[:PORT]; else the BeaconPacket on WIFI_DISCOVERY_PORT; else
  WIFI_AP_IP:WIFI_UDP_PORT when this computer's address is on the turret's own AP subnet.
- SimProcess: --sim spawns dsentry_sim (sim/build/dsentry_sim or $DSENTRY_SIM_BIN) with
  "--udp-port PORT" and talks UDP to it on 127.0.0.1; its stdout (the RA4M1's text) is relayed.

Every transport hands what it receives to a sink with three methods:
    sink.on_frame(frame, via)   sink.on_text(text, via)   sink.note(kind, text)
called from the transport's own reader thread.
"""

import ipaddress
import os
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable, Optional, Sequence, Tuple

from . import framing
from . import protocol as P
from .settings import UNO_R4_WIFI_PID, UNO_R4_WIFI_VID

# How often a blocked reader wakes to notice a stop or a failed write (not a tunable: it only
# bounds shutdown and recovery latency).
READ_POLL_S = 0.1
UDP_RECV_BYTES = 2048          # larger than any frame; a datagram is one frame
SIM_STOP_WAIT_S = 2.0          # SIGTERM, then this long, then SIGKILL
DEFAULT_SIM_BIN = Path(__file__).resolve().parents[2] / "sim" / "build" / "dsentry_sim"
SIM_BIN_ENV = "DSENTRY_SIM_BIN"
LOOPBACK = "127.0.0.1"


class PortError(Exception):
    """No usable serial port; the message says what to do."""


class SimError(Exception):
    """The simulator cannot be started; the message says what to do."""


def detect_port(ports=None) -> str:
    """The one serial port with the UNO R4 WiFi's USB ID. Listing ports never opens them."""
    if ports is None:
        from serial.tools import list_ports
        ports = list_ports.comports()
    matches = sorted(p.device for p in ports
                     if p.vid == UNO_R4_WIFI_VID and p.pid == UNO_R4_WIFI_PID)
    if not matches:
        raise PortError("no UNO R4 WiFi found (USB %04x:%04x). Plug it in or name the port "
                        "(relay: --serial/--monitor PORT; bench console: --port PORT)."
                        % (UNO_R4_WIFI_VID, UNO_R4_WIFI_PID))
    if len(matches) > 1:
        raise PortError("several UNO R4 WiFi boards found (%s); name one (relay: --serial/--monitor "
                        "PORT; bench console: --port PORT)."
                        % ", ".join(matches))
    return matches[0]


class Transport:
    """Base: a reader thread feeding a LinkReader, and send()."""

    kind = "none"
    sends = True
    expected_link_source: Optional[int] = None   # TelemetryPacket.linkSource while obeying us

    def __init__(self, settings, sink) -> None:
        self.s = settings
        self.sink = sink
        self.reader = framing.LinkReader()
        self.sent = 0
        self.dropped = 0
        self._thread: Optional[threading.Thread] = None

    @property
    def crc_errors(self) -> int:
        return self.reader.crc_errors

    def _deliver(self, items) -> None:
        for item in items:
            if isinstance(item, framing.TextLine):
                self.sink.on_text(item.text, self.kind)
            else:
                self.sink.on_frame(item, self.kind)

    def start(self, stop: threading.Event) -> None:
        self._thread = threading.Thread(target=self._run, args=(stop,), daemon=True,
                                        name="%s-reader" % self.kind)
        self._thread.start()

    def _run(self, stop: threading.Event) -> None:
        stop.wait()

    def join(self, timeout: float) -> None:
        if self._thread is not None:
            self._thread.join(timeout)

    def send(self, data: bytes) -> bool:
        return False

    def up(self) -> bool:
        return False

    def describe(self) -> str:
        return self.kind

    def status(self) -> dict:
        return {"kind": self.kind, "up": self.up(), "detail": self.describe(),
                "sent": self.sent, "dropped": self.dropped, "crcErrors": self.crc_errors,
                "sends": self.sends}


# ---------------------------------------------------------------------------------------------
class SerialTransport(Transport):
    kind = "serial"
    expected_link_source = P.DS_LINK_SRC_USB

    def __init__(self, settings, sink, port: Optional[str] = None, read_only: bool = False,
                 open_port: Optional[Callable] = None, list_ports: Optional[Callable] = None) -> None:
        super().__init__(settings, sink)
        self.port = port                  # None = auto-detect
        self.read_only = read_only
        self.sends = not read_only
        if read_only:
            self.kind = "monitor"
        self._open_port = open_port or self._open_serial
        self._list_ports = list_ports
        self._lock = threading.Lock()     # guards _link swaps and writes
        self._link = None
        self._name: Optional[str] = None
        self._broken: Optional[str] = None
        self.opens = 0
        self.last_problem: Optional[str] = None

    def _open_serial(self, name: str):
        import serial    # lazy: --demo, --udp and the tests of other parts never need pyserial
        return serial.Serial(name, self.s.serial_baud, timeout=READ_POLL_S,
                             write_timeout=self.s.serial_write_timeout_s)

    def _resolve(self) -> str:
        if self.port is not None:
            return self.port
        ports = self._list_ports() if self._list_ports else None
        return detect_port(ports)

    def _run(self, stop: threading.Event) -> None:
        last_problem = None
        while not stop.is_set():
            problem = None
            try:
                name = self._resolve()
                link = self._open_port(name)
                try:
                    with self._lock:
                        self._link, self._name, self._broken = link, name, None
                    self.reader.reset()
                    self.opens += 1
                    last_problem = None
                    self.sink.note("relay", "serial %s open at %d baud%s" % (
                        name, self.s.serial_baud, " (monitor: read only, sends nothing)" if self.read_only else ""))
                    while not stop.is_set():
                        if self._broken:
                            raise IOError("write failed: %s" % self._broken)
                        data = link.read(getattr(link, "in_waiting", 0) or 1)
                        if data:
                            self._deliver(self.reader.feed(data))
                finally:
                    with self._lock:
                        self._link = None
                    try:
                        link.close()
                    except Exception:  # noqa: BLE001
                        pass
            except PortError as error:
                problem = str(error)
            except Exception as error:  # noqa: BLE001 (deliberately broad)
                # SerialException, OSError, SerialTimeoutException from a write, and termios.error
                # (not an OSError) when a USB device is half re-enumerated: all mean "reopen".
                problem = "serial port problem: %s: %s; reopening every %g s" % (
                    type(error).__name__, error, self.s.reopen_s)
            if problem:
                self.last_problem = problem
                if problem != last_problem:
                    self.sink.note("relay", problem)     # once per distinct problem
                    last_problem = problem
            if stop.is_set():
                break
            stop.wait(self.s.reopen_s)

    def send(self, data: bytes) -> bool:
        if self.read_only:
            return False
        with self._lock:
            link = self._link
            if link is None or self._broken:
                self.dropped += 1
                return False
            try:
                link.write(data)
            except Exception as error:  # noqa: BLE001 (SerialTimeoutException after write_timeout too)
                self._broken = "%s: %s" % (type(error).__name__, error)
                self.dropped += 1
                cancel = getattr(link, "cancel_read", None)
                if cancel is not None:
                    try:
                        cancel()          # wake the reader now; it closes and reopens the port
                    except Exception:  # noqa: BLE001
                        pass
                return False
            self.sent += 1
            return True

    def up(self) -> bool:
        return self._link is not None and not self._broken

    def describe(self) -> str:
        what = "monitor (read only)" if self.read_only else "serial"
        if self._link is not None:
            return "%s %s @ %d" % (what, self._name, self.s.serial_baud)
        return "%s %s: closed%s" % (what, self.port or "auto", (" (%s)" % self.last_problem) if self.last_problem else "")


# ---------------------------------------------------------------------------------------------
def local_ip_for(host: str) -> Optional[str]:
    """The local address the OS would use to reach host (UDP connect sends nothing)."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect((host, 1))
        return probe.getsockname()[0]
    except OSError:
        return None
    finally:
        probe.close()


def on_subnet(ip: Optional[str], network_ip: str, netmask: str) -> bool:
    if not ip:
        return False
    try:
        return ipaddress.ip_address(ip) in ipaddress.ip_network("%s/%s" % (network_ip, netmask), strict=False)
    except ValueError:
        return False


def parse_hostport(text: str, default_port: int) -> Tuple[str, int]:
    """'host', 'host:port' or '[v6]:port' -> (host, port). ValueError on a bad port."""
    text = text.strip()
    if text.startswith("["):
        host, _, rest = text[1:].partition("]")
        port = rest[1:] if rest.startswith(":") else ""
    elif text.count(":") == 1:
        host, port = text.split(":")
    else:
        host, port = text, ""
    if not host:
        raise ValueError("no host in %r" % text)
    value = int(port) if port else default_port
    if not 0 < value < 65536:
        raise ValueError("port %d out of range" % value)
    return host, value


class UdpTransport(Transport):
    kind = "udp"
    expected_link_source = P.DS_LINK_SRC_WIFI

    def __init__(self, settings, sink, target: Optional[Tuple[str, int]] = None,
                 bind_host: str = "0.0.0.0", discovery: bool = True,
                 discovery_port: Optional[int] = None, ap_fallback: bool = True,
                 local_ip: Callable[[str], Optional[str]] = local_ip_for) -> None:
        super().__init__(settings, sink)
        self.explicit = target is not None
        self._target: Optional[Tuple[str, int]] = None
        if target is not None:
            self._target = (socket.gethostbyname(target[0]), int(target[1]))
        self.bind_host = bind_host
        self.discovery = discovery and not self.explicit
        self.discovery_port = settings.discovery_port if discovery_port is None else discovery_port
        self.ap_fallback = ap_fallback and not self.explicit
        self._local_ip = local_ip
        self.how = "explicit" if self.explicit else None   # explicit | beacon | ap
        self.beacon: Optional[dict] = None
        self.foreign = 0
        self.received = 0
        self._sock: Optional[socket.socket] = None
        self._send_error: Optional[str] = None
        self._disc_thread: Optional[threading.Thread] = None
        self.discovery_bound = threading.Event()

    @property
    def target(self) -> Optional[Tuple[str, int]]:
        return self._target

    def local_port(self) -> Optional[int]:
        return self._sock.getsockname()[1] if self._sock is not None else None

    def start(self, stop: threading.Event) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.bind_host, 0))
        self._sock.settimeout(READ_POLL_S)
        if self.explicit:
            self.sink.note("relay", "UDP to %s:%d (explicit)" % self._target)
        super().start(stop)
        if self.discovery or self.ap_fallback:
            self._disc_thread = threading.Thread(target=self._discover, args=(stop,), daemon=True,
                                                 name="udp-discovery")
            self._disc_thread.start()

    def _set_target(self, target: Tuple[str, int], how: str, why: str) -> None:
        if target != self._target or how != self.how:
            self._target, self.how = target, how
            self.reader.reset()
            self.sink.note("relay", "UDP turret at %s:%d (%s)" % (target[0], target[1], why))

    def _run(self, stop: threading.Event) -> None:
        sock = self._sock
        while not stop.is_set():
            try:
                data, peer = sock.recvfrom(UDP_RECV_BYTES)
            except socket.timeout:
                continue
            except OSError as error:
                # e.g. ECONNREFUSED after an ICMP port-unreachable on some systems: not fatal
                if stop.wait(READ_POLL_S):
                    break
                self._send_error = "receive: %s" % error
                continue
            target = self._target
            if target is None or peer[0] != target[0]:
                self.foreign += 1           # not from the turret we talk to
                continue
            self.received += 1
            self._deliver(self.reader.feed(data))
            self.reader.reset()             # one datagram = one frame; never join two

    def _discover(self, stop: threading.Event) -> None:
        listener = None
        next_ap_check = 0.0
        problem = None
        while not stop.is_set():
            if self.discovery and listener is None:
                try:
                    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    if hasattr(socket, "SO_REUSEPORT"):
                        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                    listener.bind(("", self.discovery_port))
                    listener.settimeout(READ_POLL_S)
                    self.discovery_bound.set()
                    self.sink.note("relay", "listening for turret beacons on UDP %d" % self.discovery_port)
                except OSError as error:
                    if listener is not None:
                        listener.close()
                    listener = None
                    text = "cannot listen for beacons on UDP %d: %s" % (self.discovery_port, error)
                    if text != problem:
                        self.sink.note("relay", text)
                        problem = text
            now = time.monotonic()
            if self.ap_fallback and self.how is None and now >= next_ap_check:
                next_ap_check = now + self.s.reopen_s
                mine = self._local_ip(self.s.ap_ip)
                if on_subnet(mine, self.s.ap_ip, self.s.ap_netmask):
                    self._set_target((self.s.ap_ip, self.s.udp_port), "ap",
                                     "no beacon yet; this computer (%s) is on the turret's own AP subnet" % mine)
            if listener is None:
                stop.wait(READ_POLL_S)
                continue
            try:
                data, peer = listener.recvfrom(UDP_RECV_BYTES)
            except socket.timeout:
                continue
            except OSError:
                continue
            try:
                beacon = P.BeaconPacket.decode(data)
            except ValueError:
                continue
            ip = ".".join(str(b) for b in beacon.ip)
            if ip == "0.0.0.0":
                ip = peer[0]
            self.beacon = {"ip": ip, "udpPort": beacon.udpPort, "bootId": beacon.bootId,
                           "mode": beacon.mode, "from": peer[0], "t": time.time()}
            port = beacon.udpPort or self.s.udp_port
            self._set_target((ip, port), "beacon", "beacon from %s, boot id %d" % (peer[0], beacon.bootId))
        if listener is not None:
            listener.close()

    def send(self, data: bytes) -> bool:
        target, sock = self._target, self._sock
        if target is None or sock is None:
            self.dropped += 1
            return False
        try:
            sock.sendto(data, target)
        except OSError as error:
            text = "UDP send to %s:%d failed: %s" % (target[0], target[1], error)
            if text != self._send_error:
                self.sink.note("relay", text)
                self._send_error = text
            self.dropped += 1
            return False
        self._send_error = None
        self.sent += 1
        return True

    def up(self) -> bool:
        return self._target is not None

    def describe(self) -> str:
        if self._target is None:
            return "udp: looking for the turret (beacon on %d%s)" % (
                self.discovery_port, ", AP %s" % self.s.ap_ip if self.ap_fallback else "")
        return "udp %s:%d (%s)" % (self._target[0], self._target[1], self.how)

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()


# ---------------------------------------------------------------------------------------------
def free_udp_port(host: str = LOOPBACK) -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind((host, 0))
        return probe.getsockname()[1]


class SimProcess:
    """Spawns the simulator; the caller then talks UDP to 127.0.0.1:port."""

    def __init__(self, settings, sink, binary: Optional[str] = None, port: Optional[int] = None,
                 extra_args: Sequence[str] = ()) -> None:
        self.s = settings
        self.sink = sink
        self.binary = Path(binary or os.environ.get(SIM_BIN_ENV) or DEFAULT_SIM_BIN)
        self.port = settings.udp_port if port is None else port
        self.extra_args = list(extra_args)
        self.proc: Optional[subprocess.Popen] = None
        self._thread: Optional[threading.Thread] = None

    def command(self):
        return [str(self.binary), "--udp-port", str(self.port), *self.extra_args]

    def start(self) -> None:
        if not self.binary.is_file() or not os.access(self.binary, os.X_OK):
            raise SimError("simulator not found at %s; build it with D-SentryCode/sim/build.sh or "
                           "set %s" % (self.binary, SIM_BIN_ENV))
        if self.port == 0:
            self.port = free_udp_port()
        self.proc = subprocess.Popen(self.command(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1, errors="replace")
        self.sink.note("relay", "simulator started: %s (pid %d)" % (" ".join(self.command()), self.proc.pid))
        self._thread = threading.Thread(target=self._pump, daemon=True, name="sim-stdout")
        self._thread.start()

    def _pump(self) -> None:
        for line in self.proc.stdout:
            text = framing.clean_text(line.encode("latin-1", "replace"))
            if text:
                self.sink.on_text(text, "sim")

    def returncode(self) -> Optional[int]:
        return None if self.proc is None else self.proc.poll()

    def stop(self) -> None:
        proc = self.proc
        if proc is None:
            return
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(SIM_STOP_WAIT_S)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        if self._thread is not None:
            self._thread.join(SIM_STOP_WAIT_S)     # the pump reads the last lines, then EOF
        if proc.stdout is not None:
            proc.stdout.close()
