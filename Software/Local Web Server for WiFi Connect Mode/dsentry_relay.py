#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""D-Sentry PC relay and dashboard (docs/architecture.md section 7).

Reads a game pad (SDL, on the main thread), sends ControlPackets at CONTROL_PACKET_HZ to the
turret, shows the turret's telemetry in a local web dashboard, and offers optional keyboard
control from that page. Every tunable comes from D-SentryCode/config.h.

    python dsentry_relay.py                       # USB serial, auto-detect the UNO R4 WiFi
    python dsentry_relay.py --serial /dev/cu.usbmodem1101
    python dsentry_relay.py --udp auto            # WiFi: beacon discovery, then the AP address
    python dsentry_relay.py --udp 192.168.1.50    # WiFi: explicit turret address
    python dsentry_relay.py --monitor             # BT mode: read telemetry over USB, send nothing
    python dsentry_relay.py --sim                 # spawn sim/build/dsentry_sim, talk UDP to it
    python dsentry_relay.py --demo                # Python stand-in turret, no hardware at all

Run it with the project venv (pygame 2.6.1, pyserial 3.5; see requirements.txt).
"""

import argparse
import os
import signal
import sys
import threading
import time
import webbrowser
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dsentry import config as C          # noqa: E402
from dsentry import demo as D            # noqa: E402
from dsentry import hub as H             # noqa: E402
from dsentry import keyboard as K        # noqa: E402
from dsentry import pad as PD            # noqa: E402
from dsentry import relay as R           # noqa: E402
from dsentry import settings as S        # noqa: E402
from dsentry import transports as T      # noqa: E402
from dsentry import web as W             # noqa: E402

PROG = "relay"
SIGNAL_EXIT = 0          # SIGINT / SIGTERM: a normal stop
SIM_DIED_EXIT = 1
USAGE_EXIT = 2


def tcp_port(text: str) -> int:
    try:
        value = int(text)
    except ValueError:
        raise argparse.ArgumentTypeError("not a number: %r" % text) from None
    if not 0 <= value <= 65535:
        raise argparse.ArgumentTypeError("must be 0..65535, got %d" % value)
    return value


def parse_args(argv, settings):
    p = argparse.ArgumentParser(prog="dsentry_relay.py", description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    link = p.add_mutually_exclusive_group()
    link.add_argument("--serial", nargs="?", const="auto", metavar="PORT|auto",
                      help="USB serial to the RA4M1 (default when no other link is given; auto = the "
                           "one port with USB ID 2341:1002)")
    link.add_argument("--udp", nargs="?", const="auto", metavar="HOST[:PORT]|auto",
                      help="WiFi mode: UDP to the turret (default port WIFI_UDP_PORT %d; auto = beacon "
                           "on WIFI_DISCOVERY_PORT %d, then WIFI_AP_IP %s when on its AP subnet)"
                           % (settings.udp_port, settings.discovery_port, settings.ap_ip))
    link.add_argument("--monitor", nargs="?", const="auto", metavar="PORT|auto",
                      help="serial, read only: show telemetry and send nothing (for BT mode)")
    link.add_argument("--sim", action="store_true",
                      help="spawn dsentry_sim (--sim-bin, $%s, or sim/build/dsentry_sim) and talk UDP to it"
                           % T.SIM_BIN_ENV)
    link.add_argument("--demo", action="store_true",
                      help="a Python stand-in turret, to preview the dashboard without hardware")
    p.add_argument("--sim-bin", metavar="PATH", help="simulator binary for --sim")
    p.add_argument("--sim-port", type=tcp_port, metavar="PORT",
                   help="UDP port passed to the simulator as --udp-port (default WIFI_UDP_PORT; 0 = a free one)")
    p.add_argument("--sim-arg", action="append", default=[], metavar="ARG",
                   help="extra argument for the simulator (repeatable)")
    p.add_argument("--no-demo-input", action="store_true",
                   help="--demo without its scripted input (only a pad or the keyboard drives it)")
    p.add_argument("--no-pad", action="store_true", help="do not open a game pad (keyboard/script only)")
    p.add_argument("--http-port", type=tcp_port, default=settings.http_port,
                   help="dashboard port (default DASHBOARD_HTTP_PORT %d; 0 = a free one)" % settings.http_port)
    p.add_argument("--host", default="127.0.0.1",
                   help="dashboard address (default 127.0.0.1; 0.0.0.0 lets another device VIEW it, "
                        "keyboard control stays loopback only)")
    p.add_argument("--no-browser", action="store_true", help="do not open a browser tab")
    p.add_argument("--quiet", action="store_true", help="no once-a-second stats line")
    p.add_argument("--config", metavar="CONFIG_H", help="config.h to read (default D-SentryCode/config.h "
                                                        "or $%s)" % C.ENV_VAR)
    return p.parse_args(argv)


def _config_path(argv):
    """--config has to be known before the parser (its help texts quote config.h values)."""
    for i, a in enumerate(argv):
        if a == "--config" and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith("--config="):
            return a.split("=", 1)[1]
    return None


def say(text: str) -> None:
    print("%s: %s" % (PROG, text), flush=True)


def err(text: str) -> None:
    print("%s: %s" % (PROG, text), file=sys.stderr, flush=True)


def ticker(hub, stop, settings, quiet):
    period = 1.0 / settings.dashboard_max_fps
    next_line = time.monotonic() + settings.stats_s
    while not stop.wait(period):
        hub.tick()
        now = time.monotonic()
        if now >= next_line:
            next_line += settings.stats_s
            if now - next_line > settings.stats_s:
                next_line = now + settings.stats_s
            if not quiet:
                say(H.console_line(hub, now)[len(PROG) + 2:])


def build_transport(args, settings, hub):
    """(mode name, transport, sim process or None)."""
    if args.demo:
        return "demo", D.DemoTransport(settings, hub), None
    if args.sim:
        sim = T.SimProcess(settings, hub, binary=args.sim_bin, port=args.sim_port, extra_args=args.sim_arg)
        sim.start()
        udp = T.UdpTransport(settings, hub, target=(T.LOOPBACK, sim.port), bind_host=T.LOOPBACK,
                             discovery=False, ap_fallback=False)
        return "sim", udp, sim
    if args.udp is not None:
        if args.udp == "auto":
            return "udp", T.UdpTransport(settings, hub), None
        host, port = T.parse_hostport(args.udp, settings.udp_port)
        return "udp", T.UdpTransport(settings, hub, target=(host, port)), None
    if args.monitor is not None:
        port = None if args.monitor == "auto" else args.monitor
        return "monitor", T.SerialTransport(settings, hub, port=port, read_only=True), None
    port = None if args.serial in (None, "auto") else args.serial
    return "serial", T.SerialTransport(settings, hub, port=port), None


def open_pad(args, settings, hub, mode):
    if args.no_pad or mode == "monitor":
        return PD.NullPad()
    try:
        return PD.SdlPad(PD.Scaler.from_settings(settings), log=lambda text: hub.note("input", text))
    except Exception as error:  # noqa: BLE001 (pygame missing, SDL init failure...)
        hub.note("input", "no game pad support (%s: %s); keyboard only" % (type(error).__name__, error))
        return PD.NullPad()


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        settings = S.Settings.load(_config_path(argv))
    except (OSError, S.SettingsError) as error:
        err("cannot use config.h: %s" % error)
        return USAGE_EXIT
    args = parse_args(argv, settings)
    if not W.PAGE_PATH.is_file():
        err("page missing: %s" % W.PAGE_PATH)
        return USAGE_EXIT

    mode_guess = "demo" if args.demo else "sim" if args.sim else "udp" if args.udp else \
        "monitor" if args.monitor else "serial"
    hub = H.Hub(settings, mode_guess, demo=args.demo)
    stop = threading.Event()
    try:
        mode, transport, sim = build_transport(args, settings, hub)
    except (T.SimError, ValueError, OSError) as error:
        err(str(error))
        return USAGE_EXIT
    hub.mode = mode
    sending = transport.sends
    kb = K.KeyboardSession(settings, on_event=hub.note) if sending else None
    pad = open_pad(args, settings, hub, mode)
    fallback = D.DemoScript(settings.cfg, time.monotonic()) if (args.demo and not args.no_demo_input) else None
    stats = R.LinkStats(settings)
    merger = sender = None
    if sending:
        merger = R.InputMerger(settings, kb, fallback=fallback, on_event=hub.note)
        sender = R.Sender(settings, transport, merger.current, stats)
    hub.attach(transport=transport, stats=stats, sender=sender, keyboard=kb, merger=merger,
               pad_info=lambda: {"present": merger.pad.connected if merger else False,
                                 "name": merger.pad.name if merger else "",
                                 "enabled": pad.kind != "none", "detail": pad.describe()})

    try:
        server = W.serve(args.host, args.http_port, W.make_handler(hub, kb, settings, stop))
    except OSError as error:
        err("cannot listen on %s:%d: %s (another relay running? try --http-port)" % (args.host, args.http_port, error))
        if sim is not None:
            sim.stop()
        return USAGE_EXIT

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stop.set())

    threads = []

    def start(target, *a, name=None):
        th = threading.Thread(target=target, args=a, daemon=True, name=name)
        th.start()
        threads.append(th)
        return th

    transport.start(stop)
    sender_thread = start(sender.run, stop, name="sender") if sender is not None else None
    start(ticker, hub, stop, settings, args.quiet, name="ticker")
    start(server.serve_forever, name="http")

    shown = "127.0.0.1" if args.host in ("0.0.0.0", "", "::") else args.host
    url = "http://%s:%d/" % (shown, server.server_address[1])
    say("%s  (mode: %s; %s; %s; Ctrl+C to stop)" % (url, mode, transport.describe(), pad.describe()))
    if mode == "demo":
        say("DEMO: the turret is a Python stand-in, not the RA4M1 logic")
    if not args.no_browser:
        webbrowser.open(url)

    exit_code = SIGNAL_EXIT
    period = 1.0 / settings.pad_poll_hz
    try:
        while not stop.is_set():
            state = pad.poll(time.monotonic())
            if merger is not None:
                merger.set_pad(state)
            while True:
                try:
                    cmd = hub.rumble_queue.get_nowait()
                except Exception:  # noqa: BLE001 (queue.Empty)
                    break
                if not state.connected:
                    continue
                if cmd[0] == "stop":
                    pad.stop_rumble()
                else:
                    pad.rumble(cmd[1], cmd[2], cmd[3])
            if sim is not None and sim.returncode() is not None:
                err("simulator exited with code %s" % sim.returncode())
                exit_code = SIM_DIED_EXIT
                break
            stop.wait(period)
    finally:
        stop.set()
        if sender_thread is not None:
            sender_thread.join(1.0)          # sends one connected=0 packet on its way out
        pad.stop_rumble()
        pad.close()
        server.shutdown()
        server.server_close()
        close = getattr(transport, "close", None)
        if close is not None:
            close()
        transport.join(1.0)
        if sim is not None:
            sim.stop()
        say("stopped")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
