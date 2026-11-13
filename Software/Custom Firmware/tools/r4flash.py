#!/usr/bin/env python3
# r4flash.py - guarded backup, flash, verify and restore of the ESP32-S3 USB-bridge
# firmware on an Arduino UNO R4 WiFi. It is glue around esptool (all flash I/O),
# pyserial (port listing only, never opened here) and hidapi (the jumper-free
# "reboot into download mode" report that Arduino's unor4wifi-reboot tool sends).
#
# Run it with the project venv, not the system python3 (the venv has esptool 5.5.0,
# pyserial and hidapi; its console scripts break on the space in the repo path, see
# DECISIONS.md D03):
#   "<repo>/.venv/bin/python" "<repo>/tools/r4flash.py" <command> [options]
# See tools/README.md for the full procedure.
#
# SPDX-License-Identifier: MIT

import argparse
import hashlib
import importlib.metadata
import io
import os
import re
import shlex
import shutil
import ssl
import subprocess
import sys
import tempfile
import time
import traceback
import urllib.request
import zipfile
from collections import namedtuple
from datetime import datetime, timezone
from pathlib import Path

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

TOOLS_DIR = Path(__file__).resolve().parent
REPO_DIR = TOOLS_DIR.parent
FIRMWARE_DIR = REPO_DIR / "firmware"
DEFAULT_BUILD_DIR = (FIRMWARE_DIR / "UNOR4USBBridge" / "build"
                     / "esp32-patched.esp32.arduino_unor4wifi_usb_bridge")
CORE_TOOLS_DIR = FIRMWARE_DIR / "hardware" / "esp32-patched" / "esp32" / "tools"
PARTITION_CSV = CORE_TOOLS_DIR / "partitions" / "unor4wifi.csv"
GEN_ESP32PART = CORE_TOOLS_DIR / "gen_esp32part.py"
BOOT_APP0_BIN = FIRMWARE_DIR / "boot" / "boot_app0.bin"
DEFAULT_CACHE_DIR = TOOLS_DIR / "cache"
LOG_DIR = TOOLS_DIR / "logs"
VENV_DIR = REPO_DIR / ".venv"

# ---------------------------------------------------------------------------
# Flash layout. Source: firmware/combine.py and unor4wifi.csv. The chip
# (ESP32-S3-MINI-1-N8) has 8 MB, but the bridge firmware only uses the first 4 MB.
# ---------------------------------------------------------------------------

PT_OFFSET = 0x8000          # partition table
PT_LEN = 0xC00              # gen_esp32part.py writes 3072 bytes (table + MD5 row + 0xFF)
OTADATA_OFFSET = 0x9000     # boot_app0.bin here selects app0
OTADATA_LEN = 0x2000
CERT_OFFSET = 0xB000        # x509_crt_bundle; the cert partition is 0x45000 bytes
CERT_LEN = 0x45000
S3_APP_OFFSET = 0xE000      # S3-APP.bin is S3-ALL[0xE000:0x1E0000]
APP0_OFFSET = 0x50000
APP0_LEN = 0x190000         # the real app limit; boards.txt's 0x198000 is 32 KiB too big
SPIFFS_OFFSET = 0x370000    # SPIFFS 0x370000-0x3F0000, then NVS 0x3F0000-0x400000
NVS_OFFSET = 0x3F0000
LAYOUT_END = 0x400000

S3_ALL_LEN = 0x400000       # bootloader..NVS: WIPES NVS (BLE bonds, Preferences) and SPIFFS
S3_BOOT_APP_LEN = 0x1E0000  # bootloader..app0, written at 0x0; what Arduino's updater flashes
S3_APP_LEN = 0x1D2000       # written at 0xE000; does not reset otadata
BACKUP_LENS = (0x400000, 0x800000)
MIB = 1024 * 1024

IMAGE_MAGIC = 0xE9
ESP32S3_CHIP_ID = 9

# sha256 of the 3072-byte table that gen_esp32part.py makes from unor4wifi.csv. It equals
# the stock build's UNOR4USBBridge.ino.partitions.bin and the table inside Arduino's
# official 0.6.0 image (checked 2026-10-11). Embedded so restore works without firmware/.
EXPECTED_PT_SHA256 = "b41d2befdcbf59d1bee69a2e36fc02a382a69f884ac3d959a8e66b9c220a3973"
# sha256 of firmware/boot/boot_app0.bin (8192 bytes, otadata that selects app0).
BOOT_APP0_SHA256 = "f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0"

OFFICIAL_ZIP_URL = ("https://github.com/arduino/uno-r4-wifi-usb-bridge/releases/download/"
                    "0.6.0/unor4wifi-update-macos.zip")
OFFICIAL_ZIP_NAME = "unor4wifi-update-macos.zip"
OFFICIAL_ZIP_SHA256 = "7d527455897d82f491d53f868077aac4e99b9e8be39a4206faff7eba847fe632"
OFFICIAL_MEMBER = "unor4wifi-update-macos/firmware/UNOR4-WIFI-S3-0.6.0.bin"
OFFICIAL_IMAGE_NAME = "UNOR4-WIFI-S3-0.6.0.bin"
OFFICIAL_IMAGE_SHA256 = "e4a0e6f9451188b5714c2ec53a6552d2ad59b59f468cd47b695bd19ba752fd3c"
# macOS's own CA bundle. python.org's macOS Python has no CA certificates of its own until
# its 'Install Certificates.command' is run, and the project venv is built on that Python.
SYSTEM_CA_FILE = Path("/etc/ssl/cert.pem")

# USB identities. Bridge mode = the running bridge firmware (CDC + CMSIS-DAP HID).
# Download mode = the S3's ROM over its built-in USB-Serial/JTAG.
BRIDGE_VID_PID = (0x2341, 0x1002)
DOWNLOAD_VID_PID = (0x303A, 0x1001)
# The bridge's DAP.h _onSetFeature restarts into ROM download mode when the first data
# byte is 0xAA. Report ID 0 (hidapi strips it) + 64 data bytes, as unor4wifi-reboot sends.
REBOOT_REPORT = bytes([0x00, 0xAA]) + bytes(63)
ENTER_DOWNLOAD_WAIT_S = 8.0

CHIP = "esp32s3"
BAUD = "115200"             # the rate Arduino's own updater uses

EXIT_OK = 0
EXIT_FAIL = 1
EXIT_REFUSED = 3            # a safety guard said no (argparse itself uses 2)

# Commands that can talk to a board; only these write a log file (and only when real).
DEVICE_COMMANDS = ("enter-download", "backup", "flash", "restore", "verify")


class Refusal(Exception):
    """A safety guard said no. main() prints 'REFUSED: ...' and exits EXIT_REFUSED."""


class Failure(Exception):
    """Something we ran or needed failed. main() prints 'FAILED: ...', exits EXIT_FAIL."""


# A checked backup: its path, size, the chip MAC from its file name (or None) and its
# bytes. The bytes are kept so that what was checked is exactly what gets used.
Backup = namedtuple("Backup", "path size mac data")
# What flash-id said about the connected chip (both None in a dry run).
Chip = namedtuple("Chip", "size mac")


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def size_label(n):
    """'8MB' for whole mebibytes (the spelling esptool prints), else hex."""
    if n % MIB == 0:
        return f"{n // MIB}MB"
    return f"{n:#x}"


def utc_stamp():
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def parse_offset(text):
    """argparse type for flash offsets: accepts 0x hex or decimal."""
    try:
        value = int(text, 0)
    except ValueError:
        raise argparse.ArgumentTypeError(f"not a number: {text!r}")
    if value < 0:
        raise argparse.ArgumentTypeError("offset must not be negative")
    return value


# ---------------------------------------------------------------------------
# Logging and running external commands
# ---------------------------------------------------------------------------

_log_path = None


def start_log(command):
    """Real device runs keep a log of every command and its output in tools/logs/."""
    global _log_path
    _log_path = LOG_DIR / f"r4flash-{utc_stamp()}-{command}.log"


def log(text):
    # The file is created on the first write, so runs that send nothing leave no file.
    if _log_path is None:
        return
    _log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(_log_path, "a", encoding="utf-8") as f:
        f.write(text if text.endswith("\n") else text + "\n")


def log_if_started(text):
    """Add to a log that already exists. A refusal before anything was sent then leaves
    no file, and a refusal after esptool ran still ends its log with the reason."""
    if _log_path is not None and _log_path.exists():
        log(text)


def clean_env():
    # ESPTOOL_FS, ESPTOOL_BAUD, ESPTOOL_PORT, ESPTOOL_CFGFILE ... would silently change
    # the command lines printed here, so they are not passed on.
    return {k: v for k, v in os.environ.items() if not k.startswith("ESPTOOL_")}


def esptool_cmd(port, after, *command):
    """The one place esptool command lines are built. Global options go first."""
    return [sys.executable, "-m", "esptool", "--chip", CHIP, "--port", port,
            "--baud", BAUD, "--before", "default-reset", "--after", after, *command]


def stream(cmd):
    """Run cmd, copying its output to our stdout as it arrives (progress bars use \\r,
    so raw bytes are passed through). Returns (exit code, output text)."""
    sys.stdout.flush()
    # cwd is tools/ so a stray esptool.cfg/setup.cfg in the caller's directory is not read.
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            env=clean_env(), cwd=TOOLS_DIR)
    chunks = []
    while True:
        chunk = proc.stdout.read1(4096)
        if not chunk:
            break
        chunks.append(chunk)
        sys.stdout.buffer.write(chunk)
        sys.stdout.buffer.flush()
    code = proc.wait()
    return code, b"".join(chunks).decode("utf-8", errors="replace")


def run(cmd, dry_run):
    """Print one external command and run it (streamed). A dry run only prints it.
    Raises Failure on a non-zero exit. Returns the output text ('' in a dry run)."""
    line = shlex.join(str(part) for part in cmd)
    if dry_run:
        print(f"DRY: {line}")
        return ""
    print(f"RUN: {line}", flush=True)
    log(f"RUN: {line}")
    code, text = stream(cmd)
    log(text)
    log(f"EXIT: {code}")
    if code != 0:
        raise Failure(f"exit code {code} from: {line}")
    return text


def run_offline(cmd):
    """Run a command that never touches a device (version, image-info, gen_esp32part).
    Prints the command, captures the output. Returns (exit code, output text)."""
    print(f"RUN: {shlex.join(str(part) for part in cmd)}")
    proc = subprocess.run(cmd, capture_output=True, text=True, env=clean_env(),
                          cwd=TOOLS_DIR)
    return proc.returncode, proc.stdout + proc.stderr


def require_esptool_v5():
    """The command lines here use esptool 5 spellings (write-flash, default-reset)."""
    try:
        version = importlib.metadata.version("esptool")
    except importlib.metadata.PackageNotFoundError:
        raise Failure(f"esptool is not installed for {sys.executable}; "
                      f"run this with {VENV_DIR / 'bin' / 'python'}")
    if not version.startswith("5."):
        raise Refusal(f"esptool {version} found, but these command lines need esptool 5.x")


# ---------------------------------------------------------------------------
# Finding the board (ports are listed, never opened)
# ---------------------------------------------------------------------------

def serial_ports():
    from serial.tools import list_ports
    return list(list_ports.comports())


def ports_with(vid_pid):
    return [p for p in serial_ports() if (p.vid, p.pid) == vid_pid]


def vid_pid_text(port):
    if port.vid is None or port.pid is None:
        return "----:----"
    return f"{port.vid:04X}:{port.pid:04X}"


def port_label(port):
    if (port.vid, port.pid) == DOWNLOAD_VID_PID:
        return "download mode (ESP32-S3 ROM, USB-Serial/JTAG)"
    if (port.vid, port.pid) == BRIDGE_VID_PID:
        return "bridge mode (UNO R4 WiFi running the USB bridge firmware)"
    return "other"


def bridge_hid_devices():
    """HID interfaces of boards in bridge mode (enumeration only, nothing is opened)."""
    import hid
    return hid.enumerate(*BRIDGE_VID_PID)


def distinct_boards(hid_infos):
    # One board should give one entry; key on serial number so a board that shows up
    # twice is not mistaken for two boards. Fall back to the path when there is none.
    keys = set()
    for info in hid_infos:
        keys.add(info.get("serial_number") or info.get("path"))
    return len(keys)


def resolve_download_port(args):
    """Guard (8): the port must be an ESP32-S3 in ROM download mode (303A:1001).
    In a dry run with no such port, '<PORT>' stands in for it."""
    ports = [p.device for p in ports_with(DOWNLOAD_VID_PID)]
    if args.port:
        if args.dry_run or args.port in ports:
            return args.port
        raise Refusal(f"--port {args.port} is not an ESP32-S3 download-mode port "
                      f"(303A:1001). Download-mode ports now: {ports or 'none'}")
    if len(ports) == 1:
        return ports[0]
    if len(ports) > 1:
        raise Refusal(f"more than one download-mode port: {ports}. Unplug the other "
                      "boards or pass --port.")
    if args.dry_run:
        print("note: no download-mode port (303A:1001) found; the dry run uses <PORT>")
        return "<PORT>"
    raise Refusal("no ESP32-S3 download-mode port (303A:1001) found. Run "
                  "'enter-download', or fit the GND-DOWNLOAD jumper and replug the board.")


def wait_for_download_port(timeout_s, ignore=()):
    """Wait for a new 303A:1001 port. Ports in 'ignore' were present before the reboot
    report was sent, so they belong to some other ESP32 device."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ports = [p.device for p in ports_with(DOWNLOAD_VID_PID) if p.device not in ignore]
        if len(ports) > 1:
            raise Refusal(f"more than one new download-mode port: {ports}")
        if len(ports) == 1:
            return ports[0]
        time.sleep(0.25)
    return None


# esptool prints the chip's MAC on connecting, as 'MAC:' padded to 20 characters
# (esptool 5.5.0 cmds.py read_mac). 'BASE MAC:' lines do not match the ^ anchor.
MAC_RE = re.compile(r"^MAC:\s+((?:[0-9a-f]{2}:){5}[0-9a-f]{2})\s*$", re.M | re.I)


def parse_mac(text):
    """The chip's MAC from esptool's 'MAC:' line, lower case, or None."""
    match = MAC_RE.search(text)
    return match.group(1).lower() if match else None


def mac_tag(mac):
    """'mac348518aabbcc-' for a backup file name; '' when the MAC is unknown."""
    return f"mac{mac.replace(':', '')}-" if mac else ""


def mac_from_backup_name(name):
    """The MAC that 'backup' put in a backup's file name, or None."""
    match = re.search(r"-mac([0-9a-f]{12})-", name)
    if not match:
        return None
    digits = match.group(1)
    return ":".join(digits[i:i + 2] for i in range(0, 12, 2))


# ---------------------------------------------------------------------------
# Image checks (pure functions on bytes; they raise Refusal)
# ---------------------------------------------------------------------------

def sidecar_path(path):
    return path.with_name(path.name + ".sha256")


def write_sidecar(path, digest):
    # Same format as 'shasum -a 256', so 'shasum -a 256 -c FILE.sha256' also works.
    sidecar_path(path).write_text(f"{digest}  {path.name}\n", encoding="utf-8")


def read_sidecar(path):
    sidecar = sidecar_path(path)
    if not sidecar.is_file():
        raise Refusal(f"backup sidecar {sidecar} is missing. A backup made by "
                      "'r4flash.py backup' always has one; without it the backup is unchecked.")
    # errors="replace": a damaged sidecar must end in the refusal below, not a traceback.
    words = sidecar.read_text(encoding="utf-8", errors="replace").split()
    digest = words[0].lower() if words else ""
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise Refusal(f"backup sidecar {sidecar} does not start with a sha256 hex digest")
    return digest


def check_backup_file(path):
    """Guard (1): the backup exists, its .sha256 sidecar matches, and it is a full-chip
    size. The file is read once, so the bytes checked here are the bytes used later.
    Returns a Backup; the caller compares its size and MAC with the chip."""
    if not path.is_file():
        raise Refusal(f"backup {path} does not exist. Make one first: "
                      "r4flash.py backup --out DIR")
    expected = read_sidecar(path)
    data = path.read_bytes()
    actual = sha256_bytes(data)
    if actual != expected:
        raise Refusal(f"backup {path} sha256 mismatch: file is {actual}, "
                      f"sidecar says {expected}. The backup is damaged or was changed.")
    if len(data) not in BACKUP_LENS:
        raise Refusal(f"backup {path} is {len(data):#x} bytes; a full backup is 0x400000 "
                      "(4MB) or 0x800000 (8MB)")
    return Backup(path, len(data), mac_from_backup_name(path.name), data)


def check_backup_layout(backup):
    """S3-APP.bin (0xE000) and a bare app (0x50000) keep the board's own partition
    table, so the backup (the board's flash as it was) must hold the unor4wifi table."""
    table = backup.data[PT_OFFSET:PT_OFFSET + PT_LEN]
    if sha256_bytes(table) != EXPECTED_PT_SHA256:
        raise Refusal("the backup's partition table at 0x8000 is not the unor4wifi table, "
                      "so app0 may not be at 0x50000 on this board. Write S3-BOOT-APP.bin "
                      "at 0x0 instead; it writes the table too.")


def check_known_shape(length, offset):
    """Guard (4) plus a whitelist: only the shapes the build produces may be written."""
    if length == S3_APP_LEN and offset != S3_APP_OFFSET:
        raise Refusal(f"this image has the size of S3-APP.bin ({S3_APP_LEN:#x}), which is "
                      f"S3-ALL[0xE000:0x1E0000] and must be written at 0xe000, not {offset:#x}")
    if length == S3_BOOT_APP_LEN and offset != 0:
        raise Refusal(f"this image has the size of S3-BOOT-APP.bin ({S3_BOOT_APP_LEN:#x}), "
                      f"which starts with the bootloader and must be written at 0x0, "
                      f"not {offset:#x}")
    if offset == 0 and length in (S3_BOOT_APP_LEN, S3_ALL_LEN):
        return
    if offset == S3_APP_OFFSET and length == S3_APP_LEN:
        return
    if offset == APP0_OFFSET:
        return  # a bare app (UNOR4USBBridge.ino.bin); check_app_slice() judges it
    raise Refusal(f"unknown image shape: {length:#x} bytes at {offset:#x}. Allowed: "
                  "S3-BOOT-APP.bin (0x1e0000) at 0x0, S3-APP.bin (0x1d2000) at 0xe000, "
                  "S3-ALL.bin (0x400000) at 0x0, or an app (<= 0x190000) at 0x50000")


def check_partition_table(data, offset):
    """Guard (6): an image that covers 0x8000 must carry exactly the unor4wifi table,
    so a default-layout image can never move app0, NVS or SPIFFS."""
    start = PT_OFFSET - offset
    if start < 0 or start >= len(data):
        return
    table = data[start:start + PT_LEN]
    digest = sha256_bytes(table)
    if len(table) < PT_LEN or digest != EXPECTED_PT_SHA256:
        raise Refusal(f"the partition table at 0x8000 in this image is not the project's "
                      f"unor4wifi.csv table (sha256 {digest[:16]}..., expected "
                      f"{EXPECTED_PT_SHA256[:16]}...). Was it built with another "
                      "partition scheme?")


def parse_image(region, where, limit):
    """Parse an ESP32-S3 image (bootloader or app) with esptool's own parser.
    Returns (length in bytes including the SHA-256, checksum ok, hash ok, chip id)."""
    from esptool.bin_image import ESP32S3FirmwareImage
    try:
        image = ESP32S3FirmwareImage(io.BytesIO(region))
    except Exception as err:
        # Any parse failure means we cannot vouch for the image, so the guard refuses.
        raise Refusal(f"the {where} does not parse as a complete ESP32-S3 image within "
                      f"{limit:#x} bytes (too big or truncated?): {err}")
    if not image.append_digest:
        # Every image the build makes has one; without it nothing proves the bytes.
        raise Refusal(f"the {where} has no appended SHA-256, so it cannot be verified")
    hash_ok = image.stored_digest == image.calc_digest
    checksum_ok = image.checksum == image.calculate_checksum()
    return image.data_length + 32, checksum_ok, hash_ok, image.chip_id


def check_esp_image(region, where, limit):
    """Magic byte, size, checksum, SHA-256 and chip id of one bootloader or app image."""
    if region[0] != IMAGE_MAGIC:
        raise Refusal(f"no image at the {where}: first byte {region[0]:#04x}, "
                      "expected 0xe9")
    length, checksum_ok, hash_ok, chip_id = parse_image(region, where, limit)
    if length > limit:
        raise Refusal(f"the {where} is {length:#x} bytes but only {limit:#x} fit there")
    if not (checksum_ok and hash_ok):
        raise Refusal(f"the {where} is corrupt: checksum ok={checksum_ok}, "
                      f"sha256 ok={hash_ok}")
    if chip_id != ESP32S3_CHIP_ID:
        raise Refusal(f"the {where} is for chip id {chip_id}, not ESP32-S3 (9)")


def check_boot_image(data, offset):
    """An image written at 0x0 carries the second-stage bootloader. A bad one stops
    the bridge from starting, which leaves only the jumper, so it gets a full check."""
    if offset == 0:
        check_esp_image(data[:PT_OFFSET], "bootloader at 0x0", PT_OFFSET)


def check_otadata_region(data, offset):
    """An image that covers otadata (0x9000-0xB000) must carry boot_app0.bin, which
    selects app0. Anything else could leave the board booting the old app in app1."""
    start = OTADATA_OFFSET - offset
    if start < 0 or start + OTADATA_LEN > len(data):
        return
    digest = sha256_bytes(data[start:start + OTADATA_LEN])
    if digest != BOOT_APP0_SHA256:
        raise Refusal(f"otadata at 0x9000 in this image is not boot_app0.bin (sha256 "
                      f"{digest[:16]}..., expected {BOOT_APP0_SHA256[:16]}...), so the "
                      "board might not boot app0")


def check_app_slice(data, offset):
    """Guard (7): the app that lands at 0x50000 must start with 0xE9, fit in app0
    (0x190000 bytes) and carry a valid checksum and SHA-256 for an ESP32-S3."""
    start = APP0_OFFSET - offset
    if start < 0 or start >= len(data):
        return
    if offset == APP0_OFFSET and len(data) > APP0_LEN:
        raise Refusal(f"the app is {len(data):#x} bytes but app0 holds only {APP0_LEN:#x} "
                      f"({len(data) - APP0_LEN} bytes too big)")
    check_esp_image(data[start:start + APP0_LEN], "app at 0x50000 (app0)", APP0_LEN)


def check_image(data, offset, allow_wipe):
    """Guards (2)-(7) for anything written to the bridge's first 4 MB."""
    end = offset + len(data)
    if not data:
        raise Refusal("the image is empty")
    if end > LAYOUT_END:
        raise Refusal(f"the image ends at {end:#x}, past {LAYOUT_END:#x} (the end of the "
                      "bridge layout). To write back a full backup use 'restore --backup'.")
    check_known_shape(len(data), offset)
    if end > SPIFFS_OFFSET and not allow_wipe:
        raise Refusal(f"this image reaches {end:#x}: it overwrites SPIFFS (0x370000) and "
                      "NVS (0x3f0000), erasing BLE bonds and Preferences. Use "
                      "S3-BOOT-APP.bin, or pass --i-know-this-wipes-nvs-and-spiffs.")
    if offset == 0 and data[0] != IMAGE_MAGIC:
        raise Refusal(f"an image written at 0x0 must start with the bootloader magic 0xe9; "
                      f"this one starts with {data[0]:#04x}")
    check_boot_image(data, offset)
    check_partition_table(data, offset)
    check_otadata_region(data, offset)
    check_app_slice(data, offset)


# ---------------------------------------------------------------------------
# esptool steps shared by backup, flash, restore and verify
# ---------------------------------------------------------------------------

def parse_flash_size(text):
    """Bytes from esptool's 'Detected flash size: 8MB' line, or None."""
    match = re.search(r"Detected flash size:\s*(\d+)\s*(MB|KB)", text)
    if not match:
        return None
    unit = MIB if match.group(2) == "MB" else 1024
    return int(match.group(1)) * unit


def identify_chip(port, dry_run):
    """flash-id: the chip's flash size and MAC. A dry run returns Chip(None, None)."""
    text = run(esptool_cmd(port, "no-reset", "flash-id"), dry_run)
    if dry_run:
        return Chip(None, None)
    size = parse_flash_size(text)
    if size is None:
        raise Failure("flash-id did not print a 'Detected flash size' line")
    mac = parse_mac(text)
    print(f"detected flash size: {size_label(size)}, MAC: {mac or 'not printed'}")
    return Chip(size, mac)


def check_backup_matches_chip(backup, chip, dry_run):
    """Guard (1), last part: the backup must come from this chip. The flash size must
    match, and so must the MAC when both are known."""
    if backup is None:
        return
    if dry_run:
        print("note: dry run, so the backup was not compared with the chip")
        return
    if backup.size != chip.size:
        raise Refusal(f"the backup is {size_label(backup.size)} but this chip's flash is "
                      f"{size_label(chip.size)}: it is not a backup of this board")
    if backup.mac and chip.mac and backup.mac != chip.mac:
        raise Refusal(f"the backup was read from the chip with MAC {backup.mac}, but the "
                      f"connected chip has MAC {chip.mac}: it is another board's backup")
    if not (backup.mac and chip.mac):
        print("note: the MAC of the backup's chip or of this chip is unknown, so only "
              "the flash size ties this backup to this board")


def print_next_steps(dry_run):
    prefix = "after a real run: " if dry_run else ""
    print(f"{prefix}next steps:")
    print("  1. Unplug the USB cable. If the GND-DOWNLOAD jumper is fitted, remove it.")
    print("  2. Plug the board back in. Pressing RST alone keeps the ESP32-S3 in download mode.")
    print("  3. 'r4flash.py ports' should show 2341:1002 (bridge mode) again.")
    print("  4. In the Arduino IDE, upload Blink to the UNO R4 WiFi to prove the bridge works.")


WRITE_FAILED_ADVICE = (
    "Do not unplug yet: write-flash runs with --after no-reset, so the ESP32-S3 should "
    "still be in download mode ('ports' shows 303A:1001). Run the same command again, "
    "or 'restore --backup BACKUP'.")
# esptool resets the chip in its teardown even when the command failed (esptool 5.5.0
# __init__.py, teardown under ctx.call_on_close), so after a failed verify-flash with
# --after hard-reset the chip has probably left download mode.
VERIFY_FAILED_ADVICE = (
    "Do not trust what was written. esptool resets the chip even when verify-flash fails, "
    "so the ESP32-S3 has probably left download mode and may be running this image. "
    "Enter download mode again ('enter-download', or the GND-DOWNLOAD jumper if the "
    "bridge no longer starts), then run 'restore --backup BACKUP'.")


def stage_image(data, name):
    """Write the checked bytes into a new private temp folder. esptool reads this copy,
    so a file rebuilt after the guards ran cannot be written instead.
    Returns (folder, path); the caller removes the folder."""
    folder = Path(tempfile.mkdtemp(prefix="r4flash-"))
    path = folder / name
    path.write_bytes(data)
    log(f"staged {len(data)} checked bytes (sha256 {sha256_bytes(data)}) at {path}")
    return folder, path


def write_steps(port, image_path, offset, dry_run):
    """write-flash (keep flags, --after no-reset), then verify-flash with --after
    hard-reset (on the S3 the hard reset clears FORCE_DOWNLOAD_BOOT)."""
    where = f"{offset:#x}"
    try:
        run(esptool_cmd(port, "no-reset", "write-flash", "--flash-mode", "keep",
                        "--flash-freq", "keep", "--flash-size", "keep", where, image_path),
            dry_run)
    except Failure as err:
        raise Failure(f"{err}\n{WRITE_FAILED_ADVICE}")
    try:
        run(esptool_cmd(port, "hard-reset", "verify-flash", where, image_path), dry_run)
    except Failure as err:
        raise Failure(f"{err}\n{VERIFY_FAILED_ADVICE}")


def write_and_verify(args, port, data, image_path, offset, backup):
    """flash-id and the backup/chip check, then write and verify the checked bytes."""
    chip = identify_chip(port, args.dry_run)
    check_backup_matches_chip(backup, chip, args.dry_run)
    if args.dry_run:
        print("note: a real run gives esptool a private copy of the checked bytes, "
              "not this path")
        write_steps(port, image_path, offset, True)
    else:
        folder, staged = stage_image(data, image_path.name)
        try:
            write_steps(port, staged, offset, False)
        finally:
            shutil.rmtree(folder, ignore_errors=True)
        print(f"flash OK: {image_path} (sha256 {sha256_bytes(data)}) written at "
              f"{offset:#x} and verified")
    print_next_steps(args.dry_run)


# ---------------------------------------------------------------------------
# ports
# ---------------------------------------------------------------------------

def cmd_ports(args):
    ports = serial_ports()
    print("Serial ports (listed only, never opened):")
    for port in ports:
        print(f"  {port.device:<36} {vid_pid_text(port)}  {port_label(port)}")
    if not ports:
        print("  (none)")
    found = any(port_label(p) != "other" for p in ports)
    try:
        boards = bridge_hid_devices()
    except ImportError:
        print("HID: hidapi is not importable here, so the HID check was skipped")
        boards = []
    state = "present" if boards else "not present"
    print(f"HID 2341:1002 (bridge CMSIS-DAP, used by enter-download): {state}")
    for info in boards:
        print(f"  {info.get('product_string')!s} serial={info.get('serial_number')!s} "
              f"release={info.get('release_number', 0):#06x}")
    if not found and not boards:
        print("no device found: no UNO R4 WiFi in bridge mode (2341:1002) "
              "or download mode (303A:1001)")
    return EXIT_OK


# ---------------------------------------------------------------------------
# enter-download
# ---------------------------------------------------------------------------

def send_reboot_report(path):
    """Open the bridge's HID interface and send the 0xAA feature report.
    Returns True when hidapi reported success."""
    import hid
    device = hid.device()
    log(f"HID: open_path({path!r}); send_feature_report({REBOOT_REPORT[:2].hex()} + 63 x 00)")
    try:
        device.open_path(path)
    except OSError as err:
        raise Failure(f"could not open the bridge HID interface: {err}")
    try:
        result = device.send_feature_report(REBOOT_REPORT)
    except OSError as err:
        result = -1
        print(f"note: send_feature_report raised: {err}")
    finally:
        device.close()
    log(f"HID: send_feature_report returned {result}")
    return result >= 0


def print_jumper_fallback():
    print("fallback: unplug the board, short GND to DOWNLOAD on the 6-pin ESP header next to")
    print("  the USB-C socket, plug the board in, then release the short. 'ports' should")
    print("  then show 303A:1001 (download mode).")


def cmd_enter_download(args):
    already = [p.device for p in ports_with(DOWNLOAD_VID_PID)]
    boards = bridge_hid_devices()
    if not boards:
        if already:
            print(f"already in download mode: {', '.join(already)} "
                  "(no 2341:1002 bridge device is present)")
            return EXIT_OK
        print("no device found: no UNO R4 WiFi bridge HID interface (2341:1002) is present")
        print_jumper_fallback()
        return EXIT_OK if args.dry_run else EXIT_FAIL
    if distinct_boards(boards) > 1:
        raise Refusal("more than one UNO R4 WiFi in bridge mode; unplug all but one")
    if already:
        # The R4 is still in bridge mode, so these ports belong to another ESP32 device.
        print(f"note: other download-mode ports are present ({', '.join(already)}); "
              "they are not this board, so later commands will need --port")
    path = boards[0]["path"]
    if args.dry_run:
        print(f"DRY: hid open_path {path!r}; send_feature_report(00 aa + 63 x 00)")
        print(f"DRY: then wait up to {ENTER_DOWNLOAD_WAIT_S:.0f} s for a new 303A:1001 port")
        return EXIT_OK
    if not send_reboot_report(path):
        # The board may restart before the USB transfer completes; the port decides.
        print("note: hidapi reported an error; waiting for the port anyway")
    port = wait_for_download_port(ENTER_DOWNLOAD_WAIT_S, ignore=already)
    if port is None:
        print(f"FAILED: no 303A:1001 port appeared within {ENTER_DOWNLOAD_WAIT_S:.0f} s")
        print_jumper_fallback()
        return EXIT_FAIL
    print(f"download mode: {port}")
    if already:
        print(f"pass --port {port} to backup, flash, restore and verify")
    return EXIT_OK


# ---------------------------------------------------------------------------
# backup
# ---------------------------------------------------------------------------

def backup_names(out_dir, size, mac, stamp):
    # The chip MAC in the name ties the backup to its board; 'flash' and 'restore'
    # compare it with the connected chip.
    name = f"r4-esp32s3-backup-{size}-{mac_tag(mac)}{stamp}.bin"
    return out_dir / name, out_dir / (name + ".partial"), out_dir / "copy2" / name


def check_read_size(partial, detected, read_text):
    got = partial.stat().st_size
    reported = parse_flash_size(read_text)
    if got != detected or (reported is not None and reported != detected):
        raise Refusal(f"read {got:#x} bytes but the chip reports {detected:#x}; "
                      f"the incomplete read is left at {partial}")


def make_second_copy(final, copy, digest):
    copy.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(final, copy)
    if sha256_file(copy) != digest:
        raise Failure(f"the second copy {copy} does not match the backup")
    write_sidecar(copy, digest)


def cmd_backup(args):
    require_esptool_v5()
    out_dir = Path(args.out).expanduser().resolve()
    port = resolve_download_port(args)
    stamp = utc_stamp()
    if args.dry_run:
        final, partial, copy = backup_names(out_dir, "<size>", "<MAC>", stamp)
        identify_chip(port, True)
        run(esptool_cmd(port, "no-reset", "read-flash", "0", "ALL", partial), True)
        run(esptool_cmd(port, "no-reset", "verify-flash", "0x0", partial), True)
        print(f"DRY: rename to {final}, write {sidecar_path(final).name}, copy to {copy}")
        return EXIT_OK
    out_dir.mkdir(parents=True, exist_ok=True)
    chip = identify_chip(port, False)
    final, partial, copy = backup_names(out_dir, size_label(chip.size), chip.mac, stamp)
    # Read into .partial; only a read that is the right size and verifies against the
    # chip gets the final name and a sidecar, so a final-named file is always good.
    text = run(esptool_cmd(port, "no-reset", "read-flash", "0", "ALL", partial), False)
    check_read_size(partial, chip.size, text)
    run(esptool_cmd(port, "no-reset", "verify-flash", "0x0", partial), False)
    digest = sha256_file(partial)
    partial.rename(final)
    write_sidecar(final, digest)
    make_second_copy(final, copy, digest)
    message = (f"backup OK: {final} ({chip.size} bytes, {size_label(chip.size)}, "
               f"MAC {chip.mac or 'unknown'}) sha256 {digest}")
    print(message)
    print(f"second copy: {copy}")
    log(message)
    return EXIT_OK


# ---------------------------------------------------------------------------
# flash
# ---------------------------------------------------------------------------

def read_image(path):
    if not path.is_file():
        raise Refusal(f"image {path} does not exist")
    return path.read_bytes()


def print_otadata_note(backup):
    # S3-APP.bin (0xE000) and a bare app (0x50000) do not write otadata (0x9000), so the
    # board keeps booting whichever app slot otadata already selects.
    print("note: this image does not reset otadata; if this board last booted app1 "
          "(after an ESP OTA) it will keep booting app1. S3-BOOT-APP.bin avoids this.")
    otadata = backup.data[OTADATA_OFFSET:OTADATA_OFFSET + OTADATA_LEN]
    if sha256_bytes(otadata) != BOOT_APP0_SHA256:
        print("WARNING: otadata in the backup is not boot_app0.bin, so this board may "
              "boot app1 and ignore what is written to app0")


def cmd_flash(args):
    image_path = Path(args.image).expanduser().resolve()
    backup = check_backup_file(Path(args.backup).expanduser().resolve())
    data = read_image(image_path)
    check_image(data, args.offset, args.i_know_this_wipes_nvs_and_spiffs)
    keeps_board_layout = args.offset in (S3_APP_OFFSET, APP0_OFFSET)
    if keeps_board_layout:
        check_backup_layout(backup)
    print(f"guards passed: {image_path.name}, {len(data):#x} bytes at {args.offset:#x}")
    if keeps_board_layout:
        print_otadata_note(backup)
    require_esptool_v5()
    port = resolve_download_port(args)
    write_and_verify(args, port, data, image_path, args.offset, backup)
    return EXIT_OK


# ---------------------------------------------------------------------------
# restore
# ---------------------------------------------------------------------------

def check_cached(path, expected_sha):
    digest = sha256_file(path)
    if digest != expected_sha:
        raise Refusal(f"cached file {path} has sha256 {digest}, expected {expected_sha}. "
                      "Delete it and run again.")


def https_context():
    """A TLS context that can verify github.com. When this Python has no CA certificates
    of its own, macOS's bundle is loaded. The sha256 pins still decide what is used."""
    context = ssl.create_default_context()
    if context.cert_store_stats()["x509_ca"] == 0 and SYSTEM_CA_FILE.is_file():
        context.load_verify_locations(cafile=str(SYSTEM_CA_FILE))
    return context


def download_official_zip(zip_path):
    zip_path.parent.mkdir(parents=True, exist_ok=True)
    part = zip_path.with_name(zip_path.name + ".part")
    print(f"RUN: download {OFFICIAL_ZIP_URL} -> {zip_path}", flush=True)
    log(f"RUN: download {OFFICIAL_ZIP_URL} -> {zip_path}")
    try:
        with urllib.request.urlopen(OFFICIAL_ZIP_URL, timeout=120,
                                    context=https_context()) as response:
            with open(part, "wb") as f:
                shutil.copyfileobj(response, f)
    except OSError as err:  # URLError and socket timeouts are OSErrors
        part.unlink(missing_ok=True)
        raise Failure(f"download failed: {err}")
    digest = sha256_file(part)
    if digest != OFFICIAL_ZIP_SHA256:
        part.unlink()
        raise Failure(f"downloaded zip has sha256 {digest}, expected {OFFICIAL_ZIP_SHA256}")
    part.rename(zip_path)


def extract_official_image(zip_path):
    with zipfile.ZipFile(zip_path) as archive:
        data = archive.read(OFFICIAL_MEMBER)
    digest = sha256_bytes(data)
    if digest != OFFICIAL_IMAGE_SHA256:
        raise Refusal(f"{OFFICIAL_MEMBER} has sha256 {digest}, expected {OFFICIAL_IMAGE_SHA256}")
    return data


def official_image(cache_dir, dry_run):
    """Returns (image path, image bytes). A dry run never downloads or writes to the
    cache; with nothing cached it returns (path, None) and says what it would do."""
    image_path = cache_dir / OFFICIAL_IMAGE_NAME
    zip_path = cache_dir / OFFICIAL_ZIP_NAME
    if image_path.is_file():
        check_cached(image_path, OFFICIAL_IMAGE_SHA256)
        return image_path, image_path.read_bytes()
    if not zip_path.is_file():
        if dry_run:
            print(f"DRY: download {OFFICIAL_ZIP_URL} -> {zip_path} "
                  f"(expect sha256 {OFFICIAL_ZIP_SHA256})")
            print(f"DRY: extract {OFFICIAL_MEMBER} -> {image_path} "
                  f"(expect sha256 {OFFICIAL_IMAGE_SHA256})")
            return image_path, None
        download_official_zip(zip_path)
    check_cached(zip_path, OFFICIAL_ZIP_SHA256)
    data = extract_official_image(zip_path)
    if dry_run:
        print(f"DRY: extract {OFFICIAL_MEMBER} -> {image_path} (sha256 checked in memory)")
    else:
        image_path.write_bytes(data)
    return image_path, data


def print_no_backup_warning():
    bar = "!" * 78
    print(bar)
    print("!! WARNING: no --backup given. Nothing proves a copy of this board's current")
    print("!! flash exists. The official image rewrites 0x0-0x1E0000 (bootloader, partition")
    print("!! table, otadata, certificates, app0); NVS and SPIFFS are left as they are.")
    print("!! Take one first if the board still answers: r4flash.py backup --out DIR")
    print(bar)


def restore_official(args):
    backup = None
    if args.backup:
        backup = check_backup_file(Path(args.backup).expanduser().resolve())
    else:
        print_no_backup_warning()
    require_esptool_v5()
    # Guard (8) before the download, so a missing board is reported before 5 MB arrive.
    port = resolve_download_port(args)
    cache_dir = Path(args.cache_dir).expanduser().resolve()
    image_path, data = official_image(cache_dir, args.dry_run)
    if data is None:
        print("note: dry run with an empty cache: the image guards run after the download")
    else:
        check_image(data, 0, allow_wipe=False)
        print(f"guards passed: {image_path.name}, {len(data):#x} bytes at 0x0")
    write_and_verify(args, port, data, image_path, 0, backup)
    return EXIT_OK


def warn_if_foreign_table(data):
    # A backup restores whatever the board had, so a different table is not refused.
    table = data[PT_OFFSET:PT_OFFSET + PT_LEN]
    if sha256_bytes(table) != EXPECTED_PT_SHA256:
        print("note: this backup's partition table is not the unor4wifi table; restoring "
              "it puts back the layout the board had when the backup was taken")


def warn_if_bad_bootloader(data):
    # Same reason: a backup holds what the board had, so this is reported, not refused.
    try:
        check_boot_image(data, 0)
    except Refusal as err:
        print(f"WARNING: in this backup, {err}")


def restore_backup(args):
    backup = check_backup_file(Path(args.backup).expanduser().resolve())
    if backup.data[0] != IMAGE_MAGIC:
        raise Refusal(f"backup {backup.path} does not start with the bootloader magic 0xe9")
    warn_if_foreign_table(backup.data)
    warn_if_bad_bootloader(backup.data)
    print(f"guards passed: {backup.path.name}, {size_label(backup.size)} full backup at 0x0")
    require_esptool_v5()
    port = resolve_download_port(args)
    write_and_verify(args, port, backup.data, backup.path, 0, backup)
    return EXIT_OK


def cmd_restore(args):
    if args.official:
        return restore_official(args)
    if args.backup:
        return restore_backup(args)
    raise Refusal("restore needs --official or --backup FILE")


# ---------------------------------------------------------------------------
# verify
# ---------------------------------------------------------------------------

def cmd_verify(args):
    image_path = Path(args.image).expanduser().resolve()
    data = read_image(image_path)
    if args.offset + len(data) > max(BACKUP_LENS):
        raise Refusal(f"{len(data):#x} bytes at {args.offset:#x} is past the 8 MB chip")
    require_esptool_v5()
    port = resolve_download_port(args)
    where = f"{args.offset:#x}"
    run(esptool_cmd(port, "no-reset", "verify-flash", where, image_path), args.dry_run)
    if not args.dry_run:
        print(f"verify OK: flash at {where} matches {image_path}")
    return EXIT_OK


# ---------------------------------------------------------------------------
# check (the no-board dry run of a build)
# ---------------------------------------------------------------------------

BUILD_FILES = {
    "all": "S3-ALL.bin",
    "boot_app": "S3-BOOT-APP.bin",
    "app_part": "S3-APP.bin",
    "app": "UNOR4USBBridge.ino.bin",
    "partitions": "UNOR4USBBridge.ino.partitions.bin",
    "bootloader": "UNOR4USBBridge.ino.bootloader.bin",
    "certs": "x509_crt_bundle",
}


def add(rows, ok, name, detail=""):
    """ok: True = PASS, False = FAIL, None = WARN (reported, does not fail the check)."""
    status = {True: "PASS", False: "FAIL", None: "WARN"}[ok]
    rows.append((status, name, detail))


def check_python(rows):
    in_venv = Path(sys.prefix).resolve() == VENV_DIR.resolve()
    detail = f"{sys.version.split()[0]} at {sys.executable}"
    add(rows, True if in_venv else None, "python (project venv)", detail)


def check_esptool(rows):
    code, text = run_offline([sys.executable, "-m", "esptool", "version"])
    lines = text.strip().splitlines()
    version = lines[-1].strip() if lines else ""
    add(rows, code == 0 and version.startswith("5."), "esptool 5.x", version or text.strip())


def check_imports(rows):
    try:
        import hid
        add(rows, True, "hidapi import", f"hid {hid.version_str()}")
    except ImportError as err:
        add(rows, False, "hidapi import", str(err))
    try:
        import serial
        add(rows, True, "pyserial import", f"pyserial {serial.__version__}")
    except ImportError as err:
        add(rows, False, "pyserial import", str(err))


def check_esptool_config(rows):
    # esptool reads esptool.cfg/setup.cfg/tox.ini from cwd, ~/.config/esptool and ~.
    home = Path.home()
    found = []
    for folder in (TOOLS_DIR, home / ".config" / "esptool", home):
        for name in ("esptool.cfg", "setup.cfg", "tox.ini"):
            if (folder / name).is_file():
                found.append(str(folder / name))
    add(rows, None if found else True, "no esptool config files", ", ".join(found) or "none")


def load_build(build_dir, rows):
    # An empty file is treated as missing, so no comparison row can pass on nothing.
    images = {}
    for key, name in BUILD_FILES.items():
        path = build_dir / name
        data = path.read_bytes() if path.is_file() else b""
        if data:
            images[key] = data
        elif path.is_file():
            add(rows, False, f"{name} present", "the file is empty")
        else:
            add(rows, False, f"{name} present", f"missing in {build_dir}")
    return images


def check_sizes(images, rows):
    expected = {"all": S3_ALL_LEN, "boot_app": S3_BOOT_APP_LEN, "app_part": S3_APP_LEN,
                "partitions": PT_LEN}
    for key, want in expected.items():
        if key in images:
            got = len(images[key])
            add(rows, got == want, f"{BUILD_FILES[key]} size", f"{got:#x} (want {want:#x})")


def check_slices(images, rows):
    if "all" not in images:
        return
    full = images["all"]
    if "boot_app" in images:
        add(rows, images["boot_app"] == full[:S3_BOOT_APP_LEN],
            "S3-BOOT-APP == S3-ALL[:0x1E0000]")
    if "app_part" in images:
        add(rows, images["app_part"] == full[S3_APP_OFFSET:S3_BOOT_APP_LEN],
            "S3-APP == S3-ALL[0xE000:0x1E0000]")


def check_magic(boot_app, rows):
    for offset in (0, APP0_OFFSET):
        got = boot_app[offset] if len(boot_app) > offset else None
        detail = f"{got:#04x}" if got is not None else "image too short"
        add(rows, got == IMAGE_MAGIC, f"magic 0xE9 at {offset:#x} of S3-BOOT-APP", detail)


def check_app_size(images, rows):
    if "app" not in images:
        return
    app = images["app"]
    headroom = APP0_LEN - len(app)
    detail = (f"{len(app)} of {APP0_LEN} bytes, headroom {headroom} bytes "
              f"({100.0 * headroom / APP0_LEN:.1f}%)")
    if headroom < 0:
        detail = f"{len(app)} of {APP0_LEN} bytes: {-headroom} bytes too big"
    add(rows, headroom >= 0, "app fits app0 (<= 0x190000)", detail)
    if "boot_app" in images and headroom >= 0:
        boot_app = images["boot_app"]
        slot = boot_app[APP0_OFFSET:APP0_OFFSET + APP0_LEN]
        padded = app + b"\xff" * (APP0_LEN - len(app))
        add(rows, slot == padded, "app0 slice == UNOR4USBBridge.ino.bin + 0xFF")


def check_bootloader(images, rows):
    if "bootloader" in images and "boot_app" in images:
        boot = images["bootloader"]
        add(rows, images["boot_app"][:len(boot)] == boot,
            "bootloader slice == UNOR4USBBridge.ino.bootloader.bin", f"{len(boot)} bytes")


def check_certs(images, rows):
    # combine.py writes the bundle at 0xB000 and then the app at 0x50000, so a bundle
    # over 0x45000 bytes would be cut short by the app without any error.
    if "certs" not in images or "boot_app" not in images:
        return
    certs = images["certs"]
    add(rows, len(certs) <= CERT_LEN, "x509_crt_bundle fits cert (<= 0x45000)",
        f"{len(certs)} bytes")
    slot = images["boot_app"][CERT_OFFSET:CERT_OFFSET + len(certs)]
    add(rows, slot == certs, "cert slice at 0xB000 == x509_crt_bundle")


def generate_partition_table():
    """Run the core's own gen_esp32part.py on unor4wifi.csv. Returns (bytes or None, why)."""
    if not (GEN_ESP32PART.is_file() and PARTITION_CSV.is_file()):
        return None, "gen_esp32part.py or unor4wifi.csv not found under firmware/"
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "partitions.bin"
        cmd = [sys.executable, GEN_ESP32PART, "-q", PARTITION_CSV, out]
        code, text = run_offline(cmd)
        if code != 0 or not out.is_file():
            return None, text.strip()
        return out.read_bytes(), ""


def check_partition_rows(images, rows):
    generated, why = generate_partition_table()
    if generated is None:
        add(rows, None, "table from unor4wifi.csv (gen_esp32part)", why)
    else:
        add(rows, sha256_bytes(generated) == EXPECTED_PT_SHA256,
            "table from unor4wifi.csv == embedded sha256", sha256_bytes(generated)[:16])
    if "partitions" in images:
        add(rows, sha256_bytes(images["partitions"]) == EXPECTED_PT_SHA256,
            "UNOR4USBBridge.ino.partitions.bin == expected table")
    if "boot_app" in images:
        boot_app = images["boot_app"]
        table = boot_app[PT_OFFSET:PT_OFFSET + PT_LEN]
        tail = boot_app[PT_OFFSET + PT_LEN:OTADATA_OFFSET]
        add(rows, sha256_bytes(table) == EXPECTED_PT_SHA256, "partition table at 0x8000",
            sha256_bytes(table)[:16])
        add(rows, tail == b"\xff" * len(tail), "0x8C00-0x9000 erased (0xFF)")


def check_otadata(images, rows):
    if "boot_app" not in images:
        return
    otadata = images["boot_app"][OTADATA_OFFSET:OTADATA_OFFSET + OTADATA_LEN]
    add(rows, sha256_bytes(otadata) == BOOT_APP0_SHA256, "otadata at 0x9000 == boot_app0.bin")
    if BOOT_APP0_BIN.is_file():
        add(rows, BOOT_APP0_BIN.read_bytes() == otadata, "firmware/boot/boot_app0.bin matches")


def image_info_verdict(text):
    checksum_ok = re.search(r"^Checksum: .*\(valid\)", text, re.M) is not None
    hash_ok = re.search(r"^Validation hash: .*\(valid\)", text, re.M) is not None
    chip_ok = "Chip ID: 9 (ESP32-S3)" in text
    detail = f"checksum valid={checksum_ok}, hash valid={hash_ok}, ESP32-S3={chip_ok}"
    return checksum_ok and hash_ok and chip_ok, detail


def check_image_info(region, name, rows):
    # esptool image-info exits 0 even for a corrupt image, so its text is parsed.
    with tempfile.TemporaryDirectory() as tmp:
        slice_path = Path(tmp) / "slice.bin"
        slice_path.write_bytes(region)
        cmd = [sys.executable, "-m", "esptool", "--chip", CHIP, "image-info", slice_path]
        code, text = run_offline(cmd)
    ok, detail = image_info_verdict(text)
    add(rows, code == 0 and ok, f"esptool image-info on {name}", detail)


def check_guards(images, rows):
    shapes = (("boot_app", 0, False), ("app_part", S3_APP_OFFSET, False), ("all", 0, True))
    for key, offset, wipe in shapes:
        if key not in images:
            continue
        name = f"flash guards: {BUILD_FILES[key]} @ {offset:#x}"
        try:
            check_image(images[key], offset, wipe)
            add(rows, True, name)
        except Refusal as err:
            add(rows, False, name, str(err))


def print_table(rows):
    width = max(len(name) for _, name, _ in rows)
    for status, name, detail in rows:
        print(f"{status:<5} {name:<{width}}  {detail}".rstrip())


def cmd_check(args):
    build_dir = DEFAULT_BUILD_DIR
    if args.build_dir:
        build_dir = Path(args.build_dir).expanduser().resolve()
    print(f"check: build dir {build_dir} (offline; no device is touched)")
    if args.dry_run:
        print("note: check never touches a device, so --dry-run changes nothing here; "
              "it still runs its offline commands")
    rows = []
    check_python(rows)
    check_esptool(rows)
    check_imports(rows)
    check_esptool_config(rows)
    images = load_build(build_dir, rows)
    check_sizes(images, rows)
    check_slices(images, rows)
    check_bootloader(images, rows)
    check_certs(images, rows)
    check_app_size(images, rows)
    check_partition_rows(images, rows)
    check_otadata(images, rows)
    if "boot_app" in images:
        boot_app = images["boot_app"]
        check_magic(boot_app, rows)
        check_image_info(boot_app[:PT_OFFSET], "bootloader slice", rows)
        check_image_info(boot_app[APP0_OFFSET:APP0_OFFSET + APP0_LEN], "app0 slice", rows)
    check_guards(images, rows)
    print()
    print_table(rows)
    failures = sum(1 for status, _, _ in rows if status == "FAIL")
    print(f"\ncheck: {'FAILED' if failures else 'OK'} ({failures} failure(s), "
          f"{len(rows)} checks)")
    return EXIT_FAIL if failures else EXIT_OK


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

def common_options():
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--dry-run", action="store_true",
                        help="print every external command as 'DRY: ...' and run nothing "
                             "that touches a device")
    common.add_argument("--port",
                        help="serial port of the board in download mode (default: the one "
                             "303A:1001 port; refuses if there are 0 or several)")
    return common


def add_simple_commands(sub, common):
    p = sub.add_parser("ports", parents=[common],
                       help="list serial ports and the bridge HID device (opens nothing)")
    p.set_defaults(func=cmd_ports)
    p = sub.add_parser("check", parents=[common],
                       help="offline check of a build's images and the tools (no board; "
                            "--dry-run changes nothing)")
    p.add_argument("--build-dir", help=f"default: {DEFAULT_BUILD_DIR}")
    p.set_defaults(func=cmd_check)
    p = sub.add_parser("enter-download", parents=[common],
                       help="send the bridge's HID reboot report, wait for 303A:1001")
    p.set_defaults(func=cmd_enter_download)
    p = sub.add_parser("backup", parents=[common],
                       help="read the whole flash, verify it, keep two copies")
    p.add_argument("--out", required=True, help="directory for the backup files")
    p.set_defaults(func=cmd_backup)


def add_write_commands(sub, common):
    p = sub.add_parser("flash", parents=[common], help="write an image, guarded")
    p.add_argument("image")
    p.add_argument("--offset", type=parse_offset, default=0, help="default 0x0")
    p.add_argument("--backup", required=True, help="a full backup made by 'backup'")
    p.add_argument("--i-know-this-wipes-nvs-and-spiffs", action="store_true",
                   help="allow an image that reaches SPIFFS/NVS (S3-ALL.bin)")
    p.set_defaults(func=cmd_flash)
    p = sub.add_parser("restore", parents=[common],
                       help="write back Arduino's 0.6.0 image or a full backup")
    p.add_argument("--official", action="store_true",
                   help="download (once, sha256-checked) and write Arduino's 0.6.0 image")
    p.add_argument("--backup", help="alone: the full backup to write back; with --official: "
                                    "the safety backup to check first, as for 'flash'")
    p.add_argument("--cache-dir", default=str(DEFAULT_CACHE_DIR), help="default: tools/cache")
    p.set_defaults(func=cmd_restore)
    p = sub.add_parser("verify", parents=[common],
                       help="compare flash with an image (verify-flash, --after no-reset)")
    p.add_argument("image")
    p.add_argument("--offset", type=parse_offset, default=0, help="default 0x0")
    p.set_defaults(func=cmd_verify)


def build_parser():
    parser = argparse.ArgumentParser(
        prog="r4flash.py",
        description="Guarded flashing of the UNO R4 WiFi's ESP32-S3 bridge firmware. "
                    "Exit codes: 0 ok, 1 failed, 2 usage error, 3 refused by a guard.")
    sub = parser.add_subparsers(dest="command", required=True)
    common = common_options()
    add_simple_commands(sub, common)
    add_write_commands(sub, common)
    return parser


INTERRUPTED_ADVICE = (
    "interrupted. If a write was running, the flash may be half written. Do not unplug. "
    "Run 'ports': with 303A:1001 present, run the same command again or "
    "'restore --backup BACKUP'; without it, enter download mode again "
    "('enter-download', or the GND-DOWNLOAD jumper) and run 'restore --backup BACKUP'.")


def fail(message):
    print(f"FAILED: {message}", file=sys.stderr)
    log(f"FAILED: {message}")
    return EXIT_FAIL


def main(argv=None):
    args = build_parser().parse_args(argv)
    device_run = args.command in DEVICE_COMMANDS and not args.dry_run
    if device_run:
        start_log(args.command)
    try:
        return args.func(args)
    except Refusal as err:
        print(f"REFUSED: {err}", file=sys.stderr)
        log_if_started(f"REFUSED: {err}")
        return EXIT_REFUSED
    except Failure as err:
        return fail(err)
    except ImportError as err:
        return fail(f"{err}. Run with {VENV_DIR / 'bin' / 'python'}")
    except KeyboardInterrupt:
        return fail(INTERRUPTED_ADVICE if device_run else "interrupted")
    except (OSError, ValueError) as err:
        # Unreadable files, a directory where a file was expected and similar input errors.
        return fail(f"{type(err).__name__}: {err}")
    except Exception as err:
        # A bug in this tool. The clear line comes first; the traceback is for fixing it.
        log(traceback.format_exc())
        code = fail(f"unexpected {type(err).__name__}: {err} (a bug in r4flash.py)")
        traceback.print_exc()
        return code


if __name__ == "__main__":
    sys.exit(main())
