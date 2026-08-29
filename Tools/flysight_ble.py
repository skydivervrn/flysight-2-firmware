#!/usr/bin/env python3
"""Inspect and flash a FlySight 2 over Bluetooth, from a Mac — no tablet, no USB.

Three subcommands, in increasing order of consequence:

  --info                     read-only: version, batch, mode, battery, MTU
  --upload FILE --dest PATH  write a file, read it back, compare sha256
  --flash FILE.sfb           the real thing; dry-run unless --really-install

Protocol sources, in order of authority: the Groundrush app's device layer
(`lib/ble/crs_client.dart`, `crs_framing.dart`, `opcodes.dart`,
`lib/services/firmware_update_service.dart`), then the firmware in this repo
(`FlySight/crs.c`, `FlySight/device_state.c`, `FlySight/state.c`,
`STM32_WPAN/App/custom_stm.c`), then `Docs/ble.md`. Every constant below cites
the line it came from so it can be re-checked rather than trusted.

Run it in the bleak venv, with the macOS sandbox OFF — CoreBluetooth is not
reachable from a sandboxed process:

    ~/.venvs/ble/bin/python Tools/flysight_ble.py --info

Bonding is a manual, one-time step: `bleak.pair()` raises NotImplementedError
on CoreBluetooth, so the first connection needs the FlySight in PAIRING mode
(double-press) and the macOS system dialog. See Docs/FLASH_OVER_BLE.md.

Every command here is one process, which means it misses that 30 s window more
often than it catches it. `Tools/flysight_app.py` imports this module into a
process that stays up, holds the link and keeps retrying, and puts a page and a
JSON API on 127.0.0.1 — same frames, same gates, same uploader.
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import hashlib
import os
import re
import sys
import time
from dataclasses import dataclass

try:  # Only the radio needs bleak; every pure helper below imports clean.
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover - exercised only outside the venv
    BleakClient = BleakScanner = None

# ---------------------------------------------------------------------------
# Wire constants
# ---------------------------------------------------------------------------

# UUIDs: STM32_WPAN/App/custom_stm.c:150-161 (COPY_*_UUID, MSB first).
FT_PACKET_OUT = "00000001-8e22-4541-9d4c-21edae82ed19"  # notify, device -> host
FT_PACKET_IN = "00000002-8e22-4541-9d4c-21edae82ed19"   # write w/o response
DS_MODE = "00000005-8e22-4541-9d4c-21edae82ed19"        # read + indicate
DS_CONTROL_POINT = "00000007-8e22-4541-9d4c-21edae82ed19"  # write + indicate
BATTERY_LEVEL = "00002a19-0000-1000-8000-00805f9b34fb"  # read + notify

ADV_NAME = "FlySight"  # STM32_WPAN/App/app_ble.c advertising payload

# File transfer (CRS) opcodes — FlySight/crs.c:49-61.
CRS_CREATE = 0x00
CRS_DELETE = 0x01
CRS_READ = 0x02
CRS_WRITE = 0x03
CRS_MK_DIR = 0x04
CRS_READ_DIR = 0x05
CRS_FILE_DATA = 0x10
CRS_FILE_INFO = 0x11
CRS_FILE_ACK = 0x12
CRS_NAK = 0xF0
CRS_ACK = 0xF1
CRS_PING = 0xFE
CRS_CANCEL = 0xFF

# Payload bytes per data frame (FRAME_LENGTH, FlySight/crs.c:39). The device's
# read path hands out exactly this much and counts read offsets in these units.
FRAME_LENGTH = 242

# Go-Back-N window (FS_CRS_WINDOW_LENGTH, FlySight/crs.h:27). The device's
# receive ring is the same size and silently drops the overflow
# (STM32_WPAN/App/custom_app.c:808-822), so never exceed it.
WINDOW_LENGTH = 8

# Device State control point — FlySight/device_state.c:39-50.
DS_GET_FW_VERSION = 0x01
DS_REBOOT = 0x02
DS_GET_DEVICE_ID = 0x03
DS_INSTALL_UPLOADED_FIRMWARE = 0x04
DS_REQUEST_SLEEP = 0x10
DS_REQUEST_ACTIVE = 0x11

# Control-point framing — STM32_WPAN/App/control_point_protocol.h:29-41.
CP_RESPONSE_ID = 0xF0
CP_STATUS = {
    0x01: "success",
    0x02: "not supported",
    0x03: "invalid parameter",
    0x04: "operation failed",
    0x05: "not permitted",
    0x06: "busy",
    0x07: "unknown error",
}
CP_SUCCESS = 0x01
CP_NOT_SUPPORTED = 0x02

# DS_Mode values — FlySight/mode.h:41-46. The order is not alphabetical.
MODE_SLEEP = 0x00
MODE_ACTIVE = 0x01
MODE_CONFIG = 0x02
MODE_USB = 0x03
MODE_PAIRING = 0x04
MODE_START = 0x05
MODE_NAMES = {
    MODE_SLEEP: "SLEEP",
    MODE_ACTIVE: "ACTIVE",
    MODE_CONFIG: "CONFIG",
    MODE_USB: "USB",
    MODE_PAIRING: "PAIRING",
    MODE_START: "START",
}

# Production batches, keyed by the first 8 hex characters of Pubkey_X.
# Derived from Deploy/Public_Keys/pub_key_b*.bin, which are 65-byte
# uncompressed points (0x04 || X[32] || Y[32]) — so X starts at byte 1.
# Tests/test_flysight_ble.py re-derives this table from those files.
BATCHES = {
    "486bee2d": "B2",
    "211d721d": "B3",
    "dac40d25": "B4",
    "3157e028": "B5",
    "8ee78709": "B6",
    "72816871": "B7",
    "b36f45a1": "B8",
    "df8afa69": "B9",
}

FLYSIGHT_TXT = "/flysight.txt"

# Where the SBSFU bootloader takes its image from. INFERRED, not read out of
# any source in this repo: the bootloader is a prebuilt binary and names the
# path nowhere. It is the path the USB file-drop uses (Docs/BUILDING.md §4).
FIRMWARE_PATH = "/FW/APP.SFB"
FIRMWARE_DIR = "/FW"

# Gate thresholds.
MIN_BATTERY_PCT = 50
# One data frame is 244 bytes on the wire (opcode + counter + 242), and a
# write-without-response carries ATT_MTU - 3. The device caps the MTU at 250
# (CFG_BLE_MAX_ATT_MTU, Core/Inc/app_conf.h:227) and its characteristic is
# 244 bytes long (SizeFt_Packet_In, custom_stm.c:79).
MIN_MTU = 247

# Timeouts, mirroring the app's CrsClient defaults.
COMMAND_TIMEOUT = 10.0   # ACK/NAK that opens an operation (SD card spin-up)
PACKET_TIMEOUT = 5.0     # a download going quiet
ACK_TIMEOUT = 0.5        # an upload waiting for the ACK of its oldest frame
KEEPALIVE_INTERVAL = 10.0  # device drops the link after 30 s of silence
MAX_RETRIES = 8


# ---------------------------------------------------------------------------
# Pure: CRS framing
# ---------------------------------------------------------------------------

def path_payload(path: str) -> bytes:
    """NUL-terminated path, as every path-carrying command wants it.

    The firmware terminates the payload itself for read/write/readdir
    (crs.c:200,249,308) but not for create/delete/mkdir (crs.c:152,177,285),
    which pass the buffer straight to FatFs. Terminating always is uniform and
    correct for both.
    """
    return path.encode("ascii") + b"\x00"


def build_write_file(path: str) -> bytes:
    """0x03 — open for writing, truncating (FA_CREATE_ALWAYS, crs.c:252)."""
    return bytes([CRS_WRITE]) + path_payload(path)


def build_mkdir(path: str) -> bytes:
    return bytes([CRS_MK_DIR]) + path_payload(path)


def build_read_file(path: str, offset_frames: int = 0, stride_frames: int = 1) -> bytes:
    """0x02 — read, with offset and stride counted in 242-byte frames.

    The device seeks to `offset_frames * 242` and advances `stride_frames * 242`
    after each frame (crs.c:206-207), so stride 1 is a contiguous read and
    anything larger returns a decimated file.
    """
    if offset_frames < 0:
        raise ValueError("offset_frames must be zero or greater")
    if stride_frames < 1:
        raise ValueError("stride_frames must be at least 1 (1 = contiguous)")
    return (
        bytes([CRS_READ])
        + offset_frames.to_bytes(4, "little")
        + (stride_frames - 1).to_bytes(4, "little")
        + path_payload(path)
    )


def build_file_data(counter: int, data: bytes) -> bytes:
    """0x10 — one data frame. Zero-length data is the end-of-file marker."""
    if len(data) > FRAME_LENGTH:
        raise ValueError(f"data must not exceed {FRAME_LENGTH} bytes")
    return bytes([CRS_FILE_DATA, counter & 0xFF]) + data


def build_file_ack(counter: int) -> bytes:
    return bytes([CRS_FILE_ACK, counter & 0xFF])


def build_ping() -> bytes:
    return bytes([CRS_PING])


def build_cancel() -> bytes:
    return bytes([CRS_CANCEL])


def opcode_of(packet: bytes):
    """First byte of an inbound packet, or None for an empty one."""
    return packet[0] if packet else None


def parse_file_data(packet: bytes):
    """(counter, payload) of a 0x10 packet. Empty payload = end of file."""
    if len(packet) < 2:
        raise ValueError(f"file-data packet needs 2 bytes, got {len(packet)}")
    if packet[0] != CRS_FILE_DATA:
        raise ValueError(f"expected opcode 0x10, got 0x{packet[0]:02x}")
    return packet[1], bytes(packet[2:])


def parse_ack_nak(packet: bytes) -> int:
    """The opcode an ACK (0xF1) or NAK (0xF0) refers to (crs.c:127,132)."""
    if len(packet) < 2:
        raise ValueError(f"ack/nak packet needs 2 bytes, got {len(packet)}")
    if packet[0] not in (CRS_ACK, CRS_NAK):
        raise ValueError(f"expected opcode 0xF1 or 0xF0, got 0x{packet[0]:02x}")
    return packet[1]


def decode_cp_response(raw: bytes):
    """Decode `[0xF0][echoed opcode][status][data…]`, or None if malformed.

    A malformed indication must never complete a pending request, so this
    rejects rather than guesses (control_point_protocol.h:29-41).
    """
    if len(raw) < 3 or raw[0] != CP_RESPONSE_ID:
        return None
    return raw[1], raw[2], bytes(raw[3:])


def cp_text(data: bytes) -> str:
    """Response data as ASCII, trailing NULs stripped (device_state.c:147)."""
    return data.rstrip(b"\x00").decode("ascii", "replace")


# ---------------------------------------------------------------------------
# Pure: the Go-Back-N sender
# ---------------------------------------------------------------------------

class UploadWindow:
    """The uploader's whole state machine, with no radio attached.

    Mirrors `CrsClient.writeFile` in the app (`lib/ble/crs_client.dart:528-608`):
    send up to `window` frames ahead of the oldest unacknowledged one, advance
    only on the ACK the device is expected to send next, and on silence rewind
    the whole window.

    The one rule that matters more than the rest: **the zero-length end-of-file
    frame is held back until every data frame has been acknowledged.**
    `FlySight/crs.c:558-561` closes the file on *any* two-byte data packet,
    and that check sits outside the counter test at `:547` — so an EOF that
    slips into the window while a data frame is still unacknowledged makes the
    device close a **truncated** file and go idle. Every retransmission after
    that comes back as `NAK 0x10`, and the image that gets installed is short.
    """

    def __init__(self, data: bytes, window: int = WINDOW_LENGTH,
                 frame_length: int = FRAME_LENGTH):
        if not 1 <= window <= WINDOW_LENGTH:
            raise ValueError(
                f"window must be 1..{WINDOW_LENGTH}; the device buffers no more "
                "(custom_app.c:808)")
        self.data = data
        self.window = window
        self.frame_length = frame_length
        self.data_frames = (len(data) + frame_length - 1) // frame_length
        # One extra frame: the empty packet that closes the file.
        self.total_frames = self.data_frames + 1
        self.base = 0  # oldest unacknowledged frame, unbounded
        self.next = 0  # next frame to put on the wire, unbounded

    @property
    def done(self) -> bool:
        return self.base >= self.total_frames

    @property
    def eof_sent(self) -> bool:
        return self.next > self.data_frames

    def frame_at(self, index: int) -> bytes:
        """Frame `index`: a slice of the data, or the EOF packet past the end."""
        if index >= self.data_frames:
            return build_file_data(index & 0xFF, b"")
        start = index * self.frame_length
        return build_file_data(index & 0xFF,
                               self.data[start:start + self.frame_length])

    def pending(self):
        """Frames to put on the wire right now, advancing `next` past them."""
        out = []
        while (self.next < self.base + self.window
               and self.next < self.total_frames
               # The EOF frame waits for every data frame to be acknowledged.
               and (self.next < self.data_frames or self.base == self.data_frames)):
            out.append(self.frame_at(self.next))
            self.next += 1
        return out

    def on_ack(self, counter: int) -> bool:
        """Feed one ACK. True if it advanced the window, False if it was stale.

        The device acknowledges a frame only when its counter is the one it was
        waiting for, and only after writing it (crs.c:547-552). So an ACK for
        any frame still in flight means every frame up to it was written, and
        the window advances past the lot.

        **This is the one place that does not mirror the app.**
        `crs_client.dart:586` advances only on an exact match with `base` and
        drops everything else as a stale duplicate — which deadlocks if a
        single ACK goes missing: the device has moved on and will only ever
        acknowledge later frames, while the sender rewinds to a frame the
        device now ignores. Reading an ACK cumulatively cannot skip a frame,
        because the device never acknowledges out of order.
        """
        in_flight = self.next - self.base
        ahead = (counter - (self.base & 0xFF)) & 0xFF
        if ahead >= in_flight:
            return False  # nothing in flight, or an ACK from before a rewind
        self.base += ahead + 1
        return True

    def on_timeout(self) -> None:
        """Go-Back-N: forget what was in flight and resend from `base`."""
        self.next = self.base

    def bytes_acknowledged(self) -> int:
        if self.base >= self.data_frames:
            return len(self.data)
        return self.base * self.frame_length


# ---------------------------------------------------------------------------
# Pure: flysight.txt, batches, digests, gates
# ---------------------------------------------------------------------------

def parse_flysight_txt(raw: bytes) -> dict:
    """Parse the status file the firmware rewrites at every boot.

    Layout: `f_printf("Key:  value\\n")` with fixed padding
    (FlySight/state.c:263-315). Keys come back lower-cased. Deliberately loose
    about whitespace and tolerant of `;` comments and of a user-edited file —
    one stray byte in Device_Name must not cost us Pubkey_X, so it decodes as
    latin-1 rather than UTF-8.
    """
    text = raw.decode("latin-1", "replace")
    fields = {}
    for line in text.splitlines():
        comment = line.find(";")
        if comment >= 0:
            line = line[:comment]
        colon = line.find(":")
        if colon <= 0:
            continue
        key = line[:colon].strip().lower()
        value = line[colon + 1:].strip()
        if key and value and key not in fields:
            fields[key] = value
    return fields


def batch_for_pubkey_x(pubkey_x):
    """Production batch for a device's Pubkey_X, or None if unrecognised.

    A None here is a hard stop before a flash: guessing means pushing an image
    the bootloader cannot decrypt.
    """
    if not pubkey_x or len(pubkey_x) < 8:
        return None
    return BATCHES.get(pubkey_x[:8].lower())


_BATCH_TOKEN = re.compile(r"(?:^|[^0-9A-Za-z])([Bb][2-9])(?:[^0-9A-Za-z]|$)")


def batch_from_filename(name: str):
    """The batch a .sfb file names, or None when it names none or several.

    Release and locally-built images are called `B2_UserApp.sfb`,
    `B2_v2024.12.30.10.sfb` and so on. A file that does not say which batch it
    is for cannot be checked against the device, and that is a refusal — the
    bootloader would simply fail to decrypt it.
    """
    found = {m.group(1).upper() for m in _BATCH_TOKEN.finditer(os.path.basename(name))}
    return found.pop() if len(found) == 1 else None


def parse_sha256sums(text: str) -> dict:
    """`digest␠␠name` lines, as `shasum -a 256` writes them."""
    out = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 2 and re.fullmatch(r"[0-9a-fA-F]{64}", parts[0]):
            out[os.path.basename(parts[-1].lstrip("*"))] = parts[0].lower()
    return out


def expected_digest_for(path: str, explicit=None):
    """(digest, source) for an image: the flag, then a sibling sha256sums.txt.

    Returns (None, None) when neither exists — see `flash_gates`, which reports
    that as a warning rather than a refusal.
    """
    if explicit:
        return explicit.strip().lower(), "--sha256"
    sums = os.path.join(os.path.dirname(os.path.abspath(path)), "sha256sums.txt")
    if os.path.exists(sums):
        with open(sums, "r", encoding="utf-8", errors="replace") as f:
            table = parse_sha256sums(f.read())
        name = os.path.basename(path)
        if name in table:
            return table[name], "sha256sums.txt"
        return "", "sha256sums.txt"  # present but silent about this file
    return None, None


@dataclass(frozen=True)
class Gate:
    """One pre-flash check. `ok` false stops everything; `warn` only prints."""
    name: str
    ok: bool
    detail: str
    warn: bool = False


def flash_gates(*, image: bytes, filename: str, expected_sha256, digest_source,
                mode, battery, mtu, pubkey_x,
                min_battery: int = MIN_BATTERY_PCT, min_mtu: int = MIN_MTU):
    """Every check that must pass before a byte of firmware is written.

    Pure, so the whole decision table is exercised in Tests/test_flysight_ble.py
    without a device. Returns the checks in the order a person should read them;
    the caller refuses if any one of them is not `ok`.
    """
    gates = []

    # 1. Container magic. An .sfb is an SBSFU image; the first four bytes are
    #    ASCII SFU1 (Docs/BUILDING.md §3).
    magic = image[:4]
    gates.append(Gate(
        "SFU1 magic", magic == b"SFU1",
        f"first four bytes are {magic!r}, expected b'SFU1' — this is not an "
        ".sfb image" if magic != b"SFU1" else "first four bytes are SFU1"))

    # 2. Digest. Fails closed when a reference exists and disagrees; when no
    #    reference exists at all it warns, because a locally built image has
    #    nothing to be compared against and the dry-run default is the real
    #    guard there.
    actual = hashlib.sha256(image).hexdigest()
    if expected_sha256:
        gates.append(Gate(
            "sha256", actual == expected_sha256,
            f"{actual} matches {digest_source}" if actual == expected_sha256
            else f"{actual} does not match {expected_sha256} from {digest_source}"))
    elif digest_source:
        gates.append(Gate(
            "sha256", False,
            f"{digest_source} exists but does not list {os.path.basename(filename)} "
            f"— refusing rather than assuming ({actual})"))
    else:
        gates.append(Gate(
            "sha256", True,
            f"{actual} (no --sha256 and no sha256sums.txt beside the file: "
            "NOT checked against a reference)", warn=True))

    # 3. Batch. The image is encrypted to one production batch's public key;
    #    the wrong one cannot be decrypted by the bootloader.
    file_batch = batch_from_filename(filename)
    device_batch = batch_for_pubkey_x(pubkey_x)
    prefix = (pubkey_x or "")[:8] or "unknown"
    if file_batch is None:
        gates.append(Gate("batch", False,
                          f"{os.path.basename(filename)} does not name exactly one "
                          "batch (expected something like B2_UserApp.sfb)"))
    elif device_batch is None:
        gates.append(Gate("batch", False,
                          f"device Pubkey_X starts {prefix} — not a batch this tool "
                          f"knows ({', '.join(sorted(BATCHES.values()))})"))
    else:
        gates.append(Gate("batch", file_batch == device_batch,
                          f"file says {file_batch}, device says {device_batch} "
                          f"(Pubkey_X {prefix})"))

    # 4. Mode. Every CRS command claims the SD card
    #    (FS_ResourceManager_RequestResource, crs.c:245), and ACTIVE/START/
    #    CONFIG hold it for as long as they run — so SLEEP is the only mode in
    #    which a file can be written at all.
    gates.append(Gate(
        "mode", mode == MODE_SLEEP,
        f"{MODE_NAMES.get(mode, 'unknown')} "
        f"({'file transfer allowed' if mode == MODE_SLEEP else 'file transfer refused; put it to sleep'})"))

    # 5. Battery. 0 is not flat, it is unmeasured: the level is only sampled in
    #    ACTIVE (FlySight/active_control.c:246-256 returns early in every other
    #    mode) and starts at 0 on every boot (custom_app.c:109,417). Both are a
    #    refusal, but for different reasons and with different advice.
    if battery is None:
        gates.append(Gate("battery", False, "could not be read"))
    elif battery == 0:
        gates.append(Gate("battery", False,
                          "reads 0 %, which means never measured, not flat — the "
                          "FlySight only samples it in ACTIVE. Wake it to ACTIVE "
                          "for a few seconds, put it back to SLEEP, and retry"))
    else:
        gates.append(Gate("battery", battery >= min_battery,
                          f"{battery} % (need {min_battery} %)"))

    # 6. MTU. A data frame is 244 bytes on the wire and a write-without-response
    #    carries ATT_MTU - 3, so anything under 247 would silently truncate.
    gates.append(Gate("ATT MTU", (mtu or 0) >= min_mtu,
                      f"{mtu} (need {min_mtu} for 244-byte frames)"))

    return gates


# ---------------------------------------------------------------------------
# The link
# ---------------------------------------------------------------------------

class CrsError(Exception):
    """Anything the file-transfer layer refuses or gives up on."""


class _Inbox:
    """A mailbox for FT_Packet_Out, awaited packet by packet with a timeout."""

    def __init__(self):
        self._queue = collections.deque()
        self._waiter = None
        self._error = None

    def add(self, packet: bytes) -> None:
        self._queue.append(packet)
        self._wake()

    def push_front(self, packet: bytes) -> None:
        self._queue.appendleft(packet)

    def fail(self, error: Exception) -> None:
        if self._error is None:
            self._error = error
        self._wake()

    def _wake(self) -> None:
        waiter, self._waiter = self._waiter, None
        if waiter is not None and not waiter.done():
            waiter.set_result(None)

    async def next(self, timeout: float):
        """The next packet, or None if `timeout` passed with none."""
        while True:
            if self._error is not None:
                raise self._error
            if self._queue:
                return self._queue.popleft()
            waiter = asyncio.get_running_loop().create_future()
            self._waiter = waiter
            try:
                await asyncio.wait_for(waiter, timeout)
            except asyncio.TimeoutError:
                if self._waiter is waiter:
                    self._waiter = None
                return None


class FlySight:
    """One connected FlySight: control points, and file transfer over CRS."""

    def __init__(self, client, window: int = WINDOW_LENGTH):
        self.client = client
        self.window = window
        self.mode = None
        self.battery = None
        self._inbox = None
        self._cp_waiters = {}
        self._last_send = 0.0
        self._keepalive = None

    # --- setup ------------------------------------------------------------

    async def start(self) -> None:
        """Subscribe to everything, then read the state that is free to read.

        Indications first: the firmware only answers a control-point write at
        all if the central has subscribed (`if (notification_enabled_flag)`,
        FlySight/device_state.c:230), so a request sent before this would hang
        forever.
        """
        await self.client.start_notify(FT_PACKET_OUT, self._on_ft)
        await self.client.start_notify(DS_CONTROL_POINT, self._on_ds_cp)
        await self.client.start_notify(DS_MODE, self._on_ds_mode)
        try:
            self.mode = (await self.client.read_gatt_char(DS_MODE))[0]
        except Exception as e:
            log(f"could not read DS_Mode: {e}")
        try:
            self.battery = (await self.client.read_gatt_char(BATTERY_LEVEL))[0]
        except Exception as e:
            log(f"could not read Battery_Level: {e}")

    def _on_ft(self, _char, data: bytearray) -> None:
        if self._inbox is not None:
            self._inbox.add(bytes(data))

    def _on_ds_mode(self, _char, data: bytearray) -> None:
        if data:
            self.mode = data[0]

    def _on_ds_cp(self, _char, data: bytearray) -> None:
        decoded = decode_cp_response(bytes(data))
        if decoded is None:
            log(f"dropping malformed control-point indication {bytes(data).hex()}")
            return
        opcode, status, payload = decoded
        queue = self._cp_waiters.get(opcode)
        if not queue:
            log(f"unsolicited control-point response for opcode 0x{opcode:02x}")
            return
        future = queue.pop(0)
        if not future.done():
            future.set_result((status, payload))

    # --- control point ----------------------------------------------------

    async def cp_request(self, opcode: int, params: bytes = b"", timeout: float = 5.0):
        """Write `[opcode][params…]` and wait for the echoed response.

        Returns (status, data). A refusal is a status, not an exception — the
        caller decides what a `not permitted` means.
        """
        future = asyncio.get_running_loop().create_future()
        self._cp_waiters.setdefault(opcode, []).append(future)
        try:
            # Both control points are CHAR_PROP_WRITE, not write-without-
            # response (custom_stm.c:896,1074).
            await self.client.write_gatt_char(
                DS_CONTROL_POINT, bytes([opcode]) + params, response=True)
            return await asyncio.wait_for(future, timeout)
        except asyncio.TimeoutError:
            raise CrsError(
                f"no answer to control-point opcode 0x{opcode:02x} within "
                f"{timeout:.0f} s") from None
        finally:
            queue = self._cp_waiters.get(opcode)
            if queue and future in queue:
                queue.remove(future)

    async def firmware_version(self):
        """GIT_TAG, truncated to 16 characters by the response buffer.

        `strncpy(buf, GIT_TAG, MAX_CP_OPTIONAL_RESPONSE_DATA_LEN - 1)`
        (device_state.c:147, control_point_protocol.h:41) — so equality against
        a full tag is a hint, never proof.
        """
        status, data = await self.cp_request(DS_GET_FW_VERSION)
        return cp_text(data) if status == CP_SUCCESS else None

    async def device_id(self):
        status, data = await self.cp_request(DS_GET_DEVICE_ID)
        return cp_text(data) if status == CP_SUCCESS else None

    async def install_uploaded_firmware(self) -> int:
        """Send 0x04. The device answers, *then* reboots into the bootloader.

        The response is sent from the notification-complete callback before the
        reset (device_state.c:240-260), so a success means "accepted and about
        to reboot" and the disconnect that follows is expected.
        """
        status, _ = await self.cp_request(DS_INSTALL_UPLOADED_FIRMWARE)
        return status

    # --- file transfer ----------------------------------------------------

    async def _send(self, packet: bytes) -> None:
        self._last_send = time.monotonic()
        await self.client.write_gatt_char(FT_PACKET_IN, packet, response=False)

    async def _keepalive_loop(self) -> None:
        """Ping only when the link has actually been silent.

        Every ACK and data frame is a write to FT_Packet_In and refreshes the
        device's 30 s timer by itself (custom_app.c:824-825), so an
        unconditional ping would just eat one of the eight ring slots.
        """
        try:
            while True:
                await asyncio.sleep(KEEPALIVE_INTERVAL)
                if time.monotonic() - self._last_send < KEEPALIVE_INTERVAL:
                    continue
                try:
                    await self._send(build_ping())
                except Exception as e:
                    log(f"keepalive ping failed: {e}")
        except asyncio.CancelledError:
            pass

    def _begin(self) -> _Inbox:
        if self._inbox is not None:
            raise CrsError("another file operation is already running")
        self._inbox = _Inbox()
        self._keepalive = asyncio.ensure_future(self._keepalive_loop())
        return self._inbox

    async def _end(self, finished: bool) -> None:
        if not finished:
            # Only a cancel, completion or a dropped link ends a transfer on
            # the device side (crs.c:355-358,421-424,524-527); until then it
            # keeps the SD card claimed. A stray cancel is answered with
            # NAK 0xFF and ignored by the next operation.
            try:
                await self._send(build_cancel())
            except Exception as e:
                log(f"could not abort the transfer: {e}")
        if self._keepalive is not None:
            self._keepalive.cancel()
            self._keepalive = None
        self._inbox = None

    async def _await_ack(self, inbox: _Inbox, opcode: int, what: str) -> None:
        """Wait for the ACK/NAK that opens an operation, stashing anything else."""
        stashed = []
        try:
            while True:
                raw = await inbox.next(COMMAND_TIMEOUT)
                if raw is None:
                    raise CrsError(
                        f"the FlySight did not answer the request to {what} — it "
                        "may be busy with its SD card")
                op = opcode_of(raw)
                if op == CRS_ACK:
                    if parse_ack_nak(raw) == opcode:
                        return
                    continue  # a keepalive's ACK, or a leftover
                if op == CRS_NAK:
                    refused = parse_ack_nak(raw)
                    if refused in (CRS_PING, CRS_CANCEL):
                        continue  # leftover, not a refusal of this command
                    raise CrsError(
                        f"the FlySight refused to {what} (NAK 0x{refused:02x}) — "
                        "check the path, and that it is in SLEEP with its card in")
                stashed.append(raw)
        finally:
            for raw in reversed(stashed):
                inbox.push_front(raw)

    async def read_file(self, path: str, on_progress=None) -> bytes:
        """Download `path`, mirroring the app's `CrsClient.readFile`."""
        inbox = self._begin()
        finished = False
        try:
            await self._send(build_read_file(path))
            await self._await_ack(inbox, CRS_READ, f"read {path}")

            chunks = []
            received = 0
            expected = 0  # unbounded; only the low byte goes on the wire
            retries = 0
            started = time.monotonic()

            while True:
                raw = await inbox.next(PACKET_TIMEOUT)
                if raw is None:
                    if not self.client.is_connected:
                        raise CrsError(f"the FlySight disconnected reading {path}")
                    retries += 1
                    if retries > MAX_RETRIES:
                        raise CrsError(f"the FlySight stopped responding reading {path}")
                    # The device rewinds to its last unacknowledged packet on
                    # its own 200 ms timer (crs.c:426-436); repeating the last
                    # ACK is a harmless nudge and refreshes the 30 s link timer.
                    if expected > 0:
                        await self._send(build_file_ack((expected - 1) & 0xFF))
                    continue

                op = opcode_of(raw)
                if op == CRS_NAK:
                    refused = parse_ack_nak(raw)
                    if refused in (CRS_PING, CRS_CANCEL):
                        continue
                    raise CrsError(f"the FlySight NAK'd 0x{refused:02x} reading {path}")
                if op != CRS_FILE_DATA:
                    continue

                counter, payload = parse_file_data(raw)
                behind = (expected - counter) & 0xFF
                if behind == 0:
                    retries = 0
                    # Acknowledge before deciding this is the end: the device
                    # only closes the file once the empty packet is acknowledged
                    # (next_ack == last_packet, crs.c:497-500).
                    await self._send(build_file_ack(counter))
                    expected += 1
                    if not payload:
                        finished = True
                        break
                    chunks.append(payload)
                    received += len(payload)
                    if on_progress:
                        on_progress(received, None, time.monotonic() - started)
                elif behind <= self.window:
                    # Already had it — our ACK was lost and the device rewound.
                    await self._send(build_file_ack(counter))
                else:
                    # Ahead of us. Deliberately not acknowledged, so the
                    # device's timer rewinds it to the frame we are missing.
                    log(f"out-of-order frame {counter}, expecting {expected & 0xFF}")

            return b"".join(chunks)
        finally:
            await self._end(finished)

    async def write_file(self, path: str, data: bytes, on_progress=None) -> None:
        """Upload `data` to `path`, truncating whatever was there.

        The window logic lives in `UploadWindow` — including the rule that the
        end-of-file frame waits for every data frame to be acknowledged.
        """
        inbox = self._begin()
        finished = False
        try:
            await self._send(build_write_file(path))
            await self._await_ack(inbox, CRS_WRITE, f"write {path}")

            window = UploadWindow(data, window=self.window)
            retries = 0
            started = time.monotonic()

            while not window.done:
                for frame in window.pending():
                    await self._send(frame)

                raw = await inbox.next(ACK_TIMEOUT)
                if raw is None:
                    if not self.client.is_connected:
                        raise CrsError(f"the FlySight disconnected writing {path}")
                    retries += 1
                    if retries > MAX_RETRIES:
                        raise CrsError(f"the FlySight stopped acknowledging {path}")
                    window.on_timeout()
                    continue

                op = opcode_of(raw)
                if op == CRS_NAK:
                    refused = parse_ack_nak(raw)
                    if refused in (CRS_PING, CRS_CANCEL):
                        continue
                    raise CrsError(
                        f"the FlySight NAK'd 0x{refused:02x} writing {path} — if "
                        "this is 0x10 the file was closed early and what is on "
                        "the card is truncated")
                if op != CRS_FILE_ACK or len(raw) < 2:
                    continue

                if window.on_ack(raw[1]):
                    retries = 0
                    if on_progress:
                        on_progress(window.bytes_acknowledged(), len(data),
                                    time.monotonic() - started)

            finished = True
            if on_progress:
                on_progress(len(data), len(data), time.monotonic() - started)
        finally:
            await self._end(finished)

    async def make_directory(self, path: str) -> None:
        """Create `path`; an existing directory answers NAK, which is normal."""
        inbox = self._begin()
        try:
            await self._send(build_mkdir(path))
            await self._await_ack(inbox, CRS_MK_DIR, f"create {path}")
        finally:
            await self._end(True)


# ---------------------------------------------------------------------------
# Connecting
# ---------------------------------------------------------------------------

VERBOSE = False

# Where `log` sends its lines besides stderr. Tools/flysight_app.py points this
# at its own ring buffer so protocol oddities reach the page instead of a
# terminal nobody is watching; a plain CLI run leaves it None.
LOG_SINK = None


def log(message: str) -> None:
    if LOG_SINK is not None:
        try:
            LOG_SINK(message)
        except Exception:  # a broken sink must never break a transfer
            pass
    if VERBOSE:
        print(f"  [ble] {message}", file=sys.stderr)


def select_device(found, address=None, name: str = ADV_NAME):
    """Pick the strongest match out of one scan, or None if there is none.

    `found` is what `BleakScanner.discover(return_adv=True)` hands back:
    `{address: (BLEDevice, AdvertisementData)}`. An explicit address wins
    whatever the device calls itself — a FlySight renamed in its config still
    answers to its CoreBluetooth UUID — and without one only the exact
    advertised name counts. Pure, so the choice is testable without a radio.
    """
    best = None
    for device, adv in found.values():
        advertised = adv.local_name or device.name or ""
        if address:
            if device.address.lower() != address.lower():
                continue
        elif advertised != name:
            continue
        rssi = -127 if adv.rssi is None else adv.rssi
        if best is None or rssi > best[2]:
            best = (device, advertised, rssi)
    return best


async def find_device(address=None, timeout: float = 10.0):
    """Find the FlySight by name, or by the CoreBluetooth UUID given.

    The advertisement carries only the complete local name and the
    manufacturer id 0xDB09 — no service UUID — so filtering on services finds
    nothing (Docs/ble.md, §Advertising Details).
    """
    if BleakScanner is None:
        raise SystemExit(
            "bleak is not installed. Run this with ~/.venvs/ble/bin/python.")
    print(f"Scanning {timeout:.0f} s for a FlySight…", flush=True)
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    best = select_device(found, address)
    if best is None:
        raise SystemExit(
            f"No FlySight found ({len(found)} advertisers seen). It stops "
            "advertising while another phone or Mac is connected, and it is "
            "silent while switched off.")
    device, name, rssi = best
    print(f"Found {name!r} at {device.address} ({rssi} dBm)")
    return device


async def connect_device(device, *, window: int = WINDOW_LENGTH,
                         timeout: float = 30.0, disconnected_callback=None):
    """Connect to `device`, subscribe, and hand back `(client, FlySight)`.

    **A connect that raises must still be cancelled.** macOS keeps the attempt
    pending long after our timeout has given up, the FlySight ends up counting
    itself linked, and it then stops advertising to EVERYONE — the phone
    included, which reads as "the device has died". The guard catches
    `BaseException` on purpose: a caller that wraps this in `asyncio.wait_for`
    cancels it, and a cancellation leaves exactly the same pending attempt
    behind as a failure does.
    """
    client = BleakClient(device, timeout=timeout,
                         disconnected_callback=disconnected_callback)
    try:
        await client.connect()
        fs = FlySight(client, window=window)
        await fs.start()
    except BaseException:
        try:
            await client.disconnect()
        except Exception:
            pass
        raise
    return client, fs


class connected:
    """`async with connected(args) as fs:` — connect, subscribe, tear down."""

    def __init__(self, args):
        self.args = args
        self.client = None

    async def __aenter__(self):
        device = await find_device(self.args.address, self.args.scan_timeout)
        # `__aexit__` never runs when `__aenter__` raises, so the cancellation
        # of a failed attempt lives inside connect_device.
        self.client, fs = await connect_device(device, window=self.args.window)
        return fs

    async def __aexit__(self, *exc):
        try:
            await self.client.disconnect()
        except Exception:
            pass
        return False


def progress_printer(label: str):
    """A one-line progress callback, rewritten in place."""
    state = {"last": 0.0}

    def report(done, total, elapsed):
        now = time.monotonic()
        final = total is not None and done >= total
        if now - state["last"] < 0.25 and not final:
            return
        state["last"] = now
        rate = done / elapsed / 1024 if elapsed > 0 else 0
        if total:
            print(f"\r  {label}: {done}/{total} B ({100 * done // total} %) "
                  f"{rate:.1f} kB/s   ", end="", flush=True)
        else:
            print(f"\r  {label}: {done} B {rate:.1f} kB/s   ", end="", flush=True)
        if final:
            print()

    return report


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------

async def cmd_info(args) -> int:
    """Read-only. Writes nothing to the device, and changes no mode."""
    async with connected(args) as fs:
        mtu = fs.client.mtu_size
        version = await fs.firmware_version()
        device_id = await fs.device_id()

        print()
        print(f"  ATT MTU:       {mtu}"
              f"{'' if mtu >= MIN_MTU else f'  (below {MIN_MTU} — too small to flash)'}")
        print(f"  Mode:          {MODE_NAMES.get(fs.mode, f'unknown ({fs.mode})')}")
        battery = "unreadable" if fs.battery is None else f"{fs.battery} %"
        if fs.battery == 0:
            battery += "  (never measured — it only samples in ACTIVE)"
        print(f"  Battery:       {battery}")
        print(f"  Firmware:      {version or 'unreadable'}   "
              f"(BLE reports at most 16 characters of the tag)")
        print(f"  Device ID:     {device_id or 'unreadable'}")

        if fs.mode != MODE_SLEEP:
            print()
            print("  /flysight.txt was NOT read: file transfer only works in "
                  "SLEEP, and this tool does not change the mode by itself. "
                  "Put the FlySight to sleep and run --info again for the batch "
                  "and the full firmware version.")
            return 0

        raw = await fs.read_file(FLYSIGHT_TXT)
        fields = parse_flysight_txt(raw)
        pubkey_x = fields.get("pubkey_x", "")
        batch = batch_for_pubkey_x(pubkey_x)
        print()
        print(f"  Firmware_Ver:  {fields.get('firmware_ver', '?')}   "
              f"(complete tag, from {FLYSIGHT_TXT})")
        print(f"  Stack_Ver:     {fields.get('stack_ver', '?')}")
        print(f"  FUS_Ver:       {fields.get('fus_ver', '?')}")
        print(f"  Device_Name:   {fields.get('device_name', '?')}")
        print(f"  Pubkey_X:      {pubkey_x[:8] or '?'}…  → batch "
              f"{batch or 'UNKNOWN — do not flash'}")
        if version and fields.get("firmware_ver"):
            full = fields["firmware_ver"]
            if not full.startswith(version):
                print(f"  ! BLE says {version!r} but the file says {full!r} — "
                      "one of them is stale")
        return 0


async def cmd_upload(args) -> int:
    """Write a file to a harmless path and prove it arrived intact."""
    with open(args.upload, "rb") as f:
        data = f.read()
    digest = hashlib.sha256(data).hexdigest()
    print(f"{args.upload}: {len(data)} bytes, sha256 {digest}")

    async with connected(args) as fs:
        if fs.mode != MODE_SLEEP:
            print(f"REFUSED: the FlySight is in "
                  f"{MODE_NAMES.get(fs.mode, 'an unknown mode')}; file transfer "
                  "needs SLEEP. Nothing was written.")
            return 2
        if fs.client.mtu_size < MIN_MTU:
            print(f"REFUSED: ATT MTU {fs.client.mtu_size} is below {MIN_MTU}; "
                  "a 244-byte frame would not fit. Nothing was written.")
            return 2

        print(f"Uploading to {args.dest}…")
        await fs.write_file(args.dest, data, progress_printer("up"))

        print("Reading it back…")
        echo = await fs.read_file(args.dest, progress_printer("down"))
        print()

    echoed = hashlib.sha256(echo).hexdigest()
    if len(echo) != len(data):
        print(f"MISMATCH: wrote {len(data)} bytes, read back {len(echo)}. "
              "A short read-back is what a truncated write looks like.")
        return 1
    if echoed != digest:
        print(f"MISMATCH: read back sha256 {echoed}, expected {digest}.")
        return 1
    print(f"Round trip OK: {len(echo)} bytes, sha256 {echoed} — the "
          "Go-Back-N uploader is sound on this link.")
    return 0


async def cmd_flash(args) -> int:
    """The real thing. Every gate fails closed; the install needs an opt-in."""
    with open(args.flash, "rb") as f:
        image = f.read()
    expected, source = expected_digest_for(args.flash, args.sha256)

    async with connected(args) as fs:
        mtu = fs.client.mtu_size
        version = await fs.firmware_version()
        pubkey_x = ""
        if fs.mode == MODE_SLEEP:
            fields = parse_flysight_txt(await fs.read_file(FLYSIGHT_TXT))
            pubkey_x = fields.get("pubkey_x", "")
            print(f"Device: {fields.get('firmware_ver', '?')} "
                  f"(BLE reports {version or '?'}), "
                  f"Pubkey_X {pubkey_x[:8] or '?'}…")

        gates = flash_gates(
            image=image, filename=args.flash, expected_sha256=expected,
            digest_source=source, mode=fs.mode, battery=fs.battery, mtu=mtu,
            pubkey_x=pubkey_x)

        print()
        for gate in gates:
            mark = "ok  " if gate.ok and not gate.warn else ("WARN" if gate.ok else "STOP")
            print(f"  [{mark}] {gate.name}: {gate.detail}")
        print()

        if any(not gate.ok for gate in gates):
            print("REFUSED. Nothing was written; the FlySight still boots what "
                  "it always did.")
            return 2

        # From here the device is written to — and everything up to the install
        # opcode is still reversible: the image is only a file on the card.
        try:
            await fs.make_directory(FIRMWARE_DIR)
            print(f"Created {FIRMWARE_DIR}.")
        except CrsError:
            print(f"{FIRMWARE_DIR} already exists.")

        print(f"Uploading {len(image)} bytes to {FIRMWARE_PATH}…")
        await fs.write_file(FIRMWARE_PATH, image, progress_printer("up"))

        # Read it back before offering to install it. A short or corrupt image
        # that gets installed leaves a device in its bootloader, recoverable
        # only over USB — so this is the last cheap moment to catch it.
        print("Verifying what landed on the card…")
        echo = await fs.read_file(FIRMWARE_PATH, progress_printer("down"))
        print()
        if hashlib.sha256(echo).hexdigest() != hashlib.sha256(image).hexdigest():
            print(f"MISMATCH: {len(echo)} of {len(image)} bytes on the card and "
                  "the digests differ. NOT installing. Delete /FW/APP.SFB or "
                  "simply upload again — nothing was installed and the device "
                  "still boots its current firmware.")
            return 1
        print(f"On-device image verified: {len(echo)} bytes, "
              f"sha256 {hashlib.sha256(echo).hexdigest()}")

        if not args.really_install:
            print()
            print("DRY RUN: the image is on the card and verified, and "
                  "DS_CMD_INSTALL_UPLOADED_FIRMWARE (0x04) was NOT sent. The "
                  "FlySight still runs its current firmware. Re-run with "
                  "--really-install to install it.")
            return 0

        print()
        print("Sending the install command. The FlySight will reboot into its "
              "bootloader — do not switch it off.")
        try:
            status = await fs.install_uploaded_firmware()
        except CrsError as e:
            print(f"No answer to the install command: {e}")
            print("If the device rebooted anyway it is installing; give it a "
                  "minute, then re-run --info.")
            return 1

        if status == CP_NOT_SUPPORTED:
            print("REFUSED: this firmware has no install-over-BLE command "
                  "(it arrived in v0.0.12). The image is on the card and is "
                  "harmless there; install once over USB.")
            return 2
        if status != CP_SUCCESS:
            print(f"REFUSED: the FlySight answered "
                  f"'{CP_STATUS.get(status, hex(status))}'. Nothing was "
                  "installed; it still runs the firmware it had.")
            return 2
        print("Accepted. It is rebooting to install — the disconnect is "
              "expected. Give it half a minute, then re-run --info and check "
              "Firmware_Ver.")
        return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="flysight_ble.py",
        description="Inspect and flash a FlySight 2 over Bluetooth from a Mac.",
        epilog="Run in the bleak venv with the macOS sandbox OFF: "
               "~/.venvs/ble/bin/python Tools/flysight_ble.py --info")
    what = parser.add_mutually_exclusive_group(required=True)
    what.add_argument("--info", action="store_true",
                      help="read-only: version, batch, mode, battery, MTU")
    what.add_argument("--upload", metavar="FILE",
                      help="write FILE to --dest, read it back, compare sha256")
    what.add_argument("--flash", metavar="FILE.sfb",
                      help="upload firmware to /FW/APP.SFB (dry run by default)")

    parser.add_argument("--dest", metavar="PATH", default=None,
                        help="destination path for --upload, e.g. /TEST.BIN")
    parser.add_argument("--sha256", metavar="HEX", default=None,
                        help="expected digest of the --flash image; otherwise a "
                             "sha256sums.txt beside it is used if present")
    parser.add_argument("--really-install", action="store_true",
                        help="with --flash, actually send the install opcode "
                             "(0x04) after the upload verifies")
    parser.add_argument("--address", metavar="UUID", default=None,
                        help="connect to this CoreBluetooth UUID instead of "
                             "scanning by name")
    parser.add_argument("--scan-timeout", type=float, default=10.0,
                        metavar="SEC", help="how long to scan (default 10)")
    parser.add_argument("--window", type=int, default=WINDOW_LENGTH,
                        metavar="N",
                        help=f"Go-Back-N window, 1..{WINDOW_LENGTH} (default "
                             f"{WINDOW_LENGTH}); lower it if uploads stall")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="log protocol oddities to stderr")
    return parser


def main(argv=None) -> int:
    global VERBOSE
    args = build_parser().parse_args(argv)
    VERBOSE = args.verbose

    if args.upload and not args.dest:
        print("--upload needs --dest, e.g. --dest /TEST.BIN", file=sys.stderr)
        return 2
    if args.flash and not args.flash.lower().endswith(".sfb"):
        print("--flash expects an .sfb image (the encrypted, batch-specific "
              "container the bootloader installs).", file=sys.stderr)
        return 2
    if not 1 <= args.window <= WINDOW_LENGTH:
        print(f"--window must be 1..{WINDOW_LENGTH}", file=sys.stderr)
        return 2
    if BleakClient is None:
        print("bleak is not installed. Run this with ~/.venvs/ble/bin/python.",
              file=sys.stderr)
        return 1

    if args.info:
        command = cmd_info
    elif args.upload:
        command = cmd_upload
    else:
        command = cmd_flash

    try:
        return asyncio.run(command(args))
    except KeyboardInterrupt:
        print("\nInterrupted. Nothing is installed unless the install command "
              "was already accepted.")
        return 1
    except CrsError as e:
        print(f"Failed: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        # bleak raises its own family (BleakError, BleakDeviceNotFoundError…);
        # a person holding a FlySight wants a sentence, not a traceback.
        print(f"Failed: {type(e).__name__}: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
