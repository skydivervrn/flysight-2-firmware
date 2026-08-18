#!/usr/bin/env python3
"""A long-lived Mac app that holds a link to a FlySight 2 and flashes it.

`Tools/flysight_ble.py` already speaks the protocol correctly, but every
subcommand is a fresh process: it scans for ten seconds, does one thing, and
drops the link. The FlySight's PAIRING window is thirty seconds long and opens
when the owner double-presses the button — so a one-shot command spends most of
its life missing that window, and the owner spends his pressing the button at a
process that is not listening yet.

This is the same protocol with the process turned inside out. One asyncio loop
owns the connection for the lifetime of the app: it scans, connects, and when
the link drops it goes back to scanning by itself. The owner presses the button
whenever he likes and the app is already there. Once connected it *keeps* the
link, so reading a file and flashing an image cost nothing but the transfer.

Two front ends onto that one loop, and they are the same front end:

  * a page at http://127.0.0.1:8765/ that polls `/api/state` once a second;
  * a JSON API under `/api/` where every button has an endpoint, documented
    with a working curl line in Docs/FLASH_OVER_BLE.md.

The API is the point, not a debug hatch: it is how an agent drives this thing.
Long operations start and return immediately — the work is reported through
`/api/state`, so nothing has to hold a socket open for a two-minute upload.

**The HTTP server binds 127.0.0.1 only, and has no authentication.** That is
deliberate and it is the whole security model: nothing off this Mac can reach
it, and anything running on this Mac as this user could drive CoreBluetooth
directly anyway. Do not put it on 0.0.0.0.

Run it in the bleak venv with the macOS sandbox OFF — CoreBluetooth is not
reachable from a sandboxed process:

    Tools/flysight_app.sh          # venv + browser
    ~/.venvs/ble/bin/python Tools/flysight_app.py

Nothing here re-implements the protocol. Every frame, gate and timeout comes
from `flysight_ble`, imported as a module.
"""
from __future__ import annotations

import argparse
import asyncio
import concurrent.futures
import errno
import glob
import hashlib
import http.server
import json
import os
import socket
import sys
import threading
import time
import urllib.parse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import flysight_ble as fb  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMAGE_DIR = os.path.join(REPO, "Deploy", "Firmware_To_Deploy")

HOST = "127.0.0.1"  # loopback on purpose — see the module docstring
PORT = 8765

LOG_LINES = 50

# Timeouts. Every one of them exists so that no single failure can wedge the
# loop: a task that cannot finish is cancelled and the reason lands in the
# state, where the page and the API can both see it.
SCAN_TIMEOUT = 6.0        # one scan pass; the retry loop just runs another
CONNECT_TIMEOUT = 45.0    # connect + subscribe + first reads
INFO_TIMEOUT = 20.0       # version and device id after a link comes up
REFRESH_INTERVAL = 5.0    # re-read mode and battery while idle
FILE_JOB_TIMEOUT = 120.0  # a whole file read
FLASH_JOB_TIMEOUT = 900.0  # upload + read-back + install of a ~160 kB image
HTTP_CALL_TIMEOUT = 10.0  # bridging an HTTP thread into the loop
HTTP_FILE_TIMEOUT = 45.0  # …except a file read, which answers with the file
RETRY_PAUSE = 2.0         # after a failed connect, before scanning again
# How long one press of "Connect & pair" keeps looking before giving up. Long
# enough to walk to the FlySight, wake it and double-press it; short enough
# that a button pressed by mistake does not leave the radio scanning an empty
# room all day. A link that comes up resets it.
RETRY_GIVE_UP = 240.0

# The word a caller must send with an install request. A dry run and the real
# thing follow the same code path right up to the last line; this token is the
# only difference, and it is never remembered.
INSTALL_CONFIRM = "INSTALL"


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def newest_sfb(directory: str = IMAGE_DIR):
    """The most recently modified .sfb in `directory`, or None.

    This is what the page pre-fills its image field with. Newest-by-mtime is
    the right guess because the deploy step rewrites the same file name for
    every build, so a name-sorted pick would be stale as often as not. It is
    only a default: the field stays editable and the gates still decide.
    """
    newest = None
    for path in sorted(glob.glob(os.path.join(directory, "*.sfb"))):
        try:
            stamp = os.path.getmtime(path)
        except OSError:
            continue  # deleted between the glob and here; it is only a default
        if newest is None or stamp >= newest[0]:
            newest = (stamp, path)
    return newest[1] if newest else None


def gate_status(gate) -> str:
    """One of `ok` / `warn` / `stop`, in the order the caller must read it."""
    if not gate.ok:
        return "stop"
    return "warn" if gate.warn else "ok"


def render_gates(gates):
    """`flash_gates` output as JSON, keeping the order and the verdicts."""
    return [{"name": g.name, "status": gate_status(g), "ok": bool(g.ok),
             "warn": bool(g.warn), "detail": g.detail} for g in gates]


def gates_passed(gates) -> bool:
    """True only if every gate passes. A warning passes; a stop does not."""
    return all(g.ok for g in gates)


# Link states. `scanning` and `connecting` are only ever reached while the
# retry loop is running, so the page can say "still trying" without guessing.
IDLE = "idle"
SCANNING = "scanning"
CONNECTING = "connecting"
CONNECTED = "connected"


def link_hint(state: str, retrying: bool) -> str:
    """The one line the page shows under the link state.

    While disconnected it says the thing the owner actually needs to know: the
    app is already retrying, so the button press is his to make whenever he
    likes.
    """
    if state == CONNECTED:
        return ("Linked. The FlySight stops advertising while a central holds "
                "it, so the tablet will not see it until this app disconnects.")
    if not retrying:
        return ("Idle — no radio in use. Press “Connect & pair” when you are "
                "standing next to the FlySight; it will then keep trying until "
                "you double-press the button, and stop by itself if that never "
                "happens.")
    if state == CONNECTING:
        return "Found it — connecting."
    return ("Scanning. Double-press the FlySight's button to open PAIRING mode "
            "(the LED pulses green for 30 s) — the app is already retrying, so "
            "press it whenever you like. The first bond also needs the macOS "
            "pairing dialog.")


async def bounded(coro, timeout: float):
    """`asyncio.wait_for`, except the inner work is always waited out.

    Plain `wait_for` runs its argument in a task of its own, and when the
    *caller* is cancelled it only **requests** that task's cancellation — it
    does not wait for it, and the caller unwinds immediately. Mid-flash that
    leaves a second copy of the work running against a link nobody is holding
    any more: measured here, a cancelled flash went on to upload, verify and
    install its image after `disconnect()` had already returned. Everything
    below waits for the inner task to actually stop before it lets go.
    """
    task = asyncio.ensure_future(coro)
    try:
        return await asyncio.wait_for(asyncio.shield(task), timeout)
    except BaseException:
        task.cancel()
        try:
            await task
        except BaseException:
            pass
        raise


def progress_record(label: str, done: int, total, elapsed: float) -> dict:
    """A transfer's progress, with the percentage worked out once, here."""
    percent = None
    if total:
        percent = min(100, int(100 * done / total))
    rate = done / elapsed / 1024 if elapsed > 0 else 0.0
    return {"label": label, "done": done, "total": total,
            "percent": percent, "rate_kbs": round(rate, 1)}


# ---------------------------------------------------------------------------
# The shared state
# ---------------------------------------------------------------------------

class AppState:
    """Everything the page and the API can see, behind one lock.

    Written from the asyncio loop, read from the HTTP threads, so every
    accessor takes the lock and `snapshot()` hands back a deep-enough copy to
    serialise safely while the loop keeps going.
    """

    def __init__(self, image_dir: str = IMAGE_DIR):
        self._lock = threading.RLock()
        self.image_dir = image_dir
        self.link = IDLE
        self.retrying = False
        self.attempt = 0
        self.busy = None
        self.device = {}
        self.progress = None
        self.gates = []
        self.flash = {"stage": "idle", "image": None, "message": "",
                      "install": False, "sha256": None}
        self.file = None
        self.log = []
        self.started = time.time()

    # --- log --------------------------------------------------------------

    def note(self, message: str) -> None:
        """Append one line to the ring the page shows. Never raises."""
        stamped = f"{time.strftime('%H:%M:%S')}  {message}"
        with self._lock:
            self.log.append(stamped)
            del self.log[:-LOG_LINES]

    # --- link -------------------------------------------------------------

    def set_link(self, link: str, *, retrying=None, attempt=None) -> None:
        with self._lock:
            self.link = link
            if retrying is not None:
                self.retrying = retrying
            if attempt is not None:
                self.attempt = attempt
            if link != CONNECTED:
                self.device = {}

    def set_device(self, **fields) -> None:
        with self._lock:
            self.device.update({k: v for k, v in fields.items() if v is not None})

    def set_busy(self, busy) -> None:
        with self._lock:
            self.busy = busy
            if busy is None:
                self.progress = None

    def set_progress(self, progress) -> None:
        with self._lock:
            self.progress = progress

    def set_gates(self, gates) -> None:
        with self._lock:
            self.gates = list(gates)

    def set_flash(self, **fields) -> None:
        with self._lock:
            self.flash.update(fields)

    def set_file(self, record) -> None:
        with self._lock:
            self.file = record

    # --- read -------------------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "link": self.link,
                "retrying": self.retrying,
                "attempt": self.attempt,
                "hint": link_hint(self.link, self.retrying),
                "busy": self.busy,
                "device": dict(self.device),
                "progress": dict(self.progress) if self.progress else None,
                "gates": [dict(g) for g in self.gates],
                "flash": dict(self.flash),
                "file": dict(self.file) if self.file else None,
                "default_image": newest_sfb(self.image_dir),
                "image_dir": self.image_dir,
                "log": list(self.log),
                "uptime": round(time.time() - self.started, 1),
            }


# ---------------------------------------------------------------------------
# The API, as a pure function of (request, controller)
# ---------------------------------------------------------------------------

class ApiError(Exception):
    """A refusal with an HTTP status. Never an HTML page, never a traceback."""

    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status


def parse_json_body(raw: bytes) -> dict:
    """A request body as a dict. Empty is `{}`; anything else must be an object."""
    if not raw or not raw.strip():
        return {}
    try:
        body = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as e:
        raise ApiError(400, f"body is not JSON: {e}") from None
    if not isinstance(body, dict):
        raise ApiError(400, "body must be a JSON object")
    return body


def handle_api(method: str, path: str, query: dict, raw: bytes, ctrl):
    """Route one API request. Returns `(status, object)` — always an object.

    Pure with respect to the radio: `ctrl` is the only thing that touches a
    device, so Tests/test_flysight_app.py drives every endpoint with a fake
    controller and no bleak in sight.
    """
    try:
        if path == "/api/state":
            _require(method, "GET")
            return 200, {"ok": True, **ctrl.snapshot()}

        if path == "/api/connect":
            _require(method, "POST")
            return 200, {"ok": True, **ctrl.start_retry()}

        if path == "/api/stop":
            _require(method, "POST")
            return 200, {"ok": True, **ctrl.stop_retry()}

        if path == "/api/disconnect":
            _require(method, "POST")
            return 200, {"ok": True, **ctrl.disconnect()}

        if path == "/api/file":
            _require(method, "GET")
            target = query.get("path", fb.FLYSIGHT_TXT)
            if not target.startswith("/"):
                raise ApiError(400, "path must be absolute, e.g. /flysight.txt")
            return 200, {"ok": True, **ctrl.read_file(target)}

        if path == "/api/flash":
            _require(method, "POST")
            body = parse_json_body(raw)
            image = body.get("image") or newest_sfb(ctrl.image_dir)
            if not image:
                raise ApiError(400, "no image given and no .sfb found in "
                                    f"{ctrl.image_dir}")
            if not str(image).lower().endswith(".sfb"):
                raise ApiError(400, "expected an .sfb image (the encrypted, "
                                    "batch-specific container the bootloader "
                                    "installs), not a raw .bin")
            if not os.path.isfile(image):
                raise ApiError(400, f"no such file: {image}")
            install = bool(body.get("install"))
            # The confirmation is a token in the request and is never
            # remembered: the next call is a dry run again unless it says so.
            if install and body.get("confirm") != INSTALL_CONFIRM:
                raise ApiError(400,
                               "install needs \"confirm\": \"%s\" in the same "
                               "request. Without it this would have been a dry "
                               "run: uploaded, read back and verified, with "
                               "opcode 0x04 never sent." % INSTALL_CONFIRM)
            return 200, {"ok": True, **ctrl.start_flash(os.path.abspath(image),
                                                        install)}

        raise ApiError(404, f"no endpoint {path}. Try /api/state.")
    except ApiError as e:
        return e.status, {"ok": False, "error": str(e)}


def _require(method: str, expected: str) -> None:
    if method != expected:
        raise ApiError(405, f"use {expected} on this endpoint, not {method}")


# ---------------------------------------------------------------------------
# The link — everything below here runs in the asyncio loop
# ---------------------------------------------------------------------------

class Link:
    """The one object that owns the connection, for the life of the process."""

    def __init__(self, state: AppState, *, address=None,
                 window: int = fb.WINDOW_LENGTH, scan_timeout: float = SCAN_TIMEOUT):
        self.state = state
        self.address = address
        self.window = window
        self.scan_timeout = scan_timeout
        self.client = None
        self.fs = None
        self._retry_task = None
        self._job = None
        self._want_link = False
        # Created inside the loop, once there is one: an Event built on the
        # main thread binds itself to the wrong loop on older Pythons.
        self._dropped = None

    # --- retrying ---------------------------------------------------------

    async def start_retry(self) -> dict:
        self._want_link = True
        if self._retry_task is not None and not self._retry_task.done():
            self.state.set_link(self.state.link, retrying=True)
            return {"started": False, "note": "already looking"}
        self.state.set_link(self.state.link, retrying=True)
        self._retry_task = asyncio.ensure_future(self._retry_loop())
        self.state.note("looking for a FlySight — press the button whenever "
                        "you like, this keeps trying")
        return {"started": True}

    async def stop_retry(self) -> dict:
        """Stop looking. An existing link is left alone — that is Disconnect.

        A live link is not torn down here, and the loop holding it is not
        cancelled either: it is that task which keeps mode and battery current,
        and the flash gates read both. It stops of its own accord once the link
        drops, instead of scanning again.
        """
        self._want_link = False
        if self.client is not None:
            self.state.set_link(CONNECTED, retrying=False)
            self.state.note("stopped looking; the link stays up until it drops")
            return {"stopped": True, "connected": True}
        await self._cancel_retry()
        self.state.set_link(IDLE, retrying=False)
        self.state.note("stopped looking")
        return {"stopped": True, "connected": False}

    async def _cancel_retry(self) -> None:
        task, self._retry_task = self._retry_task, None
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):
                pass

    async def disconnect(self) -> dict:
        """Drop the link and stop looking. Any running job is cancelled first."""
        self._want_link = False
        await self._cancel_retry()
        await self._cancel_job()
        await self._drop_client("disconnected on request")
        self.state.set_link(IDLE, retrying=False)
        return {"disconnected": True}

    async def _drop_client(self, why: str) -> None:
        client, self.client, self.fs = self.client, None, None
        if client is None:
            return
        try:
            await bounded(client.disconnect(), 10.0)
        except Exception as e:
            self.state.note(f"disconnect complained: {type(e).__name__}: {e}")
        self.state.note(why)

    def _on_disconnected(self, _client) -> None:
        # bleak calls this on the loop, so setting the event straight is safe.
        if self._dropped is not None:
            self._dropped.set()

    async def _retry_loop(self) -> None:
        """Scan, connect, hold, and start again when the link goes away.

        Bounded on purpose. Looking is only useful while somebody is standing
        next to the FlySight ready to double-press it, so a search that has
        found nothing for RETRY_GIVE_UP seconds gives up and says so, rather
        than scanning an empty house until the laptop's battery notices. A
        link that comes up resets the budget: once connected, reconnecting
        after a drop is worth the same patience all over again.
        """
        attempt = 0
        deadline = time.monotonic() + RETRY_GIVE_UP
        while True:
            if time.monotonic() > deadline:
                self.state.note("gave up looking — press “Connect & pair” "
                                "again when you are next to the device")
                self.state.set_link(IDLE, retrying=False, attempt=attempt)
                self._retry_task = None
                return
            attempt += 1
            self.state.set_link(SCANNING, retrying=True, attempt=attempt)
            try:
                seen = await self._scan_once()
                if seen is None:
                    continue
                device, name, rssi = seen
                self.state.set_link(CONNECTING, retrying=True, attempt=attempt)
                self._dropped = asyncio.Event()
                # connect_device cancels a failed *or cancelled* attempt on the
                # way out — without that, macOS leaves the attempt pending, the
                # FlySight counts itself linked and stops advertising to
                # everyone, tablet included. Do not "simplify" the wait_for away.
                self.client, self.fs = await bounded(
                    fb.connect_device(device, window=self.window,
                                      disconnected_callback=self._on_disconnected),
                    CONNECT_TIMEOUT)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self.state.note(f"connect failed: {type(e).__name__}: {e}")
                await asyncio.sleep(RETRY_PAUSE)
                continue

            # Linked: the patience budget starts over, so a drop later gets a
            # full search of its own rather than the remains of this one.
            deadline = time.monotonic() + RETRY_GIVE_UP
            self.state.set_link(CONNECTED, retrying=True, attempt=attempt)
            self.state.set_device(name=name or fb.ADV_NAME, rssi=rssi,
                                  address=device.address,
                                  mtu=self.client.mtu_size)
            self.state.note(f"linked to {device.address} "
                            f"(ATT MTU {self.client.mtu_size})")
            await self._read_identity()

            refresher = asyncio.ensure_future(self._refresh_loop())
            try:
                await self._dropped.wait()
            finally:
                refresher.cancel()
            await self._cancel_job()
            self.client = self.fs = None
            if not self._want_link:  # Stop was pressed while it was linked
                self.state.set_link(IDLE, retrying=False)
                self.state.note("link dropped, and Stop was pressed — idle")
                return
            self.state.set_link(SCANNING, retrying=True)
            self.state.note("link dropped — back to scanning")

    async def _scan_once(self):
        if fb.BleakScanner is None:
            raise RuntimeError("bleak is not installed; run this in ~/.venvs/ble")
        found = await bounded(
            fb.BleakScanner.discover(timeout=self.scan_timeout, return_adv=True),
            self.scan_timeout + 15.0)
        best = fb.select_device(found, self.address)
        if best is None:
            self.state.note(f"no FlySight in that scan ({len(found)} advertisers "
                            "seen); it is silent while another central holds it")
            return None
        device, name, rssi = best
        self.state.note(f"saw {name!r} at {device.address} ({rssi} dBm)")
        return best

    async def _read_identity(self) -> None:
        """Version and device id — cheap, and free of the SD card."""
        try:
            version = await bounded(self.fs.firmware_version(), INFO_TIMEOUT)
            device_id = await bounded(self.fs.device_id(), INFO_TIMEOUT)
        except Exception as e:
            self.state.note(f"could not read the identity: {type(e).__name__}: {e}")
            return
        self.state.set_device(firmware=version, device_id=device_id,
                              mode=fb.MODE_NAMES.get(self.fs.mode, "unknown"),
                              mode_code=self.fs.mode, battery=self.fs.battery)

    async def _refresh_loop(self) -> None:
        """Keep mode and battery honest while nothing else is going on.

        The mode arrives by indication on its own, but the battery does not,
        and the flash gates turn on both. It reads nothing while a job holds
        the link — a GATT read in the middle of a transfer buys nothing and
        risks the timing.
        """
        try:
            while True:
                await asyncio.sleep(REFRESH_INTERVAL)
                if self._job is not None and not self._job.done():
                    continue
                if self.client is None or not self.client.is_connected:
                    continue
                try:
                    raw = await bounded(
                        self.client.read_gatt_char(fb.BATTERY_LEVEL), 10.0)
                    if raw:
                        self.fs.battery = raw[0]
                except Exception as e:
                    self.state.note(f"battery read failed: {type(e).__name__}: {e}")
                self.state.set_device(mode=fb.MODE_NAMES.get(self.fs.mode, "unknown"),
                                      mode_code=self.fs.mode,
                                      battery=self.fs.battery,
                                      mtu=self.client.mtu_size)
        except asyncio.CancelledError:
            pass

    # --- jobs -------------------------------------------------------------

    def _claim(self, name: str) -> None:
        if self.fs is None or self.client is None:
            raise ApiError(409, "not connected — press Connect and the button "
                                "on the FlySight")
        if self._job is not None and not self._job.done():
            raise ApiError(409, f"busy: {self.state.busy} is still running")
        self.state.set_busy(name)

    async def _cancel_job(self) -> None:
        job, self._job = self._job, None
        if job is not None and not job.done():
            job.cancel()
            try:
                await job
            except (asyncio.CancelledError, Exception):
                pass
        self.state.set_busy(None)

    def _progress(self, label: str):
        def report(done, total, elapsed):
            self.state.set_progress(progress_record(label, done, total, elapsed))
        return report

    async def read_file(self, path: str) -> dict:
        """Start a read and wait for it, but only up to a bound.

        A small file answers inside the HTTP request, which is what makes
        `curl /api/file` worth typing. If it takes longer than that the request
        gets an honest "still reading" and the result lands in `/api/state`
        when it arrives — the socket is never held hostage.
        """
        self._claim("read")
        self._job = asyncio.ensure_future(self._read_job(path))
        # If the wait below gives up, the job carries on and its failure would
        # otherwise be reported by asyncio as "exception was never retrieved".
        # It is already in the log and in /api/state; collect it and be quiet.
        self._job.add_done_callback(
            lambda t: None if t.cancelled() else t.exception())
        try:
            return await asyncio.wait_for(asyncio.shield(self._job),
                                          HTTP_FILE_TIMEOUT)
        except asyncio.TimeoutError:
            raise ApiError(504, f"still reading {path} — watch /api/state") from None

    async def _read_job(self, path: str) -> dict:
        try:
            self.state.note(f"reading {path}")
            raw = await bounded(
                self.fs.read_file(path, self._progress("read")), FILE_JOB_TIMEOUT)
            record = {"path": path, "size": len(raw),
                      "sha256": hashlib.sha256(raw).hexdigest(),
                      "text": raw.decode("utf-8", "replace"),
                      "at": time.strftime("%H:%M:%S")}
            self.state.set_file(record)
            self.state.note(f"read {path}: {len(raw)} bytes")
            if path == fb.FLYSIGHT_TXT:
                self._absorb_flysight_txt(raw)
            return record
        except asyncio.CancelledError:
            self.state.note(f"read of {path} cancelled")
            raise
        except Exception as e:
            self.state.note(f"read of {path} failed: {type(e).__name__}: {e}")
            raise ApiError(502, f"{type(e).__name__}: {e}") from None
        finally:
            self.state.set_busy(None)

    def _absorb_flysight_txt(self, raw: bytes) -> str:
        """Pull the batch and the full version out of a freshly read card."""
        fields = fb.parse_flysight_txt(raw)
        pubkey_x = fields.get("pubkey_x", "")
        self.state.set_device(firmware_full=fields.get("firmware_ver"),
                              stack_ver=fields.get("stack_ver"),
                              fus_ver=fields.get("fus_ver"),
                              device_name=fields.get("device_name"),
                              pubkey_x=pubkey_x[:8],
                              batch=fb.batch_for_pubkey_x(pubkey_x) or "unknown")
        return pubkey_x

    async def start_flash(self, image: str, install: bool) -> dict:
        """Kick the flash off and answer at once; the rest is in /api/state."""
        self._claim("flash")
        self.state.set_gates([])
        self.state.set_flash(stage="running", image=image, install=install,
                             sha256=None,
                             message="checking the image against the device")
        self._job = asyncio.ensure_future(self._flash_job(image, install))
        return {"started": "flash", "image": image, "install": install,
                "dry_run": not install}

    async def _flash_job(self, image_path: str, install: bool) -> None:
        try:
            await bounded(self._flash(image_path, install), FLASH_JOB_TIMEOUT)
        except asyncio.CancelledError:
            self._fail("cancelled — nothing was installed")
            raise
        except asyncio.TimeoutError:
            self._fail(f"gave up after {FLASH_JOB_TIMEOUT:.0f} s. Nothing was "
                       "installed; the image on the card may be incomplete, "
                       "which is harmless until it is installed.")
        except Exception as e:
            self._fail(f"{type(e).__name__}: {e}")
        finally:
            self.state.set_busy(None)

    def _fail(self, message: str) -> None:
        self.state.set_flash(stage="failed", message=message)
        self.state.note(f"flash failed: {message}")

    async def _flash(self, image_path: str, install: bool) -> None:
        """The whole sequence. Identical for a dry run until the very last line."""
        with open(image_path, "rb") as f:
            image = f.read()
        expected, source = fb.expected_digest_for(image_path, None)
        self.state.note(f"flashing {os.path.basename(image_path)}: {len(image)} "
                        f"bytes, sha256 {hashlib.sha256(image).hexdigest()}")

        pubkey_x = ""
        if self.fs.mode == fb.MODE_SLEEP:
            raw = await bounded(self.fs.read_file(fb.FLYSIGHT_TXT),
                                FILE_JOB_TIMEOUT)
            pubkey_x = self._absorb_flysight_txt(raw)

        gates = fb.flash_gates(image=image, filename=image_path,
                               expected_sha256=expected, digest_source=source,
                               mode=self.fs.mode, battery=self.fs.battery,
                               mtu=self.client.mtu_size, pubkey_x=pubkey_x)
        self.state.set_gates(render_gates(gates))
        for gate in gates:
            self.state.note(f"[{gate_status(gate)}] {gate.name}: {gate.detail}")

        if not gates_passed(gates):
            self.state.set_flash(stage="refused",
                                 message="A gate said stop. Nothing was "
                                         "written; the FlySight still boots "
                                         "what it always did.")
            self.state.note("refused — nothing written")
            return

        # From here the device is written to, and all of it is still reversible:
        # the image is only a file on the card until opcode 0x04 goes out.
        try:
            await bounded(self.fs.make_directory(fb.FIRMWARE_DIR), 30.0)
            self.state.note(f"created {fb.FIRMWARE_DIR}")
        except Exception:
            self.state.note(f"{fb.FIRMWARE_DIR} already exists")

        self.state.set_flash(stage="uploading",
                             message=f"uploading {len(image)} bytes to "
                                     f"{fb.FIRMWARE_PATH}")
        await self.fs.write_file(fb.FIRMWARE_PATH, image, self._progress("upload"))

        self.state.set_flash(stage="verifying",
                             message="reading the image back off the card")
        echo = await self.fs.read_file(fb.FIRMWARE_PATH, self._progress("verify"))
        digest = hashlib.sha256(echo).hexdigest()
        self.state.set_flash(sha256=digest)
        if digest != hashlib.sha256(image).hexdigest():
            self.state.set_flash(
                stage="failed",
                message=f"MISMATCH: {len(echo)} of {len(image)} bytes on the "
                        "card and the digests differ. NOT installing — upload "
                        "again; the device still boots its current firmware.")
            self.state.note("read-back mismatch — not installing")
            return
        self.state.note(f"on-device image verified: {len(echo)} bytes, "
                        f"sha256 {digest}")

        if not install:
            self.state.set_flash(
                stage="verified",
                message="DRY RUN: the image is on the card and verified, and "
                        "DS_CMD_INSTALL_UPLOADED_FIRMWARE (0x04) was NOT sent. "
                        "The FlySight still runs its current firmware.")
            self.state.note("dry run complete — 0x04 not sent")
            return

        self.state.set_flash(stage="installing",
                             message="install command sent; it reboots into "
                                     "its bootloader — do not switch it off")
        self.state.note("sending 0x04 (install uploaded firmware)")
        try:
            status = await bounded(self.fs.install_uploaded_firmware(), 30.0)
        except Exception as e:
            self.state.set_flash(
                stage="failed",
                message=f"no answer to the install command ({type(e).__name__}: "
                        f"{e}). If it rebooted anyway it is installing; give it "
                        "a minute, then reconnect and check the version.")
            return
        if status == fb.CP_NOT_SUPPORTED:
            self.state.set_flash(
                stage="refused",
                message="this firmware has no install-over-BLE command (it "
                        "arrived in v0.0.12). The image is on the card and is "
                        "harmless there; install once over USB.")
            return
        if status != fb.CP_SUCCESS:
            self.state.set_flash(
                stage="refused",
                message=f"the FlySight answered "
                        f"'{fb.CP_STATUS.get(status, hex(status))}'. Nothing "
                        "was installed; it still runs the firmware it had.")
            return
        self.state.set_flash(
            stage="installed",
            message="Accepted. It is rebooting to install — the disconnect is "
                    "expected. Give it half a minute; the app will find it "
                    "again by itself.")
        self.state.note("install accepted — rebooting")


# ---------------------------------------------------------------------------
# The bridge from HTTP threads into the loop
# ---------------------------------------------------------------------------

class LinkController:
    """What `handle_api` is allowed to do, and the only thread crossing."""

    def __init__(self, loop, link: Link, state: AppState,
                 image_dir: str = IMAGE_DIR):
        self.loop = loop
        self.link = link
        self.state = state
        self.image_dir = image_dir

    def _call(self, coro, timeout: float):
        """Run a coroutine on the link's loop and wait, but never forever."""
        try:
            future = asyncio.run_coroutine_threadsafe(coro, self.loop)
        except RuntimeError as e:  # loop already closed
            coro.close()
            raise ApiError(503, f"the link loop is not running: {e}") from None
        try:
            return future.result(timeout)
        except concurrent.futures.TimeoutError:
            future.cancel()
            raise ApiError(504, f"the link loop did not answer within "
                                f"{timeout:.0f} s") from None
        except ApiError:
            raise
        except Exception as e:
            raise ApiError(500, f"{type(e).__name__}: {e}") from None

    def snapshot(self) -> dict:
        return self.state.snapshot()

    def start_retry(self) -> dict:
        return self._call(self.link.start_retry(), HTTP_CALL_TIMEOUT)

    def stop_retry(self) -> dict:
        return self._call(self.link.stop_retry(), HTTP_CALL_TIMEOUT)

    def disconnect(self) -> dict:
        return self._call(self.link.disconnect(), HTTP_CALL_TIMEOUT + 15.0)

    def read_file(self, path: str) -> dict:
        return self._call(self.link.read_file(path), HTTP_FILE_TIMEOUT + 5.0)

    def start_flash(self, image: str, install: bool) -> dict:
        return self._call(self.link.start_flash(image, install), HTTP_CALL_TIMEOUT)


# ---------------------------------------------------------------------------
# The page
# ---------------------------------------------------------------------------

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<title>FlySight over Bluetooth</title>
<style>
 body{font:14px/1.45 -apple-system,Helvetica,sans-serif;margin:0 auto;padding:1.2rem;
      max-width:52rem;color:#222}
 h1{font-size:1.2rem;margin:0 0 .2rem} h2{font-size:1rem;margin:1.4rem 0 .4rem}
 .hint{color:#555;margin:.2rem 0 .8rem}
 .badge{display:inline-block;padding:.1rem .5rem;border-radius:.5rem;color:#fff;
        font-weight:600}
 .idle{background:#888} .scanning{background:#c78100} .connecting{background:#c78100}
 .connected{background:#1a7f37}
 button{font:inherit;padding:.35rem .8rem;margin:0 .3rem .3rem 0;border-radius:.4rem;
        border:1px solid #bbb;background:#f6f6f6;cursor:pointer}
 button:disabled{opacity:.45;cursor:default}
 button.danger{border-color:#a11;color:#a11}
 input[type=text]{font:inherit;width:100%;padding:.3rem;box-sizing:border-box}
 table{border-collapse:collapse} td{padding:.1rem .8rem .1rem 0;vertical-align:top}
 td.k{color:#666}
 pre{background:#f4f4f4;padding:.6rem;overflow:auto;max-height:20rem;
     white-space:pre-wrap;word-break:break-word}
 .bar{height:.9rem;background:#eee;border-radius:.45rem;overflow:hidden;margin:.4rem 0}
 .bar>div{height:100%;background:#1a7f37;width:0}
 ul.gates{list-style:none;padding:0;margin:.3rem 0}
 ul.gates li{padding:.15rem 0} .ok{color:#1a7f37} .warn{color:#c78100} .stop{color:#a11}
 .msg{margin:.4rem 0;padding:.5rem;background:#f4f4f4;border-left:3px solid #888}
 small{color:#666}
</style></head><body>

<h1>FlySight over Bluetooth</h1>
<div><span id="badge" class="badge idle">idle</span>
     <span id="busy"></span></div>
<p class="hint" id="hint"></p>

<button onclick="post('/api/connect')">Connect &amp; pair</button>
<button onclick="post('/api/stop')">Stop</button>
<button onclick="post('/api/disconnect')">Disconnect</button>

<h2>Device</h2>
<table id="device"></table>

<h2>Files</h2>
<button onclick="readFile()">Read /flysight.txt</button>
<input type="text" id="filepath" value="/flysight.txt">
<pre id="file">—</pre>

<h2>Firmware</h2>
<input type="text" id="image" placeholder="path to a .sfb">
<div>
  <button onclick="flash(false)">Verify image (dry run)</button>
  <label><input type="checkbox" id="arm"> arm install</label>
  <button class="danger" id="install" onclick="flash(true)" disabled>Install firmware</button>
</div>
<small>The dry run does everything the install does — gates, upload,
read-back, sha256 — except send opcode 0x04. The checkbox is off by default and
turns itself off again after every use.</small>
<div class="msg" id="flashmsg">idle</div>
<div class="bar"><div id="bar"></div></div>
<div id="progress"><small>&nbsp;</small></div>
<ul class="gates" id="gates"></ul>

<h2>Log</h2>
<pre id="log">—</pre>

<script>
const $ = id => document.getElementById(id);
let touchedImage = false;
$('image').addEventListener('input', () => { touchedImage = true; });
$('arm').addEventListener('change', e => { $('install').disabled = !e.target.checked; });

async function post(url, body) {
  try {
    const r = await fetch(url, {method:'POST', headers:{'Content-Type':'application/json'},
                               body: body ? JSON.stringify(body) : null});
    const j = await r.json();
    if (!j.ok) alert(j.error);
    refresh();
  } catch (e) { alert(e); }
}

async function readFile() {
  const p = $('filepath').value.trim() || '/flysight.txt';
  $('file').textContent = 'reading ' + p + '…';
  try {
    const r = await fetch('/api/file?path=' + encodeURIComponent(p));
    const j = await r.json();
    if (!j.ok) { $('file').textContent = j.error; return; }
    $('file').textContent = j.text;
  } catch (e) { $('file').textContent = String(e); }
}

function flash(install) {
  const body = {image: $('image').value.trim(), install: install};
  if (install) {
    if (!confirm('Install this image? The FlySight reboots into its bootloader.')) return;
    body.confirm = 'INSTALL';
  }
  $('arm').checked = false; $('install').disabled = true;   // resets every time
  post('/api/flash', body);
}

function row(k, v) { return v == null || v === '' ? '' :
  '<tr><td class="k">' + k + '</td><td>' + String(v).replace(/[<&]/g, c =>
    c === '<' ? '&lt;' : '&amp;') + '</td></tr>'; }

function refresh() {
  fetch('/api/state').then(r => r.json()).then(s => {
    $('badge').textContent = s.link;
    $('badge').className = 'badge ' + s.link;
    $('busy').textContent = s.busy ? '· ' + s.busy + ' in progress' : '';
    $('hint').textContent = s.hint;

    const d = s.device || {};
    $('device').innerHTML = row('Mode', d.mode) + row('Battery',
        d.battery == null ? null : d.battery + ' %' +
          (d.battery === 0 ? '  (never measured — it only samples in ACTIVE)' : '')) +
      row('ATT MTU', d.mtu) + row('Firmware', d.firmware_full || d.firmware) +
      row('Batch', d.batch) + row('Pubkey_X', d.pubkey_x ? d.pubkey_x + '…' : null) +
      row('Stack / FUS', d.stack_ver ? d.stack_ver + ' / ' + d.fus_ver : null) +
      row('Device ID', d.device_id) + row('Address', d.address) +
      row('RSSI', d.rssi == null ? null : d.rssi + ' dBm') ||
      '<tr><td class="k">—</td><td>nothing read yet</td></tr>';

    if (!touchedImage && s.default_image && !$('image').value)
      $('image').value = s.default_image;

    const f = s.flash || {};
    $('flashmsg').textContent = (f.stage || 'idle') +
      (f.message ? ' — ' + f.message : '');
    $('gates').innerHTML = (s.gates || []).map(g =>
      '<li class="' + g.status + '">[' + g.status + '] ' + g.name + ': ' +
      g.detail + '</li>').join('');

    const p = s.progress;
    $('bar').style.width = p && p.percent != null ? p.percent + '%' : '0';
    $('progress').innerHTML = '<small>' + (p ?
      p.label + ' ' + p.done + (p.total ? '/' + p.total : '') + ' B ' +
      (p.percent != null ? '(' + p.percent + ' %) ' : '') + p.rate_kbs + ' kB/s'
      : '&nbsp;') + '</small>';

    $('log').textContent = (s.log || []).join('\\n') || '—';
    if (s.file && $('file').textContent === '—') $('file').textContent = s.file.text;
  }).catch(() => { $('badge').textContent = 'app not answering'; });
}
refresh(); setInterval(refresh, 1000);
</script>
</body></html>
"""


# ---------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------

class Handler(http.server.BaseHTTPRequestHandler):
    """A thin adapter: parse, hand to `handle_api`, serialise. No logic here."""

    controller = None
    protocol_version = "HTTP/1.1"
    server_version = "flysight_app"

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler's naming
        self._serve("GET")

    def do_POST(self):  # noqa: N802
        self._serve("POST")

    # Anything else gets a JSON 404/405 too, rather than the base class's
    # "Unsupported method" HTML.
    def do_PUT(self):  # noqa: N802
        self._serve("PUT")

    def do_DELETE(self):  # noqa: N802
        self._serve("DELETE")

    def do_PATCH(self):  # noqa: N802
        self._serve("PATCH")

    def _serve(self, method: str) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if method == "GET" and parsed.path in ("/", "/index.html"):
            self._send(200, PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return
        if parsed.path == "/favicon.ico":
            # There isn't one. Say so quietly, or every page load logs an
            # error in the browser console that looks like the app's fault.
            self._send(204, b"", "text/plain")
            return
        query = {k: v[0] for k, v in
                 urllib.parse.parse_qs(parsed.query, keep_blank_values=True).items()}
        # Always drain the body, whatever the method: an unread body desyncs a
        # keep-alive connection and the next poll would read garbage.
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        raw = self.rfile.read(length) if length > 0 else b""
        try:
            status, obj = handle_api(method, parsed.path, query, raw, self.controller)
        except Exception as e:  # a bug here must still answer JSON
            status, obj = 500, {"ok": False, "error": f"{type(e).__name__}: {e}"}
        self._send(status, json.dumps(obj, indent=1).encode("utf-8"),
                   "application/json")

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        try:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass  # a page that reloaded mid-poll; nothing to say about it

    def log_message(self, *_args):
        pass  # the interesting log is on the page, not in the terminal


class Server(http.server.ThreadingHTTPServer):
    daemon_threads = True
    # SO_REUSEADDR lets a restart bind a port still in TIME_WAIT from the run
    # that just ended — without it, stopping and starting the app fails with
    # "Address already in use" for a minute. It does *not* let two instances
    # listen at once (that would need SO_REUSEPORT), and `port_is_taken`
    # catches the live-instance case before the bind anyway.
    allow_reuse_address = True


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="flysight_app.py",
        description="A long-lived app that holds a Bluetooth link to a "
                    "FlySight 2 and flashes it, with a page and a JSON API on "
                    "127.0.0.1.",
        epilog="Run in the bleak venv with the macOS sandbox OFF: "
               "Tools/flysight_app.sh")
    parser.add_argument("--port", type=int, default=PORT, metavar="N",
                        help=f"loopback port for the page and the API "
                             f"(default {PORT})")
    parser.add_argument("--address", metavar="UUID", default=None,
                        help="only ever connect to this CoreBluetooth UUID")
    parser.add_argument("--image-dir", default=IMAGE_DIR, metavar="DIR",
                        help="where the page looks for the newest .sfb "
                             f"(default {IMAGE_DIR})")
    parser.add_argument("--window", type=int, default=fb.WINDOW_LENGTH,
                        metavar="N",
                        help=f"Go-Back-N window, 1..{fb.WINDOW_LENGTH} "
                             f"(default {fb.WINDOW_LENGTH}); lower it if "
                             "uploads stall")
    parser.add_argument("--scan-timeout", type=float, default=SCAN_TIMEOUT,
                        metavar="SEC", help="length of one scan pass "
                                            f"(default {SCAN_TIMEOUT:.0f})")
    # Idle by default. Scanning only finds a FlySight that is in the room, and
    # the owner is usually not in it — an app that scans from launch until
    # someone comes home is a radio and a battery burned for nothing, and its
    # log fills with "no FlySight in that scan" until the one real line is
    # impossible to see. Connecting is a thing you ask for, with the button,
    # standing next to the device.
    parser.add_argument("--autoconnect", action="store_true",
                        help="start looking straight away instead of idle")
    return parser


def port_is_taken(port: int, host: str = HOST) -> bool:
    """True if something is already listening. Checked before the bind so the
    message can name the likely culprit instead of raising OSError at a user."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.settimeout(0.5)
    try:
        return probe.connect_ex((host, port)) == 0
    finally:
        probe.close()


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if not 1 <= args.window <= fb.WINDOW_LENGTH:
        print(f"--window must be 1..{fb.WINDOW_LENGTH}", file=sys.stderr)
        return 2
    if fb.BleakClient is None:
        print("bleak is not installed. Run this with ~/.venvs/ble/bin/python, "
              "and with the macOS sandbox off — CoreBluetooth is not reachable "
              "from a sandboxed process.", file=sys.stderr)
        return 1

    # Before anything is built: a second instance on the same port must say so
    # in a sentence, not raise OSError at someone holding a FlySight.
    if port_is_taken(args.port, HOST):
        print(f"{HOST}:{args.port} is already listening — this app is probably "
              f"running already. Open http://{HOST}:{args.port}/ , or stop the "
              f"other one first (curl {HOST}:{args.port}/api/state to see "
              "whose it is).", file=sys.stderr)
        return 2

    state = AppState(image_dir=os.path.abspath(args.image_dir))
    fb.LOG_SINK = state.note  # protocol oddities land on the page

    loop = asyncio.new_event_loop()
    link = Link(state, address=args.address, window=args.window,
                scan_timeout=args.scan_timeout)
    Handler.controller = LinkController(loop, link, state,
                                        image_dir=state.image_dir)

    try:
        server = Server((HOST, args.port), Handler)
    except OSError as e:
        loop.close()
        if e.errno in (errno.EADDRINUSE, errno.EACCES):
            print(f"cannot bind {HOST}:{args.port}: {e.strerror}. Pick another "
                  "port with --port.", file=sys.stderr)
            return 2
        raise

    thread = threading.Thread(target=loop.run_forever, name="ble", daemon=True)
    thread.start()
    state.note("app started")
    if args.autoconnect:
        try:
            Handler.controller.start_retry()
        except ApiError as e:
            state.note(f"could not start looking: {e}")
    else:
        state.note("idle — press Connect & pair when you are next to the FlySight")

    print(f"FlySight app on http://{HOST}:{args.port}/  (loopback only, no auth)",
          flush=True)
    print("Ctrl-C to stop.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping…")
    finally:
        server.shutdown()
        server.server_close()
        try:
            asyncio.run_coroutine_threadsafe(link.disconnect(), loop).result(15)
        except Exception:
            pass
        loop.call_soon_threadsafe(loop.stop)
        thread.join(timeout=5)
    return 0


if __name__ == "__main__":
    sys.exit(main())
