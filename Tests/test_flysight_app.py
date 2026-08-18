#!/usr/bin/env python3
"""Host tests for Tools/flysight_app.py — no radio, no FlySight, no browser.

The app is a long-lived process wrapped around `Tools/flysight_ble.py`, and
everything it decides on its own is pure: which image the page offers, how a
gate reads, what the API answers, and what the link's job machine does. All of
that is pinned down here.

The flash and file jobs are driven end to end against the same Python model of
`FS_CRS_State_Idle/_Read/_Write` that Tests/test_flysight_ble.py uses — so the
assertion that a dry run never sends opcode 0x04 is made against a device
model, not against a mock of our own code.

    cd Tests && python3 test_flysight_app.py     # or: make run
"""
import asyncio
import io
import json
import os
import socket
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, os.pardir, "Tools"))

import flysight_ble as fb  # noqa: E402
import flysight_app as fa  # noqa: E402
import test_flysight_ble as tb  # noqa: E402  (the device model lives there)

SFU1 = b"SFU1" + b"\xa5" * 900


# ---------------------------------------------------------------------------
# Which image the page offers
# ---------------------------------------------------------------------------

class TestNewestSfb(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def write(self, name, mtime=None):
        path = os.path.join(self.tmp.name, name)
        with open(path, "wb") as f:
            f.write(SFU1)
        if mtime is not None:
            os.utime(path, (mtime, mtime))
        return path

    def test_an_empty_directory_offers_nothing(self):
        self.assertIsNone(fa.newest_sfb(self.tmp.name))

    def test_a_missing_directory_offers_nothing(self):
        self.assertIsNone(fa.newest_sfb(os.path.join(self.tmp.name, "nope")))

    def test_the_newest_wins_regardless_of_name(self):
        now = time.time()
        self.write("zzz_old.sfb", now - 3600)
        newest = self.write("aaa_new.sfb", now)
        self.assertEqual(fa.newest_sfb(self.tmp.name), newest)

    def test_only_sfb_files_count(self):
        # The deploy step leaves UserApp.bin and sha256sums.txt in the same
        # tree; offering either would be an image the bootloader cannot read.
        self.write("B2_UserApp.sfb", time.time() - 60)
        for other in ("UserApp.bin", "sha256sums.txt", "notes.sfb.txt"):
            with open(os.path.join(self.tmp.name, other), "w") as f:
                f.write("x")
        self.assertTrue(fa.newest_sfb(self.tmp.name).endswith("B2_UserApp.sfb"))


# ---------------------------------------------------------------------------
# Gate rendering
# ---------------------------------------------------------------------------

class TestGateRendering(unittest.TestCase):

    def test_the_three_verdicts_are_distinct(self):
        self.assertEqual(fa.gate_status(fb.Gate("a", True, "")), "ok")
        self.assertEqual(fa.gate_status(fb.Gate("b", True, "", warn=True)), "warn")
        self.assertEqual(fa.gate_status(fb.Gate("c", False, "")), "stop")

    def test_a_failed_gate_is_a_stop_even_if_it_also_warns(self):
        # `ok` is the only thing that decides; a warning never rescues a stop.
        self.assertEqual(fa.gate_status(fb.Gate("d", False, "", warn=True)), "stop")
        self.assertFalse(fa.gates_passed([fb.Gate("d", False, "", warn=True)]))

    def test_rendering_keeps_the_order_and_the_detail(self):
        gates = [fb.Gate("SFU1 magic", True, "first four bytes are SFU1"),
                 fb.Gate("sha256", True, "not checked", warn=True),
                 fb.Gate("battery", False, "reads 0 %")]
        rows = fa.render_gates(gates)
        self.assertEqual([r["name"] for r in rows],
                         ["SFU1 magic", "sha256", "battery"])
        self.assertEqual([r["status"] for r in rows], ["ok", "warn", "stop"])
        self.assertEqual(rows[2]["detail"], "reads 0 %")
        json.dumps(rows)  # it has to survive the API

    def test_a_warning_alone_still_passes(self):
        self.assertTrue(fa.gates_passed([fb.Gate("sha256", True, "", warn=True)]))

    def test_the_real_gate_set_renders_whole(self):
        gates = fb.flash_gates(image=SFU1, filename="B2_UserApp.sfb",
                               expected_sha256=None, digest_source=None,
                               mode=fb.MODE_SLEEP, battery=91, mtu=250,
                               pubkey_x="486bee2dfeed")
        rows = fa.render_gates(gates)
        self.assertEqual(len(rows), 6)
        self.assertTrue(fa.gates_passed(gates))
        self.assertEqual(rows[1]["status"], "warn")  # no reference digest


# ---------------------------------------------------------------------------
# The state machine
# ---------------------------------------------------------------------------

class TestAppState(unittest.TestCase):

    def setUp(self):
        self.state = fa.AppState(image_dir=tempfile.mkdtemp())

    def test_a_fresh_app_is_idle_and_not_looking(self):
        snap = self.state.snapshot()
        self.assertEqual(snap["link"], fa.IDLE)
        self.assertFalse(snap["retrying"])
        self.assertIsNone(snap["busy"])
        self.assertEqual(snap["flash"]["stage"], "idle")
        json.dumps(snap)

    def test_the_hint_names_the_double_press_while_it_is_looking(self):
        self.state.set_link(fa.SCANNING, retrying=True)
        hint = self.state.snapshot()["hint"]
        self.assertIn("Double-press", hint)
        self.assertIn("already retrying", hint)

    def test_a_still_idle_app_says_press_connect(self):
        self.assertIn("Press Connect", fa.link_hint(fa.IDLE, False))

    def test_a_connected_app_stops_talking_about_pairing(self):
        self.assertNotIn("Double-press", fa.link_hint(fa.CONNECTED, True))

    def test_losing_the_link_forgets_the_device(self):
        # Stale battery and mode on the page would be read as current, and the
        # flash gates turn on exactly those two.
        self.state.set_link(fa.CONNECTED)
        self.state.set_device(battery=91, mode="SLEEP")
        self.state.set_link(fa.SCANNING, retrying=True)
        self.assertEqual(self.state.snapshot()["device"], {})

    def test_device_fields_merge_and_none_never_overwrites(self):
        self.state.set_link(fa.CONNECTED)
        self.state.set_device(battery=91, mode="SLEEP")
        self.state.set_device(battery=None, mtu=250)
        device = self.state.snapshot()["device"]
        self.assertEqual(device["battery"], 91)
        self.assertEqual(device["mtu"], 250)

    def test_the_log_keeps_the_last_fifty_lines(self):
        for i in range(120):
            self.state.note(f"line {i}")
        log = self.state.snapshot()["log"]
        self.assertEqual(len(log), fa.LOG_LINES)
        self.assertIn("line 119", log[-1])
        self.assertIn("line 70", log[0])

    def test_going_idle_clears_the_progress_bar(self):
        self.state.set_busy("flash")
        self.state.set_progress(fa.progress_record("upload", 10, 100, 1.0))
        self.state.set_busy(None)
        self.assertIsNone(self.state.snapshot()["progress"])

    def test_a_snapshot_is_a_copy(self):
        self.state.set_link(fa.CONNECTED)
        self.state.set_device(battery=91)
        snap = self.state.snapshot()
        self.state.set_device(battery=12)
        self.assertEqual(snap["device"]["battery"], 91)


class TestProgress(unittest.TestCase):

    def test_a_percentage_and_a_rate(self):
        p = fa.progress_record("upload", 51200, 102400, 2.0)
        self.assertEqual(p["percent"], 50)
        self.assertEqual(p["rate_kbs"], 25.0)

    def test_a_download_of_unknown_length_has_no_percentage(self):
        # A read does not know the file's size until the end of it.
        self.assertIsNone(fa.progress_record("read", 900, None, 1.0)["percent"])

    def test_the_bar_never_exceeds_full_and_never_divides_by_zero(self):
        self.assertEqual(fa.progress_record("upload", 300, 100, 0.0)["percent"], 100)
        self.assertEqual(fa.progress_record("upload", 300, 100, 0.0)["rate_kbs"], 0.0)


# ---------------------------------------------------------------------------
# The API's request and response shapes
# ---------------------------------------------------------------------------

class FakeController:
    """The whole surface `handle_api` is allowed to touch."""

    def __init__(self, image_dir):
        self.image_dir = image_dir
        self.calls = []
        self.error = None

    def _do(self, name, **kwargs):
        self.calls.append((name, kwargs))
        if self.error is not None:
            raise self.error
        return dict(kwargs)

    def snapshot(self):
        self.calls.append(("snapshot", {}))
        return {"link": fa.IDLE, "log": []}

    def start_retry(self):
        return self._do("start_retry")

    def stop_retry(self):
        return self._do("stop_retry")

    def disconnect(self):
        return self._do("disconnect")

    def read_file(self, path):
        return self._do("read_file", path=path)

    def start_flash(self, image, install):
        # Same shape the real LinkController returns: started at once, with
        # the rest reported through /api/state.
        answer = self._do("start_flash", image=image, install=install)
        answer.update({"started": "flash", "dry_run": not install})
        return answer


def call(ctrl, method, path, query=None, body=None):
    raw = b"" if body is None else json.dumps(body).encode()
    status, obj = fa.handle_api(method, path, query or {}, raw, ctrl)
    json.dumps(obj)  # every answer has to be serialisable
    return status, obj


class TestApi(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.image = os.path.join(self.tmp.name, "B2_UserApp.sfb")
        with open(self.image, "wb") as f:
            f.write(SFU1)
        self.ctrl = FakeController(self.tmp.name)

    # --- the simple ones --------------------------------------------------

    def test_state_reports_ok_alongside_the_state(self):
        status, obj = call(self.ctrl, "GET", "/api/state")
        self.assertEqual(status, 200)
        self.assertTrue(obj["ok"])
        self.assertEqual(obj["link"], fa.IDLE)

    def test_the_three_link_buttons(self):
        for path, name in (("/api/connect", "start_retry"),
                           ("/api/stop", "stop_retry"),
                           ("/api/disconnect", "disconnect")):
            status, obj = call(self.ctrl, "POST", path)
            self.assertEqual(status, 200, path)
            self.assertTrue(obj["ok"], path)
            self.assertIn(name, [c[0] for c in self.ctrl.calls])

    def test_the_wrong_method_is_a_405_not_a_crash(self):
        status, obj = call(self.ctrl, "GET", "/api/connect")
        self.assertEqual(status, 405)
        self.assertFalse(obj["ok"])
        self.assertIn("POST", obj["error"])
        self.assertEqual(self.ctrl.calls, [])

    def test_an_unknown_endpoint_is_json_not_html(self):
        status, obj = call(self.ctrl, "GET", "/favicon.ico")
        self.assertEqual(status, 404)
        self.assertFalse(obj["ok"])
        self.assertIn("/api/state", obj["error"])

    # --- files ------------------------------------------------------------

    def test_file_defaults_to_flysight_txt(self):
        status, obj = call(self.ctrl, "GET", "/api/file")
        self.assertEqual(status, 200)
        self.assertEqual(obj["path"], fb.FLYSIGHT_TXT)

    def test_a_relative_path_is_refused(self):
        status, obj = call(self.ctrl, "GET", "/api/file", {"path": "flysight.txt"})
        self.assertEqual(status, 400)
        self.assertEqual(self.ctrl.calls, [])

    def test_a_failed_read_keeps_its_status(self):
        self.ctrl.error = fa.ApiError(502, "the FlySight disconnected")
        status, obj = call(self.ctrl, "GET", "/api/file")
        self.assertEqual(status, 502)
        self.assertIn("disconnected", obj["error"])

    def test_not_connected_is_a_409(self):
        self.ctrl.error = fa.ApiError(409, "not connected")
        self.assertEqual(call(self.ctrl, "POST", "/api/flash",
                              body={"image": self.image})[0], 409)

    # --- flashing ---------------------------------------------------------

    def test_a_flash_starts_and_returns_at_once(self):
        status, obj = call(self.ctrl, "POST", "/api/flash", body={"image": self.image})
        self.assertEqual(status, 200)
        self.assertEqual(obj["image"], self.image)
        self.assertFalse(obj["install"])
        self.assertEqual(self.ctrl.calls[-1][1]["install"], False)

    def test_an_omitted_image_falls_back_to_the_newest_sfb(self):
        status, obj = call(self.ctrl, "POST", "/api/flash", body={})
        self.assertEqual(status, 200)
        self.assertEqual(obj["image"], self.image)

    def test_an_empty_image_directory_is_a_400(self):
        self.ctrl.image_dir = os.path.join(self.tmp.name, "empty")
        os.mkdir(self.ctrl.image_dir)
        status, obj = call(self.ctrl, "POST", "/api/flash", body={})
        self.assertEqual(status, 400)
        self.assertIn("no .sfb", obj["error"])

    def test_a_raw_bin_is_refused_before_anything_is_read(self):
        status, obj = call(self.ctrl, "POST", "/api/flash",
                           body={"image": "/tmp/UserApp.bin"})
        self.assertEqual(status, 400)
        self.assertIn(".sfb", obj["error"])
        self.assertEqual(self.ctrl.calls, [])

    def test_a_missing_file_is_refused_here_not_in_the_loop(self):
        status, obj = call(self.ctrl, "POST", "/api/flash",
                           body={"image": os.path.join(self.tmp.name, "B2_gone.sfb")})
        self.assertEqual(status, 400)
        self.assertIn("no such file", obj["error"])

    def test_install_without_the_confirmation_is_refused(self):
        status, obj = call(self.ctrl, "POST", "/api/flash",
                           body={"image": self.image, "install": True})
        self.assertEqual(status, 400)
        self.assertIn("confirm", obj["error"])
        self.assertEqual(self.ctrl.calls, [])  # nothing was started

    def test_install_with_the_wrong_word_is_refused(self):
        status, _ = call(self.ctrl, "POST", "/api/flash",
                         body={"image": self.image, "install": True,
                               "confirm": "yes"})
        self.assertEqual(status, 400)
        self.assertEqual(self.ctrl.calls, [])

    def test_install_with_the_confirmation_goes_through(self):
        status, obj = call(self.ctrl, "POST", "/api/flash",
                           body={"image": self.image, "install": True,
                                 "confirm": fa.INSTALL_CONFIRM})
        self.assertEqual(status, 200)
        self.assertTrue(obj["install"])
        self.assertFalse(obj["dry_run"])
        self.assertEqual(self.ctrl.calls[-1][1]["install"], True)

    def test_the_confirmation_is_never_remembered(self):
        # It is a word in one request, not a mode the app sits in: the very
        # next call, without it, is a dry run again.
        call(self.ctrl, "POST", "/api/flash",
             body={"image": self.image, "install": True,
                   "confirm": fa.INSTALL_CONFIRM})
        status, obj = call(self.ctrl, "POST", "/api/flash", body={"image": self.image})
        self.assertEqual(status, 200)
        self.assertFalse(obj["install"])
        self.assertTrue(obj["dry_run"])

    # --- bodies -----------------------------------------------------------

    def test_a_broken_body_is_a_400_with_a_sentence(self):
        status, obj = fa.handle_api("POST", "/api/flash", {}, b"{not json",
                                    self.ctrl)
        self.assertEqual(status, 400)
        self.assertIn("not JSON", obj["error"])

    def test_a_json_array_is_not_a_body(self):
        status, _ = fa.handle_api("POST", "/api/flash", {}, b"[1,2]", self.ctrl)
        self.assertEqual(status, 400)

    def test_an_empty_body_is_an_empty_object(self):
        self.assertEqual(fa.parse_json_body(b""), {})
        self.assertEqual(fa.parse_json_body(b"  \n"), {})


# ---------------------------------------------------------------------------
# The jobs, against the device model from test_flysight_ble.py
# ---------------------------------------------------------------------------

class TestJobs(unittest.IsolatedAsyncioTestCase):
    """`Link` driven end to end with the radio replaced by a state machine."""

    def setUp(self):
        self._saved = (fb.COMMAND_TIMEOUT, fb.PACKET_TIMEOUT, fb.ACK_TIMEOUT)
        fb.COMMAND_TIMEOUT = fb.PACKET_TIMEOUT = fb.ACK_TIMEOUT = 0.05
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.image = os.path.join(self.tmp.name, "B2_UserApp.sfb")
        with open(self.image, "wb") as f:
            f.write(SFU1)

    def tearDown(self):
        fb.COMMAND_TIMEOUT, fb.PACKET_TIMEOUT, fb.ACK_TIMEOUT = self._saved

    async def link(self, mode=fb.MODE_SLEEP, battery=91, mtu=250, files=None):
        state = fa.AppState(image_dir=self.tmp.name)
        device = tb.FakeFlySight(files=files if files is not None
                                 else {fb.FLYSIGHT_TXT: tb.SAMPLE_TXT})
        client = tb.FakeClient(device, mode=mode, battery=battery, mtu=mtu)
        link = fa.Link(state)
        link.client = client
        link.fs = fb.FlySight(client)
        await link.fs.start()
        return link, state, device, client

    async def finish(self, link):
        if link._job is not None:
            await asyncio.wait_for(asyncio.shield(link._job), 20)

    # --- reading ----------------------------------------------------------

    async def test_reading_flysight_txt_fills_in_the_batch(self):
        link, state, _, _ = await self.link()
        record = await link.read_file(fb.FLYSIGHT_TXT)
        self.assertIn("Pubkey_X", record["text"])
        snap = state.snapshot()
        self.assertEqual(snap["device"]["batch"], "B2")
        self.assertEqual(snap["device"]["pubkey_x"], "486bee2d")
        self.assertEqual(snap["file"]["path"], fb.FLYSIGHT_TXT)
        self.assertIsNone(snap["busy"])

    async def test_reading_a_missing_file_fails_without_wedging_the_link(self):
        link, state, _, _ = await self.link()
        with self.assertRaises(fa.ApiError) as caught:
            await link.read_file("/NOPE.TXT")
        self.assertEqual(caught.exception.status, 502)
        self.assertIsNone(state.snapshot()["busy"])
        # …and the link still works afterwards.
        await link.read_file(fb.FLYSIGHT_TXT)

    async def test_a_job_refuses_to_run_twice_at_once(self):
        link, _, _, _ = await self.link()
        first = asyncio.ensure_future(link.read_file(fb.FLYSIGHT_TXT))
        await asyncio.sleep(0)
        with self.assertRaises(fa.ApiError) as caught:
            await link.read_file(fb.FLYSIGHT_TXT)
        self.assertEqual(caught.exception.status, 409)
        await first

    async def test_nothing_can_be_asked_of_a_link_that_is_not_up(self):
        state = fa.AppState(image_dir=self.tmp.name)
        link = fa.Link(state)
        with self.assertRaises(fa.ApiError) as caught:
            await link.read_file(fb.FLYSIGHT_TXT)
        self.assertEqual(caught.exception.status, 409)
        with self.assertRaises(fa.ApiError):
            await link.start_flash(self.image, False)

    # --- flashing ---------------------------------------------------------

    async def test_a_dry_run_uploads_verifies_and_never_sends_0x04(self):
        link, state, device, client = await self.link()
        await link.start_flash(self.image, install=False)
        await self.finish(link)

        snap = state.snapshot()
        self.assertEqual(snap["flash"]["stage"], "verified")
        self.assertIn("NOT sent", snap["flash"]["message"])
        self.assertEqual(device.written[fb.FIRMWARE_PATH], SFU1)
        self.assertFalse(client.install_requested)
        self.assertTrue(all(g["ok"] for g in snap["gates"]))
        self.assertIsNone(snap["busy"])

    async def test_the_install_path_is_the_dry_run_plus_one_opcode(self):
        link, state, device, client = await self.link()
        await link.start_flash(self.image, install=True)
        await self.finish(link)

        snap = state.snapshot()
        self.assertEqual(snap["flash"]["stage"], "installed")
        self.assertEqual(device.written[fb.FIRMWARE_PATH], SFU1)  # same upload
        self.assertTrue(client.install_requested)

    async def test_a_refused_gate_writes_nothing_at_all(self):
        link, state, device, client = await self.link(mode=fb.MODE_ACTIVE)
        await link.start_flash(self.image, install=True)
        await self.finish(link)

        snap = state.snapshot()
        self.assertEqual(snap["flash"]["stage"], "refused")
        self.assertEqual(device.written, {})
        self.assertFalse(client.install_requested)
        # Two stops, and the batch one is not noise: /flysight.txt can only be
        # read in SLEEP, so a device that is awake cannot say which batch it
        # is, and an unknown batch is a refusal in its own right.
        stopped = [g["name"] for g in snap["gates"] if g["status"] == "stop"]
        self.assertEqual(stopped, ["batch", "mode"])

    async def test_a_flat_battery_stops_it_even_though_everything_else_passes(self):
        link, state, device, client = await self.link(battery=0)
        await link.start_flash(self.image, install=True)
        await self.finish(link)
        self.assertEqual(state.snapshot()["flash"]["stage"], "refused")
        self.assertEqual(device.written, {})
        self.assertFalse(client.install_requested)

    async def test_the_wrong_batch_stops_it(self):
        # A B3 file against a B2 device: the bootloader could not decrypt it.
        wrong = os.path.join(self.tmp.name, "B3_UserApp.sfb")
        with open(wrong, "wb") as f:
            f.write(SFU1)
        link, state, device, client = await self.link()
        await link.start_flash(wrong, install=True)
        await self.finish(link)
        self.assertEqual(state.snapshot()["flash"]["stage"], "refused")
        self.assertFalse(client.install_requested)

    async def test_a_raw_binary_stops_at_the_magic(self):
        raw = os.path.join(self.tmp.name, "B2_UserApp.sfb")
        with open(raw, "wb") as f:
            f.write(b"\x00\x01\x02\x03" + b"x" * 100)
        link, state, device, client = await self.link()
        await link.start_flash(raw, install=True)
        await self.finish(link)
        snap = state.snapshot()
        self.assertEqual(snap["flash"]["stage"], "refused")
        self.assertEqual(snap["gates"][0]["status"], "stop")
        self.assertFalse(client.install_requested)

    async def test_progress_is_reported_while_it_runs(self):
        link, state, _, _ = await self.link()
        seen = []
        original = state.set_progress

        def spy(progress):
            if progress:
                seen.append(progress)
            original(progress)

        state.set_progress = spy
        await link.start_flash(self.image, install=False)
        await self.finish(link)
        self.assertTrue(any(p["label"] == "upload" for p in seen))
        self.assertTrue(any(p["label"] == "verify" for p in seen))
        self.assertEqual(seen[-1]["percent"], None if seen[-1]["total"] is None
                         else seen[-1]["percent"])

    async def test_a_gate_failure_leaves_the_link_usable(self):
        link, state, _, _ = await self.link(mode=fb.MODE_ACTIVE)
        await link.start_flash(self.image, install=False)
        await self.finish(link)
        self.assertIsNone(state.snapshot()["busy"])
        # The mode gate read the mode; a second attempt is not blocked by the
        # first one's failure.
        await link.start_flash(self.image, install=False)
        await self.finish(link)

    # --- the buttons that must never wedge --------------------------------

    async def test_stopping_and_disconnecting_an_idle_app_is_harmless(self):
        state = fa.AppState(image_dir=self.tmp.name)
        link = fa.Link(state)
        self.assertEqual((await link.stop_retry())["connected"], False)
        self.assertTrue((await link.disconnect())["disconnected"])
        self.assertEqual(state.snapshot()["link"], fa.IDLE)

    async def test_disconnect_really_stops_a_running_flash(self):
        # The version of this that used asyncio.wait_for for the job's overall
        # timeout passed this test for the wrong reason and then went on to
        # upload, verify and install the image *after* disconnect() had
        # returned — wait_for only requests its inner task's cancellation and
        # does not wait for it. Hence `bounded`. The settle below is what
        # catches a regression: an orphan task needs a few loop turns to do
        # its damage.
        link, state, device, client = await self.link()
        await link.start_flash(self.image, install=True)
        await asyncio.sleep(0)
        await link.disconnect()
        await asyncio.sleep(0.3)

        self.assertEqual(device.written, {})
        self.assertFalse(client.install_requested)
        self.assertIsNone(state.snapshot()["busy"])
        self.assertEqual(state.snapshot()["link"], fa.IDLE)
        self.assertIn("cancelled", state.snapshot()["flash"]["message"])

    async def test_bounded_waits_for_the_work_it_cancels(self):
        stopped = []

        async def slow():
            try:
                await asyncio.sleep(10)
            except asyncio.CancelledError:
                await asyncio.sleep(0)      # cleanup that needs a loop turn
                stopped.append("cleaned up")
                raise

        outer = asyncio.ensure_future(fa.bounded(slow(), 10))
        await asyncio.sleep(0)
        outer.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await outer
        self.assertEqual(stopped, ["cleaned up"])  # already done, not "soon"

    async def test_bounded_times_out_and_stops_the_work(self):
        async def slow():
            await asyncio.sleep(10)

        with self.assertRaises(asyncio.TimeoutError):
            await fa.bounded(slow(), 0.01)


# ---------------------------------------------------------------------------
# The retry loop, with the radio replaced
# ---------------------------------------------------------------------------

class TestRetryLoop(unittest.IsolatedAsyncioTestCase):
    """Scan, connect, hold, and start again by itself when the link drops."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        # The real pauses are seconds; the loop is the same shape without them.
        saved = (fa.RETRY_PAUSE, fa.REFRESH_INTERVAL)
        fa.RETRY_PAUSE, fa.REFRESH_INTERVAL = 0.001, 0.01
        self.addCleanup(lambda: setattr_pair(fa, saved))

        self.attempts = []
        self.fail_next = 0
        test = self

        class FakeScanner:
            @staticmethod
            async def discover(timeout=5.0, return_adv=False):
                await asyncio.sleep(0)
                return {"AAAA": (tb.FakeBleDevice("AAAA"),
                                 tb.FakeAdv("FlySight", -55))}

        class Client(tb.FakeClient):
            def __init__(self):
                super().__init__(tb.FakeFlySight(
                    files={fb.FLYSIGHT_TXT: tb.SAMPLE_TXT}))

            async def disconnect(self):
                self.is_connected = False

        async def fake_connect(device, window=8, timeout=30.0,
                               disconnected_callback=None):
            test.attempts.append(device.address)
            if test.fail_next > 0:
                test.fail_next -= 1
                raise RuntimeError("peripheral is not connectable")
            client = Client()
            flysight = fb.FlySight(client)
            await flysight.start()
            return client, flysight

        for name, value in (("BleakScanner", FakeScanner),
                            ("connect_device", fake_connect)):
            original = getattr(fb, name)
            setattr(fb, name, value)
            self.addCleanup(lambda n=name, o=original: setattr(fb, n, o))

    async def until(self, predicate, what, timeout=3.0):
        """Wait for a condition, or fail saying what the state actually was."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            await asyncio.sleep(0.005)
        self.fail(f"timed out waiting for {what}")

    async def settle(self, link, want, timeout=3.0):
        """Wait for the link to reach `want`, identity and all."""
        await self.until(
            lambda: link.state.link == want
            and (want != fa.CONNECTED or link.state.device.get("firmware")),
            f"link {want!r} (it is {link.state.link!r})", timeout)

    async def make(self):
        state = fa.AppState(image_dir=self.tmp.name)
        link = fa.Link(state, scan_timeout=0.01)
        self.addCleanup(lambda: link._retry_task and link._retry_task.cancel())
        return link, state

    async def test_it_scans_connects_and_reads_the_identity(self):
        link, state = await self.make()
        await link.start_retry()
        await self.settle(link, fa.CONNECTED)
        snap = state.snapshot()
        self.assertTrue(snap["retrying"])
        self.assertEqual(snap["device"]["address"], "AAAA")
        self.assertEqual(snap["device"]["mtu"], 250)
        self.assertEqual(snap["device"]["mode"], "SLEEP")
        self.assertTrue(snap["device"]["firmware"])
        self.assertIn("Linked", snap["hint"])

    async def test_a_dropped_link_goes_back_to_scanning_by_itself(self):
        link, state = await self.make()
        await link.start_retry()
        await self.settle(link, fa.CONNECTED)

        link._on_disconnected(None)  # what bleak calls when the link goes away
        await self.until(lambda: len(self.attempts) == 2, "a second connect")
        await self.settle(link, fa.CONNECTED)  # …and it comes back on its own
        self.assertTrue(any("link dropped" in line for line in state.snapshot()["log"]))
        self.assertTrue(state.snapshot()["retrying"])

    async def test_a_failed_connect_is_reported_and_retried(self):
        link, state = await self.make()
        self.fail_next = 2
        await link.start_retry()
        await self.settle(link, fa.CONNECTED)
        self.assertEqual(len(self.attempts), 3)  # two refusals, then the link
        self.assertTrue(any("connect failed" in line
                            for line in state.snapshot()["log"]))

    async def test_starting_twice_does_not_start_a_second_loop(self):
        link, state = await self.make()
        await link.start_retry()
        await self.settle(link, fa.CONNECTED)
        answer = await link.start_retry()
        self.assertFalse(answer["started"])
        await asyncio.sleep(0.05)
        self.assertEqual(self.attempts, ["AAAA"])

    async def test_stop_keeps_a_live_link_and_stops_looking(self):
        link, state = await self.make()
        await link.start_retry()
        await self.settle(link, fa.CONNECTED)

        answer = await link.stop_retry()
        self.assertTrue(answer["connected"])
        self.assertEqual(state.snapshot()["link"], fa.CONNECTED)
        self.assertFalse(state.snapshot()["retrying"])
        # …and the link is still usable, which is the point of not tearing it
        # down: the mode and battery the flash gates read stay current.
        record = await link.read_file(fb.FLYSIGHT_TXT)
        self.assertIn("Pubkey_X", record["text"])
        # When it does eventually drop, it goes idle instead of scanning again.
        link._on_disconnected(None)
        await self.until(lambda: link.state.link == fa.IDLE, "an idle link")
        await asyncio.sleep(0.05)
        self.assertEqual(self.attempts, ["AAAA"])

    async def test_disconnect_stops_the_loop_and_drops_the_link(self):
        link, state = await self.make()
        await link.start_retry()
        await self.settle(link, fa.CONNECTED)
        await link.disconnect()
        self.assertEqual(state.snapshot()["link"], fa.IDLE)
        self.assertFalse(state.snapshot()["retrying"])
        await asyncio.sleep(0.05)
        self.assertEqual(self.attempts, ["AAAA"])  # no reconnect behind our back

    async def test_the_battery_is_refreshed_while_the_link_is_idle(self):
        link, state = await self.make()
        await link.start_retry()
        await self.settle(link, fa.CONNECTED)
        link.client.battery = 42  # as if it had just been woken to ACTIVE
        await self.until(lambda: state.snapshot()["device"].get("battery") == 42,
                         "the battery to be re-read")


def setattr_pair(module, values):
    module.RETRY_PAUSE, module.REFRESH_INTERVAL = values


# ---------------------------------------------------------------------------
# The HTTP layer, over a real socket
# ---------------------------------------------------------------------------

class TestHttp(unittest.TestCase):
    """The server itself: a page, JSON everywhere else, and no HTML errors."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.ctrl = FakeController(cls.tmp.name)
        fa.Handler.controller = cls.ctrl
        cls.server = fa.Server((fa.HOST, 0), fa.Handler)  # 0 = any free port
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=5)
        cls.tmp.cleanup()

    def request(self, path, method="GET", body=None):
        url = f"http://{fa.HOST}:{self.port}{path}"
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(url, data=data, method=method,
                                         headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, response.headers["Content-Type"], \
                       response.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers["Content-Type"], e.read()

    def test_the_page_is_served_at_the_root(self):
        status, content_type, body = self.request("/")
        self.assertEqual(status, 200)
        self.assertIn("text/html", content_type)
        self.assertIn(b"/api/state", body)

    def test_state_comes_back_as_json(self):
        status, content_type, body = self.request("/api/state")
        self.assertEqual(status, 200)
        self.assertIn("application/json", content_type)
        self.assertTrue(json.loads(body)["ok"])

    def test_a_missing_endpoint_is_json_not_an_html_error_page(self):
        status, content_type, body = self.request("/api/nope")
        self.assertEqual(status, 404)
        self.assertIn("application/json", content_type)
        self.assertFalse(json.loads(body)["ok"])

    def test_an_unsupported_method_is_json_too(self):
        # BaseHTTPRequestHandler answers 501 in HTML on its own; an agent
        # parsing that gets a traceback instead of an error message.
        status, content_type, body = self.request("/api/state", method="DELETE")
        self.assertEqual(status, 405)
        self.assertIn("application/json", content_type)
        self.assertIn("GET", json.loads(body)["error"])

    def test_a_post_body_reaches_the_endpoint(self):
        image = os.path.join(self.tmp.name, "B2_UserApp.sfb")
        with open(image, "wb") as f:
            f.write(SFU1)
        status, _, body = self.request("/api/flash", "POST", {"image": image})
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["image"], image)

    def test_the_socket_survives_a_refused_request(self):
        # Same connection, one bad call, then a good one: a body that is read
        # but not drained desyncs keep-alive and the next poll reads garbage.
        self.request("/api/flash", "POST", {"image": "/tmp/x.bin"})
        self.assertEqual(self.request("/api/state")[0], 200)


class TestStartup(unittest.TestCase):

    def test_a_listening_port_is_seen_as_taken(self):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        sock.listen(1)
        self.addCleanup(sock.close)
        port = sock.getsockname()[1]
        self.assertTrue(fa.port_is_taken(port))
        sock.close()
        self.assertFalse(fa.port_is_taken(port))

    def test_starting_twice_on_one_port_is_a_sentence_not_a_traceback(self):
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        sock.listen(1)
        self.addCleanup(sock.close)
        port = sock.getsockname()[1]

        saved, fb.BleakClient = fb.BleakClient, object  # get past the venv check
        stderr, sys.stderr = sys.stderr, io.StringIO()
        try:
            code = fa.main(["--port", str(port)])
            message = sys.stderr.getvalue()
        finally:
            sys.stderr = stderr
            fb.BleakClient = saved
        self.assertEqual(code, 2)
        self.assertIn("already", message)
        self.assertNotIn("Traceback", message)

    def test_an_impossible_window_is_refused(self):
        stderr, sys.stderr = sys.stderr, io.StringIO()
        try:
            self.assertEqual(fa.main(["--window", "9"]), 2)
        finally:
            sys.stderr = stderr


if __name__ == "__main__":
    unittest.main(verbosity=2)
