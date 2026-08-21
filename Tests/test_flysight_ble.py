#!/usr/bin/env python3
"""Host tests for Tools/flysight_ble.py — no radio, no FlySight, no bleak.

Everything the tool decides before it touches a device is pure, and this is
where those decisions are pinned down: the CRS framing, the Go-Back-N window
(including the end-of-file rule that keeps a half-sent image from being
installed), the `/flysight.txt` parse, the batch table and the flash gates.

    cd Tests && python3 test_flysight_ble.py     # or: make run
"""
import asyncio
import hashlib
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "Tools"))

import flysight_ble as fb  # noqa: E402

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)


# ---------------------------------------------------------------------------
# A model of the firmware's write state machine
# ---------------------------------------------------------------------------

class FakeWriteTarget:
    """`FS_CRS_State_Write` from FlySight/crs.c:517-570, in twelve lines.

    Two behaviours matter and both are copied deliberately:

     * a data frame is written and acknowledged **only** when its counter is
       the one the device is waiting for (`:547`);
     * a two-byte data packet closes the file **whatever its counter was**
       (`:558-561`), because that check sits outside the counter test.

    The second is the trap: an end-of-file frame that arrives while a data
    frame is still missing closes a truncated file, and the truncated image
    installs and bricks the unit.
    """

    def __init__(self):
        self.next_packet = 0
        self.file = bytearray()
        self.closed = False

    def feed(self, packet):
        """Feed one packet; returns the ACK packets the device would send."""
        acks = []
        if self.closed or not packet:
            return acks
        if packet[0] == fb.CRS_FILE_DATA and len(packet) >= 2:
            if packet[1] == (self.next_packet & 0xFF):
                self.file += packet[2:]
                acks.append(fb.build_file_ack(packet[1]))
                self.next_packet += 1
            if len(packet) == 2:
                self.closed = True
        return acks


def drive_upload(data, device, drop=(), window=fb.WINDOW_LENGTH,
                 max_rounds=100000):
    """Run an UploadWindow against a FakeWriteTarget, dropping some sends.

    `drop` holds ordinals counted over every frame put on the wire, so a
    retransmission can be dropped as easily as a first attempt. Returns
    (window, early_eof) where `early_eof` records the invariant the uploader
    must never break.
    """
    w = fb.UploadWindow(data, window=window)
    drop = set(drop)
    ordinal = 0
    rounds = 0
    early_eof = False

    while not w.done:
        rounds += 1
        if rounds > max_rounds:
            raise AssertionError("upload did not converge")
        advanced = False
        for frame in w.pending():
            if len(frame) == 2 and w.base < w.data_frames:
                early_eof = True  # the thing that truncates files
            this, ordinal = ordinal, ordinal + 1
            if this in drop:
                continue
            for ack in device.feed(frame):
                if w.on_ack(ack[1]):
                    advanced = True
        if not advanced:
            w.on_timeout()  # the 500 ms ACK timeout, without the waiting

    return w, early_eof


# ---------------------------------------------------------------------------
# Framing
# ---------------------------------------------------------------------------

class TestFraming(unittest.TestCase):

    def test_paths_are_nul_terminated(self):
        self.assertEqual(fb.build_write_file("/FW/APP.SFB"),
                         b"\x03/FW/APP.SFB\x00")
        self.assertEqual(fb.build_mkdir("/FW"), b"\x04/FW\x00")

    def test_read_file_carries_offset_and_stride_minus_one(self):
        # crs.c:206-207 multiplies both by FRAME_LENGTH and adds 1 to the
        # stride, so a contiguous read puts a zero on the wire.
        packet = fb.build_read_file("/flysight.txt")
        self.assertEqual(packet[0], fb.CRS_READ)
        self.assertEqual(packet[1:5], b"\x00\x00\x00\x00")
        self.assertEqual(packet[5:9], b"\x00\x00\x00\x00")
        self.assertEqual(packet[9:], b"/flysight.txt\x00")

        packet = fb.build_read_file("/T.BIN", offset_frames=2, stride_frames=4)
        self.assertEqual(int.from_bytes(packet[1:5], "little"), 2)
        self.assertEqual(int.from_bytes(packet[5:9], "little"), 3)

    def test_read_file_rejects_impossible_parameters(self):
        # Both would be encoded as an enormous unsigned value.
        with self.assertRaises(ValueError):
            fb.build_read_file("/T.BIN", stride_frames=0)
        with self.assertRaises(ValueError):
            fb.build_read_file("/T.BIN", offset_frames=-1)

    def test_file_data_frames(self):
        self.assertEqual(fb.build_file_data(0x2A, b"hi"), b"\x10\x2ahi")
        self.assertEqual(fb.build_file_data(0x100 + 7, b""), b"\x10\x07")
        with self.assertRaises(ValueError):
            fb.build_file_data(0, b"x" * (fb.FRAME_LENGTH + 1))

    def test_parse_file_data(self):
        self.assertEqual(fb.parse_file_data(b"\x10\x05abc"), (5, b"abc"))
        self.assertEqual(fb.parse_file_data(b"\x10\x05"), (5, b""))
        with self.assertRaises(ValueError):
            fb.parse_file_data(b"\x10")
        with self.assertRaises(ValueError):
            fb.parse_file_data(b"\x11\x00")

    def test_parse_ack_nak(self):
        self.assertEqual(fb.parse_ack_nak(b"\xf1\x03"), fb.CRS_WRITE)
        self.assertEqual(fb.parse_ack_nak(b"\xf0\x10"), fb.CRS_FILE_DATA)
        with self.assertRaises(ValueError):
            fb.parse_ack_nak(b"\x10\x00")

    def test_control_point_responses(self):
        self.assertEqual(fb.decode_cp_response(b"\xf0\x01\x01v0.0.19"),
                         (0x01, 0x01, b"v0.0.19"))
        # Anything malformed is rejected, never guessed at: it must not be
        # allowed to complete a pending request.
        self.assertIsNone(fb.decode_cp_response(b"\xf0\x01"))
        self.assertIsNone(fb.decode_cp_response(b"\x01\x01\x01"))
        self.assertEqual(fb.cp_text(b"v0.0.19\x00\x00"), "v0.0.19")


# ---------------------------------------------------------------------------
# The uploader
# ---------------------------------------------------------------------------

class TestUploadWindow(unittest.TestCase):

    def test_window_is_bounded_by_the_devices_ring(self):
        with self.assertRaises(ValueError):
            fb.UploadWindow(b"x", window=fb.WINDOW_LENGTH + 1)
        with self.assertRaises(ValueError):
            fb.UploadWindow(b"x", window=0)

    def test_frame_count_includes_the_end_of_file_frame(self):
        w = fb.UploadWindow(b"x" * (fb.FRAME_LENGTH * 2 + 1))
        self.assertEqual(w.data_frames, 3)
        self.assertEqual(w.total_frames, 4)

    def test_empty_file_is_one_end_of_file_frame(self):
        w = fb.UploadWindow(b"")
        self.assertEqual(w.data_frames, 0)
        self.assertEqual([bytes(f) for f in w.pending()], [b"\x10\x00"])
        self.assertTrue(w.on_ack(0))
        self.assertTrue(w.done)

    def test_eof_is_held_back_until_every_data_frame_is_acknowledged(self):
        # The rule the whole design turns on. With four data frames, the
        # window would happily hold the EOF as its fifth — and the device
        # would close a truncated file (crs.c:558-561).
        data = b"a" * (fb.FRAME_LENGTH * 4)
        w = fb.UploadWindow(data)
        self.assertEqual(len(w.pending()), 4)  # not 5
        for counter in range(3):
            self.assertTrue(w.on_ack(counter))
            self.assertEqual(w.pending(), [])  # still nothing more to send
        self.assertTrue(w.on_ack(3))
        pending = w.pending()
        self.assertEqual([bytes(f) for f in pending], [b"\x10\x04"])

    def test_an_ack_inside_the_window_is_cumulative(self):
        # The device acknowledges in order and only after writing, so an ACK
        # for frame 2 means 0 and 1 landed as well. Waiting for each ACK
        # individually would deadlock when one goes missing.
        w = fb.UploadWindow(b"a" * (fb.FRAME_LENGTH * 3))
        w.pending()
        self.assertTrue(w.on_ack(2))
        self.assertEqual(w.base, 3)

    def test_stale_acks_do_not_advance_the_window(self):
        w = fb.UploadWindow(b"a" * (fb.FRAME_LENGTH * 3))
        w.pending()
        self.assertTrue(w.on_ack(0))
        self.assertFalse(w.on_ack(0))   # duplicate of one already counted
        self.assertFalse(w.on_ack(200))  # never sent
        w.on_ack(2)
        self.assertTrue(w.done or w.base == 3)
        self.assertFalse(w.on_ack(2))   # nothing in flight any more

    def test_timeout_rewinds_the_whole_window(self):
        w = fb.UploadWindow(b"a" * (fb.FRAME_LENGTH * 9))
        self.assertEqual(len(w.pending()), 8)  # frames 0..7
        w.on_ack(0)
        w.on_timeout()
        # Frames 1..8 go again — the window slid by one, and the ninth data
        # frame joins it; the EOF still does not, since frame 8 is unacked.
        resent = w.pending()
        self.assertEqual(len(resent), 8)
        self.assertEqual([f[1] for f in resent], list(range(1, 9)))

    def test_clean_upload_arrives_byte_for_byte(self):
        data = bytes(range(256)) * 20  # 5120 B, 22 frames, a partial last one
        device = FakeWriteTarget()
        _, early_eof = drive_upload(data, device)
        self.assertEqual(bytes(device.file), data)
        self.assertTrue(device.closed)
        self.assertFalse(early_eof)

    def test_lossy_upload_still_arrives_whole(self):
        data = os.urandom(fb.FRAME_LENGTH * 10 + 7)
        for drop in ([0], [3], [0, 1, 2, 3, 4, 5, 6, 7], [9, 10, 11], [0, 12, 25]):
            with self.subTest(drop=drop):
                device = FakeWriteTarget()
                _, early_eof = drive_upload(data, device, drop=drop)
                self.assertEqual(bytes(device.file), data)
                self.assertTrue(device.closed)
                self.assertFalse(early_eof)

    def test_upload_survives_the_counter_wrapping(self):
        # 300 frames: the one-byte counter wraps, and `base`/`next` must stay
        # unbounded while only their low byte goes on the wire.
        data = os.urandom(fb.FRAME_LENGTH * 300)
        device = FakeWriteTarget()
        w, early_eof = drive_upload(data, device, drop=[5, 260, 299])
        self.assertEqual(bytes(device.file), data)
        self.assertTrue(device.closed)
        self.assertFalse(early_eof)
        self.assertEqual(w.base, w.total_frames)

    def test_the_last_data_frame_may_be_short(self):
        data = b"z" * (fb.FRAME_LENGTH + 5)
        w = fb.UploadWindow(data)
        self.assertEqual(len(w.frame_at(0)), fb.FRAME_LENGTH + 2)
        self.assertEqual(len(w.frame_at(1)), 7)

    def test_progress_never_overstates_what_landed(self):
        data = b"q" * (fb.FRAME_LENGTH * 2 + 10)
        w = fb.UploadWindow(data)
        self.assertEqual(w.bytes_acknowledged(), 0)
        w.pending()
        w.on_ack(0)
        self.assertEqual(w.bytes_acknowledged(), fb.FRAME_LENGTH)
        w.on_ack(1)
        w.on_ack(2)
        self.assertEqual(w.bytes_acknowledged(), len(data))

    def test_an_eager_uploader_would_truncate_the_image(self):
        # Not a test of our code — a test of the trap it avoids, so that the
        # rule above is never "simplified" away. Send the EOF inside the
        # window with one data frame still missing, exactly as a naive
        # Go-Back-N sender would.
        data = b"a" * fb.FRAME_LENGTH + b"b" * fb.FRAME_LENGTH
        device = FakeWriteTarget()
        device.feed(fb.build_file_data(0, data[:fb.FRAME_LENGTH]))
        # frame 1 is lost on the air, and the naive sender sends EOF anyway
        device.feed(fb.build_file_data(2, b""))
        self.assertTrue(device.closed)
        self.assertEqual(len(device.file), fb.FRAME_LENGTH)  # half an image
        self.assertNotEqual(bytes(device.file), data)


# ---------------------------------------------------------------------------
# flysight.txt and the batch table
# ---------------------------------------------------------------------------

SAMPLE_TXT = b"""; FlySight - http://flysight.ca

; Firmware version

FUS_Ver:      1.2.0
Stack_Ver:    1.19.0
Firmware_Ver: v0.0.19-n.caf1a19

; Device information

Device_ID:    0123456789abcdef01234567
Device_Name:  FlySight

Pubkey_X:     486bee2d1ca903f3486bee2d1ca903f3486bee2d1ca903f3486bee2d1ca903f3
Pubkey_Y:     0dc18b7f3dcf346d0dc18b7f3dcf346d0dc18b7f3dcf346d0dc18b7f3dcf346d
"""


class TestFlysightTxt(unittest.TestCase):

    def test_fields_and_batch(self):
        fields = fb.parse_flysight_txt(SAMPLE_TXT)
        self.assertEqual(fields["firmware_ver"], "v0.0.19-n.caf1a19")
        self.assertEqual(fields["stack_ver"], "1.19.0")
        self.assertEqual(fields["fus_ver"], "1.2.0")
        self.assertEqual(fields["device_id"], "0123456789abcdef01234567")
        self.assertEqual(fields["device_name"], "FlySight")
        self.assertEqual(fb.batch_for_pubkey_x(fields["pubkey_x"]), "B2")

    def test_comments_and_padding_are_ignored(self):
        fields = fb.parse_flysight_txt(b"; not: a field\r\nStack_Ver:   1.19.0  \r\n")
        self.assertNotIn("not", fields)
        self.assertEqual(fields["stack_ver"], "1.19.0")

    def test_a_user_edited_file_still_yields_the_key(self):
        # The file is hand-editable on the USB drive; one non-UTF-8 byte in
        # Device_Name must not cost us Pubkey_X.
        raw = b"Device_Name:  Andr\xe9\nPubkey_X:     211d721d00\n"
        fields = fb.parse_flysight_txt(raw)
        self.assertEqual(fb.batch_for_pubkey_x(fields["pubkey_x"]), "B3")

    def test_unknown_or_missing_keys_are_no_batch(self):
        self.assertIsNone(fb.batch_for_pubkey_x(None))
        self.assertIsNone(fb.batch_for_pubkey_x("abc"))
        self.assertIsNone(fb.batch_for_pubkey_x("deadbeef" + "00" * 28))

    def test_batch_table_matches_the_deploy_keys(self):
        """The table is not folklore: re-derive it from the shipped keys.

        `Deploy/Public_Keys/pub_key_b*.bin` are 65-byte uncompressed points,
        `0x04 || X[32] || Y[32]`, so the batch prefix is bytes 1..5 of the file.
        """
        keys = os.path.join(REPO, "Deploy", "Public_Keys")
        derived = {}
        for name in sorted(os.listdir(keys)):
            if not name.startswith("pub_key_b") or not name.endswith(".bin"):
                continue
            with open(os.path.join(keys, name), "rb") as f:
                blob = f.read()
            self.assertEqual(len(blob), 65, name)
            self.assertEqual(blob[0], 0x04, name)
            derived[blob[1:5].hex()] = name[len("pub_key_"):-len(".bin")].upper()
        self.assertEqual(derived, fb.BATCHES)


class TestImageNaming(unittest.TestCase):

    def test_batch_from_filename(self):
        self.assertEqual(fb.batch_from_filename("B2_UserApp.sfb"), "B2")
        self.assertEqual(fb.batch_from_filename("B2_v2024.12.30.10.sfb"), "B2")
        self.assertEqual(fb.batch_from_filename("/a/b/UserApp_B6.sfb"), "B6")

    def test_a_file_that_names_no_batch_is_refused(self):
        self.assertIsNone(fb.batch_from_filename("APP.SFB"))
        self.assertIsNone(fb.batch_from_filename("firmware.sfb"))
        self.assertIsNone(fb.batch_from_filename("SUB2SET.sfb"))

    def test_an_ambiguous_name_is_refused(self):
        self.assertIsNone(fb.batch_from_filename("B2_and_B3.sfb"))

    def test_sha256sums_parsing(self):
        table = fb.parse_sha256sums(
            "%s  B2_UserApp.sfb\n%s *Deploy/B3_UserApp.sfb\ngarbage\n"
            % ("a" * 64, "b" * 64))
        self.assertEqual(table["B2_UserApp.sfb"], "a" * 64)
        self.assertEqual(table["B3_UserApp.sfb"], "b" * 64)

    def test_expected_digest_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            image = os.path.join(tmp, "B2_UserApp.sfb")
            with open(image, "wb") as f:
                f.write(b"SFU1data")
            digest = hashlib.sha256(b"SFU1data").hexdigest()

            self.assertEqual(fb.expected_digest_for(image), (None, None))
            self.assertEqual(fb.expected_digest_for(image, "ABC")[0], "abc")

            with open(os.path.join(tmp, "sha256sums.txt"), "w") as f:
                f.write(f"{digest}  B2_UserApp.sfb\n")
            self.assertEqual(fb.expected_digest_for(image),
                             (digest, "sha256sums.txt"))

            # A sums file that does not mention this image is a refusal, not a
            # shrug: an empty expected digest with a source set fails the gate.
            other = os.path.join(tmp, "B3_UserApp.sfb")
            with open(other, "wb") as f:
                f.write(b"SFU1data")
            self.assertEqual(fb.expected_digest_for(other), ("", "sha256sums.txt"))


# ---------------------------------------------------------------------------
# The gates
# ---------------------------------------------------------------------------

GOOD_IMAGE = b"SFU1" + b"\x00" * 4096
GOOD_DIGEST = hashlib.sha256(GOOD_IMAGE).hexdigest()
B2_PUBKEY = "486bee2d" + "0" * 56


def gates(**overrides):
    kwargs = dict(
        image=GOOD_IMAGE, filename="B2_UserApp.sfb",
        expected_sha256=GOOD_DIGEST, digest_source="sha256sums.txt",
        mode=fb.MODE_SLEEP, battery=87, mtu=250, pubkey_x=B2_PUBKEY)
    kwargs.update(overrides)
    return {g.name: g for g in fb.flash_gates(**kwargs)}


class TestFlashGates(unittest.TestCase):

    def test_a_good_flash_passes_every_gate(self):
        for name, gate in gates().items():
            self.assertTrue(gate.ok, f"{name}: {gate.detail}")
            self.assertFalse(gate.warn, name)

    def test_a_non_sfb_file_is_refused(self):
        self.assertFalse(gates(image=b"\x7fELF" + b"\x00" * 100)["SFU1 magic"].ok)

    def test_a_wrong_digest_is_refused(self):
        self.assertFalse(gates(expected_sha256="f" * 64)["sha256"].ok)

    def test_a_missing_digest_warns_but_does_not_refuse(self):
        gate = gates(expected_sha256=None, digest_source=None)["sha256"]
        self.assertTrue(gate.ok)
        self.assertTrue(gate.warn)
        self.assertIn("NOT checked", gate.detail)

    def test_a_sums_file_that_omits_the_image_is_refused(self):
        self.assertFalse(gates(expected_sha256="")["sha256"].ok)

    def test_the_wrong_batch_is_refused(self):
        self.assertFalse(gates(filename="B3_UserApp.sfb")["batch"].ok)
        self.assertFalse(gates(filename="APP.SFB")["batch"].ok)
        self.assertFalse(gates(pubkey_x="deadbeef" + "0" * 56)["batch"].ok)
        self.assertFalse(gates(pubkey_x="")["batch"].ok)

    def test_only_sleep_may_be_written_to(self):
        self.assertTrue(gates(mode=fb.MODE_SLEEP)["mode"].ok)
        for mode in (fb.MODE_ACTIVE, fb.MODE_CONFIG, fb.MODE_USB,
                     fb.MODE_PAIRING, fb.MODE_START, None):
            self.assertFalse(gates(mode=mode)["mode"].ok, mode)

    def test_a_zero_battery_is_unknown_rather_than_flat(self):
        gate = gates(battery=0)["battery"]
        self.assertFalse(gate.ok)
        self.assertIn("never measured", gate.detail)
        self.assertFalse(gates(battery=None)["battery"].ok)
        self.assertFalse(gates(battery=49)["battery"].ok)
        self.assertTrue(gates(battery=50)["battery"].ok)

    def test_a_small_mtu_is_refused(self):
        # 244-byte frames need ATT_MTU >= 247; 185 is a common phone default.
        self.assertFalse(gates(mtu=185)["ATT MTU"].ok)
        self.assertFalse(gates(mtu=None)["ATT MTU"].ok)
        self.assertTrue(gates(mtu=247)["ATT MTU"].ok)

    def test_every_gate_is_reported_even_when_one_fails(self):
        # The screen shows all of them at once: a user with a flat battery and
        # the wrong file should learn both in one go.
        result = fb.flash_gates(
            image=b"nope", filename="APP.SFB", expected_sha256=None,
            digest_source=None, mode=fb.MODE_ACTIVE, battery=0, mtu=23,
            pubkey_x="")
        self.assertEqual(len(result), 6)
        self.assertEqual(sum(1 for g in result if not g.ok), 5)  # sha256 warns


# ---------------------------------------------------------------------------
# The client loops, against a model of the whole device
# ---------------------------------------------------------------------------

class FakeFlySight:
    """`FS_CRS_State_Idle/_Read/_Write` (FlySight/crs.c:135-570), in Python.

    Packets in, packets out — no radio, no timing. `drop_out` and `drop_in`
    hold ordinals of packets to throw away in each direction, which is how the
    client's retransmission paths get exercised.
    """

    def __init__(self, files=None, drop_in=(), drop_out=()):
        self.files = dict(files or {})
        self.state = "idle"
        self.drop_in, self.drop_out = set(drop_in), set(drop_out)
        self.in_ordinal = self.out_ordinal = 0
        self.written = {}

    # --- plumbing ---------------------------------------------------------

    def handle(self, packet):
        """One inbound write; returns the packets the device would send."""
        ordinal, self.in_ordinal = self.in_ordinal, self.in_ordinal + 1
        if ordinal in self.drop_in:
            return []
        out = getattr(self, f"_state_{self.state}")(bytes(packet))
        kept = []
        for pkt in out:
            ordinal, self.out_ordinal = self.out_ordinal, self.out_ordinal + 1
            if ordinal not in self.drop_out:
                kept.append(pkt)
        return kept

    # --- states -----------------------------------------------------------

    def _state_idle(self, p):
        op = p[0]
        if op == fb.CRS_PING:
            return [bytes([fb.CRS_ACK, fb.CRS_PING])]
        if op == fb.CRS_READ:
            path = p[9:].split(b"\x00")[0].decode()
            if path not in self.files:
                return [bytes([fb.CRS_NAK, fb.CRS_READ])]
            self.data = self.files[path]
            self.next_packet = self.next_ack = 0
            self.last_packet = None
            self.state = "read"
            return [bytes([fb.CRS_ACK, fb.CRS_READ])] + self._read_pump()
        if op == fb.CRS_WRITE:
            self.path = p[1:].split(b"\x00")[0].decode()
            self.buffer = bytearray()
            self.next_packet = 0
            self.state = "write"
            return [bytes([fb.CRS_ACK, fb.CRS_WRITE])]
        if op == fb.CRS_MK_DIR:
            return [bytes([fb.CRS_ACK, fb.CRS_MK_DIR])]
        return [bytes([fb.CRS_NAK, op])]

    def _read_pump(self):
        out = []
        while (self.next_packet < self.next_ack + fb.WINDOW_LENGTH
               and (self.last_packet is None or self.next_packet < self.last_packet)):
            offset = self.next_packet * fb.FRAME_LENGTH
            header = bytes([fb.CRS_FILE_DATA, self.next_packet & 0xFF])
            if offset >= len(self.data):
                out.append(header)  # empty packet = end of file
                self.next_packet += 1
                self.last_packet = self.next_packet
            else:
                out.append(header + self.data[offset:offset + fb.FRAME_LENGTH])
                self.next_packet += 1
        return out

    def _state_read(self, p):
        if p[0] == fb.CRS_CANCEL:
            self.state = "idle"
            return []
        if p[0] == fb.CRS_FILE_ACK and len(p) >= 2:
            if p[1] == (self.next_ack & 0xFF):
                self.next_ack += 1
            else:
                # An ACK it is not waiting for means our earlier one was lost,
                # and by the time it arrives the device's own 200 ms timer has
                # long since rewound it to `next_ack` and re-seeked the file
                # (crs.c:426-436). Modelled here as an immediate rewind.
                self.next_packet = self.next_ack
                if self.last_packet is not None and self.next_packet < self.last_packet:
                    self.last_packet = None
            out = self._read_pump()
            if self.next_ack == self.last_packet:
                self.state = "idle"
            return out
        return []

    def _state_write(self, p):
        if p[0] == fb.CRS_CANCEL:
            self.state = "idle"
            return []
        out = []
        if p[0] == fb.CRS_FILE_DATA and len(p) >= 2:
            if p[1] == (self.next_packet & 0xFF):
                self.buffer += p[2:]
                out.append(fb.build_file_ack(p[1]))
                self.next_packet += 1
            if len(p) == 2:  # closes the file whatever the counter said
                self.files[self.path] = bytes(self.buffer)
                self.written[self.path] = bytes(self.buffer)
                self.state = "idle"
        return out


class FakeClient:
    """The slice of bleak's BleakClient the tool uses.

    Answers the Device State control point too, and remembers whether it was
    ever told to install — which is what the dry-run test asserts on.
    """

    def __init__(self, device, mode=fb.MODE_SLEEP, battery=91, mtu=250,
                 version="v0.0.19-n.caf1"):
        self.device = device
        self.is_connected = True
        self.mtu_size = mtu
        self.mode = mode
        self.battery = battery
        self.version = version
        self.install_requested = False
        self._notify = {}

    async def start_notify(self, uuid, callback):
        self._notify[uuid] = callback

    async def read_gatt_char(self, uuid):
        return bytearray([self.mode if uuid == fb.DS_MODE else self.battery])

    async def write_gatt_char(self, uuid, data, response=False):
        data = bytes(data)
        if uuid == fb.FT_PACKET_IN:
            for packet in self.device.handle(data):
                self._notify[fb.FT_PACKET_OUT](None, bytearray(packet))
            return
        if uuid != fb.DS_CONTROL_POINT:
            return
        opcode, payload = data[0], b""
        if opcode == fb.DS_GET_FW_VERSION:
            payload = self.version.encode()[:17]
        elif opcode == fb.DS_GET_DEVICE_ID:
            payload = b"0123456789abcdef01234567"[:17]
        elif opcode == fb.DS_INSTALL_UPLOADED_FIRMWARE:
            self.install_requested = True
        self._notify[fb.DS_CONTROL_POINT](
            None, bytearray(bytes([fb.CP_RESPONSE_ID, opcode, fb.CP_SUCCESS]) + payload))


class TestClientLoops(unittest.IsolatedAsyncioTestCase):

    def setUp(self):
        # The real timeouts are seconds; the loops are the same at 20 ms.
        self._saved = (fb.COMMAND_TIMEOUT, fb.PACKET_TIMEOUT, fb.ACK_TIMEOUT)
        fb.COMMAND_TIMEOUT = fb.PACKET_TIMEOUT = fb.ACK_TIMEOUT = 0.02

    def tearDown(self):
        fb.COMMAND_TIMEOUT, fb.PACKET_TIMEOUT, fb.ACK_TIMEOUT = self._saved

    async def _link(self, device):
        client = FakeClient(device)
        fs = fb.FlySight(client)
        await fs.start()
        return fs

    async def test_round_trip_over_a_clean_link(self):
        data = os.urandom(fb.FRAME_LENGTH * 5 + 31)
        device = FakeFlySight()
        fs = await self._link(device)
        await fs.write_file("/TEST.BIN", data)
        self.assertEqual(device.written["/TEST.BIN"], data)
        self.assertEqual(await fs.read_file("/TEST.BIN"), data)

    async def test_an_empty_file_round_trips(self):
        device = FakeFlySight()
        fs = await self._link(device)
        await fs.write_file("/EMPTY.BIN", b"")
        self.assertEqual(device.written["/EMPTY.BIN"], b"")
        self.assertEqual(await fs.read_file("/EMPTY.BIN"), b"")

    async def test_upload_recovers_from_lost_frames(self):
        data = os.urandom(fb.FRAME_LENGTH * 12)
        # Ordinal 0 is the write command itself; 1.. are the data frames.
        device = FakeFlySight(drop_in=[3, 4, 5, 11])
        fs = await self._link(device)
        await fs.write_file("/LOSSY.BIN", data)
        self.assertEqual(device.written["/LOSSY.BIN"], data)

    async def test_upload_recovers_from_lost_acks(self):
        data = os.urandom(fb.FRAME_LENGTH * 6)
        device = FakeFlySight(drop_out=[2, 3])
        fs = await self._link(device)
        await fs.write_file("/LOSSY2.BIN", data)
        self.assertEqual(device.written["/LOSSY2.BIN"], data)

    async def test_download_recovers_from_lost_data_frames(self):
        data = os.urandom(fb.FRAME_LENGTH * 6 + 5)
        device = FakeFlySight(files={"/T.BIN": data}, drop_out=[2, 3, 9])
        fs = await self._link(device)
        self.assertEqual(await fs.read_file("/T.BIN"), data)

    async def test_a_missing_file_is_a_clear_refusal(self):
        device = FakeFlySight()
        fs = await self._link(device)
        with self.assertRaises(fb.CrsError) as caught:
            await fs.read_file("/NOPE.BIN")
        self.assertIn("refused", str(caught.exception))

    async def test_the_device_is_left_idle_after_a_failure(self):
        # A transfer the device is still in keeps its SD card claimed, so a
        # failed operation must send the cancel that drops it out.
        device = FakeFlySight()
        fs = await self._link(device)
        with self.assertRaises(fb.CrsError):
            await fs.read_file("/NOPE.BIN")
        self.assertEqual(device.state, "idle")
        await fs.write_file("/AFTER.BIN", b"still works")
        self.assertEqual(device.written["/AFTER.BIN"], b"still works")

    async def test_flysight_txt_is_read_and_parsed(self):
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        fs = await self._link(device)
        fields = fb.parse_flysight_txt(await fs.read_file(fb.FLYSIGHT_TXT))
        self.assertEqual(fb.batch_for_pubkey_x(fields["pubkey_x"]), "B2")
        self.assertEqual(fields["firmware_ver"], "v0.0.19-n.caf1a19")


# ---------------------------------------------------------------------------
# Finding a device, and connecting to one
# ---------------------------------------------------------------------------

class FakeAdv:
    def __init__(self, local_name=None, rssi=-60):
        self.local_name = local_name
        self.rssi = rssi


class FakeBleDevice:
    def __init__(self, address, name=None):
        self.address = address
        self.name = name


def scan_result(*entries):
    """`{address: (device, adv)}`, the shape BleakScanner.discover returns."""
    return {d.address: (d, a) for d, a in entries}


class TestSelectDevice(unittest.TestCase):

    def test_only_the_exact_advertised_name_counts(self):
        found = scan_result(
            (FakeBleDevice("AAAA"), FakeAdv("FlySight", -50)),
            (FakeBleDevice("BBBB"), FakeAdv("FlySight Viewer", -40)),
            (FakeBleDevice("CCCC"), FakeAdv("ENGO 3 000000", -30)))
        device, name, rssi = fb.select_device(found)
        self.assertEqual(device.address, "AAAA")
        self.assertEqual(name, "FlySight")

    def test_the_strongest_of_several_wins(self):
        found = scan_result(
            (FakeBleDevice("AAAA"), FakeAdv("FlySight", -80)),
            (FakeBleDevice("BBBB"), FakeAdv("FlySight", -42)),
            (FakeBleDevice("CCCC"), FakeAdv("FlySight", -70)))
        self.assertEqual(fb.select_device(found)[0].address, "BBBB")

    def test_an_address_beats_the_name_a_device_renamed_itself_to(self):
        # Device_Name is a config field; a renamed FlySight still answers to
        # its CoreBluetooth UUID, and pinning to one is the point of --address.
        found = scan_result((FakeBleDevice("AAAA"), FakeAdv("Bob's logger", -60)))
        self.assertEqual(fb.select_device(found, address="aaaa")[0].address, "AAAA")

    def test_an_address_that_is_not_there_finds_nothing(self):
        found = scan_result((FakeBleDevice("AAAA"), FakeAdv("FlySight", -60)))
        self.assertIsNone(fb.select_device(found, address="ZZZZ"))

    def test_an_empty_scan_finds_nothing(self):
        self.assertIsNone(fb.select_device({}))

    def test_the_name_falls_back_to_the_devices_own(self):
        # CoreBluetooth caches a name and sometimes gives no local_name in the
        # advertisement at all.
        found = scan_result((FakeBleDevice("AAAA", "FlySight"), FakeAdv(None, -60)))
        self.assertEqual(fb.select_device(found)[1], "FlySight")

    def test_a_missing_rssi_does_not_crash_the_comparison(self):
        found = scan_result(
            (FakeBleDevice("AAAA"), FakeAdv("FlySight", None)),
            (FakeBleDevice("BBBB"), FakeAdv("FlySight", -90)))
        self.assertEqual(fb.select_device(found)[0].address, "BBBB")


class TestConnectDevice(unittest.IsolatedAsyncioTestCase):
    """The one rule: a connect that does not succeed is always cancelled.

    macOS keeps a failed attempt pending long after our timeout has given up;
    the FlySight then counts itself linked and stops advertising to everyone,
    the owner's tablet included. It reads exactly like a dead device.
    """

    def install(self, *, fail_connect=False, fail_start=False, slow=False):
        made = []

        class Client(FakeClient):
            def __init__(self, device, timeout=None, disconnected_callback=None):
                super().__init__(FakeFlySight())
                self.disconnect_calls = 0
                self.callback = disconnected_callback
                made.append(self)

            async def connect(self):
                if slow:
                    await asyncio.sleep(30)
                if fail_connect:
                    raise RuntimeError("peripheral is not connectable")

            async def start_notify(self, uuid, callback):
                if fail_start:
                    raise RuntimeError("characteristic not found")
                await super().start_notify(uuid, callback)

            async def disconnect(self):
                self.disconnect_calls += 1

        saved = fb.BleakClient
        fb.BleakClient = Client
        self.addCleanup(lambda: setattr(fb, "BleakClient", saved))
        return made

    async def test_a_good_connect_hands_back_a_subscribed_link(self):
        made = self.install()
        client, fs = await fb.connect_device(FakeBleDevice("AAAA"))
        self.assertIs(client, made[0])
        self.assertEqual(client.disconnect_calls, 0)
        self.assertIn(fb.FT_PACKET_OUT, client._notify)
        self.assertEqual(fs.mode, fb.MODE_SLEEP)

    async def test_a_failed_connect_is_cancelled(self):
        made = self.install(fail_connect=True)
        with self.assertRaises(RuntimeError):
            await fb.connect_device(FakeBleDevice("AAAA"))
        self.assertEqual(made[0].disconnect_calls, 1)

    async def test_a_failure_while_subscribing_is_cancelled_too(self):
        # The link is up by then, and leaving it up would hold the FlySight
        # silent just as surely as a half-open attempt.
        made = self.install(fail_start=True)
        with self.assertRaises(RuntimeError):
            await fb.connect_device(FakeBleDevice("AAAA"))
        self.assertEqual(made[0].disconnect_calls, 1)

    async def test_a_cancelled_connect_is_cancelled_on_the_device_too(self):
        # A caller giving up (a timeout, or the app being told to stop) leaves
        # exactly the same pending attempt behind as a failure does — which is
        # why the guard catches BaseException and not Exception.
        made = self.install(slow=True)
        task = asyncio.ensure_future(fb.connect_device(FakeBleDevice("AAAA")))
        await asyncio.sleep(0)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertEqual(made[0].disconnect_calls, 1)

    async def test_the_disconnect_callback_is_handed_to_bleak(self):
        made = self.install()
        marker = object()
        await fb.connect_device(FakeBleDevice("AAAA"), disconnected_callback=marker)
        self.assertIs(made[0].callback, marker)


class TestLogSink(unittest.TestCase):

    def test_lines_reach_an_installed_sink(self):
        lines = []
        saved, fb.LOG_SINK = fb.LOG_SINK, lines.append
        try:
            fb.log("out-of-order frame 7")
        finally:
            fb.LOG_SINK = saved
        self.assertEqual(lines, ["out-of-order frame 7"])

    def test_a_broken_sink_never_breaks_a_transfer(self):
        def explode(_message):
            raise RuntimeError("the page went away")

        saved, fb.LOG_SINK = fb.LOG_SINK, explode
        try:
            fb.log("still fine")  # must not raise
        finally:
            fb.LOG_SINK = saved


# ---------------------------------------------------------------------------
# The subcommands, end to end
# ---------------------------------------------------------------------------

class TestCommands(unittest.IsolatedAsyncioTestCase):
    """Every command driven start to finish against the modelled device.

    The point of these is the one assertion hardware cannot make cheaply:
    that a dry run writes the image and never sends the install opcode.
    """

    def setUp(self):
        self._saved = (fb.COMMAND_TIMEOUT, fb.PACKET_TIMEOUT, fb.ACK_TIMEOUT)
        fb.COMMAND_TIMEOUT = fb.PACKET_TIMEOUT = fb.ACK_TIMEOUT = 0.02
        self._connected = fb.connected
        self.tmp = tempfile.TemporaryDirectory()
        self.image = b"SFU1" + os.urandom(4000)
        self.sfb = os.path.join(self.tmp.name, "B2_UserApp.sfb")
        with open(self.sfb, "wb") as f:
            f.write(self.image)

    def tearDown(self):
        fb.COMMAND_TIMEOUT, fb.PACKET_TIMEOUT, fb.ACK_TIMEOUT = self._saved
        fb.connected = self._connected
        self.tmp.cleanup()

    def install(self, client):
        """Point the tool's `connected(args)` at the fake, and return output."""
        class _Fake:
            def __init__(self, args):
                pass

            async def __aenter__(self):
                fs = fb.FlySight(client)
                await fs.start()
                return fs

            async def __aexit__(self, *exc):
                return False

        fb.connected = _Fake

    async def run_command(self, command, argv, client):
        self.install(client)
        import contextlib
        import io
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = await command(fb.build_parser().parse_args(argv))
        return code, out.getvalue()

    async def test_info_reports_the_batch_and_the_full_version(self):
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        code, out = await self.run_command(
            fb.cmd_info, ["--info"], FakeClient(device))
        self.assertEqual(code, 0)
        self.assertIn("batch B2", out)
        self.assertIn("v0.0.19-n.caf1a19", out)   # from the file
        self.assertIn("1.19.0", out)              # Stack_Ver
        self.assertIn("250", out)                 # ATT MTU
        self.assertIn("SLEEP", out)

    async def test_info_outside_sleep_reads_no_files_and_changes_no_mode(self):
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        client = FakeClient(device, mode=fb.MODE_ACTIVE)
        code, out = await self.run_command(fb.cmd_info, ["--info"], client)
        self.assertEqual(code, 0)
        self.assertIn("was NOT read", out)
        self.assertIn("ACTIVE", out)
        self.assertEqual(device.in_ordinal, 0)  # not one packet was written
        self.assertFalse(client.install_requested)

    async def test_info_flags_a_battery_that_was_never_measured(self):
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        _, out = await self.run_command(
            fb.cmd_info, ["--info"], FakeClient(device, battery=0))
        self.assertIn("never measured", out)

    async def test_upload_round_trips_and_says_so(self):
        payload = os.urandom(fb.FRAME_LENGTH * 4 + 9)
        path = os.path.join(self.tmp.name, "payload.bin")
        with open(path, "wb") as f:
            f.write(payload)
        device = FakeFlySight()
        code, out = await self.run_command(
            fb.cmd_upload, ["--upload", path, "--dest", "/TEST.BIN"],
            FakeClient(device))
        self.assertEqual(code, 0)
        self.assertIn("Round trip OK", out)
        self.assertEqual(device.written["/TEST.BIN"], payload)

    async def test_upload_refuses_outside_sleep_and_writes_nothing(self):
        path = os.path.join(self.tmp.name, "payload.bin")
        with open(path, "wb") as f:
            f.write(b"whatever")
        device = FakeFlySight()
        code, out = await self.run_command(
            fb.cmd_upload, ["--upload", path, "--dest", "/TEST.BIN"],
            FakeClient(device, mode=fb.MODE_START))
        self.assertEqual(code, 2)
        self.assertIn("Nothing was written", out)
        self.assertEqual(device.in_ordinal, 0)

    async def test_flash_dry_run_uploads_and_verifies_but_never_installs(self):
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        client = FakeClient(device)
        code, out = await self.run_command(
            fb.cmd_flash, ["--flash", self.sfb], client)
        self.assertEqual(code, 0)
        self.assertIn("DRY RUN", out)
        self.assertEqual(device.written[fb.FIRMWARE_PATH], self.image)
        self.assertIn("On-device image verified", out)
        self.assertFalse(client.install_requested)

    async def test_flash_installs_only_when_asked_in_words(self):
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        client = FakeClient(device)
        code, out = await self.run_command(
            fb.cmd_flash, ["--flash", self.sfb, "--really-install"], client)
        self.assertEqual(code, 0)
        self.assertTrue(client.install_requested)
        self.assertIn("rebooting to install", out)

    async def test_flash_refuses_a_flat_or_unmeasured_battery_before_writing(self):
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        client = FakeClient(device, battery=0)
        code, out = await self.run_command(
            fb.cmd_flash, ["--flash", self.sfb, "--really-install"], client)
        self.assertEqual(code, 2)
        self.assertIn("REFUSED", out)
        self.assertNotIn(fb.FIRMWARE_PATH, device.written)
        self.assertFalse(client.install_requested)

    async def test_flash_refuses_an_image_for_another_batch(self):
        wrong = os.path.join(self.tmp.name, "B5_UserApp.sfb")
        with open(wrong, "wb") as f:
            f.write(self.image)
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        client = FakeClient(device)
        code, out = await self.run_command(fb.cmd_flash, ["--flash", wrong], client)
        self.assertEqual(code, 2)
        self.assertIn("file says B5, device says B2", out)
        self.assertNotIn(fb.FIRMWARE_PATH, device.written)

    async def test_flash_refuses_when_the_device_is_awake(self):
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        client = FakeClient(device, mode=fb.MODE_ACTIVE)
        code, out = await self.run_command(fb.cmd_flash, ["--flash", self.sfb], client)
        self.assertEqual(code, 2)
        self.assertEqual(device.in_ordinal, 0)  # /flysight.txt not even read
        self.assertIn("ACTIVE", out)

    async def test_flash_honours_a_sha256sums_file(self):
        with open(os.path.join(self.tmp.name, "sha256sums.txt"), "w") as f:
            f.write("%s  B2_UserApp.sfb\n" % ("0" * 64))
        device = FakeFlySight(files={fb.FLYSIGHT_TXT: SAMPLE_TXT})
        client = FakeClient(device)
        code, out = await self.run_command(fb.cmd_flash, ["--flash", self.sfb], client)
        self.assertEqual(code, 2)
        self.assertIn("does not match", out)
        self.assertNotIn(fb.FIRMWARE_PATH, device.written)


# ---------------------------------------------------------------------------
# CLI argument handling
# ---------------------------------------------------------------------------

class TestCli(unittest.TestCase):

    def test_upload_requires_a_destination(self):
        self.assertEqual(fb.main(["--upload", "/dev/null"]), 2)

    def test_flash_only_accepts_an_sfb(self):
        self.assertEqual(fb.main(["--flash", "UserApp.bin"]), 2)

    def test_the_window_cannot_exceed_the_devices_ring(self):
        self.assertEqual(fb.main(["--info", "--window", "9"]), 2)

    def test_a_command_is_required(self):
        with self.assertRaises(SystemExit):
            fb.build_parser().parse_args([])


if __name__ == "__main__":
    unittest.main(verbosity=2)
