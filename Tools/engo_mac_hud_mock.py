#!/usr/bin/env python3
"""Mac-side mock of the FlySight ENGO HUD screen (activelook_mode0.c v0.0.10).

Reproduces the exact frame the firmware draws each tick, with zero-filled
placeholder values: header font0 @ (296,240), data rows font2, labels x=296,
values x=150, rows y=196/154/112/70. Frame = holdFlush(HOLD), clear, texts,
holdFlush(FLUSH) — same order as Mode0_Update().
"""
import asyncio
import sys

from bleak import BleakClient, BleakScanner

RX = "0783b03e-8535-b5a0-7140-a304d2495cba"
TX = "0783b03e-8535-b5a0-7140-a304d2495cb8"
CTRL = "0783b03e-8535-b5a0-7140-a304d2495cb9"
NAME_PREFIXES = ("engo", "al-", "al ", "activelook", "a.look")

# Layout v3: header f0 (24px); speeds side by side in ONE row f3 (64px);
# GR and Alt each their own row f3. Small f0 labels + big f3 values.
# X mirrored (296 = viewer left); glyphs draw downward from anchor y.
HEADER = "X A:00% F:00% N:00 v0.0.10"
DRAW = [  # (x, y, font, text) -- labels removed, values only
    (296, 232, 0, HEADER),
    # speeds row: two values side by side (font3, 64px)  -- raised +3px
    (268, 208, 3, "000"),
    (134, 208, 3, "000"),
    # glide ratio value (font4, 75px)
    (250, 147, 4, "0.00"),
    # altitude value (font4, 75px)
    (250, 73, 4, "0000"),
]


def frame(cmd: int, data: bytes = b"") -> bytes:
    total = len(data) + 5
    assert total <= 255
    return bytes([0xFF, cmd, 0x00, total]) + data + bytes([0xAA])


def txt(x: int, y: int, font: int, s: str) -> bytes:
    payload = (
        x.to_bytes(2, "big") + y.to_bytes(2, "big")
        + bytes([4, font, 15]) + s.encode("ascii") + b"\x00"
    )
    return frame(0x37, payload)


async def main() -> int:
    print("Scanning 10s for ActiveLook glasses...", flush=True)
    devices = await BleakScanner.discover(timeout=10.0, return_adv=True)
    target = None
    for dev, adv in devices.values():
        name = (adv.local_name or dev.name or "")
        if name.lower().startswith(NAME_PREFIXES):
            target = dev
            print(f"FOUND: {name!r} rssi={adv.rssi}")
            break
    if target is None:
        print(f"NOT FOUND ({len(devices)} adv seen)")
        return 1

    flow = {"byte": 0x01}

    def on_ctrl(_, data: bytearray):
        flow["byte"] = data[0] if data else 0
        print(f"  [CTRL] {data.hex()}")

    async with BleakClient(target, timeout=20.0) as client:
        print(f"Connected. MTU={client.mtu_size}")
        await client.start_notify(CTRL, on_ctrl)

        async def send(f: bytes):
            # firmware/glidex rule: pause ONLY on 0x02 STOP, anything else = go
            for _ in range(60):
                if flow["byte"] != 0x02:
                    break
                await asyncio.sleep(0.1)
            await client.write_gatt_char(RX, f, response=False)

        await send(frame(0x39, b"\x00"))          # holdFlush HOLD
        await send(frame(0x01))                   # clear
        for x, y, font, s in DRAW:
            await send(txt(x, y, font, s))
        await send(frame(0x39, b"\x01"))          # holdFlush FLUSH
        print("HUD mock sent, holding link 3s...")
        await asyncio.sleep(3.0)
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
