#!/usr/bin/env python3
"""Mac-side ENGO 3 (ActiveLook) BLE test.

Scans for the glasses, connects, and direct-draws a test screen using the
same verified frame format as the firmware (activelook_proto.c):
  0xFF [cmd] [0x00] [total_len] [data...] 0xAA
Verified display facts (CLAUDE.md): rotation=4, fonts 0/1/2 only, X mirrored
(x=296 = viewer left, y=240 = top), clear=0x01, txt=0x37, holdFlush=0x39.
"""
import asyncio
import sys

from bleak import BleakClient, BleakScanner

SVC = "0783b03e-8535-b5a0-7140-a304d2495cb7"
RX = "0783b03e-8535-b5a0-7140-a304d2495cba"   # write commands here
TX = "0783b03e-8535-b5a0-7140-a304d2495cb8"   # notify (cmd answers)
CTRL = "0783b03e-8535-b5a0-7140-a304d2495cb9"  # flow control: 0x01 ok / 0x02 stop

NAME_PREFIXES = ("engo", "al-", "al ", "activelook", "a.look")


def frame(cmd: int, data: bytes = b"") -> bytes:
    total = len(data) + 5
    assert total <= 255
    return bytes([0xFF, cmd, 0x00, total]) + data + bytes([0xAA])


def txt(x: int, y: int, rotation: int, font: int, color: int, s: str) -> bytes:
    payload = (
        x.to_bytes(2, "big") + y.to_bytes(2, "big")
        + bytes([rotation, font, color]) + s.encode("ascii") + b"\x00"
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
            print(f"FOUND: {name!r} addr={dev.address} rssi={adv.rssi}")
            break
    if target is None:
        seen = sorted({(a.local_name or d.name) for d, a in devices.values() if (a.local_name or d.name)})
        print(f"NOT FOUND. Total adv seen: {len(devices)}. Named devices: {seen}")
        return 1

    flow = {"byte": 0x01}

    def on_ctrl(_, data: bytearray):
        flow["byte"] = data[0] if data else 0
        print(f"  [CTRL] {data.hex()}")

    def on_tx(_, data: bytearray):
        print(f"  [TX] {data.hex()}")

    async with BleakClient(target, timeout=20.0) as client:
        print(f"Connected. MTU={client.mtu_size}")
        await client.start_notify(CTRL, on_ctrl)
        try:
            await client.start_notify(TX, on_tx)
        except Exception as e:
            print(f"  (TX notify unavailable: {e})")

        async def send(f: bytes):
            # honor flow control like the firmware does
            for _ in range(60):
                if flow["byte"] == 0x01:
                    break
                await asyncio.sleep(0.1)
            await client.write_gatt_char(RX, f, response=False)

        await send(frame(0x04))            # vers query -> answer on TX if alive
        await asyncio.sleep(0.5)

        await send(frame(0x39, b"\x00"))   # holdFlush HOLD
        await send(frame(0x01))            # clear
        await send(txt(296, 240, 4, 1, 15, "FLYSIGHT MAC TEST"))
        await send(txt(296, 190, 4, 2, 15, "HELLO 123"))
        await send(txt(296, 140, 4, 2, 15, "BLE OK"))
        await send(txt(296, 70, 4, 0, 15, "font0: -12.3 m/s"))
        await send(frame(0x39, b"\x01"))   # holdFlush FLUSH
        print("Test screen sent, holding link 3s...")
        await asyncio.sleep(3.0)
    print("Disconnected. Image should remain on the glasses.")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
