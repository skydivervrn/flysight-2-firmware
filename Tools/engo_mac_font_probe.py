#!/usr/bin/env python3
"""Font probe for ENGO 3: query fontList (0x50) + vers (0x06), then render
one sample line per font id 0..4 so the user can see which actually work.
"""
import asyncio
import sys

from bleak import BleakClient, BleakScanner

RX = "0783b03e-8535-b5a0-7140-a304d2495cba"
TX = "0783b03e-8535-b5a0-7140-a304d2495cb8"
CTRL = "0783b03e-8535-b5a0-7140-a304d2495cb9"
NAME_PREFIXES = ("engo", "al-", "al ", "activelook", "a.look")


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


def parse_reply(data: bytes):
    # reply frame: 0xFF cmd fmt len [data] 0xAA
    if len(data) >= 5 and data[0] == 0xFF and data[-1] == 0xAA:
        return data[1], data[4:-1]
    return None, data


async def main() -> int:
    print("Scanning 10s...", flush=True)
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

    def on_tx(_, data: bytearray):
        cmd, payload = parse_reply(bytes(data))
        if cmd == 0x50:
            pairs = [(payload[i], payload[i + 1]) for i in range(0, len(payload) - 1, 2)]
            print(f"  [FONTLIST] {pairs}  (id, height_px)")
        elif cmd == 0x06:
            print(f"  [VERS] {payload.hex()}")
        else:
            print(f"  [TX] cmd=0x{cmd:02x} data={payload.hex() if payload else ''}")

    async with BleakClient(target, timeout=20.0) as client:
        print(f"Connected. MTU={client.mtu_size}")
        await client.start_notify(CTRL, on_ctrl)
        await client.start_notify(TX, on_tx)

        async def send(f: bytes):
            for _ in range(60):
                if flow["byte"] != 0x02:
                    break
                await asyncio.sleep(0.1)
            await client.write_gatt_char(RX, f, response=False)

        await send(frame(0x06))   # vers
        await send(frame(0x50))   # fontList
        await asyncio.sleep(1.0)  # let replies arrive

        await send(frame(0x39, b"\x00"))
        await send(frame(0x01))
        # one sample per font id; y spaced for up to 49px glyphs
        samples = [(0, 246), (1, 196), (2, 146), (3, 90), (4, 40)]
        for fid, y in samples:
            await send(txt(296, y, fid, f"F{fid} 0123"))
        await send(frame(0x39, b"\x01"))
        print("Samples sent, holding 3s...")
        await asyncio.sleep(3.0)
    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
