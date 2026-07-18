#!/usr/bin/env python3
"""Replay the best-known POP4 laser/projector arm candidate over UVC XU.

Working candidate (2026-07-18, native Ubuntu, scored via Y16 nonzero count):

    CS=10 SET_CUR 01000000          # sticky enable flag (GET_CUR reads back)
    CS=7  SET_CUR <60-byte pop4>    # fire-and-forget; GET_CUR stays zero

pop4 payload (LE int32 × 5, zero-padded to 60):

    gain=1, exposure=800, irLum=130, crossLaserLight=780, singleLaserLight=706

Depth Y16 640x512 nonzero/frame jumps from ~35 (projector off) to thousands when
this sequence takes effect. Effect can persist after CS10/CS7 read back as zero —
USB re-enumeration clears it. CS7 alone has also armed in some states; prefer
CS10-then-CS7. Avoid CS=5 2-byte and casual CS=4 writes (disconnect risk).

Usage:
    .venv/bin/python tools/re_probe/xu_arm_replay.py
    .venv/bin/python tools/re_probe/xu_arm_replay.py --score
    .venv/bin/python tools/re_probe/xu_arm_replay.py --gain 1 --exposure 800 \\
        --ir-lum 130 --cross 780 --single 706
"""
from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from xu_probe import XuSession  # noqa: E402

import usb.core  # noqa: E402

DEPTH_DEV = "/dev/video0"
WIDTH, HEIGHT = 640, 512


def wait_for_device(timeout: float = 30.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if usb.core.find(idVendor=0x2207, idProduct=0x110C) is not None:
            if Path(DEPTH_DEV).exists():
                time.sleep(0.5)
                return
        time.sleep(0.25)
    raise RuntimeError("POP4 not found after wait")


def score_depth(frames: int = 2) -> int:
    out = Path("/tmp/pop4_arm_replay_score.raw")
    if out.exists():
        out.unlink()
    subprocess.run(
        [
            "gst-launch-1.0", "-q",
            "v4l2src", f"device={DEPTH_DEV}", f"num-buffers={frames}", "!",
            f"video/x-raw,format=GRAY16_LE,width={WIDTH},height={HEIGHT}", "!",
            "filesink", f"location={out}",
        ],
        check=True,
        capture_output=True,
        timeout=25,
    )
    data = out.read_bytes()
    n = len(data) // 2
    vals = struct.unpack(f"<{n}H", data[: n * 2])
    nz = sum(1 for v in vals if v)
    return nz // max(1, n // (WIDTH * HEIGHT))


def pop4_payload(gain: int, exposure: int, ir_lum: int, cross: int, single: int) -> bytes:
    return struct.pack("<5i", gain, exposure, ir_lum, cross, single) + bytes(40)


def arm(gain: int, exposure: int, ir_lum: int, cross: int, single: int, settle: float) -> None:
    payload = pop4_payload(gain, exposure, ir_lum, cross, single)
    with XuSession(0) as xu:
        xu.set_cur(10, bytes.fromhex("01000000"))
        time.sleep(settle)
        cs10 = xu.get_cur(10, 4)
        xu.set_cur(7, payload)
        time.sleep(settle)
        cs7 = xu.get_cur(7, 60)
    print(f"SET CS=10 -> 01000000  readback={cs10.hex()} sticky={cs10 == bytes.fromhex('01000000')}")
    print(f"SET CS=7  -> {payload.hex()}")
    print(f"     CS=7 readback head={cs7[:20].hex()} (expected zeros; write is fire-and-forget)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gain", type=int, default=1)
    ap.add_argument("--exposure", type=int, default=800)
    ap.add_argument("--ir-lum", type=int, default=130)
    ap.add_argument("--cross", type=int, default=780)
    ap.add_argument("--single", type=int, default=706)
    ap.add_argument("--settle", type=float, default=0.15)
    ap.add_argument("--score", action="store_true", help="gstreamer Y16 nonzero score before/after")
    ap.add_argument("--cs7-only", action="store_true", help="skip CS10 (less reliable)")
    args = ap.parse_args()

    wait_for_device()
    before = None
    if args.score:
        before = score_depth()
        print(f"BEFORE nonzero/frame ≈ {before}")

    if args.cs7_only:
        payload = pop4_payload(args.gain, args.exposure, args.ir_lum, args.cross, args.single)
        with XuSession(0) as xu:
            xu.set_cur(7, payload)
            time.sleep(args.settle)
        print(f"SET CS=7 -> {payload.hex()}")
    else:
        arm(args.gain, args.exposure, args.ir_lum, args.cross, args.single, args.settle)

    wait_for_device()
    time.sleep(0.8)
    if args.score:
        after = score_depth()
        print(f"AFTER  nonzero/frame ≈ {after}  (Δ{after - (before or 0):+d})")
        if after >= 500:
            print("LOOKS ARMED (score >> ~35 baseline)")
        else:
            print("still near baseline — try again after USB re-plug, or check scene/IR")
    return 0


if __name__ == "__main__":
    sys.exit(main())
