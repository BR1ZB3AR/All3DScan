#!/usr/bin/env python3
"""Systematically probe POP4 UVC XU SET sequences for laser/projector arming.

Scores success by Y16 depth nonzero pixel count (baseline ~35 at 640x512 with
projector off). Uses gstreamer for scoring so OpenCV is optional; falls back
to depth_capture if cv2 is available.

Detaches uvcvideo only for the SET burst, then reattaches before scoring so
/dev/video0 stays usable. Brief full-device disconnect is possible — waits for
recovery.

Usage:
    .venv/bin/python tools/re_probe/xu_arm_probe.py
    .venv/bin/python tools/re_probe/xu_arm_probe.py --only cs5_enable,cs10_plus_cs5
"""
from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import time
from pathlib import Path

import usb.core
import usb.util

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from xu_probe import XuSession, UNIT_ID  # noqa: E402

DEPTH_DEV = "/dev/video0"
WIDTH, HEIGHT = 640, 512
BASELINE_EXPECT = 80  # generous; hit if >> this (e.g. thousands)
HIT_THRESHOLD = 500


def wait_for_device(timeout: float = 30.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if usb.core.find(idVendor=0x2207, idProduct=0x110C) is not None:
            if Path(DEPTH_DEV).exists():
                # give uvcvideo a moment after re-enumeration
                time.sleep(0.3)
                return
        time.sleep(0.25)
    raise RuntimeError("POP4 did not reappear after USB re-enumeration")


def ascii60(cmd: str) -> bytes:
    raw = cmd.encode("ascii")
    if len(raw) > 60:
        raise ValueError(f"cmd too long ({len(raw)}): {cmd!r}")
    return raw + bytes(60 - len(raw))


def score_gstreamer(frames: int = 2) -> int:
    """Capture raw Y16 via gstreamer; return mean nonzero count per frame."""
    out = Path("/tmp/pop4_arm_score.raw")
    if out.exists():
        out.unlink()
    cmd = [
        "gst-launch-1.0", "-q",
        "v4l2src", f"device={DEPTH_DEV}", f"num-buffers={frames}", "!",
        f"video/x-raw,format=GRAY16_LE,width={WIDTH},height={HEIGHT}", "!",
        "filesink", f"location={out}",
    ]
    subprocess.run(cmd, check=True, capture_output=True, timeout=20)
    data = out.read_bytes()
    n = len(data) // 2
    if n == 0:
        raise RuntimeError("empty depth capture")
    vals = struct.unpack(f"<{n}H", data[: n * 2])
    nz = sum(1 for v in vals if v)
    # gstreamer may concatenate frames; normalize to per-frame
    frame_px = WIDTH * HEIGHT
    nframes = max(1, n // frame_px)
    return nz // nframes


def score_opencv(frames: int = 2) -> int:
    from depth_capture import open_depth, nonzero_score

    cap = open_depth(DEPTH_DEV, (WIDTH, HEIGHT))
    try:
        scores = [nonzero_score(cap) for _ in range(frames)]
    finally:
        cap.release()
    return int(sum(scores) / len(scores))


def score_depth(prefer: str = "gst") -> int:
    wait_for_device()
    time.sleep(0.4)
    if prefer == "cv2":
        return score_opencv()
    try:
        return score_gstreamer()
    except Exception as exc:
        print(f"  gstreamer score failed ({exc}); trying opencv", file=sys.stderr)
        return score_opencv()


def pack_i32(*vals: int) -> bytes:
    return b"".join(struct.pack("<i", int(v)) for v in vals)


def pack_u32(*vals: int) -> bytes:
    return b"".join(struct.pack("<I", int(v)) for v in vals)


def pack_u16(*vals: int) -> bytes:
    return b"".join(struct.pack("<H", int(v)) for v in vals)


def pad60(data: bytes) -> bytes:
    if len(data) > 60:
        raise ValueError(f"payload {len(data)} > 60")
    return data + bytes(60 - len(data))


def pop4_step_payload(
    gain: int = 1,
    exposure: int = 800,
    ir_lum: int = 130,
    cross: int = 780,
    single: int = 706,
    layout: str = "a",
) -> bytes:
    """60-byte candidates modeled on pop4.json step fields."""
    if layout == "a":
        # gain, exposure, irLum, cross, single as int32
        return pad60(pack_i32(gain, exposure, ir_lum, cross, single))
    if layout == "b":
        # leading mode/enable dword then params
        return pad60(pack_i32(1, gain, exposure, ir_lum, cross, single))
    if layout == "c":
        # u16 fields
        return pad60(pack_u16(gain, exposure, ir_lum, cross, single))
    if layout == "d":
        # exposure at front (mirrors CS3 style) + laser ints
        return pad60(pack_i32(exposure, gain, ir_lum, cross, single, 1))
    if layout == "e":
        # single-laser focus (dsp1 702-like) as first field
        return pad60(pack_i32(single, cross, ir_lum, exposure, gain, 1))
    if layout == "f":
        # flag byte stream: enable + u16 lights
        return pad60(bytes([1, 0]) + pack_u16(cross, single, ir_lum, exposure, gain))
    raise ValueError(layout)


def apply_sets(interface: int, ops: list[tuple[int, bytes]], settle: float) -> list[str]:
    """Apply SET_CUR ops under one claim; return status lines."""
    lines: list[str] = []
    with XuSession(interface) as xu:
        for cs, data in ops:
            try:
                xu.set_cur(cs, data)
                time.sleep(settle)
                length = xu.get_len(cs)
                cur = xu.get_cur(cs, length)
                ok = cur[: len(data)] == data or cur == data.ljust(length, b"\x00")[:length]
                lines.append(f"SET CS={cs} -> {data.hex()} readback={cur.hex()} sticky={ok}")
            except usb.core.USBError as exc:
                lines.append(f"SET CS={cs} -> {data.hex()} FAILED: {exc}")
    return lines


def candidates() -> dict[str, list[tuple[int, bytes]]]:
    """Named SET sequences to try (interface 0 unless noted in name)."""
    p_a = pop4_step_payload(layout="a")
    p_b = pop4_step_payload(layout="b")
    p_e = pop4_step_payload(layout="e")
    p_f = pop4_step_payload(layout="f")
    cs10_on = bytes.fromhex("01000000")
    cs10_ff = bytes.fromhex("ffffffff")
    cs10_20 = bytes.fromhex("20000000")  # DEF
    cs14_ff = pad60(bytes([0xFF]))
    cs14_01 = pad60(bytes([0x01]))

    return {
        # Best known (2026-07-18): CS10 sticky enable + CS7 pop4 int32 block
        "cs10_then_cs7": [(10, cs10_on), (7, p_a)],
        "cs7_pop4_a": [(7, p_a)],
        "cs10_on": [(10, cs10_on)],
        "cs10_ff": [(10, cs10_ff)],
        "cs10_def": [(10, cs10_20)],
        "cs10_then_cs8": [(10, cs10_on), (8, p_a)],
        "cs10_then_cs1": [(10, cs10_on), (1, p_a)],
        "cs10_then_cs2": [(10, cs10_on), (2, p_a)],
        # structured 60-byte variants
        "cs1_pop4_a": [(1, p_a)],
        "cs1_pop4_b": [(1, p_b)],
        "cs2_pop4_a": [(2, p_a)],
        "cs2_pop4_b": [(2, p_b)],
        "cs7_pop4_e": [(7, p_e)],
        "cs8_pop4_a": [(8, p_a)],
        "cs8_pop4_f": [(8, p_f)],
        "cs14_pop4_a": [(14, p_a)],
        "cs14_pop4_b": [(14, p_b)],
        "cs14_01": [(14, cs14_01)],
        "cs14_ff": [(14, cs14_ff)],
        "cs7e_cs10": [(10, cs10_on), (7, p_e)],
        "cs8f_cs10": [(10, cs10_on), (8, p_f)],
        "full_arm_safe": [
            (10, cs10_on),
            (1, p_b),
            (7, p_a),
            (8, p_f),
            (14, p_b),
        ],
        # Avoid CS5 2-byte / CS4 flag SETs — they can disconnect the composite.
        "cs5_1byte": [(5, bytes([1]))],
        "cs3_exp800_cs10": [(3, struct.pack("<I", 800)), (10, cs10_on)],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", help="comma-separated candidate names")
    ap.add_argument("--settle", type=float, default=0.15)
    ap.add_argument("--score", choices=["gst", "cv2"], default="gst")
    ap.add_argument("--interface", type=int, default=0, choices=[0, 2])
    ap.add_argument("--threshold", type=int, default=HIT_THRESHOLD)
    ap.add_argument("--stop-on-hit", action="store_true", default=True)
    ap.add_argument("--no-stop-on-hit", action="store_false", dest="stop_on_hit")
    args = ap.parse_args()

    all_cands = candidates()
    names = list(all_cands)
    if args.only:
        want = [n.strip() for n in args.only.split(",") if n.strip()]
        missing = [n for n in want if n not in all_cands]
        if missing:
            print(f"unknown candidates: {missing}", file=sys.stderr)
            print(f"known: {', '.join(names)}", file=sys.stderr)
            return 2
        names = want

    print("Waiting for device / baseline score...")
    wait_for_device()
    baseline = score_depth(args.score)
    print(f"BASELINE nonzero/frame ≈ {baseline} (expect ~35)")

    hits: list[tuple[str, int]] = []
    for name in names:
        ops = all_cands[name]
        print(f"\n=== {name} ===")
        try:
            for line in apply_sets(args.interface, ops, args.settle):
                print(" ", line)
            wait_for_device()
            time.sleep(0.5)
            sc = score_depth(args.score)
        except Exception as exc:
            print(f"  ERROR: {exc}")
            wait_for_device(20)
            continue
        delta = sc - baseline
        mark = "HIT" if sc >= args.threshold else ("bump" if delta > 50 else "miss")
        print(f"  SCORE nonzero/frame ≈ {sc}  (Δ{delta:+d})  [{mark}]")
        if mark == "HIT":
            hits.append((name, sc))
            if args.stop_on_hit:
                break

    print("\n==== SUMMARY ====")
    if hits:
        for name, sc in hits:
            print(f"HIT: {name} score={sc}")
    else:
        print("No arming hit above threshold.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
