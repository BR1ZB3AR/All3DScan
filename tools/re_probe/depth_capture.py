#!/usr/bin/env python3
"""V4L2 capture harness for the POP4 depth camera (/dev/video0, raw Y16 modes).

Standalone: dumps per-frame stats (min/max/nonzero count) so you can see
whether the structured-light projector is firing. Also importable by
hid_probe.py as the scoring function for the HID opcode sweep.

Usage:
    python3 depth_capture.py [--device /dev/video0] [--width 640] [--height 512] [--frames 8]
"""
import argparse
import sys

import cv2
import numpy as np

DEFAULT_DEVICE = "/dev/video0"
DEFAULT_SIZE = (640, 512)  # (width, height) — also valid: 640x1024, 640x2560, 1920x512


def open_depth(device: str = DEFAULT_DEVICE, size: tuple[int, int] = DEFAULT_SIZE) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"could not open {device} — check permissions / udev rule")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"Y16 "))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    return cap


def read_frame(cap: cv2.VideoCapture) -> np.ndarray:
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("frame grab failed")
    # OpenCV may hand back a 2-channel uint8 view of the uint16 buffer
    # depending on backend/version; normalize to a flat uint16 array.
    if frame.dtype != np.uint16:
        frame = frame.view(np.uint16).reshape(frame.shape[0], -1)
    return frame


def nonzero_score(cap: cv2.VideoCapture) -> int:
    """Cheap proxy for 'is the projector firing': count of nonzero pixels.

    Baseline with the projector off (measured 2026-07-16, 640x512): ~35
    nonzero pixels at fixed positions. A HID command that meaningfully
    changes this is a strong candidate for the scan-arm/laser-on opcode.
    """
    frame = read_frame(cap)
    return int(np.count_nonzero(frame))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--device", default=DEFAULT_DEVICE)
    ap.add_argument("--width", type=int, default=DEFAULT_SIZE[0])
    ap.add_argument("--height", type=int, default=DEFAULT_SIZE[1])
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--save", help="dump the last frame's raw bytes to this path")
    args = ap.parse_args()

    cap = open_depth(args.device, (args.width, args.height))
    last = None
    for i in range(args.frames):
        frame = read_frame(cap)
        last = frame
        print(
            f"frame {i}: shape={frame.shape} min={frame.min()} max={frame.max()} "
            f"nonzero={np.count_nonzero(frame)}/{frame.size}"
        )
    cap.release()

    if args.save and last is not None:
        last.tofile(args.save)
        print(f"saved raw frame to {args.save}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
