#!/usr/bin/env python3
"""Fit the POP4 right-camera row rectification from raw speckle captures.

The POP4's packed 640x2560 Y16 frame is ten 320x512 images (row stride 320):
images 0-4 come from the right camera, 5-9 from the left, and images 4 / 9
carry the projected IR speckle. The two speckle images are NOT row-aligned:
the right camera is rolled a few degrees with strong keystone, so plain row-wise
stereo only matches in a narrow central strip.

This tool fits the quadratic used by ``Pop4DepthDecoder``::

    u = (xR - 160) / 160,  v = (yR - 256) / 256
    yL = 256 + 256 * (c0 + c1 u + c2 v + c3 u^2 + c4 u v + c5 v^2)

Capture frames of a static, textured scene with some depth variation
(e.g. a box on a desk, ~300 mm away)::

    ALL3DSCAN_POP4_DUMP_DIR=/tmp/pop4 all3dscan --probe-pop4
    python3 tools/re_probe/pop4_rectify_fit.py /tmp/pop4

then run the app with the printed ``ALL3DSCAN_POP4_RECT=...`` value.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover - exercised only without OpenCV
    cv2 = None

W, H = 320, 512
RIGHT_SPECKLE, LEFT_SPECKLE = 4, 9
# Initial guess: the fit from the first unit studied (2026-10-12).
DEFAULT_COEFFS = np.array([-0.00437, -0.03004, 1.01091, -0.01107, 0.13940, -0.00572])


def load_images(path: Path) -> np.ndarray:
    """Return ten 320x512 float images (0..255) from one packed Y16 dump."""
    raw = np.fromfile(path, "<u2")
    if raw.size != 640 * 2560:
        raise ValueError(f"{path}: expected 640x2560 Y16, got {raw.size} samples")
    return raw.reshape(10, H, W).astype(np.float32) / 257.0


def basis(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    u, v = (x - 160.0) / 160.0, (y - 256.0) / 256.0
    return np.stack([np.ones_like(u), u, v, u * u, u * v, v * v], -1)


def mapped_row(c: np.ndarray, x: np.ndarray, y_right: np.ndarray) -> np.ndarray:
    return 256.0 + 256.0 * (basis(x, y_right) @ c)


def source_rows(c: np.ndarray) -> np.ndarray:
    """For each output pixel, the raw right-image row that maps onto it."""
    ys, xs = np.mgrid[0:H, 0:W].astype(np.float64)
    y_right = ys.copy()
    for _ in range(8):
        err = mapped_row(c, xs, y_right) - ys
        slope = mapped_row(c, xs, y_right + 0.5) - mapped_row(c, xs, y_right - 0.5)
        y_right -= err / np.where(np.abs(slope) > 1e-6, slope, 1.0)
    return y_right.astype(np.float32)


def rectify_right(right: np.ndarray, c: np.ndarray) -> np.ndarray:
    xs = np.tile(np.arange(W, dtype=np.float32), (H, 1))
    return cv2.remap(right, xs, source_rows(c), cv2.INTER_LINEAR, borderValue=0)


def correspondences(left: np.ndarray, right_rect: np.ndarray, dy_range: int,
                    half: int = 10, step: int = 8, min_score: float = 0.6) -> np.ndarray:
    """Block-match left speckle into the rectified right image (x, y, xR, yR)."""
    pts = []
    for y in range(20, H - 20, step):
        for x in range(20, W - 20, step):
            block = left[y - half:y + half + 1, x - half:x + half + 1]
            if block.std() < 3:
                continue
            y0, y1 = max(0, y - half - dy_range), min(H, y + half + dy_range + 1)
            x0, x1 = max(0, x - half - 100), min(W, x + half + 6)
            search = right_rect[y0:y1, x0:x1]
            if search.shape[0] <= 2 * half + 1 or search.shape[1] <= 2 * half + 1:
                continue
            scores = cv2.matchTemplate(search, block, cv2.TM_CCOEFF_NORMED)
            _, best, _, loc = cv2.minMaxLoc(scores)
            if best < min_score:
                continue
            rival = scores.copy()
            cv2.rectangle(rival, (loc[0] - 3, loc[1] - 3), (loc[0] + 3, loc[1] + 3), -1, -1)
            if rival.max() > best - 0.06:
                continue  # ambiguous match
            row, sub = loc[1], 0.0
            if 0 < row < scores.shape[0] - 1:
                a, b, c = scores[row - 1, loc[0]], scores[row, loc[0]], scores[row + 1, loc[0]]
                den = a - 2 * b + c
                if abs(den) > 1e-6:
                    sub = 0.5 * (a - c) / den
            pts.append((x, y, x0 + loc[0] + half, y0 + row + sub + half))
    return np.array(pts, np.float64).reshape(-1, 4)


def fit(left: np.ndarray, right: np.ndarray, c: np.ndarray, iterations: int = 5,
        verbose: bool = True) -> tuple[np.ndarray, float, int]:
    rms, kept = float("nan"), 0
    for it in range(iterations):
        rows = source_rows(c)
        pts = correspondences(left, rectify_right(right, c), dy_range=8 if it == 0 else 4)
        if len(pts) < 30:
            raise RuntimeError(f"only {len(pts)} correspondences; need a textured, lit scene")
        x_left, y_left, x_rect, y_rect = pts.T
        # Back-project rectified-right samples to raw right-image rows.
        y_right = cv2.remap(rows, x_rect.astype(np.float32)[None],
                            y_rect.astype(np.float32)[None], cv2.INTER_LINEAR)[0].astype(np.float64)
        A, target = basis(x_rect, y_right), (y_left - 256.0) / 256.0
        keep = np.ones(len(target), bool)
        sol = np.linalg.lstsq(A, target, rcond=None)[0]
        for _ in range(4):
            resid = np.abs(A @ sol - target) * 256.0
            keep = resid < max(0.6, 2.5 * np.median(resid))
            sol = np.linalg.lstsq(A[keep], target[keep], rcond=None)[0]
        c = sol
        rms = float(np.sqrt(np.mean(((A[keep] @ c - target[keep]) * 256.0) ** 2)))
        kept = int(keep.sum())
        if verbose:
            print(f"iter {it}: {len(pts)} matches, {kept} kept, residual {rms:.3f} px", file=sys.stderr)
    return c, rms, kept


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dumps", type=Path, help="directory of pop4_640x2560_*.y16 dumps (or one file)")
    ap.add_argument("--swap", action="store_true", help="treat image 4 as left (ALL3DSCAN_POP4_SWAP_LR=1)")
    args = ap.parse_args()
    if cv2 is None:
        print("opencv-python-headless is required (pip install -r tools/re_probe/requirements.txt)",
              file=sys.stderr)
        return 2
    files = sorted(args.dumps.glob("*.y16")) if args.dumps.is_dir() else [args.dumps]
    if not files:
        print(f"no .y16 dumps in {args.dumps}", file=sys.stderr)
        return 2
    # Average frames of the static scene to suppress sensor noise.
    stack = np.mean([load_images(f) for f in files], axis=0)
    li, ri = (RIGHT_SPECKLE, LEFT_SPECKLE) if args.swap else (LEFT_SPECKLE, RIGHT_SPECKLE)
    left = cv2.GaussianBlur(stack[li], (0, 0), 1.0)
    right = cv2.GaussianBlur(stack[ri], (0, 0), 1.0)
    coeffs, rms, kept = fit(left, right, DEFAULT_COEFFS.copy())
    roll = np.degrees(np.arctan2(coeffs[1] * 256 / 160, coeffs[2]))
    print(f"# {len(files)} frames, {kept} correspondences, residual {rms:.3f} px, roll {roll:.2f} deg")
    print("ALL3DSCAN_POP4_RECT=" + ",".join(f"{v:.5f}" for v in coeffs))
    return 0 if rms < 0.5 else 1


if __name__ == "__main__":
    raise SystemExit(main())
