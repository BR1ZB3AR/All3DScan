#!/usr/bin/env python3
"""Characterize POP4 packed Y16 modes and extract four-step wrapped phase.

Findings from RevoScan 6.0.2.4's libRPASpeckleStereo.so:

* ``calculatePhaseFromFourStep`` consumes four images.
* It computes two differences (frame 1 - frame 3 and frame 0 - frame 2),
  then calls OpenCV ``phase`` and rejects pixels with modulation < 3.
* ``processPhaseDepth`` computes phase for two four-image vectors, refines
  disparity, then maps uint16 disparity through a 65,536-entry depth LUT.

The POP4's 640x2560 Y16 mode is exactly five vertically stacked 640x512
planes. This tool tests the first four as one four-step phase set and writes
diagnostic PGM images. It does not claim metric depth: a second phase set and
the per-device stereo calibration are still required.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def split_planes(frame: np.ndarray, width: int, height: int) -> tuple[str, list[np.ndarray]]:
    if height % 512 == 0 and width == 640:
        return "vertical-640x512", list(np.split(frame, height // 512, axis=0))
    if width % 640 == 0 and height == 512:
        return "horizontal-640x512", list(np.split(frame, width // 640, axis=1))
    return "single", [frame]


def intensity_float(plane: np.ndarray) -> np.ndarray:
    """Normalize duplicated-byte Y16 (0xNNNN) to an approximate 8-bit value."""
    return plane.astype(np.float32) / 257.0


def four_step_phase(planes: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(planes) < 4:
        raise ValueError("four-step phase requires at least four planes")
    i0, i1, i2, i3 = (intensity_float(plane) for plane in planes[:4])
    # Recovered native order: x=(I0-I2), y=(I1-I3), cv::phase(x,y,false).
    x = i0 - i2
    y = i1 - i3
    phase = np.mod(np.arctan2(y, x), 2.0 * math.pi)
    modulation = np.hypot(x, y)
    valid = modulation >= 3.0
    phase = np.where(valid, phase, 0.0)
    return phase, modulation, valid


def write_pgm(path: Path, image: np.ndarray) -> None:
    values = np.clip(image, 0, 65535).astype(">u2", copy=False)
    with path.open("wb") as output:
        output.write(f"P5\n{values.shape[1]} {values.shape[0]}\n65535\n".encode())
        output.write(values.tobytes())


def summarize_plane(index: int, plane: np.ndarray) -> dict:
    nonzero = plane[plane != 0]
    low = plane & 0xFF
    high = plane >> 8
    return {
        "index": index,
        "shape": list(plane.shape),
        "nonzero": int(nonzero.size),
        "min": int(plane.min()),
        "max": int(plane.max()),
        "duplicated_byte_fraction": float(np.mean(low == high)),
        "percentiles_nonzero": (
            np.percentile(nonzero, [1, 5, 50, 95, 99]).round(2).tolist()
            if nonzero.size
            else []
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path, help="little-endian uint16 frame")
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--output-prefix", type=Path)
    args = parser.parse_args()

    samples = np.fromfile(args.raw, dtype="<u2")
    expected = args.width * args.height
    if samples.size != expected:
        parser.error(f"expected {expected} uint16 samples, got {samples.size}")
    frame = samples.reshape(args.height, args.width)
    layout, planes = split_planes(frame, args.width, args.height)
    report = {
        "file": str(args.raw),
        "layout": layout,
        "planes": [summarize_plane(i, plane) for i, plane in enumerate(planes)],
    }

    if len(planes) >= 4:
        phase, modulation, valid = four_step_phase(planes)
        report["four_step"] = {
            "order": "x=I0-I2, y=I1-I3, phase=atan2(y,x)",
            "valid_pixels": int(np.count_nonzero(valid)),
            "valid_fraction": float(np.mean(valid)),
            "phase_percentiles_rad": (
                np.percentile(phase[valid], [1, 5, 50, 95, 99]).round(6).tolist()
                if np.any(valid)
                else []
            ),
            "modulation_percentiles": np.percentile(
                modulation, [1, 5, 50, 95, 99]
            ).round(3).tolist(),
        }
        if args.output_prefix:
            prefix = args.output_prefix
            prefix.parent.mkdir(parents=True, exist_ok=True)
            phase_u16 = phase * (65535.0 / (2.0 * math.pi))
            modulation_u16 = np.minimum(modulation * 257.0, 65535.0)
            write_pgm(prefix.with_suffix(".phase.pgm"), phase_u16)
            write_pgm(prefix.with_suffix(".modulation.pgm"), modulation_u16)
            np.save(prefix.with_suffix(".phase.npy"), phase.astype(np.float32))

        if layout == "vertical-640x512" and planes[0].shape[1] == 640:
            stereo = {}
            for name, column_slice in (
                ("left", slice(0, 320)),
                ("right", slice(320, 640)),
            ):
                camera_planes = [plane[:, column_slice] for plane in planes[:4]]
                camera_phase, camera_modulation, camera_valid = four_step_phase(
                    camera_planes
                )
                stereo[name] = {
                    "shape": list(camera_phase.shape),
                    "valid_pixels": int(np.count_nonzero(camera_valid)),
                    "valid_fraction": float(np.mean(camera_valid)),
                    "phase_percentiles_rad": (
                        np.percentile(
                            camera_phase[camera_valid], [1, 5, 50, 95, 99]
                        ).round(6).tolist()
                        if np.any(camera_valid)
                        else []
                    ),
                }
                if args.output_prefix:
                    phase_u16 = camera_phase * (65535.0 / (2.0 * math.pi))
                    modulation_u16 = np.minimum(
                        camera_modulation * 257.0, 65535.0
                    )
                    write_pgm(
                        args.output_prefix.with_suffix(f".{name}.phase.pgm"),
                        phase_u16,
                    )
                    write_pgm(
                        args.output_prefix.with_suffix(f".{name}.modulation.pgm"),
                        modulation_u16,
                    )
                    np.save(
                        args.output_prefix.with_suffix(f".{name}.phase.npy"),
                        camera_phase.astype(np.float32),
                    )
            report["stereo_four_step"] = stereo

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
