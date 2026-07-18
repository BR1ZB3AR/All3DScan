#!/usr/bin/env python3
"""Static analysis helper for an installed Revo Scan 6 tree (Wine or Windows).

Does not copy proprietary binaries into the project. Prints protocol-relevant
symbols/strings and POP4 config summaries to accelerate clean-room RE.

Usage:
    python3 analyze_revoscan_install.py "/path/to/Program Files/Revo Scan 6"
"""
from __future__ import annotations

import argparse
import json
import re
import struct
import subprocess
import sys
from pathlib import Path

PROPERTY_RE = re.compile(r"PROPERTY_EXT_[A-Z0-9_]+")
API_RE = re.compile(
    r"(ledAndLaserCtrl|setProjectorCtrlInWP|setFringePattern|setlaserCalParam|"
    r"getIntrinsics|setPropertyExtension|getPropertyExtension|setXu|getXu|"
    r"getXuData|WRITE_READ_HID|PAUSE_DEPTH_STREAM|RESUME_DEPTH_STREAM|"
    r"EXECUTE_CMD|DEPTH_SCALE|POP4)"
)


def run_strings(path: Path) -> list[str]:
    result = subprocess.run(
        ["strings", "-n", "6", str(path)],
        check=False,
        capture_output=True,
        text=True,
        errors="replace",
    )
    return result.stdout.splitlines()


def summarize_dll(path: Path) -> None:
    lines = run_strings(path)
    props = sorted({m.group(0) for line in lines for m in PROPERTY_RE.finditer(line)})
    apis = sorted({m.group(0) for line in lines for m in API_RE.finditer(line)})
    print(f"\n## {path.name}")
    print(f"size={path.stat().st_size}")
    if props:
        print("PROPERTY_EXT:")
        for name in props:
            print(f"  - {name}")
    if apis:
        print("API / markers:")
        for name in apis:
            print(f"  - {name}")


def summarize_pop4_json(path: Path) -> None:
    data = json.loads(path.read_text(encoding="utf-8"))
    print(f"\n## {path.relative_to(path.parents[4]) if len(path.parts) > 4 else path.name}")
    if isinstance(data, list) and data:
        sample = data[0]
        keys = [
            "7_gain",
            "8_exposure",
            "j_irLum",
            "k_crossLaserLight",
            "l_singleLaserLight",
        ]
        print(f"steps={len(data)}")
        print("first_step:")
        for key in keys:
            if key in sample:
                print(f"  {key}: {sample[key]}")
    elif isinstance(data, dict):
        print(f"keys={list(data)[:12]}")


def summarize_camparam(path: Path) -> None:
    blob = path.read_bytes()
    print(f"\n## {path.name}")
    print(f"size={len(blob)}")
    preview = blob[:32]
    floats = [struct.unpack_from("<f", preview, i)[0] for i in range(0, 32, 4)]
    print(f"head_hex={preview.hex()}")
    print(f"head_f32={floats}")
    plausible = [v for v in floats if 100.0 < abs(v) < 10000.0]
    print(
        "plain_intrinsics_likely="
        + ("yes" if plausible else "no (obfuscated/encrypted or non-float layout)")
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("install_root", type=Path)
    args = parser.parse_args()
    root: Path = args.install_root
    if not root.is_dir():
        print(f"not a directory: {root}", file=sys.stderr)
        return 1

    print(f"# Revo Scan 6 install analysis\nroot={root}")
    for name in (
        "3DCamera.dll",
        "RPACamera.dll",
        "scannermgr.dll",
        "calibration.dll",
        "cscalibration.dll",
        "RPALaserRecon.dll",
        "RPAFusionLaser.dll",
    ):
        path = root / name
        if path.is_file():
            summarize_dll(path)

    pop4 = root / "Resources/config/calibration/stepInfo/HighResolution/pop4.json"
    if pop4.is_file():
        summarize_pop4_json(pop4)

    camparam = root / "Resources/config/pop4/camparam.bin"
    if camparam.is_file():
        summarize_camparam(camparam)

    print(
        "\nNext: capture usbmon while RevoScan arms the POP4, then:\n"
        "  python3 tools/re_probe/usbmon_parse.py capture.mon --channel xu --json xu.json\n"
        "  python3 tools/re_probe/xu_probe.py --interface 0 --sequence xu.json\n"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
