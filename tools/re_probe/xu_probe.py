#!/usr/bin/env python3
"""GET/SET probing over the POP4's vendor UVC Extension Unit.

Found 2026-07-16 by reading the raw USB config descriptor (uvcvideo
doesn't expose this XU as a V4L2 control since it doesn't recognize the
GUID). Lives identically on interface 0 (depth) and interface 2 (RGB):

    GUID:    C2211A6D-787E-3C4A-AC06-7E434B676F9A
    Unit ID: 4
    Active control selectors (bmControls bitmap ff a2 a7): 1-8, 10, 14

Requires temporarily detaching the kernel's uvcvideo driver from the
target interface (libusb can't send interface-recipient control
transfers while it holds the interface). This script reattaches it
afterward, but see docs/protocol/RE_NOTES.md's "Caution" note: detaching
interface 0 has been observed to cause the whole composite device to
drop off the bus briefly (self-recovers in ~5s, no action needed).

Usage:
    python3 xu_probe.py --interface 0 --dump              # GET_LEN + GET_CUR for all known CS
    python3 xu_probe.py --interface 0 --cs 10 --set 01000000  # SET_CUR (hex bytes)
    python3 xu_probe.py --interface 0 --sequence xu_transactions.json
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Optional

VENDOR_ID = 0x2207
PRODUCT_ID = 0x110C
UNIT_ID = 4
KNOWN_CS = [1, 2, 3, 4, 5, 6, 7, 8, 10, 14]

GET_CUR, GET_MIN, GET_MAX, GET_RES, GET_LEN, GET_INFO, GET_DEF = (
    0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
)
SET_CUR = 0x01


def _usb():
    try:
        import usb.core
        import usb.util
    except ImportError as exc:
        raise RuntimeError(
            "pyusb is required for live XU access (pip install pyusb)"
        ) from exc
    return usb.core, usb.util


class XuSession:
    """Claims the interface on enter, detaches/reattaches the kernel driver
    around it so multiple GET/SET calls share one claim (avoids repeated
    detach/reattach cycles, which is what seems to trigger the disconnect)."""

    def __init__(self, interface: int):
        usb_core, usb_util = _usb()
        self._usb_util = usb_util
        self.interface = interface
        self.dev = usb_core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if self.dev is None:
            raise RuntimeError("POP4 not found (check it's connected)")
        self.detached = False

    def __enter__(self):
        if self.dev.is_kernel_driver_active(self.interface):
            self.dev.detach_kernel_driver(self.interface)
            self.detached = True
        self._usb_util.claim_interface(self.dev, self.interface)
        return self

    def __exit__(self, *exc):
        self._usb_util.release_interface(self.dev, self.interface)
        if self.detached:
            self.dev.attach_kernel_driver(self.interface)

    def get_len(self, cs: int) -> int:
        wValue = cs << 8
        wIndex = (UNIT_ID << 8) | self.interface
        raw = self.dev.ctrl_transfer(0xA1, GET_LEN, wValue, wIndex, 2, timeout=500)
        return raw[0] | (raw[1] << 8)

    def get_cur(self, cs: int, length: int) -> bytes:
        wValue = cs << 8
        wIndex = (UNIT_ID << 8) | self.interface
        return bytes(self.dev.ctrl_transfer(0xA1, GET_CUR, wValue, wIndex, length, timeout=500))

    def set_cur(self, cs: int, data: bytes) -> None:
        wValue = cs << 8
        wIndex = (UNIT_ID << 8) | self.interface
        self.dev.ctrl_transfer(0x21, SET_CUR, wValue, wIndex, data, timeout=500)


def load_set_sequence(path: str, interface: int) -> list[dict]:
    """Load successful SET_CUR transactions emitted by usbmon_parse.py."""
    with open(path, encoding="utf-8") as source:
        document = json.load(source)
    transactions = document.get("xu", document) if isinstance(document, dict) else document
    if not isinstance(transactions, list):
        raise ValueError("sequence must be a list or an object containing an 'xu' list")

    sequence = []
    for transaction in transactions:
        if (
            transaction.get("direction") == "set"
            and transaction.get("request", SET_CUR) == SET_CUR
            and transaction.get("interface") == interface
            and transaction.get("status") in (None, "0")
            and transaction.get("data")
        ):
            sequence.append(transaction)
    if not sequence:
        raise ValueError(f"no successful SET_CUR transactions found for interface {interface}")
    return sequence


def format_transcript(sequence: list[dict], preserve_timing: bool, max_delay: float) -> list[str]:
    """Build a human-readable dry-run transcript without touching USB."""
    lines = []
    previous_timestamp = None
    for index, transaction in enumerate(sequence, start=1):
        timestamp = transaction.get("timestamp_us")
        delay = 0.0
        if preserve_timing and previous_timestamp is not None and timestamp is not None:
            delay = min(max(0.0, (timestamp - previous_timestamp) / 1_000_000), max_delay)
        selector = int(transaction["selector"])
        data = bytes.fromhex(transaction["data"])
        lines.append(
            f"[{index:03d}/{len(sequence):03d}] wait={delay:.3f}s "
            f"SET CS={selector} iface={transaction.get('interface')} -> {data.hex()}"
        )
        previous_timestamp = timestamp
    return lines


def replay_sequence(
    xu: Optional[XuSession],
    sequence: list[dict],
    preserve_timing: bool,
    max_delay: float,
    dry_run: bool = False,
) -> None:
    previous_timestamp = None
    for index, transaction in enumerate(sequence, start=1):
        timestamp = transaction.get("timestamp_us")
        if preserve_timing and previous_timestamp is not None and timestamp is not None:
            delay = max(0.0, (timestamp - previous_timestamp) / 1_000_000)
            if not dry_run:
                time.sleep(min(delay, max_delay))
        selector = int(transaction["selector"])
        data = bytes.fromhex(transaction["data"])
        prefix = "DRY" if dry_run else f"{index:03d}/{len(sequence):03d}"
        print(f"[{prefix}] SET CS={selector} -> {data.hex()}")
        if not dry_run:
            assert xu is not None
            xu.set_cur(selector, data)
        previous_timestamp = timestamp


def wait_for_device(timeout_s: float = 10.0) -> bool:
    """Poll until POP4 reappears after a USB re-enumeration."""
    usb_core, _ = _usb()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if usb_core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID) is not None:
            return True
        time.sleep(0.25)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interface", type=int, default=0, choices=[0, 2], help="0=depth VC, 2=RGB VC")
    ap.add_argument("--dump", action="store_true", help="GET_LEN + GET_CUR for all known control selectors")
    ap.add_argument("--cs", type=int, help="control selector to target for --get/--set")
    ap.add_argument("--get", action="store_true")
    ap.add_argument("--set", metavar="HEXBYTES", help="e.g. 01000000")
    ap.add_argument(
        "--sequence", metavar="JSON",
        help="replay SET_CUR transactions exported by usbmon_parse.py / pcapng_import.py",
    )
    ap.add_argument(
        "--dry-run", action="store_true",
        help="print a replay transcript without opening the device or writing XU",
    )
    ap.add_argument(
        "--transcript", metavar="PATH",
        help="write dry-run / live replay transcript lines to this file",
    )
    ap.add_argument(
        "--wait-reenum", type=float, default=0.0,
        help="seconds to wait for POP4 reappearance before claiming (0=disabled)",
    )
    ap.add_argument(
        "--preserve-timing", action="store_true",
        help="preserve captured delays while replaying a sequence",
    )
    ap.add_argument(
        "--max-delay", type=float, default=0.5,
        help="maximum preserved delay between sequence writes (default: 0.5s)",
    )
    ap.add_argument("--settle", type=float, default=0.1, help="seconds to sleep after SET before readback")
    args = ap.parse_args()

    if args.sequence and args.dry_run:
        sequence = load_set_sequence(args.sequence, args.interface)
        lines = format_transcript(sequence, args.preserve_timing, args.max_delay)
        for line in lines:
            print(line)
        if args.transcript:
            with open(args.transcript, "w", encoding="utf-8") as out:
                out.write("\n".join(lines) + "\n")
        return 0

    if args.wait_reenum > 0 and not wait_for_device(args.wait_reenum):
        print(f"POP4 did not reappear within {args.wait_reenum:.1f}s", file=sys.stderr)
        return 2

    with XuSession(args.interface) as xu:
        if args.sequence:
            sequence = load_set_sequence(args.sequence, args.interface)
            if args.transcript:
                lines = format_transcript(sequence, args.preserve_timing, args.max_delay)
                with open(args.transcript, "w", encoding="utf-8") as out:
                    out.write("\n".join(lines) + "\n")
            replay_sequence(xu, sequence, args.preserve_timing, args.max_delay, dry_run=False)
            return 0

        if args.dump:
            for cs in KNOWN_CS:
                try:
                    length = xu.get_len(cs)
                    cur = xu.get_cur(cs, length)
                    print(f"CS={cs:2d} len={length:3d} cur={cur.hex()}")
                except Exception as e:  # noqa: BLE001 — pyusb USBError when available
                    print(f"CS={cs:2d} failed: {e}")
            return 0

        if args.cs is None:
            ap.error("--cs is required unless --dump / --sequence")

        if args.set is not None:
            data = bytes.fromhex(args.set)
            xu.set_cur(args.cs, data)
            time.sleep(args.settle)
            print(f"SET CS={args.cs} -> {data.hex()}")

        if args.get or args.set is not None:
            length = xu.get_len(args.cs)
            cur = xu.get_cur(args.cs, length)
            print(f"CS={args.cs} len={length} cur={cur.hex()}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
