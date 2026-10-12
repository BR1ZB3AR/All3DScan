#!/usr/bin/env python3
"""Import POP4 UVC XU / HID transactions from TShark JSON (USBPcap PCAPNG).

USBPcap PCAPNG is not parsed directly — export fields with tshark first
(see docs/protocol/USBCAP_CHECKLIST.md). Output schema matches
usbmon_parse.py so xu_probe.py --sequence can replay Windows captures.

Usage:
    python3 pcapng_import.py session.tshark.json --out xu.json
    python3 pcapng_import.py session.tshark.json --vid 0x2207 --pid 0x110c
"""
from __future__ import annotations

import argparse
import json
import sys
from typing import Any

XU_UNIT_ID = 4
XU_SET = 0x21
XU_GET = 0xA1
SET_CUR = 0x01


def _layers(packet: dict[str, Any]) -> dict[str, Any]:
    return packet.get("_source", {}).get("layers", {})


def _first(value: Any, default: str = "") -> str:
    if isinstance(value, list):
        return str(value[0]) if value else default
    if value is None:
        return default
    return str(value)


def _hex_payload(raw: str) -> str:
    cleaned = raw.replace(":", "").replace(" ", "").lower()
    if cleaned.startswith("0x"):
        cleaned = cleaned[2:]
    if len(cleaned) % 2:
        cleaned = "0" + cleaned
    return cleaned


def _int_field(value: str, default: int = 0) -> int:
    text = value.strip()
    if not text:
        return default
    if text.startswith("0x") or text.startswith("0X"):
        return int(text, 16)
    # TShark sometimes emits decimal for lengths and hex for USB fields.
    try:
        return int(text, 0)
    except ValueError:
        return int(text, 16)


def extract_xu_transactions(packets: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Correlate USB control submissions into completed XU transactions."""
    pending: dict[str, dict[str, Any]] = {}
    completed: list[dict[str, Any]] = []

    for packet in packets:
        layers = _layers(packet)
        bm = _first(layers.get("usb.bmRequestType"))
        if not bm:
            # Completion packets often omit setup; match by urb id.
            urb = _first(layers.get("usb.urb_id") or layers.get("usb.usbpcap.urb_id"))
            data = _first(layers.get("usb.capdata"))
            if urb and urb in pending and data:
                pending[urb]["data"] = _hex_payload(data)
                pending[urb]["status"] = "0"
                completed.append(pending.pop(urb))
            continue

        request_type = _int_field(bm)
        if request_type not in (XU_SET, XU_GET):
            continue

        index = _int_field(_first(layers.get("usb.wIndex")))
        if (index >> 8) != XU_UNIT_ID:
            continue

        request = _int_field(_first(layers.get("usb.bRequest")))
        value = _int_field(_first(layers.get("usb.wValue")))
        length = _int_field(_first(layers.get("usb.wLength")))
        selector = (value >> 8) & 0xFF
        interface = index & 0xFF
        data = _hex_payload(_first(layers.get("usb.capdata")))
        ts = _first(layers.get("frame.time_epoch") or layers.get("frame.time_relative"))
        try:
            timestamp_us = int(float(ts) * 1_000_000) if ts else None
        except ValueError:
            timestamp_us = None

        tx = {
            "direction": "set" if request_type == XU_SET else "get",
            "request_type": request_type,
            "request": request,
            "selector": selector,
            "interface": interface,
            "length": length,
            "data": data,
            "status": "0" if data or request_type == XU_SET else None,
            "timestamp_us": timestamp_us,
        }

        urb = _first(layers.get("usb.urb_id") or layers.get("usb.usbpcap.urb_id"))
        if urb and not data and request_type == XU_GET:
            pending[urb] = tx
        else:
            if request_type == XU_SET and not data and urb:
                pending[urb] = tx
            else:
                completed.append(tx)

    # Flush unmatched submissions that already carried payload.
    for tx in pending.values():
        if tx.get("data") is not None:
            completed.append(tx)

    completed.sort(key=lambda item: item.get("timestamp_us") or 0)
    return completed


def load_tshark_json(path: str) -> list[dict[str, Any]]:
    with open(path, encoding="utf-8") as source:
        document = json.load(source)
    if isinstance(document, list):
        return document
    if isinstance(document, dict) and "packets" in document:
        return document["packets"]
    raise ValueError("expected a TShark -T json array (or {\"packets\": [...]})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="TShark JSON export from a USBPcap PCAPNG")
    ap.add_argument("--out", help="Write {\"xu\": [...]} JSON here (default: stdout)")
    ap.add_argument("--vid", default=None, help="Optional filter hint (unused; retained for docs)")
    ap.add_argument("--pid", default=None, help="Optional filter hint (unused; retained for docs)")
    args = ap.parse_args()

    packets = load_tshark_json(args.input)
    xu = extract_xu_transactions(packets)
    payload = {"xu": xu, "source": args.input, "count": len(xu)}
    text = json.dumps(payload, indent=2)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as out:
            out.write(text + "\n")
    else:
        sys.stdout.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
