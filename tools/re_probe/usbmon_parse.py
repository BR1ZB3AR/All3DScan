#!/usr/bin/env python3
"""Parse POP4 HID and UVC Extension Unit traffic from usbmon text captures.

Capture (needs root):
    sudo modprobe usbmon
    # find the bus number: check `readlink -f /sys/bus/usb/devices/1-3` -> busnum file,
    # or just use 0u to capture all buses if unsure.
    sudo cat /sys/kernel/debug/usb/usbmon/0u > capture.mon
    # ^ run in background, trigger hid_probe.py or the real app in another
    #   shell, then Ctrl-C.

Usage:
    python3 usbmon_parse.py capture.mon
    python3 usbmon_parse.py capture.mon --channel xu --json xu_transactions.json
"""
import argparse
import json
import re
import sys

# usbmon text format (one line per URB event), fields of interest:
#   urb_tag timestamp event_type address(bus:dev:ep) status length data...
# address examples: "Ci:1:003:1" (control in), "Ii:1:003:83" (interrupt in),
# "Io:1:003:01" (interrupt out). The endpoint byte at the end is what we
# filter on for the HID interface (0x01 out / 0x83 in).
LINE_RE = re.compile(
    r"^(?P<tag>\S+)\s+(?P<ts>\d+)\s+(?P<event>\S+)\s+(?P<addr>\S+)\s+(?P<status>\S+)\s+(?P<rest>.*)$"
)
SETUP_RE = re.compile(
    r"(?:^|\s)s\s+"
    r"(?P<request_type>[0-9a-fA-F]{2})\s+"
    r"(?P<request>[0-9a-fA-F]{2})\s+"
    r"(?P<value>[0-9a-fA-F]{4})\s+"
    r"(?P<index>[0-9a-fA-F]{4})\s+"
    r"(?P<length>[0-9a-fA-F]{4})(?:\s|$)"
)
DATA_RE = re.compile(r"(?:^|\s)=\s+(?P<data>(?:[0-9a-fA-F]{2}\s*)+)$")

XU_UNIT_ID = 4
XU_REQUEST_TYPES = {0x21, 0xA1}


def parse_line(line: str):
    m = LINE_RE.match(line.strip())
    if not m:
        return None
    addr = m.group("addr")
    # address form: <type><dir>:<bus>:<devnum>:<ep>
    parts = addr.split(":")
    if len(parts) < 4:
        return None
    ep = parts[-1]
    return {
        "tag": m.group("tag"),
        "ts": int(m.group("ts")),
        "event": m.group("event"),
        "addr": addr,
        "ep": ep,
        "urb_status": m.group("status"),
        "rest": m.group("rest"),
    }


def parse_data(rest: str) -> bytes:
    match = DATA_RE.search(rest)
    if not match:
        return b""
    return bytes.fromhex(match.group("data"))


def parse_xu_setup(rest: str):
    """Decode a UVC XU SETUP packet, if this line addresses POP4 unit 4."""
    match = SETUP_RE.search(rest)
    if not match:
        return None
    request_type = int(match.group("request_type"), 16)
    index = int(match.group("index"), 16)
    if request_type not in XU_REQUEST_TYPES or index >> 8 != XU_UNIT_ID:
        return None
    return {
        "direction": "set" if request_type == 0x21 else "get",
        "request_type": request_type,
        "request": int(match.group("request"), 16),
        "selector": int(match.group("value"), 16) >> 8,
        "interface": index & 0xFF,
        "length": int(match.group("length"), 16),
    }


def collect_transactions(lines):
    """Return decoded HID events and completed XU control transactions."""
    hid = []
    pending_xu = {}
    xu = []
    for line in lines:
        parsed = parse_line(line)
        if parsed is None:
            continue

        if parsed["ep"].lower() in ("01", "83"):
            hid.append({**parsed, "data": parse_data(parsed["rest"]).hex()})

        setup = parse_xu_setup(f"{parsed['urb_status']} {parsed['rest']}")
        if parsed["event"] == "S" and setup is not None:
            transaction = {
                **setup,
                "tag": parsed["tag"],
                "timestamp_us": parsed["ts"],
                "address": parsed["addr"],
                "data": parse_data(parsed["rest"]).hex(),
                "status": None,
            }
            pending_xu[parsed["tag"]] = transaction
            continue

        if parsed["event"] in ("C", "E") and parsed["tag"] in pending_xu:
            transaction = pending_xu.pop(parsed["tag"])
            response = parse_data(parsed["rest"])
            if transaction["direction"] == "get" and response:
                transaction["data"] = response.hex()
            transaction["status"] = parsed["urb_status"]
            transaction["completion_timestamp_us"] = parsed["ts"]
            xu.append(transaction)

    # Retain incomplete submissions: interrupted captures are common.
    xu.extend(pending_xu.values())
    xu.sort(key=lambda item: item["timestamp_us"])
    return hid, xu


def print_hid(events):
    print(f"found {len(events)} HID URB events on endpoints 0x01/0x83")
    for event in events:
        print(
            f"[{event['ts']:>12}] {event['event']:>2} "
            f"{event['addr']:<14} {event['rest']}"
        )


def print_xu(transactions):
    print(f"found {len(transactions)} UVC XU transactions for unit {XU_UNIT_ID}")
    for tx in transactions:
        status = tx["status"] if tx["status"] is not None else "incomplete"
        print(
            f"[{tx['timestamp_us']:>12}] {tx['direction'].upper():>3} "
            f"if={tx['interface']} cs={tx['selector']:>2} "
            f"req=0x{tx['request']:02x} len={tx['length']:>3} "
            f"status={status} data={tx['data']}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", help="usbmon text capture")
    parser.add_argument(
        "--channel", choices=("all", "hid", "xu"), default="all",
        help="traffic to display (default: all)",
    )
    parser.add_argument("--json", metavar="PATH", help="write decoded transactions as JSON")
    args = parser.parse_args()

    with open(args.capture, encoding="utf-8", errors="replace") as capture:
        hid, xu = collect_transactions(capture)

    if args.channel in ("all", "hid"):
        print_hid(hid)
    if args.channel in ("all", "xu"):
        print_xu(xu)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as output:
            json.dump({"hid": hid, "xu": xu}, output, indent=2)
            output.write("\n")
        print(f"wrote decoded capture to {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
