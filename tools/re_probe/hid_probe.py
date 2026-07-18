#!/usr/bin/env python3
"""Blind HID opcode sweep for the POP4 control interface (interface 4).

Requires /dev/hidraw* access for VID:PID 2207:110c — either run as root, or
install resources/99-all3dscan-scanners.rules first:
    sudo cp resources/99-all3dscan-scanners.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger

Strategy: send a candidate report, read back whatever the device replies
with (if anything), then score the depth stream's nonzero-pixel count
(depth_capture.nonzero_score). A report that meaningfully increases the
score is a strong candidate for "arm scan" / "laser on". This is a search,
not a guaranteed hit — the real framing may need a header/checksum this
sweep doesn't try. Treat every "hit" as a hypothesis to confirm manually
(repeat it alone, check it's reproducible, then read back what changed).

Everything is logged to hid_probe_log.csv so a hit isn't lost even if the
sweep is interrupted.
"""
import argparse
import csv
import sys
import time

import hid

from depth_capture import open_depth, nonzero_score

VENDOR_ID = 0x2207
PRODUCT_ID = 0x110C

# From the actual HID report descriptor (read 2026-07-16 via
# hid.device.get_report_descriptor(), decoded by hand):
#   Usage Page (Vendor 0x8c), Usage 0x01, Collection (Application)
#     Usage 0x03, Report Size 8, Report Count 0x80 (128), Input
#     Usage 0x04, Report Size 8, Report Count 0x80 (128), Output
#   End Collection
# No Report ID tag -> the device does not use numbered reports. hidapi's
# write() convention still requires a leading dummy 0x00 "report ID" byte
# regardless, so the actual wire buffer is 1 + 128 = 129 bytes. The first
# sweep used a 64-byte buffer (wrong on both counts: too short, and the
# leading byte ate into what should have been payload) - the device
# silently no-op'd every candidate. This is corrected here.
PAYLOAD_LEN = 128
REPORT_LEN = 1 + PAYLOAD_LEN  # leading dummy report-ID byte + payload
SETTLE_FRAMES = 5
SCORE_MULTIPLIER = 3.0  # "hit" if score jumps to > baseline * this


def build_candidate(opcode: int, arg: int) -> list[int]:
    buf = [0] * REPORT_LEN
    buf[0] = 0x00  # dummy report-ID byte required by hidapi's write() convention
    buf[1] = opcode
    buf[2] = arg
    return buf


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--opcode-start", type=int, default=0x00)
    ap.add_argument("--opcode-end", type=int, default=0x100)
    ap.add_argument("--arg", type=int, default=0x01)
    ap.add_argument("--log", default="hid_probe_log.csv")
    ap.add_argument("--depth-device", default="/dev/video0")
    ap.add_argument("--delay", type=float, default=0.1, help="seconds between write and score")
    args = ap.parse_args()

    try:
        h = hid.device()
        h.open(VENDOR_ID, PRODUCT_ID)
    except OSError as e:
        print(f"could not open HID device: {e}", file=sys.stderr)
        print("check resources/99-all3dscan-scanners.rules is installed, or run as root", file=sys.stderr)
        return 1
    h.set_nonblocking(True)

    cap = open_depth(args.depth_device)

    baseline = nonzero_score(cap)
    print(f"baseline nonzero score: {baseline}")

    hits = []
    with open(args.log, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["opcode", "arg", "score", "response_hex"])

        for opcode in range(args.opcode_start, args.opcode_end):
            candidate = build_candidate(opcode, args.arg)
            try:
                h.write(candidate)
            except OSError as e:
                writer.writerow([hex(opcode), hex(args.arg), "write_failed", str(e)])
                continue

            time.sleep(args.delay)
            response = h.read(PAYLOAD_LEN, timeout_ms=50)

            for _ in range(SETTLE_FRAMES):
                nonzero_score(cap)
            score = nonzero_score(cap)

            resp_hex = bytes(response).hex() if response else ""
            writer.writerow([hex(opcode), hex(args.arg), score, resp_hex])

            if score > baseline * SCORE_MULTIPLIER:
                hits.append((opcode, score, resp_hex))
                print(f"HIT  opcode=0x{opcode:02x} score={score} (baseline {baseline}) resp={resp_hex}")

    cap.release()
    h.close()

    print(f"\nswept 0x{args.opcode_start:02x}..0x{args.opcode_end:02x}, {len(hits)} hit(s)")
    for opcode, score, resp_hex in hits:
        print(f"  0x{opcode:02x}: score={score} resp={resp_hex}")
    print(f"full log: {args.log}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
