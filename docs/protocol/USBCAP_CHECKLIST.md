# Vendor-app USBPcap Capture Checklist (POP4)

Use this checklist to produce capture-backed protocol fixtures for All3DScan.
Metric depth, blue-laser mode, and lighting controls must not be invented —
only replay commands confirmed by these captures.

## Environment

- The vendor Windows app (note the exact version used)
- POP4 `2207:110c`, cold power-cycle before each session
- Windows host or VM with exclusive USB passthrough
- USBPcap + Wireshark
- Diffuse matte target; never look into the projector/laser

## Capture procedure

1. Start USBPcap on the hub that will host the POP4 **before** plugging in.
2. One short experiment per PCAPNG file.
3. Record wall-clock notes: UI clicks, visible emitter state, distance, mode.
4. Save matching official PLY/mesh when scanning a measured plane.
5. Sanitize serial numbers before committing fixtures.

## Required sessions

| File stem | Actions |
| --- | --- |
| `01_cold_plug` | Plug in → app recognizes → exit |
| `02_idle_connect` | Connect, idle preview off |
| `03_structured_preview` | Structured-light preview 2–3 s → stop |
| `04_blue_laser` | Blue-laser mode start → steady → stop |
| `05_align_light` | Alignment/cross guide on → off |
| `06_illum_low_med_high` | IR/depth brightness low/med/high |
| `07_exposure_only` | One exposure change |
| `08_gain_only` | One gain change |
| `09_depth_modes` | Each advertised Y16 mode |
| `10_single_frame` | Single-frame capture |
| `11_scan_pause` | Scan → pause → resume → stop |
| `12_device_info` | Device info / calibration dialog |
| `12b_depth_scale_intrinsics` | Capture around connect/info while SDK reads `PROPERTY_EXT_DEPTH_SCALE` (`0x0918`) and `cameraGetStreamIntrinsics(streamType=0)` (68-byte out). Prefer Frida/API hooks if USB framing is opaque. |
| `13_shutdown` | App exit + disconnect |

## Measured-plane dataset

For structured-light and blue-laser modes, capture a matte plane at known
distances (e.g. 150 / 200 / 250 / 300 mm) plus one tilted plane. Keep:

- PCAPNG (or TShark JSON export)
- Official Revo Scan PLY/mesh
- Distance notes and mode settings

## Import into All3DScan tools

```sh
# Prefer JSON fields (portable) over raw PCAPNG blobs in git:
tshark -r session.pcapng -T json \
  -e frame.time_epoch -e usb.urb_id -e usb.endpoint_address \
  -e usb.bmRequestType -e usb.bRequest -e usb.wValue -e usb.wIndex \
  -e usb.wLength -e usb.capdata -e usb.usbpcap.usb_addr \
  > session.tshark.json

python3 tools/re_probe/pcapng_import.py session.tshark.json \
  --out fixtures/pop4/session_xu.json

# Dry-run replay transcript (no USB writes):
python3 tools/re_probe/xu_probe.py --interface 0 \
  --sequence fixtures/pop4/session_xu.json --dry-run

# Live replay only after reviewing the transcript:
python3 tools/re_probe/xu_probe.py --interface 0 \
  --sequence fixtures/pop4/session_xu.json --preserve-timing
```

## Analysis focus

Filter XU unit 4 (`wIndex` high byte `0x04`):

- SET_CUR `bmRequestType=0x21`, `bRequest=0x01`
- GET_* `bmRequestType=0xA1`

Diff captures that differ by one UI action to recover CS ordering, CS7 field
layout, mode/light selectors, off sequence, and calibration reads.

### Depth-scale / intrinsics focus

Known from APK static analysis ([`APK_REVOSCAN_6.0.2.4.md`](APK_REVOSCAN_6.0.2.4.md)):

- SDK property ID `0x0918` = `PROPERTY_EXT_DEPTH_SCALE`
- Adjacent `0x0919` = trip-depth gain (ignore for scale)
- POP4 fallback scale in SDK is `0.01f` when the property is missing/invalid
- Depth intrinsics ABI is 68 bytes; stream type `0`

When reviewing a capture or hook log, record:

1. Exact request/response bytes around the first `0x0918` GET.
2. Whether the payload subtype is `4` (`scale = 1.0 / integer`).
3. The full 68-byte intrinsics buffer before any float conversion.
4. Any accompanying reads of `/data/camparam/*` or `camparam.tar(.gz)`.

Do **not** probe `0x0918` as a raw UVC selector on Linux; replay only a
capture-backed envelope.
