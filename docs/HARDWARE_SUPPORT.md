# Hardware support

## Every scanner, in the sense that can be promised

There are two halves to working with a scanner, and they are not equally
open.

**Taking a scan from it live** needs that model's USB protocol, which no
vendor documents and which has to be recovered with the hardware in hand.
That is per-device work; the matrix below says honestly how far it has got
for each one, and detection is not scanning.

**Everything after the scan** — cleaning, fusing, meshing, fitting planes and
cylinders, STEP, and saving — needs no protocol at all. Every scanner on the
market writes a point cloud or a mesh, so All3DScan reads

| | |
| --- | --- |
| `.ply` | text and binary, either byte order |
| `.obj` | mesh or cloud |
| `.stl` | text and binary |
| `.pcd` | text and binary (not `binary_compressed`) |
| `.asc` `.xyz` `.pts` `.txt` `.p3` | plain columns, layout worked out from the numbers |

and writes all of those back. **File → Open scan** brings any of them in as a
model with the whole toolchain behind it, so a scanner All3DScan cannot yet
drive is still a scanner it can work with today. Normals are worked out for a
file that carries none, since fitting needs them; a file written in metres is
noticed and offered in millimetres.

The one thing an opened file cannot do is **Mesh**, which melts a scan's own
depth views together and so needs a scan taken here. A file that already
holds a mesh keeps it.

## What the app says about a scanner it finds

Every scanner in `resources/devices.json` that is on the USB bus appears in
the scanner list, whether or not it can be driven. One that cannot says so
in plain words and points at File → Open scan, rather than being left out as
though the app had not noticed it.

A model joins the drivable list in `gui_python/scanners.py` only once
scanning has actually been tested on it.

## Live capture

Support levels are evidence-based:

- **Candidate**: family identified; protocol work has not started.
- **Detected**: Linux enumerates the device and All3DScan can match it.
- **Capture**: raw depth/RGB streams can be acquired.
- **Scanning**: projector/control, calibration, metric depth, tracking, and
  reconstruction work end-to-end.
- **Verified**: scanning is repeatedly tested on the named model/firmware.

## Current matrix

| Vendor | Model/family | USB identity | Level | Notes |
| --- | --- | --- | --- | --- |
| Revopoint | POP4 | `2207:110c` | Capture (experimental) | Native XU arming + packed 640×2560 Y16 → approximate metric decode. Factory camparam/Q still open. Revo “AI” features are host-side (see `docs/protocol/RE_NOTES.md` AI parity). |
| Revopoint | POP/POP2/POP3/MINI/RANGE | vendor `2207` | Candidate | Vendor-wide detection is enabled, but this does not prove protocol compatibility. |
| Creality | CR-Scan Otter | `2bc5:06da` | Detected | Public Linux descriptor evidence confirms three UVC functions plus HID. Capture/control protocol is not implemented. |
| Creality | Ferret/Ferret Pro/Raptor | vendor `2bc5` | Candidate | Vendor-wide detection is enabled; model VID/PIDs and protocol behavior require hardware captures. |
| Einstar / Shining3D | Einstar family | unknown | Candidate | Placeholder entry is disabled until a real `lsusb` capture confirms VID/PID. |
| Einstar / Shining3D | Einstar Rockit | `2207:2001` (reported) | Candidate | VID/PID and a full protocol description were contributed in [issue #2](https://github.com/BR1ZB3AR/All3DScan/issues/2). Nothing in it is verified here — no Rockit has been attached — and none of its commands are wired up. It describes a laser-line scanner with on-device peak extraction, which shares no transport with the POP4's UVC path. |

## Reporting another model

Use the hardware-support issue template and provide:

- exact model and firmware version;
- `lsusb` and `lsusb -v -d VID:PID` output;
- V4L2 device/format output;
- operating system and kernel version;
- whether RGB, infrared/depth, HID, or network interfaces appear;
- sanitized USB/network captures if available.

Never publish device credentials, serial numbers, proprietary binaries, or
vendor SDK files.
