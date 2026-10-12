# Hardware support

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
