# RevoScan Android APK 6.0.2.4 — clean-room notes

Source (read-only corpus, **not redistributed**):

`RevoScan_V6.0.2.4_202607021708_default_release.apk` (arm64-v8a only).

Goal: extract POP4 protocol / preset / calibration *facts* that we reimplement
natively in All3DScan. We do **not** ship `lib3DCamera.so` or other proprietary
binaries.

Working extract lives under `artifacts/apk_revoscan_6.0.2.4/` (gitignored).

## Architecture confirmed by the APK

| APK artifact | Role | Linux native counterpart |
|---|---|---|
| `lib3DCamera.so` | Device SDK (libusb + UVC XU) | `RevopointPop4Protocol` |
| `libcameraSdk.so` / `libscanManageSdk.so` | Scan session / depth scale | `ScannerController` + `ScanPipeline` |
| `libRPA*.so` | Registration / fusion / laser | Open3D workers + future native |
| `assets/param_pop4.json` | POP4 model `0E1` scan presets | `resources/param_pop4.json` |
| Ogre/Media assets | Proprietary UI renderer | Our Qt/OpenGL viewer |

Same control surface as the Windows `3DCamera.dll` analysis in
[`REVOSCAN6_RE.md`](REVOSCAN6_RE.md):

- `libusb_control_transfer` + `libuvcSetExtension`
- `cameraSetPropertyExtension` / `cameraGetPropertyExtension` (`type=%04X`)
- `PROPERTY_EXT_DEPTH_SCALE`, `PROPERTY_EXT_WRITE_READ_HID`,
  `PROPERTY_EXT_EXECUTE_CMD`, pause/resume depth stream
- `ledAndLaserCtrl`, `setProjectorCtrlInWP`, `getDepthScale`
- Calibration path hint: `/userdata/camparam/` on-device (fetched via SDK,
  not baked into the APK as POP4 intrinsics)

Depth helper class name observed: `cs::DecoderDepthQuicklz` — some transports
ship QuickLZ-compressed depth. POP4's larger V4L2 `Y16` modes are instead
packed structured-light image sets, not a precomputed depth plane.

## POP4 phase/depth pipeline recovered (2026-07-19)

Static ARM64 analysis plus live POP4 fixtures established:

- `libRPASpeckleStereo.so` exports `SpeckleStereoImpl::processPhaseDepth`,
  `calculatePhaseFromFourStep`, `calculatePreciseDisparity`, and
  `processGetDepth`.
- `640x2560 Y16` is five vertically stacked `640x512` temporal planes.
  Each plane contains left/right `320x512` camera images side-by-side.
- The first four planes are a four-step phase set. The native implementation
  computes `x = I0 - I2`, `y = I1 - I3`, then `cv::phase(x, y, false)`;
  modulation below `3` is invalidated.
- `processPhaseDepth` runs that operation independently for left and right,
  performs row-wise/subpixel phase correspondence, filters disparity, then
  indexes a 65,536-entry uint16 depth LUT.
- The LUT is built from the stereo reprojection `Q` matrix using the standard
  form `Z = Q(2,3) / (disparity * Q(3,2) + Q(3,3))`, adjusted by the SDK
  `depthScale`.
- Stereo calibration is loaded with OpenCV `FileStorage`. Confirmed field
  names are `width`, `height`, `cameraMatrixL`, `distCoeffL`,
  `cameraMatrixR`, `distCoeffR`, `steroRotation`, and `steroTranslation`
  (the vendor binary contains the `stero*` misspelling).
- `libcameraSdk.so` names the on-device inputs:
  `/data/camparam/camparam.yaml`, `LC_RT.bin`, `Distort.bin`, `Pl.bin`,
  `camparamLR/Q.bin`, `mapparamL.bin`, `mapparamR.bin`, and `Prgb.bin`.

The clean-room analysis utilities are:

```sh
PYTHONPATH=artifacts/python python3 tools/re_probe/elf_arm64_trace.py \
  artifacts/apk_revoscan_6.0.2.4/native/libRPASpeckleStereo.so \
  'calculatePhaseFromFourStep'

python3 tools/re_probe/pop4_y16_analyze.py frame.raw \
  --width 640 --height 2560 --output-prefix build/pop4_phase
```

Live phase extraction produced valid wrapped phase for about 61% of pixels.
This proves the transport layout and phase formula, but it is not metric depth
until per-device stereo calibration and the exact phase matcher are supplied.

An alternate hypothesis from format-name strings (`Z16Y8Y8`, `S64Y8Y8`, …)
treats the odd Y16 sizes as byte-count disguises for interleaved components.
Live `640x2560` fixtures currently match the temporal stereo-phase layout above
more closely; keep both interpretations until official mode captures are
correlated with the Java `FrameFormatType` switches.

## `PROPERTY_EXT_DEPTH_SCALE` = `0x0918` (confirmed)

In APK `lib3DCamera.so`, `getDepthScale` (`0x639e94`) issues property type
`0x0918` through the camera-device vtable getter (`+0x60`):

```text
0x63a1e8  mov w1, #0x918
0x63a1ec  sub x2, x29, #0x50   ; response buffer
0x63a1f0  sub x3, x29, #0x54   ; status / length out
```

Confirmed accompanying facts:

| Fact | Evidence |
|---|---|
| Property ID | `0x0918` (`PROPERTY_EXT_DEPTH_SCALE`) |
| Adjacent property | `0x0919` is trip-depth gain, not depth scale |
| POP4 camera types | `0x20`, `0x21` (POP4 / POP4 Pro) |
| Response subtype `4` | `scale = 1.0f / returned_integer` |
| POP4 fallback when invalid/zero | `0.01f` (`0x3c23d70a`) via `[SDK TRACE] get default depthScale %f` |
| Other fallbacks in same routine | `0.005f`, `0.05f`, `0.1f` for other camera types |

`0x0918` is an **SDK property ID inside the vendor framing**, not a raw UVC
control selector. Do **not** send it as `xu_probe.py --cs 0x918`. Capture the
framing from an official-app property-extension GET, then replay that envelope.

## Intrinsics ABI (confirmed shape)

`cameraGetStreamIntrinsics(handle, streamType, out)` at `0x71244c` is a thin
vtable trampoline (`+0xd8`). Callers use `streamType = 0` for depth. Failure
and success paths both treat the output as **68 bytes** (`0x44`):

- `uint16_t width` at offset `0`
- `uint16_t height` at offset `2`
- remaining 64 bytes hold the intrinsic matrix fields named in logs
  (`fx`, `fy`, `cx`, `cy`, `zero01`, `zero10`, `zero20`, `zero21`, `one22`)

A live 68-byte dump is still required before locking the float layout
(padded 3×3 vs 4×4). Matching on-device files: `/data/camparam/Pl.bin` and
`camparam.yaml`.

## `param_pop4.json` (model `0E1`) — usable now

Official distance / exposure / fill-light / laser presets for POP4. Example
**High Accuracy → surface → feature → general**:

| Field | Value | Use in All3DScan |
|---|---|---|
| `dmin` / `dmax` | 210 / 350 mm | Distance-scale “excellent” band |
| `histogram1..4` | 290, 350, 400, 460 | Distance-scale band edges |
| `zmin` / `zmax` | 250 / 500 mm | Clip / scale range |
| `exposure` | 3000 | CS7 exposure default (was 800) |
| `gain` | 1 | CS7 gain |
| `fillLightLuminance` | 0 (marker modes use 128) | Illumination slider target |
| `laserLuminance` | 180 (max 255) | Laser slider target (laser modes) |

These values guide CS7 field writes we already arm with. They do **not** by
themselves prove the HID/`EXECUTE_CMD` opcodes for mode switches.

## Still blocked without live capture / on-device calib

1. The XU/HID **envelope** that carries property `0x0918` (ID known; framing
   unknown). Blind selector writes are unsafe.
2. Mapping of `ledAndLaserCtrl` / blue-laser mode to exact XU or HID payloads.
3. A live 68-byte `cameraGetStreamIntrinsics(..., streamType=0, ...)` dump and
   the matching `/data/camparam` files (`camparam.yaml`, `Q.bin`, `Pl.bin`).
4. Verified emitter-off sequence (USB re-enum remains the dependable reset).

Next probes on the physical POP4:

- Prefer Windows USBPcap / Frida of Revo Scan 6 opening device info or starting
  a scan, filtered for property `0x0918` and 68-byte intrinsics
  ([`USBCAP_CHECKLIST.md`](USBCAP_CHECKLIST.md)).
- Keep capturing `640x2560` fixtures with `tools/re_probe/pop4_y16_analyze.py`
  while that framing is missing — phase layout is already useful without mm.
- Do **not** treat `0.01` as a metric decode constant until an official GET
  confirms the device returns that scale (it is only the SDK fallback).

## Legal / packaging

- APK and extracted `.so` files stay out of git and out of `.deb` packages.
- Only clean-room notes + the JSON preset values are kept in-tree.
