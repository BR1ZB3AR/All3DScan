# Revo Scan 6 → Native Ubuntu (clean-room)

**Goal:** a from-scratch Linux app that talks to the same POP4 hardware and
produces comparable scan results, shipped as a `.deb`, running **natively**
on Ubuntu (no Wine at runtime).

**Why not Wine?** Running the official Windows app under Wine/Proton puts
USB and GPU behind a compatibility layer. In practice the POP4 often does
not enumerate cleanly to the Windows process, and Revopoint’s Direct3D/CUDA
renderer does not map to a stable native GPU path. That is unsuitable for
daily scanning — hence this clean-room native port.

**Not the goal:** wrapping `RevoScan6-*.exe` or shipping proprietary
`3DCamera.dll` / CUDA / Direct3D binaries inside a Debian package. That would
be redistribution of closed-source Windows software, not a native Linux port.

**Optional RE aid only:** a one-shot USB capture from Windows (VM with USB
passthrough, or a temporary dual-boot) can reveal the XU arming sequence.
Day-to-day use of *this* project never requires Wine. Static analysis of an
already-extracted install tree is also enough to guide probing from Linux
alone (`tools/re_probe/`).

Source analyzed: Wine install of **Revo Scan 6 v6.2.3.875** used as a
**read-only SDK corpus** (same build as
`RevoScan6-6.2.3.875-windows-x86_64.exe`). The 1.4 GiB installer is an Inno
Setup / PE GUI package; once extracted, the interesting payload is the
`Program Files/Revo Scan 6/` tree. That tree is reference material — it is
not how end users run scans on Ubuntu.

## What the Windows app actually is

| Layer | Windows artifact | Linux native equivalent |
|---|---|---|
| UI shell | `RevoScan6.exe` + Qt/CTK widgets | Our Qt5 `all3dscan` |
| Device SDK | `3DCamera.dll` | `IScannerProtocol` + V4L2 + UVC XU |
| Scanner mgr | `scannermgr.dll`, `scan.dll`, `rvscan.dll` | C++ acquisition + Python workers |
| Laser recon | `RPALaserRecon.dll`, `RPAFusionLaser.dll` | Open3D / custom decode |
| Calibration | `calibration.dll`, `cscalibration.dll`, `RPACalibLaser.dll` | Extracted/device-queried params |
| GPU path | CUDA 12 + cuDNN + Direct3D | Optional Open3D/CUDA later; CPU first |

POP4 is explicitly named in the SDK (`Inspire2/POP4/POP4 Pro`).

## Critical protocol findings (from `3DCamera.dll` strings)

Control is **not** a bespoke bulk USB protocol. The SDK funnels vendor
operations through **UVC Extension Unit** helpers:

- `setXu` / `getXu` / `getXuData`
- `setPropertyExtension` / `getPropertyExtension` (`type=%04X`)
- `cameraSetPropertyExtension` / `cameraGetPropertyExtension`

High-value `PROPERTY_EXT_*` names observed:

- `PROPERTY_EXT_WRITE_READ_HID` — HID reports are wrapped **through the XU**
- `PROPERTY_EXT_EXECUTE_CMD`
- `PROPERTY_EXT_PAUSE_DEPTH_STREAM` / `PROPERTY_EXT_RESUME_DEPTH_STREAM`
- `PROPERTY_EXT_DEPTH_SCALE` (errors mention `setXu` / `getXuData`)
- `PROPERTY_EXT_TRIGGER_MODE`, `PROPERTY_EXT_HDR_EXPOSURE`
- `PROPERTY_EXT_HARDSYNCSWITCH`, `PROPERTY_EXT_SERIAL_NUMBER`
- RGB exposure / gyro / temperature extensions (many are network-model only)

Camera API surface we must reimplement for POP4 USB:

- `cs::Camera::ledAndLaserCtrl`
- `cs::Camera::setProjectorCtrlInWP`
- `cs::Camera::setFringePattern`
- `cs::Camera::setlaserCalParam`
- `cs::playCamera::getIntrinsics` / `getPropertyExtension`

This aligns with our existing Linux findings: UVC depth/RGB nodes + vendor XU
GUID `C2211A6D-787E-3C4A-AC06-7E434B676F9A` (unit ID 4). Blind HID opcode
sweeps failed earlier because the real path is likely
**XU → PROPERTY_EXT_WRITE_READ_HID / EXECUTE_CMD**, not raw hidraw framing.

Network models still use `/cgi-bin/zx_cmd.cgi?...` HTTP; that path is irrelevant
for POP4-over-USB but explains the JSON-RPC strings in the same DLL.

## Calibration assets (not full intrinsics)

`Resources/config/calibration/stepInfo/HighResolution/pop4.json` is a list of
staging steps (gain, exposure, `j_irLum`, `k_crossLaserLight`,
`l_singleLaserLight`). Useful for reconstructing the arm/exposure sequence.

`Resources/config/pop4/camparam.bin` (2061 bytes) is **not** plain float32
intrinsics (values decode as garbage) — likely obfuscated or CRC-wrapped.
Per-device intrinsics are expected via `getIntrinsics` / XU GETs at runtime.

## Clean-room roadmap → `.deb`

1. **Capture** Revo Scan 6 (Wine or Windows VM) arming a POP4 scan with
   `usbmon`; decode XU SET/GET with `tools/re_probe/usbmon_parse.py`.
2. **Replay** the captured SET sequence with `xu_probe.py --sequence` until
   depth `Y16` nonzero count jumps (laser on).
3. **Decode** packed `Y16` pattern buffers → depth; validate against a
   RevoScan-exported PLY via ICP RMS.
4. **Implement** `RevopointProtocol : IScannerProtocol` in C++ (V4L2 + XU).
5. **Mesh** via hybrid Python Open3D worker, then ship:

```sh
./tools/package_linux.sh deb
# → dist/all3dscan_0.1.0_amd64.deb
sudo apt install ./dist/all3dscan_0.1.0_amd64.deb
```

The `.deb` already installs: `all3dscan`, desktop/AppStream metadata, icon,
`devices.json`, `mesh_worker.py`, docs, and the POP4 udev rule.

## Legal / engineering boundary

- **Allowed:** observe behavior, document wire formats, reimplement
  interoperable drivers and algorithms in our own code.
- **Not allowed in our `.deb`:** bundling `RevoScan6.exe`, `3DCamera.dll`,
  CUDA redistributables, or other proprietary payloads from the installer.

Analyze an installed tree without copying proprietary binaries into the repo:

```sh
python3 tools/re_probe/analyze_revoscan_install.py \
  "/path/to/Program Files/Revo Scan 6"
```
