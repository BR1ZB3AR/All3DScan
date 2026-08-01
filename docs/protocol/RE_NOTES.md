# Revopoint POP4 USB Protocol — Reverse Engineering Notes

Status: **Phase B in progress.** Device identified and enumerated
2026-07-16 with a physical POP4 connected. Key finding: the POP4 is a
**UVC/HID composite device**, not a proprietary bulk-transfer device — see
"Step 1" and "Step 3" below. This changes the implementation plan (see
bottom of this doc).

## Native C++ milestone (2026-07-18)

All3DScan now contains a native `RevopointPop4Protocol` backend:

- opens the confirmed `2207:110c` device through libusb;
- detaches `uvcvideo` from depth VideoControl interface 0;
- sends XU unit 4 CS10 (`01 00 00 00`) and the 60-byte CS7 settings payload;
- releases the interface and reattaches `uvcvideo`;
- discovers the matching V4L2 capture node through sysfs/VID/PID and advertised
  format instead of assuming `/dev/video0`;
- captures 640×512 Y16 with V4L2 MMAP.

The native smoke test:

```sh
./build/all3dscan --probe-pop4
```

was run against a physical POP4 and captured 327,680 Y16 samples, 14,678 of
them nonzero. The frame is intentionally tagged `VendorRawY16`; point-cloud
generation rejects it because packed structured-light decoding and factory
calibration are not yet known. This prevents visually plausible but false
metric geometry.

## Background reading (not POP4-specific, but relevant context)

- `frostworx/revopoint-pop2-linux-info` (GitHub): the POP2 runs an embedded
  Linux system with SSH exposed (default root password `internet`), open
  ports 22/tcp and 80/tcp. This is the POP2's *host-side control channel*
  finding, not a USB structured-light streaming protocol, and POP2's
  architecture may or may not resemble POP4's. Worth checking whether POP4
  exposes similar embedded services when first connected.
- Revopoint's official `Revopoint/3DViewer` (GitHub, GPL-3.0) is a Qt5/CMake
  app built against their proprietary "3DCamera" SDK. It doesn't include
  POP4 support and its SDK is closed-source, but its structure is a useful
  reference for how they structure Qt + camera-control apps.
- No public reverse-engineering of POP4's USB protocol was found as of
  2026-07-11.

## Step 1: Identify the device

Once connected:
```
lsusb
lsusb -t
lsusb -v -d <vendor>:<product>   # full descriptor dump, once IDs are known
```
Record vendor ID, product ID, and all interface/endpoint descriptors here.
Update `resources/devices.json` and `resources/99-all3dscan-scanners.rules`
with the real values.

### Findings (2026-07-16)

`lsusb` wasn't installed on the dev machine; descriptors were read directly
from `/sys/bus/usb/devices/<bus-port>/` instead (works identically, no
extra packages needed).

**Device-level:**
| Field | Value |
|---|---|
| idVendor | `2207` (Fuzhou Rockchip — the SoC vendor, not Revopoint itself) |
| idProduct | `110c` |
| bcdDevice | `0601` |
| manufacturer string | `Chishine3d` (Revopoint's manufacturing entity) |
| product string | `REVO_PRODUCT` (generic, not POP4-specific in the string itself) |
| bDeviceClass/Sub/Proto | `ef/02/01` (Miscellaneous — Interface Association Descriptor, standard for composite devices) |
| bNumInterfaces | 5 |
| bMaxPower | 500 mA |
| USB version | 2.00, High Speed (480 Mbps) |
| serial | `D26545575B6S00E1B` |

**Interface layout — the device is a UVC + HID composite, not a bespoke
bulk protocol:**

| Interface | Class | Subclass | Role |
|---|---|---|---|
| 0 | `0e` (Video) | `01` (VideoControl) | Depth function control |
| 1 | `0e` (Video) | `02` (VideoStreaming) | Depth function stream |
| 2 | `0e` (Video) | `01` (VideoControl) | RGB function control |
| 3 | `0e` (Video) | `02` (VideoStreaming) | RGB function stream |
| 4 | `03` (HID) | — | Vendor control channel: 2 interrupt endpoints (`0x01` OUT, `0x83` IN), both `wMaxPacketSize=0x400`, `bInterval=4` |

All 5 interfaces bind to stock Linux drivers with **no custom kernel
module needed**: interfaces 0-3 → `uvcvideo`, interface 4 → `usbhid`.
This produced standard device nodes:

- `/dev/video0` — depth camera, capture (`REVO_PRODUCT: POP4-REVO-DepthCa`)
- `/dev/video1` — depth camera, metadata-only companion node
- `/dev/video2` — RGB camera, capture (`REVO_PRODUCT: POP4-REVO-RGBCam`)
- `/dev/video3` — RGB camera, metadata-only companion node
- `/dev/hidraw7` (index varies by system) — the HID control interface

**Depth camera (`/dev/video0`) supported formats** (`v4l2-ctl -d /dev/video0
--list-formats-ext`):
- `MJPG` 1280x1024 @ up to 30fps
- `YUYV` 1280x1024 and 640x512 @ up to 15fps
- `Y16 ` (16-bit greyscale, note trailing space in the fourcc) at four
  sizes: 640x512, 640x1024, 640x2560, 1920x512 — all @ up to 15fps

The non-square multiples in the Y16 sizes (1024=512×2, 2560=512×5,
1920=640×3) strongly suggest these are **multiple raw structured-light
pattern captures packed into one V4L2 buffer** (e.g., a Gray-code or
phase-shift pattern sequence), not pre-computed depth-in-mm. Decoding
those patterns into an actual depth map is likely where Revopoint's
proprietary logic lives — this is probably the crux of the RE effort, not
the USB transport itself.

**RGB camera (`/dev/video2`)** only advertises `MJPG` 1280x1024.

### Step 1 result

Real VID:PID (`2207:110c`) recorded in `resources/devices.json` and
`resources/99-all3dscan-scanners.rules` (also added a hidraw permission rule,
since `/dev/hidraw*` defaults to root-only and interface 4 is needed for
control).

## Phase 1 tooling (Python prototyping, `tools/re_probe/`)

Separate from the C++ app — these are throwaway probing scripts, not
production code. Install deps with `pip install -r
tools/re_probe/requirements.txt`.

- `depth_capture.py` — V4L2 capture harness for `/dev/video0` (Y16 raw
  depth). Standalone: dumps per-frame min/max/nonzero stats. Also
  importable — `nonzero_score()` is the scoring function `hid_probe.py`
  uses to detect when a HID command changes the depth stream. Verified
  working 2026-07-16 (matches the manual `v4l2-ctl` capture: baseline
  ~35/327680 nonzero pixels, projector off).
- `hid_probe.py` — blind brute-force sweep over HID opcodes on the
  control interface, scoring each candidate against `depth_capture`'s
  nonzero count and logging every `(opcode, response, score)` to
  `hid_probe_log.csv`. Unblocked 2026-07-16 once the udev rule from
  `resources/99-all3dscan-scanners.rules` was installed on the real machine.
  Ran a full 0x00-0xFF sweep twice (once with a wrong 64-byte frame,
  once corrected to the real 129-byte frame per the actual HID report
  descriptor) — **zero hits both times, zero device responses at all**.
  See "HID control channel: findings" below for why — the real control
  channel turned out to be a UVC Extension Unit, not this HID interface
  (or at least not with simple single-byte framing).
- `usbmon_parse.py` — parser for `usbmon` text captures, filtered to
  endpoints `0x01`/`0x83` (the HID interface's OUT/IN endpoints). Not yet
  used — became lower priority once the XU path (below) opened up a
  faster route to real control.
- `xu_probe.py` — GET/SET probing over the UVC Extension Unit found on
  interface 0 (see below). This is the active line of investigation as
  of 2026-07-16.
- `xu_arm_replay.py` — replay best-known arm sequence (CS10 + CS7 pop4
  payload) with optional gstreamer depth scoring (2026-07-18).
- `xu_arm_probe.py` — systematic SET candidate sweep scored by depth
  nonzero count.

## Static analysis of the real Revo Scan 6 Windows app (2026-07-16)

A working Wine/Proton install of the actual RevoScan6 app (v6.2.3.875)
already existed on the dev machine at
`~/Faugus/default/drive_c/Program Files/Revo Scan 6/`, with logs and
cached config from real past scanning sessions
(`~/Faugus/default/drive_c/users/steamuser/AppData/{Local,Roaming}/Revopoint/RevoScan6/`).
No `wine` binary was available in the dev sandbox to run it live, but
static analysis of the installed DLLs (`strings`, no disassembly) turned
up a lot:

### Follow-up (2026-07-18) — installer / SDK control path confirmed

The matching Windows installer
`RevoScan6-6.2.3.875-windows-x86_64.exe` is a 1.4 GiB PE GUI package.
We analyze the **installed** tree (not redistributing the EXE). Full
clean-room plan + `.deb` boundary: `docs/protocol/REVOSCAN6_RE.md`.

`3DCamera.dll` shows POP4 control goes through UVC Extension Unit helpers
(`setXu` / `getXu` / `getXuData`) exposed as `setPropertyExtension`
(`type=%04X`). Notably `PROPERTY_EXT_WRITE_READ_HID` means HID traffic is
wrapped via the XU — which explains why raw hidraw opcode sweeps scored
zero hits. Other POP4-relevant extensions:
`PROPERTY_EXT_EXECUTE_CMD`, `PAUSE/RESUME_DEPTH_STREAM`,
`PROPERTY_EXT_DEPTH_SCALE`, `TRIGGER_MODE`, `HDR_EXPOSURE`.

Re-run analysis anytime:

```sh
python3 tools/re_probe/analyze_revoscan_install.py \
  "$HOME/Faugus/default/drive_c/Program Files/Revo Scan 6"
```

- **The vendor SDK statically links `hidapi`** (`hid_open`, `hid_write`,
  `hid_read`, `hid_get_feature_report`, etc. all present as symbols in
  `3DCamera.dll`) — confirms `hidapi` is the right library for whatever
  the real HID protocol turns out to be, even though it wasn't the
  answer for laser control (see below).
- **Calibration files exist on disk as vendor defaults**, not something
  that needs extracting from device flash:
  `Resources/config/calibration/stepInfo/HighResolution/pop4.json` (+
  `pop4_check.json`, `pop4_prepose.json`) and
  `Resources/config/brightnessGainMap_POP4.json`. These turned out to be
  **auto-scan staging parameters** (per working-distance step: gain,
  exposure, `crossLaserLight`, `singleLaserLight`, `irLum`), not the
  intrinsic/extrinsic camera matrix — real per-device intrinsics are
  presumably read from the device itself at runtime. Confirms there are
  two distinct laser modes: a "cross" alignment guide and a "single" scan
  pattern, each with its own power level (~700-900 in whatever unit the
  firmware uses).
- **A JSON-RPC-shaped protocol exists in the SDK** (`3DCamera.dll` has
  literal strings like `{"msgId":"getIntrinsics","streamType":%d}` and
  `{"msgId":"getIntrinsicsRsp","result":"success",...}`, plus
  `CameraDeviceRpcClient`, `RPCServerCamera` symbols). **Tested empirically
  and ruled out for the USB HID channel**: sent the exact JSON (several
  framing variants: exact string, with newline, with a 4-byte
  length-prefix) over the HID interface via `hidapi` — zero response
  every time. Most likely this JSON-RPC path belongs to Revopoint's
  network/Wi-Fi-connected models (mini2/miracoF etc., which the same SDK
  also supports), not the POP4's USB transport.
- **Key `cs::Camera::` API surface** (symbol names only, no decompiled
  logic): `ledAndLaserCtrl`, `setProjectorCtrlInWP`, `setFringePattern`,
  `laserLineDirectionChange`, `laserLineExpTimeAdjust`, `setlaserCalParam`,
  `startStream`/`stopStream`, `setStreamResolution`, `setCameraState`,
  `setTriggerInterval`, `getIMUCalibrationState`. These are the real
  function names for the operations we need — useful as a checklist even
  without their implementations.

## HID control channel: findings (interface 4)

Ruled out as the laser/projector control path, at least for simple
framing: two full brute-force sweeps (wrong-size and correct-size, see
`tools/re_probe/hid_probe.py` above) and hand-crafted JSON-RPC attempts
all produced zero response and zero visible effect on the depth stream.
Not fully closed off — a checksum/handshake-gated framing we haven't
guessed could still be in play — but deprioritized in favor of the UVC
Extension Unit path below, which is already confirmed live.

## UVC Extension Unit (found 2026-07-16) — active investigation

The depth (interface 0) and RGB (interface 2) VideoControl interfaces
each declare a **vendor Extension Unit** in their raw class-specific
descriptors — invisible to `v4l2-ctl --list-ctrls` because `uvcvideo`
only auto-exposes XUs it recognizes by GUID, and this one isn't in its
known list. Found by reading the raw config descriptor via `pyusb`
(`intf.extra_descriptors`) since `usbmon`/debugfs weren't available in
the sandbox:

- GUID: `C2211A6D-787E-3C4A-AC06-7E434B676F9A`
- Unit ID: `4`
- Declared controls (bmControls bitmap `ff a2 a7`, bNumControls=14):
  selectors 1-8, 10, and 14 are active; 9, 11, 12, 13 are not.
- Present identically on both interface 0 (depth) and interface 2 (RGB)
  — each camera function has its own instance of this XU.

**Accessing it requires detaching the kernel's `uvcvideo` driver from
the interface first** (`usb.core.Device.detach_kernel_driver()` /
`usb.util.claim_interface()`), since libusb refuses interface-recipient
control transfers ("Resource busy") while the kernel driver holds it.
Reattach afterward (`attach_kernel_driver()`) to restore `/dev/video0`.

**Caution — sandbox-specific(?) disconnect risk**: detaching interface 0
once caused the entire composite device to drop off the bus (`/dev/video*`
and `/sys/bus/usb/devices/1-3` disappeared entirely, not just the
targeted interface). It re-enumerated on its own after a few seconds
with no action needed. Not confirmed whether this is a quirk of this
specific USB-passthrough dev sandbox or would also happen on bare metal
— be ready for it either way, and don't panic if it happens (wait ~5s,
recheck `/sys/bus/usb/devices/1-3/idVendor`, don't force a physical
replug prematurely).

**Standard UVC control requests used** (`bmRequestType`/`bRequest`/
`wValue`/`wIndex` per the UVC spec, XU addressed via
`wIndex = (unit_id << 8) | interface_number`, control selector via
`wValue = cs << 8`):

| CS | GET_LEN | GET_CUR (at rest) | Notes |
|---|---|---|---|
| 1 | 60 | all zero | unidentified — likely a structured param block |
| 2 | 60 | all zero | unidentified |
| 3 | 4 | `89130000` LE = `5001` | matches `exposure_time_absolute` reported by standard UVC camera controls — probably mirrors/reports current exposure |
| 4 | 60 | all zero | unidentified |
| 5 | 2 | `0000` | candidate enable/mode flag; **SET_CUR with a 2-byte payload timed out on the request itself; a 1-byte payload was accepted by the write call but the immediate GET_LEN readback then also timed out**, and a fresh readback moments later showed it back at `0000`. Inconsistent — possibly needs an exact-format payload the firmware validates before ACKing, or isn't writable at all from this state. |
| 6 | 60 | ASCII `"v5.3.80.20260427\|v101.8.6.0414"` | **firmware/algorithm version string** — confirmed useful read-only control |
| 7 | 60 | all zero | unidentified |
| 8 | 60 | all zero | unidentified |
| 10 | 4 | `00000000` | **SET_CUR to `01000000` succeeded and reads back correctly, confirmed clean (no disconnect) on a second try with `tools/re_probe/xu_probe.py`** — held for 6 depth frames afterward, nonzero-pixel count stayed at baseline (~35/327680). Write path definitely works; this specific value on this specific control does not visibly fire the laser. Not fully ruled out (could need a companion control set at the same time, e.g. alongside CS=5) but deprioritized. |
| 14 | 60 | all zero | unidentified |

**Next steps for this thread:**
- Retry CS=10 SET_CUR without an intervening device reset, holding the
  interface claimed across both the write and several seconds of
  `depth_capture.py` polling, to get a clean before/after comparison.
- Try CS=5 SET_CUR with a 1-byte or different-endianness payload (the
  timeout suggests the device didn't like the length or format we sent).
- Try the same GET/SET sweep against interface 2's copy of the XU
  (RGB function) — control semantics may differ per function.
- For the six 60-byte all-zero blocks (CS 1,2,4,7,8,14): try writing a
  plausible structured payload modeled on `pop4.json`'s step fields
  (gain, exposure, `crossLaserLight`, `singleLaserLight`, `irLum` — all
  small integers that would fit as e.g. a run of `int32`/`float` fields)
  rather than a single flag byte, since these are clearly not simple
  toggles.

## Session 2026-07-18 — XU dump, GET_INFO, arming hit (native Ubuntu)

Environment: project `.venv` with `pyusb` + `numpy` + `opencv-python-headless`
(+ existing `hidapi`). Depth scoring via gstreamer
`GRAY16_LE` 640×512 (no `v4l2-ctl` on host). Device live at `2207:110c`,
`/dev/video0` + `/dev/hidraw7`.

### XU dump (interface 0 = depth VC; interface 2 identical)

| CS | GET_LEN | GET_INFO | GET_CUR (rest) | Notes |
|---|---|---|---|---|
| 1 | 60 | `03` (R/W) | zeros | structured block; SET accepted, GET stays zero |
| 2 | 60 | `03` | zeros | same |
| 3 | 4 | `03` | `89130000` (=5001) | exposure mirror; MIN=`581b0000` |
| 4 | 60 | `03` | zeros | MAX first dword=`1`; **SET can disconnect whole composite** |
| 5 | 2 | `03` | `0000` | MAX=`0100` (LE enable=1); **2-byte SET times out + disconnect** |
| 6 | 60 | `01` (RO) | `v5.3.80.20260427\|v101.8.6.0414` | firmware string |
| 7 | 60 | `03` | zeros | **arm candidate** (see below); SET is fire-and-forget |
| 8 | 60 | `03` | zeros | same family as 7 |
| 10 | 4 | `03` | `00000000` | SET `01000000` sticky; alone does **not** raise depth score |
| 14 | 60 | `03` | zeros | MAX first byte `ff` |

Reproduce dump:

```sh
.venv/bin/python tools/re_probe/xu_probe.py --interface 0 --dump
.venv/bin/python tools/re_probe/xu_probe.py --interface 2 --dump
```

### DLL / PreISP notes (strings only, no redistribution)

From `3DCamera.dll` (Wine install tree):

- Still no numeric `PROPERTY_EXT_*` type IDs in strings — only
  `setPropertyExtension type=%04X` / `Type=%04X` / JSON `propertyType:%d`.
  28 `PROPERTY_EXT_*` names + `XU_DSP_GAIN`, `XU_FILE_*`, `XU_NEAR_FAR_STATE_CHANGE`.
- Network/embedded path shells Rockchip PreISP registers, e.g.
  `echo s 0x706 %d >/dev/rk_preisp`, `0x702 %d %d`, `0xb00`/`0xb01` laser-ish,
  `0x701 1`. USB path logs `hid execute cmd failed` /
  `PROPERTY_EXT_WRITE_READ_HID` — shell strings are **not** the USB wire format.
- HID report descriptor on this POP4: Report Size 64 bits × Count 128 =
  **1024-byte** reports (matches `wMaxPacketSize=0x400`). Raw hidraw writes of
  ASCII `echo s 0xb00 1 > /dev/rk_preisp` (1024-byte padded) accepted with
  **no response and no depth-score change**.

### Arming results (depth nonzero/frame @ 640×512 Y16)

| Trial | Result |
|---|---|
| Baseline (projector off) | ~34–35 |
| CS10=`01000000` alone | sticky, score stays ~35 |
| CS5=`0100` (2-byte) | timeout + full USB drop (~few s recover) |
| CS4 flag / ASCII SET | can drop device |
| Raw HID PreISP ASCII | miss |
| **CS10=`01000000` then CS7=pop4 int32 block** | **HIT: score → ~7k–24k** |
| CS7=pop4 alone | HIT in some states, miss in others — prefer CS10-first |
| After arm, GET CS7/CS10 | often zeros / CS10 may clear; **score stays high** until USB re-enum |

pop4 60-byte payload (LE `int32` × 5 + 40 zero bytes), from
`Resources/config/calibration/stepInfo/HighResolution/pop4.json` step 0:

```
gain=1, exposure=800, irLum=130, crossLaserLight=780, singleLaserLight=706
hex: 0100000020030000820000000c030000c2020000 + 40×00
```

Replay helper:

```sh
.venv/bin/python tools/re_probe/xu_arm_replay.py --score
# equivalent manual:
.venv/bin/python tools/re_probe/xu_probe.py --interface 0 --cs 10 --set 01000000
.venv/bin/python tools/re_probe/xu_probe.py --interface 0 --cs 7 \
  --set 0100000020030000820000000c030000c202000000000000000000000000000000000000000000000000000000000000000000000000000000000000
```

Broader sweep harness (optional): `tools/re_probe/xu_arm_probe.py`.

### Caveats / next steps

- Arming is **stateful in firmware**: XU GET_CUR does not reliably reflect
  “projector on”; use depth nonzero score. Clearing may need USB re-plug /
  `authorized` sysfs toggle (needs root) — `dev.reset()` often fails with
  `Entity not found` from userspace here.
- Exact field layout inside the 60-byte CS7 block is still guessed (5× int32
  from pop4.json). Narrow which fields matter; test CS8/CS1/CS2 with same
  layout after CS10.
- Capture a real Revo Scan 6 USB session (`usbmon`) to confirm whether
  production uses CS7 or HID-wrapped `EXECUTE_CMD` / `WRITE_READ_HID`.
- Map `PROPERTY_EXT_*` numeric IDs via disassembly or a logged
  `type=%04X` session — strings alone are insufficient.

## Step 2: Capture USB traffic

Preferred: run the official Revo Scan 6 app (Windows, or Wine on Linux)
while capturing:
- Linux native: `usbmon` + Wireshark (`sudo modprobe usbmon`, capture on
  `usbmonN` interface matching the bus the device is on).
- Windows: USBPcap + Wireshark, or a Windows VM with USB passthrough if
  capturing from the host.

Save captures to `tools/captures/*.pcapng` (gitignored by default — check
in only sanitized/annotated captures if useful for other contributors, not
raw dumps that might contain unrelated traffic).

Capture at minimum:
1. Device connect/enumeration handshake (cold plug-in through app
   recognizing the device).
2. Start of a scan (single-frame or live preview) — the init/config command
   sequence.
3. Steady-state streaming (a few seconds) — frame format/cadence.
4. Stop scan / disconnect.

## Step 3: Analyze

Questions to answer and document here as they're resolved:
- [x] Control vs bulk vs interrupt transfers used for commands vs data?
      Video data comes through the standard UVC isochronous/bulk pipes
      (handled transparently by `uvcvideo`/V4L2, no custom transport code
      needed for the pixel data itself). Commands go over the HID
      interface's interrupt endpoints (`0x01` OUT / `0x83` IN).
- [ ] Command framing: header format, opcode table, checksum/CRC if any.
      **Blocked**: `/dev/hidraw7` (this device's HID node) is root-only by
      default and the dev sandbox used for this session has no `sudo`, so
      we haven't captured a single HID report yet. Needs the udev rule in
      `resources/99-all3dscan-scanners.rules` installed on a real machine (`sudo
      cp ... && udevadm control --reload-rules && udevadm trigger`), or
      running as root once, to read/write `/dev/hidraw*`.
- [x] Frame data format (partial): captured raw `Y16` frames at 640x512
      from `/dev/video0` via `v4l2-ctl --stream-mmap` with **no scan
      triggered**. Result: only ~35/327680 pixels nonzero, at fixed
      positions, consistent across an 8-frame burst — i.e. the depth
      pipe is alive and streaming, but the laser/structured-light
      projector is almost certainly off, because nothing arms it yet.
      That arming is expected to be one of the HID commands above.
      Still open: once the projector *is* on, is Y16 raw multiplexed
      pattern data (per the odd 512×2/512×5/640×3 buffer sizes noted in
      Step 1) or something else? Requires HID access to test.
- [ ] Sensor resolution(s) — POP4 spec sheet mentions multiple modes
      (structured light vs blue laser); do they differ in frame format?
- [ ] Calibration data — is it read from the device (e.g., a "get
      calibration" command) or fixed/documented externally?
- [ ] Any required keep-alive/heartbeat commands during streaming?

Also noted (2026-07-16, same sandbox): attempting to stream `MJPG` from
the RGB camera (`/dev/video2`) via `v4l2-ctl --stream-mmap` hung
indefinitely (no frames, no error) where the depth camera's `Y16`/bulk-ish
stream worked fine. Not yet understood whether this is specific to this
sandboxed/virtualized USB environment (isochronous bandwidth is often the
first casualty of USB passthrough) or a real device quirk — retest on bare
metal before reading anything into it.

## Step 4: Implement

**This plan changes given the Step 1 finding that POP4 is a UVC/HID
composite device, not a bespoke bulk-USB device:**

- The video/depth pixel pipes do **not** need `src/usb/UsbDevice` or a
  custom `RevopointProtocol` transport at all — they're standard V4L2
  capture (`/dev/video0` depth, `/dev/video2` RGB), reachable with plain
  `open()`/`ioctl(VIDIOC_*)`/`mmap()`, or a small V4L2 wrapper. This is
  much simpler than the bulk-transfer path `IScannerProtocol` was
  designed around.
- `src/usb/UsbDevice` (libusb) is still relevant, but narrows to **just
  the HID control channel** (or it can move to `hidraw` ioctls instead of
  libusb, which is simpler for a HID interrupt endpoint) — sending
  "start scan" / "set mode" / laser on-off commands and reading status
  back.
- Once HID command framing is known (blocked above), implement
  `RevopointProtocol` as: HID commands to arm/configure the scan, V4L2
  capture for the actual depth/RGB frames, then decode the Y16 buffer
  (single depth-in-mm frame, or multiplexed raw patterns needing
  Gray-code/phase decode — TBD) into the point cloud pipeline. Still
  implements `IScannerProtocol` (`src/protocol/ScannerProtocol.h`) so
  `src/app/MainWindow.cpp` only needs `protocol::makeMockProtocol()`
  swapped for the real one, per the original design.
