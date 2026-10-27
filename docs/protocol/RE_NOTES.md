# Revopoint POP4 USB Protocol — Reverse Engineering Notes

Status: **Phase B in progress.** Device identified and enumerated
2026-07-16 with a physical POP4 connected. Key finding: the POP4 is a
**UVC/HID composite device**, not a proprietary bulk-transfer device — see
"Step 1" and "Step 3" below. This changes the implementation plan (see
bottom of this doc).

## Driving the scanner from the new app (2026-10-27)

Measured while wiring the POP4 into the PyQt6 app (`gui_python/app.py`,
through `hw_cpp/src/Pop4Sensor.cpp`, which drives the same
`RevopointPop4Protocol` backend). Tests:
`ALL3DSCAN_POP4_TESTS=1 .venv/bin/python -m unittest gui_python/tests/test_pop4_hardware.py`,
which judges each control by what the scanner's own cameras then see.

### The pattern projector has to be switched on once frames are flowing

**Observed.** After `connect()` the backend's controls said the projector
was on, but its pattern was missing from the packed images it lights: local
contrast of image 5 was **0.040** (the value measured with the projector
explicitly off), against **0.118** with it on. Sending `0xb04 1` during
`connect()`, before streaming, did not change that. Sending it after
`startStream()` did, about **five frames** later.

`startStream()` now re-sends the projector's state right after the stream
starts, and again five frames in (one send alone did not take after a
connect that had to reset the scanner). Toggling it from the app's
"Pattern projector" button then shows in the infrared preview:
on **0.100**, off **0.040**, on again **0.118**.

**Inferred, untested:** the register write is dropped, or has no effect,
while the depth node is not streaming.

### The depth preview must come from an image the projector lights

**Observed, consistent with 2026-10-14.** The depth decoder matches images
4 and 9, which an always-on speckle emitter lights, so the user's projector
button never changes them: toggling it moved their local contrast by under
2% (0.129 against 0.128). Image 5 — the left camera's first ambient
exposure — is one of the images `0xb04` does light. The app's "Depth
camera" preview therefore shows image 5, so that switching the projector is
visible where the user is looking.

### A slow consumer needs the newest queued frame, not the next one

**Observed.** V4L2 hands back the oldest queued buffer. Reading one frame
per GUI repaint, the app ran the queue's depth behind the camera, and a
light switched in one frame only showed several frames later — long enough
that the projector test measured the *previous* state and failed.
`V4l2Capture::readFrame(timeoutMs, newest = true)` now drains to the newest
ready buffer, requeueing the older ones unread. `Pop4Sensor` asks for it;
the legacy app keeps the old every-frame behaviour.

### Fill light and blue laser, re-confirmed through the app's own controls

| Control | Measured through the window |
|---|---|
| Fill light on / off | Colour image mean **52 → 145-183 → 50** |
| Blue laser mode | Strongly blue pixels (B − R > 25) **0.07% → 65.95%** |
| Back to infrared | Reboot completes; depth returns (0.82% of pixels) |

### "Connected" must mean frames are flowing, not that arming worked

**Observed.** A session showed the status bar claiming the scanner was
connected while every capture failed with `V4L2 poll error (revents=0x18)` —
`POLLHUP | POLLERR`. The kernel log explains it: the scanner left the bus
mid-session (`usb 1-14.1: USB disconnect, device number 15`) and came back
twelve and twenty-four seconds later as device 17, **with different node
numbers** (`/dev/video4`, `/dev/video5`; `video0` and `video2` were gone).
The app held a descriptor on the dead node.

Two faults, both in the app rather than the scanner:

- `connect()` and `startStream()` can both succeed against a scanner that is
  on its way out, so "connected" never meant a frame had arrived.
  `Pop4Sensor` now waits for a first frame (three tries of two seconds)
  before reporting a connection, and fails with the scanner's own reason
  otherwise.
- Nothing ever un-set the status. `V4l2Capture` now records `deviceGone()`
  on poll hang-up/error or `ENODEV`, the protocol reports it through
  `stillConnected()`, and the window drops to the mock sensor, says
  "disconnected — waiting for it", and stops the scan.

**Recovery.** The window reconnects by itself: it looks for the scanner every
two seconds and opens it after two consecutive sightings, so a device still
re-enumerating is not grabbed too early. Tries are only spent while a scanner
is actually present, so a reboot of a minute or more costs none of them.

Verified on hardware by resetting the scanner's USB port mid-scan
(`--usb-reset`): the window reported `revents=0x18`, stopped claiming a
connection, disabled the lighting controls, cancelled the scan, reconnected
once the scanner returned, and scanned again.

**Also found by the test for this:** the in-flight flag that stops two
captures overlapping was never cleared when a capture died with the device,
so after reconnecting a scan would sit silent forever. Cleared on loss now.

### Depth yield is far below the 27.8% of 2026-10-16: the scene, not the code

**Observed.** Every session this day gave **0.4-1.1%** valid depth (300-1,800
points of 163,840 pixels) against 27.8% measured on 2026-10-16. Dumped frames
(`ALL3DSCAN_POP4_DUMP_DIR`) settle why, without needing the 2026-10-16 scene
back.

The two images the decoder matches, 4 (right) and 9 (left), both carry a
stronger pattern than their own ambient exposures, so the projector is firing
and the preview is reading the right plane:

| image | mean | local contrast | contrast / mean |
|---|---|---|---|
| 0-3 right ambient | 4,360 | 547 | 0.125 |
| **4 right speckle** | 8,617 | **2,094** | 0.243 |
| 5-8 left ambient | 4,020 | 175 | 0.043 |
| **9 left speckle** | 7,279 | **1,170** | 0.161 |

Subtracting a camera's ambient exposure from its speckle one isolates what the
projector adds: **45% (right) and 40% (left) of pixels gain more than 2,000
counts**, so the light reaches the scene. But what comes back is smooth
illumination of dark fabric, with no crisp dots — and **57% (right) / 64%
(left) of each image sits below 9% of full scale**. Decoding a dumped frame
offline gives disparities spread over almost the whole search band
(p10/p50/p90 = **19 / 78 / 153 px**), which is scattered false matches, not a
surface.

**Derived:** the scene — a dark mesh seat a couple of metres off, well past
the preset's 250-500 mm — is too dark and too far for the dot pattern to
resolve, so there is nothing to match. Sweeping the decoder's band over
150-1500 mm moved the yield by under 0.2 points, as expected when no distance
holds a matchable surface.

**Still unresolved:** no scan of a near, light-coloured object has been
measured on this build, so 27.8% has not been reproduced here. Also seen:
`V4L2 frame timeout` on the first capture of many sessions, occasional runs
where every capture timed out until the scanner was left alone for a minute,
and 1.5-7.4 frames/s against the 15 fps the Y16 modes advertise.

**In the app:** the window now shows the coverage in the status bar and, after
eight frames under 5%, says what to check (distance band, dark or shiny
surface) instead of leaving an empty viewport.

## Factory calibration read from the scanner (2026-10-13)

**This supersedes the fitted row map, the "lens bias" and the 28 mm
provisional baseline described below.** They remain only as a fallback when
no factory calibration is available.

- **File channel.** `libuvcReadFile` (RevoScan Android `lib3DCamera.so`
  @`0x72c450`) is read-only and uses XU unit 4 / interface 0. Open is
  `SET_CUR` selector 1 with 60 B `[0x01, path, NUL]`. Each read is
  `GET_CUR` selector 2 returning 60 B `[len LE32 ≤56, data]`, repeated
  until `len == 0`. Close is `SET_CUR` selector 1 with 60 zero bytes.
  `all3dscan --read-device-file <path> <out>` implements it.
  A scanner left in the post-session state answers with `LIBUSB_ERROR_IO`;
  `all3dscan --usb-reset` reboots it (~40 s).
- **Files.** `/data/camparam/`: `camparam.yaml`, `camparamLR/{P,Q}.bin`,
  `mapparam{L,R}.bin`, `Distort.bin`, `LC_RT.bin`, `Pl.bin`, `Prgb.bin`,
  `metroExtra.bin`; `/data/camparam_blue/` holds the blue-laser
  equivalents. `camparam.yaml` is OpenCV YAML: `cameraMatrixL/R`,
  `distCoeffL/R`, `steroRotation`, `steroTranslation`, `pL`, `pR`, `Q`, all
  at the sensors' native **1280×1024**.
- **Geometry (this unit).** Baseline 118.6 mm. The cameras are toed in by
  ~20°: Bouguet rectification rotates each about ±10°. Focal length is
  1489 px.
- **Our 320×512 images are the unrectified sensor images scaled x/4, y/2.**
  Non-square pixels: fx 372, fy 745 on our grid. Camparam "L" is the block
  of images **0–4**; the legacy model had the pair reversed. Validation:
  - Rectifying our images with `cv::stereoRectify` and
    `initUndistortRectifyMap`, using `K' = S·K` and `P' = S·P` with
    S = diag(¼, ½, 1), gives row-aligned pairs; the best vertical shift is
    0 on four captures.
  - The rectified flat faces are planar: 0.22–0.24 px RMS against 0.30–0.32
    with the fitted model, and the "bowl" disappears.
  - Depth is about 463 mm for the box (1 px of disparity ≈ 4.9 mm there).
  - `cv::stereoRectify` with alpha = −1 reproduces the file's `pL`/`pR`
    exactly.
- **Decoder.** It rectifies both speckle images with native maps
  (`decoder/StereoRig`, matching OpenCV to 1e-9 for R1/R2 and 2e-3 px for
  the maps). The right image is shifted by K px, chosen per depth preset,
  so that disparities stay positive: Z = fx·B / (d − K + Δcx). CPU and
  CUDA agree byte for byte. On the 400-frame recording, 351 frames track
  (was 332).
- **The app** reads `camparam.yaml` on Connect and caches it as
  `$XDG_CONFIG_HOME/All3DScan/camparam/<deviceID>.yaml`; it falls back to
  the cache, then to the approximate model. `ALL3DSCAN_CAMPARAM=<file>`
  overrides, including for `--decode-dump` and `--replay-dumps`.

## Foreground isolation was deleting most of the object (2026-10-18)

**Observed on a real 400-frame sweep** (the first recording of an actual
hand-held scan, `/tmp/scan1`). Tracking held only 213 of 400 frames and the
fused model came to 16k points. Stage counts per frame showed why:

| stage | points | |
|---|---|---|
| decoder output | ~24,000 valid depth pixels | |
| back-projected inside the 250-500 mm preset | 3,021 | |
| after the nearest-surface band | 1,614 | 53% |
| after dominant-plane removal | 1,614 | 100% — innocent |
| **after clustering** | **521** | **32%** |

The clustering kept only the single largest connected cluster. A close,
patchy object arrives as 5-8 clusters, not one, so most of it was discarded
every frame. Turning isolation off entirely gave 348 tracked and a 202k
model, which proved the filter — not the sensor — was the limit.

**Fix:** keep the other clusters too when each is a meaningful share of the
largest *and* its centroid lies within the chosen cluster's own depth extent
(+-20 mm). Result on the same recording: **213 -> 340 frames tracked, 187 ->
60 lost, 16k -> 53k model points.**

The first version of this test used `nearBandM` for "same depth", which a unit
test caught: that parameter is the whole working window, wide enough to
absorb a background sheet a hand's width behind the object. The cluster's own
extent is the right reference.

**Not the limiting factors, each measured and ruled out:** the scanning
distance preset (250-500 mm is the best of the three; 210-400 collapses to 1
tracked frame, and wider ranges are worse still, because the range steers the
stereo search band rather than merely clipping), and the dominant-plane
removal (already off by default).

## Exposure: the CS7 field is inert, 0x911 is real (2026-10-17)

**Observed — the CS7 settings block does not control the sensor.** Sweeping
its exposure field 3000 -> 200 (15x) left the left speckle image's saturated
fraction at 19.2% +- 0.1 and coverage unchanged. Whatever that block arms, it
is not exposure (its irLum field was already shown inert on 2026-10-14).

**Observed — `echo s 0x911 <us> > /dev/rk_preisp` is the exposure register.**
From the SDK's parameter-collection routine (which logs "set exposure time
failed" beside it; 0x910 is frame time). Sweeping it on hardware: saturation
19.4% -> 3.4% -> 0.9% -> 0.3%, mean level 22578 -> 8212. Values below ~700
clamp. The app's Exposure control now sends this register.

**Observed — saturation was not what limited that scene.** Coverage was best
at the *most* saturated setting (3.9% at 3000 vs 1.2% clean at 700): the
scene in front of the scanner sat at roughly the near clip (disparity ~125 ~=
230 mm with the 250-500 mm preset), so most of the object was range-clipped
whatever the exposure. Lesson repeated from the blue-laser work: measure the
limiting factor before tuning a lever.

**Measured and rejected — auto-exposure.** A hill-climb on the decoder's
valid fraction lost to a fixed exposure twice (median 2.5% vs 3.7%; after
fixing an oscillation bug, 2.8% vs 4.1%): per-frame yield noise exceeds the
exposure effect. Left behind `ALL3DSCAN_POP4_AUTOEXPOSURE=1`, default off.

## Colour texture from the scanner's own colour camera (2026-10-16)

**Observed — why meshes came out grey.** The "rgb" the decoder attached to
each frame was synthesised from the *infrared* images: it averages four
ambient sub-images and writes the same value into R, G and B. Every point and
every mesh vertex therefore carried a grey. The real 1280x1024 colour camera
was only ever used for the preview panel.

**Observed — the colour calibration is on the scanner.** Three small blobs sit
beside `camparam.yaml`, all little-endian float32, read over the XU file
channel:

| file | bytes | contents |
|---|---|---|
| `Prgb.bin` | 40 | one pad float, then the 3x3 colour camera matrix: fx = fy = 1557.547, cx = 633.888, cy = 518.032 |
| `Distort.bin` | 20 | k1 k2 p1 p2 k3 for that camera |
| `LC_RT.bin` | 48 | 3x3 rotation + 3 translation, T = (-24.033, 0.027, -2.404) mm |
| `Pl.bin` | 40 | same layout; fx 1489.284, cx 331.878, cy 510.769 |

**Derived — which frame LC_RT maps from.** `Pl.bin` is byte-identical to
`camparam.yaml`'s `pL`, the *rectified left* projection we already
back-project depth with. The vendor stores it beside `LC_RT.bin`, so the
rotation/translation take the rectified left frame to the colour camera, and
no un-rectification is needed. The principal point lands at the centre of a
1280x1024 sensor and the 24 mm baseline matches the physical spacing, which is
what makes the reading credible.

Implemented as `decoder::RgbCalibration` with the byte layout pinned in
`tests/test_stereo_rig.cpp`; the protocol reads and caches the blobs per
device at connect, decodes the MJPEG with libturbojpeg, and samples a colour
for each depth pixel. `ALL3DSCAN_POP4_COLOR=0` restores the old grey.

**Observed — the mapping is correct.** Confirmed by pointing the scanner at a
printed carton (camera mean saturation 32.8, p95 137, versus 17 for the grey
wall that made three earlier attempts useless). 28,516 depth pixels took
colour and 74.6% of them were genuinely chromatic. Every colour region lands
on its own geometry: the blue banner at the top of the box, the magenta panel
below it, the red flower and green leaves centred, the yellow flowers at the
lower right, and the text blocks legible and in place. Nothing is displaced
sideways and no background bleeds onto the object, which is the failure the
24 mm colour-camera offset would have produced had the parallax been wrong.

A flat, unsaturated scene cannot test this: the sampled image looks plausible
whether the offset is handled correctly, wrongly, or not at all. Any future
change to the colour path needs a chromatic object in view to be meaningful.

**Still open:** whether colour survives the TSDF averaging into the finished
mesh (Complete), which is a separate stage from this per-frame sampling.

## Blue laser projects but cannot scan (2026-10-16)

**Observed — the blue laser is visible but nearly useless to the depth
decoder as it stands.** Measured on hardware against an unchanged scene
(the infrared reference held at 27.8% across every run):

| mode | median valid depth, 20 frames | range |
|---|---|---|
| infrared | **27.8%** | 27.4-28.1 |
| blue laser, default exposure | 4.4% | 1.8-11.4 |
| blue laser, exposure 3000 | 3.0% | 2.2-10.5 |
| blue laser, exposure 4500 | 4.5% | 1.9-10.6 |

**Exposure is not the problem.** An earlier 3-frame-per-step sample suggested
3.7% -> 9.3% and that was sampling noise: per-frame yield in blue mode ranges
1.8-11.4% whatever the exposure, so three samples carry no signal. Twenty
frames per step show the exposures are indistinguishable. Nothing was shipped
on the strength of the small sample; the constant written for it was reverted.

**Tracking fails.** Replaying blue-laser frames through the normal pipeline
(`--replay-dumps`, blue calibration) tracked 4-19 of 45 frames across runs
versus 25 of 25 for infrared, with fused models of 61-12k points versus 123k.
The spread between identical runs is itself a finding: at ~4% coverage the
OpenMP floating-point reduction order in ICP is enough to flip the outcome,
so blue-laser tracking is not merely poor but irreproducible.

**Derived — the sub-image pair must be time-aligned.** In blue-laser mode
planes 1, 3, 6 and 8 go flat (local contrast ~150 against ~4600 in infrared)
while 0, 2, 5 and 7 carry the laser. Sweeping which pair is matched
(`ALL3DSCAN_POP4_PLANES=<left>,<right>`, diagnostic only) gives: default
(9,4) 2.93%, (7,2) 4.74%, (5,0) 3.03%, and the cross-slot pairs (7,4) and
(9,2) collapse to ~0.6%. So a pair must come from the same time slot in the
ten-image cycle, and (7,2) holds the laser better than the speckle default --
but 4.74% still does not track.

**Why, and what it would take.** The decoder is a dense speckle stereo matcher
(census + SGM): it needs texture everywhere. A line laser only illuminates
the lines, so per-frame coverage is bounded at a few percent by the pattern
itself, not by tuning. Making the blue laser scan needs a different depth
path: sub-pixel line-centre detection in both images, matching line segments
along epipolar lines, triangulating those, and accumulating across a sweep.
That is a new subsystem, not a parameter change. **Unresolved.**

## Speckle emitter search, and the laser-line off path (2026-10-16)

Toolchain: REA + Ghidra 12.1.4 are now wired up (see CLAUDE.md), so vendor
functions are read as decompiler output instead of hand-read AArch64.
Decompiled code remains "what the app sends", never "what the scanner does".

**Observed — CS7 `irLum` does not drive the speckle emitter.** Swept the CS7
settings field to 0, 255 and back to 130 while streaming
(`--pop4-cs7-test`); speckle contrast in the depth images stayed flat
(img4 local contrast 8287 / 8270 / 8270 / 8281, img9 similar). Combined with
the earlier sweeps (0xb04, 0xb01, 0xb07, 0x48d, 0xb12 — all no effect on
images 4/9), every emitter control we are willing to send is now excluded.
The speckle emitter appears firmware-driven, tied to streaming itself.
**Unresolved.** Next safe avenue: map the XU property channel
(`PROPERTY_EXT_*` names in the SDK strings) with the decompiler and see
whether any property is POP4-gated emitter control; nothing gets sent until
a gated write path is identified.

**Derived — the vendor's laser-line OFF sequence.** Ghidra decompilation of
the laser-line mode function (file offset 0x635234) shows the off path sends
the mode register (`0x3001 7`) *followed by* the group command with 0
(`0x3010 0x17 0`) — and that an optional precondition command (fetched from
another member; empty on our unit path) runs first and must succeed. Our
hardware test only ever sent the group-0 half alone, which left the laser
on. The full pair is untested on hardware; if it works it could replace the
45 s reboot when leaving blue-laser mode. The function also records the
on/off state at object offset 0x4cd and resets stream bookkeeping after the
switch.

## Lights, projector and blue-laser speckle mode (2026-10-14)

All of this was measured on a live POP4 (`--pop4-lights-test`,
`--pop4-mode-test`), not just read out of the SDK. "Effect" is the change in
local pattern contrast (std of image minus its 5x5 box mean) of the packed
sub-images; images 4 and 9 are the ones the depth decoder matches on.

| What | Command (XU4 selector 7) | Measured effect |
|---|---|---|
| Pattern projector on/off | `echo s 0xb04 1\|0 >/dev/rk_preisp` | Pattern contrast in images 0–3 / 5–8 collapses 3957 -> 517 when off; **images 4 and 9 do not change at all**. Infrared: toggling it is invisible to the colour camera (B/R 1.025 vs 1.022). On after arming. |
| Fill light on | `0xb03 1`, `0xb02 1`, `0xb12 255` | Scene brightness +10% in all IR images; colour camera blue-shifts (B/R 1.04 -> 1.30). White/blue LEDs for the texture camera. |
| Fill light off | `0xb03 0` | Returns to ambient. |
| Fill light level | `0xb12 <0..255>` | Monotonic; 60 vs 255 clearly differs. |
| Projector ("laser") level | `0x48d <0..255>` | **No measurable effect** in the default mode at any of 40/128/255. |
| IR LED rail | `0xb01 1\|0`, level `0xb07` | Accepted, **no measurable effect** on a POP4. |

Source for the command forms: `cs::Camera::ledAndLaserCtrl`
(`lib3DCamera.so` @0x630fd4) dispatches on (ledid, ctrltype): ledid 0 = IR
LED, 1 = RGB LED, 2 = laser; ctrltype 0/2 = on, 3 = off, 4 = luminance. The
0xb12 255 in the fill-light sequence is the POP_4 / POP_4_PRO branch
(@0x631884 tests camera type 0x20 / 0x21).

**The depth images are not lit by the switchable projector.** Images 4 and 9
keep their speckle whatever `0xb04` does, so depth yield is unchanged by it
(9.03% vs 9.13% valid). There is a second, always-on speckle emitter.

### The visible blue laser (2026-10-14, confirmed)

The POP4 really does have a **visible blue laser**, and it is a different
thing from speckle mode. It is the vendor's "laser line" path
(`lib3DCamera.so` @0x635234, gated to camera types that include 0x20 / 0x21):

```
echo s 0x3001 7 > /dev/rk_preisp        # laser-line mode register
echo s 0x3010 0x17 1 > /dev/rk_preisp   # "enter laser line group"
echo s 0xb04 1 > /dev/rk_preisp         # laser rail on (ledAndLaserCtrl ledid 2)
echo s 0x48d 180 > /dev/rk_preisp       # laserLuminance, from the POP4 presets
```

Measured with the scanner's own colour camera: **0.00% strongly-blue pixels
before, 24-27% during** — bright blue laser lines across the scene, plainly
visible to the eye too. The function takes an on/off flag; off also sends
`echo s 0x3010 0x17 0`, which on its own does **not** extinguish the laser.
Returning to infrared is the reboot path below.

**Why an earlier attempt looked dead:** `echo s 0x3001 7` was used on its own
as "back to infrared". That is only the first half of this sequence — it
selects laser-line mode and leaves every emitter off, so all images collapse
to noise. The missing half is the enter-laser-line-group command.

### Blue-laser speckle mode

`cs::Camera::setSpeckleMode` (@0x63725c) is gated to camera types **0x20 /
0x21 — POP_4 and POP_4_PRO only** — and sends:

```
echo s 0x3001 12 > /dev/rk_preisp     # enter speckle mode
echo s 0x48d 180 > /dev/rk_preisp     # laserLuminance from the POP4 presets
```

Measured: pattern contrast of the depth images jumps **6463 -> 10456 (+62%)**
and image 0 doubles (4006 -> 8551). Decoded depth yields **+21 to +30% more
valid points** (9.7% -> 11.9% of frame). The scanner carries a matching
second factory calibration at `/data/camparam_blue/camparam.yaml` (same
`deviceID`, calibrated 12 s after the infrared one, different focal lengths
and distortion — the two light sources have different wavelengths). The app
prefetches it at connect and caches it as `<deviceID>-blue.yaml`.

`resources/param_pop4.json` agrees: `laserLuminance` 180 appears only under
accuracy modes `speckle` and `speckle_stripe` (exposure 200–1400 us, gain
2–6), never under `surface` (exposure 3000–5400, what we use by default);
`fillLightLuminance` 128 appears only under the `marker` presets.

**Leaving speckle mode needs a reboot.** `echo s 0x3001 7` is the vendor's
*other* POP4 mode value (laser line, @0x635410); sending it leaves the
projector dead — all images drop to noise (contrast 460/300) — and re-arming
with CS10+CS7 does not revive it. Only a USB reset does. `setScanMode` back
to infrared therefore does reset + re-arm + reopen, which takes ~40 s and is
run off the UI thread. A rebooting POP4 enumerates, drops and re-enumerates
several times before settling, so the reset path retries the arm until it
sticks rather than opening once.

**The XU command channel is not ready the moment the scanner attaches.** For
the first seconds after connect (and after any re-enumeration) uvcvideo has
not finished building the chain that carries extension unit 4, and
`UVCIOC_CTRL_QUERY` answers `ENOENT` ("Extension unit 4 not found"), even
though the depth node is open and streaming. Probing the nodes directly shows
unit 4 appearing on the depth and RGB nodes a moment later (`GET_LEN` = 60).
Every command is an idempotent register write, so `sendDeviceCommands` retries
for up to 5 s instead of failing. Without that, a mode or light change clicked
right after connecting silently failed.

Not found: how to select the vendor's `surface` / `speckle_stripe` accuracy
modes as such, and what the always-on speckle emitter's controls are.
`LASER_SPECKLE_MODE` in `libcameraSdk.so` is gated to camera type 0x19
(TRACER_R1), not POP4.

## Device command channel, fringe exposures and open questions (2026-10-13)

**Corrections to the 2026-10-12 section below.** (1) Exposures 0–3 / 5–8
*do* carry fringes in normal operation. (2) Selector 7 is a general command
channel, not only a settings block.

**XU selector 7 = device shell command channel.** Static analysis of the
RevoScan 6.0.2.4 Android `lib3DCamera.so` (read-only: capstone + pyelftools,
scripts in the session notes):

- `libuvcExecuteCmd(popVersion, devh, cmd)` at `0x72cdf4` copies an ASCII
  command (≤60 bytes, NUL-padded) into a 60-byte buffer. When
  `popVersion > 9` it sends `SET_CUR` (0x21/0x01) with wValue `0x0700`
  (selector 7), wIndex `0x0400` (unit 4, interface 0), and a 1000 ms
  timeout. Older units use a vendor request (0x41/0xff, wIndex 0x0300).
  Success means all 60 bytes were transferred.
- The commands are shell lines run on the scanner's Linux (Rockchip). For
  example, `cs::Camera::setFringePattern` (`0x62f36c`) pauses the depth
  stream, sends `echo s 0xa05 <type> >/dev/rk_preisp` (or to
  `rk_preisp_1` and then `rk_preisp_2` on dual-ISP device types), then
  resumes. `<type>` is validated to 0..6. Java enum `FringePatternType`:
  STANDARD, STANDARD_WHITE_ADDED, MULTI_DEPTH, MULTI_DEPTH_WHITE_ADDED,
  MULTI_DEPTH_DOUBLE_WHITE_ADDED, 3FREQ4STEP_MULTI, 3FREQ4STEP_MULTI_DEPTH.
  Many other `echo s 0x… >/dev/rk_preisp` commands exist (laser line mode
  0x102–0x104, 0x3001, 0x48x, …); they are not used here.
- Our POP4 accepted `echo s 0xa05 1 >/dev/rk_preisp` (60 bytes). It is
  available as an opt-in experiment: `ALL3DSCAN_POP4_FRINGE=<0..6>`,
  `ALL3DSCAN_POP4_PREISP=single|dual|all`. Only the validated fringe
  command is ever sent. **Do not send other commands blindly**: the SDK
  also has firmware, remount and `setlaserCalParam` paths that could damage
  the device's state or calibration.
- Our binary projector "settings" block is also written to selector 7. How
  the firmware tells it apart from a text command is unknown.

**Fringe exposures.** From about 17 frames (~1.1 s) after streaming starts,
exposures 0–3 (right camera) and 5–8 (left camera) show one-frequency
four-step fringes: vertical stripes, period ≈5.3 px, phase steps −90°
(measured by FFT). The first capture studied had 12 frames, all inside
that warm-up, which is why the section below wrongly calls them ambient.

**The speckle and fringe images are not in the same geometry.** On a static
scene, each block's speckle image is shifted ≈−13 px (left camera) /
+12 px (right camera) horizontally from that block's fringe images, so the
fringe pair's disparity is ≈ speckle + 25 px (~60 px at 30 cm). The fringe
period is nearly constant across x in both cameras, which the bowl-shaped
speckle disparity cannot explain. So the "lens bias" applies to the speckle
pair, not to the fringe pair.

**Phase refinement has not worked yet.** Matching equal phase near the
speckle coarse disparity, in speckle geometry and in fringe-native geometry
(with the offsets above), gave residuals spread over a whole period rather
than a sharp peak. On a static scene the phase at a fixed pixel also jitters
±20° (~0.3 px) from frame to frame. Open questions: the exact fringe-pair
rectification, inter-camera exposure timing, and whether the phase is a
shared projector code. Until these are answered, depth stays speckle-only.

**Environment note.** A long-running Wine/Proton `winedevice.exe` had the
scanner's USB node open. Wine's USB layer can interfere with libusb access
and may explain some stuck-arming episodes.

## Packed frame is speckle stereo, not four-step phase (2026-10-12)

**This supersedes the frame-layout and decoding claims in the 2026-07-18,
2026-07-19 and 2026-07-21 sections below.** Those sections are kept for
history. The phase knobs (`_MAXCOST`, `_MARGIN`, `_LRCHECK`,
`_SPECKLE_SUPPORT`, `_STEREO_BAND`) are no longer used by the decoder.

Evidence: 12 raw frames dumped from a live POP4 (box ~300 mm away,
`ALL3DSCAN_POP4_DUMP_DIR`), analysed offline.

- **Layout.** The 640×2560 Y16 buffer is **ten 320×512 images** with row
  stride 320, not five 640×512 planes. Autocorrelation of the flat buffer
  peaks at a 320-sample period (0.853, with no peak at 640). Reading it as
  640-wide puts image row 2k in the "left half" and row 2k+1 in the "right
  half", which is why the old "L|R" halves matched at zero disparity.
- **Cameras.** Images 0–4 come from the right IR camera and 5–9 from the left.
- **Content.** In each group, images 0–3 (5–8) are ambient/flood exposures
  (mean ≈17/255) with **no fringe modulation**: the four-step phase computed
  from them is uniform noise. Image 4 (9) carries the projected **IR speckle**
  (mean ≈54 / 38). The layout was stable across all frames.
- **Not row-aligned.** A dense block-match field between the two speckle
  images shows the vertical offset sweeping from −10 px (left edge) to +10 px
  (right edge). The right camera is rolled and keystoned, so plain row-wise
  stereo only matches in a narrow central strip. A quadratic row map (6
  coefficients) fits it with 0.15–0.17 px residual over ~400 correspondences;
  see `tools/re_probe/pop4_rectify_fit.py`.
- **Disparity.** After alignment, the flat box face reads 33–38 px, stable
  frame to frame.
- **Horizontal lens bias.** On flat surfaces the matched disparity bows up
  toward the left/right edges: + ≈24·u² px, where u = (x−160)/160. Two
  independent planar scenes (the box face and a carton face, different poses)
  gave u² = 24.22 and 24.66. Correcting each with the *other's* coefficients
  dropped the plane-fit residual from 2.9–3.3 px to 0.37–0.39 px. This is a
  camera property (differential distortion along the baseline), not scene
  shape. Uncorrected, every flat surface comes out bowl-shaped, fusion smears
  into streaky fans, and edge pixels fall outside the preset's disparity band.
  The decoder searches the band widened by the bias range and subtracts
  `24.44·u² − 0.745·uv + 0.06·v²` (`ALL3DSCAN_POP4_DISPX` overrides; `0,0,0`
  disables). Linear terms can't be separated from surface tilt and are not
  modelled.

Decoder (`Pop4DepthDecoder`; CUDA when available, else CPU + OpenMP):

1. Left = image 9, right = image 4 (`ALL3DSCAN_POP4_SWAP_LR=1` swaps them).
2. Resample the right image onto left rows with the quadratic row map
   (`ALL3DSCAN_POP4_RECT=c0,…,c5` overrides the built-in fit).
3. [1 4 6 4 1] blur, 7×7 census, Hamming cost.
4. 4-direction semi-global aggregation (`_P1`=16, `_P2`=120).
5. Winner-take-all with a uniqueness gate (`_UNIQ`=0.05), parabolic sub-pixel
   refinement, left-right check (`_LRDIFF`=1), and a texture gate on the left
   speckle (`_TEXTURE`=1.5).
6. 3×3 median, then speckle-region removal (`_SPECKLE_REGION`=80,
   `_SPECKLE_DIFF`=1).
7. Output 320×512 depth plus grey texture taken from the left ambient images.

On the dumped frames this gives about 50k valid depth samples per frame
(31% of the image, and most of the speckle-lit box face). The textureless
floor is rejected rather than filled with noise. The old decoder produced
about 5k samples per frame, and they were noise.

**GPU path.** `Pop4StereoCuda.cu` ports steps 2–5 (alignment through
winner-take-all) and produces the same disparity as the CPU matcher. It is
built with `--fmad=false`, uses integer SGM, and puts one warp per scan line
with no block barriers. A unit test checks GPU output against CPU, and the two
real dumps were byte-identical. On an RTX 5060 (sm_120, CUDA 13.4) the GPU
match takes ~2.2 ms (SGM 1.0 ms). The full decode takes ~9 ms, against ~32 ms
CPU-only; the rest is CPU clean-up (median, speckle regions, texture).
`ALL3DSCAN_POP4_CUDA=0` forces the CPU path. `ALL3DSCAN_POP4_DEBUG=1` prints
per-stage GPU timings. A device error falls back to the CPU for the rest of
the process.

```sh
cmake -S . -B build -DCMAKE_CUDA_COMPILER=/opt/cuda/bin/nvcc   # arch defaults to native
```

Calibration is still approximate: fx comes from a 46° FOV across 320 px, and
the **provisional baseline is 28 mm**, chosen so the measured ~35 px maps to
roughly 300 mm. Verify it with a ruler and correct it with
`ALL3DSCAN_POP4_BASELINE_MM` (Z ∝ B). Each unit likely needs its own row-map
fit:

```sh
ALL3DSCAN_POP4_DUMP_DIR=/tmp/pop4 all3dscan --probe-pop4   # dumps ~30 frames
python3 tools/re_probe/pop4_rectify_fit.py /tmp/pop4       # prints ALL3DSCAN_POP4_RECT=…
all3dscan --decode-dump /tmp/pop4/pop4_640x2560_000.y16 disp.pgm  # offline check
```

Still open: factory camparam (true fx, baseline, and full rectification
including the x direction), the role of the four ambient exposures (possibly
AE or background subtraction), and the blue-laser mode.

## Native C++ milestone (2026-07-18)

All3DScan now contains a native `RevopointPop4Protocol` backend:

- opens the confirmed `2207:110c` device through libusb;
- detaches `uvcvideo` from depth VideoControl interface 0;
- sends XU unit 4 CS10 (`01 00 00 00`) and the 60-byte CS7 settings payload;
- releases the interface and reattaches `uvcvideo`;
- discovers the matching V4L2 capture node through sysfs/VID/PID and advertised
  format instead of assuming `/dev/video0`;
- captures **640×2560** packed Y16 when advertised (five stacked 640×512
  planes), with **640×512** as fallback;
- runs an approximate full-field decoder (`Pop4DepthDecoder`): four-step phase
  → row phase match → Q-based mm using FOV/baseline defaults marked
  `ApproximatePop4FullField`.

The native smoke test:

```sh
./build/all3dscan --probe-pop4
```

reports packed-plane nonzero counts and whether metric decode succeeded.
Factory `camparam.yaml` / true `depthScale` (`PROPERTY_EXT_DEPTH_SCALE=0x0918`)
are still required for absolute millimetre accuracy — the shipped path is
explicitly approximate.

For physical target calibration before factory parameters are available, the
assumed 55 mm stereo baseline can be overridden without rebuilding:

```sh
ALL3DSCAN_POP4_BASELINE_MM=55 ./build/all3dscan --probe-pop4
```

Measure a planar target at a known distance and adjust this value
proportionally (`newBaseline = oldBaseline × knownDistance / measuredDistance`).
Accepted values are 20–120 mm.

### CUDA stereo decode (optional)

When built with `ALL3DSCAN_WITH_CUDA=ON` (default if a CUDA toolkit is found),
`Pop4DepthDecoder` runs phase + disparity matching on the GPU and falls back to
CPU on failure. Force CPU with `ALL3DSCAN_POP4_CUDA=0`. Rebuild:

```sh
cmake -S . -B build -DALL3DSCAN_WITH_CUDA=ON
cmake --build build -j
```

CUDA speeds matching; it does **not** replace factory `camparam` / Q accuracy.
The status string reports `CUDA` or `CPU` in the POP4 backend name.

### Robust disparity — fixing the "green blob / slab" (2026-07-21)

Symptom: live POP4 scans ran (CUDA active, depth view plausible) but the fused
3D result was a dense green cube/blob instead of the real object (a handheld
fan).

Root cause: the four-step phase is a **single-frequency wrapped phase** (0..2π),
which is *periodic* across the field. The winner-take-all disparity search
matched wrapped phase per pixel with only a weak margin gate, so the depth
clamp (210–500 mm → disparity ≈ 42–99 px, narrower than one fringe period)
left essentially one surviving periodic candidate per pixel at a nearly
constant disparity. That back-projects to a **fronto-parallel slab**; ICP then
fuses slabs from every sweep pose into a filled cube. Foreground isolation also
kept the background plane, and no median/speckle cleanup removed wrong-fringe
matches.

Fixes (mirrored in `Pop4DepthDecoder.cpp` **decodeCpu** and
`Pop4StereoCuda.cu`, driven by shared `DecodeParams`):

- **Left↔right consistency**: match L→R and R→L, keep only mutually consistent
  disparities (`|dispL − dispR| ≤ lrMaxDiff`). Kills wrong-fringe matches.
- **Uniqueness**: additive margin **and** a ratio test against the best rival at
  a *different* fringe period.
- **3×3 disparity median** + **5×5 local-support speckle removal** before the
  disparity→depth conversion.
- **Foreground isolation**: RANSAC removal of the dominant *fronto-parallel*
  plane (`isolateForegroundObject`) before nearest-cluster selection, so the
  object relief stands out of the slab. Guarded so it never empties the cloud
  and only fires on large, near-perpendicular planes.

Source label stays `ApproximatePop4FullField` (no factory-accuracy claims).

Tuning env knobs (all optional; sensible defaults on):

```sh
ALL3DSCAN_POP4_LRCHECK=1      # left-right consistency (default on)
ALL3DSCAN_POP4_UNIQ=0.75      # best < UNIQ*rival ratio test
ALL3DSCAN_POP4_MARGIN=0.03    # additive cost margin
ALL3DSCAN_POP4_MEDIAN=1       # 3x3 disparity median
ALL3DSCAN_POP4_SPECKLE=1      # local-support speckle removal
ALL3DSCAN_POP4_SPECKLE_SUPPORT=6
ALL3DSCAN_POP4_MAXCOST=0.15   # phase-agreement gate
ALL3DSCAN_POP4_DEBUG=1        # stderr disparity/valid% histogram (every 15 frames)
```

Existing knobs (`ALL3DSCAN_POP4_CUDA`, `_SWAP_LR`, `_STEREO_BAND`,
`_BASELINE_MM`) are unchanged. If a scan is still blobby, first try
`ALL3DSCAN_POP4_DEBUG=1` to inspect disparity spread; a near-zero std means the
phase is still collapsing (raise `_UNIQ` toward 0.5, raise `_MARGIN`), while too
few valid points means over-filtering (lower `_SPECKLE_SUPPORT`, or set
`_LRCHECK=0`).

## Packed phase-stream milestone (2026-07-19)

Live captures of every advertised Y16 size corrected the earlier assumption
that `640x512` was a useful complete scan frame:

- `640x512` was partial/sparse in the tested state (nonzero data only in rows
  0-255).
- `640x1024` populated one 640x512 half in the tested state.
- `640x2560` produced five complete 640x512 temporal planes.
- Each 640x512 plane contains two 320x512 views side-by-side. The scene is
  visibly duplicated once per stereo camera.

ARM64 analysis of APK `libRPASpeckleStereo.so` confirms the matching decoder:

1. Split the first four temporal planes into left and right image vectors.
2. For each camera compute `x = I0-I2`, `y = I1-I3`, and wrapped phase
   `atan2(y,x)`; reject modulation below 3.
3. Rectify both phase maps with per-device stereo calibration.
4. Match phase along epipolar rows with subpixel refinement to obtain uint16
   disparity.
5. Map disparity through a 65,536-entry LUT derived from the OpenCV stereo
   `Q` matrix and the SDK depth scale.

The fifth temporal plane is auxiliary intensity/confidence data; its exact
role is not yet confirmed. `tools/re_probe/pop4_y16_analyze.py` implements
the confirmed splitting and four-step wrapped-phase stage. A physical fixture
produced valid phase for roughly 61% of the combined stereo pixels.

**Native status (2026-07-19):** `RevopointPop4Protocol` + `Pop4DepthDecoder`
now implement steps 1–2 and an approximate stand-in for 4–5 (FOV-based
intrinsics, ~55 mm baseline, `Z = fx·B / disparity`, clamped 210–500 mm).
Step 3 (factory rectification / `camparam`) remains open. Do not treat
approximate mm as factory-accurate until Q/`0x0918` are recovered from the
device.

APK static analysis (2026-07-19) resolved the depth-scale **property ID** to
`0x0918` (`PROPERTY_EXT_DEPTH_SCALE`) and confirmed a POP4 SDK fallback of
`0.01f` when that read fails. The XU framing that carries `0x0918` is still
unknown — see [`APK_REVOSCAN_6.0.2.4.md`](APK_REVOSCAN_6.0.2.4.md). Do not send
`0x0918` as a raw control selector.

## TSDF live-fusion Complete path (2026-07-19)

`Complete` now prefers **volumetric TSDF reconstruction** over a single Poisson
pass on the fused point cloud — this is closer to how RevoScan builds its live
model (frame-to-model tracking + volumetric integration) than accumulating a
cloud and Poisson-meshing it once.

Data flow:

1. `ScannerController` retains the full metric depth map + intrinsics of each
   metric frame (`lastMetricDepth()` / `lastIntrinsics()`), not just the
   isolated cloud.
2. `MainWindow::onMetricFrame` forwards these via
   `ScanPipeline::submitMetricFrame`.
3. The pipeline's ICP worker recovers each frame's camera→world pose (unchanged
   live path) and, subject to a motion gate (≥8 mm or ≥~3°, capped at 160
   frames), snapshots a `DepthKeyframe` `{depthMm, intrinsics, pose}`.
4. On `Complete`, if ≥4 keyframes exist, the pipeline writes raw uint16 depth
   bins + a JSON manifest to a temp dir and calls the new `tsdf` worker command.
   `mesh_worker.py::tsdf_reconstruct` integrates each frame into an Open3D
   `ScalableTSDFVolume` (`extrinsic = inv(pose)`, depth cropped to the current
   working-distance band, `depth_scale = 1000`) and extracts a watertight mesh.
   The mesh is written as **ASCII PLY** so the lightweight C++ parser can read
   it (Poisson/smooth writes are now ASCII too).
5. Any TSDF failure (no Open3D, empty volume, too few keyframes) falls back to
   the previous Poisson path; the TSDF reason is surfaced in the error string.

Caveats until factory rectification lands: keyframe poses come from approximate
metric depth, so the TSDF surface inherits the same scale uncertainty as the
decoder. Integration uses the full depth map (not the isolated cloud) cropped to
the isolation band, so background inside the band can leak in — crop after
Complete if needed.

## Registered IR-albedo texture (2026-07-19)

The live model and TSDF mesh are now **textured** instead of untextured/gray.
The four-step phase planes carry, per left-view pixel, a DC term
`A = (I0+I1+I2+I3)/4` — the surface's IR albedo under the projected pattern.
Because it comes from the *same* left camera pixels as the decoded depth, it is
inherently registered with zero depth↔RGB extrinsics:

- `Pop4DepthDecoder::decode` fills `RawFrame.rgb` (320×512×3) with this DC term
  as grayscale (mild gain `kTextureGain`). `pointcloud::fromDepthFrame` already
  maps `rgb` onto point colors, so the live cloud is shaded immediately.
- `ScanPipeline` retains the texture per `DepthKeyframe` and writes a `.rgb`
  companion per keyframe; `mesh_worker.py::tsdf_reconstruct` loads it as the
  Open3D color image so `extract_triangle_mesh` returns a colored surface
  (ASCII-PLY colors parsed back into `TriangleMesh`).

This is **grayscale IR texture**, not the POP4 RGB camera. True color from the
separate visible-light sensor is a larger follow-up: it needs (a) capturing the
RGB UVC node on Linux and (b) a depth→RGB extrinsic + intrinsics to project
depth points into the color image. Both are still open (no factory `camparam`).

## Live tracking / fusion improvements (2026-07-19)

Real-time scanning was tightened to feel closer to RevoScan's live model:

- **ICP recovery**: tight pass → wide correspondence (~50 mm) around the last
  pose → identity relocalization when the user returns to a known view.
- **Reject noisy poses**: fitness floor + RMSE gate so bad frames never corrupt
  the model; `lastPose_` is preserved across Lost frames as a prediction.
- **Denser live model**: ICP on a 5 mm cloud for speed; fuse a 2.5 mm downsample
  of the full frame into the document.
- **Coverage tint**: well-aligned frames tint newly fused voxels green (newest
  sample wins in the voxel grid) so freshly covered surface pops like RevoScan.
- **HUD**: tracking label shows fitness %, model point count, and TSDF keyframe
  count (`KF`).

Still open for live feel: async Complete with ETA, and factory calibration
for less noisy depth.

## Live shaded mesh preview (2026-07-19)

During scanning with **Live Fusion** + **Live Mesh** (default on), the Scan
viewport shows a growing **shaded triangle mesh** instead of only points:

1. Each tracked metric depth frame is converted to an organized depth mesh
   (`geometry::meshFromOrganizedDepth`, stride 2, discontinuity reject).
2. Vertices are transformed by the live ICP pose and voxel-welded into an
   accumulating preview (`mergeMeshVoxel`, ~4 mm, capped ~180k verts).
3. Updates are throttled (~8 Hz) and rendered with the existing mesh shader
   (clay when Color is off, IR texture when on).

This is a fast preview surface — Complete still runs TSDF/Poisson for the
final Model mesh. Toggle **Live Mesh** off to fall back to the point cloud.

## Model stage tabs (2026-07-19)

The Edit/Model page now mirrors Revo Scan's post-scan workspace with
`Raw / Point Clouds / Meshed / Textured` stage tabs over one viewport:

- **Raw** — fused metric cloud, color off (plain geometry).
- **Point Clouds** — fused cloud with IR-albedo color.
- **Meshed** — the TSDF/Poisson surface as a shaded clay mesh.
- **Textured** — the same surface with its per-vertex IR texture.

`PointCloudView` gained a triangle-mesh renderer (indexed draw, per-vertex
normals, camera-headlight shading) alongside the point renderer; selection
editing stays on the two cloud stages. `Complete` lands on the richest available
stage (Textured if the mesh has colors, else Meshed). Not yet matched from the
reference screen: an async "Processing… (ETA)" dialog (Complete still runs
synchronously) and a separate re-run "Fusion" action.

## POP4 “AI-powered” parity (host vs device)

Marketing for POP4 as an **AI-powered scanner** refers almost entirely to
**host-side RevoScan / Revo AI features**, not to a neural net inside the
USB depth path. The sensor still delivers structured-light / stereo phase
data; AI layers improve registration, isolation, and mesh cleanup after
(or beside) metric depth.

| Layer | Where it runs | RevoScan role | All3DScan today | Native candidate |
| --- | --- | --- | --- | --- |
| Four-step phase → disparity → depth | Device + host decoder | Core depth | `Pop4DepthDecoder` (approx mm) | Factory `camparam` / Q / `0x0918` |
| Marker-free tracking | Host | Revo AI tracking | ICP live fusion (`ScanPipeline` / `Registration`) | Learned registration / loop closure |
| Volumetric fusion | Host | Live model integration | TSDF on Complete (`mesh_worker tsdf`), Poisson fallback | Real-time incremental TSDF |
| Surface texture | Host | RGB-camera color | Registered IR-albedo grayscale (decoder DC term) | RGB UVC capture + depth→RGB extrinsic |
| Object vs background | Host | AI segmentation | Heuristic `isolateForegroundObject` | Lightweight segmentation model |
| Mesh cleanup / hole-fill | Host post | AI mesh tools | Mesh worker + Edit panel | Learned denoise / hole-fill pass |

**Implications for this project:**

1. Closing the **depth accuracy** gap (factory stereo calib + true depth
   scale) remains the priority for exportable millimetres.
2. Closing the **AI UX** gap is optional and separable: ship geometric
   equivalents first, then swap in learned models without changing USB
   arming or the packed Y16 layout.
3. Do **not** ship proprietary Revo AI model weights or `lib3DCamera.so`.
   Any learned parity must use open weights or models we train/license
   ourselves.

**Suggested order if we pursue AI parity later:**

1. Learned **segmentation** (biggest visible win vs table/background).
2. Learned **frame registration / loop closure** (less drift on flat /
   featureless objects).
3. Learned **mesh denoise / hole-fill** after Complete.

Gaussian splat / full Revo AI reconstruction stay out of scope until
metric depth and basic AI parity are solid.

## Capture-backed protocol import (2026-07-19)

Official vendor control sequences (mode, lighting, blue laser, calibration
reads, emitter-off) are **not yet confirmed by USBPcap**. Until captures land:

- Linux arming remains the empirical CS10 + CS7 sequence above.
- Metric decode, lighting UI, and blue-laser mode stay disabled / raw-preview.
- Do **not** invent selector names or off commands here.

Capture procedure and import pipeline:

- Checklist: [`USBCAP_CHECKLIST.md`](USBCAP_CHECKLIST.md)
- Windows PCAPNG → TShark JSON → `tools/re_probe/pcapng_import.py`
- Dry-run / live replay: `tools/re_probe/xu_probe.py --sequence … [--dry-run]`

Only capture-backed CS payloads and a verified projector/laser-off sequence
may be promoted into `RevopointPop4Protocol` and this notes file.

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
- PlusToolkit `PlusLib` has a `Revopoint3DCamera` device
  (<https://pluslib.readthedocs.io/en/latest/devices/DeviceRevopointSurface.html>)
  that also runs on Linux (Ubuntu ≥18.04) **only via the closed Revopoint
  3D Camera SDK** (tested SDK 2.4.0). Not a clean-room decoder, but it
  corroborates several facts: the SDK exposes both `DEPTH` and `PCL`
  (x-y-z point cloud) frame types (depth→cloud is host/SDK-side, matching
  our `fromDepthFrame`); an example stream config uses `FrameWidth=1920`,
  `FrameRate=8` — consistent with the advertised **`1920×512`** mode
  alongside our packed `640×2560`; and the SDK-wide default `DepthRange` is
  `5 500` mm (generic default, not POP4-specific — our 210–500 clamp still
  matches POP4's working band). It also references a udev-rules repo
  (`latimagine/PlusHelpers`) worth comparing to
  `resources/70-all3dscan-scanners.rules` if permission gaps appear.
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
Update `resources/devices.json` and `resources/70-all3dscan-scanners.rules`
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

## Windows-app analysis removed (2026-10-16)

Sections derived from the Revopoint Windows application were removed at the
repository owner's request. Do not add findings from that application to
this repository. Protocol facts used by the code are recorded in the
device-measured and Android-SDK sections above and below.

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

### Windows-app notes removed (2026-10-16)

Material derived from the Revopoint Windows application was removed at the
repository owner's request. Do not add findings from that application to
this repository.

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
- Capture a real vendor USB session (`usbmon`) to confirm whether
  production uses CS7 or HID-wrapped `EXECUTE_CMD` / `WRITE_READ_HID`.
- Map `PROPERTY_EXT_*` numeric IDs via disassembly or a logged
  `type=%04X` session — strings alone are insufficient.

## Step 2: Capture USB traffic

Preferred: run the official vendor app while capturing:
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
