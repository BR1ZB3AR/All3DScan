# Revopoint POP4 USB Protocol — Reverse Engineering Notes

Status: **Phase B in progress.** Device identified and enumerated
2026-07-16 with a physical POP4 connected. Key finding: the POP4 is a
**UVC/HID composite device**, not a proprietary bulk-transfer device — see
"Step 1" and "Step 3" below. This changes the implementation plan (see
bottom of this doc).

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

Official Revo Scan 6 control sequences (mode, lighting, blue laser, calibration
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
