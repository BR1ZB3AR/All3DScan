# Changelog

All notable changes to All3DScan are logged here, newest first. Versions
follow `Alpha-vYYYYMMDD-HHMM` (local time at the point of the change) — this
project is pre-release reverse-engineering work, not semver.

## Alpha-v20261014-2219

### Added
- **The scanner's built-in lights are now controllable**, using the
  scanner's own commands (recovered from the RevoScan Android SDK and
  checked against the hardware — see `docs/protocol/RE_NOTES.md`):
  - **Pattern projector** on/off. This is the infrared projector that draws
    the scan pattern; switching it off makes the pattern vanish from the
    camera images.
  - **Fill light** on/off with a brightness slider. These are the white LEDs
    for the colour camera; RevoScan turns them on for marker scanning.
  - "Emitters Off" now really turns both off.
- **Blue laser speckle mode** (Scan Mode -> "Blue laser speckle"). The POP4
  has a second, stronger projector with its own factory calibration on the
  device (`/data/camparam_blue`), which the app now reads and caches. In
  this mode the projected pattern is **62% stronger** and the scanner
  returns **about 20-30% more depth points** per frame.
  - Switching back to infrared reboots the scanner (~40 s) — that is the
    only way back that works. It runs in the background, so the app stays
    responsive, and the controls are disabled while it happens.
- `all3dscan --pop4-lights-test <dir>` and `--pop4-mode-test <dir>` record
  what each light and mode actually does, for future work.

### Fixed
- **Connect failed if you pressed it too soon after the previous session.**
  The scanner drops off USB and re-enumerates after every session, and its
  permissions land a moment later still; the app now waits for it instead of
  giving up. Reconnecting after a USB reset is also retried until the
  scanner settles, rather than once.
- The controls that were never confirmed against the hardware (alignment
  light, illumination and laser sliders) are hidden instead of looking
  functional.

## Alpha-v20261013-2327

### Added
- **Automatic alignment while scanning.**
  - Each frame starts from a predicted pose (the last motion, damped), then
    runs a coarse alignment and a fine one.
  - A frame is fused only if at least 30% of it lies within 4 mm of the
    scanned surface, with an error of 2.5 mm or less. The old check counted
    anything within 35 mm, so badly placed frames got fused and doubled
    surfaces.
  - **Auto re-align:** when tracking is lost, each new frame is tried
    against the poses of earlier keyframes. Move back over an area you
    already scanned and the scan continues by itself. Tested with simulated
    jumps of up to 72°, with no loss of accuracy.
  - The tracking bar says what to do when tracking is LOST and shows
    "(re-aligned)" when the scan snaps back in.
- Replay test options: `ALL3DSCAN_REPLAY_REALTIME=1` (feed frames at camera
  rate, as live) and `ALL3DSCAN_REPLAY_BLACKOUT=a,b,r` (blank frames a..b,
  then resume r frames back).

### Fixed
- **Tracking ran far slower than it should.** The decoder and the tracker
  each started a full set of CPU threads on the same cores, and the two
  stalled each other, making alignment about 20× slower. Each now uses half
  the cores. On a real recording played at camera speed, 367 of 400 frames
  are processed (167 before).
- Camera poses could slowly stop being rigid transforms. They are now kept
  rigid after every step.

## Alpha-v20261013-2242

### Added
- **New Scan** button (scan page and Edit page) and File → New Scan
  (Ctrl+N). It discards the current model, mesh, keyframes and live mesh,
  then returns to the scan page with the scanner still connected and the
  preview running. It asks for confirmation when there is a scan to lose.
- `all3dscan --screenshot <file.png>` renders the main window to an image.

### Fixed
- Cancel cleared the point model but left the pipeline's keyframes, live
  TSDF and live mesh behind: the old live mesh stayed on screen, and old
  keyframes leaked into the next Complete. It now resets the pipeline too.
- After Complete or Cancel stopped the camera stream, Scan did nothing until
  you reconnected. Scan now restarts the stream.

## Alpha-v20261013-2235

### Fixed
- **Calibration: the app now uses the scanner's own factory calibration.**
  It is read from the scanner on Connect (`/data/camparam/camparam.yaml`,
  over the vendor's read-only file channel) and cached per device. This
  replaces the guessed 28 mm baseline and the fitted rectification.
  - Real metric depth: 118.6 mm baseline, factory lens model.
  - Correct vertical scale: the images are x/4, y/2 of the 1280×1024
    sensors, so point clouds and meshes were stretched about 2× vertically
    before.
  - Flatter surfaces (19–30% less depth noise) and better tracking on real
    scans (351/400 frames, was 332).
  - CPU and CUDA decode stay byte-identical; without a factory file the
    previous behaviour is unchanged.

### Added
- `all3dscan --read-device-file <path> <out>` (read-only file transfer from
  the scanner) and `all3dscan --usb-reset`.
- `ALL3DSCAN_CAMPARAM=<camparam.yaml>` for offline decode and replay.

## Alpha-v20261013-2159

### Changed
- **New app logo.** It is used for the menu entry, the taskbar and window
  icon, the About dialog (with the live version number instead of a stale
  hard-coded one) and the README. It is installed as hicolor PNG icons
  (16-512 px), with a 1024 px master in `resources/branding/`. The old
  placeholder SVG and the generic "camera-photo" window icon are gone.

### Fixed
- Complete builds the mesh in the background instead of freezing the window.
- Data race between scan fusion and the UI that crashed the 3D view.

## Alpha-v20261013-2133

### Added
- Live 3D polygon model while scanning (see the live TSDF commit): Live Mesh
  is on by default and no longer needs Live Fusion.
- Opt-in `ALL3DSCAN_POP4_FRINGE=<0..6>` sends the vendor SDK's validated
  fringe-pattern command over the selector-7 command channel (found by
  static analysis of the RevoScan Android SDK).

### Changed
- RE notes: exposures 0–3 / 5–8 do contain four-step fringes after ~1 s of
  streaming (the earlier "no fringes" claim came from a capture inside the
  warm-up). They are in a different geometry from the speckle pair, and
  phase refinement is not working yet.

## Alpha-v20261012-2310

### Fixed
- **Flat surfaces decoded as bowls.** The POP4 pair has a horizontal lens bias:
  matched disparity on flat surfaces rises by ~24 px toward the left/right
  edges. The decoder now subtracts a fitted quadratic bias and widens the
  search band to cover it. Plane-fit residual on two real scenes went from
  ~3.9 px to 0.37 px, which removes the streaky "fan" artefacts in fused
  scans and the edge drop-outs at the 250–500 mm preset.
- **Connect froze the window.** Connecting (projector arming, uvcvideo
  rebinds, and a possible ~40 s reboot after a USB reset) now runs on a
  worker thread. The button shows "Connecting…" and the UI stays live.
- **USB reset storms.** The "no video node" fallback reset the scanner and
  retried after 1.5 s while it was still rebooting. Both reset paths now
  share one reset-and-wait helper, used at most once per connect.

## Alpha-v20261012-2229

### Added
- **CUDA speckle decoder.** `Pop4StereoCuda.cu` is rewritten for the speckle
  pipeline: row alignment, blur, census, cost volume, 4-path SGM (one warp
  per scan line, shuffle exchange, no block barriers) and winner-take-all with
  the same gates as the CPU. Output matches the CPU path exactly on real
  frames (built with `--fmad=false`; parity unit test). On an RTX 5060 the
  full decode takes ~9 ms, against ~32 ms CPU-only. The decoder uses CUDA
  automatically when available and falls back to the CPU on device errors;
  `ALL3DSCAN_POP4_CUDA=0` forces the CPU.
- `ALL3DSCAN_POP4_DEBUG=1` prints the backend, matching time and per-stage
  CUDA timings. `--decode-dump` reports steady-state and first-call times.

## Alpha-v20261012-2202

### Fixed
- **POP4 depth decoding rewritten — scans were noise.** The packed 640×2560 Y16
  frame is ten 320×512 images (images 0–4 from the right camera, 5–9 from the
  left), not five 640×512 planes with L|R halves. The images the old decoder
  ran four-step phase on contain no fringes, so it was matching noise, which
  gave "Depth lock ~6%" and blob-shaped scans. `Pop4DepthDecoder` now matches
  the two speckle images (9 = left, 4 = right) with census + 4-path SGM, a
  uniqueness gate, a left-right check, a texture gate, and speckle-region
  removal. It outputs 320×512 depth with ambient-IR texture. On live frames
  this gives ~50k valid samples per frame instead of ~5k noisy ones.
- **Right-camera row alignment.** The speckle pair is not row-aligned (row
  offset swings ±10 px across the frame). A fitted quadratic row map is
  applied before matching; override it with `ALL3DSCAN_POP4_RECT`.
- **L/R flipping between frames.** Calling `setCalibration()` on every frame
  reset the auto-swap cache, so the left/right choice was re-scored per frame
  and flipped. Auto-swap is removed; the camera order is now fixed.
- **Scanner access on Arch/Fedora.** The udev rule is now
  `70-all3dscan-scanners.rules` with `TAG+="uaccess"` instead of the
  Debian-only `plugdev` group.
- **Menu launcher.** The desktop entry gets `Icon=` and `StartupWMClass`, the
  icon is installed to hicolor, and `Exec` is an absolute path so `~/.local`
  installs launch from the KDE menu.

### Added
- `ALL3DSCAN_POP4_DUMP_DIR` dumps raw packed frames (`--probe-pop4` captures a
  burst of 30).
- `all3dscan --decode-dump <frame.y16> [disp.pgm]` decodes a dump offline.
- `tools/re_probe/pop4_rectify_fit.py` fits the per-unit row map.
- Decoder hot loops use OpenMP (113 → 63 ms per frame).

### Changed
- Provisional POP4 baseline is 28 mm (was 70 mm), scaled so the measured
  disparity maps to ~300 mm. Verify with a ruler; Z ∝ `ALL3DSCAN_POP4_BASELINE_MM`.
- The CUDA phase path is no longer called by the decoder; it needs a rewrite
  for the speckle pipeline.

## Alpha-v20260802-1557

### Fixed
- **Stereo Auto-Swap Startup Lock Fix**: Fixed a bug where `commitAutoSwap()` locked `autoSwapCache()` on frame 0 during startup when illumination/power was initializing:
  - Added a minimum valid point threshold (`validPoints >= 100`) before committing auto-swap orientation, preventing unilluminated startup frames from permanently caching the wrong L/R view order.
  - Added `resetAutoSwapCache()` inside `Pop4DepthDecoder::setCalibration()` to reset cached orientation whenever the scanner connects or updates calibration, allowing continuous accurate orientation scoring under active illumination.

## Alpha-v20260802-1550

### Fixed
- **Emitter Control Power Scaling & Shutter Fix**: Corrected raw default values in `ScannerControlState` ([ScannerProtocol.h](file:///home/br1zb3ar/Reverse-Engineer-All3DScan-On-Linux/src/protocol/ScannerProtocol.h#L102)):
  - Updated illumination, laser, and exposure to default to their proper 50% / 70% raw byte constants (`kIllumDefault = 130`, `kLaserDefault = 180`, `kExposureDefault = 3000us`).
  - Previously, setting small percentage values (e.g. 50) directly into 0..255 and 500..6000 hardware fields clamped exposure to 500us (minimum shutter time / 0% exposure) and laser/illumination to <10%, starving the camera of structured light.
  - Now on connect, the scanner emits bright structured light patterns at 50% illumination, 70% laser, and 3000us exposure.

## Alpha-v20260802-1538

### Fixed
- **Fringe Modulation Threshold Adjustment**: Lowered `kModulationMin` from `6.5f` to **`2.5f`** in both CPU ([Pop4DepthDecoder.cpp](file:///home/br1zb3ar/Reverse-Engineer-All3DScan-On-Linux/src/decoder/Pop4DepthDecoder.cpp#L25)) and CUDA ([Pop4StereoCuda.cu](file:///home/br1zb3ar/Reverse-Engineer-All3DScan-On-Linux/src/decoder/Pop4StereoCuda.cu#L21)) kernels to match the official APK `libRPASpeckleStereo.so` threshold:
  - Previously, `6.5f` was over 2x too strict, discarding valid low-contrast fringe phase data across 97% of the surface and starving the decoder (`Depth lock: 2%`).
  - Setting it to `2.5f` allows the phase decoder to process valid fringe signals across the full object surface.

## Alpha-v20260802-1534

### Fixed
- **Fringe Period Uniqueness Exclusion Fix**: Resolved the mathematical bug causing 3% depth lock and concentric funnel ring steps:
  - Previously, the rival cost search only excluded $\pm 1\text{px}$ (`|d - bestD| <= 1`), accidentally comparing the winner against the shoulder of its own local cost bowl ($\pm 2, \pm 3\text{px}$) rather than actual rival fringe periods ($\approx 8\text{--}16\text{px}$).
  - Updated the exclusion window to **`|d - bestD| < 6`** in both CPU ([Pop4DepthDecoder.cpp](file:///home/br1zb3ar/Reverse-Engineer-All3DScan-On-Linux/src/decoder/Pop4DepthDecoder.cpp#L280)) and CUDA ([Pop4StereoCuda.cu](file:///home/br1zb3ar/Reverse-Engineer-All3DScan-On-Linux/src/decoder/Pop4StereoCuda.cu#L165)) kernels.
  - This allows real surface points to easily pass uniqueness against actual rival periods, unlocking dense valid 3D points and eliminating concentric ring step artifacts.

## Alpha-v20260802-1528

### Fixed
- **Structured Light Projector / Laser Default Power Fix**: Fixed a critical bug in control initialization where `illuminationPower` and `laserPower` defaulted to `0%`. This caused the projector hardware to stream without active fringe patterns, leading to 2% depth lock and noisy funnel clouds. Default active power levels are now initialized on connect (`Illumination: 50%`, `Scan Laser: 70%`, `Exposure: 50%`, `Emitters On`).

## Alpha-v20260802-1516

### Added
- **PCL Moving Least Squares (MLS) Surface Smoothing**: Integrated PCL's native `pcl::MovingLeastSquares` polynomial surface reconstruction engine (inspired by `3DScanr` and `pointclouds.org`):
  - Smooths out depth sensor noise while preserving sharp feature edges.
  - Re-computes accurate surface normals for high-fidelity meshing.
  - Added the **MLS Smooth Surface (PCL)** button in the post-processing Edit panel.

## Alpha-v20260802-1504

### Fixed
- **Specular & Metallic Target Support**: Resolved the fronto-parallel cone/vase deformation on highly specular/reflective surfaces (e.g. metallic soda cans):
  - Balanced the intensity weighting to `0.05f` to prevent specular highlight mismatches from corrupting phase matching.
  - Tightened the Left-Right consistency check from `3.5px` to `1.2px` (`lrMaxDiff`) to geometrically filter out wrong-period ghost layers rather than relying heavily on surface albedo.
  - Set uniqueness ratio to `0.93f` for a balanced match selection.

## Alpha-v20260802-1453

### Added
- **PCL (Point Cloud Library) Integration**: Fully linked PCL (v1.14.0) into the build system and replaced the slow, custom $O(N^2)$ outlier removal algorithm in the edit panel with a high-performance, KdTree-based `pcl::StatisticalOutlierRemoval` filter.

## Alpha-v20260802-1112

### Added
- **Immediate Live Preview (Preview Mode)**: Automatically start streaming and polling from the scanner immediately on connection to show the live camera view in the viewport.
- **Fringe Ambiguity Resolution**: Tightened matching parameters to resolve true 3D shape and eliminate concentric ghost layers:
  - Tightened uniqueness ratio from `0.95f` to `0.90f` to reject ambiguous rival matches.
  - Increased minimum cost margin from `0.001f` to `0.005f`.
  - Tuned default modulation threshold to `6.5f` and intensity weighting to `0.08f`.

### Fixed
- **Scan Accumulation Control**: Rewired the Scan/Pause button to toggle point cloud accumulation/fusion, keeping the live stream running in preview mode prior to scanning.

## Alpha-v20260802-1033

### Fixed
- **Clean Grayscale IR Stream**: Removed the experimental blue/cyan coverage overlay tint from the IR Stream albedo view, restoring a pure grayscale feed that matches Revo Scan.
- **Pill-shaped Zoom Button Layout**: Embedded the `1x`, `1.5x`, and `2x` zoom buttons inside a semi-transparent floating pill-shaped overlay widget directly on top of the video container, preventing them from shifting the video layout downward.
- **Default Mode to Depth**: Set the default camera mode of the Depth Camera view to the colorized Depth Map on launch/stream.

## Alpha-v20260802-1025

### Added
- **Depth Map View Mode in Depth Camera**: The depth camera panel can now toggle between displaying the raw IR stream (grayscale fringes) and a colorized metric depth map. The colorization uses standard zone colors matching the working distance widget (Too Near: red, Excellent: green/cyan, Good: green, Far: orange, Too Far: red).
- **Digital Zoom Crops**: Added `1x`, `1.5x`, and `2x` digital zoom buttons inside the depth camera preview to crop and focus on fine-grained areas during scans.

### Fixed
- **Distance Scale Edge Sync**: Dynamic band edges are now updated dynamically on the vertical scale widget alongside scanning distance combo selections.

## Alpha-v20260802-1009

Prompted by a real-hardware scan of a small object (Revopoint POP4) where the
live point cloud was a shapeless blob unrelated to the object's actual shape,
even though tracking/fusion reported success. Root cause: the ribbon's
"Scanning Distance" presets (120-219mm / 210-400mm / 250-500mm) only ever
adjusted post-decode `ForegroundIsolation` clipping — the POP4 stereo phase
decoder itself always searched a fixed disparity window (8-120px) and clamped
to a fixed 200-650mm depth band, regardless of the selected preset. Objects
closer than ~220mm produce real disparities the decoder structurally couldn't
search, so it locked onto the best *available but wrong* fringe-period match
inside the fixed window — producing incoherent, noisy geometry that still
passed the decoder's own validity gates.

### Fixed
- **Decoder search band now follows the selected scanning distance.**
  `Pop4DepthDecoder::setDepthRange(minMm, maxMm)` derives the disparity
  search range `[dispMin, dispMax]` from the calibration's stereo Q matrix
  each decode, instead of a hardcoded band tuned for one preset. Both the CPU
  path (`Pop4DepthDecoder.cpp`) and the CUDA kernel (`Pop4StereoCuda.cu`) take
  this as a runtime parameter now. `MainWindow::onScanDistanceChanged` calls
  `IScannerProtocol::setDepthRangeHint()` alongside the existing isolation/
  depth-clip updates, so picking "120-219mm" for a close-up object actually
  retunes the decoder instead of only affecting downstream cropping.
- **"Accuracy" ribbon control was decorative.** The "High Accuracy" /
  "Standard" dropdown existed but had zero signal connections; fusion voxel
  sizes were hardcoded `constexpr`. `ScanPipeline::setPointDistance(mm)` makes
  the fused-model point spacing (Revo Scan's "point distance") a runtime
  atomic; the combo now drives it (1.0mm / 2.2mm), matching the point-distance
  quality/speed trade-off praised in the POP4 hands-on review this update was
  based on watching.

### Added
- **Live depth-decode confidence readout.** A new "Depth lock: NN% (CUDA/CPU,
  σ=X.Xpx)" label next to Tracking status, driven by
  `IScannerProtocol::depthDecodeStats()` / `Pop4DepthDecoder::lastStats()`.
  Surfaces per-frame valid-match fraction and disparity spread that were
  previously only visible via `ALL3DSCAN_POP4_DEBUG=1` stderr logging, so a
  garbage frame is visibly flagged (red/amber) instead of silently accepted.

### Known limitation (not fixed by this update)
This does not make POP4 depth reconstruction calibrated or accurate — the
decoder is still a clean-room approximation (`ApproximatePop4FullField`
calibration, single-frequency phase matching) without Revopoint's factory
calibration or exact algorithm. Single-frequency phase matching is inherently
period-ambiguous; even with the search band now correctly targeted, textureless
or repetitive-pattern surfaces can still produce wrong-period matches. The new
"Depth lock" readout is meant to make that visible and actionable (reposition
the object, adjust the distance preset) rather than to eliminate it. See
`docs/protocol/RE_NOTES.md` and `docs/protocol/USBCAP_CHECKLIST.md` for the
path to real calibration.

### Also in this session
- Wired the previously-decorative "Accuracy" ribbon dropdown (see Fixed,
  above) after a review of the Revopoint POP4 hands-on video prompted a
  broader pass over scan-mode / quality UI stubs.
