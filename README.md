# All3DScan — Open-Source Native Linux 3D Scanning App 
[Project Still Under-Development]

A clean-room, open-source **native** Linux scanning application with a dark interface, targeting consumer structured-light 3D scanners
from **Revopoint, Creality, and Einstar**. This is **not** a Wine wrapper:
day-to-day scanning uses Ubuntu's own USB (`uvcvideo`/`hidraw`) and GPU stacks.
Wine often fails to expose these scanners and cannot drive their Direct3D/CUDA
paths reliably — that is exactly why this project exists.

The production app uses C++/Qt for deterministic capture and UI, with Python
workers for protocol analysis and rapidly evolving Open3D processing. It
targets Ubuntu 24.04 and is portable to other Linux distributions. None of
these vendors ship a Linux build of their scanning software, so this project
reverse-engineers the USB protocol directly.

## Supported hardware

Device IDs live in `resources/devices.json` and are matched at runtime, so new
models can be added without recompiling. Entries may match a specific product
or an entire vendor.

See [`docs/HARDWARE_SUPPORT.md`](docs/HARDWARE_SUPPORT.md) for the
evidence-based compatibility matrix. Detection does not imply that capture,
calibration, projector control, or reconstruction is implemented.

| Vendor | USB VID | Status |
| --- | --- | --- |
| Revopoint (POP4/POP/POP2/POP3/MINI/RANGE) | `2207` | Confirmed `2207:110c`; vendor-wide match enabled |
| Creality (CR-Scan Otter/Ferret/Ferret Pro/Raptor) | `2bc5` | Confirmed `2bc5:06da` (Otter); vendor-wide match enabled |
| Einstar / Shining3D | `2231`* | Placeholder, disabled — confirm VID via `lsusb` and enable |

\* The Einstar VID is not yet confirmed here. Attach the scanner, run `lsusb`,
set the real `vendorId` in `resources/devices.json`, and flip `"enabled": true`.

## Status

**Native C++ projector arming and raw capture work on Revopoint POP4.** The
backend issues the vendor UVC Extension Unit CS10 + CS7 sequence through
libusb, returns control to `uvcvideo`, discovers the correct node without
assuming `/dev/videoN`, and streams 640×512 Y16 through V4L2 MMAP. A physical
POP4 smoke test captured 327,680 samples with 14,678 nonzero values.

Still open: structured-light decoding of packed `Y16` buffers into metric
depth, factory calibration extraction, RGB/depth registration, and a known
projector-off command. All3DScan explicitly marks these frames as vendor-raw
and refuses to project them as millimetres.

## Build (Ubuntu / Debian-based)

```sh
sudo apt-get install -y build-essential cmake qtbase5-dev qt5-qmake \
    libqt5opengl5-dev libusb-1.0-0-dev catch2 nlohmann-json3-dev

cmake -B build -S .
cmake --build build -j
```

Run the app:
```sh
./build/all3dscan
```

The default backend is the native POP4 backend. Run the synthetic UI backend
without hardware with:

```sh
ALL3DSCAN_BACKEND=mock ./build/all3dscan
```

Run a native, non-GUI POP4 arm/capture smoke test:

```sh
./build/all3dscan --probe-pop4
```

Run tests:
```sh
cd build && ctest --output-on-failure
```

## Hybrid C++/Python architecture

- C++ owns Qt UI, V4L2/libusb acquisition, frame queues, back-pressure, and
  point-cloud display/export.
- Python owns USB capture analysis, calibration experiments, and Open3D
  meshing while those algorithms are still changing.
- `src/bridge/PythonWorker.*` provides a JSON-lines subprocess boundary; a
  Python failure cannot corrupt the capture process.
- `tools/pipeline/mesh_worker.py` implements the first worker command:
  normal estimation plus Poisson reconstruction.

Install the optional meshing environment:

```sh
python3 -m venv ~/.local/share/all3dscan/venv
~/.local/share/all3dscan/venv/bin/pip install -r tools/pipeline/requirements.txt
```

## Linux installers

Build a Debian package, an AppImage, or both:

```sh
chmod +x tools/package_linux.sh
./tools/package_linux.sh deb
./tools/package_linux.sh appimage
# or: ./tools/package_linux.sh all
```

Artifacts are written to `dist/`. The AppImage script downloads `linuxdeploy`
and its Qt plugin on first use. The `.deb` installs the desktop entry,
AppStream metadata, icon, native executable, Python worker, `devices.json`,
documentation, and the scanner udev rules. AppImage cannot install host udev
permissions; copy `usr/share/doc/all3dscan/99-all3dscan-scanners.rules` into
`/etc/udev/rules.d/` manually after first launch. Open3D remains optional
because its wheel is large; point-cloud capture and export do not depend on it.

## USB device permissions

Install the udev rules so the app can access the scanner without root:
```sh
sudo cp resources/99-all3dscan-scanners.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Project layout

- `src/protocol/` — `IScannerProtocol` interface + `MockProtocol`. Real
  device protocol goes here once reverse-engineered.
- `src/usb/` — libusb transport wrapper (multi-vendor device enumeration,
  bulk I/O).
- `src/bridge/` — isolated C++ → Python worker process bridge.
- `src/pointcloud/` — depth-frame → point cloud back-projection.
- `src/io/` — PLY/OBJ export.
- `src/app/` — Qt GUI: `MainWindow` (Revo Scan-style shell), `PointCloudView`,
  `DistanceScaleWidget`, `CameraPanel`, `Theme`, `ScannerController`.
- `tests/` — Catch2 unit tests, runnable without hardware.
- `docs/protocol/RE_NOTES.md` — reverse-engineering plan and findings log.
- `docs/protocol/REVOSCAN6_RE.md` — clean-room map from Revo Scan 6 Windows
  installer/SDK → native Ubuntu `.deb`.
- `tools/re_probe/` — usbmon/XU/HID protocol analysis and replay tools.
- `tools/pipeline/` — Python/Open3D processing workers.

## Contributing

If you have a Revopoint, Creality, or Einstar scanner and can help capture USB
traffic (especially confirming VID/PIDs and control sequences), see
`docs/protocol/RE_NOTES.md` for the capture procedure. That is the critical
unblock for broad hardware support.

Read [`CONTRIBUTING.md`](CONTRIBUTING.md) before submitting code or captures.
The repository includes hardware-support and bug-report issue templates plus
an Ubuntu 24.04 build/test workflow.
