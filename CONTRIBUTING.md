# Contributing to All3DScan

All3DScan is a clean-room, native Linux project. Contributions for additional
scanner models, protocol documentation, calibration formats, reconstruction,
UI, packaging, and tests are welcome.

## Clean-room requirements

- Do not commit proprietary vendor binaries, SDK libraries, firmware,
  calibration files from devices you do not own, license keys, or decompiled
  vendor source code.
- Protocol documentation must describe observed behavior in original words.
- USB/network captures may contain serial numbers, Wi-Fi credentials, or other
  identifiers. Sanitize them before sharing.
- State how evidence was obtained: Linux descriptor output, a capture from
  hardware you own, public documentation, or an independently written test.

## Adding a scanner

1. Open a hardware-support issue using the template.
2. Attach sanitized output from:

   ```sh
   lsusb
   v4l2-ctl --list-devices
   v4l2-ctl --device=/dev/videoX --list-formats-ext
   udevadm info --attribute-walk --name=/dev/videoX
   ```

3. Add only confirmed VID/PID values to `resources/devices.json`.
4. Implement vendor/model behavior behind `IScannerProtocol`; do not add
   model-specific conditions to the UI.
5. Include tests that run without hardware where possible.
6. Update `docs/HARDWARE_SUPPORT.md` with the evidence-backed support level.

Detection alone is not scanner support. A model is only marked "Capture" after
depth/RGB acquisition works, and "Scanning" only after projector control,
calibration, metric depth generation, and continuous reconstruction work.

## Building and testing

```sh
cmake -S . -B build -DBUILD_TESTING=ON -DBUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Keep changes focused and do not include generated `build/`, `dist/`, packet
capture, or proprietary files.
