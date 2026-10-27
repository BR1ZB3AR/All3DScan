# hw_cpp — sensor layer (C++)

(The other C++ the engine compiles is `../fusion_cpp`, which wraps the Qt
app's registration so a scan's frames can be fused into one model.)

Talks to sensor hardware and vendor SDKs and hands raw point and depth
buffers to the Rust engine. It has no build of its own: `engine_rust`'s build
script compiles these files with `cxx-build` and links them into the engine.

- `include/all3dscan/` — headers, including the ones the cxx bridge reads
- `src/` — implementations
  - `MockDepthSensor` — a rendered scene, for development and tests
  - `Pop4Sensor` — the Revopoint POP 4 over USB. It drives the scanner
    through the legacy app's backend (`../src/protocol`, `../src/usb`,
    `../src/decoder`), which `engine_rust/build.rs` compiles too, with the
    CUDA depth decoder when nvcc is there. That backend needs libusb-1.0;
    without it `Pop4SensorStub.cpp` is built instead and opening a POP 4
    says so.

Nothing here may depend on Qt or Python; the Rust engine is its only caller.
