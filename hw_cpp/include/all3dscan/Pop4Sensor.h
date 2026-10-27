#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace revoscan::protocol {
class IScannerProtocol;
}

namespace all3dscan::hw {

// The Revopoint POP 4 over USB, driven through the scanner backend the
// legacy app uses (src/protocol/RevopointPop4Protocol): it arms the
// projector, streams the depth camera's packed infrared images, decodes them
// to metric depth (on the GPU when built with CUDA) and colours that depth
// from the colour camera with the scanner's factory calibration.
//
// capture() fills the same kinds of buffers as MockDepthSensor: points()
// holds x0, y0, z0, x1, ... in millimetres in the camera frame (x right, y
// down, z forward), one point per depth pixel that is not 0, in row-major
// order. Everything stays valid until the next capture() or until the sensor
// is destroyed. Not thread-safe: one caller at a time.
class Pop4Sensor {
public:
    // Built-in lights, as measured on a POP 4 (docs/protocol/RE_NOTES.md).
    static constexpr uint8_t kProjector = 0; // draws the scan pattern
    static constexpr uint8_t kFillLight = 1; // lights the scene for the colour camera
    // Scan modes. Leaving blue laser or speckle reboots the scanner (~45 s).
    static constexpr uint8_t kInfrared = 0;
    static constexpr uint8_t kBlueLaser = 1;
    static constexpr uint8_t kSpeckle = 2;
    // The depth decoder's grid: the left infrared camera, rectified.
    static constexpr uint32_t kDecodedWidth = 320;
    static constexpr uint32_t kDecodedHeight = 512;
    // How long opening one waits for the scanner's first frame, which proves
    // the stream really works: three tries of two seconds.
    static constexpr int kFirstFrameAttempts = 3;
    static constexpr int kFirstFrameTimeoutMs = 2000;

    // Connects and starts streaming. Throws std::runtime_error with the
    // scanner's reason if it cannot.
    Pop4Sensor();
    // Stops streaming and puts the lights back as connecting leaves them; in
    // blue laser or speckle mode it reboots the scanner into infrared without
    // waiting, so the next session starts there.
    ~Pop4Sensor();
    Pop4Sensor(const Pop4Sensor&) = delete;
    Pop4Sensor& operator=(const Pop4Sensor&) = delete;

    // Waits up to timeoutMs for the next frame. False if none came; then
    // lastError() says why and the buffers keep the previous frame. A frame
    // whose images gave no depth (most of them, in blue laser mode) still
    // counts, with no points and lastError() saying why.
    bool capture(int32_t timeoutMs);

    const std::vector<float>& points() const noexcept { return points_; }
    // Depth in millimetres, width() x height(); 0 where the pixel has none.
    const std::vector<uint16_t>& depthMm() const noexcept { return depthMm_; }
    // The left infrared camera's view, one byte per pixel, on the same grid
    // as the depth: the exposure the pattern projector lights, so that
    // switching it shows here (the depth is matched on a speckle image an
    // always-on emitter lights instead).
    const std::vector<uint8_t>& ir() const noexcept { return ir_; }
    // The colour of each depth pixel (R, G, B), on the depth's grid; 0 where
    // the colour camera does not see it.
    const std::vector<uint8_t>& depthRgb() const noexcept { return depthRgb_; }
    // The colour camera's own image (R, G, B), colorWidth() x colorHeight();
    // empty until one has been decoded.
    const std::vector<uint8_t>& colorImage() const noexcept { return color_; }
    uint32_t width() const noexcept { return width_; }
    uint32_t height() const noexcept { return height_; }
    uint32_t colorWidth() const noexcept { return colorWidth_; }
    uint32_t colorHeight() const noexcept { return colorHeight_; }
    // Number of frames captured so far.
    uint64_t frameIndex() const noexcept { return frameIndex_; }
    // Whether the scanner is still there. False once it has unplugged or
    // re-enumerated: capture() then never succeeds again, and this object
    // should be dropped and a new one opened.
    bool connected() const;

    // The share of the newest frame's pixels that got depth.
    float validFraction() const;

    // Controls; each returns false, with lastError(), if the scanner refused.
    // Throw std::invalid_argument for a light or mode it does not have.
    bool setLight(uint8_t light, bool on);
    bool setLightLevel(uint8_t light, int32_t level); // 0..255
    bool lightOn(uint8_t light) const;
    int32_t lightLevel(uint8_t light) const;
    bool setExposure(int32_t microseconds);
    int32_t exposure() const;
    bool setScanMode(uint8_t mode);
    uint8_t scanMode() const;
    // The working distance the depth decoder searches, in millimetres.
    void setDepthRange(float nearMm, float farMm);

    const std::string& serial() const noexcept { return serial_; }
    // Which decoder and calibration are in use, e.g. "Revopoint POP4
    // (factory calibration, CUDA)".
    const std::string& backendName() const noexcept { return backend_; }
    // Why the last call that failed did.
    const std::string& lastError() const noexcept { return error_; }

private:
    bool check(bool ok);
    bool captureFirstFrame();

    std::unique_ptr<revoscan::protocol::IScannerProtocol> scanner_;
    std::string serial_;
    std::string backend_;
    std::string error_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t colorWidth_ = 0;
    uint32_t colorHeight_ = 0;
    uint64_t frameIndex_ = 0;
    std::vector<float> points_;
    std::vector<uint16_t> depthMm_;
    std::vector<uint8_t> ir_;
    std::vector<uint8_t> depthRgb_;
    std::vector<uint8_t> color_;
};

// cxx cannot call constructors, so the Rust engine opens the scanner here.
std::unique_ptr<Pop4Sensor> openPop4Sensor();

// Whether a POP 4 is on USB, from sysfs, without opening it.
bool pop4Present();

} // namespace all3dscan::hw
