#pragma once

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

namespace all3dscan::hw {

// Stands in for a depth camera until a real sensor SDK is wired in. Each
// capture renders a ball, drifting sideways and towards and away from the
// camera, and a vertical pipe in front of a chequered wall (a sphere, a
// cylinder and a plane to fit), and produces what a structured-light
// scanner gives: a point cloud, a depth image, an infrared image with the
// projector's speckle and a colour image. Depth gets noise and dropped
// pixels. The same seed gives the same sequence of frames.
//
// points() holds x0, y0, z0, x1, y1, z1, ... in millimetres in the camera
// frame (x right, y down, z forward), one point per pixel that has depth. The
// images are row-major, width x height pixels, all from the same viewpoint.
// Everything stays valid until the next capture() or until the sensor is
// destroyed.
class MockDepthSensor {
public:
    // Throws std::invalid_argument if width or height is 0 or above 8192.
    MockDepthSensor(uint32_t width, uint32_t height, uint64_t seed);

    // Renders the next frame into the buffers. It reuses the buffers the
    // constructor allocated, so it never allocates and never throws.
    void capture() noexcept;

    const std::vector<float>& points() const noexcept { return points_; }
    // Depth in millimetres; 0 where the pixel has none.
    const std::vector<uint16_t>& depthMm() const noexcept { return depthMm_; }
    // Infrared intensity, one byte per pixel.
    const std::vector<uint8_t>& ir() const noexcept { return ir_; }
    // Colour, three bytes (R, G, B) per pixel.
    const std::vector<uint8_t>& rgb() const noexcept { return rgb_; }
    uint32_t width() const noexcept { return width_; }
    uint32_t height() const noexcept { return height_; }
    // Number of frames captured so far.
    uint64_t frameIndex() const noexcept { return frameIndex_; }

private:
    uint32_t width_;
    uint32_t height_;
    uint64_t frameIndex_ = 0;
    std::mt19937_64 rng_;
    std::vector<float> points_;
    std::vector<uint16_t> depthMm_;
    std::vector<uint8_t> ir_;
    std::vector<uint8_t> rgb_;
    std::vector<uint8_t> speckle_; // the projector's fixed dot pattern; 1 = dot
};

// cxx cannot call constructors, so the Rust engine creates sensors here.
std::unique_ptr<MockDepthSensor> makeMockDepthSensor(uint32_t width, uint32_t height,
                                                     uint64_t seed);

} // namespace all3dscan::hw
