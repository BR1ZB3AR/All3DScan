#include "all3dscan/MockDepthSensor.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace all3dscan::hw {

namespace {

constexpr uint32_t kMaxSide = 8192;

// The scene, as depths along the optical axis.
constexpr float kWallDepthMm = 600.0f;
constexpr float kWallSquareMm = 40.0f;      // chequer size on the wall
constexpr float kBallDepthMm = 450.0f;      // ball centre at frame 0
constexpr float kBallRadiusMm = 120.0f;
constexpr float kBallSwayMm = 40.0f;        // side-to-side drift amplitude
constexpr float kBallSwayPerFrame = 0.2f;   // drift phase step, radians
constexpr float kBallSurgeMm = 80.0f;       // towards-and-away drift amplitude
constexpr float kBallSurgePerFrame = 0.05f; // surge phase step, radians
// A vertical pipe standing beside the ball, so there is a cylinder to fit.
constexpr float kPipeXMm = 175.0f;
constexpr float kPipeZMm = 470.0f;
constexpr float kPipeRadiusMm = 40.0f;
constexpr float kPipeHalfHeightMm = 110.0f;

// Sensor imperfections.
constexpr float kDepthNoiseMm = 0.5f;    // one standard deviation
constexpr float kDropoutRate = 0.02f;    // share of pixels with no depth
constexpr float kSpeckleDensity = 0.08f; // share of pixels a projector dot lands on

uint8_t toByte(float v) {
    return static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
}

// Image grain from a hash of the pixel and frame, in [-0.5, 0.5], so that it
// does not consume the depth generator's random numbers.
float grain(uint32_t u, uint32_t v, uint64_t frame) {
    uint64_t h = u * 0x9E3779B97F4A7C15ull ^ v * 0xC2B2AE3D27D4EB4Full ^ frame * 0x165667B19E3779F9ull;
    h ^= h >> 29;
    h *= 0xBF58476D1CE4E5B9ull;
    h ^= h >> 32;
    return static_cast<float>(h & 0xFFFF) / 65535.0f - 0.5f;
}

} // namespace

MockDepthSensor::MockDepthSensor(uint32_t width, uint32_t height, uint64_t seed)
    : width_(width), height_(height), rng_(seed) {
    if (width == 0 || height == 0 || width > kMaxSide || height > kMaxSide) {
        throw std::invalid_argument("MockDepthSensor: width and height must be 1-" +
                                    std::to_string(kMaxSide) + ", got " +
                                    std::to_string(width) + "x" + std::to_string(height));
    }
    const size_t pixels = static_cast<size_t>(width) * height;
    // At most one point per pixel, so capture() never has to grow a buffer.
    points_.reserve(pixels * 3);
    depthMm_.resize(pixels);
    ir_.resize(pixels);
    rgb_.resize(pixels * 3);

    // The projector's dots stay put relative to the camera.
    speckle_.resize(pixels);
    std::mt19937 dots(static_cast<uint32_t>(seed) ^ 0x5eedu);
    std::bernoulli_distribution lit(kSpeckleDensity);
    for (uint8_t& dot : speckle_) {
        dot = lit(dots) ? 1 : 0;
    }
}

void MockDepthSensor::capture() noexcept {
    // Pinhole camera, about 64 degrees across.
    const float fx = 0.8f * static_cast<float>(width_);
    const float fy = fx;
    const float cx = 0.5f * static_cast<float>(width_ - 1);
    const float cy = 0.5f * static_cast<float>(height_ - 1);

    // The ball drifts along x and along the optical axis.
    const float frame = static_cast<float>(frameIndex_);
    const float ballX = kBallSwayMm * std::sin(kBallSwayPerFrame * frame);
    const float ballZ = kBallDepthMm + kBallSurgeMm * std::sin(kBallSurgePerFrame * frame);
    // Offset from the ball's centre to the camera.
    const float ox = -ballX;
    const float oz = -ballZ;
    const float c = ox * ox + oz * oz - kBallRadiusMm * kBallRadiusMm;
    const float pipeC = kPipeXMm * kPipeXMm + kPipeZMm * kPipeZMm - kPipeRadiusMm * kPipeRadiusMm;

    std::normal_distribution<float> noise(0.0f, kDepthNoiseMm);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);

    points_.clear();
    for (uint32_t v = 0; v < height_; ++v) {
        for (uint32_t u = 0; u < width_; ++u) {
            const size_t i = static_cast<size_t>(v) * width_ + u;
            // Ray through the pixel, scaled so its z is 1: the point t along
            // it has depth z = t.
            const float dx = (static_cast<float>(u) - cx) / fx;
            const float dy = (static_cast<float>(v) - cy) / fy;

            // Nearest hit on the ball or the pipe, else the wall behind them.
            const float a = dx * dx + dy * dy + 1.0f;
            const float b = 2.0f * (dx * ox + oz);
            const float disc = b * b - 4.0f * a * c;
            float z = kWallDepthMm;
            enum class Surface { Wall, Ball, Pipe } surface = Surface::Wall;
            if (disc >= 0.0f) {
                const float t = (-b - std::sqrt(disc)) / (2.0f * a);
                if (t > 0.0f && t < z) {
                    z = t;
                    surface = Surface::Ball;
                }
            }
            // The pipe's side: (x - px)^2 + (z - pz)^2 = r^2, within its height.
            const float pipeA = dx * dx + 1.0f;
            const float pipeB = -2.0f * (dx * kPipeXMm + kPipeZMm);
            const float pipeDisc = pipeB * pipeB - 4.0f * pipeA * pipeC;
            if (pipeDisc >= 0.0f) {
                const float t = (-pipeB - std::sqrt(pipeDisc)) / (2.0f * pipeA);
                if (t > 0.0f && t < z && std::abs(dy * t) <= kPipeHalfHeightMm) {
                    z = t;
                    surface = Surface::Pipe;
                }
            }

            // Lambert shading, lit from the camera.
            const float x = dx * z;
            const float y = dy * z;
            const float distance = std::sqrt(a) * z;
            float shade = z / distance; // the wall faces the camera
            if (surface == Surface::Ball) {
                const float nx = (x - ballX) / kBallRadiusMm;
                const float ny = y / kBallRadiusMm;
                const float nz = (z - ballZ) / kBallRadiusMm;
                shade = std::max(0.0f, -(nx * x + ny * y + nz * z) / distance);
            } else if (surface == Surface::Pipe) {
                const float nx = (x - kPipeXMm) / kPipeRadiusMm;
                const float nz = (z - kPipeZMm) / kPipeRadiusMm;
                shade = std::max(0.0f, -(nx * x + nz * z) / distance);
            }

            // Colour: an orange ball and a teal pipe in front of a grey
            // chequered wall.
            float red = 224.0f, green = 122.0f, blue = 47.0f;
            if (surface == Surface::Pipe) {
                red = 60.0f;
                green = 150.0f;
                blue = 160.0f;
            } else if (surface == Surface::Wall) {
                const int square = static_cast<int>(std::floor(x / kWallSquareMm)) +
                                   static_cast<int>(std::floor(y / kWallSquareMm));
                const bool dark = (square & 1) != 0;
                red = dark ? 150.0f : 205.0f;
                green = dark ? 155.0f : 208.0f;
                blue = dark ? 162.0f : 214.0f;
            }
            const float light = 0.25f + 0.75f * shade;
            const float colourGrain = 6.0f * grain(u, v, frameIndex_);
            rgb_[3 * i] = toByte(red * light + colourGrain);
            rgb_[3 * i + 1] = toByte(green * light + colourGrain);
            rgb_[3 * i + 2] = toByte(blue * light + colourGrain);

            // Infrared: the projector's dots over a dim background, fading
            // with distance.
            const float falloff = (400.0f / z) * (400.0f / z);
            const float dot = speckle_[i] ? 210.0f : 35.0f;
            ir_[i] = toByte(18.0f + dot * shade * falloff + 10.0f * grain(v, u, frameIndex_));

            // Depth, with noise and dropped pixels.
            if (unit(rng_) < kDropoutRate) {
                depthMm_[i] = 0;
                continue;
            }
            z += noise(rng_);
            depthMm_[i] = static_cast<uint16_t>(std::lround(z));
            points_.push_back(dx * z);
            points_.push_back(dy * z);
            points_.push_back(z);
        }
    }
    ++frameIndex_;
}

std::unique_ptr<MockDepthSensor> makeMockDepthSensor(uint32_t width, uint32_t height,
                                                     uint64_t seed) {
    return std::make_unique<MockDepthSensor>(width, height, seed);
}

} // namespace all3dscan::hw
