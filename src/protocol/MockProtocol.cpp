#include "ScannerProtocol.h"

#include <chrono>
#include <cmath>

namespace revoscan::protocol {
namespace {

// Synthetic sphere so the GUI/pipeline have something real to render and
// export before the actual sensor format is known.
class MockProtocol final : public IScannerProtocol {
public:
    bool connect() override {
        state_ = ConnectionState::Connected;
        return true;
    }

    void disconnect() override {
        state_ = ConnectionState::Disconnected;
    }

    ConnectionState state() const override {
        return state_;
    }

    bool startStream() override {
        if (state_ == ConnectionState::Disconnected) {
            return false;
        }
        state_ = ConnectionState::Streaming;
        return true;
    }

    void stopStream() override {
        if (state_ == ConnectionState::Streaming) {
            state_ = ConnectionState::Connected;
        }
    }

    std::optional<RawFrame> readFrame(int /*timeoutMs*/) override {
        if (state_ != ConnectionState::Streaming) {
            return std::nullopt;
        }

        RawFrame frame;
        frame.width = kWidth;
        frame.height = kHeight;
        frame.depthMm.resize(static_cast<size_t>(kWidth) * kHeight);
        frame.rgb.resize(static_cast<size_t>(kWidth) * kHeight * 3);

        // Depth of a sphere of radius kRadiusMm centered in frame, in mm
        // from the sensor plane. Phase pattern rotates each call so a
        // live-preview consumer sees motion.
        const double phase = static_cast<double>(frameCounter_) * 0.05;
        for (uint32_t y = 0; y < kHeight; ++y) {
            for (uint32_t x = 0; x < kWidth; ++x) {
                const double nx = (static_cast<double>(x) / kWidth - 0.5) * 2.0;
                const double ny = (static_cast<double>(y) / kHeight - 0.5) * 2.0;
                const double r2 = nx * nx + ny * ny;
                const size_t idx = static_cast<size_t>(y) * kWidth + x;
                uint16_t depth = 0;
                if (r2 < 1.0) {
                    const double z = std::sqrt(1.0 - r2);
                    depth = static_cast<uint16_t>(kBaseDistanceMm - z * kRadiusMm);
                    const uint8_t shade = static_cast<uint8_t>(
                        128 + 127 * std::sin(phase + nx * 3.0) * std::cos(ny * 3.0));
                    frame.rgb[idx * 3 + 0] = shade;
                    frame.rgb[idx * 3 + 1] = static_cast<uint8_t>(200 - shade / 2);
                    frame.rgb[idx * 3 + 2] = 220;
                }
                frame.depthMm[idx] = depth;
            }
        }

        frame.timestampUs = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        ++frameCounter_;
        return frame;
    }

    void setParams(const ScannerParams& params) override {
        lastParams_ = params;
    }

    CameraIntrinsics intrinsics() const override {
        // Placeholder pinhole intrinsics for a kWidth x kHeight sensor at a
        // plausible focal length; replaced with real calibration data in
        // Phase B.
        return CameraIntrinsics{
            .fx = 570.0,
            .fy = 570.0,
            .cx = kWidth / 2.0,
            .cy = kHeight / 2.0,
        };
    }

private:
    static constexpr uint32_t kWidth = 320;
    static constexpr uint32_t kHeight = 240;
    static constexpr double kRadiusMm = 150.0;
    static constexpr double kBaseDistanceMm = 500.0;

    ConnectionState state_ = ConnectionState::Disconnected;
    uint64_t frameCounter_ = 0;
    ScannerParams lastParams_;
};

} // namespace

std::unique_ptr<IScannerProtocol> makeMockProtocol() {
    return std::make_unique<MockProtocol>();
}

} // namespace revoscan::protocol
