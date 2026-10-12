#include "ScannerProtocol.h"

#include <chrono>
#include <cmath>

namespace revoscan::protocol {
namespace {

class MockProtocol final : public IScannerProtocol {
public:
    bool connect() override {
        state_ = ConnectionState::Connected;
        controls_.emittersOn = false;
        return true;
    }

    void disconnect() override {
        emittersOff();
        state_ = ConnectionState::Disconnected;
    }

    ConnectionState state() const override { return state_; }

    bool startStream() override {
        if (state_ == ConnectionState::Disconnected) {
            return false;
        }
        controls_.emittersOn = true;
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
        frame.encoding = DepthEncoding::MetricMillimeters;
        frame.depthMm.resize(static_cast<size_t>(kWidth) * kHeight);
        frame.rgb.resize(static_cast<size_t>(kWidth) * kHeight * 3);

        const double phase = static_cast<double>(frameCounter_) * 0.05;
        // Mild camera orbit so registration / fusion see motion.
        const double orbit = static_cast<double>(frameCounter_) * 0.01;
        for (uint32_t y = 0; y < kHeight; ++y) {
            for (uint32_t x = 0; x < kWidth; ++x) {
                const double nx = (static_cast<double>(x) / kWidth - 0.5) * 2.0;
                const double ny = (static_cast<double>(y) / kHeight - 0.5) * 2.0;
                const double ox = nx * std::cos(orbit) - 0.05 * std::sin(orbit);
                const double r2 = ox * ox + ny * ny;
                const size_t idx = static_cast<size_t>(y) * kWidth + x;
                uint16_t depth = 0;
                if (r2 < 1.0) {
                    const double z = std::sqrt(1.0 - r2);
                    depth = static_cast<uint16_t>(kBaseDistanceMm - z * kRadiusMm);
                    const uint8_t shade = static_cast<uint8_t>(
                        128 + 127 * std::sin(phase + ox * 3.0) * std::cos(ny * 3.0));
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

    void setParams(const ScannerParams& params) override { lastParams_ = params; }

    CameraIntrinsics intrinsics() const override {
        return CameraIntrinsics{
            .fx = 570.0,
            .fy = 570.0,
            .cx = kWidth / 2.0,
            .cy = kHeight / 2.0,
        };
    }

    ScannerCalibration calibration() const override {
        ScannerCalibration cal;
        cal.depth = intrinsics();
        cal.rgb = intrinsics();
        cal.depthScale = 1.0;
        cal.serial = "MOCK-0001";
        cal.firmware = "mock-1.0";
        cal.source = "mock";
        cal.valid = true;
        return cal;
    }

    ScannerControlCapabilities controlCapabilities() const override {
        ScannerControlCapabilities caps;
        caps.scanMode = true;
        caps.alignmentGuide = true;
        caps.illuminationPower = true;
        caps.laserPower = true;
        caps.exposure = true;
        caps.emitterOff = true;
        caps.illuminationMin = 0;
        caps.illuminationMax = 1000;
        caps.laserMin = 0;
        caps.laserMax = 1000;
        caps.exposureMin = 1;
        caps.exposureMax = 5000;
        return caps;
    }

    ScannerControlState controlState() const override { return controls_; }

    bool setScanMode(ScanMode mode) override {
        controls_.mode = mode;
        return true;
    }

    bool setAlignmentGuide(bool on) override {
        controls_.alignmentGuideOn = on;
        return true;
    }

    bool setIlluminationPower(int value) override {
        const auto caps = controlCapabilities();
        if (value < caps.illuminationMin || value > caps.illuminationMax) {
            return false;
        }
        controls_.illuminationPower = value;
        return true;
    }

    bool setLaserPower(int value) override {
        const auto caps = controlCapabilities();
        if (value < caps.laserMin || value > caps.laserMax) {
            return false;
        }
        controls_.laserPower = value;
        return true;
    }

    bool setExposure(int value) override {
        const auto caps = controlCapabilities();
        if (value < caps.exposureMin || value > caps.exposureMax) {
            return false;
        }
        controls_.exposure = value;
        return true;
    }

    bool emittersOff() override {
        controls_.emittersOn = false;
        controls_.alignmentGuideOn = false;
        controls_.illuminationPower = 0;
        controls_.laserPower = 0;
        return true;
    }

    std::string backendName() const override { return "Mock scanner"; }
    std::string lastError() const override { return {}; }

private:
    static constexpr uint32_t kWidth = 320;
    static constexpr uint32_t kHeight = 240;
    static constexpr double kRadiusMm = 150.0;
    static constexpr double kBaseDistanceMm = 500.0;

    ConnectionState state_ = ConnectionState::Disconnected;
    uint64_t frameCounter_ = 0;
    ScannerParams lastParams_;
    ScannerControlState controls_{};
};

} // namespace

std::unique_ptr<IScannerProtocol> makeMockProtocol() {
    return std::make_unique<MockProtocol>();
}

} // namespace revoscan::protocol
