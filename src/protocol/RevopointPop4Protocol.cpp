#include "RevopointPop4Protocol.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

#include <libusb-1.0/libusb.h>
#include <linux/videodev2.h>

#include "ScannerProtocol.h"
#include "usb/UsbDevice.h"
#include "usb/V4l2Capture.h"

namespace revoscan::protocol {
namespace {

constexpr uint8_t kUvcSetRequestType = 0x21;
constexpr uint8_t kUvcSetCur = 0x01;
constexpr int kControlTimeoutMs = 500;

uint16_t xuValue(uint8_t selector) {
    return static_cast<uint16_t>(selector) << 8;
}

uint16_t xuIndex(uint8_t unitId, uint8_t interfaceNumber) {
    return (static_cast<uint16_t>(unitId) << 8) | interfaceNumber;
}

class RevopointPop4Protocol final : public IScannerProtocol {
public:
    bool connect() override {
        disconnect();
        lastError_.clear();

        if (!armProjector()) {
            disconnect();
            return false;
        }

        // The kernel may recreate video nodes after reattaching uvcvideo.
        std::optional<std::string> node;
        for (int attempt = 0; attempt < 40 && !node; ++attempt) {
            node = usb::findV4l2CaptureNode(
                pop4::kVendorId,
                pop4::kProductId,
                V4L2_PIX_FMT_Y16,
                pop4::kCaptureWidth,
                pop4::kCaptureHeight);
            if (!node) {
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            }
        }
        if (!node) {
            lastError_ = "POP4 armed, but no 640x512 Y16 V4L2 capture node was found";
            disconnect();
            return false;
        }

        if (!capture_.open(*node,
                           V4L2_PIX_FMT_Y16,
                           pop4::kCaptureWidth,
                           pop4::kCaptureHeight)) {
            lastError_ = capture_.lastError();
            disconnect();
            return false;
        }
        state_ = ConnectionState::Connected;
        return true;
    }

    void disconnect() override {
        capture_.close();
        usb_.close();
        state_ = ConnectionState::Disconnected;
    }

    ConnectionState state() const override { return state_; }

    bool startStream() override {
        if (state_ == ConnectionState::Disconnected) {
            lastError_ = "POP4 is not connected";
            return false;
        }
        if (!capture_.start()) {
            lastError_ = capture_.lastError();
            return false;
        }
        state_ = ConnectionState::Streaming;
        return true;
    }

    void stopStream() override {
        capture_.stop();
        if (state_ != ConnectionState::Disconnected) {
            state_ = ConnectionState::Connected;
        }
    }

    std::optional<RawFrame> readFrame(int timeoutMs) override {
        if (state_ != ConnectionState::Streaming) {
            return std::nullopt;
        }
        auto sensor = capture_.readFrame(timeoutMs);
        if (!sensor) {
            if (!capture_.lastError().empty()) {
                lastError_ = capture_.lastError();
            }
            return std::nullopt;
        }

        constexpr size_t expectedBytes =
            static_cast<size_t>(pop4::kCaptureWidth) * pop4::kCaptureHeight * 2;
        if (sensor->bytes.size() < expectedBytes) {
            lastError_ = "POP4 returned a truncated Y16 frame";
            return std::nullopt;
        }

        RawFrame frame;
        frame.width = pop4::kCaptureWidth;
        frame.height = pop4::kCaptureHeight;
        frame.timestampUs = sensor->timestampUs;
        frame.encoding = DepthEncoding::VendorRawY16;
        frame.depthMm.resize(expectedBytes / 2);
        // V4L2 Y16 is little-endian on this device. Decode explicitly so the
        // diagnostic path remains correct on big-endian hosts too.
        for (size_t i = 0; i < frame.depthMm.size(); ++i) {
            frame.depthMm[i] =
                static_cast<uint16_t>(sensor->bytes[i * 2]) |
                (static_cast<uint16_t>(sensor->bytes[i * 2 + 1]) << 8);
        }
        return frame;
    }

    void setParams(const ScannerParams& params) override { params_ = params; }

    CameraIntrinsics intrinsics() const override {
        // Factory calibration parsing and packed-Y16 decoding are not known.
        // Zero intrinsics prevent accidental metric projection.
        return {};
    }

    std::string backendName() const override { return "Revopoint POP4 (native raw Y16)"; }
    std::string lastError() const override { return lastError_; }

private:
    bool armProjector() {
        if (!usb_.open(pop4::kVendorId, pop4::kProductId)) {
            lastError_ =
                "Revopoint POP4 2207:110c not found or permission denied";
            return false;
        }

        bool detached = false;
        const int active = usb_.kernelDriverActive(pop4::kDepthInterface);
        if (active == 1) {
            const int rc = usb_.detachKernelDriver(pop4::kDepthInterface);
            if (rc != LIBUSB_SUCCESS) {
                lastError_ = "Could not detach uvcvideo from POP4 depth interface";
                return false;
            }
            detached = true;
        } else if (active < 0 && active != LIBUSB_ERROR_NOT_SUPPORTED) {
            lastError_ = "Could not query POP4 depth interface driver";
            return false;
        }

        if (!usb_.claimInterface(pop4::kDepthInterface)) {
            lastError_ = "Could not claim POP4 depth control interface";
            if (detached) {
                usb_.attachKernelDriver(pop4::kDepthInterface);
            }
            return false;
        }

        auto enable = pop4::makeEnablePayload();
        int rc = usb_.controlTransfer(
            kUvcSetRequestType,
            kUvcSetCur,
            xuValue(pop4::kEnableSelector),
            xuIndex(pop4::kXuUnitId, pop4::kDepthInterface),
            enable.data(),
            enable.size(),
            kControlTimeoutMs);
        if (rc != static_cast<int>(enable.size())) {
            lastError_ = "POP4 CS10 projector-enable transfer failed";
            finishControlClaim(detached);
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        auto settings = pop4::makeSettingsPayload();
        rc = usb_.controlTransfer(
            kUvcSetRequestType,
            kUvcSetCur,
            xuValue(pop4::kSettingsSelector),
            xuIndex(pop4::kXuUnitId, pop4::kDepthInterface),
            settings.data(),
            settings.size(),
            kControlTimeoutMs);
        if (rc != static_cast<int>(settings.size())) {
            lastError_ = "POP4 CS7 projector-settings transfer failed";
            finishControlClaim(detached);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(150));

        if (!finishControlClaim(detached)) {
            return false;
        }
        // V4L2 owns the device after this point.
        usb_.close();
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        return true;
    }

    bool finishControlClaim(bool detached) {
        if (usb_.releaseInterface() != LIBUSB_SUCCESS) {
            lastError_ = "Could not release POP4 depth control interface";
            return false;
        }
        if (detached) {
            const int rc = usb_.attachKernelDriver(pop4::kDepthInterface);
            if (rc != LIBUSB_SUCCESS) {
                lastError_ = "Could not reattach uvcvideo to POP4 depth interface";
                return false;
            }
        }
        return true;
    }

    ConnectionState state_ = ConnectionState::Disconnected;
    usb::UsbDevice usb_;
    usb::V4l2Capture capture_;
    ScannerParams params_;
    std::string lastError_;
};

void writeLe32(uint8_t* output, int32_t value) {
    const uint32_t bits = static_cast<uint32_t>(value);
    output[0] = static_cast<uint8_t>(bits & 0xff);
    output[1] = static_cast<uint8_t>((bits >> 8) & 0xff);
    output[2] = static_cast<uint8_t>((bits >> 16) & 0xff);
    output[3] = static_cast<uint8_t>((bits >> 24) & 0xff);
}

} // namespace

namespace pop4 {

std::array<uint8_t, 4> makeEnablePayload() {
    return {1, 0, 0, 0};
}

std::array<uint8_t, 60> makeSettingsPayload(int32_t gain,
                                            int32_t exposure,
                                            int32_t irLum,
                                            int32_t crossLaserLight,
                                            int32_t singleLaserLight) {
    std::array<uint8_t, 60> payload{};
    const std::array<int32_t, 5> values{
        gain, exposure, irLum, crossLaserLight, singleLaserLight};
    for (size_t i = 0; i < values.size(); ++i) {
        writeLe32(payload.data() + i * 4, values[i]);
    }
    return payload;
}

} // namespace pop4

std::unique_ptr<IScannerProtocol> makeRevopointPop4Protocol() {
    return std::make_unique<RevopointPop4Protocol>();
}

} // namespace revoscan::protocol
