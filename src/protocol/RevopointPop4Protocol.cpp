#include "RevopointPop4Protocol.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>

#include <libusb-1.0/libusb.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

#include "ScannerProtocol.h"
#include "decoder/Pop4DepthDecoder.h"
#include "usb/UsbDevice.h"
#include "usb/V4l2Capture.h"

namespace revoscan::protocol {
namespace {

constexpr uint8_t kUvcSetRequestType = 0x21;
constexpr uint8_t kUvcSetCur = 0x01;
constexpr int kControlTimeoutMs = 500;

// Ranges / defaults sourced from RevoScan APK param_pop4.json (model 0E1).
constexpr int kExposureMin = 500;
constexpr int kExposureMax = 6000;
constexpr int kExposureDefault = 3000;
constexpr int kIllumMin = 0;
constexpr int kIllumMax = 255;
constexpr int kIllumDefault = 130;
constexpr int kLaserMin = 0;
constexpr int kLaserMax = 255;
constexpr int kLaserDefault = 180;
constexpr int kCs7CrossDefault = 780;
constexpr int kCs7SingleDefault = 706;

uint16_t xuValue(uint8_t selector) {
    return static_cast<uint16_t>(selector) << 8;
}

uint16_t xuIndex(uint8_t unitId, uint8_t interfaceNumber) {
    return (static_cast<uint16_t>(unitId) << 8) | interfaceNumber;
}

int mapLaserUiToCross(int laserUi) {
    // UI uses 0..255 (APK laserLuminance). CS7 cross field was empirically ~780.
    return std::clamp((laserUi * kCs7CrossDefault) / kLaserMax, 0, 2000);
}

int mapLaserUiToSingle(int laserUi) {
    return std::clamp((laserUi * kCs7SingleDefault) / kLaserMax, 0, 2000);
}

class RevopointPop4Protocol final : public IScannerProtocol {
public:
    RevopointPop4Protocol() {
        controls_.exposure = kExposureDefault;
        controls_.illuminationPower = kIllumDefault;
        controls_.laserPower = kLaserDefault;
        controls_.mode = ScanMode::StructuredLight;
        controls_.emittersOn = false;
        calibration_ = decoder::makeApproximatePop4FullFieldCalibration();
    }

    bool connect() override {
        disconnect();
        lastError_.clear();
        // Re-install approximate calib each connect until factory camparam lands.
        calibration_ = decoder::makeApproximatePop4FullFieldCalibration();

        // At most one USB reset per connect: every reset reboots the POP4,
        // and stacking them (as the old 1.5 s retry did) caused reset storms.
        bool resetUsed = false;
        if (!armProjector()) {
            // After a previous session the depth XU can reject CS10 with
            // LIBUSB_ERROR_IO until the device is reset.
            const std::string firstError = lastError_;
            resetUsed = true;
            if (!resetAndRearm()) {
                lastError_ = firstError + " (USB reset retry: " + lastError_ + ")";
                disconnect();
                return false;
            }
        }

        // Prefer packed 640×2560; fall back to 640×512 if the node rejects it.
        captureWidth_ = pop4::kCaptureWidth;
        captureHeight_ = 0;
        std::optional<std::string> node = findDepthCaptureNode();
        if (!node) {
            // uvcvideo often stays "bound" after XU claim without recreating
            // /dev/video* for the depth IAD. Force a sysfs unbind/bind.
            usb::rebindUvcvideoForUsbDevice(pop4::kVendorId, pop4::kProductId);
            node = findDepthCaptureNode();
        }
        if (!node && !resetUsed && resetAndRearm()) {
            // Last resort: USB port reset (device reboots), re-arm, look again.
            node = findDepthCaptureNode();
        }
        if (!node || captureHeight_ == 0) {
            lastError_ =
                "POP4 armed, but no 640x2560/640x512 Y16 V4L2 capture node "
                "was found (depth camera stuck — unplug/replug the scanner "
                "or reconnect after the app rebinds uvcvideo)";
            disconnect();
            return false;
        }

        if (!capture_.open(*node,
                           V4L2_PIX_FMT_Y16,
                           captureWidth_,
                           captureHeight_)) {
            lastError_ = capture_.lastError();
            disconnect();
            return false;
        }

        // Best-effort RGB preview (MJPEG 1280×1024 on POP4-REVO-RGBCam).
        rgbCapture_.close();
        if (auto rgbNode = usb::findV4l2CaptureNodeNamed(
                pop4::kVendorId,
                pop4::kProductId,
                V4L2_PIX_FMT_MJPEG,
                1280,
                1024,
                "RGB")) {
            if (!rgbCapture_.open(*rgbNode, V4L2_PIX_FMT_MJPEG, 1280, 1024, 2)) {
                rgbCapture_.close(); // preview is optional
            }
        }

        controls_.emittersOn = true;
        controls_.illuminationPower = kIllumDefault;
        controls_.laserPower = kLaserDefault;
        controls_.exposure = kExposureDefault;
        state_ = ConnectionState::Connected;
        return true;
    }

    void disconnect() override {
        rgbCapture_.stop();
        rgbCapture_.close();
        capture_.close();
        usb_.close();
        controls_.emittersOn = false;
        captureHeight_ = 0;
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
        if (rgbCapture_.isOpen()) {
            (void)rgbCapture_.start(); // optional; depth still works if this fails
        }
        state_ = ConnectionState::Streaming;
        return true;
    }

    void stopStream() override {
        rgbCapture_.stop();
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

        const size_t expectedBytes =
            static_cast<size_t>(captureWidth_) * captureHeight_ * 2;
        if (sensor->bytes.size() < expectedBytes || captureHeight_ == 0) {
            lastError_ = "POP4 returned a truncated Y16 frame";
            return std::nullopt;
        }

        RawFrame frame;
        frame.width = captureWidth_;
        frame.height = captureHeight_;
        frame.timestampUs = sensor->timestampUs;
        frame.encoding = DepthEncoding::VendorRawY16;
        frame.depthMm.resize(expectedBytes / 2);
        for (size_t i = 0; i < frame.depthMm.size(); ++i) {
            frame.depthMm[i] =
                static_cast<uint16_t>(sensor->bytes[i * 2]) |
                (static_cast<uint16_t>(sensor->bytes[i * 2 + 1]) << 8);
        }
        dumpRawFrame(*sensor, expectedBytes);

        decoder_.setCalibration(calibration_);
        if (auto metric = decoder_.decode(frame)) {
            return metric;
        }
        // Keep last decode reason for --probe-pop4 / diagnostics; preview path
        // still receives the raw packed frame.
        if (!decoder_.lastError().empty()) {
            lastError_ = decoder_.lastError();
        }
        return frame;
    }

    std::optional<std::vector<uint8_t>> readRgbJpegPreview(int timeoutMs) override {
        if (state_ != ConnectionState::Streaming || !rgbCapture_.isOpen()) {
            return std::nullopt;
        }
        auto frame = rgbCapture_.readFrame(timeoutMs);
        if (!frame || frame->bytes.empty()) {
            return std::nullopt;
        }
        return std::move(frame->bytes);
    }

    void setParams(const ScannerParams& params) override { params_ = params; }

    CameraIntrinsics intrinsics() const override {
        return calibration_.hasDepthIntrinsics() ? calibration_.depth
                                                 : CameraIntrinsics{};
    }

    ScannerCalibration calibration() const override { return calibration_; }

    void setDepthRangeHint(float nearMm, float farMm) override {
        decoder_.setDepthRange(nearMm, farMm);
    }

    DepthDecodeStats depthDecodeStats() const override {
        return decoder_.lastStats();
    }

    ScannerControlCapabilities controlCapabilities() const override {
        ScannerControlCapabilities caps;
        caps.illuminationPower = true;
        caps.laserPower = true;
        caps.exposure = true;
        caps.scanMode = true;
        caps.alignmentGuide = true;
        caps.emitterOff = true;
        caps.illuminationMin = kIllumMin;
        caps.illuminationMax = kIllumMax;
        caps.laserMin = kLaserMin;
        caps.laserMax = kLaserMax;
        caps.exposureMin = kExposureMin;
        caps.exposureMax = kExposureMax;
        return caps;
    }

    ScannerControlState controlState() const override { return controls_; }

    bool setScanMode(ScanMode mode) override {
        controls_.mode = mode;
        if (mode == ScanMode::BlueLaser) {
            controls_.laserPower = 255;
            controls_.illuminationPower = 255;
            controls_.exposure = 4500;
        } else {
            controls_.laserPower = kLaserDefault;
            controls_.illuminationPower = kIllumDefault;
            controls_.exposure = kExposureDefault;
        }
        return applyCs7Settings();
    }

    bool setAlignmentGuide(bool on) override {
        controls_.alignmentGuideOn = on;
        if (on) {
            controls_.illuminationPower = 255;
            controls_.exposure = 5500;
        } else {
            controls_.illuminationPower = kIllumDefault;
            controls_.exposure = kExposureDefault;
        }
        return applyCs7Settings();
    }

    bool setIlluminationPower(int value) override {
        controls_.illuminationPower = std::clamp(value, kIllumMin, kIllumMax);
        return applyCs7Settings();
    }

    bool setLaserPower(int value) override {
        controls_.laserPower = std::clamp(value, kLaserMin, kLaserMax);
        return applyCs7Settings();
    }

    bool setExposure(int value) override {
        controls_.exposure = std::clamp(value, kExposureMin, kExposureMax);
        return applyCs7Settings();
    }

    bool emittersOff() override {
        controls_.emittersOn = false;
        controls_.illuminationPower = 0;
        controls_.laserPower = 0;
        return applyCs7Settings();
    }

    std::string backendName() const override {
        const std::string decodePath = decoder::Pop4DepthDecoder::backendName();
        if (calibration_.source == "ApproximatePop4FullField") {
            return "Revopoint POP4 (full-field approx mm, " + decodePath + ")";
        }
        return calibration_.hasStereoCalibration()
                   ? ("Revopoint POP4 (metric decode, " + decodePath + ")")
                   : "Revopoint POP4 (native raw Y16)";
    }
    std::string lastError() const override { return lastError_; }

private:
    // ALL3DSCAN_POP4_DUMP_DIR=<dir> writes the first 30 packed Y16 frames as
    // pop4_<w>x<h>_NNN.y16 (little-endian) for offline decoder work.
    void dumpRawFrame(const usb::V4l2Frame& sensor, size_t bytes) {
        const char* dir = std::getenv("ALL3DSCAN_POP4_DUMP_DIR");
        if (!dir || dumpCount_ >= 30) {
            return;
        }
        char name[64];
        std::snprintf(name, sizeof(name), "/pop4_%ux%u_%03d.y16", captureWidth_,
                      captureHeight_, dumpCount_++);
        if (FILE* f = std::fopen((std::string(dir) + name).c_str(), "wb")) {
            std::fwrite(sensor.bytes.data(), 1, bytes, f);
            std::fclose(f);
        }
    }

    std::array<uint8_t, 60> currentSettingsPayload() const {
        return pop4::makeSettingsPayload(
            1,
            controls_.exposure,
            controls_.illuminationPower,
            mapLaserUiToCross(controls_.laserPower),
            mapLaserUiToSingle(controls_.laserPower));
    }

    bool reopenCapture(bool wasStreaming) {
        auto node = usb::findV4l2CaptureNode(
            pop4::kVendorId,
            pop4::kProductId,
            V4L2_PIX_FMT_Y16,
            captureWidth_,
            captureHeight_);
        if (!node && captureHeight_ == pop4::kCaptureHeightPacked) {
            node = usb::findV4l2CaptureNode(
                pop4::kVendorId,
                pop4::kProductId,
                V4L2_PIX_FMT_Y16,
                pop4::kCaptureWidth,
                pop4::kCaptureHeightFallback);
            if (node) {
                captureHeight_ = pop4::kCaptureHeightFallback;
            }
        }
        if (!node || captureHeight_ == 0 ||
            !capture_.open(*node, V4L2_PIX_FMT_Y16, captureWidth_, captureHeight_)) {
            return false;
        }
        if (wasStreaming) {
            if (!capture_.start()) {
                lastError_ = capture_.lastError();
                state_ = ConnectionState::Connected;
                return false;
            }
            state_ = ConnectionState::Streaming;
        }
        return true;
    }

    bool applyCs7Settings() {
        if (state_ == ConnectionState::Disconnected) {
            // Values are stored and used on the next connect/arm.
            return true;
        }

        const bool wasStreaming = (state_ == ConnectionState::Streaming);
        if (wasStreaming) {
            capture_.stop();
        }
        if (capture_.isOpen()) {
            capture_.close();
        }
        if (rgbCapture_.isOpen()) {
            rgbCapture_.close();
        }

        const bool ok = writeCs7Only();

        if (auto node = findDepthCaptureNode()) {
            capture_.open(*node, V4L2_PIX_FMT_Y16, captureWidth_, pop4::kCaptureHeight);
            if (wasStreaming) {
                capture_.start();
            }
        }
        return ok;
    }

    bool writeCs7Only() {
        if (!usb_.open(pop4::kVendorId, pop4::kProductId)) {
            lastError_ = "POP4 not found while applying CS7 settings";
            return false;
        }
        bool detached = false;
        if (usb_.kernelDriverActive(pop4::kDepthInterface) == 1) {
            usb_.detachKernelDriver(pop4::kDepthInterface);
            detached = true;
        }
        usb_.claimInterface(pop4::kDepthInterface);

        auto settings = currentSettingsPayload();
        int rc = usb_.controlTransfer(
            kUvcSetRequestType,
            kUvcSetCur,
            xuValue(pop4::kSettingsSelector),
            xuIndex(pop4::kXuUnitId, pop4::kDepthInterface),
            settings.data(),
            settings.size(),
            kControlTimeoutMs);

        auto enable = pop4::makeEnablePayload();
        usb_.controlTransfer(
            kUvcSetRequestType,
            kUvcSetCur,
            xuValue(pop4::kEnableSelector),
            xuIndex(pop4::kXuUnitId, pop4::kDepthInterface),
            enable.data(),
            enable.size(),
            kControlTimeoutMs);

        usb_.releaseInterface();
        if (detached) {
            usb_.attachKernelDriver(pop4::kDepthInterface);
        }
        usb_.close();
        return rc == static_cast<int>(settings.size());
    }

    // Port-resets the POP4, waits for it to reboot and re-enumerate (~37 s
    // measured), lets udev apply access ACLs, then arms again.
    bool resetAndRearm() {
        usb_.close();
        if (!usb_.open(pop4::kVendorId, pop4::kProductId)) {
            lastError_ = "Revopoint POP4 2207:110c not found or permission denied";
            return false;
        }
        usb_.resetDevice();
        std::this_thread::sleep_for(std::chrono::seconds(3));
        for (int waited = 3; waited < 60; ++waited) {
            if (usb_.open(pop4::kVendorId, pop4::kProductId)) {
                usb_.close();
                std::this_thread::sleep_for(std::chrono::seconds(2)); // udev ACLs
                return armProjector();
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        lastError_ = "POP4 did not come back within 60 s after a USB reset";
        return false;
    }

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
            lastError_ =
                "POP4 CS10 projector-enable transfer failed: " +
                std::string(libusb_error_name(rc));
            finishControlClaim(detached);
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        auto settings = currentSettingsPayload();
        rc = usb_.controlTransfer(
            kUvcSetRequestType,
            kUvcSetCur,
            xuValue(pop4::kSettingsSelector),
            xuIndex(pop4::kXuUnitId, pop4::kDepthInterface),
            settings.data(),
            settings.size(),
            kControlTimeoutMs);
        if (rc != static_cast<int>(settings.size())) {
            lastError_ =
                "POP4 CS7 projector-settings transfer failed: " +
                std::string(libusb_error_name(rc));
            finishControlClaim(detached);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(150));

        if (!finishControlClaim(detached)) {
            return false;
        }
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
            // uvcvideo may auto-bind as soon as the interface is released.
            // libusb then reports BUSY even though the desired state has
            // already been reached.
            if (rc != LIBUSB_SUCCESS &&
                usb_.kernelDriverActive(pop4::kDepthInterface) != 1) {
                lastError_ = "Could not reattach uvcvideo to POP4 depth interface";
                return false;
            }
        }
        return true;
    }

    std::optional<std::string> findDepthCaptureNode() {
        captureHeight_ = 0;
        for (int attempt = 0; attempt < 24; ++attempt) {
            if (auto node = usb::findV4l2CaptureNode(
                    pop4::kVendorId,
                    pop4::kProductId,
                    V4L2_PIX_FMT_Y16,
                    pop4::kCaptureWidth,
                    pop4::kCaptureHeightPacked)) {
                captureHeight_ = pop4::kCaptureHeightPacked;
                return node;
            }
            if (auto node = usb::findV4l2CaptureNode(
                    pop4::kVendorId,
                    pop4::kProductId,
                    V4L2_PIX_FMT_Y16,
                    pop4::kCaptureWidth,
                    pop4::kCaptureHeightFallback)) {
                captureHeight_ = pop4::kCaptureHeightFallback;
                return node;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        return std::nullopt;
    }

    ConnectionState state_ = ConnectionState::Disconnected;
    usb::UsbDevice usb_;
    usb::V4l2Capture capture_;
    usb::V4l2Capture rgbCapture_;
    uint32_t captureWidth_ = pop4::kCaptureWidth;
    uint32_t captureHeight_ = 0;
    int dumpCount_ = 0;
    ScannerParams params_;
    ScannerCalibration calibration_{};
    ScannerControlState controls_{};
    decoder::Pop4DepthDecoder decoder_;
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
