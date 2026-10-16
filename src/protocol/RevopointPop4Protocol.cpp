#include "RevopointPop4Protocol.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <array>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <thread>

#include <libusb-1.0/libusb.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

#include "ScannerProtocol.h"
#include "decoder/Pop4DepthDecoder.h"
#include "decoder/RgbCalibration.h"
#include "decoder/StereoRig.h"

#ifdef ALL3DSCAN_HAVE_TURBOJPEG
#include <turbojpeg.h>
#endif
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
// RevoScan's POP4 "speckle" presets run a far shorter exposure than the
// "surface" ones (200-1400 us vs 3000-5400): resources/param_pop4.json.
constexpr int kSpeckleExposure = 1400;
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
        loadCalibration();

        // The scanner re-enumerates after a session (and after any USB
        // reset), and the udev ACL lands a moment later still. Connecting in
        // that window used to fail outright; wait for it to come back.
        waitForDevice(std::chrono::seconds(20));

        // Blue-laser speckle mode uses its own factory calibration. Read it
        // now, while nothing is streaming: a device file read detaches
        // uvcvideo, so it must never happen mid-stream.
        blueRig_ = loadRig("/data/camparam_blue/camparam.yaml", "-blue");
        loadRgbCalibration();

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

        captureNode_ = *node;
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
        // Arming always leaves the scanner in infrared. Without this the app
        // could still believe it was in blue-laser mode from an earlier
        // session, and then skip the commands when you picked blue laser
        // again — the laser simply never came on.
        controls_.mode = ScanMode::StructuredLight;
        lightOn_ = {{true, false, false}};
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
        updateColorFrame();
        if (auto metric = decoder_.decode(frame)) {
            colorizeDepth(*metric);
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
        // readFrame() drains this node to texture the depth, so serve the
        // cached frame rather than competing with it for buffers.
        if (auto frame = rgbCapture_.readFrame(timeoutMs)) {
            if (!frame->bytes.empty()) {
                lastColorJpeg_ = std::move(frame->bytes);
            }
        }
        if (lastColorJpeg_.empty()) {
            return std::nullopt;
        }
        return lastColorJpeg_;
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

    // Blue-laser speckle mode vs the default IR structured-light mode. The
    // commands are POP4-specific and verified against the vendor SDK; the
    // exposure/gain pairing follows RevoScan's own POP4 presets (speckle mode
    // is much shorter-exposure than surface mode).
    bool setScanMode(ScanMode mode) override {
        // Only the infrared path is skipped when already there, because it
        // costs a 45 s reboot. The projector commands are idempotent register
        // writes, so picking blue laser (or speckle) always re-sends them:
        // if our idea of the current mode were ever stale, skipping would
        // leave the user with a control that silently does nothing.
        if (mode == ScanMode::StructuredLight && controls_.mode == mode) {
            return true;
        }
        if (mode != ScanMode::StructuredLight) {
            if (!sendDeviceCommands(pop4::scanModeCommands(mode))) {
                return false;
            }
            // The blue laser is a different wavelength from the infrared
            // projector, so it has its own factory calibration (prefetched at
            // connect). Speckle mode is still infrared and keeps the default.
            if (mode == ScanMode::BlueLaser && blueRig_) {
                decoder_.setStereoRig(*blueRig_);
                calibration_ = *decoder_.factoryCalibration();
            }
            controls_.mode = mode;
            controls_.exposure = kSpeckleExposure;
            return true;
        }
        // Leaving speckle mode: "echo s 0x3001 7" (the vendor's other POP4
        // mode value) leaves the projector dead and re-arming does not revive
        // it — only a device reboot does. Verified on hardware 2026-10-14.
        const bool wasStreaming = state_ == ConnectionState::Streaming;
        stopStream();
        capture_.close();
        if (!resetAndRearm()) {
            return false;
        }
        loadCalibration();
        controls_.mode = mode;
        controls_.exposure = kExposureDefault;
        if (!reopenCapture(wasStreaming)) {
            return false;
        }
        return true;
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
        // Verified device commands: projector off + fill light off.
        bool ok = sendDeviceCommands(pop4::lightSwitchCommands(Light::Projector, false));
        ok = sendDeviceCommands(pop4::lightSwitchCommands(Light::FillLight, false)) && ok;
        controls_.emittersOn = !ok ? controls_.emittersOn : false;
        return ok;
    }

    bool supportsLight(Light) const override { return true; }

    bool setLight(Light light, bool on) override {
        if (!sendDeviceCommands(pop4::lightSwitchCommands(light, on))) {
            return false;
        }
        lightOn_[static_cast<size_t>(light)] = on;
        if (light == Light::FillLight && on) {
            lightLevel_[static_cast<size_t>(light)] = 255; // the on sequence sets 0xb12 255
        }
        return true;
    }

    bool setLightLevel(Light light, int level) override {
        const int clamped = std::clamp(level, 0, 255);
        if (!sendDeviceCommands({pop4::lightLevelCommand(light, clamped)})) {
            return false;
        }
        lightLevel_[static_cast<size_t>(light)] = clamped;
        return true;
    }

    bool lightIsOn(Light light) const override {
        return lightOn_[static_cast<size_t>(light)];
    }

    int lightLevel(Light light) const override {
        return lightLevel_[static_cast<size_t>(light)];
    }

    std::string backendName() const override {
        const std::string decodePath = decoder::Pop4DepthDecoder::backendName();
        if (calibration_.source == "FactoryCamparam") {
            return "Revopoint POP4 (factory calibration, " + decodePath + ")";
        }
        if (calibration_.source == "ApproximatePop4FullField") {
            return "Revopoint POP4 (full-field approx mm, " + decodePath + ")";
        }
        return calibration_.hasStereoCalibration()
                   ? ("Revopoint POP4 (metric decode, " + decodePath + ")")
                   : "Revopoint POP4 (native raw Y16)";
    }
    std::string lastError() const override { return lastError_; }

private:
    // Factory stereo calibration, in order of preference:
    //  1. ALL3DSCAN_CAMPARAM=<camparam.yaml> (offline / override)
    //  2. /data/camparam/camparam.yaml read from the scanner (cached per device
    //     under $XDG_CONFIG_HOME/All3DScan/camparam/<deviceID>.yaml)
    //  3. the most recent cached file
    //  4. the approximate FOV/baseline model (legacy fitted rectification)
    // `devicePath` selects which factory calibration to load: the default IR
    // one, or /data/camparam_blue/camparam.yaml for blue-laser speckle mode
    // (the two are calibrated at different wavelengths, so they have different
    // focal lengths and distortion). `tag` keeps their caches apart.
    void loadCalibration(const std::string& devicePath = "/data/camparam/camparam.yaml",
                         const std::string& tag = "") {
        if (auto rig = loadRig(devicePath, tag)) {
            decoder_.setStereoRig(*rig);
            calibration_ = *decoder_.factoryCalibration();
        } else {
            decoder_.clearStereoRig();
            calibration_ = decoder::makeApproximatePop4FullFieldCalibration();
        }
    }

    // Colour-camera calibration: three small blobs beside camparam.yaml,
    // cached per device so later runs work without re-reading the scanner.
    void loadRgbCalibration() {
        namespace fs = std::filesystem;
        const char* xdg = std::getenv("XDG_CONFIG_HOME");
        const char* home = std::getenv("HOME");
        const fs::path dir = xdg && *xdg ? fs::path(xdg) / "All3DScan" / "camparam"
                             : home     ? fs::path(home) / ".config" / "All3DScan" / "camparam"
                                        : fs::path();
        const std::array<const char*, 3> names{{"Prgb.bin", "Distort.bin", "LC_RT.bin"}};
        std::array<std::vector<uint8_t>, 3> blobs;
        const std::string id =
            calibration_.serial.empty() ? std::string("device") : calibration_.serial;
        for (size_t i = 0; i < names.size(); ++i) {
            std::string err;
            if (auto bytes = pop4::readDeviceFile(std::string("/data/camparam/") + names[i], &err)) {
                blobs[i] = std::move(*bytes);
                if (!dir.empty()) {
                    std::error_code ec;
                    fs::create_directories(dir, ec);
                    std::ofstream(dir / (id + "-" + names[i]), std::ios::binary)
                        .write(reinterpret_cast<const char*>(blobs[i].data()),
                               static_cast<std::streamsize>(blobs[i].size()));
                }
            } else if (!dir.empty()) {
                std::ifstream in(dir / (id + "-" + names[i]), std::ios::binary);
                if (in) {
                    blobs[i].assign(std::istreambuf_iterator<char>(in),
                                    std::istreambuf_iterator<char>());
                }
            }
        }
        std::string error;
        rgbCal_ = decoder::parseRgbCalibration(blobs[0], blobs[1], blobs[2], &error);
        if (!rgbCal_ && std::getenv("ALL3DSCAN_POP4_DEBUG")) {
            std::fprintf(stderr, "[POP4] colour calibration unavailable: %s\n", error.c_str());
        }
    }

    std::optional<decoder::StereoRig> loadRig(const std::string& devicePath,
                                              const std::string& tag) {
        namespace fs = std::filesystem;
        std::optional<decoder::StereoRig> rig;
        std::string note;
        if (const char* env = std::getenv(tag.empty() ? "ALL3DSCAN_CAMPARAM"
                                                      : "ALL3DSCAN_CAMPARAM_BLUE")) {
            rig = decoder::loadCamparamFile(env, &note);
        }
        const char* xdg = std::getenv("XDG_CONFIG_HOME");
        const char* home = std::getenv("HOME");
        const fs::path cacheDir = xdg && *xdg ? fs::path(xdg) / "All3DScan" / "camparam"
                                  : home     ? fs::path(home) / ".config" / "All3DScan" / "camparam"
                                             : fs::path();
        if (!rig) {
            std::string err;
            if (const auto bytes = pop4::readDeviceFile(devicePath, &err)) {
                const std::string text(bytes->begin(), bytes->end());
                rig = decoder::parseCamparamYaml(text, &note);
                if (rig && !cacheDir.empty() && !rig->deviceId.empty()) {
                    std::error_code ec;
                    fs::create_directories(cacheDir, ec);
                    std::ofstream(cacheDir / (rig->deviceId + tag + ".yaml"), std::ios::binary) << text;
                }
            }
        }
        if (!rig && !cacheDir.empty()) {
            std::error_code ec;
            fs::path newest;
            fs::file_time_type newestTime{};
            for (const auto& e : fs::directory_iterator(cacheDir, ec)) {
                // Only consider caches of the same kind (IR vs blue).
                const bool isBlue = e.path().stem().string().size() > 5 &&
                                    e.path().stem().string().rfind("-blue") ==
                                        e.path().stem().string().size() - 5;
                if (isBlue != !tag.empty()) {
                    continue;
                }
                if (e.path().extension() == ".yaml" && (newest.empty() || e.last_write_time() > newestTime)) {
                    newest = e.path();
                    newestTime = e.last_write_time();
                }
            }
            if (!newest.empty()) {
                rig = decoder::loadCamparamFile(newest.string(), &note);
            }
        }
        return rig;
    }

    // ALL3DSCAN_POP4_DUMP_DIR=<dir> writes the first ALL3DSCAN_POP4_DUMP_MAX
    // (default 30, max 2000) packed Y16 frames as pop4_<w>x<h>_NNN.y16
    // (little-endian, 3.2 MB each) for offline decoder / tracking work.
    void dumpRawFrame(const usb::V4l2Frame& sensor, size_t bytes) {
        const char* dir = std::getenv("ALL3DSCAN_POP4_DUMP_DIR");
        int maxFrames = 30;
        if (const char* m = std::getenv("ALL3DSCAN_POP4_DUMP_MAX")) {
            maxFrames = std::clamp(std::atoi(m), 1, 2000);
        }
        if (!dir || dumpCount_ >= maxFrames) {
            return;
        }
        char name[64];
        std::snprintf(name, sizeof(name), "/pop4_%ux%u_%04d.y16", captureWidth_,
                      captureHeight_, dumpCount_++);
        if (FILE* f = std::fopen((std::string(dir) + name).c_str(), "wb")) {
            std::fwrite(sensor.bytes.data(), 1, bytes, f);
            std::fclose(f);
        }
    }

    std::vector<uint8_t> lastColorJpeg_;        // newest colour frame, as sent
    std::vector<uint8_t> lastColorRgb_;         // and decoded, RGB888
    int colorWidth_ = 0;
    int colorHeight_ = 0;
    size_t lastColouredPixels_ = 0;
    std::optional<decoder::RgbCalibration> rgbCal_;
    std::string captureNode_;                   // which /dev/video* depth node
    std::optional<decoder::StereoRig> blueRig_; // /data/camparam_blue, if present
    // Indexed by Light (Projector, FillLight, IrLed). Arming leaves the
    // projector on and the fill light off.
    std::array<bool, 3> lightOn_{{true, false, false}};
    std::array<int, 3> lightLevel_{{180, 255, 255}};

    // Pulls whatever the colour camera has ready (never blocks) and keeps the
    // JPEG plus its decoded pixels. Both the texture mapping and the preview
    // read this cache, so they cannot steal frames from each other.
    void updateColorFrame() {
        if (!rgbCapture_.isOpen()) {
            return;
        }
        auto frame = rgbCapture_.readFrame(0);
        if (!frame || frame->bytes.empty()) {
            return;
        }
        lastColorJpeg_ = frame->bytes;
#ifdef ALL3DSCAN_HAVE_TURBOJPEG
        if (!colorEnabled()) {
            return;
        }
        tjhandle tj = tj3Init(TJINIT_DECOMPRESS);
        if (!tj) {
            return;
        }
        if (tj3DecompressHeader(tj, lastColorJpeg_.data(), lastColorJpeg_.size()) == 0) {
            const int w = tj3Get(tj, TJPARAM_JPEGWIDTH);
            const int h = tj3Get(tj, TJPARAM_JPEGHEIGHT);
            if (w > 0 && h > 0) {
                std::vector<uint8_t> pixels(static_cast<size_t>(w) * h * 3);
                if (tj3Decompress8(tj, lastColorJpeg_.data(), lastColorJpeg_.size(),
                                   pixels.data(), 0, TJPF_RGB) == 0) {
                    lastColorRgb_ = std::move(pixels);
                    colorWidth_ = w;
                    colorHeight_ = h;
                }
            }
        }
        tj3Destroy(tj);
#endif
    }

    // Replaces the decoder's grey (synthesised from the infrared images) with
    // real colour: every depth pixel is back-projected into the rectified left
    // frame, mapped into the colour camera through the factory LC_RT/Prgb
    // calibration, and sampled.
    void colorizeDepth(RawFrame& metric) {
        if (!rgbCal_ || lastColorRgb_.empty() || !colorEnabled()) {
            return;
        }
        const auto& depth = calibration_.depth;
        if (!(depth.fx > 0.0 && depth.fy > 0.0)) {
            return;
        }
        const size_t pixels = static_cast<size_t>(metric.width) * metric.height;
        if (metric.depthMm.size() != pixels) {
            return;
        }
        std::vector<uint8_t> out(pixels * 3, 0);
        size_t coloured = 0;
        for (uint32_t v = 0; v < metric.height; ++v) {
            for (uint32_t u = 0; u < metric.width; ++u) {
                const size_t i = static_cast<size_t>(v) * metric.width + u;
                const double z = metric.depthMm[i];
                if (z <= 0.0) {
                    continue;
                }
                const double x = (static_cast<double>(u) - depth.cx) * z / depth.fx;
                const double y = (static_cast<double>(v) - depth.cy) * z / depth.fy;
                double cu = 0.0;
                double cv = 0.0;
                if (!decoder::projectToRgb(*rgbCal_, x, y, z, &cu, &cv)) {
                    continue;
                }
                const int su = static_cast<int>(std::lround(cu));
                const int sv = static_cast<int>(std::lround(cv));
                if (su < 0 || su >= colorWidth_ || sv < 0 || sv >= colorHeight_) {
                    continue;
                }
                const size_t src = (static_cast<size_t>(sv) * colorWidth_ + su) * 3;
                out[i * 3 + 0] = lastColorRgb_[src + 0];
                out[i * 3 + 1] = lastColorRgb_[src + 1];
                out[i * 3 + 2] = lastColorRgb_[src + 2];
                ++coloured;
            }
        }
        if (coloured > 0) {
            metric.rgb = std::move(out);
            lastColouredPixels_ = coloured;
        }
    }

    static bool colorEnabled() {
        static const bool on = [] {
            const char* v = std::getenv("ALL3DSCAN_POP4_COLOR");
            return !(v && v[0] == '0');
        }();
        return on;
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
        // After a device reboot the /dev/video* nodes take a moment to come
        // back (and may need a uvcvideo rebind), so use the same retrying
        // search connect() does rather than looking exactly once.
        captureWidth_ = pop4::kCaptureWidth;
        std::optional<std::string> node = findDepthCaptureNode();
        if (!node) {
            usb::rebindUvcvideoForUsbDevice(pop4::kVendorId, pop4::kProductId);
            node = findDepthCaptureNode();
        }
        if (!node || captureHeight_ == 0) {
            lastError_ = "POP4 depth capture node did not reappear";
            return false;
        }
        captureNode_ = *node;
        if (!capture_.open(*node, V4L2_PIX_FMT_Y16, captureWidth_, captureHeight_)) {
            lastError_ = capture_.lastError();
            return false;
        }
        // The colour node is a different /dev/video* that the reboot also
        // took away; without this the texture preview stays dead until the
        // user reconnects.
        rgbCapture_.stop();
        rgbCapture_.close();
        if (auto rgbNode = usb::findV4l2CaptureNodeNamed(pop4::kVendorId, pop4::kProductId,
                                                         V4L2_PIX_FMT_MJPEG, 1280, 1024, "RGB")) {
            if (!rgbCapture_.open(*rgbNode, V4L2_PIX_FMT_MJPEG, 1280, 1024, 2)) {
                rgbCapture_.close(); // preview is optional
            }
        }
        if (wasStreaming) {
            if (!capture_.start()) {
                lastError_ = capture_.lastError();
                state_ = ConnectionState::Connected;
                return false;
            }
            if (rgbCapture_.isOpen()) {
                (void)rgbCapture_.start(); // optional, like startStream()
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
    // Polls until the device can be opened (it is absent while it reboots
    // and root-only for a moment after that). Returns false on timeout.
    // Opens the scanner, waiting while it is absent or still root-only. A
    // rebooting POP4 enumerates, drops and enumerates again several times, so
    // this leaves the handle OPEN on success: closing and reopening in
    // between used to lose that race.
    bool openWithin(std::chrono::seconds timeout) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        for (;;) {
            if (usb_.open(pop4::kVendorId, pop4::kProductId)) {
                return true;
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    bool waitForDevice(std::chrono::seconds timeout) {
        if (!openWithin(timeout)) {
            return false;
        }
        usb_.close();
        return true;
    }

    bool resetAndRearm() {
        usb_.close();
        if (!openWithin(std::chrono::seconds(60))) {
            lastError_ = "Revopoint POP4 2207:110c not found or permission denied";
            return false;
        }
        usb_.resetDevice();
        usb_.close();
        std::this_thread::sleep_for(std::chrono::seconds(3));
        // The device reboots (~40 s) and may flap on the bus before it
        // settles, so keep retrying the arm until it sticks.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
        while (std::chrono::steady_clock::now() < deadline) {
            if (openWithin(std::chrono::seconds(10))) {
                usb_.close();
                std::this_thread::sleep_for(std::chrono::seconds(2)); // udev ACL
                if (armProjector()) {
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        lastError_ = "POP4 did not come back after a USB reset";
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

        applyFringePatternFromEnv();

        if (!finishControlClaim(detached)) {
            return false;
        }
        usb_.close();
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
        return true;
    }

    // Runs a shell command on the scanner the way the vendor SDK's
    // libuvcExecuteCmd does: ASCII, NUL-padded to 60 bytes, UVC SET_CUR on
    // XU unit 4 selector 7 (the same selector as the settings block), 1 s
    // timeout; success means all 60 bytes were accepted.
    bool executeDeviceCommand(const std::string& cmd) {
        std::array<uint8_t, 60> buf{};
        std::memcpy(buf.data(), cmd.data(), std::min(cmd.size(), buf.size() - 1));
        const int rc = usb_.controlTransfer(kUvcSetRequestType,
                                            kUvcSetCur,
                                            xuValue(pop4::kSettingsSelector),
                                            xuIndex(pop4::kXuUnitId, pop4::kDepthInterface),
                                            buf.data(),
                                            buf.size(),
                                            1000);
        return rc == static_cast<int>(buf.size());
    }

    // Sends scanner shell commands in order, pausing like the SDK does. While
    // connected the depth node is open, so they go through uvcvideo and also
    // work mid-stream; nothing is sent when disconnected.
    bool sendDeviceCommands(const std::vector<std::string>& cmds) {
        if (!capture_.isOpen()) {
            lastError_ = "POP4 is not connected";
            return false;
        }
        for (size_t i = 0; i < cmds.size(); ++i) {
            std::array<uint8_t, 60> buf{};
            std::memcpy(buf.data(), cmds[i].data(), std::min(cmds[i].size(), buf.size() - 1));
            std::string error;
            // For the first seconds after the scanner is (re)attached, uvcvideo
            // has not yet built the chain that carries extension unit 4 and
            // answers ENOENT. These are idempotent register writes, so retry
            // briefly instead of failing the user's click.
            bool sent = false;
            for (int attempt = 0; attempt < 20 && !sent; ++attempt) {
                sent = usb::uvcExtensionSetCur(capture_.fd(), pop4::kXuUnitId,
                                               pop4::kSettingsSelector, buf.data(),
                                               static_cast<uint16_t>(buf.size()), &error);
                if (!sent) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }
            }
            if (!sent) {
                lastError_ = "POP4 command \"" + cmds[i] + "\" failed on " +
                             (captureNode_.empty() ? "the depth node" : captureNode_) + ": " +
                             error;
                return false;
            }
            if (std::getenv("ALL3DSCAN_POP4_DEBUG")) {
                std::fprintf(stderr, "[POP4] device cmd \"%s\" -> ok\n", cmds[i].c_str());
            }
            if (i + 1 < cmds.size()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
        return true;
    }

    // EXPERIMENTAL: the SDK's cs::Camera::setFringePattern sends exactly
    // "echo s 0xa05 <type> >/dev/rk_preisp" (types 0..6, validated), or the
    // same to rk_preisp_1 then rk_preisp_2 on dual-ISP models. Opt-in via
    // ALL3DSCAN_POP4_FRINGE=<0..6>; ALL3DSCAN_POP4_PREISP=single|dual|all.
    void applyFringePatternFromEnv() {
        const char* fringe = std::getenv("ALL3DSCAN_POP4_FRINGE");
        if (!fringe) {
            return;
        }
        char* end = nullptr;
        const long type = std::strtol(fringe, &end, 10);
        if (end == fringe || type < 0 || type >= 7) {
            std::fprintf(stderr, "[POP4] ALL3DSCAN_POP4_FRINGE=%s rejected (SDK allows 0..6)\n", fringe);
            return;
        }
        const std::string mode = std::getenv("ALL3DSCAN_POP4_PREISP")
                                     ? std::getenv("ALL3DSCAN_POP4_PREISP")
                                     : "single";
        std::vector<std::string> targets;
        if (mode == "single" || mode == "all") {
            targets.push_back("rk_preisp");
        }
        if (mode == "dual" || mode == "all") {
            targets.push_back("rk_preisp_1");
            targets.push_back("rk_preisp_2");
        }
        for (const auto& target : targets) {
            const std::string cmd = "echo s 0xa05 " + std::to_string(type) + " >/dev/" + target;
            const bool ok = executeDeviceCommand(cmd);
            std::fprintf(stderr, "[POP4] device cmd \"%s\" -> %s\n", cmd.c_str(), ok ? "ok" : "FAILED");
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
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

std::vector<std::string> pop4::lightSwitchCommands(Light light, bool on) {
    switch (light) {
    case Light::Projector:
        // ledAndLaserCtrl ledid 2 ("laser") -> cs helper @0x648920.
        return {on ? "echo s 0xb04 1 > /dev/rk_preisp" : "echo s 0xb04 0 > /dev/rk_preisp"};
    case Light::FillLight:
        // ledid 1 ("RGB led") ctrltype 0; POP_4/POP_4_PRO also set 0xb12 255.
        if (on) {
            return {"echo s 0xb03 1 > /dev/rk_preisp", "echo s 0xb02 1 > /dev/rk_preisp",
                    "echo s 0xb12 255 > /dev/rk_preisp"};
        }
        return {"echo s 0xb03 0 > /dev/rk_preisp"};
    case Light::IrLed:
        // ledid 0 ("IR led"). Accepted by the scanner, no measurable effect.
        return {on ? "echo s 0xb01 1 > /dev/rk_preisp" : "echo s 0xb01 0 > /dev/rk_preisp"};
    }
    return {};
}

std::vector<std::string> pop4::scanModeCommands(ScanMode mode) {
    switch (mode) {
    case ScanMode::BlueLaser:
        return laserLineCommands(true);
    case ScanMode::Speckle:
        // cs::Camera::setSpeckleMode @0x63725c (POP_4 / POP_4_PRO only), with
        // laserLuminance 180 from the vendor's own POP4 speckle presets.
        return {"echo s 0x3001 12 > /dev/rk_preisp", "echo s 0x48d 180 > /dev/rk_preisp"};
    case ScanMode::StructuredLight:
        break;
    }
    // Returning to infrared is done by rebooting the scanner, not by a
    // command: every mode register we know leaves an emitter off instead.
    return {};
}

std::vector<std::string> pop4::laserLineCommands(bool on) {
    if (on) {
        return {"echo s 0x3001 7 > /dev/rk_preisp",
                "echo s 0x3010 0x17 1 > /dev/rk_preisp",
                "echo s 0xb04 1 > /dev/rk_preisp",
                "echo s 0x48d 180 > /dev/rk_preisp"};
    }
    return {"echo s 0x3010 0x17 0 > /dev/rk_preisp"};
}

std::string pop4::lightLevelCommand(Light light, int level) {
    // Laser luminance 0x48d, RGB led 0xb12, IR led 0xb07 (cs helpers @0x64912c,
    // @0x6492b4, @0x649770). Only 0xb12 has a measurable effect on a POP4 in
    // the default infrared mode.
    const char* reg = light == Light::Projector  ? "0x48d"
                      : light == Light::FillLight ? "0xb12"
                                                  : "0xb07";
    return "echo s " + std::string(reg) + " " + std::to_string(std::clamp(level, 0, 255)) +
           " > /dev/rk_preisp";
}

std::optional<std::vector<uint8_t>> pop4::readDeviceFile(const std::string& path,
                                                         std::string* error,
                                                         size_t maxBytes) {
    auto fail = [&](const std::string& msg) -> std::optional<std::vector<uint8_t>> {
        if (error) {
            *error = msg;
        }
        return std::nullopt;
    };
    if (path.empty() || path.size() > 58) {
        return fail("device path must be 1-58 characters");
    }
    usb::UsbDevice dev;
    // The scanner is absent while it re-enumerates after a session, and
    // root-only for a moment after that; wait rather than fail outright.
    for (int waited = 0; !dev.open(kVendorId, kProductId); ++waited) {
        if (waited >= 40) {
            return fail("Revopoint POP4 2207:110c not found or permission denied");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    bool detached = false;
    if (dev.kernelDriverActive(kDepthInterface) == 1) {
        if (dev.detachKernelDriver(kDepthInterface) != LIBUSB_SUCCESS) {
            return fail("could not detach uvcvideo from the depth interface");
        }
        detached = true;
    }
    if (!dev.claimInterface(kDepthInterface)) {
        if (detached) {
            dev.attachKernelDriver(kDepthInterface);
        }
        return fail("could not claim the depth control interface (is the scanner in use?)");
    }
    const uint16_t index = xuIndex(kXuUnitId, kDepthInterface);
    std::array<uint8_t, 60> buf{};
    buf[0] = 0x01; // open for reading
    std::memcpy(buf.data() + 1, path.data(), path.size());
    std::string err;
    std::vector<uint8_t> data;
    int rc = dev.controlTransfer(kUvcSetRequestType, kUvcSetCur, xuValue(1), index, buf.data(),
                                 buf.size(), 1000);
    if (rc < 0) {
        err = std::string("open failed: ") + libusb_error_name(rc);
    } else {
        for (int chunk = 0; chunk < 400000; ++chunk) {
            buf.fill(0);
            rc = dev.controlTransfer(0xa1 /*GET, class, interface*/, 0x81 /*GET_CUR*/, xuValue(2),
                                     index, buf.data(), buf.size(), 1000);
            if (rc <= 0) {
                err = std::string("read failed: ") + (rc < 0 ? libusb_error_name(rc) : "empty");
                break;
            }
            uint32_t len = 0;
            std::memcpy(&len, buf.data(), 4); // little-endian on all supported hosts
            if (len == 0) {
                break; // end of file
            }
            if (len > 56) {
                err = "device reported a chunk longer than 56 bytes";
                break;
            }
            if (data.size() + len > maxBytes) {
                err = "file larger than the read limit";
                break;
            }
            data.insert(data.end(), buf.begin() + 4, buf.begin() + 4 + len);
        }
    }
    buf.fill(0); // close
    dev.controlTransfer(kUvcSetRequestType, kUvcSetCur, xuValue(1), index, buf.data(), buf.size(),
                        1000);
    dev.releaseInterface();
    if (detached) {
        dev.attachKernelDriver(kDepthInterface);
    }
    dev.close();
    if (!err.empty() && data.empty()) {
        return fail(err);
    }
    if (data.empty()) {
        return fail("file is empty or does not exist on the device");
    }
    if (!err.empty() && error) {
        *error = err + " (partial data returned)";
    }
    return data;
}

std::unique_ptr<IScannerProtocol> makeRevopointPop4Protocol() {
    return std::make_unique<RevopointPop4Protocol>();
}

} // namespace revoscan::protocol
