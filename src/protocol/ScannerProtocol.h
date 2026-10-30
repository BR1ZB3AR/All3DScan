#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace revoscan::protocol {

enum class DepthEncoding {
    MetricMillimeters,
    // Raw vendor sensor words. These must never be projected as metric depth
    // until the model-specific decoder and factory calibration are known.
    VendorRawY16,
};

enum class ScanMode {
    // The default: an infrared projector draws the pattern. Nothing visible.
    StructuredLight,
    // The POP4's visible blue laser. Confirmed on hardware: blue lines appear
    // in the colour camera and to the eye.
    BlueLaser,
    // Infrared speckle mode: same invisible light, much stronger pattern,
    // ~20-30% more depth points.
    Speckle,
};

// One captured frame from the scanner. Depth is in millimeters, row-major,
// width*height entries. rgb is optional (interleaved 8-bit RGB), also
// row-major, unless encoding explicitly marks a vendor-raw diagnostic frame.
struct RawFrame {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint16_t> depthMm;
    // The same depth before it was rounded to whole millimetres: width*height
    // floats, or empty from a producer that has none to give (the raw Y16
    // capture, the mock, a replayed dump). The stereo match knows depth to a
    // fraction of a millimetre, and rounding it away before the point cloud
    // was built was costing about 40% of the per-frame noise.
    std::vector<float> depth;
    std::vector<uint8_t> rgb; // empty if not available
    uint64_t timestampUs = 0;
    DepthEncoding encoding = DepthEncoding::MetricMillimeters;
};

// An 8-bit image, row-major: grey with one channel, or RGB with three.
struct Image {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t channels = 0;
    std::vector<uint8_t> pixels;
};

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

// Per-device calibration required before VendorRawY16 may become metric depth.
// Approximate POP4 full-field defaults may be installed until factory camparam
// is dumped; mark source accordingly (e.g. ApproximatePop4FullField).
struct ScannerCalibration {
    CameraIntrinsics depth;
    std::array<double, 5> depthDistortion{{0, 0, 0, 0, 0}}; // k1,k2,p1,p2,k3
    double depthScale = 1.0; // multiplies decoded depth words → millimetres
    CameraIntrinsics rgb;
    // Row-major 3x4 depth→RGB extrinsic [R|t], metres.
    std::array<double, 12> depthToRgb{{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}};

    // Stereo full-field (left primary view). Optional until camparam is loaded.
    CameraIntrinsics depthRight;
    std::array<double, 5> depthDistortionRight{{0, 0, 0, 0, 0}};
    // Row-major 3x3 rotation and 3-vector translation (metres) from left→right.
    std::array<double, 9> stereoRotation{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    std::array<double, 3> stereoTranslation{{0, 0, 0}};
    // OpenCV-style 4x4 reprojection matrix Q (row-major). Used as
    // Z = Q23 / (disparity * Q32 + Q33), then scaled by depthScale.
    std::array<double, 16> qMatrix{{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};
    double baselineMm = 0.0;

    std::string serial;
    std::string firmware;
    std::string source; // e.g. "device-xu", "ApproximatePop4FullField"
    bool valid = false;

    bool hasDepthIntrinsics() const {
        return valid && depth.fx > 0.0 && depth.fy > 0.0;
    }

    bool hasStereoCalibration() const {
        return hasDepthIntrinsics() && baselineMm > 0.0 &&
               (qMatrix[11] > 1e-12 || qMatrix[11] < -1e-12); // Q(2,3)
    }
};

struct ScannerParams {
    std::vector<std::pair<std::string, double>> values;
};

// Capture-backed POP4 emitter / mode controls. Ranges stay inactive until
// confirmed by USBPcap analysis; implementations must refuse unknown writes.
struct ScannerControlCapabilities {
    bool scanMode = false;
    bool alignmentGuide = false;
    bool illuminationPower = false;
    bool laserPower = false;
    bool exposure = false;
    bool emitterOff = false;
    int illuminationMin = 0;
    int illuminationMax = 0;
    int laserMin = 0;
    int laserMax = 0;
    int exposureMin = 0;
    int exposureMax = 0;
};

struct ScannerControlState {
    ScanMode mode = ScanMode::StructuredLight;
    bool alignmentGuideOn = true;
    int illuminationPower = 130;
    int laserPower = 180;
    int exposure = 3000;
    bool emittersOn = true;
};

// Per-frame confidence report from a structured-light / stereo depth decode.
// Lets the UI show whether the last frame's geometry is trustworthy (dense,
// spread-out disparities) or likely garbage (sparse / collapsed — a sign the
// object is outside the decoder's current search band or the surface lacks
// texture for phase matching).
struct DepthDecodeStats {
    uint32_t validDepth = 0;
    uint32_t totalPixels = 0;
    float dispStdPx = 0.0f; // disparity std-dev in pixels; near-zero = collapsed slab
    std::string backend;    // "CPU" / "CUDA"
    bool attempted = false; // false until at least one decode has run

    float validFraction() const {
        return totalPixels > 0
                   ? static_cast<float>(validDepth) / static_cast<float>(totalPixels)
                   : 0.0f;
    }
};

// The scanner's built-in light sources, named for what they were measured to
// do on a POP4 (2026-10-14), not for what the vendor SDK calls them.
enum class Light {
    // Draws the scan pattern. Infrared: toggling it changes the projected
    // pattern in the depth images but nothing in the colour camera.
    Projector,
    // White/blue LEDs lighting the scene for the colour camera. RevoScan
    // turns these on for marker scanning.
    FillLight,
    // The SDK's separate "IR LED" rail. It exists in the command set but has
    // no measurable effect on a POP4; kept so the mapping stays documented.
    IrLed,
};

enum class ConnectionState {
    Disconnected,
    Connected,
    Streaming,
};

class IScannerProtocol {
public:
    virtual ~IScannerProtocol() = default;

    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual ConnectionState state() const = 0;

    virtual bool startStream() = 0;
    virtual void stopStream() = 0;

    // Blocks until a frame is available or timeout elapses; nullopt on
    // timeout or disconnect.
    virtual std::optional<RawFrame> readFrame(int timeoutMs) = 0;

    virtual void setParams(const ScannerParams& params) = 0;
    virtual CameraIntrinsics intrinsics() const = 0;
    virtual ScannerCalibration calibration() const {
        ScannerCalibration cal;
        cal.depth = intrinsics();
        cal.valid = cal.depth.fx > 0.0 && cal.depth.fy > 0.0;
        cal.source = "intrinsics-only";
        return cal;
    }

    virtual ScannerControlCapabilities controlCapabilities() const {
        return {};
    }
    virtual ScannerControlState controlState() const { return {}; }
    virtual bool setScanMode(ScanMode) { return false; }
    virtual bool setAlignmentGuide(bool) { return false; }
    virtual bool setIlluminationPower(int) { return false; }
    virtual bool setLaserPower(int) { return false; }
    virtual bool setExposure(int) { return false; }
    virtual bool emittersOff() { return false; }

    // Built-in lights. Level is 0..255. Only sent while connected.
    virtual bool supportsLight(Light) const { return false; }
    virtual bool setLight(Light, bool /*on*/) { return false; }
    virtual bool setLightLevel(Light, int /*level*/) { return false; }
    virtual bool lightIsOn(Light) const { return false; }
    virtual int lightLevel(Light) const { return 0; }

    // Working-distance hint (millimetres) for backends that decode depth from
    // a bounded disparity/phase search (e.g. POP4 stereo phase matching). The
    // decode search band follows this hint so close-range presets can find
    // near-surface disparities instead of always searching a fixed default
    // band. No-op for backends without a bounded search (e.g. ToF).
    virtual void setDepthRangeHint(float /*nearMm*/, float /*farMm*/) {}

    // Whether the scanner is still there. False once it has unplugged or
    // re-enumerated mid-stream, where state() alone cannot tell: a stream
    // that was started stays "Streaming" while its device is gone.
    virtual bool stillConnected() const { return state() != ConnectionState::Disconnected; }

    // Whether readFrame() gives the newest frame the camera has ready,
    // skipping older ones, rather than every frame in turn: what a live view
    // wants, where a frame that has waited in the queue is a stale one.
    // Off by default; a no-op for backends without a frame queue.
    virtual void setNewestFrameOnly(bool /*newest*/) {}

    // Confidence of the most recent depth decode; default/empty when the
    // backend doesn't produce a bounded phase/stereo match (mock, ToF, etc.).
    virtual DepthDecodeStats depthDecodeStats() const { return {}; }

    // Optional RGB camera JPEG preview (empty if the backend has no color cam).
    virtual std::optional<std::vector<uint8_t>> readRgbJpegPreview(int /*timeoutMs*/) {
        return std::nullopt;
    }

    // An infrared image from the newest frame, scaled to 8 bits, and the
    // newest decoded colour camera image. Empty when the backend has none
    // (or, for colour, was built without a JPEG decoder).
    virtual Image lastInfrared() const { return {}; }
    virtual Image lastColor() const { return {}; }

    virtual std::string backendName() const = 0;
    virtual std::string lastError() const = 0;
};

std::unique_ptr<IScannerProtocol> makeMockProtocol();
std::unique_ptr<IScannerProtocol> makeRevopointPop4Protocol();

} // namespace revoscan::protocol
