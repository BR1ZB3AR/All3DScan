#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "protocol/ScannerProtocol.h"

namespace revoscan::decoder {

// Approximate POP4 stereo calibration (FOV-derived focal length, provisional
// baseline). Not factory camparam — source is marked ApproximatePop4FullField.
protocol::ScannerCalibration makeApproximatePop4FullFieldCalibration();

// Right-camera row alignment for the POP4 speckle pair. The device does NOT
// deliver row-aligned stereo: the right image is rolled a few degrees with
// strong keystone (row offset swings about ±10 px across the frame). This
// quadratic maps raw right-image coords to left-image rows:
//
//   u = (xR - 160) / 160,  v = (yR - 256) / 256
//   yL = 256 + 256 * (c0 + c1 u + c2 v + c3 u² + c4 u v + c5 v²)
//
// Fitted from live captures with tools/re_probe/pop4_rectify_fit.py. Units
// differ, so ALL3DSCAN_POP4_RECT="c0,c1,c2,c3,c4,c5" overrides the default.
using RowRectification = std::array<double, 6>;
RowRectification defaultPop4RowRectification();
constexpr RowRectification kIdentityRowRectification{{0, 0, 1, 0, 0, 0}};

// Converts packed POP4 vendor-raw Y16 into metric millimetre depth.
//
// The 640×2560 buffer is ten 320×512 images (row stride 320, not 640):
// images 0–4 come from the right camera and 5–9 from the left. In each group
// the first four are ambient/flood exposures and the fifth carries the
// projected IR speckle. Depth = census + semi-global matching between the two
// speckle images (left = image 9, right = image 4) after row alignment.
class Pop4DepthDecoder {
public:
    Pop4DepthDecoder();

    void setCalibration(const protocol::ScannerCalibration& calibration);
    void setRowRectification(const RowRectification& coeffs);

    // Working-distance band (millimetres). The disparity search range and the
    // final depth clamp are both derived from it via the stereo Q matrix.
    // Clamped to [50, 2000] mm; min must stay below max.
    void setDepthRange(float minMm, float maxMm);

    // Returns a MetricMillimeters RawFrame (320×512, left camera) on success.
    std::optional<protocol::RawFrame> decode(const protocol::RawFrame& rawY16);

    const std::string& lastError() const { return lastError_; }

    // Confidence stats from the most recent decode() call (valid regardless
    // of success/failure so the UI can show why a frame was rejected).
    const protocol::DepthDecodeStats& lastStats() const { return lastStats_; }

    // Last decode's disparity map (320×512, <0 = invalid) for diagnostics.
    const std::vector<float>& lastDisparity() const { return lastDisparity_; }

    static std::string backendName();

    // Test helper: interpret already-metric uint16 buffers using calibration
    // scale / range checks without claiming structured-light knowledge.
    static std::optional<protocol::RawFrame> validateMetricPlane(
        const protocol::RawFrame& metricCandidate,
        const protocol::ScannerCalibration& calibration,
        std::string* errorOut = nullptr);

private:
    void rebuildRectificationMap();

    protocol::ScannerCalibration calibration_;
    RowRectification rect_{};
    std::vector<float> rectMapY_; // per output pixel: source row in raw right image
    std::vector<float> lastDisparity_;
    std::string lastError_;
    protocol::DepthDecodeStats lastStats_;
    float depthMinMm_ = 200.0f;
    float depthMaxMm_ = 650.0f;
};

} // namespace revoscan::decoder
