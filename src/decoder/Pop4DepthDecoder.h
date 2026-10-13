#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "decoder/StereoRig.h"
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

// Horizontal disparity bias of the POP4 pair: matched disparity on a flat
// surface bows up by ~24 px at the left/right image edges (differential lens
// distortion along the baseline). Measured on two different planar scenes
// with matching curvature (plane-fit residual 3.9 px -> 0.37 px). In left
// image coords u = (x-160)/160, v = (y-256)/256 the decoder subtracts
//
//   bias = k0 u² + k1 u v + k2 v²
//
// Linear terms are indistinguishable from surface tilt and are not modelled.
// ALL3DSCAN_POP4_DISPX="k0,k1,k2" overrides; "0,0,0" disables.
using DisparityBias = std::array<double, 3>;
DisparityBias defaultPop4DisparityBias();
constexpr DisparityBias kZeroDisparityBias{{0, 0, 0}};

// Per-output-pixel source coordinates for the left and right speckle images.
struct RectifyMaps {
    std::vector<float> lx, ly, rx, ry;
};

// The POP4 delivers its 1280×1024 sensors as 320×512 images (x/4, y/2).
constexpr double kRigScaleX = 0.25;
constexpr double kRigScaleY = 0.5;

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
    void setDisparityBias(const DisparityBias& coeffs);

    // Factory calibration (camparam.yaml from the scanner). When set it
    // replaces the fitted row map / disparity bias / approximate Q entirely:
    // both images are rectified with the factory model and depth uses the
    // factory baseline and focal length.
    void setStereoRig(const StereoRig& rig);
    void clearStereoRig();
    bool hasStereoRig() const { return rig_.has_value(); }
    // Rectified-left intrinsics + Q (shift 0) on the 320×512 grid, if a rig is set.
    std::optional<protocol::ScannerCalibration> factoryCalibration() const;

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
    protocol::ScannerCalibration rigCalibration(double shift) const;

    protocol::ScannerCalibration calibration_;
    RowRectification rect_{};
    RectifyMaps maps_;
    std::optional<StereoRig> rig_;
    std::array<double, 9> rigR1_{}, rigR2_{};
    double rigShift_ = 0.0;
    DisparityBias bias_{};
    std::vector<float> biasMap_;  // per left pixel: disparity bias in px
    float biasMin_ = 0.0f;
    float biasMax_ = 0.0f;
    std::vector<float> lastDisparity_;
    std::string lastError_;
    protocol::DepthDecodeStats lastStats_;
    float depthMinMm_ = 200.0f;
    float depthMaxMm_ = 650.0f;
};

} // namespace revoscan::decoder
