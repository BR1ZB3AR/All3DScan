#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "protocol/ScannerProtocol.h"

namespace revoscan::decoder {

// Approximate POP4 full-field stereo calibration from published FOV
// (46°×37°) and working band (250–500 mm). Not factory camparam — source is
// marked ApproximatePop4FullField.
protocol::ScannerCalibration makeApproximatePop4FullFieldCalibration();

// Converts packed POP4 vendor-raw Y16 (640×2560 = 5×640×512 planes) into
// metric millimetre depth for the top-left 320×256 IR view via four-step phase
// and approximate stereo Q.
class Pop4DepthDecoder {
public:
    void setCalibration(const protocol::ScannerCalibration& calibration);

    // Working-distance band (millimetres) the decoder should search. The
    // disparity search range [dispMin, dispMax] and the final depth clamp are
    // both derived from this each decode() call via the stereo Q matrix, so
    // close-range presets (e.g. 120-219mm) can actually find near-surface
    // disparities instead of always searching the ~220-650mm default band.
    // Clamped to a sane [50, 2000] mm span; min must stay below max.
    void setDepthRange(float minMm, float maxMm);

    // Returns a MetricMillimeters RawFrame on success (320×256 left view).
    // Prefers CUDA stereo when built/available; falls back to CPU.
    std::optional<protocol::RawFrame> decode(const protocol::RawFrame& rawY16);

    const std::string& lastError() const { return lastError_; }

    // Confidence stats from the most recent decode() call (valid regardless
    // of success/failure so the UI can show why a frame was rejected).
    const protocol::DepthDecodeStats& lastStats() const { return lastStats_; }

    // "CUDA" or "CPU" for the active decode path preference.
    static std::string backendName();

    // Test helper: interpret already-metric uint16 buffers using calibration
    // scale / range checks without claiming structured-light knowledge.
    static std::optional<protocol::RawFrame> validateMetricPlane(
        const protocol::RawFrame& metricCandidate,
        const protocol::ScannerCalibration& calibration,
        std::string* errorOut = nullptr);

private:
    protocol::ScannerCalibration calibration_;
    std::string lastError_;
    protocol::DepthDecodeStats lastStats_;
    float depthMinMm_ = 200.0f;
    float depthMaxMm_ = 650.0f;
};

} // namespace revoscan::decoder
