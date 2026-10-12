#include "RustBridge.h"

#include <iostream>

extern "C" {
    const char* rust_core_version();
    void rust_build_cs10_payload(bool emitters_on, uint8_t* out_buffer);
    bool rust_analyze_depth_frame(
        const uint8_t* raw_data,
        size_t data_len,
        float depth_scale_mm,
        size_t* out_valid_points,
        float* out_min_depth,
        float* out_max_depth
    );
    bool rust_export_ply(
        const revoscan::bridge::RustPoint3D* points_ptr,
        size_t points_len,
        const char* output_path
    );
    bool rust_project_ar_points(
        const revoscan::bridge::RustPoint3D* points_ptr,
        size_t points_len,
        const revoscan::bridge::RustArIntrinsics* intrinsics,
        revoscan::bridge::RustArPoint2D* out_2d_buffer,
        size_t buffer_capacity,
        size_t* out_written_count
    );
}

namespace revoscan::bridge {

std::string RustBridge::getVersion() {
    const char* ver = rust_core_version();
    return ver ? std::string(ver) : "0.0.0";
}

std::vector<uint8_t> RustBridge::buildCs10Payload(bool emittersOn) {
    std::vector<uint8_t> buffer(16, 0);
    rust_build_cs10_payload(emittersOn, buffer.data());
    return buffer;
}

bool RustBridge::analyzeDepthFrame(const uint16_t* rawData, size_t sampleCount, float depthScaleMm, RustDepthInfo& outInfo) {
    if (!rawData || sampleCount == 0) return false;
    size_t validPts = 0;
    float minD = 0.0f;
    float maxD = 0.0f;
    bool ok = rust_analyze_depth_frame(
        reinterpret_cast<const uint8_t*>(rawData),
        sampleCount * sizeof(uint16_t),
        depthScaleMm,
        &validPts,
        &minD,
        &maxD
    );
    if (ok) {
        outInfo.validPoints = validPts;
        outInfo.minDepthMm = minD;
        outInfo.maxDepthMm = maxD;
    }
    return ok;
}

bool RustBridge::exportPly(const std::vector<RustPoint3D>& points, const std::string& path) {
    if (points.empty()) return false;
    return rust_export_ply(points.data(), points.size(), path.c_str());
}

bool RustBridge::projectArPoints(
    const std::vector<RustPoint3D>& points,
    const RustArIntrinsics& intrinsics,
    std::vector<RustArPoint2D>& out2DPoints
) {
    out2DPoints.clear();
    if (points.empty()) return true;

    out2DPoints.resize(points.size());
    size_t written = 0;

    bool ok = rust_project_ar_points(
        points.data(),
        points.size(),
        &intrinsics,
        out2DPoints.data(),
        out2DPoints.size(),
        &written
    );

    if (ok) {
        out2DPoints.resize(written);
    } else {
        out2DPoints.clear();
    }
    return ok;
}

} // namespace revoscan::bridge
