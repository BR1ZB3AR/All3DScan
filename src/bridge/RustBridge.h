#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace revoscan::bridge {

struct RustPoint3D {
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct RustDepthInfo {
    size_t validPoints;
    float minDepthMm;
    float maxDepthMm;
};

struct RustArIntrinsics {
    float fx;
    float fy;
    float cx;
    float cy;
    uint32_t width;
    uint32_t height;
};

struct RustArPoint2D {
    float u;
    float v;
    float depthM;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

class RustBridge {
public:
    static std::string getVersion();
    static std::vector<uint8_t> buildCs10Payload(bool emittersOn);
    static bool analyzeDepthFrame(const uint16_t* rawData, size_t sampleCount, float depthScaleMm, RustDepthInfo& outInfo);
    static bool exportPly(const std::vector<RustPoint3D>& points, const std::string& path);
    static bool projectArPoints(
        const std::vector<RustPoint3D>& points,
        const RustArIntrinsics& intrinsics,
        std::vector<RustArPoint2D>& out2DPoints
    );
};

} // namespace revoscan::bridge
