#pragma once

#include <cstdint>
#include <vector>

#include "protocol/ScannerProtocol.h"

namespace revoscan::pointcloud {

struct Point {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
};

struct PointCloud {
    std::vector<Point> points;
};

// Back-projects a depth (+ optional RGB) frame into a camera-space point
// cloud using pinhole intrinsics. Points with zero depth are skipped.
PointCloud fromDepthFrame(const protocol::RawFrame& frame, const protocol::CameraIntrinsics& intrinsics);

} // namespace revoscan::pointcloud
