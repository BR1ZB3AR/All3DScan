#include "PointCloud.h"

namespace revoscan::pointcloud {

PointCloud fromDepthFrame(const protocol::RawFrame& frame, const protocol::CameraIntrinsics& intrinsics) {
    PointCloud cloud;
    const size_t expected =
        static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
    if (frame.encoding != protocol::DepthEncoding::MetricMillimeters ||
        frame.width == 0 || frame.height == 0 ||
        frame.depthMm.size() != expected ||
        intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
        return cloud;
    }
    cloud.points.reserve(frame.depthMm.size());

    const bool hasColor = frame.rgb.size() == frame.depthMm.size() * 3;

    for (uint32_t y = 0; y < frame.height; ++y) {
        for (uint32_t x = 0; x < frame.width; ++x) {
            const size_t idx = static_cast<size_t>(y) * frame.width + x;
            const uint16_t depthMm = frame.depthMm[idx];
            if (depthMm == 0) {
                continue;
            }

            const float z = static_cast<float>(depthMm) / 1000.0f; // meters
            const float px = (static_cast<float>(x) - static_cast<float>(intrinsics.cx)) * z / static_cast<float>(intrinsics.fx);
            const float py = (static_cast<float>(y) - static_cast<float>(intrinsics.cy)) * z / static_cast<float>(intrinsics.fy);

            Point p;
            p.x = px;
            p.y = py;
            p.z = z;
            if (hasColor) {
                p.r = frame.rgb[idx * 3 + 0];
                p.g = frame.rgb[idx * 3 + 1];
                p.b = frame.rgb[idx * 3 + 2];
            }
            cloud.points.push_back(p);
        }
    }

    return cloud;
}

} // namespace revoscan::pointcloud
