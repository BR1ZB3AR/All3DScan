#include "PointCloudExport.h"
#include "bridge/RustBridge.h"

#include <fstream>
#include <vector>

namespace revoscan::io {

bool writePly(const std::string& path, const pointcloud::PointCloud& cloud) {
    std::vector<bridge::RustPoint3D> rustPoints;
    rustPoints.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        rustPoints.push_back({p.x, p.y, p.z, p.r, p.g, p.b});
    }
    return bridge::RustBridge::exportPly(rustPoints, path);
}

bool writeObj(const std::string& path, const pointcloud::PointCloud& cloud) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }

    for (const auto& p : cloud.points) {
        out << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
    }

    return true;
}

} // namespace revoscan::io
