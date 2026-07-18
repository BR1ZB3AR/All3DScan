#include "PointCloudExport.h"

#include <fstream>

namespace revoscan::io {

bool writePly(const std::string& path, const pointcloud::PointCloud& cloud) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }

    out << "ply\n"
        << "format ascii 1.0\n"
        << "element vertex " << cloud.points.size() << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "property uchar red\n"
        << "property uchar green\n"
        << "property uchar blue\n"
        << "end_header\n";

    for (const auto& p : cloud.points) {
        out << p.x << ' ' << p.y << ' ' << p.z << ' '
            << static_cast<int>(p.r) << ' ' << static_cast<int>(p.g) << ' ' << static_cast<int>(p.b) << '\n';
    }

    return true;
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
