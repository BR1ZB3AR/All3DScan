#include "GeometryExport.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

namespace revoscan::io {
namespace {

float scaleFactor(LengthUnit unit) {
    return unit == LengthUnit::Millimeters ? 1000.0f : 1.0f;
}

bool atomicRename(const std::string& tempPath, const std::string& finalPath) {
    if (std::rename(tempPath.c_str(), finalPath.c_str()) != 0) {
        std::remove(tempPath.c_str());
        return false;
    }
    return true;
}

ExportResult rejectNonMetric(bool metric, bool requireMetric) {
    if (requireMetric && !metric) {
        return {false, "refusing to export non-metric / vendor-raw preview geometry"};
    }
    return {true, {}};
}

bool finitePoint(const pointcloud::Point& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

} // namespace

ExportResult writePlyCloud(const std::string& path,
                           const pointcloud::PointCloud& cloud,
                           bool metric,
                           const ExportOptions& options) {
    if (auto gate = rejectNonMetric(metric, options.requireMetric); !gate.ok) {
        return gate;
    }
    const std::string temp = path + ".tmp";
    std::ofstream out(temp, std::ios::binary);
    if (!out.is_open()) {
        return {false, "failed to open temporary export file"};
    }
    const float s = scaleFactor(options.unit);
    out << "ply\nformat ascii 1.0\n"
        << "comment all3dscan metric=" << (metric ? "1" : "0") << "\n"
        << "element vertex " << cloud.points.size() << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        << "end_header\n";
    for (const auto& p : cloud.points) {
        if (!finitePoint(p)) {
            out.close();
            std::remove(temp.c_str());
            return {false, "cloud contains non-finite coordinates"};
        }
        out << p.x * s << ' ' << p.y * s << ' ' << p.z * s << ' '
            << static_cast<int>(p.r) << ' ' << static_cast<int>(p.g) << ' '
            << static_cast<int>(p.b) << '\n';
    }
    out.close();
    if (!out || !atomicRename(temp, path)) {
        return {false, "failed to finalize PLY export"};
    }
    return {true, {}};
}

ExportResult writePlyMesh(const std::string& path,
                          const geometry::TriangleMesh& mesh,
                          const ExportOptions& options) {
    if (auto gate = rejectNonMetric(mesh.metric, options.requireMetric); !gate.ok) {
        return gate;
    }
    if (mesh.empty()) {
        return {false, "mesh is empty"};
    }
    const std::string temp = path + ".tmp";
    std::ofstream out(temp);
    if (!out.is_open()) {
        return {false, "failed to open temporary export file"};
    }
    const float s = scaleFactor(options.unit);
    out << "ply\nformat ascii 1.0\n"
        << "element vertex " << mesh.vertices.size() << "\n"
        << "property float x\nproperty float y\nproperty float z\n";
    if (mesh.hasColors) {
        out << "property uchar red\nproperty uchar green\nproperty uchar blue\n";
    }
    out << "element face " << mesh.triangleCount() << "\n"
        << "property list uchar int vertex_indices\n"
        << "end_header\n";
    for (const auto& p : mesh.vertices) {
        out << p.x * s << ' ' << p.y * s << ' ' << p.z * s;
        if (mesh.hasColors) {
            out << ' ' << static_cast<int>(p.r) << ' ' << static_cast<int>(p.g) << ' '
                << static_cast<int>(p.b);
        }
        out << '\n';
    }
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        out << "3 " << mesh.indices[i] << ' ' << mesh.indices[i + 1] << ' '
            << mesh.indices[i + 2] << '\n';
    }
    out.close();
    if (!out || !atomicRename(temp, path)) {
        return {false, "failed to finalize mesh PLY"};
    }
    return {true, {}};
}

ExportResult writeObjMesh(const std::string& path,
                          const geometry::TriangleMesh& mesh,
                          const ExportOptions& options) {
    if (auto gate = rejectNonMetric(mesh.metric, options.requireMetric); !gate.ok) {
        return gate;
    }
    if (mesh.empty()) {
        return {false, "mesh is empty"};
    }
    const std::string temp = path + ".tmp";
    std::ofstream out(temp);
    if (!out.is_open()) {
        return {false, "failed to open temporary export file"};
    }
    const float s = scaleFactor(options.unit);
    for (const auto& p : mesh.vertices) {
        out << "v " << p.x * s << ' ' << p.y * s << ' ' << p.z * s << '\n';
    }
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        // OBJ is 1-based.
        out << "f " << mesh.indices[i] + 1 << ' ' << mesh.indices[i + 1] + 1 << ' '
            << mesh.indices[i + 2] + 1 << '\n';
    }
    out.close();
    if (!out || !atomicRename(temp, path)) {
        return {false, "failed to finalize OBJ"};
    }
    return {true, {}};
}

ExportResult writeStlBinary(const std::string& path,
                            const geometry::TriangleMesh& mesh,
                            const ExportOptions& options) {
    if (auto gate = rejectNonMetric(mesh.metric, options.requireMetric); !gate.ok) {
        return gate;
    }
    if (mesh.empty()) {
        return {false, "STL export requires a triangle mesh"};
    }
    if (mesh.indices.size() % 3 != 0) {
        return {false, "mesh indices are not a multiple of 3"};
    }

    const std::string temp = path + ".tmp";
    std::ofstream out(temp, std::ios::binary);
    if (!out.is_open()) {
        return {false, "failed to open temporary STL file"};
    }

    const float s = scaleFactor(options.unit);
    char header[80] = {};
    const char* tag = "All3DScan binary STL";
    for (size_t i = 0; tag[i] != '\0' && i < sizeof(header); ++i) {
        header[i] = tag[i];
    }
    out.write(header, 80);
    const uint32_t triCount = static_cast<uint32_t>(mesh.triangleCount());
    out.write(reinterpret_cast<const char*>(&triCount), 4);

    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const auto& a = mesh.vertices[mesh.indices[i]];
        const auto& b = mesh.vertices[mesh.indices[i + 1]];
        const auto& c = mesh.vertices[mesh.indices[i + 2]];
        const float ax = a.x * s, ay = a.y * s, az = a.z * s;
        const float bx = b.x * s, by = b.y * s, bz = b.z * s;
        const float cx = c.x * s, cy = c.y * s, cz = c.z * s;
        float ux = bx - ax, uy = by - ay, uz = bz - az;
        float vx = cx - ax, vy = cy - ay, vz = cz - az;
        float nx = uy * vz - uz * vy;
        float ny = uz * vx - ux * vz;
        float nz = ux * vy - uy * vx;
        const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-12f) {
            out.close();
            std::remove(temp.c_str());
            return {false, "degenerate triangle in mesh"};
        }
        nx /= len;
        ny /= len;
        nz /= len;
        float facet[12] = {nx, ny, nz, ax, ay, az, bx, by, bz, cx, cy, cz};
        out.write(reinterpret_cast<const char*>(facet), sizeof(facet));
        const uint16_t attr = 0;
        out.write(reinterpret_cast<const char*>(&attr), 2);
    }
    out.close();
    if (!out || !atomicRename(temp, path)) {
        return {false, "failed to finalize STL"};
    }
    return {true, {}};
}

} // namespace revoscan::io
