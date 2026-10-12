#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pointcloud/PointCloud.h"

namespace revoscan::geometry {

struct TriangleMesh {
    std::vector<pointcloud::Point> vertices;
    std::vector<uint32_t> indices; // triples
    bool hasNormals = false;
    bool hasColors = false;
    bool metric = false;

    bool empty() const { return vertices.empty() || indices.size() < 3; }
    size_t triangleCount() const { return indices.size() / 3; }
};

struct GeometrySnapshot {
    uint64_t revision = 0;
    pointcloud::PointCloud cloud;
    TriangleMesh mesh;
    bool metric = false;
    std::string label;
};

} // namespace revoscan::geometry
