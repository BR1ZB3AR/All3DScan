#pragma once

#include <string>

#include "geometry/TriangleMesh.h"
#include "pointcloud/PointCloud.h"

namespace revoscan::io {

enum class LengthUnit {
    Meters,
    Millimeters,
};

struct ExportOptions {
    LengthUnit unit = LengthUnit::Meters;
    bool requireMetric = true;
};

struct ExportResult {
    bool ok = false;
    std::string error;
};

ExportResult writePlyCloud(const std::string& path,
                           const pointcloud::PointCloud& cloud,
                           bool metric,
                           const ExportOptions& options = {});

ExportResult writePlyMesh(const std::string& path,
                          const geometry::TriangleMesh& mesh,
                          const ExportOptions& options = {});

ExportResult writeObjMesh(const std::string& path,
                          const geometry::TriangleMesh& mesh,
                          const ExportOptions& options = {});

// Binary STL. Mesh only. Unit must be chosen explicitly (STL has no units).
ExportResult writeStlBinary(const std::string& path,
                            const geometry::TriangleMesh& mesh,
                            const ExportOptions& options);

} // namespace revoscan::io
