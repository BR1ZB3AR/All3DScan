#pragma once

#include <string>

#include "pointcloud/PointCloud.h"

namespace revoscan::io {

// Writes an ASCII PLY point cloud (vertex x/y/z/r/g/b). Returns false on
// file-open failure.
bool writePly(const std::string& path, const pointcloud::PointCloud& cloud);

// Writes an OBJ point cloud as vertex-only "v x y z" lines (no color; OBJ
// vertex color is a non-standard extension some tools don't read, so it's
// omitted for compatibility).
bool writeObj(const std::string& path, const pointcloud::PointCloud& cloud);

} // namespace revoscan::io
