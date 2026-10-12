#pragma once

#include <optional>
#include <string>

#include "pointcloud/PointCloud.h"

namespace revoscan::io {

// Loads an ASCII PLY point cloud (x/y/z + optional uchar RGB), the format
// All3DScan exports. Returns nullopt with errorOut set on failure.
std::optional<pointcloud::PointCloud> readPlyCloud(const std::string& path,
                                                   std::string* errorOut = nullptr);

} // namespace revoscan::io
