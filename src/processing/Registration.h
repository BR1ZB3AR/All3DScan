#pragma once

#include <array>
#include <cstddef>

#include "pointcloud/PointCloud.h"

// Self-contained rigid registration (point-to-point ICP) with no external
// math dependencies. This is the native counterpart to RevoScan's real-time
// frame-to-model tracking (libRPA*): each incoming frame is aligned to the
// growing model and fused. Works on best-effort preview depth today and on
// metric depth once PROPERTY_EXT_DEPTH_SCALE is recovered.
namespace revoscan::processing {

// Row-major 4x4 rigid transform (rotation + translation), identity by default.
struct RigidTransform {
    std::array<double, 16> m{{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};

    static RigidTransform identity() { return RigidTransform{}; }

    // Composition: (this * rhs) applies rhs first, then this.
    RigidTransform operator*(const RigidTransform& rhs) const;

    // Translation magnitude (m) and rotation angle (rad) of this transform.
    double translationNorm() const;
    double rotationAngle() const;
};

void transformInPlace(pointcloud::PointCloud& cloud, const RigidTransform& t);
pointcloud::PointCloud transformed(const pointcloud::PointCloud& cloud,
                                   const RigidTransform& t);

// Voxel-grid downsample (keeps one representative point per occupied voxel).
pointcloud::PointCloud voxelDownsample(const pointcloud::PointCloud& in, float voxel);

struct IcpOptions {
    int maxIterations = 30;
    float maxCorrespondenceDistM = 0.01f;
    double convergenceTransEpsM = 1e-5;
    double convergenceRotEps = 1e-5;
    std::size_t minCorrespondences = 10;
};

struct IcpResult {
    RigidTransform transform;        // maps source into target frame
    double fitness = 0.0;            // inlier fraction of source [0,1]
    double rmseM = 0.0;              // inlier RMSE (m)
    std::size_t correspondences = 0; // inliers at the final iteration
    std::size_t iterations = 0;
    bool converged = false;
};

// Aligns source onto target. `init` seeds the pose (use the previous frame's
// pose for temporal coherence). Returns identity/zero-fitness if either cloud
// is too small to constrain a rigid transform.
IcpResult icpPointToPoint(const pointcloud::PointCloud& source,
                          const pointcloud::PointCloud& target,
                          const RigidTransform& init = {},
                          const IcpOptions& opts = {});

} // namespace revoscan::processing
