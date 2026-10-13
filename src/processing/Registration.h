#pragma once

#include <array>
#include <cstddef>
#include <memory>

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

// Point-to-plane ICP (Gauss-Newton on n·(Rp + t − q)). Target normals come
// from a PCA plane fit over neighbours within `normalRadiusM`; points without
// a well-defined plane are skipped. Locks onto flat faces where point-to-point
// slides, which matters for box-like objects. Fitness/RMSE are reported with
// the same point-distance semantics as icpPointToPoint.
struct PlaneIcpOptions : IcpOptions {
    float normalRadiusM = 0.006f;
    std::size_t minNormalNeighbours = 6;
};

IcpResult icpPointToPlane(const pointcloud::PointCloud& source,
                          const pointcloud::PointCloud& target,
                          const RigidTransform& init = {},
                          const PlaneIcpOptions& opts = {});

// A point-to-plane target prepared once — k-d tree plus PCA normals — and
// reusable across several ICP calls against the same model (tight, wide and
// relocalisation passes within one frame). Building it is most of the cost.
class PlaneIcpTarget {
public:
    PlaneIcpTarget(pointcloud::PointCloud cloud, float normalRadiusM,
                   std::size_t minNormalNeighbours = 6);
    ~PlaneIcpTarget();
    PlaneIcpTarget(const PlaneIcpTarget&) = delete;
    PlaneIcpTarget& operator=(const PlaneIcpTarget&) = delete;

    const pointcloud::PointCloud& cloud() const;
    struct Impl;
    const Impl& impl() const { return *impl_; }

private:
    std::unique_ptr<Impl> impl_;
};

IcpResult icpPointToPlane(const pointcloud::PointCloud& source,
                          const PlaneIcpTarget& target,
                          const RigidTransform& init = {},
                          const IcpOptions& opts = {});

} // namespace revoscan::processing
