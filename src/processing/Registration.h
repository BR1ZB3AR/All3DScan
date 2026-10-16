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

// OpenMP team size for parallel work started from the calling thread (a
// per-thread setting; 0 = half the logical CPUs). The decoder and the tracker
// run at the same time: two full-size teams on the same cores stall each
// other's barriers, making ICP ~20x slower than with a smaller team.
// ALL3DSCAN_TRACK_THREADS overrides. Returns the team size used.
int limitParallelThreadsForCallingThread(int threads = 0);

// Rigid inverse (R^T, -R^T t).
RigidTransform inverted(const RigidTransform& t);

// Nearest rotation (Gram-Schmidt) with the same translation. Chained pose
// arithmetic slowly loses orthonormality; R^T stops being the inverse and
// prediction then amplifies the error, so poses are re-projected.
RigidTransform orthonormalized(const RigidTransform& t);

// `delta` with its rotation angle and translation scaled by s (s=1: same
// motion, 0: identity) — for damping a predicted inter-frame motion.
RigidTransform scaledMotion(const RigidTransform& delta, double s);

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

// How well `source`, moved by `pose`, sits on the target surface: the share
// of source points within `toleranceM` of a target point, and the RMS
// distance of those points. Unlike IcpResult::fitness (measured at the ICP
// search radius, tens of mm) this separates a real fit from a frame that
// merely lies near a big model.
struct AlignmentQuality {
    double overlap = 0.0; // [0,1]
    double rmseM = 0.0;
    std::size_t inliers = 0;
};
// Aligns a scan to another one when nothing is known about how they are
// posed relative to each other — the "scan the front, scan the back, line
// them up" case. Frame-to-frame tracking cannot do this: it needs a pose
// guess, and between two separate scans there is none.
//
// Both clouds are centred, then a spread of starting orientations is tried,
// each refined by ICP and scored on the same strict surface-overlap test the
// tracker uses. The best scoring pose wins, so a wrong answer shows up as a
// low overlap rather than a confident mistake.
struct GlobalAlignOptions {
    int yawSteps = 24;            // orientations around the vertical axis
    int tiltSteps = 3;            // tilts away from it, including upright
    float coarseCorrespondenceM = 0.030f;
    float fineCorrespondenceM = 0.008f;
    double toleranceM = 0.004;    // "on the surface" distance when scoring
    // Two gates, because neither alone is enough. Overlap must stay low
    // because two scans of different sides of an object genuinely share only
    // a band. But overlap alone accepts nonsense: a sphere dropped into a box
    // corner scores 0.43, since much of it does sit within a few mm of the
    // faces. What separates them is how well the matched points agree —
    // a real match sits far inside a millimetre, that sphere sits at 2.25 mm.
    double minOverlap = 0.25;
    double maxRmseM = 0.0015;
    std::size_t maxSourcePoints = 4000;
};

struct GlobalAlignResult {
    RigidTransform transform;  // maps source into the target's frame
    AlignmentQuality quality;
    bool ok = false;
    int orientationsTried = 0;
};

GlobalAlignResult alignGlobally(const pointcloud::PointCloud& source,
                                const PlaneIcpTarget& target,
                                const GlobalAlignOptions& opts = {});

AlignmentQuality evaluateAlignment(const pointcloud::PointCloud& source,
                                   const PlaneIcpTarget& target,
                                   const RigidTransform& pose,
                                   double toleranceM);

} // namespace revoscan::processing
