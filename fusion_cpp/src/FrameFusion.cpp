#include "all3dscan/FrameFusion.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "pointcloud/PointCloud.h"
#include "processing/Registration.h"

namespace all3dscan::fusion {

namespace processing = revoscan::processing;
namespace pointcloud = revoscan::pointcloud;

namespace {

// A pose with its translation in millimetres, as the engine's points are.
std::vector<double> poseInMm(const processing::RigidTransform& pose, float mmPerM) {
    std::vector<double> out(pose.m.begin(), pose.m.end());
    out[3] *= mmPerM;
    out[7] *= mmPerM;
    out[11] *= mmPerM;
    return out;
}

// The registration works in metres; the engine hands over millimetres.
constexpr float kMmPerM = 1000.0f;

// Settings the Qt app's pipeline uses (src/processing/ScanPipeline.cpp),
// kept in its units so they can be compared with it directly.
constexpr float kVoxelRatio = 1.8f;         // ICP works on a coarser cloud than the model
constexpr float kCoarseCorrDistM = 0.035f;  // first pass: catches a frame that has moved
constexpr float kFineCorrDistM = 0.008f;    // second pass: settles it
constexpr double kMinOverlap = 0.30;        // share of the frame on the known surface
constexpr double kOverlapToleranceMm = 4.0; // what counts as "on it"
constexpr double kMaxRmseMm = 2.5;          // of those points
constexpr size_t kMaxIcpSourcePoints = 8000;
constexpr size_t kMinSourcePoints = 15;
constexpr int kTargetRefreshFrames = 4; // rebuild the ICP target every N merges

pointcloud::PointCloud toCloud(const float* xyz, uint64_t count, const uint8_t* rgb) {
    pointcloud::PointCloud cloud;
    cloud.points.resize(count);
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        auto& point = cloud.points[i];
        point.x = xyz[3 * i] / kMmPerM;
        point.y = xyz[3 * i + 1] / kMmPerM;
        point.z = xyz[3 * i + 2] / kMmPerM;
        point.r = rgb[3 * i];
        point.g = rgb[3 * i + 1];
        point.b = rgb[3 * i + 2];
    }
    return cloud;
}

// Every nth point, so ICP sees the whole cloud rather than one corner of it.
pointcloud::PointCloud strided(const pointcloud::PointCloud& cloud, size_t most) {
    if (cloud.points.size() <= most) {
        return cloud;
    }
    pointcloud::PointCloud out;
    const size_t step = cloud.points.size() / most;
    out.points.reserve(most + 1);
    for (size_t i = 0; i < cloud.points.size(); i += step) {
        out.points.push_back(cloud.points[i]);
    }
    return out;
}

} // namespace

struct FrameFusion::Impl {
    float modelVoxelM = 0.001f;
    pointcloud::PointCloud model;                           // metres, first frame's axes
    std::unique_ptr<processing::PlaneIcpTarget> target;     // the model, ready for ICP
    int sinceTargetBuilt = 0;
    processing::RigidTransform pose; // where the last merged frame sat

    float icpVoxelM() const { return modelVoxelM * kVoxelRatio; }

    void rebuildTarget() {
        auto coarse = processing::voxelDownsample(model, icpVoxelM());
        target = std::make_unique<processing::PlaneIcpTarget>(
            std::move(coarse), std::max(0.004f, 3.0f * icpVoxelM()));
        sinceTargetBuilt = 0;
    }
};

FrameFusion::FrameFusion(float voxel_mm) : impl_(std::make_unique<Impl>()) {
    if (!(voxel_mm > 0.0f && voxel_mm <= 50.0f)) {
        throw std::invalid_argument("FrameFusion: voxel size must be 0 to 50 mm, got " +
                                    std::to_string(voxel_mm));
    }
    impl_->modelVoxelM = voxel_mm / kMmPerM;
    // ICP's hot loops are OpenMP; a full-size team on every core stalls on
    // its own barriers while the decoder runs beside it.
    processing::limitParallelThreadsForCallingThread();
}

FrameFusion::~FrameFusion() = default;

uint64_t FrameFusion::modelPoints() const noexcept {
    return impl_->model.points.size();
}

void FrameFusion::clear() {
    impl_->model.points.clear();
    impl_->target.reset();
    impl_->sinceTargetBuilt = 0;
    impl_->pose = processing::RigidTransform::identity();
    lastOverlap_ = 0.0;
    lastRmseMm_ = 0.0;
    lastPose_ = poseInMm(processing::RigidTransform::identity(), kMmPerM);
}

bool FrameFusion::add(const float* xyz, uint64_t points, const uint8_t* rgb) {
    if (points > 0 && (xyz == nullptr || rgb == nullptr)) {
        throw std::invalid_argument("FrameFusion: null points or colours");
    }
    lastOverlap_ = 0.0;
    lastRmseMm_ = 0.0;
    pointcloud::PointCloud frame = toCloud(xyz, points, rgb);

    if (impl_->model.points.empty()) {
        // The first frame is the model, and fixes the axes the rest align to.
        impl_->model = processing::voxelDownsample(frame, impl_->modelVoxelM);
        impl_->pose = processing::RigidTransform::identity();
        lastPose_ = poseInMm(impl_->pose, kMmPerM);
        impl_->rebuildTarget();
    } else {
        auto source = processing::voxelDownsample(frame, impl_->icpVoxelM());
        if (source.points.size() < kMinSourcePoints) {
            return false; // too little to say where it goes
        }
        if (!impl_->target) {
            impl_->rebuildTarget();
        }
        // Coarse first, to catch a frame that has moved since the last one,
        // then fine from where that left off. Both start at the last pose.
        processing::IcpOptions coarse;
        coarse.maxCorrespondenceDistM = kCoarseCorrDistM;
        const auto rough = processing::icpPointToPlane(
            strided(source, kMaxIcpSourcePoints), *impl_->target, impl_->pose, coarse);
        processing::IcpOptions fine;
        fine.maxCorrespondenceDistM = kFineCorrDistM;
        const auto settled = processing::icpPointToPlane(
            strided(source, kMaxIcpSourcePoints), *impl_->target, rough.transform, fine);
        const auto pose = processing::orthonormalized(settled.transform);

        // Fitness is measured at the search radius, which is wide; this asks
        // the narrower question of whether the frame sits on the surface.
        const auto quality = processing::evaluateAlignment(source, *impl_->target, pose,
                                                           kOverlapToleranceMm / kMmPerM);
        lastOverlap_ = quality.overlap;
        lastRmseMm_ = quality.rmseM * kMmPerM;
        if (lastOverlap_ < kMinOverlap || lastRmseMm_ > kMaxRmseMm) {
            return false; // lost: leave the model as it is rather than smear it
        }

        auto placed = processing::transformed(frame, pose);
        impl_->model.points.insert(impl_->model.points.end(), placed.points.begin(),
                                   placed.points.end());
        impl_->model = processing::voxelDownsample(impl_->model, impl_->modelVoxelM);
        impl_->pose = pose;
        lastPose_ = poseInMm(pose, kMmPerM);
        if (++impl_->sinceTargetBuilt >= kTargetRefreshFrames) {
            impl_->rebuildTarget();
        }
    }

    return true;
}

std::unique_ptr<FrameFusion> makeFrameFusion(float voxel_mm) {
    return std::make_unique<FrameFusion>(voxel_mm);
}

} // namespace all3dscan::fusion
