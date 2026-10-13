#include "ScanPipeline.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <random>
#include <string>
#include <string_view>
#include <utility>

#include <nlohmann/json.hpp>

#include "bridge/PythonWorker.h"
#include "geometry/TsdfVolume.h"
#include "io/GeometryExport.h"
#include "processing/Registration.h"

namespace revoscan::processing {
namespace {

constexpr size_t kMaxQueue = 6;

// Live-fusion tuning. Default high-precision voxel resolution for consumer
// 3D objects; runtime-adjustable via ScanPipeline::setPointDistance (Worker
// reads modelVoxelM_ each frame). Source/ICP voxels keep the 1.8x ratio over
// the model voxel that this default pair (1.8mm/1.0mm) was tuned with.
constexpr float kDefaultModelVoxelM = 0.0010f;
constexpr float kVoxelRatio = 1.8f; // source & ICP-model voxel = model voxel * ratio
constexpr size_t kMinSourcePoints = 15;
constexpr double kMinFitness = 0.10; // below this → try recovery, then Lost
constexpr double kGoodFitness = 0.25; // above this → coverage tint + keyframe OK
constexpr float kMaxCorrDistM = 0.035f;
constexpr int kIcpTargetRefreshFrames = 4;    // rebuild ICP model every N frames
constexpr size_t kMaxIcpSourcePoints = 8000;  // strided ICP source subset
constexpr float kWideCorrDistM = 0.120f;
constexpr float kMaxRmseM = 0.025f; // reject alignments noisier than ~25 mm
constexpr int kMaxLostBeforeIdle = 15;
constexpr auto kLiveMeshMinInterval = std::chrono::milliseconds(500);
constexpr float kLiveVolumeVoxelM = 0.002f;           // live TSDF resolution
// Taubin rounds: per-frame depth noise is ~3 mm RMS at 30 cm (speckle SGM).
constexpr int kLiveMeshSmoothIterations = 6;
constexpr int kFinalMeshSmoothIterations = 10;
constexpr size_t kLiveVolumeMaxVoxels = 8u * 1000u * 1000u; // ~96 MB
constexpr auto kUiUpdateMinInterval = std::chrono::milliseconds(66); // ~15 Hz view

// ALL3DSCAN_TRACK_PROFILE=1 prints per-stage timings for each frame.
struct StageTimer {
    bool on = false;
    std::chrono::steady_clock::time_point last = std::chrono::steady_clock::now();
    std::string line;
    void mark(const char* name) {
        if (!on) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        char buf[64];
        std::snprintf(buf, sizeof(buf), " %s=%.1f", name,
                      std::chrono::duration<double, std::milli>(now - last).count());
        line += buf;
        last = now;
    }
    ~StageTimer() {
        if (on && !line.empty()) {
            std::fprintf(stderr, "[TRACK-PROFILE]%s\n", line.c_str());
        }
    }
};

// Removes the support surface (desk/turntable) under the object. A plane only
// counts as ground if (1) it is roughly horizontal in camera space (|ny| large;
// image rows run top→bottom, so the desk normal is mostly along y) and (2)
// nearly every off-plane point lies on the camera's side of it — things rest
// *on* the desk. The object's own faces fail (2): a carton's top has its sides
// below it, its front has the background behind it. The previous version
// accepted any plane facing the camera and deleted object faces outright.
pointcloud::PointCloud removeGroundPlaneRansac(const pointcloud::PointCloud& in,
                                               float distanceThresholdM = 0.006f) {
    const size_t n = in.points.size();
    if (n < 100) {
        return in;
    }
    std::mt19937 rng(0x5eed);
    std::uniform_int_distribution<size_t> pick(0, n - 1);

    size_t bestInliers = 0;
    float bestA = 0, bestB = 0, bestC = 0, bestD = 0;
    for (int iter = 0; iter < 120; ++iter) {
        const auto& p1 = in.points[pick(rng)];
        const auto& p2 = in.points[pick(rng)];
        const auto& p3 = in.points[pick(rng)];
        const float v1x = p2.x - p1.x, v1y = p2.y - p1.y, v1z = p2.z - p1.z;
        const float v2x = p3.x - p1.x, v2y = p3.y - p1.y, v2z = p3.z - p1.z;
        float a = v1y * v2z - v1z * v2y;
        float b = v1z * v2x - v1x * v2z;
        float c = v1x * v2y - v1y * v2x;
        const float norm = std::sqrt(a * a + b * b + c * c);
        if (norm < 1e-9f) {
            continue;
        }
        a /= norm;
        b /= norm;
        c /= norm;
        if (std::abs(b) < 0.5f) {
            continue; // not horizontal enough to be the support surface
        }
        const float d = -(a * p1.x + b * p1.y + c * p1.z);

        size_t inliers = 0;
        size_t cameraSide = 0;
        size_t farSide = 0;
        for (size_t i = 0; i < n; i += 4) {
            const auto& q = in.points[i];
            const float dist = a * q.x + b * q.y + c * q.z + d;
            if (std::abs(dist) < distanceThresholdM) {
                ++inliers;
            } else if ((dist > 0.0f) == (d > 0.0f)) {
                ++cameraSide; // camera origin evaluates to d
            } else {
                ++farSide;
            }
        }
        const size_t off = cameraSide + farSide;
        if (off > 0 && farSide * 10 > off) {
            continue; // >10% of the scene beyond it: not what things rest on
        }
        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestA = a;
            bestB = b;
            bestC = c;
            bestD = d;
        }
    }

    if (bestInliers * 4 < n / 10) {
        return in; // no meaningful support plane in view
    }
    pointcloud::PointCloud out;
    out.points.reserve(n);
    for (const auto& p : in.points) {
        const float dist = bestA * p.x + bestB * p.y + bestC * p.z + bestD;
        // Drop the plane and anything just beneath it (camera-side keeps).
        const bool cameraSide = (dist > 0.0f) == (bestD > 0.0f);
        if (cameraSide && std::abs(dist) >= distanceThresholdM) {
            out.points.push_back(p);
        }
    }
    return out.points.size() >= 100 ? out : in;
}

// TSDF keyframe policy: keep the model bounded but multi-view.
// When full, every other keyframe is dropped so the whole scan stays covered
// (the old cap simply stopped, so sides scanned later never reached the mesh).
constexpr size_t kMaxKeyframes = 300;
// Also keep every Nth tracked frame while stationary, so TSDF averages noise.
constexpr int kKeyframeEveryNFrames = 5;
constexpr double kKeyframeMinTransM = 0.008; // 8 mm of motion
constexpr double kKeyframeMinRotRad = 0.05;  // ~2.9 degrees
constexpr size_t kMinTsdfKeyframes = 4;

geometry::TriangleMesh parseAsciiPlyMesh(const std::string& meshPath,
                                         bool* okOut) {
    geometry::TriangleMesh mesh;
    mesh.metric = true;
    if (okOut) {
        *okOut = false;
    }
    std::ifstream in(meshPath);
    if (!in) {
        return mesh;
    }
    std::string line;
    size_t vertexCount = 0;
    size_t faceCount = 0;
    bool headerDone = false;
    while (std::getline(in, line)) {
        if (!headerDone) {
            if (line.rfind("element vertex", 0) == 0) {
                vertexCount = std::stoul(line.substr(15));
            } else if (line.rfind("element face", 0) == 0) {
                faceCount = std::stoul(line.substr(13));
            } else if (line == "end_header") {
                headerDone = true;
                mesh.vertices.reserve(vertexCount);
                mesh.indices.reserve(faceCount * 3);
            }
            continue;
        }
        if (mesh.vertices.size() < vertexCount) {
            std::istringstream ss(line);
            pointcloud::Point p;
            ss >> p.x >> p.y >> p.z;
            if (ss) {
                // Skip optional normals (nx ny nz) then read optional colors.
                float rest[6];
                int got = 0;
                while (got < 6 && (ss >> rest[got])) {
                    ++got;
                }
                if (got >= 3) {
                    // color is the last three parsed fields
                    p.r = static_cast<uint8_t>(std::clamp(rest[got - 3], 0.0f, 255.0f));
                    p.g = static_cast<uint8_t>(std::clamp(rest[got - 2], 0.0f, 255.0f));
                    p.b = static_cast<uint8_t>(std::clamp(rest[got - 1], 0.0f, 255.0f));
                    mesh.hasColors = true;
                }
                mesh.vertices.push_back(p);
            }
        } else {
            std::istringstream ss(line);
            int n = 0;
            uint32_t a = 0, b = 0, c = 0;
            if (ss >> n >> a >> b >> c && n >= 3) {
                mesh.indices.push_back(a);
                mesh.indices.push_back(b);
                mesh.indices.push_back(c);
            }
        }
    }
    if (okOut) {
        *okOut = !mesh.empty();
    }
    return mesh;
}

} // namespace

class ScanPipeline::Worker : public QObject {
    Q_OBJECT
public:
    Worker(document::ScanDocument* document,
           std::atomic<float>* nearClip,
           std::atomic<float>* farClip,
           std::atomic<float>* modelVoxel)
        : document_(document), nearClip_(nearClip), farClip_(farClip),
          modelVoxel_(modelVoxel) {}

    float modelVoxelM() const {
        return modelVoxel_ ? modelVoxel_->load() : kDefaultModelVoxelM;
    }
    float sourceVoxelM() const { return modelVoxelM() * kVoxelRatio; }
    float icpModelVoxelM() const { return modelVoxelM() * kVoxelRatio; }

    void enqueue(pointcloud::PointCloud cloud, uint64_t timestampUs, bool metric) {
        QMutexLocker lock(&mutex_);
        if (queue_.size() >= kMaxQueue) {
            queue_.pop_front();
        }
        queue_.push_back({std::move(cloud), timestampUs, metric, {}, {}, 0, 0, {}, false});
        wait_.wakeOne();
    }

    void enqueueWithDepth(pointcloud::PointCloud cloud, uint64_t timestampUs,
                          std::vector<uint16_t> depthMm, std::vector<uint8_t> rgb,
                          uint32_t width, uint32_t height,
                          protocol::CameraIntrinsics intrinsics) {
        QMutexLocker lock(&mutex_);
        if (queue_.size() >= kMaxQueue) {
            queue_.pop_front();
        }
        queue_.push_back({std::move(cloud), timestampUs, true, std::move(depthMm),
                          std::move(rgb), width, height, intrinsics, true});
        wait_.wakeOne();
    }

    void requestStop() {
        QMutexLocker lock(&mutex_);
        stopping_ = true;
        wait_.wakeOne();
    }

    std::vector<DepthKeyframe> snapshotKeyframes() {
        QMutexLocker lock(&kfMutex_);
        return keyframes_;
    }

    void clearKeyframes() {
        QMutexLocker lock(&kfMutex_);
        keyframes_.clear();
        hasLastKfPose_ = false;
        framesSinceKf_ = 0;
    }

    bool hasLiveMeshSnapshot() {
        QMutexLocker lock(&liveMeshMutex_);
        return !liveMesh_.empty();
    }

    geometry::TriangleMesh snapshotLiveMesh() {
        QMutexLocker lock(&liveMeshMutex_);
        return liveMesh_;
    }

    void clearLiveMesh() {
        QMutexLocker lock(&liveMeshMutex_);
        liveMesh_ = {};
        lastLiveMeshEmit_ = {};
    }

public slots:
    void processLoop() {
        while (true) {
            Item item;
            {
                QMutexLocker lock(&mutex_);
                while (queue_.empty() && !stopping_) {
                    wait_.wait(&mutex_);
                }
                if (stopping_ && queue_.empty()) {
                    break;
                }
                item = std::move(queue_.front());
                queue_.pop_front();
            }
            processFrame(std::move(item));
            processed_.fetch_add(1, std::memory_order_release);
        }
    }

    uint64_t processedCount() const { return processed_.load(std::memory_order_acquire); }
    uint64_t liveMeshRevision() const { return liveMeshRevision_.load(std::memory_order_acquire); }

signals:
    void updated();
    void trackingState(int state);
    void stats(double fitness, int modelPoints, int keyframes);
    void liveMeshReady();

private:
    struct Item {
        pointcloud::PointCloud cloud;
        uint64_t timestampUs = 0;
        bool metric = false;
        std::vector<uint16_t> depthMm;
        std::vector<uint8_t> rgb;
        uint32_t width = 0;
        uint32_t height = 0;
        protocol::CameraIntrinsics intrinsics;
        bool hasDepth = false;
    };

    static void tintCoverage(pointcloud::PointCloud& cloud) {
        // Light green wash over IR albedo — keep shape readable (not a solid wall).
        for (auto& p : cloud.points) {
            const float r = p.r * 0.75f + 18.0f;
            const float g = p.g * 0.65f + 70.0f;
            const float b = p.b * 0.75f + 22.0f;
            p.r = static_cast<uint8_t>(std::clamp(r, 0.0f, 255.0f));
            p.g = static_cast<uint8_t>(std::clamp(g, 0.0f, 255.0f));
            p.b = static_cast<uint8_t>(std::clamp(b, 0.0f, 255.0f));
        }
    }

    void emitLiveStats(double fitness) {
        int kf = 0;
        {
            QMutexLocker lock(&kfMutex_);
            kf = static_cast<int>(keyframes_.size());
        }
        emit stats(fitness, static_cast<int>(document_->cloudSize()), kf);
    }

    void seedModel(const pointcloud::PointCloud& src, bool metric) {
        icpTarget_.reset();
        lastPose_ = RigidTransform::identity();
        hasPose_ = true;
        lostStreak_ = 0;
        pointcloud::PointCloud seed = src;
        tintCoverage(seed);
        document_->fuseCloud(seed, modelVoxelM(), metric, metric ? "fused" : "live");
        emit trackingState(static_cast<int>(TrackingState::Tracking));
        emitLiveStats(1.0);
        emit updated();
    }

    // Live polygon model: each tracked frame's object pixels are fused into a
    // TSDF cube around the object, and a Surface Nets mesh is re-extracted at
    // most every kLiveMeshMinInterval. This replaces a per-frame depth-grid
    // triangulation welded together, which layered noisy copies of every view
    // (and the desk) into a triangle soup instead of one surface.
    void updateLiveMesh(const Item& item, const RigidTransform& pose) {
        if (!item.hasDepth || item.depthMm.empty() || item.width < 2 || item.height < 2 ||
            item.intrinsics.fx <= 0.0 || item.intrinsics.fy <= 0.0 ||
            item.depthMm.size() != static_cast<size_t>(item.width) * item.height) {
            return;
        }
        // Object-only depth: keep the pixels whose point survived isolation and
        // ground removal (item.cloud is still in camera coordinates).
        const auto& k = item.intrinsics;
        std::vector<uint16_t> masked(item.depthMm.size(), 0);
        float lo[3] = {1e9f, 1e9f, 1e9f};
        float hi[3] = {-1e9f, -1e9f, -1e9f};
        for (const auto& p : item.cloud.points) {
            if (p.z <= 0.0f) {
                continue;
            }
            const long u = std::lround(k.fx * p.x / p.z + k.cx);
            const long v = std::lround(k.fy * p.y / p.z + k.cy);
            if (u < 0 || v < 0 || u >= static_cast<long>(item.width) ||
                v >= static_cast<long>(item.height)) {
                continue;
            }
            const size_t i = static_cast<size_t>(v) * item.width + static_cast<size_t>(u);
            masked[i] = item.depthMm[i];
            lo[0] = std::min(lo[0], p.x);
            lo[1] = std::min(lo[1], p.y);
            lo[2] = std::min(lo[2], p.z);
            hi[0] = std::max(hi[0], p.x);
            hi[1] = std::max(hi[1], p.y);
            hi[2] = std::max(hi[2], p.z);
        }
        if (!liveVolume_) {
            if (hi[0] < lo[0]) {
                return;
            }
            // Cube centred on the first view's object, sized to it.
            const double c[3] = {(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5,
                                 (lo[2] + hi[2]) * 0.5};
            const auto& m = pose.m;
            const double wc[3] = {m[0] * c[0] + m[1] * c[1] + m[2] * c[2] + m[3],
                                  m[4] * c[0] + m[5] * c[1] + m[6] * c[2] + m[7],
                                  m[8] * c[0] + m[9] * c[1] + m[10] * c[2] + m[11]};
            const float extent = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
            const float side = std::clamp(1.6f * extent + 0.08f, 0.20f, 0.50f);
            geometry::TsdfParams params;
            params.voxelM = kLiveVolumeVoxelM;
            params.maxVoxels = kLiveVolumeMaxVoxels;
            params.minWeight = 2.0f;
            params.minDepthM = nearClip_ ? nearClip_->load() : 0.10f;
            params.maxDepthM = farClip_ ? farClip_->load() : 0.80f;
            const float h = side * 0.5f;
            liveVolume_ = std::make_unique<geometry::TsdfVolume>(
                std::array<float, 3>{static_cast<float>(wc[0]) - h, static_cast<float>(wc[1]) - h,
                                     static_cast<float>(wc[2]) - h},
                std::array<float, 3>{static_cast<float>(wc[0]) + h, static_cast<float>(wc[1]) + h,
                                     static_cast<float>(wc[2]) + h},
                params);
        }
        liveVolume_->integrate(masked, item.rgb, item.width, item.height, k, pose.m);

        const auto now = std::chrono::steady_clock::now();
        if (lastLiveMeshEmit_.time_since_epoch().count() != 0 &&
            now - lastLiveMeshEmit_ < kLiveMeshMinInterval) {
            return;
        }
        geometry::TriangleMesh mesh = liveVolume_->extractMesh();
        geometry::smoothMeshTaubin(mesh, kLiveMeshSmoothIterations);
        {
            QMutexLocker lock(&liveMeshMutex_);
            liveMesh_ = std::move(mesh);
        }
        liveMeshRevision_.fetch_add(1, std::memory_order_release);
        lastLiveMeshEmit_ = now;
        emit liveMeshReady();
    }

    void maybeCaptureKeyframe(const Item& item, const RigidTransform& pose,
                              double fitness) {
        if (fitness < kGoodFitness) {
            return; // only lock keyframes when alignment is solid
        }
        if (!item.hasDepth || !item.metric || item.depthMm.empty() ||
            item.width == 0 || item.height == 0) {
            return;
        }
        QMutexLocker lock(&kfMutex_);
        ++framesSinceKf_;
        if (hasLastKfPose_ && keyframes_.size() >= 1) {
            const RigidTransform delta = pose * invert(lastKfPose_);
            if (delta.translationNorm() < kKeyframeMinTransM &&
                delta.rotationAngle() < kKeyframeMinRotRad &&
                framesSinceKf_ < kKeyframeEveryNFrames) {
                return; // too close to the previous keyframe
            }
        }
        if (keyframes_.size() >= kMaxKeyframes) {
            std::vector<DepthKeyframe> thinned;
            thinned.reserve(kMaxKeyframes / 2 + 1);
            for (size_t i = 0; i < keyframes_.size(); i += 2) {
                thinned.push_back(std::move(keyframes_[i]));
            }
            keyframes_.swap(thinned);
        }
        framesSinceKf_ = 0;
        DepthKeyframe kf;
        kf.depthMm = item.depthMm;
        if (item.rgb.size() ==
            static_cast<size_t>(item.width) * item.height * 3) {
            kf.rgb = item.rgb;
        }
        kf.width = item.width;
        kf.height = item.height;
        kf.intrinsics = item.intrinsics;
        kf.pose = pose.m;
        keyframes_.push_back(std::move(kf));
        lastKfPose_ = pose;
        hasLastKfPose_ = true;
    }

    // Point-to-plane by default (locks onto flat faces); ALL3DSCAN_ICP=point
    // restores the original point-to-point solver.
    IcpResult icp(const pointcloud::PointCloud& src, const PlaneIcpTarget& model,
                  const RigidTransform& init, const IcpOptions& base) const {
        if (usePointToPoint()) {
            return icpPointToPoint(src, model.cloud(), init, base);
        }
        return icpPointToPlane(src, model, init, base);
    }

    static bool usePointToPoint() {
        const char* v = std::getenv("ALL3DSCAN_ICP");
        return v && (v[0] == 'p' || v[0] == 'P') && std::string_view(v) != "plane";
    }

    IcpResult alignFrame(const pointcloud::PointCloud& fullSrc, StageTimer& timer) {
        // ICP against a coarse model copy (k-d tree + normals), rebuilt every
        // few frames: preparing it dominated per-frame cost, and tracking
        // against a model a few frames old loses nothing measurable.
        if (!icpTarget_ || ++framesSinceTarget_ >= kIcpTargetRefreshFrames) {
            pointcloud::PointCloud coarse;
            document_->withCloud([&](const pointcloud::PointCloud& c) {
                coarse = voxelDownsample(c, icpModelVoxelM());
            });
            icpTarget_ = std::make_unique<PlaneIcpTarget>(
                std::move(coarse), std::max(0.004f, 3.0f * icpModelVoxelM()));
            framesSinceTarget_ = 0;
        }
        const PlaneIcpTarget& model = *icpTarget_;
        timer.mark("modelPrep");

        // An evenly strided subset is plenty to constrain a rigid pose.
        pointcloud::PointCloud strided;
        const pointcloud::PointCloud* srcPtr = &fullSrc;
        if (fullSrc.points.size() > kMaxIcpSourcePoints) {
            const size_t stride = (fullSrc.points.size() + kMaxIcpSourcePoints - 1) /
                                  kMaxIcpSourcePoints;
            strided.points.reserve(fullSrc.points.size() / stride + 1);
            for (size_t i = 0; i < fullSrc.points.size(); i += stride) {
                strided.points.push_back(fullSrc.points[i]);
            }
            srcPtr = &strided;
        }
        const pointcloud::PointCloud& src = *srcPtr;

        IcpOptions tight;
        tight.maxCorrespondenceDistM = kMaxCorrDistM;
        tight.maxIterations = lastFitness_ >= kGoodFitness ? 12 : 20;
        IcpResult best = icp(src, model, lastPose_, tight);

        // Widen / relocalize only when tracking is weak (skip the expensive
        // recovery passes while we are locked on).
        if (best.fitness < kGoodFitness) {
            IcpOptions wide = tight;
            wide.maxCorrespondenceDistM = kWideCorrDistM;
            wide.maxIterations = 25;
            const IcpResult r2 = icp(src, model, lastPose_, wide);
            if (r2.fitness > best.fitness ||
                (r2.fitness >= best.fitness && r2.rmseM < best.rmseM)) {
                best = r2;
            }
        }

        if (best.fitness < kMinFitness) {
            IcpOptions wide;
            wide.maxCorrespondenceDistM = kWideCorrDistM;
            wide.maxIterations = 30;
            const IcpResult r3 = icp(src, model, RigidTransform::identity(), wide);
            if (r3.fitness > best.fitness) {
                best = r3;
            }
        }
        lastFitness_ = best.fitness;
        return best;
    }

    void processFrame(Item item) {
        StageTimer timer;
        static const bool profile = std::getenv("ALL3DSCAN_TRACK_PROFILE") != nullptr;
        timer.on = profile;
        item.cloud = removeGroundPlaneRansac(item.cloud);
        timer.mark("ground");
        pointcloud::PointCloud src = voxelDownsample(item.cloud, sourceVoxelM());
        timer.mark("srcDown");
        if (src.points.size() < kMinSourcePoints) {
            ++lostStreak_;
            emit trackingState(static_cast<int>(TrackingState::Lost));
            emitLiveStats(0.0);
            return;
        }

        const bool modelUsable = !document_->cloudEmpty() &&
                                 document_->isMetric() == item.metric && hasPose_;
        if (!modelUsable) {
            // Seed with a finer cloud so the first view isn't sparse.
            pointcloud::PointCloud seed =
                voxelDownsample(item.cloud, modelVoxelM());
            seedModel(seed, item.metric);
            maybeCaptureKeyframe(item, RigidTransform::identity(), 1.0);
            updateLiveMesh(item, RigidTransform::identity());
            return;
        }

        const IcpResult r = alignFrame(src, timer);
        timer.mark("icp");
        if (r.fitness < acceptFitness() || r.rmseM > acceptRmseM()) {
            ++lostStreak_;
            emit trackingState(static_cast<int>(TrackingState::Lost));
            emitLiveStats(r.fitness);
            // Keep lastPose_ — next frame still has a useful prediction.
            return;
        }

        lostStreak_ = 0;
        lastPose_ = r.transform;

        // Fuse a finer downsample of the full frame (ICP used a coarser one).
        pointcloud::PointCloud dense =
            voxelDownsample(item.cloud, modelVoxelM());
        transformInPlace(dense, r.transform);
        if (r.fitness >= kGoodFitness) {
            tintCoverage(dense);
        }
        timer.mark("denseDown");
        document_->fuseCloud(dense, modelVoxelM(), item.metric,
                             item.metric ? "fused" : "live");
        timer.mark("fuse");
        maybeCaptureKeyframe(item, r.transform, r.fitness);
        timer.mark("keyframe");
        updateLiveMesh(item, r.transform);
        timer.mark("liveMesh");
        emit trackingState(static_cast<int>(TrackingState::Tracking));
        emitLiveStats(r.fitness);
        const auto now = std::chrono::steady_clock::now();
        if (lastUiEmit_.time_since_epoch().count() == 0 ||
            now - lastUiEmit_ >= kUiUpdateMinInterval) {
            lastUiEmit_ = now;
            emit updated();
        }
        (void)kMaxLostBeforeIdle;
    }

    // Gate for fusing a tracked frame. A poor alignment fused into the model
    // corrupts every later alignment, so be strict; overridable for tuning.
    static double acceptFitness() {
        static const double v = envDouble("ALL3DSCAN_TRACK_MIN_FITNESS", kMinFitness, 0.0, 1.0);
        return v;
    }
    static double acceptRmseM() {
        static const double v =
            envDouble("ALL3DSCAN_TRACK_MAX_RMSE_MM", kMaxRmseM * 1000.0, 0.1, 100.0) / 1000.0;
        return v;
    }
    static double envDouble(const char* name, double def, double lo, double hi) {
        if (const char* s = std::getenv(name)) {
            char* end = nullptr;
            const double v = std::strtod(s, &end);
            if (end != s && v >= lo && v <= hi) {
                return v;
            }
        }
        return def;
    }

    static RigidTransform invert(const RigidTransform& t) {
        // Rigid inverse: R^T and -R^T * translation.
        RigidTransform out;
        const auto& m = t.m;
        // Transpose rotation.
        out.m[0] = m[0];  out.m[1] = m[4];  out.m[2] = m[8];
        out.m[4] = m[1];  out.m[5] = m[5];  out.m[6] = m[9];
        out.m[8] = m[2];  out.m[9] = m[6];  out.m[10] = m[10];
        const double tx = m[3], ty = m[7], tz = m[11];
        out.m[3] = -(out.m[0] * tx + out.m[1] * ty + out.m[2] * tz);
        out.m[7] = -(out.m[4] * tx + out.m[5] * ty + out.m[6] * tz);
        out.m[11] = -(out.m[8] * tx + out.m[9] * ty + out.m[10] * tz);
        out.m[12] = out.m[13] = out.m[14] = 0.0;
        out.m[15] = 1.0;
        return out;
    }

    document::ScanDocument* document_ = nullptr;
    std::atomic<float>* nearClip_ = nullptr;
    std::atomic<float>* farClip_ = nullptr;
    std::atomic<float>* modelVoxel_ = nullptr;
    QMutex mutex_;
    QWaitCondition wait_;
    std::deque<Item> queue_;
    std::atomic<uint64_t> processed_{0};
    bool stopping_ = false;
    RigidTransform lastPose_;
    bool hasPose_ = false;
    std::unique_ptr<PlaneIcpTarget> icpTarget_;
    int framesSinceTarget_ = 0;
    int lostStreak_ = 0;
    double lastFitness_ = 1.0;

    QMutex kfMutex_;
    std::vector<DepthKeyframe> keyframes_;
    RigidTransform lastKfPose_;
    bool hasLastKfPose_ = false;
    int framesSinceKf_ = 0;

    QMutex liveMeshMutex_;
    geometry::TriangleMesh liveMesh_;
    std::unique_ptr<geometry::TsdfVolume> liveVolume_;
    std::atomic<uint64_t> liveMeshRevision_{0};
    std::chrono::steady_clock::time_point lastLiveMeshEmit_{};
    std::chrono::steady_clock::time_point lastUiEmit_{};
};

ScanPipeline::ScanPipeline(document::ScanDocument* document, QObject* parent)
    : QObject(parent), document_(document) {}

ScanPipeline::~ScanPipeline() {
    stop();
}

void ScanPipeline::start() {
    if (thread_) {
        return;
    }
    thread_ = new QThread(this);
    worker_ = new Worker(document_, &depthClipNearM_, &depthClipFarM_, &modelVoxelM_);
    worker_->moveToThread(thread_);
    connect(thread_, &QThread::started, worker_, &Worker::processLoop);
    connect(worker_, &Worker::updated, this, &ScanPipeline::documentUpdated);
    connect(worker_, &Worker::trackingState, this, [this](int state) {
        trackingState_ = static_cast<TrackingState>(state);
        emit trackingChanged(state);
    });
    connect(worker_, &Worker::stats, this, &ScanPipeline::statsUpdated);
    connect(worker_, &Worker::liveMeshReady, this, &ScanPipeline::liveMeshUpdated);
    connect(thread_, &QThread::finished, worker_, &QObject::deleteLater);
    thread_->start();
}

void ScanPipeline::stop() {
    if (!thread_) {
        return;
    }
    worker_->requestStop();
    thread_->quit();
    thread_->wait(3000);
    thread_ = nullptr;
    worker_ = nullptr;
    trackingState_ = TrackingState::Idle;
}

void ScanPipeline::reset() {
    stop();
    if (document_) {
        document_->clear();
    }
    start();
}

geometry::TriangleMesh ScanPipeline::liveMesh() const {
    if (!worker_) {
        return {};
    }
    return worker_->snapshotLiveMesh();
}

uint64_t ScanPipeline::liveMeshRevision() const {
    return worker_ ? worker_->liveMeshRevision() : 0;
}

uint64_t ScanPipeline::framesProcessed() const {
    return worker_ ? worker_->processedCount() : 0;
}

bool ScanPipeline::hasLiveMesh() const {
    if (!worker_) {
        return false;
    }
    return worker_->hasLiveMeshSnapshot();
}

void ScanPipeline::submitMetricCloud(pointcloud::PointCloud cloud, uint64_t timestampUs) {
    if (!worker_) {
        start();
    }
    if (worker_) {
        worker_->enqueue(std::move(cloud), timestampUs, /*metric=*/true);
    }
}

void ScanPipeline::submitMetricFrame(pointcloud::PointCloud cloud,
                                     const protocol::RawFrame& depthFrame,
                                     const protocol::CameraIntrinsics& intrinsics,
                                     uint64_t timestampUs) {
    if (!worker_) {
        start();
    }
    if (!worker_) {
        return;
    }
    const bool depthUsable =
        depthFrame.encoding == protocol::DepthEncoding::MetricMillimeters &&
        depthFrame.width > 0 && depthFrame.height > 0 &&
        depthFrame.depthMm.size() ==
            static_cast<size_t>(depthFrame.width) * depthFrame.height &&
        intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
    if (depthUsable) {
        std::vector<uint8_t> rgb;
        if (depthFrame.rgb.size() ==
            static_cast<size_t>(depthFrame.width) * depthFrame.height * 3) {
            rgb = depthFrame.rgb;
        }
        worker_->enqueueWithDepth(std::move(cloud), timestampUs, depthFrame.depthMm,
                                  std::move(rgb), depthFrame.width, depthFrame.height,
                                  intrinsics);
    } else {
        worker_->enqueue(std::move(cloud), timestampUs, /*metric=*/true);
    }
}

void ScanPipeline::submitPreviewCloud(pointcloud::PointCloud cloud, uint64_t timestampUs) {
    if (!worker_) {
        start();
    }
    if (worker_) {
        worker_->enqueue(std::move(cloud), timestampUs, /*metric=*/false);
    }
}

void ScanPipeline::setDepthClip(float nearM, float farM) {
    if (nearM > 0.0f && farM > nearM) {
        depthClipNearM_ = nearM;
        depthClipFarM_ = farM;
    }
}

void ScanPipeline::setPointDistance(float mm) {
    const float clampedMm = std::clamp(mm, 0.3f, 4.0f);
    modelVoxelM_ = clampedMm / 1000.0f;
}

bool ScanPipeline::completeAndMesh(const std::string& workerScript,
                                   const std::string& pythonExecutable,
                                   std::string* errorOut) {
    if (!document_ || !document_->isMetric() || document_->cloudEmpty()) {
        if (errorOut) {
            *errorOut = "no metric point cloud to mesh";
        }
        return false;
    }

    // Prefer volumetric TSDF reconstruction from retained depth keyframes; it
    // produces a cleaner, watertight surface than Poisson over the fused cloud.
    std::vector<DepthKeyframe> keyframes;
    if (worker_) {
        keyframes = worker_->snapshotKeyframes();
    }
    // Native C++ TSDF first: no Python/Open3D needed (Open3D has no build for
    // newer Pythons such as 3.14).
    if (!keyframes.empty()) {
        std::vector<geometry::TsdfFrame> frames;
        frames.reserve(keyframes.size());
        for (const auto& kf : keyframes) {
            geometry::TsdfFrame f;
            f.depthMm = &kf.depthMm;
            f.rgb = &kf.rgb;
            f.width = kf.width;
            f.height = kf.height;
            f.intrinsics = kf.intrinsics;
            f.cameraToWorld = kf.pose;
            frames.push_back(f);
        }
        geometry::TsdfParams params;
        params.voxelM = std::max(0.001f, modelVoxelM_.load());
        params.minDepthM = depthClipNearM_.load();
        params.maxDepthM = depthClipFarM_.load();
        // Copy once: the worker may still be fusing queued frames.
        const pointcloud::PointCloud model = document_->cloudCopy();
        // Robust volume bounds: a few mis-tracked points far away used to
        // inflate the volume, coarsening voxels and slowing Complete a lot.
        pointcloud::PointCloud bounds;
        if (!model.points.empty()) {
            std::array<std::vector<float>, 3> axis;
            for (auto& a : axis) {
                a.reserve(model.points.size());
            }
            for (const auto& p : model.points) {
                axis[0].push_back(p.x);
                axis[1].push_back(p.y);
                axis[2].push_back(p.z);
            }
            std::array<float, 3> lo{}, hi{};
            for (int k = 0; k < 3; ++k) {
                auto& a = axis[static_cast<size_t>(k)];
                const size_t i0 = a.size() / 200;              // 0.5th percentile
                const size_t i1 = a.size() - 1 - a.size() / 200; // 99.5th percentile
                std::nth_element(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(i0), a.end());
                lo[static_cast<size_t>(k)] = a[i0];
                std::nth_element(a.begin(), a.begin() + static_cast<std::ptrdiff_t>(i1), a.end());
                hi[static_cast<size_t>(k)] = a[i1];
            }
            bounds.points.push_back({lo[0], lo[1], lo[2]});
            bounds.points.push_back({hi[0], hi[1], hi[2]});
        }
        geometry::TriangleMesh mesh = geometry::reconstructTsdf(frames, bounds, 0.02f, params);
        // Base removal + floating-fragment cleanup against the tracked model.
        geometry::cleanMesh(mesh, model, std::max(0.004f, 3.0f * params.voxelM));
        geometry::smoothMeshTaubin(mesh, kFinalMeshSmoothIterations);
        if (!mesh.empty()) {
            document_->setMesh(std::move(mesh), true, "tsdf");
            return true;
        }
    }
    if (keyframes.size() >= kMinTsdfKeyframes) {
        std::string tsdfError;
        if (completeWithTsdf(workerScript, pythonExecutable, keyframes, &tsdfError)) {
            return true;
        }
        // Fall back to Poisson but keep the TSDF reason for diagnostics.
        if (errorOut) {
            *errorOut = "TSDF failed (" + tsdfError + "); using Poisson";
        }
    }
    return completeWithPoisson(workerScript, pythonExecutable, errorOut);
}

bool ScanPipeline::completeWithTsdf(const std::string& workerScript,
                                    const std::string& pythonExecutable,
                                    const std::vector<DepthKeyframe>& keyframes,
                                    std::string* errorOut) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path dir = fs::temp_directory_path(ec) / "all3dscan_tsdf";
    fs::create_directories(dir, ec);
    if (ec) {
        if (errorOut) {
            *errorOut = "could not create TSDF temp dir";
        }
        return false;
    }

    nlohmann::json manifest;
    const std::string meshPath = (dir / "tsdf.mesh.ply").string();
    manifest["out"] = meshPath;
    manifest["voxel_length"] = 0.003;
    manifest["sdf_trunc"] = 0.012;
    manifest["depth_scale"] = 1000.0; // decoder emits millimetres
    manifest["depth_min"] = static_cast<double>(depthClipNearM_.load());
    manifest["depth_max"] = static_cast<double>(depthClipFarM_.load());
    manifest["frames"] = nlohmann::json::array();

    for (size_t i = 0; i < keyframes.size(); ++i) {
        const DepthKeyframe& kf = keyframes[i];
        char name[64];
        std::snprintf(name, sizeof(name), "kf%04zu.bin", i);
        const fs::path binPath = dir / name;
        std::ofstream out(binPath, std::ios::binary);
        if (!out) {
            if (errorOut) {
                *errorOut = "could not write keyframe depth";
            }
            return false;
        }
        out.write(reinterpret_cast<const char*>(kf.depthMm.data()),
                  static_cast<std::streamsize>(kf.depthMm.size() * sizeof(uint16_t)));
        out.close();

        nlohmann::json frame;
        frame["depth"] = binPath.string();
        frame["width"] = kf.width;
        frame["height"] = kf.height;
        frame["fx"] = kf.intrinsics.fx;
        frame["fy"] = kf.intrinsics.fy;
        frame["cx"] = kf.intrinsics.cx;
        frame["cy"] = kf.intrinsics.cy;
        frame["pose"] = kf.pose;

        // Optional registered RGB8 texture (grayscale IR albedo today).
        if (kf.rgb.size() == static_cast<size_t>(kf.width) * kf.height * 3) {
            char colorName[64];
            std::snprintf(colorName, sizeof(colorName), "kf%04zu.rgb", i);
            const fs::path colorPath = dir / colorName;
            std::ofstream cout(colorPath, std::ios::binary);
            if (cout) {
                cout.write(reinterpret_cast<const char*>(kf.rgb.data()),
                           static_cast<std::streamsize>(kf.rgb.size()));
                cout.close();
                frame["color"] = colorPath.string();
            }
        }
        manifest["frames"].push_back(std::move(frame));
    }

    const std::string manifestPath = (dir / "manifest.json").string();
    {
        std::ofstream mf(manifestPath);
        if (!mf) {
            if (errorOut) {
                *errorOut = "could not write TSDF manifest";
            }
            return false;
        }
        mf << manifest.dump();
    }

    bridge::PythonWorker worker(workerScript, pythonExecutable);
    if (!worker.start()) {
        if (errorOut) {
            *errorOut = worker.lastError();
        }
        return false;
    }
    auto reply = worker.reconstructTsdf(manifestPath, 240000);
    worker.stop();
    if (!reply) {
        if (errorOut) {
            *errorOut = worker.lastError();
        }
        return false;
    }

    bool ok = false;
    geometry::TriangleMesh mesh = parseAsciiPlyMesh(meshPath, &ok);
    if (!ok) {
        if (errorOut) {
            *errorOut = "failed to parse TSDF mesh PLY";
        }
        return false;
    }
    document_->setMesh(std::move(mesh), true, "tsdf");
    emit documentUpdated();
    return true;
}

bool ScanPipeline::completeWithPoisson(const std::string& workerScript,
                                       const std::string& pythonExecutable,
                                       std::string* errorOut) {
    const std::string ply = "/tmp/all3dscan_fuse.ply";
    const std::string meshPath = "/tmp/all3dscan_fuse.mesh.ply";
    auto plyResult = io::writePlyCloud(ply, document_->cloudCopy(), true, {});
    if (!plyResult.ok) {
        if (errorOut) {
            *errorOut = plyResult.error;
        }
        return false;
    }

    bridge::PythonWorker worker(workerScript, pythonExecutable);
    if (!worker.start()) {
        if (errorOut) {
            *errorOut = worker.lastError();
        }
        return false;
    }
    auto reply = worker.meshPly(ply, meshPath, 8, 180000);
    worker.stop();
    if (!reply) {
        if (errorOut) {
            *errorOut = worker.lastError();
        }
        return false;
    }

    bool ok = false;
    geometry::TriangleMesh mesh = parseAsciiPlyMesh(meshPath, &ok);
    if (!ok) {
        if (errorOut) {
            *errorOut = "failed to parse meshed PLY (is Open3D installed?)";
        }
        return false;
    }
    document_->setMesh(std::move(mesh), true, "poisson");
    emit documentUpdated();
    return true;
}

} // namespace revoscan::processing

#include "ScanPipeline.moc"
