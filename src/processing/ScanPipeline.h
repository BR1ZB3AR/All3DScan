#pragma once

#include <QObject>
#include <QMutex>
#include <QThread>
#include <QWaitCondition>

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "document/ScanDocument.h"
#include "pointcloud/PointCloud.h"
#include "protocol/ScannerProtocol.h"

namespace revoscan::processing {

enum class TrackingState {
    Idle,
    Tracking,
    Lost,
};

// A tracked metric depth frame retained for volumetric (TSDF) reconstruction.
// pose is the row-major 4x4 camera→world transform recovered by live ICP.
struct DepthKeyframe {
    std::vector<uint16_t> depthMm;
    std::vector<uint8_t> rgb; // width*height*3, empty if no texture
    uint32_t width = 0;
    uint32_t height = 0;
    protocol::CameraIntrinsics intrinsics;
    std::array<double, 16> pose{{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}};
};

// Asynchronous metric fusion / registration orchestration.
// Acquisition stays in ScannerController; this worker owns document updates.
class ScanPipeline : public QObject {
    Q_OBJECT
public:
    explicit ScanPipeline(document::ScanDocument* document, QObject* parent = nullptr);
    ~ScanPipeline() override;

    void start();
    void stop();
    void reset();

    // Enqueue a metric camera-space cloud (already back-projected).
    // Drops oldest frames when the queue is full (back-pressure).
    void submitMetricCloud(pointcloud::PointCloud cloud, uint64_t timestampUs);

    // Enqueue a metric cloud together with its source depth map + intrinsics so
    // the pipeline can retain keyframes for volumetric (TSDF) reconstruction.
    void submitMetricFrame(pointcloud::PointCloud cloud,
                           const protocol::RawFrame& depthFrame,
                           const protocol::CameraIntrinsics& intrinsics,
                           uint64_t timestampUs);

    // Enqueue a best-effort (non-metric) preview cloud for live ICP fusion.
    // The fused model stays flagged non-metric so export remains blocked.
    void submitPreviewCloud(pointcloud::PointCloud cloud, uint64_t timestampUs);

    // Working-distance band (metres) used to crop TSDF depth integration.
    void setDepthClip(float nearM, float farM);

    // Fused-model point spacing ("point distance" in Revo Scan terms), in
    // millimetres. Smaller values keep more surface detail at the cost of
    // point count / fusion speed; larger values scan faster. Clamped to
    // [0.3, 4.0] mm. ICP source/model downsample voxels scale with it so the
    // 1.8mm/1.0mm ratio tuned for POP4 detail capture is preserved.
    void setPointDistance(float mm);

    TrackingState trackingState() const { return trackingState_; }
    document::ScanDocument* document() const { return document_; }

    // Growing shaded mesh built from organized depth during live tracking.
    // Empty when fusion is off or no depth frames have been integrated yet.
    geometry::TriangleMesh liveMesh() const;
    bool hasLiveMesh() const;
    // Frames the worker has finished (for offline replay pacing).
    uint64_t framesProcessed() const;

    // Finalize: TSDF reconstruction from retained keyframes when available,
    // otherwise an Open3D Poisson mesh of the fused cloud.
    bool completeAndMesh(const std::string& workerScript,
                         const std::string& pythonExecutable,
                         std::string* errorOut);

signals:
    void documentUpdated();
    void trackingChanged(int state);
    // Live registration quality: fitness (0..1), model size, retained keyframes.
    void statsUpdated(double fitness, int modelPoints, int keyframes);
    // Live shaded mesh preview updated (throttled ~5–8 Hz).
    void liveMeshUpdated();
    void pipelineError(const QString& message);

private:
    class Worker;
    bool completeWithTsdf(const std::string& workerScript,
                          const std::string& pythonExecutable,
                          const std::vector<DepthKeyframe>& keyframes,
                          std::string* errorOut);
    bool completeWithPoisson(const std::string& workerScript,
                             const std::string& pythonExecutable,
                             std::string* errorOut);

    document::ScanDocument* document_ = nullptr;
    Worker* worker_ = nullptr;
    QThread* thread_ = nullptr;
    std::atomic<TrackingState> trackingState_{TrackingState::Idle};
    std::atomic<float> depthClipNearM_{0.15f};
    std::atomic<float> depthClipFarM_{0.60f};
    std::atomic<float> modelVoxelM_{0.0010f};
};

} // namespace revoscan::processing
