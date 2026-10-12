#pragma once

#include <QObject>
#include <QTimer>
#include <QVector>

#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "pointcloud/PointCloud.h"
#include "protocol/ScannerProtocol.h"

namespace revoscan::app {

class ScannerController : public QObject {
    Q_OBJECT
public:
    explicit ScannerController(std::unique_ptr<protocol::IScannerProtocol> proto,
                               QObject* parent = nullptr);
    ~ScannerController() override;

    bool connectDevice();
    // Runs protocol connect() on a worker thread (arming, uvcvideo rebinds and
    // a possible device reboot can take tens of seconds) and emits
    // connectFinished() on the GUI thread. No-op while one is in flight.
    void connectDeviceAsync();
    bool isConnecting() const { return connecting_; }
    void disconnectDevice();
    bool startScan();
    void stopScan();
    bool captureSingleFrame();
    void resetAccumulation();

    // Live fusion mode: emit per-frame preview clouds (for ICP registration)
    // instead of the static max-hold accumulation used when fusion is off.
    void setFusionEnabled(bool on) { fusionEnabled_ = on; }
    bool fusionEnabled() const { return fusionEnabled_; }

    // Real-time object isolation: keep only the nearest connected object,
    // dropping background/table/noise before preview and fusion.
    void setObjectIsolation(bool on) { objectIsolation_ = on; }
    bool objectIsolation() const { return objectIsolation_; }
    void setIsolationParams(const pointcloud::ForegroundIsolation& params) {
        isolationParams_ = params;
    }
    const pointcloud::ForegroundIsolation& isolationParams() const {
        return isolationParams_;
    }

    const pointcloud::PointCloud& lastCloud() const { return lastCloud_; }
    bool lastCloudMetric() const { return lastCloudMetric_; }
    // Full metric depth map + intrinsics of the latest metric frame (for TSDF).
    const protocol::RawFrame& lastMetricDepth() const { return lastMetricDepth_; }
    const protocol::CameraIntrinsics& lastIntrinsics() const { return lastIntrinsics_; }
    protocol::IScannerProtocol* protocol() { return proto_.get(); }

signals:
    void connected();
    void disconnected();
    void connectFinished(bool ok);
    void frameReady(const pointcloud::PointCloud& cloud);
    void metricFrameReady(const pointcloud::PointCloud& cloud, quint64 timestampUs);
    void previewFrameReady(const pointcloud::PointCloud& cloud, quint64 timestampUs);
    void rawSensorFrameReady(uint32_t width,
                             uint32_t height,
                             uint32_t nonZeroSamples,
                             uint64_t timestampUs,
                             const QVector<uint16_t>& samples);
    void rgbPreviewReady(const QByteArray& jpegBytes);
    void errorOccurred(const QString& message);

private slots:
    void poll();

private:
    bool publishFrame(const protocol::RawFrame& frame);
    void tryEmitRgbPreview();

    std::unique_ptr<protocol::IScannerProtocol> proto_;
    std::thread connectThread_;
    bool connecting_ = false;
    QTimer pollTimer_;
    pointcloud::PointCloud lastCloud_;
    protocol::RawFrame lastMetricDepth_;
    protocol::CameraIntrinsics lastIntrinsics_;
    bool lastCloudMetric_ = false;

    bool fusionEnabled_ = true;
    bool objectIsolation_ = true;
    pointcloud::ForegroundIsolation isolationParams_;
    std::vector<uint16_t> accumulated_;
    uint32_t accWidth_ = 0;
    uint32_t accHeight_ = 0;
};

} // namespace revoscan::app
