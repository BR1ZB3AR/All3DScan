#include "ScannerController.h"

#include <QByteArray>

#include <algorithm>
#include <utility>

namespace revoscan::app {
namespace {

QString protocolError(const protocol::IScannerProtocol& proto,
                      const QString& fallback) {
    const std::string detail = proto.lastError();
    return detail.empty() ? fallback : QString::fromStdString(detail);
}

} // namespace

ScannerController::ScannerController(std::unique_ptr<protocol::IScannerProtocol> proto,
                                     QObject* parent)
    : QObject(parent), proto_(std::move(proto)) {
    pollTimer_.setInterval(33);
    connect(&pollTimer_, &QTimer::timeout, this, &ScannerController::poll);
}

bool ScannerController::publishFrame(const protocol::RawFrame& frame) {
    const size_t expected =
        static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
    if (frame.width == 0 || frame.height == 0 || frame.depthMm.size() != expected) {
        return false;
    }
    if (frame.encoding == protocol::DepthEncoding::VendorRawY16) {
        const auto nonZero = static_cast<uint32_t>(
            std::count_if(frame.depthMm.begin(), frame.depthMm.end(),
                          [](uint16_t value) { return value != 0; }));

        QVector<uint16_t> samples;
        samples.reserve(static_cast<int>(expected));
        for (uint16_t sample : frame.depthMm) {
            samples.push_back(sample);
        }

        const auto scale = pointcloud::defaultPop4PreviewDepthScale();

        if (fusionEnabled_) {
            // Per-frame preview cloud, registered/fused downstream by ICP.
            auto preview = pointcloud::previewFromVendorRawY16(frame, scale);
            lastCloud_ = objectIsolation_
                             ? pointcloud::isolateForegroundObject(preview, isolationParams_)
                             : std::move(preview);
            lastCloudMetric_ = false;
            emit rawSensorFrameReady(
                frame.width, frame.height, nonZero, frame.timestampUs, samples);
            emit previewFrameReady(lastCloud_, frame.timestampUs);
            emit frameReady(lastCloud_);
            return true;
        }

        // Fusion off: static max-hold accumulation (no registration).
        if (accWidth_ != frame.width || accHeight_ != frame.height) {
            accWidth_ = frame.width;
            accHeight_ = frame.height;
            accumulated_.assign(expected, 0);
        }
        for (size_t i = 0; i < expected; ++i) {
            accumulated_[i] = std::max(accumulated_[i], frame.depthMm[i]);
        }

        protocol::RawFrame merged;
        merged.width = frame.width;
        merged.height = frame.height;
        merged.timestampUs = frame.timestampUs;
        merged.encoding = protocol::DepthEncoding::VendorRawY16;
        merged.depthMm = accumulated_;
        auto mergedPreview = pointcloud::previewFromVendorRawY16(merged, scale);
        lastCloud_ = objectIsolation_
                         ? pointcloud::isolateForegroundObject(mergedPreview, isolationParams_)
                         : std::move(mergedPreview);
        lastCloudMetric_ = false;

        emit rawSensorFrameReady(
            frame.width, frame.height, nonZero, frame.timestampUs, samples);
        emit frameReady(lastCloud_);
        return true;
    }

    const auto intrinsics = proto_->intrinsics();
    if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
        return false;
    }
    auto cloud = pointcloud::fromDepthFrame(frame, intrinsics);
    if (cloud.points.empty()) {
        return false;
    }
    if (objectIsolation_) {
        cloud = pointcloud::isolateForegroundObject(cloud, isolationParams_);
    }
    // Retain the full metric depth map + intrinsics for volumetric (TSDF)
    // reconstruction, which needs per-pixel depth rather than the isolated cloud.
    lastMetricDepth_ = frame;
    lastIntrinsics_ = intrinsics;
    lastCloud_ = std::move(cloud);
    lastCloudMetric_ = true;

    // Depth-camera panel: false-color preview from the metric depth map
    // (metric path never went through rawSensorFrameReady before).
    {
        QVector<uint16_t> samples;
        samples.reserve(static_cast<int>(frame.depthMm.size()));
        uint32_t nonZero = 0;
        for (uint16_t d : frame.depthMm) {
            samples.push_back(d);
            if (d != 0) {
                ++nonZero;
            }
        }
        emit rawSensorFrameReady(
            frame.width, frame.height, nonZero, frame.timestampUs, samples);
    }

    emit metricFrameReady(lastCloud_, frame.timestampUs);
    emit frameReady(lastCloud_);
    return true;
}

void ScannerController::resetAccumulation() {
    accumulated_.assign(accumulated_.size(), 0);
    lastCloud_ = {};
    lastCloudMetric_ = false;
    emit frameReady(lastCloud_);
}

bool ScannerController::connectDevice() {
    if (!proto_->connect()) {
        emit errorOccurred(protocolError(*proto_, tr("Failed to connect to scanner")));
        return false;
    }
    emit connected();
    return true;
}

void ScannerController::disconnectDevice() {
    stopScan();
    proto_->emittersOff();
    proto_->disconnect();
    emit disconnected();
}

bool ScannerController::startScan() {
    if (!proto_->startStream()) {
        emit errorOccurred(protocolError(*proto_, tr("Failed to start stream")));
        return false;
    }
    pollTimer_.start();
    return true;
}

void ScannerController::stopScan() {
    pollTimer_.stop();
    proto_->stopStream();
}

bool ScannerController::captureSingleFrame() {
    if (proto_->state() == protocol::ConnectionState::Disconnected) {
        emit errorOccurred(tr("Not connected"));
        return false;
    }
    const bool wasStreaming = proto_->state() == protocol::ConnectionState::Streaming;
    if (!wasStreaming && !proto_->startStream()) {
        emit errorOccurred(tr("Failed to start stream for capture"));
        return false;
    }

    auto frame = proto_->readFrame(1000);
    if (!wasStreaming) {
        proto_->stopStream();
    }

    if (!frame) {
        emit errorOccurred(tr("Timed out waiting for frame"));
        return false;
    }

    if (!publishFrame(*frame)) {
        emit errorOccurred(tr("Scanner returned an invalid or undecodable frame"));
        return false;
    }
    return true;
}

void ScannerController::poll() {
    auto frame = proto_->readFrame(0);
    if (!frame) {
        // Still try a non-blocking RGB preview grab so the panel stays live
        // even if depth momentarily stalls.
        tryEmitRgbPreview();
        return;
    }
    if (!publishFrame(*frame)) {
        emit errorOccurred(tr("Scanner returned an invalid or undecodable frame"));
    }
    tryEmitRgbPreview();
}

void ScannerController::tryEmitRgbPreview() {
    // Throttle RGB MJPEG decode — 1280×1024 JPEG every depth frame is costly.
    static int skip = 0;
    if ((++skip % 3) != 0) {
        return;
    }
    auto jpeg = proto_->readRgbJpegPreview(0);
    if (!jpeg || jpeg->empty()) {
        return;
    }
    emit rgbPreviewReady(QByteArray(reinterpret_cast<const char*>(jpeg->data()),
                                    static_cast<int>(jpeg->size())));
}

} // namespace revoscan::app
