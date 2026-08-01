#include "ScannerController.h"

#include <algorithm>

namespace revoscan::app {
namespace {

QString protocolError(const protocol::IScannerProtocol& proto,
                      const QString& fallback) {
    const std::string detail = proto.lastError();
    return detail.empty() ? fallback : QString::fromStdString(detail);
}

bool publishFrame(ScannerController* controller,
                  protocol::IScannerProtocol& proto,
                  const protocol::RawFrame& frame,
                  pointcloud::PointCloud& lastCloud) {
    const size_t expected =
        static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
    if (frame.width == 0 || frame.height == 0 || frame.depthMm.size() != expected) {
        return false;
    }
    if (frame.encoding == protocol::DepthEncoding::VendorRawY16) {
        const auto nonZero = static_cast<uint32_t>(
            std::count_if(frame.depthMm.begin(), frame.depthMm.end(),
                          [](uint16_t value) { return value != 0; }));
        emit controller->rawSensorFrameReady(
            frame.width, frame.height, nonZero, frame.timestampUs);
        return true;
    }

    const auto intrinsics = proto.intrinsics();
    if (intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0) {
        return false;
    }
    lastCloud = pointcloud::fromDepthFrame(frame, intrinsics);
    emit controller->frameReady(lastCloud);
    return true;
}

} // namespace

ScannerController::ScannerController(std::unique_ptr<protocol::IScannerProtocol> proto, QObject* parent)
    : QObject(parent), proto_(std::move(proto)) {
    pollTimer_.setInterval(33); // ~30 fps preview polling
    connect(&pollTimer_, &QTimer::timeout, this, &ScannerController::poll);
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

    if (!publishFrame(this, *proto_, *frame, lastCloud_)) {
        emit errorOccurred(tr("Scanner returned an invalid or undecodable frame"));
        return false;
    }
    return true;
}

void ScannerController::poll() {
    auto frame = proto_->readFrame(0);
    if (!frame) {
        return;
    }
    if (!publishFrame(this, *proto_, *frame, lastCloud_)) {
        emit errorOccurred(tr("Scanner returned an invalid or undecodable frame"));
    }
}

} // namespace revoscan::app
