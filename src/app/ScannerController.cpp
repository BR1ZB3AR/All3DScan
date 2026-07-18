#include "ScannerController.h"

namespace revoscan::app {

ScannerController::ScannerController(std::unique_ptr<protocol::IScannerProtocol> proto, QObject* parent)
    : QObject(parent), proto_(std::move(proto)) {
    pollTimer_.setInterval(33); // ~30 fps preview polling
    connect(&pollTimer_, &QTimer::timeout, this, &ScannerController::poll);
}

bool ScannerController::connectDevice() {
    if (!proto_->connect()) {
        emit errorOccurred(tr("Failed to connect to scanner"));
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
        emit errorOccurred(tr("Failed to start stream"));
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

    lastCloud_ = pointcloud::fromDepthFrame(*frame, proto_->intrinsics());
    emit frameReady(lastCloud_);
    return true;
}

void ScannerController::poll() {
    auto frame = proto_->readFrame(0);
    if (!frame) {
        return;
    }
    lastCloud_ = pointcloud::fromDepthFrame(*frame, proto_->intrinsics());
    emit frameReady(lastCloud_);
}

} // namespace revoscan::app
