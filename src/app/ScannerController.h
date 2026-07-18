#pragma once

#include <QObject>
#include <QTimer>

#include <memory>

#include "pointcloud/PointCloud.h"
#include "protocol/ScannerProtocol.h"

namespace revoscan::app {

// Owns the protocol implementation and polls it on a QTimer, converting
// each frame to a point cloud and emitting it for the GUI to render. Uses
// whatever IScannerProtocol it's constructed with — MockProtocol today,
// the real Phase B implementation later — with no changes needed here.
class ScannerController : public QObject {
    Q_OBJECT
public:
    explicit ScannerController(std::unique_ptr<protocol::IScannerProtocol> proto, QObject* parent = nullptr);

    bool connectDevice();
    void disconnectDevice();
    bool startScan();
    void stopScan();
    bool captureSingleFrame();

    const pointcloud::PointCloud& lastCloud() const { return lastCloud_; }

signals:
    void connected();
    void disconnected();
    void frameReady(const pointcloud::PointCloud& cloud);
    void errorOccurred(const QString& message);

private slots:
    void poll();

private:
    std::unique_ptr<protocol::IScannerProtocol> proto_;
    QTimer pollTimer_;
    pointcloud::PointCloud lastCloud_;
};

} // namespace revoscan::app
