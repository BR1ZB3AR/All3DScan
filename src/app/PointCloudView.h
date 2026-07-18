#pragma once

#include <QWidget>

#include "pointcloud/PointCloud.h"

namespace revoscan::app {

// Minimal orthographic-projection point cloud viewer. Renders points via
// QPainter — no OpenGL/Qt3D dependency for v1. Sufficient for a live
// preview and sanity-checking scans; can be swapped for a Qt3D/VTK-based
// viewer later without touching ScannerController or the pipeline.
class PointCloudView : public QWidget {
    Q_OBJECT
public:
    explicit PointCloudView(QWidget* parent = nullptr);

public slots:
    void setPointCloud(const pointcloud::PointCloud& cloud);
    void setColorEnabled(bool on);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    pointcloud::PointCloud cloud_;
    float yawDeg_ = 0.0f;
    float pitchDeg_ = 0.0f;
    float zoom_ = 200.0f;
    QPoint lastMousePos_;
    bool dragging_ = false;
    bool colorEnabled_ = true;
};

} // namespace revoscan::app
