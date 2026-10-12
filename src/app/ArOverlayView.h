#pragma once

#include <QWidget>
#include <QImage>
#include <QPaintEvent>
#include <vector>

#include "pointcloud/PointCloud.h"
#include "bridge/RustBridge.h"

namespace revoscan::app {

class ArOverlayView : public QWidget {
    Q_OBJECT
public:
    explicit ArOverlayView(QWidget* parent = nullptr);

    void setRgbFrame(const QImage& frame);
    void setPointCloud(const pointcloud::PointCloud& cloud);
    void setArEnabled(bool enabled);
    bool isArEnabled() const { return arEnabled_; }

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage currentRgbFrame_;
    pointcloud::PointCloud currentCloud_;
    bool arEnabled_ = true;

    void drawArOverlay(QPainter& painter);
};

} // namespace revoscan::app
