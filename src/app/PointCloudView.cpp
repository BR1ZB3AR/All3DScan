#include "PointCloudView.h"

#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace revoscan::app {

PointCloudView::PointCloudView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 240);
}

void PointCloudView::setPointCloud(const pointcloud::PointCloud& cloud) {
    cloud_ = cloud;
    update();
}

void PointCloudView::setColorEnabled(bool on) {
    colorEnabled_ = on;
    update();
}

void PointCloudView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    // Subtle vertical gradient like the Revo Scan viewport.
    QLinearGradient bg(0, 0, 0, height());
    bg.setColorAt(0.0, QColor(0x15, 0x18, 0x1c));
    bg.setColorAt(1.0, QColor(0x0c, 0x0e, 0x10));
    painter.fillRect(rect(), bg);
    painter.setRenderHint(QPainter::Antialiasing, true);

    if (cloud_.points.empty()) {
        painter.setPen(QColor(0x5b, 0x61, 0x69));
        painter.drawText(rect(), Qt::AlignCenter,
                         tr("Connect the scanner and press Scan"));
        return;
    }

    const float yaw = yawDeg_ * static_cast<float>(M_PI) / 180.0f;
    const float pitch = pitchDeg_ * static_cast<float>(M_PI) / 180.0f;
    const float cx = width() / 2.0f;
    const float cy = height() / 2.0f;

    for (const auto& p : cloud_.points) {
        // Rotate around Y (yaw) then X (pitch), then orthographic project.
        const float x1 = p.x * std::cos(yaw) + p.z * std::sin(yaw);
        const float z1 = -p.x * std::sin(yaw) + p.z * std::cos(yaw);
        const float y1 = p.y * std::cos(pitch) - z1 * std::sin(pitch);

        const float sx = cx + x1 * zoom_;
        const float sy = cy + y1 * zoom_;

        if (sx < 0 || sx >= width() || sy < 0 || sy >= height()) {
            continue;
        }

        if (colorEnabled_) {
            painter.setPen(QColor(p.r, p.g, p.b));
        } else {
            // Depth-shaded blue-teal ramp, matching "No Color" mode.
            const float t = std::clamp((z1 + 0.3f) / 0.6f, 0.0f, 1.0f);
            const int r = static_cast<int>(40 + 40 * t);
            const int g = static_cast<int>(120 + 90 * t);
            const int b = static_cast<int>(200 + 40 * (1.0f - t));
            painter.setPen(QColor(r, g, std::min(255, b)));
        }
        painter.drawPoint(QPointF(sx, sy));
    }
}

void PointCloudView::mousePressEvent(QMouseEvent* event) {
    dragging_ = true;
    lastMousePos_ = event->pos();
}

void PointCloudView::mouseMoveEvent(QMouseEvent* event) {
    if (!dragging_) {
        return;
    }
    const QPoint delta = event->pos() - lastMousePos_;
    lastMousePos_ = event->pos();
    yawDeg_ += delta.x() * 0.5f;
    pitchDeg_ += delta.y() * 0.5f;
    update();
}

void PointCloudView::wheelEvent(QWheelEvent* event) {
    zoom_ *= (event->angleDelta().y() > 0) ? 1.1f : 0.9f;
    update();
}

} // namespace revoscan::app
