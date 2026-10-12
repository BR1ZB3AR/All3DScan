#include "ArOverlayView.h"

#include <QPainter>
#include <QPen>
#include <QColor>
#include <algorithm>
#include <cmath>

namespace revoscan::app {

ArOverlayView::ArOverlayView(QWidget* parent)
    : QWidget(parent) {
    setAttribute(Qt::WA_OpaquePaintEvent, false);
    setMinimumSize(320, 240);
}

void ArOverlayView::setRgbFrame(const QImage& frame) {
    currentRgbFrame_ = frame;
    update();
}

void ArOverlayView::setPointCloud(const pointcloud::PointCloud& cloud) {
    currentCloud_ = cloud;
    update();
}

void ArOverlayView::setArEnabled(bool enabled) {
    arEnabled_ = enabled;
    update();
}

void ArOverlayView::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    if (!currentRgbFrame_.isNull()) {
        // Scale and draw video frame centered inside widget
        QImage scaled = currentRgbFrame_.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        int x = (width() - scaled.width()) / 2;
        int y = (height() - scaled.height()) / 2;
        painter.drawImage(x, y, scaled);
    } else {
        painter.fillRect(rect(), QColor(18, 22, 28));
        painter.setPen(QColor(100, 110, 125));
        painter.drawText(rect(), Qt::AlignCenter, "AR Viewport — Awaiting RGB Stream");
    }

    if (arEnabled_) {
        drawArOverlay(painter);
    }
}

void ArOverlayView::drawArOverlay(QPainter& painter) {
    if (currentCloud_.points.empty()) return;

    // Convert PointCloud points to RustPoint3D
    std::vector<bridge::RustPoint3D> rust3D;
    rust3D.reserve(std::min(currentCloud_.points.size(), size_t(5000)));

    for (size_t i = 0; i < currentCloud_.points.size(); i += std::max(size_t(1), currentCloud_.points.size() / 3000)) {
        const auto& p = currentCloud_.points[i];
        rust3D.push_back({p.x, p.y, p.z, p.r, p.g, p.b});
    }

    bridge::RustArIntrinsics intrin{};
    intrin.width = static_cast<uint32_t>(width());
    intrin.height = static_cast<uint32_t>(height());
    intrin.fx = static_cast<float>(width()) * 0.85f;
    intrin.fy = intrin.fx;
    intrin.cx = static_cast<float>(width()) * 0.5f;
    intrin.cy = static_cast<float>(height()) * 0.5f;

    std::vector<bridge::RustArPoint2D> projected2D;
    if (bridge::RustBridge::projectArPoints(rust3D, intrin, projected2D)) {
        for (const auto& pt : projected2D) {
            QColor col(pt.r == 0 && pt.g == 0 && pt.b == 0 ? QColor(0, 230, 200) : QColor(pt.r, pt.g, pt.b));
            painter.setPen(QPen(col, 2));
            painter.drawPoint(QPointF(pt.u, pt.v));
        }
    }

    // AR HUD Overlay Badge
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 0, 0, 160));
    painter.drawRoundedRect(10, 10, 180, 28, 6, 6);

    painter.setPen(QColor(0, 230, 200));
    painter.drawText(20, 28, QString("AR Overlay Active (%1 pts)").arg(projected2D.size()));
}

} // namespace revoscan::app
