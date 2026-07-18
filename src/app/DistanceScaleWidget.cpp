#include "DistanceScaleWidget.h"

#include <QPainter>
#include <QPaintEvent>

#include <algorithm>
#include <cmath>

#include "Theme.h"

namespace revoscan::app {

DistanceScaleWidget::DistanceScaleWidget(QWidget* parent) : QWidget(parent) {
    setMinimumWidth(120);
    hist_.fill(0.0f);
}

void DistanceScaleWidget::setRangeMm(float minMm, float maxMm) {
    if (maxMm > minMm) {
        minMm_ = minMm;
        maxMm_ = maxMm;
        update();
    }
}

void DistanceScaleWidget::setDepthSamplesMm(const std::vector<float>& depthsMm) {
    hist_.fill(0.0f);
    const float span = maxMm_ - minMm_;
    if (span <= 0.0f) {
        return;
    }
    for (float d : depthsMm) {
        if (d <= 0.0f) {
            continue;
        }
        float t = (d - minMm_) / span;
        t = std::clamp(t, 0.0f, 0.9999f);
        const int bin = static_cast<int>(t * kBins);
        hist_[bin] += 1.0f;
    }
    histMax_ = 1.0f;
    for (float v : hist_) {
        histMax_ = std::max(histMax_, v);
    }
    update();
}

void DistanceScaleWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(theme::kWindowBg));

    const int top = 24;
    const int bottom = height() - 24;
    const int h = bottom - top;
    if (h <= 0) {
        return;
    }

    const int scaleX = width() - 40;   // where tick labels sit
    const int barRight = scaleX - 8;
    const int barLeft = 8;

    // Band definitions as fractions of the range (near -> far, top -> bottom).
    struct Band {
        float t0;
        float t1;
        const char* label;
        QColor color;
    };
    const Band bands[] = {
        {0.00f, 0.20f, "Too Near", QColor(theme::kBad)},
        {0.20f, 0.45f, "Excellent", QColor(theme::kAccent)},
        {0.45f, 0.65f, "Good", QColor(theme::kAccentGood)},
        {0.65f, 0.85f, "Far", QColor(theme::kWarn)},
        {0.85f, 1.00f, "Too Far", QColor(theme::kBad)},
    };

    QFont f = p.font();
    f.setPointSize(8);
    p.setFont(f);

    // Colored band strip on the far left edge.
    const int stripW = 4;
    for (const auto& b : bands) {
        const int y0 = top + static_cast<int>(b.t0 * h);
        const int y1 = top + static_cast<int>(b.t1 * h);
        p.fillRect(QRect(0, y0, stripW, y1 - y0), b.color);
        // Band label near its center.
        p.setPen(QColor(theme::kTextDim));
        const int yc = (y0 + y1) / 2;
        p.drawText(QRect(barLeft, yc - 8, barRight - barLeft, 16),
                   Qt::AlignLeft | Qt::AlignVCenter, b.label);
    }

    // mm tick labels every 1/6 of the range.
    p.setPen(QColor(theme::kTextDim));
    for (int i = 0; i <= 6; ++i) {
        const float t = static_cast<float>(i) / 6.0f;
        const int y = top + static_cast<int>(t * h);
        const int mm = static_cast<int>(std::lround(minMm_ + t * (maxMm_ - minMm_)));
        p.drawText(QRect(scaleX, y - 8, 40, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QString::number(mm));
        p.setPen(QColor(theme::kBorder));
        p.drawLine(scaleX - 4, y, scaleX, y);
        p.setPen(QColor(theme::kTextDim));
    }

    // Histogram bars (green, right-aligned) per Revo Scan's live sample plot.
    const float binH = static_cast<float>(h) / kBins;
    for (int i = 0; i < kBins; ++i) {
        const float v = hist_[i] / histMax_;
        if (v <= 0.0f) {
            continue;
        }
        const int barLen = static_cast<int>(v * (barRight - barLeft) * 0.8f);
        const int y = top + static_cast<int>(i * binH);
        const int bh = std::max(1, static_cast<int>(binH) - 1);
        // Color the bar by which band it falls in.
        const float t = (i + 0.5f) / kBins;
        QColor c(theme::kAccentGood);
        for (const auto& b : bands) {
            if (t >= b.t0 && t < b.t1) {
                c = b.color;
                break;
            }
        }
        c.setAlpha(200);
        p.fillRect(QRect(barRight - barLen, y, barLen, bh), c);
    }

    // "Unit: mm" header.
    p.setPen(QColor(theme::kTextDim));
    p.drawText(QRect(0, 2, width() - 6, 18), Qt::AlignRight | Qt::AlignVCenter, "Unit: mm");
}

} // namespace revoscan::app
