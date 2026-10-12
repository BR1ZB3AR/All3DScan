#include "DistanceScaleWidget.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QToolTip>

#include <algorithm>
#include <cmath>

#include "Theme.h"

namespace revoscan::app {

DistanceScaleWidget::DistanceScaleWidget(QWidget* parent) : QWidget(parent) {
    setMinimumWidth(120);
    setMouseTracking(true);
    hist_.fill(0.0f);
    animHist_ = hist_;
    medianAnimTimer_.start();
}

void DistanceScaleWidget::setRangeMm(float minMm, float maxMm) {
    if (maxMm > minMm) {
        minMm_ = minMm;
        maxMm_ = maxMm;
        update();
    }
}

void DistanceScaleWidget::setBandEdgesMm(float tooNearEnd,
                                         float excellentEnd,
                                         float goodEnd,
                                         float farEnd) {
    bandEdgesMm_[0] = tooNearEnd;
    bandEdgesMm_[1] = excellentEnd;
    bandEdgesMm_[2] = goodEnd;
    bandEdgesMm_[3] = farEnd;
    update();
}

void DistanceScaleWidget::setDepthSamplesMm(const std::vector<float>& depthsMm) {
    hist_.fill(0.0f);
    medianMm_ = -1.0f;
    const float span = maxMm_ - minMm_;
    if (span <= 0.0f) {
        return;
    }

    std::vector<float> valid;
    valid.reserve(depthsMm.size());
    for (float d : depthsMm) {
        if (d <= 0.0f) {
            continue;
        }
        valid.push_back(d);
        float t = (d - minMm_) / span;
        t = std::clamp(t, 0.0f, 0.9999f);
        const int bin = static_cast<int>(t * kBins);
        hist_[bin] += 1.0f;
    }
    histMax_ = 1.0f;
    for (float v : hist_) {
        histMax_ = std::max(histMax_, v);
    }
    if (!valid.empty()) {
        const size_t mid = valid.size() / 2;
        std::nth_element(valid.begin(), valid.begin() + static_cast<std::ptrdiff_t>(mid),
                         valid.end());
        medianMm_ = valid[mid];
        // Initialize animated median if not yet set.
        if (animMedianMm_ < 0.0f) {
            animMedianMm_ = medianMm_;
        }
    }
    update();
}

QPointF DistanceScaleWidget::depthMmToWidget(float depthMm) const {
    const int top = 28;
    const int bottom = height() - 28;
    const int h = bottom - top;
    const float t = (depthMm - minMm_) / (maxMm_ - minMm_);
    return QPointF(width() * 0.5f, top + (1.0f - std::clamp(t, 0.0f, 1.0f)) * h);
}

float DistanceScaleWidget::depthAtY(int widgetY) const {
    const int top = 28;
    const int bottom = height() - 28;
    const int h = bottom - top;
    if (h <= 0) return minMm_;
    const float t = 1.0f - static_cast<float>(widgetY - top) / static_cast<float>(h);
    return minMm_ + std::clamp(t, 0.0f, 1.0f) * (maxMm_ - minMm_);
}

void DistanceScaleWidget::mouseMoveEvent(QMouseEvent* event) {
    tooltipMm_ = depthAtY(event->pos().y());
    update();
    const int mm = static_cast<int>(std::lround(tooltipMm_));
    QToolTip::showText(event->globalPos(), tr("%1 mm").arg(mm), this);
}

bool DistanceScaleWidget::event(QEvent* event) {
    if (event->type() == QEvent::Leave) {
        tooltipMm_ = -1.0f;
        update();
    }
    return QWidget::event(event);
}

void DistanceScaleWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), QColor(theme::kWindowBg));

    const int top = 28;
    const int bottom = height() - 28;
    const int h = bottom - top;
    if (h <= 0) {
        return;
    }

    const float span = maxMm_ - minMm_;

    auto mmToT = [&](float mm) {
        return std::clamp((mm - minMm_) / span, 0.0f, 1.0f);
    };

    // Smooth animation of histogram and median.
    const qint64 dt = medianAnimTimer_.elapsed();
    const float animSpeed = std::clamp(static_cast<float>(dt) * 0.003f, 0.0f, 0.3f);
    if (dt > 0) {
        for (int i = 0; i < kBins; ++i) {
            animHist_[i] += (hist_[i] - animHist_[i]) * animSpeed;
        }
        if (medianMm_ > 0.0f && animMedianMm_ > 0.0f) {
            animMedianMm_ += (medianMm_ - animMedianMm_) * animSpeed;
        } else if (medianMm_ > 0.0f) {
            animMedianMm_ = medianMm_;
        }
    }
    medianAnimTimer_.restart();

    struct Band {
        float t0;
        float t1;
        const char* label;
        QColor color;
    };
    const Band bands[] = {
        {0.0f, mmToT(bandEdgesMm_[0]), "Too Near", QColor(theme::kBad)},
        {mmToT(bandEdgesMm_[0]), mmToT(bandEdgesMm_[1]), "Excellent", QColor(theme::kAccent)},
        {mmToT(bandEdgesMm_[1]), mmToT(bandEdgesMm_[2]), "Good", QColor(theme::kAccentGood)},
        {mmToT(bandEdgesMm_[2]), mmToT(bandEdgesMm_[3]), "Far", QColor(theme::kWarn)},
        {mmToT(bandEdgesMm_[3]), 1.0f, "Too Far", QColor(theme::kBad)},
    };

    QFont f = p.font();
    f.setPointSize(8);
    p.setFont(f);

    // === Gradient band strip (left side) ===
    const int stripLeft = 4;
    const int stripW = 6;
    const int barLeft = stripLeft + stripW + 6;
    const int barRight = width() - 8;
    const int scaleX = width() - 36;

    // Draw gradient bands.
    for (const auto& b : bands) {
        if (b.t1 <= b.t0) continue;
        const int y0 = top + static_cast<int>((1.0f - b.t1) * h);
        const int y1 = top + static_cast<int>((1.0f - b.t0) * h);
        const QRect bandRect(stripLeft, y0, stripW, std::max(1, y1 - y0));

        // Gradient fill for the band.
        QLinearGradient grad(0, y0, 0, y1);
        QColor c = b.color;
        c.setAlpha(180);
        grad.setColorAt(0.0, c.lighter(120));
        grad.setColorAt(0.5, c);
        grad.setColorAt(1.0, c.darker(120));
        p.fillRect(bandRect, grad);

        // Label text.
        p.setPen(QColor(theme::kTextDim));
        const int yc = (y0 + y1) / 2;
        p.drawText(QRect(barLeft, yc - 8, barRight - barLeft, 16),
                   Qt::AlignLeft | Qt::AlignVCenter, b.label);
    }

    // === Tick marks with mm labels (right side) ===
    p.setPen(QColor(theme::kTextDim));
    for (int i = 0; i <= 6; ++i) {
        const float t = static_cast<float>(i) / 6.0f;
        const int y = top + static_cast<int>((1.0f - t) * h);
        const int mm = static_cast<int>(std::lround(minMm_ + t * span));
        p.drawText(QRect(scaleX, y - 8, 36, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QString::number(mm));
        p.setPen(QColor(theme::kBorder));
        p.drawLine(scaleX - 4, y, scaleX, y);
        p.setPen(QColor(theme::kTextDim));
    }

    // === Animated histogram bars ===
    const float binH = static_cast<float>(h) / kBins;
    float localHistMax = 1.0f;
    for (int i = 0; i < kBins; ++i) {
        localHistMax = std::max(localHistMax, animHist_[i]);
    }
    localHistMax = std::max(localHistMax, 1.0f);

    for (int i = 0; i < kBins; ++i) {
        const float v = animHist_[i] / localHistMax;
        if (v <= 0.005f) continue;
        const int barLen = static_cast<int>(v * (barRight - barLeft) * 0.75f);
        const int y = top + static_cast<int>((1.0f - (i + 0.5f) / kBins) * h);
        const int bh = std::max(1, static_cast<int>(binH) - 1);
        const float t = (i + 0.5f) / kBins;
        QColor c(theme::kAccentGood);
        for (const auto& b : bands) {
            if (t >= b.t0 && t < b.t1) {
                c = b.color;
                break;
            }
        }
        c.setAlpha(180);
        p.fillRect(QRect(barRight - barLen - 2, y - bh / 2, barLen, bh), c);
    }

    // === Live animated median marker ===
    if (animMedianMm_ > 0.0f) {
        const float t = mmToT(animMedianMm_);
        const int y = top + static_cast<int>((1.0f - t) * h);

        // Glow line for the median.
        QPen glowPen(QColor(255, 255, 255, 60), 5);
        p.setPen(glowPen);
        p.drawLine(barLeft - 2, y, barRight + 2, y);

        // Main median line.
        QPen medianPen(QColor(255, 255, 255, 220), 2);
        p.setPen(medianPen);
        p.drawLine(barLeft - 2, y, barRight + 2, y);

        // Median value label.
        p.setPen(QColor(theme::kText));
        QFont valFont = p.font();
        valFont.setPointSize(9);
        valFont.setBold(true);
        p.setFont(valFont);
        p.drawText(QRect(barLeft - 2, y - 20, barRight - barLeft + 4, 18),
                   Qt::AlignLeft | Qt::AlignVCenter,
                   QStringLiteral("%1 mm").arg(static_cast<int>(std::lround(animMedianMm_))));
    }

    // === Tooltip indicator (hover) ===
    if (tooltipMm_ > 0.0f) {
        const float t = mmToT(tooltipMm_);
        const int y = top + static_cast<int>((1.0f - t) * h);
        p.setPen(QPen(QColor(0x12, 0xb6, 0xb0, 80), 1, Qt::DashLine));
        p.drawLine(barLeft - 2, y, barRight + 2, y);
    }

    // Title.
    p.setPen(QColor(theme::kTextDim));
    QFont titleFont = p.font();
    titleFont.setPointSize(8);
    p.setFont(titleFont);
    p.drawText(QRect(0, 4, width() - 6, 18), Qt::AlignRight | Qt::AlignVCenter, "mm");
}

} // namespace revoscan::app
