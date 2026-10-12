#pragma once

#include <QElapsedTimer>
#include <QWidget>

#include <array>
#include <vector>

namespace revoscan::app {

// Vertical working-distance gauge like Revo Scan 6: a mm scale from Too Near
// -> Excellent -> Good -> Far -> Too Far, with a live histogram of how many
// depth samples fall in each band. Purely a preview widget.
class DistanceScaleWidget : public QWidget {
    Q_OBJECT
public:
    explicit DistanceScaleWidget(QWidget* parent = nullptr);

    // Feed the current depth distribution (millimetres). Values outside the
    // gauge range are clamped into the end bands.
    void setDepthSamplesMm(const std::vector<float>& depthsMm);
    void setRangeMm(float minMm, float maxMm);

    // Absolute band edges in mm (param_pop4 histogram1..4 style).
    void setBandEdgesMm(float tooNearEnd,
                        float excellentEnd,
                        float goodEnd,
                        float farEnd);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    bool event(QEvent* event) override;

private:
    QPointF depthMmToWidget(float depthMm) const;
    float depthAtY(int widgetY) const;

    float minMm_ = 210.0f;
    float maxMm_ = 500.0f;
    float bandEdgesMm_[4] = {290.0f, 350.0f, 400.0f, 460.0f};
    float medianMm_ = -1.0f;
    float animMedianMm_ = -1.0f; // smoothly interpolated median for display
    QElapsedTimer medianAnimTimer_;
    float tooltipMm_ = -1.0f;
    static constexpr int kBins = 48;
    std::array<float, kBins> hist_{};
    std::array<float, kBins> animHist_{}; // smoothed histogram
    float histMax_ = 1.0f;
};

} // namespace revoscan::app
