#pragma once

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

    // Feed the current depth distribution (millimeters). Values outside the
    // gauge range are clamped into the end bands.
    void setDepthSamplesMm(const std::vector<float>& depthsMm);
    void setRangeMm(float minMm, float maxMm);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    float minMm_ = 120.0f;
    float maxMm_ = 300.0f;
    static constexpr int kBins = 48;
    std::array<float, kBins> hist_{};
    float histMax_ = 1.0f;
};

} // namespace revoscan::app
