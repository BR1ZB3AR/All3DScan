#pragma once

#include <QWidget>

class QImage;
class QLabel;
class QToolButton;

namespace revoscan::app {

// One camera block in the left column (Depth Camera or RGB Camera in Revo
// Scan 6): a title, a live preview area, and an exposure row with Auto +
// stepper. RGB panels additionally show an AWB row.
class CameraPanel : public QWidget {
    Q_OBJECT
public:
    enum class Kind { Depth, Rgb };

    explicit CameraPanel(const QString& title, Kind kind, QWidget* parent = nullptr);

    bool showDepthMap() const { return showDepthMap_; }
    float zoomFactor() const { return zoomFactor_; }

public slots:
    void setPreview(const QImage& image);
    void clearPreview();

signals:
    void exposureChanged(int value);
    void autoExposureToggled(bool on);
    void depthModeToggled(bool showDepthMap);

private slots:
    void onModeToggle();
    void onZoomSelect();

private:
    Kind kind_;
    QLabel* preview_ = nullptr;
    QToolButton* exposureAuto_ = nullptr;

    // Overlay controls
    QToolButton* modeToggleBtn_ = nullptr;
    QWidget* zoomGroupWidget_ = nullptr;

    bool showDepthMap_ = false;
    float zoomFactor_ = 1.0f;
    QImage lastImage_;
};

} // namespace revoscan::app
