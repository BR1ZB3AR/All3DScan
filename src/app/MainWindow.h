#pragma once

#include <QElapsedTimer>
#include <QMainWindow>

#include <cstdint>
#include <memory>

#include "ScannerController.h"

class QLabel;
class QPushButton;
class QComboBox;

namespace revoscan::app {

class PointCloudView;
class DistanceScaleWidget;
class CameraPanel;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onConnectToggle();
    void onScanToggle();
    void onSingleShot();
    void onComplete();
    void onCancel();
    void onExport();
    void onFrameReady(const pointcloud::PointCloud& cloud);
    void onRawSensorFrame(uint32_t width,
                          uint32_t height,
                          uint32_t nonZeroSamples,
                          uint64_t timestampUs);
    void onError(const QString& message);

private:
    QWidget* buildRibbon();
    QWidget* buildLeftColumn();
    QWidget* buildViewport();
    QWidget* buildStatusBar();
    void buildMenus();
    void updateControlStates();
    void updateFrameTelemetry();

    std::unique_ptr<ScannerController> controller_;

    PointCloudView* view_ = nullptr;
    DistanceScaleWidget* distanceScale_ = nullptr;
    CameraPanel* depthPanel_ = nullptr;
    CameraPanel* rgbPanel_ = nullptr;

    QPushButton* scanButton_ = nullptr;
    QPushButton* singleShotButton_ = nullptr;
    QPushButton* completeButton_ = nullptr;
    QPushButton* cancelButton_ = nullptr;
    QPushButton* undoButton_ = nullptr;
    QPushButton* redoButton_ = nullptr;
    QPushButton* colorButton_ = nullptr;
    QPushButton* noColorButton_ = nullptr;

    QLabel* connLabel_ = nullptr;
    QLabel* framesLabel_ = nullptr;
    QLabel* fpsLabel_ = nullptr;

    bool connected_ = false;
    bool streaming_ = false;
    bool useColor_ = true;

    QElapsedTimer fpsTimer_;
    int frameCount_ = 0;
    int renderFrames_ = 0;
};

} // namespace revoscan::app
