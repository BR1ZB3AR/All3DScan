#pragma once

#include <QElapsedTimer>
#include <QMainWindow>

#include <cstdint>
#include <memory>

#include "ScannerController.h"
#include "document/ScanDocument.h"
#include "net/RevopointWifiControl.h"
#include "processing/ScanPipeline.h"

class QLabel;
class QPushButton;
class QComboBox;
class QStackedWidget;

namespace revoscan::app {

class PointCloudView;
class DistanceScaleWidget;
class CameraPanel;
class ScannerControlPanel;
class EditPanel;
class ArOverlayView;

// Two-page shell like Revo Scan: a Scan page (device, live preview, scan
// controls) and an Edit page (selection tools, filters, export) that
// operates on the completed/loaded scan document.
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    void keyPressEvent(QKeyEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void onConnectToggle();
    void onScanToggle();
    void onSingleShot();
    void onComplete();
    void onCancel();
    void onExport();
    void onSendToSlicer();
    void onImportPly();
    void onUndo();
    void onRedo();
    void onDeleteSelection();
    void onFrameReady(const pointcloud::PointCloud& cloud);
    void onMetricFrame(const pointcloud::PointCloud& cloud, quint64 timestampUs);
    void onPreviewFrame(const pointcloud::PointCloud& cloud, quint64 timestampUs);
    void onFusionToggled(bool on);
    void onIsolateToggled(bool on);
    void onScanDistanceChanged(int index);
    void onAccuracyChanged(int index);
    void onDocumentUpdated();
    void onRawSensorFrame(uint32_t width,
                          uint32_t height,
                          uint32_t nonZeroSamples,
                          uint64_t timestampUs,
                          const QVector<uint16_t>& samples);
    void onRgbPreview(const QByteArray& jpegBytes);
    void onError(const QString& message);

    void showScanPage();
    void showEditPage();
    void onConnectWifi();
    void onAbout();

private:
    // Post-scan Model stages, mirroring Revo Scan's Raw / Point Clouds /
    // Meshed / Textured workspace tabs.
    enum class ModelStage {
        Raw,
        PointClouds,
        Meshed,
        Textured,
    };
    void applyModelStage(ModelStage stage);
    void updateStageAvailability();

    QWidget* buildScanPage();
    QWidget* buildEditPage();
    QWidget* buildRibbon();
    QWidget* buildLeftColumn();
    QWidget* buildViewport();
    QWidget* buildStatusBar();
    void buildMenus();
    void updateControlStates();
    void updateFrameTelemetry();
    void refreshScanView();
    void refreshEditView();

    std::unique_ptr<ScannerController> controller_;
    document::ScanDocument document_;
    std::unique_ptr<processing::ScanPipeline> pipeline_;
    net::RevopointWifiControl* wifiControl_ = nullptr;

    QStackedWidget* pages_ = nullptr;

    // Scan page
    PointCloudView* view_ = nullptr;
    DistanceScaleWidget* distanceScale_ = nullptr;
    CameraPanel* depthPanel_ = nullptr;
    CameraPanel* rgbPanel_ = nullptr;
    ScannerControlPanel* scannerControls_ = nullptr;
    QPushButton* scanButton_ = nullptr;
    QPushButton* singleShotButton_ = nullptr;
    QPushButton* completeButton_ = nullptr;
    QPushButton* cancelButton_ = nullptr;
    QPushButton* colorButton_ = nullptr;
    QPushButton* noColorButton_ = nullptr;
    QPushButton* openEditBtn_ = nullptr;
    QPushButton* fusionButton_ = nullptr;
    QPushButton* isolateButton_ = nullptr;
    QPushButton* liveMeshButton_ = nullptr;
    QPushButton* arOverlayButton_ = nullptr;
    ArOverlayView* arOverlayView_ = nullptr;
    QComboBox* distanceCombo_ = nullptr;
    QComboBox* accuracyCombo_ = nullptr;

    // Edit page
    PointCloudView* editView_ = nullptr;
    EditPanel* editPanel_ = nullptr;
    QPushButton* undoButton_ = nullptr;
    QPushButton* redoButton_ = nullptr;
    QPushButton* deleteSelBtn_ = nullptr;
    QLabel* editInfoLabel_ = nullptr;
    QWidget* editSelectBar_ = nullptr;
    QPushButton* stageRawBtn_ = nullptr;
    QPushButton* stagePointsBtn_ = nullptr;
    QPushButton* stageMeshBtn_ = nullptr;
    QPushButton* stageTexBtn_ = nullptr;
    ModelStage modelStage_ = ModelStage::PointClouds;

    QLabel* connLabel_ = nullptr;
    QLabel* framesLabel_ = nullptr;
    QLabel* fpsLabel_ = nullptr;
    QLabel* trackLabel_ = nullptr;
    QLabel* depthLockLabel_ = nullptr;
    // Keyboard shortcut hint overlay.
    QLabel* shortcutHint_ = nullptr;

    bool connected_ = false;
    bool streaming_ = false;
    bool scanning_ = false;
    bool useColor_ = true;
    bool liveFusion_ = true;
    bool liveMeshPreview_ = false;
    bool objectIsolation_ = true;
    bool shortcutOverlayVisible_ = false;

    float activeBandEdgesMm_[4] = {290.0f, 350.0f, 400.0f, 460.0f};
    QElapsedTimer fpsTimer_;
    int frameCount_ = 0;
    int renderFrames_ = 0;
};

} // namespace revoscan::app
