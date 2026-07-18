#include "MainWindow.h"

#include <QComboBox>
#include <QFileDialog>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <vector>

#include "CameraPanel.h"
#include "DistanceScaleWidget.h"
#include "PointCloudView.h"
#include "Theme.h"
#include "io/PointCloudExport.h"
#include "protocol/ScannerProtocol.h"

namespace revoscan::app {
namespace {

QComboBox* ribbonCombo(const QStringList& items, QWidget* parent) {
    auto* box = new QComboBox(parent);
    box->addItems(items);
    box->setMinimumWidth(120);
    return box;
}

QWidget* ribbonGroup(const QString& title, QWidget* content, QWidget* parent) {
    auto* group = new QFrame(parent);
    group->setObjectName("RibbonGroup");
    auto* v = new QVBoxLayout(group);
    v->setContentsMargins(10, 4, 10, 4);
    v->setSpacing(2);
    v->addWidget(content, 0, Qt::AlignCenter);
    auto* t = new QLabel(title, group);
    t->setObjectName("RibbonGroupTitle");
    t->setAlignment(Qt::AlignCenter);
    v->addWidget(t);
    return group;
}

QFrame* ribbonSeparator(QWidget* parent) {
    auto* sep = new QFrame(parent);
    sep->setObjectName("RibbonSep");
    sep->setFrameShape(QFrame::VLine);
    sep->setFixedWidth(1);
    return sep;
}

} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle(tr("Untitled — All3DScan"));

    controller_ = std::make_unique<ScannerController>(protocol::makeMockProtocol(), this);
    connect(controller_.get(), &ScannerController::frameReady, this, &MainWindow::onFrameReady);
    connect(controller_.get(), &ScannerController::errorOccurred, this, &MainWindow::onError);

    buildMenus();

    auto* central = new QWidget(this);
    central->setObjectName("Central");
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    root->addWidget(buildRibbon());

    auto* body = new QHBoxLayout();
    body->setContentsMargins(0, 0, 0, 0);
    body->setSpacing(0);
    body->addWidget(buildLeftColumn());
    body->addWidget(buildViewport(), 1);
    root->addLayout(body, 1);

    root->addWidget(buildStatusBar());

    setCentralWidget(central);
    setStyleSheet(revoScanStyleSheet());
    resize(1100, 720);

    fpsTimer_.start();
    updateControlStates();
}

void MainWindow::buildMenus() {
    QMenu* fileMenu = menuBar()->addMenu(tr("File"));
    fileMenu->addAction(tr("Export…"), this, &MainWindow::onExport);
    fileMenu->addSeparator();
    fileMenu->addAction(tr("Quit"), this, &QWidget::close);

    QMenu* scanMenu = menuBar()->addMenu(tr("Scan"));
    scanMenu->addAction(tr("Start/Pause"), this, &MainWindow::onScanToggle);
    scanMenu->addAction(tr("Single Shot"), this, &MainWindow::onSingleShot);
    scanMenu->addAction(tr("Complete"), this, &MainWindow::onComplete);
    scanMenu->addAction(tr("Cancel"), this, &MainWindow::onCancel);

    menuBar()->addMenu(tr("Edit"));
    menuBar()->addMenu(tr("View"));
    menuBar()->addMenu(tr("Help"));
}

QWidget* MainWindow::buildRibbon() {
    auto* ribbon = new QFrame(this);
    ribbon->setObjectName("Ribbon");
    ribbon->setFixedHeight(64);
    auto* h = new QHBoxLayout(ribbon);
    h->setContentsMargins(6, 4, 6, 4);
    h->setSpacing(0);

    h->addWidget(ribbonGroup(tr("Accuracy"),
                             ribbonCombo({tr("High Accuracy"), tr("Standard")}, ribbon), ribbon));
    h->addWidget(ribbonGroup(tr("Object Type"),
                             ribbonCombo({tr("Normal"), tr("Dark"), tr("Marker")}, ribbon), ribbon));
    h->addWidget(ribbonGroup(tr("Scanning Distance"),
                             ribbonCombo({tr("120-219mm"), tr("220-400mm")}, ribbon), ribbon));
    h->addWidget(ribbonSeparator(ribbon));

    colorButton_ = new QPushButton(tr("Color Scanning"), ribbon);
    colorButton_->setCheckable(true);
    h->addWidget(ribbonGroup(tr("RGB"), colorButton_, ribbon));

    auto* baseRemoval = new QPushButton(tr("Base Removal"), ribbon);
    baseRemoval->setCheckable(true);
    h->addWidget(ribbonGroup(QString(), baseRemoval, ribbon));
    h->addWidget(ribbonSeparator(ribbon));

    h->addWidget(ribbonGroup(QString(), new QPushButton(tr("Accessories"), ribbon), ribbon));
    h->addWidget(ribbonGroup(QString(), new QPushButton(tr("Scanner Info…"), ribbon), ribbon));
    h->addStretch(1);
    return ribbon;
}

QWidget* MainWindow::buildLeftColumn() {
    auto* panel = new QFrame(this);
    panel->setObjectName("SidePanel");
    panel->setFixedWidth(240);
    auto* v = new QVBoxLayout(panel);
    v->setContentsMargins(0, 0, 0, 0);
    v->setSpacing(1);

    depthPanel_ = new CameraPanel(tr("Depth Camera"), CameraPanel::Kind::Depth, panel);
    rgbPanel_ = new CameraPanel(tr("RGB Camera"), CameraPanel::Kind::Rgb, panel);
    v->addWidget(depthPanel_, 1);
    v->addWidget(rgbPanel_, 1);
    return panel;
}

QWidget* MainWindow::buildViewport() {
    auto* container = new QWidget(this);
    auto* h = new QHBoxLayout(container);
    h->setContentsMargins(0, 0, 0, 0);
    h->setSpacing(0);

    distanceScale_ = new DistanceScaleWidget(container);
    distanceScale_->setFixedWidth(130);
    h->addWidget(distanceScale_);

    // The 3D area plus overlays (Color/No Color toggle, scan controls).
    auto* viewArea = new QWidget(container);
    auto* vv = new QVBoxLayout(viewArea);
    vv->setContentsMargins(0, 0, 0, 0);
    vv->setSpacing(0);

    // Top overlay row: Color / No Color segmented toggle, right-aligned.
    auto* topRow = new QHBoxLayout();
    topRow->setContentsMargins(8, 8, 8, 0);
    topRow->addStretch(1);
    auto* toggle = new QFrame(viewArea);
    toggle->setObjectName("SegmentedToggle");
    auto* th = new QHBoxLayout(toggle);
    th->setContentsMargins(2, 2, 2, 2);
    th->setSpacing(0);
    noColorButton_ = new QPushButton(tr("No Color"), toggle);
    auto* colorViewBtn = new QPushButton(tr("Color"), toggle);
    colorViewBtn->setCheckable(true);
    noColorButton_->setCheckable(true);
    colorViewBtn->setChecked(true);
    th->addWidget(colorViewBtn);
    th->addWidget(noColorButton_);
    topRow->addWidget(toggle);
    vv->addLayout(topRow);

    view_ = new PointCloudView(viewArea);
    vv->addWidget(view_, 1);

    connect(colorViewBtn, &QPushButton::clicked, this, [this, colorViewBtn] {
        useColor_ = true;
        colorViewBtn->setChecked(true);
        noColorButton_->setChecked(false);
        onFrameReady(controller_->lastCloud());
    });
    connect(noColorButton_, &QPushButton::clicked, this, [this, colorViewBtn] {
        useColor_ = false;
        colorViewBtn->setChecked(false);
        noColorButton_->setChecked(true);
        onFrameReady(controller_->lastCloud());
    });

    // Bottom overlay row: Scan / Single Shot / Complete / Cancel / Undo / Redo.
    auto* controls = new QFrame(viewArea);
    auto* ch = new QHBoxLayout(controls);
    ch->setContentsMargins(8, 6, 8, 8);
    ch->setSpacing(8);
    ch->addStretch(1);

    scanButton_ = new QPushButton(tr("Scan"), controls);
    scanButton_->setObjectName("ScanPrimary");
    singleShotButton_ = new QPushButton(tr("Single Shot"), controls);
    completeButton_ = new QPushButton(tr("Complete"), controls);
    cancelButton_ = new QPushButton(tr("Cancel"), controls);
    undoButton_ = new QPushButton(tr("Undo"), controls);
    redoButton_ = new QPushButton(tr("Redo"), controls);
    for (QPushButton* b : {scanButton_, singleShotButton_, completeButton_, cancelButton_,
                           undoButton_, redoButton_}) {
        ch->addWidget(b);
    }
    ch->addStretch(1);
    vv->addWidget(controls);

    h->addWidget(viewArea, 1);

    connect(scanButton_, &QPushButton::clicked, this, &MainWindow::onScanToggle);
    connect(singleShotButton_, &QPushButton::clicked, this, &MainWindow::onSingleShot);
    connect(completeButton_, &QPushButton::clicked, this, &MainWindow::onComplete);
    connect(cancelButton_, &QPushButton::clicked, this, &MainWindow::onCancel);
    return container;
}

QWidget* MainWindow::buildStatusBar() {
    auto* bar = new QFrame(this);
    bar->setObjectName("StatusBar");
    bar->setFixedHeight(28);
    auto* h = new QHBoxLayout(bar);
    h->setContentsMargins(10, 0, 10, 0);
    h->setSpacing(16);

    auto* hint = new QLabel(
        tr("Space: Start/Pause Scan    Esc: Cancel    "
           "Depth/RGB Exposure ± via panel"),
        bar);
    hint->setObjectName("StatusHint");
    h->addWidget(hint);
    h->addStretch(1);

    framesLabel_ = new QLabel(tr("Rendering Frames: 0"), bar);
    framesLabel_->setObjectName("Fps");
    h->addWidget(framesLabel_);

    fpsLabel_ = new QLabel(tr("Frame Rate: 0.0 FPS"), bar);
    fpsLabel_->setObjectName("Fps");
    h->addWidget(fpsLabel_);

    connLabel_ = new QLabel(tr("● Disconnected"), bar);
    connLabel_->setObjectName("StatusConn");
    connLabel_->setStyleSheet("color:#d95c5c;");
    h->addWidget(connLabel_);

    auto* connectBtn = new QPushButton(tr("Connect"), bar);
    connect(connectBtn, &QPushButton::clicked, this, [this, connectBtn] {
        onConnectToggle();
        connectBtn->setText(connected_ ? tr("Disconnect") : tr("Connect"));
    });
    h->addWidget(connectBtn);

    auto* exportBtn = new QPushButton(tr("Export…"), bar);
    connect(exportBtn, &QPushButton::clicked, this, &MainWindow::onExport);
    h->addWidget(exportBtn);
    return bar;
}

void MainWindow::onConnectToggle() {
    if (!connected_) {
        connected_ = controller_->connectDevice();
    } else {
        controller_->disconnectDevice();
        connected_ = false;
        streaming_ = false;
    }
    if (connected_) {
        connLabel_->setText(tr("● Scanner Connected"));
        connLabel_->setStyleSheet("color:#5bbf5b;");
    } else {
        connLabel_->setText(tr("● Disconnected"));
        connLabel_->setStyleSheet("color:#d95c5c;");
    }
    updateControlStates();
}

void MainWindow::onScanToggle() {
    if (!connected_) {
        return;
    }
    if (!streaming_) {
        streaming_ = controller_->startScan();
    } else {
        controller_->stopScan();
        streaming_ = false;
    }
    scanButton_->setText(streaming_ ? tr("Pause") : tr("Scan"));
    updateControlStates();
}

void MainWindow::onSingleShot() {
    if (connected_) {
        controller_->captureSingleFrame();
    }
}

void MainWindow::onComplete() {
    if (streaming_) {
        controller_->stopScan();
        streaming_ = false;
        scanButton_->setText(tr("Scan"));
    }
    updateControlStates();
}

void MainWindow::onCancel() {
    if (streaming_) {
        controller_->stopScan();
        streaming_ = false;
        scanButton_->setText(tr("Scan"));
    }
    updateControlStates();
}

void MainWindow::onExport() {
    const auto& cloud = controller_->lastCloud();
    if (cloud.points.empty()) {
        QMessageBox::warning(this, tr("Export"), tr("No point cloud captured yet."));
        return;
    }
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Export Point Cloud"), QString(), tr("PLY (*.ply);;OBJ (*.obj)"));
    if (path.isEmpty()) {
        return;
    }
    const bool ok = path.endsWith(".obj", Qt::CaseInsensitive)
                        ? io::writeObj(path.toStdString(), cloud)
                        : io::writePly(path.toStdString(), cloud);
    if (!ok) {
        QMessageBox::critical(this, tr("Export"), tr("Failed to write file."));
    }
}

void MainWindow::onFrameReady(const pointcloud::PointCloud& cloud) {
    view_->setColorEnabled(useColor_);
    view_->setPointCloud(cloud);

    std::vector<float> depths;
    depths.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        // PointCloud is in meters here; the distance gauge wants mm.
        depths.push_back(p.z * 1000.0f);
    }
    distanceScale_->setDepthSamplesMm(depths);

    ++frameCount_;
    ++renderFrames_;
    framesLabel_->setText(tr("Rendering Frames: %1").arg(renderFrames_));
    const qint64 elapsed = fpsTimer_.elapsed();
    if (elapsed >= 500) {
        const double fps = frameCount_ * 1000.0 / static_cast<double>(elapsed);
        fpsLabel_->setText(tr("Frame Rate: %1 FPS").arg(fps, 0, 'f', 1));
        frameCount_ = 0;
        fpsTimer_.restart();
    }
}

void MainWindow::onError(const QString& message) {
    connLabel_->setText(tr("● %1").arg(message));
    connLabel_->setStyleSheet("color:#e0a83e;");
}

void MainWindow::updateControlStates() {
    scanButton_->setEnabled(connected_);
    singleShotButton_->setEnabled(connected_);
    completeButton_->setEnabled(connected_ && streaming_);
    cancelButton_->setEnabled(connected_ && streaming_);
}

} // namespace revoscan::app
