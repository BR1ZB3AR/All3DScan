#include "MainWindow.h"
#include "ArOverlayView.h"

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDialog>
#include <QFileDialog>
#include <QFrame>
#include <QHBoxLayout>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QTabWidget>
#include <QTextBrowser>
#include <QDateTime>
#include <QDir>
#include <QInputDialog>
#include <QLineEdit>
#include <QMenuBar>
#include <QKeySequence>
#include <QMessageBox>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

#include "CameraPanel.h"
#include "DistanceScaleWidget.h"
#include "EditPanel.h"
#include "PointCloudView.h"
#include "ScannerControlPanel.h"
#include "Theme.h"
#include "io/GeometryExport.h"
#include "io/PlyImport.h"
#include "io/PointCloudExport.h"
#include "protocol/ScannerProtocol.h"
#include "util/ResourcePaths.h"

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

QImage buildLeftDepthColorMapPreview(const uint16_t* depthMm,
                                     uint32_t srcW,
                                     uint32_t srcH,
                                     const float bandEdgesMm[4]) {
    constexpr uint32_t kPlaneH = 512;
    constexpr uint32_t kPlaneW = 640;
    constexpr uint32_t kViewW = 320; // one stereo half

    uint32_t extractOffY = 0;
    uint32_t extractH = srcH;

    if (srcH >= kPlaneH * 2 && srcW == kPlaneW) {
        extractH = kPlaneH;
    }
    const uint32_t outW = std::min(kViewW, srcW);
    const uint32_t outH = std::min(extractH, srcH - extractOffY);
    if (outW == 0 || outH == 0) {
        return {};
    }

    QImage img(static_cast<int>(outW), static_cast<int>(outH), QImage::Format_RGB32);

    QRgb cBad = QColor(revoscan::app::theme::kBad).rgb();
    QRgb cAccent = QColor(revoscan::app::theme::kAccent).rgb();
    QRgb cAccentGood = QColor(revoscan::app::theme::kAccentGood).rgb();
    QRgb cWarn = QColor(revoscan::app::theme::kWarn).rgb();

    for (uint32_t y = 0; y < outH; ++y) {
        for (uint32_t x = 0; x < outW; ++x) {
            const uint16_t val = depthMm[static_cast<size_t>(y + extractOffY) * srcW + x];
            if (val == 0) {
                img.setPixel(static_cast<int>(x), static_cast<int>(y), qRgb(16, 18, 20));
                continue;
            }
            float d = static_cast<float>(val);
            if (d < bandEdgesMm[0]) {
                img.setPixel(static_cast<int>(x), static_cast<int>(y), cBad);
            } else if (d < bandEdgesMm[1]) {
                img.setPixel(static_cast<int>(x), static_cast<int>(y), cAccent);
            } else if (d < bandEdgesMm[2]) {
                img.setPixel(static_cast<int>(x), static_cast<int>(y), cAccentGood);
            } else if (d < bandEdgesMm[3]) {
                img.setPixel(static_cast<int>(x), static_cast<int>(y), cWarn);
            } else {
                img.setPixel(static_cast<int>(x), static_cast<int>(y), cBad);
            }
        }
    }
    return img;
}

// Build a depth/IR preview image suitable for the CameraPanel (typically
// displayed at ~220 px wide). For packed Y16 (e.g. 640×2560 = 5×640×512
// temporal planes), extracts the first-plane left stereo half (320×512).
// For regular 640×512, shows the left 320×512 stereo view.
// Uses local histogram-based contrast enhancement and mild horizontal blur
// to suppress projector fringes while preserving object detail.
QImage buildLeftIrPreview(const uint8_t* rgb,
                          const uint16_t* depthOrRaw,
                          uint32_t srcW,
                          uint32_t srcH,
                          bool fromMetricRgb) {
    // --- Step 1: unpack the region of interest ---
    // For packed 640×2560 (5×640×512 planes), extract the first plane.
    // Each 640×512 plane has left (cols 0..319) and right (cols 320..639)
    // stereo views side by side. Show the left half.
    constexpr uint32_t kPlaneH = 512;
    constexpr uint32_t kPlaneW = 640;
    constexpr uint32_t kViewW = 320; // one stereo half

    uint32_t extractOffY = 0; // row offset into source
    uint32_t extractH = srcH;

    if (srcH >= kPlaneH * 2 && srcW == kPlaneW) {
        // Packed multi-plane: extract the first 640×512 plane.
        extractH = kPlaneH;
    }
    // Use the left stereo half (cols 0..kViewW-1).
    const uint32_t outW = std::min(kViewW, srcW);
    const uint32_t outH = std::min(extractH, srcH - extractOffY);
    if (outW == 0 || outH == 0) {
        return {};
    }

    // --- Step 2: build 8-bit grayscale from source ---
    std::vector<uint8_t> gray(static_cast<size_t>(outW) * outH, 0);

    if (fromMetricRgb && rgb) {
        // IR albedo from metric decoder: copy red channel (grayscale).
        for (uint32_t y = 0; y < outH; ++y) {
            for (uint32_t x = 0; x < outW; ++x) {
                const size_t si = (static_cast<size_t>(y + extractOffY) * srcW + x);
                gray[static_cast<size_t>(y) * outW + x] = rgb[si * 3];
            }
        }
    } else if (depthOrRaw) {
        // Raw Y16: collect nonzero samples for percentile range estimation.
        std::vector<uint16_t> nonzero;
        nonzero.reserve(outW * outH);
        for (uint32_t y = 0; y < outH; ++y) {
            for (uint32_t x = 0; x < outW; ++x) {
                const uint16_t s = depthOrRaw[(static_cast<size_t>(y + extractOffY) * srcW + x)];
                if (s != 0) {
                    nonzero.push_back(s);
                }
            }
        }

        uint16_t lo = 0;
        uint16_t hi = 65535;
        if (!nonzero.empty()) {
            std::sort(nonzero.begin(), nonzero.end());
            const size_t nz = nonzero.size();
            lo = nonzero[nz / 20];           // 5th percentile
            hi = nonzero[(nz * 19) / 20];     // 95th percentile
        }
        // Clamp range so we get a decent spread.
        if (hi <= lo) {
            hi = lo + 1;
        }
        // Narrower range = more contrast on the active signal.
        const float span = static_cast<float>(hi - lo);

        // Local contrast: use a tile-based adaptive stretch.
        // Divide image into 32×32 tiles, compute local 5th/95th percentile
        // per tile, and interpolate for each pixel.
        constexpr int kTile = 32;
        const int tilesX = std::max(2, (static_cast<int>(outW) + kTile - 1) / kTile);
        const int tilesY = std::max(2, (static_cast<int>(outH) + kTile - 1) / kTile);
        std::vector<float> tileLo(static_cast<size_t>(tilesX) * tilesY, static_cast<float>(lo));
        std::vector<float> tileHi(static_cast<size_t>(tilesX) * tilesY, static_cast<float>(hi));

        // Sample each tile.
        for (int ty = 0; ty < tilesY; ++ty) {
            for (int tx = 0; tx < tilesX; ++tx) {
                const int x0 = tx * kTile;
                const int y0 = ty * kTile;
                const int x1 = std::min(x0 + kTile, static_cast<int>(outW));
                const int y1 = std::min(y0 + kTile, static_cast<int>(outH));
                if (x1 <= x0 || y1 <= y0) continue;
                uint16_t tlo = 65535, thi = 0;
                int count = 0;
                for (int yp = y0; yp < y1; ++yp) {
                    for (int xp = x0; xp < x1; ++xp) {
                        const uint16_t s = depthOrRaw[
                            (static_cast<size_t>(yp + static_cast<int>(extractOffY)) * srcW + static_cast<size_t>(xp))];
                        if (s != 0) {
                            tlo = std::min(tlo, s);
                            thi = std::max(thi, s);
                            ++count;
                        }
                    }
                }
                if (count > 0 && thi > tlo) {
                    const size_t idx = static_cast<size_t>(ty) * tilesX + tx;
                    tileLo[idx] = static_cast<float>(tlo);
                    tileHi[idx] = static_cast<float>(thi);
                }
            }
        }

        // Map each pixel using bilinearly interpolated local range.
        for (uint32_t y = 0; y < outH; ++y) {
            for (uint32_t x = 0; x < outW; ++x) {
                const uint16_t s = depthOrRaw[
                    (static_cast<size_t>(y + extractOffY) * srcW + x)];
                if (s == 0) {
                    gray[static_cast<size_t>(y) * outW + x] = 0;
                    continue;
                }
                const float fy = static_cast<float>(y) / static_cast<float>(outH) * (tilesY - 1);
                const float fx = static_cast<float>(x) / static_cast<float>(outW) * (tilesX - 1);
                const int iy = std::min(static_cast<int>(fy), tilesY - 2);
                const int ix = std::min(static_cast<int>(fx), tilesX - 2);
                const float dy = fy - iy;
                const float dx = fx - ix;
                const auto idx00 = static_cast<size_t>(iy) * tilesX + ix;
                const auto idx10 = idx00 + 1;
                const auto idx01 = idx00 + static_cast<size_t>(tilesX);
                const auto idx11 = idx01 + 1;

                const float tLo = tileLo[idx00] * (1 - dx) * (1 - dy) +
                                  tileLo[idx10] * dx * (1 - dy) +
                                  tileLo[idx01] * (1 - dx) * dy +
                                  tileLo[idx11] * dx * dy;
                const float tHi = tileHi[idx00] * (1 - dx) * (1 - dy) +
                                  tileHi[idx10] * dx * (1 - dy) +
                                  tileHi[idx01] * (1 - dx) * dy +
                                  tileHi[idx11] * dx * dy;
                const float span = std::max(1.0f, tHi - tLo);
                float t = (static_cast<float>(s) - tLo) / span;
                t = std::clamp(t, 0.0f, 1.0f);
                // Gamma-like mid-tone boost for better object visibility.
                t = t * t * (3.0f - 2.0f * t); // smoothstep
                gray[static_cast<size_t>(y) * outW + x] =
                    static_cast<uint8_t>(t * 255.0f);
            }
        }
    }

    // --- Step 3: mild horizontal blur to suppress vertical projector fringes ---
    // Only apply to raw Y16 (not metric IR albedo which is already clean).
    std::vector<uint8_t> display;
    const uint8_t* sourcePtr = gray.data();
    std::vector<uint8_t> blurred;

    if (!fromMetricRgb && depthOrRaw) {
        blurred.resize(gray.size());
        // 3-tap box blur (gentler than the old 5-tap, preserves more detail).
        for (uint32_t y = 0; y < outH; ++y) {
            for (uint32_t x = 0; x < outW; ++x) {
                int sum = 0;
                int n = 0;
                for (int dx = -1; dx <= 1; ++dx) {
                    const int xx = static_cast<int>(x) + dx;
                    if (xx < 0 || xx >= static_cast<int>(outW)) continue;
                    sum += gray[static_cast<size_t>(y) * outW + static_cast<size_t>(xx)];
                    ++n;
                }
                blurred[static_cast<size_t>(y) * outW + x] =
                    static_cast<uint8_t>(n > 0 ? sum / n : 0);
            }
        }
        sourcePtr = blurred.data();
        display = std::move(blurred);
    } else {
        display = gray;
        sourcePtr = display.data();
    }

    // --- Step 4: build the final QImage ---
    QImage preview(static_cast<int>(outW), static_cast<int>(outH),
                   QImage::Format_RGB32);

    // Compute a mild global stretch so nothing clips harshly.
    std::vector<uint8_t> sorted(display.begin(), display.end());
    std::sort(sorted.begin(), sorted.end());
    const size_t n = sorted.size();
    const uint8_t gLow = (n > 0) ? sorted[n / 40] : 0;       // 2.5th pctile (mild)
    const uint8_t gHigh = (n > 0) ? sorted[(n * 39) / 40] : 255; // 97.5th pctile
    const float gSpan = std::max(1.0f, static_cast<float>(gHigh - gLow));

    for (uint32_t y = 0; y < outH; ++y) {
        auto* line = reinterpret_cast<QRgb*>(preview.scanLine(static_cast<int>(y)));
        for (uint32_t x = 0; x < outW; ++x) {
            const size_t i = static_cast<size_t>(y) * outW + x;
            float t = (static_cast<float>(sourcePtr[i]) - static_cast<float>(gLow)) / gSpan;
            t = std::clamp(t, 0.0f, 1.0f);
            const int g = static_cast<int>(t * 255.0f);

            line[x] = qRgb(g, g, g);
        }
    }
    return preview;
}

bool pop4DebugPreview() {
    if (const char* v = std::getenv("ALL3DSCAN_POP4_DEBUG")) {
        return v[0] == '1' || v[0] == 'y' || v[0] == 'Y' || v[0] == 't' ||
               v[0] == 'T';
    }
    return false;
}

} // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    const QString backend =
        QProcessEnvironment::systemEnvironment().value("ALL3DSCAN_BACKEND").toLower();
    auto scannerProtocol = backend == QStringLiteral("mock")
                               ? protocol::makeMockProtocol()
                               : protocol::makeRevopointPop4Protocol();
    setWindowTitle(tr("Untitled — All3DScan [%1]")
                       .arg(QString::fromStdString(scannerProtocol->backendName())));
    controller_ =
        std::make_unique<ScannerController>(std::move(scannerProtocol), this);
    pipeline_ = std::make_unique<processing::ScanPipeline>(&document_, this);
    pipeline_->start();

    wifiControl_ = new net::RevopointWifiControl(this);
    connect(wifiControl_, &net::RevopointWifiControl::probed, this,
            [this](bool ok, const QString& info) {
                if (ok) {
                    connLabel_->setText(
                        tr("● WiFi scanner reachable (SN %1) at %2")
                            .arg(info, wifiControl_->host()));
                    connLabel_->setStyleSheet("color:#5bbf5b;");
                } else {
                    onError(tr("WiFi scanner not reachable: %1").arg(info));
                }
            });

    connect(controller_.get(), &ScannerController::frameReady, this, &MainWindow::onFrameReady);
    connect(controller_.get(),
            &ScannerController::metricFrameReady,
            this,
            &MainWindow::onMetricFrame);
    connect(controller_.get(),
            &ScannerController::previewFrameReady,
            this,
            &MainWindow::onPreviewFrame);
    connect(controller_.get(),
            &ScannerController::rawSensorFrameReady,
            this,
            &MainWindow::onRawSensorFrame);
    connect(controller_.get(),
            &ScannerController::rgbPreviewReady,
            this,
            &MainWindow::onRgbPreview);
    controller_->setFusionEnabled(liveFusion_);
    controller_->setObjectIsolation(objectIsolation_);
    // Match ribbon default "250-500mm (full-field)" isolation window.
    {
        pointcloud::ForegroundIsolation params;
        params.nearClipM = 0.25f;
        params.farClipM = 0.50f;
        params.nearBandM = 0.12f;
        controller_->setIsolationParams(params);
        if (pipeline_) {
            pipeline_->setDepthClip(params.nearClipM, params.farClipM);
        }
    }
    connect(controller_.get(), &ScannerController::errorOccurred, this, &MainWindow::onError);
    connect(pipeline_.get(),
            &processing::ScanPipeline::documentUpdated,
            this,
            &MainWindow::onDocumentUpdated);
    connect(pipeline_.get(), &processing::ScanPipeline::trackingChanged, this, [this](int state) {
        if (!trackLabel_) {
            return;
        }
        using processing::TrackingState;
        switch (static_cast<TrackingState>(state)) {
        case TrackingState::Tracking:
            trackLabel_->setText(tr("Tracking: OK"));
            trackLabel_->setStyleSheet("color:#5bbf5b;");
            break;
        case TrackingState::Lost:
            trackLabel_->setText(tr("Tracking: LOST"));
            trackLabel_->setStyleSheet("color:#e0a83e;");
            break;
        default:
            trackLabel_->setText(tr("Tracking: Idle"));
            trackLabel_->setStyleSheet("color:#5b6169;");
            break;
        }
    });
    connect(pipeline_.get(), &processing::ScanPipeline::statsUpdated, this,
            [this](double fitness, int modelPoints, int keyframes) {
                if (!trackLabel_) {
                    return;
                }
                const int pct = static_cast<int>(std::lround(fitness * 100.0));
                // `fitness` is the share of the frame lying on the scanned
                // surface (strict overlap test).
                if (pipeline_->trackingState() == processing::TrackingState::Lost) {
                    trackLabel_->setText(
                        tr("Tracking LOST — move back over a scanned area to auto re-align "
                           "(%1% overlap)  •  %2 pts  •  %3 KF")
                            .arg(pct)
                            .arg(modelPoints)
                            .arg(keyframes));
                    trackLabel_->setStyleSheet("color:#e0a83e;");
                } else {
                    const bool justRealigned = relocalizedAt_.isValid() &&
                                               relocalizedAt_.elapsed() < 2000;
                    trackLabel_->setText(
                        tr("Tracking: %1%%2  •  %3 pts  •  %4 KF")
                            .arg(pct)
                            .arg(justRealigned ? tr(" (re-aligned)") : QString())
                            .arg(modelPoints)
                            .arg(keyframes));
                    trackLabel_->setStyleSheet(
                        pct >= 50 ? "color:#5bbf5b;" : "color:#e0a83e;");
                }
            });
    connect(pipeline_.get(), &processing::ScanPipeline::liveMeshUpdated, this,
            [this] { refreshScanView(); });
    connect(pipeline_.get(), &processing::ScanPipeline::relocalized, this,
            [this] { relocalizedAt_.start(); });

    buildMenus();

    auto* central = new QWidget(this);
    central->setObjectName("Central");
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    pages_ = new QStackedWidget(central);
    pages_->addWidget(buildScanPage());
    pages_->addWidget(buildEditPage());
    root->addWidget(pages_, 1);
    root->addWidget(buildStatusBar());

    setCentralWidget(central);
    setStyleSheet(revoScanStyleSheet());
    resize(1280, 780);

    scannerControls_->bindProtocol(controller_->protocol());
    connect(scannerControls_, &ScannerControlPanel::controlError, this, &MainWindow::onError);
    // A mode change resets and reopens the protocol, so the controller runs it
    // with frame polling suspended, like connect.
    connect(scannerControls_, &ScannerControlPanel::scanModeRequested, this, [this](int mode) {
        if (!connected_) {
            scannerControls_->finishModeChange(false, tr("Connect the scanner first"));
            return;
        }
        controller_->setScanModeAsync(static_cast<protocol::ScanMode>(mode));
    });
    connect(controller_.get(), &ScannerController::scanModeFinished, this, [this](bool ok) {
        scannerControls_->finishModeChange(ok);
        updateControlStates();
    });
    connect(editPanel_, &EditPanel::documentEdited, this, &MainWindow::onDocumentUpdated);
    connect(editPanel_, &EditPanel::editError, this, &MainWindow::onError);

    fpsTimer_.start();
    updateControlStates();

    // App logo (set on QApplication in main()).
    setWindowIcon(QApplication::windowIcon());
}

MainWindow::~MainWindow() {
    // A running Complete uses the pipeline and the document; let it finish.
    if (completeThread_.joinable()) {
        completeThread_.join();
    }
    if (pipeline_) {
        pipeline_->stop();
    }
    if (controller_ && controller_->protocol()) {
        controller_->protocol()->emittersOff();
    }
}

void MainWindow::buildMenus() {
    QMenu* fileMenu = menuBar()->addMenu(tr("File"));
    QAction* newScanAction = fileMenu->addAction(tr("New Scan"), this, &MainWindow::onNewScan);
    newScanAction->setShortcut(QKeySequence::New);
    fileMenu->addSeparator();
    fileMenu->addAction(tr("Open PLY…"), this, &MainWindow::onImportPly);
    fileMenu->addAction(tr("Export…"), this, &MainWindow::onExport);
    fileMenu->addAction(tr("Send to OrcaSlicer"), this, &MainWindow::onSendToSlicer);
    fileMenu->addSeparator();
    fileMenu->addAction(tr("Quit"), this, &QWidget::close);

    QMenu* scanMenu = menuBar()->addMenu(tr("Scan"));
    scanMenu->addAction(tr("Scan Page"), this, &MainWindow::showScanPage);
    scanMenu->addAction(tr("Connect over WiFi…"), this, &MainWindow::onConnectWifi);
    scanMenu->addSeparator();
    scanMenu->addAction(tr("Start/Pause"), this, &MainWindow::onScanToggle);
    scanMenu->addAction(tr("Single Shot"), this, &MainWindow::onSingleShot);
    scanMenu->addAction(tr("Complete"), this, &MainWindow::onComplete);
    scanMenu->addAction(tr("Cancel"), this, &MainWindow::onCancel);

    QMenu* editMenu = menuBar()->addMenu(tr("Edit"));
    editMenu->addAction(tr("Edit Page"), this, &MainWindow::showEditPage);
    editMenu->addSeparator();
    editMenu->addAction(tr("Undo"), this, &MainWindow::onUndo);
    editMenu->addAction(tr("Redo"), this, &MainWindow::onRedo);

    menuBar()->addMenu(tr("View"));
    QMenu* helpMenu = menuBar()->addMenu(tr("Help"));
    helpMenu->addAction(tr("Keyboard Shortcuts"), [this]() {
        if (shortcutHint_) shortcutHint_->setVisible(!shortcutHint_->isVisible());
    });
    helpMenu->addAction(tr("About All3DScan"), this, &MainWindow::onAbout);

    QMenu* aboutMenu = menuBar()->addMenu(tr("About"));
    aboutMenu->addAction(tr("About & Changelog"), this, &MainWindow::onAbout);
}

void MainWindow::onAbout() {
    QDialog dialog(this);
    dialog.setWindowTitle(tr("About All3DScan"));
    dialog.setFixedSize(560, 440);

    auto* root = new QVBoxLayout(&dialog);
    root->setContentsMargins(16, 16, 16, 16);
    root->setSpacing(12);

    auto* tabs = new QTabWidget(&dialog);

    // --- Tab 1: About ---
    auto* aboutWidget = new QWidget(tabs);
    auto* aboutLayout = new QVBoxLayout(aboutWidget);
    aboutLayout->setContentsMargins(14, 14, 14, 14);
    aboutLayout->setSpacing(10);

    auto* header = new QHBoxLayout();
    header->setSpacing(14);
    auto* logoLabel = new QLabel(aboutWidget);
    logoLabel->setPixmap(QApplication::windowIcon().pixmap(96, 96));
    logoLabel->setFixedSize(96, 96);
    header->addWidget(logoLabel, 0, Qt::AlignTop);
    auto* headerText = new QVBoxLayout();
    auto* titleLabel = new QLabel(
        tr("<h2 style='color:#12b6b0; margin:0;'>All3DScan</h2>"
           "<p style='color:#8b9198; font-size:12px; margin:2px 0 0 0;'>Open-Source Native Linux 3D Scanning Application</p>"),
        aboutWidget);
    headerText->addWidget(titleLabel);
    auto* verLabel = new QLabel(
        tr("<b>Version:</b> %1").arg(QApplication::applicationVersion()), aboutWidget);
    headerText->addWidget(verLabel);
    headerText->addStretch(1);
    header->addLayout(headerText, 1);
    aboutLayout->addLayout(header);

    auto* descText = new QTextBrowser(aboutWidget);
    descText->setOpenExternalLinks(true);
    descText->setStyleSheet(QStringLiteral(
        "QTextBrowser { background-color: #1b1d21; border: 1px solid #3a3f45; border-radius: 4px; padding: 8px; color: #d6dae0; }"));
    descText->setHtml(tr(
        "<p><b>All3DScan</b> is an open-source, clean-room native Linux scanning application with a dark Revo Scan 6-style interface.</p>"
        "<p>It targets consumer structured-light 3D scanners from <b>Revopoint, Creality, and Einstar</b> using Ubuntu's native USB (<code>uvcvideo</code>/<code>hidraw</code>) and GPU stack without Wine wrappers.</p>"
        "<p><b>Features:</b></p>"
        "<ul>"
        "<li>Native C++/Qt UI with deterministic frame queues</li>"
        "<li>POP4 hardware arming & raw Y16 streaming</li>"
        "<li>PCL Point Cloud Library integration (SOR filtering, MLS surface smoothing, ICP registration, RANSAC plane segmentation)</li>"
        "<li>Metric point cloud back-projection & post-scan editing</li>"
        "<li>PLY, OBJ, STL export & direct OrcaSlicer integration</li>"
        "<li>Embedded Python worker bridge for Open3D meshing</li>"
        "</ul>"
        "<p><b>License:</b> MIT License</p>"
    ));
    aboutLayout->addWidget(descText);
    tabs->addTab(aboutWidget, tr("About"));

    // --- Tab 2: Changelog ---
    auto* changelogWidget = new QWidget(tabs);
    auto* changelogLayout = new QVBoxLayout(changelogWidget);
    changelogLayout->setContentsMargins(14, 14, 14, 14);
    changelogLayout->setSpacing(10);

    auto* changelogText = new QTextBrowser(changelogWidget);
    changelogText->setStyleSheet(QStringLiteral(
        "QTextBrowser { background-color: #1b1d21; border: 1px solid #3a3f45; border-radius: 4px; padding: 8px; color: #d6dae0; }"));
    changelogText->setHtml(tr(
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1557</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Stereo Auto-Swap Startup Lock Fix</p>"
        "<ul>"
        "<li><b>Orientation Auto-Swap Fix:</b> Added validPoints >= 100 guard before caching auto-swap stereo choice, preventing unilluminated startup frames from permanently locking reversed L/R view orientation. Resetting autoSwapCache() on connect.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1550</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Emitter Control Raw Byte Scaling & Exposure Fix</p>"
        "<ul>"
        "<li><b>Raw Emitter Scaling:</b> Fixed raw hardware value mapping so illumination defaults to 130 (51%), laser defaults to 180 (70%), and exposure defaults to 3000us (50%), preventing minimum shutter time clamping.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1538</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Fringe Modulation Threshold Adjustment</p>"
        "<ul>"
        "<li><b>Official Modulation Threshold:</b> Lowered kModulationMin from 6.5f to 2.5f (matching the official APK libRPASpeckleStereo.so cutoff) to prevent discarding valid low-contrast fringe phase data across 97% of the surface.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1534</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Fringe Period Uniqueness Exclusion Fix</p>"
        "<ul>"
        "<li><b>Rival Exclusion Window Fix:</b> Fixed mathematical bug where rival cost search only excluded ±1px around the winner. Expanding exclusion window to |d - bestD| < 6 prevents comparing against the winner's local cost bowl shoulders, unlocking dense valid 3D points and eliminating concentric ring steps.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1528</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Structured Light Emitter Default Power Fix</p>"
        "<ul>"
        "<li><b>Active Projector Default Power:</b> Fixed a critical bug where initial illumination and laser power defaulted to 0%. Scanner now initializes with active emitter power levels (Illumination 50%, Laser 70%, Exposure 50%).</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1516</h3>"
        "<p style='color:#8b9198; font-size:11px;'>PCL Moving Least Squares (MLS) Surface Smoothing</p>"
        "<ul>"
        "<li><b>MLS Surface Smoothing Engine:</b> Integrated PCL's native <code>pcl::MovingLeastSquares</code> polynomial surface fitting engine to smooth point noise and re-compute surface normals.</li>"
        "<li><b>Edit Panel Controls:</b> Added the <b>MLS Smooth Surface (PCL)</b> action button to the post-processing editor.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1504</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Specular & Metallic Target Support</p>"
        "<ul>"
        "<li><b>Specular Target Matching:</b> Reduced the default intensity weighting to 0.05f to support shiny/reflective surfaces (like soda cans) where specular reflections differ between stereo views.</li>"
        "<li><b>Tightened Left-Right Consistency:</b> Tightened lrMaxDiff to 1.2px to filter out wrong-period periodic ghost matches geometrically.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1453</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Point Cloud Library (PCL) Outlier Removal Integration</p>"
        "<ul>"
        "<li><b>PCL (Point Cloud Library) Integration:</b> Replaced the extremely slow brute-force outlier removal filter with PCL's highly optimized, KdTree-based <code>pcl::StatisticalOutlierRemoval</code> filter.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1112</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Immediate Live Preview Mode & Fringe Ambiguity Resolution</p>"
        "<ul>"
        "<li><b>Immediate Live Preview:</b> Automatically start streaming on scanner connection, displaying a real-time live preview before starting the scan.</li>"
        "<li><b>Scan Accumulation Control:</b> Separated previewing and scanning: the Scan/Pause button now toggles model accumulation/fusion.</li>"
        "<li><b>Fringe Ambiguity Resolution:</b> Tightened uniqueness ratio to 0.90f and margin to 0.005f to eliminate concentric duplicate ghost sheets and resolve the true 3D object geometry.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1033</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Clean Grayscale IR Stream & Pill-shaped Floating Zoom Layout</p>"
        "<ul>"
        "<li><b>Clean Grayscale IR Stream:</b> Removed the experimental blue/cyan coverage overlay tint from the IR Stream albedo view, restoring a pure grayscale feed that matches Revo Scan.</li>"
        "<li><b>Pill-shaped Zoom Button Layout:</b> Embedded the zoom buttons inside a semi-transparent floating pill-shaped overlay widget directly on top of the video container, preventing them from shifting the video layout downward.</li>"
        "<li><b>Default Mode to Depth:</b> Set the default camera mode of the Depth Camera view to the colorized Depth Map on launch/stream.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1025</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Depth Map Colorization Mode & Digital Zoom Controls</p>"
        "<ul>"
        "<li><b>Depth Map Colormap Mode:</b> Toggle between raw IR grayscale view and a colorized metric depth map inside the Depth Camera preview widget, color-coded by distance zones.</li>"
        "<li><b>Digital Zoom crops:</b> Supported 1x, 1.5x, and 2x digital zoom inside the Depth Camera view to focus closely on scanner target geometry.</li>"
        "<li><b>Distance Scale Sync:</b> Synced range band boundaries dynamically on the vertical distance indicator with active scanner preset switches.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-1009</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Dynamic Depth Search Range & Active Accuracy Settings Control</p>"
        "<ul>"
        "<li><b>Dynamic Depth Search Range:</b> Pop4DepthDecoder dynamically updates search ranges based on real-time calibration parameter matrices, replacing static preset bounds.</li>"
        "<li><b>Active Accuracy Settings:</b> Connected the previously decorative point distance UI dropdowns to actively drive mesh resolution and voxels.</li>"
        "<li><b>Live Depth Lock Confidence:</b> Added depth-match ratio metrics and disparity spread output dynamically on-screen next to tracking.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v20260802-0058</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Reliable Hardware Emitter Detach Sequence & 70mm POP4 Baseline Calibration</p>"
        "<ul>"
        "<li><b>Hardware Emitter Command Detach:</b> Updated <code>applyCs7Settings</code> to perform a clean 30ms V4L2 pause and kernel driver detach, reliably triggering white/IR alignment LEDs and blue laser array on POP4 hardware.</li>"
        "<li><b>70mm Baseline Metric Calibration:</b> Corrected POP4 baseline distance from 55mm to 70mm in <code>Pop4DepthDecoder</code>, resolving metric depth scaling so scanned 3D point clouds preserve true object geometry.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<li><b>24H Timestamped Versioning:</b> Updated build versioning policy to <code>Alpha-vYYYYMMDD-HHMM</code> (e.g. <code>Alpha-v20260802-0052</code>).</li>"
        "<li><b>Timestamped Build Versioning:</b> Adopted ISO timestamp build tag scheme <code>Alpha-v20260802-0048</code>.</li>"
        "<li><b>3D Surface Relief Recovery:</b> Re-balanced modulation threshold (6.5), intensity weighting (0.08), and uniqueness margins in <code>Pop4DepthDecoder</code> to eliminate constant-fringe flat slab artifacts and capture true 3D object shape.</li>"
        "<li><b>Fringe Ambiguity Resolution:</b> Surface intensity and phase circular gradient weighting break single-frequency periodic ambiguity across curved surfaces.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.11</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Multi-Interface Emitter Transfers & Viewport Bounding Fix</p>"
        "<ul>"
        "<li><b>Multi-Interface LED Control:</b> Targeted CS7 illumination control transfers across both UVC Interface 0 (Depth) and Interface 2 (RGB) to physically light up white/IR alignment LEDs.</li>"
        "<li><b>Fused Model Bounding Fix:</b> Anchored <code>PointCloudView</code> camera projection bounds to the full accumulated point cloud, preventing live meshes from shrinking/shifting the 3D model view.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.10</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Direct Control Transfer Hardware Emitter Power & Mode Switching</p>"
        "<ul>"
        "<li><b>Direct Control Transfer:</b> Optimized USB <code>controlTransfer</code> on endpoint 0 to write CS7 projector and LED power settings directly while streaming without kernel driver detach conflicts.</li>"
        "<li><b>Alignment Light & Laser Mode Boost:</b> Configured Alignment Light and Blue Laser modes to boost exposure and laser intensity to maximum hardware thresholds.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.9</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Dense Phase Stereo Depth Decoding & Uniqueness Tuning</p>"
        "<ul>"
        "<li><b>Dense Depth Sampling:</b> Loosened over-strict phase ratio and cost margin gates in <code>Pop4DepthDecoder</code>, increasing valid depth points from ~180 points per frame to tens of thousands of continuous 3D surface points per frame.</li>"
        "<li><b>Solid Surface Reconstruction:</b> Dense depth point streaming enables rich 3D geometry reconstruction across object surfaces.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.8</h3>"
        "<p style='color:#8b9198; font-size:11px;'>RANSAC Desk Plane Removal & High-Precision Voxel Tuning</p>"
        "<ul>"
        "<li><b>RANSAC Ground Plane Removal:</b> Automatically detects and strips background table/desk surfaces during scanning so ICP registration locks strictly onto the 3D target object.</li>"
        "<li><b>High-Precision Voxel Downsampling:</b> Refined ICP voxel grid from 5.0 mm to 1.8 mm (and model resolution to 1.0 mm) to capture fine surface curves on small handheld objects.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.7</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Real-Time Fused 3D Point Cloud Scanning</p>"
        "<ul>"
        "<li><b>Continuous Real-Time 3D Fusion:</b> Fixed viewport renderer that previously hid the accumulated fused point cloud (100k+ points) during live scanning. Users can now watch the full 3D object geometry populate in real-time.</li>"
        "<li><b>Viewport Model Preservation:</b> Fixed <code>PointCloudView</code> to preserve the accumulated fused point cloud state when overlaying live mesh previews.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.6</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Instant Non-Blocking Hardware Emitter Control</p>"
        "<ul>"
        "<li><b>Fast V4L2 Extension Unit IOCTL:</b> Implemented sub-millisecond <code>UVCIOC_CTRL_QUERY</code> ioctl calls directly on active streaming file descriptors for real-time laser, flash light, and exposure control without stopping V4L2 streaming.</li>"
        "<li><b>Slider Debouncing:</b> Connected control sliders to released triggers to eliminate redundant UVC command bursts.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.5</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Hardware Projector, Flash Light & Blue Laser Controls</p>"
        "<ul>"
        "<li><b>Hardware Emitter Controls:</b> Enabled native UVC Extension Unit (CS7/CS10) commands for scanner Flash Light (Illumination), Blue Laser, Scan Mode selection, Alignment Light, and Emitters Off.</li>"
        "<li><b>USB Permissions:</b> Configured udev rules and non-root USB device access for direct libusb projector arming.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.4</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Continuous 3D Surface Triangulation & Mesh Repair</p>"
        "<ul>"
        "<li><b>Fixed Quad Rejection:</b> Fixed <code>DepthMesh</code> triangulation algorithm that previously discarded 99.9% of valid triangles, enabling tens of thousands of smooth continuous 3D surface triangles to generate.</li>"
        "<li><b>Dense Surface Mesh:</b> Increased grid sampling stride and edge threshold (60 mm) to form solid 3D surfaces instead of sparse specks.</li>"
        "<li><b>UI Performance:</b> Subsampled distance gauge computations to prevent main thread event loop freezing.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.3</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Point Cloud Library (PCL) Integration</p>"
        "<ul>"
        "<li><b>PCL Point Cloud Architecture:</b> Added optional native CMake & C++ integration for Point Cloud Library (PCL).</li>"
        "<li><b>Filtering & Alignment Scaffold:</b> Laid groundwork for real-time ICP frame registration, Statistical Outlier Removal (SOR), and RANSAC table segmentation.</li>"
        "<li><b>Build Infrastructure:</b> Integrated <code>find_package(PCL QUIET)</code> with fallback to high-performance built-in algorithms.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.2</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Incremental Update</p>"
        "<ul>"
        "<li><b>About & Changelog:</b> Added dedicated About menu entry and dialog with interactive Changelog tab.</li>"
        "<li><b>System Integration:</b> Installed native Linux main menu desktop entry (<code>io.github.all3dscan.All3DScan.desktop</code>) for instant application menu access.</li>"
        "<li><b>Versioning Scheme:</b> Enforced strict <code>0.0.1</code> step-increment versioning policy across all builds.</li>"
        "</ul>"
        "<hr style='border:none; border-top:1px solid #3a3f45; margin:12px 0;'>"
        "<h3 style='color:#12b6b0; margin-top:0;'>Version Alpha-v0.0.1</h3>"
        "<p style='color:#8b9198; font-size:11px;'>Initial Public Alpha Release</p>"
        "<ul>"
        "<li><b>UI & Navigation:</b> Implemented dark Revo Scan 6-style desktop shell with ribbon control bar, viewport stage tabs, and side panels.</li>"
        "<li><b>Device Support & Protocol:</b> Built multi-vendor auto-detection for Revopoint, Creality, and Einstar USB scanners.</li>"
        "<li><b>POP4 Capture Backend:</b> Native UVC Extension Unit CS10/CS7 probe, V4L2 MMAP frame acquisition, and 327,680 sample raw frame streaming.</li>"
        "<li><b>Metric Scaffolding & Visualizer:</b> GPU point-cloud viewport (QOpenGLWidget) with live depth colorization and telemetry.</li>"
        "<li><b>Post-Processing & Editing:</b> Interactive crop box, table/background removal, normal estimation, and Poisson surface reconstruction.</li>"
        "<li><b>3D Slicer Integration:</b> One-click export to STL/PLY/OBJ and direct handoff to OrcaSlicer / PrusaSlicer / Bambu Studio.</li>"
        "</ul>"
    ));
    changelogLayout->addWidget(changelogText);
    tabs->addTab(changelogWidget, tr("Changelog"));

    root->addWidget(tabs);

    auto* closeBtn = new QPushButton(tr("Close"), &dialog);
    connect(closeBtn, &QPushButton::clicked, &dialog, &QDialog::accept);
    auto* btnLayout = new QHBoxLayout();
    btnLayout->addStretch(1);
    btnLayout->addWidget(closeBtn);
    root->addLayout(btnLayout);

    dialog.exec();
}

QWidget* MainWindow::buildScanPage() {
    auto* page = new QWidget(this);
    auto* root = new QVBoxLayout(page);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    root->addWidget(buildRibbon());

    auto* body = new QHBoxLayout();
    body->setContentsMargins(0, 0, 0, 0);
    body->setSpacing(0);
    body->addWidget(buildLeftColumn());
    body->addWidget(buildViewport(), 1);

    auto* rightScroll = new QScrollArea(page);
    rightScroll->setWidgetResizable(true);
    rightScroll->setFixedWidth(260);
    rightScroll->setFrameShape(QFrame::NoFrame);
    auto* right = new QWidget(rightScroll);
    auto* rv = new QVBoxLayout(right);
    scannerControls_ = new ScannerControlPanel(right);
    rv->addWidget(scannerControls_);
    rv->addStretch(1);
    rightScroll->setWidget(right);
    body->addWidget(rightScroll);

    // Keyboard shortcut hint overlay (shown on ? key).
    shortcutHint_ = new QLabel(page);
    shortcutHint_->setObjectName("ShortcutHint");
    shortcutHint_->setAlignment(Qt::AlignCenter);
    shortcutHint_->setWordWrap(true);
    shortcutHint_->setStyleSheet(
        QStringLiteral(
            "background-color: rgba(16, 18, 20, 220);"
            "color: #d6dae0;"
            "border: 1px solid #3a3f45;"
            "border-radius: 6px;"
            "padding: 18px 24px;"
            "font-size: 12px;"));
    shortcutHint_->setText(
        QStringLiteral(
            "<b>Keyboard Shortcuts</b><br><br>"
            "Space &mdash; Start / Pause Scan<br>"
            "Esc &mdash; Cancel / Clear Selection<br>"
            "F &mdash; Fit View<br>"
            "R &mdash; Reset View<br>"
            "? &mdash; Toggle this overlay<br><br>"
            "<i>Selection tools (Edit page):</i><br>"
            "Click + drag &mdash; Rectangular / Lasso selection<br>"
            "Shift + Click + drag &mdash; Add to selection<br>"
            "Delete / Backspace &mdash; Remove selected points"));
    shortcutHint_->setVisible(false);
    shortcutHint_->raise();

    root->addLayout(body, 1);
    return page;
}

QWidget* MainWindow::buildEditPage() {
    auto* page = new QWidget(this);
    auto* root = new QVBoxLayout(page);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    // Edit page header, Revo-style: title + exit back to scanning.
    auto* header = new QFrame(page);
    header->setObjectName("Ribbon");
    header->setFixedHeight(48);
    auto* hh = new QHBoxLayout(header);
    hh->setContentsMargins(10, 4, 10, 4);
    auto* title = new QLabel(tr("Edit Scan"), header);
    title->setObjectName("PanelTitle");
    hh->addWidget(title);
    editInfoLabel_ = new QLabel(header);
    editInfoLabel_->setObjectName("StatusHint");
    hh->addWidget(editInfoLabel_);
    hh->addStretch(1);
    auto* openBtn = new QPushButton(tr("Open PLY…"), header);
    connect(openBtn, &QPushButton::clicked, this, &MainWindow::onImportPly);
    hh->addWidget(openBtn);
    auto* exportBtn = new QPushButton(tr("Export…"), header);
    connect(exportBtn, &QPushButton::clicked, this, &MainWindow::onExport);
    hh->addWidget(exportBtn);
    auto* slicerBtn = new QPushButton(tr("Send to OrcaSlicer"), header);
    slicerBtn->setToolTip(
        tr("Mesh the scan (Complete first), export STL in mm and open it in "
           "OrcaSlicer / PrusaSlicer / Bambu Studio"));
    connect(slicerBtn, &QPushButton::clicked, this, &MainWindow::onSendToSlicer);
    hh->addWidget(slicerBtn);
    auto* editNewScanBtn = new QPushButton(tr("New Scan"), header);
    editNewScanBtn->setToolTip(tr("Discard this model and go back to scanning (Ctrl+N)."));
    connect(editNewScanBtn, &QPushButton::clicked, this, &MainWindow::onNewScan);
    hh->addWidget(editNewScanBtn);
    auto* exitBtn = new QPushButton(tr("Exit Edit Mode"), header);
    connect(exitBtn, &QPushButton::clicked, this, &MainWindow::showScanPage);
    hh->addWidget(exitBtn);
    root->addWidget(header);

    auto* body = new QHBoxLayout();
    body->setContentsMargins(0, 0, 0, 0);
    body->setSpacing(0);

    auto* viewArea = new QWidget(page);
    auto* vv = new QVBoxLayout(viewArea);
    vv->setContentsMargins(0, 0, 0, 0);
    vv->setSpacing(0);

    // Model stage tabs (Revo Scan: Raw / Point Clouds / Meshed / Textured).
    auto* stageBar = new QFrame(viewArea);
    stageBar->setObjectName("SegmentedToggle");
    auto* sh = new QHBoxLayout(stageBar);
    sh->setContentsMargins(8, 4, 8, 4);
    sh->setSpacing(6);
    sh->addStretch(1);
    stageRawBtn_ = new QPushButton(tr("Raw"), stageBar);
    stagePointsBtn_ = new QPushButton(tr("Point Clouds"), stageBar);
    stageMeshBtn_ = new QPushButton(tr("Meshed"), stageBar);
    stageTexBtn_ = new QPushButton(tr("Textured"), stageBar);
    for (QPushButton* b : {stageRawBtn_, stagePointsBtn_, stageMeshBtn_, stageTexBtn_}) {
        b->setCheckable(true);
        sh->addWidget(b);
    }
    sh->addStretch(1);
    vv->addWidget(stageBar);
    connect(stageRawBtn_, &QPushButton::clicked, this,
            [this] { applyModelStage(ModelStage::Raw); });
    connect(stagePointsBtn_, &QPushButton::clicked, this,
            [this] { applyModelStage(ModelStage::PointClouds); });
    connect(stageMeshBtn_, &QPushButton::clicked, this,
            [this] { applyModelStage(ModelStage::Meshed); });
    connect(stageTexBtn_, &QPushButton::clicked, this,
            [this] { applyModelStage(ModelStage::Textured); });

    editView_ = new PointCloudView(viewArea);
    vv->addWidget(editView_, 1);

    // Selection toolbar (matches Revo Scan's bottom edit bar).
    auto* editBar = new QFrame(viewArea);
    editSelectBar_ = editBar;
    editBar->setObjectName("SegmentedToggle");
    auto* eh = new QHBoxLayout(editBar);
    eh->setContentsMargins(8, 4, 8, 4);
    eh->setSpacing(6);
    eh->addStretch(1);

    auto* orbitBtn = new QPushButton(tr("Orbit"), editBar);
    auto* rectBtn = new QPushButton(tr("Rectangular Selection"), editBar);
    auto* lassoBtn = new QPushButton(tr("Lasso Selection"), editBar);
    for (QPushButton* b : {orbitBtn, rectBtn, lassoBtn}) {
        b->setCheckable(true);
        eh->addWidget(b);
    }
    orbitBtn->setChecked(true);

    auto* invertBtn = new QPushButton(tr("Invert Selection"), editBar);
    deleteSelBtn_ = new QPushButton(tr("Delete"), editBar);
    auto* clearSelBtn = new QPushButton(tr("Clear"), editBar);
    deleteSelBtn_->setEnabled(false);
    undoButton_ = new QPushButton(tr("Undo"), editBar);
    redoButton_ = new QPushButton(tr("Redo"), editBar);
    eh->addWidget(invertBtn);
    eh->addWidget(deleteSelBtn_);
    eh->addWidget(clearSelBtn);
    eh->addWidget(undoButton_);
    eh->addWidget(redoButton_);
    eh->addStretch(1);
    vv->addWidget(editBar);

    auto setTool = [this, orbitBtn, rectBtn, lassoBtn](PointCloudView::Tool tool) {
        orbitBtn->setChecked(tool == PointCloudView::Tool::Orbit);
        rectBtn->setChecked(tool == PointCloudView::Tool::RectSelect);
        lassoBtn->setChecked(tool == PointCloudView::Tool::LassoSelect);
        editView_->setTool(tool);
    };
    connect(orbitBtn, &QPushButton::clicked, this,
            [setTool] { setTool(PointCloudView::Tool::Orbit); });
    connect(rectBtn, &QPushButton::clicked, this,
            [setTool] { setTool(PointCloudView::Tool::RectSelect); });
    connect(lassoBtn, &QPushButton::clicked, this,
            [setTool] { setTool(PointCloudView::Tool::LassoSelect); });
    connect(invertBtn, &QPushButton::clicked, editView_, &PointCloudView::invertSelection);
    connect(clearSelBtn, &QPushButton::clicked, editView_, &PointCloudView::clearSelection);
    connect(editView_, &PointCloudView::selectionChanged, this, [this](int count) {
        deleteSelBtn_->setEnabled(count > 0 && document_.isMetric());
    });
    connect(deleteSelBtn_, &QPushButton::clicked, this, &MainWindow::onDeleteSelection);
    connect(undoButton_, &QPushButton::clicked, this, &MainWindow::onUndo);
    connect(redoButton_, &QPushButton::clicked, this, &MainWindow::onRedo);

    body->addWidget(viewArea, 1);

    auto* rightScroll = new QScrollArea(page);
    rightScroll->setWidgetResizable(true);
    rightScroll->setFixedWidth(260);
    rightScroll->setFrameShape(QFrame::NoFrame);
    auto* right = new QWidget(rightScroll);
    auto* rv = new QVBoxLayout(right);
    editPanel_ = new EditPanel(&document_, right);
    rv->addWidget(editPanel_);
    rv->addStretch(1);
    rightScroll->setWidget(right);
    body->addWidget(rightScroll);

    root->addLayout(body, 1);
    return page;
}

QWidget* MainWindow::buildRibbon() {
    auto* ribbon = new QFrame(this);
    ribbon->setObjectName("Ribbon");
    ribbon->setFixedHeight(64);
    auto* h = new QHBoxLayout(ribbon);
    h->setContentsMargins(6, 4, 6, 4);
    h->setSpacing(0);

    accuracyCombo_ = ribbonCombo({tr("High Accuracy"), tr("Standard")}, ribbon);
    accuracyCombo_->setToolTip(
        tr("Point distance: the spacing between fused surface points.\n"
           "High Accuracy (1.0 mm) keeps fine detail like seams and labels.\n"
           "Standard (2.2 mm) fuses faster and is lighter on large scans."));
    connect(accuracyCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &MainWindow::onAccuracyChanged);
    h->addWidget(ribbonGroup(tr("Accuracy"), accuracyCombo_, ribbon));
    h->addWidget(ribbonGroup(tr("Object Type"),
                             ribbonCombo({tr("Normal"), tr("Dark"), tr("Marker")}, ribbon), ribbon));
    distanceCombo_ = ribbonCombo({tr("120-219mm"), tr("210-400mm"), tr("250-500mm (full-field)")}, ribbon);
    distanceCombo_->setCurrentIndex(2); // POP4 full-field default range
    distanceCombo_->setToolTip(
        tr("Working distance used to isolate the object from background.\n"
           "Full-field structured light is strongest around 250–500 mm.\n"
           "Pick the range that matches how far the scanner is from the object."));
    connect(distanceCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &MainWindow::onScanDistanceChanged);
    h->addWidget(ribbonGroup(tr("Scanning Distance"), distanceCombo_, ribbon));
    h->addWidget(ribbonSeparator(ribbon));

    colorButton_ = new QPushButton(tr("Color Scanning"), ribbon);
    colorButton_->setCheckable(true);
    h->addWidget(ribbonGroup(tr("RGB"), colorButton_, ribbon));
    h->addWidget(ribbonSeparator(ribbon));

    openEditBtn_ = new QPushButton(tr("Edit Scan…"), ribbon);
    connect(openEditBtn_, &QPushButton::clicked, this, &MainWindow::showEditPage);
    h->addWidget(ribbonGroup(tr("Post-process"), openEditBtn_, ribbon));

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

    auto* viewArea = new QWidget(container);
    auto* vv = new QVBoxLayout(viewArea);
    vv->setContentsMargins(0, 0, 0, 0);
    vv->setSpacing(0);

    auto* topRow = new QHBoxLayout();
    topRow->setContentsMargins(8, 8, 8, 0);
    trackLabel_ = new QLabel(tr("Tracking: Idle"), viewArea);
    trackLabel_->setStyleSheet("color:#5b6169;");
    topRow->addWidget(trackLabel_);
    depthLockLabel_ = new QLabel(viewArea);
    depthLockLabel_->setStyleSheet("color:#5b6169;");
    depthLockLabel_->setToolTip(
        tr("Fraction of pixels where the stereo phase decoder found a confident "
           "match, and how spread out the resulting disparities are. Low/near-zero "
           "spread usually means the object is outside the current Scanning "
           "Distance band, or the surface lacks IR texture for phase matching."));
    depthLockLabel_->hide();
    topRow->addSpacing(12);
    topRow->addWidget(depthLockLabel_);
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
        refreshScanView();
        refreshEditView();
    });
    connect(noColorButton_, &QPushButton::clicked, this, [this, colorViewBtn] {
        useColor_ = false;
        colorViewBtn->setChecked(false);
        noColorButton_->setChecked(true);
        refreshScanView();
        refreshEditView();
    });

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
    newScanButton_ = new QPushButton(tr("New Scan"), controls);
    newScanButton_->setToolTip(tr("Discard the current scan and start a fresh one (Ctrl+N).\n"
                                  "The scanner stays connected."));
    for (QPushButton* b :
         {scanButton_, singleShotButton_, completeButton_, cancelButton_, newScanButton_}) {
        ch->addWidget(b);
    }

    isolateButton_ = new QPushButton(tr("Isolate Object"), controls);
    isolateButton_->setCheckable(true);
    isolateButton_->setChecked(objectIsolation_);
    isolateButton_->setToolTip(
        tr("Scan only the nearest object: clips to a working-distance band and\n"
           "keeps the dominant connected cluster (drops background/table)."));
    ch->addWidget(isolateButton_);

    fusionButton_ = new QPushButton(tr("Live Fusion"), controls);
    fusionButton_->setCheckable(true);
    fusionButton_->setChecked(liveFusion_);
    fusionButton_->setToolTip(
        tr("Real-time ICP registration: aligns and fuses each frame into a\n"
           "growing model (like Revo Scan). Off = static max-hold preview."));
    ch->addWidget(fusionButton_);

    liveMeshButton_ = new QPushButton(tr("Live Mesh"), controls);
    liveMeshButton_->setCheckable(true);
    liveMeshButton_->setChecked(liveMeshPreview_);
    liveMeshButton_->setToolTip(
        tr("Live 3D polygon model: tracked frames are fused into a volume and\n"
           "re-meshed about twice a second. Off = point-cloud view."));
    ch->addWidget(liveMeshButton_);

    arOverlayButton_ = new QPushButton(tr("AR Overlay"), controls);
    arOverlayButton_->setCheckable(true);
    arOverlayButton_->setChecked(true);
    arOverlayButton_->setToolTip(
        tr("Real-time Augmented Reality overlay of 3D scan geometry on camera stream."));
    ch->addWidget(arOverlayButton_);

    ch->addStretch(1);
    vv->addWidget(controls);

    h->addWidget(viewArea, 1);

    connect(scanButton_, &QPushButton::clicked, this, &MainWindow::onScanToggle);
    connect(singleShotButton_, &QPushButton::clicked, this, &MainWindow::onSingleShot);
    connect(completeButton_, &QPushButton::clicked, this, &MainWindow::onComplete);
    connect(cancelButton_, &QPushButton::clicked, this, &MainWindow::onCancel);
    connect(newScanButton_, &QPushButton::clicked, this, &MainWindow::onNewScan);
    connect(fusionButton_, &QPushButton::toggled, this, &MainWindow::onFusionToggled);
    connect(isolateButton_, &QPushButton::toggled, this, &MainWindow::onIsolateToggled);
    connect(liveMeshButton_, &QPushButton::toggled, this, [this](bool on) {
        liveMeshPreview_ = on;
        refreshScanView();
    });
    connect(arOverlayButton_, &QPushButton::toggled, this, [this](bool on) {
        if (arOverlayView_) {
            arOverlayView_->setArEnabled(on);
        }
    });
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
           "Complete or Edit Scan… opens the Edit page"),
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

    connectBtn_ = new QPushButton(tr("Connect"), bar);
    connect(connectBtn_, &QPushButton::clicked, this, &MainWindow::onConnectToggle);
    connect(controller_.get(), &ScannerController::connectFinished, this,
            &MainWindow::onConnectFinished);
    h->addWidget(connectBtn_);
    return bar;
}

void MainWindow::showScanPage() {
    pages_->setCurrentIndex(0);
}

void MainWindow::showEditPage() {
    if (!document_.isMetric() || document_.cloudEmpty()) {
        onError(tr("Edit page needs a completed metric scan or an imported PLY"));
    }
    refreshEditView();
    pages_->setCurrentIndex(1);
}

void MainWindow::onConnectWifi() {
    bool ok = false;
    const QString current = wifiControl_->host().isEmpty()
                                ? QStringLiteral("192.168.3.1")
                                : wifiControl_->host();
    const QString ip = QInputDialog::getText(
        this, tr("Connect over WiFi"),
        tr("Scanner IP (POP4 WiFi host mode):"), QLineEdit::Normal, current,
        &ok);
    if (!ok || ip.trimmed().isEmpty()) {
        return;
    }
    wifiControl_->setHost(ip.trimmed());
    connLabel_->setText(tr("● Probing WiFi scanner at %1…").arg(ip.trimmed()));
    connLabel_->setStyleSheet("color:#d0a24c;");
    wifiControl_->probe();
}

void MainWindow::onConnectToggle() {
    if (controller_->isConnecting()) {
        return;
    }
    if (!connected_) {
        // Arming can take tens of seconds if the scanner needs a reset and
        // reboot; keep the window responsive and finish in onConnectFinished().
        connectBtn_->setEnabled(false);
        connectBtn_->setText(tr("Connecting…"));
        connLabel_->setText(tr("● Connecting… (up to ~45 s if the scanner has to reboot)"));
        connLabel_->setStyleSheet("color:#d0a24c;");
        controller_->connectDeviceAsync();
        if (scannerControls_) {
            scannerControls_->refreshFromProtocol();
        }
        return;
    }
    controller_->disconnectDevice();
    connected_ = false;
    streaming_ = false;
    scanning_ = false;
    if (scannerControls_) {
        scannerControls_->refreshFromProtocol(); // grey the scanner controls out
    }
    connectBtn_->setText(tr("Connect"));
    connLabel_->setText(tr("● Disconnected"));
    connLabel_->setStyleSheet("color:#d95c5c;");
    updateControlStates();
}

void MainWindow::onConnectFinished(bool ok) {
    connected_ = ok;
    if (scannerControls_) {
        // Also on failure: the panel must stop offering scanner controls.
        scannerControls_->bindProtocol(controller_->protocol());
    }
    streaming_ = connected_ && controller_->startScan();
    connectBtn_->setEnabled(true);
    connectBtn_->setText(connected_ ? tr("Disconnect") : tr("Connect"));
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
    if (!connected_ || completing_) {
        return;
    }
    // Complete / Cancel stop the camera stream; restart it so Scan works
    // again without reconnecting.
    if (!scanning_ && !streaming_) {
        streaming_ = controller_->startScan();
        if (!streaming_) {
            return;
        }
    }
    scanning_ = !scanning_;
    scanButton_->setText(scanning_ ? tr("Pause") : tr("Scan"));
    updateControlStates();
}

void MainWindow::onSingleShot() {
    if (connected_) {
        controller_->captureSingleFrame();
    }
}

void MainWindow::onComplete() {
    if (completing_) {
        return;
    }
    if (streaming_) {
        controller_->stopScan();
        streaming_ = false;
        scanning_ = false;
        scanButton_->setText(tr("Scan"));
    }
    if (!document_.isMetric()) {
        onError(tr("Complete/meshing needs metric depth (scan until full-field decode succeeds)"));
        updateControlStates();
        return;
    }

    // Meshing can take many seconds on a long scan (hundreds of keyframes into
    // a large volume). Doing it on the UI thread froze the window, so run it
    // in the background and keep the window responsive but locked.
    completing_ = true;
    if (centralWidget()) {
        centralWidget()->setEnabled(false);
    }
    connLabel_->setText(tr("● Building mesh…"));
    connLabel_->setStyleSheet("color:#d0a24c;");
    if (completeThread_.joinable()) {
        completeThread_.join();
    }
    const std::string worker =
        util::findMeshWorkerScript().value_or("tools/pipeline/mesh_worker.py");
    completeThread_ = std::thread([this, worker] {
        std::string error;
        const bool ok = pipeline_->completeAndMesh(worker, "python3", &error);
        QMetaObject::invokeMethod(
            this,
            [this, ok, error] { onCompleteFinished(ok, QString::fromStdString(error)); },
            Qt::QueuedConnection);
    });
}

void MainWindow::onCompleteFinished(bool ok, const QString& error) {
    completing_ = false;
    if (centralWidget()) {
        centralWidget()->setEnabled(true);
    }
    if (!ok) {
        onError(tr("Mesh complete: %1").arg(error));
    } else {
        connLabel_->setText(tr("● Scan complete — mesh ready"));
        connLabel_->setStyleSheet("color:#5bbf5b;");
    }
    // Land on the richest available stage, like Revo Scan's Model view.
    if (!document_.meshEmpty()) {
        modelStage_ = document_.meshHasColors() ? ModelStage::Textured : ModelStage::Meshed;
    } else {
        modelStage_ = ModelStage::PointClouds;
    }
    showEditPage();
    updateControlStates();
}

void MainWindow::onCancel() {
    if (completing_) {
        return;
    }
    if (streaming_) {
        controller_->stopScan();
        streaming_ = false;
    }
    discardScan();
}

void MainWindow::onNewScan() {
    if (completing_) {
        return;
    }
    if (!document_.cloudEmpty() || !document_.meshEmpty()) {
        const auto answer = QMessageBox::question(
            this, tr("New Scan"),
            tr("Start a new scan? The current scan and mesh will be discarded.\n\n"
               "Use File → Export… first if you want to keep it."),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) {
            return;
        }
    }
    discardScan();
    showScanPage();
    // Keep the scanner live and ready: restart the preview if Complete or
    // Cancel stopped it. Scanning itself starts when the user presses Scan.
    if (connected_ && !streaming_) {
        streaming_ = controller_->startScan();
    }
    if (connLabel_ && connected_) {
        connLabel_->setText(tr("● New scan — press Scan to start"));
        connLabel_->setStyleSheet("color:#5bbf5b;");
    }
    updateControlStates();
}

void MainWindow::discardScan() {
    scanning_ = false;
    scanButton_->setText(tr("Scan"));
    controller_->resetAccumulation();
    // Resetting the pipeline drops its keyframes, live TSDF and live mesh and
    // clears the document; clearing the document alone left the old live mesh
    // on screen and fed old keyframes into the next Complete.
    if (pipeline_) {
        pipeline_->reset();
    }
    document_.clear();
    showingLiveMesh_ = false;
    shownLiveMeshRev_ = 0;
    modelStage_ = ModelStage::PointClouds;
    if (trackLabel_) {
        trackLabel_->setText(liveFusion_ ? tr("Tracking: Idle") : tr("Fusion off"));
        trackLabel_->setStyleSheet("color:#5b6169;");
    }
    refreshScanView();
    refreshEditView();
    updateControlStates();
}

void MainWindow::onUndo() {
    if (document_.undo()) {
        refreshEditView();
        refreshScanView();
        updateControlStates();
    }
}

void MainWindow::onRedo() {
    if (document_.redo()) {
        refreshEditView();
        refreshScanView();
        updateControlStates();
    }
}

void MainWindow::onDeleteSelection() {
    if (!document_.isMetric()) {
        onError(tr("Selection delete requires a metric scan document"));
        return;
    }
    const auto indices = editView_->selectedIndices();
    if (indices.empty()) {
        return;
    }
    if (!document_.deleteSelected(indices)) {
        onError(tr("Delete selection failed"));
        return;
    }
    editView_->clearSelection();
    onDocumentUpdated();
}

void MainWindow::onImportPly() {
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Open Point Cloud"), QString(), tr("PLY (*.ply)"));
    if (path.isEmpty()) {
        return;
    }
    std::string error;
    auto cloud = io::readPlyCloud(path.toStdString(), &error);
    if (!cloud) {
        QMessageBox::critical(this, tr("Open PLY"), QString::fromStdString(error));
        return;
    }
    document_.setCloud(std::move(*cloud), true, "import");
    modelStage_ = ModelStage::PointClouds;
    showEditPage();
    updateControlStates();
}

void MainWindow::onExport() {
    if (!document_.isMetric() && !controller_->lastCloudMetric()) {
        QMessageBox::warning(
            this,
            tr("Export"),
            tr("Refusing to export non-metric / vendor-raw preview geometry.\n"
               "Use ALL3DSCAN_BACKEND=mock for metric pipeline testing, or wait "
               "for POP4 capture-backed decode."));
        return;
    }

    // One consistent copy: the scan pipeline may still be fusing.
    const geometry::TriangleMesh mesh = document_.meshCopy();
    const pointcloud::PointCloud docCloud = document_.cloudCopy();
    const bool hasMesh = !mesh.empty();
    const QString filter = hasMesh
                               ? tr("Binary STL (*.stl);;OBJ Mesh (*.obj);;PLY Mesh (*.ply);;PLY Cloud (*.ply)")
                               : tr("PLY Cloud (*.ply);;OBJ Points (*.obj)");
    const QString path =
        QFileDialog::getSaveFileName(this, tr("Export Geometry"), QString(), filter);
    if (path.isEmpty()) {
        return;
    }

    io::ExportOptions options;
    options.requireMetric = true;
    options.unit = io::LengthUnit::Meters;
    if (path.endsWith(".stl", Qt::CaseInsensitive)) {
        options.unit = io::LengthUnit::Millimeters; // STL has no units; use mm for printing
        auto result = io::writeStlBinary(path.toStdString(), mesh, options);
        if (!result.ok) {
            QMessageBox::critical(this, tr("Export"), QString::fromStdString(result.error));
        }
        return;
    }

    if (path.endsWith(".obj", Qt::CaseInsensitive)) {
        if (hasMesh) {
            auto result = io::writeObjMesh(path.toStdString(), mesh, options);
            if (!result.ok) {
                QMessageBox::critical(this, tr("Export"), QString::fromStdString(result.error));
            }
        } else if (document_.isMetric()) {
            if (!io::writeObj(path.toStdString(), docCloud)) {
                QMessageBox::critical(this, tr("Export"), tr("Failed to write OBJ."));
            }
        } else {
            QMessageBox::warning(this, tr("Export"), tr("No metric geometry to export."));
        }
        return;
    }

    if (hasMesh && path.endsWith(".ply", Qt::CaseInsensitive)) {
        auto result = io::writePlyMesh(path.toStdString(), mesh, options);
        if (!result.ok) {
            result = io::writePlyCloud(path.toStdString(), docCloud, true, options);
        }
        if (!result.ok) {
            QMessageBox::critical(this, tr("Export"), QString::fromStdString(result.error));
        }
        return;
    }

    auto result = io::writePlyCloud(path.toStdString(),
                                    document_.isMetric() ? docCloud
                                                         : controller_->lastCloud(),
                                    document_.isMetric() || controller_->lastCloudMetric(),
                                    options);
    if (!result.ok) {
        QMessageBox::critical(this, tr("Export"), QString::fromStdString(result.error));
    }
}

void MainWindow::onSendToSlicer() {
    const geometry::TriangleMesh mesh = document_.meshCopy();
    // Slicers need a watertight, metric mesh — not a raw point cloud.
    if (mesh.empty()) {
        QMessageBox::information(
            this,
            tr("Send to OrcaSlicer"),
            tr("A slicer needs a watertight mesh, not a point cloud.\n\n"
               "Press \"Complete\" after scanning to build the mesh (or open a "
               "mesh), then send it to the slicer.\n\n"
               "POP4 full-field decode produces approximate millimetres; if the "
               "cloud is still raw preview, wait for a packed 640×2560 decode or "
               "use ALL3DSCAN_BACKEND=mock to test the full mesh → slice flow."));
        return;
    }
    if (!mesh.metric) {
        QMessageBox::warning(
            this,
            tr("Send to OrcaSlicer"),
            tr("Refusing to slice non-metric geometry (dimensions would be "
               "wrong)."));
        return;
    }

    const auto slicer = util::findSlicerCommand();
    if (!slicer) {
        QMessageBox::warning(
            this,
            tr("Send to OrcaSlicer"),
            tr("No slicer found. Install OrcaSlicer (flatpak "
               "com.orcaslicer.OrcaSlicer), PrusaSlicer or Bambu Studio, or set "
               "ALL3DSCAN_SLICER to its launch command."));
        return;
    }

    // Export STL (mm) to a stable, slicer-accessible location. Flatpak slicers
    // only see files under $HOME, so avoid /tmp.
    const QString outDir = QDir::homePath() + "/All3DScan";
    QDir().mkpath(outDir);
    const QString stlPath =
        outDir + "/scan_" +
        QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".stl";

    io::ExportOptions options;
    options.requireMetric = true;
    options.unit = io::LengthUnit::Millimeters;
    auto result = io::writeStlBinary(stlPath.toStdString(), mesh, options);
    if (!result.ok) {
        QMessageBox::critical(this, tr("Send to OrcaSlicer"),
                              QString::fromStdString(result.error));
        return;
    }

    QStringList args;
    for (size_t i = 1; i < slicer->size(); ++i) {
        args << QString::fromStdString((*slicer)[i]);
    }
    args << stlPath;
    const QString program = QString::fromStdString(slicer->front());

    if (!QProcess::startDetached(program, args)) {
        QMessageBox::warning(
            this,
            tr("Send to OrcaSlicer"),
            tr("Exported %1 but failed to launch the slicer (%2).\n"
               "Open the STL manually.")
                .arg(stlPath, program));
        return;
    }
    connLabel_->setText(tr("● Sent %1 to slicer").arg(QDir(stlPath).dirName()));
    connLabel_->setStyleSheet("color:#5bbf5b;");
}

void MainWindow::onFrameReady(const pointcloud::PointCloud& cloud) {
    if (depthLockLabel_ && controller_ && controller_->protocol()) {
        const auto stats = controller_->protocol()->depthDecodeStats();
        if (stats.attempted) {
            const float validPct = stats.validFraction() * 100.0f;
            depthLockLabel_->setText(tr("Depth lock: %1% (%2, σ=%3px)")
                                         .arg(validPct, 0, 'f', 0)
                                         .arg(QString::fromStdString(stats.backend))
                                         .arg(stats.dispStdPx, 0, 'f', 1));
            // Low valid% or near-zero disparity spread both indicate the
            // decoder is guessing rather than resolving real surface relief
            // (see docs/protocol/RE_NOTES.md — periodic phase ambiguity).
            const char* color = "#5b6169";
            if (validPct >= 40.0f && stats.dispStdPx >= 3.0f) {
                color = "#5bbf5b";
            } else if (validPct >= 15.0f) {
                color = "#e0a83e";
            } else {
                color = "#d9534f";
            }
            depthLockLabel_->setStyleSheet(QString("color:%1;").arg(color));
            depthLockLabel_->show();
        } else {
            depthLockLabel_->hide();
        }
    }
    // When live fusion is building a model, the fused document drives the view
    // (via onDocumentUpdated); don't overwrite it with the single raw frame.
    const bool docEmpty = document_.cloudEmpty();
    const bool fusedActive = liveFusion_ && !docEmpty;
    if (!fusedActive && (!document_.isMetric() || docEmpty)) {
        view_->setColorEnabled(useColor_);
        view_->setPointCloud(cloud);
        std::vector<float> depths;
        depths.reserve(cloud.points.size());
        for (const auto& p : cloud.points) {
            depths.push_back(p.z * 1000.0f);
        }
        distanceScale_->setDepthSamplesMm(depths);
    }
    updateFrameTelemetry();
}

void MainWindow::onMetricFrame(const pointcloud::PointCloud& cloud, quint64 timestampUs) {
    if (scanning_ && pipeline_) {
        // Forward the full metric depth map + intrinsics so the pipeline can
        // retain keyframes for volumetric (TSDF) reconstruction on Complete.
        if (controller_) {
            pipeline_->submitMetricFrame(cloud, controller_->lastMetricDepth(),
                                         controller_->lastIntrinsics(), timestampUs);
        } else {
            pipeline_->submitMetricCloud(cloud, timestampUs);
        }
    }
    if (connLabel_) {
        connLabel_->setText(
            tr("● POP4 full-field (approx mm) — %1 points")
                .arg(cloud.points.size()));
        connLabel_->setStyleSheet("color:#5bbf5b;");
    }
    (void)timestampUs;
}

void MainWindow::onPreviewFrame(const pointcloud::PointCloud& cloud, quint64 timestampUs) {
    if (scanning_ && liveFusion_ && pipeline_) {
        pipeline_->submitPreviewCloud(cloud, timestampUs);
    }
}

void MainWindow::onIsolateToggled(bool on) {
    objectIsolation_ = on;
    if (controller_) {
        controller_->setObjectIsolation(on);
        controller_->resetAccumulation();
    }
    if (pipeline_) {
        pipeline_->reset(); // rebuild model with the new isolation setting
    }
    refreshScanView();
}

void MainWindow::onScanDistanceChanged(int index) {
    if (!controller_) {
        return;
    }
    pointcloud::ForegroundIsolation params = controller_->isolationParams();
    // Ranges match RevoScan ribbon presets / param_pop4 working volume.
    switch (index) {
    case 0: // 120-219mm — close-up small objects
        params.nearClipM = 0.10f;
        params.farClipM = 0.25f;
        params.nearBandM = 0.08f;
        activeBandEdgesMm_[0] = 120.0f;
        activeBandEdgesMm_[1] = 160.0f;
        activeBandEdgesMm_[2] = 200.0f;
        activeBandEdgesMm_[3] = 230.0f;
        break;
    case 1: // 210-400mm — typical POP4 object scan
        params.nearClipM = 0.21f;
        params.farClipM = 0.40f;
        params.nearBandM = 0.10f;
        activeBandEdgesMm_[0] = 240.0f;
        activeBandEdgesMm_[1] = 290.0f;
        activeBandEdgesMm_[2] = 340.0f;
        activeBandEdgesMm_[3] = 380.0f;
        break;
    default: // 250-500mm — full-field high-accuracy volume
        params.nearClipM = 0.25f;
        params.farClipM = 0.50f;
        params.nearBandM = 0.12f;
        activeBandEdgesMm_[0] = 290.0f;
        activeBandEdgesMm_[1] = 350.0f;
        activeBandEdgesMm_[2] = 400.0f;
        activeBandEdgesMm_[3] = 460.0f;
        break;
    }
    controller_->setIsolationParams(params);
    controller_->resetAccumulation();
    if (pipeline_) {
        pipeline_->setDepthClip(params.nearClipM, params.farClipM);
        if (liveFusion_) {
            pipeline_->reset();
        }
    }
    // Sync the distance-scale widget to the chosen working volume.
    if (distanceScale_) {
        distanceScale_->setRangeMm(params.nearClipM * 1000.0f, params.farClipM * 1000.0f);
        distanceScale_->setBandEdgesMm(activeBandEdgesMm_[0], activeBandEdgesMm_[1],
                                        activeBandEdgesMm_[2], activeBandEdgesMm_[3]);
    }
    // Retune the decoder's own disparity/depth search band to match — without
    // this the phase decoder always searched a fixed ~220-650mm band no
    // matter which preset was picked, so close presets silently couldn't find
    // near-surface disparities.
    if (controller_->protocol()) {
        controller_->protocol()->setDepthRangeHint(params.nearClipM * 1000.0f,
                                                    params.farClipM * 1000.0f);
    }
}

void MainWindow::onAccuracyChanged(int index) {
    if (!pipeline_) {
        return;
    }
    // "Point distance" trade-off (Revo Scan 6 terminology): smaller spacing
    // keeps fine surface detail at the cost of fusion speed / point count.
    const float pointDistanceMm = index == 0 ? 1.0f : 2.2f;
    pipeline_->setPointDistance(pointDistanceMm);
    if (liveFusion_) {
        pipeline_->reset(); // rebuild the model at the new point spacing
    }
}

void MainWindow::onFusionToggled(bool on) {
    liveFusion_ = on;
    if (controller_) {
        controller_->setFusionEnabled(on);
        controller_->resetAccumulation();
    }
    if (pipeline_) {
        pipeline_->reset(); // clears the live model so the mode switch starts clean
    }
    if (trackLabel_) {
        trackLabel_->setText(on ? tr("Tracking: Idle") : tr("Fusion off"));
        trackLabel_->setStyleSheet("color:#5b6169;");
    }
    refreshScanView();
}

void MainWindow::onDocumentUpdated() {
    refreshScanView();
    refreshEditView();
    updateControlStates();
}

void MainWindow::refreshScanView() {
    // The pipeline fuses into document_ on its worker thread: read it only
    // through the document's locked accessors (a plain reference raced with
    // fusion and crashed in PointCloudView::recomputeBounds).
    const bool docEmpty = document_.cloudEmpty();
    const bool showFused = (document_.isMetric() || liveFusion_) && !docEmpty;
    view_->setColorEnabled(useColor_);

    // Live polygon model (TSDF) whenever it exists. Metric frames are always
    // fused, so this no longer depends on the Live Fusion toggle. Only
    // re-upload when the pipeline published a new mesh or colour changed.
    const bool showLiveMesh = liveMeshPreview_ && pipeline_ && pipeline_->hasLiveMesh();
    if (showLiveMesh) {
        const uint64_t rev = pipeline_->liveMeshRevision();
        if (!showingLiveMesh_ || rev != shownLiveMeshRev_ || useColor_ != shownLiveMeshColor_) {
            view_->setMesh(pipeline_->liveMesh(), useColor_);
            shownLiveMeshRev_ = rev;
            shownLiveMeshColor_ = useColor_;
            showingLiveMesh_ = true;
        }
    } else {
        if (showFused) {
            document_.withCloud([&](const pointcloud::PointCloud& c) { view_->setPointCloud(c); });
        } else {
            view_->setPointCloud(controller_->lastCloud());
        }
        showingLiveMesh_ = false;
    }

    std::vector<float> depths;
    auto sampleDepths = [&depths](const pointcloud::PointCloud& c) {
        const size_t ptCount = c.points.size();
        if (ptCount == 0) {
            return;
        }
        const size_t maxSamples = 500;
        const size_t stride = std::max<size_t>(1, ptCount / maxSamples);
        depths.reserve(std::min(ptCount, maxSamples));
        for (size_t i = 0; i < ptCount; i += stride) {
            depths.push_back(c.points[i].z * 1000.0f);
        }
    };
    if (!docEmpty) {
        document_.withCloud(sampleDepths);
    } else {
        sampleDepths(controller_->lastCloud());
    }
    distanceScale_->setDepthSamplesMm(depths);
}

void MainWindow::updateStageAvailability() {
    const bool hasCloud = !document_.cloudEmpty();
    const bool hasMesh = !document_.meshEmpty();
    const bool hasTexture = hasMesh && document_.meshHasColors();
    if (stageRawBtn_) {
        stageRawBtn_->setEnabled(hasCloud);
    }
    if (stagePointsBtn_) {
        stagePointsBtn_->setEnabled(hasCloud);
    }
    if (stageMeshBtn_) {
        stageMeshBtn_->setEnabled(hasMesh);
    }
    if (stageTexBtn_) {
        stageTexBtn_->setEnabled(hasTexture);
    }

    // Fall back to an available stage if the current one no longer exists.
    if ((modelStage_ == ModelStage::Textured && !hasTexture) ||
        (modelStage_ == ModelStage::Meshed && !hasMesh)) {
        modelStage_ = hasCloud ? ModelStage::PointClouds : ModelStage::Raw;
    }
}

void MainWindow::applyModelStage(ModelStage stage) {
    if (!editView_) {
        return;
    }
    modelStage_ = stage;
    if (stageRawBtn_) {
        stageRawBtn_->setChecked(stage == ModelStage::Raw);
    }
    if (stagePointsBtn_) {
        stagePointsBtn_->setChecked(stage == ModelStage::PointClouds);
    }
    if (stageMeshBtn_) {
        stageMeshBtn_->setChecked(stage == ModelStage::Meshed);
    }
    if (stageTexBtn_) {
        stageTexBtn_->setChecked(stage == ModelStage::Textured);
    }

    // Selection editing only makes sense on the point cloud stages.
    const bool cloudStage =
        stage == ModelStage::Raw || stage == ModelStage::PointClouds;
    if (editSelectBar_) {
        editSelectBar_->setEnabled(cloudStage);
    }
    if (!cloudStage) {
        editView_->setTool(PointCloudView::Tool::Orbit);
    }

    switch (stage) {
    case ModelStage::Raw:
        editView_->setColorEnabled(false);
        document_.withCloud([&](const pointcloud::PointCloud& c) { editView_->setPointCloud(c); });
        break;
    case ModelStage::PointClouds:
        editView_->setColorEnabled(useColor_);
        document_.withCloud([&](const pointcloud::PointCloud& c) { editView_->setPointCloud(c); });
        break;
    case ModelStage::Meshed:
        document_.withMesh([&](const geometry::TriangleMesh& m) { editView_->setMesh(m, false); });
        break;
    case ModelStage::Textured:
        document_.withMesh([&](const geometry::TriangleMesh& m) { editView_->setMesh(m, true); });
        break;
    }
}

void MainWindow::refreshEditView() {
    if (!editView_) {
        return;
    }
    updateStageAvailability();
    applyModelStage(modelStage_);
    if (editInfoLabel_) {
        size_t triangles = 0;
        document_.withMesh([&](const geometry::TriangleMesh& m) { triangles = m.triangleCount(); });
        editInfoLabel_->setText(
            tr("%1 points, %2 triangles, revision %3%4")
                .arg(document_.cloudSize())
                .arg(triangles)
                .arg(document_.revision())
                .arg(document_.isMetric() ? QString() : tr(" (non-metric)")));
    }
}

void MainWindow::onRawSensorFrame(uint32_t width,
                                  uint32_t height,
                                  uint32_t nonZeroSamples,
                                  uint64_t,
                                  const QVector<uint16_t>& samples) {
    // Don't overwrite the metric connection banner when this is a depth preview
    // from the full-field decode path.
    if (!controller_ || !controller_->lastCloudMetric()) {
        connLabel_->setText(
            tr("● POP4 raw Y16 %1×%2 — %3 active samples (preview only, not mm)")
                .arg(width)
                .arg(height)
                .arg(nonZeroSamples));
        connLabel_->setStyleSheet("color:#e0a83e;");
    }

    if (depthPanel_ && width > 0 && height > 0 &&
        samples.size() == static_cast<int>(width * height)) {
        QImage preview;

        // Prefer left-view IR albedo from the metric decoder (single 320×256
        // tile). Never draw the packed 640×2560 montage (~20 sub-images).
        const protocol::RawFrame* metric = nullptr;
        if (controller_ && controller_->lastCloudMetric()) {
            const auto& md = controller_->lastMetricDepth();
            if (!md.rgb.empty() && md.width > 0 && md.height > 0 &&
                md.rgb.size() == static_cast<size_t>(md.width) * md.height * 3) {
                metric = &md;
            }
        }

        if (depthPanel_->showDepthMap() && metric && !metric->depthMm.empty()) {
            preview = buildLeftDepthColorMapPreview(
                metric->depthMm.data(), metric->width, metric->height,
                activeBandEdgesMm_);
        } else if (metric) {
            preview = buildLeftIrPreview(
                metric->rgb.data(), metric->depthMm.data(), metric->width,
                metric->height, /*fromMetricRgb=*/true);
        } else {
            preview = buildLeftIrPreview(nullptr, samples.constData(), width,
                                         height, /*fromMetricRgb=*/false);
        }

        if (!preview.isNull()) {
            if (pop4DebugPreview()) {
                static int dumpN = 0;
                if ((dumpN++ % 30) == 0) {
                    preview.save(QStringLiteral("/tmp/all3dscan_depth_preview.png"));
                }
            }
            depthPanel_->setPreview(preview);
        }
    }

    updateFrameTelemetry();
}

void MainWindow::onRgbPreview(const QByteArray& jpegBytes) {
    if (!rgbPanel_ || jpegBytes.isEmpty()) {
        return;
    }
    QImage image;
    if (!image.loadFromData(jpegBytes, "JPEG")) {
        return;
    }
    if (image.width() > 320) {
        rgbPanel_->setPreview(image.scaledToWidth(320, Qt::FastTransformation));
    } else {
        rgbPanel_->setPreview(image);
    }
}

void MainWindow::updateFrameTelemetry() {
    ++frameCount_;
    ++renderFrames_;
    framesLabel_->setText(tr("Captured Frames: %1").arg(renderFrames_));
    const qint64 elapsed = fpsTimer_.elapsed();
    if (elapsed >= 500) {
        const double fps = frameCount_ * 1000.0 / static_cast<double>(elapsed);
        fpsLabel_->setText(tr("Frame Rate: %1 FPS").arg(fps, 0, 'f', 1));
        frameCount_ = 0;
        fpsTimer_.restart();
    }
}

void MainWindow::keyPressEvent(QKeyEvent* event) {
    // Handle `?` on both dedicated keys and US-layout (Shift+`/`).
    const bool isQuestion = (event->key() == Qt::Key_Question) ||
        (event->key() == Qt::Key_Slash &&
         (event->modifiers() & Qt::ShiftModifier));
    if (isQuestion) {
        shortcutOverlayVisible_ = !shortcutOverlayVisible_;
        if (shortcutHint_) {
            shortcutHint_->setVisible(shortcutOverlayVisible_);
            if (shortcutOverlayVisible_) {
                shortcutHint_->raise();
                shortcutHint_->resize(360, shortcutHint_->sizeHint().height());
                shortcutHint_->move(
                    (width() - shortcutHint_->width()) / 2,
                    (height() - shortcutHint_->height()) / 2);
            }
        }
        return;
    }
    if (completing_) {
        QMainWindow::keyPressEvent(event); // no Space/Esc actions while meshing
        return;
    }
    if (event->key() == Qt::Key_Space) {
        if (connected_) {
            onScanToggle();
            return;
        }
    }
    if (event->key() == Qt::Key_Escape) {
        if (shortcutOverlayVisible_) {
            shortcutOverlayVisible_ = false;
            if (shortcutHint_) shortcutHint_->hide();
            return;
        }
        if (streaming_) {
            onCancel();
            return;
        }
    }
    QMainWindow::keyPressEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    // Reposition the shortcut overlay when the window is resized.
    if (shortcutHint_ && shortcutHint_->isVisible()) {
        shortcutHint_->resize(360, shortcutHint_->sizeHint().height());
        shortcutHint_->move(
            (width() - shortcutHint_->width()) / 2,
            (height() - shortcutHint_->height()) / 2);
    }
}

void MainWindow::onError(const QString& message) {
    connLabel_->setText(tr("● %1").arg(message));
    connLabel_->setStyleSheet("color:#e0a83e;");
}

void MainWindow::updateControlStates() {
    scanButton_->setEnabled(connected_);
    singleShotButton_->setEnabled(connected_);
    completeButton_->setEnabled(connected_);
    cancelButton_->setEnabled(connected_ && streaming_);
    if (openEditBtn_) {
        openEditBtn_->setEnabled(document_.isMetric() && !document_.cloudEmpty());
    }
    if (undoButton_) {
        undoButton_->setEnabled(document_.canUndo());
    }
    if (redoButton_) {
        redoButton_->setEnabled(document_.canRedo());
    }
    if (deleteSelBtn_ && editView_) {
        deleteSelBtn_->setEnabled(editView_->selectedCount() > 0 && document_.isMetric());
    }
}

} // namespace revoscan::app
