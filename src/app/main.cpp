#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QIcon>
#include <QTimer>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include <string_view>

#include "MainWindow.h"
#include "bridge/RustBridge.h"
#include "decoder/Pop4DepthDecoder.h"
#include "document/ScanDocument.h"
#include "io/GeometryExport.h"
#include "io/PointCloudExport.h"
#include "pointcloud/PointCloud.h"
#include "processing/Registration.h"
#include "protocol/RevopointPop4Protocol.h"
#include "usb/UsbDevice.h"
#include "processing/ScanPipeline.h"
#include "util/ResourcePaths.h"
#include "protocol/ScannerProtocol.h"

namespace {

int probePop4() {
    auto scanner = revoscan::protocol::makeRevopointPop4Protocol();
    std::cout << "Backend: " << scanner->backendName() << '\n';
    const auto cal = scanner->calibration();
    std::cout << "Calibration source: " << cal.source
              << " stereo=" << (cal.hasStereoCalibration() ? "yes" : "no")
              << " baselineMm=" << cal.baselineMm << '\n';
    if (!scanner->connect()) {
        std::cerr << "Connect failed: " << scanner->lastError() << '\n';
        return 2;
    }
    {
        // connect() loads the factory calibration from the scanner (or cache).
        const auto connected = scanner->calibration();
        std::cout << "After connect: " << scanner->backendName() << ", calibration "
                  << connected.source << " baselineMm=" << connected.baselineMm
                  << (connected.serial.empty() ? "" : " device=" + connected.serial) << '\n';
    }
    if (!scanner->startStream()) {
        std::cerr << "Stream start failed: " << scanner->lastError() << '\n';
        scanner->disconnect();
        return 3;
    }

    std::optional<revoscan::protocol::RawFrame> frame;
    for (int attempt = 1; attempt <= 3 && !frame; ++attempt) {
        frame = scanner->readFrame(5000);
        if (!frame) {
            std::cerr << "Capture attempt " << attempt
                      << "/3: " << scanner->lastError() << '\n';
        }
    }
    if (frame && std::getenv("ALL3DSCAN_POP4_DUMP_DIR")) {
        // Keep streaming so the protocol dumps a short burst for offline work.
        int burst = 30;
        if (const char* m = std::getenv("ALL3DSCAN_POP4_DUMP_MAX")) {
            burst = std::max(1, std::atoi(m));
        }
        for (int i = 1; i < burst; ++i) {
            (void)scanner->readFrame(5000);
        }
    }
    scanner->stopStream();
    scanner->disconnect();
    if (!frame) {
        std::cerr << "Capture failed: " << scanner->lastError() << '\n';
        return 4;
    }
    const auto nonZero = std::count_if(
        frame->depthMm.begin(), frame->depthMm.end(),
        [](uint16_t value) { return value != 0; });
    std::cout << "Captured " << frame->width << 'x' << frame->height
              << " encoding="
              << (frame->encoding == revoscan::protocol::DepthEncoding::MetricMillimeters
                      ? "MetricMillimeters"
                      : "VendorRawY16")
              << ", samples=" << frame->depthMm.size()
              << ", nonzero=" << nonZero << '\n';

    if (frame->encoding == revoscan::protocol::DepthEncoding::MetricMillimeters) {
        std::cout << "Metric decoding: ok (" << scanner->calibration().source << ")\n";
        return 0;
    }

    if (frame->width == 640 && frame->height == 2560) {
        const size_t plane = 640ull * 512ull;
        for (int p = 0; p < 5; ++p) {
            const auto begin = frame->depthMm.begin() + static_cast<std::ptrdiff_t>(p) * plane;
            const auto end = begin + static_cast<std::ptrdiff_t>(plane);
            const auto planeNz = std::count_if(begin, end, [](uint16_t v) { return v != 0; });
            std::cout << "  plane[" << p << "] nonzero=" << planeNz << '\n';
        }
        std::cout << "Metric decoding: packed frame captured but decode failed: "
                  << scanner->lastError() << '\n';
    } else {
        std::cout << "Metric decoding: fallback "
                  << frame->width << 'x' << frame->height
                  << " (need 640x2560 for phase decode); "
                  << scanner->lastError() << '\n';
    }
    return 0;
}

// ALL3DSCAN_CAMPARAM=<camparam.yaml>: use the scanner's factory calibration
// (e.g. copied with --read-device-file) instead of the approximate model.
void applyCamparamEnv(revoscan::decoder::Pop4DepthDecoder& decoder) {
    if (const char* path = std::getenv("ALL3DSCAN_CAMPARAM")) {
        std::string error;
        if (const auto rig = revoscan::decoder::loadCamparamFile(path, &error)) {
            decoder.setStereoRig(*rig);
            std::cerr << "factory calibration " << rig->deviceId << " (baseline "
                      << rig->baselineMm() << " mm)\n";
        } else {
            std::cerr << "ALL3DSCAN_CAMPARAM ignored: " << error << '\n';
        }
    }
}

// Offline decoder check on frames written by ALL3DSCAN_POP4_DUMP_DIR:
//   all3dscan --decode-dump frame.y16 [disparity.pgm]
int decodeDump(const char* path, const char* pgmOut) {
    std::ifstream in(path, std::ios::binary);
    std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (bytes.size() != 640ull * 2560ull * 2ull) {
        std::cerr << path << ": expected a 640x2560 Y16 dump, got " << bytes.size() << " bytes\n";
        return 2;
    }
    revoscan::protocol::RawFrame raw;
    raw.width = 640;
    raw.height = 2560;
    raw.encoding = revoscan::protocol::DepthEncoding::VendorRawY16;
    raw.depthMm.resize(bytes.size() / 2);
    for (size_t i = 0; i < raw.depthMm.size(); ++i) {
        raw.depthMm[i] = static_cast<uint16_t>(static_cast<uint8_t>(bytes[i * 2])) |
                         static_cast<uint16_t>(static_cast<uint8_t>(bytes[i * 2 + 1]) << 8);
    }
    revoscan::decoder::Pop4DepthDecoder decoder;
    decoder.setCalibration(revoscan::decoder::makeApproximatePop4FullFieldCalibration());
    applyCamparamEnv(decoder);
    decoder.setDepthRange(150.0f, 650.0f);
    // Decode several times: the first call pays one-off costs (CUDA context,
    // allocations); report the best steady-state time.
    std::optional<revoscan::protocol::RawFrame> metric;
    double ms = 0.0;
    double firstMs = 0.0;
    for (int run = 0; run < 5; ++run) {
        const auto t0 = std::chrono::steady_clock::now();
        metric = decoder.decode(raw);
        const double t = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        firstMs = run == 0 ? t : firstMs;
        ms = run == 0 ? t : std::min(ms, t);
    }
    const auto& stats = decoder.lastStats();
    std::vector<float> valid;
    for (float d : decoder.lastDisparity()) {
        if (d >= 0.0f) {
            valid.push_back(d);
        }
    }
    std::sort(valid.begin(), valid.end());
    std::cout << "decode " << (metric ? "ok" : "failed") << " [" << stats.backend << "] in " << ms
              << " ms (first call " << firstMs << " ms): valid="
              << 100.0 * stats.validFraction() << "% (" << stats.validDepth << ")";
    if (!valid.empty()) {
        std::cout << " disparity p10/p50/p90=" << valid[valid.size() / 10] << '/'
                  << valid[valid.size() / 2] << '/' << valid[valid.size() * 9 / 10];
    }
    std::cout << (metric ? "" : " — " + decoder.lastError()) << '\n';
    if (pgmOut && std::string_view(pgmOut).size() > 4 &&
        std::string_view(pgmOut).substr(std::string_view(pgmOut).size() - 4) == ".f32") {
        // Raw float32 disparity (320×512, <0 = invalid) for sub-pixel analysis.
        std::ofstream f32(pgmOut, std::ios::binary);
        const auto& disp = decoder.lastDisparity();
        f32.write(reinterpret_cast<const char*>(disp.data()),
                  static_cast<std::streamsize>(disp.size() * sizeof(float)));
    } else if (pgmOut) {
        std::ofstream pgm(pgmOut, std::ios::binary);
        pgm << "P5\n320 512\n255\n";
        for (float d : decoder.lastDisparity()) {
            pgm.put(static_cast<char>(d < 0.0f ? 0 : std::clamp(static_cast<int>(d * 4.0f), 1, 255)));
        }
    }
    return metric ? 0 : 1;
}

// Offline tracking check: feeds a directory of ALL3DSCAN_POP4_DUMP_DIR frames
// through the same chain as a live metric scan (decoder -> cloud -> object
// isolation -> ScanPipeline tracking/fusion), one frame at a time, and logs
// tracking per frame. Optional PLY of the fused model.
//   all3dscan --replay-dumps <dir> [fused.ply]
int replayDumps(int argc, char** argv, const char* dir, const char* plyOut) {
    QCoreApplication app(argc, argv);
    QDir d(QString::fromLocal8Bit(dir));
    const QStringList files = d.entryList({"*.y16"}, QDir::Files, QDir::Name);
    if (files.isEmpty()) {
        std::cerr << dir << ": no .y16 dumps\n";
        return 2;
    }
    revoscan::decoder::Pop4DepthDecoder decoder;
    decoder.setCalibration(revoscan::decoder::makeApproximatePop4FullFieldCalibration());
    applyCamparamEnv(decoder);
    const auto cal = decoder.factoryCalibration().value_or(
        revoscan::decoder::makeApproximatePop4FullFieldCalibration());
    decoder.setDepthRange(250.0f, 500.0f); // the app's default 250-500 mm preset

    revoscan::document::ScanDocument document;
    revoscan::processing::ScanPipeline pipeline(&document);
    pipeline.setDepthClip(0.25f, 0.50f);
    pipeline.start();
    int state = 0;
    double fitness = 0.0;
    int modelPoints = 0;
    int signalTracked = 0;
    int signalLost = 0;
    QObject::connect(&pipeline, &revoscan::processing::ScanPipeline::trackingChanged,
                     [&](int s) {
                         state = s;
                         signalTracked += s == 1;
                         signalLost += s == 2;
                     });
    // ALL3DSCAN_REPLAY_REALTIME=1: feed frames at the camera rate (~15 fps)
    // without waiting, as live; the pipeline drops what it cannot keep up with.
    const bool realtime = std::getenv("ALL3DSCAN_REPLAY_REALTIME") != nullptr;
    const auto replayStart = std::chrono::steady_clock::now();
    QObject::connect(&pipeline, &revoscan::processing::ScanPipeline::statsUpdated,
                     [&](double f, int pts, int) { fitness = f; modelPoints = pts; });
    bool relocalized = false;
    int relocalizations = 0;
    QObject::connect(&pipeline, &revoscan::processing::ScanPipeline::relocalized,
                     [&] { relocalized = true; ++relocalizations; });

    int tracked = 0;
    int lost = 0;
    revoscan::pointcloud::PointCloud firstFrame;
    for (int i = 0; i < files.size(); ++i) {
        std::ifstream in(d.filePath(files[i]).toStdString(), std::ios::binary);
        std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        if (bytes.size() != 640ull * 2560ull * 2ull) {
            continue;
        }
        revoscan::protocol::RawFrame raw;
        raw.width = 640;
        raw.height = 2560;
        raw.encoding = revoscan::protocol::DepthEncoding::VendorRawY16;
        raw.timestampUs = static_cast<uint64_t>(i) * 66000;
        raw.depthMm.resize(bytes.size() / 2);
        std::memcpy(raw.depthMm.data(), bytes.data(), bytes.size());
        const auto metric = decoder.decode(raw);
        if (!metric) {
            std::cout << i << " decode-failed " << decoder.lastError() << '\n';
            continue;
        }
        auto cloud = revoscan::pointcloud::fromDepthFrame(*metric, cal.depth);
        cloud = revoscan::pointcloud::isolateForegroundObject(cloud, {});
        if (i == 0) {
            firstFrame = cloud; // reference surface for the orbit accuracy check
        }
        // ALL3DSCAN_REPLAY_ORBIT_DEG=<deg/frame>: simulate the scanner orbiting
        // the object (rotation about the vertical axis through the cloud
        // centroid) so tracking can be checked against a known motion.
        // ALL3DSCAN_REPLAY_BLACKOUT=a,b,r: frames a..b arrive empty (scanner
        // pointed away); afterwards the orbit resumes r frames back, i.e. the
        // scanner returns to an area already scanned (relocalization test).
        int orbitIndex = i;
        if (const char* blackout = std::getenv("ALL3DSCAN_REPLAY_BLACKOUT")) {
            int a = -1, b = -1, r = 0;
            if (std::sscanf(blackout, "%d,%d,%d", &a, &b, &r) == 3) {
                if (i >= a && i <= b) {
                    cloud.points.clear();
                } else if (i > b) {
                    orbitIndex = i - r;
                }
            }
        }
        if (const char* orbit = std::getenv("ALL3DSCAN_REPLAY_ORBIT_DEG")) {
            static double cx = 0, cz = 0;
            if (i == 0 && !cloud.points.empty()) {
                for (const auto& p : cloud.points) { cx += p.x; cz += p.z; }
                cx /= cloud.points.size();
                cz /= cloud.points.size();
            }
            const double a = std::atof(orbit) * orbitIndex * 3.14159265358979 / 180.0;
            const double c = std::cos(a), sn = std::sin(a);
            for (auto& p : cloud.points) {
                const double x = p.x - cx, z = p.z - cz;
                p.x = static_cast<float>(c * x + sn * z + cx);
                p.z = static_cast<float>(-sn * x + c * z + cz);
            }
        }
        const uint64_t before = pipeline.framesProcessed();
        const auto submitted = std::chrono::steady_clock::now();
        state = -1;
        relocalized = false;
        pipeline.submitMetricFrame(cloud, *metric, cal.depth, raw.timestampUs);
        if (realtime) {
            const auto due = replayStart + std::chrono::milliseconds(66 * (i + 1));
            while (std::chrono::steady_clock::now() < due) {
                app.processEvents(QEventLoop::AllEvents, 2);
            }
            continue;
        }
        while (pipeline.framesProcessed() == before) {
            app.processEvents(QEventLoop::AllEvents, 5);
        }
        app.processEvents(); // deliver this frame's signals
        tracked += state == static_cast<int>(revoscan::processing::TrackingState::Tracking);
        lost += state == static_cast<int>(revoscan::processing::TrackingState::Lost);
        std::cout << i << ' '
                  << (state == 1 ? (relocalized ? "RELOC" : "TRACK") : state == 2 ? "LOST " : "seed ")
                  << " fit=" << fitness << " pts=" << cloud.points.size()
                  << " model=" << modelPoints << " ms="
                  << std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - submitted).count()
                  << std::endl;
    }
    if (realtime) {
        // Drain what is still queued.
        uint64_t seen = pipeline.framesProcessed();
        auto quietSince = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - quietSince < std::chrono::milliseconds(500)) {
            app.processEvents(QEventLoop::AllEvents, 5);
            if (pipeline.framesProcessed() != seen) {
                seen = pipeline.framesProcessed();
                quietSince = std::chrono::steady_clock::now();
            }
        }
        std::cout << "realtime: processed=" << seen << " of " << files.size()
                  << " tracked=" << signalTracked << " lost=" << signalLost << " | ";
        tracked = signalTracked;
        lost = signalLost;
    }
    std::cout << "summary: frames=" << files.size() << " tracked=" << tracked
              << " lost=" << lost << " relocalized=" << relocalizations
              << " model=" << document.cloud().points.size();
    if (std::getenv("ALL3DSCAN_REPLAY_ORBIT_DEG")) {
        // Every frame is the same surface moved by a known orbit, so a correctly
        // registered model lies on frame 0's surface: score how much of it does.
        revoscan::processing::IcpOptions probe;
        probe.maxIterations = 0;
        probe.maxCorrespondenceDistM = 0.002f;
        const auto on2mm = revoscan::processing::icpPointToPoint(document.cloud(), firstFrame, {}, probe);
        probe.maxCorrespondenceDistM = 0.010f;
        const auto on10mm = revoscan::processing::icpPointToPoint(document.cloud(), firstFrame, {}, probe);
        std::cout << " | model on true surface: " << 100.0 * on2mm.fitness << "% within 2 mm, "
                  << 100.0 * on10mm.fitness << "% within 10 mm";
    }
    std::cout << '\n';
    if (plyOut && !revoscan::io::writePly(plyOut, document.cloud())) {
        std::cerr << "failed to write " << plyOut << '\n';
    }
    // What the scan view shows with Live Mesh on.
    if (const char* liveOut = std::getenv("ALL3DSCAN_REPLAY_LIVEMESH")) {
        const auto live = pipeline.liveMesh();
        std::cout << "live mesh: " << live.vertices.size() << " vertices, "
                  << live.triangleCount() << " triangles\n";
        if (!live.empty() && !revoscan::io::writePlyMesh(liveOut, live).ok) {
            std::cerr << "failed to write " << liveOut << '\n';
        }
    }
    // Same as pressing Complete: volumetric mesh from the tracked keyframes.
    if (const char* meshOut = std::getenv("ALL3DSCAN_REPLAY_MESH")) {
        const auto t0 = std::chrono::steady_clock::now();
        std::string error;
        const bool ok = pipeline.completeAndMesh("", "python3", &error);
        const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        const auto& mesh = document.mesh();
        std::cout << "complete: " << (ok ? "ok" : "failed") << " in " << s << " s, "
                  << mesh.vertices.size() << " vertices, " << mesh.triangleCount()
                  << " triangles" << (error.empty() ? "" : " (" + error + ")") << '\n';
        if (ok && !revoscan::io::writePlyMesh(meshOut, mesh).ok) {
            std::cerr << "failed to write " << meshOut << '\n';
        }
    }
    pipeline.stop(); // after Complete: stopping drops the worker's keyframes
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::cout << "All3DScan Rust Core v" << revoscan::bridge::RustBridge::getVersion() << " initialized.\n";

    for (int i = 1; i < argc; ++i) {
        if (std::string_view(argv[i]) == "--probe-pop4") {
            return probePop4();
        }
        if (std::string_view(argv[i]) == "--usb-reset") {
            // Port-reset the scanner: it reboots and re-enumerates (~40 s).
            // Clears the state where vendor controls fail with LIBUSB_ERROR_IO.
            revoscan::usb::UsbDevice dev;
            if (!dev.open(revoscan::protocol::pop4::kVendorId, revoscan::protocol::pop4::kProductId)) {
                std::cerr << "scanner not found or permission denied\n";
                return 2;
            }
            const int rc = dev.resetDevice();
            std::cout << "reset requested (rc=" << rc << "); the scanner reboots in ~40 s\n";
            return 0;
        }
        if (std::string_view(argv[i]) == "--read-device-file" && i + 2 < argc) {
            // Read-only copy of a file from the scanner (e.g. the factory
            // calibration /data/camparam/camparam.yaml).
            std::string error;
            const auto data = revoscan::protocol::pop4::readDeviceFile(argv[i + 1], &error);
            if (!data) {
                std::cerr << "read " << argv[i + 1] << " failed: " << error << '\n';
                return 2;
            }
            std::ofstream out(argv[i + 2], std::ios::binary);
            out.write(reinterpret_cast<const char*>(data->data()),
                      static_cast<std::streamsize>(data->size()));
            std::cout << "read " << data->size() << " bytes from " << argv[i + 1] << " -> "
                      << argv[i + 2] << (error.empty() ? "" : " [" + error + "]") << '\n';
            return out ? 0 : 3;
        }
        if (std::string_view(argv[i]) == "--replay-dumps" && i + 1 < argc) {
            return replayDumps(argc, argv, argv[i + 1], i + 2 < argc ? argv[i + 2] : nullptr);
        }
        if (std::string_view(argv[i]) == "--decode-dump" && i + 1 < argc) {
            return decodeDump(argv[i + 1], i + 2 < argc ? argv[i + 2] : nullptr);
        }
    }

    QApplication app(argc, argv);
    QApplication::setApplicationName("All3DScan");
    QApplication::setApplicationVersion("Alpha-v20261013-2327");
    QApplication::setOrganizationName("All3DScan");
    QApplication::setDesktopFileName("io.github.all3dscan.All3DScan");
    // App logo for windows, dialogs and the taskbar: the installed icon theme
    // entry when present, else the PNG shipped next to the binary/sources.
    {
        QIcon icon = QIcon::fromTheme(QStringLiteral("io.github.all3dscan.All3DScan"));
        if (icon.isNull()) {
            if (const auto path = revoscan::util::findAppIcon()) {
                icon = QIcon(QString::fromStdString(*path));
            }
        }
        if (!icon.isNull()) {
            QApplication::setWindowIcon(icon);
        }
    }

    revoscan::app::MainWindow window;
    window.show();
    // --screenshot <file.png>: render the main window itself (QWidget::grab,
    // nothing else on screen) after startup and exit. For docs / UI checks.
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string_view(argv[i]) == "--screenshot") {
            const QString path = QString::fromLocal8Bit(argv[i + 1]);
            QTimer::singleShot(2000, &window, [&window, path] {
                window.grab().save(path);
                QApplication::quit();
            });
        }
    }
    return app.exec();
}
