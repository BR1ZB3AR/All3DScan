#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QIcon>
#include <QTimer>
#include <QPushButton>
#include <QComboBox>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <vector>
#include <string_view>

#include "MainWindow.h"
#include "ScannerController.h"
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

// Switches each built-in light while streaming and records what the cameras
// see: raw frames (per-step index ranges in steps.txt) and one colour JPEG
// per step. Ends with the laser and fill light off.
//   all3dscan --pop4-lights-test <outdir>
int pop4LightsTest(const char* outDir) {
    using revoscan::protocol::Light;
    std::filesystem::create_directories(outDir);
    ::setenv("ALL3DSCAN_POP4_DUMP_DIR", outDir, 1);
    ::setenv("ALL3DSCAN_POP4_DUMP_MAX", "2000", 1);
    auto scanner = revoscan::protocol::makeRevopointPop4Protocol();
    if (!scanner->connect() || !scanner->startStream()) {
        std::cerr << "POP4 connect/stream failed: " << scanner->lastError() << '\n';
        return 2;
    }
    std::ofstream steps(std::string(outDir) + "/steps.txt");
    int frameIndex = 0;
    auto run = [&](const std::string& name, const std::function<bool()>& action, int frames) {
        const bool ok = action();
        std::cout << name << ": " << (ok ? "ok" : "FAILED " + scanner->lastError()) << std::endl;
        const int first = frameIndex;
        for (int i = 0; i < frames; ++i) {
            if (scanner->readFrame(3000)) {
                ++frameIndex;
            }
        }
        steps << name << ' ' << first << ' ' << frameIndex - 1 << ' ' << (ok ? "ok" : "failed") << '\n';
        steps.flush();
        // The colour node queues old frames: drain it so the image is current.
        std::optional<std::vector<uint8_t>> latest;
        for (int i = 0; i < 6; ++i) {
            if (auto jpeg = scanner->readRgbJpegPreview(i == 0 ? 500 : 80)) {
                latest = std::move(jpeg);
            }
        }
        if (latest) {
            std::ofstream(std::string(outDir) + "/" + name + ".jpg", std::ios::binary)
                .write(reinterpret_cast<const char*>(latest->data()),
                       static_cast<std::streamsize>(latest->size()));
        }
    };
    auto level = [&](const char* name, Light light, int v, int frames = 16) {
        run(name, [&, light, v] { return scanner->setLightLevel(light, v); }, frames);
    };
    run("baseline", [] { return true; }, 30);
    // Projector ("laser", 0xb04) and its brightness (0x48d).
    run("proj_on", [&] { return scanner->setLight(Light::Projector, true); }, 16);
    level("proj_lum_255", Light::Projector, 255);
    level("proj_lum_128", Light::Projector, 128);
    level("proj_lum_40", Light::Projector, 40);
    level("proj_lum_255b", Light::Projector, 255);
    run("proj_off", [&] { return scanner->setLight(Light::Projector, false); }, 16);
    run("proj_on2", [&] { return scanner->setLight(Light::Projector, true); }, 16);
    // IR emitter (0xb01) and its brightness (0xb07) — candidate for the
    // speckle pattern the depth decode actually matches on.
    level("ir_lum_255", Light::IrLed, 255);
    level("ir_lum_128", Light::IrLed, 128);
    level("ir_lum_40", Light::IrLed, 40);
    level("ir_lum_255b", Light::IrLed, 255);
    run("ir_off", [&] { return scanner->setLight(Light::IrLed, false); }, 16);
    run("ir_on", [&] { return scanner->setLight(Light::IrLed, true); }, 16);
    // White/colour LED for the texture camera (0xb03/0xb02) + level (0xb12).
    run("fill_on", [&] { return scanner->setLight(Light::FillLight, true); }, 16);
    level("fill_lum_60", Light::FillLight, 60);
    level("fill_lum_255", Light::FillLight, 255);
    run("fill_off", [&] { return scanner->setLight(Light::FillLight, false); }, 16);
    scanner->stopStream();
    scanner->disconnect();
    return 0;
}

// Switches the POP4 into blue-laser speckle mode and back, recording what
// both cameras see in each state. Always restores the default IR mode.
//   all3dscan --pop4-mode-test <outdir>
int pop4ModeTest(const char* outDir) {
    using revoscan::protocol::ScanMode;
    std::filesystem::create_directories(outDir);
    ::setenv("ALL3DSCAN_POP4_DUMP_DIR", outDir, 1);
    ::setenv("ALL3DSCAN_POP4_DUMP_MAX", "2000", 1);
    auto scanner = revoscan::protocol::makeRevopointPop4Protocol();
    if (!scanner->connect() || !scanner->startStream()) {
        std::cerr << "POP4 connect/stream failed: " << scanner->lastError() << '\n';
        return 2;
    }
    std::ofstream steps(std::string(outDir) + "/steps.txt");
    int frameIndex = 0;
    auto capture = [&](const std::string& name, int frames) {
        const int first = frameIndex;
        for (int i = 0; i < frames; ++i) {
            if (scanner->readFrame(3000)) {
                ++frameIndex;
            }
        }
        steps << name << ' ' << first << ' ' << frameIndex - 1 << " ok\n";
        steps.flush();
        std::optional<std::vector<uint8_t>> latest;
        for (int i = 0; i < 6; ++i) {
            if (auto jpeg = scanner->readRgbJpegPreview(i == 0 ? 500 : 80)) {
                latest = std::move(jpeg);
            }
        }
        if (latest) {
            std::ofstream(std::string(outDir) + "/" + name + ".jpg", std::ios::binary)
                .write(reinterpret_cast<const char*>(latest->data()),
                       static_cast<std::streamsize>(latest->size()));
        }
        std::cout << name << ": " << (frameIndex - first) << " frames" << std::endl;
    };
    capture("ir_mode", 25);
    const bool entered = scanner->setScanMode(ScanMode::BlueLaser);
    std::cout << "blue laser on: " << (entered ? "ok" : "FAILED " + scanner->lastError())
              << std::endl;
    capture("blue_1st", 25);
    // Pick it a second time without leaving the mode: a stale idea of the
    // current mode used to make this a silent no-op.
    const bool again = scanner->setScanMode(ScanMode::BlueLaser);
    std::cout << "blue laser on again: " << (again ? "ok" : "FAILED " + scanner->lastError())
              << std::endl;
    capture("blue_2nd", 25);
    // Always go back, whatever happened above.
    const bool restored = scanner->setScanMode(ScanMode::StructuredLight);
    std::cout << "restore IR mode: " << (restored ? "ok" : "FAILED " + scanner->lastError())
              << std::endl;
    capture("ir_mode_restored", 25);
    // And straight back into blue laser from infrared, as a user would.
    const bool third = scanner->setScanMode(ScanMode::BlueLaser);
    std::cout << "blue laser on after restore: "
              << (third ? "ok" : "FAILED " + scanner->lastError()) << std::endl;
    capture("blue_3rd", 25);
    scanner->setScanMode(ScanMode::StructuredLight);
    capture("final_ir", 20);
    scanner->stopStream();
    scanner->disconnect();
    return entered && again && restored && third ? 0 : 3;
}

// Exercises the exact path the GUI uses — ScannerController with frame
// polling running — rather than the protocol on its own, so a mode switch
// that races the poll timer shows up here instead of only in the app.
//   all3dscan --pop4-mode-cycle
int pop4ModeCycle() {
    int argc = 0;
    QApplication app(argc, nullptr); // the controller needs a Qt event loop
    revoscan::app::ScannerController controller(revoscan::protocol::makeRevopointPop4Protocol());
    int frames = 0;
    QObject::connect(&controller, &revoscan::app::ScannerController::frameReady,
                     [&] { ++frames; });
    QObject::connect(&controller, &revoscan::app::ScannerController::errorOccurred,
                     [](const QString& e) { std::cout << "  error: " << e.toStdString() << '\n'; });
    if (!controller.connectDevice()) {
        std::cerr << "connect failed\n";
        return 2;
    }
    if (!controller.startScan()) {
        std::cerr << "startScan failed\n";
        return 3;
    }
    auto pump = [&](int ms) {
        const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
        while (std::chrono::steady_clock::now() < until) {
            app.processEvents(QEventLoop::AllEvents, 10);
        }
    };
    auto switchTo = [&](revoscan::protocol::ScanMode mode, const char* name) {
        bool done = false;
        bool ok = false;
        QMetaObject::Connection c = QObject::connect(
            &controller, &revoscan::app::ScannerController::scanModeFinished,
            [&](bool r) { ok = r; done = true; });
        const int before = frames;
        const auto t0 = std::chrono::steady_clock::now();
        controller.setScanModeAsync(mode);
        while (!done && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(120)) {
            app.processEvents(QEventLoop::AllEvents, 10);
        }
        QObject::disconnect(c);
        const double secs =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::cout << name << ": " << (done ? (ok ? "ok" : "FAILED") : "TIMED OUT") << " in "
                  << secs << " s (frames during switch: " << frames - before << ")" << std::endl;
        pump(4000);
        std::cout << "    protocol state after: "
                  << static_cast<int>(controller.protocol()->state())
                  << " (2=streaming), decode: "
                  << controller.protocol()->depthDecodeStats().validFraction() * 100 << "% valid"
                  << std::endl;
        return done && ok;
    };
    pump(2000);
    std::cout << "streaming before switch: " << frames << " frames\n";
    const int a = frames;
    const bool toBlue = switchTo(revoscan::protocol::ScanMode::BlueLaser, "-> blue laser speckle");
    pump(4000);
    std::cout << "frames after entering speckle: " << frames - a << '\n';
    const int b = frames;
    const bool toIr = switchTo(revoscan::protocol::ScanMode::StructuredLight, "-> infrared");
    pump(4000);
    std::cout << "frames after restoring infrared: " << frames - b << '\n';
    controller.stopScan();
    controller.disconnectDevice();
    return toBlue && toIr ? 0 : 4;
}

// Sweeps the CS7 settings block's irLum field while streaming and records
// raw frames per step, to measure whether that field drives the always-on
// speckle emitter (the one the depth decoder matches on). Restores the
// default value at the end.
//   all3dscan --pop4-cs7-test <outdir>
int pop4Cs7Test(const char* outDir) {
    std::filesystem::create_directories(outDir);
    ::setenv("ALL3DSCAN_POP4_DUMP_DIR", outDir, 1);
    ::setenv("ALL3DSCAN_POP4_DUMP_MAX", "2000", 1);
    auto scanner = revoscan::protocol::makeRevopointPop4Protocol();
    if (!scanner->connect() || !scanner->startStream()) {
        std::cerr << "POP4 connect/stream failed: " << scanner->lastError() << '\n';
        return 2;
    }
    std::ofstream steps(std::string(outDir) + "/steps.txt");
    int frameIndex = 0;
    auto capture = [&](const std::string& name, int frames) {
        const int first = frameIndex;
        for (int i = 0; i < frames; ++i) {
            if (scanner->readFrame(3000)) {
                ++frameIndex;
            }
        }
        steps << name << ' ' << first << ' ' << frameIndex - 1 << " ok\n";
        steps.flush();
        std::cout << name << ": " << (frameIndex - first) << " frames" << std::endl;
    };
    auto setIr = [&](const char* name, int v) {
        const bool ok = scanner->setIlluminationPower(v);
        std::cout << name << " (irLum=" << v << "): "
                  << (ok ? "ok" : "FAILED " + scanner->lastError()) << std::endl;
        capture(name, 25);
    };
    capture("baseline_130", 25);
    setIr("ir0", 0);
    setIr("ir255", 255);
    setIr("restore_130", 130);
    scanner->stopStream();
    scanner->disconnect();
    return 0;
}

// Sweeps exposure while the blue laser is on and records frames per step, to
// find whether the dim blue-laser images are an exposure problem. Restores
// infrared at the end.
//   all3dscan --pop4-blue-exposure-test <outdir>
int pop4BlueExposureTest(const char* outDir) {
    using revoscan::protocol::ScanMode;
    std::filesystem::create_directories(outDir);
    ::setenv("ALL3DSCAN_POP4_DUMP_DIR", outDir, 1);
    ::setenv("ALL3DSCAN_POP4_DUMP_MAX", "4000", 1);
    auto scanner = revoscan::protocol::makeRevopointPop4Protocol();
    if (!scanner->connect() || !scanner->startStream()) {
        std::cerr << "POP4 connect/stream failed: " << scanner->lastError() << '\n';
        return 2;
    }
    std::ofstream steps(std::string(outDir) + "/steps.txt");
    int frameIndex = 0;
    auto capture = [&](const std::string& name, int frames) {
        const int first = frameIndex;
        for (int i = 0; i < frames; ++i) {
            if (scanner->readFrame(3000)) {
                ++frameIndex;
            }
        }
        steps << name << ' ' << first << ' ' << frameIndex - 1 << " ok\n";
        steps.flush();
        std::cout << name << ": " << (frameIndex - first) << " frames" << std::endl;
    };
    capture("ir_ref", 25);
    std::cout << "blue laser: "
              << (scanner->setScanMode(ScanMode::BlueLaser) ? "on" : "FAILED") << std::endl;
    capture("blue_default", 45);
    for (int exp : {3000, 4500}) {
        const bool ok = scanner->setExposure(exp);
        std::cout << "exposure " << exp << ": " << (ok ? "ok" : "FAILED " + scanner->lastError())
                  << std::endl;
        capture("blue_exp" + std::to_string(exp), 45);
    }
    std::cout << "restore: "
              << (scanner->setScanMode(ScanMode::StructuredLight) ? "ok" : "FAILED") << std::endl;
    capture("ir_restored", 20);
    scanner->stopStream();
    scanner->disconnect();
    return 0;
}

// Grabs one live frame with colour texturing applied and writes it as a
// coloured PLY, plus the colour camera frame it sampled, so the mapping can
// be checked by eye.
//   all3dscan --pop4-color-test <outdir>
int pop4ColorTest(const char* outDir) {
    std::filesystem::create_directories(outDir);
    auto scanner = revoscan::protocol::makeRevopointPop4Protocol();
    if (!scanner->connect() || !scanner->startStream()) {
        std::cerr << "POP4 connect/stream failed: " << scanner->lastError() << '\n';
        return 2;
    }
    std::optional<revoscan::protocol::RawFrame> frame;
    for (int i = 0; i < 40; ++i) { // warm-up, and let the colour camera settle
        if (auto f = scanner->readFrame(3000)) {
            frame = std::move(f);
        }
    }
    if (auto jpeg = scanner->readRgbJpegPreview(500)) {
        std::ofstream(std::string(outDir) + "/color_camera.jpg", std::ios::binary)
            .write(reinterpret_cast<const char*>(jpeg->data()),
                   static_cast<std::streamsize>(jpeg->size()));
    }
    const auto cal = scanner->calibration();
    scanner->stopStream();
    scanner->disconnect();
    if (!frame) {
        std::cerr << "no frame\n";
        return 3;
    }
    const bool hasRgb = frame->rgb.size() == frame->depthMm.size() * 3;
    std::cout << "frame " << frame->width << "x" << frame->height
              << " rgb=" << (hasRgb ? "yes" : "no") << '\n';
    size_t coloured = 0;
    size_t chromatic = 0;
    if (hasRgb) {
        for (size_t i = 0; i < frame->depthMm.size(); ++i) {
            const int r = frame->rgb[i * 3], g = frame->rgb[i * 3 + 1], b = frame->rgb[i * 3 + 2];
            if (r || g || b) {
                ++coloured;
            }
            if (std::abs(r - g) > 8 || std::abs(g - b) > 8 || std::abs(r - b) > 8) {
                ++chromatic; // grey would have r==g==b
            }
        }
        std::cout << "coloured pixels: " << coloured << ", genuinely chromatic: " << chromatic
                  << " (" << (100.0 * chromatic / std::max<size_t>(coloured, 1)) << "% of them)\n";
    }
    if (hasRgb) {
        // The colour resampled onto the depth grid: compare it with
        // color_camera.jpg to judge whether the mapping lines up.
        std::ofstream ppm(std::string(outDir) + "/depth_colored.ppm", std::ios::binary);
        ppm << "P6\n" << frame->width << ' ' << frame->height << "\n255\n";
        ppm.write(reinterpret_cast<const char*>(frame->rgb.data()),
                  static_cast<std::streamsize>(frame->rgb.size()));
    }
    auto cloud = revoscan::pointcloud::fromDepthFrame(*frame, cal.depth);
    const std::string ply = std::string(outDir) + "/colored.ply";
    if (!revoscan::io::writePly(ply, cloud)) {
        std::cerr << "failed to write " << ply << '\n';
        return 4;
    }
    std::cout << "wrote " << ply << " with " << cloud.points.size() << " points\n";
    return 0;
}

// Sweeps exposure in the normal infrared mode and records frames per step,
// to find what the scene in front of the scanner actually wants.
//   all3dscan --pop4-exposure-sweep <outdir>
int pop4ExposureSweep(const char* outDir) {
    std::filesystem::create_directories(outDir);
    ::setenv("ALL3DSCAN_POP4_DUMP_DIR", outDir, 1);
    ::setenv("ALL3DSCAN_POP4_DUMP_MAX", "4000", 1);
    auto scanner = revoscan::protocol::makeRevopointPop4Protocol();
    if (!scanner->connect() || !scanner->startStream()) {
        std::cerr << "POP4 connect/stream failed: " << scanner->lastError() << '\n';
        return 2;
    }
    std::ofstream steps(std::string(outDir) + "/steps.txt");
    int frameIndex = 0;
    for (int exp : {3000, 2000, 1200, 700, 400, 200}) {
        const bool ok = scanner->setExposure(exp);
        std::cout << "exposure " << exp << ": " << (ok ? "ok" : "FAILED " + scanner->lastError())
                  << std::endl;
        const int first = frameIndex;
        for (int i = 0; i < 20; ++i) {
            if (scanner->readFrame(3000)) {
                ++frameIndex;
            }
        }
        steps << "exp" << exp << ' ' << first << ' ' << frameIndex - 1 << " ok\n";
        steps.flush();
    }
    scanner->setExposure(3000);
    scanner->stopStream();
    scanner->disconnect();
    return 0;
}

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
    // The app's default preset, overridable so a recording can be replayed
    // against the other scanning-distance presets:
    //   ALL3DSCAN_REPLAY_RANGE=<nearMm>,<farMm>
    float nearMm = 250.0f;
    float farMm = 500.0f;
    if (const char* r = std::getenv("ALL3DSCAN_REPLAY_RANGE")) {
        float a = 0.0f, b = 0.0f;
        if (std::sscanf(r, "%f,%f", &a, &b) == 2 && a > 0.0f && b > a) {
            nearMm = a;
            farMm = b;
        }
    }
    decoder.setDepthRange(nearMm, farMm);

    revoscan::document::ScanDocument document;
    revoscan::processing::ScanPipeline pipeline(&document);
    pipeline.setDepthClip(nearMm / 1000.0f, farMm / 1000.0f);
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
        const size_t rawPoints = cloud.points.size();
        // ALL3DSCAN_REPLAY_NOISOLATE=1 keeps the back-projected cloud whole,
        // to separate "the scanner saw little" from "a filter discarded it".
        if (!std::getenv("ALL3DSCAN_REPLAY_NOISOLATE")) {
            cloud = revoscan::pointcloud::isolateForegroundObject(cloud, {});
        }
        // ALL3DSCAN_REPLAY_STAGES=1 reports what each filter keeps, so a scan
        // that arrives with geometry and leaves without it can be traced.
        if (std::getenv("ALL3DSCAN_REPLAY_STAGES")) {
            std::vector<float> zs;
            zs.reserve(cloud.points.size());
            for (const auto& pt : cloud.points) {
                zs.push_back(pt.z);
            }
            float zMed = 0.0f;
            if (!zs.empty()) {
                std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(zs.size() / 2),
                                 zs.end());
                zMed = zs[zs.size() / 2];
            }
            std::cout << "  stages: backprojected=" << rawPoints
                      << " isolated=" << cloud.points.size()
                      << " medianZmm=" << zMed * 1000.0f << '\n';
        }
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
        if (std::string_view(argv[i]) == "--pop4-blue-exposure-test" && i + 1 < argc) {
            return pop4BlueExposureTest(argv[i + 1]);
        }
        if (std::string_view(argv[i]) == "--pop4-color-test" && i + 1 < argc) {
            return pop4ColorTest(argv[i + 1]);
        }
        if (std::string_view(argv[i]) == "--pop4-exposure-sweep" && i + 1 < argc) {
            return pop4ExposureSweep(argv[i + 1]);
        }
        if (std::string_view(argv[i]) == "--pop4-cs7-test" && i + 1 < argc) {
            return pop4Cs7Test(argv[i + 1]);
        }
        if (std::string_view(argv[i]) == "--pop4-mode-cycle") {
            return pop4ModeCycle();
        }
        if (std::string_view(argv[i]) == "--pop4-mode-test" && i + 1 < argc) {
            return pop4ModeTest(argv[i + 1]);
        }
        if (std::string_view(argv[i]) == "--pop4-lights-test" && i + 1 < argc) {
            return pop4LightsTest(argv[i + 1]);
        }
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
    QApplication::setApplicationVersion("Alpha-v20261018-0030");
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
    // --gui-mode-test: drive the real window the way a user does — press
    // Connect, then pick "Blue laser speckle" in the Scan Mode box — and
    // report what the widgets actually do. Exercises the panel -> MainWindow
    // -> ScannerController chain that a protocol-level test cannot reach.
    for (int i = 1; i < argc; ++i) {
        if (std::string_view(argv[i]) != "--gui-mode-test") {
            continue;
        }
        QTimer::singleShot(500, &window, [&window] {
            auto pump = [](int ms) {
                const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
                while (std::chrono::steady_clock::now() < until) {
                    QApplication::processEvents(QEventLoop::AllEvents, 10);
                }
            };
            auto* combo = window.findChild<QComboBox*>();
            const auto boxes = window.findChildren<QComboBox*>();
            QComboBox* modeBox = nullptr;
            for (auto* b : boxes) {
                for (int k = 0; k < b->count(); ++k) {
                    if (b->itemText(k).contains("laser", Qt::CaseInsensitive)) {
                        modeBox = b;
                    }
                }
            }
            (void)combo;
            if (!modeBox) {
                std::cout << "FAIL: no Scan Mode box found in the window\n";
                QApplication::quit();
                return;
            }
            std::cout << "Scan Mode box: enabled=" << modeBox->isEnabled()
                      << " items=" << modeBox->count() << " current=\""
                      << modeBox->currentText().toStdString() << "\"\n";

            QPushButton* connectBtn = nullptr;
            for (auto* b : window.findChildren<QPushButton*>()) {
                if (b->text().compare("Connect", Qt::CaseInsensitive) == 0) {
                    connectBtn = b;
                }
            }
            if (!connectBtn) {
                std::cout << "FAIL: no Connect button\n";
                QApplication::quit();
                return;
            }
            std::cout << "pressing Connect…\n";
            connectBtn->click();
            for (int t = 0; t < 180 && connectBtn->text().compare("Disconnect", Qt::CaseInsensitive) != 0; ++t) {
                pump(1000);
            }
            std::cout << "after connect: button=\"" << connectBtn->text().toStdString()
                      << "\" modeBox enabled=" << modeBox->isEnabled() << "\n";
            pump(3000);

            int blueIndex = -1;
            for (int k = 0; k < modeBox->count(); ++k) {
                if (modeBox->itemText(k).contains("Blue laser", Qt::CaseInsensitive)) {
                    blueIndex = k;
                }
            }
            std::cout << "selecting item " << blueIndex << " (\""
                      << modeBox->itemText(blueIndex).toStdString() << "\")\n";
            modeBox->setCurrentIndex(blueIndex);
            for (int t = 0; t < 90; ++t) {
                pump(1000);
                if (modeBox->isEnabled()) {
                    break; // re-enabled once the switch reported back
                }
            }
            std::cout << "after selecting: current=\"" << modeBox->currentText().toStdString()
                      << "\" enabled=" << modeBox->isEnabled() << "\n";
            pump(4000);
            if (const char* shot = std::getenv("ALL3DSCAN_GUI_SHOT")) {
                window.grab().save(QString::fromLocal8Bit(shot));
                std::cout << "saved window shot while blue laser is selected\n";
            }
            pump(1000);
            std::cout << "selecting item 0 (back to infrared)…\n";
            modeBox->setCurrentIndex(0);
            pump(1000);
            std::cout << "  during switch: enabled=" << modeBox->isEnabled() << "\n";
            for (int t = 0; t < 120; ++t) {
                pump(1000);
                if (modeBox->isEnabled()) {
                    break;
                }
            }
            std::cout << "after switching back: current=\"" << modeBox->currentText().toStdString()
                      << "\" enabled=" << modeBox->isEnabled() << "\n";
            pump(2000);
            QApplication::quit();
        });
    }
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
