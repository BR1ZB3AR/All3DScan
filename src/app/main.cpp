#include <QApplication>

#include <algorithm>
#include <iostream>
#include <string_view>

#include "MainWindow.h"
#include "bridge/RustBridge.h"
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
        std::cout << "Metric decoding: ok (approximate full-field mm)\n";
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

} // namespace

int main(int argc, char** argv) {
    std::cout << "All3DScan Rust Core v" << revoscan::bridge::RustBridge::getVersion() << " initialized.\n";

    for (int i = 1; i < argc; ++i) {
        if (std::string_view(argv[i]) == "--probe-pop4") {
            return probePop4();
        }
    }

    QApplication app(argc, argv);
    QApplication::setApplicationName("All3DScan");
    QApplication::setApplicationVersion("Alpha-v20260802-1033");
    QApplication::setOrganizationName("All3DScan");
    QApplication::setDesktopFileName("io.github.all3dscan.All3DScan");
    // No window / taskbar logo — plain Qt default decoration.

    revoscan::app::MainWindow window;
    window.show();
    return app.exec();
}
