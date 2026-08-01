#include <QApplication>
#include <QIcon>

#include <algorithm>
#include <iostream>
#include <string_view>

#include "MainWindow.h"
#include "protocol/ScannerProtocol.h"
#include "util/ResourcePaths.h"

namespace {

int probePop4() {
    auto scanner = revoscan::protocol::makeRevopointPop4Protocol();
    std::cout << "Backend: " << scanner->backendName() << '\n';
    if (!scanner->connect()) {
        std::cerr << "Connect failed: " << scanner->lastError() << '\n';
        return 2;
    }
    if (!scanner->startStream()) {
        std::cerr << "Stream start failed: " << scanner->lastError() << '\n';
        scanner->disconnect();
        return 3;
    }

    auto frame = scanner->readFrame(5000);
    scanner->stopStream();
    scanner->disconnect();
    if (!frame) {
        std::cerr << "Capture failed: " << scanner->lastError() << '\n';
        return 4;
    }
    const auto nonZero = std::count_if(
        frame->depthMm.begin(), frame->depthMm.end(),
        [](uint16_t value) { return value != 0; });
    std::cout << "Captured raw Y16 " << frame->width << 'x' << frame->height
              << ", samples=" << frame->depthMm.size()
              << ", nonzero=" << nonZero << '\n';
    std::cout << "Metric decoding: not implemented (raw data was not projected)\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::string_view(argv[i]) == "--probe-pop4") {
            return probePop4();
        }
    }

    QApplication app(argc, argv);
    QApplication::setApplicationName("All3DScan");
    QApplication::setApplicationVersion("0.1.0");
    QApplication::setOrganizationName("All3DScan");
    QApplication::setDesktopFileName("io.github.all3dscan.All3DScan");

    if (const auto iconPath = revoscan::util::findAppIcon()) {
        app.setWindowIcon(QIcon(QString::fromStdString(*iconPath)));
    }

    revoscan::app::MainWindow window;
    window.show();
    return app.exec();
}
