#include <QApplication>
#include <QIcon>

#include "MainWindow.h"
#include "util/ResourcePaths.h"

int main(int argc, char** argv) {
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
