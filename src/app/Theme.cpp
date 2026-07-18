#include "Theme.h"

namespace revoscan::app {

QString revoScanStyleSheet() {
    return QStringLiteral(R"QSS(
QMainWindow, QWidget#Central {
    background-color: #1b1d21;
    color: #d6dae0;
}
QWidget {
    color: #d6dae0;
    font-size: 12px;
}
QMenuBar {
    background-color: #26292e;
    color: #d6dae0;
    border-bottom: 1px solid #3a3f45;
}
QMenuBar::item { background: transparent; padding: 6px 12px; }
QMenuBar::item:selected { background: #33383e; }
QMenu {
    background-color: #26292e;
    border: 1px solid #3a3f45;
}
QMenu::item:selected { background: #12b6b0; color: #10161a; }

QFrame#Ribbon {
    background-color: #26292e;
    border-bottom: 1px solid #3a3f45;
}
QFrame#RibbonGroup { background: transparent; }
QLabel#RibbonGroupTitle { color: #8b9198; font-size: 10px; }
QFrame#RibbonSep { background-color: #3a3f45; max-width: 1px; }

QFrame#SidePanel {
    background-color: #232629;
    border-right: 1px solid #3a3f45;
}
QLabel#PanelTitle {
    color: #d6dae0;
    font-size: 12px;
    font-weight: bold;
    padding: 6px 8px;
    background-color: #2b2f33;
}
QLabel#FieldLabel { color: #b8bdc4; }

QToolButton#CamAutoBtn, QToolButton#CamStepBtn {
    padding: 3px 2px;
    background-color: #2f343a;
    border: 1px solid #3a3f45;
    border-radius: 3px;
}
QToolButton#CamAutoBtn:checked { background-color: #12b6b0; color: #10161a; }
QSpinBox#CamSpin { padding: 2px 2px; }

QLabel#PreviewArea {
    background-color: #101214;
    border: 1px solid #3a3f45;
    color: #5b6169;
}

QToolButton, QPushButton {
    background-color: #2f343a;
    border: 1px solid #3a3f45;
    border-radius: 3px;
    padding: 5px 10px;
    color: #d6dae0;
}
QToolButton:hover, QPushButton:hover { background-color: #3a4046; }
QToolButton:pressed, QPushButton:pressed { background-color: #12b6b0; color: #10161a; }
QToolButton:checked, QPushButton:checked { background-color: #12b6b0; color: #10161a; }
QPushButton:disabled { color: #5b6169; border-color: #2b2f33; }

QPushButton#ScanPrimary {
    background-color: #12b6b0;
    color: #0e1417;
    font-weight: bold;
    border: none;
    padding: 7px 16px;
}
QPushButton#ScanPrimary:hover { background-color: #17c9c2; }
QPushButton#ScanPrimary:disabled { background-color: #2b3a3a; color: #5b6169; }

QComboBox {
    background-color: #2f343a;
    border: 1px solid #3a3f45;
    border-radius: 3px;
    padding: 4px 8px;
    min-height: 20px;
}
QComboBox:hover { border-color: #12b6b0; }
QComboBox::drop-down { border: none; width: 18px; }
QComboBox QAbstractItemView {
    background-color: #26292e;
    selection-background-color: #12b6b0;
    selection-color: #10161a;
    border: 1px solid #3a3f45;
}

QSpinBox, QLineEdit {
    background-color: #1b1d21;
    border: 1px solid #3a3f45;
    border-radius: 3px;
    padding: 3px 6px;
    color: #d6dae0;
}

QFrame#StatusBar {
    background-color: #26292e;
    border-top: 1px solid #3a3f45;
}
QLabel#StatusHint { color: #8b9198; font-size: 11px; }
QLabel#StatusConn { color: #5bbf5b; font-size: 11px; }
QLabel#Fps { color: #b8bdc4; font-size: 11px; }

QFrame#SegmentedToggle {
    background-color: #2b2f33;
    border: 1px solid #3a3f45;
    border-radius: 4px;
}
)QSS");
}

} // namespace revoscan::app
