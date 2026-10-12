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

/* ---- Menu ---- */
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
    padding: 4px 0;
}
QMenu::item { padding: 5px 28px 5px 12px; }
QMenu::item:selected { background: #12b6b0; color: #10161a; }
QMenu::separator { height: 1px; background: #3a3f45; margin: 4px 8px; }

/* ---- Ribbon ---- */
QFrame#Ribbon {
    background-color: #26292e;
    border-bottom: 1px solid #3a3f45;
}
QFrame#RibbonGroup { background: transparent; }
QLabel#RibbonGroupTitle { color: #8b9198; font-size: 10px; }
QFrame#RibbonSep { background-color: #3a3f45; max-width: 1px; }

/* ---- Side Panels ---- */
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
    border-bottom: 1px solid #3a3f45;
}
QLabel#FieldLabel { color: #b8bdc4; }

/* ---- Camera Panel ---- */
QToolButton#CamAutoBtn, QToolButton#CamStepBtn {
    padding: 3px 2px;
    background-color: #2f343a;
    border: 1px solid #3a3f45;
    border-radius: 3px;
}
QToolButton#CamAutoBtn:hover, QToolButton#CamStepBtn:hover { background-color: #3a4046; }
QToolButton#CamAutoBtn:checked { background-color: #12b6b0; color: #10161a; }
QSpinBox#CamSpin { padding: 2px 2px; }

QLabel#PreviewArea {
    background-color: #101214;
    border: 1px solid #3a3f45;
    color: #5b6169;
}
QToolButton#OverlayModeBtn {
    background-color: rgba(30, 32, 36, 180);
    border: 1px solid #3a3f45;
    border-radius: 4px;
    color: #d6dae0;
    padding: 3px 6px;
    font-size: 10px;
    font-weight: bold;
}
QToolButton#OverlayModeBtn:hover {
    background-color: rgba(47, 52, 58, 220);
    color: #ffffff;
    border-color: #12b6b0;
}
QToolButton#OverlayModeBtn:checked {
    background-color: #12b6b0;
    color: #10161a;
    border-color: #12b6b0;
}
QWidget#ZoomGroup {
    background-color: rgba(20, 22, 26, 200);
    border: 1px solid #3a3f45;
    border-radius: 12px;
}
QToolButton#OverlayZoomBtn {
    background-color: transparent;
    border: none;
    border-radius: 10px;
    color: #8b9198;
    padding: 2px 8px;
    font-size: 10px;
    font-weight: bold;
}
QToolButton#OverlayZoomBtn:hover {
    color: #ffffff;
}
QToolButton#OverlayZoomBtn:checked {
    background-color: #12b6b0;
    color: #10161a;
}

/* ---- Buttons ---- */
QToolButton, QPushButton {
    background-color: #2f343a;
    border: 1px solid #3a3f45;
    border-radius: 3px;
    padding: 5px 10px;
    color: #d6dae0;
}
QToolButton:hover, QPushButton:hover {
    background-color: #3a4046;
    border-color: #12b6b0;
}
QToolButton:pressed, QPushButton:pressed {
    background-color: #12b6b0;
    color: #10161a;
    border-color: #12b6b0;
}
QToolButton:checked, QPushButton:checked {
    background-color: #12b6b0;
    color: #10161a;
    border-color: #12b6b0;
}
QPushButton:disabled {
    color: #5b6169;
    border-color: #2b2f33;
    background-color: #232629;
}

QPushButton#ScanPrimary {
    background-color: #12b6b0;
    color: #0e1417;
    font-weight: bold;
    border: none;
    padding: 7px 16px;
    border-radius: 4px;
}
QPushButton#ScanPrimary:hover {
    background-color: #17c9c2;
}
QPushButton#ScanPrimary:pressed {
    background-color: #0ea09b;
}
QPushButton#ScanPrimary:disabled {
    background-color: #2b3a3a;
    color: #5b6169;
}

/* ---- Combo Box ---- */
QComboBox {
    background-color: #2f343a;
    border: 1px solid #3a3f45;
    border-radius: 3px;
    padding: 4px 8px;
    min-height: 20px;
}
QComboBox:hover { border-color: #12b6b0; }
QComboBox:focus { border-color: #12b6b0; }
QComboBox::drop-down { border: none; width: 18px; }
QComboBox::down-arrow { width: 10px; height: 10px; }
QComboBox QAbstractItemView {
    background-color: #26292e;
    selection-background-color: #12b6b0;
    selection-color: #10161a;
    border: 1px solid #3a3f45;
    outline: none;
}

/* ---- SpinBox, LineEdit ---- */
QSpinBox, QLineEdit {
    background-color: #1b1d21;
    border: 1px solid #3a3f45;
    border-radius: 3px;
    padding: 3px 6px;
    color: #d6dae0;
}
QSpinBox:focus, QLineEdit:focus { border-color: #12b6b0; }

/* ---- Horizontal Slider ---- */
QSlider::groove:horizontal {
    background: #1b1d21;
    border: 1px solid #3a3f45;
    height: 6px;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: #12b6b0;
    border: 1px solid #12b6b0;
    height: 6px;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #d6dae0;
    border: 1px solid #5b6169;
    width: 14px;
    height: 14px;
    margin: -5px 0;
    border-radius: 7px;
}
QSlider::handle:horizontal:hover {
    background: #ffffff;
    border-color: #12b6b0;
}
QSlider::handle:horizontal:pressed {
    background: #12b6b0;
    border-color: #12b6b0;
}
QSlider:disabled::groove:horizontal { background: #232629; border-color: #2b2f33; }
QSlider:disabled::handle:horizontal { background: #3a4046; border-color: #2b2f33; }

/* ---- Scrollbar ---- */
QScrollBar:vertical {
    background: #232629;
    width: 10px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #3a3f45;
    border-radius: 4px;
    min-height: 30px;
    margin: 2px;
}
QScrollBar::handle:vertical:hover { background: #4a5058; }
QScrollBar::handle:vertical:pressed { background: #12b6b0; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}
QScrollBar:horizontal {
    background: #232629;
    height: 10px;
}
QScrollBar::handle:horizontal {
    background: #3a3f45;
    border-radius: 4px;
    min-width: 30px;
    margin: 2px;
}
QScrollBar::handle:horizontal:hover { background: #4a5058; }
QScrollBar::handle:horizontal:pressed { background: #12b6b0; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0;
}

/* ---- Status Bar ---- */
QFrame#StatusBar {
    background-color: #26292e;
    border-top: 1px solid #3a3f45;
}
QLabel#StatusHint { color: #8b9198; font-size: 11px; }
QLabel#StatusConn { color: #5bbf5b; font-size: 11px; font-weight: bold; }
QLabel#Fps { color: #b8bdc4; font-size: 11px; padding: 0 4px; }

/* ---- Segmented Toggle ---- */
QFrame#SegmentedToggle {
    background-color: #2b2f33;
    border: 1px solid #3a3f45;
    border-radius: 4px;
}

/* ---- Tab Widget ---- */
QTabWidget::pane {
    border: 1px solid #3a3f45;
    background-color: #232629;
    border-radius: 4px;
}
QTabBar::tab {
    background-color: #2b2f33;
    color: #b8bdc4;
    border: 1px solid #3a3f45;
    border-bottom: none;
    padding: 6px 16px;
    margin-right: 2px;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
}
QTabBar::tab:selected {
    background-color: #12b6b0;
    color: #10161a;
    font-weight: bold;
}
QTabBar::tab:hover:!selected {
    background-color: #3a4046;
    color: #ffffff;
}
)QSS");
}

} // namespace revoscan::app
