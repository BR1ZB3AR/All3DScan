"""All3DScan desktop window (PyQt6).

Captures point clouds through the engine (C++ sensor -> Rust -> numpy) on a
background QThread, so the window stays responsive while a frame is built.
Run from the repository root after `maturin develop`:

    .venv/bin/python gui_python/main.py

With --smoke-test the window captures three frames by itself, prints what it
got and exits, so the whole pipeline can be checked without anyone clicking.
"""

import argparse
import sys
import time
import traceback

# Imported up front so a missing numpy stops the app at launch, not in the worker.
import numpy as np
from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QLabel,
    QMainWindow,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

import all3dscan_engine as engine
from common import APP_ICON, APP_ID, apply_dark_theme


class CaptureWorker(QObject):
    """Runs engine captures on whichever thread it has been moved to."""

    captured = pyqtSignal(object, float)  # (N, 3) float32 array, seconds taken
    failed = pyqtSignal(str)

    def __init__(self, sensor: engine.MockDepthSensor) -> None:
        super().__init__()
        self._sensor = sensor

    @pyqtSlot(float)
    def capture(self, voxel_mm: float) -> None:
        start = time.perf_counter()
        try:
            cloud = self._sensor.capture(voxel_mm=voxel_mm if voxel_mm > 0 else None)
        # BaseException, because a Rust panic arrives as PanicException, which
        # is not an Exception; letting anything escape a slot ends the app.
        except BaseException as exc:
            self.failed.emit(f"{type(exc).__name__}: {exc}")
            return
        self.captured.emit(cloud, time.perf_counter() - start)


class MainWindow(QMainWindow):
    capture_requested = pyqtSignal(float)

    def __init__(self, sensor: engine.MockDepthSensor) -> None:
        super().__init__()
        self.setWindowTitle("All3DScan")
        self.resize(480, 380)

        heading = QLabel("Mock depth sensor")
        heading.setObjectName("heading")
        details = QLabel(f"{sensor.width} × {sensor.height} pixels · engine {engine.__version__}")
        details.setObjectName("subtle")

        self._voxel = QDoubleSpinBox()
        self._voxel.setRange(0.0, 50.0)
        self._voxel.setSingleStep(0.5)
        self._voxel.setValue(4.0)
        self._voxel.setSuffix(" mm")
        self._voxel.setSpecialValueText("off")  # shown at 0: keep every point

        self._capture_button = QPushButton("Capture")
        self._capture_button.setObjectName("primary")
        self._capture_button.clicked.connect(self.capture)

        # Animated by the GUI thread, so it visibly stalls if that thread ever blocks.
        self._busy = QProgressBar()
        self._busy.setRange(0, 0)
        self._busy.setTextVisible(False)
        self._busy.setFixedHeight(6)
        self._busy.setVisible(False)

        self._points = self._value_label()
        self._shape = self._value_label()
        self._took = self._value_label()
        self._extent = self._value_label()
        results = QFormLayout()
        results.addRow("Voxel size", self._voxel)
        results.addRow("Points", self._points)
        results.addRow("Array shape", self._shape)
        results.addRow("Capture time", self._took)
        results.addRow("Extent", self._extent)

        layout = QVBoxLayout()
        layout.setContentsMargins(24, 20, 24, 16)
        layout.setSpacing(12)
        layout.addWidget(heading)
        layout.addWidget(details)
        layout.addLayout(results)
        layout.addWidget(self._busy)
        layout.addWidget(self._capture_button)
        layout.addStretch()
        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)
        self.statusBar().showMessage("Ready")

        self._thread = QThread(self)
        self._worker = CaptureWorker(sensor)
        self._worker.moveToThread(self._thread)
        self._thread.finished.connect(self._worker.deleteLater)
        self.capture_requested.connect(self._worker.capture)
        self._worker.captured.connect(self._on_captured)
        self._worker.failed.connect(self._on_failed)
        self._thread.start()
        QApplication.instance().aboutToQuit.connect(self._stop_worker)

    @staticmethod
    def _value_label() -> QLabel:
        label = QLabel("—")
        label.setProperty("role", "value")
        return label

    def capture(self) -> None:
        self._capture_button.setEnabled(False)
        self._busy.setVisible(True)
        self.statusBar().showMessage("Capturing…")
        self.capture_requested.emit(self._voxel.value())

    def _on_captured(self, cloud: np.ndarray, seconds: float) -> None:
        print(f"point cloud shape: {cloud.shape}, dtype {cloud.dtype}, "
              f"{seconds * 1000:.1f} ms", flush=True)
        self._points.setText(f"{len(cloud):,}")
        self._shape.setText(f"{cloud.shape}  {cloud.dtype}")
        self._took.setText(f"{seconds * 1000:.1f} ms")
        if len(cloud):
            size = cloud.max(axis=0) - cloud.min(axis=0)
            self._extent.setText("  ×  ".join(f"{v:.0f}" for v in size) + " mm")
        else:
            self._extent.setText("—")
        self._finish("Captured")

    def _on_failed(self, message: str) -> None:
        print(f"capture failed: {message}", file=sys.stderr, flush=True)
        self._finish(f"Capture failed: {message}")

    def _finish(self, status: str) -> None:
        self._busy.setVisible(False)
        self._capture_button.setEnabled(True)
        self.statusBar().showMessage(status)

    def _stop_worker(self) -> None:
        # Runs on close and on quit: Qt aborts the app if a QThread is
        # destroyed while it is still running.
        if self._thread.isRunning():
            self._thread.quit()
            self._thread.wait()

    def closeEvent(self, event) -> None:
        self._stop_worker()
        super().closeEvent(event)


def run_smoke_test(app: QApplication, window: MainWindow, frames: int = 3) -> None:
    """Captures `frames` frames through the window, then exits 0, or 1 on failure."""
    beat = {"last": time.perf_counter(), "longest": 0.0, "done": 0}

    def tick() -> None:
        now = time.perf_counter()
        beat["longest"] = max(beat["longest"], now - beat["last"])
        beat["last"] = now

    heartbeat = QTimer(window)
    heartbeat.timeout.connect(tick)

    def start_capture() -> None:
        beat["last"] = time.perf_counter()  # measure from here, not from start-up
        window.capture()

    def on_captured(cloud: np.ndarray, _seconds: float) -> None:
        if cloud.ndim != 2 or cloud.shape[1] != 3 or len(cloud) == 0:
            print(f"smoke test: unexpected array {cloud.shape}", file=sys.stderr)
            app.exit(1)
            return
        beat["done"] += 1
        if beat["done"] < frames:
            QTimer.singleShot(0, start_capture)
            return
        heartbeat.stop()
        print(f"smoke test: {frames} frames captured; the GUI thread's longest pause "
              f"while capturing was {beat['longest'] * 1000:.1f} ms", flush=True)
        app.exit(0)

    def time_out() -> None:
        print("smoke test: timed out", file=sys.stderr)
        app.exit(2)

    window._worker.captured.connect(on_captured)
    window._worker.failed.connect(lambda _message: app.exit(1))
    QTimer.singleShot(30_000, time_out)
    heartbeat.start(5)
    QTimer.singleShot(0, start_capture)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--smoke-test", action="store_true",
                        help="capture three frames without user input, then exit")
    args, qt_args = parser.parse_known_args(argv)

    app = QApplication([sys.argv[0], *qt_args])
    app.setApplicationName("All3DScan")
    app.setDesktopFileName(APP_ID)
    if APP_ICON.exists():
        app.setWindowIcon(QIcon(str(APP_ICON)))
    apply_dark_theme(app)

    def report_unhandled(exc_type, exc, tb) -> None:
        # With the default hook, PyQt6 aborts the whole app when a slot raises.
        traceback.print_exception(exc_type, exc, tb)
        if args.smoke_test:
            app.exit(1)

    sys.excepthook = report_unhandled

    try:
        sensor = engine.MockDepthSensor(args.width, args.height, args.seed)
    except ValueError as exc:
        parser.error(str(exc))
    window = MainWindow(sensor)
    window.show()
    if args.smoke_test:
        run_smoke_test(app, window)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
