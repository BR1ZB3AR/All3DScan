"""Tests of the All3DScan window, drawn offscreen.

Run from the repository root after `maturin develop`:

    .venv/bin/python -m unittest discover -s gui_python/tests
"""

import os
import subprocess
import sys
import time
import unittest
from pathlib import Path

# Offscreen unless the caller picks a platform, so no windows open on the desktop.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication

GUI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GUI_DIR))

import all3dscan_engine as engine  # noqa: E402
import main  # noqa: E402

app = QApplication.instance() or QApplication([])
main.apply_dark_theme(app)


def wait_until(condition, timeout_s: float = 10.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError("timed out waiting for the window")
        QTest.qWait(5)


class FakePanic(BaseException):
    """Like pyo3's PanicException: a BaseException, not an Exception."""


class UnpluggedSensor:
    width, height = 8, 8

    def capture(self, voxel_mm=None):
        raise FakePanic("sensor unplugged")


class MainWindowTests(unittest.TestCase):
    def open_window(self, sensor) -> main.MainWindow:
        window = main.MainWindow(sensor)
        self.addCleanup(window.close)
        window.show()
        return window

    def click_capture(self, window: main.MainWindow) -> None:
        QTest.mouseClick(window._capture_button, Qt.MouseButton.LeftButton)
        self.assertFalse(window._capture_button.isEnabled())
        wait_until(window._capture_button.isEnabled)

    def test_capture_button_fills_in_the_results(self):
        window = self.open_window(engine.MockDepthSensor(64, 48, seed=1))
        self.click_capture(window)
        self.assertRegex(window._shape.text(), r"^\(\d+, 3\)  float32$")
        self.assertRegex(window._extent.text(), r"^\d+  ×  \d+  ×  \d+ mm$")
        self.assertEqual(window.statusBar().currentMessage(), "Captured")

    def test_voxel_size_off_keeps_every_point(self):
        expected = len(engine.MockDepthSensor(64, 48, seed=2).capture())
        window = self.open_window(engine.MockDepthSensor(64, 48, seed=2))
        window._voxel.setValue(0.0)
        self.assertEqual(window._voxel.text(), "off")
        self.click_capture(window)
        self.assertEqual(window._points.text(), f"{expected:,}")

    def test_a_failed_capture_is_reported_and_the_window_carries_on(self):
        window = self.open_window(UnpluggedSensor())
        self.click_capture(window)
        self.assertEqual(
            window.statusBar().currentMessage(), "Capture failed: FakePanic: sensor unplugged"
        )
        self.click_capture(window)  # still usable afterwards

    def test_closing_the_window_stops_its_worker_thread(self):
        window = main.MainWindow(engine.MockDepthSensor(8, 8))
        window.show()
        self.assertTrue(window._thread.isRunning())
        window.close()
        self.assertFalse(window._thread.isRunning())

    def test_the_window_keeps_responding_while_it_captures(self):
        sensor = engine.MockDepthSensor(2048, 1536, seed=3)
        start = time.perf_counter()
        sensor.capture()
        capture_s = time.perf_counter() - start
        if capture_s < 0.02:
            self.skipTest(f"a capture took {capture_s:.3f}s, too quick to tell a stall apart")

        window = self.open_window(sensor)
        beat = {"last": time.perf_counter(), "longest": 0.0}

        def tick():
            now = time.perf_counter()
            beat["longest"] = max(beat["longest"], now - beat["last"])
            beat["last"] = now

        timer = QTimer()
        timer.timeout.connect(tick)
        timer.start(2)
        for _ in range(3):
            beat["last"] = time.perf_counter()
            self.click_capture(window)
        timer.stop()
        self.assertLess(
            beat["longest"],
            capture_s / 2,
            f"the GUI thread stalled {beat['longest']:.3f}s; a capture takes {capture_s:.3f}s",
        )


class SmokeTestModeTests(unittest.TestCase):
    def test_captures_three_frames_and_exits_cleanly(self):
        # Also guards the exit path: a still-running QThread aborts the process.
        result = subprocess.run(
            [sys.executable, str(GUI_DIR / "main.py"), "--smoke-test",
             "--width", "160", "--height", "120"],
            capture_output=True, text=True, timeout=60,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count("point cloud shape: ("), 3, result.stdout)
        self.assertIn("smoke test: 3 frames captured", result.stdout)


if __name__ == "__main__":
    unittest.main()
