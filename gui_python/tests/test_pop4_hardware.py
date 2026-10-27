"""Tests of the main window with a real POP 4: what the user does, judged by
what the scanner's own cameras then see (CLAUDE.md, "Verify at the layer the
user operates" and "Prefer evidence the device itself gives you").

They take the scanner for a minute or so, so they only run when asked, with
a POP 4 plugged in:

    ALL3DSCAN_POP4_TESTS=1 .venv/bin/python -m unittest gui_python/tests/test_pop4_hardware.py

The blue laser test also reboots the scanner (about 45 s), so it needs
ALL3DSCAN_POP4_SLOW=1 as well.
"""

import os
import sys
import time
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402
from PyQt6.QtTest import QTest  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

from common import apply_dark_theme, configure_opengl  # noqa: E402

if QApplication.instance() is None:
    configure_opengl()
qt_app = QApplication.instance() or QApplication([])
apply_dark_theme(qt_app)

import all3dscan_engine as engine  # noqa: E402
import app  # noqa: E402
from panels import ScanState  # noqa: E402

ASKED = os.environ.get("ALL3DSCAN_POP4_TESTS") == "1"
SLOW = os.environ.get("ALL3DSCAN_POP4_SLOW") == "1"


def wait_until(condition, timeout_s: float, what: str) -> None:
    deadline = time.monotonic() + timeout_s
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError(f"timed out waiting for {what}")
        QTest.qWait(5)


def pattern_contrast(ir: np.ndarray) -> float:
    """The std of the image minus its 5x5 box mean, over its mean: high
    while the projector draws its speckle, low without it."""
    image = ir.astype(np.float32)
    box = np.lib.stride_tricks.sliding_window_view(image, (5, 5)).mean(axis=(2, 3))
    return float((image[2:-2, 2:-2] - box).std() / max(image.mean(), 1.0))


def blue_pixels(rgb: np.ndarray) -> float:
    """The share of the colour image that is strongly blue (B - R > 25)."""
    rgb = rgb.astype(np.int16)
    return float(np.mean(rgb[..., 2] - rgb[..., 0] > 25))


@unittest.skipUnless(ASKED, "set ALL3DSCAN_POP4_TESTS=1 to test with a real POP 4")
class Pop4WindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not engine.pop4_present():
            raise unittest.SkipTest("no POP 4 on USB")
        cls.window = app.MainWindow()
        try:
            cls.window.resize(1440, 900)
            cls.window.show()
            QTest.qWait(50)
            # As the user does: pick it in the device list. Connecting can
            # include a reset, which reboots the scanner.
            start = time.monotonic()
            cls.window.scan_panel.device.setCurrentIndex(1)
            wait_until(lambda: cls.window._pop4
                       or cls.window.scan_panel.device.currentIndex() == 0,
                       120, "the POP 4 to connect")
            if not cls.window._pop4:
                raise AssertionError(
                    f"the POP 4 did not open: {cls.window.viewport.toast.text()}")
            wait_until(cls.window.scan_panel.new_scan_button.isEnabled, 10, "New scan")
            print(f"\n  connected in {time.monotonic() - start:.1f} s")
            cls.window.scan_panel.new_scan()
        except BaseException:
            cls.window.close()  # stops its threads, or Qt aborts the process
            raise

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "window"):
            cls.window.close()

    def frames(self, count: int, settle: int = 0) -> list:
        """The next `count` frames the window shows, after letting `settle` go by."""
        window, seen = self.window, []
        last = window.frames
        deadline = time.monotonic() + 20 + 0.5 * (count + settle)
        while len(seen) < count:
            if time.monotonic() > deadline:
                raise AssertionError(f"only {len(seen)} of {count} frames came")
            QTest.qWait(5)
            if window.frames != last:
                last = window.frames
                if settle:
                    settle -= 1
                else:
                    seen.append(window.last_frame)
        return seen

    def test_1_it_connects_with_factory_calibration_and_streams(self):
        window = self.window
        self.assertRegex(window._connection.text(), r"Revopoint POP 4 \w+ connected")
        start = time.monotonic()
        frames = self.frames(10)
        rate = len(frames) / (time.monotonic() - start)
        frame = frames[-1]
        print(f"\n  {rate:.1f} frames/s; depth {frame.depth_mm.shape}, colour {frame.rgb.shape}; "
              f"{len(frame.points):,} points ({np.mean(frame.depth_mm > 0):.2%} of pixels)")
        self.assertIs(window.scan_panel.state, ScanState.SCANNING)
        self.assertEqual(frame.depth_mm.shape, (512, 320))
        self.assertEqual(frame.ir.shape, (512, 320))
        self.assertEqual(frame.rgb.shape[2], 3)
        self.assertGreater(frame.rgb.size, 0)  # the colour camera's image arrived
        for feed in (window.camera_panel.depth_view, window.camera_panel.rgb_view):
            self.assertFalse(feed.image.pixmap().isNull())
        self.assertEqual(window.viewport.view3d.point_count, len(frame.points))

    def test_2_the_projector_button_patterns_the_infrared_view(self):
        # A light changes about five frames after its command (measured
        # 2026-10-27), hence the frames let go by after each click.
        camera = self.window.camera_panel
        on = np.median([pattern_contrast(f.ir) for f in self.frames(5)])
        camera.projector_button.click()  # off
        off = np.median([pattern_contrast(f.ir) for f in self.frames(5, settle=6)])
        camera.projector_button.click()  # on again
        back = np.median([pattern_contrast(f.ir) for f in self.frames(5, settle=6)])
        print(f"\n  infrared pattern contrast: on {on:.3f}, off {off:.3f}, on again {back:.3f}")
        self.assertLess(off, 0.7 * on)
        self.assertGreater(back, 0.7 * on)

    def test_3_the_fill_light_lights_what_the_colour_camera_sees(self):
        camera = self.window.camera_panel
        dark = np.median([f.rgb.mean() for f in self.frames(5)])
        camera.fill_light_button.click()  # on
        lit = np.median([f.rgb.mean() for f in self.frames(5, settle=6)])
        camera.fill_light_button.click()  # off
        back = np.median([f.rgb.mean() for f in self.frames(5, settle=6)])
        print(f"\n  colour image brightness: fill light off {dark:.1f}, on {lit:.1f}, "
              f"off again {back:.1f}")
        self.assertGreater(lit, dark + 10)
        self.assertLess(back, lit - 10)

    @unittest.skipUnless(SLOW, "set ALL3DSCAN_POP4_SLOW=1: leaving blue laser reboots the scanner")
    def test_4_blue_laser_shows_in_the_colour_camera_and_infrared_comes_back(self):
        window, scan = self.window, self.window.scan_panel
        window.scan_panel.toggle_pause()  # the mode can only change between scans
        scan.complete()
        before = np.median([blue_pixels(window.last_frame.rgb)])

        scan.mode.setCurrentIndex(scan.mode.findData("blue laser"))
        scan.mode.activated.emit(scan.mode.currentIndex())
        wait_until(scan.new_scan_button.isEnabled, 30, "blue laser mode")
        self.assertEqual(window._scan_mode, "blue laser", window.viewport.toast.text())
        scan.new_scan()
        blue = np.median([blue_pixels(f.rgb) for f in self.frames(5, settle=6)])
        scan.complete()

        scan.mode.setCurrentIndex(scan.mode.findData("infrared"))
        scan.mode.activated.emit(scan.mode.currentIndex())
        wait_until(scan.new_scan_button.isEnabled, 120, "the reboot into infrared")
        self.assertEqual(window._scan_mode, "infrared", window.viewport.toast.text())
        scan.new_scan()
        frames = self.frames(5, settle=6)
        print(f"\n  strongly blue pixels: infrared {before:.2%}, blue laser {blue:.2%}; "
              f"after the reboot {np.mean(frames[-1].depth_mm > 0):.2%} of pixels have depth")
        self.assertGreater(blue, before + 0.005)


if __name__ == "__main__":
    unittest.main()
