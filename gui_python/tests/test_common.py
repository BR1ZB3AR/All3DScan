"""Tests of what the windows share (common.py): working out which Qt platform
is coming, and asking for the OpenGL context to suit it.

Run from the repository root after `maturin develop`:

    .venv/bin/python -m unittest discover -s gui_python/tests
"""

import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest import mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PyQt6.QtGui import QSurfaceFormat  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

import common  # noqa: E402

if QApplication.instance() is None:
    common.configure_opengl()
qt_app = QApplication.instance() or QApplication([])


class PlatformTests(unittest.TestCase):
    def setUp(self):
        runtime = tempfile.TemporaryDirectory()
        self.addCleanup(runtime.cleanup)
        self.runtime = Path(runtime.name)
        (self.runtime / "wayland-9").touch()  # a compositor's socket

    def platform(self, argv=("app",), **env) -> str:
        with mock.patch.dict(os.environ, {"XDG_RUNTIME_DIR": str(self.runtime), **env},
                             clear=True):
            return common.chosen_platform(list(argv))

    def test_the_command_line_comes_first(self):
        self.assertEqual(self.platform(["app", "-platform", "xcb"], QT_QPA_PLATFORM="wayland"),
                         "xcb")
        self.assertEqual(self.platform(["app", "--platform=wayland-egl"]), "wayland-egl")

    def test_then_qt_qpa_platform_and_its_first_choice(self):
        self.assertEqual(self.platform(QT_QPA_PLATFORM="wayland;xcb"), "wayland")
        self.assertEqual(self.platform(QT_QPA_PLATFORM="xcb:nograb"), "xcb")

    def test_then_a_wayland_session_with_its_socket_there(self):
        self.assertEqual(self.platform(WAYLAND_DISPLAY="wayland-9"), "wayland")
        self.assertEqual(self.platform(WAYLAND_DISPLAY=str(self.runtime / "wayland-9")),
                         "wayland")
        self.assertEqual(self.platform(WAYLAND_DISPLAY="wayland-1"), "xcb")  # no socket
        self.assertEqual(self.platform(WAYLAND_DISPLAY="wayland-9", XDG_SESSION_TYPE="x11"),
                         "xcb")
        self.assertEqual(self.platform(), "xcb")


class OpenGLFormatTests(unittest.TestCase):
    def setUp(self):
        default = QSurfaceFormat.defaultFormat()
        configured = common._configured_for
        self.addCleanup(QSurfaceFormat.setDefaultFormat, default)
        self.addCleanup(setattr, common, "_configured_for", configured)

    def test_wayland_gets_opengl_es_and_the_rest_desktop_opengl(self):
        common.configure_opengl(["app", "-platform", "wayland"])
        es = QSurfaceFormat.defaultFormat()
        self.assertEqual((es.renderableType(), es.majorVersion()),
                         (QSurfaceFormat.RenderableType.OpenGLES, 3))
        common.configure_opengl(["app", "-platform", "xcb"])
        desktop = QSurfaceFormat.defaultFormat()
        self.assertEqual((desktop.majorVersion(), desktop.minorVersion(), desktop.profile()),
                         (3, 3, QSurfaceFormat.OpenGLContextProfile.CoreProfile))
        self.assertEqual(desktop.depthBufferSize(), 24)

    def test_a_platform_other_than_expected_is_reported(self):
        common._configured_for = "wayland"  # while the tests run offscreen
        with redirect_stderr(io.StringIO()) as err:
            common.warn_if_platform_differs()
        self.assertIn("OpenGL was set up for wayland, but Qt is running on offscreen",
                      err.getvalue())
        common._configured_for = "offscreen"
        with redirect_stderr(io.StringIO()) as err:
            common.warn_if_platform_differs()
        self.assertEqual(err.getvalue(), "")


if __name__ == "__main__":
    unittest.main()
