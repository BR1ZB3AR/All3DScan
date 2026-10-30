"""Tests of the 3D viewport (viewport3d.py): the camera's maths, and real
OpenGL renders read back pixel by pixel, drawn offscreen; and, in separate
processes, the OpenGL ES path Wayland takes and the core profile X11 gives.

Run from the repository root after `maturin develop`:

    .venv/bin/python -m unittest discover -s gui_python/tests
"""

import json
import os
import subprocess
import sys
import unittest
from pathlib import Path

# Offscreen unless the caller picks a platform, so no windows open on the desktop.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
GUI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GUI_DIR))

import numpy as np  # noqa: E402
from PyQt6.QtCore import QPoint, QPointF, Qt  # noqa: E402
from PyQt6.QtGui import QOpenGLContext, QWheelEvent  # noqa: E402
from PyQt6.QtTest import QSignalSpy, QTest  # noqa: E402
from PyQt6.QtWidgets import QApplication, QWidget  # noqa: E402

from common import apply_dark_theme, configure_opengl  # noqa: E402

if QApplication.instance() is None:
    configure_opengl()  # before the QApplication, which reads it
qt_app = QApplication.instance() or QApplication([])
apply_dark_theme(qt_app)

import viewport3d  # noqa: E402
from panels import Viewport  # noqa: E402
from viewport3d import SCAN_BLUE, OrbitCamera, PointCloudView, image_pixels  # noqa: E402

BLUE = tuple(round(c * 255) for c in SCAN_BLUE)


def wall(size_mm: float = 200.0, depth_mm: float = 400.0, centre=(0.0, 0.0), count: int = 120,
         colour=(220, 30, 30)):
    """A square wall facing the sensor, as points, colours and normals."""
    side = np.linspace(-size_mm / 2, size_mm / 2, count)
    grid = np.stack(np.meshgrid(side, side), -1).reshape(-1, 2) + centre
    points = np.c_[grid, np.full(len(grid), depth_mm)].astype(np.float32)
    colors = np.tile(colour, (len(points), 1)).astype(np.uint8)
    normals = np.tile([0.0, 0.0, -1.0], (len(points), 1)).astype(np.float32)
    return points, colors, normals


def walls():
    """A red wall 200 mm across at 400 mm, and a larger green one behind it at 600 mm."""
    near, far = wall(), wall(320.0, 600.0, colour=(30, 220, 30))
    return tuple(np.vstack(pair) for pair in zip(near, far))


def project(camera: OrbitCamera, point, aspect: float = 4 / 3) -> np.ndarray:
    """Where a point in the sensor's axes lands, in normalised device coordinates."""
    clip = camera.projection(aspect) @ camera.view_matrix() @ np.array(
        [point[0], -point[1], -point[2], 1.0])
    return clip[:3] / clip[3]


def close(actual, expected, tolerance: int = 3) -> bool:
    return all(abs(a - e) <= tolerance for a, e in zip(actual, expected))


class OrbitCameraTests(unittest.TestCase):
    def test_framing_shows_the_whole_cloud_from_the_sensors_side(self):
        camera = OrbitCamera()
        points, _, _ = walls()
        camera.frame(points)
        np.testing.assert_allclose(project(camera, (0.0, 0.0, 500.0))[:2], 0.0, atol=1e-9)
        np.testing.assert_allclose(camera.eye()[:2], 0.0, atol=1e-9)  # straight out from the sensor
        on_screen = np.array([project(camera, p) for p in points[::37]])
        self.assertTrue(np.all(np.abs(on_screen) <= 1.0))  # inside the view and the clip planes

    def test_orbit_move_and_zoom(self):
        camera = OrbitCamera()
        camera.frame(walls()[0])
        target, distance = camera.target.copy(), camera.distance
        camera.zoom(2)
        self.assertAlmostEqual(camera.distance, distance * 0.85 ** 2)
        camera.orbit(100, 0)
        self.assertNotEqual(camera.yaw, 0.0)
        self.assertAlmostEqual(float(np.linalg.norm(camera.eye() - camera.target)),
                               camera.distance)
        camera.orbit(0, 10_000)
        self.assertLessEqual(camera.pitch, 1.5)  # never tips over the top
        camera.pan(50, 0, 600)
        self.assertFalse(np.allclose(camera.target, target))
        self.assertTrue(np.all(np.isfinite(camera.projection(1.5) @ camera.view_matrix())))

    def test_the_clip_planes_follow_each_cloud_shown(self):
        camera = OrbitCamera()
        camera.frame(wall(depth_mm=300.0)[0])
        far = wall(depth_mm=900.0)[0]
        camera.show(far)  # the camera stays put, the clip planes move
        self.assertTrue(np.all(np.abs([project(camera, p) for p in far[::97]]) <= 1.0))

    def test_the_clip_planes_hold_the_cloud_from_any_side(self):
        camera = OrbitCamera()
        points = walls()[0]
        camera.frame(points)
        for yaw, pitch, steps in [(3.0, 0.0, 0), (1.2, 1.0, 6), (-2.0, -0.7, -4)]:
            camera.yaw, camera.pitch, camera.distance = yaw, pitch, 650.0 * 0.85 ** steps
            depths = np.array([project(camera, p)[2] for p in points[::53]])
            self.assertTrue(np.all(np.abs(depths) <= 1.0), (yaw, pitch, steps))


class RenderTestCase(unittest.TestCase):
    """Draws with a real OpenGL context; skipped where none can be made."""

    def setUp(self):
        self.view = PointCloudView()
        self.addCleanup(self.view.close)
        self.view.resize(320, 240)
        self.view.show()
        QTest.qWait(30)
        self.view.grabFramebuffer()
        if self.view.error or not self.view.isValid():
            self.skipTest(f"no OpenGL here: {self.view.error or 'no context'}")

    def grab(self) -> np.ndarray:
        return image_pixels(self.view.grabFramebuffer())

    def pixel(self, rgb: np.ndarray, fx: float = 0.5, fy: float = 0.5) -> tuple[int, int, int]:
        """The pixel a fraction of the way across and down the image; the
        image is the widget's size times the device pixel ratio."""
        height, width, _ = rgb.shape
        return tuple(int(c) for c in rgb[min(int(fy * height), height - 1),
                                         min(int(fx * width), width - 1)])


def brightness(pixel) -> float:
    return sum(pixel) / 3.0


def leans(pixel, channel: int, by: int = 40) -> bool:
    """Whether a shaded pixel is clearly that channel's colour.

    The lighting multiplies a colour by an amount that depends on which way
    the surface faces, and adds a white highlight, so a drawn pixel is never
    the colour that went in. What survives any lighting worth having is
    which channel leads."""
    others = [c for n, c in enumerate(pixel) if n != channel]
    return all(pixel[channel] - other > by for other in others)


class RenderTests(RenderTestCase):
    def test_no_color_draws_scan_blue_and_color_the_points_own(self):
        self.view.set_cloud(*walls())
        blue = self.pixel(self.grab())
        self.assertTrue(leans(blue, 2), blue)  # the scan's own blue
        self.view.set_color_mode(True)
        # The near red wall hides the green one behind it.
        red = self.pixel(self.grab())
        self.assertTrue(leans(red, 0), red)

    def test_colour_channels_stay_in_order(self):
        # Not the exact values, which the lighting decides, but that red
        # stays ahead of green and green ahead of blue: a channel swap
        # anywhere between numpy and the shader would show here.
        self.view.set_color_mode(True)
        self.view.set_cloud(*wall(colour=(220, 120, 30)))
        r, g, b = self.pixel(self.grab())
        self.assertGreater(r, g + 20, (r, g, b))
        self.assertGreater(g, b + 20, (r, g, b))

    def test_up_and_left_in_the_sensors_view_stay_up_and_left(self):
        # Sensor axes: x right, y down. A red patch up and left, a green one down and right.
        red = wall(20.0, 400.0, centre=(-60.0, -40.0), count=30, colour=(220, 30, 30))
        green = wall(20.0, 400.0, centre=(60.0, 40.0), count=30, colour=(30, 220, 30))
        self.view.set_color_mode(True)
        self.view.set_cloud(*(np.vstack(pair) for pair in zip(red, green)))
        rgb = self.grab()
        # Framed on both, 400 mm away: each lands 0.31 of the half-width and
        # 0.27 of the half-height from the middle. Which patch is where is
        # the point; the lighting decides how bright each one comes out.
        up_left = self.pixel(rgb, 0.5 - 0.155, 0.5 - 0.1375)
        down_right = self.pixel(rgb, 0.5 + 0.155, 0.5 + 0.1375)
        self.assertTrue(leans(up_left, 0), up_left)      # red, up and left
        self.assertTrue(leans(down_right, 1), down_right)  # green, down and right

    def test_surfaces_turned_away_are_shaded_darker(self):
        points, colors, _ = walls()
        facing = np.tile(np.array([0.0, 0.0, -1.0]), (len(points), 1))
        self.view.set_cloud(points, colors, facing.astype(np.float32))
        straight_on = self.pixel(self.grab())
        # The same wall turned 60 degrees from the viewer.
        turned = np.tile(np.array([0.0, -np.sqrt(3) / 2, -0.5]), (len(points), 1))
        self.view.set_cloud(points, colors, turned.astype(np.float32))
        away = self.pixel(self.grab())
        self.assertLess(brightness(away), brightness(straight_on),
                        (away, straight_on))
        # ...but still lit: a surface that goes black reads as a hole.
        self.assertGreater(brightness(away), 12, away)

    def test_the_same_view_is_drawn_the_same_way_twice(self):
        # The lighting is fixed to the viewer, so orbiting changes how a
        # surface is shaded — that is what shows its shape. What must not
        # change is the same view drawn again.
        self.view.set_cloud(*wall())
        for yaw in (np.radians(45), -np.radians(45)):
            self.view.camera.yaw = yaw
            self.view.update()
            once = self.pixel(self.grab())
            self.view.update()
            self.assertEqual(self.pixel(self.grab()), once, yaw)

    def test_a_curved_surface_is_shaded_across_its_curve(self):
        # A flat lamp at the eye shades everything facing the viewer alike,
        # which flattens a scan. The key light is off to one side so that
        # curvature reads as a gradient: this is that gradient.
        points, colors, _ = wall(60.0, 400.0, count=60)
        across = points[:, 0] / max(np.abs(points[:, 0]).max(), 1e-6)
        normals = np.stack([across, np.zeros_like(across),
                            -np.sqrt(np.clip(1 - across ** 2, 0, 1))], axis=1)
        self.view.set_cloud(points, colors, normals.astype(np.float32))
        rgb = self.grab()
        row = rgb[rgb.shape[0] // 2].astype(int)
        # The lit pixels: bluer than the grey background.
        lit = row[row[:, 2] - row[:, 0] > 20]
        self.assertGreater(len(lit), 20, "the curved wall was not drawn")
        spread = lit.sum(axis=1).max() - lit.sum(axis=1).min()
        self.assertGreater(spread / 3.0, 10,
                           f"a curved surface came out flat: spread {spread / 3.0:.1f}")

    def test_splats_are_sized_in_millimetres_up_to_a_cap(self):
        point = (np.array([[0.0, 0.0, 400.0]], np.float32), np.array([[220, 30, 30]], np.uint8),
                 np.array([[0.0, 0.0, -1.0]], np.float32))
        self.view.set_color_mode(True)
        self.view.set_cloud(*point)
        self.view.camera.distance = 400.0

        def width_px() -> int:
            # The splat's footprint: every pixel on the middle row that
            # differs from the background, which the row's first pixel is.
            # Its rim is soft, so "strongly red" would measure only the
            # solid middle; this measures the size it was given.
            rgb = self.grab()
            row = rgb[rgb.shape[0] // 2].astype(int)
            background = row[0]
            return int(np.sum(np.abs(row - background).max(axis=1) > 6))

        height_px = self.grab().shape[0]
        focal_px = height_px / (2 * np.tan(np.radians(OrbitCamera.FOV_Y) / 2))
        self.view.splat_mm = 10.0
        self.assertAlmostEqual(width_px(), focal_px * 10.0 / 400.0, delta=1.5)
        self.view.splat_mm = 100.0  # 82 px wide, uncapped
        ratio = self.view.devicePixelRatioF()
        self.assertAlmostEqual(width_px(), viewport3d.MAX_SPLAT_PX * ratio, delta=1.5)

    def test_a_far_cloud_after_a_near_one_is_not_clipped(self):
        self.view.set_cloud(*wall(depth_mm=300.0))
        self.grab()
        self.view.set_cloud(*wall(400.0, 900.0))  # not framed again: the camera stays
        seen = self.pixel(self.grab())
        self.assertTrue(leans(seen, 2), seen)

    def test_the_background_is_a_grey_gradient(self):
        rgb = self.grab()
        top, bottom = self.pixel(rgb, 0.5, 0.0), self.pixel(rgb, 0.5, 1.0)
        self.assertTrue(close(top, (59, 61, 66)), top)  # #3b3d42
        self.assertTrue(close(bottom, (43, 44, 48)), bottom)  # #2b2c30

    def test_clearing_leaves_only_the_background(self):
        self.view.set_cloud(*walls())
        self.grab()
        self.view.clear()
        rgb = self.grab()
        self.assertEqual((self.view.point_count, self.view.renderer.count), (0, 0))
        r, g, b = self.pixel(rgb)
        self.assertLess(max(r, g, b) - min(r, g, b), 8)  # grey: no point drawn

    def test_tinted_points_take_the_tint(self):
        points, colors, normals = wall()
        tint = np.zeros((len(points), 4), np.uint8)
        self.view.set_cloud(points, colors, normals, tint)
        untinted = self.pixel(self.grab())
        self.assertTrue(leans(untinted, 2), untinted)  # no alpha: still blue
        tint[:] = (245, 165, 36, 255)
        self.view.set_highlight(tint)
        tinted = self.pixel(self.grab())
        self.assertTrue(leans(tinted, 0), tinted)  # the tint is orange
        tint[:, 3] = 128  # half way between the tint and scan blue
        self.view.set_highlight(tint)
        half = self.pixel(self.grab())
        for channel in range(3):
            low, high = sorted((untinted[channel], tinted[channel]))
            self.assertTrue(low - 12 <= half[channel] <= high + 12,
                            f"channel {channel}: {half} is not between "
                            f"{untinted} and {tinted}")
        with self.assertRaises(ValueError):
            self.view.set_highlight(tint[:-1])

    def test_a_new_context_gets_the_cloud_again(self):
        self.view.set_cloud(*wall())
        self.grab()
        # What Qt does when it replaces the context: release, then initialise afresh.
        self.view._release()
        self.view.makeCurrent()
        self.view.initializeGL()
        self.view.doneCurrent()
        again = self.pixel(self.grab())
        self.assertTrue(leans(again, 2), again)

    def test_another_view_going_leaves_this_ones_context_current(self):
        # Python can collect a closed view while this one draws; Qt then has
        # the closed view release its GL objects, in its own context.
        other = PointCloudView()
        self.addCleanup(other.close)
        other.resize(64, 64)
        other.show()
        QTest.qWait(30)
        other.grabFramebuffer()
        self.view.makeCurrent()
        mine = QOpenGLContext.currentContext()
        other._release()
        self.assertIs(QOpenGLContext.currentContext(), mine)
        self.view.doneCurrent()

    def test_pyopengl_looks_functions_up_the_way_the_context_was_made(self):
        self.view.makeCurrent()
        made_with = "egl" if viewport3d.current_context_is_egl() else "glx"
        self.view.doneCurrent()
        self.assertEqual(os.environ["PYOPENGL_PLATFORM"], made_with)

    def test_drag_orbits_the_wheel_zooms_and_double_click_frames_again(self):
        self.view.set_cloud(*walls())
        QTest.mousePress(self.view, Qt.MouseButton.LeftButton, pos=QPoint(100, 100))
        QTest.mouseMove(self.view, QPoint(160, 110))
        QTest.mouseRelease(self.view, Qt.MouseButton.LeftButton, pos=QPoint(160, 110))
        self.assertNotEqual(self.view.camera.yaw, 0.0)

        distance = self.view.camera.distance
        wheel = QWheelEvent(QPointF(160, 120), QPointF(160, 120), QPoint(), QPoint(0, 120),
                            Qt.MouseButton.NoButton, Qt.KeyboardModifier.NoModifier,
                            Qt.ScrollPhase.NoScrollPhase, False)
        QApplication.sendEvent(self.view, wheel)
        self.assertLess(self.view.camera.distance, distance)

        QTest.mouseDClick(self.view, Qt.MouseButton.LeftButton, pos=QPoint(160, 120))
        self.assertEqual((self.view.camera.yaw, self.view.camera.pitch), (0.0, 0.0))


class FailureTests(unittest.TestCase):
    def test_a_view_that_cannot_start_says_why_and_stays_empty(self):
        viewport = Viewport()
        self.addCleanup(viewport.close)
        view = viewport.view3d

        def broken(es: bool) -> None:
            raise RuntimeError("the driver rejected the shaders")

        view.renderer.initialize = broken
        spy = QSignalSpy(view.failed)
        viewport.resize(320, 240)
        viewport.show()
        QTest.qWait(30)
        view.grabFramebuffer()  # draws nothing, and raises nothing
        self.assertEqual(view.error, "RuntimeError: the driver rejected the shaders")
        self.assertEqual(len(spy), 1)
        viewport.show_cloud(*wall())
        self.assertIn("3D view unavailable", viewport.message.text())
        self.assertFalse(viewport.message.isHidden())


class ViewportTests(unittest.TestCase):
    def test_huge_clouds_are_thinned_for_display_and_stay_aligned(self):
        viewport = Viewport()
        self.addCleanup(viewport.close)
        n = Viewport.MAX_SHOWN_POINTS * 2 + 10
        index = np.arange(n)
        points = np.c_[index, np.zeros(n), np.full(n, 400)].astype(np.float32)
        colors = np.c_[index % 251, index % 241, index % 239].astype(np.uint8)
        normals = np.tile([0.0, 0.0, -1.0], (n, 1)).astype(np.float32)
        tint = np.c_[index % 233, index % 229, index % 227, np.full(n, 255)].astype(np.uint8)
        viewport.show_cloud(points, colors, normals, tint)
        self.assertLessEqual(viewport.view3d.point_count, Viewport.MAX_SHOWN_POINTS)
        self.assertTrue(viewport.message.isHidden())
        shown_points, shown_colors, _, shown_tint = viewport.view3d._cloud
        kept = shown_points[:, 0].astype(int)  # each point still carries its own index
        np.testing.assert_array_equal(shown_colors, colors[kept])
        np.testing.assert_array_equal(shown_tint, tint[kept])
        viewport.set_highlight(tint[::-1].copy())  # a new tint, thinned the same way
        np.testing.assert_array_equal(viewport.view3d._cloud[3], tint[::-1][kept])


# Runs in a fresh process, where PyOpenGL is not loaded yet: makes a context of
# the asked-for kind on an offscreen surface, draws a wall into a framebuffer
# object with the viewport's renderer, and prints what came out.
CONTEXT_PROBE = """
import json, os, sys
sys.path.insert(0, sys.argv[2])
from PyQt6.QtGui import QGuiApplication, QOffscreenSurface, QOpenGLContext, QSurfaceFormat
from PyQt6.QtOpenGL import QOpenGLFramebufferObject, QOpenGLFramebufferObjectFormat
import numpy as np
surface_format = QSurfaceFormat()
if sys.argv[1] == "es":
    surface_format.setRenderableType(QSurfaceFormat.RenderableType.OpenGLES)
    surface_format.setVersion(3, 0)
else:
    surface_format.setVersion(3, 3)
    surface_format.setProfile(QSurfaceFormat.OpenGLContextProfile.CoreProfile)
surface_format.setDepthBufferSize(24)
app = QGuiApplication(sys.argv[:1])
context = QOpenGLContext()
context.setFormat(surface_format)
if not context.create():
    print(json.dumps({"error": "no context"})); sys.exit()
surface = QOffscreenSurface()
surface.setFormat(context.format())
surface.create()
context.makeCurrent(surface)
import viewport3d
GL = viewport3d.load_gl()
attachments = QOpenGLFramebufferObjectFormat()
attachments.setAttachment(QOpenGLFramebufferObject.Attachment.Depth)
fbo = QOpenGLFramebufferObject(160, 120, attachments)
fbo.bind()
renderer = viewport3d.PointCloudRenderer()
renderer.initialize(context.isOpenGLES())
side = np.linspace(-100, 100, 80)
grid = np.stack(np.meshgrid(side, side), -1).reshape(-1, 2)
points = np.c_[grid, np.full(len(grid), 400.0)].astype(np.float32)
renderer.set_cloud(points, np.zeros_like(points, np.uint8),
                   np.tile([0.0, 0.0, -1.0], (len(points), 1)).astype(np.float32))
camera = viewport3d.OrbitCamera()
camera.frame(points)
view = camera.view_matrix()
renderer.draw(160, 120, camera.projection(4 / 3) @ view, view[:3, :3], 300.0, 16.0, False,
              viewport3d.SCAN_BLUE)
GL.glFinish()
centre = fbo.toImage().pixelColor(80, 60)
print(json.dumps({"pyopengl": os.environ["PYOPENGL_PLATFORM"], "es": context.isOpenGLES(),
                  "profile": context.format().profile().name,
                  "centre": [centre.red(), centre.green(), centre.blue()]}))
"""


def probe_context(platform: str, kind: str) -> dict:
    env = dict(os.environ, QT_QPA_PLATFORM=platform)
    env.pop("PYOPENGL_PLATFORM", None)
    result = subprocess.run([sys.executable, "-c", CONTEXT_PROBE, kind, str(GUI_DIR)],
                            capture_output=True, text=True, timeout=60, env=env)
    lines = [line for line in result.stdout.splitlines() if line.startswith("{")]
    if result.returncode or not lines:
        raise AssertionError(f"probe failed ({result.returncode}):\n{result.stdout}"
                             f"{result.stderr}")
    return json.loads(lines[-1])


def wayland_socket() -> bool:
    display = os.environ.get("WAYLAND_DISPLAY")
    runtime = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
    return bool(display) and (runtime / display).exists()


class PlatformContextTests(unittest.TestCase):
    @unittest.skipUnless(wayland_socket(), "needs a Wayland session")
    def test_wayland_draws_with_opengl_es_through_egl(self):
        probe = probe_context("wayland", "es")
        self.assertEqual((probe.get("pyopengl"), probe.get("es")), ("egl", True), probe)
        self.assertTrue(leans(probe["centre"], 2), probe)

    @unittest.skipUnless(os.environ.get("DISPLAY"), "needs an X11 display")
    def test_x11_draws_with_a_core_profile_through_glx(self):
        # A core profile rejects GL_POINT_SPRITE, which the renderer must survive.
        probe = probe_context("xcb", "core")
        self.assertEqual((probe.get("pyopengl"), probe.get("profile")), ("glx", "CoreProfile"),
                         probe)
        self.assertTrue(leans(probe["centre"], 2), probe)


if __name__ == "__main__":
    unittest.main()
