"""Tests of the Revo Scan-style main window (app.py), drawn offscreen.

Run from the repository root after `maturin develop`:

    .venv/bin/python -m unittest discover -s gui_python/tests
"""

import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

# Offscreen unless the caller picks a platform, so no windows open on the desktop.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtTest import QSignalSpy, QTest
from PyQt6.QtWidgets import QApplication

GUI_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GUI_DIR))

from common import apply_dark_theme, configure_opengl  # noqa: E402

if QApplication.instance() is None:
    configure_opengl()  # before the QApplication, which reads it

import all3dscan_engine as engine  # noqa: E402
import app  # noqa: E402
import rea_engine as rea  # noqa: E402
import common  # noqa: E402
import theme  # noqa: E402
import viewport3d  # noqa: E402
import widgets  # noqa: E402
from panels import ScanState  # noqa: E402
from presets import load_presets  # noqa: E402
from widgets import DistanceGauge, center_distance_mm, zone_for  # noqa: E402

qt_app = QApplication.instance() or QApplication([])
apply_dark_theme(qt_app)


def wait_until(condition, timeout_s: float = 10.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError("timed out waiting for the window")
        QTest.qWait(5)


def items(combo) -> list[str]:
    return [combo.itemText(i) for i in range(combo.count())]


class FakePop4:
    """Stands in for engine.Pop4Sensor, so no test touches a real scanner:
    the mock sensor's frames, and the POP 4's controls, each call recorded."""

    serial = "TEST0001"
    backend = "Revopoint POP4 (fake)"
    last_error = "V4L2 frame timeout"

    def __init__(self, missing: int = 0) -> None:
        self._mock = engine.MockDepthSensor(160, 120, seed=1)
        self.missing = missing  # captures still to come back empty
        self.refuse: set[str] = set()  # controls that raise
        self.calls: list[tuple] = []
        self.lights = {"projector": [True, 180], "fill light": [False, 255]}
        self.scan_mode = "infrared"
        self.exposure_us = 3000
        self.closed = False
        self.connected = True  # False once the scanner has unplugged
        self.frames_taken = 0
        self.intrinsics = self._mock.intrinsics

    # How many points the scene shows at a given exposure; a flat scene by
    # default, so only the auto-exposure tests care.
    points_for_exposure = None

    def capture_frame(self, voxel_mm=None, timeout_ms=2000):
        if not self.connected:
            self.last_error = "the scanner disconnected (poll revents=0x18)"
            return None
        if self.missing:
            self.missing -= 1
            return None
        self.frames_taken += 1
        frame = self._mock.capture_frame(voxel_mm)
        if self.points_for_exposure is not None:
            frame = Thinned(frame, self.points_for_exposure(self.exposure_us))
        return frame

    def unplug(self) -> None:
        """What a POP 4 leaving the USB bus looks like to the app."""
        self.connected = False

    def light_on(self, light):
        return self.lights[light][0]

    def light_level(self, light):
        return self.lights[light][1]

    def __getattr__(self, name):
        # set_light, set_light_level, set_exposure, set_scan_mode, set_depth_range
        if not name.startswith("set_"):
            raise AttributeError(name)

        def control(*args):
            self.calls.append((name, args))
            if name in self.refuse:
                raise OSError(f"{name} was refused")
            if name == "set_light":
                self.lights[args[0]][0] = args[1]
            elif name == "set_exposure":
                self.exposure_us = args[0]
            elif name == "set_scan_mode":
                if args[0] == "infrared":
                    time.sleep(0.3)  # a reboot, much shortened
                self.scan_mode = args[0]

        return control

    def close(self):
        self.closed = True


class Thinned:
    """A frame cut down to a given number of points, standing in for a scene
    the scanner sees more or less of as the exposure changes."""

    def __init__(self, like, points: int) -> None:
        keep = max(min(points, len(like.points)), 0)
        self.points, self.colors = like.points[:keep], like.colors[:keep]
        self.normals, self.depth_mm = like.normals[:keep], like.depth_mm
        self.ir, self.rgb, self.index = like.ir, like.rgb, like.index


class BlankFrame:
    """A frame the scanner found no depth in: the images still arrive, with
    nothing in them, as they do when the object is out of range."""

    def __init__(self, like) -> None:
        self.depth_mm = np.zeros_like(like.depth_mm)
        self.ir, self.rgb, self.index = like.ir, like.rgb, like.index
        self.points = np.zeros((0, 3), np.float32)
        self.colors = np.zeros((0, 3), np.uint8)
        self.normals = np.zeros((0, 3), np.float32)


class WindowTestCase(unittest.TestCase):
    def open_window(self, sensor=None, pop4=None, present=True) -> app.MainWindow:
        """A window whose POP 4, when picked, is `pop4` (a FakePop4 by
        default): never the real scanner. self.present stands in for the USB
        bus, so a test can take the scanner off it."""
        self.pop4 = pop4 if pop4 is not None else FakePop4()
        self.present = present

        def open_pop4():
            if isinstance(self.pop4, BaseException):
                raise self.pop4
            return self.pop4

        window = app.MainWindow(sensor, open_pop4=open_pop4,
                                pop4_present=lambda: self.present)
        self.addCleanup(window.close)
        window.resize(1440, 900)
        window.show()
        QTest.qWait(20)
        return window

    def connect_pop4(self, window: app.MainWindow) -> None:
        window.scan_panel.device.setCurrentIndex(1)
        wait_until(lambda: window._pop4 and window.scan_panel.new_scan_button.isEnabled())

    def calls(self, name: str) -> list[tuple]:
        return [args for called, args in self.pop4.calls if called == name]


class LayoutTests(WindowTestCase):
    def test_viewport_sits_between_the_two_side_panels(self):
        window = self.open_window()
        self.assertIs(window.splitter.widget(1), window.viewport)
        left, middle, right = window.splitter.sizes()
        self.assertGreater(middle, 2 * max(left, right))

    def test_ribbon_groups(self):
        window = self.open_window()
        groups = {caption: [b.text() for b in group.buttons]
                  for caption, group in window.ribbon_groups.items()}
        self.assertEqual(groups, {
            "Scan": ["Feature scan", "Marker scan"],
            "Point cloud edit": ["Fusion", "Isolation", "Overlap", "Smooth", "Simplify"],
            "Mesh edit": ["Mesh", "Fill holes", "Smooth", "Simplify"],
            "REA": ["Fit plane", "Fit cylinder", "Export STEP"],
            "Output": ["Export", "Fullscreen"],
        })

    def test_editing_tools_wait_for_a_model(self):
        window = self.open_window()
        self.assertFalse(any(action.isEnabled() for action in window.needs_a_model))

    def test_pop4_lights_and_modes_follow_the_selected_device(self):
        window = self.open_window()
        camera, scan = window.camera_panel, window.scan_panel
        self.assertEqual([b.text() for b in (camera.projector_button, camera.fill_light_button)],
                         ["Pattern projector", "Flash light"])
        self.assertEqual(items(scan.mode), ["Infrared", "Blue laser (visible)", "Infrared speckle"])
        pop4_only = (camera.lights, camera.exposure_box, scan.mode)
        self.assertFalse(any(w.isEnabled() for w in pop4_only))  # the mock has none of them
        self.connect_pop4(window)
        self.assertTrue(all(w.isEnabled() for w in pop4_only))


class MenuTests(WindowTestCase):
    """The menu bar above the ribbon."""

    def menus(self, window) -> dict[str, list[str]]:
        bar = window.menuBar()
        found = {}
        for action in bar.actions():
            menu = action.menu()
            found[action.text()] = ([a.text() for a in menu.actions()] if menu else None)
        return found

    def test_the_menu_bar_holds_file_edit_theme_help_and_about(self):
        window = self.open_window()
        menus = self.menus(window)
        self.assertEqual(list(menus), ["&File", "&Edit", "&Theme", "&Help", "&About"])
        self.assertEqual(menus["&Theme"], ["Dark", "Light", "Custom…"])
        self.assertIsNone(menus["&About"])  # one thing to do, so no submenu
        self.assertIn("&Quit", menus["&File"])
        self.assertIn("Export STEP", menus["&File"])
        self.assertIn("Fusion", menus["&Edit"])
        self.assertIn("Fit plane", menus["&Edit"])
        self.assertIn("&Shortcuts…", menus["&Help"])
        # Exporting belongs to File; Edit does not repeat it.
        self.assertNotIn("Export STEP", menus["&Edit"])

    def test_the_menu_offers_the_ribbon_s_own_actions(self):
        # The same QAction in both places, so they cannot disagree about
        # what is available.
        window = self.open_window()
        menus = {a.text(): a.menu() for a in window.menuBar().actions()}
        edit = {a.text(): a for a in menus["&Edit"].actions()}
        self.assertIs(edit["Fit plane"], window.fit_plane_action)
        self.assertIs(edit["Fusion"], window.fusion_action)
        self.assertFalse(edit["Fit plane"].isEnabled())  # no model yet

    def test_the_scan_items_follow_the_scan(self):
        window = self.open_window()
        window._update_menus()
        self.assertTrue(window.new_scan_item.isEnabled())
        self.assertFalse(window.complete_item.isEnabled())
        window.scan_panel.new_scan()
        window._update_menus()
        self.assertFalse(window.new_scan_item.isEnabled())
        self.assertTrue(window.complete_item.isEnabled())
        self.assertTrue(window.cancel_item.isEnabled())

    def test_new_scan_from_the_menu_starts_one(self):
        window = self.open_window()
        window.new_scan_item.trigger()
        self.assertIs(window.scan_panel.state, ScanState.SCANNING)
        window.complete_item.trigger()
        self.assertIs(window.scan_panel.state, ScanState.IDLE)

    def test_quit_and_shortcuts_carry_their_keys(self):
        window = self.open_window()
        keys = {a.text(): a.shortcut().toString()
                for top in window.menuBar().actions() if top.menu()
                for a in top.menu().actions()}
        self.assertEqual(keys["&Quit"], "Ctrl+Q")
        self.assertEqual(keys["&Shortcuts…"], "F1")

    def test_the_dialogs_open_and_say_something(self):
        window = self.open_window()
        shown = []
        with mock.patch.object(app.QMessageBox, "information",
                               side_effect=lambda *a: shown.append(a[2])), \
             mock.patch.object(app.QMessageBox, "about",
                               side_effect=lambda *a: shown.append(a[2])):
            window._show_shortcuts()
            window._show_tips()
            window._show_about()
        self.assertIn("Space", shown[0])
        self.assertIn("250–500 mm", shown[1])  # the preset's band
        self.assertIn("All3DScan", shown[2])
        self.assertIn("mock depth camera", shown[2])


class ThemeTests(WindowTestCase):
    """Switching the window's colours."""

    def setUp(self):
        # Chosen themes are remembered, so put back whatever was set.
        was = common.current_theme()
        self.addCleanup(common.remember_theme, was)
        self.addCleanup(common.apply_theme, qt_app, common.theme_named(was))
        self.addCleanup(common.remember_custom_colours, common.custom_colours())

    def test_one_set_of_rules_recoloured_for_each_theme(self):
        dark, light = theme.DARK.stylesheet(), theme.LIGHT.stylesheet()
        self.assertNotEqual(dark, light)
        # Same rules, so the same number of lines and declarations.
        self.assertEqual(dark.count("{"), light.count("{"))
        self.assertEqual(len(dark.splitlines()), len(light.splitlines()))
        # No dark surface is left behind in the light sheet.
        for colour in (theme.WINDOW, theme.PANEL, theme.BASE, theme.TEXT, theme.EDGE):
            self.assertIn(theme.DARK.of(colour), dark)
            self.assertNotIn(theme.DARK.of(colour), light)
            self.assertIn(theme.LIGHT.of(colour), light)

    def test_a_theme_recolours_the_window_the_gauge_and_the_viewport(self):
        window = self.open_window()
        window.set_theme("Light")
        self.assertIn(theme.LIGHT.of(theme.WINDOW), qt_app.styleSheet())
        self.assertEqual(qt_app.palette().color(QPalette.ColorRole.Window).name(),
                         theme.LIGHT.of(theme.WINDOW))
        self.assertEqual(viewport3d.BACKGROUND_TOP, theme.LIGHT.rgb(theme.VIEWPORT_TOP))
        self.assertEqual(widgets._theme, theme.LIGHT)

        window.set_theme("Dark")
        self.assertIn(theme.DARK.of(theme.WINDOW), qt_app.styleSheet())
        self.assertEqual(viewport3d.BACKGROUND_TOP, theme.DARK.rgb(theme.VIEWPORT_TOP))

    def test_the_chosen_theme_is_ticked_and_remembered(self):
        window = self.open_window()
        window.set_theme("Light")
        ticked = [a.text() for a in window.theme_actions.actions() if a.isChecked()]
        self.assertEqual(ticked, ["Light"])
        self.assertEqual(common.current_theme(), "Light")
        # A window opened afterwards starts in it.
        later = self.open_window()
        self.assertEqual([a.text() for a in later.theme_actions.actions() if a.isChecked()],
                         ["Light"])

    def test_an_unknown_theme_is_ignored(self):
        window = self.open_window()
        window.set_theme("Light")
        window.set_theme("Chartreuse")
        self.assertEqual(common.current_theme(), "Light")

    def test_a_custom_theme_is_built_from_three_colours(self):
        made = theme.custom(surface="#1b2430", text="#e8eef6", accent="#2f9e68")
        self.assertEqual(made.of(theme.WINDOW), "#1b2430")
        self.assertEqual(made.of(theme.TEXT), "#e8eef6")
        self.assertEqual(made.of(theme.ACCENT), "#2f9e68")
        # The rest follow: panels sit a little off the surface, hints a
        # little back from the text, and the accent keeps its own shades.
        for name in (theme.PANEL, theme.BASE, theme.EDGE, theme.DIM, theme.ACCENT_HOVER):
            self.assertNotEqual(made.of(name), theme.DARK.of(name), name)
        self.assertNotIn(theme.DARK.of(theme.WINDOW), made.stylesheet())

    def test_the_default_colours_give_back_the_dark_theme(self):
        made = theme.custom(**theme.CUSTOM_DEFAULTS)
        for name in (theme.WINDOW, theme.PANEL, theme.TEXT, theme.ACCENT):
            self.assertEqual(QColor(made.of(name)).name(), QColor(theme.DARK.of(name)).name())

    def test_light_and_dark_surfaces_both_keep_their_hints_readable(self):
        # The offsets come from whichever theme matches the surface, so a
        # hint stays between the text and the surface either way.
        for surface, text in (("#1b2430", "#e8eef6"), ("#f6f1e8", "#241f18")):
            made = theme.custom(surface=surface, text=text, accent="#b4652a")
            lightness = [QColor(made.of(name)).lightnessF()
                         for name in (theme.TEXT, theme.DIM, theme.WINDOW)]
            self.assertEqual(sorted(lightness, reverse=surface == "#1b2430"), lightness,
                             f"{surface}: text {lightness[0]:.2f} dim {lightness[1]:.2f} "
                             f"surface {lightness[2]:.2f}")

    def test_choosing_custom_colours_shows_them_and_remembers(self):
        window = self.open_window()
        colours = {"surface": "#101820", "text": "#eef2f6", "accent": "#1f8a4c"}
        with mock.patch.object(app.CustomThemeDialog, "exec", return_value=1):
            dialog = app.CustomThemeDialog(colours)
            with mock.patch.object(app, "CustomThemeDialog", return_value=dialog):
                window.choose_custom_theme()
        self.assertEqual(common.current_theme(), theme.CUSTOM)
        self.assertEqual(common.custom_colours(), colours)
        self.assertIn("#101820", qt_app.styleSheet())
        ticked = [a.data() for a in window.theme_actions.actions() if a.isChecked()]
        self.assertEqual(ticked, [theme.CUSTOM])

    def test_cancelling_puts_the_old_theme_back(self):
        window = self.open_window()
        window.set_theme("Light")
        with mock.patch.object(app.CustomThemeDialog, "exec", return_value=0):
            dialog = app.CustomThemeDialog({"surface": "#101820", "text": "#eef2f6",
                                            "accent": "#1f8a4c"})
            with mock.patch.object(app, "CustomThemeDialog", return_value=dialog):
                window.choose_custom_theme()
        self.assertEqual(common.current_theme(), "Light")
        self.assertIn(theme.LIGHT.of(theme.WINDOW), qt_app.styleSheet())
        self.assertNotIn("#101820", qt_app.styleSheet())

    def test_the_dialog_previews_each_change(self):
        window = self.open_window()
        dialog = app.CustomThemeDialog(dict(theme.CUSTOM_DEFAULTS), window)
        dialog.changed.connect(lambda colours: window._wear_theme(theme.custom(**colours)))
        dialog.set_colour("accent", "#1f8a4c")
        self.assertIn("#1f8a4c", qt_app.styleSheet())  # on screen before OK
        dialog.reset()
        self.assertEqual(dialog.colours, dict(theme.CUSTOM_DEFAULTS))
        self.assertIn(theme.ACCENT, qt_app.styleSheet())

    def test_the_distance_bands_keep_their_meaning_in_every_theme(self):
        # Red for too near, green for good: those say something, so a theme
        # must not repaint them.
        for name in theme.THEMES:
            chosen = theme.THEMES[name]
            self.assertEqual(chosen.of(theme.ZONE_TOO_NEAR), theme.ZONE_TOO_NEAR)
            self.assertEqual(chosen.of(theme.ZONE_GOOD), theme.ZONE_GOOD)


class PresetTests(WindowTestCase):
    def test_presets_come_from_the_vendor_file(self):
        presets = load_presets()
        self.assertEqual(len(presets), 12)
        self.assertEqual({p.band_edges_mm for p in presets},
                         {(290, 350, 400, 460), (290, 400, 500, 600)})

    def test_settings_offer_only_combinations_the_presets_have(self):
        scan = self.open_window().scan_panel
        self.assertEqual(items(scan.tracking), ["Feature", "Marker", "Global marker", "Texture"])
        self.assertEqual(items(scan.object_type), ["General", "Dark", "Face"])
        scan.set_tracking("marker")
        self.assertEqual(items(scan.object_type), ["General"])
        scan.accuracy.setCurrentIndex(1)  # High speed
        self.assertEqual(items(scan.tracking), ["Feature", "Texture"])
        self.assertEqual(items(scan.object_type), ["Body", "Large object"])

    def test_gauge_exposure_and_ribbon_follow_the_preset(self):
        window = self.open_window()
        scan = window.scan_panel
        scan.accuracy.setCurrentIndex(1)  # High speed
        self.assertEqual(window.viewport.gauge.band_edges_mm, (290, 400, 500, 600))
        self.assertEqual(window.viewport.gauge.scale_mm, (250, 700))
        self.assertEqual(scan.scale_label.text(), "250 – 700 mm")
        self.assertEqual(window.camera_panel.exposure.value(), scan.preset.exposure_us)
        self.assertFalse(window.marker_scan.isEnabled())  # no marker presets at high speed
        scan.accuracy.setCurrentIndex(0)
        window.marker_scan.trigger()
        self.assertEqual(scan.tracking.currentData(), "marker")


class DistanceGaugeTests(unittest.TestCase):
    def test_band_edges(self):
        edges = (290, 350, 400, 460)
        for mm, zone in [(289.9, "too-near"), (290, "excellent"), (350, "good"),
                         (400, "far"), (460, "too-far"), (None, "idle")]:
            self.assertEqual(zone_for(mm, edges), zone, mm)

    def test_distance_is_the_median_of_the_middle(self):
        depth = np.full((100, 100), 600, np.uint16)
        depth[30:70, 30:70] = 330
        depth[50, 50] = 0  # a dropped pixel does not count
        self.assertEqual(center_distance_mm(depth), 330)
        self.assertIsNone(center_distance_mm(np.zeros((10, 10), np.uint16)))

    def test_gauge_takes_a_depth_image_and_draws_it(self):
        gauge = DistanceGauge()
        gauge.resize(170, 400)
        gauge.set_depths(np.full((48, 64), 320, np.uint16))
        self.assertEqual((gauge.distance_mm, gauge.zone), (320.0, "excellent"))
        self.assertFalse(gauge.grab().isNull())  # paints without raising
        gauge.set_depths(None)
        self.assertEqual(gauge.zone, "idle")


class ScanControlTests(WindowTestCase):
    def test_new_scan_pause_resume_complete(self):
        scan = self.open_window().scan_panel

        def buttons():
            return (scan.new_scan_button.isEnabled(), scan.pause_button.text(),
                    scan.pause_button.isEnabled(), scan.complete_button.isEnabled())

        self.assertEqual(buttons(), (True, "Pause", False, False))
        scan.new_scan_button.click()
        self.assertEqual(buttons(), (False, "Pause", True, True))
        scan.pause_button.click()
        self.assertEqual(buttons(), (False, "Resume", True, True))
        scan.pause_button.click()
        self.assertIs(scan.state, ScanState.SCANNING)
        scan.complete_button.click()
        self.assertEqual(buttons(), (True, "Pause", False, False))

    def test_space_starts_and_pauses_and_esc_cancels(self):
        window = self.open_window()
        scan = window.scan_panel
        window.space_shortcut.activated.emit()
        self.assertIs(scan.state, ScanState.SCANNING)
        window.space_shortcut.activated.emit()
        self.assertIs(scan.state, ScanState.PAUSED)
        window.escape_shortcut.activated.emit()
        self.assertIs(scan.state, ScanState.IDLE)
        self.assertEqual(window.model_panel.models.count(), 0)  # a cancelled scan is dropped

    def test_status_bar_shows_connection_and_scan_state(self):
        window = self.open_window()
        self.assertIn("Mock depth sensor connected", window._connection.text())
        self.assertEqual(window.scan_state.text(), "Ready")
        window.scan_panel.new_scan()
        self.assertEqual(window.scan_state.text(), "Scanning")
        self.assertEqual(window.statusBar().currentMessage(), "")  # nothing drawn over them

    def test_settings_are_fixed_while_scanning(self):
        scan = self.open_window().scan_panel
        scan.new_scan()
        self.assertFalse(scan.accuracy.isEnabled() or scan.device.isEnabled())
        scan.complete()
        self.assertTrue(scan.accuracy.isEnabled() and scan.device.isEnabled())


class StreamingTests(WindowTestCase):
    def test_a_scan_streams_into_the_feeds_and_the_gauge(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 3)
        self.assertFalse(window.camera_panel.depth_view.image.pixmap().isNull())
        self.assertFalse(window.camera_panel.rgb_view.image.pixmap().isNull())
        self.assertIsNotNone(window.viewport.gauge.distance_mm)
        self.assertRegex(window.viewport.frames.text(), r"^Rendering frames: \d+$")
        self.assertGreater(window.viewport.view3d.point_count, 0)
        self.assertTrue(window.viewport.message.isHidden())

    def test_the_3d_view_draws_the_live_cloud(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)
        cloud, tinted = app.cloud_pixels(window)
        self.assertGreater(cloud, 20_000)  # most of the view: the wall, ball and pipe
        self.assertEqual(tinted, 0)

    def test_a_3d_view_that_cannot_start_says_so(self):
        window = self.open_window()
        window.viewport.view3d._fail("no OpenGL driver")
        self.assertIn("3D view unavailable", window.viewport.message.text())
        self.assertFalse(window.viewport.message.isHidden())

    def test_the_color_switch_reaches_the_3d_view(self):
        window = self.open_window()
        self.assertFalse(window.viewport.view3d.use_color)
        window.viewport.color_button.click()
        self.assertTrue(window.viewport.view3d.use_color)
        window.viewport.no_color_button.click()
        self.assertFalse(window.viewport.view3d.use_color)

    def test_cancelling_clears_the_3d_view(self):
        window = self.open_window(engine.MockDepthSensor(64, 48, seed=1))
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)
        window.scan_panel.cancel()
        self.assertEqual(window.viewport.view3d.point_count, 0)
        self.assertFalse(window.viewport.message.isHidden())

    def test_pausing_stops_the_stream(self):
        window = self.open_window(engine.MockDepthSensor(64, 48, seed=1))
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)
        window.scan_panel.toggle_pause()
        QTest.qWait(100)  # let a frame already on its way arrive
        frames = window.frames
        QTest.qWait(250)
        self.assertEqual(window.frames, frames)

    def test_completing_lists_the_model_with_its_properties(self):
        window = self.open_window(engine.MockDepthSensor(64, 48, seed=1))
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)
        window.scan_panel.complete()
        models = window.model_panel
        self.assertEqual(models.models.count(), 1)
        self.assertIs(window.left.currentWidget(), models)
        self.assertEqual(models.value("Model name"), "Scan 01")
        self.assertEqual(models.value("Scan settings"), "High accuracy · Feature tracking · General")
        self.assertEqual(models.value("Rendering frames"), str(window.frames))
        self.assertTrue(window.camera_panel.depth_view.image.pixmap().isNull())  # feeds cleared
        self.assertGreater(window.viewport.view3d.point_count, 0)  # the model stays in view

    def test_the_window_keeps_responding_while_it_scans(self):
        sensor = engine.MockDepthSensor(2048, 1536, seed=3)
        start = time.perf_counter()
        sensor.capture_frame()
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
        beat["last"] = time.perf_counter()
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 4, timeout_s=30)
        window.scan_panel.toggle_pause()
        timer.stop()
        self.assertLess(
            beat["longest"], capture_s / 2,
            f"the GUI thread stalled {beat['longest']:.3f}s; a capture takes {capture_s:.3f}s",
        )


class Pop4Tests(WindowTestCase):
    """Picking the POP 4 and driving it from the window, with a FakePop4."""

    def test_it_can_be_picked_only_while_plugged_in(self):
        present = {"now": False}
        window = app.MainWindow(open_pop4=FakePop4, pop4_present=lambda: present["now"])
        self.addCleanup(window.close)
        item = window.scan_panel.device.model().item(1)
        self.assertFalse(item.isEnabled())
        present["now"] = True
        window._check_pop4_present()
        self.assertTrue(item.isEnabled())

    def test_picking_it_connects_and_sets_it_up_from_the_preset(self):
        window = self.open_window()
        scan = window.scan_panel
        scan.device.setCurrentIndex(1)
        self.assertFalse(scan.new_scan_button.isEnabled())  # not until it is connected
        self.assertIn("Connecting to the POP 4", window._connection.text())
        wait_until(lambda: window._pop4 and scan.new_scan_button.isEnabled())
        self.assertIn("Revopoint POP 4 TEST0001 connected", window._connection.text())
        wait_until(lambda: self.calls("set_depth_range") and self.calls("set_exposure"))
        self.assertEqual(self.calls("set_depth_range")[-1], (250.0, 500.0))  # the preset's range
        self.assertEqual(self.calls("set_exposure")[-1], (scan.preset.exposure_us,))
        self.assertTrue(window.camera_panel.projector_button.isChecked())  # as the scanner has it

    def test_lights_and_exposure_reach_the_scanner(self):
        window = self.open_window()
        self.connect_pop4(window)
        camera = window.camera_panel
        camera.projector_button.click()
        camera.fill_light_button.click()
        camera.fill_light_level.setValue(100)
        camera.exposure.setValue(4000)
        wait_until(lambda: self.calls("set_light_level") and (4000,) in self.calls("set_exposure"))
        self.assertEqual(self.calls("set_light"),
                         [("projector", False), ("fill light", True)])
        self.assertEqual(self.calls("set_light_level"), [("fill light", 100)])

    def test_auto_exposure_keeps_the_setting_that_sees_most(self):
        window = self.open_window()
        self.connect_pop4(window)
        # The fake sees more of the object the longer the exposure, up to a
        # peak part way through the range, as a real scene does.
        best = 3500
        self.pop4.points_for_exposure = lambda us: 9000 - abs(us - best) // 2
        window.camera_panel.auto_exposure.click()
        self.assertTrue(window._tuning)
        self.assertFalse(window.camera_panel.exposure_box.isEnabled())  # busy
        wait_until(lambda: not window._tuning, timeout_s=60)

        self.assertEqual(self.pop4.exposure_us, best)
        self.assertEqual(window.camera_panel.exposure.value(), best)
        self.assertTrue(window.camera_panel.exposure_box.isEnabled())
        self.assertIn("3.5 ms", window.viewport.toast.text())
        # Every exposure in the sweep was tried, in order, and the best one
        # put back at the end. Earlier calls are the preset's, sent on connect.
        sweep = list(app.ScanWorker.EXPOSURES)
        tried = [args[0] for name, args in self.pop4.calls if name == "set_exposure"]
        self.assertEqual(tried[-len(sweep) - 1 : -1], sweep)
        self.assertEqual(tried[-1], best)

    def test_exposure_writes_are_spaced_out(self):
        # Two writes in a row stop a real POP 4 sending frames at all, so
        # frames must pass between them.
        window = self.open_window()
        self.connect_pop4(window)
        wait_until(lambda: ("set_exposure", (3000,)) in self.pop4.calls)
        taken = self.pop4.frames_taken
        window.camera_panel.exposure.setValue(4200)
        wait_until(lambda: ("set_exposure", (4200,)) in self.pop4.calls, timeout_s=30)
        self.assertGreaterEqual(self.pop4.frames_taken - taken,
                                app.ScanWorker.EXPOSURE_GAP_FRAMES)

    def test_auto_exposure_reports_a_scanner_that_refuses_everything(self):
        window = self.open_window()
        self.connect_pop4(window)
        self.pop4.refuse.add("set_exposure")
        window.camera_panel.auto_exposure.click()
        wait_until(lambda: not window._tuning, timeout_s=60)
        self.assertIn("Auto exposure failed", window.viewport.toast.text())
        self.assertTrue(window.camera_panel.exposure_box.isEnabled())

    def test_auto_exposure_rides_out_a_setting_the_scanner_refuses(self):
        # The scanner refuses a settings write for a while after connecting
        # resets it. One refusal must not lose the whole sweep.
        window = self.open_window()
        self.connect_pop4(window)
        first = app.ScanWorker.EXPOSURES[0]
        best = app.ScanWorker.EXPOSURES[-1]
        self.pop4.points_for_exposure = lambda us: 2000 + us // 10
        original = self.pop4.set_exposure if hasattr(self.pop4, "_set") else None
        refused = {"once": True}

        def flaky(us):
            if us == first and refused["once"]:
                refused["once"] = False
                raise OSError("POP4 not found while applying CS7 settings")
            self.pop4.exposure_us = us
            self.pop4.calls.append(("set_exposure", (us,)))

        self.pop4.set_exposure = flaky
        window.camera_panel.auto_exposure.click()
        wait_until(lambda: not window._tuning, timeout_s=60)
        self.assertEqual(self.pop4.exposure_us, best)  # the rest still decided it
        self.assertIn(f"{best / 1000:.1f} ms", window.viewport.toast.text())

    def test_the_white_lights_flash_while_a_scan_runs(self):
        window = self.open_window()
        self.connect_pop4(window)
        camera = window.camera_panel
        self.assertEqual(camera.fill_light_button.text(), "Flash light")

        # Armed while idle, the lights simply come on, so you can see them.
        camera.fill_light_button.click()
        self.assertFalse(window._flash.isActive())
        wait_until(lambda: self.pop4.lights["fill light"][0])

        before = len(self.pop4.calls)
        window.scan_panel.new_scan()
        self.assertTrue(window._flash.isActive())
        wait_until(lambda: len([a for n, a in self.pop4.calls[before:]
                                if n == "set_light" and a[0] == "fill light"]) >= 4,
                   timeout_s=30)
        flashes = [a[1] for n, a in self.pop4.calls[before:]
                   if n == "set_light" and a[0] == "fill light"]
        # On, off, on, off: not left sitting in one state.
        self.assertEqual(flashes[:4], [True, False, True, False])

    def test_the_lights_go_out_when_the_scan_stops(self):
        window = self.open_window()
        self.connect_pop4(window)
        window.camera_panel.fill_light_button.click()
        window.scan_panel.new_scan()
        wait_until(lambda: window._flash_on, timeout_s=30)  # catch them lit
        window.scan_panel.complete()
        self.assertFalse(window._flash.isActive())
        wait_until(lambda: not self.pop4.lights["fill light"][0])
        self.assertFalse(window._flash_on)

    def test_the_projector_still_switches_steadily(self):
        # Only the white lights flash; the projector is a plain switch.
        window = self.open_window()
        self.connect_pop4(window)
        window.camera_panel.projector_button.click()
        wait_until(lambda: ("set_light", ("projector", False)) in self.pop4.calls)
        self.assertFalse(window._flash.isActive())

    def test_a_refused_control_says_so(self):
        window = self.open_window()
        self.connect_pop4(window)
        self.pop4.refuse.add("set_light")
        window.camera_panel.fill_light_button.click()
        wait_until(lambda: "refused" in window.viewport.toast.text())
        self.assertIn("The POP 4 refused set light: OSError: set_light was refused",
                      window.viewport.toast.text())

    def test_a_scan_streams_from_it_through_a_missed_frame_or_two(self):
        window = self.open_window(pop4=FakePop4(missing=2))
        self.connect_pop4(window)
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 3)
        self.assertIs(window.scan_panel.state, ScanState.SCANNING)
        self.assertGreater(window.viewport.view3d.point_count, 0)

    def test_almost_no_depth_is_reported_with_what_to_check(self):
        window = self.open_window()
        self.connect_pop4(window)
        # A scanner seeing nothing: no pixel comes back with a distance.
        full = self.pop4._mock.capture_frame()
        blank = BlankFrame(full)
        self.pop4.capture_frame = lambda voxel_mm=None, timeout_ms=2000: blank
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= app.MainWindow.LOW_COVERAGE_FRAMES)
        self.assertEqual(window._coverage.text(), "Coverage: 0.0%")
        warning = window.viewport.warning
        self.assertFalse(warning.isHidden())
        self.assertIn("Only 0 points a frame", warning.text())
        self.assertIn("250–500 mm away", warning.text())  # the preset's distance band
        self.assertIn("too dark", warning.text())

    def test_a_scanner_seeing_the_object_says_nothing(self):
        window = self.open_window()
        self.connect_pop4(window)
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames > app.MainWindow.LOW_COVERAGE_FRAMES)
        self.assertRegex(window._coverage.text(), r"^Coverage: 9\d\.\d%$")  # the mock's ~98%
        self.assertTrue(window.viewport.warning.isHidden())

    def test_a_small_object_filling_little_of_the_frame_is_not_a_complaint(self):
        # Measured on a real POP 4: a 60 mm ring at 360 mm covered 4.7% of the
        # depth camera and scanned perfectly well. Judging by coverage alone
        # would have called that a failure.
        window = self.open_window()
        self.connect_pop4(window)
        small = self.pop4._mock.capture_frame()
        keep = np.zeros(small.depth_mm.shape, bool)
        keep[:24, :40] = True  # a corner of the view, about 5% of it
        small.depth_mm[~keep] = 0
        self.pop4.capture_frame = lambda voxel_mm=None, timeout_ms=2000: small
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames > app.MainWindow.LOW_COVERAGE_FRAMES + 2)
        self.assertRegex(window._coverage.text(), r"^Coverage: [1-9]\.\d%$")
        self.assertTrue(window.viewport.warning.isHidden())

    def test_a_scanner_that_sends_nothing_stops_the_scan(self):
        window = self.open_window(pop4=FakePop4(missing=100))
        self.connect_pop4(window)
        window.scan_panel.new_scan()
        wait_until(lambda: window.scan_panel.state is ScanState.IDLE)
        self.assertIn("no frames from the POP 4 (V4L2 frame timeout)",
                      window.viewport.toast.text())

    def test_blue_laser_and_back_to_infrared(self):
        window = self.open_window()
        self.connect_pop4(window)
        scan = window.scan_panel
        scan.mode.setCurrentIndex(1)
        scan.mode.activated.emit(1)
        wait_until(lambda: "Scan mode: Blue laser" in window.viewport.toast.text())
        self.assertEqual(self.calls("set_scan_mode"), [("blue laser",)])

        scan.mode.setCurrentIndex(0)
        scan.mode.activated.emit(0)
        self.assertIn("Restarting the scanner into infrared (about 45 s)",
                      window.viewport.toast.text())
        self.assertFalse(scan.new_scan_button.isEnabled())  # not while it reboots
        wait_until(lambda: scan.new_scan_button.isEnabled())
        self.assertEqual(self.pop4.scan_mode, "infrared")
        self.assertEqual(scan.mode.currentData(), "infrared")

    def test_a_scanner_that_unplugs_mid_scan_says_so(self):
        window = self.open_window()
        self.connect_pop4(window)
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)
        self.pop4.unplug()
        wait_until(lambda: not window._pop4)
        # The status must not keep claiming a scanner that has gone.
        self.assertIn("disconnected", window._connection.text())
        self.assertNotIn("POP 4 TEST0001 connected", window._connection.text())
        # The heading beside the scanner list must not claim one either.
        self.assertEqual(window.scan_panel.connection.text(), "Disconnected")
        self.assertFalse(window.scan_panel.connection_ok)
        self.assertIs(window.scan_panel.state, ScanState.IDLE)
        self.assertFalse(window.camera_panel.lights.isEnabled())
        self.assertIn("The POP 4 disconnected", window.viewport.warning.text())
        self.assertTrue(self.pop4.closed)

    def test_it_reconnects_once_the_scanner_comes_back(self):
        window = self.open_window()
        self.connect_pop4(window)
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)
        self.pop4.unplug()
        wait_until(lambda: not window._pop4)
        # The scanner returns, as it does after every session.
        self.pop4 = FakePop4()
        wait_until(lambda: window._pop4, timeout_s=30)
        self.assertIn("Revopoint POP 4 TEST0001 connected", window._connection.text())
        self.assertEqual(window.scan_panel.connection.text(), "Connected")
        self.assertTrue(window.scan_panel.connection_ok)
        self.assertTrue(window.viewport.warning.isHidden())
        self.assertTrue(window.camera_panel.lights.isEnabled())
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)

    def test_it_gives_up_after_enough_failed_reconnects(self):
        window = self.open_window()
        self.connect_pop4(window)
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)
        self.pop4.unplug()
        wait_until(lambda: not window._pop4)
        self.pop4 = OSError("POP 4: no 640x2560 capture node")  # never opens again
        wait_until(lambda: window.scan_panel.device.currentIndex() == 0, timeout_s=60)
        wait_until(lambda: "Mock depth sensor connected" in window._connection.text())
        self.assertEqual(window.scan_panel.connection.text(), "Connected")
        self.assertTrue(window.viewport.warning.isHidden())

    def test_unplugging_between_scans_is_noticed_too(self):
        window = self.open_window()
        self.connect_pop4(window)
        self.present = False  # off the USB bus, with nothing capturing
        wait_until(lambda: not window._pop4, timeout_s=60)
        self.assertIn("disconnected", window._connection.text())
        self.assertIn("no longer on the USB bus", window.viewport.warning.text())

    def test_a_scanner_re_enumerating_is_not_called_gone(self):
        # Connecting resets the scanner, so it leaves the bus and comes back
        # about twelve seconds later. One empty look must not tear the
        # session down, or every connection would end in a false alarm.
        window = self.open_window()
        self.connect_pop4(window)
        self.present = False
        for _ in range(app.MainWindow.ABSENT_LOOKS - 1):
            window._check_pop4_present()
        self.assertTrue(window._pop4, "gone after a few empty looks")
        self.present = True
        window._check_pop4_present()
        self.assertEqual(window._absent_ticks, 0)  # back before it counted
        self.assertTrue(window._pop4)
        self.assertIn("TEST0001 connected", window._connection.text())

    def test_a_scanner_that_will_not_open_leaves_the_mock_in_use(self):
        window = self.open_window(pop4=OSError("POP 4: no 640x2560 capture node"))
        window.scan_panel.device.setCurrentIndex(1)
        wait_until(lambda: window.scan_panel.device.currentIndex() == 0
                   and window.scan_panel.new_scan_button.isEnabled())
        self.assertIn("Could not open the POP 4: OSError: POP 4: no 640x2560 capture node",
                      window.viewport.toast.text())
        self.assertIn("Mock depth sensor connected", window._connection.text())
        self.assertFalse(window.camera_panel.lights.isEnabled())

    def test_going_back_to_the_mock_or_closing_lets_it_go(self):
        window = self.open_window()
        self.connect_pop4(window)
        window.scan_panel.device.setCurrentIndex(0)
        wait_until(lambda: self.pop4.closed and not window._pop4)
        second = FakePop4()
        self.pop4 = second
        self.connect_pop4(window)
        window.close()
        self.assertTrue(second.closed)


class FusionTests(WindowTestCase):
    """Building one model out of a scan's frames."""

    def scan(self, window, frames: int = 4) -> None:
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= frames)

    def test_a_scan_fuses_its_frames_into_one_model(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        self.assertTrue(window.fusion_action.isChecked())  # on by default
        self.scan(window)
        wait_until(lambda: window._tracked >= 2, timeout_s=30)
        self.assertRegex(window._tracking.text(), r"^Tracking: [1-9]\d*/\d+$")
        # The model holds more than any one frame does.
        self.assertGreater(len(window._fused[0]), len(window.last_frame.points))
        self.assertEqual(window.viewport.view3d.point_count, len(window._fused[0]))

        window.scan_panel.complete()
        model = window.model_panel.current_model()
        self.assertEqual(len(model.points), len(window._fused[0]))
        self.assertRegex(model.properties["Points"], r"^[\d,]+ fused from \d+ frames$")
        # Real normals come through the fusion, which the cylinder fit needs.
        self.assertEqual(model.normals.shape, model.points.shape)
        lengths = np.linalg.norm(model.normals, axis=1)
        np.testing.assert_allclose(lengths, 1.0, atol=1e-3)

    def test_completing_takes_the_stray_points_out(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        self.scan(window)
        wait_until(lambda: window._tracked >= 2, timeout_s=30)
        fused = len(window._fused[0])
        # A handful of specks hanging in mid-air, as a bad depth match leaves.
        strays = np.array([[300.0 + 40 * i, -250.0, 250.0 + 30 * i] for i in range(9)],
                          np.float32)
        window._fused = (np.vstack([window._fused[0], strays]),
                         np.vstack([window._fused[1],
                                    np.zeros((len(strays), 3), np.uint8)]),
                         np.vstack([window._fused[2],
                                    np.tile([0.0, 0.0, -1.0], (len(strays), 1)).astype(
                                        np.float32)]))
        window.scan_panel.complete()
        model = window.model_panel.current_model()
        # Every speck is gone, and the surface is all but untouched: an
        # outlier filter also trims points at the cloud's edges, which have
        # fewer neighbours by nature.
        for speck in strays:
            nearest = np.linalg.norm(model.points - speck, axis=1).min()
            self.assertGreater(nearest, 20.0, f"a speck survived at {speck}")
        self.assertGreater(len(model.points), fused * 0.99)
        self.assertLessEqual(len(model.points), fused)
        self.assertEqual(len(model.colors), len(model.points))
        self.assertEqual(len(model.normals), len(model.points))
        self.assertIn("stray points", window.viewport.toast.text())
        self.assertRegex(model.properties["Points"], r"^[\d,]+ fused from \d+ frames$")

    def test_a_cloud_that_is_all_speckle_is_kept_as_it_is(self):
        # Rather than hand back an empty model, which would lose the scan.
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        self.scan(window)
        wait_until(lambda: window._tracked >= 1, timeout_s=30)
        scattered = np.array([[200.0 * i, 150.0 * i, 300.0 + 50 * i] for i in range(20)],
                             np.float32)
        window._fused = (scattered, np.zeros((20, 3), np.uint8),
                         np.tile([0.0, 0.0, -1.0], (20, 1)).astype(np.float32))
        window.scan_panel.complete()
        self.assertEqual(len(window.model_panel.current_model().points), 20)

    def test_a_scan_builds_a_surface_from_its_depth_views(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        self.scan(window, frames=10)
        wait_until(lambda: window._tracked >= 3, timeout_s=60)
        window.scan_panel.complete()
        model = window.model_panel.current_model()
        self.assertTrue(model.intrinsics, "the scan must remember its camera")
        self.assertTrue(window.mesh_action.isEnabled())

        window.mesh_action.trigger()
        self.assertTrue(window._meshing)
        self.assertFalse(window.mesh_action.isEnabled())  # busy
        wait_until(lambda: not window._meshing, timeout_s=120)

        self.assertIsNotNone(model.surface, "no surface was built")
        vertices, normals, indices = model.surface
        self.assertGreater(len(indices), 100)
        self.assertEqual(normals.shape, vertices.shape)
        self.assertLess(indices.max(), len(vertices))
        np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1.0, atol=1e-3)
        # The surface stands in the viewport in place of its points.
        self.assertEqual(window.viewport.view3d.triangle_count, len(indices))
        self.assertTrue(window.mesh_action.isEnabled())

    def test_showing_a_cloud_puts_away_the_surface(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        view = window.viewport.view3d
        vertices = np.array([[0, 0, 400], [20, 0, 400], [0, 20, 400]], np.float32)
        normals = np.tile([0.0, 0.0, -1.0], (3, 1)).astype(np.float32)
        view.set_surface(vertices, normals, np.array([[0, 1, 2]], np.uint32))
        self.assertEqual(view.triangle_count, 1)
        self.scan(window, frames=2)
        self.assertEqual(view.triangle_count, 0, "a cloud should replace a surface")

    def test_a_surface_with_a_broken_triangle_is_refused(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        view = window.viewport.view3d
        vertices = np.zeros((3, 3), np.float32)
        normals = np.zeros((3, 3), np.float32)
        with self.assertRaises(ValueError):
            view.set_surface(vertices, normals, np.array([[0, 1, 7]], np.uint32))
        with self.assertRaises(ValueError):
            view.set_surface(vertices, np.zeros((2, 3), np.float32),
                             np.array([[0, 1, 2]], np.uint32))

    def test_the_points_shown_are_the_models_while_fusing(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        self.scan(window)
        wait_until(lambda: window._tracked >= 2, timeout_s=30)
        # Frames keep arriving between fusions; the count must not flick back
        # to one frame's worth.
        frames = window.frames
        wait_until(lambda: window.frames > frames + 2)
        shown = int(window._points.text().removeprefix("Points: ").replace(",", ""))
        self.assertEqual(shown, len(window._fused[0]))

    def test_a_frame_that_will_not_align_does_not_stall_the_fusion(self):
        # The worker must report back even when a frame cannot be placed:
        # the window waits on that before offering another, so staying quiet
        # would stop the fusion for the rest of the scan.
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        self.scan(window)
        wait_until(lambda: window._tracked >= 1, timeout_s=30)

        spy = QSignalSpy(window._fusion_worker.model_ready)
        window._fusion_worker.model_ready.emit(None, None, {"merged": False})
        self.assertFalse(window._fusing)  # free to take the next frame
        offered = window._offered
        wait_until(lambda: window._offered > offered)
        wait_until(lambda: window._tracked >= 2, timeout_s=30)
        self.assertGreaterEqual(len(spy), 1)

    def test_the_cloud_is_green_while_scanning_and_blue_once_complete(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        view = window.viewport.view3d
        self.scan(window)
        self.assertEqual(view.base_color, viewport3d.SCAN_GREEN)
        window.scan_panel.complete()
        self.assertEqual(view.base_color, viewport3d.SCAN_BLUE)

    def test_with_fusion_off_the_model_is_the_last_frame(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        window.fusion_action.setChecked(False)
        self.scan(window)
        self.assertIsNone(window._fused)
        self.assertEqual(window._tracking.text(), "Tracking: —")
        self.assertEqual(window.viewport.view3d.point_count, len(window.last_frame.points))
        window.scan_panel.complete()
        model = window.model_panel.current_model()
        np.testing.assert_array_equal(model.points, window.last_frame.points)
        self.assertRegex(model.properties["Points"], r"in the last frame$")

    def test_each_scan_starts_its_model_afresh(self):
        window = self.open_window(engine.MockDepthSensor(160, 120, seed=1))
        self.scan(window)
        wait_until(lambda: window._tracked >= 2, timeout_s=30)
        window.scan_panel.complete()
        first = len(window.model_panel.current_model().points)

        window.scan_panel.new_scan()
        self.assertIsNone(window._fused)  # not carried over from the last scan
        self.assertEqual((window._tracked, window._offered), (0, 0))
        wait_until(lambda: window._tracked >= 2, timeout_s=30)
        window.scan_panel.complete()
        second = len(window.model_panel.current_model().points)
        self.assertLess(abs(second - first), first)  # the same scene, not twice it

    def test_fusion_runs_off_the_gui_thread(self):
        window = self.open_window(engine.MockDepthSensor(640, 480, seed=1))
        self.assertIs(window._fusion_worker.thread(), window._fusion_thread)
        beat = {"last": time.perf_counter(), "longest": 0.0}

        def tick():
            now = time.perf_counter()
            beat["longest"] = max(beat["longest"], now - beat["last"])
            beat["last"] = now

        timer = QTimer()
        timer.timeout.connect(tick)
        timer.start(2)
        beat["last"] = time.perf_counter()
        window.scan_panel.new_scan()
        wait_until(lambda: window._tracked >= 2, timeout_s=60)
        window.scan_panel.toggle_pause()
        timer.stop()
        # Fusing a 640x480 frame takes well over 100 ms; the window must not
        # stop for it.
        self.assertLess(beat["longest"], 0.1, f"the GUI thread stalled {beat['longest']:.3f}s")


class ReaToolTests(WindowTestCase):
    """The REA tools in the window, on the mock sensor's wall and pipe."""

    def scanned_window(self, width: int = 320, height: int = 240) -> app.MainWindow:
        window = self.open_window(engine.MockDepthSensor(width, height, seed=1))
        # Fusion off: these are about the fits, and whether a fusion had
        # finished by the time a two-frame scan completes is a race, which
        # would make the model — and so every measurement here — vary.
        window.fusion_action.setChecked(False)
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 2)
        window.scan_panel.complete()
        return window

    def fit(self, window: app.MainWindow, kind: str):
        model = window.model_panel.current_model()
        count = len(model.session.primitives)
        window.fit(kind)
        self.assertTrue(window._fitting)
        wait_until(lambda: not window._fitting)
        added = model.session.primitives[count:]
        return added[0] if added else None

    def test_the_tools_wait_for_a_completed_scan(self):
        window = self.open_window(engine.MockDepthSensor(64, 48, seed=1))
        tools = (window.fit_plane_action, window.fit_cylinder_action, window.export_step_action)
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 1)
        self.assertFalse(any(tool.isEnabled() for tool in tools))
        window.scan_panel.complete()
        self.assertEqual([tool.isEnabled() for tool in tools], [True, True, False])

    def test_fitting_lists_the_primitive_and_tints_its_points(self):
        window = self.scanned_window()
        wall = self.fit(window, "plane")
        np.testing.assert_allclose(wall.normal, [0, 0, -1], atol=1e-2)
        panel = window.model_panel
        self.assertEqual(panel.primitives.count(), 1)
        self.assertTrue(panel.primitives.item(0).text().startswith("Plane 1  ·  "))
        self.assertIs(panel.current_primitive(), wall)
        self.assertIn("Normal", panel.primitive_details.text())
        self.assertTrue(window.export_step_action.isEnabled())
        self.assertIn("Plane 1", window.viewport.toast.text())
        _, tinted = app.cloud_pixels(window)
        self.assertGreater(tinted, 20_000)  # the wall, in amber

        pipe = self.fit(window, "cylinder")
        self.assertAlmostEqual(pipe.radius_mm, 40.0, delta=1.0)
        self.assertEqual(panel.primitives.count(), 2)

    def test_the_window_keeps_responding_while_it_fits(self):
        window = self.scanned_window(640, 480)
        session = window.model_panel.current_model().session
        start = time.perf_counter()
        rea.fit_plane(session.points, session.normals)
        fit_s = time.perf_counter() - start
        if fit_s < 0.03:
            self.skipTest(f"a fit took {fit_s:.3f}s, too quick to tell a stall apart")
        beat = {"last": time.perf_counter(), "longest": 0.0}

        def tick():
            now = time.perf_counter()
            beat["longest"] = max(beat["longest"], now - beat["last"])
            beat["last"] = now

        timer = QTimer()
        timer.timeout.connect(tick)
        timer.start(2)
        beat["last"] = time.perf_counter()
        self.fit(window, "plane")
        timer.stop()
        self.assertIs(window._rea_worker.thread(), window._rea_thread)
        self.assertLess(beat["longest"], fit_s / 2,
                        f"the GUI thread stalled {beat['longest']:.3f}s; a fit takes {fit_s:.3f}s")

    def test_when_nothing_is_left_to_fit_it_says_so(self):
        window = self.scanned_window()
        self.fit(window, "plane")
        self.fit(window, "cylinder")
        self.assertIsNone(self.fit(window, "plane"))
        self.assertIn("No plane found", window.viewport.toast.text())
        self.assertTrue(window.fit_plane_action.isEnabled())

    def test_removing_a_primitive_frees_its_points(self):
        window = self.scanned_window()
        session = window.model_panel.current_model().session
        self.fit(window, "plane")
        window.model_panel.remove_button.click()
        self.assertEqual(window.model_panel.primitives.count(), 0)
        self.assertEqual(len(session.unclaimed()), len(session.points))
        self.assertFalse(window.export_step_action.isEnabled())
        self.assertEqual(app.cloud_pixels(window)[1], 0)  # no tint left

    def test_export_step_writes_the_fitted_primitives(self):
        window = self.scanned_window()
        pipe, wall = self.fit(window, "cylinder"), self.fit(window, "plane")
        with tempfile.TemporaryDirectory() as folder:
            written = window.export_step(Path(folder) / "part.step")
            bodies = rea.read_step(written)
        self.assertEqual([(name, colour) for name, _, colour in bodies],
                         [(pipe.name, pipe.color), (wall.name, wall.color)])
        self.assertIn("Exported 2 primitives to part.step", window.viewport.toast.text())

    def test_export_asks_where_and_adds_the_extension(self):
        window = self.scanned_window()
        self.fit(window, "plane")
        with tempfile.TemporaryDirectory() as folder:
            answer = (str(Path(folder) / "part"), "STEP files (*.step *.stp)")
            with mock.patch.object(app.QFileDialog, "getSaveFileName", return_value=answer) as ask:
                written = window.export_step()
            self.assertEqual(written, Path(folder) / "part.step")
            self.assertTrue(written.exists())
            self.assertTrue(ask.call_args.args[2].endswith("Scan 01.step"))  # the suggested name
        with mock.patch.object(app.QFileDialog, "getSaveFileName", return_value=("", "")):
            self.assertIsNone(window.export_step())  # cancelled

    def test_each_model_keeps_its_own_cloud_and_fits(self):
        window = self.scanned_window()
        first = window.model_panel.current_model()
        self.fit(window, "plane")
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 5)
        window.scan_panel.complete()
        second = window.model_panel.current_model()
        self.assertIsNot(first, second)
        self.assertEqual(window.model_panel.primitives.count(), 0)

        window.model_panel.models.setCurrentRow(0)
        self.assertIs(window.model_panel.current_model(), first)
        self.assertEqual(window.viewport.view3d.point_count, len(first.points))
        np.testing.assert_array_equal(window.viewport.view3d._cloud[0], first.points)
        self.assertEqual(window.model_panel.primitives.count(), 1)
        self.assertGreater(app.cloud_pixels(window)[1], 0)
        window.model_panel.models.setCurrentRow(1)
        np.testing.assert_array_equal(window.viewport.view3d._cloud[0], second.points)

    def test_cancelling_a_new_scan_goes_back_to_the_models(self):
        window = self.scanned_window()
        model = window.model_panel.current_model()
        window.scan_panel.new_scan()
        wait_until(lambda: window.frames >= 1)
        window.scan_panel.cancel()
        self.assertIs(window.left.currentWidget(), window.model_panel)
        np.testing.assert_array_equal(window.viewport.view3d._cloud[0], model.points)
        self.assertTrue(window.fit_plane_action.isEnabled())


class SmokeTestModeTests(unittest.TestCase):
    def test_scans_fits_exports_and_exits_cleanly(self):
        # Also guards the exit path: a still-running QThread aborts the process.
        result = subprocess.run([sys.executable, str(GUI_DIR / "app.py"), "--smoke-test"],
                                capture_output=True, text=True, timeout=120)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("previews filled", result.stdout)
        self.assertRegex(result.stdout,
                         r"3D view showing [1-9][\d,]* points \([1-9][\d,]* pixels of cloud")
        self.assertIn("model listed: True", result.stdout)
        self.assertRegex(result.stdout, r"fitted Plane 1: \d+ × \d+ mm; Cylinder 1: ⌀ 79\.\d mm")
        self.assertIn("STEP bodies read back: ['Cylinder 1', 'Plane 1']", result.stdout)


if __name__ == "__main__":
    unittest.main()
