"""All3DScan main window, laid out like Revo Scan 6.

A ribbon of tools runs along the top. The left panel holds the camera
parameters and the live depth and colour feeds while scanning, and the models
and their properties once a scan is complete. The 3D viewport fills the
middle, with the scanning-distance gauge; the scanner, New scan / Pause /
Complete and the scan settings sit on the right.

Frames come from the sensor on a background QThread, one at a time, so the
window stays responsive. Once a scan is complete, the REA tools fit planes and
cylinders to its point cloud, on another background thread, and export them
to STEP (rea_engine.py):

    .venv/bin/python gui_python/app.py
    .venv/bin/python gui_python/app.py --smoke-test   # scan, complete, fit, export, exit
"""

import argparse
import sys
import tempfile
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PyQt6.QtCore import QObject, Qt, QThread, QTimer, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QAction, QActionGroup, QIcon, QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication,
    QFileDialog,
    QLabel,
    QMainWindow,
    QSplitter,
    QStackedWidget,
    QToolBar,
)

import all3dscan_engine as engine
import rea_engine as rea
from common import APP_ICON, APP_ID, apply_dark_theme, configure_opengl, warn_if_platform_differs
from panels import CameraPanel, ModelPanel, ScanModel, ScanPanel, ScanState, Viewport, tint
from viewport3d import image_pixels
from widgets import RibbonGroup


def describe(exc: BaseException) -> str:
    return f"{type(exc).__name__}: {exc}"


class ScanWorker(QObject):
    """Owns the sensor on the thread it is moved to: captures frames, one per
    request, so that frames never pile up behind a busy window; opens and
    closes the POP 4; and runs its controls between captures, so the window
    never waits on the scanner. Its slots let nothing escape: catching
    BaseException, because a Rust panic arrives as PanicException, which is
    not an Exception, and anything raised in a slot ends the app."""

    frame_ready = pyqtSignal(object, float)  # engine.Frame, seconds taken
    frame_missed = pyqtSignal(str)  # no frame came in time: why
    failed = pyqtSignal(str)
    sensor_changed = pyqtSignal(dict)  # what the sensor now in use is and how it is set
    sensor_lost = pyqtSignal(str)  # the POP 4 went away mid-session; why
    open_failed = pyqtSignal(str)
    control_done = pyqtSignal(str, str)  # the control, and why it failed or ""

    def __init__(self, sensor, open_pop4) -> None:
        super().__init__()
        self._mock = self._sensor = sensor
        self._open_pop4 = open_pop4

    @pyqtSlot()
    def capture(self) -> None:
        start = time.perf_counter()
        try:
            frame = self._sensor.capture_frame()
        except BaseException as exc:
            if self._gone():
                self._lost(describe(exc))
            else:
                self.failed.emit(describe(exc))
            return
        if frame is None:  # the POP 4 sent nothing in time
            if self._gone():
                self._lost(self._sensor.last_error or "the scanner disconnected")
            else:
                self.frame_missed.emit(self._sensor.last_error or "no frame came")
        else:
            self.frame_ready.emit(frame, time.perf_counter() - start)

    def _gone(self) -> bool:
        """Whether the POP 4 in use has unplugged or re-enumerated."""
        try:
            return self._sensor is not self._mock and not self._sensor.connected
        except BaseException:
            return True

    def _lost(self, reason: str) -> None:
        self.close()  # back to the mock, so the window keeps working
        self.sensor_lost.emit(reason)

    @pyqtSlot()
    def open_pop4(self) -> None:
        try:
            pop4 = self._open_pop4()
            settings = self._settings(pop4)
        except BaseException as exc:
            self.open_failed.emit(describe(exc))
            return
        self.close()
        self._sensor = pop4
        self.sensor_changed.emit(settings)

    @pyqtSlot()
    def use_mock(self) -> None:
        self.close()
        self._sensor = self._mock
        self.sensor_changed.emit({"kind": "mock"})

    @pyqtSlot(str, tuple)
    def control(self, name: str, args: tuple) -> None:
        """Calls the POP 4's method `name` with args, if the POP 4 is in use."""
        if self._sensor is self._mock:
            return
        try:
            getattr(self._sensor, name)(*args)
        except BaseException as exc:
            self.control_done.emit(name, describe(exc))
            return
        self.control_done.emit(name, "")

    def close(self) -> None:
        """Lets the POP 4 go, if it is in use. Also called on the GUI thread
        once this one has stopped."""
        if self._sensor is not self._mock:
            try:
                self._sensor.close()
            except BaseException as exc:
                print(f"closing the POP 4 failed: {describe(exc)}", file=sys.stderr, flush=True)
            self._sensor = self._mock

    @staticmethod
    def _settings(pop4) -> dict:
        lights = {name: (pop4.light_on(name), pop4.light_level(name))
                  for name in ("projector", "fill light")}
        return {"kind": "pop4", "serial": pop4.serial, "backend": pop4.backend,
                "mode": pop4.scan_mode, "exposure_us": pop4.exposure_us, "lights": lights}


@dataclass(frozen=True, eq=False)
class FitJob:
    """One fit for the REA worker: a plane or a cylinder, fitted to the free
    points of a model's cloud as they were when it was asked for."""

    kind: str  # "plane" or "cylinder"
    model: ScanModel
    candidates: np.ndarray
    tolerance_mm: float
    seed: int

    def run(self):
        fit = rea.fit_plane if self.kind == "plane" else rea.fit_cylinder
        session = self.model.session
        return fit(session.points, session.normals, self.candidates,
                   tolerance_mm=self.tolerance_mm, seed=self.seed)


class ReaWorker(QObject):
    """Runs fits on the thread it is moved to. The engine releases the GIL
    while it fits, so the window keeps responding."""

    fitted = pyqtSignal(object, object, float)  # FitJob, the primitive or None, seconds
    failed = pyqtSignal(object, str)  # FitJob, what went wrong

    @pyqtSlot(object)
    def fit(self, job: FitJob) -> None:
        start = time.perf_counter()
        try:
            found = job.run()
        except BaseException as exc:  # as in ScanWorker.capture()
            self.failed.emit(job, f"{type(exc).__name__}: {exc}")
            return
        self.fitted.emit(job, found, time.perf_counter() - start)


class MainWindow(QMainWindow):
    frame_requested = pyqtSignal()
    pop4_requested = pyqtSignal()
    mock_requested = pyqtSignal()
    control_requested = pyqtSignal(str, tuple)  # a POP 4 method and its arguments
    fit_requested = pyqtSignal(object)  # FitJob
    TARGET_FPS = 30
    # Captures in a row that may bring no frame before a scan stops: each
    # waits up to two seconds, so this is about six seconds of silence.
    MAX_MISSED_FRAMES = 3
    # Below this share of pixels with depth, the scanner is not really seeing
    # the object: good infrared scanning measured 27.8% (RE_NOTES 2026-10-16),
    # and a scene too far or too dark gives under 1%.
    LOW_COVERAGE = 0.05
    # Frames in a row under it before saying so, so a glance away is ignored.
    LOW_COVERAGE_FRAMES = 8
    # After the scanner goes away mid-session it usually comes back within a
    # few seconds, so try this many times before giving up on it. Each try
    # waits for two consecutive looks that find one, about four seconds.
    MAX_RECONNECTS = 4

    def __init__(self, sensor=None, open_pop4=None, pop4_present=None) -> None:
        """sensor is the mock to scan with until a POP 4 is picked;
        open_pop4() opens a POP 4 and pop4_present() says if one is plugged
        in (engine.Pop4Sensor and engine.pop4_present unless given)."""
        super().__init__()
        self.setWindowTitle("All3DScan")
        self.resize(1440, 900)
        self.setMinimumSize(1100, 680)
        self.sensor = sensor if sensor is not None else engine.MockDepthSensor(640, 480, seed=0)

        self.camera_panel = CameraPanel()
        self.model_panel = ModelPanel()
        self.left = QStackedWidget()
        self.left.addWidget(self.camera_panel)
        self.left.addWidget(self.model_panel)
        self.left.setMinimumWidth(260)
        self.viewport = Viewport()
        self.scan_panel = ScanPanel()
        self.scan_panel.setMinimumWidth(270)
        for side in (self.left, self.scan_panel):
            side.setObjectName("sidePanel")
            side.setAttribute(Qt.WidgetAttribute.WA_StyledBackground)
        self.splitter = QSplitter(Qt.Orientation.Horizontal)
        for widget in (self.left, self.viewport, self.scan_panel):
            self.splitter.addWidget(widget)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([300, 840, 300])
        self.splitter.setChildrenCollapsible(False)
        self.setCentralWidget(self.splitter)
        self._build_ribbon()

        # Labels rather than showMessage(), which Qt draws over the bar's widgets.
        self._connection = QLabel()
        self.scan_state = QLabel(ScanState.IDLE.value)
        self._fps = QLabel("FPS: —")
        self._points = QLabel("Points: —")
        self._coverage = QLabel("Coverage: —")
        self._coverage.setToolTip("The share of the depth camera's pixels that got a distance")
        self.statusBar().addWidget(self._connection)
        self.statusBar().addWidget(self.scan_state)
        for label in (self._fps, self._points, self._coverage):
            self.statusBar().addPermanentWidget(label)

        self._state = ScanState.IDLE
        self._streaming = False
        self._in_flight = False
        self._requested_at = 0.0
        self._last_arrival: float | None = None
        self._rate = 0.0
        self.frames = 0
        self.last_frame = None
        self._scans = 0
        self._next_frame = QTimer(self)
        self._next_frame.setSingleShot(True)
        self._next_frame.timeout.connect(self._request_frame)

        self._missed = 0  # captures in a row that brought no frame
        self._low_coverage = 0  # frames in a row with almost no depth
        self._pop4 = False  # whether the POP 4 is the sensor in use
        self._connecting = False  # an open is on its way to the worker
        self._reconnects = 0  # tries since the scanner was lost
        self._present_ticks = 0  # consecutive looks that found a POP 4
        self._scan_mode = "infrared"  # the POP 4's, as last confirmed
        self._scan_mode_asked = "infrared"
        self._thread = QThread(self)
        self._worker = ScanWorker(self.sensor, open_pop4 or engine.Pop4Sensor)
        self._worker.moveToThread(self._thread)
        self._thread.finished.connect(self._worker.deleteLater)
        self.frame_requested.connect(self._worker.capture)
        self.pop4_requested.connect(self._worker.open_pop4)
        self.mock_requested.connect(self._worker.use_mock)
        self.control_requested.connect(self._worker.control)
        self._worker.frame_ready.connect(self._on_frame)
        self._worker.frame_missed.connect(self._on_frame_missed)
        self._worker.failed.connect(self._on_failed)
        self._worker.sensor_changed.connect(self._on_sensor_changed)
        self._worker.sensor_lost.connect(self._on_sensor_lost)
        self._worker.open_failed.connect(self._on_open_failed)
        self._worker.control_done.connect(self._on_control_done)
        self._thread.start()

        self._fitting = False
        self._shown_model: ScanModel | None = None  # the model in the 3D view
        self._rea_thread = QThread(self)
        self._rea_worker = ReaWorker()
        self._rea_worker.moveToThread(self._rea_thread)
        self._rea_thread.finished.connect(self._rea_worker.deleteLater)
        self.fit_requested.connect(self._rea_worker.fit)
        self._rea_worker.fitted.connect(self._on_fitted)
        self._rea_worker.failed.connect(self._on_fit_failed)
        self._rea_thread.start()
        QApplication.instance().aboutToQuit.connect(self._stop_worker)

        models = self.model_panel
        models.model_changed.connect(self._on_model_changed)
        models.primitive_changed.connect(lambda _primitive: self._show_fits())
        models.remove_requested.connect(self._remove_primitive)

        panel = self.scan_panel
        panel.state_changed.connect(self._on_scan_state)
        panel.completed.connect(self._on_completed)
        panel.cancelled.connect(self._on_cancelled)
        panel.preset_changed.connect(self._on_preset)
        panel.device_changed.connect(self._on_device)
        panel.mode_changed.connect(self._on_mode_picked)
        camera = self.camera_panel
        camera.light_toggled.connect(lambda light, on: self._control("set_light", light, on))
        camera.light_level_changed.connect(
            lambda light, level: self._control("set_light_level", light, level))
        camera.exposure_changed.connect(lambda us: self._control("set_exposure", us))
        self._on_preset(panel.preset)
        self._show_connection(False)

        # The POP 4 can be picked while one is plugged in; look now and then.
        self._pop4_present = pop4_present or engine.pop4_present
        self._presence = QTimer(self)
        self._presence.timeout.connect(self._check_pop4_present)
        self._presence.start(2000)
        self._check_pop4_present()

        self.space_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Space), self)
        self.space_shortcut.activated.connect(panel.toggle)
        self.escape_shortcut = QShortcut(QKeySequence(Qt.Key.Key_Escape), self)
        self.escape_shortcut.activated.connect(panel.cancel)

    def _build_ribbon(self) -> None:
        ribbon = QToolBar("Ribbon")
        ribbon.setObjectName("ribbon")
        ribbon.setMovable(False)
        ribbon.setFloatable(False)
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, ribbon)

        def action(text: str, icon: str, tip: str, enabled: bool = True,
                   checkable: bool = False) -> QAction:
            item = QAction(QIcon.fromTheme(icon), text, self)
            item.setToolTip(tip)
            item.setEnabled(enabled)
            item.setCheckable(checkable)
            return item

        self.feature_scan = action("Feature scan", "camera-photo",
                                   "Track the object by its own shape", checkable=True)
        self.marker_scan = action("Marker scan", "crosshairs",
                                  "Track stick-on markers", checkable=True)
        scan_type = QActionGroup(self)
        scan_type.addAction(self.feature_scan)
        scan_type.addAction(self.marker_scan)
        self.feature_scan.triggered.connect(lambda: self.scan_panel.set_tracking("feature"))
        self.marker_scan.triggered.connect(lambda: self.scan_panel.set_tracking("marker"))

        later = "Not available yet"
        point_cloud = [
            action("Fusion", "draw-polygon", later, enabled=False),
            action("Isolation", "edit-delete", later, enabled=False),
            action("Overlap", "edit-select-all", later, enabled=False),
            action("Smooth", "smooth", later, enabled=False),
            action("Simplify", "transform-scale", later, enabled=False),
        ]
        mesh = [
            action("Mesh", "draw-cuboid", later, enabled=False),
            action("Fill holes", "fill-color", later, enabled=False),
            action("Smooth", "draw-brush", later, enabled=False),
            action("Simplify", "zoom-select", later, enabled=False),
        ]
        self.fit_plane_action = action("Fit plane", "draw-line",
                                       "Fit a plane to the model's free points", enabled=False)
        self.fit_cylinder_action = action("Fit cylinder", "draw-circle",
                                          "Fit a cylinder to the model's free points",
                                          enabled=False)
        self.export_step_action = action("Export STEP", "document-export",
                                         "Save the fitted primitives as exact CAD geometry",
                                         enabled=False)
        self.fit_plane_action.triggered.connect(lambda: self.fit("plane"))
        self.fit_cylinder_action.triggered.connect(lambda: self.fit("cylinder"))
        self.export_step_action.triggered.connect(lambda: self.export_step())
        rea_tools = [self.fit_plane_action, self.fit_cylinder_action, self.export_step_action]
        self.export_action = action("Export", "document-save", later, enabled=False)
        self.fullscreen_action = action("Fullscreen", "view-fullscreen", "Shift+M",
                                        checkable=True)
        self.fullscreen_action.setShortcut(QKeySequence("Shift+M"))
        self.fullscreen_action.toggled.connect(
            lambda on: self.showFullScreen() if on else self.showNormal())
        self.addAction(self.fullscreen_action)  # so Shift+M works anywhere in the window

        self.ribbon_groups = {}
        for caption, actions in (
            ("Scan", [self.feature_scan, self.marker_scan]),
            ("Point cloud edit", point_cloud),
            ("Mesh edit", mesh),
            ("REA", rea_tools),
            ("Output", [self.export_action, self.fullscreen_action]),
        ):
            self.ribbon_groups[caption] = RibbonGroup(caption, actions)
            ribbon.addWidget(self.ribbon_groups[caption])
        self.needs_a_model = point_cloud + mesh + rea_tools + [self.export_action]

    # Scanning: each frame is asked for when the last one has been shown.

    def _on_scan_state(self, state: ScanState) -> None:
        previous, self._state = self._state, state
        self.scan_state.setText(state.value)
        if state is ScanState.SCANNING:
            self._missed = 0
            if previous is ScanState.IDLE:
                self._start_scan()
            self._streaming = True
            self._request_frame()
        else:
            self._streaming = False
            self._next_frame.stop()
            if state is ScanState.PAUSED:
                self.viewport.show_toast("Paused. Press Space to resume.")
        self._update_rea_tools()

    def _start_scan(self) -> None:
        self._low_coverage = 0
        self.viewport.show_warning("")
        self.frames = 0
        self.last_frame = None
        self._shown_model = None
        self._rate = 0.0
        self._last_arrival = None
        self.left.setCurrentWidget(self.camera_panel)
        self.viewport.set_stage(scanning=True)
        self.viewport.set_frame_count(0)
        self.viewport.clear_cloud("Scanning…")

    def _request_frame(self) -> None:
        if self._streaming and not self._in_flight:
            self._in_flight = True
            self._requested_at = time.perf_counter()
            self.frame_requested.emit()

    def _on_frame(self, frame, _seconds: float) -> None:
        self._in_flight = False
        self._missed = 0
        if self._state is ScanState.IDLE:
            return  # the frame that was on its way when the scan ended
        self.frames += 1
        self.last_frame = frame
        self.camera_panel.depth_view.show_image(frame.ir)
        self.camera_panel.rgb_view.show_image(frame.rgb)
        self.viewport.gauge.set_depths(frame.depth_mm)
        self.viewport.set_frame_count(self.frames)
        self.viewport.show_cloud(frame.points, frame.colors, frame.normals)
        self._show_coverage(frame)
        now = time.perf_counter()
        if self._last_arrival is not None:
            rate = 1.0 / max(now - self._last_arrival, 1e-6)
            self._rate = rate if self._rate == 0 else 0.9 * self._rate + 0.1 * rate
            self._fps.setText(f"FPS: {self._rate:.0f}")
        self._last_arrival = now
        self._points.setText(f"Points: {len(frame.points):,}")
        if self._streaming:
            wait = 1.0 / self.TARGET_FPS - (now - self._requested_at)
            self._next_frame.start(max(0, int(wait * 1000)))

    def _show_coverage(self, frame) -> None:
        """Reports how much of the depth camera got a distance, and says so
        when the scanner is getting almost nothing."""
        coverage = float(np.mean(frame.depth_mm > 0))
        self._coverage.setText(f"Coverage: {coverage:.1%}")
        if coverage >= self.LOW_COVERAGE:
            self._low_coverage = 0
            self.viewport.show_warning("")
            return
        self._low_coverage += 1
        if self._low_coverage == self.LOW_COVERAGE_FRAMES:
            near, far = self.scan_panel.preset.scale_mm
            self.viewport.show_warning(
                f"Almost no depth ({coverage:.1%} of the depth camera). Check that the object "
                f"is {near}–{far} mm away — the distance this scan setting looks in — and that "
                f"it is not too dark or too shiny for the scanner's pattern to show on it."
                if self._pop4 else f"Almost no depth ({coverage:.1%} of the depth camera).")

    def _on_frame_missed(self, reason: str) -> None:
        self._in_flight = False
        if self._state is ScanState.IDLE:
            return
        self._missed += 1
        if self._missed >= self.MAX_MISSED_FRAMES:
            self._on_failed(f"no frames from the POP 4 ({reason})")
        elif self._streaming:
            self._request_frame()

    def _on_failed(self, message: str) -> None:
        self._in_flight = False
        print(f"capture failed: {message}", file=sys.stderr, flush=True)
        self.scan_panel.cancel()
        self.viewport.show_toast(f"Capture failed: {message}", 6000)

    def _on_completed(self) -> None:
        self._scans += 1
        name = f"Scan {self._scans:02d}"
        panel = self.scan_panel
        frame = self.last_frame
        points = frame.points if frame is not None else None
        properties = {
            "Model name": name,
            "Scanner": panel.device.currentText(),
            "Rendering state": "Raw",
            "Rendering frames": str(self.frames),
            "Points": f"{len(points):,} in the last frame" if points is not None else "—",
            "Scan settings": f"{panel.accuracy.currentText()} · "
                             f"{panel.tracking.currentText()} tracking · "
                             f"{panel.object_type.currentText()}",
        }
        # The model is the last frame's cloud, which stays in the 3D view.
        model = ScanModel(name, properties, *((frame.points, frame.colors, frame.normals)
                                               if frame is not None else ()))
        self._shown_model = model  # already on screen: no need to draw it again
        self.left.setCurrentWidget(self.model_panel)
        self.viewport.set_stage(scanning=False)
        self.model_panel.add_model(model)
        if frame is None:
            self.viewport.clear_cloud(f"{name}: no frames captured")
        self._clear_live_view()
        shape = points.shape if points is not None else "none"
        print(f"{name} complete: {self.frames} frames, last point cloud {shape}", flush=True)

    def _on_cancelled(self) -> None:
        self._clear_live_view()
        self.viewport.set_frame_count(0)
        self._points.setText("Points: —")
        self._coverage.setText("Coverage: —")
        self.viewport.show_toast("Scan cancelled")
        model = self.model_panel.current_model()
        if model is None:
            self.viewport.clear_cloud("The live point cloud appears here")
            return
        # Back to the models, as they were before the scan began.
        self.left.setCurrentWidget(self.model_panel)
        self.viewport.set_stage(scanning=False)
        self._on_model_changed(model)

    def _clear_live_view(self) -> None:
        self.viewport.show_warning("")
        self.camera_panel.clear_feeds()
        self.viewport.gauge.set_depths(None)
        self._fps.setText("FPS: —")

    # Models and REA: fitting primitives to the selected model and exporting them.

    def _on_model_changed(self, model: ScanModel | None) -> None:
        if self._state is not ScanState.IDLE:
            return  # the live scan has the 3D view
        if model is not None and model is not self._shown_model:
            if model.points is None:
                self.viewport.clear_cloud(f"{model.name}: no frames captured")
            else:
                self.viewport.show_cloud(model.points, model.colors, model.normals,
                                         frame=True)
                self._points.setText(f"Points: {len(model.points):,}")
            self._shown_model = model
        self._show_fits()
        self._update_rea_tools()

    def _show_fits(self) -> None:
        """Tints the shown model's points by the primitives fitted to them."""
        model = self.model_panel.current_model()
        if (self._state is ScanState.IDLE and model is not None and model is self._shown_model
                and model.session is not None):
            self.viewport.set_highlight(tint(model.session, self.model_panel.current_primitive()))

    def _update_rea_tools(self) -> None:
        model = self.model_panel.current_model()
        ready = (self._state is ScanState.IDLE and not self._fitting and model is not None
                 and model.session is not None)
        self.fit_plane_action.setEnabled(ready)
        self.fit_cylinder_action.setEnabled(ready)
        self.export_step_action.setEnabled(ready and bool(model.session.primitives))
        self.model_panel.set_busy(self._fitting)

    def fit(self, kind: str) -> None:
        """Fits a "plane" or a "cylinder" to the selected model's free points."""
        model = self.model_panel.current_model()
        if not self.fit_plane_action.isEnabled() or model is None:
            return
        session = model.session
        job = FitJob(kind, model, session.unclaimed(), self.model_panel.tolerance_mm(),
                     session.next_seed())
        self._fitting = True
        self._update_rea_tools()
        self.viewport.show_toast(f"Fitting a {kind} to {len(job.candidates):,} points…", 60_000)
        self.fit_requested.emit(job)

    def _on_fitted(self, job: FitJob, found, seconds: float) -> None:
        self._fitting = False
        if found is None:
            self.viewport.show_toast(
                f"No {job.kind} found among the {len(job.candidates):,} free points", 5000)
            print(f"no {job.kind} found in {job.model.name} ({seconds:.2f} s)", flush=True)
        else:
            primitive = job.model.session.add(found)
            if self.model_panel.current_model() is job.model:
                self.model_panel.refresh_primitives(select=primitive)
            self.viewport.show_toast(f"{primitive.name}: {primitive.summary()}, "
                                     f"{len(primitive.inliers):,} points", 5000)
            details = ", ".join(f"{key.lower()} {value}" for key, value in primitive.describe())
            print(f"{job.model.name}: {primitive.name} in {seconds:.2f} s: {details}", flush=True)
        self._update_rea_tools()

    def _on_fit_failed(self, job: FitJob, message: str) -> None:
        self._fitting = False
        print(f"fit failed: {message}", file=sys.stderr, flush=True)
        self.viewport.show_toast(f"Fitting failed: {message}", 6000)
        self._update_rea_tools()

    def _remove_primitive(self, primitive) -> None:
        model = self.model_panel.current_model()
        if model is None or self._fitting or primitive not in model.session.primitives:
            return
        model.session.remove(primitive)
        self.model_panel.refresh_primitives()
        self._update_rea_tools()
        self.viewport.show_toast(f"Removed {primitive.name}")

    def export_step(self, path: str | Path | None = None) -> Path | None:
        """Saves the selected model's primitives as a STEP file, asking where
        unless given a path. Returns the file written, or None."""
        model = self.model_panel.current_model()
        if not self.export_step_action.isEnabled() or model is None:
            return None
        if path is None:
            path, _ = QFileDialog.getSaveFileName(
                self, "Export STEP", str(Path.home() / f"{model.name}.step"),
                "STEP files (*.step *.stp)")
            if not path:
                return None
        path = Path(path)
        if path.suffix.lower() not in (".step", ".stp"):
            path = path.with_name(path.name + ".step")
        try:
            written = model.session.export_step(path)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f"STEP export failed: {exc}", file=sys.stderr, flush=True)
            self.viewport.show_toast(f"Export failed: {exc}", 6000)
            return None
        count = len(model.session.primitives)
        self.viewport.show_toast(f"Exported {count} primitive{'s' * (count != 1)} to "
                                 f"{written.name}", 5000)
        print(f"{model.name}: exported {count} primitives to {written}", flush=True)
        return written

    # Settings.

    def _on_preset(self, preset) -> None:
        self.viewport.gauge.set_bands(preset.band_edges_mm, preset.scale_mm)
        # The camera panel sends a changed exposure on to the POP 4.
        self.camera_panel.exposure.setValue(preset.exposure_us)
        # The POP 4's depth decoder searches the preset's working distance.
        self._control("set_depth_range", *map(float, preset.scale_mm))
        marker = preset.tracking in ("marker", "globalMarker")
        self.marker_scan.setChecked(marker)
        self.feature_scan.setChecked(not marker)
        self.marker_scan.setEnabled(self.scan_panel.tracking.findData("marker") >= 0)

    # The POP 4: picking it, its lights, exposure and scan mode.

    def _check_pop4_present(self) -> None:
        present = bool(self._pop4_present())
        self.scan_panel.set_pop4_present(present or self.scan_panel.pop4)
        if self._pop4 and not present and not self._connecting:
            # Unplugged between scans, where no capture is running to notice.
            self._on_sensor_lost("it is no longer on the USB bus")
            return
        # A scanner just back on the bus re-enumerates for a moment, so wait
        # for a second look before opening it.
        self._present_ticks = self._present_ticks + 1 if present else 0
        wanted = self.scan_panel.pop4 and not self._pop4 and not self._connecting
        if wanted and self._present_ticks >= 2 and 0 < self._reconnects < self.MAX_RECONNECTS:
            self._reconnects += 1
            print(f"reconnecting to the POP 4 (try {self._reconnects} of "
                  f"{self.MAX_RECONNECTS})", flush=True)
            self._connect_pop4()

    def _on_device(self, pop4: bool) -> None:
        self.camera_panel.set_pop4(False)
        self.scan_panel.set_ready(False)
        self._reconnects = 0
        if pop4:
            self._connect_pop4()
        else:
            self._connecting = False
            self.viewport.show_warning("")
            self.mock_requested.emit()

    def _connect_pop4(self) -> None:
        self._connecting = True
        self.scan_panel.set_ready(False)
        self._connection.setText('<span style="color:#e0a83e">●</span> '
                                 "Connecting to the POP 4…")
        self.scan_panel.set_connection("Connecting…", ok=False)
        # After an earlier session the scanner may need a reset first, which
        # reboots it: measured at up to a minute and a half.
        self.viewport.show_toast("Connecting to the POP 4… (up to a minute and a half if it "
                                 "has to restart)", 120_000)
        self.pop4_requested.emit()

    def _on_sensor_changed(self, settings: dict) -> None:
        self._connecting = False
        self._pop4 = settings["kind"] == "pop4"
        if self._pop4:
            self._reconnects = 0
            self.viewport.show_warning("")
        self.camera_panel.set_pop4(self._pop4)
        if self._pop4:
            # The lights as the scanner has them; the exposure as the preset asks.
            self.camera_panel.show_controls(settings["lights"],
                                            self.scan_panel.preset.exposure_us)
            self._scan_mode = settings["mode"]
            self.scan_panel.set_mode(self._scan_mode)
            self._control("set_exposure", self.camera_panel.exposure.value())
            self._control("set_depth_range", *map(float, self.scan_panel.preset.scale_mm))
            self.viewport.show_toast(f"POP 4 {settings['serial']} connected: "
                                     f"{settings['backend']}", 5000)
            print(f"POP 4 {settings['serial']} connected: {settings['backend']}", flush=True)
        self._show_connection(self._pop4, settings.get("serial", ""))
        self.scan_panel.set_ready(True)

    def _on_open_failed(self, reason: str) -> None:
        self._connecting = False
        print(f"opening the POP 4 failed: {reason}", file=sys.stderr, flush=True)
        if 0 < self._reconnects < self.MAX_RECONNECTS:
            # Lost mid-session and not back yet; _check_pop4_present retries.
            self._show_waiting(reason)
            self.scan_panel.set_ready(True)
            return
        self.viewport.show_toast(f"Could not open the POP 4: {reason}", 8000)
        self.viewport.show_warning("")
        self.scan_panel.device.setCurrentIndex(0)  # back to the mock sensor

    def _on_sensor_lost(self, reason: str) -> None:
        """The scanner went away mid-session. Say so, rather than leaving a
        status bar that claims it is still there, and wait for it to return."""
        self._pop4 = False
        self._connecting = False
        # The capture that died never came back through _on_frame and friends,
        # so clear the in-flight flag here: otherwise no later frame is ever
        # asked for and a scan after reconnecting would sit silent.
        self._in_flight = False
        self._reconnects = max(self._reconnects, 1)
        self._present_ticks = 0
        self.camera_panel.set_pop4(False)
        print(f"the POP 4 disconnected: {reason}", file=sys.stderr, flush=True)
        self.scan_panel.cancel()  # a scan cannot go on without the scanner
        self._show_waiting(reason)

    def _show_waiting(self, reason: str) -> None:
        self._connection.setText('<span style="color:#e0a83e">●</span> '
                                 "Revopoint POP 4 disconnected — waiting for it")
        self.scan_panel.set_connection("Disconnected", ok=False)
        self.viewport.show_warning(
            f"The POP 4 disconnected ({reason}). It drops off USB and re-enumerates after "
            f"a session, so this happens; the app reconnects when it comes back. Unplug it "
            f"and plug it in again if it does not.")

    def _show_connection(self, pop4: bool, serial: str = "") -> None:
        name = f"Revopoint POP 4 {serial}".rstrip() if pop4 else "Mock depth sensor"
        self._connection.setText(f'<span style="color:#5bbf5b">●</span> {name} connected')
        self.scan_panel.set_connection("Connected")

    def _control(self, name: str, *args) -> None:
        """Has the POP 4 run one of its methods, between captures."""
        if self._pop4:
            self.control_requested.emit(name, args)

    def _on_mode_picked(self, mode: str) -> None:
        if not self._pop4 or mode == self._scan_mode:
            return
        self.scan_panel.set_ready(False)
        if mode == "infrared":
            # Only a reboot gets a POP 4 out of blue laser or speckle mode.
            self.viewport.show_toast("Restarting the scanner into infrared (about 45 s)…",
                                     90_000)
        else:
            self.viewport.show_toast(f"Switching to {ScanPanel.MODES[mode].lower()}…", 10_000)
        self._control("set_scan_mode", mode)
        self._scan_mode_asked = mode

    def _on_control_done(self, name: str, error: str) -> None:
        if name == "set_scan_mode":
            if not error:
                self._scan_mode = self._scan_mode_asked
                self.viewport.show_toast(f"Scan mode: {ScanPanel.MODES[self._scan_mode]}", 4000)
            self.scan_panel.set_mode(self._scan_mode)
            self.scan_panel.set_ready(True)
        if error:
            print(f"POP 4 {name} failed: {error}", file=sys.stderr, flush=True)
            self.viewport.show_toast(f"The POP 4 refused {name.replace('_', ' ')}: {error}",
                                     6000)

    def _stop_worker(self) -> None:
        # Runs on close and on quit: Qt aborts the app if a QThread is
        # destroyed while it is still running.
        self._streaming = False
        self._presence.stop()
        for thread in (self._thread, self._rea_thread):
            if thread.isRunning():
                thread.quit()
                thread.wait()
        self._worker.close()  # its thread has stopped, so it is safe here

    def closeEvent(self, event) -> None:
        self._stop_worker()
        super().closeEvent(event)


def cloud_pixels(window: MainWindow) -> tuple[int, int]:
    """How many pixels of the 3D view show the cloud in scan blue, and how
    many in a fitted primitive's tint, as last drawn."""
    rgb = image_pixels(window.viewport.view3d.grabFramebuffer()).astype(np.int16)
    red, green, blue = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    scan_blue = (blue > red + 30) & (blue > green + 15)
    tinted = np.maximum(red, green) > blue + 15  # amber, green, pink...: never grey or blue
    return int(scan_blue.sum()), int(tinted.sum())


def run_smoke_test(app: QApplication, window: MainWindow, frames: int = 30) -> None:
    """Scans `frames` frames, completes the scan, fits a plane and a cylinder
    to it and exports them to STEP, checking each step; then exits 0, or 1
    on a failed check."""
    beat = {"last": time.perf_counter(), "longest": 0.0}
    report: dict[str, object] = {}
    folder = tempfile.TemporaryDirectory(prefix="all3dscan-smoke-")

    def tick() -> None:
        now = time.perf_counter()
        beat["longest"] = max(beat["longest"], now - beat["last"])
        beat["last"] = now

    heartbeat = QTimer(window)
    heartbeat.timeout.connect(tick)

    def on_frame(frame, _seconds: float) -> None:
        if window.frames == 1:
            print(f"first frame: points {frame.points.shape}, depth {frame.depth_mm.shape}, "
                  f"ir {frame.ir.shape}, rgb {frame.rgb.shape}", flush=True)
        if window.frames == frames:
            QTimer.singleShot(0, complete)

    def complete() -> None:
        heartbeat.stop()
        feeds = [window.camera_panel.depth_view, window.camera_panel.rgb_view]
        report["previews"] = all(not f.image.pixmap().isNull() for f in feeds)
        report["distance"] = window.viewport.gauge.distance_mm
        report["shown"] = window.viewport.view3d.point_count
        report["blue pixels"], _ = cloud_pixels(window)
        window.scan_panel.complete()
        report["model"] = window.model_panel.models.count() == 1
        window._rea_worker.fitted.connect(on_fitted)
        window.fit("plane")

    def on_fitted(job, found, _seconds: float) -> None:
        # Connected after the window's own handler, which has added the primitive.
        report[job.kind] = found is not None
        if job.kind == "plane":
            QTimer.singleShot(0, lambda: window.fit("cylinder"))
        else:
            QTimer.singleShot(0, export)

    def export() -> None:
        session = window.model_panel.current_model().session
        report["fits"] = "; ".join(f"{p.name}: {p.summary()}" for p in session.primitives)
        _, report["tinted pixels"] = cloud_pixels(window)
        written = window.export_step(Path(folder.name) / "smoke.step")
        bodies = rea.read_step(written) if written else []
        report["STEP bodies"] = sorted(name for name, _, _ in bodies)
        finish()

    def finish() -> None:
        ok = (report["previews"] and report["shown"] and report["distance"] is not None
              and report["blue pixels"] > 1000 and report["model"] and report.get("plane")
              and report.get("cylinder") and report["tinted pixels"] > 1000
              and report["STEP bodies"] == ["Cylinder 1", "Plane 1"])
        print(f"smoke test: {frames} frames streamed; previews "
              f"{'filled' if report['previews'] else 'EMPTY'}; 3D view showing "
              f"{report['shown']:,} points ({report['blue pixels']:,} pixels in scan blue); "
              f"distance {report['distance']} mm; model listed: {report['model']}; "
              f"fitted {report.get('fits') or 'nothing'} ({report['tinted pixels']:,} tinted "
              f"pixels); STEP bodies read back: {report['STEP bodies']}; the GUI thread's "
              f"longest pause while scanning was {beat['longest'] * 1000:.1f} ms", flush=True)
        folder.cleanup()
        app.exit(0 if ok else 1)

    def time_out() -> None:
        print(f"smoke test: timed out; so far {report}", file=sys.stderr)
        app.exit(2)

    def start() -> None:
        beat["last"] = time.perf_counter()  # measure from here, not from start-up
        window.scan_panel.new_scan()

    # Connected after the window's own handler, so on_frame sees the new count.
    window._worker.frame_ready.connect(on_frame)
    window._worker.failed.connect(lambda _message: app.exit(1))
    window._rea_worker.failed.connect(lambda _job, _message: app.exit(1))
    QTimer.singleShot(60_000, time_out)
    heartbeat.start(5)
    QTimer.singleShot(0, start)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--smoke-test", action="store_true",
                        help="scan 30 frames, complete the scan, fit a plane and a cylinder, "
                             "export them to STEP, then exit")
    args, qt_args = parser.parse_known_args(argv)

    qt_argv = [sys.argv[0], *qt_args]
    configure_opengl(qt_argv)  # before the QApplication, which reads it
    app = QApplication(qt_argv)
    warn_if_platform_differs()
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
    window = MainWindow()
    window.show()
    if args.smoke_test:
        run_smoke_test(app, window)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
