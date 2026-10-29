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
import math
import sys
import tempfile
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PyQt6.QtCore import (
    PYQT_VERSION_STR,
    QT_VERSION_STR,
    QObject,
    Qt,
    QThread,
    QTimer,
    pyqtSignal,
    pyqtSlot,
)
from PyQt6.QtGui import QAction, QActionGroup, QIcon, QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication,
    QFileDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QSplitter,
    QStackedWidget,
    QToolBar,
)

import all3dscan_engine as engine
import rea_engine as rea
import scanners
import theme
from common import (
    APP_ICON,
    APP_ID,
    apply_theme,
    configure_opengl,
    current_theme,
    custom_colours,
    remember_custom_colours,
    remember_theme,
    theme_named,
    warn_if_platform_differs,
)
from theme_dialog import CustomThemeDialog
from panels import CameraPanel, ModelPanel, ScanModel, ScanPanel, ScanState, Viewport, tint
from viewport3d import image_pixels
from widgets import RibbonGroup


def for_display(cloud: tuple) -> tuple:
    """A cloud cut down to what the viewport draws, and made contiguous so
    that showing it copies nothing more."""
    step = max(math.ceil(len(cloud[0]) / Viewport.MAX_SHOWN_POINTS), 1)
    if step == 1:
        return cloud
    return tuple(np.ascontiguousarray(part[::step]) for part in cloud)


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
    exposure_tuned = pyqtSignal(dict)  # what each exposure saw, and the one kept

    # Frames that must pass between one exposure write and the next.
    # Measured on a POP 4 (2026-10-28): two writes back to back left the
    # scanner on the bus but sent no frames at all afterwards, where the same
    # two with six frames between them were harmless.
    EXPOSURE_GAP_FRAMES = 6

    def __init__(self, sensor, open_pop4) -> None:
        """sensor is what to scan with until a POP 4 is opened, and None when
        there is nothing: the window waits for the scanner rather than
        offering a scan of made-up depth."""
        super().__init__()
        self._mock = self._sensor = sensor
        self._open_pop4 = open_pop4
        self._since_exposure = 0  # frames since the last exposure write

    @pyqtSlot()
    def capture(self) -> None:
        if self._sensor is None:
            self.failed.emit("there is no scanner to capture from")
            return
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
            self._since_exposure += 1
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
        self._since_exposure = 0
        self.sensor_changed.emit(settings)

    @pyqtSlot()
    def use_mock(self) -> None:
        if self._mock is None:
            return
        self.close()
        self._sensor = self._mock
        self.sensor_changed.emit({"kind": "mock",
                                  "intrinsics": self._mock.intrinsics})

    @pyqtSlot(str, tuple)
    def control(self, name: str, args: tuple) -> None:
        """Calls the POP 4's method `name` with args, if the POP 4 is in use."""
        if self._sensor is self._mock:
            return
        try:
            if name == "set_exposure":
                self._write_exposure(*args)
            else:
                getattr(self._sensor, name)(*args)
        except BaseException as exc:
            self.control_done.emit(name, describe(exc))
            return
        self.control_done.emit(name, "")

    def _write_exposure(self, microseconds: int) -> None:
        """Sets the exposure, after letting enough frames pass since the last
        time it changed. Two writes in a row stop the scanner sending
        anything, and it does not report the fault: the frames simply stop."""
        while self._since_exposure < self.EXPOSURE_GAP_FRAMES:
            if self._sensor.capture_frame(timeout_ms=3000) is None and self._gone():
                return
            self._since_exposure += 1
        self._sensor.set_exposure(microseconds)
        self._since_exposure = 0

    # How the working distance is narrowed onto the object once the exposure
    # is settled. The decoder searches the range it is given, and a narrower
    # search finds more: measured on a POP 4 (2026-10-28), the same scene gave
    # 4,161 points a frame over 250-500 mm, 5,172 over 270-400 and 5,336 over
    # 290-370, with the wide range repeated afterwards to rule out drift. So
    # the band the object occupies is found and kept, with room around it.
    RANGE_BAND = (2.0, 98.0)  # the depth percentiles the object is taken to fill
    RANGE_MARGIN = 0.25  # of the band's width, left on each side
    RANGE_LEAST_MM = 60.0  # ...but never a slab thinner than this
    # Exposures tried, in microseconds, within the range the backend takes.
    EXPOSURES = (1000, 2000, 3500, 5000, 6000)
    SETTLE_FRAMES = 4  # the sensor takes a few frames to follow a change
    MEASURE_FRAMES = 8  # per-frame yield is noisy, so judge on several

    @pyqtSlot(tuple)
    def tune_exposure(self, whole_range: tuple) -> None:
        """Tries each exposure and keeps the one that sees most of the
        object. Measured on a real POP 4: the points in a frame varied three
        to one across this range, so the default is often far from the best.
        Runs here, between captures, because it needs the scanner to itself."""
        if self._sensor is self._mock:
            return
        was = self._sensor.exposure_us
        tried: list[tuple[int, int]] = []
        trouble = ""
        for microseconds in self.EXPOSURES:
            # The scanner refuses a settings write for a while after it has
            # re-enumerated, which connecting makes it do. Skip that step
            # rather than lose the sweep: the rest still say which is best.
            try:
                self._write_exposure(microseconds)
                for _ in range(self.SETTLE_FRAMES):
                    self._sensor.capture_frame(timeout_ms=3000)
                    self._since_exposure += 1
                seen = []
                for _ in range(self.MEASURE_FRAMES):
                    frame = self._sensor.capture_frame(timeout_ms=3000)
                    self._since_exposure += 1
                    if frame is not None:
                        seen.append(len(frame.points))
            except BaseException as exc:
                trouble = describe(exc)
                if self._gone():
                    break
                continue
            if seen:
                tried.append((microseconds, int(np.median(seen))))
        if not tried:
            self.exposure_tuned.emit({"error": trouble or "no frames came", "tried": []})
            return
        best = max(tried, key=lambda t: t[1])[0]
        try:
            self._write_exposure(best)
        except BaseException as exc:
            self.exposure_tuned.emit({"error": describe(exc), "tried": tried})
            return
        self.exposure_tuned.emit({"exposure_us": best, "tried": tried, "trouble": trouble,
                                  "range_mm": self._narrow_range(whole_range)})

    def _narrow_range(self, whole: tuple[float, float]) -> tuple[float, float] | None:
        """Narrows the depth search onto the band the object fills.

        The decoder is told where to look, and looking over a narrower band
        finds more of the surface. That is a trade, not a free gain: whatever
        falls outside the band is lost. So the band is taken from where the
        points actually are, given room on each side, and only written when
        it is really narrower than the preset's. An object standing at one
        distance gets a tighter search; a scene spread over the whole working
        distance is left alone. None when nothing was changed."""
        near, far = whole
        seen = []
        try:
            for _ in range(self.MEASURE_FRAMES):
                frame = self._sensor.capture_frame(timeout_ms=3000)
                self._since_exposure += 1
                if frame is not None and len(frame.points):
                    seen.append(np.asarray(frame.points)[:, 2])
        except BaseException:
            return None
        if not seen:
            return None
        depths = np.concatenate(seen)
        low, high = np.percentile(depths, self.RANGE_BAND)
        room = max((high - low) * self.RANGE_MARGIN, (self.RANGE_LEAST_MM - (high - low)) / 2)
        low, high = max(near, low - room), min(far, high + room)
        if not high - low >= self.RANGE_LEAST_MM / 2:
            return None  # too thin a slab to scan in
        if high - low >= (far - near) * 0.95:
            return None  # the points fill the whole range; nothing to narrow
        try:
            self._sensor.set_depth_range(float(low), float(high))
        except BaseException:
            return None
        return float(low), float(high)

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
                "mode": pop4.scan_mode, "exposure_us": pop4.exposure_us,
                "lights": lights, "intrinsics": pop4.intrinsics}


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


class FusionWorker(QObject):
    """Builds one cloud out of a scan's frames, on the thread it is moved to.

    Fusing a frame costs far more than capturing one, so the window offers it
    whichever frame is current when this is free and drops the rest: the live
    view keeps its frame rate and the model fills in as fast as the machine
    allows, which is how a scanner tracks."""

    # The model, a thinned copy of it to draw, and how the frame sat. The
    # first two are None for a frame that would not align: the window waits
    # on this either way before offering another, so it is always sent.
    model_ready = pyqtSignal(object, object, dict)
    surface_ready = pyqtSignal(object, object)  # the surface, and the model it belongs to
    failed = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self._fusion = None

    @pyqtSlot(float)
    def start(self, voxel_mm: float) -> None:
        try:
            self._fusion = engine.Fusion(voxel_mm=voxel_mm)
        except BaseException as exc:
            self._fusion = None
            self.failed.emit(describe(exc))

    @pyqtSlot()
    def stop(self) -> None:
        self._fusion = None

    @pyqtSlot(object, tuple, float)
    def build_surface(self, model, intrinsics: tuple, voxel_mm: float) -> None:
        """Melts the scan's depth views into one closed surface."""
        if self._fusion is None or not intrinsics or not intrinsics[0]:
            self.surface_ready.emit(None, model)
            return
        try:
            surface = self._fusion.surface(intrinsics, voxel_mm=voxel_mm)
        except BaseException as exc:
            self.failed.emit(describe(exc))
            self.surface_ready.emit(None, model)
            return
        self.surface_ready.emit(surface if len(surface[2]) else None, model)

    @pyqtSlot(object, object, object, object)
    def add(self, points, colors, normals, depth_mm) -> None:
        if self._fusion is None:
            return
        try:
            fit = self._fusion.add(points, colors, normals, depth_mm=depth_mm)
            cloud = self._fusion.cloud() if fit["merged"] else None
        except BaseException as exc:
            self.failed.emit(describe(exc))
            return
        # Thinned here rather than on the GUI thread: a model of several
        # million points costs about 100 ms to cut down, and the window would
        # stop for it. Arrays themselves pass by reference.
        display = for_display(cloud) if cloud is not None else None
        self.model_ready.emit(cloud, display, fit)


class MainWindow(QMainWindow):
    frame_requested = pyqtSignal()
    fuse_requested = pyqtSignal(object, object, object, object)  # points, colours, normals, depth
    surface_requested = pyqtSignal(object, tuple, float)  # the model, intrinsics, voxel
    fusion_start_requested = pyqtSignal(float)  # the model's voxel, in mm
    fusion_stop_requested = pyqtSignal()
    tune_exposure_requested = pyqtSignal(tuple)  # the preset's working distance
    pop4_requested = pyqtSignal()
    mock_requested = pyqtSignal()
    control_requested = pyqtSignal(str, tuple)  # a POP 4 method and its arguments
    fit_requested = pyqtSignal(object)  # FitJob
    TARGET_FPS = 30
    # Between scans the cameras stay live, so the scanner can be aimed and
    # the distance gauge read before a scan is started. Slower than scanning:
    # nothing is being captured, and the frames cost the scanner work.
    PREVIEW_FPS = 10
    # Captures in a row that may bring no frame before a scan stops: each
    # waits up to two seconds, so this is about six seconds of silence.
    MAX_MISSED_FRAMES = 3
    # Below this share of pixels with depth, the scanner is not really seeing
    # the object: good infrared scanning measured 27.8% (RE_NOTES 2026-10-16),
    # and a scene too far or too dark gives under 1%.
    # Points in a frame below which the scanner is not seeing enough of
    # anything to scan. Judged on points rather than on the share of the
    # depth camera they cover: a small object rightly fills little of the
    # frame — a 60 mm ring at 360 mm measured 4.7% of it, and scanned.
    TOO_FEW_POINTS = 1_000
    # Frames in a row under it before saying so, so a glance away is ignored.
    LOW_COVERAGE_FRAMES = 8
    # After the scanner goes away mid-session it usually comes back within a
    # few seconds, so try this many times before giving up on it. Each try
    # waits for two consecutive looks that find one, about four seconds.
    MAX_RECONNECTS = 4
    # Looks in a row that must find no scanner before it counts as gone. It
    # leaves the bus and comes back on its own — connecting resets it, which
    # costs about twelve seconds of re-enumeration — so one empty look means
    # nothing. A scanner that has really gone is caught by the captures
    # failing as well, which is immediate.
    ABSENT_LOOKS = 4
    # One point per cube of this edge in the fused model, as the Qt app's
    # pipeline uses (src/processing/ScanPipeline.cpp).
    FUSION_VOXEL_MM = 1.0
    # Points of a frame offered to the fusion. It thins what it keeps to the
    # voxel above, so a denser frame only costs time — and the copy into the
    # engine holds the GIL, which the window would feel.
    FUSION_MAX_POINTS = 200_000
    # Completing a scan drops the points with far fewer neighbours within
    # this radius than the model's points typically have: the speckle a
    # depth camera leaves in mid-air. The share is measured against the
    # model itself, so a sparse scan is not stripped bare.
    SPECKLE_RADIUS_MM = 3.0
    SPECKLE_SHARE = 0.2
    # The detail the built surface holds. The reconstruction grows it when
    # the scan covers a large box, so this is a floor rather than a promise.
    SURFACE_VOXEL_MM = 1.0
    # How long the scanner's white lights stay on, and then off, while a
    # scan runs. Slow enough that the commands do not crowd the captures.
    FLASH_MS = 500

    def __init__(self, sensor=None, open_pop4=None, pop4_present=None) -> None:
        """The POP 4 is the scanner: the window opens it as soon as one is on
        the bus, and waits for it when there is none.

        sensor is a stand-in to scan with instead, for development and the
        tests; passing one adds it to the scanner list. open_pop4() opens a
        POP 4 and pop4_present() says if one is plugged in (engine.Pop4Sensor
        and engine.pop4_present unless given)."""
        super().__init__()
        self.setWindowTitle("All3DScan")
        self.resize(1440, 900)
        self.setMinimumSize(1100, 680)
        self.sensor = sensor

        self.camera_panel = CameraPanel()
        self.model_panel = ModelPanel()
        self.left = QStackedWidget()
        self.left.addWidget(self.camera_panel)
        self.left.addWidget(self.model_panel)
        self.left.setMinimumWidth(260)
        self.viewport = Viewport()
        self.scan_panel = ScanPanel(mock=self.sensor is not None)
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
        self._build_menus()

        # Labels rather than showMessage(), which Qt draws over the bar's widgets.
        self._connection = QLabel()
        self.scan_state = QLabel(ScanState.IDLE.value)
        self._fps = QLabel("FPS: —")
        self._points = QLabel("Points: —")
        self._tracking = QLabel("Tracking: —")
        self._tracking.setToolTip("Frames the fusion placed on the model, of those it was given")
        self._coverage = QLabel("Coverage: —")
        self._coverage.setToolTip("The share of the depth camera's pixels that got a distance")
        self.statusBar().addWidget(self._connection)
        self.statusBar().addWidget(self.scan_state)
        for label in (self._fps, self._tracking, self._points, self._coverage):
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
        self._absent_ticks = 0  # and consecutive looks that did not
        self._other_scanners: list = []  # found on the bus, but not drivable
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
        self._worker.exposure_tuned.connect(self._on_exposure_tuned)
        self.tune_exposure_requested.connect(self._worker.tune_exposure)
        self._thread.start()

        self._fitting = False
        self._fusing = False  # a frame is with the fusion worker
        self._meshing = False  # ...and a surface is being built
        self._tuning = False  # the scanner is busy trying exposures
        self._fused: tuple | None = None  # the model built from this scan
        self._tracked = 0  # frames the fusion placed on the model
        self._offered = 0  # frames it was given
        self._shown_model: ScanModel | None = None  # the model in the 3D view
        # The camera a scan is being taken with, for building its surface.
        self._intrinsics: tuple = tuple(getattr(self.sensor, "intrinsics", ()))
        self._rea_thread = QThread(self)
        self._rea_worker = ReaWorker()
        self._rea_worker.moveToThread(self._rea_thread)
        self._rea_thread.finished.connect(self._rea_worker.deleteLater)
        self.fit_requested.connect(self._rea_worker.fit)
        self._rea_worker.fitted.connect(self._on_fitted)
        self._rea_worker.failed.connect(self._on_fit_failed)
        self._rea_thread.start()

        self._fusion_thread = QThread(self)
        self._fusion_worker = FusionWorker()
        self._fusion_worker.moveToThread(self._fusion_thread)
        self._fusion_thread.finished.connect(self._fusion_worker.deleteLater)
        self.fuse_requested.connect(self._fusion_worker.add)
        self.fusion_start_requested.connect(self._fusion_worker.start)
        self.fusion_stop_requested.connect(self._fusion_worker.stop)
        self._fusion_worker.model_ready.connect(self._on_model_ready)
        self._fusion_worker.surface_ready.connect(self._on_surface_ready)
        self.surface_requested.connect(self._fusion_worker.build_surface)
        self._fusion_worker.failed.connect(self._on_fusion_failed)
        self._fusion_thread.start()
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
        camera.light_toggled.connect(self._on_light_toggled)
        camera.light_level_changed.connect(
            lambda light, level: self._control("set_light_level", light, level))
        camera.exposure_changed.connect(lambda us: self._control("set_exposure", us))
        camera.auto_exposure_requested.connect(self._tune_exposure)
        self._on_preset(panel.preset)
        self._show_connection(False)

        # The POP 4 can be picked while one is plugged in; look now and then.
        self._pop4_present = pop4_present or engine.pop4_present
        self._presence = QTimer(self)
        self._presence.timeout.connect(self._check_pop4_present)
        self._presence.start(2000)

        # The white lights flash while scanning rather than staying on.
        self._flash_on = False
        self._flash = QTimer(self)
        self._flash.timeout.connect(self._flash_lights)
        # Nothing to scan with until a scanner is opened, so no scan may
        # start. After the flash timer: this reaches _on_scan_state, which
        # asks whether the lights should be flashing.
        panel.set_ready(self.sensor is not None)
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
        self.fusion_action = action(
            "Fusion", "draw-polygon",
            "Build one model from the scan's frames, aligning each onto the last.\n"
            "Off, the view shows only the newest frame.", checkable=True)
        self.fusion_action.setChecked(True)
        point_cloud = [
            self.fusion_action,
            action("Isolation", "edit-delete", later, enabled=False),
            action("Overlap", "edit-select-all", later, enabled=False),
            action("Smooth", "smooth", later, enabled=False),
            action("Simplify", "transform-scale", later, enabled=False),
        ]
        self.mesh_action = action(
            "Mesh", "draw-cuboid",
            "Melt the scan's depth views into one closed surface", enabled=False)
        self.mesh_action.triggered.connect(self.build_surface)
        self._mesh_tip = self.mesh_action.toolTip()
        mesh = [
            self.mesh_action,
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
        self.export_action = action(
            "Export", "document-save",
            "Save this model as PLY, OBJ, STL, PCD or plain columns, for another tool",
            enabled=False)
        self.export_action.triggered.connect(lambda: self.save_scan())
        self.fullscreen_action = action("Fullscreen", "view-fullscreen", "Shift+M",
                                        checkable=True)
        self.fullscreen_action.setShortcut(QKeySequence("Shift+M"))
        self.fullscreen_action.toggled.connect(
            lambda on: self.showFullScreen() if on else self.showNormal())
        self.addAction(self.fullscreen_action)  # so Shift+M works anywhere in the window

        self.point_cloud_actions = point_cloud
        self.mesh_actions = mesh
        self.rea_actions = rea_tools
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

    def _build_menus(self) -> None:
        """The menu bar above the ribbon. It offers the ribbon's own actions,
        so the two never disagree about what is available, plus the things
        that have no button: quitting, the shortcuts and what this is."""
        panel = self.scan_panel

        def item(text: str, slot, shortcut: str = "", tip: str = "") -> QAction:
            action = QAction(text, self)
            action.triggered.connect(slot)
            if shortcut:
                action.setShortcut(QKeySequence(shortcut))
            if tip:
                action.setStatusTip(tip)
            return action

        bar = self.menuBar()
        file_menu = bar.addMenu("&File")
        # The scan controls are the panel's buttons; Space and Esc already
        # reach them, so these carry no shortcut of their own.
        self.new_scan_item = item("&New scan", panel.new_scan, tip="Space")
        self.complete_item = item("&Complete scan", panel.complete)
        self.cancel_item = item("C&ancel scan", panel.cancel, tip="Esc")
        for action in (self.new_scan_item, self.complete_item, self.cancel_item):
            file_menu.addAction(action)
        file_menu.addSeparator()
        self.open_item = item("&Open scan…", self.open_scan, "Ctrl+O",
                              tip="A scan from any scanner: PLY, OBJ, STL, PCD, ASC, XYZ, PTS")
        self.save_item = item("&Save scan as…", self.save_scan, "Ctrl+S",
                              tip="Save this model where another tool can open it")
        file_menu.addAction(self.open_item)
        file_menu.addAction(self.save_item)
        file_menu.addSeparator()
        file_menu.addAction(self.export_step_action)
        file_menu.addAction(self.export_action)
        file_menu.addSeparator()
        file_menu.addAction(item("&Quit", self.close, "Ctrl+Q"))
        file_menu.aboutToShow.connect(self._update_menus)

        edit_menu = bar.addMenu("&Edit")
        edit_menu.addAction(self.fusion_action)
        edit_menu.addSeparator()
        for action in self.point_cloud_actions:
            if action is not self.fusion_action:
                edit_menu.addAction(action)
        edit_menu.addSeparator()
        for action in self.mesh_actions:
            edit_menu.addAction(action)
        edit_menu.addSeparator()
        for action in self.rea_actions:
            # Exporting is an output, and File already offers it.
            if action is not self.export_step_action:
                edit_menu.addAction(action)

        theme_menu = bar.addMenu("&Theme")
        self.theme_actions = QActionGroup(self)
        self.theme_actions.setExclusive(True)
        for name in (*theme.THEMES, theme.CUSTOM):
            choice = QAction(f"{name}…" if name == theme.CUSTOM else name,
                             self, checkable=True)
            choice.setData(name)
            choice.setChecked(name == current_theme())
            if name == theme.CUSTOM:
                choice.setToolTip("Choose the colours yourself")
                choice.triggered.connect(lambda _checked: self.choose_custom_theme())
            else:
                choice.triggered.connect(lambda _checked, name=name: self.set_theme(name))
            self.theme_actions.addAction(choice)
            theme_menu.addAction(choice)

        help_menu = bar.addMenu("&Help")
        help_menu.addAction(item("&Shortcuts…", self._show_shortcuts, "F1"))
        help_menu.addAction(item("Scanning &tips…", self._show_tips))

        # A heading of its own rather than a menu, since it holds one thing.
        bar.addAction(item("&About", self._show_about))

    def set_theme(self, name: str) -> None:
        """Dresses the window in a theme and remembers it for next time."""
        if name not in theme.THEMES and name != theme.CUSTOM:
            return
        self._wear_theme(theme_named(name))
        remember_theme(name)
        for action in self.theme_actions.actions():
            action.setChecked(action.data() == name)

    def choose_custom_theme(self) -> None:
        """Picks the colours of a custom theme, showing each change behind
        the dialog and putting the old theme back if it is cancelled."""
        was = current_theme()
        dialog = CustomThemeDialog(custom_colours(), self)
        dialog.changed.connect(lambda colours: self._wear_theme(theme.custom(**colours)))
        self._wear_theme(theme.custom(**dialog.colours))
        if dialog.exec():
            remember_custom_colours(dialog.colours)
            self.set_theme(theme.CUSTOM)
        else:
            self.set_theme(was)

    def _wear_theme(self, chosen: theme.Theme) -> None:
        apply_theme(QApplication.instance(), chosen)
        # The stylesheet reaches most things on its own; these two draw
        # themselves and have to be asked.
        self.viewport.view3d.update()
        self.viewport.gauge.update()

    def _update_menus(self) -> None:
        """Greys out the scan controls that would do nothing, as the panel's
        own buttons are."""
        idle = self.scan_panel.state is ScanState.IDLE
        self.new_scan_item.setEnabled(self.scan_panel.new_scan_button.isEnabled())
        self.complete_item.setEnabled(not idle)
        self.cancel_item.setEnabled(not idle)
        model = self.model_panel.current_model()
        self.save_item.setEnabled(model is not None and model.points is not None)
        self.open_item.setEnabled(idle)  # not in the middle of a scan

    def _show_shortcuts(self) -> None:
        QMessageBox.information(self, "Shortcuts", (
            "<table cellpadding=4>"
            "<tr><td><b>Space</b></td><td>Start a scan, or pause and resume it</td></tr>"
            "<tr><td><b>Esc</b></td><td>Cancel the scan</td></tr>"
            "<tr><td><b>Shift+M</b></td><td>Fullscreen</td></tr>"
            "<tr><td><b>Ctrl+Q</b></td><td>Quit</td></tr>"
            "<tr><td colspan=2>&nbsp;</td></tr>"
            "<tr><td><b>Drag</b></td><td>Turn the view</td></tr>"
            "<tr><td><b>Shift+drag</b></td><td>Move it</td></tr>"
            "<tr><td><b>Wheel</b></td><td>Zoom</td></tr>"
            "<tr><td><b>Double-click</b></td><td>Frame the cloud again</td></tr>"
            "<tr><td><b>Delete</b></td><td>Remove the chosen primitive</td></tr>"
            "</table>"))

    def _show_tips(self) -> None:
        near, far = self.scan_panel.preset.scale_mm
        QMessageBox.information(self, "Scanning tips", (
            f"<p>Keep the object {near}–{far} mm away — the band this scan setting "
            f"looks in — with the marker on the distance gauge in Excellent.</p>"
            "<p>The scanner reads a matte, light-coloured surface best. Black, shiny "
            "and see-through surfaces return little of its pattern, and mesh or hair "
            "give it nothing to match.</p>"
            "<p><b>Auto</b>, beside the depth exposure, tries a few exposures and "
            "keeps the one that sees most of the object. It is worth doing whenever "
            "the lighting or the object changes.</p>"
            "<p>Move slowly: each frame is aligned onto the last, and the Tracking "
            "count in the status bar shows how many are landing.</p>"))

    def _show_about(self) -> None:
        backend = "the mock depth camera"
        if self._pop4:
            backend = self.scan_panel.device.currentText()
        QMessageBox.about(self, "About All3DScan", (
            f"<h3>All3DScan</h3>"
            f"<p>3D scanning on Linux for the Revopoint POP 4.</p>"
            f"<p>Engine {engine.__version__} · PyQt {PYQT_VERSION_STR} · "
            f"Qt {QT_VERSION_STR}<br>Scanning with {backend}.</p>"
            f"<p>A C++ sensor layer and the Qt app's depth decoder, a Rust engine "
            f"for the point clouds and the fits, and this PyQt6 window.</p>"))

    # Scanning: each frame is asked for when the last one has been shown.

    def _on_scan_state(self, state: ScanState) -> None:
        previous, self._state = self._state, state
        self.scan_state.setText(state.value)
        if state is ScanState.SCANNING:
            self._missed = 0
            if previous is ScanState.IDLE:
                self._start_scan()
            self._streaming = True
        else:
            self._streaming = False
            self._fusing = False
            self._next_frame.stop()
            if state is ScanState.PAUSED:
                self.viewport.show_toast("Paused. Press Space to resume.")
        # Live while scanning, and live between scans so the scanner can be
        # aimed. After the timer is stopped above, not before.
        self._request_frame()
        self._update_rea_tools()
        self._update_flashing()

    def _start_scan(self) -> None:
        self._fused = None
        self._fusing = False
        self._tracked = self._offered = 0
        self._tracking.setText("Tracking: —")
        if self.fusion_action.isChecked():
            self.fusion_start_requested.emit(self.FUSION_VOXEL_MM)
        else:
            self.fusion_stop_requested.emit()
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

    def _previewing(self) -> bool:
        """Whether the cameras should be live with no scan running.

        A scanner is aimed before it is used: without this the previews sit
        at "No signal" until New scan is pressed, so there is no way to see
        what the scanner is looking at, whether the object is in range, or
        what Auto exposure did."""
        return (self._pop4 and self._state is ScanState.IDLE
                and not self._tuning and not self._connecting)

    def _request_frame(self) -> None:
        if (self._streaming or self._previewing()) and not self._in_flight:
            self._in_flight = True
            self._requested_at = time.perf_counter()
            self.frame_requested.emit()

    def _on_frame(self, frame, _seconds: float) -> None:
        self._in_flight = False
        self._missed = 0
        # The cameras and the gauge show what the scanner is looking at
        # whether or not a scan is running.
        self.camera_panel.depth_view.show_image(frame.ir)
        self.camera_panel.rgb_view.show_image(frame.rgb)
        self.viewport.gauge.set_depths(frame.depth_mm)
        if self._state is ScanState.IDLE:
            # Aiming, not scanning: nothing here is counted or kept, and the
            # frame that was on its way when a scan ended lands here too.
            self._keep_previewing()
            return
        self.frames += 1
        self.last_frame = frame
        self.viewport.set_frame_count(self.frames)
        if self.fusion_action.isChecked():
            self._offer_to_fusion(frame)
        if self._fused is None:
            self.viewport.show_cloud(frame.points, frame.colors, frame.normals)
        self._show_coverage(frame)
        now = time.perf_counter()
        if self._last_arrival is not None:
            rate = 1.0 / max(now - self._last_arrival, 1e-6)
            self._rate = rate if self._rate == 0 else 0.9 * self._rate + 0.1 * rate
            self._fps.setText(f"FPS: {self._rate:.0f}")
        self._last_arrival = now
        if self._fused is None:  # fusing, the model's count is the useful one
            self._points.setText(f"Points: {len(frame.points):,}")
        if self._streaming:
            wait = 1.0 / self.TARGET_FPS - (now - self._requested_at)
            self._next_frame.start(max(0, int(wait * 1000)))

    def _keep_previewing(self) -> None:
        """Asks for the next preview frame, if the cameras are still live."""
        if self._previewing():
            wait = 1.0 / self.PREVIEW_FPS - (time.perf_counter() - self._requested_at)
            self._next_frame.start(max(0, int(wait * 1000)))

    def _without_speckle(self, cloud: tuple) -> tuple:
        """The model with the stray points taken out. A scan leaves specks
        hanging in mid-air where the depth was a bad match, and they would
        otherwise be fitted and meshed along with the object."""
        try:
            cleaned = engine.remove_speckle(*cloud, radius_mm=self.SPECKLE_RADIUS_MM,
                                            share=self.SPECKLE_SHARE)
        except BaseException as exc:
            print(f"could not remove the speckle: {describe(exc)}", file=sys.stderr,
                  flush=True)
            return cloud
        dropped = len(cloud[0]) - len(cleaned[0])
        if not len(cleaned[0]):
            print("every point looked like speckle; keeping the cloud as it was",
                  file=sys.stderr, flush=True)
            return cloud
        print(f"removed {dropped:,} stray points of {len(cloud[0]):,}", flush=True)
        if dropped:
            self.viewport.show_toast(f"Removed {dropped:,} stray points", 4000)
        return cleaned

    def _offer_to_fusion(self, frame) -> None:
        """Hands the fusion this frame, unless it is still on the last one."""
        if self._fusing or not len(frame.points):
            return
        self._fusing = True
        self._offered += 1
        step = max(math.ceil(len(frame.points) / self.FUSION_MAX_POINTS), 1)
        self.fuse_requested.emit(frame.points[::step], frame.colors[::step],
                                 frame.normals[::step], frame.depth_mm)

    def build_surface(self) -> None:
        """Melts the scan's depth views into one surface and shows it."""
        model = self.model_panel.current_model()
        if model is None or self._meshing or not model.intrinsics:
            return
        if model.surface is not None:  # built already; just show it again
            self._show_surface(model)
            return
        self._meshing = True
        self._update_rea_tools()
        self.viewport.show_toast("Building the surface…", 120_000)
        self.surface_requested.emit(model, tuple(model.intrinsics), self.SURFACE_VOXEL_MM)

    def _on_surface_ready(self, surface, model) -> None:
        self._meshing = False
        self._update_rea_tools()
        if surface is None:
            self.viewport.show_toast("No surface could be built from this scan", 6000)
            print("no surface could be built", file=sys.stderr, flush=True)
            return
        model.surface = surface
        print(f"{model.name}: surface of {len(surface[2]):,} triangles over "
              f"{len(surface[0]):,} vertices", flush=True)
        if self.model_panel.current_model() is model:
            self._show_surface(model)

    def _show_surface(self, model) -> None:
        vertices, normals, indices = model.surface
        self.viewport.show_surface(vertices, normals, indices)
        self.viewport.show_toast(f"{len(indices):,} triangles", 4000)

    def _on_model_ready(self, model, display, fit: dict) -> None:
        self._fusing = False
        if model is None:
            # The frame would not sit on the model. Say so, and carry on:
            # the next one may well align.
            self._tracking.setText(f"Tracking: {self._tracked}/{self._offered}")
            return
        self._tracked += 1
        self._fused = model
        self._tracking.setText(f"Tracking: {self._tracked}/{self._offered}")
        if self._state is not ScanState.IDLE:
            self.viewport.show_cloud(*display, frame=self._tracked == 1)
            self._points.setText(f"Points: {len(model[0]):,}")

    def _on_fusion_failed(self, message: str) -> None:
        self._fusing = False
        print(f"fusion failed: {message}", file=sys.stderr, flush=True)
        self.viewport.show_toast(f"Fusion failed: {message}", 6000)
        self.fusion_action.setChecked(False)

    def _show_coverage(self, frame) -> None:
        """Reports how much of the depth camera got a distance, and says so
        when the scanner is getting too little to scan with."""
        points = len(frame.points)
        self._coverage.setText(f"Coverage: {np.mean(frame.depth_mm > 0):.1%}")
        if points >= self.TOO_FEW_POINTS:
            self._low_coverage = 0
            self.viewport.show_warning("")
            return
        self._low_coverage += 1
        if self._low_coverage == self.LOW_COVERAGE_FRAMES:
            near, far = self.scan_panel.preset.scale_mm
            self.viewport.show_warning(
                f"Only {points:,} points a frame. Check that the object is {near}–{far} mm "
                f"away — the distance this scan setting looks in — and that it is not too "
                f"dark or too shiny for the scanner's pattern to show on it. A longer depth "
                f"exposure often helps."
                if self._pop4 else f"Only {points:,} points a frame.")

    def _on_frame_missed(self, reason: str) -> None:
        self._in_flight = False
        if self._state is ScanState.IDLE:
            # Only aiming: a dropped frame costs nothing, so keep looking
            # rather than giving up on the preview.
            self._keep_previewing()
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
        fused = self._fused
        points = fused[0] if fused is not None else (frame.points if frame is not None else None)
        properties = {
            "Model name": name,
            "Scanner": panel.device.currentText(),
            "Rendering state": "Raw",
            "Rendering frames": str(self.frames),
            "Points": (f"{len(points):,} fused from {self._tracked} frames"
                       if fused is not None else
                       f"{len(points):,} in the last frame" if points is not None else "—"),
            "Scan settings": f"{panel.accuracy.currentText()} · "
                             f"{panel.tracking.currentText()} tracking · "
                             f"{panel.object_type.currentText()}",
        }
        if fused is not None:
            fused = self._fused = self._without_speckle(fused)
            points = fused[0]
            properties["Points"] = f"{len(points):,} fused from {self._tracked} frames"
        # The cloud already on screen becomes the model: the frames fused
        # together, or the last frame alone when fusion was off.
        if fused is not None:
            model = ScanModel(name, properties, *fused, intrinsics=self._intrinsics)
        elif frame is not None:
            model = ScanModel(name, properties, frame.points, frame.colors, frame.normals)
        else:
            model = ScanModel(name, properties)
        self._shown_model = model  # already on screen: no need to draw it again
        self.left.setCurrentWidget(self.model_panel)
        self.viewport.set_stage(scanning=False)
        if model.points is None:
            self.viewport.clear_cloud(f"{name}: no frames captured")
        elif fused is not None:
            # While scanning the viewport was given a cloud already thinned
            # for it; hand it the whole model now, so that what it draws is
            # a thinning of the model a fit will be measured against and the
            # two line up point for point. Before listing the model, which
            # selects it and tints its points.
            self.viewport.show_cloud(*fused)
        self.model_panel.add_model(model)
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
        # Saving wants points, not a fit: a model with no primitives in it
        # is still worth keeping.
        self.export_action.setEnabled(self._state is ScanState.IDLE and model is not None
                                      and model.points is not None)
        # Meshing melts the scan's own depth views together, so it needs a
        # scan taken here. A cloud opened from a file has no views: say why
        # rather than leaving a dead button.
        can_mesh = ready and not self._meshing and bool(model.intrinsics)
        self.mesh_action.setEnabled(can_mesh)
        self.mesh_action.setToolTip(
            self._mesh_tip if can_mesh or model is None or model.intrinsics else
            "Meshing builds the surface from the scan's own depth views, which a scan "
            "opened from a file does not carry. Open a file that already holds a mesh, "
            "or scan the object here.")
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

    # Scans from elsewhere: reading them in and writing them out.

    #: How small a scan has to measure before it is taken to be in metres.
    #: A real scanned object is tens of millimetres across at the least.
    METRES_LOOK_LIKE_MM = 10.0

    def _file_filters(self) -> tuple[str, dict[str, str]]:
        """The save dialog's filters, and which format each one names."""
        formats, by_filter = [], {}
        for name, what, extension, _mesh, _colour in engine.scan_formats():
            text = f"{what} (*.{extension})"
            formats.append(text)
            by_filter[text] = name
        return ";;".join(formats), by_filter

    def open_scan(self, path: str | Path | None = None) -> ScanModel | None:
        """Opens a scan another tool or another scanner wrote.

        Everything the app does after capture — cleaning, meshing, fitting,
        exporting — works on it exactly as on a scan taken here, so a
        scanner All3DScan cannot drive is still a scanner it can work with.
        """
        if path is None:
            path, _ = QFileDialog.getOpenFileName(
                self, "Open scan", str(Path.home()),
                "Scans (*.ply *.obj *.stl *.pcd *.asc *.xyz *.pts *.txt *.p3);;"
                "All files (*)")
            if not path:
                return None
        path = Path(path)
        self.setCursor(Qt.CursorShape.WaitCursor)
        try:
            read = engine.read_scan(str(path))
        except (ValueError, OSError) as exc:
            print(f"opening {path} failed: {describe(exc)}", file=sys.stderr, flush=True)
            QMessageBox.warning(self, "Could not open that scan", str(exc))
            return None
        finally:
            self.unsetCursor()

        points, colors = read["points"], read["colors"]
        normals, triangles = read["normals"], read["triangles"]
        points = self._as_millimetres(path, points, read["extent"])
        if normals is None:
            # The fitting needs a normal per point, and a file need not carry
            # them. Build them from the surface the points already describe.
            normals = self._normals_for(points, triangles)

        self._scans += 1
        name = path.stem or f"Scan {self._scans:02d}"
        low, high = read["extent"]
        size = tuple(round(h - l, 1) for l, h in zip(low, high))
        model = ScanModel(name, {
            "Model name": name,
            "Scanner": f"Opened from {path.name}",
            "Rendering state": "Meshed" if triangles is not None else "Raw",
            "Rendering frames": "—",
            "Points": f"{len(points):,} read from the file",
            "Scan settings": f"{size[0]} × {size[1]} × {size[2]} mm"
                             + (f" · {len(triangles):,} triangles" if triangles is not None else ""),
        }, points, colors if colors is not None else self._plain_colors(points), normals)
        if triangles is not None:
            model.surface = (points, normals, triangles)
        self._shown_model = None
        self.left.setCurrentWidget(self.model_panel)
        self.viewport.set_stage(scanning=False)
        self.model_panel.add_model(model)
        print(f"{name}: opened {len(points):,} points"
              + (f" and {len(triangles):,} triangles" if triangles is not None else "")
              + f" from {path}", flush=True)
        self.viewport.show_toast(f"Opened {len(points):,} points from {path.name}", 5000)
        return model

    def _as_millimetres(self, path: Path, points, extent) -> np.ndarray:
        """Scans are in millimetres here. A file written in metres measures
        a thousand times too small, which is plain enough to see, so ask
        rather than either guessing or leaving a model nothing will fit."""
        low, high = extent
        across = max(h - l for l, h in zip(low, high))
        if not 0 < across < self.METRES_LOOK_LIKE_MM:
            return points
        answer = QMessageBox.question(
            self, "Is this in metres?",
            f"{path.name} measures {across:.3g} across, which is too small for "
            f"millimetres — it was probably written in metres.\n\n"
            f"Scale it by 1000 to millimetres?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.Yes)
        if answer is QMessageBox.StandardButton.Yes:
            print(f"{path.name}: scaled from metres to millimetres", flush=True)
            return points * 1000.0
        return points

    @staticmethod
    def _plain_colors(points) -> np.ndarray:
        """One colour per point for a file that carried none."""
        return np.full((len(points), 3), 200, np.uint8)

    @staticmethod
    def _normals_for(points, triangles) -> np.ndarray:
        """Normals for a file that carried none.

        Fitting a plane or a cylinder needs a direction per point, so a file
        without them would otherwise open into a model nothing will fit.
        They come from the triangles where there are triangles, and
        otherwise from the shape the points themselves describe."""
        if triangles is not None and len(triangles):
            normals = np.zeros((len(points), 3), np.float32)
            corners = points[triangles.astype(np.int64)]
            faces = np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0])
            for corner in range(3):
                np.add.at(normals, triangles[:, corner].astype(np.int64), faces)
            length = np.linalg.norm(normals, axis=1, keepdims=True)
            return np.divide(normals, length, out=normals,
                             where=length > 1e-12).astype(np.float32)
        return engine.estimate_normals(points)

    def save_scan(self, path: str | Path | None = None,
                  format: str | None = None) -> Path | None:
        """Writes the model showing to a file another tool can open."""
        model = self.model_panel.current_model()
        if model is None or model.points is None:
            self.viewport.show_toast("There is no model to save", 4000)
            return None
        filters, by_filter = self._file_filters()
        if path is None:
            path, chosen = QFileDialog.getSaveFileName(
                self, "Save scan as", str(Path.home() / f"{model.name}.ply"), filters)
            if not path:
                return None
            format = format or by_filter.get(chosen)
        path = Path(path)
        keeps_mesh = dict((name, mesh) for name, _w, _e, mesh, _c in engine.scan_formats())
        # A meshed model is saved as its surface wherever the format holds
        # one, and as its points where it does not.
        as_mesh = model.surface is not None and keeps_mesh.get(format or "ply", False)
        if as_mesh:
            vertices, normals, triangles = model.surface
            points, colors = vertices, None
        else:
            points, colors, normals, triangles = (model.points, model.colors,
                                                  model.normals, None)
        self.setCursor(Qt.CursorShape.WaitCursor)
        try:
            written = engine.write_scan(str(path), points, colors=colors, normals=normals,
                                        triangles=triangles, format=format)
        except (ValueError, OSError) as exc:
            print(f"saving {path} failed: {describe(exc)}", file=sys.stderr, flush=True)
            QMessageBox.warning(self, "Could not save that scan", str(exc))
            return None
        finally:
            self.unsetCursor()
        written = Path(written)
        what = (f"{len(triangles):,} triangles" if as_mesh else f"{len(points):,} points")
        print(f"{model.name}: saved {what} to {written}", flush=True)
        self.viewport.show_toast(f"Saved {what} to {written.name}", 5000)
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

    def _look_for_scanners(self) -> None:
        """Keeps the scanner list up to date with what is on the USB bus,
        including the scanners this cannot drive."""
        try:
            self._other_scanners = [s for s in scanners.plugged_in() if not s.drivable]
        except OSError as exc:  # sysfs unreadable: carry on with what we have
            print(f"looking for scanners failed: {describe(exc)}", file=sys.stderr, flush=True)
            return
        self.scan_panel.show_other_scanners(self._other_scanners)

    def _check_pop4_present(self) -> None:
        self._look_for_scanners()
        present = bool(self._pop4_present())
        self.scan_panel.set_pop4_present(present or self.scan_panel.pop4)
        if self._pop4 and self._absent_ticks >= self.ABSENT_LOOKS and not self._connecting:
            # Unplugged between scans, where no capture is running to notice.
            self._on_sensor_lost("it is no longer on the USB bus")
            return
        # A scanner just back on the bus re-enumerates for a moment, so wait
        # for a second look before opening it.
        self._present_ticks = self._present_ticks + 1 if present else 0
        self._absent_ticks = 0 if present else self._absent_ticks + 1
        if self._absent_ticks == self.ABSENT_LOOKS:
            # Off the bus long enough to count as unplugged. Whenever it comes
            # back it gets a fresh set of tries, so a replug always reconnects
            # however many times it has been tried before.
            self._reconnects = 0
        wanted = self.scan_panel.pop4 and not self._pop4 and not self._connecting
        if wanted and self._present_ticks >= 2 and self._reconnects < self.MAX_RECONNECTS:
            self._reconnects += 1
            print(f"opening the POP 4 (try {self._reconnects} of "
                  f"{self.MAX_RECONNECTS})", flush=True)
            self._connect_pop4()

    def _on_device(self, key: str) -> None:
        self.camera_panel.set_pop4(False)
        self.scan_panel.set_ready(False)
        self._reconnects = 0
        if key == "pop4":
            self._connect_pop4()
            return
        self._connecting = False
        self.viewport.show_warning("")
        if key == "mock":
            self.mock_requested.emit()
            return
        # A scanner this cannot drive. Say what it is and what does work
        # with it, rather than failing to connect to it.
        self._explain_other_scanner(key)

    def _explain_other_scanner(self, key: str) -> None:
        found = next((s for s in self._other_scanners if s.key == key), None)
        if found is None:
            return
        self.scan_panel.set_connection("Cannot scan", ok=False)
        self._connection.setText('<span style="color:#e0a83e">●</span> '
                                 f"{found.vendor} {found.key} — no Linux protocol yet")
        print(f"{found.key} is recognised but cannot be scanned from: {found.label}",
              flush=True)
        self.viewport.show_warning(found.why_not())

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
        self._intrinsics = tuple(settings.get("intrinsics", ()))
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
            self._request_frame()  # the cameras come up with the scanner
            print(f"POP 4 {settings['serial']} connected: {settings['backend']}", flush=True)
        self._show_connection(self._pop4, settings.get("serial", ""))
        self.scan_panel.set_ready(True)

    def _on_open_failed(self, reason: str) -> None:
        self._connecting = False
        print(f"opening the POP 4 failed: {reason}", file=sys.stderr, flush=True)
        if 0 < self._reconnects < self.MAX_RECONNECTS:
            # Lost mid-session and not back yet; _check_pop4_present retries.
            self._show_waiting(reason)
            self.scan_panel.set_ready(self.sensor is not None)
            return
        self.viewport.show_toast(f"Could not open the POP 4: {reason}", 8000)
        if self.sensor is None:
            # The POP 4 is the only scanner there is. Wait for it rather than
            # leaving a window that looks ready to scan nothing.
            self._show_waiting(reason)
            return
        self.viewport.show_warning("")
        self.scan_panel.device.setCurrentIndex(
            self.scan_panel.device.findData("mock"))  # back to the stand-in

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
        self._present_ticks = self._absent_ticks = 0
        self.camera_panel.set_pop4(False)
        self.camera_panel.depth_view.clear()
        self.camera_panel.rgb_view.clear()
        print(f"the POP 4 disconnected: {reason}", file=sys.stderr, flush=True)
        self.scan_panel.cancel()  # a scan cannot go on without the scanner
        self.scan_panel.set_ready(self.sensor is not None)
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
        if not pop4 and self.sensor is None:
            self._connection.setText('<span style="color:#e0a83e">●</span> '
                                     "No scanner — plug in a POP 4")
            self.scan_panel.set_connection("No scanner", ok=False)
            return
        name = f"Revopoint POP 4 {serial}".rstrip() if pop4 else "Mock depth sensor"
        self._connection.setText(f'<span style="color:#5bbf5b">●</span> {name} connected')
        self.scan_panel.set_connection("Connected")

    def _control(self, name: str, *args) -> None:
        """Has the POP 4 run one of its methods, between captures."""
        if self._pop4:
            self.control_requested.emit(name, args)

    def _flash_lights(self) -> None:
        """Turns the scanner's white lights on and off while a scan runs."""
        self._flash_on = not self._flash_on
        self._control("set_light", "fill light", self._flash_on)

    def _update_flashing(self) -> None:
        """The lights flash only while a scan is running with them chosen."""
        wanted = (self._pop4 and self._state is ScanState.SCANNING
                  and self.camera_panel.fill_light_button.isChecked())
        if wanted and not self._flash.isActive():
            self._flash_on = False
            self._flash.start(self.FLASH_MS)
        elif not wanted and self._flash.isActive():
            self._flash.stop()
            if self._flash_on:  # never leave them lit
                self._flash_on = False
                self._control("set_light", "fill light", False)

    def _tune_exposure(self) -> None:
        """Has the scanner try a few exposures and keep the best."""
        if not self._pop4 or self._tuning:
            return
        self._tuning = True
        self.camera_panel.exposure_box.setEnabled(False)
        tries = len(ScanWorker.EXPOSURES)
        self.viewport.show_toast(f"Trying {tries} exposures to see which shows most of the "
                                 f"object, then narrowing onto where it is…", 60_000)
        self.tune_exposure_requested.emit(tuple(map(float, self.scan_panel.preset.scale_mm)))

    def _on_exposure_tuned(self, result: dict) -> None:
        self._tuning = False
        self.camera_panel.exposure_box.setEnabled(self._pop4)
        self._request_frame()  # the sweep had the scanner to itself
        tried = ", ".join(f"{us / 1000:.1f} ms: {points:,}" for us, points in result["tried"])
        if result.get("trouble"):
            print(f"auto exposure: {result['trouble']} on the way, settings skipped",
                  file=sys.stderr, flush=True)
        if "error" in result:
            print(f"auto exposure failed: {result['error']} (tried {tried})",
                  file=sys.stderr, flush=True)
            self.viewport.show_toast(f"Auto exposure failed: {result['error']}", 6000)
            return
        chosen = result["exposure_us"]
        self.camera_panel.show_exposure(chosen)
        print(f"auto exposure: kept {chosen} us (points a frame — {tried})", flush=True)
        told = f"Depth exposure {chosen / 1000:.1f} ms, which saw the most of the object"
        band = result.get("range_mm")
        if band:
            self.viewport.gauge.set_bands(self.scan_panel.preset.band_edges_mm, band)
            self.scan_panel.show_scanning_distance(*band)
            print(f"auto range: looking {band[0]:.0f}–{band[1]:.0f} mm, where the object is",
                  flush=True)
            told += f"; looking {band[0]:.0f}–{band[1]:.0f} mm, where the object is"
        self.viewport.show_toast(told, 5000)

    def _on_light_toggled(self, light: str, on: bool) -> None:
        """The white lights flash while a scan runs, so the button arms them
        rather than switching a steady light."""
        if light == "fill light":
            self._update_flashing()
            if self._flash.isActive():
                return
        self._control("set_light", light, on)

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
        for thread in (self._thread, self._rea_thread, self._fusion_thread):
            if thread.isRunning():
                thread.quit()
                thread.wait()
        self._worker.close()  # its thread has stopped, so it is safe here

    def closeEvent(self, event) -> None:
        self._stop_worker()
        super().closeEvent(event)


def cloud_pixels(window: MainWindow) -> tuple[int, int]:
    """How many pixels of the 3D view show the cloud in its scan colour —
    green while scanning, blue once complete — and how many in a fitted
    primitive's tint, as last drawn."""
    rgb = image_pixels(window.viewport.view3d.grabFramebuffer()).astype(np.int16)
    red, green, blue = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    # By ratio, not by a fixed margin: shading takes a splat down to a third
    # of full brightness, where any fixed margin stops holding.
    scan_blue = blue > 1.4 * np.maximum(red, green)
    scan_green = green > 1.4 * np.maximum(red, blue)
    # Amber, pink and the rest of the palette: never grey, nor a scan colour.
    tinted = (np.maximum(red, green) > blue + 15) & ~scan_green
    return int((scan_blue | scan_green).sum()), int(tinted.sum())


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
        report["cloud pixels"], _ = cloud_pixels(window)
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
              and report["cloud pixels"] > 1000 and report["model"] and report.get("plane")
              and report.get("cylinder") and report["tinted pixels"] > 1000
              and report["STEP bodies"] == ["Cylinder 1", "Plane 1"])
        print(f"smoke test: {frames} frames streamed; previews "
              f"{'filled' if report['previews'] else 'EMPTY'}; 3D view showing "
              f"{report['shown']:,} points ({report['cloud pixels']:,} pixels of cloud); "
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
    parser.add_argument("--mock", action="store_true",
                        help="also offer a stand-in depth camera, for working on the app "
                             "without a scanner. What it produces is not a scan of anything.")
    args, qt_args = parser.parse_known_args(argv)

    qt_argv = [sys.argv[0], *qt_args]
    configure_opengl(qt_argv)  # before the QApplication, which reads it
    app = QApplication(qt_argv)
    warn_if_platform_differs()
    app.setApplicationName("All3DScan")
    app.setDesktopFileName(APP_ID)
    if APP_ICON.exists():
        app.setWindowIcon(QIcon(str(APP_ICON)))
    apply_theme(app, theme_named(current_theme()))

    def report_unhandled(exc_type, exc, tb) -> None:
        # With the default hook, PyQt6 aborts the whole app when a slot raises.
        traceback.print_exception(exc_type, exc, tb)
        if args.smoke_test:
            app.exit(1)

    sys.excepthook = report_unhandled
    # The smoke test runs with no one watching, so it scans the stand-in.
    mock = engine.MockDepthSensor(640, 480, seed=0) if args.mock or args.smoke_test else None
    window = MainWindow(mock)
    window.show()
    if args.smoke_test:
        run_smoke_test(app, window)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
