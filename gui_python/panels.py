"""The side panels and the viewport of the main window."""

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from PyQt6.QtCore import QSize, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QIcon, QKeySequence, QPainter, QPixmap, QShortcut
from PyQt6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QRadioButton,
    QSizePolicy,
    QSlider,
    QStackedWidget,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

import rea_engine as rea
from presets import ACCURACY, OBJECT_TYPES, TRACKING, Preset, find_preset, load_presets
from viewport3d import SCAN_BLUE, SCAN_GREEN, PointCloudView
from widgets import DistanceGauge, VideoPreview


def section_title(text: str) -> QLabel:
    label = QLabel(text)
    label.setObjectName("sectionTitle")
    return label


def hint(text: str) -> QLabel:
    label = QLabel(text)
    label.setObjectName("hint")
    label.setWordWrap(True)
    return label


def toggle(text: str, checked: bool = False) -> QPushButton:
    button = QPushButton(text)
    button.setCheckable(True)
    button.setChecked(checked)
    return button


def slider(minimum: int, maximum: int, value: int, describe) -> tuple[QSlider, QHBoxLayout]:
    """A horizontal slider with a readout that describe() fills in."""
    control = QSlider(Qt.Orientation.Horizontal)
    control.setRange(minimum, maximum)
    readout = QLabel()
    readout.setMinimumWidth(48)
    control.valueChanged.connect(lambda v: readout.setText(describe(v)))
    control.setValue(value)
    readout.setText(describe(value))
    row = QHBoxLayout()
    row.addWidget(control)
    row.addWidget(readout)
    return control, row


class ScanState(Enum):
    IDLE = "Ready"
    SCANNING = "Scanning"
    PAUSED = "Paused"


def debounced(control: QSlider, emit, ms: int = 250) -> QTimer:
    """Calls emit(value) once a slider settles: on release after a drag, and
    a moment after a change by keyboard or by the program. Stopping the
    timer it returns drops a change still waiting."""
    timer = QTimer(control)
    timer.setSingleShot(True)
    timer.setInterval(ms)
    timer.timeout.connect(lambda: emit(control.value()))
    control.valueChanged.connect(lambda _value: control.isSliderDown() or timer.start())
    control.sliderReleased.connect(timer.start)
    return timer


class CameraPanel(QWidget):
    """Left, while scanning: the camera parameters over the live feeds."""

    light_toggled = pyqtSignal(str, bool)  # "projector" or "fill light", on
    light_level_changed = pyqtSignal(str, int)  # the light, 0..255
    exposure_changed = pyqtSignal(int)  # microseconds
    auto_exposure_requested = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        # The range the POP 4 backend accepts (src/protocol/RevopointPop4Protocol.cpp).
        self.exposure, exposure_row = slider(500, 6000, 3000, lambda us: f"{us / 1000:.1f} ms")
        self.exposure.setSingleStep(100)
        self.exposure.setPageStep(500)
        self.auto_exposure = QPushButton("Auto")
        self.auto_exposure.setToolTip(
            "Try a few exposures and keep the one that sees the most of the object.\n"
            "Takes about ten seconds, during which the scanner is busy.")
        self.auto_exposure.clicked.connect(self.auto_exposure_requested)
        exposure_row.insertWidget(0, self.auto_exposure)
        self._syncing = False  # while showing the scanner's settings, send nothing
        self._settling = [debounced(
            self.exposure, lambda us: self._syncing or self.exposure_changed.emit(us))]
        self.exposure_box = QWidget()
        form = QFormLayout(self.exposure_box)
        form.setContentsMargins(0, 0, 0, 0)
        form.addRow("Depth exposure", exposure_row)

        # The POP 4's own lights, as the legacy Scanner Controls panel drives
        # them (docs/protocol/RE_NOTES.md, 2026-10-14). Its alignment light is
        # left out, as there: unlike these, it was never confirmed on the
        # scanner. The blue laser is a scan mode, on the right.
        self.lights = QWidget()
        form = QFormLayout(self.lights)
        form.setContentsMargins(0, 0, 0, 0)
        self.projector_button = toggle("Pattern projector", checked=True)
        self.projector_button.setToolTip(
            "The infrared projector that patterns the depth camera's view. Depth comes from\n"
            "a second speckle emitter that stays on, so switching this does not change it.")
        self.fill_light_button = toggle("Flash light")
        self.fill_light_button.setToolTip(
            "The scanner's white lights, for the colour camera.\n"
            "They flash while a scan runs rather than staying on.")
        self.fill_light_level, fill_row = slider(0, 255, 255, str)
        form.addRow(self.projector_button)
        form.addRow(self.fill_light_button)
        form.addRow("Flash level", fill_row)
        self.projector_button.toggled.connect(
            lambda on: self._syncing or self.light_toggled.emit("projector", on))
        self.fill_light_button.toggled.connect(
            lambda on: self._syncing or self.light_toggled.emit("fill light", on))
        self._settling.append(debounced(
            self.fill_light_level,
            lambda level: self.light_level_changed.emit("fill light", level)))
        self.pop4_hint = hint("Exposure and lights need a connected POP 4.")

        self.depth_view = VideoPreview("Depth camera")
        self.rgb_view = VideoPreview("RGB camera")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(10)
        layout.addWidget(section_title("Camera parameters"))
        layout.addWidget(self.exposure_box)
        layout.addWidget(self.lights)
        layout.addWidget(self.pop4_hint)
        layout.addWidget(self.depth_view, 1)
        layout.addWidget(self.rgb_view, 1)
        self.set_pop4(False)

    def set_pop4(self, pop4: bool) -> None:
        # The mock sensor has no exposure or lights; the POP 4 has both.
        self.exposure_box.setEnabled(pop4)
        self.lights.setEnabled(pop4)
        self.pop4_hint.setVisible(not pop4)

    def show_controls(self, lights: dict[str, tuple[bool, int]], exposure_us: int) -> None:
        """Shows the scanner's own settings, without sending them back to it."""
        self._syncing = True
        self.projector_button.setChecked(lights["projector"][0])
        self.fill_light_button.setChecked(lights["fill light"][0])
        self.fill_light_level.setValue(lights["fill light"][1])
        self.show_exposure(exposure_us)
        self._syncing = False

    def show_exposure(self, exposure_us: int) -> None:
        """Shows an exposure the scanner already has, without sending it back."""
        was = self._syncing
        self._syncing = True
        self.exposure.setValue(exposure_us)
        for timer in self._settling:
            timer.stop()
        self._syncing = was

    def clear_feeds(self) -> None:
        self.depth_view.clear()
        self.rgb_view.clear()


@dataclass(eq=False)
class ScanModel:
    """A completed scan: its properties, its point cloud (none if no frame
    came) and the primitives fitted to that cloud."""

    name: str
    properties: dict[str, str]
    points: np.ndarray | None = None
    colors: np.ndarray | None = None
    normals: np.ndarray | None = None
    # The surface built from the scan's depth views, once it has been.
    surface: tuple | None = None
    # The depth camera's fx, fy, cx, cy, which the surface is built with.
    intrinsics: tuple = ()
    session: rea.ReaSession | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        if self.session is None and self.points is not None:
            self.session = rea.ReaSession(self.points, self.normals)


def swatch(colour: str, size: int = 12) -> QIcon:
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.setBrush(QColor(colour))
    painter.setPen(Qt.PenStyle.NoPen)
    painter.drawRoundedRect(0, 0, size, size, 3, 3)
    painter.end()
    return QIcon(pixmap)


def tint(session: rea.ReaSession, selected=None) -> np.ndarray:
    """An (N, 4) RGBA tint for a cloud: each primitive's points in its
    colour, and when one is selected, the others darker. (Fainter would mix
    in the scan blue, which turns amber grey.)"""
    highlight = np.zeros((len(session.points), 4), np.uint8)
    for primitive in session.primitives:
        shade = 1.0 if selected is None or primitive is selected else 0.5
        highlight[primitive.claimed] = (*(round(c * shade) for c in rea.rgb(primitive.color)),
                                        255)
    return highlight


class ModelPanel(QWidget):
    """Left, after a scan: the models, the selected one's properties and the
    primitives fitted to it."""

    FIELDS = ("Model name", "Scanner", "Rendering state", "Rendering frames", "Points",
              "Scan settings")

    model_changed = pyqtSignal(object)  # ScanModel or None
    primitive_changed = pyqtSignal(object)  # rea.Plane, rea.Cylinder or None
    remove_requested = pyqtSignal(object)  # the primitive to remove

    def __init__(self) -> None:
        super().__init__()
        self.models = QListWidget()
        self.models.setObjectName("models")
        self._values = {field: QLabel("—") for field in self.FIELDS}
        properties = QFormLayout()
        for field, value in self._values.items():
            value.setWordWrap(True)
            properties.addRow(field, value)

        self.primitives = QListWidget()
        self.primitives.setObjectName("primitives")
        self.primitive_details = QLabel()
        self.primitive_details.setObjectName("details")
        self.primitive_details.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.tolerance = QDoubleSpinBox()
        self.tolerance.setRange(0.1, 10.0)
        self.tolerance.setSingleStep(0.1)
        self.tolerance.setDecimals(1)
        self.tolerance.setSuffix(" mm")
        self.tolerance.setValue(rea.DEFAULT_TOLERANCE_MM)
        self.tolerance.setToolTip("How far a point may be from a fitted surface and still count")
        self.remove_button = QPushButton("Remove")
        self.remove_button.setToolTip("Remove the primitive and free its points (Delete)")
        self.remove_button.clicked.connect(self._remove)
        delete = QShortcut(QKeySequence(Qt.Key.Key_Delete), self.primitives)
        delete.setContext(Qt.ShortcutContext.WidgetShortcut)
        delete.activated.connect(self._remove)
        fit_row = QHBoxLayout()
        fit_row.addWidget(QLabel("Tolerance"))
        fit_row.addWidget(self.tolerance)
        fit_row.addStretch()
        fit_row.addWidget(self.remove_button)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(10)
        layout.addWidget(section_title("Models"))
        layout.addWidget(self.models, 1)
        layout.addWidget(section_title("Properties"))
        layout.addLayout(properties)
        layout.addWidget(section_title("Fitted primitives"))
        layout.addWidget(self.primitives, 1)
        layout.addWidget(self.primitive_details)
        layout.addLayout(fit_row)
        self._busy = False  # while a fit runs
        self.models.currentItemChanged.connect(self._show)
        self.primitives.currentItemChanged.connect(self._show_primitive)
        self._show(None)

    def add_model(self, model: ScanModel) -> None:
        item = QListWidgetItem(model.name)
        item.setData(Qt.ItemDataRole.UserRole, model)
        self.models.addItem(item)
        self.models.setCurrentItem(item)

    def current_model(self) -> ScanModel | None:
        item = self.models.currentItem()
        return item.data(Qt.ItemDataRole.UserRole) if item else None

    def current_primitive(self):
        item = self.primitives.currentItem()
        return item.data(Qt.ItemDataRole.UserRole) if item else None

    def tolerance_mm(self) -> float:
        return self.tolerance.value()

    def value(self, field: str) -> str:
        return self._values[field].text()

    def refresh_primitives(self, select=None) -> None:
        """Lists the current model's primitives again, selecting `select`."""
        model = self.current_model()
        self.primitives.blockSignals(True)
        self.primitives.clear()
        for primitive in model.session.primitives if model and model.session else []:
            item = QListWidgetItem(swatch(primitive.color),
                                   f"{primitive.name}  ·  {primitive.summary()}")
            item.setData(Qt.ItemDataRole.UserRole, primitive)
            self.primitives.addItem(item)
            if primitive is select:
                self.primitives.setCurrentItem(item)
        self.primitives.blockSignals(False)
        self._show_primitive(self.primitives.currentItem())

    def set_busy(self, busy: bool) -> None:
        """While a fit runs, the primitives stay as they are."""
        self._busy = busy
        self.remove_button.setEnabled(not busy and self.current_primitive() is not None)

    def _show(self, item: QListWidgetItem | None, _previous=None) -> None:
        model = item.data(Qt.ItemDataRole.UserRole) if item else None
        properties = model.properties if model else {}
        for field, value in self._values.items():
            value.setText(properties.get(field, "—"))
        self.refresh_primitives()
        self.model_changed.emit(model)

    def _show_primitive(self, item: QListWidgetItem | None, _previous=None) -> None:
        primitive = item.data(Qt.ItemDataRole.UserRole) if item else None
        if primitive is None:
            model = self.current_model()
            fittable = model is not None and model.session is not None
            self.primitive_details.setText(
                "Fit a plane or a cylinder from the REA tools above." if fittable else "")
        else:
            rows = "".join(f"<tr><td style='color:#8b8e94; padding-right:10px'>{key}</td>"
                           f"<td>{value}</td></tr>" for key, value in primitive.describe())
            self.primitive_details.setText(f"<table>{rows}</table>")
        self.remove_button.setEnabled(primitive is not None and not self._busy)
        self.primitive_changed.emit(primitive)

    def _remove(self) -> None:
        primitive = self.current_primitive()
        if primitive is not None and not self._busy:
            self.remove_requested.emit(primitive)


class ScanPanel(QWidget):
    """Right: the scanner, New scan / Pause / Complete and the scan settings."""

    state_changed = pyqtSignal(object)  # ScanState
    completed = pyqtSignal()
    cancelled = pyqtSignal()
    preset_changed = pyqtSignal(object)  # Preset
    device_changed = pyqtSignal(bool)  # True when the POP 4 is the device
    mode_changed = pyqtSignal(str)  # "infrared", "blue laser" or "speckle", as picked
    MODES = {"infrared": "Infrared", "blue laser": "Blue laser (visible)",
             "speckle": "Infrared speckle"}

    def __init__(self) -> None:
        super().__init__()
        self.connection = QLabel("Connected")
        self.connection.setObjectName("connection")
        # Red while nothing is there to scan with, so the heading beside the
        # scanner list never claims a connection the app does not have.
        self.connection_ok = True
        self.device = QComboBox()
        self.device.addItems(["Mock depth sensor", "Revopoint POP 4 (USB)"])
        self.set_pop4_present(False)

        self.new_scan_button = self._scan_button("New scan", "list-add")
        self.pause_button = self._scan_button("Pause", "media-playback-pause")
        self.complete_button = self._scan_button("Complete", "dialog-ok-apply")
        self.complete_button.setObjectName("complete")
        self.new_scan_button.clicked.connect(self.new_scan)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.complete_button.clicked.connect(self.complete)
        buttons = QHBoxLayout()
        buttons.setSpacing(8)
        for button in (self.new_scan_button, self.pause_button, self.complete_button):
            buttons.addWidget(button)

        self.mode = QComboBox()
        for key, name in self.MODES.items():
            self.mode.addItem(name, key)
        self.mode.activated.connect(lambda _index: self.mode_changed.emit(self.mode.currentData()))
        # Measured on a POP 4: docs/protocol/RE_NOTES.md, 2026-10-14 and 2026-10-16.
        self.mode.setToolTip(
            "Leaving blue laser or speckle mode restarts the scanner (about 45 s).\n"
            "The blue laser is visible, but it gives too little depth to scan with yet."
        )
        self.accuracy = QComboBox()
        for key, name in ACCURACY.items():
            self.accuracy.addItem(name, key)
        self.tracking = QComboBox()
        self.object_type = QComboBox()
        settings = QFormLayout()
        settings.addRow("Scan mode", self.mode)
        settings.addRow("Accuracy", self.accuracy)
        settings.addRow("Tracking", self.tracking)
        settings.addRow("Object type", self.object_type)
        self.scale_label = QLabel()
        self.scale_label.setObjectName("scale")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 12)
        layout.setSpacing(10)
        header = QHBoxLayout()
        header.addWidget(self.connection)
        header.addWidget(self.device, 1)
        layout.addLayout(header)
        layout.addLayout(buttons)
        layout.addWidget(section_title("Scan settings"))
        layout.addLayout(settings)
        layout.addWidget(section_title("Scanning distance"))
        layout.addWidget(self.scale_label)
        layout.addWidget(section_title("Instructions"))
        layout.addWidget(hint(
            "Point the scanner at the object and keep the marker on the distance gauge in "
            "Excellent. Move slowly around the object; press Complete when it is covered."
        ))
        layout.addStretch()

        self.accuracy.currentIndexChanged.connect(self._refill_tracking)
        self.tracking.currentIndexChanged.connect(self._refill_object_types)
        self.object_type.currentIndexChanged.connect(self._choose_preset)
        self.device.currentIndexChanged.connect(self._on_device)
        self.state = ScanState.IDLE
        self._ready = True
        self._refill_tracking()
        self._set_state(ScanState.IDLE)

    @staticmethod
    def _scan_button(text: str, icon: str) -> QToolButton:
        button = QToolButton()
        button.setObjectName("scanAction")
        button.setText(text)
        button.setIcon(QIcon.fromTheme(icon))
        button.setIconSize(QSize(22, 22))
        button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
        button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        return button

    @property
    def pop4(self) -> bool:
        return self.device.currentIndex() == 1

    def set_connection(self, text: str, ok: bool = True) -> None:
        """What the heading beside the scanner list says: Connected,
        Connecting… or Disconnected."""
        self.connection.setText(text)
        self.connection_ok = ok
        self.connection.setStyleSheet("" if ok else "color: #e5484d;")

    def set_pop4_present(self, present: bool) -> None:
        """Lets the POP 4 be picked only while one is plugged in."""
        item = self.device.model().item(1)
        item.setEnabled(present)
        item.setToolTip("" if present else "Plug in a POP 4 to scan with it")

    def set_ready(self, ready: bool) -> None:
        """Whether a scan may start: not while a scanner connects or switches mode."""
        self._ready = ready
        self._set_state(self.state)

    def set_mode(self, mode: str) -> None:
        """Shows the scanner's scan mode, without asking for it again."""
        self.mode.setCurrentIndex(max(self.mode.findData(mode), 0))

    def set_tracking(self, tracking: str) -> None:
        index = self.tracking.findData(tracking)
        if index >= 0:
            self.tracking.setCurrentIndex(index)

    # Scan control: New scan, Pause / Resume, Complete, and Esc to cancel.

    def new_scan(self) -> None:
        if self.state is ScanState.IDLE and self._ready:
            self._set_state(ScanState.SCANNING)

    def toggle_pause(self) -> None:
        if self.state is ScanState.SCANNING:
            self._set_state(ScanState.PAUSED)
        elif self.state is ScanState.PAUSED:
            self._set_state(ScanState.SCANNING)

    def toggle(self) -> None:
        """What Space does: start a scan, or pause and resume it."""
        if self.state is ScanState.IDLE:
            self.new_scan()
        else:
            self.toggle_pause()

    def complete(self) -> None:
        if self.state is not ScanState.IDLE:
            self._set_state(ScanState.IDLE)
            self.completed.emit()

    def cancel(self) -> None:
        if self.state is not ScanState.IDLE:
            self._set_state(ScanState.IDLE)
            self.cancelled.emit()

    def _set_state(self, state: ScanState) -> None:
        self.state = state
        idle = state is ScanState.IDLE
        paused = state is ScanState.PAUSED
        self.new_scan_button.setEnabled(idle and self._ready)
        self.pause_button.setEnabled(not idle)
        self.pause_button.setText("Resume" if paused else "Pause")
        self.pause_button.setIcon(
            QIcon.fromTheme("media-playback-start" if paused else "media-playback-pause"))
        self.complete_button.setEnabled(not idle)
        self._update_settings_enabled()
        self.state_changed.emit(state)

    def _update_settings_enabled(self) -> None:
        # Settings are fixed for the length of a scan.
        idle = self.state is ScanState.IDLE
        for combo in (self.device, self.accuracy, self.tracking, self.object_type):
            combo.setEnabled(idle and self._ready)
        self.mode.setEnabled(idle and self._ready and self.pop4)

    # The presets only allow some combinations, so each choice narrows the next.

    def _refill(self, combo: QComboBox, keys, names: dict[str, str]) -> None:
        current = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        for key in keys:
            combo.addItem(names[key], key)
        combo.setCurrentIndex(max(combo.findData(current), 0))
        combo.blockSignals(False)

    def _refill_tracking(self) -> None:
        accuracy = self.accuracy.currentData()
        keys = dict.fromkeys(p.tracking for p in load_presets() if p.accuracy == accuracy)
        self._refill(self.tracking, keys, TRACKING)
        self._refill_object_types()

    def _refill_object_types(self) -> None:
        accuracy, tracking = self.accuracy.currentData(), self.tracking.currentData()
        keys = dict.fromkeys(p.object_type for p in load_presets()
                             if (p.accuracy, p.tracking) == (accuracy, tracking))
        self._refill(self.object_type, keys, OBJECT_TYPES)
        self._choose_preset()

    def _choose_preset(self) -> None:
        self.preset: Preset = find_preset(self.accuracy.currentData(),
                                          self.tracking.currentData(),
                                          self.object_type.currentData())
        lo, hi = self.preset.scale_mm
        self.scale_label.setText(f"{lo} – {hi} mm")
        self.preset_changed.emit(self.preset)

    def _on_device(self) -> None:
        self._update_settings_enabled()
        self.device_changed.emit(self.pop4)


class Viewport(QFrame):
    """The centre: the 3D view, with Revo Scan-style overlays on top of it."""

    VIEW_HINTS = "Rotate: drag\nMove: Shift + drag\nZoom: wheel\nFrame the cloud: double-click"
    SCAN_HINTS = "Start / pause scan: Space\nCancel: Esc\nFullscreen: Shift+M\n\n" + VIEW_HINTS
    MODEL_HINTS = "New scan: Space\nFullscreen: Shift+M\n\n" + VIEW_HINTS
    # More points than this are thinned for display, so that uploading a huge
    # frame never holds up the GUI thread.
    MAX_SHOWN_POINTS = 600_000

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("viewport")
        self.view3d = PointCloudView()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.view3d)

        self.hint = QLabel(self.SCAN_HINTS)
        self.hint.setObjectName("viewportHint")

        self.color_button = toggle("Color")
        self.no_color_button = toggle("No color", checked=True)
        colours = QButtonGroup(self)
        switch = QWidget()
        row = QHBoxLayout(switch)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        for button in (self.color_button, self.no_color_button):
            button.setObjectName("segment")
            colours.addButton(button)
            row.addWidget(button)
        self.color_button.toggled.connect(self.view3d.set_color_mode)

        self.gauge = DistanceGauge()
        self.rendering = {name: QRadioButton(name) for name in ("Raw", "Fused", "Meshed",
                                                                 "Textured")}
        rendering = QWidget()
        column = QVBoxLayout(rendering)
        column.addWidget(section_title("Rendering"))
        for name, button in self.rendering.items():
            column.addWidget(button)
            if name != "Raw":
                button.setEnabled(False)
                button.setToolTip("Comes with point-cloud processing")
        self.rendering["Raw"].setChecked(True)
        column.addStretch()
        self.side = QStackedWidget()
        self.side.addWidget(self.gauge)
        self.side.addWidget(rendering)

        self.message = QLabel("The live point cloud appears here")
        self.message.setObjectName("viewportMessage")
        self.toast = QLabel()
        self.toast.setObjectName("toast")
        self.toast.hide()
        # Stays up while the scanner is getting almost no depth, unlike the
        # toast, which is for things that have just happened.
        self.warning = QLabel()
        self.warning.setObjectName("warning")
        self.warning.setWordWrap(True)
        self.warning.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.warning.hide()
        self.frames = QLabel("Rendering frames: 0")
        self.frames.setObjectName("viewportHint")

        # The overlays are children of the 3D view, which draws them over the scene.
        grid = QGridLayout(self.view3d)
        grid.setContentsMargins(16, 12, 12, 12)
        top_left = Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
        top_right = Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight
        grid.addWidget(self.hint, 0, 0, top_left)
        grid.addWidget(switch, 0, 1, top_right)
        grid.addWidget(self.side, 0, 2, 3, 1, Qt.AlignmentFlag.AlignRight)
        grid.addWidget(self.message, 1, 0, 1, 2, Qt.AlignmentFlag.AlignCenter)
        grid.addWidget(self.warning, 1, 0, 1, 2,
                       Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter)
        grid.addWidget(self.toast, 2, 0, 1, 2,
                       Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter)
        grid.addWidget(self.frames, 2, 1,
                       Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight)
        grid.setRowStretch(1, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        self._toast_timer = QTimer(self)
        self._toast_timer.setSingleShot(True)
        self._toast_timer.timeout.connect(self.toast.hide)
        self._step = 1  # the display thinning of the shown cloud
        self.view3d.failed.connect(self._on_3d_failed)

    def set_stage(self, scanning: bool) -> None:
        """Scanning shows the distance gauge and draws the cloud green; a
        finished model shows the rendering choice and goes blue."""
        self.hint.setText(self.SCAN_HINTS if scanning else self.MODEL_HINTS)
        self.side.setCurrentIndex(0 if scanning else 1)
        self.view3d.set_base_color(SCAN_GREEN if scanning else SCAN_BLUE)

    def set_frame_count(self, count: int) -> None:
        self.frames.setText(f"Rendering frames: {count}")

    def show_surface(self, vertices, normals, indices, frame: bool = False) -> None:
        """Draws a surface in place of the cloud it was built from."""
        self.view3d.set_surface(vertices, normals, indices, frame=frame)
        if self.view3d.error is None:
            self.message.hide()

    def show_cloud(self, points, colors, normals, highlight=None, frame: bool = False) -> None:
        """Draws a cloud (sensor axes, millimetres) with a colour and a normal
        per point, and optionally an (N, 4) RGBA tint per point."""
        self._step = max(math.ceil(len(points) / self.MAX_SHOWN_POINTS), 1)
        shown = slice(None, None, self._step)
        self.view3d.set_surface(None, None, ())  # a cloud replaces any surface
        self.view3d.set_cloud(points[shown], colors[shown], normals[shown],
                              None if highlight is None else highlight[shown], frame=frame)
        if self.view3d.error is None:
            self.message.hide()

    def set_highlight(self, highlight) -> None:
        """Tints the shown cloud afresh; highlight has a row per point of the
        cloud given to show_cloud()."""
        self.view3d.set_highlight(highlight[::self._step])

    def clear_cloud(self, message: str) -> None:
        self.view3d.clear()
        if self.view3d.error is None:
            self.message.setText(message)
        self.message.show()

    def _on_3d_failed(self, reason: str) -> None:
        self.message.setText(f"3D view unavailable\n{reason}")
        self.message.setWordWrap(True)
        self.message.show()

    def show_warning(self, text: str) -> None:
        """Puts a warning up, or takes it down when text is empty."""
        self.warning.setText(text)
        self.warning.setVisible(bool(text))

    def show_toast(self, text: str, ms: int = 2500) -> None:
        self.toast.setText(text)
        self.toast.show()
        self._toast_timer.start(ms)
