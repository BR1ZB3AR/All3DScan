"""The Settings window: what the 3D view draws with, and how it starts."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

import graphics
from graphics import ANTIALIASING, Graphics, WINDOW_SYSTEMS

#: What each window system is called in the dialog.
WINDOW_SYSTEM_NAMES = {"automatic": "Automatic", "wayland": "Wayland", "x11": "X11"}

#: What each anti-aliasing setting is called.
ANTIALIASING_NAMES = {0: "Off", 2: "2 samples", 4: "4 samples", 8: "8 samples"}


def heading(text: str) -> QLabel:
    """A small capitalised heading, as the sections are divided by."""
    label = QLabel(text.upper())
    label.setObjectName("settingsSection")
    return label


def hint(text: str) -> QLabel:
    label = QLabel(text)
    label.setObjectName("hint")
    label.setWordWrap(True)
    return label


class Segmented(QWidget):
    """A row of buttons of which one is chosen, as the presets are picked."""

    chosen = pyqtSignal(str)

    def __init__(self, options: list[tuple[str, str]]) -> None:
        super().__init__()
        self.group = QButtonGroup(self)
        self.group.setExclusive(True)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(0)
        self._buttons: dict[str, QPushButton] = {}
        for key, name in options:
            button = QPushButton(name)
            button.setCheckable(True)
            button.setObjectName("segment")
            button.setCursor(Qt.CursorShape.PointingHandCursor)
            button.clicked.connect(lambda _checked, key=key: self.chosen.emit(key))
            self.group.addButton(button)
            row.addWidget(button)
            self._buttons[key] = button
        row.addStretch()

    def show_chosen(self, key: str | None) -> None:
        """Marks one as chosen without saying so, so that filling the dialog
        in does not read as the person having pressed it."""
        for name, button in self._buttons.items():
            was = button.blockSignals(True)
            button.setChecked(name == key)
            button.blockSignals(was)

    def choose(self, key: str) -> None:
        """Picks one as a press would: marks it and says so."""
        if key in self._buttons:
            self._buttons[key].click()

    def chosen_key(self) -> str | None:
        for name, button in self._buttons.items():
            if button.isChecked():
                return name
        return None


class SettingsDialog(QDialog):
    """Graphics settings, with Cancel, Apply and OK.

    Apply hands back what is set without closing, so a choice that takes
    effect at once can be seen against the scan on screen behind.
    """

    applied = pyqtSignal(object)  # Graphics

    def __init__(self, settings: Graphics, parent=None, gpu: str = "") -> None:
        super().__init__(parent)
        self.setWindowTitle("Settings")
        self.setMinimumWidth(560)
        self._was = settings.tidy()
        self._settings = self._was
        self._filling = False

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        title = QLabel("<b>Graphics</b>")
        layout.addWidget(title)
        layout.addWidget(hint("Trade speed for smoothness. Nothing here changes what is "
                              "measured, fitted or saved — only what is drawn."))

        layout.addWidget(heading("Graphics card"))
        self.card = QComboBox()
        self.card.addItem(gpu or graphics.describe_gpu())
        self.card.setEnabled(False)  # one context, so nothing to choose between
        layout.addWidget(self.card)
        override = graphics.env_override()
        layout.addWidget(hint(
            f"The card the driver gave this app. {override} is set and overrules it."
            if override else
            "The card the driver gave this app. On a machine with two, the driver's own "
            "variables choose — DRI_PRIME, or __GLX_VENDOR_LIBRARY_NAME."))

        self.presets = Segmented([(name, name) for name in graphics.PRESETS] +
                                 [("Custom", "Custom")])
        self.presets.chosen.connect(self._pick_preset)
        layout.addWidget(self.presets)
        layout.addWidget(hint("Presets set the choices below together."))

        form = QFormLayout()
        form.setHorizontalSpacing(14)
        # Enough room that a label never clips into the control beside it.
        form.setVerticalSpacing(12)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)
        self.antialiasing = QComboBox()
        for samples in ANTIALIASING:
            self.antialiasing.addItem(ANTIALIASING_NAMES[samples], samples)
        self.antialiasing.currentIndexChanged.connect(self._changed)
        form.addRow("Smooth edges", self.antialiasing)

        self.eye_dome = QCheckBox("Shade the cloud by its own depth")
        self.eye_dome.setToolTip(
            "Darkens a point where it stands in front of the points around it, so a "
            "raw cloud shows its shape without being meshed first.")
        self.eye_dome.toggled.connect(self._changed)
        form.addRow("Depth shading", self.eye_dome)

        self.strength = QSlider(Qt.Orientation.Horizontal)
        self.strength.setMinimumHeight(22)
        self.strength.setRange(0, 100)
        self.strength.valueChanged.connect(self._changed)
        self.strength_label = QLabel()
        strength_row = QHBoxLayout()
        strength_row.addWidget(self.strength, 1)
        strength_row.addWidget(self.strength_label)
        form.addRow("How strongly", strength_row)

        self.splat = QSlider(Qt.Orientation.Horizontal)
        self.splat.setMinimumHeight(22)
        self.splat.setRange(2, 120)  # tenths of a millimetre
        self.splat.valueChanged.connect(self._changed)
        self.splat_label = QLabel()
        splat_row = QHBoxLayout()
        splat_row.addWidget(self.splat, 1)
        splat_row.addWidget(self.splat_label)
        form.addRow("Point size", splat_row)

        self.budget = QComboBox()
        for points in (750_000, 2_000_000, 6_000_000, 20_000_000):
            self.budget.addItem(f"{points:,} points", points)
        self.budget.currentIndexChanged.connect(self._changed)
        form.addRow("Drawn at once", self.budget)
        layout.addLayout(form)
        layout.addWidget(hint("A cloud larger than this is thinned for the view only. "
                              "What is fitted, meshed and saved is always the whole of it."))

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setObjectName("rule")
        layout.addWidget(line)

        layout.addWidget(heading("Window system"))
        self.window_system = Segmented(
            [(key, WINDOW_SYSTEM_NAMES[key]) for key in WINDOW_SYSTEMS])
        self.window_system.chosen.connect(self._pick_window_system)
        layout.addWidget(self.window_system)
        self.restart_note = hint("")
        layout.addWidget(self.restart_note)

        self.buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel
            | QDialogButtonBox.StandardButton.Apply)
        self.buttons.accepted.connect(self._ok)
        self.buttons.rejected.connect(self.reject)
        self.apply_button = self.buttons.button(QDialogButtonBox.StandardButton.Apply)
        self.apply_button.clicked.connect(self._apply)
        layout.addStretch()
        layout.addWidget(self.buttons)

        self._fill(self._settings)

    # What is on screen, and what it means.

    def settings(self) -> Graphics:
        return self._settings

    def _fill(self, settings: Graphics) -> None:
        self._filling = True
        self.antialiasing.setCurrentIndex(max(self.antialiasing.findData(
            settings.antialiasing), 0))
        self.eye_dome.setChecked(settings.eye_dome)
        self.strength.setValue(int(round(settings.eye_dome_strength)))
        self.splat.setValue(int(round(settings.splat_mm * 10)))
        self.budget.setCurrentIndex(max(self.budget.findData(settings.point_budget), 0))
        self.window_system.show_chosen(settings.window_system)
        self._filling = False
        self._settings = settings
        self._show_derived()

    def _from_widgets(self) -> Graphics:
        return Graphics(
            antialiasing=self.antialiasing.currentData(),
            eye_dome=self.eye_dome.isChecked(),
            eye_dome_strength=float(self.strength.value()),
            splat_mm=self.splat.value() / 10.0,
            point_budget=self.budget.currentData(),
            window_system=self.window_system.chosen_key() or "automatic",
        ).tidy()

    def _changed(self) -> None:
        if self._filling:
            return
        self._settings = self._from_widgets()
        self._show_derived()

    def _pick_preset(self, name: str) -> None:
        if name == "Custom":
            self._show_derived()  # already whatever it is
            return
        self._fill(graphics.preset_named(name, self._settings))

    def _pick_window_system(self, _key: str) -> None:
        self._changed()

    def _show_derived(self) -> None:
        """Keeps the parts that follow from the choices in step: which preset
        is lit, the slider readouts, and what needs a restart."""
        self.strength_label.setText(f"{self.strength.value()}")
        self.strength.setEnabled(self.eye_dome.isChecked())
        self.strength_label.setEnabled(self.eye_dome.isChecked())
        self.splat_label.setText(f"{self.splat.value() / 10:.1f} mm")
        self.presets.show_chosen(graphics.matching_preset(self._settings) or "Custom")

        waiting = []
        if self._settings.antialiasing != self._was.antialiasing:
            waiting.append("smoothing")
        if self._settings.window_system != self._was.window_system:
            waiting.append("the window system")
        now = graphics.current_platform_name()
        if waiting:
            self.restart_note.setText(
                f"Applies after a restart: {' and '.join(waiting)}. "
                f"Running on {now} now.")
        else:
            self.restart_note.setText(f"Running on {now} now.")
        self.apply_button.setEnabled(self._settings != self._was)

    def _apply(self) -> None:
        self._settings = self._from_widgets()
        self.applied.emit(self._settings)
        self._was = self._settings
        self._show_derived()

    def _ok(self) -> None:
        self._settings = self._from_widgets()
        self.applied.emit(self._settings)
        self.accept()
