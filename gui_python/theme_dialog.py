"""Choosing the colours of a custom theme."""

from PyQt6.QtCore import QSize, Qt, pyqtSignal
from PyQt6.QtGui import QColor, QIcon, QPainter, QPixmap
from PyQt6.QtWidgets import (
    QColorDialog,
    QDialog,
    QDialogButtonBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)

import theme

# What each colour does, in the order they are offered.
PARTS = (
    ("surface", "Surface", "Everything sits on this. The panels, buttons and lines "
                           "take their shade from it, and a light one gives a light theme."),
    ("text", "Text", "The words. Hints and the things you cannot press follow it, "
                     "a little way towards the surface."),
    ("accent", "Accent", "The scanning buttons and anything chosen. Revo Scan's is red."),
)


def swatch(colour: str, size: int = 22) -> QIcon:
    """A rounded block of colour, for the button that picks it."""
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.GlobalColor.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.setBrush(QColor(colour))
    painter.setPen(QColor(colour).darker(140))
    painter.drawRoundedRect(0, 0, size - 1, size - 1, 4, 4)
    painter.end()
    return QIcon(pixmap)


class CustomThemeDialog(QDialog):
    """Picks the three colours a custom theme is built from, showing the
    result behind the dialog as it changes."""

    changed = pyqtSignal(dict)  # the colours, whenever one of them moves

    def __init__(self, colours: dict[str, str], parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Custom theme")
        self.colours = dict(colours)
        self._buttons: dict[str, QPushButton] = {}

        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        for part, title, what in PARTS:
            heading = QLabel(f"<b>{title}</b>")
            explain = QLabel(what)
            explain.setWordWrap(True)
            explain.setObjectName("hint")
            button = QPushButton(self.colours[part])
            button.setIconSize(QSize(22, 22))
            button.clicked.connect(lambda _checked, part=part: self._pick(part))
            self._buttons[part] = button
            row = QHBoxLayout()
            row.addWidget(heading, 1)
            row.addWidget(button)
            layout.addLayout(row)
            layout.addWidget(explain)

        self.buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel
            | QDialogButtonBox.StandardButton.Reset
        )
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        self.buttons.button(QDialogButtonBox.StandardButton.Reset).clicked.connect(self.reset)
        layout.addStretch()
        layout.addWidget(self.buttons)
        self._show_colours()

    def set_colour(self, part: str, colour: str) -> None:
        """Takes one of the three colours and shows the theme it makes."""
        self.colours[part] = QColor(colour).name()
        self._show_colours()
        self.changed.emit(dict(self.colours))

    def reset(self) -> None:
        """Back to the colours the dark theme is written in."""
        self.colours = dict(theme.CUSTOM_DEFAULTS)
        self._show_colours()
        self.changed.emit(dict(self.colours))

    def _pick(self, part: str) -> None:
        chosen = QColorDialog.getColor(QColor(self.colours[part]), self,
                                       f"{dict((p, t) for p, t, _ in PARTS)[part]} colour")
        if chosen.isValid():
            self.set_colour(part, chosen.name())

    def _show_colours(self) -> None:
        for part, button in self._buttons.items():
            button.setText(self.colours[part])
            button.setIcon(swatch(self.colours[part]))
