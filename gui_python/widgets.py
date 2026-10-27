"""Widgets the main window is built from."""

import numpy as np
from PyQt6.QtCore import QPointF, QRectF, Qt
from PyQt6.QtGui import QAction, QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

ZONES = ("too-near", "excellent", "good", "far", "too-far")
ZONE_LABELS = {
    "too-near": "Too near",
    "excellent": "Excellent",
    "good": "Good",
    "far": "Far",
    "too-far": "Too far",
}
# The band colours of the C++ app's distance gauge (src/app/Theme.h).
ZONE_COLORS = {
    "too-near": "#d95c5c",
    "excellent": "#12b6b0",
    "good": "#5bbf5b",
    "far": "#e0a83e",
    "too-far": "#d95c5c",
}


def zone_for(mm: float | None, band_edges_mm) -> str:
    """The band a distance falls in, or "idle" when there is no distance."""
    if mm is None:
        return "idle"
    for edge, zone in zip(band_edges_mm, ZONES):
        if mm < edge:
            return zone
    return "too-far"


def center_distance_mm(depth_mm: np.ndarray) -> float | None:
    """Median depth over the middle of the image: the distance to the object."""
    h, w = depth_mm.shape
    centre = depth_mm[2 * h // 5 : 3 * h // 5 + 1, 2 * w // 5 : 3 * w // 5 + 1]
    valid = centre[centre > 0]
    return float(np.median(valid)) if valid.size else None


class DistanceGauge(QWidget):
    """The vertical scanning-distance gauge, near at the top: the bands, a
    histogram of the frame's depths and a marker at the object's distance."""

    BINS = 50

    def __init__(self) -> None:
        super().__init__()
        self.setFixedWidth(170)
        self.setMinimumHeight(260)
        self.band_edges_mm: tuple[int, ...] = (290, 350, 400, 460)
        self.scale_mm: tuple[int, int] = (250, 500)
        self.distance_mm: float | None = None
        self._histogram = np.zeros(self.BINS)

    @property
    def zone(self) -> str:
        return zone_for(self.distance_mm, self.band_edges_mm)

    def set_bands(self, band_edges_mm, scale_mm) -> None:
        self.band_edges_mm, self.scale_mm = tuple(band_edges_mm), tuple(scale_mm)
        self.update()

    def set_depths(self, depth_mm: np.ndarray | None) -> None:
        """Takes a frame's depth image (0 where there is none), or None to clear."""
        if depth_mm is None:
            self.distance_mm = None
            self._histogram = np.zeros(self.BINS)
        else:
            # Every other pixel each way is plenty for a histogram, and keeps
            # this cheap on the GUI thread however large the frame.
            sample = depth_mm[::2, ::2]
            counts, _ = np.histogram(sample[sample > 0], bins=self.BINS, range=self.scale_mm)
            self._histogram = counts / counts.max() if counts.max() else counts.astype(float)
            self.distance_mm = center_distance_mm(depth_mm)
        self.update()

    def paintEvent(self, event) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        small = QFont(self.font())
        small.setPointSizeF(8.5)
        p.setFont(small)
        lo, hi = self.scale_mm
        top, bottom = 26.0, self.height() - 10.0

        def y(mm: float) -> float:
            return top + (min(max(mm, lo), hi) - lo) / (hi - lo) * (bottom - top)

        strip_x, strip_w = 74.0, 8.0
        p.setPen(QColor("#a9adb3"))
        p.drawText(QRectF(0, 0, self.width() - 4, 18),
                   Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, "Unit: mm")

        edges = (lo, *self.band_edges_mm, hi)
        current = self.zone
        for zone, a, b in zip(ZONES, edges, edges[1:]):
            colour = QColor(ZONE_COLORS[zone])
            colour.setAlpha(255 if zone == current else 140)
            p.fillRect(QRectF(strip_x, y(a), strip_w, y(b) - y(a)), colour)
            label = QFont(small)
            label.setBold(zone == current)
            p.setFont(label)
            p.setPen(QColor("#ffffff") if zone == current else QColor("#b9bcc2"))
            p.drawText(QRectF(0, y(a), strip_x - 8, y(b) - y(a)),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       ZONE_LABELS[zone])
        p.setFont(small)
        p.setPen(QColor("#7d8187"))
        for mm in edges:
            p.drawText(QRectF(0, y(mm) - 13, strip_x - 8, 12),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignBottom, str(mm))

        bar_x = strip_x + strip_w + 4
        bar_w = self.width() - bar_x - 4
        bin_h = (bottom - top) / self.BINS
        for i, share in enumerate(self._histogram):
            if share > 0:
                colour = QColor(ZONE_COLORS[zone_for(lo + (i + 0.5) * (hi - lo) / self.BINS,
                                                     self.band_edges_mm)])
                colour.setAlpha(190)
                p.fillRect(QRectF(bar_x, top + i * bin_h, share * bar_w, max(1.0, bin_h - 1)),
                           colour)

        if self.distance_mm is not None:
            at = y(self.distance_mm)
            p.setPen(QPen(QColor("#ffffff"), 2))
            p.drawLine(QPointF(strip_x - 3, at), QPointF(strip_x + strip_w + 3, at))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor("#ffffff"))
            p.drawPolygon(QPolygonF([QPointF(bar_x, at), QPointF(bar_x + 8, at - 5),
                                     QPointF(bar_x + 8, at + 5)]))
            p.setPen(QColor("#ffffff"))
            p.drawText(QRectF(bar_x + 11, at - 8, bar_w - 11, 16),
                       Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                       f"{self.distance_mm:.0f}")
        p.end()


class VideoPreview(QFrame):
    """A camera feed: a caption over the image, which scales to fit."""

    def __init__(self, title: str) -> None:
        super().__init__()
        self.setObjectName("preview")
        caption = QLabel(title)
        caption.setObjectName("previewTitle")
        self.image = QLabel("No signal")
        self.image.setObjectName("previewImage")
        self.image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        # Ignored, so that a large frame never makes the panel grow.
        self.image.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.image.setMinimumHeight(120)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        layout.addWidget(caption)
        layout.addWidget(self.image, 1)

    def show_image(self, pixels: np.ndarray) -> None:
        """Shows an (h, w) grey or (h, w, 3) RGB uint8 image, scaled to fit.
        An empty one, from a camera that has sent nothing yet, changes nothing."""
        if pixels.size == 0:
            return
        # Thin a frame far larger than the preview before Qt smooths it, so
        # the GUI thread's work stays small whatever the sensor resolution.
        step = max(1, min(pixels.shape[1] // max(self.image.width(), 1),
                          pixels.shape[0] // max(self.image.height(), 1)) // 2)
        pixels = np.ascontiguousarray(pixels[::step, ::step])
        h, w = pixels.shape[:2]
        if pixels.ndim == 3:
            image = QImage(pixels.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        else:
            image = QImage(pixels.data, w, h, w, QImage.Format.Format_Grayscale8)
        # The QImage only borrows the array; the pixmap is a copy.
        self.image.setPixmap(QPixmap.fromImage(image).scaled(
            self.image.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))

    def clear(self) -> None:
        self.image.clear()
        self.image.setText("No signal")


class RibbonGroup(QWidget):
    """A captioned group of buttons on the ribbon."""

    def __init__(self, caption: str, actions: list[QAction]) -> None:
        super().__init__()
        self.setObjectName("ribbonGroup")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground)
        row = QHBoxLayout()
        row.setContentsMargins(6, 2, 6, 0)
        row.setSpacing(2)
        self.buttons = []
        for action in actions:
            button = QToolButton()
            button.setDefaultAction(action)
            button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextUnderIcon)
            button.setAutoRaise(True)
            row.addWidget(button)
            self.buttons.append(button)
        label = QLabel(caption)
        label.setObjectName("ribbonCaption")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 2)
        layout.setSpacing(0)
        layout.addLayout(row)
        layout.addWidget(label)
