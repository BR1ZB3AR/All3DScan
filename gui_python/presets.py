"""The vendor's own POP 4 scan presets, read from resources/param_pop4.json.

Each preset is one combination of accuracy, tracking mode and object type,
and carries the working-distance bands the scanning-distance gauge shows.
Only the "surface" presets are read: the default infrared ones, as the
legacy protocol code (src/protocol/RevopointPop4Protocol.cpp) treats them.
"""

import json
from dataclasses import dataclass
from functools import cache
from pathlib import Path

PRESET_FILE = Path(__file__).resolve().parents[1] / "resources" / "param_pop4.json"

# Keys in the file, and the names the window shows for them.
ACCURACY = {"accuracymode": "High accuracy", "highSpeedmode": "High speed"}
TRACKING = {
    "feature": "Feature",
    "marker": "Marker",
    "globalMarker": "Global marker",
    "colortexture": "Texture",
}
OBJECT_TYPES = {
    "general": "General",
    "dark": "Dark",
    "face": "Face",
    "body": "Body",
    "large": "Large object",
}


@dataclass(frozen=True)
class Preset:
    accuracy: str
    tracking: str
    object_type: str
    # histogram1..4: where Too near, Excellent, Good, Far and Too far meet.
    band_edges_mm: tuple[int, ...]
    # zminRange and zmaxRange: the span the gauge shows.
    scale_mm: tuple[int, int]
    exposure_us: int


@cache
def load_presets() -> tuple[Preset, ...]:
    model = json.loads(PRESET_FILE.read_text(encoding="utf-8"))[0]
    return tuple(
        Preset(
            accuracy=accuracy,
            tracking=tracking,
            object_type=entry["objectType"],
            band_edges_mm=tuple(entry[f"histogram{i}"] for i in range(1, 5)),
            scale_mm=(entry["zminRange"], entry["zmaxRange"]),
            exposure_us=entry["exposure"],
        )
        for accuracy in ACCURACY
        for tracking, entries in model[accuracy]["surface"].items()
        if tracking in TRACKING
        for entry in entries
    )


def find_preset(accuracy: str, tracking: str, object_type: str) -> Preset | None:
    wanted = (accuracy, tracking, object_type)
    return next(
        (p for p in load_presets() if (p.accuracy, p.tracking, p.object_type) == wanted), None
    )
