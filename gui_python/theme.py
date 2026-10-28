"""The window's colours, and switching between them.

There is one set of styling rules, `assets/dark.qss`, written in the dark
theme's own colours. A theme is a recolouring of it: a table saying what each
of those colours becomes. The dark theme is the empty table, so the rules are
also the thing that is shipped, and a second theme cannot drift away from the
first the way a copied stylesheet would.

The colours are named below by what they are for. Code outside this file uses
the names and asks a theme for the colour, so nothing else holds a hex value:

    theme.of(theme.ACCENT)
"""

from dataclasses import dataclass, field
from pathlib import Path

QSS = Path(__file__).resolve().parent / "assets" / "dark.qss"

# Surfaces, from the furthest back to the nearest.
EDGE = "#141517"  # the hardest lines: splitter handles, the ribbon's underside
SUNKEN = "#1b1c1f"  # the status bar
WINDOW = "#1e1f22"  # the window behind everything
BASE = "#202124"  # lists and other things you look into
PANEL = "#26272b"  # side panels and the ribbon
ALT = "#26282b"  # menus, and a button with nothing to do
SEGMENT = "#2a2b2e"  # the Color / No color switch
PRESSED = "#2a2c2f"  # a button being pressed
RAISED = "#2f3134"  # a button at rest
RAISED_ALT = "#3a3c40"  # the Complete button, which is not the main action
RIBBON_HOVER = "#34363a"  # a ribbon button under the pointer
HOVER = "#383b3f"  # any other button under the pointer
HOVER_ALT = "#44474c"  # ...and the Complete button
PREVIEW = "#000000"  # behind a camera image, where black looks right

# Lines.
BORDER_DIM = "#2e3034"
BORDER_PRESSED = "#303236"
BORDER_FAINT = "#34363b"
BORDER_SOFT = "#3a3d42"
BORDER = "#3c3f44"
BORDER_SEGMENT = "#45484d"
BORDER_ALT = "#4a4d52"

# Text, brightest first.
WHITE = "#ffffff"
TEXT_STRONG = "#f2f3f5"
TEXT_SOFT = "#e6e8eb"
TEXT = "#dfe1e5"
TEXT_MID = "#d6d8dc"
TEXT_MUTED = "#c9ccd1"
GAUGE_TICK = "#b9bcc2"  # a distance band's name
MUTED = "#a9adb3"  # the status bar
MESSAGE = "#8f939a"  # "The live point cloud appears here"
DIM = "#8b8e94"  # hints and captions
GAUGE_LINE = "#7d8187"
DISABLED_WARM = "#7a7d82"
SLIDER_DISABLED = "#6f737a"
DISABLED_ALT = "#6a6d72"
DISABLED = "#5f6369"
PREVIEW_TEXT = "#5a5d63"  # "No signal"

# The accent, which is Revo Scan's red, and what it turns into.
ACCENT = "#e5484d"
ACCENT_HOVER = "#f05d61"
ACCENT_PRESSED = "#c93c41"
ACCENT_DIM = "#4a2a2c"  # a checked button with nothing to do
ACCENT_SUNKEN = "#3a2f30"  # ...and the New scan button
SLIDER_TRACK_DISABLED = "#4a3c3d"
LINK = "#ff7b7f"

# Notices.
WARNING = "#f0a35e"
WARNING_BORDER = "#e0a83e"
WARNING_EDGE = "#c07a2a"
OVERLAY = "rgba(20, 21, 23, 215)"  # behind a toast
OVERLAY_SOLID = "rgba(20, 21, 23, 225)"  # ...and a warning, which stays up

# The 3D viewport's background gradient, in the stylesheet and in OpenGL.
VIEWPORT_TOP = "#3b3d42"
VIEWPORT_BOTTOM = "#2b2c30"

# Qt's own palette, for the widgets the stylesheet does not reach.
PALETTE_BASE = "#2b2d30"
PALETTE_DISABLED_TEXT = "#6f737a"

# The distance gauge's bands say how far away the object is, so they keep
# their meaning in every theme and are not recoloured.
ZONE_TOO_NEAR = "#d95c5c"
ZONE_EXCELLENT = "#12b6b0"
ZONE_GOOD = "#5bbf5b"
ZONE_FAR = "#e0a83e"


@dataclass(frozen=True)
class Theme:
    """A name, and what each of the colours above becomes in it."""

    name: str
    # Empty for the dark theme, whose colours the names already are.
    recolour: dict[str, str] = field(default_factory=dict)
    # Whether Qt's own widgets should be drawn light or dark.
    light: bool = False

    def of(self, colour: str) -> str:
        """This theme's version of one of the colours named above."""
        return self.recolour.get(colour, colour)

    def rgb(self, colour: str) -> tuple[float, float, float]:
        """...as three 0-to-1 floats, for OpenGL."""
        themed = self.of(colour).lstrip("#")
        return tuple(int(themed[i : i + 2], 16) / 255 for i in (0, 2, 4))

    def stylesheet(self) -> str:
        """The styling rules, recoloured for this theme."""
        rules = QSS.read_text(encoding="utf-8")
        if not self.recolour:
            return rules
        # Every colour is swapped at once. One at a time, a colour turned
        # into another theme's colour could be swapped a second time.
        parts, rest = [], rules
        for original in sorted(self.recolour, key=len, reverse=True):
            pieces = rest.split(original)
            rest = f"\0{len(parts)}\0".join(pieces)
            parts.append(self.recolour[original])
        for index, themed in enumerate(parts):
            rest = rest.replace(f"\0{index}\0", themed)
        return rest


DARK = Theme("Dark")

LIGHT = Theme(
    "Light",
    light=True,
    recolour={
        # Surfaces turn over: what was darkest becomes the lines, and the
        # window behind everything becomes the lightest thing on screen.
        EDGE: "#b4bac3",
        SUNKEN: "#e7eaee",
        WINDOW: "#f4f5f7",
        BASE: "#ffffff",
        PANEL: "#eceef1",
        ALT: "#f1f3f6",
        SEGMENT: "#e8eaee",
        PRESSED: "#ccd1d8",
        RAISED: "#e4e7eb",
        RAISED_ALT: "#dde1e6",
        RIBBON_HOVER: "#dbdfe5",
        HOVER: "#d9dde3",
        HOVER_ALT: "#cfd4db",
        PREVIEW: "#1a1c20",  # a camera image still reads best on something dark
        # Lines.
        BORDER_DIM: "#d7dbe1",
        BORDER_PRESSED: "#c3c8d0",
        BORDER_FAINT: "#d2d7de",
        BORDER_SOFT: "#ccd1d8",
        BORDER: "#c3c8d0",
        BORDER_SEGMENT: "#b8bec7",
        BORDER_ALT: "#b8bec7",
        # Text, which darkens as the surfaces lighten.
        WHITE: "#ffffff",  # on the accent, which stays red
        TEXT_STRONG: "#15181d",
        TEXT_SOFT: "#22262c",
        TEXT: "#22262c",
        TEXT_MID: "#2d3238",
        TEXT_MUTED: "#3c424a",
        GAUGE_TICK: "#4a5059",
        MUTED: "#565c66",
        MESSAGE: "#767d87",
        DIM: "#6b7079",
        GAUGE_LINE: "#7b818a",
        DISABLED_WARM: "#9aa0a9",
        SLIDER_DISABLED: "#a8aeb7",
        DISABLED_ALT: "#9aa0a9",
        DISABLED: "#a3a8b0",
        PREVIEW_TEXT: "#8d939c",
        # The accent keeps its hue; only the shades behind it lighten.
        ACCENT_DIM: "#f2c9cb",
        ACCENT_SUNKEN: "#eed4d5",
        SLIDER_TRACK_DISABLED: "#e8cfd0",
        LINK: "#c0392f",
        # Notices: amber on a light backdrop needs darkening to stay legible.
        WARNING: "#8a5200",
        WARNING_BORDER: "#c08a2a",
        WARNING_EDGE: "#a8761f",
        OVERLAY: "rgba(252, 250, 246, 232)",
        OVERLAY_SOLID: "rgba(252, 250, 246, 242)",
        # The viewport stays a gradient, light enough to sit in a light
        # window and still leave the blue and green clouds readable.
        VIEWPORT_TOP: "#e3e7ec",
        VIEWPORT_BOTTOM: "#c9cfd8",
        PALETTE_BASE: "#ffffff",
        PALETTE_DISABLED_TEXT: "#a3a8b0",
    },
)

THEMES = {theme.name: theme for theme in (DARK, LIGHT)}
DEFAULT = DARK.name
