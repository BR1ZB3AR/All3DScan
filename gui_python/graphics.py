"""What the 3D view is set to draw with, and where that is remembered.

Separate from the dialog that edits it so that the viewport and the startup
code can read the settings without pulling in any widgets — the window
system choice in particular has to be known before a QApplication exists.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, replace

from PyQt6.QtCore import QSettings

#: How the window talks to the display. "automatic" lets Qt decide, which
#: is what it did before this was a choice. It matters here beyond the usual
#: reasons: Wayland gives this app an OpenGL ES context through EGL and X11
#: gives desktop OpenGL through GLX, and the shaders differ between them.
WINDOW_SYSTEMS = ("automatic", "wayland", "x11")

#: Samples per pixel the 3D view may ask for. 0 is off.
ANTIALIASING = (0, 2, 4, 8)


@dataclass(frozen=True)
class Graphics:
    """Everything the 3D view is drawn with."""

    #: Samples per pixel, for smoothing edges. Costs fill rate.
    antialiasing: int = 4
    #: Shading a cloud from its own depth, so its shape reads without a mesh.
    eye_dome: bool = True
    #: How hard that shading bites, 0 to 100.
    eye_dome_strength: float = 55.0
    #: How wide a point is drawn, in millimetres of the scan.
    splat_mm: float = 2.0
    #: The most points drawn at once; beyond this a cloud is thinned for
    #: the view only, never for what is measured or saved.
    point_budget: int = 2_000_000
    #: Which window system to start on; see WINDOW_SYSTEMS.
    window_system: str = "automatic"

    def tidy(self) -> "Graphics":
        """The same settings with anything out of range brought back in, so
        a hand-edited config file cannot stop the app drawing."""
        return replace(
            self,
            antialiasing=min(ANTIALIASING, key=lambda n: abs(n - self.antialiasing)),
            eye_dome_strength=float(min(max(self.eye_dome_strength, 0.0), 100.0)),
            splat_mm=float(min(max(self.splat_mm, 0.2), 12.0)),
            point_budget=int(min(max(self.point_budget, 50_000), 20_000_000)),
            window_system=(self.window_system if self.window_system in WINDOW_SYSTEMS
                           else "automatic"),
        )


#: The two ends of the trade, as named choices. Custom is whatever the
#: person has set themselves, so it is not listed here.
PRESETS: dict[str, Graphics] = {
    "Performance": Graphics(antialiasing=0, eye_dome=False, splat_mm=2.0,
                            point_budget=750_000),
    "Quality": Graphics(antialiasing=8, eye_dome=True, eye_dome_strength=55.0,
                        splat_mm=2.0, point_budget=6_000_000),
}


def preset_named(name: str, keeping: Graphics) -> Graphics:
    """A preset, with the settings it does not speak for left as they were.
    The window system is one of those: it is not a matter of quality."""
    chosen = PRESETS[name]
    return replace(chosen, window_system=keeping.window_system).tidy()


def matching_preset(graphics: Graphics) -> str | None:
    """Which preset these settings are, or None when they are neither."""
    for name in PRESETS:
        if preset_named(name, graphics) == graphics.tidy():
            return name
    return None


def _store() -> QSettings:
    return QSettings("All3DScan", "All3DScan")


def current() -> Graphics:
    """What the 3D view is set to, as last saved."""
    store = _store()
    store.beginGroup("graphics")
    fallback = Graphics()
    try:
        found = Graphics(
            antialiasing=int(store.value("antialiasing", fallback.antialiasing)),
            eye_dome=store.value("eye_dome", fallback.eye_dome, type=bool),
            eye_dome_strength=float(
                store.value("eye_dome_strength", fallback.eye_dome_strength)),
            splat_mm=float(store.value("splat_mm", fallback.splat_mm)),
            point_budget=int(store.value("point_budget", fallback.point_budget)),
            window_system=str(store.value("window_system", fallback.window_system)),
        )
    except (TypeError, ValueError):
        found = fallback  # a config written by something else, or by hand
    finally:
        store.endGroup()
    return found.tidy()


def remember(graphics: Graphics) -> None:
    """Saves these settings for next time."""
    store = _store()
    store.beginGroup("graphics")
    for name, value in vars(graphics.tidy()).items():
        store.setValue(name, value)
    store.endGroup()


def platform_override(graphics: Graphics) -> str | None:
    """The Qt platform name this choice asks for, or None for automatic.

    An explicit -platform on the command line still wins: someone starting
    the app a particular way meant it.
    """
    return {"wayland": "wayland", "x11": "xcb"}.get(graphics.window_system)


def current_platform_name() -> str:
    """What the window is running on now, in the dialog's own words."""
    try:
        from PyQt6.QtGui import QGuiApplication

        running = QGuiApplication.platformName()
    except Exception:  # noqa: BLE001 - no application yet
        return "nothing yet"
    return {"wayland": "Wayland", "xcb": "X11"}.get(running, running or "nothing yet")


def describe_gpu() -> str:
    """What is doing the drawing, read from the live context.

    Needs a current OpenGL context; without one there is nothing to ask, and
    saying so is better than guessing from the hardware.
    """
    try:
        from OpenGL import GL

        renderer = GL.glGetString(GL.GL_RENDERER)
        version = GL.glGetString(GL.GL_VERSION)
        if not renderer:
            return "not known yet"
        name = renderer.decode(errors="replace")
        api = version.decode(errors="replace") if version else "OpenGL"
        return f"{name} ({api})"
    except Exception:  # noqa: BLE001 - no context, or no GL at all
        return "not known yet"


def env_override() -> str | None:
    """The environment variable that overrules the graphics card choice, if
    it is set — the same escape hatch the drivers themselves offer."""
    for name in ("__GLX_VENDOR_LIBRARY_NAME", "DRI_PRIME", "MESA_LOADER_DRIVER_OVERRIDE"):
        if os.environ.get(name):
            return f"{name}={os.environ[name]}"
    return None
