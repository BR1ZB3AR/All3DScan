"""What the All3DScan windows share: identity, icon, the dark theme and the
OpenGL setup."""

import os
import sys
from pathlib import Path

from PyQt6.QtGui import QColor, QGuiApplication, QPalette, QSurfaceFormat
from PyQt6.QtWidgets import QApplication

HERE = Path(__file__).resolve().parent
STYLESHEET = HERE / "assets" / "dark.qss"
APP_ICON = HERE.parent / "resources/icons/hicolor/256x256/apps/io.github.all3dscan.All3DScan.png"
# The launcher entry's file name (install_desktop_entry.py) and, on Wayland,
# the window's app id, which is how the panel matches the two.
APP_ID = "io.github.all3dscan.All3DScan.Gui"


def chosen_platform(argv: list[str] | None = None) -> str:
    """The Qt platform plugin a QApplication made with argv (sys.argv by
    default) is going to use, known before it exists. Qt takes -platform from
    the command line, then QT_QPA_PLATFORM; failing both, Wayland in a session
    whose compositor socket is there, and X11 otherwise."""
    args = sys.argv if argv is None else argv
    for index, arg in enumerate(args):
        if arg in ("-platform", "--platform") and index + 1 < len(args):
            return _first_platform(args[index + 1])
        if arg.startswith(("-platform=", "--platform=")):
            return _first_platform(arg.split("=", 1)[1])
    if os.environ.get("QT_QPA_PLATFORM"):
        return _first_platform(os.environ["QT_QPA_PLATFORM"])
    display = os.environ.get("WAYLAND_DISPLAY")
    if display and os.environ.get("XDG_SESSION_TYPE") != "x11":
        runtime = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
        if (runtime / display).exists():  # an absolute display stays absolute
            return "wayland"
    return "xcb"


def _first_platform(spec: str) -> str:
    # "wayland;xcb" tries Wayland first; "xcb:option=value" gives options.
    return spec.split(";")[0].split(":")[0].strip()


def uses_wayland(argv: list[str] | None = None) -> bool:
    return chosen_platform(argv).startswith("wayland")


# The platform configure_opengl() set the 3D view up for.
_configured_for: str | None = None


def configure_opengl(argv: list[str] | None = None) -> None:
    """Asks for the OpenGL context the 3D view is written for. Call it before
    the QApplication is created, with the arguments it will be given: Qt
    reads the default format then.

    On Wayland the Qt that PyQt6 ships only makes OpenGL ES contexts (desktop
    OpenGL requests fail with EGL_BAD_MATCH, measured on this PC's NVIDIA
    driver); X11 and the offscreen platform give desktop OpenGL."""
    global _configured_for
    _configured_for = chosen_platform(argv)
    surface = QSurfaceFormat()
    if _configured_for.startswith("wayland"):
        surface.setRenderableType(QSurfaceFormat.RenderableType.OpenGLES)
        surface.setVersion(3, 0)
    else:
        surface.setVersion(3, 3)
        surface.setProfile(QSurfaceFormat.OpenGLContextProfile.CoreProfile)
    surface.setDepthBufferSize(24)
    QSurfaceFormat.setDefaultFormat(surface)


def warn_if_platform_differs() -> None:
    """Says so on stderr if Qt started on another kind of platform than
    configure_opengl() expected, since the 3D view's context may then fail."""
    running = QGuiApplication.platformName()
    if _configured_for is not None and (
            _configured_for.startswith("wayland") != running.startswith("wayland")):
        print(f"All3DScan: OpenGL was set up for {_configured_for}, but Qt is running on "
              f"{running}; the 3D view may not work. Set QT_QPA_PLATFORM to choose.",
              file=sys.stderr, flush=True)


def apply_dark_theme(app: QApplication) -> None:
    """Fusion with a dark palette; assets/dark.qss adds the accents."""
    app.setStyle("Fusion")
    palette = QPalette()
    for role, color in {
        QPalette.ColorRole.Window: "#1e1f22",
        QPalette.ColorRole.WindowText: "#dfe1e5",
        QPalette.ColorRole.Base: "#2b2d30",
        QPalette.ColorRole.AlternateBase: "#26282b",
        QPalette.ColorRole.Text: "#dfe1e5",
        QPalette.ColorRole.Button: "#2b2d30",
        QPalette.ColorRole.ButtonText: "#dfe1e5",
        QPalette.ColorRole.Highlight: "#e5484d",
        QPalette.ColorRole.HighlightedText: "#ffffff",
        QPalette.ColorRole.ToolTipBase: "#2b2d30",
        QPalette.ColorRole.ToolTipText: "#dfe1e5",
        QPalette.ColorRole.PlaceholderText: "#8b8e94",
        QPalette.ColorRole.Link: "#ff7b7f",
    }.items():
        palette.setColor(role, QColor(color))
    for role in (
        QPalette.ColorRole.WindowText,
        QPalette.ColorRole.Text,
        QPalette.ColorRole.ButtonText,
    ):
        palette.setColor(QPalette.ColorGroup.Disabled, role, QColor("#6f737a"))
    app.setPalette(palette)
    if STYLESHEET.exists():
        app.setStyleSheet(STYLESHEET.read_text(encoding="utf-8"))
