"""Adds the All3DScan window to this computer's application launcher.

Writes a desktop entry for the current user (no root needed) that starts the
main window, gui_python/app.py, with the repository's .venv:

    .venv/bin/python gui_python/install_desktop_entry.py           # add or update
    .venv/bin/python gui_python/install_desktop_entry.py --remove  # take it out
"""

import argparse
import os
import shutil
import subprocess
from pathlib import Path

from common import APP_ICON, APP_ID, HERE

REPO = HERE.parent
PYTHON = REPO / ".venv" / "bin" / "python"
# Characters that force an Exec argument into quotes (desktop entry spec).
RESERVED = set(" \t\n\"'\\><~|&;$*?#()`")


def applications_dir() -> Path:
    data_home = os.environ.get("XDG_DATA_HOME") or Path.home() / ".local" / "share"
    return Path(data_home) / "applications"


def exec_arg(arg: str) -> str:
    """One Exec argument, quoted and escaped as the desktop entry spec asks."""
    arg = arg.replace("%", "%%")
    if not RESERVED.intersection(arg):
        return arg
    quoted = "".join("\\" + c if c in '"`$\\' else c for c in arg)
    # String values unescape backslashes once more before the quoting rule.
    return '"' + quoted.replace("\\", "\\\\") + '"'


def desktop_entry() -> str:
    return "\n".join([
        "[Desktop Entry]",
        "Type=Application",
        "Name=All3DScan",
        "GenericName=3D Scanner",
        "Comment=Capture point clouds through the C++ sensor layer and Rust engine",
        f"Exec={exec_arg(str(PYTHON))} {exec_arg(str(HERE / 'app.py'))}",
        f"Path={REPO}",
        f"Icon={APP_ICON}",
        "Terminal=false",
        "Categories=Graphics;3DGraphics;Engineering;",
        "Keywords=3D;scanner;point cloud;",
        "StartupNotify=true",
    ]) + "\n"


def refresh_menus(directory: Path) -> None:
    # KDE reads its own cache (kbuildsycoca6); other desktops read the database.
    for command in (["update-desktop-database", str(directory)], ["kbuildsycoca6"]):
        if shutil.which(command[0]):
            subprocess.run(command, check=False, capture_output=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--remove", action="store_true", help="take the entry out again")
    args = parser.parse_args()

    entry = applications_dir() / f"{APP_ID}.desktop"
    if args.remove:
        entry.unlink(missing_ok=True)
        print(f"removed {entry}")
    else:
        if not PYTHON.exists():
            parser.error(f"{PYTHON} not found; set up .venv first (see gui_python/README.md)")
        entry.parent.mkdir(parents=True, exist_ok=True)
        entry.write_text(desktop_entry(), encoding="utf-8")
        print(f"wrote {entry}")
    refresh_menus(entry.parent)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
