# gui_python — desktop GUI (Python, PyQt6)

The dark-theme PyQt6 windows live here. They import the compiled engine as
`all3dscan_engine` and make engine calls from background QThreads, so the
window stays responsive while a scan is processed.

- `app.py` — the main window, laid out like Revo Scan 6: a menu bar (File,
  Edit, Help, About) over a ribbon of tools,
  the camera parameters and live depth and colour feeds (or, after a scan,
  the models and the primitives fitted to them) on the left, the 3D viewport
  with the scanning-distance gauge in the middle, and the scan controls and
  settings on the right
- `panels.py`, `widgets.py` — the panels and widgets it is built from
- `viewport3d.py` — the 3D view: point clouds drawn with OpenGL, shaded by
  their normals, in scan blue ("No color") or their own colours ("Color"),
  with each fitted primitive's points tinted in its colour
- `app.py`'s **Fusion** builds one model from a scan's frames; the alignment
  is the Qt app's own tracker, reached through the engine
- `rea_engine.py` — scan-to-CAD: fits planes and cylinders to a scan (RANSAC
  in the Rust engine) and writes them to STEP as exact geometry
  (OpenCASCADE, through OCP)

The sensor is either the mock depth camera or a Revopoint POP 4 over USB;
both come from the engine, which drives the POP 4 through the same C++
backend as the legacy app (`src/protocol/RevopointPop4Protocol`).
- `presets.py` — the vendor's POP 4 scan presets (accuracy, tracking, object
  type), which set the gauge's distance bands
- `main.py` — a small window that captures from the mock sensor
- `theme.py` — the colours, the Dark and Light themes, and building a
  custom one
- `theme_dialog.py` — choosing a custom theme's colours
- `common.py` — the icon, app id and OpenGL setup both windows share, and
  dressing the app in a theme
- `assets/` — icons, stylesheets and other UI files

## Setting up

From the repository root (Python 3.11 or newer):

```sh
python3 -m venv .venv
.venv/bin/pip install maturin
.venv/bin/maturin develop --release
.venv/bin/python -m unittest discover -s engine_rust/tests
```

`maturin develop` builds the C++ sensor layer and the Rust engine and
installs them, with numpy, PyQt6 and OpenCASCADE (`cadquery-ocp`), into
`.venv`. Always run it from the
repository root: run from `engine_rust/`, it ignores `pyproject.toml` and
installs a second, conflicting package.

`.venv` is only for this GUI and the engine. The tools in `tools/` and the
C++ app's CMake build use the system Python and its packages, so run them
from a shell where `.venv` is not activated.

## Running

```sh
.venv/bin/python gui_python/app.py                  # the main window
.venv/bin/python gui_python/app.py --smoke-test     # scan, complete, fit, export STEP, exit
.venv/bin/python gui_python/main.py                 # the capture window
.venv/bin/python gui_python/install_desktop_entry.py  # add app.py to the app launcher
```

## The POP 4

Plug a POP 4 in and it can be picked in the scanner list, top right; the app
looks for one every two seconds. Connecting takes a second or two, or up to
a minute and a half when the scanner has to be reset first, and the window
says so while it waits. Then:

- **Camera parameters** (left) drive the scanner's own hardware: the
  pattern projector, the fill light and its brightness, and the depth
  exposure. Each is sent as the control settles, on the capture thread.
- **Scan mode** (right) switches between infrared, the visible blue laser
  and infrared speckle. Going back to infrared reboots the scanner, which
  takes about 45 seconds; the app disables the scan controls until it is
  back. The blue laser is visible but gives too little depth to scan with
  (docs/protocol/RE_NOTES.md, 2026-10-16).
- **If the scanner disconnects** — it drops off USB and re-enumerates after
  a session, sometimes mid-scan — the window says so instead of leaving a
  status that claims it is there, stops the scan, and reconnects once it is
  back. Opening one waits for its first frame, so "connected" means frames
  are flowing.
- **Auto**, beside the depth exposure, tries a few exposures and keeps the
  one that sees most of the object. Measured on a real POP 4, the points in
  a frame ran three to one across the range, so the preset's exposure is
  often far from the best for the scene in front of it.
- **Coverage**, in the status bar, is the share of the depth camera's pixels
  that got a distance. It is only ever a hint: a small object rightly fills
  little of the frame — a 60 mm ring at 360 mm measured 4.7%, and scanned —
  so the viewport complains when a frame holds too few *points* to scan
  with, not when coverage is low.
- The **depth exposure** and the preset's working distance are sent to the
  scanner whenever they change; the "Depth camera" preview shows the
  infrared image the projector lights, and "RGB camera" the colour camera.

Closing the app, or going back to the mock sensor, puts the lights back as
connecting left them and returns the scanner to infrared.

The scanner's own controls are only tested against the hardware, so those
tests ask for a POP 4 and are skipped without one:

```sh
ALL3DSCAN_POP4_TESTS=1 .venv/bin/python -m unittest gui_python/tests/test_pop4_hardware.py
ALL3DSCAN_POP4_SLOW=1 ...   # also the blue laser, which reboots the scanner
cd engine_rust && cargo test --release -- --ignored pop4   # the engine's own
```

The menu bar offers the ribbon's own actions, so the two never disagree
about what is available, plus what has no button: Quit (Ctrl+Q), the
shortcuts (F1), scanning tips, and what the app is built from.

**Theme** switches between Dark, Light and a custom theme of your own, and
the choice is remembered.

**Custom…** builds a theme from three colours rather than asking for all
fifty-four: the **surface** everything sits on, the **text**, and the
**accent**. Every other colour follows, keeping the place in the order of
lightness that the built-in themes gave it, and how much more or less
coloured it was — so a grey hint beside white text stays a hint instead of
turning the full colour. The offsets are taken from whichever built-in
theme matches the surface, light or dark, since on a light surface the
hints are darker than the text and on a dark one lighter. The window
recolours as you pick, and cancelling puts back what was there.
There is one set of styling rules, `assets/dark.qss`, written in the dark
theme's colours; a theme is a table saying what each of those colours
becomes, so a second theme cannot drift away from the first the way a
copied stylesheet would. `theme.py` names every colour by what it is for,
and the window, the distance gauge and the 3D viewport's background all
follow. The gauge's distance bands are left alone: red for too near and
green for good say something, whatever the theme.

In the main window, Space starts, pauses and resumes a scan, Esc cancels it
and Shift+M toggles fullscreen. In the 3D view, drag to rotate, Shift+drag
(or the middle button) to move, use the wheel to zoom and double-click to
frame the cloud again. Completing a scan prints its last point cloud's array
shape to the console; the capture window prints one per capture.
`install_desktop_entry.py --remove` takes the launcher entry out again.

## Fusion

**Fusion** in the ribbon, on by default, builds one model out of a scan's
frames instead of keeping only the last one: each frame is aligned onto the
model so far with point-to-plane ICP and merged into it, carrying its colours
and normals. A frame that cannot be placed is left out rather than smeared
across the model, and the status bar's **Tracking** count shows how many were
placed of those tried.

The cloud is drawn green while scanning and blue once the scan is complete,
as Revo Scan does, so the two states are told apart at a glance.

Fusing a frame costs far more than capturing one, so it runs on its own
thread and takes whichever frame is current when it is free, dropping the
rest; the live view keeps its frame rate and the model fills in as fast as
the machine allows. Turn Fusion off to see only the newest frame, which is
what you want when the scene itself is moving.

Completing a scan keeps the fused model, so the REA fits below measure the
whole object rather than one frame of it.

## Scan to CAD (REA)

Once a scan is complete, the REA tools in the ribbon work on the selected
model's point cloud:

- **Fit plane** and **Fit cylinder** find the plane or the cylinder with
  the most points among those no earlier fit has taken, on a background
  thread. Each lands in *Fitted primitives* on the left with its parameters
  (a plane's normal, centre and size; a cylinder's radius, axis, base and
  height; the points and RMS error of both), and its points are tinted in
  its colour in the 3D view. *Remove* (or Delete) gives the points back.
- **Tolerance** is how far a point may be from a surface and still count:
  1.5 mm, three times the mock sensor's noise, by default.
- **Export STEP** writes the primitives to a STEP file (AP214, millimetres),
  one named, coloured body each: a cylinder as a solid, a plane as a face the
  size of its points (a scan sees one side of a surface, not how thick the
  part behind it is).

The fits turn down surfaces of the wrong shape: a strip of a pipe or a cap of
a ball is not a plane, and a strip of a wall or a band round a ball is not a
cylinder. So the tools can be used in any order, and when nothing of the
kind is left they say so. Coordinates are the scan's own (x right, y down,
z forward from the sensor, which is right-handed), so the STEP geometry
lines up with the point cloud.

From Python:

```python
import all3dscan_engine as engine
import rea_engine

frame = engine.MockDepthSensor(640, 480).capture_frame()
session = rea_engine.ReaSession(frame.points, frame.normals)
wall, pipe = session.fit_plane(), session.fit_cylinder()
session.export_step("scan.step")
```

The 3D view asks for OpenGL ES 3.0 on Wayland, the only kind of context the
Qt in PyQt6 makes there, and for desktop OpenGL 3.3 elsewhere (X11, and the
offscreen platform the tests use). `common.configure_opengl()` has to run
before the QApplication is created.

## Tests

```sh
.venv/bin/python -m unittest discover -s gui_python/tests
```

They draw the window offscreen, so nothing opens on the desktop.
