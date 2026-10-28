"""The 3D viewport: point clouds drawn with OpenGL, in Revo Scan's style.

Points are drawn as round splats sized in millimetres and shaded by their
normals under a light at the viewer, in scan blue ("No color") or their own
colour ("Color"), over a grey gradient. Any point can also be tinted, to show
which points a fit took. Drag to orbit, Shift+drag or the middle button to
move, the wheel to zoom, and double-click to frame the cloud.

OpenGL differs by platform, as measured on this PC (NVIDIA, KDE Wayland):
PyQt6's Qt only creates OpenGL ES contexts on Wayland, through EGL; X11 gives
desktop OpenGL 3.3 core through GLX, and Qt's offscreen platform desktop
OpenGL through GLX too, sometimes a compatibility context whatever is asked
for. So the shaders are written once and given a GLSL ES 3.00 or GLSL 3.30
header to suit, common.configure_opengl() asks for the right kind of context
before the QApplication exists, and PyOpenGL is only loaded once the first
context is current, so that it looks functions up the way that context was
made.
"""

import ctypes
import math
import os
import sys

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QImage, QOpenGLContext
from PyQt6.QtOpenGLWidgets import QOpenGLWidget

# Revo Scan draws the cloud green while it is scanning and blue once the
# scan is finished, so the two states are told apart at a glance.
SCAN_BLUE = (0.16, 0.42, 0.86)
SCAN_GREEN = (0.24, 0.84, 0.33)
BACKGROUND_TOP = (0.231, 0.239, 0.259)  # #3b3d42
BACKGROUND_BOTTOM = (0.169, 0.173, 0.188)  # #2b2c30
MAX_SPLAT_PX = 16.0  # the largest a splat is drawn, in logical pixels

# PyOpenGL, loaded by load_gl() once a context is current.
GL = None
GLError: type[Exception] = Exception

POINTS_VERTEX = """
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
layout(location = 3) in vec4 highlight;
uniform mat4 u_mvp;
uniform mat3 u_view_rotation;
uniform float u_splat_px;  // a splat's size in pixels at 1 mm from the eye
uniform float u_max_splat_px;
out vec3 v_normal;
out vec3 v_color;
out vec4 v_highlight;
void main() {
    // The sensor's axes (x right, y down, z forward) to OpenGL's (y up, z back).
    gl_Position = u_mvp * vec4(position.x, -position.y, -position.z, 1.0);
    gl_PointSize = clamp(u_splat_px / gl_Position.w, 1.0, u_max_splat_px);
    v_normal = u_view_rotation * vec3(normal.x, -normal.y, -normal.z);
    v_color = color;
    v_highlight = highlight;
}
"""

POINTS_FRAGMENT = """
in vec3 v_normal;
in vec3 v_color;
in vec4 v_highlight;
uniform bool u_use_color;
uniform vec3 u_base_color;
out vec4 frag;
void main() {
    vec2 offset = gl_PointCoord - vec2(0.5);
    if (dot(offset, offset) > 0.25) discard;
    // A light at the eye: brightest where the surface faces the viewer.
    float light = 0.3 + 0.7 * abs(normalize(v_normal).z);
    vec3 base = u_use_color ? v_color : u_base_color;
    // A tinted point takes the tint's colour, as far as its alpha says.
    frag = vec4(mix(base, v_highlight.rgb, v_highlight.a) * light, 1.0);
}
"""

BACKGROUND_VERTEX = """
out float v_height;
void main() {
    // One triangle that covers the screen, from the vertex number alone.
    vec2 corner = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
    v_height = corner.y;
    gl_Position = vec4(corner * 2.0 - 1.0, 0.0, 1.0);
}
"""

BACKGROUND_FRAGMENT = """
in float v_height;
uniform vec3 u_top;
uniform vec3 u_bottom;
out vec4 frag;
void main() {
    frag = vec4(mix(u_bottom, u_top, clamp(v_height, 0.0, 1.0)), 1.0);
}
"""


def current_context_is_egl() -> bool:
    """Whether the current OpenGL context was made with EGL (else GLX)."""
    try:
        egl = ctypes.CDLL("libEGL.so.1")
    except OSError:
        return False
    egl.eglGetCurrentContext.restype = ctypes.c_void_p
    return bool(egl.eglGetCurrentContext())


def load_gl():
    """Loads PyOpenGL for the current context and returns its GL module.

    PyOpenGL looks functions up through EGL or GLX, and has to use the one
    that made the context; it reads which from PYOPENGL_PLATFORM when it is
    first imported, so this runs once a context is current."""
    global GL, GLError
    if GL is None:
        made_with = "egl" if current_context_is_egl() else "glx"
        chosen = os.environ.setdefault("PYOPENGL_PLATFORM", made_with)
        if chosen != made_with:
            print(f"viewport3d: PYOPENGL_PLATFORM is {chosen}, but the OpenGL context was "
                  f"made with {made_with}", file=sys.stderr)
        from OpenGL import GL as gl
        from OpenGL.error import GLError as error
        GL, GLError = gl, error
    return GL


def image_pixels(image: QImage) -> np.ndarray:
    """An image's pixels as a (height, width, 3) RGB array, such as what
    PointCloudView.grabFramebuffer() drew."""
    image = image.convertToFormat(QImage.Format.Format_RGB888)
    rows = np.frombuffer(image.constBits().asstring(image.sizeInBytes()), np.uint8)
    rows = rows.reshape(image.height(), image.bytesPerLine())
    return rows[:, :image.width() * 3].reshape(image.height(), image.width(), 3)


def look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    true_up = np.cross(right, forward)
    view = np.identity(4)
    view[0, :3], view[1, :3], view[2, :3] = right, true_up, -forward
    view[:3, 3] = -view[:3, :3] @ eye
    return view


def perspective(fov_y_deg: float, aspect: float, near: float, far: float) -> np.ndarray:
    f = 1.0 / math.tan(math.radians(fov_y_deg) / 2)
    projection = np.zeros((4, 4))
    projection[0, 0] = f / aspect
    projection[1, 1] = f
    projection[2, 2] = (far + near) / (near - far)
    projection[2, 3] = 2 * far * near / (near - far)
    projection[3, 2] = -1.0
    return projection


def bounds(points: np.ndarray) -> tuple[np.ndarray, float]:
    """The centre, in OpenGL's axes, and half the diagonal of a cloud's
    bounding box; the cloud is in the sensor's axes."""
    lo, hi = points.min(axis=0).astype(np.float64), points.max(axis=0).astype(np.float64)
    return (lo + hi) / 2 * (1.0, -1.0, -1.0), max(float(np.linalg.norm(hi - lo)) / 2, 1.0)


class OrbitCamera:
    """Orbits a target point, in OpenGL's axes (y up, looking down -z)."""

    FOV_Y = 40.0  # degrees

    def __init__(self) -> None:
        self.target = np.array([0.0, 0.0, -400.0])
        self.distance = 400.0
        self.yaw = 0.0
        self.pitch = 0.0
        # The shown cloud's centre and size, which the clipping planes follow.
        self.scene_centre = self.target.copy()
        self.scene_radius = 100.0

    def frame(self, points: np.ndarray) -> None:
        """Frames a cloud given in the sensor's axes, seen from the sensor's side."""
        self.show(points)
        self.target = self.scene_centre.copy()
        self.yaw = self.pitch = 0.0
        # Where the sensor stood, unless that is too close to show it all.
        fits = self.scene_radius / math.tan(math.radians(self.FOV_Y) / 2) * 1.1
        self.distance = max(-self.target[2], fits)

    def show(self, points: np.ndarray) -> None:
        """Keeps the clipping planes around a cloud without moving the camera."""
        self.scene_centre, self.scene_radius = bounds(points)

    def eye(self) -> np.ndarray:
        cos_pitch = math.cos(self.pitch)
        return self.target + self.distance * np.array([
            cos_pitch * math.sin(self.yaw), math.sin(self.pitch), cos_pitch * math.cos(self.yaw)])

    def view_matrix(self) -> np.ndarray:
        return look_at(self.eye(), self.target, np.array([0.0, 1.0, 0.0]))

    def projection(self, aspect: float) -> np.ndarray:
        # Wherever the camera has gone, clip just outside the shown cloud.
        reach = float(np.linalg.norm(self.eye() - self.scene_centre))
        near = max(reach - 1.5 * self.scene_radius, reach * 1e-3, 1.0)
        far = max(reach + 1.5 * self.scene_radius, near * 2)
        return perspective(self.FOV_Y, aspect, near, far)

    def orbit(self, dx_px: float, dy_px: float) -> None:
        self.yaw -= dx_px * 0.008
        self.pitch = min(max(self.pitch + dy_px * 0.008, -1.5), 1.5)

    def pan(self, dx_px: float, dy_px: float, viewport_height_px: int) -> None:
        mm_per_px = 2 * self.distance * math.tan(math.radians(self.FOV_Y) / 2) / max(
            viewport_height_px, 1)
        view = self.view_matrix()
        self.target = self.target - view[0, :3] * dx_px * mm_per_px + view[1, :3] * dy_px * mm_per_px

    def zoom(self, steps: float) -> None:
        self.distance = max(self.distance * 0.85 ** steps, 1.0)


def _log_text(log) -> str:
    # PyOpenGL gives a shader's log as bytes or as str, depending on version.
    return log.decode(errors="replace") if isinstance(log, bytes) else str(log)


def _program(header: str, vertex: str, fragment: str) -> int:
    def compile_shader(kind, source: str) -> int:
        shader = GL.glCreateShader(kind)
        GL.glShaderSource(shader, header + source)
        GL.glCompileShader(shader)
        if not GL.glGetShaderiv(shader, GL.GL_COMPILE_STATUS):
            raise RuntimeError(f"shader did not compile ({header.splitlines()[0]}): "
                               f"{_log_text(GL.glGetShaderInfoLog(shader)).strip()}")
        return shader

    program = GL.glCreateProgram()
    shaders = [compile_shader(GL.GL_VERTEX_SHADER, vertex),
               compile_shader(GL.GL_FRAGMENT_SHADER, fragment)]
    for shader in shaders:
        GL.glAttachShader(program, shader)
    GL.glLinkProgram(program)
    for shader in shaders:
        GL.glDeleteShader(shader)
    if not GL.glGetProgramiv(program, GL.GL_LINK_STATUS):
        raise RuntimeError(f"shaders did not link: "
                           f"{_log_text(GL.glGetProgramInfoLog(program)).strip()}")
    return program


class PointCloudRenderer:
    """The OpenGL side of the viewport: shaders, buffers and drawing. Every
    method but set_cloud() and set_highlight() needs a current context, and
    load_gl() to have run."""

    # Per point: position and normal (float), colour (bytes, RGB) and tint
    # (bytes, RGBA), at attribute locations 0 to 3.
    ATTRIBUTES = ((3, np.float32), (3, np.float32), (3, np.uint8), (4, np.uint8))

    def __init__(self) -> None:
        self.count = 0  # points in the GPU buffers
        self.point_size_limit = MAX_SPLAT_PX  # the driver's, once initialised
        self._pending: dict[int, np.ndarray] = {}  # attribute location -> data to upload
        self._es = False
        self._points = self._background = 0
        self._vao = self._background_vao = 0
        self._buffers: list[int] = []

    @property
    def initialized(self) -> bool:
        return bool(self._buffers)

    def initialize(self, es: bool) -> None:
        self._es = es
        header = "#version 300 es\nprecision highp float;\n" if es else "#version 330 core\n"
        self._points = _program(header, POINTS_VERTEX, POINTS_FRAGMENT)
        self._background = _program(header, BACKGROUND_VERTEX, BACKGROUND_FRAGMENT)
        self._uniforms = {name: GL.glGetUniformLocation(self._points, name)
                          for name in ("u_mvp", "u_view_rotation", "u_splat_px",
                                       "u_max_splat_px", "u_use_color", "u_base_color")}
        self._background_uniforms = {name: GL.glGetUniformLocation(self._background, name)
                                     for name in ("u_top", "u_bottom")}
        self.point_size_limit = float(GL.glGetFloatv(GL.GL_ALIASED_POINT_SIZE_RANGE)[1])
        self._vao = GL.glGenVertexArrays(1)
        GL.glBindVertexArray(self._vao)
        self._buffers = list(GL.glGenBuffers(len(self.ATTRIBUTES)))
        for location, (buffer, (size, dtype)) in enumerate(zip(self._buffers, self.ATTRIBUTES)):
            kind, normalized = (GL.GL_FLOAT, False) if dtype is np.float32 else (
                GL.GL_UNSIGNED_BYTE, True)
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, buffer)
            GL.glEnableVertexAttribArray(location)
            GL.glVertexAttribPointer(location, size, kind, normalized, 0, None)
        GL.glBindVertexArray(0)
        # Core profiles need a vertex array bound to draw, even one with no buffers.
        self._background_vao = GL.glGenVertexArrays(1)

    def set_cloud(self, points: np.ndarray, colors: np.ndarray, normals: np.ndarray,
                  highlight: np.ndarray | None = None) -> None:
        """Queues a cloud for the next draw; needs no context. highlight is an
        (N, 4) RGBA tint per point; without one, nothing is tinted."""
        if highlight is None:
            highlight = np.zeros((len(points), 4), np.uint8)
        for location, array in enumerate((points, normals, colors, highlight)):
            self._queue(location, array, len(points))

    def set_highlight(self, highlight: np.ndarray) -> None:
        """Queues a new tint for the points already set; needs no context."""
        self._queue(3, highlight, len(self._pending[0]) if 0 in self._pending else self.count)

    def _queue(self, location: int, array: np.ndarray, count: int) -> None:
        size, dtype = self.ATTRIBUTES[location]
        array = np.ascontiguousarray(array, dtype)
        if array.shape != (count, size):
            raise ValueError(f"attribute {location} must have shape ({count}, {size}), "
                             f"got {array.shape}")
        self._pending[location] = array

    def draw(self, width_px: int, height_px: int, mvp: np.ndarray, view_rotation: np.ndarray,
             splat_px: float, max_splat_px: float, use_color: bool,
             base_color: tuple[float, float, float]) -> None:
        GL.glViewport(0, 0, width_px, height_px)
        GL.glClearColor(*BACKGROUND_BOTTOM, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glUseProgram(self._background)
        GL.glUniform3f(self._background_uniforms["u_top"], *BACKGROUND_TOP)
        GL.glUniform3f(self._background_uniforms["u_bottom"], *BACKGROUND_BOTTOM)
        GL.glBindVertexArray(self._background_vao)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)

        for location, array in self._pending.items():
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._buffers[location])
            GL.glBufferData(GL.GL_ARRAY_BUFFER, array.nbytes, array if array.size else None,
                            GL.GL_DYNAMIC_DRAW)
            if location == 0:
                self.count = len(array)
        self._pending.clear()

        if self.count:
            # Qt can change GL state between frames, so set everything relied on.
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glDepthFunc(GL.GL_LESS)
            if not self._es:
                GL.glEnable(GL.GL_PROGRAM_POINT_SIZE)
                try:
                    # gl_PointCoord needs point sprites in a compatibility context;
                    # a core profile always has them and rejects the switch.
                    GL.glEnable(GL.GL_POINT_SPRITE)
                except GLError:
                    pass
            GL.glUseProgram(self._points)
            GL.glUniformMatrix4fv(self._uniforms["u_mvp"], 1, False,
                                  np.ascontiguousarray(mvp.T, np.float32))
            GL.glUniformMatrix3fv(self._uniforms["u_view_rotation"], 1, False,
                                  np.ascontiguousarray(view_rotation.T, np.float32))
            GL.glUniform1f(self._uniforms["u_splat_px"], splat_px)
            GL.glUniform1f(self._uniforms["u_max_splat_px"],
                           min(max_splat_px, self.point_size_limit))
            GL.glUniform1i(self._uniforms["u_use_color"], int(use_color))
            GL.glUniform3f(self._uniforms["u_base_color"], *base_color)
            GL.glBindVertexArray(self._vao)
            GL.glDrawArrays(GL.GL_POINTS, 0, self.count)
        GL.glBindVertexArray(0)
        GL.glUseProgram(0)

    def release(self) -> None:
        """Frees the GL objects; the context must still be current."""
        if self._buffers:
            GL.glDeleteBuffers(len(self._buffers), self._buffers)
            GL.glDeleteVertexArrays(2, [self._vao, self._background_vao])
            GL.glDeleteProgram(self._points)
            GL.glDeleteProgram(self._background)
            self._buffers = []
        self.count = 0


class PointCloudView(QOpenGLWidget):
    """The viewport widget: an orbit camera over a PointCloudRenderer."""

    # Emitted with the reason when OpenGL cannot draw here; the view then stays empty.
    failed = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        self.camera = OrbitCamera()
        self.renderer = PointCloudRenderer()
        self.use_color = False
        self.base_color = SCAN_BLUE  # when not showing each point's own colour
        self.splat_mm = 2.0
        self.point_count = 0
        self.error: str | None = None
        # The cloud shown, kept to frame it again on request and to upload it
        # again if Qt replaces the context.
        self._cloud: tuple[np.ndarray, ...] | None = None
        self._framed = False
        self._drag: tuple[Qt.MouseButton, float, float] | None = None

    def set_cloud(self, points: np.ndarray, colors: np.ndarray, normals: np.ndarray,
                  highlight: np.ndarray | None = None, frame: bool = False) -> None:
        """Shows a cloud in the sensor's axes, tinted point by point by an
        optional (N, 4) RGBA highlight. It is framed the first time after
        clear(), or when asked; otherwise the camera stays put."""
        if highlight is None:
            highlight = np.zeros((len(points), 4), np.uint8)
        self.renderer.set_cloud(points, colors, normals, highlight)
        self._cloud = (points, colors, normals, highlight)
        self.point_count = len(points)
        if len(points):
            if frame or not self._framed:
                self.camera.frame(points)
                self._framed = True
            else:
                self.camera.show(points)
        self.update()

    def set_highlight(self, highlight: np.ndarray) -> None:
        """Tints the shown cloud afresh."""
        if self._cloud is not None:
            self.renderer.set_highlight(highlight)
            self._cloud = (*self._cloud[:3], highlight)
            self.update()

    def clear(self) -> None:
        empty = np.zeros((0, 3), np.float32)
        self.renderer.set_cloud(empty, empty.astype(np.uint8), empty)
        self.point_count = 0
        self._cloud = None
        self._framed = False
        self.update()

    def set_color_mode(self, use_color: bool) -> None:
        self.use_color = use_color
        self.update()

    def set_base_color(self, color: tuple[float, float, float]) -> None:
        """The colour a cloud is drawn in while "No color" is chosen."""
        self.base_color = color
        self.update()

    def _fail(self, reason: str) -> None:
        if self.error is None:
            self.error = reason
            print(f"3D view unavailable: {reason}", file=sys.stderr, flush=True)
            self.failed.emit(reason)

    # QOpenGLWidget.

    def initializeGL(self) -> None:
        try:
            load_gl()
            self.renderer.initialize(self.context().isOpenGLES())
        except Exception as exc:  # a missing driver or a shader the driver rejects
            self._fail(f"{type(exc).__name__}: {exc}")
            return
        self.context().aboutToBeDestroyed.connect(self._release)
        if self._cloud is not None:  # a new context after the old one went
            self.renderer.set_cloud(*self._cloud)

    def paintEvent(self, event) -> None:
        super().paintEvent(event)
        # Qt only tells that it could not make a context by never initialising
        # when it had all it needed: a size and a window to draw in.
        if not self.isValid() and not self.size().isEmpty() and self.window().windowHandle():
            self._fail("Qt could not create an OpenGL context")

    def paintGL(self) -> None:
        if not self.renderer.initialized:
            return
        ratio = self.devicePixelRatioF()
        width, height = max(round(self.width() * ratio), 1), max(round(self.height() * ratio), 1)
        view = self.camera.view_matrix()
        mvp = self.camera.projection(width / height) @ view
        focal_px = height / (2 * math.tan(math.radians(OrbitCamera.FOV_Y) / 2))
        self.renderer.draw(width, height, mvp, view[:3, :3], focal_px * self.splat_mm,
                           MAX_SPLAT_PX * ratio, self.use_color, self.base_color)

    def _release(self) -> None:
        # Qt destroys a context when its view goes, and Python may collect a
        # view in the middle of another one's drawing: leave current whatever
        # context was, or the other view goes on drawing into none.
        previous = QOpenGLContext.currentContext()
        surface = previous.surface() if previous is not None else None
        self.makeCurrent()
        self.renderer.release()
        self.doneCurrent()
        if previous is not None and previous is not self.context():
            previous.makeCurrent(surface)

    # Mouse: orbit, move and zoom.

    def mousePressEvent(self, event) -> None:
        position = event.position()
        self._drag = (event.button(), position.x(), position.y())

    def mouseMoveEvent(self, event) -> None:
        if self._drag is None:
            return
        button, x, y = self._drag
        position = event.position()
        dx, dy = position.x() - x, position.y() - y
        self._drag = (button, position.x(), position.y())
        shift = event.modifiers() & Qt.KeyboardModifier.ShiftModifier
        if button == Qt.MouseButton.MiddleButton or (button == Qt.MouseButton.LeftButton
                                                      and shift):
            self.camera.pan(dx, dy, self.height())
        elif button == Qt.MouseButton.LeftButton:
            self.camera.orbit(dx, dy)
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        self._drag = None

    def wheelEvent(self, event) -> None:
        self.camera.zoom(event.angleDelta().y() / 120)
        self.update()

    def mouseDoubleClickEvent(self, event) -> None:
        if self._cloud is not None and len(self._cloud[0]):
            self.camera.frame(self._cloud[0])
            self.update()
