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
# The gradient behind a cloud. common.apply_theme() sets these, so the
# viewport matches the window it sits in.
BACKGROUND_TOP = (0.231, 0.239, 0.259)
BACKGROUND_BOTTOM = (0.169, 0.173, 0.188)


def set_background(top: tuple[float, float, float],
                   bottom: tuple[float, float, float]) -> None:
    """The gradient drawn behind the cloud, as the theme wants it. Views
    already open redraw with it on their next paint."""
    global BACKGROUND_TOP, BACKGROUND_BOTTOM
    BACKGROUND_TOP, BACKGROUND_BOTTOM = top, bottom
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

// Three lights rather than one. A single lamp at the eye flattens a scan:
// every surface facing the viewer is equally bright, so shape only reads
// where the silhouette turns. A key light off to one side puts a gradient
// across curvature, a dim fill from the other keeps the shadowed side
// readable, and a rim picks out the edges against the background.
vec3 shade(vec3 normal, vec3 base) {
    const vec3 KEY = normalize(vec3(-0.35, 0.45, 0.82));
    const vec3 FILL = normalize(vec3(0.55, -0.25, 0.55));
    float key = max(dot(normal, KEY), 0.0);
    float fill = max(dot(normal, FILL), 0.0);
    float rim = pow(1.0 - abs(normal.z), 3.0);
    vec3 lit = base * (0.22 + 0.68 * key + 0.22 * fill);
    // A narrow highlight, which is what makes a surface read as solid.
    float gloss = pow(max(dot(normal, normalize(KEY + vec3(0.0, 0.0, 1.0))), 0.0), 48.0);
    // Tinted towards the surface's own colour rather than left white: the
    // cloud's colour says what it is — green while a scan runs, blue once
    // it is done — and a white highlight washes that out.
    vec3 highlight = mix(base, vec3(1.0), 0.35) * 0.45;
    return lit + highlight * gloss + base * 0.25 * rim;
}

void main() {
    vec2 offset = gl_PointCoord - vec2(0.5);
    float sqr = dot(offset, offset);
    if (sqr > 0.25) discard;
    // Treat the splat as a little sphere, so a cloud of them reads as
    // beads rather than flat discs when they are large on screen.
    vec3 normal = normalize(v_normal);
    normal.xy += offset * 1.4;
    normal = normalize(normal);
    vec3 base = u_use_color ? v_color : u_base_color;
    base = mix(base, v_highlight.rgb, v_highlight.a);
    frag = vec4(shade(normal, base), 1.0);
}
"""

# A surface, rather than the points it was built from: no splats, and no
# discarding, since a triangle has no gl_PointCoord to measure against.
SURFACE_VERTEX = """
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
uniform mat4 u_mvp;
uniform mat3 u_view_rotation;
out vec3 v_normal;
void main() {
    gl_Position = u_mvp * vec4(position.x, -position.y, -position.z, 1.0);
    v_normal = u_view_rotation * vec3(normal.x, -normal.y, -normal.z);
}
"""

SURFACE_FRAGMENT = """
in vec3 v_normal;
uniform vec3 u_base_color;
out vec4 frag;

// The same three lights the splats use, so a scan does not change character
// when it is meshed. A surface has real facets, so this is where the
// highlight earns its keep: it is what tells a dent from a bump.
vec3 shade(vec3 normal, vec3 base) {
    const vec3 KEY = normalize(vec3(-0.35, 0.45, 0.82));
    const vec3 FILL = normalize(vec3(0.55, -0.25, 0.55));
    float key = max(dot(normal, KEY), 0.0);
    float fill = max(dot(normal, FILL), 0.0);
    float rim = pow(1.0 - abs(normal.z), 3.0);
    vec3 lit = base * (0.22 + 0.68 * key + 0.22 * fill);
    float gloss = pow(max(dot(normal, normalize(KEY + vec3(0.0, 0.0, 1.0))), 0.0), 48.0);
    // Tinted towards the surface's own colour rather than left white: the
    // cloud's colour says what it is — green while a scan runs, blue once
    // it is done — and a white highlight washes that out.
    vec3 highlight = mix(base, vec3(1.0), 0.35) * 0.45;
    return lit + highlight * gloss + base * 0.25 * rim;
}

void main() {
    // A scan's surface has no inside worth seeing, and its normals are not
    // always consistent, so both faces are lit the same way.
    vec3 normal = normalize(v_normal);
    if (!gl_FrontFacing) normal = -normal;
    frag = vec4(shade(normal, u_base_color), 1.0);
}
"""

# Eye-dome lighting: the shading that makes a raw point cloud readable.
#
# Splats carry a normal each, but a cloud of them still reads flat, because
# nothing tells the eye which parts stand in front of which. This looks at
# the depth buffer instead of the geometry: a pixel noticeably nearer than
# the pixels around it is at an edge, and gets darkened. Contours, creases
# and the far side of a hole all appear, and none of it needs a mesh. It is
# what CloudCompare and potree draw clouds with.
EDL_VERTEX = """
out vec2 v_uv;
void main() {
    // One triangle covering the screen, from the vertex index alone.
    vec2 corner = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    v_uv = corner;
    gl_Position = vec4(corner * 2.0 - 1.0, 0.0, 1.0);
}
"""

EDL_FRAGMENT = """
in vec2 v_uv;
uniform sampler2D u_color;
uniform sampler2D u_depth;
uniform vec2 u_pixel;     // one pixel, in texture coordinates
uniform float u_strength;
uniform float u_near;
uniform float u_far;
out vec4 frag;

// The depth buffer holds a non-linear value; distances have to be compared
// in eye space or everything far away shades the same.
float eye_depth(float stored) {
    float ndc = stored * 2.0 - 1.0;
    return (2.0 * u_near * u_far) / (u_far + u_near - ndc * (u_far - u_near));
}

void main() {
    vec3 color = texture(u_color, v_uv).rgb;
    float stored = texture(u_depth, v_uv).r;
    if (stored >= 1.0) {       // background: nothing in front to shade
        frag = vec4(color, 1.0);
        return;
    }
    float here = log2(eye_depth(stored));
    // Eight neighbours, two pixels out: far enough to catch a contour,
    // near enough not to smear it.
    const vec2 AROUND[8] = vec2[8](
        vec2(1.0, 0.0), vec2(-1.0, 0.0), vec2(0.0, 1.0), vec2(0.0, -1.0),
        vec2(0.7, 0.7), vec2(-0.7, 0.7), vec2(0.7, -0.7), vec2(-0.7, -0.7));
    float sum = 0.0;
    float counted = 0.0;
    for (int i = 0; i < 8; i++) {
        vec2 at = v_uv + AROUND[i] * u_pixel * 2.0;
        float neighbour = texture(u_depth, at).r;
        // A neighbour on the background says nothing about how this pixel
        // sits against the surface it belongs to, so it is not counted.
        // Giving it a made-up distance instead turns every point with
        // nothing behind it black, which is most of a sparse cloud.
        if (neighbour >= 1.0) continue;
        sum += max(0.0, log2(eye_depth(neighbour)) - here);
        counted += 1.0;
    }
    // A pixel with no lit neighbours at all is a point on its own; there is
    // nothing to shade it against, so it keeps its colour.
    float shade = counted > 0.0 ? exp(-u_strength * sum / counted) : 1.0;
    frag = vec4(color * shade, 1.0);
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

    def planes(self) -> tuple[float, float]:
        """Where the view clips, near and far. Wherever the camera has gone,
        just outside the shown cloud.

        The eye-dome pass needs these to turn a stored depth back into a
        distance, and it has to be the pair the projection was built with:
        hence one place rather than two that could drift apart."""
        reach = float(np.linalg.norm(self.eye() - self.scene_centre))
        near = max(reach - 1.5 * self.scene_radius, reach * 1e-3, 1.0)
        return near, max(reach + 1.5 * self.scene_radius, near * 2)

    def projection(self, aspect: float) -> np.ndarray:
        return perspective(self.FOV_Y, aspect, *self.planes())

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


class DepthHalo:
    """An offscreen target for drawing a cloud into, so that eye-dome
    lighting can shade it from its own depth buffer.

    The widget's framebuffer cannot be read back while it is being drawn
    into, so the cloud goes into this one first and is then put on screen
    through the EDL shader. A driver that will not give us the target leaves
    `ready` False and the cloud is drawn straight to the screen instead,
    which is how it looked before.
    """

    #: How hard the shading bites by default. Measured by eye on real
    #: scans: below about 20 the contours barely show, above about 100 the
    #: cloud goes muddy in its own creases.
    STRENGTH = 55.0

    def __init__(self) -> None:
        self.ready = False
        #: Whether the person wants it, as against whether it can be had.
        self.wanted = True
        self.strength = self.STRENGTH
        self._fbo = 0
        self._color = 0
        self._depth = 0
        self._size = (0, 0)
        self._program = 0
        self._uniforms: dict[str, int] = {}
        self._vao = 0

    def start(self, header: str) -> None:
        """Compiles the resolve pass. Safe to call when it cannot work: it
        leaves `ready` False rather than raising, since a viewport without
        eye-dome lighting is still a working viewport."""
        try:
            self._program = _program(header, EDL_VERTEX, EDL_FRAGMENT)
            self._uniforms = {
                name: GL.glGetUniformLocation(self._program, name)
                for name in ("u_color", "u_depth", "u_pixel", "u_strength", "u_near", "u_far")
            }
            self._vao = GL.glGenVertexArrays(1)
            self._fbo = GL.glGenFramebuffers(1)
            self._color = GL.glGenTextures(1)
            self._depth = GL.glGenTextures(1)
            # These are new objects, whatever the last context held: the
            # remembered size has to go with them, or the textures are never
            # attached to this framebuffer and everything draws into nothing.
            self._size = (0, 0)
            self.ready = True
        except Exception as exc:  # noqa: BLE001 - any driver complaint at all
            print(f"3D view: drawing clouds without eye-dome lighting "
                  f"({type(exc).__name__}: {exc})", file=sys.stderr, flush=True)
            self.ready = False

    def _resize(self, width: int, height: int) -> bool:
        if self._size == (width, height):
            return True
        for texture, kind, fmt, pixel in (
                (self._color, GL.GL_RGBA8, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE),
                (self._depth, GL.GL_DEPTH_COMPONENT24, GL.GL_DEPTH_COMPONENT, GL.GL_UNSIGNED_INT)):
            GL.glBindTexture(GL.GL_TEXTURE_2D, texture)
            GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, kind, width, height, 0, fmt, pixel, None)
            for setting in (GL.GL_TEXTURE_MIN_FILTER, GL.GL_TEXTURE_MAG_FILTER):
                GL.glTexParameteri(GL.GL_TEXTURE_2D, setting, GL.GL_NEAREST)
            for axis in (GL.GL_TEXTURE_WRAP_S, GL.GL_TEXTURE_WRAP_T):
                GL.glTexParameteri(GL.GL_TEXTURE_2D, axis, GL.GL_CLAMP_TO_EDGE)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._fbo)
        GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_COLOR_ATTACHMENT0,
                                  GL.GL_TEXTURE_2D, self._color, 0)
        GL.glFramebufferTexture2D(GL.GL_FRAMEBUFFER, GL.GL_DEPTH_ATTACHMENT,
                                  GL.GL_TEXTURE_2D, self._depth, 0)
        whole = GL.glCheckFramebufferStatus(GL.GL_FRAMEBUFFER) == GL.GL_FRAMEBUFFER_COMPLETE
        if whole:
            self._size = (width, height)
        else:
            self.ready = False
            print("3D view: the driver would not give an offscreen target; drawing "
                  "clouds without eye-dome lighting", file=sys.stderr, flush=True)
        return whole

    def release(self) -> None:
        """Frees what it holds. The context must still be current, and this
        must leave it able to start again in a new one."""
        try:
            if self._fbo:
                GL.glDeleteFramebuffers(1, [self._fbo])
            for texture in (self._color, self._depth):
                if texture:
                    GL.glDeleteTextures(1, [texture])
            if self._vao:
                GL.glDeleteVertexArrays(1, [self._vao])
            if self._program:
                GL.glDeleteProgram(self._program)
        except Exception:  # noqa: BLE001 - the context may already be gone
            pass
        self._fbo = self._color = self._depth = self._vao = self._program = 0
        self._size = (0, 0)
        self.ready = False

    def bind(self, width: int, height: int) -> bool:
        """Points drawing at the offscreen target. False if it cannot be
        had, in which case nothing has been changed."""
        if not (self.ready and self.wanted) or width <= 0 or height <= 0:
            return False
        self._was = GL.glGetIntegerv(GL.GL_FRAMEBUFFER_BINDING)
        if not self._resize(width, height):
            GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._was)
            return False
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._fbo)
        return True

    def resolve(self, width: int, height: int, near: float, far: float) -> None:
        """Puts what was drawn back on the screen, shaded by its depth."""
        GL.glBindFramebuffer(GL.GL_FRAMEBUFFER, self._was)
        GL.glViewport(0, 0, width, height)
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glUseProgram(self._program)
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self._color)
        GL.glUniform1i(self._uniforms["u_color"], 0)
        GL.glActiveTexture(GL.GL_TEXTURE1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self._depth)
        GL.glUniform1i(self._uniforms["u_depth"], 1)
        GL.glUniform2f(self._uniforms["u_pixel"], 1.0 / width, 1.0 / height)
        GL.glUniform1f(self._uniforms["u_strength"], self.strength)
        GL.glUniform1f(self._uniforms["u_near"], near)
        GL.glUniform1f(self._uniforms["u_far"], far)
        GL.glBindVertexArray(self._vao)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)
        GL.glBindVertexArray(0)
        GL.glActiveTexture(GL.GL_TEXTURE0)
        GL.glUseProgram(0)


class PointCloudRenderer:
    """The OpenGL side of the viewport: shaders, buffers and drawing. Every
    method but set_cloud() and set_highlight() needs a current context, and
    load_gl() to have run."""

    # Per point: position and normal (float), colour (bytes, RGB) and tint
    # (bytes, RGBA), at attribute locations 0 to 3.
    ATTRIBUTES = ((3, np.float32), (3, np.float32), (3, np.uint8), (4, np.uint8))

    def __init__(self) -> None:
        self.count = 0  # points in the GPU buffers
        self.triangles = 0  # ...and triangles, when a surface is shown
        self.point_size_limit = MAX_SPLAT_PX  # the driver's, once initialised
        # Made here rather than when the context starts, so the settings can
        # reach it before the view has ever been drawn.
        self.halo = DepthHalo()
        self._pending: dict[int, np.ndarray] = {}  # attribute location -> data to upload
        self._surface: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self._es = False
        self._points = self._background = self._surface_program = 0
        self._vao = self._background_vao = self._surface_vao = 0
        self._buffers: list[int] = []
        self._surface_buffers: list[int] = []

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

        self._surface_program = _program(header, SURFACE_VERTEX, SURFACE_FRAGMENT)
        self.halo.start(header)
        self._surface_uniforms = {
            name: GL.glGetUniformLocation(self._surface_program, name)
            for name in ("u_mvp", "u_view_rotation", "u_base_color")
        }
        self._surface_vao = GL.glGenVertexArrays(1)
        GL.glBindVertexArray(self._surface_vao)
        # Positions, normals, and the triples of indices making the triangles.
        self._surface_buffers = list(GL.glGenBuffers(3))
        for location, buffer in enumerate(self._surface_buffers[:2]):
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, buffer)
            GL.glEnableVertexAttribArray(location)
            GL.glVertexAttribPointer(location, 3, GL.GL_FLOAT, False, 0, None)
        GL.glBindVertexArray(0)

    def set_cloud(self, points: np.ndarray, colors: np.ndarray, normals: np.ndarray,
                  highlight: np.ndarray | None = None) -> None:
        """Queues a cloud for the next draw; needs no context. highlight is an
        (N, 4) RGBA tint per point; without one, nothing is tinted."""
        if highlight is None:
            highlight = np.zeros((len(points), 4), np.uint8)
        for location, array in enumerate((points, normals, colors, highlight)):
            self._queue(location, array, len(points))

    def set_surface(self, vertices, normals, indices) -> None:
        """Queues a surface to draw in place of the cloud; needs no context.
        Passing None for the vertices takes it away again."""
        if vertices is None or not len(indices):
            self._surface = None
            self.triangles = 0
            return
        vertices = np.ascontiguousarray(vertices, np.float32)
        normals = np.ascontiguousarray(normals, np.float32)
        indices = np.ascontiguousarray(indices, np.uint32).reshape(-1)
        if vertices.ndim != 2 or vertices.shape[1] != 3 or vertices.shape != normals.shape:
            raise ValueError(f"a surface needs an (N, 3) vertex and normal each, got "
                             f"{vertices.shape} and {normals.shape}")
        if indices.max() >= len(vertices):
            raise ValueError("a triangle points past the end of the vertices")
        self._surface = (vertices, normals, indices)
        self.triangles = len(indices) // 3

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
             base_color: tuple[float, float, float],
             planes: tuple[float, float] = (1.0, 1000.0)) -> None:
        # A cloud is drawn offscreen first so that eye-dome lighting can
        # shade it from its own depth buffer. A surface does not need it:
        # it has real facets and is lit by them.
        shading_a_cloud = not self.triangles and (self.count or self._pending)
        haloed = shading_a_cloud and self.halo.bind(width_px, height_px)
        GL.glViewport(0, 0, width_px, height_px)
        GL.glClearColor(*BACKGROUND_BOTTOM, 1.0)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glUseProgram(self._background)
        GL.glUniform3f(self._background_uniforms["u_top"], *BACKGROUND_TOP)
        GL.glUniform3f(self._background_uniforms["u_bottom"], *BACKGROUND_BOTTOM)
        GL.glBindVertexArray(self._background_vao)
        GL.glDrawArrays(GL.GL_TRIANGLES, 0, 3)

        if self._surface is not None:
            vertices, normals, indices = self._surface
            for buffer, array in zip(self._surface_buffers, (vertices, normals)):
                GL.glBindBuffer(GL.GL_ARRAY_BUFFER, buffer)
                GL.glBufferData(GL.GL_ARRAY_BUFFER, array.nbytes, array, GL.GL_STATIC_DRAW)
            GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self._surface_buffers[2])
            GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices,
                            GL.GL_STATIC_DRAW)
            self._surface = None

        for location, array in self._pending.items():
            GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self._buffers[location])
            GL.glBufferData(GL.GL_ARRAY_BUFFER, array.nbytes, array if array.size else None,
                            GL.GL_DYNAMIC_DRAW)
            if location == 0:
                self.count = len(array)
        self._pending.clear()

        if self.triangles:
            # A surface stands in for the cloud it was built from.
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glDepthFunc(GL.GL_LESS)
            GL.glUseProgram(self._surface_program)
            GL.glUniformMatrix4fv(self._surface_uniforms["u_mvp"], 1, False,
                                  np.ascontiguousarray(mvp.T, np.float32))
            GL.glUniformMatrix3fv(self._surface_uniforms["u_view_rotation"], 1, False,
                                  np.ascontiguousarray(view_rotation.T, np.float32))
            GL.glUniform3f(self._surface_uniforms["u_base_color"], *base_color)
            GL.glBindVertexArray(self._surface_vao)
            GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self._surface_buffers[2])
            GL.glDrawElements(GL.GL_TRIANGLES, self.triangles * 3, GL.GL_UNSIGNED_INT, None)
            GL.glBindVertexArray(0)
            GL.glUseProgram(0)
            if haloed:  # a surface arrived after the cloud was bound for
                self.halo.resolve(width_px, height_px, *planes)
            return

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
        if haloed:
            self.halo.resolve(width_px, height_px, *planes)

    def release(self) -> None:
        """Frees the GL objects; the context must still be current."""
        self.halo.release()
        if self._buffers:
            GL.glDeleteBuffers(len(self._buffers), self._buffers)
            GL.glDeleteBuffers(len(self._surface_buffers), self._surface_buffers)
            GL.glDeleteVertexArrays(3, [self._vao, self._background_vao, self._surface_vao])
            GL.glDeleteProgram(self._points)
            GL.glDeleteProgram(self._background)
            GL.glDeleteProgram(self._surface_program)
            self._buffers = []
            self._surface_buffers = []
        self.count = 0
        self.triangles = 0


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
        # Set from the Settings window; until then, the defaults.
        self._graphics = None
        self.point_count = 0
        self.triangle_count = 0
        self._shown_surface: tuple | None = None
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

    def use_graphics(self, settings) -> None:
        """Takes the graphics settings. The ones that change what is drawn
        take effect on the next frame; the ones that need a different GL
        context (how many samples a pixel gets) wait for a restart."""
        self._graphics = settings
        self.splat_mm = settings.splat_mm
        self.renderer.halo.wanted = settings.eye_dome
        self.renderer.halo.strength = settings.eye_dome_strength
        self.update()

    def describe_gpu(self) -> str:
        """What is drawing, asked of the live context."""
        import graphics as graphics_settings

        if not self.isValid():
            return graphics_settings.describe_gpu()
        self.makeCurrent()
        try:
            return graphics_settings.describe_gpu()
        finally:
            self.doneCurrent()

    def set_surface(self, vertices, normals, indices, frame: bool = False) -> None:
        """Shows a surface in place of the cloud. None for the vertices goes
        back to the cloud."""
        self.renderer.set_surface(vertices, normals, indices)
        self._shown_surface = (None if vertices is None or not len(indices)
                               else (vertices, normals, indices))
        self.triangle_count = self.renderer.triangles
        if self._shown_surface is not None and frame and len(vertices):
            self.camera.frame(np.asarray(vertices, np.float32))
            self._framed = True
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
        self.set_surface(None, None, ())
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
        if self._shown_surface is not None:
            self.renderer.set_surface(*self._shown_surface)

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
                           MAX_SPLAT_PX * ratio, self.use_color, self.base_color,
                           self.camera.planes())

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
