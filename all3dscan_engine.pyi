"""All3DScan's processing engine, built from engine_rust/ by maturin."""

from typing import TypedDict

import numpy as np
import numpy.typing as npt

__version__: str

class Frame:
    """One captured frame: its point cloud and the images it was made from."""

    @property
    def points(self) -> npt.NDArray[np.float32]:
        """(N, 3) points in millimetres, camera frame."""
    @property
    def colors(self) -> npt.NDArray[np.uint8]:
        """(N, 3) colour of each point."""
    @property
    def normals(self) -> npt.NDArray[np.float32]:
        """(N, 3) unit normal of each point, facing the camera."""
    @property
    def depth_mm(self) -> npt.NDArray[np.uint16]:
        """(height, width) depth in millimetres; 0 where there is none."""
    @property
    def ir(self) -> npt.NDArray[np.uint8]:
        """(height, width) infrared image."""
    @property
    def rgb(self) -> npt.NDArray[np.uint8]:
        """(height, width, 3) image from the colour camera, whose size may
        differ from the depth's, and which is empty until the POP 4's colour
        camera has sent one."""
    @property
    def index(self) -> int:
        """The sensor's frame count after this capture."""

class Pop4Sensor:
    """The Revopoint POP 4 over USB.

    Opening one connects to the scanner and starts it streaming, which takes
    a few seconds, longer if it has to be reset first; it raises OSError with
    the scanner's reason if it cannot. The lights are "projector" and "fill
    light"; the scan modes "infrared", "blue laser" and "speckle". Leaving
    blue laser or speckle reboots the scanner, which takes about 45 seconds.
    """

    def __init__(self) -> None:
        """Connects, starts streaming and waits for a first frame, so that a
        sensor that opens is one that really delivers."""
    def __enter__(self) -> "Pop4Sensor": ...
    def __exit__(self, *exc_info: object) -> None: ...
    def capture_frame(self, voxel_mm: float | None = None,
                      timeout_ms: int = 2000) -> Frame | None:
        """Waits up to timeout_ms for the next frame; None if none came."""
    def set_light(self, light: str, on: bool) -> None: ...
    def set_light_level(self, light: str, level: int) -> None:
        """Sets a light's level, 0 to 255."""
    def light_on(self, light: str) -> bool: ...
    def light_level(self, light: str) -> int: ...
    def set_exposure(self, microseconds: int) -> None: ...
    def set_scan_mode(self, mode: str) -> None:
        """Switches the scan mode; back to "infrared" takes about 45 seconds."""
    def set_depth_range(self, near_mm: float, far_mm: float) -> None:
        """The working distance, in millimetres, the depth decoder searches."""
    def close(self) -> None:
        """Lets the scanner go. Calls after this raise ValueError."""
    @property
    def exposure_us(self) -> int: ...
    @property
    def scan_mode(self) -> str: ...
    @property
    def valid_fraction(self) -> float:
        """The share of the newest frame's pixels that got depth."""
    @property
    def connected(self) -> bool:
        """Whether the scanner is still there; False once it has unplugged."""
    @property
    def serial(self) -> str: ...
    @property
    def backend(self) -> str:
        """Which decoder and calibration are in use."""
    @property
    def last_error(self) -> str:
        """Why the last call that failed did, such as a capture returning None."""
    @property
    def frame_index(self) -> int: ...
    @property
    def closed(self) -> bool: ...

class Fusion:
    """Builds one point cloud out of a scan's frames.

    Each frame is aligned onto the model built so far (point-to-plane ICP)
    and merged into it, thinned to one point per voxel, carrying its colours
    and normals across. The first frame becomes the model and fixes the axes
    the rest are placed in; a frame that cannot be placed is left out.
    """

    def __init__(self, voxel_mm: float = 1.0) -> None: ...
    def add(
        self, points: npt.ArrayLike, colors: npt.ArrayLike, normals: npt.ArrayLike
    ) -> FrameFit:
        """Aligns an (N, 3) frame onto the model and merges it."""
    def cloud(
        self,
    ) -> tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.float32]]:
        """The model so far: its points in millimetres, colours and normals."""
    def clear(self) -> None:
        """Throws the model away and starts again."""
    @property
    def model_points(self) -> int: ...
    @property
    def frames(self) -> int: ...
    @property
    def merged(self) -> int:
        """Frames handed over, and of those, the ones that were merged."""

class FrameFit(TypedDict):
    merged: bool
    """Whether the frame was aligned well enough to be merged."""
    overlap: float
    """The share of the frame's points lying on the model's surface."""
    rmse_mm: float
    """Their RMS distance from it."""
    model_points: int

def remove_speckle(
    points: npt.ArrayLike,
    colors: npt.ArrayLike,
    normals: npt.ArrayLike,
    radius_mm: float = 3.0,
    share: float = 0.2,
) -> tuple[npt.NDArray[np.float32], npt.NDArray[np.uint8], npt.NDArray[np.float32]]:
    """Drops the points with far fewer neighbours than the cloud's typically
    have: the speckle a depth camera leaves in mid-air. Both the radius and
    the threshold tune themselves to the cloud, so one setting suits a dense
    scan and a sparse one."""

def pop4_present() -> bool:
    """Whether a POP 4 is plugged in, without opening it."""

class MockDepthSensor:
    """The mock depth camera from hw_cpp.

    capture() returns an (N, 3) float32 array of x, y, z in millimetres in the
    camera frame (x right, y down, z forward); capture_frame() also returns the
    depth, infrared and colour images. The same seed gives the same frames.
    """

    def __init__(self, width: int = 640, height: int = 480, seed: int = 0) -> None: ...
    def capture(self, voxel_mm: float | None = None) -> npt.NDArray[np.float32]:
        """Captures a frame as an (N, 3) array, optionally thinned to one point per voxel."""
    def capture_frame(self, voxel_mm: float | None = None) -> Frame:
        """Captures a frame with its images; voxel_mm thins the points as capture() does."""
    @property
    def width(self) -> int: ...
    @property
    def height(self) -> int: ...
    @property
    def frame_index(self) -> int:
        """Number of frames captured so far."""

def voxel_downsample(points: npt.ArrayLike, voxel_mm: float) -> npt.NDArray[np.float32]:
    """Thins an (N, 3) array to one point per cube of edge voxel_mm (their centroid)."""

class PlaneFit(TypedDict):
    normal: npt.NDArray[np.float32]
    """(3,) unit normal, facing the sensor."""
    origin: npt.NDArray[np.float32]
    """(3,) the inliers' centroid, on the plane."""
    axes: npt.NDArray[np.float32]
    """(2, 3) unit axes in the plane, along the inliers' longest spread first."""
    extent: npt.NDArray[np.float32]
    """(2, 2) the inliers' (low, high) along each axis, from the origin."""
    inliers: npt.NDArray[np.uint32]
    """Indices of the points on the plane: the largest connected piece of them."""
    claimed: npt.NDArray[np.uint32]
    """The indices the fit takes: the inliers and the noise beside them."""
    rms_mm: float

class CylinderFit(TypedDict):
    axis: npt.NDArray[np.float32]
    """(3,) unit axis, its largest component positive."""
    base: npt.NDArray[np.float32]
    """(3,) the point on the axis level with the lowest inlier."""
    radius_mm: float
    height_mm: float
    inliers: npt.NDArray[np.uint32]
    claimed: npt.NDArray[np.uint32]
    rms_mm: float

def fit_plane(
    points: npt.ArrayLike,
    normals: npt.ArrayLike,
    candidates: npt.ArrayLike | None = None,
    *,
    threshold_mm: float = 1.5,
    iterations: int = 2000,
    min_inliers: int = 200,
    seed: int = 0,
) -> PlaneFit | None:
    """Fits the plane with the most support to (N, 3) points and unit normals
    with RANSAC, among the candidates (indices; all points by default)."""

def fit_cylinder(
    points: npt.ArrayLike,
    normals: npt.ArrayLike,
    candidates: npt.ArrayLike | None = None,
    *,
    radius_mm: tuple[float, float] = (2.0, 1000.0),
    threshold_mm: float = 1.5,
    iterations: int = 2000,
    min_inliers: int = 200,
    seed: int = 0,
) -> CylinderFit | None:
    """Fits the convex cylinder with the most support, as fit_plane() does,
    with a radius in radius_mm."""
