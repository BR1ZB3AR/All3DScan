"""Scan-to-CAD (REA): planes and cylinders fitted to a scanned point cloud,
exported as exact B-Rep geometry in a STEP file.

The fitting is RANSAC in the Rust engine (engine_rust/src/fitting.rs), which
also turns down surfaces of the wrong shape: a strip of a pipe is no plane,
and a band round a ball is no cylinder. The geometry is built and written with
OpenCASCADE, through OCP (the cadquery-ocp wheel). Coordinates are the scan's:
millimetres, x right, y down and z forward from the sensor. That frame is
right-handed, so the exported shapes are not mirrored.

    session = ReaSession(frame.points, frame.normals)
    wall = session.fit_plane()         # a Plane, or None if there is none
    pipe = session.fit_cylinder()      # a Cylinder, or None
    session.export_step("part.step")   # one named, coloured body per primitive

A plane is exported as a planar face, the rectangle its points span, because
a scan sees one side of a surface and not how thick the part behind it is. A
cylinder is exported as a solid.
"""

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCP.collections import Sequence_TDF_Label
from OCP.gp import gp_Ax2, gp_Ax3, gp_Dir, gp_Pln, gp_Pnt
from OCP.IFSelect import IFSelect_ReturnStatus
from OCP.Interface import Interface_Static
from OCP.Message import Message, Message_Gravity
from OCP.Quantity import Quantity_Color, Quantity_TypeOfColor
from OCP.STEPCAFControl import STEPCAFControl_Reader, STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_StepModelType
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDocStd import TDocStd_Document
from OCP.TopoDS import TopoDS_Shape, TopoDS_Shell
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_ColorTool, XCAFDoc_ColorType, XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

import all3dscan_engine as engine

# How far a point may be from a surface and still count as on it. Three times
# the mock sensor's depth noise (0.5 mm); a quieter scanner can use less.
DEFAULT_TOLERANCE_MM = 1.5
# Colours for the primitives, in the order they are fitted; none is near the
# viewport's scan blue.
PALETTE = ("#f5a524", "#3ccf91", "#c084fc", "#f472b6", "#a3e635", "#22d3ee")

# OpenCASCADE reports every STEP transfer on stdout; keep it to errors.
_printers = Message.DefaultMessenger_s().Printers()
for _index in range(1, _printers.Length() + 1):
    _printers.Value(_index).SetTraceLevel(Message_Gravity.Message_Fail)


def direction(v) -> str:
    return f"({v[0]:+.3f}, {v[1]:+.3f}, {v[2]:+.3f})"


def position(p) -> str:
    return f"({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) mm"


@dataclass(frozen=True, eq=False)
class Plane:
    """A plane fitted to part of a cloud, and the rectangle its points span."""

    normal: np.ndarray  # (3,) unit normal, facing the sensor
    origin: np.ndarray  # (3,) the inliers' centroid, which lies on the plane
    axes: np.ndarray  # (2, 3) unit axes in the plane, along the points' longest spread first
    extent: np.ndarray  # (2, 2) the inliers' (low, high) along each axis, from the origin
    inliers: np.ndarray  # indices into the cloud of the points on the plane
    claimed: np.ndarray  # the indices the fit takes from the cloud: inliers and their noise
    rms_mm: float  # the inliers' root-mean-square distance from the plane
    name: str = "Plane"
    color: str = PALETTE[0]

    kind = "Plane"

    @property
    def size_mm(self) -> tuple[float, float]:
        (u0, u1), (v0, v1) = self.extent
        return float(u1 - u0), float(v1 - v0)

    @property
    def centre(self) -> np.ndarray:
        """The middle of the rectangle."""
        return self.origin + self.axes.T @ self.extent.mean(axis=1)

    def summary(self) -> str:
        width, height = self.size_mm
        return f"{width:.0f} × {height:.0f} mm"

    def describe(self) -> list[tuple[str, str]]:
        width, height = self.size_mm
        return [
            ("Normal", direction(self.normal)),
            ("Centre", position(self.centre)),
            ("Size", f"{width:.1f} × {height:.1f} mm"),
            ("Points", f"{len(self.inliers):,}"),
            ("RMS error", f"{self.rms_mm:.2f} mm"),
        ]

    def to_shape(self) -> TopoDS_Shape:
        """An exact planar face, the rectangle the inliers span, in a shell:
        STEP keeps a lone face in one anyway, and a colour given to the face
        alone would not come back with the body."""
        frame = gp_Ax3(gp_Pnt(*map(float, self.origin)), gp_Dir(*map(float, self.normal)),
                       gp_Dir(*map(float, self.axes[0])))
        (u0, u1), (v0, v1) = self.extent.astype(float)
        builder, shell = BRep_Builder(), TopoDS_Shell()
        builder.MakeShell(shell)
        builder.Add(shell, BRepBuilderAPI_MakeFace(gp_Pln(frame), u0, u1, v0, v1).Face())
        return shell


@dataclass(frozen=True, eq=False)
class Cylinder:
    """A cylinder fitted to part of a cloud, as long as its points reach."""

    axis: np.ndarray  # (3,) unit axis, its largest component positive
    base: np.ndarray  # (3,) the point on the axis level with the lowest inlier
    radius_mm: float
    height_mm: float  # the inliers' extent along the axis, from the base
    inliers: np.ndarray
    claimed: np.ndarray
    rms_mm: float  # the inliers' root-mean-square distance from the surface
    name: str = "Cylinder"
    color: str = PALETTE[1]

    kind = "Cylinder"

    def summary(self) -> str:
        return f"⌀ {2 * self.radius_mm:.1f} mm"

    def describe(self) -> list[tuple[str, str]]:
        return [
            ("Radius", f"{self.radius_mm:.2f} mm"),
            ("Diameter", f"{2 * self.radius_mm:.2f} mm"),
            ("Axis", direction(self.axis)),
            ("Base", position(self.base)),
            ("Height", f"{self.height_mm:.1f} mm"),
            ("Points", f"{len(self.inliers):,}"),
            ("RMS error", f"{self.rms_mm:.2f} mm"),
        ]

    def to_shape(self) -> TopoDS_Shape:
        """An exact solid cylinder."""
        frame = gp_Ax2(gp_Pnt(*map(float, self.base)), gp_Dir(*map(float, self.axis)))
        return BRepPrimAPI_MakeCylinder(frame, float(self.radius_mm), float(self.height_mm)).Shape()


Primitive = Plane | Cylinder


def min_points(candidates: int) -> int:
    """The fewest points a fit needs, so that noise does not pass for a
    surface: 200, or a quarter of a percent of a large cloud."""
    return max(200, candidates // 400)


def fit_plane(points, normals, candidates=None, *, tolerance_mm: float = DEFAULT_TOLERANCE_MM,
              seed: int = 0) -> Plane | None:
    """The plane with the most points among the candidates (indices into
    points and normals; all of them by default), or None."""
    count = len(points) if candidates is None else len(candidates)
    found = engine.fit_plane(points, normals, candidates, threshold_mm=tolerance_mm,
                             min_inliers=min_points(count), seed=seed)
    return None if found is None else Plane(**found)


def fit_cylinder(points, normals, candidates=None, *, tolerance_mm: float = DEFAULT_TOLERANCE_MM,
                 seed: int = 0, radius_mm: tuple[float, float] = (2.0, 1000.0)) -> Cylinder | None:
    """The cylinder with the most points among the candidates, seen from
    outside, with a radius in radius_mm; or None."""
    count = len(points) if candidates is None else len(candidates)
    found = engine.fit_cylinder(points, normals, candidates, radius_mm=radius_mm,
                                threshold_mm=tolerance_mm, min_inliers=min_points(count),
                                seed=seed)
    return None if found is None else Cylinder(**found)


class ReaSession:
    """The primitives fitted to one cloud so far. Each fit looks only at the
    points that no earlier primitive has claimed, so each finds a different
    surface.

    Not thread-safe. A GUI can take unclaimed() and next_seed() on its own
    thread, run the module's fit_plane() or fit_cylinder() on a worker, and
    add() what it finds back on its own thread."""

    def __init__(self, points, normals, *, tolerance_mm: float = DEFAULT_TOLERANCE_MM) -> None:
        self.points = np.ascontiguousarray(points, np.float32)
        self.normals = np.ascontiguousarray(normals, np.float32)
        if self.points.ndim != 2 or self.points.shape[1:] != (3,) or (
                self.normals.shape != self.points.shape):
            raise ValueError(f"points and normals must both have shape (N, 3), got "
                             f"{self.points.shape} and {self.normals.shape}")
        self.tolerance_mm = tolerance_mm
        self.primitives: list[Primitive] = []
        self._fits = 0  # fits tried, which seeds each one differently
        self._added = 0
        self._numbers = {"Plane": 0, "Cylinder": 0}

    def unclaimed(self) -> np.ndarray:
        """The indices of the points no primitive has claimed."""
        return np.flatnonzero(self.owners() < 0).astype(np.uint32)

    def owners(self) -> np.ndarray:
        """For each point, the index in primitives of the one that claimed
        it, or -1."""
        owner = np.full(len(self.points), -1, np.int32)
        for index, primitive in enumerate(self.primitives):
            owner[primitive.claimed] = index
        return owner

    def next_seed(self) -> int:
        self._fits += 1
        return self._fits

    def add(self, primitive: Primitive) -> Primitive:
        """Names ("Plane 2") and colours a fitted primitive and keeps it."""
        self._numbers[primitive.kind] += 1
        named = replace(primitive, name=f"{primitive.kind} {self._numbers[primitive.kind]}",
                        color=PALETTE[self._added % len(PALETTE)])
        self._added += 1
        self.primitives.append(named)
        return named

    def remove(self, primitive: Primitive) -> None:
        """Lets a primitive's points be fitted again."""
        self.primitives.remove(primitive)

    def fit_plane(self, tolerance_mm: float | None = None) -> Plane | None:
        found = fit_plane(self.points, self.normals, self.unclaimed(),
                          tolerance_mm=self.tolerance_mm if tolerance_mm is None else tolerance_mm,
                          seed=self.next_seed())
        return None if found is None else self.add(found)

    def fit_cylinder(self, tolerance_mm: float | None = None) -> Cylinder | None:
        found = fit_cylinder(self.points, self.normals, self.unclaimed(),
                             tolerance_mm=self.tolerance_mm if tolerance_mm is None
                             else tolerance_mm, seed=self.next_seed())
        return None if found is None else self.add(found)

    def export_step(self, path) -> Path:
        return export_step(self.primitives, path)


def export_step(primitives, path) -> Path:
    """Writes the primitives to a STEP file (AP214, millimetres): one body
    each, with its name and colour."""
    if not primitives:
        raise ValueError("there are no primitives to export")
    path = Path(path)
    document = _document()
    shapes = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    colours = XCAFDoc_DocumentTool.ColorTool_s(document.Main())
    for primitive in primitives:
        label = shapes.AddShape(primitive.to_shape(), False)
        TDataStd_Name.Set_s(label, TCollection_ExtendedString(primitive.name))
        colours.SetColor(label, _colour(primitive.color), XCAFDoc_ColorType.XCAFDoc_ColorSurf)
    writer = STEPCAFControl_Writer()
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    writer.SetNameMode(True)
    writer.SetColorMode(True)
    if not writer.Transfer(document, STEPControl_StepModelType.STEPControl_AsIs):
        raise RuntimeError("OpenCASCADE could not translate the primitives to STEP")
    if writer.Write(str(path)) != IFSelect_ReturnStatus.IFSelect_RetDone:
        raise OSError(f"could not write {path}")
    return path


def read_step(path) -> list[tuple[str, TopoDS_Shape, str | None]]:
    """The bodies in a STEP file, each with its name and colour (or None),
    as export_step() writes them."""
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(path)) != IFSelect_ReturnStatus.IFSelect_RetDone:
        raise OSError(f"could not read {path} as STEP")
    document = _document()
    if not reader.Transfer(document):
        raise RuntimeError(f"OpenCASCADE could not translate {path}")
    labels = Sequence_TDF_Label()
    XCAFDoc_DocumentTool.ShapeTool_s(document.Main()).GetFreeShapes(labels)
    bodies = []
    for index in range(1, labels.Length() + 1):
        label = labels.Value(index)
        name, colour = TDataStd_Name(), Quantity_Color()
        named = label.FindAttribute(TDataStd_Name.GetID_s(), name)
        coloured = XCAFDoc_ColorTool.GetColor_s(label, XCAFDoc_ColorType.XCAFDoc_ColorSurf,
                                                colour)
        bodies.append((name.Get().ToExtString() if named else "",
                       XCAFDoc_ShapeTool.GetShape_s(label), _hex(colour) if coloured else None))
    return bodies


def rgb(hex_colour: str) -> tuple[int, int, int]:
    """A "#rrggbb" colour as three bytes."""
    return tuple(int(hex_colour[i:i + 2], 16) for i in (1, 3, 5))


def _colour(hex_colour: str) -> Quantity_Color:
    red, green, blue = (c / 255 for c in rgb(hex_colour))
    # STEP files hold sRGB, as hex colours are; OpenCASCADE converts.
    return Quantity_Color(red, green, blue, Quantity_TypeOfColor.Quantity_TOC_sRGB)


def _hex(colour: Quantity_Color) -> str:
    values = colour.Values(Quantity_TypeOfColor.Quantity_TOC_sRGB)
    return "#" + "".join(f"{round(c * 255):02x}" for c in values)


def _document() -> TDocStd_Document:
    document = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    XCAFApp_Application.GetApplication_s().InitDocument(document)
    return document
