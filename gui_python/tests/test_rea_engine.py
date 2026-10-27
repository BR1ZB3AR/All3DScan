"""Tests of scan-to-CAD (rea_engine.py): fitting the mock sensor's scene and
exporting it to STEP, read back with OpenCASCADE and measured.

Run from the repository root after `maturin develop`:

    .venv/bin/python -m unittest discover -s gui_python/tests
"""

import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402
from OCP.BRepAdaptor import BRepAdaptor_Surface  # noqa: E402
from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.GeomAbs import GeomAbs_SurfaceType  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402
from OCP.TopAbs import TopAbs_ShapeEnum  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS  # noqa: E402

import all3dscan_engine as engine  # noqa: E402
import rea_engine as rea  # noqa: E402

FRAME = engine.MockDepthSensor(640, 480, seed=1).capture_frame()


def faces(shape) -> list[BRepAdaptor_Surface]:
    explorer, found = TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_FACE), []
    while explorer.More():
        found.append(BRepAdaptor_Surface(TopoDS.Face(explorer.Current())))
        explorer.Next()
    return found


def xyz(gp) -> np.ndarray:
    return np.array([gp.X(), gp.Y(), gp.Z()])


class FittingTests(unittest.TestCase):
    def setUp(self):
        self.session = rea.ReaSession(FRAME.points, FRAME.normals)

    def check_wall(self, wall):
        np.testing.assert_allclose(wall.normal, [0, 0, -1], atol=1e-3)
        self.assertAlmostEqual(float(wall.centre[2]), 600.0, delta=1.0)
        width, height = wall.size_mm
        self.assertGreater(width * height, 300_000)  # most of the 750 x 563 mm in view

    def check_pipe(self, pipe):
        self.assertAlmostEqual(pipe.radius_mm, 40.0, delta=0.5)
        np.testing.assert_allclose(pipe.axis, [0, 1, 0], atol=0.02)
        np.testing.assert_allclose(pipe.base[[0, 2]], [175, 470], atol=1.0)
        self.assertAlmostEqual(pipe.height_mm, 220.0, delta=3.0)

    def test_the_wall_and_the_pipe_are_found_in_either_order(self):
        self.check_wall(self.session.fit_plane())
        self.check_pipe(self.session.fit_cylinder())
        other = rea.ReaSession(FRAME.points, FRAME.normals)
        self.check_pipe(other.fit_cylinder())
        self.check_wall(other.fit_plane())

    def test_the_ball_left_over_is_neither_a_plane_nor_a_cylinder(self):
        self.session.fit_plane()
        self.session.fit_cylinder()
        self.assertIsNone(self.session.fit_plane())
        self.assertIsNone(self.session.fit_cylinder())
        self.assertEqual(len(self.session.primitives), 2)

    def test_primitives_are_named_and_coloured_in_the_order_they_are_found(self):
        pipe, wall = self.session.fit_cylinder(), self.session.fit_plane()
        self.assertEqual([pipe.name, wall.name], ["Cylinder 1", "Plane 1"])
        self.assertEqual([pipe.color, wall.color], list(rea.PALETTE[:2]))
        self.session.remove(wall)
        self.assertEqual(self.session.fit_plane().name, "Plane 2")  # numbers are not reused

    def test_claimed_points_are_not_fitted_again_until_released(self):
        wall = self.session.fit_plane()
        owners = self.session.owners()
        self.assertTrue(np.all(owners[wall.claimed] == 0))
        self.assertEqual(len(self.session.unclaimed()), len(FRAME.points) - len(wall.claimed))
        self.assertTrue(np.isin(wall.inliers, wall.claimed).all())
        self.session.remove(wall)
        self.assertEqual(len(self.session.unclaimed()), len(FRAME.points))
        self.check_wall(self.session.fit_plane())

    def test_descriptions_give_the_parameters(self):
        pipe = dict(self.session.fit_cylinder().describe())
        self.assertRegex(pipe["Radius"], r"^(39\.[5-9]\d|40\.[0-4]\d) mm$")
        self.assertRegex(pipe["Axis"], r"^\([+-]0\.0\d\d, \+1\.000, [+-]0\.0\d\d\)$")
        wall = self.session.fit_plane()
        self.assertEqual(dict(wall.describe())["Normal"][-7:], "-1.000)")
        self.assertRegex(wall.summary(), r"^\d+ × \d+ mm$")

    def test_a_tolerance_too_tight_for_the_noise_finds_less(self):
        loose = rea.fit_plane(FRAME.points, FRAME.normals)
        tight = rea.fit_plane(FRAME.points, FRAME.normals, tolerance_mm=0.3)
        self.assertLess(len(tight.inliers), len(loose.inliers) * 0.6)

    def test_points_and_normals_must_match(self):
        with self.assertRaises(ValueError):
            rea.ReaSession(FRAME.points, FRAME.normals[:5])


class StepExportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.session = rea.ReaSession(FRAME.points, FRAME.normals)
        cls.pipe = cls.session.fit_cylinder()
        cls.wall = cls.session.fit_plane()
        cls.folder = tempfile.TemporaryDirectory()
        cls.path = cls.session.export_step(Path(cls.folder.name) / "scan.step")
        cls.bodies = {name: (shape, colour) for name, shape, colour in rea.read_step(cls.path)}

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    def test_every_primitive_is_a_named_coloured_body(self):
        self.assertEqual(sorted(self.bodies), ["Cylinder 1", "Plane 1"])
        self.assertEqual(self.bodies["Cylinder 1"][1], self.pipe.color)
        self.assertEqual(self.bodies["Plane 1"][1], self.wall.color)

    def test_the_header_says_millimetres(self):
        text = self.path.read_text()
        self.assertIn("SI_UNIT(.MILLI.,.METRE.)", text)
        self.assertIn("PRODUCT('Cylinder 1'", text)

    def test_the_cylinder_is_an_exact_solid(self):
        shape, _ = self.bodies["Cylinder 1"]
        self.assertEqual(shape.ShapeType(), TopAbs_ShapeEnum.TopAbs_SOLID)
        kinds = [face.GetType() for face in faces(shape)]
        self.assertEqual(sorted(k.name for k in kinds),
                         ["GeomAbs_Cylinder", "GeomAbs_Plane", "GeomAbs_Plane"])
        side = next(f for f in faces(shape) if f.GetType() == GeomAbs_SurfaceType.GeomAbs_Cylinder)
        cylinder = side.Cylinder()
        self.assertAlmostEqual(cylinder.Radius(), self.pipe.radius_mm, places=4)
        np.testing.assert_allclose(xyz(cylinder.Axis().Direction()), self.pipe.axis, atol=1e-6)
        np.testing.assert_allclose(xyz(cylinder.Axis().Location()), self.pipe.base, atol=1e-3)
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, props)
        self.assertAlmostEqual(props.Mass() / (math.pi * self.pipe.radius_mm ** 2
                                               * self.pipe.height_mm), 1.0, places=6)

    def test_the_plane_is_an_exact_face_the_size_of_its_points(self):
        shape, _ = self.bodies["Plane 1"]
        self.assertEqual(shape.ShapeType(), TopAbs_ShapeEnum.TopAbs_SHELL)
        [face] = faces(shape)
        self.assertEqual(face.GetType(), GeomAbs_SurfaceType.GeomAbs_Plane)
        normal = xyz(face.Plane().Axis().Direction())
        self.assertAlmostEqual(abs(float(normal @ self.wall.normal)), 1.0, places=6)
        props = GProp_GProps()
        BRepGProp.SurfaceProperties_s(shape, props)
        width, height = self.wall.size_mm
        self.assertAlmostEqual(props.Mass() / (width * height), 1.0, places=6)
        np.testing.assert_allclose(xyz(props.CentreOfMass()), self.wall.centre, atol=1e-2)

    def test_exporting_nothing_or_nowhere_fails_clearly(self):
        with self.assertRaises(ValueError):
            rea.export_step([], Path(self.folder.name) / "empty.step")
        with self.assertRaises(OSError):
            rea.export_step([self.pipe], Path(self.folder.name) / "missing" / "x.step")


if __name__ == "__main__":
    unittest.main()
