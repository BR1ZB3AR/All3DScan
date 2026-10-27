"""Tests of the all3dscan_engine Python module.

Run from the repository root after `maturin develop`:

    .venv/bin/python -m unittest discover -s engine_rust/tests
"""

import threading
import time
import unittest

import numpy as np

import all3dscan_engine as engine


class FrameTests(unittest.TestCase):
    def test_frame_carries_the_cloud_and_its_images(self):
        frame = engine.MockDepthSensor(64, 48, seed=7).capture_frame()
        self.assertEqual((frame.points.shape[1], frame.points.dtype), (3, np.float32))
        self.assertEqual((frame.depth_mm.shape, frame.depth_mm.dtype), ((48, 64), np.uint16))
        self.assertEqual((frame.ir.shape, frame.ir.dtype), ((48, 64), np.uint8))
        self.assertEqual((frame.rgb.shape, frame.rgb.dtype), ((48, 64, 3), np.uint8))
        self.assertEqual(frame.index, 1)
        # A pixel has depth exactly when it produced a point.
        self.assertEqual(int((frame.depth_mm > 0).sum()), len(frame.points))
        self.assertEqual(repr(frame), f"Frame(index=1, points={len(frame.points)}, 64x48 pixels)")

    def test_every_point_has_a_colour_and_a_normal_facing_the_camera(self):
        frame = engine.MockDepthSensor(64, 48, seed=7).capture_frame()
        n = len(frame.points)
        self.assertEqual((frame.colors.shape, frame.colors.dtype), ((n, 3), np.uint8))
        self.assertEqual((frame.normals.shape, frame.normals.dtype), ((n, 3), np.float32))
        np.testing.assert_allclose(np.linalg.norm(frame.normals, axis=1), 1.0, atol=1e-4)
        self.assertTrue(np.all(np.sum(frame.normals * frame.points, axis=1) < 0))
        # The colour of each point is the colour of its own pixel.
        np.testing.assert_array_equal(frame.colors, frame.rgb[frame.depth_mm > 0])

    def test_thinning_reduces_the_points_but_keeps_the_images(self):
        frame = engine.MockDepthSensor(160, 120, seed=2).capture_frame(voxel_mm=10.0)
        self.assertLess(len(frame.points), int((frame.depth_mm > 0).sum()) // 2)
        self.assertEqual(frame.colors.shape, frame.points.shape)
        self.assertEqual(frame.normals.shape, frame.points.shape)
        np.testing.assert_allclose(np.linalg.norm(frame.normals, axis=1), 1.0, atol=1e-4)
        self.assertEqual(frame.rgb.shape, (120, 160, 3))

    def test_same_seed_gives_the_same_images(self):
        a = engine.MockDepthSensor(32, 24, seed=5).capture_frame()
        b = engine.MockDepthSensor(32, 24, seed=5).capture_frame()
        for name in ("points", "depth_mm", "ir", "rgb"):
            np.testing.assert_array_equal(getattr(a, name), getattr(b, name), name)

    def test_the_ball_moves_between_frames(self):
        sensor = engine.MockDepthSensor(64, 48, seed=1)
        first, later = sensor.capture_frame(), [sensor.capture_frame() for _ in range(20)][-1]
        self.assertFalse(np.array_equal(first.depth_mm, later.depth_mm))


class MockDepthSensorTests(unittest.TestCase):
    def test_capture_returns_an_n_by_3_float32_array(self):
        cloud = engine.MockDepthSensor(64, 48, seed=7).capture()
        self.assertEqual(cloud.dtype, np.float32)
        self.assertEqual(cloud.shape[1], 3)
        # About 2% of pixels drop out; every other pixel gives one point.
        self.assertGreater(cloud.shape[0], 64 * 48 * 9 // 10)
        self.assertTrue(np.all(cloud[:, 2] > 0))

    def test_same_seed_gives_the_same_frames(self):
        a = engine.MockDepthSensor(32, 24, seed=1)
        b = engine.MockDepthSensor(32, 24, seed=1)
        for _ in range(3):
            np.testing.assert_array_equal(a.capture(), b.capture())

    def test_capture_can_thin_the_frame(self):
        full = engine.MockDepthSensor(160, 120, seed=2).capture()
        sensor = engine.MockDepthSensor(160, 120, seed=2)
        thinned = sensor.capture(voxel_mm=10.0)
        self.assertLess(len(thinned), len(full) // 2)
        self.assertEqual(sensor.frame_index, 1)
        self.assertEqual((sensor.width, sensor.height), (160, 120))

    def test_bad_arguments_raise_value_error(self):
        with self.assertRaises(ValueError):
            engine.MockDepthSensor(0, 48)
        with self.assertRaises(ValueError):
            engine.MockDepthSensor(8, 8).capture(voxel_mm=0.0)
        with self.assertRaises(ValueError):
            engine.voxel_downsample(np.zeros((4, 2)), 5.0)

    def test_voxel_downsample_accepts_other_dtypes(self):
        points = np.array([[1, 1, 1], [3, 2, 1], [2, 3, 4]], dtype=np.float64)
        np.testing.assert_array_equal(engine.voxel_downsample(points, 5.0), [[2, 2, 2]])

    def test_a_bad_voxel_size_is_refused_before_a_frame_is_taken(self):
        sensor = engine.MockDepthSensor(8, 8)
        for capture in (sensor.capture, sensor.capture_frame):
            with self.assertRaises(ValueError):
                capture(voxel_mm=-1.0)
        self.assertEqual(sensor.frame_index, 0)


class FittingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        frame = engine.MockDepthSensor(640, 480, seed=1).capture_frame()
        cls.points, cls.normals = frame.points, frame.normals

    def test_fits_the_mock_wall_and_pipe(self):
        wall = engine.fit_plane(self.points, self.normals)
        self.assertEqual(sorted(wall), ["axes", "claimed", "extent", "inliers", "normal",
                                        "origin", "rms_mm"])
        np.testing.assert_allclose(wall["normal"], [0, 0, -1], atol=1e-3)
        self.assertAlmostEqual(float(wall["origin"][2]), 600.0, delta=1.0)
        self.assertEqual((wall["axes"].shape, wall["extent"].shape), ((2, 3), (2, 2)))
        self.assertEqual(wall["inliers"].dtype, np.uint32)
        self.assertLess(wall["rms_mm"], 0.6)  # the mock's depth noise is 0.5 mm

        rest = np.setdiff1d(np.arange(len(self.points), dtype=np.uint32), wall["claimed"])
        pipe = engine.fit_cylinder(self.points, self.normals, rest)
        self.assertEqual(sorted(pipe), ["axis", "base", "claimed", "height_mm", "inliers",
                                        "radius_mm", "rms_mm"])
        self.assertAlmostEqual(pipe["radius_mm"], 40.0, delta=0.5)
        np.testing.assert_allclose(pipe["axis"], [0, 1, 0], atol=0.02)
        self.assertAlmostEqual(pipe["height_mm"], 220.0, delta=3.0)

        rest = np.setdiff1d(rest, pipe["claimed"])
        self.assertIsNone(engine.fit_plane(self.points, self.normals, rest))
        self.assertIsNone(engine.fit_cylinder(self.points, self.normals, rest))

    def test_candidates_limit_the_fit_and_may_be_any_integer_type(self):
        far = np.flatnonzero(self.points[:, 2] > 590)  # int64: the wall alone
        wall = engine.fit_plane(self.points, self.normals, far)
        self.assertTrue(np.isin(wall["inliers"], far).all())

    def test_the_same_seed_gives_the_same_fit(self):
        a = engine.fit_cylinder(self.points, self.normals, seed=4)
        b = engine.fit_cylinder(self.points, self.normals, seed=4)
        np.testing.assert_array_equal(a["inliers"], b["inliers"])
        self.assertEqual(a["radius_mm"], b["radius_mm"])

    def test_bad_arguments_raise_value_error(self):
        points, normals = self.points, self.normals
        for call in (
            lambda: engine.fit_plane(points[:, :2], normals),
            lambda: engine.fit_plane(points, normals[:10]),
            lambda: engine.fit_plane(points, normals, [len(points)]),
            lambda: engine.fit_plane(points, normals, [-1]),
            lambda: engine.fit_plane(points, normals, threshold_mm=0.0),
            lambda: engine.fit_plane(points, normals, iterations=0),
            lambda: engine.fit_cylinder(points, normals, radius_mm=(5.0, 1.0)),
            lambda: engine.fit_cylinder(points, normals, radius_mm=(0.0, 1.0)),
        ):
            with self.assertRaises((ValueError, OverflowError)):
                call()

    def test_fitting_on_a_worker_thread_leaves_the_main_thread_running(self):
        start = time.perf_counter()
        engine.fit_plane(self.points, self.normals)
        fit_s = time.perf_counter() - start
        if fit_s < 0.02:
            self.skipTest(f"a fit took {fit_s:.3f}s, too quick to tell a stall apart")
        worker = threading.Thread(
            target=lambda: [engine.fit_plane(self.points, self.normals) for _ in range(3)])
        longest_gap = 0.0
        worker.start()
        last = time.perf_counter()
        while worker.is_alive():
            now = time.perf_counter()
            longest_gap = max(longest_gap, now - last)
            last = now
        worker.join()
        self.assertLess(longest_gap, fit_s / 2,
                        f"main thread stalled {longest_gap:.3f}s; one fit takes {fit_s:.3f}s")

    def test_capture_on_a_worker_thread_leaves_the_main_thread_running(self):
        # The GUI relies on this: capture() must release the GIL while it works.
        engine.MockDepthSensor(8, 8).capture()  # first-call setup, kept out of the timing
        sensor = engine.MockDepthSensor(2048, 1536, seed=3)
        start = time.perf_counter()
        sensor.capture()
        capture_s = time.perf_counter() - start
        if capture_s < 0.02:
            self.skipTest(f"a capture took {capture_s:.3f}s, too quick to tell a stall apart")

        worker = threading.Thread(target=lambda: [sensor.capture() for _ in range(3)])
        longest_gap = 0.0
        worker.start()
        last = time.perf_counter()
        while worker.is_alive():
            now = time.perf_counter()
            longest_gap = max(longest_gap, now - last)
            last = now
        worker.join()
        self.assertLess(
            longest_gap,
            capture_s / 2,
            f"main thread stalled {longest_gap:.3f}s; one capture takes {capture_s:.3f}s",
        )


if __name__ == "__main__":
    unittest.main()
