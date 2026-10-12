import importlib.util
import json
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _open3d_available() -> bool:
    return importlib.util.find_spec("open3d") is not None


def _cv2_available() -> bool:
    return (importlib.util.find_spec("cv2") is not None
            and importlib.util.find_spec("numpy") is not None)


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class UsbmonParserTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parser = load_module(
            "usbmon_parse", ROOT / "tools" / "re_probe" / "usbmon_parse.py"
        )

    def test_correlates_xu_set_and_get_transactions(self):
        capture = [
            "aaaa 100 S Co:1:003:0 s 21 01 0a00 0400 0004 4 = 01 00 00 00\n",
            "aaaa 110 C Co:1:003:0 0 4 >\n",
            "bbbb 120 S Ci:1:003:0 s a1 81 0600 0400 0004 4 <\n",
            "bbbb 130 C Ci:1:003:0 0 4 = 11 22 33 44\n",
        ]
        hid, xu = self.parser.collect_transactions(capture)
        self.assertEqual(hid, [])
        self.assertEqual(len(xu), 2)
        self.assertEqual(xu[0]["direction"], "set")
        self.assertEqual(xu[0]["selector"], 10)
        self.assertEqual(xu[0]["data"], "01000000")
        self.assertEqual(xu[1]["direction"], "get")
        self.assertEqual(xu[1]["selector"], 6)
        self.assertEqual(xu[1]["data"], "11223344")


class MeshWorkerTests(unittest.TestCase):
    def test_ping_and_quit(self):
        worker = subprocess.Popen(
            [sys.executable, str(ROOT / "tools" / "pipeline" / "mesh_worker.py")],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        assert worker.stdin is not None
        assert worker.stdout is not None
        worker.stdin.write('{"cmd":"ping"}\n{"cmd":"quit"}\n')
        worker.stdin.flush()
        ping = json.loads(worker.stdout.readline())
        quit_reply = json.loads(worker.stdout.readline())
        self.assertTrue(ping["ok"])
        self.assertTrue(ping["pong"])
        self.assertTrue(quit_reply["quit"])
        self.assertEqual(worker.wait(timeout=5), 0)

    def test_capabilities_advertises_tsdf(self):
        worker = subprocess.Popen(
            [sys.executable, str(ROOT / "tools" / "pipeline" / "mesh_worker.py")],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        assert worker.stdin is not None
        assert worker.stdout is not None
        worker.stdin.write('{"cmd":"capabilities"}\n{"cmd":"quit"}\n')
        worker.stdin.flush()
        caps = json.loads(worker.stdout.readline())
        worker.wait(timeout=5)
        self.assertTrue(caps["ok"])
        self.assertIn("tsdf", caps["commands"])

    def test_tsdf_missing_manifest_reports_error(self):
        worker = subprocess.Popen(
            [sys.executable, str(ROOT / "tools" / "pipeline" / "mesh_worker.py")],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        assert worker.stdin is not None
        assert worker.stdout is not None
        worker.stdin.write(
            '{"cmd":"tsdf","manifest":"/nonexistent/all3dscan_manifest.json"}\n'
            '{"cmd":"quit"}\n'
        )
        worker.stdin.flush()
        reply = json.loads(worker.stdout.readline())
        worker.wait(timeout=5)
        self.assertFalse(reply["ok"])
        self.assertIn("manifest", reply["error"])

    @unittest.skipUnless(_open3d_available(), "open3d not installed")
    def test_tsdf_reconstructs_mesh_from_synthetic_depth(self):
        # A single fronto-parallel plane at 0.30 m should fuse into a surface.
        width, height = 64, 64
        fx = fy = 100.0
        cx, cy = width / 2.0, height / 2.0
        depth_mm = 300
        depth = struct.pack("<%dH" % (width * height), *([depth_mm] * (width * height)))
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            depth_path = tmp_path / "kf0000.bin"
            depth_path.write_bytes(depth)
            # Registered grayscale texture (same size as depth * 3 channels).
            color_path = tmp_path / "kf0000.rgb"
            color_path.write_bytes(bytes([128]) * (width * height * 3))
            out_path = tmp_path / "tsdf.mesh.ply"
            manifest = {
                "out": str(out_path),
                "voxel_length": 0.004,
                "sdf_trunc": 0.02,
                "depth_scale": 1000.0,
                "depth_min": 0.10,
                "depth_max": 0.60,
                "frames": [
                    {
                        "depth": str(depth_path),
                        "color": str(color_path),
                        "width": width,
                        "height": height,
                        "fx": fx,
                        "fy": fy,
                        "cx": cx,
                        "cy": cy,
                        "pose": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
                    }
                ],
            }
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(json.dumps(manifest))

            worker = subprocess.Popen(
                [sys.executable, str(ROOT / "tools" / "pipeline" / "mesh_worker.py")],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                text=True,
            )
            assert worker.stdin is not None
            assert worker.stdout is not None
            worker.stdin.write(
                json.dumps({"cmd": "tsdf", "manifest": str(manifest_path)}) + "\n"
            )
            worker.stdin.write('{"cmd":"quit"}\n')
            worker.stdin.flush()
            reply = json.loads(worker.stdout.readline())
            worker.wait(timeout=30)

            self.assertTrue(reply["ok"], reply)
            self.assertGreater(reply["vertices"], 0)
            self.assertGreater(reply["triangles"], 0)
            self.assertTrue(out_path.is_file())


class PcapngImportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.importer = load_module(
            "pcapng_import", ROOT / "tools" / "re_probe" / "pcapng_import.py"
        )

    def test_extracts_xu_set_and_get_from_tshark_json(self):
        packets = [
            {
                "_source": {
                    "layers": {
                        "frame.time_epoch": ["100.0"],
                        "usb.urb_id": ["0x1"],
                        "usb.bmRequestType": ["0x21"],
                        "usb.bRequest": ["0x01"],
                        "usb.wValue": ["0x0a00"],
                        "usb.wIndex": ["0x0400"],
                        "usb.wLength": ["4"],
                        "usb.capdata": ["01:00:00:00"],
                    }
                }
            },
            {
                "_source": {
                    "layers": {
                        "frame.time_epoch": ["100.1"],
                        "usb.urb_id": ["0x2"],
                        "usb.bmRequestType": ["0xa1"],
                        "usb.bRequest": ["0x81"],
                        "usb.wValue": ["0x0600"],
                        "usb.wIndex": ["0x0400"],
                        "usb.wLength": ["4"],
                    }
                }
            },
            {
                "_source": {
                    "layers": {
                        "frame.time_epoch": ["100.2"],
                        "usb.urb_id": ["0x2"],
                        "usb.capdata": ["11:22:33:44"],
                    }
                }
            },
        ]
        xu = self.importer.extract_xu_transactions(packets)
        self.assertEqual(len(xu), 2)
        self.assertEqual(xu[0]["direction"], "set")
        self.assertEqual(xu[0]["selector"], 10)
        self.assertEqual(xu[0]["data"], "01000000")
        self.assertEqual(xu[1]["direction"], "get")
        self.assertEqual(xu[1]["selector"], 6)
        self.assertEqual(xu[1]["data"], "11223344")


class XuProbeDryRunTests(unittest.TestCase):
    def test_dry_run_sequence_prints_transcript(self):
        fixture = ROOT / "build" / "tmp_xu_sequence.json"
        fixture.parent.mkdir(parents=True, exist_ok=True)
        fixture.write_text(
            json.dumps(
                {
                    "xu": [
                        {
                            "direction": "set",
                            "request": 1,
                            "interface": 0,
                            "status": "0",
                            "selector": 10,
                            "data": "01000000",
                            "timestamp_us": 1000,
                        },
                        {
                            "direction": "set",
                            "request": 1,
                            "interface": 0,
                            "status": "0",
                            "selector": 7,
                            "data": "01000000",
                            "timestamp_us": 1500,
                        },
                    ]
                }
            ),
            encoding="utf-8",
        )
        completed = subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools" / "re_probe" / "xu_probe.py"),
                "--interface",
                "0",
                "--sequence",
                str(fixture),
                "--dry-run",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("SET CS=10", completed.stdout)
        self.assertIn("SET CS=7", completed.stdout)


@unittest.skipUnless(_cv2_available(), "numpy/opencv not installed")
class Pop4RectifyFitTests(unittest.TestCase):
    def test_recovers_planted_row_warp(self):
        import cv2
        import numpy as np

        fit = load_module("pop4_rectify_fit", ROOT / "tools" / "re_probe" / "pop4_rectify_fit.py")
        rng = np.random.default_rng(7)
        H, W = fit.H, fit.W
        left = np.where(rng.random((H, W + 40)) < 0.16, 200.0, 20.0).astype(np.float32)
        left = cv2.GaussianBlur(left, (0, 0), 1.0)
        truth = np.array([0.002, -0.02, 1.0, -0.01, 0.12, -0.005])
        # right(xR, yR) = left(xR + 35, yL(xR, yR)): a 35 px disparity plane
        ys, xs = np.mgrid[0:H, 0:W].astype(np.float64)
        rows = fit.mapped_row(truth, xs, ys).astype(np.float32)
        right = cv2.remap(left, (xs + 35).astype(np.float32), rows, cv2.INTER_LINEAR)
        coeffs, rms, kept = fit.fit(left[:, :W].copy(), right, fit.DEFAULT_COEFFS.copy(),
                                    iterations=3, verbose=False)
        self.assertLess(rms, 0.3)
        self.assertGreater(kept, 100)
        # Compare mapped rows, not raw coefficients (well-conditioned check).
        err = np.abs(fit.mapped_row(coeffs, xs, ys) - fit.mapped_row(truth, xs, ys))
        self.assertLess(float(np.percentile(err[20:-20, 20:-20], 95)), 0.5)


if __name__ == "__main__":
    unittest.main()
