import importlib.util
import json
import subprocess
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


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


if __name__ == "__main__":
    unittest.main()
