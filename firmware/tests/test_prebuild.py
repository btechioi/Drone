"""Exercise the pre-build guard without PlatformIO or modifying the submodule."""
import pathlib
import tempfile
import types
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "firmware/tools/pre_gen_param.py"


class PrebuildTests(unittest.TestCase):
    def test_receiver_guard(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            project = root / "firmware"
            project.mkdir()
            header = root / "madflight/src/rcl/RclGizmoSerial.h"
            header.parent.mkdir(parents=True)
            cfg = root / "madflight/src/cfg/param.yaml"
            cfg.parent.mkdir(parents=True)
            cfg.write_text("param: {}")
            generator = types.SimpleNamespace(generate=mock.Mock())
            def run():
                with mock.patch.dict("sys.modules", {"gen_param": generator}):
                    exec(compile(SCRIPT.read_text(), str(SCRIPT), "exec"), {
                        "Import": lambda name: None,
                        "env": {"PROJECT_DIR": str(project)},
                    })
            header.write_text("unpatched receiver")
            with self.assertRaisesRegex(RuntimeError, "Unsafe or unrecognized"):
                run()
            generator.generate.assert_not_called()
            header.write_text("frame[0] = cmd;\n          uint16_t crc_calc\n"
                              "if(!_handle_frame())\n!std::isfinite(throttle)")
            run()
            generator.generate.assert_called_once_with(str(cfg))
