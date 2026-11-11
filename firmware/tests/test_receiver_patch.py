"""Compile the real pinned receiver with the project patch and lightweight HAL stubs."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]
TESTS = ROOT / "firmware/tests"


class ReceiverPatchTests(unittest.TestCase):
    def test_real_receiver(self):
        source = ROOT / "madflight/src/rcl/RclGizmoSerial.h"
        if not source.is_file():
            self.skipTest("initialize the madflight submodule to test its receiver")
        with tempfile.TemporaryDirectory() as tmp:
            tmp = pathlib.Path(tmp)
            shutil.copytree(TESTS / "stubs", tmp / "src")
            target = tmp / "src/rcl/RclGizmoSerial.h"
            shutil.copy(source, target)
            patch = ROOT / "firmware/patches/madflight-serial-crc.patch"
            # The developer's checkout may already have the patch applied.
            if "if(!_handle_frame())" not in target.read_text():
                subprocess.run(["patch", "-p1", "-i", str(patch)], cwd=tmp, check=True,
                               capture_output=True)
            binary = tmp / "receiver-test"
            subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(tmp / "src"), str(TESTS / "test_serial_receiver.cpp"),
                            "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
