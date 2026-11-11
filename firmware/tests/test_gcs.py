"""Host-side regression tests; no serial hardware or third-party modules needed."""
import pathlib
import subprocess
import sys
import unittest

TOOLS = pathlib.Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))
import gcs


class FramingTests(unittest.TestCase):
    def test_fragmentation_noise_and_overlapping_sync(self):
        wire = b"noise\xa5" + gcs.frame_ping() + gcs.frame_set_sticks(0, 0, 0, 0)
        parser = gcs.FrameParser()
        frames = []
        for byte in wire:
            frames += parser.feed(bytes([byte]))
        self.assertEqual([f[0] for f in frames], [gcs.CMD_PING, gcs.CMD_SET_STICKS])
        self.assertEqual(parser.buf, b"")

    def test_corrupt_length_does_not_swallow_next_frame(self):
        valid = gcs.frame_ping()
        broken = b"\xa5\x5a\x01\x06\x00" + b"X" + valid
        parser = gcs.FrameParser()
        self.assertEqual(parser.feed(broken), [(gcs.CMD_PING, b"")])
        self.assertEqual(parser.errors, 1)

    def test_oversized_length_recovers(self):
        parser = gcs.FrameParser()
        self.assertEqual(parser.feed(b"\xa5\x5a\x01\xff\xff" + gcs.frame_ping()),
                         [(gcs.CMD_PING, b"")])
        self.assertEqual(parser.errors, 1)

    def test_truncated_event_is_reported(self):
        self.assertIn("decode error", gcs.format_decoded(gcs.MSG_EVENT, b"\x01\x05ab"))

    def test_invalid_control_values(self):
        for args in [(float("nan"), 0, 0, 0), (0, 0, 0, 1.01), (0, -2, 0, 0)]:
            with self.assertRaises(ValueError):
                gcs.frame_set_sticks(*args)
        for mode in [-1, 6]:
            with self.assertRaises(ValueError):
                gcs.frame_set_sticks(0, 0, 0, 0, flightmode=mode)
        for channels in [[], [1500] * 21, [-1], [65536]]:
            with self.assertRaises(ValueError):
                gcs.frame_set_channels(channels)

    def test_import_without_site_packages(self):
        subprocess.run([sys.executable, "-S", "-c", "import gcs"], cwd=TOOLS, check=True)

    def test_cli_rejects_unsafe_combinations_before_opening_port(self):
        for args in [
            ["--control", "unused", "--sticks", "nan", "0", "0", "0"],
            ["--control", "unused", "--channels", "1500", "--ping"],
            ["--control", "unused", "--ping", "--arm"],
            ["--control", "unused", "--ping", "--duration", "nan"],
            ["--control", "unused", "--ping", "--rate", "0"],
            ["--control", "unused", "--ping", "--telemetry", "unused"],
        ]:
            result = subprocess.run([sys.executable, str(TOOLS / "gcs.py"), *args],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)
            self.assertNotIn("Cannot open", result.stderr)


if __name__ == "__main__":
    unittest.main()
