import contextlib
import io
import unittest
from unittest.mock import patch
from . import sil


class SilRuntimeTests(unittest.TestCase):
    def test_cycle_retry_delay_grows_and_caps(self):
        delays = [sil._cycle_retry_delay(count) for count in range(1, 8)]
        expected = [0.05, 0.1, 0.2, 0.4, 0.5, 0.5, 0.5]
        for actual, target in zip(delays, expected):
            with self.subTest(delay=actual):
                self.assertAlmostEqual(actual, target)

    def test_main_reports_wait_then_connects_after_bridge_recovers(self):
        cycle = {
            "commands": [],
            "observation": {"sequence": 1},
        }
        fake_onboard = unittest.mock.Mock()
        fake_onboard.last_command_id = 0
        fake_onboard.process_observation.return_value = {"telemetry": True}
        output = io.StringIO()

        with (
            patch.object(sil, "OnboardSoftware", return_value=fake_onboard),
            patch.object(sil, "request_json",
                         side_effect=[ConnectionResetError("bridge starting"),
                                      cycle, {}])
                as request_json,
            patch.object(sil, "request_camera_frame", return_value=b"frame"),
            patch.object(sil.time, "monotonic", return_value=0.0),
            patch.object(sil.time, "sleep",
                         side_effect=[None, KeyboardInterrupt]),
            contextlib.redirect_stdout(output),
        ):
            sil.main()

        lines = output.getvalue().splitlines()
        self.assertTrue(any("Starting CubeSat SIL; waiting" in line for line in lines))
        self.assertTrue(any("Waiting for simulation bridge" in line for line in lines))
        self.assertTrue(any("Simulation bridge connected" in line for line in lines))
        self.assertTrue(any("CubeSat SIL stopped" in line for line in lines))
        self.assertEqual(request_json.call_count, 3)


if __name__ == "__main__":
    unittest.main()
