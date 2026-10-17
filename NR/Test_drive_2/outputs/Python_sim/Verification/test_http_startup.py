"""Regressions for readable local HTTP service startup failures."""

from contextlib import redirect_stderr
import errno
import io
from unittest import TestCase
from unittest.mock import patch

from Ground_Station import main as ground_station
from Sim_Environment import main as simulation_environment
from http_startup import open_http_server


class HttpStartupTests(TestCase):
    def test_successful_server_factory_returns_server(self):
        instance = object()
        factory = lambda address, handler: instance
        self.assertIs(open_http_server(factory, ("127.0.0.1", 8123), object,
                                       "Test service"), instance)

    def test_permission_error_is_reported_without_traceback(self):
        def denied(_address, _handler):
            raise PermissionError(errno.EPERM, "Operation not permitted")

        output = io.StringIO()
        with redirect_stderr(output), self.assertRaises(SystemExit) as raised:
            open_http_server(denied, ("127.0.0.1", 8000), object,
                             "Simulation Environment")
        self.assertEqual(raised.exception.code, 1)
        self.assertIn("127.0.0.1:8000", output.getvalue())
        self.assertIn("denied local socket creation", output.getvalue())
        self.assertNotIn("Traceback", output.getvalue())

    def test_address_in_use_error_suggests_stopping_existing_service(self):
        def occupied(_address, _handler):
            raise OSError(errno.EADDRINUSE, "Address already in use")

        output = io.StringIO()
        with redirect_stderr(output), self.assertRaises(SystemExit):
            open_http_server(occupied, ("127.0.0.1", 8001), object,
                             "Ground Station")
        self.assertIn("address is already in use", output.getvalue())
        self.assertIn("stop the existing service", output.getvalue())

    def test_both_web_services_use_shared_startup_diagnostics(self):
        for module, expected_name in (
            (simulation_environment, "Simulation Environment"),
            (ground_station, "Ground Station"),
        ):
            with self.subTest(service=expected_name):
                with patch.object(module, "open_http_server",
                                  side_effect=SystemExit(1)) as open_server:
                    with self.assertRaises(SystemExit):
                        module.main()
                open_server.assert_called_once_with(
                    module.ThreadingHTTPServer,
                    (module.HOST, module.PORT),
                    module.Handler,
                    expected_name,
                )


if __name__ == "__main__":
    import unittest

    unittest.main()
