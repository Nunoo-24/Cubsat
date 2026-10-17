import math
import struct
import unittest

from .moteus_protocol_reference import (
    checked_line,
    parse_status_reply,
    torque_command_line,
    torque_register_frame,
    validate_checked_line,
    velocity_command_line,
    velocity_register_frame,
)


class MoteusProtocolReferenceTests(unittest.TestCase):
    def test_checked_line_includes_space_in_crc_scope(self):
        line = checked_line("can on")
        self.assertTrue(line.startswith(b"can on *"))
        self.assertEqual(validate_checked_line(line), b"can on")
        with self.assertRaisesRegex(ValueError, "checksum mismatch"):
            validate_checked_line(line.replace(b"can on", b"can off"))

    def test_torque_frame_has_command_query_and_fd_padding(self):
        frame = torque_register_frame(0.001, 0.001)
        self.assertEqual(len(frame), 64)
        self.assertEqual(frame[:6], b"\x01\x00\x0a\x08\x0c\x20")
        torque = struct.unpack("<f", frame[6 + 2 * 4:6 + 3 * 4])[0]
        watchdog = struct.unpack("<f", frame[6 + 7 * 4:6 + 8 * 4])[0]
        self.assertAlmostEqual(torque, 0.001)
        self.assertAlmostEqual(watchdog, 0.1)
        self.assertEqual(frame[54:60], b"\x11\x00\x11\x0f\x1f\x01")
        self.assertEqual(frame[60:], b"\x50" * 4)

    def test_torque_line_uses_reply_id_fd_brs_and_uart_checksum(self):
        line = torque_command_line(1, -0.001, 0.001)
        content = validate_checked_line(line).decode("ascii")
        self.assertTrue(content.startswith("can send 8001 "))
        self.assertTrue(content.endswith(" BF"))
        self.assertEqual(len(content.split()[3]), 128)

    def test_status_parser_accepts_healthy_reply_and_rejects_fault(self):
        response = bytearray(b"\x21\x00\x0a\x21\x0f\x00\x2f\x01")
        response.extend(struct.pack("<fff", 0.25, -2.0, 0.0005))
        healthy = checked_line(f"rcv 100 {response.hex().upper()} e B F r")
        parsed = parse_status_reply(healthy)
        self.assertTrue(parsed["valid"])
        self.assertAlmostEqual(parsed["velocity_rev_s"], -2.0)

        response[5] = 3
        faulted = checked_line(f"rcv 100 {response.hex().upper()} e B F r")
        self.assertFalse(parse_status_reply(faulted)["valid"])

    def test_torque_frame_rejects_nonfinite_or_out_of_bound_commands(self):
        with self.assertRaises(ValueError):
            torque_register_frame(math.inf, 0.001)
        with self.assertRaises(ValueError):
            torque_register_frame(0.002, 0.001)

    def test_velocity_frame_encodes_3000_rpm_as_50_revolutions_per_second(self):
        frame = velocity_register_frame(50.0, 50.0, 0.02)
        self.assertEqual(len(frame), 64)
        self.assertEqual(frame[:6], b"\x01\x00\x0a\x08\x0c\x20")
        values = struct.unpack("<12f", frame[6:54])
        self.assertTrue(math.isnan(values[0]))
        self.assertEqual(values[1], 50.0)
        self.assertEqual(values[2], 0.0)
        self.assertAlmostEqual(values[5], 0.02)
        self.assertAlmostEqual(values[7], 0.1)
        self.assertEqual(values[8], 50.0)
        self.assertEqual(frame[54:60], b"\x11\x00\x11\x0f\x1f\x01")

    def test_velocity_frame_rejects_out_of_range_and_nonfinite_requests(self):
        for target, limit in ((50.1, 50.0), (-50.1, 50.0),
                              (math.inf, 50.0), (1.0, math.nan)):
            with self.subTest(target=target, limit=limit), self.assertRaises(ValueError):
                velocity_register_frame(target, limit, 0.02)

    def test_velocity_uart_line_uses_same_checked_fdcanusb_envelope(self):
        line = velocity_command_line(1, 50.0, 50.0, 0.02)
        content = validate_checked_line(line).decode("ascii")
        self.assertTrue(content.startswith("can send 8001 "))
        self.assertTrue(content.endswith(" BF"))
        self.assertEqual(len(content.split()[3]), 128)


if __name__ == "__main__":
    unittest.main()
