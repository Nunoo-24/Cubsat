"""Host-side wire-format reference for the opt-in ESP-IDF moteus UART adapter.

This module is not used by firmware. It provides deterministic vectors for
reviewing the C encoder/parser while an ESP-IDF toolchain or moteus controller
is unavailable.
"""

import math
import struct

CRC8_TABLE = (
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB,
    0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
)
FRAME_SIZE = 64


def crc8_97(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc = CRC8_TABLE[((crc >> 4) ^ (byte >> 4)) & 0x0F] ^ ((crc << 4) & 0xFF)
        crc = CRC8_TABLE[((crc >> 4) ^ (byte & 0x0F)) & 0x0F] ^ ((crc << 4) & 0xFF)
    return crc


def checked_line(content: str) -> bytes:
    """Build the UART line using the space-before-star checksum convention."""
    raw = content.encode("ascii") + b" "
    return raw + f"*{crc8_97(raw):02X}\n".encode("ascii")


def validate_checked_line(line: bytes) -> bytes:
    stripped = line.rstrip(b"\r\n")
    star = stripped.rfind(b"*")
    if star < 0 or len(stripped) - star != 3:
        raise ValueError("missing moteus UART checksum")
    try:
        expected = int(stripped[star + 1:], 16)
    except ValueError as exc:
        raise ValueError("invalid moteus UART checksum text") from exc
    if crc8_97(stripped[:star]) != expected:
        raise ValueError("moteus UART checksum mismatch")
    return stripped[:star].rstrip(b" ")


def torque_register_frame(motor_torque_nm: float, max_torque_nm: float,
                          watchdog_s: float = 0.1) -> bytes:
    if not all(math.isfinite(x) for x in
               (motor_torque_nm, max_torque_nm, watchdog_s)):
        raise ValueError("torque, maximum torque and watchdog must be finite")
    if max_torque_nm <= 0 or watchdog_s <= 0 or abs(motor_torque_nm) > max_torque_nm:
        raise ValueError("torque command exceeds its positive finite bounds")

    # Mode write must precede the contiguous F32 command-register write.
    result = bytearray(b"\x01\x00\x0a\x08\x0c\x20")
    values = (
        math.nan, 0.0, motor_torque_nm, 0.0, 0.0, max_torque_nm,
        math.nan, watchdog_s, math.nan, math.nan, math.nan, 0.0,
    )
    for value in values:
        result.extend(struct.pack("<f", value))
    # Query mode, fault, and F32 position/velocity/torque.
    result.extend(b"\x11\x00\x11\x0f\x1f\x01")
    result.extend(b"\x50" * (FRAME_SIZE - len(result)))
    assert len(result) == FRAME_SIZE
    return bytes(result)


def torque_command_line(controller_id: int, motor_torque_nm: float,
                        max_torque_nm: float, watchdog_s: float = 0.1) -> bytes:
    if not 1 <= controller_id <= 127:
        raise ValueError("moteus controller ID must be 1..127")
    can_id = 0x8000 | controller_id
    payload = torque_register_frame(motor_torque_nm, max_torque_nm, watchdog_s)
    return checked_line(f"can send {can_id:04X} {payload.hex().upper()} BF")


def velocity_register_frame(target_rev_s: float, max_velocity_rev_s: float,
                            max_torque_nm: float,
                            watchdog_s: float = 0.1) -> bytes:
    """Encode moteus native velocity mode; speed values use revolutions/s."""
    values_to_check = (target_rev_s, max_velocity_rev_s, max_torque_nm,
                       watchdog_s)
    if not all(math.isfinite(x) for x in values_to_check):
        raise ValueError("velocity, limits and watchdog must be finite")
    if max_velocity_rev_s <= 0 or abs(target_rev_s) > max_velocity_rev_s:
        raise ValueError("velocity target exceeds its positive finite bound")
    if max_torque_nm <= 0 or watchdog_s <= 0:
        raise ValueError("torque limit and watchdog must be positive")

    # Position=NaN selects velocity control in moteus position mode. Leave
    # internal PID scales at their configured defaults (NaN), enforce the
    # supplied velocity/torque limits, and set a finite command watchdog.
    result = bytearray(b"\x01\x00\x0a\x08\x0c\x20")
    values = (
        math.nan, target_rev_s, 0.0, math.nan, math.nan, max_torque_nm,
        math.nan, watchdog_s, max_velocity_rev_s, math.nan, math.nan, math.nan,
    )
    for value in values:
        result.extend(struct.pack("<f", value))
    result.extend(b"\x11\x00\x11\x0f\x1f\x01")
    result.extend(b"\x50" * (FRAME_SIZE - len(result)))
    assert len(result) == FRAME_SIZE
    return bytes(result)


def velocity_command_line(controller_id: int, target_rev_s: float,
                          max_velocity_rev_s: float, max_torque_nm: float,
                          watchdog_s: float = 0.1) -> bytes:
    if not 1 <= controller_id <= 127:
        raise ValueError("moteus controller ID must be 1..127")
    can_id = 0x8000 | controller_id
    payload = velocity_register_frame(target_rev_s, max_velocity_rev_s,
                                      max_torque_nm, watchdog_s)
    return checked_line(f"can send {can_id:04X} {payload.hex().upper()} BF")


def parse_status_reply(line: bytes, controller_id: int = 1) -> dict:
    content = validate_checked_line(line).decode("ascii")
    fields = content.split()
    if len(fields) < 3 or fields[0] != "rcv":
        raise ValueError("not a moteus received-frame line")
    if int(fields[1], 16) != controller_id << 8:
        raise ValueError("unexpected moteus reply CAN ID")
    data = bytes.fromhex(fields[2])
    if (len(data) < 20 or data[0:2] != b"\x21\x00" or
            data[3:5] != b"\x21\x0f" or data[6:8] != b"\x2f\x01"):
        raise ValueError("unexpected moteus register reply")
    mode, fault = data[2], data[5]
    position, velocity, torque = struct.unpack("<fff", data[8:20])
    return {
        "mode": mode,
        "fault": fault,
        "position_rev": position,
        "velocity_rev_s": velocity,
        "motor_torque_nm": torque,
        "valid": mode == 10 and fault == 0 and all(
            math.isfinite(x) for x in (position, velocity, torque)
        ),
    }
