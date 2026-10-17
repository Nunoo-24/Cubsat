"""Source-level checks for firmware behavior that needs a target toolchain.

These checks guard the documented C control-path contract; they do not compile
or execute ESP-IDF firmware.
"""

from pathlib import Path
import ast
import re
import unittest


FIRMWARE_MAIN = (
    Path(__file__).resolve().parents[1]
    / "CubeSat_ESP32"
    / "firmware"
    / "main"
    / "cubesat_main.c"
)
FIRMWARE_CONTROLLER = FIRMWARE_MAIN.with_name("yaw_controller.c")
SIL_REFERENCE = FIRMWARE_MAIN.parents[2] / "sil.py"
PYTHON_SIM = FIRMWARE_MAIN.parents[2].parent


class FirmwareSourceContractTests(unittest.TestCase):
    def test_camera_and_imu_age_guard_fails_closed(self):
        source = FIRMWARE_MAIN.read_text(encoding="utf-8")
        self.assertRegex(
            source,
            r"const float observation_age = camera_age_valid && imu_age_valid"
            r"\s*\? fmaxf\(camera_age, imu_age\) : INFINITY;",
        )
        self.assertIn("sample fails closed rather than silently bypassing", source)
        self.assertIn("Camera unavailable; active control will fail closed", source)

    def test_firmware_and_sil_require_an_exact_numeric_protocol_version(self):
        firmware = FIRMWARE_MAIN.read_text(encoding="utf-8")
        sil = SIL_REFERENCE.read_text(encoding="utf-8")
        self.assertIn("version->valueint == 1", firmware)
        self.assertIn("version->valuedouble == 1.0", firmware)
        self.assertIn("isinstance(protocol_version, bool)", sil)
        self.assertIn("protocol_version != 1", sil)

    def test_firmware_and_sil_command_contracts_stay_aligned(self):
        firmware = FIRMWARE_MAIN.read_text(encoding="utf-8")
        sil = SIL_REFERENCE.read_text(encoding="utf-8")

        for mode in ("SAFE", "STANDBY", "RATE_DAMP", "ACQUIRE", "TRACK"):
            with self.subTest(mode=mode):
                self.assertIn(f'"{mode}"', firmware)
                self.assertIn(f'"{mode}"', sil)

        for reason in (
            "INVALID_MODE",
            "SPEED_TEST_ACTIVE",
            "SPEED_TEST_REQUIRES_SAFE_OR_STANDBY",
            "INVALID_SPEED_TEST_ARGUMENTS",
            "YAW_STEP_ACTIVE",
            "YAW_STEP_REQUIRES_SAFE_OR_STANDBY",
            "INVALID_YAW_STEP_ARGUMENTS",
            "ATTITUDE_ESTIMATE_UNAVAILABLE",
        ):
            with self.subTest(reason=reason):
                self.assertIn(f'"{reason}"', firmware)
                self.assertIn(f'"{reason}"', sil)

        self.assertIn("cJSON_GetArraySize(arguments) == 2", firmware)
        self.assertRegex(
            sil,
            r"set\(arguments\)\s*==\s*\{\s*\"target_rev_s\",\s*"
            r"\"duration_s\"\s*\}",
        )
        self.assertIn("cJSON_GetArraySize(arguments) != 1", firmware)
        self.assertRegex(sil, r"set\(arguments\)\s*==\s*\{\"delta_deg\"\}")

        controller = FIRMWARE_CONTROLLER.read_text(encoding="utf-8")
        self.assertIn("cube_control_mode_t mode = CUBE_CONTROL_RATE_DAMP;",
                      controller)
        self.assertIn(".kd_lost_target_nm_per_rad_s = 0.06f", controller)
        self.assertIn("config->kd_lost_target_nm_per_rad_s", controller)
        controller_header = (FIRMWARE_MAIN.parent / "yaw_controller.h").read_text(
            encoding="utf-8")
        self.assertIn("kd_lost_target_nm_per_rad_s", controller_header)
        self.assertIn("if (target_valid) {", controller)
        self.assertIn("fault_output(CUBE_CONTROL_STALE_OBSERVATION)",
                      controller)

    def test_firmware_and_sil_telemetry_fields_are_logged(self):
        sil_text = SIL_REFERENCE.read_text(encoding="utf-8")
        sil_tree = ast.parse(sil_text)
        sil_fields = set()
        for node in ast.walk(sil_tree):
            if (isinstance(node, ast.Assign) and
                    any(isinstance(target, ast.Name) and target.id == "result"
                        for target in node.targets) and
                    isinstance(node.value, ast.Dict)):
                keys = {
                    key.value for key in node.value.keys
                    if isinstance(key, ast.Constant) and
                    isinstance(key.value, str)
                }
                if {"protocol_version", "message_type", "simulation_time_s"} <= keys:
                    sil_fields = keys
        sil_fields.update(re.findall(r'result\["([^"]+)"\]\s*=', sil_text))

        firmware = FIRMWARE_MAIN.read_text(encoding="utf-8")
        packet_start = firmware.index("cJSON *packet = cJSON_CreateObject();")
        packet_end = firmware.index("char *serialized = cJSON_PrintUnformatted(packet);")
        telemetry_source = firmware[packet_start:packet_end]
        firmware_fields = set(re.findall(
            r'cJSON_Add(?:Number|String|Bool|Null)ToObject\(packet,\s*"([^"]+)"',
            telemetry_source,
        ))
        if 'cJSON_AddItemToObject(packet, "command_ack"' in telemetry_source:
            firmware_fields.add("command_ack")

        ground_station = ast.parse(
            (PYTHON_SIM / "Ground_Station" / "main.py").read_text(encoding="utf-8")
        )
        log_fields = set()
        for node in ast.walk(ground_station):
            if (isinstance(node, ast.Assign) and
                    any(isinstance(target, ast.Name) and target.id == "LOG_FIELDS"
                        for target in node.targets) and
                    isinstance(node.value, (ast.List, ast.Tuple))):
                log_fields = {
                    item.value for item in node.value.elts
                    if isinstance(item, ast.Constant) and
                    isinstance(item.value, str)
                }

        envelope_fields = {"protocol_version", "source", "message_type"}
        self.assertTrue(sil_fields, "SIL telemetry result schema was not found")
        self.assertTrue(log_fields, "Ground Station CSV schema was not found")
        self.assertLessEqual((sil_fields | firmware_fields) - envelope_fields,
                             log_fields)


if __name__ == "__main__":
    unittest.main()
