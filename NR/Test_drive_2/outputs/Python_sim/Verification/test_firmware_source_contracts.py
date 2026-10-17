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
    def test_c_yaw_defaults_and_tracking_modes_match_python_reference(self):
        firmware = FIRMWARE_CONTROLLER.read_text(encoding="utf-8")
        default_block = re.search(
            r"cube_yaw_controller_default_config\(void\).*?"
            r"return \(cube_yaw_controller_config_t\)\{(?P<body>.*?)\};",
            firmware,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(default_block)
        c_defaults = {
            name: float(value)
            for name, value in re.findall(
                r"\.(\w+)\s*=\s*([0-9.]+)f", default_block.group("body")
            )
        }

        python_source = (FIRMWARE_MAIN.parents[2] / "control.py").read_text(
            encoding="utf-8")
        python_tree = ast.parse(python_source)
        controller_class = next(
            node for node in python_tree.body
            if isinstance(node, ast.ClassDef) and node.name == "YawController"
        )
        python_defaults = {
            node.target.id: node.value.value
            for node in controller_class.body
            if isinstance(node, ast.AnnAssign) and
            isinstance(node.target, ast.Name) and
            isinstance(node.value, ast.Constant) and
            isinstance(node.value.value, (int, float))
        }
        self.assertEqual(c_defaults, python_defaults)

        normalized_c = re.sub(r"\s+", "", firmware)
        self.assertIn(
            "constfloatkd_rate=target_valid?config->kd_nm_per_rad_s:"
            "config->kd_lost_target_nm_per_rad_s;",
            normalized_c,
        )
        self.assertIn("floatcommand_nm=-kd_rate*yaw_rate_rad_s;", normalized_c)
        self.assertIn("command_nm+=config->kp_nm_per_rad*target_error_rad;",
                      normalized_c)
        self.assertIn(
            "command=(self.kp_nm_per_rad*target_error_rad"
            "-self.kd_nm_per_rad_s*yaw_rate_rad_s)",
            re.sub(r"\s+", "", python_source),
        )
        self.assertIn(
            "command=-self.kd_lost_target_nm_per_rad_s*yaw_rate_rad_s",
            re.sub(r"\s+", "", python_source),
        )

    def test_c_pitch_roll_gains_and_equations_match_python_reference(self):
        firmware = (FIRMWARE_MAIN.parent / "pitch_roll_controller.c").read_text(
            encoding="utf-8")
        default_block = re.search(
            r"cube_pitch_roll_controller_default_config\(void\).*?"
            r"return \(cube_pitch_roll_controller_config_t\)\{(?P<body>.*?)\};",
            firmware,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(default_block)
        c_defaults = {
            name: float(value)
            for name, value in re.findall(
                r"\.(\w+)\s*=\s*([0-9.]+)f", default_block.group("body")
            )
        }

        python_source = (FIRMWARE_MAIN.parents[2] / "control.py").read_text(
            encoding="utf-8")
        python_tree = ast.parse(python_source)
        controller_class = next(
            node for node in python_tree.body
            if isinstance(node, ast.ClassDef) and node.name == "PitchRollController"
        )
        python_defaults = {
            node.target.id: node.value.value
            for node in controller_class.body
            if isinstance(node, ast.AnnAssign) and
            isinstance(node.target, ast.Name) and
            isinstance(node.value, ast.Constant) and
            isinstance(node.value.value, (int, float))
        }
        self.assertEqual(c_defaults, python_defaults)

        for axis in ("pitch", "roll"):
            c_equation = re.search(
                rf"const float {axis}_torque =(?P<expr>.*?);",
                firmware,
                flags=re.DOTALL,
            )
            py_equation = re.search(
                rf"{axis}_torque = \((?P<expr>.*?)\)",
                python_source,
                flags=re.DOTALL,
            )
            self.assertIsNotNone(c_equation)
            self.assertIsNotNone(py_equation)
            normalized_c = re.sub(r"\s+", "", c_equation.group("expr"))
            normalized_c = normalized_c.replace("config->", "self.")
            normalized_python = re.sub(r"\s+", "", py_equation.group("expr"))
            self.assertEqual(normalized_c, normalized_python)

    def test_c_detector_declarations_and_callers_use_length_argument(self):
        firmware_root = FIRMWARE_MAIN.parents[1]
        occurrences = 0
        for path in firmware_root.rglob("*"):
            if path.suffix not in {".c", ".h"}:
                continue
            source = path.read_text(encoding="utf-8")
            for match in re.finditer(r"\bcube_detect_black_target\s*\(", source):
                open_paren = source.find("(", match.start())
                depth = 0
                commas = 0
                in_string = False
                escaped = False
                for character in source[open_paren:]:
                    if in_string:
                        if escaped:
                            escaped = False
                        elif character == "\\":
                            escaped = True
                        elif character == '"':
                            in_string = False
                        continue
                    if character == '"':
                        in_string = True
                    elif character == "(":
                        depth += 1
                    elif character == ")":
                        depth -= 1
                        if depth == 0:
                            break
                    elif character == "," and depth == 1:
                        commas += 1
                self.assertEqual(depth, 0, f"unclosed detector call in {path}")
                self.assertEqual(
                    commas + 1,
                    7,
                    f"detector declaration/call must use seven arguments: {path}",
                )
                occurrences += 1
        self.assertGreater(occurrences, 0, "detector API occurrences were not found")

    def test_camera_and_imu_age_guard_fails_closed(self):
        source = FIRMWARE_MAIN.read_text(encoding="utf-8")
        self.assertRegex(
            source,
            r"const float observation_age = camera_age_valid && imu_age_valid"
            r"\s*\? fmaxf\(camera_age, imu_age\) : INFINITY;",
        )
        self.assertIn("sample fails closed rather than silently bypassing", source)
        self.assertIn("Camera unavailable; active control will fail closed", source)
        self.assertIn("camera_roll_orientation_valid", source)
        self.assertIn("cube_pitch_roll_controller_update(", source)
        self.assertIn('strcmp(camera_frame_source, "simulated") == 0', source)
        pitch_bearing = re.search(
            r"const float camera_error_y_rad =.*?\? (?P<expr>.*?) : 0\.0f;",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(pitch_bearing)
        self.assertEqual(
            re.sub(r"\s+", "", pitch_bearing.group("expr")),
            "atanf(target_error_y_px/focal_length_px)",
            "the C detector reports positive image-down offsets, so a target "
            "below center must yield positive pitch error like host SIL",
        )
        normalized_firmware = re.sub(r"\s+", "", source)
        self.assertIn(
            "atan2f(-accel_z,hypotf(accel_x,accel_y))",
            normalized_firmware,
        )
        self.assertIn("atan2f(accel_x,accel_y)", normalized_firmware)
        sil_source = re.sub(r"\s+", "", SIL_REFERENCE.read_text(encoding="utf-8"))
        self.assertIn(
            'math.atan2(-accel_values["z"],math.hypot(accel_values["x"],'
            'accel_values["y"]))',
            sil_source,
        )
        self.assertIn(
            'math.atan2(accel_values["x"],accel_values["y"])',
            sil_source,
        )
        self.assertIn("0.8f*9.80665f", normalized_firmware)
        self.assertIn("1.2f*9.80665f", normalized_firmware)
        self.assertIn("0.8*9.80665<=accel_norm<=1.2*9.80665", sil_source)
        roll_reference = re.search(
            r"const float roll_control_estimate_rad =(?P<expr>.*?);",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(roll_reference)
        normalized_roll_reference = re.sub(r"\s+", "", roll_reference.group("expr"))
        self.assertEqual(
            normalized_roll_reference,
            "camera_roll_estimate_valid?camera_roll_estimate_rad:imu_roll_estimate_rad",
        )
        tilt_gate = re.search(
            r"const bool tilt_control_allowed =(?P<expr>.*?);",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(tilt_gate)
        normalized_tilt_gate = re.sub(r"\s+", "", tilt_gate.group("expr"))
        self.assertIn('strcmp(camera_frame_source,"simulated")==0',
                      normalized_tilt_gate)
        self.assertIn("imu_valid&&camera_age_valid&&observation_age<=0.1f",
                      normalized_tilt_gate)
        self.assertIn("output.mode!=CUBE_CONTROL_FAULT", normalized_tilt_gate)
        detector_source = (FIRMWARE_MAIN.parent / "black_target.c").read_text(
            encoding="utf-8")
        self.assertIn("expected_pixels > CUBE_RLE1_MAX_PIXELS",
                      detector_source)
        self.assertIn("stride > CUBE_TARGET_MAX_IMAGE_WIDTH",
                      detector_source)
        self.assertIn("gray_length < stride * height", detector_source)
        self.assertIn("simulated_camera_frame, sizeof(simulated_camera_frame)",
                      source)
        camera_adapter = (FIRMWARE_MAIN.parent / "xiao_camera.c").read_text(
            encoding="utf-8")
        self.assertLess(camera_adapter.index('#include "sdkconfig.h"'),
                        camera_adapter.index("#if CONFIG_CAMERA_AF_SUPPORT"))
        self.assertIn("frame->buf, frame->len", camera_adapter)
        self.assertIn("sensor->id.PID == OV5640_PID", camera_adapter)
        self.assertIn("sensor->id.PID == OV3660_PID", camera_adapter)
        self.assertIn('ESP_LOGE(TAG, "Unsupported camera sensor PID',
                      camera_adapter)
        self.assertIn("esp_camera_af_is_supported(sensor)", camera_adapter)
        self.assertIn("esp_camera_af_init(sensor, &autofocus)", camera_adapter)
        self.assertIn("esp_camera_af_wait(sensor, autofocus.timeout_ms, &af_status)",
                      camera_adapter)
        self.assertIn("!af_status.focused", camera_adapter)
        self.assertIn("esp_camera_deinit();", camera_adapter)
        main_cmake = (FIRMWARE_MAIN.parent / "CMakeLists.txt").read_text(
            encoding="utf-8")
        self.assertIn("list(APPEND cubesat_requirements esp32-camera)",
                      main_cmake)
        axis_controller = (FIRMWARE_MAIN.parent / "pitch_roll_controller.c").read_text(
            encoding="utf-8")
        self.assertIn("max_pitch_torque_nm = 0.02f", axis_controller)
        self.assertIn("max_roll_torque_nm = 0.035f", axis_controller)
        self.assertIn("observation_age_s > config->max_observation_age_s",
                      axis_controller)

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

    def test_c_and_sil_yaw_step_boundaries_and_terminal_precedence_match(self):
        firmware = FIRMWARE_MAIN.read_text(encoding="utf-8")
        sil = SIL_REFERENCE.read_text(encoding="utf-8")

        c_start = firmware.index("bool yaw_step_active =")
        c_timeout = firmware.index(
            "if (yaw_step_active && cycle_now_us >= yaw_step_deadline_us)",
            c_start,
        )
        c_invalid = firmware.index(
            "if (yaw_step_active && !attitude_valid)", c_start
        )
        self.assertLess(c_invalid, c_timeout)
        self.assertIn("yaw_step_deadline_us = now_us + 10000000", firmware)
        self.assertIn("fabsf(yaw_step_error_rad) <= 0.01745329252f", firmware)
        self.assertIn("fabsf(yaw_rate) <= 0.00872664626f", firmware)
        self.assertIn("cycle_now_us - yaw_step_settle_started_us >= 1000000",
                      firmware)

        py_start = sil.index("yaw_step_elapsed_s = None")
        py_end = sil.index('if self.mode in ("SAFE", "STANDBY"', py_start)
        py_step = sil[py_start:py_end]
        py_invalid = py_step.index("if not attitude_valid:")
        py_timeout = py_step.index("elif now >= self.yaw_step_deadline:")
        self.assertLess(py_invalid, py_timeout)
        self.assertIn("self.yaw_step_deadline = now + 10.0", sil)
        self.assertIn("abs(yaw_step_error_rad) <= math.radians(1.0)", py_step)
        self.assertIn("abs(rate) <= math.radians(0.5)", py_step)
        self.assertIn("now - self.yaw_step_settle_since >= 1.0", py_step)
        for terminal_status in ("ATTITUDE_INVALID", "TIMEOUT", "COMPLETE"):
            with self.subTest(status=terminal_status):
                self.assertIn(f'"{terminal_status}"', firmware)
                self.assertIn(f'"{terminal_status}"', sil)

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

    def test_esp32_virtual_axis_telemetry_matches_simulator_bridge_contract(self):
        firmware = FIRMWARE_MAIN.read_text(encoding="utf-8")
        self.assertIn(
            'cJSON_AddStringToObject(packet, "source", "cubesat_esp32");',
            firmware,
        )
        self.assertIn(
            'cJSON_AddStringToObject(packet, "camera_frame_source", '
            'camera_frame_source);',
            firmware,
        )
        virtual_block = re.search(
            r'if \(strcmp\(camera_frame_source, "simulated"\) == 0\) '
            r'\{(?P<body>.*?)cJSON_AddItemToObject\(packet, '
            r'"virtual_axis_torque_nm", axis_torque\);',
            firmware,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(virtual_block)
        self.assertIn('cJSON_AddNumberToObject(axis_torque, "pitch"',
                      virtual_block.group("body"))
        self.assertIn('cJSON_AddNumberToObject(axis_torque, "roll"',
                      virtual_block.group("body"))

    def test_firmware_magnetometer_telemetry_includes_configured_full_scale(self):
        firmware = FIRMWARE_MAIN.read_text(encoding="utf-8")
        driver = (FIRMWARE_MAIN.parent / "lis3mdl.c").read_text(
            encoding="utf-8")
        header = (FIRMWARE_MAIN.parent / "lis3mdl.h").read_text(
            encoding="utf-8")
        self.assertIn("uint8_t cube_lis3mdl_full_scale_gauss(void);", header)
        self.assertIn("return (uint8_t)LIS3MDL_RANGE_G;", driver)
        self.assertIn('cJSON_AddNumberToObject(packet, "mag_full_scale_gauss",',
                      firmware)
        self.assertIn("cube_lis3mdl_full_scale_gauss()", firmware)
        self.assertIn('cJSON_AddNullToObject(packet, "mag_full_scale_gauss");',
                      firmware)
        ground_station = (PYTHON_SIM / "Ground_Station" / "main.py").read_text(
            encoding="utf-8")
        self.assertIn('"mag_full_scale_gauss"', ground_station)

        bridge = (PYTHON_SIM / "Sim_Environment" / "main.py").read_text(
            encoding="utf-8")
        self.assertIn(
            'payload.get("source") == "cubesat_esp32" and '
            'payload.get("camera_frame_source") == "simulated"',
            re.sub(r"\s+", " ", bridge),
        )
        normalized_bridge = re.sub(r"\s+", "", bridge)
        self.assertIn(
            "body_pitch_torque_nm=self.virtual_pitch_torque_nm,"
            "body_roll_torque_nm=self.virtual_roll_torque_nm",
            normalized_bridge,
        )


if __name__ == "__main__":
    unittest.main()
