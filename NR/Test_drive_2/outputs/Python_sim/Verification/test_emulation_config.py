"""Static checks for the checked-in ESP-IDF/Wokwi project configuration.

These checks verify path and target consistency; they do not build or boot the
firmware in Wokwi.
"""

import json
from pathlib import Path
import re
import unittest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
FIRMWARE_ROOT = PROJECT_ROOT / "CubeSat_ESP32" / "firmware"


class EspIdfWokwiConfigurationTests(unittest.TestCase):
    def test_wokwi_files_match_esp_idf_target_and_project_name(self):
        wokwi_text = (FIRMWARE_ROOT / "wokwi.toml").read_text(encoding="utf-8")
        config = dict(re.findall(
            r"^(version|firmware|elf)\s*=\s*\"([^\"]+)\"?\s*$",
            wokwi_text, flags=re.MULTILINE,
        ))
        # `version = 1` is an unquoted integer, so read it separately.
        version = re.search(r"^version\s*=\s*(\d+)\s*$", wokwi_text,
                            flags=re.MULTILINE)
        self.assertIsNotNone(version)
        self.assertEqual(int(version.group(1)), 1)
        self.assertEqual(config.get("firmware"), "build/flasher_args.json")
        self.assertEqual(config.get("elf"), "build/cube_firmware.elf")

        cmake = (FIRMWARE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        project = re.search(r"^project\(([^)]+)\)", cmake, re.MULTILINE)
        self.assertIsNotNone(project)
        self.assertEqual(Path(config["elf"]).name,
                         f"{project.group(1)}.elf")

        diagram = json.loads((FIRMWARE_ROOT / "diagram.json").read_text(
            encoding="utf-8"))
        board = [part for part in diagram["parts"]
                 if part.get("type") == "board-xiao-esp32-s3"]
        self.assertEqual(len(board), 1)
        self.assertEqual(board[0]["attrs"].get("psramType"), "octal")

    def test_wokwi_defaults_use_guest_wifi_gateway_and_50_hz_period(self):
        defaults_text = (FIRMWARE_ROOT / "sdkconfig.defaults").read_text(
            encoding="utf-8")
        defaults = dict(line.split("=", 1) for line in defaults_text.splitlines()
                        if line and not line.startswith("#"))
        self.assertEqual(defaults["CONFIG_IDF_TARGET"], '"esp32s3"')
        self.assertEqual(defaults["CONFIG_CUBESAT_WIFI_SSID"], '"Wokwi-GUEST"')
        self.assertEqual(defaults["CONFIG_CUBESAT_SIM_BRIDGE_URL"],
                         '"http://host.wokwi.internal:8000"')
        tick_hz = int(defaults["CONFIG_FREERTOS_HZ"])
        self.assertEqual(tick_hz % 50, 0)
        self.assertEqual(int(defaults["CONFIG_ESP_MAIN_TASK_STACK_SIZE"]), 8192)

        firmware_main = (FIRMWARE_ROOT / "main" / "cubesat_main.c").read_text(
            encoding="utf-8")
        self.assertIn("uxTaskGetStackHighWaterMark(NULL)", firmware_main)

        manifest = (FIRMWARE_ROOT / "main" / "idf_component.yml").read_text(
            encoding="utf-8")
        self.assertIn('idf: ">=5.3"', manifest)
        self.assertIn("espressif/esp32-camera", manifest)
        main_cmake = (FIRMWARE_ROOT / "main" / "CMakeLists.txt").read_text(
            encoding="utf-8")
        self.assertIn('list(APPEND cubesat_sources "xiao_camera.c")', main_cmake)


if __name__ == "__main__":
    unittest.main()
