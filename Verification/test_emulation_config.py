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
LAUNCHER_ROOT = PROJECT_ROOT / "Launchers"


class EspIdfWokwiConfigurationTests(unittest.TestCase):
    def test_lis3mdl_full_scale_options_match_register_and_sensitivity(self):
        kconfig = (FIRMWARE_ROOT / "Kconfig.projbuild").read_text(
            encoding="utf-8")
        driver = (FIRMWARE_ROOT / "main" / "lis3mdl.c").read_text(
            encoding="utf-8")
        self.assertIn("choice CUBESAT_LIS3MDL_FULL_SCALE", kconfig)
        self.assertIn("depends on CUBESAT_LIS3MDL_ENABLED", kconfig)
        self.assertIn("default CUBESAT_LIS3MDL_FS_4G", kconfig)
        for scale, register_value, sensitivity in (
            (4, "0x00u", "6842.0f"),
            (8, "0x20u", "3421.0f"),
            (12, "0x40u", "2281.0f"),
            (16, "0x60u", "1711.0f"),
        ):
            with self.subTest(full_scale_gauss=scale):
                symbol = f"CONFIG_CUBESAT_LIS3MDL_FS_{scale}G"
                self.assertIn(f"config CUBESAT_LIS3MDL_FS_{scale}G", kconfig)
                if scale != 4:
                    self.assertIn(f"#if {symbol}" if scale == 8 else
                                  f"#elif {symbol}", driver)
                self.assertIn(f"#define LIS3MDL_MICROTESLA_PER_LSB "
                              f"(100.0f / {sensitivity})", driver)
                self.assertIn(f"#define LIS3MDL_CTRL_REG2_FS {register_value}",
                              driver)
        self.assertIn("write_register(LIS3MDL_CTRL_REG2, LIS3MDL_CTRL_REG2_FS)",
                      driver)

    def test_windows_ui_launchers_wait_for_the_matching_service(self):
        wait_script = (LAUNCHER_ROOT / "Wait_For_Service.ps1").read_text(
            encoding="utf-8")
        self.assertIn("Invoke-WebRequest", wait_script)
        self.assertIn("[int]$response.StatusCode -eq 200", wait_script)
        self.assertIn("$response.Content.Contains($ExpectedText)", wait_script)
        self.assertIn("AddSeconds($TimeoutSeconds)", wait_script)
        self.assertIn("localhost forwarding", wait_script)

        launchers = (
            ("Start_01_Simulation_Environment.bat", 8000,
             "CubeSat attitude-control simulation", "http://127.0.0.1:8000"),
            ("Start_03_Ground_Station.bat", 8001,
             "CubeSat Ground Station", "http://127.0.0.1:8001"),
        )
        for filename, port, expected_text, url in launchers:
            with self.subTest(launcher=filename):
                source = (LAUNCHER_ROOT / filename).read_text(encoding="utf-8")
                self.assertIn("cmd.exe /k", source)
                self.assertIn("Wait_For_Service.ps1", source)
                self.assertIn(f"-Port {port}", source)
                self.assertIn(f'-ExpectedText "{expected_text}"', source)
                self.assertLess(source.index("Wait_For_Service.ps1"),
                                source.index(f'start "" "{url}'))
                self.assertNotIn("timeout /t 3", source.lower())

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
        attrs = board[0]["attrs"]
        # Wokwi's XIAO ESP32-S3 model documents 2/4/8/16/32 MB flash,
        # 2/4/8 MB PSRAM, and quad/octal PSRAM variants.
        self.assertIn(attrs.get("flashSize"), {"2", "4", "8", "16", "32"})
        self.assertIn(attrs.get("psramSize"), {"2", "4", "8"})
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
