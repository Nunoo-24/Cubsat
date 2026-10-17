# AE CubeSat software workspace

This repository contains the three independently runnable project components
and their shared interface documents. The host software-in-the-loop (SIL) path
connects the browser plant, onboard control reference, and Ground Station. The
ESP32-S3 ESP-IDF application also has an HTTP bridge client for Wokwi, but its
target build and emulator run still require verification in an ESP-IDF/Wokwi
environment.

Archived, self-contained snapshots for the two test drives are in
[`NR/README.md`](NR/README.md). Each snapshot contains its own project inputs,
source files, outputs, and context handoff.

## Components

- [`Sim_Environment/`](Sim_Environment/README.md) — local browser scene and
  three-axis plant with yaw-wheel control, host-SIL/ESP32 simulator virtual
  pitch/roll torque, and an optional asymmetric camera-roll marker. Start with
  `python3 -m Sim_Environment.main` and open
  `http://127.0.0.1:8000`.
- [`Ground_Station/`](Ground_Station/README.md) — PC operator UI, supervisory
  mode/PING, bounded wheel-speed commissioning, relative yaw-step commands,
  target-error vs simulation-time graph, and CSV logging; unavailable target
  intervals and samples beyond the 100 ms camera/IMU freshness limit appear as
  gaps rather than connected measurements. The UI
  distinguishes bridge acceptance from the later CubeSat acknowledgement
  carried in telemetry.
- [`CubeSat_ESP32/`](CubeSat_ESP32/README.md) — ESP-IDF firmware for the
  XIAO ESP32-S3, yaw attitude estimator, portable controller and target detector, optional Sense camera
  and shared-I²C MPU-6050/LIS3MDL adapters, Wokwi profile, and host SIL runner.
- [`docs/System_Architecture_and_Interfaces.md`](docs/System_Architecture_and_Interfaces.md)
  — component responsibilities, signal chain, assumptions, and emulation choice.
- [`docs/TMTC-ICD.md`](docs/TMTC-ICD.md) — current Level 0 interface and planned
  onboard packet requirements.
- [`docs/Simulator_HTTP_API.md`](docs/Simulator_HTTP_API.md) — implemented
  local scene-control API, separate from the device TM/TC protocol.
- Model-level scenario measurements are recorded in
  [`../Verification/Simulation_Results.md`](../Verification/Simulation_Results.md)
  and can be regenerated with `python3 -m Verification.evaluate_scenarios`.
- Three-axis stabilization and camera-pointing evidence is recorded in
  [`../Verification/Axis_Control_Results.md`](../Verification/Axis_Control_Results.md).

## Run commands

### Current test drive

Follow [`SECOND_TEST_DRIVE.md`](SECOND_TEST_DRIVE.md) for the current test
sequence, expected observations, and Windows click launchers. The historical
first-drive setup and connection troubleshooting remain in
[`FIRST_TEST_DRIVE.md`](FIRST_TEST_DRIVE.md). Desktop shortcuts can be created by
double-clicking `Launchers/Install_Desktop_Shortcuts.bat` from Windows.
The environment and Ground Station launchers wait for their matching page to
return HTTP 200 through Windows localhost before opening the browser. If the
45-second readiness check fails, the launcher reports the port and keeps the
diagnostic console visible; the WSL service terminal stays open separately.

### Run offline verification and regenerate reports

From this directory, run:

```bash
./run_verification.sh
```

The script runs full Python unittest discovery and syntax compilation, then
regenerates the model, camera transport, geometry, image-noise, detector timing,
and three-axis camera/stabilization reports under `../Verification/`. It runs
portable C tests when a C11 compiler is on `PATH`, and builds the ESP-IDF
application when `idf.py` is available; otherwise it prints an explicit skip.
Live sockets, browser rendering, Wokwi, and hardware still need separate
runtime checks.

If a component reports `Operation not permitted` while opening its local HTTP
port, that shell does not have permission to create listening sockets. Run the
service from a normal local WSL/Linux session. If it reports that the address
is already in use, stop the other process using that port before restarting.

Run the integrated software-in-the-loop project in three terminals from this
directory. Start the plant first:

```bash
python3 -m Sim_Environment.main
```

Then start the onboard software reference:

```bash
python3 -m CubeSat_ESP32.sil
```

Then start the Ground Station:

```bash
python3 -m Ground_Station.main
```

Open `http://127.0.0.1:8001`. The plant UI is at `http://127.0.0.1:8000`.
The Ground Station appends telemetry to
`Ground_Station/logs/telemetry.csv`. To keep outputs clean, that run log is
ignored by Git.

The former PyQt / UDP heartbeat demo remains available in
`Ground_Station/level0_pyqt.py` and `Ground_Station/simulator.py` as a separate
Level 0 example; PyQt5 is needed only for that legacy demo.

Run the plant, environment and controller checks:

```bash
python3 -m unittest Sim_Environment.test_model Sim_Environment.test_environment Sim_Environment.test_camera_transport CubeSat_ESP32.test_attitude CubeSat_ESP32.test_sil_runtime CubeSat_ESP32.test_control CubeSat_ESP32.test_camera CubeSat_ESP32.test_moteus_protocol Ground_Station.test_main Verification.test_evaluate_scenarios Verification.test_speed_test_integration Verification.test_camera_transport_evaluation Verification.test_camera_image_noise Verification.test_camera_geometry Verification.test_camera_detector_timing Verification.test_ui_contracts Verification.test_emulation_config Verification.test_firmware_source_contracts Verification.test_axis_control Verification.test_http_startup
```

This suite includes in-memory HTTP-handler checks for environment and Ground
Station requests, the RLE1 bridge → SIL decoder/detector → telemetry-matched
preview path, an environment → SIL → environment actuator cycle, an end-to-end relative
yaw-step command/acknowledgement/settling cycle, and a visual acquisition loop
that fetches a sequence-matched RLE1 image, decodes it through the SIL
detector/controller path, and returns telemetry through the Ground Station
handler. All integration runs use
in-memory HTTP handlers and the simulated plant; they do not prove live TCP
networking. Run the three processes above separately
in a socket-enabled environment to verify local networking.

Run all four portable C host test suites (yaw, pitch/roll, attitude estimation,
and camera target detection) with
`./CubeSat_ESP32/firmware/run_host_tests.sh`; GCC, Clang, or a compatible C11
compiler is required and is not installed in this WSL image. The script builds
in a temporary directory and cleans up its binaries. See
[`CubeSat_ESP32/firmware/README.md`](CubeSat_ESP32/firmware/README.md).

Build the ESP-IDF 5.3+ project from `CubeSat_ESP32/firmware/` in a configured
ESP-IDF terminal. WSL/Ubuntu installation commands, the mounted Windows
ESP-IDF option, portable C-test instructions, and Wokwi setup are documented in
[`CubeSat_ESP32/firmware/README.md`](CubeSat_ESP32/firmware/README.md).

```bash
cd /home/nunot/Mestrado_Codex/AE/outputs/Python_sim/CubeSat_ESP32/firmware
idf.py set-target esp32s3
idf.py build
```

The Wokwi configuration is `CubeSat_ESP32/firmware/wokwi.toml`; it loads the
ESP-IDF flash images from `build/flasher_args.json` and the `cube_firmware.elf`
application. After building, start it from the firmware directory in the
Wokwi VS Code extension. With ESP-IDF 6.0+ and the `idf-wokwi` extension
installed, the alternate command is:

```bash
idf.py wokwi
```

Linux `idf.py`, Wokwi CLI, and Wokwi VS Code integration are not available in
this WSL session.
A Windows ESP-IDF 6.1 installation and ESP32-S3 compiler exist on the mounted
C: drive, but WSL cannot launch the Windows compiler here because interop
fails while opening a socket (`UtilBindVsockAnyPort`). The old cache under
`firmware/build/` points to the former Windows checkout and predates the
current CubeSat firmware, so it is not a build result for these sources. Build
the current `CubeSat_ESP32/firmware/` project from a functioning ESP-IDF 6.1
environment before treating firmware compilation as verified.

## Important distinction

The host SIL passes simulated sensors and commands over local HTTP. Wokwi
firmware uses the same local simulation bridge via the Wokwi private gateway.
Neither validates actual camera performance, IMU calibration, CAN-FD/moteus
behavior, real wheel dynamics, or physical system performance. The host plant
generates a sequence-paired synthetic QVGA image; host SIL and the default
Wokwi firmware profile fetch it through bounded lossless RLE1, reconstruct the
full raster, and run their respective dark-pixel coherence filters and centroid detectors on those pixels. The
raw QVGA endpoint remains available for inspection.
The physical Sense-camera profile processes a local camera frame. The host plant has a
first-pass reaction-wheel model whose parameters remain provisional.
