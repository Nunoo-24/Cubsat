# AE project

Follow `../README.md` and read `Context/HANDOFF.md` before continuing work.

## Current workspace

- `Source/` contains `0-Guiao.pdf` and `1-project_guide.pdf`.
- `inputs/` contains `Dicas.txt` and `sumury.txt` (preserved under their supplied names).
- `outputs/Python_sim/` is the software project root, with `Sim_Environment/`,
  `Ground_Station/`, `CubeSat_ESP32/`, and shared `docs/`.
- `references/` is available for supporting material.

## Current deliverables

- `outputs/Design_Description.md` — initial system architecture and design draft.
- `outputs/Subsystems/` — per-subsystem purpose, interfaces, implementation links, limits, and next verification actions.
- `outputs/Python_sim/docs/TMTC-ICD.md` — Level 0 loopback protocol definition, with requirements for later onboard interface revisions.
- `outputs/Python_sim/docs/System_Architecture_and_Interfaces.md` — subsystem roles, signal chain, model assumptions, camera adapter, and emulation research.
- `outputs/Python_sim/README.md` — component run/build commands and verification scope.
- `outputs/Verification/Simulation_Results.md` — measured Python model results
  for the guide's acquisition, hold, nudge, moving-target, camera-occlusion, yaw
  estimator and camera projection across ideal tilted poses, a nominal-FOV
  detector geometry sweep, 32-corner plant/gain and 81-case sensor-noise/
  latency sensitivity screens,
  45° step scenarios, and qualitative passive roll/pitch free-decay results,
  plus earlier motor-speed limits. These are software results, not firmware
  or hardware verification.
- `outputs/Verification/Camera_Geometry_Results.md` — synthetic target-size,
  range, horizontal/vertical bearing sweep plus five combined body poses through rendered camera frames and the onboard detector.
- `outputs/Verification/Camera_Detector_Timing_Results.md` — reproducible
  host-only QVGA detector timing on ordinary and dense synthetic frames.
- `outputs/Verification/Axis_Control_Results.md` — host-SIL stabilization and
  camera-pointing checks for yaw, pitch, and roll, with point-only fallback
  and firmware/emulation limits recorded.
- `outputs/Verification/Requirements_Traceability.md` — requirement-by-requirement
  implementation status, evidence, and remaining verification gates.
- `outputs/reports/AE_Project_Status_and_Run_Guide.md` — full status report with
  current run commands, completed work, verification limits, and remaining tasks.
- `outputs/reports/AE_Project_Current_State_Report_2026-10-17.md` — fresh
  resume report recording the latest full verification run, current evidence,
  open gates, and context-reading limits.
- `references/Hardware_and_Emulation_Notes.md` — initial manufacturer/emulator research and source links.
- `Context/AI_USAGE_LOG.md` — record of AI assistance for project work.

## Objective

Build and demonstrate a CubeSat-style autonomous attitude-control system. The
physical demonstrator currently specifies active yaw control with a reaction
wheel. The host simulation demonstrates camera pointing across yaw, pitch, and roll
using image position plus an optional asymmetric target marker; featureless
points use gravity-based roll fallback. The ESP32/Wokwi source has the same
virtual-axis path for simulator-framed images, while physical motor actuation
remains yaw-only. The marker toggle is disabled by default to preserve the
supplied point-target setup.
The XIAO ESP32-S3 performs onboard camera/image
processing, sensor processing, control, and reaction-wheel commands. The PC
ground station supervises and records telemetry but is not in the autonomous
control loop. Whether early hardware tests lock roll/pitch remains to be
confirmed. See `Context/PROJECT_BRIEF.md` for performance goals and open
implementation details.

Put human-facing deliverables in `outputs/` and update the handoff after meaningful work.

## Autonomous work loop

For each implementation cycle, read this file, `Context/HANDOFF.md`, and the
project prompt before changing files. Inspect the existing simulation and
source assets first. Research each affected subsystem, hardware item, and
interface from authoritative sources; record citations and distinguish facts
from project assumptions. Organize human-facing subsystem descriptions and
artifacts under `outputs/`, then implement each component in its intended
language and protocol. Integrate the components, keep individual run/build
commands in the project and software READMEs, and run the available checks.
Compare actual results with the supplied sources, inputs, and recorded design
decisions. Fix issues and repeat the checks until the implementation is
consistent or a remaining step requires unavailable hardware, tools, or a
project decision. At each meaningful checkpoint, update the subsystem notes,
AI-use log, and `Context/HANDOFF.md` with changes, evidence, limits, and the
next concrete action. Do not describe unavailable or blocked verification as
passed.

## Quick start

For the current second test drive and Windows click-to-run launchers, see
[`outputs/Python_sim/SECOND_TEST_DRIVE.md`](outputs/Python_sim/SECOND_TEST_DRIVE.md).
The three launchers start the environment, host CubeSat SIL, and Ground Station
in separate WSL terminal windows. From Windows, run
`outputs/Python_sim/Launchers/Install_Desktop_Shortcuts.bat` once to create
three desktop icons. They are script launchers, not compiled `.exe` files.
The environment and Ground Station launchers wait for the matching page to
respond through Windows localhost before opening a browser; a 45-second
timeout leaves the error visible for diagnosis.

Run the integrated simulation from WSL by starting each process in its own
terminal, in this order (plant, host SIL, Ground Station):

```bash
cd outputs/Python_sim
python3 -m Sim_Environment.main
```

```bash
cd outputs/Python_sim
python3 -m CubeSat_ESP32.sil
```

```bash
cd outputs/Python_sim
python3 -m Ground_Station.main
```

Open `http://127.0.0.1:8000` for the environment and
`http://127.0.0.1:8001` for the Ground Station. Run model, bridge and control
checks and regenerate the software reports with:

```bash
./run_verification.sh
```

The script also runs the portable C and ESP-IDF checks when their toolchains are
available, and reports explicit skips otherwise. It does not exercise live
sockets, browser rendering, Wokwi, or physical hardware. For a focused Python
unit-test run, use:

```bash
python3 -m unittest Sim_Environment.test_model Sim_Environment.test_environment Sim_Environment.test_camera_transport CubeSat_ESP32.test_attitude CubeSat_ESP32.test_sil_runtime CubeSat_ESP32.test_control CubeSat_ESP32.test_camera CubeSat_ESP32.test_moteus_protocol Ground_Station.test_main Verification.test_evaluate_scenarios Verification.test_speed_test_integration Verification.test_camera_transport_evaluation Verification.test_camera_image_noise Verification.test_camera_geometry Verification.test_camera_detector_timing Verification.test_ui_contracts Verification.test_emulation_config Verification.test_firmware_source_contracts Verification.test_axis_control Verification.test_http_startup
```

The legacy Level 0 Ground Station and heartbeat simulator commands, ESP-IDF
build commands, and current verification limits are in
[`outputs/Python_sim/README.md`](outputs/Python_sim/README.md).
