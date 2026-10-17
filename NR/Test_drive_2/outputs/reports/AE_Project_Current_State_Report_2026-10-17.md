# AE CubeSat Project — Current State Report

**Prepared:** 17 October 2026
**Purpose:** Resume the existing project with a freshly checked status.
**Verification run:** `outputs/Python_sim/run_verification.sh`

## Executive summary

The project has a substantial Python host simulation and software-in-the-loop (SIL) implementation, a Ground Station, system/interface documentation, and an ESP-IDF firmware source tree. This fresh offline run passed all **218 Python tests**, Python syntax compilation, and all six simulation/camera/axis report generators. Portable C host tests and the ESP-IDF firmware build were explicitly skipped because neither a C compiler nor `idf.py` is available on PATH.

The current evidence is strongest for the host Python simulation. It does not establish live multi-process networking, browser operation, ESP32 firmware compilation, Wokwi execution, or physical hardware performance. The recorded estimate of approximately **65% simulation-work completion** is qualitative and should not be read as a measured requirements score.

## Project intent and current scope

The onboard Seeed XIAO ESP32-S3 is intended to process camera and inertial data, estimate pointing error, and command the reaction wheel without requiring the PC Ground Station in the autonomous loop. The Ground Station monitors, logs, and sends bounded supervisory/test commands.

Physical actuation remains scoped to yaw via a reaction wheel. Host SIL controls yaw and demonstrates virtual pitch/roll stabilization and camera pointing. ESP32 source routes virtual pitch/roll torques only for simulator-sourced frames; these are not physical actuators. Camera roll estimation needs an optional asymmetric fiducial; the default point target has no visual roll cue and falls back to IMU leveling.

The plant, sensor and camera parameters are provisional. The guide's performance goals are proposed project criteria pending confirmation with the team/lecturer.

## Fresh verification results

| Check | Result |
|---|---|
| Python unit tests | **218 passed**, 110.024 s |
| Python syntax compilation | Passed |
| Simulation and camera report generators | All six completed: simulation, transport, geometry, image noise, detector timing, and axis control |
| Portable C host tests | Skipped: no C11 compiler on PATH |
| ESP-IDF build | Skipped: `idf.py` not on PATH |
| Live sockets/browser, Wokwi, physical hardware | Not exercised by the offline runner |

The regenerated reports record the following nominal model results:

| Scenario | Result |
|---|---:|
| Acquire from 30° to ≤2° | 1.600 s |
| Hold within ±2° for 60 s | 0.107° peak error |
| Recover from a 20° nudge | 1.340 s |
| Track a 0.2°/s target for 60 s | 0.119° peak error |
| Recover after 2 s camera occlusion | 0.760 s |
| 45° yaw step | Settled dwell began at 5.820 s; 1 s dwell completed at 6.820 s |
| Bidirectional camera yaw/pitch pointing | Settled in 2.29–2.34 s (yaw) and 1.03 s (pitch) |
| Camera-marker roll correction | Settled from +15° in 8.32 s and −15° in 9.16 s |

These results are simulations against the project's current provisional model. A separate assumed noise/latency screen met the 60 s hold bound in 80/81 cases, with one marginal 2.034° miss. The timing and noise assumptions are not measured hardware properties. Host-only camera timing is not ESP32 timing.

## Architecture and implementation state

- `Sim_Environment` provides a browser scene, decoupled 3-axis plant, synthetic target/camera/IMU/magnetometer data, disturbances, occlusion, and a versioned HTTP/JSON bridge.
- `CubeSat_ESP32` includes a tested Python SIL reference plus ESP-IDF C source for control, detection, bridge, optional camera and sensors, and a default-off moteus UART path. Firmware source has not been compiled.
- `Ground_Station` provides a dashboard, telemetry plots, camera preview, CSV logging, and bounded supervisory commands. Handler-level tests exist; no live browser/session check passed in this environment.
- Camera frames use bounded, sequence-paired RLE1 transfer in the simulator path. The reports show exact reconstruction of the synthetic frames; this does not measure HTTP/Wi-Fi/Wokwi throughput.
- ESP32-S3 TWAI is Classical CAN, while moteus uses CAN-FD. The current source candidate is the moteus r4 AUX2 UART interface; wiring and motor behavior remain unverified.

## Outstanding work and decisions

1. Run the three-process second test drive in a normal desktop session, then diagnose the reported `127.0.0.1:8000` / `:8001` access issue using the prepared checklist and launcher diagnostics.
2. Build the ESP-IDF project in an activated ESP-IDF environment, run the portable C suites with a C11 compiler, then boot the Wokwi bridge path.
3. Confirm actual kit revisions, camera focus/module, sensor breakout voltage/pull-ups, sensor axes, and camera mounting/calibration.
4. Measure wheel inertia, torque/speed limits, motor response, UART electrical compatibility, watchdog and safe-stop behavior before claiming motor milestones.
5. Confirm allowed physical gimbal-axis configuration, acceptance criteria, radio/TM/TC protocol, and whether the physical target will include an orientation marker.
6. Replace assumed plant/noise values with measured rig and sensor data; rerun performance and robustness checks.

## Context reviewed and limits

The continuation review covered the workspace/project instructions, AE README and handoff, project brief, design description, software review, AI-use history, original user prompts/notes, verification traceability, current run guide, and generated reports. `Context/original_chat.json.txt` is empty. The two supplied PDFs are present (8 and 20 pages), but no PDF text extractor is installed in this environment, so their embedded text was not newly extracted in this session. Existing project notes summarize their requirements and cite them as source material.

The AE tree also contains archived `NR/Test_drive_*` snapshots; the active `outputs/Python_sim/` worktree is the current source of truth. The full raw contents of every source/code file and duplicate archive file were not read line-by-line in this resume pass; the code inventory, project review, tests, current handoff, and recent AI-use log were used to restore working context. No claim is made that this report replaces review of a specific implementation before changing it.

## Key files for continuing

- [Project handoff](../../Context/HANDOFF.md) — durable decisions, latest checkpoint, and next actions.
- [Project brief](../../Context/PROJECT_BRIEF.md) — course targets and open design decisions.
- [Software review](../../Context/CODE_REVIEW.md) — implementation and verification limits.
- [Requirements traceability](../Verification/Requirements_Traceability.md) — evidence against project requirements.
- [Full status and run guide](AE_Project_Status_and_Run_Guide.md) — run/build commands and detailed history.
- [Second test-drive checklist](../Python_sim/SECOND_TEST_DRIVE.md) — next live desktop trial.
