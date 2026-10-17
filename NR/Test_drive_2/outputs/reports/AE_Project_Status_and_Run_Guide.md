# AE CubeSat Project: Status and Run Guide

**Report date:** 17 October 2026  
**Project state:** Updated second test drive prepared. Host-SIL yaw-rate damping and moderated roll tuning are implemented; ESP-IDF compilation, live networking, Wokwi execution, and physical hardware verification remain open.

## Second test-drive package

The three simulation components now have Windows batch launchers in
`outputs/Python_sim/Launchers/`. They start the environment, host CubeSat SIL,
and Ground Station in separate WSL terminal windows. The environment and
Ground Station launchers open their browser pages. An installer script can
create three desktop shortcuts. The current startup order, trial sequence,
shutdown instructions, and expected results are in
[`outputs/Python_sim/SECOND_TEST_DRIVE.md`](../Python_sim/SECOND_TEST_DRIVE.md).

These launchers are click-to-run scripts, not standalone `.exe` files. A native
Windows executable could not be produced or validated from this session:
WSL-to-Windows invocation currently fails, and no packager is available. The
launcher files themselves were reviewed but not executed. The first live
network/browser test must be performed from a normal socket-enabled desktop
session. Run the second test-drive checklist from that desktop session.

The first user trial reports that both `127.0.0.1:8000` and `:8001` are
unreachable. The root cause is not established yet. Updated launchers now check
WSL/Python before starting and leave component errors visible in the terminal.
`Launchers/Diagnose_Connections.bat` checks the listener and HTTP paths inside
WSL and Windows localhost forwarding, then saves a desktop transcript. Diagnosis
needs that output from the user's desktop. The user also reports that terminal
windows close when the browser opens; the launchers now wrap WSL in persistent
`cmd.exe /k` windows so startup errors remain visible. This Windows behavior
still needs a user-side retry.

> **Session refresh (16 October 2026):** The first-test-drive package was
> prepared, then `./run_verification.sh` completed successfully: 187 Python
> tests passed, syntax compilation passed, and all five model/camera reports
> regenerated. Portable C and ESP-IDF checks were explicitly skipped because no
> C compiler or `idf.py` is on PATH. A separate capability check found loopback
> socket creation denied (`PermissionError: Operation not permitted`), Windows
> executable invocation unavailable, and Playwright/Selenium/Pyppeteer absent.
> The earlier verification attempt in this session was interrupted; it is
> superseded by the completed run above. The new launchers were reviewed but not
> executed. Live networking/browser checks still need a normal desktop session.

## 1. Project at a glance

The project is a CubeSat-style visual attitude-control demonstrator. Its current software model actively controls yaw with a reaction wheel, while simplified roll and pitch pendulum dynamics represent the other gimbal axes. The planned onboard computer is the Seeed XIAO ESP32-S3 Sense. A PC Ground Station supervises and records telemetry; it does not close the autonomous pointing loop.

The workspace contains three main components:

1. `Sim_Environment` — browser scene, plant dynamics, synthetic sensors, disturbances, and HTTP/JSON bridge.
2. `CubeSat_ESP32` — Python host software-in-the-loop (SIL) reference and ESP-IDF C firmware intended for the ESP32-S3 and Wokwi.
3. `Ground_Station` — operator dashboard, telemetry plots, CSV logging, and bounded supervisory commands.

The host Python path is integrated and has passing automated tests. The firmware source and emulator configuration exist, but **firmware build or Wokwi success has not been demonstrated**.

### Simulated-part progress estimate and project tree

**Estimated simulation-work completion: about 65%.** This is a planning estimate,
not a measured requirement score. The environment, Python onboard reference, and
Ground Station implementations are present and connected by handler/model-level
integration. Automated model checks and synthetic interfaces are in place. The
remaining share is mainly live three-process networking and browser execution,
plus building and exercising the ESP-IDF/Wokwi path and tightening simulation
timing/measurement evidence. A 100% simulation estimate means these software and
emulator paths have been run end to end; it does not include physical hardware
calibration or HIL.

```text
AE simulated project
├── Sim_Environment
│   ├── 3D scene, target XYZ/scale, XYZ torque pulses, camera occlusion  IMPLEMENTED
│   ├── plant, synthetic camera/IMU/magnetometer, HTTP/JSON bridge       IMPLEMENTED
│   └── browser behavior and live bridge process                        NEEDS EXECUTION
├── CubeSat_ESP32
│   ├── Python SIL: detector, yaw tracking + IMU loss damping           IMPLEMENTED
│   ├── model, transport, and handler-level integration                  TESTED IN PYTHON
│   └── ESP-IDF C firmware and Wokwi profile                             NEEDS BUILD/RUN
├── Ground_Station
│   ├── dashboard, plots, preview, housekeeping, CSV, supervisory cmds   IMPLEMENTED
│   └── browser rendering and live process link                           NEEDS EXECUTION
├── Interfaces and project documentation
│   ├── HTTP/JSON simulator API, TM/TC ICD, run/build commands            DOCUMENTED
│   └── Python command/telemetry/frame integration                       TESTED IN MEMORY
└── Verification
    ├── 191 Python tests, model scenarios, camera geometry and transport checks  PASSING
    ├── direct yaw parameter screen: 32/32 corners meet timing targets    MODEL CHECK PASS
    ├── coherent-pixel detector and noise screen                  PYTHON TESTED; C SOURCE ONLY
    └── live TCP, ESP-IDF compilation, Wokwi, physical camera/motor       NOT VERIFIED
```

The tree describes implementation and evidence separately: a component marked
implemented may still need its live runtime check. The parameter screen uses
perfect target and rate measurements and does not establish end-to-end
robustness.

## 2. What is done

### System design and organization

- The project README, handoff, prompt, architecture/ICD documents, requirements traceability, subsystem notes, and AI-use log provide persistent working context.
- The supplied guide and notes remain under `Source/` and `inputs/`; output work is organized under `outputs/`.
- System design and interface documents describe subsystem roles, message flow, axis conventions, provisional model assumptions, hardware boundaries, and unresolved decisions.
- Human-facing subsystem notes cover simulation, camera, sensing, control, actuation, telemetry, and Ground Station.
- Run/build commands and known verification limits are in `outputs/Python_sim/README.md` and the firmware README.

### Simulation environment

- A dependency-free Python plant models yaw, roll, and pitch. Yaw has a reaction-wheel model; roll and pitch use simplified pendulum dynamics.
- The environment produces synthetic camera, IMU, and magnetometer observations and accepts target XYZ/size, bounded XYZ disturbance-torque pulses, and camera occlusion inputs. The circular camera-facing marker uses pinhole size from optical-axis depth; oriented poster geometry is not modeled.
- A browser scene presents the cube, target, state readouts, and controls.
- A versioned local HTTP/JSON bridge connects the plant to the host SIL, Ground Station, or intended Wokwi firmware path. Host SIL and Wokwi transfer frames through bounded lossless RLE1 and reconstruct the full raster before onboard detection; raw frames remain available for inspection.
- Model values, sensor characteristics, and camera calibration remain provisional rather than measured hardware parameters.

### CubeSat software and firmware

- `CubeSat_ESP32/sil.py` is the Python onboard reference, nominally running at 50 Hz.
- The host SIL and firmware source include a bounded yaw controller, yaw-only attitude estimator, dark-target centroid detector, operating modes, telemetry, and command handling.
- The estimator integrates body-Y gyro rate. SIL can correct yaw using the simulated magnetic reference; physical firmware suppresses magnetic correction until real sensor mounting and magnetic calibration are established.
- ESP-IDF source includes optional XIAO Sense camera, MPU-6050, and LIS3MDL adapters. Physical options default off. The duplicate camera Y-error member/assignment found during review was removed. The synchronous HTTP/control loop uses the main task with `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` and a debug high-water sample every 250 cycles; this is a provisional budget that requires on-target margin measurement.
- An optional moteus r4 AUX2 UART adapter is default-off and source reviewed. It has bounded torque/velocity commands, status/fault checks, a command watchdog, and an arm input. It is not a built or hardware-validated interface.
- Wokwi configuration targets the XIAO ESP32-S3 and the local simulation bridge. Wokwi is an emulator path, not physical hardware-in-the-loop.

### Ground Station

- The browser interface displays operating state, telemetry, yaw-error history, actuator information, and a recent camera preview when supplied.
- The history plot preserves time gaps during invalid target observations.
- Camera preview metadata is paired with the frame so the displayed X/Y target marker corresponds to that preview.
- The station writes telemetry to `Ground_Station/logs/telemetry.csv` when run.
- Supervisory commands include PING, selected mode changes, bounded timed wheel-speed commissioning, and relative yaw-step requests. The station does not send arbitrary body-torque commands.

### Automated and model evidence

The latest recorded Python suite result is **191 passing tests**. Python `compileall` passed, and the deterministic model report includes a five-pose tilted yaw-estimator check and a separate five-pose camera projection check against an independent rotation matrix. Both check coordinate consistency only. It also records passive roll/pitch decay, a six-second full-direction tilted pointing diagnostic, and an ideal synthetic camera detector sweep: the default 0.1 m target at 2 m is detected at 13/13 bearings over ±30°, with up to 2.91 px centroid discrepancy at the clipped image edge. A direct yaw-control/plant corner screen meets the acquisition and nudge timing criteria in all 32 combinations of 0.5×/2× inertia, damping, wheel torque, lag, and paired gains. An observation-level 81-case noise/latency screen meets acquisition and nudge timing in every case and 60 s hold in 80/81; the sole miss reaches 2.034° under assumed combined stress inputs that are not measured hardware limits. One gyro noise input corresponds to the MPU-6050 datasheet 0.05°/s point; it is a comparison, not breakout calibration. An eight-point gain sensitivity run finds seven settings pass the 81 assumed cases; the best tested Kp=0.14 N·m/rad, Kd=0.05 N·m·s/rad reaches a 1.943° worst hold peak, while Kp=0.10/Kd=0.05 misses one case at 2.018°. Even the best has little margin, so configured gains remain unchanged pending measured rig and sensor/link properties. In a separate 50-seed repeat of the critical corner, baseline and Kp=0.14/Kd=0.05 each pass 49/50; the candidate improves median error from 1.682° to 1.648° but worsens p95 from 1.885° to 1.929° and worst peak from 2.008° to 2.112°. This covers one assumed input corner only. In the 81-case observation-level control screen, camera-bearing noise is added after centroid detection. The camera geometry report adds a 144-frame size/range/horizontal/vertical-bearing grid and five fixed-target body-pose frames; the five pose samples match analytic detector X/Y metadata exactly. A separate pixel-domain study applies synthetic additive Gaussian noise to the grayscale raster before detection: using 30 deterministic seeds per bearing, the pixel-domain study retains 210/210 targets through σ=30 (max centroid change 0.457 px), 209/210 at σ=40, and 191/210 at σ=50; it produces no blank-scene detections in 30 trials at σ=40 or 50. The detector uses an 8-neighbour coherence rule. These are explicitly assumed noise levels, not camera specifications. See [camera image-noise results](../Verification/Camera_Image_Noise_Results.md). Neither study establishes calibrated sensor, optical, or rig behavior. The Python QVGA detector now uses row bitsets for its local 8-neighbour filter; a deterministic differential test checks exact output against the former direct-neighbour algorithm. The latest 50-call host timing sample measures 2.057 ms median for a clean target and 6.202 ms median / 8.774 ms maximum on an all-dark frame. This is detector-only WSL timing; firmware and full-cycle deadlines remain unmeasured. See [camera detector timing results](../Verification/Camera_Detector_Timing_Results.md). The suite includes an in-memory closed-loop visual acquisition from the sequence-paired RLE1 scene image through SIL decoding, detection and control and plant actuation to Ground Station telemetry. The regression converts detector pixel error to angle and checks both that measurement and plant truth enter the 2° acquisition bound within six simulated seconds while control torque remains limited. The suite also checks the RLE1 bridge → SIL decoder/detector → telemetry-matched preview path; these handler tests do not create real TCP sockets. The Ground Station labels camera/IMU samples older than the 100 ms control freshness limit as stale and omits stale target-error samples from its plot; handler tests check this display logic, but no browser runtime is available here. Static source checks find no duplicate DOM IDs or missing JavaScript element references, and confirm each UI endpoint is named in its matching HTTP handler; they do not execute JavaScript or establish browser rendering. A direct environment-service launch was attempted and fails at socket creation in this session (`PermissionError: Operation not permitted`), so live local networking remains unverified.

The current model report records these results:

| Scenario | Model result | Interpretation |
|---|---:|---|
| Acquire from 30° to ≤2° | 1.600 s | Model pass |
| Hold for 60 s | 0.107° peak error | Model pass |
| Recover from 20° nudge | 1.340 s | Model pass |
| Track target moving at 0.2°/s for 60 s | 0.119° peak error | Model case pass; project acceptance rate/error still to confirm |
| Recover after 2 s camera occlusion | 0.760 s | Model pass |
| Yaw estimate with synthetic magnetic reference | 0.068° peak error; 0.0503° final drift/60 s | Synthetic model result only |
| 45° yaw-step command | Entered settle band at 5.820 s; completed dwell at 6.820 s | Model pass; firmware/hardware still unverified |
| Reach 3000 rpm in 0.5 s | Placeholder plant reaches 431 rpm | Current model does not meet the proposed speed milestone; hardware performance unknown |

These are deterministic Python model results, **not** ESP32, emulator, sensor, motor, or physical-rig measurements.

### Controller tuning follow-up (17 October 2026)

Yaw control uses Y-axis gyro rate during visual tracking and a separate
`Kd=0.06 N·m·s/rad` when the target is lost (1.5× the tracking rate gain). A
closed-loop test applies Y torque of 0.1 N·m for 0.5 s, drives the target out
of frame, and verifies the rate falls below 0.02 rad/s within 4.5 s of target
loss. The simplified reaction-wheel limit controls stopping time; this is
simulation-only evidence.

After the user asked to moderate the recently strengthened roll stabilization,
host-SIL roll gains are now `Kp=0.24 N·m/rad`, `Kd=0.19 N·m·s/rad`, and the
virtual roll torque cap is ±0.035 N·m. The Z-pulse regression peaks below 11°
and enters the 1° / 0.5°/s settle band within 3 s after a 0.05 N·m, 0.5 s
disturbance. The prior high-gain configuration peaked below 5° and recovered
faster. The moderated controller permits more transient motion by design.
These are host-SIL results; physical firmware and live UI behavior remain
unverified.

## 3. How to run the Python software

Use WSL/Linux with Python 3. Open three terminals. In each, change to:

```bash
cd /home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_2/outputs/Python_sim
```

### Terminal 1: simulation environment

```bash
python3 -m Sim_Environment.main
```

Open the simulation UI at <http://127.0.0.1:8000>.

### Terminal 2: CubeSat host SIL

```bash
python3 -m CubeSat_ESP32.sil
```

The SIL polls the environment bridge and returns onboard-reference actuator telemetry. It requires the environment process to be running.

### Terminal 3: Ground Station

```bash
python3 -m Ground_Station.main
```

Open the Ground Station at <http://127.0.0.1:8001>. It uses the environment bridge and onboard telemetry; run both the environment and SIL for the integrated scenario. The Ground Station creates/appends `Ground_Station/logs/telemetry.csv` during operation.

### Run automated checks

From the `Python_sim` directory, run the complete offline test, syntax, and
report-regeneration workflow with:

```bash
./run_verification.sh
```

It runs C host tests and an ESP-IDF build when those toolchains are available;
otherwise it prints explicit skips. Live networking, browser rendering, Wokwi,
and hardware checks remain separate runtime gates. To run only the documented
Python module suite, use:

```bash
python3 -m unittest Sim_Environment.test_model Sim_Environment.test_environment Sim_Environment.test_camera_transport CubeSat_ESP32.test_attitude CubeSat_ESP32.test_control CubeSat_ESP32.test_camera CubeSat_ESP32.test_moteus_protocol Ground_Station.test_main Verification.test_evaluate_scenarios Verification.test_speed_test_integration Verification.test_camera_transport_evaluation Verification.test_camera_image_noise Verification.test_camera_geometry Verification.test_camera_detector_timing Verification.test_ui_contracts Verification.test_emulation_config Verification.test_firmware_source_contracts
```

Check Python syntax:

```bash
python3 -m compileall -q Sim_Environment Ground_Station CubeSat_ESP32 Verification
```

Regenerate the deterministic model report:

```bash
python3 -m Verification.evaluate_scenarios > ../Verification/Simulation_Results.md
```

The tests can pass in a socket-restricted environment because the HTTP tests dispatch handlers in memory. To verify actual networking and browser behavior, run all three processes in a socket-enabled Linux/WSL environment.

### Legacy Level 0 sample

The older PyQt/UDP heartbeat example is separate from the HTTP/JSON SIL system:

```bash
python3 -m Ground_Station.simulator
python3 Ground_Station/level0_pyqt.py
```

The PyQt example requires PyQt5. Do not confuse this legacy heartbeat with the current control bridge or the eventual physical telemetry protocol.

## 4. ESP32 firmware and emulation commands

The firmware project is `CubeSat_ESP32/firmware/`. In a configured ESP-IDF terminal with the ESP32-S3 toolchain and managed components available:

```bash
cd /home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_2/outputs/Python_sim/CubeSat_ESP32/firmware
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

For Wokwi, build the firmware first, start the three host components above, enable the Wokwi private gateway, then run the project in the Wokwi VS Code extension. With ESP-IDF 6.0+ and `idf-wokwi` installed, the documented alternate launch is:

```bash
idf.py wokwi
```

Do not enable the physical camera/IMU/moteus options in Wokwi. Physical moteus actuation is default-off and must remain disabled until the exact controller setup, wiring, logic levels, torque/speed limits, and independent emergency stop have been checked.

The current WSL session has no working Linux `idf.py` or host C compiler. The mounted Windows toolchain could not be launched through WSL interop, and the old `firmware/build/` cache points to a previous Windows checkout. Therefore none of the commands above should be represented as having passed in this environment.

## 5. What remains to be done

### Required software/tool verification

1. **Build firmware from a clean, supported ESP-IDF environment.** Build default/Wokwi, camera + MPU-6050, magnetometer, and opt-in moteus configurations. Resolve all compiler/API/configuration errors and save logs or reproducible results.
2. **Run portable C tests.** From `outputs/Python_sim/`, run `./CubeSat_ESP32/firmware/run_host_tests.sh` to compile/run `test_yaw_controller.c`, `test_attitude_estimator.c`, and `test_black_target.c` with a C11 compiler; compare C results with the Python references across boundary cases.
3. **Run Wokwi end to end.** Demonstrate environment sensor cycles, CubeSat command/telemetry flow, Ground Station acknowledgements, reconnect behavior, and watchdog response using the actual built firmware.
4. **Run the three Python processes with live networking.** Check the two web pages, live telemetry and commands, preview alignment, disconnect/freshness behavior, and CSV file creation in a socket-enabled environment.
5. **Compare the C moteus UART frame implementation to its Python golden reference by executing a host comparison or an equivalent UART loopback test.** The current protocol logic has source documentation and Python reference tests, but the C encoder has not been executed or tested against a controller.

### Hardware and project decisions

6. Confirm exact XIAO Sense/camera module, MPU breakout, LIS3MDL breakout, moteus r4 revision/firmware, motor, wheel, gimbal, and wiring.
7. Measure sensor mounting/sign conventions, camera centre/focal length/thresholds, IMU bias/scales, magnetic hard/soft-iron offsets, local reference, and motor interference.
8. Confirm whether roll/pitch are free, pendular, or constrained for each prototype stage; the physical fixture has not been established in the software.
9. Measure wheel inertia, motor torque/speed/current/thermal limits, controller delay, and safe-stop behavior. The current plant values are placeholders. The 3000 rpm in 0.5 s target is not met by the current placeholder model.
10. Design and validate a physical telemetry/telecommand protocol, including link loss, integrity checks, authentication/security where needed, addressing, retries, and command authority. The current HTTP/private-gateway link is for SIL/Wokwi only.
11. Confirm project acceptance thresholds and measurement methods, especially moving-target performance and the proposed guide milestones.
12. If the physical rig requires full three-axis absolute attitude rather than yaw-only relative yaw, extend and validate the estimator accordingly.

### Physical safety gate

Before any powered actuator test, confirm the actual signal voltage and connector mapping, use motor power disconnected for UART bring-up, establish an independent power cut, and agree on a restrained test setup and safe limits. The source-level arm input and watchdog are not proof of a safe laboratory system.

## 6. Main project files

| Purpose | Location |
|---|---|
| Project overview and startup instructions | [`../../README.md`](../../README.md) |
| Current project state and next actions | [`../../Context/HANDOFF.md`](../../Context/HANDOFF.md) |
| System design | [`../Design_Description.md`](../Design_Description.md) |
| Subsystem document index | [`../Subsystems/README.md`](../Subsystems/README.md) |
| Software commands and runtime limits | [`../Python_sim/README.md`](../Python_sim/README.md) |
| Architecture/interfaces | [`../Python_sim/docs/System_Architecture_and_Interfaces.md`](../Python_sim/docs/System_Architecture_and_Interfaces.md) |
| Simulation HTTP API | [`../Python_sim/docs/Simulator_HTTP_API.md`](../Python_sim/docs/Simulator_HTTP_API.md) |
| TM/TC interface contract | [`../Python_sim/docs/TMTC-ICD.md`](../Python_sim/docs/TMTC-ICD.md) |
| Model results | [`../Verification/Simulation_Results.md`](../Verification/Simulation_Results.md) |
| Requirements and verification status | [`../Verification/Requirements_Traceability.md`](../Verification/Requirements_Traceability.md) |
| Hardware/emulation references | [`../../references/Hardware_and_Emulation_Notes.md`](../../references/Hardware_and_Emulation_Notes.md) |

## 7. Current conclusion

The Python reference system is organized, documented, integrated at the handler/model level, and the updated full-direction host-SIL state passes 191 Python tests plus syntax compilation. Five model and camera reports were regenerated, and the Markdown audit found no broken relative links across 33 files. The Ground Station CSV records camera focal length alongside image offsets and target bearing; a source contract checks that C/SIL telemetry fields reach the CSV schema. Static checks confirm Wokwi project paths, board/PSRAM configuration, network defaults, and camera component declaration are internally consistent; they do not verify an emulator boot. A fresh camera frame with no target permits gyro-rate damping; missing, invalid, stale, or future camera/IMU samples fail closed to zero torque. Python SIL covers this at runtime; C source checks cover the matching freshness policy and command contract without executing the firmware parser. The physical velocity command shares the firmware torque cap (1 mN·m default, 20 mN·m maximum); at provisional model inertia, even the maximum cap yields only an ideal 955 rpm in 0.5 s versus the 62.8 mN·m required for 3000 rpm. Hardware inertia and configuration remain unknown. The Windows click launchers and desktop shortcut installer are prepared but have not been executed. Firmware build, C execution, live networking, browser rendering, Wokwi, physical interfaces, and hardware performance remain unverified. The next checkpoint is for the user to run baseline, vertical pitch, horizontal yaw, Y-axis IMU damping after target loss, and moderated Z-axis roll recovery using `SECOND_TEST_DRIVE.md`, then report observations and any discrepancies.
