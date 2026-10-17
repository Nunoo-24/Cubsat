# AE project handoff

## Objective and current state

**Current checkpoint (17 October 2026): host-SIL camera pointing is verified
for both yaw/pitch error signs and both roll directions; the last full Python
suite passed 218/218.** The fresh
simulation and all-axis reports have been regenerated. Python syntax
compilation passes. No native C compiler or Linux `idf.py` is available in the
WSL PATH. A Windows ESP-IDF 6.1 installation exists on the mounted C: drive,
but WSL cannot launch its compiler (`UtilBindVsockAnyPort`); neither C tests
nor the ESP-IDF build has passed.
Host SIL uses image X/Y for yaw/pitch and a directed image angle for roll when
the optional asymmetric marker is enabled. A 15° roll test returns to the
±1°/±0.5°/s band in 8.32 simulated seconds while the accelerometer roll cue is
hidden. Point-only images still return no visual roll estimate and fall back to
IMU leveling. The synthetic marker is disabled by default and is not part of
the supplied physical target.

### Resume review and fresh verification (17 October 2026)

Ran `outputs/Python_sim/run_verification.sh` from the active software worktree.
All 218 Python tests passed in 110.024 s, Python syntax compilation passed,
and all six report generators completed. Portable C tests and ESP-IDF build
were explicitly skipped because no C compiler and no `idf.py` are available on
PATH. Generated results are summarized in
`outputs/reports/AE_Project_Current_State_Report_2026-10-17.md`.

The continuation review covered durable workspace/project instructions,
current project brief, software review, original user prompts/notes, design
description, requirements traceability, status guide, generated results, and
recent AI-use history. `Context/original_chat.json.txt` is empty. The two PDF
sources (8 and 20 pages) were not newly text-extracted because no PDF utility is
installed; project notes already summarize the sources. The raw contents of
every source file and duplicate `NR/Test_drive_*` snapshots were not reread
line-by-line. Treat the active `outputs/Python_sim/` tree as authoritative and
inspect the affected implementation before the next code change.

Next action remains a live three-process test in the user's normal desktop
session using `outputs/Python_sim/SECOND_TEST_DRIVE.md`; afterward, follow with
a native ESP-IDF build, portable C tests, and Wokwi bridge run when those
toolchains/runtime are available. Physical component revisions and axis scope
still require confirmation.

### Test Drive 2 package refresh and GitHub publication attempt (17 October 2026)

Rebuilt `outputs/Python_sim/NR/Test_drive_2/` from the active AE project tree,
including current Context, Source, inputs, outputs, reports, and references.
Removed nested Git data, generated Python caches, build/log directories, and
machine-specific editor settings. Refreshed the snapshot readme/state and the
`NR/README.md` index; left Test Drive 1 unchanged. The snapshot is 137 files,
about 2.3 MiB. Its own `run_verification.sh` run passed 218/218 Python tests in
116.493 s, syntax compilation, and all six report generators. Portable C and
ESP-IDF steps skipped because their toolchains are unavailable.

Committed only `NR/Test_drive_2/` and `NR/README.md` as
`8551a6c` (`Refresh Test Drive 2 snapshot`). `git push origin main` failed
before authentication because this environment could not resolve `github.com`.
The commit is local only; GitHub has not been updated. The pre-existing staged
active-project changes in the nested Python_sim worktree were excluded from
this commit and remain staged locally.

The portable C detector estimates the same marker angle. ESP32 firmware now
includes a bounded pitch/roll controller and sends virtual torque to the scene
only when it is using a simulator-sourced frame. Physical motor actuation is
still yaw-only. The new C code has not been compiled because no host C compiler
or ESP-IDF toolchain is available; Wokwi, live sockets, browser behavior, and
hardware are unverified. Run `./run_verification.sh` to refresh the Python
suite/reports and record explicit C/ESP-IDF skips.

Next: complete verification and documentation for the marker-controlled path,
then use `SECOND_TEST_DRIVE.md` to check browser data flow with the marker toggle
on and off. A physical visual-roll trial requires adding and validating a
resolvable asymmetric mark on the target poster; no powered pitch/roll hardware
is in project scope yet.

The physical XIAO camera startup now detects OV5640 and requires Espressif AF
support, successful autofocus initialization, and a focused status within 10 s;
it deinitializes on failure. OV3660 is accepted as fixed focus; unknown sensor PIDs are rejected. The
managed camera dependency is `^2.1.8`, its component is conditionally required
in CMake, and AF support is enabled in defaults. Source/config tests pass, but
ESP-IDF compilation and board behavior are not verified. See the camera
subsystem note and hardware research for citations.

Build and demonstrate an autonomous CubeSat-style attitude-control system. The
host software demonstrator controls yaw with a reaction wheel and uses virtual
pitch/roll torque inputs for full-direction pointing on the project guide's
three-freedom gimbal model. Physical firmware and the selected hardware still
support only yaw. The XIAO ESP32-S3 is intended to
own image/sensor processing and control; the PC Ground Station supervises and
records telemetry. Axis constraints for the physical prototype and the exact
kit revisions still need confirmation.

The Python software path is integrated and its model/unit checks pass. The
ESP-IDF firmware and portable C additions are source implemented, but not
compiled. Live HTTP/browser operation, Wokwi, bench interfaces, and HIL have not
been verified. See [the software review](CODE_REVIEW.md), [requirements
traceability](../outputs/Verification/Requirements_Traceability.md), and the
[AI-use log](AI_USAGE_LOG.md) for focused evidence/history.

## Start each session here

Read `/home/nunot/Mestrado_Codex/README.md`, this handoff, and
`/home/nunot/Mestrado_Codex/AE/README.md`; then read
`/home/nunot/Mestrado_Codex/AE/inputs/Prompts.txt` before implementation.
Consult `PROJECT_BRIEF.md`, `CODE_REVIEW.md`, and the software README as needed.

## Architecture and decisions

The physical camera freshness check now uses the Espressif frame buffer's
`timestamp` (first DMA buffer since boot), rather than taking a timestamp after
`esp_camera_fb_get()` returns. Zero and future timestamps fail closed. This
change is source-reviewed and still needs an ESP-IDF build and target check.

- `outputs/Python_sim/Sim_Environment/` models yaw with a reaction wheel and
  pendular roll/pitch; host SIL can apply bounded virtual pitch/roll torques.
  It provides synthetic camera, IMU and fixed magnetic-field observations, a
  browser scene, and a versioned loopback HTTP/JSON bridge.
- `outputs/Python_sim/CubeSat_ESP32/sil.py` is the host onboard reference at
  nominal 50 Hz. ESP-IDF firmware includes a bounded yaw controller, dark-target
  detector, optional XIAO Sense camera and MPU-6050/LIS3MDL drivers, and an
  optional default-off moteus r4 AUX2 UART actuator profile. Wokwi uses the
  simulation bridge/private gateway; it is not physical HIL.
- In host SIL, visual yaw tracking uses the Y-axis gyro rate; if the target is
  lost, IMU-only yaw damping continues. Current lost-target damping gain is
  0.06 N·m·s/rad. Roll gains are moderated to Kp=0.24, Kd=0.19 with ±0.035 N·m
  virtual torque. Pitch/roll remain simulation-only.
- The Python and portable C paths report a **yaw-only** estimate: gyro Y
  integration, tilt compensation from gravity, and magnetic correction when a
  trusted reference is available. SIL uses the simulator's known fixed field.
  Physical firmware currently reports gyro-only relative yaw because sensor
  mounting, hard/soft-iron calibration, local reference, and motor interference
  are not characterized. Regular visual pointing remains camera-error based;
  the explicit relative yaw-step command uses the estimate as its setpoint.
- The operator UI sends PING, enumerated modes, bounded wheel-speed commissioning,
  and `SET_YAW_STEP`. The yaw command is relative to the current estimate,
  bounded to ±180°, accepted only in SAFE/STANDBY with a valid estimate, and
  reports progress/terminal status. The model settles at 1°/0.5°/s for a one-second
  dwell, with a ten-second timeout. Speed tests are limited to 0–50 rev/s and
  0.5–5 s and require SAFE/STANDBY; physical firmware also requires the
  independent arm input and requests zero speed at test end.
- Do not connect ESP32-S3 TWAI directly to moteus CAN-FD. The selected source
  path is TTL UART on moteus r4 AUX2, pending verification of exact controller
  revision/configuration, pinout, voltage levels, watchdog, and emergency stop.
  Physical command generation remains disabled by default.
- The current physical pointing path uses camera target error and gyro rate.
  Model conventions are body X=pitch, Y=yaw, Z=roll. Sensor axis/sign mapping
  and permitted gimbal axes need measurement/team confirmation.
- HTTP/private-gateway is for SIL/emulation only. The physical radio/TM/TC link,
  security, authentication, framing and replay behavior are not selected.

## Workspace and main deliverables

- Supplied material: `Source/0-Guiao.pdf`, `Source/1-project_guide.pdf`, and
  preserved user notes in `inputs/`.
- Human-facing design: `outputs/Design_Description.md`.
- Subsystem descriptions: `outputs/Subsystems/` (simulation, camera, sensing,
  control, actuation, telemetry and Ground Station).
- Runnable software: `outputs/Python_sim/` with separate `Sim_Environment/`,
  `CubeSat_ESP32/`, and `Ground_Station/` components.
- Shared contracts/research: `outputs/Python_sim/docs/` and
  `references/Hardware_and_Emulation_Notes.md`.
- Results and requirement status: `outputs/Verification/`.
- Reproducible camera transport size and host codec measurements:
  `outputs/Verification/Camera_Transport_Results.md`.
- Run/build commands: project `README.md` and
  `outputs/Python_sim/README.md`.
- Current live test sequence: `outputs/Python_sim/SECOND_TEST_DRIVE.md`.

## Verification evidence (2026-10-17, refreshed)

- **Python:** the documented module suite and
  `python3 -m unittest discover -s . -p 'test_*.py'` both pass all 218 tests;
  regressions cover Ground Station stale-data display, XYZ torque isolation,
  passive roll/pitch decay, and tilted camera projection.
  `python3 -m compileall -q Sim_Environment Ground_Station CubeSat_ESP32 Verification`
  passes. Socket-free HTTP-handler tests include
  the Ground Station command/acknowledgement path and the sequence-paired raw
  and RLE1 camera-frame endpoints; these do not prove live TCP networking.
  Static emulation checks also confirm the Wokwi firmware/ELF paths, project
  name, XIAO ESP32-S3 board and octal PSRAM setting, guest Wi-Fi/private-gateway
  configuration, and camera component declaration. They do not prove a Wokwi
  boot or ESP-IDF build.
  A C/Python source contract now checks command mode names, rejection reasons,
  and exact speed/yaw-step argument keys and counts; it cannot execute or
  compile the firmware parser.
  Telemetry-schema review also added camera focal length to Ground Station CSV
  output, with a handler-level regression confirming the value is persisted.
  A source-level contract now checks that all non-envelope C firmware and SIL
  telemetry fields are represented in the Ground Station CSV schema.
  Both camera and IMU ages remain required for onboard output. A fresh frame
  with no target allows rate damping; a missing/stale/future frame fails closed
  to zero torque. Python SIL regression and C source-contract checks guard the
  policy; C is not compiled or run here.
  The ICD and camera/control subsystem notes document this policy. These source
  checks are not a C compile or runtime test.
- **Model:** five pointing scenarios pass: 30° acquisition 1.600 s, 60 s hold
  peak error 0.107°, 20° nudge recovery 1.340 s, moving-target peak error
  0.119° at 0.2°/s, and recovery after a 2 s occlusion in 0.760 s. The
  synthetic-field yaw test reports 0.068° peak error and 0.050° final drift
  over 60 s; an ideal-vector grid recovers yaw within 0.000001° at five
  combined tilted poses. An independent rotation-matrix projection check
  matches camera X/Y pixels at five tilted poses. The 45° yaw-step model enters
  the settle band at 5.820 s and completes its one-second dwell at 6.820 s.
  With no external torque for 20 s,
  passive roll and pitch retain 13.978% of initial mechanical energy under
  provisional damping. This is a qualitative model check, not a rig target.
  These model results do not establish physical sensor accuracy; the estimator
  uses a placeholder magnetic-field calibration.
  A six-second tilted diagnostic retains about 8.82° vertical camera error
  after yaw reduces horizontal error to 1.24°; pitch/roll are not commanded.
  This is a scope limitation diagnostic, not a pointing acceptance test.
  An ideal-camera sweep detects the default 0.1 m target at 2 m across 13
  bearings from -30° to +30°; clipped-edge centroid discrepancy is at most
  2.91 px. Neither detector result measures physical detection rate or timing.
  A direct yaw-control/plant corner sweep meets the 30° acquisition and 20°
  nudge timing targets in all 32 tested combinations of 0.5×/2× inertia,
  damping, wheel torque, motor lag, and paired gains. Worst times are 3.08 s
  and 2.48 s; measurements are ideal and this is not end-to-end robustness.
  An 81-case observation-level noise/latency screen meets acquisition and nudge
  timing in 81/81 cases and the 60 s hold limit in 80/81. Separately, a pixel-domain
  evaluator applies Gaussian intensity noise before detection. Python and C now
  require each dark pixel to have at least two other dark 8-neighbours. With 30
  deterministic seeds per bearing, it retains 210/210 targets through σ=30,
  209/210 at σ=40, and 191/210 at σ=50; blank-frame detections are 0/30 at σ=40
  and 50. These are synthetic assumptions, not camera specifications; see
  `outputs/Verification/Camera_Image_Noise_Results.md`. Its one hold miss is
  2.034° under assumed 10 px RMS camera-error noise, -0.03 rad/s gyro bias,
  0.01 rad/s RMS added gyro noise, and 40 ms delay. The grid also includes
  0.05°/s RMS gyro noise as a TDK datasheet comparison point; this is not a
  calibrated breakout result. The observation-level screen injects bearing noise after detection; the separate
  image-raster noise study is described above. A reproducible eight-point gain sweep finds seven settings pass the grid; the
  best tested Kp=0.14/Kd=0.05 reaches a 1.943° worst hold peak, while
  Kp=0.10/Kd=0.05 misses once at 2.018°. The best candidate still has little
  margin, so configured gains remain unchanged pending measured properties.
- **MPU timing/noise update:** the source configures DLPF_CFG=3 (42 Hz gyro
  bandwidth, nominal 4.8 ms delay) and SMPLRT_DIV=4 (200 Hz output). The sample
  timestamp subtracts 4.9 ms nominal filter delay plus a conservative 5 ms
  output phase. This is a source-level estimate only. The 81-case screen adds
  0.05°/s RMS as the datasheet comparison point (specified at DLPF_CFG=2); it
  does not imply breakout calibration or setting equivalence.
- **Wheel-speed milestone:** the command path is implemented, but its placeholder
  wheel model reaches only 431 rpm in 0.5 s. With provisional inertia
  1e-4 kg·m², ideal torque for 3000 rpm in 0.5 s is 62.8 mN·m before losses.
  The physical native-velocity frame uses the shared Kconfig torque cap
  (1 mN·m default, 20 mN·m maximum); at the model inertia that cap allows only
  an ideal 955 rpm in 0.5 s. Meeting 3000 rpm under the maximum cap requires
  total wheel inertia ≤3.18e-5 kg·m² before losses. Actual hardware inertia and
  safe current limits remain unknown.
  These numbers are model bounds, not measured hardware performance.
- **Links:** 43 project Markdown files checked on 2026-10-16; zero broken
  relative links.
- **Not run:** current direct launch of `python3 -m Sim_Environment.main` fails
  at socket creation with `PermissionError: Operation not permitted`. No host C
  compiler, `idf.py`, Wokwi CLI, Node.js, or browser is available in this WSL
  session. Therefore C host tests, ESP-IDF build, emulator execution, live
  networking, UI rendering, physical sensors/actuator, and HIL remain unverified.
  Old build cache from the former Windows checkout is stale and is not evidence.

Regenerate `outputs/Verification/Simulation_Results.md` from
`outputs/Python_sim/` with:

```bash
python3 -m Verification.evaluate_scenarios > ../Verification/Simulation_Results.md
```

Run the Python suite with the exact command in `outputs/Python_sim/README.md`.

## Main gaps and next actions

1. In a functioning host C environment, compile/run `test_yaw_controller.c`,
   `test_attitude_estimator.c`, and `test_black_target.c`; compare Python/C
   estimator/controller behavior across edge cases.
2. In ESP-IDF 6.1 (or supported 5.3+), resolve managed components and build
   default, Wokwi, physical camera/IMU, and optional moteus profiles. Resolve
   actual compile/API errors and record reproducible build results.
3. In a socket-enabled environment, run plant, SIL/firmware, and Ground Station
   as independent processes. Verify commands, acknowledgements, telemetry, CSV,
   status freshness, camera preview, and watchdog behavior in a browser.
4. Confirm the board/camera/sensors/motor/wheel/controller revisions and gimbal
   constraints. Calibrate camera intrinsics/thresholds, IMU bias/frame/scale,
   magnetic offsets/soft-iron/reference, and motor/wheel inertia/limits.
5. Validate actuator UART, watchdog and safe-stop with motor power disconnected
   and an independent power cut before any powered test. Validate speed
   performance only after real motor/wheel parameters and safe test limits are
   established.
6. Build and run the firmware yaw-step implementation in ESP-IDF/Wokwi and
   compare it with the Python reference; decide whether a full three-axis
   estimator is required for the physical rig. Define the moving-target
   acceptance limit with the project team.
7. Compare results with supplied guide and actual hardware, update the design,
   subsystem notes, interface docs, README and this handoff, then repeat checks.

## Preserved constraints

- The guide describes a three-freedom gimbal with active yaw and pendular
  roll/pitch. Early notes proposed locking two axes; confirm the allowed
  prototype configuration instead of silently choosing one.
- The guide's 3000 rpm, attitude error/drift, and 45° step criteria are proposed
  milestones and require confirmation with the team/lecturer. Current model
  constants are provisional.
- Magnetometer calibration and motor interference are unresolved; raw physical
  field readings must not be treated as an absolute heading reference yet.
- The user-listed magnetometer range and I²C/SPI interfaces are consistent with
  the LIS3MDL family and likely match an Adafruit breakout, but the kit board/vendor
  is not confirmed. The “5.5V MPU-6050” likely describes a breakout input rating;
  the bare chip has a lower specified supply and the breakout pull-up rail is
  unknown. Check board markings/schematics and XIAO I²C voltage before wiring.
- Confirm exact board/camera module and moteus revision/configuration, plus the
  independent emergency-stop procedure, before physical actuation.
- Course policy permits project AI use with logging and primary-source checks;
  the student/team remains responsible for the submitted technical work. AI
  must not be used for the E4 written exam.

## Latest continuation checkpoint (2026-10-16)

- Camera timing audit: official Espressif `camera_fb_t` docs define its
  timestamp as since-boot time of the frame's first DMA buffer. Firmware now
  uses it rather than time-after-fetch, and rejects invalid zero/future values.
  Firmware README and hardware research notes explain the freshness behavior.
- Clarified LIS3MDL Kconfig help: readings are scaled to microtesla but not
  hard/soft-iron calibrated or fused.
- Verification rerun: 97 Python tests pass; `compileall` passes; model report
  regenerated with the seven documented scenario outcomes. Markdown audit
  found 56 files and zero broken relative links.
- This is a source-level camera change only. Linux ESP-IDF compiler and Wokwi
  remain unavailable; no claim is made for firmware build, live networking, or
  physical camera timing. Next useful step is to run `idf.py build` in a working
  ESP-IDF environment, then verify capture timestamps and fail-closed behavior
  on the XIAO camera.

## Build environment recheck (2026-10-16)

The ESP32-S3 Windows cross-compiler remains present under
`/mnt/c/Espressif/tools/xtensa-esp-elf`, but WSL interop currently fails even
for `cmd.exe /c ver` with `UtilBindVsockAnyPort: socket failed 1`. No Linux
`idf.py`, Docker/Podman, or host C compiler is available. This independently
confirms the existing ESP-IDF build blocker; do not treat cached build files or
Kconfig parsing as a build. Current CMake conditional dependencies and source
paths for physical sensors, camera freshness, and moteus UART were inspected,
but no new C compilation or hardware evidence was obtained. Resume with a
working native/Linux ESP-IDF 6.1 environment and run default plus camera/MPU/
magnetometer/moteus profiles.

## Requirements review checkpoint (2026-10-16)

The Ground Station dashboard showed `gyro_z_rad_s` as a fallback for yaw rate,
while the agreed project frame defines yaw around body Y and both SIL and
firmware set `yaw_rate_rad_s` from gyro Y. Corrected the dashboard fallback to
`gyro_y_rad_s` and added a handler test that checks the served UI contains the
correct axis mapping. The full suite now passes 98 tests; `compileall` passes;
the deterministic model report was regenerated. This is a Python/UI fix and
does not change the still-unverified firmware build or live browser status.

The Ground Station history plot previously omitted invalid-target intervals,
visually joining samples across camera occlusion. It now records every unique
telemetry sample against simulation time and breaks the plotted line across
missing-target samples. Ground Station docs describe this behavior; handler
checks protect the dashboard source. All 99 Python tests pass, `compileall`
passes, and the deterministic model report was regenerated. Browser rendering
is still unavailable in this socket-restricted session.

## Frame-matched camera preview checkpoint (2026-10-16)

The physical camera preview had a timing mismatch: Ground Station overlaid the
latest telemetry error on a preview that could be one second old. The bridge
now captures the associated telemetry validity/error when the thumbnail is
uploaded and returns those values as `X-Camera-Target-Valid` and
`X-Camera-Error-X-Px` headers. Ground Station forwards and uses those headers
for its overlay. Handler tests verify pairing and proxy preservation. Telemetry
errors are in QVGA pixel units while the image bytes are 64x48. They are drawn
on a 320x240 canvas matching the original QVGA frame dimensions, so the pixel
offsets need no extra factor of five. Telemetry now carries both camera axes;
the environment's image-up Y coordinate is converted to image-down telemetry
coordinates to match the detector. The full suite now passes 103 Python tests,
`compileall` passes, and the model report was regenerated. This does not verify
physical camera timing or browser rendering.

Wokwi configuration review: official Wokwi docs confirm that ESP-IDF projects
use `build/flasher_args.json` plus the application ELF, and that the diagram
board type `board-xiao-esp32-s3` is supported. The current `wokwi.toml` paths
match the CMake project name `cube_firmware`. The software README now gives the
optional `idf.py wokwi` command for ESP-IDF 6.0+ when `idf-wokwi` is installed.
This is source/configuration review only; the project has not been built or run
in Wokwi here.

## Native compiler acquisition recheck (2026-10-16)

No native GCC/Clang/TinyCC/Zig or ESP-IDF compiler was found. The mounted
ESP32-S3 toolchain is Windows-only and needs the WSL interop path that fails
with `UtilBindVsockAnyPort`. A non-root `apt-get download gcc` attempt also
failed because DNS resolution for `archive.ubuntu.com` is unavailable. No
system packages or source files were changed. Continue source-level review and
Python integration work here; run firmware builds when a native ESP-IDF host or
working Windows/WSL interop is available.


## Source audit and verification refresh (2026-10-16)

Reviewed the physical MPU-6050, LIS3MDL, and moteus UART C drivers. The MPU configuration and conversions match the configured ±2 g/±250 dps ranges; its startup gyro bias routine assumes the unit remains still during calibration. The LIS3MDL configuration is 80 Hz continuous conversion at ±4 gauss with block-data update, and returns a cached sample only while its conservative timestamp remains fresh. The moteus adapter uses CRC-checked fdcanusb lines, bounded torque/velocity frames, response validation, and a finite command watchdog. These are source-level observations, not protocol/hardware validation. Confirm stable startup mounting, sensor addresses and breakout wiring, moteus firmware/configuration, AUX2 electrical levels, and arm/safe-stop behavior against the actual kit before enabling actuation.

Refreshed verification on this pass: all 103 Python unit tests pass; `compileall` passes; `Verification.evaluate_scenarios` regenerated the simulation report. No JavaScript runtime/browser automation module was available, so dashboard rendering remains unverified. ESP-IDF build and live socket checks remain unavailable as described above. Next action is still to obtain a working ESP-IDF host and then complete browser/network and physical interface checks.


## Camera firmware declaration audit (2026-10-16)

A source pass found a duplicate `error_y_px` member in `cube_camera_observation_t` and a duplicate assignment in `cube_xiao_camera_observe()`. Removed both. A targeted member-name check now confirms that the camera observation struct has unique members, and the physical camera → main-loop Y-error references resolve to the single declaration. The integrated Python suite still passes all 103 tests, `compileall` passes, and the model report was regenerated. This source fix removes an obvious C compile error, but the complete ESP-IDF firmware remains unbuilt and needs compilation in a working toolchain.


## Firmware profile/build-path recheck (2026-10-16)

Rechecked the current firmware build configuration after the camera header fix. `main/CMakeLists.txt` conditionally includes camera, MPU-6050, LIS3MDL and moteus sources with their corresponding ESP-IDF driver dependencies; the default profile leaves all physical drivers off. The Kconfig dependencies require camera plus MPU-6050 before the physical moteus option can be selected. No `idf.py`, Linux GCC/Clang, Wokwi CLI, or mounted Windows ESP-IDF framework/compiler was discoverable in the current session, so a real build is still not possible here. This confirms profile wiring at source/configuration level only. The remaining firmware validation gate is a fresh ESP-IDF build of default, camera/IMU, magnetometer, and opt-in actuator profiles on a working toolchain.


## Architecture description aligned with current software (2026-10-16)

Revised `outputs/Design_Description.md` after finding stale claims that estimator design was TBD and the Ground Station functionality was only planned. The document now describes the implemented yaw-only gyro/magnetic estimator distinction (synthetic reference in SIL, physical magnetic correction disabled), separate camera-bearing control, current selectable modes, frame-matched preview and CSV/plot behavior, and the detector's actual threshold/centroid limits. Corrected the operating-mode table to distinguish reported faults from selectable modes and clarified that normal pointing does not yet enforce a measured wheel-speed limit. All relative Markdown links across 39 project Markdown files resolve. No code behavior changed; Python tests were not rerun for this documentation-only update.


## Simulator controls and integrated regression refresh (2026-10-16)

Compared the Sim_Environment UI inputs with `Environment.apply()`, the API docs and the plant model. Target XYZ/scale, time-limited XYZ disturbances, and camera occlusion are represented consistently; the plant maps torque X to pitch, Y to yaw and Z to roll as documented. No UI/API/plant interface mismatch was found in this pass. Re-ran all nine documented Python test modules: 103 tests pass; `compileall` passes; the deterministic model-results report was regenerated. This does not cover live TCP/browser behavior, ESP-IDF compilation, Wokwi, or hardware.


## Consolidated status report prepared (2026-10-16)

Created `outputs/reports/AE_Project_Status_and_Run_Guide.md` at the user's request. It consolidates completed design/software work, exact Python run/test/report commands, ESP-IDF/Wokwi instructions, current model evidence, known environment limitations, and prioritized software/hardware work remaining. Verified all 11 relative Markdown links in that report resolve. The project goal remains paused; this report-writing task did not run or claim any new software, firmware, network, emulator, or hardware verification.


## User resumed simulation work (2026-10-16)

The user authorized continuing the simulated portion while hardware-dependent integration remains separate. Fresh verification in the current Python_sim tree: 103 tests pass and `compileall` passes. Continue with Sim_Environment, host SIL, Ground Station, and available model/integration verification; do not imply live-socket/browser or ESP32/Wokwi/hardware results without running them. The user also requested that the next status report provide an estimated progress percentage for the simulated project and a compact tree mapped to `inputs/Prompts.txt`, marking each requested component as implemented, needs testing, or still to do. Include that report when next preparing a user-facing progress report.


## Simulation event timing aligned to plant clock (2026-10-16)

The environment UI accepted torque pulse and camera-occlusion durations, but `Environment.apply()` timed them with wall-clock `time.monotonic()` while reports and operator readouts use plant simulation time. This made events nondeterministic under slow/paused execution and forced model tests to fake wall time. Changed both event timers to plant simulation time. `Environment.advance()` now integrates a torque pulse proportionally when it ends inside a plant step, preserving requested torque impulse, and validates dt before calculating event overlap. Updated the UI labels, HTTP API, environment README, subsystem note, verification note and requirements result. Updated camera-occlusion model recovery from 0.740 s to 0.760 s under the clarified clock convention.

Verification: 106 tests pass (the prior 103 plus two event-clock regressions and an invalid-dt guard); `compileall` passes; `Verification.evaluate_scenarios` regenerated the report. A fresh live server launch still fails at socket creation with `PermissionError: Operation not permitted`; therefore network/browser verification remains open.

## End-to-end simulated camera path (2026-10-16; initial raw-frame transport)

The environment now generates a sequence-paired raw QVGA grayscale frame from
target geometry, size, attitude, visibility and occlusion. Host SIL fetches
that frame and runs the Python centroid detector before control; the resulting
telemetry carries its camera frame sequence. The bridge downsamples that exact
image to the Ground Station's 64x48 preview and pairs detector X/Y errors with
it. The default Wokwi firmware source fetches the same raw frame and runs the
portable C detector when the physical camera profile is off. The physical
profile captures and detects locally. The previous direct analytic-pixel-error
bypass is no longer used by either simulation runtime. Wokwi/C source has not
been compiled or run.

Added `/api/camera-frame.raw?sequence=N`, eight-frame retention, and telemetry
source/sequence fields. Updated API/architecture/TMTC docs, firmware
instructions, camera/simulation subsystem notes, design description,
traceability, CSV schema and scenario evaluation. All 118 Python tests pass;
`compileall` passes; the deterministic model report was regenerated with the
rendered-image detector in the closed-loop scenarios. A 500-cycle in-memory
environment→detector→controller→plant benchmark measured 4.612 ms mean,
5.414 ms p95, and 9.968 ms maximum against a 20 ms target; this excludes
HTTP/network overhead. An audit found 40 Markdown files and zero broken
relative links. Six static C camera-path checks pass, but no C compiler is
installed. Live HTTP/browser, ESP-IDF build, Wokwi execution, and hardware
remain unverified. Official Wokwi gateway and ESP-IDF HTTP streaming sources
are recorded in `references/Hardware_and_Emulation_Notes.md`.

One preview edge case was also closed: if firmware switches from a recent
simulated preview to the physical camera profile before the first real image
arrives, the bridge clears the simulated image instead of briefly presenting
it as current physical data.

## Simulated camera sample freshness (2026-10-16)

Previously, host SIL treated the fetched rendered frame as age zero even if
observation/image transfer or processing took long enough to make it stale.
Host SIL now measures camera age from cycle request start through frame
detection; Wokwi C measures from cycle request start through frame fetch and C
detection. Both feed the existing 100 ms sample-age guard, which returns zero
control torque for stale, invalid, or future-dated observations. Added
regressions for a 101 ms-old rendered frame and a future sample timestamp; both
verify the controller faults closed with zero torque.

The full suite now passes 118 tests, `compileall` passes, and model results
were regenerated. A fresh 500-cycle in-memory timing check measured 4.612 ms
mean / 5.414 ms p95 / 9.968 ms max versus the 20 ms cycle budget; this excludes
HTTP and Wi-Fi/gateway delay. The C freshness change is source reviewed but
still cannot be compiled/run without ESP-IDF/Wokwi; live sockets remain denied.

## Simulation control UI failure handling (2026-10-16)

Changed the Sim Environment form to read each input by explicit element ID
instead of relying on browser named-element globals. Control submissions now
show server validation failures and report when the simulation link is
unavailable, without leaving an unhandled rejected promise. Added a served-UI
handler check for target, torque, and occlusion fields and link-failure copy.

The complete documented Python suite passes 111 tests and `compileall` passes.
This verifies the HTML is served with the expected control wiring, not actual
browser execution. A current tool recheck found no Node/other JS runtime,
browser, Selenium/Playwright, or ESP-IDF executable here. Continue static UI
review and Python integration work; live UI/network and firmware checks need an
environment with those tools and loopback sockets enabled.

## Status consistency refresh (2026-10-16)

Reran the exact nine-module test command in `outputs/Python_sim/README.md`:
118 tests passed at that checkpoint. Python `compileall` and scenario-report regeneration also
passed in this session. Corrected the requirements matrix and existing status
report, which still said 103 tests and gave the older 0.740 s occlusion result;
the current model result is 0.760 s after event timing was aligned to simulation
time. Updated the report's project state to reflect resumed simulation work.
No code changed. Live multi-process communication remains the main simulation
verification gate because socket creation is denied in this environment.

## Simulated camera transport budget (2026-10-16; superseded)

This was the initial raw-frame audit. The current RLE1 transport, size limits,
and verification evidence are recorded in the later **Bounded camera-frame
transport checkpoint** below.

At that audit, source review found the default Wokwi firmware fetched one 320x240 8-bit
grayscale frame (76,800 bytes) for every 20 ms cycle. This is 3.84 MB/s or
30.72 Mbit/s of payload before HTTP/gateway overhead; host SIL requests the
same size at 50 Hz. The implementation/data format is intact, but the existing
in-memory benchmark does not measure transfer or Wokwi deadline behavior.
Recorded this as an explicit camera verification gate in the simulator API,
system architecture, camera subsystem note, and requirements matrix. No
runtime-rate claim is made. Next, measure sustained gateway throughput,
latency, frame expiry and 20 ms cycle misses in Wokwi. If the 100 ms freshness
limit is missed, adjust the simulated camera update rate or frame encoding
while keeping image detection onboard.

## Host SIL / firmware command parity (2026-10-16)

Compared supervisory command validation in `CubeSat_ESP32/sil.py` with
`firmware/main/cubesat_main.c`. Python used a generic reason for an invalid
`SET_MODE` value and accepted extra speed-test argument fields; firmware
reported `INVALID_MODE` and required exactly `target_rev_s` plus `duration_s`.
Aligned Python behavior and added regressions for invalid modes, extra speed
fields, and yaw-step abort. Also fixed firmware yaw-step
target telemetry to report JSON `null` before a request and after SAFE/STANDBY
abort, matching the Python reference instead of exposing its zero-initialized
value as a real target. Updated the API description for rejection reasons and
null/final-target behavior.

Verification: targeted control tests and the complete discovered Python suite
pass (121 tests); `compileall` passes. The C telemetry change is source-reviewed
only because ESP-IDF and a host C compiler remain unavailable. Continue with a
command-by-command parity audit and build both C and ESP-IDF profiles in a
working toolchain.

## Yaw-step stale-estimate parity (2026-10-16)

Further state-machine review found host SIL continued integrating an IMU sample
whose age exceeded 100 ms, while firmware rejected it and moved an active yaw
step to SAFE. Host SIL now refuses to update the estimator from invalid/stale
IMU input and transitions the active request to SAFE with
`ATTITUDE_INVALID`. Firmware now clears yaw-step target validity on invalid
attitude and timeout, serializes absent target/error values as JSON null, and
checks timeout before producing a current error, matching Python semantics.
Updated the API and TM/TC ICD with the common fail-safe/status behavior.

Verification: 29 focused control tests and all 123 discovered Python tests pass;
`compileall` and deterministic scenario-report regeneration pass. The ESP-IDF
source changes have not been compiled. Next,
continue parity review and run C/firmware verification in a working toolchain;
live bridge throughput/network checks remain unverified here.

## Python/C controller guard parity (2026-10-16)

Compared `CubeSat_ESP32/control.py` with the portable
`firmware/main/yaw_controller.c`. Python had a fixed 20 rad/s rate guard while
the C API exposes a configurable `max_abs_rate_rad_s`; the defaults matched,
but custom configuration could not be compared. Added the same parameter to
Python with the existing 20 rad/s default. C previously checked malformed
sensor values before range-invalid controller settings, unlike Python; moved
its range validation earlier so both report `INVALID_CONFIG` first. Added
Python regression coverage and corresponding portable C test cases for
configurable/inclusive rate limits and fault precedence.

Verification: 31 focused control tests and all 125 discovered Python tests
pass; `compileall` and scenario-report regeneration pass. Host C test sources
were updated but cannot be compiled in this WSL environment because no host C
compiler is installed. ESP-IDF, live networking, Wokwi and hardware are still
unverified.

## Non-finite acceleration estimator parity (2026-10-16)

The estimator comparison found Python integrated gyro rate even when an
accelerometer component was NaN/Inf, then could label the estimate valid as
GYRO_ONLY. The portable C estimator rejects the whole sample and leaves its
state unchanged. Python now matches C; added regression coverage in Python and
the portable C estimator test source. The sensor subsystem note records that
an active yaw step enters SAFE when attitude becomes invalid.

Verification: 37 focused attitude/control tests and all 126 discovered Python
tests pass; `compileall` and deterministic scenario-report regeneration pass.
The C test source and firmware remain uncompiled because no host compiler or
ESP-IDF toolchain is available here.

## Python/C detector input parity (2026-10-16)

Compared the Python centroid detector's runtime validation with the portable C
API. C rejects a maximum dark-pixel count larger than the image and its
`uint8_t` threshold cannot exceed 255; Python previously accepted both cases.
Python now requires integer arguments, an 8-bit threshold, and area bounds
within the image. Added a Python regression and a corresponding portable C
oversized-area test; documented the initial threshold-40 / 50–200 pixel range
as provisional camera settings.

Verification: 42 focused attitude/control/camera tests and all 127 discovered
Python tests pass; `compileall` and deterministic scenario-report regeneration
pass. C tests remain source-level only without a compiler.

## Standalone wheel model input guards (2026-10-16)

The yaw plant already validated its integration step and torque inputs, but
`ReactionWheel` can also be called directly and had no equivalent guards. Both
wheel stepping methods now require a finite step in (0, 0.1] s; torque stepping
also requires a finite requested body torque. This keeps standalone model use
consistent with the plant contract. No wheel dynamics or calibration values
changed.

Verification: all 128 Python tests pass, including direct invalid-input cases;
`compileall` and deterministic scenario-report regeneration pass. Firmware and
cross-process verification limits are unchanged.

## Rendered camera validity parity (2026-10-16)

An edge-case sweep found that environment metadata could call a target valid
from ideal projected area even when its actual rendered, clipped disk had fewer
than the onboard detector's 50-pixel minimum. Camera reference validity now
uses the dark-pixel count in the exact retained frame, and invalid but visible
small/oversized targets remain present in the raw image. The disk renderer now
fills row spans, keeping large visible targets bounded without a per-pixel
Python inner loop. Updated the API, camera subsystem and traceability notes;
added edge-clipped, undersized, and oversized frame/detector agreement tests.

Verification: 56 focused environment/model tests and all 130 Python tests pass;
`compileall` and scenario-report regeneration pass. Pointing, hold, nudge,
moving-target, occlusion, yaw-estimator and yaw-step model results are unchanged.
Centroid bias remains possible for edge-clipped targets that still pass the
area limit. C detector build and live browser/Wokwi behavior remain unverified.

## Camera edge-policy regression review (2026-10-16)

A trial change rejected every dark mask touching a frame boundary to avoid
centroid bias. Full scenario verification exposed that the 30° acquisition
scenario intentionally begins at the nominal camera FOV boundary, so this
policy invalidated its target and the controller (which has no search maneuver)
never acquired it. Reverted border rejection consistently in Python, portable
C, and environment reference validity. Kept the raster-area validity change:
small/oversized/clipped-below-minimum targets remain in the raw image while
invalid. A clipped target that still meets the area threshold may bias the
centroid; edge reacquisition or shape fitting is future work.

Verification after the revert: all 130 discovered Python tests pass; Python
`compileall` and deterministic scenario evaluation pass. The seven documented
model scenarios pass, including 30° acquisition in 1.600 s. Firmware C remains
uncompiled; live networking/browser, Wokwi, and hardware checks remain
unavailable in this session.

Next: continue simulation work with in-memory subsystem/integration checks and
interface review. Before claiming end-to-end runtime validation, a socket-enabled
host is needed to run the three processes independently. Before firmware claims,
a host C compiler and ESP-IDF/Wokwi environment are needed. No new user input is
needed for ongoing model and Python-side implementation; board revision,
sensor orientation/calibration, motor/wheel parameters, and physical safety
confirmation are needed before hardware-specific validation.

## Camera projection consistency checkpoint (2026-10-16)

Code review found an inconsistency in the synthetic camera: center position
used pinhole projection, but apparent target diameter used Euclidean slant
range. For the explicitly assumed camera-facing circular marker, changed image
size to use optical-axis depth (`focal_length × marker diameter / camera_z`),
and safely render no marker when the target is at or behind the camera plane.
Documented that this is a billboard/circular-marker approximation, not an
oriented poster model. Added regressions for equal target size at equal depth
when centered/off-axis and for a target behind the body camera.

Verification: 48 environment tests and all 132 discovered Python tests pass;
`compileall` and deterministic model evaluation pass. Existing seven scenario
outcomes remain passing. This improves geometric consistency inside the
synthetic camera model; it does not validate the physical lens, poster
orientation, or calibration. Continue with interface review and model coverage;
live sockets, browser rendering, C compilation, Wokwi, and hardware remain
unverified.

## Bounded camera-frame transport checkpoint (2026-10-16)

Replaced 76,800-byte-per-cycle image transfers in host SIL and default Wokwi
source with lossless RLE1 encoding of the exact synthetic grayscale raster.
The wire body is `RLE1` plus little-endian 16-bit run lengths and grayscale
bytes; both clients validate frame sequence/dimensions and reconstruct all
76,800 pixels before their existing detectors run. The uncompressed endpoint
remains available for inspection. Python and portable C decoders reject
malformed runs, wrong output size, and payloads above the shared 8,192-byte
limit. The default image encodes to 91 bytes (36.4 kbit/s payload at 50 Hz);
a 60-case depth/bearing/size sweep had a 1,447-byte maximum, versus raw
30.72 Mbit/s. Reproducible scene-size and host encode/decode measurements are
in `outputs/Verification/Camera_Transport_Results.md`; timing is regenerated
per run and excludes HTTP, Wi-Fi, and Wokwi gateway time.

Verification: the documented unit command and full discovery both pass all
145 Python tests; the in-memory RLE1 bridge → SIL detector → matched-preview
path also passes as a single integration test. `compileall` and deterministic
model evaluation pass. The new portable C decoder tests are source-only because
neither GCC nor Clang is installed. Wokwi throughput/latency/deadlines, live
sockets, and physical camera
performance remain unverified. Next, compile and compare the C decoder with
Python-generated RLE1 frames in a host C toolchain, then measure the actual
Wokwi cycle time and payload delivery.

## Ground Station sensor freshness display (2026-10-16)

The Ground Station previously rendered `target_valid` and `imu_valid` without
applying the onboard controller's 100 ms sample-age limit. It now labels camera
and IMU samples stale when their reported age exceeds 100 ms, shows the measured
age, and inserts a gap in the target-error plot unless both sensor streams are
fresh. Updated the Ground Station subsystem note and Python_sim README.

Verification: all 146 discovered Python tests pass, including a served-dashboard
regression for the freshness threshold, stale label, and plot gap. `compileall`
and deterministic scenario evaluation pass. This checks the served HTML source,
not JavaScript execution in a browser. A direct Python loopback socket bind still
fails with `PermissionError: Operation not permitted`; browser rendering and
live process communication remain unverified. The Markdown audit found 41 files
and zero broken relative links. Next, continue offline integration and
model/interface review, then run the live dashboard with the three processes in
a socket-enabled environment.

## Three-axis passive-mode verification (2026-10-16)

Added an independent-input regression proving X, Y, and Z disturbance torques
drive pitch, yaw, and roll respectively without cross-axis response in the
decoupled plant. Added a 20 s no-external-torque free-decay scenario for the
linear pendular roll/pitch modes and included their final angles and mechanical
energy retention in `outputs/Verification/Simulation_Results.md`. Both axes
retain 13.978% of their initial energy with the present provisional parameters;
the starting angles and duration are a model check, not an acceptance target.
Added SIM-01 to the requirements matrix and updated the Simulation Environment
note and project README.

Verification: the documented test suite and full discovery both pass all 150
tests; Python `compileall` and deterministic model report generation pass. This
does not validate physical gimbal dynamics, axis coupling, or tuned damping.
Next, continue model/interface review and work on the live-process gate in a
socket-enabled environment when available.

## Tilt-compensated yaw sensor consistency check (2026-10-16)

Added a five-pose grid with combined roll, pitch, and yaw to check the
environment's gravity/magnetic vectors against the host yaw estimator. The test
also checks vector magnitude invariance and the model's gyro X/Y/Z mapping.
Updated the simulation-results report, sensor subsystem note, PERF-07 evidence,
and report README. The ideal synthetic vectors and estimator share the same
assumed fixed magnetic-field reference, so the 0.000000° reported maximum error
is only a coordinate/sign consistency result; it is not an independent sensor
accuracy result.

Verification: the documented module suite and full discovery both pass all 150
tests; Python `compileall` and deterministic report generation pass. The
software check does not cover magnetic calibration, field disturbances, or
physical sensor mounting. Continue with source/runtime interface review; live
networking, ESP-IDF, Wokwi, and hardware gates remain open. A post-update Markdown audit checked 107 project Markdown
files and found zero broken relative links.

## Tilted camera projection consistency (2026-10-16)

Added an independent matrix-based check of target world-to-camera projection
for five combined roll/pitch/yaw poses. Both camera-plane X and Y errors match
the simulator's rounded pixel coordinates exactly. Added a reproducible
projection-grid result to `Simulation_Results.md`, SIM-02 traceability, and
camera/sensor documentation. This verifies the software transform and sign
conventions only; focal length is provisional and lens distortion, mounting,
target orientation, and camera calibration are not represented.

Verification: the documented unit suite and full discovery both pass all 152
tests; `compileall` and deterministic report generation pass. The camera
geometry result is not physical camera validation. Live sockets, browser,
ESP-IDF, Wokwi, and hardware remain unverified.

## Yaw-only tilted pointing limitation (2026-10-16)

Added a reproducible six-second SIL diagnostic with a centered world target,
combined initial body attitude, and brief X/Z disturbance torques. Across 300
valid camera samples the yaw controller reduces horizontal error from -107 px
to -6 px (-1.24°), while 43 px (8.82°) of vertical error remains because the
current controller does not command pitch or roll. The report explicitly
labels this a limitation diagnostic, not a passed acceptance criterion.

Verification: the documented suite and full discovery both pass all 153 tests;
`compileall` and deterministic report generation pass. Live sockets, browser,
ESP-IDF, Wokwi, and physical gimbal behavior remain unverified.

## Synthetic camera detector field-of-view sweep (2026-10-16)

Added a reproducible SIL evaluator and regression for the onboard Python
detector using the default 0.1 m marker at 2 m depth over 13 horizontal
bearings (-30° to +30°, 5° increments). The detector marks all 13 frames valid;
maximum centroid discrepancy from the environment's analytic target position
is 2.91 px at the clipped field-of-view boundary. The simulation and camera
notes state that this checks ideal scene consistency only, and that other
target-size/range combinations can be filtered by the 50–200 dark-pixel bounds.

Verification: the documented suite and full discovery both pass all 154 tests;
`compileall` and deterministic scenario report generation pass. This does not
measure real-camera detection rate, latency, or Wokwi transfer deadlines.

Also compared `xiao_camera.c` pin assignments with Seeed's current camera
pin table. XCLK, SCCB, all eight DVP data lines, PCLK, VSYNC, and HREF agree.
This confirms the documented board mapping only; camera module revision and
ESP-IDF capture/build behavior still need a kit check.

## Yaw model parameter sensitivity (2026-10-16)

Added a direct yaw controller/plant corner sweep. Yaw inertia, passive damping,
wheel torque, wheel motor lag, and both controller gains are independently set
to 0.5× or 2× nominal, producing 32 combinations. All 32 meet the proposed
30° acquisition and 20° nudge timing criteria; worst times are 3.08 s and
2.48 s. The sweep uses perfect target-bearing/rate measurements, so it does not
show camera, sensor, firmware-scheduling, or hardware robustness.

Verification: the documented suite and full discovery pass all 155 tests;
`compileall` and scenario report generation pass. Model assumptions remain
provisional and need measured values from the physical rig.
The user-facing status report now includes an explicitly approximate 65%
simulation-work estimate and a component/evidence tree separating implemented,
tested, and still-unverified paths.

## SIL observation noise and latency screen (2026-10-16)

Added a 54-case screen through the Python onboard observation/control path and
yaw plant. It varies camera bearing-error noise (0/5/10 px RMS), gyro bias
(-0.03/0/+0.03 rad/s), additional gyro noise (0/0.01 rad/s RMS), and matched
camera/IMU observation delay (0/40/80 ms). All cases meet acquisition and
nudge timing; 53 meet the 60 s ±2° hold criterion. The single miss is 2.025°
under the strongest combined stress. Camera noise is injected after detection,
so this does not exercise noisy image pixels. Stress values are not measured
hardware properties and the result is not an acceptance judgment.

Verification: the documented suite and full discovery pass all 156 tests;
`compileall` and deterministic report generation pass. Use measured sensor and
transport properties to select realistic stress levels before retuning gains.


## Current continuation checkpoint (2026-10-16)

Expanded the deterministic SIL noise/latency screen to 81 cases by adding the
MPU-6050 datasheet 0.05°/s RMS gyro-noise comparison point. Current observed
baseline result is acquisition 81/81, nudge 81/81, hold 80/81; the sole miss is
2.034° with 10 px RMS camera-error noise, -0.03 rad/s bias, 0.01 rad/s RMS gyro
noise, and 40 ms matched delay. The reproducible eight-point gain sweep finds seven candidates pass; best tested
Kp=0.14/Kd=0.05 reaches 1.943° worst hold, while Kp=0.10/Kd=0.05 misses once
at 2.018°. A separate 50-seed critical-corner repeat passes 49/50 for baseline
and candidate; the candidate has lower median but worse p95 and worst error.
Configured gains remain at baseline until measurements support a hardware-
relevant change. Firmware MPU sample-time estimation now accounts
for configured DLPF delay and output phase. A new in-memory closed-loop acquisition
regression carries the sequence-paired RLE1 QVGA frame through the SIL
decoder and detector/controller, plant actuator handler, and Ground Station
telemetry handler. The acquisition criterion uses detector-derived camera
pixel error converted to angle plus plant truth; both enter the 2° bound in
under 6 s while torque stays bounded. Two static UI source checks pass,
confirming unique DOM IDs, valid JavaScript element lookups, and that each UI
endpoint appears in its matching HTTP handler; browser rendering remains
unverified. A direct plant launch was retried and again failed opening
its listening socket with `PermissionError: Operation not permitted`. The full
documented suite and discovery now pass 173 tests. New static checks confirm
the Wokwi project/ELF paths, target board and PSRAM config, Wi-Fi defaults,
and camera component declaration are consistent; they do not verify a Wokwi
boot. `compileall` passes, the
model report is regenerated, and the 43-file relative-link audit has zero
broken links. The firmware C change remains uncompiled. WSL lacks a Linux
ESP-IDF/compiler, and invoking the mounted Windows ESP-IDF through `cmd.exe`
fails before startup with `UtilBindVsockAnyPort: socket failed 1`; the old
`Python_sim/firmware/build/` artifacts point at the former Windows OneDrive
checkout and are not evidence for the current firmware. There is no immediate
user action needed to continue simulation work; exact kit/breakout details and
sensor measurements are needed later for hardware-specific calibration and
acceptance.


## Pixel-domain camera noise checkpoint (2026-10-16, superseded)

The initial synthetic raster-noise screen used a global thresholded centroid and
found false detections in blank frames at σ=50. The follow-on checkpoint below
adds a matched local-coherence filter to Python and C and supersedes those
initial result figures. The evaluator and updated results remain in
`outputs/Python_sim/Verification/evaluate_camera_image_noise.py` and
`outputs/Verification/Camera_Image_Noise_Results.md`.


## Noise-resistant target detector checkpoint (2026-10-16)

At the three-seed checkpoint, the pixel-noise study exposed that a global
thresholded centroid could interpret random dark pixels as a target. Python SIL
and the C firmware detector now apply
the same local-coherence rule: retain a dark pixel only when at least two other
dark pixels occur in its 8-neighbourhood, then enforce the existing area bounds
and compute the centroid. Added Python and portable-C source regressions for
isolated speckles and a coherent square target. The regenerated noise screen
The original three-seed screen reported 21/21 targets through assumed σ=40 and
18/21 at σ=50. The later 30-seed results are recorded in the latest checkpoint
below and supersede these sample counts. This is
not full connected-component segmentation and has not been evaluated against
recorded OV5640 images. A host-only timing sample of 30 clean QVGA Python detector calls measured 2.052 ms median, 2.477 ms p95, and 2.980 ms maximum in this WSL session; this excludes RLE decoding, HTTP, scheduling, and all ESP32 timing.

Verification: the focused detector/noise checks pass (11 tests), full unittest
discovery passes 175 tests, `compileall` passes, and the deterministic scenario,
image-noise, and camera-geometry reports were regenerated. The C regression was updated but
cannot be compiled in this WSL environment. Next compare Python/C detector outputs
using a host compiler when available and test the rule against recorded camera
frames.


## Camera size/range and two-axis projection checkpoint (2026-10-16)

`outputs/Python_sim/Verification/evaluate_camera_geometry.py` now sweeps 4
target diameters × 4 depths × 3 horizontal × 3 vertical bearings (144 rendered
QVGA images). It runs the onboard Python detector, checks validity against the
environment reference, and records raw/coherent areas and both detector error
axes in `outputs/Verification/Camera_Geometry_Results.md`. The Y comparison
accounts for environment-up versus image-row-down coordinates. With the assumed
277 px focal length and 50–200 coherent-pixel limits, the default 0.1 m target
at 2 m is accepted in all 9 tested horizontal/vertical bearing pairs. Five
additional body-pose frames for a fixed world target match analytic camera X/Y
metadata with zero pixel discrepancy at each sampled yaw/pitch/roll pose. The
sweep uses an ideal camera-facing disk and background; it does not establish a
physical camera operating envelope or calibration.

Verification: all four focused camera geometry regressions pass; the documented
module suite and full discovery each pass 177 tests; `compileall` passes and the
geometry report regenerates. The 43-file Markdown link audit has zero broken
relative links. ESP-IDF, Wokwi, live-network, browser, and hardware gates remain
open. Next validate the detector against recorded camera frames and measured
intrinsics, and run the portable C host-test runner in a C11-enabled environment.


## Camera image-noise sample expansion (2026-10-16)

Raised the reproducible pixel-domain noise study from 3 to 30 deterministic
seeds per bearing (210 target frames and 30 blank frames per noise level). The
updated report records 210/210 detections through σ=30 (maximum centroid change
0.457 px), 209/210 at σ=40 (0.769 px max), and 191/210 at σ=50 (1.153 px max),
with 0/30 blank-scene detections at σ=40 and σ=50. These remain synthetic
assumptions, and 30 blank trials are not a statistical guarantee or camera
qualification.

Verification after regeneration: the documented module suite and full unittest
discovery each pass all 177 tests; `compileall` passes; Markdown audit checks 43
files with zero broken relative links. Report generation completed successfully.


## Current simulated-project checkpoint (2026-10-16)

- Expanded the deterministic pixel-domain camera noise screen to 30 seeds per
  bearing. The generated report and requirements traceability now agree: at the
  assumed noise levels, target detection is 210/210 through σ=30, 209/210 at
  σ=40, and 191/210 at σ=50. Blank-scene detections are 0/30 at σ=40 and 50.
  These are synthetic sensitivity results, not camera qualification.
- Verification on this checkpoint: documented module suite 177/177; full
  discovery 177/177; `compileall` completed; Markdown relative-link audit checked
  43 files with zero broken links. `compileall` emitted two SyntaxWarnings from
  vendored `pyparsing` files inside `.venv/Lib/site-packages`; it exited
  successfully.
- No project input or decision is needed to continue the Python simulation and
  documentation work. External validation still needs a C11 compiler and ESP-IDF
  environment, plus working local sockets/browser for live integration. Actual
  board and sensor revisions and safe test constraints are required before
  hardware calibration or actuation. See “Main gaps and next actions” above.


## Python camera detector timing checkpoint (2026-10-16)

The Python QVGA detector now uses row bitsets to evaluate its 8-neighbour
coherence rule. A deterministic differential regression compares exact output
with a direct-neighbour reference across randomized frames and strides; a dense
frame test checks full area-count preservation. A 300-case development probe
matched exactly. The documented suite and full discovery each pass 183 tests;
`compileall` passes; the Markdown audit checks 44 files with zero broken relative
links. The reproducible 50-call host timing report measures about
2.0 ms median for ordinary frames and 5.9 ms median / 10.6 ms maximum for an
all-dark frame, compared with the nominal 20 ms Python SIL cycle. This is host
Python detector-only evidence, not ESP32 or end-to-end timing. See
`outputs/Verification/Camera_Detector_Timing_Results.md`.


## Bridge protocol-version validation (2026-10-16)

Offline firmware/SIL review found the C bridge client compared only cJSON's
integer conversion of `protocol_version`, which could accept a fractional number
that truncates to 1. The C check now also requires `valuedouble == 1.0`. Python
SIL now explicitly rejects booleans and nonnumeric versions while accepting
numeric 1/1.0, matching the C value semantics. The TM/TC ICD documents the rule.
A Python regression exercises boolean, fractional, unsupported, string, and
integral-float cases; environment bridge tests cover both command and actuator
ingress; a C source contract guards the version condition. The documented suite
and full discovery both pass 183 tests; targeted `compileall` passes; Markdown
audit checks 44 files with zero broken relative links. C compilation remains
unavailable.


Toolchain recheck for protocol-validation work: no Linux `cc`, GCC, Clang,
`idf.py`, or Wokwi CLI is on PATH. The mounted Windows `idf.py.exe` helper is
present, but WSL invocation still fails with `UtilBindVsockAnyPort`; it cannot
run the build here. This confirms the C condition is source-reviewed only, not
compiled.


## ESP-IDF control-task stack budget (2026-10-16)

Review of the firmware's blocking HTTP cycle found it runs inside ESP-IDF's
`main` task and keeps a 2048-byte response body on the stack. Official ESP-IDF
v6.1 documentation lists 3584 bytes as the default main-task stack and
recommends using the runtime high-water mark after peak-use paths. The project
now sets `CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` and emits a debug-only
`uxTaskGetStackHighWaterMark(NULL)` reading every 250 cycles (5 s). The value is
reported in bytes on ESP32. This is a provisional allocation, not measured
adequacy. The documented suite and full discovery each pass 183 tests; targeted
`compileall` passes; Markdown audit checks 44 files with zero broken relative
links. Firmware build and runtime stack margin remain unverified because ESP-IDF
and WSL interop are unavailable. Before accepting this budget, build and run the
Wokwi and physical-camera paths under worst-case frame and transfer load, then
record free-stack margin.


## Consolidated offline verification runner (2026-10-16)

Added executable `outputs/Python_sim/run_verification.sh` as the repeatable
project entry point. It performs full Python test discovery, syntax compilation,
regenerates the model and four camera reports, and runs portable C and ESP-IDF
checks when their tools are available. Its first full execution completed with
183 Python tests passing, compilation passing, all five reports regenerated, and
explicit skips for C and ESP-IDF because there is no C compiler or `idf.py` on
PATH. Shell syntax passed. It does not claim live sockets, browser rendering,
Wokwi execution, or hardware checks. The C detection honors the `CC` environment
variable.

## Session pause checkpoint (2026-10-16)

The user requested a full status report and paused continued project execution.
The report at `outputs/reports/AE_Project_Status_and_Run_Guide.md` now includes
the latest capability check: no host C compiler or `idf.py` is on PATH, loopback
socket creation is denied (`PermissionError: Operation not permitted`), and no
Playwright/Selenium/Pyppeteer package is installed. A verification-runner
attempt was interrupted during the Python test phase; treat it as incomplete,
not as a pass, and retain the last completed evidence checkpoint of 183 Python
tests passing. No project change or new verification result was produced by the
interrupted run. Offline simulation work can resume without user input. Live
network/browser checks need a socket-enabled environment; portable C and
firmware checks need their toolchains. Continue from the runner/report review
when the user resumes the project.

## First test-drive release checkpoint (2026-10-16)

Prepared the current Python SIL state for the user's first live trial. Added
`outputs/Python_sim/FIRST_TEST_DRIVE.md`, three Windows `.bat` launchers, and a
PowerShell shortcut installer that creates three desktop icons. Updated the AE
and software READMEs and the status report with launcher instructions and the
test sequence. The launchers assume a WSL distribution named `Ubuntu` and the
documented project path; change `WSL_DISTRO` in each batch file if needed.

Removed generated Python bytecode caches, the ignored `.venv` whose metadata
points to a former Windows OneDrive checkout, and the ignored root `firmware/build`
cache that also points to the old checkout. Source, supplied inputs, and current
`CubeSat_ESP32/firmware/` sources were retained. A native `.exe` was not built;
Windows interop fails in this session and no packager is available. Scripts are
prepared but not run; the session also denies loopback sockets. After cleanup,
`./run_verification.sh` passed all 183 Python tests and syntax compilation,
regenerated all five model/camera reports, and explicitly skipped C and ESP-IDF
checks because their toolchains are missing. This is a prepared first-test-drive
checkpoint, not a verified live launch.

The goal is paused here for the user to try the first test drive. On resume,
start with `outputs/Python_sim/FIRST_TEST_DRIVE.md`, confirm launcher/WSL access,
and record observed issues before making the next implementation changes.

Final preparation checks for this checkpoint: `./run_verification.sh` passed
183/183 Python tests, Python syntax compilation, and regeneration of all five
model/camera reports. Portable C and ESP-IDF steps were skipped for missing
toolchains. A static launcher command review passed for all three modules;
`bash -n run_verification.sh` passed; Markdown relative-link audit checked 33
files with zero broken links. Generated `__pycache__` directories were removed
after verification. Live Windows launch, browser, socket, C, ESP-IDF, Wokwi and
hardware behavior remain unverified.

## First launch troubleshooting checkpoint (2026-10-16)

The user reports both browser URLs refuse connections. Since port 8001 serves
the Ground Station independently of the simulator bridge, both failures point
to component startup or Windows-to-WSL localhost forwarding; the actual cause
needs the terminal output. Updated each Windows launcher to preflight WSL and
Python, run a small WSL shell wrapper, and keep the window open when startup
fails. Added direct-terminal instructions and `Test-NetConnection` checks to
`outputs/Python_sim/FIRST_TEST_DRIVE.md`. Shell syntax checks passed for all
three wrappers. The Windows launch path still cannot be executed from this
session. Next request the visible terminal error and the two PowerShell
`TcpTestSucceeded` results; then distinguish startup failure from forwarding.

Added `Launchers/Diagnose_Connections.bat` and its PowerShell/Python helpers.
With the three services running, the diagnostic checks WSL-local TCP/HTTP and
listener state, Windows listener state, and Windows localhost connectivity;
it writes `AE_Connection_Diagnostic.txt` to the Windows desktop. Python syntax,
all three Bash wrapper syntax checks, and a 33-file relative-link audit pass.
The Windows diagnostic itself cannot be run in this Codex environment. Ask the
user to run the diagnostic and share the saved report before attempting further
network changes.

## Persistent terminal window adjustment (2026-10-16)

The user reports the service terminal windows close around the time the browser
opens. This can happen when the WSL process exits after a bind/startup error;
the exact cause is unconfirmed. Changed each Windows launcher to run its WSL
process inside `cmd.exe /k`, which should keep the visible terminal open at a
Windows prompt after WSL exits or after the service is stopped. Updated the
first-test-drive guide to explain this. The Windows command-line behavior cannot
be executed in this Codex session; ask the user to retry and report what remains
visible in each window.

## Direct startup attempt from Codex session (2026-10-16)

Attempted each documented module in its own PTY. `Sim_Environment.main` and
`Ground_Station.main` both exited at `ThreadingHTTPServer` socket creation with
`PermissionError: [Errno 1] Operation not permitted`. `CubeSat_ESP32.sil` started
but its bridge HTTP requests failed with the same permission error; it was
stopped after a two-second sample to avoid continuous cycle-error output. No
processes or terminals remain open. This confirms this Codex execution sandbox
cannot host the live system; it does not establish what happens in the user's
Windows/WSL terminals. The agent cannot open or keep GUI terminals on the user's
desktop. User must run the launchers or direct WSL commands and share the
visible errors/results to continue diagnosis.

## First live UI feedback and comparison controls (2026-10-16)

While the user's three processes were running, the shared Ground Station CSV
grew from 366 KB to 378 KB over three seconds; the sample was still updating.
The latest inspected rows had fresh valid IMU data, valid MAG_FUSED yaw, no
reported fault, and `RATE_DAMP`, while the target was invalid throughout the
last 100 logged samples. Its last valid sample was around simulation time
87.42 s. If the user did not intentionally change/occlude the target, investigate
why the camera detector no longer sees it. This session cannot inspect the
user's browser or query the live ports directly.

The user reports that Ground Station values do not match the environment and
that the scene cannot be orbited/zoomed. Source review found the environment
uses a fixed-view schematic canvas with no mouse controls. The Ground Station
shows onboard camera/sensor measurements, while the environment shows plant
truth; these values can differ by design, but that distinction was not visible.
Updated the environment canvas with drag-orbit, wheel zoom, and double-click
reset. Added a separately labeled simulator-truth comparison in Ground Station
using the existing `/api/state` proxy, plus user-facing README/first-test-drive
notes. Both HTML pages are served from disk on page request; user should hard
refresh both browser tabs to load the changes. No tests or live browser check
were run in this session. Ask user whether the camera interaction works and
which values still appear inconsistent; inspect the CSV's new samples after
that report.

### No-input baseline observed (2026-10-16)

The user restarted the three components and reports the no-input baseline is
running. The shared CSV has appended rows in a fresh run; the latest sample
inspected is at simulation time 86.47 s, sequence 4003. It reports TRACK /
NOMINAL, valid camera and IMU, valid MAG_FUSED yaw estimate (0.00099 rad),
camera X error -1 px (bearing error -0.00361 rad, about -0.207°), camera age
9.6 ms, IMU age 0, and no fault. Requested body torque is -0.000295 N·m while
simulated actual body torque is -0.000251 N·m, consistent with the model's
wheel response lag. This is a healthy near-centred baseline sample, not a
complete end-to-end acceptance result. Next ask the user to shift target X to
0.20 m (keep Y=0, Z=2 m and diameter=0.10 m), apply once, and capture early
camera/target error and later settling behavior from the UI and telemetry.

The user reports X movement tracks correctly, but changing target size makes
the Ground Station mark the target invalid. During this report, fresh CSV rows
show `target_valid=False`, `RATE_DAMP`, `system_state=NOMINAL`, camera age about
15 ms, valid IMU/yaw and no fault. The CSV does not include target scale or
detector pixel count, so it cannot prove which size caused this result. Source
inspection shows both the synthetic camera visibility gate and SIL detector
accept only 50–200 dark/coherent pixels. At the default 0.10 m diameter and 2 m
depth, the projected disk is about 13.9 px wide and ~150 px area. Smaller or
larger targets can therefore be rendered by the environment but rejected by
the onboard detector; exact boundary depends on rasterization. For a clean
size sweep, restore target X=0, Y=0, Z=2 m and test diameter 0.06, 0.08, 0.10,
and 0.12 m one at a time, checking target validity after each apply. Record the
exact size and validity. A known telemetry-quality caveat is that SIL serializes
invalid X/Y camera errors as 0.0; only interpret those fields when
`target_valid=True`.

## Full-direction host simulation update (2026-10-17)

The user requested control on all three simulated axes after observing that a
target at Y=0.5 m did not move the cube, while retaining the scene plane and
adding a body reference frame at the cube origin. Implemented in host SIL:
camera horizontal error drives yaw as before; vertical camera error drives
bounded virtual pitch torque; accelerometer-derived gravity tilt drives
bounded virtual roll leveling. A point target cannot observe roll about the
camera boresight. The bridge validates pitch/roll virtual torques (±0.02 N·m)
and accepts them only from `cubesat_sil`; no mapping to physical pitch/roll
actuators or ESP-IDF firmware was added. The environment UI draws origin-
attached body axes X red/pitch, Y green/yaw, Z blue/roll-camera boresight and
retains the ground plane and orbit/zoom controls. Ground Station displays
camera Y error, tilt estimates, requested/actual virtual torques, and
simulator-truth X/Y camera error. CSV logging stores the virtual-axis command
as JSON.

The (0, 0.5, 2 m) ten-second host integration produced pitch −14.14°,
simulator-truth vertical error 0.102°, horizontal error −0.104°, roll −0.020°;
the final target remained valid with zero detected vertical pixel error. This
is deterministic host simulation evidence only. The first full verification
run exposed three test/report compatibility issues; code, fixtures and the
full-direction evaluation were updated, and focused regressions passed. The
complete `./run_verification.sh` rerun passed 187/187 Python tests and syntax
compilation, regenerated all five simulation/camera reports, and explicitly
skipped portable C and ESP-IDF checks because their toolchains are unavailable.
It does not test live sockets or browser rendering.

Next: once verification finishes, the user should stop and restart the three
Python components in the documented order to load server changes, hard refresh
both browser tabs, set target `(X=0, Y=0.5, Z=2 m)` with diameter `0.10 m`, and
check target validity, camera Y pixel error, pitch response, simulated pitch
torque, simulator-truth Y error, and roll. Live user-side browser/Windows
behavior cannot be observed from this WSL session. The physical design still
needs a pitch/roll actuator decision before hardware firmware can control those
axes.

### Roll disturbance recovery tuning (2026-10-17)

The user reports that the simulated yaw response settles quickly after Y-axis
torque, while roll continues tumbling for too long after Z-axis torque. Raised
host-SIL roll PD gains from Kp=0.08/Kd=0.04 to Kp=0.4 N·m/rad and
Kd=0.3 N·m·s/rad; pitch gains and its ±0.02 N·m cap are unchanged. Raised the
roll-only virtual torque cap from ±0.02 to ±0.05 N·m. The environment bridge
enforces separate axis bounds and still accepts virtual torques only from
`cubesat_sil`.

A deterministic host test applies Z torque 0.05 N·m for 0.5 s, verifies peak
roll remains below 5°, then verifies it holds within 1° and 0.5°/s for 0.5 s
within 2 simulated seconds after the pulse. A parameter probe with the previous
gains and ±0.02 N·m cap took about 3.47 s on the same disturbance; the tuned
gains and stronger virtual roll authority take about 1.74 s. With a 0.02 N·m
pulse, recovery takes about 1.06 s. These are host simulation measurements,
not hardware values. Focused regressions pass. `./run_verification.sh`
completed successfully: 190/190 Python tests and syntax compilation passed;
all five simulation/camera reports regenerated. Portable C and ESP-IDF checks
were skipped because the toolchains are unavailable. Live network/browser
checks remain unverified.

Next user-side check after verification: restart all three processes, apply
`external_torque_nm.z=0.05` for `duration_s=0.5`, and observe simulator roll
and applied virtual roll torque. The full instructions are in
`outputs/Python_sim/FIRST_TEST_DRIVE.md`. If the user's pulse magnitude or
duration differs, capture those exact values and the roll angle/rate trace.

### Yaw IMU damping and moderated stabilization (2026-10-17)

The user clarified that yaw must continue stabilizing from its IMU rate when a
Y-axis disturbance moves the target outside camera view, and requested a
middle-strength setting after the prior roll tuning felt too strong. Host SIL
now has a separate lost-target yaw rate gain of 0.06 N·m·s/rad (1.5× the
visual-tracking D gain of 0.04). Roll gains are Kp=0.24 and Kd=0.19 with
±0.035 N·m virtual authority, halfway between the prior and raised settings.
Pitch remains unchanged. A 0.1 N·m Y pulse for 0.5 s moves the target out of
frame; the model brings yaw rate below 0.02 rad/s within 4.5 s of target loss.
A 0.05 N·m, 0.5 s Z pulse peaks below 11° and meets the 1° / 0.5°/s settle
band within 3 s after the pulse. The moderated gains allow more roll transient
than the previous high-gain tuning.

`./run_verification.sh` passes 191 Python tests and syntax compilation and
regenerates the five model/camera reports. Portable C and ESP-IDF checks are
skipped because their toolchains are unavailable. This verifies only host
Python code and model behavior. The current user test sequence is
`outputs/Python_sim/SECOND_TEST_DRIVE.md`; restart all three services before
testing so the controller changes load. Live networking, browser rendering,
firmware, Wokwi, and physical hardware remain unverified.

## Version snapshots for test-drive comparison

Two self-contained folder snapshots are stored under
`outputs/Python_sim/NR/`: `Test_drive_1/` captures the earlier first full-direction
state (190-test checkpoint, common yaw damping gain and stronger roll tuning),
and `Test_drive_2/` captures the current second-drive state (191-test
checkpoint, separate lost-target yaw damping and moderated roll tuning). Each
includes its own project inputs, supplied source PDFs, source code, design and
verification outputs, README files, and context handoff. Start at each folder's
README, then run its three services from its own `outputs/Python_sim/` folder.
Generated telemetry logs, caches, and `.git` are intentionally excluded; logs
are regenerated independently per snapshot. These are folder copies, not Git
branches or independent repositories. See `outputs/Python_sim/NR/README.md`.

## Resumed goal: camera XYZ verification checkpoint (2026-10-17)

The user resumed the same CubeSat goal with priorities to verify all
stabilization axes, verify camera direction on all axes, and follow the project
workflow. Read the workspace README, this handoff, and `inputs/Prompts.txt`
before implementation.

The optional simulator control `camera_roll_marker_enabled` is false by
default, preserving the original point-target scenario. When enabled, the
renderer adds an asymmetric three-dot marker to sequence-paired QVGA frames.
The detector estimates its directed image angle using second/third central
moments; featureless point frames return no roll cue. Host SIL selects the
camera roll estimate before IMU roll leveling. Ground Station telemetry, CSV,
and UI show marker angle, estimated body roll, and reference source.

Portable C implements the same moment-based detector and bounded pitch/roll
controller. ESP32 virtual-axis torques are accepted by the scene only when the
firmware reports `camera_frame_source=simulated`; physical camera firmware
cannot inject ideal plant torques. Physical motor actuation remains yaw-only.
Focused marker, point-fallback, source-contract, and three-axis tests pass; the
full runner passes 199 Python tests. The axis report measures 8.32 s to return
from 15° roll to ±1°/±0.5°/s with accelerometer roll hidden. The roll marker is
visible only within a 24° horizontal and 20° vertical inner field to avoid edge
clipping; camera yaw/pitch pointing still passes with the marker enabled.
Compiler, ESP-IDF, Wokwi, live sockets, browser, physical poster, and hardware
gates remain open.

### Wokwi setup audit (2026-10-17)

Rechecked the firmware Wokwi configuration against current official Wokwi
ESP-IDF, project-config, and Wi-Fi documentation. `wokwi.toml` points at
`build/flasher_args.json` and `build/cube_firmware.elf`; the CMake project is
`cube_firmware`, sdkconfig targets ESP32-S3, and the diagram selects
`board-xiao-esp32-s3` with octal PSRAM. These static paths/settings agree with
the documented setup. This is not a successful build or simulator boot.

Clarified `CubeSat_ESP32/firmware/README.md` with workspace/config selection,
the build-before-simulate step, and the required startup chain: run the local
plant, enable Wokwi's private IoT Gateway, then start the Wokwi simulator and
check serial/plant updates. The default firmware bridge address is
`host.wokwi.internal:8000`; current Wokwi docs say host access needs the private
gateway and identify that gateway as a paid-plan feature. Also corrected the
firmware README's stale claim that Wokwi fetched raw frames: the default path
fetches RLE1 and reconstructs the grayscale raster before C detection.
Authoritative links and these constraints are recorded in
`references/Hardware_and_Emulation_Notes.md`.

No compiler, ESP-IDF toolchain, Wokwi CLI, or Wokwi runtime is available in
this session, so the firmware has not been built or run. Next concrete gate:
build in a functioning ESP-IDF 6.1 environment (or supported 5.3+ toolchain),
then use the documented Wokwi workflow and verify Wi-Fi bridge cycles, frame
sequence handling, and telemetry/actuator updates. The `idf.py wokwi` shortcut
specifically requires ESP-IDF 6.0+ plus `idf-wokwi`; the VS Code extension path
is the documented alternative.

### Portable C camera-angle regression expansion (2026-10-17)

Source review found `test_black_target.c` checked a directed fiducial at only
one angle. Expanded that test to render and detect five angles (-30°, -15°, 0°,
+15°, +30°) and assert signed angle accuracy within 0.05 rad plus centered
centroid error. `run_host_tests.sh` already includes this detector test, so it
will run automatically when a C11 compiler becomes available. The source was
not compiled here because no host C compiler is installed and Windows process
launch from WSL still fails with `UtilBindVsockAnyPort`. This improves future
regression coverage but does not upgrade C/ESP-IDF verification status.
Focused Python checks for three-axis control, firmware source contracts, and
static emulation config pass 10/10; these do not compile the C test.

After the source/test changes, reran `./run_verification.sh`: 199/199 Python
tests pass, `compileall` passes, and all six model/camera/axis reports
regenerated successfully. The runner explicitly skipped C host tests (`cc` not
found) and ESP-IDF build (`idf.py` not found). Live sockets, browser, Wokwi,
and physical hardware were not exercised.

The same source review found the C firmware enabled simulator virtual tilt
control when the frame-source string merely began with `s`. Tightened this to
an exact `simulated` string match. Strengthened the source contract to assert
the exact simulation gate, fresh camera/IMU and non-fault requirements, and
camera-marker-first roll reference with IMU fallback. Re-ran the focused suite:
10/10 passed. This is stronger source-level regression evidence only; the C
application still needs compilation and runtime verification.
Updated the CAM-02 row in `outputs/Verification/Requirements_Traceability.md`
to record the exact firmware virtual-axis gate and distinguish the new C test
source coverage from executed test evidence.

### Bridge source-label rejection coverage (2026-10-17)

Expanded `Sim_Environment/test_environment.py` so its ESP32 virtual-torque
boundary test accepts only exact `camera_frame_source="simulated"`; it also
asserts rejection of `physical`, `simulated-extra`, `unavailable`, and a missing
source. The targeted bridge plus firmware-contract checks pass 5/5. A fresh
`./run_verification.sh` run passes 199/199 tests, Python syntax compilation,
and all six report generators. C and ESP-IDF remain explicit skips due missing
`cc` and `idf.py`; live sockets, browser, Wokwi, and hardware remain untested.

Audited the saved request for the next full report: it needs a simulated-part
progress estimate and a compact tree matching `inputs/Prompts.txt`. The status
report already contained a 65% qualitative estimate but its tree omitted the
prompt's named camera, sensor, control, actuation, and telemetry branches and
mixed implementation with verification labels. Reworked that section to map
the full requested architecture, list named hardware, and distinguish Python
tests from C/ESP-IDF source-only and unverified runtime states. Kept 65% as an
explicit rough planning estimate (not a test score) because host runtime and
ESP32/Wokwi execution remain major open milestones. Marked the old Oct 16
launcher checkpoint as historical/superseded.

Added assertions to the existing combined-attitude synthetic-sensor test so
the onboard pitch/roll telemetry estimates must match plant truth at five
roll/pitch/yaw combinations, in addition to the existing gravity norm and yaw
checks. The targeted test passes. The full `./run_verification.sh` then passes
199/199 tests, Python syntax compilation, and regenerates all six reports;
portable C and ESP-IDF remain skipped for missing `cc`/`idf.py`. Updated the
SIM-01 traceability evidence. This verifies the ideal simulated sensor-frame
mapping, not the physical IMU mounting or dynamic-acceleration response.

Reviewed the portable C image path for bounded memory use. Added a 640×480
maximum decoded raster bound to RLE1 and capped detector row stride at 640
bytes, preventing excessive output requests and stride multiplication from
escaping supported image dimensions. Added C test cases for oversized decoded
pixel requests (including no partial write) and oversized stride; added Python
source-contract checks for both guards. Focused checks pass, then the full
runner passes 199/199 tests, syntax compilation, and all six report generators.
The C host test and ESP-IDF build remain skipped because no compiler/toolchain is
available; the new C boundary tests are source-only evidence until executed.


## 2026-10-17 — Validate C camera buffer length contract

Extended `cube_detect_black_target` to receive the accessible grayscale buffer
length and reject frames smaller than `stride * height`. Updated both firmware
callers to pass known lengths (`sizeof(simulated_camera_frame)` and
`frame->len`), and added a truncated-buffer case to the portable C test source.
The focused source-contract suite passes 4/4. The full verification runner
passes 199/199 Python tests, `compileall`, and all six report generators. It
skips C host tests because `cc` is unavailable and skips ESP-IDF because
`idf.py` is unavailable. The new C test remains unexecuted; no firmware, live
network, browser, Wokwi, or hardware verification is claimed.
Synchronized the frame-length boundary description in the camera subsystem and
CubeSat README. A targeted audit found no broken relative links or carriage
returns in the six edited Markdown files. Next: run
`CubeSat_ESP32/firmware/run_host_tests.sh` when a C11 compiler is available,
then continue the all-axis firmware/Wokwi validation gates.


## 2026-10-17 — Check detector API call consistency

Added a source-contract check that scans every C declaration and call to
`cube_detect_black_target` under the firmware tree and verifies the seven-
argument API (including buffer length). The focused source-contract suite
passes 5/5. A fresh `./run_verification.sh` passes 200/200 Python tests,
`compileall`, and all six report generators; C host tests skip because no C11
compiler is installed, and ESP-IDF skips because `idf.py` is absent. Common
compiler/parser binaries and Python C-parser packages were also checked and not
found. This source-level check does not replace a C build. Next: run the C host
suite when a compiler becomes available, then continue firmware/Wokwi and live
network/browser verification.


## 2026-10-17 — Make browser launch wait for service readiness

Replaced the environment and Ground Station launchers' fixed three-second delay
with `Launchers/Wait_For_Service.ps1`. Each now polls the matching Windows
localhost URL for up to 45 seconds, checks HTTP 200 and the expected page title,
and opens the browser only when that service responds. Timeout leaves a visible
error and pause; the service terminal still runs under `cmd.exe /k`. The focused
launcher/Wokwi config tests pass 3/3, and all three WSL shell wrappers pass
`bash -n`. Full verification passes 201/201 Python tests, `compileall`, and all
six report generators; C host and ESP-IDF steps skip because `cc` and `idf.py`
are unavailable. Windows batch/PowerShell behavior and localhost forwarding
remain unverified in this Linux session. Next: test the three launchers on the
user's Windows/WSL desktop, then continue live data-flow and emulator gates.


## 2026-10-17 — Make SIL bridge startup resilient and legible

Host SIL previously printed “connected” before its first request and emitted an
error every failed cycle while the simulation bridge was unavailable. It now
announces that it is waiting, retries connection-reset and HTTP protocol
failures with bounded exponential backoff from 50 to 500 ms, rate-limits
repeated error lines to five-second intervals, and reports connection/recovery
only after a successful complete cycle. The normal
loop remains at its 50 Hz schedule after successful exchanges. Added regressions
for backoff and startup recovery; focused retry, launcher, and firmware-contract
tests pass 10/10. The full runner passes 203/203 Python tests, `compileall`, and
all six reports. C and ESP-IDF skip because `cc` and `idf.py` are absent. Live
network, Windows launcher, Wokwi, and hardware behavior remain unverified.


## 2026-10-17 — Correct ESP32 camera-pitch sign parity

A source comparison found that `black_target.c` reports raster Y as positive
down, but `cubesat_main.c` negated that detector offset when converting it to
the pitch-control bearing. Host SIL converts its environment positive-up value
back to the equivalent positive-down detector sign, so the C camera pitch path
was reversed relative to the reference. Removed the extra negation. A firmware
source contract now asserts `atanf(target_error_y_px / focal_length_px)`; the
existing C pitch/roll test covers the positive-error-to-positive-torque
controller sign. Focused firmware-contract and host-controller checks pass
44/44. The full runner passes 203/203 Python tests, syntax compilation, and all
six reports. C/ESP-IDF remain skipped for missing tools, so the changed firmware
behavior is source-checked, not compiled or run. Updated camera/control
subsystem, architecture, ICD, traceability, report and AI log. Next: compile C,
then run camera-above/below-center tests in the ESP32/Wokwi path and compare
telemetry and virtual pitch torque to the host SIL.


## 2026-10-17 — Guard accelerometer tilt formula parity

Compared C firmware and host SIL pitch/roll estimates against the environment's
synthetic accelerometer equations. Both use pitch `atan2(-az, hypot(ax, ay))`,
roll `atan2(ax, ay)`, and the same 0.8–1.2 g validity band. Added source
contract assertions for both formulas and bounds; focused firmware source
contracts pass 5/5. This complements the existing five-pose SIL truth regression
but does not execute the C estimator or validate physical mounting/dynamic
acceleration. Updated Sensor subsystem notes and SIM-01 traceability. The latest
full runner remains 203/203 tests, with C/ESP-IDF tools absent.


## 2026-10-17 — Guard C/Python pitch-roll controller parity

Added a source-contract test that extracts all nine portable C pitch/roll
controller defaults and compares them with the Python SIL reference. It also
normalizes and compares both axis torque equations. The focused firmware source
contract suite passes 6/6. Full verification passes 204/204 Python tests,
`compileall`, and all six reports; portable C and ESP-IDF are still skipped
because `cc` and `idf.py` are absent. This guards source parity but does not
replace executing C. Earlier in this audit, the detector's directed-marker
moment formulas and thresholds were also manually compared and found equivalent
to Python, with the existing signed-angle C test source covering ±30°.


## 2026-10-17 — Guard C/Python yaw controller parity

Added a source-contract test comparing all six portable C yaw-controller
defaults with the Python reference and checking the tracking and lost-target
damping equations. Focused firmware contracts pass 7/7. The full runner passes
205/205 Python tests, `compileall`, and all six report generators; C host tests
and ESP-IDF remain skipped because `cc` and `idf.py` are unavailable. Alongside
the pitch/roll parity test, this now guards source-level controller parameters
and equations for all three axes, but does not replace running the C suite or
firmware. Added the SIL startup-retry test module to the focused unittest
commands in both the software and AE project READMEs.


## 2026-10-17 — Align yaw-step terminal-status precedence

Compared C and Python yaw-step state machines. At the exact command deadline,
if attitude is invalid, C reports `ATTITUDE_INVALID` while Python previously
reported `TIMEOUT`; both entered SAFE, but the terminal reason differed.
Changed Python to check attitude validity before timeout, matching firmware.
Added a deterministic simultaneous-condition regression and source-contract
checks for order, 10 s deadline, ±1°/±0.5°/s settle limits, and one-second
dwell. Focused checks pass 9/9. The full runner passes 207/207 Python tests,
`compileall`, and all six reports; C/ESP-IDF remain skipped because `cc` and
`idf.py` are unavailable. C behavior remains source-reviewed only.


## 2026-10-17 — Verify ordered one-command delivery

The ICD already specified one pending telecommand per onboard cycle, and the
bridge implementation returns the oldest unexpired command. Added a regression
that queues two distinct commands and verifies the first cycle offers only the
first ID and the next cycle offers the second. The focused test passes; the full
runner passes 208/208 Python tests, syntax compilation, and all six generated
reports. Portable C and ESP-IDF checks skip because `cc` and `idf.py` are not
available. Live command delivery over sockets remains unverified.


## 2026-10-17 — Verify combined camera control across all axes

Added an integrated host-SIL test that starts at 10° yaw, −8° pitch, and 12°
roll with an off-axis target and the optional asymmetric marker enabled. The
loop simultaneously centers camera X/Y and corrects roll from the marker; all
three criteria remain in band for 0.5 s starting at 1.99 simulated seconds.
At 10 s, image error is 0.147°, roll is −0.203°, and roll rate is −0.016°/s.
The focused axis suite passes 5/5; full verification passes 209/209 Python
tests, syntax compilation, and all six reports. Firmware C/ESP-IDF remain
uncompiled, and this combined test is host-SIL evidence only.


## 2026-10-17 — Verify ESP32 virtual-axis telemetry routing

Traced firmware telemetry through the bridge into the plant. Added a source
contract checking that C emits `source=cubesat_esp32`, the camera frame source,
and nested pitch/roll torque fields only under the simulator-frame gate. The
bridge regression now advances the plant and confirms those torques produce
pitch and roll rate changes and appear as separate simulated telemetry. The two
focused checks pass; full verification passes 210/210 Python tests, syntax
compilation, and all six reports. The C source contract does not execute
firmware; C/ESP-IDF, Wokwi and live HTTP remain unverified.


## 2026-10-17 — Clarify firmware and verification commands

Audited the current tool PATH: `python3` is available, while `cc`, `gcc`,
`clang`, `idf.py`, `wokwi-cli`, Node.js, and npm are absent. Rechecked the
firmware and emulator configuration and corrected two README gaps: the
ESP-IDF firmware overview now distinguishes yaw-wheel actuation from simulated
pitch/roll inputs, and the software README no longer repeats the portable C
test command description. The last full offline verification remains 210/210
Python tests plus syntax compilation and six generated reports; its C and
ESP-IDF steps skipped for these missing tools. Next runtime gate is to run the
documented C host suite, ESP-IDF build, and Wokwi path in an environment with
those toolchains.


## 2026-10-17 — Make local service bind failures actionable

Direct starts of the environment and Ground Station in this restricted session
both fail to bind ports 8000/8001 with `PermissionError: Operation not
permitted`. Added shared startup handling so each now prints a concise message
that names its port and identifies local socket permission as the issue;
address-in-use errors instead advise stopping the existing service. The
environment binds before launching its model thread, so a denied socket leaves
no background plant thread. Added four regressions for successful server
creation, permission denial, occupied ports, and both service integrations.
Their messages were checked against the actual failures on both ports. The full
runner passes 214/214 Python tests, syntax compilation, and all six reports.
Live TCP, browser rendering, C/ESP-IDF, and Wokwi remain unverified.

## 2026-10-17 follow-on: physical OV5640 autofocus

Added a physical-camera startup check in `xiao_camera.c`: sensor PID selects the path; OV5640 must support Espressif autofocus and report `focused` within 10 s, otherwise the camera is deinitialized and startup fails closed. Other detected camera modules keep fixed focus. Raised the managed `esp32-camera` dependency to `^2.1.8`, added the conditional `esp32-camera` CMake dependency, and enabled `CONFIG_CAMERA_AF_SUPPORT` in defaults. Firmware/camera docs and hardware research notes now explain this behavior; source contracts cover the AF path and dependency. Focused checks pass 12/12; the full suite passes 214/214 tests and Python syntax compilation passes. Five full-runner reports regenerated; its final axis report was rerun successfully after the runner stopped during that step. ESP-IDF/C/Wokwi and physical sensor behavior remain unverified because those tools/hardware are unavailable here.

### Camera source review correction

A follow-up compile-order review found the AF config macro guard appeared before
`sdkconfig.h`; moved the include ahead of the guard and added a source-contract
assertion. The focused source/config suite still passes 12/12; Python syntax
compilation passes. A scan of six updated Markdown files found no broken
relative links. C/ESP-IDF remain unavailable, so this catches ordering at source
level but does not replace a firmware build.

### Concurrent three-axis disturbance verification

Extended `Verification/evaluate_axis_control.py` and its report with a
simultaneous target-lost case: 0.5 s pulses at half-authority on X/pitch,
Y/yaw, and Z/roll are applied together while the camera is occluded. The SIL
produces nonzero control on all three axes and all rates plus pitch/roll angle
settle criteria hold after 2.01 s. Six focused axis tests pass; the full Python
suite passes 215/215 and `compileall` passes. The regenerated evidence is
`outputs/Verification/Axis_Control_Results.md`. This remains host-SIL evidence;
firmware C, Wokwi, live TCP, and physical sensors/actuators are still open.


### Restrict supported physical camera variants

The physical XIAO adapter now explicitly accepts OV5640 (only after successful autofocus) and OV3660 (fixed focus); unknown sensor PIDs are deinitialized and rejected. This avoids silently treating an unverified camera as fixed focus. ESP-IDF source contracts were extended; target compilation and sensor detection still need the ESP-IDF toolchain and board.


### Configurable LIS3MDL measurement range

Added a Kconfig choice for ±4/8/12/16 gauss, retaining ±4 gauss as default. The LIS3MDL driver now writes the matching CTRL_REG2 FS bits and converts each range using ST's LSB/gauss sensitivity values before reporting microtesla. The selected range is logged at startup. Focused source/config checks pass 13/13; full Python discovery passes 216/216 and `compileall` passes. The firmware C path has not compiled because a C/ESP-IDF toolchain is absent. The specific board-level magnetometer and breakout interface still need identification; current firmware uses shared I²C, while SPI is documented as not implemented.


## 2026-10-17 follow-on: report selected LIS3MDL range in telemetry

The physical LIS3MDL range selection is now visible end to end: the driver
exports its configured ±4/8/12/16 G range, ESP-IDF telemetry emits
`mag_full_scale_gauss` (or JSON null when the physical adapter is disabled),
the host synthetic model emits null, and the Ground Station shows the
configured range and records it in CSV. The HTTP API, TM/TC ICD, telemetry
subsystem note, and project status report describe the field. Focused contracts
passed 73/73, full Python discovery passed 217/217, and Python `compileall`
passed. ESP-IDF/C compilation, live browser/TCP, Wokwi, and hardware remain
unverified.


### Verification report refresh and bilateral camera-roll test

The full runner passed 217 tests and `compileall` before stopping during report
generation. I reran the simulation evaluator directly; `Simulation_Results.md`
is fresh for 17 October 2026, not the frozen Test Drive 2 snapshot. Extended
the host-SIL camera-roll evaluation to start at both +15° and −15° while hiding
the accelerometer roll cue. The seven-test axis suite passes, including both
correction signs. `Axis_Control_Results.md` now includes both roll directions. Full discovery
confirms the updated suite count of 218.


### Latest axis verification result

The marker-based camera roll controller now has a bilateral host-SIL check.
From +15° the body settles to 0.152° / −0.073°/s in 8.32 s; from −15° it
settles to −0.417° / −0.013°/s in 9.16 s. In both runs accelerometer roll is
hidden, telemetry selects the CAMERA reference, and torque has the correcting
sign. `Verification.test_axis_control` passes 7/7; full discovery passes
218/218 and `compileall` passes. Updated evidence:
`outputs/Verification/Axis_Control_Results.md`. This proves host-SIL behavior
only; C firmware, Wokwi, live networking, and hardware remain open.

### Setup guide correction

Corrected `outputs/Python_sim/docs/set_up.md`, which still said only yaw was
actively controlled. It now distinguishes the all-axis host-SIL simulation and
simulator-framed ESP32 virtual pitch/roll inputs from the currently yaw-only
physical motor output. This was a documentation-only correction; the 218-test
suite and `compileall` results above are unchanged.

The bilateral axis test fixture now reuses the two evaluated roll cases instead
of rerunning the positive case. Focused verification after this cleanup passes
7/7 in 48.12 s. `compileall` and an updated-file content/whitespace check pass.
Current PATH has none of `cc`, `gcc`, `clang`, `idf.py`, or `wokwi-cli`, so C
execution and ESP-IDF/Wokwi build gates remain open.


### Bidirectional yaw/pitch camera pointing

Added rendered-frame host-SIL cases for ±16.70° yaw/image-X and ±14.03°
pitch/image-Y errors. All four runs retained 1000/1000 valid target frames and
settled below 2° in 2.29 s, 2.34 s, and 1.03 s for the two yaw signs and two
pitch signs. The focused axis suite passes 7/7. Full discovery after the signed-error
assertions passes 218/218 in 137.37 s; `compileall` passes. Updated report:
`outputs/Verification/Axis_Control_Results.md`.

A toolchain search found no Linux `idf.py`, Xtensa ESP32-S3 compiler, GCC, or
Clang under `/home/nunot`, `/opt`, or `/usr/local`; the Windows ESP-IDF 6.1
installation is at `C:\esp\v6.1\esp-idf` with tools under
`C:\Espressif\tools`. Launching its compiler from WSL fails with
`UtilBindVsockAnyPort`. The firmware README now documents a native WSL ESP-IDF
5.3.5 install path and a Windows ESP-IDF PowerShell attempt against the WSL
UNC project path. Neither route is build-verified. Local Markdown links resolve.

### Portable C test-runner documentation correction

Audited `CubeSat_ESP32/firmware/run_host_tests.sh`: it builds and runs four
suites—yaw control, pitch/roll control, attitude estimation, and target
detection. Corrected the firmware README and Python_sim README, which described
only three. Execution remains pending because no compatible C compiler exists
in the current environment.


### Toolchain setup handoff

Reviewed Espressif's official v5.3.5 Linux setup and Wokwi's ESP-IDF project
instructions. Added the version-pinned WSL installation steps and a separate
route for the Windows ESP-IDF 6.1 already installed on this PC. `idf.py` and the
compiler work only inside an activated ESP-IDF terminal; this agent's WSL process
cannot invoke the Windows binaries because WSL interop fails. The Windows
PowerShell build from `\\wsl.localhost\Ubuntu\home\nunot\Mestrado_Codex\AE\outputs\Python_sim\CubeSat_ESP32\firmware` is an unverified route to try. See
`outputs/Python_sim/CubeSat_ESP32/firmware/README.md`.

### Current live-runtime probe

The current sandbox process listing shows no visible
`Sim_Environment.main`, `CubeSat_ESP32.sil`, or `Ground_Station.main` processes.
Requests to `127.0.0.1:8000` and `:8001` fail before connection with
`Operation not permitted` while opening a socket. This only establishes the
sandbox limitation/current sandbox process view; it does not establish whether
services are running in a separate user desktop session or reveal their live
telemetry.

The current WSL Python is 3.14.4; Espressif's v5.3.5 Linux guide states
Python 3.8 or newer, so the interpreter version itself meets that documented
minimum. The firmware build directory currently has no `flasher_args.json`,
`cube_firmware.elf`, or `cube_firmware.bin`; no new firmware build artifact is
visible in the shared worktree.

### Wokwi board memory configuration check (17 October 2026)

Rechecked the XIAO ESP32-S3 diagram against Wokwi's official board docs. Its
8 MB flash, 8 MB octal PSRAM values are within the documented choices. Extended
`Verification/test_emulation_config.py` to guard supported flash and PSRAM
sizes as well as the existing board and PSRAM type assertions. Focused
emulation and firmware-source contract tests pass 14/14. This is a static
configuration check only: no C compiler, ESP-IDF build, or Wokwi boot is
available in this session. Next verification gate remains a real ESP-IDF build,
then a Wokwi boot and bridge exchange in an environment with those tools and
the required private gateway access.

### Full offline verification refresh (17 October 2026)

Reran `outputs/Python_sim/run_verification.sh` to completion on the current
worktree. All 218 Python tests passed in 137 s, syntax compilation passed, and
all six report generators completed. Portable C tests and ESP-IDF build skipped
explicitly because `cc`, `gcc`, `clang`, and `idf.py` are absent from PATH. The
focused axis test also passed 7/7. Updated the operator report's stale note
that the full runner had stopped during report generation. This verifies the
offline host suite and generated reports only; local TCP/browser, firmware
build, Wokwi, and physical behavior remain unverified.

Rechecked Windows-tool access: `cmd.exe /c echo ...` and the installed
`/mnt/c/Espressif/tools/esp-clang/.../clang.exe --version` both fail with
`UtilBindVsockAnyPort`. The direct native-Windows ESP-IDF PowerShell build
instructions are already in `CubeSat_ESP32/firmware/README.md`; use the
activated ESP-IDF PowerShell terminal and its documented WSL UNC project path.
If that build succeeds, continue with portable C tests in a separate compiler
environment and then Wokwi. No firmware build artifact is present yet.

Re-probed loopback by creating, binding, and listening on ports 8000 and 8001.
Socket creation itself fails for both with `PermissionError: [Errno 1]
Operation not permitted`, so a live three-process browser/API test cannot run
from this sandbox. This confirms the environment boundary; it is not a new
application-level port failure.

Audited the output tree against the prompt: all seven subsystem folders exist
with README descriptions, and the simulation, camera, sensing, control,
actuation, telemetry, and Ground Station notes are present. Corrected the
requirements matrix and status report where they still described the earlier
interrupted report-generation run; both now state the latest full runner result
of 218 passing tests, compilation success, six regenerated reports, and C/IDF
toolchain skips.

### GitHub snapshot publication status

The Python_sim repository is `https://github.com/Nunoo-24/Cubsat.git`. Commit
`e398cb19797eb065a299ce8f8222beeab96e1579` ("Add independent test-drive
snapshots") contains both `NR/Test_drive_1/` (125 tracked files) and
`NR/Test_drive_2/` (126 tracked files). The local Git reflog records
`origin/main` as updated by push at 2026-10-17 01:16 +0100, and local `main`
and cached `origin/main` both point to that commit. A new push/reachability
check from this session failed because `github.com` did not resolve; the
current live remote state could not be re-fetched. Many later live-worktree
changes are uncommitted and are not included in that snapshot commit.

### 2D system architecture diagram (17 October 2026)

Created `outputs/Diagrams/AE_CubeSat_System_Architecture.drawio` and its SVG
preview. The editable diagram has Environment/Test Plant, CubeSat onboard, and
Ground Station domain boxes, with labeled cross-domain flows and internal
camera detection, sensor conditioning/estimation, control, actuation, and
telemetry processes. It distinguishes the physical yaw-only moteus path from
simulation-only virtual pitch/roll control and marks unresolved hardware and
protocol details. Updated the AE and software READMEs and design description.
Validated both diagram files as well-formed XML. The draw.io desktop GUI is
installed on Windows, but this WSL session cannot directly control it; open the
editable file from the Windows app to review visual layout. The same diagram
and updated documentation are included in the local Test Drive 2 snapshot;
that addition is not yet on GitHub because the prior push attempt failed on DNS.


### Diagram layout revision (17 October 2026)

Reworked `outputs/Diagrams/AE_CubeSat_System_Architecture.drawio` in response
to the user's layout feedback. The three outer domains retain their positions,
while the CubeSat now has nested camera, sensing, onboard processing, physical
actuation, and telemetry/telecommand domains. Camera acquisition, image
processing, target error, sensor hardware/conditioning, attitude estimation,
guidance, control/safety, motor interface, wheel drive, telemetry packing, and
link handling are separate cards; relevant arrows now attach to those cards.
Increased body and connector label sizes, expanded the page to use available
width, and rotated the vertical `reaction torque / body motion` connector
label. Regenerated the SVG preview and synchronized both diagram files into
`outputs/Python_sim/NR/Test_drive_2/`. XML structure and connector endpoints
were validated; the diagram was not opened in the Windows GUI in this session.

### Draw.io connector-label rotation fix (17 October 2026)

The user reported that the reaction-torque label rotated in the SVG but stayed
horizontal in draw.io. Corrected the editable XML by moving that text into a
rotated child vertex of connector `e14` (`rotation=270`) instead of relying on
an edge style text-rotation key, which draw.io did not apply to the edge label.
The SVG preview already had the rotated text. Mirrored the corrected source to
Test Drive 2 and validated both files parse with the child label attached to
the correct edge. The desktop GUI still needs to be reopened by the user to
confirm the visual result.


### Make connector labels individually movable (17 October 2026)

At the user's request, converted all 29 connector labels into independent
text-box cells on the diagram layer; connector edges themselves now have no
labels. This makes text boxes movable/editable/rotatable separately from the
lines. Long vertical-route labels sit beside the path, and the reaction-torque
label remains rotated. The SVG is retained as a static preview. Both editable
files parse, each connector has preserved endpoints, and all 29 text boxes are
standalone cells.


### Add space between architecture domains and tighten boundaries (17 October 2026)

Responded to the user's spacing feedback by widening the camera and processing
card gutters, increasing the gap between sensor stages, widening the separation
between actuation and telemetry domains, and adding 10–20 px between nested
CubeSat domain rows. Tightened the Environment and Ground Station outer
boundaries to content. Repositioned all 29 standalone connector-text boxes and
refreshed the SVG preview. XML checks confirm all boxes stay within the page,
all connector endpoints remain valid, and the editable source is synchronized
with Test Drive 2.


### User-routed connectors and label layout (17 October 2026)

Using the latest user-edited diagram as the source, repositioned the 29 independent
connector text boxes to improve readability and keep them clear of subsystem
cards where the available routing space allowed. Preserved all 31 connector
cells exactly, including IDs, endpoints, styles, and route waypoints. Kept the
outer domain positions fixed; tightened the CubeSat boundary slightly without
moving its contents, and expanded the page to fit the current layout. Refreshed
the SVG and synchronized the draw.io source and preview to Test Drive 2.

### Latest diagram and label-review copy (17 October 2026)

The latest user-arranged Test Drive 2 diagram has 30 connector cells and 28
independent connector-label boxes. Refreshed its SVG preview from that editable
source. Added `outputs/Diagrams/AE_CubeSat_System_Architecture_version_codex.drawio`
and SVG as a separate label-cleanup comparison copy. All 62 visible text labels
were reviewed and corrected in that copy; cell geometries, styles, and routes
match the latest source. The original editable diagram remains unchanged by the
label cleanup.
