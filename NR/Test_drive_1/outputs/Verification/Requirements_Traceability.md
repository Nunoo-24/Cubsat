# AE project requirements and verification status

This matrix tracks requirements stated in the supplied project guide and the
user's confirmed onboard-computing decision. The guide's performance values are
proposed criteria, not confirmed course acceptance thresholds. Evidence is
classified by what was actually run; source presence alone is not runtime or
hardware verification.

Primary project source: [`../../Source/1-project_guide.pdf`](../../Source/1-project_guide.pdf).
Architecture and assumptions: [`../Design_Description.md`](../Design_Description.md)
and [`../../Context/PROJECT_BRIEF.md`](../../Context/PROJECT_BRIEF.md).

## Functional and integration requirements

| ID | Requirement | Implementation and evidence | Status |
|---|---|---|---|
| SYS-01 | Autonomous sensing and control run onboard; the PC station is supervisory. | Python onboard reference: `../Python_sim/CubeSat_ESP32/sil.py` fetches a sequence-paired QVGA camera frame and runs the centroid detector before control; ESP-IDF app: `../Python_sim/CubeSat_ESP32/firmware/main/cubesat_main.c`; Ground Station sends PING, enumerated modes, bounded wheel-speed tests, and relative yaw-step requests. Python handler/integration tests pass; ESP-IDF target execution remains unverified. The firmware main-task stack is configured to 8192 bytes with a debug high-water report every 5 s; verify its margin on target under full transfer load. | REFERENCE IMPLEMENTED; FIRMWARE UNVERIFIED |
| SYS-02 | Provide separate simulation, CubeSat software, and Ground Station components. | Independent commands are listed in [`../Python_sim/README.md`](../Python_sim/README.md). Python modules and handler-level integration are tested. This sandbox denies socket creation, so live multi-process communication is not demonstrated. | COMMANDS DOCUMENTED; LIVE RUNTIME UNVERIFIED |
| SYS-03 | Define subsystem boundaries and interfaces, including TM/TC. | [`../Python_sim/docs/System_Architecture_and_Interfaces.md`](../Python_sim/docs/System_Architecture_and_Interfaces.md), [`../Python_sim/docs/TMTC-ICD.md`](../Python_sim/docs/TMTC-ICD.md), [`../Python_sim/docs/Simulator_HTTP_API.md`](../Python_sim/docs/Simulator_HTTP_API.md). The versioned HTTP/JSON exchange is for simulation/emulation; a physical radio protocol is not selected. | DOCUMENTED; PHYSICAL LINK OPEN |
| SYS-04 | Organize subsystems and preserve resumable project context. | [`../Subsystems/README.md`](../Subsystems/README.md) indexes simulation, camera, sensor, control, actuation, telemetry, and station notes; [`../../Context/HANDOFF.md`](../../Context/HANDOFF.md) records status and next actions. | DOCUMENTED |
| SIM-01 | Simulate three-axis target pointing: yaw through the reaction-wheel model, host-SIL pitch through vertical camera error, and host-SIL roll leveling from gravity. | `../Python_sim/Sim_Environment/model.py` maps X torque to pitch, Y to yaw, and Z to roll. Host SIL sends bounded virtual pitch/roll torques, accepted only from `cubesat_sil`; its deterministic tilted-pointing case centers camera X/Y errors and levels roll. These axis controls are simulation-only; physical firmware and actuator support remain yaw-only. A separate 20 s free-decay case checks plant restoring dynamics with no control. | HOST-SIL IMPLEMENTED AND TESTED; VIRTUAL PITCH/ROLL; PHYSICAL ACTUATION OPEN |
| SIM-02 | Project world target coordinates into camera pixels using the simulated three-axis body attitude. | Environment target pixel coordinates match an independently assembled rigid-body rotation matrix at five combined roll/pitch/yaw poses, with zero rounded-pixel discrepancy; tests cover both X and Y. Camera focal length, lens distortion, mounting, and target orientation remain idealized. | SOFTWARE GEOMETRY CHECK PASS; PHYSICAL OPTICS UNVERIFIED |
| CAM-01 | Process a simulated or physical camera image onboard to locate the dark target and provide pointing error. | Environment retains raw QVGA frames by observation sequence; SIL/Wokwi fetch the exact image via bounded RLE1 and reconstruct the raster before onboard Python/C detection. Both implementations apply an 8-neighbour coherence rule before centroiding; the detector returns total thresholded area and coherent area separately. An in-memory bridge → SIL decoder/detector → telemetry-matched preview test exercises this Python path end to end. A separate ideal-scene detector sweep yields 13/13 valid samples for the default 0.1 m marker at 2 m over bearings -30° to +30°; clipped-edge centroid discrepancy reaches 2.91 px. This does not measure hardware detection rate or latency. A 60-case depth/bearing/size sweep round-trips exactly (maximum encoded size 1,447 bytes); default frame is 91 bytes; API cap is 8,192 bytes. See [camera transport results](Camera_Transport_Results.md). A separate 144-frame size/depth/horizontal/vertical-bearing sweep plus five combined body-pose raster checks verify renderer/detector validity parity and both image-error signs, and report the synthetic detector envelope in [camera geometry results](Camera_Geometry_Results.md). Noisy-camera compression and Wokwi throughput/deadlines remain unmeasured. | PYTHON SIL IMPLEMENTED AND TESTED; C/WOKWI SOURCE IMPLEMENTED, BUILD/RUN AND TRANSFER RATE UNVERIFIED; HARDWARE UNVERIFIED |

**Host detector timing:** the Python QVGA detector was optimized with row bitsets. Its output matches the former direct-neighbour implementation across 300 randomized frame, dimension, and stride cases, and a retained differential regression checks this behavior. The [50-call timing study](Camera_Detector_Timing_Results.md) measured about 2.0 ms median for ordinary synthetic frames and 5.9 ms median / 10.6 ms maximum for an all-dark frame on this WSL host. This excludes transport and controller work and is not ESP32 or full-cycle timing evidence.

**Synthetic image-noise sensitivity:** additive Gaussian intensity noise is
applied to QVGA raster pixels before target detection (not to the already
measured bearing error). The detector applies an 8-neighbour coherence rule before centroiding. With
30 deterministic seeds per bearing, assumed σ≤30 gray levels retain 210/210
target detections (maximum centroid change 0.457 px); σ=40 retains 209/210
(maximum 0.769 px), and σ=50 retains 191/210 (maximum 1.153 px). Blank-scene
false detections are 0/30 at σ=40 and 50. These are synthetic sensitivity
samples, not camera specifications or a statistical guarantee. The rule is not
full connected-component segmentation. See [camera image-noise results](Camera_Image_Noise_Results.md).

## Performance milestones from the guide

| ID | Proposed criterion | Current result | Evidence | Status |
|---|---|---|---|---|
| PERF-01 | Acquire from 30° error to ≤2° in <6 s. | 1.600 s in the deterministic Python model. A separate direct yaw-control/plant corner screen meets the timing limit in all 32 combinations of 0.5×/2× yaw inertia, damping, wheel torque, motor lag, and controller gains; worst case is 3.08 s. That screen uses perfect measurements and excludes camera/noise/firmware. Initial target is at the synthetic camera's ±30° horizontal field-of-view boundary. | [`Simulation_Results.md`](Simulation_Results.md); scenarios in `../Python_sim/CubeSat_ESP32/test_control.py` and `../Python_sim/Verification/evaluate_scenarios.py`. | NOMINAL MODEL AND BOUNDED PARAMETER SCREEN PASS; EDGE-OF-FOV; END-TO-END ROBUSTNESS UNVERIFIED |
| PERF-02 | Hold within ±2° for 60 s. | Nominal deterministic peak error is 0.107°. A separate 81-case observation-level stress screen (camera-error noise 0/5/10 px RMS, gyro bias −0.03/0/+0.03 rad/s, gyro noise 0/0.000873/0.01 rad/s RMS, and 0/40/80 ms matched delay) meets the hold limit in 80/81 cases; the remaining case reaches 2.034° at 40 ms delay. The 0.000873 rad/s point corresponds to 0.05°/s from the MPU-6050 datasheet at DLPF_CFG=2; noise and delay levels are not calibrated measurements. A reproducible eight-point gain sweep finds seven settings pass all 81 cases; the best tested Kp=0.14 N·m/rad, Kd=0.05 N·m·s/rad has a 1.943° worst hold peak. The Kp=0.10/Kd=0.05 setting misses one case at 2.018°. The best candidate margin remains small and inputs are assumed, so this does not support selecting hardware gains. A 50-seed repeat at the critical combined-stress corner yields 49/50 holds for both baseline and Kp=0.14/Kd=0.05; the candidate slightly lowers median error but raises p95 and maximum error. This is one-corner seed sensitivity only. Camera noise is injected after centroid measurement, not into pixels. | [`Simulation_Results.md`](Simulation_Results.md); nominal scenarios in `../Python_sim/CubeSat_ESP32/test_control.py`; stress evaluator in `../Python_sim/Verification/evaluate_scenarios.py`. | NOMINAL MODEL PASS; BASELINE STRESS SCREEN HAS ONE MARGINAL MISS; GAIN SENSITIVITY GRID RECORDED; NO CALIBRATED TUNING; HARDWARE LIMITS UNCALIBRATED |
| PERF-03 | Recover from an approximately 20° disturbance within 4 s. | 1.340 s in the deterministic Python model. The separate 32-corner direct yaw-control/plant screen also meets the timing limit at every tested combination; worst case is 2.48 s with perfect measurements. | [`Simulation_Results.md`](Simulation_Results.md); scenarios in `../Python_sim/CubeSat_ESP32/test_control.py` and `../Python_sim/Verification/evaluate_scenarios.py`. | NOMINAL MODEL AND BOUNDED PARAMETER SCREEN PASS; END-TO-END ROBUSTNESS UNVERIFIED |
| PERF-04 | Track a slowly moving target. | At the selected 0.2°/s for 60 s, model peak error is 0.119°. The guide does not specify a rate or error bound. | [`Simulation_Results.md`](Simulation_Results.md); scenarios in `../Python_sim/CubeSat_ESP32/test_control.py`. | MODEL CASE PASS; ACCEPTANCE LIMIT TBD |
| PERF-05 | Recover after 2 s camera occlusion without instability. | Recovery to ≤2° in 0.760 s after occlusion; peak yaw rate during occlusion 0.0011 rad/s. The occlusion interval is measured on simulation time. | [`Simulation_Results.md`](Simulation_Results.md); scenarios in `../Python_sim/CubeSat_ESP32/test_control.py`. | MODEL PASS |
| PERF-06 | Reach 3000 rpm in <0.5 s with <2% error. | The bounded speed-test path is integrated in Ground Station, bridge, host SIL, and source-level ESP-IDF firmware. The commissioning command uses SAFE/STANDBY gating and timed stopping; the physical profile also requires the arm input. The placeholder plant reaches 431 rpm in 0.5 s; ideal average torque for its 1e-4 kg·m² inertia is 62.8 mN·m versus the model's 10 mN·m limit. The physical native-velocity frame uses the same Kconfig max-torque value as torque mode: 1 mN·m default and 20 mN·m maximum. At the placeholder 1e-4 kg·m² inertia, that maximum gives an ideal 955 rpm in 0.5 s; the 62.8 mN·m requirement exceeds the software cap. With 20 mN·m, inertia must be ≤3.18e-5 kg·m² before losses; this is a model boundary, not a measured wheel property. | [`Simulation_Results.md`](Simulation_Results.md); integration test in `../Python_sim/Verification/test_speed_test_integration.py`; model in `../Python_sim/Sim_Environment/model.py`; driver API in `../Python_sim/CubeSat_ESP32/firmware/main/moteus_uart.c`; command path in `../Python_sim/CubeSat_ESP32/firmware/main/cubesat_main.c`. | SOFTWARE PATH IMPLEMENTED; PYTHON TEST PASS; FIRMWARE UNCOMPILED; PERFORMANCE NOT VERIFIED |
| PERF-07 | Attitude-estimation error <2° and drift <2°/min. | The one-axis yaw estimator is compared against simulator truth for 60 s using biased/noisy synthetic gyro and the fixed synthetic magnetic field: 0.068° peak error and 0.050° final drift. A separate ideal-vector grid checks yaw recovery at five combined roll/pitch/yaw poses; maximum error is 0.000001° (the exact tolerance is only a coordinate-consistency check). This does not estimate roll/pitch, or evaluate calibration error, magnetic disturbances, firmware, or hardware. The estimate is used by the explicit relative yaw-step mode, while regular visual pointing still uses camera error. | [`Simulation_Results.md`](Simulation_Results.md); implementation in `../Python_sim/CubeSat_ESP32/attitude.py` and `../Python_sim/CubeSat_ESP32/firmware/main/attitude_estimator.c`; tests in `../Python_sim/CubeSat_ESP32/test_attitude.py` and `../Python_sim/Sim_Environment/test_environment.py`. | YAW MODEL PASS; FULL ATTITUDE/HARDWARE NOT VERIFIED |
| PERF-08 | A 45° attitude step settles within ±1°. | `SET_YAW_STEP` is integrated in Python SIL and source-level ESP-IDF firmware, relative to onboard yaw estimate and independent of camera field of view. Model entered the ±1° / ≤0.5°/s band at 5.820 s and completed its 1 s dwell at 6.820 s. | [`Simulation_Results.md`](Simulation_Results.md); `../Python_sim/CubeSat_ESP32/sil.py`; in-memory HTTP-handler integration in `../Python_sim/Verification/test_speed_test_integration.py`; firmware command/control in `../Python_sim/CubeSat_ESP32/firmware/main/cubesat_main.c`. | PYTHON MODEL AND HTTP-HANDLER INTEGRATION PASS; FIRMWARE UNCOMPILED; HARDWARE UNVERIFIED |

## Verification gates still required

1. **Python live runtime:** start all three commands in separate terminals,
   confirm observation, actuator, command acknowledgement, and telemetry flow,
   and verify CSV output in a socket-enabled environment. Handler tests do not
   replace this check.
2. **Wokwi transfer budget:** measure QVGA frame throughput, latency, frame
   expiry, and missed 20 ms cycle deadlines through the private gateway; verify
   that camera freshness remains below 100 ms under continuous operation.
3. **ESP32 firmware:** build default and physical camera/IMU profiles from
   `../Python_sim/CubeSat_ESP32/firmware/` using a supported ESP-IDF release.
   Current sources have Kconfig/source review only; the retained old build cache
   is from a different checkout.
4. **Emulation:** run the current XIAO ESP32-S3 Wokwi project and demonstrate
   sensor cycles, commands, telemetry, reconnect, and watchdog behavior.
5. **Hardware:** confirm board, camera, sensors, motor, wheel, controller, and
   gimbal; calibrate sensor frames and validate electrical interfaces,
   torque/speed limits, and safe-stop before powered tests.
6. **Guide milestones still needing evidence:** test the implemented native
   velocity path on a disconnected and then safely powered actuator; extend
   the yaw-only estimator to full attitude only if required by the rig; build and evaluate the implemented 45° angular-step command. Define the moving-target
   rate/error acceptance for PERF-04 with the project team.

The full Python command and measured model results are in
[`../Python_sim/README.md`](../Python_sim/README.md) and
[`Simulation_Results.md`](Simulation_Results.md). The latest recorded suite
   result is 191 passing tests. ESP-IDF compilation, live TCP, browser execution,
Wokwi, and hardware behavior remain unverified in this environment.
