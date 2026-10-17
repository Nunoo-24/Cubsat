# Verification outputs

`Simulation_Results.md` reports deterministic, model-level pointing, one-axis
yaw-estimator, commanded yaw-step, and passive roll/pitch free-decay results.
It also checks yaw recovery at five combined roll/pitch/yaw poses using the
simulator's ideal sensor vectors and compares camera X/Y projection at five
tilted poses with an independently assembled rotation matrix. Guide performance
limits are proposed criteria; both grids and the passive-axis case are software
consistency checks under provisional assumptions. It also sweeps the default
synthetic marker across 13 horizontal bearings from -30° to +30° and compares
the onboard detector with the environment's analytic target location. This
checks ideal renderer/detector consistency only; the clipped boundary mask can
bias the centroid by up to about 3 px, and the sweep says nothing about physical
detection rate or frame timing.
These are simulation results; they do not demonstrate ESP32 firmware execution,
live networking, sensor calibration, actuator behavior, or physical performance.
The model uses simulation-clock torque/occlusion events, including proportional
integration when a disturbance pulse ends during a plant step. The yaw estimate
passes against the synthetic calibrated magnetic reference,
but this does not validate three-axis attitude, sensor calibration, firmware,
or hardware. The report lists the 3000 rpm milestone as a performance fail
against the placeholder model despite an implemented commissioning command,
and the 45° step passes in the Python model (5.820 s to enter the settle band,
6.820 s to complete the dwell). ESP-IDF compilation and physical response
remain unverified. A socket-free HTTP-handler integration test also carries the
`SET_YAW_STEP` command through to the onboard reference and modeled plant; it
does not prove live network behavior. The 30° acquisition case is at
the synthetic camera's field-of-view edge; the model does not acquire targets
outside that field.

The evaluator also computes a best-case wheel spin-up bound using the current
placeholder torque, inertia, and lag. That bound shows the current model cannot
meet the 3000 rpm / 0.5 s milestone; it is not a hardware performance result.

It also evaluates 32 direct yaw controller/plant parameter corners with 0.5× or
2× nominal yaw inertia, damping, wheel torque, motor lag, and paired controller
gains. All corners meet the guide's 30° acquisition and 20° nudge timing limits;
the worst cases take 3.08 s and 2.48 s, respectively. The screen uses perfect
target-bearing and rate measurements and excludes camera, sensor noise,
attitude-estimation, firmware scheduling, and hardware effects. It tests
sensitivity of the simplified yaw model, not end-to-end robustness.

A second observation-level stress screen combines camera bearing-error noise,
gyro bias/noise, and matched camera/IMU delay up to 80 ms. All 81 cases meet
acquisition and nudge timing, while 80/81 meet the 60 s ±2° hold limit. The
single miss is 2.034° at 10 px RMS camera-error noise, -0.03 rad/s gyro bias,
0.01 rad/s RMS added gyro noise, and 40 ms delay. The grid includes 0.05°/s
RMS gyro noise as a TDK datasheet comparison point, alongside larger assumed
stress levels. These are not calibrated breakout measurements; camera noise
is applied to detected pixel error rather than rendered pixels. The report also
runs an eight-point PD gain sensitivity grid. Seven settings pass acquisition,
nudge and hold in all 81 assumed cases; the best tested Kp=0.14 N·m/rad,
Kd=0.05 N·m·s/rad setting has a 1.943° worst hold peak. The Kp=0.10,
Kd=0.05 setting misses once at 2.018°. These candidate results do not change
the configured controller; all stress inputs remain uncalibrated. Retest with
measured sensor, plant and link properties.
A separate 50-seed repeat at the critical stress corner passes 49/50 for both
baseline and Kp=0.14/Kd=0.05. The candidate lowers median peak error (1.648° vs
1.682°) but worsens p95 (1.929° vs 1.885°) and maximum (2.112° vs 2.008°).
This repeats one assumed corner only; it is not Monte Carlo coverage of the
full parameter grid.

[`Requirements_Traceability.md`](Requirements_Traceability.md) maps the guide's
functional and performance requirements to implementation evidence, limits,
and remaining verification gates.

[`Axis_Control_Results.md`](Axis_Control_Results.md) tests host-SIL stabilization
after target loss on pitch, yaw, and roll, and camera-directed pointing on all
three axes. Camera roll is tested with an optional asymmetric three-dot marker;
the accelerometer roll cue is hidden during the 15° recovery case. Point-only
frames still fall back to gravity-derived leveling. The C detector and virtual
pitch/roll source path are implemented for simulator-framed ESP32 observations,
but C compilation, Wokwi, physical targets, and hardware remain unverified.

Regenerate the axis-control report from `outputs/Python_sim/` with:

```bash
python3 -m Verification.evaluate_axis_control > ../Verification/Axis_Control_Results.md
```

Regenerate the report from `outputs/Python_sim/` with:

```bash
python3 -m Verification.evaluate_scenarios > ../Verification/Simulation_Results.md
```

The command exits nonzero if any model scenario misses its target.

[`Camera_Transport_Results.md`](Camera_Transport_Results.md) reports exact RLE1
round trips and encoded payload sizes across a 60-case synthetic camera scene
grid, along with host-only codec timings. It explicitly excludes HTTP, Wi-Fi,
Wokwi gateway, and ESP32 timing.
Regenerate it from `outputs/Python_sim/` with:

```bash
python3 -m Verification.evaluate_camera_transport > ../Verification/Camera_Transport_Results.md
```


[`Camera_Geometry_Results.md`](Camera_Geometry_Results.md) sweeps target diameter, depth, horizontal and vertical bearings through rendered QVGA images and the onboard Python
detector. It checks validity parity against the environment reference and records
both image-error axes and the detector's accepted size/range envelope under provisional pixel-area limits.
This is synthetic geometry evidence, not physical camera qualification.
Regenerate from `outputs/Python_sim/` with:

```bash
python3 -m Verification.evaluate_camera_geometry > ../Verification/Camera_Geometry_Results.md
```


[`Camera_Detector_Timing_Results.md`](Camera_Detector_Timing_Results.md)
reports host-only timing of the Python QVGA detector on clean, noisy, and dense
synthetic frames. It compares with the nominal 20 ms SIL period but excludes
transport, control, and ESP32 timing. Regenerate from `outputs/Python_sim/`
with:

```bash
python3 -m Verification.evaluate_camera_detector_timing > ../Verification/Camera_Detector_Timing_Results.md
```
