# Model-level dynamics, pointing, and sensor evaluation

Generated: 2026-10-17 11:06 UTC

This report measures deterministic software-in-the-loop pointing,
one-axis yaw-estimator, and passive-axis free-decay scenarios using
the Python plant and onboard reference. Closed-loop pointing scenarios
render a sequence-paired QVGA frame and run the host centroid detector
before control. Guide performance limits are proposed in the course
project guide and summarized in
[`PROJECT_BRIEF.md`](../../Context/PROJECT_BRIEF.md); the passive-axis
free-decay row is a qualitative model check, not an acceptance target.
Passing these cases
does not verify ESP32 firmware, live HTTP, camera/IMU hardware, motor
behavior, or the physical rig. Plant constants and camera/noise models
remain provisional.

| Scenario | Criterion / model check | Model result | Outcome |
|---|---:|---:|---|
| 30° initial error to ≤2° | < 6 s | 1.600 s | PASS |
| Pointing hold | ≤2° for 60 s | 0.107 ° peak error | PASS |
| 20° nudge recovery to ≤2° | < 4 s | 1.340 s | PASS |
| Moving target at 0.2°/s | ≤2° for 60 s | 0.119 ° peak error | PASS |
| Recovery after 2 s camera occlusion | ≤2° within 4 s | 0.760 s | PASS |
| Yaw estimate with synthetic magnetic reference | <2° error and drift per minute | 0.068 ° peak error | PASS |
| Tilt-compensated yaw across combined attitudes | ≤0.000001° over 5 idealized poses | 0.000 ° peak error | PASS |
| Camera projection across combined attitudes | matches rigid-body matrix at 5 idealized poses | 0.000 px maximum X/Y discrepancy | PASS |
| Synthetic onboard detector across nominal yaw FOV | valid at all 13 bearings, ±30° in 5° steps | 13/13 valid | PASS |
| 45° commanded yaw step | settle within ±1° | 5.820 s to 1 s settled dwell | PASS |
| Passive roll/pitch free decay | energy decreases under damping | 13.978 % maximum energy retained after 20 s | PASS |
| Yaw parameter corner sensitivity | acquire <6 s and nudge recovery <4 s at all 32 corners | 32/32 pass | PASS |

Peak yaw rate during the camera-occlusion interval: 0.0012 rad/s.
Yaw-estimator final drift over 60 s: 0.0503°; peak absolute error: 0.0678°.
Tilt-compensated synthetic yaw grid: 5 combined roll/pitch/yaw poses, 0.000000000° maximum error. The field and gravity vectors are ideal simulator references, so this checks coordinate consistency only.
Synthetic camera projection grid: 5 combined attitude poses, 0 px maximum X/Y discrepancy against an independently assembled rotation matrix. This checks geometry/sign consistency, not camera calibration.
Synthetic detector sweep: 13/13 targets valid at depth 2 m and 0.1 m scale; maximum centroid discrepancy is 2.91 px. The discrepancy is caused by rasterization/clipping at the FOV edge. This checks ideal renderer/detector compatibility only, not detection rate or latency on camera hardware.
Full-direction tilted pointing diagnostic (6 s): horizontal camera error 0.00° / 0.0 px, vertical error 0.00° / 0.0 px; 300/300 target samples valid. Host SIL commands virtual pitch/roll torques; these are not physical actuator or firmware results.
Yaw parameter corner sweep (direct control-law/plant, perfect measurements): 32/32 corners meet both guide timing targets; worst acquisition 3.08 s and nudge recovery 2.48 s. Inertia, yaw damping, wheel torque, motor lag, and paired controller gains were independently set to 0.5× or 2× nominal. This is a parameter screen for the simplified yaw model, not end-to-end SIL robustness.
Observation noise/latency screen: 81 cases; acquire 81/81, nudge 81/81, hold 80/81 within 2°. Worst measured peak yaw is 2.034°. Hold miss inputs: 10 px RMS camera-error noise, -0.030 rad/s gyro bias, 0.010000 rad/s RMS gyro noise, 40 ms delay (2.034° peak). The stress levels are observation-output noise and matched sample delays, not calibrated camera/IMU properties.
Gain sensitivity candidates (Kp/Kd in N·m/rad and N·m·s/rad; each reports acquire/nudge/hold and worst hold peak): 0.10/0.04=81/81,81/81,81/81; 1.986°; 0.10/0.05=81/81,81/81,80/81; 2.018°; 0.12/0.04=81/81,81/81,81/81; 1.969°; 0.12/0.05=81/81,81/81,81/81; 1.949°; 0.14/0.05=81/81,81/81,81/81; 1.943°; 0.16/0.06=81/81,81/81,81/81; 1.947°; 0.18/0.06=81/81,81/81,81/81; 1.978°; 0.20/0.07=81/81,81/81,81/81; 1.993°. These are simulation-only candidates; configured gains remain unchanged pending measured plant and sensor/link data.
Critical-corner hold seed sensitivity (50 independent noise seeds; 10 px camera noise, -0.03 rad/s gyro bias, 0.01 rad/s gyro noise, 40 ms delay): baseline Kp/Kd=0.08/0.04 49/50 pass, median 1.682°, p95 1.885°, worst 2.008°; candidate 0.14/0.05 49/50 pass, median 1.648°, p95 1.929°, worst 2.112°. The candidate improves the median but worsens the upper tail; 50 seeds at one assumed corner are not hardware tuning evidence.
Passive-axis free-decay check (no external torque, 20 s): roll 10.0° → 1.68° (14.0% energy retained); pitch -8.0° → -1.35° (14.0% energy retained). This checks only qualitative damping under provisional parameters.

## Earlier guide milestones and current evidence limits

The project brief also records earlier staged targets. They are kept
visible here because they remain unimplemented or lack hardware
calibration. A separate best-case spin-up bound is included for the
current placeholder wheel model; it is not a motor-performance claim:

| Guide milestone | Status | Current evidence gap |
|---|---|---|
| 3000 rpm in <0.5 s with <2% error | COMMAND PATH IMPLEMENTED; PERFORMANCE NOT VERIFIED; PLACEHOLDER MODEL LIMIT 431 rpm | The Ground Station → bridge → onboard software contract and firmware velocity API call now exist, with SAFE/STANDBY gating and timed stop logic. The Python plant reaches only 431 rpm after 0.5 s under its provisional 1e-4 kg·m² inertia, 0.01 N·m torque limit and 0.05 s torque lag. At that inertia, ideal constant torque requires 62.8 mN·m before losses. Source review shows the physical velocity frame uses the shared moteus torque setting (1 mN·m default, 20 mN·m Kconfig maximum); ideal no-loss speed at the 20 mN·m cap is only 955 rpm for this inertia. Reaching 3000 rpm in 0.5 s at the maximum cap requires total wheel inertia at or below 31.8×10⁻⁶ kg·m²; the default cap requires 1.59×10⁻⁶ kg·m². These ideal torque/inertia bounds are not motor ratings or measured performance. No exact motor/wheel parameters, firmware build, or physical speed-loop result is available. |
| Yaw estimate error <2° and drift <2°/min | MODEL RESULT 0.068° peak error / 0.050° drift | The one-axis Python estimator is evaluated against simulator truth using synthetic gyro bias/noise and a placeholder fixed magnetic field. This does not assess roll/pitch attitude, magnetic calibration/disturbance, firmware, or hardware. |
| 45° angular step settles within ±1° | MODEL RESULT: first entered ±1° and ≤0.5°/s at 5.820 s; completed 1 s dwell at 6.820 s | The step uses the onboard yaw estimate and bounded body-torque controller, independent of camera visibility. This is a synthetic-model result only; controller gains and estimate are not hardware-calibrated. |

## Reproduce

From `outputs/Python_sim/`, run:

```bash
python3 -m Verification.evaluate_scenarios > ../Verification/Simulation_Results.md
```

The unit suite independently asserts pointing behavior, synthetic-field and tilted-pose yaw estimation, camera projection/detection geometry, passive-axis energy decay, parameter-corner timing, observation-noise/latency sensitivity, and the placeholder wheel spin-up bound.
