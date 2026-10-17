# Onboard control

**Role:** produce bounded body torque onboard from fresh camera-bearing error and yaw rate during visual pointing, or from a relative yaw-estimate setpoint and yaw rate during the explicit step test. In host SIL, image Y also drives bounded virtual pitch torque and gravity-derived tilt drives virtual roll-leveling torque. Telemetry loss must not move control to the PC.

**Inputs:** target validity and camera X/Y error, XYZ gyro, gravity-derived tilt, yaw-step attitude error, sample age and supervisory mode. **Output:** bounded yaw torque plus host-SIL-only virtual pitch/roll torques, mode and fault status.

If a fresh camera frame has no valid target, the controller uses fresh IMU yaw rate for damping only. Any missing, invalid, stale, or future camera/IMU sample fails closed and inhibits torque. The Python SIL exercises this behavior; the C firmware rule is source-checked only and still needs ESP-IDF and target verification.

**Implementation:** [Python reference](../../Python_sim/CubeSat_ESP32/control.py) and firmware C controller `../../Python_sim/CubeSat_ESP32/firmware/main/yaw_controller.c`.

**Limit:** current PD gains/torque bounds are provisional. Visual yaw uses X camera error; host-SIL pitch uses Y error; roll uses gravity leveling because a point target cannot observe boresight rotation. Pitch/roll torques are accepted by the simulator only from host SIL and are not hardware outputs. Yaw-step mode uses the one-axis estimated attitude and rate. The loop does not use measured wheel speed or hardware status for anti-windup/protection. C firmware remains yaw-only and has no compiler verification here.

The first test-drive tuning uses host-SIL roll gains `Kp=0.4 N·m/rad`,
`Kd=0.3 N·m·s/rad`, with virtual roll torque bounded to ±0.05 N·m. Pitch
remains bounded to ±0.02 N·m. A deterministic 0.05 N·m, 0.5 s Z pulse peaks
below 5° and enters the 1° / 0.5°/s settle band within two simulated seconds.
These are virtual plant/controller results, not hardware response claims.

The bounded wheel-speed commissioning state bypasses this attitude controller and reports zero body-torque request while the actuator adapter receives its separate native velocity target. It is admitted only from `SAFE` or `STANDBY`; ending the interval requests zero speed and then returns to `SAFE` after measured stop or timeout.

Both Python and C references now reject non-finite or physically invalid controller configuration (negative gains, nonpositive torque/age/rate limits) with zero torque and `INVALID_CONFIG`. This prevents a malformed configuration from silently reversing feedback or disabling a limit.

**Verify next:** compare Python/C outputs over randomized and boundary cases; then retune using measured inertia, damping, motor/wheel limits and delay.

A 45° relative yaw command entered the 1° error / 0.5°/s rate settle band at 5.820 s and completed the required one-second dwell at 6.820 s in the deterministic Python model. This is not firmware execution or physical performance evidence.

An in-memory integration regression also sends the Ground Station command
envelope through the simulator's HTTP handlers, the SIL, actuator telemetry,
and the simulated plant. The command is acknowledged and completes inside its
ten-second timeout. This checks Python component integration without opening
TCP sockets.

**Sensitivity evidence:** a direct yaw controller/plant sweep meets the
proposed acquisition and nudge limits at all 32 combinations of 0.5×/2× yaw
inertia, damping, wheel torque, motor lag, and paired gains. An observation-
level 81-case screen with pixel-error noise, gyro bias/noise, and 0–80 ms
sample delay meets those two timing limits in every case, but one 60 s hold
case reaches 2.034° (just outside ±2°). The expanded grid includes a 0.05°/s
gyro-noise comparison point from the MPU-6050 datasheet. These are assumed stress inputs, not
measured hardware bounds; the pixel noise is added after detection. A reproducible eight-point gain sweep is recorded in the model report. Seven
settings pass all 81 cases; the best tested Kp=0.14 N·m/rad, Kd=0.05 N·m·s/rad
setting reaches a 1.943° worst hold peak. A nearby setting (Kp=0.10, Kd=0.05)
misses one case at 2.018°. These are candidate sensitivity results, not adopted
tuning: even the best has little margin and depends on unmeasured inputs.
A 50-seed repeat of the critical stress corner also gives 49/50 holds for both
baseline and Kp=0.14/Kd=0.05. The candidate has a better median (1.648° vs
1.682°) but a worse p95 (1.929° vs 1.885°) and worst case (2.112° vs
2.008°); this is only one stress corner and does not validate the full noise
space. Measure sensor noise, bias, inertia, damping, and delay before choosing gains.
