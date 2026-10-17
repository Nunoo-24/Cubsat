# Three-Axis Stabilization and Camera-Pointing Verification

Generated: 2026-10-17 11:07:53 UTC

This report exercises the host Python SIL, synthetic QVGA camera, and
decoupled 3-axis plant. It is not a live-socket, ESP32 firmware,
emulator, calibrated sensor, or hardware test.

## Inertial stabilization after target loss

Each run starts with an 8 s camera occlusion and applies a 0.5 s torque
pulse at half the corresponding simulated actuator limit: X/pitch
0.010 N·m, Y/yaw 0.005 N·m, and Z/roll 0.0175 N·m. Rate settling
means under 0.5°/s continuously for 0.5 s. Pitch/roll attitude settling
also requires under 1° from the gravity-level reference. Yaw has no
absolute hold reference after target loss; it damps rate and retains the
new heading.

| Axis | Pulse (N·m) | Peak excursion (°) | Rate settled after pulse (s) | Attitude settled after pulse (s) | Final angle (°) | Final rate (°/s) |
|---|---:|---:|---:|---:|---:|---:|
| X / pitch | 0.0100 | 3.65 | 2.01 | 2.01 | -0.03 | -0.030 |
| Y / yaw | 0.0050 | 2.64 | 0.60 | rate-only | 2.64 | 0.066 |
| Z / roll | 0.0175 | 1.84 | 1.32 | 1.32 | -0.04 | 0.000 |

A concurrent run applies all three half-limit torques during the same
8 s target occlusion. The independent control outputs peak at
0.0041 N·m yaw,
0.0078 N·m pitch, and
0.0186 N·m roll. All three
rates stay below 0.5°/s and pitch/roll angles below 1° for a 0.5 s
dwell, settling 2.01 s after
the 0.5 s pulse. Final rates are yaw 0.066°/s,
pitch -0.030°/s, and roll
0.000°/s; target lost=
True, mode=RATE_DAMP,
fault=.

The yaw result demonstrates rate damping, not absolute yaw hold. The
camera-loss pitch/roll loops use gravity-derived attitude and return
toward level. All values are for the current provisional plant and
simulated torque limits.

## Closed-loop camera pointing

Yaw and pitch are tested independently in both error directions from
world target points at (+/-0.6, 0, 2) m and (0, +/-0.5, 2) m. A pass
requires combined camera truth error ≤2° for a continuous 0.5 s dwell.

| Camera-controlled case | Initial X error (°) | Initial Y error (°) | First settled dwell (s) | Final error (°) | Valid target samples |
|---|---:|---:|---:|---:|---:|
| Yaw / image X (+) | 16.70 | 0.00 | 2.34 | 0.15 | 1000/1000 |
| Yaw / image X (−) | -16.70 | 0.00 | 2.29 | 0.14 | 1000/1000 |
| Pitch / image Y (+) | -0.00 | -14.03 | 1.03 | 0.14 | 1000/1000 |
| Pitch / image Y (−) | 0.00 | 14.03 | 1.03 | 0.14 | 1000/1000 |

## Combined three-axis camera correction

Yaw and pitch image centering plus camera-marker roll correction were
run together from an off-axis target and combined body attitude.
Initial image error was 8.58° (36.9, -20.1 px); initial roll was 12.0° with CAMERA reference.
Image error ≤2°, roll within ±1°, and roll rate within ±0.5°/s for 0.5 s starting at 1.99 simulated seconds.
At 10 s, image error was 0.147°, roll -0.203°, and roll rate -0.016°/s; target valid=True, roll reference=CAMERA.

## Point-only target fallback

With the cube rolled 15.0° and the point centered,
camera X/Y errors remain 0/0 px
and the detector reports no roll angle. Roll torque is
-0.0350 N·m from the gravity-derived IMU.
This is the intended fallback when the optional asymmetric marker is
disabled, outside its inner-field visibility region, or not resolved.

## Camera roll closed-loop test with oriented marker

Initial body roll is 15.00°. The detected
image marker angle is -15.66°
and the inferred body-roll estimate is
15.66°. The roll
reference source is CAMERA;
accelerometer roll was overwritten as level in every cycle while gyro
Z remained available. The body returns to ±1° and ±0.5°/s in
8.32 s, ending at
0.152° and
-0.073°/s.

The same test was repeated from the opposite roll direction. With the
body initialized at -15.0°, the
camera estimated -15.66°
and commanded 0.0350 N·m
(opposite in sign to the initial roll). It returned to
-0.417° and
-0.013°/s in
9.16 s.

The simulation target is a high-contrast point with an optional
asymmetric three-dot orientation fiducial. The marker is disabled by
default and rendered only inside an inner camera field to avoid clipping
during edge acquisition. Point-only frames use IMU leveling. The physical
poster needs an equivalent resolved orientation cue for visual roll.

## Scope and reproduction

The plant is decoupled and uses provisional inertia, damping, torque,
and actuator values. This is deterministic software verification only.
The ESP32 source now computes virtual pitch/roll torque for simulator
frames; the plant accepts these only from the host SIL or simulator-framed
ESP32. The physical motor output remains yaw-only. C compilation, ESP-IDF,
Wokwi, sockets, browser behavior, and hardware were not exercised here.

From `outputs/Python_sim/`, regenerate this report with:

```bash
python3 -m Verification.evaluate_axis_control > ../Verification/Axis_Control_Results.md
```
