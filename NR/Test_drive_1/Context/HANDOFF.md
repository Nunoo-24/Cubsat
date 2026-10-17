# Test drive 1 handoff

## Current snapshot

This folder is the first full-direction test-drive release, before the later
yaw lost-target gain increase and stabilization moderation. It includes the
three runnable Python components, ESP-IDF/Wokwi source, interface documents,
requirements, original project inputs, generated reports, and this snapshot
context.

## Behavior in this version

- Yaw points from horizontal camera error and uses the body-Y gyro rate for
  damping. When the camera target is lost, IMU-only damping continues with
  the same `Kd=0.04 N·m·s/rad` gain as tracking.
- Host-SIL pitch points from vertical camera error with virtual torque capped
  at ±0.02 N·m. Host-SIL roll levels from gravity with `Kp=0.4`, `Kd=0.3` and
  virtual torque capped at ±0.05 N·m. These pitch/roll torques are simulation
  only.
- The test-drive sequence is in
  [`../outputs/Python_sim/FIRST_TEST_DRIVE.md`](../outputs/Python_sim/FIRST_TEST_DRIVE.md).

## Verification state at release

The full Python verification passed 190 tests and syntax compilation. The
portable C tests and ESP-IDF build were skipped because the required toolchains
were unavailable. Live Windows/WSL networking, browser behavior, Wokwi, and
physical hardware were not verified.

## Resume instructions

Read this file, the snapshot root `README.md`,
`inputs/Prompts.txt`, and `outputs/Python_sim/README.md`. Run each component
from `outputs/Python_sim/` in its own terminal: plant, CubeSat SIL, then Ground
Station. All run logs are generated inside this snapshot.
