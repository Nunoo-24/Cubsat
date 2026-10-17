# AE CubeSat simulation — second test drive

This checklist verifies the current three-process host software-in-the-loop
(SIL) build with vertical pointing, gravity-based roll leveling, Y-axis IMU
rate damping after target loss, moderated stabilization gains, and the cube
reference frame. It is a live desktop test
for the Python simulation; it does not verify ESP32 firmware or physical
actuators.

## 1. Start cleanly

If an older run is still open, stop each component with `Ctrl+C` and close its
terminal after it returns to the prompt. Start the three Windows launchers in
this order, keeping each terminal visible:

1. `Launchers/Start_01_Simulation_Environment.bat` — plant, camera and browser
   page at <http://127.0.0.1:8000>.
2. `Launchers/Start_02_CubeSat_SIL.bat` — simulated onboard image processing
   and control.
3. `Launchers/Start_03_Ground_Station.bat` — operator page at
   <http://127.0.0.1:8001>.

Alternatively, open three WSL terminals in this directory and run:

```bash
python3 -u -m Sim_Environment.main
python3 -u -m CubeSat_ESP32.sil
python3 -u -m Ground_Station.main
```

Use one command per terminal, in the same order. Both browser pages should load.
The terminals should show the service startup lines and stay open. If a page
fails, preserve the terminal text and run
`Launchers/Diagnose_Connections.bat`; record its results before restarting.

## 2. Baseline and scene controls

- Confirm the environment shows the cube, target and XYZ reference frame at the
  cube center (X red, Y green, Z blue). Drag to orbit, use the wheel to zoom,
  and double-click to reset the camera view.
- With the target at its default centered position, check both pages show
  current, valid data. Ground Station camera X/Y error should be near zero;
  simulator-truth X/Y and requested/applied torque should update.
- In the Ground Station, compare onboard camera measurements with the separate
  simulator-truth values. They are different measurements and may differ during
  motion or when the target is not visible.

Record: both pages load (yes/no), whether values update, any stale/invalid
indicator, and any terminal errors.

## 3. Vertical target and pitch response

1. In the environment, set target position to `X=0, Y=0.5, Z=2 m` and apply.
2. Observe the cube and Ground Station for about 10 seconds.
3. Check the target remains valid, camera vertical error moves toward zero,
   pitch changes in the body frame, the requested/applied virtual pitch torque
   is visible, and simulator-truth vertical error falls.
4. Record the largest visible error, whether it settles, and any difference
   between camera error and simulator truth. Pixel quantization means exact
   zero pixels is not required; use roughly 2° or less as the visual target.

## 4. Horizontal target and yaw response

Move the target horizontally while leaving its height near zero. Confirm yaw
tracks it, camera horizontal error trends toward zero, and the cube's Y axis
shows yaw rotation. Record whether pitch and roll remain stable during this
check.

## 5. Yaw disturbance and IMU damping

Apply a simulated Y-axis torque of `0.1 N·m` for `0.5 s` while the target is
centered. This pulse should carry the target out of the camera field of view.
Watch the Ground Station's `gyro_y_rad_s` / yaw-rate and yaw body-torque
readouts, and the environment's simulator-truth yaw rate. Once the detector
reports no target, the controller should show `RATE_DAMP`, and the Y-axis gyro
rate should decrease toward zero even though visual error is unavailable.
Record when the target is lost, the peak rate if shown, when spin appears to
stop, and whether the wheel saturates. The model regression expects the rate
below 0.02 rad/s within 4.5 s of target loss for this pulse; actual timing can
vary with UI interaction and wheel limits.

## 6. Roll disturbance and recovery

Use the environment's external torque control to apply a Z-axis torque of
`0.05 N·m` for `0.5 s` (X=0, Y=0, Z=0.05). This is a simulated disturbance.

- Confirm the cube rotates about the blue Z axis and the Ground Station's
  simulator-truth roll changes.
- Confirm the simulated roll torque readout responds and roll returns to within
  about 1° with rate below 0.5°/s in under 3 seconds after the pulse ends.
- The moderated controller may allow a transient peak up to about 11°. If the UI makes
  the peak difficult to read, report the visible result and save the telemetry
  CSV for review.

Stop if the command cannot be entered or if the displayed axis/response does
not match. Do not connect physical motor power for this test.

## 7. Save results and stop

The Ground Station appends data to
`Ground_Station/logs/telemetry.csv`. If an older file used a different schema,
startup preserves it as `telemetry.legacy-YYYYMMDD-HHMMSS.csv` and creates a
current-schema log. Keep both files. After recording observations, stop each
process with `Ctrl+C` and report:

1. Baseline: page loads, live values and errors.
2. Vertical test: whether target is valid, error trend, settling and pitch
   response.
3. Horizontal test: whether yaw tracks the target.
4. Yaw disturbance: entered torque/duration, target-loss time, rate trend,
   recovery time, and wheel saturation if shown.
5. Roll test: entered torque/duration, observed peak if available, recovery
   time, and whether torque/readouts agree.
6. Any terminal messages and the telemetry CSV (or its relevant rows).

## Expected interpretation

The host SIL uses a reaction-wheel model for yaw and bounded ideal virtual
torques for pitch and roll. Yaw tracking uses the Y-axis gyro and target-loss
mode uses IMU-only rate damping (1.5× tracking rate gain). Current pitch torque
limit is ±0.02 N·m; roll
limit is ±0.035 N·m. Roll uses gravity-derived leveling because a single point target
cannot reveal rotation about the camera boresight. These axis controls and the
roll recovery timing are simulation tuning, not implemented physical pitch/roll
actuation. The Python offline suite passed 191 tests and syntax compilation;
live Windows/WSL networking and browser behavior still need this user-side
test. Firmware build, Wokwi, and hardware remain separate verification steps.

See [`FIRST_TEST_DRIVE.md`](FIRST_TEST_DRIVE.md) for launcher troubleshooting
and [`README.md`](README.md) for offline verification and firmware build
commands.
