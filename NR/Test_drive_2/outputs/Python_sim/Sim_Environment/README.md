# Sim_Environment

The local plant model and operator view for the CubeSat demonstrator.
The plant is the source of truth for gimbal attitude, external torque, and target
geometry. It generates synthetic camera/IMU observations through a versioned
HTTP simulation bridge. Keep the simulated world here; do not place autonomous
control decisions in this component.

## Model status

The current model includes pitch/yaw/roll dynamics and a first-pass yaw
reaction-wheel actuator with provisional inertia, torque/speed limits, and
motor lag. Host SIL also applies bounded ideal virtual pitch/roll torques so
image Y error points the camera in the vertical dimension while gravity keeps
roll level.
These axes are simulation-only; no physical pitch/roll actuators have been
selected. Pendulum, body, wheel, virtual-axis, camera, and sensor parameters
must be measured or confirmed before results become engineering evidence. The
black target is a camera-facing circular marker located at the
configured world XYZ point. Its image size follows pinhole projection from
optical-axis depth; a monocular camera does not provide target range by itself.
An oriented poster or non-circular target is not modeled.

## Run

From the Python_sim project root, run `python3 -m Sim_Environment.main` and
open `http://127.0.0.1:8000`. The browser scene allows target position/scale,
simulation-time-bounded XYZ torque, and camera occlusion. Event durations use
the displayed plant clock, so host speed or pauses do not alter them. A torque
pulse ending within a model step is integrated proportionally to preserve its
requested impulse. Yaw is controlled by the reaction wheel; roll/pitch have
linear pendulum restoring dynamics plus bounded virtual SIL torque. Pitch
virtual torque is capped at ±0.02 N·m and roll at ±0.035 N·m. Host-SIL yaw
rate damping continues from the simulated Y-axis gyro when the target leaves
the camera view. The model
frame maps X torque to pitch, Y torque to yaw, and Z torque to roll. Euler-rate
coupling is omitted. Its synthetic camera returns horizontal/vertical pixel
error and validity; for each versioned observation it also renders a raw
QVGA grayscale frame with the dark target at the projected pixel location.
The scene canvas supports mouse-drag orbit, mouse-wheel zoom, and double-click
view reset; this changes only the display camera, not simulated state.
Host SIL and the ESP-IDF/Wokwi default profile fetch that sequence-matched
frame and run their Python/C onboard detector before control. The physical
camera profile uses its local sensor. The IMU
returns three body rates and projected gravity, and the magnetometer returns a
body-rotated fixed placeholder field. These remain idealized sensor inputs,
not calibrated hardware models.

The bridge provides environment observations to the CubeSat SIL or ESP32
emulator, accepts bounded actuator telemetry, and exposes queued supervisory
commands. See `../docs/Simulator_HTTP_API.md` for the versioned fields and
limits. A 100 ms actuator watchdog zeros stale commands; telemetry older than
1.5 s is reported unavailable so the Ground Station marks the link down.
