# CubeSat_ESP32

Target firmware for the XIAO ESP32-S3. Autonomous camera processing, sensor
processing, target-relative estimation, attitude control, wheel commands, and
fault handling belong on this board. The PC must not be needed to close the
control loop.

The onboard path now reports a one-axis yaw estimate. It integrates the
project's body-Y gyro rate and corrects drift with a tilt-compensated magnetic
heading when the sensor reference is valid. The SIL uses the known synthetic
field; the physical firmware profile reports gyro-only relative yaw because
the LIS3MDL calibration, mounting transform, and local magnetic reference are
not yet established. Camera telemetry includes horizontal and vertical pixel
errors for Ground Station display; host SIL uses horizontal camera error for
yaw and vertical camera error for pitch, with gravity-derived roll leveling.
Its pitch/roll torque outputs are virtual simulator inputs; physical firmware
and actuator support remain yaw-only. The explicit
`SET_YAW_STEP` milestone uses this estimate as a relative attitude setpoint,
with bounded feedback and a one-second settle check.

`sil.py` runs the host software-in-the-loop controller against simulated
sensor observations from `Sim_Environment`. It fetches a sequence-matched QVGA
grayscale camera frame and runs the same bounded dark-target detector before
control; it is not an ESP32 emulator. Host SIL uses image X for yaw, image Y
for pitch, gravity-derived roll to hold level, and Y-axis gyro rate for yaw
damping. If visual tracking loses the target, IMU-only yaw damping continues
without a stale visual setpoint. Pitch and roll commands use bounded virtual
simulator torques. The current physical actuator specification
only supports the yaw reaction-wheel path; pitch/roll hardware and firmware
remain unimplemented. The ESP-IDF project has a
bounded yaw controller and a Wi-Fi/HTTP simulation bridge client for the
emulator profile. It receives synthetic IMU/magnetometer observations and a
sequence-matched grayscale camera frame, applies the control law, and returns
body-torque telemetry to the plant. A
bounded wheel-speed commissioning command is also wired end to end: Ground
Station requests are accepted only from SAFE/STANDBY, use rev/s internally,
and trigger a zero-speed stop after 0.5–5 seconds. The model's speed servo is
optimistic and provisional; the commissioning path is integration evidence,
not validation of 3000 rpm performance. `SET_YAW_STEP` is available in the
Python SIL and source-level ESP-IDF app; only the Python model has been run.

When a fresh camera frame has no valid target, the SIL and firmware drop
visual correction but continue yaw-rate damping from a fresh IMU sample.
Missing, invalid, stale, or future camera/IMU samples fail closed to zero
torque. The SIL path has runtime regressions; the C firmware rule is
source-checked only.

The default Wokwi firmware profile downloads the observation-matched RLE1 QVGA
frame from the bridge, reconstructs all grayscale pixels, and runs the
portable C detector before control. The host SIL follows the same
transport path with the Python decoder/detector. The uncompressed raw endpoint
remains available for inspection. The physical Sense camera profile processes
its locally captured frame. The Wokwi/C path is source implemented but unbuilt
and unrun here.

RLE1 decoding is bounded to an 8,192-byte payload and exactly 76,800 output
pixels. Missing, oversized, malformed, or incorrectly sized frames fail the
camera validity path; the detector still processes reconstructed pixel data,
not an analytic target-error shortcut.

Build and emulate the firmware using the instructions in `firmware/README.md`.
A portable grayscale dark-target detector is included. Both Python and C apply an 8-neighbour coherence filter before centroiding, then enforce the coherent-pixel area limits. The optional
XIAO Sense camera adapter initializes the OV3660/OV5640-compatible camera
interface and runs the detector on grayscale QVGA frames. It is disabled in the
Wokwi profile; enable `CUBESAT_CAMERA_ENABLED` and PSRAM in `menuconfig` for the
physical Sense board. Camera threshold, dark-area limits, and focal length need
calibration, and the adapter has not been compiled or hardware-tested here.
An optional MPU-6050 I²C adapter reads XYZ gyro/accelerometer data and estimates
gyro bias at startup; it is disabled for Wokwi. It selects DLPF_CFG=3 and
SMPLRT_DIV=4 (200 Hz output). Sample age is estimated conservatively by adding
4.9 ms filter delay and up to 5 ms output phase to the read timestamp. This
driver has not been compiled or bench-tested. The current yaw controller treats MPU-6050 Y as yaw
rate, a provisional mapping that must be changed to match the measured sensor
mounting orientation and sign. An optional moteus r4 AUX2 UART adapter is now
implemented behind `CUBESAT_MOTEUS_ENABLED`, with an arm input, bounded torque,
finite controller watchdog and returned fault/status checks. It requires both
physical camera and MPU-6050 options and remains disabled in Wokwi/SIL. Its C
source has not been ESP-IDF compiled or hardware-tested. The simulator has a
separate provisional reaction-wheel model;
its parameters are not verified hardware configuration. ESP32-S3 TWAI is
Classical CAN only, so it cannot directly exchange moteus CAN-FD frames. moteus
documents a TTL UART alternative; see `firmware/README.md` and the hardware
notes. The firmware camera path includes thumbnail upload code, but it is
uncompiled. Enabling camera or IMU alone is sensor bring-up, not an integrated
physical closed-loop test.

An optional LIS3MDL driver shares the MPU-6050 I²C bus and reports a raw
three-axis magnetic-field vector in µT at 80 Hz, with a data-ready check and
bounded sample freshness. The simulation provides a fixed,
body-rotated placeholder field. Neither field is currently fused into attitude
or used by control; magnetic calibration and motor-interference checks are
required before using it as a heading reference. Enable the driver and select
the breakout's actual 0x1C/0x1E address in `menuconfig`.
