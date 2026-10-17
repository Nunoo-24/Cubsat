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
errors for Ground Station display, plus directed marker orientation when the
image contains an asymmetric fiducial. Host SIL uses horizontal error for yaw,
vertical error for pitch, and marker orientation for roll; a point-only image
falls back to gravity-derived leveling. The portable C detector and controller
implement the same orientation and simulator-only virtual pitch/roll path.
Physical motor actuation remains yaw-only. The explicit
`SET_YAW_STEP` milestone uses this estimate as a relative attitude setpoint,
with bounded feedback and a one-second settle check.

`sil.py` runs the host software-in-the-loop controller against simulated
sensor observations from `Sim_Environment`. It fetches a sequence-matched QVGA
grayscale camera frame and runs the same bounded dark-target detector before
control; it is not an ESP32 emulator. Host SIL uses image X for yaw, image Y
for pitch, and directed marker orientation for roll. The optional asymmetric
marker is disabled by default to preserve the original point target; with the
marker off, roll uses gravity-derived leveling. If visual tracking loses the target, IMU-only yaw damping continues
without a stale visual setpoint. Pitch and roll commands use bounded virtual
simulator torques. The current physical actuator specification supports only
the yaw reaction-wheel path; pitch/roll hardware remains undefined. The ESP-IDF
project has bounded yaw and virtual pitch/roll controllers and a Wi-Fi/HTTP
simulation bridge client for the emulator profile. It receives synthetic
IMU/magnetometer observations and a sequence-matched grayscale camera frame,
runs the C detector/controller, and returns body and virtual-axis torque to the
plant only when the frame source is simulated. A
bounded wheel-speed commissioning command is also wired end to end: Ground
Station requests are accepted only from SAFE/STANDBY, use rev/s internally,
and trigger a zero-speed stop after 0.5–5 seconds. The model's speed servo is
optimistic and provisional; the commissioning path is integration evidence,
not validation of 3000 rpm performance. `SET_YAW_STEP` is available in the
Python SIL and source-level ESP-IDF app; only the Python model has been run.

At startup, host SIL waits for the simulation bridge rather than claiming an
early connection. Failed exchanges use a bounded exponential retry delay from
50 to 500 ms; the initial failure and recovery are reported, and repeated
errors are rate-limited to one message per five seconds. The normal loop remains
50 Hz after successful exchanges. Start the environment first; if it starts
late, SIL continues waiting without flooding its terminal.

When a fresh camera frame has no valid target, the SIL and firmware drop
visual correction but continue yaw-rate damping from a fresh IMU sample. The
lost-target gain is 0.06 N·m·s/rad, versus 0.04 during visual tracking.
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

RLE1 decoding rejects payloads over 8,192 bytes and decoded rasters above
640×480 pixels. The current simulator call requires exactly 76,800 QVGA
pixels. Missing, oversized, malformed, or incorrectly sized frames fail the
camera validity path; the detector still processes reconstructed pixel data,
not an analytic target-error shortcut.

The portable detector also requires the accessible grayscale buffer length and
rejects a frame shorter than `stride × height` before scanning it. The
simulator caller supplies the static frame-array size; the physical camera
adapter supplies the camera driver's `frame->len`. A short-buffer regression
is present in the C host-test source but remains unexecuted until a C11
compiler is available.

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
bounded sample freshness. Its ±4/8/12/16 gauss range is selectable in
`idf.py menuconfig` (±4 gauss default). The simulation provides a fixed,
body-rotated placeholder field. Neither field is currently fused into attitude
or used by control; magnetic calibration and motor-interference checks are
required before using it as a heading reference. Enable the driver and select
the breakout's actual 0x1C/0x1E address in `menuconfig`.


The portable C pitch/roll controller is in `firmware/main/pitch_roll_controller.c`;
its host test is `firmware/test_pitch_roll_controller.c`. The camera detector
reports `camera_roll_orientation_rad` for asymmetric targets and null orientation
for a circular point. Ground Station telemetry displays and logs the image angle,
body-roll estimate, and selected reference source. ESP-IDF compilation and
Wokwi execution remain unverified when their toolchains are absent.
