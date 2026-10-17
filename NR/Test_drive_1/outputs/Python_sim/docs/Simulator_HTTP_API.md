# Simulation bridge HTTP API

**Revision:** 1.1  
**Base URL:** `http://127.0.0.1:8000`  
**Purpose:** local software-in-the-loop and Wokwi bridge  
**Physical radio protocol:** not selected

The bridge runs in `Sim_Environment/main.py`. The host SIL and ESP32-S3 Wokwi
firmware exchange versioned JSON with it. The Ground Station accesses the same
bridge through its local dashboard backend. The bridge binds to loopback only;
Wokwi reaches the host service through its private gateway.

## Environment control

`GET /api/state` returns simulated roll, pitch, and yaw angles/rates, target,
requested and actual body torque, wheel speed/torque/saturation state, and
actuator freshness. Scene XYZ maps to body rotation axes X=pitch, Y=yaw, and
Z=roll. The model has decoupled axes; yaw is driven by the provisional
reaction-wheel model. In host SIL, image Y error commands bounded virtual
pitch torque and accelerometer-derived roll commands bounded virtual leveling
torque. The bridge accepts these virtual fields only from `cubesat_sil` in
`virtual_axis_torque_nm`; they are not mapped to physical actuators. A point
target cannot reveal rotation about the camera boresight, so roll is leveled
using simulated gravity direction. Wheel and virtual actuator parameters are
provisional.
`POST /api/control` changes scene parameters. The request body is UTF-8 JSON,
maximum 4096 bytes:

```json
{
  "target_xyz_m": {"x": 0.2, "y": 0.0, "z": 2.0},
  "target_scale": 0.1,
  "external_torque_nm": {"x": 0.0, "y": 0.0, "z": 0.01},
  "duration_s": 1.0,
  "camera_occlusion_s": 2.0
}
```

Target coordinates use the scene frame (X right, Y up, Z forward) and scale is
metres. `z` must be positive; scale range is
0.001–2 m. External torque components are limited to ±1 N·m, duration to 0–60
s. In the body model frame, X torque drives pitch, Y drives yaw, and Z drives
roll. Camera occlusion duration is 0–60 s. The bridge validates all provided
fields before committing any of them; an invalid field returns HTTP 400 and
leaves the scene state unchanged. Unknown fields are rejected, and
`duration_s` is valid only with `external_torque_nm`.

Torque-pulse and camera-occlusion durations use the simulation clock shown in
the UI, not wall-clock time. Pausing or slowing the host therefore does not
shorten a simulated event. A torque pulse that ends partway through a plant
step is scaled for that step so its integrated impulse matches the requested
duration.

## Optional camera preview

`POST /api/camera-preview` accepts exactly 3072 raw grayscale bytes for a
64x48 image. The enabled ESP32-S3 camera adapter creates the thumbnail by
averaging each 5x5 block of its QVGA frame and uploads it once per second. The
Ground Station proxies `GET /api/camera-preview.bmp`; the bridge wraps the
thumbnail as an 8-bit grayscale BMP. On upload, the bridge pairs the image
with the just-received telemetry's target-valid flag and horizontal and
vertical pixel errors. The preview response carries these values in
`X-Camera-Target-Valid`, `X-Camera-Error-X-Px`, and
`X-Camera-Error-Y-Px` headers. Both offsets use original 320x240 QVGA image
coordinates, with positive X right and positive Y down. The Ground Station
uses the frame-matched offsets on its 320x240 display canvas, which shows the
64x48 thumbnail at five times its source dimensions. Firmware uploads
immediately after its actuator telemetry packet.
Frames older than two seconds return 503, so the UI can label the physical
preview unavailable. If the active source is host SIL and no physical frame is
fresh, the bridge downsamples the exact QVGA frame processed by host SIL to a
64x48 grayscale preview. It pairs that image with the detector's X/Y errors
and labels it `X-Camera-Preview-Source: simulated`; uploaded frames are labeled
`physical`. If SIL telemetry has no matching frame sequence, a simple marker
fallback is rendered from its reported pixel offsets. The bridge uses a
simulator frame for ESP32 telemetry only when firmware explicitly labels the
camera source simulated and returns its paired sequence; a missing physical
camera frame stays unavailable. Invalid-target frames omit X/Y
error headers. This local HTTP path is for emulator/SIL observation and does
not define the physical radio protocol.

## Onboard cycle exchange

`GET /api/cycle?after=<command_id>` returns one environment observation plus
at most one pending supervisory command. Each observation includes a monotonically
increasing `sequence`; the environment retains the paired camera frame. Host
SIL and the default Wokwi firmware profile fetch the paired frame from
`GET /api/camera-frame.rle?sequence=<sequence>` before processing the cycle.
The response repeats the sequence and QVGA dimensions in headers and declares
`X-Camera-Frame-Encoding: RLE1`. Its body is `RLE1` followed by records of
`uint16 little-endian run length, uint8 grayscale value`; zero-length runs,
truncated records, expansion beyond 76,800 pixels, and incomplete images are
rejected. Encoded payloads are capped at 8,192 bytes. Both clients reconstruct
the exact 320x240 grayscale frame before running the existing Python/C bounded
centroid detector onboard. `GET /api/camera-frame.raw?sequence=<sequence>` is
also available for inspection and tests and returns the uncompressed 76,800
bytes. The bridge retains the latest eight frames and returns 404 for an
unavailable/expired sequence. Its simple image model draws a bright background
and one dark disk based on target position, size, attitude, visibility and
occlusion. When the physical Sense camera profile is enabled, firmware uses
the local sensor instead. The C codec/detector path is source implemented but
has not been built or run in Wokwi. Both clients post actuator results to
`/api/actuator`.

For the deterministic default frame, the RLE1 body is 91 bytes; at the
configured 50 Hz this is 4.55 kB/s (36.4 kbit/s) of image payload, about 844x
smaller than raw QVGA. The 8,192-byte per-frame cap bounds the payload to
409.6 kB/s (3.28 Mbit/s) at 50 Hz. These are encoded payload measurements and
limits, not delivered-throughput or latency measurements. RLE1 compresses this
synthetic flat-background scene well and is not representative of noisy real
camera images. A deterministic sweep of 60 target depths, bearings, and sizes
produced a largest payload of 1,447 bytes; the test enforces that each sampled
scene remains under the cap and decodes exactly. The reproducible host size and
codec timing results are in [`Camera_Transport_Results.md`](../../Verification/Camera_Transport_Results.md).
Measure Wokwi gateway latency, dropped/expired frames and cycle
deadlines before claiming real-time behavior; stale or unavailable frames
continue to fail closed at the existing 100 ms sensor-age guard.

`POST /api/command` accepts PING, enumerated SET_MODE values, bounded
`SET_WHEEL_SPEED` commissioning requests, and relative `SET_YAW_STEP`
commands. `GET /api/cycle` includes
`actuator_feedback.wheel_speed_rad_s` from the simulated plant so onboard
software can report model speed and decide when a stop has settled.
Command IDs must be integers from 1 through 2^53−1 and increase for each new
command; this ceiling keeps IDs exact through the firmware's JSON number
parser. Repeating an identical ID and payload while its original ten-second
validity window is active is idempotent; reusing an ID with different content,
retrying an expired command, or submitting an older ID returns HTTP 400.
Commands expire after ten seconds. IDs are held in memory and reset when the
environment process restarts.

```json
{
  "protocol_version": 1,
  "message_type": "simulation_cycle",
  "observation": {
    "protocol_version": 1,
    "source": "sim_environment",
    "message_type": "environment_observation",
    "sequence": 42,
    "simulation_time_s": 0.84,
    "sensors": {
      "camera": {"valid": true, "pixel_error_x": 14,
                  "pixel_error_y": 6,
                  "image_width_px": 320, "focal_length_x_px": 277.0,
                  "target_dark_pixel_area": 148,
                  "age_s": 0.0},
      "imu": {"gyro_rad_s": {"x": 0.0, "y": 0.12, "z": 0.0},
              "accel_m_s2": {"x": 0.0, "y": 9.80665, "z": 0.0},
              "age_s": 0.0},
      "magnetometer": {"valid": true,
          "magnetic_field_uT": {"x": 20.0, "y": 0.0, "z": 45.0},
          "age_s": 0.0}
    }
  },
  "commands": []
}
```

The environment includes analytically projected camera errors as reference
metadata and generates the paired QVGA image. Host SIL and default Wokwi use
their respective Python and C detector results for control. The image uses a provisional 277 px focal
length. The simulation also provides deterministic XYZ gyro bias/noise, gravity
projected into the body frame, and a fixed placeholder world magnetic vector
rotated into the body frame. The magnetic field is not location-specific and
is not used by the yaw controller. These values do not replace OV5640/OV3660,
MPU-6050, or LIS3MDL measurements. Model-frame yaw rate is gyro Y; the physical
MPU mounting transform must be measured and configured to match. The
environment projects target scale into the image and counts dark pixels in the
rendered QVGA frame. The target is modeled as a camera-facing circular marker;
its apparent diameter follows the pinhole relation `focal_length × marker
diameter / optical-axis depth`. `target_dark_pixel_area` is this raster pixel
count, not ideal continuous area. The synthetic camera marks the target valid only when
50–200 rendered dark pixels remain within the nominal camera field of view. An
in-view target that is too small, too large, or has fewer than 50 visible pixels
remains in the raw frame while its reference validity and pixel-error fields
are false/null, matching the onboard detector's area decision. A clipped mask
that still passes the area limit can bias its centroid; edge reacquisition or
shape fitting is a future improvement. These rules are software test settings,
not a measured optical detection envelope.
The software projection is checked at five combined roll/pitch/yaw poses
against an independently assembled rigid-body rotation matrix; this does not
calibrate focal length, lens distortion, or camera mounting.
Environment `pixel_error_y` is positive up in the body-view convention. Onboard
telemetry `camera_error_y_px` is positive down, matching row direction in the
captured image; the host SIL converts this sign. Host SIL uses horizontal
error for yaw and vertical error for pitch. Roll leveling uses gravity
direction because a single point target does not constrain camera-boresight
rotation. These additional axes are host simulation behavior, not a claim of
three-axis physical actuation.
The onboard control law converts pixel error to bearing as
`atan(pixel_error_x / focal_length_x_px)`. Positive pixel error is positive
target bearing under the current image/body sign convention; calibrate that
sign on hardware.

An actuator packet is posted to `POST /api/actuator`:

```json
{
  "protocol_version": 1,
  "source": "cubesat_sil",
  "message_type": "actuator_telemetry",
  "sequence": 42,
  "simulation_time_s": 0.84,
  "mode": "TRACK",
  "system_state": "NOMINAL",
  "fault": "",
  "target_valid": true,
  "camera_frame_source": "simulated",
  "camera_frame_sequence": 42,
  "camera_error_x_px": 14,
  "camera_error_y_px": -6,
  "camera_focal_length_x_px": 277.0,
  "camera_age_s": 0.0,
  "imu_valid": true,
  "imu_age_s": 0.0,
  "target_error_rad": 0.05,
  "gyro_x_rad_s": 0.0,
  "gyro_y_rad_s": 0.12,
  "gyro_z_rad_s": 0.0,
  "yaw_rate_rad_s": 0.12,
  "yaw_estimate_rad": 0.001,
  "yaw_estimate_valid": true,
  "yaw_estimate_source": "MAG_FUSED",
  "accel_x_m_s2": 0.0,
  "accel_y_m_s2": 9.80665,
  "accel_z_m_s2": 0.0,
  "mag_valid": true,
  "mag_frame": "body",
  "mag_age_s": 0.01,
  "mag_x_uT": 20.0,
  "mag_y_uT": 0.0,
  "mag_z_uT": 45.0,
  "body_torque_nm": 0.0008
}
```

`camera_age_s` and `imu_age_s` report their own sensor sample ages. Control
uses the older of the two samples and commands zero torque if either age is
missing, negative, non-finite, or older than the controller's 100 ms limit.
In the simulated camera path, age is measured from the start of the cycle
request through delivery and processing of its paired raw frame; the physical
camera path uses its capture timestamp.
Invalid ages are reported as JSON `null`; they are never averaged or hidden by
the other sensor's fresher timestamp.

`body_torque_nm` is the desired net body torque, limited by the bridge to
±0.1 N·m. The plant maps yaw-axis requests to opposite wheel motor torque and
simulates motor lag, wheel inertia, torque limit, and speed saturation. The
wheel parameter defaults are unmeasured placeholders; motor-controller delay,
current/thermal limits, and CAN-FD are not simulated. `/api/telemetry` adds
`simulated_wheel_speed_rad_s`, `simulated_wheel_motor_torque_nm`,
`simulated_body_torque_nm`, and `simulated_wheel_saturated` for the local
Ground Station only. A command not refreshed for 100 ms is
set to zero by the simulation watchdog. This watchdog is for the simulation
bridge; physical actuator safety must be implemented and verified onboard.
Host Python SIL may additionally include `virtual_axis_torque_nm` with
`pitch` limited to ±0.02 N·m and `roll` limited to ±0.05 N·m. Roll gains and
authority are halfway between the previous and stronger settings; this softer
tuning peaks below 11° and settles a 0.05 N·m, 0.5 s Z disturbance within
three simulated seconds.
Yaw uses the Y-axis gyro for rate damping, including the IMU-only lost-target
mode. The bridge accepts these only when `source` is `cubesat_sil`;
they drive ideal pitch/roll body torques in the simulated plant and are not
physical actuator commands. The
environment canvas draws the body reference frame at the cube center: X red
(pitch), Y green (yaw), Z blue (roll/camera boresight).
When the optional physical moteus adapter is enabled in ESP-IDF, its
`moteus_enabled`, arm/link state, returned mode/fault, position, velocity, and
measured/commanded motor torque fields are passed through to the Ground
Station and CSV. These fields report hardware communication only; the local
plant remains simulated and does not consume measured moteus state.
Magnetometer telemetry uses raw XYZ magnetic field in microtesla and identifies
its coordinate frame: `body` for the synthetic observation and `sensor` for
physical LIS3MDL readings until a mounting transform is configured. When either
physical MPU-6050 or LIS3MDL profile is enabled, firmware suppresses magnetic
correction in the yaw estimator because the sensor frame/calibration is not
established; it may still forward bridge magnetic values for display when the
LIS3MDL adapter is off.
The host SIL fuses only the known synthetic reference field. `yaw_estimate_*`
reports the one-axis estimate, its validity, and either `MAG_FUSED`,
`GYRO_ONLY`, or `INVALID`; a gyro-only result is relative to the initialization
heading and will drift over time.

`GET /api/telemetry` returns the latest onboard packet, augmented with a
monotonic `ground_sequence` and receive simulation time. It returns HTTP 503
until the SIL or firmware posts telemetry, and again when no actuator/telemetry
packet has arrived for 1.5 s so the Ground Station does not show a stale packet
as a live link.

## Supervisory telecommands

The Ground Station sends `POST /api/command` with a versioned JSON packet:

```json
{
  "protocol_version": 1,
  "source": "ground_station",
  "message_type": "telecommand",
  "command_id": 1792000000000,
  "command": "SET_MODE",
  "arguments": {"mode": "RATE_DAMP"},
  "issued_unix_s": 1792000000.0
}
```

Supported commands are `PING`, `SET_MODE`, `SET_WHEEL_SPEED`, and `SET_YAW_STEP`; supported
modes are `SAFE`, `STANDBY`, `RATE_DAMP`, `ACQUIRE`, and `TRACK`.
`SET_WHEEL_SPEED` arguments contain exactly `target_rev_s` in [0, 50] and
`duration_s` in [0.5, 5]. The onboard side accepts it only in SAFE/STANDBY,
commands zero speed at the end of the interval, and reports stop completion or
a 10-second stop timeout. The bridge applies the setpoint to its provisional
speed-servo model, which does not represent a validated moteus speed loop.
Terminal test status remains in telemetry until another test begins. The
environment accepts
positive integer IDs and rejects requests issued more than 10 seconds earlier
or over one second in the future. The bridge queues at most 50 commands and
offers them in ID order, one per cycle. The onboard implementation ignores IDs
it has already processed and reports `command_ack` in its next telemetry.
The Ground Station cannot set wheel torque or close the control loop.
The onboard reference and firmware use the same rejection reason for an
unsupported `SET_MODE` value (`INVALID_MODE`) and for extra/missing speed-test
arguments (`INVALID_SPEED_TEST_ARGUMENTS`). Other reason strings identify
mode gating, unavailable attitude, or unsupported command types.

`SET_YAW_STEP` contains exactly `delta_deg` in [-180, 180], relative to the
current onboard yaw estimate. It is accepted only in SAFE/STANDBY with a valid
estimate. The onboard controller uses the estimate and yaw rate, independent
of camera visibility, and completes after error ≤1° and rate ≤0.5°/s for one
continuous second. A ten-second timeout or invalid/stale attitude sample stops
the request and enters SAFE. Telemetry reports target, error, elapsed time,
and RUNNING/COMPLETE/TIMEOUT/ABORTED/ATTITUDE_INVALID status. The target and
current error are `null` before a request and after abort, timeout, or invalid
attitude; a completed request retains its final target and error. This is
model/controller evidence, not calibrated physical performance.

## Other endpoints and failure behavior

- `POST /api/ack` and `GET /api/ack` expose the most recent command
  acknowledgement. A POST body must include a known command ID, status
  `ACKNOWLEDGED` or `REJECTED`, and an optional text reason (required for a
  rejection). An acknowledgement included in `/api/actuator` is validated
  before telemetry state is updated, so malformed acknowledgements reject the
  full packet without partially accepting its torque or telemetry.
- Invalid JSON, schema, values, or command names return HTTP 400.
- Missing telemetry returns HTTP 503.
- The service binds to `127.0.0.1` and has no authentication. Keep it local.
- This HTTP/JSON bridge is an emulator/SIL interface; it is not CCSDS and is
  not the physical Wi-Fi TM/TC interface. Decide the physical protocol,
  addressing, authentication, integrity check, retransmission policy, and
  link-loss behavior before hardware networking.
