# Telemetry and Telecommand Interface Control Document

**Revision:** 0.3
**Status:** Level 0 legacy protocol plus version 1 simulation bridge
**Scope:** host SIL and Wokwi emulator; physical TM/TC link remains TBD

## 1. Purpose and architecture rule

This document distinguishes the legacy UDP heartbeat from the integrated
version 1 HTTP/JSON software-in-the-loop interface. The latter is used by the
host controller and planned Wokwi ESP32-S3 runtime; it is not the project's
physical Wi-Fi/radio protocol.

The autonomous camera/sensor/control loop runs on the CubeSat's XIAO ESP32-S3.
The ground station monitors and configures the system; it does not calculate
the image error or command the reaction wheel in the autonomous mode.

Both interfaces are local software exercises. Neither is a radio specification
or a CCSDS-compliant protocol.

## 2. Level 0 local transport

| Direction | Address | Port | Purpose |
|---|---|---:|---|
| Simulator → Ground station | `127.0.0.1` | `5005/UDP` | Periodic telemetry |
| Ground station → Simulator | `127.0.0.1` | `5006/UDP` | Telecommands |

- Encoding: UTF-8 JSON, one JSON object per UDP datagram.
- Simulator telemetry period: 0.5 s (2 Hz).
- No delivery, ordering, or retransmission guarantee is provided by UDP.
- The current Ground Station binds only to loopback. A real ESP32 on Wi-Fi
  cannot send to this interface until the transport/address binding is changed.

## 3. Implemented Level 0 messages

### 3.1 Telemetry datagram

The simulator sends:

```json
{
  "sequence": 1,
  "timestamp": "2026-10-06T10:00:00+00:00",
  "mode": "SIMULATION",
  "system_state": "NOMINAL",
  "alive": true,
  "last_command": "—",
  "command_status": "—"
}
```

| Field | Type | Meaning |
|---|---|---|
| `sequence` | integer | Increments for each generated telemetry packet. |
| `timestamp` | string | UTC timestamp in ISO 8601 format. |
| `mode` | string | Prototype mode; currently `SIMULATION`. |
| `system_state` | string | Prototype health state; currently `NOMINAL`. |
| `alive` | boolean | Simulator heartbeat indicator. |
| `last_command` | string | Most recently observed command, or `—`. |
| `command_status` | string | `ACKNOWLEDGED`, `REJECTED`, `INVALID COMMAND`, or `—`. |

The ground-station UI displays these fields and marks the link connected when a
valid JSON telemetry datagram arrives. It treats telemetry older than 2 s as
stale.

### 3.2 Legacy Level 0 PING datagram example

This early UDP/JSON datagram is retained as a separate demonstration; the current
version 1 simulation bridge and command contract are specified in Section 4.



The prototype ground station sends this when the user presses **Send PING
command**:

```json
{
  "type": "telecommand",
  "command_id": 1,
  "command": "PING",
  "timestamp": "2026-10-06T10:00:00+00:00"
}
```

| Field | Type | Meaning |
|---|---|---|
| `type` | string | Set to `telecommand`; currently informational only. |
| `command_id` | integer | Ground-station incrementing command number. |
| `command` | string | Prototype command name. Only `PING` is accepted. |
| `timestamp` | string | UTC timestamp in ISO 8601 format. |

The simulator reports the PING acknowledgement in later telemetry using
`last_command` (for example, `PING #1`) and `command_status` (`ACKNOWLEDGED`).
There is no separate acknowledgement datagram in this version. Other valid JSON
commands are reported as `REJECTED`; malformed UTF-8/JSON is reported as
`INVALID COMMAND` in subsequent telemetry.

## 4. Version 1 simulation bridge

The host plant exposes a loopback HTTP server on `127.0.0.1:8000`. The Ground
Station dashboard serves locally on `127.0.0.1:8001` and proxies telemetry and
commands to that bridge. The Wokwi ESP32-S3 firmware reaches the bridge through
Wokwi's private gateway using `host.wokwi.internal`; the host SIL uses
`http://127.0.0.1:8000` directly.

| Request | Direction | Purpose |
|---|---|---|
| `GET /api/cycle?after=<id>` | Plant → Onboard | Latest IMU observation, camera-frame sequence, analytic camera reference metadata, and one queued supervisory command |
| `GET /api/camera-frame.rle?sequence=<n>` | Plant → Onboard | Sequence-paired QVGA RLE1 grayscale frame, reconstructed before host SIL/C detector processing; max 8192 encoded bytes |
| `GET /api/camera-frame.raw?sequence=<n>` | Plant → Operator/test tools | Uncompressed 320x240 grayscale frame for inspection and verification |
| `POST /api/actuator` | Onboard → Plant | Desired yaw body torque; bounded virtual pitch/roll torques from host SIL or simulator-framed ESP32; mode/fault; camera X/Y errors and optional directed marker angle; sensor freshness; optional frame source/sequence, IMU/magnetometer and configured magnetometer range, yaw estimate and command acknowledgement |
| `GET /api/telemetry` | Ground Station → Plant | Most recent onboard telemetry |
| `POST /api/command` | Ground Station → Plant | Queue `PING`, `SET_MODE`, `SET_WHEEL_SPEED`, or `SET_YAW_STEP` |
| `POST /api/control` | Scene UI → Plant | Target XYZ/scale, disturbance torque, and camera occlusion |

Messages are UTF-8 JSON. All onboard messages carry `protocol_version: 1`,
source and message type. Observation and onboard sequence counters are
unsigned integers in practice; command identifiers are positive integer
milliseconds from Unix time to avoid ordinary restarts colliding. The
simulation-time timestamp is in seconds. Angles use radians, rate rad/s,
torque N·m, target coordinates metres, and image displacement pixels.
Receivers accept only a numeric protocol-version value equal to 1; booleans,
strings, fractional versions, and unsupported version numbers are invalid.

### 4.1 Commands and acknowledgement

`PING` requests a health/heartbeat response. `SET_MODE` arguments support
`SAFE`, `STANDBY`, `RATE_DAMP`, `ACQUIRE`, and `TRACK`. `SET_WHEEL_SPEED`
accepts exactly `target_rev_s` (0–50 rev/s) and `duration_s` (0.5–5 s), and is
allowed only while the onboard mode is `SAFE` or `STANDBY`. The onboard code
commands zero speed when the interval ends and remains in a stopping state
until measured speed is at most 0.5 rev/s or a 10 s stop timeout expires. A
timeout is reported as `STOP_TIMEOUT`; it is not evidence that the hardware
stopped. During the test the normal attitude torque loop is inhibited. The
physical Moteus profile additionally requires its hardware arm input and a
valid drive response. The bridge accepts a
command ID once, bounds its queue to 50, offers one command per onboard cycle,
and discards commands after 10 seconds. The onboard software ignores IDs it has
already handled and returns `command_ack: {command_id, status, reason}` in its
next telemetry. The bridge accepts an acknowledgement only for an ID it
issued, with status `ACKNOWLEDGED` or `REJECTED`; a rejected command must
include a reason. Invalid embedded acknowledgements reject the actuator packet
atomically. No command can specify wheel torque or close the pointing loop
from the Ground Station.

`SET_YAW_STEP` contains exactly `delta_deg` in [-180, 180], relative to the
current onboard yaw estimate. It is accepted in SAFE/STANDBY only when that
estimate is valid. The onboard controller settles at ≤1° error and ≤0.5°/s
for one continuous second, with a ten-second timeout. A SAFE/STANDBY command
aborts an active step. An invalid or stale IMU sample aborts the step into
SAFE. Telemetry reports target, error, elapsed time, and
RUNNING/COMPLETE/TIMEOUT/ABORTED/ATTITUDE_INVALID status. Target/error are
null before a step and after abort, timeout, or invalid attitude; COMPLETE
retains its final values. This command is available in host SIL and
source-level firmware.

### 4.2 Actuator and fail-safe semantics

For a speed test, onboard telemetry may include `motor_speed_target_rev_s`,
`motor_speed_measured_rev_s`, `motor_speed_error_percent`,
`motor_speed_test_status`, and `motor_speed_test_elapsed_s`. The simulation
bridge applies the speed target to its provisional wheel model and reports
simulated wheel feedback in observations. Terminal status remains visible in
telemetry until a new test begins. These values do not validate a real motor
speed loop.

`body_torque_nm` is ideal desired torque on the simulated body, limited by the
bridge to ±0.1 N·m; the current controller limits its requested value to ±0.05
N·m. The bridge has a 100 ms watchdog that zeros stale commands. This is a
simulation safeguard only. The plant separately models provisional wheel speed,
motor torque/lag, and saturation; `/api/telemetry` adds these as
`simulated_*` fields for local display. They are not sent in the onboard
`POST /api/actuator` packet and are not hardware measurements. Wheel momentum,
motor-controller delay/current/thermal limits, CAN-FD, and physical safe-stop
behavior are not represented by that packet.

Host SIL or simulator-framed ESP32 telemetry may include
`virtual_axis_torque_nm` with `pitch` (±0.02 N·m) and `roll` (±0.035 N·m).
Both values must be finite. The ESP32 source is accepted only when its camera
frame source is `simulated`; physical-camera firmware output cannot drive the
ideal axes. These torques are virtual plant inputs, not physical actuator
commands. Raster camera Y error is positive down; the C adapter converts it to
pitch bearing with `atan(error_y_px / focal_length_px)`, matching the host SIL
feedback sign. Roll uses the directed image orientation of the asymmetric
simulator marker, or gravity-derived leveling
when only a point is visible. Host SIL keeps Y-axis gyro rate damping active
after target loss with a 1.5× rate gain relative to visual tracking. C source and ESP-IDF/Wokwi execution still require compiler and emulator
verification.

In visual modes, both camera and IMU samples must have valid ages within the
100 ms control limit. A missing, invalid, stale, or future-dated sample faults
the controller and commands zero torque. If a fresh valid camera frame has no
detected target, visual correction drops out and fresh IMU yaw rate provides
damping only. Python SIL tests exercise these cases; C firmware source checks
cover the policy but await ESP-IDF build and runtime tests.

Synthetic camera/IMU/magnetometer contents and exact examples are in
[`Simulator_HTTP_API.md`](Simulator_HTTP_API.md). The browser scene produces a
horizontal pixel offset and target-valid flag; the current focal length and
noise and magnetic field are placeholders. SIL uses the synthetic field in a
one-axis yaw estimate and reports that estimate with its source/validity.
Physical firmware reports relative gyro-only yaw and does not fuse the raw
LIS3MDL field until calibration, sensor orientation and local reference are
established. The estimate feeds the explicit relative yaw-step mode; regular
visual pointing still uses camera error. Optional
camera, MPU-6050, and LIS3MDL adapters remain bring-up paths; they have not been
hardware-tested or integrated with a physical plant/actuator.

## 5. Physical onboard interface requirements

The following requirements apply before connecting to the real kit/radio:

- Keep target detection, sensor fusion, attitude control, and wheel command
  generation onboard. Telemetry loss must not interrupt autonomous control.
- Add a protocol version, source/packet type, sequence number, and timestamp.
- Add telemetry for operating mode, health/fault flags, sensor freshness,
  gyroscope rate, optional magnetometer XYZ when fitted, estimated yaw, target
  validity and image-plane error, wheel speed/command/status, and housekeeping
  values available on the kit.
- Add command identifier, command type, explicit parameters, and a distinct
  acknowledgement/rejection with reason. The simulation bridge requires
  integer command IDs from 1 through 2^53−1, strictly increasing, and treats
  retry of an identical, unexpired ID as idempotent; expired, conflicting, or
  older IDs are rejected. Define timeout behavior for the
  eventual physical link.
- Define operator commands for status request, mode/configuration, target
  setpoint, and test start/stop. Do not provide a ground-station command that
  closes the autonomous pointing loop remotely.
- Define message rates and payload size from measured link capacity. Send raw
  camera images only if a link budget and operator need justify them.
- Define stale-link behavior, watchdog behavior, command authorization, and
  actuator-safe responses with the hardware owner before motor testing.

The physical radio/link, addressing, authentication, packet format,
CRC/error-detection, retransmission, and link-loss strategy remain open design
decisions. Do not describe either local JSON interface as CCSDS-compatible.

The ESP-IDF packet may also include optional moteus adapter state when the
default-off physical profile is built: enabled/arm/link flags, controller mode
and fault, motor position/velocity, and measured/commanded motor torque. The
bridge preserves these extension fields and the Ground Station records them;
the plant does not treat them as simulated actuator inputs or hardware-in-loop
state.

## 6. Implementation references

- Ground Station dashboard: `../Ground_Station/main.py`
- Host onboard software-in-the-loop: `../CubeSat_ESP32/sil.py`
- Simulation environment and HTTP bridge: `../Sim_Environment/main.py`
- Legacy Level 0 Ground Station: `../Ground_Station/level0_pyqt.py`
- System architecture: `../../Design_Description.md`
