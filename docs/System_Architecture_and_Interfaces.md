# System architecture and interfaces

**Revision:** 0.2 — integrated software-in-the-loop baseline  
**Status:** host SIL path implemented; firmware build/emulation and hardware
interfaces still require verification

## Responsibility split

| Element | Owns | Does not own |
|---|---|---|
| `Sim_Environment` | Simulated gimbal motion, external torque, target geometry, synthetic QVGA camera frames/IMU data, plant UI, bridge watchdog | Target choice, pointing control, or physical motor commands |
| `CubeSat_ESP32` | Target-bearing extraction from camera pixels, yaw-rate feedback, one-axis yaw estimation, mode logic, bounded control, fault response, actuator telemetry | Ground-station-dependent control |
| `Ground_Station` | Telemetry display/CSV log, link health, PING and mode telecommands | Image processing, pointing control, direct wheel-torque commands |

## Integrated software-in-the-loop path

```mermaid
sequenceDiagram
    participant UI as Sim Environment UI
    participant Plant as Three-axis gimbal plant + HTTP bridge
    participant MCU as ESP32-S3 firmware or host SIL
    participant GS as Ground Station
    UI->>Plant: target, external torque, occlusion
    MCU->>Plant: GET /api/cycle (IMU + camera frame sequence + command)
    Plant-->>MCU: version 1 observation with frame sequence
    MCU->>Plant: GET /api/camera-frame.rle?sequence=N (host SIL or Wokwi)
    Plant-->>MCU: paired QVGA grayscale frame
    MCU->>MCU: Python/C target detection + pixel bearing + gyro rate + bounded control
    MCU->>Plant: POST /api/actuator (body torque + telemetry)
    Plant->>Plant: integrate yaw; zero stale command after 100 ms
    GS->>Plant: proxied telemetry poll / supervisory command
    Plant-->>GS: latest onboard telemetry / command acknowledgement
```

For local software testing, `CubeSat_ESP32/sil.py` runs the host reference at
50 Hz, rebasing after an overrun instead of catching up with back-to-back
cycles. For target-code simulation, the ESP-IDF app polls the same bridge
from Wokwi over HTTP/Wi-Fi using a 20 ms `xTaskDelayUntil()` schedule at the
configured 100 Hz RTOS tick. It rebases its schedule after missed deadlines.
The ESP32 controller makes the target and torque decisions. The simulation
bridge supplies a sensor scenario and applies the commanded body torque. The
Ground Station monitors telemetry and issues `PING`, enumerated operating-mode,
bounded wheel-speed commissioning, and relative yaw-step commands.

The simulated camera uses a lossless RLE1 transport envelope for host SIL and
Wokwi; each client reconstructs the exact QVGA image before its onboard
detector runs. The deterministic default frame encodes to 91 bytes (36.4
kbit/s at 50 Hz); payloads above 8,192 bytes are rejected. This compresses the
synthetic flat-background scene and says nothing about noisy-camera compression
or delivered rate. Wokwi gateway latency, sustained throughput, and deadline
behavior remain unmeasured, so real-time operation is unverified.

## Interface and protocol boundary

The running SIL bridge is versioned HTTP/JSON documented in
[`Simulator_HTTP_API.md`](Simulator_HTTP_API.md). The legacy UDP/JSON heartbeat
example remains separate. The new bridge supports environment observation,
actuator telemetry, and supervisory commands for mode, wheel-speed commissioning,
and relative yaw-step testing.

The simulation uses HTTP because it is directly reachable from the Wokwi
private host gateway. This is an emulator/SIL transport, not a choice of radio
or physical TM/TC protocol. Physical Wi-Fi/UDP, packet integrity, authentication,
retries, address management, and the meaning of any CCSDS-inspired frame still
require an explicit engineering decision. Keep all autonomous measurement and
feedback onboard after replacing synthetic observations with real sensors.

## Simulated physics and sensors

The model has three decoupled rotational axes. The simulation body frame maps
X torque to pitch, Y torque to yaw, and Z torque to roll; confirm this frame
against the physical mount before hardware use. Yaw is a rigid-body axis with
inertia `0.02 kg·m²` and viscous damping `0.002 N·m/(rad/s)`. Roll/pitch use
linear pendulum restoring terms with `0.02 kg·m²` inertia, `0.002 N·m/(rad/s)`
damping, and `0.02 N·m/rad` restoring stiffness per axis. These are software
placeholders, not measured hardware values. Euler-rate coupling, cross-axis
inertia, and large-angle gimbal effects are omitted. A semi-implicit Euler
update runs at a 10 ms nominal period. The requested body torque drives a
provisional reaction-wheel model with motor torque, first-order lag, rotor
speed integration, a speed limit, and equal/opposite body reaction torque.
Initial wheel inertia (1e-4 kg·m²), torque limit (0.01 N·m), speed limit
(500 rad/s), and 50 ms time constant are unmeasured placeholders, not hardware
specifications. Motor/controller bus delay, current/thermal limits, wheel
imbalance, desaturation, and moteus response are not modeled. A simulation
watchdog zeros requested torque if actuator telemetry is absent for 100 ms.

The target is a point at XYZ in the simulated world, with positive Z in front
of the camera. Camera bearing uses the full body rotation and provisional 277
px focal length. Each observation generates a paired raw QVGA grayscale frame:
a bright background and a dark circular point projected at the calculated
pixel location and size. Host-SIL frames add an asymmetric three-dot fiducial
whose directed image angle represents the target's world-up reference. Python
and portable C detectors estimate its directed angle from image moments; a
featureless point returns no roll angle. The synthetic marker is a design
addition for simulation. A physical poster needs an equivalent resolved
orientation feature for camera roll to be observable. Apparent disk size uses
pinhole projection from optical-axis depth and is not used as a target-range
measurement. A five-pose roll/pitch/yaw regression matches camera X/Y pixels to
an independently assembled rigid-body rotation matrix; this checks frame/sign
consistency, not lens calibration. Host SIL fetches that exact decoded frame
and runs the Python detector before control; the Ground Station shows a
5x5-averaged thumbnail of the same frame. The default Wokwi profile fetches
and decodes RLE1 before running the portable C detector; the physical camera
profile uses its locally captured frame. Telemetry uses positive X right and
positive Y down, while the environment projection reports positive Y up. The
portable detector's pixel Y is positive down; firmware converts it to the
pitch-control bearing with `atan(error_y_px / focal_length_px)`, matching the
host SIL sign after its environment-to-image coordinate conversion. A source
contract guards this sign, but firmware execution remains unverified. The
image has no lens distortion, lighting variation, noise or motion blur.
Deterministic gyro bias/noise is added; the accelerometer returns idealized
gravity projected into the body frame. A fixed placeholder world magnetic
vector is rotated into body coordinates for telemetry-interface tests; it is
not location-specific and is not used by control. Rigid-body axis coupling is
omitted. Host SIL uses image X/Y and directed marker orientation for all three
camera axes. Point-only frames use IMU roll fallback. The C/Wokwi firmware path
has a portable pitch/roll controller and sends bounded virtual torque to the
plant only for simulator-sourced frames; physical actuator output remains the
yaw reaction wheel.

The portable Python control reference and the C firmware module implement a
PD law with a 0.05 N·m output bound, 100 ms observation-age limit, ±20 rad/s
rate limit, and rate damping during target loss. Those are initial values for
SIL only. The firmware adapter converts image pixel error to angle with
`atan(pixel_error_x / focal_length_x_px)`. Actual gain tuning and sign
calibration require the assembled plant and motor safety limits.

The Python integration suite also exercises a complete visual acquisition using
a sequence-paired RLE1 image through the environment handlers, SIL decoder,
detector/controller, plant actuator handler and Ground Station telemetry
handler in memory. It verifies both detector-derived error and plant truth
reach the guide acquisition bound with bounded control torque; it does not
exercise TCP or browser rendering.

The Python regression suite checks proposed guide criteria in the idealized
model: acquisition from 30° to within 2° in under 6 simulated seconds, a
one-minute hold within ±2°, recovery from a 20° yaw displacement in under 4
simulated seconds, tracking a target moving at 0.2°/s within ±2° for one minute,
and recovery after a two-second camera occlusion. These are software-model
checks; the guide labels the project performance values as targets to confirm,
and no physical performance is claimed.

## Hardware and implementation boundary

- **XIAO ESP32-S3 Sense:** onboard computer selected by the user. Seeed lists
  Sense camera modules with OV3660 and OV5640 variants; verify the kit revision.
- **Camera:** a portable grayscale detector retains thresholded pixels with at
  least two dark 8-neighbours before applying coherent-area limits and computing
  the centroid. Python SIL and C firmware use the same rule; raw thresholded area
  and retained coherent area are reported separately by the detector.
  in `CubeSat_ESP32/firmware/main/black_target.c`. The optional hardware
  adapter in `xiao_camera.c` initializes the XIAO Sense DVP pin map, captures
  grayscale QVGA frames, and passes them to the detector. Enable it with
  `CONFIG_CUBESAT_CAMERA_ENABLED` and enable ESP32-S3 PSRAM in the ESP-IDF
  configuration. Wokwi leaves it disabled and uses synthetic pixel error.
  Each physical frame carries a local capture timestamp; camera and IMU ages
  are checked independently. Visual tracking requires fresh camera and IMU
  samples. If a fresh frame contains no valid target, tracking drops out and
  fresh IMU data can provide yaw-rate damping. A missing, invalid, stale, or
  future-dated camera or IMU sample fails closed and inhibits torque.
  The detector assumes one dark target on a bright field, uses configurable
  threshold/area limits, and has no connected-component filtering. The focal
  length default is provisional and must be calibrated for the lens and
  resolution. This adapter is source implemented but not compiled or tested on
  a board; when enabled, it downsamples each frame to 64x48 grayscale and
  posts a 3 KB preview once per second to the local bridge. The Ground Station
  pairs each preview with its frame's X/Y error metadata and displays it on a
  320x240 canvas. When no physical preview is fresh, host SIL gets a simple
  grayscale target frame rendered from the same telemetry; the bridge labels
  the source explicitly and never substitutes it for a missing ESP32 frame.
  Fresh previews expire after two seconds. This is not a physical radio image
  protocol. When enabled, the current firmware still receives synthetic IMU
  and plant data,
  so this is camera bring-up only, not a coherent hardware-in-the-loop test.
- **MPU-6050:** optional I²C driver uses XIAO D4/GPIO5 SDA, D5/GPIO6 SCL,
  configures ±250°/s and ±2 g, and averages 200 startup gyro samples while
  stationary. Wokwi defaults to synthetic IMU observations. Driver source is
  implemented but not compiled or bench-tested; calibration and sample timing
  need hardware validation. A portable yaw estimator uses accelerometer gravity
  for tilt compensation and the fixed synthetic magnetic vector to correct
  gyro drift in SIL/Wokwi. Physical firmware uses relative gyro-only yaw until
  LIS3MDL calibration, sensor orientation and a field reference are established.
  The estimate drives the explicit relative yaw-step test; ordinary visual
  acquisition/tracking continues to use camera bearing error.
- **LIS3MDL:** optional driver shares the MPU-6050 I²C bus and reports XYZ
  field values scaled to µT with Kconfig-selectable ±4/8/12/16 gauss range
  (±4 default). SPI is not implemented. The host simulation rotates a fixed
  placeholder field for interface tests and SIL uses that synthetic field in
  the yaw estimate. Physical raw samples are not fused until calibration and
  motor-interference testing.
  Breakout voltage, magnetic calibration, placement and motor interference
  must be checked. ST marks the part out of production.
- **moteus r4.11 + reaction wheel:** moteus specifies CAN-FD frames with 1 Mbit/s
  arbitration and 5 Mbit/s data rates. ESP32-S3 TWAI is Classical CAN only, so
  the built-in TWAI plus a simple CAN transceiver cannot communicate with that
  protocol. The moteus r4 documentation also supports TTL UART control on AUX2
  (default 921600 8N1 on sufficiently recent firmware). For one controller,
  UART is the selected software adapter path, pending confirmation of the
  actual kit and electrical interface; alternatively an SPI CAN-FD controller
  such as MCP2518FD plus a CAN-FD transceiver could provide a CAN-FD route.
  The default firmware profile reports desired body torque to the model. When
  `CUBESAT_MOTEUS_ENABLED` is explicitly enabled with physical camera/IMU and
  arm input, the optional adapter also sends motor commands. That C path is
  source implemented but uncompiled and not hardware-verified. The simulation
  wheel parameters are placeholders; motor limits and safe-stop behavior are
  not hardware-verified.

## Reaction-wheel actuator interface status

The software plant accepts desired net body torque; the physical controller
accepts motor-side commands. These are not interchangeable. The wheel applies
body torque opposite to motor torque, and wheel momentum/speed limits must be
enforced using measured hardware parameters.

The moteus r4 UART is the selected software implementation path for one
physical controller, pending kit inspection. The official guide specifies TTL
UART on AUX2 at 921600 baud, 8N1, using an fdcanusb-compatible line protocol
that carries CAN-FD register frames. The optional adapter in
`CubeSat_ESP32/firmware/main/moteus_uart.c` implements checked line encoding,
CRC-8/0x97, torque register serialization, status/fault response parsing, a
finite torque clamp, a 100 ms drive watchdog, arm input, and fault latch.
Torque mode uses position mode with NaN position, zero velocity and feedback
scales, plus feedforward torque and finite maximum torque. Manufacturer docs
note lower bandwidth than the internal position controller.

The option is disabled by default and requires physical camera/IMU config. It
uses XIAO D0/GPIO1 TX, D1/GPIO2 RX and an active-high arm input on D2/GPIO3
unless configured otherwise. This C path has not been compiled, UART tested,
or hardware verified. Exact r4 firmware/config, connector wiring/logic,
motor/wheel limits and an independent emergency stop remain open. See the
implementation guide and primary references in
[`../../Subsystems/Actuation/README.md`](../../Subsystems/Actuation/README.md)
and [`../../../references/Hardware_and_Emulation_Notes.md`](../../../references/Hardware_and_Emulation_Notes.md).

Neither Wokwi nor QEMU can validate the actual camera processing time, IMU
calibration, CAN-FD physical layer, real motor response, or actuator safety.
Use software simulation, target emulation, and hardware-in-the-loop as distinct
validation stages and label results accordingly.

## Primary sources

- Espressif, [ESP32-S3 QEMU emulator](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-guides/tools/qemu.html).
- Wokwi, [ESP32 simulation and board support](https://docs.wokwi.com/guides/esp32), [ESP-IDF integration](https://docs.wokwi.com/vscode/esp-idf), and [Wi-Fi/private gateway](https://docs.wokwi.com/guides/esp32-wifi).
- Seeed Studio, [XIAO ESP32-S3/Sense guide](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/).
- STMicroelectronics, [LIS3MDL product page and datasheet](https://www.st.com/en/mems-and-sensors/lis3mdl.html).
- TDK/InvenSense, [MPU-6000/MPU-6050 product specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf).
- mjbots, [moteus r4.11 and CAN-FD](https://github.com/mjbots/moteus).
- mjbots, [moteus CAN-FD protocol](https://mjbots.github.io/moteus/protocol/can/) and [TTL UART integration](https://mjbots.github.io/moteus/integration/uart/).
- Espressif, [ESP32-S3 TWAI constraints](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/api-reference/peripherals/twai.html) and [SPI master interface](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/spi_master.html).
- Microchip, [MCP2518FD SPI CAN-FD controller](https://www.microchip.com/en-us/product/mcp2518fd).
- OpenCV, [pinhole projection model](https://docs.opencv.org/5.0/main_modules/d_projection.html).

Additional part-specific findings are in `../../references/Hardware_and_Emulation_Notes.md`.
