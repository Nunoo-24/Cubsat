# ESP32-S3 firmware

This ESP-IDF project contains a Wi-Fi/HTTP simulation bridge client and a
bounded yaw controller. By default it consumes synthetic camera/IMU data from
`Sim_Environment`, responds to PING/mode commands, and returns body-torque
telemetry to the simulated plant. Optional hardware adapters support camera
capture, MPU-6050 and LIS3MDL sensing. An opt-in moteus r4 UART adapter
serializes bounded torque commands and checks returned mode/fault/position/
velocity/torque. It is disabled by default, requires the physical camera and
IMU options, and has not been ESP-IDF compiled or hardware-tested.

## Build

Use ESP-IDF 5.3 or newer from this directory:

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

Reconfigure after relocation. Any old build products from the Windows OneDrive
path are stale and are not evidence that this tree builds.

## Run all portable C host tests

With GCC, Clang, or another C11 compiler installed, run from the `Python_sim`
project root:

```bash
./CubeSat_ESP32/firmware/run_host_tests.sh
```

The script compiles and runs the yaw controller, attitude estimator, and target
detector tests in a temporary directory, then removes the binaries. Set `CC` to
the compiler executable if it is not available as `cc`. These checks do not
replace an ESP-IDF build or target/emulator run. The current WSL image has no
host C compiler, so the script cannot pass here.

## Host check for the portable control module

With a host C compiler installed, run this from the `Python_sim` project root:

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -ICubeSat_ESP32/firmware/main \
  CubeSat_ESP32/firmware/main/yaw_controller.c \
  CubeSat_ESP32/firmware/test_yaw_controller.c \
  -lm -o /tmp/ae-yaw-controller-test
/tmp/ae-yaw-controller-test
```

The current WSL image has no host C compiler, so this check has not been run.
The Python control reference tests are a separate check and do not validate C.

## Host check for the yaw attitude estimator

The estimator uses only the C standard library and can be checked separately:

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -ICubeSat_ESP32/firmware/main \
  CubeSat_ESP32/firmware/main/attitude_estimator.c \
  CubeSat_ESP32/firmware/test_attitude_estimator.c \
  -lm -o /tmp/ae-attitude-estimator-test
/tmp/ae-attitude-estimator-test
```

This host check has not run in the current WSL image because no C compiler is
installed. Python estimator tests are not a substitute for compiling the C
implementation.

The portable target detector has its own host test. From the `Python_sim`
project root, run:

```bash
gcc -std=c11 -Wall -Wextra -Werror \
  -ICubeSat_ESP32/firmware/main \
  CubeSat_ESP32/firmware/main/black_target.c \
  CubeSat_ESP32/firmware/test_black_target.c \
  -lm -o /tmp/ae-black-target-test
/tmp/ae-black-target-test
```

## Runtime loop

The firmware connects to the configured Wi-Fi access point and exchanges
version 1 HTTP/JSON messages with the simulation bridge every nominal 20 ms.
The build defaults to `CONFIG_FREERTOS_HZ=100`, so the 20 ms period is two
FreeRTOS ticks. The control task uses `xTaskDelayUntil()` for fixed-deadline
scheduling and rebases after a missed deadline to avoid catch-up bursts. The
configured tick rate must be a multiple of 50 Hz. This is nominal scheduling;
the control-cycle runtime and jitter still require measurement on Wokwi and
hardware.
The control loop currently runs inside ESP-IDF's `main` task. The project sets
`CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192` bytes because the synchronous HTTP path
uses a 2048-byte body buffer plus nested library calls. A debug-level log samples
the main/control task's minimum free stack every 250 cycles (5 s); enable Debug
logging to view it. Check that minimum during normal and worst-case transfers
and retain margin before changing this setting. This is an initial sizing, not
a verified stack requirement; target build and high-water measurement remain
open.
The Wokwi profile defaults to `Wokwi-GUEST` and
`http://host.wokwi.internal:8000`; enable the Wokwi private gateway so the
simulated board can reach the local host bridge. Wi-Fi reconnect is retried
after disconnect; if bridge requests fail, the simulation bridge's 100 ms
watchdog zeros body torque. This bridge is a simulation transport only. It is
not the physical TM/TC protocol.

For the physical XIAO ESP32-S3 Sense camera, enable `CUBESAT_CAMERA_ENABLED`
and ESP32-S3 PSRAM in `idf.py menuconfig`. The camera adapter captures one
grayscale QVGA frame per control cycle and uses the onboard dark-pixel centroid
detector. Python and C retain a thresholded pixel only when at least two other
dark pixels occupy its 8-neighbourhood, then apply the configured coherent-area
bounds. Confirm the installed OV3660/OV5640 variant, threshold, target pixel
area, camera orientation, and focal length before using its output. Each frame
is timestamped on capture; the controller checks its age with the selected IMU
sample rather than using the synthetic camera timestamp. The camera driver's
frame timestamp marks the first DMA buffer since boot, so freshness includes
frame acquisition and processing delay. A missing, zero, or future timestamp
is rejected. A target not found in a fresh frame disables visual correction
and leaves gyro-rate damping active. A failed capture or invalid/stale camera
age fails closed to zero torque, as does invalid or stale IMU data. Lost-target
damping uses 0.06 N·m·s/rad, versus 0.04 during visual tracking. The C source
and host test reflect this, but firmware is not compiled in this environment.
Leave the
physical camera option disabled for Wokwi;
that profile fetches a sequence-matched raw QVGA frame from the simulator
bridge and runs the portable C detector. A fresh frame with no target permits
gyro-rate damping; a missing or expired frame fails closed to zero torque.
With the physical camera enabled, a 5x5 box-averaged 64x48 grayscale
thumbnail is uploaded once per second as 3072 raw bytes to `/api/camera-preview`; the
bridge pairs it with X/Y centroid errors from the matching telemetry packet,
and the Ground Station displays the image/error overlay while fresh (2 s).
Telemetry image coordinates use +X right and +Y down; yaw control consumes
horizontal error only. The camera adapter and upload
path have not yet been compiled or validated on hardware.

For the physical MPU-6050 breakout, enable `CUBESAT_MPU6050_ENABLED` in
`idf.py menuconfig`. The default bus is XIAO D4/GPIO5 SDA and D5/GPIO6 SCL at
100 kHz; the sensor address defaults to 0x68 and can be set to 0x69. The sensor
is configured for ±250°/s and ±2 g, and the firmware averages 200 startup gyro
samples for bias while the board is stationary. Provide suitable external I²C
pull-ups and verify the breakout supply/logic levels before connecting it. A
read failure or missing sensor inhibits torque output in this firmware path.
This path remains uncompiled and requires bench verification.

For the optional LIS3MDL magnetometer, enable `CUBESAT_LIS3MDL_ENABLED` and
choose address 0x1C or 0x1E in `idf.py menuconfig`. It shares the same I²C bus,
samples at 80 Hz with a data-ready check, and reports a ±4 gauss raw vector in
µT for telemetry only; its frame is labelled `sensor` until the physical
mounting transform is measured. No heading fusion or motor-interference
compensation is implemented. The LIS3MDL chip supply is 1.9–3.6 V, so check
the exact breakout's regulator and logic
level handling before connecting it. This driver is uncompiled and
unbench-tested.

For physical actuation, keep `CUBESAT_MOTEUS_ENABLED` disabled in Wokwi/SIL.
The option is available only when both physical camera and MPU-6050 adapters
are enabled. Defaults use XIAO D0/GPIO1 for TX and D1/GPIO2 for RX at
921600 8N1, keeping the adapter clear of the usual D6/D7 UART0 console pins;
verify board wiring and electrical levels before use. The driver
requires a separate active-high arm input on D2/GPIO3. Its internal pulldown
leaves the actuator disarmed when open. An independent motor power cut remains
required. Dropping the arm input latches the adapter offline until firmware
restart. The default software torque clamp is 1 mN·m, configurable up to
20 mN·m; confirm the value against the motor, wheel and rig before arming. The
driver applies a finite 100 ms moteus command watchdog and latches offline
when a reply reports a drive fault or unexpected mode. It sends opposite-sign
motor torque for the reaction wheel. A failed UART exchange lets the moteus
watchdog expire.

The adapter performs a zero-torque/status handshake at startup and requires a
fresh valid mode/fault/position/velocity/torque reply for each command. It has
no calibrated wheel-speed trip or project-specific emergency-stop circuit.
The serialized request assumes the moteus CAN prefix is zero and uses the
configured controller ID (default 1); read both values from the actual device.
Do not enable it until wiring, independent power isolation, limits and a safe
test procedure are confirmed. Source implementation does not establish
electrical compatibility or safe physical behavior.

The host-side UART protocol reference can be checked from the `Python_sim`
root with:

```bash
python3 -m unittest CubeSat_ESP32.test_moteus_protocol
```

This checks the torque and native velocity wire-format references; it does not
compile or execute the ESP-IDF C adapter.

## moteus r4.11 actuator adapter

The optional adapter is in `main/moteus_uart.c/.h`; it sends checked fdcanusb
UART lines carrying 64-byte CAN-FD register frames. The current application
calls the torque API: it applies feedforward torque with zero feedback gains,
a configurable finite torque limit, and a 100 ms watchdog, then validates the
returned mode, fault, position, velocity and torque. A fault/mode mismatch
latches the adapter offline until restart. The body-to-motor sign is inverted
for reaction-wheel torque.

The adapter also has a separate native velocity API, limited to ±50 rev/s
(±3000 rpm), with a 100 ms watchdog and the configured torque ceiling. The
command sets position to NaN and supplies velocity in rev/s, leaving moteus
feedback gains at their configured values. The CubeSat application calls it
only during the bounded SAFE/STANDBY commissioning state, which Ground Station
can request and observe. Terminal stop status remains in telemetry until a new
test starts so the dashboard can display it. This path is source reviewed only;
compile and UART tests are still required. The Python
`moteus_protocol_reference.py` has unit-tested torque and velocity frame
encoders, but that does not prove the C bytes match until the C implementation
is built and compared.

Do not connect ESP32-S3 TWAI to moteus CAN-FD: S3 TWAI supports Classical CAN
only. If the external CAN-FD route is required, use an FD controller over SPI
and a CAN-FD transceiver. The UART adapter defaults to XIAO D0/GPIO1 TX and
D1/GPIO2 RX, and needs an active-high arm input on D2/GPIO3. Verify the exact
controller firmware, connector pins, logic levels and arm/power-cut circuit
before enabling the Kconfig option. See
[`../../docs/System_Architecture_and_Interfaces.md`](../../docs/System_Architecture_and_Interfaces.md)
and [`../../../../references/Hardware_and_Emulation_Notes.md`](../../../../references/Hardware_and_Emulation_Notes.md).

## Wokwi

`wokwi.toml` and `diagram.json` select the XIAO ESP32-S3 board. Build the
firmware, enable Wokwi's private gateway, then use the Wokwi VS Code extension
to start the simulation. This configuration has not yet been built or run in
this environment. See the project root README for the full three-process host
simulation commands.
