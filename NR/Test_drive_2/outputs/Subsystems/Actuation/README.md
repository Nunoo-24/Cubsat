# Reaction-wheel actuation

**Role:** convert onboard yaw-control output into a wheel-motor command, monitor actuator response, and enforce safe start/stop behavior.

**Hardware described:** 5208 Kv330 brushless motor, reaction wheel and moteus r4.11 controller. Confirm exact motor, wheel, supply, controller revision, firmware and configuration against the kit.

**Current state:** the simulated plant has a provisional wheel inertia/torque/speed/lag model. An optional ESP-IDF UART adapter is source implemented behind `CUBESAT_MOTEUS_ENABLED`; it is disabled by default and has not been compiled or hardware-tested. The default profile reports desired body torque to the simulation bridge; an explicitly enabled physical profile also sends motor commands and reports adapter status.

## Interface finding

ESP32-S3 built-in TWAI supports Classical CAN only; moteus CAN uses CAN-FD. A simple CAN transceiver cannot bridge that difference. The moteus r4 AUX2 is documented for TTL UART on a JST ZH-4 connector, 921600 baud, 8 data bits, no parity, one stop bit, on sufficiently recent firmware. The TX/RX pins are documented as host TX → moteus RX and host RX ← moteus TX.

On UART, moteus emulates fdcanusb. The host sends newline-terminated ASCII commands that carry CAN-FD frames; it is not a direct textual motor-command API. UART checksums are CRC-8 with polynomial 0x97; the official client includes a space before `*XX` in the CRC scope. Once checksummed commands are used, subsequent commands need checksums and responses include them. A host UART supports one moteus controller. The UART path is now implemented as an optional firmware adapter, conditional on inspection of the actual controller and board.

## Torque command contract

The official moteus control-mode guide expresses torque control through position mode with a special NaN position, zero velocity, zero proportional/derivative/integral-limit scale, and feedforward torque. The adapter also writes a finite maximum torque and 100 ms watchdog, and requests mode, fault, position, velocity and torque on each command. It rejects invalid replies; a returned fault or unexpected mode latches physical output offline. A missing UART response leaves the moteus watchdog to expire. The configured loop period and watchdog still need timing measurements. moteus warns that external torque control has substantially lower bandwidth than using its internal position controller.

The sign conversion must be explicit: the motor applies torque to the rotor and the body receives the equal-and-opposite reaction. The adapter sends the negative of requested body torque as motor torque, assuming the documented body/wheel axis orientation. Confirm this sign on the rig before enabling actuation. Never assume the current simulated wheel constants are hardware limits.

## Speed-setpoint milestone

The project guide's 3000 rpm / 0.5 s milestone is a separate operating mode.
The Ground Station exposes a bounded commissioning request (0–50 rev/s,
0.5–5 s); the bridge, host SIL and source-level ESP-IDF app route it through
the native velocity API. Requests require SAFE/STANDBY and the physical profile
also requires the arm input. The normal attitude path sends feedforward torque
with the moteus internal position/velocity gains set to zero. mjbots documents native velocity
control as position mode with position set to NaN and velocity set in
revolutions per second. Thus 3000 rpm corresponds to 50 rev/s. The UART driver
contains a separate bounded `cube_moteus_command_velocity()` frame builder and
the application calls it only during the dedicated speed-test state. Python
integration tests cover the Ground Station-to-model path. Firmware remains
uncompiled; the test does not establish UART interoperability or real speed
tracking. Performance still requires measured motor/wheel parameters, verified
acceleration/torque limits, and a safe physical test configuration. Do not
send this setpoint through the attitude torque interface.

The current plant's best-case wheel spin-up reaches about 431 rpm after 0.5 s
with its placeholder 1e-4 kg·m² inertia, 0.01 N·m torque limit, and 0.05 s
torque lag. Even the constant-torque ideal calculation requires 62.8 mN·m to
reach 3000 rpm in 0.5 s at that inertia. Those values only show that the
current model cannot validate the milestone; they are not estimates of the
actual wheel or motor. Source review also confirms that the ESP-IDF native velocity frame writes
the same Kconfig maximum-torque value used by the torque frame: default
1 mN·m, configurable only from 1 to 20 mN·m. At the model inertia, an ideal
20 mN·m constant torque reaches only about 955 rpm in 0.5 s; 3000 rpm needs
62.8 mN·m. Conversely, at the 20 mN·m cap the total wheel inertia must be no
more than 3.18×10⁻⁵ kg·m² (without losses), versus the provisional model
1×10⁻⁴ kg·m². The default cap permits only 1.59×10⁻⁶ kg·m². These are source
and ideal-physics bounds, not motor ratings. The native command being present
does not show that the requested milestone is attainable with current
firmware settings. The setting is `CUBESAT_MOTEUS_MAX_TORQUE_MNM` in
[`Kconfig.projbuild`](../../Python_sim/CubeSat_ESP32/firmware/Kconfig.projbuild);
[`moteus_uart.c`](../../Python_sim/CubeSat_ESP32/firmware/main/moteus_uart.c)
writes it into both torque and native-velocity frames.

## Implementation sequence

1. Confirm r4 revision, firmware, controller ID and `can.prefix=0`; identify AUX2 connector pins/electrical levels; verify XIAO D0/D1 wiring and the external D2 arm circuit.
2. Build the `CUBESAT_MOTEUS_ENABLED` profile in ESP-IDF and fix any API/compiler issues. The adapter is compile-time disabled by default, requires physical camera and IMU profiles, and latches disarmed until restart after the arm input is dropped. The bounded commissioning state is now present; validate its command frame and requested/measured speed reporting on a disconnected drive before powered tests.
3. With motor power disconnected, verify checked UART lines, CRC, response parsing and zero-torque handshake. Confirm returned mode/fault and watchdog behavior.
4. Check torque sign and limits with a safe, mechanically constrained setup and an independent emergency power cut.
5. Measure actuator response, wheel inertia/speed limits, timing and current/thermal limits; then tune control and replace plant placeholders.

An external CAN-FD route remains possible if required: it needs both an FD-capable controller (for example MCP2518FD over SPI) and a CAN-FD transceiver. ESP32-S3 TWAI plus a transceiver is not sufficient.

## Evidence and open items

Primary-source links and the detailed hardware findings are in [`../../../references/Hardware_and_Emulation_Notes.md`](../../../references/Hardware_and_Emulation_Notes.md). Protocol references: [moteus UART](https://mjbots.github.io/moteus/integration/uart/), [fdcanusb line protocol](https://github.com/mjbots/fdcanusb/blob/master/README.md), [moteus CAN register protocol](https://mjbots.github.io/moteus/protocol/can/), [control modes and native velocity mode](https://mjbots.github.io/moteus/guides/control-modes/), and [register reference](https://mjbots.github.io/moteus/protocol/registers/).

**Not verified:** C compilation, physical pin mapping/voltage, current controller ID/configuration, torque/wheel limits, communication timing, fault response, safe-stop procedure, and physical actuation. No motor test is implied by this source implementation.
