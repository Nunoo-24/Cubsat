# AE project — working interpretation

## What the project appears to be

Design, simulate, and demonstrate a CubeSat-style attitude determination and
control system. The proposed demonstrator actively points in yaw with a
reaction wheel; the guide's gimbal has three rotational freedoms, while
roll/pitch are pendular modes or disturbances. A camera detects a
high-contrast black target on a poster; an onboard processor estimates
pointing error and commands a brushless motor with a reaction wheel. The wheel
changes its angular momentum and turns the suspended cube in the opposite
direction. An IMU supplies fast rotational measurements; camera observations
provide a slower visual reference. The intended result is for the cube to
acquire and hold the target autonomously. The permitted early bench
configuration (full gimbal or locked roll/pitch) still needs confirmation.

The user confirmed that all autonomous CubeSat processing runs on the XIAO ESP32-S3. The PC ground station is supervisory and is not part of the autonomous control loop.

## Intended development path

The guide describes seven stages, beginning in simulation and moving to hardware:

0. Set up the environment and establish telemetry to a computer ground station.
1. Control a motor to a speed setpoint.
2. Read inertial measurements and estimate orientation.
3. Close the attitude loop and command angular steps.
4. Detect the visual target and estimate image/pointing error.
5. Use that error for autonomous pointing.
6. Demonstrate disturbance rejection, moving-target tracking, and recovery from camera occlusion on the physical setup.

The guide emphasizes reusing the same control code from simulator to hardware, with model/SIL/PIL/HIL validation as increasingly realistic validation stages. It describes a gimbal/suspension with three rotational freedoms, but only yaw is actively controlled by the reaction wheel in the proposed one-axis demonstrator; roll and pitch are treated as pendular modes/disturbances to damp.

## Performance targets in the project guide

- From 30 degrees off target, acquire the target in under 6 seconds.
- Stay within ±2 degrees of the target for one minute.
- Recover from an approximately 20-degree nudge in under 4 seconds.
- Follow a slowly moving target.
- Recover without instability after the camera is covered for 2 seconds.
- Earlier simulation milestones include 3000 rpm in under 0.5 seconds with under 2% error, IMU orientation error under 2 degrees and drift under 2 degrees/minute, and a 45-degree step settling within ±1 degree.

The guide labels the motor speed tolerance as to be confirmed and these values should be treated as proposed project criteria until verified with the lecturer/team.

## Deliverables and course context

The guide names a TM/TC interface control document (ICD), design description, ground-station software, and CubeSat firmware as report deliverables. The course syllabus identifies a four-person group project and a group report due 7 December 2026, plus an individual oral presentation/discussion. AI use is allowed for the project/report under the syllabus conditions: log usage, verify generated values/formulas against cited primary sources, and take responsibility for submitted work. AI is forbidden in the written exam.

## Design decisions that need reconciliation

1. **Axis constraints:** `Dicas.txt` proposes one free axis and two locked axes before expanding. The project guide describes a three-ring, three-freedom suspension; yaw is controlled and roll/pitch are pendular/damped. Confirm the permitted initial prototype configuration with the team/lecturer.
2. **Sensor fusion:** The guide explains camera + IMU fusion. The current software adds a yaw-only complementary estimate: gyro Y integration corrected by tilt-compensated magnetic heading. SIL corrects against its fixed synthetic field; physical fusion is disabled until field calibration, mounting transform, and magnetic interference are characterized. The estimate remains telemetry for normal visual pointing, but the explicit 45° yaw-step milestone now uses it as a relative setpoint in the Python SIL and source-level firmware.
3. **Camera model:** The project guide lists OV3660 and OV5640 options, while the personal notes name OV5640. Confirm the actual kit camera before finalizing image-processing assumptions.
4. **Visual output:** A monocular image can directly provide 2D image-plane error to the target; a three-dimensional position error is not established by the notes. For the one-axis yaw controller, define the exact measured image/angle error and coordinate convention.
5. **Communication link/protocol:** Notes mention Wi-Fi and a simplified CCSDS-like protocol; the project guide calls for a TM/TC ICD but does not settle the radio/link or protocol. Choose these explicitly rather than implying CCSDS compatibility.

## Current software snapshot

See `Context/CODE_REVIEW.md` and `outputs/Python_sim/README.md` for the current
code inventory and exact verification limits. The Python SIL connects the
browser three-axis plant, synthetic camera/IMU data, onboard control reference, and
Ground Station through a versioned local HTTP/JSON bridge. ESP-IDF firmware has
a Wokwi bridge client, bounded yaw controller, one-axis yaw estimator, optional
XIAO Sense camera capture/dark-target detector, optional MPU-6050 I²C driver,
and a separate timed wheel-speed commissioning path that calls the optional
moteus velocity adapter. Firmware builds, emulator execution, physical motor/CAN, and integrated
HIL are still unverified. The host bridge/SIL/Ground Station path has passing
integration tests; wheel-speed performance still fails against the placeholder
plant bound and has no measured hardware evidence.
The host plant models decoupled pitch/yaw/roll. Yaw is controlled by a
first-pass reaction-wheel model. Following the user's request for full
directional simulated pointing, host SIL drives pitch from vertical image
error and roll from an optional asymmetric marker orientation (falling back to
gravity-derived leveling) using bounded virtual plant torques. ESP32/Wokwi
source now emits the same bounded pitch/roll torques only for simulator-framed
observations. The physical motor path remains yaw-only. These
pitch/roll inputs do not exist in physical firmware or a selected actuator
design. Body frame X=pitch, Y=yaw, Z=roll. Inertias, torque/speed limits,
stiffness, gains and lag are placeholders, not hardware-calibrated.

## Hardware and software described

- Seeed Studio XIAO ESP32-S3 Sense; the user's input names an OV5640 camera
  evaluation board. Confirm the actual module fitted to the kit because Seeed
  also documents OV3660/OV2640 variants.
- MPU-6050 3-axis gyro/accelerometer; the user input calls its breakout “5.5V”, but the bare chip is specified for lower supply/logic levels, so the exact module and I²C pull-ups must be checked. The magnetometer range/interface description matches the LIS3MDL family and likely an Adafruit breakout, but vendor/revision are not confirmed.
- Brushless 5208 Kv330 motor, moteus r4.11 controller over CAN-FD, reaction wheel, onboard battery, and a gimbal/suspension.
- Python ground station/simulator tools; ESP-IDF C/C++ firmware; Git for version control. The guide marks simulator tool details as still to be confirmed.

The specific purchased/available components and their exact revisions remain to be confirmed against the lab kit and data sheets.

### Newly verified actuator interface constraint

The ESP32-S3's built-in TWAI controller supports Classical CAN, not CAN-FD,
while the moteus CAN protocol specifies FD frames. A plain CAN transceiver
cannot bridge that capability gap. moteus documents a TTL UART control path on
r4 AUX2 at 921600 8N1 by default on sufficiently recent firmware; this looks
as the selected software adapter path for a single controller, pending exact
kit/revision and electrical validation. A default-off ESP-IDF UART adapter is
source implemented; it has not been compiled or verified on the actual motor.
The alternative is an SPI CAN-FD controller plus a CAN-FD transceiver. Neither
path has been validated on hardware.

## Source basis

- `Source/1-project_guide.pdf`: proposed CubeSat demonstrator, architecture, staging, targets, components, and report deliverables.
- `Source/0-Guiao.pdf`: course syllabus, learning objectives, assessment, and AI-use policy.
- `inputs/Dicas.txt`: working architecture notes, updated to reflect the user's confirmed onboard-computing decision.
- `inputs/sumury.txt`: detailed context summary and roadmap, updated to reflect the same decision; remaining specifics still require confirmation.
