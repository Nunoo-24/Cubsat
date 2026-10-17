# Telemetry and telecommand

**Role:** report onboard sensor, image-error, mode, actuator and health data; accept restricted supervisory commands without closing the control loop from the Ground Station.

**Implemented local protocol:** versioned JSON over HTTP for host SIL/Wokwi, including observations, actuator telemetry, PING, enumerated SET_MODE, bounded SET_WHEEL_SPEED commissioning requests, and relative
SET_YAW_STEP commands with target/error/progress telemetry. Speed telemetry includes target, measured rev/s, error, elapsed duration, and a terminal stop status retained until another test starts. Optional camera preview is a separate 64x48 raw grayscale local HTTP transfer paired with X/Y centroid error headers for overlay alignment.

**Implementation:** [HTTP API](../../Python_sim/docs/Simulator_HTTP_API.md) and [TM/TC ICD](../../Python_sim/docs/TMTC-ICD.md).

**Limit:** HTTP is only the emulator/SIL bridge; physical radio transport, security/authentication, packet integrity, retries, link loss and message rates are TBD. Speed feedback from the plant is simulated and does not validate a physical speed loop. Preview is not an image-radio protocol.

**Verify next:** live local process exchange; later select a radio link and define its framing, rate, CRC, authorization and stale-link procedures.
