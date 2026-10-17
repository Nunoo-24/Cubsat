# AE software status review

The software project is `outputs/Python_sim/`. Its current architecture has
three roles: `Sim_Environment` models a gimbal with active yaw and pendular
roll/pitch and provides synthetic
sensors; `CubeSat_ESP32` runs the autonomous control reference (host SIL or
ESP-IDF firmware); `Ground_Station` monitors/logs telemetry and sends PING,
enumerated mode commands, bounded timed wheel-speed commissioning, and
relative yaw-step commands.

## Current implementation

- `Sim_Environment/main.py` serves the browser scene and version 1 HTTP/JSON
  bridge. It models target geometry, camera visibility/area, noisy synthetic
  gyro, ideal gravity, a fixed placeholder magnetic field, external torque,
  and a 100 ms actuator watchdog.
- Synthetic camera frames remain exact raw QVGA images in the environment.
  Host SIL and Wokwi use a sequence/dimension-tagged RLE1 transport capped at
  8,192 bytes, reconstruct all 76,800 pixels, then run their Python/C detector.
  The raw endpoint is retained for inspection. Python tests cover exact frame
  round trips, a 60-case scene sweep (largest payload 1,447 bytes), and an
  in-memory bridge → SIL decoder/detector → matched-preview integration path.
  This does not exercise TCP; Wokwi transfer and deadline measurements remain
  open. `outputs/Verification/Camera_Transport_Results.md` makes scene-size
  and local codec-timing measurements reproducible; its timings exclude the
  HTTP server and network path.
- `Sim_Environment/model.py` now includes a first-pass reaction-wheel model
  with torque limits, motor lag, wheel-speed saturation, and equal/opposite
  body torque. All wheel parameters are placeholders requiring real hardware
  measurements.
- `CubeSat_ESP32/sil.py` runs the Python reference loop at 50 Hz. The bounded
  yaw controller, target-loss behavior, supervisory mode handling, and
  portable camera-centroid reference are covered by Python tests. Both host
  SIL and target firmware now rebase after a missed 20 ms deadline instead of
  running catch-up cycles. ESP-IDF uses `xTaskDelayUntil()` and explicitly
  configures a 100 Hz tick; actual target jitter remains unmeasured.
  Python and C control references now fail closed with `INVALID_CONFIG` for
  non-finite or invalid gains/limits, including nonpositive torque, age, or
  rate bounds. Firmware fault serialization maps this code to the same
  `INVALID_CONFIG` string. The C addition is source-reviewed only because no
  host C compiler or working ESP-IDF build is available here.
- `CubeSat_ESP32/firmware/main/` contains ESP-IDF Wi-Fi/HTTP bridge code, a C
  yaw controller, portable dark-target centroid detector, optional XIAO Sense
  grayscale QVGA camera, and shared-I²C MPU-6050/LIS3MDL adapters. Wokwi
  uses synthetic observations; optional hardware sensor profiles override
  camera/IMU or magnetometer inputs. Physical sensor paths are source
  implemented but uncompiled and untested. Camera telemetry carries horizontal
  and vertical pixel offsets (positive right/down); yaw control uses horizontal
  error only. Physical camera frames use the
  driver's first-DMA-buffer timestamp since boot, rejecting zero/future times;
  camera/IMU ages are validated independently before
  the controller uses their maximum. The optional moteus UART adapter is
  also source implemented behind a default-off Kconfig gate; it requires
  physical camera/IMU options, an external arm input, and verified wiring.
- Hardware interface constraint confirmed from primary sources: ESP32-S3's
  TWAI is Classical CAN only, whereas moteus CAN uses FD frames. The UART
  adapter uses the documented r4 AUX2 path; the alternative is an external SPI
  CAN-FD controller plus transceiver. Neither physical path has been compiled
  or verified on hardware.
- Source review of the opt-in UART call path confirms the configured 1–20
  mN·m Kconfig bound is enforced by a per-request clamp and included in the
  outbound torque frame; SAFE/STANDBY/control-fault requests are zero torque.
  UART timing, electrical interface, drive watchdog and physical stop behavior
  remain unverified.
- ESP-IDF 6.1 source inspection found that UART, GPIO, and new I²C master APIs
  belong to `esp_driver_uart`, `esp_driver_gpio`, and `esp_driver_i2c`; the old
  umbrella `driver` dependency does not expose these headers in that release.
  CMake now declares the conditional direct dependencies, and the component
  manifest requires IDF 5.3+ (the first release with these split components).
  The updated build graph has not yet been compiled.
- `Ground_Station/main.py` serves the local dashboard, proxies telemetry and
  commands, and writes a CSV log. The UI labels camera/IMU samples older than
  the 100 ms control limit as stale and leaves history gaps unless both streams
  are fresh. The previous PyQt and UDP heartbeat examples remain in
  `level0_pyqt.py` and `simulator.py`.
- The environment browser scene now has a display-only orbit camera with
  pointer-drag rotation, wheel zoom and double-click reset. The Ground Station
  separately displays simulator truth from `/api/state` beside CubeSat sensor
  telemetry, explicitly labeling the difference. These browser changes have
  not yet been exercised in the user's browser.
- Wi-Fi reconnect now clears the firmware's connected flag on disconnect so
  exhausted retries re-enter the retry loop. The environment hides telemetry
  after 1.5 s without a new onboard packet, allowing the Ground Station to show
  a stale link as down instead of relabeling an old packet as live.
- `docs/Simulator_HTTP_API.md` and `docs/System_Architecture_and_Interfaces.md`
  describe the software bridge and model boundaries. HTTP is an emulator/SIL
  transport, not the selected physical radio protocol.
- The simulation bridge enforces increasing supervisory command IDs and
  idempotent identical retries while the original ten-second command window
  remains active. Expired, old, or conflicting IDs are rejected. IDs are
  restricted to exact JSON integer range 1..2^53−1 and checked before firmware
  conversion. Old or conflicting IDs are rejected; command IDs are in-memory
  and reset on bridge restart. The physical-link command persistence/replay
  contract is not yet selected.
- Acknowledgements are restricted to an issued command ID, recognized status,
  and a rejection reason. Embedded acknowledgement validation happens before
  actuator telemetry state is committed, avoiding partial acceptance.
- `POST /api/control` validates all supplied fields before applying any, so a
  malformed mixed scene-control request cannot change the target or external
  torque while returning HTTP 400. Unknown fields are rejected, and torque
  duration cannot be sent without a torque vector.

## Verification status

- The documented Python module suite and full discovery each pass 183 tests.
  `python3 -m compileall -q Sim_Environment Ground_Station CubeSat_ESP32
  Verification` passes. In-memory handler tests cover the environment,
  Ground Station, actuator/command exchange, yaw-step completion, and the
  RLE1 camera-frame → SIL detector → matched-preview path. These checks do not
  bind TCP sockets or execute browser JavaScript.
- `outputs/Verification/Simulation_Results.md` records five guide-based
  pointing cases, one-minute synthetic yaw-estimator error/drift, a 45° yaw
  step, five combined tilted-pose yaw checks, a camera projection matrix
  comparison at five combined poses, and a qualitative passive roll/pitch
  decay case. Current results include 1.600 s acquisition, 0.107°
  peak 60 s hold error, 1.340 s nudge recovery, 0.119° moving-target peak error,
  and 0.760 s recovery after 2 s camera occlusion. The 20 s roll/pitch run
  retains 13.978% of initial mechanical energy on each axis under provisional
  parameters. The ideal-vector yaw grid reports 0.000000° maximum error; it
  checks software coordinate consistency because the simulator and estimator
  share the same field assumption. The camera projection grid has zero
  rounded-pixel discrepancy against an independent rotation matrix; this
  validates software geometry only. None of these results validates physical
  dynamics, optics, or sensor accuracy.
- Firmware and SIL now reject bridge packets whose protocol version is not numeric value 1 (including booleans and fractional values); a source contract checks the C condition and Python tests exercise the input cases. C behavior remains source-reviewed only until ESP-IDF compilation/runtime.
- The Python detector uses row bitsets for exact local-coherence counts and centroids; a randomized differential regression checks it against the former direct-neighbour implementation. The host-only 50-call timing screen reports about 2 ms median on ordinary synthetic frames and 5.9 ms median / 10.6 ms maximum on all-dark input. See `outputs/Verification/Camera_Detector_Timing_Results.md`; ESP32 and complete-loop timing remain unmeasured.
- Firmware `app_main()` runs the synchronous 20 ms HTTP/control loop on ESP-IDF's main task. Defaults now allocate 8192 bytes (ESP-IDF v6.1 default is 3584) and a debug log samples the stack high-water mark every 250 cycles. This prevents relying on the small default without evidence, but target stack margin still needs Wokwi/board measurement.
- `outputs/Verification/Camera_Transport_Results.md` measures exact RLE1
  reconstruction across 60 synthetic scenes (10–1,447 bytes; default frame
  91 bytes) and host-only codec timings. HTTP, Wi-Fi, Wokwi gateway throughput,
  and ESP32 timing remain unmeasured.
- The placeholder wheel reaches 431 rpm after 0.5 s, below the proposed
  3000 rpm target; this is a model bound, not a motor measurement. A camera
  boundary case confirms that acquisition beyond the approximately 30° half
  field is unavailable.
- Python syntax compilation and the ESP-IDF Kconfig/profile checks have passed.
  The Kconfig checks validate options, not CMake configuration or C
  compilation. Host C tests, ESP-IDF build, Wokwi execution, physical sensor
  and actuator behavior, and HIL have not been verified.
- A direct Python loopback socket bind fails with
  `PermissionError: Operation not permitted`; the three separate processes
  and browser UI cannot be exercised in this environment. Linux `idf.py`,
  Wokwi CLI, and a host C compiler are also unavailable. The mounted Windows
  ESP-IDF 6.1 compiler cannot be launched through WSL because interop fails
  opening a socket (`UtilBindVsockAnyPort`). The old build cache predates the
  current firmware and is not build evidence.

## Main engineering gaps

1. Build the ESP-IDF project and run the Wokwi bridge with the private gateway.
2. Connect and test the camera detector against actual captured frames; verify
   board camera variant, memory, frame timing, and optical calibration.
3. Build and bench-validate the optional MPU-6050 interface, startup bias
   calibration, sample timing, axis mapping, and breakout electrical levels.
   Build and bench-validate the shared-bus LIS3MDL path and confirm its
   breakout voltage/address; calibrate field offsets before any fusion.
4. Build the default firmware and the opt-in UART profile. With motor power
   disconnected, verify the moteus framing, CRC, register responses, finite
   limits and watchdog. Replace provisional wheel values with measured
   inertia/speed/torque data and validate the direction/safe-stop procedure.
5. Confirm axis constraints and course performance targets against the project
   guide and the team's actual test rig. Current plant and control constants
   are provisional software settings.
