# Ground_Station

Run the local browser dashboard from the Python_sim project root with
`python3 -m Ground_Station.main`, then open `http://127.0.0.1:8001`.

It monitors onboard telemetry from the simulation bridge, displays target
error, XYZ gyro and accelerometer readings, optional magnetometer field and
frame, sensor freshness/health, mode and fault state, plots target-error history
against simulation time (with visible gaps when the target is unavailable or a
camera/IMU sample exceeds the controller's 100 ms freshness limit),
sends `PING`, enumerated mode commands, bounded timed wheel-speed
commissioning requests, and relative yaw-step commands, then appends received
telemetry to `Ground_Station/logs/telemetry.csv`. It never computes pointing
control or sends reaction-wheel torque commands.

When the telemetry field schema changes, the logger moves the previous CSV to
`telemetry.legacy-YYYYMMDD-HHMMSS.csv` before creating a fresh file with the
current header. This preserves earlier rows and keeps new columns aligned.

The CSV preserves the reported camera focal length alongside pixel offsets
and the onboard bearing error so synthetic and later calibrated results can
be audited from the same telemetry record.

The local simulation bridge also exposes wheel speed and simulated actuator
outputs as distinct fields; these are model data, not hardware telemetry.
The dashboard includes a separately labeled simulation-truth readout for plant
yaw/roll/pitch and target geometry/error. It is fetched from the environment
state endpoint for debugging and is not CubeSat telemetry. Differences from
onboard camera error or estimated yaw are expected when camera visibility,
target detection, attitude, or estimation affect the measurement.

The dashboard displays a 64x48 grayscale frame from `/api/camera-preview.bmp`.
Fresh Sense-camera thumbnails are labeled physical; host-SIL or default-Wokwi
telemetry gets a 5x5-averaged thumbnail of the exact QVGA frame processed by
its detector, labeled simulated, when no physical frame is fresh.
Only the host-SIL source is synthesized; a missing ESP32 camera frame remains
unavailable. Frames expire after two seconds. Each frame includes a centre
crosshair and its paired X/Y target offset, so a newer measurement is not
overlaid on an older image.

The previous PyQt Level 0 demo is retained as `level0_pyqt.py`; it requires
PyQt5 and pairs only with the legacy UDP heartbeat simulator. The integrated
dashboard and host SIL use Python's standard library.
