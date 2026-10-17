# Ground Station

**Role:** provide operator status, history and approved supervisory commands. It receives telemetry, reports connection freshness and never computes the autonomous pointing command.

**Implementation:** [`../../Python_sim/Ground_Station/`](../../Python_sim/Ground_Station/) provides the browser UI, CSV log, telemetry proxy, PING, enumerated mode selection, bounded timed wheel-speed commissioning, and relative yaw-step requests. It displays speed target/feedback/status and a frame-matched grayscale camera image. Host SIL produces a labeled synthetic target frame; a missing ESP32 camera frame remains unavailable. Camera and IMU samples older than the controller's 100 ms freshness limit are labeled stale, and stale samples create gaps in the target-error history.

The CSV records camera focal length together with X/Y pixel offset and onboard bearing error, preserving the values needed to review the image-to-angle conversion.

**Limit:** the UI and bridge proxy have not been live-runtime tested in this environment because local socket creation is denied. The dashboard is coupled to the simulation bridge and is not a physical-radio client.

**Verify next:** run environment, SIL/firmware and Ground Station concurrently; check freshness, errors, image display, command acknowledgement and CSV output.
