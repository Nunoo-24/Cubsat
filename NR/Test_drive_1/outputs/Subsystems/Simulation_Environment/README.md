# Simulation environment

**Role:** provide a repeatable target scene, three-axis simplified gimbal dynamics, synthetic camera/IMU observations, disturbance injection, and the local bridge used by host SIL and Wokwi firmware.

**Inputs:** target XYZ and scale, external torque XYZ and duration, camera occlusion interval, onboard actuator telemetry. Torque and occlusion durations follow simulation time; a partial-step torque pulse is integrated proportionally.

**Outputs:** browser scene/state, versioned sensor/command cycle, sequence-
paired raw QVGA grayscale camera frames, telemetry freshness, and idealized
applied body torque to the three-axis plant (yaw reaction wheel plus bounded
host-SIL virtual pitch/roll torques).

**Implementation:** [`../../Python_sim/Sim_Environment/`](../../Python_sim/Sim_Environment/) and the [HTTP API](../../Python_sim/docs/Simulator_HTTP_API.md).

**Limit:** yaw uses the reaction-wheel model; host SIL may apply bounded virtual pitch/roll torques. Body frame maps X=pitch, Y=yaw, Z=roll. Euler-rate coupling, realistic optical effects/noise, and sensor fusion are not modeled. The QVGA frame uses pinhole projection with a camera-facing circular marker whose apparent size uses optical-axis depth; it is not an oriented-poster or photorealistic camera model. Wheel and virtual-axis parameters are provisional; real motor/controller behavior is omitted.

**Model check:** the deterministic verification report includes a 20 s, no-external-torque roll/pitch free-decay case. With the current provisional starting angles and parameters, each axis retains about 14.0% of its initial mechanical energy. This checks qualitative damping and axis mapping only; it is not a measured settling target for the rig.

The report also compares camera X/Y projection against an independent rigid-body rotation matrix at five tilted poses; exact rounded pixels agree. This verifies the software geometry only, not the lens, camera mount, or target orientation.

**Verify next:** live three-process HTTP/SIL exchange and browser rendering, camera geometry against the actual lens/mount, and plant constants against measured hardware.
