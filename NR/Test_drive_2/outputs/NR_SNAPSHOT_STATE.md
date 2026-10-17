# Snapshot state — Test_drive_2

Use this note and `Context/HANDOFF.md` as the authoritative description of
this copy. Yaw tracking uses the Y-axis gyro; target loss switches to
IMU-only damping at 0.06 N·m·s/rad. Roll uses Kp=0.24/Kd=0.19 with a
±0.035 N·m virtual limit. Pitch/roll remain simulation-only. Full Python
verification at capture: 191 tests passed; portable C/ESP-IDF checks skipped
without toolchains. The second test-drive sequence is in
`outputs/Python_sim/SECOND_TEST_DRIVE.md`.
