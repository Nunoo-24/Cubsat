# Snapshot state — Test_drive_1

Use this note and `Context/HANDOFF.md` as the authoritative description of
this copy. It predates the later separate lost-target yaw gain and roll-gain
moderation. Roll is Kp=0.4/Kd=0.3 with a ±0.05 N·m virtual limit; yaw uses
Kd=0.04 in both tracking and IMU-only target-loss damping. Full Python
verification at capture: 190 tests passed. Toolchain and live runtime gates
are listed in the handoff.
