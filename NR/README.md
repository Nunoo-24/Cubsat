# NR — test-drive snapshots

Two self-contained, runnable snapshots of the AE CubeSat project are stored
here. Each includes its own source tree, project inputs, design/output files,
README, and context handoff. Run the commands from that snapshot's
`outputs/Python_sim` directory. The snapshots do not share runtime logs.

- [`Test_drive_1/`](Test_drive_1/README.md) — first full-direction test state
  (190 Python tests; original stronger roll tuning; baseline yaw-rate gain).
- [`Test_drive_2/`](Test_drive_2/README.md) — current state (191 Python tests;
  separate stronger IMU yaw damping after target loss; moderated roll tuning).

These are source snapshots inside the existing Git repository, not separate
Git repositories or GitHub branches. The live workspace's existing Git history
and uncommitted changes remain outside these folders. Each copy omits `.git`,
Python caches, generated build directories, and transient Ground Station logs.
