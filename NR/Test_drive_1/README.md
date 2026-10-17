# AE CubeSat — Test drive 1

First full-direction test-drive state: same yaw rate gain during tracking and target loss (Kd=0.04); roll gains Kp=0.4/Kd=0.3, cap ±0.05 N·m; full verification 190 Python tests passed.

## Start a test session

Read [`Context/HANDOFF.md`](Context/HANDOFF.md),
[`inputs/Prompts.txt`](inputs/Prompts.txt), and
[`outputs/Python_sim/README.md`](outputs/Python_sim/README.md). The full
project overview from the source workspace is preserved in
[`AE_PROJECT_README.md`](AE_PROJECT_README.md).

Follow [`outputs/Python_sim/FIRST_TEST_DRIVE.md`](outputs/Python_sim/FIRST_TEST_DRIVE.md) for the
test procedure. Start one terminal per component from this snapshot:

```bash
cd outputs/Python_sim
python3 -u -m Sim_Environment.main
```

```bash
cd outputs/Python_sim
python3 -u -m CubeSat_ESP32.sil
```

```bash
cd outputs/Python_sim
python3 -u -m Ground_Station.main
```

Open the environment at <http://127.0.0.1:8000> and the Ground Station at
<http://127.0.0.1:8001>. Logs and generated runtime files stay inside this
snapshot. The snapshots are ordinary folders within the parent Git repo; they
do not contain a separate `.git` directory.
