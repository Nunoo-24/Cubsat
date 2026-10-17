# AE CubeSat — Test Drive 2 (refreshed)

This is a self-contained snapshot of the active AE project, refreshed on
17 October 2026. It contains the current source, inputs, context, outputs,
verification records, and run instructions. It does not contain Git internals,
Python caches, build products, runtime logs, or machine-specific IDE settings.

## Start the host simulation

From the snapshot root, follow [`outputs/Python_sim/SECOND_TEST_DRIVE.md`](outputs/Python_sim/SECOND_TEST_DRIVE.md).
Start each component in its own terminal, in this order:

```bash
cd outputs/Python_sim
python3 -m Sim_Environment.main
```

```bash
cd outputs/Python_sim
python3 -m CubeSat_ESP32.sil
```

```bash
cd outputs/Python_sim
python3 -m Ground_Station.main
```

Open the environment at <http://127.0.0.1:8000> and Ground Station at
<http://127.0.0.1:8001>. The Windows launchers are under
`outputs/Python_sim/Launchers/`.

## Verification

Run the offline suite from `outputs/Python_sim/`:

```bash
./run_verification.sh
```

The source workspace's latest run passed 218 Python tests, Python syntax
compilation, and all six report generators. Portable C and ESP-IDF checks were
skipped there because the required compiler and `idf.py` were unavailable.
Rerun the command above to verify this packaged copy in your environment.
Live TCP/browser behavior, Wokwi, and physical hardware require separate
runtime checks.

## Project context

- [`Context/HANDOFF.md`](Context/HANDOFF.md) — current architecture, decisions,
  history, and continuation steps.
- [`outputs/reports/AE_Project_Current_State_Report_2026-10-17.md`](outputs/reports/AE_Project_Current_State_Report_2026-10-17.md)
  — concise current-state and verification summary.
- [`AE_PROJECT_README.md`](AE_PROJECT_README.md) — project-level workspace
  guide copied into this snapshot.
- [`outputs/Python_sim/README.md`](outputs/Python_sim/README.md) — component,
  test, firmware, and emulation commands.

This is a folder snapshot in the parent Git repository; it is not a separate
Git repository.

## System architecture diagram

Open the editable [diagrams.net file](outputs/Diagrams/AE_CubeSat_System_Architecture.drawio) or its [SVG preview](outputs/Diagrams/AE_CubeSat_System_Architecture.svg). It keeps those three top-level domains and divides the CubeSat into nested camera, sensor, processing, actuation, and telemetry domains. Processing uses smaller connected cards for estimation, guidance, and control/safety; the reaction-torque label is vertical for a cleaner route.
