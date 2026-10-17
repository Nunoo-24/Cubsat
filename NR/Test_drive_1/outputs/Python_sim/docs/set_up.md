# Setup and run guide

All host-side components use Python's standard library. The checked-in `.venv/`
is a Windows environment from the old OneDrive path and is not reusable in
WSL.

## Integrated host software-in-the-loop run

Open three WSL terminals in the Python_sim root. Start one process in each
terminal, in this order:

```bash
cd /home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_1/outputs/Python_sim
python3 -m Sim_Environment.main
```

```bash
cd /home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_1/outputs/Python_sim
python3 -m CubeSat_ESP32.sil
```

```bash
cd /home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_1/outputs/Python_sim
python3 -m Ground_Station.main
```

Open the simulation UI at `http://127.0.0.1:8000` and the Ground Station at
`http://127.0.0.1:8001`. The Ground Station CSV log is written to
`Ground_Station/logs/telemetry.csv`. The simulator applies XYZ disturbance
torques to pitch/yaw/roll respectively as X=pitch, Y=yaw, Z=roll. Roll/pitch
are simplified pendulum models; only yaw is actively controlled.

## Model, bridge, and interface checks

```bash
cd /home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_1/outputs/Python_sim
python3 -m unittest Sim_Environment.test_model Sim_Environment.test_environment CubeSat_ESP32.test_control CubeSat_ESP32.test_camera Ground_Station.test_main
```

## Legacy Level 0 example

The old UDP heartbeat demo is separate from the integrated HTTP bridge. Run
`python3 Ground_Station/simulator.py` in one terminal and
`python3 Ground_Station/level0_pyqt.py` in another. The GUI requires PyQt5 and
a working display such as WSLg.

## ESP32-S3 emulation

Install ESP-IDF and the Wokwi VS Code extension in a development environment.
Set `CUBESAT_WIFI_SSID`, `CUBESAT_WIFI_PASSWORD`, and
`CUBESAT_SIM_BRIDGE_URL` under the firmware's CubeSat simulation menu in
`idf.py menuconfig`. For Wokwi use the default `Wokwi-GUEST` and
`http://host.wokwi.internal:8000`, with Wokwi's private IoT gateway enabled.

```bash
cd /home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_1/outputs/Python_sim/CubeSat_ESP32/firmware
idf.py set-target esp32s3
idf.py build
```

Open this firmware directory in VS Code and start the Wokwi simulation. The
configuration selects the Seeed XIAO ESP32-S3. Wokwi build/runtime have not
been verified in this WSL environment because ESP-IDF and Wokwi tools are not
installed. The host SIL can be used independently while that toolchain is
unavailable.
