# AE CubeSat simulation — first test drive

This describes the original first-version software-in-the-loop package. It uses three
separate processes: the simulated plant and camera, the CubeSat host SIL, and
the Ground Station. The host SIL is a Python reference for the intended
onboard flow; it is not ESP32 firmware.

## Start from Windows

The `Launchers/` folder contains three click-to-run Windows batch launchers.
Double-click them in this order:

1. `Start_01_Simulation_Environment.bat` — starts the simulated plant and opens
   <http://127.0.0.1:8000>.
2. `Start_02_CubeSat_SIL.bat` — starts the onboard Python software loop.
3. `Start_03_Ground_Station.bat` — starts the operator station and opens
   <http://127.0.0.1:8001>.

In the environment scene, drag to orbit around the CubeSat, use the mouse wheel
to zoom, and double-click to reset the view. The Ground Station now shows a
separate simulator-truth readout beside onboard telemetry. The simulator truth
is a debugging reference; camera measurements and CubeSat estimates can differ
because they use sensor visibility, detection and estimation.

For three desktop icons, double-click `Launchers/Install_Desktop_Shortcuts.bat`
once. It creates shortcuts for those launchers on the current Windows desktop.
Each component opens in its own persistent Command Prompt window running WSL.
The window remains open at a prompt if startup fails or after you stop the
service with `Ctrl+C`; close it manually after reading any error. Start the
environment before the SIL and Ground Station; they depend on its HTTP bridge.

The launchers assume the WSL distribution is named `Ubuntu` and the project is
at `/home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_1/outputs/Python_sim`. If `wsl.exe -l -q` shows
a different distribution name, edit `WSL_DISTRO` in each `.bat` file. Windows
localhost forwarding must be enabled by WSL for the browser links to work.
Each launcher first checks that WSL can start `python3`; the component terminal
then stays open and displays startup errors so they can be copied for diagnosis.

These are click-to-run scripts rather than standalone `.exe` applications.
They start the project’s existing Python modules inside WSL, which keeps the
source tree and the runtime in the same Linux environment. This workspace does
not currently have a working Windows compiler/packager path, so a native `.exe`
has not been produced.

## Start from a WSL terminal

Open three terminals in this directory:

```bash
python3 -u -m Sim_Environment.main
```

```bash
python3 -u -m CubeSat_ESP32.sil
```

```bash
python3 -u -m Ground_Station.main
```

The simulation scene is on port 8000; the Ground Station is on port 8001. The
Ground Station writes runtime telemetry to `Ground_Station/logs/telemetry.csv`.
If its schema changed, the previous log is kept beside it as
`telemetry.legacy-YYYYMMDD-HHMMSS.csv` and the new run starts with the current
header.

## If a page says it cannot connect

The environment prints `Simulation environment: http://127.0.0.1:8000` after
its listener starts. The Ground Station prints `Ground Station: http://127.0.0.1:8001`.
Keep both component windows open. If either line is missing, read the error in
that component's terminal; the service did not start.

To bypass the Windows shortcuts, open Ubuntu from the Start menu and run the
three WSL commands above in separate terminals, in order. Check the Windows WSL
distribution name with `wsl.exe -l -q` from PowerShell. If the service prints
its listening URL but the browser still cannot connect, run this in PowerShell:

```powershell
Test-NetConnection 127.0.0.1 -Port 8000
Test-NetConnection 127.0.0.1 -Port 8001
```

Send the terminal error text and each `TcpTestSucceeded` result. Both pages
failing means we first need to determine whether the processes bound their
ports or whether Windows-to-WSL localhost forwarding failed. The Ground Station
page can load without the environment, though its live data will be unavailable.

For a single diagnostic file, leave the component terminals open and
double-click `Launchers/Diagnose_Connections.bat`. It checks WSL-local TCP and
HTTP access, WSL listener state, Windows listener state, and Windows localhost
connectivity. It saves the complete output as `AE_Connection_Diagnostic.txt`
on the Windows desktop; send that file's contents.

## First test-drive sequence

1. Start all three processes and confirm both pages load.
2. Confirm the cube and black target appear in the environment scene.
3. Move the target to `X=0, Y=0.5, Z=2 m` and apply. Confirm the cube pitches
   until the target is vertically centered. In Ground Station, verify camera
   target is valid, `camera_error_y_px` approaches zero, pitch changes, the
   requested virtual pitch torque appears, and simulated truth Y error falls.
   The frame is drawn at the cube center: X (red) is pitch, Y (green) is yaw,
   and Z (blue) is roll/camera boresight. Gravity levels roll; a single point
   target cannot determine roll about the camera boresight.
4. Move the target horizontally, then confirm yaw tracks it as before. Apply a
   short bounded XYZ disturbance only after the no-disturbance pointing checks.
5. Apply a Z-axis torque of `0.05 N·m` for `0.5 s`. Confirm roll remains below
   about 5° and returns to within 1° with rate below 0.5°/s in under 2 s after
   the pulse ends. Ground Station simulator truth shows roll attitude; the
   simulated applied torque readout shows roll recovery commands.
6. Check the telemetry display and CSV log, then stop all three processes.

Keep the first run to simulated commands. The physical moteus actuator is not
part of this test drive, and no physical motor should be connected based on
this simulation alone.

## Known limits of this first version

- The integrated Python model and in-memory suite now passes 190 tests and
  Python syntax compilation. This does not verify the live three-process path
  or browser rendering.
- ESP-IDF firmware compilation, Wokwi execution, and physical hardware tests
  remain unverified.
- Model values and camera calibration are provisional. Yaw uses the modeled
  reaction wheel. Host SIL pitch/roll commands are bounded virtual torques;
  no physical pitch/roll actuators are specified or controlled.
- The roll recovery gain and ±0.05 N·m virtual roll authority are simulation
  tuning choices. The Z-disturbance recovery regression is not hardware timing
  or physical stabilisation evidence.
- The current Codex WSL session cannot create loopback sockets and cannot invoke
  Windows executables, so it could not run these launchers or perform the live
  first test drive. Run them on the user's normal Windows/WSL desktop session.
- Latest offline verification completed: 190/190 Python tests passed, Python
  syntax compilation passed, and all five model/camera reports regenerated.
  Portable C and ESP-IDF checks were skipped because the required toolchains
  are unavailable. The Markdown link audit from the previous checkpoint found
  no broken relative links across 33 files.
- For offline checks and regenerated results, see `./run_verification.sh` and
  the project status report at `../reports/AE_Project_Status_and_Run_Guide.md`.
