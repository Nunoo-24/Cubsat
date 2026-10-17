@echo off
setlocal
set "WSL_DISTRO=Ubuntu"
set "LINUX_LAUNCHER=/home/nunot/Mestrado_Codex/AE/outputs/Python_sim/Launchers/run_environment.sh"

"%SystemRoot%\System32\wsl.exe" --distribution "%WSL_DISTRO%" --exec python3 --version
if errorlevel 1 (
    echo WSL failed to start or the distribution "%WSL_DISTRO%" has no python3.
    echo Check the visible error above. You can verify the distribution with: wsl.exe -l -q
    pause
    exit /b 1
)
start "AE Simulation Environment" cmd.exe /k %SystemRoot%\System32\wsl.exe --distribution %WSL_DISTRO% --exec bash %LINUX_LAUNCHER%
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Wait_For_Service.ps1" -Port 8000 -Url "http://127.0.0.1:8000/" -ExpectedText "CubeSat attitude-control simulation"
if errorlevel 1 (
    echo Environment did not become ready. Its terminal remains open for diagnosis.
    pause
    exit /b 1
)
start "" "http://127.0.0.1:8000"
