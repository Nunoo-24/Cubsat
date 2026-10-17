@echo off
setlocal
set "WSL_DISTRO=Ubuntu"
set "LINUX_LAUNCHER=/home/nunot/Mestrado_Codex/AE/outputs/Python_sim/Launchers/run_cubesat_sil.sh"

"%SystemRoot%\System32\wsl.exe" --distribution "%WSL_DISTRO%" --exec python3 --version
if errorlevel 1 (
    echo WSL failed to start or the distribution "%WSL_DISTRO%" has no python3.
    echo Check the visible error above. You can verify the distribution with: wsl.exe -l -q
    pause
    exit /b 1
)
start "AE CubeSat SIL" cmd.exe /k %SystemRoot%\System32\wsl.exe --distribution %WSL_DISTRO% --exec bash %LINUX_LAUNCHER%
