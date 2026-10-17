@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Create_Desktop_Shortcuts.ps1"
if errorlevel 1 (
    echo Shortcut creation failed. You can still double-click the three Start_*.bat files.
    pause
    exit /b 1
)
pause
