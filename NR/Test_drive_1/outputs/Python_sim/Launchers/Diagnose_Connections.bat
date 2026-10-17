@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0Diagnose_Connections.ps1"
if errorlevel 1 echo The diagnostic encountered an error; read the output above.
echo Keep your three service terminals open while running this diagnostic.
pause
