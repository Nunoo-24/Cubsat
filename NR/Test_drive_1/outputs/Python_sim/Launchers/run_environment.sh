#!/usr/bin/env bash
PROJECT_DIR=/home/nunot/Mestrado_Codex/AE/outputs/Python_sim/NR/Test_drive_1/outputs/Python_sim
cd "$PROJECT_DIR" || exit 1

echo "Starting AE Simulation Environment from $PROJECT_DIR"
if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 is not installed in this WSL distribution."
    read -r -p "Press Enter to close this window..."
    exit 127
fi

python3 -u -m Sim_Environment.main
status=$?
if (( status != 0 )); then
    echo "ERROR: Simulation Environment exited with status $status."
    read -r -p "Press Enter to close this window..."
fi
exit "$status"
