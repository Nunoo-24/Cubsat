#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

printf '%s\n' '== Python unit tests =='
python3 -m unittest discover -s . -p 'test_*.py'

printf '%s\n' '== Python syntax compilation =='
python3 -m compileall -q http_startup.py Sim_Environment Ground_Station CubeSat_ESP32 Verification

printf '%s\n' '== Regenerating simulation and camera reports =='
python3 -m Verification.evaluate_scenarios > ../Verification/Simulation_Results.md
python3 -m Verification.evaluate_camera_transport > ../Verification/Camera_Transport_Results.md
python3 -m Verification.evaluate_camera_geometry > ../Verification/Camera_Geometry_Results.md
python3 -m Verification.evaluate_camera_image_noise > ../Verification/Camera_Image_Noise_Results.md
python3 -m Verification.evaluate_camera_detector_timing > ../Verification/Camera_Detector_Timing_Results.md
python3 -m Verification.evaluate_axis_control > ../Verification/Axis_Control_Results.md

C_HOST_COMPILER=${CC:-cc}
if command -v "$C_HOST_COMPILER" >/dev/null 2>&1; then
    printf '%s\n' '== Portable C host tests =='
    ./CubeSat_ESP32/firmware/run_host_tests.sh
else
    printf '%s\n' 'SKIP: portable C host tests (no C11 compiler on PATH)'
fi

if command -v idf.py >/dev/null 2>&1; then
    printf '%s\n' '== ESP-IDF firmware build =='
    (cd CubeSat_ESP32/firmware && idf.py set-target esp32s3 && idf.py build)
else
    printf '%s\n' 'SKIP: ESP-IDF build (idf.py is not on PATH)'
fi

printf '%s\n' '== Offline verification complete =='
printf '%s\n' 'Live sockets, browser rendering, Wokwi execution, and hardware checks require their respective runtime environments.'
