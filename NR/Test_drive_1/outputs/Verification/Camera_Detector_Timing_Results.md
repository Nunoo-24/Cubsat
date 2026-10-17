# Host Python Camera Detector Timing

Generated: 2026-10-16 23:45:21 UTC

This measures only the Python QVGA pixel detector on this host. It
excludes frame transfer, HTTP, sensor reads, control calculations,
scheduling, and actuator communication. It is not ESP32 timing evidence.

## Setup

Frames are 320×240 grayscale; dark threshold 40; accepted coherent area 50–200 pixels.
Each case includes 5 warmups and 50 timed calls. Python 3.14.4 on `Linux-6.18.54.1-microsoft-standard-WSL2-x86_64-with-glibc2.43`.
The nominal SIL cycle is 20 ms (50 Hz); the table compares detector-only
time with that period as a rough budget reference, not a real-time pass.

## Results

| Frame case | Median (ms) | p95 (ms) | Maximum (ms) |
|---|---:|---:|---:|
| clean target | 2.002 | 3.769 | 3.952 |
| σ=40 target | 2.020 | 3.110 | 3.962 |
| σ=50 target | 3.641 | 12.822 | 14.498 |
| blank scene | 2.143 | 4.054 | 4.768 |
| all dark | 6.120 | 8.653 | 10.243 |
| checkerboard | 4.136 | 7.040 | 7.879 |
| half dark | 4.120 | 6.614 | 6.939 |

## Interpretation and limits

The bitset neighborhood implementation preserves the direct 8-neighbour
coherence rule and exact raw/coherent pixel counts. A deterministic
differential regression compares it with a direct-neighbour reference.
Dense synthetic scenes are not camera measurements, but they expose
input-dependent processing cost. Python host timing cannot predict C
firmware timing; compile and measure the detector on the ESP32-S3, then
measure the complete image-to-actuator cycle under the actual link load.

## Reproduce

From `outputs/Python_sim/`, run:

```bash
python3 -m Verification.evaluate_camera_detector_timing > ../Verification/Camera_Detector_Timing_Results.md
```
