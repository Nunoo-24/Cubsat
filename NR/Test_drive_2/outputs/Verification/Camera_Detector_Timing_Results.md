# Host Python Camera Detector Timing

Generated: 2026-10-17 11:07:05 UTC

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
| clean target | 2.039 | 2.604 | 3.412 |
| σ=40 target | 2.063 | 2.236 | 2.634 |
| σ=50 target | 2.076 | 2.450 | 3.561 |
| blank scene | 1.923 | 2.129 | 2.204 |
| all dark | 5.953 | 6.672 | 10.102 |
| checkerboard | 4.252 | 6.646 | 6.900 |
| half dark | 3.928 | 4.276 | 4.527 |

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
