# Synthetic Camera Target Geometry Sweep

Generated: 2026-10-16 23:44:47 UTC

This screen renders the environment's synthetic QVGA grayscale
frames, runs the onboard Python detector, and compares its validity with
the environment's analytic camera-reference flag. It is a renderer to
detector consistency check, not a physical-camera measurement.

## Setup

Target diameters: 0.05, 0.10, 0.15, 0.20 m; depths: 1.0, 2.0, 3.0, 4.0 m; horizontal bearings: -30°, +0°, +30°; vertical bearings: -20°, +0°, +20°.
The detector uses threshold 40 gray levels, minimum 50 and maximum 200
coherent pixels, and the 8-neighbour coherence filter. Its focal length
is the provisional simulator value 277.0 px. The size/
range grid contains 144 images; a separate five-pose grid checks the
detector against the analytic camera reference as the body attitude changes.

## Results

| Target diameter (m) | Depth (m) | Valid samples | Raw dark pixels | Coherent pixels | Max X error (px) | Max Y error (px) |
|---:|---:|---:|---:|---:|---:|---:|
| 0.05 | 1.0 | 9/9 | 74–148 | 74–148 | 2.83 | 0.18 |
| 0.05 | 2.0 | 0/9 | 16–32 | 16–32 | — | — |
| 0.05 | 3.0 | 0/9 | 8–16 | 8–16 | — | — |
| 0.05 | 4.0 | 0/9 | 6–12 | 6–12 | — | — |
| 0.10 | 1.0 | 0/9 | 304–608 | 304–608 | — | — |
| 0.10 | 2.0 | 9/9 | 74–148 | 74–148 | 2.83 | 0.18 |
| 0.10 | 3.0 | 3/9 | 34–68 | 34–68 | 0.00 | 0.18 |
| 0.10 | 4.0 | 0/9 | 16–32 | 16–32 | — | — |
| 0.15 | 1.0 | 0/9 | 668–1356 | 668–1356 | — | — |
| 0.15 | 2.0 | 6/9 | 166–332 | 166–332 | 4.31 | 0.18 |
| 0.15 | 3.0 | 9/9 | 74–148 | 74–148 | 2.83 | 0.18 |
| 0.15 | 4.0 | 3/9 | 44–88 | 44–88 | 0.00 | 0.18 |
| 0.20 | 1.0 | 0/9 | 1086–2416 | 1086–2416 | — | — |
| 0.20 | 2.0 | 0/9 | 304–608 | 304–608 | — | — |
| 0.20 | 3.0 | 6/9 | 134–268 | 134–268 | 3.86 | 0.18 |
| 0.20 | 4.0 | 9/9 | 74–148 | 74–148 | 2.83 | 0.18 |

Raw dark pixels count all threshold crossings; coherent pixels count
only pixels with at least two other dark 8-neighbours and determine the
area acceptance. A `—` means no tested bearing was accepted. The
simulator currently checks renderer/detector validity parity; the grid
shows the detector's size/range envelope under the current assumed
threshold and area settings. Horizontal pixel error is right-positive;
vertical detector error is down-positive and is sign-flipped when compared
with the environment's up-positive camera coordinate. Confirm target
dimensions, camera intrinsics,
lighting, and threshold using recorded hardware images before treating
this as an operational envelope.

## Rendered-frame detector check across body attitude

A fixed 0.1 m target at world position (0, 0, 2) m is rendered with
the following combined body poses. Errors compare the actual raster
detector with camera-reference metadata for the same observation.
Vertical detector error is sign-flipped from image-down to camera-up.

| Yaw (°) | Pitch (°) | Roll (°) | Valid | Coherent pixels | X discrepancy (px) | Y discrepancy (px) |
|---:|---:|---:|:---:|---:|---:|---:|
| +0 | +0 | +0 | yes | 148 | 0.00 | 0.00 |
| +10 | +0 | +0 | yes | 156 | 0.00 | 0.00 |
| -15 | +10 | +5 | yes | 164 | 0.00 | 0.00 |
| +5 | -8 | +10 | yes | 156 | 0.00 | 0.00 |
| -10 | -8 | -12 | yes | 156 | 0.00 | 0.00 |

This five-pose raster check supplements the 144-case target-geometry
grid. It verifies consistency at these sampled poses only; it does not
establish camera calibration, dynamics, or hardware performance.

## Reproduce

From `outputs/Python_sim/`, run:

```bash
python3 -m Verification.evaluate_camera_geometry > ../Verification/Camera_Geometry_Results.md
```
