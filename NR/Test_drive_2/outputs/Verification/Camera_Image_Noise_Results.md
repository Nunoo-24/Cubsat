# Synthetic Camera Image-Noise Evaluation

Generated: 2026-10-16 23:45:19 UTC

This study injects independent additive Gaussian noise into grayscale
pixels before the onboard detector. It replaces no hardware measurement:
noise levels are assumptions and do not model blur, exposure changes,
structured illumination, Bayer processing, or compression artifacts.

## Setup

The renderer supplies 320×240 grayscale images with background value 190 and target value 12. The detector threshold is 40, with accepted dark area 50–200 coherent pixels.
Target-present frames use a 0.10 m target at 2.0 m and bearings -30° to 30°; each noise level uses 30 deterministic noise seeds per bearing. A blank renderer-background image is used for false-positive trials.
Centroid error is the absolute difference from the clean raster's own
detector result, so it isolates added pixel noise from edge clipping.

## Results

| Gaussian σ (gray levels) | Target detections | Mean / p95 / max centroid change (px) | Blank-scene false detections | Blank dark pixels |
|---:|---:|---:|---:|---:|
| 0 | 210/210 | 0.000 / 0.000 / 0.000 | 0/30 | 0–0 |
| 10 | 210/210 | 0.005 / 0.033 / 0.068 | 0/30 | 0–0 |
| 20 | 210/210 | 0.057 / 0.142 / 0.237 | 0/30 | 0–0 |
| 30 | 210/210 | 0.102 / 0.264 / 0.457 | 0/30 | 0–1 |
| 40 | 209/210 | 0.137 / 0.346 / 0.769 | 0/30 | 3–12 |
| 50 | 191/210 | 0.164 / 0.384 / 1.153 | 0/30 | 87–125 |

The detector now keeps a dark pixel only when at least two other dark
pixels occur in its 8-neighbourhood. This rejects isolated random
threshold crossings without changing the clean renderer's target mask.
At σ=50, no blank-scene detections occur in 30 sampled trials, while 19 of 210 target trials are rejected across the 7 bearings.
These are observed counts for the selected deterministic seeds, not a
statistical guarantee. The local-coherence rule is not full
connected-component segmentation;
validate it using recorded camera images. Do not interpret these σ values
as OV camera specifications or this study as sensor qualification.

## Reproduce

From `outputs/Python_sim/`, run:

```bash
python3 -m Verification.evaluate_camera_image_noise > ../Verification/Camera_Image_Noise_Results.md
```
