"""Measure synthetic pixel-domain noise effects on the onboard target detector."""

from datetime import datetime, timezone
import math
import random

from CubeSat_ESP32.camera import detect_black_target
from Sim_Environment.main import (CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX,
                                  Environment)


FRAME_PIXELS = CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX
DEFAULT_NOISE_SIGMA_GRAY = (0.0, 10.0, 20.0, 30.0, 40.0, 50.0)
DEFAULT_BEARINGS_DEG = (-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0)
DEFAULT_SEED_COUNT = 30
TARGET_DEPTH_M = 2.0
TARGET_DIAMETER_M = 0.1
BACKGROUND_GRAY = 190
TARGET_GRAY = 12


def add_gaussian_intensity_noise(frame, sigma_gray, seed):
    """Return a deterministic 8-bit frame with additive pixel noise applied."""
    if not isinstance(frame, (bytes, bytearray)) or len(frame) != FRAME_PIXELS:
        raise ValueError("frame must be one raw QVGA grayscale image")
    if (isinstance(sigma_gray, bool) or not isinstance(sigma_gray, (int, float))
            or not math.isfinite(sigma_gray) or sigma_gray < 0.0):
        raise ValueError("sigma_gray must be a finite nonnegative number")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError("seed must be an integer")
    if sigma_gray == 0.0:
        return bytes(frame)
    generator = random.Random(seed)
    return bytes(max(0, min(255, round(value + generator.gauss(0.0, sigma_gray))))
                 for value in frame)


def _target_frame(bearing_deg):
    environment = Environment()
    x_m = TARGET_DEPTH_M * math.tan(math.radians(bearing_deg))
    environment.apply({
        "target_xyz_m": {"x": x_m, "y": 0.0, "z": TARGET_DEPTH_M},
        "target_scale": TARGET_DIAMETER_M,
    })
    observation = environment.observation()
    frame = environment.camera_frame_snapshot(observation["sequence"])
    return frame


def _percentile(values, fraction):
    values = sorted(values)
    return values[math.ceil(len(values) * fraction) - 1]


def evaluate_camera_image_noise(noise_levels=DEFAULT_NOISE_SIGMA_GRAY,
                                bearings_deg=DEFAULT_BEARINGS_DEG,
                                seed_count=DEFAULT_SEED_COUNT):
    """Evaluate target detections and false positives on noisy QVGA images.

    Noise is independent additive Gaussian intensity noise in grayscale code
    values, clipped to [0, 255]. The levels are sensitivity assumptions, not
    measured camera noise characteristics.
    """
    if isinstance(seed_count, bool) or not isinstance(seed_count, int) or seed_count < 1:
        raise ValueError("seed_count must be a positive integer")
    noise_levels = tuple(noise_levels)
    bearings_deg = tuple(bearings_deg)
    if not noise_levels or not bearings_deg:
        raise ValueError("at least one noise level and bearing are required")
    for sigma in noise_levels:
        if (isinstance(sigma, bool) or not isinstance(sigma, (int, float)) or
                not math.isfinite(sigma) or sigma < 0.0):
            raise ValueError("noise levels must be finite nonnegative numbers")
    for bearing in bearings_deg:
        if (isinstance(bearing, bool) or not isinstance(bearing, (int, float)) or
                not math.isfinite(bearing) or not -30.0 <= bearing <= 30.0):
            raise ValueError("bearings must be finite and within ±30 degrees")

    target_frames = []
    for bearing in bearings_deg:
        frame = _target_frame(float(bearing))
        baseline = detect_black_target(frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
        if not baseline["valid"]:
            raise RuntimeError(f"clean reference target is invalid at {bearing}°")
        target_frames.append((float(bearing), frame, baseline["error_x_px"]))

    no_target_frame = bytes([BACKGROUND_GRAY]) * FRAME_PIXELS
    results = []
    for level_index, sigma in enumerate(noise_levels):
        detections = 0
        target_trials = 0
        centroid_error_px = []
        for bearing_index, (bearing, frame, baseline_error) in enumerate(target_frames):
            for seed_index in range(seed_count):
                seed = level_index * 1_000_000 + bearing_index * 1_000 + seed_index
                noisy = add_gaussian_intensity_noise(frame, sigma, seed)
                detection = detect_black_target(noisy, CAMERA_WIDTH_PX,
                                                CAMERA_HEIGHT_PX)
                target_trials += 1
                if detection["valid"]:
                    detections += 1
                    centroid_error_px.append(
                        abs(detection["error_x_px"] - baseline_error)
                    )

        false_positives = 0
        no_target_trials = seed_count
        no_target_dark_counts = []
        for seed_index in range(seed_count):
            seed = level_index * 1_000_000 + 900_000 + seed_index
            noisy = add_gaussian_intensity_noise(no_target_frame, sigma, seed)
            detection = detect_black_target(noisy, CAMERA_WIDTH_PX,
                                            CAMERA_HEIGHT_PX)
            no_target_dark_counts.append(detection["dark_pixel_count"])
            false_positives += int(detection["valid"])

        results.append({
            "sigma_gray": float(sigma),
            "target_trials": target_trials,
            "target_detections": detections,
            "target_detection_rate": detections / target_trials,
            "centroid_error_mean_px": (
                sum(centroid_error_px) / len(centroid_error_px)
                if centroid_error_px else None
            ),
            "centroid_error_p95_px": (
                _percentile(centroid_error_px, 0.95)
                if centroid_error_px else None
            ),
            "centroid_error_max_px": max(centroid_error_px, default=None),
            "no_target_trials": no_target_trials,
            "no_target_false_positives": false_positives,
            "no_target_dark_count_min": min(no_target_dark_counts),
            "no_target_dark_count_max": max(no_target_dark_counts),
        })

    return {
        "generated_utc": datetime.now(timezone.utc).strftime(
            "%Y-%m-%d %H:%M:%S UTC"),
        "frame_width_px": CAMERA_WIDTH_PX,
        "frame_height_px": CAMERA_HEIGHT_PX,
        "target_depth_m": TARGET_DEPTH_M,
        "target_diameter_m": TARGET_DIAMETER_M,
        "background_gray": BACKGROUND_GRAY,
        "target_gray": TARGET_GRAY,
        "threshold": 40,
        "min_dark_pixels": 50,
        "max_dark_pixels": 200,
        "bearings_deg": list(bearings_deg),
        "seed_count": seed_count,
        "results": results,
    }


def render_report():
    result = evaluate_camera_image_noise()
    highest_noise = result["results"][-1]
    lines = [
        "# Synthetic Camera Image-Noise Evaluation",
        "",
        f"Generated: {result['generated_utc']}",
        "",
        "This study injects independent additive Gaussian noise into grayscale",
        "pixels before the onboard detector. It replaces no hardware measurement:",
        "noise levels are assumptions and do not model blur, exposure changes,",
        "structured illumination, Bayer processing, or compression artifacts.",
        "",
        "## Setup",
        "",
        (f"The renderer supplies {result['frame_width_px']}×"
         f"{result['frame_height_px']} grayscale images with background value "
         f"{result['background_gray']} and target value {result['target_gray']}. "
         f"The detector threshold is {result['threshold']}, with accepted dark "
         f"area {result['min_dark_pixels']}–{result['max_dark_pixels']} coherent pixels."),
        (f"Target-present frames use a {result['target_diameter_m']:.2f} m target "
         f"at {result['target_depth_m']:.1f} m and bearings "
         f"{min(result['bearings_deg']):.0f}° to "
         f"{max(result['bearings_deg']):.0f}°; each noise level uses "
         f"{result['seed_count']} deterministic noise seeds per bearing. A blank "
         "renderer-background image is used for false-positive trials."),
        "Centroid error is the absolute difference from the clean raster's own",
        "detector result, so it isolates added pixel noise from edge clipping.",
        "",
        "## Results",
        "",
        "| Gaussian σ (gray levels) | Target detections | Mean / p95 / max centroid change (px) | Blank-scene false detections | Blank dark pixels |",
        "|---:|---:|---:|---:|---:|",
    ]
    for row in result["results"]:
        if row["centroid_error_mean_px"] is None:
            error_text = "—"
        else:
            error_text = (
                f"{row['centroid_error_mean_px']:.3f} / "
                f"{row['centroid_error_p95_px']:.3f} / "
                f"{row['centroid_error_max_px']:.3f}"
            )
        lines.append(
            f"| {row['sigma_gray']:.0f} | "
            f"{row['target_detections']}/{row['target_trials']} | {error_text} | "
            f"{row['no_target_false_positives']}/{row['no_target_trials']} | "
            f"{row['no_target_dark_count_min']}–{row['no_target_dark_count_max']} |"
        )
    lines.extend([
        "",
        "The detector now keeps a dark pixel only when at least two other dark",
        "pixels occur in its 8-neighbourhood. This rejects isolated random",
        "threshold crossings without changing the clean renderer's target mask.",
        (f"At σ={highest_noise['sigma_gray']:.0f}, no blank-scene detections occur "
         f"in {highest_noise['no_target_trials']} sampled trials, while "
         f"{highest_noise['target_trials'] - highest_noise['target_detections']} "
         f"of {highest_noise['target_trials']} target trials are rejected across "
         f"the {len(result['bearings_deg'])} bearings."),
        "These are observed counts for the selected deterministic seeds, not a",
        "statistical guarantee. The local-coherence rule is not full",
        "connected-component segmentation;",
        "validate it using recorded camera images. Do not interpret these σ values",
        "as OV camera specifications or this study as sensor qualification.",
        "",
        "## Reproduce",
        "",
        "From `outputs/Python_sim/`, run:",
        "",
        "```bash",
        "python3 -m Verification.evaluate_camera_image_noise > ../Verification/Camera_Image_Noise_Results.md",
        "```",
    ])
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    print(render_report(), end="")
