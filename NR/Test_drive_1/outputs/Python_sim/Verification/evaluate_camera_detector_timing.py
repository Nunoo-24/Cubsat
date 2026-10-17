"""Benchmark the host Python QVGA detector on synthetic and dense frames."""

from datetime import datetime, timezone
import math
import platform
import random
import statistics
import sys
import time

from CubeSat_ESP32.camera import detect_black_target
from Sim_Environment.main import (CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX,
                                  Environment)


DEFAULT_ITERATIONS = 50
CONTROL_PERIOD_MS = 20.0
BACKGROUND_GRAY = 190
TARGET_DIAMETER_M = 0.1
TARGET_DEPTH_M = 2.0


def _render_target():
    environment = Environment()
    environment.apply({"target_xyz_m": {"x": 0.35, "y": 0.0, "z": 2.0},
                       "target_scale": TARGET_DIAMETER_M})
    observation = environment.observation()
    return environment.camera_frame_snapshot(observation["sequence"])


def _noise(frame, sigma, seed):
    generator = random.Random(seed)
    return bytes(max(0, min(255, round(value + generator.gauss(0.0, sigma))))
                 for value in frame)


def benchmark_camera_detector(iterations=DEFAULT_ITERATIONS):
    if (isinstance(iterations, bool) or not isinstance(iterations, int) or
            iterations < 2):
        raise ValueError("iterations must be an integer of at least two")
    width, height = CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX
    clean_target = _render_target()
    cases = {
        "clean target": clean_target,
        "σ=40 target": _noise(clean_target, 40.0, seed=740),
        "σ=50 target": _noise(clean_target, 50.0, seed=750),
        "blank scene": bytes([BACKGROUND_GRAY]) * (width * height),
        "all dark": bytes([0]) * (width * height),
        "checkerboard": bytes(
            0 if (x + y) % 2 else BACKGROUND_GRAY
            for y in range(height) for x in range(width)
        ),
        "half dark": bytes(
            0 if x < width // 2 else BACKGROUND_GRAY
            for _y in range(height) for x in range(width)
        ),
    }
    results = []
    for name, frame in cases.items():
        for _ in range(5):
            detect_black_target(frame, width, height)
        samples_ms = []
        for _ in range(iterations):
            start = time.perf_counter()
            detect_black_target(frame, width, height)
            samples_ms.append((time.perf_counter() - start) * 1000.0)
        samples_ms.sort()
        p95_index = math.ceil(0.95 * len(samples_ms)) - 1
        results.append({
            "case": name,
            "median_ms": statistics.median(samples_ms),
            "p95_ms": samples_ms[p95_index],
            "max_ms": samples_ms[-1],
            "iterations": iterations,
        })
    return {
        "generated_utc": datetime.now(timezone.utc).strftime(
            "%Y-%m-%d %H:%M:%S UTC"),
        "python_version": sys.version.split()[0],
        "platform": platform.platform(),
        "width_px": width,
        "height_px": height,
        "threshold": 40,
        "minimum_dark_pixels": 50,
        "maximum_dark_pixels": 200,
        "target_diameter_m": TARGET_DIAMETER_M,
        "target_depth_m": TARGET_DEPTH_M,
        "warmup_calls_per_case": 5,
        "control_period_ms": CONTROL_PERIOD_MS,
        "results": results,
    }


def render_report():
    result = benchmark_camera_detector()
    lines = [
        "# Host Python Camera Detector Timing",
        "",
        f"Generated: {result['generated_utc']}",
        "",
        "This measures only the Python QVGA pixel detector on this host. It",
        "excludes frame transfer, HTTP, sensor reads, control calculations,",
        "scheduling, and actuator communication. It is not ESP32 timing evidence.",
        "",
        "## Setup",
        "",
        (f"Frames are {result['width_px']}×{result['height_px']} grayscale; "
         f"dark threshold {result['threshold']}; accepted coherent area "
         f"{result['minimum_dark_pixels']}–{result['maximum_dark_pixels']} pixels."),
        (f"Each case includes {result['warmup_calls_per_case']} warmups and "
         f"{result['results'][0]['iterations']} timed calls. Python "
         f"{result['python_version']} on `{result['platform']}`."),
        "The nominal SIL cycle is 20 ms (50 Hz); the table compares detector-only",
        "time with that period as a rough budget reference, not a real-time pass.",
        "",
        "## Results",
        "",
        "| Frame case | Median (ms) | p95 (ms) | Maximum (ms) |",
        "|---|---:|---:|---:|",
    ]
    for row in result["results"]:
        lines.append(
            f"| {row['case']} | {row['median_ms']:.3f} | "
            f"{row['p95_ms']:.3f} | {row['max_ms']:.3f} |"
        )
    lines.extend([
        "",
        "## Interpretation and limits",
        "",
        "The bitset neighborhood implementation preserves the direct 8-neighbour",
        "coherence rule and exact raw/coherent pixel counts. A deterministic",
        "differential regression compares it with a direct-neighbour reference.",
        "Dense synthetic scenes are not camera measurements, but they expose",
        "input-dependent processing cost. Python host timing cannot predict C",
        "firmware timing; compile and measure the detector on the ESP32-S3, then",
        "measure the complete image-to-actuator cycle under the actual link load.",
        "",
        "## Reproduce",
        "",
        "From `outputs/Python_sim/`, run:",
        "",
        "```bash",
        "python3 -m Verification.evaluate_camera_detector_timing > ../Verification/Camera_Detector_Timing_Results.md",
        "```",
    ])
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    print(render_report(), end="")
