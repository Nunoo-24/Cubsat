"""Sweep synthetic target size, depth, and 2D bearing through the detector."""

from datetime import datetime, timezone
import math

from CubeSat_ESP32.camera import detect_black_target
from Sim_Environment.main import (CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX,
                                  CAMERA_FOCAL_PX, Environment)


TARGET_DIAMETERS_M = (0.05, 0.10, 0.15, 0.20)
TARGET_DEPTHS_M = (1.0, 2.0, 3.0, 4.0)
BEARINGS_DEG = (-30.0, 0.0, 30.0)
ELEVATIONS_DEG = (-20.0, 0.0, 20.0)
BODY_POSES_DEG = (
    (0.0, 0.0, 0.0),
    (10.0, 0.0, 0.0),
    (-15.0, 10.0, 5.0),
    (5.0, -8.0, 10.0),
    (-10.0, -8.0, -12.0),
)


def evaluate_camera_geometry(diameters_m=TARGET_DIAMETERS_M,
                             depths_m=TARGET_DEPTHS_M,
                             bearings_deg=BEARINGS_DEG,
                             elevations_deg=ELEVATIONS_DEG):
    """Return one grouped result per diameter/depth, from actual raster images."""
    diameters_m = tuple(diameters_m)
    depths_m = tuple(depths_m)
    bearings_deg = tuple(bearings_deg)
    elevations_deg = tuple(elevations_deg)
    if not diameters_m or not depths_m or not bearings_deg or not elevations_deg:
        raise ValueError("diameter, depth, and 2D bearing grids must be nonempty")
    if any(not math.isfinite(v) or v <= 0.0 for v in diameters_m + depths_m):
        raise ValueError("target diameters and depths must be positive finite values")
    if any(not math.isfinite(v) or abs(v) > 30.0 for v in bearings_deg):
        raise ValueError("bearings must be finite and within ±30 degrees")
    if any(not math.isfinite(v) or abs(v) > 20.0 for v in elevations_deg):
        raise ValueError("elevations must be finite and within ±20 degrees")

    results = []
    for diameter_m in diameters_m:
        for depth_m in depths_m:
            valid_bearings = 0
            coherent_counts = []
            raw_counts = []
            centroid_errors = []
            centroid_y_errors = []
            for bearing_deg in bearings_deg:
                bearing_rad = math.radians(bearing_deg)
                for elevation_deg in elevations_deg:
                    elevation_rad = math.radians(elevation_deg)
                    environment = Environment()
                    environment.apply({
                        "target_xyz_m": {
                            "x": depth_m * math.tan(bearing_rad),
                            "y": depth_m * math.tan(elevation_rad),
                            "z": depth_m,
                        },
                        "target_scale": diameter_m,
                    })
                    observation = environment.observation()
                    camera = observation["sensors"]["camera"]
                    frame = environment.camera_frame_snapshot(
                        observation["sequence"]
                    )
                    detection = detect_black_target(
                        frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX
                    )
                    raw_counts.append(detection["dark_pixel_count"])
                    coherent_counts.append(detection["coherent_pixel_count"])
                    if detection["valid"]:
                        valid_bearings += 1
                        expected_x_px = CAMERA_FOCAL_PX * math.tan(bearing_rad)
                        expected_y_up_px = (
                            CAMERA_FOCAL_PX * math.tan(elevation_rad)
                        )
                        centroid_errors.append(abs(
                            detection["error_x_px"] - expected_x_px
                        ))
                        # Detector image coordinates are positive down, while
                        # environment camera coordinates are positive up.
                        centroid_y_errors.append(abs(
                            -detection["error_y_px"] - expected_y_up_px
                        ))
                    # The analytic reference and raster detector must agree
                    # before either camera error is exposed to control.
                    if bool(camera["valid"]) != bool(detection["valid"]):
                        raise RuntimeError(
                            f"renderer/detector validity mismatch for "
                            f"d={diameter_m}, z={depth_m}, h={bearing_deg}, "
                            f"v={elevation_deg}"
                        )
            results.append({
                "diameter_m": float(diameter_m),
                "depth_m": float(depth_m),
                "trials": len(bearings_deg) * len(elevations_deg),
                "valid": valid_bearings,
                "raw_dark_pixels_min": min(raw_counts),
                "raw_dark_pixels_max": max(raw_counts),
                "coherent_pixels_min": min(coherent_counts),
                "coherent_pixels_max": max(coherent_counts),
                "centroid_error_max_px": max(centroid_errors, default=None),
                "centroid_y_error_max_px": max(centroid_y_errors, default=None),
            })
    return results


def evaluate_camera_pose_grid(poses_deg=BODY_POSES_DEG):
    """Run the detector on a fixed world target across combined body attitudes."""
    pose_results = []
    for yaw_deg, pitch_deg, roll_deg in poses_deg:
        environment = Environment()
        environment.plant.yaw_rad = math.radians(yaw_deg)
        environment.plant.pitch_rad = math.radians(pitch_deg)
        environment.plant.roll_rad = math.radians(roll_deg)
        observation = environment.observation()
        camera = observation["sensors"]["camera"]
        frame = environment.camera_frame_snapshot(observation["sequence"])
        detection = detect_black_target(frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
        if bool(camera["valid"]) != bool(detection["valid"]):
            raise RuntimeError(
                f"renderer/detector validity mismatch for pose "
                f"yaw={yaw_deg}, pitch={pitch_deg}, roll={roll_deg}"
            )
        if not detection["valid"]:
            error_x_px = error_y_px = None
        else:
            reference_x = camera["pixel_error_x"]
            reference_y_up = camera["pixel_error_y"]
            error_x_px = abs(detection["error_x_px"] - reference_x)
            error_y_px = abs(-detection["error_y_px"] - reference_y_up)
        pose_results.append({
            "yaw_deg": float(yaw_deg),
            "pitch_deg": float(pitch_deg),
            "roll_deg": float(roll_deg),
            "valid": detection["valid"],
            "coherent_pixels": detection["coherent_pixel_count"],
            "error_x_px": error_x_px,
            "error_y_px": error_y_px,
        })
    return pose_results


def render_report(results, pose_results=()):
    lines = [
        "# Synthetic Camera Target Geometry Sweep",
        "",
        f"Generated: {datetime.now(timezone.utc):%Y-%m-%d %H:%M:%S UTC}",
        "",
        "This screen renders the environment's synthetic QVGA grayscale",
        "frames, runs the onboard Python detector, and compares its validity with",
        "the environment's analytic camera-reference flag. It is a renderer to",
        "detector consistency check, not a physical-camera measurement.",
        "",
        "## Setup",
        "",
        f"Target diameters: {', '.join(f'{x:.2f}' for x in TARGET_DIAMETERS_M)} m; "
        f"depths: {', '.join(f'{x:.1f}' for x in TARGET_DEPTHS_M)} m; horizontal "
        f"bearings: {', '.join(f'{x:+.0f}°' for x in BEARINGS_DEG)}; vertical "
        f"bearings: {', '.join(f'{x:+.0f}°' for x in ELEVATIONS_DEG)}.",
        "The detector uses threshold 40 gray levels, minimum 50 and maximum 200",
        "coherent pixels, and the 8-neighbour coherence filter. Its focal length",
        f"is the provisional simulator value {CAMERA_FOCAL_PX:.1f} px. The size/",
        "range grid contains 144 images; a separate five-pose grid checks the",
        "detector against the analytic camera reference as the body attitude changes.",
        "",
        "## Results",
        "",
        "| Target diameter (m) | Depth (m) | Valid samples | Raw dark pixels | Coherent pixels | Max X error (px) | Max Y error (px) |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in results:
        err = item["centroid_error_max_px"]
        err_text = "—" if err is None else f"{err:.2f}"
        err_y = item["centroid_y_error_max_px"]
        err_y_text = "—" if err_y is None else f"{err_y:.2f}"
        lines.append(
            f"| {item['diameter_m']:.2f} | {item['depth_m']:.1f} | "
            f"{item['valid']}/{item['trials']} | "
            f"{item['raw_dark_pixels_min']}–{item['raw_dark_pixels_max']} | "
            f"{item['coherent_pixels_min']}–{item['coherent_pixels_max']} | "
            f"{err_text} | {err_y_text} |"
        )
    lines.extend([
        "",
        "Raw dark pixels count all threshold crossings; coherent pixels count",
        "only pixels with at least two other dark 8-neighbours and determine the",
        "area acceptance. A `—` means no tested bearing was accepted. The",
        "simulator currently checks renderer/detector validity parity; the grid",
        "shows the detector's size/range envelope under the current assumed",
        "threshold and area settings. Horizontal pixel error is right-positive;",
        "vertical detector error is down-positive and is sign-flipped when compared",
        "with the environment's up-positive camera coordinate. Confirm target",
        "dimensions, camera intrinsics,",
        "lighting, and threshold using recorded hardware images before treating",
        "this as an operational envelope.",
    ])
    if pose_results:
        lines.extend([
            "",
            "## Rendered-frame detector check across body attitude",
            "",
            "A fixed 0.1 m target at world position (0, 0, 2) m is rendered with",
            "the following combined body poses. Errors compare the actual raster",
            "detector with camera-reference metadata for the same observation.",
            "Vertical detector error is sign-flipped from image-down to camera-up.",
            "",
            "| Yaw (°) | Pitch (°) | Roll (°) | Valid | Coherent pixels | X discrepancy (px) | Y discrepancy (px) |",
            "|---:|---:|---:|:---:|---:|---:|---:|",
        ])
        for item in pose_results:
            xerr = "—" if item["error_x_px"] is None else f"{item['error_x_px']:.2f}"
            yerr = "—" if item["error_y_px"] is None else f"{item['error_y_px']:.2f}"
            lines.append(
                f"| {item['yaw_deg']:+.0f} | {item['pitch_deg']:+.0f} | "
                f"{item['roll_deg']:+.0f} | {'yes' if item['valid'] else 'no'} | "
                f"{item['coherent_pixels']} | {xerr} | {yerr} |"
            )
        lines.extend([
            "",
            "This five-pose raster check supplements the 144-case target-geometry",
            "grid. It verifies consistency at these sampled poses only; it does not",
            "establish camera calibration, dynamics, or hardware performance.",
        ])
    lines.extend([
        "",
        "## Reproduce",
        "",
        "From `outputs/Python_sim/`, run:",
        "",
        "```bash",
        "python3 -m Verification.evaluate_camera_geometry > ../Verification/Camera_Geometry_Results.md",
        "```",
        "",
    ])
    return "\n".join(lines)


def main():
    print(render_report(evaluate_camera_geometry(),
                        evaluate_camera_pose_grid()), end="")


if __name__ == "__main__":
    main()
