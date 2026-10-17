"""Evaluate lossless synthetic-camera frame size and codec cost."""

from datetime import datetime, timezone
import math
import time

from Sim_Environment.camera_transport import (MAX_ENCODED_BYTES,
                                              decode_rle_frame,
                                              encode_rle_frame)
from Sim_Environment.main import (CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX,
                                  Environment)

CONTROL_RATE_HZ = 50
FRAME_BYTES = CAMERA_WIDTH_PX * CAMERA_HEIGHT_PX
DEPTHS_M = (0.001, 0.1, 2.0, 1000.0)
BEARINGS_DEG = (-30.0, 0.0, 30.0)
TARGET_SCALES_M = (0.001, 0.015, 0.1, 0.5, 2.0)


def _scene_frames():
    for depth_m in DEPTHS_M:
        for bearing_deg in BEARINGS_DEG:
            for scale_m in TARGET_SCALES_M:
                environment = Environment()
                environment.apply({
                    "target_xyz_m": {
                        "x": depth_m * math.tan(math.radians(bearing_deg)),
                        "y": 0.0,
                        "z": depth_m,
                    },
                    "target_scale": scale_m,
                })
                observation = environment.observation()
                frame = environment.camera_frame_snapshot(
                    observation["sequence"]
                )
                yield depth_m, bearing_deg, scale_m, frame


def _percentile(sorted_values, fraction):
    return sorted_values[math.ceil(len(sorted_values) * fraction) - 1]


def _timing_samples(operation, iterations):
    samples_ms = []
    for _ in range(iterations):
        started = time.perf_counter()
        operation()
        samples_ms.append((time.perf_counter() - started) * 1000.0)
    samples_ms.sort()
    return {
        "mean_ms": sum(samples_ms) / len(samples_ms),
        "p95_ms": _percentile(samples_ms, 0.95),
        "max_ms": samples_ms[-1],
    }


def evaluate_camera_transport(timing_iterations=300):
    """Return deterministic scene-size evidence and local codec timings."""
    if (isinstance(timing_iterations, bool) or
            not isinstance(timing_iterations, int) or timing_iterations < 1):
        raise ValueError("timing_iterations must be a positive integer")

    scene_payload_sizes = []
    scene_count = 0
    for depth_m, bearing_deg, scale_m, frame in _scene_frames():
        encoded = encode_rle_frame(frame)
        reconstructed = decode_rle_frame(encoded, FRAME_BYTES)
        if reconstructed != frame:
            raise RuntimeError("RLE1 failed exact frame reconstruction")
        if len(encoded) > MAX_ENCODED_BYTES:
            raise RuntimeError(
                "scene frame exceeds the RLE1 transfer bound: "
                f"depth={depth_m}, bearing={bearing_deg}, scale={scale_m}, "
                f"bytes={len(encoded)}"
            )
        scene_count += 1
        scene_payload_sizes.append(len(encoded))

    default_environment = Environment()
    observation = default_environment.observation()
    default_frame = default_environment.camera_frame_snapshot(
        observation["sequence"]
    )
    default_encoded = encode_rle_frame(default_frame)
    default_decoded = decode_rle_frame(default_encoded, FRAME_BYTES)
    encode_timing = _timing_samples(
        lambda: encode_rle_frame(default_frame), timing_iterations
    )
    decode_timing = _timing_samples(
        lambda: decode_rle_frame(default_encoded, FRAME_BYTES),
        timing_iterations,
    )

    return {
        "generated_utc": datetime.now(timezone.utc).strftime(
            "%Y-%m-%d %H:%M:%S UTC"
        ),
        "frame_width_px": CAMERA_WIDTH_PX,
        "frame_height_px": CAMERA_HEIGHT_PX,
        "raw_frame_bytes": len(default_frame),
        "scene_count": scene_count,
        "scene_payload_min_bytes": min(scene_payload_sizes),
        "scene_payload_max_bytes": max(scene_payload_sizes),
        "scene_payload_mean_bytes": sum(scene_payload_sizes) / scene_count,
        "default_payload_bytes": len(default_encoded),
        "default_round_trip_exact": default_decoded == default_frame,
        "max_encoded_bytes": MAX_ENCODED_BYTES,
        "control_rate_hz": CONTROL_RATE_HZ,
        "raw_payload_kbit_s": len(default_frame) * CONTROL_RATE_HZ * 8 / 1000,
        "default_payload_kbit_s": (len(default_encoded) * CONTROL_RATE_HZ *
                                   8 / 1000),
        "max_bound_payload_mbit_s": (MAX_ENCODED_BYTES * CONTROL_RATE_HZ *
                                     8 / 1_000_000),
        "compression_ratio": len(default_frame) / len(default_encoded),
        "timing_iterations": timing_iterations,
        "encode_timing": encode_timing,
        "decode_timing": decode_timing,
    }


def render_report(timing_iterations=300):
    result = evaluate_camera_transport(timing_iterations)
    lines = [
        "# Synthetic Camera Transport Evaluation",
        "",
        f"Generated: {result['generated_utc']}",
        "",
        "This report checks the synthetic scene's lossless RLE1 transport and",
        "measures codec CPU time on the host that runs this command. It does not",
        "measure HTTP, Wi-Fi, Wokwi gateway throughput, ESP32 execution time, or",
        "compression of noisy physical camera frames.",
        "",
        "## Payload results",
        "",
        "| Measurement | Result |",
        "|---|---:|",
        (f"| Raw QVGA frame | {result['raw_frame_bytes']:,} bytes "
         f"({result['frame_width_px']}×{result['frame_height_px']}, 8-bit) |"),
        (f"| Default RLE1 frame | {result['default_payload_bytes']} bytes "
         f"({result['compression_ratio']:.1f}× smaller) |"),
        (f"| Default frame payload at {CONTROL_RATE_HZ} Hz | "
         f"{result['default_payload_kbit_s']:.2f} kbit/s |"),
        (f"| Raw image payload at {CONTROL_RATE_HZ} Hz | "
         f"{result['raw_payload_kbit_s'] / 1000:.2f} Mbit/s |"),
        (f"| RLE1 API maximum | {result['max_encoded_bytes']:,} bytes/frame "
         f"({result['max_bound_payload_mbit_s']:.2f} Mbit/s at "
         f"{CONTROL_RATE_HZ} Hz) |"),
        (f"| Scene sweep | {result['scene_count']} frames; "
         f"{result['scene_payload_min_bytes']}–"
         f"{result['scene_payload_max_bytes']} bytes; mean "
         f"{result['scene_payload_mean_bytes']:.1f} bytes |"),
        (f"| Exact reconstruction | "
         f"{'PASS' if result['default_round_trip_exact'] else 'FAIL'} "
         "for every sweep frame |"),
        "",
        "The sweep combines four optical-axis depths (0.001, 0.1, 2, and 1000 m),",
        "three horizontal bearings (−30°, 0°, and +30°), and five marker sizes",
        "(0.001, 0.015, 0.1, 0.5, and 2 m). Every encoded frame is checked against",
        "the 8,192-byte API cap and decoded byte-for-byte.",
        "",
        "## Host codec timing",
        "",
        f"Measured over {result['timing_iterations']} encode/decode iterations "
        "of the default frame on this host:",
        "",
        "| Operation | Mean | p95 | Maximum |",
        "|---|---:|---:|---:|",
        (f"| RLE1 encode | {result['encode_timing']['mean_ms']:.3f} ms | "
         f"{result['encode_timing']['p95_ms']:.3f} ms | "
         f"{result['encode_timing']['max_ms']:.3f} ms |"),
        (f"| RLE1 decode | {result['decode_timing']['mean_ms']:.3f} ms | "
         f"{result['decode_timing']['p95_ms']:.3f} ms | "
         f"{result['decode_timing']['max_ms']:.3f} ms |"),
        "",
        "These local timings exclude camera rendering and all network activity.",
        "The C decoder is source-tested but has not been compiled in the current",
        "environment. The encoded-size result applies only to the simulator's",
        "flat background and single circular marker.",
        "",
        "## Reproduce",
        "",
        "From `outputs/Python_sim/`, run:",
        "",
        "```bash",
        "python3 -m Verification.evaluate_camera_transport > ../Verification/Camera_Transport_Results.md",
        "```",
    ]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    print(render_report(), end="")
