# Synthetic Camera Transport Evaluation

Generated: 2026-10-17 11:06:33 UTC

This report checks the synthetic scene's lossless RLE1 transport and
measures codec CPU time on the host that runs this command. It does not
measure HTTP, Wi-Fi, Wokwi gateway throughput, ESP32 execution time, or
compression of noisy physical camera frames.

## Payload results

| Measurement | Result |
|---|---:|
| Raw QVGA frame | 76,800 bytes (320×240, 8-bit) |
| Default RLE1 frame | 91 bytes (844.0× smaller) |
| Default frame payload at 50 Hz | 36.40 kbit/s |
| Raw image payload at 50 Hz | 30.72 Mbit/s |
| RLE1 API maximum | 8,192 bytes/frame (3.28 Mbit/s at 50 Hz) |
| Scene sweep | 60 frames; 10–1447 bytes; mean 263.5 bytes |
| Exact reconstruction | PASS for every sweep frame |

The sweep combines four optical-axis depths (0.001, 0.1, 2, and 1000 m),
three horizontal bearings (−30°, 0°, and +30°), and five marker sizes
(0.001, 0.015, 0.1, 0.5, and 2 m). Every encoded frame is checked against
the 8,192-byte API cap and decoded byte-for-byte.

## Host codec timing

Measured over 300 encode/decode iterations of the default frame on this host:

| Operation | Mean | p95 | Maximum |
|---|---:|---:|---:|
| RLE1 encode | 4.373 ms | 6.011 ms | 8.536 ms |
| RLE1 decode | 0.011 ms | 0.012 ms | 0.042 ms |

These local timings exclude camera rendering and all network activity.
The C decoder is source-tested but has not been compiled in the current
environment. The encoded-size result applies only to the simulator's
flat background and single circular marker.

## Reproduce

From `outputs/Python_sim/`, run:

```bash
python3 -m Verification.evaluate_camera_transport > ../Verification/Camera_Transport_Results.md
```
