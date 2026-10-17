# Camera and visual target sensing

**Role:** capture an image, threshold dark pixels, locate the target centroid, convert image-plane errors to target bearings, and expose a low-rate thumbnail to the operator when the physical camera is enabled.

**Input:** image frames from the environment or XIAO Sense OV camera. The environment synthesizes a bright QVGA scene with a dark point and optional asymmetric orientation fiducial using target geometry, attitude, visibility, and occlusion. Host SIL fetches the frame paired to its observation and runs the Python centroid detector before control; default Wokwi firmware fetches it and runs the portable C detector. Both report the frame sequence. The physical camera profile processes a local frame. **Onboard output:** validity, horizontal and vertical centroid offsets, optional directed target image orientation, and calibrated focal length. Host SIL uses horizontal error for yaw, vertical error for pitch, and the marker orientation for roll. The portable C detector also estimates marker orientation; ESP-IDF emits it in telemetry and uses it for virtual roll control only when processing a simulator frame. Physical actuation remains yaw-only. Telemetry uses image coordinates: positive X is right and positive Y is down. **Ground output:** 64x48 thumbnail averaged from the same detected frame, with matching X/Y error. A missing physical frame stays unavailable; only an explicitly simulator-sourced sequence can select a synthetic preview.

The C detector's Y offset is positive downward. Firmware converts it to
`atan(error_y_px / focal_length_px)` before pitch control, matching the host
SIL's positive-pitch feedback sign. Source contracts check this conversion;
the portable C controller test also checks positive camera error produces
positive pitch torque. This boundary is important because the environment's
analytic Y coordinate uses positive up while raster rows increase down.

**Implementation:** [Python reference](../../Python_sim/CubeSat_ESP32/camera.py), [lossless simulator frame codec](../../Python_sim/Sim_Environment/camera_transport.py); firmware RLE1 decoder, detector and XIAO adapter: `../../Python_sim/CubeSat_ESP32/firmware/main/black_target.c` and `xiao_camera.c`.

**Physical focus:** XIAO Sense boards may have OV3660 or OV5640 modules. The
firmware uses the sensor PID: OV5640 must support and complete Espressif
autofocus (10 s timeout and an explicit focused status) or physical camera
startup is rejected; OV3660 follows a fixed-focus path, and unknown sensor
variants are rejected. The
managed `esp32-camera` component is pinned to compatible range `^2.1.8`, and
`CONFIG_CAMERA_AF_SUPPORT=y` is in the project defaults. See the [Seeed
camera guide](https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/) and
[Espressif component/API notes](https://components.espressif.com/components/espressif/esp32-camera/versions/2.1.8/readme).

**Firmware frame boundary:** the portable C detector supports rasters up to
640×480 pixels and row strides up to 640 bytes. Its API requires callers to
pass the accessible grayscale buffer length and rejects any frame shorter than
`stride × height` before reading pixels. The physical camera adapter passes the
driver's `frame->len`; the simulator passes the static QVGA array size. C test
source covers a truncated frame, but the host C suite has not run because this
workspace has no C compiler.

**Initial detector settings:** both paths use threshold 40 and accept 50–200 dark pixels. The Python reference now rejects thresholds outside 0–255 and area limits larger than the image, matching the portable C API. These settings are demonstrator values and need calibration on actual camera frames.

**Synthetic validity:** the environment reports raster dark-pixel count from the exact retained QVGA frame. In-view targets that are undersized, oversized, or clipped below the 50-pixel minimum remain in the raw image but have invalid reference detection and null analytic pixel-error fields. A clipped mask that still passes the area limit can bias the centroid; add and validate an edge-reacquisition or shape-fitting strategy before relying on pointing at the FOV boundary.

**Target size/range envelope:** [the synthetic geometry sweep](../../Verification/Camera_Geometry_Results.md) tests four target diameters, four depths, three horizontal bearings, and three vertical bearings against actual rendered frames and the onboard detector. It records raw/coherent pixel counts, checks renderer/detector validity parity, and compares horizontal/vertical image errors with their coordinate signs. Five extra combined body poses verify the raster detector against the analytic camera reference with zero pixel discrepancy in both axes at those samples. This is an assumed focal length, a 60°×46.9° nominal field of view, one camera-facing disk, and ideal background; it does not define a hardware operating envelope.

**Projection check:** five combined roll/pitch/yaw poses match an independently assembled rigid-body rotation matrix with zero rounded-pixel discrepancy in X and Y. This verifies the simulated coordinate transforms, not lens intrinsics or physical camera alignment.

**Closed-loop axes:** host SIL has verified camera-centering commands on yaw
(image X), pitch (image Y), and roll (directed orientation of the asymmetric
marker). The directed-marker roll loop was tested from both +15° and −15°
with the accelerometer roll cue intentionally hidden. It settled to 0.152° /
−0.073°/s in 8.32 s and −0.417° / −0.013°/s in 9.16 s, respectively.
Featureless point-only frames return no orientation and use gravity-derived IMU
leveling. The optional
simulator marker is a project design addition; the physical poster must include
an equivalent resolvable asymmetric feature before visual roll control is
available. The portable C detector reports directed marker angle, and the C
firmware sends virtual pitch/roll torque only for a simulator-sourced frame;
physical motor actuation is still yaw-only. See
[`Axis_Control_Results.md`](../../Verification/Axis_Control_Results.md).

**Detector geometry check:** the default 0.1 m synthetic disk at 2 m remains detectable across 13 bearings from -30° to +30° in 5° steps. The detector centroid differs from the environment's analytic pixel error by at most 2.91 px because the rasterized disk is clipped at the FOV boundary. This is an ideal scene check, not detection-rate, latency, or hardware evidence. Target scales and ranges outside this baseline may fall below the 50-pixel minimum or exceed the 200-pixel maximum; tune those settings against the actual poster and camera.

**Pixel-noise sensitivity:** [the reproducible image study](../../Verification/Camera_Image_Noise_Results.md) injects additive Gaussian intensity noise before the Python detector. The detector keeps a thresholded pixel only when at least two other dark pixels occur in its 8-neighbourhood; this local-coherence filter is mirrored in the C source. With 30 deterministic seeds per bearing, σ≤30 retains 210/210 target samples with at most 0.457 px centroid change; σ=40 retains 209/210 (0.769 px max), and σ=50 retains 191/210 (1.153 px max), with 0/30 blank-scene detections at both higher levels. These are stress assumptions, not camera specifications. The filter is not full connected-component segmentation; actual noise, lighting, blur, and exposure robustness still require measured images.

**Host detector timing:** [the reproducible timing study](../../Verification/Camera_Detector_Timing_Results.md) measures only the Python QVGA detector. The bitset implementation matches a direct-neighbour reference exactly while avoiding the dense-frame slowdown in the earlier nested-neighbour scan. In this WSL run, 50 calls per case measured about 2.0 ms median on clean/noisy/blank scenes and 5.9 ms median on an all-dark frame (10.6 ms maximum). The synthetic dense cases are stress patterns, and host Python timing is not ESP32/C timing or end-to-end 50 Hz evidence. The 20 ms firmware-cycle budget still needs on-target measurement.

**Freshness:** physical firmware timestamps each locally captured frame; simulated SIL/Wokwi age from the start of the observation request through delivery of the paired frame. Both camera and IMU samples must be valid and within the controller's 100 ms limit; a missing, negative, non-finite, stale, or future age fails closed and inhibits torque. If a fresh valid frame has no detected target, camera correction is disabled and the controller uses fresh IMU data for yaw-rate damping. Invalid ages are reported as null telemetry.

**Simulation transport budget:** host SIL and Wokwi fetch sequence-paired frames in the lossless RLE1 format, then decode to the full grayscale raster before image detection. The deterministic default frame encodes to 91 bytes (36.4 kbit/s at 50 Hz); the 60-case report sweep reached 1,447 bytes maximum. The API rejects encoded frames above 8,192 bytes (3.28 Mbit/s maximum payload at 50 Hz). See [reproducible camera transport results](../../Verification/Camera_Transport_Results.md). These figures describe this synthetic flat-background scene and a payload bound, not Wokwi throughput. Measure gateway latency, throughput, frame expiry and missed control deadlines before claiming real-time emulation. The raw endpoint remains available for inspection.

**Preview alignment:** the firmware posts actuator telemetry for a captured frame immediately before uploading that frame's 64x48 preview. The bridge stores the telemetry's target-valid flag and X/Y offsets with the thumbnail and returns them in `X-Camera-Target-Valid`, `X-Camera-Error-X-Px`, and `X-Camera-Error-Y-Px` response headers. Offsets remain in original QVGA pixel units; the Ground Station uses them on its 320x240 display canvas, which matches the upscaled preview dimensions.

**Limit:** assumes one dark target on a bright field; no full connected-component segmentation, optical distortion correction, or validated thresholds. Firmware is not compiled or tested on the kit. Confirm whether the module is OV5640 and calibrate lens, image centre, orientation, and thresholds.

**Verify next:** compare the Python and C detectors on identical synthetic QVGA frames, measure the Wokwi image-transfer budget under its private gateway, verify ESP-IDF/Wokwi build and run, perform real-frame bench tests, assess exposure/lighting robustness, and confirm image-error sign convention. C/Wokwi behavior and live browser rendering remain unverified because build/emulator/browser tools and loopback sockets are unavailable here.
