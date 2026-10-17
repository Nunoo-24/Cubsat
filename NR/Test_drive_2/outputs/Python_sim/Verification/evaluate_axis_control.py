"""Closed-loop axis checks for host-SIL stabilization and camera pointing."""

from datetime import datetime, timezone
import math

from CubeSat_ESP32.sil import OnboardSoftware
from Sim_Environment.main import DT_S, Environment
from Sim_Environment.model import wrap_pi


DISTURBANCES = (
    ("X / pitch", {"x": 0.01, "y": 0.0, "z": 0.0},
     "pitch_rad", "pitch_rate_rad_s", 0.02),
    ("Y / yaw", {"x": 0.0, "y": 0.005, "z": 0.0},
     "yaw_rad", "rate_rad_s", 0.01),
    ("Z / roll", {"x": 0.0, "y": 0.0, "z": 0.0175},
     "roll_rad", "roll_rate_rad_s", 0.035),
)


def _axis_disturbance_case(label, torque, angle_field, rate_field,
                           actuator_limit_nm):
    """Test target-lost axis recovery with a half-authority 0.5 s pulse."""
    environment = Environment()
    pulse_duration_s = 0.5
    environment.apply({
        "camera_occlusion_s": 8.0,
        "external_torque_nm": torque,
        "duration_s": pulse_duration_s,
    })
    onboard = OnboardSoftware()
    previous_angle = getattr(environment.plant, angle_field)
    angular_excursion_rad = 0.0
    peak_excursion_rad = 0.0
    rate_settle_since_s = None
    angle_settle_since_s = None
    rate_settle_s = None
    attitude_settle_s = None
    maximum_sim_s = 6.0

    for _ in range(round(maximum_sim_s / DT_S)):
        observation = environment.observation()
        frame = environment.camera_frame_snapshot(observation["sequence"])
        telemetry = onboard.process_observation(observation,
                                                camera_frame=frame)
        environment.accept_actuator(telemetry)
        environment.advance(DT_S)

        angle = getattr(environment.plant, angle_field)
        rate = abs(getattr(environment.plant, rate_field))
        angular_excursion_rad += wrap_pi(angle - previous_angle)
        previous_angle = angle
        peak_excursion_rad = max(peak_excursion_rad,
                                 abs(angular_excursion_rad))

        rate_in_band = (environment.plant.time_s >= pulse_duration_s and
                        rate < math.radians(0.5))
        if rate_in_band:
            if rate_settle_since_s is None:
                rate_settle_since_s = environment.plant.time_s
            if (rate_settle_s is None and
                    environment.plant.time_s - rate_settle_since_s >= 0.5):
                rate_settle_s = rate_settle_since_s - pulse_duration_s
        else:
            rate_settle_since_s = None

        # The simulated gravity reference restores pitch and roll to level.
        # Yaw has no preferred direction after target loss, so only its rate
        # is expected to settle; its post-disturbance angle is retained.
        angle_in_band = (rate_in_band and
                         abs(angle) < math.radians(1.0))
        if angle_in_band:
            if angle_settle_since_s is None:
                angle_settle_since_s = environment.plant.time_s
            if (attitude_settle_s is None and
                    environment.plant.time_s - angle_settle_since_s >= 0.5):
                attitude_settle_s = angle_settle_since_s - pulse_duration_s
        else:
            angle_settle_since_s = None

    return {
        "axis": label,
        "disturbance_nm": next(value for value in torque.values()
                                if value != 0.0),
        "actuator_limit_nm": actuator_limit_nm,
        "peak_excursion_deg": math.degrees(peak_excursion_rad),
        "rate_settle_after_pulse_s": rate_settle_s,
        "attitude_settle_after_pulse_s": attitude_settle_s,
        "final_angle_deg": math.degrees(
            getattr(environment.plant, angle_field)),
        "final_rate_deg_s": math.degrees(
            getattr(environment.plant, rate_field)),
        "target_lost": not telemetry["target_valid"],
        "mode": telemetry["mode"],
        "fault": telemetry["fault"],
    }


def evaluate_stabilization_axes():
    results = []
    for label, torque, angle_field, rate_field, limit in DISTURBANCES:
        results.append(_axis_disturbance_case(
            label, torque, angle_field, rate_field, limit))
    return results


def evaluate_simultaneous_stabilization():
    """Exercise the three inertial loops under one target-lost XYZ pulse."""
    environment = Environment()
    pulse_duration_s = 0.5
    disturbance = {"x": 0.01, "y": 0.005, "z": 0.0175}
    environment.apply({
        "camera_occlusion_s": 8.0,
        "external_torque_nm": disturbance,
        "duration_s": pulse_duration_s,
    })
    onboard = OnboardSoftware()
    torque_peaks = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
    settle_since_s = None
    settle_after_pulse_s = None
    telemetry = None

    for _ in range(round(6.0 / DT_S)):
        observation = environment.observation()
        frame = environment.camera_frame_snapshot(observation["sequence"])
        telemetry = onboard.process_observation(observation,
                                                camera_frame=frame)
        environment.accept_actuator(telemetry)
        environment.advance(DT_S)
        torque_peaks["yaw"] = max(torque_peaks["yaw"],
                                  abs(telemetry["body_torque_nm"]))
        torque_peaks["pitch"] = max(torque_peaks["pitch"],
                                    abs(telemetry["pitch_body_torque_nm"]))
        torque_peaks["roll"] = max(torque_peaks["roll"],
                                   abs(telemetry["roll_body_torque_nm"]))

        rates_settled = all(abs(value) < math.radians(0.5) for value in (
            environment.plant.rate_rad_s,
            environment.plant.pitch_rate_rad_s,
            environment.plant.roll_rate_rad_s,
        ))
        tilt_settled = (abs(environment.plant.pitch_rad) < math.radians(1.0)
                        and abs(environment.plant.roll_rad) < math.radians(1.0))
        if (environment.plant.time_s >= pulse_duration_s and rates_settled
                and tilt_settled):
            if settle_since_s is None:
                settle_since_s = environment.plant.time_s
            if (settle_after_pulse_s is None and
                    environment.plant.time_s - settle_since_s >= 0.5):
                settle_after_pulse_s = settle_since_s - pulse_duration_s
        else:
            settle_since_s = None

    return {
        "disturbance_nm": disturbance,
        "torque_peaks_nm": torque_peaks,
        "settle_after_pulse_s": settle_after_pulse_s,
        "final_yaw_rate_deg_s": math.degrees(environment.plant.rate_rad_s),
        "final_pitch_deg": math.degrees(environment.plant.pitch_rad),
        "final_pitch_rate_deg_s": math.degrees(environment.plant.pitch_rate_rad_s),
        "final_roll_deg": math.degrees(environment.plant.roll_rad),
        "final_roll_rate_deg_s": math.degrees(environment.plant.roll_rate_rad_s),
        "target_lost": telemetry is not None and not telemetry["target_valid"],
        "mode": telemetry["mode"] if telemetry is not None else "NO_DATA",
        "fault": telemetry["fault"] if telemetry is not None else "NO_DATA",
    }


def _camera_pointing_case(name, target):
    environment = Environment()
    environment.apply({"target_xyz_m": target,
                       "camera_roll_marker_enabled": True})
    onboard = OnboardSoftware()
    within_band_since_s = None
    settle_time_s = None
    initial_error_deg = None
    final_error_deg = None
    initial_error_x_rad = None
    initial_error_y_rad = None
    final_error_x_rad = None
    final_error_y_rad = None
    target_valid_samples = 0

    for _ in range(round(10.0 / DT_S)):
        observation = environment.observation()
        frame = environment.camera_frame_snapshot(observation["sequence"])
        telemetry = onboard.process_observation(observation,
                                                camera_frame=frame)
        environment.accept_actuator(telemetry)
        environment.advance(DT_S)
        snapshot = environment.snapshot()
        error_x_rad = snapshot["camera_truth_error_x_rad"]
        error_y_rad = snapshot["camera_truth_error_y_rad"]
        error = math.hypot(snapshot["camera_truth_error_x_rad"],
                           snapshot["camera_truth_error_y_rad"])
        error_deg = math.degrees(error)
        if initial_error_deg is None:
            initial_error_deg = error_deg
            initial_error_x_rad = error_x_rad
            initial_error_y_rad = error_y_rad
        final_error_deg = error_deg
        final_error_x_rad = error_x_rad
        final_error_y_rad = error_y_rad
        target_valid_samples += int(telemetry["target_valid"])
        if telemetry["target_valid"] and error <= math.radians(2.0):
            if within_band_since_s is None:
                within_band_since_s = environment.plant.time_s
            if (settle_time_s is None and
                    environment.plant.time_s - within_band_since_s >= 0.5):
                settle_time_s = within_band_since_s
        else:
            within_band_since_s = None

    return {
        "axis": name,
        "initial_error_deg": initial_error_deg,
        "initial_error_x_rad": initial_error_x_rad,
        "initial_error_y_rad": initial_error_y_rad,
        "settle_to_2deg_s": settle_time_s,
        "final_error_deg": final_error_deg,
        "final_error_x_rad": final_error_x_rad,
        "final_error_y_rad": final_error_y_rad,
        "valid_samples": target_valid_samples,
        "sample_count": round(10.0 / DT_S),
    }


def evaluate_camera_pointing_axes():
    return [
        _camera_pointing_case("Yaw / image X (+)",
                              {"x": 0.6, "y": 0.0, "z": 2.0}),
        _camera_pointing_case("Yaw / image X (−)",
                              {"x": -0.6, "y": 0.0, "z": 2.0}),
        _camera_pointing_case("Pitch / image Y (+)",
                              {"x": 0.0, "y": 0.5, "z": 2.0}),
        _camera_pointing_case("Pitch / image Y (−)",
                              {"x": 0.0, "y": -0.5, "z": 2.0}),
    ]



def evaluate_camera_roll_pointing(initial_roll_deg=15.0):
    """Close roll loop from image marker while hiding roll from gravity data."""
    if (not math.isfinite(initial_roll_deg) or
            initial_roll_deg == 0.0 or abs(initial_roll_deg) > 30.0):
        raise ValueError("initial roll must be finite, nonzero, and within ±30°")
    environment = Environment()
    environment.apply({"camera_roll_marker_enabled": True})
    environment.plant.roll_rad = math.radians(initial_roll_deg)
    onboard = OnboardSoftware()
    first = None
    settle_since_s = None
    settle_after_s = None
    sample_count = round(12.0 / DT_S)
    for _ in range(sample_count):
        observation = environment.observation()
        # Remove the accelerometer roll cue to verify the camera path is doing
        # the work. Gyro Z remains available for angular-rate damping.
        imu = observation["sensors"]["imu"]
        imu["accel_m_s2"].update({"x": 0.0, "y": 9.80665, "z": 0.0})
        frame = environment.camera_frame_snapshot(observation["sequence"])
        telemetry = onboard.process_observation(observation,
                                                camera_frame=frame)
        if first is None:
            first = telemetry
        environment.accept_actuator(telemetry)
        environment.advance(DT_S)
        in_band = (
            environment.plant.time_s >= 0.02 and
            abs(environment.plant.roll_rad) < math.radians(1.0) and
            abs(environment.plant.roll_rate_rad_s) < math.radians(0.5)
        )
        if in_band:
            if settle_since_s is None:
                settle_since_s = environment.plant.time_s
            if settle_after_s is None and environment.plant.time_s - settle_since_s >= 0.5:
                settle_after_s = settle_since_s
        else:
            settle_since_s = None
    return {
        "initial_roll_deg": initial_roll_deg,
        "initial_camera_orientation_deg": math.degrees(
            first["camera_roll_orientation_rad"]),
        "initial_camera_roll_estimate_deg": math.degrees(
            first["camera_roll_estimate_rad"]),
        "initial_roll_reference_source": first["roll_reference_source"],
        "initial_roll_torque_nm": first["roll_body_torque_nm"],
        "settle_to_1deg_0_5deg_s": settle_after_s,
        "final_roll_deg": math.degrees(environment.plant.roll_rad),
        "final_roll_rate_deg_s": math.degrees(
            environment.plant.roll_rate_rad_s),
        "final_roll_reference_source": telemetry["roll_reference_source"],
        "accelerometer_roll_hidden": True,
        "sample_count": sample_count,
    }


def evaluate_camera_roll_both_directions():
    """Verify marker-derived roll correction for positive and negative roll."""
    return [evaluate_camera_roll_pointing(angle_deg)
            for angle_deg in (15.0, -15.0)]

def evaluate_combined_camera_pointing():
    """Exercise yaw/pitch image centering and marker roll in one run."""
    environment = Environment()
    environment.apply({
        "target_xyz_m": {"x": 0.6, "y": 0.5, "z": 2.0},
        "camera_roll_marker_enabled": True,
    })
    environment.plant.yaw_rad = math.radians(10.0)
    environment.plant.pitch_rad = math.radians(-8.0)
    environment.plant.roll_rad = math.radians(12.0)
    initial = environment.snapshot()
    onboard = OnboardSoftware()
    first = None
    settle_since_s = None
    settle_after_s = None
    last_state = initial
    last_telemetry = None
    sample_count = round(10.0 / DT_S)

    for _ in range(sample_count):
        observation = environment.observation()
        frame = environment.camera_frame_snapshot(observation["sequence"])
        telemetry = onboard.process_observation(observation,
                                                camera_frame=frame)
        if first is None:
            first = telemetry.copy()
        environment.accept_actuator(telemetry)
        environment.advance(DT_S)
        last_state = environment.snapshot()
        last_telemetry = telemetry
        image_error_rad = math.hypot(
            last_state["camera_truth_error_x_rad"],
            last_state["camera_truth_error_y_rad"],
        )
        in_band = (
            telemetry["target_valid"] and
            telemetry["roll_reference_source"] == "CAMERA" and
            image_error_rad <= math.radians(2.0) and
            abs(environment.plant.roll_rad) <= math.radians(1.0) and
            abs(environment.plant.roll_rate_rad_s) <= math.radians(0.5)
        )
        if in_band:
            if settle_since_s is None:
                settle_since_s = environment.plant.time_s
            if (settle_after_s is None and
                    environment.plant.time_s - settle_since_s >= 0.5):
                settle_after_s = settle_since_s
        else:
            settle_since_s = None

    return {
        "initial_image_error_deg": math.degrees(math.hypot(
            initial["camera_truth_error_x_rad"],
            initial["camera_truth_error_y_rad"],
        )),
        "initial_image_error_x_px": first["camera_error_x_px"],
        "initial_image_error_y_px": first["camera_error_y_px"],
        "initial_roll_deg": 12.0,
        "initial_roll_reference_source": first["roll_reference_source"],
        "settle_to_all_axes_s": settle_after_s,
        "final_image_error_deg": math.degrees(math.hypot(
            last_state["camera_truth_error_x_rad"],
            last_state["camera_truth_error_y_rad"],
        )),
        "final_roll_deg": math.degrees(environment.plant.roll_rad),
        "final_roll_rate_deg_s": math.degrees(
            environment.plant.roll_rate_rad_s),
        "final_roll_reference_source": last_telemetry["roll_reference_source"],
        "final_target_valid": last_telemetry["target_valid"],
        "sample_count": sample_count,
    }


def evaluate_point_target_roll_observability():
    """Show that the current point-only camera cue does not command roll."""
    environment = Environment()
    environment.apply({"camera_roll_marker_enabled": True})
    environment.plant.roll_rad = math.radians(15.0)
    onboard = OnboardSoftware()
    observation = environment.observation()
    frame = environment.camera_frame_snapshot(observation["sequence"])
    telemetry = onboard.process_observation(observation, camera_frame=frame)
    camera = observation["sensors"]["camera"]
    return {
        "roll_deg": math.degrees(environment.plant.roll_rad),
        "target_valid": telemetry["target_valid"],
        "pixel_error_x": camera["pixel_error_x"],
        "pixel_error_y": camera["pixel_error_y"],
        "imu_roll_torque_nm": telemetry["roll_body_torque_nm"],
    }


def render_report(stabilization=None, pointing=None, roll_cue=None, camera_roll=None,
                  combined=None, simultaneous=None, camera_roll_both=None):
    stabilization = (evaluate_stabilization_axes() if stabilization is None
                     else stabilization)
    pointing = evaluate_camera_pointing_axes() if pointing is None else pointing
    roll_cue = (evaluate_point_target_roll_observability()
                if roll_cue is None else roll_cue)
    if camera_roll_both is None:
        if camera_roll is None:
            camera_roll_both = evaluate_camera_roll_both_directions()
        else:
            camera_roll_both = [camera_roll,
                                evaluate_camera_roll_pointing(-15.0)]
    camera_roll = camera_roll_both[0]
    combined = (evaluate_combined_camera_pointing() if combined is None
                else combined)
    simultaneous = (evaluate_simultaneous_stabilization()
                    if simultaneous is None else simultaneous)
    lines = [
        "# Three-Axis Stabilization and Camera-Pointing Verification",
        "",
        f"Generated: {datetime.now(timezone.utc):%Y-%m-%d %H:%M:%S UTC}",
        "",
        "This report exercises the host Python SIL, synthetic QVGA camera, and",
        "decoupled 3-axis plant. It is not a live-socket, ESP32 firmware,",
        "emulator, calibrated sensor, or hardware test.",
        "",
        "## Inertial stabilization after target loss",
        "",
        "Each run starts with an 8 s camera occlusion and applies a 0.5 s torque",
        "pulse at half the corresponding simulated actuator limit: X/pitch",
        "0.010 N·m, Y/yaw 0.005 N·m, and Z/roll 0.0175 N·m. Rate settling",
        "means under 0.5°/s continuously for 0.5 s. Pitch/roll attitude settling",
        "also requires under 1° from the gravity-level reference. Yaw has no",
        "absolute hold reference after target loss; it damps rate and retains the",
        "new heading.",
        "",
        "| Axis | Pulse (N·m) | Peak excursion (°) | Rate settled after pulse (s) | Attitude settled after pulse (s) | Final angle (°) | Final rate (°/s) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for item in stabilization:
        rate_time = ("—" if item["rate_settle_after_pulse_s"] is None else
                     f"{item['rate_settle_after_pulse_s']:.2f}")
        attitude_time = (
            "rate-only" if item["attitude_settle_after_pulse_s"] is None else
            f"{item['attitude_settle_after_pulse_s']:.2f}")
        lines.append(
            f"| {item['axis']} | {item['disturbance_nm']:.4f} | "
            f"{item['peak_excursion_deg']:.2f} | {rate_time} | "
            f"{attitude_time} | {item['final_angle_deg']:.2f} | "
            f"{item['final_rate_deg_s']:.3f} |"
        )
    lines.extend([
        "",
        "A concurrent run applies all three half-limit torques during the same",
        "8 s target occlusion. The independent control outputs peak at",
        f"{simultaneous['torque_peaks_nm']['yaw']:.4f} N·m yaw,",
        f"{simultaneous['torque_peaks_nm']['pitch']:.4f} N·m pitch, and",
        f"{simultaneous['torque_peaks_nm']['roll']:.4f} N·m roll. All three",
        "rates stay below 0.5°/s and pitch/roll angles below 1° for a 0.5 s",
        f"dwell, settling {simultaneous['settle_after_pulse_s']:.2f} s after",
        f"the 0.5 s pulse. Final rates are yaw {simultaneous['final_yaw_rate_deg_s']:.3f}°/s,",
        f"pitch {simultaneous['final_pitch_rate_deg_s']:.3f}°/s, and roll",
        f"{simultaneous['final_roll_rate_deg_s']:.3f}°/s; target lost=",
        f"{simultaneous['target_lost']}, mode={simultaneous['mode']},",
        f"fault={simultaneous['fault']}.",
        "",
        "The yaw result demonstrates rate damping, not absolute yaw hold. The",
        "camera-loss pitch/roll loops use gravity-derived attitude and return",
        "toward level. All values are for the current provisional plant and",
        "simulated torque limits.",
        "",
        "## Closed-loop camera pointing",
        "",
        "Yaw and pitch are tested independently in both error directions from",
        "world target points at (+/-0.6, 0, 2) m and (0, +/-0.5, 2) m. A pass",
        "requires combined camera truth error ≤2° for a continuous 0.5 s dwell.",
        "",
        "| Camera-controlled case | Initial X error (°) | Initial Y error (°) | First settled dwell (s) | Final error (°) | Valid target samples |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for item in pointing:
        settle = ("not settled" if item["settle_to_2deg_s"] is None else
                  f"{item['settle_to_2deg_s']:.2f}")
        lines.append(
            f"| {item['axis']} | "
            f"{math.degrees(item['initial_error_x_rad']):.2f} | "
            f"{math.degrees(item['initial_error_y_rad']):.2f} | "
            f"{settle} | {item['final_error_deg']:.2f} | "
            f"{item['valid_samples']}/{item['sample_count']} |"
        )
    lines.extend([
        "",
        "## Combined three-axis camera correction",
        "",
        "Yaw and pitch image centering plus camera-marker roll correction were",
        "run together from an off-axis target and combined body attitude.",
        f"Initial image error was {combined['initial_image_error_deg']:.2f}° "
        f"({combined['initial_image_error_x_px']:.1f}, "
        f"{combined['initial_image_error_y_px']:.1f} px); initial roll was "
        f"{combined['initial_roll_deg']:.1f}° with "
        f"{combined['initial_roll_reference_source']} reference.",
        f"Image error ≤2°, roll within ±1°, and roll rate within ±0.5°/s "
        f"for 0.5 s starting at {combined['settle_to_all_axes_s']:.2f} "
        "simulated seconds.",
        f"At 10 s, image error was {combined['final_image_error_deg']:.3f}°, "
        f"roll {combined['final_roll_deg']:.3f}°, and roll rate "
        f"{combined['final_roll_rate_deg_s']:.3f}°/s; target valid="
        f"{combined['final_target_valid']}, roll reference="
        f"{combined['final_roll_reference_source']}.",
        "",
        "## Point-only target fallback",
        "",
        f"With the cube rolled {roll_cue['roll_deg']:.1f}° and the point centered,",
        f"camera X/Y errors remain {roll_cue['pixel_error_x']}/{roll_cue['pixel_error_y']} px",
        f"and the detector reports no roll angle. Roll torque is",
        f"{roll_cue['imu_roll_torque_nm']:.4f} N·m from the gravity-derived IMU.",
        "This is the intended fallback when the optional asymmetric marker is",
        "disabled, outside its inner-field visibility region, or not resolved.",
        "",
        "## Camera roll closed-loop test with oriented marker",
        "",
        f"Initial body roll is {camera_roll['initial_roll_deg']:.2f}°. The detected",
        f"image marker angle is {camera_roll['initial_camera_orientation_deg']:.2f}°",
        f"and the inferred body-roll estimate is",
        f"{camera_roll['initial_camera_roll_estimate_deg']:.2f}°. The roll",
        f"reference source is {camera_roll['initial_roll_reference_source']};",
        "accelerometer roll was overwritten as level in every cycle while gyro",
        f"Z remained available. The body returns to ±1° and ±0.5°/s in",
        f"{camera_roll['settle_to_1deg_0_5deg_s']:.2f} s, ending at",
        f"{camera_roll['final_roll_deg']:.3f}° and",
        f"{camera_roll['final_roll_rate_deg_s']:.3f}°/s.",
        "",
        "The same test was repeated from the opposite roll direction. With the",
        f"body initialized at {camera_roll_both[1]['initial_roll_deg']:.1f}°, the",
        f"camera estimated {camera_roll_both[1]['initial_camera_roll_estimate_deg']:.2f}°",
        f"and commanded {camera_roll_both[1]['initial_roll_torque_nm']:.4f} N·m",
        f"(opposite in sign to the initial roll). It returned to",
        f"{camera_roll_both[1]['final_roll_deg']:.3f}° and",
        f"{camera_roll_both[1]['final_roll_rate_deg_s']:.3f}°/s in",
        f"{camera_roll_both[1]['settle_to_1deg_0_5deg_s']:.2f} s.",
        "",
        "The simulation target is a high-contrast point with an optional",
        "asymmetric three-dot orientation fiducial. The marker is disabled by",
        "default and rendered only inside an inner camera field to avoid clipping",
        "during edge acquisition. Point-only frames use IMU leveling. The physical",
        "poster needs an equivalent resolved orientation cue for visual roll.",
        "",
        "## Scope and reproduction",
        "",
        "The plant is decoupled and uses provisional inertia, damping, torque,",
        "and actuator values. This is deterministic software verification only.",
        "The ESP32 source now computes virtual pitch/roll torque for simulator",
        "frames; the plant accepts these only from the host SIL or simulator-framed",
        "ESP32. The physical motor output remains yaw-only. C compilation, ESP-IDF,",
        "Wokwi, sockets, browser behavior, and hardware were not exercised here.",
        "",
        "From `outputs/Python_sim/`, regenerate this report with:",
        "",
        "```bash",
        "python3 -m Verification.evaluate_axis_control > ../Verification/Axis_Control_Results.md",
        "```",
        "",
    ])
    return "\n".join(lines)


if __name__ == "__main__":
    print(render_report(), end="")
