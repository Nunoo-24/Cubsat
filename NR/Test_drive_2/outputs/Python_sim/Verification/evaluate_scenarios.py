"""Measure the existing model-level CubeSat pointing scenarios."""

import math
import random
import statistics
import time

from CubeSat_ESP32.sil import OnboardSoftware
from CubeSat_ESP32.attitude import wrap_pi
from CubeSat_ESP32.camera import detect_black_target
from CubeSat_ESP32.control import YawController
from Sim_Environment.main import (CAMERA_FOCAL_PX, CAMERA_HALF_FOV_RAD,
                                  CAMERA_HEIGHT_PX, CAMERA_WIDTH_PX,
                                  Environment)
from Sim_Environment.model import ReactionWheel, YawPlant

DT_S = 0.02


def step(env, onboard):
    observation = env.observation()
    frame = env.camera_frame_snapshot(observation["sequence"])
    packet = onboard.process_observation(observation, camera_frame=frame)
    env.plant.step(body_wheel_torque_nm=packet["body_torque_nm"], dt_s=DT_S)
    return (abs(math.degrees(env.snapshot()["target_error_rad"])), packet)


def acquire_30_degrees():
    env = Environment()
    env.apply({"target_xyz_m": {"x": 1.154700538, "y": 0.0, "z": 2.0}})
    onboard = OnboardSoftware()
    for _ in range(300):
        error_deg, _ = step(env, onboard)
        if error_deg <= 2.0:
            return env.plant.time_s
    return None


def hold_for_one_minute():
    env = Environment()
    onboard = OnboardSoftware()
    peak_error = 0.0
    for _ in range(round(60.0 / DT_S)):
        error_deg, _ = step(env, onboard)
        peak_error = max(peak_error, error_deg)
    return peak_error, env.plant.time_s


def recover_from_nudge():
    env = Environment()
    env.plant.yaw_rad = math.radians(20.0)
    onboard = OnboardSoftware()
    for _ in range(round(4.0 / DT_S)):
        error_deg, _ = step(env, onboard)
        if error_deg <= 2.0:
            return env.plant.time_s
    return None


def track_slow_target():
    env = Environment()
    onboard = OnboardSoftware()
    peak_error = 0.0
    for index in range(round(60.0 / DT_S)):
        bearing = math.radians(0.2 * (index + 1) * DT_S)
        env.apply({"target_xyz_m": {
            "x": 2.0 * math.tan(bearing), "y": 0.0, "z": 2.0
        }})
        error_deg, packet = step(env, onboard)
        if not packet["target_valid"]:
            return None, env.plant.time_s
        peak_error = max(peak_error, error_deg)
    return peak_error, env.plant.time_s


def recover_after_occlusion():
    env = Environment()
    onboard = OnboardSoftware()
    env.plant.yaw_rad = math.radians(5.0)
    env.apply({"camera_occlusion_s": 2.0})
    peak_rate = 0.0
    occlusion_steps = round(2.0 / DT_S)
    for index in range(occlusion_steps - 1):
        _, packet = step(env, onboard)
        if packet["target_valid"]:
            raise RuntimeError("camera became valid during the configured occlusion")
        peak_rate = max(peak_rate, abs(env.plant.rate_rad_s))
    for index in range(round(4.0 / DT_S)):
        error_deg, packet = step(env, onboard)
        if packet["target_valid"] and error_deg <= 2.0:
            return env.plant.time_s - 2.0, peak_rate
    return None, peak_rate


def optimistic_wheel_speed_after_half_second():
    """Simulate maximum positive wheel torque for the guide's 0.5 s window.

    This is a bound for the current placeholder wheel model, not a model of the
    actual motor. It omits voltage/back-EMF and current/thermal derating, so it
    must not be interpreted as hardware performance.
    """
    wheel = ReactionWheel()
    dt_s = 0.001
    for _ in range(round(0.5 / dt_s)):
        # ReactionWheel.step accepts body torque and applies the opposite torque
        # to the motor. A negative body request therefore spins the motor up.
        wheel.step(-wheel.max_motor_torque_nm, dt_s)
    return wheel.speed_rad_s * 60.0 / (2.0 * math.pi)


def evaluate_wheel_speed_torque_bound():
    """Calculate ideal speed and inertia bounds for configured torque caps.

    Ignores lag, voltage/back-EMF, losses and the drive's velocity-loop
    response. The caps are firmware software settings, not motor ratings.
    """
    target_rpm = 3000.0
    duration_s = 0.5
    inertia_kg_m2 = ReactionWheel().inertia_kg_m2
    target_rate_rad_s = target_rpm * 2.0 * math.pi / 60.0
    torque_caps_nm = {"default": 0.001, "kconfig_max": 0.020}
    return {
        "model_inertia_kg_m2": inertia_kg_m2,
        "required_ideal_torque_nm": (
            inertia_kg_m2 * target_rate_rad_s / duration_s),
        "caps": {
            name: {
                "torque_nm": torque_nm,
                "ideal_speed_rpm": (torque_nm / inertia_kg_m2 * duration_s
                                     * 60.0 / (2.0 * math.pi)),
                "max_inertia_kg_m2": (torque_nm * duration_s /
                                      target_rate_rad_s),
            }
            for name, torque_nm in torque_caps_nm.items()
        },
    }


def evaluate_passive_roll_pitch_decay():
    """Check that the provisional damped roll/pitch modes lose energy.

    Starts the uncoupled passive axes at +10° roll and -8° pitch with no
    external torque, then measures the mechanical energy remaining after 20 s.
    This checks the implementation's qualitative damping behavior; these
    initial conditions and plant parameters are not hardware acceptance limits.
    """
    plant = YawPlant(
        roll_rad=math.radians(10.0),
        pitch_rad=math.radians(-8.0),
    )

    def axis_energy(inertia, angle, rate):
        return (0.5 * inertia * rate * rate
                + 0.5 * plant.pendulum_stiffness_nm_per_rad * angle * angle)

    initial_roll_energy = axis_energy(
        plant.roll_inertia_kg_m2, plant.roll_rad, plant.roll_rate_rad_s)
    initial_pitch_energy = axis_energy(
        plant.pitch_inertia_kg_m2, plant.pitch_rad, plant.pitch_rate_rad_s)
    initial_roll_rad = plant.roll_rad
    initial_pitch_rad = plant.pitch_rad
    for _ in range(round(20.0 / DT_S)):
        plant.step(dt_s=DT_S)

    roll_ratio = axis_energy(
        plant.roll_inertia_kg_m2, plant.roll_rad, plant.roll_rate_rad_s
    ) / initial_roll_energy
    pitch_ratio = axis_energy(
        plant.pitch_inertia_kg_m2, plant.pitch_rad, plant.pitch_rate_rad_s
    ) / initial_pitch_energy
    return {
        "duration_s": plant.time_s,
        "initial_roll_rad": initial_roll_rad,
        "initial_pitch_rad": initial_pitch_rad,
        "final_roll_rad": plant.roll_rad,
        "final_pitch_rad": plant.pitch_rad,
        "roll_energy_ratio": roll_ratio,
        "pitch_energy_ratio": pitch_ratio,
    }


def evaluate_yaw_parameter_corner_sweep():
    """Screen acquisition/nudge timing at 32 model-parameter corners.

    Inertia, passive yaw damping, wheel torque, wheel motor lag, and both
    controller gains are independently scaled by either 0.5 or 2.0 from their
    provisional defaults. Each run uses the direct yaw controller and plant,
    with perfect target-bearing and rate measurements; camera, sensor noise,
    attitude estimation, firmware scheduling, and hardware are excluded.
    """
    import itertools

    factors = (0.5, 2.0)
    worst_acquisition_s = 0.0
    worst_nudge_recovery_s = 0.0
    passing_corner_count = 0
    total_corner_count = 0

    for inertia_scale, damping_scale, torque_scale, lag_scale, gain_scale in (
            itertools.product(factors, repeat=5)):
        total_corner_count += 1

        def settle(initial_yaw_deg, duration_s):
            plant = YawPlant(
                inertia_kg_m2=0.02 * inertia_scale,
                damping_n_m_per_rad_s=0.002 * damping_scale,
                wheel=ReactionWheel(
                    max_motor_torque_nm=0.01 * torque_scale,
                    motor_time_constant_s=0.05 * lag_scale,
                ),
            )
            plant.yaw_rad = math.radians(initial_yaw_deg)
            controller = YawController(
                kp_nm_per_rad=0.08 * gain_scale,
                kd_nm_per_rad_s=0.04 * gain_scale,
            )
            for index in range(round(duration_s / DT_S)):
                error_rad = wrap_pi(-plant.yaw_rad)
                output = controller.update(
                    error_rad, plant.rate_rad_s, target_valid=True,
                    observation_age_s=0.0,
                )
                plant.step(body_wheel_torque_nm=output.body_torque_nm,
                           dt_s=DT_S)
                if abs(math.degrees(wrap_pi(plant.yaw_rad))) <= 2.0:
                    return (index + 1) * DT_S
            return None

        acquisition_s = settle(-30.0, 6.0)
        nudge_recovery_s = settle(20.0, 4.0)
        if acquisition_s is not None and nudge_recovery_s is not None:
            passing_corner_count += 1
            worst_acquisition_s = max(worst_acquisition_s, acquisition_s)
            worst_nudge_recovery_s = max(
                worst_nudge_recovery_s, nudge_recovery_s)

    return {
        "passing_corner_count": passing_corner_count,
        "total_corner_count": total_corner_count,
        "worst_acquisition_s": worst_acquisition_s,
        "worst_nudge_recovery_s": worst_nudge_recovery_s,
    }


def evaluate_sil_sensor_noise_latency_screen(kp_nm_per_rad=0.08,
                                             kd_nm_per_rad_s=0.04):
    """Characterize visual-yaw SIL response to observation noise and delay.

    This screen modifies camera error and gyro samples at the observation
    interface, then passes them through OnboardSoftware and the yaw plant. It
    is not image noise injection and its stress levels are not hardware
    measurements or project acceptance limits. Optional gains support
    sensitivity analysis; the defaults match the configured SIL controller.
    """
    if (not math.isfinite(kp_nm_per_rad) or kp_nm_per_rad < 0.0 or
            not math.isfinite(kd_nm_per_rad_s) or kd_nm_per_rad_s < 0.0):
        raise ValueError("controller gains must be finite and nonnegative")
    import itertools

    combinations = tuple(itertools.product(
        (0.0, 5.0, 10.0),       # camera bearing-error noise RMS, px
        (-0.03, 0.0, 0.03),     # gyro bias, rad/s
        (0.0, math.radians(0.05), 0.01),  # datasheet-typical and stress noise
        (0.0, 0.04, 0.08),     # matched camera/IMU sample delay, s
    ))
    results = []

    def simulate(initial_yaw_deg, duration_s, noise, seed):
        pixel_sigma, gyro_bias, gyro_sigma, delay_s = noise
        delay_steps = round(delay_s / DT_S)
        rng = random.Random(seed)
        plant = YawPlant()
        plant.yaw_rad = math.radians(initial_yaw_deg)
        onboard = OnboardSoftware(controller=YawController(
            kp_nm_per_rad=kp_nm_per_rad,
            kd_nm_per_rad_s=kd_nm_per_rad_s,
        ))
        history = []
        first_in_band_s = None
        peak_abs_yaw_deg = 0.0

        for index in range(round(duration_s / DT_S)):
            current_time_s = index * DT_S
            history.append((plant.yaw_rad, plant.rate_rad_s, current_time_s))
            sample_yaw, sample_rate, sample_time_s = history[
                max(0, len(history) - 1 - delay_steps)]
            camera_error_px = round(
                CAMERA_FOCAL_PX * math.tan(-sample_yaw) +
                rng.gauss(0.0, pixel_sigma))
            gyro_y = (sample_rate + gyro_bias +
                      rng.gauss(0.0, gyro_sigma))
            packet = {
                "protocol_version": 1,
                "message_type": "environment_observation",
                "sequence": index + 1,
                "simulation_time_s": sample_time_s,
                "sensors": {
                    "camera": {
                        "valid": True,
                        "pixel_error_x": camera_error_px,
                        "pixel_error_y": 0,
                        "focal_length_x_px": CAMERA_FOCAL_PX,
                        "age_s": delay_s,
                    },
                    "imu": {
                        "gyro_rad_s": {"x": 0.0, "y": gyro_y, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 9.80665, "z": 0.0},
                        "age_s": delay_s,
                    },
                    "magnetometer": {
                        "valid": False,
                        "magnetic_field_uT": {},
                        "age_s": 0.0,
                    },
                },
            }
            output = onboard.process_observation(packet, now=current_time_s)
            plant.step(body_wheel_torque_nm=output["body_torque_nm"],
                       dt_s=DT_S)
            abs_yaw_deg = abs(math.degrees(wrap_pi(plant.yaw_rad)))
            peak_abs_yaw_deg = max(peak_abs_yaw_deg, abs_yaw_deg)
            if first_in_band_s is None and abs_yaw_deg <= 2.0:
                first_in_band_s = (index + 1) * DT_S
        return first_in_band_s, peak_abs_yaw_deg

    for case_index, noise in enumerate(combinations):
        acquisition_s, _ = simulate(-30.0, 6.0, noise, 100 + case_index)
        nudge_recovery_s, _ = simulate(20.0, 4.0, noise, 200 + case_index)
        _, hold_peak_deg = simulate(0.0, 60.0, noise, 300 + case_index)
        results.append({
            "noise": noise,
            "acquisition_s": acquisition_s,
            "nudge_recovery_s": nudge_recovery_s,
            "hold_peak_deg": hold_peak_deg,
        })

    hold_failures = [row for row in results if row["hold_peak_deg"] > 2.0]
    return {
        "case_count": len(results),
        "acquisition_pass_count": sum(
            row["acquisition_s"] is not None and row["acquisition_s"] < 6.0
            for row in results),
        "nudge_pass_count": sum(
            row["nudge_recovery_s"] is not None and
            row["nudge_recovery_s"] < 4.0 for row in results),
        "hold_pass_count": len(results) - len(hold_failures),
        "worst_acquisition_s": max(
            row["acquisition_s"] or math.inf for row in results),
        "worst_nudge_recovery_s": max(
            row["nudge_recovery_s"] or math.inf for row in results),
        "worst_hold_peak_deg": max(row["hold_peak_deg"] for row in results),
        "hold_failure_cases": [
            {"pixel_noise_rms_px": row["noise"][0],
             "gyro_bias_rad_s": row["noise"][1],
             "gyro_noise_rms_rad_s": row["noise"][2],
             "delay_s": row["noise"][3],
             "hold_peak_deg": row["hold_peak_deg"]}
            for row in hold_failures],
    }


def evaluate_sil_hold_seed_sensitivity(kp_nm_per_rad=0.08,
                                       kd_nm_per_rad_s=0.04,
                                       seed_count=50):
    """Repeat the most stressful baseline hold input over independent seeds.

    This fixed corner uses 10 px camera-error noise, -0.03 rad/s gyro bias,
    0.01 rad/s RMS gyro noise, and 40 ms matched observation delay. It probes
    seed sensitivity for that one assumed case, not the full parameter space.
    """
    if (not math.isfinite(kp_nm_per_rad) or kp_nm_per_rad < 0.0 or
            not math.isfinite(kd_nm_per_rad_s) or kd_nm_per_rad_s < 0.0):
        raise ValueError("controller gains must be finite and nonnegative")
    if (isinstance(seed_count, bool) or not isinstance(seed_count, int) or
            seed_count < 1):
        raise ValueError("seed_count must be a positive integer")

    pixel_sigma = 10.0
    gyro_bias = -0.03
    gyro_sigma = 0.01
    delay_s = 0.04
    delay_steps = round(delay_s / DT_S)
    peaks = []
    for replication in range(seed_count):
        rng = random.Random(10000 + replication)
        plant = YawPlant()
        onboard = OnboardSoftware(controller=YawController(
            kp_nm_per_rad=kp_nm_per_rad,
            kd_nm_per_rad_s=kd_nm_per_rad_s,
        ))
        history = []
        peak_abs_yaw_deg = 0.0
        for index in range(round(60.0 / DT_S)):
            current_time_s = index * DT_S
            history.append((plant.yaw_rad, plant.rate_rad_s, current_time_s))
            sample_yaw, sample_rate, sample_time_s = history[
                max(0, len(history) - 1 - delay_steps)]
            camera_error_px = round(
                CAMERA_FOCAL_PX * math.tan(-sample_yaw) +
                rng.gauss(0.0, pixel_sigma))
            gyro_y = sample_rate + gyro_bias + rng.gauss(0.0, gyro_sigma)
            packet = {
                "protocol_version": 1,
                "message_type": "environment_observation",
                "sequence": index + 1,
                "simulation_time_s": sample_time_s,
                "sensors": {
                    "camera": {
                        "valid": True,
                        "pixel_error_x": camera_error_px,
                        "pixel_error_y": 0,
                        "focal_length_x_px": CAMERA_FOCAL_PX,
                        "age_s": delay_s,
                    },
                    "imu": {
                        "gyro_rad_s": {"x": 0.0, "y": gyro_y, "z": 0.0},
                        "accel_m_s2": {"x": 0.0, "y": 9.80665, "z": 0.0},
                        "age_s": delay_s,
                    },
                    "magnetometer": {
                        "valid": False,
                        "magnetic_field_uT": {},
                        "age_s": 0.0,
                    },
                },
            }
            output = onboard.process_observation(packet, now=current_time_s)
            plant.step(body_wheel_torque_nm=output["body_torque_nm"],
                       dt_s=DT_S)
            peak_abs_yaw_deg = max(
                peak_abs_yaw_deg,
                abs(math.degrees(wrap_pi(plant.yaw_rad))))
        peaks.append(peak_abs_yaw_deg)

    peaks.sort()
    p95_index = max(0, math.ceil(0.95 * seed_count) - 1)
    return {
        "seed_count": seed_count,
        "hold_pass_count": sum(peak <= 2.0 for peak in peaks),
        "worst_hold_peak_deg": peaks[-1],
        "p95_hold_peak_deg": peaks[p95_index],
        "median_hold_peak_deg": statistics.median(peaks),
        "hold_peak_deg_by_seed": peaks,
    }


def evaluate_yaw_attitude_estimator():
    """Measure yaw error/drift against simulator truth with synthetic sensors."""
    env = Environment()
    env.plant.yaw_rad = math.radians(15.0)
    onboard = OnboardSoftware()
    onboard.mode = "SAFE"
    initial_estimate = None
    peak_error_deg = 0.0
    for _ in range(round(60.0 / DT_S)):
        telemetry = onboard.process_observation(env.observation())
        estimate = telemetry["yaw_estimate_rad"]
        if estimate is None or not telemetry["yaw_estimate_valid"]:
            return None, None
        if initial_estimate is None:
            initial_estimate = estimate
        error = (estimate - env.plant.yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
        peak_error_deg = max(peak_error_deg, abs(math.degrees(error)))
        env.plant.step(body_wheel_torque_nm=0.0, dt_s=DT_S)
    drift = (estimate - initial_estimate + math.pi) % (2.0 * math.pi) - math.pi
    return peak_error_deg, abs(math.degrees(drift))


def evaluate_tilt_compensated_yaw_grid():
    """Check yaw fusion at several combined roll/pitch/yaw poses.

    The synthetic accelerometer and magnetic field are generated from known
    simulator truth and use the estimator's assumed reference. This catches
    rotation/sign inconsistencies in the software path, not sensor calibration
    error or real-world estimator accuracy.
    """
    poses_deg = (
        (-35.0, -25.0, -140.0),
        (-20.0, 30.0, -60.0),
        (0.0, 0.0, 0.0),
        (15.0, -20.0, 45.0),
        (35.0, 25.0, 140.0),
    )
    errors_deg = []
    for roll_deg, pitch_deg, yaw_deg in poses_deg:
        env = Environment()
        env.plant.roll_rad = math.radians(roll_deg)
        env.plant.pitch_rad = math.radians(pitch_deg)
        env.plant.yaw_rad = math.radians(yaw_deg)
        telemetry = OnboardSoftware().process_observation(env.observation())
        if not telemetry["yaw_estimate_valid"]:
            return None, len(errors_deg)
        error = wrap_pi(telemetry["yaw_estimate_rad"] - env.plant.yaw_rad)
        errors_deg.append(abs(math.degrees(error)))
    return max(errors_deg), len(errors_deg)


def evaluate_tilted_camera_projection_grid():
    """Compare synthetic camera pixels with a rigid-body rotation matrix."""
    poses_deg = (
        (-15.0, -10.0, -20.0),
        (-10.0, 15.0, -10.0),
        (0.0, 0.0, 0.0),
        (10.0, -15.0, 10.0),
        (15.0, 10.0, 20.0),
    )
    target_world = (0.2, 0.1, 2.0)

    def multiply(left, right):
        return [[sum(left[row][k] * right[k][col] for k in range(3))
                 for col in range(3)] for row in range(3)]

    max_pixel_error = 0
    for roll_deg, pitch_deg, yaw_deg in poses_deg:
        roll, pitch, yaw = map(
            math.radians, (roll_deg, pitch_deg, yaw_deg))
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotate_z = [[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]]
        rotate_x = [[1.0, 0.0, 0.0],
                    [0.0, cp, -sp], [0.0, sp, cp]]
        rotate_y = [[cy, 0.0, sy], [0.0, 1.0, 0.0],
                    [-sy, 0.0, cy]]
        body_to_world = multiply(rotate_y, multiply(rotate_x, rotate_z))
        camera_xyz = [
            sum(body_to_world[row][col] * target_world[row]
                for row in range(3))
            for col in range(3)
        ]
        expected_x_px = round(
            CAMERA_FOCAL_PX * camera_xyz[0] / camera_xyz[2])
        expected_y_px = round(
            CAMERA_FOCAL_PX * camera_xyz[1] / camera_xyz[2])
        env = Environment()
        env.plant.roll_rad, env.plant.pitch_rad, env.plant.yaw_rad = (
            roll, pitch, yaw)
        env.apply({"target_xyz_m": dict(zip(
            ("x", "y", "z"), target_world))})
        camera = env.observation()["sensors"]["camera"]
        if not camera["valid"]:
            return None, len(poses_deg)
        max_pixel_error = max(
            max_pixel_error,
            abs(camera["pixel_error_x"] - expected_x_px),
            abs(camera["pixel_error_y"] - expected_y_px),
        )
    return max_pixel_error, len(poses_deg)


def evaluate_synthetic_camera_detector_sweep():
    """Measure onboard detector availability/error across nominal yaw FOV.

    Uses the default 0.1 m marker at 2 m depth. The ideal scene is swept across
    13 horizontal bearings including the ±30° FOV boundary. This characterizes
    software geometry and area-filter compatibility only; it is not a real
    camera detection-rate or latency measurement.
    """
    bearings_deg = tuple(range(-30, 31, 5))
    valid_count = 0
    maximum_centroid_error_px = 0.0
    for bearing_deg in bearings_deg:
        depth_m = 2.0
        env = Environment()
        env.apply({"target_xyz_m": {
            "x": depth_m * math.tan(math.radians(bearing_deg)),
            "y": 0.0,
            "z": depth_m,
        }})
        observation = env.observation()
        camera = observation["sensors"]["camera"]
        frame = env.camera_frame_snapshot(observation["sequence"])
        detection = detect_black_target(
            frame, CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX)
        if not camera["valid"] or not detection["valid"]:
            continue
        valid_count += 1
        maximum_centroid_error_px = max(
            maximum_centroid_error_px,
            abs(detection["error_x_px"] - camera["pixel_error_x"]),
        )
    return valid_count, len(bearings_deg), maximum_centroid_error_px


def evaluate_full_direction_tilted_pointing():
    """Measure full-direction pointing in a tilted-gimbal case.

    A centered world target is observed from combined roll, pitch, and yaw
    offsets while brief X/Z torque pulses perturb the axes. Host SIL corrects
    horizontal and vertical image error and levels roll using gravity. This is
    a simulated controller result, not physical actuator evidence.
    """
    env = Environment()
    env.plant.roll_rad = math.radians(10.0)
    env.plant.pitch_rad = math.radians(-8.0)
    env.plant.yaw_rad = math.radians(20.0)
    env.apply({
        "external_torque_nm": {"x": 0.02, "y": 0.0, "z": 0.015},
        "duration_s": 0.2,
    })
    onboard = OnboardSoftware()
    target_valid_count = 0
    initial_horizontal_px = None
    final_packet = None
    for _ in range(round(6.0 / DT_S)):
        observation = env.observation()
        frame = env.camera_frame_snapshot(observation["sequence"])
        packet = onboard.process_observation(observation, camera_frame=frame)
        if packet["target_valid"]:
            target_valid_count += 1
            if initial_horizontal_px is None:
                initial_horizontal_px = packet["camera_error_x_px"]
        env.accept_actuator(packet)
        env.advance(DT_S, now=env.last_actuator_time)
        final_packet = packet
    if (initial_horizontal_px is None or final_packet is None or
            not final_packet["target_valid"]):
        return None
    return {
        "duration_s": round(6.0 / DT_S) * DT_S,
        "target_valid_count": target_valid_count,
        "sample_count": round(6.0 / DT_S),
        "initial_horizontal_px": initial_horizontal_px,
        "final_horizontal_px": final_packet["camera_error_x_px"],
        "final_vertical_px": final_packet["camera_error_y_px"],
        "horizontal_error_deg": math.degrees(math.atan(
            final_packet["camera_error_x_px"] / CAMERA_FOCAL_PX)),
        "vertical_error_deg": math.degrees(math.atan(
            final_packet["camera_error_y_px"] / CAMERA_FOCAL_PX)),
        "final_roll_deg": math.degrees(env.plant.roll_rad),
        "final_pitch_deg": math.degrees(env.plant.pitch_rad),
        "final_yaw_deg": math.degrees(env.plant.yaw_rad),
    }


def evaluate_yaw_step_45_degrees():
    """Run the onboard setpoint state against the model and report settling."""
    env = Environment()
    onboard = OnboardSoftware()
    onboard.mode = "SAFE"
    initial = onboard.process_observation(env.observation(), now=100.0)
    if not initial["yaw_estimate_valid"]:
        return None, None
    onboard.receive_command({
        "command_id": 1,
        "command": "SET_YAW_STEP",
        "arguments": {"delta_deg": 45.0},
    }, now=100.02)
    settle_time_s = None
    for index in range(1, round(10.0 / DT_S) + 1):
        now = 100.02 + index * DT_S
        telemetry = onboard.process_observation(env.observation(), now=now)
        env.plant.step(body_wheel_torque_nm=telemetry["body_torque_nm"],
                       dt_s=DT_S)
        if telemetry["yaw_step_status"] == "SETTLING" and settle_time_s is None:
            settle_time_s = telemetry["yaw_step_elapsed_s"]
        if telemetry["yaw_step_status"] == "COMPLETE":
            error = abs(math.degrees(wrap_pi(
                telemetry["yaw_step_target_rad"] - env.plant.yaw_rad)))
            if error <= 1.0:
                return settle_time_s, telemetry["yaw_step_elapsed_s"]
            return None, telemetry["yaw_step_elapsed_s"]
        if telemetry["yaw_step_status"] == "TIMEOUT":
            return None, telemetry["yaw_step_elapsed_s"]
    return None, None


def render_report():
    acquisition = acquire_30_degrees()
    hold_peak, hold_duration = hold_for_one_minute()
    nudge = recover_from_nudge()
    tracking_peak, tracking_duration = track_slow_target()
    occlusion_recovery, occlusion_peak_rate = recover_after_occlusion()
    speed_after_half_second = optimistic_wheel_speed_after_half_second()
    speed_torque_bound = evaluate_wheel_speed_torque_bound()
    passive_axes = evaluate_passive_roll_pitch_decay()
    attitude_peak_error_deg, attitude_drift_deg = evaluate_yaw_attitude_estimator()
    tilt_grid_peak_error_deg, tilt_grid_pose_count = (
        evaluate_tilt_compensated_yaw_grid())
    camera_grid_max_pixel_error, camera_grid_pose_count = (
        evaluate_tilted_camera_projection_grid())
    detector_valid_count, detector_sample_count, detector_max_error_px = (
        evaluate_synthetic_camera_detector_sweep())
    tilted_pointing = evaluate_full_direction_tilted_pointing()
    yaw_corner_sweep = evaluate_yaw_parameter_corner_sweep()
    noise_latency_screen = evaluate_sil_sensor_noise_latency_screen()
    gain_candidates = [(0.10, 0.04), (0.10, 0.05), (0.12, 0.04),
                       (0.12, 0.05), (0.14, 0.05), (0.16, 0.06),
                       (0.18, 0.06), (0.20, 0.07)]
    gain_sensitivity = [(kp, kd, evaluate_sil_sensor_noise_latency_screen(
        kp_nm_per_rad=kp, kd_nm_per_rad_s=kd))
        for kp, kd in gain_candidates]
    baseline_seed_sensitivity = evaluate_sil_hold_seed_sensitivity(
        kp_nm_per_rad=0.08, kd_nm_per_rad_s=0.04, seed_count=50)
    candidate_seed_sensitivity = evaluate_sil_hold_seed_sensitivity(
        kp_nm_per_rad=0.14, kd_nm_per_rad_s=0.05, seed_count=50)
    hold_failure_summary = "; ".join(
        f"{case['pixel_noise_rms_px']:.0f} px RMS camera-error noise, "
        f"{case['gyro_bias_rad_s']:.3f} rad/s gyro bias, "
        f"{case['gyro_noise_rms_rad_s']:.6f} rad/s RMS gyro noise, "
        f"{case['delay_s'] * 1000:.0f} ms delay "
        f"({case['hold_peak_deg']:.3f}° peak)"
        for case in noise_latency_screen["hold_failure_cases"])
    yaw_step_settle_s, yaw_step_completion_s = evaluate_yaw_step_45_degrees()
    required_average_torque_nm = (
        ReactionWheel().inertia_kg_m2 * (3000.0 * 2.0 * math.pi / 60.0) / 0.5
    )
    checks = [
        ("30° initial error to ≤2°", "< 6 s", acquisition,
         acquisition is not None and acquisition < 6.0, "s"),
        ("Pointing hold", "≤2° for 60 s", hold_peak,
         hold_duration >= 59.99 and hold_peak <= 2.0, "° peak error"),
        ("20° nudge recovery to ≤2°", "< 4 s", nudge,
         nudge is not None and nudge < 4.0, "s"),
        ("Moving target at 0.2°/s", "≤2° for 60 s", tracking_peak,
         tracking_duration >= 59.99 and tracking_peak is not None and tracking_peak <= 2.0,
         "° peak error"),
        ("Recovery after 2 s camera occlusion", "≤2° within 4 s",
         occlusion_recovery,
         occlusion_recovery is not None and occlusion_recovery < 4.0, "s"),
        ("Yaw estimate with synthetic magnetic reference",
         "<2° error and drift per minute", attitude_peak_error_deg,
         attitude_peak_error_deg is not None and attitude_peak_error_deg < 2.0
         and attitude_drift_deg is not None and attitude_drift_deg < 2.0,
         "° peak error"),
        ("Tilt-compensated yaw across combined attitudes",
         "≤0.000001° over 5 idealized poses", tilt_grid_peak_error_deg,
         tilt_grid_peak_error_deg is not None and
         tilt_grid_peak_error_deg <= 0.000001 and tilt_grid_pose_count == 5,
         "° peak error"),
        ("Camera projection across combined attitudes",
         "matches rigid-body matrix at 5 idealized poses",
         camera_grid_max_pixel_error,
         camera_grid_max_pixel_error == 0 and camera_grid_pose_count == 5,
         "px maximum X/Y discrepancy"),
        ("Synthetic onboard detector across nominal yaw FOV",
         "valid at all 13 bearings, ±30° in 5° steps",
         f"{detector_valid_count}/{detector_sample_count} valid",
         detector_valid_count == detector_sample_count,
         ""),
        ("45° commanded yaw step", "settle within ±1°",
         yaw_step_settle_s,
         yaw_step_settle_s is not None and yaw_step_completion_s is not None,
         "s to 1 s settled dwell"),
        ("Passive roll/pitch free decay", "energy decreases under damping",
         max(passive_axes["roll_energy_ratio"],
             passive_axes["pitch_energy_ratio"]) * 100.0,
         passive_axes["roll_energy_ratio"] < 1.0 and
         passive_axes["pitch_energy_ratio"] < 1.0,
         "% maximum energy retained after 20 s"),
        ("Yaw parameter corner sensitivity",
         "acquire <6 s and nudge recovery <4 s at all 32 corners",
         f"{yaw_corner_sweep['passing_corner_count']}/"
         f"{yaw_corner_sweep['total_corner_count']} pass",
         yaw_corner_sweep["passing_corner_count"] ==
         yaw_corner_sweep["total_corner_count"], ""),
    ]
    lines = [
        "# Model-level dynamics, pointing, and sensor evaluation",
        "",
        f"Generated: {time.strftime('%Y-%m-%d %H:%M UTC', time.gmtime())}",
        "",
        "This report measures deterministic software-in-the-loop pointing,",
        "one-axis yaw-estimator, and passive-axis free-decay scenarios using",
        "the Python plant and onboard reference. Closed-loop pointing scenarios",
        "render a sequence-paired QVGA frame and run the host centroid detector",
        "before control. Guide performance limits are proposed in the course",
        "project guide and summarized in",
        "[`PROJECT_BRIEF.md`](../../Context/PROJECT_BRIEF.md); the passive-axis",
        "free-decay row is a qualitative model check, not an acceptance target.",
        "Passing these cases",
        "does not verify ESP32 firmware, live HTTP, camera/IMU hardware, motor",
        "behavior, or the physical rig. Plant constants and camera/noise models",
        "remain provisional.",
        "",
        "| Scenario | Criterion / model check | Model result | Outcome |",
        "|---|---:|---:|---|",
    ]
    for name, target, value, passed, unit in checks:
        result = ("not reached" if value is None else
                  value if isinstance(value, str) else f"{value:.3f} {unit}")
        lines.append(f"| {name} | {target} | {result} | {'PASS' if passed else 'FAIL'} |")
    lines += [
        "",
        f"Peak yaw rate during the camera-occlusion interval: {occlusion_peak_rate:.4f} rad/s.",
        (f"Yaw-estimator final drift over 60 s: {attitude_drift_deg:.4f}°; "
         f"peak absolute error: {attitude_peak_error_deg:.4f}°."
         if attitude_peak_error_deg is not None and attitude_drift_deg is not None
         else "Yaw-estimator scenario did not produce a valid estimate."),
        (f"Tilt-compensated synthetic yaw grid: {tilt_grid_pose_count} combined "
         f"roll/pitch/yaw poses, {tilt_grid_peak_error_deg:.9f}° maximum error. "
         "The field and gravity vectors are ideal simulator references, so this "
         "checks coordinate consistency only." if tilt_grid_peak_error_deg is not None
         else "Tilt-compensated yaw grid produced an invalid estimate."),
        (f"Synthetic camera projection grid: {camera_grid_pose_count} combined "
         f"attitude poses, {camera_grid_max_pixel_error} px maximum X/Y "
         "discrepancy against an independently assembled rotation matrix. "
         "This checks geometry/sign consistency, not camera calibration."
         if camera_grid_max_pixel_error is not None
         else "Synthetic camera projection grid contained an invalid observation."),
        (f"Synthetic detector sweep: {detector_valid_count}/"
         f"{detector_sample_count} targets valid at depth 2 m and 0.1 m scale; "
         f"maximum centroid discrepancy is {detector_max_error_px:.2f} px. "
         "The discrepancy is caused by rasterization/clipping at the FOV edge. "
         "This checks ideal renderer/detector compatibility only, not detection "
         "rate or latency on camera hardware."),
        (f"Full-direction tilted pointing diagnostic (6 s): horizontal camera error "
         f"{tilted_pointing['horizontal_error_deg']:.2f}° / "
         f"{tilted_pointing['final_horizontal_px']:.1f} px, vertical error "
         f"{tilted_pointing['vertical_error_deg']:.2f}° / "
         f"{tilted_pointing['final_vertical_px']:.1f} px; "
         f"{tilted_pointing['target_valid_count']}/"
         f"{tilted_pointing['sample_count']} target samples valid. Host SIL "
         "commands virtual pitch/roll torques; these are not physical actuator "
         "or firmware results."
         if tilted_pointing is not None
        else "Full-direction tilted pointing diagnostic lost the target at the final sample."),
        (f"Yaw parameter corner sweep (direct control-law/plant, perfect "
         f"measurements): {yaw_corner_sweep['passing_corner_count']}/"
         f"{yaw_corner_sweep['total_corner_count']} corners meet both guide "
         f"timing targets; worst acquisition "
         f"{yaw_corner_sweep['worst_acquisition_s']:.2f} s and nudge recovery "
         f"{yaw_corner_sweep['worst_nudge_recovery_s']:.2f} s. Inertia, yaw "
         "damping, wheel torque, motor lag, and paired controller gains were "
         "independently set to 0.5× or 2× nominal. This is a parameter screen "
         "for the simplified yaw model, not end-to-end SIL robustness."),
        (f"Observation noise/latency screen: {noise_latency_screen['case_count']} "
         f"cases; acquire {noise_latency_screen['acquisition_pass_count']}/"
         f"{noise_latency_screen['case_count']}, nudge "
         f"{noise_latency_screen['nudge_pass_count']}/"
         f"{noise_latency_screen['case_count']}, hold "
         f"{noise_latency_screen['hold_pass_count']}/"
         f"{noise_latency_screen['case_count']} within 2°. Worst measured "
         f"peak yaw is {noise_latency_screen['worst_hold_peak_deg']:.3f}°. "
         + (f"Hold miss inputs: {hold_failure_summary}. "
            if hold_failure_summary else
            "Every hold case remains within 2°. ")
         + "The stress levels are observation-output noise and matched sample "
         "delays, not calibrated camera/IMU properties."),
        ("Gain sensitivity candidates (Kp/Kd in N·m/rad and N·m·s/rad; "
         "each reports acquire/nudge/hold and worst hold peak): " + "; ".join(
             f"{kp:.2f}/{kd:.2f}={result['acquisition_pass_count']}/"
             f"{result['case_count']},{result['nudge_pass_count']}/"
             f"{result['case_count']},{result['hold_pass_count']}/"
             f"{result['case_count']}; {result['worst_hold_peak_deg']:.3f}°"
             for kp, kd, result in gain_sensitivity)
         + ". These are simulation-only candidates; configured gains remain "
           "unchanged pending measured plant and sensor/link data."),
        ("Critical-corner hold seed sensitivity (50 independent noise seeds; "
         "10 px camera noise, -0.03 rad/s gyro bias, 0.01 rad/s gyro noise, "
         "40 ms delay): baseline Kp/Kd=0.08/0.04 "
         f"{baseline_seed_sensitivity['hold_pass_count']}/50 pass, median "
         f"{baseline_seed_sensitivity['median_hold_peak_deg']:.3f}°, p95 "
         f"{baseline_seed_sensitivity['p95_hold_peak_deg']:.3f}°, worst "
         f"{baseline_seed_sensitivity['worst_hold_peak_deg']:.3f}°; candidate "
         f"0.14/0.05 {candidate_seed_sensitivity['hold_pass_count']}/50 pass, "
         f"median {candidate_seed_sensitivity['median_hold_peak_deg']:.3f}°, "
         f"p95 {candidate_seed_sensitivity['p95_hold_peak_deg']:.3f}°, worst "
         f"{candidate_seed_sensitivity['worst_hold_peak_deg']:.3f}°. The "
         "candidate improves the median but worsens the upper tail; 50 seeds "
         "at one assumed corner are not hardware tuning evidence."),
        (f"Passive-axis free-decay check (no external torque, 20 s): roll "
         f"{math.degrees(passive_axes['initial_roll_rad']):.1f}° → "
         f"{math.degrees(passive_axes['final_roll_rad']):.2f}° "
         f"({passive_axes['roll_energy_ratio'] * 100.0:.1f}% energy retained); "
         f"pitch {math.degrees(passive_axes['initial_pitch_rad']):.1f}° → "
         f"{math.degrees(passive_axes['final_pitch_rad']):.2f}° "
         f"({passive_axes['pitch_energy_ratio'] * 100.0:.1f}% energy retained). "
         "This checks only qualitative damping under provisional parameters."),
        "",
        "## Earlier guide milestones and current evidence limits",
        "",
        "The project brief also records earlier staged targets. They are kept",
        "visible here because they remain unimplemented or lack hardware",
        "calibration. A separate best-case spin-up bound is included for the",
        "current placeholder wheel model; it is not a motor-performance claim:",
        "",
        "| Guide milestone | Status | Current evidence gap |",
        "|---|---|---|",
        f"| 3000 rpm in <0.5 s with <2% error | COMMAND PATH IMPLEMENTED; PERFORMANCE NOT VERIFIED; PLACEHOLDER MODEL LIMIT {speed_after_half_second:.0f} rpm | The Ground Station → bridge → onboard software contract and firmware velocity API call now exist, with SAFE/STANDBY gating and timed stop logic. The Python plant reaches only {speed_after_half_second:.0f} rpm after 0.5 s under its provisional 1e-4 kg·m² inertia, 0.01 N·m torque limit and 0.05 s torque lag. At that inertia, ideal constant torque requires {required_average_torque_nm * 1000:.1f} mN·m before losses. Source review shows the physical velocity frame uses the shared moteus torque setting (1 mN·m default, 20 mN·m Kconfig maximum); ideal no-loss speed at the 20 mN·m cap is only {speed_torque_bound['caps']['kconfig_max']['ideal_speed_rpm']:.0f} rpm for this inertia. Reaching 3000 rpm in 0.5 s at the maximum cap requires total wheel inertia at or below {speed_torque_bound['caps']['kconfig_max']['max_inertia_kg_m2'] * 1e6:.1f}×10⁻⁶ kg·m²; the default cap requires {speed_torque_bound['caps']['default']['max_inertia_kg_m2'] * 1e6:.2f}×10⁻⁶ kg·m². These ideal torque/inertia bounds are not motor ratings or measured performance. No exact motor/wheel parameters, firmware build, or physical speed-loop result is available. |",
        (f"| Yaw estimate error <2° and drift <2°/min | MODEL RESULT {attitude_peak_error_deg:.3f}° peak error / {attitude_drift_deg:.3f}° drift | The one-axis Python estimator is evaluated against simulator truth using synthetic gyro bias/noise and a placeholder fixed magnetic field. This does not assess roll/pitch attitude, magnetic calibration/disturbance, firmware, or hardware. |"
         if attitude_peak_error_deg is not None and attitude_drift_deg is not None
         else "| Yaw estimate error <2° and drift <2°/min | NOT EVALUATED | The estimator did not produce a valid truth-referenced result. |"),
        (f"| 45° angular step settles within ±1° | MODEL RESULT: first entered ±1° and ≤0.5°/s at {yaw_step_settle_s:.3f} s; completed 1 s dwell at {yaw_step_completion_s:.3f} s | The step uses the onboard yaw estimate and bounded body-torque controller, independent of camera visibility. This is a synthetic-model result only; controller gains and estimate are not hardware-calibrated. |"
         if yaw_step_settle_s is not None and yaw_step_completion_s is not None
         else f"| 45° angular step settles within ±1° | NOT SETTLED | The model did not complete the commanded step within its 10 s timeout. Camera half-FOV is ±{math.degrees(CAMERA_HALF_FOV_RAD):.1f}°, but the setpoint path does not depend on camera visibility. |"),
        "",
        "## Reproduce",
        "",
        "From `outputs/Python_sim/`, run:",
        "",
        "```bash",
        "python3 -m Verification.evaluate_scenarios > ../Verification/Simulation_Results.md",
        "```",
        "",
        "The unit suite independently asserts pointing behavior, synthetic-field and tilted-pose yaw estimation, camera projection/detection geometry, passive-axis energy decay, parameter-corner timing, observation-noise/latency sensitivity, and the placeholder wheel spin-up bound.",
    ]
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    report = render_report()
    print(report, end="")
    if "| FAIL |" in report:
        raise SystemExit(1)
