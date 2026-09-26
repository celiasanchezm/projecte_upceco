"""
Ground truth trajectory and synthetic sensor data generation.

This file is complete and given to you as-is: it plays the role of "the
real car" and its sensors in this exercise. You should not need to modify
it, but read generate_ground_truth and generate_sensor_data to understand
exactly how the data you will feed into your filter is produced, since
that is what your filter's state and measurement models must match.
"""
import numpy as np

from vehicle_model import rk4_step


def control_input(t: float):
    """Return the (acceleration, steering angle) applied at time t.

    Defines a simple manoeuvre: accelerate in a straight line, cruise,
    steer through a turn, then cruise again in a straight line.
    """
    if t < 3.0:
        return 1.5, 0.0          # accelerate, m/s^2
    elif t < 8.0:
        return 0.0, 0.0          # cruise
    elif t < 13.0:
        return 0.0, 0.25         # turn, steering angle in radians
    else:
        return 0.0, 0.0          # cruise


def generate_ground_truth(config):
    """Simulate the true vehicle trajectory with the kinematic bicycle model.

    Returns a dict with:
        t        : (N,) time stamps, spaced by config.dt
        states   : (N, 4) array of [x, y, phi, v] at each time stamp
        controls : (N, 2) array of [a, delta] applied at each time stamp
    """
    n_steps = int(round(config.duration / config.dt)) + 1
    t = np.arange(n_steps) * config.dt

    states = np.zeros((n_steps, 4))
    controls = np.zeros((n_steps, 2))
    states[0] = config.initial_state

    for i in range(n_steps):
        a, delta = control_input(t[i])
        controls[i] = [a, delta]
        if i < n_steps - 1:
            states[i + 1] = rk4_step(states[i], (a, delta), config.dt, config.wheelbase)

    return {"t": t, "states": states, "controls": controls}


def generate_sensor_data(ground_truth, config):
    """Synthesize noisy GPS and IMU measurements from the ground truth.

    Returns a dict with two entries, "gps" and "imu":
        gps: {"t", "x", "y"}                  position, at ~gps_rate
        imu: {"t", "accel", "yaw_rate"}        at ~imu_rate (= 1 / dt)
    """
    rng = np.random.default_rng(config.random_seed)

    t = ground_truth["t"]
    states = ground_truth["states"]
    controls = ground_truth["controls"]

    # --- GPS: position, at gps_rate, interpolated from the dense ground truth ---
    gps_dt = 1.0 / config.gps_rate
    gps_t = np.arange(t[0], t[-1], gps_dt)

    if config.gps_dropout_window is not None:
        start, end = config.gps_dropout_window
        gps_t = gps_t[(gps_t < start) | (gps_t > end)]

    gps_x_true = np.interp(gps_t, t, states[:, 0])
    gps_y_true = np.interp(gps_t, t, states[:, 1])

    gps_x = gps_x_true + rng.normal(0.0, config.gps_pos_std, size=gps_t.shape)
    gps_y = gps_y_true + rng.normal(0.0, config.gps_pos_std, size=gps_t.shape)

    # --- IMU: acceleration and yaw rate, at imu_rate (matches config.dt) ---
    # Computed analytically from the true control and state, the way a real
    # accelerometer/gyro would report them, rather than by differentiating
    # the ground-truth trajectory numerically.
    imu_t = t.copy()
    true_accel = controls[:, 0]
    true_yaw_rate = states[:, 3] / config.wheelbase * np.tan(controls[:, 1])

    imu_accel = true_accel + rng.normal(0.0, config.imu_accel_std, size=imu_t.shape)
    imu_yaw_rate = true_yaw_rate + rng.normal(0.0, config.imu_yaw_rate_std, size=imu_t.shape)

    return {
        "gps": {"t": gps_t, "x": gps_x, "y": gps_y},
        "imu": {"t": imu_t, "accel": imu_accel, "yaw_rate": imu_yaw_rate},
    }
