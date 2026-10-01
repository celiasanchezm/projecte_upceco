"""
Entry point: builds the simulation, generates sensor data, runs the
Kalman filter loop, and plots the results.

Run it with the filter still unimplemented and you will already see the
ground-truth trajectory and the raw noisy sensors, so you can check that
everything runs and get a feel for the data. Once you implement predict()
and update_gps() in kalman_filter.py, running it again will also plot
your filter's estimate against ground truth and print its position RMSE.
"""
import numpy as np

from config import SimulationConfig
from simulator import generate_ground_truth, generate_sensor_data
from kalman_filter import KalmanFilter
from plotting import plot_trajectory, plot_states_over_time, compute_rmse


def run_filter(ground_truth, sensors, config):
    """Run the filter over the IMU-rate loop, applying a GPS update
    whenever a GPS measurement has arrived since the last tick. Returns
    (estimate_t, estimates)."""

    n = len(ground_truth["t"])
    estimates = np.zeros((n, 4))

    # Placeholder initial guess, deliberately not the true initial state,
    # so you can see the filter converge.
    x0 = [0.0, 0.0, 0.0, 0.0]
    P0 = np.diag([1.0, 1.0, 0.1, 1.0])
    Q = np.diag([0.0001, 0.0001, 0.05, 0.2])                              # tune these
    R = np.diag([config.gps_pos_std ** 2, config.gps_pos_std ** 2])  # starting point

    kf = KalmanFilter(x0, P0, Q, R, config.wheelbase)

    gps_t = sensors["gps"]["t"]
    gps_x = sensors["gps"]["x"]
    gps_y = sensors["gps"]["y"]
    gps_index = 0

    imu_t = sensors["imu"]["t"]
    imu_accel = sensors["imu"]["accel"]
    imu_yaw_rate = sensors["imu"]["yaw_rate"]

    for i, t in enumerate(imu_t):
        dt = config.dt
        u = (imu_accel[i], imu_yaw_rate[i])

        kf.predict(u, dt)

        while gps_index < len(gps_t) and gps_t[gps_index] <= t:
            kf.update_gps((gps_x[gps_index], gps_y[gps_index]))
            gps_index += 1

        estimates[i] = kf.x

    return imu_t, estimates


def main():
    config = SimulationConfig()

    ground_truth = generate_ground_truth(config)
    sensors = generate_sensor_data(ground_truth, config)

    print("Ground truth and sensor data generated.")
    print(f"  {len(ground_truth['t'])} ground-truth samples")
    print(f"  {len(sensors['gps']['t'])} GPS samples")
    print(f"  {len(sensors['imu']['t'])} IMU samples")

    plot_trajectory(ground_truth, sensors)

    try:
        estimate_t, estimates = run_filter(ground_truth, sensors, config)
    except NotImplementedError as exc:
        print("\nKalman filter not implemented yet:")
        print(f"  {exc}")
        print("\nComplete predict() and update_gps() in kalman_filter.py, then re-run this script.")
        return

    plot_trajectory(ground_truth, sensors, estimates=estimates)
    plot_states_over_time(ground_truth, estimates=estimates, estimate_t=estimate_t)

    metrics = compute_rmse(ground_truth, estimates, estimate_t=estimate_t)
    print(f"\nPosition RMSE: {metrics['rmse_position']:.3f} m")


if __name__ == "__main__":
    main()
