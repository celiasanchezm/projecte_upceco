"""
Simulation and sensor configuration for the Kalman filter project.

Change the values here to experiment with different noise levels, sensor
rates, or the GPS dropout window for the bonus challenge. You should not
need to touch any other file to run the simulator and look at the raw
(ground truth + noisy sensor) data.
"""
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class SimulationConfig:
    # Vehicle
    wheelbase: float = 1.55          # meters, distance between front and rear axles

    # Time
    dt: float = 0.01                 # simulation step (s), also the IMU period (1 / 100 Hz)
    duration: float = 20.0           # total simulated time (s)

    # Initial state: [x (m), y (m), heading phi (rad), speed v (m/s)]
    initial_state: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    # Sensor rates
    gps_rate: float = 60.0           # Hz
    imu_rate: float = 100.0          # Hz (kept equal to 1 / dt)

    # Sensor noise (standard deviations)
    gps_pos_std: float = 0.5         # meters, applied to both x and y
    imu_accel_std: float = 0.2       # m/s^2
    imu_yaw_rate_std: float = 0.02   # rad/s

    # Optional GPS dropout window (start_time, end_time) in seconds, for the
    # bonus challenge in section 1.4 of the project statement. Leave as
    # None for the base exercise.
    gps_dropout_window: Optional[Tuple[float, float]] = None

    random_seed: int = 42
