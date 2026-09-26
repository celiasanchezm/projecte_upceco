# Kalman Filter Project - Code Skeleton

This folder contains the code skeleton for the Kalman filter project described in the Project Statement.

## What is already done for you

- `vehicle_model.py`: the kinematic bicycle model (the physics of "the real car"), with an RK4 integration step.
- `simulator.py`: builds a ground-truth manoeuvre and generates synthetic, noisy GPS (~60 Hz) and IMU (~100 Hz) measurements from it. This plays the role of the actual sensors.
- `plotting.py`: trajectory and per-state plots, plus a simple RMSE metric.
- `main.py`: orchestrates everything, including the IMU-rate prediction loop with asynchronous GPS updates.
- `config.py`: all the simulation and sensor parameters in one place (noise levels, rates, durations, initial state). Feel free to change these to experiment.

You should not need to modify any of the files above to complete the base project.

## What you need to implement

`kalman_filter.py` contains the `KalmanFilter` class with two methods, `predict()` and `update_gps()`, both currently raising `NotImplementedError`. This is the only file you need to write. Follow sections 1.2 and 1.3 of the Project Statement and use the Estimation training slides and script as your reference for the notation and equations.

## Running it

```
pip install -r requirements.txt
python main.py
```

With the filter not yet implemented, this will already generate and plot the ground-truth trajectory and the raw noisy sensor data, so you can check that everything runs and see what you are working with. Once you implement `predict()` and `update_gps()`, running it again will also plot your filter's estimate against ground truth and print its position RMSE.

## Bonus challenge (optional)

To try the GPS dropout scenario from section 1.4, set `gps_dropout_window` in `config.py`, for example:

```python
gps_dropout_window: Optional[Tuple[float, float]] = (8.0, 12.0)
```

and re-run `main.py`. During that window no GPS updates will arrive, so `predict()` alone (using the IMU) will have to keep the estimate on track until GPS measurements resume.

## A note on the state and inputs

The ground truth and your filter are not required to use the exact same state. A reasonable starting point is to estimate `[x, y, phi, v]`, using the IMU acceleration and yaw rate as the prediction input (`u = [accel, yaw_rate]`) and the GPS position `(x, y)` as the measurement. If that feels like too much at once, start from a simpler 1D or 2D case first (for example, position and velocity along a straight line) before adding heading, as suggested in the Project Statement.
