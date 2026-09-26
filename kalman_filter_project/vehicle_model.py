"""
Kinematic bicycle model: the physics of "the real car" in this exercise.

State:  [x, y, phi, v]   (position, heading, speed)
Input:  [a, delta]       (acceleration, steering angle)

    x_dot   = v * cos(phi)
    y_dot   = v * sin(phi)
    phi_dot = (v / L) * tan(delta)
    v_dot   = a

This file is complete and given to you as-is. It is only used by
simulator.py to build the ground-truth trajectory, so you do not need to
call it directly, but it is worth reading since your filter's own state
should describe the same vehicle.
"""
import numpy as np


def dynamics(state, u, wheelbase):
    """Continuous-time derivative of the state, given the current state
    and input (a, delta)."""
    x, y, phi, v = state
    a, delta = u

    x_dot = v * np.cos(phi)
    y_dot = v * np.sin(phi)
    phi_dot = (v / wheelbase) * np.tan(delta)
    v_dot = a

    return np.array([x_dot, y_dot, phi_dot, v_dot])


def rk4_step(state, u, dt, wheelbase):
    """Advance the state by one step of size dt using 4th-order
    Runge-Kutta integration of the dynamics above."""
    state = np.asarray(state, dtype=float)

    k1 = dynamics(state, u, wheelbase)
    k2 = dynamics(state + dt / 2 * k1, u, wheelbase)
    k3 = dynamics(state + dt / 2 * k2, u, wheelbase)
    k4 = dynamics(state + dt * k3, u, wheelbase)

    return state + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
