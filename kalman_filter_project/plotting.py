"""
Plotting and error-metric helpers. This file is complete and given to you
as-is, so you can call it directly from main.py once your filter produces
estimates.
"""
import numpy as np
import matplotlib.pyplot as plt


def plot_trajectory(ground_truth, sensors, estimates=None, save_path=None):
    """Plot the x-y trajectory: ground truth, raw GPS measurements, and
    (optionally) the filter's estimated trajectory."""
    states = ground_truth["states"]

    plt.figure(figsize=(7, 6))
    plt.plot(states[:, 0], states[:, 1], label="Ground truth", color="black", linewidth=2)
    plt.scatter(sensors["gps"]["x"], sensors["gps"]["y"], s=10, color="tab:red",
                alpha=0.5, label="Raw GPS")
    if estimates is not None:
        plt.plot(estimates[:, 0], estimates[:, 1], label="KF estimate",
                  color="tab:blue", linewidth=2)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Vehicle trajectory")
    plt.axis("equal")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
    plt.show()


def plot_states_over_time(ground_truth, estimates=None, estimate_t=None, save_path=None):
    """Plot each state (x, y, phi, v) against time, ground truth vs estimate."""
    t = ground_truth["t"]
    states = ground_truth["states"]
    labels = ["x (m)", "y (m)", "phi (rad)", "v (m/s)"]

    fig, axes = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t, states[:, i], label="Ground truth", color="black")
        if estimates is not None:
            et = estimate_t if estimate_t is not None else t
            ax.plot(et, estimates[:, i], label="KF estimate", color="tab:blue")
        ax.set_ylabel(labels[i])
        ax.grid(True, alpha=0.3)
    axes[0].legend()
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("State estimate vs ground truth")
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
    plt.show()


def compute_rmse(ground_truth, estimates, estimate_t=None):
    """Return the RMSE of the position estimate (x, y), interpolated onto
    the ground-truth time base if needed."""
    t = ground_truth["t"]
    states = ground_truth["states"]

    if estimate_t is not None and len(estimate_t) != len(t):
        est_x = np.interp(t, estimate_t, estimates[:, 0])
        est_y = np.interp(t, estimate_t, estimates[:, 1])
    else:
        est_x = estimates[:, 0]
        est_y = estimates[:, 1]

    error_x = est_x - states[:, 0]
    error_y = est_y - states[:, 1]
    rmse_pos = np.sqrt(np.mean(error_x ** 2 + error_y ** 2))
    return {"rmse_position": rmse_pos}
