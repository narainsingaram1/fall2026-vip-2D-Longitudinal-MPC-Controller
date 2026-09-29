"""
Time-Series and Performance Dashboard Plotting Utilities
"""
import os
import matplotlib.pyplot as plt
from typing import List
from config import STOP_LINE, INTERSECTION_EXIT, V_DES


def plot_simulation_results(vehicles: List, output_dir: str = "."):
    """
    Generate professional publication-quality figures:
    1. Position vs Time
    2. Speed vs Time
    3. Acceleration vs Time
    4. Combined 3-Panel State Dashboard
    """
    os.makedirs(output_dir, exist_ok=True)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

    # 1. Position Plot
    plt.figure(figsize=(9, 5))
    for i, v in enumerate(vehicles):
        plt.plot(
            v.time_history,
            v.position_history,
            label=f"Vehicle {v.id} ({v.approach}) - Priority {v.priority}",
            color=colors[i % len(colors)],
            linewidth=2.0
        )
    plt.axhline(STOP_LINE, linestyle="--", color="black", linewidth=1.2, label="Stop line (0 m)")
    plt.axhline(INTERSECTION_EXIT, linestyle=":", color="red", linewidth=1.2, label="Intersection exit (12 m)")
    plt.xlabel("Time [s]", fontsize=11)
    plt.ylabel("Longitudinal Position [m]", fontsize=11)
    plt.title("Vehicle Longitudinal Position vs. Time (FIFO MPC)", fontsize=13, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="upper left")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "position_vs_time.png"), dpi=200)
    plt.close()

    # 2. Speed Plot
    plt.figure(figsize=(9, 5))
    for i, v in enumerate(vehicles):
        plt.plot(
            v.time_history,
            v.speed_history,
            label=f"Vehicle {v.id} ({v.approach})",
            color=colors[i % len(colors)],
            linewidth=2.0
        )
    plt.axhline(V_DES, linestyle="--", color="black", linewidth=1.2, label=f"Desired speed ({V_DES} m/s)")
    plt.xlabel("Time [s]", fontsize=11)
    plt.ylabel("Velocity [m/s]", fontsize=11)
    plt.title("Vehicle Velocity vs. Time (FIFO MPC)", fontsize=13, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="upper left")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "speed_vs_time.png"), dpi=200)
    plt.close()

    # 3. Acceleration Plot
    plt.figure(figsize=(9, 5))
    for i, v in enumerate(vehicles):
        plt.plot(
            v.time_history,
            v.acceleration_history,
            label=f"Vehicle {v.id} ({v.approach})",
            color=colors[i % len(colors)],
            linewidth=2.0
        )
    plt.axhline(0.0, linestyle="-", color="gray", linewidth=0.8)
    plt.xlabel("Time [s]", fontsize=11)
    plt.ylabel(r"Acceleration [$m/s^2$]", fontsize=11)
    plt.title("Control Acceleration Input vs. Time", fontsize=13, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="lower right")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "acceleration_vs_time.png"), dpi=200)
    plt.close()

    # 4. Multi-Panel Dashboard
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    for i, v in enumerate(vehicles):
        c = colors[i % len(colors)]
        label = f"Veh {v.id} ({v.approach})"
        axes[0].plot(v.time_history, v.position_history, label=label, color=c, linewidth=1.8)
        axes[1].plot(v.time_history, v.speed_history, label=label, color=c, linewidth=1.8)
        axes[2].plot(v.time_history, v.acceleration_history, label=label, color=c, linewidth=1.8)

    axes[0].axhline(STOP_LINE, linestyle="--", color="black", alpha=0.7, label="Stop line")
    axes[0].axhline(INTERSECTION_EXIT, linestyle=":", color="red", alpha=0.7, label="Exit")
    axes[0].set_ylabel("Position [m]", fontweight="bold")
    axes[0].grid(True, linestyle="--", alpha=0.5)
    axes[0].legend(loc="upper left", ncol=2, fontsize=9)
    axes[0].set_title("CAV All-Way Stop Multi-Metric Dashboard (Baseline FIFO)", fontsize=14, fontweight="bold")

    axes[1].axhline(V_DES, linestyle="--", color="black", alpha=0.7, label="V_des")
    axes[1].set_ylabel("Speed [m/s]", fontweight="bold")
    axes[1].grid(True, linestyle="--", alpha=0.5)

    axes[2].axhline(0.0, linestyle="-", color="gray", alpha=0.5)
    axes[2].set_ylabel(r"Accel [$m/s^2$]", fontweight="bold")
    axes[2].set_xlabel("Time [s]", fontweight="bold")
    axes[2].grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "mpc_dashboard.png"), dpi=200)
    plt.close()
