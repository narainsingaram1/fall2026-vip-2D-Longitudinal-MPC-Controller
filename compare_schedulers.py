"""
Comparative Benchmark: Baseline FIFO vs. Concurrent Conflict-Matrix MPC
ACT Driving Simulator VIP Team - Sub-Team 4
"""
import os
import copy
import numpy as np
import matplotlib.pyplot as plt
from config import DT, SIM_TIME, STOP_LINE, STOP_POS_TOL, STOP_SPEED_TOL, DELETE_POINT
from src.models import Vehicle
from src.controllers import MPCController, FIFOScheduler, ConcurrentScheduler
from src.metrics import MetricEvaluator


def build_scenario():
    """Generates a fresh set of initial vehicles for clean comparison."""
    return [
        Vehicle(vehicle_id=1, approach="W", position=-24.0, speed=7.0),
        Vehicle(vehicle_id=2, approach="E", position=-30.0, speed=7.5),
        Vehicle(vehicle_id=3, approach="S", position=-36.0, speed=8.0),
        Vehicle(vehicle_id=4, approach="N", position=-42.0, speed=8.0),
    ]


def simulate_policy(scheduler_class, name="Scheduler"):
    """Simulates intersection traversal under a given scheduler policy."""
    vehicles = build_scenario()
    controller = MPCController()
    scheduler = scheduler_class()
    evaluator = MetricEvaluator(dt=DT)

    times = np.arange(0.0, SIM_TIME + DT, DT)

    for time in times:
        for vehicle in vehicles:
            if vehicle.state == "APPROACH":
                close = abs(vehicle.p - STOP_LINE) <= STOP_POS_TOL
                stopped = vehicle.v <= STOP_SPEED_TOL
                if close and stopped:
                    vehicle.p = STOP_LINE
                    vehicle.v = 0.0
                    scheduler.register_stopped_vehicle(vehicle, time)

        scheduler.release_next_vehicle(vehicles, time)

        accelerations = {}
        for v in vehicles:
            if v.state == "APPROACH":
                accelerations[v.id] = controller.solve_stop_problem(v)
            elif v.state == "WAIT":
                accelerations[v.id] = 0.0
            elif v.state == "GO":
                accelerations[v.id] = controller.solve_go_problem(v)
            else:
                accelerations[v.id] = 0.0

        for v in vehicles:
            if v.state == "DONE":
                continue
            a = accelerations[v.id]
            v.record_step(time, a)
            v.step(a, DT, STOP_LINE)

        scheduler.update_intersection_clearance(vehicles, time)

        for v in vehicles:
            if v.state == "GO" and v.p >= DELETE_POINT:
                v.state = "DONE"

        if all(v.state == "DONE" for v in vehicles):
            break

    metrics = evaluator.evaluate_system(vehicles)
    return vehicles, metrics


def run_benchmark():
    os.makedirs("results", exist_ok=True)
    print("\n" + "=" * 70)
    print("      RUNNING COMPARATIVE STUDY: BASELINE FIFO vs. CONCURRENT MPC")
    print("=" * 70)

    fifo_veh, fifo_m = simulate_policy(FIFOScheduler, "Baseline FIFO")
    conc_veh, conc_m = simulate_policy(ConcurrentScheduler, "Concurrent Conflict-Matrix")

    # Console Summary
    print(f"\n{'Metric':<32} {'Baseline FIFO':<18} {'Concurrent':<18} {'Improvement':<12}")
    print("-" * 75)
    
    # Clearance Time
    c_time_diff = (fifo_m['system_clear_time_s'] - conc_m['system_clear_time_s']) / fifo_m['system_clear_time_s'] * 100
    print(f"{'Clearance Time (s)':<32} {fifo_m['system_clear_time_s']:<18.2f} {conc_m['system_clear_time_s']:<18.2f} {c_time_diff:+.1f}%")

    # Average Delay
    delay_diff = (fifo_m['avg_wait_time_s'] - conc_m['avg_wait_time_s']) / fifo_m['avg_wait_time_s'] * 100
    print(f"{'Average Stop Delay (s)':<32} {fifo_m['avg_wait_time_s']:<18.2f} {conc_m['avg_wait_time_s']:<18.2f} {delay_diff:+.1f}%")

    # Max Delay
    max_delay_diff = (fifo_m['max_wait_time_s'] - conc_m['max_wait_time_s']) / fifo_m['max_wait_time_s'] * 100
    print(f"{'Maximum Delay (s)':<32} {fifo_m['max_wait_time_s']:<18.2f} {conc_m['max_wait_time_s']:<18.2f} {max_delay_diff:+.1f}%")

    # Throughput
    tput_diff = (conc_m['throughput_vph'] - fifo_m['throughput_vph']) / fifo_m['throughput_vph'] * 100
    print(f"{'Throughput (veh/hr)':<32} {fifo_m['throughput_vph']:<18.1f} {conc_m['throughput_vph']:<18.1f} {tput_diff:+.1f}%")

    # Total Energy
    energy_diff = (fifo_m['total_control_energy'] - conc_m['total_control_energy']) / fifo_m['total_control_energy'] * 100
    print(f"{'Total Control Energy':<32} {fifo_m['total_control_energy']:<18.2f} {conc_m['total_control_energy']:<18.2f} {energy_diff:+.1f}%")
    print("=" * 75 + "\n")

    # Comparative Plot
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    metrics_names = ["System Clearance (s)", "Average Delay (s)", "Max Delay (s)"]
    fifo_vals = [fifo_m['system_clear_time_s'], fifo_m['avg_wait_time_s'], fifo_m['max_wait_time_s']]
    conc_vals = [conc_m['system_clear_time_s'], conc_m['avg_wait_time_s'], conc_m['max_wait_time_s']]

    x = np.arange(len(metrics_names))
    width = 0.35

    axes[0].bar(x - width/2, fifo_vals, width, label="Baseline FIFO", color="#e74c3c")
    axes[0].bar(x + width/2, conc_vals, width, label="Concurrent Conflict Matrix", color="#2ecc71")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(metrics_names, fontsize=9, fontweight="bold")
    axes[0].set_ylabel("Seconds [s]", fontweight="bold")
    axes[0].set_title("Time & Delay Metrics", fontsize=12, fontweight="bold")
    axes[0].grid(True, linestyle="--", alpha=0.5)
    axes[0].legend()

    # Throughput comparison
    axes[1].bar(["Baseline FIFO", "Concurrent"], [fifo_m['throughput_vph'], conc_m['throughput_vph']], color=["#e74c3c", "#2ecc71"], width=0.5)
    axes[1].set_ylabel("Vehicles / Hour", fontweight="bold")
    axes[1].set_title("Intersection Throughput", fontsize=12, fontweight="bold")
    axes[1].grid(True, linestyle="--", alpha=0.5)

    # Trajectory comparison (Position vs Time for both)
    for v in fifo_veh:
        axes[2].plot(v.time_history, v.position_history, linestyle="--", alpha=0.7, label=f"FIFO V{v.id} ({v.approach})")
    for v in conc_veh:
        axes[2].plot(v.time_history, v.position_history, linestyle="-", linewidth=2.0, label=f"Conc V{v.id} ({v.approach})")

    axes[2].axhline(0.0, linestyle=":", color="black")
    axes[2].axhline(12.0, linestyle=":", color="red")
    axes[2].set_xlabel("Time [s]", fontweight="bold")
    axes[2].set_ylabel("Position [m]", fontweight="bold")
    axes[2].set_title("Trajectory Overlay (Solid=Concurrent, Dash=FIFO)", fontsize=11, fontweight="bold")
    axes[2].grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    chart_path = "results/comparison_fifo_vs_concurrent.png"
    plt.savefig(chart_path, dpi=200)
    plt.close()
    print(f"[SUCCESS] Comparative plot saved to {chart_path}")


if __name__ == "__main__":
    run_benchmark()
