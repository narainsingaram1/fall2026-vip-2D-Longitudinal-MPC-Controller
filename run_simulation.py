"""
Main Simulation Driver for All-Way-Stop CAV Intersection Control
ACT Driving Simulator VIP Team - Sub-Team 4
"""
import os
import csv
import numpy as np
from config import (
    DT, SIM_TIME, STOP_LINE, STOP_POS_TOL, STOP_SPEED_TOL, DELETE_POINT
)
from src.models import Vehicle
from src.controllers import MPCController, FIFOScheduler
from src.metrics import MetricEvaluator
from src.visualization import plot_simulation_results, IntersectionAnimator2D


def create_scenario_vehicles():
    """Baseline 4-vehicle scenario from lab report."""
    return [
        Vehicle(vehicle_id=1, approach="W", position=-24.0, speed=7.0),
        Vehicle(vehicle_id=2, approach="E", position=-30.0, speed=7.5),
        Vehicle(vehicle_id=3, approach="S", position=-36.0, speed=8.0),
        Vehicle(vehicle_id=4, approach="N", position=-42.0, speed=8.0),
    ]


def run_simulation(vehicles=None, output_dir="results", generate_gif=True):
    """
    Executes the full closed-loop MPC + FIFO intersection simulation.
    """
    os.makedirs(output_dir, exist_ok=True)
    if vehicles is None:
        vehicles = create_scenario_vehicles()

    controller = MPCController()
    scheduler = FIFOScheduler()
    evaluator = MetricEvaluator(dt=DT)

    times = np.arange(0.0, SIM_TIME + DT, DT)

    print(f"\n[INFO] Starting Simulation (SIM_TIME = {SIM_TIME}s, DT = {DT}s)...")
    print("-" * 65)

    for time in times:
        # A. Check whether approaching vehicles have reached the stop line
        for vehicle in vehicles:
            if vehicle.state != "APPROACH":
                continue

            close_to_stop = abs(vehicle.p - STOP_LINE) <= STOP_POS_TOL
            almost_stopped = vehicle.v <= STOP_SPEED_TOL

            if close_to_stop and almost_stopped:
                vehicle.p = STOP_LINE
                vehicle.v = 0.0
                scheduler.register_stopped_vehicle(vehicle, time)

        # B. Dispatch right-of-way according to FIFO policy
        scheduler.release_next_vehicle(vehicles, time)

        # C. Compute optimal acceleration via MPC for each vehicle
        accelerations = {}
        for vehicle in vehicles:
            if vehicle.state == "APPROACH":
                accelerations[vehicle.id] = controller.solve_stop_problem(vehicle)
            elif vehicle.state == "WAIT":
                accelerations[vehicle.id] = 0.0
            elif vehicle.state == "GO":
                accelerations[vehicle.id] = controller.solve_go_problem(vehicle)
            else:  # DONE
                accelerations[vehicle.id] = 0.0

        # D. Update vehicle kinematics and record trajectory history
        for vehicle in vehicles:
            if vehicle.state == "DONE":
                continue

            a = accelerations[vehicle.id]
            vehicle.record_step(time, a)
            vehicle.step(a, DT, STOP_LINE)

        # E. Free the intersection conflict zone once vehicle exits
        scheduler.update_intersection_clearance(vehicles, time)

        # F. Retire vehicles after the downstream deletion point
        for vehicle in vehicles:
            if vehicle.state == "GO" and vehicle.p >= DELETE_POINT:
                vehicle.state = "DONE"

        if all(vehicle.state == "DONE" for vehicle in vehicles):
            print(f"[INFO] All vehicles cleared intersection by t = {time:.1f} s.")
            break

    # G. Quantitative Performance Evaluation
    metrics = evaluator.evaluate_system(vehicles)
    print_metrics_summary(metrics)
    export_metrics_csv(metrics, os.path.join(output_dir, "metrics_summary.csv"))

    # H. Visualizations
    print("\n[INFO] Generating trajectory plots...")
    plot_simulation_results(vehicles, output_dir=output_dir)
    # Also save to root directory for easy access
    plot_simulation_results(vehicles, output_dir=".")

    if generate_gif:
        print("[INFO] Rendering 2D bird's-eye view animation (GIF)...")
        animator = IntersectionAnimator2D(vehicles, dt=DT, fps=10)
        gif_path = os.path.join(output_dir, "intersection_animation.gif")
        animator.generate_gif(gif_path)
        # Also copy to root for instant visibility
        animator.generate_gif("intersection_animation.gif")
        print(f"[SUCCESS] Saved animation to {gif_path}")

    return vehicles, metrics


def print_metrics_summary(metrics):
    """Print structured terminal table of performance indicators."""
    print("\n" + "=" * 65)
    print("           CAV INTERSECTION PERFORMANCE REPORT (BASELINE FIFO)")
    print("=" * 65)
    print(f"Total Vehicles Processed : {metrics['total_vehicles']}")
    print(f"System Clearance Time   : {metrics['system_clear_time_s']:.2f} s")
    print(f"Effective Throughput     : {metrics['throughput_vph']:.1f} veh/hour")
    print(f"Average Delay at Stop    : {metrics['avg_wait_time_s']:.2f} s")
    print(f"Maximum Delay at Stop    : {metrics['max_wait_time_s']:.2f} s")
    print(f"Average Total Trip Time  : {metrics['avg_travel_time_s']:.2f} s")
    print(f"Total Control Effort     : {metrics['total_control_energy']:.2f} m^2/s^3")
    print(f"Mean RMS Jerk (Comfort)  : {metrics['avg_rms_jerk']:.2f} m/s^3")
    print("-" * 65)
    print(f"{'ID':<4} {'Appr':<5} {'Prio':<5} {'Stop(s)':<8} {'Go(s)':<8} {'Exit(s)':<8} {'Wait(s)':<8} {'Energy':<8}")
    print("-" * 65)
    for v in metrics["vehicles"]:
        stop_t = f"{v['stop_time']:.1f}" if v['stop_time'] is not None else "-"
        go_t = f"{v['release_time']:.1f}" if v['release_time'] is not None else "-"
        exit_t = f"{v['exit_time']:.1f}" if v['exit_time'] is not None else "-"
        print(
            f"{v['id']:<4} {v['approach']:<5} {v['priority']:<5} "
            f"{stop_t:<8} {go_t:<8} {exit_t:<8} "
            f"{v['wait_time']:<8.1f} {v['control_energy']:<8.1f}"
        )
    print("=" * 65 + "\n")


def export_metrics_csv(metrics, filepath):
    """Write metrics to CSV for documentation and Excel graphing."""
    with open(filepath, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["System Metric", "Value", "Unit"])
        writer.writerow(["Total Vehicles", metrics["total_vehicles"], "vehicles"])
        writer.writerow(["System Clearance Time", round(metrics["system_clear_time_s"], 2), "s"])
        writer.writerow(["Throughput", round(metrics["throughput_vph"], 1), "veh/h"])
        writer.writerow(["Average Stop Delay", round(metrics["avg_wait_time_s"], 2), "s"])
        writer.writerow(["Maximum Stop Delay", round(metrics["max_wait_time_s"], 2), "s"])
        writer.writerow(["Average Trip Time", round(metrics["avg_travel_time_s"], 2), "s"])
        writer.writerow(["Total Control Energy", round(metrics["total_control_energy"], 2), "m^2/s^3"])
        writer.writerow(["Average RMS Jerk", round(metrics["avg_rms_jerk"], 2), "m/s^3"])
        writer.writerow([])
        writer.writerow(["Vehicle ID", "Approach", "Priority", "Stop Time (s)", "Release Time (s)", "Exit Time (s)", "Wait Time (s)", "Travel Time (s)", "Control Energy", "Max Decel (m/s^2)", "Max Accel (m/s^2)", "RMS Jerk (m/s^3)"])
        for v in metrics["vehicles"]:
            writer.writerow([
                v["id"], v["approach"], v["priority"],
                v["stop_time"], v["release_time"], v["exit_time"],
                round(v["wait_time"], 2), round(v["total_travel_time"], 2),
                round(v["control_energy"], 2), round(v["max_decel"], 2),
                round(v["max_accel"], 2), round(v["rms_jerk"], 2)
            ])


if __name__ == "__main__":
    run_simulation()
