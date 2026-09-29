"""
Performance Evaluator and Metric Suite for CAV Intersection Control
"""
import numpy as np
from typing import List, Dict, Any


class MetricEvaluator:
    """
    Computes rigorous transportation and control performance metrics:
    - Delay & Travel Times
    - Control Effort & Energy Proxy
    - Passenger Comfort (Jerk)
    - Throughput & Intersection Clearance
    """
    def __init__(self, dt: float):
        self.dt = dt

    def evaluate_vehicle(self, vehicle) -> Dict[str, Any]:
        """Compute metrics for an individual vehicle."""
        times = np.array(vehicle.time_history)
        accels = np.array(vehicle.acceleration_history)
        speeds = np.array(vehicle.speed_history)

        total_travel_time = times[-1] - times[0] if len(times) > 1 else 0.0
        
        # Wait time at stop line
        wait_time = 0.0
        if vehicle.stop_time is not None and vehicle.release_time is not None:
            wait_time = vehicle.release_time - vehicle.stop_time
        
        # Crossing duration inside intersection box
        crossing_time = 0.0
        if vehicle.release_time is not None and vehicle.exit_time is not None:
            crossing_time = vehicle.exit_time - vehicle.release_time

        # Control effort: integral of a^2 dt
        control_energy = float(np.sum(accels ** 2) * self.dt)

        # Jerk analysis (da/dt)
        if len(accels) > 1:
            jerks = np.diff(accels) / self.dt
            max_jerk = float(np.max(np.abs(jerks)))
            rms_jerk = float(np.sqrt(np.mean(jerks ** 2)))
        else:
            max_jerk = 0.0
            rms_jerk = 0.0

        max_decel = float(np.min(accels)) if len(accels) > 0 else 0.0
        max_accel = float(np.max(accels)) if len(accels) > 0 else 0.0

        return {
            "id": vehicle.id,
            "approach": vehicle.approach,
            "priority": vehicle.priority,
            "stop_time": vehicle.stop_time,
            "release_time": vehicle.release_time,
            "exit_time": vehicle.exit_time,
            "wait_time": wait_time,
            "crossing_time": crossing_time,
            "total_travel_time": total_travel_time,
            "control_energy": control_energy,
            "max_decel": max_decel,
            "max_accel": max_accel,
            "max_jerk": max_jerk,
            "rms_jerk": rms_jerk,
        }

    def evaluate_system(self, vehicles: List) -> Dict[str, Any]:
        """Compute aggregate system-level performance indicators."""
        veh_metrics = [self.evaluate_vehicle(v) for v in vehicles]

        system_clear_time = max(
            [m["exit_time"] for m in veh_metrics if m["exit_time"] is not None] or [0.0]
        )
        total_vehicles = len(vehicles)
        avg_wait_time = float(np.mean([m["wait_time"] for m in veh_metrics]))
        max_wait_time = float(np.max([m["wait_time"] for m in veh_metrics]))
        avg_travel_time = float(np.mean([m["total_travel_time"] for m in veh_metrics]))
        total_energy = float(np.sum([m["control_energy"] for m in veh_metrics]))
        avg_rms_jerk = float(np.mean([m["rms_jerk"] for m in veh_metrics]))

        throughput_vph = (total_vehicles / system_clear_time * 3600.0) if system_clear_time > 0 else 0.0

        return {
            "total_vehicles": total_vehicles,
            "system_clear_time_s": system_clear_time,
            "throughput_vph": throughput_vph,
            "avg_wait_time_s": avg_wait_time,
            "max_wait_time_s": max_wait_time,
            "avg_travel_time_s": avg_travel_time,
            "total_control_energy": total_energy,
            "avg_rms_jerk": avg_rms_jerk,
            "vehicles": veh_metrics,
        }
