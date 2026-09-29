"""
Conflict-Matrix Concurrent Intersection Scheduler
Allows non-conflicting maneuvers (e.g., East <-> West straight) to cross simultaneously.
"""
from typing import List, Set
from config import INTERSECTION_EXIT


class ConcurrentScheduler:
    """
    Schedules vehicle entry using a conflict matrix.
    Opposing straight trajectories (W <-> E and S <-> N) can occupy the
    intersection conflict zone concurrently without collision.
    """
    def __init__(self, exit_point: float = INTERSECTION_EXIT):
        self.queue = []
        self.active_vehicles: Set[int] = set()
        self.next_priority = 1
        self.exit_point = exit_point

    def register_stopped_vehicle(self, vehicle, current_time: float):
        """Register newly stopped vehicle and assign priority."""
        if vehicle.priority is not None:
            return

        vehicle.stop_time = current_time
        vehicle.priority = self.next_priority
        self.next_priority += 1

        self.queue.append(vehicle.id)
        vehicle.state = "WAIT"

    def _is_conflict_free(self, candidate_veh, active_vehicles_list: List) -> bool:
        """
        Conflict Matrix Check:
        Returns True if candidate vehicle has no geometric path conflicts
        with any vehicle currently inside the intersection.
        """
        for active in active_vehicles_list:
            app_a = candidate_veh.approach
            app_b = active.approach

            # Check directional conflict:
            # {W, E} are horizontal corridors; {S, N} are vertical corridors.
            is_horizontal_a = app_a in ["W", "E"]
            is_horizontal_b = app_b in ["W", "E"]

            # If one is horizontal and one is vertical, their paths intersect perpendicularly!
            if is_horizontal_a != is_horizontal_b:
                return False

            # Same approach overtaking is prevented by queue order
            if app_a == app_b:
                return False

        return True

    def release_next_vehicle(self, vehicles: List, current_time: float):
        """
        Greedy multi-vehicle dispatch:
        Check vehicles in priority queue order; release any vehicle that does
        not conflict with already active vehicles in the intersection.
        """
        if len(self.queue) == 0:
            return

        id_to_veh = {v.id: v for v in vehicles}
        currently_active = [id_to_veh[vid] for vid in self.active_vehicles if vid in id_to_veh]

        # Scan queue in arrival order
        for veh_id in list(self.queue):
            cand = id_to_veh.get(veh_id)
            if cand is None or cand.state != "WAIT":
                continue

            if self._is_conflict_free(cand, currently_active):
                cand.state = "GO"
                cand.release_time = current_time
                self.active_vehicles.add(cand.id)
                currently_active.append(cand)

    def update_intersection_clearance(self, vehicles: List, current_time: float):
        """Free vehicles from active set once they clear the exit line."""
        cleared_ids = []
        for veh in vehicles:
            if veh.id in self.active_vehicles:
                if veh.p >= self.exit_point:
                    veh.exit_time = current_time
                    cleared_ids.append(veh.id)
                    if veh.id in self.queue:
                        self.queue.remove(veh.id)

        for cid in cleared_ids:
            self.active_vehicles.remove(cid)
