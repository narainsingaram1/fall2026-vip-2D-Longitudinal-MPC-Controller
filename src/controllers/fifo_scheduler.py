"""
First-In-First-Out (FIFO) Intersection Scheduler
"""
from typing import List
from config import INTERSECTION_EXIT


class FIFOScheduler:
    """
    Manages the queue and right-of-way assignment at the intersection
    using a strict First-In-First-Out (FIFO) policy.
    """
    def __init__(self, exit_point: float = INTERSECTION_EXIT):
        self.queue = []
        self.active_vehicle_id = None
        self.next_priority = 1
        self.exit_point = exit_point

    def register_stopped_vehicle(self, vehicle, current_time: float):
        """Add newly stopped vehicle to the queue and assign priority."""
        if vehicle.priority is not None:
            return

        vehicle.stop_time = current_time
        vehicle.priority = self.next_priority
        self.next_priority += 1

        self.queue.append(vehicle.id)
        vehicle.state = "WAIT"

    def release_next_vehicle(self, vehicles: List, current_time: float):
        """If intersection is empty, grant permission to the first vehicle in the queue."""
        if self.active_vehicle_id is not None:
            return

        if len(self.queue) == 0:
            return

        next_id = self.queue[0]
        for vehicle in vehicles:
            if vehicle.id == next_id:
                vehicle.state = "GO"
                vehicle.release_time = current_time
                self.active_vehicle_id = vehicle.id
                return

    def update_intersection_clearance(self, vehicles: List, current_time: float):
        """Free intersection once the active vehicle clears the exit line."""
        if self.active_vehicle_id is None:
            return

        for vehicle in vehicles:
            if vehicle.id == self.active_vehicle_id:
                if vehicle.p >= self.exit_point:
                    vehicle.exit_time = current_time
                    self.queue.pop(0)
                    self.active_vehicle_id = None
                return
