"""
Vehicle State and Kinematics Model
"""
import numpy as np
from config import N


class Vehicle:
    """
    Represents an autonomous vehicle approaching and crossing an intersection.
    Tracks state, kinematics, and simulation trajectory history.
    """
    def __init__(self, vehicle_id: int, approach: str, position: float, speed: float):
        self.id = vehicle_id
        self.approach = approach  # "W", "E", "S", "N"

        self.p = float(position)
        self.v = float(speed)

        self.state = "APPROACH"  # APPROACH -> WAIT -> GO -> DONE

        self.stop_time = None
        self.release_time = None
        self.exit_time = None
        self.priority = None

        # Warm start guesses for MPC
        self.stop_guess = np.zeros(N)
        self.go_guess = np.zeros(N)

        # Simulation history
        self.time_history = []
        self.position_history = []
        self.speed_history = []
        self.acceleration_history = []
        self.state_history = []

    def record_step(self, time: float, accel: float):
        """Record the state at the current time step."""
        self.time_history.append(time)
        self.position_history.append(self.p)
        self.speed_history.append(self.v)
        self.acceleration_history.append(accel)
        self.state_history.append(self.state)

    def step(self, accel: float, dt: float, stop_line: float):
        """Update vehicle physics for one discrete time step."""
        if self.state == "WAIT":
            self.p = stop_line
            self.v = 0.0
        else:
            self.p += self.v * dt + 0.5 * accel * (dt ** 2)
            self.v += accel * dt
