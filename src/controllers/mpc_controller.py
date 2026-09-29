"""
Model Predictive Controller (MPC) for Longitudinal Stopping and Acceleration
"""
import numpy as np
from scipy.optimize import minimize
from config import (
    DT, N, V_DES, MAXITER, SOLVER_METHOD,
    W_POSITION, W_SPEED, W_ACCEL, W_CROSS
)


class MPCController:
    """
    Solves finite-horizon optimal control problems for approaching a stop line
    and accelerating through an intersection.
    """
    def __init__(
        self,
        dt: float = DT,
        horizon: int = N,
        v_des: float = V_DES,
        w_pos: float = W_POSITION,
        w_speed: float = W_SPEED,
        w_acc: float = W_ACCEL,
        w_cross: float = W_CROSS,
        maxiter: int = MAXITER,
        solver_method: str = SOLVER_METHOD
    ):
        self.dt = dt
        self.N = horizon
        self.v_des = v_des
        self.w_pos = w_pos
        self.w_speed = w_speed
        self.w_acc = w_acc
        self.w_cross = w_cross
        self.maxiter = maxiter
        self.solver_method = solver_method

    def predict(self, p0: float, v0: float, acceleration_sequence: np.ndarray):
        """
        Discrete-time kinematic forward prediction over horizon N.
        p_{k+1} = p_k + v_k * dt + 0.5 * a_k * dt^2
        v_{k+1} = v_k + a_k * dt
        """
        p = np.zeros(self.N + 1)
        v = np.zeros(self.N + 1)

        p[0] = p0
        v[0] = v0

        for k in range(self.N):
            a = acceleration_sequence[k]
            p[k + 1] = p[k] + v[k] * self.dt + 0.5 * a * (self.dt ** 2)
            v[k + 1] = v[k] + a * self.dt

        return p, v

    def stop_cost(self, acceleration_sequence: np.ndarray, vehicle):
        """
        Cost function for Problem 1 (STOP):
        Penalizes terminal position error, terminal speed error, acceleration effort,
        and crossing beyond the stop line (p > 0).
        """
        p, v = self.predict(vehicle.p, vehicle.v, acceleration_sequence)

        cost = self.w_pos * (p[self.N] ** 2)
        cost += self.w_speed * (v[self.N] ** 2)

        for k in range(self.N):
            cost += self.w_acc * (acceleration_sequence[k] ** 2)
            cost += self.w_cross * (max(p[k + 1], 0.0) ** 2)

        return cost

    def solve_stop_problem(self, vehicle) -> float:
        """Solve Problem 1 (STOP) and return the first control input a_0."""
        result = minimize(
            self.stop_cost,
            vehicle.stop_guess,
            args=(vehicle,),
            method=self.solver_method,
            options={"maxiter": self.maxiter, "disp": False},
        )
        optimal_sequence = result.x
        vehicle.stop_guess = optimal_sequence
        return float(optimal_sequence[0])

    def go_cost(self, acceleration_sequence: np.ndarray, vehicle):
        """
        Cost function for Problem 2 (GO):
        Penalizes deviation from desired velocity and acceleration effort.
        """
        _, v = self.predict(vehicle.p, vehicle.v, acceleration_sequence)

        cost = 0.0
        for k in range(self.N):
            cost += self.w_speed * ((v[k + 1] - self.v_des) ** 2)
            cost += self.w_acc * (acceleration_sequence[k] ** 2)

        return cost

    def solve_go_problem(self, vehicle) -> float:
        """Solve Problem 2 (GO) and return the first control input a_0."""
        result = minimize(
            self.go_cost,
            vehicle.go_guess,
            args=(vehicle,),
            method=self.solver_method,
            options={"maxiter": self.maxiter, "disp": False},
        )
        optimal_sequence = result.x
        vehicle.go_guess = optimal_sequence
        return float(optimal_sequence[0])
