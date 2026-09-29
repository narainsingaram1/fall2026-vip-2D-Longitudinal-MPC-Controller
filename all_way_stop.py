"""
Simple MPC + FIFO Controller for an All-Way-Stop Intersection
=============================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


# =============================================================================
# 1. PARAMETERS
# =============================================================================

DT = 0.20                 # sampling time [s]
SIM_TIME = 35.0           # maximum simulation time [s]
N = 20                    # prediction horizon
V_DES = 8.0               # desired speed in GO [m/s]

STOP_LINE = 0.0           # stop line position [m]
INTERSECTION_EXIT = 12.0  # intersection is clear after this point [m]
DELETE_POINT = 25.0       # remove vehicle after this point [m]

STOP_POS_TOL = 0.50       # stopping position tolerance [m]
STOP_SPEED_TOL = 0.20     # stopping speed tolerance [m/s]

# Objective-function weights from the report
W_POSITION = 100.0
W_SPEED = 150.0
W_ACCEL = 0.20
W_CROSS = 2000.0

MAXITER = 60


# =============================================================================
# 2. CAR
# =============================================================================

class Car:
    def __init__(self, car_id, approach, position, speed):
        self.id = car_id
        self.approach = approach

        self.p = position
        self.v = speed

        self.state = "APPROACH"

        self.stop_time = None
        self.priority = None

        # Initial guesses for the two optimization problems.
        # Each guess contains N acceleration values.
        self.stop_guess = np.zeros(N)
        self.go_guess = np.zeros(N)

        # Data saved for plots.
        self.time_history = []
        self.position_history = []
        self.speed_history = []
        self.acceleration_history = []
        self.state_history = []


# =============================================================================
# 3. CONTROLLER
# =============================================================================

class Controller:
    def __init__(self):
        self.queue = []
        self.active_car = None
        self.next_priority = 1

    # -------------------------------------------------------------------------
    # Vehicle prediction model
    # -------------------------------------------------------------------------
    def predict(self, p0, v0, acceleration_sequence):

        p = np.zeros(N + 1)
        v = np.zeros(N + 1)

        p[0] = p0
        v[0] = v0

        for k in range(N):
            a = acceleration_sequence[k]

            p[k + 1] = p[k] + v[k] * DT + 0.5 * a * DT**2
            v[k + 1] = v[k] + a * DT

        return p, v

    # -------------------------------------------------------------------------
    # Problem 1: STOP
    # -------------------------------------------------------------------------
    def stop_cost(self, acceleration_sequence, car):
        """
        J_stop = W_POSITION * p(N)^2
               + W_SPEED    * v(N)^2
               + sum[ W_ACCEL * a(k)^2
                    + W_CROSS * max(p(k+1), 0)^2 ]
        """

        p, v = self.predict(car.p, car.v, acceleration_sequence)

        cost = W_POSITION * p[N]**2
        cost += W_SPEED * v[N]**2

        for k in range(N):
            cost += W_ACCEL * acceleration_sequence[k]**2
            cost += W_CROSS * max(p[k + 1], 0.0)**2

        return cost

    def solve_stop_problem(self, car):
        """Solve the STOP problem and return the first optimal acceleration."""

        result = minimize(
            self.stop_cost,
            car.stop_guess,
            args=(car,),
            method="BFGS",
            options={"maxiter": MAXITER, "disp": False},
        )

        optimal_sequence = result.x
        car.stop_guess = optimal_sequence

        # MPC: apply only the first input.
        return optimal_sequence[0]

    # -------------------------------------------------------------------------
    # Problem 2: GO
    # -------------------------------------------------------------------------
    def go_cost(self, acceleration_sequence, car):
        """
        J_go = sum[ W_SPEED * (v(k+1) - V_DES)^2
                  + W_ACCEL * a(k)^2 ]
        """

        _, v = self.predict(car.p, car.v, acceleration_sequence)

        cost = 0.0

        for k in range(N):
            cost += W_SPEED * (v[k + 1] - V_DES)**2
            cost += W_ACCEL * acceleration_sequence[k]**2

        return cost

    def solve_go_problem(self, car):
        """Solve the GO problem and return the first optimal acceleration."""

        result = minimize(
            self.go_cost,
            car.go_guess,
            args=(car,),
            method="BFGS",
            options={"maxiter": MAXITER, "disp": False},
        )

        optimal_sequence = result.x
        car.go_guess = optimal_sequence

        # MPC: apply only the first input.
        return optimal_sequence[0]

    # -------------------------------------------------------------------------
    # FIFO logic
    # -------------------------------------------------------------------------
    def register_stopped_car(self, car, time):
        """Add a newly stopped car to the FIFO queue."""

        if car.priority is not None:
            return

        car.stop_time = time
        car.priority = self.next_priority
        self.next_priority += 1

        self.queue.append(car.id)
        car.state = "WAIT"

    def release_next_car(self, cars):
        """If the intersection is free, allow the first car in the queue to go."""

        if self.active_car is not None:
            return

        if len(self.queue) == 0:
            return

        next_id = self.queue[0]

        for car in cars:
            if car.id == next_id:
                car.state = "GO"
                self.active_car = car.id
                return

    def check_intersection(self, cars):
        """Free the intersection after the active car reaches the exit point."""

        if self.active_car is None:
            return

        for car in cars:
            if car.id == self.active_car:
                if car.p >= INTERSECTION_EXIT:
                    self.queue.pop(0)
                    self.active_car = None
                return


# =============================================================================
# 4. DEMONSTRATION VEHICLES
# =============================================================================

def create_cars():
    return [
        Car(1, "W", -24.0, 7.0),
        Car(2, "E", -30.0, 7.5),
        Car(3, "S", -36.0, 8.0),
        Car(4, "N", -42.0, 8.0),
    ]


# =============================================================================
# 5. SIMULATION
# =============================================================================

def run_simulation():
    cars = create_cars()
    controller = Controller()

    times = np.arange(0.0, SIM_TIME + DT, DT)

    for time in times:

        # ---------------------------------------------------------------------
        # A. Check whether APPROACH cars have stopped.
        # ---------------------------------------------------------------------
        # Cars are checked in ID order. If two stop at exactly the same time,
        # the smaller ID receives priority first.
        for car in cars:
            if car.state != "APPROACH":
                continue

            close_to_stop_line = abs(car.p - STOP_LINE) <= STOP_POS_TOL
            almost_stopped = car.v <= STOP_SPEED_TOL

            if close_to_stop_line and almost_stopped:
                car.p = STOP_LINE
                car.v = 0.0
                controller.register_stopped_car(car, time)

        # ---------------------------------------------------------------------
        # B. If possible, release the first car in the FIFO queue.
        # ---------------------------------------------------------------------
        controller.release_next_car(cars)

        # ---------------------------------------------------------------------
        # C. Compute one acceleration to apply to each car.
        # ---------------------------------------------------------------------
        accelerations = {}

        for car in cars:
            if car.state == "APPROACH":
                accelerations[car.id] = controller.solve_stop_problem(car)

            elif car.state == "WAIT":
                accelerations[car.id] = 0.0

            elif car.state == "GO":
                accelerations[car.id] = controller.solve_go_problem(car)

            else:  # DONE
                accelerations[car.id] = 0.0

        # ---------------------------------------------------------------------
        # D. Save the current state and move each car one simulation step.
        # ---------------------------------------------------------------------
        for car in cars:
            if car.state == "DONE":
                continue

            a = accelerations[car.id]

            car.time_history.append(time)
            car.position_history.append(car.p)
            car.speed_history.append(car.v)
            car.acceleration_history.append(a)
            car.state_history.append(car.state)

            if car.state == "WAIT":
                car.p = STOP_LINE
                car.v = 0.0

            else:
                car.p = car.p + car.v * DT + 0.5 * a * DT**2
                car.v = car.v + a * DT

        # ---------------------------------------------------------------------
        # E. Check whether the active car has cleared the intersection.
        # ---------------------------------------------------------------------
        controller.check_intersection(cars)

        # ---------------------------------------------------------------------
        # F. Remove cars after the downstream deletion point.
        # ---------------------------------------------------------------------
        for car in cars:
            if car.state == "GO" and car.p >= DELETE_POINT:
                car.state = "DONE"

        if all(car.state == "DONE" for car in cars):
            break

    return cars


# =============================================================================
# 6. RESULTS
# =============================================================================

def print_summary(cars):
    print("\nFIFO crossing order")
    print("-------------------")

    ordered_cars = sorted(cars, key=lambda car: car.priority)

    for car in ordered_cars:
        print(
            f"Priority {car.priority}: "
            f"Vehicle {car.id} ({car.approach}), "
            f"stop time = {car.stop_time:.1f} s"
        )


def plot_results(cars):
    # Position plot
    plt.figure(figsize=(9, 5))

    for car in cars:
        plt.plot(
            car.time_history,
            car.position_history,
            label=f"Vehicle {car.id} ({car.approach})",
        )

    plt.axhline(STOP_LINE, linestyle="--", linewidth=1, label="Stop line")
    plt.axhline(
        INTERSECTION_EXIT,
        linestyle=":",
        linewidth=1,
        label="Intersection exit",
    )

    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.title("Vehicle Position")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("position_vs_time.png", dpi=200)
    plt.close()

    # Speed plot
    plt.figure(figsize=(9, 5))

    for car in cars:
        plt.plot(
            car.time_history,
            car.speed_history,
            label=f"Vehicle {car.id} ({car.approach})",
        )

    plt.axhline(V_DES, linestyle="--", linewidth=1, label="Desired speed")

    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Vehicle Speed")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("speed_vs_time.png", dpi=200)
    plt.close()


# =============================================================================
# 7. MAIN
# =============================================================================

if __name__ == "__main__":
    cars = run_simulation()
    print_summary(cars)
    plot_results(cars)
