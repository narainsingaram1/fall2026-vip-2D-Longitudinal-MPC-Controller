"""
Simulation and MPC Configuration Parameters
ACT Driving Simulator VIP - CAV Intersection Control
"""

# Simulation Parameters
DT = 0.20                 # sampling time [s]
SIM_TIME = 35.0           # maximum simulation time [s]
N = 20                    # prediction horizon
V_DES = 8.0               # desired speed in GO [m/s]

# Geometry [m]
STOP_LINE = 0.0           # stop line position [m]
INTERSECTION_EXIT = 12.0  # intersection is clear after this point [m]
DELETE_POINT = 25.0       # remove vehicle after this point [m]

# Tolerances
STOP_POS_TOL = 0.50       # stopping position tolerance [m]
STOP_SPEED_TOL = 0.20     # stopping speed tolerance [m/s]

# MPC Cost Weights (Baseline from Lab Report)
W_POSITION = 100.0
W_SPEED = 150.0
W_ACCEL = 0.20
W_CROSS = 2000.0

# Optimizer Settings
MAXITER = 60
SOLVER_METHOD = "BFGS"
