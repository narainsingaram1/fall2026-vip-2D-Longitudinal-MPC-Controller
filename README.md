# Connected & Autonomous Vehicle (CAV) Intersection Control
**Lab:** ACT Driving Simulator VIP Team (Prof. Srinivas Peeta)  
**Sub-Team:** Sub-Team 4 — Speed Optimization & Intelligent Intersection Control  
**Term:** Fall 2026  

---

## Overview
This repository investigates Model Predictive Control (MPC) and intelligent scheduling architectures for connected automated vehicles navigating all-way stop intersections without physical traffic signals.

The system balances:
1. **Safety:** Preventing collisions inside the conflict zone ($p \in [0, 12]\text{ m}$).
2. **Efficiency:** Minimizing vehicle waiting time, stop delay, and total intersection clearance time.
3. **Passenger Comfort & Energy:** Limiting sudden jerk and excessive acceleration/braking effort.

---

## Directory Structure
```
VIP/
├── README.md                           # Project guide and overview
├── VIP_NOTEBOOK_WEEK1.md               # Week 1 documentation for VIP grading
├── requirements.txt                    # Python dependencies
├── config.py                           # Simulation and MPC hyperparameters
├── run_simulation.py                   # Main simulation runner with metrics & visualization
├── all_way_stop.py                     # Original baseline script (preserved)
├── src/
│   ├── models/
│   │   └── vehicle.py                  # Kinematic state model & trajectory recorder
│   ├── controllers/
│   │   ├── mpc_controller.py           # Dual-stage MPC optimizer (STOP & GO)
│   │   └── fifo_scheduler.py           # First-In-First-Out intersection coordinator
│   ├── metrics/
│   │   └── evaluator.py                # Delay, throughput, energy, and jerk metrics
│   └── visualization/
│       ├── plots.py                    # Multi-panel dashboards & time-series figures
│       └── animator_2d.py              # Top-down 2D bird's-eye view animated GIF
└── results/
    ├── metrics_summary.csv             # Full tabular quantitative metrics
    ├── intersection_animation.gif      # 2D bird's-eye view animation
    ├── mpc_dashboard.png               # 3-panel synchronized trajectory dashboard
    ├── position_vs_time.png            # Longitudinal position curves
    ├── speed_vs_time.png               # Velocity profiles
    └── acceleration_vs_time.png        # Control effort profiles
```

---

## Quickstart

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Run Baseline Simulation & Generate Visuals
```bash
python3 run_simulation.py
```
This will:
- Run the closed-loop MPC simulation.
- Print a formatted KPI report in the terminal.
- Export `results/metrics_summary.csv`.
- Generate all plots and the 2D top-down `results/intersection_animation.gif`.

---

## Semester Milestones (Roadmap)
- **Week 1 (Done):** Modularization, standardized metrics suite, and 2D animated visualizer.
- **Week 2:** Multi-vehicle queueing and car-following headway model ($p_{lead} - p_{follow} \ge d_{min} + v\tau$).
- **Week 3:** Bounded optimization (`SLSQP`/`OSQP`) with hard acceleration bounds and jerk penalty.
- **Week 4–5:** Conflict-matrix concurrent crossing (simultaneous dispatch for non-conflicting directions).
- **Week 6:** Midterm benchmark evaluation (Rule-based vs. FIFO vs. Concurrent MPC).
- **Week 7–8:** OpenRouter LLM supervisor integration and deterministic mathematical safety shield.
- **Week 9–10:** High-density Poisson traffic stress test and final report preparation.
