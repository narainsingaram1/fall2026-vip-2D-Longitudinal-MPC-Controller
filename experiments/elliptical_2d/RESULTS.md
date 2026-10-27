# Measured results

Generated from `results/summary.json`. Clearance is the first sample when every center has passed its route coordinate +12 m; maximum run duration is 24 s.

| Scenario | Configuration | Complete | Clearance s | Min q, all | Min q, crossing | Body overlap pair-steps | Accel RMS m/s² | Jerk RMS m/s³ | Iteration-limit solves |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| staggered | no_obstacle | 4/4 | 6.8 | 1.112 | 1.112 | 0 | 0.162 | 0.283 | 1/34 |
| staggered | gaussian | 4/4 | 7.4 | 1.158 | 5.220 | 0 | 0.864 | 1.427 | 23/37 |
| staggered | reciprocal | 4/4 | 8.4 | 1.158 | 8.049 | 0 | 1.383 | 2.403 | 29/42 |
| staggered | softplus | 4/4 | 7.0 | 1.158 | 1.590 | 0 | 0.229 | 0.412 | 12/35 |
| staggered | softplus_low_safety | 4/4 | 6.8 | 1.158 | 1.383 | 0 | 0.193 | 0.327 | 10/34 |
| staggered | softplus_high_safety | 4/4 | 7.0 | 1.158 | 2.351 | 0 | 0.533 | 1.065 | 19/35 |
| staggered | softplus_comfort | 4/4 | 7.0 | 1.158 | 1.527 | 0 | 0.200 | 0.257 | 26/35 |
| staggered | softplus_no_jerk | 4/4 | 6.8 | 1.158 | 1.639 | 0 | 0.338 | 1.486 | 16/34 |
| simultaneous | no_obstacle | 4/4 | 5.2 | 0.230 | 0.230 | 8 | 0.332 | 0.579 | 1/26 |
| simultaneous | gaussian | 0/4 | incomplete | 6.541 | 6.541 | 0 | 0.601 | 0.561 | 120/120 |
| simultaneous | reciprocal | 0/4 | incomplete | 11.779 | 11.779 | 0 | 0.673 | 0.789 | 120/120 |
| simultaneous | softplus | 0/4 | incomplete | 2.423 | 2.423 | 0 | 0.563 | 0.400 | 120/120 |
| simultaneous | softplus_low_safety | 0/4 | incomplete | 1.950 | 1.950 | 0 | 0.548 | 0.388 | 120/120 |
| simultaneous | softplus_high_safety | 0/4 | incomplete | 2.551 | 2.551 | 0 | 0.567 | 0.501 | 120/120 |
| simultaneous | softplus_comfort | 0/4 | incomplete | 2.141 | 2.141 | 0 | 0.558 | 0.324 | 120/120 |
| simultaneous | softplus_no_jerk | 0/4 | incomplete | 2.510 | 2.510 | 0 | 0.557 | 0.600 | 120/120 |
| offset | no_obstacle | 4/4 | 6.4 | 0.009 | 0.009 | 12 | 0.343 | 0.598 | 1/32 |
| offset | gaussian | 0/4 | incomplete | 6.328 | 6.328 | 0 | 0.595 | 0.503 | 120/120 |
| offset | reciprocal | 4/4 | 10.0 | 1.158 | 4.791 | 0 | 1.520 | 2.104 | 37/50 |
| offset | softplus | 4/4 | 6.8 | 1.158 | 1.266 | 0 | 0.710 | 1.094 | 18/34 |
| offset | softplus_low_safety | 4/4 | 6.6 | 1.001 | 1.001 | 0 | 0.575 | 0.795 | 15/33 |
| offset | softplus_high_safety | 4/4 | 8.4 | 1.158 | 1.519 | 0 | 1.222 | 1.933 | 23/42 |
| offset | softplus_comfort | 4/4 | 7.0 | 1.158 | 1.234 | 0 | 0.601 | 0.715 | 32/35 |
| offset | softplus_no_jerk | 4/4 | 6.8 | 1.158 | 1.287 | 0 | 0.818 | 1.984 | 17/34 |

A pair-step counts one overlapping vehicle pair during one executed 0.2 s interval. It is not a count of distinct crash events. `q<1` is a buffer violation; body overlap is checked separately using swept rectangles.

No-obstacle uses the same new dynamics, optimizer and comfort terms with obstacle weight zero. It is an ablation, not a rerun of the original FIFO/BFGS simulator.

The all-pair minimum can be dominated by opposing parallel lanes: their lateral gap sets a floor of about q=1.158 when their x/y coordinates align. The crossing-only minimum excludes parallel pairs so the effects of changing the obstacle weight remain visible. It is independently recomputed from saved trajectories and saved in `results/crossing_separation.csv`.

RMS metrics are pooled over all four vehicles and every applied command until each run ends. Different run durations mean these are run-level comfort summaries, not matched-duration causal estimates. No physical energy consumption is computed.
