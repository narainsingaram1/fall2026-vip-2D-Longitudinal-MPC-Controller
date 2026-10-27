# Findings and recommendation

## Decision

Use the **squared-softplus elliptical potential as the working research
candidate**, with obstacle weight 100. Keep its acceleration and jerk costs.
For a comfort-focused demonstration, also show `softplus_comfort`: it raises
both comfort coefficients fourfold and gives a useful measured tradeoff.

This selects an objective for further development. It does **not** establish
a complete safe intersection controller: the simultaneous-arrival test remains
incomplete, and many optimization solves reach their iteration limit.

## What was actually run

Eight configurations were evaluated on each of three deterministic four-car
scenarios: 24 closed-loop runs. All runs use the same planar dynamics, vehicle
bodies, lane geometry, acceleration/speed bounds, 4-second prediction horizon,
60-iteration solver budget and 24-second maximum run duration.

`no_obstacle` sets only the obstacle weight to zero within the new experiment.
It is a controlled ablation, **not the original root-level FIFO simulation**.
The new experiment omits mandatory full stops, so direct claims about improving
the original all-way-stop system would be unsupported.

## Main comparison: offset arrivals

| Configuration | Completed | Clearance [s] | Minimum crossing-pair q | Body-overlap pair-steps | Acceleration RMS [m/s²] | Jerk RMS [m/s³] |
|---|---:|---:|---:|---:|---:|---:|
| No obstacle term | 4/4 | 6.4 | 0.009 | 12 | 0.343 | 0.598 |
| Gaussian | 0/4 | incomplete at 24 s | 6.328 | 0 | 0.595 | 0.503 |
| Reciprocal | 4/4 | 10.0 | 4.791 | 0 | 1.520 | 2.104 |
| Softplus, default | 4/4 | 6.8 | 1.266 | 0 | 0.710 | 1.094 |
| Softplus, comfort weights | 4/4 | 7.0 | 1.234 | 0 | 0.601 | 0.715 |

Interpretation:

1. **The no-obstacle result is physically unacceptable.** Its shorter time
   includes body overlaps, so it cannot be treated as the best controller.
2. **Large separation alone is insufficient.** Gaussian keeps vehicles far
   apart but fails to complete this crossing. Its low jerk partly reflects
   braking toward a stalled configuration rather than useful traffic flow.
3. **Reciprocal works here but is conservative.** It maintains substantially
   larger crossing separation and takes 10.0 s. Its long tail is a plausible
   contributor, but this experiment does not isolate tail shape from gradient
   conditioning or optimizer convergence; it does not prove the cause.
4. **Default softplus balances these measured outcomes.** All cars clear in
   6.8 s, no rectangle overlap occurs, and the crossing-pair minimum stays above
   the q=1 buffer boundary. Compared with reciprocal in this scenario, clearance
   is 32% shorter and RMS jerk about 48% lower. These are sample-specific results.
5. **The comfort variant reduces jerk further.** Jerk RMS falls about 35%
   from 1.094 to 0.715 m/s³ for an additional 0.2 s of sampled clearance time.
   Acceleration RMS also falls. Because both comfort weights change together,
   the result supports the combined tuning change, not an isolated coefficient.

## Weight effects: do not assume a larger safety weight is always better

### Staggered arrivals

| Obstacle weight | Minimum crossing q | Clearance [s] | Jerk RMS [m/s³] |
|---|---:|---:|---:|
| 10 | 1.383 | 6.8 | 0.327 |
| 100 | 1.590 | 7.0 | 0.412 |
| 500 | 2.351 | 7.0 | 1.065 |

Increasing obstacle weight buys additional geometric separation here, but
also increases jerk. The all-pair minimum would obscure this trend because
parallel-lane opposing cars have a near-constant closest q of about 1.158.
That is why the comparison figure also reports crossing-only separation.

### Offset arrivals

| Obstacle weight | Minimum crossing q | Clearance [s] | Jerk RMS [m/s³] |
|---|---:|---:|---:|
| 10 | 1.00116 | 6.6 | 0.795 |
| 100 | 1.266 | 6.8 | 1.094 |
| 500 | 1.519 | 8.4 | 1.933 |

Weight 10 stays outside the nominal ellipse in this run, but only barely.
It is a poor basis for claims of robustness to state uncertainty or modeling
error. Weight 500 increases separation but adds 1.6 s relative to weight 100
and produces a peak jerk of 17.502 m/s³. A soft jerk cost is not a hard jerk limit.
Weight 100 is a defensible starting compromise, not a certified safety setting.

Removing the jerk term gives the same 6.8 s offset clearance as the default,
but raises RMS jerk from 1.094 to 1.984 m/s³, about 81%. In the staggered case,
it raises RMS jerk from 0.412 to 1.486 m/s³. This provides direct evidence for
retaining a jerk penalty in this controller.

## The stress case that remains unsolved

All cars start 28 m before the center at 7 m/s in the simultaneous scenario.

- With no obstacle term, all four clear in 5.2 s but there are 8 body-overlap
  pair-steps. A fast collision is not successful coordination.
- Every configuration with an obstacle term produces zero body overlap and
  stays outside its elliptical buffer, but **0/4 cars complete within 24 s**.
- Default softplus reaches minimum crossing q=2.423 and brakes toward very low
  speeds. The dashboard shows nearly identical commands for all four cars.
- Every default softplus solve in this scenario reaches the 60-iteration limit.

The observed behavior is consistent with symmetric braking and lack of a
right-of-way tie break. It does not prove a converged local minimum or permanent
deadlock: the finite optimizer budget is a confounding factor. Longer solves,
perturbed warm starts and explicit priority would be useful separate experiments.
The supported conclusion is that the **implemented controller fails this
24-second completion test**.

Across all 21 runs with an obstacle term, no physical overlap or elliptical
buffer violation was observed. Thirteen of those 21 runs completed; seven
simultaneous runs and the offset Gaussian run did not. This is a small scenario
set, not a safety success probability.

## Numerical verification and practical limits

- All eight automated tests pass. They include analytic-gradient checks,
  an independent projection oracle, backend equivalence and a deliberately
  between-sample collision case.
- The executed dynamics are four-component constant-acceleration updates with
  no artificial position resets or speed clipping.
- All accepted objective traces are nonincreasing within numerical tolerance.
  This confirms descent behavior, not global optimality.
- The largest recorded predicted bound violation across the suite is below
  1e-7 in the numerical units of the bounds.
- Default softplus reaches the iteration limit in 12/35 staggered solves,
  18/34 offset solves and all 120/120 simultaneous solves. The last iterate is
  used after an iteration limit; it is not relabeled an optimum.
- The default symmetric run takes about 43.4 seconds of local computation for
  24 seconds of simulated motion, even with the optional compiled projection.
  It therefore does not meet an average real-time loop budget in this case.
  No worst-case execution-time guarantee is available.
- Jerk is penalized, not bounded. No passenger study, energy model, noisy sensing,
  emergency safety layer, turning dynamics or repeated traffic is included.
- Every previously tracked repository file was checked against HEAD and remains
  byte-for-byte unchanged. Only the separate experiment folder was added.

## What to say when presenting

> I extended the predictor from route distance and speed to world x, y, vx and
> vy. Cars stay in fixed lanes, but the objective now accounts for the geometry
> of every vehicle pair. I compared three smooth repulsive potentials and used
> projected gradient descent to optimize acceleration sequences. Squared
> softplus gave a useful balance of completion, separation and comfort in the
> asymmetric cases. The weight experiments show that more repulsion can increase
> jerk and crossing time. The simultaneous-arrival case still fails to complete,
> so a potential field alone is not a complete coordination solution.

## Recommended next change

Keep the working softplus/comfort objective, restore explicit right-of-way and
all-way-stop requirements if those remain the project contract, add hard
predicted separation constraints with a braking-feasibility policy, and rerun
the symmetric case. The current results justify that next step; they do not
justify removing coordination safeguards from the existing main simulator.

For background on potential-field traps and oscillation limitations, see
[Koren and Borenstein's primary paper](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/borenstein_potential_field_limitations.pdf).
That source discusses general navigation limitations; the quantitative results
above come solely from the saved local experiment runs.
