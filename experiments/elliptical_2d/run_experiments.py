"""Reproducible experiment runner. Writes only inside this experiment folder."""
import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
import platform
from pathlib import Path
import time
import numpy as np
from controller import Controller, Settings, PROJECTION_BACKEND
from model import DT, HEADINGS, PAIRS, initial_state, step, swept_pair

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'results'


def simulate(settings, scenario, duration=24.):
    state = initial_state(scenario)
    controller = Controller(settings)
    previous = np.zeros(4)
    states, controls, diagnostics = [state.copy()], [], []
    minq = np.inf
    overlaps, buffer_steps = 0, 0
    collision_pairs = set()
    exit_times = [None]*4
    start = time.perf_counter()
    for k in range(round(duration/DT)):
        a, diagnostic = controller.solve(state, previous)
        acceleration = a[:, None]*HEADINGS
        unsafe = False
        for i, j in PAIRS:
            q, collision = swept_pair(state, acceleration, i, j)
            minq = min(minq, q)
            unsafe |= q < 1-1e-7
            if collision:
                overlaps += 1
                collision_pairs.add((i+1, j+1))
        buffer_steps += int(unsafe)
        state = step(state, acceleration)
        states.append(state.copy())
        controls.append(a)
        diagnostics.append(diagnostic)
        previous = a
        progress = np.sum(state[:, :2]*HEADINGS, axis=1)
        for i in range(4):
            if exit_times[i] is None and progress[i] >= 12.:
                exit_times[i] = (k+1)*DT
        if all(t is not None for t in exit_times):
            break
    elapsed = time.perf_counter()-start
    states, controls = np.array(states), np.array(controls)
    jerk = np.diff(controls, axis=0, prepend=np.zeros((1, 4)))/DT
    speeds = np.sum(states[..., 2:]*HEADINGS, axis=-1)
    metrics = {
        'completed': sum(t is not None for t in exit_times),
        'clearance_s': max(exit_times) if all(t is not None for t in exit_times) else None,
        'min_q': minq, 'collision_pair_steps': overlaps,
        'collision_pairs': sorted(collision_pairs), 'buffer_violation_steps': buffer_steps,
        'acceleration_rms': float(np.sqrt(np.mean(controls**2))),
        'jerk_rms': float(np.sqrt(np.mean(jerk**2))),
        'peak_abs_acceleration': float(np.max(np.abs(controls))),
        'peak_abs_jerk': float(np.max(np.abs(jerk))),
        'effort_integral': float(DT*np.sum(controls**2)),
        'min_speed': float(np.min(speeds)), 'max_speed': float(np.max(speeds)),
        'max_constraint_violation': max(d['constraint_violation'] for d in diagnostics),
        'stationary_solves': sum(d['status']=='stationary' for d in diagnostics),
        'iteration_limit_solves': sum(d['status']=='iteration_limit' for d in diagnostics),
        'line_search_failures': sum(d['status']=='line_search_failed' for d in diagnostics),
        'solves': len(diagnostics), 'runtime_s': elapsed,
        'exit_times_s': exit_times,
    }
    return metrics, states, controls, diagnostics


def configurations(iterations=60):
    s = Settings(iterations=iterations)
    return {
        'no_obstacle': replace(s, obstacle_weight=0.),
        'gaussian': replace(s, potential='gaussian'),
        'reciprocal': replace(s, potential='reciprocal'),
        'softplus': s,
        'softplus_low_safety': replace(s, obstacle_weight=10.),
        'softplus_high_safety': replace(s, obstacle_weight=500.),
        'softplus_comfort': replace(s, acceleration_weight=.6, jerk_weight=.4),
        'softplus_no_jerk': replace(s, jerk_weight=0.),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--quick', action='store_true', help='one smoke run; separate quick output')
    args = parser.parse_args()
    output = OUT / 'quick' if args.quick else OUT
    output.mkdir(parents=True, exist_ok=True)
    configs = configurations()
    if args.quick:
        configs = {'softplus': Settings(iterations=40)}
    records = []
    for scenario in (['staggered'] if args.quick else ['staggered', 'simultaneous', 'offset']):
        for name, settings in configs.items():
            metrics, states, controls, diagnostic = simulate(settings, scenario)
            record = {'scenario': scenario, 'configuration': name,
                      'settings': asdict(settings), **metrics}
            records.append(record)
            stem = f'{scenario}__{name}'
            np.savez_compressed(output / f'{stem}.npz', states=states, controls=controls)
            (output / f'{stem}_solver.json').write_text(json.dumps(diagnostic, indent=2))
            print(f'{scenario:13} {name:23} complete={metrics["completed"]}/4 '
                  f'q={metrics["min_q"]:.3f} collisions={metrics["collision_pair_steps"]} '
                  f'clear={metrics["clearance_s"]} runtime={metrics["runtime_s"]:.1f}s', flush=True)
            (output / 'summary.json').write_text(json.dumps(records, indent=2))
    fields = [k for k in records[0] if k not in ['settings', 'collision_pairs', 'exit_times_s']]
    with (output / 'summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(records)
    import scipy
    metadata = {'python': platform.python_version(), 'numpy': np.__version__,
                'projection_backend': PROJECTION_BACKEND,
                'scipy': scipy.__version__, 'dt': DT, 'maximum_duration_s': 24.,
                'scenario_order': ['staggered', 'simultaneous', 'offset'],
                'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in ROOT.glob('*.py')}}
    (output / 'reproducibility.json').write_text(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
