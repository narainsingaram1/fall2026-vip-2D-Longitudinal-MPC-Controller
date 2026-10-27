"""Generate presentation figures, GIF and measured-results Markdown table."""
import os
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR', '/tmp/vip-elliptical-matplotlib')
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/vip-elliptical-cache')
import json
import csv
import hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation, PillowWriter
from controller import potential
from model import DT, PAIRS, HALF, HEADINGS, pair_matrix, swept_pair

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'results'
COLORS = ['#2563eb', '#db2777', '#059669', '#d97706']
LABELS = ['1 · west → east', '2 · east → west', '3 · south → north', '4 · north → south']
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
                     'figure.facecolor': 'white', 'axes.grid': True, 'grid.alpha': .18})


def load(scenario, config):
    data = np.load(OUT / f'{scenario}__{config}.npz')
    return data['states'], data['controls']


def candidates():
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), layout='constrained')
    q = np.linspace(0, 4, 500)
    x, y = np.meshgrid(np.linspace(-12, 12, 180), np.linspace(-7, 7, 140))
    Q = pair_matrix(0, 1)
    fieldq = Q[0, 0]*x*x + Q[1, 1]*y*y
    for col, kind in enumerate(['gaussian', 'reciprocal', 'softplus']):
        phi, _ = potential(q, kind)
        axes[0, col].plot(q, phi, color=COLORS[col], lw=2)
        axes[0, col].axvline(1, color='black', ls='--', lw=1)
        axes[0, col].set(title=kind.capitalize(), xlabel='q (dimensionless)', ylabel='φ(q)', ylim=(0, 22))
        axes[0, col].text(.98, .95, f'φ(0)={phi[0]:.2f}\nφ(2)={potential(np.array(2.), kind)[0]:.5f}',
                          transform=axes[0, col].transAxes, ha='right', va='top', fontsize=9)
        value, _ = potential(fieldq, kind)
        levels = [.01, .1, .5, 1, 2, 5, 10]
        contours = axes[1, col].contour(x, y, value, levels=levels, cmap='viridis')
        axes[1, col].clabel(contours, fontsize=7, fmt='%g')
        axes[1, col].contour(x, y, fieldq, levels=[1], colors='red', linewidths=2)
        axes[1, col].set(aspect='equal', xlabel='Relative x [m]', ylabel='Relative y [m]')
    fig.suptitle('Candidate potentials · all normalized to φ(1)=1\nRed: pair exclusion ellipse for parallel vehicle headings', fontsize=15)
    fig.savefig(OUT / '01_potential_comparison.png', dpi=170)
    plt.close(fig)


def dashboard(scenario='offset', config='softplus'):
    states, controls = load(scenario, config)
    t = np.arange(len(states))*DT
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), layout='constrained')
    for i in range(4):
        axes[0, 0].plot(states[:, i, 0], states[:, i, 1], color=COLORS[i], label=LABELS[i])
        axes[0, 0].scatter(*states[0, i, :2], color=COLORS[i], marker='o')
        axes[0, 1].plot(t, states[:, i, 2:] @ HEADINGS[i], color=COLORS[i])
        axes[0, 2].step(t[:-1], controls[:, i], where='post', color=COLORS[i])
        jerk = np.diff(controls[:, i], prepend=0)/DT
        axes[1, 0].step(t[:-1], jerk, where='post', color=COLORS[i])
    axes[0, 0].set(title='World-coordinate paths (dots = starts)', xlabel='x [m]', ylabel='y [m]', aspect='equal')
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].set(title='Speed', xlabel='Time [s]', ylabel='m/s')
    axes[0, 2].set(title='Applied acceleration', xlabel='Time [s]', ylabel='m/s²')
    axes[0, 2].axhline(-3, color='black', ls='--', lw=.8)
    axes[0, 2].axhline(2, color='black', ls='--', lw=.8)
    axes[1, 0].set(title='Jerk (includes initial a=0 → first command)', xlabel='Time [s]', ylabel='m/s³')
    for i, j in PAIRS:
        q = [swept_pair(state, a[:, None]*HEADINGS, i, j)[0]
             for state, a in zip(states[:-1], controls)]
        axes[1, 1].plot(t[:-1], q, label=f'{i+1}–{j+1}')
    axes[1, 1].axhline(1, color='red', ls='--', label='buffer boundary')
    axes[1, 1].set(title='Continuous-in-step pair separation', xlabel='Time [s]', ylabel='Minimum q per interval', ylim=(0, 6))
    axes[1, 1].legend(ncol=2, fontsize=8)
    diagnostic = json.loads((OUT / f'{scenario}__{config}_solver.json').read_text())
    idx = max(range(len(diagnostic)), key=lambda k: diagnostic[k]['cost_trace'][0]-diagnostic[k]['cost_trace'][-1])
    trace = diagnostic[idx]['cost_trace']
    axes[1, 2].plot(trace, color='#7c3aed')
    axes[1, 2].set(title=f'One MPC solve at t={idx*DT:.1f} s · {diagnostic[idx]["status"]}', xlabel='Accepted gradient update', ylabel='Objective J')
    fig.suptitle(f'{scenario.capitalize()} arrivals · {config} · measured closed-loop behavior', fontsize=16)
    fig.savefig(OUT / f'02_dashboard_{scenario}.png', dpi=170)
    plt.close(fig)


def comparison(records):
    fig, axes = plt.subplots(1, 3, figsize=(17, 7), layout='constrained')
    names = list(dict.fromkeys(r['configuration'] for r in records))
    scenarios = list(dict.fromkeys(r['scenario'] for r in records))
    for k, scenario in enumerate(scenarios):
        rows = [next(r for r in records if r['configuration']==name and r['scenario']==scenario) for name in names]
        y = np.arange(len(names)) + (k-1)*.24
        for ax, metric in zip(axes, ['crossing_min_q', 'clearance_s', 'jerk_rms']):
            values = [r[metric] if r[metric] is not None else np.nan for r in rows]
            ax.barh(y, values, height=.23, label=scenario, color=COLORS[k])
            if metric == 'clearance_s':
                for yy, r in zip(y, rows):
                    if r[metric] is None:
                        ax.text(0, yy, f'  {r["completed"]}/4 completed at 24 s', va='center', fontsize=7)
    for ax in axes:
        ax.set_yticks(np.arange(len(names)), names)
        ax.invert_yaxis()
    axes[0].axvline(1, color='red', ls='--')
    axes[0].set(title='Minimum crossing-pair q · red = buffer boundary', xlabel='q (perpendicular pairs only)')
    axes[1].set(title='Clearance time · blank = incomplete', xlabel='s')
    axes[2].set(title='RMS jerk · lower = smoother commands', xlabel='m/s³')
    axes[2].legend()
    fig.suptitle('24 controlled comparisons · 8 configurations × 3 scenarios', fontsize=16)
    fig.savefig(OUT / '03_experiment_comparison.png', dpi=170)
    plt.close(fig)


def animate(scenario='offset', config='softplus'):
    states, controls = load(scenario, config)
    fig, ax = plt.subplots(figsize=(7, 7), layout='constrained')
    ax.set(xlim=(-45, 45), ylim=(-45, 45), aspect='equal', xlabel='x [m]', ylabel='y [m]')
    ax.axhspan(-3.5, 3.5, color='#e5e7eb', zorder=0)
    ax.axvspan(-3.5, 3.5, color='#e5e7eb', zorder=0)
    bodies, trails = [], []
    for i in range(4):
        body = Rectangle((0, 0), * (2*HALF[i]), color=COLORS[i])
        ax.add_patch(body)
        bodies.append(body)
        trails.append(ax.plot([], [], color=COLORS[i], alpha=.45)[0])
    title = ax.set_title('')
    def update(k):
        for i in range(4):
            bodies[i].set_xy(states[k, i, :2]-HALF[i])
            trails[i].set_data(states[:k+1, i, 0], states[:k+1, i, 1])
        title.set_text(f'Fixed-lane planar MPC · {scenario}\nt={k*DT:.1f} s · playback at simulation speed')
        return [*bodies, *trails, title]
    animation = FuncAnimation(fig, update, frames=len(states), interval=DT*1000, blit=False)
    animation.save(OUT / f'04_{scenario}_softplus.gif', writer=PillowWriter(fps=5), dpi=100)
    plt.close(fig)


def result_table(records):
    lines = ['# Measured results', '',
             'Generated from `results/summary.json`. Clearance is the first sample when every center has passed its route coordinate +12 m; maximum run duration is 24 s.', '',
             '| Scenario | Configuration | Complete | Clearance s | Min q, all | Min q, crossing | Body overlap pair-steps | Accel RMS m/s² | Jerk RMS m/s³ | Iteration-limit solves |',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in records:
        clearance = 'incomplete' if r['clearance_s'] is None else f'{r["clearance_s"]:.1f}'
        lines.append(f'| {r["scenario"]} | {r["configuration"]} | {r["completed"]}/4 | {clearance} | {r["min_q"]:.3f} | {r["crossing_min_q"]:.3f} | {r["collision_pair_steps"]} | {r["acceleration_rms"]:.3f} | {r["jerk_rms"]:.3f} | {r["iteration_limit_solves"]}/{r["solves"]} |')
    lines += ['', 'A pair-step counts one overlapping vehicle pair during one executed 0.2 s interval. It is not a count of distinct crash events. `q<1` is a buffer violation; body overlap is checked separately using swept rectangles.', '',
              'No-obstacle uses the same new dynamics, optimizer and comfort terms with obstacle weight zero. It is an ablation, not a rerun of the original FIFO/BFGS simulator.', '',
              'The all-pair minimum can be dominated by opposing parallel lanes: their lateral gap sets a floor of about q=1.158 when their x/y coordinates align. The crossing-only minimum excludes parallel pairs so the effects of changing the obstacle weight remain visible. It is independently recomputed from saved trajectories and saved in `results/crossing_separation.csv`.', '',
              'RMS metrics are pooled over all four vehicles and every applied command until each run ends. Different run durations mean these are run-level comfort summaries, not matched-duration causal estimates. No physical energy consumption is computed.']
    (ROOT / 'RESULTS.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    records = json.loads((OUT / 'summary.json').read_text())
    for record in records:
        states, controls = load(record['scenario'], record['configuration'])
        record['crossing_min_q'] = min(
            swept_pair(state, a[:, None]*HEADINGS, i, j)[0]
            for state, a in zip(states[:-1], controls) for i, j in PAIRS
            if abs(HEADINGS[i] @ HEADINGS[j]) < .1)
    with (OUT / 'crossing_separation.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['scenario', 'configuration', 'crossing_min_q'], extrasaction='ignore')
        writer.writeheader()
        writer.writerows(records)
    candidates()
    for scenario in ['staggered', 'simultaneous', 'offset']:
        dashboard(scenario)
    comparison(records)
    animate()
    result_table(records)
    (OUT / 'showcase_manifest.json').write_text(json.dumps({
        'showcase_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'artifacts_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in OUT.iterdir() if p.suffix in ['.png', '.gif']},
    }, indent=2))
    print('Figures, GIF and RESULTS.md generated.')
