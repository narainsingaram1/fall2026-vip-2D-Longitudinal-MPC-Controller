"""Planar constant-acceleration dynamics and independent swept geometry checks."""
from itertools import combinations
import numpy as np

DT = 0.2
HORIZON = 20
LENGTH, WIDTH = 4.5, 1.8
HEADINGS = np.array([[1., 0.], [-1., 0.], [0., 1.], [0., -1.]])
LANES = np.array([[0., -1.75], [0., 1.75], [1.75, 0.], [-1.75, 0.]])
HALF = np.array([[LENGTH, WIDTH], [LENGTH, WIDTH],
                 [WIDTH, LENGTH], [WIDTH, LENGTH]]) / 2
PAIRS = list(combinations(range(4), 2))


def step(state, acceleration, dt=DT):
    """State [...,4]=(x,y,vx,vy); acceleration [...,2]. No hidden clipping."""
    return np.concatenate((state[..., :2] + state[..., 2:] * dt
                           + .5 * acceleration * dt**2,
                           state[..., 2:] + acceleration * dt), axis=-1)


def predict(state, acceleration, dt=DT):
    """Return FOUR arrays: x, y, vx, vy, including initial state."""
    trajectory = [np.array(state, copy=True)]
    for a in acceleration:
        trajectory.append(step(trajectory[-1], a, dt))
    return tuple(np.stack(trajectory)[..., k] for k in range(4))


def initial_state(scenario):
    distances, speeds = {
        "staggered": ([-24, -30, -36, -42], [7, 7.5, 8, 8]),
        "simultaneous": ([-28, -28, -28, -28], [7, 7, 7, 7]),
        "offset": ([-25, -32, -29, -38], [6, 8, 7.5, 7]),
    }[scenario]
    return np.column_stack((LANES + np.array(distances)[:, None] * HEADINGS,
                            np.array(speeds)[:, None] * HEADINGS))


def pair_matrix(i, j, margin=0.5):
    """Outer ellipse enclosing the pair's axis-aligned collision rectangle.

    q>=1 is sufficient separation for these fixed headings. q<1 is only
    a buffer violation, not necessarily physical overlap. Margin expands
    each pair exclusion half-extent by 0.5 m, not each individual vehicle.
    """
    axes = np.sqrt(2) * (HALF[i] + HALF[j] + margin)
    return np.diag(1 / axes**2)


def real_roots(coeff, end):
    coeff = np.trim_zeros(np.asarray(coeff), 'b')
    if len(coeff) < 2:
        return []
    return [float(r.real) for r in np.polynomial.polynomial.polyroots(coeff)
            if abs(r.imag) < 1e-7 and 0 < r.real < end]


def swept_pair(state, acceleration, i, j, dt=DT):
    """Continuous-in-step minimum q and rectangle overlap, via polynomial roots.

    Fixed axis-aligned bodies and constant acceleration make relative position
    quadratic and q quartic. This avoids missing collisions between samples.
    """
    coefficients = np.stack((state[i, :2] - state[j, :2],
                             state[i, 2:] - state[j, 2:],
                             .5 * (acceleration[i] - acceleration[j])))
    Q = pair_matrix(i, j)
    qcoeff = np.zeros(5)
    for k in range(2):
        term = Q[k, k] * np.polynomial.polynomial.polymul(
            coefficients[:, k], coefficients[:, k])
        qcoeff[:len(term)] += term
    stationary = real_roots(np.arange(1, 5) * qcoeff[1:], dt)
    qmin = min(np.polynomial.polynomial.polyval(t, qcoeff)
               for t in [0., dt, *stationary])
    bounds = HALF[i] + HALF[j]
    events = [0., dt]
    for k in range(2):
        for sign in [-1, 1]:
            poly = coefficients[:, k].copy()
            poly[0] -= sign * bounds[k]
            events.extend(real_roots(poly, dt))
    events = sorted(events)
    samples = events + [(a+b)/2 for a, b in zip(events[:-1], events[1:])]
    collision = any(np.all(np.abs(np.polynomial.polynomial.polyval(t, coefficients))
                               <= bounds + 1e-8) for t in samples)
    return float(qmin), bool(collision)
