"""Centralized MPC with analytic gradients and projected gradient descent."""
from dataclasses import dataclass, asdict
import numpy as np
from model import DT, HORIZON, HEADINGS, PAIRS, pair_matrix
try:
    from numba import njit
    PROJECTION_BACKEND = 'numba'
except ImportError:
    PROJECTION_BACKEND = 'numpy'
    def njit(**kwargs):
        return lambda function: function


@njit(cache=True)
def dykstra(target, V, speed0, a_min, a_max, v_max):
    """Same projection with optional JIT acceleration; NumPy fallback is valid."""
    n = len(target)
    x = target.copy()
    corrections = np.zeros((1+2*n, n, 4))
    for cycle in range(100):
        old = x.copy()
        y = x + corrections[0]
        x = np.minimum(np.maximum(y, a_min), a_max)
        corrections[0] = y-x
        for k in range(n):
            row = V[k]
            norm = np.sum(row*row)
            for side in range(2):
                idx = 1+2*k+side
                y = x + corrections[idx]
                v = speed0 + row @ y
                violation = v-v_max if side == 0 else -v
                adjustment = np.maximum(violation, 0.)/norm
                x = y - (1 if side == 0 else -1)*row[:, None]*adjustment
                corrections[idx] = y-x
        v = speed0 + V @ x
        violation = max(np.max(x-a_max), np.max(a_min-x),
                        np.max(v-v_max), np.max(-v), 0.)
        if np.max(np.abs(x-old)) < 1e-9 and violation < 1e-8:
            break
    return x


@dataclass
class Settings:
    potential: str = 'softplus'
    obstacle_weight: float = 100.
    acceleration_weight: float = .15
    jerk_weight: float = .10
    speed_weight: float = 1.
    beta: float = 5.
    iterations: int = 100
    tolerance: float = 1e-4
    a_min: float = -3.
    a_max: float = 2.
    v_max: float = 10.
    desired_speed: float = 8.


def potential(q, kind, beta=5.):
    """Return phi(q), dphi/dq. All candidates have phi(1)=1.

    Gaussian tail decays; reciprocal has a long tail; squared softplus focuses
    on penetration of the ellipse, with smooth rather than singular gradients.
    """
    if kind == 'gaussian':
        value = np.exp(1-q)
        return value, -value
    if kind == 'reciprocal':
        value = 1.05 / (q + .05)
        return value, -1.05 / (q + .05)**2
    if kind == 'softplus':
        z = beta * (1-q)
        sp = np.logaddexp(0, z)
        sigmoid = np.exp(-np.logaddexp(0, -z))
        norm = np.log(2)**2
        return sp**2 / norm, -2 * beta * sp * sigmoid / norm
    raise ValueError(kind)


class Controller:
    def __init__(self, settings=None, horizon=HORIZON):
        self.settings = settings or Settings()
        self.n = horizon
        rows, cols = np.indices((horizon, horizon))
        self.P = np.where(cols <= rows, (rows-cols+.5)*DT**2, 0.)
        self.V = np.tril(np.ones((horizon, horizon))) * DT
        self.Q = {pair: pair_matrix(*pair) for pair in PAIRS}
        self.guess = np.zeros((horizon, 4))

    def objective(self, a, state, previous, gradient=True):
        s = self.settings
        speed0 = np.sum(state[:, 2:] * HEADINGS, axis=1)
        speed = speed0 + self.V @ a
        pos = (state[None, :, :2] + np.arange(1, self.n+1)[:, None, None]
               * DT * state[None, :, 2:] + (self.P @ a)[..., None] * HEADINGS)
        error = speed - s.desired_speed
        delta = np.diff(a, axis=0, prepend=previous[None])
        value = s.speed_weight*np.sum(error**2)
        value += s.acceleration_weight*np.sum(a**2)
        value += s.jerk_weight*np.sum((delta / DT)**2)
        g = 2*s.speed_weight*self.V.T @ error + 2*s.acceleration_weight*a
        jerk_g = 2*s.jerk_weight*delta/DT**2
        g += jerk_g
        g[:-1] -= jerk_g[1:]
        gp = np.zeros_like(pos)
        for i, j in PAIRS:
            r = pos[:, i] - pos[:, j]
            rq = r @ self.Q[i, j]
            q = np.sum(rq*r, axis=1)
            phi, derivative = potential(q, s.potential, s.beta)
            value += s.obstacle_weight*np.sum(phi)
            force = 2*s.obstacle_weight*derivative[:, None]*rq
            gp[:, i] += force
            gp[:, j] -= force
        g += self.P.T @ np.sum(gp*HEADINGS, axis=-1)
        return float(value), g

    def project(self, target, speed0):
        """Euclidean projection by Dykstra onto accel box and speed halfspaces.

        Horizon velocities are affine in acceleration. Intersection is convex.
        Corrections are retained within each projection, never across targets.
        Return residual and reject any inaccurate projection in the optimizer.
        """
        s = self.settings
        clipped = np.clip(target, s.a_min, s.a_max)
        if self.violation(clipped, speed0) < 1e-10:
            return clipped
        return dykstra(target, self.V, speed0, s.a_min, s.a_max, s.v_max)

    def violation(self, a, speed0):
        s = self.settings
        v = speed0 + self.V @ a
        return max(float(np.max(a-s.a_max)), float(np.max(s.a_min-a)),
                   float(np.max(v-s.v_max)), float(np.max(-v)), 0.)

    def solve(self, state, previous):
        s = self.settings
        speed0 = np.sum(state[:, 2:]*HEADINGS, axis=1)
        a = self.project(self.guess, speed0)
        # Zero is always feasible when current speed is feasible.
        if self.violation(a, speed0) > 1e-7:
            a = np.zeros_like(a)
        value, g = self.objective(a, state, previous)
        trace = [value]
        status = 'iteration_limit'
        residual = np.inf
        for iteration in range(s.iterations):
            alpha = .15
            accepted = False
            for _ in range(22):
                candidate = self.project(a-alpha*g, speed0)
                direction = candidate-a
                residual = float(np.linalg.norm(direction)/alpha)
                trial, _ = self.objective(candidate, state, previous)
                if (self.violation(candidate, speed0) < 1e-7 and
                        trial <= value + 1e-4*np.sum(g*direction) + 1e-10):
                    accepted = True
                    break
                alpha *= .5
            if not accepted:
                status = 'line_search_failed'
                break
            a, value = candidate, trial
            trace.append(value)
            _, g = self.objective(a, state, previous)
            if residual < s.tolerance:
                status = 'stationary'
                break
        self.guess = np.vstack((a[1:], a[-1:]))
        return a[0], {'status': status, 'iterations': iteration+1,
                      'projected_residual': residual, 'cost_trace': trace,
                      'constraint_violation': self.violation(a, speed0)}
