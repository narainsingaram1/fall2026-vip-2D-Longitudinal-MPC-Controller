"""Independent numerical checks; run: python3 -m unittest discover -s ..."""
import unittest
import numpy as np
from scipy.optimize import minimize, LinearConstraint
from controller import Controller, Settings, potential
from model import DT, HEADINGS, predict, step, initial_state, swept_pair


class ExperimentTests(unittest.TestCase):
    def test_four_state_dynamics(self):
        state = np.array([[1., 2., 3., -4.]])
        acceleration = np.array([[2., -1.]])
        np.testing.assert_allclose(step(state, acceleration, .5), [[2.75, -.125, 4., -4.5]])
        result = predict(state, np.array([acceleration]), .5)
        self.assertEqual(len(result), 4)
        np.testing.assert_allclose(np.stack(result, axis=-1)[-1], step(state, acceleration, .5))

    def test_full_objective_gradients(self):
        rng = np.random.default_rng(42)
        state = initial_state('staggered')
        # Bring vehicles close enough that obstacle gradients are substantial.
        state[:, :2] += 18*HEADINGS
        a = rng.uniform(-.5, .5, (6, 4))
        previous = rng.uniform(-.3, .3, 4)
        for kind in ['softplus', 'gaussian', 'reciprocal']:
            controller = Controller(Settings(potential=kind), horizon=6)
            _, analytical = controller.objective(a, state, previous)
            numerical = np.zeros_like(a)
            for index in np.ndindex(a.shape):
                plus, minus = a.copy(), a.copy()
                plus[index] += 1e-5
                minus[index] -= 1e-5
                numerical[index] = (controller.objective(plus, state, previous)[0]
                                    - controller.objective(minus, state, previous)[0])/2e-5
            np.testing.assert_allclose(analytical, numerical, rtol=2e-5, atol=1e-5)

    def test_projection_against_independent_qp(self):
        rng = np.random.default_rng(7)
        controller = Controller(horizon=6)
        target = rng.normal(0, 5, (6, 4))
        speed = np.array([.1, 9.9, 5., 1.])
        projected = controller.project(target, speed)
        for i in range(4):
            oracle = minimize(lambda x: .5*np.sum((x-target[:, i])**2),
                              np.zeros(6), jac=lambda x: x-target[:, i],
                              method='SLSQP', bounds=[(-3, 2)]*6,
                              constraints=[LinearConstraint(controller.V, -speed[i], 10-speed[i])],
                              options={'ftol': 1e-12, 'maxiter': 300})
            self.assertTrue(oracle.success, oracle.message)
            np.testing.assert_allclose(projected[:, i], oracle.x, atol=2e-6)
        self.assertLess(controller.violation(projected, speed), 1e-7)

    def test_continuous_collision_between_endpoints(self):
        state = np.zeros((4, 4))
        state[0] = [-10, 0, 100, 0]
        state[1] = [10, 0, -100, 0]
        q, collision = swept_pair(state, np.zeros((4, 2)), 0, 1)
        self.assertTrue(collision)
        self.assertAlmostEqual(q, 0., places=8)
        # At both endpoints centers are 20 m apart: endpoint-only tests miss it.
        self.assertGreater(abs(state[0, 0]-state[1, 0]), 4.5)
        end = step(state, np.zeros((4, 2)))
        self.assertGreater(abs(end[0, 0]-end[1, 0]), 4.5)

    def test_parallel_lanes_are_not_same_position(self):
        state = initial_state('simultaneous')
        state[0, :2], state[1, :2] = [0, -1.75], [0, 1.75]
        q, collision = swept_pair(state, np.zeros((4, 2)), 0, 1)
        self.assertFalse(collision)
        self.assertGreater(q, 1.)

    def test_potential_normalization_and_sign(self):
        for kind in ['softplus', 'gaussian', 'reciprocal']:
            values, derivative = potential(np.array([0., 1., 2.]), kind)
            self.assertAlmostEqual(values[1], 1.)
            self.assertTrue(np.all(np.diff(values) < 0))
            self.assertTrue(np.all(derivative < 0))

    def test_optional_projection_backend_matches_numpy(self):
        from controller import dykstra
        if not hasattr(dykstra, 'py_func'):
            self.skipTest('Optional Numba absent; NumPy is active')
        controller = Controller(horizon=6)
        target = np.random.default_rng(12).normal(0, 3, (6, 4))
        args = (target, controller.V, np.array([.1, 9.9, 2., 5.]), -3., 2., 10.)
        np.testing.assert_allclose(dykstra(*args), dykstra.py_func(*args), atol=1e-10)

    def test_closed_loop_and_descent_trace(self):
        from run_experiments import simulate
        metrics, states, controls, diagnostics = simulate(Settings(iterations=60), 'staggered')
        self.assertEqual(metrics['completed'], 4)
        self.assertEqual(metrics['collision_pair_steps'], 0)
        self.assertGreaterEqual(metrics['min_q'], 1.)
        self.assertGreaterEqual(metrics['min_speed'], -1e-7)
        self.assertLessEqual(metrics['max_speed'], 10+1e-7)
        self.assertLessEqual(np.max(controls), 2+1e-7)
        self.assertGreaterEqual(np.min(controls), -3-1e-7)
        for diagnostic in diagnostics:
            self.assertTrue(np.all(np.diff(diagnostic['cost_trace']) <= 1e-8))
        np.testing.assert_allclose(states[1:], step(states[:-1], controls[..., None]*HEADINGS), atol=1e-10)


if __name__ == '__main__':
    unittest.main()
