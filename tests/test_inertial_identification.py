import copy
import unittest
import numpy as np
from forestlab.inertial_identification import *
from scipy.spatial.transform import Rotation

def rpy_rotation(degrees):
    return Rotation.from_euler("xyz",degrees,degrees=True).as_matrix()


class InertialIdentificationTests(unittest.TestCase):
    def test_parallel_axis_round_trip_and_physical_validation(self):
        m, c = 1.3, np.array([.1, -.07, .03])
        ic = np.array([[.08, .002, -.001], [.002, .12, .003], [-.001, .003, .16]])
        out = decode(parameters(m, c, ic))
        np.testing.assert_allclose(out['com'], c, atol=1e-15)
        np.testing.assert_allclose(out['inertia_com'], ic, atol=1e-15)
        self.assertTrue(out['physical'])
        self.assertFalse(decode(parameters(1., [0, 0, 0], np.diag([1., 1., 3.])))['physical'])

    def test_momentum_matches_direct_point_mass_integration(self):
        rng = np.random.default_rng(833)
        for _ in range(20):
            cells = rng.normal(size=(15, 3))*.3; weights = rng.uniform(.01, .1, 15)
            m = weights.sum(); c = weights@cells/m; rr = cells-c
            ic = np.eye(3)*np.sum(weights*np.sum(rr**2, axis=1))-(rr.T*weights)@rr
            theta = parameters(m, c, ic)
            p, v, w = rng.normal(size=(3, 3)); r = rpy_rotation(rng.uniform(-80, 80, 3))
            positions = p+cells@r.T; velocities = v+np.cross(w, cells@r.T)
            direct = np.r_[weights@velocities, (np.cross(positions, velocities)*weights[:, None]).sum(axis=0)]
            np.testing.assert_allclose(momentum_matrix(p, v, r, w)@theta, direct, atol=1e-14)

    def test_wrench_matches_direct_point_mass_accelerations(self):
        rng = np.random.default_rng(229)
        cells = rng.normal(size=(20, 3))*.3; weights = rng.uniform(.01, .1, 20)
        m = weights.sum(); c = weights@cells/m; rr = cells-c
        ic = np.eye(3)*np.sum(weights*np.sum(rr**2, axis=1))-(rr.T*weights)@rr
        r = rpy_rotation([20, 30, -40]); w = np.array([.3, -.7, .2]); alpha = np.array([.2, .1, -.5]); a = np.array([.6, -.2, .1])
        arms = cells@r.T
        forces = weights[:, None]*(a+[0, 0, 9.81]+np.cross(alpha, arms)+np.cross(w, np.cross(w, arms)))
        direct = np.r_[forces.sum(axis=0), np.cross(arms, forces).sum(axis=0)]
        np.testing.assert_allclose(wrench_matrix(r, w, a, alpha)@parameters(m, c, ic), direct, atol=1e-14)

    def hover_rows(self):
        anchors = np.array([[-.4, -.3, .2], [-.4, .3, .2], [.4, -.3, .2], [.4, .3, .2]])
        rows = []
        for t in np.arange(0., 2.0001, .01):
            rows.append(dict(time=t, position=np.array([0., 0., 3.]), velocity=np.zeros(3), rotation=np.eye(3),
                             omega=np.zeros(3), uav_position=np.array([0., 0., 3.])+anchors+[0., 0., 2.5],
                             tension=np.full(4, 1.2*9.81/4)))
        return anchors, rows

    def test_hover_refuses_full_identification_and_has_seven_null_directions(self):
        anchors, rows = self.hover_rows(); out = fit(rows, anchors)
        self.assertEqual(out['rank'], 3)
        self.assertEqual(out['nullspace_normalized'].shape, (7, 10))
        self.assertFalse(out['activated']); self.assertIsNone(out['decoded'])

    def test_hidden_labels_rejected_and_inputs_unmodified(self):
        anchors, rows = self.hover_rows(); before = copy.deepcopy(rows)
        fit(rows, anchors)
        for a, b in zip(rows, before):
            for k in a:
                np.testing.assert_array_equal(a[k], b[k])
        rows[0]['mass'] = 1.2
        with self.assertRaises(ValueError):
            fit(rows, anchors)

    def test_noisy_hover_is_not_mistaken_for_rich_excitation(self):
        anchors, rows = self.hover_rows(); rng = np.random.default_rng(778)
        for row in rows:
            row['omega'] = rng.uniform(-.001, .001, 3)
            row['velocity'] = rng.uniform(-.001, .001, 3)
        out = fit(rows, anchors)
        self.assertFalse(out['excitation_passed']); self.assertFalse(out['activated'])

    def test_online_equals_offline_and_rejects_duplicate_time(self):
        anchors, rows = self.hover_rows(); online = OnlineIdentifier(anchors)
        for row in rows:
            online.observe(row)
        a, b = online.estimate(), fit(rows, anchors)
        np.testing.assert_allclose(a['system']['A'], b['system']['A'], atol=1e-14)
        np.testing.assert_allclose(a['system']['b'], b['system']['b'], atol=1e-14)
        np.testing.assert_allclose(a['theta'], b['theta'], atol=1e-12)
        with self.assertRaises(ValueError):
            online.observe(rows[-1])

    def test_zero_tension_sensor_floor_refuses_identification(self):
        anchors, rows = self.hover_rows()
        rows[17]['tension'][2] = 0.
        offline = fit(rows, anchors, tension_floor=.01)
        online = OnlineIdentifier(anchors, tension_floor=.01)
        for row in rows:
            online.observe(row)
        causal = online.estimate()
        self.assertFalse(offline['tension_gate_passed'])
        self.assertFalse(offline['activated'])
        self.assertEqual(offline['activation_refusal'], 'tension_at_or_below_sensor_error_bound')
        self.assertFalse(causal['tension_gate_passed'])
        self.assertFalse(causal['activated'])

    def test_wrench_changes_with_unknown_center_and_inertia(self):
        base = parameters(1.2, [0, 0, 0], np.diag([.08, .1, .13]))
        shifted = parameters(1.2, [.1, 0, .04], np.diag([.12, .09, .18]))
        mat = wrench_matrix(np.eye(3), [.2, .1, -.3], [.4, 0, 0], [.3, .2, .1])
        self.assertGreater(np.linalg.norm((mat@(base-shifted))[3:]), 1.)


if __name__ == '__main__':
    unittest.main()
