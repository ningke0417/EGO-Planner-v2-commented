import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('paper_error', Path(__file__).resolve().parents[1]/'scripts/analyze_paper_formation_error.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class PaperErrorTests(unittest.TestCase):
    def setUp(self):
        self.ref = np.array([[x, y, 0] for x in [.5, -.5] for y in [-1.5, -.5, .5, 1.5]])

    def test_sim3_and_graph_invariance(self):
        angle = .78
        r = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        x = 2.5*(self.ref @ r)+np.array([10, -7, 4])
        residual, scale = m.similarity_residual(x, self.ref)
        self.assertAlmostEqual(float(residual), 0, places=20)
        self.assertAlmostEqual(float(scale), .4)
        np.testing.assert_allclose(m.normalized_laplacian(x), m.normalized_laplacian(self.ref), atol=1e-14)

    def test_no_reflection_in_3d(self):
        ref = np.array([[0, 0, 0], [2, 0, 0], [0, 3, 0], [0, 0, 4]], float)
        mirrored = ref*np.array([-1, 1, 1])
        residual, _ = m.similarity_residual(mirrored, ref)
        self.assertGreater(float(residual), 0.1)

    def test_squared_sum_not_rmse(self):
        x = self.ref.copy()
        x[:, 0] *= 2
        residual, scale = m.similarity_residual(x, self.ref)
        # ||X||^2=18, <X,Y>=14, ||Y||^2=12 => SSE=12-196/18.
        self.assertAlmostEqual(float(scale), 14/18)
        self.assertAlmostEqual(float(residual), 12-196/18)

    def test_path_weighting_not_time_weighting(self):
        t = np.array([0, 1, 2.])
        points = np.array([self.ref+[x, 0, 0] for x in [0, 1, 4]])
        arc, temporal, length = m.weighted_means(t, points, np.array([0, 0, 4.]), 1)
        self.assertEqual(length, 4)
        self.assertEqual(arc, 1.5)
        self.assertEqual(temporal, 1)

    def test_async_odom_interpolation_includes_endpoints(self):
        streams = [[(99.98, -0.02, 0, 0), (100.02, .02, 0, 0),
                    (100.08, .08, 0, 0), (100.12, .12, 0, 0)]]*8
        t, points, gap = m.sample_formation(streams, 100, 100.1, 25)
        self.assertEqual(t[0], 0)
        self.assertAlmostEqual(t[-1], .1)
        np.testing.assert_allclose(points[:, 0, 0], t, atol=1e-12)
        self.assertLess(gap, .1)

    def test_async_applied_swaps_follow_physical_agent(self):
        history = [[(0, i)] for i in range(8)]
        history[0].append((1, 4))
        history[4].append((1.2, 0))
        ref, slots, changes = m.applied_reference(self.ref, history, np.array([.5, 1.1, 1.3]))
        self.assertEqual(slots[:, 0].tolist(), [0, 4, 4])
        self.assertEqual(slots[:, 4].tolist(), [4, 4, 0])
        np.testing.assert_allclose(ref[2, 0], self.ref[4])
        residual, _ = m.similarity_residual(ref[2]+[10, 20, 3], ref[2])
        self.assertAlmostEqual(float(residual), 0, places=20)
        self.assertEqual(sum(changes), 2)


if __name__ == '__main__':
    unittest.main()
