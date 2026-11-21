#!/usr/bin/env python3
"""Independent geometric and integration checks for the paper metrics."""
import unittest

import numpy as np

from analyze_formation_error import (aligned_squared_error, normalized_laplacian,
                                      spatial_average, synchronize)


class PaperFormationMetricTest(unittest.TestCase):
    def setUp(self):
        self.reference = np.array([[0.5, y, 0] for y in [-1.5, -0.5, 0.5, 1.5]]
                                  + [[-0.5, y, 0] for y in [-1.5, -0.5, 0.5, 1.5]])

    def test_translation_rotation_and_scale_invariance(self):
        rotation = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)
        actual = 2.7 * (self.reference @ rotation) + [13, -4, 7]
        sse, scale = aligned_squared_error(actual, self.reference)
        self.assertLess(float(sse), 1e-25)
        self.assertAlmostEqual(float(scale), 1 / 2.7)
        np.testing.assert_allclose(normalized_laplacian(actual), normalized_laplacian(self.reference), atol=1e-14)

    def test_local_deformation_and_fixed_correspondences(self):
        actual = self.reference.copy()
        actual[0] += [0.7, -0.2, 0.4]
        self.assertGreater(float(aligned_squared_error(actual, self.reference)[0]), 0.1)
        self.assertGreater(float(np.sum((normalized_laplacian(actual) - normalized_laplacian(self.reference)) ** 2)), 0.001)
        permuted = self.reference.copy()
        permuted[[0, 1]] = permuted[[1, 0]]
        self.assertGreater(float(aligned_squared_error(permuted, self.reference)[0]), 0.1)

    def test_reflection_is_not_a_proper_rotation(self):
        tetrahedron = np.array([[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3]], dtype=float)
        reflected = tetrahedron * [-1, 1, 1]
        self.assertGreater(float(aligned_squared_error(reflected, tetrahedron)[0]), 0.1)

    def test_batched_alignment_matches_single_frames(self):
        frames = np.stack([self.reference, self.reference * 3 + 4, self.reference.copy()])
        frames[2, 0, 0] += 1
        batched, scales = aligned_squared_error(frames, self.reference)
        for i in range(3):
            single, scale = aligned_squared_error(frames[i], self.reference)
            self.assertAlmostEqual(float(batched[i]), float(single))
            self.assertAlmostEqual(float(scales[i]), float(scale))

    def test_path_weighted_trapezoid_not_time_average(self):
        # Endpoint errors [0,2,8], path increments [1,3]: (1*1+5*3)/4 = 4.
        self.assertEqual(spatial_average(np.array([0, 2, 8]), np.array([1, 3]), 1), 4)
        self.assertEqual(spatial_average(np.array([0, 2, 8]), np.array([1, 3]), 2), 2)
        # No hidden division by eight or square root.
        self.assertEqual(spatial_average(np.array([8, 8, 8]), np.array([1, 3]), 1), 8)

    def test_interpolation_uses_timestamps_and_exact_endpoints(self):
        a = np.array([[10, 0, 0, 0], [10.1, 1, 0, 0], [10.2, 2, 0, 0]])
        b = np.array([[9.99, 0, 1, 0], [10.09, 1, 1, 0], [10.19, 2, 1, 0], [10.29, 3, 1, 0]])
        time, position, _ = synchronize([a, b], 10.02, 10.17, 20, max_gap=0.11)
        self.assertEqual(time[0], 0)
        self.assertAlmostEqual(time[-1], 0.15)
        self.assertAlmostEqual(position[0, 0, 0], 0.2)
        self.assertAlmostEqual(position[0, 1, 0], 0.3)
        with self.assertRaises(ValueError):
            synchronize([a, b], 10.02, 10.3, 20, max_gap=0.11)

    def test_collapsed_formations_are_rejected(self):
        with self.assertRaises(ValueError):
            aligned_squared_error(np.zeros((8, 3)), self.reference)
        with self.assertRaises(ValueError):
            normalized_laplacian(np.zeros((8, 3)))


if __name__ == '__main__':
    unittest.main()
