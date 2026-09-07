#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from criterion.geometry import Pillar, Vec2  # noqa: E402
from criterion.partition import (  # noqa: E402
    LocalMotionOracle,
    candidate_bipartitions,
    connected_components,
    feasible_direction_degrees,
)


class PartitionTest(unittest.TestCase):
    def setUp(self) -> None:
        self.oracle = LocalMotionOracle(0.5, 0.0, 80.0, 2.0)
        self.pillar = Pillar(Vec2(0.0, 0.0), 1.35)

    def test_threshold_cuts_only_low_score_edge(self) -> None:
        edges = [(0, 1), (1, 2)]
        scores = {(0, 1): 0.8, (1, 2): 0.2}
        self.assertEqual(
            connected_components(3, edges, scores, 0.5),
            ((0, 1), (2,)),
        )

    def test_connected_bipartitions_exclude_scattered_groups(self) -> None:
        partitions = candidate_bipartitions(
            4, [(0, 1), (1, 2), (2, 3)], 2, True
        )
        self.assertEqual(partitions, (((0, 1), (2, 3)),))

    def test_symmetric_pair_has_no_common_forward_escape(self) -> None:
        positions = [Vec2(-1.4, -0.5), Vec2(-1.4, 0.5)]
        directions = feasible_direction_degrees(
            (0, 1), positions, [self.pillar], self.oracle
        )
        self.assertEqual(directions, ())

    def test_split_singletons_each_have_an_escape_direction(self) -> None:
        positions = [Vec2(-1.4, -0.5), Vec2(-1.4, 0.5)]
        lower = feasible_direction_degrees((0,), positions, [self.pillar], self.oracle)
        upper = feasible_direction_degrees((1,), positions, [self.pillar], self.oracle)
        self.assertTrue(lower)
        self.assertTrue(upper)
        self.assertLess(max(lower), 0.0)
        self.assertGreater(min(upper), 0.0)


if __name__ == "__main__":
    unittest.main()
