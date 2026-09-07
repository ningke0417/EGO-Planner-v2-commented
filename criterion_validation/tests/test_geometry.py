#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from criterion.geometry import (  # noqa: E402
    Pillar,
    Vec2,
    build_star_space,
    forward_window_mask,
    pair_score,
    ray_distance,
    sample_memberships,
    sampled_overlap,
    segment_clearance,
)


class GeometryTest(unittest.TestCase):
    def setUp(self) -> None:
        self.pillar = Pillar(Vec2(0.0, 0.0), 1.0)

    def test_ray_hits_near_surface(self) -> None:
        distance = ray_distance(Vec2(-3.0, 0.0), 0.0, [self.pillar], 5.0)
        self.assertAlmostEqual(distance, 2.0, places=9)

    def test_ray_away_from_pillar_reaches_sensor_range(self) -> None:
        distance = ray_distance(Vec2(-3.0, 0.0), math.pi, [self.pillar], 5.0)
        self.assertAlmostEqual(distance, 5.0, places=9)

    def test_center_inside_pillar_has_zero_radii(self) -> None:
        space = build_star_space(Vec2(0.0, 0.0), [self.pillar], 5.0, 32)
        self.assertTrue(space.center_in_obstacle)
        self.assertEqual(set(space.radii), {0.0})

    def test_segment_through_pillar_is_occluded(self) -> None:
        clearance = segment_clearance(Vec2(-2.0, 0.0), Vec2(2.0, 0.0), [self.pillar])
        self.assertAlmostEqual(clearance, -1.0, places=9)

    def test_identical_spaces_have_unit_overlap(self) -> None:
        space = build_star_space(Vec2(-3.0, 0.0), [self.pillar], 5.0, 32)
        memberships, cell_area = sample_memberships([space, space], 0.25)
        score = pair_score(
            space, space, memberships[0], memberships[1], cell_area,
            [self.pillar], 0.6,
            {"VISIBLE": 1.0, "NARROW": 0.45, "OCCLUDED": 0.0},
        )
        self.assertAlmostEqual(score.overlap_min, 1.0, places=12)
        self.assertAlmostEqual(score.overlap_iou, 1.0, places=12)

    def test_forward_window_excludes_points_behind_pair(self) -> None:
        points = [Vec2(-0.1, 0.0), Vec2(0.1, 0.0), Vec2(2.9, 1.9), Vec2(3.1, 0.0)]
        mask = forward_window_mask(points, Vec2(0.0, 0.0), 0.0, 0.0, 3.0, 2.0)
        self.assertEqual(mask, [False, True, True, False])

    def test_masked_overlap_uses_only_forward_cells(self) -> None:
        overlap = sampled_overlap(
            [True, True, False], [False, True, True], 0.25,
            [False, True, True],
        )
        self.assertAlmostEqual(overlap.area_a, 0.25)
        self.assertAlmostEqual(overlap.area_b, 0.50)
        self.assertAlmostEqual(overlap.overlap_min, 1.0)


if __name__ == "__main__":
    unittest.main()
