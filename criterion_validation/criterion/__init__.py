"""Geometry primitives for the isolated split-criterion experiment."""

from .geometry import (
    PairScore,
    OverlapScore,
    Pillar,
    StarSpace,
    Vec2,
    build_star_space,
    pair_score,
    forward_window_mask,
    sampled_overlap,
    sample_grid_memberships,
    sample_memberships,
    segment_clearance,
)
from .partition import (
    LocalMotionOracle,
    candidate_bipartitions,
    connected_components,
    feasible_direction_degrees,
)

__all__ = [
    "PairScore",
    "OverlapScore",
    "Pillar",
    "StarSpace",
    "Vec2",
    "build_star_space",
    "pair_score",
    "forward_window_mask",
    "sampled_overlap",
    "sample_grid_memberships",
    "sample_memberships",
    "segment_clearance",
    "LocalMotionOracle",
    "candidate_bipartitions",
    "connected_components",
    "feasible_direction_degrees",
]
