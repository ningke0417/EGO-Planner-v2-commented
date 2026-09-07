"""Current-state 2-D radial free-space and pair-relation calculations.

This module intentionally implements the small analytic prototype, not the
3-D QuickHull-based StarConvex algorithm from the separate StarConvex project.
Keeping the implementation explicit lets us validate the overlap/visibility
hypothesis before deciding whether the full 3-D construction is necessary.
"""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Sequence, Tuple


TAU = 2.0 * math.pi


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float


@dataclass(frozen=True)
class Pillar:
    center: Vec2
    radius: float


@dataclass(frozen=True)
class StarSpace:
    center: Vec2
    sensor_range: float
    radii: Tuple[float, ...]
    center_in_obstacle: bool

    def radius_at(self, theta: float) -> float:
        wrapped = theta % TAU
        scaled = wrapped / TAU * len(self.radii)
        lower_floor = math.floor(scaled)
        lower = int(lower_floor) % len(self.radii)
        upper = (lower + 1) % len(self.radii)
        alpha = scaled - lower_floor
        return self.radii[lower] * (1.0 - alpha) + self.radii[upper] * alpha

    def contains(self, point: Vec2) -> bool:
        dx = point.x - self.center.x
        dy = point.y - self.center.y
        distance = math.hypot(dx, dy)
        if distance > self.sensor_range:
            return False
        if distance < 1e-12:
            return not self.center_in_obstacle
        return distance <= self.radius_at(math.atan2(dy, dx))


@dataclass(frozen=True)
class PairScore:
    area_a: float
    area_b: float
    intersection_area: float
    union_area: float
    overlap_min: float
    overlap_iou: float
    clearance: float
    visibility: str
    visibility_weight: float
    composite_min: float
    composite_iou: float


@dataclass(frozen=True)
class OverlapScore:
    area_a: float
    area_b: float
    intersection_area: float
    union_area: float
    overlap_min: float
    overlap_iou: float


def squared_distance(first: Vec2, second: Vec2) -> float:
    dx = first.x - second.x
    dy = first.y - second.y
    return dx * dx + dy * dy


def ray_distance(
    origin: Vec2,
    theta: float,
    pillars: Sequence[Pillar],
    sensor_range: float,
) -> float:
    direction_x = math.cos(theta)
    direction_y = math.sin(theta)
    best = sensor_range
    for pillar in pillars:
        relative_x = origin.x - pillar.center.x
        relative_y = origin.y - pillar.center.y
        constant = (
            relative_x * relative_x
            + relative_y * relative_y
            - pillar.radius * pillar.radius
        )
        if constant <= 0.0:
            return 0.0

        linear = 2.0 * (
            relative_x * direction_x + relative_y * direction_y
        )
        discriminant = linear * linear - 4.0 * constant
        if discriminant < 0.0:
            continue

        square_root = math.sqrt(discriminant)
        near = (-linear - square_root) * 0.5
        far = (-linear + square_root) * 0.5
        hit = near if near >= 0.0 else far
        if 0.0 <= hit < best:
            best = hit
    return best


def build_star_space(
    center: Vec2,
    pillars: Sequence[Pillar],
    sensor_range: float,
    ray_count: int,
) -> StarSpace:
    if sensor_range <= 0.0:
        raise ValueError("sensor_range must be positive")
    if ray_count < 8:
        raise ValueError("ray_count must be at least 8")
    center_in_obstacle = any(
        squared_distance(center, pillar.center) <= pillar.radius * pillar.radius
        for pillar in pillars
    )
    radii = tuple(
        ray_distance(center, TAU * index / ray_count, pillars, sensor_range)
        for index in range(ray_count)
    )
    return StarSpace(center, sensor_range, radii, center_in_obstacle)


def sample_grid_memberships(
    spaces: Sequence[StarSpace], resolution: float
) -> Tuple[List[Vec2], List[List[bool]], float]:
    """Sample all spaces on one common cell-centre grid.

    A common grid avoids giving different formation edges different sampling
    phases.  Returned area estimates use `cell_count * resolution**2`.
    """
    if not spaces:
        raise ValueError("at least one star space is required")
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")
    sensor_range = spaces[0].sensor_range
    if any(abs(space.sensor_range - sensor_range) > 1e-12 for space in spaces):
        raise ValueError("all star spaces must use one sensor range")

    minimum_x = min(space.center.x for space in spaces) - sensor_range
    maximum_x = max(space.center.x for space in spaces) + sensor_range
    minimum_y = min(space.center.y for space in spaces) - sensor_range
    maximum_y = max(space.center.y for space in spaces) + sensor_range
    count_x = int(math.ceil((maximum_x - minimum_x) / resolution))
    count_y = int(math.ceil((maximum_y - minimum_y) / resolution))

    points: List[Vec2] = []
    memberships: List[List[bool]] = [[] for _ in spaces]
    for index_x in range(count_x):
        x = minimum_x + (index_x + 0.5) * resolution
        for index_y in range(count_y):
            y = minimum_y + (index_y + 0.5) * resolution
            point = Vec2(x, y)
            points.append(point)
            for index, space in enumerate(spaces):
                memberships[index].append(space.contains(point))
    return points, memberships, resolution * resolution


def sample_memberships(
    spaces: Sequence[StarSpace], resolution: float
) -> Tuple[List[List[bool]], float]:
    _, memberships, cell_area = sample_grid_memberships(spaces, resolution)
    return memberships, cell_area


def forward_window_mask(
    points: Sequence[Vec2],
    origin: Vec2,
    heading_deg: float,
    longitudinal_start: float,
    length: float,
    half_width: float,
) -> List[bool]:
    if length <= 0.0 or half_width <= 0.0:
        raise ValueError("forward window length and half_width must be positive")
    heading = math.radians(heading_deg)
    forward_x = math.cos(heading)
    forward_y = math.sin(heading)
    left_x = -forward_y
    left_y = forward_x
    longitudinal_end = longitudinal_start + length
    mask = []
    for point in points:
        relative_x = point.x - origin.x
        relative_y = point.y - origin.y
        longitudinal = relative_x * forward_x + relative_y * forward_y
        lateral = relative_x * left_x + relative_y * left_y
        mask.append(
            longitudinal_start <= longitudinal <= longitudinal_end
            and abs(lateral) <= half_width
        )
    return mask


def sampled_overlap(
    membership_a: Sequence[bool],
    membership_b: Sequence[bool],
    cell_area: float,
    mask: Sequence[bool] = None,
) -> OverlapScore:
    if len(membership_a) != len(membership_b):
        raise ValueError("pair memberships must share one sampling grid")
    if mask is None:
        mask = [True] * len(membership_a)
    if len(mask) != len(membership_a):
        raise ValueError("overlap mask must share the membership grid")

    count_a = 0
    count_b = 0
    intersection_count = 0
    for in_window, in_a, in_b in zip(mask, membership_a, membership_b):
        if not in_window:
            continue
        count_a += int(in_a)
        count_b += int(in_b)
        intersection_count += int(in_a and in_b)
    union_count = count_a + count_b - intersection_count
    smaller = min(count_a, count_b)
    return OverlapScore(
        area_a=count_a * cell_area,
        area_b=count_b * cell_area,
        intersection_area=intersection_count * cell_area,
        union_area=union_count * cell_area,
        overlap_min=intersection_count / smaller if smaller else 0.0,
        overlap_iou=intersection_count / union_count if union_count else 0.0,
    )


def segment_clearance(first: Vec2, second: Vec2, pillars: Iterable[Pillar]) -> float:
    delta_x = second.x - first.x
    delta_y = second.y - first.y
    length_squared = delta_x * delta_x + delta_y * delta_y
    clearance = math.inf
    for pillar in pillars:
        if length_squared > 1e-12:
            projection = (
                (pillar.center.x - first.x) * delta_x
                + (pillar.center.y - first.y) * delta_y
            ) / length_squared
            projection = min(1.0, max(0.0, projection))
        else:
            projection = 0.0
        closest = Vec2(
            first.x + projection * delta_x,
            first.y + projection * delta_y,
        )
        clearance = min(
            clearance,
            math.sqrt(squared_distance(closest, pillar.center)) - pillar.radius,
        )
    return clearance


def visibility_class(clearance: float, narrow_clearance: float) -> str:
    if clearance <= 0.0:
        return "OCCLUDED"
    if clearance < narrow_clearance:
        return "NARROW"
    return "VISIBLE"


def pair_score(
    space_a: StarSpace,
    space_b: StarSpace,
    membership_a: Sequence[bool],
    membership_b: Sequence[bool],
    cell_area: float,
    pillars: Sequence[Pillar],
    narrow_clearance: float,
    visibility_weights: Dict[str, float],
) -> PairScore:
    overlap = sampled_overlap(membership_a, membership_b, cell_area)

    clearance = segment_clearance(space_a.center, space_b.center, pillars)
    visibility = visibility_class(clearance, narrow_clearance)
    weight = float(visibility_weights[visibility])
    return PairScore(
        area_a=overlap.area_a,
        area_b=overlap.area_b,
        intersection_area=overlap.intersection_area,
        union_area=overlap.union_area,
        overlap_min=overlap.overlap_min,
        overlap_iou=overlap.overlap_iou,
        clearance=clearance,
        visibility=visibility,
        visibility_weight=weight,
        composite_min=overlap.overlap_min * weight,
        composite_iou=overlap.overlap_iou * weight,
    )
