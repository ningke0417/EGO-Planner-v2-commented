"""Graph partition and independent local-feasibility labelling."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Sequence, Tuple

from .geometry import Pillar, Vec2, segment_clearance


Edge = Tuple[int, int]
Partition = Tuple[Tuple[int, ...], Tuple[int, ...]]


@dataclass(frozen=True)
class LocalMotionOracle:
    translation_distance: float
    goal_heading_deg: float
    direction_half_angle_deg: float
    direction_step_deg: float

    def candidate_angles_deg(self) -> List[float]:
        if self.translation_distance <= 0.0:
            raise ValueError("translation_distance must be positive")
        if not 0.0 < self.direction_half_angle_deg < 90.0:
            raise ValueError("direction_half_angle_deg must lie between 0 and 90")
        if self.direction_step_deg <= 0.0:
            raise ValueError("direction_step_deg must be positive")
        count = int(math.floor(
            2.0 * self.direction_half_angle_deg / self.direction_step_deg
        ))
        return [
            self.goal_heading_deg - self.direction_half_angle_deg
            + index * self.direction_step_deg
            for index in range(count + 1)
        ]


def connected_components(
    num_uavs: int,
    edges: Sequence[Edge],
    edge_scores: Dict[Edge, float],
    threshold: float,
) -> Tuple[Tuple[int, ...], ...]:
    """Cut edges whose score is strictly lower than `threshold`."""
    adjacency = [set() for _ in range(num_uavs)]
    for raw_first, raw_second in edges:
        edge = tuple(sorted((raw_first, raw_second)))
        if edge not in edge_scores:
            raise ValueError(f"missing score for edge {edge}")
        if edge_scores[edge] >= threshold:
            adjacency[raw_first].add(raw_second)
            adjacency[raw_second].add(raw_first)

    remaining = set(range(num_uavs))
    components = []
    while remaining:
        root = min(remaining)
        reached = {root}
        frontier = [root]
        remaining.remove(root)
        while frontier:
            current = frontier.pop()
            for neighbour in sorted(adjacency[current]):
                if neighbour in reached:
                    continue
                reached.add(neighbour)
                remaining.discard(neighbour)
                frontier.append(neighbour)
        components.append(tuple(sorted(reached)))
    return tuple(sorted(components))


def induced_group_connected(members: Iterable[int], edges: Sequence[Edge]) -> bool:
    member_set = set(members)
    if not member_set:
        return False
    adjacency = {member: set() for member in member_set}
    for first, second in edges:
        if first in member_set and second in member_set:
            adjacency[first].add(second)
            adjacency[second].add(first)
    root = min(member_set)
    reached = {root}
    frontier = [root]
    while frontier:
        current = frontier.pop()
        for neighbour in adjacency[current] - reached:
            reached.add(neighbour)
            frontier.append(neighbour)
    return reached == member_set


def candidate_bipartitions(
    num_uavs: int,
    edges: Sequence[Edge],
    minimum_group_size: int,
    require_induced_connectivity: bool,
) -> Tuple[Partition, ...]:
    """Enumerate each unordered bipartition once by fixing UAV 0 in group A."""
    if num_uavs < 2:
        raise ValueError("num_uavs must exceed one")
    if minimum_group_size < 1 or 2 * minimum_group_size > num_uavs:
        raise ValueError("minimum_group_size is incompatible with num_uavs")

    all_members = set(range(num_uavs))
    partitions = []
    for mask in range(1 << num_uavs):
        if not mask & 1:
            continue
        first_group = tuple(index for index in range(num_uavs) if mask & (1 << index))
        second_group = tuple(sorted(all_members - set(first_group)))
        if min(len(first_group), len(second_group)) < minimum_group_size:
            continue
        if require_induced_connectivity and (
            not induced_group_connected(first_group, edges)
            or not induced_group_connected(second_group, edges)
        ):
            continue
        partitions.append((first_group, second_group))
    return tuple(partitions)


def feasible_direction_degrees(
    members: Iterable[int],
    positions: Sequence[Vec2],
    pillars: Sequence[Pillar],
    oracle: LocalMotionOracle,
) -> Tuple[float, ...]:
    member_ids = tuple(members)
    if not member_ids:
        raise ValueError("a group must contain at least one UAV")

    feasible = []
    for angle_deg in oracle.candidate_angles_deg():
        angle = math.radians(angle_deg)
        displacement = Vec2(
            oracle.translation_distance * math.cos(angle),
            oracle.translation_distance * math.sin(angle),
        )
        safe = True
        for member in member_ids:
            start = positions[member]
            end = Vec2(start.x + displacement.x, start.y + displacement.y)
            if segment_clearance(start, end, pillars) <= 0.0:
                safe = False
                break
        if safe:
            feasible.append(angle_deg)
    return tuple(feasible)
