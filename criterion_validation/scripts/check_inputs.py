#!/usr/bin/env python3
"""Validate the isolated criterion experiment inputs.

This script intentionally performs no star-space calculation and makes no
split/merge decision.  Its only job is to fail early on ambiguous geometry or
an invalid candidate graph before those inputs are used in an experiment.
"""

import argparse
import math
from pathlib import Path
from typing import Any, Dict, List

import yaml


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def finite_vector(value: Any, length: int, label: str) -> List[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label}: expected a list of length {length}")
    vector = [float(item) for item in value]
    if not all(math.isfinite(item) for item in vector):
        raise ValueError(f"{label}: all values must be finite")
    return vector


def validate_formation(data: Dict[str, Any]) -> None:
    num_uavs = int(data["num_uavs"])
    offsets = data.get("offsets")
    if num_uavs <= 1:
        raise ValueError("formation.num_uavs must exceed one")
    if not isinstance(offsets, list) or len(offsets) != num_uavs:
        raise ValueError("formation.offsets must contain one entry per UAV")
    for index, offset in enumerate(offsets):
        finite_vector(offset, 3, f"formation.offsets[{index}]")

    seen = set()
    edges = data.get("candidate_edges")
    if not isinstance(edges, list) or not edges:
        raise ValueError("formation.candidate_edges must be a non-empty list")
    adjacency = [set() for _ in range(num_uavs)]
    for index, edge in enumerate(edges):
        pair = finite_vector(edge, 2, f"formation.candidate_edges[{index}]")
        first, second = int(pair[0]), int(pair[1])
        if pair != [float(first), float(second)]:
            raise ValueError(f"formation.candidate_edges[{index}] must use integer IDs")
        if first == second or min(first, second) < 0 or max(first, second) >= num_uavs:
            raise ValueError(f"formation.candidate_edges[{index}] has invalid UAV IDs")
        canonical = tuple(sorted((first, second)))
        if canonical in seen:
            raise ValueError(f"formation.candidate_edges contains duplicate {canonical}")
        seen.add(canonical)
        adjacency[first].add(second)
        adjacency[second].add(first)

    reached = {0}
    frontier = [0]
    while frontier:
        current = frontier.pop()
        for neighbour in adjacency[current] - reached:
            reached.add(neighbour)
            frontier.append(neighbour)
    if len(reached) != num_uavs:
        raise ValueError("formation.candidate_edges must initially form one connected graph")


def validate_scenario(data: Dict[str, Any]) -> None:
    pillars = data.get("pillars")
    if not isinstance(pillars, list) or not pillars:
        raise ValueError("scenario.pillars must be a non-empty list")
    for index, pillar in enumerate(pillars):
        if not isinstance(pillar, dict):
            raise ValueError(f"scenario.pillars[{index}] must be a mapping")
        finite_vector(pillar.get("center"), 2, f"scenario.pillars[{index}].center")
        radius = float(pillar["radius"])
        ground_z = float(pillar["ground_z"])
        height = float(pillar["height"])
        if radius <= 0.0 or height <= ground_z:
            raise ValueError(f"scenario.pillars[{index}] has invalid dimensions")

    scan = data.get("formation_center_scan")
    if not isinstance(scan, dict):
        raise ValueError("scenario.formation_center_scan must be a mapping")
    start = finite_vector(scan.get("start"), 3, "scenario.formation_center_scan.start")
    end = finite_vector(scan.get("end"), 3, "scenario.formation_center_scan.end")
    step = float(scan["step"])
    if step <= 0.0 or start == end:
        raise ValueError("formation-center scan needs a positive step and distinct endpoints")

    geometry = data.get("geometry")
    if not isinstance(geometry, dict):
        raise ValueError("scenario.geometry must be a mapping")
    for key in ("sensor_range", "obstacle_inflation", "area_sample_resolution"):
        if float(geometry[key]) <= 0.0:
            raise ValueError(f"scenario.geometry.{key} must be positive")
    if int(geometry["ray_count"]) < 8:
        raise ValueError("scenario.geometry.ray_count must be at least 8")
    if float(geometry["narrow_clearance"]) < 0.0:
        raise ValueError("scenario.geometry.narrow_clearance cannot be negative")
    forward_window = geometry.get("forward_window")
    if not isinstance(forward_window, dict):
        raise ValueError("scenario.geometry.forward_window must be a mapping")
    if float(forward_window["length"]) <= 0.0:
        raise ValueError("scenario.geometry.forward_window.length must be positive")
    if float(forward_window["half_width"]) <= 0.0:
        raise ValueError("scenario.geometry.forward_window.half_width must be positive")

    oracle = data.get("local_feasibility_oracle")
    if not isinstance(oracle, dict):
        raise ValueError("scenario.local_feasibility_oracle must be a mapping")
    if float(oracle["translation_distance"]) <= 0.0:
        raise ValueError("local_feasibility_oracle.translation_distance must be positive")
    half_angle = float(oracle["direction_half_angle_deg"])
    if not 0.0 < half_angle < 90.0:
        raise ValueError("local_feasibility_oracle.direction_half_angle_deg must be in (0, 90)")
    if float(oracle["direction_step_deg"]) <= 0.0:
        raise ValueError("local_feasibility_oracle.direction_step_deg must be positive")

    sweep = data.get("partition_threshold_sweep")
    if not isinstance(sweep, dict):
        raise ValueError("scenario.partition_threshold_sweep must be a mapping")
    if sweep["score_field"] not in {
        "overlap_min", "overlap_iou", "composite_min", "composite_iou",
        "forward_overlap_min", "forward_overlap_iou",
        "forward_composite_min", "forward_composite_iou",
    }:
        raise ValueError("partition_threshold_sweep.score_field is unsupported")
    sweep_start = float(sweep["start"])
    sweep_end = float(sweep["end"])
    sweep_step = float(sweep["step"])
    if sweep_step <= 0.0 or sweep_end < sweep_start:
        raise ValueError("partition_threshold_sweep range is invalid")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formation", required=True, type=Path)
    parser.add_argument("--scenario", required=True, type=Path)
    args = parser.parse_args()

    formation = load_yaml(args.formation)
    scenario = load_yaml(args.scenario)
    validate_formation(formation)
    validate_scenario(scenario)

    print("INPUTS_OK")
    print(f"formation={formation['name']} uavs={formation['num_uavs']} "
          f"candidate_edges={len(formation['candidate_edges'])}")
    print(f"scenario={scenario['name']} pillars={len(scenario['pillars'])}")
    print("operational_decision=none score_preview_horizon=0.0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
