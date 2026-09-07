#!/usr/bin/env python3
"""Compute raw current-state overlap and visibility for a static pose scan."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Sequence

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from criterion.geometry import (  # noqa: E402
    Pillar,
    Vec2,
    build_star_space,
    forward_window_mask,
    pair_score,
    sampled_overlap,
    sample_grid_memberships,
)


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def scan_centres(start: Sequence[float], end: Sequence[float], step: float) -> Iterable[List[float]]:
    delta = [float(end[index]) - float(start[index]) for index in range(3)]
    distance = math.sqrt(sum(component * component for component in delta))
    segment_count = max(1, int(math.ceil(distance / step)))
    for index in range(segment_count + 1):
        alpha = index / segment_count
        yield [float(start[axis]) + alpha * delta[axis] for axis in range(3)]


def inflated_pillars(scenario: Dict[str, Any]) -> List[Pillar]:
    inflation = float(scenario["geometry"]["obstacle_inflation"])
    return [
        Pillar(
            Vec2(float(item["center"][0]), float(item["center"][1])),
            float(item["radius"]) + inflation,
        )
        for item in scenario["pillars"]
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formation", required=True, type=Path)
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--csv-out", required=True, type=Path)
    parser.add_argument("--summary-out", type=Path)
    args = parser.parse_args()

    formation = load_yaml(args.formation)
    scenario = load_yaml(args.scenario)
    geometry = scenario["geometry"]
    sensor_range = float(geometry["sensor_range"])
    ray_count = int(geometry["ray_count"])
    sample_resolution = float(geometry["area_sample_resolution"])
    narrow_clearance = float(geometry["narrow_clearance"])
    visibility_weights = {
        str(key): float(value)
        for key, value in geometry["visibility_weights"].items()
    }
    if set(visibility_weights) != {"VISIBLE", "NARROW", "OCCLUDED"}:
        raise ValueError("visibility_weights must define VISIBLE, NARROW and OCCLUDED")
    forward_window = geometry["forward_window"]

    pillars = inflated_pillars(scenario)
    offsets = formation["offsets"]
    edges = formation["candidate_edges"]
    scan = scenario["formation_center_scan"]
    centres = list(scan_centres(scan["start"], scan["end"], float(scan["step"])))

    fieldnames = [
        "scan_index", "center_x", "center_y", "center_z",
        "uav_i", "uav_j", "uav_i_x", "uav_i_y", "uav_j_x", "uav_j_y",
        "uav_i_in_obstacle", "uav_j_in_obstacle",
        "area_i", "area_j", "intersection_area", "union_area",
        "overlap_min", "overlap_iou", "clearance", "visibility",
        "visibility_weight", "composite_min", "composite_iou",
        "forward_area_i", "forward_area_j", "forward_intersection_area",
        "forward_union_area", "forward_overlap_min", "forward_overlap_iou",
        "forward_composite_min", "forward_composite_iou",
    ]
    args.csv_out.parent.mkdir(parents=True, exist_ok=True)
    rows_written = 0
    visibility_counts = {"VISIBLE": 0, "NARROW": 0, "OCCLUDED": 0}
    minimum_composite = math.inf
    maximum_composite = -math.inf
    minimum_forward_composite = math.inf
    maximum_forward_composite = -math.inf

    with args.csv_out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for scan_index, centre in enumerate(centres):
            positions = [
                Vec2(centre[0] + float(offset[0]), centre[1] + float(offset[1]))
                for offset in offsets
            ]
            spaces = [
                build_star_space(position, pillars, sensor_range, ray_count)
                for position in positions
            ]
            points, memberships, cell_area = sample_grid_memberships(
                spaces, sample_resolution
            )
            for first_raw, second_raw in edges:
                first, second = int(first_raw), int(second_raw)
                score = pair_score(
                    spaces[first], spaces[second],
                    memberships[first], memberships[second], cell_area,
                    pillars, narrow_clearance, visibility_weights,
                )
                midpoint = Vec2(
                    0.5 * (positions[first].x + positions[second].x),
                    0.5 * (positions[first].y + positions[second].y),
                )
                window_mask = forward_window_mask(
                    points,
                    midpoint,
                    float(forward_window["heading_deg"]),
                    float(forward_window["longitudinal_start"]),
                    float(forward_window["length"]),
                    float(forward_window["half_width"]),
                )
                forward_overlap = sampled_overlap(
                    memberships[first], memberships[second], cell_area, window_mask
                )
                forward_composite_min = (
                    forward_overlap.overlap_min * score.visibility_weight
                )
                forward_composite_iou = (
                    forward_overlap.overlap_iou * score.visibility_weight
                )
                writer.writerow({
                    "scan_index": scan_index,
                    "center_x": centre[0], "center_y": centre[1], "center_z": centre[2],
                    "uav_i": first, "uav_j": second,
                    "uav_i_x": positions[first].x, "uav_i_y": positions[first].y,
                    "uav_j_x": positions[second].x, "uav_j_y": positions[second].y,
                    "uav_i_in_obstacle": int(spaces[first].center_in_obstacle),
                    "uav_j_in_obstacle": int(spaces[second].center_in_obstacle),
                    "area_i": score.area_a, "area_j": score.area_b,
                    "intersection_area": score.intersection_area,
                    "union_area": score.union_area,
                    "overlap_min": score.overlap_min,
                    "overlap_iou": score.overlap_iou,
                    "clearance": score.clearance,
                    "visibility": score.visibility,
                    "visibility_weight": score.visibility_weight,
                    "composite_min": score.composite_min,
                    "composite_iou": score.composite_iou,
                    "forward_area_i": forward_overlap.area_a,
                    "forward_area_j": forward_overlap.area_b,
                    "forward_intersection_area": forward_overlap.intersection_area,
                    "forward_union_area": forward_overlap.union_area,
                    "forward_overlap_min": forward_overlap.overlap_min,
                    "forward_overlap_iou": forward_overlap.overlap_iou,
                    "forward_composite_min": forward_composite_min,
                    "forward_composite_iou": forward_composite_iou,
                })
                rows_written += 1
                visibility_counts[score.visibility] += 1
                minimum_composite = min(minimum_composite, score.composite_min)
                maximum_composite = max(maximum_composite, score.composite_min)
                minimum_forward_composite = min(
                    minimum_forward_composite, forward_composite_min
                )
                maximum_forward_composite = max(
                    maximum_forward_composite, forward_composite_min
                )

    summary = {
        "formation": formation["name"],
        "scenario": scenario["name"],
        "preview_horizon": 0.0,
        "decision_thresholds_applied": False,
        "scan_positions": len(centres),
        "candidate_edges": len(edges),
        "rows": rows_written,
        "visibility_counts": visibility_counts,
        "composite_min_range": [minimum_composite, maximum_composite],
        "forward_composite_min_range": [
            minimum_forward_composite, maximum_forward_composite
        ],
        "forward_window": forward_window,
        "csv": str(args.csv_out.resolve()),
    }
    if args.summary_out:
        args.summary_out.parent.mkdir(parents=True, exist_ok=True)
        with args.summary_out.open("w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
