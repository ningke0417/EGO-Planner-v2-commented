#!/usr/bin/env python3
"""Sweep graph thresholds against an independent local-motion label."""

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from criterion.geometry import Pillar, Vec2  # noqa: E402
from criterion.partition import (  # noqa: E402
    LocalMotionOracle,
    connected_components,
    feasible_direction_degrees,
)


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def threshold_values(start: float, end: float, step: float) -> Iterable[float]:
    if step <= 0.0 or end < start:
        raise ValueError("invalid partition threshold sweep")
    count = int(math.floor((end - start) / step + 1e-9))
    for index in range(count + 1):
        yield round(start + index * step, 12)
    if start + count * step < end - 1e-9:
        yield end


def load_score_rows(path: Path) -> Dict[int, List[Dict[str, str]]]:
    grouped: Dict[int, List[Dict[str, str]]] = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            grouped[int(row["scan_index"])].append(row)
    if not grouped:
        raise ValueError(f"{path}: no score rows")
    return dict(sorted(grouped.items()))


def reconstruct_positions(rows: Sequence[Dict[str, str]], num_uavs: int) -> List[Vec2]:
    positions: List[Any] = [None] * num_uavs
    for row in rows:
        for suffix in ("i", "j"):
            uav_id = int(row[f"uav_{suffix}"])
            point = Vec2(float(row[f"uav_{suffix}_x"]), float(row[f"uav_{suffix}_y"]))
            previous = positions[uav_id]
            if previous is not None and previous != point:
                raise ValueError(f"inconsistent position for UAV {uav_id}")
            positions[uav_id] = point
    if any(point is None for point in positions):
        raise ValueError("candidate graph does not expose every UAV position")
    return positions


def build_oracle(scenario: Dict[str, Any]) -> LocalMotionOracle:
    config = scenario["local_feasibility_oracle"]
    return LocalMotionOracle(
        translation_distance=float(config["translation_distance"]),
        goal_heading_deg=float(config["goal_heading_deg"]),
        direction_half_angle_deg=float(config["direction_half_angle_deg"]),
        direction_step_deg=float(config["direction_step_deg"]),
    )


def build_pillars(scenario: Dict[str, Any]) -> List[Pillar]:
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
    parser.add_argument("--scores", required=True, type=Path)
    parser.add_argument("--csv-out", required=True, type=Path)
    parser.add_argument("--summary-out", type=Path)
    parser.add_argument(
        "--score-field",
        choices=(
            "overlap_min", "overlap_iou", "composite_min", "composite_iou",
            "forward_overlap_min", "forward_overlap_iou",
            "forward_composite_min", "forward_composite_iou",
        ),
        help="override partition_threshold_sweep.score_field",
    )
    args = parser.parse_args()

    formation = load_yaml(args.formation)
    scenario = load_yaml(args.scenario)
    grouped_rows = load_score_rows(args.scores)
    num_uavs = int(formation["num_uavs"])
    edges = [tuple(map(int, edge)) for edge in formation["candidate_edges"]]
    sweep = scenario["partition_threshold_sweep"]
    score_field = args.score_field or str(sweep["score_field"])
    thresholds = list(threshold_values(
        float(sweep["start"]), float(sweep["end"]), float(sweep["step"])
    ))
    oracle = build_oracle(scenario)
    pillars = build_pillars(scenario)

    fieldnames = [
        "scan_index", "center_x", "center_y", "threshold", "score_field",
        "any_uav_in_obstacle", "intact_feasible", "intact_direction_count",
        "component_count", "components", "component_direction_counts",
        "all_components_individually_feasible", "singleton_count", "largest_component",
        "partition_locally_useful", "split_while_intact_motion_feasible",
    ]
    args.csv_out.parent.mkdir(parents=True, exist_ok=True)
    rows_written = 0
    intact_infeasible_positions = 0
    collision_positions = 0
    collision_free_intact_infeasible_positions = 0
    positions_with_useful_partition = set()
    threshold_stats = {
        threshold: {
            "useful_positions": 0,
            "split_while_intact_motion_feasible_positions": 0,
        }
        for threshold in thresholds
    }

    with args.csv_out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for scan_index, rows in grouped_rows.items():
            if len(rows) != len(edges):
                raise ValueError(
                    f"scan {scan_index}: expected {len(edges)} edges, got {len(rows)}"
                )
            positions = reconstruct_positions(rows, num_uavs)
            edge_scores: Dict[Tuple[int, int], float] = {}
            for row in rows:
                edge = tuple(sorted((int(row["uav_i"]), int(row["uav_j"]))))
                edge_scores[edge] = float(row[score_field])

            direction_cache: Dict[Tuple[int, ...], Tuple[float, ...]] = {}

            def directions(component: Tuple[int, ...]) -> Tuple[float, ...]:
                if component not in direction_cache:
                    direction_cache[component] = feasible_direction_degrees(
                        component, positions, pillars, oracle
                    )
                return direction_cache[component]

            intact = tuple(range(num_uavs))
            intact_directions = directions(intact)
            intact_feasible = bool(intact_directions)
            any_uav_in_obstacle = any(
                bool(int(row["uav_i_in_obstacle"]))
                or bool(int(row["uav_j_in_obstacle"]))
                for row in rows
            )
            if any_uav_in_obstacle:
                collision_positions += 1
            if not intact_feasible:
                intact_infeasible_positions += 1
                if not any_uav_in_obstacle:
                    collision_free_intact_infeasible_positions += 1

            centre_x = float(rows[0]["center_x"])
            centre_y = float(rows[0]["center_y"])
            for threshold in thresholds:
                components = connected_components(
                    num_uavs, edges, edge_scores, threshold
                )
                direction_counts = [len(directions(component)) for component in components]
                all_components_feasible = all(count > 0 for count in direction_counts)
                is_split = len(components) > 1
                useful = (not intact_feasible) and is_split and all_components_feasible
                split_while_intact_feasible = intact_feasible and is_split
                if useful:
                    positions_with_useful_partition.add(scan_index)
                    threshold_stats[threshold]["useful_positions"] += 1
                if split_while_intact_feasible:
                    threshold_stats[threshold][
                        "split_while_intact_motion_feasible_positions"
                    ] += 1

                writer.writerow({
                    "scan_index": scan_index,
                    "center_x": centre_x,
                    "center_y": centre_y,
                    "threshold": threshold,
                    "score_field": score_field,
                    "any_uav_in_obstacle": int(any_uav_in_obstacle),
                    "intact_feasible": int(intact_feasible),
                    "intact_direction_count": len(intact_directions),
                    "component_count": len(components),
                    "components": json.dumps(components, separators=(",", ":")),
                    "component_direction_counts": json.dumps(
                        direction_counts, separators=(",", ":")
                    ),
                    "all_components_individually_feasible": int(all_components_feasible),
                    "singleton_count": sum(len(component) == 1 for component in components),
                    "largest_component": max(map(len, components)),
                    "partition_locally_useful": int(useful),
                    "split_while_intact_motion_feasible": int(
                        split_while_intact_feasible
                    ),
                })
                rows_written += 1

    summary = {
        "score_field": score_field,
        "preview_horizon_for_score": 0.0,
        "oracle_is_offline_only": True,
        "oracle_translation_distance": oracle.translation_distance,
        "scan_positions": len(grouped_rows),
        "thresholds": thresholds,
        "rows": rows_written,
        "collision_positions": collision_positions,
        "intact_infeasible_positions": intact_infeasible_positions,
        "collision_free_intact_infeasible_positions": (
            collision_free_intact_infeasible_positions
        ),
        "positions_with_any_locally_useful_partition": len(positions_with_useful_partition),
        "useful_scan_indices": sorted(positions_with_useful_partition),
        "threshold_stats": {str(key): value for key, value in threshold_stats.items()},
        "partition_csv": str(args.csv_out.resolve()),
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
