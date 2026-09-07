#!/usr/bin/env python3
"""Rank connected candidate bipartitions against offline motion labels."""

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any, Dict, List, Sequence, Set, Tuple

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from criterion.geometry import Pillar, Vec2  # noqa: E402
from criterion.partition import (  # noqa: E402
    LocalMotionOracle,
    candidate_bipartitions,
    feasible_direction_degrees,
)


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def grouped_rows(path: Path) -> Dict[int, List[Dict[str, str]]]:
    grouped = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            grouped[int(row["snapshot_id"])].append(row)
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
    if any(position is None for position in positions):
        raise ValueError("missing UAV position in snapshot")
    return positions


def partition_text(partition) -> str:
    return json.dumps(partition, separators=(",", ":"))


def intersect_direction_sets(
    members: Sequence[int], individual_sets: Sequence[Set[float]]
) -> Set[float]:
    result = set(individual_sets[members[0]])
    for member in members[1:]:
        result.intersection_update(individual_sets[member])
    return result


def edge_features(partition, edges, scores):
    first_group = set(partition[0])
    cut_values = []
    internal_values = []
    for edge in edges:
        canonical = tuple(sorted(edge))
        if (edge[0] in first_group) != (edge[1] in first_group):
            cut_values.append(scores[canonical])
        else:
            internal_values.append(scores[canonical])
    if not cut_values:
        raise ValueError("candidate bipartition has no cut edge")
    return {
        "mean_cut": sum(cut_values) / len(cut_values),
        "max_cut": max(cut_values),
        "mean_internal": sum(internal_values) / len(internal_values),
        "min_internal": min(internal_values),
        "cut_edge_count": len(cut_values),
    }


def sort_key(record, objective: str):
    features = record["features"]
    if objective == "mean_cut":
        return (
            features["mean_cut"],
            features["max_cut"],
            -features["mean_internal"],
            partition_text(record["partition"]),
        )
    if objective == "max_cut":
        return (
            features["max_cut"],
            features["mean_cut"],
            -features["mean_internal"],
            partition_text(record["partition"]),
        )
    raise ValueError(f"unsupported objective {objective}")


def primary_value(record, objective: str) -> float:
    return float(record["features"][objective])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formation", required=True, type=Path)
    parser.add_argument("--base-scenario", required=True, type=Path)
    parser.add_argument("--sweep", required=True, type=Path)
    parser.add_argument("--scores", required=True, type=Path)
    parser.add_argument("--csv-out", required=True, type=Path)
    parser.add_argument("--summary-out", type=Path)
    parser.add_argument(
        "--metric-fields",
        default="composite_min,forward_composite_min",
        help="comma-separated score columns to rank",
    )
    args = parser.parse_args()

    formation = load_yaml(args.formation)
    scenario = load_yaml(args.base_scenario)
    sweep = load_yaml(args.sweep)
    snapshots = grouped_rows(args.scores)
    num_uavs = int(formation["num_uavs"])
    edges = [tuple(map(int, edge)) for edge in formation["candidate_edges"]]
    partition_config = sweep["candidate_bipartitions"]
    partitions = candidate_bipartitions(
        num_uavs,
        edges,
        int(partition_config["minimum_group_size"]),
        bool(partition_config["require_induced_connectivity"]),
    )
    oracle_base = scenario["local_feasibility_oracle"]
    oracle_distances = [
        float(value) for value in sweep["oracle_translation_distances"]
    ]
    metric_fields = tuple(
        field.strip() for field in args.metric_fields.split(",") if field.strip()
    )
    if not metric_fields:
        raise ValueError("--metric-fields must contain at least one column")
    first_snapshot_rows = next(iter(snapshots.values()))
    missing_metrics = [
        field for field in metric_fields if field not in first_snapshot_rows[0]
    ]
    if missing_metrics:
        raise ValueError(f"score CSV is missing metric columns: {missing_metrics}")
    objectives = ("mean_cut", "max_cut")

    fieldnames = [
        "snapshot_id", "case_id", "pillar_radius", "pillar_y", "center_x",
        "oracle_translation_distance", "any_uav_in_obstacle", "intact_feasible",
        "candidate_partition_count", "valid_partition_count", "metric", "objective",
        "top_partition", "top_partition_valid", "top_group_direction_counts",
        "top_mean_cut", "top_max_cut", "top_mean_internal", "top_min_internal",
        "top_cut_edge_count", "top_primary_tie_count", "top_primary_valid_count",
        "top_primary_contains_valid", "top_primary_all_valid",
        "margin_to_next_primary_value", "best_valid_rank", "best_valid_partition",
        "top3_contains_valid",
    ]
    args.csv_out.parent.mkdir(parents=True, exist_ok=True)
    output_rows = []

    for snapshot_index, (snapshot_id, rows) in enumerate(snapshots.items(), start=1):
        if len(rows) != len(edges):
            raise ValueError(
                f"snapshot {snapshot_id}: expected {len(edges)} edge rows, got {len(rows)}"
            )
        positions = reconstruct_positions(rows, num_uavs)
        pillar = Pillar(
            Vec2(0.0, float(rows[0]["pillar_y"])),
            float(rows[0]["inflated_pillar_radius"]),
        )
        any_collision = any(
            bool(int(row["uav_i_in_obstacle"]))
            or bool(int(row["uav_j_in_obstacle"]))
            for row in rows
        )

        feature_cache = {}
        for metric in metric_fields:
            score_map = {
                tuple(sorted((int(row["uav_i"]), int(row["uav_j"])))): float(row[metric])
                for row in rows
            }
            feature_cache[metric] = {
                partition: edge_features(partition, edges, score_map)
                for partition in partitions
            }

        for distance in oracle_distances:
            oracle = LocalMotionOracle(
                distance,
                float(oracle_base["goal_heading_deg"]),
                float(oracle_base["direction_half_angle_deg"]),
                float(oracle_base["direction_step_deg"]),
            )
            individual_sets = [
                set(feasible_direction_degrees((uav_id,), positions, [pillar], oracle))
                for uav_id in range(num_uavs)
            ]
            intact_directions = intersect_direction_sets(
                tuple(range(num_uavs)), individual_sets
            )
            partition_directions = {}
            for partition in partitions:
                first_directions = intersect_direction_sets(partition[0], individual_sets)
                second_directions = intersect_direction_sets(partition[1], individual_sets)
                partition_directions[partition] = (
                    len(first_directions), len(second_directions)
                )

            valid_partitions = {
                partition
                for partition, counts in partition_directions.items()
                if counts[0] > 0 and counts[1] > 0
            }
            for metric in metric_fields:
                candidates = [
                    {"partition": partition, "features": feature_cache[metric][partition]}
                    for partition in partitions
                ]
                for objective in objectives:
                    ranked = sorted(candidates, key=lambda record: sort_key(record, objective))
                    top = ranked[0]
                    top_value = primary_value(top, objective)
                    primary_ties = [
                        record for record in ranked
                        if abs(primary_value(record, objective) - top_value) <= 1e-12
                    ]
                    primary_valid_count = sum(
                        record["partition"] in valid_partitions
                        for record in primary_ties
                    )
                    next_primary_values = [
                        primary_value(record, objective)
                        for record in ranked
                        if primary_value(record, objective) > top_value + 1e-12
                    ]
                    primary_margin = (
                        min(next_primary_values) - top_value
                        if next_primary_values else math.inf
                    )
                    best_valid_rank = 0
                    best_valid_partition = ""
                    for rank, record in enumerate(ranked, start=1):
                        if record["partition"] in valid_partitions:
                            best_valid_rank = rank
                            best_valid_partition = partition_text(record["partition"])
                            break
                    top_partition = top["partition"]
                    top_counts = partition_directions[top_partition]
                    features = top["features"]
                    output_rows.append({
                        "snapshot_id": snapshot_id,
                        "case_id": rows[0]["case_id"],
                        "pillar_radius": rows[0]["pillar_radius"],
                        "pillar_y": rows[0]["pillar_y"],
                        "center_x": rows[0]["center_x"],
                        "oracle_translation_distance": distance,
                        "any_uav_in_obstacle": int(any_collision),
                        "intact_feasible": int(bool(intact_directions)),
                        "candidate_partition_count": len(partitions),
                        "valid_partition_count": len(valid_partitions),
                        "metric": metric,
                        "objective": objective,
                        "top_partition": partition_text(top_partition),
                        "top_partition_valid": int(top_partition in valid_partitions),
                        "top_group_direction_counts": json.dumps(top_counts),
                        "top_mean_cut": features["mean_cut"],
                        "top_max_cut": features["max_cut"],
                        "top_mean_internal": features["mean_internal"],
                        "top_min_internal": features["min_internal"],
                        "top_cut_edge_count": features["cut_edge_count"],
                        "top_primary_tie_count": len(primary_ties),
                        "top_primary_valid_count": primary_valid_count,
                        "top_primary_contains_valid": int(primary_valid_count > 0),
                        "top_primary_all_valid": int(
                            primary_valid_count == len(primary_ties)
                        ),
                        "margin_to_next_primary_value": primary_margin,
                        "best_valid_rank": best_valid_rank,
                        "best_valid_partition": best_valid_partition,
                        "top3_contains_valid": int(any(
                            record["partition"] in valid_partitions for record in ranked[:3]
                        )),
                    })
        if snapshot_index % 100 == 0 or snapshot_index == len(snapshots):
            print(
                f"evaluated {snapshot_index}/{len(snapshots)} snapshots",
                flush=True,
            )

    with args.csv_out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output_rows)

    aggregate = {}
    for metric in metric_fields:
        for objective in objectives:
            for distance in oracle_distances:
                selected = [
                    row for row in output_rows
                    if row["metric"] == metric
                    and row["objective"] == objective
                    and float(row["oracle_translation_distance"]) == distance
                ]
                relevant = [
                    row for row in selected
                    if not row["any_uav_in_obstacle"]
                    and not row["intact_feasible"]
                    and row["valid_partition_count"] > 0
                ]
                blocked_without_partition = sum(
                    not row["any_uav_in_obstacle"]
                    and not row["intact_feasible"]
                    and row["valid_partition_count"] == 0
                    for row in selected
                )
                key = f"{metric}|{objective}|d={distance:.2f}"
                top1_count = sum(row["top_partition_valid"] for row in relevant)
                top3_count = sum(row["top3_contains_valid"] for row in relevant)
                top_primary_contains_count = sum(
                    row["top_primary_contains_valid"] for row in relevant
                )
                top_primary_all_count = sum(
                    row["top_primary_all_valid"] for row in relevant
                )
                ambiguous_top_count = sum(
                    row["top_primary_tie_count"] > 1 for row in relevant
                )
                ranks = [row["best_valid_rank"] for row in relevant]
                aggregate[key] = {
                    "collision_free_blocked_with_valid_partition": len(relevant),
                    "collision_free_blocked_without_valid_partition": blocked_without_partition,
                    "top1_valid_count": top1_count,
                    "top1_valid_rate": top1_count / len(relevant) if relevant else None,
                    "top3_contains_valid_count": top3_count,
                    "top3_contains_valid_rate": top3_count / len(relevant) if relevant else None,
                    "top_primary_contains_valid_count": top_primary_contains_count,
                    "top_primary_contains_valid_rate": (
                        top_primary_contains_count / len(relevant) if relevant else None
                    ),
                    "top_primary_all_valid_count": top_primary_all_count,
                    "top_primary_all_valid_rate": (
                        top_primary_all_count / len(relevant) if relevant else None
                    ),
                    "ambiguous_top_primary_count": ambiguous_top_count,
                    "median_best_valid_rank": statistics.median(ranks) if ranks else None,
                }

    summary = {
        "candidate_partition_count": len(partitions),
        "minimum_group_size": int(partition_config["minimum_group_size"]),
        "require_induced_connectivity": bool(
            partition_config["require_induced_connectivity"]
        ),
        "snapshots": len(snapshots),
        "oracle_translation_distances": oracle_distances,
        "metrics": list(metric_fields),
        "objectives": list(objectives),
        "rows": len(output_rows),
        "aggregate": aggregate,
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
