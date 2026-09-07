#!/usr/bin/env python3
"""Sweep a current-state split/merge state machine over spatial scan cases."""

import argparse
from collections import defaultdict
import csv
import itertools
import json
import math
from pathlib import Path
import statistics
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from criterion.partition import candidate_bipartitions  # noqa: E402


def load_yaml(path):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def grouped_csv(path):
    grouped = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            grouped[int(row["snapshot_id"])].append(row)
    return grouped


def inclusive_values(start, end, step):
    count = int(math.floor((end - start) / step + 1e-9))
    return [round(start + index * step, 12) for index in range(count + 1)]


def partition_text(partition):
    return json.dumps(partition, separators=(",", ":"))


def partition_features(partition, edges, score_map):
    first_group = set(partition[0])
    cut = []
    internal = []
    for edge in edges:
        value = score_map[tuple(sorted(edge))]
        if (edge[0] in first_group) != (edge[1] in first_group):
            cut.append(value)
        else:
            internal.append(value)
    return {
        "mean_cut": statistics.mean(cut),
        "max_cut": max(cut),
        "mean_internal": statistics.mean(internal),
    }


def best_partition(snapshot_rows, metric, partitions, edges):
    scores = {
        tuple(sorted((int(row["uav_i"]), int(row["uav_j"])))): float(row[metric])
        for row in snapshot_rows
    }
    records = []
    for partition in partitions:
        features = partition_features(partition, edges, scores)
        records.append((
            features["mean_cut"], features["max_cut"],
            -features["mean_internal"], partition_text(partition),
            partition, features,
        ))
    return min(records)[4:]


def case_labels(ranking_path, oracle_distance, metric):
    labels = {}
    with ranking_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["metric"] != metric or row["objective"] != "mean_cut":
                continue
            if abs(float(row["oracle_translation_distance"]) - oracle_distance) > 1e-9:
                continue
            labels[int(row["snapshot_id"])] = {
                "collision": bool(int(row["any_uav_in_obstacle"])),
                "intact_feasible": bool(int(row["intact_feasible"])),
                "valid_partition_count": int(row["valid_partition_count"]),
            }
    return labels


def infer_step(snapshots):
    differences = [
        second["x"] - first["x"]
        for first, second in zip(snapshots, snapshots[1:])
        if second["x"] > first["x"] + 1e-9
    ]
    return min(differences)


def simulate(case, split_threshold, merge_threshold, internal_threshold,
             persistence_distance):
    snapshots = case["snapshots"]
    nominal_step = infer_step(snapshots)
    maximum_contiguous_gap = 1.5 * nominal_step + 1e-9
    state = "INTACT"
    split_start = None
    split_previous = None
    merge_start = None
    merge_previous = None
    active_partition = None
    split_x = None
    merge_x = None

    for snapshot in snapshots:
        x = snapshot["x"]
        if state == "INTACT":
            split_condition = (
                snapshot["best_features"]["mean_cut"] <= split_threshold
                and snapshot["best_features"]["mean_internal"] >= internal_threshold
            )
            if split_condition:
                if split_start is None or x - split_previous > maximum_contiguous_gap:
                    split_start = x
                split_previous = x
                if x - split_start + 1e-9 >= persistence_distance:
                    state = "SPLIT"
                    split_x = x
                    active_partition = snapshot["best_partition"]
                    merge_start = None
                    merge_previous = None
            else:
                split_start = None
                split_previous = None
        else:
            active_features = snapshot["partition_features"][active_partition]
            merge_condition = active_features["mean_cut"] >= merge_threshold
            if merge_condition:
                if merge_start is None or x - merge_previous > maximum_contiguous_gap:
                    merge_start = x
                merge_previous = x
                if x - merge_start + 1e-9 >= persistence_distance:
                    merge_x = x
                    break
            else:
                merge_start = None
                merge_previous = None

    blocked_x = case["first_blocked_x"]
    detected_before_block = blocked_x is not None and split_x is not None and split_x <= blocked_x + 1e-9
    return {
        "split_x": split_x,
        "merge_x": merge_x,
        "partition": partition_text(active_partition) if active_partition else "",
        "detected_before_block": detected_before_block,
        "lead_distance": blocked_x - split_x if detected_before_block else None,
        "premature_merge": bool(merge_x is not None and blocked_x is not None and merge_x <= blocked_x),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formation", required=True, type=Path)
    parser.add_argument("--scores", required=True, type=Path)
    parser.add_argument("--rankings", required=True, type=Path)
    parser.add_argument("--metric", default="real_composite_min")
    parser.add_argument("--oracle-distance", default=0.5, type=float)
    parser.add_argument("--csv-out", required=True, type=Path)
    parser.add_argument("--summary-out", required=True, type=Path)
    parser.add_argument("--events-out", type=Path)
    parser.add_argument("--fixed-split-threshold", type=float)
    parser.add_argument("--fixed-merge-threshold", type=float)
    parser.add_argument("--fixed-internal-threshold", type=float)
    parser.add_argument("--fixed-persistence-distance", type=float)
    args = parser.parse_args()

    formation = load_yaml(args.formation)
    edges = [tuple(map(int, edge)) for edge in formation["candidate_edges"]]
    partitions = candidate_bipartitions(
        int(formation["num_uavs"]), edges, 2, True
    )
    score_groups = grouped_csv(args.scores)
    labels = case_labels(args.rankings, args.oracle_distance, args.metric)
    by_case = defaultdict(list)
    for snapshot_id, rows in score_groups.items():
        partition, features = best_partition(rows, args.metric, partitions, edges)
        score_map = {
            tuple(sorted((int(row["uav_i"]), int(row["uav_j"])))): float(row[args.metric])
            for row in rows
        }
        by_case[rows[0]["case_id"]].append({
            "snapshot_id": snapshot_id,
            "x": float(rows[0]["center_x"]),
            "best_partition": partition,
            "best_features": features,
            "partition_features": {
                candidate: partition_features(candidate, edges, score_map)
                for candidate in partitions
            },
        })

    cases = {}
    for case_id, snapshots in by_case.items():
        snapshots.sort(key=lambda item: item["x"])
        blocked_positions = [
            snapshot["x"] for snapshot in snapshots
            if snapshot["snapshot_id"] in labels
            and not labels[snapshot["snapshot_id"]]["collision"]
            and not labels[snapshot["snapshot_id"]]["intact_feasible"]
            and labels[snapshot["snapshot_id"]]["valid_partition_count"] > 0
        ]
        cases[case_id] = {
            "snapshots": snapshots,
            "first_blocked_x": min(blocked_positions) if blocked_positions else None,
        }

    fixed_values = (
        args.fixed_split_threshold, args.fixed_merge_threshold,
        args.fixed_internal_threshold, args.fixed_persistence_distance,
    )
    if any(value is not None for value in fixed_values) and not all(
            value is not None for value in fixed_values):
        raise ValueError("all four --fixed-* options must be supplied together")
    fixed_mode = all(value is not None for value in fixed_values)
    # The original StarConvex score scale changes with formation size and graph
    # degree.  Four-UAV lateral formations produce useful cut scores below the
    # eight-UAV search range, so keep the generic sweep broad enough for both.
    split_values = [args.fixed_split_threshold] if fixed_mode else inclusive_values(0.28, 0.50, 0.01)
    merge_values = [args.fixed_merge_threshold] if fixed_mode else inclusive_values(0.60, 0.90, 0.05)
    internal_values = [args.fixed_internal_threshold] if fixed_mode else [0.0, 0.4, 0.5, 0.55, 0.6, 0.65]
    persistence_values = [args.fixed_persistence_distance] if fixed_mode else [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    output_rows = []
    for split_threshold, merge_threshold, internal_threshold, persistence in itertools.product(
            split_values, merge_values, internal_values, persistence_values):
        events = {
            case_id: simulate(case, split_threshold, merge_threshold,
                              internal_threshold, persistence)
            for case_id, case in cases.items()
        }
        blocked_cases = [case_id for case_id, case in cases.items() if case["first_blocked_x"] is not None]
        passable_cases = [case_id for case_id, case in cases.items() if case["first_blocked_x"] is None]
        detected = sum(events[case_id]["detected_before_block"] for case_id in blocked_cases)
        false_splits = sum(events[case_id]["split_x"] is not None for case_id in passable_cases)
        premature = sum(events[case_id]["premature_merge"] for case_id in blocked_cases)
        merge_count = sum(events[case_id]["merge_x"] is not None for case_id in blocked_cases)
        leads = [events[case_id]["lead_distance"] for case_id in blocked_cases
                 if events[case_id]["lead_distance"] is not None]
        output_rows.append({
            "split_threshold": split_threshold,
            "merge_threshold": merge_threshold,
            "internal_threshold": internal_threshold,
            "persistence_distance": persistence,
            "blocked_case_count": len(blocked_cases),
            "detected_before_block_count": detected,
            "missed_blocked_case_count": len(blocked_cases) - detected,
            "passable_case_count": len(passable_cases),
            "false_split_case_count": false_splits,
            "premature_merge_count": premature,
            "nominal_merge_proxy_count": merge_count,
            "minimum_lead_distance": min(leads) if leads else "",
            "median_lead_distance": statistics.median(leads) if leads else "",
            "maximum_lead_distance": max(leads) if leads else "",
        })

    fieldnames = list(output_rows[0])
    args.csv_out.parent.mkdir(parents=True, exist_ok=True)
    with args.csv_out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output_rows)

    best_error = min(
        (row["missed_blocked_case_count"], row["false_split_case_count"],
         row["premature_merge_count"])
        for row in output_rows
    )
    best_rows = [
        row for row in output_rows
        if (row["missed_blocked_case_count"], row["false_split_case_count"],
            row["premature_merge_count"]) == best_error
    ]
    summary = {
        "metric": args.metric,
        "oracle_distance": args.oracle_distance,
        "case_count": len(cases),
        "blocked_cases": sum(case["first_blocked_x"] is not None for case in cases.values()),
        "passable_cases": sum(case["first_blocked_x"] is None for case in cases.values()),
        "parameter_combinations": len(output_rows),
        "best_error_tuple": {
            "missed_blocked_cases": best_error[0],
            "false_split_cases": best_error[1],
            "premature_merges": best_error[2],
        },
        "best_error_combination_count": len(best_rows),
        "best_error_examples": best_rows[:20],
        "merge_is_nominal_rigid_scan_proxy_only": True,
        "fixed_mode": fixed_mode,
    }
    if fixed_mode:
        fixed_events = {
            case_id: simulate(
                case, args.fixed_split_threshold, args.fixed_merge_threshold,
                args.fixed_internal_threshold, args.fixed_persistence_distance,
            )
            for case_id, case in cases.items()
        }
        event_rows = []
        for case_id, event in sorted(fixed_events.items()):
            event_rows.append({
                "case_id": case_id,
                "first_blocked_x": cases[case_id]["first_blocked_x"]
                if cases[case_id]["first_blocked_x"] is not None else "",
                "split_x": event["split_x"] if event["split_x"] is not None else "",
                "merge_x": event["merge_x"] if event["merge_x"] is not None else "",
                "detected_before_block": int(event["detected_before_block"]),
                "lead_distance": event["lead_distance"]
                if event["lead_distance"] is not None else "",
                "premature_merge": int(event["premature_merge"]),
                "active_partition": event["partition"],
            })
        if args.events_out:
            args.events_out.parent.mkdir(parents=True, exist_ok=True)
            with args.events_out.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(event_rows[0]))
                writer.writeheader()
                writer.writerows(event_rows)
        summary["fixed_parameters"] = {
            "split_threshold": args.fixed_split_threshold,
            "merge_threshold": args.fixed_merge_threshold,
            "internal_threshold": args.fixed_internal_threshold,
            "persistence_distance": args.fixed_persistence_distance,
        }
        summary["fixed_events"] = event_rows
    args.summary_out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
