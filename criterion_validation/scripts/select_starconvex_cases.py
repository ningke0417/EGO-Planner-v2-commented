#!/usr/bin/env python3
"""Select blocked key snapshots plus nearby intact controls for 3-D comparison."""

import argparse
import csv
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scores", required=True, type=Path)
    parser.add_argument("--rankings", required=True, type=Path)
    parser.add_argument("--csv-out", required=True, type=Path)
    parser.add_argument("--summary-out", required=True, type=Path)
    parser.add_argument("--oracle-distance", default=0.5, type=float)
    args = parser.parse_args()

    with args.rankings.open(newline="", encoding="utf-8") as stream:
        ranking_rows = list(csv.DictReader(stream))
    primary_rows = [
        row for row in ranking_rows
        if row["metric"] == "composite_min"
        and row["objective"] == "mean_cut"
        and abs(float(row["oracle_translation_distance"]) - args.oracle_distance) < 1e-9
    ]
    relevant = [
        row for row in primary_rows
        if not bool(int(row["any_uav_in_obstacle"]))
        and not bool(int(row["intact_feasible"]))
        and int(row["valid_partition_count"]) > 0
    ]
    selected_ids = {int(row["snapshot_id"]) for row in relevant}
    controls = []
    for blocked in relevant:
        same_case = [
            row for row in primary_rows
            if row["case_id"] == blocked["case_id"]
            and not bool(int(row["any_uav_in_obstacle"]))
            and bool(int(row["intact_feasible"]))
        ]
        before = [row for row in same_case if float(row["center_x"]) < float(blocked["center_x"])]
        after = [row for row in same_case if float(row["center_x"]) > float(blocked["center_x"])]
        if before:
            controls.append(max(before, key=lambda row: float(row["center_x"])))
        if after:
            controls.append(min(after, key=lambda row: float(row["center_x"])))
    selected_ids.update(int(row["snapshot_id"]) for row in controls)

    with args.scores.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        score_rows = [row for row in reader if int(row["snapshot_id"]) in selected_ids]
        fieldnames = reader.fieldnames
    args.csv_out.parent.mkdir(parents=True, exist_ok=True)
    with args.csv_out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(score_rows)

    summary = {
        "oracle_distance": args.oracle_distance,
        "blocked_key_snapshot_ids": sorted(int(row["snapshot_id"]) for row in relevant),
        "control_snapshot_ids": sorted(set(int(row["snapshot_id"]) for row in controls)),
        "selected_snapshot_ids": sorted(selected_ids),
        "selected_snapshots": len(selected_ids),
        "selected_edge_rows": len(score_rows),
    }
    args.summary_out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

