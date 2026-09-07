#!/usr/bin/env python3
"""Keep complete snapshots whose UAV centres are outside inflated obstacles."""

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scores", required=True, type=Path)
    parser.add_argument("--csv-out", required=True, type=Path)
    parser.add_argument("--summary-out", required=True, type=Path)
    args = parser.parse_args()

    with args.scores.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        fieldnames = reader.fieldnames
    grouped = defaultdict(list)
    for row in rows:
        grouped[int(row["snapshot_id"])].append(row)
    kept_ids = {
        snapshot_id
        for snapshot_id, snapshot_rows in grouped.items()
        if not any(
            bool(int(row["uav_i_in_obstacle"]))
            or bool(int(row["uav_j_in_obstacle"]))
            for row in snapshot_rows
        )
    }
    kept_rows = [row for row in rows if int(row["snapshot_id"]) in kept_ids]
    args.csv_out.parent.mkdir(parents=True, exist_ok=True)
    with args.csv_out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(kept_rows)
    summary = {
        "input_snapshots": len(grouped),
        "collision_free_snapshots": len(kept_ids),
        "removed_snapshots": len(grouped) - len(kept_ids),
        "output_rows": len(kept_rows),
    }
    args.summary_out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_out.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
