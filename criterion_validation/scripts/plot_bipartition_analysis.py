#!/usr/bin/env python3
"""Plot development/holdout bipartition ranking results."""

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, List

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import colors, patches  # noqa: E402


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def is_relevant(row: Dict[str, str]) -> bool:
    return (
        row["oracle_translation_distance"] == "0.5"
        and row["any_uav_in_obstacle"] == "0"
        and row["intact_feasible"] == "0"
        and int(row["valid_partition_count"]) > 0
    )


def ranking_rate(rows, metric, objective):
    selected = [
        row for row in rows
        if is_relevant(row)
        and row["metric"] == metric
        and row["objective"] == objective
    ]
    if not selected:
        return float("nan"), 0
    return sum(int(row["top_partition_valid"]) for row in selected) / len(selected), len(selected)


def plot_rate_matrix(development, holdout, output: Path) -> None:
    combinations = [
        ("composite_min", "mean_cut", "full / mean"),
        ("composite_min", "max_cut", "full / max"),
        ("forward_composite_min", "mean_cut", "forward / mean"),
        ("forward_composite_min", "max_cut", "forward / max"),
    ]
    datasets = [("development", development), ("holdout", holdout)]
    matrix = []
    counts = []
    for _, rows in datasets:
        rates = []
        sample_counts = []
        for metric, objective, _ in combinations:
            rate, count = ranking_rate(rows, metric, objective)
            rates.append(rate)
            sample_counts.append(count)
        matrix.append(rates)
        counts.append(sample_counts)

    figure, axis = plt.subplots(figsize=(9.5, 3.8))
    image = axis.imshow(matrix, cmap="RdYlGn", norm=colors.Normalize(0.0, 1.0), aspect="auto")
    axis.set_xticks(range(len(combinations)))
    axis.set_xticklabels([item[2] for item in combinations])
    axis.set_yticks(range(len(datasets)))
    axis.set_yticklabels([item[0] for item in datasets])
    for row_index in range(len(datasets)):
        for column_index in range(len(combinations)):
            rate = matrix[row_index][column_index]
            count = counts[row_index][column_index]
            label = "n/a" if count == 0 else f"{rate:.0%}\n(n={count})"
            axis.text(column_index, row_index, label, ha="center", va="center", fontsize=11)
    figure.colorbar(image, ax=axis, label="top-ranked partition is locally valid")
    axis.set_title("Connected bipartition ranking at 0.5 m oracle distance")
    figure.tight_layout()
    figure.savefig(str(output), dpi=180)
    plt.close(figure)


def primary_relevant_rows(rows):
    return [
        row for row in rows
        if is_relevant(row)
        and row["metric"] == "composite_min"
        and row["objective"] == "mean_cut"
    ]


def plot_relevant_distribution(development, holdout, output: Path) -> None:
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.8), sharex=True, sharey=True)
    colour_map = plt.get_cmap("viridis", 3)
    for axis, (name, rows) in zip(
        axes, (("development", development), ("holdout", holdout))
    ):
        selected = primary_relevant_rows(rows)
        for row in selected:
            partition = json.loads(row["top_partition"])
            smaller_group = min(map(len, partition))
            axis.scatter(
                float(row["center_x"]), float(row["pillar_y"]),
                s=150.0 * float(row["pillar_radius"]),
                color=colour_map((smaller_group - 2) / 2.0),
                edgecolors="black", linewidths=0.7,
            )
            axis.text(
                float(row["center_x"]) + 0.015,
                float(row["pillar_y"]) + 0.04,
                str(smaller_group), fontsize=8,
            )
        axis.set_title(f"{name}: {len(selected)} relevant snapshots")
        axis.set_xlabel("formation centre x [m]")
        axis.grid(True, alpha=0.25)
    axes[0].set_ylabel("pillar lateral offset y [m]")
    figure.suptitle("Primary rule: marker label is smaller subgroup size")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    figure.savefig(str(output), dpi=180)
    plt.close(figure)


def score_rows_by_snapshot(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault(int(row["snapshot_id"]), []).append(row)
    return grouped


def plot_primary_examples(ranking_rows, score_rows, output: Path) -> None:
    selected = [
        row for row in primary_relevant_rows(ranking_rows)
        if row["case_id"] in {"r1.00_y-1.00", "r1.00_y+0.00", "r1.00_y+1.00"}
    ]
    selected.sort(key=lambda row: float(row["pillar_y"]))
    grouped_scores = score_rows_by_snapshot(score_rows)
    figure, axes = plt.subplots(1, len(selected), figsize=(12, 4.2), sharex=True, sharey=True)
    if len(selected) == 1:
        axes = [axes]
    colours = ("#2166ac", "#f28e2b")
    for axis, ranking in zip(axes, selected):
        snapshot_rows = grouped_scores[int(ranking["snapshot_id"])]
        partition = json.loads(ranking["top_partition"])
        group_by_uav = {
            uav_id: group_index
            for group_index, group in enumerate(partition)
            for uav_id in group
        }
        positions = {}
        for row in snapshot_rows:
            first, second = int(row["uav_i"]), int(row["uav_j"])
            positions[first] = (float(row["uav_i_x"]), float(row["uav_i_y"]))
            positions[second] = (float(row["uav_j_x"]), float(row["uav_j_y"]))
            cut = group_by_uav[first] != group_by_uav[second]
            axis.plot(
                [positions[first][0], positions[second][0]],
                [positions[first][1], positions[second][1]],
                color="#d73027" if cut else "0.65",
                linestyle="--" if cut else "-",
                linewidth=2.0 if cut else 1.2,
            )
        pillar_y = float(ranking["pillar_y"])
        nominal_radius = float(ranking["pillar_radius"])
        inflated_radius = float(snapshot_rows[0]["inflated_pillar_radius"])
        axis.add_patch(patches.Circle((0.0, pillar_y), nominal_radius,
                                      facecolor="0.75", edgecolor="0.35"))
        axis.add_patch(patches.Circle((0.0, pillar_y), inflated_radius,
                                      facecolor="none", edgecolor="#d73027", linestyle="--"))
        for uav_id, point in sorted(positions.items()):
            group_index = group_by_uav[uav_id]
            axis.scatter(*point, s=55, color=colours[group_index], edgecolor="black", zorder=3)
            axis.text(point[0] + 0.07, point[1] + 0.07, str(uav_id), fontsize=8)
        sizes = sorted(map(len, partition))
        axis.set_title(f"pillar y={pillar_y:+.1f}, split {sizes[0]}+{sizes[1]}")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlim(-3.2, 2.0)
        axis.set_ylim(-3.0, 3.0)
        axis.grid(True, alpha=0.2)
    figure.suptitle("Development examples: blue/orange groups, red dashed cut edges")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.92))
    figure.savefig(str(output), dpi=180)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--development-rankings", required=True, type=Path)
    parser.add_argument("--holdout-rankings", required=True, type=Path)
    parser.add_argument("--development-scores", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()
    development = read_csv(args.development_rankings)
    holdout = read_csv(args.holdout_rankings)
    development_scores = read_csv(args.development_scores)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "ranking_matrix": args.out_dir / "ranking_matrix.png",
        "relevant_distribution": args.out_dir / "relevant_distribution.png",
        "primary_examples": args.out_dir / "primary_examples.png",
    }
    plot_rate_matrix(development, holdout, outputs["ranking_matrix"])
    plot_relevant_distribution(development, holdout, outputs["relevant_distribution"])
    plot_primary_examples(development, development_scores, outputs["primary_examples"])
    print(json.dumps({key: str(path.resolve()) for key, path in outputs.items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
