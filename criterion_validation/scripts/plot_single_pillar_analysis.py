#!/usr/bin/env python3
"""Plot full-space and forward-window current-state criterion results."""

import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
from typing import Dict, List, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import cm, colors, patches  # noqa: E402
import yaml  # noqa: E402


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def load_yaml(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def grouped_by_scan(rows: Sequence[Dict[str, str]]) -> Dict[int, List[Dict[str, str]]]:
    grouped = defaultdict(list)
    for row in rows:
        grouped[int(row["scan_index"])].append(row)
    return dict(sorted(grouped.items()))


def score_curve(grouped: Dict[int, List[Dict[str, str]]], field: str):
    x_values = []
    minimum_scores = []
    for rows in grouped.values():
        x_values.append(float(rows[0]["center_x"]))
        minimum_scores.append(min(float(row[field]) for row in rows))
    return x_values, minimum_scores


def oracle_state_by_scan(partition_rows: Sequence[Dict[str, str]]) -> Dict[int, Dict[str, str]]:
    result = {}
    for row in partition_rows:
        scan_index = int(row["scan_index"])
        if scan_index not in result:
            result[scan_index] = row
    return result


def plot_score_comparison(score_rows, full_partition_rows, output: Path) -> None:
    grouped = grouped_by_scan(score_rows)
    x_values, full_scores = score_curve(grouped, "composite_min")
    _, forward_scores = score_curve(grouped, "forward_composite_min")
    oracle_states = oracle_state_by_scan(full_partition_rows)
    half_step = 0.1

    figure, axis = plt.subplots(figsize=(11, 5.8))
    used_collision_label = False
    used_blocked_label = False
    for scan_index, x_value in enumerate(x_values):
        state = oracle_states[scan_index]
        if state["any_uav_in_obstacle"] == "1":
            axis.axvspan(
                x_value - half_step, x_value + half_step,
                color="#d73027", alpha=0.10,
                label="at least one UAV inside inflated pillar"
                if not used_collision_label else None,
            )
            used_collision_label = True
        elif state["intact_feasible"] == "0":
            axis.axvspan(
                x_value - half_step, x_value + half_step,
                color="#fdae61", alpha=0.35,
                label="collision-free but intact local motion infeasible"
                if not used_blocked_label else None,
            )
            used_blocked_label = True

    axis.plot(x_values, full_scores, color="#2166ac", linewidth=2.2,
              label="minimum full-space composite")
    axis.plot(x_values, forward_scores, color="#1a9850", linewidth=2.2,
              label="minimum forward-window composite")
    axis.axvline(0.0, color="0.35", linestyle="--", linewidth=1.0,
                 label="pillar centre")
    axis.set_xlabel("formation centre x [m]")
    axis.set_ylabel("minimum edge composite score")
    axis.set_ylim(-0.03, 1.03)
    axis.grid(True, alpha=0.25)
    axis.legend(loc="lower right", fontsize=8)
    axis.set_title("Current-state scores; no predicted trajectory")
    figure.tight_layout()
    figure.savefig(str(output), dpi=180)
    plt.close(figure)


def threshold_counts(rows: Sequence[Dict[str, str]]):
    useful = defaultdict(int)
    split_while_moving = defaultdict(int)
    for row in rows:
        threshold = float(row["threshold"])
        useful[threshold] += int(row["partition_locally_useful"])
        split_while_moving[threshold] += int(
            row["split_while_intact_motion_feasible"]
        )
    thresholds = sorted(useful)
    return (
        thresholds,
        [useful[value] for value in thresholds],
        [split_while_moving[value] for value in thresholds],
    )


def plot_threshold_tradeoff(full_rows, forward_rows, output: Path) -> None:
    thresholds, full_useful, full_split_moving = threshold_counts(full_rows)
    _, forward_useful, forward_split_moving = threshold_counts(forward_rows)
    figure, axes = plt.subplots(1, 2, figsize=(11, 4.6), sharex=True)

    axes[0].plot(thresholds, full_useful, "o-", color="#2166ac", label="full")
    axes[0].plot(thresholds, forward_useful, "s-", color="#1a9850", label="forward")
    axes[0].set_title("Locally useful partition positions")
    axes[0].set_ylabel("number of scan positions")
    axes[0].legend()

    axes[1].plot(
        thresholds, full_split_moving, "o-", color="#2166ac", label="full"
    )
    axes[1].plot(
        thresholds, forward_split_moving, "s-", color="#1a9850", label="forward"
    )
    axes[1].set_title("Split while intact local motion is feasible")
    axes[1].legend()

    for axis in axes:
        axis.set_xlabel("edge score threshold")
        axis.grid(True, alpha=0.25)
        axis.set_xlim(0.0, 1.0)
    figure.suptitle("Threshold sweep (no threshold selected)")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    figure.savefig(str(output), dpi=180)
    plt.close(figure)


def closest_scan(grouped, target_x: float) -> List[Dict[str, str]]:
    return min(
        grouped.values(),
        key=lambda rows: abs(float(rows[0]["center_x"]) - target_x),
    )


def plot_edge_snapshots(score_rows, scenario, output: Path) -> None:
    grouped = grouped_by_scan(score_rows)
    target_positions = (-2.2, -1.8, 1.8, 2.2)
    fields = ("composite_min", "forward_composite_min")
    labels = ("full-space", "forward-window")
    inflation = float(scenario["geometry"]["obstacle_inflation"])
    nominal_radius = float(scenario["pillars"][0]["radius"])
    pillar_x, pillar_y = map(float, scenario["pillars"][0]["center"])

    figure, axes = plt.subplots(2, len(target_positions), figsize=(14, 7), sharex=True, sharey=True)
    normalizer = colors.Normalize(vmin=0.0, vmax=1.0)
    colour_map = cm.get_cmap("RdYlGn")
    for row_index, (field, label) in enumerate(zip(fields, labels)):
        for column_index, target_x in enumerate(target_positions):
            axis = axes[row_index][column_index]
            rows = closest_scan(grouped, target_x)
            positions = {}
            for row in rows:
                positions[int(row["uav_i"])] = (
                    float(row["uav_i_x"]), float(row["uav_i_y"])
                )
                positions[int(row["uav_j"])] = (
                    float(row["uav_j_x"]), float(row["uav_j_y"])
                )
                first = int(row["uav_i"])
                second = int(row["uav_j"])
                first_point = positions[first]
                second_point = positions[second]
                axis.plot(
                    [first_point[0], second_point[0]],
                    [first_point[1], second_point[1]],
                    color=colour_map(normalizer(float(row[field]))),
                    linewidth=3.0,
                    solid_capstyle="round",
                )

            axis.add_patch(patches.Circle(
                (pillar_x, pillar_y), nominal_radius,
                facecolor="0.75", edgecolor="0.35", linewidth=1.2,
            ))
            axis.add_patch(patches.Circle(
                (pillar_x, pillar_y), nominal_radius + inflation,
                facecolor="none", edgecolor="#d73027", linestyle="--", linewidth=1.2,
            ))
            for uav_id, point in sorted(positions.items()):
                axis.plot(point[0], point[1], "ko", markersize=4)
                axis.text(point[0] + 0.08, point[1] + 0.08, str(uav_id), fontsize=8)
            centre_x = float(rows[0]["center_x"])
            axis.set_title(f"{label}, centre x={centre_x:.1f}", fontsize=9)
            axis.set_aspect("equal", adjustable="box")
            axis.set_xlim(-3.8, 3.8)
            axis.set_ylim(-2.5, 2.5)
            axis.grid(True, alpha=0.18)

    scalar_map = cm.ScalarMappable(norm=normalizer, cmap=colour_map)
    scalar_map.set_array([])
    figure.colorbar(scalar_map, ax=axes.ravel().tolist(), fraction=0.02, pad=0.02,
                    label="edge composite score")
    figure.suptitle("Candidate relation edges (red low, green high)")
    figure.savefig(str(output), dpi=180, bbox_inches="tight")
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scores", required=True, type=Path)
    parser.add_argument("--partitions-full", required=True, type=Path)
    parser.add_argument("--partitions-forward", required=True, type=Path)
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    args = parser.parse_args()

    score_rows = read_csv(args.scores)
    full_rows = read_csv(args.partitions_full)
    forward_rows = read_csv(args.partitions_forward)
    scenario = load_yaml(args.scenario)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    outputs = {
        "score_comparison": args.out_dir / "score_comparison.png",
        "threshold_tradeoff": args.out_dir / "threshold_tradeoff.png",
        "edge_snapshots": args.out_dir / "edge_snapshots.png",
    }
    plot_score_comparison(score_rows, full_rows, outputs["score_comparison"])
    plot_threshold_tradeoff(full_rows, forward_rows, outputs["threshold_tradeoff"])
    plot_edge_snapshots(score_rows, scenario, outputs["edge_snapshots"])
    print(json.dumps({key: str(value.resolve()) for key, value in outputs.items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
