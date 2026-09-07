#!/usr/bin/env python3

"""Plot four-UAV split/merge scores and topology events from a ROS bag."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag  # noqa: E402


SUMMARY_TOPIC = "/starconvex_shadow_4uav/summary"
STATUS_TOPIC = "/formation_backend/auto_split_status"
COMMAND_TOPIC = "/formation_backend/topology_command"
EDGES = ("01", "13", "32", "20")
COLORS = ("tab:blue", "tab:orange", "tab:green", "tab:purple")
STATE_LEVELS = {"INTACT": 0, "WATCH": 1, "PARTIAL_CUT": 2, "SPLIT": 3}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--plot-out", type=Path)
    parser.add_argument("--scores-csv", type=Path)
    parser.add_argument("--events-csv", type=Path)
    return parser.parse_args()


def state_changes(samples):
    changes = []
    previous = None
    for timestamp, summary in samples:
        state = summary.get("state", "UNKNOWN")
        if state != previous:
            changes.append((timestamp, state))
            previous = state
    return changes


def main():
    args = parse_args()
    if not args.bag.is_file():
        raise FileNotFoundError(args.bag)

    plot_out = args.plot_out or args.bag.with_suffix(".score_timeline.png")
    scores_csv = args.scores_csv or args.bag.with_suffix(".score_timeline.csv")
    events_csv = args.events_csv or args.bag.with_suffix(".events.csv")

    summaries = []
    statuses = []
    commands = []
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, stamp in bag.read_messages(
            topics=[SUMMARY_TOPIC, STATUS_TOPIC, COMMAND_TOPIC]
        ):
            timestamp = stamp.to_sec()
            if topic == SUMMARY_TOPIC:
                try:
                    summaries.append((timestamp, json.loads(message.data)))
                except json.JSONDecodeError:
                    continue
            elif topic == STATUS_TOPIC:
                statuses.append((timestamp, message.data))
            else:
                commands.append(
                    (
                        timestamp,
                        int(message.epoch),
                        "SPLIT" if int(message.mode) == 1 else "INTACT",
                        message.components,
                        float(message.formation_scale),
                        message.reason,
                    )
                )

    if not summaries:
        raise RuntimeError(f"{SUMMARY_TOPIC} has no valid JSON samples")

    start = summaries[0][0]
    times = np.asarray([timestamp - start for timestamp, _ in summaries])
    scores = {
        edge: np.asarray([summary[f"score_{edge}"] for _, summary in summaries])
        for edge in EDGES
    }
    merge_scores = {
        edge: np.asarray(
            [summary[f"merge_score_{edge}"] for _, summary in summaries]
        )
        for edge in EDGES
    }
    direction_valid = {
        edge: np.asarray(
            [summary[f"merge_direction_valid_{edge}"] for _, summary in summaries],
            dtype=bool,
        )
        for edge in EDGES
    }

    score_fields = [
        "time_s", "state", "components", "cut_edges", "center_x", "center_y"
    ]
    for edge in EDGES:
        score_fields.extend(
            [
                f"score_{edge}",
                f"merge_score_{edge}",
                f"overlap_{edge}",
                f"front_overlap_{edge}",
                f"clearance_{edge}",
                f"visibility_weight_{edge}",
                f"distance_{edge}",
                f"merge_direction_valid_{edge}",
            ]
        )
    with scores_csv.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=score_fields)
        writer.writeheader()
        for (timestamp, summary) in summaries:
            row = {
                "time_s": timestamp - start,
                "state": summary.get("state", ""),
                "components": summary.get("components", ""),
                "cut_edges": summary.get("cut_edges", ""),
                "center_x": summary.get("center_x", ""),
                "center_y": summary.get("center_y", ""),
            }
            for edge in EDGES:
                row[f"score_{edge}"] = summary.get(f"score_{edge}", "")
                row[f"merge_score_{edge}"] = summary.get(
                    f"merge_score_{edge}", ""
                )
                row[f"overlap_{edge}"] = summary.get(f"overlap_{edge}", "")
                row[f"front_overlap_{edge}"] = summary.get(
                    f"front_overlap_{edge}", ""
                )
                row[f"clearance_{edge}"] = summary.get(
                    f"clearance_{edge}", ""
                )
                row[f"visibility_weight_{edge}"] = summary.get(
                    f"visibility_weight_{edge}", ""
                )
                row[f"distance_{edge}"] = summary.get(f"distance_{edge}", "")
                row[f"merge_direction_valid_{edge}"] = summary.get(
                    f"merge_direction_valid_{edge}", ""
                )
            writer.writerow(row)

    changes = state_changes(summaries)
    event_rows = []
    for timestamp, state in changes:
        event_rows.append((timestamp - start, "CRITERION_STATE", state, "", ""))
    previous_status = None
    for timestamp, status in statuses:
        if status == previous_status:
            continue
        event_rows.append((timestamp - start, "AUTO_STATUS", status, "", ""))
        previous_status = status
    seen_epochs = set()
    unique_commands = []
    for command in commands:
        timestamp, epoch, mode, components, scale, reason = command
        if epoch in seen_epochs:
            continue
        seen_epochs.add(epoch)
        unique_commands.append(command)
        event_rows.append(
            (
                timestamp - start,
                "TOPOLOGY_COMMAND",
                mode,
                components,
                f"scale={scale:g}; reason={reason}; epoch={epoch}",
            )
        )
    event_rows.sort(key=lambda row: row[0])
    with events_csv.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(["time_s", "event_type", "value", "components", "details"])
        writer.writerows(event_rows)

    fig, axes = plt.subplots(
        3, 1, figsize=(15, 10), sharex=True,
        gridspec_kw={"height_ratios": [3.0, 3.0, 1.2]},
    )
    score_axis, merge_axis, state_axis = axes
    for edge, color in zip(EDGES, COLORS):
        score_axis.plot(times, scores[edge], color=color, linewidth=1.4, label=f"S {edge}")
        merge_axis.plot(
            times, merge_scores[edge], color=color, linewidth=1.4, label=f"M {edge}"
        )
        invalid_times = times[~direction_valid[edge]]
        if invalid_times.size:
            merge_axis.scatter(
                invalid_times, np.full_like(invalid_times, -0.035), marker="x",
                s=8, linewidths=0.6, color=color, alpha=0.45,
            )

    score_axis.axhline(0.34, color="tab:red", linestyle="--", linewidth=1.0,
                       label="split threshold 0.34")
    score_axis.axhline(0.20, color="tab:red", linestyle=":", linewidth=1.0,
                       label="hard split 0.20")
    score_axis.axhline(0.70, color="tab:green", linestyle="--", linewidth=1.0,
                       label="KEEP recovery 0.70")
    merge_axis.axhline(0.70, color="tab:green", linestyle="--", linewidth=1.0,
                       label="edge merge threshold 0.70")

    state_values = np.asarray(
        [STATE_LEVELS.get(summary.get("state", ""), -1) for _, summary in summaries]
    )
    state_axis.step(times, state_values, where="post", color="black", linewidth=1.4)
    state_axis.set_yticks(list(STATE_LEVELS.values()))
    state_axis.set_yticklabels(list(STATE_LEVELS.keys()))
    state_axis.set_ylim(-0.4, 3.4)

    for timestamp, _epoch, mode, components, scale, reason in unique_commands:
        relative_time = timestamp - start
        if reason == "BRIDGE_STARTUP":
            continue
        color = "tab:red" if mode == "SPLIT" else "tab:green"
        for axis in axes:
            axis.axvline(relative_time, color=color, linewidth=1.0, alpha=0.75)
        label = f"{mode} {components}\n{reason} scale={scale:g}"
        state_axis.annotate(
            label, xy=(relative_time, 3.25), xytext=(3, 0),
            textcoords="offset points", rotation=90, va="top", ha="left",
            fontsize=7, color=color,
        )

    score_axis.set_ylabel("split score S")
    merge_axis.set_ylabel("forward merge score M")
    state_axis.set_ylabel("criterion")
    state_axis.set_xlabel("time since first criterion sample (s)")
    score_axis.set_ylim(-0.05, 1.05)
    merge_axis.set_ylim(-0.07, 1.05)
    for axis in axes:
        axis.grid(True, alpha=0.25)
    score_axis.legend(loc="upper right", ncol=2, fontsize=8)
    merge_axis.legend(loc="upper right", ncol=2, fontsize=8)
    fig.suptitle(args.bag.name)
    fig.tight_layout()
    fig.savefig(plot_out, dpi=180)
    plt.close(fig)

    print(f"first criterion stamp: {start:.6f}")
    print(f"criterion samples: {len(summaries)}")
    print(f"plot: {plot_out}")
    print(f"scores CSV: {scores_csv}")
    print(f"events CSV: {events_csv}")
    print("topology commands:")
    for timestamp, _epoch, mode, components, scale, reason in unique_commands:
        print(
            f"  t={timestamp - start:8.3f}s  {mode:6s}  {components:18s} "
            f"scale={scale:g}  {reason}"
        )


if __name__ == "__main__":
    main()
