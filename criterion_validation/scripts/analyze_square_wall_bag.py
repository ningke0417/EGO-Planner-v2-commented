#!/usr/bin/env python3

"""Analyze the four-UAV square-formation wall experiment from a ROS bag."""

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import rosbag  # noqa: E402


ODOM_TOPICS = [
    f"/drone_{drone_id}_visual_slam/odom" for drone_id in range(4)
]
GROUP_TOPICS = [
    f"/formation_backend/drone_{drone_id}_group" for drone_id in range(4)
]
AUTO_STATUS_TOPIC = "/formation_backend/auto_split_status"
CRITERION_STATE_TOPIC = "/starconvex_shadow_4uav/state"
CRITERION_SCORE_TOPIC = "/starconvex_shadow_4uav/scores"
SUBGROUP_SUMMARY_TOPIC = "/formation_backend/subgroup_summary"
ROUTE_INTENT_TOPIC = "/subgroup_route_intent_4uav/intents"
ROUTE_INTENT_SUMMARY_TOPIC = "/subgroup_route_intent_4uav/summary"

EXPECTED_SPLIT_GROUPS = ["G0", "G0", "G1", "G1"]
EDGE_NAMES = ["01", "12", "23", "30"]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--wall-x", type=float, default=0.0)
    parser.add_argument("--wall-y", type=float, default=0.0)
    parser.add_argument("--wall-size-x", type=float, default=0.5)
    parser.add_argument("--wall-size-y", type=float, default=5.0)
    parser.add_argument("--safety-distance", type=float, default=0.35)
    parser.add_argument("--desired-pair-distance", type=float, default=1.0)
    parser.add_argument("--goal-x", type=float, default=6.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-z", type=float, default=1.5)
    parser.add_argument("--arrival-radius", type=float, default=0.30)
    parser.add_argument("--arrival-speed", type=float, default=0.10)
    parser.add_argument("--arrival-dwell", type=float, default=1.0)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--csv-out", type=Path)
    parser.add_argument("--plot-out", type=Path)
    return parser.parse_args()


def norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def distance(first, second):
    return norm([first[index] - second[index] for index in range(3)])


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_distance(first, second):
    return abs(wrap_angle(first - second))


def route_side(angle, reference_angle, deadband=0.12):
    lateral = math.sin(wrap_angle(angle - reference_angle))
    if lateral > deadband:
        return 1
    if lateral < -deadband:
        return -1
    return 0


def directions_conflict(first, second, reference_angle):
    first_side = route_side(first, reference_angle)
    second_side = route_side(second, reference_angle)
    conflict = (
        first_side == 0
        or second_side == 0
        or first_side == second_side
        or angle_distance(first, second) < 0.35
    )
    return conflict, [first_side, second_side]


def maximum_true_span(samples, maximum_gap=1.0):
    """Longest timestamp span of consecutive true samples."""
    active_start = None
    previous_time = None
    maximum = 0.0
    for timestamp, value in samples:
        if previous_time is not None and timestamp - previous_time > maximum_gap:
            if active_start is not None:
                maximum = max(maximum, previous_time - active_start)
            active_start = None
        if value:
            if active_start is None:
                active_start = timestamp
        elif active_start is not None:
            maximum = max(maximum, timestamp - active_start)
            active_start = None
        previous_time = timestamp
    if active_start is not None and previous_time is not None:
        maximum = max(maximum, previous_time - active_start)
    return maximum


def analyze_route_intents(samples, start_time, goal):
    active = []
    for timestamp, summary in samples:
        if summary.get("state") != "ACTIVE":
            continue
        groups = summary.get("groups")
        if not isinstance(groups, list) or len(groups) != 2:
            continue
        groups = sorted(groups, key=lambda group: int(group.get("id", -1)))
        required = {
            "id",
            "centroid",
            "local_angle",
            "selected_angle",
            "common_radius",
            "usable_length",
            "score",
        }
        if any(not required.issubset(group) for group in groups):
            continue
        active.append((timestamp, summary, groups))

    if not active:
        return {
            "available": False,
            "active_sample_count": 0,
            "reason": "no valid ACTIVE route-intent summaries",
        }

    group_stats = [
        {
            "minimum_common_radius_m": math.inf,
            "minimum_usable_length_m": math.inf,
            "minimum_score": math.inf,
            "maximum_selected_angle_step_rad": 0.0,
            "maximum_locked_angle_step_rad": 0.0,
            "selected_angle_step_sum_rad": 0.0,
            "selected_angle_step_count": 0,
            "selected_angle_jump_count_over_30deg": 0,
            "angle_limit_applied_sample_count": 0,
            "emergency_turn_sample_count": 0,
        }
        for _ in range(2)
    ]
    previous_angles = [None, None]
    previous_locked_angles = [None, None]
    coordination_count = 0
    reported_avoided_count = 0
    local_conflict_count = 0
    selected_conflict_count = 0
    coordination_conflict_count = 0
    resolved_local_conflict_count = 0
    selected_conflict_samples = []
    coordination_conflict_samples = []
    side_pair_histogram = {}
    feedback_active_count = 0
    locked_side_conflict_count = 0
    feedback_samples = []
    locked_side_pair_histogram = {}
    feedback_gains = []
    release_fading_count = 0

    for timestamp, summary, groups in active:
        centroid_x = sum(float(group["centroid"][0]) for group in groups) / 2.0
        centroid_y = sum(float(group["centroid"][1]) for group in groups) / 2.0
        reference_angle = math.atan2(goal[1] - centroid_y, goal[0] - centroid_x)
        local_angles = [float(group["local_angle"]) for group in groups]
        selected_angles = [float(group["selected_angle"]) for group in groups]
        local_conflict, _ = directions_conflict(
            local_angles[0], local_angles[1], reference_angle
        )
        selected_conflict, selected_sides = directions_conflict(
            selected_angles[0], selected_angles[1], reference_angle
        )
        coordination_active = bool(summary.get("coordination_active", False))
        feedback_active = bool(summary.get("route_locked", False)) or any(
            bool(group.get("feedback_active", False)) for group in groups
        )
        feedback_gain = float(
            summary.get(
                "feedback_gain",
                1.0 if feedback_active else 0.0,
            )
        )
        locked_sides = [int(group.get("locked_side", 0)) for group in groups]
        locked_side_conflict = feedback_active and (
            locked_sides[0] == 0
            or locked_sides[1] == 0
            or locked_sides[0] == locked_sides[1]
        )

        coordination_count += int(coordination_active)
        reported_avoided_count += int(summary.get("conflict_avoided", False))
        local_conflict_count += int(local_conflict)
        selected_conflict_count += int(selected_conflict)
        coordination_conflict_count += int(
            coordination_active and selected_conflict
        )
        resolved_local_conflict_count += int(
            coordination_active and local_conflict and not selected_conflict
        )
        feedback_active_count += int(feedback_active)
        locked_side_conflict_count += int(locked_side_conflict)
        feedback_samples.append((timestamp, feedback_active))
        feedback_gains.append(feedback_gain)
        release_fading_count += int(summary.get("release_fading", False))
        if feedback_active:
            locked_key = f"{locked_sides[0]},{locked_sides[1]}"
            locked_side_pair_histogram[locked_key] = (
                locked_side_pair_histogram.get(locked_key, 0) + 1
            )
        selected_conflict_samples.append((timestamp, selected_conflict))
        coordination_conflict_samples.append(
            (timestamp, coordination_active and selected_conflict)
        )
        side_key = f"{selected_sides[0]},{selected_sides[1]}"
        side_pair_histogram[side_key] = side_pair_histogram.get(side_key, 0) + 1

        for group_id, group in enumerate(groups):
            stats = group_stats[group_id]
            stats["minimum_common_radius_m"] = min(
                stats["minimum_common_radius_m"],
                float(group["common_radius"]),
            )
            stats["minimum_usable_length_m"] = min(
                stats["minimum_usable_length_m"],
                float(group["usable_length"]),
            )
            stats["minimum_score"] = min(
                stats["minimum_score"], float(group["score"])
            )
            stats["angle_limit_applied_sample_count"] += int(
                group.get("angle_limit_applied", False)
            )
            stats["emergency_turn_sample_count"] += int(
                group.get("emergency_turn", False)
            )
            angle = selected_angles[group_id]
            if previous_angles[group_id] is not None:
                step = angle_distance(angle, previous_angles[group_id])
                stats["maximum_selected_angle_step_rad"] = max(
                    stats["maximum_selected_angle_step_rad"], step
                )
                stats["selected_angle_step_sum_rad"] += step
                stats["selected_angle_step_count"] += 1
                stats["selected_angle_jump_count_over_30deg"] += int(
                    step > math.radians(30.0)
                )
            previous_angles[group_id] = angle
            if feedback_active:
                if previous_locked_angles[group_id] is not None:
                    locked_step = angle_distance(
                        angle, previous_locked_angles[group_id]
                    )
                    stats["maximum_locked_angle_step_rad"] = max(
                        stats["maximum_locked_angle_step_rad"], locked_step
                    )
                previous_locked_angles[group_id] = angle
            else:
                previous_locked_angles[group_id] = None

    groups_result = []
    for group_id, stats in enumerate(group_stats):
        step_count = stats.pop("selected_angle_step_count")
        step_sum = stats.pop("selected_angle_step_sum_rad")
        maximum_step = stats.pop("maximum_selected_angle_step_rad")
        maximum_locked_step = stats.pop("maximum_locked_angle_step_rad")
        groups_result.append(
            {
                "group_id": group_id,
                **stats,
                "maximum_selected_angle_step_deg": math.degrees(maximum_step),
                "maximum_locked_angle_step_deg": math.degrees(
                    maximum_locked_step
                ),
                "mean_selected_angle_step_deg": (
                    math.degrees(step_sum / step_count) if step_count else 0.0
                ),
            }
        )

    active_duration = active[-1][0] - active[0][0]
    return {
        "available": True,
        "active_sample_count": len(active),
        "first_active_time_s": active[0][0] - start_time,
        "last_active_time_s": active[-1][0] - start_time,
        "active_duration_s": active_duration,
        "effective_publish_rate_hz": (
            (len(active) - 1) / active_duration if active_duration > 0.0 else None
        ),
        "coordination_active_sample_count": coordination_count,
        "feedback_active_sample_count": feedback_active_count,
        "feedback_active_duration_s": maximum_true_span(feedback_samples),
        "minimum_feedback_gain": min(feedback_gains),
        "maximum_feedback_gain": max(feedback_gains),
        "release_fading_sample_count": release_fading_count,
        "locked_side_conflict_sample_count": locked_side_conflict_count,
        "locked_side_conflict_ratio": (
            locked_side_conflict_count / feedback_active_count
            if feedback_active_count
            else 0.0
        ),
        "locked_side_pair_histogram": locked_side_pair_histogram,
        "local_conflict_sample_count": local_conflict_count,
        "selected_conflict_sample_count": selected_conflict_count,
        "coordination_selected_conflict_sample_count": coordination_conflict_count,
        "resolved_local_conflict_sample_count": resolved_local_conflict_count,
        "reported_conflict_avoided_sample_count": reported_avoided_count,
        "selected_conflict_ratio": selected_conflict_count / len(active),
        "coordination_selected_conflict_ratio": (
            coordination_conflict_count / coordination_count
            if coordination_count
            else 0.0
        ),
        "maximum_selected_conflict_span_s": maximum_true_span(
            selected_conflict_samples
        ),
        "maximum_coordination_conflict_span_s": maximum_true_span(
            coordination_conflict_samples
        ),
        "selected_side_pair_histogram": side_pair_histogram,
        "groups": groups_result,
    }


def analyze_subgroup_same_side(samples, start_time):
    result = {}
    for group in ("g0", "g1"):
        valid = [
            (timestamp, bool(summary[f"{group}_same_side"]))
            for timestamp, summary in samples
            if f"{group}_same_side" in summary
        ]
        failures = [(timestamp, value) for timestamp, value in valid if not value]
        result[group] = {
            "sample_count": len(valid),
            "opposite_side_sample_count": len(failures),
            "opposite_side_ratio": len(failures) / len(valid) if valid else None,
            "first_opposite_side_time_s": (
                failures[0][0] - start_time if failures else None
            ),
            "maximum_opposite_side_span_s": maximum_true_span(
                [(timestamp, not same_side) for timestamp, same_side in valid]
            ),
        }
    return result


def interpolate(trace, timestamps, timestamp):
    index = bisect.bisect_left(timestamps, timestamp)
    if index <= 0:
        return trace[0][1]
    if index >= len(trace):
        return trace[-1][1]
    before = trace[index - 1]
    after = trace[index]
    duration = after[0] - before[0]
    if duration <= 1e-9:
        return after[1]
    ratio = (timestamp - before[0]) / duration
    return [
        before[1][axis] + ratio * (after[1][axis] - before[1][axis])
        for axis in range(len(before[1]))
    ]


def signed_wall_surface_clearance(position, wall_x, wall_y, size_x, size_y):
    """Signed XY distance to an axis-aligned rectangular wall footprint."""
    dx = abs(position[0] - wall_x) - 0.5 * size_x
    dy = abs(position[1] - wall_y) - 0.5 * size_y
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    inside = min(max(dx, dy), 0.0)
    return outside + inside


def pair_trace(first_trace, second_trace):
    common_start = max(first_trace[0][0], second_trace[0][0])
    common_end = min(first_trace[-1][0], second_trace[-1][0])
    second_timestamps = [sample[0] for sample in second_trace]
    values = []
    for timestamp, first_position, _ in first_trace:
        if timestamp < common_start or timestamp > common_end:
            continue
        second_position = interpolate(second_trace, second_timestamps, timestamp)
        values.append((timestamp, distance(first_position, second_position)))
    return values


def pair_statistics(values, desired_distance, start_time):
    distances = [value[1] for value in values]
    errors = [abs(value - desired_distance) for value in distances]
    min_index = min(range(len(values)), key=lambda index: distances[index])
    max_index = max(range(len(values)), key=lambda index: distances[index])
    error_index = max(range(len(values)), key=lambda index: errors[index])
    return {
        "sample_count": len(values),
        "minimum_distance_m": distances[min_index],
        "minimum_distance_time_s": values[min_index][0] - start_time,
        "maximum_distance_m": distances[max_index],
        "maximum_distance_time_s": values[max_index][0] - start_time,
        "mean_distance_m": sum(distances) / len(distances),
        "maximum_absolute_error_m": errors[error_index],
        "maximum_absolute_error_time_s": values[error_index][0] - start_time,
        "rms_error_m": math.sqrt(
            sum((value - desired_distance) ** 2 for value in distances)
            / len(distances)
        ),
    }


def first_dwell_arrival(trace, target, radius, speed_limit, dwell):
    stable_since = None
    for timestamp, position, speed in trace:
        if distance(position, target) <= radius and speed <= speed_limit:
            if stable_since is None:
                stable_since = timestamp
            if timestamp - stable_since >= dwell:
                return timestamp
        else:
            stable_since = None
    return None


def round_floats(value):
    if isinstance(value, float):
        return round(value, 4) if math.isfinite(value) else None
    if isinstance(value, list):
        return [round_floats(item) for item in value]
    if isinstance(value, dict):
        return {key: round_floats(item) for key, item in value.items()}
    return value


def main():
    args = parse_args()
    if not args.bag.is_file():
        raise FileNotFoundError(args.bag)
    if args.wall_size_x <= 0.0 or args.wall_size_y <= 0.0:
        raise ValueError("Wall dimensions must be positive.")

    json_out = args.json_out or args.bag.with_suffix(".analysis.json")
    csv_out = args.csv_out or args.bag.with_suffix(".pair_distances.csv")
    plot_out = args.plot_out or args.bag.with_suffix(".pair_distances.png")

    traces = [[] for _ in range(4)]
    group_events = [[] for _ in range(4)]
    auto_status_events = []
    criterion_states = []
    criterion_scores = []
    subgroup_summaries = []
    route_intent_summaries = []
    topic_to_drone = {topic: index for index, topic in enumerate(ODOM_TOPICS)}
    group_to_drone = {topic: index for index, topic in enumerate(GROUP_TOPICS)}

    topics = (
        ODOM_TOPICS
        + GROUP_TOPICS
        + [
            AUTO_STATUS_TOPIC,
            CRITERION_STATE_TOPIC,
            CRITERION_SCORE_TOPIC,
            SUBGROUP_SUMMARY_TOPIC,
            ROUTE_INTENT_TOPIC,
            ROUTE_INTENT_SUMMARY_TOPIC,
        ]
    )
    with rosbag.Bag(str(args.bag), "r") as bag:
        for topic, message, stamp in bag.read_messages(topics=topics):
            timestamp = stamp.to_sec()
            if topic in topic_to_drone:
                velocity = message.twist.twist.linear
                speed = norm([velocity.x, velocity.y, velocity.z])
                position = [
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    message.pose.pose.position.z,
                ]
                traces[topic_to_drone[topic]].append(
                    (timestamp, position, speed)
                )
            elif topic in group_to_drone:
                group_events[group_to_drone[topic]].append(
                    (timestamp, message.data)
                )
            elif topic == AUTO_STATUS_TOPIC:
                auto_status_events.append((timestamp, message.data))
            elif topic == CRITERION_STATE_TOPIC:
                criterion_states.append((timestamp, message.data))
            elif topic == CRITERION_SCORE_TOPIC:
                criterion_scores.append((timestamp, list(message.data)))
            elif topic == SUBGROUP_SUMMARY_TOPIC:
                try:
                    subgroup_summaries.append((timestamp, json.loads(message.data)))
                except json.JSONDecodeError:
                    pass
            elif topic == ROUTE_INTENT_SUMMARY_TOPIC:
                try:
                    route_intent_summaries.append(
                        (timestamp, json.loads(message.data))
                    )
                except json.JSONDecodeError:
                    pass

    missing = [drone_id for drone_id, trace in enumerate(traces) if not trace]
    if missing:
        raise RuntimeError(f"Missing odometry for UAV IDs: {missing}")

    start_time = min(trace[0][0] for trace in traces)
    end_time = max(trace[-1][0] for trace in traces)

    backend_split_time = None
    latest_groups = [None] * 4
    merged_group_events = sorted(
        (
            (timestamp, drone_id, group)
            for drone_id, events in enumerate(group_events)
            for timestamp, group in events
        ),
        key=lambda item: item[0],
    )
    for timestamp, drone_id, group in merged_group_events:
        latest_groups[drone_id] = group
        if latest_groups == EXPECTED_SPLIT_GROUPS:
            backend_split_time = timestamp
            break

    auto_confirm_time = next(
        (
            timestamp
            for timestamp, status in auto_status_events
            if status == "AUTO_SPLIT CONFIRMED"
        ),
        None,
    )
    criterion_split_time = next(
        (
            timestamp
            for timestamp, state in criterion_states
            if state == "SPLIT"
        ),
        None,
    )
    criterion_recovery_time = next(
        (
            timestamp
            for timestamp, state in criterion_states
            if criterion_split_time is not None
            and timestamp > criterion_split_time
            and state == "INTACT"
        ),
        None,
    )

    trace_timestamps = [[sample[0] for sample in trace] for trace in traces]
    split_geometry = None
    if backend_split_time is not None:
        positions = [
            interpolate(trace, timestamps, backend_split_time)
            for trace, timestamps in zip(traces, trace_timestamps)
        ]
        center = [
            sum(position[axis] for position in positions) / 4.0
            for axis in range(3)
        ]
        front_surface_x = args.wall_x - 0.5 * args.wall_size_x
        front_safety_x = front_surface_x - args.safety_distance
        leading_x = max(position[0] for position in positions)
        split_geometry = {
            "positions": positions,
            "formation_center": center,
            "center_remaining_to_wall_surface_m": front_surface_x - center[0],
            "center_remaining_to_safety_boundary_m": front_safety_x - center[0],
            "leading_uav_x_m": leading_x,
            "leading_uav_remaining_to_safety_boundary_m": front_safety_x
            - leading_x,
            "per_uav_surface_clearance_m": [
                signed_wall_surface_clearance(
                    position,
                    args.wall_x,
                    args.wall_y,
                    args.wall_size_x,
                    args.wall_size_y,
                )
                for position in positions
            ],
        }

    minimum_surface = math.inf
    minimum_surface_record = None
    for drone_id, trace in enumerate(traces):
        for timestamp, position, _ in trace:
            clearance = signed_wall_surface_clearance(
                position,
                args.wall_x,
                args.wall_y,
                args.wall_size_x,
                args.wall_size_y,
            )
            if clearance < minimum_surface:
                minimum_surface = clearance
                minimum_surface_record = {
                    "uav_id": drone_id,
                    "time_s": timestamp - start_time,
                    "position": position,
                }

    pair_01 = pair_trace(traces[0], traces[1])
    pair_23 = pair_trace(traces[2], traces[3])
    pair_stats = {
        "01": pair_statistics(
            pair_01, args.desired_pair_distance, start_time
        ),
        "23": pair_statistics(
            pair_23, args.desired_pair_distance, start_time
        ),
    }

    targets = [
        [args.goal_x - 0.5, args.goal_y - 0.5, args.goal_z],
        [args.goal_x + 0.5, args.goal_y - 0.5, args.goal_z],
        [args.goal_x + 0.5, args.goal_y + 0.5, args.goal_z],
        [args.goal_x - 0.5, args.goal_y + 0.5, args.goal_z],
    ]
    arrivals = []
    for drone_id, trace in enumerate(traces):
        arrival_time = first_dwell_arrival(
            trace,
            targets[drone_id],
            args.arrival_radius,
            args.arrival_speed,
            args.arrival_dwell,
        )
        arrivals.append(
            {
                "uav_id": drone_id,
                "target": targets[drone_id],
                "arrived": arrival_time is not None,
                "arrival_time_s": (
                    arrival_time - start_time if arrival_time is not None else None
                ),
                "final_position": trace[-1][1],
                "final_goal_error_m": distance(trace[-1][1], targets[drone_id]),
            }
        )

    valid_score_samples = [
        (timestamp, values)
        for timestamp, values in criterion_scores
        if len(values) == 4
    ]
    minimum_scores = {}
    for edge_index, edge_name in enumerate(EDGE_NAMES):
        if valid_score_samples:
            timestamp, values = min(
                valid_score_samples, key=lambda sample: sample[1][edge_index]
            )
            minimum_scores[edge_name] = {
                "value": values[edge_index],
                "time_s": timestamp - start_time,
            }

    criterion_rate = None
    if len(criterion_states) >= 2:
        duration = criterion_states[-1][0] - criterion_states[0][0]
        if duration > 0.0:
            criterion_rate = (len(criterion_states) - 1) / duration

    route_intent_analysis = analyze_route_intents(
        route_intent_summaries,
        start_time,
        [args.goal_x, args.goal_y],
    )
    subgroup_same_side_analysis = analyze_subgroup_same_side(
        subgroup_summaries, start_time
    )

    result = {
        "bag": str(args.bag.resolve()),
        "duration_s": end_time - start_time,
        "wall": {
            "center": [args.wall_x, args.wall_y],
            "size": [args.wall_size_x, args.wall_size_y],
            "safety_distance_m": args.safety_distance,
        },
        "events": {
            "criterion_split_time_s": (
                criterion_split_time - start_time
                if criterion_split_time is not None
                else None
            ),
            "backend_split_time_s": (
                backend_split_time - start_time
                if backend_split_time is not None
                else None
            ),
            "auto_split_confirm_time_s": (
                auto_confirm_time - start_time
                if auto_confirm_time is not None
                else None
            ),
            "criterion_recovery_time_s": (
                criterion_recovery_time - start_time
                if criterion_recovery_time is not None
                else None
            ),
            "criterion_to_backend_delay_s": (
                backend_split_time - criterion_split_time
                if backend_split_time is not None
                and criterion_split_time is not None
                else None
            ),
        },
        "split_geometry": split_geometry,
        "obstacle_clearance": {
            "minimum_surface_clearance_m": minimum_surface,
            "minimum_safety_clearance_m": minimum_surface
            - args.safety_distance,
            "minimum_record": minimum_surface_record,
            "entered_wall": minimum_surface < 0.0,
            "entered_safety_envelope": minimum_surface < args.safety_distance,
        },
        "subgroup_distance": {
            "desired_distance_m": args.desired_pair_distance,
            **pair_stats,
        },
        "subgroup_same_side": subgroup_same_side_analysis,
        "arrival": {
            "all_arrived": all(item["arrived"] for item in arrivals),
            "uavs": arrivals,
        },
        "criterion": {
            "state_sample_count": len(criterion_states),
            "score_sample_count": len(criterion_scores),
            "valid_four_edge_score_samples": len(valid_score_samples),
            "effective_publish_rate_hz": criterion_rate,
            "minimum_edge_scores": minimum_scores,
        },
        "route_intent_shadow": route_intent_analysis,
        "recorded_subgroup_summary_count": len(subgroup_summaries),
        "auto_status_events": [
            {"time_s": timestamp - start_time, "status": status}
            for timestamp, status in auto_status_events
        ],
    }
    result = round_floats(result)

    json_out.parent.mkdir(parents=True, exist_ok=True)
    csv_out.parent.mkdir(parents=True, exist_ok=True)
    plot_out.parent.mkdir(parents=True, exist_ok=True)
    json_out.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    pair_23_scalar_trace = [
        (timestamp, [value], 0.0) for timestamp, value in pair_23
    ]
    pair_23_timestamps = [sample[0] for sample in pair_23_scalar_trace]
    with csv_out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "distance_01_m", "distance_23_m"])
        for timestamp, distance_01 in pair_01:
            distance_23 = interpolate(
                pair_23_scalar_trace,
                pair_23_timestamps,
                timestamp,
            )[0]
            writer.writerow(
                [
                    f"{timestamp - start_time:.4f}",
                    f"{distance_01:.4f}",
                    f"{distance_23:.4f}",
                ]
            )

    plt.figure(figsize=(10, 5.5))
    plt.plot(
        [timestamp - start_time for timestamp, value in pair_01],
        [value for timestamp, value in pair_01],
        label="G0 distance d01",
        linewidth=1.5,
    )
    plt.plot(
        [timestamp - start_time for timestamp, value in pair_23],
        [value for timestamp, value in pair_23],
        label="G1 distance d23",
        linewidth=1.5,
    )
    plt.axhline(
        args.desired_pair_distance,
        color="black",
        linestyle="--",
        linewidth=1.0,
        label="desired distance",
    )
    if criterion_split_time is not None:
        plt.axvline(
            criterion_split_time - start_time,
            color="orange",
            linestyle=":",
            label="criterion SPLIT",
        )
    if backend_split_time is not None:
        plt.axvline(
            backend_split_time - start_time,
            color="red",
            linestyle=":",
            label="backend SPLIT",
        )
    plt.xlabel("Time from first odometry (s)")
    plt.ylabel("Subgroup pair distance (m)")
    plt.title("Four-UAV square wall experiment: subgroup distances")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(plot_out, dpi=160)
    plt.close()

    events = result["events"]
    clearance = result["obstacle_clearance"]
    print("Analysis complete")
    print(f"  criterion split: {events['criterion_split_time_s']} s")
    print(f"  backend split:   {events['backend_split_time_s']} s")
    print(
        "  minimum wall surface clearance: "
        f"{clearance['minimum_surface_clearance_m']} m"
    )
    print(
        "  minimum safety clearance: "
        f"{clearance['minimum_safety_clearance_m']} m"
    )
    print(
        "  d01 range: "
        f"{result['subgroup_distance']['01']['minimum_distance_m']} .. "
        f"{result['subgroup_distance']['01']['maximum_distance_m']} m"
    )
    print(
        "  d23 range: "
        f"{result['subgroup_distance']['23']['minimum_distance_m']} .. "
        f"{result['subgroup_distance']['23']['maximum_distance_m']} m"
    )
    print(f"  all arrived: {result['arrival']['all_arrived']}")
    same_side = result["subgroup_same_side"]
    print(
        "  subgroup opposite-side samples: "
        f"G0={same_side['g0']['opposite_side_sample_count']}, "
        f"G1={same_side['g1']['opposite_side_sample_count']}"
    )
    route_intent = result["route_intent_shadow"]
    if route_intent["available"]:
        print(
            "  route-intent feedback: samples="
            f"{route_intent['feedback_active_sample_count']}, duration="
            f"{route_intent['feedback_active_duration_s']} s, locked_pairs="
            f"{route_intent['locked_side_pair_histogram']}"
        )
        if route_intent["feedback_active_sample_count"]:
            print(
                "  route-intent locked-side conflict ratio: "
                f"{route_intent['locked_side_conflict_ratio']}"
            )
        else:
            print(
                "  route-intent coordinated conflict ratio: "
                f"{route_intent['coordination_selected_conflict_ratio']}"
            )
            print(
                "  route-intent maximum conflict span: "
                f"{route_intent['maximum_coordination_conflict_span_s']} s"
            )
        for group in route_intent["groups"]:
            print(
                f"  G{group['group_id']} intent: min_free="
                f"{group['minimum_common_radius_m']} m, max_jump="
                f"{group['maximum_selected_angle_step_deg']} deg, limited="
                f"{group['angle_limit_applied_sample_count']}, emergency="
                f"{group['emergency_turn_sample_count']}, locked_max_jump="
                f"{group['maximum_locked_angle_step_deg']} deg"
            )
    else:
        print("  route-intent shadow: not recorded")
    print(f"  JSON: {json_out}")
    print(f"  CSV:  {csv_out}")
    print(f"  plot: {plot_out}")


if __name__ == "__main__":
    main()
