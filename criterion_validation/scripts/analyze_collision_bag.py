#!/usr/bin/env python3
"""Summarize inter-UAV collision evidence and planner events in a ROS bag."""

import argparse
import bisect
import json
import math
import re
from pathlib import Path

import rosbag


ODOM_RE = re.compile(r"^/drone_(\d+)_visual_slam/odom$")
COMMAND_RE = re.compile(r"^/drone_(\d+)_planning/pos_cmd$")
TRAJECTORY_RE = re.compile(r"^/drone_(\d+)_planning/trajectory$")
SAFETY_RE = re.compile(r"^/formation_safety/drone_(\d+)/status$")
VIOLATION_RE = re.compile(
    r"^/formation_safety/drone_(\d+)/first_wall_violation$"
)
STATE_RE = re.compile(r"(?:^| )state=([^ ]+)")
KEY_LOG_WORDS = (
    "swarm distance",
    "predicted collision",
    "emergency stop",
    "SAFETY_COLLISION",
    "FIRST_WALL_VIOLATION",
    "wall guard",
    "unable to generate",
    "plan fail",
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--safe-distance",
        type=float,
        default=0.4,
        help="Distance treated as unsafe (current FSM default: 0.4 m).",
    )
    parser.add_argument(
        "--context-seconds",
        type=float,
        default=5.0,
        help="Event window printed before the first unsafe separation.",
    )
    parser.add_argument("--json-out", type=Path)
    return parser.parse_args()


def point_distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def position_from_odom(message):
    point = message.pose.pose.position
    return [point.x, point.y, point.z]


def position_from_command(message):
    point = message.position
    return [point.x, point.y, point.z]


def interpolate(trace, timestamp, stamps=None):
    if stamps is None:
        stamps = [sample[0] for sample in trace]
    index = bisect.bisect_left(stamps, timestamp)
    if index <= 0:
        return trace[0][1]
    if index >= len(trace):
        return trace[-1][1]
    before_t, before = trace[index - 1]
    after_t, after = trace[index]
    if after_t <= before_t:
        return before
    ratio = (timestamp - before_t) / (after_t - before_t)
    return [a + ratio * (b - a) for a, b in zip(before, after)]


def pair_minimum(first, second, threshold):
    if len(first) > len(second):
        first, second = second, first
    minimum = None
    first_unsafe = None
    second_stamps = [sample[0] for sample in second]
    second_start = second[0][0]
    second_end = second[-1][0]
    for timestamp, position in first:
        if timestamp < second_start or timestamp > second_end:
            continue
        value = point_distance(
            position, interpolate(second, timestamp, second_stamps)
        )
        if minimum is None or value < minimum[1]:
            minimum = (timestamp, value)
        if first_unsafe is None and value < threshold:
            first_unsafe = (timestamp, value)
    return minimum, first_unsafe


def latest_before(samples, timestamp):
    if not samples:
        return None
    stamps = [sample[0] for sample in samples]
    index = bisect.bisect_right(stamps, timestamp) - 1
    return samples[index] if index >= 0 else None


def unique_state_events(samples):
    events = []
    previous = None
    for timestamp, text in samples:
        match = STATE_RE.search(text)
        state = match.group(1) if match else text
        if state != previous:
            events.append((timestamp, state, text))
            previous = state
    return events


def rounded(value):
    if isinstance(value, float):
        return round(value, 4)
    if isinstance(value, list):
        return [rounded(item) for item in value]
    if isinstance(value, dict):
        return {key: rounded(item) for key, item in value.items()}
    return value


def main():
    args = parse_args()
    if not args.bag.is_file():
        raise FileNotFoundError(args.bag)
    if args.safe_distance <= 0.0 or args.context_seconds < 0.0:
        raise ValueError("Distances must be positive and context non-negative.")

    odometry = {}
    odometry_speeds = {}
    commands = {}
    command_details = {}
    trajectory_times = {}
    trajectory_events = {}
    safety = {}
    violations = []
    criterion_states = []
    criterion_summaries = []
    topology_commands = []
    topology_acks = []
    relevant_logs = []
    warning_logs = []
    available_topics = set()

    with rosbag.Bag(str(args.bag), "r") as bag:
        info = bag.get_type_and_topic_info()
        available_topics = set(info.topics)
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        for topic, message, stamp in bag.read_messages():
            timestamp = stamp.to_sec()
            match = ODOM_RE.match(topic)
            if match:
                uav_id = int(match.group(1))
                odometry.setdefault(uav_id, []).append(
                    (timestamp, position_from_odom(message))
                )
                velocity = message.twist.twist.linear
                odometry_speeds.setdefault(uav_id, []).append(
                    (timestamp, math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2))
                )
                continue
            match = COMMAND_RE.match(topic)
            if match:
                uav_id = int(match.group(1))
                commands.setdefault(uav_id, []).append(
                    (timestamp, position_from_command(message))
                )
                velocity = message.velocity
                command_details.setdefault(uav_id, []).append(
                    (
                        timestamp,
                        {
                            "trajectory_id": int(message.trajectory_id),
                            "speed_mps": math.sqrt(
                                velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2
                            ),
                            "position": position_from_command(message),
                        },
                    )
                )
                continue
            match = TRAJECTORY_RE.match(topic)
            if match:
                uav_id = int(match.group(1))
                trajectory_times.setdefault(uav_id, []).append(timestamp)
                trajectory_events.setdefault(uav_id, []).append(
                    (
                        timestamp,
                        {
                            "trajectory_id": int(message.traj_id),
                            "start_time": message.start_time.to_sec(),
                            "duration_s": sum(message.duration),
                        },
                    )
                )
                continue
            match = SAFETY_RE.match(topic)
            if match:
                safety.setdefault(int(match.group(1)), []).append(
                    (timestamp, message.data)
                )
                continue
            match = VIOLATION_RE.match(topic)
            if match:
                violations.append((timestamp, int(match.group(1)), message.data))
                continue
            if topic == "/starconvex_shadow_4uav/state":
                criterion_states.append((timestamp, message.data))
            elif topic == "/starconvex_shadow_4uav/summary":
                try:
                    criterion_summaries.append((timestamp, json.loads(message.data)))
                except json.JSONDecodeError:
                    criterion_summaries.append((timestamp, {"raw": message.data}))
            elif topic == "/formation_backend/topology_command":
                topology_commands.append(
                    (
                        timestamp,
                        {
                            "epoch": int(message.epoch),
                            "mode": "INTACT" if message.mode == 0 else "SPLIT",
                            "components": message.components,
                            "formation_scale": message.formation_scale,
                            "progress_sync_scale": message.progress_sync_scale,
                            "reason": message.reason,
                        },
                    )
                )
            elif topic == "/formation_backend/topology_ack":
                topology_acks.append(
                    (
                        timestamp,
                        int(message.epoch),
                        int(message.drone_id),
                        "INTACT" if message.mode == 0 else "SPLIT",
                    )
                )
            elif topic in {"/rosout", "/rosout_agg"}:
                text = message.msg
                if message.level >= 4:
                    warning_logs.append((timestamp, message.name, text))
                if any(word.lower() in text.lower() for word in KEY_LOG_WORDS):
                    relevant_logs.append((timestamp, text))

    if len(odometry) < 2:
        raise RuntimeError("Bag needs odometry from at least two UAVs.")

    pair_results = []
    first_unsafe = None
    uav_ids = sorted(odometry)
    for index, first_id in enumerate(uav_ids):
        for second_id in uav_ids[index + 1 :]:
            minimum, unsafe = pair_minimum(
                odometry[first_id], odometry[second_id], args.safe_distance
            )
            if minimum is None:
                continue
            pair_results.append(
                {
                    "pair": [first_id, second_id],
                    "minimum_distance_m": minimum[1],
                    "minimum_time_s": minimum[0] - bag_start,
                    "first_unsafe_time_s": (
                        unsafe[0] - bag_start if unsafe is not None else None
                    ),
                }
            )
            if unsafe is not None and (
                first_unsafe is None or unsafe[0] < first_unsafe[0]
            ):
                first_unsafe = (unsafe[0], first_id, second_id, unsafe[1])
    pair_results.sort(key=lambda item: item["minimum_distance_m"])

    command_pair_results = []
    for index, first_id in enumerate(sorted(commands)):
        for second_id in sorted(commands)[index + 1 :]:
            minimum, unsafe = pair_minimum(
                commands[first_id], commands[second_id], args.safe_distance
            )
            if minimum is not None:
                command_pair_results.append(
                    {
                        "pair": [first_id, second_id],
                        "minimum_distance_m": minimum[1],
                        "minimum_time_s": minimum[0] - bag_start,
                        "first_unsafe_time_s": (
                            unsafe[0] - bag_start if unsafe is not None else None
                        ),
                    }
                )
    command_pair_results.sort(key=lambda item: item["minimum_distance_m"])

    command_events = [
        {"time_s": timestamp - bag_start, **event}
        for timestamp, event in topology_commands
    ]
    ack_by_epoch = {}
    for timestamp, epoch, drone_id, mode in topology_acks:
        record = ack_by_epoch.setdefault(
            epoch, {"uavs": set(), "mode": mode, "first": timestamp, "last": timestamp}
        )
        record["uavs"].add(drone_id)
        record["first"] = min(record["first"], timestamp)
        record["last"] = max(record["last"], timestamp)
    ack_summary = [
        {
            "epoch": epoch,
            "mode": record["mode"],
            "uavs": sorted(record["uavs"]),
            "first_time_s": record["first"] - bag_start,
            "last_time_s": record["last"] - bag_start,
        }
        for epoch, record in sorted(ack_by_epoch.items())
    ]

    safety_state_events = {
        uav_id: unique_state_events(samples) for uav_id, samples in safety.items()
    }
    first_emergency = min(
        (
            (timestamp, uav_id)
            for uav_id, events in safety_state_events.items()
            for timestamp, state, _ in events
            if state == "EMERGENCY_STOP"
        ),
        default=None,
    )
    first_collision_log = next(
        (
            (timestamp, text)
            for timestamp, text in relevant_logs
            if "swarm distance" in text.lower()
            or "predicted collision" in text.lower()
            or "emergency stop" in text.lower()
        ),
        None,
    )

    anchor = None
    anchor_kind = None
    if first_unsafe is not None:
        anchor = first_unsafe[0]
        anchor_kind = "unsafe_distance"
    elif first_emergency is not None:
        anchor = first_emergency[0]
        anchor_kind = "emergency_stop"
    elif first_collision_log is not None:
        anchor = first_collision_log[0]
        anchor_kind = "predicted_collision_log"

    context = None
    if anchor is not None:
        positions_at_anchor = {
            uav_id: interpolate(trace, anchor)
            for uav_id, trace in odometry.items()
            if trace[0][0] <= anchor <= trace[-1][0]
        }
        anchor_pairs = [
            (point_distance(positions_at_anchor[first_id], positions_at_anchor[second_id]),
             first_id, second_id)
            for index, first_id in enumerate(sorted(positions_at_anchor))
            for second_id in sorted(positions_at_anchor)[index + 1 :]
        ]
        anchor_distance, first_id, second_id = min(anchor_pairs)
        commanded_positions = {
            uav_id: interpolate(trace, anchor)
            for uav_id, trace in commands.items()
            if trace[0][0] <= anchor <= trace[-1][0]
        }
        context = {
            "trigger": anchor_kind,
            "time_s": anchor - bag_start,
            "pair": [first_id, second_id],
            "distance_m": anchor_distance,
            "commanded_pair_distance_m": (
                point_distance(
                    commanded_positions[first_id], commanded_positions[second_id]
                )
                if first_id in commanded_positions and second_id in commanded_positions
                else None
            ),
            "tracking_error_m": {
                str(uav_id): point_distance(
                    positions_at_anchor[uav_id], commanded_positions[uav_id]
                )
                for uav_id in positions_at_anchor.keys() & commanded_positions.keys()
            },
            "latest_criterion_state": latest_before(criterion_states, anchor),
            "latest_criterion_summary": latest_before(
                criterion_summaries, anchor
            ),
            "latest_topology_command": latest_before(topology_commands, anchor),
            "latest_safety_status": {
                str(uav_id): latest_before(samples, anchor)
                for uav_id, samples in safety.items()
            },
            "preceding_logs": [
                [timestamp, text]
                for timestamp, text in relevant_logs
                if anchor - args.context_seconds <= timestamp <= anchor
            ],
        }
        for key in (
            "latest_criterion_state",
            "latest_criterion_summary",
            "latest_topology_command",
        ):
            sample = context[key]
            if sample is not None:
                context[key] = [sample[0] - bag_start, sample[1]]
        context["latest_safety_status"] = {
            uav_id: ([sample[0] - bag_start, sample[1]] if sample else None)
            for uav_id, sample in context["latest_safety_status"].items()
        }
        context["preceding_logs"] = [
            [timestamp - bag_start, text]
            for timestamp, text in context["preceding_logs"]
        ]

    final_motion = {}
    final_window = 3.0
    for uav_id, trace in odometry.items():
        final_time = trace[-1][0]
        window_start = final_time - final_window
        window_positions = [sample for sample in trace if sample[0] >= window_start]
        speed_samples = [
            speed for timestamp, speed in odometry_speeds[uav_id]
            if timestamp >= window_start
        ]
        final_motion[str(uav_id)] = {
            "window_s": min(final_window, final_time - trace[0][0]),
            "displacement_m": point_distance(
                window_positions[0][1], window_positions[-1][1]
            ),
            "mean_speed_mps": (
                sum(speed_samples) / len(speed_samples) if speed_samples else None
            ),
            "final_position": trace[-1][1],
        }

    result = rounded(
        {
            "bag": str(args.bag.resolve()),
            "duration_s": bag_end - bag_start,
            "uav_ids": uav_ids,
            "safe_distance_m": args.safe_distance,
            "pair_distances": pair_results,
            "command_pair_distances": command_pair_results,
            "first_unsafe_context": context,
            "final_motion": final_motion,
            "criterion_state_events": [
                {"time_s": timestamp - bag_start, "state": state}
                for timestamp, state, _ in unique_state_events(criterion_states)
            ],
            "topology_commands": command_events,
            "topology_ack_epochs": ack_summary,
            "safety_state_events": {
                str(uav_id): [
                    {"time_s": timestamp - bag_start, "state": state}
                    for timestamp, state, _ in events
                ]
                for uav_id, events in safety_state_events.items()
            },
            "first_wall_violations": [
                {"time_s": timestamp - bag_start, "uav_id": uav_id, "text": text}
                for timestamp, uav_id, text in violations
            ],
            "trajectory_message_counts": {
                str(uav_id): len(samples)
                for uav_id, samples in trajectory_times.items()
            },
            "trajectory_events": {
                str(uav_id): [
                    {"time_s": timestamp - bag_start, **event}
                    for timestamp, event in samples
                ]
                for uav_id, samples in trajectory_events.items()
            },
            "final_position_commands": {
                str(uav_id): {
                    "time_s": samples[-1][0] - bag_start,
                    "age_at_bag_end_s": bag_end - samples[-1][0],
                    **samples[-1][1],
                }
                for uav_id, samples in command_details.items()
            },
            "relevant_log_events": [
                {"time_s": timestamp - bag_start, "text": text}
                for timestamp, text in relevant_logs
            ],
            "warning_log_events": [
                {
                    "time_s": timestamp - bag_start,
                    "node": node,
                    "text": text,
                }
                for timestamp, node, text in warning_logs
            ],
            "recording_coverage": {
                "has_position_commands": bool(commands),
                "has_trajectories": bool(trajectory_times),
                "has_safety_status": bool(safety),
                "has_rosout": bool({"/rosout", "/rosout_agg"} & available_topics),
                "has_criterion_summary": bool(criterion_summaries),
                "has_topology_commands": bool(topology_commands),
            },
        }
    )

    json_out = args.json_out or args.bag.with_suffix(".collision.json")
    json_out.parent.mkdir(parents=True, exist_ok=True)
    json_out.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    print(f"bag: {args.bag} ({result['duration_s']:.2f} s)")
    print(f"UAVs: {uav_ids}; unsafe threshold: {args.safe_distance:.3f} m")
    print("closest actual pairs:")
    for item in pair_results[: min(8, len(pair_results))]:
        unsafe_text = (
            f", first unsafe t={item['first_unsafe_time_s']:.3f} s"
            if item["first_unsafe_time_s"] is not None
            else ""
        )
        print(
            f"  {item['pair'][0]}-{item['pair'][1]}: "
            f"{item['minimum_distance_m']:.3f} m at "
            f"t={item['minimum_time_s']:.3f} s{unsafe_text}"
        )
    if command_pair_results:
        item = command_pair_results[0]
        print(
            "closest commanded pair: "
            f"{item['pair'][0]}-{item['pair'][1]}, "
            f"{item['minimum_distance_m']:.3f} m at "
            f"t={item['minimum_time_s']:.3f} s"
        )
    if first_unsafe is None:
        print("No pair crossed the unsafe threshold in recorded odometry.")
    if context is None:
        print("No emergency-stop or predicted-collision event was recorded.")
    else:
        print(
            f"diagnostic anchor ({context['trigger']}): "
            f"pair {context['pair'][0]}-{context['pair'][1]} at "
            f"t={context['time_s']:.3f} s, d={context['distance_m']:.3f} m"
        )
        if context.get("commanded_pair_distance_m") is not None:
            print(
                "commanded separation at anchor: "
                f"{context['commanded_pair_distance_m']:.3f} m; "
                f"tracking errors: {context['tracking_error_m']}"
            )
        latest_command = context.get("latest_topology_command")
        if latest_command:
            event = latest_command[1]
            age = context["time_s"] - latest_command[0]
            print(
                f"latest topology: {event['mode']} ({event['reason']}), "
                f"{age:.3f} s before unsafe separation"
            )
        if context.get("preceding_logs"):
            print("relevant logs immediately before first unsafe separation:")
            for timestamp, text in context["preceding_logs"][-12:]:
                print(f"  t={timestamp:.3f}: {text}")
    print(f"full JSON: {json_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
