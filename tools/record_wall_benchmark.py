#!/usr/bin/env python3
"""Record independent ROS 1 eight-drone wall/random trials without map payloads.

Run after sourcing ROS Noetic and formation_ws/devel/original_baseline/setup.bash.
Each trial uses a new ROS master and fresh simulator/planner processes. No GUI.
"""
import argparse
import csv
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

from benchmark_files import artifact_path, initialize as initialize_files

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'swarm-playground/formation_ws/src/planner/plan_manage/launch'
SCENE = 'rectangle_8uav_fixed_split_merge_wall.launch'
SCENES = dict(wall=SCENE, random='rectangle_8uav_sparse_random.launch')
ODOM = ['/drone_%d_visual_slam/odom' % i for i in range(8)]
TOPICS = ODOM + [
    '/drone_%d_planning/%s' % (i, suffix)
    for i in range(8) for suffix in ('pos_cmd', 'trajectory', 'data_display')
] + ['/drone_%d_traj_server/heartbeat' % i for i in range(8)] + [
    '/broadcast_traj_from_planner', '/broadcast_traj_to_planner',
    '/traj_start_trigger', '/mandatory_stop_to_planner', '/rosout_agg',
    '/tf', '/tf_static', '/benchmark/metadata', '/benchmark/events', '/benchmark/formation_error',
]


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def git_output(*arguments):
    try:
        return subprocess.check_output(['git', '-C', str(ROOT), *arguments],
                                       text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        # Minimal ROS runtime images may omit git. Source checksums still apply.
        return None


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def box_distance(p, box):
    """Signed Euclidean distance to an axis-aligned solid box."""
    q = [abs(p[k] - box['center'][k]) - box['size'][k] / 2 for k in range(3)]
    return math.sqrt(sum(max(x, 0) ** 2 for x in q)) + min(max(q), 0)


class Monitor:
    """20 Hz checks; odometry stays at its original rate in bag and CSV."""
    def __init__(self, config):
        self.config = config
        self.latest = {}
        self.initial = config['initial_positions']
        self.goals = config['goals']
        self.trigger = None
        self.departure = None
        self.all_inside_since = None
        self.arrival = None
        self.arrival_confirmed = None
        self.first_inside = [None] * 8
        self.min_pair = None
        self.min_wall = None
        self.pair_violation = False
        self.wall_violation = False
        self.invalid = False
        self.samples = 0
        self.lock = threading.RLock()

    def update(self, i, stamp, p, v):
        with self.lock:
            if not all(math.isfinite(x) for x in [stamp] + list(p) + list(v)):
                self.invalid = True
                return
            self.latest[i] = (stamp, list(p), list(v))
            if self.trigger is not None and stamp >= self.trigger:
                if self.departure is None and distance(p, self.initial[i]) >= self.config['departure_m']:
                    self.departure = stamp

    def evaluate(self, now):
        with self.lock:
            if len(self.latest) != 8 or any(now - s[0] > self.config['freshness_s']
                                            or now < s[0] - 0.1 for s in self.latest.values()):
                self.all_inside_since = None
                return False
            if self.trigger is None:
                return False
            self.samples += 1
            positions = [self.latest[i][1] for i in range(8)]
            pair = min(distance(positions[i], positions[j]) for i in range(8) for j in range(i))
            clearance = min(box_distance(p, b) - self.config['vehicle_radius_m']
                            for p in positions for b in self.config.get('obstacles', self.config.get('walls', [])))
            self.min_pair = pair if self.min_pair is None else min(self.min_pair, pair)
            self.min_wall = clearance if self.min_wall is None else min(self.min_wall, clearance)
            self.pair_violation |= pair < 2 * self.config['vehicle_radius_m']
            self.wall_violation |= clearance < 0
            inside = []
            for i in range(8):
                ok = (distance(positions[i], self.goals[i]) <= self.config['arrival_m']
                      and distance(self.latest[i][2], [0, 0, 0]) <= self.config['arrival_speed_mps'])
                inside.append(ok)
                if ok and self.first_inside[i] is None:
                    self.first_inside[i] = now
            if all(inside) and self.departure is not None:
                if self.all_inside_since is None:
                    self.all_inside_since = now
                if now - self.all_inside_since >= self.config['settle_s']:
                    self.arrival = self.all_inside_since
                    self.arrival_confirmed = now
                    return True
            else:
                self.all_inside_since = None
            return False

    def result(self):
        with self.lock:
            reached = self.arrival is not None
            safe = not (self.pair_violation or self.wall_violation or self.invalid)
            return dict(arrival_success=reached, success=reached and safe,
                        departure_stamp_ros_s=self.departure,
                        arrival_stamp_ros_s=self.arrival,
                        arrival_confirmed_stamp_ros_s=self.arrival_confirmed,
                        completion_time_s=(self.arrival - self.departure if reached else None),
                        trigger_to_arrival_s=(self.arrival - self.trigger if reached else None),
                        first_inside_stamp_ros_s=self.first_inside,
                        min_pair_center_distance_m=self.min_pair,
                        min_obstacle_surface_clearance_m=self.min_wall,
                        min_wall_surface_clearance_m=self.min_wall if self.config.get('scene', 'wall') == 'wall' else None,
                        sampled_pair_collision=self.pair_violation,
                        sampled_obstacle_collision=self.wall_violation,
                        sampled_wall_collision=self.wall_violation if self.config.get('scene', 'wall') == 'wall' else False,
                        nonfinite_odometry=self.invalid, monitor_samples=self.samples,
                        final_goal_errors_m=[distance(self.latest[i][1], self.goals[i])
                                             if i in self.latest else None for i in range(8)])


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)]:
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError('Could not stop owned process %d' % process.pid)


def wait_until(check, timeout, process=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError('Scene process exited during initialization')
        try:
            if check():
                return
        except (ConnectionError, OSError):
            pass
        time.sleep(0.1)
    raise RuntimeError('Initialization timed out: ' + check.__name__)


def audit_and_export(directory, result):
    import rosbag
    counts, types = {}, {}
    lengths, max_speeds, previous = [0.0] * 8, [0.0] * 8, {}
    max_gaps, last_stamps = [0.0] * 8, {}
    position_streams = [[] for _ in range(8)]
    with rosbag.Bag(str(artifact_path(directory, 'bag'))) as bag:
        info = bag.get_type_and_topic_info().topics
        for topic, detail in info.items():
            if topic not in TOPICS:
                raise RuntimeError('Unexpected recorded topic: ' + topic)
            if detail.msg_type in ('sensor_msgs/PointCloud2', 'sensor_msgs/PointCloud',
                                   'sensor_msgs/Image', 'nav_msgs/OccupancyGrid'):
                raise RuntimeError('Forbidden sensor/map payload: ' + topic)
            counts[topic], types[topic] = detail.message_count, detail.msg_type
        if any(counts.get(t, 0) == 0 for t in ODOM):
            raise RuntimeError('Bag is missing one or more drone odometry streams')
        if not counts.get('/benchmark/metadata') or not counts.get('/benchmark/events'):
            raise RuntimeError('Bag is missing benchmark metadata/events')
        trigger = result.get('trigger_stamp_ros_s')
        departure = result.get('departure_stamp_ros_s')
        cutoff = result.get('decision_stamp_ros_s')
        with (artifact_path(directory, 'odometry')).open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['stamp_ros_s', 'bag_stamp_ros_s', 'from_trigger_s', 'from_departure_s',
                             'drone_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
                             'vx', 'vy', 'vz', 'angular_x', 'angular_y', 'angular_z'])
            for topic, msg, bag_stamp in bag.read_messages(topics=ODOM):
                i = ODOM.index(topic)
                stamp = msg.header.stamp.to_sec()
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                v, w = msg.twist.twist.linear, msg.twist.twist.angular
                position = [p.x, p.y, p.z]
                position_streams[i].append([stamp, *position])
                writer.writerow([format(stamp, '.9f'), format(bag_stamp.to_sec(), '.9f'),
                                 stamp - trigger if trigger is not None else '',
                                 stamp - departure if departure is not None else '',
                                 i, *position, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z])
                if trigger is not None and trigger <= stamp and (cutoff is None or stamp <= cutoff):
                    if i in previous:
                        step = distance(position, previous[i])
                        if math.isfinite(step):
                            lengths[i] += step
                    previous[i] = position
                    speed = distance([v.x, v.y, v.z], [0, 0, 0])
                    if math.isfinite(speed):
                        max_speeds[i] = max(max_speeds[i], speed)
                    if i in last_stamps:
                        max_gaps[i] = max(max_gaps[i], stamp - last_stamps[i])
                    last_stamps[i] = stamp
    # Calculate from the actual recorded samples after the recorder closes.
    # The derived scalar/result is also appended to the bag as a small String.
    import numpy as np
    import rospy
    from std_msgs.msg import String
    from analyze_formation_error import evaluate, synchronize
    metadata = json.loads((artifact_path(directory, 'metadata')).read_text())
    start = result.get('departure_stamp_ros_s')
    end = result.get('arrival_stamp_ros_s') or result.get('decision_stamp_ros_s')
    formation = dict(equation=45, initial_scale=1.0, sample_hz=100.0, path_measure='centroid',
                     complete_mission=bool(result.get('arrival_success')),
                     start_stamp_ros_s=start, end_stamp_ros_s=end,
                     window='actual departure to all-drone arrival; departure to decision for unfinished flights',
                     definition='Sim(3)-aligned SUM of squared position residuals; arc-length trapezoid / (s0 L); no division by N')
    if start is not None and end is not None and end > start:
        streams = [np.asarray(rows, dtype=float) for rows in position_streams]
        desired = np.asarray([metadata['ros_parameters']['drone_%d_ego_planner_node' % i]
                              ['formation']['drone%d' % i] for i in range(8)], dtype=float)
        relative, positions, gaps = synchronize(streams, start, end, 100.0)
        values, _ = evaluate(positions, desired, relative, 1.0)
        formation.update(values, status='computed', samples=len(relative), max_odom_gap_s=max(gaps))
        result['formation_error_eq45'] = values['e_dist_centroid']
    else:
        formation['status'] = 'not_departed_or_missing_endpoint'
        result['formation_error_eq45'] = None
    result['formation_metrics'] = formation
    write_json(artifact_path(directory, 'formation'), formation)
    with rosbag.Bag(str(artifact_path(directory, 'bag')), 'a', compression=rosbag.Compression.LZ4) as bag:
        # Appending an earlier window-end timestamp makes rosbag report the
        # last chunk's end before the recorded tail. Keep the append chronological;
        # the metric's actual integration endpoints remain inside the payload.
        append_stamp = max(bag.get_end_time() + 0.001, end or 0.0)
        bag.write('/benchmark/formation_error', String(data=json.dumps(formation, allow_nan=False)),
                  t=rospy.Time.from_sec(append_stamp))
    with rosbag.Bag(str(artifact_path(directory, 'bag')), 'r') as bag:
        info = bag.get_type_and_topic_info().topics
        counts = {topic: value.message_count for topic, value in info.items()}
        types = {topic: value.msg_type for topic, value in info.items()}
        assert counts['/benchmark/formation_error'] == 1
    result.update(bag_valid=True, bag_bytes=(artifact_path(directory, 'bag')).stat().st_size,
                  topic_message_counts=counts, topic_types=types, path_lengths_m=lengths,
                  max_speeds_mps=max_speeds, max_odom_stamp_gaps_s=max_gaps)


def worker(args):
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    recording_files = initialize_files(directory, args.file_prefix, args.trial)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port,
                      ROS_HOSTNAME='127.0.0.1', ROS_IP='127.0.0.1',
                      ROS_LOG_DIR=str(directory / 'ros_logs'), ROS_HOME=str(directory / 'ros_home'))
    # Refuse to attach to any existing master, even if it belongs to this user.
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1', args.port)) == 0:
            raise RuntimeError('ROS port is already occupied: %d' % args.port)
    # The official bridge hard-codes this port independently of ROS_MASTER_URI.
    # Do not allow another experiment to silently break sequential drone startup.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(('0.0.0.0', 8081))
        except OSError as exc:
            raise RuntimeError('Official bridge UDP port 8081 is occupied; use an isolated network') from exc
    import rosgraph
    import rospy
    import yaml
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Empty, String
    from visualization_msgs.msg import MarkerArray
    command = ['roslaunch', '-p', str(args.port), 'ego_planner', SCENES[args.scene],
               'rviz:=false', 'start_planning:=false', 'record_bag:=false']
    if args.scene == 'wall':
        command.append('wall_size_y:=%s' % args.wall_length)
    else:
        command += ['random_seed:=%d' % args.random_seed, 'small_position_seed:=%d' % args.small_position_seed,
                    'small_obstacle_count:=%d' % args.small_obstacle_count]
    scene = recorder = None
    result = dict(trial=args.trial, success=False, arrival_success=False, bag_valid=False,
                  status='infrastructure_error', completion_time_s=None, recording_files=recording_files)
    scene_log = (artifact_path(directory, 'console')).open('w')
    record_log = (artifact_path(directory, 'recorder')).open('w')
    try:
        scene = subprocess.Popen(command, stdout=scene_log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        master = rosgraph.Master('/wall_benchmark_probe')
        def master_ready():
            try:
                return bool(master.getPid())
            except Exception:
                return False
        wait_until(master_ready, 30, scene)
        rospy.init_node('wall_benchmark', disable_signals=True)
        if rospy.get_param('/use_sim_time', False):
            raise RuntimeError('This simulator requires wall-clock ROS time')
        def parameters_ready():
            return rospy.has_param('/drone_7_ego_planner_node/formation/drone7')
        wait_until(parameters_ready, 30, scene)
        params = rospy.get_param('/')
        (artifact_path(directory, 'parameters')).write_text(yaml.safe_dump(params, sort_keys=True))
        initial, goals = [], []
        for i in range(8):
            prefix = '/drone_%d_ego_planner_node' % i
            offset = rospy.get_param(prefix + '/formation/drone%d' % i)
            start = rospy.get_param(prefix + '/formation/start')
            goal = [rospy.get_param(prefix + '/fsm/waypoint0_' + k) for k in 'xyz']
            angle = math.atan2(goal[1] - start[1], goal[0] - start[0])
            rotated = [math.cos(angle) * offset[0] - math.sin(angle) * offset[1],
                       math.sin(angle) * offset[0] + math.cos(angle) * offset[1], offset[2]]
            goals.append([goal[k] + rotated[k] for k in range(3)])
            initial.append([rospy.get_param('/drone_%d_poscmd_2_odom/init_%s' % (i, k)) for k in 'xyz'])
        geometry_topic = ('/map_generator/wall_geometry' if args.scene == 'wall'
                          else '/map_generator/sparse_random_geometry')
        # Subscribe once to lightweight actual solid-box geometry. Neither the
        # MarkerArray nor global/local point clouds are added to the bag.
        geometry = rospy.wait_for_message(geometry_topic, MarkerArray, timeout=30)
        obstacles = []
        for marker in geometry.markers:
            p, q, size = marker.pose.position, marker.pose.orientation, marker.scale
            values = [p.x, p.y, p.z, size.x, size.y, size.z, q.x, q.y, q.z, q.w]
            if (marker.type != marker.CUBE or marker.action != marker.ADD
                    or marker.header.frame_id != 'world' or not all(math.isfinite(v) for v in values)
                    or min(size.x, size.y, size.z) <= 0
                    or max(abs(q.x), abs(q.y), abs(q.z), abs(abs(q.w) - 1)) > 1e-8):
                raise RuntimeError('Obstacle geometry must consist of finite world-frame axis-aligned boxes')
            obstacles.append(dict(center=[p.x, p.y, p.z], size=[size.x, size.y, size.z],
                                  id=marker.id, namespace=marker.ns))
        expected_boxes = 3
        if args.scene == 'random':
            mp = params['sparse_random_obstacle_map']
            expected_boxes = mp['large_obstacle_count'] + mp['small_obstacle_count'] + (5 if mp['structured_challenge_enabled'] else 0)
            if (mp['seed'] != args.random_seed or mp['small_position_seed'] != args.small_position_seed
                    or mp['small_obstacle_count'] != args.small_obstacle_count):
                raise RuntimeError('Random map seeds/count did not match requested configuration')
        elif abs(params['fixed_split_merge_wall_map']['first_wall_size_y'] - args.wall_length) > 1e-9:
            raise RuntimeError('Wall length override did not take effect')
        if len(obstacles) != expected_boxes:
            raise RuntimeError('Actual obstacle geometry is incomplete')
        geometry_hash = hashlib.sha256(b''.join(struct.pack('<6d', *(b['center'] + b['size']))
                                                for b in obstacles)).hexdigest()
        suite_id = None
        suite_path = ROOT / 'docs/random_scene_suite.json'
        if args.scene == 'random' and suite_path.exists():
            entries = json.loads(suite_path.read_text())['scenes']
            matching = [s for s in entries if (s['random_seed'], s['small_obstacle_count'], s['small_position_seed'])
                        == (args.random_seed, args.small_obstacle_count, args.small_position_seed)]
            if matching:
                if matching[0]['geometry_sha256'] != geometry_hash:
                    raise RuntimeError('Actual geometry differs from the validated comparison-scene manifest')
                suite_id = matching[0]['scene_id']
        config = dict(initial_positions=initial, goals=goals, obstacles=obstacles,
                      walls=obstacles if args.scene == 'wall' else [], scene=args.scene,
                      planner_mode='official distributed', random_seed=args.random_seed if args.scene == 'random' else None,
                      small_position_seed=args.small_position_seed if args.scene == 'random' else None,
                      small_obstacle_count=args.small_obstacle_count if args.scene == 'random' else None,
                      arrival_m=args.arrival_m, arrival_speed_mps=args.arrival_speed,
                      settle_s=args.settle, departure_m=args.departure_m,
                      timeout_s=args.timeout, freshness_s=0.5, vehicle_radius_m=args.vehicle_radius,
                      monitor_hz=20, timing='ROS time; first >= departure_m displacement of any drone to onset of confirmed all-drone arrival',
                      success_definition='arrival_success AND no sampled sphere-obstacle or sphere-sphere overlap AND finite odometry',
                      collision_scope='20 Hz latest odometry samples, sphere model; not a continuous collision guarantee',
                      timeout_origin='start trigger', frame='world', compression='lz4', topics=TOPICS,
                      roslaunch_command=command, trial=args.trial, wall_length_m=args.wall_length if args.scene == 'wall' else None,
                      formation_error=dict(equation=45, sample_hz=100, initial_scale=1.0,
                                           path_measure='centroid', correspondence='fixed physical IDs',
                                           window='actual departure to arrival; decision cutoff for unfinished flights'),
                      recording_files=recording_files, recording_prefix=args.file_prefix,
                      geometry_sha256=geometry_hash, matched_scene_suite_id=suite_id,
                      ros_parameters=params,
                      utc_created=datetime.now(timezone.utc).isoformat())
        write_json(artifact_path(directory, 'metadata'), config)
        monitor = Monitor(config)
        def callback(msg, i):
            p, v = msg.pose.pose.position, msg.twist.twist.linear
            monitor.update(i, msg.header.stamp.to_sec(), [p.x, p.y, p.z], [v.x, v.y, v.z])
        subscriptions = [rospy.Subscriber(topic, Odometry, callback, callback_args=i, queue_size=1000)
                         for i, topic in enumerate(ODOM)]
        heartbeats = {}
        def heartbeat_callback(msg, i):
            heartbeats[i] = time.monotonic()
        subscriptions.extend(rospy.Subscriber('/drone_%d_traj_server/heartbeat' % i, Empty,
                                             heartbeat_callback, callback_args=i, queue_size=10)
                             for i in range(8))
        trigger = rospy.Publisher('/traj_start_trigger', PoseStamped, queue_size=1, latch=True)
        metadata = rospy.Publisher('/benchmark/metadata', String, queue_size=1, latch=True)
        events = rospy.Publisher('/benchmark/events', String, queue_size=20, latch=True)
        def scene_ready():
            with monitor.lock:
                return (len(monitor.latest) == 8 and len(heartbeats) == 8
                        and all(time.monotonic() - t < 2 for t in list(heartbeats.values()))
                        and trigger.get_num_connections() == 8
                        and all(distance(monitor.latest[i][1], initial[i]) < 0.01 for i in range(8))
                        and all(rospy.get_time() - state[0] < 0.5 for state in monitor.latest.values()))
        wait_until(scene_ready, 50, scene)
        recorder = subprocess.Popen(['rosbag', 'record', '--lz4', '--buffsize=128',
                                     '-O', str(artifact_path(directory, 'bag')), *TOPICS, '__name:=wall_bag_record'],
                                    stdout=record_log, stderr=subprocess.STDOUT, start_new_session=True)
        def recorder_ready():
            if recorder.poll() is not None:
                raise RuntimeError('rosbag recorder exited')
            subs = dict(master.getSystemState()[1])
            return (all('/wall_bag_record' in subs.get(t, []) for t in TOPICS)
                    and metadata.get_num_connections() >= 1 and events.get_num_connections() >= 1)
        wait_until(recorder_ready, 25, scene)
        metadata.publish(String(data=json.dumps(config)))
        time.sleep(1.0)  # Capture an idle baseline with all subscriptions established.
        with monitor.lock:
            monitor.trigger = rospy.get_time()
        result['trigger_stamp_ros_s'] = monitor.trigger
        events.publish(String(data=json.dumps(dict(event='trigger', stamp_ros_s=monitor.trigger))))
        message = PoseStamped()
        message.header.stamp = rospy.Time.from_sec(monitor.trigger)
        message.header.frame_id = 'world'
        trigger.publish(message)
        deadline = time.monotonic() + args.timeout
        stale_since = None
        next_status = 0
        while True:
            now = rospy.get_time()
            if scene.poll() is not None or recorder.poll() is not None:
                raise RuntimeError('Scene or recorder exited during flight')
            if any(time.monotonic() - t > 3 for t in list(heartbeats.values())):
                raise RuntimeError('A planner heartbeat has been absent for more than 3 seconds')
            if monitor.invalid:
                result['status'] = 'nonfinite_odometry'
                break
            with monitor.lock:
                stale = any(now - s[0] > 0.5 for s in monitor.latest.values())
            stale_since = (stale_since or time.monotonic()) if stale else None
            if stale_since is not None and time.monotonic() - stale_since > 3:
                raise RuntimeError('Odometry missing/stale for more than 3 seconds')
            if monitor.evaluate(now):
                result['status'] = 'arrived'
                break
            if time.monotonic() >= deadline:
                result['status'] = 'timeout'
                break
            if time.monotonic() >= next_status:
                with monitor.lock:
                    live = dict(monitor.result(), trigger_elapsed_s=now - monitor.trigger,
                                positions={i: state[1] for i, state in monitor.latest.items()})
                write_json(artifact_path(directory, 'live'), live)
                next_status = time.monotonic() + 5
            time.sleep(0.05)
        result.update(monitor.result(), decision_stamp_ros_s=rospy.get_time())
        pending = {k: v for k, v in result.items() if k != 'bag_valid'}
        events.publish(String(data=json.dumps(dict(event='result', audit_pending=True, **pending))))
        time.sleep(1.0)
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or type(exc).__name__
        result['status'] = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'infrastructure_error'
        if 'monitor' in locals():
            result.update(monitor.result())
        result['success'] = False
    finally:
        stop_process(recorder)  # Finalize bag before stopping publishers/master.
        if rospy.core.is_initialized():
            rospy.signal_shutdown('trial complete')
        stop_process(scene)
        scene_log.close()
        record_log.close()
        if (artifact_path(directory, 'bag')).exists():
            try:
                audit_and_export(directory, result)
            except Exception as exc:
                result.update(status='infrastructure_error', success=False, bag_valid=False, error=str(exc))
        write_json(artifact_path(directory, 'result'), result)
        # Planner stdout is verbose; keep it compressed for later failure diagnosis.
        for path in [artifact_path(directory, 'console'), *directory.glob('ros_logs/**/*.log')]:
            if path.exists():
                with path.open('rb') as src, gzip.open(str(path) + '.gz', 'wb') as dst:
                    for chunk in iter(lambda: src.read(1024 * 1024), b''):
                        dst.write(chunk)
                path.unlink()
    print(json.dumps({k: result.get(k) for k in ['trial', 'status', 'success', 'arrival_success',
                                               'completion_time_s', 'formation_error_eq45', 'bag_bytes', 'error']}), flush=True)
    return 2 if result['status'] in ('infrastructure_error', 'interrupted') else 0


def summarize(output, requested):
    paths = [artifact_path(d, 'result') for d in sorted(output.glob('trial_*')) if d.is_dir()]
    results = [json.loads(p.read_text()) for p in paths if p.exists()]
    valid = [r for r in results if r.get('bag_valid') and r['status'] not in ('infrastructure_error', 'interrupted')]
    successes = [r for r in valid if r['success']]
    arrivals = [r for r in valid if r['arrival_success']]
    times = [r['completion_time_s'] for r in successes]
    errors = [r['formation_error_eq45'] for r in valid if r.get('formation_error_eq45') is not None]
    success_errors = [r['formation_error_eq45'] for r in successes if r.get('formation_error_eq45') is not None]
    summary = dict(requested_trials=requested, recorded_trials=len(results), valid_trials=len(valid),
                   infrastructure_errors=len(results) - len(valid), successful_trials=len(successes),
                   arrived_trials=len(arrivals), success_rate=len(successes) / len(valid) if valid else None,
                   arrival_rate=len(arrivals) / len(valid) if valid else None,
                   mean_success_completion_time_s=sum(times) / len(times) if times else None,
                   formation_error_eq45_trials=len(errors),
                   mean_formation_error_eq45=sum(errors) / len(errors) if errors else None,
                   mean_success_formation_error_eq45=sum(success_errors) / len(success_errors) if success_errors else None,
                   incomplete_formation_windows=sum(not r.get('formation_metrics', {}).get('complete_mission', False) for r in valid),
                   total_bag_bytes=sum(r.get('bag_bytes', 0) for r in results), trials=results)
    write_json(output / 'summary.json', summary)
    fields = ['trial', 'status', 'bag_valid', 'success', 'arrival_success', 'completion_time_s',
              'trigger_to_arrival_s', 'formation_error_eq45', 'min_pair_center_distance_m',
              'min_obstacle_surface_clearance_m', 'min_wall_surface_clearance_m',
              'sampled_pair_collision', 'sampled_obstacle_collision', 'sampled_wall_collision', 'bag_bytes', 'error']
    with (output / 'summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)
    return summary


def main(default_scene='wall', default_runs=20):
    def terminate(signum, frame):
        raise KeyboardInterrupt('termination requested')
    signal.signal(signal.SIGTERM, terminate)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=int, default=default_runs)
    parser.add_argument('--scene', choices=sorted(SCENES), default=default_scene)
    parser.add_argument('--random-seed', type=int, default=17)
    parser.add_argument('--small-position-seed', type=int, default=-1)
    parser.add_argument('--small-obstacle-count', type=int, default=6)
    parser.add_argument('--file-prefix', default='', help='Readable filename prefix; run01..runNN is appended')
    parser.add_argument('--wall-length', type=float, default=8.0)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=11347)
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--arrival-m', type=float, default=0.3)
    parser.add_argument('--arrival-speed', type=float, default=0.15)
    parser.add_argument('--settle', type=float, default=1.0)
    parser.add_argument('--departure-m', type=float, default=0.05)
    parser.add_argument('--vehicle-radius', type=float, default=0.1)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--trial', type=int, default=1, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.file_prefix and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', args.file_prefix):
        parser.error('file-prefix must use letters, digits, underscores or hyphens')
    if args.small_obstacle_count < 0:
        parser.error('small-obstacle-count must be nonnegative')
    for name in ('runs', 'wall_length', 'timeout', 'arrival_m', 'arrival_speed', 'settle', 'departure_m', 'vehicle_radius'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(name + ' must be finite and positive')
    if args.worker:
        return worker(args)
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    provenance = dict(arguments=vars(args).copy(), git_commit=git_output('rev-parse', 'HEAD'),
        git_status=git_output('status', '--short'),
        source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in [Path(__file__), ROOT / 'tools/record_random_benchmark.py',
                                 ROOT / 'tools/benchmark_files.py',
                                 ROOT / 'tools/analyze_formation_error.py',
                                 LAUNCH / SCENE, LAUNCH / SCENES[args.scene], LAUNCH / 'baseline_8uav_member.launch',
                                 LAUNCH / 'baseline_8uav_advanced.xml', LAUNCH / 'baseline_8uav_simulator.xml']})
    provenance['arguments']['output'] = str(args.output)
    write_json(args.output / 'batch.json', provenance)
    for trial in range(1, args.runs + 1):
        command = [sys.executable, str(Path(__file__).resolve()), '--worker', '--trial', str(trial),
                   '--output', str(args.output / ('trial_%03d' % trial)), '--port', str(args.port),
                   '--scene', args.scene, '--random-seed', str(args.random_seed),
                   '--small-position-seed', str(args.small_position_seed),
                   '--small-obstacle-count', str(args.small_obstacle_count), '--file-prefix', args.file_prefix]
        for option in ['wall-length', 'timeout', 'arrival-m', 'arrival-speed', 'settle', 'departure-m', 'vehicle-radius']:
            command += ['--' + option, str(getattr(args, option.replace('-', '_')))]
        print('Starting trial %d/%d' % (trial, args.runs), flush=True)
        child = subprocess.Popen(command)
        try:
            code = child.wait()
        except KeyboardInterrupt:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=60)
            summarize(args.output, args.runs)
            return 130
        summary = summarize(args.output, args.runs)
        print('Progress %d/%d; arrival %d; success %d; %.2f MiB bags' % (
            summary['recorded_trials'], args.runs, summary['arrived_trials'], summary['successful_trials'],
            summary['total_bag_bytes'] / 1024 ** 2), flush=True)
        if code:
            print('Batch stopped: inspect the last trial result/log; no failed flight was replaced.', flush=True)
            return code
    return 0


if __name__ == '__main__':
    sys.exit(main())
