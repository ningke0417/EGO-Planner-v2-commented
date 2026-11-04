#!/usr/bin/env python3
"""Detached, serial three-project ROS1 benchmark with a common evaluator.

The host supervisor starts one isolated, disposable container per flight. The
worker records before scene startup and computes identical metrics from bags.
"""
import argparse
import csv
import datetime as dt
import gzip
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client
import xml.etree.ElementTree as ET

BASE = Path('/home/rise-lab/split-merge')
OURS = BASE/'EGO-Planner-v2-commented'
PROJECTS = {
    'ours': (OURS, 'swarm-playground/formation_ws/devel'),
    'zhou': (BASE/'EGO-Planner-v2-official-baseline', 'swarm-playground/formation_ws/devel/original_baseline'),
    'quan': (BASE/'Swarm-Formation', 'devel'),
}
SLOTS = [(0.5 if i < 4 else -0.5, i % 4-1.5, 0.0) for i in range(8)]
STARTS = [(x-6, y, z+1.5) for x, y, z in SLOTS]
GOALS = [(x+10, y, z+1.5) for x, y, z in SLOTS]
ODOM = {'/drone_%d_visual_slam/odom' % i: i for i in range(8)}
TOPICS = (r'^/(clock|rosout|rosout_agg|diagnostics|tf|tf_static|goal|move_base_simple/goal|'
          r'traj_start_trigger|mandatory_stop_to_planner|benchmark/.*|'
          r'broadcast_traj_(from|to)_planner|swarm/(trajectories|agent_state|edge_evidence|assignment|slot_swap)|'
          r'formation_backend/.*|formation_safety/.*|formation_metrics/.*|'
          r'drone_[0-7]_visual_slam/odom|drone_[0-7]_planning/(pos_cmd|trajectory|data_display|heartbeat|start|finish)|'
          r'drone_[0-7]_traj_server/heartbeat|drone_[0-7]/formation_applied|drone_[0-7]_formation_agent/topology)$')
FORBIDDEN = {'sensor_msgs/PointCloud2', 'sensor_msgs/PointCloud', 'sensor_msgs/Image',
             'sensor_msgs/CompressedImage', 'nav_msgs/OccupancyGrid', 'visualization_msgs/MarkerArray'}


def save(path, value):
    temporary = Path(str(path)+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def distance(a, b):
    return math.sqrt(sum((x-y)**2 for x, y in zip(a, b)))


class Monitor:
    """Common arrival gate, independent of a planner's own FSM/safety flags."""
    def __init__(self, starts=None, goals=None):
        self.latest, self.first = {}, {}
        self.starts = STARTS if starts is None else starts
        self.goals = dict(enumerate(GOALS if goals is None else goals))
        self.slots = list(range(8))
        self.departure = self.candidate = self.arrival = None
        self.invalid = 0
        self.lock = threading.RLock()

    def update(self, i, stamp, p, v):
        if not all(math.isfinite(x) for x in (stamp, *p, *v)):
            self.invalid += 1
            return
        if i in self.latest and stamp <= self.latest[i][0]:
            return
        self.first.setdefault(i, list(p))
        self.latest[i] = (stamp, list(p), list(v))
        if self.departure is None and distance(p, self.starts[i]) >= .05:
            self.departure = stamp

    def goal(self, i, p, slot):
        if distance(p, self.goals[i]) > 1e-6:
            self.candidate = None
        self.goals[i], self.slots[i] = list(p), int(slot)

    def tick(self, now):
        fresh = len(self.latest) == 8 and all(0 <= now-s[0] <= .5 for s in self.latest.values())
        inside = fresh and self.departure is not None and all(
            distance(self.latest[i][1], self.goals[i]) <= .3 and
            distance(self.latest[i][2], (0, 0, 0)) <= .15 for i in range(8))
        if not inside:
            self.candidate = None
        elif self.candidate is None:
            self.candidate = now
        elif now-self.candidate >= 1.0:
            self.arrival = self.candidate
        return self.arrival is not None

    def state(self, now):
        return dict(departure_ros_time=self.departure, all_arrived_ros_time=self.arrival,
                    arrival_success=self.arrival is not None,
                    completion_time_s=self.arrival-self.departure if self.arrival is not None else None,
                    elapsed_flight_s=now-self.departure if self.departure is not None else None,
                    odometry_streams=len(self.latest), positions={i: x[1] for i, x in self.latest.items()},
                    applied_slots=list(self.slots), final_goals={i: list(p) for i, p in self.goals.items()},
                    invalid_odometry=self.invalid)


def stop(p):
    if p is None or p.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 3)):
        try:
            os.killpg(p.pid, sig)
            p.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return
    raise RuntimeError('Owned process did not terminate: %d' % p.pid)


def wait(check, timeout, processes=()):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('Required ROS process exited')
        try:
            if check():
                return
        except (OSError, xmlrpc.client.Error):
            pass
        time.sleep(.1)
    raise RuntimeError('ROS startup readiness timeout')


def launch_command(project, wall):
    command = ['roslaunch', 'ego_planner', 'rectangle_8uav_fixed_split_merge_wall.launch',
               'rviz:=false', 'start_x:=-6.0', 'goal_x:=10.0', 'max_vel:=0.5', 'safety_distance:=0.2']
    if project == 'quan':
        command += ['first_wall_size_y:='+str(wall), 'flight_height:=1.5', 'randomize_obstacles:=false']
    else:
        command += ['wall_size_y:='+str(wall)]
    if project == 'ours':
        command += ['coordination_mode:=distributed', 'swap_enabled:=true', 'swap_scope:=all']
    return command


def expected_boxes(wall):
    return [[-2.25, -1.75, -wall/2, wall/2, -.1, 3.0],
            [6.25, 6.75, -5.0, -.9, -.1, 3.0], [6.25, 6.75, .9, 5.0, -.1, 3.0]]


def bag_filename(task, project, wall, trial):
    return 'task%02d_%s_wall%02dm_8uav_run%03d.bag' % (task, project, wall, trial)


def is_random(config):
    return config.get('scene', 'fixed_wall') == 'sparse_random'


def scene_positions(config):
    start, goal = (-14, 14) if is_random(config) else (-6, 10)
    return ([(x+start, y, z+1.5) for x, y, z in SLOTS],
            [(x+goal, y, z+1.5) for x, y, z in SLOTS])


def scene_fields(config):
    if is_random(config):
        return dict(scene='sparse_random', **config['scene_parameters'])
    return dict(scene='fixed_wall', wall_length_m=config['wall_length_m'])


def trial_bag_filename(config, project, trial):
    if not is_random(config):
        return bag_filename(config['task'], project, config['wall_length_m'], trial)
    p = config['scene_parameters']
    return 'task%02d_%s_random_map%d_small%02d_pos%02d_8uav_run%03d.bag' % (
        config['task'], project, p['random_seed'], p['small_obstacle_count'], p['small_position_seed'], trial)


def scene_launch_command(project, config):
    if not is_random(config):
        return launch_command(project, config['wall_length_m'])
    command = ['roslaunch', 'ego_planner', 'rectangle_8uav_sparse_random.launch',
               'rviz:=false', 'start_x:=-14.0', 'goal_x:=14.0', 'max_vel:=0.5']
    command += [key+':='+str(value) for key, value in config['scene_parameters'].items()]
    if project == 'ours':
        command += ['coordination_mode:=distributed', 'swap_enabled:=true', 'swap_scope:=all']
    if project == 'quan':
        command += ['flight_height:=1.5']
    return command


def random_map_parameters(project, parameters):
    repo, _ = PROJECTS[project]
    source = repo/('src' if project == 'quan' else 'swarm-playground/formation_ws/src')
    launch = ET.parse(source/'planner/plan_manage/launch/rectangle_8uav_sparse_random.launch').getroot()
    arguments = {a.get('name'): a.get('default') for a in launch.findall('arg')}
    arguments.update({k: str(v) for k, v in parameters.items()})
    arguments.update(start_x='-14.0', goal_x='14.0')
    node = next(n for n in launch.findall('node') if n.get('type') == 'sparse_random_obstacle_map')
    result = {}
    for param in node.findall('param'):
        value = param.get('value')
        match = re.fullmatch(r'\$\(arg ([^)]+)\)', value)
        if match:
            value = arguments[match.group(1)]
        if value.lower() in ('true', 'false'):
            value = value.lower() == 'true'
        else:
            try:
                value = float(value) if any(c in value.lower() for c in '.e') else int(value)
            except ValueError:
                pass
        result[param.get('name')] = value
    return result


def scene_title(config):
    if not is_random(config):
        return '第一堵墙%gm' % config['wall_length_m']
    p = config['scene_parameters']
    return '随机障碍map%d / small%02d / pos%02d' % (
        p['random_seed'], p['small_obstacle_count'], p['small_position_seed'])


def collision_metrics(xyz, boxes, radius=.1):
    import numpy as np
    pair = min(float(np.linalg.norm(xyz[:, i]-xyz[:, j], axis=1).min())
               for i, j in itertools.combinations(range(xyz.shape[1]), 2))
    clearance = math.inf
    for box in boxes:
        center = np.asarray([(box[0]+box[1])/2, (box[2]+box[3])/2, (box[4]+box[5])/2])
        half = np.asarray([(box[1]-box[0])/2, (box[3]-box[2])/2, (box[5]-box[4])/2])
        q = np.abs(xyz-center)-half
        signed = np.linalg.norm(np.maximum(q, 0), axis=-1)+np.minimum(q.max(axis=-1), 0)
        clearance = min(clearance, float(signed.min())-radius)
    return pair, clearance, pair < 2*radius or clearance < 0


def audit(run, bag_path, result, project, boxes):
    import numpy as np
    import rosbag
    from std_msgs.msg import String
    from genpy import Time
    from analyze_paper_formation_error import sample_formation, applied_reference, evaluate
    streams = [[] for _ in range(8)]
    history = [[] for _ in range(8)]
    acknowledgements = {'/drone_%d/formation_applied' % i: i for i in range(8)}
    safety = []
    with rosbag.Bag(str(bag_path)) as bag:
        topics = bag.get_type_and_topic_info().topics
        if any(t.msg_type in FORBIDDEN or not re.fullmatch(TOPICS, name) for name, t in topics.items()):
            raise RuntimeError('Unexpected or forbidden bag topic')
        if not all(t in topics and topics[t].message_count > 0 for t in ODOM):
            raise RuntimeError('Missing drone odometry')
        save(run/'recorded_topics.json', {k: dict(type=v.msg_type, count=v.message_count) for k, v in topics.items()})
        safety_topics = [k for k in topics if k.endswith('/first_wall_violation')]
        with (run/'drone_positions.csv').open('w') as f:
            writer = csv.writer(f)
            writer.writerow(['odom_ros_time', 'bag_ros_time', 'drone_id', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                             'qx', 'qy', 'qz', 'qw'])
            for topic, msg, t in bag.read_messages(topics=list(ODOM)+list(acknowledgements)+safety_topics):
                if topic in ODOM:
                    i = ODOM[topic]
                    p, v, q = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation
                    stamp = msg.header.stamp.to_sec()
                    streams[i].append((stamp, p.x, p.y, p.z))
                    writer.writerow([stamp, t.to_sec(), i, p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w])
                elif topic in acknowledgements:
                    i = acknowledgements[topic]
                    if sorted(msg.drone_to_slot) != list(range(8)):
                        raise RuntimeError('Invalid applied slot acknowledgement')
                    history[i].append((msg.header.stamp.to_sec() or t.to_sec(), msg.drone_to_slot[i]))
                else:
                    safety.append(dict(topic=topic, ros_time=t.to_sec(), message=msg.data))
    save(run/'planner_safety_diagnostics.json', safety)
    result['planner_safety_event_messages'] = len(safety)
    result['odometry_counts'] = [len(x) for x in streams]
    start = result['departure_ros_time']
    end = result['all_arrived_ros_time'] or result['decision_ros_time']
    if start is not None:
        t, xyz, gap = sample_formation(streams, start, end, 100)
        desired = np.asarray(SLOTS)
        if project == 'ours':
            reference, mappings, changes = applied_reference(desired, history, t+start)
        else:
            reference, mappings, changes = desired, np.tile(np.arange(8), (len(t), 1)), [0]*8
        values, residual, graph, scales = evaluate(t, xyz, reference, 1.0)
        tc, pc, _ = sample_formation(streams, start, end, 50)
        coarse_ref = applied_reference(desired, history, tc+start)[0] if project == 'ours' else desired
        coarse = evaluate(tc, pc, coarse_ref, 1.0)[0]
        pair, clearance, collision = collision_metrics(xyz, boxes)
        interval = 'departure_to_all_arrived' if result['arrival_success'] else 'departure_to_failure_cutoff'
        result.update(formation_error_eq45=values['e_dist_eq45'], formation_error_eq46=values['e_sim_eq46'],
                      formation_error_interval=interval, min_pair_distance_m=pair,
                      min_obstacle_surface_clearance_m=clearance, sampled_collision=collision,
                      collision_free_success=result['arrival_success'] and not collision and not result['invalid_odometry'],
                      assignment_slot_changes=sum(changes), max_odometry_gap_s=gap,
                      eq45_relative_50hz_change=abs(values['e_dist_eq45']-coarse['e_dist_eq45'])/max(values['e_dist_eq45'], 1e-14),
                      path_lengths_m=np.linalg.norm(np.diff(xyz, axis=0), axis=-1).sum(axis=0).tolist())
        save(run/'formation_error_eq45.json', dict(**values, sample_hz=100, check_hz=50,
             s0=1, path='3D centroid arc length', unit='m^2', interval=interval,
             correspondence='own applied slot' if project == 'ours' else 'fixed physical IDs',
             coarse_eq45=coarse['e_dist_eq45'], max_odometry_gap_s=gap))
        np.savetxt(str(run/'formation_error_timeseries.csv'), np.column_stack([t, residual, graph, scales, mappings]),
                   delimiter=',', header='elapsed_s,residual_squared_sum_m2,graph_error,alignment_scale,'+
                   ','.join('drone_%d_slot' % i for i in range(8)), comments='')
    else:
        result.update(collision_free_success=False, formation_error_eq45=None, formation_error_interval='not_departed')
    result.update(bag_valid=True)
    with rosbag.Bag(str(bag_path), 'a', compression=rosbag.Compression.LZ4) as bag:
        bag.write('/benchmark/final_metrics', String(data=json.dumps(result)),
                  Time.from_sec(result['decision_ros_time']))
    result.update(bag_bytes=bag_path.stat().st_size, bag_sha256=sha(bag_path))


def worker(args):
    import numpy as np
    import rosbag
    import rospy
    import yaml
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray
    config = json.loads((args.output/'config.json').read_text())
    project = args.project
    repo, devel = PROJECTS[project]
    run = args.output/project/('run_%03d' % args.trial)
    run.mkdir(parents=True, exist_ok=False)
    starts, goals = scene_positions(config)
    bag_name = trial_bag_filename(config, project, args.trial)
    bag_path = run/bag_name
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP='127.0.0.1',
               ROS_LOG_DIR=str(run/'ros_logs'), ROS_HOME=str(run/'ros_home'))
    env.pop('ROS_HOSTNAME', None)
    os.environ.update(env)
    handles, processes = [], []
    master = recorder = scene = None
    monitor = Monitor(starts, goals)
    boxes = []
    result = dict(project=project, trial=args.trial, bag_file=bag_name, arrival_success=False,
                  collision_free_success=False, completion_time_s=None, bag_valid=False)
    def spawn(command, name):
        f = (run/name).open('w'); handles.append(f)
        p = subprocess.Popen(command, stdout=f, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        processes.append(p)
        return p
    def terminate(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, terminate)
    try:
        found = subprocess.check_output(['rospack', 'find', 'ego_planner'], env=env, text=True).strip()
        if repo.resolve() not in Path(found).resolve().parents:
            raise RuntimeError('Wrong planner workspace: '+found)
        master = spawn(['roscore', '-p', '11311'], 'ros_master.log')
        proxy = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        wait(lambda: proxy.getPid('/comparison')[0] == 1, 25, [master])
        rospy.init_node('comparison_metrics', disable_signals=True)
        event = rospy.Publisher('/benchmark/events', String, queue_size=10, latch=True)
        metrics = rospy.Publisher('/benchmark/live_metrics', String, queue_size=10, latch=True)
        def odom(msg, i):
            p, v = msg.pose.pose.position, msg.twist.twist.linear
            with monitor.lock:
                monitor.update(i, msg.header.stamp.to_sec(), (p.x, p.y, p.z), (v.x, v.y, v.z))
        subscriptions = [rospy.Subscriber(topic, Odometry, odom, i, queue_size=1000) for topic, i in ODOM.items()]
        if project == 'ours':
            from ego_planner.msg import FormationTopologyAck
            def applied(msg, i):
                if msg.goal_valid:
                    with monitor.lock:
                        monitor.goal(i, msg.assigned_goal, msg.drone_to_slot[i])
            subscriptions += [rospy.Subscriber('/drone_%d/formation_applied' % i,
                              FormationTopologyAck, applied, i, queue_size=100) for i in range(8)]
        def geometry(msg):
            parsed = []
            for m in msg.markers:
                if m.type != m.CUBE or m.action != m.ADD:
                    continue
                q, p, s = m.pose.orientation, m.pose.position, m.scale
                if m.header.frame_id != 'world' or max(abs(q.x), abs(q.y), abs(q.z), abs(abs(q.w)-1)) > 1e-8:
                    return
                parsed.append([p.x-s.x/2, p.x+s.x/2, p.y-s.y/2, p.y+s.y/2, p.z-s.z/2, p.z+s.z/2])
            boxes[:] = parsed
        geometry_topic = '/map_generator/sparse_random_geometry' if is_random(config) else '/map_generator/wall_geometry'
        subscriptions.append(rospy.Subscriber(geometry_topic, MarkerArray, geometry, queue_size=1))
        recorder = spawn(['rosbag', 'record', '--lz4', '--buffsize=256', '--chunksize=768', '-O', str(bag_path),
                          '-e', TOPICS, '__name:=comparison_bag_recorder'], 'bag_recorder.log')
        wait(lambda: proxy.lookupNode('/comparison', '/comparison_bag_recorder')[0] == 1 and
             event.get_num_connections() > 0, 25, [master, recorder])
        time.sleep(1)
        command = scene_launch_command(project, config)
        save(run/'launch_command.json', command)
        scene = spawn(command, 'planner.log')
        reference_boxes = config['expected_boxes'] if is_random(config) else expected_boxes(config['wall_length_m'])
        wait(lambda: len(boxes) == len(reference_boxes) and rospy.has_param('/drone_7_ego_planner_node/fsm/waypoint0_x'),
             45, [master, recorder, scene])
        params = rospy.get_param('/')
        if is_random(config):
            for key, value in config['generator_parameters'].items():
                if params['sparse_random_obstacle_map'].get(key) != value:
                    raise RuntimeError('Random map parameter mismatch: '+key)
            np.testing.assert_allclose(sorted(boxes), sorted(reference_boxes), rtol=0, atol=1e-8)
        else:
            if params['fixed_split_merge_wall_map']['first_wall_size_y'] != config['wall_length_m']:
                raise RuntimeError('First wall length mismatch')
            np.testing.assert_allclose(boxes, reference_boxes, rtol=0, atol=1e-8)
        for i in range(8):
            node = params['drone_%d_ego_planner_node' % i]
            waypoint = [node['fsm']['waypoint0_'+a] for a in 'xyz']
            if project == 'quan':
                np.testing.assert_allclose(waypoint, goals[i], rtol=0, atol=1e-8)
            else:
                np.testing.assert_allclose(waypoint, (14 if is_random(config) else 10, 0, 1.5), rtol=0, atol=1e-8)
                np.testing.assert_allclose([node['formation']['drone%d' % j] for j in range(8)], SLOTS, rtol=0, atol=1e-8)
                if project == 'ours' and not node['formation']['distributed_enabled']:
                    raise RuntimeError('Ours must run distributed')
        (run/'ros_parameters.yaml').write_text(yaml.safe_dump(params))
        save(run/'scene_geometry.json', dict(boxes_xyz_min_max=boxes, vehicle_radius_m=.1))
        save(run/'scene_metadata.json', dict(project=project, launch_command=command,
             expected_start_positions=starts, expected_goals=goals, template=SLOTS,
             evaluation=config['evaluation'], **scene_fields(config)))
        event.publish(String(data=json.dumps(dict(event='scene_ready', project=project, trial=args.trial))))
        begin, next_status = time.monotonic(), 0
        reason = 'flight_timeout'
        while not rospy.is_shutdown():
            if any(p.poll() is not None for p in (master, recorder, scene)):
                raise RuntimeError('ROS scene/recorder exited during flight')
            now = rospy.get_time()
            with monitor.lock:
                if any(distance(p, starts[i]) > .02 for i, p in monitor.first.items()):
                    raise RuntimeError('Initial physical drone positions differ from benchmark')
                if monitor.invalid:
                    reason = 'invalid_odometry'; break
                if monitor.tick(now):
                    reason = 'all_arrived'; break
                if monitor.departure is None and time.monotonic()-begin > 90:
                    reason = 'no_departure'; break
                if monitor.departure is not None and now-monitor.departure >= 240:
                    reason = 'flight_timeout'; break
                if monitor.departure is not None and (len(monitor.latest) != 8 or any(now-x[0] > 5 for x in monitor.latest.values())):
                    raise RuntimeError('Odometry absent for over 5 seconds')
                if time.monotonic() >= next_status:
                    state = monitor.state(now)
                    save(run/'flight_status.json', state)
                    metrics.publish(String(data=json.dumps(state)))
                    next_status = time.monotonic()+2
            time.sleep(.05)
        result.update(monitor.state(rospy.get_time()), reason=reason, decision_ros_time=rospy.get_time())
        event.publish(String(data=json.dumps(dict(event='flight_finished', **result))))
        time.sleep(.5)
    except (Exception, KeyboardInterrupt) as exc:
        result.update(reason='infrastructure_error', error=str(exc) or type(exc).__name__, arrival_success=False)
    finally:
        for p in (recorder, scene, master):
            try:
                stop(p)
            except Exception as exc:
                result.update(reason='infrastructure_error', error=str(exc))
        if rospy.core.is_initialized():
            rospy.signal_shutdown('finished')
        for f in handles:
            f.close()
    if result['reason'] != 'infrastructure_error':
        try:
            if not bag_path.exists() or Path(str(bag_path)+'.active').exists():
                raise RuntimeError('Bag not finalized')
            audit(run, bag_path, result, project, boxes)
        except Exception as exc:
            result.update(reason='infrastructure_error', error='postflight audit: '+str(exc))
    save(run/'metrics.json', result)
    for path in [run/'planner.log', *run.glob('ros_logs/**/*.log')]:
        if path.exists():
            with path.open('rb') as src, gzip.open(str(path)+'.gz', 'wb') as dst:
                shutil.copyfileobj(src, dst)
            path.unlink()
    print(json.dumps(result), flush=True)
    return 2 if result['reason'] == 'infrastructure_error' else 0


def summarize(output, config):
    rows = []
    for name in PROJECTS:
        for f in sorted((output/name).glob('run_*/metrics.json')):
            r = json.loads(f.read_text())
            r['bag_path'] = str(f.parent.relative_to(output)/r['bag_file'])
            r['positions_path'] = str(f.parent.relative_to(output)/'drone_positions.csv')
            rows.append(r)
    report = {}
    def mean(items, key):
        v = [r[key] for r in items if r.get(key) is not None]
        return sum(v)/len(v) if v else None
    for name in PROJECTS:
        all_rows = [r for r in rows if r['project'] == name]
        valid = [r for r in all_rows if r['reason'] != 'infrastructure_error' and r.get('bag_valid')]
        arrived = [r for r in valid if r['arrival_success']]
        safe = [r for r in valid if r['collision_free_success']]
        report[name] = dict(requested_runs=config['runs_per_project'], completed_runs=len(all_rows), valid_runs=len(valid),
            infrastructure_errors=len(all_rows)-len(valid), arrived_runs=len(arrived), collision_free_arrived_runs=len(safe),
            arrival_success_rate=len(arrived)/len(valid) if valid else None,
            collision_free_arrival_rate=len(safe)/len(valid) if valid else None,
            mean_arrival_time_s=mean(arrived, 'completion_time_s'), mean_collision_free_arrival_time_s=mean(safe, 'completion_time_s'),
            mean_eq45_arrived_m2=mean(arrived, 'formation_error_eq45'), mean_eq45_collision_free_arrived_m2=mean(safe, 'formation_error_eq45'),
            mean_eq45_all_departed_m2=mean(valid, 'formation_error_eq45'), total_bag_bytes=sum(r.get('bag_bytes', 0) for r in all_rows))
    save(output/'comparison_summary.json', report)
    fields = ['project', 'trial', 'arrival_success', 'collision_free_success', 'reason', 'completion_time_s',
              'formation_error_eq45', 'formation_error_interval', 'min_pair_distance_m', 'min_obstacle_surface_clearance_m',
              'planner_safety_event_messages', 'bag_bytes', 'bag_path', 'positions_path']
    with (output/'all_trial_metrics.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore'); writer.writeheader(); writer.writerows(rows)
    with (output/'comparison_summary.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=['project']+list(next(iter(report.values())))); writer.writeheader()
        writer.writerows(dict(project=k, **v) for k, v in report.items())
    lines = ['# 任务%d：%s，三项目八机2×4编队' % (config['task'], scene_title(config)), '',
             '本文件随每轮完成更新。当前进度及最终状态见 status.json。', '',
             '统一口径：到达半径0.30m、速度≤0.15m/s并保持1s；从首架机移动0.05m计时，到全员稳定到达区间起点。',
             '到达率与无碰撞到达率分列；球体半径0.10m，100Hz同步插值检查实体墙盒与机间距离（不构成连续时间证明）。',
             '超时240s，未出发90s；飞行失败保留并计入分母，基础设施错误单列并停止。',
             '论文Robust.pdf第12页式45：Sim(3)最优对齐后八机残差平方和，质心弧长积分，s0=1；100Hz计算、50Hz复核，单位m²。',
             'ours使用自身实际应用槽位；Zhou、Quan使用固定物理编号与2×4模板；不优化无人机编号对应。',
             'ours为分布式且开启跨列换槽。三项目各自沿用规划算法设置，统一起终点、地图和最大速度0.5m/s；RViz关闭。',
             '按轮次交替串行运行ours、Zhou、Quan；各次独立容器、独立网络，排除UDP轨迹互串。', '',
             '|项目|完成/目标|到达率|无碰撞到达率|平均到达耗时/s|平均式45/m²|', '|---|---:|---:|---:|---:|---:|']
    def fmt(v): return '—' if v is None else '%.6f' % v
    for name, r in report.items():
        lines.append('|%s|%d/%d|%s|%s|%s|%s|' % (name, r['completed_runs'], config['runs_per_project'],
            fmt(r['arrival_success_rate']), fmt(r['collision_free_arrival_rate']), fmt(r['mean_arrival_time_s']), fmt(r['mean_eq45_arrived_m2'])))
    lines += ['', '文件：all_trial_metrics.csv（含bag/位置路径）、comparison_summary.csv/json、各轮metrics.json、drone_positions.csv、formation_error_timeseries.csv、scene_geometry.json、ros_parameters.yaml及压缩日志。',
              '只录位置、姿态、速度、控制指令、规划轨迹、心跳、编队消息及诊断；不录点云、图像、地图栅格或MarkerArray。',
              '规划器安全诊断单列，不能以几何通过推断没有触发局部地图告警。', '']
    (output/'REPORT.md').write_text('\n'.join(lines))
    return report


def supervisor(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    code = output/('code_resume_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S') if args.resume else 'code')
    code.mkdir()
    for name in ('run_comparison_benchmark.py', 'analyze_paper_formation_error.py'):
        shutil.copyfile(OURS/'scripts'/name, code/name)
    shutil.copyfile(BASE/'对比实验.txt', output/'requested_experiments.txt')
    binaries = {}
    for name, (repo, devel) in PROJECTS.items():
        map_binary = 'sparse_random_obstacle_map' if args.scene == 'sparse_random' else 'fixed_split_merge_wall_map'
        binaries[name] = {str(repo/devel/'lib/ego_planner/ego_planner_node'): sha(repo/devel/'lib/ego_planner/ego_planner_node'),
                          str(repo/devel/'lib/map_generator'/map_binary): sha(repo/devel/'lib/map_generator'/map_binary)}
    config = dict(task=args.task, wall_length_m=float(args.wall_length), runs_per_project=args.runs, project_order=list(PROJECTS),
                  binaries=binaries, paper_sha256=sha(BASE/'Robust.pdf'), image=args.image,
                  created_at=dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(),
                  evaluation=dict(arrival_radius_m=.3, arrival_speed_mps=.15, arrival_hold_s=1,
                                  departure_displacement_m=.05, drone_radius_m=.1, collision_sample_hz=100,
                                  metric_sample_hz=100, metric_check_hz=50, flight_timeout_s=240, startup_timeout_s=90))
    config['scene'] = args.scene
    if args.scene == 'sparse_random':
        reference = json.loads(args.geometry_reference.read_text())
        parameters = dict(random_seed=args.random_seed, small_obstacle_count=args.small_obstacle_count,
                          small_position_seed=args.small_position_seed)
        if not reference.get('passed') or reference['scene_parameters'] != parameters:
            raise RuntimeError('Random geometry reference differs from requested scene')
        for name, (_, devel) in PROJECTS.items():
            if random_map_parameters(name, parameters) != reference['generator_parameters']:
                raise RuntimeError('Random launch geometry differs from preflight: '+name)
            for path, digest in reference['map_binary_sha256'][name].items():
                if binaries[name].get(path) != digest:
                    raise RuntimeError('Random map binary changed since geometry preflight')
        config.pop('wall_length_m')
        config.update(scene_parameters=parameters, generator_parameters=reference['generator_parameters'],
                      expected_boxes=reference['boxes_xyz_min_max'])
        shutil.copyfile(args.geometry_reference, output/'geometry_preflight.json')
    if args.resume:
        previous = json.loads((output/'config.json').read_text())
        for key in ('task', 'runs_per_project', 'wall_length_m', 'binaries', 'paper_sha256', 'image', 'evaluation',
                    'scene', 'scene_parameters', 'generator_parameters', 'expected_boxes'):
            fallback = 'fixed_wall' if key == 'scene' else None
            if previous.get(key, fallback) != config.get(key, fallback):
                raise RuntimeError('Resume configuration differs: '+key)
        config = previous
    else:
        save(output/'config.json', config)
    save(output/'supervisor.json', dict(pid=os.getpid(), command=sys.argv))
    summarize(output, config)
    active = None
    def interrupt(*_): raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    try:
        for trial in range(1, args.runs+1):
            for project, (repo, devel) in PROJECTS.items():
                existing = output/project/('run_%03d' % trial)
                if existing.exists():
                    old = json.loads((existing/'metrics.json').read_text())
                    if old.get('bag_valid') and old.get('reason') != 'infrastructure_error':
                        print('KEEP completed', project, trial, flush=True)
                        continue
                    raise RuntimeError('Preserve and recover incomplete worker directory before resume: '+str(existing))
                if shutil.disk_usage(output).free < 5*1024**3:
                    raise RuntimeError('Less than 5 GiB disk space left')
                for path, digest in binaries[project].items():
                    if sha(Path(path)) != digest:
                        raise RuntimeError('Planning binary changed during benchmark: '+path)
                label = 'random-pos%02d' % config['scene_parameters']['small_position_seed'] if is_random(config) else 'wall%02d' % config['wall_length_m']
                active = 'cmp-task%02d-%s-%s-%03d-%s' % (
                    config['task'], label, project, trial,
                    output.name[-6:].lower().replace('_', '-'))
                save(output/'status.json', dict(state='running', task=config['task'], **scene_fields(config),
                     project=project, trial=trial,
                     target_per_project=args.runs, active_container=active, updated_at=dt.datetime.now().isoformat()))
                command = ['docker', 'run', '--rm', '--network', 'none', '--name', active, '--user', '1000:1000']
                for mount_repo, _ in PROJECTS.values():
                    mode = '' if mount_repo == OURS else ',readonly'
                    command += ['--mount', 'type=bind,src=%s,dst=%s%s' % (mount_repo, mount_repo, mode)]
                shell = 'set -e\nsource /opt/ros/noetic/setup.bash\nsource '+shlex.quote(str(repo/devel/'setup.bash'))+'\nexec '+shlex.join([
                    '/usr/bin/python3', str(code/'run_comparison_benchmark.py'), '--worker', '--project', project,
                    '--trial', str(trial), '--output', str(output)])
                command += ['--entrypoint', '/bin/bash', args.image, '-lc', shell]
                print('START', project, trial, flush=True)
                with (output/('%s_run%03d_supervisor.log' % (project, trial))).open('w') as log:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                    try:
                        rc = process.wait(timeout=480)
                    except subprocess.TimeoutExpired:
                        subprocess.run(['docker', 'stop', '--time', '15', active], timeout=35, check=False)
                        process.wait(timeout=20)
                        raise RuntimeError('Worker wall-clock watchdog expired: '+active)
                summary = summarize(output, config)
                if rc:
                    raise RuntimeError('Worker failed: %s run %d; inspect supervisor log and metrics.json' % (project, trial))
                r = json.loads((output/project/('run_%03d' % trial)/'metrics.json').read_text())
                if not r.get('bag_valid') or r['reason'] == 'infrastructure_error':
                    raise RuntimeError('Worker output failed validation')
                print('DONE', project, trial, json.dumps(summary[project]), flush=True)
                active = None
        save(output/'status.json', dict(state='complete', task=config['task'], **scene_fields(config),
             completed_runs=args.runs*3,
             summary=summarize(output, config), finished_at=dt.datetime.now().isoformat()))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        if active:
            subprocess.run(['docker', 'stop', '--time', '15', active], timeout=35, check=False)
        save(output/'status.json', dict(state='stopped_error' if not isinstance(exc, KeyboardInterrupt) else 'interrupted',
             task=config['task'], **scene_fields(config),
             error=str(exc) or type(exc).__name__, active_container=active, summary=summarize(output, config)))
        print('STOP', str(exc), flush=True)
        return 1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--runs', type=int, default=50)
    p.add_argument('--task', type=int, default=1, help='task number used in configuration and artifact names')
    p.add_argument('--wall-length', type=int, default=6, help='first wall length along y, meters')
    p.add_argument('--scene', choices=['fixed_wall', 'sparse_random'], default='fixed_wall')
    p.add_argument('--random-seed', type=int, default=17)
    p.add_argument('--small-obstacle-count', type=int, default=8)
    p.add_argument('--small-position-seed', type=int, default=1)
    p.add_argument('--geometry-reference', type=Path, help='three-project random geometry preflight JSON')
    p.add_argument('--image', default='fishros2/ros:noetic-desktop-full')
    p.add_argument('--worker', action='store_true')
    p.add_argument('--resume', action='store_true')
    p.add_argument('--project', choices=list(PROJECTS))
    p.add_argument('--trial', type=int, default=1)
    args = p.parse_args()
    if args.runs < 1: p.error('--runs must be positive')
    if args.task < 1 or args.wall_length < 1:
        p.error('--task and --wall-length must be positive')
    if args.small_obstacle_count < 0 or args.small_position_seed < -1:
        p.error('Invalid small obstacle count or position seed')
    if args.scene == 'sparse_random' and not args.worker and args.geometry_reference is None:
        p.error('--scene sparse_random requires --geometry-reference')
    return worker(args) if args.worker else supervisor(args)


if __name__ == '__main__':
    sys.exit(main())
