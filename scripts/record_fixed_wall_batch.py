#!/usr/bin/env python3
"""Isolated ROS1 fixed-wall/random-obstacle trials and compressed metric bags.

Run with the formation_ws Noetic environment sourced; see docs/bag-experiments.md.
ROS imports are deferred so --help and metric tests work without ROS.
"""
import argparse
import csv
import datetime as dt
import json
import math
import re
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = 'rectangle_8uav_fixed_split_merge_wall.launch'
RANDOM_LAUNCH = 'rectangle_8uav_sparse_random.launch'
# An allowlist deliberately excludes PointCloud/occupancy, images, TF visualization
# and growing path/marker arrays. New topics within these interfaces are discovered.
TOPICS = (r'^/(clock|rosout|traj_start_trigger|goal|experiment/.*|'
          r'broadcast_traj_(from|to)_planner|swarm/(trajectories|agent_state|'
          r'edge_evidence|assignment|slot_swap)|formation_backend/.*|'
          r'formation_safety/.*|formation_metrics/.*|'
          r'starconvex_shadow_8uav/(state|scores|merge_scores|summary)|'
          r'subgroup_route_intent_8uav/(intents|summary)|'
          r'drone_[0-7]_visual_slam/odom|'
          r'drone_[0-7]_planning/(pos_cmd|trajectory|data_display|heartbeat)|'
          r'drone_[0-7]_traj_server/heartbeat|'
          r'drone_[0-7]/formation_applied|drone_[0-7]_formation_agent/topology)$')


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def distance(a, b):
    return norm([x-y for x, y in zip(a, b)])


def save_json(path, value):
    tmp = Path(str(path) + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    tmp.replace(path)


class Metrics:
    """Pure evaluator. Arrival needs fresh odometry AND applied-goal evidence."""
    def __init__(self, config):
        self.c = config
        self.initial, self.odom, self.goals, self.status = {}, {}, {}, {}
        self.path = [0.0]*8
        self.departure = None
        self.all_since = None
        self.arrival = [None]*8
        self.min_pair = None
        self.min_wall = None
        self.max_speed = [0.0]*8
        self.collisions = []
        self.clearance_violations = set()
        self.assignment = list(range(8))
        self.assignment_epoch = 0
        self.assignment_changes = 0
        self.applied_slots = list(range(8))
        self.obstacle_boxes = None if config.get('scene') == 'sparse_random' else [
            (-2.25, -1.75, -config['wall_length']/2, config['wall_length']/2, -0.1, 3.0),
            (6.25, 6.75, -5.0, -0.9, -0.1, 3.0),
            (6.25, 6.75, 0.9, 5.0, -0.1, 3.0)]
        self.online_formation = None

    def update_odom(self, i, t, p, v):
        if not all(math.isfinite(x) for x in (*p, *v, t)):
            return
        if i in self.odom:
            self.path[i] += distance(p, self.odom[i][1])
        self.initial.setdefault(i, p)
        self.odom[i] = (t, p, v)
        self.max_speed[i] = max(self.max_speed[i], norm(v))
        # Initial-position displacement avoids treating estimator velocity noise
        # or initial message delivery as departure. Timing error <= 5 cm travel.
        if self.departure is None and distance(p, self.initial[i]) >= self.c['departure_distance']:
            self.departure = t

    def update_goal(self, i, t, goal, slots, epoch):
        goal = tuple(goal)
        if len(slots) != 8 or sorted(slots) != list(range(8)):
            return
        if i in self.goals and distance(goal, self.goals[i][1]) > 1e-6:
            self.all_since = None
            self.arrival[i] = None
        self.goals[i] = (t, goal)
        self.applied_slots[i] = slots[i]
        if epoch > self.assignment_epoch:
            if list(slots) != self.assignment:
                self.assignment_changes += 1
            self.assignment_epoch, self.assignment = epoch, list(slots)

    def wall_distance(self, p):
        # Random scenes install their real boxes from the latched geometry
        # message. Geometry is saved as small JSON metadata, never as a cloud.
        if not self.obstacle_boxes:
            return None
        return min(norm((max(lo_x-p[0], 0, p[0]-hi_x),
                         max(lo_y-p[1], 0, p[1]-hi_y),
                         max(lo_z-p[2], 0, p[2]-hi_z)))
                   for lo_x, hi_x, lo_y, hi_y, lo_z, hi_z in self.obstacle_boxes)

    def tick(self, t):
        fresh = {i: x for i, x in self.odom.items() if 0 <= t-x[0] <= 0.5}
        for i, (_, p, _) in fresh.items():
            clearance = self.wall_distance(p)
            if clearance is not None:
                self.min_wall = clearance if self.min_wall is None else min(self.min_wall, clearance)
            if clearance is not None and clearance < 0.2:
                self.clearance_violations.add(i)
            if clearance is not None and clearance <= 1e-6 and ('wall', i) not in self.collisions:
                self.collisions.append(('wall', i))
            for j in fresh:
                if j <= i:
                    continue
                d = distance(p, fresh[j][1])
                self.min_pair = d if self.min_pair is None else min(self.min_pair, d)
                if d < self.c['pair_collision_distance'] and ('pair', i, j) not in self.collisions:
                    self.collisions.append(('pair', i, j))
        all_arrived = len(fresh) == 8 and self.departure is not None
        for i in range(8):
            arrived = False
            if i in fresh and i in self.goals and i in self.status:
                _, p, v = fresh[i]
                status_t, state, unsafe = self.status[i]
                arrived = (0 <= t-self.goals[i][0] <= 1.5 and
                           0 <= t-status_t <= 1.5 and state == 'WAIT_TARGET' and
                           not unsafe and distance(p, self.goals[i][1]) <= self.c['arrival_radius'] and
                           norm(v) <= self.c['arrival_speed'])
            if not arrived:
                self.arrival[i] = None
                all_arrived = False
            elif self.arrival[i] is None:
                self.arrival[i] = t
        if not all_arrived:
            self.all_since = None
        elif self.all_since is None:
            self.all_since = t
        return self.all_since is not None and t-self.all_since >= self.c['arrival_hold']

    def result(self, reason, t):
        reached = reason == 'all_arrived'
        success = reached and not self.collisions and self.obstacle_boxes is not None
        return dict(success=success, all_arrived=reached,
                    reason=('collision_detected' if reached and self.collisions else reason),
                    departure_ros_time=self.departure,
                    all_arrived_ros_time=self.all_since if reached else None,
                    completion_time_s=(self.all_since-self.departure if success else None),
                    observed_flight_time_s=(t-self.departure if self.departure else None),
                    arrival_ros_times=self.arrival, path_lengths_m=self.path,
                    max_speeds_mps=self.max_speed, min_pair_distance_m=self.min_pair,
                    min_wall_distance_m=self.min_wall,
                    min_obstacle_distance_m=self.min_wall,
                    obstacle_geometry_received=self.obstacle_boxes is not None,
                    recording_end_ros_time=t,
                    formation_error_online_20hz=self.online_formation,
                    sampled_collisions=self.collisions,
                    wall_clearance_below_0_2m=sorted(self.clearance_violations),
                    final_positions={i: x[1] for i, x in self.odom.items()},
                    final_goals={i: x[1] for i, x in self.goals.items()},
                    final_drone_to_slot=self.applied_slots,
                    assignment_changes=self.assignment_changes)


def monitor(run):
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from ego_planner.msg import FormationTopologyAck
    from visualization_msgs.msg import MarkerArray
    import numpy as np
    from analyze_paper_formation_error import similarity_residual
    config = json.loads((run.parent/'config.json').read_text())
    m = Metrics(config)
    lock = threading.RLock()
    rospy.init_node('experiment_metrics', disable_signals=True)
    event = rospy.Publisher('/experiment/events', String, queue_size=10, latch=True)
    metric_pub = rospy.Publisher('/experiment/metrics', String, queue_size=10, latch=True)
    formation_pub = rospy.Publisher('/experiment/formation_error', String, queue_size=10, latch=True)
    subs = []
    def odom(msg, i):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        t = msg.header.stamp.to_sec() or rospy.get_time()
        with lock:
            m.update_odom(i, t, (p.x, p.y, p.z), (v.x, v.y, v.z))
    def applied(msg, i):
        with lock:
            if msg.goal_valid:
                m.update_goal(i, rospy.get_time(), msg.assigned_goal,
                              msg.drone_to_slot, msg.assignment_epoch)
            elif i in m.goals:
                # FSM clears have_target on completion; keep the last applied
                # endpoint while fresh acknowledgements prove the planner lives.
                m.goals[i] = (rospy.get_time(), m.goals[i][1])
    def safety(msg, i):
        fields = dict(x.split('=', 1) for x in msg.data.split() if '=' in x)
        with lock:
            m.status[i] = (rospy.get_time(), fields.get('state'),
                           fields.get('wall_guard') != '0' or fields.get('map_occ') != '0')
    def geometry(msg):
        boxes = []
        for marker in msg.markers:
            if marker.type != marker.CUBE or marker.action != marker.ADD:
                continue
            if any(abs(x) > 1e-8 for x in (marker.pose.orientation.x,
                                          marker.pose.orientation.y, marker.pose.orientation.z)):
                raise ValueError('Only axis-aligned obstacle boxes are supported')
            p, s = marker.pose.position, marker.scale
            boxes.append((p.x-s.x/2, p.x+s.x/2, p.y-s.y/2, p.y+s.y/2, p.z-s.z/2, p.z+s.z/2))
        if boxes:
            with lock:
                m.obstacle_boxes = boxes
                save_json(run/'obstacle_geometry.json', {'boxes_xyz_min_max': boxes,
                          'source': '/map_generator/sparse_random_geometry', 'frame': 'world'})
    if config.get('scene') == 'sparse_random':
        subs.append(rospy.Subscriber('/map_generator/sparse_random_geometry', MarkerArray, geometry, queue_size=1))
    for i in range(8):
        subs.extend([
            rospy.Subscriber('/drone_%d_visual_slam/odom' % i, Odometry, odom, i, queue_size=100),
            rospy.Subscriber('/drone_%d/formation_applied' % i, FormationTopologyAck, applied, i),
            rospy.Subscriber('/formation_safety/drone_%d/status' % i, String, safety, i)])
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    (run/'monitor.ready').touch()
    begin, last_metric, departed = time.monotonic(), 0, False
    reason = 'interrupted'
    previous_formation = None
    formation_integral = formation_length = 0.0
    with (run/config.get('positions_file', 'positions.csv')).open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['sample_ros_time', 'odom_ros_time', 'drone_id', 'x', 'y', 'z',
                         'vx', 'vy', 'vz', 'slot_id', 'goal_x', 'goal_y', 'goal_z'])
        while not stop.is_set() and not rospy.is_shutdown():
            now = rospy.get_time()
            with lock:
                complete = m.tick(now)
                for i, (stamp, p, v) in sorted(m.odom.items()):
                    if 0 <= now-stamp <= 0.5:
                        goal = m.goals.get(i, (None, ('', '', '')))[1]
                        writer.writerow([now, stamp, i, *p, *v, m.applied_slots[i], *goal])
                if m.departure is not None and len(m.goals) == 8 and len(m.odom) == 8 and all(
                        0 <= now-m.odom[i][0] <= 0.5 for i in range(8)):
                    points = np.asarray([m.odom[i][1] for i in range(8)])
                    desired = np.asarray([m.goals[i][1] for i in range(8)])
                    value, scale = similarity_residual(points, desired)
                    center = points.mean(axis=0)
                    if previous_formation is not None:
                        old_center, old_value = previous_formation
                        step = float(np.linalg.norm(center-old_center))
                        formation_integral += step*(old_value+float(value))/2
                        formation_length += step
                    previous_formation = (center, float(value))
                    m.online_formation = dict(instantaneous_squared_sum_m2=float(value),
                        e_dist_eq45=(formation_integral/formation_length if formation_length > 1e-10 else None),
                        centroid_path_length_m=formation_length, alignment_scale=float(scale),
                        s0=1.0, sample_hz=20, authoritative_offline_hz=100)
                    formation_pub.publish(String(data=json.dumps(m.online_formation)))
                if m.departure is not None and not departed:
                    departed = True
                    event.publish(String(data=json.dumps({'event': 'departure', 'ros_time': m.departure})))
                if now-last_metric > 1:
                    metric_pub.publish(String(data=json.dumps(m.result('running', now))))
                    last_metric = now
                    f.flush()
                    save_json(run/'progress.json', m.result('running', now))
                if complete:
                    reason = 'all_arrived'
                    break
                if m.departure is None and time.monotonic()-begin > config['startup_timeout']:
                    reason = 'no_departure'
                    break
                if m.departure is not None and now-m.departure > config['timeout']:
                    reason = 'flight_timeout'
                    break
                if m.departure is not None and any(i not in m.odom or now-m.odom[i][0] > 5 for i in range(8)):
                    reason = 'odometry_lost'
                    break
            stop.wait(0.05)
    result = m.result(reason, rospy.get_time())
    metric_pub.publish(String(data=json.dumps(result)))
    event.publish(String(data=json.dumps({'event': 'finish', **result})))
    time.sleep(0.3)
    save_json(run/'result.json', result)
    rospy.signal_shutdown('trial finished')


def spawn(cmd, log, env):
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)


def stop_process(p):
    if p is None or p.poll() is not None:
        return
    # A process already in kernel VMA teardown has finished user code but may
    # not be reapable. This host occasionally blocks there; waiting cannot
    # finish its already committed experiment data or release the kernel wait.
    try:
        if Path('/proc/%d/wchan' % p.pid).read_text().strip() == 'unlink_file_vma_batch_process':
            return
    except OSError:
        pass
    for sig, timeout in [(signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)]:
        try:
            os.killpg(p.pid, sig)
            p.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def wait_until(predicate, timeout, processes=()):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('required process exited; inspect logs')
        try:
            if predicate():
                return
        except (OSError, xmlrpc.client.Error):
            pass
        time.sleep(0.2)
    raise RuntimeError('startup readiness timeout')


def aggregate(output):
    rows = []
    for path in sorted(output.glob('trial_*/result.json')):
        result = json.loads(path.read_text())
        result['trial'] = path.parent.name
        result['bag_path'] = str(Path(path.parent.name)/result.get('bag_file', 'flight.bag'))
        result['positions_path'] = str(Path(path.parent.name)/result.get('positions_file', 'positions.csv'))
        rows.append(result)
    valid = [r for r in rows if r.get('reason') not in ('infrastructure_error', 'interrupted')]
    successes = [r for r in valid if r['success']]
    summary = dict(completed_trials=len(rows), valid_trials=len(valid),
                   infrastructure_errors=len(rows)-len(valid), successful_trials=len(successes),
                   success_rate=(len(successes)/len(valid) if valid else None),
                   success_rate_all_attempts=(len(successes)/len(rows) if rows else None),
                   mean_completion_time_s=(sum(r['completion_time_s'] for r in successes)/len(successes)
                                           if successes else None))
    errors = [r['formation_error_eq45'] for r in successes if r.get('formation_error_eq45') is not None]
    summary['mean_formation_error_eq45_successful_m2'] = sum(errors)/len(errors) if errors else None
    summary['formation_error_successful_trials'] = len(errors)
    save_json(output/'summary.json', summary)
    fields = ['trial', 'bag_path', 'positions_path', 'success', 'reason', 'completion_time_s', 'observed_flight_time_s',
              'min_pair_distance_m', 'min_obstacle_distance_m', 'assignment_changes',
              'formation_error_eq45', 'formation_error_eq46', 'bag_bytes']
    with (output/'summary.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    return summary


def launch_configuration(args):
    launch_name = RANDOM_LAUNCH if args.scene == 'sparse_random' else LAUNCH
    launch_path = ROOT/'swarm-playground/formation_ws/src/planner/plan_manage/launch'/launch_name
    defaults = {x.attrib['name']: x.attrib['default'] for x in ET.parse(launch_path).getroot().findall('arg')
                if 'default' in x.attrib}
    swap_enabled = defaults['swap_enabled'] == 'true' if args.swap_enabled is None else args.swap_enabled
    cmd = ['roslaunch', 'ego_planner', launch_name, 'rviz:=false', 'coordination_mode:=distributed',
           'swap_enabled:='+str(swap_enabled).lower(), 'swap_scope:='+defaults['swap_scope']]
    seed = None
    if args.scene == 'sparse_random':
        seed = int(defaults['random_seed']) if args.random_seed is None else args.random_seed
        cmd.append('random_seed:='+str(seed))
        for parameter in ('small_obstacle_count', 'small_position_seed'):
            value = getattr(args, parameter, None)
            if value is None:
                value = int(defaults[parameter])
            cmd.append(parameter+':='+str(value))
    else:
        cmd.append('wall_size_y:='+str(args.wall_length))
    return launch_name, cmd, swap_enabled, seed


def batch(args):
    if os.environ.get('ROS_VERSION') != '1':
        raise SystemExit('Source /opt/ros/noetic/setup.bash and formation_ws/devel/setup.bash first.')
    # Keep independent runs isolated; never reuse an existing master or journal.
    if args.port:
        with socket.socket() as s:
            s.bind(('127.0.0.1', args.port))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=args.resume)
    config = vars(args).copy()
    launch_name, launch_cmd, swap_enabled, random_seed = launch_configuration(args)
    config['output'] = str(output)
    config['launch'] = launch_name
    config['swap_enabled'] = swap_enabled
    config['random_seed'] = random_seed
    if args.scene == 'sparse_random':
        values = dict(value.split(':=', 1) for value in launch_cmd if ':=' in value)
        config['small_obstacle_count'] = int(values['small_obstacle_count'])
        config['small_position_seed'] = int(values['small_position_seed'])
    config['positions_file'] = 'drone_positions.csv' if args.experiment_label else 'positions.csv'
    config['topic_regex'] = TOPICS
    previous = json.loads((output/'config.json').read_text()) if args.resume else None
    config['git_commit'] = (previous['git_commit'] if previous else
                            subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip())
    config['created_at'] = dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat()
    snapshot_tag = 'resume_' if args.resume else ''
    if args.resume:
        for key in ('runs', 'scene', 'random_seed', 'small_obstacle_count', 'small_position_seed',
                    'experiment_label', 'swap_enabled', 'timeout', 'startup_timeout',
                    'arrival_radius', 'arrival_speed', 'arrival_hold', 'departure_distance',
                    'pair_collision_distance'):
            if previous.get(key) != config.get(key):
                raise ValueError('Resume configuration mismatch: '+key)
        save_json(output/'resume_config.json', config)
        config = previous
    else:
        save_json(output/'config.json', config)
    if not args.resume:
        with (output/'source.diff').open('w') as f:
            subprocess.run(['git', '-C', str(ROOT), 'diff'], stdout=f, check=True)
    (output/(snapshot_tag+'recorder_source.py')).write_bytes(Path(__file__).read_bytes())
    (output/(snapshot_tag+'analysis_source.py')).write_bytes((ROOT/'scripts/analyze_paper_formation_error.py').read_bytes())
    print('OUTPUT=' + str(output), flush=True)
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port, ROS_IP='127.0.0.1')
    env.pop('ROS_HOSTNAME', None)
    previous_handler = signal.getsignal(signal.SIGTERM)
    def interrupt_batch(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt_batch)
    try:
        for trial in range(1, args.runs+1):
            run = output/('trial_%02d' % trial)
            if run.exists() and args.resume:
                old = json.loads((run/'result.json').read_text())
                if (old.get('reason') not in ('infrastructure_error', 'interrupted') and
                        old.get('bag_bytes', 0) > 0 and (run/old.get('bag_file', 'flight.bag')).exists() and
                        old.get('formation_error_status') in ('ok', 'not_available')):
                    print('RESUME keeping completed '+run.name, flush=True)
                    continue
                raise RuntimeError('Recover incomplete trial before resuming: '+str(run))
            run.mkdir()
            if args.port == 0:
                with socket.socket() as reservation:
                    reservation.bind(('127.0.0.1', 0))
                    trial_port = reservation.getsockname()[1]
            else:
                trial_port = args.port
            env['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % trial_port
            save_json(run/'ros_master.json', dict(port=trial_port, uri=env['ROS_MASTER_URI']))
            bag_name = (args.experiment_label+'_run%02d.bag' % trial) if args.experiment_label else 'flight.bag'
            bag_path = run/bag_name
            save_json(run/'recording_files.json', dict(bag_file=bag_name, positions_file=config['positions_file'],
                                                      metrics_file='result.json', trial=trial))
            env['ROS_LOG_DIR'] = str(run/'ros_logs')
            env['ROS_HOME'] = str(run/'ros_home')
            master = recorder = worker = launch = None
            handles = []
            interrupted = False
            error = None
            print('START trial=%02d/%d' % (trial, args.runs), flush=True)
            try:
                def logfile(name):
                    f = (run/(name+'.log')).open('w')
                    handles.append(f)
                    return f
                master = spawn(['roscore', '-p', str(trial_port)], logfile('master'), env)
                proxy = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
                wait_until(lambda: proxy.getPid('/batch_recorder')[0] == 1, 25, [master])
                recorder = spawn(['rosbag', 'record', '--lz4', '--buffsize=256', '--chunksize=768',
                                  '-O', str(bag_path), '-e', TOPICS,
                                  '__name:=experiment_bag_recorder'], logfile('recorder'), env)
                worker = spawn([sys.executable, str(Path(__file__).resolve()), '--monitor', str(run)],
                               logfile('metrics'), env)
                wait_until(lambda: (run/'monitor.ready').exists() and
                           proxy.lookupNode('/batch', '/experiment_bag_recorder')[0] == 1,
                           30, [master, recorder, worker])
                # Recorder and metric subscribers exist before any vehicle starts.
                time.sleep(1)
                save_json(run/'command.json', launch_cmd)
                launch = spawn(launch_cmd, logfile('launch'), env)
                parameter = ('/sparse_random_obstacle_map/seed' if args.scene == 'sparse_random' else
                             '/fixed_split_merge_wall_map/first_wall_size_y')
                wait_until(lambda: proxy.hasParam('/batch', parameter)[2],
                           30, [master, recorder, launch])
                actual = proxy.getParam('/batch', parameter)[2]
                if actual != (random_seed if args.scene == 'sparse_random' else args.wall_length):
                    raise RuntimeError('scene configuration mismatch')
                if args.scene == 'sparse_random':
                    for name in ('small_obstacle_count', 'small_position_seed'):
                        actual = proxy.getParam('/batch', '/sparse_random_obstacle_map/'+name)[2]
                        if actual != config[name]:
                            raise RuntimeError('scene configuration mismatch: '+name)
                    wait_until(lambda: (run/'obstacle_geometry.json').exists(), 30,
                               [master, recorder, launch, worker])
                # Query the master directly. A separate rosparam interpreter can
                # finish writing but block in this host's kernel exit path.
                import yaml
                code, message, parameters = proxy.getParam('/batch', '/')
                if code != 1:
                    raise RuntimeError('parameter snapshot failed: '+message)
                (run/'params.yaml').write_text(yaml.safe_dump(parameters))
                deadline = time.monotonic()+args.startup_timeout+args.timeout+40
                while worker.poll() is None:
                    if (run/'result.json').exists():
                        # The monitor writes this only after its final event and
                        # CSV flush. Kernel exit delays must not extend a flight.
                        break
                    if launch.poll() is not None or recorder.poll() is not None or master.poll() is not None:
                        raise RuntimeError('simulation, recorder or master exited')
                    if time.monotonic() > deadline:
                        raise RuntimeError('monitor exceeded wall-clock watchdog')
                    time.sleep(0.5)
                if not (run/'result.json').exists():
                    raise RuntimeError('metrics process failed')
            except KeyboardInterrupt:
                interrupted = True
                error = 'interrupted'
            except Exception as exc:
                error = str(exc)
            finally:
                stop_process(worker)
                stop_process(launch)
                stop_process(recorder)
                stop_process(master)
                for f in handles:
                    f.close()
            result_path = run/'result.json'
            result = json.loads(result_path.read_text()) if result_path.exists() else {}
            result.update(bag_file=bag_name, positions_file=config['positions_file'])
            if error:
                result.update(success=False, reason='interrupted' if interrupted else 'infrastructure_error', error=error)
            result['bag_bytes'] = bag_path.stat().st_size if bag_path.exists() else 0
            if not result['bag_bytes'] or Path(str(bag_path)+'.active').exists():
                result.update(success=False, reason='infrastructure_error',
                              error=result.get('error', 'bag missing or not finalized'))
            if result['bag_bytes']:
                try:
                    import rosbag
                    with rosbag.Bag(str(bag_path)) as bag:
                        info = bag.get_type_and_topic_info().topics
                        save_json(run/'topics.json', {k: {'type': v.msg_type, 'count': v.message_count}
                                                     for k, v in info.items()})
                        forbidden = [k for k, v in info.items() if v.msg_type in (
                            'sensor_msgs/PointCloud2', 'sensor_msgs/PointCloud',
                            'sensor_msgs/Image', 'sensor_msgs/CompressedImage', 'nav_msgs/OccupancyGrid')]
                        missing = ['/drone_%d_visual_slam/odom' % i for i in range(8)
                                   if '/drone_%d_visual_slam/odom' % i not in info]
                        if forbidden or missing:
                            result.update(success=False, reason='infrastructure_error',
                                          error='invalid bag topics', forbidden=forbidden, missing=missing)
                except Exception as exc:
                    result.update(success=False, reason='infrastructure_error', error='bag validation: '+str(exc))
            save_json(result_path, result)
            if not error and result.get('departure_ros_time') is not None and result['bag_bytes']:
                try:
                    with (run/'analysis.log').open('w') as f:
                        marker = run/'analysis_complete.json'
                        analysis_process = spawn(
                            [sys.executable, str(ROOT/'scripts/analyze_paper_formation_error.py'), str(output),
                             '--include-failures', '--incremental', '--completion-marker', str(marker)], f, env)
                        try:
                            wait_until(lambda: marker.exists(), 120)
                            if not json.loads(marker.read_text()).get('complete'):
                                raise RuntimeError('Analysis completion marker invalid')
                        finally:
                            stop_process(analysis_process)
                    audit = json.loads((output/'formation_error_paper'/(run.name+'_audit.json')).read_text())
                    result.update(formation_error_eq45=audit['e_dist_eq45'],
                                  formation_error_eq46=audit['e_sim_eq46'],
                                  formation_error_interval=audit['interval'],
                                  formation_error_status='ok')
                except Exception as exc:
                    result.update(formation_error_status='error', formation_error_error=str(exc))
                    save_json(result_path, result)
                    aggregate(output)
                    print('ANALYSIS ERROR: '+str(exc), flush=True)
                    return 1
            else:
                result.update(formation_error_status='not_available')
            save_json(result_path, result)
            summary = aggregate(output)
            print('DONE trial=%02d %s bytes=%d summary=%s' %
                  (trial, result['reason'], result['bag_bytes'], json.dumps(summary)), flush=True)
            # Never burn the remaining 19 attempts on a broken launch/recorder.
            if interrupted or result['reason'] == 'infrastructure_error':
                return 1
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
    save_json(output/'batch_complete.json', dict(complete=True, trials=args.runs, summary=aggregate(output)))
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs', type=int, default=20)
    p.add_argument('--scene', choices=['fixed_wall', 'sparse_random'], default='fixed_wall')
    p.add_argument('--random-seed', type=int, help='default: selected launch seed, repeated for every trial')
    p.add_argument('--small-obstacle-count', type=int, help='random scene: number of small blocks')
    p.add_argument('--small-position-seed', type=int, help='random scene: independent small-block position seed')
    p.add_argument('--experiment-label', help='readable prefix in every bag filename')
    p.add_argument('--wall-length', type=float, default=8.0)
    swaps = p.add_mutually_exclusive_group()
    swaps.add_argument('--swap-enabled', action='store_true', dest='swap_enabled')
    swaps.add_argument('--no-swap', action='store_false', dest='swap_enabled')
    p.set_defaults(swap_enabled=None)
    p.add_argument('--timeout', type=float, default=240.0, help='flight timeout in ROS seconds')
    p.add_argument('--startup-timeout', type=float, default=90.0)
    p.add_argument('--arrival-radius', type=float, default=0.30)
    p.add_argument('--arrival-speed', type=float, default=0.15)
    p.add_argument('--arrival-hold', type=float, default=1.0)
    p.add_argument('--departure-distance', type=float, default=0.05)
    p.add_argument('--pair-collision-distance', type=float, default=0.20)
    p.add_argument('--port', type=int, default=11420)
    p.add_argument('--output', type=Path)
    p.add_argument('--resume', action='store_true', help='continue a matching batch, preserving all completed trials')
    p.add_argument('--monitor', type=Path, help=argparse.SUPPRESS)
    args = p.parse_args()
    if args.monitor:
        monitor(args.monitor)
        return 0
    if args.output is None:
        args.output = ROOT/'bags'/((args.experiment_label or args.scene)+'_'+dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S'))
    if args.experiment_label and not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', args.experiment_label):
        p.error('experiment label must contain only letters, digits, underscores or hyphens (max 100)')
    if args.small_obstacle_count is not None and args.small_obstacle_count < 0:
        p.error('small obstacle count cannot be negative')
    if args.runs < 1 or any(getattr(args, x) <= 0 for x in (
            'wall_length', 'timeout', 'startup_timeout', 'arrival_radius', 'arrival_speed',
            'arrival_hold', 'departure_distance', 'pair_collision_distance')):
        p.error('counts and thresholds must be positive')
    return batch(args)


if __name__ == '__main__':
    sys.exit(main())
