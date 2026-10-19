#!/usr/bin/env python3
"""Execution-feedback checks against one isolated real planner, no simulator.

Requires an empty disposable master at http://127.0.0.1:11342. Phase one keeps
the distributed barrier closed while injecting malformed/foreign execution
packets. With --allow-synthetic-flight, phase two releases only the logical
planner barrier, requires fresh-heartbeat WAITING readiness, injects a HOLDING
executor and moved fresh odometry, and checks replacement plus executor-restart
handshakes. Known-peer status and actual-odometry expiry must explicitly hold.
There is no trajectory server, aircraft controller, or simulated vehicle.
"""
import argparse
import copy
import math
import os
import re
import signal
import subprocess
import tempfile
import threading
import time

import roslaunch.config
import rosnode
import rospkg
import rospy
from ego_planner.msg import FormationTopologyAck, FormationTopologyCommand
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, Header, String
from traj_utils.msg import MINCOTraj, PolyTraj, TrajectoryExecution


def has_motion(trajectory):
    if any(abs(value) > 1e-8 for value in list(trajectory.start_v) +
           list(trajectory.start_a) + list(trajectory.end_v) + list(trajectory.end_a)):
        return True
    if any(abs(a - b) > 1e-8 for a, b in zip(trajectory.start_p, trajectory.end_p)):
        return True
    return any(abs(value - trajectory.start_p[axis]) > 1e-8
               for axis, coordinates in enumerate(
                   (trajectory.inner_x, trajectory.inner_y, trajectory.inner_z))
               for value in coordinates)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-synthetic-flight', action='store_true')
    args = parser.parse_args()
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11342', (
        'Execution injection is restricted to the dedicated localhost:11342 master')
    rospy.init_node('test_execution_planner_runtime', anonymous=True)
    assert not any(node.startswith('/drone_') for node in rosnode.get_node_names()), (
        'Use an empty disposable master; do not attach to a running experiment')

    package = rospkg.RosPack().get_path('ego_planner')
    launch_file = os.path.join(package, 'launch', 'rectangle_8uav_member.launch')
    arguments = ['distributed_enabled:=true', 'swap_enabled:=false',
                 'drone_id:=0', 'init_x:=-5.5', 'init_y:=-1.5',
                 'map_size_x:=24', 'map_size_y:=14', 'map_size_z:=3',
                 'start_x:=-6', 'goal_x:=8']
    config = roslaunch.config.load_config_default(
        [(launch_file, arguments)], None, assign_machines=False)
    planner = next(node for node in config.nodes if node.name == 'drone_0_ego_planner_node')
    hold_topic = dict(planner.remap_args).get(
        '~planning/hold', '/drone_0_ego_planner_node/planning/hold')
    for parameter in config.params.values():
        rospy.set_param(parameter.key, parameter.value)
    rospy.set_param('/drone_0_ego_planner_node/formation/distributed_startup_timeout', 60.0)
    # Isolate execution-triggered replanning from the normal periodic replan.
    rospy.set_param('/drone_0_ego_planner_node/fsm/thresh_replan_time', 1000.0)

    lock = threading.Lock()
    applied, local_trajectories, local_polynomials, safety, hold_requests = [], [], [], [], []

    def record(target, message, own_only=False):
        if own_only and message.drone_id != 0:
            return
        with lock:
            target.append((time.monotonic(), message))

    subscribers = [
        rospy.Subscriber('/drone_0/formation_applied', FormationTopologyAck,
                         lambda msg: record(applied, msg), queue_size=20),
        rospy.Subscriber('/swarm/trajectories', MINCOTraj,
                         lambda msg: record(local_trajectories, msg, True), queue_size=50),
        rospy.Subscriber('/drone_0_planning/trajectory', PolyTraj,
                         lambda msg: record(local_polynomials, msg), queue_size=50),
        rospy.Subscriber('/formation_safety/drone_0/status', String,
                         lambda msg: record(safety, msg), queue_size=50),
        rospy.Subscriber(hold_topic, Empty,
                         lambda msg: record(hold_requests, msg), queue_size=50),
    ]
    command_pub = rospy.Publisher('/drone_0_formation_agent/topology',
                                  FormationTopologyCommand, queue_size=1)
    stop_topic = dict(planner.remap_args).get(
        '~mandatory_stop', '/drone_0_ego_planner_node/mandatory_stop')
    stop_pub = rospy.Publisher(stop_topic, Empty, queue_size=1)
    odom_pub = rospy.Publisher('/drone_0_visual_slam/odom', Odometry, queue_size=1)
    cloud_pub = rospy.Publisher('/drone_0_pcl_render_node/cloud', PointCloud2, queue_size=1)
    execution_pub = rospy.Publisher('/swarm/execution_state', TrajectoryExecution, queue_size=30)
    command = FormationTopologyCommand()
    command.mode = FormationTopologyCommand.SPLIT
    command.edge_count = 10
    command.drone_to_slot = list(range(8))
    command.assignment_epoch = 1
    command.assignment_digest = 1
    command.formation_scale = 0.15
    command.components = '|'.join('{%d}' % i for i in range(8))
    command.reason = 'DISTRIBUTED_STARTUP_WAIT'
    position = [-5.5, -1.5, 1.5]
    velocity = [0.0, 0.0, 0.0]
    session = 4242001
    sequences = [0] * 8
    automatic_running = False
    waiting_ready_enabled = False
    force_waiting_ready = False
    ready_heartbeat_fresh = True
    odometry_enabled = True
    foreign_execution_enabled = True
    foreign_execution_factory = None
    extra_execution = None
    last_cloud = 0.0

    def latest(target):
        with lock:
            return target[-1][1] if target else None

    def metadata():
        status = latest(safety)
        return {} if status is None else dict(re.findall(r'([A-Za-z_]+)=([^ ]+)', status.data))

    def packet(trajectory=None, drone=0, mode=None):
        sequences[drone] += 1
        now = rospy.Time.now()
        msg = TrajectoryExecution()
        msg.header.stamp = now
        msg.header.frame_id = 'world'
        msg.session = session if drone == 0 else 4242001 + drone
        msg.sequence = sequences[drone]
        msg.drone_id = drone
        msg.trajectory_id = trajectory.traj_id if trajectory is not None else 999
        msg.trajectory_start_time = trajectory.start_time if trajectory is not None else now
        msg.trajectory_duration = (sum(trajectory.duration) if trajectory is not None else 2.0)
        msg.elapsed = max(0.0, min(msg.trajectory_duration,
                                  (now - msg.trajectory_start_time).to_sec()))
        msg.position = list(position)
        if drone != 0:
            # Foreign execution is not our own executor. Keep its possible
            # conservative prediction away from this test's forward route.
            msg.position = [-5.5, 4.0, 1.5]
        msg.velocity = list(velocity)
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.mode = TrajectoryExecution.RUNNING if mode is None else mode
        msg.require_replan = msg.mode == TrajectoryExecution.HOLDING
        msg.heartbeat_fresh = True
        if msg.mode in (TrajectoryExecution.HOLDING, TrajectoryExecution.FINISHED):
            msg.velocity = [0.0, 0.0, 0.0]
        return msg

    def waiting_packet():
        msg = packet(mode=TrajectoryExecution.WAITING)
        msg.trajectory_id = 0
        msg.trajectory_start_time = rospy.Time(0)
        msg.trajectory_duration = 0.0
        msg.elapsed = 0.0
        # WAITING is readiness only, never a spatial peer/own position.
        msg.position = [0.0, 0.0, 0.0]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.require_replan = False
        msg.heartbeat_fresh = ready_heartbeat_fresh
        return msg

    def pump(seconds=0.2):
        nonlocal last_cloud
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert process.poll() is None, 'planner exited during execution-feedback test'
            now = rospy.Time.now()
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = now, 'world'
            odom.pose.pose.orientation.w = 1
            (odom.pose.pose.position.x, odom.pose.pose.position.y,
             odom.pose.pose.position.z) = position
            (odom.twist.twist.linear.x, odom.twist.twist.linear.y,
             odom.twist.twist.linear.z) = velocity
            if odometry_enabled:
                odom_pub.publish(odom)
            command.header.stamp = now
            command_pub.publish(command)
            if time.monotonic() - last_cloud > 0.1:
                cloud_pub.publish(point_cloud2.create_cloud_xyz32(
                    Header(stamp=now, frame_id='world'), [(-5.5, 4.0, 1.5)]))
                last_cloud = time.monotonic()
            current = latest(local_trajectories)
            if waiting_ready_enabled and (current is None or force_waiting_ready):
                execution_pub.publish(waiting_packet())
            elif automatic_running and current is not None:
                execution_pub.publish(packet(current))
            # Once any peer executor has been observed, silence must remain
            # fail-closed. Keep this deliberately far-away held peer fresh in
            # both phases instead of accidentally testing stale-peer blocking.
            if foreign_execution_enabled:
                execution_pub.publish(foreign_execution_factory() if foreign_execution_factory
                                      else packet(drone=1, mode=TrajectoryExecution.HOLDING))
            if extra_execution is not None:
                execution_pub.publish(extra_execution())
            time.sleep(0.02)

    def wait_for(description, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pump(0.1)
            if predicate():
                return
        raise AssertionError('%s: safety=%s, local trajectories=%s' % (
            description, metadata(), len(local_trajectories)))

    log = tempfile.TemporaryFile(mode='w+')
    argv = ['rosrun', planner.package, planner.type, '__name:=' + planner.name]
    argv.extend('%s:=%s' % (source, target) for source, target in planner.remap_args)
    process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
    try:
        wait_for('planner target available behind closed startup barrier',
                 lambda: latest(applied) is not None and latest(applied).goal_valid)
        wait_for('execution feedback subscriber connected',
                 lambda: execution_pub.get_num_connections() > 0)
        # No valid local trajectory exists. None of these packets may invent one
        # or cause a HOLDING callback to bypass WAIT_TARGET and the startup gate.
        cases = [
            ('unknown own HOLDING', lambda: packet(mode=TrajectoryExecution.HOLDING)),
            ('foreign HOLDING', lambda: packet(drone=1, mode=TrajectoryExecution.HOLDING)),
            ('unknown own FINISHED', lambda: packet(mode=TrajectoryExecution.FINISHED)),
        ]
        for description, factory in cases:
            extra_execution = factory
            pump(0.3)
            assert not local_trajectories, description + ' bypassed the flight barrier'
        bad = packet(mode=TrajectoryExecution.HOLDING)
        bad.session = 0
        extra_execution = lambda: bad
        pump(0.3)
        bad = packet(mode=TrajectoryExecution.HOLDING)
        bad.header.stamp = rospy.Time.now() + rospy.Duration(10)
        extra_execution = lambda: bad
        pump(0.3)
        bad = packet(mode=TrajectoryExecution.HOLDING)
        bad.position[0] = float('nan')
        extra_execution = lambda: bad
        pump(0.3)
        extra_execution = None
        assert not local_trajectories, 'invalid packets released the flight barrier'
        assert metadata().get('state') == 'WAIT_TARGET', metadata()
        assert metadata().get('execution_session') == '0', (
            'unknown/foreign packets became our accepted execution state', metadata())
        print('PASS: six execution packet cases cannot bypass startup barrier', flush=True)
        if not args.allow_synthetic_flight:
            return

        # This releases only the planner in this empty test graph, not a vehicle.
        command.edge_keep_mask = (1 << 10) - 1
        command.components = '{0,1,2,3,4,5,6,7}'
        command.reason = 'DISTRIBUTED_ENDPOINT_LEASES'
        pump(0.3)
        assert not local_trajectories, 'cold start published before executor readiness'
        waiting_ready_enabled = True
        ready_heartbeat_fresh = False
        pump(0.3)
        assert not local_trajectories, 'WAITING without fresh heartbeat authorized publication'
        ready_heartbeat_fresh = True
        automatic_running = True
        wait_for('first real local trajectory', lambda: latest(local_trajectories) is not None)
        wait_for('matching RUNNING feedback acknowledged',
                 lambda: metadata().get('execution_mode') == '1' and
                 metadata().get('execution_replan') == '0')
        extra_execution = lambda: packet(mode=TrajectoryExecution.HOLDING)
        pump(0.3)
        assert metadata().get('execution_mode') == '1', (
            'mismatched own trajectory identity replaced valid RUNNING feedback', metadata())
        extra_execution = None
        def replayed_sequence():
            feedback = packet(latest(local_trajectories), mode=TrajectoryExecution.HOLDING)
            feedback.sequence = 1
            return feedback
        extra_execution = replayed_sequence
        pump(0.25)
        assert metadata().get('execution_mode') == '1', (
            'reordered execution sequence replaced newer RUNNING feedback', metadata())
        def wrong_duration():
            feedback = packet(latest(local_trajectories), mode=TrajectoryExecution.HOLDING)
            feedback.trajectory_duration += 1.0
            return feedback
        extra_execution = wrong_duration
        pump(0.25)
        assert metadata().get('execution_mode') == '1', (
            'mismatched trajectory duration replaced valid RUNNING feedback', metadata())
        extra_execution = None
        held = copy.deepcopy(latest(local_trajectories))
        position[:] = [-5.38, -1.4, 1.5]
        velocity[:] = [0.05, 0.02, 0.0]
        automatic_running = False
        hold_sent = rospy.Time.now()
        old_id = held.traj_id
        def holding_packet():
            feedback = packet(held, mode=TrajectoryExecution.HOLDING)
            # Commanded hold and actual odometry may differ slightly. The new
            # trajectory must use the latter, not this old command anchor.
            feedback.position = list(held.start_p)
            return feedback
        extra_execution = holding_packet
        wait_for('new local trajectory after executor hold',
                 lambda: latest(local_trajectories).traj_id > old_id, timeout=12.0)
        replacement = latest(local_trajectories)
        extra_execution = None
        automatic_running = True
        assert replacement.start_time >= hold_sent, 'replacement predates hold feedback'
        wait_for('matching command polynomial', lambda: any(
            message.traj_id == replacement.traj_id for _, message in local_polynomials))
        with lock:
            command_polynomial = next(message for _, message in reversed(local_polynomials)
                                      if message.traj_id == replacement.traj_id)
        assert command_polynomial.start_time == replacement.start_time, (
            'executor command and broadcast disagree on absolute trajectory time')
        assert abs(sum(command_polynomial.duration) - sum(replacement.duration)) < 1e-4
        assert math.sqrt(sum((replacement.start_p[i] - position[i]) ** 2
                             for i in range(3))) < 0.05, (
            'replacement used old polynomial state, not fresh odometry',
            list(replacement.start_p), position)
        assert math.sqrt(sum((replacement.start_v[i] - velocity[i]) ** 2
                             for i in range(3))) < 0.05, (
            'replacement discarded fresh odometry velocity',
            list(replacement.start_v), velocity)
        wait_for('replacement acknowledged as RUNNING',
                 lambda: metadata().get('execution_mode') == '1' and
                 metadata().get('execution_replan') == '0')
        # A newer packet sequence is not permission to resurrect an older
        # trajectory identity after this replacement has been acknowledged.
        extra_execution = holding_packet
        pump(0.3)
        assert metadata().get('execution_mode') == '1', (
            'old-trajectory HOLD superseded acknowledged replacement', metadata())
        extra_execution = None
        print('PASS: real planner replaced held trajectory from fresh moved odometry '
              'and velocity; replacement start time follows hold', flush=True)

        # A restarted executor has no old trajectory to resume. A higher
        # session WAITING+fresh-heartbeat handshake authorizes a NEW plan from
        # current odometry, and old-session readiness cannot supersede it.
        before_restart = copy.deepcopy(latest(local_trajectories))
        previous_session = session
        session += 1000000
        sequences[0] = 0
        position[0] += 0.08
        position[1] += 0.04
        automatic_running = False
        force_waiting_ready = True
        def old_session_ready():
            feedback = waiting_packet()
            feedback.session = previous_session
            feedback.sequence += 1000000
            return feedback
        extra_execution = old_session_ready
        restarted_at = rospy.Time.now()
        wait_for('fresh plan after executor restart readiness',
                 lambda: latest(local_trajectories).start_time >= restarted_at and
                 (latest(local_trajectories).traj_id != before_restart.traj_id or
                  latest(local_trajectories).start_time != before_restart.start_time))
        restarted = latest(local_trajectories)
        assert restarted.start_time >= restarted_at
        assert math.sqrt(sum((restarted.start_p[i] - position[i]) ** 2
                             for i in range(3))) < 0.05, (
            'executor restart reused the old planned start instead of fresh odometry')
        extra_execution = None
        force_waiting_ready = False
        automatic_running = True
        wait_for('new executor session RUNNING acknowledgement',
                 lambda: metadata().get('execution_session') == str(session) and
                 metadata().get('execution_mode') == '1' and
                 metadata().get('execution_replan') == '0')
        print('PASS: cold-start/restarted executor requires fresh-heartbeat WAITING '
              'readiness; old-session readiness cannot impersonate the new executor', flush=True)

        # The real MINCO input is independently received, then applied before
        # matching execution. A short cross-topic handoff is allowed; a
        # persistent RUNNING identity mismatch must fail closed instead.
        peer_poly_pub = rospy.Publisher('/swarm/trajectories', MINCOTraj, queue_size=10)
        pump(0.2)
        peer_poly = MINCOTraj()
        peer_poly.drone_id, peer_poly.traj_id, peer_poly.order = 1, 501, 5
        peer_poly.start_time = rospy.Time.now()
        peer_poly.duration = [3.0]
        peer_poly.start_p = peer_poly.end_p = [-5.5, 4.0, 1.5]
        def peer_running():
            feedback = packet(peer_poly, drone=1)
            feedback.velocity = [0.0, 0.0, 0.0]
            return feedback
        holds_before = len(hold_requests)
        foreign_execution_factory = peer_running
        pump(0.12) # Deliberately deliver the execution status before MINCO.
        for _ in range(3):
            peer_poly_pub.publish(peer_poly)
            pump(0.08)
        pump(0.25)
        assert len(hold_requests) == holds_before, 'normal peer MINCO handoff caused a spurious hold'
        peer_poly.traj_id = 502 # Never broadcast this identity.
        wait_for('persistent peer RUNNING/MINCO mismatch requests hold',
                 lambda: len(hold_requests) > holds_before, timeout=3.0)
        foreign_execution_factory = None
        pump(0.2)
        print('PASS: independent peer MINCO handoff tolerates short delivery skew; '
              'persistent execution/trajectory mismatch explicitly holds', flush=True)

        holds_before = len(hold_requests)
        foreign_execution_enabled = False
        stopped_peer = time.monotonic()
        wait_for('explicit hold request after known peer status timeout',
                 lambda: len(hold_requests) > holds_before, timeout=3.0)
        peer_hold_trajectory = copy.deepcopy(latest(local_trajectories))
        automatic_running = False
        extra_execution = lambda: packet(peer_hold_trajectory, mode=TrajectoryExecution.HOLDING)
        pump(0.3)
        with lock:
            late_publications = [stamp for stamp, trajectory in local_trajectories
                                 if stamp > stopped_peer + 0.65 and has_motion(trajectory)]
        assert not late_publications, 'stale peer status allowed new moving trajectory publication'
        foreign_execution_enabled = True
        wait_for('replanning after peer feedback recovery',
                 lambda: latest(local_trajectories).traj_id > peer_hold_trajectory.traj_id)
        extra_execution = None
        automatic_running = True
        wait_for('recovered trajectory RUNNING acknowledgement',
                 lambda: metadata().get('execution_mode') == '1' and
                 metadata().get('execution_replan') == '0')
        print('PASS: known-peer status expiry requests HOLD and blocks motion '
              'until peer feedback recovers', flush=True)

        # Keep synthetic executor RUNNING reports and normal planner commands
        # alive but stop actual odometry. Fresh feedback is not a substitute
        # for fresh state: the planner must explicitly ask its server to hold.
        holds_before = len(hold_requests)
        odometry_enabled = False
        stopped_odom = time.monotonic()
        wait_for('explicit hold request after odometry timeout',
                 lambda: len(hold_requests) > holds_before, timeout=3.0)
        pump(0.5)
        with lock:
            late_publications = [stamp for stamp, trajectory in local_trajectories
                                 if stamp > stopped_odom + 0.4 and has_motion(trajectory)]
        assert not late_publications, 'stale odometry still allowed moving trajectory publication'
        print('PASS: stale odometry requests explicit executor HOLD and blocks '
              'new moving trajectories despite fresh executor feedback', flush=True)

        # A user's mandatory stop outranks recovery/readiness. Even a brand
        # new executor may only receive a stationary stop, never the old goal.
        odometry_enabled = True
        velocity[:] = [0.0, 0.0, 0.0]
        automatic_running = False
        last_running = copy.deepcopy(latest(local_trajectories))
        extra_execution = lambda: packet(last_running, mode=TrajectoryExecution.HOLDING)
        holds_before = len(hold_requests)
        mandatory_at = time.monotonic()
        stop_pub.publish(Empty())
        wait_for('mandatory stop state and immediate executor hold',
                 lambda: metadata().get('state') == 'EMERGENCY_STOP' and
                 len(hold_requests) > holds_before)
        session += 1000000
        sequences[0] = 0
        force_waiting_ready = True
        extra_execution = None
        mandatory_restart_at = rospy.Time.now()
        wait_for('stationary stop restored to restarted executor',
                 lambda: latest(local_trajectories).start_time >= mandatory_restart_at)
        stopped = latest(local_trajectories)
        assert not has_motion(stopped), 'executor restart resumed navigation after mandatory stop'
        force_waiting_ready = False
        automatic_running = True
        pump(0.5)
        assert metadata().get('state') == 'EMERGENCY_STOP', metadata()
        with lock:
            illegal_motion = [trajectory for stamp, trajectory in local_trajectories
                              if stamp > mandatory_at + 0.1 and has_motion(trajectory)]
        assert not illegal_motion, 'mandatory stop was overwritten by a moving recovery/navigation plan'
        print('PASS: mandatory stop survives fresh executor restart/readiness; '
              'only a stationary stop is republished and navigation stays disabled', flush=True)
    except Exception:
        log.seek(0)
        print(log.read()[-14000:], flush=True)
        raise
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == '__main__':
    main()
