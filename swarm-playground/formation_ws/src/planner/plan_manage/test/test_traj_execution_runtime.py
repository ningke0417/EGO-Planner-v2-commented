#!/usr/bin/env python3
"""Real trajectory-server execution contract on disposable localhost:11341.

Starts only its own traj_server process. Synthetic heartbeats and trajectories
verify absolute-time preservation, latched holds, rejection of invalid/replayed
commands, fresh replacement, and indefinitely stationary completed endpoints.
No planner, flight simulator, or pre-existing experiment should share this master.
"""

import copy
import json
import math
import os
import subprocess
import threading
import time

import rosnode
import rospy
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty
from traj_utils.msg import PolyTraj, TrajectoryExecution


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.states = []
        self.commands = []
        self.heartbeat_enabled = False
        prefix = '/trajectory_execution_test/server'
        self.heartbeat = rospy.Publisher(prefix + '/heartbeat', Empty, queue_size=1)
        self.hold = rospy.Publisher(prefix + '/hold', Empty, queue_size=1)
        self.trajectory = rospy.Publisher(prefix + '/planning/trajectory',
                                          PolyTraj, queue_size=10)
        self.state_sub = rospy.Subscriber(prefix + '/execution_state',
                                          TrajectoryExecution, self.on_state,
                                          queue_size=1000)
        self.command_sub = rospy.Subscriber('/trajectory_execution_test/command',
                                            PositionCommand, self.on_command,
                                            queue_size=1000)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.on_heartbeat)

    def on_heartbeat(self, _):
        if self.heartbeat_enabled:
            self.heartbeat.publish(Empty())

    def on_state(self, msg):
        with self.lock:
            self.states.append(msg)

    def on_command(self, msg):
        with self.lock:
            self.commands.append(msg)

    def snapshot(self):
        with self.lock:
            return list(self.states), list(self.commands)

    def latest(self):
        states, _ = self.snapshot()
        return states[-1] if states else None

    def wait(self, predicate, description, timeout=3.0):
        until = time.monotonic() + timeout
        while time.monotonic() < until and not rospy.is_shutdown():
            state = self.latest()
            if state is not None and predicate(state):
                return state
            time.sleep(0.01)
        raise AssertionError('%s; latest=%s' % (description, self.latest()))


def linear_trajectory(identity, x, duration=2.0, speed=0.2):
    value = PolyTraj()
    value.drone_id = 0
    value.traj_id = identity
    value.order = 5
    value.start_time = rospy.Time.now()
    value.duration = [duration]
    value.coef_x = [0, 0, 0, 0, speed, x]
    value.coef_y = [0, 0, 0, 0, 0, 0]
    value.coef_z = [0, 0, 0, 0, 0, 1.5]
    return value


def require_held(probe, trajectory_id, position, seconds=0.2):
    start = rospy.Time.now()
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        time.sleep(0.01)
    states, commands = probe.snapshot()
    states = [state for state in states if state.header.stamp >= start]
    commands = [command for command in commands if command.header.stamp >= start]
    assert states and commands, 'Hold stopped publishing execution/command feedback'
    for state in states:
        assert state.trajectory_id == trajectory_id
        assert state.mode == TrajectoryExecution.HOLDING and state.require_replan
        assert max(abs(a - b) for a, b in zip(state.position, position)) < 1e-6
        assert max(abs(v) for v in state.velocity) < 1e-9
    for command in commands:
        actual = (command.position.x, command.position.y, command.position.z)
        assert max(abs(a - b) for a, b in zip(actual, position)) < 1e-6
        assert max(abs(command.velocity.x), abs(command.velocity.y),
                   abs(command.velocity.z)) < 1e-9


def main():
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11341', (
        'Use a fresh isolated ROS master on port 11341')
    rospy.init_node('test_traj_execution_runtime', anonymous=True)
    assert not any(name.startswith('/drone_') or
                   name.startswith('/trajectory_execution_test/')
                   for name in rosnode.get_node_names()), 'Test master is not empty'
    probe = Probe()
    server_command = [
        'rosrun', 'ego_planner', 'traj_server',
        '__ns:=/trajectory_execution_test', '__name:=server',
        '_drone_id:=0', '_traj_server/time_forward:=0.2',
        '/position_cmd:=/trajectory_execution_test/command']
    process = subprocess.Popen(server_command)
    try:
        until = time.monotonic() + 8.0
        while time.monotonic() < until:
            assert process.poll() is None, 'traj_server exited during startup'
            if (probe.trajectory.get_num_connections() and
                    probe.heartbeat.get_num_connections()):
                break
            time.sleep(0.05)
        else:
            raise AssertionError('traj_server subscriptions did not connect')
        time.sleep(1.15)  # The server initializes before entering ros::spin().
        waiting = probe.wait(lambda s: s.mode == TrajectoryExecution.WAITING and
                             not s.heartbeat_fresh, 'Cold server must report nonspatial WAITING')
        assert waiting.trajectory_id == 0 and waiting.trajectory_start_time.is_zero()
        assert waiting.trajectory_duration == 0 and waiting.elapsed == 0
        assert not waiting.require_replan and not probe.snapshot()[1]
        assert list(waiting.position) == list(waiting.velocity) == list(waiting.acceleration) == [0, 0, 0]
        probe.hold.publish(Empty())  # No commanded position yet: harmless no-op.
        time.sleep(0.05)
        probe.heartbeat_enabled = True
        ready = probe.wait(lambda s: s.mode == TrajectoryExecution.WAITING and
                           s.heartbeat_fresh, 'Fresh heartbeat must advertise readiness')
        assert ready.session == waiting.session and not probe.snapshot()[1]

        original = linear_trajectory(1, 0.0, duration=3.0)
        probe.trajectory.publish(original)
        state = probe.wait(lambda s: s.trajectory_id == 1 and
                           s.mode == TrajectoryExecution.RUNNING,
                           'Initial trajectory must execute')
        assert state.trajectory_start_time == original.start_time
        time.sleep(0.3)
        running, _ = probe.snapshot()
        samples = [s for s in running if s.mode == TrajectoryExecution.RUNNING]
        assert len(samples) >= 4, 'Execution feedback is not periodically published'
        for sample in samples:
            assert abs(sample.elapsed - (sample.header.stamp -
                       sample.trajectory_start_time).to_sec()) < 0.03

        probe.heartbeat_enabled = False
        held = probe.wait(lambda s: s.mode == TrajectoryExecution.HOLDING and
                          s.require_replan, 'Missing heartbeat must latch HOLD')
        held_position = tuple(held.position)
        assert held.trajectory_start_time == original.start_time
        # A fresh-looking candidate received without communication must not
        # re-arm the server later when heartbeat alone returns.
        probe.trajectory.publish(linear_trajectory(2, held_position[0]))
        require_held(probe, 1, held_position)
        probe.heartbeat_enabled = True
        require_held(probe, 1, held_position, seconds=0.3)

        invalid = []
        malformed = linear_trajectory(2, held_position[0]); malformed.coef_y = []
        invalid.append(malformed)
        nonfinite = linear_trajectory(2, held_position[0]); nonfinite.coef_z[-1] = math.nan
        invalid.append(nonfinite)
        bad_duration = linear_trajectory(2, held_position[0]); bad_duration.duration = [0.0]
        invalid.append(bad_duration)
        wrong_drone = linear_trajectory(2, held_position[0]); wrong_drone.drone_id = 1
        invalid.append(wrong_drone)
        wrong_order = linear_trajectory(2, held_position[0]); wrong_order.order = 3
        invalid.append(wrong_order)
        discontinuous = linear_trajectory(2, held_position[0] + 1.0)
        invalid.append(discontinuous)
        for message in invalid:
            message.start_time = rospy.Time.now()
            probe.trajectory.publish(message)
            require_held(probe, 1, held_position, seconds=0.08)
        for offset in (-1.0, 0.2):
            wrong_time = linear_trajectory(2, held_position[0])
            wrong_time.start_time = rospy.Time.now() + rospy.Duration(offset)
            probe.trajectory.publish(wrong_time)
            require_held(probe, 1, held_position, seconds=0.08)
        # Identical replay and an older timestamp with a higher ID are both old.
        probe.trajectory.publish(original)
        older = copy.deepcopy(original); older.traj_id = 10000
        probe.trajectory.publish(older)
        require_held(probe, 1, held_position)

        replacement = linear_trajectory(2, held_position[0], duration=0.6)
        probe.trajectory.publish(replacement)
        active = probe.wait(lambda s: s.trajectory_id == 2 and
                            s.mode == TrajectoryExecution.RUNNING,
                            'Fresh post-hold replacement must execute')
        assert not active.require_replan
        assert active.trajectory_start_time == replacement.start_time
        probe.trajectory.publish(replacement)  # Never restart a duplicate.
        finished = probe.wait(lambda s: s.trajectory_id == 2 and
                              s.mode == TrajectoryExecution.FINISHED,
                              'Completed trajectory must retain its endpoint')
        assert not finished.require_replan
        assert finished.trajectory_start_time == replacement.start_time
        assert abs(finished.position[0] - (held_position[0] + 0.12)) < 1e-5
        assert max(abs(v) for v in finished.velocity) < 1e-9
        finish_stamp = finished.header.stamp
        time.sleep(0.3)
        states, commands = probe.snapshot()
        endpoints = [s for s in states if s.header.stamp >= finish_stamp]
        assert len(endpoints) >= 4
        assert all(s.mode == TrajectoryExecution.FINISHED and
                   not s.require_replan and
                   max(abs(a - b) for a, b in zip(s.position, finished.position)) < 1e-6 and
                   max(abs(v) for v in s.velocity) < 1e-9 for s in endpoints)
        # An independent safety hold must work even with healthy heartbeats.
        probe.hold.publish(Empty())
        explicit = probe.wait(lambda s: s.trajectory_id == 2 and
                              s.mode == TrajectoryExecution.HOLDING and
                              s.require_replan, 'Explicit hold must latch')
        require_held(probe, 2, tuple(explicit.position), seconds=0.25)
        final_replacement = linear_trajectory(3, explicit.position[0],
                                              duration=0.4, speed=0.0)
        probe.trajectory.publish(final_replacement)
        probe.wait(lambda s: s.trajectory_id == 3 and
                   s.mode == TrajectoryExecution.RUNNING and not s.require_replan,
                   'Fresh post-explicit-hold replacement must execute')
        probe.wait(lambda s: s.trajectory_id == 3 and
                   s.mode == TrajectoryExecution.FINISHED,
                   'Stationary replacement must finish')
        previous_session = probe.latest().session
        process.terminate()
        process.wait(timeout=5)
        restart_stamp = rospy.Time.now()
        process = subprocess.Popen(server_command)
        restarted = probe.wait(lambda s: s.session > previous_session and
                               s.mode == TrajectoryExecution.WAITING and
                               s.heartbeat_fresh,
                               'Restarted executor must advertise a fresh ready session',
                               timeout=5.0)
        assert restarted.trajectory_id == 0 and restarted.trajectory_start_time.is_zero()
        assert restarted.trajectory_duration == 0 and not restarted.require_replan
        assert list(restarted.position) == [0, 0, 0]  # Not a spatial obstacle.
        # Restart is not permission to replay an old path or publish commands.
        probe.trajectory.publish(original)
        time.sleep(0.15)
        assert probe.latest().mode == TrajectoryExecution.WAITING
        assert not any(command.header.stamp > restart_stamp
                       for command in probe.snapshot()[1])
        restarted_path = linear_trajectory(1, explicit.position[0], duration=0.4, speed=0.0)
        probe.trajectory.publish(restarted_path)
        probe.wait(lambda s: s.session == restarted.session and
                   s.trajectory_id == 1 and s.mode == TrajectoryExecution.RUNNING,
                   'Restarted server needs a newly created trajectory')
        probe.wait(lambda s: s.session == restarted.session and
                   s.mode == TrajectoryExecution.FINISHED,
                   'Restarted server must retain its new stationary endpoint')
        states, commands = probe.snapshot()
        expected_starts = {(waiting.session, 1): original.start_time,
                           (waiting.session, 2): replacement.start_time,
                           (waiting.session, 3): final_replacement.start_time,
                           (restarted.session, 1): restarted_path.start_time}
        spatial = [s for s in states if s.mode != TrajectoryExecution.WAITING]
        assert all((s.session, s.trajectory_id) in expected_starts and
                   s.trajectory_start_time == expected_starts[(s.session, s.trajectory_id)]
                   for s in spatial), 'Server changed an absolute trajectory timestamp'
        assert all(s.session != 0 for s in states)
        for session in {s.session for s in states}:
            ordered = [s for s in states if s.session == session]
            assert all(a.sequence < b.sequence for a, b in zip(ordered, ordered[1:]))
        print(json.dumps({'absolute_execution_time': 'PASS',
                          'heartbeat_hold_never_resumes_old_path': 'PASS',
                          'fresh_replacement_only': 'PASS',
                          'malformed_and_replayed_trajectories': 'PASS',
                          'stationary_finished_endpoint': 'PASS',
                          'explicit_safety_hold': 'PASS',
                          'nonspatial_ready_and_server_restart': 'PASS',
                          'periodic_ordered_feedback': 'PASS',
                          'scope': 'command execution, not actual tracking/braking safety'}),
              flush=True)
    finally:
        probe.timer.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


if __name__ == '__main__':
    main()
