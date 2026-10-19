#!/usr/bin/env python3
"""Exercise a real planner's full-assignment input on an empty test master.

No simulator, flight trigger or formation agent is launched. Synthetic odometry
and startup-wait commands keep the flight barrier closed while exercising real
waypoint refresh and optimizer permutation application. Use a disposable master
at http://127.0.0.1:11337 and a sourced, freshly built formation_ws.
"""
import os
import signal
import subprocess
import tempfile
import time

import roslaunch.config
import rosnode
import rospkg
import rospy
from ego_planner.msg import FormationTopologyAck, FormationTopologyCommand
from nav_msgs.msg import Odometry
from traj_utils.msg import MINCOTraj


def digest(assignment):
    rank = 0
    for i, slot in enumerate(assignment):
        rank = rank * (8 - i) + sum(other < slot for other in assignment[i + 1:])
    return rank + 1


def main():
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11337', (
        'Use the dedicated disposable test master on port 11337')
    rospy.init_node('test_distributed_assignment_planner', anonymous=True)
    assert not any(name.startswith('/drone_') for name in rosnode.get_node_names()), (
        'The dedicated test master must have no experiment nodes')

    launch_file = os.path.join(rospkg.RosPack().get_path('ego_planner'), 'launch',
                               'rectangle_8uav_member.launch')
    arguments = ['distributed_enabled:=true', 'swap_enabled:=false',
                 'drone_id:=0', 'init_x:=-5.5', 'init_y:=-1.5',
                 'map_size_x:=24', 'map_size_y:=14', 'map_size_z:=3',
                 'start_x:=-6', 'goal_x:=8']
    config = roslaunch.config.load_config_default(
        [(launch_file, arguments)], None, assign_machines=False)
    planner = next(node for node in config.nodes
                   if node.name == 'drone_0_ego_planner_node')
    for parameter in config.params.values():
        rospy.set_param(parameter.key, parameter.value)
    # Allow command-rejection checks without a watchdog event confusing the
    # accepted assignment. The production watchdog remains enabled.
    rospy.set_param('/drone_0_ego_planner_node/formation/distributed_startup_timeout', 60.0)

    applied = []
    trajectories = []
    ack_sub = rospy.Subscriber('/drone_0/formation_applied', FormationTopologyAck,
                               applied.append, queue_size=100)
    traj_sub = rospy.Subscriber('/swarm/trajectories', MINCOTraj,
                                trajectories.append, queue_size=10)
    command_pub = rospy.Publisher('/drone_0_formation_agent/topology',
                                  FormationTopologyCommand, queue_size=1)
    odom_pub = rospy.Publisher('/drone_0_visual_slam/odom', Odometry, queue_size=1)
    command = FormationTopologyCommand()
    command.mode = FormationTopologyCommand.SPLIT
    command.edge_count = 10
    command.components = '|'.join('{%d}' % i for i in range(8))
    command.formation_scale = 0.15
    command.reason = 'DISTRIBUTED_STARTUP_WAIT'

    def set_assignment(epoch, assignment):
        command.assignment_epoch = epoch
        command.assignment_digest = digest(assignment)
        command.drone_to_slot = assignment

    def pump(seconds=0.3):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, 'planner process exited'
            now = rospy.Time.now()
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'world'
            odom.pose.pose.position.x = -5.5
            odom.pose.pose.position.y = -1.5
            odom.pose.pose.position.z = 1.5
            odom.pose.pose.orientation.w = 1
            odom.twist.twist.linear.x = 0.2
            odom_pub.publish(odom)
            command.header.stamp = now
            command_pub.publish(command)
            time.sleep(0.03)

    def await_assignment(epoch, assignment, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pump(0.15)
            if (applied and applied[-1].goal_valid and
                    applied[-1].assignment_epoch == epoch and
                    list(applied[-1].drone_to_slot) == assignment):
                assert applied[-1].assignment_digest == digest(assignment)
                return applied[-1]
        raise AssertionError('planner did not apply epoch %d map %s' % (epoch, assignment))

    def reject(epoch, assignment, accepted_epoch, accepted_assignment, bad_digest=None):
        set_assignment(epoch, assignment)
        if bad_digest is not None:
            command.assignment_digest = bad_digest
        first = len(applied)
        pump(0.3)
        for ack in applied[first:]:
            assert ack.assignment_epoch == accepted_epoch, 'rejected command advanced epoch'
            assert list(ack.drone_to_slot) == accepted_assignment, 'rejected command changed map'
        set_assignment(accepted_epoch, accepted_assignment)
        await_assignment(accepted_epoch, accepted_assignment)

    log = tempfile.TemporaryFile(mode='w+')
    argv = ['rosrun', planner.package, planner.type,
            '__name:=' + planner.name]
    argv.extend('%s:=%s' % (source, target)
                for source, target in planner.remap_args)
    process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT)
    try:
        identity = list(range(8))
        set_assignment(1, identity)
        ack = await_assignment(1, identity)
        assert all(abs(a - b) < 1e-6 for a, b in
                   zip(ack.assigned_goal, [8.5, -1.5, 1.5])), ack.assigned_goal
        # Drone 0 crosses columns; its physical ID remains unchanged, but its
        # own assigned goal moves to the opposite side of the rectangle.
        cross_column = [3, 1, 2, 0, 7, 5, 6, 4]
        set_assignment(2, cross_column)
        ack = await_assignment(2, cross_column)
        assert all(abs(a - b) < 1e-6 for a, b in
                   zip(ack.assigned_goal, [8.5, 1.5, 1.5])), ack.assigned_goal
        # Reassign only other aircraft: optimizer still receives the complete
        # map; drone 0's waypoint must remain its current slot's waypoint.
        other_columns = [3, 2, 1, 0, 7, 6, 5, 4]
        set_assignment(3, other_columns)
        ack = await_assignment(3, other_columns)
        assert abs(ack.assigned_goal[1] - 1.5) < 1e-6
        reject(2, cross_column, 3, other_columns)
        reject(3, identity, 3, other_columns)
        reject(4, identity, 3, other_columns, bad_digest=999)
        reject(4, [0] * 8, 3, other_columns, bad_digest=0)
        reject(0, identity, 3, other_columns, bad_digest=0)
        assert not trajectories, 'startup-wait test must never enable flight'
        print('PASS: real planner accepts full permutations, refreshes own goal, '
              'retains unchanged own goal, rejects rollback/conflict/digest/duplicate slots; '
              'startup barrier remained closed', flush=True)
    except Exception:
        log.seek(0)
        print(log.read()[-10000:], flush=True)
        raise
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        ack_sub.unregister()
        traj_sub.unregister()


if __name__ == '__main__':
    main()
