#!/usr/bin/env python3
"""Check real planner control-map freshness on an isolated disposable master.

Run after sourcing Noetic and the freshly built formation_ws. This script owns
its master and one planner, publishes only synthetic odometry/lidar, and leaves
the flight trigger and distributed startup barrier closed. Logs stay in /tmp.
"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time


def choose_test_port(requested):
    if requested and not 20000 <= requested <= 65535:
        raise ValueError('use port 0 or a dedicated port in 20000..65535')
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', requested))
        return probe.getsockname()[1]


def descendant(pid, ancestor):
    """Verify the responding master belongs to the roscore just started here."""
    for _ in range(30):
        if pid == ancestor:
            return True
        try:
            status = Path('/proc/%d/status' % pid).read_text()
            pid = int(next(line.split()[1] for line in status.splitlines()
                           if line.startswith('PPid:')))
        except (OSError, StopIteration, ValueError):
            return False
        if pid <= 1:
            return False
    return False


def stop_owned(process):
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 3),
                         (signal.SIGKILL, 3)):
        if process.poll() is not None:
            return
        try:
            # Each process below starts a fresh session; never signal the
            # caller's process group or kill nodes through an existing master.
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=0)
    args = parser.parse_args()
    port = choose_test_port(args.port)
    logs = Path(tempfile.mkdtemp(prefix='ego-empty-cloud-', dir='/tmp'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % port,
                      ROS_IP='127.0.0.1', ROS_HOME=str(logs / 'ros_home'),
                      ROS_LOG_DIR=str(logs / 'ros_logs'))
    os.environ.pop('ROS_HOSTNAME', None)

    # Import ROS only after selecting the dedicated environment.
    import rosgraph
    import roslaunch.config
    import rospkg
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs.point_cloud2 as clouds
    from std_msgs.msg import Header

    core = planner_process = None
    subscribers = []
    log_handles = []
    try:
        core_log = (logs / 'master.log').open('w')
        log_handles.append(core_log)
        core = subprocess.Popen(['roscore', '-p', str(port)], stdout=core_log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        master = rosgraph.Master('/empty_cloud_test')
        deadline = time.monotonic() + 15
        while True:
            assert core.poll() is None, 'test master exited; see %s' % logs
            try:
                master_pid = master.getPid()
                assert descendant(master_pid, core.pid), 'refuse an unowned ROS master'
                break
            except (OSError, rosgraph.MasterError, rosgraph.MasterException):
                if time.monotonic() >= deadline:
                    raise AssertionError('test master did not start; see %s' % logs)
                time.sleep(0.05)

        rospy.init_node('test_distributed_empty_cloud', anonymous=True,
                        disable_signals=True)
        package = Path(__file__).resolve().parents[1]
        assert Path(rospkg.RosPack().get_path('ego_planner')).resolve() == package, (
            'source this checkout\'s freshly built formation_ws')
        member = package / 'launch/rectangle_8uav_member.launch'
        config = roslaunch.config.load_config_default(
            [(str(member), ['distributed_enabled:=true', 'swap_enabled:=false',
                            'drone_id:=0', 'init_x:=0', 'init_y:=0',
                            'map_size_x:=24', 'map_size_y:=14', 'map_size_z:=3',
                            'start_x:=-6', 'goal_x:=10'])],
            None, assign_machines=False)
        planner = next(node for node in config.nodes
                       if node.name == 'drone_0_ego_planner_node')
        for param in config.params.values():
            rospy.set_param(param.key, param.value)
        prefix = '/drone_0_ego_planner_node'
        # No trigger, formation agent, executor or simulated vehicle is started.
        rospy.set_param(prefix + '/fsm/realworld_experiment', True)
        rospy.set_param(prefix + '/formation/distributed_startup_timeout', 120.0)
        timeout = float(rospy.get_param(prefix + '/grid_map/odom_depth_timeout', 1.0))
        assert 0 < timeout <= 10, 'unexpected sensor timeout in test fixture'

        snapshots = deque(maxlen=200)
        trajectories = []
        lock = threading.Lock()

        def snapshot(msg):
            with lock:
                snapshots.append(msg)

        subscribers.append(rospy.Subscriber(prefix + '/grid_map/formation_occupancy',
                                             PointCloud2, snapshot, queue_size=20))
        subscribers.append(rospy.Subscriber('/drone_0_planning/trajectory',
                                             rospy.AnyMsg, trajectories.append,
                                             queue_size=10))
        odom_pub = rospy.Publisher('/drone_0_visual_slam/odom', Odometry, queue_size=1)
        cloud_pub = rospy.Publisher('/drone_0_pcl_render_node/cloud',
                                     PointCloud2, queue_size=1)
        planner_log = (logs / 'planner.log').open('w')
        log_handles.append(planner_log)
        argv = ['rosrun', planner.package, planner.type, '__name:=' + planner.name]
        argv += ['%s:=%s' % tuple(pair) for pair in planner.remap_args]
        planner_process = subprocess.Popen(argv, stdout=planner_log,
                                          stderr=subprocess.STDOUT,
                                          start_new_session=True)

        def pump(seconds, publish_odom=True):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert planner_process.poll() is None, 'planner exited; see %s' % logs
                if publish_odom:
                    odom = Odometry()
                    odom.header.stamp = rospy.Time.now()
                    odom.header.frame_id = 'world'
                    odom.pose.pose.position.z = 1.5
                    odom.pose.pose.orientation.w = 1.0
                    odom_pub.publish(odom)
                time.sleep(0.025)

        def observations():
            with lock:
                return list(snapshots)

        def wait_stamp(stamp, publish_odom=True):
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                pump(0.05, publish_odom)
                matches = [msg for msg in observations()
                           if msg.header.stamp == stamp]
                if matches:
                    return matches[-1]
            raise AssertionError('no snapshot with sensor stamp %s; see %s' % (stamp, logs))

        def points(msg):
            return frozenset(tuple(round(float(value), 5) for value in xyz)
                             for xyz in clouds.read_points(
                                 msg, field_names=('x', 'y', 'z'), skip_nans=True))

        def empty(stamp, frame='world'):
            return clouds.create_cloud_xyz32(Header(stamp=stamp, frame_id=frame), [])

        deadline = time.monotonic() + 10
        while not (odom_pub.get_num_connections() and cloud_pub.get_num_connections()):
            assert time.monotonic() < deadline, 'planner did not subscribe; see %s' % logs
            pump(0.1)
        pump(0.2)
        occupied_stamp = rospy.Time.now()
        cloud_pub.publish(clouds.create_cloud_xyz32(
            Header(stamp=occupied_stamp, frame_id='world'), [(1.0, 0.0, 1.5)]))
        occupied_map = wait_stamp(occupied_stamp)
        occupied_points = points(occupied_map)
        assert any(abs(x - 1.0) < 0.3 and abs(y) < 0.3 and abs(z - 1.5) < 0.2
                   for x, y, z in occupied_points), 'fixture obstacle was not represented'

        accepted_stamp = rospy.Time.now()
        cloud_pub.publish(empty(accepted_stamp))
        refreshed = wait_stamp(accepted_stamp)
        assert accepted_stamp > occupied_stamp
        assert points(refreshed) == occupied_points, 'empty lidar cleared prior occupancy'
        print('PASS fresh empty cloud refreshes sensor stamp and preserves occupancy', flush=True)

        def reject(label, message, publish_odom=True):
            before = len(observations())
            cloud_pub.publish(message)
            pump(0.5, publish_odom)
            messages = observations()
            assert len(messages) > before, 'control snapshot timer stopped'
            assert all(msg.header.stamp == accepted_stamp for msg in messages[before:]), (
                label + ' incorrectly refreshed control-map freshness')
            assert points(messages[-1]) == occupied_points, label + ' altered occupancy'
            print('PASS rejects ' + label, flush=True)

        reject('repeated empty timestamp', empty(accepted_stamp))
        reject('older empty timestamp', empty(occupied_stamp))
        reject('zero empty timestamp', empty(rospy.Time(0)))
        reject('future empty timestamp', empty(rospy.Time.now() + rospy.Duration(5)))
        reject('wrong empty-cloud frame', empty(rospy.Time.now(), 'wrong_frame'))
        pump(timeout + 0.2, publish_odom=False)
        reject('fresh empty cloud with stale odometry', empty(rospy.Time.now()),
               publish_odom=False)
        pump(0.15)
        accepted_stamp = rospy.Time.now()
        cloud_pub.publish(empty(accepted_stamp))
        recovered = wait_stamp(accepted_stamp)
        assert points(recovered) == occupied_points
        assert not trajectories, 'fixture unexpectedly enabled flight'
        print(json.dumps({'result': 'PASS', 'master_port': port,
                          'occupied_voxels': len(occupied_points),
                          'fresh_odom_recovery': True, 'flight_triggered': False,
                          'logs': str(logs)}), flush=True)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        stop_owned(planner_process)
        if 'rospy' in locals() and rospy.core.is_initialized():
            rospy.signal_shutdown('isolated empty-cloud regression finished')
        stop_owned(core)
        for handle in log_handles:
            handle.close()
        print('Test logs: %s' % logs, flush=True)


if __name__ == '__main__':
    main()
