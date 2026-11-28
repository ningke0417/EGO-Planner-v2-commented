#!/usr/bin/env python3
"""Check a headless scene; --rows/--columns select the reusable rectangle entry."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scene', choices=['wall', 'random', 'cluttered'])
    parser.add_argument('--port', type=int, default=11327)
    parser.add_argument('--small-position-seed', type=int, default=-1)
    parser.add_argument('--rows', type=int)
    parser.add_argument('--columns', type=int)
    parser.add_argument('--rviz', action='store_true', help='also open the generated RViz view')
    parser.add_argument('--planning-seconds', type=float, default=20.0)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if (args.rows is None) != (args.columns is None):
        parser.error('--rows and --columns must be supplied together')
    rows, columns = (args.rows, args.columns) if args.rows is not None else (2, 4)
    if min(rows, columns) < 1 or not 2 <= rows * columns <= 16:
        parser.error('grid must have between 2 and 16 members')
    count = rows * columns
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % args.port
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    os.environ['ROS_IP'] = '127.0.0.1'
    log_dir = args.report.parent / (args.report.stem + '-ros')
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(log_dir)
    os.environ['ROS_HOME'] = str(log_dir / 'home')
    import rospy
    import rosgraph
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from traj_utils.msg import PolyTraj
    from visualization_msgs.msg import Marker, MarkerArray
    filename = ('rectangle_8uav_fixed_split_merge_wall.launch' if args.scene == 'wall'
                else 'rectangle_8uav_sparse_random.launch')
    cmd = ['roslaunch', '-p', str(args.port), 'ego_planner', filename,
           'rviz:=' + ('true' if args.rviz else 'false'), 'start_planning:=false']
    if args.rows is not None or args.scene == 'cluttered':
        cmd[4] = 'rectangle_formation.launch'
        cmd.extend(['scene:=' + {'wall': 'fixed_wall', 'random': 'sparse_random',
                                'cluttered': 'cluttered_random'}[args.scene],
                    'formation_rows:=%d' % rows, 'formation_columns:=%d' % columns])
    if args.scene != 'wall':
        cmd.append('small_position_seed:=%d' % args.small_position_seed)
    log = (log_dir / 'console.log').open('w')
    process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        master = rosgraph.Master('/baseline_smoke_probe')
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('roslaunch exited; see ' + str(log_dir / 'console.log'))
            try:
                master.getPid()
                break
            except Exception:
                time.sleep(0.2)
        else:
            raise RuntimeError('Isolated ROS master did not start')
        rospy.init_node('baseline_smoke_probe', disable_signals=True)
        state = {'odom': {}, 'scans': {}, 'robots': {}, 'trajectories': [0] * count,
                 'invalid_trajectories': [0] * count, 'cloud': None, 'geometry': None}
        def odom_cb(msg, i):
            p = msg.pose.pose.position
            assert all(math.isfinite(v) for v in [p.x, p.y, p.z])
            state['odom'][i] = [p.x, p.y, p.z]
        def scan_cb(msg, i):
            state['scans'][i] = msg.width * msg.height
        def traj_cb(msg, i):
            values = list(msg.duration) + list(msg.coef_x) + list(msg.coef_y) + list(msg.coef_z)
            if not all(math.isfinite(v) for v in values) or not all(v > 0.0 for v in msg.duration):
                state['invalid_trajectories'][i] += 1
            state['trajectories'][i] += 1
        subscriptions = []
        for i in range(count):
            subscriptions.append(rospy.Subscriber('/drone_%d_visual_slam/odom' % i, Odometry, odom_cb, callback_args=i))
            subscriptions.append(rospy.Subscriber('/drone_%d_pcl_render_node/cloud' % i, PointCloud2, scan_cb, callback_args=i))
            subscriptions.append(rospy.Subscriber('/drone_%d_planning/trajectory' % i, PolyTraj, traj_cb, callback_args=i))
            subscriptions.append(rospy.Subscriber('/drone_%d_odom_visualization/robot' % i, Marker,
                lambda msg, drone: state['robots'].update({drone: msg}), callback_args=i))
        subscriptions.append(rospy.Subscriber('/map_generator/global_cloud', PointCloud2,
            lambda msg: state.update(cloud=msg)))
        topic = '/map_generator/wall_geometry' if args.scene == 'wall' else '/map_generator/sparse_random_geometry'
        subscriptions.append(rospy.Subscriber(topic, MarkerArray, lambda msg: state.update(geometry=msg)))
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            if (len(state['odom']) == count and len(state['scans']) == count
                    and len(state['robots']) == count and state['cloud'] and state['geometry']):
                break
            if process.poll() is not None:
                raise RuntimeError('Scene stopped before initialization')
            time.sleep(0.1)
        assert len(state['odom']) == count, ('Missing odometry', sorted(state['odom']))
        assert len(state['scans']) == count, ('Missing local scans', sorted(state['scans']))
        assert len(state['robots']) == count, ('Missing robot markers', sorted(state['robots']))
        assert state['cloud'] and state['cloud'].width > 0, 'Missing physical map'
        assert state['geometry'], 'Missing physical geometry markers'
        start_x = {'wall': -6.0, 'random': -14.0, 'cluttered': -17.0}[args.scene]
        initial = dict(state['odom'])
        for i in range(count):
            expected = [start_x + (rows - 1) / 2.0 - i // columns,
                        i % columns - (columns - 1) / 2.0, 1.5]
            assert max(abs(a-b) for a,b in zip(initial[i], expected)) < 1e-8, (i, initial[i], expected)
            marker = state['robots'][i]
            assert marker.type == Marker.MESH_RESOURCE and marker.mesh_resource == 'package://odom_visualization/meshes/fake_drone.dae'
            assert [marker.scale.x, marker.scale.y, marker.scale.z] == [0.35] * 3
        expected_boxes = {'wall': 3, 'random': 15, 'cluttered': 60}[args.scene]
        assert len(state['geometry'].markers) == expected_boxes
        trigger = rospy.Publisher('/traj_start_trigger', PoseStamped, queue_size=1, latch=True)
        trigger.publish(PoseStamped())
        deadline = time.monotonic() + args.planning_seconds
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('roslaunch exited during planning')
            time.sleep(0.2)
        assert all(n > 0 for n in state['trajectories']), ('Some planners published no trajectory', state['trajectories'])
        assert not any(state['invalid_trajectories']), ('Non-finite trajectory', state['invalid_trajectories'])
        moved = [math.sqrt(sum((a-b)**2 for a,b in zip(state['odom'][i], initial[i]))) for i in range(count)]
        assert min(moved) > 0.05, ('Some vehicles did not execute trajectories', moved)
        # Check graph health, rather than treating initialization as a successful flight.
        import rosnode
        nodes = sorted(rosnode.get_node_names())
        assert all('/drone_%d_ego_planner_node' % i in nodes and '/drone_%d_traj_server' % i in nodes for i in range(count))
        if args.rviz:
            assert '/rviz' in nodes, 'RViz did not start'
        report = {'scene': args.scene, 'small_position_seed': args.small_position_seed,
                  'formation_rows': rows, 'formation_columns': columns, 'num_uavs': count,
                  'rviz': args.rviz,
                  'planning_seconds': args.planning_seconds, 'initial_positions': initial,
                  'map_points': state['cloud'].width * state['cloud'].height,
                  'map_data_sha256': hashlib.sha256(state['cloud'].data).hexdigest(),
                  'physical_boxes': expected_boxes, 'local_scans_received': count,
                  'robot_markers_received': count, 'mesh_resource': marker.mesh_resource,
                  'robot_scale': marker.scale.x,
                  'trajectory_counts': state['trajectories'],
                  'nonfinite_trajectory_counts': state['invalid_trajectories'],
                  'displacements': moved, 'nodes': nodes,
                  'scope': 'Startup and short planning/execution smoke check; not a mission success or safety result.'}
        args.report.write_text(json.dumps(report, indent=2) + '\n')
        print('PASS %s: map, %dx%d formation, %d scans/models and executing official planners' % (args.scene, rows, columns, count))
        print('Report: ' + str(args.report))
    finally:
        rospy.signal_shutdown('smoke check finished')
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        log.close()


if __name__ == '__main__':
    main()
