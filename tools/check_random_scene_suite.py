#!/usr/bin/env python3
"""Compare actual map binaries and resolved map parameters across both projects."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = Path('swarm-playground/formation_ws')
MAP_SOURCE = WORKSPACE / 'src/uav_simulator/map_generator/src/sparse_random_obstacle_map.cpp'
SCENE = WORKSPACE / 'src/planner/plan_manage/launch/rectangle_8uav_sparse_random.launch'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def map_parameters(root, count, seed, reference, cache):
    devel = root / WORKSPACE / ('devel' if reference else 'devel/original_baseline')
    env = dict(os.environ, ROS_PACKAGE_PATH=str(root / WORKSPACE / 'src') + ':/opt/ros/noetic/share',
               CMAKE_PREFIX_PATH=str(devel) + ':/opt/ros/noetic', ROS_HOME=str(cache))
    cache.mkdir(parents=True, exist_ok=True)
    code = '''import json,sys,roslaunch.config
c=roslaunch.config.load_config_default([(sys.argv[1],sys.argv[2:])],11561,verbose=False,assign_machines=False)
p='/sparse_random_obstacle_map/'
print(json.dumps({k[len(p):]:v.value for k,v in c.params.items() if k.startswith(p)}))'''
    args = ['rviz:=false', 'random_seed:=17', 'small_obstacle_count:=%d' % count,
            'small_position_seed:=%d' % seed]
    if reference:
        args.append('coordination_mode:=distributed')
    result = subprocess.check_output([sys.executable, '-c', code, str(root / SCENE), *args], env=env, text=True)
    return json.loads(result.strip().splitlines()[-1])


def stop(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5)


def snapshot(binary, params, name, output):
    import rospy
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import MarkerArray
    rospy.set_param('/' + name, params)
    geometry_topic, cloud_topic = '/' + name + '/geometry', '/' + name + '/cloud'
    with (output / (name + '.log')).open('w') as log:
        process = subprocess.Popen([str(binary), '__name:=' + name,
            '/map_generator/sparse_random_geometry:=' + geometry_topic, 'global_cloud:=' + cloud_topic],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            geometry = rospy.wait_for_message(geometry_topic, MarkerArray, timeout=20)
            cloud = rospy.wait_for_message(cloud_topic, PointCloud2, timeout=20)
            boxes = []
            for marker in geometry.markers:
                p, s, q = marker.pose.position, marker.scale, marker.pose.orientation
                assert marker.type == marker.CUBE and marker.header.frame_id == 'world'
                assert q.x == q.y == q.z == 0 and abs(q.w) == 1
                boxes.append([p.x, p.y, p.z, s.x, s.y, s.z])
            packed = b''.join(struct.pack('<6d', *box) for box in boxes)
            return dict(boxes_center_xyz_size_xyz=boxes, obstacle_count=len(boxes),
                        geometry_sha256=hashlib.sha256(packed).hexdigest(),
                        pointcloud_data_sha256=hashlib.sha256(cloud.data).hexdigest(),
                        points=cloud.width * cloud.height, point_step=cloud.point_step,
                        point_fields=[[f.name, f.offset, f.datatype, f.count] for f in cloud.fields])
        finally:
            stop(process)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=ROOT.parent / 'EGO-Planner-v2-commented')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--write-manifest', type=Path)
    parser.add_argument('--port', type=int, default=11561)
    args = parser.parse_args()
    reference, output = args.reference.resolve(), args.output.resolve()
    assert (ROOT / MAP_SOURCE).read_bytes() == (reference / MAP_SOURCE).read_bytes(), 'Map sources differ'
    binaries = [ROOT / WORKSPACE / 'devel/original_baseline/lib/map_generator/sparse_random_obstacle_map',
                reference / WORKSPACE / 'devel/lib/map_generator/sparse_random_obstacle_map']
    output.mkdir(parents=True, exist_ok=False)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', args.port))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port, ROS_IP='127.0.0.1',
                      ROS_HOSTNAME='127.0.0.1', ROS_HOME=str(output / 'ros_home'),
                      ROS_LOG_DIR=str(output / 'ros_logs'))
    import rosgraph
    import rospy
    master_process = None
    cases, by_key, suite = [], {}, []
    try:
        with (output / 'master.log').open('w') as master_log:
            master_process = subprocess.Popen(['roscore', '-p', str(args.port)],
                                               stdout=master_log, stderr=subprocess.STDOUT)
            master = rosgraph.Master('/scene_suite_probe')
            deadline = time.monotonic() + 30
            while True:
                try:
                    master.getPid()
                    break
                except Exception:
                    if time.monotonic() > deadline:
                        raise RuntimeError('Scene-check ROS master did not start')
                    time.sleep(0.1)
            rospy.init_node('scene_suite_probe', disable_signals=True)
            specifications = [(6, -1), (8, -1), (0, 2), (6, 1)] + [(8, i) for i in range(1, 11)] + [(8, 1)]
            for index, (count, seed) in enumerate(specifications):
                params = [map_parameters(root, count, seed, bool(i), output / ('config_cache_%d' % i))
                          for i, root in enumerate([ROOT, reference])]
                assert params[0] == params[1], 'Resolved map parameters differ'
                values = [snapshot(binary, params[i], 'scene_%02d_%d' % (index, i), output)
                          for i, binary in enumerate(binaries)]
                assert values[0] == values[1], 'Actual geometry or point-cloud bytes differ'
                current = values[0]
                assert current['obstacle_count'] == 9 + count
                if by_key:
                    assert current['boxes_center_xyz_size_xyz'][:9] == by_key[(6, -1)]['boxes_center_xyz_size_xyz'][:9], 'Fixed nine obstacles changed'
                if (count, seed) in by_key:
                    assert current == by_key[(count, seed)], 'Same seed/count did not reproduce identical map'
                by_key[(count, seed)] = current
                cases.append(dict(small_obstacle_count=count, small_position_seed=seed, **current))
                if count == 8 and seed >= 1 and index != len(specifications) - 1:
                    command = 'roslaunch ego_planner rectangle_8uav_sparse_random.launch random_seed:=17 small_obstacle_count:=8 small_position_seed:=%d' % seed
                    suite.append(dict(scene_id='%02d' % seed, random_seed=17,
                                      small_obstacle_count=8, small_position_seed=seed,
                                      baseline_command=command,
                                      commented_command=command + ' coordination_mode:=distributed', **current))
                print('PASS count=%d seed=%d: both binaries identical; fixed 9 unchanged; cloud=%s' %
                      (count, seed, current['pointcloud_data_sha256'][:16]), flush=True)
            default = by_key[(6, -1)]['boxes_center_xyz_size_xyz']
            assert by_key[(8, -1)]['boxes_center_xyz_size_xyz'][:15] == default
            assert by_key[(8, 1)]['boxes_center_xyz_size_xyz'][:15] == by_key[(6, 1)]['boxes_center_xyz_size_xyz']
            for seed in range(1, 11):
                assert [b[3:] for b in by_key[(8, seed)]['boxes_center_xyz_size_xyz'][9:]] == [b[3:] for b in by_key[(8, 1)]['boxes_center_xyz_size_xyz'][9:]], 'Small sizes vary with position seed'
            assert len({s['geometry_sha256'] for s in suite}) == 10, 'Suite maps are not distinct'
            previous = ROOT / 'recordings/random_default_10runs_20261025/trial_001/metadata.json'
            preserved = None
            if previous.exists():
                recorded = json.loads(previous.read_text())['obstacles']
                assert default == [b['center'] + b['size'] for b in recorded], 'Default map differs from previously recorded bags'
                preserved = True
            report = dict(passed=True, reference=str(reference), source_sha256=sha(ROOT / MAP_SOURCE),
                          baseline_binary_sha256=sha(binaries[0]), reference_binary_sha256=sha(binaries[1]),
                          fixed_obstacles=9, same_parameters_geometry_and_cloud_bytes=True,
                          small_sizes_independent_of_position_seed=True, repeated_seed_reproducible=True,
                          previous_default_bag_geometry_preserved=preserved, cases=cases)
            (output / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
            if args.write_manifest:
                manifest = dict(description='Matched scenes: fixed random_seed=17, eight small blocks, position seeds 1..10',
                                reference_project=str(reference), map_source_sha256=sha(ROOT / MAP_SOURCE),
                                geometry_encoding='SHA-256 of consecutive little-endian <6d center XYZ and size XYZ in marker order; timestamps excluded',
                                validation_report=str(output / 'validation.json'), scenes=suite)
                args.write_manifest.write_text(json.dumps(manifest, indent=2) + '\n')
            print('PASS: ten distinct matched scenes; default layout preserved; counts 0/6/8 verified.', flush=True)
    finally:
        if rospy.core.is_initialized():
            rospy.signal_shutdown('scene comparison finished')
        stop(master_process)


if __name__ == '__main__':
    main()
