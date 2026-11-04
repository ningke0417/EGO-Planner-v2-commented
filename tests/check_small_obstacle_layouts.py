"""Map-only integration regression; run in sourced Noetic, without UAVs."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import numpy as np
import rospy
import yaml
from visualization_msgs.msg import MarkerArray

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'bags/small_obstacle_layout_check'
OUT.mkdir(parents=True, exist_ok=True)
params = yaml.safe_load((ROOT/'bags/sparse_random_distributed_default_20261025/trial_01/params.yaml').read_text())['sparse_random_obstacle_map']
baseline = np.asarray(json.loads((ROOT/'bags/sparse_random_distributed_default_20261025/trial_01/obstacle_geometry.json').read_text())['boxes_xyz_min_max'])
env = os.environ.copy()
env.update(ROS_MASTER_URI='http://127.0.0.1:11441', ROS_IP='127.0.0.1',
           ROS_HOME=str(OUT/'ros_home'), ROS_LOG_DIR=str(OUT/'ros_logs'))
os.environ.update(env)
with socket.socket() as check_port:
    check_port.bind(('127.0.0.1', 11441))
master = subprocess.Popen(['roscore', '-p', '11441'], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
cases = {}
try:
    for _ in range(100):
        try:
            import rosgraph
            if rosgraph.Master('/layout_check').getPid():
                break
        except Exception:
            time.sleep(.1)
    rospy.init_node('layout_check', disable_signals=True)
    for index, (count, seed) in enumerate([(6, -1), (8, -1), (8, 1), (8, 2), (0, 2)]):
        name = 'layout_check_case_%d' % index
        config = dict(params, small_obstacle_count=count, small_position_seed=seed)
        rospy.set_param('/'+name, config)
        topic = '/'+name+'/geometry'
        with (OUT/(name+'.log')).open('w') as log:
            node = subprocess.Popen(['rosrun', 'map_generator', 'sparse_random_obstacle_map',
                '__name:='+name, '/map_generator/sparse_random_geometry:='+topic,
                'global_cloud:=/'+name+'/cloud'], env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                markers = rospy.wait_for_message(topic, MarkerArray, timeout=15)
                boxes = []
                for m in markers.markers:
                    p, s = m.pose.position, m.scale
                    boxes.append((p.x-s.x/2, p.x+s.x/2, p.y-s.y/2, p.y+s.y/2, p.z-s.z/2, p.z+s.z/2))
                cases[(count, seed)] = np.asarray(boxes)
                assert len(boxes) == 9+count
                np.testing.assert_allclose(cases[(count, seed)][:9], baseline[:9], atol=1e-12, rtol=0)
                print('PASS count=%d small_position_seed=%d: fixed 9 boxes unchanged' % (count, seed), flush=True)
            finally:
                node.send_signal(signal.SIGINT)
                node.wait(timeout=10)
    np.testing.assert_allclose(cases[(6, -1)], baseline, atol=1e-12, rtol=0)
    np.testing.assert_allclose(cases[(8, -1)][:15], baseline, atol=1e-12, rtol=0)
    a, b = cases[(8, 1)][9:], cases[(8, 2)][9:]
    assert not np.allclose(a[:, :4], b[:, :4])
    np.testing.assert_allclose(a[:, 1]-a[:, 0], b[:, 1]-b[:, 0], atol=1e-12)
    np.testing.assert_allclose(a[:, 3]-a[:, 2], b[:, 3]-b[:, 2], atol=1e-12)
    (OUT/'result.json').write_text(json.dumps(dict(passed=True,
        default_matches_recorded_map=True, large_stacked_gap_unchanged=True,
        small_sizes_equal_between_seeds=True, small_positions_differ_between_seeds=True,
        cases=[dict(count=k[0], small_position_seed=k[1], boxes=v.tolist()) for k,v in cases.items()]), indent=2)+'\n')
    print('PASS default preserved; independent seeds change positions and preserve small sizes.', flush=True)
finally:
    master.send_signal(signal.SIGINT)
    master.wait(timeout=15)
