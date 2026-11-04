#!/usr/bin/env python3
"""Read geometry from each project's real map binary in an isolated ROS1 container."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import time
import xmlrpc.client

from run_comparison_benchmark import PROJECTS, random_map_parameters, save, sha, stop, wait


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--random-seed', type=int, default=17)
    parser.add_argument('--small-obstacle-count', type=int, default=8)
    parser.add_argument('--small-position-seed', type=int, default=1)
    args = parser.parse_args()
    import numpy as np
    import rospy
    from visualization_msgs.msg import MarkerArray
    args.output.mkdir(parents=True, exist_ok=False)
    parameters = dict(random_seed=args.random_seed, small_obstacle_count=args.small_obstacle_count,
                      small_position_seed=args.small_position_seed)
    all_parameters = {name: random_map_parameters(name, parameters) for name in PROJECTS}
    if any(p != all_parameters['ours'] for p in all_parameters.values()):
        raise RuntimeError('Project launch files specify different random map parameters')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP='127.0.0.1',
                      ROS_HOME=str(args.output/'ros_home'), ROS_LOG_DIR=str(args.output/'ros_logs'))
    master = generator = None
    geometries, hashes = {}, {}
    try:
        with (args.output/'roscore.log').open('w') as master_log:
            master = subprocess.Popen(['roscore'], stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True)
        proxy = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        wait(lambda: proxy.getPid('/preflight')[0] == 1, 25, [master])
        rospy.init_node('random_geometry_preflight', disable_signals=True)
        for project, (repo, devel) in PROJECTS.items():
            binary = repo/devel/'lib/map_generator/sparse_random_obstacle_map'
            hashes[project] = {str(binary): sha(binary)}
            received = []
            def geometry(msg):
                boxes = []
                for marker in msg.markers:
                    if marker.type != marker.CUBE or marker.action != marker.ADD:
                        continue
                    p, s, q = marker.pose.position, marker.scale, marker.pose.orientation
                    if marker.header.frame_id != 'world' or max(abs(q.x), abs(q.y), abs(q.z), abs(abs(q.w)-1)) > 1e-8:
                        raise RuntimeError('Invalid geometry frame/orientation')
                    boxes.append([p.x-s.x/2, p.x+s.x/2, p.y-s.y/2, p.y+s.y/2, p.z-s.z/2, p.z+s.z/2])
                received[:] = sorted(boxes)
            subscription = rospy.Subscriber('/map_generator/sparse_random_geometry', MarkerArray, geometry, queue_size=1)
            values = all_parameters[project]
            arguments = ['_'+key+':='+str(value).lower() for key, value in values.items()]
            command = 'set -e\nsource /opt/ros/noetic/setup.bash\nsource '+shlex.quote(str(repo/devel/'setup.bash'))+'\nexec '+shlex.join([
                str(binary), '__name:=geometry_'+project, *arguments])
            with (args.output/(project+'_map.log')).open('w') as log:
                generator = subprocess.Popen(['/bin/bash', '-lc', command], stdout=log,
                                             stderr=subprocess.STDOUT, start_new_session=True)
                wait(lambda: len(received) == values['large_obstacle_count']+values['small_obstacle_count']+5,
                     30, [master, generator])
                array = np.asarray(received)
                if not np.isfinite(array).all() or not (array[:, [1, 3, 5]] > array[:, [0, 2, 4]]).all():
                    raise RuntimeError('Invalid obstacle extents')
                geometries[project] = received.copy()
                save(args.output/(project+'_geometry.json'), dict(boxes_xyz_min_max=received, parameters=values))
                stop(generator); generator = None
            subscription.unregister()
            time.sleep(.3)
        for project in PROJECTS:
            np.testing.assert_allclose(geometries[project], geometries['ours'], rtol=0, atol=1e-8)
        save(args.output/'geometry_reference.json', dict(passed=True, scene_parameters=parameters,
             generator_parameters=all_parameters['ours'], boxes_xyz_min_max=geometries['ours'],
             map_binary_sha256=hashes, three_project_geometry_identical=True))
        print(json.dumps(dict(passed=True, box_count=len(geometries['ours']), projects=list(PROJECTS))))
    finally:
        stop(generator); stop(master)
        if rospy.core.is_initialized():
            rospy.signal_shutdown('preflight complete')


if __name__ == '__main__':
    main()
