#!/usr/bin/env python3
"""Check the baseline against the actual resolved reference scenes and official core."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = Path('swarm-playground/formation_ws/src')
LAUNCH = SRC / 'planner/plan_manage/launch'


def dump_config(repository, filename, overrides):
    # Resolve each workspace in a separate process to avoid rospack's package cache.
    env = dict(os.environ)
    env['ROS_PACKAGE_PATH'] = str(repository / SRC) + ':/opt/ros/noetic/share'
    code = '''import json, sys, roslaunch.config
c = roslaunch.config.load_config_default([(sys.argv[1], sys.argv[2:])], 11311, verbose=False, assign_machines=False)
print(json.dumps({'params': {k: p.value for k, p in c.params.items()}, 'nodes': [{'package': n.package, 'type': n.type, 'name': n.name} for n in c.nodes]}))
'''
    output = subprocess.check_output(
        [sys.executable, '-c', code, str(repository / LAUNCH / filename),
         'rviz:=false'] + overrides, env=env, text=True)
    return json.loads(output)


def verify(reference):
    manifest = json.loads((ROOT / 'docs/original_8uav_scene_sources.json').read_text())
    for path, record in manifest['files'].items():
        actual = hashlib.sha256((reference / path).read_bytes()).hexdigest()
        assert actual == record['sha256'], 'Reference has changed: ' + path
        if record['copied_verbatim']:
            assert (ROOT / path).read_bytes() == (reference / path).read_bytes(), path
    # Check every official planner file, including launch defaults and build files.
    official_files = subprocess.check_output(
        ['git', '-C', str(ROOT), 'ls-tree', '-r', '--name-only',
         manifest['official_commit'], '--', str(SRC / 'planner')], text=True).splitlines()
    for path in official_files:
        expected = subprocess.check_output(
            ['git', '-C', str(ROOT), 'show', manifest['official_commit'] + ':' + path])
        assert (ROOT / path).read_bytes() == expected, 'Official planner file changed: ' + path
    cases = [
        ('wall', 'rectangle_8uav_fixed_split_merge_wall.launch', []),
        ('wall_custom', 'rectangle_8uav_fixed_split_merge_wall.launch',
         ['wall_size_y:=3.5', 'second_wall_gap_size_y:=2.2', 'max_vel:=0.4']),
        ('random_seed17', 'rectangle_8uav_sparse_random.launch', []),
        ('random_small42', 'rectangle_8uav_sparse_random.launch',
         ['random_seed:=17', 'small_position_seed:=42']),
    ]
    results = []
    for label, filename, overrides in cases:
        baseline = dump_config(ROOT, filename, overrides)
        original = dump_config(reference, filename, ['coordination_mode:=distributed'] + overrides)
        bp, rp = baseline['params'], original['params']
        map_prefixes = ('/fixed_split_merge_wall_map/', '/sparse_random_obstacle_map/')
        environment_prefixes = map_prefixes + tuple(
            '/drone_%d_%s/' % (i, suffix) for i in range(8)
            for suffix in ['poscmd_2_odom', 'pcl_render_node', 'odom_visualization'])
        environment = {k: v for k, v in bp.items() if k.startswith(environment_prefixes)}
        expected_environment = {k: v for k, v in rp.items() if k.startswith(environment_prefixes)}
        assert environment == expected_environment, (label, 'environment parameters differ',
            {k: (environment.get(k), expected_environment.get(k)) for k in set(environment) | set(expected_environment)
             if environment.get(k) != expected_environment.get(k)})
        common = [k for k in bp if k in rp and '_ego_planner_node/' in k]
        differences = {k: (bp[k], rp[k]) for k in common if bp[k] != rp[k]}
        assert not differences, (label, 'shared planner parameters differ', differences)
        planners = [n for n in baseline['nodes'] if n['type'] == 'ego_planner_node']
        assert len(planners) == 8
        allowed = {'ego_planner_node', 'traj_server', 'poscmd_2_odom', 'odom_visualization',
                   'pcl_render_node', 'bridge_node_udp', 'traj2odom_node',
                   'fixed_split_merge_wall_map', 'sparse_random_obstacle_map'}
        assert all(n['type'] in allowed for n in baseline['nodes']), 'Unexpected control node'
        for i in range(8):
            prefix = '/drone_%d_ego_planner_node/' % i
            assert bp[prefix + 'formation/num'] == 8
            assert bp[prefix + 'formation/drone%d' % i] == [0.5 if i < 4 else -0.5, -1.5 + i % 4, 0.0]
            assert bp[prefix + 'fsm/waypoint_num'] == 1
        results.append({'case': label, 'environment_parameters_matched': len(environment),
                        'shared_planner_parameters_matched': len(common), 'planner_nodes': 8})
        print('PASS ' + label + ': maps, sensors, simulator, formation, mission and common planner parameters match')
    report = {'official_planner_files_unchanged': len(official_files), 'cases': results}
    print(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=ROOT.parent / 'EGO-Planner-criterion-validation')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    report = verify(args.reference.resolve())
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + '\n')
