#!/usr/bin/env python3
"""Audit the first trial of each project against a previous random-scene batch."""
import argparse
import csv
import json
import math
from pathlib import Path

from run_comparison_benchmark import (PROJECTS, FORBIDDEN, save, sha,
                                      trial_bag_filename)


def verify_startup(output, reference):
    if not all((output/project/'run_001/metrics.json').exists() for project in PROJECTS):
        return None
    import numpy as np
    import yaml
    config = json.loads((output/'config.json').read_text())
    checks, geometries = [], []
    for project in PROJECTS:
        run = output/project/'run_001'
        result = json.loads((run/'metrics.json').read_text())
        if result['reason'] == 'infrastructure_error' or not result.get('bag_valid'):
            raise RuntimeError('First trial failed recording audit: '+project)
        filename = trial_bag_filename(config, project, 1)
        bag = run/filename
        if (result['bag_file'] != filename or not bag.is_file()
                or Path(str(bag)+'.active').exists() or bag.stat().st_size != result['bag_bytes']
                or sha(bag) != result['bag_sha256']):
            raise RuntimeError('First trial bag checksum/filename failed: '+project)
        bounds, rows = {}, 0
        with (run/'drone_positions.csv').open() as stream:
            for row in csv.DictReader(stream):
                drone = int(row['drone_id']); rows += 1
                if not all(math.isfinite(float(row[key])) for key in ['x', 'y', 'z', 'vx', 'vy', 'vz']):
                    raise RuntimeError('Invalid native odometry value: '+project)
                stamp = float(row['odom_ros_time'])
                lo, hi = bounds.get(drone, (math.inf, -math.inf))
                bounds[drone] = (min(lo, stamp), max(hi, stamp))
        if set(bounds) != set(range(8)):
            raise RuntimeError('Missing physical drone position streams: '+project)
        if result['departure_ros_time'] is not None:
            end = result.get('all_arrived_ros_time') or result['decision_ros_time']
            if not all(lo <= result['departure_ros_time'] and hi >= end for lo, hi in bounds.values()):
                raise RuntimeError('Position CSV does not cover evaluated flight: '+project)
        actual = sorted(json.loads((run/'scene_geometry.json').read_text())['boxes_xyz_min_max'])
        np.testing.assert_allclose(actual, sorted(config['expected_boxes']), atol=1e-8, rtol=0)
        geometries.append(actual)
        old = yaml.load((reference/project/'run_001/ros_parameters.yaml').read_text(), Loader=yaml.CSafeLoader)
        new = yaml.load((run/'ros_parameters.yaml').read_text(), Loader=yaml.CSafeLoader)
        for key, value in config['generator_parameters'].items():
            if new['sparse_random_obstacle_map'].get(key) != value:
                raise RuntimeError('Actual random map parameters differ: '+project+'/'+key)
        for drone in range(8):
            key = 'drone_%d_ego_planner_node' % drone
            for group in ('optimization', 'formation', 'fsm', 'global_goal'):
                if old[key].get(group) != new[key].get(group):
                    raise RuntimeError('Planner parameters differ: '+project+'/'+key+'/'+group)
        if project == 'ours' and (not new['distributed_swap_enabled'] or not all(
                new['drone_%d_ego_planner_node' % i]['formation']['distributed_enabled'] for i in range(8))):
            raise RuntimeError('Ours must use distributed formation with swapping enabled')
        topics = json.loads((run/'recorded_topics.json').read_text())
        if {item['type'] for item in topics.values()} & FORBIDDEN or not all(
                '/drone_%d_visual_slam/odom' % i in topics for i in range(8)):
            raise RuntimeError('Missing odometry or forbidden heavy topic: '+project)
        error = result.get('formation_error_eq45')
        if error is not None and not math.isfinite(error):
            raise RuntimeError('Nonfinite equation45 metric: '+project)
        checks.append(dict(project=project, first_trial_verified=True, bag_sha256_verified=True,
             all_eight_position_streams_verified=True, planner_parameters_match_reference=True,
             obstacle_count=len(actual), topic_count=len(topics), position_rows=rows,
             arrival_success=result['arrival_success'], collision_free_success=result['collision_free_success']))
    for geometry in geometries[1:]:
        np.testing.assert_allclose(geometry, geometries[0], atol=1e-8, rtol=0)
    result = dict(passed=True, three_project_geometry_identical=True,
                  reference=str(reference), checks=checks)
    save(output/'startup_verification.json', result)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    arguments = parser.parse_args()
    result = verify_startup(arguments.output, arguments.reference)
    if result is None:
        raise SystemExit('First three trials are not yet complete')
    print(json.dumps(result, ensure_ascii=False, indent=2))
