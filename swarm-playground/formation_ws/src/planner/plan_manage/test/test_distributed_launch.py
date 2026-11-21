#!/usr/bin/env python3
"""Configuration regressions for the reusable launcher; no ROS master required."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'distributed_launcher', PACKAGE / 'scripts/launch_distributed_formation.py')
launcher = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launcher)


class DistributedLaunchTest(unittest.TestCase):
    def test_all_shapes_and_scenes_have_complete_isolated_members(self):
        for rows, columns in ((2, 3), (2, 4), (3, 4), (4, 4)):
            for scene in ('fixed_wall', 'sparse_random', 'cluttered_random'):
                with self.subTest(rows=rows, columns=columns, scene=scene):
                    params = dict(scene=scene, formation_rows=rows, formation_columns=columns,
                                  rviz=False)
                    config = launcher.normalized_params(params)
                    tree = ET.fromstring(launcher.build_launch_xml(params))
                    count = rows * columns
                    members = [e for e in tree.findall('include')
                               if e.get('file').endswith('/run_in_sim.launch')]
                    self.assertEqual(len(members), count)
                    offsets = launcher.formation_offsets(params)
                    self.assertAlmostEqual(sum(p[0] for p in offsets), 0.0)
                    self.assertAlmostEqual(sum(p[1] for p in offsets), 0.0)
                    self.assertEqual(len({tuple(p) for p in offsets}), count)
                    for drone, member in enumerate(members):
                        arguments = {a.get('name'): a.get('value') for a in member.findall('arg')}
                        self.assertEqual(arguments['distributed_enabled'], 'true')
                        self.assertEqual(int(arguments['drone_id']), drone)
                        self.assertEqual(int(arguments['formation_num']), count)
                        self.assertEqual(arguments['initial_assignment_enabled'], 'false')
                        self.assertEqual(arguments['subgroup_route_intent_feedback_enabled'], 'false')
                        self.assertAlmostEqual(float(arguments['init_x']),
                                               config['start_x'] + offsets[drone][0])
                    nodes = tree.findall('node')
                    agents = [n for n in nodes if n.get('type') == 'distributed_formation_agent']
                    observers = [n for n in nodes if n.get('type') == 'swarm_observed_map_node']
                    self.assertEqual(len(agents), count)
                    self.assertEqual(len(observers), count)
                    for drone, observer in enumerate(observers):
                        values = {p.get('name'): p.get('value') for p in observer.findall('param')}
                        self.assertEqual(int(values['num_uavs']), count)
                        self.assertEqual(int(values['local_drone_id']), drone)
                        destinations = {e.get('to') for e in observer.findall('remap')}
                        self.assertEqual(destinations, {'/drone_%d/observed_free' % drone,
                                                        '/drone_%d/observed_occupied' % drone})
                    map_types = [n.get('type') for n in nodes if n.get('pkg') == 'map_generator']
                    self.assertEqual(map_types, ['fixed_split_merge_wall_map' if scene == 'fixed_wall'
                                                 else 'sparse_random_obstacle_map'])
                    self.assertFalse(any('shadow' in n.get('type') or 'bridge' in n.get('type')
                                         for n in nodes))
                    slot_params = {p.get('param'): p.text for p in tree.findall('rosparam')}
                    for drone in range(count):
                        for slot in range(count):
                            self.assertIn('/drone_%d_ego_planner_node/formation/drone%d' % (drone, slot),
                                          slot_params)

    def test_eight_member_offsets_match_existing_launch(self):
        self.assertEqual(launcher.formation_offsets({}), [
            [0.5, -1.5, 0.0], [0.5, -0.5, 0.0],
            [0.5, 0.5, 0.0], [0.5, 1.5, 0.0],
            [-0.5, -1.5, 0.0], [-0.5, -0.5, 0.0],
            [-0.5, 0.5, 0.0], [-0.5, 1.5, 0.0]])

    def test_cluttered_profile_is_a_wide_field_of_posts_with_clear_endpoints(self):
        config = launcher.normalized_params({'scene': 'cluttered_random'})
        tree = ET.fromstring(launcher.build_launch_xml(config))
        generator = next(n for n in tree.findall('node') if n.get('pkg') == 'map_generator')
        params = {p.get('name'): p.get('value') for p in generator.findall('param')}
        self.assertEqual(generator.get('type'), 'sparse_random_obstacle_map')
        self.assertEqual(int(params['large_obstacle_count']), 0)
        self.assertEqual(int(params['small_obstacle_count']), 60)
        self.assertEqual(params['structured_challenge_enabled'], 'false')
        self.assertEqual(float(params['small_size_min']), 0.45)
        self.assertEqual(float(params['small_size_max']), 1.1)
        self.assertEqual(float(params['small_center_y_limit']), 8.0)
        self.assertEqual(float(params['obstacle_separation']), 0.6)
        self.assertEqual(float(params['endpoint_clearance']), 5.0)
        self.assertTrue(config['swap_enabled'])
        self.assertEqual(config['map_size_x'], 40.0)
        self.assertEqual(config['map_size_y'], 18.0)
        # The nearest possible obstacle face stays clear of the entire 4x4
        # start/goal footprint, including the configured collision clearance.
        largest = launcher.normalized_params(dict(scene='cluttered_random', formation_rows=4, formation_columns=4))
        half_formation_x = (largest['formation_rows'] - 1) * largest['row_spacing'] / 2.0
        self.assertGreater(largest['endpoint_clearance'], half_formation_x + largest['safety_distance'])

    def test_original_scene_defaults_are_preserved(self):
        wall = launcher.normalized_params({'scene': 'fixed_wall'})
        sparse = launcher.normalized_params({'scene': 'sparse_random'})
        self.assertEqual((wall['map_size_x'], wall['map_size_y'], wall['start_x'], wall['goal_x']),
                         (24.0, 14.0, -6.0, 10.0))
        self.assertEqual((sparse['map_size_x'], sparse['map_size_y'], sparse['start_x'], sparse['goal_x']),
                         (32.0, 14.0, -14.0, 14.0))
        self.assertEqual((sparse['large_obstacle_count'], sparse['small_obstacle_count']), (4, 6))
        self.assertTrue(sparse['structured_challenge_enabled'])
        self.assertEqual((sparse['small_size_min'], sparse['small_size_max']), (0.45, 0.9))
        self.assertEqual((sparse['boundary_clearance'], sparse['endpoint_clearance'], sparse['obstacle_separation']),
                         (0.8, 3.0, 0.7))

    def test_invalid_configuration_fails_before_launch(self):
        for params in (
                {'formation_rows': 0}, {'formation_columns': 4.5},
                {'formation_rows': True}, {'formation_rows': 5},
                {'row_spacing': float('nan')}, {'column_spacing': -1},
                {'formation_rows': 3, 'swap_scope': 'column'},
                {'scene': 'not_a_scene'}, {'swap_enabled': 'invalid'},
                {'small_obstacle_count': -1}, {'small_obstacle_count': 3.5},
                {'small_size_min': 1.1}, {'small_size_max': float('inf')},
                {'obstacle_separation': -0.1},
                {'start_x': 10.0}, {'map_size_y': 2.0}, {'not_a_parameter': 1}):
            with self.subTest(params=params), self.assertRaises(ValueError):
                launcher.build_launch_xml(params)

    def test_public_entries_delegate_to_the_same_executable(self):
        for filename, scene in (('distributed_fixed_wall.launch', 'fixed_wall'),
                                ('distributed_sparse_random.launch', 'sparse_random'),
                                ('distributed_cluttered_random.launch', 'cluttered_random')):
            tree = ET.parse(PACKAGE / 'launch' / filename).getroot()
            node = tree.find('node')
            self.assertEqual(node.get('type'), 'launch_distributed_formation.py')
            values = {p.get('name'): p.get('value') for p in node.findall('param')}
            self.assertEqual(values['scene'], scene)
            self.assertEqual(values['formation_rows'], '$(arg formation_rows)')
            self.assertEqual(values['formation_columns'], '$(arg formation_columns)')


if __name__ == '__main__':
    unittest.main()
