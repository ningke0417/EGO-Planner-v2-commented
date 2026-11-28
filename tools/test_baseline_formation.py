#!/usr/bin/env python3
"""Check actual ROS-resolved fleets, scene reuse, and all RViz topic bindings."""
import importlib.util
import copy
from pathlib import Path
import re
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'swarm-playground/formation_ws/src/planner/plan_manage'
LAUNCH = PACKAGE / 'launch'
spec = importlib.util.spec_from_file_location('baseline_formation',
                                            PACKAGE / 'scripts/launch_baseline_formation.py')
fleet = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fleet)
GRIDS = [(2, 3), (3, 4), (4, 4)]


class FormationConfigTest(unittest.TestCase):
    def test_front_to_rear_row_major_numbering(self):
        offsets = fleet.formation_offsets({'formation_rows': 2, 'formation_columns': 3})
        self.assertEqual(offsets, [[0.5, -1.0, 0.0], [0.5, 0.0, 0.0], [0.5, 1.0, 0.0],
                                   [-0.5, -1.0, 0.0], [-0.5, 0.0, 0.0], [-0.5, 1.0, 0.0]])
        for rows, columns in GRIDS:
            offsets = fleet.formation_offsets({'formation_rows': rows, 'formation_columns': columns})
            self.assertEqual(len(offsets), rows * columns)
            self.assertEqual(len(set(map(tuple, offsets))), rows * columns)
            for axis in range(3):
                self.assertAlmostEqual(sum(p[axis] for p in offsets), 0.0)

    def test_invalid_missions_are_rejected(self):
        for params in ({'formation_rows': 0}, {'formation_rows': 5},
                       {'formation_rows': 2.5}, {'formation_columns': True},
                       {'row_spacing': -1}, {'column_spacing': float('nan')},
                       {'start_x': float('inf')}, {'goal_x': -8},
                       {'map_size_x': 10}, {'altitude': 4},
                       {'small_obstacle_count': -1}, {'scene': 'missing'},
                       {'rviz': 'yes'}, {'odom_topic': '/absolute'},
                       {'small_size_min': 1.2}, {'small_size_max': 0},
                       {'boundary_clearance': -0.1},
                       {'second_wall_gap_size_y': 20}):
            with self.subTest(params=params), self.assertRaises((ValueError, OverflowError)):
                fleet.normalized_params(params)

    def test_rviz_all_and_only_active_vehicles(self):
        template = yaml.safe_load((LAUNCH / 'baseline_8uav.rviz').read_text())
        original = copy.deepcopy(template)
        for rows, columns in GRIDS:
            config = fleet.build_rviz_config({'formation_rows': rows, 'formation_columns': columns}, template)
            manager = config['Visualization Manager']
            self.assertEqual(manager['Global Options'], template['Visualization Manager']['Global Options'])
            groups = [d for d in manager['Displays'] if re.fullmatch(r'drone\d+', d.get('Name', ''))]
            self.assertEqual([g['Name'] for g in groups], ['drone%d' % i for i in range(rows * columns)])
            for drone, group in enumerate(groups):
                text = yaml.safe_dump(group)
                self.assertEqual(set(re.findall(r'/drone_(\d+)_', text)), {str(drone)})
                self.assertIn('/drone_%d_odom_visualization/robot' % drone, text)
                self.assertIn('/drone_%d_ego_planner_node/optimal_list' % drone, text)
            self.assertEqual(len([d for d in manager['Displays'] if d.get('Name') == 'simulation_map']), 1)
        self.assertEqual(template, original)


class ResolvedLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import roslaunch.config
        except ImportError:
            raise unittest.SkipTest('source ROS Noetic to check resolved launch configurations')
        cls.loader = staticmethod(roslaunch.config.load_config_default)

    def resolve_file(self, path, overrides=()):
        return self.loader([(str(path), list(overrides))], 11311,
                           verbose=False, assign_machines=False)

    def resolve(self, params):
        with tempfile.TemporaryDirectory(prefix='ego-rectangle-test-') as directory:
            directory = Path(directory)
            formation = directory / 'formation.yaml'
            rviz = directory / 'formation.rviz'
            formation.write_text(yaml.safe_dump(fleet.formation_config(params)))
            child = directory / 'fleet.launch'
            child.write_text(fleet.build_launch_xml(params, formation, rviz))
            return self.resolve_file(child)

    def test_entry_arguments_match_launcher_defaults(self):
        for scene in ('fixed_wall', 'sparse_random', 'cluttered_random'):
            config = self.resolve_file(LAUNCH / 'rectangle_formation.launch', ['scene:=' + scene])
            values = {key.rsplit('/', 1)[-1]: param.value for key, param in config.params.items()
                      if key.startswith('/baseline_formation_launcher/')}
            self.assertEqual(values, fleet.normalized_params({'scene': scene}))

    def test_cluttered_defaults_and_member_positions(self):
        params = fleet.normalized_params(dict(scene='cluttered_random',
                                             formation_rows=4, formation_columns=4))
        config = self.resolve(params)
        values = {k: p.value for k, p in config.params.items()}
        prefix = '/sparse_random_obstacle_map/'
        for key, expected in dict(seed=17, small_position_seed=-1,
                                  large_obstacle_count=0, small_obstacle_count=60,
                                  small_size_min=0.45, small_size_max=1.1,
                                  map_size_y=18.0, start_x=-17.0, goal_x=17.0,
                                  endpoint_clearance=5.0, boundary_clearance=0.8,
                                  obstacle_separation=0.6, small_center_y_limit=8.0,
                                  structured_challenge_enabled=False).items():
            self.assertEqual(values[prefix + key], expected)
        self.assertEqual(sum(n.type == 'ego_planner_node' for n in config.nodes), 16)
        for drone, offset in enumerate(fleet.formation_offsets(params)):
            planner = '/drone_%d_ego_planner_node/' % drone
            self.assertEqual(values[planner + 'formation/num'], 16)
            self.assertEqual(values[planner + 'formation/start'], [-17.0, 0.0, 1.5])
            self.assertEqual(values[planner + 'fsm/waypoint0_x'], 17.0)
            self.assertEqual(values[planner + 'grid_map/map_size_x'], 40.0)
            simulator = '/drone_%d_poscmd_2_odom/' % drone
            self.assertEqual(values[simulator + 'init_x'], -17.0 + offset[0])
            self.assertEqual(values[simulator + 'init_y'], offset[1])

    def test_cluttered_map_matches_reference_project(self):
        reference = ROOT.parent / 'EGO-Planner-criterion-validation'
        reference_package = reference / PACKAGE.relative_to(ROOT)
        reference_script = reference_package / 'scripts/launch_distributed_formation.py'
        if not reference_script.exists():
            self.skipTest('criterion-validation reference checkout is not available')
        reference_spec = importlib.util.spec_from_file_location('reference_formation', reference_script)
        reference_fleet = importlib.util.module_from_spec(reference_spec)
        reference_spec.loader.exec_module(reference_fleet)
        map_source = Path('swarm-playground/formation_ws/src/uav_simulator/map_generator/src/sparse_random_obstacle_map.cpp')
        self.assertEqual((ROOT / map_source).read_bytes(), (reference / map_source).read_bytes())
        for overrides in ([], ['random_seed:=23', 'small_obstacle_count:=40',
                               'small_size_min:=0.5', 'small_size_max:=0.8',
                               'boundary_clearance:=1.0']):
            entry = self.resolve_file(reference_package / 'launch/distributed_cluttered_random.launch', overrides)
            inputs = {k.rsplit('/', 1)[-1]: p.value for k, p in entry.params.items()
                      if k.startswith('/distributed_formation_launcher/')}
            reference_params = reference_fleet.normalized_params(inputs)
            baseline_entry = self.resolve_file(LAUNCH / 'rectangle_formation.launch',
                                               ['scene:=cluttered_random'] + overrides)
            baseline_params = {k.rsplit('/', 1)[-1]: p.value for k, p in baseline_entry.params.items()
                               if k.startswith('/baseline_formation_launcher/')}
            reference_xml = ET.fromstring(reference_fleet.build_launch_xml(reference_params, reference_package))
            map_node = next(n for n in reference_xml.findall('node')
                            if n.get('type') == 'sparse_random_obstacle_map')
            map_launch = ET.Element('launch')
            map_launch.append(map_node)
            with tempfile.TemporaryDirectory(prefix='reference-map-test-') as directory:
                path = Path(directory) / 'map.launch'
                path.write_text(ET.tostring(map_launch, encoding='unicode'))
                expected = {k: p.value for k, p in self.resolve_file(path).params.items()
                            if k.startswith('/sparse_random_obstacle_map/')}
            for rows, columns in GRIDS + [(2, 4)]:
                with self.subTest(overrides=overrides, rows=rows, columns=columns):
                    params = fleet.normalized_params(dict(baseline_params,
                                                         formation_rows=rows, formation_columns=columns))
                    config = self.resolve(params)
                    actual = {k: p.value for k, p in config.params.items()
                              if k.startswith('/sparse_random_obstacle_map/')}
                    self.assertEqual(actual, expected)
                    shape = dict(reference_params, formation_rows=rows, formation_columns=columns)
                    self.assertEqual(fleet.formation_offsets(params), reference_fleet.formation_offsets(shape))

    def test_six_fleets_and_shared_scene_parameters(self):
        for scene in ('fixed_wall', 'sparse_random'):
            legacy_file = ('rectangle_8uav_fixed_split_merge_wall.launch' if scene == 'fixed_wall'
                           else 'rectangle_8uav_sparse_random.launch')
            overrides = ['rviz:=false'] + (['wall_size_y:=6.0'] if scene == 'fixed_wall' else [])
            legacy = self.resolve_file(LAUNCH / legacy_file, overrides)
            map_prefix = '/fixed_split_merge_wall_map/' if scene == 'fixed_wall' else '/sparse_random_obstacle_map/'
            expected_map = {k: p.value for k, p in legacy.params.items() if k.startswith(map_prefix)}
            for rows, columns in GRIDS:
                with self.subTest(scene=scene, rows=rows, columns=columns):
                    params = fleet.normalized_params(dict(scene=scene, formation_rows=rows, formation_columns=columns))
                    config = self.resolve(params)
                    values = {k: p.value for k, p in config.params.items()}
                    count = rows * columns
                    self.assertEqual({k: v for k, v in values.items() if k.startswith(map_prefix)}, expected_map)
                    for executable in ('ego_planner_node', 'traj_server', 'poscmd_2_odom',
                                       'odom_visualization', 'pcl_render_node'):
                        self.assertEqual(sum(n.type == executable for n in config.nodes), count)
                    for drone, offset in enumerate(fleet.formation_offsets(params)):
                        prefix = '/drone_%d_ego_planner_node/' % drone
                        actual = {k[len(prefix + 'formation/'):]: v for k, v in values.items()
                                  if k.startswith(prefix + 'formation/')}
                        self.assertEqual(actual['num'], count)
                        self.assertEqual(len(actual), count + 2)  # count, start, all slots
                        for slot, expected in enumerate(fleet.formation_offsets(params)):
                            self.assertEqual(actual['drone%d' % slot], expected)
                        self.assertEqual(actual['start'], [params['start_x'], 0.0, 1.5])
                        self.assertEqual(values[prefix + 'fsm/waypoint0_x'], params['goal_x'])
                        self.assertEqual(values['/drone_%d_poscmd_2_odom/init_x' % drone], params['start_x'] + offset[0])
                        self.assertEqual(values['/drone_%d_poscmd_2_odom/init_y' % drone], offset[1])
                        self.assertEqual(values['/drone_%d_odom_visualization/robot_scale' % drone], 0.35)
                        self.assertTrue(values['/drone_%d_pcl_render_node/full_360_sensing' % drone])

    def test_custom_spacing_altitude_odometry_and_bag(self):
        params = dict(formation_rows=3, formation_columns=4, row_spacing=0.8,
                      column_spacing=1.2, altitude=1.2, odom_topic='state/odom',
                      record_bag=True, bag_prefix='/tmp/rectangle_test')
        config = self.resolve(params)
        values = {k: p.value for k, p in config.params.items()}
        for drone in range(12):
            prefix = '/drone_%d_ego_planner_node/' % drone
            self.assertEqual(values[prefix + 'formation/start'], [-6.0, 0.0, 1.2])
            self.assertEqual(values[prefix + 'fsm/waypoint0_z'], 1.2)
            visualizer = next(n for n in config.nodes if n.name == 'drone_%d_odom_visualization' % drone)
            self.assertIn(['~odom', 'drone_%d_state/odom' % drone], visualizer.remap_args)
        recorder = next(n for n in config.nodes if n.type == 'record')
        self.assertIn('/drone_11_state/odom', recorder.args)
        self.assertNotIn('/drone_12_', recorder.args)


if __name__ == '__main__':
    unittest.main()
