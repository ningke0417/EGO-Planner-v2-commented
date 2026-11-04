"""Regression tests for experiment accounting, without a ROS installation."""
import importlib.util
from pathlib import Path
import re
import unittest
from argparse import Namespace

spec = importlib.util.spec_from_file_location('batch', Path(__file__).resolve().parents[1]/'scripts/record_fixed_wall_batch.py')
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)


class MetricsTests(unittest.TestCase):
    def setUp(self):
        self.m = batch.Metrics(dict(wall_length=8, departure_distance=0.05,
            pair_collision_distance=0.2, arrival_radius=0.3, arrival_speed=0.15, arrival_hold=1))

    def feed(self, t, count=8, state='WAIT_TARGET'):
        for i in range(count):
            p = (10.5 if i < 4 else 9.5, i % 4-1.5, 1.5)
            self.m.update_odom(i, t, p, (0, 0, 0))
            self.m.update_goal(i, t, p, list(range(8)), 1)
            self.m.status[i] = (t, state, False)

    def test_departure_and_stable_all_arrival(self):
        self.m.update_odom(0, 1, (-5.5, -1.5, 1.5), (0, 0, 0))
        self.assertIsNone(self.m.departure)
        self.m.update_odom(0, 2, (-5.44, -1.5, 1.5), (0.5, 0, 0))
        self.assertEqual(self.m.departure, 2)
        self.feed(40, 7)
        self.assertFalse(self.m.tick(40))
        self.feed(41)
        self.assertFalse(self.m.tick(41))
        self.feed(42.1)
        self.assertTrue(self.m.tick(42.1))
        self.assertEqual(self.m.result('all_arrived', 42.1)['completion_time_s'], 39)

    def test_stale_data_cannot_succeed(self):
        self.m.departure = 1
        self.feed(10)
        self.m.tick(10)
        self.assertFalse(self.m.tick(12))

    def test_goal_change_resets_hold(self):
        self.m.departure = 1
        self.feed(10)
        self.m.tick(10)
        self.m.update_goal(0, 10.9, (9.5, -1.5, 1.5), [4, 1, 2, 3, 0, 5, 6, 7], 2)
        self.assertIsNone(self.m.all_since)
        self.assertFalse(self.m.tick(11))
        self.assertEqual(self.m.assignment_changes, 1)

    def test_timeout_runtime_null(self):
        self.m.departure = 1
        self.assertIsNone(self.m.result('flight_timeout', 241)['completion_time_s'])

    def test_wall_and_pair_collision_invalidate_success(self):
        self.m.update_odom(0, 1, (-2, 3.9, 1.5), (0, 0, 0))
        self.m.update_odom(1, 1, (-2, 3.95, 1.5), (0, 0, 0))
        self.m.tick(1)
        self.assertIn(('wall', 0), self.m.collisions)
        self.assertIn(('pair', 0, 1), self.m.collisions)
        self.m.departure, self.m.all_since = 1, 10
        self.assertFalse(self.m.result('all_arrived', 11)['success'])

    def test_topic_allowlist(self):
        for name in ['/swarm/agent_state', '/swarm/assignment', '/drone_7/formation_applied',
                     '/drone_7_traj_server/heartbeat',
                     '/drone_3_visual_slam/odom', '/formation_metrics/drone_1/planning']:
            self.assertIsNotNone(re.fullmatch(batch.TOPICS, name), name)
        for name in ['/map_generator/global_cloud', '/drone_0/observed_free',
                     '/drone_1_ego_planner_node/grid_map/formation_occupancy',
                     '/drone_2_pcl_render_node/cloud', '/camera/image_raw']:
            self.assertIsNone(re.fullmatch(batch.TOPICS, name), name)

    def test_random_scene_never_uses_fixed_wall_geometry(self):
        config = self.m.c.copy()
        config['scene'] = 'sparse_random'
        m = batch.Metrics(config)
        self.assertIsNone(m.wall_distance((-2, 0, 1.5)))
        m.obstacle_boxes = [(1, 2, 3, 4, 0, 3)]
        self.assertEqual(m.wall_distance((1.5, 3.5, 1.5)), 0)
        self.assertAlmostEqual(m.wall_distance((2.5, 3.5, 1.5)), .5)
        m.update_odom(0, 1, (-2, 0, 1.5), (0, 0, 0))
        m.tick(1)
        self.assertEqual(m.collisions, [])

    def test_random_defaults_and_local_applied_slot(self):
        name, cmd, swaps, seed = batch.launch_configuration(Namespace(
            scene='sparse_random', swap_enabled=None, random_seed=None, wall_length=8))
        self.assertEqual(name, batch.RANDOM_LAUNCH)
        self.assertTrue(swaps)
        self.assertEqual(seed, 17)
        self.assertFalse(any('wall_size_y' in x for x in cmd))
        slots = [4, 1, 2, 3, 0, 5, 6, 7]
        self.m.update_goal(0, 1, (9.5, -1.5, 1.5), slots, 2)
        self.assertEqual(self.m.applied_slots[0], 4)
        self.assertEqual(self.m.applied_slots[4], 4)  # peer not applied yet

    def test_second_random_scene_parameters(self):
        _, cmd, swaps, seed = batch.launch_configuration(Namespace(
            scene='sparse_random', swap_enabled=None, random_seed=17, wall_length=8,
            small_obstacle_count=8, small_position_seed=1))
        self.assertIn('small_obstacle_count:=8', cmd)
        self.assertIn('small_position_seed:=1', cmd)
        self.assertEqual(seed, 17)
        self.assertTrue(swaps)


if __name__ == '__main__':
    unittest.main()
