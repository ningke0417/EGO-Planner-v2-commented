import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from run_comparison_benchmark import (Monitor, STARTS, GOALS, collision_metrics, launch_command,
    bag_filename, expected_boxes, scene_positions, scene_launch_command, trial_bag_filename,
    random_map_parameters, scene_fields, summarize)
import tempfile


class ComparisonTests(unittest.TestCase):
    def feed(self, m, t):
        for i, p in enumerate(GOALS):
            m.update(i, t, p, (0, 0, 0))

    def test_velocity_does_not_change_departure_definition(self):
        m = Monitor()
        m.update(0, 1, STARTS[0], (.1, 0, 0))
        self.assertIsNone(m.departure)
        p = list(STARTS[0]); p[0] += .06
        m.update(0, 2, p, (0, 0, 0))
        self.assertEqual(m.departure, 2)

    def test_all_eight_fresh_and_stable_then_hold_start_time(self):
        m = Monitor(); m.departure = 1
        self.feed(m, 10); self.assertFalse(m.tick(10))
        self.assertFalse(m.tick(11.1))  # old odometry cannot confirm arrival
        self.feed(m, 12); self.assertFalse(m.tick(12))
        self.feed(m, 13.1); self.assertTrue(m.tick(13.1))
        self.assertEqual(m.state(13.1)['completion_time_s'], 11)

    def test_goal_swap_resets_hold_and_changes_physical_target(self):
        m = Monitor(); m.departure = 1
        self.feed(m, 10); m.tick(10)
        m.goal(0, GOALS[4], 4)
        self.assertIsNone(m.candidate)
        self.assertFalse(m.tick(10.1))

    def test_sphere_wall_clearance_checks_radius(self):
        xyz = np.array([[[1.05, 0, 0], [4, 0, 0]]])
        pair, clearance, collision = collision_metrics(xyz, [[-1, 1, -1, 1, -1, 1]])
        self.assertAlmostEqual(clearance, -.05)
        self.assertTrue(collision)  # center is outside, but sphere overlaps

    def test_pair_sphere_overlap_and_safe_case(self):
        far_wall = [[10, 11, 10, 11, 0, 1]]
        for separation, expected in [(.19, True), (.21, False)]:
            xyz = np.array([[[0, 0, 0], [separation, 0, 0]]])
            self.assertEqual(collision_metrics(xyz, far_wall)[2], expected)

    def test_same_scene_different_argument_names(self):
        for project in ('ours', 'zhou', 'quan'):
            command = launch_command(project, 6)
            self.assertIn('first_wall_size_y:=6' if project == 'quan' else 'wall_size_y:=6', command)
            self.assertIn('max_vel:=0.5', command)
        self.assertIn('coordination_mode:=distributed', launch_command('ours', 6))
        self.assertIn('swap_enabled:=true', launch_command('ours', 6))

    def test_task_two_changes_first_wall_only(self):
        a, b = expected_boxes(6), expected_boxes(7)
        self.assertEqual(b[0], [-2.25, -1.75, -3.5, 3.5, -.1, 3.0])
        self.assertEqual(a[1:], b[1:])
        for project in ('ours', 'zhou', 'quan'):
            command = launch_command(project, 7)
            self.assertIn('first_wall_size_y:=7' if project == 'quan' else 'wall_size_y:=7', command)

    def test_task_names_preserve_task_one_and_distinguish_task_two(self):
        self.assertEqual(bag_filename(1, 'ours', 6, 1), 'task01_ours_wall06m_8uav_run001.bag')
        self.assertEqual(bag_filename(2, 'zhou', 7, 50), 'task02_zhou_wall07m_8uav_run050.bag')

    def random_config(self):
        return dict(task=6, scene='sparse_random', runs_per_project=20,
                    scene_parameters=dict(random_seed=17, small_obstacle_count=8, small_position_seed=1))

    def test_random_departure_uses_actual_start_and_goal_not_wall_scene(self):
        starts, goals = scene_positions(self.random_config())
        self.assertEqual(starts[0], (-13.5, -1.5, 1.5))
        self.assertEqual(goals[0], (14.5, -1.5, 1.5))
        monitor = Monitor(starts, goals)
        for i, p in enumerate(starts): monitor.update(i, 1, p, (0, 0, 0))
        self.assertIsNone(monitor.departure)
        p = list(starts[0]); p[0] += .06
        monitor.update(0, 2, p, (0, 0, 0))
        self.assertEqual(monitor.departure, 2)
        for i, p in enumerate(GOALS): monitor.update(i, 3, p, (0, 0, 0))
        self.assertFalse(monitor.tick(3))
        for stamp in (10, 11.1):
            for i, p in enumerate(goals): monitor.update(i, stamp, p, (0, 0, 0))
            monitor.tick(stamp)
        self.assertEqual(monitor.arrival, 10)

    def test_random_commands_have_exact_seeds_and_no_wall_arguments(self):
        config = self.random_config()
        for project in ('ours', 'zhou', 'quan'):
            command = scene_launch_command(project, config)
            self.assertIn('rectangle_8uav_sparse_random.launch', command)
            for argument in ('random_seed:=17', 'small_obstacle_count:=8', 'small_position_seed:=1', 'start_x:=-14.0', 'goal_x:=14.0'):
                self.assertIn(argument, command)
            self.assertFalse(any('wall_size' in a for a in command))
        self.assertIn('coordination_mode:=distributed', scene_launch_command('ours', config))
        self.assertIn('swap_enabled:=true', scene_launch_command('ours', config))
        self.assertEqual(trial_bag_filename(config, 'quan', 20), 'task06_quan_random_map17_small08_pos01_8uav_run020.bag')
        self.assertNotIn('wall_length_m', scene_fields(config))

    def test_random_generators_resolve_to_identical_parameters(self):
        parameters = self.random_config()['scene_parameters']
        expected = random_map_parameters('ours', parameters)
        self.assertEqual(expected['seed'], 17)
        self.assertEqual(expected['small_obstacle_count'], 8)
        self.assertEqual(expected['small_position_seed'], 1)
        self.assertEqual(expected['start_x'], -14)
        self.assertTrue(expected['structured_challenge_enabled'])
        for project in ('zhou', 'quan'):
            self.assertEqual(random_map_parameters(project, parameters), expected)

    def test_random_report_does_not_require_or_describe_a_wall(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            summarize(output, self.random_config())
            report = (output/'REPORT.md').read_text()
            self.assertIn('small08 / pos01', report)
            self.assertNotIn('第一堵墙', report)


if __name__ == '__main__':
    unittest.main()
