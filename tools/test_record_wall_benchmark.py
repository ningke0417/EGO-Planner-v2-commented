#!/usr/bin/env python3
"""Check mission classification and timing without a ROS installation."""
import unittest

from record_wall_benchmark import Monitor, box_distance


class MissionMetricsTest(unittest.TestCase):
    def monitor(self):
        return Monitor(dict(initial_positions=[[0, i, 1.5] for i in range(8)],
                            goals=[[10, i, 1.5] for i in range(8)], walls=[],
                            arrival_m=0.3, arrival_speed_mps=0.15, settle_s=1.0,
                            departure_m=0.05, freshness_s=0.5, vehicle_radius_m=0.1))

    def states(self, monitor, now, x, speed=0):
        for i in range(8):
            monitor.update(i, now, [x, i, 1.5], [speed, 0, 0])

    def setUp(self):
        self.m = self.monitor()
        self.m.config['walls'] = [dict(center=[-20, 0, 1.5], size=[0.5, 8, 3.1])]
        self.m.trigger = 100

    def test_actual_departure_and_settled_arrival_timing(self):
        self.states(self.m, 101, 0)
        self.assertFalse(self.m.evaluate(101))
        self.states(self.m, 102, 0.06)
        self.states(self.m, 150, 10)
        self.assertFalse(self.m.evaluate(150))
        self.states(self.m, 151.1, 10)
        self.assertTrue(self.m.evaluate(151.1))
        result = self.m.result()
        self.assertTrue(result['success'])
        self.assertEqual(result['completion_time_s'], 48)
        self.assertEqual(result['trigger_to_arrival_s'], 50)

    def test_one_drone_missing_or_stale_never_succeeds(self):
        self.states(self.m, 110, 10)
        self.assertFalse(self.m.evaluate(110))
        self.assertFalse(self.m.evaluate(112))
        self.assertIsNone(self.m.all_inside_since)
        for i in range(7):
            self.m.update(i, 113, [10, i, 1.5], [0, 0, 0])
        self.assertFalse(self.m.evaluate(113))
        self.assertIsNone(self.m.result()['completion_time_s'])

    def test_overshoot_or_fast_pass_resets_settling(self):
        self.states(self.m, 110, 10)
        self.m.evaluate(110)
        self.states(self.m, 110.7, 10, speed=0.5)
        self.assertFalse(self.m.evaluate(110.7))
        self.assertIsNone(self.m.all_inside_since)
        self.states(self.m, 111, 10)
        self.assertFalse(self.m.evaluate(111))
        self.states(self.m, 112.1, 10)
        self.assertTrue(self.m.evaluate(112.1))
        self.assertEqual(self.m.arrival, 111)

    def test_collision_is_retained_even_after_arrival(self):
        self.m.config['walls'] = [dict(center=[5, 0, 1.5], size=[0.5, 8, 3.1])]
        self.states(self.m, 110, 5)
        self.m.evaluate(110)
        self.states(self.m, 150, 10)
        self.m.evaluate(150)
        self.states(self.m, 151.1, 10)
        self.assertTrue(self.m.evaluate(151.1))
        self.assertTrue(self.m.result()['arrival_success'])
        self.assertFalse(self.m.result()['success'])
        self.assertTrue(self.m.result()['sampled_wall_collision'])

    def test_pair_collision_and_nonfinite(self):
        self.states(self.m, 110, 5)
        self.m.update(1, 110, [5, 0.1, 1.5], [0, 0, 0])
        self.m.evaluate(110)
        self.assertTrue(self.m.result()['sampled_pair_collision'])
        self.m.update(0, 111, [float('nan'), 0, 1.5], [0, 0, 0])
        self.assertTrue(self.m.invalid)

    def test_signed_wall_distance(self):
        wall = dict(center=[0, 0, 0], size=[2, 2, 2])
        self.assertEqual(box_distance([0, 0, 0], wall), -1)
        self.assertEqual(box_distance([2, 0, 0], wall), 1)
        self.assertAlmostEqual(box_distance([2, 2, 0], wall), 2 ** 0.5)

    def test_random_obstacle_collision_is_not_mislabeled_as_wall(self):
        self.m.config['scene'] = 'random'
        self.m.config['obstacles'] = [dict(center=[5, 0, 1.5], size=[0.8, 1, 3.1])]
        self.states(self.m, 110, 5)
        self.m.evaluate(110)
        result = self.m.result()
        self.assertTrue(result['sampled_obstacle_collision'])
        self.assertFalse(result['sampled_wall_collision'])
        self.assertLess(result['min_obstacle_surface_clearance_m'], 0)
        self.assertIsNone(result['min_wall_surface_clearance_m'])

    def test_random_unfinished_flight_has_no_success_duration(self):
        self.m.config['scene'] = 'random'
        self.m.config['obstacles'] = self.m.config['walls']
        self.states(self.m, 110, 5)
        self.m.evaluate(110)
        result = self.m.result()
        self.assertIsNotNone(result['departure_stamp_ros_s'])
        self.assertFalse(result['arrival_success'])
        self.assertFalse(result['success'])
        self.assertIsNone(result['completion_time_s'])


if __name__ == '__main__':
    unittest.main()
