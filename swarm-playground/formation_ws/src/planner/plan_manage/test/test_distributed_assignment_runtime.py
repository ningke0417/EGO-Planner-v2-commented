#!/usr/bin/env python3
"""Exercise eight real assignment agents with synthetic sensor/planner feedback.

Use ONLY an otherwise disposable localhost:11336 ROS master. Start the eight
formation agents externally, with their normal odom/map remaps, swap_scope=all,
and no real planners or sensors. This script publishes fake observations and
planner acknowledgements; it does not test trajectory feasibility or collisions.
With --allow-fault-injection, withhold one member's odometry to verify that stale
state cannot authorize reassignment. No node is stopped or killed.
"""

import argparse
import json
import math
import os
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from ego_planner.msg import (FormationAgentState, FormationAssignment,
                             FormationTopologyAck, FormationTopologyCommand)


AGENTS = 8
FULL_MASK = (1 << 10) - 1
INITIAL = [(0.5 if drone < 4 else -0.5, drone % 4 - 1.5, 1.5)
           for drone in range(AGENTS)]
GOALS = [(5.5 if slot < 4 else 4.5, slot % 4 - 1.5, 1.5)
         for slot in range(AGENTS)]
SCRAMBLED = [(2.5 if drone < 4 else 1.5, 1.5 - drone % 4, 1.5)
             for drone in range(AGENTS)]


def remaining_distance(positions, mapping):
    return sum(math.sqrt(sum((positions[drone][axis] -
                              GOALS[mapping[drone]][axis]) ** 2
                             for axis in range(3)))
               for drone in range(AGENTS))


class Harness:
    def __init__(self, timeout):
        self.deadline = time.monotonic() + timeout
        self.lock = threading.Lock()
        self.commands = {}
        self.states = {}
        self.transactions = []
        self.positions = list(INITIAL)
        self.missing_odom = set()
        self.last_maps = 0.0
        self.subscribers = [
            rospy.Subscriber('/swarm/agent_state', FormationAgentState,
                             self.state, queue_size=128),
            rospy.Subscriber('/swarm/assignment', FormationAssignment,
                             self.transaction, queue_size=256),
        ]
        self.odom_publishers = []
        self.free_publishers = []
        self.occupied_publishers = []
        self.inflated_publishers = []
        self.ack_publishers = []
        for drone in range(AGENTS):
            self.odom_publishers.append(rospy.Publisher(
                '/drone_%d_visual_slam/odom' % drone, Odometry, queue_size=1))
            self.free_publishers.append(rospy.Publisher(
                '/drone_%d/observed_free' % drone, PointCloud2, queue_size=1))
            self.occupied_publishers.append(rospy.Publisher(
                '/drone_%d/observed_occupied' % drone, PointCloud2, queue_size=1))
            self.inflated_publishers.append(rospy.Publisher(
                '/drone_%d_ego_planner_node/grid_map/formation_occupancy' % drone,
                PointCloud2, queue_size=1))
            self.ack_publishers.append(rospy.Publisher(
                '/drone_%d/formation_applied' % drone, FormationTopologyAck,
                queue_size=1))
            self.subscribers.append(rospy.Subscriber(
                '/drone_%d_formation_agent/topology' % drone,
                FormationTopologyCommand, self.command, callback_args=drone,
                queue_size=1))
        # Independent known-free certification is intentionally still 0.2 m.
        # Occupancy snapshots have the actual planner's empty 0.1 m layout.
        points = [(x * 0.2, y * 0.2, z * 0.2)
                  for x in range(-10, 36)
                  for y in range(-30, 31)
                  for z in range(5, 11)]
        header = Header(frame_id='world')
        self.free_cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.empty_cloud = point_cloud2.create_cloud_xyz32(header, [])

    def state(self, msg):
        with self.lock:
            self.states[msg.drone_id] = (time.monotonic(), msg)

    def command(self, msg, drone):
        with self.lock:
            self.commands[drone] = (time.monotonic(), msg)

    def transaction(self, msg):
        with self.lock:
            self.transactions.append((time.monotonic(), msg))

    def snapshot(self):
        with self.lock:
            return dict(self.commands), dict(self.states), list(self.transactions)

    def pump(self):
        if rospy.is_shutdown():
            raise AssertionError('ROS shut down during assignment regression')
        if time.monotonic() > self.deadline:
            raise AssertionError('Runtime regression exceeded total timeout')
        stamp = rospy.Time.now()
        commands, _, _ = self.snapshot()
        for drone in range(AGENTS):
            if drone not in self.missing_odom:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = 'world'
                odom.pose.pose.orientation.w = 1.0
                (odom.pose.pose.position.x, odom.pose.pose.position.y,
                 odom.pose.pose.position.z) = self.positions[drone]
                self.odom_publishers[drone].publish(odom)
            if drone in commands:
                command = commands[drone][1]
                ack = FormationTopologyAck()
                ack.header.stamp = stamp
                ack.epoch = command.epoch
                ack.drone_id = drone
                ack.mode = command.mode
                ack.components = command.components
                ack.edge_count = command.edge_count
                ack.edge_keep_mask = command.edge_keep_mask
                ack.weak_edge_mask = command.weak_edge_mask
                ack.assignment_versions = command.assignment_versions
                ack.assignment_epoch = command.assignment_epoch
                ack.assignment_digest = command.assignment_digest
                ack.drone_to_slot = command.drone_to_slot
                ack.assigned_goal = GOALS[command.drone_to_slot[drone]]
                ack.goal_valid = True
                ack.formation_scale = command.formation_scale
                ack.progress_sync_scale = command.progress_sync_scale
                self.ack_publishers[drone].publish(ack)
        now = time.monotonic()
        if now - self.last_maps >= 0.2:
            self.free_cloud.header.stamp = stamp
            self.empty_cloud.header.stamp = stamp
            for drone in range(AGENTS):
                self.occupied_publishers[drone].publish(self.empty_cloud)
                self.free_publishers[drone].publish(self.free_cloud)
                self.inflated_publishers[drone].publish(self.empty_cloud)
            self.last_maps = now
        time.sleep(0.05)

    def wait_for(self, description, predicate, timeout=15.0):
        end = min(self.deadline, time.monotonic() + timeout)
        while time.monotonic() < end:
            self.pump()
            if predicate(*self.snapshot()):
                return
        commands, states, transactions = self.snapshot()
        raise AssertionError('%s: masks=%s epochs=%s states=%s phases=%s' % (
            description,
            {i: msg.edge_keep_mask for i, (_, msg) in commands.items()},
            {i: msg.assignment_epoch for i, (_, msg) in commands.items()},
            sorted(states), [msg.phase for _, msg in transactions[-20:]]))

    def hold(self, seconds, invariant=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pump()
            if invariant is not None:
                invariant(*self.snapshot())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--allow-fault-injection', action='store_true')
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args()
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11336', (
        'Synthetic sensor/ACK injection is restricted to localhost:11336')
    assert args.timeout > 0
    rospy.init_node('distributed_assignment_runtime', anonymous=True)
    harness = Harness(args.timeout)
    harness.wait_for('all eight agents must complete the startup barrier',
        lambda commands, states, _: len(commands) == AGENTS and
        len(states) == AGENTS and all(
            msg.reason == 'DISTRIBUTED_ENDPOINT_LEASES' and
            msg.edge_keep_mask == FULL_MASK and msg.assignment_epoch == 1 and
            list(msg.drone_to_slot) == list(range(AGENTS))
            for _, msg in commands.values()), timeout=25.0)
    print(json.dumps({'startup_barrier': 'PASS', 'initial_epoch': 1}), flush=True)

    # An intact formation must first experience a genuine split to arm swaps.
    harness.positions = [(p[0], p[1] * 3.0, p[2]) for p in INITIAL]
    harness.wait_for('split must be seen by every agent',
        lambda commands, _, __: len(commands) == AGENTS and all(
            msg.edge_keep_mask != FULL_MASK for _, msg in commands.values()))

    def no_commit(commands, states, transactions):
        assert all(msg.assignment_epoch == 1 for _, msg in commands.values()), (
            'An incomplete/stale member set changed the assignment epoch')
        assert not any(msg.phase == FormationAssignment.COMMIT
                       for _, msg in transactions), 'Unexpected early COMMIT'

    if args.allow_fault_injection:
        harness.missing_odom.add(7)
        # Exceed the default 1.5 s state lease before introducing a profitable
        # candidate. Otherwise the old valid sample may still authorize it.
        harness.hold(2.2, no_commit)
        harness.positions = list(SCRAMBLED)
        harness.hold(2.5, no_commit)
        print(json.dumps({'stale_member_blocks_commit': 'PASS',
                          'withheld_odometry': 7}), flush=True)
        harness.missing_odom.clear()
    else:
        harness.hold(0.8, no_commit)
        harness.positions = list(SCRAMBLED)

    def converged(commands, states, transactions):
        if len(commands) != AGENTS or len(states) != AGENTS:
            return False
        mappings = {tuple(msg.drone_to_slot) for _, msg in commands.values()}
        epochs = {msg.assignment_epoch for _, msg in commands.values()}
        if len(mappings) != 1 or len(epochs) != 1 or min(epochs) <= 1:
            return False
        mapping = next(iter(mappings))
        epoch = next(iter(epochs))
        return all(msg.assignment_epoch == epoch and
                   tuple(msg.drone_to_slot) == mapping
                   for _, msg in states.values()) and any(
                       msg.phase == FormationAssignment.COMMIT
                       for _, msg in transactions)

    harness.wait_for('unanimous eight-slot assignment must converge',
                     converged, timeout=25.0)
    commands, _, transactions = harness.snapshot()
    first = commands[0][1]
    mapping = list(first.drone_to_slot)
    assert sorted(mapping) == list(range(AGENTS)), 'Duplicated/missing slot'
    assert any(mapping[i] % 4 != i % 4 for i in range(AGENTS)), (
        'The test did not perform cross-column reassignment')
    original_cost = remaining_distance(SCRAMBLED, list(range(AGENTS)))
    candidate_cost = remaining_distance(SCRAMBLED, mapping)
    assert candidate_cost < original_cost - 0.15, (
        'Accepted mapping does not improve remaining straight-line distance')
    commits = [msg for _, msg in transactions
               if msg.phase == FormationAssignment.COMMIT]
    assert all(all(session != 0 for session in msg.ack_sessions) and
               all(sequence != 0 for sequence in msg.ack_sequences)
               for msg in commits), 'COMMIT lacks an all-eight ACK certificate'
    # Hold the same state long enough to observe replay and ensure no immediate
    # flip back to the original mapping. This is not a complete cooldown test.
    accepted_epoch = first.assignment_epoch
    def stable(commands, states, _):
        assert all(msg.assignment_epoch == accepted_epoch and
                   list(msg.drone_to_slot) == mapping
                   for _, msg in commands.values()), 'Assignment oscillated'
    harness.hold(1.5, stable)
    print(json.dumps({'eight_slot_assignment': 'PASS',
                      'assignment_epoch': accepted_epoch,
                      'drone_to_slot': mapping,
                      'original_distance': original_cost,
                      'assigned_distance': candidate_cost,
                      'all_eight_ack_certificate': 'PASS',
                      'test_scope': 'coordination only; synthetic sensor/ACK feedback'}),
          flush=True)

    # Six aircraft have already arrived. They must retain their slots while
    # the two still misplaced aircraft are allowed to exchange lateral goals.
    parked_mapping = list(mapping)
    harness.positions = [GOALS[slot] for slot in parked_mapping]
    harness.positions[0] = (4.0, GOALS[parked_mapping[3]][1], 1.5)
    harness.positions[3] = (4.0, GOALS[parked_mapping[0]][1], 1.5)
    expected = list(parked_mapping)
    expected[0], expected[3] = expected[3], expected[0]

    def parked_converged(commands, states, transactions):
        return len(commands) == AGENTS and len(states) == AGENTS and all(
            msg.assignment_epoch > accepted_epoch and list(msg.drone_to_slot) == expected
            for _, msg in commands.values()) and all(
                msg.assignment_epoch > accepted_epoch and list(msg.drone_to_slot) == expected
                for _, msg in states.values())

    harness.wait_for('parked aircraft must not freeze correction of two misplaced aircraft',
                     parked_converged, timeout=25.0)
    commands, _, _ = harness.snapshot()
    assert all(commands[drone][1].drone_to_slot[drone] == parked_mapping[drone]
               for drone in range(AGENTS) if drone not in (0, 3))
    print(json.dumps({'per_aircraft_goal_lock': 'PASS',
                      'parked_members': [1, 2, 4, 5, 6, 7],
                      'remaining_pair_reassigned': [0, 3],
                      'drone_to_slot': expected}), flush=True)


if __name__ == '__main__':
    main()
