#!/usr/bin/env python3
"""Marker integration test. Run on an empty, disposable ROS master."""
import subprocess
import signal
import time

import rosgraph
import rosnode
import rospy
from ego_planner.msg import FormationTopologyAck
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


def assignment_digest(assignment):
    """Collision-free permutation rank, matching fullAssignmentSignature()."""
    rank = 0
    for i, slot in enumerate(assignment):
        smaller = sum(other < slot for other in assignment[i + 1:])
        rank = rank * (8 - i) + smaller
    return rank + 1


def main():
    rospy.init_node('test_distributed_visualizer', anonymous=True)
    name = '/distributed_formation_visualizer'
    assert not any(n.startswith('/drone_') or n == name
                   for n in rosnode.get_node_names()), 'Use an empty test master'
    rospy.set_param(name + '/state_timeout', 0.5)
    rospy.set_param(name + '/odom_timeout', 0.5)
    states = [rospy.Publisher('/drone_%d/formation_applied' % i,
                             FormationTopologyAck, queue_size=1) for i in range(8)]
    odoms = [rospy.Publisher('/drone_%d_visual_slam/odom' % i,
                            Odometry, queue_size=1) for i in range(8)]
    latest = {}
    def receive(msg):
        for marker in msg.markers:
            latest[(marker.ns, marker.id)] = marker
    subscriber = rospy.Subscriber(name + '/markers', MarkerArray, receive)
    process = subprocess.Popen(['rosrun', 'ego_planner', 'distributed_formation_visualizer'])
    masks = [0] * 8
    weak_masks = [0] * 8
    assignment = list(range(8))
    versions = [0] * 4
    assignment_epoch = 0
    digest = 0
    saved = {}

    def pump(duration=0.8, omit_state=None, omit_odom=None, replay=False,
             invalid=False, state_overrides=None):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            for i in range(8):
                if i != omit_odom:
                    msg = Odometry()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = 'world'
                    msg.pose.pose.position.x = i % 4
                    msg.pose.pose.position.y = i // 4
                    msg.pose.pose.position.z = 1.5
                    odoms[i].publish(msg)
                if i == omit_state:
                    if replay and i in saved:
                        states[i].publish(saved[i])
                    if invalid:
                        msg = FormationTopologyAck()
                        msg.header.stamp = rospy.Time.now()
                        msg.drone_id = 99
                        msg.edge_count = 10
                        msg.edge_keep_mask = 1023
                        msg.mode = 1
                        states[i].publish(msg)
                    continue
                msg = FormationTopologyAck()
                msg.header.stamp = rospy.Time.now()
                msg.drone_id = i
                msg.mode = 1
                msg.edge_count = 10
                msg.edge_keep_mask = masks[i]
                msg.weak_edge_mask = weak_masks[i]
                msg.drone_to_slot = assignment
                msg.assignment_versions = versions
                msg.assignment_epoch = assignment_epoch
                msg.assignment_digest = digest
                for field, value in (state_overrides or {}).get(i, {}).items():
                    setattr(msg, field, value)
                states[i].publish(msg)
                saved[i] = msg
            time.sleep(0.03)

    def edge():
        return latest[('distributed_edges', 0)]

    def check(rgb, message):
        m = edge()
        assert m.action == Marker.ADD, message
        assert all(abs(x-y) < 0.01 for x,y in zip(
            (m.color.r, m.color.g, m.color.b), rgb)), message
        assert 0 < m.lifetime.to_sec() <= 1, 'markers must expire if observer stops'

    try:
        deadline = time.monotonic() + 8
        while not latest and time.monotonic() < deadline:
            assert process.poll() is None, 'visualizer exited'
            time.sleep(0.05)
        assert edge().action == Marker.DELETE, 'no invented endpoint positions'
        pump()
        check((1, 0.10, 0.05), 'both endpoints CUT should be red')
        assert abs(edge().scale.x - 0.045) < 1e-6
        assert len(edge().points) == 8, 'cut edge should contain four dashes'
        assert abs(edge().points[0].z - 1.52) < 1e-6
        assert latest[('distributed_edge_labels', 0)].action == Marker.DELETE
        rospy.set_param(name + '/show_edge_labels', True)
        masks[0] = masks[1] = 1
        # Other members disagree about edge 0. Only its endpoints are authoritative.
        pump()
        check((0.15, 0.95, 0.95), 'both endpoint KEEP should match centralized cyan')
        assert abs(edge().scale.x - 0.085) < 1e-6
        assert len(edge().points) == 2
        weak_masks[0] = weak_masks[1] = 1
        pump()
        check((0.65, 0.35, 1.0), 'weak edge should be purple')
        assert 'WEAK 1' in latest[('distributed_legend', 0)].text
        weak_masks[0] = weak_masks[1] = 0
        pump()
        check((0.15, 0.95, 0.95), 'normal edge should return to cyan')
        masks[1] = 0
        pump()
        check((1, 0.8, 0), 'endpoint mismatch should be yellow')
        masks[0] = 0
        pump()
        rospy.set_param(name + '/show_cut_edges', False)
        pump(0.25)
        assert edge().action == Marker.DELETE, 'hidden CUT line must be explicitly removed'
        assert latest[('distributed_edge_labels', 0)].action == Marker.DELETE
        rospy.set_param(name + '/show_cut_edges', True)
        masks[0] = masks[1] = 1
        pump()
        pump(0.8, omit_state=1, replay=True)
        check((0.65, 0.65, 0.65), 'replayed stale state must not refresh a lease')
        pump()
        check((0.15, 0.95, 0.95), 'fresh reports should recover from stale')
        pump(0.8, omit_state=1, invalid=True)
        check((0.65, 0.65, 0.65), 'wrong endpoint ID must not refresh state')
        pump()
        pump(0.8, omit_odom=1)
        check((0.65, 0.65, 0.65), 'stale odometry should match centralized unknown gray')
        assert 'last pose' in latest[('distributed_edge_labels', 0)].text
        pubs, subs, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        outputs = {topic for topic, nodes in pubs if name in nodes}
        assert outputs <= {'/rosout', name + '/markers'}, outputs
        assert len([topic for topic, nodes in subs if name in nodes]) == 16
        # Logical edge 0-1 is now physically owned by drones 4 and 1.
        assignment[0], assignment[4] = assignment[4], assignment[0]
        versions[0] = 1
        masks[:] = [0] * 8
        masks[4] = masks[1] = 1
        pump()
        check((0.15, 0.95, 0.95), 'swapped owners must supply logical edge evidence')
        assert abs(edge().points[0].x) < 1e-6
        assert abs(edge().points[0].y - 1.0) < 1e-6, 'edge must start at drone 4'

        # A complete permutation can exchange arbitrary columns, not just rows.
        assignment[:] = [3, 2, 1, 0, 7, 6, 5, 4]
        assignment_epoch = 1
        digest = assignment_digest(assignment)
        masks[:] = [0] * 8
        masks[3] = masks[2] = 1
        pump()
        check((0.15, 0.95, 0.95), 'arbitrary full-assignment endpoints should match')
        assert abs(edge().points[0].x - 3.0) < 1e-6, 'edge must start at drone 3'
        assert abs(edge().points[1].x - 2.0) < 1e-6, 'edge must end at drone 2'
        pump(state_overrides={2: {'assignment_digest': digest + 1}})
        check((0.65, 0.65, 0.65), 'invalid digest cannot renew an endpoint lease')
        pump()
        # Valid legacy packets must not roll a full assignment back to epoch 0.
        pump(state_overrides={2: {
            'assignment_epoch': 0, 'assignment_digest': 0,
            'assignment_versions': [0] * 4, 'drone_to_slot': list(range(8))}})
        check((0.65, 0.65, 0.65), 'legacy rollback cannot renew full-assignment state')
        pump()

        previous_assignment = list(assignment)
        previous_digest = digest
        assignment[:] = [1, 0, 2, 3, 5, 4, 6, 7]
        assignment_epoch = 2
        digest = assignment_digest(assignment)
        masks[:] = [0] * 8
        masks[1] = masks[0] = 1
        pump(0.3, state_overrides={0: {
            'assignment_epoch': 1, 'assignment_digest': previous_digest,
            'drone_to_slot': previous_assignment}})
        assert edge().action == Marker.DELETE, 'never draw mixed-assignment endpoints'
        pump()
        check((0.15, 0.95, 0.95), 'consistent newer assignment should recover')
        assert abs(edge().points[0].x - 1.0) < 1e-6, 'edge must now start at drone 1'
        pump(state_overrides={0: {
            'drone_to_slot': list(range(8)), 'assignment_digest': 1}})
        check((0.65, 0.65, 0.65), 'same-epoch conflicting maps cannot refresh state')
        pump()
        pump(state_overrides={0: {'drone_to_slot': [0] * 8}})
        check((0.65, 0.65, 0.65), 'duplicate-slot assignments cannot refresh state')
        pump()
        check((0.15, 0.95, 0.95), 'valid full assignment recovers after rejection')
        print('PASS: missing positions, endpoint-only colors, weak stage, hide/delete, stale/replay, '
              'invalid ID, recovery, stale odometry, marker lifetime, swapped endpoints, '
              'full permutations, bad digests, rollback/conflict rejection, mixed-epoch deletion, '
              'read-only graph', flush=True)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == '__main__':
    main()
