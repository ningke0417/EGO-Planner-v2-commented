#!/usr/bin/env python3
"""Marker integration test. Run on an empty, disposable ROS master."""
import subprocess
import time

import rosgraph
import rosnode
import rospy
from ego_planner.msg import FormationTopologyAck
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


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
    saved = {}

    def pump(duration=0.8, omit_state=None, omit_odom=None, replay=False, invalid=False):
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
        print('PASS: missing positions, endpoint-only colors, weak stage, hide/delete, stale/replay, '
              'invalid ID, recovery, stale odometry, marker lifetime, swapped endpoints, '
              'read-only graph', flush=True)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


if __name__ == '__main__':
    main()
