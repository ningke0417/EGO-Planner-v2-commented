#!/usr/bin/env python3
"""Run only against a disposable ROS master/launch, not an existing experiment.

Checks graph isolation, all eight planners, and loss of one formation agent.
The --allow-fault-injection flag is required because the test stops agent 0.
"""
import argparse
import json
import time
import rosgraph
import rosnode
import rospy
from ego_planner.msg import FormationTopologyAck, FormationTopologyCommand
from traj_utils.msg import MINCOTraj


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--allow-fault-injection', action='store_true')
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args()
    rospy.init_node('distributed_smoke', anonymous=True)
    applied, commands, trajectories = {}, {}, set()
    subscribers = []
    def applied_callback(msg, agent):
        applied[agent] = (time.monotonic(), msg)
    def command_callback(msg, agent):
        commands[agent] = (time.monotonic(), msg)
    for agent in range(8):
        subscribers.append(rospy.Subscriber(
            '/drone_%d/formation_applied' % agent, FormationTopologyAck,
            applied_callback, callback_args=agent))
        subscribers.append(rospy.Subscriber(
            '/drone_%d_formation_agent/topology' % agent, FormationTopologyCommand,
            command_callback, callback_args=agent))
    subscribers.append(rospy.Subscriber('/swarm/trajectories', MINCOTraj,
                                       lambda msg: trajectories.add(msg.drone_id)))
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and (
            len(applied) != 8 or len(commands) != 8 or len(trajectories) != 8 or
            not any(value[1].edge_keep_mask for value in applied.values()) or
            (args.allow_fault_injection and
             (0 not in applied or not applied[0][1].edge_keep_mask & 65))):
        time.sleep(0.1)
    assert len(applied) == 8, 'Missing applied topology: %s' % sorted(applied)
    assert len(commands) == 8, 'Missing local agents: %s' % sorted(commands)
    assert trajectories == set(range(8)), 'Missing planners: %s' % sorted(trajectories)
    assert any(value[1].edge_keep_mask for value in applied.values()), 'No edge ever connected'
    if args.allow_fault_injection:
        assert applied[0][1].edge_keep_mask & 65, 'Agent 0 must have an active incident edge before injection'
    master = rosgraph.Master(rospy.get_name())
    pubs, subs, _ = master.getSystemState()
    publishers, subscribers_by_topic = dict(pubs), dict(subs)
    assert '/formation_backend/topology_command' not in publishers
    assert '/swarm/observed_free' not in publishers
    assert len(publishers['/swarm/trajectories']) == 8
    assert len(publishers['/swarm/edge_evidence']) == 8
    for agent in range(8):
        node = '/drone_%d_observed_map' % agent
        inputs = [topic for topic, nodes in subs if node in nodes]
        assert '/drone_%d_pcl_render_node/cloud' % agent in inputs
        assert not any('/drone_%d_' % peer in topic for peer in range(8)
                       if peer != agent for topic in inputs), inputs
        planner = '/drone_%d_ego_planner_node' % agent
        assert planner not in subscribers_by_topic.get('/formation_backend/topology_command', [])
        assert planner not in subscribers_by_topic.get('/formation_backend/initial_assignment', [])
    print(json.dumps({'graph': 'PASS', 'planners': sorted(trajectories),
                      'masks': {i: value[1].edge_keep_mask for i, value in applied.items()}}),
          flush=True)
    if not args.allow_fault_injection:
        return
    # Kill only the agent in the isolated test graph; local EGO must remain alive.
    success, failed = rosnode.kill_nodes(['/drone_0_formation_agent'])
    assert not failed, failed
    time.sleep(0.5)
    trajectories.clear()
    stopped = time.monotonic()
    deadline = stopped + 8
    while time.monotonic() < deadline:
        own_cleared = applied[0][0] > stopped and applied[0][1].edge_keep_mask == 0
        peers_cleared = all(
            commands[i][0] > stopped and commands[i][1].edge_keep_mask & 65 == 0
            for i in range(1, 8))
        if own_cleared and peers_cleared and trajectories == set(range(8)):
            print(json.dumps({'agent_loss': 'PASS', 'planners_still_publishing':
                              sorted(trajectories)}), flush=True)
            return
        time.sleep(0.1)
    raise AssertionError('agent-loss watchdog/peer expiry/planner continuity failed')


if __name__ == '__main__':
    main()
