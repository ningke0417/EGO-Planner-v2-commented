#!/usr/bin/env python3
"""Boot a real distributed grid on its own disposable localhost ROS master.

Checks startup, topology application, local-map isolation and trajectory output.
Optionally requires stable full connectivity after every member reaches its goal.
Neither mode constitutes a collision-safety test.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

from distributed_test_config import formation_digest


def terminal_snapshot(states, applied, count, edges, now):
    """Evaluate only fresh measured state and fresh planner-applied topology."""
    full_mask = (1 << edges) - 1
    arrived, connected, details = set(), set(), []
    for drone in range(count):
        state, ack = states.get(drone), applied.get(drone)
        detail = dict(drone=drone)
        if state is not None:
            age = now - state.header.stamp.to_sec()
            distance = math.sqrt(sum((p - g) ** 2
                                     for p, g in zip(state.position, state.assigned_goal)))
            speed = math.sqrt(sum(v ** 2 for v in state.velocity))
            detail.update(goal_valid=state.goal_valid, goal_error=round(distance, 3),
                          speed=round(speed, 3), state_age=round(age, 3))
            if (state.goal_valid and state.header.stamp.to_sec() > 0 and
                    -0.05 <= age <= 1.5 and distance <= 0.30 and speed <= 0.15):
                arrived.add(drone)
        else:
            detail['missing_state'] = True
        if ack is not None:
            age = now - ack.header.stamp.to_sec()
            detail.update(mask=ack.edge_keep_mask, weak_mask=ack.weak_edge_mask,
                          ack_age=round(age, 3))
            if (ack.header.stamp.to_sec() > 0 and -0.05 <= age <= 1.5 and
                    ack.edge_count == edges and ack.edge_keep_mask == full_mask and
                    ack.weak_edge_mask == 0):
                connected.add(drone)
        else:
            detail['missing_ack'] = True
        details.append(detail)
    return arrived, connected, details


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rows', type=int, required=True)
    parser.add_argument('--columns', type=int, required=True)
    parser.add_argument('--scene', choices=['fixed_wall', 'sparse_random', 'cluttered_random'], required=True)
    parser.add_argument('--port', type=int, default=11561)
    parser.add_argument('--timeout', type=float, default=100)
    parser.add_argument('--check-goal-connectivity', action='store_true',
                        help='Also require every member arrived and full topology stable before the total timeout')
    parser.add_argument('--goal-hold', type=float, default=10.0,
                        help='Required continuous seconds of full connectivity at the goal (default: 10)')
    args = parser.parse_args()
    if not math.isfinite(args.goal_hold) or args.goal_hold <= 0:
        parser.error('--goal-hold must be finite and positive')
    assert 11400 <= args.port <= 65535, 'Use a dedicated high-numbered test port'
    count = args.rows * args.columns
    assert 2 <= count <= 16
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', args.port))
    output = Path(tempfile.mkdtemp(prefix='ego-grid-runtime-', dir='/tmp'))
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port,
                      ROS_IP='127.0.0.1', ROS_HOME=str(output / 'ros'),
                      ROS_LOG_DIR=str(output / 'logs'))
    environment = os.environ.copy()
    command = ['roslaunch', '-p', str(args.port), 'ego_planner',
               'distributed_%s.launch' % args.scene, 'rviz:=false',
               'formation_rows:=%d' % args.rows,
               'formation_columns:=%d' % args.columns]
    log = (output / 'launch.log').open('w')
    process = subprocess.Popen(command, env=environment, stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    try:
        import rosgraph
        import rospy
        from ego_planner.msg import FormationAgentState, FormationTopologyAck, FormationEdgeEvidence
        from sensor_msgs.msg import PointCloud2
        from traj_utils.msg import MINCOTraj
        master = rosgraph.Master('/grid_runtime_check')
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, 'launch process exited'
            try:
                master.getPid()
                break
            except Exception:
                time.sleep(0.1)
        rospy.init_node('grid_runtime_check', anonymous=True, disable_signals=True)
        states, applied, trajectories, ready, evidence = {}, {}, set(), set(), {}
        sensor = dict(frame=None, stamp=0.0, empty_seen=False)
        expected_digest = formation_digest(args.rows, args.columns)
        edges = args.rows * (args.columns - 1) + (args.rows - 1) * args.columns

        def on_state(message):
            if message.formation_digest == expected_digest:
                states[message.drone_id] = message
                if message.startup_ready:
                    ready.add(message.drone_id)

        def on_applied(message):
            if message.formation_digest == expected_digest:
                applied[message.drone_id] = message

        def on_scan(message):
            sensor['frame'] = message.header.frame_id
            sensor['stamp'] = message.header.stamp.to_sec()
            sensor['empty_seen'] |= message.width * message.height == 0

        subscribers = [rospy.Subscriber('/drone_%d_pcl_render_node/cloud' % (count - 1),
                                        PointCloud2, on_scan, queue_size=1),
                       rospy.Subscriber('/swarm/agent_state', FormationAgentState,
                                        on_state, queue_size=256),
                       rospy.Subscriber('/swarm/edge_evidence', FormationEdgeEvidence,
                                        lambda msg: evidence.__setitem__((msg.edge_index, msg.observer_id), msg),
                                        queue_size=256),
                       rospy.Subscriber('/swarm/trajectories', MINCOTraj,
                                        lambda msg: trajectories.add(msg.drone_id), queue_size=256)]
        for drone in range(count):
            subscribers.append(rospy.Subscriber('/drone_%d/formation_applied' % drone,
                                                FormationTopologyAck, on_applied, queue_size=10))
        next_progress = 0
        expected_ids = set(range(count))
        while time.monotonic() < deadline:
            assert process.poll() is None, 'launch process exited'
            if set(applied) == expected_ids and ready == expected_ids and trajectories == expected_ids:
                break
            if time.monotonic() >= next_progress:
                print(json.dumps(dict(rows=args.rows, columns=args.columns, scene=args.scene,
                                      states=len(states), applied=len(applied),
                                      ready=len(ready), trajectories=len(trajectories),
                                      endpoint_evidence=len(evidence))), flush=True)
                next_progress = time.monotonic() + 15
            time.sleep(0.1)
        assert set(states) == expected_ids, ('missing states', sorted(states))
        assert set(applied) == expected_ids, ('missing applied topology', sorted(applied))
        assert ready == expected_ids, ('startup barrier', sorted(ready))
        assert trajectories == expected_ids, ('missing trajectory output', sorted(trajectories))
        assert sensor['frame'] == 'world' and sensor['stamp'] > 0, sensor
        for drone, message in applied.items():
            assert message.edge_count == edges, (drone, message.edge_count)
            assert len(message.assignment_versions) == args.columns
            assert sorted(message.drone_to_slot) == list(range(count))
            assert message.edge_keep_mask < (1 << edges)
            assert message.weak_edge_mask & ~message.edge_keep_mask == 0
        goal_connectivity = False
        if args.check_goal_connectivity:
            hold_since = None
            next_progress = 0
            _, _, terminal_details = terminal_snapshot(
                dict(states), dict(applied), count, edges, rospy.Time.now().to_sec())
            while time.monotonic() < deadline:
                assert process.poll() is None, 'launch process exited before stable goal connectivity'
                arrived, connected, terminal_details = terminal_snapshot(
                    dict(states), dict(applied), count, edges, rospy.Time.now().to_sec())
                now = time.monotonic()
                if arrived == expected_ids and connected == expected_ids:
                    if hold_since is None:
                        hold_since = now
                    if now - hold_since >= args.goal_hold:
                        goal_connectivity = True
                        break
                else:
                    hold_since = None
                if now >= next_progress:
                    print(json.dumps(dict(rows=args.rows, columns=args.columns, scene=args.scene,
                                          stage='goal_connectivity', arrived=len(arrived),
                                          fully_connected=len(connected),
                                          held_seconds=round(now - hold_since, 2) if hold_since is not None else 0)),
                          flush=True)
                    next_progress = now + 15
                time.sleep(0.1)
            assert goal_connectivity, ('no stable full connectivity at the goal', terminal_details)
        publishers, subscriptions, _ = master.getSystemState()
        pubs, subs = dict(publishers), dict(subscriptions)
        assert '/formation_backend/topology_command' not in pubs
        assert '/swarm/observed_free' not in pubs
        assert len(pubs['/swarm/agent_state']) == count
        assert len(pubs['/swarm/trajectories']) == count
        for drone in range(count):
            prefix = '/drone_%d' % drone
            planner = prefix + '_ego_planner_node'
            agent = prefix + '_formation_agent'
            occupancy = planner + '/grid_map/formation_occupancy'
            assert planner in pubs.get(occupancy, [])
            assert agent in subs.get(occupancy, [])
            for peer in range(count):
                if peer != drone:
                    assert '/drone_%d_formation_agent' % peer not in subs.get(occupancy, [])
            observed_inputs = [topic for topic, nodes in subscriptions
                               if prefix + '_observed_map' in nodes]
            assert prefix + '_pcl_render_node/cloud' in observed_inputs
            assert not any('/drone_%d_' % peer in topic for topic in observed_inputs
                           for peer in range(count) if peer != drone)
        print(json.dumps(dict(result='PASS', rows=args.rows, columns=args.columns,
                              agents=count, edges=edges, scene=args.scene,
                              local_map_isolation=True, startup_barrier=True,
                              goal_connectivity=goal_connectivity,
                              sensor_frame=sensor['frame'], empty_scan_seen=sensor['empty_seen'],
                              trajectories=sorted(trajectories), logs=str(output))), flush=True)
    except Exception:
        log.flush()
        if 'states' in locals():
            _, _, terminal_details = terminal_snapshot(
                dict(states), dict(applied), count, edges, rospy.Time.now().to_sec())
            print('Latest goal errors and applied masks:', json.dumps(terminal_details), flush=True)
        if 'evidence' in locals():
            now = rospy.Time.now().to_sec()
            print('Endpoint evidence ages:', [(key, round(now - msg.header.stamp.to_sec(), 3),
                                              int(msg.decision)) for key, msg in sorted(evidence.items())], flush=True)
        print('\n'.join((output / 'launch.log').read_text(errors='replace').splitlines()[-60:]), flush=True)
        raise
    finally:
        try:
            import rospy
            if rospy.core.is_initialized():
                rospy.signal_shutdown('isolated grid test complete')
        except Exception:
            pass
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=10)
        log.close()


if __name__ == '__main__':
    main()
