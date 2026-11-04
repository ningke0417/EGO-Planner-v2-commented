#!/usr/bin/env python3
"""Read-only independent coverage/content checks for a completed ROS1 batch."""
import argparse
import json
from pathlib import Path
from analyze_paper_formation_error import recording_bag_path


def validate(batch):
    import rosbag
    import yaml
    config = json.loads((batch/'config.json').read_text())
    checks = []
    safety_diagnostics = []
    for run in sorted(batch.glob('trial_*')):
        if not (run/'result.json').exists():
            continue
        result = json.loads((run/'result.json').read_text())
        errors = []
        first, last, finish = {}, {}, None
        topics = {}
        try:
            bag_path = recording_bag_path(run, result)
            with rosbag.Bag(str(bag_path)) as bag:
                topics = bag.get_type_and_topic_info().topics
                forbidden = [name for name, info in topics.items() if info.msg_type in (
                    'sensor_msgs/PointCloud2', 'sensor_msgs/PointCloud', 'sensor_msgs/Image',
                    'sensor_msgs/CompressedImage', 'nav_msgs/OccupancyGrid', 'visualization_msgs/MarkerArray')]
                if forbidden:
                    errors.append('forbidden topics: '+str(forbidden))
                odometry = {'/drone_%d_visual_slam/odom' % i: i for i in range(8)}
                safety_topics = [name for name in topics if name.endswith('/first_wall_violation')]
                for topic, msg, stamp in bag.read_messages(topics=list(odometry)+['/experiment/events']+safety_topics):
                    if topic in odometry:
                        i = odometry[topic]
                        first.setdefault(i, msg.header.stamp.to_sec())
                        last[i] = msg
                    elif topic == '/experiment/events':
                        event = json.loads(msg.data)
                        if event.get('event') == 'finish':
                            finish = event
                    else:
                        safety_diagnostics.append(dict(trial=run.name, topic=topic,
                                                       bag_ros_time=stamp.to_sec(), message=msg.data))
                if len(first) != 8:
                    errors.append('missing odometry')
                start = result.get('departure_ros_time')
                end = result.get('all_arrived_ros_time') or result.get('recording_end_ros_time')
                if start is not None and any(t > start for t in first.values()):
                    errors.append('odometry starts after departure')
                if end is not None and any(m.header.stamp.to_sec() < end for m in last.values()):
                    errors.append('odometry ends before analysis interval')
                if result['success']:
                    import math
                    for i, msg in last.items():
                        p = msg.pose.pose.position
                        g = result['final_goals'][str(i)]
                        if math.sqrt(sum((x-y)**2 for x, y in zip((p.x, p.y, p.z), g))) > config['arrival_radius']:
                            errors.append('final goal not reached: '+str(i))
                if not finish or any(finish.get(k) != result.get(k) for k in ('success', 'reason', 'completion_time_s')):
                    errors.append('bag finish event differs from result')
                if '/experiment/formation_error' not in topics:
                    errors.append('online formation metric missing')
            params = yaml.safe_load((run/'params.yaml').read_text())
            if config['scene'] == 'sparse_random':
                if params['sparse_random_obstacle_map']['seed'] != config['random_seed']:
                    errors.append('random seed mismatch')
                for name in ('small_obstacle_count', 'small_position_seed'):
                    if name in config and params['sparse_random_obstacle_map'].get(name) != config[name]:
                        errors.append('scene parameter mismatch: '+name)
                geometry = json.loads((run/'obstacle_geometry.json').read_text())
                if not geometry['boxes_xyz_min_max']:
                    errors.append('missing obstacle geometry')
            if start is not None:
                audit = json.loads((batch/'formation_error_paper'/(run.name+'_audit.json')).read_text())
                if abs(audit['e_dist_eq45']-result['formation_error_eq45']) > 1e-12:
                    errors.append('offline formation metric mismatch')
                if not audit['per_agent_applied_assignment_verified']:
                    errors.append('applied assignment not verified')
            if Path(str(bag_path)+'.active').exists():
                errors.append('bag not finalized')
        except Exception as exc:
            errors.append(str(exc))
        check = dict(trial=run.name, passed=not errors, errors=errors, topics=len(topics),
                     messages=sum(t.message_count for t in topics.values()))
        checks.append(check)
        print(run.name, 'PASS' if check['passed'] else errors, flush=True)
    report = dict(validated_trials=len(checks), expected_trials=config['runs'],
                  complete=len(checks) == config['runs'],
                  passed=bool(checks) and all(x['passed'] for x in checks), trials=checks)
    (batch/'planner_safety_diagnostics.json').write_text(
        json.dumps(safety_diagnostics, indent=2, ensure_ascii=False)+'\n')
    (batch/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch', type=Path)
    args = parser.parse_args()
    report = validate(args.batch)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)
