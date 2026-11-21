#!/usr/bin/env python3
"""Launch a configured rectangular formation using one shared node implementation.

The XML is kept in memory: changing rows or columns never creates launch files.
build_launch_xml is deliberately independent of ROS for configuration regression
checks. The executable starts no central formation controller.
"""

import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


COMMON_DEFAULTS = {
    'formation_rows': 2, 'formation_columns': 4,
    'row_spacing': 1.0, 'column_spacing': 1.0,
    'rviz': True, 'swap_scope': 'all', 'max_vel': 0.5,
    'map_size_y': 14.0, 'map_size_z': 3.0, 'altitude': 1.5,
    'odom_topic': 'visual_slam/odom', 'safety_distance': 0.2,
    'formation_weight': 200.0, 'weak_reconnect_distance': 4.0,
    'distributed_startup_timeout': 60.0,
    'wall_x': -2.0, 'wall_y': 0.0, 'wall_size_x': 0.5, 'wall_size_y': 6.0,
    'second_wall_x': 6.5, 'second_wall_size_x': 0.5,
    'second_wall_total_size_y': 10.0, 'second_wall_gap_size_y': 1.8,
    'random_seed': 17, 'small_position_seed': -1,
    'large_obstacle_count': 4, 'small_obstacle_count': 6,
    'endpoint_clearance': 3.0, 'obstacle_separation': 0.7,
    'small_size_min': 0.45, 'small_size_max': 0.9, 'boundary_clearance': 0.8,
    'goal_side_obstacle_size_y': 4.0, 'goal_side_obstacle_center_y': 0.0,
    'large_longitudinal_separation': 3.5, 'small_center_y_limit': 2.0,
    'structured_challenge_enabled': True, 'stacked_cluster_x': -2.0,
    'narrow_gap_x': 4.8, 'narrow_gap_size_y': 1.6,
}


def _boolean(value):
    if isinstance(value, bool):
        return value
    if value in ('true', '1', 1):
        return True
    if value in ('false', '0', 0):
        return False
    raise ValueError('expected true or false, got %r' % value)


def normalized_params(params):
    """Validate geometry before any ROS nodes or processes are started."""
    scene = params.get('scene', 'fixed_wall')
    if scene not in ('fixed_wall', 'sparse_random', 'cluttered_random'):
        raise ValueError('scene must be fixed_wall, sparse_random or cluttered_random')
    result = dict(COMMON_DEFAULTS)
    random_scene = scene != 'fixed_wall'
    result.update(scene=scene, swap_enabled=random_scene,
                  map_size_x=32.0 if random_scene else 24.0,
                  start_x=-14.0 if random_scene else -6.0,
                  goal_x=14.0 if random_scene else 10.0)
    if scene == 'cluttered_random':
        # A wide field of independent posts, with no long walls or forced gap.
        # Reuse the same seeded box sampler and all distributed member nodes.
        result.update(map_size_x=40.0, map_size_y=18.0, start_x=-17.0, goal_x=17.0,
                      endpoint_clearance=5.0, obstacle_separation=0.6,
                      large_obstacle_count=0, small_obstacle_count=60,
                      small_center_y_limit=8.0, small_size_max=1.1,
                      structured_challenge_enabled=False)
    unknown = set(params) - set(result)
    if unknown:
        raise ValueError('unknown formation parameters: %s' % sorted(unknown))
    result.update(params)
    for name in ('formation_rows', 'formation_columns'):
        value = result[name]
        if isinstance(value, bool) or int(value) != float(value) or int(value) < 1:
            raise ValueError('%s must be a positive integer' % name)
        result[name] = int(value)
    rows, columns = result['formation_rows'], result['formation_columns']
    count = rows * columns
    edges = rows * (columns - 1) + (rows - 1) * columns
    if count < 2 or count > 16 or edges >= 32:
        raise ValueError('supported formation: 2..16 members and fewer than 32 grid edges')
    for name in ('rviz', 'swap_enabled', 'structured_challenge_enabled'):
        result[name] = _boolean(result[name])
    if result['swap_scope'] not in ('all', 'column'):
        raise ValueError('swap_scope must be all or column')
    if result['swap_scope'] == 'column' and rows != 2:
        raise ValueError('column swaps require exactly two rows; use swap_scope:=all')
    for name in ('row_spacing', 'column_spacing', 'max_vel', 'map_size_x',
                 'map_size_y', 'map_size_z', 'altitude', 'safety_distance',
                 'formation_weight', 'weak_reconnect_distance', 'distributed_startup_timeout'):
        result[name] = float(result[name])
        if not math.isfinite(result[name]) or result[name] <= 0:
            raise ValueError('%s must be finite and positive' % name)
    for name in ('start_x', 'goal_x'):
        result[name] = float(result[name])
        if not math.isfinite(result[name]):
            raise ValueError('%s must be finite' % name)
    for name in ('large_obstacle_count', 'small_obstacle_count'):
        value = result[name]
        if isinstance(value, bool) or int(value) != float(value) or int(value) < 0:
            raise ValueError('%s must be a nonnegative integer' % name)
        result[name] = int(value)
    for name in ('small_size_min', 'small_size_max', 'small_center_y_limit'):
        result[name] = float(result[name])
        if not math.isfinite(result[name]) or result[name] <= 0:
            raise ValueError('%s must be finite and positive' % name)
    for name in ('endpoint_clearance', 'obstacle_separation', 'boundary_clearance'):
        result[name] = float(result[name])
        if not math.isfinite(result[name]) or result[name] < 0:
            raise ValueError('%s must be finite and nonnegative' % name)
    if result['small_size_min'] > result['small_size_max']:
        raise ValueError('small_size_min must not exceed small_size_max')
    if result['start_x'] >= result['goal_x']:
        raise ValueError('start_x must be less than goal_x')
    if ((rows > 1 and result['row_spacing'] > 1.7) or
            (columns > 1 and result['column_spacing'] > 1.7)):
        raise ValueError('initial adjacent spacing must be at most the 1.7 m reconnect distance')
    if result['weak_reconnect_distance'] < 1.7:
        raise ValueError('weak_reconnect_distance must be at least 1.7 m')
    half_x = (rows - 1) * result['row_spacing'] / 2.0
    half_y = (columns - 1) * result['column_spacing'] / 2.0
    if (max(abs(result['start_x']), abs(result['goal_x'])) + half_x >= result['map_size_x'] / 2.0
            or half_y >= result['map_size_y'] / 2.0
            or result['altitude'] >= result['map_size_z']):
        raise ValueError('initial/goal formation must fit inside the configured map')
    return result


def formation_offsets(params):
    p = normalized_params(params)
    return [
        [(0.5 * (p['formation_rows'] - 1) - row) * p['row_spacing'],
         (column - 0.5 * (p['formation_columns'] - 1)) * p['column_spacing'], 0.0]
        for row in range(p['formation_rows'])
        for column in range(p['formation_columns'])
    ]


def _text(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _param(parent, name, value):
    kind = 'bool' if isinstance(value, bool) else 'int' if isinstance(value, int) else 'double' if isinstance(value, float) else 'str'
    ET.SubElement(parent, 'param', name=name, value=_text(value), type=kind)


def _node(parent, package, executable, name, **attributes):
    return ET.SubElement(parent, 'node', pkg=package, type=executable,
                         name=name, output='log', **attributes)


def build_launch_xml(params, package_dir=None):
    """Return the complete child launch XML, with no disk or ROS side effects."""
    p = normalized_params(params)
    offsets = formation_offsets(p)
    count = len(offsets)
    if package_dir is None:
        package_dir = Path(__file__).resolve().parents[1]
    package_dir = Path(package_dir)
    launch = ET.Element('launch')
    _param(launch, '/distributed_swap_enabled', p['swap_enabled'])
    if p['scene'] == 'fixed_wall':
        node = _node(launch, 'map_generator', 'fixed_split_merge_wall_map',
                     'fixed_split_merge_wall_map', required='true')
        map_params = {
            'first_wall_x': p['wall_x'], 'first_wall_y': p['wall_y'],
            'first_wall_size_x': p['wall_size_x'], 'first_wall_size_y': p['wall_size_y'],
            'second_wall_x': p['second_wall_x'], 'second_wall_y': p['wall_y'],
            'second_wall_size_x': p['second_wall_size_x'],
            'second_wall_total_size_y': p['second_wall_total_size_y'],
            'second_wall_gap_size_y': p['second_wall_gap_size_y'],
            'safety_distance': p['safety_distance'], 'resolution': 0.1,
            'height': p['map_size_z'], 'randomize': False, 'random_seed': 1,
        }
    else:
        node = _node(launch, 'map_generator', 'sparse_random_obstacle_map',
                     'sparse_random_obstacle_map', required='true')
        map_params = {name: p[name] for name in (
            'small_position_seed', 'large_obstacle_count', 'small_obstacle_count',
            'map_size_y', 'start_x', 'goal_x', 'endpoint_clearance', 'obstacle_separation',
            'large_longitudinal_separation', 'small_center_y_limit',
            'structured_challenge_enabled', 'stacked_cluster_x', 'narrow_gap_x',
            'narrow_gap_size_y')}
        map_params.update(seed=p['random_seed'], boundary_clearance=p['boundary_clearance'],
                          stacked_cluster_y=-0.3, stacked_block_size=1.2,
                          stacked_block_step=0.75, narrow_gap_wall_size_x=0.8,
                          narrow_gap_segment_size_y=4.6, large_size_x_min=0.6,
                          large_size_x_max=1.2, large_size_y_min=2.5,
                          large_size_y_max=4.5,
                          goal_side_large_size_y_max=p['goal_side_obstacle_size_y'],
                          center_goal_side_large=True,
                          goal_side_large_center_y=p['goal_side_obstacle_center_y'],
                          small_size_min=p['small_size_min'], small_size_max=p['small_size_max'],
                          resolution=0.1, height=p['map_size_z'])
    ET.SubElement(node, 'remap', {'from': 'global_cloud', 'to': '/map_generator/global_cloud'})
    for name, value in map_params.items():
        _param(node, name, value)

    shape_params = {name: p[name] for name in (
        'formation_rows', 'formation_columns', 'row_spacing', 'column_spacing')}
    for drone, offset in enumerate(offsets):
        include = ET.SubElement(launch, 'include', file=str(package_dir / 'launch/run_in_sim.launch'))
        arguments = dict(distributed_enabled=True, drone_id=drone,
                         init_x=p['start_x'] + offset[0], init_y=offset[1],
                         init_z=p['altitude'], map_size_x=p['map_size_x'],
                         map_size_y=p['map_size_y'], map_size_z=p['map_size_z'],
                         odom_topic=p['odom_topic'], max_vel=p['max_vel'],
                         obstacles_inflation=p['safety_distance'],
                         obstacle_clearance=p['safety_distance'],
                         use_wall_collision_guard=True,
                         wall_safety_distance=p['safety_distance'],
                         general_dynamic_topology_enabled=True,
                         manual_formation_split=False, split_side_guidance_enabled=False,
                         split_subgroup_formation_weight_scale=0.15,
                         split_progress_sync_weight_scale=0.0,
                         merge_sync_enabled=True, merge_sync_duration=20.0,
                         merge_sync_backtrack_tolerance=0.03, merge_sync_weight=1000.0,
                         initial_assignment_enabled=False,
                         subgroup_route_intent_feedback_enabled=False,
                         formation_weight=p['formation_weight'], formation_num=count,
                         formation_start_x=p['start_x'], formation_start_y=0.0,
                         formation_start_z=p['altitude'], point_num=1,
                         point0_x=p['goal_x'], point0_y=0.0, point0_z=p['altitude'])
        for name, value in arguments.items():
            ET.SubElement(include, 'arg', name=name, value=_text(value))
        observer = _node(launch, 'ego_planner', 'swarm_observed_map_node',
                         'drone_%d_observed_map' % drone)
        observation_params = dict(num_uavs=count, local_drone_id=drone,
                                  odom_topic_suffix=p['odom_topic'], resolution=0.2,
                                  horizontal_fov=2.0 * math.pi, azimuth_step=0.04,
                                  elevation_step=0.05, map_size_x=p['map_size_x'],
                                  map_size_y=p['map_size_y'], map_size_z=p['map_size_z'])
        for name, value in observation_params.items():
            _param(observer, name, value)
        for source, destination in (
                ('/swarm/observed_occupied', '/drone_%d/observed_occupied' % drone),
                ('/swarm/observed_free', '/drone_%d/observed_free' % drone)):
            ET.SubElement(observer, 'remap', {'from': source, 'to': destination})
        agent = _node(launch, 'ego_planner', 'distributed_formation_agent',
                      'drone_%d_formation_agent' % drone)
        agent_params = dict(shape_params, drone_id=drone,
                            weak_reconnect_distance=p['weak_reconnect_distance'],
                            weak_reconnect_hold=0.3, weak_formation_scale=0.22,
                            swap_enabled=p['swap_enabled'], swap_scope=p['swap_scope'],
                            formation_forward_x=1.0, formation_forward_y=0.0)
        for name, value in agent_params.items():
            _param(agent, name, value)
        for source, destination in (
                ('~odom', '/drone_%d_%s' % (drone, p['odom_topic'])),
                ('~occupied', '/drone_%d/observed_occupied' % drone),
                ('~free', '/drone_%d/observed_free' % drone),
                ('~inflated', '/drone_%d_ego_planner_node/grid_map/formation_occupancy' % drone)):
            ET.SubElement(agent, 'remap', {'from': source, 'to': destination})

    # These absolute overrides are deliberately after every include. This also
    # supplies drone8..drone15 without adding per-count args to legacy templates.
    for drone in range(count):
        prefix = '/drone_%d_ego_planner_node/formation/' % drone
        for key, value in (('rows', p['formation_rows']), ('columns', p['formation_columns']),
                           ('row_spacing', p['row_spacing']), ('column_spacing', p['column_spacing']),
                           ('distributed_startup_timeout', p['distributed_startup_timeout'])):
            _param(launch, prefix + key, value)
        for slot, offset in enumerate(offsets):
            parameter = ET.SubElement(launch, 'rosparam', param=prefix + 'drone%d' % slot)
            parameter.text = json.dumps(offset)
        _param(launch, '/drone_%d_pcl_render_node/full_360_sensing' % drone, True)

    visualizer = _node(launch, 'ego_planner', 'distributed_formation_visualizer',
                       'distributed_formation_visualizer')
    for name, value in dict(shape_params, odom_topic_suffix=p['odom_topic']).items():
        _param(visualizer, name, value)
    if p['rviz']:
        ET.SubElement(launch, 'include', file=str(package_dir / 'launch/rviz.launch'))
    return ET.tostring(launch, encoding='unicode')


def main():
    import roslaunch
    import rospkg
    import rospy

    rospy.init_node('distributed_formation_launcher')
    scene = rospy.get_param('~scene', 'fixed_wall')
    defaults = normalized_params({'scene': scene})
    params = {key: rospy.get_param('~' + key, value) for key, value in defaults.items()}
    xml = build_launch_xml(params, package_dir=rospkg.RosPack().get_path('ego_planner'))
    count = int(params['formation_rows']) * int(params['formation_columns'])
    rospy.loginfo('Starting distributed %sx%s formation (%s UAVs), scene=%s',
                  params['formation_rows'], params['formation_columns'], count, scene)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    parent = roslaunch.parent.ROSLaunchParent(uuid, [], roslaunch_strs=[xml])
    rospy.on_shutdown(parent.shutdown)
    try:
        parent.start()
        parent.spin()
    finally:
        parent.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('Unable to launch distributed formation: %s' % error, file=sys.stderr)
        sys.exit(1)
