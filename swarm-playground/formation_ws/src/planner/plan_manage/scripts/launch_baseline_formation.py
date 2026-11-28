#!/usr/bin/env python3
"""Launch rectangular grids with the official planner and shared scene assets.

Rows run front to rear along x, columns run along y, and IDs are row-major,
matching criterion-validation's launch_distributed_formation.py. Generated YAML
and RViz files live only for this launch; no per-count launch files are needed.
The configuration builders can be exercised without a running ROS master.
"""

import copy
import math
from pathlib import Path
import re
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml


COMMON_DEFAULTS = {
    'formation_rows': 2, 'formation_columns': 4,
    'row_spacing': 1.0, 'column_spacing': 1.0,
    'altitude': 1.5, 'rviz': True, 'max_vel': 0.5,
    'map_size_y': 14.0, 'map_size_z': 3.0,
    'odom_topic': 'visual_slam/odom', 'safety_distance': 0.2,
    'formation_weight': 200.0, 'start_planning': True,
    'full_360_sensing': True, 'record_bag': False,
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

MAP_ARGUMENTS = {
    'fixed_wall': (
        'wall_x', 'wall_y', 'wall_size_x', 'wall_size_y', 'second_wall_x',
        'second_wall_size_x', 'second_wall_total_size_y', 'second_wall_gap_size_y',
        'safety_distance', 'map_size_z'),
    'sparse_random': (
        'random_seed', 'small_position_seed', 'large_obstacle_count',
        'small_obstacle_count', 'map_size_y', 'start_x', 'goal_x',
        'endpoint_clearance', 'obstacle_separation', 'goal_side_obstacle_size_y',
        'goal_side_obstacle_center_y', 'large_longitudinal_separation',
        'small_center_y_limit', 'structured_challenge_enabled', 'stacked_cluster_x',
        'narrow_gap_x', 'narrow_gap_size_y', 'map_size_z'),
}
MAP_ARGUMENTS['sparse_random'] += ('small_size_min', 'small_size_max', 'boundary_clearance')
MAP_ARGUMENTS['cluttered_random'] = MAP_ARGUMENTS['sparse_random']

# The reference RViz cycles ten colors. Its first eight live in our template.
RVIZ_EXTRA_COLORS = {8: '226; 0; 158', 9: '170; 85; 0'}


def _boolean(value):
    if value in (True, 'true', '1', 1):
        return True
    if value in (False, 'false', '0', 0):
        return False
    raise ValueError('expected true or false, got %r' % value)


def _integer(value, name):
    if isinstance(value, bool) or int(value) != float(value):
        raise ValueError('%s must be an integer' % name)
    return int(value)


def normalized_params(params):
    """Reject invalid grids and missions before starting any child processes."""
    scene = params.get('scene', 'fixed_wall')
    if scene not in MAP_ARGUMENTS:
        raise ValueError('scene must be fixed_wall, sparse_random or cluttered_random')
    random_scene = scene != 'fixed_wall'
    result = dict(COMMON_DEFAULTS, scene=scene,
                  map_size_x=32.0 if random_scene else 24.0,
                  start_x=-14.0 if random_scene else -6.0,
                  goal_x=14.0 if random_scene else 10.0,
                  bag_prefix='/tmp/ego_rectangle_' + scene)
    if scene == 'cluttered_random':
        # Match criterion-validation's distributed_cluttered_random.launch.
        result.update(map_size_x=40.0, map_size_y=18.0, start_x=-17.0, goal_x=17.0,
                      endpoint_clearance=5.0, obstacle_separation=0.6,
                      large_obstacle_count=0, small_obstacle_count=60,
                      small_center_y_limit=8.0, small_size_max=1.1,
                      structured_challenge_enabled=False)
    unknown = set(params) - set(result)
    if unknown:
        raise ValueError('unknown parameters: %s' % sorted(unknown))
    result.update(params)
    integers = ('formation_rows', 'formation_columns', 'random_seed',
                'small_position_seed', 'large_obstacle_count', 'small_obstacle_count')
    booleans = ('rviz', 'start_planning', 'full_360_sensing', 'record_bag',
                'structured_challenge_enabled')
    strings = ('scene', 'odom_topic', 'bag_prefix')
    for name in integers:
        result[name] = _integer(result[name], name)
    rows, columns = result['formation_rows'], result['formation_columns']
    if rows < 1 or columns < 1 or not 2 <= rows * columns <= 16:
        raise ValueError('formation_rows * formation_columns must be between 2 and 16')
    if min(result['large_obstacle_count'], result['small_obstacle_count']) < 0:
        raise ValueError('obstacle counts must be nonnegative')
    for name in booleans:
        result[name] = _boolean(result[name])
    for name in set(result) - set(integers + booleans + strings):
        result[name] = float(result[name])
        if not math.isfinite(result[name]):
            raise ValueError('%s must be finite' % name)
    for name in ('row_spacing', 'column_spacing', 'altitude', 'max_vel',
                 'map_size_x', 'map_size_y', 'map_size_z', 'wall_size_x',
                 'wall_size_y', 'second_wall_size_x', 'second_wall_total_size_y',
                 'second_wall_gap_size_y', 'narrow_gap_size_y',
                 'goal_side_obstacle_size_y', 'large_longitudinal_separation',
                 'small_center_y_limit', 'small_size_min', 'small_size_max'):
        if result[name] <= 0:
            raise ValueError('%s must be positive' % name)
    for name in ('safety_distance', 'formation_weight', 'endpoint_clearance',
                 'obstacle_separation', 'boundary_clearance'):
        if result[name] < 0:
            raise ValueError('%s must be nonnegative' % name)
    if result['small_size_min'] > result['small_size_max']:
        raise ValueError('small_size_min must not exceed small_size_max')
    if result['start_x'] >= result['goal_x']:
        raise ValueError('start_x must be less than goal_x')
    if result['second_wall_gap_size_y'] >= result['second_wall_total_size_y']:
        raise ValueError('second wall gap must be smaller than total wall width')
    half_x = (rows - 1) * result['row_spacing'] / 2.0
    half_y = (columns - 1) * result['column_spacing'] / 2.0
    if (max(abs(result['start_x']), abs(result['goal_x'])) + half_x >= result['map_size_x'] / 2.0
            or half_y >= result['map_size_y'] / 2.0
            or result['altitude'] >= result['map_size_z']):
        raise ValueError('initial and goal formations must fit inside the map')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', result['odom_topic']):
        raise ValueError('odom_topic must be a relative ROS topic suffix')
    if not result['bag_prefix'] or any(c.isspace() for c in result['bag_prefix']):
        raise ValueError('bag_prefix must be a nonempty path without whitespace')
    return result


def formation_offsets(params):
    p = normalized_params(params)
    return [[((p['formation_rows'] - 1) / 2.0 - row) * p['row_spacing'],
             (column - (p['formation_columns'] - 1) / 2.0) * p['column_spacing'], 0.0]
            for row in range(p['formation_rows'])
            for column in range(p['formation_columns'])]


def formation_config(params):
    offsets = formation_offsets(params)
    return {'formation': dict(num=len(offsets),
                              **{'drone%d' % i: offset for i, offset in enumerate(offsets)})}


def _text(value):
    return ('true' if value else 'false') if isinstance(value, bool) else str(value)


def _include(parent, filename, arguments):
    node = ET.SubElement(parent, 'include', file='$(find ego_planner)/launch/' + filename)
    for name, value in arguments.items():
        ET.SubElement(node, 'arg', name=name, value=_text(value))
    return node


def build_launch_xml(params, formation_path, rviz_path):
    p = normalized_params(params)
    offsets = formation_offsets(p)
    launch = ET.Element('launch')
    bridge = ET.SubElement(launch, 'include', file='$(find swarm_bridge)/launch/bridge_udp.launch')
    ET.SubElement(bridge, 'arg', name='drone_id', value='999')
    ET.SubElement(bridge, 'arg', name='broadcast_ip', value='127.0.0.255')
    map_file = ('baseline_fixed_wall_map.launch' if p['scene'] == 'fixed_wall'
                else 'baseline_sparse_random_map.launch')
    _include(launch, map_file, {name: p[name] for name in MAP_ARGUMENTS[p['scene']]})
    shared = {name: p[name] for name in (
        'start_x', 'goal_x', 'max_vel', 'safety_distance', 'formation_weight',
        'start_planning', 'full_360_sensing', 'map_size_x', 'map_size_y',
        'map_size_z', 'odom_topic')}
    for drone, offset in enumerate(offsets):
        _include(launch, 'baseline_8uav_member.launch', dict(
            shared, drone_id=drone, init_x=p['start_x'] + offset[0],
            init_y=offset[1], init_z=p['altitude'], formation_config=str(formation_path)))
    if p['record_bag']:
        topics = ['/map_generator/global_cloud', '/map_generator/wall_geometry',
                  '/map_generator/sparse_random_geometry', '/rosout_agg']
        for drone in range(len(offsets)):
            topics.extend(['/drone_%d_%s' % (drone, p['odom_topic']),
                           '/drone_%d_planning/trajectory' % drone,
                           '/drone_%d_planning/pos_cmd' % drone])
        # ROS splits node args with shlex; quote the user-supplied path as data.
        import shlex
        ET.SubElement(launch, 'node', pkg='rosbag', type='record', name='rectangle_record',
                      output='screen', args='-O ' + shlex.quote(p['bag_prefix'] + '.bag')
                      + ' ' + ' '.join(topics))
    if p['rviz']:
        import shlex
        ET.SubElement(launch, 'node', pkg='rviz', type='rviz', name='rviz',
                      required='true', args='-d ' + shlex.quote(str(rviz_path)))
    return ET.tostring(launch, encoding='unicode')


def build_rviz_config(params, template):
    """Expand the established eight-UAV view, including all robot markers."""
    p = normalized_params(params)
    config = copy.deepcopy(template)
    manager = config['Visualization Manager']
    displays = manager['Displays']
    prototypes = [d for d in displays if re.fullmatch(r'drone\d+', d.get('Name', ''))]
    if not prototypes:
        raise ValueError('RViz template has no drone group')

    def remap(value, drone):
        if isinstance(value, dict):
            result = {key: remap(item, drone) for key, item in value.items()}
            color = RVIZ_EXTRA_COLORS.get(drone % 10)
            if color and value.get('Name') in ('drone_path', 'map inflate'):
                result['Color'] = color
            return result
        if isinstance(value, list):
            return [remap(item, drone) for item in value]
        if isinstance(value, str):
            value = re.sub(r'/drone_\d+_', '/drone_%d_' % drone, value)
            return value.replace('/ego_planner_node/', '/drone_%d_ego_planner_node/' % drone)
        return value

    groups = []
    for drone in range(p['formation_rows'] * p['formation_columns']):
        style = drone % 10
        prototype = prototypes[style] if style < len(prototypes) else prototypes[0]
        group = remap(copy.deepcopy(prototype), drone)
        group['Name'] = 'drone%d' % drone
        groups.append(group)
    # Preserve ordering and all scene displays, replacing the original groups once.
    manager['Displays'] = []
    inserted = False
    for display in displays:
        if re.fullmatch(r'drone\d+', display.get('Name', '')):
            if not inserted:
                manager['Displays'].extend(groups)
                inserted = True
        else:
            manager['Displays'].append(display)
    view = manager['Views']['Current']
    view['Focal Point']['X'] = (p['start_x'] + p['goal_x']) / 2.0
    view['Distance'] = max(view['Distance'], 1.15 * p['map_size_x'])
    return config


def main():
    import roslaunch
    import rospkg
    import rospy

    rospy.init_node('baseline_formation_launcher')
    defaults = normalized_params({'scene': rospy.get_param('~scene', 'fixed_wall')})
    params = normalized_params({name: rospy.get_param('~' + name, value)
                                for name, value in defaults.items()})
    package = Path(rospkg.RosPack().get_path('ego_planner'))
    template = yaml.safe_load((package / 'launch/baseline_8uav.rviz').read_text())
    # Keep shared resources alive until every child has stopped, then clean up.
    with tempfile.TemporaryDirectory(prefix='ego-rectangle-') as directory:
        formation_path = Path(directory) / 'formation.yaml'
        rviz_path = Path(directory) / 'formation.rviz'
        formation_path.write_text(yaml.safe_dump(formation_config(params)))
        rviz_path.write_text(yaml.safe_dump(build_rviz_config(params, template), allow_unicode=True))
        xml = build_launch_xml(params, formation_path, rviz_path)
        rospy.loginfo('Starting official baseline %sx%s (%s UAVs), scene=%s',
                      params['formation_rows'], params['formation_columns'],
                      params['formation_rows'] * params['formation_columns'], params['scene'])
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
        print('Unable to launch baseline formation: %s' % error, file=sys.stderr)
        sys.exit(1)
