#!/usr/bin/env python3
"""Hardware-free activation, dependency and disabled-input contracts.
Use the same installed package and isolated environment as smoke_test.py.
"""
import copy
import itertools
import subprocess

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import UInt8
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from control_manager.msg import MotionState, MotionPlan, ObjectState, MissionLimit
from smoke_test import ROOT, ENV, Stack, wait_for

OVERRIDES = itertools.count()


def command(executable, *params):
    args = [str(get_package_prefix('control_manager') + '/lib/control_manager/' + executable),
            '--ros-args', '-r', '__ns:=/control']
    for filename in ('vehicle.yaml', 'pipeline.yaml', 'operation.yaml'):
        args += ['--params-file', str(ROOT / 'config' / filename)]
    name = executable[:-5] if executable.endswith('_node') else executable
    if params:
        # Exact-node YAML rules outrank wildcard CLI rules; use an exact-node overlay.
        values = {key: yaml.safe_load(value) for key, value in
                  (param.split(':=', 1) for param in params)}
        overlay = ROOT / 'log' / ('operation_override_' + str(next(OVERRIDES)) + '.yaml')
        overlay.write_text(yaml.safe_dump({'/control/' + name: {'ros__parameters': values}}))
        args += ['--params-file', str(overlay)]
    return args


def owns(node, topic, name, publisher=False):
    infos = (node.get_publishers_info_by_topic(topic) if publisher
             else node.get_subscriptions_info_by_topic(topic))
    return any(i.node_name == name and i.node_namespace == '/control' for i in infos)


def activation_graph():
    node = rclpy.create_node('operation_graph_fixture')
    # Wired but disabled inputs must be allowed without creating subscriptions.
    wired = ('topics.external_lane:=/test/lane',
             'topics.external_motor_feedback:=/test/motor',
             'topics.external_steering_feedback:=/test/steering')
    pairs = [('/gnss/odom', '/control/sensors/gnss_odom'),
             ('/imu/data', '/control/sensors/imu'), ('/camera/image_raw', '/control/sensors/rgb'),
             ('/lidar/points', '/control/sensors/lidar'),
             ('/camera_dep/depth/image_raw', '/control/sensors/depth'),
             ('/test/lane', '/control/lane_observation')]
    for label, extra, expected in [
            ('gnss_only', (), {0}),
            ('all_supported', ('imu_enabled:=true', 'rgb_enabled:=true', 'lidar_enabled:=true',
                               'depth_enabled:=true', 'lane_enabled:=true',
                               'vehicle_feedback_enabled:=true'), set(range(6))),
            ('all_off', ('gnss_enabled:=false',), set())]:
        stack = Stack(command('sensor_interface_node', *wired, *extra), 'operation_' + label)
        try:
            wait_for(node, lambda: ('sensor_interface', '/control') in node.get_node_names_and_namespaces(), label)
            def matches():
                return all(owns(node, source, 'sensor_interface') == (i in expected) and
                           owns(node, output, 'sensor_interface', True) == (i in expected)
                           for i, (source, output) in enumerate(pairs))
            wait_for(node, matches, 'subscriptions and relay publishers: ' + label)
            assert not owns(node, '/test/motor', 'sensor_interface')
            assert not owns(node, '/test/steering', 'sensor_interface')
            assert owns(node, '/control/vehicle_feedback', 'sensor_interface', True) == (label == 'all_supported')
        finally:
            stack.close()
            wait_for(node, lambda: ('sensor_interface', '/control') not in node.get_node_names_and_namespaces(), 'ingress stopped')
    stacks = [Stack(command(executable), 'operation_disabled_' + name) for executable, name in [
        ('lane_state_estimator', 'lane_state_estimator'), ('object_manager_node', 'object_manager'),
        ('mission_based_limit_node', 'mission_based_limit')]]
    try:
        wait_for(node, lambda: all((name, '/control') in node.get_node_names_and_namespaces()
                 for name in ('lane_state_estimator', 'object_manager', 'mission_based_limit')), 'disabled nodes remain launchable')
        for name, topic in [('lane_state_estimator', 'lane_pose'), ('object_manager', 'object_state'),
                            ('mission_based_limit', 'mission_limit')]:
            assert not owns(node, '/control/' + topic, name, True)
        assert not owns(node, '/control/lane_observation', 'lane_state_estimator')
        for topic in ('rgb', 'lidar', 'depth'):
            assert not owns(node, '/control/sensors/' + topic, 'object_manager')
        assert not owns(node, '/control/object_state', 'mission_based_limit')
    finally:
        for stack in stacks: stack.close()
        node.destroy_node()
    print('PASS: activation graph, wired-but-disabled inputs, inactive Lane/Object/Mission', flush=True)


def rejection_tests():
    cases = [
        ('global_state_estimator', ('gnss_enabled:=false',), 'requires gnss_enabled'),
        ('global_state_estimator', ('motion_mode:=GNSS_IMU',), 'requires imu_enabled'),
        ('global_state_estimator', ('motion_mode:=WHEEL',), 'requires vehicle_feedback_enabled'),
        ('sensor_interface_node', ('lane_enabled:=true',), 'requires a nonempty'),
        ('sensor_interface_node', ('motor_feedback_enabled:=true',), 'encoder adapter not implemented'),
        ('sensor_interface_node', ('steering_feedback_enabled:=true',), 'encoder adapter not implemented'),
        ('path_planning_node', ('guidance_mode:=LANE',), 'requires lane_enabled'),
        ('path_planning_node', ('guidance_mode:=HYBRID',), 'requires lane_enabled'),
        ('path_planning_node', ('guidance_mode:=NONE',), 'guidance_mode must be'),
        ('path_planning_node', ('guidance_mode:=INVALID',), 'guidance_mode must be'),
        ('global_path_provider', ('guidance_mode:=INVALID',), 'guidance_mode must be'),
        ('global_path_provider', ('map_file:=missing_map_file.json',), 'No such file'),
        ('object_manager_node', ('object_state_enabled:=true',), 'requires at least one'),
        ('mission_based_limit_node', ('mission_limit_enabled:=true', 'traffic_light_applies:=true'),
         'requires object_state_enabled')]
    for executable, params, reason in cases:
        result = subprocess.run(command(executable, *params), env=ENV, capture_output=True, text=True, timeout=8)
        assert result.returncode != 0 and reason in result.stdout + result.stderr, (executable, params, result.stderr)
    print('PASS: contradictory configuration and unimplemented encoder activation rejected', flush=True)


def planning_dependencies():
    for objects, mission in ((False, False), (True, False), (False, True), (True, True)):
        node = rclpy.create_node('operation_plan_fixture')
        values = []
        node.create_subscription(MotionPlan, '/control/motion_plan', values.append, 10)
        pubs = {key: node.create_publisher(cls, '/control/' + key, 10) for key, cls in [
            ('motion_state', MotionState), ('reference_path', PathMsg),
            ('object_state', ObjectState), ('mission_limit', MissionLimit)]}
        state = {'objects': not objects, 'mission': not mission, 'obstacle': False, 'mission_valid': True}
        def publish():
            stamp = node.get_clock().now().to_msg()
            motion = MotionState(speed_mps=.7, speed_valid=True)
            motion.header.stamp = stamp; motion.header.frame_id = 'base_link'
            path = PathMsg(); path.header.stamp = stamp; path.header.frame_id = 'map'
            for x in (0., 20.):
                p = PoseStamped(); p.header = copy.deepcopy(path.header); p.pose.position.x = x; p.pose.orientation.w = 1.
                path.poses.append(p)
            pubs['motion_state'].publish(motion); pubs['reference_path'].publish(path)
            if state['objects']:
                obj = ObjectState(valid=objects, obstacle_valid=objects, obstacle_detected=state['obstacle'])
                obj.header.stamp = stamp; obj.header.frame_id = 'base_link'; pubs['object_state'].publish(obj)
            if state['mission']:
                limit = MissionLimit(valid=mission and state['mission_valid'], max_speed_mps=.4)
                limit.header.stamp = stamp; limit.header.frame_id = 'map'; pubs['mission_limit'].publish(limit)
        timer = node.create_timer(.05, publish)
        stack = Stack(command('motion_planning_node', 'cruise_speed_mps:=1.0',
                      'object_state_enabled:=' + str(objects).lower(),
                      'mission_limit_enabled:=' + str(mission).lower()), 'operation_plan_' + str(objects) + str(mission))
        try:
            wait_for(node, lambda: bool(values), 'MotionPlan received')
            if objects or mission:
                assert not values[-1].valid and values[-1].stop_request
            state.update(objects=True, mission=True)
            speed = .4 if mission else 1.
            wait_for(node, lambda: values[-1].valid and values[-1].target_speed_mps == speed, 'active inputs only')
            assert owns(node, '/control/object_state', 'motion_planning') == objects
            assert owns(node, '/control/mission_limit', 'motion_planning') == mission
            assert (':OBJECT_DISABLED' in values[-1].reason) == (not objects)
            assert (':MISSION_DISABLED' in values[-1].reason) == (not mission)
            if objects:
                state['obstacle'] = True
                wait_for(node, lambda: values[-1].valid and values[-1].stop_request and values[-1].target_speed_mps == 0., 'active obstacle stops')
                state.update(obstacle=False, objects=False)
                wait_for(node, lambda: not values[-1].valid and values[-1].stop_request, 'active object timeout closes plan')
                state['objects'] = True
                wait_for(node, lambda: values[-1].valid and values[-1].target_speed_mps == speed, 'object recovery')
            if mission:
                state['mission_valid'] = False
                wait_for(node, lambda: not values[-1].valid and values[-1].stop_request, 'active invalid mission closes plan')
        finally:
            stack.close(); node.destroy_timer(timer); node.destroy_node()
    print('PASS: all Object/Mission dependency combinations, ignored invalid inactive data, active stop/timeout', flush=True)


def guidance_startup_modes():
    for mode, expected in [('GPS', {'global_pose', 'global_path'}),
                           ('LANE', {'lane_pose'}),
                           ('HYBRID', {'global_pose', 'global_path', 'lane_pose'})]:
        node = rclpy.create_node('operation_guidance_fixture'); values = []
        node.create_subscription(PathMsg, '/control/reference_path', values.append, 10)
        legacy_request = node.create_publisher(UInt8, '/control/guidance_mode_request', 10)
        # Even with Lane enabled globally, GPS must subscribe only to global inputs.
        stack = Stack(command('path_planning_node', 'guidance_mode:=' + mode,
                              'lane_enabled:=true'), 'operation_guidance_' + mode)
        client = node.create_client(SetParameters, '/control/path_planning/set_parameters')
        try:
            wait_for(node, lambda: bool(values), 'ReferencePath received: ' + mode)
            wait_for(node, lambda: all(owns(node, '/control/' + topic, 'path_planning') ==
                     (topic in expected) for topic in ('global_pose', 'global_path', 'lane_pose')),
                     'only required Guidance inputs subscribed: ' + mode)
            assert not owns(node, '/control/guidance_mode_request', 'path_planning')
            assert not owns(node, '/control/guidance_status', 'path_planning', True)
            assert client.wait_for_service(timeout_sec=3)
            request = SetParameters.Request(parameters=[
                Parameter('guidance_mode', value='GPS' if mode != 'GPS' else 'LANE').to_parameter_msg()])
            future = client.call_async(request)
            wait_for(node, future.done, 'runtime parameter change answered: ' + mode)
            assert not future.result().results[0].successful
            for _ in range(8):
                legacy_request.publish(UInt8(data=255))
                rclpy.spin_once(node, timeout_sec=.05)
            assert not values[-1].poses  # Missing inputs or unsupported algorithm.
            assert all(owns(node, '/control/' + topic, 'path_planning') == (topic in expected)
                       for topic in ('global_pose', 'global_path', 'lane_pose'))
        finally:
            stack.close(); node.destroy_client(client); node.destroy_node()
    print('PASS: GPS/LANE/HYBRID startup modes, read-only selection and no request-topic subscription', flush=True)


if __name__ == '__main__':
    rclpy.init()
    try:
        rejection_tests(); activation_graph(); planning_dependencies(); guidance_startup_modes()
    finally:
        rclpy.shutdown()
