#!/usr/bin/env python3
"""Read the real final map; emulate only GNSS GetParameters, never external actuators."""
import json
import sys

import rclpy
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path as PathMsg

from control_manager.msg import GlobalPose
from smoke_test import ROOT, Stack, wait_for
from operation_smoke_test import command
sys.path.insert(0, str(ROOT / 'scripts'))
from global_path_map import load_route, project_route


def main():
    rclpy.init()
    node = rclpy.create_node('global_path_fixture')
    filename = ROOT.parents[1] / 'decision/script/map_generator/test/review/maps/MAP_2026-10-15_195100_test.json'
    origin = json.loads(filename.read_text())['coordinate_system']['origin']
    route = load_route(filename)
    state = {'lat': float('nan'), 'lon': float('nan'), 'frame': 'map', 'calls': 0}
    def parameters(request, response):
        assert request.names == ['datum_lat', 'datum_lon', 'map_frame', 'base_frame']
        state['calls'] += 1
        response.values = [
            ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=state['lat']),
            ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=state['lon']),
            ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=state['frame']),
            ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='base_link')]
        return response
    service = node.create_service(GetParameters, '/gnss_node/get_parameters', parameters)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    paths, references, late = [], [], []
    node.create_subscription(PathMsg, '/control/global_path', paths.append, durable)
    node.create_subscription(PathMsg, '/control/reference_path', references.append, 10)
    pub = node.create_publisher(GlobalPose, '/control/global_pose', 10)
    def pose():
        msg = GlobalPose(valid=True)
        msg.header.frame_id = 'map'; msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
    node.create_timer(.05, pose)
    for label, overrides in [('lane', ('guidance_mode:=LANE', 'map_file:=not_loaded.json')),
                             ('external', ('map_file:=""',))]:
        stack = Stack(command('global_path_provider', *overrides), 'map_inactive_' + label)
        try:
            wait_for(node, lambda: ('global_path_provider', '/control') in
                     node.get_node_names_and_namespaces(), 'inactive file provider: ' + label)
            assert not node.get_publishers_info_by_topic('/control/global_path')
            assert state['calls'] == 0
        finally:
            stack.close()
            wait_for(node, lambda: ('global_path_provider', '/control') not in
                     node.get_node_names_and_namespaces(), 'inactive provider stopped')
    stacks = [Stack(command('global_path_provider'), 'map_provider'),
              Stack(command('path_planning_node'), 'map_path_planning')]
    try:
        wait_for(node, lambda: state['calls'] >= 2 and bool(references), 'GNSS datum awaited')
        assert not paths and not references[-1].poses
        state.update(lat=origin[0], lon=origin[1])
        wait_for(node, lambda: bool(paths) and len(paths[-1].poses) == len(route), 'real final map published')
        expected = project_route(route, *origin)
        for i in (0, 1200, len(route) - 1):
            position = paths[-1].poses[i].pose.position
            assert abs(position.x - expected[i][0]) < 1e-9
            assert abs(position.y - expected[i][1]) < 1e-9
        wait_for(node, lambda: references[-1].poses and
                 abs(references[-1].poses[0].pose.position.x - expected[0][0]) < 1e-9,
                 'Path Planning forwards aligned real route')
        count, calls = len(paths), state['calls']
        wait_for(node, lambda: state['calls'] >= calls + 2, 'GNSS origin monitored')
        assert len(paths) == count  # Static path is not rebuilt/re-published at planner rate.
        subscription = node.create_subscription(PathMsg, '/control/global_path', late.append, durable)
        wait_for(node, lambda: bool(late), 'late subscriber receives latched route')
        assert len(late[-1].poses) == len(route)
        state['lat'] += .0001
        shifted = project_route(route, state['lat'], state['lon'])
        wait_for(node, lambda: paths[-1].poses and
                 abs(paths[-1].poses[0].pose.position.y - shifted[0][1]) < 1e-9,
                 'changed GNSS origin reprojects the map')
        state['frame'] = 'wrong_map'
        wait_for(node, lambda: not paths[-1].poses and not references[-1].poses,
                 'frame mismatch withdraws route and stops reference')
        state['frame'] = 'map'
        wait_for(node, lambda: paths[-1].poses and references[-1].poses, 'matching frame recovers')
        state['lat'] = float('nan')
        wait_for(node, lambda: not paths[-1].poses and not references[-1].poses,
                 'uninitialized datum withdraws previous route')
        state['lat'] = origin[0]
        wait_for(node, lambda: paths[-1].poses and references[-1].poses, 'datum recovery')
        node.destroy_service(service); service = None
        wait_for(node, lambda: not paths[-1].poses and not references[-1].poses,
                 'lost GNSS parameter service withdraws route')
        node.destroy_subscription(subscription)
    finally:
        for stack in stacks:
            stack.close()
        node.destroy_node()
        rclpy.shutdown()
    print('ALL GLOBAL PATH TESTS PASSED', flush=True)


if __name__ == '__main__':
    main()
