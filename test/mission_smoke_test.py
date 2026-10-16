#!/usr/bin/env python3
"""Hardware-free MissionLimit -> MotionPlan tests; all generated files stay local."""
import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from control_manager.msg import GlobalPose, MotionState, MotionPlan, MissionLimit, MissionMap
from smoke_test import Stack, wait_for
from operation_smoke_test import command, owns


def fixture(sections):
    node = rclpy.create_node('mission_fixture')
    limits, plans = [], []
    node.create_subscription(MissionLimit, '/control/mission_limit', limits.append, 10)
    node.create_subscription(MotionPlan, '/control/motion_plan', plans.append, 10)
    pubs = {key: node.create_publisher(cls, '/control/' + key, 10) for key, cls in (
        ('global_pose', GlobalPose), ('motion_state', MotionState), ('reference_path', PathMsg))}
    map_pub = node.create_publisher(MissionMap, '/control/mission_map', QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    state = {'x': 5., 'y': 0., 'variance': 0., 'pose': True, 'stamp': 'fresh'}
    path = PathMsg(); path.header.frame_id = 'map'
    for x in (0., 10., 20., 30.):
        p = PoseStamped(); p.header.frame_id = 'map'; p.pose.position.x = x; p.pose.orientation.w = 1.
        path.poses.append(p)
    def publish():
        stamp = node.get_clock().now().to_msg()
        reference = copy.deepcopy(path); reference.header.stamp = stamp
        pubs['reference_path'].publish(reference)
        motion = MotionState(speed_valid=True, speed_mps=.8)
        motion.header.frame_id = 'base_link'; motion.header.stamp = stamp
        pubs['motion_state'].publish(motion)
        if state['pose']:
            pose = GlobalPose(valid=True, x_m=state['x'], y_m=state['y'],
                              position_variance_x_m2=state['variance'])
            pose.header.frame_id = 'map'; pose.header.stamp = copy.deepcopy(stamp)
            if state['stamp'] == 'stale': pose.header.stamp.sec -= 2
            if state['stamp'] == 'malformed': pose.header.stamp.nanosec = 1_000_000_000
            pubs['global_pose'].publish(pose)
    node.create_timer(.05, publish)
    # Keep algorithm fixture speed independent of operator edits to mission.yaml.
    mission_args = ['mission_limit_enabled:=true', 'cruise_speed_kph:=10.0',
                    'mission_sections_enabled:=' + str(sections).lower()]
    if sections:
        mission_args += ['sections.straight_1.max_speed_kph:=3.6',
                         'sections.straight_1.margin_before_m:=3.0',
                         'sections.straight_1.margin_after_m:=4.0',
                         'sections.straight_1.stop_required:=true']
    mission = Stack(command('mission_based_limit_node', *mission_args), 'mission_sections_' + str(sections))
    planning = Stack(command('motion_planning_node', 'mission_limit_enabled:=true',
                             'cruise_speed_kph:=10.0'), 'mission_motion_' + str(sections))
    def map_message():
        msg = MissionMap(); msg.header.frame_id = 'map'; msg.header.stamp = node.get_clock().now().to_msg()
        msg.path = copy.deepcopy(path)
        msg.stations_m = [0., 10., 20., 30.]
        msg.section_names = ['직진1']; msg.section_start_s_m = [10.]; msg.section_end_s_m = [20.]
        return msg
    def speed(value, stopped=False):
        return bool(limits and plans and limits[-1].valid and plans[-1].valid and
                    plans[-1].stop_request == stopped and
                    math.isclose(plans[-1].target_speed_mps, value, abs_tol=1e-9))
    def invalid(reason):
        return bool(limits and plans and not limits[-1].valid and limits[-1].stop_required and
                    reason in limits[-1].reason and not plans[-1].valid and plans[-1].stop_request)
    try:
        if not sections:
            wait_for(node, lambda: speed(10. / 3.6), 'constant 10 km/h target without mission map/pose dependency')
            assert not owns(node, '/control/global_pose', 'mission_based_limit')
            assert not owns(node, '/control/mission_map', 'mission_based_limit')
        else:
            wait_for(node, lambda: invalid('MISSION_MAP_UNAVAILABLE'), 'section mode requires map')
            map_pub.publish(map_message())
            wait_for(node, lambda: speed(10. / 3.6), 'outside configured section uses default speed')
            state['x'] = 8.
            wait_for(node, lambda: speed(1.) and not limits[-1].stop_required, 'before margin slows without early stop')
            state['x'] = 12.
            wait_for(node, lambda: speed(0., True) and limits[-1].stop_required, 'original section requires stop')
            state['x'] = 22.
            wait_for(node, lambda: speed(1.), 'after margin retains speed cap')
            state['x'] = 25.
            wait_for(node, lambda: speed(10. / 3.6), 'outside margin restores constant target')
            state.update(x=9.5, variance=.25)
            wait_for(node, lambda: speed(0., True), 'uncertain stop boundary uses conservative stop')
            state.update(x=5., variance=0., y=2.)
            wait_for(node, lambda: invalid('MISSION_OFF_PATH'), 'off-path location invalidates mission')
            state.update(y=0., stamp='stale')
            wait_for(node, lambda: invalid('MISSION_POSE_UNAVAILABLE'), 'stale pose cannot refresh section limit')
            state['stamp'] = 'malformed'
            wait_for(node, lambda: invalid('MISSION_POSE_UNAVAILABLE'), 'malformed pose stamp rejected without crash')
            state['stamp'] = 'fresh'
            wait_for(node, lambda: speed(10. / 3.6), 'pose recovery')
            withdrawn = MissionMap(); withdrawn.header.frame_id = 'map'; withdrawn.path.header.frame_id = 'map'
            map_pub.publish(withdrawn)
            wait_for(node, lambda: invalid('INVALID_MISSION_MAP'), 'withdrawn map invalidates mission')
            missing = map_message(); missing.section_names = ['different_section']
            map_pub.publish(missing)
            wait_for(node, lambda: invalid('MISSION_SECTION_NOT_IN_MAP'), 'missing configured section stops')
            map_pub.publish(map_message())
            wait_for(node, lambda: speed(10. / 3.6), 'map recovery')
        mission.close()
        wait_for(node, lambda: plans and not plans[-1].valid and plans[-1].stop_request,
                 'MissionLimit timeout stops downstream plan')
    finally:
        mission.close(); planning.close(); node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try:
        fixture(False)
        fixture(True)
    finally:
        rclpy.shutdown()
    print('ALL MISSION TESTS PASSED', flush=True)
