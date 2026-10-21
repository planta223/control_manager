#!/usr/bin/env python3
"""real-map PP diagnostics, hardware-free.

Run with an installed control_manager and the isolated environment in smoke_test.py.
Only motion_control_node is launched; no actuator sender or hardware interface.
"""
import copy
import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from control_manager.msg import (ControlStatus, GlobalPose, MotionPlan, MotionState,
                                 PurePursuitDiagnostic, VehicleCommand)
from smoke_test import LOG, ROOT, Stack, wait_for
from operation_smoke_test import command


def run(map_xy):
    points = [tuple(map(float, line.split())) for line in Path(map_xy).read_text().splitlines()]
    assert len(points) == 5032, 'Use the projected MAP_preliminary fixture'
    node = rclpy.create_node('mission_pp_fixture')
    values = {}
    for topic, cls in [('pp_diagnostic', PurePursuitDiagnostic), ('auto_cmd', VehicleCommand)]:
        node.create_subscription(cls, '/control/' + topic,
                                 lambda msg, key=topic: values.update({key: msg}), 10)
    pubs = {topic: node.create_publisher(cls, '/control/' + topic, 10) for topic, cls in
            [('global_pose', GlobalPose), ('motion_state', MotionState),
             ('motion_plan', MotionPlan), ('status', ControlStatus)]}
    path = PathMsg()
    path.header.frame_id = 'map'
    for x, y in points:
        p = PoseStamped()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.w = 1.0
        path.poses.append(p)
    active = dict(index=0, paused=False, sequence=0, valid=True, mismatch=False)

    def publish():
        stamp = node.get_clock().now().to_msg()
        i = active['index']
        x, y = points[i]
        yaw = math.atan2(points[i+1][1]-y, points[i+1][0]-x)
        pose = GlobalPose(x_m=x, y_m=y, yaw_rad=yaw, valid=active['valid'],
                          position_valid=active['valid'])
        pose.header.frame_id, pose.header.stamp = 'map', copy.deepcopy(stamp)
        motion = MotionState(speed_mps=5.0, speed_valid=True)
        motion.header.frame_id, motion.header.stamp = 'base_link', copy.deepcopy(stamp)
        path.header.stamp = copy.deepcopy(stamp)
        plan = MotionPlan(reference_path=path, target_speed_mps=5.0, valid=True)
        if active['mismatch']:
            plan.reference_path = copy.deepcopy(path)
            plan.reference_path.poses.pop()
        plan.header.frame_id, plan.header.stamp = 'map', copy.deepcopy(stamp)
        status = ControlStatus(mode=ControlStatus.AUTO, fsm_state=ControlStatus.DRIVING,
                               paused=active['paused'], resume_sequence=active['sequence'])
        status.header.frame_id, status.header.stamp = 'base_link', copy.deepcopy(stamp)
        for key, msg in [('global_pose', pose), ('motion_state', motion),
                         ('motion_plan', plan), ('status', status)]:
            pubs[key].publish(msg)

    timer = node.create_timer(0.05, publish)
    args = command('motion_control_node') + [
        '--params-file', str(ROOT/'config/modes/preliminary_lap.yaml')]
    stack = Stack(args, 'mission_pp')

    def current_valid(index, after_ns):
        d = values.get('pp_diagnostic')
        return (d is not None and d.valid and
                d.pose_stamp.sec*10**9+d.pose_stamp.nanosec >= after_ns and
                abs(d.matched_segment-index) <= 2)

    try:
        shortened = False
        for index in range(0, 741, 10):
            active['index'] = index
            after = node.get_clock().now().nanoseconds
            wait_for(node, lambda: current_valid(index, after),
                     'preliminary PP stays on segment ' + str(index))
            d = values['pp_diagnostic']
            assert d.header.frame_id == 'map' and not d.heading_held
            assert d.motion_stamp.sec > 0 and abs(d.used_yaw_rad) <= math.pi
            assert d.minimum_lookahead_m <= d.effective_lookahead_m <= d.speed_lookahead_m
            assert abs(d.lateral_error_m) < 0.02
            shortened |= d.effective_lookahead_m < d.speed_lookahead_m-0.01
        assert shortened, 'Map fixture must exercise curvature-adaptive lookahead'
        wait_for(node, lambda: 'auto_cmd' in values and values['auto_cmd'].valid and
                 not values['auto_cmd'].target_brake, 'PP diagnostic accompanies a valid command')
        last_steer = values['auto_cmd'].target_steer_rad
        active['paused'] = True
        wait_for(node, lambda: not values['pp_diagnostic'].valid and
                 values['pp_diagnostic'].reason == 'AUTO_PAUSED' and
                 values['auto_cmd'].target_brake, 'pause invalidates the displayed PP target')
        assert abs(values['auto_cmd'].target_steer_rad-last_steer) < 1e-9
        active['paused'], active['sequence'] = False, 1
        after = node.get_clock().now().nanoseconds
        wait_for(node, lambda: current_valid(active['index'], after), 'fresh state resumes PP diagnostics')
        active['valid'] = False
        wait_for(node, lambda: not values['pp_diagnostic'].valid and
                 not values['auto_cmd'].valid and values['auto_cmd'].target_brake,
                 'invalid localization clears diagnostic validity')
        active['valid'], active['mismatch'] = True, True
        wait_for(node, lambda: not values['pp_diagnostic'].valid and
                 values['pp_diagnostic'].reason == 'ROUTE_PROFILE_POINT_COUNT_MISMATCH',
                 'wrong map profile fails closed')
        print('ALL MISSION PP CHECKS PASSED', flush=True)
    finally:
        stack.close()
        node.destroy_timer(timer)
        node.destroy_node()


if __name__ == '__main__':
    for directory in (LOG, LOG/'tmp', LOG/'ros'):
        directory.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    try:
        run(sys.argv[1])
    finally:
        rclpy.shutdown()
