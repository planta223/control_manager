#!/usr/bin/env python3
"""GNSS -> estimators -> planning -> control, hardware-free heading startup checks."""
import copy
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import UInt8
from control_manager.msg import ControlStatus
from smoke_test import ROOT, LOG, Stack, observe, wait_for


def run():
    node = rclpy.create_node('heading_bootstrap_fixture')
    values = observe(node)
    odom_pub = node.create_publisher(Odometry, '/gnss/odom', qos_profile_sensor_data)
    path_pub = node.create_publisher(PathMsg, '/control/global_path', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    mode_pub = node.create_publisher(UInt8, '/control/mode_request', 10)
    state = dict(y=0.0, covariance=4.5, yaw_covariance=1e12, path=True)

    def publish():
        stamp = node.get_clock().now().to_msg()
        odom = Odometry(); odom.header.stamp = stamp
        odom.header.frame_id = 'map'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = 1.0; odom.pose.pose.position.y = state['y']
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0] = odom.pose.covariance[7] = state['covariance']
        odom.pose.covariance[35] = state['yaw_covariance']
        odom.twist.covariance[0] = 0.01  # Fresh, observed zero speed at rest.
        odom_pub.publish(odom)
        path = PathMsg(); path.header.stamp = copy.deepcopy(stamp); path.header.frame_id = 'map'
        if state['path']:
            for x in (0.0,20.0):
                pose = PoseStamped(); pose.pose.position.x = x; pose.pose.orientation.w = 1.0
                path.poses.append(pose)
        path_pub.publish(path)

    timer = node.create_timer(0.05,publish)
    operation = yaml.safe_load((ROOT/'config/operation.yaml').read_text())
    operation.setdefault('/control/global_path_provider',{'ros__parameters':{}})['ros__parameters']['map_file'] = ''
    operation['/**']['ros__parameters'].update(object_state_enabled=False)
    operation['/control/motion_control']['ros__parameters']['heading_bootstrap_enabled'] = True
    control = yaml.safe_load((ROOT/'config/control.yaml').read_text())
    control['/control/motion_control']['ros__parameters']['heading_bootstrap_timeout_s'] = 0.7
    operation_file = LOG/'heading_bootstrap_operation.yaml'; operation_file.write_text(yaml.safe_dump(operation))
    control_file = LOG/'heading_bootstrap_control.yaml'; control_file.write_text(yaml.safe_dump(control))
    stack = Stack(['ros2','launch','control_manager','control_manager.xml',
                   'operation_file:='+str(operation_file),'control_file:='+str(control_file)], 'heading_bootstrap')

    def auto(reason):
        return 'auto_cmd' in values and values['auto_cmd'].reason == reason

    def settle(seconds):
        until = time.monotonic()+seconds
        while time.monotonic() < until: rclpy.spin_once(node,timeout_sec=0.02)

    try:
        wait_for(node,lambda: 'global_pose' in values and values['global_pose'].position_valid and
                 values['global_pose'].initial_heading_unavailable and not values['global_pose'].valid,
                 '3 m position accepted while initial heading remains explicitly unavailable')
        wait_for(node,lambda: auto('HEADING_BOOTSTRAP_WAITING_FOR_AUTO') and
                 values['auto_cmd'].target_brake,'IDLE does not issue a straight driving command')
        settle(0.9)
        assert auto('HEADING_BOOTSTRAP_WAITING_FOR_AUTO')
        assert values['reference_path'].poses and values['motion_plan'].valid
        state['path'] = False
        wait_for(node,lambda: 'motion_plan' in values and not values['motion_plan'].valid,
                 'missing map blocks startup')
        wait_for(node,lambda: mode_pub.get_subscription_count()>0 and 'status' in values,
                 'selector ready for AUTO request')
        mode_pub.publish(UInt8(data=ControlStatus.AUTO))
        wait_for(node,lambda: values['status'].mode==ControlStatus.AUTO,'AUTO selected')
        settle(0.9)
        assert not values['auto_cmd'].valid and values['auto_cmd'].target_brake
        state['path'] = True
        wait_for(node,lambda: auto('HEADING_BOOTSTRAP_ACTIVE') and values['auto_cmd'].valid and
                 values['auto_cmd'].target_steer_rad==0.0 and values['auto_cmd'].target_speed_mps>0 and
                 values['protected_cmd'].valid,'bounded straight startup reaches protected command')
        wait_for(node,lambda: auto('HEADING_BOOTSTRAP_TIMEOUT') and
                 not values['auto_cmd'].valid and values['auto_cmd'].target_brake,
                 'heading startup expires without stopping the node')
        settle(0.4)
        assert auto('HEADING_BOOTSTRAP_TIMEOUT')
        state['yaw_covariance'] = 0.02
        wait_for(node,lambda: auto('PURE_PURSUIT_ACTIVE') and values['auto_cmd'].valid,
                 'heading acquisition transfers to Pure Pursuit')
        state['covariance'] = 4.5001
        wait_for(node,lambda: not values['global_pose'].position_valid and not values['auto_cmd'].valid,
                 'position standard deviation above 3 m stops AUTO')
        state['covariance'] = 4.5
        wait_for(node,lambda: auto('PURE_PURSUIT_ACTIVE') and values['auto_cmd'].valid,
                 'position standard deviation at 3 m recovers')
        state['y'] = 3.01
        wait_for(node,lambda: auto('PATH_DEVIATION_EXCEEDED') and not values['auto_cmd'].valid,
                 'path deviation above 3 m stops AUTO')
        state['y'] = 3.0
        wait_for(node,lambda: (auto('PURE_PURSUIT_ACTIVE') or auto('PURE_PURSUIT_ACTIVE_LIMITED')) and values['auto_cmd'].valid,
                 'path deviation at 3 m accepted')
        state['yaw_covariance'] = 1e12
        wait_for(node,lambda: not values['global_pose'].valid and
                 not values['global_pose'].initial_heading_unavailable and not values['auto_cmd'].valid,
                 'heading loss after acquisition never starts another straight-drive window')
        settle(0.9)
        assert not values['auto_cmd'].valid and values['auto_cmd'].target_brake
        assert stack.process.poll() is None
        print('ALL HEADING BOOTSTRAP CHECKS PASSED',flush=True)
    finally:
        stack.close(); node.destroy_timer(timer); node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
