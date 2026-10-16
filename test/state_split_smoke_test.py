#!/usr/bin/env python3
"""State/Guidance migration contracts. Synthetic fixtures only, localhost domain 179.
Uses the same confined logs/environment as smoke_test.py; no external node starts.
"""
import copy
import math
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from control_manager.msg import (GlobalPose, MotionState, LanePose,
                                 MissionLimit, ObjectState, VehicleCommand, MotionPlan)
from smoke_test import ROOT, Stack, observe, wait_for


def start(executable, label, *params):
    args = ['ros2', 'run', 'control_manager', executable if executable in {'global_state_estimator', 'lane_state_estimator'} else executable + '_node', '--ros-args',
            '-r', '__ns:=/control', '--params-file', str(ROOT/'config/vehicle.yaml')]
    # These fixtures explicitly supply IMU/Lane observations when testing those paths.
    if executable == 'global_state_estimator': args += ['-p', 'imu_enabled:=true']
    if executable == 'lane_state_estimator' or any(
            p.startswith('topics.external_lane:=') or p in {'guidance_mode:=LANE', 'guidance_mode:=HYBRID'}
            for p in params):
        args += ['-p', 'lane_enabled:=true']
    for param in params:
        args += ['-p', param]
    return Stack(args, label)


def settle(node, seconds=0.65):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.02)


def estimators():
    node = rclpy.create_node('state_split_estimator_fixture')
    values = observe(node)
    pub = node.create_publisher(Odometry, '/control/sensors/gnss_odom', qos_profile_sensor_data)
    state = {'case': 'good', 'send': True}
    observed_stamp = [None]

    def publish():
        if not state['send']: return
        o = Odometry()
        o.header.frame_id = 'map'; o.child_frame_id = 'base_link'
        o.header.stamp = node.get_clock().now().to_msg()
        o.pose.pose.position.x = 3.; o.pose.pose.position.y = -2.
        o.pose.pose.orientation.z = math.sin(.1); o.pose.pose.orientation.w = math.cos(.1)
        o.pose.covariance[0] = .02; o.pose.covariance[7] = .03; o.pose.covariance[35] = .04
        o.twist.twist.linear.x = 1.2; o.twist.covariance[0] = .05
        o.twist.twist.angular.z = 10.  # Deliberately untrusted, must NOT become yaw rate.
        case = state['case']
        if case == 'stale': o.header.stamp.sec -= 10
        if case == 'future': o.header.stamp.sec += 10
        if case == 'zero': o.header.stamp.sec = o.header.stamp.nanosec = 0
        if case == 'negative_time': o.header.stamp.sec = -1
        if case == 'nanosecond_boundary': o.header.stamp.nanosec = 1000000000
        if case == 'noncanonical_time':
            # This would normalize to the current time without a wire-format check.
            o.header.stamp.sec -= 1; o.header.stamp.nanosec += 1000000000
        if case == 'map_frame': o.header.frame_id = 'wrong'
        if case == 'child_frame': o.child_frame_id = 'wrong'
        if case == 'position_cov': o.pose.covariance[0] = 2.
        if case == 'sum_cov': o.pose.covariance[0] = o.pose.covariance[7] = .6
        if case == 'negative_cov': o.pose.covariance[7] = -.1
        if case == 'nan_cov': o.pose.covariance[0] = float('nan')
        if case == 'yaw_cov': o.pose.covariance[35] = 2.
        if case == 'quaternion': o.pose.pose.orientation.w = 0.
        if case == 'nan_quaternion': o.pose.pose.orientation.w = float('nan')
        if case == 'nan_position': o.pose.pose.position.x = float('nan')
        if case == 'speed_cov': o.twist.covariance[0] = 1e12
        if case == 'negative_speed_cov': o.twist.covariance[0] = -.1
        if case == 'nan_speed_cov': o.twist.covariance[0] = float('nan')
        if case == 'nan_speed': o.twist.twist.linear.x = float('nan')
        observed_stamp[0] = copy.deepcopy(o.header.stamp)
        pub.publish(o)

    timer = node.create_timer(.05, publish)
    stacks = [start('global_state_estimator', 'split_global')]
    try:
        def good():
            return ('global_pose' in values and 'motion_state' in values and
                    values['global_pose'].valid and values['motion_state'].speed_valid)
        wait_for(node, good, 'independent GNSS GlobalPose and MotionState')
        p, m = values['global_pose'], values['motion_state']
        assert (p.x_m, p.y_m) == (3., -2.) and abs(p.yaw_rad-.2) < 1e-12
        assert (p.position_variance_x_m2, p.position_variance_y_m2, p.yaw_variance_rad2) == (.02,.03,.04)
        assert p.header.frame_id == 'map' and m.header.frame_id == 'base_link'
        assert m.speed_mps == 1.2 and not m.yaw_rate_valid
        print('PASS: pose uncertainty, speed, frames and unsupported yaw rate', flush=True)
        malformed_stamps = ['negative_time', 'nanosecond_boundary', 'noncanonical_time']
        shared = ['stale', 'future', 'zero', 'map_frame', 'child_frame'] + malformed_stamps
        pose_only = ['position_cov', 'sum_cov', 'negative_cov', 'nan_cov', 'yaw_cov',
                     'quaternion', 'nan_quaternion', 'nan_position']
        motion_only = ['speed_cov', 'negative_speed_cov', 'nan_speed_cov', 'nan_speed']
        for case in shared + pose_only + motion_only:
            state['case'] = case
            want_pose = case in motion_only
            want_motion = case in pose_only
            wait_for(node, lambda: values['global_pose'].valid == want_pose and
                     values['motion_state'].speed_valid == want_motion,
                     'split validation: ' + case)
            assert not values['motion_state'].yaw_rate_valid
            if case in malformed_stamps:
                assert stacks[0].process.poll() is None
                for key in ('global_pose', 'motion_state'):
                    assert values[key].reason == 'INVALID_GNSS_TIMESTAMP'
                    assert values[key].header.stamp.sec == values[key].header.stamp.nanosec == 0
            state['case'] = 'good'
            wait_for(node, good, 'restore measured input after ' + case)
        state['send'] = False
        settle(node)
        wait_for(node, lambda: not values['global_pose'].valid and not values['motion_state'].speed_valid,
                 'receipt timeout expires both states')
        assert values['global_pose'].header.stamp == observed_stamp[0]
        assert values['motion_state'].header.stamp == observed_stamp[0]
        print('PASS: estimator heartbeat preserves original observation stamp', flush=True)
    finally:
        for stack in stacks: stack.close()
    # Each unsupported mode remains visible as an invalid heartbeat, never GNSS fallback.
    state['send'] = True
    for executable, params, key, reason in [
        ('global_state_estimator', ['pose_mode:=GNSS_IMU'], 'global_pose', 'GNSS_IMU_NOT_IMPLEMENTED'),
        ('global_state_estimator', ['motion_mode:=WHEEL'], 'motion_state', 'WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED')]:
        values.clear()
        stack = start(executable, 'split_' + reason.lower(), *params)
        try:
            wait_for(node, lambda: key in values and values[key].reason == reason,
                     'explicit unsupported state: ' + reason)
            out = values[key]
            assert not (out.valid if key == 'global_pose' else out.speed_valid)
            if 'NOT_IMPLEMENTED' in reason:
                assert out.header.stamp.sec == out.header.stamp.nanosec == 0
        finally: stack.close()
    node.destroy_timer(timer); node.destroy_node()


def guidance():
    node = rclpy.create_node('state_split_guidance_fixture')
    values = observe(node)
    pubs = {key: node.create_publisher(cls, '/test/external_lane' if key == 'lane_pose' else '/control/' + key, 10) for key, cls in [
        ('lane_pose', LanePose), ('global_pose', GlobalPose)]}
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    path_pub = node.create_publisher(PathMsg, '/control/global_path', durable)
    state = {'lane': 'good', 'pose': False, 'path': False}

    def publish():
        stamp = node.get_clock().now().to_msg()
        lane = LanePose(lateral_error_m=.2, heading_error_rad=.3, confidence=.9)
        lane.header.stamp = copy.deepcopy(stamp)
        if state['lane'] == 'stale': lane.header.stamp.sec -= 10
        if state['lane'] == 'future': lane.header.stamp.sec += 10
        if state['lane'] == 'zero': lane.header.stamp.sec = lane.header.stamp.nanosec = 0
        if state['lane'] == 'low': lane.confidence = .1
        if state['lane'] == 'nan': lane.heading_error_rad = float('nan')
        if state['lane'] == 'confidence_nan': lane.confidence = float('nan')
        if state['lane'] == 'range': lane.confidence = 1.1
        if state['lane'] != 'absent': pubs['lane_pose'].publish(lane)
        if state['pose']:
            pose = GlobalPose(valid=True); pose.header.stamp = stamp; pose.header.frame_id = 'map'
            pubs['global_pose'].publish(pose)
        path = PathMsg(); path.header.frame_id = 'map'; path.header.stamp = stamp
        if state['path']:
            for x in [0., 10.]:
                p = PoseStamped(); p.pose.position.x = x; p.pose.orientation.w = 1.
                path.poses.append(p)
        path_pub.publish(path)

    timer = node.create_timer(.05, publish)
    ingress = start('sensor_interface', 'split_ingress', 'topics.external_lane:=/test/external_lane')
    lane_estimator = start('lane_state_estimator', 'split_lane')
    stack = start('path_planning', 'split_guidance', 'guidance_mode:=LANE', 'map_datum_verified:=true')
    try:
        wait_for(node, lambda: 'reference_path' in values and not values['reference_path'].poses,
                 'LANE without global inputs publishes an empty path')
        state.update(pose=True, path=True)
        settle(node)
        assert not values['reference_path'].poses
        print('PASS: LANE never falls back to healthy global inputs', flush=True)
        stack.close()
        values.pop('reference_path', None)
        stack = start('path_planning', 'split_guidance_hybrid', 'guidance_mode:=HYBRID', 'map_datum_verified:=true')
        wait_for(node, lambda: 'reference_path' in values and not values['reference_path'].poses,
                 'HYBRID healthy inputs still publish an empty path until implemented')
        settle(node)
        assert stack.process.poll() is None and not values['reference_path'].poses
        state['lane'] = 'nan'
        stack.close()
        values.pop('reference_path', None)
        stack = start('path_planning', 'split_guidance_gps', 'guidance_mode:=GPS', 'map_datum_verified:=true')
        wait_for(node, lambda: 'reference_path' in values and len(values['reference_path'].poses)==2,
                 'GPS startup mode ignores ingress-rejected Lane')
        state['pose'] = False
        wait_for(node, lambda: not values['reference_path'].poses, 'GPS GlobalPose receipt timeout')
        state['pose'] = True
        wait_for(node, lambda: len(values['reference_path'].poses)==2, 'GPS recovers with fresh pose')
        assert not node.get_publishers_info_by_topic('/control/guidance_status')
    finally:
        stack.close(); lane_estimator.close(); ingress.close(); node.destroy_timer(timer); node.destroy_node()


def downstream():
    node = rclpy.create_node('state_split_downstream_fixture')
    values = observe(node)
    pubs = {key: node.create_publisher(cls, '/control/' + key, 10) for key, cls in [
        ('motion_state', MotionState),
        ('selected_cmd', VehicleCommand), ('object_state', ObjectState),
        ('mission_limit', MissionLimit), ('reference_path', PathMsg)]}
    state = {'motion': True, 'speed_valid': True}
    def publish():
        stamp = node.get_clock().now().to_msg()
        def send(key, msg, frame):
            msg.header.stamp = stamp; msg.header.frame_id = frame; pubs[key].publish(msg)
        if state['motion']:
            send('motion_state', MotionState(speed_mps=1., speed_valid=state['speed_valid']), 'base_link')
        send('selected_cmd', VehicleCommand(target_speed_mps=.5, target_brake=False, valid=True,
                                           reason='TEST_ONLY_SELECTED_COMMAND'), 'base_link')
        send('object_state', ObjectState(valid=True, obstacle_valid=True), 'base_link')
        send('mission_limit', MissionLimit(valid=True, max_speed_mps=1.), 'map')
        path = PathMsg()
        for x in [0., 10.]:
            p = PoseStamped(); p.header.frame_id = 'base_link'; p.pose.position.x = x
            p.pose.orientation.w = 1.; path.poses.append(p)
        send('reference_path', path, 'base_link')  # Explicit TEST geometry; never generated by Lane runtime.
    timer = node.create_timer(.05, publish)
    stacks = [start('motion_planning', 'split_motion_planning', 'cruise_speed_mps:=0.5'),
              start('motion_control', 'split_local_path_blocker', 'tracking_mode:=PURE_PURSUIT'),
              start('vehicle_interface', 'split_safety', 'operating_limits_verified:=true',
                    'max_steer_rad:=0.5', 'max_speed_mps:=1.0')]
    try:
        wait_for(node, lambda: 'motion_plan' in values and values['motion_plan'].valid and
                 values['motion_plan'].header.frame_id == 'base_link',
                 'MotionPlanning uses MotionState and ReferencePath with NO GlobalPose')
        wait_for(node, lambda: 'auto_cmd' in values and not values['auto_cmd'].valid and
                 values['auto_cmd'].reason == 'LOCAL_REFERENCE_PATH_NOT_SUPPORTED',
                 'legacy controller explicitly rejects local path, never invents zero pose')
        wait_for(node, lambda: 'protected_cmd' in values and values['protected_cmd'].valid,
                 'isolated selected command needs no GlobalPose or Guidance at actuator boundary')
        state['speed_valid'] = False
        wait_for(node, lambda: not values['motion_plan'].valid and values['protected_cmd'].valid,
                 'invalid MotionState blocks planning; no duplicate actuator state gate')
        state['speed_valid'] = True
        wait_for(node, lambda: values['motion_plan'].valid, 'restore valid MotionState')
        state['motion'] = False
        wait_for(node, lambda: not values['motion_plan'].valid and values['protected_cmd'].valid,
                 'MotionState timeout blocks planning')
        state['motion'] = True
        wait_for(node, lambda: values['motion_plan'].valid, 'restore fresh MotionState')
        for topic in ['global_pose','motion_state','guidance_status','object_state']:
            assert all(info.node_name!='vehicle_interface' for info in
                       node.get_subscriptions_info_by_topic('/control/'+topic))
        print('PASS: actuator boundary has no state/guidance/object subscriptions',flush=True)
    finally:
        for stack in stacks: stack.close()
        node.destroy_timer(timer); node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try:
        estimators(); guidance(); downstream()
        print('ALL STATE SPLIT SMOKE CHECKS PASSED', flush=True)
    finally: rclpy.shutdown()
