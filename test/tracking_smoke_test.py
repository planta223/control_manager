#!/usr/bin/env python3
"""Tracking integration fixtures only; no sensors, hardware or actuator senders.
Use the same localhost/domain 179 environment as smoke_test.py.
"""
import copy
import math
import time
import xml.etree.ElementTree as ET
import rclpy
import yaml
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from control_manager.msg import GlobalPose, MotionState, VehicleCommand, MotionPlan, ObjectState, ControlStatus, TrackingStatus
from smoke_test import ROOT, LOG, Stack, observe, wait_for


def run():
    node = rclpy.create_node('tracking_fixture_test')
    values = observe(node)
    pubs = {name: node.create_publisher(cls, '/control/' + name, 10) for name, cls in [
        ('global_pose', GlobalPose), ('motion_state', MotionState), ('motion_plan', MotionPlan), ('object_state', ObjectState),
        ('manual_cmd', VehicleCommand), ('mode_request', UInt8), ('tracking_mode_request', UInt8)]}
    active = dict(state=True, motion=True, speed_valid=True, motion_age=0, motion_frame='base_link',
                  plan=True, state_age=0, plan_age=0, path_age=0,
                  localization=True, plan_valid=True, stop=False, path='normal', y=0.0,
                  state_y=-1.0, status_frame='map')

    def publish():
        stamp = node.get_clock().now().to_msg()
        s = GlobalPose(x_m=1., y_m=active['state_y'], yaw_rad=0., valid=active['localization'])
        m = MotionState(speed_mps=1., speed_valid=active['speed_valid'])
        m.header.frame_id = active['motion_frame']; m.header.stamp = copy.deepcopy(stamp)
        m.header.stamp.sec -= active['motion_age']
        s.header.frame_id = active['status_frame']; s.header.stamp = copy.deepcopy(stamp)
        s.header.stamp.sec -= active['state_age']
        path = PathMsg(); path.header.frame_id = 'map'; path.header.stamp = copy.deepcopy(stamp)
        path.header.stamp.sec -= active['path_age']
        for x in [0., 20.]:
            p = PoseStamped(); p.header.frame_id = 'map'; p.pose.position.x = x
            p.pose.position.y = active['y']; p.pose.orientation.w = 1.
            path.poses.append(p)
        if active['path'] == 'empty': path = PathMsg()
        if active['path'] == 'one': path.poses = path.poses[:1]
        if active['path'] == 'nan': path.poses[1].pose.position.y = float('nan')
        if active['path'] == 'inf': path.poses[1].pose.position.x = float('inf')
        if active['path'] == 'wrong_frame': path.header.frame_id = 'other'
        plan = MotionPlan(reference_path=path, target_speed_mps=1., stop_request=active['stop'],
                          valid=active['plan_valid'], reason='TEST_ONLY')
        plan.header.stamp = copy.deepcopy(stamp); plan.header.frame_id = 'map'
        plan.header.stamp.sec -= active['plan_age']
        if active['state']:
            pubs['global_pose'].publish(s)
        if active['motion']: pubs['motion_state'].publish(m)
        if active['plan']: pubs['motion_plan'].publish(plan)
        obj = ObjectState(valid=True, obstacle_valid=True, obstacle_detected=False)
        obj.header.frame_id = 'base_link'; obj.header.stamp = stamp
        pubs['object_state'].publish(obj)
        manual = VehicleCommand(target_steer_rad=-0.1, target_speed_mps=0.4,
                                target_brake=False, valid=True, reason='MANUAL_FIXTURE')
        manual.header.frame_id = 'base_link'; manual.header.stamp = stamp
        pubs['manual_cmd'].publish(manual)

    timer = node.create_timer(0.05, publish)
    default = Stack(['ros2','run','control_manager','motion_control_node','--ros-args',
                     '-r','__ns:=/control'], 'tracking_default')
    try:
        wait_for(node, lambda: 'tracking_status' in values and
                 values['tracking_status'].requested_mode == TrackingStatus.PID and
                 values['tracking_status'].actual_mode == TrackingStatus.NONE and
                 values['tracking_status'].reason == 'PID_NOT_CONFIGURED' and
                 'auto_cmd' in values and not values['auto_cmd'].valid and values['auto_cmd'].target_brake,
                 'default PID with healthy fixtures fails closed until configured')
    finally:
        default.close()
    config = yaml.safe_load((ROOT/'config/control_manager.yaml').read_text())
    config['/control/motion_control']['ros__parameters']['pid'].update({
        'enabled': True, 'kp_lateral': 0.2, 'ki_lateral': 0.3, 'kd_lateral': 0.02,
        'kp_heading': 0.4, 'integral_limit_m_s': 0.5, 'max_steering_rad': 0.5})
    config['/control/safety_limit']['ros__parameters'].update({
        'operating_limits_verified': True, 'max_steer_rad': 0.6,
        'max_speed_mps': 2.0, 'emergency_distance_m': 1.0})
    cfg = LOG/'tracking_fixture.yaml'; cfg.write_text(yaml.safe_dump(config))
    xml = ET.parse(ROOT/'launch/control_manager.xml')
    for elem in list(xml.getroot()):
        if elem.tag == 'node' and elem.attrib['name'] not in {
                'motion_control','control_manager','safety_limit','vehicle_interface'}:
            xml.getroot().remove(elem)
    launch = LOG/'tracking_fixture.xml'; xml.write(launch, encoding='unicode')
    values.clear()
    stack = Stack(['ros2','launch',str(launch),'params_file:='+str(cfg)], 'tracking_integration')

    def selected(mode, reason=None):
        return ('tracking_status' in values and values['tracking_status'].requested_mode == mode and
                (reason is None or values['tracking_status'].reason == reason))

    def auto_pid():
        return (selected(TrackingStatus.PID) and values['tracking_status'].valid and
                values['tracking_status'].actual_mode == TrackingStatus.PID and
                'final_cmd' in values and values['final_cmd'].valid and
                values['final_cmd'].target_speed_mps == 1.)

    def invalid():
        return ('auto_cmd' in values and not values['auto_cmd'].valid and values['auto_cmd'].target_brake and
                not values['tracking_status'].valid and values['tracking_status'].actual_mode == TrackingStatus.NONE and
                'final_cmd' in values and not values['final_cmd'].valid and values['final_cmd'].target_brake)

    def wait_reset(label, expected=0.2):
        wait_for(node, lambda: auto_pid() and abs(values['auto_cmd'].target_steer_rad-expected)<1e-6, label)

    try:
        wait_for(node, lambda: selected(TrackingStatus.PID, 'PID_PREVIEW_STATE_RESET'),
                 'IDLE PID preview uses no integral history')
        assert abs(values['auto_cmd'].target_steer_rad-0.2)<1e-6
        pubs['mode_request'].publish(UInt8(data=ControlStatus.AUTO))
        wait_for(node, auto_pid, 'configured default PID produces AUTO command')
        assert values['auto_cmd'].target_speed_mps == 1. and not values['auto_cmd'].target_brake
        wait_for(node, lambda: auto_pid() and values['auto_cmd'].target_steer_rad > 0.28,
                 'PID integral accumulates while AUTO is healthy')
        # Controller-only output bound; final safety limits remain a separate gate.
        active['state_y'] = -5.
        wait_for(node, lambda: auto_pid() and values['auto_cmd'].target_steer_rad == 0.5,
                 'PID output clamps at configured software bound')
        active['state_y'] = -1.
        pubs['tracking_mode_request'].publish(UInt8(data=TrackingStatus.PURE_PURSUIT))
        expected_pp = math.atan2(2*1.328, 3.*3.+1.)
        wait_for(node, lambda: selected(TrackingStatus.PURE_PURSUIT, 'PURE_PURSUIT_ACTIVE') and
                 values['tracking_status'].actual_mode == TrackingStatus.PURE_PURSUIT and
                 values['auto_cmd'].valid and abs(values['auto_cmd'].target_steer_rad-expected_pp)<1e-9,
                 'PURE_PURSUIT request preserves known legacy steering')
        assert values['status'].mode == ControlStatus.AUTO
        pubs['tracking_mode_request'].publish(UInt8(data=TrackingStatus.PID))
        wait_reset('PP -> PID resets accumulated I and derivative')
        for mode, reason in [(TrackingStatus.ADAPTIVE_PURE_PURSUIT,'ADAPTIVE_PURE_PURSUIT_NOT_IMPLEMENTED'),
                             (255,'INVALID_TRACKING_MODE')]:
            pubs['tracking_mode_request'].publish(UInt8(data=mode))
            wait_for(node, lambda: selected(mode,reason) and invalid(), 'request fails closed: '+reason)
            assert values['status'].mode == ControlStatus.AUTO
            pubs['mode_request'].publish(UInt8(data=ControlStatus.MANUAL))
            wait_for(node, lambda: values['status'].mode == ControlStatus.MANUAL and
                     values['final_cmd'].valid and values['final_cmd'].target_speed_mps==0.4,
                     'invalid tracking does not select or replace MANUAL')
            assert selected(mode,reason)
            pubs['mode_request'].publish(UInt8(data=ControlStatus.AUTO))
            wait_for(node, lambda: values['status'].mode == ControlStatus.AUTO and invalid(),
                     'AUTO cannot fall back to manual with invalid tracking')
            pubs['tracking_mode_request'].publish(UInt8(data=TrackingStatus.PID))
            wait_reset('explicit PID recovery starts with zero history')
        wait_for(node, lambda: auto_pid() and values['auto_cmd'].target_steer_rad>0.28,
                 'accumulate before MANUAL reset')
        pubs['mode_request'].publish(UInt8(data=ControlStatus.MANUAL))
        wait_for(node, lambda: values['status'].mode == ControlStatus.MANUAL and
                 selected(TrackingStatus.PID,'PID_PREVIEW_STATE_RESET') and
                 abs(values['auto_cmd'].target_steer_rad-0.2)<1e-6,
                 'MANUAL resets PID state while arbitration stays independent')
        pubs['mode_request'].publish(UInt8(data=ControlStatus.AUTO))
        wait_for(node, auto_pid, 'AUTO resumes after MANUAL')
        for key, bad, good in [('state',False,True), ('motion',False,True),
                               ('speed_valid',False,True), ('motion_age',10,0),
                               ('motion_frame','other','base_link'), ('plan',False,True),
                               ('state_age',10,0), ('plan_age',10,0), ('path_age',10,0),
                               ('localization',False,True), ('plan_valid',False,True),
                               ('status_frame','other','map'), ('state_y',float('nan'),-1.),
                               ('path','empty','normal'), ('path','one','normal'), ('path','nan','normal'),
                               ('path','inf','normal'), ('path','wrong_frame','normal')]:
            active[key]=bad
            wait_for(node, invalid, 'invalid/stale input stops AUTO: '+key+'='+str(bad))
            active[key]=good
            wait_reset('input recovery resets PID history: '+key)
        active['stop']=True
        wait_for(node, lambda: selected(TrackingStatus.PID,'PLAN_STOP') and
                 not values['tracking_status'].valid and values['auto_cmd'].valid and
                 values['auto_cmd'].target_brake and values['auto_cmd'].target_speed_mps==0.,
                 'MotionPlan stop keeps brake/speed policy and resets tracking')
        active['path']='empty'
        wait_for(node, lambda: selected(TrackingStatus.PID,'PLAN_STOP') and values['auto_cmd'].valid,
                 'stop-only plan needs no nested path geometry')
        active['path']='normal'
        active['stop']=False
        wait_reset('resume after stop has zero PID history')
        wait_for(node, lambda: auto_pid() and values['auto_cmd'].target_steer_rad>0.28,
                 'accumulate before path replacement')
        active['y']=-0.25
        wait_reset('path geometry replacement resets I/D',expected=0.15)
        print('ALL TRACKING SMOKE CHECKS PASSED',flush=True)
    finally:
        stack.close(); node.destroy_timer(timer); node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
