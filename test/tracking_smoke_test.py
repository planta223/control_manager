#!/usr/bin/env python3
"""Tracking integration fixtures only; no sensors, hardware or actuator senders.
Use the same localhost/domain 179 environment as smoke_test.py.
"""
import copy
import math
import subprocess
import time
import xml.etree.ElementTree as ET
import rclpy
import yaml
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from control_manager.msg import GlobalPose, MotionState, VehicleCommand, MotionPlan, ObjectState, ControlStatus
from smoke_test import ROOT, LOG, Stack, observe, wait_for
from operation_smoke_test import command


def run():
    node = rclpy.create_node('tracking_fixture_test')
    values = observe(node)
    pubs = {name: node.create_publisher(cls, '/control/' + name, 10) for name, cls in [
        ('global_pose', GlobalPose), ('motion_state', MotionState), ('motion_plan', MotionPlan), ('object_state', ObjectState),
        ('manual_cmd', VehicleCommand), ('mode_request', UInt8), ('control_status_fixture', ControlStatus)]}
    legacy_request = node.create_publisher(UInt8, '/control/tracking_mode_request', 10)
    active = dict(state=True, motion=True, speed_valid=True, motion_age=0, motion_frame='base_link',
                  plan=True, state_age=0, plan_age=0, path_age=0,
                  localization=True, plan_valid=True, stop=False, path='normal', y=0.0,
                  state_x=1.0, state_y=-1.0, target_speed=1.0, status_frame='map',
                  pose_stamp='good', motion_stamp='good', plan_stamp='good',
                  path_stamp='good', control_stamp='good')

    def corrupt_stamp(stamp, kind):
        if kind == 'negative': stamp.sec = -1
        if kind == 'noncanonical':
            # Equivalent total time, malformed wire representation: must not normalize.
            stamp.sec -= 1
            stamp.nanosec += 1000000000

    def publish():
        stamp = node.get_clock().now().to_msg()
        s = GlobalPose(x_m=active['state_x'], y_m=active['state_y'], yaw_rad=0., valid=active['localization'])
        m = MotionState(speed_mps=1., speed_valid=active['speed_valid'])
        m.header.frame_id = active['motion_frame']; m.header.stamp = copy.deepcopy(stamp)
        m.header.stamp.sec -= active['motion_age']
        s.header.frame_id = active['status_frame']; s.header.stamp = copy.deepcopy(stamp)
        s.header.stamp.sec -= active['state_age']
        path = PathMsg(); path.header.frame_id = 'map'; path.header.stamp = copy.deepcopy(stamp)
        path.header.stamp.sec -= active['path_age']
        for x in ([i/10.0 for i in range(201)] if active['path']=='dense' else [0.,20.]):
            p = PoseStamped(); p.header.frame_id = 'map'; p.pose.position.x = x
            p.pose.position.y = active['y']; p.pose.orientation.w = 1.
            path.poses.append(p)
        if active['path'] == 'empty': path = PathMsg()
        if active['path'] == 'one': path.poses = path.poses[:1]
        if active['path'] == 'nan': path.poses[1].pose.position.y = float('nan')
        if active['path'] == 'inf': path.poses[1].pose.position.x = float('inf')
        if active['path'] == 'wrong_frame': path.header.frame_id = 'other'
        plan = MotionPlan(reference_path=path, target_speed_mps=active['target_speed'], stop_request=active['stop'],
                          valid=active['plan_valid'], reason='TEST_ONLY')
        plan.header.stamp = copy.deepcopy(stamp); plan.header.frame_id = 'map'
        plan.header.stamp.sec -= active['plan_age']
        for key, sample in [('pose_stamp',s), ('motion_stamp',m),
                            ('plan_stamp',plan), ('path_stamp',plan.reference_path)]:
            corrupt_stamp(sample.header.stamp, active[key])
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
        if 'status' in values:
            status = copy.deepcopy(values['status'])
            corrupt_stamp(status.header.stamp, active['control_stamp'])
            pubs['control_status_fixture'].publish(status)

    timer = node.create_timer(0.05, publish)
    default = Stack([command('motion_control_node')[0], '--ros-args',
                     '-r','__ns:=/control', '--params-file', str(ROOT/'config/vehicle.yaml')], 'tracking_default')
    try:
        wait_for(node, lambda: 'auto_cmd' in values and values['auto_cmd'].reason == 'PID_NOT_CONFIGURED' and
                 not values['auto_cmd'].valid and values['auto_cmd'].target_brake,
                 'default PID with healthy fixtures fails closed until configured')
        assert not node.get_publishers_info_by_topic('/control/tracking_status')
    finally:
        default.close()
    for mode in ('NONE', 'INVALID', 'ADAPTIVE_PURE_PURSUIT'):
        result = subprocess.run(command('motion_control_node', 'tracking_mode:=' + mode),
                                capture_output=True, text=True, timeout=8)
        assert result.returncode != 0 and 'tracking_mode must be' in result.stdout + result.stderr
    print('PASS: removed ADAPTIVE_PURE_PURSUIT and invalid startup modes rejected', flush=True)
    config = yaml.safe_load((ROOT/'config/operation.yaml').read_text())
    config['/control/motion_control']['ros__parameters']['pid']['enabled'] = True
    config['/control/motion_control']['ros__parameters'].update({
        'route_progress_enabled': False,  # This fixture supplies a two-point route.
        'topics.status': '/control/control_status_fixture',
        'vehicle.limits.min_steering_deg': -math.degrees(0.5),
        'vehicle.limits.max_steering_deg': math.degrees(0.5),
        'vehicle.limits.min_speed_kph': -7.2, 'vehicle.limits.max_speed_kph': 7.2})
    control_config = yaml.safe_load((ROOT/'config/control.yaml').read_text())
    control_config['/control/motion_control']['ros__parameters']['pid'].update({
        'kp_lateral': 0.2, 'ki_lateral': 0.3, 'kd_lateral': 0.02,
        'kp_heading': 0.4, 'integral_limit_m_s': 0.5})
    control_cfg = LOG/'tracking_control_fixture.yaml'
    control_cfg.write_text(yaml.safe_dump(control_config))
    config['/control/vehicle_interface']['ros__parameters'].update({
        'operating_limits_verified': True,
        'vehicle.limits.min_steering_deg': -math.degrees(0.6),
        'vehicle.limits.max_steering_deg': math.degrees(0.6),
        'vehicle.limits.min_speed_kph': -7.2, 'vehicle.limits.max_speed_kph': 7.2,
        'command_timeout_s': 0.5})
    cfg = LOG/'tracking_fixture.yaml'; cfg.write_text(yaml.safe_dump(config))
    xml = ET.parse(ROOT/'launch/control_manager.xml')
    for elem in list(xml.getroot()):
        if elem.tag == 'node' and elem.attrib['name'] not in {
                'control_selector','vehicle_interface'}:
            xml.getroot().remove(elem)
    launch = LOG/'tracking_fixture.xml'; xml.write(launch, encoding='unicode')
    values.clear()
    stack = Stack(['ros2','launch',str(launch),'operation_file:='+str(cfg)], 'tracking_integration')
    def start_controller(mode):
        config['/control/motion_control']['ros__parameters']['tracking_mode'] = mode
        filename = LOG / ('tracking_startup_' + mode + '.yaml')
        filename.write_text(yaml.safe_dump(config))
        return Stack(command('motion_control_node') + ['--params-file', str(control_cfg),
                                                     '--params-file', str(filename)],
                     'tracking_controller_' + mode)
    controller = start_controller('PID')
    def restart_controller(mode):
        nonlocal controller
        saved_control_stamp = active['control_stamp']
        # Hold PID in stable P-only preview until DDS discovers the complete
        # command path; otherwise its single zero-I sample can be missed at startup.
        active['control_stamp'] = 'negative'
        controller.close()
        wait_for(node, lambda: ('motion_control', '/control') not in node.get_node_names_and_namespaces(),
                 'previous Motion Control stopped')
        values.pop('auto_cmd', None)
        controller = start_controller(mode)
        wait_for(node, lambda: 'auto_cmd' in values and values['auto_cmd'].valid and
                 not values['auto_cmd'].target_brake and values['protected_cmd'].valid and
                 not values['protected_cmd'].target_brake,
                 'restarted '+mode+' command path discovered before PID accumulation')
        active['control_stamp'] = saved_control_stamp

    def spin_for(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=0.02)

    def command_reason(reason):
        return 'auto_cmd' in values and values['auto_cmd'].reason == reason

    def auto_pid():
        return ('auto_cmd' in values and values['auto_cmd'].valid and
                values['auto_cmd'].reason in ('PID_ACTIVE', 'PID_PREVIEW_STATE_RESET') and
                'protected_cmd' in values and values['protected_cmd'].valid and
                values['protected_cmd'].target_speed_mps == 1.)

    def invalid():
        return ('auto_cmd' in values and not values['auto_cmd'].valid and values['auto_cmd'].target_brake and
                'protected_cmd' in values and not values['protected_cmd'].valid and values['protected_cmd'].target_brake)

    def wait_reset(label, expected=0.2):
        wait_for(node, lambda: auto_pid() and abs(values['auto_cmd'].target_steer_rad-expected)<1e-6, label)

    try:
        wait_for(node, lambda: command_reason('PID_PREVIEW_STATE_RESET'),
                 'IDLE PID preview uses no integral history')
        assert abs(values['auto_cmd'].target_steer_rad-0.2)<1e-6
        wait_for(node, lambda: pubs['mode_request'].get_subscription_count() > 0 and 'status' in values,
                 'Control Selector discovered before one-shot mode request')
        pubs['mode_request'].publish(UInt8(data=ControlStatus.AUTO))
        wait_for(node, auto_pid, 'configured default PID produces AUTO command')
        assert values['auto_cmd'].target_speed_mps == 1. and not values['auto_cmd'].target_brake
        wait_for(node, lambda: auto_pid() and values['auto_cmd'].target_steer_rad > 0.28,
                 'PID integral accumulates while AUTO is healthy')
        assert not node.get_subscriptions_info_by_topic('/control/tracking_mode_request')
        assert not node.get_publishers_info_by_topic('/control/tracking_status')
        legacy_request.publish(UInt8(data=1))
        client = node.create_client(SetParameters, '/control/motion_control/set_parameters')
        wait_for(node, client.service_is_ready, 'tracking parameter service available')
        future = client.call_async(SetParameters.Request(parameters=[
            Parameter('tracking_mode', value='PURE_PURSUIT').to_parameter_msg()]))
        wait_for(node, future.done, 'tracking startup parameter change answered')
        assert not future.result().results[0].successful
        assert values['auto_cmd'].reason in ('PID_ACTIVE', 'PID_PREVIEW_STATE_RESET')
        node.destroy_client(client)
        print('PASS: no tracking request subscription and startup mode is read-only', flush=True)
        # Path deviation gate precedes steering saturation.
        active['state_y'] = -5.
        wait_for(node, lambda: command_reason('PATH_DEVIATION_EXCEEDED') and invalid(),
                 'PID path deviation over 3 m stops AUTO')
        active['state_y'] = -1.
        wait_reset('PID recovers inside path deviation limit')
        restart_controller('PURE_PURSUIT')
        expected_pp = math.atan2(2*1.328, 3.*3.+1.)
        wait_for(node, lambda: command_reason('PURE_PURSUIT_ACTIVE') and
                 values['auto_cmd'].valid and abs(values['auto_cmd'].target_steer_rad-expected_pp)<1e-9,
                 'PURE_PURSUIT startup config produces expected steering')
        assert values['status'].mode == ControlStatus.AUTO
        active['state_y'] = 3.01
        wait_for(node, lambda: command_reason('PATH_DEVIATION_EXCEEDED') and invalid(),
                 'Pure Pursuit path deviation over 3 m stops AUTO')
        active['state_y'] = 3.0
        wait_for(node, lambda: values['auto_cmd'].valid and command_reason('PURE_PURSUIT_ACTIVE'),
                 'Pure Pursuit accepts the 3 m deviation boundary')
        active.update(state_x=19.99, state_y=-1.0, target_speed=3.0)
        for y, steering in [(-1.0,0.5),(1.0,-0.5)]:
            active['state_y'] = y
            wait_for(node, lambda: command_reason('PURE_PURSUIT_ACTIVE_LIMITED') and
                     values['auto_cmd'].valid and abs(values['auto_cmd'].target_steer_rad-steering)<1e-12 and
                     values['auto_cmd'].target_speed_mps==2.0,
                     'Pure Pursuit steering and speed obey shared vehicle limits: y='+str(y))
        active['target_speed'] = -0.1
        wait_for(node, lambda: command_reason('INVALID_OR_STALE_STATE_OR_PLAN') and invalid(),
                 'negative AUTO plan remains rejected despite signed vehicle range')
        active.update(state_x=1.0, state_y=-1.0, target_speed=1.0)
        wait_for(node, lambda: command_reason('PURE_PURSUIT_ACTIVE') and values['auto_cmd'].valid,
                 'Pure Pursuit recovers after range test')
        restart_controller('PID')
        wait_reset('PID restart starts with zero accumulated history')
        wait_for(node, lambda: auto_pid() and values['auto_cmd'].target_steer_rad>0.28,
                 'accumulate before MANUAL reset')
        pubs['mode_request'].publish(UInt8(data=ControlStatus.MANUAL))
        wait_for(node, lambda: values['status'].mode == ControlStatus.MANUAL and
                 command_reason('PID_PREVIEW_STATE_RESET') and
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
        for mode in ('PID','PURE_PURSUIT'):
            if mode == 'PURE_PURSUIT': restart_controller(mode)
            for key in ('pose_stamp','motion_stamp','plan_stamp','path_stamp'):
                for kind in ('negative','noncanonical'):
                    active[key]=kind
                    wait_for(node, invalid, mode+' malformed timestamp stops AUTO: '+key+'='+kind)
                    assert controller.process.poll() is None
                    active[key]='good'
                    wait_for(node, lambda: values['auto_cmd'].valid and
                             not values['auto_cmd'].target_brake and values['protected_cmd'].valid,
                             mode+' timestamp recovery without node restart: '+key)
        restart_controller('PID')
        wait_reset('PID restart after malformed timestamp checks')
        for kind in ('negative','noncanonical'):
            active['control_stamp']=kind
            wait_for(node, lambda: command_reason('PID_PREVIEW_STATE_RESET') and
                     values['auto_cmd'].valid and abs(values['auto_cmd'].target_steer_rad-0.2)<1e-6,
                     'malformed ControlStatus resets PID history without node exit: '+kind)
            assert controller.process.poll() is None
            active['control_stamp']='good'
            wait_reset('ControlStatus timestamp recovery resets PID history')
        active['stop']=True
        wait_for(node, lambda: command_reason('PLAN_STOP') and values['auto_cmd'].valid and
                 values['auto_cmd'].target_brake and values['auto_cmd'].target_speed_mps==0.,
                 'MotionPlan stop keeps brake/speed policy and resets tracking')
        active['path']='empty'
        wait_for(node, lambda: command_reason('PLAN_STOP') and values['auto_cmd'].valid,
                 'stop-only plan needs no nested path geometry')
        active['path']='normal'
        active['stop']=False
        wait_reset('resume after stop has zero PID history')
        wait_for(node, lambda: auto_pid() and values['auto_cmd'].target_steer_rad>0.28,
                 'accumulate before path replacement')
        active['y']=-0.25
        wait_reset('path geometry replacement resets I/D',expected=0.15)
        for mode in ('PID','PURE_PURSUIT'):
            restart_controller(mode)
            active.update(state_x=1.0, state_y=0.0, y=0.0, path='dense')
            pubs['mode_request'].publish(UInt8(data=ControlStatus.MANUAL))
            wait_for(node, lambda: values['status'].mode==ControlStatus.MANUAL,
                     mode+' MANUAL selected before goal preview')
            spin_for(0.2)  # Deliver the relayed MANUAL status before moving the synthetic pose.
            active['state_x']=19.8
            wait_for(node, lambda: command_reason('GOAL_REACHED'), mode+' goal preview')
            active['state_x']=1.0
            wait_for(node, lambda: values['auto_cmd'].valid and not values['auto_cmd'].target_brake,
                     mode+' MANUAL preview does not latch AUTO completion')
            pubs['mode_request'].publish(UInt8(data=ControlStatus.AUTO))
            wait_for(node, lambda: values['status'].mode==ControlStatus.AUTO and
                     values['protected_cmd'].valid and not values['protected_cmd'].target_brake,
                     mode+' AUTO run selected')
            spin_for(0.2)
            active['state_x']=19.8
            wait_for(node, lambda: command_reason('GOAL_REACHED') and values['protected_cmd'].valid and
                     values['protected_cmd'].target_brake, mode+' AUTO arrival completes run')

            def stays_completed(label, seconds=0.3):
                spin_for(seconds)
                assert command_reason('GOAL_REACHED') and values['auto_cmd'].valid
                assert values['auto_cmd'].target_brake and values['auto_cmd'].target_speed_mps==0.0
                assert values['auto_cmd'].target_steer_rad==0.0
                print('PASS: '+mode+' completion survives '+label,flush=True)

            active['state_x']=19.6
            stays_completed('GNSS position jitter')
            active.update(state=False,motion=False,plan=False)
            stays_completed('all required input timeouts',0.7)
            active.update(state=True,motion=True,plan=True,y=1.0,state_x=1.0)
            stays_completed('path replacement')
            pubs['mode_request'].publish(UInt8(data=ControlStatus.MANUAL))
            wait_for(node, lambda: values['status'].mode==ControlStatus.MANUAL and
                     values['protected_cmd'].target_speed_mps==0.4,
                     mode+' explicit MANUAL remains independent after completion')
            pubs['mode_request'].publish(UInt8(data=ControlStatus.AUTO))
            wait_for(node, lambda: values['status'].mode==ControlStatus.AUTO and
                     values['protected_cmd'].valid and values['protected_cmd'].target_brake,
                     mode+' returning to AUTO retains completed stop')
            stays_completed('mode changes')
            restart_controller(mode)
            wait_for(node, lambda: 'auto_cmd' in values and values['auto_cmd'].valid and not values['auto_cmd'].target_brake and
                     values['protected_cmd'].valid and not values['protected_cmd'].target_brake,
                     mode+' explicit node restart permits a new run')
            active.update(state_x=20.35,state_y=0.0,y=0.0)
            wait_for(node,lambda: not values['auto_cmd'].valid and values['auto_cmd'].target_brake and
                     not command_reason('GOAL_REACHED'),mode+' behind endpoint without approach is not completion')
            active['state_x']=19.6
            wait_for(node,lambda: values['auto_cmd'].valid and not values['auto_cmd'].target_brake and
                     values['protected_cmd'].valid and not values['protected_cmd'].target_brake,
                     mode+' approaching dense route endpoint')
            spin_for(0.3)
            active.update(state_x=20.35,state_y=0.31)
            wait_for(node,lambda: not values['auto_cmd'].valid and values['auto_cmd'].target_brake and
                     not command_reason('GOAL_REACHED'),mode+' passage outside endpoint corridor does not finish')
            active.update(state_x=19.6,state_y=0.0)
            wait_for(node,lambda: values['auto_cmd'].valid and not values['auto_cmd'].target_brake and
                     values['protected_cmd'].valid and not values['protected_cmd'].target_brake,
                     mode+' valid endpoint approach recovers')
            spin_for(0.3)
            active['state_x']=20.35
            wait_for(node,lambda: command_reason('GOAL_REACHED') and values['protected_cmd'].valid and
                     values['protected_cmd'].target_brake,mode+' skipped arrival radius completes on endpoint passage')
            active['state_x']=19.6
            stays_completed('position jitter after endpoint passage')
        print('ALL TRACKING SMOKE CHECKS PASSED',flush=True)
    finally:
        controller.close(); stack.close(); node.destroy_timer(timer); node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
