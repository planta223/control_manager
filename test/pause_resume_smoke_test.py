#!/usr/bin/env python3
"""AUTO pause/resume through the full GNSS-only pipeline; no hardware or UDP."""
import copy
import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from control_manager.msg import PauseRequest, ControlStatus
from smoke_test import ROOT, LOG, Stack, observe, wait_for


def run():
    node = rclpy.create_node('pause_resume_fixture')
    values = observe(node)
    node.create_subscription(Float32MultiArray, '/control/actuator_raw',
                             lambda m: values.update(raw=m), 10)
    odom_pub = node.create_publisher(Odometry, '/gnss/odom', qos_profile_sensor_data)
    path_pub = node.create_publisher(PathMsg, '/control/global_path', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    request_pub = node.create_publisher(PauseRequest, '/control/pause_request', 10)
    route = [(0.,-10.), (0.,-5.), (0.,0.), (0.,5.), (-5.,5.), (-10.,5.),
             (-10.,15.), (10.,15.), (10.,5.), (5.,5.), (0.,0.), (-5.,-5.), (-10.,-5.)]
    state = dict(x=0., y=-10., yaw=math.pi/2, yaw_variance=.02, variance=.01,
                 send=True, repeat_resume=False, speed=1.)

    def request(pause, kind='good'):
        m = PauseRequest(pause=pause)
        m.header.stamp = node.get_clock().now().to_msg()
        if kind == 'zero': m.header.stamp.sec = m.header.stamp.nanosec = 0
        elif kind == 'stale': m.header.stamp.sec -= 5
        elif kind == 'future': m.header.stamp.sec += 5
        elif kind == 'negative': m.header.stamp.sec = -1
        elif kind == 'noncanonical':
            m.header.stamp.sec -= 1; m.header.stamp.nanosec += 1_000_000_000
        elif kind == 'frame': m.header.frame_id = 'wrong'
        request_pub.publish(m)
        return m

    def publish():
        if state['repeat_resume']: request(False)
        if not state['send']: return
        stamp = node.get_clock().now().to_msg()
        odom = Odometry(); odom.header.stamp = stamp
        odom.header.frame_id = 'map'; odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = state['x']; odom.pose.pose.position.y = state['y']
        odom.pose.pose.orientation.z = math.sin(state['yaw']/2)
        odom.pose.pose.orientation.w = math.cos(state['yaw']/2)
        odom.pose.covariance[0] = odom.pose.covariance[7] = state['variance']
        odom.pose.covariance[35] = state['yaw_variance']
        odom.twist.twist.linear.x = state['speed']; odom.twist.covariance[0] = .01
        odom_pub.publish(odom)
        path = PathMsg(); path.header.frame_id = 'map'; path.header.stamp = stamp
        for x, y in route:
            p = PoseStamped(); p.pose.position.x = x; p.pose.position.y = y
            p.pose.orientation.w = 1.; path.poses.append(p)
        path_pub.publish(path)

    timer = node.create_timer(.05, publish)
    operation = yaml.safe_load((ROOT/'config/modes/test.yaml').read_text())
    operation['/control/global_path_provider'] = {'ros__parameters': {'map_file': ''}}
    operation['/control/motion_planning']['ros__parameters']['cruise_speed_mps'] = 1.
    operation['/control/vehicle_interface']['ros__parameters']['dry_run'] = False  # Isolated ROS only; no trans.
    operation['/control/motion_control']['ros__parameters']['route_progress_enabled'] = True
    control = yaml.safe_load((ROOT/'config/control.yaml').read_text())
    control['/control/motion_control']['ros__parameters'].update(
        heading_bootstrap_timeout_s=.9,
        stopped_heading=dict(enabled=True, stop_speed_mps=.1, reacquire_speed_mps=.7,
                             max_distance_m=1., max_reacquire_time_s=.9),
        route_progress=dict(expected_points=13, front_end_index=4, rear_start_index=9,
                            initial_search_m=6., search_back_m=1., search_forward_m=8.))
    op_file = LOG/'pause_resume_operation.yaml'; op_file.write_text(yaml.safe_dump(operation))
    ctrl_file = LOG/'pause_resume_control.yaml'; ctrl_file.write_text(yaml.safe_dump(control))
    stack = Stack(['ros2','launch','control_manager','control_manager.xml',
                   'operation_file:='+str(op_file),'control_file:='+str(ctrl_file)], 'pause_resume')

    def settle(seconds=.18):
        end = time.monotonic()+seconds
        while time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.02)

    def reason(text): return 'auto_cmd' in values and values['auto_cmd'].reason == text

    def driving():
        return (reason('PURE_PURSUIT_ACTIVE') or reason('PURE_PURSUIT_ACTIVE_LIMITED')) and values['auto_cmd'].valid and \
            values['selected_cmd'].valid and not values['selected_cmd'].target_brake

    def paused():
        return 'status' in values and values['status'].paused and \
            values['status'].fsm_state == ControlStatus.STOPPED and \
            values['selected_cmd'].reason == 'AUTO_PAUSED' and \
            'raw' in values and list(values['raw'].data) == [0.,0.,1.]

    def pause():
        state['speed'] = 0.
        request(True)
        wait_for(node, paused, 'pause latches neutral/brake through Vehicle Interface')
        wait_for(node, lambda: reason('AUTO_PAUSED'), 'Motion Control observes pause')

    def resume():
        seq = values['status'].resume_sequence
        request(False)
        wait_for(node, lambda: not values['status'].paused and
                 values['status'].resume_sequence == seq+1, 'one accepted resume transition')

    try:
        wait_for(node, paused, 'AUTO starts paused, even with valid GNSS and map')
        wait_for(node, lambda: request_pub.get_subscription_count()>0, 'pause input discovered')
        for kind in ('zero','stale','future','negative','noncanonical','frame'):
            request(False, kind); settle(.12)
            assert paused() and values['status'].resume_sequence == 0, kind
        print('PASS: malformed/old/future pause requests cannot release startup pause', flush=True)
        # A late resume cannot override a newer accepted pause, even while still fresh.
        old = PauseRequest(pause=False); old.header.stamp = node.get_clock().now().to_msg()
        settle(.04); request(True); settle(.08); request_pub.publish(old); settle(.12)
        assert paused() and values['status'].resume_sequence == 0
        print('PASS: request ordering prevents a delayed resume from overriding a pause', flush=True)
        resume(); wait_for(node, driving, 'valid heading resumes directly into Pure Pursuit')
        pause(); settle(1.1)
        assert paused() and reason('AUTO_PAUSED')
        state['yaw_variance'] = 1e12
        wait_for(node, lambda: values['global_pose'].heading_unavailable and
                 not values['global_pose'].initial_heading_unavailable and
                 values['motion_plan'].valid and values['reference_path'].poses,
                 'long-stop heading loss retains a position-only GPS path')
        resume()
        wait_for(node, lambda: values['auto_cmd'].reason.startswith('HEADING_HOLD_ACTIVE:') and
                 values['selected_cmd'].valid and not values['selected_cmd'].target_brake,
                 'explicit resume uses the yaw retained at the measured stop')
        state.update(repeat_resume=True, speed=.2)
        seq = values['status'].resume_sequence
        wait_for(node, lambda: reason('HEADING_HOLD_REACQUIRE_TIMEOUT') and
                 values['selected_cmd'].target_brake and list(values['raw'].data) == [0.,0.,1.],
                 'repeated requests cannot extend the remembered-yaw movement budget')
        assert values['status'].resume_sequence == seq
        state['repeat_resume'] = False
        pause(); resume(); settle(.2)
        assert not values['auto_cmd'].valid
        state.update(yaw_variance=.02, speed=1.)
        wait_for(node, driving, 'fresh heading restores Pure Pursuit')
        state['yaw_variance'] = 1e12
        wait_for(node, lambda: not values['auto_cmd'].valid and values['auto_cmd'].target_brake,
                 'unrequested heading loss still stops normal driving')
        pause(); state['variance'] = 4.5001
        resume(); settle(.3)
        assert not values['global_pose'].position_valid and not values['auto_cmd'].valid
        print('PASS: resume never bypasses the 3 m position quality gate', flush=True)
        state.update(variance=.01, x=3.01, yaw_variance=.02, speed=1.)
        wait_for(node, lambda: reason('PATH_DEVIATION_EXCEEDED'), 'resume keeps the 3 m path deviation stop')
        state.update(x=0., yaw_variance=.02)
        wait_for(node, driving, 'valid inputs recover without resetting route progress')
        state['send'] = False
        wait_for(node, lambda: not values['auto_cmd'].valid and values['auto_cmd'].target_brake,
                 'stale GNSS still stops after a resume')
        state['send'] = True; wait_for(node, driving, 'fresh GNSS recovery')
        for i, ((ax,ay),(bx,by)) in enumerate(zip(route, route[1:])):
            dx,dy=bx-ax,by-ay; length=math.hypot(dx,dy); steps=math.ceil(length/2.)
            for step in range(1,steps+1):
                f=step/steps; yaw=math.atan2(dy,dx)
                if step==steps and i+2<len(route):
                    nx,ny=route[i+2][0]-bx,route[i+2][1]-by; nl=math.hypot(nx,ny)
                    yaw=math.atan2(dy/length+ny/nl,dx/length+nx/nl)
                state.update(x=ax+f*dx,y=ay+f*dy,yaw=yaw)
                settle(.22)
                if i+2==len(route) and step==steps: break
                assert driving(), (i,step,values.get('auto_cmd'))
            if i==9:
                pause(); state.update(x=-5.,y=-5.,yaw_variance=1e12); settle(1.1)
                assert paused() and reason('AUTO_PAUSED')
                state.update(x=bx,y=by)
                resume()
                wait_for(node, lambda: not values['auto_cmd'].valid,
                         'moving the stopped fixture invalidates yaw memory without advancing route progress')
                state.update(yaw_variance=.02, speed=1.)
                wait_for(node, driving, 'same branch resumes at the second crossing without node restart')
        wait_for(node, lambda: reason('GOAL_REACHED') and values['auto_cmd'].target_brake,
                 'true endpoint completes the run')
        request(True); wait_for(node, paused, 'completed run can still be paused')
        state.update(x=-9.5,y=-5.,yaw_variance=1e12); resume(); settle(.3)
        assert reason('GOAL_REACHED') and values['selected_cmd'].target_brake
        print('PASS: pause/resume never clears endpoint completion', flush=True)
        print('ALL PAUSE/RESUME CHECKS PASSED', flush=True)
    finally:
        stack.close(); node.destroy_timer(timer); node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
