#!/usr/bin/env python3
"""Three-phase route association through a twice-visited crossing; no hardware."""
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from control_manager.msg import GlobalPose, MotionState, MotionPlan, ControlStatus
from smoke_test import Stack, observe, wait_for
from operation_smoke_test import command


def run():
    node=rclpy.create_node('route_progress_fixture'); values=observe(node)
    route=[(0.,-10.),(0.,-5.),(0.,0.),(0.,5.),(-5.,5.),(-10.,5.),
           (-10.,15.),(10.,15.),(10.,5.),(5.,5.),(0.,0.),(-5.,-5.),(-10.,-5.)]
    state=dict(x=0.,y=0.,yaw=-3*math.pi/4,send=True,changed=False,mode=ControlStatus.AUTO)
    pubs={key:node.create_publisher(cls,'/control/'+key,10) for key,cls in [
        ('global_pose',GlobalPose),('motion_state',MotionState),('motion_plan',MotionPlan),('status',ControlStatus)]}

    def publish():
        stamp=node.get_clock().now().to_msg()
        status=ControlStatus(mode=state['mode'],fsm_state=ControlStatus.DRIVING)
        status.header.frame_id='base_link';status.header.stamp=stamp;pubs['status'].publish(status)
        if not state['send']: return
        pose=GlobalPose(x_m=state['x'],y_m=state['y'],yaw_rad=state['yaw'],position_valid=True,valid=True)
        pose.header.frame_id='map';pose.header.stamp=stamp;pubs['global_pose'].publish(pose)
        motion=MotionState(speed_mps=1.,speed_valid=True)
        motion.header.frame_id='base_link';motion.header.stamp=stamp;pubs['motion_state'].publish(motion)
        path=PathMsg();path.header.frame_id='map';path.header.stamp=stamp
        for i,(x,y) in enumerate(route):
            point=PoseStamped();point.pose.position.x=x-(0.2 if state['changed'] and i==len(route)-1 else 0.)
            point.pose.position.y=y;point.pose.orientation.w=1.;path.poses.append(point)
        plan=MotionPlan(reference_path=path,target_speed_mps=1.,valid=True)
        plan.header.frame_id='map';plan.header.stamp=stamp;pubs['motion_plan'].publish(plan)

    timer=node.create_timer(0.05,publish)
    stack=Stack(command('motion_control_node','tracking_mode:=PURE_PURSUIT','route_progress_enabled:=true',
        'route_progress.expected_points:=13','route_progress.front_end_index:=4','route_progress.rear_start_index:=9',
        'route_progress.initial_search_m:=6.0','route_progress.search_back_m:=1.0','route_progress.search_forward_m:=8.0'),
        'route_progress')

    def reason(value): return 'auto_cmd' in values and values['auto_cmd'].reason==value

    def settle(seconds=0.18):
        until=time.monotonic()+seconds
        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=0.02)

    def driving():
        return (reason('PURE_PURSUIT_ACTIVE') or reason('PURE_PURSUIT_ACTIVE_LIMITED')) and \
            values['auto_cmd'].valid and not values['auto_cmd'].target_brake

    try:
        wait_for(node,lambda: reason('PATH_DEVIATION_EXCEEDED') and not values['auto_cmd'].valid,
                 'initial location at a later crossing cannot initialize a run')
        state.update(x=0.,y=-10.,yaw=math.pi/2)
        wait_for(node,driving,'known route start initializes FRONT')
        for i,((ax,ay),(bx,by)) in enumerate(zip(route,route[1:])):
            dx,dy=bx-ax,by-ay;length=math.hypot(dx,dy);steps=math.ceil(length/2.)
            for n in range(1,steps+1):
                fraction=n/steps;yaw=math.atan2(dy,dx)
                if n==steps and i+2<len(route):
                    nx,ny=route[i+2][0]-bx,route[i+2][1]-by;nl=math.hypot(nx,ny)
                    yaw=math.atan2(dy/length+ny/nl,dx/length+nx/nl)
                state.update(x=ax+fraction*dx,y=ay+fraction*dy,yaw=yaw)
                if i+2==len(route) and n==steps:
                    # Skip the arrival circle; completion must use the observed approach and end-plane passage.
                    state.update(x=bx+0.35*dx/length,y=by+0.35*dy/length)
                settle()
                if i+2==len(route) and n==steps: break
                assert driving(), (i,n,values.get('auto_cmd'))
                if n==steps and i in (3,8):
                    print('PASS: phase boundary is not treated as a goal: '+str(i),flush=True)
                if n==steps and i in (1,9):
                    state.update(x=0.03,y=-0.01);settle();assert driving()
                    print('PASS: correct branch at crossing visit '+str(1 if i==1 else 2),flush=True)
                    state.update(x=bx,y=by)
                if n==steps and i==9:
                    state['send']=False;settle(0.7)
                    assert reason('INVALID_OR_STALE_STATE_OR_PLAN') and not values['auto_cmd'].valid
                    state['send']=True
                    wait_for(node,driving,'input recovery retains REAR progress at the shared crossing')
                    state['mode']=ControlStatus.MANUAL;settle();assert driving()
                    state['mode']=ControlStatus.AUTO;settle();assert driving()
                    state['changed']=True
                    wait_for(node,lambda: reason('ROUTE_CHANGED_RESTART_REQUIRED'),
                             'changed geometry cannot silently restart progress')
                    state['changed']=False
                    wait_for(node,driving,'same route recovery retains progress')
        wait_for(node,lambda: reason('GOAL_REACHED') and values['auto_cmd'].valid and
                 values['auto_cmd'].target_brake,'real endpoint passage completes AUTO after an observed approach')
        state.update(x=-9.5,y=-5.);settle()
        assert reason('GOAL_REACHED') and values['auto_cmd'].target_speed_mps==0.
        print('ALL ROUTE PROGRESS SMOKE CHECKS PASSED',flush=True)
    finally:
        stack.close();node.destroy_timer(timer);node.destroy_node()


if __name__=='__main__':
    rclpy.init()
    try: run()
    finally:rclpy.shutdown()
