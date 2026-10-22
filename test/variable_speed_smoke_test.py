#!/usr/bin/env python3
"""Synthetic path -> variable MotionPlan -> PP -> slew -> RPM. Never starts trans."""
import copy
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Float32MultiArray
from control_manager.msg import GlobalPose, MotionState, MotionPlan, VehicleCommand
from operation_smoke_test import command
from smoke_test import LOG, Stack, wait_for


def run():
    node=rclpy.create_node('variable_speed_fixture')
    points=[(i*.1,0.0) for i in range(501)]
    points += [(50+10*math.sin(i*math.pi/400),10*(1-math.cos(i*math.pi/400))) for i in range(1,201)]
    points += [(60.0,10+i*.1) for i in range(1,501)]
    state=dict(index=0,pose=True)
    values={}; selected=[]
    for topic,cls in [('motion_plan',MotionPlan),('selected_cmd',VehicleCommand),('actuator_raw',Float32MultiArray)]:
        node.create_subscription(cls,'/control/'+topic,lambda m,k=topic: values.update({k:m}),100)
    node.create_subscription(VehicleCommand,'/control/selected_cmd',selected.append,100)
    pubs={key:node.create_publisher(cls,'/control/'+key,10) for key,cls in
          [('reference_path',PathMsg),('global_pose',GlobalPose),('motion_state',MotionState)]}
    def publish():
        stamp=node.get_clock().now().to_msg(); i=state['index']
        path=PathMsg(); path.header.frame_id='map'; path.header.stamp=copy.deepcopy(stamp)
        for x,y in points:
            p=PoseStamped(); p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.w=1.0
            path.poses.append(p)
        pose=GlobalPose(x_m=points[i][0],y_m=points[i][1],valid=True,position_valid=True)
        j=min(i,len(points)-2)
        pose.yaw_rad=math.atan2(points[j+1][1]-points[j][1],points[j+1][0]-points[j][0])
        pose.header.frame_id='map'; pose.header.stamp=copy.deepcopy(stamp)
        motion=MotionState(speed_mps=3.0,speed_valid=True)
        motion.header.frame_id='base_link'; motion.header.stamp=copy.deepcopy(stamp)
        pubs['reference_path'].publish(path); pubs['motion_state'].publish(motion)
        if state['pose']: pubs['global_pose'].publish(pose)
    node.create_timer(.05,publish)
    stacks=[Stack(command('motion_planning_node','speed_profile.enabled:=true',
                'speed_profile.max_lateral_accel_mps2:=1.5','speed_profile.planned_decel_mps2:=1.0',
                'cruise_speed_mps:=5.555555555555556'),'variable_planner'),
            Stack(command('motion_control_node'),'variable_pp'),
            Stack(command('control_selector_node','start_paused:=false',
                'max_accel_mps2:=1.388888888888889','max_decel_mps2:=1.388888888888889'),'variable_selector'),
            Stack(command('vehicle_interface_node','dry_run:=false'),'variable_interface')]
    def planned():
        return values.get('motion_plan')
    try:
        wait_for(node,lambda:planned() and planned().valid and abs(planned().target_speed_mps-20/3.6)<1e-9,
                 'straight targets configured top speed')
        wait_for(node,lambda:values.get('selected_cmd') and values['selected_cmd'].target_speed_mps>5.5,
                 'straight acceleration reaches ceiling')
        state['index']=480
        wait_for(node,lambda:planned().valid and 0<planned().target_speed_mps<4.8,
                 'planning slows on straight before curve entry')
        state['index']=600; selected.clear()
        wait_for(node,lambda:planned().valid and planned().target_speed_mps<4.0,
                 'curve target respects lateral acceleration budget')
        wait_for(node,lambda:selected and not selected[-1].target_brake and selected[-1].target_speed_mps<4.0,
                 'normal deceleration reaches curve target')
        moving=[m for m in selected if m.valid and not m.target_brake]
        assert any(m.target_speed_mps>4.0 for m in moving)
        assert all(a.target_speed_mps-b.target_speed_mps<0.15 for a,b in zip(moving,moving[1:]))
        wait_for(node,lambda:abs(values['selected_cmd'].target_speed_mps-planned().target_speed_mps)<1e-8,
                 'curve command settles to planned speed')
        target=values['selected_cmd'].target_speed_mps
        wait_for(node,lambda:values.get('actuator_raw') and abs(values['actuator_raw'].data[1]-
                 target*60/(math.pi*.47)/891.387)<1e-4,'variable command reaches actual-RPM conversion')
        state['index']=900
        wait_for(node,lambda:planned().valid and planned().target_speed_mps>5.5,
                 'curve exit permits acceleration again')
        wait_for(node,lambda:values['selected_cmd'].target_speed_mps>4.5,'selected speed increases after curve')
        state['pose']=False
        wait_for(node,lambda:not planned().valid and values['selected_cmd'].target_brake,
                 'lost GNSS position immediately stops variable control')
        state['pose']=True
        wait_for(node,lambda:planned().valid,'fresh position recovers planning')
        state['index']=1180
        wait_for(node,lambda:planned().valid and planned().target_speed_mps<2.0,'end distance lowers target')
        state['index']=1200
        wait_for(node,lambda:planned().stop_request and values['selected_cmd'].target_brake,
                 'end stop bypasses normal deceleration ramp')
        print('PASS: automatic straight/curve speed changes and final RPM path',flush=True)
    finally:
        for stack in reversed(stacks): stack.close()
        node.destroy_node()


if __name__=='__main__':
    for directory in (LOG,LOG/'tmp',LOG/'ros'): directory.mkdir(parents=True,exist_ok=True)
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
