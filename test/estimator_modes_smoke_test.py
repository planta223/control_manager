#!/usr/bin/env python3
"""Independent estimator modes and lane admission. Internal samples are TEST fixtures.
Uses confined logs and localhost DDS domain 179; no hardware or actuator sender.
"""
import copy
import subprocess
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from control_manager.msg import GlobalPose, MotionState, VehicleFeedback, LanePose
from smoke_test import ENV, observe, wait_for
from state_split_smoke_test import start, settle


def run_mode(pose_mode, motion_mode):
    node = rclpy.create_node('independent_estimator_mode_fixture')
    values = observe(node)
    pubs = {key: node.create_publisher(cls, topic, qos) for key,cls,topic,qos in [
        ('gnss',Odometry,'/control/sensors/gnss_odom',qos_profile_sensor_data),
        ('imu',Imu,'/control/sensors/imu',qos_profile_sensor_data),
        ('feedback',VehicleFeedback,'/control/vehicle_feedback',10)]}
    needs_gnss = pose_mode != 'NONE' or motion_mode in {'GNSS','GNSS_IMU'}
    needs_imu = pose_mode == 'GNSS_IMU' or motion_mode == 'GNSS_IMU'
    needs_feedback = motion_mode == 'WHEEL'
    state = {'gnss': needs_gnss, 'imu': needs_imu, 'feedback': needs_feedback, 'poison': False}
    def publish():
        stamp = node.get_clock().now().to_msg()
        if state['gnss']:
            o = Odometry(); o.header.stamp=stamp; o.header.frame_id='map'; o.child_frame_id='base_link'
            o.pose.pose.orientation.w=1.; o.pose.covariance[0]=o.pose.covariance[7]=.01
            o.pose.covariance[35]=.02; o.twist.twist.linear.x=1.3; o.twist.covariance[0]=.02
            pubs['gnss'].publish(o)
        if state['imu']:
            imu = Imu(); imu.header.stamp=copy.deepcopy(stamp); imu.header.frame_id='unverified_imu_frame'
            imu.orientation.w=1.; imu.angular_velocity.z=.2
            if state['poison']:
                imu.header.stamp.sec-=10; imu.angular_velocity.z=float('nan')
            pubs['imu'].publish(imu)
        if state['feedback']:
            fb=VehicleFeedback(speed_mps=1.,steering_angle_rad=.1,speed_valid=True,steering_valid=True)
            fb.header.frame_id='base_link'; fb.header.stamp=copy.deepcopy(stamp)
            if state['poison']:
                fb.header.stamp.sec-=10; fb.speed_valid=False; fb.steering_valid=False
                fb.speed_mps=float('nan'); fb.steering_angle_rad=float('inf')
            pubs['feedback'].publish(fb)
    timer=node.create_timer(.05,publish)
    stack=start('global_state_estimator','modes_'+pose_mode+'_'+motion_mode,
                'pose_mode:='+pose_mode,'motion_mode:='+motion_mode,'localization_reference_verified:=true')
    expected_pose={'GNSS':'GNSS_POSE_VALID','GNSS_IMU':'GNSS_IMU_NOT_IMPLEMENTED','NONE':'POSE_DISABLED'}[pose_mode]
    expected_motion={'GNSS':'GNSS_SPEED_VALID','GNSS_IMU':'GNSS_IMU_NOT_IMPLEMENTED',
                     'WHEEL':'WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED','NONE':'MOTION_DISABLED'}[motion_mode]
    def match():
        return ('global_pose' in values and 'motion_state' in values and
                values['global_pose'].reason==expected_pose and values['motion_state'].reason==expected_motion)
    try:
        wait_for(node,match,'independent mode combination: pose='+pose_mode+' motion='+motion_mode)
        pose,motion=values['global_pose'],values['motion_state']
        assert pose.valid==(pose_mode=='GNSS') and motion.speed_valid==(motion_mode=='GNSS')
        assert not motion.yaw_rate_valid
        if pose_mode!='GNSS': assert pose.header.stamp.sec==pose.header.stamp.nanosec==0
        if motion_mode!='GNSS': assert motion.header.stamp.sec==motion.header.stamp.nanosec==0
        if motion_mode=='GNSS': assert motion.speed_mps==1.3
        def subscribed(topic):
            return any(i.node_name=='global_state_estimator' and i.node_namespace=='/control'
                       for i in node.get_subscriptions_info_by_topic(topic))
        wait_for(node,lambda: subscribed('/control/sensors/gnss_odom')==needs_gnss and
                 subscribed('/control/sensors/imu')==needs_imu and
                 subscribed('/control/vehicle_feedback')==needs_feedback,
                 'only selected source dependencies subscribed: '+pose_mode+'/'+motion_mode)
        assert not subscribed('/gnss/odom') and not subscribed('/imu/data')
        # Poison unused sensor/feedback inputs. No output may inherit their invalidity.
        state.update(imu=True,feedback=True,poison=True)
        settle(node,.7)
        assert match()
        assert values['global_pose'].valid==(pose_mode=='GNSS')
        assert values['motion_state'].speed_valid==(motion_mode=='GNSS')
        print('PASS: unrelated absent/stale/invalid inputs cannot contaminate '+pose_mode+'/'+motion_mode,flush=True)
        if pose_mode=='NONE' and motion_mode=='WHEEL':
            assert not state['gnss'] and not values['global_pose'].valid
            assert values['global_pose'].reason=='POSE_DISABLED'
            assert values['motion_state'].reason=='WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED'
            print('PASS: No-GNSS LANE target configuration needs no GNSS; WHEEL is explicitly unsupported',flush=True)
        if pose_mode=='GNSS' and motion_mode in {'WHEEL','NONE'}:
            state['gnss']=False
            wait_for(node,lambda:not values['global_pose'].valid and
                     values['motion_state'].reason==expected_motion,
                     'pose source loss does not change selected motion mode reason')
    finally:
        stack.close(); node.destroy_timer(timer); node.destroy_node()


def lane_admission():
    node=rclpy.create_node('lane_admission_fixture')
    received=[]
    node.create_subscription(LanePose,'/control/lane_pose',lambda m:received.append(m),10)
    pub=node.create_publisher(LanePose,'/control/lane_observation',10)
    active={'kind':'good'}
    def publish():
        if active['kind']=='silent': return
        s=LanePose(lateral_error_m=-.3,heading_error_rad=.8,confidence=.9)
        s.header.stamp=node.get_clock().now().to_msg(); s.header.frame_id='lane_reference'
        if active['kind']=='stale': s.header.stamp.sec-=10
        if active['kind']=='negative_time': s.header.stamp.sec=-1
        if active['kind']=='nan': s.heading_error_rad=float('nan')
        if active['kind']=='range': s.confidence=-.1
        if active['kind']=='low': s.confidence=.1
        if active['kind']=='negative_heading': s.heading_error_rad=-.8
        pub.publish(s)
    timer=node.create_timer(.05,publish)
    stack=start('lane_state_estimator','lane_admission')
    try:
        wait_for(node,lambda:len(received)>0,'lane_state_estimator accepts four-field internal observation')
        assert received[-1].lateral_error_m==-.3 and received[-1].heading_error_rad==.8
        assert received[-1].header.frame_id=='lane_reference'
        active['kind']='negative_heading'
        wait_for(node,lambda:received[-1].heading_error_rad==-.8,
                 'lane estimator preserves negative heading radians')
        active['kind']='good'
        wait_for(node,lambda:received[-1].heading_error_rad==.8,
                 'lane estimator preserves positive heading radians')
        for case in ['stale','negative_time','nan','range','silent']:
            active['kind']=case; settle(node,.2); count=len(received); settle(node,.7)
            assert len(received)==count
            print('PASS: lane_state_estimator does not forward or replay '+case,flush=True)
            active['kind']='good'
            wait_for(node,lambda:len(received)>count,'lane state recovers after '+case)
        active['kind']='low'
        wait_for(node,lambda:received[-1].confidence==.1,'lane confidence threshold remains a planner policy')
    finally:
        stack.close(); node.destroy_timer(timer); node.destroy_node()
    result=subprocess.run(['ros2','run','control_manager','lane_state_estimator','--ros-args',
                           '-r','__ns:=/control','-p','lane_enabled:=true',
                           '-p','topics.lane_observation:=lane_pose'],
                          env=ENV,capture_output=True,text=True,timeout=10)
    assert result.returncode!=0 and 'different topics' in result.stdout+result.stderr
    print('PASS: lane observation/state remap loop rejected',flush=True)


if __name__=='__main__':
    rclpy.init()
    try:
        for pose in ['GNSS','GNSS_IMU','NONE']:
            for motion in ['GNSS','GNSS_IMU','WHEEL','NONE']:
                run_mode(pose,motion)
        lane_admission()
        print('ALL ESTIMATOR MODE SMOKE CHECKS PASSED',flush=True)
    finally: rclpy.shutdown()
