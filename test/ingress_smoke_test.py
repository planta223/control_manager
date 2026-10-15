#!/usr/bin/env python3
"""Localhost-only ingress contracts; all data below are explicit test fixtures.
No external sensor, firmware, actuator or perception package is started.
"""
import copy
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Image, PointCloud2
from nav_msgs.msg import Odometry
from control_manager.msg import LanePose, VehicleFeedback, MotionState
from smoke_test import ROOT, ENV, Stack, observe, wait_for
from state_split_smoke_test import settle, start


def rejection_tests():
    cases = [(['-p','topics.external_lane:=/control/lane_observation'], 'different topics'),
             (['-p','topics.external_lane:=/control/lane_pose'], 'different topics'),
             (['-p','topics.external_lane:=lane_pose'], 'different topics'),
             (['-p','topics.external_lane:=/test/lane',
               '-r','/test/lane:=/control/lane_pose'], 'different topics'),
             (['-p','topics.gnss:=/control/sensors/gnss_odom'], 'same topic')]
    for args, expected in cases:
        result = subprocess.run(['ros2','run','control_manager','sensor_interface_node',
                                 '--ros-args','-r','__ns:=/control']+args,
                                env=ENV, capture_output=True, text=True, timeout=10)
        assert result.returncode != 0 and expected in result.stdout+result.stderr, result
        print('PASS: resolved topic loop rejected: '+str(args), flush=True)


def run():
    assert set(LanePose.get_fields_and_field_types()) == {
        'header', 'lateral_error_m', 'heading_error_rad', 'confidence'}
    print('PASS: LanePose wire contract contains only Header and the three supplied values',flush=True)
    node = rclpy.create_node('ingress_fixture')
    values = observe(node)
    counts = {'lane': 0}
    raw = [('gnss',Odometry,'/gnss/odom','/control/sensors/gnss_odom'),
           ('imu',Imu,'/imu/data','/control/sensors/imu'),
           ('rgb',Image,'/camera/image_raw','/control/sensors/rgb'),
           ('lidar',PointCloud2,'/lidar/points','/control/sensors/lidar'),
           ('depth',Image,'/camera_dep/depth/image_raw','/control/sensors/depth')]
    pubs = {key:node.create_publisher(cls,external,qos_profile_sensor_data)
            for key,cls,external,internal in raw}
    for key,cls,external,internal in raw:
        node.create_subscription(cls,internal,lambda m,k=key:values.__setitem__('raw_'+k,m),
                                 qos_profile_sensor_data)
    node.create_subscription(LanePose,'/control/lane_observation',
                             lambda m:values.__setitem__('observation',m),10)
    lane_pub = node.create_publisher(LanePose,'/test/external_lane',qos_profile_sensor_data)
    def lane_receive(msg):
        values['lane'] = msg; counts['lane'] += 1
    node.create_subscription(LanePose,'/control/lane_pose',lane_receive,10)
    node.create_subscription(VehicleFeedback,'/control/vehicle_feedback',
                             lambda m:values.__setitem__('feedback',m),10)
    node.create_subscription(MotionState,'/test/wheel_motion_state',
                             lambda m:values.__setitem__('wheel_motion',m),10)
    state = {'lane':'good','raw':True,'lane_send':True}
    sent = {}
    def publish():
        stamp = node.get_clock().now().to_msg()
        for key,cls,external,internal in raw:
            msg = cls(); msg.header.stamp = copy.deepcopy(stamp)
            msg.header.frame_id = 'map' if key=='gnss' else 'external_'+key+'_frame'
            if key=='gnss':
                msg.child_frame_id = 'base_link'; msg.pose.pose.orientation.w = 1.
                msg.pose.pose.position.x = 2.; msg.pose.pose.position.y = 3.
                msg.pose.covariance[0] = .01; msg.pose.covariance[7] = .02; msg.pose.covariance[35] = .03
                msg.twist.twist.linear.x = 1.1; msg.twist.covariance[0] = .04
            if key=='imu':
                msg.orientation.w = 1.; msg.angular_velocity.z = .12
                msg.angular_velocity_covariance[8] = .08
            if key in ['rgb','depth']:
                msg.height=1; msg.width=2; msg.encoding='mono8'; msg.step=2; msg.data=[12,34]
            if key=='lidar':
                msg.height=1; msg.width=1; msg.point_step=4; msg.row_step=4; msg.data=[1,2,3,4]
            if state['raw']:
                sent[(key,stamp.sec,stamp.nanosec)] = msg; pubs[key].publish(msg)
        lane = LanePose(lateral_error_m=-.2,heading_error_rad=.7,confidence=.9)
        lane.header.stamp=copy.deepcopy(stamp); lane.header.frame_id='external_lane_reference'
        case=state['lane']
        if case=='negative_heading': lane.heading_error_rad=-.7
        if case=='low': lane.confidence=.1
        if case=='nan_lateral': lane.lateral_error_m=float('nan')
        if case=='inf_heading': lane.heading_error_rad=float('inf')
        if case=='nan_confidence': lane.confidence=float('nan')
        if case=='range': lane.confidence=1.2
        if case=='stale': lane.header.stamp.sec-=10
        if case=='future': lane.header.stamp.sec+=10
        if case=='zero': lane.header.stamp.sec=lane.header.stamp.nanosec=0
        if case=='negative': lane.header.stamp.sec=-1
        if case=='noncanonical': lane.header.stamp.nanosec=1000000000
        if state['lane_send']:
            sent[('lane',lane.header.stamp.sec,lane.header.stamp.nanosec)] = lane
            lane_pub.publish(lane)
    timer=node.create_timer(.05,publish)
    # No external contract is enabled implicitly by default.
    default=start('sensor_interface','ingress_default')
    try:
        wait_for(node,lambda:'feedback' in values,'default ingress announces unavailable internal feedback')
        settle(node,.7)
        assert counts['lane']==0
        external_subs=node.get_subscriptions_info_by_topic('/test/external_lane')
        assert all(s.node_name!='sensor_interface' for s in external_subs)
        fb=values['feedback']
        assert not fb.speed_valid and not fb.steering_valid
        assert fb.header.stamp.sec==fb.header.stamp.nanosec==0
        print('PASS: no guessed external Lane/feedback subscription; unavailable stamp stays zero',flush=True)
    finally: default.close()
    values.clear()
    wheel=Stack(['ros2','run','control_manager','global_state_estimator','--ros-args',
                 '-r','__ns:=/control','-r','__node:=wheel_motion_fixture',
                 '--params-file',str(ROOT/'config/vehicle.yaml'),'-p','pose_mode:=NONE','-p','motion_mode:=WHEEL',
                 '-p','topics.global_pose:=/test/wheel_global_pose',
                 '-p','topics.motion_state:=/test/wheel_motion_state'],'ingress_wheel')
    stacks=[wheel,
            start('sensor_interface','ingress_enabled','topics.external_lane:=/test/external_lane',
                  'imu_enabled:=true','depth_enabled:=true'),
            start('global_state_estimator','ingress_global','localization_reference_verified:=true'),
            start('lane_state_estimator','ingress_lane'),
            start('path_planning','ingress_planner','guidance_mode:=LANE')]
    try:
        for key,cls,external,internal in raw:
            wait_for(node,lambda k=key:'raw_'+k in values, 'external -> ingress -> internal '+key)
            sample=values['raw_'+key]; stamp=sample.header.stamp
            assert sample==sent[(key,stamp.sec,stamp.nanosec)]
            print('PASS: exact raw frame/time/payload/covariance preserved: '+key,flush=True)
        wait_for(node,lambda:'global_pose' in values and values['global_pose'].valid and
                 'motion_state' in values and values['motion_state'].speed_valid,
                 'global_state_estimator publishes independent states from internal Odometry')
        wait_for(node,lambda:'lane' in values,'external Lane -> ingress -> lane_state_estimator -> LanePose')
        lane=values['lane']; stamp=lane.header.stamp
        assert lane==sent[('lane',stamp.sec,stamp.nanosec)]
        assert (lane.lateral_error_m,lane.heading_error_rad,lane.confidence)==(-.2,.7,.9)
        observation=values['observation']; stamp=observation.header.stamp
        assert observation==sent[('lane',stamp.sec,stamp.nanosec)]
        print('PASS: Lane four fields and source frame/stamp preserved exactly',flush=True)
        wait_for(node,lambda:'guidance_status' in values and
                 values['guidance_status'].reason=='LANE_NOT_IMPLEMENTED',
                 'planner consumes ingress Lane; no path algorithm introduced')
        wait_for(node,lambda:'wheel_motion' in values and
                 values['wheel_motion'].reason=='WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED',
                 'WHEEL interface connected but estimator remains unsupported')
        assert not values['wheel_motion'].speed_valid and not values['wheel_motion'].yaw_rate_valid
        assert values['wheel_motion'].header.stamp.sec==values['wheel_motion'].header.stamp.nanosec==0
        def subscribers(topic):
            return {i.node_name for i in node.get_subscriptions_info_by_topic(topic)
                    if i.node_namespace=='/control'}
        def publishers(topic):
            return {i.node_name for i in node.get_publishers_info_by_topic(topic)
                    if i.node_namespace=='/control'}
        wait_for(node,lambda:'wheel_motion_fixture' in subscribers('/control/vehicle_feedback'),
                 'internal feedback subscriber graph discovered')
        assert publishers('/control/vehicle_feedback')=={'sensor_interface'}
        assert publishers('/control/lane_pose')=={'lane_state_estimator'}
        assert publishers('/control/lane_observation')=={'sensor_interface'}
        assert subscribers('/control/lane_observation')=={'lane_state_estimator'}
        assert subscribers('/test/external_lane')=={'sensor_interface'}
        assert subscribers('/control/lane_pose')=={'path_planning'}
        for topic in ['/control/lane_observation','/control/lane_pose']:
            assert {i.topic_type for i in node.get_publishers_info_by_topic(topic)}=={
                'control_manager/msg/LanePose'}
        print('PASS: observation and final pose topics both use control_manager/msg/LanePose',flush=True)
        assert subscribers('/gnss/odom')=={'sensor_interface'}
        assert {'global_state_estimator'}<=subscribers('/control/sensors/gnss_odom')
        assert subscribers('/imu/data')=={'sensor_interface'}
        for topic in ['/control/global_path','/control/mode_request','/control/guidance_mode_request',
                      '/control/tracking_mode_request','/control/manual_cmd','/control/estop']:
            assert 'sensor_interface' not in subscribers(topic)
            assert 'sensor_interface' not in publishers(topic)
        assert 'path_planning' in subscribers('/control/global_path')
        print('PASS: sensor/Lane/feedback ownership and map/operator/safety exclusions',flush=True)
        for case,heading in [('negative_heading',-.7),('good',.7)]:
            state['lane']=case
            wait_for(node,lambda:values['lane'].heading_error_rad==heading,
                     'Lane heading radians preserved without sign conversion: '+str(heading))
            lane=values['lane']; stamp=lane.header.stamp
            assert lane==sent[('lane',stamp.sec,stamp.nanosec)]
        state['lane']='low'
        wait_for(node,lambda:values['guidance_status'].reason=='LANE_CONFIDENCE_LOW',
                 'confidence policy stays in planning; ingress forwards low finite confidence')
        for case in ['nan_lateral','inf_heading','nan_confidence','range',
                     'stale','future','zero','negative','noncanonical']:
            state['lane']=case
            settle(node,.2); before=counts['lane']
            settle(node,.7)
            assert counts['lane']==before,case+' was forwarded'
            wait_for(node,lambda:values['guidance_status'].reason=='LANE_POSE_UNAVAILABLE',
                     'reject malformed Lane without renewing last accepted timestamp: '+case)
            state['lane']='good'
            wait_for(node,lambda:values['guidance_status'].reason=='LANE_NOT_IMPLEMENTED',
                     'Lane recovers after '+case)
        state['lane_send']=False
        wait_for(node,lambda:values['guidance_status'].reason=='LANE_POSE_UNAVAILABLE',
                 'Lane source silence expires downstream, no synthetic heartbeat')
        state['raw']=False
        wait_for(node,lambda:not values['global_pose'].valid and not values['motion_state'].speed_valid,
                 'raw source silence still stops both GNSS estimator outputs')
        print('ALL INGRESS SMOKE CHECKS PASSED',flush=True)
    finally:
        for stack in stacks: stack.close()
        node.destroy_timer(timer); node.destroy_node()


if __name__=='__main__':
    rclpy.init()
    try: rejection_tests(); run()
    finally: rclpy.shutdown()
