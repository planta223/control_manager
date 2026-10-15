#!/usr/bin/env python3
"""Hardware-free integration fixtures. Never start external packages or actuator senders.
Run with this package installed, ROS_DOMAIN_ID=179 and localhost-only UDP transport.
All generated files/logs are confined to control_manager/log.
"""
import copy
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
import yaml
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, UInt8
from control_manager.msg import VehicleState, VehicleCommand, ObjectState, MotionPlan, ControlStatus, GuidanceStatus, LaneState, TrackingStatus

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / 'log'
ENV = dict(os.environ)
assert ENV.get('ROS_DOMAIN_ID') == '179', 'Use dedicated test ROS_DOMAIN_ID=179'
assert ENV.get('ROS_LOCALHOST_ONLY') == '1', 'Localhost isolation is required'
assert ENV.get('FASTRTPS_DEFAULT_PROFILES_FILE') == str(ROOT / 'test' / 'dds_local.xml')
ENV['ROS_LOG_DIR'] = str(LOG / 'ros')
ENV['ROS_HOME'] = str(LOG / 'ros')
ENV['TMPDIR'] = str(LOG / 'tmp')
ENV['PYTHONDONTWRITEBYTECODE'] = '1'


class Stack:
    def __init__(self, args, label):
        self.log = open(LOG / (label + '.log'), 'w')
        self.process = subprocess.Popen(args, env=ENV, stdout=self.log,
                                        stderr=subprocess.STDOUT, start_new_session=True)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        self.log.close()


def wait_for(node, condition, label, timeout=10):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            print('PASS:', label, flush=True)
            return
    raise AssertionError('Timeout: ' + label)


def observe(node):
    values = {}
    for topic, cls in [('vehicle_state', VehicleState), ('selected_cmd', VehicleCommand),
                       ('final_cmd', VehicleCommand), ('auto_cmd', VehicleCommand),
                       ('motion_plan', MotionPlan), ('status', ControlStatus),
                       ('object_state', ObjectState), ('guidance_status', GuidanceStatus),
                       ('reference_path', PathMsg), ('tracking_status', TrackingStatus)]:
        node.create_subscription(cls, '/control/' + topic,
                                 lambda msg, key=topic: values.__setitem__(key, msg), 10)
    return values


def default_stack():
    node = rclpy.create_node('default_contract_test')
    values = observe(node)
    stack = Stack(['ros2', 'launch', 'control_manager', 'control_manager.xml'], 'smoke_default')
    try:
        wait_for(node, lambda: len([n for n, ns in node.get_node_names_and_namespaces()
                                   if ns == '/control']) == 10, 'XML launches all ten nodes')
        wait_for(node, lambda: 'final_cmd' in values and 'object_state' in values and
                 'auto_cmd' in values and 'status' in values, 'custom messages delivered')
        wait_for(node, lambda: 'tracking_status' in values, 'TrackingStatus delivered')
        assert values['tracking_status'].requested_mode == TrackingStatus.PID
        assert not values['tracking_status'].valid and values['tracking_status'].actual_mode == TrackingStatus.NONE
        assert values['status'].mode == ControlStatus.IDLE
        assert not values['final_cmd'].valid and values['final_cmd'].target_brake
        assert values['final_cmd'].target_speed_mps == 0.0
        assert not values['object_state'].valid and not values['object_state'].obstacle_valid
        assert not values['auto_cmd'].valid
        print('PASS: default IDLE, unknown objects, final safe stop', flush=True)
        required = {'/control/' + s for s in ['vehicle_state', 'lane_state', 'object_state',
                    'reference_path', 'mission_limit', 'motion_plan', 'auto_cmd', 'manual_cmd',
                    'selected_cmd', 'final_cmd', 'status', 'guidance_status', 'guidance_mode_request', 'tracking_status', 'tracking_mode_request']}
        assert required <= {name for name, _ in node.get_topic_names_and_types()}
        print('PASS: required topic graph contracts', flush=True)
    finally:
        stack.close()
        node.destroy_node()


def startup_rejections():
    cases = [('vehicle_interface_node', 'dry_run:=false', 'UDP backend does not exist'),
             ('state_estimator_node', 'fusion_enable:=true', 'not implemented'),
             ('state_estimator_node', 'estimation_mode:=GNSS_IMU', 'not implemented'),
             ('path_planning_node', 'lane_correction_enabled:=true', 'not implemented'),
             ('path_planning_node', 'map_file:=unsupported.json', 'not implemented'),
             ('path_planning_node', 'guidance_mode:=INVALID', 'guidance_mode must be'),
             ('state_estimator_node', 'estimation_mode:=GNSS_IMU_WHEEL', 'not implemented'),
             ('motion_control_node', 'tracking_mode:=INVALID', 'tracking_mode must be')]
    for executable, parameter, reason in cases:
        result = subprocess.run(['ros2', 'run', 'control_manager', executable, '--ros-args', '-p',
                                 parameter], env=ENV, capture_output=True, text=True, timeout=10)
        assert result.returncode != 0 and reason in result.stdout + result.stderr, result
        print('PASS: explicit startup rejection ' + parameter, flush=True)


def integrated_fixtures():
    config = yaml.safe_load((ROOT / 'config/control_manager.yaml').read_text())
    overrides = {
        'state_estimator': {'localization_reference_verified': True},
        'path_planning': {'map_datum_verified': True},
        'mission_based_limit': {'mission_baseline_enabled': True, 'mission_max_speed_mps': 2.0},
        'motion_planning': {'cruise_speed_mps': 1.0},
        'motion_control': {'tracking_mode': 'PURE_PURSUIT'},  # Preserve legacy regression fixture.
        'safety_limit': {'operating_limits_verified': True, 'max_steer_rad': 0.7,
                         'max_speed_mps': 2.0, 'emergency_distance_m': 1.0},
    }
    for name, params in overrides.items():
        config['/control/' + name]['ros__parameters'].update(params)
    fixture_config = LOG / 'fixture_params.yaml'
    fixture_config.write_text(yaml.safe_dump(config))
    xml = ET.parse(ROOT / 'launch/control_manager.xml')
    # Replace only the unimplemented perception node with explicit test observations.
    for element in list(xml.getroot()):
        if element.tag == 'node' and element.attrib['name'] == 'object_manager':
            xml.getroot().remove(element)
    fixture_launch = LOG / 'fixture_launch.xml'
    xml.write(fixture_launch, encoding='unicode')
    node = rclpy.create_node('integration_fixture_test')
    values = observe(node)
    gnss_pub = node.create_publisher(Odometry, '/gnss/odom', qos_profile_sensor_data)
    object_pub = node.create_publisher(ObjectState, '/control/object_state', 10)
    manual_pub = node.create_publisher(VehicleCommand, '/control/manual_cmd', 10)
    mode_pub = node.create_publisher(UInt8, '/control/mode_request', 10)
    guidance_pub = node.create_publisher(UInt8, '/control/guidance_mode_request', 10)
    lane_pub = node.create_publisher(LaneState, '/control/lane_state', 10)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    path_pub = node.create_publisher(PathMsg, '/control/global_path', durable)
    estop_pub = node.create_publisher(Bool, '/control/estop', durable)
    active = {'gnss': True, 'object': True, 'manual': True,
              'gnss_stamp': 'fresh', 'lane': 'absent', 'path': 'valid'}
    manual = VehicleCommand(target_steer_rad=-0.15, target_speed_mps=0.4,
                            target_brake=False, valid=True, reason='TEST_ONLY')
    object_state = ObjectState(valid=True, obstacle_valid=True, obstacle_detected=False)
    object_state.header.frame_id = 'base_link'
    path = PathMsg()
    path.header.frame_id = 'map'
    for x in [0.0, 10.0, 20.0]:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = 1.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    def publish():
        stamp = node.get_clock().now().to_msg()
        if active['gnss']:
            odom = Odometry()
            odom.header.stamp = copy.deepcopy(stamp)
            if active['gnss_stamp'] == 'stale': odom.header.stamp.sec -= 10
            if active['gnss_stamp'] == 'future': odom.header.stamp.sec += 10
            if active['gnss_stamp'] == 'zero':
                odom.header.stamp.sec = odom.header.stamp.nanosec = 0
            odom.header.frame_id = 'map'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation.w = 1.0
            odom.pose.covariance[0] = odom.pose.covariance[7] = 0.01
            odom.pose.covariance[35] = 0.01
            odom.twist.twist.linear.x = 1.0
            odom.twist.covariance[0] = 0.01
            gnss_pub.publish(odom)
        if active['object']:
            object_state.header.stamp = stamp
            object_pub.publish(object_state)
        if active['manual']:
            manual.header.stamp = stamp
            manual.header.frame_id = 'base_link'
            manual_pub.publish(manual)
        path.header.stamp = stamp
        sent_path = copy.deepcopy(path)
        if active['path'] == 'empty': sent_path.poses = []
        if active['path'] == 'wrong_frame': sent_path.header.frame_id = 'wrong_frame'
        path_pub.publish(sent_path)
        if active['lane'] != 'absent':
            lane = LaneState(valid=active['lane'] != 'invalid', confidence=1.0)
            lane.header.stamp = copy.deepcopy(stamp)
            lane.header.frame_id = 'base_link'
            lane.lateral_error_m = 2.0
            lane.heading_error_rad = 0.3
            if active['lane'] == 'stale': lane.header.stamp.sec -= 10
            if active['lane'] == 'nan': lane.lateral_error_m = float('nan')
            lane_pub.publish(lane)

    timer = node.create_timer(0.05, publish)
    stack = Stack(['ros2', 'launch', str(fixture_launch), 'params_file:=' + str(fixture_config)],
                  'smoke_integration')
    try:
        wait_for(node, lambda: 'vehicle_state' in values and values['vehicle_state'].localization_valid,
                 'GNSS relay and covariance/pose adapter')
        assert not values['vehicle_state'].yaw_rate_valid
        wait_for(node, lambda: 'auto_cmd' in values and values['auto_cmd'].valid and
                 values['auto_cmd'].target_speed_mps == 1.0 and values['auto_cmd'].target_steer_rad > 0.0,
                 'global Path -> mission -> plan -> Adaptive Pure Pursuit')
        mode_pub.publish(UInt8(data=ControlStatus.MANUAL))
        wait_for(node, lambda: 'final_cmd' in values and values['final_cmd'].valid and
                 values['final_cmd'].target_speed_mps == 0.4 and
                 values['final_cmd'].target_steer_rad == -0.15, 'MANUAL selects only manual')
        # Selection retains the original command time (not a fresh periodic timestamp).
        active['manual'] = False
        old_stamp = copy.deepcopy(values['selected_cmd'].header.stamp)
        wait_for(node, lambda: not values['final_cmd'].valid and values['final_cmd'].target_brake,
                 'manual source timeout stops despite healthy AUTO')
        assert old_stamp.sec > 0
        active['manual'] = True
        mode_pub.publish(UInt8(data=ControlStatus.AUTO))
        wait_for(node, lambda: values['final_cmd'].valid and values['final_cmd'].target_speed_mps == 1.0,
                 'AUTO selects only auto')
        wait_for(node, lambda: 'guidance_status' in values and values['guidance_status'].valid,
                 'GPS_ONLY default publishes valid GuidanceStatus')
        for lane_input in ['valid', 'invalid', 'stale', 'nan', 'absent']:
            active['lane'] = lane_input
            deadline = time.monotonic() + 0.7
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                status = values['guidance_status']
                assert status.requested_mode == status.GPS_ONLY == status.actual_mode
                assert status.valid and not status.degraded and status.active_source == 'GLOBAL_PATH'
                assert [(p.pose.position.x, p.pose.position.y) for p in values['reference_path'].poses] == [(0., 1.), (10., 1.), (20., 1.)]
            print('PASS: GPS_ONLY ignores LaneState ' + lane_input, flush=True)

        for request, reason in [(GuidanceStatus.LANE_ONLY, 'LANE_ONLY_NOT_IMPLEMENTED'),
                                (GuidanceStatus.HYBRID, 'HYBRID_NOT_IMPLEMENTED'),
                                (255, 'INVALID_GUIDANCE_MODE')]:
            guidance_pub.publish(UInt8(data=request))
            wait_for(node, lambda: values['guidance_status'].requested_mode == request and
                     values['guidance_status'].reason == reason and
                     values['guidance_status'].actual_mode == GuidanceStatus.NONE and
                     not values['guidance_status'].valid and not values['reference_path'].poses and
                     not values['motion_plan'].valid and values['motion_plan'].stop_request and
                     not values['auto_cmd'].valid and values['auto_cmd'].target_brake and
                     not values['final_cmd'].valid and values['final_cmd'].target_brake,
                     'unsupported/invalid guidance closes AUTO: ' + reason)
            assert values['status'].mode == ControlStatus.AUTO
            assert not values['guidance_status'].degraded
            assert values['guidance_status'].active_source == 'NONE'
            # Even healthy lane input and absent GNSS cannot fake lane/hybrid support.
            active['lane'], active['gnss'] = 'valid', False
            wait_for(node, lambda: not values['vehicle_state'].localization_valid,
                     'unsupported guidance fixture: localization expires')
            assert not values['guidance_status'].valid
            assert values['guidance_status'].reason == reason
            active['gnss'] = True
            wait_for(node, lambda: values['vehicle_state'].localization_valid, 'restore fixture GNSS')
            mode_pub.publish(UInt8(data=ControlStatus.MANUAL))
            wait_for(node, lambda: values['status'].mode == ControlStatus.MANUAL and
                     values['final_cmd'].valid and values['final_cmd'].target_speed_mps == 0.4,
                     'Guidance request does not select or block MANUAL')
            assert values['guidance_status'].requested_mode == request
            mode_pub.publish(UInt8(data=ControlStatus.AUTO))
            wait_for(node, lambda: values['status'].mode == ControlStatus.AUTO and
                     not values['final_cmd'].valid and values['final_cmd'].target_brake,
                     'AUTO cannot use manual command with invalid guidance')
            guidance_pub.publish(UInt8(data=GuidanceStatus.GPS_ONLY))
            wait_for(node, lambda: values['guidance_status'].requested_mode == GuidanceStatus.GPS_ONLY and
                     values['guidance_status'].actual_mode == GuidanceStatus.GPS_ONLY and
                     values['guidance_status'].valid and values['final_cmd'].valid and
                     values['final_cmd'].target_speed_mps == 1.0, 'explicit GPS_ONLY recovery')

        for shape in ['empty', 'wrong_frame']:
            active['path'] = shape
            wait_for(node, lambda: values['guidance_status'].reason == 'GLOBAL_PATH_UNAVAILABLE' and
                     not values['guidance_status'].valid and not values['reference_path'].poses and
                     not values['final_cmd'].valid and values['final_cmd'].target_brake,
                     'invalid global path stops AUTO: ' + shape)
            active['path'] = 'valid'
            wait_for(node, lambda: values['guidance_status'].valid and values['final_cmd'].valid,
                     'restore valid global path')
        for timestamp in ['stale', 'future', 'zero']:
            active['gnss_stamp'] = timestamp
            wait_for(node, lambda: values['guidance_status'].reason == 'LOCALIZATION_UNAVAILABLE' and
                     not values['guidance_status'].valid and not values['reference_path'].poses and
                     not values['final_cmd'].valid and values['final_cmd'].target_brake,
                     'fresh callbacks cannot hide bad source timestamp: ' + timestamp)
            active['gnss_stamp'] = 'fresh'
            wait_for(node, lambda: values['guidance_status'].valid and values['final_cmd'].valid,
                     'restore fresh GNSS timestamp')
        # No GUI process or heartbeat is needed: node-side freshness still gates AUTO.
        active['gnss'] = False
        wait_for(node, lambda: not values['guidance_status'].valid and
                 not values['final_cmd'].valid and values['final_cmd'].target_brake,
                 'no GUI requests: sensor timeout still stops AUTO')
        active['gnss'] = True
        wait_for(node, lambda: values['guidance_status'].valid and values['final_cmd'].valid,
                 'restore fixture after timeout')
        for mode in [ControlStatus.AUTO, ControlStatus.MANUAL]:
            mode_pub.publish(UInt8(data=mode))
            estop_pub.publish(Bool(data=True))
            wait_for(node, lambda: values['final_cmd'].reason == 'E_STOP' and
                     values['final_cmd'].target_brake and values['final_cmd'].target_speed_mps == 0.0,
                     'E-STOP overrides mode ' + str(mode))
            estop_pub.publish(Bool(data=False))
            wait_for(node, lambda: values['final_cmd'].valid, 'explicit E-STOP reset')
        manual.target_steer_rad = 0.8
        wait_for(node, lambda: values['final_cmd'].reason == 'COMMAND_OUT_OF_RANGE', 'steering range stop')
        manual.target_steer_rad = float('nan')
        wait_for(node, lambda: values['final_cmd'].reason == 'INVALID_COMMAND_VALUES', 'NaN command stop')
        manual.target_steer_rad = -0.15
        active['gnss'] = False
        wait_for(node, lambda: values['final_cmd'].reason == 'INVALID_OR_STALE_LOCALIZATION',
                 'GNSS timeout applies in MANUAL')
        active['gnss'] = True
        wait_for(node, lambda: values['final_cmd'].valid, 'fresh localization recovery')
        object_state.obstacle_detected = True
        object_state.nearest_obstacle_distance_m = 0.5
        wait_for(node, lambda: values['final_cmd'].reason == 'OBSTACLE_EMERGENCY_OR_UNSET_DISTANCE',
                 'LiDAR observation emergency distance')
        object_state.obstacle_detected = False
        active['object'] = False
        wait_for(node, lambda: values['final_cmd'].reason == 'OBSTACLE_STATE_UNAVAILABLE',
                 'unknown/stale obstacle state blocks motion')
        active['object'] = True
        mode_pub.publish(UInt8(data=255))
        wait_for(node, lambda: values['status'].mode == ControlStatus.IDLE and values['final_cmd'].valid
                 and values['final_cmd'].target_brake, 'invalid mode request returns to IDLE')
    finally:
        stack.close()
        node.destroy_timer(timer)
        node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try:
        default_stack()
        startup_rejections()
        integrated_fixtures()
        print('ALL SMOKE CHECKS PASSED', flush=True)
    finally:
        rclpy.shutdown()
