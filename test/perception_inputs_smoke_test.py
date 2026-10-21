#!/usr/bin/env python3
"""Result ingress integration. Temporary --rm container, network=none, no devices/trans."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import String, UInt8
import yaml
from control_manager.msg import MotionState, ObjectState, TrafficLight
from encoder_motor.msg import MotorFeedback, SteeringFeedback

assert os.environ.get('CONTROL_PERCEPTION_ISOLATED_TEST') == '1'
assert os.environ.get('ROS_DOMAIN_ID') == '179'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(os.environ['CONTROL_PERCEPTION_TEST_LOG'])
OUT.mkdir(parents=True, exist_ok=True)
PREFIX = Path(get_package_prefix('control_manager')) / 'lib/control_manager'
rclpy.init()
node = rclpy.create_node('perception_input_fixture')


def wait(predicate, label, timeout=8):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.03)
        if predicate():
            print('PASS:', label, flush=True)
            return
    raise AssertionError(label)


def owns(topic, name, publisher=False):
    infos = (node.get_publishers_info_by_topic(topic) if publisher
             else node.get_subscriptions_info_by_topic(topic))
    return any(info.node_namespace == '/control' and info.node_name == name for info in infos)


class Process:
    def __init__(self, label, executable, mode='test', overrides=None):
        self.file = (OUT / (label + '.log')).open('w')
        self.command = [str(PREFIX / executable), '--ros-args', '-r', '__ns:=/control',
                        '--params-file', str(ROOT / 'config/pipeline.yaml'),
                        '--params-file', str(ROOT / 'config/modes' / (mode + '.yaml'))]
        if overrides:
            overlay = OUT / (label + '.yaml')
            name = executable[:-5] if executable.endswith('_node') else executable
            overlay.write_text(yaml.safe_dump({'/control/' + name: {'ros__parameters': overrides}}))
            self.command += ['--params-file', str(overlay)]
        (OUT / (label + '-command.json')).write_text(json.dumps(self.command, indent=2))
        self.process = subprocess.Popen(self.command, stdout=self.file, stderr=subprocess.STDOUT)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=8)
        self.file.close()


def stopped():
    wait(lambda: not any(ns == '/control' for _, ns in node.get_node_names_and_namespaces()),
         'fixture nodes stopped')


try:
    assert 'control_manager/msg/VehicleFeedback' not in subprocess.check_output(
        ['ros2', 'interface', 'list'], text=True, timeout=8)
    print('PASS: removed VehicleFeedback is absent from the installed interface index', flush=True)
    for mode in ('test', 'brake_inspection', 'preliminary_lap', 'final_lap'):
        process = Process(mode, 'sensor_interface_node', mode)
        objects = Process(mode + '-objects', 'object_manager_node', mode)
        try:
            wait(lambda: ('sensor_interface', '/control') in node.get_node_names_and_namespaces()
                 and ('object_manager', '/control') in node.get_node_names_and_namespaces(), mode + ' starts')
            expected = mode in ('preliminary_lap', 'final_lap')
            wait(lambda: owns('/sensors/detectTraffic', 'sensor_interface') == expected
                 and owns('/control/sensors/traffic', 'sensor_interface', True) == expected,
                 mode + ' traffic activation')
            for name in ('Cone', 'Obstacle'):
                assert not owns('/sensors/detect' + name, 'sensor_interface')
                assert not owns('/control/sensors/' + name.lower(), 'sensor_interface', True)
            for topic in ('/lidar/points', '/imu/data', '/camera_dep/depth/image_raw', '/camera/image_raw'):
                assert not owns(topic, 'sensor_interface')
            assert owns('/gnss/odom', 'sensor_interface')
            assert owns('/motor_feedback', 'sensor_interface')
            assert owns('/steering_feedback', 'sensor_interface')
            assert not owns('/control/sensors/lidar', 'sensor_interface', True)
            assert not owns('/control/object_state', 'object_manager', True)
            assert not node.get_publishers_info_by_topic('/control/vehicle_feedback')
            assert not node.get_subscriptions_info_by_topic('/control/vehicle_feedback')
            assert process.process.poll() is None and objects.process.poll() is None
            print('PASS: disabled result/raw inputs absent; GNSS/encoder unchanged', flush=True)
        finally:
            process.close()
            objects.close()
            stopped()

    # String/UInt8 below are TEST-ONLY arbitrary payloads, not proposed Cone/Obstacle schemas.
    options = {'cone_enabled': True, 'obstacle_enabled': True, 'traffic_enabled': True,
               'perception_types.cone': 'std_msgs/msg/String',
               'perception_types.obstacle': 'std_msgs/msg/UInt8'}
    values, subscriptions, publishers = {}, [], {}
    for name, cls, external in [('cone', String, '/sensors/detectCone'),
                                ('obstacle', UInt8, '/sensors/detectObstacle'),
                                ('traffic', TrafficLight, '/sensors/detectTraffic'),
                                ('gnss_odom', Odometry, '/gnss/odom'),
                                ('motor_feedback', MotorFeedback, '/motor_feedback'),
                                ('steering_feedback', SteeringFeedback, '/steering_feedback')]:
        publishers[name] = node.create_publisher(cls, external, qos_profile_sensor_data if name == 'traffic' else 10)
        subscriptions.append(node.create_subscription(cls, '/control/sensors/' + name,
            lambda msg, key=name: values.__setitem__(key, serialize_message(msg)), qos_profile_sensor_data))
    objects_seen = []
    subscriptions.append(node.create_subscription(ObjectState, '/control/object_state', objects_seen.append, 10))
    process = Process('relay', 'sensor_interface_node', overrides=options)
    objects = Process('aggregate-placeholder', 'object_manager_node', overrides={**options, 'object_state_enabled': True})
    payloads = {'cone': String(data='TEST-only arbitrary cone bytes'), 'obstacle': UInt8(data=37),
                'traffic': TrafficLight(traffic_id=4, status='Unknown', valid=True, color_confidence=0.0),
                'gnss_odom': Odometry(),
                'motor_feedback': MotorFeedback(motor_rpm=-1500, motor_rpm_mcu_ms=123, motor_rpm_valid=True),
                'steering_feedback': SteeringFeedback(steer_angle=12.25, steer_angle_mcu_ms=456, steer_angle_valid=False)}
    payloads['traffic'].timestamp.sec = 123
    payloads['traffic'].timestamp.nanosec = 450000000
    payloads['gnss_odom'].header.frame_id = 'map'
    payloads['gnss_odom'].header.stamp.sec = 100
    payloads['gnss_odom'].pose.pose.position.x = 1.25

    def publish():
        for name, publisher in publishers.items():
            publisher.publish(payloads[name])

    timer = node.create_timer(0.05, publish)
    try:
        wait(lambda: all(values.get(k) == serialize_message(v) for k, v in payloads.items()),
             'result, GNSS and raw encoder payloads relayed byte-for-byte')
        payloads['traffic'].valid = False
        payloads['traffic'].status = 'R'
        payloads['traffic'].color_confidence = 0.93
        wait(lambda: values.get('traffic') == serialize_message(payloads['traffic']),
             'traffic ID, original timestamp, confidence and valid=false preserved')
        wait(lambda: bool(objects_seen), 'ObjectState placeholder is explicit')
        assert all(not s.valid and not s.obstacle_valid and not s.traffic_light_valid
                   and s.traffic_light_state == ObjectState.UNKNOWN for s in objects_seen)
        for name in ('cone', 'obstacle', 'traffic'):
            assert owns('/control/sensors/' + name, 'object_manager')
        for name in ('rgb', 'lidar', 'depth'):
            assert not owns('/control/sensors/' + name, 'object_manager')
        print('PASS: receipt never fabricates a valid control observation', flush=True)
    finally:
        node.destroy_timer(timer)
        process.close()
        objects.close()
        stopped()

    motion = []
    motion_sub = node.create_subscription(MotionState, '/control/motion_state', motion.append, 10)
    process = Process('wheel-placeholder', 'global_state_estimator',
                      overrides={'pose_mode': 'NONE', 'motion_mode': 'WHEEL'})
    try:
        wait(lambda: motion and motion[-1].reason == 'WHEEL_FEEDBACK_CONTRACT_NOT_IMPLEMENTED',
             'WHEEL remains explicitly unimplemented without a feedback topic')
        assert not motion[-1].speed_valid and not motion[-1].yaw_rate_valid
        assert not node.get_publishers_info_by_topic('/control/vehicle_feedback')
        assert not node.get_subscriptions_info_by_topic('/control/vehicle_feedback')
        assert not owns('/control/sensors/gnss_odom', 'global_state_estimator')
    finally:
        process.close()
        node.destroy_subscription(motion_sub)
        stopped()

    for label, executable, overrides, reason in [
        ('empty-cone-type', 'sensor_interface_node', {'cone_enabled': True}, 'requires nonempty perception_types.cone'),
        ('empty-obstacle-type', 'sensor_interface_node', {'obstacle_enabled': True}, 'requires nonempty perception_types.obstacle'),
        ('loop', 'sensor_interface_node', {'traffic_enabled': True, 'topics.sensor_traffic': '/sensors/detectTraffic'}, 'same topic'),
        ('no-result-input', 'object_manager_node', {'object_state_enabled': True}, 'requires at least one'),
    ]:
        process = Process(label, executable, overrides=overrides)
        try:
            process.process.wait(timeout=8)
            assert process.process.returncode != 0
        finally:
            process.close()
        assert reason in (OUT / (label + '.log')).read_text()
        print('PASS:', label, 'rejected', flush=True)
    print('PASS: mode activation, result relay, raw LiDAR removal and config rejection', flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
