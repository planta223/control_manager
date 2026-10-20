#!/usr/bin/env python3
"""Raw encoder ingress contracts; isolated DDS fixtures, no hardware or trans."""

import os
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from encoder_motor.msg import MotorFeedback, SteeringFeedback
from control_manager.msg import ControlStatus
from smoke_test import ROOT, ENV, Stack, wait_for
from operation_smoke_test import command, owns


def pump(node, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.005)


def relay_values(source_qos, label):
    node = rclpy.create_node('feedback_relay_' + label)
    messages = {'motor': [], 'steering': []}
    sources = {
        'motor': node.create_publisher(MotorFeedback, '/motor_feedback', source_qos),
        'steering': node.create_publisher(SteeringFeedback, '/steering_feedback', source_qos),
    }
    subscriptions = [
        node.create_subscription(MotorFeedback, '/control/sensors/motor_feedback',
                                 messages['motor'].append, qos_profile_sensor_data),
        node.create_subscription(SteeringFeedback, '/control/sensors/steering_feedback',
                                 messages['steering'].append, qos_profile_sensor_data),
    ]
    stack = Stack(command('sensor_interface_node', 'rate_hz:=1.0'), 'feedback_relay_' + label)
    try:
        wait_for(node, lambda: all(owns(node, topic, 'sensor_interface') for topic in
                                  ('/motor_feedback', '/steering_feedback')),
                 'both feedback input subscriptions discovered')
        wait_for(node, lambda: all(owns(node, topic, 'sensor_interface', True) for topic in
                                  ('/control/sensors/motor_feedback',
                                   '/control/sensors/steering_feedback')),
                 'both feedback output publishers discovered')
        assert not owns(node, '/control/vehicle_feedback', 'sensor_interface', True)
        assert not owns(node, '/motor_feedback', 'sensor_interface', True)
        assert not owns(node, '/steering_feedback', 'sensor_interface', True)
        for infos in (node.get_publishers_info_by_topic('/control/sensors/motor_feedback'),
                      node.get_subscriptions_info_by_topic('/motor_feedback')):
            endpoint = next(info for info in infos if info.node_name == 'sensor_interface')
            assert endpoint.qos_profile.reliability == qos_profile_sensor_data.reliability

        # Tick zero, wrap, reset, false validity and values outside an operating
        # envelope are observations, not an excuse to rewrite or discard them.
        cases = [(0, 0, False), (-1500, 0, True), (1500, 0xFFFFFFFF, True),
                 (-32000, 10, False), (32000, 5, True)]
        for index, (rpm, tick, valid) in enumerate(cases):
            motor = MotorFeedback(motor_rpm=rpm, motor_rpm_mcu_ms=tick, motor_rpm_valid=valid)
            steering = SteeringFeedback(steer_angle=(-24.5, 0.0, 12.25, 24.0, -1.0)[index],
                                        steer_angle_mcu_ms=tick, steer_angle_valid=valid)
            def matching():
                sources['motor'].publish(motor)
                sources['steering'].publish(steering)
                return motor in messages['motor'] and steering in messages['steering']
            wait_for(node, matching, 'all three fields preserved: ' + str(index))

        # Even a 1 Hz node timer configuration must pass a faster incoming stream.
        # Repeated samples remain unchanged rather than receiving synthetic stamps.
        motor = MotorFeedback(motor_rpm=-1000, motor_rpm_mcu_ms=123, motor_rpm_valid=True)
        steering = SteeringFeedback(steer_angle=3.5, steer_angle_mcu_ms=456, steer_angle_valid=False)
        pump(node, 0.1)
        before = {key: len(value) for key, value in messages.items()}
        for _ in range(30):
            sources['motor'].publish(motor)
            sources['steering'].publish(steering)
            pump(node, 0.02)
        pump(node, 0.05)
        for key, sample in (('motor', motor), ('steering', steering)):
            stream = messages[key][before[key]:]
            assert len(stream) >= 15, (key, len(stream))
            assert all(value == sample for value in stream), key

        pump(node, 0.1)
        stopped_counts = {key: len(value) for key, value in messages.items()}
        pump(node, 0.2)
        assert stopped_counts == {key: len(value) for key, value in messages.items()}
        assert stack.process.poll() is None
    finally:
        stack.close()
        node.destroy_node()
    print('PASS: exact feedback relay, false/zero/wrap/reset, no rate limit or cache replay: '
          + label, flush=True)


def rejection_tests():
    cases = [
        ('topics.external_motor_feedback:=/control/sensors/motor_feedback',),
        ('topics.external_steering_feedback:=/control/sensors/steering_feedback',),
        ('topics.external_motor_feedback:=/test/motor',),
        ('topics.external_steering_feedback:=/test/steering',),
        ("topics.external_motor_feedback:=''",),
        ("topics.sensor_motor_feedback:=''",),
        ("topics.external_steering_feedback:=''",),
        ("topics.sensor_steering_feedback:=''",),
    ]
    for index, overrides in enumerate(cases):
        args = command('sensor_interface_node', *overrides)
        if index == 2:
            args += ['-r', '/test/motor:=/control/sensors/motor_feedback']
        if index == 3:
            args += ['-r', '/test/steering:=/control/sensors/steering_feedback']
        result = subprocess.run(args, env=ENV, capture_output=True, text=True, timeout=8)
        output = result.stdout + result.stderr
        assert result.returncode != 0 and ('same topic' in output or 'require nonempty' in output), output
    print('PASS: empty paths and feedback loops, including remaps, are rejected', flush=True)


def launch_defaults():
    map_file = os.environ.get('FEEDBACK_TEST_MAP')
    if not map_file:
        return
    node = rclpy.create_node('feedback_default_launch_probe')
    statuses = []
    sub = node.create_subscription(ControlStatus, '/control/status', statuses.append, 10)
    stack = Stack(['ros2', 'launch', 'control_manager', 'control_manager.xml',
                   'map_file:=' + map_file], 'feedback_default_launch')
    try:
        wait_for(node, lambda: len([name for name, ns in node.get_node_names_and_namespaces()
                                  if ns == '/control']) == 11,
                 'default launch starts all eleven control nodes')
        wait_for(node, lambda: statuses and statuses[-1].paused and
                 statuses[-1].mode == ControlStatus.AUTO, 'default AUTO remains paused')
        for topic in ('/motor_feedback', '/steering_feedback'):
            assert owns(node, topic, 'sensor_interface')
        assert not owns(node, '/control/vehicle_feedback', 'sensor_interface', True)
        # No real command publisher is enabled by adding observation inputs.
        assert not node.get_publishers_info_by_topic('/control/actuator_raw')
    finally:
        stack.close()
        node.destroy_node()
    print('PASS: default eleven-node launch enables feedback relays, preserves AUTO pause/dry_run', flush=True)


def run():
    rclpy.init()
    try:
        relay_values(QoSProfile(depth=10), 'reliable_source')
        relay_values(qos_profile_sensor_data, 'best_effort_source')
        rejection_tests()
        launch_defaults()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    run()
