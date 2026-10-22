#!/usr/bin/env python3
"""Actual-RPM float32 packet, current steering scale and stop hold.
Vehicle Interface -> existing trans encoders; no UDP sender or hardware starts."""
import math
import struct
import sys

import rclpy
from ament_index_python.packages import get_package_prefix
from std_msgs.msg import Float32MultiArray
from control_manager.msg import VehicleCommand
from smoke_test import ROOT, Stack, wait_for
from state_split_smoke_test import settle

# Read the existing pure conversion modules without writing external bytecode.
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT.parent / 'trans'))
from trans import brake, motor, steer


def run():
    node = rclpy.create_node('actuator_raw_fixture')
    received = {}
    for topic in ('/control/actuator_raw', '/test/actuator_raw'):
        node.create_subscription(Float32MultiArray, topic,
                                 lambda msg, key=topic: received.__setitem__(key, list(msg.data)), 10)
    pub = node.create_publisher(VehicleCommand, '/control/selected_cmd', 10)
    active = {'steer': -math.radians(12), 'speed': 10/3.6, 'brake': False, 'valid': True}

    def publish():
        cmd = VehicleCommand(target_steer_rad=active['steer'], target_speed_mps=active['speed'],
                             target_brake=active['brake'], valid=active['valid'], reason='RAW_FIXTURE')
        cmd.header.frame_id = 'base_link'; cmd.header.stamp = node.get_clock().now().to_msg()
        pub.publish(cmd)

    timer = node.create_timer(.05, publish)
    stack = None

    def launch(*params):
        nonlocal stack
        if stack is not None:
            stack.close()
            wait_for(node, lambda: ('vehicle_interface', '/control') not in
                     node.get_node_names_and_namespaces(), 'previous interface stopped')
        received.clear()
        args = [get_package_prefix('control_manager')+'/lib/control_manager/vehicle_interface_node',
                '--ros-args', '-r', '__ns:=/control', '--params-file', str(ROOT/'config/vehicle.yaml'),
                '--params-file', str(ROOT/'config/pipeline.yaml'),
                '-p', 'dry_run:=false', '-p', 'operating_limits_verified:=true']
        for param in params: args += ['-p', param]
        stack = Stack(args, 'actuator_raw_interface')

    def expect(values, label, topic='/control/actuator_raw'):
        wait_for(node, lambda: topic in received and len(received[topic]) == 3 and
                 all(abs(actual-want) < 1e-6 for actual,want in zip(received[topic], values)), label)
        raw = received[topic]
        # Same encoders and packet layout as the existing trans node; no socket.
        return struct.unpack('<ffB', struct.pack('<ffB', steer.to_deg(raw[0]),
                             motor.to_speed_raw(raw[1]), brake.to_misc(raw[2])))

    def packet_matches(packet, angle, rpm, misc):
        assert math.isclose(packet[0], angle, abs_tol=2e-5), packet
        assert math.isclose(packet[1], rpm, abs_tol=2e-5), packet
        assert packet[2] == misc, packet

    try:
        launch('vehicle.wheel_diameter_m:=0.5')  # Fixed fixture; production diameter is editable.
        target_rpm = (10/3.6) * 60 / (math.pi*.50)
        speed_raw = target_rpm / 891.387
        steering_raw = 12/17.01
        packet_matches(expect([steering_raw, speed_raw, 0],
                              'right 12 degrees / 10 km/h normalized output'), -12, -target_rpm, 0)
        assert {i.node_name for i in node.get_publishers_info_by_topic('/control/actuator_raw')} == {
            'vehicle_interface'}
        active.update(steer=math.radians(12), speed=-10/3.6)
        packet_matches(expect([-steering_raw, -speed_raw, 0],
                              'left steering and reverse match existing trans'), 12, target_rpm, 0)
        active.update(steer=0., speed=0., brake=True)
        packet_matches(expect([-steering_raw, 0, 1],
                              'brake holds prior left steering and misc bit 7'), 12, 0, 128)
        active.update(steer=-math.radians(12), speed=10/3.6, brake=False, valid=True)
        expect([steering_raw, speed_raw, 0], 'brake release returns to normal command')
        active.update(valid=False)
        expect([steering_raw, 0, 1], 'invalid selected command holds prior right steering')
        active.update(valid=True)
        expect([steering_raw, speed_raw, 0], 'normal command recovers')
        active.update(steer=math.radians(31))
        expect([steering_raw, 0, 1], 'out-of-range steering cannot replace prior valid steering')
        active.update(steer=-math.radians(12))
        launch('vehicle.wheel_diameter_m:=1.0')
        packet_matches(expect([steering_raw, speed_raw/2, 0],
                              'wheel diameter change updates RPM conversion'), -12, -target_rpm/2, 0)
        launch('topics.actuator_raw:=/test/actuator_raw', 'vehicle.wheel_diameter_m:=0.5')
        expect([steering_raw, speed_raw, 0], 'actuator_raw topic can be configured', '/test/actuator_raw')
        assert not node.get_publishers_info_by_topic('/control/actuator_raw')
        launch('dry_run:=true')
        settle(node, .7)
        assert stack.process.poll() is None
        assert not node.get_publishers_info_by_topic('/control/actuator_raw')
        assert not node.get_publishers_info_by_topic('/test/actuator_raw')
        print('PASS: dry_run suppresses actuator_raw publication', flush=True)
        print('ALL ACTUATOR RAW SMOKE CHECKS PASSED', flush=True)
    finally:
        if stack is not None: stack.close()
        node.destroy_timer(timer); node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
