#!/usr/bin/env python3
"""Real keyboard/ROS pipeline for motor-zero versus braking. No trans or hardware."""
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from control_manager.msg import ControlStatus, PauseRequest, VehicleCommand
import yaml

assert os.environ.get('ROS_DOMAIN_ID') == '189'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('CONTROL_CONSOLE_ISOLATED_TEST') == '1'
ROOT = Path(__file__).resolve().parents[1]
OUT = Path('/evidence/motor-zero')
OUT.mkdir(parents=True, exist_ok=True)


def run():
    node = rclpy.create_node('motor_zero_fixture')
    state = dict(x=0., y=0., speed=5., send=True, variance=.01)
    values, raw_trace, checks = {}, [], []
    odom_pub = node.create_publisher(Odometry, '/gnss/odom', 10)
    path_pub = node.create_publisher(PathMsg, '/control/global_path', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    requests = node.create_publisher(PauseRequest, '/control/pause_request', 10)

    def raw(message):
        values['raw'] = list(message.data)
        raw_trace.append(dict(received_ros_ns=node.get_clock().now().nanoseconds, data=list(message.data)))

    node.create_subscription(Float32MultiArray, '/control/actuator_raw', raw, 10)
    node.create_subscription(ControlStatus, '/control/status', lambda m: values.update(status=m), 10)
    node.create_subscription(VehicleCommand, '/control/selected_cmd', lambda m: values.update(selected=m), 10)
    node.create_subscription(VehicleCommand, '/control/auto_cmd', lambda m: values.update(auto=m), 10)

    def publish():
        if not state['send']: return
        stamp = node.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.frame_id, msg.child_frame_id = 'map', 'base_link'
        msg.header.stamp = stamp
        msg.pose.pose.position.x, msg.pose.pose.position.y = state['x'], state['y']
        msg.pose.pose.orientation.w = 1.
        msg.pose.covariance[0] = msg.pose.covariance[7] = state['variance']
        msg.pose.covariance[35] = .01
        msg.twist.twist.linear.x = state['speed']
        msg.twist.covariance[0] = .01
        odom_pub.publish(msg)
        path = PathMsg()
        path.header.frame_id, path.header.stamp = 'map', stamp
        for x in (0., 50., 100.):
            point = PoseStamped()
            point.pose.position.x, point.pose.orientation.w = x, 1.
            path.poses.append(point)
        path_pub.publish(path)

    timer = node.create_timer(.05, publish)
    for label, variable_speed in [('constant', False), ('variable', True)]:
        values.clear(); raw_trace.clear()
        state.update(x=0., y=0., speed=5., send=True, variance=.01)
        operation = yaml.safe_load((ROOT/'config/modes/test.yaml').read_text())
        operation['/control/global_path_provider']['ros__parameters']['map_file'] = ''
        operation['/control/motion_control']['ros__parameters']['route_progress_enabled'] = False
        operation['/control/vehicle_interface']['ros__parameters']['dry_run'] = False
        operation_file = OUT/(label+'-operation.yaml')
        operation_file.write_text(yaml.safe_dump(operation))
        command = [sys.executable, str(ROOT/'scripts/control_console'), '--mode', 'test',
                   '--operation-file', str(operation_file), '--log-root', str(OUT/label),
                   '--cruise-speed-kph', '10', '--max-accel-kphps', '2']
        if variable_speed: command += ['--variable-speed', '--max-decel-kphps', '5']
        master, slave = pty.openpty()
        process = subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
        os.close(slave)
        terminal = bytearray()

        def spin():
            rclpy.spin_once(node, timeout_sec=.01)
            if select.select([master], [], [], 0)[0]:
                try: terminal.extend(os.read(master, 65536))
                except OSError: pass

        def settle(duration=.2):
            end = time.monotonic()+duration
            while time.monotonic() < end: spin()

        def wait(predicate, description, timeout=15.):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                spin()
                if predicate():
                    print('PASS:', label, description, flush=True)
                    checks.append(dict(case=label, check=description, passed=True))
                    return
                assert process.poll() is None, terminal.decode(errors='replace')[-6000:]
            raise AssertionError(description+'\n'+terminal.decode(errors='replace')[-6000:])

        def key(value): os.write(master, value.encode())

        def paused():
            return ('status' in values and values['status'].paused and
                    not values['status'].motor_zero and values.get('raw', [])[1:] == [0., 1.])

        def motor_zero():
            return ('status' in values and values['status'].motor_zero and
                    not values['status'].paused and values.get('raw', [])[1:] == [0., 0.])

        def driving():
            return ('status' in values and not values['status'].paused and
                    not values['status'].motor_zero and 'raw' in values and
                    values['raw'][1] > 0 and values['raw'][2] == 0.)

        try:
            wait(lambda: paused() and '일시정지 상태로 준비됐습니다.'.encode() in terminal,
                 'startup remains motor 0/brake ON')
            assert 'trans_node' not in [name for name, _ in node.get_node_names_and_namespaces()]
            key('r'); wait(driving, 'R resumes powered driving')
            seq = values['status'].resume_sequence
            key('0'); wait(motor_zero, '0 selects motor 0/brake OFF')
            assert values['status'].resume_sequence == seq
            state.update(x=10., y=.5)
            wait(lambda: motor_zero() and abs(values['raw'][0]) > .001,
                 'steering remains active with motor zero')
            first = values['raw'][0]
            state.update(x=20., y=-.5)
            wait(lambda: motor_zero() and first*values['raw'][0] < 0.,
                 'steering changes with path error while motor stays zero')
            key('c'); settle()
            assert motor_zero() and values['status'].resume_sequence == seq
            checks.append(dict(case=label, check='repeated C does not resume motor or advance sequence', passed=True))
            key('1'); wait(driving, '1 resumes from motor zero')
            assert values['status'].resume_sequence == seq+1
            key('C'); wait(motor_zero, 'C selects motor zero')
            # A delayed old motor-zero request must not release a newer brake request.
            old = PauseRequest(pause=False, motor_zero=True)
            old.header.stamp = node.get_clock().now().to_msg()
            old.header.frame_id = 'base_link'
            settle(.04)
            held = values['raw'][0]
            key('p'); wait(paused, 'P keeps existing motor 0/brake ON')
            assert abs(values['raw'][0]-held) < 1e-5
            requests.publish(old); settle()
            assert paused()
            contradictory = PauseRequest(pause=True, motor_zero=True)
            contradictory.header.stamp = node.get_clock().now().to_msg()
            contradictory.header.frame_id = 'base_link'
            requests.publish(contradictory); settle()
            assert paused()
            checks.append(dict(case=label, check='stale and contradictory requests cannot release braking', passed=True))
            key('r'); wait(driving, 'R still releases the original pause')
            key('c'); wait(motor_zero, 'motor zero can be selected again')
            state['send'] = False
            wait(lambda: values.get('raw', [])[1:] == [0., 1.] and values['auto'].target_brake,
                 'stale GNSS brakes even with motor-zero request latched')
            state['send'] = True
            wait(motor_zero, 'fresh input recovers motor-zero without restoring drive')
            state['variance'] = 10.
            wait(lambda: values.get('raw', [])[1:] == [0., 1.] and values['auto'].target_brake,
                 'invalid localization still applies brakes')
            state['variance'] = .01
            wait(motor_zero, 'valid localization restores only motor-zero mode')
            if not variable_speed:
                state.update(x=98., y=0.)
                wait(lambda: motor_zero() and values['auto'].valid,
                     'motor-zero steering approaches goal')
                settle()
                state['x'] = 100.
                wait(lambda: values.get('raw', [])[1:] == [0., 1.] and values['auto'].target_brake,
                     'goal brake has priority over motor zero')
                key('c'); settle()
                assert values['raw'][1:] == [0., 1.]
                key('q')
            else:
                process.send_signal(signal.SIGINT)
            wait(paused, 'exit switches motor-zero to braking')
            settle(.2)
            assert process.poll() is None, 'must wait for measured stop'
            key('c'); settle(.1)
            assert paused(), 'C cannot interrupt exit braking'
            state['speed'] = 0.
            wait(lambda: process.poll() is not None, 'exit requires measured stop')
            assert process.returncode == 0, terminal.decode(errors='replace')
            journal = next((OUT/label).glob('*/events.jsonl'))
            events = [json.loads(line) for line in journal.read_text().splitlines()]
            assert any(e['event'] == 'acknowledged' and e.get('motor_zero') for e in events)
            assert any(e['event'] == 'exit_ready' and e['verification'] == 'measured_stop' for e in events)
        finally:
            if process.poll() is None:
                process.kill(); process.wait()
                for metadata in (OUT/label).glob('*/metadata.json'):
                    launch_pid = json.loads(metadata.read_text()).get('launch_pid')
                    if launch_pid:
                        try: os.killpg(launch_pid, signal.SIGKILL)
                        except ProcessLookupError: pass
            (OUT/(label+'-terminal.log')).write_bytes(terminal)
            (OUT/(label+'-command.json')).write_text(json.dumps(command, indent=2))
            (OUT/(label+'-raw.json')).write_text(json.dumps(raw_trace, indent=2))
            os.close(master)
        end = time.monotonic()+8
        while any(ns == '/control' for _, ns in node.get_node_names_and_namespaces()):
            assert time.monotonic() < end, 'control nodes did not exit'
            rclpy.spin_once(node, timeout_sec=.02)
    (OUT/'result.json').write_text(json.dumps(checks, indent=2))
    node.destroy_timer(timer); node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
